use crate::{
    components::{
        FlightAssist, FlightBody, FlightControlChannels, FlightControlInput, FlightEnvironment,
        FlightKinematics, FlightMessageState, LandingGearState, StallState,
    },
    math::{move_towards, shape_axis, smoothstep01},
    telemetry::{FlightAeroState, FlightForces, FlightTelemetry},
};
use bevy::prelude::*;

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControlInputSource {
    #[default]
    None,
    Pitch,
    Roll,
    Yaw,
    Throttle,
    Collective,
    Transition,
}

impl ControlInputSource {
    pub fn sample(self, input: FlightControlInput) -> f32 {
        match self {
            Self::None => 0.0,
            Self::Pitch => input.pitch.clamp(-1.0, 1.0),
            Self::Roll => input.roll.clamp(-1.0, 1.0),
            Self::Yaw => input.yaw.clamp(-1.0, 1.0),
            Self::Throttle => input.throttle.clamp(0.0, 1.0),
            Self::Collective => input.collective.clamp(0.0, 1.0),
            Self::Transition => input.vtol_transition.clamp(0.0, 1.0),
        }
    }

    pub fn write(self, input: &mut FlightControlInput, value: f32) -> bool {
        match self {
            Self::None => return false,
            Self::Pitch => input.pitch = value.clamp(-1.0, 1.0),
            Self::Roll => input.roll = value.clamp(-1.0, 1.0),
            Self::Yaw => input.yaw = value.clamp(-1.0, 1.0),
            Self::Throttle => input.throttle = value.clamp(0.0, 1.0),
            Self::Collective => input.collective = value.clamp(0.0, 1.0),
            Self::Transition => input.vtol_transition = value.clamp(0.0, 1.0),
        }
        true
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrimInputSource {
    #[default]
    Pitch,
    Roll,
    Yaw,
}

impl TrimInputSource {
    pub fn sample(self, input: FlightControlInput) -> f32 {
        match self {
            Self::Pitch => input.pitch_trim,
            Self::Roll => input.roll_trim,
            Self::Yaw => input.yaw_trim,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct ChannelResponse {
    pub rise_per_second: f32,
    pub fall_per_second: f32,
    pub exponent: f32,
}

impl Default for ChannelResponse {
    fn default() -> Self {
        Self {
            rise_per_second: 8.0,
            fall_per_second: 8.0,
            exponent: 1.0,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct ControlChannelBinding {
    pub source: ControlInputSource,
    pub trim: Option<TrimInputSource>,
    pub trim_scale: f32,
    pub scale: f32,
    pub offset: f32,
    pub clamp_min: f32,
    pub clamp_max: f32,
    pub response: ChannelResponse,
}

impl ControlChannelBinding {
    pub fn resolve_target(self, input: FlightControlInput) -> f32 {
        let trim = self
            .trim
            .map(|trim| trim.sample(input) * self.trim_scale)
            .unwrap_or(0.0);
        let desired = shape_axis(self.source.sample(input), self.response.exponent) * self.scale
            + self.offset
            + trim;
        desired.clamp(self.clamp_min, self.clamp_max)
    }

    pub fn resolve(self, input: FlightControlInput, current: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return current;
        }
        let target = self.resolve_target(input);
        let rate = if target >= current {
            self.response.rise_per_second
        } else {
            self.response.fall_per_second
        };
        move_towards(current, target, rate.max(0.0) * dt)
    }
}

impl Default for ControlChannelBinding {
    fn default() -> Self {
        Self {
            source: ControlInputSource::None,
            trim: None,
            trim_scale: 0.0,
            scale: 1.0,
            offset: 0.0,
            clamp_min: -1.0,
            clamp_max: 1.0,
            response: ChannelResponse::default(),
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct VehicleControlMap {
    pub pitch: ControlChannelBinding,
    pub roll: ControlChannelBinding,
    pub yaw: ControlChannelBinding,
    pub forward_thrust: ControlChannelBinding,
    pub vertical_thrust: ControlChannelBinding,
    pub lateral_thrust: ControlChannelBinding,
    pub transition: ControlChannelBinding,
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct ContactGeometry {
    pub contact_offset_below_origin_m: f32,
    pub retractable: bool,
    pub extension_rate_per_second: f32,
    pub ground_effect_height_m: f32,
    pub ground_effect_boost: f32,
}

impl ContactGeometry {
    pub fn effective_offset_m(self, gear_position: f32) -> f32 {
        if self.retractable {
            self.contact_offset_below_origin_m * gear_position.clamp(0.0, 1.0)
        } else {
            self.contact_offset_below_origin_m
        }
    }
}

impl Default for ContactGeometry {
    fn default() -> Self {
        Self {
            contact_offset_below_origin_m: 1.0,
            retractable: true,
            extension_rate_per_second: 1.0,
            ground_effect_height_m: 4.0,
            ground_effect_boost: 0.12,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq)]
#[reflect(Component, Debug)]
pub struct GroundHandling {
    pub enabled: bool,
    pub contact_geometry: ContactGeometry,
    pub longitudinal_damping_per_second: f32,
    pub lateral_damping_per_second: f32,
    pub angular_damping_per_second: f32,
}

impl Default for GroundHandling {
    fn default() -> Self {
        Self {
            enabled: false,
            contact_geometry: ContactGeometry::default(),
            longitudinal_damping_per_second: 1.8,
            lateral_damping_per_second: 1.8,
            angular_damping_per_second: 1.8,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct FixedWingModel {
    pub wing_area_m2: f32,
    pub wingspan_m: f32,
    pub mean_chord_m: f32,
    pub cl0: f32,
    pub lift_curve_slope_per_rad: f32,
    pub max_lift_coefficient: f32,
    pub post_stall_lift_coefficient: f32,
    pub zero_lift_drag_coefficient: f32,
    pub induced_drag_factor: f32,
    pub stall_drag_coefficient: f32,
    pub side_force_slope_per_rad: f32,
    pub roll_rate_damping: f32,
    pub pitch_rate_damping: f32,
    pub yaw_rate_damping: f32,
    pub roll_stability: f32,
    pub pitch_stability: f32,
    pub yaw_stability: f32,
    pub gear_drag_coefficient: f32,
    pub stall_alpha_rad: f32,
    pub recovery_alpha_rad: f32,
    pub stall_response_per_second: f32,
}

impl Default for FixedWingModel {
    fn default() -> Self {
        Self {
            wing_area_m2: 16.2,
            wingspan_m: 10.9,
            mean_chord_m: 1.52,
            cl0: 0.22,
            lift_curve_slope_per_rad: 5.25,
            max_lift_coefficient: 1.45,
            post_stall_lift_coefficient: 0.58,
            zero_lift_drag_coefficient: 0.028,
            induced_drag_factor: 0.055,
            stall_drag_coefficient: 0.72,
            side_force_slope_per_rad: 1.1,
            roll_rate_damping: 0.65,
            pitch_rate_damping: 0.88,
            yaw_rate_damping: 0.48,
            roll_stability: 0.18,
            pitch_stability: 0.95,
            yaw_stability: 0.52,
            gear_drag_coefficient: 0.03,
            stall_alpha_rad: 16.0_f32.to_radians(),
            recovery_alpha_rad: 12.0_f32.to_radians(),
            stall_response_per_second: 2.8,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct RotorcraftModel {
    pub rotor_disc_area_m2: f32,
    pub parasite_drag_coefficient: f32,
    pub side_drag_coefficient: f32,
    pub translational_lift_gain: f32,
    pub translational_lift_full_speed_mps: f32,
    pub angular_damping: Vec3,
}

impl Default for RotorcraftModel {
    fn default() -> Self {
        Self {
            rotor_disc_area_m2: 62.0,
            parasite_drag_coefficient: 0.26,
            side_drag_coefficient: 0.20,
            translational_lift_gain: 0.24,
            translational_lift_full_speed_mps: 22.0,
            angular_damping: Vec3::new(0.45, 0.65, 0.40),
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct HybridVehicleModel {
    pub fixed_wing: FixedWingModel,
    pub rotorcraft: RotorcraftModel,
    pub wingborne_blend_start: f32,
    pub wingborne_blend_end: f32,
}

impl HybridVehicleModel {
    pub fn wingborne_blend(self, transition: f32) -> f32 {
        let width = (self.wingborne_blend_end - self.wingborne_blend_start).max(0.001);
        let normalized = (transition - self.wingborne_blend_start) / width;
        smoothstep01(normalized)
    }
}

impl Default for HybridVehicleModel {
    fn default() -> Self {
        Self {
            fixed_wing: FixedWingModel {
                wing_area_m2: 28.0,
                wingspan_m: 15.6,
                mean_chord_m: 1.94,
                max_lift_coefficient: 1.62,
                zero_lift_drag_coefficient: 0.031,
                stall_alpha_rad: 18.0_f32.to_radians(),
                recovery_alpha_rad: 13.0_f32.to_radians(),
                ..default()
            },
            rotorcraft: RotorcraftModel {
                rotor_disc_area_m2: 92.0,
                translational_lift_gain: 0.18,
                translational_lift_full_speed_mps: 32.0,
                angular_damping: Vec3::new(0.52, 0.70, 0.44),
                ..default()
            },
            wingborne_blend_start: 0.22,
            wingborne_blend_end: 0.72,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct SpacecraftModel {
    pub angular_damping: Vec3,
    pub linear_drag_coefficient: f32,
}

impl Default for SpacecraftModel {
    fn default() -> Self {
        Self {
            angular_damping: Vec3::new(0.55, 0.55, 0.55),
            linear_drag_coefficient: 0.0,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub enum VehicleModelKind {
    FixedWing(FixedWingModel),
    Rotorcraft(RotorcraftModel),
    Hybrid(HybridVehicleModel),
    Spacecraft(SpacecraftModel),
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
#[require(
    Transform,
    GlobalTransform,
    FlightBody,
    FlightAssist,
    FlightAeroState,
    FlightControlChannels,
    FlightControlInput,
    FlightEnvironment,
    FlightForces,
    FlightKinematics,
    FlightMessageState,
    LandingGearState,
    FlightTelemetry,
    StallState
)]
pub struct VehicleModel {
    pub kind: VehicleModelKind,
}

impl VehicleModel {
    pub fn fixed_wing(model: FixedWingModel) -> Self {
        Self {
            kind: VehicleModelKind::FixedWing(model),
        }
    }

    pub fn rotorcraft(model: RotorcraftModel) -> Self {
        Self {
            kind: VehicleModelKind::Rotorcraft(model),
        }
    }

    pub fn hybrid(model: HybridVehicleModel) -> Self {
        Self {
            kind: VehicleModelKind::Hybrid(model),
        }
    }

    pub fn spacecraft(model: SpacecraftModel) -> Self {
        Self {
            kind: VehicleModelKind::Spacecraft(model),
        }
    }

    pub fn stall_response_per_second(self) -> Option<f32> {
        match self.kind {
            VehicleModelKind::FixedWing(model) => Some(model.stall_response_per_second),
            VehicleModelKind::Hybrid(model) => Some(model.fixed_wing.stall_response_per_second),
            VehicleModelKind::Rotorcraft(_) | VehicleModelKind::Spacecraft(_) => None,
        }
    }
}

impl Default for VehicleModel {
    fn default() -> Self {
        Self::fixed_wing(FixedWingModel::default())
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct FixedWingActuators {
    pub thrust_axis_local: Vec3,
    pub max_forward_thrust_newtons: f32,
    pub aileron_authority: f32,
    pub elevator_authority: f32,
    pub rudder_authority: f32,
    pub trim_authority: Vec3,
}

impl Default for FixedWingActuators {
    fn default() -> Self {
        Self {
            thrust_axis_local: Vec3::NEG_Z,
            max_forward_thrust_newtons: 3_200.0,
            aileron_authority: 0.55,
            elevator_authority: 0.72,
            rudder_authority: 0.42,
            trim_authority: Vec3::new(0.10, 0.16, 0.12),
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct RotorcraftActuators {
    pub lift_axis_local: Vec3,
    pub max_lift_newtons: f32,
    pub pitch_torque_authority: f32,
    pub roll_torque_authority: f32,
    pub yaw_torque_authority: f32,
    pub anti_torque_per_vertical_thrust: f32,
    pub trim_authority: Vec3,
}

impl Default for RotorcraftActuators {
    fn default() -> Self {
        Self {
            lift_axis_local: Vec3::Y,
            max_lift_newtons: 17_000.0,
            pitch_torque_authority: 3_200.0,
            roll_torque_authority: 3_000.0,
            yaw_torque_authority: 2_600.0,
            anti_torque_per_vertical_thrust: 1_450.0,
            trim_authority: Vec3::new(0.10, 0.08, 0.12),
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct ThrustRouting {
    pub hover_axis_local: Vec3,
    pub cruise_axis_local: Vec3,
}

impl Default for ThrustRouting {
    fn default() -> Self {
        Self {
            hover_axis_local: Vec3::Y,
            cruise_axis_local: Vec3::NEG_Z,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct HybridActuators {
    pub fixed_wing: FixedWingActuators,
    pub rotorcraft: RotorcraftActuators,
    pub rotor_thrust_routing: ThrustRouting,
}

impl Default for HybridActuators {
    fn default() -> Self {
        Self {
            fixed_wing: FixedWingActuators {
                max_forward_thrust_newtons: 0.0,
                ..default()
            },
            rotorcraft: RotorcraftActuators {
                max_lift_newtons: 31_000.0,
                pitch_torque_authority: 4_400.0,
                roll_torque_authority: 4_100.0,
                yaw_torque_authority: 3_000.0,
                anti_torque_per_vertical_thrust: 1_200.0,
                ..default()
            },
            rotor_thrust_routing: ThrustRouting::default(),
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct SpacecraftActuators {
    pub forward_axis_local: Vec3,
    pub lateral_axis_local: Vec3,
    pub vertical_axis_local: Vec3,
    pub max_forward_thrust_newtons: f32,
    pub max_lateral_thrust_newtons: f32,
    pub max_vertical_thrust_newtons: f32,
    pub pitch_torque_authority: f32,
    pub roll_torque_authority: f32,
    pub yaw_torque_authority: f32,
}

impl Default for SpacecraftActuators {
    fn default() -> Self {
        Self {
            forward_axis_local: Vec3::NEG_Z,
            lateral_axis_local: Vec3::X,
            vertical_axis_local: Vec3::Y,
            max_forward_thrust_newtons: 12_000.0,
            max_lateral_thrust_newtons: 3_000.0,
            max_vertical_thrust_newtons: 3_000.0,
            pitch_torque_authority: 6_000.0,
            roll_torque_authority: 5_500.0,
            yaw_torque_authority: 4_000.0,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Default)]
pub enum VehicleActuatorKind {
    #[default]
    None,
    FixedWing(FixedWingActuators),
    Rotorcraft(RotorcraftActuators),
    Hybrid(HybridActuators),
    Spacecraft(SpacecraftActuators),
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct VehicleActuators {
    pub kind: VehicleActuatorKind,
}

impl VehicleActuators {
    pub fn fixed_wing(actuators: FixedWingActuators) -> Self {
        Self {
            kind: VehicleActuatorKind::FixedWing(actuators),
        }
    }

    pub fn rotorcraft(actuators: RotorcraftActuators) -> Self {
        Self {
            kind: VehicleActuatorKind::Rotorcraft(actuators),
        }
    }

    pub fn hybrid(actuators: HybridActuators) -> Self {
        Self {
            kind: VehicleActuatorKind::Hybrid(actuators),
        }
    }

    pub fn spacecraft(actuators: SpacecraftActuators) -> Self {
        Self {
            kind: VehicleActuatorKind::Spacecraft(actuators),
        }
    }
}
