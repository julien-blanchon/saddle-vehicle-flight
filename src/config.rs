use crate::{
    components::{
        FlightAssist, FlightBody, FlightControlInput, FlightEnvironment, FlightKinematics,
        FlightMessageState, LandingGearState, ResolvedFlightControls, StallState,
    },
    telemetry::{FlightAeroState, FlightForces, FlightTelemetry},
};
use bevy::prelude::*;

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct FlightControlResponse {
    pub pitch_rate_per_second: f32,
    pub roll_rate_per_second: f32,
    pub yaw_rate_per_second: f32,
    pub pitch_exponent: f32,
    pub roll_exponent: f32,
    pub yaw_exponent: f32,
}

impl Default for FlightControlResponse {
    fn default() -> Self {
        Self {
            pitch_rate_per_second: 5.0,
            roll_rate_per_second: 6.0,
            yaw_rate_per_second: 4.0,
            pitch_exponent: 1.25,
            roll_exponent: 1.15,
            yaw_exponent: 1.0,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct PowerResponse {
    pub throttle_rise_per_second: f32,
    pub throttle_fall_per_second: f32,
    pub collective_rise_per_second: f32,
    pub collective_fall_per_second: f32,
}

impl Default for PowerResponse {
    fn default() -> Self {
        Self {
            throttle_rise_per_second: 0.65,
            throttle_fall_per_second: 1.0,
            collective_rise_per_second: 1.0,
            collective_fall_per_second: 1.4,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct ContactGeometry {
    pub contact_offset_below_origin_m: f32,
    pub retractable: bool,
    pub extension_rate_per_second: f32,
    pub surface_damping_per_second: f32,
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
            surface_damping_per_second: 1.8,
            ground_effect_height_m: 4.0,
            ground_effect_boost: 0.12,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
#[require(
    Transform,
    GlobalTransform,
    FlightBody,
    FlightAssist,
    FlightAeroState,
    FlightControlInput,
    FlightEnvironment,
    FlightForces,
    FlightKinematics,
    FlightMessageState,
    LandingGearState,
    ResolvedFlightControls,
    FlightTelemetry,
    StallState
)]
pub struct FixedWingAircraft {
    pub wing_area_m2: f32,
    pub wingspan_m: f32,
    pub mean_chord_m: f32,
    pub max_thrust_newtons: f32,
    pub cl0: f32,
    pub lift_curve_slope_per_rad: f32,
    pub max_lift_coefficient: f32,
    pub post_stall_lift_coefficient: f32,
    pub zero_lift_drag_coefficient: f32,
    pub induced_drag_factor: f32,
    pub stall_drag_coefficient: f32,
    pub side_force_slope_per_rad: f32,
    pub aileron_authority: f32,
    pub elevator_authority: f32,
    pub rudder_authority: f32,
    pub roll_rate_damping: f32,
    pub pitch_rate_damping: f32,
    pub yaw_rate_damping: f32,
    pub roll_stability: f32,
    pub pitch_stability: f32,
    pub yaw_stability: f32,
    pub gear_drag_coefficient: f32,
    pub trim_authority: Vec3,
    pub stall_alpha_rad: f32,
    pub recovery_alpha_rad: f32,
    pub stall_response_per_second: f32,
    pub control_response: FlightControlResponse,
    pub power_response: PowerResponse,
    pub landing_contact: ContactGeometry,
}

impl FixedWingAircraft {
    pub fn trainer() -> Self {
        Self {
            wing_area_m2: 16.2,
            wingspan_m: 10.9,
            mean_chord_m: 1.52,
            max_thrust_newtons: 3_200.0,
            cl0: 0.22,
            lift_curve_slope_per_rad: 5.25,
            max_lift_coefficient: 1.45,
            post_stall_lift_coefficient: 0.58,
            zero_lift_drag_coefficient: 0.028,
            induced_drag_factor: 0.055,
            stall_drag_coefficient: 0.72,
            side_force_slope_per_rad: 1.1,
            aileron_authority: 0.55,
            elevator_authority: 0.72,
            rudder_authority: 0.42,
            roll_rate_damping: 0.65,
            pitch_rate_damping: 0.88,
            yaw_rate_damping: 0.48,
            roll_stability: 0.18,
            pitch_stability: 0.95,
            yaw_stability: 0.52,
            gear_drag_coefficient: 0.03,
            trim_authority: Vec3::new(0.10, 0.16, 0.12),
            stall_alpha_rad: 16.0_f32.to_radians(),
            recovery_alpha_rad: 12.0_f32.to_radians(),
            stall_response_per_second: 2.8,
            control_response: FlightControlResponse::default(),
            power_response: PowerResponse::default(),
            landing_contact: ContactGeometry {
                contact_offset_below_origin_m: 1.15,
                retractable: true,
                extension_rate_per_second: 0.75,
                surface_damping_per_second: 1.4,
                ground_effect_height_m: 5.0,
                ground_effect_boost: 0.15,
            },
        }
    }

    pub fn arcade_racer() -> Self {
        let mut model = Self::trainer();
        model.max_thrust_newtons = 8_000.0;
        model.lift_curve_slope_per_rad = 6.0;
        model.max_lift_coefficient = 1.9;
        model.zero_lift_drag_coefficient = 0.02;
        model.stall_alpha_rad = 22.0_f32.to_radians();
        model.recovery_alpha_rad = 18.0_f32.to_radians();
        model.control_response.pitch_rate_per_second = 9.0;
        model.control_response.roll_rate_per_second = 10.0;
        model.control_response.yaw_rate_per_second = 7.0;
        model
    }

    pub fn trainer_body() -> FlightBody {
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0))
    }

    pub fn arcade_racer_body() -> FlightBody {
        FlightBody::new(650.0, Vec3::new(620.0, 820.0, 940.0))
    }

    /// Arcade fighter preset — no stall, high control authority, forgiving physics.
    /// Inspired by Star Fox / Ace Combat-style gameplay: fast, responsive, fun.
    pub fn arcade_fighter() -> Self {
        Self {
            wing_area_m2: 18.0,
            wingspan_m: 12.0,
            mean_chord_m: 1.6,
            max_thrust_newtons: 14_000.0,
            cl0: 0.30,
            lift_curve_slope_per_rad: 7.0,
            max_lift_coefficient: 3.0,
            post_stall_lift_coefficient: 2.4,
            zero_lift_drag_coefficient: 0.018,
            induced_drag_factor: 0.035,
            stall_drag_coefficient: 0.0,
            side_force_slope_per_rad: 0.8,
            aileron_authority: 0.80,
            elevator_authority: 0.90,
            rudder_authority: 0.60,
            roll_rate_damping: 0.50,
            pitch_rate_damping: 0.70,
            yaw_rate_damping: 0.40,
            roll_stability: 0.08,
            pitch_stability: 0.45,
            yaw_stability: 0.30,
            gear_drag_coefficient: 0.0,
            trim_authority: Vec3::new(0.05, 0.08, 0.06),
            // Very high stall angle means stall essentially never triggers
            stall_alpha_rad: 85.0_f32.to_radians(),
            recovery_alpha_rad: 80.0_f32.to_radians(),
            stall_response_per_second: 0.5,
            control_response: FlightControlResponse {
                pitch_rate_per_second: 14.0,
                roll_rate_per_second: 16.0,
                yaw_rate_per_second: 10.0,
                pitch_exponent: 1.0,
                roll_exponent: 1.0,
                yaw_exponent: 1.0,
            },
            power_response: PowerResponse {
                throttle_rise_per_second: 1.8,
                throttle_fall_per_second: 2.2,
                ..default()
            },
            landing_contact: ContactGeometry {
                contact_offset_below_origin_m: 1.0,
                retractable: false,
                extension_rate_per_second: 1.0,
                surface_damping_per_second: 1.6,
                ground_effect_height_m: 4.0,
                ground_effect_boost: 0.20,
            },
        }
    }

    pub fn arcade_fighter_body() -> FlightBody {
        FlightBody::new(550.0, Vec3::new(500.0, 700.0, 800.0))
    }
}

impl Default for FixedWingAircraft {
    fn default() -> Self {
        Self::trainer()
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
#[require(
    Transform,
    GlobalTransform,
    FlightBody,
    FlightAssist,
    FlightAeroState,
    FlightControlInput,
    FlightEnvironment,
    FlightForces,
    FlightKinematics,
    FlightMessageState,
    LandingGearState,
    ResolvedFlightControls,
    FlightTelemetry,
    StallState
)]
pub struct HelicopterAircraft {
    pub rotor_disc_area_m2: f32,
    pub max_main_lift_newtons: f32,
    pub parasite_drag_coefficient: f32,
    pub side_drag_coefficient: f32,
    pub translational_lift_gain: f32,
    pub translational_lift_full_speed_mps: f32,
    pub pitch_torque_authority: f32,
    pub roll_torque_authority: f32,
    pub yaw_torque_authority: f32,
    pub anti_torque_per_collective: f32,
    pub angular_damping: Vec3,
    pub trim_authority: Vec3,
    pub control_response: FlightControlResponse,
    pub power_response: PowerResponse,
    pub contact_geometry: ContactGeometry,
}

impl HelicopterAircraft {
    pub fn utility() -> Self {
        Self {
            rotor_disc_area_m2: 62.0,
            max_main_lift_newtons: 17_000.0,
            parasite_drag_coefficient: 0.26,
            side_drag_coefficient: 0.20,
            translational_lift_gain: 0.24,
            translational_lift_full_speed_mps: 22.0,
            pitch_torque_authority: 3_200.0,
            roll_torque_authority: 3_000.0,
            yaw_torque_authority: 2_600.0,
            anti_torque_per_collective: 1_450.0,
            angular_damping: Vec3::new(0.45, 0.65, 0.40),
            trim_authority: Vec3::new(0.10, 0.08, 0.12),
            control_response: FlightControlResponse {
                pitch_rate_per_second: 6.0,
                roll_rate_per_second: 6.5,
                yaw_rate_per_second: 5.0,
                ..default()
            },
            power_response: PowerResponse {
                collective_rise_per_second: 1.25,
                collective_fall_per_second: 1.45,
                ..default()
            },
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.65,
                retractable: false,
                extension_rate_per_second: 1.0,
                surface_damping_per_second: 2.0,
                ground_effect_height_m: 3.6,
                ground_effect_boost: 0.18,
            },
        }
    }

    pub fn arcade() -> Self {
        let mut model = Self::utility();
        model.max_main_lift_newtons = 23_000.0;
        model.translational_lift_gain = 0.18;
        model.pitch_torque_authority = 5_000.0;
        model.roll_torque_authority = 5_000.0;
        model.yaw_torque_authority = 4_200.0;
        model
    }

    pub fn utility_body() -> FlightBody {
        FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0))
    }

    pub fn arcade_body() -> FlightBody {
        FlightBody::new(900.0, Vec3::new(1_350.0, 1_250.0, 1_650.0))
    }
}

impl Default for HelicopterAircraft {
    fn default() -> Self {
        Self::utility()
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
#[require(
    Transform,
    GlobalTransform,
    FlightBody,
    FlightAssist,
    FlightAeroState,
    FlightControlInput,
    FlightEnvironment,
    FlightForces,
    FlightKinematics,
    FlightMessageState,
    LandingGearState,
    ResolvedFlightControls,
    FlightTelemetry,
    StallState
)]
pub struct VtolAircraft {
    pub fixed_wing: FixedWingAircraft,
    pub rotorcraft: HelicopterAircraft,
    pub contact_geometry: ContactGeometry,
    pub transition_rate_per_second: f32,
    pub wingborne_blend_start: f32,
    pub wingborne_blend_end: f32,
}

impl VtolAircraft {
    pub fn tiltrotor_transport() -> Self {
        let mut fixed_wing = FixedWingAircraft::trainer();
        fixed_wing.wing_area_m2 = 28.0;
        fixed_wing.wingspan_m = 15.6;
        fixed_wing.mean_chord_m = 1.94;
        fixed_wing.max_thrust_newtons = 0.0;
        fixed_wing.max_lift_coefficient = 1.62;
        fixed_wing.zero_lift_drag_coefficient = 0.031;
        fixed_wing.stall_alpha_rad = 18.0_f32.to_radians();
        fixed_wing.recovery_alpha_rad = 13.0_f32.to_radians();
        fixed_wing.control_response = FlightControlResponse {
            pitch_rate_per_second: 5.8,
            roll_rate_per_second: 6.2,
            yaw_rate_per_second: 4.2,
            ..default()
        };
        fixed_wing.power_response = PowerResponse {
            throttle_rise_per_second: 0.75,
            throttle_fall_per_second: 1.1,
            ..default()
        };

        let mut rotorcraft = HelicopterAircraft::utility();
        rotorcraft.max_main_lift_newtons = 31_000.0;
        rotorcraft.rotor_disc_area_m2 = 92.0;
        rotorcraft.translational_lift_gain = 0.18;
        rotorcraft.translational_lift_full_speed_mps = 32.0;
        rotorcraft.pitch_torque_authority = 4_400.0;
        rotorcraft.roll_torque_authority = 4_100.0;
        rotorcraft.yaw_torque_authority = 3_000.0;
        rotorcraft.anti_torque_per_collective = 1_200.0;
        rotorcraft.angular_damping = Vec3::new(0.52, 0.70, 0.44);
        rotorcraft.control_response = FlightControlResponse {
            pitch_rate_per_second: 5.0,
            roll_rate_per_second: 5.4,
            yaw_rate_per_second: 4.6,
            ..default()
        };
        rotorcraft.power_response = PowerResponse {
            throttle_rise_per_second: 0.85,
            throttle_fall_per_second: 1.25,
            collective_rise_per_second: 1.15,
            collective_fall_per_second: 1.35,
        };

        Self {
            fixed_wing,
            rotorcraft,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.95,
                retractable: true,
                extension_rate_per_second: 0.85,
                surface_damping_per_second: 1.6,
                ground_effect_height_m: 5.5,
                ground_effect_boost: 0.16,
            },
            transition_rate_per_second: 0.35,
            wingborne_blend_start: 0.22,
            wingborne_blend_end: 0.72,
        }
    }

    pub fn wingborne_blend(self, transition: f32) -> f32 {
        let width = (self.wingborne_blend_end - self.wingborne_blend_start).max(0.001);
        let normalized = (transition - self.wingborne_blend_start) / width;
        crate::math::smoothstep01(normalized)
    }

    pub fn tiltrotor_transport_body() -> FlightBody {
        FlightBody::new(2_950.0, Vec3::new(6_400.0, 7_200.0, 8_400.0))
    }
}

impl Default for VtolAircraft {
    fn default() -> Self {
        Self::tiltrotor_transport()
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
#[require(
    Transform,
    GlobalTransform,
    FlightBody,
    FlightAssist,
    FlightAeroState,
    FlightControlInput,
    FlightEnvironment,
    FlightForces,
    FlightKinematics,
    FlightMessageState,
    LandingGearState,
    ResolvedFlightControls,
    FlightTelemetry,
    StallState
)]
pub struct SpacecraftConfig {
    pub max_thrust_newtons: f32,
    pub rcs_thrust_newtons: f32,
    pub pitch_torque_authority: f32,
    pub roll_torque_authority: f32,
    pub yaw_torque_authority: f32,
    pub angular_damping: Vec3,
    pub linear_drag_coefficient: f32,
    pub control_response: FlightControlResponse,
    pub power_response: PowerResponse,
}

impl SpacecraftConfig {
    /// Generic space fighter preset — responsive controls, zero drag.
    pub fn fighter() -> Self {
        Self {
            max_thrust_newtons: 12_000.0,
            rcs_thrust_newtons: 3_000.0,
            pitch_torque_authority: 6_000.0,
            roll_torque_authority: 5_500.0,
            yaw_torque_authority: 4_000.0,
            angular_damping: Vec3::new(0.55, 0.55, 0.55),
            linear_drag_coefficient: 0.0,
            control_response: FlightControlResponse {
                pitch_rate_per_second: 12.0,
                roll_rate_per_second: 14.0,
                yaw_rate_per_second: 10.0,
                pitch_exponent: 1.0,
                roll_exponent: 1.0,
                yaw_exponent: 1.0,
            },
            power_response: PowerResponse {
                throttle_rise_per_second: 2.0,
                throttle_fall_per_second: 2.5,
                ..default()
            },
        }
    }

    /// Slow cargo vessel preset — sluggish but stable.
    pub fn cargo() -> Self {
        Self {
            max_thrust_newtons: 18_000.0,
            rcs_thrust_newtons: 2_000.0,
            pitch_torque_authority: 3_500.0,
            roll_torque_authority: 3_000.0,
            yaw_torque_authority: 2_800.0,
            angular_damping: Vec3::new(0.72, 0.72, 0.72),
            linear_drag_coefficient: 0.0,
            control_response: FlightControlResponse {
                pitch_rate_per_second: 5.0,
                roll_rate_per_second: 4.0,
                yaw_rate_per_second: 3.5,
                pitch_exponent: 1.2,
                roll_exponent: 1.2,
                yaw_exponent: 1.1,
            },
            power_response: PowerResponse {
                throttle_rise_per_second: 0.45,
                throttle_fall_per_second: 0.6,
                ..default()
            },
        }
    }

    /// Arcade space fighter — game-feel drag keeps speed manageable.
    pub fn arcade_fighter() -> Self {
        let mut model = Self::fighter();
        model.linear_drag_coefficient = 0.35;
        model.pitch_torque_authority = 8_000.0;
        model.roll_torque_authority = 7_500.0;
        model.yaw_torque_authority = 6_000.0;
        model.control_response.pitch_rate_per_second = 16.0;
        model.control_response.roll_rate_per_second = 18.0;
        model.control_response.yaw_rate_per_second = 14.0;
        model
    }

    pub fn fighter_body() -> FlightBody {
        FlightBody {
            mass_kg: 800.0,
            inertia_kgm2: Vec3::new(700.0, 900.0, 1_000.0),
            gravity_acceleration_mps2: 0.0,
            use_internal_integration: true,
        }
    }

    pub fn cargo_body() -> FlightBody {
        FlightBody {
            mass_kg: 4_500.0,
            inertia_kgm2: Vec3::new(8_000.0, 10_000.0, 12_000.0),
            gravity_acceleration_mps2: 0.0,
            use_internal_integration: true,
        }
    }
}

impl Default for SpacecraftConfig {
    fn default() -> Self {
        Self::fighter()
    }
}
