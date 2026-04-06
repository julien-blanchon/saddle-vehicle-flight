use bevy::prelude::*;
use saddle_vehicle_flight::{
    ChannelResponse, ContactGeometry, ControlChannelBinding, ControlInputSource,
    FixedWingActuators, FixedWingModel, FlightAssist, FlightBody, GroundHandling, HybridActuators,
    HybridVehicleModel, RotorcraftActuators, RotorcraftModel, SpacecraftActuators, SpacecraftModel,
    TrimInputSource, VehicleActuators, VehicleControlMap, VehicleModel,
};

#[derive(Debug, Clone, Copy)]
pub struct ExampleVehiclePreset {
    pub model: VehicleModel,
    pub actuators: VehicleActuators,
    pub control_map: VehicleControlMap,
    pub ground_handling: GroundHandling,
    pub body: FlightBody,
    pub assist: FlightAssist,
}

pub fn fixed_wing_trainer() -> ExampleVehiclePreset {
    ExampleVehiclePreset {
        model: VehicleModel::fixed_wing(FixedWingModel::default()),
        actuators: VehicleActuators::fixed_wing(FixedWingActuators::default()),
        control_map: VehicleControlMap {
            pitch: signed_axis(
                ControlInputSource::Pitch,
                Some(TrimInputSource::Pitch),
                5.0,
                1.25,
            ),
            roll: signed_axis(
                ControlInputSource::Roll,
                Some(TrimInputSource::Roll),
                6.0,
                1.15,
            ),
            yaw: signed_axis(
                ControlInputSource::Yaw,
                Some(TrimInputSource::Yaw),
                4.0,
                1.0,
            ),
            forward_thrust: positive_axis(ControlInputSource::Throttle, 0.65, 1.0),
            ..default()
        },
        ground_handling: GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.15,
                retractable: true,
                extension_rate_per_second: 0.75,
                ground_effect_height_m: 5.0,
                ground_effect_boost: 0.15,
            },
            longitudinal_damping_per_second: 0.14,
            lateral_damping_per_second: 1.4,
            angular_damping_per_second: 1.4,
        },
        body: FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        assist: FlightAssist {
            wings_leveling: 0.22,
            coordinated_turn: 0.18,
            ..default()
        },
    }
}

pub fn fixed_wing_arcade_racer() -> ExampleVehiclePreset {
    let mut preset = fixed_wing_trainer();
    let VehicleModel { kind } = &mut preset.model;
    if let saddle_vehicle_flight::VehicleModelKind::FixedWing(model) = kind {
        model.wing_area_m2 = 16.2;
        model.wingspan_m = 10.9;
        model.mean_chord_m = 1.52;
        model.lift_curve_slope_per_rad = 6.0;
        model.max_lift_coefficient = 1.9;
        model.zero_lift_drag_coefficient = 0.02;
        model.stall_alpha_rad = 22.0_f32.to_radians();
        model.recovery_alpha_rad = 18.0_f32.to_radians();
    }
    let VehicleActuators { kind } = &mut preset.actuators;
    if let saddle_vehicle_flight::VehicleActuatorKind::FixedWing(actuators) = kind {
        actuators.max_forward_thrust_newtons = 8_000.0;
    }
    preset.control_map.pitch = signed_axis(
        ControlInputSource::Pitch,
        Some(TrimInputSource::Pitch),
        9.0,
        1.0,
    );
    preset.control_map.roll = signed_axis(
        ControlInputSource::Roll,
        Some(TrimInputSource::Roll),
        10.0,
        1.0,
    );
    preset.control_map.yaw = signed_axis(
        ControlInputSource::Yaw,
        Some(TrimInputSource::Yaw),
        7.0,
        1.0,
    );
    preset.body = FlightBody::new(650.0, Vec3::new(620.0, 820.0, 940.0));
    preset.assist = FlightAssist {
        wings_leveling: 0.08,
        coordinated_turn: 0.06,
        ..default()
    };
    preset
}

pub fn rotorcraft_utility() -> ExampleVehiclePreset {
    ExampleVehiclePreset {
        model: VehicleModel::rotorcraft(RotorcraftModel::default()),
        actuators: VehicleActuators::rotorcraft(RotorcraftActuators::default()),
        control_map: VehicleControlMap {
            pitch: signed_axis(
                ControlInputSource::Pitch,
                Some(TrimInputSource::Pitch),
                6.0,
                1.25,
            ),
            roll: signed_axis(
                ControlInputSource::Roll,
                Some(TrimInputSource::Roll),
                6.5,
                1.15,
            ),
            yaw: signed_axis(
                ControlInputSource::Yaw,
                Some(TrimInputSource::Yaw),
                5.0,
                1.0,
            ),
            vertical_thrust: positive_axis(ControlInputSource::Collective, 1.25, 1.45),
            ..default()
        },
        ground_handling: GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.65,
                retractable: false,
                extension_rate_per_second: 1.0,
                ground_effect_height_m: 3.6,
                ground_effect_boost: 0.18,
            },
            longitudinal_damping_per_second: 2.0,
            lateral_damping_per_second: 2.0,
            angular_damping_per_second: 2.0,
        },
        body: FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0)),
        assist: FlightAssist {
            hover_leveling: 0.50,
            coordinated_turn: 0.12,
            ..default()
        },
    }
}

pub fn rotorcraft_arcade() -> ExampleVehiclePreset {
    let mut preset = rotorcraft_utility();
    let VehicleModel { kind } = &mut preset.model;
    if let saddle_vehicle_flight::VehicleModelKind::Rotorcraft(model) = kind {
        model.translational_lift_gain = 0.18;
    }
    let VehicleActuators { kind } = &mut preset.actuators;
    if let saddle_vehicle_flight::VehicleActuatorKind::Rotorcraft(actuators) = kind {
        actuators.max_lift_newtons = 23_000.0;
        actuators.pitch_torque_authority = 5_000.0;
        actuators.roll_torque_authority = 5_000.0;
        actuators.yaw_torque_authority = 4_200.0;
    }
    preset.body = FlightBody::new(900.0, Vec3::new(1_350.0, 1_250.0, 1_650.0));
    preset.assist = FlightAssist {
        hover_leveling: 0.22,
        coordinated_turn: 0.08,
        ..default()
    };
    preset
}

pub fn tiltrotor_transport() -> ExampleVehiclePreset {
    ExampleVehiclePreset {
        model: VehicleModel::hybrid(HybridVehicleModel::default()),
        actuators: VehicleActuators::hybrid(HybridActuators::default()),
        control_map: VehicleControlMap {
            pitch: signed_axis(
                ControlInputSource::Pitch,
                Some(TrimInputSource::Pitch),
                5.4,
                1.25,
            ),
            roll: signed_axis(
                ControlInputSource::Roll,
                Some(TrimInputSource::Roll),
                5.8,
                1.15,
            ),
            yaw: signed_axis(
                ControlInputSource::Yaw,
                Some(TrimInputSource::Yaw),
                4.4,
                1.0,
            ),
            forward_thrust: positive_axis(ControlInputSource::Throttle, 0.85, 1.25),
            vertical_thrust: positive_axis(ControlInputSource::Collective, 1.15, 1.35),
            transition: ControlChannelBinding {
                source: ControlInputSource::Transition,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 0.35,
                    fall_per_second: 0.35,
                    exponent: 1.0,
                },
                ..default()
            },
            ..default()
        },
        ground_handling: GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.95,
                retractable: true,
                extension_rate_per_second: 0.85,
                ground_effect_height_m: 5.5,
                ground_effect_boost: 0.16,
            },
            longitudinal_damping_per_second: 0.16,
            lateral_damping_per_second: 1.6,
            angular_damping_per_second: 1.6,
        },
        body: FlightBody::new(2_950.0, Vec3::new(6_400.0, 7_200.0, 8_400.0)),
        assist: FlightAssist {
            wings_leveling: 0.18,
            coordinated_turn: 0.16,
            hover_leveling: 0.44,
        },
    }
}

pub fn spacecraft_fighter() -> ExampleVehiclePreset {
    ExampleVehiclePreset {
        model: VehicleModel::spacecraft(SpacecraftModel::default()),
        actuators: VehicleActuators::spacecraft(SpacecraftActuators::default()),
        control_map: VehicleControlMap {
            pitch: signed_axis(ControlInputSource::Pitch, None, 12.0, 1.0),
            roll: signed_axis(ControlInputSource::Roll, None, 14.0, 1.0),
            yaw: signed_axis(ControlInputSource::Yaw, None, 10.0, 1.0),
            forward_thrust: positive_axis(ControlInputSource::Throttle, 2.0, 2.5),
            vertical_thrust: centered_positive_axis(ControlInputSource::Collective, 2.0, 2.5),
            lateral_thrust: signed_axis(ControlInputSource::Yaw, None, 10.0, 1.0),
            ..default()
        },
        ground_handling: GroundHandling::default(),
        body: FlightBody {
            mass_kg: 800.0,
            inertia_kgm2: Vec3::new(700.0, 900.0, 1_000.0),
            gravity_acceleration_mps2: 0.0,
            use_internal_integration: true,
        },
        assist: FlightAssist::default(),
    }
}

pub fn spacecraft_cargo() -> ExampleVehiclePreset {
    let mut preset = spacecraft_fighter();
    let VehicleModel { kind } = &mut preset.model;
    if let saddle_vehicle_flight::VehicleModelKind::Spacecraft(model) = kind {
        model.angular_damping = Vec3::new(0.72, 0.72, 0.72);
    }
    let VehicleActuators { kind } = &mut preset.actuators;
    if let saddle_vehicle_flight::VehicleActuatorKind::Spacecraft(actuators) = kind {
        actuators.max_forward_thrust_newtons = 18_000.0;
        actuators.max_lateral_thrust_newtons = 2_000.0;
        actuators.max_vertical_thrust_newtons = 2_000.0;
        actuators.pitch_torque_authority = 3_500.0;
        actuators.roll_torque_authority = 3_000.0;
        actuators.yaw_torque_authority = 2_800.0;
    }
    preset.body = FlightBody {
        mass_kg: 4_500.0,
        inertia_kgm2: Vec3::new(8_000.0, 10_000.0, 12_000.0),
        gravity_acceleration_mps2: 0.0,
        use_internal_integration: true,
    };
    preset.control_map.pitch = signed_axis(ControlInputSource::Pitch, None, 5.0, 1.2);
    preset.control_map.roll = signed_axis(ControlInputSource::Roll, None, 4.0, 1.2);
    preset.control_map.yaw = signed_axis(ControlInputSource::Yaw, None, 3.5, 1.1);
    preset.control_map.forward_thrust = positive_axis(ControlInputSource::Throttle, 0.45, 0.6);
    preset.control_map.vertical_thrust =
        centered_positive_axis(ControlInputSource::Collective, 0.45, 0.6);
    preset
}

pub fn spacecraft_arcade_fighter() -> ExampleVehiclePreset {
    let mut preset = spacecraft_fighter();
    let VehicleModel { kind } = &mut preset.model;
    if let saddle_vehicle_flight::VehicleModelKind::Spacecraft(model) = kind {
        model.linear_drag_coefficient = 0.35;
    }
    let VehicleActuators { kind } = &mut preset.actuators;
    if let saddle_vehicle_flight::VehicleActuatorKind::Spacecraft(actuators) = kind {
        actuators.pitch_torque_authority = 8_000.0;
        actuators.roll_torque_authority = 7_500.0;
        actuators.yaw_torque_authority = 6_000.0;
    }
    preset.control_map.pitch = signed_axis(ControlInputSource::Pitch, None, 16.0, 1.0);
    preset.control_map.roll = signed_axis(ControlInputSource::Roll, None, 18.0, 1.0);
    preset.control_map.yaw = signed_axis(ControlInputSource::Yaw, None, 14.0, 1.0);
    preset
}

fn signed_axis(
    source: ControlInputSource,
    trim: Option<TrimInputSource>,
    rise_and_fall: f32,
    exponent: f32,
) -> ControlChannelBinding {
    ControlChannelBinding {
        source,
        trim,
        trim_scale: 1.0,
        clamp_min: -1.0,
        clamp_max: 1.0,
        response: ChannelResponse {
            rise_per_second: rise_and_fall,
            fall_per_second: rise_and_fall,
            exponent,
        },
        ..default()
    }
}

fn positive_axis(
    source: ControlInputSource,
    rise_per_second: f32,
    fall_per_second: f32,
) -> ControlChannelBinding {
    ControlChannelBinding {
        source,
        clamp_min: 0.0,
        clamp_max: 1.0,
        response: ChannelResponse {
            rise_per_second,
            fall_per_second,
            exponent: 1.0,
        },
        ..default()
    }
}

fn centered_positive_axis(
    source: ControlInputSource,
    rise_per_second: f32,
    fall_per_second: f32,
) -> ControlChannelBinding {
    ControlChannelBinding {
        source,
        scale: 2.0,
        offset: -1.0,
        clamp_min: -1.0,
        clamp_max: 1.0,
        response: ChannelResponse {
            rise_per_second,
            fall_per_second,
            exponent: 1.0,
        },
        ..default()
    }
}
