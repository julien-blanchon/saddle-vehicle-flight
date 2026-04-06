use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

use crate::{
    FlightAssist, FlightBody, FlightControlInput, FlightEnvironment, FlightForces,
    FlightKinematics, FlightPlugin, FlightTelemetry, GearStateChanged, StallEntered,
    config::{
        ChannelResponse, ContactGeometry, ControlChannelBinding, ControlInputSource,
        FixedWingActuators, FixedWingModel, GroundHandling, RotorcraftActuators, RotorcraftModel,
        TrimInputSource, VehicleActuators, VehicleControlMap, VehicleModel,
    },
};

#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]
struct NeverDeactivate;

fn test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));
    app.init_schedule(NeverDeactivate);
    app.add_plugins(FlightPlugin::new(Startup, NeverDeactivate, Update));
    app
}

fn fixed_wing_trainer_setup() -> (
    VehicleModel,
    VehicleActuators,
    VehicleControlMap,
    GroundHandling,
    FlightBody,
    FlightAssist,
) {
    (
        VehicleModel::fixed_wing(FixedWingModel::default()),
        VehicleActuators::fixed_wing(FixedWingActuators::default()),
        VehicleControlMap {
            pitch: ControlChannelBinding {
                source: ControlInputSource::Pitch,
                trim: Some(TrimInputSource::Pitch),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 5.0,
                    fall_per_second: 5.0,
                    exponent: 1.25,
                },
                ..default()
            },
            roll: ControlChannelBinding {
                source: ControlInputSource::Roll,
                trim: Some(TrimInputSource::Roll),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 6.0,
                    fall_per_second: 6.0,
                    exponent: 1.15,
                },
                ..default()
            },
            yaw: ControlChannelBinding {
                source: ControlInputSource::Yaw,
                trim: Some(TrimInputSource::Yaw),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 4.0,
                    fall_per_second: 4.0,
                    exponent: 1.0,
                },
                ..default()
            },
            forward_thrust: ControlChannelBinding {
                source: ControlInputSource::Throttle,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 0.65,
                    fall_per_second: 1.0,
                    exponent: 1.0,
                },
                ..default()
            },
            ..default()
        },
        GroundHandling {
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
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FlightAssist {
            wings_leveling: 0.22,
            coordinated_turn: 0.18,
            ..default()
        },
    )
}

fn rotorcraft_utility_setup() -> (
    VehicleModel,
    VehicleActuators,
    VehicleControlMap,
    GroundHandling,
    FlightBody,
    FlightAssist,
) {
    (
        VehicleModel::rotorcraft(RotorcraftModel::default()),
        VehicleActuators::rotorcraft(RotorcraftActuators::default()),
        VehicleControlMap {
            pitch: ControlChannelBinding {
                source: ControlInputSource::Pitch,
                trim: Some(TrimInputSource::Pitch),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 6.0,
                    fall_per_second: 6.0,
                    exponent: 1.25,
                },
                ..default()
            },
            roll: ControlChannelBinding {
                source: ControlInputSource::Roll,
                trim: Some(TrimInputSource::Roll),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 6.5,
                    fall_per_second: 6.5,
                    exponent: 1.15,
                },
                ..default()
            },
            yaw: ControlChannelBinding {
                source: ControlInputSource::Yaw,
                trim: Some(TrimInputSource::Yaw),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 5.0,
                    fall_per_second: 5.0,
                    exponent: 1.0,
                },
                ..default()
            },
            vertical_thrust: ControlChannelBinding {
                source: ControlInputSource::Collective,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 1.25,
                    fall_per_second: 1.45,
                    exponent: 1.0,
                },
                ..default()
            },
            ..default()
        },
        GroundHandling {
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
        FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0)),
        FlightAssist {
            hover_leveling: 0.50,
            coordinated_turn: 0.12,
            ..default()
        },
    )
}

fn spawn_fixed_wing(
    app: &mut App,
    input: FlightControlInput,
    environment: FlightEnvironment,
    body_override: Option<FlightBody>,
    kinematics: FlightKinematics,
    transform: Transform,
) {
    let (model, actuators, control_map, ground_handling, body, assist) = fixed_wing_trainer_setup();
    app.world_mut().spawn((
        model,
        actuators,
        control_map,
        ground_handling,
        body_override.unwrap_or(body),
        assist,
        input,
        environment,
        kinematics,
        transform,
    ));
}

fn spawn_rotorcraft(
    app: &mut App,
    input: FlightControlInput,
    assist_override: Option<FlightAssist>,
    transform: Transform,
) {
    let (model, actuators, control_map, ground_handling, body, assist) = rotorcraft_utility_setup();
    app.world_mut().spawn((
        model,
        actuators,
        control_map,
        ground_handling,
        body,
        assist_override.unwrap_or(assist),
        input,
        transform,
    ));
}

#[test]
fn telemetry_updates_when_inputs_change() {
    let mut app = test_app();
    spawn_fixed_wing(
        &mut app,
        FlightControlInput {
            throttle: 0.9,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        None,
        FlightKinematics::default(),
        Transform::from_xyz(0.0, 5.0, 0.0),
    );
    for _ in 0..120 {
        app.update();
    }

    let telemetry = app
        .world_mut()
        .query::<&FlightTelemetry>()
        .single(app.world())
        .expect("telemetry should exist");
    assert!(telemetry.forward_thrust > 0.0);
}

#[test]
fn telemetry_vertical_speed_uses_world_frame_climb_rate() {
    let mut app = test_app();
    let (_, _, _, _, body, _) = fixed_wing_trainer_setup();
    spawn_fixed_wing(
        &mut app,
        FlightControlInput::default(),
        FlightEnvironment::default(),
        Some(FlightBody {
            use_internal_integration: false,
            ..body
        }),
        FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, 12.0, -28.0),
            ..default()
        },
        Transform::from_xyz(0.0, 40.0, 0.0)
            .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
    );

    app.update();
    app.update();

    let telemetry = app
        .world_mut()
        .query::<&FlightTelemetry>()
        .single(app.world())
        .expect("telemetry should exist");
    assert!((telemetry.vertical_speed_mps - 12.0).abs() < 0.01);
}

#[test]
fn stall_messages_fire_for_high_alpha() {
    let mut app = test_app();
    spawn_fixed_wing(
        &mut app,
        FlightControlInput::default(),
        FlightEnvironment::default(),
        None,
        FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, -25.0, -18.0),
            ..default()
        },
        Transform::from_xyz(0.0, 150.0, 0.0),
    );
    let mut saw_stall_message = false;
    for _ in 0..120 {
        app.update();
        let events = app
            .world_mut()
            .resource_mut::<Messages<StallEntered>>()
            .drain()
            .collect::<Vec<_>>();
        if !events.is_empty() {
            saw_stall_message = true;
            break;
        }
    }

    assert!(saw_stall_message);
}

#[test]
fn gear_messages_fire_when_requested_state_changes() {
    let mut app = test_app();
    spawn_fixed_wing(
        &mut app,
        FlightControlInput {
            requested_gear_down: Some(false),
            ..default()
        },
        FlightEnvironment::default(),
        None,
        FlightKinematics::default(),
        Transform::from_xyz(0.0, 50.0, 0.0),
    );

    let mut saw_gear_message = false;
    for _ in 0..90 {
        app.update();
        let events = app
            .world_mut()
            .resource_mut::<Messages<GearStateChanged>>()
            .drain()
            .collect::<Vec<_>>();
        if !events.is_empty() {
            saw_gear_message = true;
            break;
        }
    }
    assert!(saw_gear_message);
}

#[test]
fn fixed_wing_and_helicopter_can_coexist() {
    let mut app = test_app();
    spawn_fixed_wing(
        &mut app,
        FlightControlInput::default(),
        FlightEnvironment::default(),
        None,
        FlightKinematics::default(),
        Transform::from_xyz(-10.0, 40.0, 0.0),
    );
    spawn_rotorcraft(
        &mut app,
        FlightControlInput {
            collective: 0.65,
            ..default()
        },
        Some(FlightAssist {
            hover_leveling: 0.2,
            ..default()
        }),
        Transform::from_xyz(10.0, 40.0, 0.0),
    );
    app.update();
    app.update();

    let force_count = app
        .world_mut()
        .query::<&FlightForces>()
        .iter(app.world())
        .count();
    assert_eq!(force_count, 2);
}

#[test]
fn fixed_wing_ground_contact_keeps_forward_rollout() {
    let mut app = test_app();
    let (model, actuators, control_map, ground_handling, body, assist) = fixed_wing_trainer_setup();
    let entity = app
        .world_mut()
        .spawn((
            model,
            actuators,
            control_map,
            ground_handling,
            body,
            assist,
            FlightEnvironment {
                surface_altitude_msl_m: Some(0.0),
                ..default()
            },
            FlightKinematics {
                linear_velocity_world_mps: Vec3::new(4.0, -2.0, -32.0),
                ..default()
            },
            Transform::from_xyz(0.0, 0.9, 0.0),
        ))
        .id();

    for _ in 0..2 {
        app.update();
    }

    let (kinematics, gear) = app
        .world_mut()
        .query::<(&FlightKinematics, &crate::LandingGearState)>()
        .get(app.world(), entity)
        .expect("fixed-wing runtime state should exist");
    assert!(gear.contact);
    assert!(kinematics.linear_velocity_world_mps.z.abs() > 20.0);
    assert!(kinematics.linear_velocity_world_mps.x.abs() < 4.0);
    assert!(kinematics.linear_velocity_world_mps.y >= 0.0);
}
