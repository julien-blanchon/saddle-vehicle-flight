use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

use crate::{
    FixedWingAircraft, FlightAssist, FlightBody, FlightControlInput, FlightEnvironment,
    FlightForces, FlightKinematics, FlightPlugin, FlightTelemetry, GearStateChanged,
    HelicopterAircraft, StallEntered,
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

#[test]
fn telemetry_updates_when_inputs_change() {
    let mut app = test_app();
    app.world_mut().spawn((
        FixedWingAircraft::trainer(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FlightControlInput {
            throttle: 0.9,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        Transform::from_xyz(0.0, 5.0, 0.0),
    ));
    for _ in 0..120 {
        app.update();
    }

    let telemetry = app
        .world_mut()
        .query::<&FlightTelemetry>()
        .single(app.world())
        .expect("telemetry should exist");
    assert!(telemetry.throttle > 0.0);
}

#[test]
fn telemetry_vertical_speed_uses_world_frame_climb_rate() {
    let mut app = test_app();
    app.world_mut().spawn((
        FixedWingAircraft::trainer(),
        FlightBody {
            use_internal_integration: false,
            ..FixedWingAircraft::trainer_body()
        },
        FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, 12.0, -28.0),
            ..default()
        },
        Transform::from_xyz(0.0, 40.0, 0.0)
            .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
    ));

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
    app.world_mut().spawn((
        FixedWingAircraft::trainer(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, -25.0, -18.0),
            ..default()
        },
        Transform::from_xyz(0.0, 150.0, 0.0),
    ));
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
    app.world_mut().spawn((
        FixedWingAircraft::trainer(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FlightControlInput {
            requested_gear_down: Some(false),
            ..default()
        },
        Transform::from_xyz(0.0, 50.0, 0.0),
    ));

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
    app.world_mut().spawn((
        FixedWingAircraft::trainer(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        Transform::from_xyz(-10.0, 40.0, 0.0),
    ));
    app.world_mut().spawn((
        HelicopterAircraft::utility(),
        FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0)),
        FlightControlInput {
            collective: 0.65,
            ..default()
        },
        FlightAssist {
            hover_leveling: 0.2,
            ..default()
        },
        Transform::from_xyz(10.0, 40.0, 0.0),
    ));
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
    let entity = app
        .world_mut()
        .spawn((
            FixedWingAircraft::trainer(),
            FixedWingAircraft::trainer_body(),
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
