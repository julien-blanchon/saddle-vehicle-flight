use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_camera_third_person_camera::{
    AutoRecenterSettings, CollisionSettings, FollowAlignment, OrbitSettings, SmoothingSettings,
    ThirdPersonCamera, ThirdPersonCameraSettings, ThirdPersonCameraTarget, ZoomSettings,
};
use saddle_vehicle_flight::{FlightAeroState, FlightControlInput, FlightTelemetry};
use support::{
    configure_example_app_with_follow_camera, spawn_fixed_wing_demo, spawn_lights_and_ground,
    spawn_overlay,
};

fn main() {
    let mut app = App::new();
    configure_example_app_with_follow_camera(&mut app, false);
    app.add_systems(Startup, setup);
    app.add_systems(Update, (fly_demo, update_overlay));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_and_ground(&mut commands, &mut meshes, &mut materials);
    let aircraft = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Instrument Trainer",
        Transform::from_xyz(-40.0, 85.0, 50.0).with_rotation(Quat::from_rotation_y(0.18)),
        Vec3::new(5.0, 0.0, -54.0),
        0.70,
        false,
    );

    commands.spawn((
        Name::new("Instruments Camera"),
        ThirdPersonCamera::new(20.0, 0.4, -0.36),
        ThirdPersonCameraTarget::new(aircraft),
        ThirdPersonCameraSettings {
            orbit: OrbitSettings {
                yaw_speed: 1.2,
                pitch_speed: 1.1,
                min_pitch: -1.45,
                max_pitch: 0.10,
                ..default()
            },
            smoothing: SmoothingSettings {
                orientation_smoothing: 6.0,
                target_follow_smoothing: 4.0,
                zoom_smoothing: 12.0,
                ..default()
            },
            zoom: ZoomSettings {
                min_distance: 6.0,
                max_distance: 50.0,
                default_distance: 20.0,
                step: 2.0,
            },
            collision: CollisionSettings {
                enabled: false,
                ..default()
            },
            auto_recenter: AutoRecenterSettings {
                enabled: true,
                inactivity_seconds: 1.5,
                follow_alignment: FollowAlignment::TargetForward,
            },
            ..default()
        },
    ));

    spawn_overlay(&mut commands, "Instrument Overlay Example");
}

fn fly_demo(time: Res<Time>, mut query: Query<&mut FlightControlInput>) {
    let Ok(mut input) = query.single_mut() else {
        return;
    };
    let t = time.elapsed_secs();
    input.pitch = (t * 0.4).sin() * 0.16;
    input.roll = (t * 0.22).sin() * 0.28;
    input.yaw = (t * 0.18).cos() * 0.08;
    input.throttle = 0.72;
}

fn update_overlay(
    aircraft: Query<(&FlightTelemetry, &FlightAeroState)>,
    mut overlay: Query<&mut Text, With<Node>>,
) {
    let Ok((telemetry, aero)) = aircraft.single() else {
        return;
    };
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    text.0 = format!(
        "Instrument Overlay Example\n\nTrue airspeed {:>6.1} m/s\nIndicated airspeed {:>6.1} m/s\nAltitude MSL {:>6.1} m\nVertical speed {:>6.1} m/s\nAngle of attack {:>6.1} deg\nSideslip {:>6.1} deg\nDynamic pressure {:>7.1} Pa\nFwd {:>4.2}  Vert {:>4.2}  Lat {:>4.2}  Gear {:>4.2}\n\nMouse: orbit camera  Scroll: zoom",
        telemetry.true_airspeed_mps,
        telemetry.indicated_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.vertical_speed_mps,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        aero.dynamic_pressure_pa,
        telemetry.forward_thrust,
        telemetry.vertical_thrust,
        telemetry.lateral_thrust,
        telemetry.gear_position,
    );
}
