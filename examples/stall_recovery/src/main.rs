use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_camera_third_person_camera::{
    AutoRecenterSettings, CollisionSettings, FollowAlignment, OrbitSettings, SmoothingSettings,
    ThirdPersonCamera, ThirdPersonCameraSettings, ThirdPersonCameraTarget, ZoomSettings,
};
use saddle_vehicle_flight::{FlightAeroState, FlightControlInput, FlightTelemetry};
use support::{ExamplePilot, configure_example_app_with_follow_camera, spawn_fixed_wing_demo};

#[derive(Resource, Debug, Clone, Copy)]
struct StallDemoState {
    elapsed: f32,
}

fn main() {
    let mut app = App::new();
    configure_example_app_with_follow_camera(&mut app, false);
    app.insert_resource(StallDemoState { elapsed: 0.0 });
    app.add_systems(Startup, setup);
    app.add_systems(Update, (script_demo, update_overlay));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let aircraft = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Stall Demo Aircraft",
        Transform::from_xyz(0.0, 120.0, 50.0),
        Vec3::new(0.0, 0.0, -52.0),
        0.58,
        true,
    );

    commands.spawn((
        Name::new("Stall Demo Camera"),
        ThirdPersonCamera::new(22.0, 0.0, -0.36),
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
                min_distance: 8.0,
                max_distance: 50.0,
                default_distance: 22.0,
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
        Transform::from_xyz(-16.0, 16.0, 38.0).looking_at(Vec3::new(0.0, 115.0, 0.0), Vec3::Y),
    ));

    support::spawn_overlay(&mut commands, "Stall Recovery Example");
}

fn script_demo(
    time: Res<Time>,
    mut state: ResMut<StallDemoState>,
    mut aircraft: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    state.elapsed += time.delta_secs();
    let Ok(mut input) = aircraft.single_mut() else {
        return;
    };

    match state.elapsed {
        t if t < 4.0 => {
            input.pitch = 0.86;
            input.roll = 0.0;
            input.yaw = 0.0;
            input.throttle = 0.45;
        }
        t if t < 8.0 => {
            input.pitch = -0.28;
            input.roll = 0.0;
            input.yaw = 0.0;
            input.throttle = 1.0;
        }
        _ => {
            input.pitch = 0.0;
            input.roll = 0.12;
            input.yaw = 0.05;
            input.throttle = 0.72;
        }
    }
}

fn update_overlay(
    state: Res<StallDemoState>,
    aircraft: Query<(&FlightTelemetry, &FlightAeroState), With<ExamplePilot>>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ExamplePilot>)>,
) {
    let Ok((telemetry, aero)) = aircraft.single() else {
        return;
    };
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    text.0 = format!(
        "Stall Recovery Example\nPhase {:.1}s\n\nTAS {:>6.1} m/s  Alt {:>6.1} m\nAoA {:>6.1} deg  Slip {:>6.1} deg\nFwd {:>4.2}  Stalled {}\n\nThis example scripts: pitch-up -> stall -> unload + power -> recover\nMouse: orbit camera  Scroll: zoom",
        state.elapsed,
        telemetry.true_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        telemetry.forward_thrust,
        telemetry.stalled,
    );
    let _ = aero;
}
