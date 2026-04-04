use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_vehicle_flight::{
    FixedWingAircraft, FlightAeroState, FlightControlInput, FlightTelemetry,
};
use support::configure_example_app_with_follow_camera;

#[derive(Resource, Debug, Clone, Copy)]
struct StallDemoState {
    elapsed: f32,
}

fn main() {
    let mut app = App::new();
    configure_example_app_with_follow_camera(&mut app, false);
    app.insert_resource(StallDemoState { elapsed: 0.0 });
    app.add_systems(Startup, setup);
    app.add_systems(Update, (script_demo, follow_camera, update_overlay));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight {
            illuminance: 20_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(18.0, 24.0, 12.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        PointLight {
            intensity: 30_000.0,
            range: 180.0,
            ..default()
        },
        Transform::from_xyz(-16.0, 14.0, 28.0),
    ));

    commands.spawn((
        Name::new("Stall Demo Camera"),
        Camera3d::default(),
        Transform::from_xyz(-16.0, 16.0, 38.0).looking_at(Vec3::new(0.0, 115.0, 0.0), Vec3::Y),
    ));

    commands.spawn((
        Name::new("Stall Demo Aircraft"),
        FixedWingAircraft::trainer(),
        FixedWingAircraft::trainer_body(),
        saddle_vehicle_flight::FlightAssist {
            wings_leveling: 0.08,
            coordinated_turn: 0.06,
            ..default()
        },
        saddle_vehicle_flight::FlightEnvironment::default(),
        saddle_vehicle_flight::FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, 0.0, -52.0),
            ..default()
        },
        FlightControlInput {
            throttle: 0.58,
            ..default()
        },
        Mesh3d(meshes.add(Cuboid::new(1.0, 0.55, 5.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.84, 0.25, 0.17),
            perceptual_roughness: 0.48,
            ..default()
        })),
        Transform::from_xyz(0.0, 120.0, 50.0),
        children![
            (
                Mesh3d(meshes.add(Cuboid::new(13.0, 0.18, 1.4))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.94, 0.94, 0.96),
                    perceptual_roughness: 0.56,
                    ..default()
                })),
                Transform::from_xyz(0.0, 0.0, 0.1),
            ),
            (
                Mesh3d(meshes.add(Cuboid::new(4.2, 0.16, 0.9))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.94, 0.94, 0.96),
                    perceptual_roughness: 0.56,
                    ..default()
                })),
                Transform::from_xyz(0.0, 0.35, 2.1),
            )
        ],
    ));

    commands.spawn((
        Text::new("stall"),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(18.0),
            top: Val::Px(18.0),
            width: Val::Px(420.0),
            padding: UiRect::all(Val::Px(12.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.05, 0.06, 0.08, 0.78)),
    ));
}

fn script_demo(
    time: Res<Time>,
    mut state: ResMut<StallDemoState>,
    mut aircraft: Query<&mut FlightControlInput, With<FixedWingAircraft>>,
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

fn follow_camera(
    time: Res<Time>,
    aircraft: Query<&Transform, With<FixedWingAircraft>>,
    mut camera: Query<&mut Transform, (With<Camera3d>, Without<FixedWingAircraft>)>,
) {
    let Ok(target) = aircraft.single() else {
        return;
    };
    let Ok(mut camera) = camera.single_mut() else {
        return;
    };

    let desired = target.translation - target.forward() * 22.0 + Vec3::Y * 8.5;
    let alpha = 1.0 - (-3.5 * time.delta_secs()).exp();
    camera.translation = camera.translation.lerp(desired, alpha);
    camera.look_at(target.translation + target.forward() * 10.0, Vec3::Y);
}

fn update_overlay(
    state: Res<StallDemoState>,
    aircraft: Query<(&FlightTelemetry, &FlightAeroState), With<FixedWingAircraft>>,
    mut overlay: Query<&mut Text, (With<Node>, Without<FixedWingAircraft>)>,
) {
    let Ok((telemetry, aero)) = aircraft.single() else {
        return;
    };
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    text.0 = format!(
        "Stall Recovery Example\nPhase {:.1}s\nTAS {:>6.1} m/s  Alt {:>6.1} m\nAoA {:>6.1} deg  Slip {:>6.1} deg\nThrottle {:>4.2}  Stalled {}\nThis example intentionally scripts: pitch-up -> stall -> unload + power -> recover",
        state.elapsed,
        telemetry.true_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        telemetry.throttle,
        telemetry.stalled,
    );
    let _ = aero;
}
