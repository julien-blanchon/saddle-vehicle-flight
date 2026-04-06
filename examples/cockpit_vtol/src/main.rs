use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_vehicle_flight::{FlightAeroState, FlightTelemetry};
use support::{
    ExamplePilot, configure_example_app_with_follow_camera, draw_force_vectors,
    spawn_lights_and_ground, spawn_overlay, spawn_vtol_demo, update_single_overlay,
};

#[derive(Component)]
struct CockpitCamera;

fn main() {
    let mut app = App::new();
    configure_example_app_with_follow_camera(&mut app, false);
    app.add_systems(Startup, setup);
    app.add_systems(Update, (update_overlay, cockpit_camera, draw_force_vectors));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_and_ground(&mut commands, &mut meshes, &mut materials);
    spawn_airfield_detail(&mut commands, &mut meshes, &mut materials);
    spawn_vtol_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Tiltrotor Test Ship",
        Transform::from_xyz(28.0, 2.1, -8.0).with_rotation(Quat::from_rotation_y(-0.24)),
        Vec3::new(0.0, 0.0, -6.0),
        0.64,
        0.18,
        true,
    );

    commands.spawn((
        Name::new("Cockpit Camera"),
        Camera3d::default(),
        CockpitCamera,
        Transform::from_xyz(26.5, 3.5, -10.0).looking_at(Vec3::new(0.0, 2.5, -40.0), Vec3::Y),
    ));
    spawn_overlay(&mut commands, "VTOL Cockpit Demo");
}

fn spawn_airfield_detail(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    commands.spawn((
        Name::new("Control Tower"),
        Mesh3d(meshes.add(Cuboid::new(4.5, 16.0, 4.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.36, 0.38, 0.42),
            perceptual_roughness: 0.92,
            ..default()
        })),
        Transform::from_xyz(-18.0, 8.0, -40.0),
    ));
    commands.spawn((
        Name::new("Tower Cab"),
        Mesh3d(meshes.add(Cuboid::new(7.2, 3.2, 7.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.60, 0.74, 0.88, 0.42),
            perceptual_roughness: 0.08,
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(-18.0, 17.2, -40.0),
    ));
    commands.spawn((
        Name::new("Hangar"),
        Mesh3d(meshes.add(Cuboid::new(22.0, 8.5, 18.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.29, 0.31, 0.35),
            perceptual_roughness: 0.88,
            ..default()
        })),
        Transform::from_xyz(-28.0, 4.25, 18.0),
    ));

    for index in 0..10 {
        let z = -90.0 + index as f32 * 18.0;
        for x in [-7.2, 7.2] {
            commands.spawn((
                Name::new(format!("Runway Light {} {}", x, index)),
                PointLight {
                    intensity: 9_000.0,
                    range: 8.0,
                    color: Color::srgb(0.98, 0.84, 0.42),
                    ..default()
                },
                Transform::from_xyz(x, 0.8, z),
            ));
        }
    }
}

fn cockpit_camera(
    aircraft: Query<&Transform, With<ExamplePilot>>,
    mut camera: Query<&mut Transform, (With<CockpitCamera>, Without<ExamplePilot>)>,
) {
    let Ok(target) = aircraft.single() else {
        return;
    };
    let Ok(mut camera) = camera.single_mut() else {
        return;
    };

    let eye_offset = target.rotation * Vec3::new(0.0, 1.15, -1.85);
    let look_offset = target.rotation * Vec3::new(0.0, 1.05, -24.0);
    camera.translation = target.translation + eye_offset;
    camera.look_at(target.translation + look_offset, Vec3::Y);
}

fn update_overlay(
    telemetry: Query<(&FlightTelemetry, &FlightAeroState), With<ExamplePilot>>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ExamplePilot>)>,
) {
    let Ok((telemetry, aero)) = telemetry.single() else {
        return;
    };
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    update_single_overlay(
        "VTOL Cockpit Demo",
        "Tiltrotor departure, hover translation, and wing-borne transition",
        telemetry,
        aero,
        &mut text,
    );
}
