//! Helicopter hover example — utility helicopter hover and translation.
//!
//! Shows full `HelicopterAircraft` configuration (rotor, torque authorities,
//! angular damping, contact geometry), `FlightBody`, `FlightAssist`, and
//! `FlightKinematics` with every field visible.  Arrow keys for pitch/roll,
//! Q/E for yaw, [/] for collective.

use bevy::prelude::*;
use saddle_vehicle_flight::{
    ContactGeometry, FlightAeroState, FlightAssist, FlightBody, FlightControlInput,
    FlightControlResponse, FlightEnvironment, FlightKinematics, FlightTelemetry,
    HelicopterAircraft, PowerResponse,
};
use saddle_vehicle_flight_example_support as support;
use support::{
    ExamplePilot, RotorDisk, configure_example_app, draw_force_vectors,
    spawn_lights_ground_and_camera, spawn_overlay, update_single_overlay,
};

fn main() {
    let mut app = App::new();
    configure_example_app(&mut app);
    app.add_systems(Startup, setup);
    app.add_systems(Update, (update_overlay, draw_force_vectors));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_ground_and_camera(&mut commands, &mut meshes, &mut materials);
    spawn_overlay(&mut commands, "Helicopter Hover Example");

    // ---------------------------------------------------------------------------
    // Helicopter configuration — utility rotorcraft
    // ---------------------------------------------------------------------------
    let helicopter = HelicopterAircraft {
        rotor_disc_area_m2: 62.0,
        max_main_lift_newtons: 17_000.0,
        // Drag
        parasite_drag_coefficient: 0.26,
        side_drag_coefficient: 0.20,
        // Translational lift
        translational_lift_gain: 0.24,
        translational_lift_full_speed_mps: 22.0,
        // Torque authority (pitch / roll / yaw)
        pitch_torque_authority: 3_200.0,
        roll_torque_authority: 3_000.0,
        yaw_torque_authority: 2_600.0,
        anti_torque_per_collective: 1_450.0,
        // Angular damping
        angular_damping: Vec3::new(0.45, 0.65, 0.40),
        // Trim
        trim_authority: Vec3::new(0.10, 0.08, 0.12),
        // Control response
        control_response: FlightControlResponse {
            pitch_rate_per_second: 6.0,
            roll_rate_per_second: 6.5,
            yaw_rate_per_second: 5.0,
            ..default()
        },
        // Power response
        power_response: PowerResponse {
            collective_rise_per_second: 1.25,
            collective_fall_per_second: 1.45,
            ..default()
        },
        // Contact / landing
        contact_geometry: ContactGeometry {
            contact_offset_below_origin_m: 1.65,
            retractable: false,
            extension_rate_per_second: 1.0,
            surface_damping_per_second: 2.0,
            ground_effect_height_m: 3.6,
            ground_effect_boost: 0.18,
        },
    };

    let body = FlightBody {
        mass_kg: 1_150.0,
        inertia_kgm2: Vec3::new(1_800.0, 1_600.0, 2_100.0),
        gravity_acceleration_mps2: 9.80665,
        use_internal_integration: true,
    };

    // ---------------------------------------------------------------------------
    // Spawn the helicopter entity
    // ---------------------------------------------------------------------------
    let transform = Transform::from_xyz(28.0, 2.6, 0.0);

    let mut entity = commands.spawn((
        Name::new("Utility Helicopter"),
        helicopter,
        body,
        FlightAssist {
            hover_leveling: 0.50,
            coordinated_turn: 0.12,
            ..default()
        },
        FlightKinematics {
            linear_velocity_world_mps: Vec3::ZERO,
            ..default()
        },
        FlightControlInput {
            collective: 0.60,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        // Visual fuselage
        Mesh3d(meshes.add(Capsule3d::new(0.65, 2.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.17, 0.46, 0.73),
            perceptual_roughness: 0.42,
            metallic: 0.06,
            ..default()
        })),
        transform,
    ));

    entity.insert(support::pilot_actions());

    entity.with_children(|parent| {
        parent.spawn((
            Name::new("Main Rotor"),
            RotorDisk { speed_rps: 16.0 },
            Mesh3d(meshes.add(Cuboid::new(10.5, 0.06, 0.22))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.12, 0.12, 0.13),
                perceptual_roughness: 0.38,
                ..default()
            })),
            Transform::from_xyz(0.0, 1.35, 0.0),
        ));
        parent.spawn((
            Name::new("Tail Boom"),
            Mesh3d(meshes.add(Cuboid::new(0.25, 0.25, 5.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.15, 0.17, 0.19),
                perceptual_roughness: 0.56,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.3, 3.8),
        ));
        parent.spawn((
            Name::new("Tail Rotor"),
            RotorDisk { speed_rps: 34.0 },
            Mesh3d(meshes.add(Cuboid::new(0.12, 1.5, 0.08))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.12, 0.12, 0.13),
                perceptual_roughness: 0.38,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.6, 6.5),
        ));
        parent.spawn((
            Name::new("Skids"),
            Mesh3d(meshes.add(Cuboid::new(2.6, 0.08, 5.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.08, 0.08, 0.09),
                perceptual_roughness: 0.64,
                ..default()
            })),
            Transform::from_xyz(0.0, -1.0, 0.0),
        ));
    });
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
        "Helicopter Hover Example",
        "Utility helicopter hover and translation",
        telemetry,
        aero,
        &mut text,
    );
}
