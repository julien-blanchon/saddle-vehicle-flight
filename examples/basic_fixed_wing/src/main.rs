//! Basic fixed-wing aircraft example — trainer aircraft takeoff and flight.
//!
//! Shows full `FixedWingAircraft` configuration (lift curve, drag, stall, control
//! authority, landing gear), `FlightBody`, `FlightAssist`, and `FlightKinematics`
//! with every field visible.  Arrow keys for pitch/roll, Q/E for yaw, [/] for
//! throttle, G to toggle gear.

use bevy::prelude::*;
use saddle_vehicle_flight::{
    ContactGeometry, FixedWingAircraft, FlightAeroState, FlightAssist, FlightBody,
    FlightControlInput, FlightControlResponse, FlightEnvironment, FlightKinematics,
    FlightTelemetry, PowerResponse,
};
use saddle_vehicle_flight_example_support as support;
use support::{
    ExamplePilot, configure_example_app, draw_force_vectors, spawn_lights_ground_and_camera,
    spawn_overlay, update_single_overlay,
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
    spawn_overlay(&mut commands, "Fixed-Wing Example");

    // ---------------------------------------------------------------------------
    // Aircraft configuration — single-engine trainer
    // ---------------------------------------------------------------------------
    let aircraft = FixedWingAircraft {
        wing_area_m2: 16.2,
        wingspan_m: 10.9,
        mean_chord_m: 1.52,
        max_thrust_newtons: 3_200.0,
        // Lift
        cl0: 0.22,
        lift_curve_slope_per_rad: 5.25,
        max_lift_coefficient: 1.45,
        post_stall_lift_coefficient: 0.58,
        // Drag
        zero_lift_drag_coefficient: 0.028,
        induced_drag_factor: 0.055,
        stall_drag_coefficient: 0.72,
        gear_drag_coefficient: 0.03,
        // Lateral
        side_force_slope_per_rad: 1.1,
        // Control authority
        aileron_authority: 0.55,
        elevator_authority: 0.72,
        rudder_authority: 0.42,
        // Angular damping
        roll_rate_damping: 0.65,
        pitch_rate_damping: 0.88,
        yaw_rate_damping: 0.48,
        // Static stability
        roll_stability: 0.18,
        pitch_stability: 0.95,
        yaw_stability: 0.52,
        // Trim
        trim_authority: Vec3::new(0.10, 0.16, 0.12),
        // Stall
        stall_alpha_rad: 16.0_f32.to_radians(),
        recovery_alpha_rad: 12.0_f32.to_radians(),
        stall_response_per_second: 2.8,
        // Response curves
        control_response: FlightControlResponse::default(),
        power_response: PowerResponse::default(),
        // Landing gear
        landing_contact: ContactGeometry {
            contact_offset_below_origin_m: 1.15,
            retractable: true,
            extension_rate_per_second: 0.75,
            surface_damping_per_second: 1.4,
            ground_effect_height_m: 5.0,
            ground_effect_boost: 0.15,
        },
    };

    let body = FlightBody {
        mass_kg: 980.0,
        inertia_kgm2: Vec3::new(900.0, 1_450.0, 1_700.0),
        gravity_acceleration_mps2: 9.80665,
        use_internal_integration: true,
    };

    // ---------------------------------------------------------------------------
    // Spawn the aircraft entity
    // ---------------------------------------------------------------------------
    let transform = Transform::from_xyz(0.0, 1.16, 58.0);
    let initial_velocity = Vec3::new(0.0, 0.0, -24.0);

    let mut entity = commands.spawn((
        Name::new("Trainer Aircraft"),
        aircraft,
        body,
        FlightAssist {
            wings_leveling: 0.22,
            coordinated_turn: 0.18,
            ..default()
        },
        FlightKinematics {
            linear_velocity_world_mps: initial_velocity,
            ..default()
        },
        FlightControlInput {
            throttle: 0.62,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        // Visual fuselage
        Mesh3d(meshes.add(Cuboid::new(1.0, 0.55, 5.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.82, 0.21, 0.19),
            perceptual_roughness: 0.48,
            metallic: 0.08,
            ..default()
        })),
        transform,
    ));

    entity.insert(support::pilot_actions());

    entity.with_children(|parent| {
        parent.spawn((
            Name::new("Wing"),
            Mesh3d(meshes.add(Cuboid::new(13.0, 0.18, 1.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.93, 0.93, 0.96),
                perceptual_roughness: 0.54,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.1),
        ));
        parent.spawn((
            Name::new("Tailplane"),
            Mesh3d(meshes.add(Cuboid::new(4.2, 0.16, 0.9))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.93, 0.93, 0.96),
                perceptual_roughness: 0.54,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.35, 2.1),
        ));
        parent.spawn((
            Name::new("Vertical Tail"),
            Mesh3d(meshes.add(Cuboid::new(0.22, 1.2, 1.1))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.93, 0.93, 0.96),
                perceptual_roughness: 0.54,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.82, 2.0),
        ));
        parent.spawn((
            Name::new("Nose Spinner"),
            Mesh3d(meshes.add(Capsule3d::new(0.22, 0.8))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.14, 0.14, 0.15),
                metallic: 0.32,
                perceptual_roughness: 0.4,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, -2.9)
                .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
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
        "Fixed-Wing Example",
        "Trainer takeoff and circuit",
        telemetry,
        aero,
        &mut text,
    );
}
