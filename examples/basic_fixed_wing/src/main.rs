use bevy::prelude::*;
use saddle_vehicle_flight::{FlightAeroState, FlightTelemetry};
use saddle_vehicle_flight_example_support as support;
use support::{
    ExamplePilot, configure_example_app, draw_force_vectors, spawn_fixed_wing_demo,
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
    spawn_overlay(&mut commands, "Fixed-Wing Example");
    spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Trainer Aircraft",
        Transform::from_xyz(0.0, 1.16, 58.0),
        Vec3::new(0.0, 0.0, -24.0),
        0.62,
        true,
    );
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
