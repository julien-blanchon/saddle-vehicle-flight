use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_vehicle_flight::{FlightAeroState, FlightEnvironment, FlightTelemetry};
use support::{
    ExamplePilot, configure_example_app, draw_force_vectors, spawn_fixed_wing_demo,
    spawn_lights_ground_and_camera, spawn_overlay,
};

fn main() {
    let mut app = App::new();
    configure_example_app(&mut app);
    app.add_systems(Startup, setup);
    app.add_systems(Update, (animate_gusts, update_overlay, draw_force_vectors));
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_ground_and_camera(&mut commands, &mut meshes, &mut materials);
    let aircraft = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Crosswind Trainer",
        Transform::from_xyz(0.0, 44.0, 24.0).with_rotation(Quat::from_rotation_y(0.10)),
        Vec3::new(0.0, 0.0, -46.0),
        0.72,
        true,
    );
    commands.entity(aircraft).insert(FlightEnvironment {
        wind_world_mps: Vec3::new(6.0, 0.0, 1.5),
        gust_world_mps: Vec3::ZERO,
        surface_altitude_msl_m: Some(0.0),
        ..default()
    });
    spawn_overlay(&mut commands, "Wind And Gusts Example");
}

fn animate_gusts(time: Res<Time>, mut query: Query<&mut FlightEnvironment, With<ExamplePilot>>) {
    let Ok(mut environment) = query.single_mut() else {
        return;
    };

    let t = time.elapsed_secs();
    environment.gust_world_mps = Vec3::new(
        (t * 0.8).sin() * 4.0,
        ((t * 0.43).sin() * 0.5 + 0.5) * 1.5,
        (t * 0.33).cos() * 2.0,
    );
}

fn update_overlay(
    aircraft: Query<(&FlightTelemetry, &FlightAeroState, &FlightEnvironment), With<ExamplePilot>>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ExamplePilot>)>,
) {
    let Ok((telemetry, aero, environment)) = aircraft.single() else {
        return;
    };
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    text.0 = format!(
        "Wind And Gusts Example\nTrainer in persistent crosswind with animated gusts\n\nTAS {:>6.1} m/s  IAS {:>6.1} m/s\nAlt {:>6.1} m  AoA {:>6.1} deg  Slip {:>6.1} deg\nWind [{:>5.1}, {:>5.1}, {:>5.1}] m/s\nGust [{:>5.1}, {:>5.1}, {:>5.1}] m/s\nq {:>7.1} Pa  Throttle {:>4.2}  Gear {:>4.2}\n\nControls:\n  Arrow keys: pitch/roll  Q/E: yaw\n  [ / ]: power  G: gear\n  Mouse: orbit camera  Scroll: zoom",
        telemetry.true_airspeed_mps,
        telemetry.indicated_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        environment.wind_world_mps.x,
        environment.wind_world_mps.y,
        environment.wind_world_mps.z,
        environment.gust_world_mps.x,
        environment.gust_world_mps.y,
        environment.gust_world_mps.z,
        aero.dynamic_pressure_pa,
        telemetry.throttle,
        telemetry.gear_position,
    );
}
