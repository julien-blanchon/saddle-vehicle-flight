use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_vehicle_flight::{
    FixedWingAircraft, FlightAeroState, FlightControlInput, FlightPlugin, FlightTelemetry,
};
use support::{spawn_fixed_wing_demo, spawn_lights_ground_and_camera, spawn_overlay};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.60, 0.76, 0.95)))
        .add_plugins((DefaultPlugins, FlightPlugin::default()))
        .add_systems(Startup, setup)
        .add_systems(Update, (fly_demo, follow_camera, update_overlay))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_ground_and_camera(&mut commands, &mut meshes, &mut materials);
    spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Instrument Trainer",
        Transform::from_xyz(-40.0, 85.0, 50.0).with_rotation(Quat::from_rotation_y(0.18)),
        Vec3::new(5.0, 0.0, -54.0),
        0.70,
        false,
    );
    spawn_overlay(&mut commands, "Instrument Overlay Example");
}

fn fly_demo(time: Res<Time>, mut query: Query<&mut FlightControlInput, With<FixedWingAircraft>>) {
    let Ok(mut input) = query.single_mut() else {
        return;
    };
    let t = time.elapsed_secs();
    input.pitch = (t * 0.4).sin() * 0.16;
    input.roll = (t * 0.22).sin() * 0.28;
    input.yaw = (t * 0.18).cos() * 0.08;
    input.throttle = 0.72;
}

fn follow_camera(
    time: Res<Time>,
    aircraft: Query<&Transform, With<FixedWingAircraft>>,
    mut camera: Query<(&mut Transform, &Camera), (With<Camera3d>, Without<FixedWingAircraft>)>,
) {
    let Ok(target) = aircraft.single() else {
        return;
    };
    let Ok((mut camera, _)) = camera.single_mut() else {
        return;
    };
    let desired =
        target.translation - target.forward() * 20.0 + Vec3::Y * 8.0 + target.right() * 4.0;
    let alpha = 1.0 - (-3.0 * time.delta_secs()).exp();
    camera.translation = camera.translation.lerp(desired, alpha);
    camera.look_at(target.translation + target.forward() * 12.0, Vec3::Y);
}

fn update_overlay(
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
        "Instrument Overlay Example\nTrue airspeed {:>6.1} m/s\nIndicated airspeed {:>6.1} m/s\nAltitude MSL {:>6.1} m\nVertical speed {:>6.1} m/s\nAngle of attack {:>6.1} deg\nSideslip {:>6.1} deg\nDynamic pressure {:>7.1} Pa\nThrottle {:>4.2}  Gear {:>4.2}",
        telemetry.true_airspeed_mps,
        telemetry.indicated_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.vertical_speed_mps,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        aero.dynamic_pressure_pa,
        telemetry.throttle,
        telemetry.gear_position,
    );
}
