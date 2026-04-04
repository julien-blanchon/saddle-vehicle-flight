use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
use saddle_vehicle_flight::{
    FixedWingAircraft, FlightAssist, FlightBody, FlightControlInput, FlightTelemetry,
    HelicopterAircraft,
};
use support::{
    configure_example_app, spawn_fixed_wing_demo, spawn_helicopter_demo,
    spawn_lights_ground_and_camera, spawn_overlay,
};

#[derive(Component, Debug, Clone, Copy)]
struct ProfileLabel(&'static str);

#[derive(Component)]
struct FixedWingTrainerProfile;

#[derive(Component)]
struct FixedWingArcadeProfile;

#[derive(Component)]
struct HelicopterUtilityProfile;

#[derive(Component)]
struct HelicopterArcadeProfile;

fn main() {
    let mut app = App::new();
    configure_example_app(&mut app);
    app.add_systems(Startup, setup);
    app.add_systems(
        Update,
        (
            script_fixed_wing_profiles,
            script_helicopter_profiles,
            update_overlay,
        ),
    );
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lights_ground_and_camera(&mut commands, &mut meshes, &mut materials);

    let trainer = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Trainer Profile",
        Transform::from_xyz(-12.0, 12.0, 18.0).with_rotation(Quat::from_rotation_y(0.10)),
        Vec3::new(1.5, 0.0, -42.0),
        0.70,
        false,
    );
    commands
        .entity(trainer)
        .insert((ProfileLabel("Trainer Fixed-Wing"), FixedWingTrainerProfile));

    let racer = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Arcade Racer Profile",
        Transform::from_xyz(6.0, 16.0, 10.0).with_rotation(Quat::from_rotation_y(0.16)),
        Vec3::new(3.0, 0.0, -58.0),
        0.88,
        false,
    );
    commands.entity(racer).insert((
        ProfileLabel("Arcade Fixed-Wing"),
        FixedWingArcadeProfile,
        FixedWingAircraft::arcade_racer(),
        FixedWingAircraft::arcade_racer_body(),
        FlightAssist {
            wings_leveling: 0.08,
            coordinated_turn: 0.06,
            ..default()
        },
    ));

    let utility = spawn_helicopter_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Utility Helicopter Profile",
        Transform::from_xyz(24.0, 5.5, 12.0),
        Vec3::ZERO,
        0.60,
        false,
    );
    commands
        .entity(utility)
        .insert((ProfileLabel("Utility Helicopter"), HelicopterUtilityProfile));

    let arcade = spawn_helicopter_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Arcade Helicopter Profile",
        Transform::from_xyz(34.0, 8.5, -2.0),
        Vec3::ZERO,
        0.64,
        false,
    );
    commands.entity(arcade).insert((
        ProfileLabel("Arcade Helicopter"),
        HelicopterArcadeProfile,
        HelicopterAircraft::arcade(),
        FlightBody::new(900.0, Vec3::new(1_350.0, 1_250.0, 1_650.0)),
        FlightAssist {
            hover_leveling: 0.22,
            coordinated_turn: 0.08,
            ..default()
        },
    ));

    spawn_overlay(&mut commands, "Control Profiles Example");
}

fn script_fixed_wing_profiles(
    time: Res<Time>,
    mut trainer: Query<
        &mut FlightControlInput,
        (With<FixedWingTrainerProfile>, Without<FixedWingArcadeProfile>),
    >,
    mut racer: Query<
        &mut FlightControlInput,
        (With<FixedWingArcadeProfile>, Without<FixedWingTrainerProfile>),
    >,
) {
    let t = time.elapsed_secs();

    if let Ok(mut input) = trainer.single_mut() {
        input.pitch = (t * 0.45).sin() * 0.08;
        input.roll = (t * 0.22).sin() * 0.22;
        input.yaw = (t * 0.18).cos() * 0.04;
        input.throttle = 0.70;
    }

    if let Ok(mut input) = racer.single_mut() {
        input.pitch = (t * 0.72).sin() * 0.16;
        input.roll = (t * 0.48).sin() * 0.38;
        input.yaw = (t * 0.34).cos() * 0.10;
        input.throttle = 0.92;
    }
}

fn script_helicopter_profiles(
    time: Res<Time>,
    mut utility: Query<
        &mut FlightControlInput,
        (With<HelicopterUtilityProfile>, Without<HelicopterArcadeProfile>),
    >,
    mut arcade: Query<
        &mut FlightControlInput,
        (With<HelicopterArcadeProfile>, Without<HelicopterUtilityProfile>),
    >,
) {
    let t = time.elapsed_secs();

    if let Ok(mut input) = utility.single_mut() {
        input.pitch = (t * 0.38).sin() * 0.14;
        input.roll = (t * 0.32).cos() * 0.10;
        input.yaw = (t * 0.22).sin() * 0.10;
        input.collective = 0.60 + (t * 0.18).sin() * 0.02;
    }

    if let Ok(mut input) = arcade.single_mut() {
        input.pitch = (t * 0.72).sin() * 0.24;
        input.roll = (t * 0.58).cos() * 0.20;
        input.yaw = (t * 0.44).sin() * 0.16;
        input.collective = 0.66 + (t * 0.24).sin() * 0.03;
    }
}

fn update_overlay(
    profiles: Query<(&ProfileLabel, &FlightTelemetry)>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ProfileLabel>)>,
) {
    let Ok(mut text) = overlay.single_mut() else {
        return;
    };

    let mut rows = profiles
        .iter()
        .map(|(label, telemetry)| {
            format!(
                "{:<20} TAS {:>5.1}  V/S {:>5.1}  AoA {:>5.1}  Slip {:>5.1}  Power {:>4.2}",
                label.0,
                telemetry.true_airspeed_mps,
                telemetry.vertical_speed_mps,
                telemetry.angle_of_attack_deg,
                telemetry.sideslip_deg,
                telemetry.throttle.max(telemetry.collective),
            )
        })
        .collect::<Vec<_>>();
    rows.sort();

    let mut lines = vec![
        "Control Profiles Example".to_string(),
        "Compare the calmer trainer/utility presets against the faster arcade presets.".to_string(),
        String::new(),
    ];
    lines.extend(rows);
    text.0 = lines.join("\n");
}
