use bevy::color::palettes::css;
use bevy::prelude::*;
use bevy_enhanced_input::context::InputContextAppExt;
use bevy_enhanced_input::prelude::{
    Action, Bidirectional, Bindings, Cancel as InputCancel, Complete, ContextActivity, Fire,
    InputAction, Press as InputPress, Start, actions, bindings,
};
use saddle_camera_third_person_camera::{
    AutoRecenterSettings, CollisionSettings, FollowAlignment, OrbitSettings, SmoothingSettings,
    ThirdPersonCamera, ThirdPersonCameraPlugin, ThirdPersonCameraSettings, ThirdPersonCameraTarget,
    ZoomSettings,
};
use saddle_pane::prelude::*;
use saddle_vehicle_flight::{
    ControlInputSource, FlightAeroState, FlightAssist, FlightControlInput, FlightEnvironment,
    FlightForces, FlightPlugin, FlightTelemetry, VehicleControlMap,
};
use saddle_vehicle_flight_example_presets::{
    fixed_wing_trainer, rotorcraft_utility, tiltrotor_transport,
};

#[derive(Resource, Clone, Pane)]
#[pane(title = "Flight Tuning")]
pub struct FlightExamplePane {
    #[pane(slider, min = -20.0, max = 20.0, step = 0.5)]
    pub wind_x_mps: f32,
    #[pane(slider, min = -20.0, max = 20.0, step = 0.5)]
    pub wind_z_mps: f32,
    #[pane(slider, min = 0.5, max = 1.5, step = 0.01)]
    pub density_multiplier: f32,
    #[pane(slider, min = 0.0, max = 1.0, step = 0.02)]
    pub wings_leveling: f32,
    #[pane(slider, min = 0.0, max = 1.0, step = 0.02)]
    pub coordinated_turn: f32,
    #[pane(slider, min = 0.0, max = 1.0, step = 0.02)]
    pub hover_leveling: f32,
}

impl Default for FlightExamplePane {
    fn default() -> Self {
        Self {
            wind_x_mps: 0.0,
            wind_z_mps: 0.0,
            density_multiplier: 1.0,
            wings_leveling: 0.18,
            coordinated_turn: 0.16,
            hover_leveling: 0.20,
        }
    }
}

#[derive(Resource, Default, Clone, Pane)]
#[pane(title = "Flight Telemetry")]
pub struct FlightExampleStats {
    #[pane(monitor)]
    pub true_airspeed_mps: f32,
    #[pane(monitor)]
    pub altitude_msl_m: f32,
    #[pane(monitor)]
    pub angle_of_attack_deg: f32,
    #[pane(monitor)]
    pub vtol_transition: f32,
}

pub fn pane_plugins() -> (
    bevy_flair::FlairPlugin,
    bevy_input_focus::InputDispatchPlugin,
    bevy_ui_widgets::UiWidgetsPlugins,
    bevy_input_focus::tab_navigation::TabNavigationPlugin,
    saddle_pane::PanePlugin,
) {
    (
        bevy_flair::FlairPlugin,
        bevy_input_focus::InputDispatchPlugin,
        bevy_ui_widgets::UiWidgetsPlugins,
        bevy_input_focus::tab_navigation::TabNavigationPlugin,
        saddle_pane::PanePlugin,
    )
}

#[derive(Component)]
pub struct ExamplePilot;

#[derive(Component)]
pub struct RotorDisk {
    pub speed_rps: f32,
}

#[derive(Debug, InputAction)]
#[action_output(f32)]
pub struct PitchAction;

#[derive(Debug, InputAction)]
#[action_output(f32)]
pub struct RollAction;

#[derive(Debug, InputAction)]
#[action_output(f32)]
pub struct YawAction;

#[derive(Debug, InputAction)]
#[action_output(f32)]
pub struct PowerAdjustAction;

#[derive(Debug, InputAction)]
#[action_output(f32)]
pub struct TransitionAdjustAction;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct GearToggleAction;

pub fn configure_example_app(app: &mut App) {
    configure_example_app_with_follow_camera(app, true);
}

pub fn configure_example_app_with_follow_camera(app: &mut App, enable_follow_camera: bool) {
    app.add_plugins((
        DefaultPlugins,
        FlightPlugin::default(),
        ThirdPersonCameraPlugin::default(),
        pane_plugins(),
    ))
    .insert_resource(FlightExamplePane::default())
    .insert_resource(FlightExampleStats::default())
    .register_pane::<FlightExamplePane>()
    .register_pane::<FlightExampleStats>();
    if !app.is_plugin_added::<bevy_enhanced_input::prelude::EnhancedInputPlugin>() {
        app.add_plugins(bevy_enhanced_input::prelude::EnhancedInputPlugin);
    }
    app.add_input_context::<ExamplePilot>()
        .insert_resource(ClearColor(Color::srgb(0.59, 0.75, 0.94)))
        .add_observer(apply_pitch)
        .add_observer(clear_pitch_on_cancel)
        .add_observer(clear_pitch_on_complete)
        .add_observer(apply_roll)
        .add_observer(clear_roll_on_cancel)
        .add_observer(clear_roll_on_complete)
        .add_observer(apply_yaw)
        .add_observer(clear_yaw_on_cancel)
        .add_observer(clear_yaw_on_complete)
        .add_observer(adjust_power)
        .add_observer(adjust_transition)
        .add_observer(trigger_gear_toggle)
        .add_systems(
            Update,
            (spin_rotors, sync_pane_to_runtime, update_pane_stats).chain(),
        )
        .add_systems(PostUpdate, clear_one_shot_controls);
    if enable_follow_camera {
        app.add_systems(Update, sync_camera_target);
    }
}

pub fn pilot_actions() -> impl Bundle {
    (
        ExamplePilot,
        actions!(ExamplePilot[
            (
                Action::<PitchAction>::new(),
                Bindings::spawn(Bidirectional::new(KeyCode::ArrowDown, KeyCode::ArrowUp)),
            ),
            (
                Action::<RollAction>::new(),
                Bindings::spawn(Bidirectional::new(KeyCode::ArrowLeft, KeyCode::ArrowRight)),
            ),
            (
                Action::<YawAction>::new(),
                Bindings::spawn(Bidirectional::new(KeyCode::KeyQ, KeyCode::KeyE)),
            ),
            (
                Action::<PowerAdjustAction>::new(),
                Bindings::spawn(Bidirectional::new(KeyCode::BracketLeft, KeyCode::BracketRight)),
            ),
            (
                Action::<TransitionAdjustAction>::new(),
                Bindings::spawn(Bidirectional::new(KeyCode::Comma, KeyCode::Period)),
            ),
            (
                Action::<GearToggleAction>::new(),
                InputPress::default(),
                bindings![KeyCode::KeyG],
            ),
        ]),
    )
}

pub fn spawn_lights_and_ground(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    commands.spawn((
        Name::new("Example Sun"),
        DirectionalLight {
            illuminance: 22_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(18.0, 30.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("Fill Light"),
        PointLight {
            intensity: 35_000.0,
            range: 120.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(-24.0, 14.0, 18.0),
    ));

    commands.spawn((
        Name::new("Runway Ground"),
        Mesh3d(meshes.add(Plane3d::default().mesh().size(280.0, 280.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.28, 0.39, 0.28),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));
    commands.spawn((
        Name::new("Runway Strip"),
        Mesh3d(meshes.add(Cuboid::new(14.0, 0.02, 180.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.14, 0.15, 0.17),
            perceptual_roughness: 0.96,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.01, -20.0),
    ));
    commands.spawn((
        Name::new("Helipad"),
        Mesh3d(meshes.add(Cylinder::new(12.0, 0.05))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.19, 0.20, 0.22),
            perceptual_roughness: 0.88,
            ..default()
        })),
        Transform::from_xyz(28.0, 0.025, 0.0),
    ));

    commands.spawn((
        Name::new("Runway Wind Sock"),
        Mesh3d(meshes.add(Cuboid::new(0.18, 3.2, 0.18))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.74, 0.74, 0.78),
            perceptual_roughness: 0.78,
            ..default()
        })),
        Transform::from_xyz(18.0, 1.6, -34.0),
    ));
}

pub fn spawn_lights_ground_and_camera(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    spawn_lights_and_ground(commands, meshes, materials);
    // Spawn a chase camera using saddle-camera-third-person-camera
    commands.spawn((
        Name::new("Example Camera"),
        ThirdPersonCamera::new(18.0, 0.0, -0.32),
        ThirdPersonCameraSettings {
            orbit: OrbitSettings {
                yaw_speed: 1.2,
                pitch_speed: 1.1,
                min_pitch: -1.45,
                max_pitch: 0.10,
                ..default()
            },
            smoothing: SmoothingSettings {
                orientation_smoothing: 8.0,
                target_follow_smoothing: 6.0,
                zoom_smoothing: 12.0,
                ..default()
            },
            zoom: ZoomSettings {
                min_distance: 6.0,
                max_distance: 45.0,
                default_distance: 18.0,
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
        Transform::from_xyz(-15.0, 8.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

pub fn spawn_fixed_wing_demo(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    name: &str,
    transform: Transform,
    initial_velocity_world_mps: Vec3,
    throttle: f32,
    pilot_controlled: bool,
) -> Entity {
    let preset = fixed_wing_trainer();
    let mut entity = commands.spawn((
        Name::new(name.to_string()),
        preset.model,
        preset.actuators,
        preset.control_map,
        preset.ground_handling,
        preset.body,
        preset.assist,
        saddle_vehicle_flight::FlightKinematics {
            linear_velocity_world_mps: initial_velocity_world_mps,
            ..default()
        },
        FlightControlInput {
            throttle,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        Mesh3d(meshes.add(Cuboid::new(1.0, 0.55, 5.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.82, 0.21, 0.19),
            perceptual_roughness: 0.48,
            metallic: 0.08,
            ..default()
        })),
        transform,
    ));

    if pilot_controlled {
        entity.insert(pilot_actions());
    }

    let entity_id = entity.id();
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

    entity_id
}

pub fn spawn_helicopter_demo(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    name: &str,
    transform: Transform,
    initial_velocity_world_mps: Vec3,
    collective: f32,
    pilot_controlled: bool,
) -> Entity {
    let preset = rotorcraft_utility();
    let mut entity = commands.spawn((
        Name::new(name.to_string()),
        preset.model,
        preset.actuators,
        preset.control_map,
        preset.ground_handling,
        preset.body,
        preset.assist,
        saddle_vehicle_flight::FlightKinematics {
            linear_velocity_world_mps: initial_velocity_world_mps,
            ..default()
        },
        FlightControlInput {
            collective,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        Mesh3d(meshes.add(Capsule3d::new(0.65, 2.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.17, 0.46, 0.73),
            perceptual_roughness: 0.42,
            metallic: 0.06,
            ..default()
        })),
        transform,
    ));

    if pilot_controlled {
        entity.insert(pilot_actions());
    }

    let entity_id = entity.id();
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

    entity_id
}

pub fn spawn_vtol_demo(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    name: &str,
    transform: Transform,
    initial_velocity_world_mps: Vec3,
    power: f32,
    transition: f32,
    pilot_controlled: bool,
) -> Entity {
    let preset = tiltrotor_transport();
    let mut entity = commands.spawn((
        Name::new(name.to_string()),
        preset.model,
        preset.actuators,
        preset.control_map,
        preset.ground_handling,
        preset.body,
        preset.assist,
        saddle_vehicle_flight::FlightKinematics {
            linear_velocity_world_mps: initial_velocity_world_mps,
            ..default()
        },
        FlightControlInput {
            throttle: power,
            collective: power,
            vtol_transition: transition,
            ..default()
        },
        FlightEnvironment {
            surface_altitude_msl_m: Some(0.0),
            ..default()
        },
        Mesh3d(meshes.add(Cuboid::new(2.2, 1.4, 8.4))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.22, 0.35, 0.42),
            perceptual_roughness: 0.46,
            metallic: 0.12,
            ..default()
        })),
        transform,
    ));

    if pilot_controlled {
        entity.insert(pilot_actions());
    }

    let entity_id = entity.id();
    entity.with_children(|parent| {
        parent.spawn((
            Name::new("Vtol Wing"),
            Mesh3d(meshes.add(Cuboid::new(16.0, 0.28, 2.2))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.75, 0.80, 0.84),
                perceptual_roughness: 0.56,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.18, 0.2),
        ));
        parent.spawn((
            Name::new("Vtol Tailplane"),
            Mesh3d(meshes.add(Cuboid::new(5.4, 0.22, 1.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.75, 0.80, 0.84),
                perceptual_roughness: 0.56,
                ..default()
            })),
            Transform::from_xyz(0.0, 1.0, 3.2),
        ));
        parent.spawn((
            Name::new("Vtol Left Nacelle"),
            Mesh3d(meshes.add(Capsule3d::new(0.45, 1.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.16, 0.18, 0.21),
                perceptual_roughness: 0.42,
                ..default()
            })),
            Transform::from_xyz(-6.2, 0.48, -0.1)
                .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
        ));
        parent.spawn((
            Name::new("Vtol Right Nacelle"),
            Mesh3d(meshes.add(Capsule3d::new(0.45, 1.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.16, 0.18, 0.21),
                perceptual_roughness: 0.42,
                ..default()
            })),
            Transform::from_xyz(6.2, 0.48, -0.1)
                .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
        ));
        parent.spawn((
            Name::new("Left Rotor"),
            RotorDisk { speed_rps: 18.0 },
            Mesh3d(meshes.add(Cuboid::new(0.22, 0.08, 8.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.08, 0.08, 0.09),
                perceptual_roughness: 0.30,
                ..default()
            })),
            Transform::from_xyz(-6.2, 1.3, -0.1),
        ));
        parent.spawn((
            Name::new("Right Rotor"),
            RotorDisk { speed_rps: 18.0 },
            Mesh3d(meshes.add(Cuboid::new(0.22, 0.08, 8.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.08, 0.08, 0.09),
                perceptual_roughness: 0.30,
                ..default()
            })),
            Transform::from_xyz(6.2, 1.3, -0.1),
        ));
        parent.spawn((
            Name::new("Cockpit Canopy"),
            Mesh3d(meshes.add(Capsule3d::new(0.78, 2.2))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.58, 0.74, 0.86, 0.32),
                perceptual_roughness: 0.06,
                metallic: 0.12,
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.72, -1.4)
                .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        ));
    });

    entity_id
}

pub fn spawn_overlay(commands: &mut Commands, title: &str) {
    commands.spawn((
        Name::new("Telemetry Overlay"),
        Text::new(title.to_string()),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(18.0),
            top: Val::Px(18.0),
            width: Val::Px(460.0),
            padding: UiRect::all(Val::Px(12.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.05, 0.06, 0.08, 0.78)),
    ));
}

/// Points the `ThirdPersonCamera`'s target at the currently active pilot.
fn sync_camera_target(
    mut commands: Commands,
    pilots: Query<(Entity, Option<&ContextActivity<ExamplePilot>>), With<ExamplePilot>>,
    cameras: Query<(Entity, Option<&ThirdPersonCameraTarget>), With<ThirdPersonCamera>>,
) {
    // Find the active pilot entity
    let active_pilot = pilots
        .iter()
        .find(|(_, activity)| activity.is_some_and(|a| **a))
        .or_else(|| pilots.iter().next());

    let Some((pilot_id, _)) = active_pilot else {
        return;
    };

    // Find the camera and update its target
    let Some((camera_id, existing_target)) = cameras.iter().next() else {
        return;
    };

    // Skip if already targeting the right entity
    if existing_target.is_some_and(|t| t.target == pilot_id) {
        return;
    }

    commands
        .entity(camera_id)
        .insert(ThirdPersonCameraTarget::new(pilot_id));
}

fn sync_pane_to_runtime(
    pane: Res<FlightExamplePane>,
    mut active_aircraft: Query<(
        &mut FlightEnvironment,
        &mut FlightAssist,
        Option<&ContextActivity<ExamplePilot>>,
    )>,
) {
    if !pane.is_changed() {
        return;
    }

    let has_active = active_aircraft
        .iter()
        .any(|(_, _, activity)| activity.is_some_and(|activity| **activity));
    let selected = if has_active {
        active_aircraft
            .iter_mut()
            .find(|(_, _, activity)| activity.is_some_and(|activity| **activity))
    } else {
        active_aircraft.iter_mut().next()
    };
    let Some((mut environment, mut assist, _)) = selected else {
        return;
    };

    environment.wind_world_mps.x = pane.wind_x_mps;
    environment.wind_world_mps.z = pane.wind_z_mps;
    environment.density_multiplier = pane.density_multiplier;

    assist.wings_leveling = pane.wings_leveling;
    assist.coordinated_turn = pane.coordinated_turn;
    assist.hover_leveling = pane.hover_leveling;
}

fn update_pane_stats(
    mut stats: ResMut<FlightExampleStats>,
    active_aircraft: Query<(
        &FlightTelemetry,
        &FlightAeroState,
        Option<&ContextActivity<ExamplePilot>>,
    )>,
) {
    let active_first = active_aircraft
        .iter()
        .find(|(_, _, activity)| activity.is_some_and(|activity| **activity));
    let Some((telemetry, aero, _)) = active_first.or_else(|| active_aircraft.iter().next()) else {
        return;
    };

    stats.true_airspeed_mps = telemetry.true_airspeed_mps;
    stats.altitude_msl_m = telemetry.altitude_msl_m;
    stats.angle_of_attack_deg = aero.angle_of_attack_rad.to_degrees();
    stats.vtol_transition = telemetry.vtol_transition;
}

pub fn update_single_overlay(
    title: &str,
    label: &str,
    telemetry: &saddle_vehicle_flight::FlightTelemetry,
    aero: &saddle_vehicle_flight::FlightAeroState,
    text: &mut Text,
) {
    text.0 = format!(
        "{title}\n{label}\n\nTAS {:>6.1} m/s  IAS {:>6.1}\nAlt {:>6.1} m  AGL {:>6.1?}\nV/S {:>6.1}  AoA {:>6.1} deg\nSlip {:>6.1} deg  q {:>7.1} Pa\nFwd {:>4.2}  Vert {:>4.2}  Lat {:>4.2}  VTOL {:>4.2}\nGear {:>4.2} {}  Stalled {}\n\nControls:\n  Arrow keys: pitch/roll\n  Q/E: yaw\n  [ / ]: mapped power channels\n  , / .: mapped transition channel\n  G: toggle gear\n  Mouse: orbit camera\n  Scroll: zoom",
        telemetry.true_airspeed_mps,
        telemetry.indicated_airspeed_mps,
        telemetry.altitude_msl_m,
        telemetry.altitude_agl_m,
        telemetry.vertical_speed_mps,
        telemetry.angle_of_attack_deg,
        telemetry.sideslip_deg,
        aero.dynamic_pressure_pa,
        telemetry.forward_thrust,
        telemetry.vertical_thrust,
        telemetry.lateral_thrust,
        telemetry.vtol_transition,
        telemetry.gear_position,
        if telemetry.gear_deployed {
            "down"
        } else {
            "up"
        },
        telemetry.stalled,
    );
}

pub fn draw_force_vectors(
    mut gizmos: Gizmos,
    query: Query<(&Transform, &FlightForces), With<ExamplePilot>>,
) {
    for (transform, forces) in &query {
        let origin = transform.translation + Vec3::Y * 0.6;
        draw_vector(
            &mut gizmos,
            origin,
            forces.lift_world_newtons / 2_000.0,
            css::AQUA,
        );
        draw_vector(
            &mut gizmos,
            origin,
            forces.drag_world_newtons / 2_000.0,
            css::ORANGE_RED,
        );
        draw_vector(
            &mut gizmos,
            origin,
            forces.thrust_world_newtons / 2_000.0,
            css::LIMEGREEN,
        );
    }
}

fn draw_vector(gizmos: &mut Gizmos, origin: Vec3, vector: Vec3, color: impl Into<Color>) {
    if vector.length_squared() > 1e-6 {
        gizmos.arrow(origin, origin + vector, color);
    }
}

fn apply_pitch(
    trigger: On<Fire<PitchAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.pitch = trigger.value;
    }
}

fn clear_pitch_on_cancel(
    trigger: On<InputCancel<PitchAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.pitch = 0.0;
    }
}

fn clear_pitch_on_complete(
    trigger: On<Complete<PitchAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.pitch = 0.0;
    }
}

fn apply_roll(
    trigger: On<Fire<RollAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.roll = trigger.value;
    }
}

fn clear_roll_on_cancel(
    trigger: On<InputCancel<RollAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.roll = 0.0;
    }
}

fn clear_roll_on_complete(
    trigger: On<Complete<RollAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.roll = 0.0;
    }
}

fn apply_yaw(
    trigger: On<Fire<YawAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.yaw = trigger.value;
    }
}

fn clear_yaw_on_cancel(
    trigger: On<InputCancel<YawAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.yaw = 0.0;
    }
}

fn clear_yaw_on_complete(
    trigger: On<Complete<YawAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.yaw = 0.0;
    }
}

fn adjust_power(
    trigger: On<Fire<PowerAdjustAction>>,
    time: Res<Time>,
    mut query: Query<(&mut FlightControlInput, &VehicleControlMap), With<ExamplePilot>>,
) {
    let Ok((mut input, control_map)) = query.get_mut(trigger.context) else {
        return;
    };
    let delta = trigger.value * time.delta_secs() * 0.6;
    let mut changed = false;
    changed |= apply_delta_to_source(&mut input, control_map.forward_thrust.source, delta);
    changed |= apply_delta_to_source(&mut input, control_map.vertical_thrust.source, delta);
    if !changed {
        input.throttle = (input.throttle + delta).clamp(0.0, 1.0);
    }
}

fn adjust_transition(
    trigger: On<Fire<TransitionAdjustAction>>,
    time: Res<Time>,
    mut query: Query<(&mut FlightControlInput, &VehicleControlMap), With<ExamplePilot>>,
) {
    let Ok((mut input, control_map)) = query.get_mut(trigger.context) else {
        return;
    };
    if !matches!(
        control_map.transition.source,
        ControlInputSource::Transition
    ) {
        return;
    }

    let delta = trigger.value * time.delta_secs() * 0.45;
    input.vtol_transition = (input.vtol_transition + delta).clamp(0.0, 1.0);
}

fn trigger_gear_toggle(
    trigger: On<Start<GearToggleAction>>,
    mut query: Query<&mut FlightControlInput, With<ExamplePilot>>,
) {
    if let Ok(mut input) = query.get_mut(trigger.context) {
        input.toggle_gear = trigger.value;
    }
}

fn clear_one_shot_controls(mut query: Query<&mut FlightControlInput, With<ExamplePilot>>) {
    for mut input in &mut query {
        input.toggle_gear = false;
        input.requested_gear_down = None;
    }
}

fn spin_rotors(time: Res<Time>, mut query: Query<(&RotorDisk, &mut Transform)>) {
    for (rotor, mut transform) in &mut query {
        transform.rotate_y(rotor.speed_rps * std::f32::consts::TAU * time.delta_secs());
    }
}

fn apply_delta_to_source(
    input: &mut FlightControlInput,
    source: ControlInputSource,
    delta: f32,
) -> bool {
    let current = source.sample(*input);
    source.write(input, current + delta)
}
