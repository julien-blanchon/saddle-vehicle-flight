#[cfg(feature = "e2e")]
mod e2e;
#[cfg(feature = "e2e")]
mod scenarios;
use saddle_vehicle_flight_example_support as support;

use bevy::prelude::*;
#[cfg(all(feature = "dev", not(target_arch = "wasm32")))]
use bevy::remote::{RemotePlugin, http::RemoteHttpPlugin};
#[cfg(all(feature = "dev", not(target_arch = "wasm32")))]
use bevy_brp_extras::BrpExtrasPlugin;
use bevy_enhanced_input::{
    context::InputContextAppExt,
    prelude::{
        Action, ContextActivity, InputAction, Press as InputPress, Start, actions, bindings,
    },
};
use saddle_vehicle_flight::{FlightAeroState, FlightForces, FlightTelemetry};
use support::{
    ExamplePilot, FollowCamera, configure_example_app, draw_force_vectors, spawn_fixed_wing_demo,
    spawn_helicopter_demo, spawn_lights_ground_and_camera, spawn_overlay,
};

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub struct LabState {
    pub plane: Entity,
    pub helicopter: Entity,
    pub active: ActiveVehicle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveVehicle {
    FixedWing,
    Helicopter,
}

#[derive(Component)]
struct LabSwitcher;

#[derive(Debug, InputAction)]
#[action_output(bool)]
struct SelectFixedWingAction;

#[derive(Debug, InputAction)]
#[action_output(bool)]
struct SelectHelicopterAction;

fn main() {
    let mut app = App::new();
    configure_example_app(&mut app);
    app.add_input_context::<LabSwitcher>()
        .add_observer(select_fixed_wing)
        .add_observer(select_helicopter);
    #[cfg(all(feature = "dev", not(target_arch = "wasm32")))]
    app.add_plugins((
        RemotePlugin::default(),
        BrpExtrasPlugin::with_http_plugin(RemoteHttpPlugin::default()),
    ));
    #[cfg(feature = "e2e")]
    app.add_plugins(e2e::FlightLabE2EPlugin);

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
    let plane = spawn_fixed_wing_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Lab Bush Plane",
        Transform::from_xyz(0.0, 1.16, 58.0),
        Vec3::new(0.0, 0.0, -24.0),
        0.62,
        true,
    );
    let helicopter = spawn_helicopter_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Lab Utility Helicopter",
        Transform::from_xyz(28.0, 2.6, 0.0),
        Vec3::ZERO,
        0.60,
        true,
    );

    commands
        .entity(plane)
        .insert(ContextActivity::<ExamplePilot>::ACTIVE);
    commands
        .entity(helicopter)
        .insert(ContextActivity::<ExamplePilot>::INACTIVE);
    commands.insert_resource(LabState {
        plane,
        helicopter,
        active: ActiveVehicle::FixedWing,
    });
    commands.spawn((
        Name::new("Lab Input"),
        LabSwitcher,
        actions!(LabSwitcher[
            (
                Action::<SelectFixedWingAction>::new(),
                InputPress::default(),
                bindings![KeyCode::Digit1],
            ),
            (
                Action::<SelectHelicopterAction>::new(),
                InputPress::default(),
                bindings![KeyCode::Digit2],
            )
        ]),
    ));

    spawn_overlay(&mut commands, "Flight Lab");
}

fn select_fixed_wing(
    _: On<Start<SelectFixedWingAction>>,
    mut commands: Commands,
    mut state: ResMut<LabState>,
    mut cameras: Query<&mut FollowCamera>,
) {
    set_active_vehicle(
        &mut commands,
        &mut state,
        &mut cameras,
        ActiveVehicle::FixedWing,
    );
}

fn select_helicopter(
    _: On<Start<SelectHelicopterAction>>,
    mut commands: Commands,
    mut state: ResMut<LabState>,
    mut cameras: Query<&mut FollowCamera>,
) {
    set_active_vehicle(
        &mut commands,
        &mut state,
        &mut cameras,
        ActiveVehicle::Helicopter,
    );
}

fn set_active_vehicle(
    commands: &mut Commands,
    state: &mut LabState,
    cameras: &mut Query<&mut FollowCamera>,
    active: ActiveVehicle,
) {
    if state.active == active {
        return;
    }

    state.active = active;
    commands
        .entity(state.plane)
        .insert(if state.active == ActiveVehicle::FixedWing {
            ContextActivity::<ExamplePilot>::ACTIVE
        } else {
            ContextActivity::<ExamplePilot>::INACTIVE
        });
    commands
        .entity(state.helicopter)
        .insert(if state.active == ActiveVehicle::Helicopter {
            ContextActivity::<ExamplePilot>::ACTIVE
        } else {
            ContextActivity::<ExamplePilot>::INACTIVE
        });

    if let Ok(mut camera) = cameras.single_mut() {
        match state.active {
            ActiveVehicle::FixedWing => {
                camera.distance = 18.0;
                camera.height = 6.0;
                camera.lateral_offset = 0.0;
            }
            ActiveVehicle::Helicopter => {
                camera.distance = 15.0;
                camera.height = 5.5;
                camera.lateral_offset = -2.0;
            }
        }
    }
}

fn update_overlay(
    state: Res<LabState>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ExamplePilot>)>,
    plane: Query<
        (&FlightTelemetry, &FlightAeroState, &FlightForces),
        With<saddle_vehicle_flight::FixedWingAircraft>,
    >,
    helicopter: Query<
        (&FlightTelemetry, &FlightAeroState, &FlightForces),
        With<saddle_vehicle_flight::HelicopterAircraft>,
    >,
) {
    let Ok(mut overlay) = overlay.single_mut() else {
        return;
    };
    let Ok((plane_telemetry, plane_aero, plane_forces)) = plane.single() else {
        return;
    };
    let Ok((heli_telemetry, heli_aero, heli_forces)) = helicopter.single() else {
        return;
    };

    overlay.0 = format!(
        "Flight Lab\nActive craft: {:?}  (1 fixed-wing, 2 helicopter)\n\nFixed-wing\n  TAS {:>6.1}  Alt {:>6.1}  AoA {:>5.1}  Stall {}\n  q {:>7.1}  Throttle {:>4.2}  Gear {:>4.2}\n  Lift {:>7.1}N  Thrust {:>7.1}N\n\nHelicopter\n  TAS {:>6.1}  Alt {:>6.1}  Slip {:>5.1}  Vertical {:>5.1}\n  q {:>7.1}  Collective {:>4.2}  Gear {:>4.2}\n  Lift {:>7.1}N  Torque {:>7.1}Nm\n\nPilot controls: arrows pitch/roll, Q/E yaw, [/] throttle or collective, G gear.\nBRP targets: Lab Bush Plane, Lab Utility Helicopter, Example Camera.",
        state.active,
        plane_telemetry.true_airspeed_mps,
        plane_telemetry.altitude_msl_m,
        plane_telemetry.angle_of_attack_deg,
        plane_telemetry.stalled,
        plane_aero.dynamic_pressure_pa,
        plane_telemetry.throttle,
        plane_telemetry.gear_position,
        plane_forces.lift_world_newtons.length(),
        plane_forces.thrust_world_newtons.length(),
        heli_telemetry.true_airspeed_mps,
        heli_telemetry.altitude_msl_m,
        heli_telemetry.sideslip_deg,
        heli_telemetry.vertical_speed_mps,
        heli_aero.dynamic_pressure_pa,
        heli_telemetry.collective,
        heli_telemetry.gear_position,
        heli_forces.lift_world_newtons.length(),
        heli_forces.total_torque_body_nm.length(),
    );
}
