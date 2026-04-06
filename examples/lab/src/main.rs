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
    ExamplePilot, configure_example_app, draw_force_vectors, spawn_fixed_wing_demo,
    spawn_helicopter_demo, spawn_lights_ground_and_camera, spawn_overlay, spawn_vtol_demo,
};

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub struct LabState {
    pub plane: Entity,
    pub helicopter: Entity,
    pub vtol: Entity,
    pub active: ActiveVehicle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveVehicle {
    FixedWing,
    Helicopter,
    Vtol,
}

#[derive(Component)]
struct LabSwitcher;

#[derive(Debug, InputAction)]
#[action_output(bool)]
struct SelectFixedWingAction;

#[derive(Debug, InputAction)]
#[action_output(bool)]
struct SelectHelicopterAction;

#[derive(Debug, InputAction)]
#[action_output(bool)]
struct SelectVtolAction;

fn main() {
    let mut app = App::new();
    configure_example_app(&mut app);
    app.add_input_context::<LabSwitcher>()
        .add_observer(select_fixed_wing)
        .add_observer(select_helicopter)
        .add_observer(select_vtol);
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
    let vtol = spawn_vtol_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        "Lab Tiltrotor",
        Transform::from_xyz(28.0, 2.1, -18.0).with_rotation(Quat::from_rotation_y(-0.22)),
        Vec3::new(0.0, 0.0, -6.0),
        0.64,
        0.20,
        true,
    );

    commands
        .entity(plane)
        .insert(ContextActivity::<ExamplePilot>::ACTIVE);
    commands
        .entity(helicopter)
        .insert(ContextActivity::<ExamplePilot>::INACTIVE);
    commands
        .entity(vtol)
        .insert(ContextActivity::<ExamplePilot>::INACTIVE);
    commands.insert_resource(LabState {
        plane,
        helicopter,
        vtol,
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
            ),
            (
                Action::<SelectVtolAction>::new(),
                InputPress::default(),
                bindings![KeyCode::Digit3],
            )
        ]),
    ));

    spawn_overlay(&mut commands, "Flight Lab");
}

fn select_fixed_wing(
    _: On<Start<SelectFixedWingAction>>,
    mut commands: Commands,
    mut state: ResMut<LabState>,
) {
    set_active_vehicle(&mut commands, &mut state, ActiveVehicle::FixedWing);
}

fn select_helicopter(
    _: On<Start<SelectHelicopterAction>>,
    mut commands: Commands,
    mut state: ResMut<LabState>,
) {
    set_active_vehicle(&mut commands, &mut state, ActiveVehicle::Helicopter);
}

fn select_vtol(
    _: On<Start<SelectVtolAction>>,
    mut commands: Commands,
    mut state: ResMut<LabState>,
) {
    set_active_vehicle(&mut commands, &mut state, ActiveVehicle::Vtol);
}

fn set_active_vehicle(commands: &mut Commands, state: &mut LabState, active: ActiveVehicle) {
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
    commands
        .entity(state.vtol)
        .insert(if state.active == ActiveVehicle::Vtol {
            ContextActivity::<ExamplePilot>::ACTIVE
        } else {
            ContextActivity::<ExamplePilot>::INACTIVE
        });
    // Camera target is automatically switched by sync_camera_target in the support crate
}

fn update_overlay(
    state: Res<LabState>,
    mut overlay: Query<&mut Text, (With<Node>, Without<ExamplePilot>)>,
    telemetry: Query<(&FlightTelemetry, &FlightAeroState, &FlightForces)>,
) {
    let Ok(mut overlay) = overlay.single_mut() else {
        return;
    };
    let Ok((plane_telemetry, plane_aero, plane_forces)) = telemetry.get(state.plane) else {
        return;
    };
    let Ok((heli_telemetry, heli_aero, heli_forces)) = telemetry.get(state.helicopter) else {
        return;
    };
    let Ok((vtol_telemetry, vtol_aero, vtol_forces)) = telemetry.get(state.vtol) else {
        return;
    };

    overlay.0 = format!(
        "Flight Lab\nActive craft: {:?}  (1 fixed-wing, 2 helicopter, 3 VTOL)\n\nFixed-wing\n  TAS {:>6.1}  Alt {:>6.1}  AoA {:>5.1}  Stall {}\n  q {:>7.1}  Fwd {:>4.2}  Gear {:>4.2}\n  Lift {:>7.1}N  Thrust {:>7.1}N\n\nHelicopter\n  TAS {:>6.1}  Alt {:>6.1}  Slip {:>5.1}  Vertical {:>5.1}\n  q {:>7.1}  Vert {:>4.2}  Gear {:>4.2}\n  Lift {:>7.1}N  Torque {:>7.1}Nm\n\nVTOL\n  TAS {:>6.1}  Alt {:>6.1}  AoA {:>5.1}  Transition {:>4.2}\n  q {:>7.1}  Fwd {:>4.2}  Vert {:>4.2}  Gear {:>4.2}\n  Lift {:>7.1}N  Thrust {:>7.1}N\n\nPilot controls: arrows pitch/roll, Q/E yaw, [/] mapped power channels, ,/. mapped VTOL transition, G gear.\nBRP targets: Lab Bush Plane, Lab Utility Helicopter, Lab Tiltrotor, Example Camera.",
        state.active,
        plane_telemetry.true_airspeed_mps,
        plane_telemetry.altitude_msl_m,
        plane_telemetry.angle_of_attack_deg,
        plane_telemetry.stalled,
        plane_aero.dynamic_pressure_pa,
        plane_telemetry.forward_thrust,
        plane_telemetry.gear_position,
        plane_forces.lift_world_newtons.length(),
        plane_forces.thrust_world_newtons.length(),
        heli_telemetry.true_airspeed_mps,
        heli_telemetry.altitude_msl_m,
        heli_telemetry.sideslip_deg,
        heli_telemetry.vertical_speed_mps,
        heli_aero.dynamic_pressure_pa,
        heli_telemetry.vertical_thrust,
        heli_telemetry.gear_position,
        heli_forces.lift_world_newtons.length(),
        heli_forces.total_torque_body_nm.length(),
        vtol_telemetry.true_airspeed_mps,
        vtol_telemetry.altitude_msl_m,
        vtol_telemetry.angle_of_attack_deg,
        vtol_telemetry.vtol_transition,
        vtol_aero.dynamic_pressure_pa,
        vtol_telemetry.forward_thrust,
        vtol_telemetry.vertical_thrust,
        vtol_telemetry.gear_position,
        vtol_forces.lift_world_newtons.length(),
        vtol_forces.thrust_world_newtons.length(),
    );
}
