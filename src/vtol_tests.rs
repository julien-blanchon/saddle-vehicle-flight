use bevy::prelude::*;

use crate::{
    AtmosphereSample, FlightAeroState, FlightBody, FlightControlInput, FlightKinematics,
    FlightPlugin, FlightTelemetry, LandingGearState, VtolAircraft,
    components::ResolvedFlightControls,
    model::{common::sample_motion, vtol::evaluate_vtol},
};

fn sample_transform() -> Transform {
    Transform::IDENTITY
}

#[test]
fn vtol_hover_mode_produces_upward_thrust() {
    let transform = sample_transform();
    let motion = sample_motion(
        &transform,
        &FlightKinematics::default(),
        &crate::FlightEnvironment::default(),
    );
    let evaluated = evaluate_vtol(
        motion,
        &transform,
        VtolAircraft::tiltrotor_transport_body(),
        VtolAircraft::tiltrotor_transport(),
        ResolvedFlightControls {
            collective: 0.78,
            transition: 0.0,
            ..default()
        },
        FlightControlInput {
            collective: 0.78,
            vtol_transition: 0.0,
            ..default()
        },
        crate::FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
    );

    assert!(evaluated.forces.thrust_world_newtons.y > 0.0);
}

#[test]
fn vtol_forward_mode_points_thrust_along_forward_axis() {
    let transform = sample_transform();
    let motion = sample_motion(
        &transform,
        &FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, 0.0, -55.0),
            ..default()
        },
        &crate::FlightEnvironment::default(),
    );
    let evaluated = evaluate_vtol(
        motion,
        &transform,
        VtolAircraft::tiltrotor_transport_body(),
        VtolAircraft::tiltrotor_transport(),
        ResolvedFlightControls {
            throttle: 0.82,
            transition: 1.0,
            ..default()
        },
        FlightControlInput {
            throttle: 0.82,
            vtol_transition: 1.0,
            ..default()
        },
        crate::FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
    );

    assert!(evaluated.forces.thrust_world_newtons.z < 0.0);
    assert!(
        evaluated.forces.thrust_world_newtons.y.abs()
            < evaluated.forces.thrust_world_newtons.z.abs()
    );
}

#[test]
fn telemetry_exposes_vtol_transition_state() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(FlightPlugin::always_on(Update));
    app.world_mut().spawn((
        VtolAircraft::tiltrotor_transport(),
        FlightBody {
            use_internal_integration: false,
            ..VtolAircraft::tiltrotor_transport_body()
        },
        FlightControlInput {
            throttle: 0.7,
            collective: 0.7,
            vtol_transition: 0.6,
            ..default()
        },
        Transform::from_xyz(0.0, 20.0, 0.0),
    ));

    for _ in 0..20 {
        app.update();
    }

    let telemetry = app
        .world_mut()
        .query::<&FlightTelemetry>()
        .single(app.world())
        .expect("telemetry should exist");
    assert!(telemetry.vtol_transition > 0.0);
}
