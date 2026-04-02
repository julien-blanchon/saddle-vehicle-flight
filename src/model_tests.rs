use bevy::prelude::*;

use crate::{
    AtmosphereSample, FlightAeroState, FlightAssist, FlightBody, FlightControlInput,
    FlightEnvironment, FlightKinematics, LandingGearState,
    components::ResolvedFlightControls,
    config::{FixedWingAircraft, HelicopterAircraft},
    model::{
        common::{alpha_from_motion, beta_from_motion, lift_direction_world, sample_motion},
        fixed_wing::evaluate_fixed_wing_with_motion,
        helicopter::evaluate_helicopter,
    },
};

fn sample_transform() -> Transform {
    Transform::IDENTITY
}

fn sample_motion_for(velocity: Vec3) -> crate::model::common::MotionSample {
    sample_motion(
        &sample_transform(),
        &FlightKinematics {
            linear_velocity_world_mps: velocity,
            ..default()
        },
        &FlightEnvironment::default(),
    )
}

#[test]
fn positive_aoa_comes_from_velocity_below_nose() {
    let motion = sample_motion_for(Vec3::new(0.0, -5.0, -50.0));
    let alpha = alpha_from_motion(motion);
    assert!(alpha > 0.0);
}

#[test]
fn positive_sideslip_means_motion_to_the_right() {
    let motion = sample_motion_for(Vec3::new(5.0, 0.0, -50.0));
    let beta = beta_from_motion(motion);
    assert!(beta > 0.0);
}

#[test]
fn lift_points_up_for_positive_aoa() {
    let motion = sample_motion_for(Vec3::new(0.0, -8.0, -55.0));
    let lift = lift_direction_world(motion);
    assert!(lift.y > 0.0);
}

#[test]
fn zero_speed_produces_zero_aero_forces() {
    let evaluated = evaluate_fixed_wing_with_motion(
        sample_motion_for(Vec3::ZERO),
        &sample_transform(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FixedWingAircraft::trainer(),
        ResolvedFlightControls::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
    );
    assert!(evaluated.forces.lift_world_newtons.length() < 1e-4);
    assert!(evaluated.forces.drag_world_newtons.length() < 1e-4);
}

#[test]
fn fixed_wing_can_enter_stall_when_alpha_is_high() {
    let motion = sample_motion_for(Vec3::new(0.0, -20.0, -20.0));
    let evaluated = evaluate_fixed_wing_with_motion(
        motion,
        &sample_transform(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FixedWingAircraft::trainer(),
        ResolvedFlightControls::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
    );
    assert!(evaluated.stall_target > 0.5);
}

#[test]
fn symmetric_inputs_do_not_create_unwanted_yaw() {
    let aircraft = FixedWingAircraft::trainer();
    let evaluated = evaluate_fixed_wing_with_motion(
        sample_motion_for(Vec3::new(0.0, 0.0, -55.0)),
        &sample_transform(),
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        aircraft,
        ResolvedFlightControls::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
    );
    assert!(evaluated.forces.total_torque_body_nm.y.abs() < 1e-3);
}

#[test]
fn helicopter_collective_produces_upward_thrust() {
    let evaluated = evaluate_helicopter(
        sample_motion_for(Vec3::new(0.0, 0.0, -2.0)),
        &sample_transform(),
        FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0)),
        HelicopterAircraft::utility(),
        ResolvedFlightControls {
            collective: 0.8,
            ..default()
        },
        FlightControlInput {
            collective: 0.8,
            ..default()
        },
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        LandingGearState::default(),
    );
    assert!(evaluated.forces.thrust_world_newtons.y > 0.0);
}
