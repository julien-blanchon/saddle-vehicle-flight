use bevy::prelude::*;

use crate::{
    AtmosphereSample, FlightAeroState, FlightAssist, FlightControlChannels, FlightControlInput,
    FlightEnvironment, FlightKinematics, LandingGearState,
    config::{
        ContactGeometry, FixedWingActuators, FixedWingModel, GroundHandling, RotorcraftActuators,
        RotorcraftModel,
    },
    model::{
        common::{alpha_from_motion, beta_from_motion, lift_direction_world, sample_motion},
        fixed_wing::evaluate_fixed_wing_with_motion,
        helicopter::evaluate_rotorcraft,
    },
};

fn fixed_wing_trainer_setup() -> (
    FixedWingModel,
    FixedWingActuators,
    crate::FlightBody,
    GroundHandling,
) {
    (
        FixedWingModel::default(),
        FixedWingActuators::default(),
        crate::FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.15,
                retractable: true,
                extension_rate_per_second: 0.75,
                ground_effect_height_m: 5.0,
                ground_effect_boost: 0.15,
            },
            longitudinal_damping_per_second: 0.14,
            lateral_damping_per_second: 1.4,
            angular_damping_per_second: 1.4,
        },
    )
}

fn rotorcraft_utility_setup() -> (
    RotorcraftModel,
    RotorcraftActuators,
    crate::FlightBody,
    GroundHandling,
) {
    (
        RotorcraftModel::default(),
        RotorcraftActuators::default(),
        crate::FlightBody::new(1_150.0, Vec3::new(1_800.0, 1_600.0, 2_100.0)),
        GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.65,
                retractable: false,
                extension_rate_per_second: 1.0,
                ground_effect_height_m: 3.6,
                ground_effect_boost: 0.18,
            },
            longitudinal_damping_per_second: 2.0,
            lateral_damping_per_second: 2.0,
            angular_damping_per_second: 2.0,
        },
    )
}

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
    let (model, actuators, body, ground) = fixed_wing_trainer_setup();
    let evaluated = evaluate_fixed_wing_with_motion(
        sample_motion_for(Vec3::ZERO),
        &sample_transform(),
        body,
        model,
        actuators,
        FlightControlChannels::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
        Some(ground),
    );
    assert!(evaluated.forces.lift_world_newtons.length() < 1e-4);
    assert!(evaluated.forces.drag_world_newtons.length() < 1e-4);
}

#[test]
fn fixed_wing_can_enter_stall_when_alpha_is_high() {
    let (model, actuators, body, ground) = fixed_wing_trainer_setup();
    let motion = sample_motion_for(Vec3::new(0.0, -20.0, -20.0));
    let evaluated = evaluate_fixed_wing_with_motion(
        motion,
        &sample_transform(),
        body,
        model,
        actuators,
        FlightControlChannels::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
        Some(ground),
    );
    assert!(evaluated.stall_target > 0.5);
}

#[test]
fn symmetric_inputs_do_not_create_unwanted_yaw() {
    let (model, actuators, body, ground) = fixed_wing_trainer_setup();
    let evaluated = evaluate_fixed_wing_with_motion(
        sample_motion_for(Vec3::new(0.0, 0.0, -55.0)),
        &sample_transform(),
        body,
        model,
        actuators,
        FlightControlChannels::default(),
        FlightControlInput::default(),
        FlightAssist::default(),
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
        Some(ground),
    );
    assert!(evaluated.forces.total_torque_body_nm.y.abs() < 1e-3);
}

#[test]
fn helicopter_collective_produces_upward_thrust() {
    let (model, actuators, body, ground) = rotorcraft_utility_setup();
    let evaluated = evaluate_rotorcraft(
        sample_motion_for(Vec3::new(0.0, 0.0, -2.0)),
        &sample_transform(),
        body,
        model,
        actuators,
        FlightControlChannels {
            vertical_thrust: 0.8,
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
        Some(ground),
    );
    assert!(evaluated.forces.thrust_world_newtons.y > 0.0);
}
