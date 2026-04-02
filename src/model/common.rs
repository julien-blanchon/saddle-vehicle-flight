use crate::{
    components::{FlightEnvironment, FlightKinematics},
    math::smoothstep01,
};
use bevy::prelude::*;

pub(crate) const AIRSPEED_EPSILON_MPS: f32 = 0.1;

#[derive(Debug, Clone, Copy)]
pub(crate) struct MotionSample {
    pub right_world: Vec3,
    pub up_world: Vec3,
    pub forward_world: Vec3,
    pub air_direction_world: Vec3,
    pub airspeed_mps: f32,
    pub forward_speed_mps: f32,
    pub side_speed_mps: f32,
    pub body_vertical_speed_mps: f32,
    pub angular_velocity_body_rps: Vec3,
}

pub(crate) fn sample_motion(
    transform: &Transform,
    kinematics: &FlightKinematics,
    environment: &FlightEnvironment,
) -> MotionSample {
    let right_world = transform.right().as_vec3();
    let up_world = transform.up().as_vec3();
    let forward_world = transform.forward().as_vec3();
    let air_velocity_world_mps =
        kinematics.linear_velocity_world_mps - environment.airmass_velocity_world();
    let airspeed_mps = air_velocity_world_mps.length();
    let air_direction_world = if airspeed_mps > AIRSPEED_EPSILON_MPS {
        air_velocity_world_mps / airspeed_mps
    } else {
        forward_world
    };

    MotionSample {
        right_world,
        up_world,
        forward_world,
        air_direction_world,
        airspeed_mps,
        forward_speed_mps: air_velocity_world_mps.dot(forward_world),
        side_speed_mps: air_velocity_world_mps.dot(right_world),
        body_vertical_speed_mps: air_velocity_world_mps.dot(up_world),
        angular_velocity_body_rps: kinematics.angular_velocity_body_rps,
    }
}

pub(crate) fn alpha_from_motion(sample: MotionSample) -> f32 {
    -sample
        .body_vertical_speed_mps
        .atan2(sample.forward_speed_mps.max(AIRSPEED_EPSILON_MPS))
}

pub(crate) fn beta_from_motion(sample: MotionSample) -> f32 {
    sample
        .side_speed_mps
        .atan2(sample.forward_speed_mps.max(AIRSPEED_EPSILON_MPS))
}

pub(crate) fn lift_direction_world(sample: MotionSample) -> Vec3 {
    let lift = sample.right_world.cross(sample.air_direction_world);
    if lift.length_squared() > 1e-6 {
        lift.normalize()
    } else {
        sample.up_world
    }
}

pub(crate) fn ground_effect_multiplier(
    altitude_agl_m: Option<f32>,
    ground_effect_height_m: f32,
    ground_effect_boost: f32,
) -> f32 {
    let Some(altitude_agl_m) = altitude_agl_m else {
        return 1.0;
    };
    if ground_effect_height_m <= f32::EPSILON {
        return 1.0;
    }
    let normalized = 1.0 - (altitude_agl_m / ground_effect_height_m).clamp(0.0, 1.0);
    1.0 + ground_effect_boost.max(0.0) * smoothstep01(normalized)
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct EvaluatedFlight {
    pub aero: crate::telemetry::FlightAeroState,
    pub forces: crate::telemetry::FlightForces,
    pub stall_target: f32,
}
