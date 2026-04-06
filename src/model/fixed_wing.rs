use crate::{
    atmosphere::{SEA_LEVEL_AIR_DENSITY_KG_PER_M3, dynamic_pressure},
    components::{
        FlightAssist, FlightBody, FlightControlChannels, FlightControlInput, LandingGearState,
    },
    config::{FixedWingActuators, FixedWingModel, GroundHandling},
    model::common::{
        AIRSPEED_EPSILON_MPS, EvaluatedFlight, alpha_from_motion, axis_world, beta_from_motion,
        ground_effect_multiplier, lift_direction_world,
    },
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

use super::common::MotionSample;

pub(crate) fn evaluate_fixed_wing_with_motion(
    motion: MotionSample,
    transform: &Transform,
    body: FlightBody,
    model: FixedWingModel,
    actuators: FixedWingActuators,
    controls: FlightControlChannels,
    control_input: FlightControlInput,
    assist: FlightAssist,
    current_aero: FlightAeroState,
    current_stall_amount: f32,
    gear: LandingGearState,
    ground: Option<GroundHandling>,
) -> EvaluatedFlight {
    let alpha = if motion.airspeed_mps > AIRSPEED_EPSILON_MPS {
        alpha_from_motion(motion)
    } else {
        0.0
    };
    let beta = if motion.airspeed_mps > AIRSPEED_EPSILON_MPS {
        beta_from_motion(motion)
    } else {
        0.0
    };
    let qbar = dynamic_pressure(
        current_aero.atmosphere.density_kg_per_m3,
        motion.airspeed_mps.max(0.0),
    );
    let stall_target = if alpha.abs() >= model.stall_alpha_rad {
        1.0
    } else if alpha.abs() <= model.recovery_alpha_rad {
        0.0
    } else {
        current_stall_amount
    };
    let elevator =
        (controls.pitch + control_input.pitch_trim * actuators.trim_authority.y).clamp(-1.0, 1.0);
    let aileron =
        (controls.roll + control_input.roll_trim * actuators.trim_authority.z).clamp(-1.0, 1.0);
    let rudder =
        (controls.yaw + control_input.yaw_trim * actuators.trim_authority.x).clamp(-1.0, 1.0);

    let cl_linear = model.cl0
        + model.lift_curve_slope_per_rad * alpha
        + actuators.elevator_authority * elevator;
    let cl_linear = cl_linear.clamp(-model.max_lift_coefficient, model.max_lift_coefficient);
    let cl_stalled = model.post_stall_lift_coefficient * alpha.signum();
    let cl = cl_linear * (1.0 - current_stall_amount) + cl_stalled * current_stall_amount;

    let gear_drag = if gear.deployed() {
        model.gear_drag_coefficient
    } else {
        0.0
    };
    let cd = model.zero_lift_drag_coefficient
        + model.induced_drag_factor * cl.powi(2)
        + model.stall_drag_coefficient * current_stall_amount
        + gear_drag;
    let cy = (-model.side_force_slope_per_rad * beta) + actuators.rudder_authority * rudder;

    let ground_effect = ground
        .filter(|ground| ground.enabled)
        .map(|ground| {
            ground_effect_multiplier(
                current_aero.altitude_agl_m,
                ground.contact_geometry.ground_effect_height_m,
                ground.contact_geometry.ground_effect_boost,
            )
        })
        .unwrap_or(1.0);
    let lift_force_mag = cl * qbar * model.wing_area_m2 * ground_effect;
    let drag_force_mag = cd * qbar * model.wing_area_m2;
    let side_force_mag = cy * qbar * model.wing_area_m2;
    let thrust_force_mag = controls.forward_thrust.max(0.0) * actuators.max_forward_thrust_newtons;

    let lift_world = lift_direction_world(motion) * lift_force_mag;
    let drag_world = -motion.air_direction_world * drag_force_mag;
    let side_world = -motion.right_world * side_force_mag;
    let thrust_world =
        axis_world(transform, actuators.thrust_axis_local, motion.forward_world) * thrust_force_mag;
    let gravity_world = Vec3::NEG_Y * body.mass_kg * body.gravity_acceleration_mps2;

    let p = motion.angular_velocity_body_rps.z;
    let q = motion.angular_velocity_body_rps.x;
    let r = motion.angular_velocity_body_rps.y;
    let rate_scale = (motion.airspeed_mps / 40.0).clamp(0.0, 3.0);
    let control_torque_body_nm = Vec3::new(
        -(actuators.elevator_authority * elevator + model.pitch_stability * alpha)
            * qbar
            * model.wing_area_m2
            * model.mean_chord_m,
        (actuators.rudder_authority * rudder - model.yaw_stability * beta)
            * qbar
            * model.wing_area_m2
            * model.wingspan_m,
        -(actuators.aileron_authority * aileron + model.roll_stability * beta)
            * qbar
            * model.wing_area_m2
            * model.wingspan_m,
    );
    let damping_torque_body_nm = Vec3::new(
        -model.pitch_rate_damping * q * rate_scale,
        -model.yaw_rate_damping * r * rate_scale,
        -model.roll_rate_damping * p * rate_scale,
    ) * qbar.max(1.0);
    let assist_torque_body_nm = Vec3::new(
        -motion.forward_world.y * assist.wings_leveling * qbar * model.mean_chord_m,
        -beta * assist.coordinated_turn * qbar * model.wingspan_m,
        motion.right_world.y * assist.wings_leveling * qbar * model.wingspan_m,
    );
    let total_torque_body_nm =
        control_torque_body_nm + damping_torque_body_nm + assist_torque_body_nm;

    let atmosphere = current_aero.atmosphere;
    EvaluatedFlight {
        aero: FlightAeroState {
            atmosphere,
            altitude_msl_m: transform.translation.y,
            altitude_agl_m: current_aero.altitude_agl_m,
            airspeed_mps: motion.airspeed_mps,
            forward_speed_mps: motion.forward_speed_mps,
            side_speed_mps: motion.side_speed_mps,
            body_vertical_speed_mps: motion.body_vertical_speed_mps,
            dynamic_pressure_pa: qbar,
            angle_of_attack_rad: alpha,
            sideslip_rad: beta,
        },
        forces: FlightForces {
            thrust_world_newtons: thrust_world,
            lift_world_newtons: lift_world,
            drag_world_newtons: drag_world,
            side_force_world_newtons: side_world,
            gravity_world_newtons: gravity_world,
            total_force_world_newtons: thrust_world
                + lift_world
                + drag_world
                + side_world
                + gravity_world,
            control_torque_body_nm,
            damping_torque_body_nm,
            assist_torque_body_nm,
            total_torque_body_nm,
        },
        stall_target,
    }
}

pub(crate) fn approximate_ias(true_airspeed_mps: f32, density_kg_per_m3: f32) -> f32 {
    true_airspeed_mps * (density_kg_per_m3 / SEA_LEVEL_AIR_DENSITY_KG_PER_M3).sqrt()
}
