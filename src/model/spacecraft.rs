use crate::{
    components::{FlightAssist, FlightBody, FlightControlInput, LandingGearState},
    config::SpacecraftConfig,
    model::common::EvaluatedFlight,
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

use super::common::MotionSample;

pub(crate) fn evaluate_spacecraft(
    motion: MotionSample,
    transform: &Transform,
    body: FlightBody,
    spacecraft: SpacecraftConfig,
    controls: crate::components::ResolvedFlightControls,
    _control_input: FlightControlInput,
    assist: FlightAssist,
    current_aero: FlightAeroState,
    _gear: LandingGearState,
) -> EvaluatedFlight {
    // --- linear forces ---
    // Main thrust along forward axis
    let thrust_forward = controls.throttle.clamp(0.0, 1.0) * spacecraft.max_thrust_newtons;
    let thrust_world = motion.forward_world * thrust_forward;

    // RCS translation (optional lateral and vertical thrusters)
    let rcs_right = controls.yaw * spacecraft.rcs_thrust_newtons;
    let rcs_up = (controls.collective - 0.5) * 2.0 * spacecraft.rcs_thrust_newtons;
    let rcs_world = motion.right_world * rcs_right + motion.up_world * rcs_up;

    // Linear drag (configurable — zero for pure Newtonian, nonzero for game-feel)
    let speed = motion.airspeed_mps;
    let drag_world = if speed > 0.01 {
        -motion.air_direction_world * speed * spacecraft.linear_drag_coefficient * body.mass_kg
    } else {
        Vec3::ZERO
    };

    let gravity_world = Vec3::NEG_Y * body.mass_kg * body.gravity_acceleration_mps2;
    let total_force_world = thrust_world + rcs_world + drag_world + gravity_world;

    // --- torques ---
    let control_torque_body_nm = Vec3::new(
        -controls.pitch * spacecraft.pitch_torque_authority,
        controls.yaw * spacecraft.yaw_torque_authority,
        -controls.roll * spacecraft.roll_torque_authority,
    );

    let damping_torque_body_nm =
        -motion.angular_velocity_body_rps * spacecraft.angular_damping * 1_000.0;

    let assist_torque_body_nm = Vec3::new(
        -motion.forward_world.y * assist.wings_leveling * 1_500.0,
        0.0,
        motion.right_world.y * assist.wings_leveling * 1_500.0,
    );

    let total_torque_body_nm =
        control_torque_body_nm + damping_torque_body_nm + assist_torque_body_nm;

    EvaluatedFlight {
        aero: FlightAeroState {
            atmosphere: current_aero.atmosphere,
            altitude_msl_m: transform.translation.y,
            altitude_agl_m: current_aero.altitude_agl_m,
            airspeed_mps: speed,
            forward_speed_mps: motion.forward_speed_mps,
            side_speed_mps: motion.side_speed_mps,
            body_vertical_speed_mps: motion.body_vertical_speed_mps,
            dynamic_pressure_pa: 0.0,
            angle_of_attack_rad: 0.0,
            sideslip_rad: 0.0,
        },
        forces: FlightForces {
            thrust_world_newtons: thrust_world + rcs_world,
            lift_world_newtons: Vec3::ZERO,
            drag_world_newtons: drag_world,
            side_force_world_newtons: Vec3::ZERO,
            gravity_world_newtons: gravity_world,
            total_force_world_newtons: total_force_world,
            control_torque_body_nm,
            damping_torque_body_nm,
            assist_torque_body_nm,
            total_torque_body_nm,
        },
        stall_target: 0.0,
    }
}
