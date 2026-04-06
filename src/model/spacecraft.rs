use crate::{
    components::{
        FlightAssist, FlightBody, FlightControlChannels, FlightControlInput, LandingGearState,
    },
    config::{SpacecraftActuators, SpacecraftModel},
    model::common::{EvaluatedFlight, axis_world},
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

use super::common::MotionSample;

pub(crate) fn evaluate_spacecraft(
    motion: MotionSample,
    transform: &Transform,
    body: FlightBody,
    model: SpacecraftModel,
    actuators: SpacecraftActuators,
    controls: FlightControlChannels,
    _control_input: FlightControlInput,
    assist: FlightAssist,
    current_aero: FlightAeroState,
    _gear: LandingGearState,
) -> EvaluatedFlight {
    let forward_world = axis_world(
        transform,
        actuators.forward_axis_local,
        motion.forward_world,
    );
    let lateral_world = axis_world(transform, actuators.lateral_axis_local, motion.right_world);
    let vertical_world = axis_world(transform, actuators.vertical_axis_local, motion.up_world);

    let thrust_world = forward_world
        * controls.forward_thrust.clamp(-1.0, 1.0)
        * actuators.max_forward_thrust_newtons;
    let lateral_world = lateral_world
        * controls.lateral_thrust.clamp(-1.0, 1.0)
        * actuators.max_lateral_thrust_newtons;
    let vertical_world = vertical_world
        * controls.vertical_thrust.clamp(-1.0, 1.0)
        * actuators.max_vertical_thrust_newtons;

    let speed = motion.airspeed_mps;
    let drag_world = if speed > 0.01 {
        -motion.air_direction_world * speed * model.linear_drag_coefficient * body.mass_kg
    } else {
        Vec3::ZERO
    };

    let gravity_world = Vec3::NEG_Y * body.mass_kg * body.gravity_acceleration_mps2;
    let total_force_world =
        thrust_world + lateral_world + vertical_world + drag_world + gravity_world;

    let control_torque_body_nm = Vec3::new(
        -controls.pitch * actuators.pitch_torque_authority,
        controls.yaw * actuators.yaw_torque_authority,
        -controls.roll * actuators.roll_torque_authority,
    );

    let damping_torque_body_nm =
        -motion.angular_velocity_body_rps * model.angular_damping * 1_000.0;

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
            thrust_world_newtons: thrust_world + lateral_world + vertical_world,
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
