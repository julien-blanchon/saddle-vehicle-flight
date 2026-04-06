use crate::{
    atmosphere::dynamic_pressure,
    components::{
        FlightAssist, FlightBody, FlightControlChannels, FlightControlInput, LandingGearState,
    },
    config::{GroundHandling, RotorcraftActuators, RotorcraftModel},
    model::common::{
        AIRSPEED_EPSILON_MPS, EvaluatedFlight, axis_world, beta_from_motion,
        ground_effect_multiplier,
    },
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

use super::common::MotionSample;

pub(crate) fn evaluate_rotorcraft(
    motion: MotionSample,
    transform: &Transform,
    body: FlightBody,
    model: RotorcraftModel,
    actuators: RotorcraftActuators,
    controls: FlightControlChannels,
    control_input: FlightControlInput,
    assist: FlightAssist,
    current_aero: FlightAeroState,
    gear: LandingGearState,
    ground: Option<GroundHandling>,
) -> EvaluatedFlight {
    let vertical_thrust = controls.vertical_thrust.clamp(0.0, 1.0);
    let pitch_cmd =
        (controls.pitch + control_input.pitch_trim * actuators.trim_authority.y).clamp(-1.0, 1.0);
    let roll_cmd =
        (controls.roll + control_input.roll_trim * actuators.trim_authority.z).clamp(-1.0, 1.0);
    let yaw_cmd =
        (controls.yaw + control_input.yaw_trim * actuators.trim_authority.x).clamp(-1.0, 1.0);

    let translational_lift = if motion.airspeed_mps <= AIRSPEED_EPSILON_MPS {
        0.0
    } else {
        (motion.airspeed_mps / model.translational_lift_full_speed_mps).clamp(0.0, 1.0)
            * model.translational_lift_gain
    };
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
    let qbar = dynamic_pressure(
        current_aero.atmosphere.density_kg_per_m3,
        motion.airspeed_mps.max(0.0),
    );
    let lift_force_mag =
        vertical_thrust * actuators.max_lift_newtons * (1.0 + translational_lift) * ground_effect;
    let parasite_drag_mag = qbar * model.rotor_disc_area_m2 * model.parasite_drag_coefficient;
    let side_drag_mag = motion.side_speed_mps.abs()
        * current_aero.atmosphere.density_kg_per_m3
        * model.side_drag_coefficient
        * model.rotor_disc_area_m2;

    let thrust_world =
        axis_world(transform, actuators.lift_axis_local, motion.up_world) * lift_force_mag;
    let drag_world = -motion.air_direction_world * parasite_drag_mag;
    let side_world = -motion.right_world * motion.side_speed_mps.signum() * side_drag_mag;
    let gravity_world = Vec3::NEG_Y * body.mass_kg * body.gravity_acceleration_mps2;
    let total_force_world_newtons = thrust_world + drag_world + side_world + gravity_world;

    let control_torque_body_nm = Vec3::new(
        -pitch_cmd * actuators.pitch_torque_authority,
        yaw_cmd * actuators.yaw_torque_authority
            - vertical_thrust * actuators.anti_torque_per_vertical_thrust,
        -roll_cmd * actuators.roll_torque_authority,
    );
    let damping_torque_body_nm =
        -motion.angular_velocity_body_rps * model.angular_damping * 2_000.0;
    let assist_torque_body_nm = Vec3::new(
        -motion.forward_world.y * assist.hover_leveling * 2_200.0,
        -beta_from_motion(motion) * assist.coordinated_turn * 1_100.0,
        motion.right_world.y * assist.hover_leveling * 2_200.0,
    );
    let total_torque_body_nm =
        control_torque_body_nm + damping_torque_body_nm + assist_torque_body_nm;

    let _ = gear;
    EvaluatedFlight {
        aero: FlightAeroState {
            atmosphere: current_aero.atmosphere,
            altitude_msl_m: transform.translation.y,
            altitude_agl_m: current_aero.altitude_agl_m,
            airspeed_mps: motion.airspeed_mps,
            forward_speed_mps: motion.forward_speed_mps,
            side_speed_mps: motion.side_speed_mps,
            body_vertical_speed_mps: motion.body_vertical_speed_mps,
            dynamic_pressure_pa: qbar,
            angle_of_attack_rad: 0.0,
            sideslip_rad: if motion.airspeed_mps > AIRSPEED_EPSILON_MPS {
                beta_from_motion(motion)
            } else {
                0.0
            },
        },
        forces: FlightForces {
            thrust_world_newtons: thrust_world,
            lift_world_newtons: thrust_world,
            drag_world_newtons: drag_world,
            side_force_world_newtons: side_world,
            gravity_world_newtons: gravity_world,
            total_force_world_newtons,
            control_torque_body_nm,
            damping_torque_body_nm,
            assist_torque_body_nm,
            total_torque_body_nm,
        },
        stall_target: 0.0,
    }
}
