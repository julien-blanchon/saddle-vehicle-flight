use crate::{
    components::{FlightAssist, FlightBody, FlightControlInput, LandingGearState},
    config::VtolAircraft,
    model::{
        common::{EvaluatedFlight, MotionSample},
        fixed_wing::evaluate_fixed_wing_with_motion,
        helicopter::evaluate_helicopter,
    },
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

pub(crate) fn evaluate_vtol(
    motion: MotionSample,
    transform: &Transform,
    body: FlightBody,
    aircraft: VtolAircraft,
    controls: crate::components::ResolvedFlightControls,
    control_input: FlightControlInput,
    assist: FlightAssist,
    current_aero: FlightAeroState,
    current_stall_amount: f32,
    gear: LandingGearState,
) -> EvaluatedFlight {
    let wingborne_blend = aircraft.wingborne_blend(controls.transition);
    let power_command = controls.throttle.max(controls.collective).clamp(0.0, 1.0);

    let mut fixed_wing = aircraft.fixed_wing;
    fixed_wing.max_thrust_newtons = 0.0;
    fixed_wing.landing_contact = aircraft.contact_geometry;
    let fixed_eval = evaluate_fixed_wing_with_motion(
        motion,
        transform,
        body,
        fixed_wing,
        controls,
        control_input,
        assist,
        current_aero,
        current_stall_amount,
        gear,
    );

    let mut rotorcraft = aircraft.rotorcraft;
    rotorcraft.contact_geometry = aircraft.contact_geometry;
    let rotor_controls = crate::components::ResolvedFlightControls {
        collective: power_command,
        ..controls
    };
    let rotor_input = FlightControlInput {
        collective: power_command,
        ..control_input
    };
    let rotor_eval = evaluate_helicopter(
        motion,
        transform,
        body,
        rotorcraft,
        rotor_controls,
        rotor_input,
        assist,
        current_aero,
        gear,
    );

    let tilt_direction_world = motion
        .up_world
        .lerp(motion.forward_world, controls.transition.clamp(0.0, 1.0))
        .normalize_or_zero();
    let rotor_force_world = tilt_direction_world * rotor_eval.forces.thrust_world_newtons.length();
    let hover_blend = 1.0 - wingborne_blend;

    let lift_world = rotor_force_world + fixed_eval.forces.lift_world_newtons * wingborne_blend;
    let drag_world = rotor_eval.forces.drag_world_newtons * hover_blend
        + fixed_eval.forces.drag_world_newtons * wingborne_blend;
    let side_world = rotor_eval.forces.side_force_world_newtons * hover_blend
        + fixed_eval.forces.side_force_world_newtons * wingborne_blend;
    let gravity_world = fixed_eval.forces.gravity_world_newtons;

    let control_torque_body_nm = rotor_eval.forces.control_torque_body_nm * hover_blend
        + fixed_eval.forces.control_torque_body_nm * wingborne_blend;
    let damping_torque_body_nm = rotor_eval.forces.damping_torque_body_nm * hover_blend
        + fixed_eval.forces.damping_torque_body_nm * wingborne_blend;
    let assist_torque_body_nm = rotor_eval.forces.assist_torque_body_nm * hover_blend
        + fixed_eval.forces.assist_torque_body_nm * wingborne_blend;
    let total_torque_body_nm =
        control_torque_body_nm + damping_torque_body_nm + assist_torque_body_nm;

    EvaluatedFlight {
        aero: FlightAeroState {
            angle_of_attack_rad: fixed_eval.aero.angle_of_attack_rad * wingborne_blend,
            ..fixed_eval.aero
        },
        forces: FlightForces {
            thrust_world_newtons: rotor_force_world,
            lift_world_newtons: lift_world,
            drag_world_newtons: drag_world,
            side_force_world_newtons: side_world,
            gravity_world_newtons: gravity_world,
            total_force_world_newtons: lift_world + drag_world + side_world + gravity_world,
            control_torque_body_nm,
            damping_torque_body_nm,
            assist_torque_body_nm,
            total_torque_body_nm,
        },
        stall_target: fixed_eval.stall_target * wingborne_blend,
    }
}
