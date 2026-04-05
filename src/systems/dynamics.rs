use crate::{
    components::{
        FlightAssist, FlightBody, FlightControlInput, FlightEnvironment, FlightKinematics,
        LandingGearState, ResolvedFlightControls, StallState,
    },
    config::{FixedWingAircraft, HelicopterAircraft, SpacecraftConfig, VtolAircraft},
    math::move_towards,
    model::{
        common::sample_motion, fixed_wing::evaluate_fixed_wing_with_motion,
        helicopter::evaluate_helicopter, spacecraft::evaluate_spacecraft, vtol::evaluate_vtol,
    },
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

pub(crate) fn compute_fixed_wing_dynamics(
    time: Res<Time>,
    mut query: Query<
        (
            &Transform,
            &FlightBody,
            &FixedWingAircraft,
            &ResolvedFlightControls,
            &FlightControlInput,
            &FlightAssist,
            &FlightKinematics,
            &FlightEnvironment,
            &LandingGearState,
            &mut StallState,
            &mut FlightAeroState,
            &mut FlightForces,
        ),
        Without<HelicopterAircraft>,
    >,
) {
    let dt = time.delta_secs();
    for (
        transform,
        body,
        aircraft,
        controls,
        control_input,
        assist,
        kinematics,
        environment,
        gear,
        mut stall,
        mut aero,
        mut forces,
    ) in &mut query
    {
        let motion = sample_motion(transform, kinematics, environment);
        let evaluated = evaluate_fixed_wing_with_motion(
            motion,
            transform,
            *body,
            *aircraft,
            *controls,
            *control_input,
            *assist,
            *aero,
            stall.amount,
            *gear,
        );
        stall.amount = move_towards(
            stall.amount,
            evaluated.stall_target,
            aircraft.stall_response_per_second * dt,
        );
        stall.is_stalled = stall.amount >= 0.55;
        *aero = evaluated.aero;
        *forces = evaluated.forces;
    }
}

pub(crate) fn compute_helicopter_dynamics(
    mut query: Query<
        (
            &Transform,
            &FlightBody,
            &HelicopterAircraft,
            &ResolvedFlightControls,
            &FlightControlInput,
            &FlightAssist,
            &FlightKinematics,
            &FlightEnvironment,
            &LandingGearState,
            &mut StallState,
            &mut FlightAeroState,
            &mut FlightForces,
        ),
        Without<FixedWingAircraft>,
    >,
) {
    for (
        transform,
        body,
        aircraft,
        controls,
        control_input,
        assist,
        kinematics,
        environment,
        gear,
        mut stall,
        mut aero,
        mut forces,
    ) in &mut query
    {
        let motion = sample_motion(transform, kinematics, environment);
        let evaluated = evaluate_helicopter(
            motion,
            transform,
            *body,
            *aircraft,
            *controls,
            *control_input,
            *assist,
            *aero,
            *gear,
        );
        stall.amount = 0.0;
        stall.is_stalled = false;
        *aero = evaluated.aero;
        *forces = evaluated.forces;
    }
}

pub(crate) fn compute_spacecraft_dynamics(
    mut query: Query<
        (
            &Transform,
            &FlightBody,
            &SpacecraftConfig,
            &ResolvedFlightControls,
            &FlightControlInput,
            &FlightAssist,
            &FlightKinematics,
            &FlightEnvironment,
            &LandingGearState,
            &mut StallState,
            &mut FlightAeroState,
            &mut FlightForces,
        ),
        (
            Without<FixedWingAircraft>,
            Without<HelicopterAircraft>,
            Without<VtolAircraft>,
        ),
    >,
) {
    for (
        transform,
        body,
        spacecraft,
        controls,
        control_input,
        assist,
        kinematics,
        environment,
        gear,
        mut stall,
        mut aero,
        mut forces,
    ) in &mut query
    {
        let motion = sample_motion(transform, kinematics, environment);
        let evaluated = evaluate_spacecraft(
            motion,
            transform,
            *body,
            *spacecraft,
            *controls,
            *control_input,
            *assist,
            *aero,
            *gear,
        );
        stall.amount = 0.0;
        stall.is_stalled = false;
        *aero = evaluated.aero;
        *forces = evaluated.forces;
    }
}

pub(crate) fn compute_vtol_dynamics(
    time: Res<Time>,
    mut query: Query<
        (
            &Transform,
            &FlightBody,
            &VtolAircraft,
            &ResolvedFlightControls,
            &FlightControlInput,
            &FlightAssist,
            &FlightKinematics,
            &FlightEnvironment,
            &LandingGearState,
            &mut StallState,
            &mut FlightAeroState,
            &mut FlightForces,
        ),
        (Without<FixedWingAircraft>, Without<HelicopterAircraft>),
    >,
) {
    let dt = time.delta_secs();
    for (
        transform,
        body,
        aircraft,
        controls,
        control_input,
        assist,
        kinematics,
        environment,
        gear,
        mut stall,
        mut aero,
        mut forces,
    ) in &mut query
    {
        let motion = sample_motion(transform, kinematics, environment);
        let evaluated = evaluate_vtol(
            motion,
            transform,
            *body,
            *aircraft,
            *controls,
            *control_input,
            *assist,
            *aero,
            stall.amount,
            *gear,
        );
        stall.amount = move_towards(
            stall.amount,
            evaluated.stall_target,
            aircraft.fixed_wing.stall_response_per_second * dt,
        );
        stall.is_stalled = stall.amount >= 0.55;
        *aero = evaluated.aero;
        *forces = evaluated.forces;
    }
}
