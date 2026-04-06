use crate::{
    components::{
        FlightAssist, FlightBody, FlightControlChannels, FlightControlInput, FlightEnvironment,
        FlightKinematics, LandingGearState, StallState,
    },
    config::{
        GroundHandling, VehicleActuatorKind, VehicleActuators, VehicleModel, VehicleModelKind,
    },
    math::move_towards,
    model::{
        common::sample_motion, fixed_wing::evaluate_fixed_wing_with_motion,
        helicopter::evaluate_rotorcraft, spacecraft::evaluate_spacecraft, vtol::evaluate_hybrid,
    },
    telemetry::{FlightAeroState, FlightForces},
};
use bevy::prelude::*;

pub(crate) fn compute_vehicle_dynamics(
    time: Res<Time>,
    mut query: Query<(
        &Transform,
        &FlightBody,
        &VehicleModel,
        &VehicleActuators,
        Option<&GroundHandling>,
        &FlightControlChannels,
        &FlightControlInput,
        &FlightAssist,
        &FlightKinematics,
        &FlightEnvironment,
        &LandingGearState,
        &mut StallState,
        &mut FlightAeroState,
        &mut FlightForces,
    )>,
) {
    let dt = time.delta_secs();
    for (
        transform,
        body,
        vehicle_model,
        actuators,
        ground,
        channels,
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
        let evaluated = match (vehicle_model.kind, actuators.kind) {
            (VehicleModelKind::FixedWing(model), VehicleActuatorKind::FixedWing(actuators)) => {
                evaluate_fixed_wing_with_motion(
                    motion,
                    transform,
                    *body,
                    model,
                    actuators,
                    *channels,
                    *control_input,
                    *assist,
                    *aero,
                    stall.amount,
                    *gear,
                    ground.copied(),
                )
            }
            (VehicleModelKind::Rotorcraft(model), VehicleActuatorKind::Rotorcraft(actuators)) => {
                evaluate_rotorcraft(
                    motion,
                    transform,
                    *body,
                    model,
                    actuators,
                    *channels,
                    *control_input,
                    *assist,
                    *aero,
                    *gear,
                    ground.copied(),
                )
            }
            (VehicleModelKind::Hybrid(model), VehicleActuatorKind::Hybrid(actuators)) => {
                evaluate_hybrid(
                    motion,
                    transform,
                    *body,
                    model,
                    actuators,
                    *channels,
                    *control_input,
                    *assist,
                    *aero,
                    stall.amount,
                    *gear,
                    ground.copied(),
                )
            }
            (VehicleModelKind::Spacecraft(model), VehicleActuatorKind::Spacecraft(actuators)) => {
                evaluate_spacecraft(
                    motion,
                    transform,
                    *body,
                    model,
                    actuators,
                    *channels,
                    *control_input,
                    *assist,
                    *aero,
                    *gear,
                )
            }
            _ => continue,
        };

        if let Some(stall_response_per_second) = vehicle_model.stall_response_per_second() {
            stall.amount = move_towards(
                stall.amount,
                evaluated.stall_target,
                stall_response_per_second * dt,
            );
            stall.is_stalled = stall.amount >= 0.55;
        } else {
            stall.amount = 0.0;
            stall.is_stalled = false;
        }

        *aero = evaluated.aero;
        *forces = evaluated.forces;
    }
}
