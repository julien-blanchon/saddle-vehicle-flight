use crate::{
    components::{FlightControlChannels, FlightKinematics, LandingGearState, StallState},
    model::fixed_wing::approximate_ias,
    telemetry::{FlightAeroState, FlightTelemetry},
};
use bevy::prelude::*;

pub(crate) fn update_telemetry(
    mut query: Query<(
        &FlightAeroState,
        &FlightKinematics,
        &FlightControlChannels,
        &LandingGearState,
        &StallState,
        &mut FlightTelemetry,
    )>,
) {
    for (aero, kinematics, controls, gear, stall, mut telemetry) in &mut query {
        telemetry.true_airspeed_mps = aero.airspeed_mps;
        telemetry.indicated_airspeed_mps =
            approximate_ias(aero.airspeed_mps, aero.atmosphere.density_kg_per_m3);
        telemetry.altitude_msl_m = aero.altitude_msl_m;
        telemetry.altitude_agl_m = aero.altitude_agl_m;
        telemetry.vertical_speed_mps = kinematics.linear_velocity_world_mps.y;
        telemetry.angle_of_attack_deg = aero.angle_of_attack_rad.to_degrees();
        telemetry.sideslip_deg = aero.sideslip_rad.to_degrees();
        telemetry.forward_thrust = controls.forward_thrust;
        telemetry.vertical_thrust = controls.vertical_thrust;
        telemetry.lateral_thrust = controls.lateral_thrust;
        telemetry.vtol_transition = controls.transition;
        telemetry.gear_position = gear.position;
        telemetry.gear_deployed = gear.deployed();
        telemetry.stalled = stall.is_stalled;
    }
}
