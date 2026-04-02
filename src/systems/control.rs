use crate::{
    components::{FlightControlInput, LandingGearState, ResolvedFlightControls},
    config::{FixedWingAircraft, HelicopterAircraft},
    math::{move_towards, shape_axis},
};
use bevy::prelude::*;

pub(crate) fn resolve_controls(
    time: Res<Time>,
    mut query: Query<(
        &FlightControlInput,
        &mut ResolvedFlightControls,
        &mut LandingGearState,
        Option<&FixedWingAircraft>,
        Option<&HelicopterAircraft>,
    )>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (input, mut resolved, mut gear, fixed_wing, helicopter) in &mut query {
        let (response, power, contact) = if let Some(aircraft) = fixed_wing {
            (
                aircraft.control_response,
                aircraft.power_response,
                aircraft.landing_contact,
            )
        } else if let Some(aircraft) = helicopter {
            (
                aircraft.control_response,
                aircraft.power_response,
                aircraft.contact_geometry,
            )
        } else {
            continue;
        };

        let desired_pitch = shape_axis(input.pitch, response.pitch_exponent);
        let desired_roll = shape_axis(input.roll, response.roll_exponent);
        let desired_yaw = shape_axis(input.yaw, response.yaw_exponent);

        resolved.pitch = move_towards(
            resolved.pitch,
            desired_pitch,
            response.pitch_rate_per_second * dt,
        );
        resolved.roll = move_towards(
            resolved.roll,
            desired_roll,
            response.roll_rate_per_second * dt,
        );
        resolved.yaw = move_towards(resolved.yaw, desired_yaw, response.yaw_rate_per_second * dt);
        resolved.throttle = move_towards(
            resolved.throttle,
            input.throttle.clamp(0.0, 1.0),
            if input.throttle >= resolved.throttle {
                power.throttle_rise_per_second
            } else {
                power.throttle_fall_per_second
            } * dt,
        );
        resolved.collective = move_towards(
            resolved.collective,
            input.collective.clamp(0.0, 1.0),
            if input.collective >= resolved.collective {
                power.collective_rise_per_second
            } else {
                power.collective_fall_per_second
            } * dt,
        );

        if let Some(requested) = input.requested_gear_down {
            gear.target_deployed = requested;
        } else if input.toggle_gear && !resolved.gear_toggle_latched && contact.retractable {
            gear.target_deployed = !gear.target_deployed;
        }
        resolved.gear_toggle_latched = input.toggle_gear;

        if contact.retractable {
            gear.position = move_towards(
                gear.position,
                if gear.target_deployed { 1.0 } else { 0.0 },
                contact.extension_rate_per_second * dt,
            );
        } else {
            gear.target_deployed = true;
            gear.position = 1.0;
        }
    }
}
