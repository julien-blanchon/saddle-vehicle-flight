use crate::{
    components::{FlightControlInput, LandingGearState, ResolvedFlightControls},
    config::{FixedWingAircraft, HelicopterAircraft, SpacecraftConfig, VtolAircraft},
    math::{move_towards, shape_axis},
};
use bevy::prelude::*;

fn blend_response(
    hover: crate::FlightControlResponse,
    wing: crate::FlightControlResponse,
    blend: f32,
) -> crate::FlightControlResponse {
    crate::FlightControlResponse {
        pitch_rate_per_second: hover
            .pitch_rate_per_second
            .lerp(wing.pitch_rate_per_second, blend),
        roll_rate_per_second: hover
            .roll_rate_per_second
            .lerp(wing.roll_rate_per_second, blend),
        yaw_rate_per_second: hover
            .yaw_rate_per_second
            .lerp(wing.yaw_rate_per_second, blend),
        pitch_exponent: hover.pitch_exponent.lerp(wing.pitch_exponent, blend),
        roll_exponent: hover.roll_exponent.lerp(wing.roll_exponent, blend),
        yaw_exponent: hover.yaw_exponent.lerp(wing.yaw_exponent, blend),
    }
}

fn blend_power(
    hover: crate::PowerResponse,
    wing: crate::PowerResponse,
    blend: f32,
) -> crate::PowerResponse {
    crate::PowerResponse {
        throttle_rise_per_second: hover
            .throttle_rise_per_second
            .lerp(wing.throttle_rise_per_second, blend),
        throttle_fall_per_second: hover
            .throttle_fall_per_second
            .lerp(wing.throttle_fall_per_second, blend),
        collective_rise_per_second: hover
            .collective_rise_per_second
            .lerp(wing.collective_rise_per_second, blend),
        collective_fall_per_second: hover
            .collective_fall_per_second
            .lerp(wing.collective_fall_per_second, blend),
    }
}

pub(crate) fn resolve_controls(
    time: Res<Time>,
    mut query: Query<(
        &FlightControlInput,
        &mut ResolvedFlightControls,
        &mut LandingGearState,
        Option<&FixedWingAircraft>,
        Option<&HelicopterAircraft>,
        Option<&VtolAircraft>,
        Option<&SpacecraftConfig>,
    )>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (input, mut resolved, mut gear, fixed_wing, helicopter, vtol, spacecraft) in &mut query {
        let (response, power, contact, target_transition, transition_rate_per_second) =
            if let Some(aircraft) = fixed_wing {
                (
                    aircraft.control_response,
                    aircraft.power_response,
                    aircraft.landing_contact,
                    1.0,
                    0.0,
                )
            } else if let Some(aircraft) = helicopter {
                (
                    aircraft.control_response,
                    aircraft.power_response,
                    aircraft.contact_geometry,
                    0.0,
                    0.0,
                )
            } else if let Some(aircraft) = vtol {
                let blend = aircraft.wingborne_blend(resolved.transition);
                (
                    blend_response(
                        aircraft.rotorcraft.control_response,
                        aircraft.fixed_wing.control_response,
                        blend,
                    ),
                    blend_power(
                        aircraft.rotorcraft.power_response,
                        aircraft.fixed_wing.power_response,
                        blend,
                    ),
                    aircraft.contact_geometry,
                    input.vtol_transition.clamp(0.0, 1.0),
                    aircraft.transition_rate_per_second,
                )
            } else if let Some(craft) = spacecraft {
                (
                    craft.control_response,
                    craft.power_response,
                    crate::config::ContactGeometry::default(),
                    0.0,
                    0.0,
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
        resolved.transition = if transition_rate_per_second <= 0.0 {
            target_transition
        } else {
            move_towards(
                resolved.transition,
                target_transition,
                transition_rate_per_second * dt,
            )
        };

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
