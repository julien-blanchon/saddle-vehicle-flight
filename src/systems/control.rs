use crate::{
    components::{FlightControlChannels, FlightControlInput, LandingGearState},
    config::{ContactGeometry, GroundHandling, VehicleControlMap},
    math::move_towards,
};
use bevy::prelude::*;

pub(crate) fn resolve_controls(
    time: Res<Time>,
    mut query: Query<(
        &FlightControlInput,
        &VehicleControlMap,
        Option<&GroundHandling>,
        &mut FlightControlChannels,
        &mut LandingGearState,
    )>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (input, control_map, ground, mut channels, mut gear) in &mut query {
        channels.pitch = control_map.pitch.resolve(*input, channels.pitch, dt);
        channels.roll = control_map.roll.resolve(*input, channels.roll, dt);
        channels.yaw = control_map.yaw.resolve(*input, channels.yaw, dt);
        channels.forward_thrust =
            control_map
                .forward_thrust
                .resolve(*input, channels.forward_thrust, dt);
        channels.vertical_thrust =
            control_map
                .vertical_thrust
                .resolve(*input, channels.vertical_thrust, dt);
        channels.lateral_thrust =
            control_map
                .lateral_thrust
                .resolve(*input, channels.lateral_thrust, dt);
        channels.transition = control_map
            .transition
            .resolve(*input, channels.transition, dt);

        let contact = ground
            .map(|ground| ground.contact_geometry)
            .unwrap_or_else(|| ContactGeometry {
                retractable: false,
                ..default()
            });

        if let Some(requested) = input.requested_gear_down {
            gear.target_deployed = requested;
        } else if input.toggle_gear && !channels.gear_toggle_latched && contact.retractable {
            gear.target_deployed = !gear.target_deployed;
        }
        channels.gear_toggle_latched = input.toggle_gear;

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
