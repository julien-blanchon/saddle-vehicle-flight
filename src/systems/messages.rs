use crate::{
    components::{FlightMessageState, LandingGearState, StallState},
    messages::{GearStateChanged, StallEntered, StallRecovered},
    telemetry::FlightAeroState,
};
use bevy::prelude::*;

pub(crate) fn emit_stall_messages(
    mut query: Query<(
        Entity,
        &StallState,
        &FlightAeroState,
        &mut FlightMessageState,
    )>,
    mut entered: MessageWriter<StallEntered>,
    mut recovered: MessageWriter<StallRecovered>,
) {
    for (entity, stall, aero, mut state) in &mut query {
        if stall.is_stalled && !state.last_stalled {
            entered.write(StallEntered {
                entity,
                angle_of_attack_rad: aero.angle_of_attack_rad,
                stall_amount: stall.amount,
            });
        } else if !stall.is_stalled && state.last_stalled {
            recovered.write(StallRecovered {
                entity,
                angle_of_attack_rad: aero.angle_of_attack_rad,
            });
        }
        state.last_stalled = stall.is_stalled;
    }
}

pub(crate) fn emit_gear_messages(
    mut query: Query<(Entity, &LandingGearState, &mut FlightMessageState)>,
    mut messages: MessageWriter<GearStateChanged>,
) {
    for (entity, gear, mut state) in &mut query {
        let deployed = gear.deployed();
        if deployed != state.last_gear_deployed {
            messages.write(GearStateChanged {
                entity,
                deployed,
                position: gear.position,
            });
        }
        state.last_gear_deployed = deployed;
    }
}
