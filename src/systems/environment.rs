use crate::{
    atmosphere::sample_us_standard_atmosphere_1976, components::FlightEnvironment,
    telemetry::FlightAeroState,
};
use bevy::prelude::*;

pub(crate) fn sample_environment(
    mut query: Query<(&Transform, &FlightEnvironment, &mut FlightAeroState)>,
) {
    for (transform, environment, mut aero) in &mut query {
        let atmosphere = sample_us_standard_atmosphere_1976(transform.translation.y);
        aero.atmosphere = crate::atmosphere::AtmosphereSample {
            density_kg_per_m3: atmosphere.density_kg_per_m3
                * environment.density_multiplier.max(0.01),
            density_ratio: atmosphere.density_ratio * environment.density_multiplier.max(0.01),
            ..atmosphere
        };
        aero.altitude_msl_m = transform.translation.y;
        aero.altitude_agl_m = environment
            .surface_altitude_msl_m
            .map(|surface| transform.translation.y - surface);
    }
}
