use bevy::prelude::*;

pub const SEA_LEVEL_AIR_DENSITY_KG_PER_M3: f32 = 1.225;

#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct AtmosphereSample {
    pub pressure_pa: f32,
    pub density_kg_per_m3: f32,
    pub temperature_k: f32,
    pub density_ratio: f32,
}

impl Default for AtmosphereSample {
    fn default() -> Self {
        sample_us_standard_atmosphere_1976(0.0)
    }
}

pub fn dynamic_pressure(air_density_kg_per_m3: f32, airspeed_mps: f32) -> f32 {
    0.5 * air_density_kg_per_m3.max(0.0) * airspeed_mps.max(0.0).powi(2)
}

pub fn sample_us_standard_atmosphere_1976(altitude_m: f32) -> AtmosphereSample {
    // Constants from the US Standard Atmosphere 1976 model.
    const G0: f64 = 9.80665;
    const M: f64 = 0.0289644;
    const R: f64 = 8.31432;
    const EARTH_RADIUS_M: f64 = 6_356_766.0;
    const LAYERS: [(f64, f64, f64, f64); 7] = [
        (0.0, 101_325.0, 288.15, -0.0065),
        (11_000.0, 22_632.1, 216.65, 0.0),
        (20_000.0, 5_474.89, 216.65, 0.001),
        (32_000.0, 868.019, 228.65, 0.0028),
        (47_000.0, 110.906, 270.65, 0.0),
        (51_000.0, 66.9389, 270.65, -0.0028),
        (71_000.0, 3.95642, 214.65, -0.002),
    ];

    let geometric_altitude_m = altitude_m.max(0.0) as f64;
    let geopotential_altitude_m =
        EARTH_RADIUS_M * geometric_altitude_m / (EARTH_RADIUS_M + geometric_altitude_m);

    let (base_h, base_p, base_t, lapse_rate) = LAYERS
        .iter()
        .rev()
        .find(|layer| geopotential_altitude_m >= layer.0)
        .copied()
        .unwrap_or(LAYERS[0]);

    let temperature_k = base_t + lapse_rate * (geopotential_altitude_m - base_h);
    let pressure_pa = if lapse_rate.abs() <= f64::EPSILON {
        base_p * (-G0 * M * (geopotential_altitude_m - base_h) / (R * base_t)).exp()
    } else {
        base_p * (base_t / temperature_k).powf(G0 * M / (R * lapse_rate))
    };
    let density_kg_per_m3 = pressure_pa * M / (R * temperature_k);

    AtmosphereSample {
        pressure_pa: pressure_pa as f32,
        density_kg_per_m3: density_kg_per_m3 as f32,
        temperature_k: temperature_k as f32,
        density_ratio: density_kg_per_m3 as f32 / SEA_LEVEL_AIR_DENSITY_KG_PER_M3,
    }
}
