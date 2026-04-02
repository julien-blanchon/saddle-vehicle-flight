use crate::{
    SEA_LEVEL_AIR_DENSITY_KG_PER_M3, dynamic_pressure, sample_us_standard_atmosphere_1976,
};

#[test]
fn sea_level_matches_standard_atmosphere() {
    let sample = sample_us_standard_atmosphere_1976(0.0);
    assert!((sample.temperature_k - 288.15).abs() < 0.01);
    assert!((sample.pressure_pa - 101_325.0).abs() < 5.0);
    assert!((sample.density_kg_per_m3 - SEA_LEVEL_AIR_DENSITY_KG_PER_M3).abs() < 0.002);
}

#[test]
fn density_decreases_with_altitude() {
    let low = sample_us_standard_atmosphere_1976(0.0);
    let high = sample_us_standard_atmosphere_1976(5_000.0);
    assert!(high.density_kg_per_m3 < low.density_kg_per_m3);
}

#[test]
fn tropopause_sample_is_reasonable() {
    let sample = sample_us_standard_atmosphere_1976(11_000.0);
    assert!((sample.temperature_k - 216.8).abs() < 0.25);
    assert!((sample.pressure_pa - 22_700.0).abs() < 120.0);
    assert!((sample.density_kg_per_m3 - 0.365).abs() < 0.02);
}

#[test]
fn dynamic_pressure_matches_half_rho_v_squared() {
    let q = dynamic_pressure(1.225, 50.0);
    assert!((q - 1_531.25).abs() < 0.01);
}
