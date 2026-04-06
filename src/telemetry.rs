use crate::atmosphere::AtmosphereSample;
use bevy::prelude::*;

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
pub struct FlightAeroState {
    pub atmosphere: AtmosphereSample,
    pub altitude_msl_m: f32,
    pub altitude_agl_m: Option<f32>,
    pub airspeed_mps: f32,
    pub forward_speed_mps: f32,
    pub side_speed_mps: f32,
    pub body_vertical_speed_mps: f32,
    pub dynamic_pressure_pa: f32,
    pub angle_of_attack_rad: f32,
    pub sideslip_rad: f32,
}

impl Default for FlightAeroState {
    fn default() -> Self {
        Self {
            atmosphere: AtmosphereSample::default(),
            altitude_msl_m: 0.0,
            altitude_agl_m: None,
            airspeed_mps: 0.0,
            forward_speed_mps: 0.0,
            side_speed_mps: 0.0,
            body_vertical_speed_mps: 0.0,
            dynamic_pressure_pa: 0.0,
            angle_of_attack_rad: 0.0,
            sideslip_rad: 0.0,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct FlightForces {
    pub thrust_world_newtons: Vec3,
    pub lift_world_newtons: Vec3,
    pub drag_world_newtons: Vec3,
    pub side_force_world_newtons: Vec3,
    pub gravity_world_newtons: Vec3,
    pub total_force_world_newtons: Vec3,
    pub control_torque_body_nm: Vec3,
    pub damping_torque_body_nm: Vec3,
    pub assist_torque_body_nm: Vec3,
    pub total_torque_body_nm: Vec3,
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct FlightTelemetry {
    pub true_airspeed_mps: f32,
    pub indicated_airspeed_mps: f32,
    pub altitude_msl_m: f32,
    pub altitude_agl_m: Option<f32>,
    pub vertical_speed_mps: f32,
    pub angle_of_attack_deg: f32,
    pub sideslip_deg: f32,
    pub forward_thrust: f32,
    pub vertical_thrust: f32,
    pub lateral_thrust: f32,
    pub vtol_transition: f32,
    pub gear_position: f32,
    pub gear_deployed: bool,
    pub stalled: bool,
}
