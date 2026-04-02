use bevy::prelude::*;

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
pub struct FlightBody {
    pub mass_kg: f32,
    pub inertia_kgm2: Vec3,
    pub gravity_acceleration_mps2: f32,
    pub use_internal_integration: bool,
}

impl FlightBody {
    pub fn new(mass_kg: f32, inertia_kgm2: Vec3) -> Self {
        Self {
            mass_kg,
            inertia_kgm2,
            ..default()
        }
    }
}

impl Default for FlightBody {
    fn default() -> Self {
        Self {
            mass_kg: 1_000.0,
            inertia_kgm2: Vec3::new(1_200.0, 1_600.0, 1_900.0),
            gravity_acceleration_mps2: 9.80665,
            use_internal_integration: true,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
pub struct FlightEnvironment {
    pub wind_world_mps: Vec3,
    pub gust_world_mps: Vec3,
    pub density_multiplier: f32,
    pub surface_altitude_msl_m: Option<f32>,
}

impl FlightEnvironment {
    pub fn airmass_velocity_world(self) -> Vec3 {
        self.wind_world_mps + self.gust_world_mps
    }
}

impl Default for FlightEnvironment {
    fn default() -> Self {
        Self {
            wind_world_mps: Vec3::ZERO,
            gust_world_mps: Vec3::ZERO,
            density_multiplier: 1.0,
            surface_altitude_msl_m: None,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct FlightControlInput {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub collective: f32,
    pub pitch_trim: f32,
    pub roll_trim: f32,
    pub yaw_trim: f32,
    pub requested_gear_down: Option<bool>,
    pub toggle_gear: bool,
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct FlightAssist {
    pub wings_leveling: f32,
    pub coordinated_turn: f32,
    pub hover_leveling: f32,
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct FlightKinematics {
    pub linear_velocity_world_mps: Vec3,
    pub angular_velocity_body_rps: Vec3,
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Debug)]
pub struct LandingGearState {
    pub target_deployed: bool,
    pub position: f32,
    pub contact: bool,
}

impl LandingGearState {
    pub fn deployed(self) -> bool {
        self.position >= 0.5
    }
}

impl Default for LandingGearState {
    fn default() -> Self {
        Self {
            target_deployed: true,
            position: 1.0,
            contact: false,
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component, Debug)]
pub struct StallState {
    pub is_stalled: bool,
    pub amount: f32,
}

#[derive(Component, Debug, Clone, Copy, Default)]
pub(crate) struct ResolvedFlightControls {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub collective: f32,
    pub gear_toggle_latched: bool,
}

#[derive(Component, Debug, Clone, Copy, Default)]
pub(crate) struct FlightMessageState {
    pub last_stalled: bool,
    pub last_gear_deployed: bool,
}
