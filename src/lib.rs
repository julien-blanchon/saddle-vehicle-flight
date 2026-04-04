mod atmosphere;
mod components;
mod config;
mod math;
mod messages;
mod model;
mod systems;
mod telemetry;

pub use atmosphere::{
    AtmosphereSample, SEA_LEVEL_AIR_DENSITY_KG_PER_M3, dynamic_pressure,
    sample_us_standard_atmosphere_1976,
};
pub use components::{
    FlightAssist, FlightBody, FlightControlInput, FlightEnvironment, FlightKinematics,
    LandingGearState, StallState,
};
pub use config::{
    ContactGeometry, FixedWingAircraft, FlightControlResponse, HelicopterAircraft, PowerResponse,
    VtolAircraft,
};
pub use messages::{GearStateChanged, StallEntered, StallRecovered};
pub use telemetry::{FlightAeroState, FlightForces, FlightTelemetry};

use bevy::{
    app::PostStartup,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

#[derive(SystemSet, Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub enum FlightSystems {
    ResolveControls,
    SampleEnvironment,
    ComputeDynamics,
    IntegrateMotion,
    UpdateTelemetry,
    EmitMessages,
}

#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]
struct NeverDeactivateSchedule;

pub struct FlightPlugin {
    pub activate_schedule: Interned<dyn ScheduleLabel>,
    pub deactivate_schedule: Interned<dyn ScheduleLabel>,
    pub update_schedule: Interned<dyn ScheduleLabel>,
}

impl FlightPlugin {
    pub fn new(
        activate_schedule: impl ScheduleLabel,
        deactivate_schedule: impl ScheduleLabel,
        update_schedule: impl ScheduleLabel,
    ) -> Self {
        Self {
            activate_schedule: activate_schedule.intern(),
            deactivate_schedule: deactivate_schedule.intern(),
            update_schedule: update_schedule.intern(),
        }
    }

    pub fn always_on(update_schedule: impl ScheduleLabel) -> Self {
        Self::new(PostStartup, NeverDeactivateSchedule, update_schedule)
    }
}

impl Default for FlightPlugin {
    fn default() -> Self {
        Self::always_on(Update)
    }
}

impl Plugin for FlightPlugin {
    fn build(&self, app: &mut App) {
        if self.deactivate_schedule == NeverDeactivateSchedule.intern() {
            app.init_schedule(NeverDeactivateSchedule);
        }

        app.init_resource::<systems::activation::FlightRuntime>()
            .add_message::<StallEntered>()
            .add_message::<StallRecovered>()
            .add_message::<GearStateChanged>()
            .register_type::<AtmosphereSample>()
            .register_type::<ContactGeometry>()
            .register_type::<FixedWingAircraft>()
            .register_type::<FlightAeroState>()
            .register_type::<FlightAssist>()
            .register_type::<FlightBody>()
            .register_type::<FlightControlInput>()
            .register_type::<FlightControlResponse>()
            .register_type::<FlightEnvironment>()
            .register_type::<FlightForces>()
            .register_type::<FlightKinematics>()
            .register_type::<FlightTelemetry>()
            .register_type::<HelicopterAircraft>()
            .register_type::<LandingGearState>()
            .register_type::<PowerResponse>()
            .register_type::<StallState>()
            .register_type::<VtolAircraft>()
            .add_systems(
                self.activate_schedule,
                systems::activation::activate_runtime,
            )
            .add_systems(
                self.deactivate_schedule,
                systems::activation::deactivate_runtime,
            )
            .configure_sets(
                self.update_schedule,
                (
                    FlightSystems::ResolveControls,
                    FlightSystems::SampleEnvironment,
                    FlightSystems::ComputeDynamics,
                    FlightSystems::IntegrateMotion,
                    FlightSystems::UpdateTelemetry,
                    FlightSystems::EmitMessages,
                )
                    .chain(),
            )
            .add_systems(
                self.update_schedule,
                (
                    systems::control::resolve_controls.in_set(FlightSystems::ResolveControls),
                    systems::environment::sample_environment
                        .in_set(FlightSystems::SampleEnvironment),
                    systems::dynamics::compute_fixed_wing_dynamics
                        .in_set(FlightSystems::ComputeDynamics),
                    systems::dynamics::compute_helicopter_dynamics
                        .in_set(FlightSystems::ComputeDynamics),
                    systems::dynamics::compute_vtol_dynamics.in_set(FlightSystems::ComputeDynamics),
                    (
                        systems::integration::integrate_motion
                            .in_set(FlightSystems::IntegrateMotion),
                        systems::integration::resolve_ground_contact
                            .in_set(FlightSystems::IntegrateMotion),
                        systems::integration::sanitize_state.in_set(FlightSystems::IntegrateMotion),
                    )
                        .chain(),
                    systems::telemetry::update_telemetry.in_set(FlightSystems::UpdateTelemetry),
                    systems::messages::emit_stall_messages.in_set(FlightSystems::EmitMessages),
                    systems::messages::emit_gear_messages.in_set(FlightSystems::EmitMessages),
                )
                    .run_if(systems::activation::runtime_is_active),
            );
    }
}

#[cfg(test)]
#[path = "atmosphere_tests.rs"]
mod atmosphere_tests;

#[cfg(test)]
#[path = "model_tests.rs"]
mod model_tests;

#[cfg(test)]
#[path = "systems_tests.rs"]
mod systems_tests;

#[cfg(test)]
#[path = "plugin_tests.rs"]
mod plugin_tests;

#[cfg(test)]
#[path = "vtol_tests.rs"]
mod vtol_tests;
