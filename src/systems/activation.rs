use bevy::prelude::*;

#[derive(Resource, Debug, Clone, Copy, Default)]
pub(crate) struct FlightRuntime {
    pub active: bool,
}

pub(crate) fn activate_runtime(mut runtime: ResMut<FlightRuntime>) {
    runtime.active = true;
}

pub(crate) fn deactivate_runtime(mut runtime: ResMut<FlightRuntime>) {
    runtime.active = false;
}

pub(crate) fn runtime_is_active(runtime: Res<FlightRuntime>) -> bool {
    runtime.active
}
