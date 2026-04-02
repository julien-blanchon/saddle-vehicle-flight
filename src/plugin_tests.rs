use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;

use crate::{FlightPlugin, FlightSystems};

#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]
struct NeverDeactivate;

#[test]
fn plugin_builds_with_injectable_schedules() {
    let mut app = App::new();
    app.init_schedule(NeverDeactivate);
    app.add_plugins(FlightPlugin::new(Startup, NeverDeactivate, Update));

    assert!(
        app.world()
            .contains_resource::<crate::systems::activation::FlightRuntime>()
    );
    let _ = FlightSystems::ResolveControls;
}
