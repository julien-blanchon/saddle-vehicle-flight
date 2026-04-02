use bevy::prelude::*;
use saddle_bevy_e2e::{
    action::Action,
    actions::{assertions, inspect},
    scenario::Scenario,
};
use bevy_enhanced_input::prelude::ContextActivity;
use saddle_vehicle_flight::{FlightControlInput, FlightKinematics, FlightTelemetry, StallState};

use crate::{ActiveVehicle, LabState, support::ExamplePilot};

pub fn scenario_by_name(name: &str) -> Option<Scenario> {
    match name {
        "flight_fixed_wing_smoke" => Some(build_fixed_wing_smoke()),
        "flight_stall_recovery" => Some(build_stall_recovery()),
        "flight_helicopter_hover" => Some(build_helicopter_hover()),
        _ => None,
    }
}

pub fn list_scenarios() -> Vec<&'static str> {
    vec![
        "flight_fixed_wing_smoke",
        "flight_stall_recovery",
        "flight_helicopter_hover",
    ]
}

fn build_fixed_wing_smoke() -> Scenario {
    Scenario::builder("flight_fixed_wing_smoke")
        .description("Verify the fixed-wing aircraft spawns, accelerates, and settles into a real climb from low altitude.")
        .then(Action::Custom(Box::new(|world: &mut World| {
            set_active_vehicle(world, ActiveVehicle::FixedWing);
            let plane = world.resource::<LabState>().plane;
            reset_vehicle(
                world,
                plane,
                Transform::from_xyz(0.0, 6.0, 58.0),
                Vec3::new(0.0, 0.0, -42.0),
            );
            set_controls(
                world,
                plane,
                FlightControlInput {
                    throttle: 0.92,
                    ..default()
                },
            );
        })))
        .then(Action::WaitFrames(45))
        .then(Action::Screenshot("fixed_wing_runup".into()))
        .then(Action::WaitFrames(1))
        .then(Action::Custom(Box::new(|world: &mut World| {
            let plane = world.resource::<LabState>().plane;
            set_controls(
                world,
                plane,
                FlightControlInput {
                    pitch: -0.24,
                    throttle: 0.95,
                    ..default()
                },
            );
        })))
        .then(Action::WaitUntil {
            label: "plane entered climb".into(),
            condition: Box::new(|world| {
                let plane = world.resource::<LabState>().plane;
                world.get::<FlightTelemetry>(plane).is_some_and(|telemetry| {
                    telemetry.altitude_msl_m > 8.0 && telemetry.vertical_speed_mps > 1.0
                })
            }),
            max_frames: 180,
        })
        .then(assertions::custom("plane entity exists", |world| {
            let plane = world.resource::<LabState>().plane;
            world.get::<Transform>(plane).is_some()
        }))
        .then(assertions::custom("plane rotated into climb attitude", |world| {
            let plane = world.resource::<LabState>().plane;
            world
                .get::<FlightTelemetry>(plane)
                .is_some_and(|telemetry| {
                    telemetry.altitude_msl_m > 8.0
                        && telemetry.vertical_speed_mps > 1.0
                        && telemetry.angle_of_attack_deg > 5.0
                        && !telemetry.stalled
                })
        }))
        .then(assertions::custom("plane built useful speed", |world| {
            let plane = world.resource::<LabState>().plane;
            world
                .get::<FlightTelemetry>(plane)
                .is_some_and(|telemetry| telemetry.true_airspeed_mps > 25.0)
        }))
        .then(Action::Screenshot("fixed_wing_climb".into()))
        .then(Action::WaitFrames(1))
        .then(inspect::log_component::<FlightTelemetry>("fixed_wing_smoke_telemetry"))
        .then(assertions::log_summary("flight_fixed_wing_smoke summary"))
        .build()
}

fn build_stall_recovery() -> Scenario {
    Scenario::builder("flight_stall_recovery")
        .description("Force the fixed-wing aircraft into a stall, verify the stall state toggles, then recover.")
        .then(Action::Custom(Box::new(|world: &mut World| {
            set_active_vehicle(world, ActiveVehicle::FixedWing);
            let plane = world.resource::<LabState>().plane;
            reset_vehicle(
                world,
                plane,
                Transform::from_xyz(0.0, 120.0, 30.0),
                Vec3::new(0.0, 0.0, -52.0),
            );
            set_controls(
                world,
                plane,
                FlightControlInput {
                    pitch: 0.88,
                    throttle: 0.40,
                    ..default()
                },
            );
        })))
        .then(Action::WaitFrames(10))
        .then(Action::Screenshot("stall_entry_start".into()))
        .then(Action::WaitFrames(1))
        .then(Action::WaitUntil {
            label: "stall entered".into(),
            condition: Box::new(|world| {
                let plane = world.resource::<LabState>().plane;
                world.get::<StallState>(plane).is_some_and(|stall| stall.is_stalled)
            }),
            max_frames: 240,
        })
        .then(assertions::custom("stall state entered", |world| {
            let plane = world.resource::<LabState>().plane;
            world.get::<StallState>(plane).is_some_and(|stall| stall.is_stalled)
        }))
        .then(Action::Screenshot("stall_entry".into()))
        .then(Action::WaitFrames(1))
        .then(Action::Custom(Box::new(|world: &mut World| {
            let plane = world.resource::<LabState>().plane;
            set_controls(
                world,
                plane,
                FlightControlInput {
                    pitch: -0.30,
                    throttle: 1.0,
                    ..default()
                },
            );
        })))
        .then(Action::WaitUntil {
            label: "stall recovered".into(),
            condition: Box::new(|world| {
                let plane = world.resource::<LabState>().plane;
                world.get::<StallState>(plane).is_some_and(|stall| !stall.is_stalled)
            }),
            max_frames: 300,
        })
        .then(assertions::custom("stall recovered", |world| {
            let plane = world.resource::<LabState>().plane;
            let telemetry = world.get::<FlightTelemetry>(plane);
            let stall = world.get::<StallState>(plane);
            telemetry.is_some_and(|telemetry| telemetry.true_airspeed_mps > 25.0)
                && stall.is_some_and(|stall| !stall.is_stalled)
        }))
        .then(Action::Screenshot("stall_recovery".into()))
        .then(Action::WaitFrames(1))
        .then(inspect::log_component::<FlightTelemetry>("stall_recovery_telemetry"))
        .then(assertions::log_summary("flight_stall_recovery summary"))
        .build()
}

fn build_helicopter_hover() -> Scenario {
    Scenario::builder("flight_helicopter_hover")
        .description("Verify the helicopter can stabilize into a hover and then translate forward under cyclic input.")
        .then(Action::Custom(Box::new(|world: &mut World| {
            set_active_vehicle(world, ActiveVehicle::Helicopter);
            let helicopter = world.resource::<LabState>().helicopter;
            reset_vehicle(
                world,
                helicopter,
                Transform::from_xyz(28.0, 3.2, 0.0),
                Vec3::ZERO,
            );
            set_controls(
                world,
                helicopter,
                FlightControlInput {
                    collective: 0.63,
                    ..default()
                },
            );
        })))
        .then(Action::WaitFrames(180))
        .then(assertions::custom("hover altitude window", |world| {
            let helicopter = world.resource::<LabState>().helicopter;
            world
                .get::<FlightTelemetry>(helicopter)
                .is_some_and(|telemetry| {
                    telemetry.altitude_msl_m > 2.0
                        && telemetry.altitude_msl_m < 7.0
                        && telemetry.vertical_speed_mps.abs() < 1.5
                })
        }))
        .then(Action::Screenshot("helicopter_hover".into()))
        .then(Action::WaitFrames(1))
        .then(Action::Custom(Box::new(|world: &mut World| {
            let helicopter = world.resource::<LabState>().helicopter;
            set_controls(
                world,
                helicopter,
                FlightControlInput {
                    pitch: 0.55,
                    yaw: 0.0,
                    collective: 0.72,
                    ..default()
                },
            );
        })))
        .then(Action::WaitFrames(160))
        .then(assertions::custom("helicopter translated forward", |world| {
            let helicopter = world.resource::<LabState>().helicopter;
            let moved = world.get::<Transform>(helicopter).is_some_and(|transform| {
                transform.translation.xz().distance(Vec2::new(28.0, 0.0)) > 6.0
            });
            let airspeed = world
                .get::<FlightTelemetry>(helicopter)
                .is_some_and(|telemetry| telemetry.true_airspeed_mps > 4.0);
            moved && airspeed
        }))
        .then(Action::Screenshot("helicopter_translate".into()))
        .then(Action::WaitFrames(1))
        .then(inspect::log_component::<FlightTelemetry>("helicopter_hover_telemetry"))
        .then(assertions::log_summary("flight_helicopter_hover summary"))
        .build()
}

fn set_active_vehicle(world: &mut World, active: ActiveVehicle) {
    let state = *world.resource::<LabState>();
    world.resource_mut::<LabState>().active = active;

    let plane_activity = if active == ActiveVehicle::FixedWing {
        ContextActivity::<ExamplePilot>::ACTIVE
    } else {
        ContextActivity::<ExamplePilot>::INACTIVE
    };
    let helicopter_activity = if active == ActiveVehicle::Helicopter {
        ContextActivity::<ExamplePilot>::ACTIVE
    } else {
        ContextActivity::<ExamplePilot>::INACTIVE
    };
    world.entity_mut(state.plane).insert(plane_activity);
    world
        .entity_mut(state.helicopter)
        .insert(helicopter_activity);
}

fn set_controls(world: &mut World, entity: Entity, controls: FlightControlInput) {
    let mut input = world
        .get_mut::<FlightControlInput>(entity)
        .expect("aircraft control input should exist");
    *input = controls;
}

fn reset_vehicle(world: &mut World, entity: Entity, transform: Transform, velocity: Vec3) {
    *world
        .get_mut::<Transform>(entity)
        .expect("transform should exist") = transform;
    *world
        .get_mut::<FlightKinematics>(entity)
        .expect("flight kinematics should exist") = FlightKinematics {
        linear_velocity_world_mps: velocity,
        angular_velocity_body_rps: Vec3::ZERO,
    };
}
