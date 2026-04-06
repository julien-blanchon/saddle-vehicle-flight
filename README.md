# Saddle Vehicle Flight

Reusable flight toolkit for Bevy 0.18 covering fixed-wing, rotorcraft, hybrid VTOL, and spacecraft handling, US Standard Atmosphere 1976 sampling, configurable control-channel mapping, actuator routing, telemetry, and crate-local E2E verification.

The crate is intentionally backend-agnostic. It owns atmosphere sampling, aerodynamic math, actuator-driven force or torque evaluation, telemetry, and a lightweight internal rigid-body integrator. If a game wants Avian, Rapier, or another physics backend, the intended pattern is to keep this crate's force and torque outputs and replace only the integration or application layer.

## Quick Start

```toml
[dependencies]
bevy = "0.18"
saddle-vehicle-flight = { git = "https://github.com/julien-blanchon/saddle-vehicle-flight" }
```

```rust
use bevy::prelude::*;
use saddle_vehicle_flight::{
    ChannelResponse, ContactGeometry, ControlChannelBinding, ControlInputSource,
    FixedWingActuators, FixedWingModel, FlightBody, FlightControlInput, FlightPlugin,
    GroundHandling, TrimInputSource, VehicleActuators, VehicleControlMap, VehicleModel,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(FlightPlugin::default())
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Name::new("Trainer"),
        VehicleModel::fixed_wing(FixedWingModel::default()),
        VehicleActuators::fixed_wing(FixedWingActuators::default()),
        VehicleControlMap {
            pitch: ControlChannelBinding {
                source: ControlInputSource::Pitch,
                trim: Some(TrimInputSource::Pitch),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 5.0,
                    fall_per_second: 5.0,
                    exponent: 1.25,
                },
                ..default()
            },
            roll: ControlChannelBinding {
                source: ControlInputSource::Roll,
                trim: Some(TrimInputSource::Roll),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 6.0,
                    fall_per_second: 6.0,
                    exponent: 1.15,
                },
                ..default()
            },
            yaw: ControlChannelBinding {
                source: ControlInputSource::Yaw,
                trim: Some(TrimInputSource::Yaw),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 4.0,
                    fall_per_second: 4.0,
                    exponent: 1.0,
                },
                ..default()
            },
            forward_thrust: ControlChannelBinding {
                source: ControlInputSource::Throttle,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 0.65,
                    fall_per_second: 1.0,
                    exponent: 1.0,
                },
                ..default()
            },
            ..default()
        },
        GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.15,
                retractable: true,
                extension_rate_per_second: 0.75,
                ground_effect_height_m: 5.0,
                ground_effect_boost: 0.15,
            },
            longitudinal_damping_per_second: 0.14,
            lateral_damping_per_second: 1.4,
            angular_damping_per_second: 1.4,
        },
        FlightBody::new(980.0, Vec3::new(900.0, 1_450.0, 1_700.0)),
        FlightControlInput {
            throttle: 0.6,
            ..default()
        },
        Transform::from_xyz(0.0, 1.2, 40.0),
    ));
}
```

For examples and crate-local labs, `FlightPlugin::default()` is the always-on entrypoint. It activates on `PostStartup`, never deactivates, and updates in `Update`.

## Coordinate And Unit Conventions

- Distances: meters
- Velocities: meters per second
- Angular rates: radians per second
- Forces: newtons
- Torques: newton-meters
- Altitude: meters above mean sea level unless a field explicitly says AGL
- Atmosphere: US Standard Atmosphere 1976 sampled from `Transform::translation.y`
- Aircraft forward: `Transform::forward()` which in Bevy is local `-Z`
- Body axes: right = local `+X`, up = local `+Y`, forward = local `-Z`
- `FlightKinematics::angular_velocity_body_rps`: axis-aligned body rotation vector `(about right, about up, about forward)`

## Public API

| Type | Purpose |
| --- | --- |
| `FlightPlugin` | Registers the runtime with injectable activate, deactivate, and update schedules |
| `FlightSystems` | Public ordering hooks: `ResolveControls`, `SampleEnvironment`, `ComputeDynamics`, `IntegrateMotion`, `UpdateTelemetry`, `EmitMessages` |
| `VehicleModel` / `VehicleModelKind` | Selects the aerodynamic or inertial model path: fixed-wing, rotorcraft, hybrid, or spacecraft |
| `VehicleActuators` / `VehicleActuatorKind` | Describes thrust axes, control-surface authority, torque authority, and hybrid thrust routing |
| `VehicleControlMap` | Maps `FlightControlInput` fields into resolved control channels with per-channel shaping and slew |
| `GroundHandling` | Per-vehicle contact, ground-effect, damping, and retractability settings |
| `FixedWingModel`, `RotorcraftModel`, `HybridVehicleModel`, `SpacecraftModel` | Model-family data only; no baked-in preset control mapping or ground rules |
| `FixedWingActuators`, `RotorcraftActuators`, `HybridActuators`, `SpacecraftActuators` | Model-family actuation and thrust-routing configuration |
| `FlightControlChannels` | Runtime resolved channels after `VehicleControlMap` shaping |
| `FlightBody`, `FlightEnvironment`, `FlightControlInput`, `FlightAssist` | Shared runtime or authoring surface |
| `FlightAeroState`, `FlightForces`, `FlightTelemetry` | Debug and instrumentation surface |
| `StallEntered`, `StallRecovered`, `GearStateChanged` | Cross-crate messages for common gameplay reactions |

`VehicleModel` auto-requires the shared runtime components it needs, including `FlightControlChannels`, so a spawn only needs to add the model, actuators, control map, and any non-default body or ground-handling config.

## Why The Surface Changed

The runtime no longer hardcodes control-family branches such as "spacecraft uses yaw for lateral RCS" or "VTOL power must be routed as fixed-wing plus rotorcraft in-core". Instead:

- `VehicleControlMap` decides which pilot input drives each resolved channel.
- `VehicleActuators` decides how each resolved channel turns into force or torque.
- `GroundHandling` decides how contact behaves, independent of vehicle family.
- Named presets moved out of the library API into the example workspace under `examples/presets`.

That keeps the library surface generic and lets two games use the same model with different mappings, different thrust routing, or different ground behavior.

## Supported Model Paths

- **Fixed-wing**: lift and induced drag from angle of attack and dynamic pressure, stall hysteresis, post-stall lift degradation, sideslip force, control-surface torque, ground effect, and landing-gear drag.
- **Rotorcraft**: collective-style lift channel, cyclic-style torque channels, translational lift, anti-torque coupling, parasite drag, and hover assistance.
- **Hybrid VTOL**: fixed-wing and rotorcraft sub-models blended through a model config plus actuator-controlled thrust tilt routing.
- **Spacecraft**: pure 6-DOF thrust and torque with configurable linear drag and zero-atmosphere operation.

## Example Presets

Named presets such as `arcade_racer`, `cargo`, and `tiltrotor_transport` now live in the example workspace at `examples/presets`. The library keeps only generic defaults and low-level config structs.

## Examples

All example apps include live `saddle-pane` controls for wind, assists, and camera tuning.

| Example | Purpose | Run |
| --- | --- | --- |
| `basic_fixed_wing` | Pilot-controlled baseline fixed-wing setup using the example trainer preset | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-basic-fixed-wing` |
| `helicopter_hover` | Pilot-controlled utility rotorcraft hover and translation | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-helicopter-hover` |
| `cockpit_vtol` | Tiltrotor cockpit demo with hover-to-cruise transition over an airfield | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-cockpit-vtol` |
| `stall_recovery` | Scripted stall entry and recovery using the fixed-wing example preset | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-stall-recovery` |
| `wind_gusts` | Persistent crosswind plus animated gusts using `FlightEnvironment` | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-wind-gusts` |
| `instruments` | Camera-follow and overlay focused on telemetry consumers | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-instruments` |
| `control_profiles` | Side-by-side comparison of baseline and arcade example presets | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-control-profiles` |

## Crate-Local Lab

The workspace includes a crate-local standalone lab app at `examples/lab`:

```bash
cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-lab
```

E2E verification commands:

```bash
cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-lab --features e2e -- flight_fixed_wing_smoke
cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-lab --features e2e -- flight_stall_recovery
cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-lab --features e2e -- flight_helicopter_hover
cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-lab --features e2e -- flight_vtol_transition
```

## More Docs

- [Architecture](docs/architecture.md)
- [Configuration](docs/configuration.md)
- [Tuning](docs/tuning.md)
