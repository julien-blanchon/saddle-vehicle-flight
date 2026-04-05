# Saddle Vehicle Flight

Reusable aircraft flight toolkit for Bevy covering fixed-wing, helicopter, VTOL, and spacecraft handling, US Standard Atmosphere 1976 sampling, normalized pilot controls, instrument-ready telemetry, and crate-local E2E verification.

The crate is intentionally backend-agnostic. It owns aerodynamic math, control shaping, telemetry, and a lightweight internal rigid-body integrator. If a game wants Avian or another physics backend, the intended pattern is to keep this crate's force and torque outputs and swap only the application layer.

## Quick Start

```toml
[dependencies]
bevy = "0.18"
saddle-vehicle-flight = { git = "https://github.com/julien-blanchon/saddle-vehicle-flight" }
```

```rust
use bevy::prelude::*;
use saddle_vehicle_flight::{FixedWingAircraft, FlightControlInput, FlightPlugin};

#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum DemoState {
    #[default]
    Running,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_state::<DemoState>()
        .add_plugins(FlightPlugin::new(
            OnEnter(DemoState::Running),
            OnExit(DemoState::Running),
            Update,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Name::new("Trainer"),
        FixedWingAircraft::trainer(),
        FixedWingAircraft::trainer_body(),
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
- Positive angle of attack: positive body pitch-up relative to the airflow
- Positive sideslip: positive body-right air-relative velocity component

## Public API

| Type | Purpose |
| --- | --- |
| `FlightPlugin` | Registers the runtime with injectable activate, deactivate, and update schedules |
| `FlightSystems` | Public ordering hooks: `ResolveControls`, `SampleEnvironment`, `ComputeDynamics`, `IntegrateMotion`, `UpdateTelemetry`, `EmitMessages` |
| `FixedWingAircraft` | Fixed-wing configuration component with `trainer()` and `arcade_racer()` presets |
| `HelicopterAircraft` | Helicopter configuration component with `utility()` and `arcade()` presets |
| `VtolAircraft` | Tiltrotor / transition aircraft configuration with `tiltrotor_transport()` preset |
| `SpacecraftConfig` | Pure 6-DOF spacecraft configuration with `fighter()`, `cargo()`, and `arcade_fighter()` presets |
| `FlightBody` | Mass, inertia, gravity, and integration-mode inputs |
| `FlightEnvironment` | Wind, gust, density multiplier, and optional surface altitude |
| `FlightControlInput` | Normalized pilot controls plus trim, VTOL transition, and gear requests |
| `FlightAssist` | Stability augmentation knobs for wings-level, coordinated-turn, and hover leveling |
| `FlightKinematics` | Runtime linear and angular velocity state |
| `LandingGearState` | Runtime gear deployment and contact state |
| `StallState` | Runtime fixed-wing stall state and hysteresis amount |
| `FlightAeroState` | Runtime atmosphere sample, qbar, AoA, sideslip, and air-relative velocities |
| `FlightForces` | Runtime thrust, lift, drag, side force, gravity, and torque breakdown including assist torques |
| `FlightTelemetry` | Instrument/UI ready outputs for airspeed, altitude, world-frame vertical speed, AoA, sideslip, power, VTOL transition, gear, and stall |
| `FlightControlResponse`, `PowerResponse`, `ContactGeometry` | Shared response-curve and contact tuning structs |
| `StallEntered`, `StallRecovered`, `GearStateChanged` | Cross-crate messages for common gameplay reactions |

`FixedWingAircraft`, `HelicopterAircraft`, `VtolAircraft`, and `SpacecraftConfig` auto-require the runtime components they need, so a spawn only needs the aircraft config, a `FlightBody`, and a `Transform` unless the caller wants to override defaults.

## Supported Flight Models

- **Fixed-wing**: lift and induced drag from angle of attack and dynamic pressure, stall hysteresis, post-stall lift degradation, sideslip force, control-surface torque, ground effect, and landing-gear drag. Presets: `trainer()` (realistic) and `arcade_racer()` (no-stall, high authority).
- **Helicopter**: collective-driven main lift, cyclic-style pitch/roll/yaw torque inputs, translational lift, anti-torque, ground effect, and skid/gear contact. Presets: `utility()` (realistic) and `arcade()` (snappy, reduced inertia).
- **VTOL**: tiltrotor-style transition blending with shared runway contact, hover control, wing-borne lift, and instrument-visible transition state. Preset: `tiltrotor_transport()`.
- **Spacecraft**: pure 6-DOF thrust and RCS, no atmosphere, configurable linear drag for game-feel. Presets: `fighter()`, `cargo()`, `arcade_fighter()`.
- **Shared**: wind and gust input, throttle or collective lag, trim biases, assist torques, telemetry, and runtime force inspection.

## Examples

All example apps include live `saddle-pane` controls for wind, assist tuning, and camera offsets.

| Example | Purpose | Run |
| --- | --- | --- |
| `basic_fixed_wing` | Pilot-controlled trainer aircraft with force-vector gizmos and telemetry overlay | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-basic-fixed-wing` |
| `helicopter_hover` | Pilot-controlled utility helicopter with hover and translation | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-helicopter-hover` |
| `cockpit_vtol` | Tiltrotor cockpit demo with hover-to-cruise transition over an airfield | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-cockpit-vtol` |
| `stall_recovery` | Scripted stall entry and recovery for fixed-wing tuning | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-stall-recovery` |
| `wind_gusts` | Persistent crosswind plus animated gusts using `FlightEnvironment` | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-wind-gusts` |
| `instruments` | Camera-follow and overlay focused on telemetry consumers | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-instruments` |
| `control_profiles` | Side-by-side comparison of trainer or utility presets against arcade presets | `cargo run --manifest-path examples/Cargo.toml -p saddle-vehicle-flight-example-control-profiles` |

## Crate-Local Lab

The workspace includes a crate-local standalone lab app at `shared/vehicle/saddle-vehicle-flight/examples/lab`:

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

## BRP

Useful BRP commands against the lab:

```bash
uv run --project .codex/skills/bevy-brp/script brp app launch saddle-vehicle-flight-lab
uv run --project .codex/skills/bevy-brp/script brp world query bevy_ecs::name::Name
uv run --project .codex/skills/bevy-brp/script brp world query bevy_ecs::name::Name bevy_transform::components::transform::Transform
uv run --project .codex/skills/bevy-brp/script brp extras screenshot /tmp/flight_lab.png
uv run --project .codex/skills/bevy-brp/script brp extras shutdown
```

## More Docs

- [Architecture](docs/architecture.md)
- [Configuration](docs/configuration.md)
- [Tuning](docs/tuning.md)
