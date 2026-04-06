# Architecture

## Layering

`saddle-vehicle-flight` keeps reusable flight math separate from Bevy orchestration.

Pure or mostly pure logic:

1. `atmosphere.rs`
2. `math.rs`
3. `model/common.rs`
4. `model/fixed_wing.rs`
5. `model/helicopter.rs`
6. `model/vtol.rs`
7. `model/spacecraft.rs`

Bevy-facing orchestration:

1. `components.rs`
2. `config.rs`
3. `messages.rs`
4. `telemetry.rs`
5. `systems/*`
6. `lib.rs`

The main architectural shift is that the runtime no longer branches directly on four independent public config components. Instead, the public surface is split into four orthogonal pieces:

1. `VehicleModel`: the model family and its aerodynamic or inertial data
2. `VehicleActuators`: thrust axes, surface authority, torque authority, and hybrid thrust routing
3. `VehicleControlMap`: how `FlightControlInput` turns into resolved channels
4. `GroundHandling`: per-vehicle contact and damping behavior

That means input semantics, thrust routing, and contact behavior are now configurable without rewriting the core systems.

## Runtime Flow

```text
FlightControlInput
  -> VehicleControlMap
  -> FlightControlChannels
  -> FlightEnvironment / atmosphere sample
  -> motion sample in body space
  -> VehicleModel + VehicleActuators evaluation
  -> FlightAeroState + FlightForces
  -> integrate or hand off to external physics
  -> FlightTelemetry
  -> StallEntered / StallRecovered / GearStateChanged
```

## Schedule Ordering

`FlightSystems` is public and chained in this order:

1. `ResolveControls`
2. `SampleEnvironment`
3. `ComputeDynamics`
4. `IntegrateMotion`
5. `UpdateTelemetry`
6. `EmitMessages`

This keeps downstream reads stable inside one frame:

- control shaping and gear targets settle before any dynamics math runs
- atmosphere density and AGL are current before force evaluation
- telemetry always reflects the same frame's resolved channels and force state
- messages are emitted from final runtime state, not speculative inputs

The plugin accepts injectable activate, deactivate, and update schedules so games can map the runtime into their own state machine or feature pipeline.

## Coordinate Frames

### World frame

- right-handed Bevy 3D world
- `Vec3::Y` is up
- `Transform::translation.y` is mean-sea-level altitude input for atmosphere sampling

### Body frame

- body right = `transform.right()` = local `+X`
- body up = `transform.up()` = local `+Y`
- body forward = `transform.forward()` = local `-Z`

This means aircraft meshes should be authored so the nose points along Bevy forward.

### Air-relative quantities

The runtime computes air-relative motion from:

```text
air velocity = linear velocity - (wind + gust)
```

From that it derives:

- true airspeed magnitude
- body-forward speed
- body-right speed
- body-up speed
- angle of attack
- sideslip
- dynamic pressure

`FlightKinematics::angular_velocity_body_rps` is an axis-aligned body rotation vector:

- `x`: rotation about body right
- `y`: rotation about body up
- `z`: rotation about body forward

## Control Mapping Layer

`VehicleControlMap` resolves the user-authored `FlightControlInput` surface into runtime `FlightControlChannels`.

Each channel binding owns:

- an input source such as `Pitch`, `Throttle`, or `Transition`
- optional trim coupling
- scalar gain and offset
- clamp range
- slew-up and slew-down rates
- axis shaping exponent

This is the layer that makes the crate generic:

- spacecraft no longer hardcode yaw or collective for translation
- hybrid VTOL no longer hardcode a special power handoff in the control system
- alternate games can reuse one model with completely different control semantics

## Model Evaluation Layer

`compute_vehicle_dynamics` is now a single system that queries `VehicleModel`, `VehicleActuators`, and `GroundHandling`, then dispatches into the pure evaluators.

### Fixed-wing path

`evaluate_fixed_wing_with_motion` handles:

1. AoA and sideslip extraction
2. qbar evaluation
3. lift, drag, side-force, and thrust evaluation
4. stall hysteresis target generation
5. control-surface, stability, damping, and assist torques
6. gear drag and ground-effect scaling

### Rotorcraft path

`evaluate_rotorcraft` handles:

1. lift-channel-driven rotor thrust
2. translational lift gain
3. parasite and side drag
4. torque channels and anti-torque coupling
5. hover assist torques
6. ground-effect scaling

### Hybrid path

`evaluate_hybrid` combines the fixed-wing and rotorcraft sub-models, but the combination is now expressed by model and actuator config:

1. `HybridVehicleModel` owns the wingborne blend window
2. `HybridActuators` owns rotor thrust routing from hover axis to cruise axis
3. control-channel mapping decides whether forward and vertical thrust stay coupled or separate
4. the evaluator blends fixed-wing lift or torque with rotorcraft lift or torque using the resolved transition channel

### Spacecraft path

`evaluate_spacecraft` provides a pure 6-DOF thruster model with no atmosphere:

1. forward, lateral, and vertical thrust channels
2. configurable body-axis translation directions
3. configurable angular torque authority
4. configurable angular damping
5. optional linear drag for arcade game-feel
6. zero- or nonzero-gravity operation through `FlightBody`

## Ground Handling Layer

`GroundHandling` replaces family-specific contact branching.

It owns:

- whether contact is enabled at all
- `ContactGeometry`
- longitudinal damping
- lateral damping
- angular damping

Every vehicle family uses the same contact resolver. Different rollout or skid behavior now comes from per-vehicle config, not from hardcoded `"fixed-wing vs VTOL"` rules inside the system.

## Integration Boundary

By default `FlightBody::use_internal_integration = true`, so the crate updates:

- `FlightKinematics`
- `Transform`

using the resolved total force and total torque.

If a project wants an external physics backend, the intended boundary is:

- keep `ResolveControls`, `SampleEnvironment`, `ComputeDynamics`, `UpdateTelemetry`, and `EmitMessages`
- disable internal motion integration on the entity
- read `FlightForces` and apply them through the external backend
- write the resulting `Transform` and `FlightKinematics` back into the shared runtime state

That keeps the reusable atmosphere and flight-model code independent from Avian, Rapier, or project-specific gameplay crates.

## Debug Surface

The crate publishes its debug and instrumentation surface through normal ECS state:

- `FlightControlChannels` for resolved command inspection
- `FlightAeroState` for atmosphere, qbar, AoA, sideslip, and air-relative velocity
- `FlightForces` for lift, drag, thrust, gravity, and torque breakdown
- `FlightTelemetry` for instrument-style UI consumption
- `StallState` and `LandingGearState` for state transitions
- messages for stall and gear changes

The examples and lab deliberately expose these values in overlays and gizmo arrows so BRP and E2E can inspect both behavior and visuals.

## Testing Strategy

The crate verifies the pure and Bevy-facing layers separately:

- pure unit tests for atmosphere sampling and model math
- Bevy integration tests for plugin wiring, generic control resolution, state updates, and message emission
- example workspace presets that exercise the public generic surface
- a crate-local lab with E2E scenarios for fixed-wing climb, stall recovery, rotorcraft hover, and hybrid transition
