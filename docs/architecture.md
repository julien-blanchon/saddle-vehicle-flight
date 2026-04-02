# Architecture

## Layering

`saddle-vehicle-flight` keeps reusable flight math separate from Bevy orchestration.

Pure or mostly pure logic:

1. `atmosphere.rs`
2. `math.rs`
3. `model/common.rs`
4. `model/fixed_wing.rs`
5. `model/helicopter.rs`

Bevy-facing orchestration:

1. `components.rs`
2. `config.rs`
3. `messages.rs`
4. `telemetry.rs`
5. `systems/*`
6. `lib.rs`

The pure modules own atmosphere sampling, angle-of-attack and sideslip math, coefficient evaluation, and force or torque composition. The Bevy layer only resolves control state, updates runtime components, integrates motion when requested, and emits messages.

## Runtime Flow

```text
FlightControlInput
  -> ResolveControls
  -> FlightEnvironment / atmosphere sample
  -> sample body-relative motion
  -> fixed-wing or helicopter model path
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

- control shaping and gear targets settle before any aerodynamic math runs
- atmosphere density and AGL are current before force evaluation
- telemetry always reflects the same frame's resolved aero state
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

The fixed-wing model internally maps these to pitch, yaw, and roll damping terms respectively.

## Fixed-Wing Path

`compute_fixed_wing_dynamics` performs:

1. motion sampling in body space
2. AoA and sideslip extraction
3. qbar evaluation using current density
4. lift, drag, side-force, and thrust evaluation
5. stall hysteresis update and post-stall lift degradation
6. stability, damping, control, and assist torque composition
7. ground-effect and landing-gear drag contribution

The model is coefficient-driven enough for sim-lite tuning, but intentionally simpler than a full six-degree-of-freedom study-level FDM.

## Helicopter Path

`compute_helicopter_dynamics` uses a separate internal model that resolves:

1. collective-driven rotor lift
2. translational-lift gain with forward speed
3. parasite and side drag
4. cyclic-style pitch and roll torques
5. yaw torque and anti-torque coupling
6. hover-leveling and coordinated-turn assist torques
7. ground effect and contact behavior

The helicopter path is intentionally generic. It does not simulate blade-element aerodynamics, vortex ring state, or tail-rotor geometry in detail.

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

## Contact And Ground Effect

`ContactGeometry` provides a minimal runway or skid-contact model:

- optional retractable gear scaling of contact offset
- vertical clamp against `FlightEnvironment::surface_altitude_msl_m`
- vertical velocity cancellation when penetrating the surface
- configurable horizontal and angular damping while in contact
- low-altitude ground-effect lift multiplier

This is intentionally simple and deterministic. It is good enough for examples, labs, AI traffic, and sim-lite gameplay, but not a full tire or suspension model.

## Debug Surface

The crate publishes its debug and instrumentation surface through normal ECS state:

- `FlightAeroState` for atmosphere, qbar, AoA, sideslip, and air-relative velocity
- `FlightForces` for lift, drag, thrust, gravity, and control/damping/assist torque breakdown
- `FlightTelemetry` for instrument-style UI consumption, including world-frame climb or descent rate
- `StallState` and `LandingGearState` for state transitions
- messages for stall and gear changes

The examples and lab deliberately expose these values in overlays and gizmo arrows so BRP and E2E can inspect both behavior and visuals.

## Testing Strategy

The crate verifies the pure and Bevy-facing layers separately:

- pure unit tests for US Standard Atmosphere sampling and model math
- Bevy integration tests for plugin wiring, system ordering, state updates, and message emission
- focused examples for fixed-wing, helicopter, stall, wind, and telemetry use cases
- a crate-local lab with E2E scenarios for fixed-wing takeoff, stall recovery, and helicopter hover or translation
