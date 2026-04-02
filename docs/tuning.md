# Tuning

## Baseline Workflow

Tune aircraft in this order:

1. Pick mass and inertia in `FlightBody`.
2. Make the aircraft hold altitude at a believable cruise power setting.
3. Tune control response slew and authority.
4. Tune damping and stability.
5. Tune stall or hover behavior.
6. Only then add assists, gusts, and contact polish.

If several knobs appear to fight each other, reset to a simpler case:

- zero wind
- moderate altitude
- no trim
- assists disabled
- gear fixed down

## Mass And Inertia

### `FlightBody::mass_kg`

- Too low: every force looks exaggerated, takeoff and hover become twitchy.
- Too high: the aircraft feels unresponsive and may never climb.

### `FlightBody::inertia_kgm2`

- Raise the body-right axis to slow pitch rotation.
- Raise the body-up axis to slow yaw rotation.
- Raise the body-forward axis to slow roll rotation.

If the aircraft feels visually plausible in translation but snaps too fast in attitude, inertia is usually the first place to adjust.

## Fixed-Wing Tuning

### Lift And Drag

Primary lift knobs:

- `wing_area_m2`
- `cl0`
- `lift_curve_slope_per_rad`
- `max_lift_coefficient`

Primary drag knobs:

- `zero_lift_drag_coefficient`
- `induced_drag_factor`
- `stall_drag_coefficient`
- `gear_drag_coefficient`

Practical guidance:

- If takeoff speed is too high, increase lift before adding more thrust.
- If cruise speed is too high, raise parasite drag before weakening controls.
- If climb is strong but turns bleed no energy, induced drag is probably too low.

### Stability And Damping

- `pitch_stability` and `pitch_rate_damping` control how quickly the nose stops bobbing.
- `roll_stability` and `roll_rate_damping` control dutch-roll-like lateral settling.
- `yaw_stability` and `yaw_rate_damping` help keep sideslip under control.

Use damping first to kill oscillation. Use stability terms to bias the aircraft back toward coordinated flight.

### Control Authority

- `elevator_authority`: pitch response and flare authority
- `aileron_authority`: roll rate and bank capture
- `rudder_authority`: yaw trimming and sideslip cleanup

If the aircraft can stall but cannot recover with sensible inputs, elevator authority is often too weak or the stall hysteresis is too sticky.

### Stall

Relevant knobs:

- `stall_alpha_rad`
- `recovery_alpha_rad`
- `stall_response_per_second`
- `post_stall_lift_coefficient`
- `stall_drag_coefficient`

Patterns:

- Harder, more abrupt break: larger gap between pre-stall lift and post-stall lift, more stall drag.
- Softer trainer-like stall: higher `recovery_alpha_rad`, moderate `stall_response_per_second`, less dramatic post-stall drop.
- If the aircraft flickers between stalled and recovered every frame, widen the hysteresis gap between entry and recovery AoA.

## Helicopter Tuning

### Hover

Start with:

- `max_main_lift_newtons`
- `FlightBody::mass_kg`
- `power_response.collective_*`

At a neutral hover test, the helicopter should hold altitude with collective somewhere near the middle of the range instead of requiring either `0.1` or `0.95`.

### Translational Lift

Relevant knobs:

- `translational_lift_gain`
- `translational_lift_full_speed_mps`

If the helicopter feels glued in place and never benefits from forward motion, increase the gain or reduce the speed needed to reach it.

### Yaw And Anti-Torque

Relevant knobs:

- `yaw_torque_authority`
- `anti_torque_per_collective`

If the helicopter yaws uncontrollably upward under collective, anti-torque is too strong relative to pedal authority. If collective changes feel completely disconnected from yaw workload, anti-torque is too weak.

### Hover Stability

`FlightAssist::hover_leveling` is intentionally simple. Use it for arcade-friendly or camera-friendly helicopters, but keep it near zero if the goal is a more manual hover.

## Response Curves And Trim

`FlightControlResponse` shapes how quickly the runtime chases the requested input. It does not directly change the aerodynamic or rotor authority.

Recommended pattern:

- use `*_rate_per_second` for feel and spool-up
- use aircraft authority fields for actual force or torque power
- use exponents above `1.0` to make center-stick motion calmer without losing full-range authority

Trim is best kept small. Large trim values are usually a sign that the aircraft or center-of-mass assumptions need retuning instead.

## Wind, Gusts, And Density

`FlightEnvironment` is the external forcing surface:

- `wind_world_mps` for persistent crosswind or weather
- `gust_world_mps` for transient disturbances
- `density_multiplier` for stylized weather, thin atmosphere, or debug cases

Tune aircraft in still air first. Then introduce wind. Otherwise crosswind compensation can hide basic stability problems.

## Contact And Ground Effect

`ContactGeometry` controls the simple runway or skid-contact model.

Checklist:

- If the aircraft visibly sinks into the runway, increase `contact_offset_below_origin_m`.
- If touchdown slides forever, increase `surface_damping_per_second`.
- If flare or hover near the ground feels flat, increase `ground_effect_boost`.
- If ground effect feels like a trampoline, reduce `ground_effect_boost` or lower `ground_effect_height_m`.

## Debug Checklist

When behavior looks wrong, inspect these in order:

1. `FlightAeroState::airspeed_mps`
2. `FlightAeroState::angle_of_attack_rad`
3. `FlightAeroState::sideslip_rad`
4. `FlightForces::lift_world_newtons`
5. `FlightForces::drag_world_newtons`
6. `FlightForces::thrust_world_newtons`
7. `FlightForces::total_torque_body_nm`
8. `LandingGearState` and `StallState`

Typical failure patterns:

- Aircraft accelerates but never climbs:
  lift too low, mass too high, or mesh not authored facing Bevy forward
- Aircraft instantly tumbles:
  inertia too low or control authority too high
- Helicopter hovers only at extreme collective:
  `max_main_lift_newtons` and `mass_kg` are mismatched
- Sideslip always grows during turns:
  `yaw_stability` or `rudder_authority` too low
- Stall never happens:
  `stall_alpha_rad` too high or lift slope too low
- Stall never clears:
  `recovery_alpha_rad` too high, recovery inputs too weak, or stall blend too sticky

## Verification Loop

Use the examples and lab as a repeatable tuning harness:

```bash
cargo run -p saddle-vehicle-flight --example basic_fixed_wing
cargo run -p saddle-vehicle-flight --example wind_gusts
cargo run -p saddle-vehicle-flight --example control_profiles
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_fixed_wing_smoke
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_stall_recovery
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_helicopter_hover
```

When visual behavior matters, keep the E2E screenshots and BRP force inspection in the loop instead of tuning from numeric values alone.
