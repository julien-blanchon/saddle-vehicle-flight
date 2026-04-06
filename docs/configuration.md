# Configuration

Defaults below refer to each type's `Default` implementation unless the section says otherwise.

## Conventions

- Distances are meters.
- Speeds are meters per second.
- Angles stored in config are radians unless a field name says `deg`.
- Runtime telemetry converts AoA and sideslip to degrees for UI.
- `FlightControlInput` fields remain the author-facing input surface.
- `VehicleControlMap` converts those input fields into runtime `FlightControlChannels`.

## Shared Runtime Inputs

### `FlightBody`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `mass_kg` | `f32` | `1000.0` | Converts resolved force into linear acceleration |
| `inertia_kgm2` | `Vec3` | `(1200, 1600, 1900)` | Converts resolved torque into angular acceleration |
| `gravity_acceleration_mps2` | `f32` | `9.80665` | Gravity magnitude applied every frame |
| `use_internal_integration` | `bool` | `true` | When `true`, the crate updates `Transform` and `FlightKinematics` directly |

### `FlightEnvironment`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `wind_world_mps` | `Vec3` | `Vec3::ZERO` | Persistent airmass velocity |
| `gust_world_mps` | `Vec3` | `Vec3::ZERO` | Additive transient wind |
| `density_multiplier` | `f32` | `1.0` | Multiplies sampled atmosphere density |
| `surface_altitude_msl_m` | `Option<f32>` | `None` | Enables AGL reporting and ground contact |

### `FlightControlInput`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `pitch`, `roll`, `yaw` | `f32` | `0.0` | Pilot-facing angular inputs before mapping |
| `throttle`, `collective` | `f32` | `0.0` | Pilot-facing power inputs before mapping |
| `vtol_transition` | `f32` | `0.0` | Pilot-facing transition input before mapping |
| `pitch_trim`, `roll_trim`, `yaw_trim` | `f32` | `0.0` | Optional trim bias consumed by `VehicleControlMap` |
| `requested_gear_down` | `Option<bool>` | `None` | Direct gear target override |
| `toggle_gear` | `bool` | `false` | Edge-triggered gear toggle |

### `FlightAssist`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `wings_leveling` | `f32` | `0.0` | Adds roll or pitch stabilization for winged craft |
| `coordinated_turn` | `f32` | `0.0` | Adds yaw correction from sideslip |
| `hover_leveling` | `f32` | `0.0` | Adds rotorcraft attitude stabilization |

## Control Mapping

### `ControlInputSource`

The input source for a channel binding:

- `None`
- `Pitch`
- `Roll`
- `Yaw`
- `Throttle`
- `Collective`
- `Transition`

### `TrimInputSource`

Optional trim source for a channel binding:

- `Pitch`
- `Roll`
- `Yaw`

### `ChannelResponse`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `rise_per_second` | `f32` | `8.0` | Max slew rate when the channel target increases |
| `fall_per_second` | `f32` | `8.0` | Max slew rate when the channel target decreases |
| `exponent` | `f32` | `1.0` | Shapes low-end stick sensitivity |

### `ControlChannelBinding`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `source` | `ControlInputSource` | `None` | Which `FlightControlInput` field drives the channel |
| `trim` | `Option<TrimInputSource>` | `None` | Optional trim source |
| `trim_scale` | `f32` | `0.0` | Trim gain applied before clamp |
| `scale` | `f32` | `1.0` | Multiplies the shaped source value |
| `offset` | `f32` | `0.0` | Adds a post-scale bias |
| `clamp_min` / `clamp_max` | `f32` | `-1.0 / 1.0` | Final bounds for the resolved channel |
| `response` | `ChannelResponse` | default response | Slew and shaping for the channel |

### `VehicleControlMap`

`VehicleControlMap` owns seven resolved channels:

- `pitch`
- `roll`
- `yaw`
- `forward_thrust`
- `vertical_thrust`
- `lateral_thrust`
- `transition`

This is the primary genericization layer in the crate. Examples:

- a fixed-wing craft can map `Throttle -> forward_thrust`
- a rotorcraft can map `Collective -> vertical_thrust`
- a spacecraft can map `Collective -> vertical_thrust` and `Yaw -> lateral_thrust`
- a hybrid craft can map `Transition -> transition` without special runtime logic

## Ground Handling

### `ContactGeometry`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `contact_offset_below_origin_m` | `f32` | `1.0` | Distance from the entity origin to the contact point |
| `retractable` | `bool` | `true` | Whether gear position scales contact offset |
| `extension_rate_per_second` | `f32` | `1.0` | Gear extension or retraction speed |
| `ground_effect_height_m` | `f32` | `4.0` | AGL window where ground effect is applied |
| `ground_effect_boost` | `f32` | `0.12` | Additional lift multiplier near the surface |

### `GroundHandling`

| Field | Type | Default | Effect |
| --- | --- | --- | --- |
| `enabled` | `bool` | `false` | Enables actual contact resolution |
| `contact_geometry` | `ContactGeometry` | default contact | Gear and ground-effect geometry |
| `longitudinal_damping_per_second` | `f32` | `1.8` | Forward rollout damping in contact |
| `lateral_damping_per_second` | `f32` | `1.8` | Side-slip damping in contact |
| `angular_damping_per_second` | `f32` | `1.8` | Angular damping in contact |

`GroundHandling` now replaces the old family-specific contact behavior. Fixed-wing, rotorcraft, and hybrid craft all use the same contact system with different config values.

## Model Data

### `FixedWingModel`

`FixedWingModel` contains aerodynamic and stability data only:

- geometry: `wing_area_m2`, `wingspan_m`, `mean_chord_m`
- lift and drag: `cl0`, `lift_curve_slope_per_rad`, `max_lift_coefficient`, `post_stall_lift_coefficient`, `zero_lift_drag_coefficient`, `induced_drag_factor`, `stall_drag_coefficient`
- lateral response: `side_force_slope_per_rad`, `gear_drag_coefficient`
- damping and stability: `roll_rate_damping`, `pitch_rate_damping`, `yaw_rate_damping`, `roll_stability`, `pitch_stability`, `yaw_stability`
- stall tuning: `stall_alpha_rad`, `recovery_alpha_rad`, `stall_response_per_second`

### `FixedWingActuators`

`FixedWingActuators` contains the control and propulsion side:

- `thrust_axis_local`
- `max_forward_thrust_newtons`
- `aileron_authority`
- `elevator_authority`
- `rudder_authority`
- `trim_authority`

### `RotorcraftModel`

`RotorcraftModel` contains rotor-disc and drag behavior:

- `rotor_disc_area_m2`
- `parasite_drag_coefficient`
- `side_drag_coefficient`
- `translational_lift_gain`
- `translational_lift_full_speed_mps`
- `angular_damping`

### `RotorcraftActuators`

`RotorcraftActuators` contains lift and torque authority:

- `lift_axis_local`
- `max_lift_newtons`
- `pitch_torque_authority`
- `roll_torque_authority`
- `yaw_torque_authority`
- `anti_torque_per_vertical_thrust`
- `trim_authority`

### `HybridVehicleModel`

`HybridVehicleModel` combines:

- `fixed_wing: FixedWingModel`
- `rotorcraft: RotorcraftModel`
- `wingborne_blend_start`
- `wingborne_blend_end`

`wingborne_blend(transition)` converts the resolved transition channel into the blend factor used by the hybrid evaluator.

### `HybridActuators`

`HybridActuators` combines:

- `fixed_wing: FixedWingActuators`
- `rotorcraft: RotorcraftActuators`
- `rotor_thrust_routing: ThrustRouting`

### `ThrustRouting`

`ThrustRouting` defines how hybrid rotor thrust tilts:

- `hover_axis_local`
- `cruise_axis_local`

The runtime interpolates between those axes using the resolved transition channel.

### `SpacecraftModel`

`SpacecraftModel` is intentionally small:

- `angular_damping`
- `linear_drag_coefficient`

### `SpacecraftActuators`

`SpacecraftActuators` owns translation and torque routing:

- `forward_axis_local`
- `lateral_axis_local`
- `vertical_axis_local`
- `max_forward_thrust_newtons`
- `max_lateral_thrust_newtons`
- `max_vertical_thrust_newtons`
- `pitch_torque_authority`
- `roll_torque_authority`
- `yaw_torque_authority`

## Top-Level Assembly

### `VehicleModel`

`VehicleModel` is the component inserted on the entity. Its `kind` field is one of:

- `VehicleModelKind::FixedWing(FixedWingModel)`
- `VehicleModelKind::Rotorcraft(RotorcraftModel)`
- `VehicleModelKind::Hybrid(HybridVehicleModel)`
- `VehicleModelKind::Spacecraft(SpacecraftModel)`

`VehicleModel` auto-requires the shared runtime state, including:

- `Transform`
- `GlobalTransform`
- `FlightBody`
- `FlightAssist`
- `FlightAeroState`
- `FlightControlChannels`
- `FlightControlInput`
- `FlightEnvironment`
- `FlightForces`
- `FlightKinematics`
- `LandingGearState`
- `FlightTelemetry`
- `StallState`

### `VehicleActuators`

`VehicleActuators` mirrors the model-family choice:

- `VehicleActuatorKind::FixedWing(FixedWingActuators)`
- `VehicleActuatorKind::Rotorcraft(RotorcraftActuators)`
- `VehicleActuatorKind::Hybrid(HybridActuators)`
- `VehicleActuatorKind::Spacecraft(SpacecraftActuators)`

## Runtime Outputs

These are not authoring inputs, but consumers commonly read them directly:

| Type | Purpose |
| --- | --- |
| `FlightControlChannels` | Resolved mapped channels after shaping and slew |
| `FlightAeroState` | Atmosphere sample, altitude, air-relative velocities, qbar, AoA, and sideslip |
| `FlightForces` | Force and torque breakdown for debugging or physics adapters |
| `FlightTelemetry` | Instrument-ready human-readable outputs |
| `LandingGearState` | Runtime gear position, target, and contact status |
| `StallState` | Fixed-wing or hybrid stall flag plus hysteresis amount |

## Presets

Named presets such as `arcade_racer`, `cargo`, and `tiltrotor_transport` no longer live in the library API. They now live in `examples/presets` so the crate surface stays generic and low-level.
