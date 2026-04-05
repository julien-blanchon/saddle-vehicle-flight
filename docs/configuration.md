# Configuration

Defaults below refer to each type's `Default` implementation unless the section says otherwise.

## Conventions

- Distances are meters.
- Speeds are meters per second.
- Angles stored in config are radians unless a field name says `deg`.
- Runtime telemetry converts AoA and sideslip to degrees for human-readable UI.
- `FlightControlInput` axes are normalized to `[-1, 1]` except throttle and collective, which are clamped to `[0, 1]`.

## `FlightBody`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `mass_kg` | `f32` | kg | `1000.0` | `> 0` | Converts resolved force into linear acceleration |
| `inertia_kgm2` | `Vec3` | kg m^2 | `(1200, 1600, 1900)` | each axis `> 0` | Converts resolved body torque into angular acceleration |
| `gravity_acceleration_mps2` | `f32` | m/s^2 | `9.80665` | `>= 0` | Gravity magnitude applied every frame |
| `use_internal_integration` | `bool` | n/a | `true` | `true` or `false` | When `true`, the crate updates `Transform` and `FlightKinematics` directly |

## `FlightEnvironment`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `wind_world_mps` | `Vec3` | m/s | `Vec3::ZERO` | any finite vector | Persistent airmass velocity in world space |
| `gust_world_mps` | `Vec3` | m/s | `Vec3::ZERO` | any finite vector | Additive transient wind contribution |
| `density_multiplier` | `f32` | ratio | `1.0` | practical `> 0`, clamped to `>= 0.01` internally | Multiplies sampled atmosphere density for weather or stylization |
| `surface_altitude_msl_m` | `Option<f32>` | m MSL | `None` | any finite altitude | Enables simple ground contact and AGL reporting when present |

## `FlightControlInput`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `pitch` | `f32` | normalized | `0.0` | `[-1, 1]` recommended | Pitch command before response shaping |
| `roll` | `f32` | normalized | `0.0` | `[-1, 1]` recommended | Roll command before response shaping |
| `yaw` | `f32` | normalized | `0.0` | `[-1, 1]` recommended | Yaw or rudder command before response shaping |
| `throttle` | `f32` | normalized | `0.0` | `[0, 1]` | Fixed-wing power command |
| `collective` | `f32` | normalized | `0.0` | `[0, 1]` | Helicopter lift command |
| `vtol_transition` | `f32` | normalized | `0.0` | `[0, 1]` | VTOL nacelle or transition command where `0 = hover` and `1 = wing-borne` |
| `pitch_trim` | `f32` | normalized | `0.0` | small values recommended | Adds bias through the aircraft trim authority |
| `roll_trim` | `f32` | normalized | `0.0` | small values recommended | Adds bias through the aircraft trim authority |
| `yaw_trim` | `f32` | normalized | `0.0` | small values recommended | Adds bias through the aircraft trim authority |
| `requested_gear_down` | `Option<bool>` | n/a | `None` | `Some(true/false)` or `None` | Explicit gear target override |
| `toggle_gear` | `bool` | n/a | `false` | edge-triggered | Toggles gear when retractable and not already latched |

## `FlightAssist`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `wings_leveling` | `f32` | gain | `0.0` | `>= 0` | Adds roll and pitch stabilization for fixed-wing craft |
| `coordinated_turn` | `f32` | gain | `0.0` | `>= 0` | Adds yaw correction from sideslip for both flight families |
| `hover_leveling` | `f32` | gain | `0.0` | `>= 0` | Adds helicopter attitude stabilization around hover |

## `FlightControlResponse`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `pitch_rate_per_second` | `f32` | 1/s | `5.0` | `> 0` | Max slew rate toward commanded pitch input |
| `roll_rate_per_second` | `f32` | 1/s | `6.0` | `> 0` | Max slew rate toward commanded roll input |
| `yaw_rate_per_second` | `f32` | 1/s | `4.0` | `> 0` | Max slew rate toward commanded yaw input |
| `pitch_exponent` | `f32` | exponent | `1.25` | `>= 1` typical | Shapes low-end pitch sensitivity |
| `roll_exponent` | `f32` | exponent | `1.15` | `>= 1` typical | Shapes low-end roll sensitivity |
| `yaw_exponent` | `f32` | exponent | `1.0` | `>= 1` typical | Shapes low-end yaw sensitivity |

## `PowerResponse`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `throttle_rise_per_second` | `f32` | 1/s | `0.65` | `> 0` | Engine spool-up speed toward higher throttle |
| `throttle_fall_per_second` | `f32` | 1/s | `1.0` | `> 0` | Power reduction speed toward lower throttle |
| `collective_rise_per_second` | `f32` | 1/s | `1.0` | `> 0` | Collective rise speed for helicopters |
| `collective_fall_per_second` | `f32` | 1/s | `1.4` | `> 0` | Collective drop speed for helicopters |

## `ContactGeometry`

| Field | Type | Unit | Default | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `contact_offset_below_origin_m` | `f32` | m | `1.0` | `>= 0` | Distance from entity origin to the lowest contact point |
| `retractable` | `bool` | n/a | `true` | `true` or `false` | When `true`, contact offset scales with gear extension |
| `extension_rate_per_second` | `f32` | 1/s | `1.0` | `> 0` | Gear extension or retraction speed |
| `surface_damping_per_second` | `f32` | 1/s | `1.8` | `>= 0` | Horizontal and angular damping while in ground contact |
| `ground_effect_height_m` | `f32` | m | `4.0` | `>= 0` | AGL window where ground effect is applied |
| `ground_effect_boost` | `f32` | ratio | `0.12` | `>= 0` | Additional lift multiplier at very low altitude |

## `FixedWingAircraft`

`Default` for `FixedWingAircraft` is `FixedWingAircraft::trainer()`.

| Field | Type | Unit | Default (`trainer`) | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `wing_area_m2` | `f32` | m^2 | `16.2` | `> 0` | Scales lift, drag, and side force |
| `wingspan_m` | `f32` | m | `10.9` | `> 0` | Scales roll and yaw moments |
| `mean_chord_m` | `f32` | m | `1.52` | `> 0` | Scales pitch moment |
| `max_thrust_newtons` | `f32` | N | `3200.0` | `>= 0` | Full-throttle propulsive force |
| `cl0` | `f32` | coefficient | `0.22` | finite | Base lift coefficient at zero AoA |
| `lift_curve_slope_per_rad` | `f32` | 1/rad | `5.25` | `> 0` typical | Lift growth with AoA before stall |
| `max_lift_coefficient` | `f32` | coefficient | `1.45` | `> 0` | Clamp for pre-stall lift |
| `post_stall_lift_coefficient` | `f32` | coefficient | `0.58` | `>= 0` | Residual lift once stalled |
| `zero_lift_drag_coefficient` | `f32` | coefficient | `0.028` | `>= 0` | Parasite drag floor |
| `induced_drag_factor` | `f32` | coefficient | `0.055` | `>= 0` | Additional drag from lift production |
| `stall_drag_coefficient` | `f32` | coefficient | `0.72` | `>= 0` | Extra drag when stalled |
| `side_force_slope_per_rad` | `f32` | 1/rad | `1.1` | `>= 0` | Sideslip-to-side-force coupling |
| `aileron_authority` | `f32` | gain | `0.55` | `>= 0` | Roll control contribution |
| `elevator_authority` | `f32` | gain | `0.72` | `>= 0` | Pitch control contribution |
| `rudder_authority` | `f32` | gain | `0.42` | `>= 0` | Yaw and sideslip-control contribution |
| `roll_rate_damping` | `f32` | damping | `0.65` | `>= 0` | Roll-rate damping |
| `pitch_rate_damping` | `f32` | damping | `0.88` | `>= 0` | Pitch-rate damping |
| `yaw_rate_damping` | `f32` | damping | `0.48` | `>= 0` | Yaw-rate damping |
| `roll_stability` | `f32` | gain | `0.18` | `>= 0` | Roll restoring moment from sideslip |
| `pitch_stability` | `f32` | gain | `0.95` | `>= 0` | Pitch restoring moment from AoA |
| `yaw_stability` | `f32` | gain | `0.52` | `>= 0` | Yaw restoring moment from sideslip |
| `gear_drag_coefficient` | `f32` | coefficient | `0.03` | `>= 0` | Extra drag with gear deployed |
| `trim_authority` | `Vec3` | normalized bias | `(0.10, 0.16, 0.12)` | small magnitudes recommended | Trim gain applied to yaw, pitch, and roll respectively |
| `stall_alpha_rad` | `f32` | rad | `0.2793` | `> recovery_alpha_rad` | AoA threshold that drives stall entry |
| `recovery_alpha_rad` | `f32` | rad | `0.2094` | `>= 0` | AoA threshold that clears stall target |
| `stall_response_per_second` | `f32` | 1/s | `2.8` | `> 0` | Hysteresis blending speed between attached and stalled flight |
| `control_response` | `FlightControlResponse` | n/a | default response | see above | Input slew and shaping |
| `power_response` | `PowerResponse` | n/a | default response | see above | Throttle lag |
| `landing_contact` | `ContactGeometry` | n/a | tuned trainer contact | see above | Ground-contact and ground-effect behavior |

## `HelicopterAircraft`

`Default` for `HelicopterAircraft` is `HelicopterAircraft::utility()`.

| Field | Type | Unit | Default (`utility`) | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `rotor_disc_area_m2` | `f32` | m^2 | `62.0` | `> 0` | Scales drag and translational-lift response |
| `max_main_lift_newtons` | `f32` | N | `17000.0` | `>= 0` | Full-collective main rotor lift |
| `parasite_drag_coefficient` | `f32` | coefficient | `0.26` | `>= 0` | Airspeed-aligned drag strength |
| `side_drag_coefficient` | `f32` | coefficient | `0.20` | `>= 0` | Lateral drag strength from side slip |
| `translational_lift_gain` | `f32` | ratio | `0.24` | `>= 0` | Extra lift multiplier gained with forward speed |
| `translational_lift_full_speed_mps` | `f32` | m/s | `22.0` | `> 0` | Speed where translational lift reaches full effect |
| `pitch_torque_authority` | `f32` | N m | `3200.0` | `>= 0` | Cyclic pitch torque authority |
| `roll_torque_authority` | `f32` | N m | `3000.0` | `>= 0` | Cyclic roll torque authority |
| `yaw_torque_authority` | `f32` | N m | `2600.0` | `>= 0` | Pedal or anti-torque yaw authority |
| `anti_torque_per_collective` | `f32` | N m | `1450.0` | `>= 0` | Collective-linked yaw torque that the pilot must counter |
| `angular_damping` | `Vec3` | gain | `(0.45, 0.65, 0.40)` | each `>= 0` | Body-axis angular damping |
| `trim_authority` | `Vec3` | normalized bias | `(0.10, 0.08, 0.12)` | small magnitudes recommended | Trim gain applied to yaw, pitch, and roll respectively |
| `control_response` | `FlightControlResponse` | n/a | custom utility response | see above | Cyclic and pedal slew tuning |
| `power_response` | `PowerResponse` | n/a | utility collective response | see above | Collective lag tuning |
| `contact_geometry` | `ContactGeometry` | n/a | utility skid contact | see above | Ground-contact and ground-effect behavior |

## `VtolAircraft`

`Default` for `VtolAircraft` is `VtolAircraft::tiltrotor_transport()`.

| Field | Type | Unit | Default (`tiltrotor_transport`) | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `fixed_wing` | `FixedWingAircraft` | n/a | tuned tiltrotor wing model | see fixed-wing section | Wing-borne lift, drag, stall, and airplane-style torque path used during transition |
| `rotorcraft` | `HelicopterAircraft` | n/a | tuned tiltrotor rotor model | see helicopter section | Hover lift, low-speed drag, and hover-style control path |
| `contact_geometry` | `ContactGeometry` | n/a | retractable transport gear | see above | Shared runway contact and ground-effect behavior for the VTOL airframe |
| `transition_rate_per_second` | `f32` | 1/s | `0.35` | `> 0` | Slew rate from the commanded `vtol_transition` input to the resolved transition state | Too low makes mode swaps feel unresponsive |
| `wingborne_blend_start` | `f32` | normalized | `0.22` | `0..=1` | Transition point where fixed-wing lift and torque blending begin | Too low makes wings dominate before the aircraft has speed |
| `wingborne_blend_end` | `f32` | normalized | `0.72` | `>= start`, `<= 1` | Transition point where the aircraft is treated as fully wing-borne | Too high leaves hover behavior active for too long |

## `SpacecraftConfig`

`Default` for `SpacecraftConfig` is `SpacecraftConfig::fighter()`.

| Field | Type | Unit | Default (`fighter`) | Valid range | Effect |
| --- | --- | --- | --- | --- | --- |
| `max_thrust_newtons` | `f32` | N | `24000.0` | `>= 0` | Main engine thrust along the forward axis |
| `rcs_thrust_newtons` | `f32` | N | `4000.0` | `>= 0` | Reaction control system thrust for translation (yaw + collective inputs) |
| `pitch_torque_authority` | `f32` | N m | `8000.0` | `>= 0` | Pitch RCS torque authority |
| `roll_torque_authority` | `f32` | N m | `6000.0` | `>= 0` | Roll RCS torque authority |
| `yaw_torque_authority` | `f32` | N m | `5000.0` | `>= 0` | Yaw RCS torque authority |
| `angular_damping` | `Vec3` | gain | `(0.25, 0.35, 0.20)` | each `>= 0` | Body-axis angular rate damping |
| `linear_drag_coefficient` | `f32` | 1/s | `0.0` | `>= 0` | Optional linear drag for arcade game-feel (0.0 = pure Newtonian) |
| `control_response` | `FlightControlResponse` | n/a | default response | see above | Input slew and shaping |
| `power_response` | `PowerResponse` | n/a | default response | see above | Throttle lag |

### Spacecraft Presets

- `fighter()` — agile fighter with 2000 kg mass and high thrust-to-weight
- `cargo()` — heavy cargo ship (8000 kg) with low angular authority and high linear drag
- `arcade_fighter()` — very high authority, fast response, and zero drag for arcade space combat

## Runtime Outputs

These are not authoring inputs, but consumers commonly read them directly:

| Type | Purpose |
| --- | --- |
| `FlightAeroState` | Atmosphere sample, altitude, air-relative velocities, qbar, AoA, and sideslip |
| `FlightForces` | Force and torque breakdown for debugging, BRP, or backend adapters, including assist torques |
| `FlightTelemetry` | Instrument-ready human-readable values, including world-frame vertical speed and VTOL transition |
| `LandingGearState` | Runtime gear position, target, and contact status |
| `StallState` | Fixed-wing stall flag plus hysteresis amount |

## Required Runtime Components

Spawning `FixedWingAircraft`, `HelicopterAircraft`, `VtolAircraft`, or `SpacecraftConfig` automatically requires:

- `Transform`
- `GlobalTransform`
- `FlightBody`
- `FlightAssist`
- `FlightAeroState`
- `FlightControlInput`
- `FlightEnvironment`
- `FlightForces`
- `FlightKinematics`
- `LandingGearState`
- `FlightTelemetry`
- `StallState`

That keeps the common spawn path small while still allowing callers to override any runtime component explicitly.
