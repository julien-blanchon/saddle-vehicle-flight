# Flight Lab

Crate-local standalone lab app for validating the shared `saddle-vehicle-flight` crate in a real Bevy application.

## Purpose

- verify fixed-wing and helicopter runtime behavior in one scene
- expose telemetry, force vectors, and active-aircraft switching for BRP and E2E inspection
- provide deterministic screenshot gates for takeoff, stall recovery, and hover or translation

## Status

Working

## Run

```bash
cargo run -p saddle-vehicle-flight-lab
```

## E2E

```bash
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_fixed_wing_smoke
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_stall_recovery
cargo run -p saddle-vehicle-flight-lab --features e2e -- flight_helicopter_hover
```

## BRP

```bash
uv run --project .codex/skills/bevy-brp/script brp app launch saddle-vehicle-flight-lab
uv run --project .codex/skills/bevy-brp/script brp world query bevy_ecs::name::Name
uv run --project .codex/skills/bevy-brp/script brp extras screenshot /tmp/flight_lab.png
uv run --project .codex/skills/bevy-brp/script brp extras shutdown
```

## Notes

- The lab keeps both a fixed-wing aircraft and a helicopter alive at once so BRP can inspect stable named entities.
- Pilot input and the `1` or `2` active-aircraft switch both run through `bevy_enhanced_input`.
- The overlay is intentionally dense so screenshot artifacts and BRP state queries can be cross-checked against the same runtime values.
