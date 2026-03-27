# MJCF Energy Flag Cleanup Spec

**Status:** Draft — ready for next session
**Date:** 2026-03-27

## Problem

14 examples manually set `model.enableflags |= ENABLE_ENERGY` after
`load_model()` even though the MJCF `<flag energy="enable"/>` already
handles this automatically during model building. The manual flag is
unnecessary — but these 14 examples don't have the `<flag>` element in
their MJCF at all. They rely entirely on the manual post-load flag.

This is fragile: if someone copies the MJCF from an example but forgets
the manual `enableflags` line, `total_energy()` silently returns 0.

## Fix

For each of the 14 examples:
1. Add `<flag energy="enable"/>` to the MJCF `<option>` block
2. Remove `model.enableflags |= ENABLE_ENERGY` (now redundant)
3. Remove `use sim_core::ENABLE_ENERGY` import (now unused)
4. Change `let mut model` to `let model` (no longer mutated)
5. Verify energy tracking still works

## Affected Examples (14)

| Example | Path |
|---------|------|
| simple-pendulum | `hinge-joint/simple-pendulum` |
| double-pendulum | `hinge-joint/double-pendulum` |
| horizontal slider | `slide-joint/horizontal` |
| vertical slider | `slide-joint/vertical` |
| spherical-pendulum | `ball-joint/spherical-pendulum` |
| conical-pendulum | `ball-joint/conical-pendulum` |
| cone-limit | `ball-joint/cone-limit` |
| cone-limit-orbit | `ball-joint/cone-limit-orbit` |
| motor actuator | `actuators/motor` |
| clock sensor | `sensors/clock` |
| joint-pos-vel | `sensors/joint-pos-vel` |
| frame-pos-quat | `sensors/frame-pos-quat` |
| gyro-velocimeter | `sensors/gyro-velocimeter` |
| geom-distance | `sensors/geom-distance` |

## Validation

After each example is updated, run it and verify:
- Energy values in the HUD/console are non-zero
- Validation checks still pass (energy conservation, monotonicity, etc.)
- No clippy warnings

## Risk

Low — the MJCF flag path is already tested (integrator examples use it
successfully) and the model builder has explicit test coverage for
`<flag energy="enable"/>` → `ENABLE_ENERGY`.

## Scope

Mechanical cleanup only — no behavior changes. Each example produces
identical output before and after.
