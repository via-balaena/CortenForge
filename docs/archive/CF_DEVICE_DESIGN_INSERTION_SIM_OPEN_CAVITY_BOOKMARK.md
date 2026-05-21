# cf-device-design Insertion Sim Open-Cavity Rework — Bookmark

**Status**: BOOKMARK 2026-05-16 (cavity-mouth arc shipped on dev
`a403a4f3`; this followup is silently invalidated by it).
**Parent**: [`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md)
§1 Q5.
**Blocks**: nothing (fit-viz rungs 2-6 do NOT depend on this).
**Blocked by**: nothing.

## What changed

The cavity-mouth arc (sub-leaves 1-5, commits `03bb695f` →
`a403a4f3`) opened the cf-device-design preview's cavity at the cap
plane. The Insertion Sim (`mod insertion_sim` + slice-7 panel) does
NOT consume the preview's `CachedScanSdf`; it has its own SDF
pipeline (`build_grid_sdf` → `Solid::offset/subtract` in
`tools/cf-device-design/src/insertion_sim.rs`), so the sim build is
not broken. But the sim's calibration is now stale.

## The two specific problems

### 1. `INSERTION_CONTACT_KAPPA = 1.0e3` is tuned for closed cavity

`insertion_sim.rs:857-865` docstring: "the *whole* cavity wall
engages at once, unlike the rows' localized probe". Explicit pin of
the closed-cavity assumption. With the cavity now open at the cap
plane:

- Contact becomes PROGRESSIVE (intruder enters mouth, contacts dome
  last), not full-surface.
- Default `1e4` (the row-precedent value) might suffice; needs a
  re-tune sweep on iter-1 to confirm.

### 2. Rigid intruder trajectory is closed-cavity-shaped

The sim builds the intruder as a copy of the scan SDF offset by
`interference_m + cavity_offset_m`. With a closed cavity it presses
into a sealed dome from `interference_m = 0`. With an open cavity it
needs to anchor outside the mouth and travel through it before
engaging the dome.

## What to do (when re-prioritized)

This is a separate arc, not blocking fit-viz. When the user is ready:

1. Refactor `build_insertion_geometry` to use the open-cavity body.
   `Solid::subtract` doesn't directly support half-space clip; would
   need a custom `body = outer.subtract(cavity).intersect(below_cap_plane)`
   or equivalent.
2. Re-anchor the rigid intruder trajectory to start outside the
   cavity mouth.
3. Run the iter-1 fixture; observe contact-pressure ramp; re-tune
   `INSERTION_CONTACT_KAPPA` (likely settle around the row-precedent
   `1e4`).
4. Add a regression test that pins the new ramp behaviour.

## Why this is NOT blocking fit-viz

Fit-viz rungs 2-6 (per-step playback, scan-as-intruder, pressure
scoring, retraction + over-cycle, optional auto-search) operate on
the cf-device-design PREVIEW's open-cavity surfaces — exactly the
shells the cavity-mouth arc just delivered. The slice-7 Insertion
Sim panel is a SEPARATE consumer (FEM solve, not visual fit) and its
readouts being stale doesn't block the fit-viz program.

## Status note

User may see the Insertion Sim panel's pressure readouts looking
"off" against the now-open preview. Pre-existing condition (the sim
was always physically meaningless against v1.0 device geometry even
before this arc); the cavity-mouth feature just makes the
discrepancy visually obvious.
