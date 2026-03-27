# Examples API Improvements Spec

**Status:** Final — stress-tested twice, trimmed to essentials
**Date:** 2026-03-26
**Scope:** sim-core (L0) + sim-bevy (L1) — targeted API fixes driven by patterns
observed across all 10 actuator examples.

## Motivation

The 10 actuator examples share defensive workarounds and fragile patterns that
should be fixed in the engine, not papered over in example code. This spec
proposes 4 targeted changes — each fixes a real problem (bugs, footguns, or
confusing output), not aesthetics. Two originally-proposed changes
(ReportBuilder, TimeSampler) were killed after stress testing as unnecessary
abstraction.

---

## Change 1: Targeted Safe Accessors on Data (HIGH)

### Problem

Two specific access patterns are genuinely painful:

**1a. `set_ctrl` — 9/10 examples write this 3-line guard:**
```rust
if !data.ctrl.is_empty() {
    data.ctrl[0] = 5.0;
}
```

**1b. `actuator_moment` — 2-3 examples write this 4-line nested check:**
```rust
if data.actuator_moment.len() > IDX && !data.actuator_moment[IDX].is_empty() {
    data.actuator_moment[IDX][0]
} else { 0.0 }
```

Other fields (`act`, `act_dot`, `actuator_length`, `actuator_force`) already
have `DVector::get(i) -> Option<&f64>` from nalgebra. Writing
`.get(0).copied().unwrap_or(0.0)` is one line and idiomatic Rust — no new
API needed for those.

### Proposed API

Add exactly two methods to `Data`:

```rust
impl Data {
    /// Set ctrl[i], no-op if out of range.
    pub fn set_ctrl(&mut self, i: usize, value: f64);

    /// Read actuator_moment[actuator_idx][dof_idx], returning 0.0 if either
    /// index is out of range. The nested Vec<DVector> makes manual bounds
    /// checking particularly noisy.
    pub fn actuator_moment_val(&self, actuator_idx: usize, dof_idx: usize) -> f64;
}
```

### Before / After

```rust
// BEFORE (motor, 3 lines):
if !data.ctrl.is_empty() {
    data.ctrl[0] = 5.0;
}
// AFTER (1 line):
data.set_ctrl(0, 5.0);

// BEFORE (slider-crank, 4 lines):
let moment = if data.actuator_moment.len() > LINKAGE_IDX
    && !data.actuator_moment[LINKAGE_IDX].is_empty()
{ data.actuator_moment[LINKAGE_IDX][0] } else { 0.0 };
// AFTER (1 line):
let moment = data.actuator_moment_val(LINKAGE_IDX, 0);
```

### Why not more accessors?

`act_val`, `actuator_length_val`, `actuator_force_val`, etc. were proposed
initially and cut. The underlying fields are `DVector<f64>` or `Vec<f64>`,
which already have `.get(i)`. Writing `.get(0).copied().unwrap_or(0.0)` is
idiomatic Rust that every contributor knows. Adding 5+ methods that rename
this pattern creates API bloat without solving a real problem. The two
methods we keep solve genuinely worse patterns: `set_ctrl` (write-side guard)
and `actuator_moment_val` (nested indexing).

### Scope

- File: `sim/L0/core/src/types/data.rs`
- Two trivial methods (~4 lines each)
- No behavioral change — pure convenience

---

## Change 2: Sensor-by-Name Accessor (HIGH)

### Problem

Every example uses hard-coded sensor indices:

```rust
let force = data.sensor_data(&model, 0)[0];  // what if someone adds a sensor above?
let angle = data.sensor_data(&model, 1)[0];
let pos   = data.sensor_data(&model, 2)[0];
```

These indices appear 2-3 times per example (display closure, HUD, diagnostics).
Adding, removing, or reordering sensors in the MJCF silently breaks all of
them. This is a latent bug factory.

The model already has `sensor_name_to_id: HashMap<String, usize>` and
`model.sensor_id(name) -> Option<usize>`, but there's no one-call accessor
for the common case (read a 1D sensor by name).

### Proposed API

```rust
impl Data {
    /// Read the first element of a named sensor, returning None if the
    /// sensor name is not found.
    ///
    /// Most sensors (jointpos, actuatorfrc, etc.) are 1D — this covers
    /// the common case. For multi-dimensional sensors, use
    /// `model.sensor_id(name)` + `data.sensor_data(model, id)`.
    ///
    /// **Performance:** O(hash lookup). Use index-based `sensor_data()`
    /// in tight loops.
    pub fn sensor_scalar(&self, model: &Model, name: &str) -> Option<f64>;
}
```

Returns `Option<f64>`, not a silent `0.0`. Callers use `.unwrap_or(0.0)` if
they want that — explicit is better than silent defaults that hide bugs.

### Before / After

```rust
// BEFORE (fragile — reordering sensors breaks this):
let brake = data.sensor_data(&model, 0)[0];
let vel   = data.sensor_data(&model, 1)[0];

// AFTER (robust — sensor order doesn't matter):
let brake = data.sensor_scalar(&model, "brake_force").unwrap_or(0.0);
let vel   = data.sensor_scalar(&model, "velocity").unwrap_or(0.0);
```

### Note on closures

The `ValidationHarness.display()` closure receives raw `(&Model, &Data)`.
`sensor_scalar` is on `Data` taking `&Model`, so it works directly — no
PhysicsData wrapper needed.

### Scope

- File: `sim/L0/core/src/types/data.rs`
- One method (~5 lines), uses existing `model.sensor_name_to_id`
- Pure additive — index-based `sensor_data()` stays for hot paths

---

## Change 3: First-Frame Forward Pass (HIGH)

### Problem

9 of 10 examples skip the first physics frame with:

```rust
if time < 1e-6 { return; }
```

This is a workaround: `actuator_force`, `actuator_moment`, `site_xpos`, etc.
aren't computed until the first `step()` + `forward()` call. On frame 0,
`step_physics_realtime` takes zero steps (Bevy delta time is 0.0), and the
`if steps > 0` guard skips `forward()`. PostUpdate systems see stale zeros.

Every new example must independently discover this footgun.

### Proposed Fix

In `step_physics_realtime`, add a first-frame forward pass:

```rust
// After the stepping loop:
if steps > 0 {
    data.0.forward(&model.0);
} else if data.0.time == 0.0 {
    // First frame: no steps taken (delta_t = 0), but ensure derived
    // quantities (actuator_moment, site_xpos, etc.) are populated.
    data.0.forward(&model.0);
}
```

**Check `data.0.time` (physics time), not Bevy's `Time` resource.**

**Idempotency:** `forward()` is a pure function of `qpos`/`qvel`. If
`setup()` already called `forward()`, calling it again produces identical
results. The duplicate work is one extra forward pass, ever.

**Determinism:** No effect. `forward()` doesn't modify `qpos`/`qvel`.

### Scope

- File: `sim/L1/bevy/src/model_data.rs` (`step_physics_realtime`)
- One conditional addition (~3 lines)
- All 9 examples can remove their `if time < 1e-6` guards

---

## Change 4: Empty Harness Report Suppression (LOW)

### Problem

Examples with custom validation (not harness trackers) get a confusing
empty validation report header:

```
=== Validation Report (t=17s) ===
=================================
```

This prints because `validation_system` calls `print_report` even when
`entries` is empty and `checks` is empty.

### Proposed Fix

```rust
// In validation_system, phase 3 — add one guard:
if !checks.is_empty() {
    let title = format!("Validation Report (t={report_time:.0}s)");
    let _ = print_report(&title, &checks);
}
```

### Scope

- File: `sim/L1/bevy/src/examples.rs` (`validation_system`)
- One-line change, zero risk

---

## Killed Changes (with reasoning)

### ~~ReportBuilder + ReportOnce~~ — KILLED

Saves 1 line per example (7 lines → 6 lines for the check-building block).
Introduces 2 new types (`ReportBuilder`, `ReportOnce`) that future
contributors must learn. The existing `vec![Check{...}] + print_report`
pattern is perfectly readable and self-documenting. `ReportOnce` is an
abstraction over a boolean flag — the definition of unnecessary abstraction.

### ~~TimeSampler + WindowSampler~~ — KILLED

40 lines of library code to save ~16 lines of clear inline code across 4-6
examples. The inline patterns (closest-to-target, first-in-window) are 5
lines of straightforward comparison logic that are easier to debug than
opaque `sampler.offer(time, val)` calls. The abstraction trades readability
for brevity — the wrong tradeoff for teaching code.

### ~~Example App Builder~~ — KILLED (earlier)

Bevy's type system makes storing generic systems in a builder impractical.
The boilerplate it removes (DefaultPlugins, system chains) is pedagogically
valuable — it teaches Bevy concepts that examples should demonstrate.

---

## Implementation Order

1. **Change 4** (empty report) — one-liner, do first
2. **Change 3** (first-frame forward) — one-point fix, high value
3. **Change 1** (set_ctrl + actuator_moment_val) — two methods, high value
4. **Change 2** (sensor_scalar) — one method, high value

All 4 can be implemented and validated in a single session.

---

## Validation Strategy

After each change:
1. All 10 actuator examples still compile
2. All 10 pass their validation checks
3. `cargo test -p sim-core -p sim-bevy` passes
4. Migrate 1-2 examples to use the new API, verify they're shorter/clearer
5. Batch-update the remaining examples
