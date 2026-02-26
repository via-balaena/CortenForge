# Phase 4 — Lazy Evaluation Gates & Persistent Subtree Fields

**Status:** Draft
**Scope:** §56 (`subtree_linvel`/`subtree_angmom` persistent fields) + `flg_rnepost`/`flg_subtreevel` lazy evaluation gates.
**MuJoCo ref:** `engine_core_smooth.c` (`mj_subtreeVel`, `mj_rnePostConstraint`), `engine_forward.c` (flag clearing), `engine_sensor.c` (flag triggering).

---

## Problem Statement

Two conformance gaps exist in the forward pipeline:

1. **`mj_body_accumulators()` is always called.** MuJoCo gates
   `mj_rnePostConstraint()` behind `flg_rnepost` — a lazy flag that is only
   triggered when acceleration-stage sensors actually need `cacc`/`cfrc_int`/`cfrc_ext`.
   Models without accelerometer, force, torque, framelinacc, or frameangacc
   sensors skip the entire RNE post-constraint pass. We unconditionally run it
   every step.

2. **`subtree_linvel`/`subtree_angmom` are not persistent fields.** MuJoCo
   stores these as persistent `Data` arrays and computes them via `mj_subtreeVel()`,
   gated behind `flg_subtreevel`. Our sensors recompute them on-demand per sensor
   via O(n²) parent-chain walks (in `sensor/derived.rs`). This is both
   non-conformant (no persistent fields) and asymptotically worse (O(n²) vs O(n)).

---

## MuJoCo Reference

### Lazy Flag Lifecycle

MuJoCo defines four lazy flags in `mjData`:

```c
mjtByte flg_energypos;   // has mj_energyPos been called
mjtByte flg_energyvel;   // has mj_energyVel been called
mjtByte flg_subtreevel;  // has mj_subtreeVel been called
mjtByte flg_rnepost;     // has mj_rnePostConstraint been called
```

Each follows the same pattern:

1. **Clear** → set to 0 at the start of the pipeline stage that invalidates it.
2. **Guard** → before computing, check `if (!flag) { compute(); }`.
3. **Set** → computation function sets flag to 1 at the end.
4. **Effect** → subsequent consumers see flag=1 and skip redundant computation.

### `flg_rnepost` — Post-Constraint RNE

**Cleared** right after `mj_fwdConstraint()`, before `mj_sensorAcc()`:
- In `mj_forwardSkip()`: `d->flg_rnepost = 0;`
- In `mj_step2()`: `d->flg_rnepost = 0;`
- In `mj_inverseSkip()`: `d->flg_rnepost = 0;`

**Triggered** inside `mj_sensorAcc()` when any of these sensor types is
encountered:
- `accelerometer`
- `force`
- `torque`
- `framelinacc`
- `frameangacc`
- `user` sensor at ACC stage (conservative — opaque callback)

**Guard code** (from `engine_sensor.c`):
```c
if (!d->flg_rnepost &&
    (type == mjSENS_ACCELEROMETER ||
     type == mjSENS_FORCE         ||
     type == mjSENS_TORQUE        ||
     type == mjSENS_FRAMELINACC   ||
     type == mjSENS_FRAMEANGACC)) {
  mj_rnePostConstraint(m, d);
}
```

**Set** at the end of `mj_rnePostConstraint()`: `d->flg_rnepost = 1;`

**Key detail:** Touch, contact, and tactile sensors do NOT trigger
`flg_rnepost`. They read `efc_force` directly via `mj_contactForce()`.

### `flg_subtreevel` — Subtree Velocity/Momentum

**Cleared** at the start of `mj_fwdVelocity()`:
```c
d->flg_subtreevel = 0;
d->flg_energyvel = 0;
```

**Triggered** inside `mj_sensorVel()` when:
- `subtreelinvel`
- `subtreeangmom`
- `user` sensor at VEL stage (conservative)

**Set** at the end of `mj_subtreeVel()`: `d->flg_subtreevel = 1;`

### `mj_subtreeVel()` Algorithm

MuJoCo's `mj_subtreeVel` (from `engine_core_smooth.c`) is an O(n) backward
accumulation — NOT the O(n²) per-body parent-chain walk our sensors use:

1. **Initialize** each body's contribution:
   - `subtree_linvel[b] = mass[b] * linvel[b]` (linear momentum)
   - `subtree_angmom[b] = I[b] * omega[b]` (spin angular momentum)

2. **Backward pass** (leaf → root): for each body b with parent p:
   - `subtree_linvel[p] += subtree_linvel[b]` (accumulate momentum)
   - `subtree_angmom[p] += subtree_angmom[b] + (xipos[b] - subtree_com[p]) × subtree_linvel[b]`
     (transfer angular momentum about parent's subtree COM)

3. **Finalize** each body: divide momentum by subtree mass to get velocity:
   - `subtree_linvel[b] /= subtree_mass[b]`
   - Adjust `subtree_angmom[b]` to be about the subtree COM:
     `subtree_angmom[b] -= subtree_mass[b] * (subtree_com[b] - xipos[b]) × subtree_linvel[b]`

This requires `subtree_com` and `subtree_mass` from the position stage (already
computed in our `forward/position.rs`).

---

## Design

### Phase 4A: `flg_rnepost` — Lazy Body Accumulators

**Goal:** Stop calling `mj_body_accumulators()` unconditionally. Gate it behind
a demand flag triggered by sensors.

#### 4A.1 Data Field

Add to `Data`:

```rust
/// Lazy evaluation flag: true if `mj_body_accumulators()` has been called
/// this step. Cleared before `mj_sensor_acc()`, set by
/// `mj_body_accumulators()`. Mirrors MuJoCo's `flg_rnepost`.
pub flg_rnepost: bool,
```

#### 4A.2 Flag Clearing

Clear `flg_rnepost = false` in `forward_acc()`, after constraint solve,
before sensor evaluation. This is the point where `cacc`/`cfrc_int`/`cfrc_ext`
become stale (qacc has changed).

```rust
// In forward_acc(), after mj_fwd_acceleration:
data.flg_rnepost = false;
```

Also clear in `Data::reset()` and in `inverse()`.

**`forward_skip_sensors()` interaction:** RK4 intermediate stages call
`forward_skip_sensors()` which passes `compute_sensors = false`. In this
path, `mj_sensor_acc()` is skipped entirely. `flg_rnepost` is cleared (at
the top of `forward_acc()`) but never triggered — no sensor demands
`mj_body_accumulators()`. The body accumulator fields (`cacc`, `cfrc_int`,
`cfrc_ext`) remain zeroed for intermediate stages. This is correct — no
sensor reads stale data, and `compare_fwd_inv()` is also skipped.

#### 4A.3 Remove Unconditional Call

Remove the unconditional `mj_body_accumulators(model, self)` call from
`forward_acc()` (currently at line 416).

#### 4A.4 Demand Triggering in `mj_sensor_acc()`

In `mj_sensor_acc()`, before evaluating a sensor that needs body accumulators,
check the flag and compute on demand:

```rust
// Sensor types that require body accumulators
MjSensorType::Accelerometer
| MjSensorType::Force
| MjSensorType::Torque
| MjSensorType::FrameLinAcc
| MjSensorType::FrameAngAcc => {
    if !data.flg_rnepost {
        acceleration::mj_body_accumulators(model, data);
        // flag is set inside mj_body_accumulators
    }
    // ... existing sensor logic ...
}

// User sensors at ACC stage (conservative)
MjSensorType::User => {
    if !data.flg_rnepost {
        acceleration::mj_body_accumulators(model, data);
    }
    // ... callback ...
}
```

#### 4A.5 Flag Setting

Set `flg_rnepost = true` at the end of `mj_body_accumulators()`:

```rust
pub fn mj_body_accumulators(model: &Model, data: &mut Data) {
    // ... existing 3-phase algorithm ...
    data.flg_rnepost = true;
}
```

#### 4A.6 Sensor Refactor: Use `cacc` Instead of Recomputing

Currently, `mj_sensor_acc()` calls `compute_body_acceleration()` and
`compute_body_angular_acceleration()` from `sensor/derived.rs`. These
recompute accelerations from `qacc` via joint traversal — duplicating
work that `mj_body_accumulators()` already stores in `cacc`.

After the lazy gate ensures `mj_body_accumulators()` has run, these
sensors should read directly from `cacc`:

- **Accelerometer**: `a_world = cacc[body_id].linear()` (indices 3-5)
- **FrameLinAcc**: `a_world = cacc[body_id].linear()`
- **FrameAngAcc**: `alpha = cacc[body_id].angular()` (indices 0-2)
- **Force/Torque**: `compute_site_force_torque()` should read from
  `cfrc_int` instead of recomputing via subtree inverse dynamics.

This is both a conformance fix (MuJoCo reads from these fields) and a
performance win (no redundant computation).

**Note:** `compute_site_force_torque()` currently re-derives the force/torque
from first principles (subtree sum of m*a - f_gravity). MuJoCo reads
`cfrc_int` at the parent joint and transforms to the site frame. The refactor
to use `cfrc_int` is a conformance improvement but algorithmically non-trivial
— it requires extracting the joint-space force from `cfrc_int` and
transforming to the sensor site frame. This should be specced as a separate
sub-step (4A.6b) with its own acceptance criteria.

#### 4A.7 Inverse Dynamics Consumer

`inverse()` currently calls `mj_body_accumulators()` directly. This is
correct — inverse dynamics always needs the accumulators. Keep this direct
call. After the call, `flg_rnepost = true`, so any subsequent sensor
evaluation in the same step will skip the redundant computation.

#### 4A.8 `ENABLE_FWDINV` Consumer

`compare_fwd_inv()` calls `inverse()` which calls `mj_body_accumulators()`.
No change needed — the flag lifecycle handles it.

**Pipeline ordering detail:** In `forward_acc()`, the order is:
1. Constraint solve
2. `flg_rnepost = false` (clear)
3. `mj_sensor_acc()` — may trigger `mj_body_accumulators()`, setting
   `flg_rnepost = true`
4. `compare_fwd_inv()` — calls `inverse()` which calls
   `mj_body_accumulators()` again

If sensors triggered `mj_body_accumulators()` in step 3, the inverse call in
step 4 re-runs it. This is intentional — `inverse()` needs the accumulators
computed in the context of its own `qacc`, which may differ from the forward
context. The redundant computation is safe (idempotent for the same `qacc`)
and matches MuJoCo's behavior.

---

### Phase 4B: `flg_subtreevel` — Persistent Subtree Fields

**Goal:** Add `subtree_linvel` and `subtree_angmom` as persistent Data fields,
computed via O(n) backward accumulation, gated behind a lazy flag.

#### 4B.1 Data Fields

Add to `Data`:

```rust
/// Per-body subtree linear velocity (length `nbody`).
/// `subtree_linvel[b]` = total linear momentum of b's subtree / subtree mass.
/// Computed by `mj_subtree_vel()` on demand.
pub subtree_linvel: Vec<Vector3<f64>>,

/// Per-body subtree angular momentum about subtree COM (length `nbody`).
/// Computed by `mj_subtree_vel()` on demand.
pub subtree_angmom: Vec<Vector3<f64>>,

/// Lazy evaluation flag: true if `mj_subtree_vel()` has been called
/// this step. Cleared at the start of velocity stage, set by
/// `mj_subtree_vel()`. Mirrors MuJoCo's `flg_subtreevel`.
pub flg_subtreevel: bool,
```

#### 4B.2 Allocation

In `Model::init_data()`: allocate both as `vec![Vector3::zeros(); nbody]`.

#### 4B.3 Reset

In `Data::reset()`: zero both fields and set `flg_subtreevel = false`.

#### 4B.4 Flag Clearing

Clear `flg_subtreevel = false` at the start of `mj_fwd_velocity()` (in
`forward/velocity.rs`). This is where velocity kinematics invalidates the
previous step's subtree velocities.

#### 4B.5 `mj_subtree_vel()` — O(n) Backward Accumulation

The complete algorithm for `mj_subtree_vel()` is specified in
**[S56_SUBTREE_VEL_SPEC.md](./S56_SUBTREE_VEL_SPEC.md)** — the authoritative
reference for this function. It is a three-phase O(n) backward accumulation:

1. **Phase 1:** Initialize per-body linear momentum and spin angular momentum,
   shifting `cvel` from `xpos[b]` to `xipos[b]` (body COM).
2. **Phase 2:** Backward accumulate linear momentum (leaf→root), then divide
   by subtree mass to get velocity.
3. **Phase 3:** Backward accumulate angular momentum with reference-point
   corrections (parallel axis theorem for momentum transfer).

The function lives in `forward/velocity.rs` and sets `flg_subtreevel = true`
at the end. See the S56 spec for the full Rust implementation, convention
notes (cvel reference point, subtree_mass Data vs Model), and all acceptance
criteria.

#### 4B.6 Demand Triggering in `mj_sensor_vel()`

In `mj_sensor_vel()`, before evaluating subtree sensors, check the flag:

```rust
MjSensorType::SubtreeLinVel | MjSensorType::SubtreeAngMom => {
    if !data.flg_subtreevel {
        mj_subtree_vel(model, data);
    }
    // Read from persistent fields
    sensor_write3(&mut data.sensordata, adr, &data.subtree_linvel[objid]);
    // (or subtree_angmom[objid])
}

// User sensors at VEL stage (conservative)
MjSensorType::User => {
    if !data.flg_subtreevel {
        mj_subtree_vel(model, data);
    }
    // ... callback ...
}
```

#### 4B.7 Remove Redundant Helpers

After sensors read from persistent fields, ALL per-sensor O(n²) helpers
become dead code:
- `compute_subtree_com()` — dead. `SubtreeCom` sensor should read
  `data.subtree_com[objid]` directly (already computed in position stage).
- `compute_subtree_momentum()` — dead (subtree_linvel replaces it)
- `compute_subtree_angmom()` — dead (subtree_angmom replaces it)

Delete all three from `sensor/derived.rs`. See
[S56_SUBTREE_VEL_SPEC.md](./S56_SUBTREE_VEL_SPEC.md) Step 8 for details.

---

## Implementation Order

0. **Step 0: cvel reference point fixes**
   ([CVEL_REFERENCE_POINT_FIXES.md](./CVEL_REFERENCE_POINT_FIXES.md)) —
   prerequisite. Fixes 5 latent bugs where sensor/energy code reads
   `cvel[b][3:6]` without shifting from `xpos[b]` to the physically correct
   reference point. Must land first to establish the corrected sensor baseline
   that §56 validates against.

1. **4B: subtree fields** (§56,
   [S56_SUBTREE_VEL_SPEC.md](./S56_SUBTREE_VEL_SPEC.md)) — adds new Data
   fields + O(n) algorithm + lazy gate. Self-contained; no existing behavior
   changes except sensor read path.

2. **4A: rnepost gate** — modifies existing unconditional
   `mj_body_accumulators()` call to be demand-driven. Requires more careful
   testing since it changes the pipeline flow for models without the
   triggering sensors.

3. **4A.6: sensor refactor** to read `cacc`/`cfrc_int` — optional
   conformance improvement that depends on 4A being solid.

---

## Acceptance Criteria

### AC1: `subtree_linvel` persistent field
`data.subtree_linvel[0]` equals whole-model COM velocity (total momentum /
total mass). Verified for a multi-body model with known velocities.

### AC2: `subtree_angmom` persistent field
`data.subtree_angmom[0]` equals total angular momentum of the system about
the system COM. Verified against analytical formula for spinning rigid body.

### AC3: Sensor regression
`SubtreeLinVel` and `SubtreeAngMom` sensor outputs match the **corrected
baseline** established by [CVEL_REFERENCE_POINT_FIXES.md](./CVEL_REFERENCE_POINT_FIXES.md)
(which must land first). For models without COM offsets (most existing tests),
values are unchanged from the original pre-fix baseline. For models WITH COM
offsets, the cvel fixes change the baseline, and §56 must match the corrected
values (±1e-14).

### AC4: O(n) complexity *(code review — not a runtime test)*
`mj_subtree_vel()` visits each body exactly once per phase (3 passes). No
parent-chain walks. No `is_body_in_subtree()` calls.

### AC5: Lazy gate — subtree not computed when no sensors need it
For a model with no `SubtreeLinVel`/`SubtreeAngMom`/`User` velocity sensors,
`mj_subtree_vel()` is never called. `flg_subtreevel` remains false.

### AC6: Lazy gate — rnepost not computed when no sensors need it
For a model with no accelerometer/force/torque/framelinacc/frameangacc/User
acc sensors, `mj_body_accumulators()` is never called. `flg_rnepost` remains
false. `cacc`/`cfrc_int`/`cfrc_ext` remain zeroed.

### AC7: Lazy gate — computed at most once per step
For a model with multiple accelerometer sensors, `mj_body_accumulators()` is
called exactly once (on the first such sensor). Second sensor sees
`flg_rnepost = true` and skips.

### AC8: Lazy gate — computed at most once per step (subtree)
For a model with multiple `SubtreeLinVel` sensors on different bodies,
`mj_subtree_vel()` is called exactly once.

### AC9: Flag clearing
`flg_rnepost` is false after `mj_fwd_constraint` and before `mj_sensor_acc`.
`flg_subtreevel` is false after `mj_fwd_velocity` starts.

### AC10: Reset clears fields
After `data.reset()`: `subtree_linvel` and `subtree_angmom` are zeroed,
both flags are false.

### AC11: Inverse dynamics still works
`inverse()` calls `mj_body_accumulators()` directly and sets `flg_rnepost`.
Subsequent `mj_sensor_acc()` does not recompute.

### AC12: `DISABLE_SENSOR` interaction
When `DISABLE_SENSOR` is set, `mj_sensor_vel()` and `mj_sensor_acc()` return
early. Neither `mj_subtree_vel()` nor `mj_body_accumulators()` is triggered.
The fields remain stale/zeroed. This matches MuJoCo.

### AC13: Sleep interaction
Lazy gates are orthogonal to sleep. If a sensor's body is asleep, the sensor
is skipped (existing behavior), but the lazy computation (if triggered by
another sensor) still runs for all bodies. MuJoCo's `mj_rnePostConstraint`
and `mj_subtreeVel` are not sleep-gated — they compute for all bodies.

---

## Test Plan

### T1: `subtree_linvel` — free-falling body
Single free body, initial velocity `[1, 0, 0]`. After `forward()`,
`subtree_linvel[1]` should be `[1, 0, 0]` (pre-integration velocity).

### T2: `subtree_linvel` — multi-body chain
3-body chain with different velocities. `subtree_linvel[0]` (world) =
mass-weighted average. `subtree_linvel[1]` (root of chain) = mass-weighted
average of chain bodies.

### T3: `subtree_angmom` — spinning body
Single body rotating about z-axis with known inertia. `subtree_angmom[1]`
should equal `I_zz * omega_z`.

### T4: `subtree_angmom` — orbital
Two bodies orbiting common center. `subtree_angmom[0]` includes orbital +
spin contributions.

### T5: Sensor regression
Model with `SubtreeLinVel` and `SubtreeAngMom` sensors. Compare outputs
before/after refactor. Must be identical (bitwise or ±1e-15).

### T6: Lazy gate — no sensors
Model with no subtree or acc sensors. Step 100 times. Assert
`flg_subtreevel == false` and `flg_rnepost == false` after each step.

### T7: Lazy gate — triggered once
Model with 3 accelerometer sensors. Instrument (or assert post-hoc) that
`mj_body_accumulators()` ran exactly once per step.

### T8: Lazy gate — inverse dynamics
Call `forward()` on model with no acc sensors (flg_rnepost = false).
Then call `inverse()`. Assert `flg_rnepost = true` and `cacc` is populated.

### T9: Reset clears everything
Step, then reset, then check fields are zeroed and flags are false.

### T10: O(n) vs O(n²) equivalence
For a 10-body chain, compare `mj_subtree_vel()` output against the old
`compute_subtree_momentum()`/`compute_subtree_angmom()` helpers. Values
must match within f64 epsilon.

### T11: DISABLE_SENSOR — neither lazy gate triggered
Model with subtree + acc sensors, `DISABLE_SENSOR` flag set. After
`forward()`: `flg_subtreevel == false`, `flg_rnepost == false`. Neither
`mj_subtree_vel()` nor `mj_body_accumulators()` was called. Fields remain
zeroed.

### T12: Sleep interaction
Model with 2 bodies — one sleeping, one awake. `SubtreeLinVel` sensor on
the awake body. After `forward()`: `flg_subtreevel == true`.
`mj_subtree_vel()` computed values for ALL bodies (including the sleeping
one) because MuJoCo's `mj_subtreeVel()` and `mj_rnePostConstraint()` are
not sleep-gated — they compute for the entire tree.

---

## Files Affected

| File | Change |
|------|--------|
| `sim/L0/core/src/types/data.rs` | Add `subtree_linvel`, `subtree_angmom`, `flg_rnepost`, `flg_subtreevel` fields |
| `sim/L0/core/src/types/model_init.rs` | Allocate new fields |
| `sim/L0/core/src/forward/mod.rs` | Remove unconditional `mj_body_accumulators()` call; clear `flg_rnepost` |
| `sim/L0/core/src/forward/velocity.rs` | Clear `flg_subtreevel`; add `mj_subtree_vel()` |
| `sim/L0/core/src/forward/acceleration.rs` | Set `flg_rnepost` in `mj_body_accumulators()` |
| `sim/L0/core/src/sensor/velocity.rs` | Demand-trigger `mj_subtree_vel()`; read from persistent fields |
| `sim/L0/core/src/sensor/acceleration.rs` | Demand-trigger `mj_body_accumulators()` |
| `sim/L0/core/src/sensor/derived.rs` | Delete `compute_subtree_momentum()`, `compute_subtree_angmom()` |
| `sim/L0/core/src/inverse.rs` | No change (already calls `mj_body_accumulators` directly) |
| `sim/L0/tests/integration/` | New test file `lazy_eval.rs` or `subtree_vel.rs` |

---

## Out of Scope

- `flg_energypos` / `flg_energyvel` — already gated behind `ENABLE_ENERGY` flag. Not lazy in the same sense (it's a user-facing toggle, not demand-driven by sensors). Could be unified later but no conformance gap today.
- 4A.6b: Refactoring `compute_site_force_torque()` to read `cfrc_int` instead of recomputing from first principles. Tracked as a follow-on conformance improvement.
