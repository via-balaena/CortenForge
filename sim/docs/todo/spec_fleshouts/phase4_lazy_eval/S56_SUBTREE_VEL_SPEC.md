# §56 — `subtree_linvel` / `subtree_angmom` Persistent Data Fields

**Status:** Draft
**Source:** `future_work_14.md` §56
**MuJoCo ref:** `mj_subtreeVel()` in `engine_core_smooth.c`
**Prerequisites:**
- `subtree_com`, `subtree_mass` (already computed in position stage)
- [CVEL_REFERENCE_POINT_FIXES.md](./CVEL_REFERENCE_POINT_FIXES.md) must land
  first — the corrected O(n²) helpers serve as the validation baseline for T9
  (O(n) equivalence test)

---

## Problem

`subtree_linvel` and `subtree_angmom` do not exist as persistent `Data` fields.
Two problems follow:

1. **No persistent fields.** MuJoCo stores `subtree_linvel` (3 × nbody) and
   `subtree_angmom` (3 × nbody) on `mjData`. Users and downstream code can
   read them anytime after `mj_subtreeVel()` has run. We have no equivalent.

2. **O(n²) per-sensor computation.** Our `SubtreeLinVel` and `SubtreeAngMom`
   sensors call `compute_subtree_momentum()` and `compute_subtree_angmom()`
   from `sensor/derived.rs`. Each helper walks the parent chain for every body
   to check descendancy — O(depth) per body, O(n × depth) per sensor call.
   With k sensors on different bodies, total cost is O(k × n × depth).
   MuJoCo's `mj_subtreeVel()` is O(n) total and computed once.

---

## MuJoCo Algorithm: `mj_subtreeVel()`

Three-phase O(n) backward accumulation. Requires a temporary `body_vel`
array (6 × nbody) for per-body COM velocities in world frame.

### Inputs (read-only)

| Field | Source | Description |
|-------|--------|-------------|
| `body_mass[b]` | Model | Scalar mass of body b |
| `body_inertia[b]` | Model | Diagonal principal inertia `[I_xx, I_yy, I_zz]` |
| `body_subtreemass[b]` | Model | Total mass of subtree rooted at b (**see convention note below**) |
| `body_parent[b]` | Model | Parent body index |
| `cvel[b]` | Data | 6D spatial velocity `[ω; v]` at reference point |
| `xpos[b]` | Data | Body origin in world frame |
| `xipos[b]` | Data | Body COM in world frame |
| `ximat[b]` | Data | Body inertia frame rotation (3×3) |
| `subtree_com[b]` | Data | Subtree COM in world frame (from position stage) |

### Outputs (written)

| Field | Shape | Description |
|-------|-------|-------------|
| `subtree_linvel[b]` | nbody × 3 | Linear velocity of subtree b's COM |
| `subtree_angmom[b]` | nbody × 3 | Angular momentum of subtree b about `subtree_com[b]` |

### Convention Note: `subtree_mass` — Data vs Model

MuJoCo stores `body_subtreemass[b]` on `mjModel` (static — computed once at
model compile time). Our `subtree_mass[b]` is on `Data` (recomputed each
`forward()` in the position stage alongside `subtree_com`). Both approaches
are correct for rigid-body models where body masses don't change. Our
convention is the more general one — if masses ever became state-dependent
(e.g., fuel consumption), it would already work. For §56, we read
`data.subtree_mass[b]` where MuJoCo reads `m->body_subtreemass[b]`.

### Convention Note: `cvel` Reference Point

MuJoCo's `cvel[b]` is stored at `subtree_com[body_rootid[b]]` (the root of
b's kinematic tree). It calls `mj_objectVelocity(flg_local=0)` to shift
`cvel` to `xipos[b]`.

**Our `cvel[b]` is stored at `xpos[b]`** (the body origin, not subtree COM
of the root). To get the velocity at `xipos[b]` (body COM) we need:

```
ω_com = cvel[b][0:3]              // angular unchanged by translation
v_com = cvel[b][3:6] + ω × (xipos[b] - xpos[b])   // lever arm shift
```

This is simpler than MuJoCo's transform because our reference point (`xpos`)
is closer to the target (`xipos`).

### Phase 1: Initialize Per-Body Quantities

For each body b:

```
body_vel[b] = shift_cvel_to_xipos(cvel[b], xpos[b], xipos[b])
    ω_b = body_vel[b][0:3]      // angular velocity in world frame
    v_b = body_vel[b][3:6]      // linear velocity at COM in world frame

// Linear momentum
subtree_linvel[b] = body_mass[b] * v_b

// Spin angular momentum about own COM
ω_local = ximat[b]ᵀ * ω_b
L_local = [I_xx * ω_local.x, I_yy * ω_local.y, I_zz * ω_local.z]
subtree_angmom[b] = ximat[b] * L_local
```

### Phase 2: Backward Accumulation — Linear Momentum → Velocity

Iterate bodies from leaf to root (b = nbody-1 down to 0):

```
// Accumulate momentum into parent
if b > 0:
    subtree_linvel[parent[b]] += subtree_linvel[b]

// Convert momentum to velocity
subtree_linvel[b] /= max(ε, subtree_mass[b])
```

After this phase, `subtree_linvel[b]` is the true COM velocity of subtree b.

### Phase 3: Backward Accumulation — Angular Momentum

Iterate bodies from leaf to root (b = nbody-1 down to 1, skipping world body):

```
parent = body_parent[b]

// Part A: Correct body b's angmom from "about xipos[b]" to "about subtree_com[b]"
dx = xipos[b] - subtree_com[b]
dv = body_vel[b][3:6] - subtree_linvel[b]
dp = body_mass[b] * dv
dL = dx × dp
subtree_angmom[b] += dL

// Part B: Transfer subtree b's angmom to parent
subtree_angmom[parent] += subtree_angmom[b]

// Part C: Reference point correction (subtree_com[b] → subtree_com[parent])
dx = subtree_com[b] - subtree_com[parent]
dv = subtree_linvel[b] - subtree_linvel[parent]
dp = subtree_mass[b] * dv
dL = dx × dp
subtree_angmom[parent] += dL
```

**Derivation of Part C:** When transferring angular momentum from reference
point A to reference point B, the correction is:

```
L_B = L_A + (r_A - r_B) × M * (v_A - v_B)
```

Here A = `subtree_com[b]`, B = `subtree_com[parent]`, M = `subtree_mass[b]`,
v_A = `subtree_linvel[b]`, v_B = `subtree_linvel[parent]`. The formula is
the parallel axis theorem for angular momentum (not the moment-of-inertia
parallel axis theorem — this is the momentum transfer identity).

---

## Implementation Plan

### Step 1: Data Fields

Add to `Data` (in the subtree mass/COM section):

```rust
/// Linear velocity of subtree COM in world frame (length `nbody`).
/// `subtree_linvel[0]` = whole-model COM velocity.
/// Computed on demand by `mj_subtree_vel()`.
pub subtree_linvel: Vec<Vector3<f64>>,

/// Angular momentum of subtree about `subtree_com[b]` in world frame (length `nbody`).
/// `subtree_angmom[0]` = total angular momentum about system COM.
/// Computed on demand by `mj_subtree_vel()`.
pub subtree_angmom: Vec<Vector3<f64>>,

/// Lazy flag: `mj_subtree_vel()` has been called this step.
/// Cleared at velocity stage start, set by `mj_subtree_vel()`.
pub flg_subtreevel: bool,
```

### Step 2: Allocation (`model_init.rs`)

```rust
subtree_linvel: vec![Vector3::zeros(); self.nbody],
subtree_angmom: vec![Vector3::zeros(); self.nbody],
flg_subtreevel: false,
```

### Step 3: Reset (`data.rs`)

In `Data::reset()`, in the subtree section:

```rust
for v in &mut self.subtree_linvel { *v = Vector3::zeros(); }
for v in &mut self.subtree_angmom { *v = Vector3::zeros(); }
self.flg_subtreevel = false;
```

### Step 4: Clone

Add `subtree_linvel`, `subtree_angmom`, `flg_subtreevel` to the manual
`Clone` impl.

### Step 5: Flag Clearing

In `mj_fwd_velocity()` (`forward/velocity.rs`), at the very start:

```rust
data.flg_subtreevel = false;
```

This invalidates the previous step's values whenever velocity kinematics
reruns (new qvel → new cvel → old subtree velocities are stale).

### Step 6: `mj_subtree_vel()` Implementation

New public function. Location: `forward/velocity.rs` (colocated with
`mj_fwd_velocity` since it operates on velocity-stage data).

```rust
/// Compute subtree linear velocity and angular momentum (§56).
///
/// O(n) three-phase backward accumulation:
/// 1. Initialize per-body linear momentum and spin angular momentum
/// 2. Backward accumulate linear momentum → convert to velocity
/// 3. Backward accumulate angular momentum with reference-point corrections
///
/// Requires `subtree_com` and `subtree_mass` from position stage,
/// and `cvel` from velocity stage.
///
/// # MuJoCo Equivalence
///
/// Matches `mj_subtreeVel()` in `engine_core_smooth.c`.
pub fn mj_subtree_vel(model: &Model, data: &mut Data) {
    // Temporary: per-body 6D velocity at COM in world frame
    let mut body_vel: Vec<[f64; 6]> = vec![[0.0; 6]; model.nbody];

    // Phase 1: Per-body quantities
    for b in 0..model.nbody {
        let cvel = &data.cvel[b];
        let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
        let v_origin = Vector3::new(cvel[3], cvel[4], cvel[5]);

        // Shift linear velocity from xpos[b] to xipos[b]
        let dif = data.xipos[b] - data.xpos[b];
        let v_com = v_origin + omega.cross(&dif);

        body_vel[b] = [omega.x, omega.y, omega.z, v_com.x, v_com.y, v_com.z];

        // Linear momentum
        let mass = model.body_mass[b];
        data.subtree_linvel[b] = mass * v_com;

        // Spin angular momentum: ximat * diag(I) * ximat^T * omega
        let inertia = model.body_inertia[b];
        let ximat = &data.ximat[b];
        let omega_local = ximat.transpose() * omega;
        let i_omega_local = Vector3::new(
            inertia.x * omega_local.x,
            inertia.y * omega_local.y,
            inertia.z * omega_local.z,
        );
        data.subtree_angmom[b] = ximat * i_omega_local;
    }

    // Phase 2: Backward accumulate linear momentum, then convert to velocity
    for b in (0..model.nbody).rev() {
        if b > 0 {
            let parent = model.body_parent[b];
            data.subtree_linvel[parent] += data.subtree_linvel[b];
        }
        let smass = data.subtree_mass[b];
        // MuJoCo uses MJ_MINVAL = 1e-15 as the zero-mass guard
        if smass > 1e-15 {
            data.subtree_linvel[b] /= smass;
        } else {
            data.subtree_linvel[b] = Vector3::zeros();
        }
    }

    // Phase 3: Backward accumulate angular momentum
    for b in (1..model.nbody).rev() {
        let parent = model.body_parent[b];
        let mass = model.body_mass[b];
        let smass = data.subtree_mass[b];
        let v_body = Vector3::new(body_vel[b][3], body_vel[b][4], body_vel[b][5]);

        // Part A: Correct angmom from "about xipos[b]" to "about subtree_com[b]"
        let dx_a = data.xipos[b] - data.subtree_com[b];
        let dv_a = v_body - data.subtree_linvel[b];
        let dp_a = mass * dv_a;
        data.subtree_angmom[b] += dx_a.cross(&dp_a);

        // Part B: Transfer subtree b's angmom to parent
        let angmom_b = data.subtree_angmom[b];
        data.subtree_angmom[parent] += angmom_b;

        // Part C: Reference point correction (subtree_com[b] → subtree_com[parent])
        let dx_c = data.subtree_com[b] - data.subtree_com[parent];
        let dv_c = data.subtree_linvel[b] - data.subtree_linvel[parent];
        let dp_c = smass * dv_c;
        data.subtree_angmom[parent] += dx_c.cross(&dp_c);
    }

    data.flg_subtreevel = true;
}
```

### Step 7: Demand Triggering in `mj_sensor_vel()`

Replace the on-demand computation with a persistent field read:

```rust
MjSensorType::SubtreeLinVel => {
    if !data.flg_subtreevel {
        mj_subtree_vel(model, data);
    }
    if objid < model.nbody {
        sensor_write3(&mut data.sensordata, adr, &data.subtree_linvel[objid]);
    }
}

MjSensorType::SubtreeAngMom => {
    if !data.flg_subtreevel {
        mj_subtree_vel(model, data);
    }
    if objid < model.nbody {
        sensor_write3(&mut data.sensordata, adr, &data.subtree_angmom[objid]);
    }
}

// User sensors at VEL stage — conservative trigger
MjSensorType::User => {
    if model.sensor_datatype[sensor_id] == MjSensorDataType::Velocity {
        if !data.flg_subtreevel {
            mj_subtree_vel(model, data);
        }
        if let Some(ref cb) = model.cb_sensor {
            (cb.0)(model, data, sensor_id, SensorStage::Vel);
        }
    }
}
```

### Step 8: Delete Dead Helpers

Remove from `sensor/derived.rs`:
- `compute_subtree_momentum()` — replaced by `subtree_linvel`
- `compute_subtree_angmom()` — replaced by `subtree_angmom`

Remove the imports from `sensor/velocity.rs`:
- `compute_subtree_angmom`
- `compute_subtree_com` (no longer needed in velocity stage)
- `compute_subtree_momentum`

**Resolved:** `compute_subtree_com()` should also be deleted. The `SubtreeCom`
sensor (`sensor/position.rs:163`) currently calls `compute_subtree_com()` but
the persistent `data.subtree_com[objid]` field — computed via backward
accumulation in `forward/position.rs:187-211` — is already available and is
more accurate (consistent floating-point ordering with the rest of the
pipeline). The sensor should read the field directly:

```rust
MjSensorType::SubtreeCom => {
    if objid < model.nbody {
        sensor_write3(&mut data.sensordata, adr, &data.subtree_com[objid]);
    }
}
```

Delete `compute_subtree_com()` from `sensor/derived.rs`. This also eliminates
the O(n × depth) parent-chain walk per sensor invocation.

### Step 9: Update Data Size Staleness Guard

The `data_reset_field_inventory` compile-time size check will break when new
fields are added. Update `EXPECTED_SIZE` in the test.

---

## Acceptance Criteria

### AC1: Field existence
`data.subtree_linvel` and `data.subtree_angmom` are `Vec<Vector3<f64>>` of
length `nbody` on `Data`.

### AC2: Whole-system COM velocity
For a single free body with mass 1.0 and initial velocity `[1, 2, 3]`:
`data.subtree_linvel[0]` = `[1, 2, 3]` after `forward()` (world body subtree
= entire model).

### AC3: Mass-weighted average
3-body chain: body A (mass 2, v=[1,0,0]), body B (mass 1, v=[4,0,0]), body C
(mass 1, v=[0,0,0]). `subtree_linvel[root]` = (2*1 + 1*4 + 1*0) / 4 =
`[1.5, 0, 0]`.

### AC4: Spin angular momentum
Single body, inertia I_zz = 2.0, rotating at ω_z = 3.0 rad/s.
`subtree_angmom[body]` = `[0, 0, 6.0]`.

### AC5: Orbital angular momentum
Body at position [1,0,0] with velocity [0,1,0], mass 1.0, subtree COM at
origin. Orbital contribution: [1,0,0] × (1.0 * [0,1,0]) = [0,0,1].

### AC6: Sensor regression
Run existing `SubtreeLinVel` and `SubtreeAngMom` sensor tests. Output values
must be identical (±1e-14) to pre-refactor values.

### AC7: O(n) complexity *(code review — not a runtime test)*
`mj_subtree_vel()` contains no parent-chain walks. Each body visited exactly
once per phase (3 passes over nbody). No `is_body_in_subtree()` calls.

### AC8: Lazy — not computed when no sensors need it
Model with only position/acceleration sensors. After `forward()`,
`flg_subtreevel == false`. Fields remain zeroed.

### AC9: Lazy — computed once
Model with 5 `SubtreeLinVel` sensors on different bodies. `mj_subtree_vel()`
called exactly once on the first sensor. Subsequent sensors see
`flg_subtreevel == true`.

### AC10: Flag cleared each step
After step N, `flg_subtreevel == true` (sensors triggered it). At the start
of step N+1's velocity stage, `flg_subtreevel` is cleared to false.

### AC11: Reset
After `data.reset()`: both fields zeroed, `flg_subtreevel == false`.

### AC12: `SubtreeCom` sensor reads persistent field *(code review)*
`SubtreeCom` sensor reads `data.subtree_com[objid]` directly, not via
`compute_subtree_com()` helper.

### AC13: Dead code removed *(code review)*
`compute_subtree_momentum()`, `compute_subtree_angmom()`, and
`compute_subtree_com()` deleted from `sensor/derived.rs`. No references
remain.

---

## Test Plan

### T1: Free body COM velocity
Single free body, mass=1, v=[1,2,3]. `subtree_linvel[1]` = [1,2,3].
`subtree_linvel[0]` = [1,2,3] (world subtree = whole model).

### T2: Multi-body mass-weighted velocity
3-link chain, masses [2, 1, 1], velocities [[1,0,0], [4,0,0], [0,0,0]].
Verify `subtree_linvel` at each level of the tree.

### T3: Spinning body angular momentum
Hinge joint, I_zz = 2.0, qvel = 3.0. `subtree_angmom[body]` ≈ [0, 0, 6.0].

### T4: Orbital + spin angular momentum
Two bodies, one orbiting the other. Verify total `subtree_angmom[0]` matches
analytical L = I*ω + m*r×v.

### T5: Sensor regression — SubtreeLinVel
Build model with SubtreeLinVel sensor. Compare old vs new output. Must match.

### T6: Sensor regression — SubtreeAngMom
Build model with SubtreeAngMom sensor. Compare old vs new output. Must match.

### T7: Lazy gate — no trigger
Model with no subtree sensors. Step. Assert `flg_subtreevel == false`.

### T8: Lazy gate — single trigger
Model with 3 SubtreeLinVel sensors. Step. Assert `flg_subtreevel == true`.
All 3 sensor values populated correctly.

### T9: O(n) equivalence
10-body chain. Compare `mj_subtree_vel()` output against brute-force
`compute_subtree_momentum()/compute_subtree_angmom()` for each body. Must
match within f64 epsilon.

### T10: Reset clears fields
Step, then reset. Assert both fields zeroed and flag false.

### T11: SubtreeCom direct read
Model with SubtreeCom sensor. Verify sensor reads `data.subtree_com[objid]`
and matches expected value.

### T12: World body subtree_angmom
World body (body 0) has zero inertia and zero mass. `subtree_angmom[0]` =
total angular momentum of the entire system about the system COM. Verify this
matches the sum of all body contributions (orbital + spin). World body's own
contribution is zero.

### T13: Zero-mass body in chain
3-body chain where middle body has `mass = 0`. No NaN or panic.
`subtree_linvel` for the parent subtree uses the zero-mass guard (divides
by max(ε, subtree_mass)). Zero-mass body's own `subtree_linvel` = zeros.

### T14: DISABLE_SENSOR interaction
Model with SubtreeLinVel sensor and `DISABLE_SENSOR` flag set.
`mj_sensor_vel()` returns early. `flg_subtreevel` stays false. Fields remain
zeroed. No crash.

### T15: Sleep interaction
Model with sleeping and awake bodies, SubtreeLinVel sensor on awake body.
`mj_subtree_vel()` is triggered (by the awake sensor) and computes for ALL
bodies — not just awake ones. MuJoCo's `mj_subtreeVel()` is not sleep-gated.

---

## Files Affected

| File | Change |
|------|--------|
| `types/data.rs` | Add `subtree_linvel`, `subtree_angmom`, `flg_subtreevel` |
| `types/model_init.rs` | Allocate new fields |
| `forward/velocity.rs` | Clear `flg_subtreevel`; add `mj_subtree_vel()` |
| `sensor/velocity.rs` | Demand-trigger; read persistent fields |
| `sensor/derived.rs` | Delete `compute_subtree_momentum()`, `compute_subtree_angmom()`, `compute_subtree_com()` |
| `sensor/position.rs` | `SubtreeCom` reads `data.subtree_com` directly |
| `tests/integration/` | New test file |
