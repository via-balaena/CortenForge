# Future Work 9 — Phase 3A: Noslip + Actuator/Dynamics + Tendon Equality (Items #33–37)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers the noslip post-processor for PGS/CG, actuator/dynamics features, and
tendon equality constraints.

**MuJoCo source references** (verified against MuJoCo 3.x, Feb 2026):
- `engine_solver.c` → `mj_solNoSlip()`
- `engine_forward.c` → `mj_fwdActuation()`, `mj_nextActivation()`
- `engine_passive.c` → `mj_gravcomp()`
- `engine_core_smooth.c` → `mj_transmission()` (`mjTRN_BODY` case)
- `engine_core_constraint.c` → `mjEQ_TENDON` case in equality assembly

---

### 33. Noslip Post-Processor for PGS/CG Solvers ✅
**Status:** Complete (A+ conformance) | **Effort:** S | **Prerequisites:** None

#### Current State

The noslip post-processor is **fully implemented with A+ MuJoCo conformance** as
`noslip_postprocess()` (`mujoco_pipeline.rs`, ~440 LOC).
Twenty integration tests verify it (3 in `newton_solver.rs`, 17 in `noslip.rs`):
- PGS/CG/Newton slip reduction, zero-iterations no-op
- Friction-loss clamping, pyramidal 2x2 block solve, elliptic QCQP
- Full-matrix residual check (efc_b correctness), cone constraint satisfaction
- Cross-coupling with limits, state-independent processing, convergence stability

The implementation:
1. Identifies noslip-eligible rows: friction-loss + contact friction (elliptic & pyramidal).
2. Builds friction-only Delassus submatrix `A` (UNREGULARIZED — no R on diagonal) and
   effective bias `b_eff[i] = efc_b[row_i] + Σ_{j NOT noslip} A[row_i,j] * f_fixed[j]`,
   absorbing cross-coupling from non-noslip rows into the bias vector.
3. Runs PGS iterations with per-type projection:
   - **Friction-loss**: scalar GS + interval clamping `[-floss, +floss]`
   - **Elliptic**: QCQP cone projection via `noslip_qcqp2()`/`noslip_qcqp3()` (Newton on
     dual Lagrange multiplier λ in Delassus-weighted norm)
   - **Pyramidal**: 2x2 block solve on `(f_pos, f_neg)` pairs with cost rollback
4. Cost-based convergence: `improvement * scale < noslip_tol` where
   `scale = 1/(meaninertia * max(1, nv))` (matches CG/Newton pattern).
5. Writes back updated `efc_force` and recomputes `qfrc_constraint` and `qacc`.

MJCF parsing is complete: `noslip_iterations` and `noslip_tolerance` are read from
`<option>` and stored on `Model` (defaults: 0 iterations, 1e-6 tolerance).
All solver paths (PGS, CG, Newton, Newton→PGS fallback) dispatch noslip when
`noslip_iterations > 0`.

#### MuJoCo Reference (`mj_solNoSlip` in `engine_solver.c`)

MuJoCo's noslip processes two row ranges (NOT equality rows):

1. **Friction-loss rows** (`ne..ne+nf`): PGS update with interval clamping
   `[-floss, +floss]`. Uses unregularized `ARinv[i]` = `1/A[i,i]` (computed
   via `ARdiaginv` with `flg_subR=1`, which subtracts R from the AR diagonal).
2. **Contact friction rows** (`ne+nf..nefc`): Two sub-cases:
   - **Pyramidal**: Processes friction pairs `(j, j+1)` via 2x2 block solve with
     non-negativity clamping on each pair component.
   - **Elliptic**: Solves a QCQP for tangential force components using
     `mju_QCQP2`/`mju_QCQP3` depending on dimension, rescales to friction cone.

Equality rows (`0..ne`) are **NOT** processed by noslip. This is a known MuJoCo
design decision (see MuJoCo issue #2430).

Convergence: breaks when `improvement < noslip_tolerance`.

#### Specification

##### S1. Verify `noslip_postprocess()` is solver-agnostic (confirmed)

The existing `noslip_postprocess()` operates on `efc_force`, `efc_type`,
`efc_dim`, `efc_state`, and the Jacobians — none of which are Newton-specific.
All three solvers use the same unified constraint assembly
(`assemble_unified_constraints()` called before solver dispatch). The `efc_*`
layout is identical regardless of solver type.

**Verified:** No Newton-specific state accessed. Function is already solver-agnostic.

##### S2. Add noslip call after PGS/CG solve

In `mj_fwd_constraint()`, at the solver dispatch (currently ~line 17886), add
noslip after PGS and CG:

```rust
match model.solver_type {
    SolverType::Newton => {
        let result = newton_solve(model, data);
        match result {
            NewtonResult::Converged => {
                if model.noslip_iterations > 0 {
                    noslip_postprocess(model, data);
                }
                data.newton_solved = true;
            }
            NewtonResult::CholeskyFailed | NewtonResult::MaxIterationsExceeded => {
                pgs_solve_unified(model, data);
                if model.noslip_iterations > 0 {  // NEW
                    noslip_postprocess(model, data);
                }
            }
        }
    }
    SolverType::CG => {
        cg_solve_unified(model, data);
        if model.noslip_iterations > 0 {  // NEW
            noslip_postprocess(model, data);
        }
    }
    SolverType::PGS => {
        pgs_solve_unified(model, data);
        if model.noslip_iterations > 0 {  // NEW
            noslip_postprocess(model, data);
        }
    }
}
```

Note: noslip also runs after Newton's PGS fallback — this matches MuJoCo behavior.

##### S3. Add friction-loss row processing to `noslip_postprocess()`

The existing implementation only processes contact friction rows. MuJoCo also
processes friction-loss rows (`ConstraintType::FrictionLoss`). Add a second loop
before the contact friction loop:

```rust
// Phase A: Friction-loss rows (dry friction on DOFs and tendons)
// MuJoCo range: [ne..ne+nf] — we identify by ConstraintType::FrictionLoss
for row in 0..nefc {
    if data.efc_type[row] != ConstraintType::FrictionLoss {
        continue;
    }
    // PGS update on UNREGULARIZED system: force[row] -= residual * diag_inv[row]
    let residual = compute_row_residual_no_reg(data, row);  // b + A*f (no R*f)
    let diag_inv = 1.0 / a_diag[row];  // UNREGULARIZED: 1/A[i,i], not 1/(A[i,i]+R[i])
    data.efc_force[row] -= residual * diag_inv;
    // Clamp to [-floss, +floss]
    let floss = data.efc_floss[row];
    data.efc_force[row] = data.efc_force[row].clamp(-floss, floss);
}
```

**Noslip solves the unregularized system for ALL row types.** MuJoCo's
`mj_solNoSlip` calls `ARdiaginv(m, d, ARinv, 1)` with `flg_subR=1`, which
computes `1 / (efc_AR[i,i] - R[i])` = `1 / A[i,i]` — stripping the
regularizer. Similarly, the residual uses `flg_subR=1`: `res = b + AR*f - R*f`
= `b + A*f`. This applies to both friction-loss rows AND contact friction rows.

This differs from the regular PGS solver (`mj_solPGS`) which calls
`ARdiaginv(m, d, ARinv, 0)` with `flg_subR=0`, solving the regularized system
`(A+R)*f = -b`. Noslip deliberately removes regularization to get a tighter
(less compliant) friction solution.

##### S4. Cone projection modes

Both pyramidal and elliptic friction cones are implemented (§32 ✅). MuJoCo's
noslip handles both:
- **Elliptic**: QCQP projection (our current implementation).
- **Pyramidal**: 2x2 block solve on friction pairs with non-negativity clamping.

Our current `noslip_postprocess()` only handles elliptic. For pyramidal support,
add a `match model.cone` branch in the contact friction loop. Elliptic uses the
existing QCQP path. Pyramidal uses the 2x2 block path described below.

**Delassus access note:** The existing elliptic noslip builds a dense
friction-only submatrix `a_fric` locally. For pyramidal noslip, we also need
local Delassus entries. The approach: build a local dense matrix for all
friction rows being processed (same pattern as existing elliptic noslip), then
index into it. The helper names below (`compute_delassus_cross`,
`compute_row_residual_no_reg`) represent operations on this local matrix, not
fields on Data.

**Pyramidal loop structure** (contact rows start at `ne + nf`):
```rust
// Iterate contact rows. Each pyramidal contact has 2*(dim-1) rows.
let mut i = ne_plus_nf;
while i < nefc {
    if data.efc_type[i] != ConstraintType::ContactPyramidal { i += 1; continue; }
    let dim = data.efc_dim[i];
    let npyramid = dim - 1;  // number of friction directions
    // Process pairs: (i, i+1), (i+2, i+3), ..., (i+2*npyramid-2, i+2*npyramid-1)
    for k in (0..2 * npyramid).step_by(2) {
        let j = i + k;
        noslip_pyramid_pair(data, a_diag, j);
    }
    i += 2 * npyramid;
}
```

**2x2 block solve algorithm** (MuJoCo `mj_solNoSlip`, verified against source):

The key insight: each pyramid pair `(f[j], f[j+1])` represents opposing edges
for one tangential friction direction. The normal force contribution is
`f[j] + f[j+1]` (held constant). The tangential redistribution is optimized
via a change of variables:

```rust
fn noslip_pyramid_pair(data: &mut Data, a_diag: &[f64], j: usize) {
    let old = [data.efc_force[j], data.efc_force[j + 1]];

    // Change of variables: f0 = mid + y, f1 = mid - y
    // mid is FIXED (preserves normal force from primary solve)
    let mid = 0.5 * (old[0] + old[1]);

    // Extract 2x2 block of UNREGULARIZED A matrix (no regularizer)
    // A[j,j]   = a_diag[j]    (already without R if we extract correctly)
    // A[j,j+1] = cross-term from Delassus submatrix
    let a00 = a_diag[j];
    let a11 = a_diag[j + 1];
    let a01 = compute_delassus_cross(data, j, j + 1);  // off-diagonal

    // Compute residual without regularizer contribution
    let res = [
        compute_row_residual_no_reg(data, j),
        compute_row_residual_no_reg(data, j + 1),
    ];

    // Constant part: bc = residual - A * old_force
    let bc = [
        res[0] - a00 * old[0] - a01 * old[1],
        res[1] - a01 * old[0] - a11 * old[1],
    ];

    // 1D curvature and gradient for optimization over y
    let k1 = a00 + a11 - 2.0 * a01;  // curvature (always >= 0)
    let k0 = mid * (a00 - a11) + bc[0] - bc[1];  // gradient at y=0

    const MIN_VAL: f64 = 1e-10;

    if k1 < MIN_VAL {
        // Degenerate curvature: split evenly
        data.efc_force[j] = mid;
        data.efc_force[j + 1] = mid;
    } else {
        // Unconstrained 1D minimum
        let y = -k0 / k1;
        // Clamp: enforce f0 >= 0 and f1 >= 0 ⟹ |y| <= mid
        if y < -mid {
            data.efc_force[j] = 0.0;
            data.efc_force[j + 1] = 2.0 * mid;
        } else if y > mid {
            data.efc_force[j] = 2.0 * mid;
            data.efc_force[j + 1] = 0.0;
        } else {
            data.efc_force[j] = mid + y;
            data.efc_force[j + 1] = mid - y;
        }
    }

    // Cost change check — rollback if cost increased (safety mechanism)
    let d0 = data.efc_force[j] - old[0];
    let d1 = data.efc_force[j + 1] - old[1];
    let cost = 0.5 * (d0 * d0 * a00 + d1 * d1 * a11 + 2.0 * d0 * d1 * a01)
             + d0 * res[0] + d1 * res[1];
    if cost > 1e-10 {
        data.efc_force[j] = old[0];
        data.efc_force[j + 1] = old[1];
    }
}
```

**Key conformance details:**
- ALL noslip processing uses the **unregularized A** (without R on diagonal).
  This applies uniformly to friction-loss rows (S3) and contact friction rows
  (S4). MuJoCo passes `flg_subR=1` to both `ARdiaginv()` and `residual()`.
- `mid` is held constant — normal force is preserved from the primary solver.
  Only the tangential distribution between opposing edges is optimized.
- Cost rollback prevents the noslip iteration from worsening the solution.
- The `compute_delassus_cross(data, j, j+1)` extracts the unregularized
  off-diagonal entry. Since `efc_AR` stores `A + diag(R)`, the off-diagonal
  entries are already `A[j,k]` (R only appears on the diagonal).

#### Acceptance Criteria

1. ✅ **PGS + noslip**: Box on 20° incline with `solver="PGS"` — noslip reduces slip.
2. ✅ **CG + noslip**: Same benchmark with `solver="CG"` — noslip reduces slip.
3. ✅ **Newton regression**: Newton noslip tests pass (no behavior change).
4. ✅ **Newton fallback**: Newton → PGS fallback also runs noslip (wired in dispatch).
5. ✅ **`noslip_iterations=0` is no-op**: For PGS/CG, zero iterations = no change.
6. ✅ **Normal forces unchanged**: Contact normal forces bit-exact with/without noslip.
7. ✅ **Friction-loss rows**: Forces clamped to `[-floss, +floss]` after noslip.
8. ✅ **Pyramidal noslip**: 2x2 block solve reduces slip, forces non-negative.
9. ✅ **Full-matrix residual (efc_b)**: After convergence,
   `efc_b[i] + Σ_j A[i,j]*f[j] ≈ 0` for unclamped friction rows.
   Verifies correct RHS (efc_b, not efc_jar) and cross-coupling.
10. ✅ **Elliptic cone satisfied**: After QCQP projection,
    `Σ(f_j/μ_j)² ≤ f_normal²` holds for all contact groups.
11. ✅ **Cross-coupling**: Noslip works correctly with mixed constraint types
    (limits + friction-loss + contacts) — b_eff absorbs non-noslip coupling.
12. ✅ **State-independent**: Contacts near zero normal force (Satisfied state)
    processed without crash — QCQP zeroes friction when fn ≈ 0.

#### Implementation Notes

**A+ conformance fixes applied:**
- **RHS vector**: Uses `efc_b` (constraint bias, fixed at assembly) — NOT `efc_jar`
  (which is stale after PGS, only updated during warmstart).
- **Cross-coupling**: Delassus build iterates ALL `nefc` rows, absorbing non-noslip
  coupling into `b_eff[fi]`. PGS iteration sees the full-matrix residual.
- **QCQP cone projection**: `noslip_qcqp2()` (condim=3) and `noslip_qcqp3()`
  (condim=4) match MuJoCo's `mju_QCQP2`/`mju_QCQP3` — Newton on dual λ in
  Delassus-weighted norm, not simple rescaling.
- **Cost-based convergence**: `improvement * scale < noslip_tol` with
  `scale = 1/(meaninertia * max(1, nv))`, matching CG/Newton pattern.
- **No state filtering**: All contacts processed regardless of `efc_state`.

**Unified constraint layout.** All three solvers call
`assemble_unified_constraints()` before dispatch. The `efc_*` arrays are
identical regardless of solver type. No layout verification needed.

**Friction-loss data.** The `efc_floss` field on Data stores per-row friction
loss saturation values (populated during constraint assembly for `FrictionLoss`
rows). This is the clamping limit for the noslip PGS update.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add noslip call after PGS/CG solve
  (6 lines at dispatch); add friction-loss loop and pyramidal branch to
  `noslip_postprocess()` (~60-80 LOC)
- `sim/L0/tests/integration/` — new tests for PGS+noslip, CG+noslip,
  friction-loss+noslip, pyramidal+noslip

---

### 34. `actlimited` / `actrange` / `actearly` — Activation State Clamping ✅
**Status:** Complete (A+ conformance) | **Effort:** S–M | **Prerequisites:** None

#### Current State

**Fully implemented with A+ MuJoCo conformance.** The `mj_next_activation()`
helper (`mujoco_pipeline.rs`) handles integration (Euler/FilterExact) and
`actlimited` clamping in a single function, matching MuJoCo's
`mj_nextActivation()` from `engine_forward.c`. Fifteen integration tests
verify the implementation (`activation_clamping.rs`).

The implementation:
1. Stores `actuator_actlimited`, `actuator_actrange`, `actuator_actearly` on
   Model (forwarded from parsed MJCF via model builder).
2. `mj_next_activation()` integrates activation (Euler or FilterExact exact
   exponential) and clamps to `actrange` when `actlimited` is true.
3. `actearly` in `mj_fwd_actuation()`: when true, force computation uses
   the predicted next-step activation (integrated + clamped) instead of
   current activation, removing the one-timestep delay.
4. All three integration paths (Euler, RK4, GPU) use the unified clamping.
5. Muscle default: `actlimited=true, actrange=[0,1]` when not explicitly set.
6. `autolimits` compiler flag infers `actlimited` from `actrange` presence.
7. `act_dot` is computed from UNCLAMPED current activation (MuJoCo convention).

#### Objective

~~Parse `actlimited` and `actrange` from MJCF~~ Done, store on Model, clamp
activation state after dynamics integration, and wire `actearly` for semi-implicit
force computation.

#### MuJoCo Reference (`engine_forward.c`)

MuJoCo implements activation clamping via a helper function `mj_nextActivation()`:

```c
mjtNum mj_nextActivation(const mjModel* m, const mjData* d,
                         int actuator_id, int act_adr, mjtNum act_dot) {
    mjtNum act = d->act[act_adr];

    if (dyntype == mjDYN_FILTEREXACT) {
        // Exact: act(h) = act(0) + act_dot * tau * (1 - exp(-h/tau))
        mjtNum tau = max(MINVAL, dynprm[0]);
        act = act + act_dot * tau * (1 - exp(-timestep / tau));
    } else {
        // Euler: act = act + act_dot * h
        act = act + act_dot * timestep;
    }

    // Clamp AFTER integration
    if (m->actuator_actlimited[i]) {
        act = clip(act, actrange[0], actrange[1]);
    }
    return act;
}
```

**`actearly` behavior** (force computation in `mj_fwdActuation`):
```c
if (actearly[i]) {
    act = mj_nextActivation(m, d, i, act_adr, act_dot[act_adr]);  // predicted next-step
} else {
    act = d->act[act_adr];  // current activation
}
force[i] = gain * act + bias;
```

When `actearly=true`, the force at time `t` uses the activation at time `t+1`
(integrated + clamped). This removes the one-timestep delay between control
input and force response. The `actlimited` clamp applies inside
`nextActivation` in both the `actearly` path and the actual state update path.

**Key details:**
- `act_dot` is always computed from the UNCLAMPED current `d->act[act_adr]`.
- The clamp only applies after integration, never before.
- `mj_nextActivation` is called twice: once for `actearly` force computation
  (if enabled), and once for the actual state update at end-of-step.
- For `FilterExact`, the integration uses the exact exponential formula
  instead of Euler. The Euler path handles `Integrator`, `Filter`, and `Muscle`.

#### Specification

##### S1. Model storage

Add three fields to `Model`:

```rust
pub actuator_actlimited: Vec<bool>,    // default: false
pub actuator_actrange: Vec<(f64, f64)>,  // default: (0.0, 0.0)
pub actuator_actearly: Vec<bool>,      // default: false
```

Initialize from parsed MJCF values in `model_builder.rs`. Forward from
`MjcfActuator.actlimited`, `.actrange`, `.actearly`.

##### S2. `mj_next_activation()` helper

Implement a helper function matching MuJoCo's `mj_nextActivation`:

```rust
fn mj_next_activation(
    model: &Model,
    data: &Data,
    actuator_id: usize,
    act_adr: usize,
    act_dot: f64,
) -> f64 {
    let mut act = data.act[act_adr];

    // Integration step
    if model.actuator_dyntype[actuator_id] == ActuatorDynamics::FilterExact {
        let tau = model.actuator_dynprm[actuator_id][0].max(1e-10);
        act += act_dot * tau * (1.0 - (-model.timestep / tau).exp());
    } else {
        act += act_dot * model.timestep;
    }

    // Activation clamping
    if model.actuator_actlimited[actuator_id] {
        let range = model.actuator_actrange[actuator_id];
        act = act.clamp(range[0], range[1]);
    }

    act
}
```

##### S3. `actearly` in force computation

In `mj_fwd_actuation()`, Phase 2 (force generation), where the `input` variable
is determined for each actuator:

```rust
let input = if model.actuator_actadr[i].is_none() {
    ctrl_clamped  // stateless actuator
} else {
    let act_adr = model.actuator_actadr[i].unwrap();
    if model.actuator_actearly[i] {
        mj_next_activation(model, data, i, act_adr, data.act_dot[act_adr])
    } else {
        data.act[act_adr]
    }
};
// Then: force = gain * input + bias
```

##### S4. Activation state update in integrator

In the integration step (Euler / RK4 / Implicit), replace the current
muscle-only clamping with `mj_next_activation()`:

**Current code** (integration step):
```rust
// Muscle hardcoded clamp
if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
    self.act[act_adr] = self.act[act_adr].clamp(0.0, 1.0);
}
```

**New code:**
```rust
let new_act = mj_next_activation(model, self, i, act_adr, self.act_dot[act_adr]);
self.act[act_adr] = new_act;
```

This naturally handles:
- **Muscle with no `actlimited`**: `mj_next_activation` does Euler integration
  with no clamping. But MuJoCo still clamps muscles to `[0, 1]` by default even
  without `actlimited`. We preserve this by setting `actlimited=true` and
  `actrange=[0, 1]` for muscles during model building when `actlimited` is not
  explicitly set.
- **Muscle with explicit `actlimited`**: Uses the user-specified `actrange`.
- **Filter/Integrator with `actlimited`**: Clamps after integration.
- **Filter/Integrator without `actlimited`**: Unbounded (correct).
- **FilterExact**: Uses the exact exponential formula.

##### S5. Muscle default actrange in model builder

In `model_builder.rs`, when processing muscle actuators where `actlimited` is
not explicitly set:

```rust
if actuator_type == MjcfActuatorType::Muscle && !actuator.actlimited.unwrap_or(false) {
    model.actuator_actlimited[i] = true;
    model.actuator_actrange[i] = [0.0, 1.0];
}
```

This preserves backward compatibility: muscles default to `[0, 1]` clamping
without requiring explicit `actlimited="true"` in MJCF.

#### Acceptance Criteria

1. ✅ **Integrator clamping**: An integrator-type actuator with
   `actlimited="true" actrange="-1 1"` has activation clamped to `[-1, 1]`
   after 1000 steps of `ctrl=1.0`.
2. ✅ **Unbounded without actlimited**: Same actuator without `actlimited` reaches
   activation > 1 (existing behavior preserved).
3. ✅ **Muscle default**: Muscle actuators still default to `[0, 1]` clamping when
   `actlimited` is not explicitly set.
4. ✅ **Muscle override**: Muscle with `actlimited="true" actrange="-0.5 1.5"`
   clamps to `[-0.5, 1.5]` instead of `[0, 1]`.
5. ✅ **FilterExact integration**: A `<position>` actuator (FilterExact dynamics)
   with `actlimited="true"` uses the exact exponential integration formula
   AND clamps to actrange.
6. ✅ **`actearly` force**: With `actearly="true"`, the force at step `t` uses
   `act(t+1)` instead of `act(t)`. Verify by comparing a single-step force
   output: with `actearly`, force responds to ctrl change one step earlier.
7. ✅ **`actearly` + `actlimited`**: With both flags, the force uses the
   clamped predicted activation (integrated + clamped before force computation).
8. ✅ **MuJoCo reference**: Compare 2000-step activation trajectory for an
   integrator actuator with `actlimited="true" actrange="-1 1"` against
   analytical expectation. Tolerance: `1e-12` (exact match for Euler + clamping).
9. ✅ **~~Default class inheritance~~** Done (Batch 1 defaults refactor).

#### Implementation Notes

**A+ conformance verified** by dual audit:
- MuJoCo conformance: 13/13 checklist items PASS.
- Code quality: 9/10 PASS, 1 minor WARN (stale doc, fixed).

**Key design decisions:**
- `mj_next_activation()` is a standalone helper factored to serve both `actearly`
  force prediction and the actual integration step. This avoids duplication and
  guarantees consistent behavior.
- RK4 integration cannot use `mj_next_activation()` directly (RK4 uses weighted
  combinations of act_dot across 4 stages), so RK4 applies `actlimited` clamping
  separately after integration.
- `autolimits` compiler flag: `actlimited` is auto-inferred from `actrange`
  presence (same pattern as `ctrllimited`/`forcelimited`).

#### Files

- ~~`sim/L0/mjcf/src/model_builder.rs` — parse actlimited/actrange from MJCF~~ Done
- ~~`sim/L0/mjcf/src/model_builder.rs` — forward parsed values to Model; muscle
  default actrange setup~~ Done
- ~~`sim/L0/core/src/mujoco_pipeline.rs` — add Model fields; implement
  `mj_next_activation()`; wire `actearly` in force computation; replace
  muscle-only clamping in integrator with `mj_next_activation()` call~~ Done
- `sim/L0/tests/integration/activation_clamping.rs` — 15 acceptance tests

---

### 35. `gravcomp` — Body Gravity Compensation ✅
**Status:** Complete (A+ conformance) | **Effort:** S–M | **Prerequisites:** None

#### Current State

**Fully implemented with A+ MuJoCo conformance.** Per-body gravity compensation
as a passive force, matching MuJoCo's `mj_gravcomp()` from `engine_passive.c`.
Fourteen integration tests verify the implementation (`gravcomp.rs`).

The implementation:
1. Parses `gravcomp` from `<body>` as `Option<f64>`, stores `body_gravcomp: Vec<f64>`
   and `ngravcomp: usize` on Model (via model builder).
2. Dedicated `qfrc_gravcomp: DVector<f64>` on Data, zeroed each step.
3. `mj_gravcomp()` iterates bodies with nonzero `body_gravcomp`, computes
   `force = -gravity * (mass * gc)` at `xipos` (body CoM), and projects via
   `mj_apply_ft()` (Jacobian-transpose chain-walk) to `qfrc_gravcomp`.
4. `mj_apply_ft()` is a general utility (not gravcomp-specific): walks from body
   to root via `body_parent`, dispatches on joint type (Hinge/Slide/Ball/Free),
   using body-frame axes for Ball/Free rotational DOFs (cdof convention).
5. Guard: `ngravcomp == 0 || gravity.norm() == 0.0` (fast exit).
6. Sleep filtering: bodies in sleeping trees are skipped via `tree_awake`.
7. Routing: `qfrc_gravcomp` added to `qfrc_passive` at end of `mj_fwd_passive()`.
   Future `jnt_actgravcomp` routing to `qfrc_actuator` is documented as TODO.
8. `ngravcomp` intentionally uses `!= 0.0` (not MuJoCo's `> 0`) to handle negative values.

#### Objective

Parse `gravcomp` from `<body>`, store on Model, compute an anti-gravity force
per body, and add it to `qfrc_passive` during the passive force computation.

#### MuJoCo Reference (`mj_gravcomp` in `engine_passive.c`)

MuJoCo implements gravcomp as a **passive force**, NOT a modification to
`qfrc_bias` or `mj_rne`. The function `mj_gravcomp()` is called from
`mj_passive()`:

```c
static int mj_gravcomp(const mjModel* m, mjData* d) {
    if (!m->ngravcomp || mjDISABLED(mjDSBL_GRAVITY) || mju_norm3(m->opt.gravity) == 0)
        return 0;

    // Sleep filtering
    int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;
    int nbody = sleep_filter ? d->nbody_awake : m->nbody;

    for (int b = 1; b < nbody; b++) {
        int i = sleep_filter ? d->body_awake_ind[b] : b;
        if (m->body_gravcomp[i]) {    // any non-zero value (including negative)
            has_gravcomp = 1;
            mji_scl3(force, m->opt.gravity, -(m->body_mass[i] * m->body_gravcomp[i]));
            mj_applyFT(m, d, force, zero_torque, d->xipos + 3*i, i, d->qfrc_gravcomp);
        }
    }
    return has_gravcomp;
}
```

**Critical details:**
- Uses **`body_mass[i]`** (individual body mass), NOT `body_subtreemass`.
- Result goes to **`qfrc_gravcomp`** (a dedicated nv-dimensional array), which
  is then added to `qfrc_passive` or `qfrc_actuator` depending on
  `jnt_actgravcomp`.
- The force is applied at **`xipos[b]`** (body center of mass in world frame),
  NOT `xpos[b]` (body frame origin). `xipos = xpos + xquat * ipos`.
- `mj_applyFT()` projects the Cartesian force through the kinematic chain
  Jacobian to generalized forces (`qfrc += J_p^T * force + J_r^T * torque`).
- Guard checks: (1) `ngravcomp == 0`, (2) gravity disabled, (3) gravity zero.
- World body (b=0) is skipped.
- **Sleep filtering**: bodies in sleeping trees are skipped for efficiency.
- The check is `body_gravcomp[i]` (any non-zero, not just positive). Negative
  values amplify gravity; values > 1.0 over-compensate (body accelerates up).

**MuJoCo `mj_passive()` orchestration order:**
1. Zero all sub-arrays: `qfrc_spring`, `qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive`
2. Compute spring/damper → `qfrc_spring`, `qfrc_damper`
3. Compute gravcomp → `qfrc_gravcomp`
4. Compute fluid forces → `qfrc_fluid`
5. Accumulate: `qfrc_passive += qfrc_spring + qfrc_damper`
6. Conditionally add gravcomp per-DOF (based on `jnt_actgravcomp`)
7. Add fluid forces

#### Specification

##### S1. MJCF parsing

Parse `gravcomp` (float, default 0.0) from `<body>` elements.

In `types.rs`, add to `MjcfBody`:
```rust
pub gravcomp: Option<f64>,
```

In `parser.rs`, add to `parse_body_attrs()`:
```rust
"gravcomp" => { body.gravcomp = Some(parse_f64(value)?); }
```

MuJoCo allows any real value. No range clamping. Typical values:
- `0` — no compensation (default)
- `1` — full compensation (weightless)
- `0.5` — partial (half gravity)
- `2` — over-compensation (accelerates upward)
- `7.2` — buoyancy simulation (air/helium density ratio)

**Note:** `gravcomp` is a body attribute. It does NOT participate in MJCF
`<default>` class inheritance (MuJoCo defaults only apply to sub-elements
like joint, geom, etc., not to body-level attributes). No defaults handling
needed.

##### S2. Model storage

Add to `Model`:
```rust
pub body_gravcomp: Vec<f64>,   // per-body, default 0.0 (length nbody)
pub ngravcomp: usize,          // count of bodies with gravcomp != 0 (early-exit)
```

Add to `Data`:
```rust
pub qfrc_gravcomp: DVector<f64>,  // nv-dimensional, zeroed each step
```

Forward from `MjcfBody.gravcomp` in `model_builder.rs`. Compute `ngravcomp`
as the count of bodies where `body_gravcomp[b] != 0.0` for early-exit
optimization. World body (index 0) always has `gravcomp = 0.0`.

**`Model::empty()` initialization:**
```rust
body_gravcomp: vec![0.0],  // World body has no gravcomp
ngravcomp: 0,
```

**`make_data()` initialization:**
```rust
qfrc_gravcomp: DVector::zeros(self.nv),
```

Ensure `Data::clone()` includes `qfrc_gravcomp` (the derive macro handles this
if the field is declared in the struct).

**`ngravcomp` divergence note:** MuJoCo's `engine_setconst.c` counts
`body_gravcomp[i] > 0` (positive only). Our spec intentionally counts
`!= 0.0` (any non-zero), so models with only negative gravcomp values still
work. In MuJoCo, negative-only gravcomp models silently no-op due to the
`> 0` guard — this appears to be a latent MuJoCo bug, not intentional design.

**Note:** MuJoCo writes gravcomp forces to a **dedicated `qfrc_gravcomp`** array,
NOT directly to `qfrc_passive`. This is because the routing depends on
`jnt_actgravcomp` — see S4.

##### S3. Runtime: `mj_gravcomp()` as passive force

Implement `mj_gravcomp()` as a function called from `mj_fwd_passive()`:

```rust
fn mj_gravcomp(model: &Model, data: &mut Data) -> bool {
    // Guard: no bodies with gravcomp, or gravity is zero
    if model.ngravcomp == 0 || model.gravity.norm() == 0.0 {
        return false;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let mut has_gravcomp = false;

    for b in 1..model.nbody {
        let gc = model.body_gravcomp[b];
        if gc == 0.0 {
            continue;
        }
        // Sleep filtering: skip bodies in sleeping trees
        if sleep_enabled && !data.tree_awake[model.body_treeid[b]] {
            continue;
        }
        has_gravcomp = true;
        // Anti-gravity force at body CoM (xipos, NOT xpos)
        let force = -model.gravity * model.body_mass[b] * gc;
        // Project through Jacobian to generalized forces
        mj_apply_ft(model, data, &force, &Vector3::zeros(), &data.xipos[b], b,
                     &mut data.qfrc_gravcomp);
    }

    has_gravcomp
}
```

**Critical implementation notes:**

1. Uses **`data.xipos[b]`** (body center of mass in world frame), NOT
   `data.xpos[b]` (body frame origin). `xipos = xpos + xquat * ipos`.
   Gravity acts at the CoM. Using `xpos` would produce wrong torques for
   bodies whose CoM doesn't coincide with their frame origin.

2. The skip condition is `gc == 0.0` (not `gc > 0.0`). Negative gravcomp
   values amplify gravity — MuJoCo processes any non-zero value.

3. `qfrc_gravcomp` is zeroed at the **top of `mj_fwd_passive()`** —
   immediately after `data.qfrc_passive.fill(0.0)` (line 11460). The
   `mj_gravcomp()` call and routing go **after** all existing passive force
   blocks (after flex bending forces, around line 11729). This matches
   MuJoCo's `mj_passive()` which zeros all sub-arrays upfront, then
   computes spring/damper → gravcomp → fluid → accumulates.

4. **Gravity disable flag**: MuJoCo checks `mjDISABLED(mjDSBL_GRAVITY)` in
   addition to `gravity.norm() == 0.0`. Our codebase does not yet define
   `DISABLE_GRAVITY` (only `DISABLE_ISLAND` exists at line 835). The
   `gravity.norm() == 0.0` check suffices because `<option gravity="0 0 0"/>`
   is the only way to disable gravity in our pipeline. When/if `DISABLE_GRAVITY`
   is added, update this guard.

5. **Zero-mass bodies**: `force = -gravity * body_mass * gc` produces zero
   for massless bodies regardless of `gc`. This is correct — gravcomp is a
   no-op for frame bodies (mass=0). No special handling needed.

##### S3b. `mj_apply_ft()` — Cartesian wrench to generalized forces

General-purpose utility matching MuJoCo's `mj_applyFT()`. Projects a Cartesian
force + torque at a world-frame point on a body into generalized forces via the
Jacobian transpose: `qfrc += J_p^T * force + J_r^T * torque`.

MuJoCo's `mj_applyFT` internally calls `mj_jac()` to build the full `jacp`
(3×nv) and `jacr` (3×nv) matrices, then multiplies. Our implementation avoids
materializing the full Jacobian by walking the kinematic chain and accumulating
per-DOF, matching the pattern of `accumulate_point_jacobian()` (line 9644).

```rust
fn mj_apply_ft(
    model: &Model, data: &Data,
    force: &Vector3<f64>, torque: &Vector3<f64>,
    point: &Vector3<f64>, body_id: usize,
    qfrc: &mut DVector<f64>,
) {
    if body_id == 0 { return; }
    let mut current = body_id;
    while current != 0 {
        let jnt_start = model.body_jnt_adr[current];
        let jnt_end = jnt_start + model.body_jnt_num[current];
        for jnt_id in jnt_start..jnt_end {
            let dof = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let anchor = data.xpos[jnt_body]
                               + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    qfrc[dof] += axis.cross(&r).dot(force) + axis.dot(torque);
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    qfrc[dof] += axis.dot(force);
                }
                MjJointType::Ball => {
                    let anchor = data.xpos[jnt_body]
                               + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    // Body-frame axes (matching MuJoCo's cdof convention for
                    // ball joints, same as accumulate_point_jacobian)
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        qfrc[dof + i] += omega.cross(&r).dot(force)
                                        + omega.dot(torque);
                    }
                }
                MjJointType::Free => {
                    // Translation DOFs (0,1,2): world-frame x,y,z
                    qfrc[dof]     += force[0];
                    qfrc[dof + 1] += force[1];
                    qfrc[dof + 2] += force[2];
                    // Rotation DOFs (3,4,5): body-frame axes (cdof convention)
                    let anchor = data.xpos[jnt_body];
                    let r = point - anchor;
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        qfrc[dof + 3 + i] += omega.cross(&r).dot(force)
                                             + omega.dot(torque);
                    }
                }
            }
        }
        current = model.body_parent[current];
    }
}
```

**Axis convention:** Ball and Free rotational DOFs use **body-frame axes**
(`rot * e_i`), matching MuJoCo's `cdof` layout and our
`accumulate_point_jacobian()` (line 9644). This differs from
`add_body_point_jacobian_row()` (line 18178) which uses world-frame axes for
Free joints (a known pre-existing inconsistency noted at line 9691).

**Design note:** This is a general-purpose utility — not gravcomp-specific.
Place near the other Jacobian utilities (`accumulate_point_jacobian`,
`mj_jac_site`). Future uses include `xfrc_applied` projection.

##### S4. Routing: `qfrc_gravcomp` → `qfrc_passive`

After `mj_gravcomp()` returns, add the gravcomp forces to `qfrc_passive`. In
MuJoCo, this routing is conditional on `jnt_actgravcomp` — but since we don't
yet support `actgravcomp`, we unconditionally add all gravcomp to passive:

```rust
// In mj_fwd_passive(), after joint/tendon/flex passive forces:
let has_gc = mj_gravcomp(model, data);
if has_gc {
    // Unconditional routing to qfrc_passive.
    // TODO(future): when jnt_actgravcomp is implemented, only add for DOFs
    // where jnt_actgravcomp[dof_jntid[dof]] == false. DOFs with
    // actgravcomp=true route through qfrc_actuator instead.
    data.qfrc_passive += &data.qfrc_gravcomp;
}
```

**MuJoCo routing detail:** When `jnt_actgravcomp="true"` on a joint, that
joint's DOFs' gravcomp forces go to `qfrc_actuator` (not `qfrc_passive`). This
makes the gravcomp load subject to the joint's `actfrcrange` clamping.
The dedicated `qfrc_gravcomp` array makes this future change trivial.

##### S5. Where NOT to implement

Do NOT modify `mj_rne()` or `qfrc_bias`. MuJoCo computes gravcomp as a
**separate passive force** added to `qfrc_passive` (via `qfrc_gravcomp`), not
as a bias force adjustment. Gravcomp counteracts the gravity portion of
`qfrc_bias` through the equations of motion:
`M * qacc = qfrc_passive + qfrc_actuator - qfrc_bias + qfrc_constraint`.

#### Acceptance Criteria

1. **Full compensation (free body)**: Free body with `gravcomp="1"`, no
   contact → `qfrc_gravcomp` exactly cancels gravity component of `qfrc_bias`.
   Verify: `qacc` gravitational component ≈ 0 (tolerance 1e-12).
2. **No compensation**: `gravcomp="0"` (or absent) → `qfrc_gravcomp` is all
   zeros. Verify: identical to model without gravcomp attribute.
3. **Partial compensation**: `gravcomp="0.5"` → `qfrc_gravcomp` is exactly
   half the full-compensation value. Verify ratio at 1e-12.
4. **Over-compensation**: `gravcomp="2"` → `qacc` reverses sign (body
   accelerates upward). Verify sign of vertical qacc.
5. **Parsing**: `gravcomp="0"` and `gravcomp="7.2"` parse without error.
   Apptronik Apollo asset loads cleanly.
6. **Kinematic chain**: 3-link arm with `gravcomp="1"` on all links →
   `qfrc_passive` contains antigravity forces, arm holds static equilibrium
   against gravity (zero qacc at rest). Tolerance 1e-10.
7. **Gravity disabled**: `<option gravity="0 0 0"/>` with `gravcomp="1"` →
   `qfrc_gravcomp` is all zeros.
8. **CoM offset**: Body with `inertial pos="0 0 0.1"` (non-trivial ipos) and
   `gravcomp="1"` on a hinge → verify torque uses `xipos` (not `xpos`).
   Compare against analytical `J^T * (-mass * g)` at `xipos`. Tolerance 1e-12.
9. **`qfrc_gravcomp` isolation**: After `forward()`, `qfrc_gravcomp` is
   nonzero for affected DOFs, zero for unaffected. Verify the dedicated array
   is populated independently of `qfrc_passive`.
10. **Analytical free-body verification**: Free body, `gravcomp="0.7"`,
    mass=2.0, gravity=[0,0,-9.81] → verify analytically:
    `qfrc_gravcomp = [0, 0, +0.7*2.0*9.81, 0, 0, 0]` (force cancels 70% of
    gravity on translation DOFs, zero on rotation DOFs since force at CoM
    produces no torque about the free joint origin when ipos=0). Tolerance 1e-12.
11. **Sleep filtering**: Body in a sleeping tree with `gravcomp="1"` →
    no contribution to `qfrc_gravcomp`. Verify array stays zero for sleeping
    tree's DOFs.
12. **Negative gravcomp**: Free body with `gravcomp="-1"` → gravity force
    is doubled (amplified, not compensated). Verify `qfrc_gravcomp` has same
    sign as gravity (pushes body downward), magnitude equals `mass * g`.
    Tolerance 1e-12.
13. **Selective sub-chain**: Parent body `gravcomp="0"`, child body
    `gravcomp="1"` → only the child's mass is compensated, but the force
    projects through both the child's and parent's joints. Verify
    `qfrc_gravcomp` is nonzero on parent's DOFs (child's gravity force
    propagates up the chain).
14. **Free body with CoM offset**: Free body with `gravcomp="1"` and
    `inertial pos="0.1 0 0"` (non-zero ipos) → gravcomp force at xipos
    produces torque on the free joint's rotational DOFs. Verify rotational
    DOFs of `qfrc_gravcomp` are nonzero (unlike AC10 where ipos=0).
    Tolerance 1e-12.

#### Files

- `sim/L0/mjcf/src/types.rs` — add `gravcomp: Option<f64>` to `MjcfBody`
- `sim/L0/mjcf/src/parser.rs` — parse `gravcomp` in `parse_body_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — forward to `Model.body_gravcomp`,
  compute `ngravcomp`
- `sim/L0/core/src/mujoco_pipeline.rs` — add `body_gravcomp: Vec<f64>` and
  `ngravcomp: usize` to Model; add `qfrc_gravcomp: DVector<f64>` to Data;
  implement `mj_apply_ft()` (general utility near Jacobian functions) and
  `mj_gravcomp()`; call from `mj_fwd_passive()` with routing to `qfrc_passive`;
  zero `qfrc_gravcomp` at top of `mj_fwd_passive()`
- `sim/L0/tests/integration/` — new `gravcomp.rs` with tests for AC1-AC14

---

### 36. Body-Transmission Actuators (Adhesion) ✅
**Status:** Complete (A+ conformance) | **Effort:** M | **Prerequisites:** None

#### Current State

**Fully implemented with A+ MuJoCo conformance.** Body transmission
(`ActuatorTransmission::Body`) for adhesion actuators, matching MuJoCo's
`mjTRN_BODY` in `engine_core_smooth.c`.
Thirteen integration tests verify the implementation (`adhesion.rs`, AC1–AC14;
AC12 documented skip — flex contacts not yet wired).

The implementation:
1. `ActuatorTransmission::Body` enum variant added.
2. `compute_contact_normal_jacobian()` — computes `J(b2) - J(b1)` via
   `accumulate_point_jacobian()` (matching `mj_jacDifPair` sign convention).
3. `mj_transmission_body()` — iterates contacts, accumulates normal Jacobians
   for contacts involving the target body, applies negated average (`*= -1.0 / count`),
   sets `actuator_length = 0`.
4. `mj_transmission_body_dispatch()` — thin loop over `0..model.nu` filtering
   for `Body` transmission.
5. Pipeline insertion in `forward_core()` after `mj_collision()` and equality
   wake cycle, before `mj_sensor_pos()`.
6. Phase 3 force application via `qfrc += moment * force` (merged with Site arm).
7. Model builder: body name → ID resolution replacing former `ModelConversionError`.
8. Body arm added to all match sites: `mj_actuator_length`, `compute_muscle_params`
   (Phase 1 & 2), `mj_sensor_pos` (ActuatorPos), derivatives (`derivatives.rs`),
   and sleep policy resolution.

**Files:** `mujoco_pipeline.rs` (7 changes), `model_builder.rs` (2 changes),
`derivatives.rs` (1 change).
- Body case in `mj_actuator_length()` for velocity computation.

**Existing transmission patterns** (for reference):
- **Joint**: `qfrc_actuator[dof] += gear * force` (direct DOF application).
  Gear baked into moment: `moment[dof] = gear[0]`.
- **Tendon**: `apply_tendon_force()` with cached `ten_J` Jacobian.
  Gear baked into length/velocity: `length = gear * ten_length`.
- **Site**: `qfrc_actuator[dof] += actuator_moment[i][dof] * force` (pre-computed
  moment Jacobian from `mj_transmission_site()`). Gear baked into moment via
  6D wrench projection.

Body transmission follows the **Site pattern** — pre-compute a moment arm vector
in a transmission function, then apply it in Phase 3. Unlike other transmission
types, body transmission does **not** apply gear to the moment (see MuJoCo
Reference below).

#### Objective

Implement body transmission (`ActuatorTransmission::Body`) so adhesion actuators
load and function. The adhesion force pulls/pushes the target body toward/away from
surfaces via contact normal Jacobians.

#### MuJoCo Reference (`mjTRN_BODY` in `engine_core_smooth.c`)

MuJoCo's body transmission in `mj_transmission()` iterates all contacts and
separates them into two categories based on `con->exclude`:

1. **Active contacts** (`con->exclude == 0`): Weighted into a temporary
   `efc_force` vector. Weights depend on cone type:
   - Condim 1 (frictionless) or elliptic: `efc_force[normal_row] = 1.0`.
   - Pyramidal: `efc_force[edge_k] = 0.5 / npyramid` for each of `2*npyramid`
     edges. The symmetric weighting ensures the pyramid edges average back to the
     normal Jacobian (tangential components cancel).
   The accumulated result is extracted via `mj_mulJacTVec(efc_force)`, which
   computes `J^T * efc_force` over the constraint Jacobian.

2. **Excluded-in-gap contacts** (`con->exclude == 1`): Contacts filtered by
   `<exclude>` but still detected because they're within the gap margin.
   Since these have no `efc_J` rows, the Jacobian is computed directly via
   `mj_jacDifPair()` at the contact point, projected along the contact normal
   (`con->frame[0..3]`). Accumulated into a separate `moment_exclude` buffer.

3. **Fully excluded** (`con->exclude > 1`): Skipped entirely.

Both active and excluded-in-gap categories are counted, and the final moment is:
```
moment = -(1/counter) * (J_active + J_exclude)
```

The negative sign makes positive ctrl produce an **attractive** force.

**Gear is NOT applied for body transmission.** Unlike Joint (where
`moment[dof] = gear[0]`), Tendon (where `length *= gear[0]`), and Site (where
the 6D gear vector is projected through the wrench), the `mjTRN_BODY` case in
`mj_transmission()` does NOT multiply gear into the moment or length. The
`-1.0/counter` scaling is the only post-processing. Force magnitude is
controlled entirely by `gainprm[0]` (the `gain` attribute on `<adhesion>`),
not by `gear`. The default `gear[0] = 1` has no effect.

**Contact filtering:**
- Skip flex contacts (`geom1 < 0 || geom2 < 0`, i.e., flex vertex contacts)
- Skip contacts not involving the target body
- `exclude == 0`: active contact → weight into efc_force
- `exclude == 1`: excluded but in gap → compute Jacobian directly
- `exclude > 1`: skip entirely

**Edge cases:**
- Zero contacts: moment is zero, actuator produces no force.
- `length[i] = 0` always (adhesion has no length concept).

#### Specification

##### S1. `ActuatorTransmission::Body` enum variant

Add a new variant to the transmission enum (line ~527 in `mujoco_pipeline.rs`).
Since the body ID is already stored in `actuator_trnid`, we do NOT need to embed
it in the enum:

```rust
pub enum ActuatorTransmission {
    #[default]
    Joint,
    Tendon,
    Site,
    Body,
}
```

The target body ID is stored in `model.actuator_trnid[i][0]`, same as other
transmission types.

##### S2. `mj_transmission_body()` — moment arm computation

Compute the adhesion moment arm as the negated average of contact normal
Jacobians for all contacts involving the target body.

**Why direct Jacobian, not `efc_J`?** MuJoCo has `efc_J` available during
`mj_transmission()` because it assembles constraint rows earlier in its
pipeline. In our pipeline, `mj_transmission_body()` runs **after**
`mj_collision()` (which populates `data.contacts`) but **before** constraint
assembly (which builds `efc_J`). We therefore use direct Jacobian computation
for ALL contacts via `accumulate_point_jacobian()`. This produces identical
results because:
- For active contacts, MuJoCo's `efc_force` weighting reconstructs the normal
  Jacobian (see MuJoCo Reference above).
- For excluded-in-gap contacts, MuJoCo already uses direct Jacobian computation.
- Our collision pipeline filters `<exclude>` body pairs entirely out of
  `data.contacts` (in `check_collision_affinity()`, line ~5384), so
  `data.contacts` never contains `exclude > 0` contacts. All contacts in
  `data.contacts` are active. This means MuJoCo's two-path approach collapses
  to our single-path approach for the supported feature set.

```rust
fn mj_transmission_body(
    model: &Model,
    data: &mut Data,
    actuator_id: usize,
) {
    let body_id = model.actuator_trnid[actuator_id][0];
    let moment = &mut data.actuator_moment[actuator_id];
    moment.fill(0.0);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];

        // Skip flex contacts (MuJoCo: geom < 0)
        if contact.flex_vertex.is_some() { continue; }

        // Skip contacts not involving target body
        let b1 = model.geom_body[contact.geom1];
        let b2 = model.geom_body[contact.geom2];
        if b1 != body_id && b2 != body_id { continue; }

        // Compute normal^T · (J(b2, pos) - J(b1, pos))
        let j_normal = compute_contact_normal_jacobian(model, data, contact);
        *moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        // Negate and average. NO gear scaling (MuJoCo omits gear for body
        // transmission — force magnitude is controlled by gainprm, not gear).
        *moment *= -1.0 / (count as f64);
    }

    // Body transmission has no length concept
    data.actuator_length[actuator_id] = 0.0;
}
```

**No gear in moment.** MuJoCo's `mjTRN_BODY` case scales by `-1.0/counter`
only — no `gear[0]` multiplication. This differs from Joint (`moment = gear`),
Tendon (`length *= gear`), and Site (6D wrench projection includes gear).
The adhesion shortcut (`mjs_setToAdhesion`) does not set gear; force magnitude
is controlled by `gainprm[0]` (the `gain` attribute), not gear. The `gear`
attribute is accepted by the parser (it's a common actuator attribute) but has
no effect on body transmission.

##### S2b. `compute_contact_normal_jacobian()` helper

Computes the contact normal Jacobian difference: `n^T · (J_p(b2, pos) - J_p(b1, pos))`,
where `J_p` is the positional Jacobian at the contact point, `b1` is the body
of `geom1`, and `b2` is the body of `geom2`. This is a 1×nv row vector stored
as a DVector.

**Sign convention:** MuJoCo's `mj_jacDifPair(b1, b2)` computes
`J(b2) - J(b1)` (second argument minus first). For contacts, it is called as
`mj_jacDifPair(bid[0], bid[1])` where `bid[0]` = body of `geom[0]` and
`bid[1]` = body of `geom[1]`, producing `J(geom1_body) - J(geom0_body)`.
The contact normal points from `geom[0]` toward `geom[1]`. This sign convention
ensures that the subsequent `-1/counter` negation in body transmission produces
an **attractive** force (toward the contact surface).

Our tendon Jacobian code follows the same convention:
`accumulate_point_jacobian(body1, +1.0)` and
`accumulate_point_jacobian(body0, -1.0)` producing `J(body1) - J(body0)`.
For body transmission we follow the same pattern.

```rust
fn compute_contact_normal_jacobian(
    model: &Model,
    data: &Data,
    contact: &Contact,
) -> DVector<f64> {
    let nv = model.nv;
    let b1 = model.geom_body[contact.geom1];
    let b2 = model.geom_body[contact.geom2];
    let point = &contact.pos;
    let normal = &contact.normal;

    let mut j_normal = DVector::zeros(nv);

    // Walk kinematic chain for body2 (+1 contribution)
    // MuJoCo convention: jacdifp = J(b2) - J(b1)
    accumulate_point_jacobian(
        model, &data.xpos, &data.xquat,
        &mut j_normal, b2, point, normal, 1.0,
    );
    // Walk kinematic chain for body1 (-1 contribution)
    accumulate_point_jacobian(
        model, &data.xpos, &data.xquat,
        &mut j_normal, b1, point, normal, -1.0,
    );

    j_normal
}
```

`accumulate_point_jacobian()` (line ~9656 in `mujoco_pipeline.rs`) is an
existing function used for tendon Jacobian computation. It walks the kinematic
chain from `body_id` to root, accumulating
`scale * (direction · joint_contribution)` per DOF. For each joint type:
- **Hinge**: `scale * axis.cross(point - anchor).dot(direction)`
- **Slide**: `scale * axis.dot(direction)`
- **Ball/Free rotational**: `scale * (body_frame_axis.cross(point - anchor)).dot(direction)`
  (body-frame axes, matching `cdof` convention)
- **Free translational**: `scale * e_i.dot(direction)` (world-frame axes)

This correctly computes `n^T · J_p` because the rotational-to-translational
coupling at the contact point is included (the cross product `omega × r` gives
the velocity contribution from rotation). Matches MuJoCo's `mj_jacDifPair` +
`mju_mulMatMat(jac, con->frame, jacdifp, 1, 3, NV)` for the excluded contact
path, and equivalently reconstructs the normal Jacobian that MuJoCo's weighted
`efc_force` approach extracts for active contacts.

**Sign trace (sphere on horizontal plane, verification):**
- `geom[0]` = plane (world body 0), `geom[1]` = sphere (body B, free joint)
- Contact normal = `[0, 0, +1]` (upward, from plane toward sphere)
- `j_normal = n^T · (J(sphere) - J(world))` = `[0,0,+1]^T · ([0,0,1,...] - 0)` = `+1` on sphere z DOF
- `moment = -(1/1) * (+1) = -1` on sphere z DOF
- `qfrc_actuator[z] = -1 * (gain * ctrl) = -100` (downward = attractive toward plane)
- Correct: adhesion pulls sphere toward the surface.

##### S3. Call site in forward pipeline

**Critical ordering:** `mj_transmission_body()` requires contacts from
`mj_collision()`, so it CANNOT run alongside `mj_transmission_site()` (which
runs before collision at line ~4632). It must run **after** `mj_collision()`
and after the wake-on-contact re-collision cycle.

In `forward_core()` (line ~4610), add the body transmission call after the
collision + wake cycle (after line ~4651) and before `mj_sensor_pos()`:

```rust
// ===== Position Stage (continued) =====
mj_transmission_site(model, self);           // line 4632 (existing)

// ... tendon wake, collision, wake-on-contact, equality wake ...

// Body transmission — requires contacts from mj_collision()
mj_transmission_body_dispatch(model, self);  // NEW: after line 4651

if compute_sensors {
    mj_sensor_pos(model, self);              // existing
}
```

Where `mj_transmission_body_dispatch` iterates body-transmission actuators:

```rust
fn mj_transmission_body_dispatch(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] == ActuatorTransmission::Body {
            mj_transmission_body(model, data, i);
        }
    }
}
```

**Note:** `mj_transmission_site()` stays at its current location (line 4632)
because it only needs FK data, not contacts. Only body transmission moves after
collision.

##### S4. Force application in Phase 3

In `mj_fwd_actuation()` Phase 3 (line ~10726), add the Body case to the
`match model.actuator_trntype[i]` block, identical to the existing Site case:

```rust
ActuatorTransmission::Body => {
    // Same as Site: moment is pre-computed by mj_transmission_body()
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```

##### S5. Velocity computation in `mj_actuator_length()`

In `mj_actuator_length()` (line ~10439), add the Body case to the
`match model.actuator_trntype[i]` block. Body transmission has no length
(always 0, set by `mj_transmission_body`), and velocity is moment · qvel:

```rust
ActuatorTransmission::Body => {
    // Length already set to 0 by mj_transmission_body (position stage).
    // Velocity from cached moment (same as Site):
    data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
}
```

##### S6. Model builder: remove error, resolve body name to ID

In `model_builder.rs` (line ~2019), replace the error with body name resolution:

```rust
} else if let Some(ref body_name) = actuator.body {
    let body_id = *self.body_name_to_id.get(body_name.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!("Actuator '{}': body '{}' not found", actuator.name, body_name),
        })?;
    transmission = ActuatorTransmission::Body;
    trnid[0] = body_id;
}
```

##### S7. Contact filtering edge cases

- **No contacts**: `moment` stays zero, `force * 0 = 0`. Actuator is inert.
- **Multiple contacts**: Averaged by `count`. Prevents adhesion magnitude from
  scaling with contact count (e.g., 4 box-on-plane contacts produce the same
  moment as 1 sphere-on-plane contact, modulo Jacobian differences).
- **Flex contacts**: Skipped via `contact.flex_vertex.is_some()`. Matches
  MuJoCo's `geom < 0` filter (flex contacts store a vertex index instead of
  a geom ID).
- **World body (body 0) as target**: `accumulate_point_jacobian()` returns
  immediately for `body_id == 0` (no DOFs). The other contact body's Jacobian
  is subtracted from zero, producing a valid moment. No special case needed.
- **World body as contact partner**: When one contact body is body 0 (e.g.,
  floor), its chain walk contributes zero. Only the movable body's DOFs appear
  in the result. Correct, no special case.
- **`<exclude>` body pairs**: Our collision pipeline filters `<exclude>` pairs
  entirely out of `data.contacts` in `check_collision_affinity()` (line ~5384).
  `data.contacts` never contains excluded contacts, so the MuJoCo three-way
  `con->exclude` dispatch (0 = active, 1 = excluded-in-gap, >1 = skip) is
  unnecessary in our implementation. All contacts in `data.contacts` are active.
  When `<exclude>` is extended in the future to support gap-based inclusion,
  the direct Jacobian approach handles it naturally (no code change needed —
  any contact in `data.contacts` gets the same treatment).
- **Pyramidal vs elliptic equivalence**: Our direct normal Jacobian computation
  is equivalent to MuJoCo's `efc_force` weighting because:
  - **Elliptic/frictionless**: MuJoCo sets `efc_force[normal_row] = 1.0`, then
    `J^T * f` extracts exactly the normal Jacobian row. Same as our direct
    computation.
  - **Pyramidal**: MuJoCo sets each of `2*(dim-1)` edges to `0.5/(dim-1)`.
    Pyramid edges are `J_n ± μ*J_t` — the symmetric weighting causes
    tangential components to cancel, leaving `J_n`. Mathematically equivalent.
  Both approaches yield the same nv-dimensional moment vector per contact.

#### Acceptance Criteria

1. **Loading**: `<actuator><adhesion body="box" gain="100"/>` loads without error
   for a model with a body named "box". Body name resolves to correct body ID.
2. **Zero force when no contact**: Adhesion actuator with `ctrl=1.0` but the
   target body not in contact → `actuator_moment[i]` is all zeros,
   `qfrc_actuator` contribution from this actuator is zero.
3. **Attractive force sign**: A sphere resting on a plane with adhesion
   `ctrl=1.0` → `qfrc_actuator` pushes the sphere toward the plane (downward
   for a sphere above a horizontal plane). Verify the sign is attractive:
   the vertical component of `qfrc_actuator` has the same sign as gravity
   (both pull the sphere toward the surface).
4. **Force magnitude (single contact)**: For a sphere (free joint, mass=1) on
   a horizontal plane with a single contact, adhesion `gain=100`, `ctrl=1.0`:
   - Scalar force: `gain * ctrl = 100`
   - `J_normal = n^T · (J(sphere) - J(world))`: normal = `[0,0,+1]` (upward),
     `J(sphere)` z DOF = `+1`, `J(world)` = 0, so `J_normal = [0, 0, +1, 0, 0, 0]`.
   - Moment: `-(1/1) * J_normal = [0, 0, -1, 0, 0, 0]`.
   - Generalized force: `moment * 100 = [0, 0, -100, 0, 0, 0]`.
   - Verify: negative z = downward = attractive toward plane. Correct sign.
   Verify analytically. Tolerance: `1e-12`.
5. **Multiple contacts**: Box (4 contacts) on a plane with adhesion. Moment
   arm is the average of 4 normal Jacobians. Verify `actuator_moment` is
   `-(1/4) * Σ J_normal_k`. Compare scalar `qfrc_actuator` against analytical
   expectation. Tolerance: `1e-10`.
6. **Gear has no effect**: Adhesion with `gear="2"` produces the **same**
   moment and generalized force as `gear="1"` (default). MuJoCo does not apply
   gear for body transmission. Verify by comparing `actuator_moment` for
   `gear="1"` vs `gear="2"` — they must be identical.
7. **Gain controls magnitude**: Adhesion with `gain="200"` produces 2x the
   scalar force (and thus 2x the generalized force) compared to `gain="100"`,
   for the same `ctrl=1.0` and contacts. Verify ratio.
8. **Regression**: Models without adhesion actuators produce identical results
   to baseline (no new code paths touched for Joint/Tendon/Site actuators).
9. **`actuator_moment` populated**: After `forward()`, `data.actuator_moment[i]`
   for a body-transmission actuator is nonzero when contacts exist (and body is
   involved) and all zeros when no contacts exist.
10. **`actuator_length` is zero**: Body transmission always has
    `actuator_length[i] == 0.0`.
11. **`actuator_velocity` correct**: With contacts and nonzero `qvel`, verify
    `actuator_velocity[i] == actuator_moment[i].dot(qvel)`.
12. **Flex contacts excluded**: A body with both rigid-rigid and flex-rigid
    contacts → only rigid-rigid contacts contribute to the adhesion moment.
    (Test only if flex contacts are wired through collision — otherwise document
    as future verification.)
13. **Kinematic chain correctness**: A hinged body (not free) with adhesion on
    a horizontal plane → `actuator_moment` has exactly one nonzero DOF entry
    (the hinge DOF), and the value equals the projected lever arm. Verify
    against analytical `n^T · (axis × (pos - anchor))` for the hinge joint.
    Tolerance: `1e-12`.
14. **Two-body contact**: Both contact bodies are non-world (e.g., sphere on a
    movable platform). The adhesion target is the sphere. The moment arm
    involves DOFs from both bodies (attraction pulls them together). Verify
    `actuator_moment` is nonzero on both bodies' DOFs.

#### Implementation Notes

**`accumulate_point_jacobian()` reuse.** This function already exists (line ~9656)
and is used for tendon Jacobians. It walks the kinematic chain and projects a
direction vector through each joint's contribution. For body transmission, call
it once per contact body with `direction = contact.normal`, `scale = +1.0` for
body2 (geom2's body) and `scale = -1.0` for body1 (geom1's body). This matches
MuJoCo's `mj_jacDifPair(b1, b2)` convention which computes `J(b2) - J(b1)`,
and matches our tendon code which uses the same `(body1, +1.0), (body0, -1.0)`
pattern. The existing function handles all joint types (Hinge, Slide, Ball,
Free) with correct body-frame axes for rotational DOFs. No modifications needed.

**Pipeline ordering.** The forward pipeline (lines 4610–4694) runs:
1. `mj_fwd_position()` — FK, computes `xpos`, `xquat`
2. `mj_flex()` — flex deformation
3. `mj_transmission_site()` — site moment computation (needs FK only)
4. Tendon wake cycle
5. `mj_collision()` — populates `data.contacts`
6. Wake-on-contact + re-collision cycle
7. Equality wake cycle
8. **`mj_transmission_body_dispatch()` — NEW (needs contacts from step 5)**
9. `mj_sensor_pos()` — reads `actuator_length`
10. `mj_fwd_velocity()` — velocity computation
11. `mj_actuator_length()` — computes `actuator_velocity` (needs moment from step 8)
12. `mj_fwd_actuation()` — Phase 2 force + Phase 3 application

Body transmission must run after step 5 (collision) and before step 9 (sensors
read `actuator_length`). The chosen location (step 8) satisfies both constraints.

**`make_data()` initialization.** `actuator_moment` is already initialized as
`vec![DVector::zeros(self.nv); self.nu]` (line ~3289), so Body-transmission
actuators get a correctly-sized zero vector. No additional initialization needed.
`actuator_length` defaults to 0.0 (Vec<f64> init), also correct.

**Match exhaustiveness.** Adding `Body` to `ActuatorTransmission` requires
updating all `match` expressions on this enum. The compiler will enforce this.
Six match/if-check sites exist:

1. `mj_fwd_actuation()` Phase 3 (line ~10726) — add Body arm (S4: moment-based
   force application, identical to Site arm).
2. `mj_actuator_length()` (line ~10442) — add Body arm (S5: velocity from
   cached moment, length already set to 0).
3. `compute_muscle_params()` Phase 1, lengthrange estimation (line ~3731) — add
   `Body => {}`. Body transmission has no length concept, so lengthrange is
   meaningless. No-op, same reasoning as the existing Site arm.
4. `compute_muscle_params()` Phase 2, J-vector for acc0 (line ~3809) — add
   `Body => {}`. At qpos0 (model setup time) there are no contacts, so the
   J-vector would be zero. Leave `j_vec` as-is (already initialized to zeros).
   Note: muscle + body transmission is an unusual combination but not prevented
   by the parser; the zero J-vector means acc0 = 0 for such actuators, which is
   physically correct (no contacts = no adhesion force = no acceleration).
5. `mj_sensor_pos()` ActuatorPos sensor (line ~8373) — add Body arm that reads
   `data.actuator_length[objid]` (always 0.0), identical to the existing Site
   arm: `Body => { sensor_write(&mut data.sensordata, adr, 0, data.actuator_length[objid]); }`.
6. `mj_transmission_site()` (line ~10329) — uses `if != Site { continue }`,
   so Body actuators are already skipped. No change needed.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - Add `Body` variant to `ActuatorTransmission` enum (line ~527)
  - Implement `mj_transmission_body()` and `compute_contact_normal_jacobian()`
    (new functions, place near `mj_transmission_site()` at line ~10321)
  - Implement `mj_transmission_body_dispatch()` (thin loop wrapper)
  - Add call to `mj_transmission_body_dispatch()` in `forward_core()` after
    the collision + wake cycle (after line ~4651)
  - Add `Body` arm to Phase 3 match in `mj_fwd_actuation()` (line ~10726)
  - Add `Body` arm to `mj_actuator_length()` (line ~10442)
  - Add `Body => {}` to both matches in `compute_muscle_params()` (lines ~3731, ~3809)
  - Add `Body` arm to `mj_sensor_pos()` ActuatorPos sensor (line ~8373)
- `sim/L0/mjcf/src/model_builder.rs`:
  - Replace error (line ~2019) with body name → ID resolution
- `sim/L0/tests/integration/`:
  - New `adhesion.rs` test file with tests for AC1–AC14

---

### 37. Tendon Equality Constraints
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

**Pipeline:** `EqualityType::Tendon` exists in the enum (`mujoco_pipeline.rs:611`)
but is ignored during constraint assembly. Row count returns 0 (line 14553),
extraction returns `continue` (line 14719). Any model using `<equality><tendon>`
silently gets no constraint enforcement.

**MJCF parser:** `MjcfEquality` (`types.rs:1859`) has no `tendons` field. The
`<tendon>` element inside `<equality>` is explicitly skipped (`parser.rs:2132`,
comment: "Skip other equality constraint types (tendon, flex)"). Models with
tendon equality constraints silently drop the element.

**Tendon kinematics:** Fully computed for both fixed and spatial tendons:
- `data.ten_length: Vec<f64>` — current tendon lengths (`mujoco_pipeline.rs:2370`)
- `data.ten_velocity: Vec<f64>` — tendon velocities (`mujoco_pipeline.rs:2373`)
- `data.ten_J: Vec<DVector<f64>>` — tendon Jacobians, nv-dimensional per tendon
  (`mujoco_pipeline.rs:2378`)
- `model.tendon_length0: Vec<f64>` — reference length at qpos0
  (`mujoco_pipeline.rs:1692`), populated for both fixed (line 3669) and spatial
  (line 4018) tendons during model building

**Tendon tree mapping:** Already precomputed during model building
(`model_builder.rs:3757–3808`):
- `model.tendon_treenum: Vec<usize>` — trees spanned per tendon
  (`mujoco_pipeline.rs:1721`)
- `model.tendon_tree: Vec<usize>` — packed tree IDs, indexed as
  `tendon_tree[2*t]` and `tendon_tree[2*t+1]`, sentinel `usize::MAX` for unused
  slots (`mujoco_pipeline.rs:1725`)

**Pre-existing bug in tendon tree builder:** The tree ID computation at
`model_builder.rs:3801` only stores tree IDs when `tree_set.len() == 2`. For
single-tree tendons (`treenum == 1`), `tendon_tree[2*t]` remains at the
initialization value of `usize::MAX`. This causes the existing tendon-limit wake
propagation code (`mujoco_pipeline.rs:12770–12771`) to silently skip single-tree
tendons. This bug must be fixed as part of this work (see S6).

**Existing pattern:** `extract_joint_equality_jacobian()` (`mujoco_pipeline.rs:18294`)
implements the Joint equality case with polynomial coupling. Tendon equality
follows the same pattern but uses tendon lengths and Jacobians instead of joint
positions and DOF columns.

**Name lookup:** `tendon_name_to_id: HashMap<String, usize>` exists on the model
builder (`model_builder.rs:563`) and is populated during tendon processing
(line 1777).

#### Objective

Parse `<equality><tendon>` from MJCF, implement constraint assembly for
`EqualityType::Tendon`, and integrate with the unified constraint solver.

#### MuJoCo Reference (`mjEQ_TENDON` in `engine_core_constraint.c`)

MuJoCo's tendon equality has two modes:

**Two-tendon coupling** (most common):
```c
dif = ten_length[id1] - tendon_length0[id1];   // L2 deviation from reference
cpos = (ten_length[id0] - tendon_length0[id0])  // L1 deviation from reference
     - data[0]                                    // constant offset
     - (data[1]*dif + data[2]*dif^2 + data[3]*dif^3 + data[4]*dif^4);  // polynomial

deriv = data[1] + 2*data[2]*dif + 3*data[3]*dif^2 + 4*data[4]*dif^3;
jac = J_tendon0 + (-deriv) * J_tendon1;
```

The constraint enforces: `(L1 - L1_0) = data[0] + P(L2 - L2_0)`, where:
- `L1`, `L2` = current tendon lengths
- `L1_0`, `L2_0` = reference tendon lengths at qpos0 (`tendon_length0`)
- `data[0]` = constant offset
- `data[1..4]` = polynomial coefficients (degree 1 through 4)
- `P(x) = data[1]*x + data[2]*x^2 + data[3]*x^3 + data[4]*x^4`
- Note: `data[0]` is NOT part of the polynomial — it's a separate offset

**Single-tendon mode** (tendon2 not specified, `id1 < 0`):
```c
cpos = ten_length[id0] - tendon_length0[id0] - data[0];
jac = J_tendon0;
```

This constrains the tendon to its reference length plus offset.

**Default `polycoef`:** MuJoCo defaults to `polycoef="0 1 0 0 0"`, which gives
`data[0]=0, data[1]=1` → constraint: `(L1-L1_0) = (L2-L2_0)`, i.e., both
tendons deviate equally from their reference lengths.

#### Specification

##### S1. MJCF parsing

Add `MjcfTendonEquality` type and parse `<equality><tendon>`.

In `types.rs`, following the `MjcfJointEquality` pattern (`types.rs:1655`):
```rust
/// A tendon equality constraint from `<tendon>` element within `<equality>`.
///
/// Constrains a tendon to its reference length, or couples two tendons
/// with a polynomial relationship: `(L1-L1_0) = data[0] + P(L2-L2_0)`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfTendonEquality {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first (primary) tendon.
    pub tendon1: String,
    /// Name of the second tendon (optional, for coupling).
    pub tendon2: Option<String>,
    /// Polynomial coefficients: `[offset, c1, c2, c3, c4]`.
    /// Default is `[0, 1]` meaning equal deviation from reference (trailing zeros implicit).
    pub polycoef: Vec<f64>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}
```

**Design notes vs. original spec:**
- `polycoef: Vec<f64>` (not `[f64; 5]`) — matches `MjcfJointEquality` pattern.
  Default: `vec![0.0, 1.0]` (matching `MjcfJointEquality:1688`). Builder takes
  up to 5 via `.take(5)` into zero-initialized `[f64; 11]`; trailing zeros are
  implicit.
- `active: bool` (not `Option<bool>`) — all other equality types use `bool`
  with `true` default at parse time (`MjcfConnect:1487`, `MjcfWeld:1579`,
  `MjcfJointEquality:1678`, `MjcfDistance:1791`).
- `class: Option<String>` — present on all other equality types for default
  class inheritance.

Add `tendons` field to `MjcfEquality` (`types.rs:1859`):
```rust
pub struct MjcfEquality {
    pub connects: Vec<MjcfConnect>,
    pub welds: Vec<MjcfWeld>,
    pub joints: Vec<MjcfJointEquality>,
    pub distances: Vec<MjcfDistance>,
    pub tendons: Vec<MjcfTendonEquality>,  // NEW
}
```

In `parser.rs`, update `parse_equality()` (line 2102). The function currently
matches child element names; `<tendon>` falls through to the wildcard `_ =>
skip_element()` (line 2132, with comment "Skip other equality constraint types
(tendon, flex)"). Add `b"tendon"` arms in both the `Event::Start` and
`Event::Empty` branches, calling a new `parse_tendon_equality_attrs()` helper:
- `tendon1` attribute (required)
- `tendon2` attribute (optional)
- `polycoef` attribute (5 floats, default `"0 1 0 0 0"`)
- Standard equality attributes: `name`, `class`, `active`, `solref`, `solimp`

**Parser disambiguation:** The `<tendon>` element name appears in two different
contexts in MJCF:
1. Top-level `<tendon>` section — parsed by `parse_tendons()` (line 3059),
   contains `<spatial>` and `<fixed>` children.
2. `<equality><tendon>` — a constraint with attributes `tendon1`, `tendon2`,
   `polycoef`, `solref`, `solimp`.

Our parser already handles this via nesting context: `parse_equality()` only
processes children of `<equality>`, and `parse_tendons()` only processes
the top-level `<tendon>`. The attribute schemas are completely disjoint, so
there is no ambiguity at any level.

##### S2. Model builder

In `process_equality_constraints()` (`model_builder.rs:2664`), add a branch for
tendon equality after the existing Distance block (line 2859). Follows the
identical pattern used by Joint equality (lines 2774–2813):

```rust
// Process Tendon equality constraints
for ten_eq in &equality.tendons {
    let t1_id = *self
        .tendon_name_to_id
        .get(ten_eq.tendon1.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!(
                "Tendon equality references unknown tendon1: '{}'",
                ten_eq.tendon1
            ),
        })?;

    let t2_id = if let Some(ref t2_name) = ten_eq.tendon2 {
        *self
            .tendon_name_to_id
            .get(t2_name.as_str())
            .ok_or_else(|| ModelConversionError {
                message: format!(
                    "Tendon equality references unknown tendon2: '{t2_name}'"
                ),
            })?
    } else {
        usize::MAX // Sentinel: single-tendon mode
    };

    // Pack polynomial coefficients (up to 5 terms)
    let mut data = [0.0; 11];
    for (i, &coef) in ten_eq.polycoef.iter().take(5).enumerate() {
        data[i] = coef;
    }

    self.eq_type.push(EqualityType::Tendon);
    self.eq_obj1id.push(t1_id);
    self.eq_obj2id.push(t2_id);
    self.eq_data.push(data);
    self.eq_active.push(ten_eq.active);
    self.eq_solimp.push(ten_eq.solimp.unwrap_or(DEFAULT_SOLIMP));
    self.eq_solref.push(ten_eq.solref.unwrap_or(DEFAULT_SOLREF));
    self.eq_name.push(ten_eq.name.clone());
}
```

Note: `neq` is computed from `self.eq_type.len()` during `build()`, not
incremented manually.

##### S3. Constraint assembly: row counting

In `assemble_unified_constraints()` (`mujoco_pipeline.rs:14553`), change:

```rust
EqualityType::Tendon => 0,  // was: not implemented
```

to:

```rust
EqualityType::Tendon => 1,  // scalar constraint
```

##### S4. Constraint assembly: extraction

Implement `extract_tendon_equality_jacobian()` in `mujoco_pipeline.rs`, adjacent
to `extract_joint_equality_jacobian()` (line 18294). The function follows the
same signature and return type (`EqualityConstraintRows`):

```rust
fn extract_tendon_equality_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
    let nv = model.nv;
    let t1_id = model.eq_obj1id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    let l1 = data.ten_length[t1_id];
    let l1_0 = model.tendon_length0[t1_id];
    let j1 = &data.ten_J[t1_id];  // DVector<f64>, length nv

    let has_tendon2 = model.eq_obj2id[eq_id] != usize::MAX;

    let (pos_error, vel_error, j_row) = if has_tendon2 {
        // Two-tendon coupling
        let t2_id = model.eq_obj2id[eq_id];
        let l2 = data.ten_length[t2_id];
        let l2_0 = model.tendon_length0[t2_id];
        let j2 = &data.ten_J[t2_id];

        let dif = l2 - l2_0;

        // Residual: (L1 - L1_0) - data[0] - P(L2 - L2_0)
        let poly_val = eq_data[1] * dif
                     + eq_data[2] * dif * dif
                     + eq_data[3] * dif * dif * dif
                     + eq_data[4] * dif * dif * dif * dif;
        let pos = (l1 - l1_0) - eq_data[0] - poly_val;

        // Polynomial derivative: dP/d(dif)
        let deriv = eq_data[1]
                  + 2.0 * eq_data[2] * dif
                  + 3.0 * eq_data[3] * dif * dif
                  + 4.0 * eq_data[4] * dif * dif * dif;

        // Jacobian: J_tendon1 - deriv * J_tendon2
        let mut j = DMatrix::zeros(1, nv);
        for d in 0..nv {
            j[(0, d)] = j1[d] - deriv * j2[d];
        }

        // Velocity: J * qvel
        let vel = j.row(0).dot(&data.qvel);

        (pos, vel, j)
    } else {
        // Single-tendon mode: (L1 - L1_0) - data[0] = 0
        let pos = (l1 - l1_0) - eq_data[0];

        let mut j = DMatrix::zeros(1, nv);
        for d in 0..nv {
            j[(0, d)] = j1[d];
        }

        let vel = j.row(0).dot(&data.qvel);

        (pos, vel, j)
    };

    EqualityConstraintRows {
        j_rows: j_row,
        pos: vec![pos_error],
        vel: vec![vel_error],
    }
}
```

##### S5. Constraint extraction dispatch

In the extraction phase (`mujoco_pipeline.rs:14719`), change:

```rust
EqualityType::Tendon => continue,
```

to:

```rust
EqualityType::Tendon => extract_tendon_equality_jacobian(model, data, eq_id),
```

##### S6. Fix tendon tree builder + implement equality_trees for Tendon

**Background:** `tendon_treenum` and `tendon_tree` already exist on Model and
are computed during model building (`model_builder.rs:3757–3808`). No new fields
or computation functions are needed. However, the existing builder has a bug: it
only stores tree IDs for two-tree tendons (`tree_set.len() == 2`), leaving
single-tree tendons with `tendon_tree[2*t] == usize::MAX`. This breaks the
existing wake propagation code at `mujoco_pipeline.rs:12770–12771`.

**S6a. Fix tendon tree builder** (`model_builder.rs:3801`):

Change:
```rust
if tree_set.len() == 2 {
    let mut iter = tree_set.iter();
    if let (Some(&a), Some(&b)) = (iter.next(), iter.next()) {
        model.tendon_tree[2 * t] = a;
        model.tendon_tree[2 * t + 1] = b;
    }
}
```

To:
```rust
if tree_set.len() >= 1 {
    let mut iter = tree_set.iter();
    if let Some(&a) = iter.next() {
        model.tendon_tree[2 * t] = a;
    }
    if let Some(&b) = iter.next() {
        model.tendon_tree[2 * t + 1] = b;
    }
}
```

This stores the tree ID for single-tree tendons in `tendon_tree[2*t]`, fixing
both the existing wake propagation bug and enabling correct lookups for tendon
equality constraints.

**S6b. Implement equality_trees for Tendon** (`mujoco_pipeline.rs:13029`):

Replace:
```rust
EqualityType::Tendon => {
    // Tendon equality: scan tendon waypoints
    // For now, return sentinel (tendon equality not yet implemented)
    (sentinel, sentinel)
}
```

With:
```rust
EqualityType::Tendon => {
    let t1_id = model.eq_obj1id[eq_id];
    // Primary tree for tendon 1 (sentinel if treenum == 0, i.e. static)
    let tree1 = if model.tendon_treenum[t1_id] >= 1 {
        model.tendon_tree[2 * t1_id]
    } else {
        sentinel
    };

    if model.eq_obj2id[eq_id] != usize::MAX {
        // Two-tendon coupling: primary tree from each tendon
        let t2_id = model.eq_obj2id[eq_id];
        let tree2 = if model.tendon_treenum[t2_id] >= 1 {
            model.tendon_tree[2 * t2_id]
        } else {
            sentinel
        };
        (tree1, tree2)
    } else {
        // Single-tendon: if it spans two trees, return both
        if model.tendon_treenum[t1_id] == 2 {
            (model.tendon_tree[2 * t1_id], model.tendon_tree[2 * t1_id + 1])
        } else {
            (tree1, tree1)
        }
    }
}
```

**Tree semantics:**
- `treenum == 0`: tendon is static (all waypoints on world body) → returns
  `(sentinel, sentinel)`, constraint has no sleep effect.
- `treenum == 1`: returns `(tree, tree)` self-edge — tree wakes when disturbed.
- `treenum == 2`: both trees returned — either waking propagates to the other.
- `treenum >= 3`: only first two trees stored (conservative). Transitive closure
  via the existing union-find edge propagation handles reachability.

##### S7. Wake propagation

No explicit changes needed. Wake propagation already iterates all active equality
constraints and calls `equality_trees()` (`mujoco_pipeline.rs:12715–12732`).
Fixing the Tendon arm in `equality_trees()` (S6b) is sufficient — the existing
edge-propagation logic handles tree-pair waking correctly for all constraint
types.

#### Acceptance Criteria

1. **Loading**: `<equality><tendon tendon1="T1" tendon2="T2"/>` loads without
   error for a model with tendons named "T1" and "T2".
2. **Default polycoef (analytical trace)**: Two fixed tendons on separate hinge
   joints: T1 wraps J1 (coef=1.0), T2 wraps J2 (coef=1.0). Default
   `polycoef="0 1 0 0 0"`. Reference: L1_0 = L2_0 = 0 (both joints at qpos0=0).
   Set q1=0.5, q2=0.2:
   - dif = L2 - L2_0 = 0.2
   - poly_val = 1.0 * 0.2 = 0.2
   - pos_error = (0.5 - 0) - 0 - 0.2 = 0.3
   - deriv = 1.0
   - Jacobian: J = J_T1 - 1.0 * J_T2 = [1, 0] - [0, 1] = [1, -1]
   - Velocity (qvel=[1.0, 0.5]): vel = [1, -1] · [1.0, 0.5] = 0.5
   Verify `pos == 0.3`, `J == [1, -1]`, `vel == 0.5`. Tolerance: `1e-12`.
3. **Default polycoef (convergence)**: Same model as AC2. Displace q1=0.5,
   q2=0. After 1000 steps with default `solref`/`solimp`, both tendons
   converge to equal deviation: `|(L1-L1_0) - (L2-L2_0)| < 0.05`.
4. **Constant offset**: With `polycoef="0.1 1 0 0 0"`, same model as AC2.
   Set q1=0.5, q2=0.2:
   - pos_error = (0.5 - 0) - 0.1 - 1.0 * (0.2 - 0) = 0.2
   Verify `pos == 0.2`. Tolerance: `1e-12`.
5. **Single-tendon mode (analytical)**: `<tendon tendon1="T1"/>` (no tendon2)
   with `polycoef="0.1 0 0 0 0"`. Fixed tendon T1 wraps hinge J1 (coef=1.0).
   Set q1=0.4:
   - pos_error = (0.4 - 0) - 0.1 = 0.3
   - Jacobian: J = J_T1 = [1] (single DOF)
   Verify analytically. Tolerance: `1e-12`.
6. **Single-tendon mode (convergence)**: Same model as AC5. After 1000 steps
   from q1=0, verify convergence: `|L1 - L1_0 - 0.1| < 0.05`.
7. **Polynomial coupling (linear half-ratio)**: With `polycoef="0 0.5 0 0 0"`,
   two-tendon model. Test 5 configurations (q2 ∈ {-0.4, -0.2, 0, 0.2, 0.4},
   q1=0). Verify pos_error = `(L1-L1_0) - 0.5*(L2-L2_0)` at each
   configuration. Tolerance: `1e-12`.
8. **Quadratic polynomial**: With `polycoef="0 1 0.1 0 0"`, verify nonlinear
   coupling against MuJoCo for 5 different configurations. Tolerance: `1e-8`.
9. **Solver compatibility**: Works with PGS, CG, and Newton solvers. After 500
   steps, constraint violation < `0.05` for all three. Verify no NaN/Inf in
   `efc_force`.
10. **solref/solimp**: Two simulations, identical model except:
    - Stiff: `solref="0.02 1.0"` (default)
    - Soft: `solref="0.1 1.0"`
    After 200 steps from identical displaced initial state, measure constraint
    violation. Verify `violation_soft / violation_stiff.max(1e-15) > 3.0`.
11. **Inactive constraint**: `active="false"` produces no constraint rows
    (`nefc` unchanged). Verify `efc_force` is identical to a model with no
    tendon equality.
12. **Regression**: Models without tendon equality produce identical results.
13. **MuJoCo reference**: Compare constraint violation magnitude and `efc_force`
    against MuJoCo for a two-tendon coupling scenario over 100 steps.
    Tolerance: `1e-6`.

#### Implementation Notes

**Follow the Joint equality pattern.** `extract_joint_equality_jacobian()`
(`mujoco_pipeline.rs:18294`) is the exact template. The tendon case differs
only in:
- Uses `data.ten_length[t]` instead of `data.qpos[dof]`
- Uses `model.tendon_length0[t]` for reference lengths
- Uses `data.ten_J[t]` (`DVector<f64>`, nv-dimensional) instead of a single DOF
  column
- Polynomial is evaluated on `L2 - L2_0` (second tendon deviation), not `q1`

**Reference length computation.** `tendon_length0` is computed during model
building by running FK at `qpos0` and evaluating tendon lengths. Fixed tendons
are computed at `model_builder.rs:3669`; spatial tendons at
`model_builder.rs:4018`. Both are populated before constraint assembly runs.

**Jacobian storage.** `data.ten_J[t]` is a `DVector<f64>` of length `nv`,
recomputed each step by `mj_fwd_tendon()` (`mujoco_pipeline.rs:9362`). It is
available when constraint assembly runs (after the velocity phase).

**Tendon tree fields.** `tendon_treenum` and `tendon_tree` are precomputed
during model building (`model_builder.rs:3757–3808`) by walking each tendon's
wrap objects to find body → tree mappings. No new model fields are needed.

**Match exhaustiveness.** `EqualityType::Tendon` appears in 3 match sites in
`mujoco_pipeline.rs`, plus the builder. The compiler enforces exhaustiveness
on all `EqualityType` matches, so a missing arm is a compile error.

| # | Location | Current | Change | Spec |
|---|----------|---------|--------|------|
| 1 | `equality_trees()` (line 13029) | Returns `(sentinel, sentinel)` | Tendon tree lookup | S6b |
| 2 | Row counting (line 14553) | Returns `0` | Returns `1` | S3 |
| 3 | Extraction dispatch (line 14719) | `continue` | Calls `extract_tendon_equality_jacobian()` | S5 |
| 4 | `model_builder.rs` `process_equality_constraints()` (line 2664) | No Tendon branch | Pushes `EqualityType::Tendon` + tendon IDs + polycoef | S2 |

No other match sites exist. The enum definition (`mujoco_pipeline.rs:611`)
and parser (`parser.rs`) are additive (new variant / new parse arm), not
match-dependent.

#### Files

- `sim/L0/mjcf/src/types.rs` — add `MjcfTendonEquality` struct, add `tendons`
  field to `MjcfEquality`
- `sim/L0/mjcf/src/parser.rs` — parse `<equality><tendon>` elements (add
  `b"tendon"` arms in `parse_equality()`, implement
  `parse_tendon_equality_attrs()`)
- `sim/L0/mjcf/src/model_builder.rs` — process tendon equality in
  `process_equality_constraints()`; fix tendon tree builder to store IDs for
  single-tree tendons (S6a)
- `sim/L0/core/src/mujoco_pipeline.rs` — implement
  `extract_tendon_equality_jacobian()`; update row counting (0 → 1);
  update extraction dispatch; update `equality_trees()` Tendon arm
- `sim/L0/tests/integration/` — new tests for tendon equality scenarios
