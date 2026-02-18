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
`noslip_postprocess()` (`mujoco_pipeline.rs`, ~350 LOC).
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

### 34. `actlimited` / `actrange` / `actearly` — Activation State Clamping
**Status:** Parsing done, runtime remaining | **Effort:** S–M | **Prerequisites:** None

#### Current State

**Parsing complete (Batch 1 defaults refactor):** `actlimited`, `actrange`,
`actearly`, `lengthrange`, and `group` are parsed on both `MjcfActuator` and
`MjcfActuatorDefaults` (`types.rs`, `parser.rs`). The defaults pipeline merges
and applies them via `merge_actuator_defaults()` / `apply_to_actuator()`
(`defaults.rs`). All 5 fields use `Option<T>` with `is_none()` guards.

**Not yet wired to runtime:** The parsed values are not forwarded to `Model`
fields and not enforced during activation dynamics. Only muscle activations
are clamped (hardcoded `[0, 1]`). All other stateful actuators (`dyntype=Integrator`,
`Filter`, `FilterExact`) can grow unbounded.

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
pub actuator_actrange: Vec<[f64; 2]>,  // default: [0.0, 0.0]
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

1. **Integrator clamping**: An integrator-type actuator with
   `actlimited="true" actrange="-1 1"` has activation clamped to `[-1, 1]`
   after 1000 steps of `ctrl=1.0`.
2. **Unbounded without actlimited**: Same actuator without `actlimited` reaches
   activation > 1 (existing behavior preserved).
3. **Muscle default**: Muscle actuators still default to `[0, 1]` clamping when
   `actlimited` is not explicitly set.
4. **Muscle override**: Muscle with `actlimited="true" actrange="-0.5 1.5"`
   clamps to `[-0.5, 1.5]` instead of `[0, 1]`.
5. **FilterExact integration**: A `<position>` actuator (FilterExact dynamics)
   with `actlimited="true"` uses the exact exponential integration formula
   AND clamps to actrange.
6. **`actearly` force**: With `actearly="true"`, the force at step `t` uses
   `act(t+1)` instead of `act(t)`. Verify by comparing a single-step force
   output: with `actearly`, force responds to ctrl change one step earlier.
7. **`actearly` + `actlimited`**: With both flags, the force uses the
   clamped predicted activation (integrated + clamped before force computation).
8. **MuJoCo reference**: Compare 100-step activation trajectory for an
   integrator actuator with `actlimited="true" actrange="-1 1"` against MuJoCo.
   Tolerance: `1e-12` (exact match expected for Euler integration + clamping).
9. **~~Default class inheritance~~** Done (Batch 1 defaults refactor).

#### Files

- ~~`sim/L0/mjcf/src/model_builder.rs` — parse actlimited/actrange from MJCF~~ Done
- `sim/L0/mjcf/src/model_builder.rs` — forward parsed values to Model; muscle
  default actrange setup
- `sim/L0/core/src/mujoco_pipeline.rs` — add Model fields; implement
  `mj_next_activation()`; wire `actearly` in force computation; replace
  muscle-only clamping in integrator with `mj_next_activation()` call

---

### 35. `gravcomp` — Body Gravity Compensation
**Status:** Not started | **Effort:** S–M | **Prerequisites:** None

#### Current State

Not parsed, not stored, not enforced. Silently ignored when present in MJCF. Test
assets (Apptronik Apollo) use `gravcomp="0"` which is silently dropped.

#### Objective

Parse `gravcomp` from `<body>`, store on Model, compute an anti-gravity force
per body, and add it to `qfrc_passive` during the passive force computation.

#### MuJoCo Reference (`mj_gravcomp` in `engine_passive.c`)

MuJoCo implements gravcomp as a **passive force**, NOT a modification to
`qfrc_bias` or `mj_rne`. The function `mj_gravcomp()` is called from
`mj_passive()`:

```c
static int mj_gravcomp(const mjModel* m, mjData* d) {
    if (!m->ngravcomp || DISABLED(mjDSBL_GRAVITY) || norm3(gravity) == 0)
        return 0;

    for (int b = 1; b < nbody; b++) {
        if (body_gravcomp[b]) {
            // force = -gravity * body_mass[b] * body_gravcomp[b]
            scl3(force, gravity, -(body_mass[b] * body_gravcomp[b]));
            // Apply at body CoM position, project to joint space
            mj_applyFT(m, d, force, zero_torque, xipos[b], b, qfrc_gravcomp);
        }
    }
    return has_gravcomp;
}
```

**Critical details:**
- Uses **`body_mass[i]`** (individual body mass), NOT `body_subtreemass`.
- Result goes to **`qfrc_gravcomp`** (a dedicated array), which is then added
  to `qfrc_passive` or `qfrc_actuator` depending on the `jnt_actgravcomp` flag.
- The force is applied at `xipos[b]` (body center of mass in world frame).
- `mj_applyFT()` projects the Cartesian force through the kinematic chain
  Jacobian to generalized forces.
- Gravity disable flag is respected.
- World body (b=0) is skipped.

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

MuJoCo allows any non-negative value (not just `[0, 1]`). A value > 1.0 means
over-compensation (the body floats upward). We match MuJoCo: no range clamping.

##### S2. Model storage

Add to `Model`:
```rust
pub body_gravcomp: Vec<f64>,   // per-body, default 0.0
pub ngravcomp: usize,          // count of bodies with gravcomp > 0 (optimization)
```

Add to `Data`:
```rust
pub qfrc_gravcomp: DVector<f64>,  // nv-dimensional, zeroed each step
```

Forward from `MjcfBody.gravcomp` in `model_builder.rs`. Compute `ngravcomp`
during model building for early-exit optimization.

**Note:** MuJoCo writes gravcomp forces to a **dedicated `qfrc_gravcomp`** array,
NOT directly to `qfrc_passive`. This is because the routing depends on
`jnt_actgravcomp` — see S5.

##### S3. Runtime: `mj_gravcomp()` as passive force

Implement `mj_gravcomp()` as a function called from `mj_fwd_passive()`:

```rust
fn mj_gravcomp(model: &Model, data: &mut Data) -> bool {
    if model.ngravcomp == 0 {
        return false;
    }
    let gravity = model.gravity;
    if gravity.norm() == 0.0 {
        return false;  // gravity disabled or zero — no compensation needed
    }

    data.qfrc_gravcomp.fill(0.0);
    let mut has_gravcomp = false;

    for b in 1..model.nbody {
        let gc = model.body_gravcomp[b];
        if gc == 0.0 {
            continue;
        }
        has_gravcomp = true;
        // Anti-gravity force at body CoM (xipos, NOT xpos)
        let force = -gravity * model.body_mass[b] * gc;
        // Project through Jacobian to generalized forces
        mj_apply_ft(model, data, &force, &Vector3::zeros(), &data.xipos[b], b,
                     &mut data.qfrc_gravcomp);
    }

    has_gravcomp
}
```

**Critical:** Uses `data.xipos[b]` (body center of mass in world frame), NOT
`data.xpos[b]` (body frame origin). MuJoCo applies the antigravity force at
`xipos` because gravity acts at the CoM. These differ when `ipos` (body-local
CoM offset) is nonzero. Using `xpos` instead of `xipos` would produce wrong
torques for bodies whose CoM doesn't coincide with their frame origin.

##### S3b. `mj_apply_ft()` — Cartesian wrench to generalized forces

This function computes the full translational + rotational Jacobian at a point
on a body and projects a 6D wrench to generalized forces. It follows MuJoCo's
`mj_applyFT` which internally calls `mj_jac()` to build `jacp` (3×nv) and
`jacr` (3×nv), then computes `qfrc += jacp^T * force + jacr^T * torque`.

Our implementation walks the kinematic chain (same traversal as
`accumulate_point_jacobian()`) but accumulates the full 3D force/torque
projection per DOF instead of a scalar direction projection:

```rust
fn mj_apply_ft(
    model: &Model, data: &Data,
    force: &Vector3<f64>, torque: &Vector3<f64>,
    point: &Vector3<f64>, body_id: usize,
    qfrc: &mut DVector<f64>,
) {
    if body_id == 0 {
        return;  // world body has no DOFs
    }
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
                    // jacp column = axis × r; jacr column = axis
                    // qfrc = (axis × r) · force + axis · torque
                    qfrc[dof] += axis.cross(&r).dot(force) + axis.dot(torque);
                }
                MjJointType::Slide => {
                    // jacp column = axis; jacr column = 0
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    qfrc[dof] += axis.dot(force);
                }
                MjJointType::Ball => {
                    let anchor = data.xpos[jnt_body]
                               + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    // Ball DOFs use body-frame axes (columns of xmat/xquat rotation)
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        // jacp column = omega × r; jacr column = omega
                        qfrc[dof + i] += omega.cross(&r).dot(force)
                                        + omega.dot(torque);
                    }
                }
                MjJointType::Free => {
                    // Translation DOFs (0,1,2): global x,y,z
                    qfrc[dof]     += force[0];
                    qfrc[dof + 1] += force[1];
                    qfrc[dof + 2] += force[2];
                    // Rotation DOFs (3,4,5): body-frame axes
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

**Design note:** This function closely mirrors `accumulate_point_jacobian()` but
computes the full wrench projection `J_p^T * f + J_r^T * τ` per DOF instead of
the scalar `direction^T * J_p` used for tendon Jacobians. Both use the same
chain walk and the same body-frame axis convention for Ball/Free joints (matching
MuJoCo's `cdof` layout).

##### S4. Routing: `qfrc_gravcomp` → `qfrc_passive`

After `mj_gravcomp()` returns, add the gravcomp forces to `qfrc_passive`. In
MuJoCo, this routing is conditional on `jnt_actgravcomp` — but since we don't
yet support `actgravcomp`, we unconditionally add all gravcomp to passive:

```rust
// In mj_fwd_passive(), after calling mj_gravcomp():
let has_gc = mj_gravcomp(model, data);
if has_gc {
    // For now, add all gravcomp to passive forces.
    // TODO(future): when jnt_actgravcomp is implemented, only add for joints
    // where jnt_actgravcomp[dof_jntid[dof]] == false. The rest goes through
    // the actuator system.
    for dof in 0..model.nv {
        data.qfrc_passive[dof] += data.qfrc_gravcomp[dof];
    }
}
```

**Future-proofing note:** MuJoCo's `jnt_actgravcomp` flag controls whether a
joint's gravity compensation is applied as a passive force or routed through
the actuator system. We leave a TODO for this. The dedicated `qfrc_gravcomp`
array makes the future routing change a trivial addition.

##### S5. Where NOT to implement

Do NOT modify `mj_rne()` or `qfrc_bias`. MuJoCo computes gravcomp as a
**separate passive force** added to `qfrc_passive` (via `qfrc_gravcomp`), not
as a bias force adjustment. This keeps the bias force computation clean and
makes gravcomp composable with other passive forces.

#### Acceptance Criteria

1. **Full compensation**: A free body with `gravcomp="1"` and no contact
   experiences zero net gravitational acceleration (gravity + antigravity
   cancel). Verify `qacc` has zero gravitational component after `forward()`.
2. **No compensation**: A body with `gravcomp="0"` (default) produces
   identical results to a body without the attribute.
3. **Partial compensation**: A body with `gravcomp="0.5"` has half the
   gravitational acceleration of an uncompensated body. Verify by comparing
   `qacc` magnitudes.
4. **Over-compensation**: A free body with `gravcomp="2"` accelerates
   upward (against gravity). Verify `qacc` sign reversal.
5. **Apptronik Apollo**: `gravcomp="0"` parses without warning, produces
   no behavioral change.
6. **Kinematic chain**: A 3-link arm with `gravcomp="1"` on all links:
   verify `qfrc_passive` contains antigravity forces and the arm holds
   position against gravity without actuator input.
7. **Gravity disabled**: With `<option gravity="0 0 0">`, gravcomp produces
   zero force regardless of `gravcomp` values.
8. **CoM offset correctness**: A body with non-trivial `ipos` (local CoM
   offset ≠ origin) and `gravcomp="1"`: verify force is applied at `xipos`
   (body CoM), not `xpos` (body frame origin). The torque contribution
   should differ from the case where `ipos = [0,0,0]`.
9. **`qfrc_gravcomp` populated**: After `forward()`, `data.qfrc_gravcomp`
   is nonzero for DOFs affected by bodies with `gravcomp > 0`, and zero
   for unaffected DOFs.
10. **MuJoCo reference**: Compare `qfrc_passive` for a body with
    `gravcomp="0.7"` against MuJoCo. Tolerance: `1e-10`.

#### Files

- `sim/L0/mjcf/src/types.rs` — add `gravcomp: Option<f64>` to `MjcfBody`
- `sim/L0/mjcf/src/parser.rs` — parse `gravcomp` in `parse_body_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — forward to `Model.body_gravcomp`
- `sim/L0/core/src/mujoco_pipeline.rs` — add `body_gravcomp: Vec<f64>` and
  `ngravcomp: usize` to Model; add `qfrc_gravcomp: DVector<f64>` to Data;
  implement `mj_gravcomp()` and `mj_apply_ft()`; call from
  `mj_fwd_passive()` with routing to `qfrc_passive`
- `sim/L0/tests/integration/` — new tests for gravcomp scenarios

---

### 36. Body-Transmission Actuators (Adhesion)
**Status:** Parsing done, transmission missing | **Effort:** M | **Prerequisites:** None

#### Current State

**What exists:**
- **MJCF parsing**: `<actuator><adhesion>` is fully parsed. `MjcfActuatorType::Adhesion`
  sets `gaintype = Fixed` (with user-specified gain), `biastype = None`,
  `dyntype = None`, `ctrllimited = true`. The `body` attribute is parsed and stored
  as `actuator.body: Option<String>`.
- **Force generation**: Adhesion uses standard gain/bias: `force = gain * ctrl`.
  Clamped by `forcerange`. No activation dynamics. This already works through the
  standard `mj_fwd_actuation()` Phase 2 force computation.
- **Error on body transmission**: `model_builder.rs` returns
  `ModelConversionError` when `actuator.body` is `Some(...)`: "Actuator '...' uses
  body transmission '...' which is not yet supported." This is the **only** blocker.

**What's missing:**
- `ActuatorTransmission::Body` variant in the enum (currently: Joint/Tendon/Site).
- `mj_transmission_body()` — computes moment arm from contact normal Jacobians.
- Force application via `qfrc_actuator += moment_arm * force` in Phase 3 of
  `mj_fwd_actuation()`.

**Existing transmission patterns** (for reference):
- **Joint**: `qfrc_actuator[dof] += gear * force` (direct DOF application).
- **Tendon**: `apply_tendon_force()` with cached `ten_J` Jacobian.
- **Site**: `qfrc_actuator[dof] += actuator_moment[i][dof] * force` (pre-computed
  moment Jacobian from `mj_transmission_site()`).

Body transmission follows the **Site pattern** — pre-compute a moment arm vector
in a transmission function, then apply it in Phase 3.

#### Objective

Implement body transmission (`ActuatorTransmission::Body`) so adhesion actuators
load and function. The adhesion force pulls/pushes the target body toward/away from
surfaces via contact normal Jacobians.

#### MuJoCo Reference (`mjTRN_BODY` in `engine_core_smooth.c`)

MuJoCo's body transmission iterates all contacts and separates them into two
categories:

1. **Active contacts** (`!con->exclude`): Weighted into an `efc_force` vector
   (weight 1.0 for elliptic/frictionless, `0.5/npyramid` per edge for pyramidal).
   The Jacobian contribution is extracted via `mj_mulJacTVec(efc_force)` — this
   effectively sums the weighted constraint Jacobian rows.

2. **Excluded-in-gap contacts** (`con->exclude == 1`): Jacobian computed directly
   via `mj_jacDifPair()` at the contact point, projected along contact normal
   (`con->frame`). Accumulated into a separate `moment_exclude` buffer.

Both categories are counted, and the final moment is:
```
moment = -(1/counter) * (J_active + J_exclude)
```

The negative sign makes positive ctrl produce an **attractive** force.

**Contact filtering:**
- Skip flex contacts (`geom < 0`)
- Skip contacts not involving the target body
- `exclude == 0`: active contact → weight into efc_force
- `exclude == 1`: excluded but in gap → compute Jacobian directly
- `exclude > 1`: skip entirely

**Edge cases:**
- Zero contacts: moment is zero, actuator produces no force.
- `actuator_length[i] = 0` always (adhesion has no length concept).
- Only `gear[0]` is used (scalar gear for body transmission).

#### Specification

##### S1. `ActuatorTransmission::Body` enum variant

Add a new variant to the transmission enum. Since the body ID is already stored
in `actuator_trnid`, we do NOT need to embed it in the enum:

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

Implement following MuJoCo's algorithm. We have two sub-approaches depending on
whether `efc_J` (constraint Jacobian) is available at the call site.

**Key architectural question:** MuJoCo calls `mj_transmission()` from the
"smooth" phase (`mj_fwd_position` → `mj_fwd_velocity`), where contacts exist
but constraint rows (`efc_J`) are not yet assembled. However, MuJoCo's active-
contact path uses `efc_force` as a selector vector and `mj_mulJacTVec()` which
reads `efc_J`. This means MuJoCo has `efc_J` available during transmission.

In our pipeline, `mj_transmission_site()` runs during the position/velocity
phase (before constraint assembly). If `efc_J` is not yet populated, we must
use the direct Jacobian computation approach for ALL contacts (not just
excluded-in-gap ones).

**Approach: Direct Jacobian computation (safe, no efc_J dependency):**

```rust
fn mj_transmission_body(
    model: &Model,
    data: &mut Data,
    actuator_id: usize,
) {
    let body_id = model.actuator_trnid[actuator_id][0];
    let nv = model.nv;
    let moment = &mut data.actuator_moment[actuator_id];
    moment.fill(0.0);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];
        // Skip flex contacts (MuJoCo: geom < 0)
        if contact.flex_vertex.is_some() { continue; }

        let b1 = model.geom_body[contact.geom1];
        let b2 = model.geom_body[contact.geom2];
        if b1 != body_id && b2 != body_id { continue; }

        // Compute J_diff = J(b1, contact_pos) - J(b2, contact_pos)
        // Project along contact normal: j_normal = frame[0..3] · J_diff
        let j_normal = compute_contact_normal_jacobian(model, data, contact);
        *moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        let gear = model.actuator_gear[actuator_id][0];
        *moment *= -gear / (count as f64);
    }

    data.actuator_length[actuator_id] = 0.0;
}
```

**`compute_contact_normal_jacobian()`** helper:

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
    let normal = &contact.normal;  // contact frame row 0

    // J_diff_trans = J_trans(b1, point) - J_trans(b2, point)  [3 x nv]
    // j_normal = normal^T · J_diff_trans                       [1 x nv]
    let mut j_normal = DVector::zeros(nv);

    // Walk kinematic chain for body1 (+1 contribution)
    accumulate_point_jacobian(
        model, &data.xpos, &data.xquat,
        &mut j_normal, b1, point, normal, 1.0,
    );
    // Walk kinematic chain for body2 (-1 contribution)
    accumulate_point_jacobian(
        model, &data.xpos, &data.xquat,
        &mut j_normal, b2, point, normal, -1.0,
    );

    j_normal
}
```

Where `accumulate_point_jacobian()` (line ~9655 in `mujoco_pipeline.rs`) walks
the chain from `body_id` to root, accumulating `scale * (direction . joint_contribution)`
for each joint. This function already exists and is used for tendon Jacobian
computation. Its signature:
```rust
fn accumulate_point_jacobian(
    model: &Model, xpos: &[Vector3<f64>], xquat: &[UnitQuaternion<f64>],
    ten_j: &mut DVector<f64>, body_id: usize,
    point: &Vector3<f64>, direction: &Vector3<f64>, scale: f64,
)
```
Call it with `direction = contact.normal` and `scale = +1.0` for body1,
`scale = -1.0` for body2.

##### S3. Call site in forward pipeline

Call `mj_transmission_body()` alongside `mj_transmission_site()` in the
position/velocity phase, before `mj_fwd_actuation()`:

```rust
// In forward(), after collision detection and before actuation:
for i in 0..model.nu {
    match model.actuator_trntype[i] {
        ActuatorTransmission::Site => mj_transmission_site(model, data, i),
        ActuatorTransmission::Body => mj_transmission_body(model, data, i),
        _ => {}
    }
}
```

##### S4. Force application in Phase 3

In `mj_fwd_actuation()` Phase 3, add the Body case:

```rust
ActuatorTransmission::Body => {
    // Same as Site: moment is pre-computed
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```

##### S5. Model builder: remove error, resolve body name to ID

In `model_builder.rs`, replace the error with body name resolution:

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

##### S6. Contact filtering edge cases

- **No contacts**: `moment` stays zero, force has no effect. Correct.
- **Multiple contacts**: Averaged by `count`. Prevents adhesion from scaling
  with contact count.
- **Flex contacts**: Skipped (`flex_vertex.is_some()`). Matches MuJoCo
  (`geom < 0` filter).
- **World body (body 0) as target**: If `body_id == 0` (world body), the
  target itself has no DOFs. The chain walk for body 0 returns immediately
  (see `accumulate_point_jacobian` body_id==0 early return). The other contact
  body's Jacobian is subtracted from zero, producing a valid moment. This is
  handled naturally — no special case needed.
- **World body as contact partner**: When one contact body is body 0 (e.g.,
  floor), the chain walk for body 0 contributes zero Jacobian. Only the
  movable body's DOFs appear in the result. Correct behavior, no special case.
- **Excluded-in-gap contacts**: MuJoCo distinguishes active contacts (with
  `efc_J` rows) from excluded-in-gap contacts (no `efc_J` rows). Our codebase
  stores all detected contacts in `data.contacts` and decides constraint
  inclusion during assembly (via `includemargin`). Since we use direct Jacobian
  computation for ALL contacts (not `efc_J`), the active vs excluded-in-gap
  distinction is irrelevant — all contacts in `data.contacts` involving the
  target body contribute equally. This produces the correct result because
  MuJoCo's two-path approach (efc_J for active, direct Jacobian for excluded)
  yields the same Jacobian as our single-path approach (direct Jacobian for all).
- **Pyramidal vs elliptic weighting**: MuJoCo weights active contacts
  differently based on cone type when using `efc_J`. Since we use direct
  Jacobian computation (one normal Jacobian per contact), every contact
  contributes weight 1.0 and is divided by `count`. This produces the same
  result as MuJoCo's approach because:
  - Elliptic: MuJoCo sets `efc_force[normal_row] = 1.0`, then `J^T * f` gives
    the normal Jacobian directly. Same as our direct computation.
  - Pyramidal: MuJoCo sets each of `2*(dim-1)` edges to `0.5/(dim-1)`. The sum
    of pyramid edges weighted this way reconstructs the normal Jacobian (edges
    are `J_n ± μ*J_t`, the tangential parts cancel, leaving `J_n`). Same result.
  - The mathematical equivalence holds because the tangential components cancel
    in the symmetric weighting scheme.

#### Acceptance Criteria

1. **Loading**: `<actuator><adhesion body="box" gain="100"/>` loads without error
   for a model with a body named "box".
2. **Zero force when no contact**: With `ctrl=1.0` but the body not in contact,
   `qfrc_actuator` contribution from the adhesion actuator is zero.
3. **Attractive force**: A sphere resting on a plane with adhesion `ctrl=1.0`
   produces a downward `qfrc_actuator` (pulling sphere toward plane). Verify
   sign by checking the sphere's equilibrium penetration is deeper than without
   adhesion.
4. **Force magnitude**: For a single contact, the generalized force equals
   `-(gain * ctrl) * J_normal`. Compare against MuJoCo `qfrc_actuator`
   for a sphere-on-plane adhesion scenario. Tolerance: `1e-10`.
5. **Multiple contacts**: With 4 contacts (box on plane), moment arm is the
   average of 4 normal Jacobians. Compare against MuJoCo.
6. **Regression**: Models without adhesion actuators produce identical results.
7. **`actuator_moment` populated**: After `forward()`, `data.actuator_moment[i]`
   for a body-transmission actuator is nonzero when contacts exist and zero
   when no contacts exist.
8. **`actuator_length` is zero**: Body transmission always has
   `actuator_length[i] == 0`.
9. **Gear scaling**: Adhesion with `gear="2"` produces 2x the moment arm
   (but the same scalar force). Verify `actuator_moment` scales with gear.

#### Implementation Notes

**`accumulate_point_jacobian()` reuse.** This function already exists (line ~9655)
and is used for tendon Jacobians. It walks the kinematic chain and projects a
direction vector through each joint's contribution. For body transmission, call
it once per body per contact with `direction = contact.normal`. For `mj_apply_ft()`
(#35), the same chain-walk pattern is needed but with wrench (force + torque)
projection — a new function but sharing the same traversal logic.

**Pipeline ordering.** Contacts are detected in `mj_fwd_position()` →
`mj_collision()`, which runs before `mj_transmission_body()`. The contacts are
from the **current step's** collision phase (not the previous timestep).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add `Body` to `ActuatorTransmission`;
  implement `mj_transmission_body()` and `compute_contact_normal_jacobian()`;
  add Body case to Phase 3 force application; add Body case to transmission
  dispatch in forward pipeline
- `sim/L0/mjcf/src/model_builder.rs` — replace error with body name resolution
- `sim/L0/tests/integration/` — new test file or section for adhesion actuators

---

### 37. Tendon Equality Constraints
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

**Pipeline:** `EqualityType::Tendon` exists in the enum but is ignored during
constraint assembly. Row count returns 0, extraction returns `continue`. Any
model using `<equality><tendon>` silently gets no constraint enforcement.

**MJCF parser:** `MjcfEquality` has no `tendons` field. The `<tendon>` element
inside `<equality>` is not parsed. Models with tendon equality constraints will
fail to load or silently drop the element.

**Tendon kinematics:** `ten_length`, `ten_velocity`, `ten_J` (Jacobian) are
fully computed for both fixed and spatial tendons. `tendon_length0` (reference
length at qpos0) exists on Model.

**Existing pattern:** `extract_joint_equality_jacobian()` implements the Joint
equality case with polynomial coupling. Tendon equality follows the same pattern
but uses tendon lengths and Jacobians instead of joint positions and DOF columns.

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

Add `MjcfTendonEquality` type and parse `<equality><tendon>`:

In `types.rs`:
```rust
pub struct MjcfTendonEquality {
    pub name: Option<String>,
    pub tendon1: String,          // required
    pub tendon2: Option<String>,  // optional (single-tendon mode if absent)
    pub polycoef: [f64; 5],       // default: [0, 1, 0, 0, 0]
    pub active: Option<bool>,
    pub solref: Option<[f64; 2]>,
    pub solimp: Option<[f64; 5]>,
}
```

In `MjcfEquality`:
```rust
pub struct MjcfEquality {
    pub connects: Vec<MjcfConnect>,
    pub welds: Vec<MjcfWeld>,
    pub joints: Vec<MjcfJointEquality>,
    pub distances: Vec<MjcfDistance>,
    pub tendons: Vec<MjcfTendonEquality>,  // NEW
}
```

In `parser.rs`, parse `<tendon>` elements inside `<equality>`:
- `tendon1` attribute (required)
- `tendon2` attribute (optional)
- `polycoef` attribute (5 floats, default `"0 1 0 0 0"`)
- Standard equality attributes: `name`, `active`, `solref`, `solimp`

**Parser disambiguation:** The `<tendon>` element name appears in two different
contexts in MJCF:
1. Top-level `<tendon>` section — contains `<spatial>` and `<fixed>` children
   with attributes like `springlength`, `damping`, `range`, etc.
2. `<equality><tendon>` — a constraint with attributes `tendon1`, `tendon2`,
   `polycoef`, `solref`, `solimp`.

Our parser already handles this via nesting context: `parse_equality()` only
processes children of `<equality>`, and `parse_tendon_section()` only processes
the top-level `<tendon>`. When we see `<tendon>` inside an `<equality>` parse
context, we create `MjcfTendonEquality` (not a tendon definition). The attribute
schemas are completely disjoint, so there's no ambiguity at the attribute level.

##### S2. Model builder

In `process_equality_constraints()`, add a branch for tendon equality:

```rust
for ten_eq in &equality.tendons {
    let t1_id = *self.tendon_name_to_id.get(ten_eq.tendon1.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!("Tendon equality: tendon1 '{}' not found", ten_eq.tendon1),
        })?;
    let t2_id = ten_eq.tendon2.as_ref()
        .map(|name| self.tendon_name_to_id.get(name.as_str())
            .copied()
            .ok_or_else(|| ModelConversionError {
                message: format!("Tendon equality: tendon2 '{}' not found", name),
            }))
        .transpose()?;

    model.eq_type.push(EqualityType::Tendon);
    model.eq_obj1id.push(t1_id);
    model.eq_obj2id.push(t2_id.unwrap_or(usize::MAX));  // sentinel if single-tendon
    let mut data = [0.0; 11];
    data[..5].copy_from_slice(&ten_eq.polycoef);
    model.eq_data.push(data);
    model.eq_active.push(ten_eq.active.unwrap_or(true));
    model.eq_solref.push(ten_eq.solref.unwrap_or(model.default_solref));
    model.eq_solimp.push(ten_eq.solimp.unwrap_or(model.default_solimp));
    model.eq_name.push(ten_eq.name.clone());
    model.neq += 1;
}
```

##### S3. Constraint assembly: row counting

In `assemble_unified_constraints()`, Phase 1 row counting, change:

```rust
EqualityType::Tendon => 0,  // was: not implemented
```

to:

```rust
EqualityType::Tendon => 1,  // scalar constraint
```

##### S4. Constraint assembly: extraction

Implement `extract_tendon_equality_jacobian()`:

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
    let j1 = &data.ten_J[t1_id];

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

        // Jacobian: J_tendon1 + (-deriv) * J_tendon2
        let mut j = DMatrix::zeros(1, nv);
        for d in 0..nv {
            j[(0, d)] = j1[d] - deriv * j2[d];
        }

        // Velocity: J · qvel
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

In the extraction phase, change:

```rust
EqualityType::Tendon => continue,
```

to:

```rust
EqualityType::Tendon => extract_tendon_equality_jacobian(model, data, eq_id),
```

##### S6. Tree detection for sleeping

MuJoCo precomputes two fields during model setup:
- `tendon_treenum[t]` — number of distinct kinematic trees along tendon t's path
- `tendon_treeid[2*t]`, `tendon_treeid[2*t+1]` — first two tree IDs (or -1)

These are computed by walking the tendon's wrap objects to find body → tree
mappings. We need either (a) precomputed fields like MuJoCo, or (b) a runtime
helper. Option (a) is cleaner and avoids repeated work.

**S6a. Model fields for tendon tree membership:**

Add to `Model`:
```rust
pub tendon_treenum: Vec<usize>,         // per-tendon, 0 = static
pub tendon_treeid: Vec<[Option<usize>; 2]>,  // first two tree IDs
```

**S6b. Compute during model building (in `setconst` equivalent):**

```rust
fn compute_tendon_tree_ids(model: &mut Model) {
    for t in 0..model.ntendon {
        let mut seen_trees: Vec<usize> = Vec::new();
        let adr = model.tendon_adr[t];
        let num = model.tendon_num[t];
        for w in adr..adr + num {
            let body_id = match model.wrap_type[w] {
                WrapType::Joint => model.jnt_body[model.wrap_objid[w]],
                WrapType::Site  => model.site_body[model.wrap_objid[w]],
                WrapType::Geom  => model.geom_body[model.wrap_objid[w]],
                WrapType::Pulley => continue,
            };
            if body_id == 0 { continue; }  // world body = no tree
            let tree_id = model.body_treeid[body_id];
            if !seen_trees.contains(&tree_id) {
                seen_trees.push(tree_id);
            }
        }
        model.tendon_treenum[t] = seen_trees.len();
        model.tendon_treeid[t] = [
            seen_trees.get(0).copied(),
            seen_trees.get(1).copied(),
        ];
    }
}
```

**S6c. Sleep state for tendon equality constraints:**

In `equality_trees()`, implement the Tendon case using the precomputed fields:

```rust
EqualityType::Tendon => {
    let t1_id = model.eq_obj1id[eq_id];
    let tree1 = model.tendon_treeid[t1_id][0];

    if model.eq_obj2id[eq_id] != usize::MAX {
        let t2_id = model.eq_obj2id[eq_id];
        let tree2 = model.tendon_treeid[t2_id][0];
        (tree1, tree2)
    } else {
        (tree1, tree1)  // single-tendon: same tree both sides
    }
}
```

**Sleep state logic** (matches MuJoCo's `mj_tendonSleepState`):
- 0 trees: tendon is static (all waypoints on world body) → `STATIC`
- 1 tree: awake iff that tree is awake
- 2 trees: awake iff either tree is awake
- 3+ trees: always treated as awake (conservative, since only 2 IDs stored)

##### S7. Wake propagation

In `mj_wake_equality()`, add the Tendon case so that tendon equality constraints
wake connected trees when one side is disturbed. Uses the same `tendon_treeid`
fields to identify which trees to wake.

#### Acceptance Criteria

1. **Loading**: `<equality><tendon tendon1="T1" tendon2="T2"/>` loads without
   error for a model with tendons named "T1" and "T2".
2. **Default polycoef**: With default `polycoef="0 1 0 0 0"`, both tendons
   maintain equal deviation from their reference lengths.
3. **Constant offset**: With `polycoef="0.1 1 0 0 0"`, tendon1 is offset by 0.1
   from the coupled position. Verify equilibrium position.
4. **Single-tendon mode**: `<tendon tendon1="T1"/>` (no tendon2) constrains T1
   to its reference length + data[0].
5. **Polynomial coupling**: With `polycoef="0 0.5 0 0 0"`, tendon1's deviation
   is half of tendon2's deviation. Verify ratio across a range of configurations.
6. **Quadratic polynomial**: With `polycoef="0 1 0.1 0 0"`, verify nonlinear
   coupling against MuJoCo for 5 different configurations. Tolerance: `1e-8`.
7. **Solver compatibility**: Works with PGS, CG, and Newton solvers. Verify
   constraint violation converges to zero for all three.
8. **solref/solimp**: Custom `solref` and `solimp` values affect constraint
   stiffness/damping. Verify stiffer `solref` reduces violation faster.
9. **Inactive constraint**: `active="false"` produces no constraint effect.
10. **Regression**: Models without tendon equality produce identical results.
11. **MuJoCo reference**: Compare constraint violation magnitude and `efc_force`
    against MuJoCo for a two-tendon coupling scenario over 100 steps.
    Tolerance: `1e-6`.

#### Implementation Notes

**Follow the Joint equality pattern.** `extract_joint_equality_jacobian()` is
the exact template. The tendon case differs only in:
- Uses `ten_length[t]` instead of `qpos[dof]`
- Uses `tendon_length0[t]` for reference lengths
- Uses `ten_J[t]` (nv-dimensional) instead of a single DOF column
- Polynomial is evaluated on `L2 - L2_0` (second tendon deviation), not `q1`

**Reference length computation.** `tendon_length0` is computed during model
building by running FK at `qpos0` and evaluating tendon lengths. Verify this
is populated for all tendons, not just spatial ones.

**Jacobian storage.** `ten_J[t]` is an `nv`-dimensional vector stored on Data,
recomputed each step by `mj_fwd_tendon()`. It is available when constraint
assembly runs (after the velocity phase).

#### Files

- `sim/L0/mjcf/src/types.rs` — add `MjcfTendonEquality` struct, add `tendons`
  field to `MjcfEquality`
- `sim/L0/mjcf/src/parser.rs` — parse `<equality><tendon>` elements
- `sim/L0/mjcf/src/model_builder.rs` — process tendon equality in
  `process_equality_constraints()`
- `sim/L0/core/src/mujoco_pipeline.rs` — add `tendon_treenum: Vec<usize>` and
  `tendon_treeid: Vec<[Option<usize>; 2]>` to Model; implement
  `compute_tendon_tree_ids()` in model setup; implement
  `extract_tendon_equality_jacobian()`; update row counting (0 → 1);
  update extraction dispatch; update `equality_trees()` for sleeping;
  update `mj_wake_equality()` for wake propagation
- `sim/L0/tests/integration/` — new tests for tendon equality scenarios
