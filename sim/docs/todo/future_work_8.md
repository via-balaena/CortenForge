# Future Work 8 — Phase 3A: Constraint System Overhaul (Items #28–32)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers the constraint system migration: friction loss from passive forces to
constraint rows, PGS/CG unification, and advanced constraint features.

#28, #29, #30, #31, #32 are all independent. #29 is the largest item (L effort)
and is the core architectural unification. #30 is a small collision filtering
bug in flex-rigid collision.

---

## 28. Friction Loss solref/solimp: Per-DOF Parameters (MuJoCo `dof_solref`)

**Status:** ✅ Complete | **Effort:** S | **Prerequisites:** None

### Background

§15 (Phase C) implemented friction loss as Huber-cost constraint rows in the
Newton path. The following are **already working**:

- `assemble_unified_constraints()` emits DOF friction loss rows (lines
  15435–15456) and tendon friction loss rows (lines 15458–15481)
- `classify_constraint_states()` implements Huber three-state classification
  (lines 15789–15809): Quadratic / LinearPos / LinearNeg
- `newton_solve()` subtracts `qfrc_frictionloss` from RHS (line 17248)
- The `data.newton_solved` branch in force balance accounts for the split
- Force balance identity holds for the Newton path

### Remaining Gap

Friction loss rows currently use `DEFAULT_SOLREF = [0.02, 1.0]` and
`DEFAULT_SOLIMP = [0.9, 0.95, 0.001, 0.5, 2.0]` unconditionally
(lines 15444–15445, 15469–15470). MuJoCo uses **per-DOF** solver parameters
stored in dedicated fields.

### MuJoCo Reference: Field Architecture

**Critical distinction**: MuJoCo uses separate `mjModel` fields for different
constraint types. These are NOT shared:

| mjModel field | Indexed by | MJCF source | Used for |
|---------------|-----------|-------------|----------|
| `dof_solref[nv × 2]` | DOF | `<joint solreffriction="...">` | Friction loss DOF |
| `dof_solimp[nv × 5]` | DOF | `<joint solimpfriction="...">` | Friction loss DOF |
| `jnt_solref[njnt × 2]` | Joint | `<joint solreflimit="...">` | Joint limits |
| `jnt_solimp[njnt × 5]` | Joint | `<joint solimplimit="...">` | Joint limits |
| `tendon_solref_fri[nten × 2]` | Tendon | `<tendon solreffriction="...">` | Friction loss tendon |
| `tendon_solimp_fri[nten × 5]` | Tendon | `<tendon solimpfriction="...">` | Friction loss tendon |
| `tendon_solref_lim[nten × 2]` | Tendon | `<tendon solreflimit="...">` | Tendon limits |
| `tendon_solimp_lim[nten × 5]` | Tendon | `<tendon solimplimit="...">` | Tendon limits |

**No runtime fallback chain.** MuJoCo resolves all defaults at **compile time**
(MJCF compiler). The runtime function `getsolparam()` does a direct read:

```c
// engine_core_constraint.c, getsolparam():
case mjCNSTR_FRICTION_DOF:
    mju_copy(solref, m->dof_solref + mjNREF*id, mjNREF);   // direct read
    mju_copy(solimp, m->dof_solimp + mjNIMP*id, mjNIMP);   // no fallback
    break;

case mjCNSTR_FRICTION_TENDON:
    mju_copy(solref, m->tendon_solref_fri + mjNREF*id, mjNREF);
    mju_copy(solimp, m->tendon_solimp_fri + mjNIMP*id, mjNIMP);
    break;
```

The **compile-time** default chain in MuJoCo's MJCF compiler:
```
mj_defaultSolRefImp()        → [0.02, 1.0] / [0.9, 0.95, 0.001, 0.5, 2.0]
<default><joint> class       → overrides if set
<joint solreffriction="..."> → overrides if set
CopyTree(): m->dof_solref[dof_adr] = final_value  (per-DOF copy)
```

**Ball/Free joints**: All DOFs of a multi-DOF joint get **identical** values:
```c
for (int j1 = 0; j1 < pj->nv(); j1++) {
    mjuu_copyvec(m->dof_solref + mjNREF*dofadr, pj->solref_friction, mjNREF);
    dofadr++;
}
```

### Specification

#### 28.1 Model fields

Add per-DOF and per-tendon solver parameter fields to `Model`, always populated
(no `Option<T>` — defaults resolved at build time):

```rust
// Per-DOF friction loss solver parameters (from <joint solreffriction/solimpfriction>)
pub dof_solref: Vec<[f64; 2]>,    // nv entries, default [0.02, 1.0]
pub dof_solimp: Vec<[f64; 5]>,    // nv entries, default [0.9, 0.95, 0.001, 0.5, 2.0]

// Per-tendon friction loss solver parameters
pub tendon_solref_fri: Vec<[f64; 2]>,  // ntendon entries
pub tendon_solimp_fri: Vec<[f64; 5]>,  // ntendon entries

// Rename existing limit fields for clarity:
// jnt_solref → jnt_solref_lim (already for limits, but rename for disambiguation)
// tendon_solref → tendon_solref_lim
```

The MJCF compiler populates these at build time. Every entry has a concrete
value — no runtime fallback needed.

#### 28.2 Constraint assembly changes

In `assemble_unified_constraints()`, replace the hardcoded `DEFAULT_SOLREF` /
`DEFAULT_SOLIMP` for friction loss rows:

```rust
// DOF friction loss (line ~15444):
let sr = model.dof_solref[dof_idx];   // direct read, no fallback
let si = model.dof_solimp[dof_idx];

// Tendon friction loss (line ~15469):
let sr = model.tendon_solref_fri[t];   // direct read
let si = model.tendon_solimp_fri[t];
```

#### 28.3 MJCF parsing and model building

Add `solreffriction` and `solimpfriction` attribute parsing to:
- `<joint>` element → compiled into `dof_solref` / `dof_solimp` (per-DOF)
- `<tendon>` element → compiled into `tendon_solref_fri` / `tendon_solimp_fri`
- `<default><joint>` and `<default><tendon>` for inheritance

The model builder resolves the default chain:
```rust
// model_builder.rs, for each joint:
let solref_friction = joint.solreffriction
    .or(default_class.solreffriction)
    .unwrap_or(DEFAULT_SOLREF);

// Copy to all DOFs of this joint (ball=3, free=6, hinge/slide=1):
for dof in joint_dof_range {
    model.dof_solref[dof] = solref_friction;
    model.dof_solimp[dof] = solimp_friction;
}
```

Also add `solreflimit` / `solimplimit` parsing for joint limits. MuJoCo uses
**only** `solreflimit` / `solimplimit` (not `solref` / `solimp`) on `<joint>`.
There is no plain `solref` attribute on `<joint>` in MuJoCo — that attribute
exists only on `<geom>`, `<pair>`, and `<equality>`. If our codebase currently
parses `<joint solref="...">`, migrate to `solreflimit` for MuJoCo conformance.
Optionally accept both names during a transition period.

### Acceptance Criteria

1. DOF friction loss rows use `model.dof_solref[dof_idx]` (per-DOF, not hardcoded).
2. Tendon friction loss rows use `model.tendon_solref_fri[t]` (per-tendon).
3. Default values `[0.02, 1.0]` produce identical behavior to current code.
4. `<joint solreffriction="0.05 0.5">` compiles into `dof_solref` for all DOFs
   of that joint and produces different friction loss dynamics.
5. Ball joint: all 3 DOFs get the same `dof_solref` from the parent joint.
6. `<default><joint solreffriction="...">` inheritance works.
7. Existing tests pass (default parameters produce equivalent behavior).
8. New test: hinge with `solreffriction="0.05 0.5"` vs default, verify different
   friction loss constraint forces.

### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — `Model` struct (add `dof_solref`,
  `dof_solimp`, `tendon_solref_fri`, `tendon_solimp_fri`),
  `assemble_unified_constraints()` (use new fields)
- `sim/L0/mjcf/src/model_builder.rs` — compile `solreffriction`/`solimpfriction`
  into per-DOF fields, resolve default chain at build time
- `sim/L0/mjcf/src/types.rs` — add `solreffriction`/`solimpfriction` to
  `MjcfJoint`, `MjcfTendon`, `MjcfJointDefaults`, `MjcfTendonDefaults`
- `sim/L0/tests/integration/newton_solver.rs` — per-DOF solref friction test

### Origin

Discovered during spec review: friction loss rows assembled at lines 15435–15481
use `DEFAULT_SOLREF` instead of per-DOF parameters. MuJoCo stores
friction-specific solver params in `dof_solref`/`dof_solimp` (DOF-indexed),
completely separate from `jnt_solref`/`jnt_solimp` (joint-indexed, for limits).

---

## 29. PGS/CG Unified Constraint Migration

**Status:** ✅ Complete | **Effort:** L | **Prerequisites:** None

### Current State

The Newton solver routes ALL constraint types through unified constraint
assembly (`assemble_unified_constraints()`). The PGS/CG path uses a
fundamentally different — and incorrect — architecture:

| Constraint Type | Newton Path | PGS/CG Path (current) | MuJoCo |
|----------------|-------------|----------------------|--------|
| Equality | Solver rows (unified) | Penalty + Baumgarte | Solver rows |
| Joint limits | Solver rows (unified) | Penalty + Baumgarte | Solver rows |
| Tendon limits | Solver rows (unified) | Penalty + Baumgarte | Solver rows |
| Contacts | Solver rows (unified) | PGS/CG solver | Solver rows |
| Friction loss | Solver rows (Huber) | tanh passive force | Solver rows |
| Flex edges | Solver rows (unified) | Penalty + Baumgarte | Solver rows |

The PGS/CG path has two categories of divergence:

**A. Friction loss as tanh passive force** (instead of solver constraint rows):

```rust
// mujoco_pipeline.rs:11719 (DOF friction loss)
let smooth_sign = (qvel * model.friction_smoothing).tanh();
let fl_force = frictionloss * smooth_sign;
self.data.qfrc_passive[dof_idx] -= fl_force;
self.data.qfrc_frictionloss[dof_idx] -= fl_force;
```

Friction loss is **never** part of `qfrc_passive` in MuJoCo. It is always
constraint rows (`mjCNSTR_FRICTION_DOF`, `mjCNSTR_FRICTION_TENDON`) handled
by the solver. The tanh approximation produces qualitatively different force
profiles near zero velocity (smooth sigmoid vs Huber kink with flat saturation).

**B. Penalty forces for limits and equality** (instead of solver rows):

1. **Joint limits**: Penalty forces at lines 19447–19497
   - `solref_to_penalty()` converts solref to spring-damper `(k, b)`
   - Force = `k * penetration + b * vel_into_limit`
   - Only activated when `q < limit_min` or `q > limit_max` (no margin)

2. **Tendon limits**: Penalty forces at lines 19499–19550

3. **Equality constraints**: `apply_equality_constraints()` at line 14639
   - Connect, Weld, Joint, Distance types via penalty
   - Default penalty params 4× stiffer than MuJoCo (`k=10000` vs effective
     `k≈2500` from `solref=[0.02, 1.0]`)
   - Tendon equality: ✅ implemented (§37, `extract_tendon_equality_jacobian()`)

4. **Flex edge constraints**: `apply_flex_edge_constraints()` at line 19564

**Key divergences from MuJoCo:**

- **Impedance formula**: Penalty applies `F = -imp·k·pos - imp·b·vel` (both
  terms scaled by impedance). MuJoCo KBIP: `aref = -B·vel - K·imp·pos`
  (impedance only on position term).

- **Activation margin**: Penalty limits activate only when violated
  (`q < min` or `q > max`). MuJoCo constraint rows use `solimp` margin to
  activate the constraint BEFORE violation (soft activation zone).

- **No regularization scaling**: Penalty uses raw stiffness. MuJoCo constraint
  rows use `R = (1-imp)/imp · diagApprox` regularization that scales with the
  effective inertia, giving robust behavior across different mass scales.

- **`solref_to_penalty()` formula**: Uses simplified `k = 1/tc²`, `b = 2·dr/tc`
  instead of MuJoCo's `K = 1/(dmax²·tc²·dr²)`, `B = 2/(dmax·tc)` which
  incorporates impedance normalization.

### Objective

Route ALL constraint types through the unified solver for PGS/CG, matching
MuJoCo's architecture. After this item, `assemble_unified_constraints()` is
the single constraint assembly path for ALL solver types. All penalty force
code and the tanh friction loss approximation are removed.

### Specification

#### 29.1 Unified constraint assembly for all solvers

Refactor `mj_fwd_constraint()` so that `assemble_unified_constraints()` is
called for ALL solver types, not just Newton:

```rust
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    // Step 1: Assemble ALL constraints into unified efc_* arrays
    // (This is currently Newton-only. Make it universal.)
    let qacc_smooth = compute_qacc_smooth(model, data);
    assemble_unified_constraints(model, data, &qacc_smooth);

    // Step 2: Dispatch to solver
    match model.solver_type {
        SolverType::Newton => { newton_solve(model, data); }
        SolverType::PGS    => { pgs_solve_unified(model, data); }
        SolverType::CG     => { cg_solve_unified(model, data); }
    }

    // Step 3: Map efc_force → qfrc_constraint via J^T
    compute_qfrc_constraint(model, data);
}
```

The `qacc_smooth` computation (currently at Newton lines 17242–17262) is
factored out since all solvers need it for constraint assembly.

**Constraint assembly must emit rows in order**: equality first, then friction
loss, then limits+contacts. Store `data.ne` (equality count) and `data.nf`
(friction loss count) for solver dispatch. See §29.4 for why ordering matters.

#### 29.2 qacc_smooth shared computation

Factor out `qacc_smooth` from `newton_solve()` so all solver types can use it:

```rust
fn compute_qacc_smooth(model: &Model, data: &Data) -> DVector<f64> {
    let nv = model.nv;
    let mut qfrc_smooth = DVector::zeros(nv);
    for k in 0..nv {
        qfrc_smooth[k] = data.qfrc_applied[k]
            + data.qfrc_actuator[k]
            + data.qfrc_passive[k]
            - data.qfrc_bias[k];
    }
    let mut qacc_smooth = qfrc_smooth;
    mj_solve_sparse(..., &mut qacc_smooth);  // M⁻¹ · qfrc_smooth
    qacc_smooth
}
```

Note: `qfrc_passive` no longer contains friction loss after this item (see
§29.8), so no `qfrc_frictionloss` subtraction is needed.

#### 29.3 Unified PGS solver

Replace `pgs_solve_contacts()` with `pgs_solve_unified()` that handles ALL
constraint types in the unified `efc_*` arrays.

**MuJoCo PGS architecture** (from `engine_solver.c`, `mj_solPGS`):

PGS is a **dual** solver — it operates directly on constraint forces using
the AR matrix. This is fundamentally different from CG/Newton which are
**primal** solvers operating on accelerations.

1. **AR matrix**: MuJoCo computes `AR = J·M⁻¹·J^T + diag(R)` explicitly
   (the regularized Delassus matrix). PGS reads AR during iteration.
2. **Flat iteration with elliptic grouping**: All scalar constraints (equality,
   friction loss, limits, frictionless/pyramidal contacts) are processed
   row-by-row with `dim=1`. Only elliptic contacts are multi-row blocks.
3. **Cost guard**: After each GS update + projection, MuJoCo checks whether
   the dual cost actually decreased. If it increased (by > 1e-10), the old
   force is restored. This makes PGS monotonically non-increasing in cost.
4. **Elliptic two-phase projection**: Normal via ray method, friction via
   QCQP — more sophisticated than simple scale-to-boundary.
5. **Residual**: `res = efc_b + AR · force`. PGS always uses `flg_subR=0`
   (R included in residual). The `flg_subR=1` variant is used only by the
   NoSlip solver (#33).

```rust
fn pgs_solve_unified(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    if nefc == 0 { return; }

    // Build regularized Delassus: AR = J·M⁻¹·J^T + diag(R)
    let AR = compute_delassus_regularized(model, data);

    // Precompute inverse diagonal
    let AR_diag_inv: Vec<f64> = (0..nefc)
        .map(|i| 1.0 / AR[(i,i)])
        .collect();

    // PGS iterations
    for _iter in 0..model.solver_iterations {
        let mut i = 0;
        while i < nefc {
            let dim = if data.efc_type[i] == ConstraintType::ContactElliptic {
                data.efc_dim[i]
            } else {
                1
            };

            // Save old force for cost guard
            let old_force = data.efc_force[i..i+dim].to_vec();

            // Compute residual: res = b + AR * force (rows i..i+dim-1)
            let mut res = vec![0.0; dim];
            for j in 0..dim {
                res[j] = data.efc_b[i+j];
                for c in 0..nefc {
                    res[j] += AR[(i+j, c)] * data.efc_force[c];
                }
            }

            if dim == 1 {
                // Scalar GS update + projection
                data.efc_force[i] -= res[0] * AR_diag_inv[i];
                project_pgs_row(data, i);
            } else {
                // Elliptic contact: two-phase projection
                // Phase 1: ray update for normal
                // Phase 2: QCQP for friction directions
                project_elliptic_pgs(data, &AR, i, dim, &res);
            }

            // Cost guard: revert if dual cost increased
            // cost_change = 0.5 · δ^T · AR_block · δ + δ^T · res
            // where δ = force_new - force_old (the update delta)
            let delta: Vec<f64> = (0..dim)
                .map(|j| data.efc_force[i+j] - old_force[j])
                .collect();
            let cost_change = compute_cost_change(&AR, &delta, &res, i, dim);
            if cost_change > 1e-10 {
                data.efc_force[i..i+dim].copy_from_slice(&old_force);
            }

            i += dim;
        }
    }
}
```

The cost guard is critical for robustness — without it, PGS can oscillate
on over-constrained problems. MuJoCo's PGS is guaranteed monotonically
non-increasing in dual cost.

**Elliptic two-phase projection (`project_elliptic_pgs`) — full algorithm**:

For each elliptic contact with `dim` rows (1 normal + `dim-1` friction):

```rust
fn project_elliptic_pgs(
    data: &mut Data, AR: &DMatrix<f64>, i: usize, dim: usize, res: &[f64]
) {
    let force = &mut data.efc_force;
    let mu = &data.contact_mu;  // per-direction friction coefficients

    // Extract dim×dim diagonal block from AR
    let Athis = AR.slice((i,i), (dim,dim));

    // --- Phase 1: Ray update (scale entire force along ray from origin) ---
    let v: Vec<f64> = force[i..i+dim].to_vec();  // ray = current force

    // Optimal step: x = -v^T · res / (v^T · Athis · v)
    let v1 = Athis * &v;
    let denom = v.dot(&v1);
    if denom >= MINVAL {
        let mut x = -v.dot(res) / denom;

        // Clamp: ensure normal stays non-negative
        if force[i] + x * v[0] < 0.0 {
            x = -force[i] / v[0];  // zeros entire force vector
        }

        // Apply ray update to ALL components (normal + friction)
        for j in 0..dim {
            force[i+j] += x * v[j];
        }
    }

    // --- Guard: if normal ≈ 0, zero friction and return ---
    if force[i] < MINVAL {
        for j in 1..dim { force[i+j] = 0.0; }
        return;
    }

    // --- Phase 2: QCQP friction subproblem ---
    // Build friction sub-QP: min 0.5·v^T·Ac·v + bc^T·v
    //   s.t. Σ(v[j]/mu[j])² ≤ fn²

    // Ac = Athis[1:,1:] (friction-friction sub-block)
    let Ac = Athis.slice((1,1), (dim-1,dim-1));

    // bc = adjusted residual incorporating normal force change
    let oldforce = saved_old_force;  // saved before ray update
    let mut bc = vec![0.0; dim-1];
    for j in 0..dim-1 {
        bc[j] = res[j+1];  // tangential residual
        for k in 0..dim-1 {
            bc[j] -= Ac[(j,k)] * oldforce[1+k];  // remove old friction
        }
        bc[j] += Athis[(j+1, 0)] * (force[i] - oldforce[0]);  // normal change
    }

    // Solve QCQP via Newton on dual Lagrange multiplier
    let (v_friction, active) = match dim - 1 {
        2 => qcqp_2d(&Ac, &bc, mu, force[i]),
        3 => qcqp_3d(&Ac, &bc, mu, force[i]),
        _ => qcqp_nd(&Ac, &bc, mu, force[i], dim-1),
    };

    // Ellipsoidal rescale if constraint active (enforce exact boundary)
    if active {
        let s2: f64 = v_friction.iter().enumerate()
            .map(|(j, &fj)| fj * fj / (mu[j] * mu[j]))
            .sum();
        let s = (force[i] * force[i] / s2.max(MINVAL)).sqrt();
        for j in 0..dim-1 { force[i+1+j] = v_friction[j] * s; }
    } else {
        for j in 0..dim-1 { force[i+1+j] = v_friction[j]; }
    }
}
```

**QCQP solver** (`qcqp_2d` / `qcqp_3d` / `qcqp_nd`): Newton iteration on
the dual Lagrange multiplier λ. The KKT system is `(A_scaled + λI)·y = -b_scaled`
with the constraint `||y||² = r²`. The iteration:

1. Scale to unit-friction space: `y_j = x_j / mu_j`, `A_s = D·A·D`, `b_s = D·b`
2. If unconstrained solution `y = -A_s⁻¹·b_s` satisfies `||y|| ≤ r`, return it
3. Otherwise, iterate: `λ ← λ - φ(λ)/φ'(λ)` where
   `φ(λ) = ||y(λ)||² - r²` and `y(λ) = -(A_s + λI)⁻¹·b_s`
4. `φ'(λ) = -2·y(λ)ᵀ·(A_s + λI)⁻¹·y(λ)` (analytic derivative)
5. For dim=2: closed-form 2×2 inverse. For dim=3: cofactor 3×3 inverse.
   For dim>3: Cholesky factorization.
6. Converge when `|φ| < 1e-10` or `|δλ| < 1e-10` (up to 20 iterations)

#### 29.4 PGS projection rules per constraint type

**Constraint ordering requirement**: MuJoCo orders constraints as:
`[0..ne) = equality`, `[ne..ne+nf) = friction`, `[ne+nf..nefc) = limits + contacts`.
Both PGS and the primal solver (`mj_constraintUpdate_impl`) use `ne` and `nf`
as index boundaries for dispatch — NOT `efc_type`. This ordering is **mandatory**.

Our `assemble_unified_constraints()` must emit rows in this order:
1. Equality constraints (`ne` rows)
2. Friction loss DOF/tendon (`nf` rows)
3. Limits (joint + tendon) + Contacts (any type, interleaved OK)

Within group 3, the relative ordering of limits vs contacts doesn't matter —
they all receive the same non-negativity projection (except elliptic contacts
which are identified by `efc_type`). We must store `ne` and `nf` counts
on `Data` for the solvers.

The projection dispatch per constraint type:

```rust
fn project_pgs_row(data: &mut Data, i: usize) {
    match data.efc_type[i] {
        // Equality: bilateral (unclamped, force can be any real value)
        ConstraintType::Equality | ConstraintType::FlexEdge => {}

        // Friction loss: box clamp [-floss, +floss]
        // MuJoCo: efc_frictionloss stores the friction loss magnitude.
        // The dual of the Huber penalty — force bounded to the box.
        ConstraintType::FrictionDof | ConstraintType::FrictionTendon => {
            let fl = data.efc_frictionloss[i];
            data.efc_force[i] = data.efc_force[i].clamp(-fl, fl);
        }

        // Limits: unilateral (force ≥ 0)
        ConstraintType::LimitJoint | ConstraintType::LimitTendon => {
            data.efc_force[i] = data.efc_force[i].max(0.0);
        }

        // Frictionless contact: unilateral (normal ≥ 0)
        ConstraintType::ContactFrictionless => {
            data.efc_force[i] = data.efc_force[i].max(0.0);
        }

        // Pyramidal facet: non-negative (each facet independent)
        ConstraintType::ContactPyramidal => {
            data.efc_force[i] = data.efc_force[i].max(0.0);
        }

        // Elliptic contact: handled by caller (group projection)
        ConstraintType::ContactElliptic => {
            unreachable!("elliptic contacts projected by caller");
        }
    }
}
```

**Friction loss projection detail**: MuJoCo's PGS uses **box clamping** for
friction loss rows — the dual of the Huber penalty. The force is clamped to
`[-floss, +floss]`. This is a simple box constraint, NOT the Huber three-state
(Quadratic/LinearPos/LinearNeg) from the Newton primal formulation. PGS works
in the dual (force) space, so it projects onto the box `[-floss, +floss]`
which is the dual of the Huber cost. Newton works in the primal (acceleration)
space and needs the three-state classification for the Hessian.

#### 29.5 KBIP formula (MuJoCo reference)

`assemble_unified_constraints()` computes KBIP per constraint row via
`mj_makeImpedance`. Key details for correctness:

**Standard format** (`solref[0] > 0`): interprets as `(timeconst, dampratio)`:
```
dmax = solimp[1]
K = 1 / max(MINVAL, dmax² · timeconst² · dampratio²)
B = 2 / max(MINVAL, dmax · timeconst)
```

**Direct format** (`solref[0] ≤ 0`): interprets as `(-stiffness, -damping)`:
```
K = -solref[0] / max(MINVAL, dmax²)
B = -solref[1] / max(MINVAL, dmax)
```

All denominators are guarded with `max(mjMINVAL, ...)` to prevent division
by zero. `mjMINVAL` is a small positive constant (~1e-15).

**Friction rows** (friction loss DOF/tendon, elliptic friction directions):
**K is ALWAYS zero.** Friction rows have no positional spring, only damping.
`aref = -B · vel` (the K·I·pos term vanishes since K=0 AND pos=0).

**Regularization**:
```
imp = getimpedance(solimp, pos, margin)   // in (0, 1)
R[i] = max(mjMINVAL, (1 - imp) / imp * diagApprox[i])
```

**Reference acceleration**:
```
aref[i] = -B · efc_vel[i] - K · imp · (efc_pos[i] - efc_margin[i])
```

**Friction R adjustment**: For contact friction directions, MuJoCo adjusts:
```
R[friction_row] = R[normal_row] / max(mjMINVAL, impratio)
```
where `impratio` is `model.opt.impratio` (default 1.0). This allows tuning
friction stiffness relative to normal stiffness. Applied unconditionally to
all frictional contacts (both elliptic and pyramidal), independent of
`solreffriction`.

Verify our existing `assemble_unified_constraints()` KBIP computation matches
these formulas exactly. Pay special attention to the `dmax` normalization and
the friction `K=0` rule.

#### 29.6 Unified CG solver (primal, NOT dual)

**Critical architectural note**: MuJoCo's CG solver is a **primal** solver,
NOT a dual solver. `mj_solCG` is a one-line wrapper:

```c
void mj_solCG(const mjModel* m, mjData* d, int maxiter) {
    mj_solPrimal(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/0);
}
```

CG shares the `mj_solPrimal` driver with Newton. It works in **acceleration
space** (`qacc`), not force space. This means CG does NOT use the AR matrix
or PGS-style per-row projection. The relationship is:

| | PGS | CG | Newton |
|---|-----|-----|--------|
| Space | Dual (forces) | **Primal (qacc)** | Primal (qacc) |
| Matrix | AR = J M⁻¹ Jᵀ + R | M, J (no AR) | M, J, Hessian |
| Direction | Row-by-row GS | Polak-Ribiere | -H⁻¹·grad |
| Step size | N/A | Newton line search | Newton line search |
| Preconditioner | N/A | M⁻¹ (via LDL) | H⁻¹ (via Cholesky) |
| Cone handling | QCQP projection | Analytic zone eval | Analytic + Hessian |
| Cost guard | Per-update revert | Line search ensures | Line search ensures |
| Shared infra | Standalone | mj_solPrimal | mj_solPrimal |

Replace `cg_solve_contacts()` with `cg_solve_unified()` that works in primal
space, sharing the `mj_solPrimal` driver with Newton:

```rust
fn cg_solve_unified(model: &Model, data: &mut Data) {
    // CG operates in primal (acceleration) space, NOT dual (force) space.
    // It shares mj_solPrimal with Newton. The differences are:
    // 1. Preconditioner: M⁻¹ (CG) vs H⁻¹ (Newton)
    // 2. Direction: Polak-Ribiere (CG) vs Newton direction (Newton)
    // 3. No Hessian cone (flg_HessianCone = false for CG)

    let nv = model.nv;
    let mut qacc = data.qacc_smooth.clone();  // start from smooth acceleration
    let mut search = DVector::zeros(nv);
    let mut grad = DVector::zeros(nv);
    let mut mgrad = DVector::zeros(nv);       // M⁻¹ · grad

    for iter in 0..model.solver_iterations {
        // 1. Constraint update: compute forces from qacc (primal → dual)
        //    Uses mj_constraintUpdate_impl — same as Newton, but without
        //    cone Hessian (flg_HessianCone = false)
        primal_update_constraint(model, data, &qacc, false);

        // 2. Gradient: grad = M·qacc - qfrc_smooth - qfrc_constraint
        //    This is ∂L/∂qacc of the primal cost function
        let ma = mass_matrix_multiply(model, &qacc);
        for k in 0..nv {
            grad[k] = ma[k] - data.qfrc_smooth[k] - data.qfrc_constraint[k];
        }

        // 3. Precondition: mgrad = M⁻¹ · grad (via LDL solve)
        mgrad = grad.clone();
        mj_solve_sparse(model, &mut mgrad);  // M⁻¹ · grad

        // 4. Polak-Ribiere direction update
        if iter == 0 {
            search = -mgrad.clone();
        } else {
            let mgrad_diff = &mgrad - &mgrad_old;
            let beta = grad.dot(&mgrad_diff)
                / grad_old.dot(&mgrad_old).max(MINVAL);
            let beta = beta.max(0.0);  // reset to steepest descent if negative
            search = -&mgrad + beta * &search;
        }

        // 5. Newton line search along search direction
        //    PrimalSearch: piecewise-quadratic 1D optimization with
        //    analytic derivatives. Returns alpha=0 if no improvement.
        let alpha = primal_search(model, data, &qacc, &search);
        if alpha == 0.0 { break; }

        // 6. Update qacc
        qacc += alpha * &search;

        // 7. Check convergence (improvement + gradient norm)
        let scale = 1.0 / (model.stat_meaninertia * nv.max(1) as f64);
        let improvement = scale * (old_cost - data.solver_cost);
        let gradient = scale * grad.norm();
        if improvement < model.opt_tolerance || gradient < model.opt_tolerance {
            break;
        }

        grad_old = grad.clone();
        mgrad_old = mgrad.clone();
    }
}
```

**Key difference from our current CG**: Our current `cg_solve_contacts()`
works in dual (force) space with the AR matrix. MuJoCo's CG works in primal
(acceleration) space with M and J. This is a fundamental architectural change.

**Line search (`PrimalSearch`)**: Newton-based line search on the 1D cost
function along the search direction. The cost is piecewise-quadratic (each
constraint contributes a quadratic that activates/deactivates at a breakpoint).
The search computes analytic first AND second derivatives, then uses Newton
steps with bisection fallback. This is NOT Barzilai-Borwein.

**Constraint handling in CG**: Constraints are handled by
`primal_update_constraint()` (shared with Newton), which:
1. Computes `jar = aref - J · qacc` (constraint violation)
2. Classifies state: Quadratic/Satisfied/LinearPos/LinearNeg/Cone
3. Computes `force[i] = -D[i] · jar[i]` with clamping per state
4. Computes `qfrc_constraint = J^T · force`

For elliptic cones, this does analytic zone-based projection (top/middle/bottom),
NOT the QCQP used by PGS. CG never calls `mju_QCQP*`.

**Implication**: CG and Newton share ~90% of their infrastructure. The unified
CG implementation should be factored from the Newton solver, parameterized by:
- Preconditioner: M⁻¹ (CG) vs H⁻¹ (Newton)
- Direction: Polak-Ribiere (CG) vs Newton (Newton)
- Hessian cone: off (CG) vs on for elliptic (Newton)

#### 29.7 Regularized Delassus matrix (AR) assembly

MuJoCo computes `AR = J·M⁻¹·J^T + diag(R)` **explicitly** (stored as a full
`nefc × nefc` matrix, or sparse equivalent). This is `efc_AR` in MuJoCo.

Generalize `assemble_contact_system()` to handle all constraint types:

```rust
fn compute_delassus_regularized(model: &Model, data: &Data) -> DMatrix<f64> {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    // Step 1: Compute A = J · M⁻¹ · J^T
    // MuJoCo: B = L⁻¹ · J^T, then A = B^T · B (using Cholesky of M)
    // Our approach: M⁻¹·J^T column by column via sparse LDL solve
    let mut minv_jt = DMatrix::zeros(nv, nefc);
    for col in 0..nefc {
        let j_col = data.efc_J.row(col).transpose();
        let mut x = j_col.clone_owned();
        mj_solve_sparse(..., &mut x);  // x = M⁻¹ · J[col,:]^T
        minv_jt.set_column(col, &x);
    }
    let mut AR = &data.efc_J * &minv_jt;

    // Step 2: Add regularization to diagonal: AR = A + diag(R)
    for i in 0..nefc {
        AR[(i, i)] += data.efc_R[i];
    }

    AR
}
```

**Storage format**: MuJoCo supports both **dense** (`nefc × nefc` full matrix)
and **sparse** (CSR-like with `efc_AR_rownnz`, `efc_AR_rowadr`, `efc_AR_colind`
index arrays) representations of `efc_AR`. The format is selected based on
sparsity heuristics. For initial implementation, dense is sufficient; sparse
can be added as a performance optimization.

**Performance consideration**: For large `nefc`, this is O(nefc · nv²) with
the sparse solve. The existing contact-only system uses the same approach but
only for contact rows. The total row count is typically `ne + nf + nl + nc`
where `ne` (equality) and `nf` (friction loss) and `nl` (limits) are usually
small compared to `nc` (contacts), so the overhead is modest.

**Note on `diagApprox`**: MuJoCo's `efc_diagApprox` approximates the diagonal
of `J·M⁻¹·J^T` cheaply (from body-level inverse inertia estimates). It is used
**only** for computing R: `R[i] = max(mjMINVAL, (1-imp)/imp * diagApprox[i])`.
It is NOT used inside the PGS iteration itself — PGS uses the full AR matrix.
After R is computed, MuJoCo back-adjusts `diagApprox` for consistency:
`diagApprox[i] = R[i] * imp / (1 - imp)`.

#### 29.8 Remove tanh friction loss from mj_fwd_passive

Remove all friction loss computation from `mj_fwd_passive()`:

1. **DOF loop** (line ~11718–11724): Remove the `frictionloss > 0` branch.
   DOF friction loss no longer contributes to `qfrc_passive`.

2. **Tendon loop** (line ~11444–11451): Remove the `fl > 0` branch.
   Tendon friction loss no longer contributes to passive forces.

3. **`qfrc_frictionloss` field**: Keep for diagnostics. After the solve,
   extract friction loss forces from `efc_force`:
   ```rust
   data.qfrc_frictionloss.fill(0.0);
   for i in 0..nefc {
       if matches!(data.efc_type[i], ConstraintType::FrictionDof | ConstraintType::FrictionTendon) {
           for col in 0..nv {
               data.qfrc_frictionloss[col] += data.efc_J[(i, col)] * data.efc_force[i];
           }
       }
   }
   ```

4. **`friction_smoothing` on Model**: Mark as deprecated (dead field). Do not
   remove yet — it's a public API field.

#### 29.9 Force balance update

With friction loss removed from `qfrc_passive`, the force balance simplifies
for ALL solver types:

```
M · qacc = qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias
```

Where `qfrc_constraint` now includes friction loss forces (via `J^T · efc_force`
for friction loss rows). The separate `qfrc_frictionloss` subtraction in
`newton_solve()` (line 17248) must also be updated:

```rust
// Before (current):
qfrc_smooth[k] = qfrc_applied[k] + qfrc_actuator[k]
    + (qfrc_passive[k] - qfrc_frictionloss[k]) - qfrc_bias[k];

// After (this item):
qfrc_smooth[k] = qfrc_applied[k] + qfrc_actuator[k]
    + qfrc_passive[k] - qfrc_bias[k];
// qfrc_frictionloss is no longer in qfrc_passive, so no subtraction needed.
```

#### 29.10 Remove penalty code paths

After the unified solver is working:

1. Remove `solref_to_penalty()` (line ~14749) and all callers
2. Remove `apply_equality_constraints()` (line ~14639) and all sub-functions:
   - `apply_connect_constraint()`
   - `apply_weld_constraint()`
   - `apply_joint_equality_constraint()`
   - `apply_distance_constraint()`
3. Remove penalty joint limit code (lines ~19447–19497)
4. Remove penalty tendon limit code (lines ~19499–19550)
5. Remove `apply_flex_edge_constraints()` penalty path
6. Remove `default_eq_stiffness` and `default_eq_damping` from `Model`
7. Remove `max_constraint_vel` and `max_constraint_angvel` (penalty-specific
   velocity clamping, not used by solver-based constraints)

#### 29.11 Warmstart considerations

MuJoCo warmstarts from **accelerations** (`qacc_warmstart`), not from constraint
forces. The warmstart pipeline in `engine_forward.c`:

1. Save `qacc_warmstart = qacc` at end of `mj_advance()` (carried to next step)
2. At next step: compute `efc_vel = J · qacc_warmstart`, evaluate `jar = aref - efc_vel`
3. Call `mj_constraintUpdate()` to recompute `efc_force` from `jar`
4. Evaluate total dual cost of warmstarted forces
5. If `cost_warm > 0` (worse than zero forces), fall back to cold start:
   `efc_force = 0`, `qfrc_constraint = 0`
6. PGS/CG then runs in-place on `efc_force`

**Key detail**: warmstart is universal — it does NOT key by constraint type.
The cost comparison gates the entire solution (accept all or reject all).
There is no per-constraint-type encode/decode during warmstart.

Our implementation should match this: store `qacc_warmstart` after the solve,
use it for the next step's warmstart initialization.

#### 29.12 diagApprox for non-contact rows

`assemble_unified_constraints()` computes `diagApprox` for each row using
`J·M⁻¹·J^T` diagonal approximation. The same formula applies to all
constraint types (type-agnostic). Verify the existing computation works
correctly for equality and limit rows.

### Acceptance Criteria

1. `assemble_unified_constraints()` is called for ALL solver types.
2. PGS solver handles all constraint types with correct projections:
   - Equality: bilateral (unclamped)
   - Friction loss: box clamp `[-floss, +floss]`
   - Limits: unilateral `max(0, f)`
   - Frictionless contacts: unilateral `max(0, f)`
   - Pyramidal contacts: per-facet `max(0, f)`
   - Elliptic contacts: two-phase (ray + QCQP) group projection
3. CG solver handles all constraint types via primal `mj_constraintUpdate_impl`
   (shared with Newton, `flg_HessianCone=false`). Uses Polak-Ribiere direction,
   M⁻¹ preconditioner, Newton line search.
4. PGS cost guard: after each GS update + projection, revert if dual cost
   increased (by > 1e-10). Verify monotonic decrease on test problems.
5. `mj_fwd_passive()` no longer computes friction loss for any solver type.
6. `qfrc_passive` matches MuJoCo semantics (no friction loss component).
7. `solref_to_penalty()` and all penalty force code removed.
8. `apply_equality_constraints()` and all sub-functions removed.
9. `default_eq_stiffness` / `default_eq_damping` removed from Model.
10. Newton `qfrc_smooth` no longer subtracts `qfrc_frictionloss`.
11. Force balance identity `M·qacc = Σforces` is uniform across all solvers.
12. `qfrc_frictionloss` is still populated (extracted from `efc_force`).
13. `friction_smoothing` field deprecated on Model (dead, not removed).
14. PGS friction loss test: single hinge with `frictionloss=1.0`, PGS solver.
    Compare constraint force at multiple velocities against MuJoCo 3.4.0:
    - `qvel=0.0` → force near 0 (quadratic zone)
    - `qvel=1.0` → force = ±1.0 (saturated)
    - `qvel=0.001` → force proportional to velocity (quadratic zone)
15. Joint limit forces match MuJoCo 3.4.0 PGS within solver tolerance:
    - Test: single hinge with `range="-1.57 1.57"`, position beyond limit,
      PGS solver. Compare `efc_force` against MuJoCo reference.
16. Weld constraint forces match MuJoCo 3.4.0 PGS reference:
    - Test: two bodies welded, gravity applied, PGS solver. Compare
      `efc_force` against MuJoCo reference.
17. Connect constraint forces match MuJoCo 3.4.0 PGS reference.
18. Flex edge constraints work through solver rows (not penalty).
19. All existing Newton tests pass without modification.
20. No regressions on Newton path (already unified).
21. Constraint ordering: `efc_*` arrays ordered as `[equality | friction | limits+contacts]`
    with `data.ne` and `data.nf` counts stored for solver dispatch.
22. CG and Newton share `mj_solPrimal` driver, parameterized by preconditioner
    and direction update method.

### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - `mj_fwd_constraint()` — refactored dispatcher
  - `compute_qacc_smooth()` — factored out from `newton_solve()`
  - `pgs_solve_unified()` — new unified PGS (dual space, AR matrix)
  - `project_pgs_row()` — per-type projection for PGS
  - `project_elliptic_pgs()` — ray + QCQP for elliptic contacts in PGS
  - `qcqp_2d()` / `qcqp_3d()` / `qcqp_nd()` — QCQP solvers (Newton on λ)
  - `mj_sol_primal()` — shared CG/Newton driver (primal space)
  - `primal_update_constraint()` — shared constraint update (from Newton)
  - `primal_search()` — shared Newton line search (from Newton)
  - `compute_delassus_regularized()` — generalized from `assemble_contact_system()`
  - `mj_fwd_passive()` — remove friction loss computation
  - `newton_solve()` — refactored into `mj_sol_primal(flg_Newton=true)`
  - Remove: `solref_to_penalty()`, `apply_equality_constraints()`,
    `apply_connect_constraint()`, `apply_weld_constraint()`,
    `apply_joint_equality_constraint()`, `apply_distance_constraint()`,
    `apply_flex_edge_constraints()`, penalty limit code,
    `pgs_solve_contacts()`, `cg_solve_contacts()`, `assemble_contact_system()`
- `sim/L0/tests/integration/equality_constraints.rs` — update to solver-based
- `sim/L0/tests/integration/passive_forces.rs` — update: friction loss no
  longer in `qfrc_passive`; limit force tests
- `sim/L0/tests/integration/newton_solver.rs` — verify no regression
- `sim/L0/tests/integration/pgs_solver.rs` (new) — unified PGS tests

### Origin

This combines the friction loss migration (§15 Phase C continuation) with
the penalty-to-solver migration (acknowledged in code comments at lines 19452
and 19554 of `mujoco_pipeline.rs`). These were originally tracked as separate
items (#29 and #30) but are merged because:
1. They share the same architectural change (unified assembly for PGS/CG)
2. Implementing friction loss separately would create an intermediate state
   (partially unified PGS) that must be maintained temporarily
3. One clean migration is simpler than two incremental ones

---

## 30. Flex Collision contype/conaffinity Filtering

**Status:** ✅ Complete | **Effort:** S | **Prerequisites:** None

### Bug Description

`mj_collision_flex()` does not perform proper contype/conaffinity bitmask
filtering when testing flex vertices against rigid geoms. The current code
(line ~5564) only skips geoms where `contype=0 AND conaffinity=0` — a
degenerate check that passes for any geom with default collision settings
(`contype=1, conaffinity=1`).

This means flex vertices collide with ALL rigid geoms regardless of collision
group membership. In MuJoCo, flex elements have their own `contype`/`conaffinity`
attributes on `<flex><contact>`, and collision filtering follows the same
bitmask protocol as rigid-rigid: `(flex_contype & geom_conaffinity) != 0 ||
(geom_contype & flex_conaffinity) != 0`.

**Observed symptom:** Integration tests with flex bodies near rigid geoms
on unrelated bodies generate spurious contacts, because the flex collision
broadphase tests every vertex against every geom with no bitmask gate.

### MuJoCo Reference

#### MJCF attribute location

Attributes live on the `<contact>` sub-element of `<flex>`, not on `<flex>`
directly:
```xml
<flex name="cloth">
  <contact contype="1" conaffinity="1" condim="3" margin="0.001"/>
</flex>
```

Defaults: `contype=1, conaffinity=1` (same as geom defaults). Confirmed in
MuJoCo `mjs_defaultFlex()` in `user_init.c`.

#### Flex-rigid collision filtering (`filterBitmask`)

MuJoCo uses a unified `filterBitmask()` function in `engine_collision_driver.c`:

```c
static int filterBitmask(int contype1, int conaffinity1,
                         int contype2, int conaffinity2) {
    // Returns 1 to REJECT (filter out), 0 to ALLOW
    return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}
```

A collision proceeds when:
```
can_collide = (flex_contype & geom_conaffinity) != 0
           || (geom_contype & flex_conaffinity) != 0
```

This is identical to the rigid-rigid protocol in `mj_geomCanCollide()`.

#### Self-collision bitmask gate (required, not optional)

MuJoCo gates ALL flex self-collision behind a self-bitmask check **in addition
to** the `selfcollide` flag. From `mj_collision()` in `engine_collision_driver.c`:

```c
for (int f = 0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
        // internal collisions (adjacent elements sharing vertices/edges)
        if (m->flex_internal[f]) {
            mj_collideFlexInternal(m, d, f);
        }
        // self-collisions (non-adjacent elements)
        if (m->flex_selfcollide[f] != mjFLEXSELF_NONE) {
            // midphase: BVH for dim=3, SAP otherwise (when selfcollide=auto)
        }
    }
}
```

Three conditions gate self-collision conjunctively:
1. `!flex_rigid[f]` — rigid flexes (all vertices on same body) skip entirely
2. `(flex_contype[f] & flex_conaffinity[f]) != 0` — self-bitmask check
3. `selfcollide != NONE` — dedicated self-collision flag

Consequence: setting `contype=2, conaffinity=4` disables self-collision even
when `selfcollide != NONE`, because `2 & 4 = 0`.

**`internal` vs `selfcollide`:** These are independent concepts. `internal`
controls adjacent-element collision (elements sharing vertices/edges).
`selfcollide` controls non-adjacent element collision. Both are independently
gated behind the self-bitmask check.

#### Flex-flex collision filtering

MuJoCo uses a unified bodyflex index space where bodies occupy `[0, nbody)`
and flexes occupy `[nbody, nbody+nflex)`. The `canCollide2()` function
handles cross-object filtering for flex-flex pairs:

```c
static int canCollide2(const mjModel* m, int bf1, int bf2) {
    // Resolve contype/conaffinity from body or flex based on index range
    int contype1  = (bf1 < nbody) ? m->body_contype[bf1]  : m->flex_contype[bf1-nbody];
    int conaffinity1 = ...;
    // ...
    return (!filterBitmask(contype1, conaffinity1, contype2, conaffinity2));
}
```

This means two flex objects with incompatible bitmasks will not collide with
each other. This is a broadphase-level filter.

### Implementation

1. **Add `contype`/`conaffinity` to `MjcfFlex` struct** (`types.rs:3330–3349`)
   ```rust
   pub contype: Option<i32>,      // Collision type bitmask
   pub conaffinity: Option<i32>,  // Collision affinity bitmask
   ```

2. **Parse in `parse_flex_contact_attrs()`** (`parser.rs:2570–2618`)
   ```rust
   flex.contype = parse_int_attr(e, "contype");
   flex.conaffinity = parse_int_attr(e, "conaffinity");
   ```
   Follow exact pattern from geom parsing at `parser.rs:1646–1647`.

3. **Add builder arrays** (`model_builder.rs:613–642`)
   ```rust
   flex_contype: Vec<u32>,
   flex_conaffinity: Vec<u32>,
   ```
   Push with `unwrap_or(1) as u32` default (same as `geom_contype` at
   `model_builder.rs:1655–1657`).

4. **Add Model fields** (`mujoco_pipeline.rs:1389–1455`)
   ```rust
   pub flex_contype: Vec<u32>,
   pub flex_conaffinity: Vec<u32>,
   ```

5. **Fix `mj_collision_flex()` bitmask check** (replace lines 5564–5567):
   ```rust
   // Proper contype/conaffinity bitmask filtering (matches rigid-rigid protocol)
   let flex_contype = model.flex_contype[flex_id];
   let flex_conaffinity = model.flex_conaffinity[flex_id];
   let geom_contype = model.geom_contype[gi];
   let geom_conaffinity = model.geom_conaffinity[gi];
   if (flex_contype & geom_conaffinity) == 0 && (geom_contype & flex_conaffinity) == 0 {
       continue;
   }
   ```

6. **Add self-collision bitmask gate** — deferred to §42A-iv (flex
   self-collision dispatch). The gate code:
   ```rust
   if !model.flex_rigid[flex_id]
       && (model.flex_contype[flex_id] & model.flex_conaffinity[flex_id]) != 0
       && model.flex_selfcollide[flex_id]
   {
       // proceed with self-collision
   }
   ```
   Requires §42A-ii (`flex_rigid` fields) as prerequisite.

7. **Flex-flex filtering** — deferred to §42A-v (flex-flex cross-object
   collision). Applies `filterBitmask()` between two flex objects'
   contype/conaffinity at the broadphase level.

### Files Modified

- `sim/L0/mjcf/src/types.rs`: Add `contype`/`conaffinity` to `MjcfFlex` (lines 3330–3349)
- `sim/L0/mjcf/src/parser.rs`: Parse in `parse_flex_contact_attrs()` (lines 2570–2618)
- `sim/L0/mjcf/src/model_builder.rs`: Add builder arrays + push logic (lines 613–642, 2899–3098)
- `sim/L0/core/src/mujoco_pipeline.rs`: Model fields (lines 1389–1455) + `mj_collision_flex()` fix (lines 5564–5568) + self-collision gate
- `sim/L0/tests/integration/flex_unified.rs`: New bitmask filtering tests

### Acceptance Criteria

- AC1: `mj_collision_flex()` uses proper bitmask filtering matching rigid-rigid `filterBitmask()` protocol
- AC2: Flex vertices with `contype=0` do not collide with any geom
- AC3: Geoms with `conaffinity=0` do not collide with any flex vertex
- AC4: Custom bitmask groups work (e.g. `contype=2` flex vs `conaffinity=2` geom collides; `contype=2` flex vs `conaffinity=4` geom does not)
- AC5: Default behavior unchanged (both default to 1, so default models still collide)
- AC6: Self-collision disabled when `(flex_contype & flex_conaffinity) == 0`, even if `selfcollide` flag is set
- AC7: Rigid flexes (`flex_rigid=true`) skip all self/internal collision regardless of bitmask
- AC8: MJCF `<flex><contact contype="..." conaffinity="..."/>` parsed correctly with default=1

---

## 31. `solreffriction` Per-Direction Solver Parameters (Elliptic Contacts Only)

**Status:** ✅ Complete | **Effort:** M | **Prerequisites:** None

### Current State

The `ContactPair` struct has a `solreffriction: [f64; 2]` field (line ~1851)
that is **parsed and stored** from `<pair>` definitions, but **never propagated
to the runtime `Contact` struct** and **never applied** during constraint
assembly. The runtime `Contact` struct (line ~1865) has no `solreffriction`
field — only `solref`. Comment at line ~6038 in `apply_pair_overrides()`:
"solreffriction is NOT applied here — Contact has a single solref field;
per-direction solver params require solver changes (see §31)."

All contact constraint rows (normal, tangent, torsional, rolling) currently use
the same `solref` for computing `aref`. MuJoCo supports separate
`solreffriction` parameters for friction directions.

### MuJoCo Reference

**Critical scope constraint**: `solreffriction` applies **only** to elliptic
contact friction rows. It does NOT apply to pyramidal contacts, friction loss
DOF/tendon, or any other constraint type.

#### Storage

- `mjModel.pair_solreffriction[npair × 2]` — only on explicit `<pair>` contacts
- `mjContact.solreffriction[2]` — runtime contact field
- **There is no `geom_solreffriction` in `mjModel`.** Auto-generated contacts
  (from `<geom>` collision) always have `solreffriction = [0, 0]`.
- **There is no `solimpfriction` for contacts.** The `solimp` is shared between
  normal and friction rows.

#### Selection logic (`mj_makeImpedance` in `engine_core_constraint.c`)

```c
// The selection is inside the per-row KBIP loop:
int elliptic_friction = (tp == mjCNSTR_CONTACT_ELLIPTIC) && (j > 0);
mjtNum* ref = elliptic_friction && (solreffriction[0] || solreffriction[1])
              ? solreffriction : solref;
```

Both conditions must hold:
1. `tp == mjCNSTR_CONTACT_ELLIPTIC` — constraint type must be elliptic
2. `j > 0` — must NOT be the normal row (j=0 is normal)

The fallback check uses C truthiness: `solreffriction[0] || solreffriction[1]`
— if EITHER component is nonzero, use `solreffriction`.

#### Effect on KBIP

`solreffriction` affects **only the B (damping) term** for friction rows:

```
K = 0                                                   // ALWAYS zero for friction rows
B = 2 / max(mjMINVAL, dmax · solreffriction[0])        // standard format
```

Since K=0 and `efc_pos=0` for friction rows:
```
aref_friction = -B_friction · vel_friction
```

#### Effect on R (regularization)

**`solreffriction` does NOT affect R.** The R adjustment for friction rows is
independent of `solreffriction` and happens in a separate loop within
`mj_makeImpedance()`. Regularization depends only on `solimp` and `diagApprox`,
which are shared between normal and friction rows:
```
R[i] = max(mjMINVAL, (1-imp) * diagApprox[i] / imp)
```

MuJoCo then adjusts friction R via `impratio` **unconditionally** (regardless
of whether `solreffriction` is specified):
```
R[friction_row] = R[normal_row] / max(mjMINVAL, impratio)
```

MuJoCo also adjusts the friction coefficient for regularized cones:
```
mu_regularized = friction[0] * sqrt(R[friction] / R[normal])
```

And for higher friction dimensions, maintains the invariant:
```
R[j] * mu[j]^2 = R[1] * mu[1]^2    for j = 1..dim-1
```

This ensures consistent friction force weighting across all friction
directions despite potentially different friction coefficients.

#### Validation

MuJoCo validates `solreffriction` similarly to `solref`, but with a different
fallback on failure:

- **Mixed sign check**: `(solreffriction[0] > 0) ^ (solreffriction[1] > 0)`
  triggers `mju_warning` and resets to `[0, 0]` (sentinel for "use solref").
  Note: mixed `solref` resets to `mj_defaultSolRefImp()` defaults (`[0.02, 1.0]`),
  but mixed `solreffriction` resets to `[0, 0]` — the behaviors differ.
- **REFSAFE**: `solreffriction[0] = max(solreffriction[0], 2*timestep)` for
  standard format (only when `solreffriction[0] > 0`). Skipped when
  `solreffriction = [0, 0]` since `0 > 0` is false.

### Specification

#### 31.1 MJCF parsing + runtime propagation

`solreffriction` is only available on `<pair>`, not on `<geom>`:

- `<pair solreffriction="0.05 1"/>` → `ContactPair.solreffriction` in model →
  `Contact.solreffriction` at runtime
- Auto-generated contacts: `solreffriction = [0.0, 0.0]` always
- `<default><pair solreffriction="..."/>` inheritance works

**Step 1 — Add field to `Contact`**: The runtime `Contact` struct currently
lacks `solreffriction`. Add `pub solreffriction: [f64; 2]` (default `[0.0, 0.0]`).

**Step 2 — Propagate in `apply_pair_overrides()`**: Replace the existing
"NOT applied" comment (line ~6038) with actual propagation:
```rust
contact.solreffriction = pair.solreffriction;
```
Auto-generated (non-pair) contacts keep the default `[0.0, 0.0]`.

MuJoCo's collision driver (`engine_collision_driver.c`) handles this:
```c
// Dynamic (non-pair) contacts: always [0, 0]
mjtNum solreffriction[mjNREF] = {0};

// Pair contacts: conditionally copied only if nonzero
if (m->pair_solreffriction[mjNREF*ipair] ||
    m->pair_solreffriction[mjNREF*ipair + 1]) {
    mju_copy(solreffriction, m->pair_solreffriction+mjNREF*ipair, mjNREF);
}
```

If `<geom solreffriction>` is currently parsed, it should be accepted
syntactically but only `<pair>` values are compiled into the model (matching
MuJoCo's architecture where only `pair_solreffriction` exists).

#### 31.2 Constraint assembly — KBIP computation

In `assemble_unified_constraints()`, the contact assembly loop (section 3f,
line ~14407) iterates `for r in 0..dim` and passes `contact.solref` to the
`finalize_row!` macro for every row. Split the solref for **elliptic friction
rows only**:

```rust
for r in 0..dim {
    // ...existing J, pos, vel computation...

    // §31: select solref for this row
    let is_elliptic_friction = is_elliptic && r > 0;  // r=0 is normal
    let sr = if is_elliptic_friction
        && (contact.solreffriction[0] != 0.0 || contact.solreffriction[1] != 0.0) {
        contact.solreffriction
    } else {
        contact.solref
    };

    finalize_row!(sr, si, pos, margin_r, vel, 0.0, ctype, dim, ci, contact.mu);
}
```

**Pyramidal contacts are NOT affected** — they use `solref` for all facet rows.

#### 31.3 Impact on Newton Hessian

The Newton solver's Hessian operates per-row. Since `solreffriction` changes
B for friction rows (and B enters the `aref` computation, which enters the
RHS), the Newton solver automatically picks up the different friction damping.

**R is NOT affected** by `solreffriction` — regularization uses `solimp` only.
The Newton Hessian's diagonal contribution from R is unchanged.

#### 31.4 Impact on PGS/CG

PGS and CG use the `efc_b` vector which incorporates `aref`. Since `aref` is
computed per-row during constraint assembly, PGS/CG automatically pick up
`solreffriction` without modification (after #29 unifies the assembly).

### Acceptance Criteria

1. `<pair solreffriction="0.05 1">` on an explicit contact pair produces
   different friction `aref` (damping) than the normal direction's `solref`.
2. `solreffriction="0 0"` (default) matches current behavior (no regression).
3. `aref` for normal row uses `solref`; `aref` for elliptic friction rows uses
   `solreffriction` when nonzero.
4. `solreffriction` has NO effect on pyramidal contacts (all facets use `solref`).
5. `solreffriction` does NOT change R (regularization) for friction rows.
6. K=0 for friction rows regardless of `solreffriction` value.
7. Newton solver: friction forces change when `solreffriction` differs from
   `solref`.
8. PGS/CG solver: friction forces change when `solreffriction` differs from
   `solref`.
9. Test: explicit `<pair>` with `solref="0.02 1" solreffriction="0.1 0.5"`.
   Compare normal and friction `efc_aref` — they must differ (B values differ).
10. MuJoCo 3.4.0 reference comparison: identical model with `<pair>`, verify
    `efc_aref` for friction rows matches within tolerance.
11. Validation: mixed-sign `solreffriction` triggers warning and resets to
    `[0, 0]`.

### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - `Contact` struct — add `solreffriction: [f64; 2]` field (default `[0.0, 0.0]`)
  - `apply_pair_overrides()` — propagate `ContactPair.solreffriction` → `Contact`
  - `assemble_unified_constraints()` section 3f — split solref for elliptic friction rows
  - `finalize_row!` / `compute_kbip()` — use `solreffriction` for B when applicable
- `sim/L0/mjcf/src/model_builder.rs` — verify `solreffriction` on `<pair>`
- `sim/L0/mjcf/src/types.rs` — verify field exists on parsed pair types
- `sim/L0/tests/integration/newton_solver.rs` — solreffriction test
- `sim/L0/tests/integration/contact_solver.rs` — PGS/CG solreffriction test

---

## 32. Pyramidal Friction Cones

**Status:** ✅ Complete (AC12/AC13 cross-validated against MuJoCo 3.5.0) | **Effort:** L | **Prerequisites:** #29 (✅ complete), #31 (✅ complete)

### Current State

`Model.cone` is stored (0=pyramidal, 1=elliptic, line ~1771) and wired from
`<option cone="..."/>` via `set_options()` (§31 fix). The default is `cone: 0`
(pyramidal, MuJoCo default).

When `cone=0`, contacts are classified as `ContactNonElliptic` and handled as
scalar unilateral constraints (Quadratic/Satisfied) in `classify_constraint_states`
(line ~15171). Newton runs normally — there is no cone-specific fallback; the
general Newton failure fallback (Cholesky/max-iter) at line ~17760 falls back
to PGS regardless of cone type.

There is no actual pyramidal cone implementation — `ContactNonElliptic` treats
each contact row as an independent unilateral constraint without the pyramidal
facet Jacobian structure. MuJoCo supports both; pyramidal cones linearize the
friction constraint into facets with `2*(dim-1)` rows per contact.

### MuJoCo Reference

#### Row counts per contact

| condim | Elliptic rows | Pyramidal rows | Pyramidal facets |
|--------|--------------|----------------|------------------|
| 1 | 1 | 1 | (frictionless, no difference) |
| 3 | 3 | 4 | 4 = 2 tangent directions × 2 signs |
| 4 | 4 | 6 | 6 = 3 directions × 2 signs |
| 6 | 6 | 10 | 10 = 5 directions × 2 signs |

General formula for condim > 1: pyramidal rows = 2 × (condim − 1).

#### Pyramidal Jacobian structure

For condim=3 with friction coefficient μ and tangent directions t₁, t₂, the
elliptic Jacobian has 3 rows:

```
J_elliptic = [J_normal; J_t1; J_t2]     // 3 × nv
```

The pyramidal Jacobian replaces these with 4 facet rows, each combining
normal and tangent contributions:

```
J_pyr[0] = J_normal + μ · J_t1    // facet: +t1
J_pyr[1] = J_normal - μ · J_t1    // facet: -t1
J_pyr[2] = J_normal + μ · J_t2    // facet: +t2
J_pyr[3] = J_normal - μ · J_t2    // facet: -t2
```

For condim=4 (with torsional friction μ_t):
```
J_pyr[0..3] = as above for tangent directions
J_pyr[4] = J_normal + μ_t · J_torsion    // facet: +torsion
J_pyr[5] = J_normal - μ_t · J_torsion    // facet: -torsion
```

For condim=6 (with rolling friction μ_r1, μ_r2):
```
J_pyr[0..5] = as above
J_pyr[6] = J_normal + μ_r1 · J_roll1
J_pyr[7] = J_normal - μ_r1 · J_roll1
J_pyr[8] = J_normal + μ_r2 · J_roll2
J_pyr[9] = J_normal - μ_r2 · J_roll2
```

#### Force recovery

Physical contact forces are recovered from pyramidal facet forces:

```
f_normal = Σ f_facet[k]                    // sum of all facets
f_t1 = μ · (f_facet[0] - f_facet[1])      // tangent 1
f_t2 = μ · (f_facet[2] - f_facet[3])      // tangent 2
f_torsion = μ_t · (f_facet[4] - f_facet[5])  // torsional
// etc.
```

#### PGS/CG projection

Pyramidal facets are independent non-negative constraints:
```
f_facet[k] = max(0, f_facet[k])    for all k
```

No coupled cone projection needed. This is the key advantage of pyramidal
cones — each row is an independent box constraint, making PGS trivially
correct without needing `project_elliptic_cone()`.

#### Newton solver

MuJoCo's Newton solver **does execute** with pyramidal contacts but without
cone Hessian modifications. The key line:

```c
PrimalUpdateConstraint(&ctx, flg_Newton & (m->opt.cone == mjCONE_ELLIPTIC));
```

The `flg_HessianCone` parameter is `true` only for elliptic cones. For
pyramidal contacts:
- State is binary: `SATISFIED` (if `jar ≥ 0`) or `QUADRATIC` (if `jar < 0`)
- Never enters `CONE` state (that's elliptic-only)
- The HessianCone update is skipped
- Facets are treated as independent limit-like unilateral constraints

**Decision**: Match MuJoCo behavior. Newton already runs regardless of cone
type (no cone-specific fallback exists). `ContactNonElliptic` is already
handled as Quadratic/Satisfied in `classify_constraint_states` (line ~15171).
The new `ContactPyramidal` variant should use the same Quadratic/Satisfied
classification — each facet is a unilateral constraint with no Cone state.

#### R diagonal scaling for pyramidal facets

MuJoCo applies a special R scaling for pyramidal facet rows in
`mj_makeImpedance()`:

```c
// First: compute regularized mu (same loop that handles elliptic)
d->contact[id].mu = friction[0] * mju_sqrt(R[i+1] / R[i]);
// Where R[i+1] = R[i] / impratio, so:
//   contact.mu = friction[0] * sqrt(1 / impratio)

// Then: for pyramidal contacts
Rpy = 2 * d->contact[id[i]].mu * d->contact[id[i]].mu * R[i];

// assign Rpy to ALL pyramidal R
for (int j = 0; j < 2*(dim-1); j++) {
    R[i+j] = Rpy;
}
```

Key details:
- **`contact.mu` is the regularized friction coefficient**, NOT `friction[0]`.
  It is `friction[0] · √(1/impratio)`, computed inside `mj_makeImpedance()`
  during the R adjustment loop. When `impratio=1` (default), `contact.mu = friction[0]`.
- Expanding: `Rpy = 2 · friction[0]² · R[i] / impratio`
- `R[i]` at this point is the impedance computed from solref/solimp for the
  contact's normal direction (before the pyramidal scaling).
- `Rpy` is assigned to **ALL** `2*(dim-1)` facet rows, including the first
  one. There is no separate normal row in pyramidal — `i` points to the
  first facet row.
- The factor `2·μ²` comes from matching the friction-direction diagonal of the
  equivalent elliptic model. Must be implemented for correct conditioning.

#### solreffriction interaction

`solreffriction` is **NOT used for pyramidal contacts**. All pyramidal facets
use the same `solref` as the normal direction. Only elliptic friction rows
use `solreffriction`.

### Specification

#### 32.1 Constraint row assembly

In `assemble_unified_constraints()`, where contact rows are currently emitted
(the contact section), branch on `model.cone`:

```rust
if model.cone == 0 {
    // Pyramidal: emit 2*(condim-1) facet rows per contact
    emit_pyramidal_contact_rows(model, data, contact, &mut row);
} else {
    // Elliptic: emit condim rows per contact (existing)
    emit_elliptic_contact_rows(model, data, contact, &mut row);
}
```

#### 32.2 `emit_pyramidal_contact_rows()`

```rust
fn emit_pyramidal_contact_rows(
    model: &Model, data: &mut Data, contact: &Contact, row: &mut usize
) {
    let J_normal = contact.frame.row(0) * contact.jacobian; // 1×nv
    let dim = contact.dim;

    if dim == 1 {
        // Frictionless: single normal row (same as elliptic)
        emit_unilateral_row(J_normal, ...);
        return;
    }

    // For each friction direction d (indices 1..dim-1):
    for d in 1..dim {
        let J_friction = contact.frame.row(d) * contact.jacobian; // 1×nv
        let mu_d = contact.mu[d-1];

        // Positive facet: J_normal + μ_d · J_friction
        let J_pos = &J_normal + mu_d * &J_friction;
        emit_pyramidal_facet_row(J_pos, ...);

        // Negative facet: J_normal - μ_d · J_friction
        let J_neg = &J_normal - mu_d * &J_friction;
        emit_pyramidal_facet_row(J_neg, ...);
    }
}
```

MuJoCo's exact Jacobian construction (from `mj_instantiateContact`):
```c
// pos = dist, margin = includemargin — set ONCE before loop
cpos[0] = cpos[1] = con->dist;
cmargin[0] = cmargin[1] = con->includemargin;

for (int k = 1; k < con->dim; k++) {
    // J_facet+ = J_normal + mu[k-1] * J_friction_k
    mju_addScl(jacdifp,      jac, jac + k*NV, con->friction[k-1], NV);
    // J_facet- = J_normal - mu[k-1] * J_friction_k
    mju_addScl(jacdifp + NV, jac, jac + k*NV, -con->friction[k-1], NV);
    mj_addConstraint(m, d, jacdifp, cpos, cmargin, 0,
                     2, mjCNSTR_CONTACT_PYRAMIDAL, i, ...);
}
```

Each pair of pyramidal facet rows has:
- `efc_type = ConstraintType::ContactPyramidal` (new variant)
- `efc_pos = contact.dist` for ALL facet rows (set once before loop, reused)
- `efc_margin = contact.includemargin` for ALL facet rows
- Unilateral classification: `jar < 0 → Quadratic`, `jar ≥ 0 → Satisfied`
- MuJoCo emits facets in pairs via `mj_addConstraint(..., 2, ...)` where `2`
  is `ncon` (rows per call). Each friction direction produces one pair call.
- **No separate normal row** — the normal Jacobian is embedded in every facet.

#### 32.3 New ConstraintType variant

Current enum (line ~868):
```rust
pub enum ConstraintType {
    Equality,
    FrictionLoss,        // DOF + tendon friction (combined)
    LimitJoint,
    LimitTendon,
    ContactNonElliptic,  // currently: frictionless + pyramidal fallback
    ContactElliptic,
    FlexEdge,
}
```

Add `ContactPyramidal` and split `ContactNonElliptic` into
`ContactFrictionless` + `ContactPyramidal`, matching MuJoCo's `mjtConstraint`:

```rust
// MuJoCo correspondence:
// mjCNSTR_EQUALITY            → Equality
// mjCNSTR_FRICTION_DOF        → FrictionDof
// mjCNSTR_FRICTION_TENDON     → FrictionTendon
// mjCNSTR_LIMIT_JOINT         → LimitJoint
// mjCNSTR_LIMIT_TENDON        → LimitTendon
// mjCNSTR_CONTACT_FRICTIONLESS → ContactFrictionless
// mjCNSTR_CONTACT_PYRAMIDAL   → ContactPyramidal
// mjCNSTR_CONTACT_ELLIPTIC    → ContactElliptic

pub enum ConstraintType {
    Equality,
    FrictionDof,         // was FrictionLoss
    FrictionTendon,      // NEW: separate from DOF friction
    LimitJoint,
    LimitTendon,
    ContactFrictionless, // was ContactNonElliptic (condim=1 or mu≈0)
    ContactPyramidal,    // NEW: pyramidal friction cone facet
    ContactElliptic,
    FlexEdge,
}
```

Note: `FrictionLoss` → `FrictionDof` + `FrictionTendon` split is optional for
§32 but recommended for MuJoCo enum parity. Can be deferred to a separate
cleanup task if preferred.

Pyramidal facets use limit-like classification (Quadratic/Satisfied), not
the elliptic Cone state. In PGS: `f = max(0, f)`. In CG projection: same.

#### 32.4 PGS/CG projection for pyramidal

In the unified PGS (#29, ✅ complete), add a match arm:

```rust
ConstraintType::ContactPyramidal => {
    // Each facet is independently non-negative
    data.efc_force[i] = data.efc_force[i].max(0.0);
    i += 1;
}
```

No coupled projection needed. This replaces the elliptic cone projection
for pyramidal contacts.

#### 32.5 Newton with pyramidal (simplified classification)

MuJoCo's Newton solver runs with pyramidal cones using simplified state
classification: `Quadratic/Satisfied` only (never `Cone`). Our Newton solver
already runs regardless of cone type (no cone-specific fallback exists), and
`classify_constraint_states()` already handles `ContactNonElliptic` with
Quadratic/Satisfied at line ~15171. Add `ContactPyramidal` to the same arm:

```rust
ConstraintType::LimitJoint
| ConstraintType::LimitTendon
| ConstraintType::ContactNonElliptic
| ConstraintType::ContactPyramidal => {
    // jar < 0 → Quadratic; jar >= 0 → Satisfied
    ...
}
```

No HessianCone modification for pyramidal facets. Each facet is treated as
an independent unilateral constraint (like a limit).

#### 32.6 Force recovery for output

After solving, recover physical contact forces from pyramidal facet forces
for `data.contact_force` output:

```rust
fn recover_pyramidal_forces(data: &Data, contact_idx: usize) -> ContactForce {
    let base = efc_contact_offset[contact_idx];
    let dim = contact.dim;
    let n_facets = 2 * (dim - 1);

    let mut f_normal = 0.0;
    for k in 0..n_facets {
        f_normal += data.efc_force[base + k];
    }

    let mut f_friction = vec![0.0; dim - 1];
    for d in 0..(dim - 1) {
        let f_pos = data.efc_force[base + 2*d];
        let f_neg = data.efc_force[base + 2*d + 1];
        f_friction[d] = contact.mu[d] * (f_pos - f_neg);
    }

    ContactForce { normal: f_normal, friction: f_friction }
}
```

The inverse operation (`encode_pyramid`, for converting physical forces to
facet forces) distributes the normal force equally across pairs:
```
a = f_normal / (dim - 1)          // normal force per pair
b = min(a, f_friction[d] / mu[d]) // friction contribution
pyramid[2*d]     = 0.5 * (a + b)  // positive facet
pyramid[2*d + 1] = 0.5 * (a - b)  // negative facet
```

This is a user-facing utility (`mju_encodePyramid`), not used internally by
MuJoCo's solver. Implement only if needed for API compatibility.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-84.

#### 32.7 `nefc` sizing

The total constraint count changes based on cone type:

```rust
fn count_contact_rows(contact: &Contact, cone: u8) -> usize {
    if contact.dim == 1 { return 1; }  // frictionless
    if cone == 0 {  // pyramidal
        2 * (contact.dim - 1)
    } else {  // elliptic
        contact.dim
    }
}
```

This affects `efc_J` matrix allocation, `efc_force` vector sizing, etc.

#### 32.8 KBIP for pyramidal facets

**Critical: pyramidal facets have NONZERO K.** Unlike elliptic friction rows
(which get K=0), pyramidal facets are NOT in the K=0 condition:

```c
// K=0 condition in mj_makeImpedance — pyramidal NOT included:
if (tp == mjCNSTR_FRICTION_DOF || tp == mjCNSTR_FRICTION_TENDON
    || elliptic_friction) {
    KBIP[4*(i+j)] = 0;
}
// mjCNSTR_CONTACT_PYRAMIDAL falls through to normal K computation
```

This makes physical sense: each pyramidal facet row is `J_normal ± μ·J_friction`,
embedding the normal direction. A nonzero K produces a positional restoring
force during penetration via `K · imp · efc_pos`.

All pyramidal facet rows share identical KBIP values because:
- Same `efc_pos = contact.dist` for all facets
- Same `solref` and `solimp` (solreffriction not used for pyramidal)
- Same `imp` value (computed from shared solimp and pos)

Each pyramidal facet row computes its own `aref`:
```
aref_facet = -B · efc_vel_facet - K · imp · (efc_pos_facet - efc_margin_facet)
```

Where `efc_vel_facet = J_facet · qvel` (facet row's velocity projection).
Since `J_facet = J_normal ± μ·J_friction`, the facet velocity combines normal
and friction velocity components. `K ≠ 0` and `efc_pos = contact.dist ≠ 0`
during penetration, so both terms contribute.

### Acceptance Criteria

1. `<option cone="pyramidal"/>` uses linearized friction (no warning).
2. `<option cone="elliptic"/>` (default) unchanged (no regression).
3. Pyramidal condim=3: 4 rows per contact, condim=4: 6 rows, condim=6: 10 rows.
4. PGS/CG with pyramidal cones: each facet force non-negative after projection.
5. Force recovery: `f_normal = Σfacets`, `f_friction = μ·(f_pos - f_neg)`.
6. R scaling: pyramidal facet R = `2·μ_reg²·R_normal` where `μ_reg = friction[0]·√(1/impratio)` (matching MuJoCo). When `impratio=1`: `R = 2·friction[0]²·R_normal`.
7. All facet rows share `efc_pos = con.dist` and `efc_margin = con.includemargin`.
8. Newton with pyramidal: runs with Quadratic/Satisfied classification
   (no Cone state, no HessianCone). No cone-specific fallback needed.
9. State classification: pyramidal facets never enter `Cone` state.
10. `solreffriction` has NO effect on pyramidal contacts.
    (Already verified by §31 test `test_s31_pyramidal_ignores_solreffriction`.)
11. Pyramidal facets have NONZERO K (unlike elliptic friction rows which get K=0).
    Verify `aref` includes both `-B·vel` and `-K·imp·pos` terms for each facet.
12. Test: flat ground contact, condim=3, pyramidal vs elliptic. Total normal
    force and friction force magnitude match MuJoCo 3.4.0 within 5%.
13. Test: box on inclined plane, condim=3, pyramidal. Sliding friction force
    direction matches MuJoCo within 5%.
14. `ConstraintType::ContactPyramidal` variant added and handled by all solvers.
15. `mju_decodePyramid` equivalent implemented for force recovery (converting
    facet forces to physical normal+friction forces for user output).
    Note: `mju_encodePyramid` is a user-facing utility in MuJoCo, NOT used
    internally for warmstart. MuJoCo warmstarts from accelerations
    (`qacc_warmstart`), not from pyramid-encoded forces. Implement
    `decode_pyramid` for output; `encode_pyramid` only if needed for API
    compatibility.

### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - `ConstraintType` enum (line ~868) — add `ContactPyramidal`, optionally split `ContactNonElliptic`
  - `assemble_unified_constraints()` section 3f (line ~14422) — pyramidal contact row emission
  - `classify_constraint_states()` (line ~15171) — add `ContactPyramidal` to Quadratic/Satisfied arm
  - PGS projection (line ~14636) — non-negativity for pyramidal facets
  - CG projection — same non-negativity treatment
  - Noslip postprocessor (line ~16891) — handle pyramidal facets
  - R scaling — pyramidal `Rpy = 2·μ_reg²·R_normal` after `compute_regularization`
  - Force recovery — new `decode_pyramid()` function for physical force output
  - Model default `cone: 1` (line ~3108) — change to `cone: 0` (MuJoCo default)
- `sim/L0/tests/integration/unified_solvers.rs` — pyramidal cone tests (PGS, CG, Newton)
- `sim/L0/tests/integration/newton_solver.rs` — Newton + pyramidal classification tests
