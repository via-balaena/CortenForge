# Future Work 8 — Phase 3A: Constraint System Overhaul (Items #28–32)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers the constraint system migration: friction loss from passive forces to
constraint rows, PGS/CG unification, and advanced constraint features.

Items #28→#29→#30 form a strict dependency chain.

---

### 28. Friction Loss Migration: Tanh Passive Force → Huber Constraint Rows
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

CortenForge computes friction loss as a **passive force** in `mj_fwd_passive()`:

```rust
// mujoco_pipeline.rs:10922
let smooth_sign = (qvel * self.model.friction_smoothing).tanh();
let fl_force = frictionloss * smooth_sign;
self.data.qfrc_passive[dof_idx] -= fl_force;
self.data.qfrc_frictionloss[dof_idx] -= fl_force;
```

MuJoCo treats friction loss as **constraint rows** (`mjCNSTR_FRICTION_DOF`,
`mjCNSTR_FRICTION_TENDON`) with Huber cost in the constraint solver. It is
**never** part of `qfrc_passive` in MuJoCo.

The infrastructure for migration already exists:
- `Data.qfrc_frictionloss: DVector<f64>` — separate accumulator (approach (b)
  from §15, already implemented)
- Newton unified constraint assembly (`assemble_unified_constraints()`) already
  has the Jacobian structure and Huber state classification
- `ConstraintType::FrictionLoss` with states `Quadratic`, `LinearPos`,
  `LinearNeg` defined in §15

What's missing: the actual constraint row assembly for friction loss is not
wired into `assemble_unified_constraints()`, and the tanh path is still active
for the Newton solver.

**Impact:** Every model with `frictionloss > 0` produces different dynamics from
MuJoCo. The force profile near zero velocity differs qualitatively (smooth tanh
vs Huber kink).

#### Objective

Implement friction loss as Huber-cost constraint rows in the unified constraint
system, matching MuJoCo's `mjCNSTR_FRICTION_DOF` / `mjCNSTR_FRICTION_TENDON`.

#### Specification

##### Row assembly

In `assemble_unified_constraints()`, after equality constraint rows and before
joint limit rows, emit friction loss rows:

1. **DOF friction loss**: For each DOF with `model.dof_frictionloss[i] > 0`:
   - Emit 1 row with Jacobian: 1×nv sparse, `J[row, i] = 1.0`
   - `efc_type[row] = ConstraintType::FrictionLoss`
   - `efc_pos[row] = 0.0` (friction loss has no position error)
   - `efc_floss[row] = model.dof_frictionloss[i]`
   - `solref` / `solimp`: from the joint's solver parameters (or model defaults)
   - `aref[row] = -B * efc_vel - K * imp * 0 = -B * efc_vel`
     (position term is zero; only velocity damping matters)

2. **Tendon friction loss**: For each tendon with `model.tendon_frictionloss[t] > 0`:
   - Emit 1 row with Jacobian: 1×nv dense, `J[row, :] = ten_J[t, :]`
   - Same metadata as DOF friction, but `efc_floss = tendon_frictionloss[t]`

##### Huber cost classification

Friction loss rows use the three-state Huber classification (from §15):

```
let threshold = R_i * floss_i;  // R_i = regularization, floss_i = friction loss value
if jar_i.abs() <= threshold {
    state = Quadratic;          // smooth quadratic zone near zero
} else if jar_i > threshold {
    state = LinearPos;          // linear positive zone
} else {
    state = LinearNeg;          // linear negative zone
}
```

This produces dry-friction-like behavior: quadratic resistance near zero
velocity, constant force at high velocity — matching MuJoCo's Huber cost
semantics.

##### Newton path changes

- `mj_fwd_passive()`: **Keep** friction loss in `qfrc_passive` (for PGS/CG
  fallback compatibility). Continue populating `qfrc_frictionloss` separately.
- `newton_solve()`: Subtract `qfrc_frictionloss` from RHS (already implemented).
  Friction loss is now handled by the constraint rows in J.
- The existing `data.newton_solved` branch in force balance already accounts for
  this (implemented in previous §19 spec work).

##### PGS/CG path changes

For this item, PGS/CG continues using the tanh passive force path unchanged.
The friction loss constraint rows are only active when Newton is the solver.
PGS/CG migration to unified constraints is item #30.

##### Removal scope

- Do NOT remove the tanh computation from `mj_fwd_passive()` yet — PGS/CG
  still needs it until #30.
- Do NOT remove `friction_smoothing` from Model — it becomes unused after #30
  but removing it is a breaking API change for later.

#### Acceptance Criteria

1. `assemble_unified_constraints()` emits friction loss rows for all DOFs/tendons
   with `frictionloss > 0`.
2. Newton solver produces Huber-cost friction loss forces (not tanh).
3. `efc_force` contains friction loss constraint forces after Newton solve.
4. Force balance identity holds: `M * qacc == rhs` where RHS excludes
   `qfrc_frictionloss` (Newton path).
5. Regression: PGS/CG path unchanged — `qfrc_passive` still includes tanh
   friction loss.
6. New test: single hinge with `frictionloss=1.0`, Newton solver. Compare
   `efc_force` friction loss component against MuJoCo 3.4.0 reference.
7. Existing `test_frictionloss_scaling` updated to reflect Newton path behavior.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `assemble_unified_constraints()`,
  friction loss row emission
- `sim/L0/tests/integration/passive_forces.rs` — update friction loss tests
- `sim/L0/tests/integration/newton_solver.rs` — Newton friction loss validation

#### Origin

Migrated from scattered notes in:
- `future_work_5.md` §15 "Friction loss migration" (lines 130–166) — design
  context, approach (a)/(b), compatibility notes

---

### 29. Friction Loss Migration: PGS/CG Passive Force → Constraint Rows
**Status:** Not started | **Effort:** M | **Prerequisites:** #28

#### Current State

After #28, friction loss is handled correctly for the Newton solver path via
Huber constraint rows. However, PGS/CG still uses the tanh passive force
approximation in `mj_fwd_passive()`. MuJoCo handles friction loss as constraint
rows for ALL solver types — not just Newton.

#### Objective

Extend friction loss constraint rows to the PGS/CG solver path, eliminating the
tanh passive force approximation entirely.

#### Specification

1. **Constraint assembly**: Friction loss rows (from #28) must also be emitted
   when PGS/CG is the solver. Currently `assemble_unified_constraints()` is only
   called for Newton — either extend its use to PGS/CG, or add friction loss row
   assembly to the PGS/CG contact-only path.

2. **PGS solver**: PGS must handle friction loss rows. These are single-row
   constraints with Huber cost — PGS projects each row independently. The
   projection for Huber cost:
   ```
   if state == Quadratic:  f = clamp(f_unclamped, -threshold, +threshold)
   if state == LinearPos:  f = max(f_unclamped, 0)   // or threshold boundary
   if state == LinearNeg:  f = min(f_unclamped, 0)
   ```
   (Exact projection depends on MuJoCo's PGS implementation — verify against
   `engine_solver.c:mj_solPGS`.)

3. **CG solver**: Same treatment — friction loss rows participate in the CG
   solve alongside contact rows.

4. **Remove tanh path**: Once PGS/CG handles friction loss as constraint rows:
   - Remove the friction loss computation from `mj_fwd_passive()` entirely
   - `qfrc_passive` no longer includes friction loss for ANY solver
   - `qfrc_frictionloss` accumulator can be removed (or repurposed for
     diagnostics — extract from `efc_force` instead)
   - `friction_smoothing` on Model becomes dead — deprecate

5. **Force balance update**: With friction loss removed from `qfrc_passive`,
   the force balance identity simplifies for all solver paths:
   `M * qacc == qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias`
   No more `qfrc_frictionloss` subtraction needed (friction loss is now in
   `qfrc_constraint` via the solver).

#### Acceptance Criteria

1. PGS solver handles friction loss constraint rows.
2. CG solver handles friction loss constraint rows.
3. `mj_fwd_passive()` no longer computes friction loss.
4. `qfrc_passive` matches MuJoCo's `qfrc_passive` (no friction loss component).
5. Force balance identity is uniform across all solver types.
6. `friction_smoothing` field deprecated on Model.
7. PGS friction loss forces match MuJoCo 3.4.0 reference within solver tolerance.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_passive()`, `mj_fwd_constraint()`,
  PGS/CG solver paths
- `sim/L0/tests/integration/passive_forces.rs` — remove tanh-specific tests
- `sim/L0/tests/integration/newton_solver.rs` — verify uniform behavior

#### Origin

Continuation of #28. Completes the friction loss migration started in
`future_work_5.md` §15 "Compatibility" section (lines 162–166).

---

### 30. PGS/CG Unified Constraints: Penalty Limits + Equality → Solver Rows
**Status:** Not started | **Effort:** L | **Prerequisites:** #29

#### Current State

The Newton solver path handles ALL constraint types (equality, limits, contacts)
through the unified constraint assembly (`assemble_unified_constraints()`). The
PGS/CG path uses a different architecture:

- **Contacts**: Go through the PGS/CG solver (correct)
- **Joint limits**: Penalty forces with Baumgarte stabilization
  (`mujoco_pipeline.rs:18423–18473`). Comment: "MuJoCo uses solver-based
  approach, but penalty is acceptable for most robotics."
- **Tendon limits**: Same penalty approach (`mujoco_pipeline.rs:18475–18526`)
- **Equality constraints**: Penalty forces via `apply_equality_constraints()`
  (`mujoco_pipeline.rs:18528–18536`). Comment: "Using penalty method with
  Baumgarte stabilization (like joint limits). MuJoCo uses solver-based approach
  via PGS, but penalty is robust and simpler."

MuJoCo passes ALL constraint types through the solver for ALL solver types.
The penalty approach produces different force magnitudes, stability
characteristics, and impedance scaling.

Related divergences that this item eliminates:
- Default penalty parameters 4× stiffer than MuJoCo (`default_eq_stiffness:
  10000.0` vs MuJoCo's `solref=[0.02, 1.0]` → k≈2500)
- Impedance consumption formula differs: penalty scales both position and
  velocity terms by impedance; MuJoCo KBIP applies impedance only to position

#### Objective

Route joint limits, tendon limits, and equality constraints through the PGS/CG
solver as constraint rows, matching MuJoCo's architecture for all solver types.

#### Specification

1. **Extend constraint assembly to PGS/CG**: Reuse
   `assemble_unified_constraints()` (or a shared subset) to emit equality, limit,
   and contact rows into the unified Jacobian when PGS/CG is the solver.

2. **PGS projection rules** (per constraint type):
   - **Equality**: Unclamped (no projection needed — always Quadratic)
   - **Joint/tendon limits**: Unilateral clamp: `f = max(f, 0)` (force must push
     away from limit, never pull)
   - **Contacts**: Existing cone projection (unchanged)
   - **Friction loss**: Huber projection (from #29)

3. **CG solver**: Same unified system. CG inherently handles mixed constraint
   types through the KKT system.

4. **Remove penalty code paths**:
   - Remove `solref_to_penalty()` and all callers
   - Remove `apply_equality_constraints()` penalty implementation
   - Remove `apply_connect_constraint()`, `apply_weld_constraint()`,
     `apply_joint_equality_constraint()` penalty implementations
   - Remove penalty limit code at lines 18423–18526
   - Remove `default_eq_stiffness`, `default_eq_damping` from Model

5. **Impedance**: Switch from penalty-style `F = -imp * k * pos - imp * b * vel`
   to MuJoCo KBIP: `aref = -B * vel - K * imp * pos`. This is already
   implemented in the Newton path's `compute_aref()` — reuse it.

6. **solref/solimp**: All constraints now use the standard solref/solimp
   parameterization through the solver (no more `solref_to_penalty()` conversion
   layer). Direct mode (`solref[0] ≤ 0`) works for all constraint types.

#### Acceptance Criteria

1. PGS/CG solver handles equality, limit, and contact constraints through
   unified row assembly.
2. `solref_to_penalty()` and all penalty force code removed.
3. `apply_equality_constraints()` penalty path removed.
4. Joint limit forces match MuJoCo 3.4.0 PGS reference.
5. Weld constraint forces match MuJoCo 3.4.0 PGS reference.
6. `default_eq_stiffness` / `default_eq_damping` removed from Model.
7. Force balance identity uniform across all solver types and constraint types.
8. No regressions on Newton path (already unified).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()` PGS/CG path,
  `solref_to_penalty()`, equality/limit penalty code
- `sim/L0/tests/integration/equality_constraints.rs` — update tests
- `sim/L0/tests/integration/passive_forces.rs` — limit force tests

#### Origin

This divergence was acknowledged in code comments at `mujoco_pipeline.rs:18428`
and `18530` but never tracked as a work item.

---

### 31. `solreffriction` Per-Direction Solver Parameters
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`Contact` has a single `solref` field applied to all constraint rows. MuJoCo supports
`solreffriction` — separate solver parameters for friction directions (tangent,
torsional, rolling). Comment at `mujoco_pipeline.rs:5662`: "solreffriction is NOT
applied here."

#### Objective
Apply per-direction solver reference parameters to friction constraint rows.

#### Specification

1. **MJCF parsing**: Parse `solreffriction` from `<geom>`, `<pair>`, and `<default>`
   elements. Store on `Contact` as `solreffriction: [f64; 2]`.
2. **Constraint assembly**: When building the RHS vector `b`:
   - Normal row: use `solref` (existing behavior)
   - Friction rows (tangent, torsional, rolling): use `solreffriction` if specified,
     otherwise fall back to `solref`
3. **Default**: `solreffriction = [0, 0]` means "use solref" (MuJoCo convention —
   zero values indicate "inherit from solref").

#### Acceptance Criteria
1. `solreffriction="0.05 1"` produces different friction damping than `solref`.
2. `solreffriction="0 0"` matches current behavior (regression).
3. Per-direction stabilization is numerically stable.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — parse `solreffriction`
- `sim/L0/core/src/mujoco_pipeline.rs` — constraint assembly RHS

---

### 32. Pyramidal Friction Cones
**Status:** Stub | **Effort:** L | **Prerequisites:** None

#### Current State
`Model.cone` field is stored (0=pyramidal, 1=elliptic). When pyramidal is requested,
the solver emits a warning and falls back to elliptic cones. MuJoCo supports both;
pyramidal cones linearize the friction constraint into facets, producing a different
(larger) constraint system.

Most models use the default elliptic cones. Pyramidal cones are mainly relevant for
legacy compatibility and for solvers that cannot handle second-order cone constraints.

#### Objective
Implement pyramidal friction cone constraints as an alternative to elliptic cones.

#### Specification

1. **Linearization**: For a contact with friction μ and condim=3, pyramidal cones
   replace the single elliptic cone `||f_t|| ≤ μ f_n` with 4 linear inequalities:
   `f_t1 ± f_t2 ≤ μ f_n` (4 constraint rows instead of 3).
2. **Constraint sizing**: `nefc` changes based on cone type:
   - Elliptic condim=3: 3 rows per contact (normal + 2 tangent)
   - Pyramidal condim=3: 4 rows per contact (4 facets)
   - Higher condim scales similarly
3. **PGS projection**: Pyramidal constraints use simple non-negativity projection
   (each facet force ≥ 0) instead of cone projection.
4. **Newton solver**: Pyramidal cones require different Hessian blocks. Currently
   Newton falls back to PGS for pyramidal — this item should make both paths work.

#### Acceptance Criteria
1. `<option cone="pyramidal"/>` uses linearized friction (no warning).
2. Friction force magnitude matches MuJoCo pyramidal output within 5%.
3. `<option cone="elliptic"/>` (default) is unchanged (regression).
4. Newton solver handles pyramidal cones without PGS fallback.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — constraint assembly, PGS projection,
  Newton Hessian blocks
