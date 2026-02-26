# Phase 3 — Core API Gaps: Retroactive Audit & Spec

**Scope:** 7 items implemented as a batch without individual specs.
**Goal:** Audit each item as if it had been specced individually from the start.
For each item: MuJoCo reference behavior, implementation audit, gap analysis,
and remediation spec for anything below A-grade.

**Rubric:** [PHASE3_RUBRIC.md](./PHASE3_RUBRIC.md) (7 criteria, all must be A)

---

## Table of Contents

1. [DT-21: `xfrc_applied` Projection](#1-dt-21-xfrc_applied-projection)
2. [DT-41: Newton Solver](#2-dt-41-newton-solver)
3. [S51: Body Force Accumulators](#3-s51-body-force-accumulators)
4. [S52: Inverse Dynamics](#4-s52-inverse-dynamics)
5. [S53: Split Stepping API](#5-s53-split-stepping-api)
6. [S59: Name-Index Lookup](#6-s59-name-index-lookup)
7. [DT-79: User Callbacks](#7-dt-79-user-callbacks)
8. [Cross-Cutting Issues](#8-cross-cutting-issues)
9. [Remediation Summary](#9-remediation-summary)

---

## 1. DT-21: `xfrc_applied` Projection

### 1.1 MuJoCo Reference

MuJoCo stores `xfrc_applied` as a per-body `[force_3, torque_3]` 6-vector in
world frame (note: MuJoCo's layout is `[fx,fy,fz, tx,ty,tz]`). It is projected
into joint-space forces via `mj_xfrcAccumulate()` → `mj_applyFT()` during
`mj_fwdAcceleration()` — **not** during `mj_passive()`. The projection uses the
body frame origin (`xipos`) as the application point and walks the kinematic chain from
body to root, accumulating `J^T * f` for each ancestor joint.

Key behaviors:
- `xfrc_applied[0]` (world body) is ignored (body 0 has no joints)
- Sleeping bodies are **not** skipped during `xfrc_applied` projection —
  `mj_xfrcAccumulate()` loops over all bodies unconditionally. Note: the earlier
  `qfrc_smooth` accumulation (passive - bias + applied + actuator) IS sleep-gated
  via indexed operations on awake DOFs only. The xfrc_applied projection is the
  exception that writes to all DOF indices regardless of sleep state.
- `DISABLE_GRAVITY` does **not** disable `xfrc_applied` — they are independent
- Forces enter `qfrc_smooth`, not `qfrc_passive`

**Layout convention difference:** Our SpatialVector uses Featherstone convention
`[torque_3, force_3]` (angular-first), while MuJoCo's user-facing layout is
`[force_3, torque_3]` (linear-first). Our code is internally consistent —
`constraint/mod.rs:84-108` correctly interprets indices 0-2 as torque and 3-5 as
force per our convention. Users who port MuJoCo MJCF models do not need to worry
about this because the MJCF parser handles the mapping.

### 1.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| Data field | `types/data.rs` | 172 | **Pass** — `Vec<SpatialVector>`, len=nbody |
| Allocation | `types/model_init.rs` | 440 | **Pass** — zeros |
| Reset | `types/data.rs` | 865-867 | **Pass** — zeroed |
| Primary projection | `constraint/mod.rs` | 84-108 | **Pass** — `compute_qacc_smooth()` via `mj_apply_ft()` |
| Secondary projection | `forward/acceleration.rs` | 123-150 | **Pass** — implicit path duplicates for `scratch_force` |
| J^T algorithm | `jacobian.rs` | 178-238 | **Pass** — chain walk, all 4 joint types |
| Sleep guard | `constraint/mod.rs` | 92-94 | **Drift** — skips sleeping bodies; MuJoCo does NOT skip during projection |
| Zero skip | `constraint/mod.rs` | 89 | **Pass** — early exit for zero xfrc |
| DISABLE_GRAVITY independence | `constraint/mod.rs` | 84-108 | **Pass** — no gravity guard on xfrc |
| cfrc_ext initialization | `forward/acceleration.rs` | 396-399 | **Pass** — cfrc_ext starts from xfrc_applied |
| Tests | `tests/integration/xfrc_applied.rs` | 4 tests | **Pass** — upward force, torque, anti-gravity, pure torque |

### 1.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G1 | **Low** | `future_work_10c.md` line 19 says "projection in `mj_fwd_passive()`" — this is outdated. Actual projection is in `compute_qacc_smooth()` (acceleration stage). | Doc-only |
| G19 | **Medium** | Sleep gating: our code skips `xfrc_applied` projection for sleeping bodies (`constraint/mod.rs:92-94`). MuJoCo's `mj_xfrcAccumulate()` projects for ALL bodies unconditionally (note: the earlier qfrc_smooth accumulation IS sleep-gated, but xfrc_applied projection is the exception). This can cause different force accumulation when bodies wake up. | Conformance |
| G20 | **Info** | Layout convention: our `SpatialVector` uses `[torque_3, force_3]` (Featherstone), MuJoCo user API uses `[force_3, torque_3]`. Internally consistent; no user-facing issue since MJCF parser handles mapping. | By design |

### 1.4 Remediation

- **G1**: Update `future_work_10c.md` DT-21 description to: "projection in `compute_qacc_smooth()` (acceleration stage), 4 tests"
- **G19**: Remove the sleep guard from `xfrc_applied` projection in `constraint/mod.rs:92-94`. MuJoCo projects all bodies unconditionally; sleep filtering at the solver level is sufficient. This ensures correct force accumulation state when bodies are woken by external forces.
- **G20**: No action needed — document the convention difference in `dynamics/spatial.rs` docstring.

### 1.5 Grade: **B+** (correct physics, sleep-gating conformance gap)

---

## 2. DT-41: Newton Solver

### 2.1 MuJoCo Reference

MuJoCo's Newton solver (`mjSOL_NEWTON`) is a primal-space solver operating on
`qacc` directly. Key algorithm:

1. **Warmstart**: Compare cost at `qacc_warmstart` vs `qacc_smooth`; pick better
2. **Hessian**: `H = M + J^T D J` (mass matrix + active constraint contribution)
3. **Gradient**: `grad = M*qacc - qfrc_smooth - qfrc_constraint` (where `qfrc_constraint = J^T * efc_force`, computed as a separate array)
4. **Search direction**: `search = -H^{-1} * grad` (via Cholesky)
5. **Line search**: Bracketed Newton refinement (bracket the optimum via Newton
   steps, then refine via midpoint evaluation and Newton-derived candidates)
6. **Convergence**: `scale * ||grad|| < tolerance` where `scale = 1/(meaninertia * max(1,nv))`
7. **Failure mode**: On Cholesky failure, MuJoCo calls `mjERROR()` which invokes
   `mju_error_raw()` — by default this calls `exit(EXIT_FAILURE)` (hard process
   exit; NOT `abort()`). Users can override via `mju_user_error` callback (Python
   bindings raise exceptions). MuJoCo does **not** fall back to PGS. Max
   iterations exceeded returns best iterate.
8. **Sparse threshold**: Dense below ~60 DOFs, sparse LDL^T above

### 2.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| Newton outer loop | `constraint/solver/newton.rs` | 42-338 | **Pass** — 3-phase architecture (init, iterate, recover) |
| Warmstart selection | `newton.rs` | ~60-70 | **Pass** — cost comparison at two starting points |
| Hessian assembly (dense) | `solver/hessian.rs` | 32-77 | **Pass** — `M_eff + sum(D_i * J_i^T * J_i)` |
| Hessian assembly (sparse) | `solver/hessian.rs` | 93-246 | **Pass** — CSC + LDL^T |
| Sparse threshold | `newton.rs` | constant | **Pass** — `NV_SPARSE_THRESHOLD = 60` |
| Gradient + search direction | `solver/primal.rs` | 24-97 | **Pass** — Cholesky solve |
| Line search (prepare) | `solver/primal.rs` | 144-250 | **Pass** — quadratic polynomial coefficients |
| Line search (search) | `solver/primal.rs` | 391-560 | **Pass** — 3-phase bracketed Newton |
| Constraint states | `types/enums.rs` | 612-625 | **Pass** — Quadratic/Satisfied/LinearNeg/LinearPos/Cone |
| Convergence check | `newton.rs` | 280-307 | **Pass** — scale * gradient norm < tolerance |
| PGS fallback | `constraint/mod.rs` | 422-429 | **Divergence** — falls back to PGS on CholeskyFailed; MuJoCo hard-exits via `mjERROR()` → `exit(EXIT_FAILURE)`. Intentional robustness improvement. |
| Implicit support (M_impl) | `constraint/mod.rs` | 140-167 | **Pass** — `M + h*D + h^2*K` with tendon K/D |
| Implicit support (qfrc_impl) | `constraint/mod.rs` | 184-235 | **Pass** — deadband-aware stiffness |
| Incremental Cholesky | `solver/hessian.rs` | ~255 | **Pass** — rank-1 updates on state change |
| Cone Hessian | `solver/hessian.rs` | `hessian_cone()` | **Pass** — elliptic friction cone augmentation |
| Solver stats | `types/enums.rs` | 627-646 | **Pass** — SolverStat with 6 fields |
| Noslip post-processor | `constraint/mod.rs` | after Newton | **Pass** — applied after convergence |
| Config fields | `types/model.rs` | 693-716 | **Pass** — solver_iterations, tolerance, ls_* |
| Tests | `tests/integration/newton_solver.rs` | ~1900 lines | **Pass** — 45+ tests, MuJoCo reference comparisons |

### 2.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G21 | **Info** | PGS fallback on Cholesky failure: MuJoCo calls `mjERROR()` → `exit(EXIT_FAILURE)` (hard process exit; overridable via `mju_user_error`). Our code gracefully falls back to PGS solver. This is an **intentional robustness improvement** — hard exit is unacceptable in a Rust library. Documented as a justified divergence. | By design |

### 2.4 Grade: **A** (comprehensive implementation; PGS fallback is a justified improvement over MuJoCo's hard exit)

---

## 3. S51: Body Force Accumulators (`cacc`, `cfrc_int`, `cfrc_ext`)

### 3.1 MuJoCo Reference

MuJoCo computes per-body accumulators in `mj_rnePostConstraint()`
(`engine_core_smooth.c`) after the constraint solver runs:

- **`cacc`**: Per-body 6D acceleration in world frame. `cacc[0]` is the gravity
  pseudo-acceleration `[0,0,0, -gx,-gy,-gz]` (in `[angular, linear]` order).
  Forward pass: propagate root-to-leaf via
  `cacc[b] = cacc[parent] + cdof_dot*qvel + cdof*qacc`. MuJoCo precomputes
  `cdof_dot` (the time-derivative of the motion-subspace columns) in `mj_comVel`,
  encoding the Coriolis/centripetal acceleration. This is equivalent to the
  textbook `S_dot*qvel + cvel_parent x (S*qvel)` but computed via the
  precomputed `cdof_dot` array.

- **`cfrc_ext`**: Per-body external forces. Sum of `xfrc_applied` + contact/constraint
  forces from the solver (contact, connect, weld, flex equality). **Not** actuator
  forces (actuators act in generalized coordinates; they don't produce Cartesian
  body forces directly).

- **`cfrc_int`**: Per-body internal (reaction) forces. Backward pass (leaf-to-root):
  `cfrc_int[b] = I*cacc[b] + v x*(I*v) - cfrc_ext[b]`, accumulated into parent.

**Important**: In MuJoCo, `mj_rnePostConstraint()` is **not** called
automatically by `mj_forward()` or `mj_step()`. It is triggered lazily via the
`flg_rnepost` flag, which is set only when sensors require post-constraint RNE
data. Our implementation always calls it during `forward_acc()`, which is a
superset of MuJoCo's behavior — correct but potentially doing unnecessary work
when no sensors need it. MuJoCo zeros these fields via bulk `memset` in
`mj_resetData()`, not per-field zeroing.

### 3.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| Data fields | `types/data.rs` | 562-573 | **Pass** — `Vec<SpatialVector>`, documented |
| Allocation | `types/model_init.rs` | 655-657 | **Pass** — zeros |
| Reset | `types/data.rs` | 824-904 | **Fail** — `cacc`, `cfrc_int`, `cfrc_ext` NOT zeroed in `reset()` |
| Pipeline call | `forward/mod.rs` | 406-407 | **Pass** — called in `forward_acc()` after constraints |
| Inverse call | `inverse.rs` | 36 | **Pass** — called in `inverse()` |
| DISABLE_GRAVITY guard | `forward/acceleration.rs` | 533-543 | **Pass** — zeros pseudo-accel when disabled |
| cacc forward pass | `forward/acceleration.rs` | 531-584 | **Pass** — correct root-to-leaf with Coriolis |
| Coriolis term | `forward/acceleration.rs` | 573 | **Pass** — uses `cdof_dot * qvel` (MuJoCo precomputes cdof_dot in mj_comVel) |
| cfrc_ext: xfrc_applied | `forward/acceleration.rs` | 396-399 | **Pass** — initializes from xfrc_applied |
| cfrc_ext: contact forces | `forward/acceleration.rs` | 401-528 | **Pass** — reconstructs from efc rows |
| cfrc_ext: actuator forces | `forward/acceleration.rs` | — | **Pass** — correctly excluded (MuJoCo excludes them too) |
| cfrc_int backward pass | `forward/acceleration.rs` | 586-612 | **Pass** — `I*cacc + v x*(I*v) - cfrc_ext` |
| cfrc_int propagation | `forward/acceleration.rs` | 604-612 | **Pass** — leaf-to-root including world body |
| Contact force models | `forward/acceleration.rs` | 401-528 | **Pass** — frictionless, pyramidal, elliptic |
| Newton's 3rd law | `forward/acceleration.rs` | ~500 | **Pass** — +f on body2, -f on body1 |
| Tests | `tests/integration/body_accumulators.rs` | 4 tests | **Weak** — see gaps |

### 3.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G2 | **Medium** | `cacc`, `cfrc_int`, `cfrc_ext` not zeroed in `Data::reset()`. Stale values survive reset. | Bug |
| G3 | **Medium** | `data.rs` docstring (line 572) says "Copy of `xfrc_applied`" — misleading, because contact forces are also included. | Doc |
| G4 | **Medium** | `future_work_13.md` spec (line 33) says `cfrc_ext` includes "actuator forces" — this was incorrect in the spec. MuJoCo does NOT include actuator forces. Implementation is correct. | Spec-doc mismatch |
| G5 | **Low** | No test for `cfrc_ext` with contact forces present (only tests xfrc_applied path). | Test gap |
| G6 | **Low** | No sleep-state test for body accumulators. | Test gap |
| G7 | **Low** | No test for multi-body cfrc_int propagation chain. | Test gap |

### 3.4 Remediation

- **G2** (Bug): Add to `Data::reset()`:
  ```rust
  for v in &mut self.cacc { *v = SpatialVector::zeros(); }
  for v in &mut self.cfrc_int { *v = SpatialVector::zeros(); }
  for v in &mut self.cfrc_ext { *v = SpatialVector::zeros(); }
  ```
- **G3** (Doc): Update `data.rs` docstring to: "Per-body external forces: `xfrc_applied` + contact/constraint solver forces."
- **G4** (Doc): Update `future_work_13.md` to remove "actuator" from cfrc_ext description.
- **G5-G7** (Tests): Add tests per [PHASE3_TEST_SPEC.md](./PHASE3_TEST_SPEC.md).

### 3.5 Grade: **B** (correct physics, reset bug + test gaps)

---

## 4. S52: Inverse Dynamics (`mj_inverse`)

### 4.1 MuJoCo Reference

MuJoCo's `mj_inverse()` (`engine_inverse.c`) computes the generalized forces
that produce given accelerations. The full algorithm is:

```
qfrc_inverse = RNE(q, qvel, qacc=0)     // bias forces via recursive Newton-Euler
             + tendon_bias               // tendon wrapping forces
             + M * qacc                  // inertial forces
             - qfrc_passive              // passive joint forces
             - qfrc_constraint           // constraint forces (via mj_invConstraint)
```

Note: `RNE(q, qvel, qacc=0)` with `flg_acc=0` produces the bias forces
(Coriolis + gravity), which is conceptually equivalent to `qfrc_bias` but
computed via the RNE traversal rather than reading a stored field. The
constraint forces `qfrc_constraint` are computed by `mj_invConstraint()` which
reconstructs constraint forces from the solver output.

Our simplified formula `M*qacc + qfrc_bias - qfrc_passive` is equivalent for
the unconstrained case and for systems where `qfrc_bias` already incorporates
tendon bias. The `qfrc_constraint` term is the main omission — see G22.

Body accumulators are populated via `mj_rnePostConstraint()`, but only lazily
(via `flg_rnepost` flag set by sensors, not directly by `mj_inverse()`).

Runtime flags:
- `ENABLE_FWDINV`: When set, `mj_compareFwdInv()` runs after forward dynamics.
  It assembles `qforce = qfrc_applied + qfrc_actuator + xfrcAccumulate(xfrc_applied)`
  (note: `mj_xfrcAccumulate` projects body-space wrenches into joint space via
  per-body `mj_applyFT` chain walks, not a single J^T multiply; `qfrc_passive`
  is excluded). MuJoCo stores **two** L2 norms in `solver_fwdinv[2]`:
  (1) constraint force discrepancy and (2) applied force discrepancy. Our
  implementation uses a **single** scalar `fwdinv_error` — see G23.
- `ENABLE_INVDISCRETE`: When set, modifies `qacc` before inverse computation to
  undo implicit integration effects. Transforms `qacc_discrete` →
  `qacc_continuous` via `mj_discreteAcc()`, which modifies the mass-acceleration
  product to undo implicit integration effects. For Euler: adds `h*B*qacc`
  damping correction. For implicit integrators: uses modified mass matrix
  `M - dt*dDeriv/dvel`. Conceptually: `qacc_new = M^{-1} * M_hat * qacc_old`.

### 4.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| inverse() method | `inverse.rs` | 30-41 | **Pass** — `M*qacc + qfrc_bias - qfrc_passive` |
| qfrc_inverse field | `types/data.rs` | 556-560 | **Pass** — `DVector<f64>`, documented |
| fwdinv_error field | `types/data.rs` | 393-396 | **Pass** — diagnostic field |
| Body accumulators call | `inverse.rs` | 36 | **Pass** — calls `mj_body_accumulators()` |
| nv=0 guard | `inverse.rs` | 32 | **Pass** — early return |
| ENABLE_FWDINV constant | `types/enums.rs` | 542 | **Pass** — `1 << 2` |
| ENABLE_FWDINV guard | `forward/mod.rs` | 415-417 | **Pass** — gates `compare_fwd_inv()` |
| compare_fwd_inv() | `forward/mod.rs` | 422-447 | **Pass** — correct comparison formula |
| ENABLE_INVDISCRETE constant | `types/enums.rs` | 544 | **Pass** — defined |
| ENABLE_INVDISCRETE impl | — | — | **Gap** — constant defined, no behavioral effect |
| qfrc_inverse in reset() | `types/data.rs` | 824-904 | **Fail** — not zeroed |
| qfrc_inverse initialization | `types/model_init.rs` | 652 | **Pass** — `DVector::zeros(nv)` |
| Builder FWDINV warning | `mjcf/builder/mod.rs` | 837-847 | **Drift** — warns "not implemented" but it IS |
| Tests | `tests/integration/inverse_dynamics.rs` | 3 tests | **Pass** — round-trip, identity, free body |

### 4.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G8 | **Medium** | `qfrc_inverse` not zeroed in `Data::reset()`. | Bug |
| G9 | **Low** | `ENABLE_INVDISCRETE` constant defined but has no implementation. Builder warns it has no effect; this is accurate but should be explicitly tracked. MuJoCo's implementation transforms qacc via `M^{-1} * M_hat * qacc` to undo implicit effects. | Deferred feature |
| G10 | **Low** | Builder warning at `mod.rs:844` says "ENABLE_FWDINV ... not implemented" — outdated, it IS implemented. | Stale warning |
| ~~G11~~ | ~~**Low**~~ | ~~`fwdinv_error` not zeroed in `Data::reset()`.~~ Sixth-pass correction: `fwdinv_error` IS zeroed at `data.rs:889`. Not a bug. | ~~Bug~~ Verified |
| G22 | **Medium** | Inverse formula omits `qfrc_constraint` term. MuJoCo's full formula includes `- qfrc_constraint` (recomputed from scratch by `mj_invConstraint()` via `jar = J*qacc - aref` → `mj_constraintUpdate`, not read from stored forward solver output). Our simplified formula `M*qacc + qfrc_bias - qfrc_passive` is only exact for the unconstrained case. For systems with active contacts/constraints, the inverse result will differ from MuJoCo's. | Conformance |
| G23 | **Low** | `fwdinv_error` is a single `f64` scalar (max norm). MuJoCo stores `solver_fwdinv[2]`: two L2 norms (constraint force discrepancy + applied force discrepancy). Our single scalar loses the decomposition. | Conformance |

### 4.4 Remediation

- **G8** (Bug): Add `self.qfrc_inverse.fill(0.0);` to `Data::reset()`.
- **G9** (Track): Add `ENABLE_INVDISCRETE` to ROADMAP as a sub-item of S52 (deferred). Reference MuJoCo's `M^{-1} * M_hat * qacc` transform.
- **G10** (Stale): Remove or update the ENABLE_FWDINV warning in the builder.
- ~~**G11** (Bug): Add `self.fwdinv_error = 0.0;` to `Data::reset()`.~~ Sixth-pass correction: `fwdinv_error` IS zeroed at `data.rs:889`. No action needed.
- **G22** (Conformance): Implement `mj_invConstraint()` equivalent: compute `jar = efc_J * qacc - efc_aref`, then implement `mj_constraintUpdate` (impedance-based mapping from `jar` to `efc_force` to `qfrc_constraint = J^T * efc_force`). Subtract `qfrc_constraint` from `qfrc_inverse`. **Prerequisites:** (a) build standalone `mj_constraintUpdate` function; (b) update `compare_fwd_inv` to save/restore `efc_force` and `qfrc_constraint` before calling `inverse()`, since `mj_invConstraint` overwrites both arrays. Track as a Phase 3 follow-up. **G23 is blocked by G22.**
- **G23** (Conformance): Replace `fwdinv_error: f64` with `solver_fwdinv: [f64; 2]` to store both constraint and applied force discrepancy L2 norms separately. **Blocked by G22** — without `mj_invConstraint`, the two norms are meaningless. Also fix `compare_fwd_inv` to use L2 norm (not max-norm) and to build the forward force vector as `qfrc_applied + xfrc_project + qfrc_actuator` (excluding `qfrc_constraint`, which MuJoCo excludes from the forward side). Low priority — the single scalar is sufficient for most diagnostic use cases.

### 4.5 Grade: **B** (core algorithm correct, inverse formula simplified, reset bug + stale warning)

---

## 5. S53: Split Stepping API (`step1`/`step2`)

### 5.1 MuJoCo Reference

MuJoCo's `mj_step1()` runs position + velocity stages and fires `mjcb_control`.
`mj_step2()` runs actuation, acceleration, constraints, integration, sleep, warmstart.
Together they are equivalent to `mj_step()` for Euler/implicit integrators.

Key behaviors:
- `step1()` fires `mjcb_control` at the velocity/acceleration boundary
- `step2()` dispatches implicit integrators correctly (`mjINT_IMPLICIT`,
  `mjINT_IMPLICITFAST` → `mj_implicit`) but falls back to Euler for RK4 —
  no warning emitted. RK4 is incompatible with split-step force injection.
- `step()` is the canonical entry point (not refactored to call step1+step2)
- State validation split: positions/velocities in step1, accelerations in step2
- `mjcb_control` in `mj_step1()`: NOT gated by `DISABLE_ACTUATION`
- `mjcb_control` in `mj_forwardSkip()`: IS gated by `DISABLE_ACTUATION`
- RK4 in `step()`: `mjcb_control` fires 4 times (1 initial + 3 RK4 substeps)
- `step1()` computes position and velocity sensors

### 5.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| step1() | `forward/mod.rs` | 62-115 | **Pass** — pos+vel stages, fires cb_control |
| step2() | `forward/mod.rs` | 117-163 | **Pass** — acc stage + Euler integration |
| Pipeline split | `forward/mod.rs` | 279-420 | **Pass** — `forward_pos_vel()` / `forward_acc()` |
| cb_control in step1 | `forward/mod.rs` | 109-112 | **Pass** — fires after forward_pos_vel |
| cb_control in forward_core | `forward/mod.rs` | 268-274 | **Pass** — same pipeline point |
| Timestep validation | `forward/mod.rs` | 98-100 | **Pass** — `<= 0.0 || !is_finite()` |
| State validation | `forward/mod.rs` | 103-104 | **Pass** — check_pos + check_vel in step1 |
| Accel validation | `forward/mod.rs` | 146-147 | **Pass** — check_acc in step2 |
| Sensor split (pos+vel) | `forward/mod.rs` | 339, 352 | **Pass** — in forward_pos_vel |
| Sensor split (acc) | `forward/mod.rs` | 410-411 | **Pass** — in forward_acc |
| Sleep + warmstart | `forward/mod.rs` | 152-161 | **Pass** — in step2 after integrate |
| RK4 not-refactored doc | `forward/mod.rs` | 88-91 | **Pass** — explicitly documented |
| Tests | `tests/integration/split_step.rs` | 4 tests | **Pass** — Euler equiv, implicit equiv, force injection, invalid timestep |

### 5.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G12 | **Medium** | No guard/warning when `model.integrator = RK4` and user calls `step2()`. Integration silently uses Euler, producing different results than `step()`. MuJoCo also silently falls back, so this is conformant — but a Rust-idiomatic `tracing::warn!` would be better UX. | UX improvement |
| G13 | **Low** | Original spec (future_work_13.md line 121) says "step() remains step1()+step2() for convenience" — implementation intentionally does NOT do this (correct decision, but spec wording is misleading). | Spec-doc mismatch |
| G24 | **Medium** | `cb_control` gating: `step1()` correctly fires unconditionally (matches MuJoCo's `mj_step1`). `forward_core()` is missing the `DISABLE_ACTUATION` gate — MuJoCo's `mj_forwardSkip()` gates with `!mjDISABLED(mjDSBL_ACTUATION)`. | **Fail** — conformance gap |

### 5.4 Remediation

- **G12**: Add a `tracing::warn!` in `step2()` when `model.integrator == RungeKutta4`:
  ```rust
  if model.integrator == Integrator::RungeKutta4 {
      tracing::warn!("step2() uses Euler integration regardless of model.integrator; \
                      RK4 is not compatible with split-step force injection");
  }
  ```
- **G13**: Update `future_work_13.md` line 121 to clarify the intentional separation.
- **G24** (Conformance): In `forward_core()` at `forward/mod.rs:270-273`, add `DISABLE_ACTUATION` gate around `cb_control` invocation: `if !disabled(model, DISABLE_ACTUATION) { (cb.0)(model, self); }`. Do NOT add this gate to `step1()` — MuJoCo's `mj_step1()` fires unconditionally.

### 5.5 Grade: **B+** (correct implementation, missing RK4 guard, cb_control DISABLE_ACTUATION gate missing in forward_core)

---

## 6. S59: Name-Index Lookup (`mj_name2id`/`mj_id2name`)

### 6.1 MuJoCo Reference

MuJoCo's `mj_name2id(model, type, name)` and `mj_id2name(model, type, id)` support
all `mjtObj` enum values. The physics-relevant subset (excluding rendering-only
types like camera, light, texture, material, skin, pair, exclude, numeric, text,
tuple, key, plugin) is:

- Body, Joint, Geom, Site, Tendon, Actuator, Sensor, Mesh, Hfield, Equality

The original spec (future_work_14.md) listed "camera, light, material, texture"
which are rendering-only types not parsed by our MJCF pipeline. These are
correctly deferred — our v1.0 scope is physics, not rendering.

### 6.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| ElementType enum | `types/enums.rs` | 9-34 | **Pass** — 10 physics-relevant types |
| name2id() | `types/model.rs` | 852-877 | **Pass** — O(1) HashMap, all 10 types |
| id2name() | `types/model.rs` | 879-897 | **Pass** — O(1) Vec indexing |
| HashMap fields (10) | `types/model.rs` | 780-800 | **Pass** — one per type |
| Name arrays | `types/model.rs` | various | **Pass** — `Vec<Option<String>>` |
| Builder population | `mjcf/builder/*.rs` | various | **Pass** — all 10 types populated |
| World body init | `mjcf/builder/init.rs` | 168 | **Pass** — "world" → 0 |
| Empty name handling | `mjcf/builder/*.rs` | various | **Pass** — skip empty names |
| Tests | `tests/integration/name_lookup.rs` | 4 tests | **Pass** — round-trip, reverse, nonexistent, out-of-bounds |

### 6.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G14 | **Info** | Camera, Light, Material, Texture types not in ElementType. These are rendering-only elements not parsed by our MJCF pipeline. Correctly out of scope for v1.0 (physics-only). | By design |
| G15 | **Low** | Spec (future_work_14.md line 161) lists "camera, light, mesh, material, texture" — should be updated to reflect the actual scope decision. | Spec-doc mismatch |

### 6.4 Remediation

- **G14**: No action needed — correct scoping decision for v1.0.
- **G15**: Update `future_work_14.md` to note that rendering types are post-v1.0.

### 6.5 Grade: **A** (complete for physics scope, correct design decisions)

---

## 7. DT-79: User Callbacks (`mjcb_*` Equivalents)

### 7.1 MuJoCo Reference

MuJoCo 3.4.0 defines 8 global callbacks:

| MuJoCo Callback | Purpose | Pipeline Stage |
|-----------------|---------|----------------|
| `mjcb_passive` | Custom passive forces | Near end of `mj_passive()` (after gravity/fluid/contact, before plugins) |
| `mjcb_control` | Control injection | Between vel and acc stages |
| `mjcb_contactfilter` | Contact pair filtering | After affinity check |
| `mjcb_sensor` | User sensor evaluation | Per-stage (pos/vel/acc) |
| `mjcb_act_dyn` | User actuator dynamics | During actuation |
| `mjcb_act_gain` | User actuator gain | During actuation |
| `mjcb_act_bias` | User actuator bias | During actuation |
| `mjcb_time` | Profiler timing (`TM_START`/`TM_END` macros) | Throughout pipeline |

Key design differences from MuJoCo:
- MuJoCo uses global function pointers (C-style)
- CortenForge uses per-Model `Arc<dyn Fn + Send + Sync>` (Rust-idiomatic, thread-safe)

### 7.2 Implementation Audit

| Aspect | File | Lines | Verdict |
|--------|------|-------|---------|
| Callback<F> wrapper | `types/callbacks.rs` | 21-32 | **Pass** — Arc + Clone + Debug |
| CbPassive type | `types/callbacks.rs` | 37-41 | **Pass** — `Fn(&Model, &mut Data)` |
| CbControl type | `types/callbacks.rs` | 43-46 | **Pass** — `Fn(&Model, &mut Data)` |
| CbContactFilter type | `types/callbacks.rs` | 48-52 | **Pass** — `Fn(&Model, &Data, usize, usize) -> bool` |
| CbSensor type | `types/callbacks.rs` | 54-59 | **Pass** — `Fn(&Model, &mut Data, usize, SensorStage)` |
| CbActDyn type | `types/callbacks.rs` | 61-65 | **Pass** — `Fn(&Model, &Data, usize) -> f64` |
| CbActGain type | `types/callbacks.rs` | 67-71 | **Pass** — `Fn(&Model, &Data, usize) -> f64` |
| CbActBias type | `types/callbacks.rs` | 73-77 | **Pass** — `Fn(&Model, &Data, usize) -> f64` |
| Model storage (7 fields) | `types/model.rs` | 814-829 | **Pass** — `Option<Cb*>` |
| Set/clear methods (14) | `types/model.rs` | 899-1007 | **Pass** — ergonomic API |
| cb_passive invocation | `forward/passive.rs` | 763-765 | **Pass** — end of mj_fwd_passive |
| cb_control invocation (step1) | `forward/mod.rs` | 109-112 | **Pass** — after forward_pos_vel |
| cb_control invocation (fwd) | `forward/mod.rs` | 268-274 | **Pass** — same pipeline point |
| cb_contactfilter invocation | `collision/mod.rs` | 412-416 | **Pass** — after affinity check |
| cb_sensor invocation (pos) | `sensor/position.rs` | 328-330 | **Pass** — position stage |
| cb_sensor invocation (vel) | `sensor/velocity.rs` | 223-225 | **Pass** — velocity stage |
| cb_sensor invocation (acc) | `sensor/acceleration.rs` | 221-223 | **Pass** — acceleration stage |
| cb_act_dyn invocation | `forward/actuation.rs` | 408-413 | **Pass** — User dynamics |
| cb_act_gain invocation | `forward/actuation.rs` | 448-452 | **Pass** — User gain |
| cb_act_bias invocation | `forward/actuation.rs` | 475-479 | **Pass** — User bias |
| SensorStage enum | `types/enums.rs` | 410-422 | **Pass** — Pos/Vel/Acc |
| Tests | `tests/integration/callbacks.rs` | 5 tests | **Pass** — passive, control, filter, none, clone |

### 7.3 Gaps Found

| # | Severity | Description | Status |
|---|----------|-------------|--------|
| G16 | **Info** | `mjcb_time` not implemented. MuJoCo uses this for **profiler timing** (called by `TM_START`/`TM_END` macros throughout the pipeline), NOT for custom time advancement. It is a performance instrumentation hook, not a physics callback. Our crate can use Rust's standard profiling tools (`tracing`, `puffin`, etc.) instead. | By design |
| G17 | **Low** | No test for `cb_sensor` callback invocation. | Test gap |
| G18 | **Low** | No test for `cb_act_dyn`, `cb_act_gain`, `cb_act_bias` callbacks. | Test gap |
| G25 | **Info** | Contact filter polarity: MuJoCo's `mjcb_contactfilter` returns nonzero to **reject** a contact. Our `CbContactFilter` returns `true` to **keep**, and the call site negates it: `if !(cb)(…) { continue; }`. The result is functionally equivalent — contacts are rejected the same way — but the callback polarity convention is inverted. | By design |

### 7.4 Remediation

- **G16**: No action needed. `mjcb_time` is a profiler hook; Rust has superior profiling infrastructure. Document in `callbacks.rs` that this is intentionally omitted.
- **G17-G18**: Add tests per [PHASE3_TEST_SPEC.md](./PHASE3_TEST_SPEC.md).
- **G25**: Document the polarity inversion prominently in `callbacks.rs` CbContactFilter docstring: "MuJoCo's `mjcb_contactfilter` returns nonzero to reject; CortenForge's `CbContactFilter` returns `true` to keep. Users porting MuJoCo filter callbacks must invert the return value." This is a porting hazard that needs visible documentation.

### 7.5 Grade: **A-** (correct design, minor test gaps)

---

## 8. Cross-Cutting Issues

### 8.1 Data::reset() Staleness

Multiple Phase 3 fields are not zeroed in `Data::reset()`:

| Field | Type | Section | Severity |
|-------|------|---------|----------|
| `cacc` | `Vec<SpatialVector>` | S51 | Medium |
| `cfrc_int` | `Vec<SpatialVector>` | S51 | Medium |
| `cfrc_ext` | `Vec<SpatialVector>` | S51 | Medium |
| `qfrc_inverse` | `DVector<f64>` | S52 | Medium |

Note: `fwdinv_error` was originally listed here but sixth-pass verification
confirmed it IS zeroed at `data.rs:889`. Only 4 fields are affected, not 5.

These are all computed fields (populated by `forward()` or `inverse()`), but
stale values after `reset()` without a subsequent `forward()` call could mislead
users. MuJoCo zeros all Data fields on `mj_resetData()`.

### 8.2 MuJoCo Conformance Divergences (Intentional)

These are intentional design divergences from MuJoCo, justified by Rust
idioms, robustness, or correctness improvements:

| Item | MuJoCo Behavior | Our Behavior | Justification |
|------|-----------------|--------------|---------------|
| G21 — Newton Cholesky | `mjERROR()` → `exit(EXIT_FAILURE)` | PGS fallback | Hard exit is unacceptable in a library |
| G20 — SpatialVector layout | `[force, torque]` | `[torque, force]` (Featherstone) | Standard robotics convention; internally consistent |
| G16 — mjcb_time | Profiler hook | Not implemented | Rust has `tracing`/`puffin`; C-style timer callback unnecessary |
| G25 — contactfilter polarity | nonzero = reject | true = keep (negated at call site) | More intuitive API; same behavior |

### 8.3 MuJoCo Conformance Gaps (Need Fixing)

| Item | MuJoCo Behavior | Our Behavior | Impact |
|------|-----------------|--------------|--------|
| G19 — xfrc_applied sleep | Projects ALL bodies | Skips sleeping | Different force state on wake-up |
| G22 — inverse formula | `RNE + tendon_bias + M*qacc - passive - constraint` | `M*qacc + bias - passive` | Incorrect for constrained systems |
| G23 — fwdinv format | `solver_fwdinv[2]` (two norms) | `fwdinv_error` (one scalar) | Loses decomposition info |

### 8.4 Documentation Drift

| File | Issue |
|------|-------|
| `future_work_10c.md:19` | DT-21 says "projection in mj_fwd_passive()" — should be "acceleration stage" |
| `future_work_13.md:33` | S51 cfrc_ext says "applied + actuator" — should be "applied + contact" |
| `future_work_13.md:121` | S53 says "step() remains step1()+step2()" — intentionally not done |
| `future_work_14.md:161` | S59 lists camera/light/material/texture — deferred (rendering scope) |
| `mjcf/builder/mod.rs:844` | ENABLE_FWDINV warning says "not implemented" — it IS implemented |

---

## 9. Remediation Summary

### Bug Fixes (implement before Phase 4)

| ID | Item | Fix | File |
|----|------|-----|------|
| G2 | S51 reset | Zero `cacc`/`cfrc_int`/`cfrc_ext` in `Data::reset()` | `types/data.rs` |
| G8 | S52 reset | Zero `qfrc_inverse` in `Data::reset()` | `types/data.rs` |
| G10 | S52 warning | Remove/update stale ENABLE_FWDINV "not implemented" warning | `mjcf/builder/mod.rs` |
| G12 | S53 RK4 guard | Add `tracing::warn!` in step2() for RK4 integrator | `forward/mod.rs` |
| G19 | DT-21 sleep | Remove sleep guard from xfrc_applied projection | `constraint/mod.rs` |

### Conformance Fixes (MuJoCo parity)

| ID | Item | Fix | File |
|----|------|-----|------|
| G22 | S52 inverse formula | Implement `mj_invConstraint()` equivalent: `jar = J*qacc - aref` → constraint update → subtract `qfrc_constraint` from `qfrc_inverse` | `inverse.rs` |
| G23 | S52 fwdinv format | Replace `fwdinv_error: f64` with `solver_fwdinv: [f64; 2]` (constraint + applied L2 norms). **Blocked by G22.** | `types/data.rs`, `forward/mod.rs` |
| G24 | S53 cb_control gating | Add `!disabled(model, DISABLE_ACTUATION)` gate to `cb_control` in `forward_core()`. Do NOT add to `step1()`. | `forward/mod.rs` |

### Documentation Fixes

| ID | Item | Fix | File |
|----|------|-----|------|
| G1 | DT-21 desc | Update to "acceleration stage" | `future_work_10c.md` |
| G3 | S51 docstring | Update cfrc_ext doc to include contact forces | `types/data.rs` |
| G4 | S51 spec | Remove "actuator" from cfrc_ext description | `future_work_13.md` |
| G13 | S53 spec | Clarify step() is intentionally separate | `future_work_13.md` |
| G15 | S59 spec | Note rendering types deferred to post-v1.0 | `future_work_14.md` |

### Test Additions

| ID | Item | Tests Needed |
|----|------|-------------|
| G5 | S51 cfrc_ext | Contact force accumulation test |
| G6 | S51 sleep | Body accumulators with sleeping bodies |
| G7 | S51 cfrc_int | Multi-body propagation chain |
| G17 | DT-79 sensor | cb_sensor invocation at all 3 stages |
| G18 | DT-79 actuator | cb_act_dyn/gain/bias invocation tests |

### Intentional Divergences (documented, no action needed)

| ID | Item | Justification |
|----|------|---------------|
| G20 | SpatialVector layout | Featherstone convention standard in robotics; internally consistent |
| G21 | Newton PGS fallback | Library cannot hard-exit; PGS fallback is strictly better |
| G25 | contactfilter polarity | `true=keep` more intuitive than MuJoCo's `nonzero=reject` |

### Deferred Items (post-v1.0)

| ID | Item | Reason |
|----|------|--------|
| G9 | ENABLE_INVDISCRETE | Discrete-time inverse: `M^{-1} * M_hat * qacc` transform. Not needed for v1.0. |
| G14 | S59 rendering types | Camera/Light/Material/Texture are rendering-only |
| G16 | mjcb_time | Profiler hook; Rust has `tracing`/`puffin` |

---

## Scoring Summary

| Item | Grade | Gaps | Critical? |
|------|:-----:|:----:|:---------:|
| DT-21: xfrc_applied | **B+** | 1 doc, 1 conformance (sleep gating), 1 info (layout) | Yes — sleep gate |
| DT-41: Newton Solver | **A** | 1 info (PGS fallback — justified) | No |
| S51: Body Accumulators | **B** | 6 (1 bug, 2 doc, 3 test) | Yes — reset bug |
| S52: Inverse Dynamics | **B** | 5 (1 bug, 1 stale, 1 deferred, 2 conformance) | Yes — formula gap |
| S53: Split Stepping | **B+** | 3 (1 UX, 1 doc, 1 conformance: cb_control DISABLE_ACTUATION gate) | Yes — cb_control gate |
| S59: Name Lookup | **A** | 2 info/doc | No |
| DT-79: Callbacks | **A-** | 4 (1 info, 2 test, 1 info polarity) | No |

**Overall Phase 3 Grade: B** — Core physics is correct across all 7 items.
The Newton solver is A-grade with a justified divergence (PGS fallback).
The primary gaps are:
1. **MuJoCo conformance**: xfrc_applied sleep gating (G19), inverse formula
   missing constraint forces (G22), fwdinv comparison format (G23)
2. **Data::reset() staleness**: 4 fields not zeroed (G2, G8)
3. **Documentation drift**: 5 mismatches across spec files and builder
4. **Test coverage**: 5 test groups missing for edge cases

**To reach A:** Fix 5 bug items (including G19 which is also a conformance gap),
address 3 conformance-only fixes (G22, G23, G24), update 5 doc mismatches,
add 1 doc fix for porting hazard (G25), and add 5 test groups. The conformance
fixes (G19, G22, G23, G24) are the most important — they represent actual
behavioral differences from MuJoCo that could cause incorrect results in
downstream simulations. **Note:** G23 is blocked by G22; implement in order.
