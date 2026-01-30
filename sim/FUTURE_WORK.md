# Future Work

> Low-priority tasks that are out of scope for current development phases.
> These items are tracked here so they don't get lost, but are not blocking any features.

---

## Crate Consolidation

The current crate structure has some redundancy that could be cleaned up:

### sim-constraint

- [ ] Reduce `sim-constraint` to joint definitions only
- Current state: Contains joint types + solver code
- Target state: Joint type definitions only (solver logic consolidated in sim-core)
- Rationale: PGS solver is now in `mujoco_pipeline.rs`, duplicates old solver code

### sim-contact

- [ ] Merge `sim-contact` into `sim-core`
- Current state: Separate crate for contact model
- Target state: Contact types and functions in `sim-core/src/contact.rs`
- Rationale: Contact is tightly coupled to collision; separate crate adds overhead

---

## When to Address

These tasks should be addressed when:
1. Major refactoring is already planned
2. Build times become a concern
3. Crate boundaries cause API friction

Not urgent because:
- Current structure works correctly
- No runtime cost
- Compile times are acceptable

---

## Code Quality Issues

Identified during solref/solimp review. Items marked ✅ have been addressed.

### P0 — Correctness

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 1 | Hardcoded Baumgarte parameters — joint limits, equality constraints ignore solref/solimp | ✅ Fixed | `eq_solref` and `jnt_solref` now read via `solref_to_penalty()`. Hardcoded `10000/1000` retained as fallback when solref ≤ 0. `eq_solimp` consumed via `compute_impedance()` — position-dependent impedance scales penalty stiffness/damping in connect, weld, and joint equality constraints. Contact solver CFM also uses `compute_impedance()` with full solimp (previously only read `solimp[0]`). |
| 2 | Distance equality constraints missing | ✅ Fixed | `apply_distance_constraint` enforces `\|p1 - p2\| = target_distance` between geom centers using penalty+impedance (Baumgarte stabilization). Added `geom_name_to_id` mapping in model builder. Handles singularity when geoms coincide (+Z fallback direction). `eq_obj1id`/`eq_obj2id` store geom IDs; body IDs derived via `model.geom_body[]`. Auto-computes initial distance when `distance` attribute omitted. 7 integration tests cover normal operation, worldbody geom, geom2-omitted (world origin sentinel), auto-distance, zero target, error handling, and inactive constraint. |

### P1 — Performance / Quality

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 3 | Contact clone in hot path | ✅ Fixed | Borrow split via `std::mem::take` on `efc_lambda`; `pgs_solve_contacts` now takes `&Data` + `&mut HashMap`. See details below. |
| 4 | Cholesky clone for implicit integrator | ✅ Fixed | `std::mem::replace` swaps populated matrix out for Cholesky consumption; zero matrix left for next step. See details below. |
| 5 | Duplicate contact structures | ✅ Improved | Old `ContactPoint` in mujoco_pipeline.rs removed. Two distinct types remain: `Contact` (mujoco pipeline constraint struct) and `ContactPoint` (sim-contact geometric contact). These serve different purposes. |

#### Issue 3 — Contact clone in hot path

**Location:** `mj_fwd_constraint()` at `mujoco_pipeline.rs:8045`

```rust
let contacts: Vec<Contact> = data.contacts.clone();
```

**Cost:** O(n_contacts) allocation per step. `Contact` is ~216 bytes; with pre-allocated
capacity of 256 contacts this clones up to ~55 KB per simulation step.

**Root cause:** Borrow checker conflict — `data.contacts` cannot be borrowed while `&mut Data`
is passed to `pgs_solve_contacts()` (line 6539). The `&mut` prevents holding any shared
reference into `data` across the call.

**Key insight:** `pgs_solve_contacts` mostly reads from `data` (`qM_cholesky`,
`qfrc_applied`, `qfrc_actuator`, `qfrc_passive`, `qfrc_bias`) but **does** write to
`data.efc_lambda` (warmstart cache: clear + insert). The `&mut Data` cannot simply become
`&Data` without addressing `efc_lambda`.

**Fix (applied):** Split the borrow by extracting `efc_lambda` from `data` via
`std::mem::take` before the contact block. Pass it as a separate `&mut HashMap` parameter
to `pgs_solve_contacts` (signature changed to `data: &Data` + `efc_lambda: &mut HashMap`).
Restore `data.efc_lambda` after the call. This eliminates the borrow conflict —
`data.contacts` is borrowed as `&[Contact]` during the PGS call without cloning.

#### Issue 4 — Cholesky clone for implicit integrator

**Location:** `mj_fwd_acceleration_implicit()` at `mujoco_pipeline.rs:8351–8355`

```rust
let chol = data
    .scratch_m_impl
    .clone()
    .cholesky()
    .ok_or(StepError::CholeskyFailed)?;
```

**Cost:** O(nv²) allocation per step. For nv=30 DOFs this copies 900 f64s (~7.2 KB).
Larger models (nv=100+) copy ~80 KB per step.

**Root cause:** nalgebra's `.cholesky()` consumes the matrix (takes ownership). Since
`scratch_m_impl` lives inside `Data`, it cannot be moved out directly, so the code clones
it to produce an owned value for the API.

**Key insight:** `scratch_m_impl` is **never read after** the Cholesky call within this
function. It is a scratch buffer that gets completely overwritten via `copy_from(&data.qM)`
at the start of every step. The clone preserves data that is immediately discarded.

**Fix (applied):** Use `std::mem::replace` to swap the populated matrix out with a
same-sized zero matrix, then factorize the owned value. This still allocates O(nv²) for
the replacement zero matrix, but avoids copying the populated values — a cheaper zero-fill
instead of element-by-element clone. The zero matrix left behind is correctly sized for the
next step's `copy_from(&data.qM)` call (nalgebra's `copy_from` requires matching
dimensions).

**True zero-allocation alternative (not yet implemented):** Implement an in-place Cholesky
factorization that operates on `&mut DMatrix<f64>` without consuming it. This could wrap
LAPACK's `dpotrf` (which factorizes in place) or use a manual implementation. The factored
lower triangle would overwrite `scratch_m_impl` directly, and a thin wrapper would provide
the `solve` interface. This is more invasive but achieves O(0) allocation.

**Future optimization (out of scope):** For serial kinematic chains the mass matrix is
banded with O(1) bandwidth. A sparse L^T D L factorization (using the existing `qLD_diag` /
`qLD_L` fields already defined in `Data`) would reduce factorization from O(nv³) to O(nv).
Featherstone's ABA algorithm is another O(nv) alternative. These are tracked under
Physics Features.

### P2 — Minor

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 6 | Block Jacobi preconditioner placeholder in `cg.rs:590` | ❌ Open | Falls back to simple Jacobi; proper block variant not implemented. |
| 7 | Various hardcoded numerical thresholds | ❌ Open | Regularization `1e-6`, penetration slop `0.001`, etc. Reasonable defaults but not per-geometry configurable. |

---

## Physics Features

### Deformable Bodies

- [ ] Implement soft body simulation (FEM or position-based dynamics)
- [ ] Implement cloth simulation
- Current state: `sim-deformable` crate exists as stub only
- Use cases: Soft tissue simulation, cloth/fabric, pneumatic actuators

### Implicit Integrators

- [ ] Add RK4 (Runge-Kutta 4th order) integrator
- [ ] Add implicit Euler for stiff systems
- Current state: Semi-implicit Euler only
- Rationale: Higher-order integrators improve accuracy for stiff systems (high damping, high stiffness springs)

### GPU Acceleration

- [ ] CUDA/Metal/WebGPU backend for parallel simulation
- [ ] Batch simulation for RL training (thousands of envs)
- Current state: CPU-only
- Use cases: Large-scale RL training, real-time multi-agent simulation
