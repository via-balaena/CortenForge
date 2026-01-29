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
| 1 | Hardcoded Baumgarte parameters — joint limits, equality constraints ignore solref/solimp | ✅ Fixed | `eq_solref` and `jnt_solref` now read via `solref_to_penalty()`. Hardcoded `10000/1000` retained as fallback when solref ≤ 0. **Remaining gap:** `eq_solimp` is stored on `Model` but never consumed. |
| 2 | Distance equality constraints missing | ❌ Open | `EqualityType::Distance` match arm prints a `warn_once` but is otherwise a no-op. Requires geom position computation to implement. See `mujoco_pipeline.rs:6921`. |

### P1 — Performance / Quality

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 3 | Contact clone in hot path | ❌ Open | `data.contacts.clone()` at `mujoco_pipeline.rs:7778`. O(n_contacts) allocation per step. Could use index-based iteration instead. |
| 4 | Cholesky clone for implicit integrator | ❌ Open | `scratch_m_impl.clone().cholesky()` at `mujoco_pipeline.rs:8085`. O(nv²) allocation per step. Could use in-place factorization or sparse methods for serial chains. |
| 5 | Duplicate contact structures | ✅ Improved | Old `ContactPoint` in mujoco_pipeline.rs removed. Two distinct types remain: `Contact` (mujoco pipeline constraint struct) and `ContactPoint` (sim-contact geometric contact). These serve different purposes. |

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
