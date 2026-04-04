# Spec: Implicit Mass Correction for ImplicitFast/Implicit + Newton Solver

**Status:** Complete
**Commit:** `b1a34de`
**Key file:** `sim/L0/core/src/forward/mod.rs` (1-condition change)

---

## Problem

When `integrator=ImplicitFast` or `integrator=Implicit`, connect equality
constraints on ball-joint chains cause velocity explosion (77,328 rad/s within
10ms). The simulation freezes because `max_qvel` exceeds numerical limits.

**Reproduction:** 3-segment cable composite (`count="4 1 1"`) with a `<connect>`
constraint pinning `AB_last` to a fixed body. Uses `implicitfast` integrator,
Newton solver, iterations=200, tolerance=1e-12, ball joint damping=0.5.

The hanging-cable example (no connect constraint) works perfectly. Existing
connect examples with free joints also work. The bug is specific to connect
constraints on ball-joint chains with implicit integrators.

---

## Root Cause

In MuJoCo, `mj_implicit()` runs **before** `mj_fwdConstraint()` for all
implicit integrators (Implicit, ImplicitFast). This modifies the mass matrix
to `M_hat = M − h·∂f/∂v` (symmetrized for ImplicitFast), increasing the
effective inertia by incorporating velocity damping derivatives. The
constraint solver then uses M_hat for its Hessian, diagonal approximation,
and regularization.

In CortenForge, only `ImplicitSpringDamper` gets the implicit mass treatment
(`build_m_impl_for_newton`, line 139 of `constraint/mod.rs`). The gate is:

```rust
// constraint/mod.rs:334
let implicit_sd = model.integrator == Integrator::ImplicitSpringDamper;
```

This flag is checked at lines 343, 368, and 378 — controlling M_impl
construction, qacc_smooth override, and constraint assembly input. For
`ImplicitFast` and `Implicit`, all three checks are false.

### The cascade for ImplicitFast + Newton:

1. Constraint solver uses **base mass matrix M** (not M_hat)
   (`constraint/mod.rs:406` passes `data.qM.clone()` as `m_eff`)
2. Newton produces `qacc` assuming base M
3. `newton_solved = true` (constraint/mod.rs:419)
4. In `forward_acc()` (forward/mod.rs:551-553):
   ```rust
   if !self.newton_solved {
       acceleration::mj_fwd_acceleration(model, self)?;
   }
   ```
   → `mj_fwd_acceleration_implicitfast()` is **skipped entirely**
5. The implicit velocity-derivative correction is **never applied** to `qacc`
6. In `integrate()` (integrate/mod.rs:156-165), ImplicitFast does only:
   ```rust
   self.qvel[i] += self.qacc[i] * h;
   ```
   No matrix solve, no damping correction.

### Why the effective mass matters:

For ball-joint chains with tiny capsules (rotational inertia ~0.0002 kg·m²)
and joint damping 0.5, the implicit correction adds `h·damping = 0.0005` to
the effective inertia — a **3.5× increase**. Without it, constraint
accelerations are 3.5× too large, causing exponential velocity growth.

### Why other integrators work:

- **Euler:** Applies `eulerdamp` in `integrate()` (integrate/mod.rs:87-132):
  builds `(M + h·D)`, refactorizes, solves `(M+h·D)·qacc_new = qfrc_smooth +
  qfrc_constraint`. Same M_hat correction, different pipeline stage.
- **ImplicitSpringDamper:** `mj_fwd_constraint` builds M_impl via
  `build_m_impl_for_newton()` and passes it to Newton as `m_eff`.

### Test coverage gap:

Existing tests cover ImplicitSpringDamper + connect constraints
(`test_implicit_with_equality_constraints` in implicit_integration.rs:651)
and ImplicitFast + tendon damping (`test_implicitfast_tendon_damping_stability`
at line 715). But **no test exists for ImplicitFast + equality constraints**.

---

## Fix

In `Data::forward_acc()` (`sim/L0/core/src/forward/mod.rs:551`), always run
`mj_fwd_acceleration()` for ImplicitFast/Implicit integrators, even when
Newton succeeded:

```rust
// Before (line 551-553):
if !self.newton_solved {
    acceleration::mj_fwd_acceleration(model, self)?;
}

// After:
let needs_implicit_qacc = matches!(
    model.integrator,
    Integrator::ImplicitFast | Integrator::Implicit
);
if !self.newton_solved || needs_implicit_qacc {
    acceleration::mj_fwd_acceleration(model, self)?;
}
```

### What `mj_fwd_acceleration_implicitfast()` does (acceleration.rs:276-316):

1. Computes `qDeriv = ∂(qfrc_smooth)/∂(qvel)` via `mjd_passive_vel` + `mjd_actuator_vel`
2. Symmetrizes `qDeriv` → `D_sym`
3. Builds `M_hat = M − h·D_sym` into `data.scratch_m_impl`
   (note: `−h·(−damping) = +h·damping`, so M_hat > M)
4. Builds `RHS = data.qfrc_smooth + data.qfrc_constraint` (line 307-308)
5. Cholesky-factors M_hat in-place into `data.scratch_m_impl` (line 311)
6. Solves `M_hat · qacc = RHS` → overwrites `data.qacc` (lines 312-313)

### Data flow verification:

- **`data.qfrc_constraint` is valid:** Populated by
  `compute_qfrc_constraint_from_efc` (constraint/mod.rs:452) during
  `mj_fwd_constraint`, which returns before `forward_acc` reaches line 551.
- **RHS is built from scratch:** Line 307-308 copies `qfrc_smooth` then adds
  `qfrc_constraint`. No accumulation — Newton's qacc is fully overwritten.
- **No efc_* fields touched:** `efc_force`, `efc_state`, `efc_jar` are
  unchanged. Only `qacc`, `qDeriv`, `scratch_m_impl`, `scratch_rhs` are written.

### Bonus fix — derivative computation:

When Newton succeeds with ImplicitFast, `scratch_m_impl` is currently never
populated with Cholesky factors. But `mjd_transition_hybrid`
(derivatives/hybrid.rs:1846-1857) reads `scratch_m_impl` for ImplicitFast
derivative column solves. The fix populates `scratch_m_impl` as a side effect,
resolving this stale-data issue.

### Approximation note:

Newton's `efc_force` was computed using base M. With M_hat, the optimal
`efc_force` would differ slightly. The fix recomputes `qacc` with M_hat but
keeps Newton's `efc_force`. The error is O(h) in the constraint forces and
does not affect steady-state constraint satisfaction (Baumgarte corrects
drift). For full MuJoCo conformance, a follow-up spec should make Newton
use M_hat directly.

### Files changed:

`sim/L0/core/src/forward/mod.rs` only (1 condition change).

**No change needed for:**
- `ImplicitSpringDamper`: Newton already uses M_impl (existing path)
- `Euler`: eulerdamp provides the correction in `integrate()`
- `RK4`: no implicit treatment (explicit integrator)

---

## Acceptance Criteria

### AC1: Reproduction case is stable
Load the minimal 3-segment cable+connect MJCF. Step for 1000 steps (1 second).
`max(|qvel|) < 10 rad/s` at all times. Currently: 77,328 rad/s at 10ms.

### AC2: Cable endpoint stays near target
After settling (t > 5s), the connect constraint position error
`|pos1 - pos2| < 0.01m`. The cable tip must be near the "right" body.

### AC3: No regression in existing tests
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-urdf`
passes with zero failures.

### AC4: Hanging cable behavior unchanged
The hanging-cable example (no connect constraint) produces identical results.
When no constraints are active, `qfrc_constraint = 0` and
`mj_fwd_acceleration_implicitfast` produces the same qacc as Newton
(both solve M_hat · qacc = qfrc_smooth + 0 = qfrc_smooth). No behavioral
change.

### AC5: New integration test
Add `test_implicitfast_connect_ball_chain_stability` in
`sim/L0/tests/integration/implicit_integration.rs` covering the reproduction
case. Verify max_qvel < 10 rad/s after 1000 steps.

---

## Follow-Up (Separate Spec)

For full MuJoCo conformance, the constraint solver should use M_hat directly
in Newton's Hessian (matching how `ImplicitSpringDamper` uses
`build_m_impl_for_newton`). This requires:
1. Building M_hat for ImplicitFast/Implicit in `mj_fwd_constraint`
2. Passing M_hat to `newton_solve` as `m_eff`
3. Recomputing `qacc_smooth` using M_hat

This is a larger change that touches the constraint pipeline. The minimal fix
in this spec provides the stability benefit without the pipeline refactor.
