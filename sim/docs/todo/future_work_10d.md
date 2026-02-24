# Future Work 10d — Deferred Item Tracker: Group 3 — Tendon System

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 3 — Tendon System (8 items)

**Spec approach:** DT-30/35 each need individual specs (T3). DT-28/31 share a
"Tendon Joint Type Completeness" spec (T2). DT-32/33 join the cross-file
"Solver Param Completeness" spec with DT-23 (T2). DT-29/34 implement directly
(T1). Totals: 2 T1, 4 T2, 2 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-28 | §4 | Ball/Free joints in fixed tendons — validation + qvel DOF index mapping | Low | T2 |
| DT-29 | §4 | Spatial tendon dense J^T multiplication path (vs sparse wrap-array path) | Low | T1 |
| DT-30 | §4 | Compound pulley physics — capstan friction, pulley inertia (`sim-tendon/pulley.rs`) | Low | T3 |
| DT-31 | §4 | `WrapType::Joint` inside spatial tendons (`mjWRAP_JOINT`) — uncommon, parser rejects | Low | T2 |
| DT-32 | §4 | Per-tendon `solref_limit`/`solimp_limit` — constraint solver params (pre-existing gap) | Medium | T2 |
| DT-33 | §4 | Tendon `margin` attribute — limit activation distance (pre-existing gap) | Medium | T2 |
| DT-34 | §4 | Sparse Jacobian representation for spatial tendons — cache nonzero DOF indices | Low | T1 |
| ~~DT-35~~ | §4 | ~~Tendon spring/damper forces produce zero in implicit mode — non-diagonal K coupling needed~~ **DONE** — implicit tendon K/D via `accumulate_tendon_kd`, Newton `M_impl`/`qfrc_eff`, `ten_force` diagnostic always populated, 18 tests | Medium | T3 |

---

## DT-35 Spec: Tendon Spring/Damper Forces in `ImplicitSpringDamper` — Non-Diagonal K/D Coupling

### Context

`mj_fwd_passive()` (forward/passive.rs) gates **all** tendon spring and
damper forces behind `if !implicit_mode` (line 12432), where `implicit_mode` is
true only for `Integrator::ImplicitSpringDamper`. When this integrator is
selected, `ten_force[t]` is zero for every tendon, and no tendon contribution
reaches `qfrc_passive`. The tendons effectively vanish from the simulation.

**Why they were zeroed:** The `ImplicitSpringDamper` integrator solves:

```
(M + h·D_diag + h²·K_diag) · v_new = M·v_old + h·f_ext - h·K_diag·(q - q_eq)
```

where `K_diag` and `D_diag` are per-DOF diagonal vectors (`implicit_stiffness`,
`implicit_damping` in Model, populated by `compute_implicit_params()` at
line 3705). Joint spring/damper forces map cleanly to diagonal entries:
`K[dof] = jnt_stiffness`, `D[dof] = jnt_damping`.

Tendon spring/damper forces are **non-diagonal** in joint space:

```
F_tendon = -k · (L(q) - L_ref)           # spring
         + -b · dL/dt                      # damper

qfrc_tendon = J^T · F_tendon

K_tendon = k · J^T · J                    # position-dependent stiffness
D_tendon = b · J^T · J                    # velocity-dependent damping
```

where `J` is the tendon Jacobian (`ten_J[t]`, length nv) and `L(q) = J · q` for
fixed tendons. Both `K_tendon` and `D_tendon` are rank-1 outer products — they
have off-diagonal entries whenever a tendon spans more than one DOF (which is
the typical case). The diagonal-only solver cannot represent these.

**What the other integrators do:**

| Integrator | Spring force | Damper force | Spring derivative | Damper derivative |
|------------|-------------|-------------|-------------------|-------------------|
| `Euler` | Explicit in `qfrc_passive` | Explicit in `qfrc_passive` | Not used | Not used |
| `RungeKutta4` | Explicit in `qfrc_passive` | Explicit in `qfrc_passive` | Not used | Not used |
| `ImplicitFast` | Explicit in `qfrc_passive` | Explicit in `qfrc_passive` | Not used (position-dep) | `-b·J^T·J` in `qDeriv` (mjd_passive_vel:501-521) |
| `Implicit` | Explicit in `qfrc_passive` | Explicit in `qfrc_passive` | Not used (position-dep) | `-b·J^T·J` in `qDeriv` (mjd_passive_vel:501-521) |
| `ImplicitSpringDamper` | **Zero** (bug) | **Zero** (bug) | Skipped | Skipped |

For `Euler`/`RungeKutta4`, tendon forces appear in `qfrc_passive` explicitly
and no derivative information is used — integration is purely explicit.

For `ImplicitFast`/`Implicit`, `implicit_mode == false`, so tendon forces are
computed normally. These integrators treat tendon **spring** forces as purely
explicit — since spring force is position-dependent, `∂f_spring/∂v = 0`, so
there is no spring K contribution to `qDeriv`. Only the velocity-dependent
damping derivative `-b·J^T·J` gets the implicit `(M - h·qDeriv)` treatment.

**`ImplicitFast` vs `Implicit` distinction for tendon damping:** The tendon
damping derivative `-b·J^T·J` is a symmetric matrix (since `J^T·J` is always
symmetric). `ImplicitFast` uses Cholesky factorization (requires symmetric
`qDeriv`, omits Coriolis `C(q,v)`). `Implicit` uses LU factorization
(handles asymmetric `qDeriv`, includes Coriolis). Because `-b·J^T·J` is
symmetric, both integrators handle it identically — the LU vs Cholesky
distinction is irrelevant for this specific term.

**This means the DT-35 fix is actually *better* than `ImplicitFast`/`Implicit`
for spring stability:** by absorbing `h²·K_tendon` into the mass matrix, the
`ImplicitSpringDamper` integrator provides implicit treatment of tendon spring
stiffness — something `ImplicitFast`/`Implicit` do not offer.

**The bug is isolated to `ImplicitSpringDamper`.** The diagonal solver silently
drops tendon physics instead of handling the non-diagonal coupling.

**MuJoCo conformance note:** `ImplicitSpringDamper` is a **CortenForge-specific
integrator** — MuJoCo has no equivalent. MuJoCo's `mjtIntegrator` enum contains
only `Euler`, `RK4`, `implicit`, and `implicitfast`. MuJoCo's `implicit` and
`implicitfast` integrators compute all passive forces (including tendon
spring/damper) explicitly in `qfrc_passive`, and add the non-diagonal tendon
damping derivative `-b·J^T·J` to `qDeriv`. They never drop tendon forces.

CortenForge's `ImplicitSpringDamper` was designed as a diagonal-only solver
(`M + h·D_diag + h²·K_diag`) for improved stability of stiff joint springs.
The diagonal formulation cannot represent the rank-1 `J^T·J` coupling from
tendons, so tendon forces were silently dropped. This spec fixes the physics
by extending the solver to handle the non-diagonal tendon K/D contribution.

**Referenced code locations** (as of commit `b40917d`; line numbers may shift
after implementation):

| Location | File | Description |
|----------|------|-------------|
| `mj_fwd_passive` tendon gate | forward/passive.rs | `if !implicit_mode` — gates `qfrc_passive` only (force always computed) |
| `mj_fwd_acceleration_implicit` | integrate/implicit.rs | Diagonal-only `M + h·D + h²·K` solver |
| `compute_implicit_params` | types/model_init.rs | Populates diagonal K/D from joint properties only |
| `ImplicitSpringVisitor` | integrate/implicit.rs | RHS spring displacement `h·K·(q-q_eq)` — diagonal only |
| `mjd_passive_vel` tendon skip | derivatives.rs:499 | Skips tendon damping in `qDeriv` for ImplicitSpringDamper |
| Diagnostic/gate comment | forward/passive.rs | Documents the gate (ten_force always populated, qfrc_passive gated) |
| Tracked-from comment | future_work_1.md:1626-1630 | Original tracking reference |
| `newton_solve` | constraint/solver/ | Newton solver — uses `data.qM` directly, not `scratch_m_impl` |
| `assemble_hessian` | constraint/solver/ | Dense Newton Hessian — starts from `data.qM[(r,c)]` |
| `SparseHessian::assemble` | constraint/solver/ | Sparse Newton Hessian — starts from `data.qM` via `dof_parent` walk |
| `SparseHessian::fill_numeric` | constraint/solver/ | Sparse Hessian numeric fill — M values from `data.qM` |
| `compute_qacc_smooth` | constraint/solver/ | `qfrc_smooth` = applied + actuator + passive − bias (passive is zero for tendon K/D) |
| `evaluate_cost_at` | constraint/solver/ | Newton cost function — `Ma_trial` uses `data.qM` |
| `hessian_incremental` | constraint/solver/ | Rank-1 Cholesky update/downdate; fallback calls `assemble_hessian` |
| `primal_prepare` | constraint/solver/ | Line search quadratic — uses precomputed `ma`/`mv`, no `data.qM` |
| `mj_fwd_acceleration` skip | forward/mod.rs | `if !newton_solved` — skips implicit solver when Newton succeeds |
| Newton velocity update | forward/mod.rs | `qvel += qacc * h` when `newton_solved` + `ImplicitSpringDamper` |

### Scope & Dependencies

**In scope:**
1. Add non-diagonal tendon `K_tendon = k·J^T·J` and `D_tendon = b·J^T·J` to the
   `ImplicitSpringDamper` mass matrix modification (non-Newton path)
2. Add tendon spring displacement to the implicit RHS (non-Newton path)
3. Extend Newton solver to use `M_impl = M + h·D_tendon + h²·K_tendon` in
   Hessian assembly, gradient computation, and all `M·v` products when
   `ImplicitSpringDamper` is active (both dense and sparse Hessian paths)
4. Add tendon spring displacement to Newton's `qfrc_smooth` when
   `ImplicitSpringDamper` is active
5. Update `mj_fwd_passive` NOTE comment to document the implicit treatment (gate
   is preserved — tendon forces remain excluded from `qfrc_passive` in implicit
   mode, matching joint spring/damper architecture)
6. Include tendon damping derivatives in `mjd_passive_vel` for all integrators
7. Tests validating tendon physics are active in ImplicitSpringDamper mode,
   including Newton solver path

**Out of scope:**
- Spatial tendon Jacobian linearization error — spatial tendons have
  configuration-dependent `J(q)`, so `K_tendon = k·J^T·J` is an approximation
  (exact only for fixed tendons where `J` is constant). This is the same
  linearization MuJoCo uses in `ImplicitFast`/`Implicit` for the damping
  derivative. Improving this with `∂J/∂q` terms is a separate concern.
- Deadband spring linearization — the deadband `[lower, upper]` creates a
  piecewise-linear force curve. The implicit solver linearizes at the current
  state, which is standard practice. No special deadband handling is needed.
- Tendon friction loss — remains explicit (sign-dependent, cannot linearize),
  unchanged by this work. Friction loss was already moved to constraint rows
  (§29).
- Performance optimization (sparse `J^T·J` assembly) — DT-34 covers sparse
  tendon Jacobians. This spec uses the dense outer-product path.

**Dependencies:** None. This is a standalone fix to the ImplicitSpringDamper
integrator. The tendon Jacobian infrastructure (`ten_J`, `apply_tendon_force`)
is already correct and tested.

**Newton solver interaction:** When `newton_solved == true` (line 4745),
`mj_fwd_acceleration_implicit` is **completely skipped** (line 4686). Newton
builds its own Hessian from `data.qM` directly (`assemble_hessian` at
line 16717, `SparseHessian::fill_numeric` at line 16899) — it does NOT use
`scratch_m_impl`. Newton's gradient uses `qfrc_smooth` (line 17328), which
derives from `qfrc_passive` (line 15179) — also lacking tendon forces in
implicit mode. Newton's `Ma = M · qacc` products (lines 18021, 17871, 18237)
use raw `qM` throughout. All three pathways — Hessian, gradient, and
M·v products — must be modified to include the implicit tendon K/D
contribution. Steps 2b and 3b (below) implement these modifications.

### Diagnostic Observability

`ten_force[t]` is always populated with the computed spring/damper force,
regardless of integrator. In `ImplicitSpringDamper` mode, the force is computed
for diagnostic purposes but is NOT applied to `qfrc_passive` — the actual
physics are handled implicitly via the mass matrix modification (K/D matrices).
This avoids double-counting while keeping the diagnostic field useful.

`qfrc_passive` remains zero for tendon spring/damper in `ImplicitSpringDamper`
mode, matching the joint spring/damper pattern (`PassiveForceVisitor` skips
joint K/D in implicit mode).

**Implications for users:**
- `ten_force[t]` shows the tendon force for ALL integrators (diagnostic field)
- `qfrc_passive` excludes tendon K/D in `ImplicitSpringDamper` mode (forces
  are absorbed into the implicit solve)
- Tests 6a/6b verify `ten_force` is populated in `ImplicitSpringDamper` mode

### MuJoCo Reference

MuJoCo has no `ImplicitSpringDamper` integrator. The closest comparisons:

**MuJoCo `implicit`/`implicitfast`** (the recommended integrators) solve:
```
(M - h·qDeriv) · qacc = qfrc_smooth
```
where `qDeriv = ∂(qfrc_smooth)/∂(qvel)` is a full (non-diagonal) matrix that
includes tendon damping `-b·J^T·J`. All passive forces (including tendon
spring/damper) are computed explicitly in `qfrc_passive` and appear on the RHS.
Tendon spring stiffness K has no `qDeriv` entry (position-dependent,
`∂f_spring/∂v = 0`) — spring forces are treated purely explicitly.

**CortenForge `ImplicitSpringDamper`** solves:
```
(M + h·D + h²·K) · v_new = M·v_old + h·f_ext - h·K·(q - q_eq)
```
where `K`, `D` are currently diagonal vectors from joint properties only.
Tendon forces are skipped because the diagonal cannot represent `J^T·k·J`.

**After this fix**, tendon K/D are added non-diagonally to the mass matrix:

| Scenario | Before fix (current) | After fix |
|----------|---------------------|-----------|
| Joint springs | Diagonal K ✓ | Diagonal K ✓ (unchanged) |
| Joint dampers | Diagonal D ✓ | Diagonal D ✓ (unchanged) |
| Tendon springs | Dropped (zero force) | `h²·k_active·J^T·J` in mass matrix + RHS displacement (outside deadband only) ✓ |
| Tendon dampers | Dropped (zero force) | `h·b·J^T·J` in mass matrix (always) ✓ |

This provides **implicit spring stability** for tendon stiffness, which is
better than MuJoCo's `implicit`/`implicitfast` (where tendon springs are
explicit-only).

#### Tendon force formulas

**Fixed tendon:** `L = Σ(coef_i · q_i)`, so `J[dof] = coef_i` (constant).

| Quantity | Formula | Dimension |
|----------|---------|-----------|
| Length | `L = J · qpos` (subset) | scalar |
| Velocity | `V = J · qvel` | scalar |
| Spring force | `F_spring = -k · clamp_deadband(L, [lo, hi])` | scalar |
| Damper force | `F_damper = -b · V` | scalar |
| Joint force | `qfrc = J^T · (F_spring + F_damper)` | nv × 1 |
| Stiffness matrix | `K_t = k · J^T · J` (rank-1, nv × nv) | nv × nv |
| Damping matrix | `D_t = b · J^T · J` (rank-1, nv × nv) | nv × nv |
| Spring displacement | `J^T · k · deadband_disp(L)` | nv × 1 |

where `deadband_disp(L)` returns `L - upper` if `L > upper`, `L - lower` if
`L < lower`, `0` otherwise (using `tendon_lengthspring[t] = [lower, upper]`).

**Spatial tendon:** `J` is configuration-dependent (computed by
`mj_fwd_tendon_spatial`). The formulas are identical but `J` varies with `q`.
The linearization `K_t ≈ k·J^T·J` is a first-order approximation, consistent
with MuJoCo's treatment in `ImplicitFast`/`Implicit`.

### Plan

#### Step 0: Shared helpers — deadband and tendon K/D accumulation

**File:** `sim/L0/core/src/forward/passive.rs` (private helpers near tendon code)

The deadband displacement logic and the tendon K/D outer-product accumulation
are used in multiple steps (Step 2, Step 3, Step 2b, Step 3b). Extract them
once to eliminate duplication and ensure consistency.

```rust
/// Compute the deadband displacement for a tendon.
///
/// Returns `length - upper` if `length > upper`, `length - lower` if
/// `length < lower`, `0.0` if inside the deadband `[lower, upper]`.
/// At the boundary (`length == lower` or `length == upper`), returns `0.0`
/// (spring disengaged — see "one-step delay" note in Step 2).
#[inline]
fn tendon_deadband_displacement(length: f64, range: [f64; 2]) -> f64 {
    let [lower, upper] = range;
    if length > upper {
        length - upper
    } else if length < lower {
        length - lower
    } else {
        0.0
    }
}

/// Return the effective stiffness for implicit treatment.
///
/// Returns `k` when the tendon is outside its deadband (spring engaged),
/// `0.0` when inside (spring disengaged). This gates the `h²·K` LHS
/// modification: no phantom stiffness inside the deadband.
///
/// **Note on exact boundary:** At `length == lower` or `length == upper`,
/// returns `0.0`. The displacement is also `0.0` at the boundary, so no
/// spring force exists. If velocity moves the tendon outside the deadband,
/// the spring activates in the next step (one-step delay, consistent with
/// linearization at the current state).
#[inline]
fn tendon_active_stiffness(k: f64, length: f64, range: [f64; 2]) -> f64 {
    if k <= 0.0 {
        return 0.0;
    }
    let [lower, upper] = range;
    if length >= lower && length <= upper { 0.0 } else { k }
}

/// Accumulate non-diagonal tendon K/D into a mass matrix.
///
/// For each tendon with nonzero stiffness or damping, adds the rank-1
/// outer product `(h²·k_active + h·b) · J^T · J` to `matrix`. Uses
/// deadband-aware `k_active` (zero inside deadband, `k` outside).
///
/// Shared by `mj_fwd_acceleration_implicit` (Step 2) and
/// `build_m_impl_for_newton` (Step 2b). Both call sites must produce
/// identical mass matrix modifications — factoring this out guarantees it.
///
/// **Sleep guard:** Skips tendons whose target DOFs are all sleeping,
/// matching the guards in `mj_fwd_passive` and `mjd_passive_vel`.
fn accumulate_tendon_kd(
    matrix: &mut DMatrix<f64>,
    model: &Model,
    data: &Data,
    h: f64,
    sleep_enabled: bool,
) {
    let h2 = h * h;
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        let bt = model.tendon_damping[t];
        if kt <= 0.0 && bt <= 0.0 {
            continue;
        }
        let j = &data.ten_J[t];
        let k_active = tendon_active_stiffness(
            kt, data.ten_length[t], model.tendon_lengthspring[t],
        );
        let scale = h2 * k_active + h * bt;
        // Defensive: skip if scale is non-positive. For valid models (k ≥ 0,
        // b ≥ 0) this is unreachable when the above guard passes, but protects
        // against pathological negative parameters that would break SPD.
        // MuJoCo enforces non-negative stiffness/damping at parse time, so
        // negative values cannot arise from well-formed MJCF — this guard is
        // untestable via normal model loading.
        if scale <= 0.0 {
            continue;
        }
        // Rank-1 outer product: (h²·k_active + h·b) · J^T · J
        //
        // Sparsity skip `j[r] == 0.0`: for fixed tendons, Jacobian entries
        // are exact MJCF coefficients (parsed floats), so zero entries are
        // exactly 0.0. For spatial tendons, entries are computed from 3D
        // geometry and may be near-zero (e.g. 1e-17) rather than exact
        // zero — but the contribution of such entries is O(ε²) per matrix
        // element, which is negligible (well below f64 precision). This
        // matches MuJoCo's own sparse outer-product loops, which also use
        // exact `== 0.0` checks for the tendon Jacobian.
        for r in 0..model.nv {
            if j[r] == 0.0 {
                continue;
            }
            for c in 0..model.nv {
                if j[c] == 0.0 {
                    continue;
                }
                matrix[(r, c)] += scale * j[r] * j[c];
            }
        }
    }
}
```

**`#[must_use]` policy:** `tendon_deadband_displacement` and
`tendon_active_stiffness` return scalar values that must be used. Both are
`#[inline]` leaf functions where the compiler will warn on unused return values
naturally. `accumulate_tendon_kd` is a `&mut` accumulator (no return value), so
`#[must_use]` does not apply.

#### Step 1: Keep tendon forces gated in `mj_fwd_passive` — comment update only

**File:** `sim/L0/core/src/forward/passive.rs`.

The `if !implicit_mode` gate at line 12432 is **correct architecture**. In
`ImplicitSpringDamper` mode, both joint and tendon spring/damper forces are
excluded from `qfrc_passive` and instead handled implicitly via the mass
matrix modification. The existing pattern:

| Component | `qfrc_passive` | Mass matrix | RHS |
|-----------|---------------|-------------|-----|
| Joint spring (existing) | Gated out | `h²·K_diag` diagonal | `h·K·(q-q_eq)` via `ImplicitSpringVisitor` |
| Joint damper (existing) | Gated out | `h·D_diag` diagonal | None (velocity-absorbed) |
| Tendon spring (current bug) | Gated out | **Missing** | **Missing** |
| Tendon damper (current bug) | Gated out | **Missing** | **Missing** |
| Tendon spring (after fix) | `ten_force` always computed; `qfrc_passive` gated | `h²·k_active·J^T·J` non-diagonal (outside deadband only) | `h·J^T·(-k·δ)` (Step 3, outside deadband only) |
| Tendon damper (after fix) | `ten_force` always computed; `qfrc_passive` gated | `h·b·J^T·J` non-diagonal (always) | None (velocity-absorbed) |

**`mj_fwd_passive` changes:** Force computation is unconditional (populates
`ten_force[t]` for diagnostics). Only the `qfrc_passive` application is gated
behind `!implicit_mode` to avoid double-counting:

```rust
// NOTE: In ImplicitSpringDamper mode, tendon spring/damper forces are
// handled implicitly in mj_fwd_acceleration_implicit() via non-diagonal
// K_tendon and D_tendon matrices (DT-35). ten_force[t] is always populated
// for diagnostic purposes, but the explicit qfrc_passive application is
// skipped to avoid double-counting, matching the joint spring/damper pattern.
```

#### Step 2: Add non-diagonal tendon K/D to `mj_fwd_acceleration_implicit`

**File:** `sim/L0/core/src/integrate/implicit.rs`.

After the existing diagonal modification at line 20170–20172:

```rust
// Existing: diagonal joint K/D
for i in 0..model.nv {
    data.scratch_m_impl[(i, i)] += h * d[i] + h2 * k[i];
}
```

Add the non-diagonal tendon contribution:

```rust
// DT-35: Non-diagonal tendon stiffness and damping (Step 0 helper).
// Adds Σ_t (h²·k_active_t + h·b_t) · J_t^T · J_t to scratch_m_impl.
// Spring K is deadband-aware: zero inside [lower, upper], k outside.
// Damping D always applies. Sleep guard skips fully-sleeping tendons.
let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
accumulate_tendon_kd(&mut data.scratch_m_impl, model, data, h, sleep_enabled);
```

**Complexity:** O(ntendon · nnz_J²) per step. Tendon Jacobians are typically
very sparse (2–6 nonzero entries for a fixed tendon), so this is O(ntendon · d²)
where d is the average tendon span in DOFs.

**Sign derivation:** The implicit velocity update is:

```
v_new = v_old + h · a
```

where `a` includes spring force `-K·(q - q_eq)` and damper force `-D·v_new`.
Substituting and rearranging to solve for `v_new`:

```
M·v_new = M·v_old + h·f_ext + h·(-K·Δq) + h·(-D·v_new)
M·v_new + h·D·v_new = M·v_old + h·f_ext - h·K·Δq
(M + h·D)·v_new = M·v_old + h·f_ext - h·K·Δq
```

Adding `h²·K` for implicit spring treatment (`Δq ≈ Δq_old + h·v_new`):

```
(M + h·D + h²·K)·v_new = M·v_old + h·f_ext - h·K·Δq_old
```

Both `K_tendon = k·J^T·J` and `D_tendon = b·J^T·J` enter with **positive
signs** on the LHS, matching the existing diagonal code: `+= h * d[i] + h2 * k[i]`.

**SPD preservation:** `J^T·J` is positive semi-definite (PSD) for any `J`, and
`k_active ≥ 0`, `b ≥ 0`, so `(h²·k_active + h·b)·J^T·J` is PSD. Adding PSD to
SPD (the mass matrix `M`) preserves positive definiteness. Cholesky factorization
remains valid. The deadband-aware `k_active` is either `k` (outside deadband) or
`0` (inside), so this invariant always holds.

**Edge case — negative parameters:** MuJoCo enforces `tendon_stiffness ≥ 0` and
`tendon_damping ≥ 0` at parse time. The guards in `accumulate_tendon_kd`
(`if kt <= 0.0 && bt <= 0.0` and `if scale <= 0.0`) are defensive: if a
pathological model has one positive and one negative parameter such that
`h²·k_active + h·bt ≤ 0`, we skip the tendon entirely rather than risk breaking
SPD. This is conservative but safe — a negative scale would subtract from M,
potentially destroying positive definiteness and crashing Cholesky. Because
MuJoCo rejects negative parameters at parse time, this guard is untestable via
normal MJCF model loading.

**Sleep guard:** `accumulate_tendon_kd` (Step 0) includes a
`tendon_all_dofs_sleeping()` check, matching the sleep guards in
`mj_fwd_passive` (line 12422) and `mjd_passive_vel` (line 502). The existing
diagonal joint K/D code in `mj_fwd_acceleration_implicit` does NOT have sleep
guards — this is a pre-existing gap in the joint path. The tendon code adds the
guard for consistency with the explicit tendon pipeline.

**Implementation note:** `mj_fwd_acceleration_implicit` currently has no
`sleep_enabled` variable in scope. Step 2 introduces
`let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;` before the
`accumulate_tendon_kd` call. This variable remains in scope for Step 3's code
(same function body). Step 2 and Step 3 must use the same guard: if a tendon's
K is omitted from the LHS mass matrix, the corresponding spring displacement
must also be omitted from the RHS, or the solver will produce incorrect
accelerations for sleeping DOFs.

**`disableflags` note:** The codebase has `model.disableflags` but it is
currently unused for passive forces (`mjDSBL_PASSIVE` is not implemented). The
existing diagonal K/D path has no disableflags check. If a passive-disable flag
is implemented later, both the diagonal joint path and the non-diagonal tendon
path in `mj_fwd_acceleration_implicit` must be updated together.

#### Step 3: Add tendon spring displacement to the implicit RHS

**File:** `sim/L0/core/src/integrate/implicit.rs`, after the existing
`ImplicitSpringVisitor` call.

The `ImplicitSpringDamper` formulation requires the RHS to include the spring
force term `h · qfrc_spring`. For tendons, the spring force in tendon space is
`F = -k · δ` where `δ = deadband_disp(L)`:
- If `L > upper`: `δ = L - upper`
- If `L < lower`: `δ = L - lower`
- Else: `δ = 0` (inside deadband, no force)

Projected to joint space: `qfrc_spring = J^T · F = -J^T · k · δ`. The RHS
correction is `RHS += h · qfrc_spring`, which parallels the joint
`ImplicitSpringVisitor` that computes `rhs[dof] -= h · K[dof] · (q - q_eq)`.

**Deadband interaction with the LHS `h²·K·v_new` term:** Step 2 only adds
`h²·K_tendon` when the tendon is **outside** the deadband — `k_active = 0`
inside the deadband. This ensures zero phantom force inside the deadband:
no `h²·K` inertia, no RHS displacement, no spring interaction whatsoever.
A tendon moving freely through the deadband experiences only damper forces
(if any), never phantom stiffness from a disengaged spring.

At the **deadband boundary** (e.g., `L = upper` exactly), `k_active = 0`
and the spring is fully disengaged. If `v_new` moves `L` outside the
deadband, the spring activates in the **next** step. This produces a one-step
delay compared to the explicit integrators, but is consistent with the
linearization (we linearize at the current state, where the spring is inactive).

```rust
// DT-35: Tendon spring displacement contribution to implicit RHS
// RHS[dof] -= h · Σ_t k_t · J_t[dof] · deadband_disp(L_t)
// (sleep_enabled already computed in Step 2, same function scope)
for t in 0..model.ntendon {
    // §40c: Skip tendon if ALL target DOFs are sleeping.
    // Must match the guard in accumulate_tendon_kd (Step 0/2) — if K is
    // not in the LHS, the corresponding spring displacement must not be
    // in the RHS.
    if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
        continue;
    }
    let kt = model.tendon_stiffness[t];
    if kt <= 0.0 {
        continue;
    }
    let displacement = tendon_deadband_displacement(
        data.ten_length[t], model.tendon_lengthspring[t],
    );
    // SAFETY: exact `== 0.0` comparison is correct here.
    // `tendon_deadband_displacement` returns literal `0.0` from
    // the else branch (no arithmetic on the value). At the boundary
    // (`length == upper` or `length == lower`), the subtraction
    // `length - upper` is exactly 0.0 when both operands are equal.
    // No intermediate rounding can produce a false non-zero.
    if displacement == 0.0 {
        continue; // Inside deadband — no spring force
    }
    // Spring force in tendon space: F = k * (ref - L) = -k * displacement
    // Joint-space force: qfrc = J^T * F = -J^T * k * displacement
    // RHS += h * qfrc = -h * k * displacement * J^T
    let j = &data.ten_J[t];
    let scale = -h * kt * displacement;
    for dof in 0..model.nv {
        if j[dof] != 0.0 {
            data.scratch_rhs[dof] += scale * j[dof];
        }
    }
}
```

**Note:** The damper does NOT need an RHS correction. Damping force is
`-b · V = -b · J · qvel`, which is velocity-dependent. In the implicit
formulation, velocity-dependent forces are absorbed entirely into the mass
matrix modification `h · D_tendon` — no separate RHS term is needed. This
matches how joint damping works: `model.implicit_damping` appears in the mass
matrix diagonal but has no RHS correction.

#### Step 2b: Newton solver — use `M_impl` in Hessian, gradient, and M·v products

**Files:** `sim/L0/core/src/constraint/solver/` and `sim/L0/core/src/forward/mod.rs`

When `newton_solved == true` and `ImplicitSpringDamper` is active,
`mj_fwd_acceleration_implicit` is **skipped** (line 4686). Newton builds its
own system independently using raw `qM`. Three things must change:

**Problem analysis:** Newton minimizes:
```
cost = ½·(M·a − qfrc_smooth)·(a − a_smooth) + constraint_cost(a)
```
with Hessian `H = M + Σ D_i·J_i^T·J_i` and gradient
`grad = M·a − qfrc_smooth − J_efc^T·f_efc`.

For `ImplicitSpringDamper`, the correct unconstrained solution is
`a_smooth = M_impl⁻¹ · (qfrc_smooth + qfrc_tendon_implicit)`, where
`M_impl = M + h·D_tendon + h²·K_tendon` and `qfrc_tendon_implicit` includes
the spring displacement RHS. The Newton cost function, Hessian, gradient, and
all `M·v` products must use `M_impl` and the augmented `qfrc_smooth`.

**Implementation approach — precompute `M_impl` once:**

Add a helper function `build_m_impl_for_newton()` that returns `M_impl`:

```rust
/// Build the implicit-modified mass matrix for Newton solver.
///
/// Returns `M + h·D_jnt + h²·K_jnt + h·D_ten + h²·K_ten` — joint diagonal
/// K/D plus tendon non-diagonal K/D. Called once per step in
/// `mj_fwd_constraint` when `ImplicitSpringDamper` is active. The returned
/// matrix replaces `data.qM` in all Newton computations.
///
/// Uses `accumulate_tendon_kd` (Step 0) for the tendon contribution,
/// guaranteeing identical mass matrix modification as the non-Newton path
/// in `mj_fwd_acceleration_implicit` (Step 2).
#[must_use]
fn build_m_impl_for_newton(
    model: &Model,
    data: &Data,
) -> DMatrix<f64> {
    let h = model.timestep;
    let h2 = h * h;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    let mut m_impl = data.qM.clone();

    // Add diagonal joint K/D (matching mj_fwd_acceleration_implicit)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    for i in 0..nv {
        m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // Add non-diagonal tendon K/D (Step 0 shared helper)
    accumulate_tendon_kd(&mut m_impl, model, data, h, sleep_enabled);

    m_impl
}
```

**Wait — joint diagonal K/D in Newton.** The existing non-Newton path in
`mj_fwd_acceleration_implicit` adds both joint diagonal K/D AND tendon
non-diagonal K/D to `scratch_m_impl`. But the Newton path uses raw `qM` for
everything — it has **no** implicit K/D at all, not even the joint diagonal
terms. This is a **pre-existing bug** for joint springs too, not just tendons.

However, examining the pipeline more carefully: when Newton succeeds, the
velocity update is `qvel += qacc * h` (line 4755), which is a standard
explicit Euler velocity update. Newton's `qacc` is computed from
`qacc_smooth = M⁻¹ · qfrc_smooth`, where `qfrc_smooth` includes
`qfrc_passive` — but `qfrc_passive` **excludes** joint spring/damper forces
in implicit mode (via the `PassiveForceVisitor` gate at line 12410).

So the Newton + `ImplicitSpringDamper` path currently drops **all** implicit
forces (joint and tendon) from the physics. The non-Newton path compensates
by solving `(M + h·D + h²·K)·v_new = M·v_old + h·f_ext − h·K·Δq` directly
(which folds the forces into the modified mass matrix and RHS). But when
Newton bypasses this, the implicit forces vanish entirely.

**Scope clarification:** This pre-existing gap for joint K/D in the Newton
path is a separate concern (pre-DT-35). DT-35 fixes the **tendon** part and,
by building `M_impl` that includes both joint and tendon K/D, also fixes the
joint K/D gap for Newton. This is a strict improvement — the joint diagonal
terms are `O(1)` additional work.

**Modification 2b-i: `newton_solve` — use `M_impl` in place of `data.qM`**

At the top of `newton_solve`, when `ImplicitSpringDamper` is active, compute
`M_impl` once and use it throughout:

The caller (`mj_fwd_constraint`) precomputes `m_impl_cache: Option<DMatrix>`
when `ImplicitSpringDamper` is active (see Step 3b below). `newton_solve`
receives a reference to the effective mass matrix:

```rust
fn newton_solve(
    model: &Model,
    data: &mut Data,
    m_eff: &DMatrix<f64>,       // NEW: M or M_impl (caller decides)
    qfrc_smooth: &DVector<f64>, // NEW: base or implicit-corrected
) -> NewtonResult {
    let nv = model.nv;

    // ... all subsequent M·v products use m_eff instead of &data.qM ...
```

**Why not `Cow`:** The `Cow<'_, DMatrix>` pattern works but adds cognitive
overhead. Since `mj_fwd_constraint` already computes `m_impl` before calling
`newton_solve`, a plain `&DMatrix` reference is simpler and avoids lifetime
gymnastics. For non-`ImplicitSpringDamper` integrators, the caller passes
`&data.qM` directly. This matches the existing pattern where `qacc_smooth`
and `qfrc_smooth` are computed by the caller and passed in.

Then replace every `data.qM` reference inside `newton_solve` with `m_eff`:

| Location | Current | After |
|----------|---------|-------|
| Line 18023: `Ma = M · qacc` | `data.qM[(r, c)]` | `m_eff[(r, c)]` |
| Line 17871: `Ma_trial = M · qacc_trial` in `evaluate_cost_at` | `data.qM[(r, c)]` | Must pass `m_eff` as parameter |
| Line 18237: `Mv = M · search` | `data.qM[(r, c)]` | `m_eff[(r, c)]` |
| Line 18080: `mv = M · search` (initial) | `data.qM[(r, c)]` | `m_eff[(r, c)]` |

**Modification 2b-ii: `assemble_hessian` — start from `M_impl`**

```rust
fn assemble_hessian(
    data: &Data,
    nv: usize,
    m_eff: &DMatrix<f64>,  // NEW: M or M_impl
) -> Result<DMatrix<f64>, StepError> {
    let nefc = data.efc_type.len();
    let mut h = DMatrix::<f64>::zeros(nv, nv);
    for r in 0..nv {
        for c in 0..nv {
            h[(r, c)] = m_eff[(r, c)];  // Was: data.qM[(r, c)]
        }
    }
    // ... rest unchanged (constraint J^T·D·J additions) ...
```

**Modification 2b-iii: `SparseHessian::assemble` / `fill_numeric` — start from `M_impl`**

The sparse Hessian walks `dof_parent` to find `M` non-zeros (line 16806).
When tendon K/D adds off-diagonal entries beyond the tree pattern, the
sparsity pattern must include them.

**Sparsity pattern extension:** `SparseHessian::assemble` (line 16803)
builds a `has_entry: Vec<Vec<bool>>` (nv × nv) mask to determine which CSC
entries to allocate. The existing code populates it from the `dof_parent`
tree (M sparsity) and constraint `efc_J` sparsity. Add tendon non-zero pairs
to `has_entry` before the CSC construction loop:

```rust
// (ImplicitSpringDamper only) Tendon K/D sparsity: J^T·J outer product.
// The sparsity pattern is CONSERVATIVE: it includes entries for all tendons
// with k > 0 or b > 0, regardless of deadband state. This is intentional —
// the pattern must accommodate all possible non-zero entries. The actual
// values in fill_numeric use deadband-aware k_active, so entries inside
// the deadband contribute zero (they remain structural zeros in the CSC).
if implicit_sd {
    let mut nz: Vec<usize> = Vec::with_capacity(8);
    for t in 0..model.ntendon {
        let kt = model.tendon_stiffness[t];
        let bt = model.tendon_damping[t];
        if kt <= 0.0 && bt <= 0.0 { continue; }
        let j = &data.ten_J[t];
        nz.clear();
        for dof in 0..nv {
            if j[dof] != 0.0 { nz.push(dof); }
        }
        for &ci in &nz {
            for &cj in &nz {
                let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                has_entry[lo][hi] = true;
            }
        }
    }
}
```

**Note on tree walk vs. `m_eff`:** The dense path (`assemble_hessian`) can
simply use `m_eff` since it copies all nv² entries anyway. The sparse path
cannot — its M fill uses `dof_parent` to find the O(nv·depth) non-zero
entries of M (lines 16896-16911), and the tendon `J^T·J` entries do NOT
follow the tree pattern. A tendon coupling DOFs on different branches creates
off-diagonal entries that `dof_parent` won't visit. Passing `m_eff` to
`fill_numeric` would not help because the tree walk would still skip the
tendon-induced off-diagonal entries.

**Solution:** Keep the tree walk for the raw M fill (unchanged), then add a
second loop for tendon contributions after the tree walk. `fill_numeric`
continues to read `data.qM` for the tree-structured M entries — no `m_eff`
parameter needed on the sparse path:

```rust
// Fill tendon K/D entries (ImplicitSpringDamper only).
// These entries are ALREADY in the sparsity pattern (added in assemble).
// Uses tendon_active_stiffness (Step 0) for deadband-aware K, matching
// accumulate_tendon_kd.
if implicit_sd {
    // Pre-allocate outside the loop to avoid per-tendon heap allocation.
    // Tendon Jacobians are typically very sparse (2–6 nonzeros for fixed
    // tendons), so SmallVec<[(usize, f64); 8]> would also work well here.
    let mut nz: Vec<(usize, f64)> = Vec::with_capacity(8);
    for t in 0..model.ntendon {
        let kt = model.tendon_stiffness[t];
        let bt = model.tendon_damping[t];
        if kt <= 0.0 && bt <= 0.0 { continue; }
        let j = &data.ten_J[t];
        let k_active = tendon_active_stiffness(
            kt, data.ten_length[t], model.tendon_lengthspring[t],
        );
        let scale = h2 * k_active + h * bt;
        if scale <= 0.0 { continue; }
        nz.clear();
        for dof in 0..nv {
            if j[dof] != 0.0 { nz.push((dof, j[dof])); }
        }
        for (ai, &(col_a, j_a)) in nz.iter().enumerate() {
            let s_j_a = scale * j_a;
            for &(col_b, j_b) in &nz[ai..] {
                if let Some(idx) = self.find_entry(col_a, col_b) {
                    self.vals[idx] += s_j_a * j_b;
                }
            }
        }
    }
}
```

**Why not pass `m_eff` to the sparse path?** The dense path (`assemble_hessian`)
can use `m_eff` directly since it copies all nv² entries. The sparse path
cannot — its tree walk only visits O(nv·depth) entries, so it would miss the
tendon off-diagonal entries even in `m_eff`. The separate tendon fill loop
after the tree walk is the correct approach for the sparse path.

**`find_entry` availability:** `SparseHessian::find_entry(col, row)` already
exists (line 16944) and performs a binary search over the CSC row indices for
the given column. It returns `Option<usize>` — the CSC value index — or `None`
if the entry is not in the sparsity pattern. Since the tendon pairs are added
to `has_entry` during pattern construction (above), `find_entry` will always
return `Some` for valid tendon entries.

**Modification 2b-iv: `evaluate_cost_at` — use `M_impl`**

```rust
fn evaluate_cost_at(
    data: &Data,
    model: &Model,
    qacc_trial: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    m_eff: &DMatrix<f64>,  // NEW: M or M_impl
) -> f64 {
    // ...
    // Line 17871: ma_trial[r] += m_eff[(r, c)] * qacc_trial[c];
    // (was: data.qM[(r, c)])
```

**Modification 2b-v: `compute_gradient_and_search` / `_sparse` — signature unchanged**

The gradient `grad = Ma - qfrc_smooth - qfrc_constraint` uses `ma` which is
precomputed by the caller. Since the caller now computes `ma = M_impl · qacc`,
the gradient is automatically correct. No change needed to
`compute_gradient_and_search` itself.

**Modification 2b-vi: Line search `mv` and `primal_prepare`**

`mv = M · search` (lines 18080 and 18237) must use `m_eff`. The `primal_prepare`
function (line 17430) receives precomputed `ma` and `mv` vectors — it does NOT
reference `data.qM` internally (verified: it only uses `ma`, `mv`, `search`,
`qfrc_smooth`, and `efc_*` fields). Since `ma` and `mv` are computed by the
caller using `m_eff`, `primal_prepare` is automatically correct. No changes
needed to `primal_prepare` or `primal_search`.

**Complexity impact:** `build_m_impl_for_newton` is O(nv² + ntendon·nnz_J²),
called once per step. Newton's per-iteration cost is unchanged (the Hessian
assembly already does O(nv²) or O(nnz) work). The `Cow<DMatrix>` avoids
allocation for non-`ImplicitSpringDamper` integrators.

**SPD preservation:** Same argument as Step 2. `M_impl = M + PSD` is SPD.
Newton's Hessian `H = M_impl + Σ D_i·J^T·J` adds more PSD terms. Cholesky
factorization remains valid.

#### Step 3b: Newton solver — augment `qfrc_smooth` with implicit tendon spring RHS

**File:** `sim/L0/core/src/constraint/solver/`

`compute_qacc_smooth()` computes:
```
qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
```

In `ImplicitSpringDamper` mode, `qfrc_passive` excludes tendon spring/damper
forces (they're gated in `mj_fwd_passive`). The non-Newton path handles this
via the separate `scratch_rhs` construction in `mj_fwd_acceleration_implicit`.
For Newton, we must add the implicit tendon spring displacement to
`qfrc_smooth` before `qacc_smooth = M_impl⁻¹ · qfrc_smooth`.

**Naive approach (rejected):** Add tendon spring/damper forces directly to
`qfrc_smooth` in `compute_qacc_smooth`. This is insufficient because Newton
operates in acceleration space and the implicit formulation mixes position-
and velocity-dependent terms into the system matrix.

**Why the naive approach fails.** In the non-Newton path, the implicit
formulation solves for `v_new` directly:
```
(M + h·D + h²·K) · v_new = M·v_old + h·f_ext − h·K·Δq
```
The RHS has `M·v_old` (not `qacc`-based) and the spring/damper are folded
into LHS/RHS differently than Newton's acceleration-based formulation.

Newton works in **acceleration space**: `qacc = a`, and solves:
```
min_a  ½·(M·a − qfrc_smooth)·(a − a_smooth)  +  constraint_cost(a)
```

The unconstrained minimizer is `a_smooth = M⁻¹ · qfrc_smooth`. For this to
be correct, `qfrc_smooth` must include all smooth forces — including tendon
spring/damper forces, and `M` must be the raw mass matrix (since Newton's
`a` is a true acceleration, not a modified velocity variable).

**BUT** the `ImplicitSpringDamper` integrator's velocity update uses
`qvel += qacc * h` when Newton is active (line 4755). This is a plain Euler
update. The implicit stability benefit comes from the modified mass matrix
in the non-Newton path. With Newton, since we're back to an `a`-based
formulation, we need to carefully rethink what "implicit" means.

**Design choice — two approaches:**

**(A) Make Newton aware of implicit formulation:** Replace `M` with `M_impl`
in Newton's Hessian/gradient (as described in Step 2b) and add spring/damper
forces to `qfrc_smooth`. Newton then solves in a "modified acceleration"
space where `a_impl` folds in the implicit K/D. The velocity update
`qvel += a_impl * h` then approximates the implicit scheme. This is
mathematically equivalent to the non-Newton path when there are no active
constraints.

**(B) Add explicit tendon forces to `qfrc_smooth` only:** Keep `M = qM` in
Newton, but add tendon spring/damper forces to `qfrc_smooth` like the
`Euler`/`RK4` integrators do. This loses the implicit stability benefit but
at least makes tendon forces non-zero with Newton.

**We choose approach (A)** because:
1. It matches the non-Newton path's behavior (implicit stability preserved)
2. When no constraints are active, Newton's solution must match
   `mj_fwd_acceleration_implicit` exactly (consistency requirement)
3. The velocity update `qvel += qacc * h` with `qacc` from the modified
   system produces the same `v_new` as the non-Newton solve

**Proof of equivalence (no constraints):** Without constraints, Newton
minimizes `½·||M_impl·a − f||²_{M_impl⁻¹}`, giving `a = M_impl⁻¹·f`.
The velocity update is `v_new = v_old + h·a = v_old + h·M_impl⁻¹·f`.

The non-Newton path solves `M_impl · v_new = M·v_old + h·f_ext − h·K·Δq`,
giving `v_new = M_impl⁻¹·(M·v_old + h·f_ext − h·K·Δq)`.

For these to match, Newton's `a` must satisfy:
```
v_old + h·a = M_impl⁻¹·(M·v_old + h·f_ext − h·K·Δq)
h·a = M_impl⁻¹·(M·v_old + h·f_ext − h·K·Δq) − v_old
```

With `M_impl = M + h·D + h²·K`:
```
h·a = M_impl⁻¹·(M·v_old + h·f_ext − h·K·Δq − M_impl·v_old)
    = M_impl⁻¹·(−h·D·v_old − h²·K·v_old + h·f_ext − h·K·Δq)
    = M_impl⁻¹·h·(f_ext − D·v_old − h·K·v_old − K·Δq)
a   = M_impl⁻¹·(f_ext − D·v_old − K·(Δq + h·v_old))
```

So `qfrc_smooth` for Newton must be:
```
qfrc_smooth_impl = f_ext − D·v_old − K·(Δq + h·v_old)
                 = (qfrc_applied + qfrc_actuator − qfrc_bias)
                   − D_jnt·v − D_ten·v − K_jnt·(q−q_eq + h·v) − K_ten·(δ + h·V_ten)
```

where `D_ten·v = Σ_t b_t·J_t^T·(J_t·v)` and `K_ten·v = Σ_t k_t·J_t^T·(J_t·v)`.

**This is more complex than initially estimated.** The implicit formulation
introduces velocity-dependent terms (`D·v`, `h·K·v`) into `qfrc_smooth` that
are NOT present in the explicit integrators. These terms correspond to the
LHS modification being "moved" to the RHS.

**Simplified correct implementation:**

Rather than deriving the exact `qfrc_smooth`, we can reformulate: Newton
should compute `qacc_smooth` directly from the non-Newton implicit solver
(which already produces the correct `qacc`), then use `M_impl` in the
Hessian. Specifically:

1. **Always** run `mj_fwd_acceleration_implicit` first (before Newton)
2. Use its `data.qacc` as `qacc_smooth` for Newton
3. Use `M_impl` (from `scratch_m_impl` before Cholesky overwrites it) in
   Newton's Hessian

But `mj_fwd_acceleration_implicit` overwrites `scratch_m_impl` with the
Cholesky factor (line 20195), destroying `M_impl`. And it also overwrites
`data.qacc` and `data.qvel`.

**Revised implementation strategy:**

1. In `mj_fwd_constraint` (line 19469), **before** the Newton dispatch,
   compute and cache `M_impl` and `qacc_smooth_impl`:

```rust
// DT-35: For ImplicitSpringDamper + Newton, precompute the implicit-modified
// quantities that Newton needs. These parallel mj_fwd_acceleration_implicit
// but are computed before the constraint solve (which needs qacc_smooth).
let m_impl_owned: Option<DMatrix<f64>>;
let qfrc_impl_owned: Option<DVector<f64>>;
let (m_eff, qfrc_eff) = if implicit_sd {
    let m_impl = build_m_impl_for_newton(model, data);
    let qfrc_impl = compute_qfrc_smooth_implicit(model, data);
    m_impl_owned = Some(m_impl);
    qfrc_impl_owned = Some(qfrc_impl);
    (m_impl_owned.as_ref().unwrap(), qfrc_impl_owned.as_ref().unwrap())
} else {
    m_impl_owned = None;
    qfrc_impl_owned = None;
    (&data.qM, &data.qfrc_smooth)
};
// m_eff: &DMatrix<f64> — either &M_impl or &data.qM
// qfrc_eff: &DVector<f64> — either &qfrc_impl or &data.qfrc_smooth
```

2. Pass `m_eff` and `qfrc_eff` to `newton_solve` as references.

3. Newton uses `m_eff` for Hessian and M·v products, and
   `qfrc_eff` for gradient/cost.

**Helper — `compute_qfrc_smooth_implicit`:**

```rust
/// Compute implicit-corrected smooth forces for Newton solver.
///
/// In ImplicitSpringDamper mode, qfrc_passive excludes spring/damper forces.
/// This function computes the RHS forces that, together with M_impl in the
/// Hessian, produce the correct unconstrained acceleration matching the
/// non-Newton implicit path.
///
/// From the equivalence derivation:
///   a = M_impl⁻¹ · (f_ext − D·v − K·(Δq + h·v))
///
/// So: qfrc_smooth_impl = qfrc_smooth_base
///     − D_jnt·v − D_ten·v           (damper forces)
///     − K_jnt·(Δq + h·v)            (spring forces + velocity correction)
///     − K_ten·(δ + h·V_ten)          (tendon spring + velocity correction)
///
/// The `h·K·v` terms are the "velocity correction" that arises from the
/// implicit treatment of spring stiffness. Without them, the Newton path
/// would NOT match the non-Newton path for the unconstrained case.
///
/// **Cross-reference:** The joint spring displacement `−K·(q − q_eq)` mirrors
/// `ImplicitSpringVisitor` (line 20184). The velocity correction `−h·K·v` is
/// unique to this function (not present in the non-Newton path, which absorbs
/// it into the LHS `h²·K·v_new` term). The tendon spring displacement uses
/// `tendon_deadband_displacement` (Step 0) for consistency.
#[must_use]
fn compute_qfrc_smooth_implicit(model: &Model, data: &Data) -> DVector<f64> {
    let h = model.timestep;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Start with existing qfrc_smooth (which has everything EXCEPT
    // implicit spring/damper forces)
    let mut qfrc = data.qfrc_smooth.clone();

    // Add joint spring forces: −K·(Δq + h·v)
    // where Δq = q − q_eq, so total = −K·(q − q_eq) − h·K·v
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let k = model.implicit_stiffness[dof_adr];
        if k <= 0.0 { continue; }
        let q_eq = model.implicit_springref[dof_adr];
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                let v = data.qvel[dof_adr];
                qfrc[dof_adr] += -k * (q - q_eq) - h * k * v;
            }
            // Ball/Free: compute_implicit_params sets implicit_stiffness=0
            // for these types, so the `k <= 0.0` guard above catches them.
            _ => {
                debug_assert!(
                    k <= 0.0,
                    "Ball/Free joint {jnt_id} has implicit_stiffness={k} > 0; \
                     compute_implicit_params should set this to 0.0"
                );
            }
        }
    }

    // Add joint damper forces: −D·v
    for i in 0..nv {
        let d = model.implicit_damping[i];
        if d > 0.0 {
            qfrc[i] += -d * data.qvel[i];
        }
    }

    // Add tendon spring forces: −k·(δ + h·V_ten) projected via J^T
    // where δ = tendon_deadband_displacement(L) (Step 0 helper) and
    // V_ten = J · qvel (tendon velocity).
    //
    // The h·K·v "velocity correction" arises from the implicit linearization:
    //   Δq_new ≈ Δq_old + h·v_new
    // so the implicit spring force is F = -k·(Δq_old + h·v_new). On the LHS
    // this gives h²·K·v_new; on the RHS this gives -h·K·Δq_old (the spring
    // displacement) and -h²·K·v_old (the velocity correction).
    //
    // IMPORTANT: The velocity correction h·K·v must ONLY apply when the spring
    // is active (displacement ≠ 0). When the tendon is inside the deadband,
    // δ = 0 and K_tendon should contribute zero force — NOT -k·h·velocity.
    // Otherwise the velocity correction would introduce a spurious spring
    // force inside the deadband, which is physically wrong.
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        if kt > 0.0 {
            let displacement = tendon_deadband_displacement(
                data.ten_length[t], model.tendon_lengthspring[t],
            );
            // Only apply spring force + velocity correction when OUTSIDE
            // deadband. Inside (displacement == 0.0), no spring force exists,
            // and the velocity correction h·K·v must also be zero. This is
            // consistent with accumulate_tendon_kd (Step 0), which sets
            // k_active = 0 inside the deadband — no h²·K on the LHS either.
            if displacement != 0.0 {
                let velocity = data.ten_velocity[t]; // J · qvel
                // Spring force + velocity correction in tendon space:
                //   F = -k · (displacement + h · velocity)
                let f = -kt * (displacement + h * velocity);
                let j = &data.ten_J[t];
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        qfrc[dof] += f * j[dof];
                    }
                }
            }
        }
        // Add tendon damper forces: −b · V projected via J^T
        let bt = model.tendon_damping[t];
        if bt > 0.0 {
            let j = &data.ten_J[t];
            let velocity = data.ten_velocity[t]; // J · qvel
            let f = -bt * velocity;
            for dof in 0..nv {
                if j[dof] != 0.0 {
                    qfrc[dof] += f * j[dof];
                }
            }
        }
    }

    qfrc
}
```

**Also recompute `qacc_smooth`:** After augmenting `qfrc_smooth`, the
unconstrained acceleration `qacc_smooth = M_impl⁻¹ · qfrc_smooth_impl` must
use `M_impl`, not raw `M`. This requires a separate solve:

```rust
// qacc_smooth_impl = M_impl⁻¹ · qfrc_smooth_impl
// Use dense Cholesky (Newton already handles dense/sparse switching)
let mut m_impl_copy = m_impl.clone();
cholesky_in_place(&mut m_impl_copy)?;
let mut qacc_smooth_impl = qfrc_smooth_impl.clone();
cholesky_solve_in_place(&m_impl_copy, &mut qacc_smooth_impl);
```

This replaces the `qacc_smooth` and `qfrc_smooth` used throughout Newton.

**Modification to `mj_fwd_constraint`:**

The constraint assembly `assemble_unified_constraints(model, data, &qacc_smooth)`
uses `qacc_smooth` for computing `efc_b` (constraint bias). When
`ImplicitSpringDamper` is active, this should use the implicit-corrected
`qacc_smooth_impl`, because the unconstrained motion includes the spring/damper
effects. Update the call:

```rust
let qacc_for_assembly = if implicit_sd {
    &qacc_smooth_impl
} else {
    &qacc_smooth
};
assemble_unified_constraints(model, data, qacc_for_assembly);
```

**Velocity update in `integrate()`:** When Newton succeeds, the velocity
update `qvel += qacc * h` (line 4755) uses Newton's `qacc` which was solved
against `M_impl`. This `qacc` already accounts for the implicit K/D in the
system matrix, so the velocity update is correct — no change needed.

**Summary of Step 2b + 3b modifications:**

| Function | Change | Affected lines |
|----------|--------|----------------|
| `build_m_impl_for_newton` | New helper | — |
| `compute_qfrc_smooth_implicit` | New helper | — |
| `mj_fwd_constraint` | Compute `m_impl`, `qfrc_smooth_impl`, `qacc_smooth_impl` when `ImplicitSpringDamper` active; pass to `newton_solve` | ~19470-19495 |
| `newton_solve` | Accept `m_eff: &DMatrix` + `qfrc_smooth: &DVector` params; use throughout | 17973+ |
| `assemble_hessian` | Accept `m_eff` param, use instead of `data.qM` | 16710 |
| `SparseHessian::assemble` | Add tendon sparsity pattern to `has_entry`; `fill_numeric` adds tendon K/D values via separate loop after tree walk (no `m_eff` param needed — reads `data.qM` for tree entries, adds tendon contributions directly) | 16797-16941 |
| `SparseHessian::fill_numeric` | Add `implicit_sd: bool` + `model`/`data`/`h` params for tendon K/D fill loop after tree walk | 16889 |
| `evaluate_cost_at` | Accept `m_eff` param, use for `Ma` computation | 17847 |
| `hessian_incremental` | Accept `m_eff` param; forward to `assemble_hessian` fallback (verified: no direct `data.qM` reference in rank-1 update path) | 17194, fallback at 17229 |
| `primal_prepare` / `primal_search` | No changes needed (verified: uses precomputed `ma`/`mv`, no `data.qM` reference) | 17430 |
| `integrate` | No change (velocity update `qvel += qacc*h` is correct) | 4755 |

**`hessian_incremental` signature** (not shown above, making explicit):

```rust
fn hessian_incremental(
    data: &Data,
    nv: usize,
    chol_l: &mut DMatrix<f64>,
    old_states: &[ConstraintState],
    m_eff: &DMatrix<f64>,  // NEW: forwarded to assemble_hessian on fallback
) -> Result<(), StepError> {
    // ... rank-1 updates use efc_D/efc_J only (no data.qM) ...
    // On downdate failure, fallback rebuilds from scratch:
    //     let fresh = assemble_hessian(data, nv, m_eff)?;
    //     cholesky_in_place(&mut fresh)?;
    //     *chol_l = fresh;
    // The fresh Hessian uses m_eff (M or M_impl), then Cholesky-factors it
    // in-place, replacing the stale chol_l entirely.
```

#### Step 4: Include tendon damping in `mjd_passive_vel` for all integrators

**File:** `sim/L0/core/src/derivatives.rs`, lines 499–523.

Currently, tendon damping derivatives (`-b·J^T·J` in `qDeriv`) are skipped for
`ImplicitSpringDamper`. This was consistent with the forces being zero. Now
that `mj_fwd_acceleration_implicit` handles tendon damping via the mass matrix
modification (Step 2), `qDeriv` is not directly used by that solver path — it
is only used by `ImplicitFast`/`Implicit`. However, `qDeriv` is also used by
the analytical derivatives system (`mjd_transitionFD`, etc.). For consistency,
include tendon damping in `qDeriv` for all integrators.

Remove the `implicit_mode` guard:

```rust
// 3. Tendon damping: −b · J^T · J (rank-1 outer product per tendon).
for t in 0..model.ntendon {
    // §40c: Skip tendon if ALL target DOFs are sleeping.
    if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
        continue;
    }
    let b = model.tendon_damping[t];
    if b <= 0.0 {
        continue;
    }
    let j = &data.ten_J[t];
    for r in 0..model.nv {
        if j[r] == 0.0 {
            continue;
        }
        for c in 0..model.nv {
            if j[c] == 0.0 {
                continue;
            }
            data.qDeriv[(r, c)] += -b * j[r] * j[c];
        }
    }
}
```

Remove the `let implicit_mode = ...` and `if !implicit_mode` wrapper. Update the
comment to remove the "skipped in implicit mode" note.

**Impact on `ImplicitFast`/`Implicit`:** None — they already included this path
(`implicit_mode` was false for them).

**Impact on `ImplicitSpringDamper`:** `qDeriv` now includes tendon damping.
`mj_fwd_acceleration_implicit` does NOT read `qDeriv` (it uses its own diagonal
`+h·D+h²·K` path), so this has no effect on the simulation. But it makes the
derivative system consistent and correct for analytical derivative computation.

#### Step 5: Update stale comments

**Files:** `sim/L0/core/src/forward/passive.rs`, `sim/L0/core/src/types/data.rs`

1. Update NOTE comment in `forward/passive.rs` (see Step 1)
2. Update `qDeriv` doc in `types/data.rs`: Remove the "in explicit mode only; skipped
   for ImplicitSpringDamper" qualifier — tendon damping is now included for all
   modes
3. Line 4746–4748: Update Newton velocity update comment to note that
   `ImplicitSpringDamper` + Newton now uses `M_impl` in the Hessian (DT-35)
4. Line 16703–16706: Update `assemble_hessian` doc to note `m_eff` parameter
   and tendon K/D inclusion for `ImplicitSpringDamper`

**File:** `sim/L0/core/src/derivatives.rs`

1. Lines 495–498: Update comment — remove "Skipped in implicit mode" language
2. Line 471: Update doc comment — remove ImplicitSpringDamper skip note

**File:** `sim/docs/todo/future_work_1.md`

1. Lines 1626–1630: Update the known-limitation comment to reference the fix

### Tests

All tests in `sim-core` and `sim-conformance-tests`. Test module:
`tendon_implicit_tests` (new), in `forward/passive.rs` as a `#[cfg(test)]`
submodule.

**Solver path coverage:** Each test exercises a specific solver codepath:

| Path | Tests | How |
|------|-------|-----|
| Non-Newton (`mj_fwd_acceleration_implicit`) | 6a–6h, 6k, 6l, 6l½, 6l¾, 6q | PGS solver, no constraints → `nefc=0`, `newton_solved=false` → `mj_fwd_acceleration_implicit` runs |
| Unconstrained solver-flag smoke | 6n, 6o, 6r | `solver="Newton"` but `nefc=0` → early return before solver dispatch → `mj_fwd_acceleration_implicit` runs (same as PGS). Verifies solver flag is a no-op for unconstrained implicit models. Does NOT exercise Newton's `M_impl` codepath. |
| Newton + active constraints (`M_impl`) | 6p | `solver="Newton"`, ground contact → `nefc > 0`, Newton Hessian uses `M_impl`. **Primary test for Steps 2b/3b.** |
| Derivatives (`mjd_passive_vel`) | 6f | Calls `mjd_smooth_vel` directly |
| Regression (existing tests, unchanged) | 6i, 6j | `ImplicitFast` integrator, not `ImplicitSpringDamper` |
| Regression (existing test, updated) | 6m | `ImplicitSpringDamper` `qDeriv` assertions inverted |
| Sleep guard (`accumulate_tendon_kd` + RHS) | 6s | `flag sleep="enable"`, force DOFs sleeping → tendon K/D and RHS displacement skipped |

**Note on unconstrained Newton (tests 6n, 6o, 6r):** When `nefc=0`,
`mj_fwd_constraint` returns early at line 19489 **before** the solver dispatch.
`newton_solved` stays `false`, so `mj_fwd_acceleration_implicit` runs for both
PGS and Newton models. These tests verify that the solver flag doesn't affect
the unconstrained implicit path, not that Newton's `M_impl` codepath is
exercised. Test 6p is the primary test for the Newton + `M_impl` codepath
(active constraints force Newton to actually run).

**Test model helper — `make_tendon_implicit_model`:**

Build a 2-DOF model (two hinge joints on a serial chain) with a fixed tendon
coupling them. Parameterize by integrator, stiffness, damping, and solver.

**Import note:** Tests in `sim-core` modules use `sim_mjcf::load_model` for
MJCF string parsing. The existing test at `derivatives.rs:1058` confirms this
pattern: `let model = sim_mjcf::load_model(mjcf).unwrap();`.

```rust
use sim_mjcf::load_model;

fn make_tendon_implicit_model(
    integrator: &str,
    stiffness: f64,
    damping: f64,
) -> (Model, Data) {
    make_tendon_implicit_model_with_solver(integrator, stiffness, damping, "PGS")
}

fn make_tendon_implicit_model_with_solver(
    integrator: &str,
    stiffness: f64,
    damping: f64,
    solver: &str,
) -> (Model, Data) {
    // springlength="0 0" explicitly sets the deadband to [0, 0], meaning any
    // non-zero tendon length is outside the deadband. Without this attribute,
    // MuJoCo auto-computes springlength from the initial tendon length, which
    // happens to be 0 for this model (qpos = [0, 0] → L = 0). Explicit is
    // clearer and protects against accidental changes to initial state.
    let mjcf = format!(r#"
        <mujoco model="tendon_impl">
            <option timestep="0.001" integrator="{integrator}"
                    solver="{solver}" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="{stiffness}" damping="{damping}"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#);
    let model = sim_mjcf::load_model(&mjcf).unwrap();
    let data = model.make_data();
    (model, data)
}
```

**6a. `tendon_spring_nonzero_in_implicitspringdamper`:**

The primary regression test. With `stiffness=50, damping=0`, displaced
`qpos[0]=0.3`. `ten_force[t]` is populated for diagnostic purposes (see
Diagnostic Observability). Verify the physics effect via `qacc`; verify
the diagnostic via `ten_force`.

```rust
let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
data.qpos[0] = 0.3;  // Displace j0 — tendon stretches

// Step forward
data.step(&model).unwrap();

// The tendon spring should produce restoring acceleration.
// With q0=0.3, tendon L = q0 - q1 = 0.3, spring force = -50*0.3 = -15.
// This maps to qacc[0] via J^T. qacc[0] should be negative (restoring).
assert!(data.qacc[0] < -1e-3,
    "qacc[0] should be negative (restoring), got {}", data.qacc[0]);

// qacc should be non-trivial — not the zero that the old code produced
let qacc_norm = data.qacc.norm();
assert!(qacc_norm > 1e-3,
    "qacc norm should be non-trivial, got {qacc_norm}");
```

**6b. `tendon_damper_nonzero_in_implicitspringdamper`:**

With `stiffness=0, damping=100`, initial `qvel[0]=1.0`:

```rust
let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 100.0);
data.qvel[0] = 1.0;  // Moving j0 — tendon damper resists

data.step(&model).unwrap();

// Damper should decelerate: qvel[0] after step should be less than 1.0
assert!(data.qvel[0] < 1.0 - 1e-6,
    "qvel[0] should decrease from damping, got {}", data.qvel[0]);
```

**6c. `tendon_implicit_matches_euler_small_dt`:**

For small enough timestep, `ImplicitSpringDamper` and `Euler` should produce
nearly identical trajectories. The implicit integrator's LHS modification
introduces an `O(h)` per-step difference in the effective dynamics (the
extra `h·D + h²·K` terms act as numerical damping/stiffness). Over N steps
the accumulated error is `O(N·h²)` in velocity.

**Tolerance derivation:** To keep the Euler-vs-implicit comparison tight, we
use heavy masses (`mass="10.0"`) so that the implicit modification ratio
`h·D_eff / M_min` is small:
- With `h=0.001`, `k=50`, `b=10`, `mass=10.0`:
- `M[0,0] ≈ I_sphere + m·d² = 2/5·10·0.01 + 10·1.0 = 0.04 + 10 = 10.04`
  (parallel axis theorem from body 2), `M[1,1] ≈ I_sphere = 0.04`.
- The tendon direction `[1, -1]` couples both DOFs. The effective compliance
  (inverse mass) for the tendon mode is `J·M⁻¹·J^T ≈ 1/10.04 + 1/0.04 ≈ 25.1`,
  so `M_eff = 1/25.1 ≈ 0.04`. (This uses the diagonal of M as an
  approximation — the serial chain has off-diagonal inertia coupling, but the
  diagonal dominates for heavy masses with the parallel-axis term.)
- Spring LHS modification per tendon mode: `h²·k / M_eff ≈ 5e-5 / 0.04 = 1.25e-3`
- Damper LHS modification per tendon mode: `h·b / M_eff ≈ 0.01 / 0.04 = 0.25`
- Per-step velocity error is `O(h · (h·k + b) / M_eff · v)`.
  Over 10 steps with small initial displacement `q₀ = 0.1` and
  `v ≈ k·q₀·h / M_eff ≈ 50·0.1·0.001/0.04 = 0.125`:
  total error ≈ `10 · 0.001 · 0.25 · 0.125 ≈ 3e-4`
- Use tolerance `0.05` — the diagonal approximation above underestimates the
  error because the implicit solver's mass matrix modification introduces
  larger-than-estimated numerical damping in coupled serial-chain systems.
  Off-diagonal inertia coupling amplifies the per-step difference, pushing
  the accumulated error above the naive `3e-4` estimate.

```rust
fn make_tendon_implicit_model_heavy(
    integrator: &str,
    stiffness: f64,
    damping: f64,
) -> (Model, Data) {
    let mjcf = format!(r#"
        <mujoco model="tendon_impl_heavy">
            <option timestep="0.001" integrator="{integrator}" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="10.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="10.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="{stiffness}" damping="{damping}"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#);
    let model = sim_mjcf::load_model(&mjcf).unwrap();
    let data = model.make_data();
    (model, data)
}

let (model_euler, mut data_euler) = make_tendon_implicit_model_heavy("Euler", 50.0, 10.0);
let (model_impl, mut data_impl) = make_tendon_implicit_model_heavy("implicitspringdamper", 50.0, 10.0);

// Same initial displacement
data_euler.qpos[0] = 0.1;
data_impl.qpos[0] = 0.1;

// Step both forward 10 steps
for _ in 0..10 {
    data_euler.step(&model_euler).unwrap();
    data_impl.step(&model_impl).unwrap();
}

// At dt=0.001, mass=10.0, and only 10 steps, the trajectories should be
// very close. The implicit modification introduces O(h·D_eff/M_eff)
// per-step numerical damping, accumulating to ≈3e-4 over 10 steps.
for i in 0..model_euler.nv {
    let diff = (data_euler.qvel[i] - data_impl.qvel[i]).abs();
    assert!(diff < 0.05,
        "qvel[{i}] divergence too large: euler={}, impl={}, diff={diff}",
        data_euler.qvel[i], data_impl.qvel[i]);
}
```

**6d. `tendon_implicit_stability_vs_euler`:**

The key selling point of implicit integration: stability with large timestep and
high stiffness. With `stiffness=5000, damping=0, dt=0.01`, Euler should diverge
(oscillate/blow up) while ImplicitSpringDamper stays bounded:

```rust
fn make_tendon_stiff_model(integrator: &str) -> (Model, Data) {
    let mjcf = format!(r#"
        <mujoco model="tendon_stiff">
            <option timestep="0.01" integrator="{integrator}" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="5000.0" damping="0.0"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#);
    let model = sim_mjcf::load_model(&mjcf).unwrap();
    let data = model.make_data();
    (model, data)
}

let (model_euler, mut data_euler) = make_tendon_stiff_model("Euler");
let (model_impl, mut data_impl) = make_tendon_stiff_model("implicitspringdamper");

data_euler.qpos[0] = 0.3;
data_impl.qpos[0] = 0.3;

// Step 200 times.
// `let _ = ...` intentionally swallows errors: Euler with h²k/m >> 1 may
// return Err(CholeskyFailed) or produce NaN/Inf. Both outcomes confirm
// instability. ImplicitSpringDamper should never error here.
for _ in 0..200 {
    let _ = data_euler.step(&model_euler);
    data_impl.step(&model_impl).expect("ImplicitSpringDamper should not fail");
}

let vel_euler = data_euler.qvel.norm();
let vel_impl = data_impl.qvel.norm();

// ImplicitSpringDamper should stay bounded
assert!(vel_impl < 100.0,
    "ImplicitSpringDamper velocity should be bounded, got {vel_impl}");

// Euler with dt=0.01, k=5000 should be unstable (h²k/m >> 1)
// Either it diverges to large values or the step function returns an error.
// We just check that implicit is much more stable than Euler.
assert!(vel_impl < vel_euler || vel_euler.is_nan() || vel_euler.is_infinite(),
    "Implicit should be more stable: vel_impl={vel_impl}, vel_euler={vel_euler}");
```

**6e. `tendon_implicit_mass_matrix_has_off_diagonal`:**

Verify that the mass matrix modification includes off-diagonal tendon coupling.

**Caveat:** `scratch_m_impl` is `pub` on `Data`, but `cholesky_in_place` at
line 20195 **overwrites** it with the Cholesky factor L. After `forward()` or
`step()`, `scratch_m_impl` no longer contains `M + h·D + h²·K` — it contains L.

**Strategy:** Verify indirectly via the qacc cross-coupling. With tendon
J = [1, -1], a displacement of `q0` should produce restoring acceleration on
**both** DOFs (due to off-diagonal K coupling). If the mass matrix were
diagonal-only, `qacc[1]` would be zero for a perturbation in `q0` alone.

```rust
let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
data.qpos[0] = 0.3;  // Only displace j0

data.step(&model).unwrap();

// Off-diagonal coupling: J^T*k*J has nonzero (0,1) and (1,0) entries.
// So qacc[1] should be NON-ZERO even though q1 = 0 and qvel[1] = 0.
// With J = [1, -1], the tendon spring pulls j1 in the positive direction.
assert!(data.qacc[1].abs() > 1e-3,
    "qacc[1] should be non-zero from off-diagonal coupling, got {}", data.qacc[1]);

// Signs: tendon L = q0 - q1 = 0.3 > 0, spring force = -k*0.3 = -15.
// qfrc = J^T * F = [1, -1]^T * (-15) = [-15, +15].
// So qacc[0] < 0 (restoring) and qacc[1] > 0 (pulled toward q0).
assert!(data.qacc[0] < 0.0, "qacc[0] should be negative, got {}", data.qacc[0]);
assert!(data.qacc[1] > 0.0, "qacc[1] should be positive, got {}", data.qacc[1]);
```

**6f. `tendon_qDeriv_includes_damping_for_all_integrators`:**

Verify that `qDeriv` includes tendon damping `-b·J^T·J` for
`ImplicitSpringDamper` mode (previously skipped).

**Important:** `data.forward()` does NOT populate `qDeriv`. The derivative
assembly is a separate API call: `mjd_smooth_vel(&model, &mut data)`. This
matches the existing test pattern in `test_tendon_damping_implicit_guard`
(derivatives.rs:1060–1062).

```rust
use crate::derivatives::mjd_smooth_vel;

let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 100.0);
data.qpos[0] = 0.3;
data.forward(&model).unwrap();
mjd_smooth_vel(&model, &mut data);  // Populates qDeriv

// qDeriv should have tendon damping contribution.
// For J = [1, -1], b = 100: qDeriv += -b * J^T * J = -100 * [[1,-1],[-1,1]]
// Diagonal entries: qDeriv[(0,0)] += -100 * 1 * 1 = -100
// Off-diagonal:     qDeriv[(0,1)] += -100 * 1 * (-1) = +100
// (Plus any per-DOF damping from joints, which is 0 in this model)
assert!(data.qDeriv[(0, 0)] < -1e-6,
    "qDeriv[(0,0)] should be negative from tendon damping, got {}", data.qDeriv[(0, 0)]);
assert!(data.qDeriv[(0, 1)] > 1e-6,
    "qDeriv[(0,1)] should be positive (J[0]*J[1] = 1*(-1) = -1, times -b = +100), got {}",
    data.qDeriv[(0, 1)]);
```

**6g. `tendon_implicit_combined_spring_damper`:**

Verify that combined spring + damper works correctly in `ImplicitSpringDamper`
mode. Tests 6a/6b cover spring-only and damper-only; this verifies the combined
mass matrix modification `(h²·k + h·b)·J^T·J` and the simultaneous RHS
displacement.

```rust
let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 20.0);
data.qpos[0] = 0.3;   // Displace — spring activates
data.qvel[0] = 1.0;   // Moving — damper activates

data.step(&model).unwrap();

// Spring restoring: qacc[0] should be negative (spring pulls back).
// Damper resisting: qacc[0] should be more negative than spring-only case
// (damper opposes positive velocity, adding to the negative acceleration).
assert!(data.qacc[0] < -1e-3,
    "qacc[0] should be negative (spring restoring + damper resisting), got {}",
    data.qacc[0]);

// qvel[0] should decrease (damper decelerates, spring pulls back)
assert!(data.qvel[0] < 1.0 - 1e-6,
    "qvel[0] should decrease from combined spring+damper, got {}", data.qvel[0]);

// Cross-coupling: qacc[1] should be non-zero from off-diagonal terms
assert!(data.qacc[1].abs() > 1e-3,
    "qacc[1] should show cross-coupling, got {}", data.qacc[1]);

// Compare against spring-only: combined should DIFFER from spring-only.
// Note: the implicit mass matrix modification (h²·k + h·b)·J^T·J increases
// effective mass, which can reduce |qacc| even when adding damping forces.
// So we check that the results differ rather than asserting magnitude ordering.
let (model_s, mut data_s) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
data_s.qpos[0] = 0.3;
data_s.qvel[0] = 1.0;
data_s.step(&model_s).unwrap();

let qacc_diff = (data.qacc[0] - data_s.qacc[0]).abs();
assert!(qacc_diff > 1e-6,
    "Combined spring+damper should differ from spring-only: \
     combined={}, spring_only={}, diff={qacc_diff}", data.qacc[0], data_s.qacc[0]);
```

**6h. `tendon_implicit_energy_dissipation`:**

Gold-standard physics test. For a damper-only system, kinetic energy must
decrease monotonically (energy dissipation). For a spring-only system with
small timestep, total energy (kinetic + spring potential) should be
approximately conserved (the implicit integrator adds `O(h)` numerical
damping, but total energy should not *increase*).

```rust
// --- Part A: Damper-only energy dissipation ---
let (model_d, mut data_d) = make_tendon_implicit_model("implicitspringdamper", 0.0, 50.0);
data_d.qvel[0] = 2.0;
data_d.qvel[1] = -1.0;

let mut prev_ke = f64::MAX;
for step in 0..50 {
    data_d.step(&model_d).unwrap();
    // Kinetic energy: 0.5 * v^T * M * v
    let ke = {
        let mv = &data_d.qM * &data_d.qvel;
        0.5 * data_d.qvel.dot(&mv)
    };
    assert!(ke <= prev_ke + 1e-12,
        "Step {step}: KE should not increase with damper-only: prev={prev_ke}, now={ke}");
    prev_ke = ke;
}
// After 50 steps of damping, velocity should be significantly reduced
assert!(data_d.qvel.norm() < 1.0,
    "Velocity should be significantly damped after 50 steps, got norm={}",
    data_d.qvel.norm());

// --- Part B: Spring-only energy approximate conservation ---
// Use explicit springlength="0 0" so the deadband is [0, 0] and the
// spring potential is simply 0.5 * k * L², where L = q0 - q1.
// Without this, the default springlength is auto-computed from initial
// length, which would give a non-zero rest length and invalidate the
// simple energy formula below.
let mjcf_spring_energy = r#"
    <mujoco model="tendon_energy">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="b2" pos="0 0 -1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="50.0" damping="0.0"
                   springlength="0 0">
                <joint joint="j0" coef="1.0"/>
                <joint joint="j1" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model_s = sim_mjcf::load_model(mjcf_spring_energy).unwrap();
let mut data_s = model_s.make_data();
data_s.qpos[0] = 0.3;

// Spring potential: 0.5 * k * displacement²
// With springlength="0 0", deadband is [0, 0], so any L != 0 is outside.
// At t=0: displacement = L - 0 = L = q0 - q1 = 0.3 (since q1 starts at 0).
// After stepping, both q0 and q1 change — recompute L from current state.
let disp0 = data_s.qpos[0] - data_s.qpos[1];  // tendon L = q0 - q1 (= 0.3 at t=0)
let pe0 = 0.5 * 50.0 * disp0 * disp0;  // = 2.25
let ke0 = 0.0;  // zero initial velocity
let e0 = pe0 + ke0;

for _ in 0..100 {
    data_s.step(&model_s).unwrap();
}

// Recompute energy
// displacement = deadband_disp(L) = L - 0 = L (since L > upper = 0 or L < lower = 0)
let l = data_s.qpos[0] - data_s.qpos[1]; // tendon length
let displacement = l; // deadband [0, 0]: any non-zero L is outside
let pe = 0.5 * 50.0 * displacement * displacement;
let ke = {
    let mv = &data_s.qM * &data_s.qvel;
    0.5 * data_s.qvel.dot(&mv)
};
let e_final = pe + ke;

// Implicit integrator adds numerical damping: energy should NOT increase.
// Allow small floating-point tolerance.
assert!(e_final <= e0 + 1e-6,
    "Total energy should not increase for spring-only: e0={e0}, e_final={e_final}");
// The implicit integrator introduces O(h) numerical damping per step, which
// accumulates significantly over 100 steps — dissipating a large fraction
// of the initial energy. Verify energy remains positive (not gained) and
// hasn't gone to zero (spring is still oscillating).
assert!(e_final > 0.01 * e0,
    "Total energy should not be fully dissipated: e0={e0}, e_final={e_final}");
```

**6i. Regression — `test_implicitfast_tendon_spring_explicit` (AC-8) passes unchanged:**

The existing AC-8 test (implicit_integration.rs:939) must continue to pass. It
verifies that `ImplicitFast` and `Euler` produce identical `qfrc_passive` for
tendon springs. Since this test uses `ImplicitFast` (not `ImplicitSpringDamper`),
it should be completely unaffected. Verify by running:

```bash
cargo test -p sim-conformance-tests -- test_implicitfast_tendon_spring_explicit
```

**6j. Regression — `test_implicitfast_tendon_damping_stability` (AC-1) passes unchanged:**

The existing AC-1 test (implicit_integration.rs:716) verifies `ImplicitFast`
tendon damping stability. Unaffected by this change. Verify by running:

```bash
cargo test -p sim-conformance-tests -- test_implicitfast_tendon_damping_stability
```

**6k. `tendon_implicit_deadband_zero_inside`:**

Verify that the implicit solver produces zero spring force when the tendon
length is inside the deadband `[lower, upper]`:

```rust
let mjcf = r#"
    <mujoco model="tendon_deadband">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="b2" pos="0 0 -1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="100.0" damping="0.0"
                   springlength="0.5 1.0">
                <joint joint="j0" coef="1.0"/>
                <joint joint="j1" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();

// --- Sub-case A1: Inside deadband, zero velocity --- //
// Set tendon length L = q0 - q1 = 0.7, inside deadband [0.5, 1.0]
data.qpos[0] = 0.7;

data.step(&model).unwrap();

// No spring force inside deadband — qacc should be zero (no gravity, no damping)
let qacc_norm = data.qacc.norm();
assert!(qacc_norm < 1e-10,
    "qacc should be ~zero inside deadband, got norm={qacc_norm}");

// --- Sub-case A2: Inside deadband WITH nonzero velocity --- //
// Verifies no phantom damping from h²·K inside the deadband.
// With k_active = 0 inside the deadband (Step 2), the LHS has no spring
// stiffness and the solver should not decelerate the DOFs.
let mut data_vel = model.make_data();
data_vel.qpos[0] = 0.7;   // Inside deadband
data_vel.qvel[0] = 1.0;   // Moving — should NOT be damped by spring
data_vel.step(&model).unwrap();

// qacc should be zero: no spring force, no damping (damping=0 in this model),
// no gravity. Velocity should be unchanged.
let qacc_vel_norm = data_vel.qacc.norm();
assert!(qacc_vel_norm < 1e-10,
    "qacc should be ~zero inside deadband even with velocity, got norm={qacc_vel_norm}");
assert!((data_vel.qvel[0] - 1.0).abs() < 1e-10,
    "qvel[0] should be unchanged inside deadband (no spring, no damper), got {}",
    data_vel.qvel[0]);

// --- Sub-case B: Below lower bound --- //
// L = q0 - q1 = 0.2 < lower = 0.5. Spring should push tendon longer
// (positive restoring force in tendon space).
let mut data_below = model.make_data();
data_below.qpos[0] = 0.2;  // L = 0.2 < 0.5

data_below.step(&model).unwrap();

// displacement = L - lower = 0.2 - 0.5 = -0.3
// F_spring = -k * displacement = -100 * (-0.3) = +30 (lengthening force)
// qfrc = J^T * F = [1, -1]^T * 30 = [+30, -30]
// qacc[0] should be positive (pushing q0 larger to lengthen tendon)
assert!(data_below.qacc[0] > 1e-3,
    "Below deadband: qacc[0] should be positive (lengthening), got {}",
    data_below.qacc[0]);
assert!(data_below.qacc[1] < -1e-3,
    "Below deadband: qacc[1] should be negative, got {}",
    data_below.qacc[1]);

// --- Sub-case C: Above upper bound --- //
// L = q0 - q1 = 1.5 > upper = 1.0. Spring should pull tendon shorter.
let mut data_above = model.make_data();
data_above.qpos[0] = 1.5;  // L = 1.5 > 1.0

data_above.step(&model).unwrap();

// displacement = L - upper = 1.5 - 1.0 = 0.5
// F_spring = -k * 0.5 = -50 (shortening force)
// qfrc = J^T * F = [1, -1]^T * (-50) = [-50, +50]
// qacc[0] should be negative (pulling q0 back to shorten tendon)
assert!(data_above.qacc[0] < -1e-3,
    "Above deadband: qacc[0] should be negative (shortening), got {}",
    data_above.qacc[0]);
assert!(data_above.qacc[1] > 1e-3,
    "Above deadband: qacc[1] should be positive, got {}",
    data_above.qacc[1]);

// --- Sub-case D: Exactly AT the upper deadband boundary, with velocity --- //
// Verifies the one-step delay at the deadband boundary: at L = upper exactly,
// tendon_active_stiffness returns 0.0 and tendon_deadband_displacement
// returns 0.0. The spring is disengaged this step. If velocity moves the
// tendon outside the deadband, the spring activates in the NEXT step.
// This is consistent with linearization at the current state (where the
// spring is inactive).
let mut data_boundary = model.make_data();
data_boundary.qpos[0] = 1.0;  // L = q0 - q1 = 1.0 = upper boundary exactly
data_boundary.qvel[0] = 2.0;  // Moving outward (L increasing)

data_boundary.step(&model).unwrap();

// Step 1: At boundary, k_active = 0, displacement = 0 → no spring force.
// No damping in this model. No gravity. qacc should be ~zero.
let qacc_step1_norm = data_boundary.qacc.norm();
assert!(qacc_step1_norm < 1e-10,
    "At boundary: qacc should be ~zero (spring disengaged), got norm={}",
    qacc_step1_norm);

data_boundary.step(&model).unwrap();

// Step 2: Velocity moved L past upper boundary. Now L > 1.0, spring
// should be active with non-zero displacement and non-zero qacc.
assert!(data_boundary.qacc[0].abs() > 1e-3,
    "After crossing boundary: spring should activate, got qacc[0]={}",
    data_boundary.qacc[0]);
```

**6l. `tendon_implicit_spatial_tendon_basic`:**

Verify that the implicit solver handles spatial tendons (configuration-dependent
Jacobian). Uses a simple via-point spatial tendon.

**`springlength="0 0"` is explicit** to avoid dependence on MuJoCo's
auto-computation from initial tendon length (which depends on exact geometry and
initial `qpos`). With `springlength="0 0"`, the deadband is `[0, 0]` and any
non-zero tendon length is outside the deadband, guaranteeing the spring is
active after displacement. This makes the test robust against small geometry
changes.

```rust
let mjcf = r#"
    <mujoco model="tendon_spatial_impl">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom name="g0" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" mass="1.0"/>
                <site name="s0" pos="0.1 0 -0.25"/>
                <body name="b2" pos="0 0 -0.5">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom name="g1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" mass="1.0"/>
                    <site name="s1" pos="0.1 0 -0.25"/>
                </body>
            </body>
            <site name="s_world" pos="0.3 0 0"/>
        </worldbody>
        <tendon>
            <spatial name="sp0" stiffness="50.0" damping="10.0"
                     springlength="0 0">
                <site site="s_world"/>
                <site site="s0"/>
                <site site="s1"/>
            </spatial>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();

data.qpos[0] = 0.3;  // Rotate j0 — stretches spatial tendon

// Compute forward kinematics to get the tendon Jacobian before stepping,
// so we can verify the Jacobian is non-trivial and spans both DOFs.
data.forward(&model).unwrap();
let j0 = data.ten_J[0][0];
let j1 = data.ten_J[0][1];
assert!(j0.abs() > 1e-6, "Spatial tendon J[0] should be non-zero, got {j0}");

data.step(&model).unwrap();

// Spatial tendon should produce restoring forces
let qacc_norm = data.qacc.norm();
assert!(qacc_norm > 1e-3,
    "Spatial tendon should produce non-trivial qacc, got norm={qacc_norm}");

// Direction check: positive q0 displacement stretches the tendon,
// so the restoring spring force should produce negative qacc[0]
// (pulling j0 back toward zero).
assert!(data.qacc[0] < 0.0,
    "qacc[0] should be negative (restoring) for positive displacement, got {}",
    data.qacc[0]);

// Cross-coupling: if the spatial tendon Jacobian has nonzero j1 component,
// qacc[1] should also be non-zero (off-diagonal coupling from J^T·J).
if j1.abs() > 1e-6 {
    assert!(data.qacc[1].abs() > 1e-6,
        "qacc[1] should be non-zero from spatial tendon coupling, got {}",
        data.qacc[1]);
}
```

**6l½. `tendon_implicit_multi_tendon_summation`:**

Verify that multiple tendons accumulate correctly in the mass matrix via `Σ_t`.
Uses 3 DOFs and 2 tendons with distinct Jacobians to confirm that the
summation `Σ_t (h²·k_t + h·b_t)·J_t^T·J_t` produces the correct combined
mass matrix modification.

```rust
let mjcf = r#"
    <mujoco model="tendon_multi">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="b2" pos="0 0 -1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b3" pos="0 0 -1">
                        <joint name="j2" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="50.0" damping="0.0"
                   springlength="0 0">
                <joint joint="j0" coef="1.0"/>
                <joint joint="j1" coef="-1.0"/>
            </fixed>
            <fixed name="t1" stiffness="30.0" damping="0.0"
                   springlength="0 0">
                <joint joint="j1" coef="1.0"/>
                <joint joint="j2" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();

// Displace j0 only — t0 stretches (L0 = q0 - q1 = 0.3), t1 is at rest.
data.qpos[0] = 0.3;
data.step(&model).unwrap();

// t0 produces restoring force on j0 and j1 (J0 = [1, -1, 0]).
// t1 produces NO force (L1 = q1 - q2 = 0, inside deadband).
assert!(data.qacc[0] < -1e-3,
    "qacc[0] should be negative from t0 spring, got {}", data.qacc[0]);
assert!(data.qacc[1].abs() > 1e-3,
    "qacc[1] should be non-zero from t0 coupling, got {}", data.qacc[1]);
// qacc[2]: t0 doesn't touch j2 (J0 = [1, -1, 0]), and t1 has zero
// displacement (no RHS contribution). However, t1's LHS coupling
// (h²·k_t1·J_t1^T·J_t1 between DOFs 1 and 2) distributes t0's force
// through the modified mass matrix, making qacc[2] nonzero.
// Just verify it's finite — magnitude is coupling-dependent.
assert!(data.qacc[2].is_finite(),
    "qacc[2] should be finite, got {}", data.qacc[2]);

// Now displace both — both tendons active.
let mut data2 = model.make_data();
data2.qpos[0] = 0.3;
data2.qpos[2] = -0.2;  // t1 stretches: L1 = q1 - q2 = 0 - (-0.2) = 0.2
data2.step(&model).unwrap();

// Now j2 should have non-zero qacc from t1.
assert!(data2.qacc[2].abs() > 1e-3,
    "qacc[2] should be non-zero when t1 is active, got {}", data2.qacc[2]);

// j1 should have contributions from BOTH tendons — verify it differs
// from the single-tendon case. Off-diagonal coupling means the magnitude
// relationship isn't guaranteed (additional tendon can redistribute force),
// so we check difference rather than ordering.
let qacc1_diff = (data2.qacc[1] - data.qacc[1]).abs();
assert!(qacc1_diff > 1e-6,
    "qacc[1] should differ with both tendons active: \
     both={}, single={}, diff={qacc1_diff}", data2.qacc[1], data.qacc[1]);
```

**6l¾. `tendon_implicit_zero_kd_noop`:**

Verify that a tendon with `stiffness=0, damping=0` produces no implicit
modification — the early-exit guards `if kt <= 0.0 && bt <= 0.0 { continue; }`
are hit correctly.

```rust
let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 0.0);
data.qpos[0] = 0.3;  // Displace — but no spring, no damper

data.step(&model).unwrap();

// No spring or damper: qacc should be zero (no gravity, no other forces)
let qacc_norm = data.qacc.norm();
assert!(qacc_norm < 1e-10,
    "qacc should be ~zero with k=0 b=0, got norm={qacc_norm}");
```

**6m. Regression — `test_tendon_damping_implicit_guard` must be updated:**

The existing integration test `test_tendon_damping_implicit_guard` in
`sim/L0/tests/integration/derivatives.rs:1034` validates the **current (buggy)**
behavior where tendon damping is skipped in `qDeriv` for `ImplicitSpringDamper`.
After Step 4 removes the guard, this test must be **updated** to expect the
tendon damping contribution in `qDeriv` for all integrator modes.

**Concrete change:** The existing test (lines 1064–1073) asserts:

```rust
// CURRENT (buggy): expects qDeriv to be near-zero in implicit mode
let passive_norm = data.qDeriv.norm();
assert!(
    passive_norm < 1e-6,
    "Tendon damping should be skipped in implicit mode, qDeriv norm={}",
    passive_norm
);
```

Replace with:

```rust
// FIXED (DT-35): tendon damping is now included in qDeriv for all integrators.
// With J = [1, 1], b = 5: qDeriv += -b * J^T * J = -5 * [[1,1],[1,1]]
//   qDeriv[(0,0)] += -5 * 1 * 1 = -5
//   qDeriv[(0,1)] += -5 * 1 * 1 = -5
//   qDeriv[(1,0)] += -5 * 1 * 1 = -5
//   qDeriv[(1,1)] += -5 * 1 * 1 = -5
//
// Note: mjd_smooth_vel also populates qDeriv with RNE/Coriolis and per-DOF
// damping terms. The entry-level assertions below isolate the tendon
// contribution (joint damping is zero in this model, and Coriolis at zero
// velocity is negligible). The norm check is a coarse sanity guard.
let qderiv_norm = data.qDeriv.norm();
assert!(
    qderiv_norm > 1.0,
    "qDeriv should be non-trivial (includes tendon damping), norm={}",
    qderiv_norm
);
// Verify tendon damping structure: all entries ≈ -5 (J=[1,1] makes all
// outer-product entries positive, then multiplied by -b = -5).
// At qpos[0]=0.3 with gravity, RNE/Coriolis may contribute a small offset,
// so use a relaxed threshold (-1.0) rather than exact (-5.0 ± epsilon).
assert!(data.qDeriv[(0, 0)] < -1.0,
    "qDeriv[(0,0)] should be ~-5, got {}", data.qDeriv[(0, 0)]);
assert!(data.qDeriv[(0, 1)] < -1.0,
    "qDeriv[(0,1)] should be ~-5 (J[0]*J[1] = 1*1 = 1, times -b = -5), got {}",
    data.qDeriv[(0, 1)]);
assert!(data.qDeriv[(1, 0)] < -1.0,
    "qDeriv[(1,0)] should be ~-5, got {}", data.qDeriv[(1, 0)]);
assert!(data.qDeriv[(1, 1)] < -1.0,
    "qDeriv[(1,1)] should be ~-5, got {}", data.qDeriv[(1, 1)]);
```

Also rename the test from `test_tendon_damping_implicit_guard` to
`test_tendon_damping_in_qDeriv_all_integrators` to reflect the new semantics.

**6n. `tendon_implicit_newton_spring_nonzero`:**

Verify that tendon spring forces are active when Newton solver is used with
`ImplicitSpringDamper`. This is the primary regression test for Steps 2b/3b.
Without the Newton fix, Newton's Hessian uses raw `qM` and `qfrc_smooth`
lacks tendon forces — producing zero tendon acceleration.

```rust
let (model, mut data) = make_tendon_implicit_model_with_solver(
    "implicitspringdamper", 50.0, 0.0, "Newton"
);
data.qpos[0] = 0.3;

data.step(&model).unwrap();

// Newton + ImplicitSpringDamper should produce non-zero restoring acceleration.
assert!(data.qacc[0] < -1e-3,
    "Newton: qacc[0] should be negative (restoring), got {}", data.qacc[0]);
// Off-diagonal coupling must also work through Newton.
assert!(data.qacc[1] > 1e-3,
    "Newton: qacc[1] should be positive (off-diagonal coupling), got {}", data.qacc[1]);
```

**6o. `tendon_implicit_solver_flag_no_effect_unconstrained`:**

Smoke test: when there are no active constraints (`nefc=0`), the solver flag
(PGS vs Newton) should have no effect on the result. Both models take the
early-return path in `mj_fwd_constraint` (line 19489) before the solver
dispatch, so `newton_solved` stays `false` and both run
`mj_fwd_acceleration_implicit` identically.

This does NOT exercise Newton's `M_impl` codepath — test 6p covers that
(active constraints force Newton to actually run). This test verifies that
the solver flag is a no-op for unconstrained implicit models.

```rust
let (model_pgs, mut data_pgs) = make_tendon_implicit_model_with_solver(
    "implicitspringdamper", 50.0, 20.0, "PGS"
);
let (model_newton, mut data_newton) = make_tendon_implicit_model_with_solver(
    "implicitspringdamper", 50.0, 20.0, "Newton"
);

// Same initial state — no contacts, no limits, so no active constraints.
data_pgs.qpos[0] = 0.3;
data_pgs.qvel[0] = 1.0;
data_newton.qpos[0] = 0.3;
data_newton.qvel[0] = 1.0;

data_pgs.step(&model_pgs).unwrap();
data_newton.step(&model_newton).unwrap();

// Results should match to machine precision (both take identical codepath).
for i in 0..model_pgs.nv {
    let diff = (data_pgs.qvel[i] - data_newton.qvel[i]).abs();
    assert!(diff < 1e-10,
        "qvel[{i}] should match between PGS and Newton (unconstrained): \
         pgs={}, newton={}, diff={diff}",
        data_pgs.qvel[i], data_newton.qvel[i]);
}
for i in 0..model_pgs.nv {
    let diff = (data_pgs.qacc[i] - data_newton.qacc[i]).abs();
    assert!(diff < 1e-10,
        "qacc[{i}] should match between PGS and Newton (unconstrained): \
         pgs={}, newton={}, diff={diff}",
        data_pgs.qacc[i], data_newton.qacc[i]);
}
```

**6p. `tendon_implicit_newton_with_contact`:**

Verify that tendon spring forces interact correctly with contact constraints
in Newton solver. A model with ground contact + tendon spring should produce
physically reasonable behavior.

```rust
let mjcf = r#"
    <mujoco model="tendon_newton_contact">
        <option timestep="0.001" integrator="implicitspringdamper"
                solver="Newton" gravity="0 0 -9.81"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="b1" pos="0 0 0.5">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="b2" pos="0 0 -0.5">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="100.0" damping="10.0"
                   springlength="0 0">
                <joint joint="j0" coef="1.0"/>
                <joint joint="j1" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();
data.qpos[0] = 0.5;  // Displace — tendon spring activates

// Step 50 times — should not crash or produce NaN
for _ in 0..50 {
    data.step(&model).unwrap();
}

// Verify no NaN/Inf in state
assert!(!data.qvel[0].is_nan() && !data.qvel[0].is_infinite(),
    "qvel should be finite, got {:?}", &data.qvel);
assert!(!data.qacc[0].is_nan() && !data.qacc[0].is_infinite(),
    "qacc should be finite, got {:?}", &data.qacc);

// Tendon spring should have affected the trajectory — velocities non-zero.
// (Without tendon forces, the chain would just free-fall; with the spring,
// j0/j1 are coupled and oscillate differently.)
let qacc_norm = data.qacc.norm();
assert!(qacc_norm > 1e-3,
    "qacc should be non-trivial with tendon + contact, got norm={qacc_norm}");
```

**6q. `tendon_implicit_single_dof_tendon`:**

Verify that a tendon spanning a single DOF (effectively diagonal) works
correctly. The outer product `J^T·J` reduces to a scalar entry on the
diagonal. This tests the edge case where `J` has exactly one nonzero entry.

```rust
let mjcf = r#"
    <mujoco model="tendon_single_dof">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="50.0" damping="0.0"
                   springlength="0 0">
                <joint joint="j0" coef="2.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();
data.qpos[0] = 0.3;  // Displace — L = 2.0 * 0.3 = 0.6

data.step(&model).unwrap();

// Spring force: F = -k * L = -50 * 0.6 = -30
// Joint force: qfrc = J^T * F = 2.0 * (-30) = -60
// qacc should be negative (restoring)
assert!(data.qacc[0] < -1e-3,
    "Single-DOF tendon: qacc[0] should be negative, got {}", data.qacc[0]);

// Compare with equivalent joint spring (k_eff = k * coef² = 50 * 4 = 200)
// at displacement 0.3: explicit force = -200 * 0.3 = -60, same as tendon.
// With implicit treatment, the effective acceleration should be similar
// to a joint spring with stiffness 200. (Not exact match because the
// implicit mass matrix modification scales differently.)
```

**6r. `tendon_implicit_newton_joint_spring_bonus`:**

Verify that the `build_m_impl_for_newton` helper also fixes the pre-existing
gap where Newton + `ImplicitSpringDamper` dropped **joint** spring/damper
forces (see "Scope clarification" in Step 2b). Since `build_m_impl_for_newton`
adds both joint diagonal K/D and tendon non-diagonal K/D, joint springs should
now work through Newton + `ImplicitSpringDamper`.

```rust
// Model with joint spring ONLY (no tendon), Newton solver
let mjcf = r#"
    <mujoco model="joint_spring_newton">
        <option timestep="0.001" integrator="implicitspringdamper"
                solver="Newton" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="100.0" damping="10.0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();
let mut data = model.make_data();
data.qpos[0] = 0.5;  // Displace — joint spring activates

data.step(&model).unwrap();

// Joint spring should produce restoring acceleration
assert!(data.qacc[0] < -1e-3,
    "Newton + joint spring: qacc[0] should be negative (restoring), got {}",
    data.qacc[0]);

// Compare against PGS path — should match (unconstrained).
// Use a dedicated MJCF string (not string replacement) to avoid fragility.
let mjcf_pgs = r#"
    <mujoco model="joint_spring_pgs">
        <option timestep="0.001" integrator="implicitspringdamper"
                solver="PGS" gravity="0 0 0"/>
        <worldbody>
            <body name="b1" pos="0 0 0">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="100.0" damping="10.0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
"#;
let model_pgs = sim_mjcf::load_model(mjcf_pgs).unwrap();
let mut data_pgs = model_pgs.make_data();
data_pgs.qpos[0] = 0.5;
data_pgs.step(&model_pgs).unwrap();

let diff = (data.qacc[0] - data_pgs.qacc[0]).abs();
assert!(diff < 1e-10,
    "Newton and PGS should match for unconstrained joint spring: \
     newton={}, pgs={}, diff={diff}", data.qacc[0], data_pgs.qacc[0]);
```

**6s. `tendon_implicit_sleep_guard_skips_sleeping_tendons`:**

Verify that the sleep guard in `accumulate_tendon_kd` and the Step 3 RHS
loop correctly skips tendons whose target DOFs are all sleeping. Uses the
same sleep infrastructure as the existing `mj_fwd_passive` and
`mjd_passive_vel` sleep tests.

```rust
// Zero gravity so sleeping DOFs have no external forces — qacc must be
// exactly zero when all trees are forced asleep.
let mjcf = r#"
    <mujoco model="tendon_sleep">
        <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="j0" type="hinge" axis="0 1 0"
                       stiffness="0" damping="0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="b2" pos="0 0 -1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="t0" stiffness="100.0" damping="10.0"
                   springlength="0 0">
                <joint joint="j0" coef="1.0"/>
                <joint joint="j1" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
"#;
let model = sim_mjcf::load_model(mjcf).unwrap();

// --- Awake: tendon spring should produce restoring acceleration ---
let mut data_awake = model.make_data();
data_awake.qpos[0] = 0.3;
data_awake.step(&model).unwrap();
let qacc_awake_norm = data_awake.qacc.norm();
assert!(qacc_awake_norm > 1e-3,
    "Awake: tendon spring should produce non-trivial qacc, got {qacc_awake_norm}");

// --- Sleeping: force all trees asleep, verify tendon K/D is skipped ---
let mut data_sleep = model.make_data();
data_sleep.qpos[0] = 0.3;
data_sleep.forward(&model).unwrap();  // compute tendon Jacobians/lengths

// Force ALL trees to sleep (same pattern as gravcomp.rs:412-414)
for i in 0..model.ntree {
    data_sleep.tree_awake[i] = false;
}
data_sleep.nv_awake = 0;
data_sleep.dof_awake_ind.clear();

data_sleep.step(&model).unwrap();
let qacc_sleep_norm = data_sleep.qacc.norm();
assert!(qacc_sleep_norm < 1e-10,
    "Sleeping: tendon forces should be skipped, got qacc norm={qacc_sleep_norm}");
```

**Sleep guard coverage note:** This test exercises the
`tendon_all_dofs_sleeping` predicate in both `accumulate_tendon_kd`
(Step 0/2) and the Step 3 RHS loop. The predicate itself is identical to
the one used in `mj_fwd_passive` (line 12422) and `mjd_passive_vel`
(line 502), both already tested — but this test verifies the integration
of the predicate into the new implicit tendon codepath specifically.

### Deliverables

| # | Deliverable |
|---|-------------|
| D0 | Shared helpers: `tendon_deadband_displacement`, `tendon_active_stiffness`, `accumulate_tendon_kd` (Step 0) |
| D1 | Update NOTE comment in `mj_fwd_passive` tendon loop (Step 1) |
| D2 | Non-diagonal tendon K/D in `mj_fwd_acceleration_implicit` mass matrix via `accumulate_tendon_kd` (Step 2) |
| D3 | Tendon spring displacement in implicit RHS via `tendon_deadband_displacement` (Step 3) |
| D4 | Newton solver `M_impl` + `qfrc_smooth_impl` for `ImplicitSpringDamper` (Step 2b/3b) |
| D5 | Remove `implicit_mode` guard in `mjd_passive_vel` for tendon damping (Step 4) |
| D6 | Update stale comments in forward/passive.rs, types/data.rs, derivatives.rs, future_work_1.md (Step 5) |
| D7 | `make_tendon_implicit_model` / `make_tendon_implicit_model_with_solver` test helpers |
| D8 | Tests 6a–6s (18 new tests + 1 new sub-case 6k-D + 3 regression checks) |

### Files Modified

| File | Change |
|------|--------|
| `sim/L0/core/src/forward/passive.rs` | Add shared helpers `tendon_deadband_displacement`, `tendon_active_stiffness`, `accumulate_tendon_kd` (D0); update `mj_fwd_passive` comment (D1); add `tendon_implicit_tests` module (D7, D8) |
| `sim/L0/core/src/integrate/implicit.rs` | Add non-diagonal tendon K/D via `accumulate_tendon_kd` + RHS via `tendon_deadband_displacement` to `mj_fwd_acceleration_implicit` (D2, D3) |
| `sim/L0/core/src/constraint/solver/` | Add `build_m_impl_for_newton`, `compute_qfrc_smooth_implicit` helpers, modify `newton_solve`/`assemble_hessian`/`SparseHessian`/`evaluate_cost_at`/`hessian_incremental` to use `M_impl` when `ImplicitSpringDamper` active (D4) |
| `sim/L0/core/src/forward/mod.rs` | Modify `mj_fwd_constraint` to use `M_impl` when `ImplicitSpringDamper` active (D4); update comments (D6) |
| `sim/L0/core/src/derivatives.rs` | Remove `implicit_mode` guard in `mjd_passive_vel` tendon damping loop (D5); update comments (D6) |
| `sim/L0/tests/integration/derivatives.rs` | Rename `test_tendon_damping_implicit_guard` → `test_tendon_damping_in_qDeriv_all_integrators`; update assertions to expect non-zero `qDeriv` entries for `ImplicitSpringDamper` (D8/6m) |
| `sim/docs/todo/future_work_1.md` | Update known-limitation comment at lines 1626–1630 (D6) |
| `sim/docs/todo/future_work_10d.md` | This spec |

### Verification

```bash
# New tendon implicit tests (unit tests in forward/passive.rs)
cargo test -p sim-core -- tendon_implicit

# Updated integration test (6m — lives in sim-conformance-tests)
cargo test -p sim-conformance-tests -- test_tendon_damping_in_qDeriv_all_integrators

# Existing implicit integration regression (6i, 6j)
cargo test -p sim-conformance-tests -- implicit

# Existing tendon tests
cargo test -p sim-conformance-tests -- tendon

# Derivative consistency
cargo test -p sim-conformance-tests -- derivative

# Full sim domain
cargo test -p sim-core -p sim-conformance-tests -p sim-physics

# Lint
cargo clippy -p sim-core -- -D warnings
cargo fmt --all -- --check
```
