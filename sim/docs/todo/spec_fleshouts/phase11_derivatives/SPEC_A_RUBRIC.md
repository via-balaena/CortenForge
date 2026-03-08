# Spec A — Analytical Position Derivatives (`mjd_smooth_pos`): Spec Quality Rubric

Grades the Spec A spec on 11 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to the finite-difference position columns it
replaces."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest.

> **Extension spec (split mandate):** `mjd_smooth_pos` does NOT exist in
> MuJoCo. MuJoCo computes ALL transition derivative position columns via
> finite differencing (`mjd_stepFD` in `engine_derivative_fd.c`). Spec A is
> a **CortenForge extension** that replaces FD position columns with
> analytical computation. The split mandate is:
>
> - **Conformance subset:** The final transition A matrix (position columns)
>   must match what pure FD produces — which matches MuJoCo's output. The
>   analytical computation is an implementation strategy; the numerical
>   result must be equivalent.
> - **Extension subset:** The `mjd_smooth_pos()` function itself, the
>   `Data.qDeriv_pos` field, and the chain-rule algorithm through FK/RNE/
>   passive are CortenForge extensions. No MuJoCo C source to port —
>   mathematical correctness is validated against FD.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source discovered a critical
scope correction from the umbrella spec.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "`mjd_smooth_pos()` in `engine_derivative.c` — analytical derivatives of smooth forces with respect to positions" | **Function does NOT exist.** MuJoCo has no `mjd_smooth_pos`. All position columns use FD via `mjd_stepFD` with `mj_stepSkip(m, d, mjSTAGE_NONE, ...)`. Verified: `engine_derivative.c` contains only velocity derivative functions (`mjd_smooth_vel`, `mjd_passive_vel`, `mjd_actuator_vel`, `mjd_rne_vel`). `engine_derivative.h` declares no position derivative functions. MuJoCo header `mujoco.h` lists only `mjd_transitionFD`, `mjd_inverseFD`, `mjd_subQuat`, `mjd_quatIntegrate` as public `mjd_*` API. | **Reclassify as CortenForge extension.** Split mandate applies. Validation against FD (not against nonexistent MuJoCo function). |
| "Three components: FK position derivatives, RNE position derivatives, passive force position derivatives" | Components are mathematically correct (chain rule through FK → forces). MuJoCo doesn't compute them analytically, but the math is standard rigid-body mechanics. | **In scope** as extension. Validate each component against FD. |
| "Output stored in `Data.qDeriv_pos`" | MuJoCo has no `qDeriv_pos` field. CortenForge extension. | **In scope** as extension. New Data field. |
| "Integration into `mjd_transition_hybrid()` to replace FD position columns" | CortenForge's hybrid path already uses analytical velocity columns (also a CortenForge extension — MuJoCo uses pure FD for everything). Replacing FD position columns is a natural extension. | **In scope.** Final A matrix must match FD A matrix. |
| "≥1.5× speedup over FD position columns" | Not a conformance item — purely performance. | **In scope** as performance AC. |
| Actuator position derivatives (`∂qfrc_actuator/∂qpos`) | Actuator force depends on position through `actuator_length` → `gain(L)` and `bias(L)`. MuJoCo captures this via FD. Analytical: `∂force/∂L · ∂L/∂qpos = ∂force/∂L · moment`. | **In scope.** Must include for completeness — position-dependent actuator forces (Affine gain with L term, muscle FL curve) need analytical position derivatives. |

**Final scope:**

1. `mjd_smooth_pos(model, data)` — analytical position force derivatives stored in `Data.qDeriv_pos`. Contains `∂qfrc_smooth/∂qpos − (∂M/∂q)·qacc` (the RNE component differentiates RNEA at actual acceleration to capture mass matrix position dependence — see EGT-10).
2. Three sub-components: `mjd_passive_pos`, `mjd_actuator_pos`, `mjd_rne_pos`
3. `Data.qDeriv_pos: DMatrix<f64>` (nv × nv) — new field
4. Integration into `mjd_transition_hybrid()` — replace FD position columns with analytical
5. `IntegrationDerivatives.dqpos_dqpos` activated (currently `#[allow(dead_code)]`)
6. Validation: analytical position columns match FD within `1e-6` relative tolerance

---

## Empirical Ground Truth

### EGT-1: MuJoCo has no analytical position derivatives

**MuJoCo version:** 3.x (current main branch, verified 2026-03-08).
**Source files examined:**
- `engine_derivative.c` — contains ONLY velocity derivative functions. No
  function with "pos" in its name computes `∂qfrc/∂qpos`.
- `engine_derivative.h` — declares `mjd_smooth_vel`, `mjd_actuator_vel`,
  `mjd_passive_vel`, `mjd_rne_vel_dense`, `mjd_flexInterp_mulKD`,
  `mjd_flexInterp_addH`, `mjd_subQuat`, `mjd_quatIntegrate`. No
  `mjd_smooth_pos` or similar.
- `engine_derivative_fd.c` → `mjd_stepFD()` — ALL transition derivative
  columns (position, velocity, activation, control) use finite differencing.
  Position columns: `mj_integratePos(m, d->qpos, dpos, eps)` then
  `mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor)`.
- `mujoco.h` — public API has only `mjd_transitionFD`, `mjd_inverseFD`,
  `mjd_subQuat`, `mjd_quatIntegrate`.

**Exception:** `mjd_flexInterp_addH` adds `(h² + h·damping) · J^T·K·J` to
the implicit integrator's system matrix for flex interpolation. This is a
narrow position stiffness term for a specific feature, not a general
`∂qfrc/∂qpos` computation. CortenForge's implicit integrator already handles
this via separate infrastructure (DT-35).

**Conclusion:** Spec A is a CortenForge extension. The MuJoCo C source
provides no reference implementation to port. Correctness is validated by
comparing analytical results against FD results (which produce
MuJoCo-conformant transition matrices).

### EGT-2: Three analytical components and their mathematical basis

The smooth force `qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias`.
Its position derivative decomposes into three independent terms:

**Component 1 — Passive position derivatives (`∂qfrc_passive/∂qpos`):**
- **Joint springs (Hinge/Slide):** `qfrc_spring[dof] -= k · (q − q_ref)`.
  Derivative: `∂/∂qpos[dof] = −k` (diagonal, constant stiffness).
  Location: `passive.rs:895` (1-DOF), `passive.rs:930` (Free translational).
- **Joint springs (Ball/Free rotational):** `qfrc_spring[dof..+3] -= k · subquat(q, q_ref)`.
  Derivative: `−k · ∂subquat/∂q` — requires the quaternion subtraction
  Jacobian from DT-52's `mjd_sub_quat()`. 3×4 tangent-space Jacobian
  (3 rotational DOFs × 4 quaternion coordinates, projected to 3×3 tangent).
  Location: `passive.rs:949-951` (Free rot), `passive.rs:967-969` (Ball).
- **Tendon springs:** `qfrc += J^T · k · (ref − length)` when length is
  outside the deadband `[lower, upper]`. Derivative:
  `∂/∂qpos = J^T · k · (∂length/∂qpos)` where `∂length/∂qpos` is the
  tendon Jacobian itself (since `length = J · qpos` for fixed-point tendons).
  Plus the cross-term `(∂J^T/∂qpos) · force` for curved tendons.
  Location: `passive.rs:413-421` (spring computation), `passive.rs:444+`
  (force application via `ten_J`).
- **Flex edge springs:** `qfrc_spring += J_edge^T · k · (rest − length)`.
  Position derivative: `J_edge^T · k · J_edge` (same pattern as tendon
  springs). For free-vertex flexes, `J_edge` is `[−vec^T, +vec^T]`.
  Plus force × `∂J_edge/∂qpos` cross-term for body-attached vertices.
  Flex bending forces (cotangent Laplacian, Bridson dihedral) also contribute
  to `qfrc_spring` and have position-dependent derivatives through vertex
  position geometry.
  Location: `passive.rs:496-655` (edge springs, bending forces).
- **Fluid forces:** Position-dependent through body orientation and velocity.
  Complex chain rule through xmat → drag coefficients. Analytical velocity
  derivatives exist (`mjd_fluid_vel`), but position derivatives require
  `∂xmat/∂qpos` chain rule. Candidate for deferral to FD fallback.
- **Gravity compensation:** `∂qfrc_gravcomp/∂qpos` — through body Jacobian
  position dependence (`mj_apply_ft` at `xipos[b]`). Candidate for deferral.
  Location: `rne.rs:325-373`.

> **Deferred components warning:** If `mjd_passive_pos` omits fluid,
> gravcomp, or flex position derivatives, the analytical result will NOT
> match FD for models with those features. The spec must either: (a)
> implement all components, (b) have the hybrid path selectively fall back
> to FD for models with deferred components, or (c) clearly document which
> model features cause reduced accuracy and adjust tolerances. This
> directly impacts conformance (P1 split mandate) because the transition
> A matrix position columns will differ from FD on affected models.

**Component 2 — Actuator position derivatives (`∂qfrc_actuator/∂qpos`):**
- Actuator force: `force = gain(L, V, act) · input + bias(L, V, act)`.
- Position dependence is through actuator length `L`:
  `∂force/∂qpos = (∂gain/∂L · input + ∂bias/∂L) · ∂L/∂qpos`.
- `∂L/∂qpos = moment^T` (the actuator moment arm IS the length Jacobian).
- **Force chain-rule term:** `qDeriv_pos += moment · (∂force/∂L) · moment^T`.
- For `GainType::Fixed`: `∂gain/∂L = 0` (no position dependence).
- For `GainType::Affine`: `∂gain/∂L = gainprm[1]`.
- For `GainType::Muscle/HillMuscle`: `∂gain/∂L` from force-length (FL)
  curve derivative.
- Same pattern for bias types.
- **Moment-arm cross-term:** The full derivative of `qfrc = moment(q) · force`
  includes `(∂moment/∂qpos) · force`, which is the change in the moment arm
  geometry itself. This term is ZERO for Joint/JointInParent transmissions
  (moment = constant gear ratio) and for fixed tendons (J is constant).
  It is NONZERO for site/body/slidercrank/spatial-tendon transmissions where
  the moment arm depends on body poses through FK. The spec must address
  this: either compute analytically (requires FK Jacobian of the moment arm)
  or defer to FD fallback for non-joint/non-fixed-tendon actuators.
  Location: `actuation.rs:72-359` (transmission computations).
- Location: `derivatives.rs:532-651` (velocity version as template).

**Component 3 — RNE position derivatives (`−∂RNEA(q,v,qacc)/∂qpos`):**

> **Critical distinction from velocity derivatives:** `mjd_rne_vel`
> differentiates RNEA at zero joint acceleration (the bias force). For
> position derivatives, `mjd_rne_pos` must differentiate RNEA at the
> **actual acceleration** `qacc` — see EGT-10. Using zero acceleration
> misses the `(∂M/∂q)·qacc` term (mass matrix position derivative).

- **Gravity torques:** `τ_g = J^T · (M_subtree · g)`. Position derivative:
  `∂τ_g/∂qpos` through the Jacobian's position dependence and the subtree
  COM position dependence. For hinge: `∂(r × F · axis)/∂q` where
  `r = subtree_com − jnt_pos` depends on `qpos` through FK.
  Location: `rne.rs:64-118`.
- **Coriolis/centrifugal:** Quadratic in velocity, but the Coriolis matrix
  `C(q)` itself depends on position (through body poses and inertias in
  world frame). `∂(C·v)/∂qpos` requires differentiating the spatial inertia
  and velocity transforms through the kinematic tree.
  Location: `rne.rs:120+` (Coriolis computation).
- **Gyroscopic:** `ω × (I·ω)` where `I` is body-frame inertia (constant)
  and `ω` is body-frame angular velocity. Position derivative: `∂ω/∂qpos`
  through the body orientation quaternion. For Ball/Free joints in body
  frame, the inertia doesn't change with position, but the mapping from
  generalized coordinates to body-frame angular velocity does.
- **Mass matrix acceleration term `(∂M/∂q)·qacc`:** The mass matrix
  `M(q)` depends on position through body transforms. The full inverse
  dynamics derivative is `∂RNEA(q,v,qacc)/∂q = (∂M/∂q)·qacc + ∂(C·v+g)/∂q`.
  Using zero acceleration gives only `∂(C·v+g)/∂q`, missing the first term.
  In the RNEA recursive structure, this manifests as:
  (a) `∂(X_b)/∂q · cacc_full[parent]` — the parent's FULL acceleration
  (including qacc contributions) transforms differently with position changes,
  (b) `∂S_b/∂q · qacc_b` — the motion subspace position dependence applied
  to actual joint accelerations (not just bias). See EGT-10 for details.

### EGT-3: Codebase context — files and match sites

| File | Lines | What Spec A touches | Risk |
|------|-------|---------------------|------|
| `sim/L0/core/src/derivatives.rs` | ~2,700 | New `mjd_smooth_pos()`, `mjd_passive_pos()`, `mjd_actuator_pos()`, `mjd_rne_pos()` functions. Modify `mjd_transition_hybrid()` lines 1448–1511 (replace FD position loop with analytical). | **High** — intra-file. 4 new functions (~400–600 lines total). FD position loop replaced. |
| `sim/L0/core/src/derivatives.rs` | 1092–1097 | Remove `#[allow(dead_code)]` from `IntegrationDerivatives.dqpos_dqpos`. Wire into hybrid position columns. | **Medium** — `dqpos_dqpos` is currently computed but unused. |
| `sim/L0/core/src/types/data.rs` | ~580 | New `qDeriv_pos: DMatrix<f64>` field. Possibly new scratch Jacobians `deriv_Dcvel_pos`, `deriv_Dcacc_pos`, `deriv_Dcfrc_pos`. | **Medium** — additive field. Must update `make_data()`, `Clone`, `reset()`. |
| `sim/L0/core/src/lib.rs` | ~265 | Export `mjd_smooth_pos` (and sub-functions if public). | **Low** — additive. |
| `sim/L0/tests/integration/derivatives.rs` | varies | New tests for position derivative validation. | **Low** — additive tests. |

**Existing tests at risk:**
- `derivatives.rs` integration tests (Steps 0–7): Should be UNCHANGED.
  Analytical position derivatives replace FD but produce same result.
- Hybrid transition tests: Values should remain within tolerance. If any
  test values change, they must be MORE accurate (closer to pure FD).
- Phase 4 39-test suite: Unaffected (doesn't test position derivatives).

**Key match sites that WON'T need changes (unlike enum-extension specs):**
- No new enum variants. No new joint types. No parser changes.
- No builder changes. No Model field changes.
- All changes are in the derivative computation layer (derivatives.rs + Data fields).

### EGT-4: MuJoCo's `mjd_smooth_vel` as algorithmic template

MuJoCo's `mjd_smooth_vel` (verified in `engine_derivative.c`):
```c
void mjd_smooth_vel(const mjModel* m, mjData* d, int flg_bias) {
    // clear qDeriv
    mju_zero(d->qDeriv, m->nD);
    // qDeriv += d qfrc_actuator / d qvel
    mjd_actuator_vel(m, d);
    // qDeriv += d qfrc_passive / d qvel
    mjd_passive_vel(m, d);
    // qDeriv -= d qfrc_bias / d qvel; optional
    if (flg_bias) { mjd_rne_vel(m, d); }
}
```

CortenForge's `mjd_smooth_vel` (derivatives.rs:1020–1025) follows this
exactly. `mjd_smooth_pos` should mirror this pattern:
```rust
pub fn mjd_smooth_pos(model: &Model, data: &mut Data) {
    data.qDeriv_pos.fill(0.0);
    mjd_passive_pos(model, data);
    mjd_actuator_pos(model, data);
    mjd_rne_pos(model, data);
}
```

The `flg_bias` parameter is always true for transition derivatives (gravity
position derivatives are always needed). Spec A should hardcode this.

### EGT-5: `IntegrationDerivatives.dqpos_dqpos` — currently unused

`compute_integration_derivatives()` (derivatives.rs:1111–1227) computes
`dqpos_dqpos` for all joint types but it's marked `#[allow(dead_code)]`
(line 1096) with comment: "Currently unused — position columns use FD which
captures this implicitly. Retained for potential future fully-analytical
position columns."

After Spec A, the hybrid path's position columns use the chain rule:
```
∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq
```
where `dvdq` comes from the integrator-specific solve:
- Euler: `dvdq = h · M⁻¹ · qDeriv_pos`
- ISD: `dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos` ← see EGT-7 (NO correction needed)
- ImplicitFast/Implicit: `dvdq = h · (M−hD)⁻¹ · qDeriv_pos`

And the velocity rows:
```
∂v⁺/∂q = dvdq
```

This parallels the velocity column structure where `∂q⁺/∂v = dqpos_dqvel · dvdv`.

### EGT-7: ISD integrator — position columns need NO stiffness correction

**Critical insight: the ISD position column formula does NOT parallel the
velocity column formula.** The velocity column has a `+h·D_diag` correction;
one might expect position columns to need `+h²·K_diag`. This is **wrong**.

**Velocity column derivation** (existing code, line 1299–1313):

The ISD step: `(M+hD+h²K) · v⁺ = M·v + h·f_ext − h·K·δq`

Differentiate w.r.t. `v` (position terms are constant):
```
(M+hD+h²K) · ∂v⁺/∂v = M + h · ∂f_ext/∂v
```
Since `f_ext` excludes implicit damping but `qDeriv` includes it (`-D`):
`∂f_ext/∂v = qDeriv + D`. Thus: RHS = `M + h·qDeriv + h·D_diag`.

```rust
// Velocity columns (existing code, line 1302-1308):
rhs[i] = qM[(i,j)] + h * qDeriv[(i,j)] + if i==j { h * d[i] } else { 0.0 };
//                                         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                                         +D cancels the -D in qDeriv
```

**Position column derivation** (Spec A — new):

Differentiate the same ISD step w.r.t. `q` (velocity terms are constant):
```
(M+hD+h²K) · ∂v⁺/∂q = h · ∂f_ext/∂q − h·K
```
Since `f_ext` excludes implicit springs but `qDeriv_pos` includes them (`-K`):
`∂f_ext/∂q = qDeriv_pos + K`. Substituting:
```
RHS = h·(qDeriv_pos + K) − h·K = h·qDeriv_pos
```

**The spring displacement term `−h·K·δq` in the ISD RHS provides its own
cancellation when differentiated.** No correction is needed.

The ISD position column formula is simply:
```
dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos
```

**Verified analytically** with 1-DOF system (m=1, k=10, h=0.01):
- Correct: `dvdq = (1.002)⁻¹ · 0.01·(−10) = −0.0998`
- Wrong (`+h²·K`): `dvdq = (1.002)⁻¹ · (−0.1 + 0.001) = −0.0988` ← **error**

**Why the asymmetry:** Velocity columns have a `M·v` term in the RHS whose
derivative gives `M` — this has no position analogue. Position columns have
a `−h·K·δq` term whose derivative `−h·K` exactly cancels the `+K` from
converting `∂f_ext/∂q` to `qDeriv_pos`. The two derivatives are structurally
different despite appearing analogous.

**The spec must derive the ISD position formula** from the implicit update
equation `(M+hD+h²K)·v⁺ = M·v + h·f_ext − h·K·δq`, differentiating w.r.t.
`q` explicitly. The derivation must NOT use velocity-column analogy.

### EGT-8: Motion subspace position dependence

For RNE position derivatives, the joint motion subspace `S(q)` depends on
ancestor joint positions:

- **Hinge:** `S = [axis_world; axis_world × r]` where `axis_world = R(q_ancestors) · axis_body`.
  When an ancestor joint rotates, the hinge axis rotates with it.
  `∂S/∂q_ancestor ≠ 0` for any ancestor that changes body orientation.
- **Slide:** Similar — `S = [0; axis_world]` where axis depends on body orientation.
- **Ball:** `S = [e_i; 0]` in body frame — but if expressed in world frame,
  `S = [R · e_i; 0]`, making it position-dependent.
- **Free:** Translation part `S = [0; e_i]` is constant; rotation part
  `S = [e_i; 0]` in world frame (constant) or `[R · e_i; 0]` in body frame.

The position derivative of the RNE forward pass requires propagating
`∂S/∂qpos` through the kinematic tree. This is the most complex part of
the position derivative algorithm — it's what makes `mjd_rne_pos` significantly
harder than `mjd_rne_vel` (where S does not depend on qvel).

### EGT-9: Constraint force position derivatives — excluded by design

The analytical computation targets `∂qfrc_smooth/∂qpos` only. The total
force derivative is `∂(qfrc_smooth + qfrc_constraint)/∂qpos`, but
`∂qfrc_constraint/∂qpos` is NOT computed analytically. This is the same
approximation already made by the **existing analytical velocity columns**:
`mjd_smooth_vel` computes `∂qfrc_smooth/∂qvel` but excludes
`∂qfrc_constraint/∂qvel`.

**Impact by model type:**
- **Contact-free models (no geoms, or `DISABLE_CONTACT`):** Analytical
  position columns match FD exactly (modulo deferred passive sub-components).
  No constraint forces → no missing derivative. This is the cleanest
  validation target.
- **Models with contacts/constraints:** The FD position columns run
  `mj_stepSkip(mjSTAGE_NONE)` which recomputes collision detection and
  constraint forces. Analytical columns miss the constraint response to
  position perturbation (new contacts, changed penetration depths).
- **Models with equality constraints only (welds, joint limits at limit):**
  `qfrc_constraint ≠ 0` even without contacts. The analytical approximation
  still applies.

**Precedent:** The existing hybrid velocity columns have this same limitation
and pass conformance testing. The error is bounded by the constraint force
magnitude × sensitivity to the perturbation. For well-behaved contact models
(smooth contacts, small penetrations), the error is typically within the FD
tolerance. The spec must document this as a known approximation, paralleling
the velocity-column precedent.

### EGT-10: `mjd_rne_pos` must use actual acceleration, not bias

**Key asymmetry with `mjd_rne_vel`:** The velocity derivative function
differentiates RNEA at zero joint acceleration (bias forces only), because
`M(q)` does not depend on velocity — the term `(∂M/∂v)·qacc = 0`. For
position derivatives, `M(q)` DOES depend on `q`, so the term
`(∂M/∂q)·qacc ≠ 0` and must be captured.

**Derivation:** The equation of motion is `M(q)·qacc = F_total`. The
acceleration-level transition derivative is:
```
∂qacc/∂q = M⁻¹ · [∂F_total/∂q − (∂M/∂q)·qacc]
```

If `qDeriv_pos` only contains `∂qfrc_smooth/∂q` (i.e., `−∂(C·v+g)/∂q`
from bias forces), the `(∂M/∂q)·qacc` term is missing. The fix: define
`mjd_rne_pos` to differentiate `RNEA(q, v, qacc)` — NOT `RNEA(q, v, 0)`.

```
RNEA(q, v, qacc) = M(q)·qacc + C(q,v)·v + g(q)
∂/∂q [RNEA(q, v, qacc)] = (∂M/∂q)·qacc + ∂(C·v+g)/∂q
```

Then `qDeriv_pos_RNE = −∂RNEA(q,v,qacc)/∂q` naturally includes
`−(∂M/∂q)·qacc`, and the transition formula `dvdq = h · M⁻¹ · qDeriv_pos`
is complete (minus constraint forces, per EGT-9).

**In the RNEA recursive structure**, this manifests as:
1. Forward pass initializes `cacc[0] = −g` (same as bias), but propagates
   with `S_b · qacc_b` terms included: `cacc_full[b] = X_b · cacc_full[parent]
   + cvel × S · qdot + S · qacc_b`.
2. Position derivative: `∂cacc_full[b]/∂q` includes
   `(∂X_b/∂q) · cacc_full[parent]` (transform derivative applied to FULL
   parent acceleration, not just bias) and `(∂S_b/∂q) · qacc_b` (motion
   subspace derivative applied to actual joint acceleration).
3. These additional terms accumulate through the tree to produce the
   `(∂M/∂q)·qacc` effect.

**Precondition:** `mjd_rne_pos` requires `data.qacc` to be populated
(forward acceleration must have run). This is always true at the point
where `mjd_smooth_pos` would be called in the hybrid path (after the
forward pass).

**Magnitude:** For a system at rest (`qacc ≈ 0`), the term vanishes and
bias-only derivatives are sufficient. For dynamic motion (significant qacc),
the term can be substantial — proportional to `||qacc|| · ||∂M/∂q||`.

### EGT-6: RNE velocity derivative structure (template for position)

`mjd_rne_vel` (derivatives.rs:780–1000) uses a three-phase algorithm:
1. **Forward pass** (root to leaves): Propagate `Dcvel[b]` = `∂cvel/∂qvel` (6×nv per body). Direct joint velocity contribution + chain rule through parent.
2. **Backward pass** (leaves to root):
   - Phase 1: Compute `Dcfrc[b]` = `I·Dcacc + crossForce_vel(I·v)·Dcvel + crossForce_frc(v)·I·Dcvel`
   - Phase 2: Accumulate `Dcfrc[parent] += Dcfrc[child]`
3. **Projection**: `qDeriv[dof,:] -= S^T · Dcfrc[body]`

For position derivatives, the structure is similar but with critical differences:
- Forward pass propagates `∂cvel/∂qpos` AND `∂cacc/∂qpos`. Unlike velocity
  derivatives (where `Dcacc[world] = 0` because gravity is velocity-independent),
  the position forward pass must use the **actual body accelerations** `cacc_full`
  (including qacc) — not just the bias acceleration. This is because
  `∂(X_b · cacc_full[parent])/∂q = (∂X_b/∂q) · cacc_full[parent] + X_b · Dcacc_pos[parent]`
  requires the FULL parent acceleration to capture the `(∂M/∂q)·qacc` effect.
  See EGT-10 for the mathematical justification.
- The position derivative of `cvel` involves `∂(R(q)·v_body)/∂q` terms.
- Additional term: `∂S_b/∂q · qacc_b` (motion subspace position dependence
  applied to actual joint accelerations) — absent in velocity derivatives.
- Backward pass accumulates position-dependent force derivatives.
- Gravity torques (which are zero in the velocity derivative) contribute
  significantly in the position derivative.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion, split mandate)*

> Spec accurately describes the conformance boundary. The conformance subset
> (transition A matrix position columns match FD/MuJoCo) is rigorously
> validated. The extension subset (analytical `mjd_smooth_pos` computation)
> is explicitly documented as a CortenForge extension with a clear boundary
> statement.

| Grade | Bar |
|-------|-----|
| **A+** | Spec explicitly states that `mjd_smooth_pos` does NOT exist in MuJoCo (citing `engine_derivative.c`, `engine_derivative.h`, `mujoco.h`). Split mandate clearly delineated: (1) conformance = final transition A matrix position columns match FD within tolerance (which matches MuJoCo's FD output), (2) extension = analytical computation itself. Each component (passive, actuator, RNE position derivatives) is mathematically derived from first principles (rigid-body chain rules) rather than ported from nonexistent MuJoCo C code. MuJoCo's FD approach cited as the validation oracle — not as the algorithm to port. The spec acknowledges that MuJoCo's velocity derivative functions (`mjd_passive_vel`, `mjd_actuator_vel`, `mjd_rne_vel`) serve as structural templates but the position math differs fundamentally (velocity derivatives are generally simpler because `∂cvel/∂qvel` is the motion subspace S, while `∂cvel/∂qpos` involves transform derivatives). **Constraint force exclusion** documented: analytical columns compute `∂qfrc_smooth/∂qpos` only, excluding `∂qfrc_constraint/∂qpos` — same approximation as existing analytical velocity columns (per EGT-9). For contact-free models, analytical matches FD exactly; for models with contacts, the approximation error is bounded and consistent with velocity-column precedent. **Deferred component strategy** explicit: which passive sub-components (fluid, gravcomp, flex bending, actuator moment-arm cross-terms) are deferred vs implemented analytically, what the conformance impact is for models with those features, and how the hybrid path handles it (selective FD fallback or documented tolerance relaxation). Edge cases addressed: world body (body_id == 0, no parent), zero-mass bodies (zero gravity contribution), disabled gravity (`DISABLE_GRAVITY`), disabled springs (`DISABLE_SPRING`), sleeping bodies, `nv=0`, single-joint vs multi-joint bodies, Free joint (absolute position), models with fluid forces, models with gravity compensation. |
| **A** | Split mandate present. Mathematical derivation correct. Minor gaps in edge cases or MuJoCo non-existence citation. |
| **B** | Describes `mjd_smooth_pos` as if it exists in MuJoCo, or conflates the extension with conformance. Mathematical derivation correct but MuJoCo boundary unclear. |
| **C** | Treats Spec A as a MuJoCo port. Does not cite that the function doesn't exist. |

> **Boundary between P1 and P9:** P1 grades whether the spec correctly
> identifies the MuJoCo conformance boundary (split mandate) and edge cases.
> P9 (Chain-Rule Correctness) grades whether the mathematical derivation of
> each analytical component is rigorous and complete.

### P2. Algorithm Completeness

> Every algorithmic step for each of the three position derivative components
> (passive, actuator, RNE) is specified unambiguously. The chain-rule
> structure through the kinematic tree is fully worked out. Per-joint-type
> differentiation formulas are explicit.

| Grade | Bar |
|-------|-----|
| **A+** | Every per-joint-type formula written in Rust pseudocode: Hinge (`∂R(θ,axis)/∂θ`), Slide (trivial), Ball (`∂R(q)/∂q` tangent-space), Free (translation direct + Ball rotation). RNE position derivative forward/backward pass fully specified — each term of `∂(I·a_bias + v ×* I·v)/∂qpos` worked out. Passive spring derivatives per joint type explicit (including Ball/Free quaternion spring via `mjd_sub_quat` Jacobian from DT-52). Actuator position derivatives via `∂force/∂L · moment^T` for each gain/bias type. Tendon spring position derivatives via `J^T · k · J`. Assembly function combining all three components. Integration into hybrid path: chain-rule formula `∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq` with `dvdq` derived per integrator type. An implementer can type it in without reading any source. |
| **A** | Algorithm is complete. One or two minor details left implicit (e.g., exact tendon cross-term). |
| **B** | Algorithm structure clear but some chain-rule terms hand-waved (e.g., "differentiate the FK" without showing the per-joint formulas). |
| **C** | Skeleton only — "compute FK position derivatives somehow." |

### P3. Convention Awareness

> Spec explicitly addresses tangent-space conventions for position
> perturbations, SpatialVector layout, quaternion conventions, and the
> critical difference between coordinate-space (nq) and tangent-space (nv)
> position derivatives.

| Grade | Bar |
|-------|-----|
| **A+** | Tangent-space mandate explicit: `qDeriv_pos` is nv×nv (tangent), not nq×nq (coordinate). Position perturbation convention matches `mj_integrate_pos_explicit` (tangent → coordinate mapping). Ball/Free quaternion derivatives projected to 3D/6D tangent space. Convention difference table present: CortenForge `SpatialVector` = `[angular; linear]` vs MuJoCo `[rotation; translation]` — same layout, different naming. `qDeriv_pos` storage mirrors `qDeriv` (dense nv×nv, same index convention). `xquat` quaternion convention (w,x,y,z) documented. `subquat` returns body-frame 3-vector (angular difference). Each porting rule verified to preserve numerical equivalence with FD. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted (e.g., tangent space) but quaternion derivative projection or SpatialVector layout not addressed. |
| **C** | Derivatives written in coordinate space or without addressing tangent mapping. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete models,
> tolerances, and expected value sources. The gold standard: every runtime AC
> asserts that analytical position columns match FD position columns within
> tolerance.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete input model (e.g., 3-link pendulum with hinge joints, humanoid with Ball/Free joints), (2) tolerance (`1e-6` relative for analytical-vs-FD), (3) what to check (`qDeriv_pos` entries, or transition A matrix position columns). At least one AC per joint type (Hinge, Slide, Ball, Free). At least one AC with mixed joint types. At least one AC for tendon spring position derivatives. At least one AC for position-dependent actuators. Performance AC: analytical ≥1.5× faster than FD position columns (wall-clock timing). Code-review ACs: `dqpos_dqpos` no longer `dead_code`, `qDeriv_pos` field exists on Data, FD position loop removed from hybrid path. |
| **A** | ACs are testable. Some lack specific model configurations or miss one joint type. |
| **B** | ACs directionally correct but vague tolerances or "should match FD." |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover all joint types, passive/actuator/RNE components, edge cases,
> and the integration into the hybrid transition path. At least one test
> validates the full transition A matrix (position columns) against pure FD.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: world body (no joints), zero-mass body, disabled gravity, disabled springs, sleeping bodies, `nv=0`, Free joint (absolute position), single-joint vs multi-joint body, tendon-only spring (no joint springs), flex edge springs (if implemented), ISD integrator (verifies correct position formula per EGT-7). Negative cases: `mjd_smooth_pos` on model with no position-dependent forces (result ≈ zero matrix). At least one multi-body test (3+ links with mixed joint types). **Contact isolation tests:** models with `DISABLE_CONTACT` (or no contact geoms) to validate smooth force derivatives in isolation — analytical should match FD exactly (modulo deferred passive sub-components). This provides the cleanest validation target. Models WITH contacts tested separately to characterize the constraint-exclusion approximation (per EGT-9). **Deferred component tests:** if any components are deferred (fluid, gravcomp, flex bending, moment-arm cross-terms), test that models using deferred features either (a) correctly fall back to FD, or (b) have documented accuracy degradation. Supplementary tests justified. Performance test (benchmark or timing assertion). FD convergence test: shrinking ε confirms analytical matches FD limit. |
| **A** | Good coverage. Minor edge-case gaps. Convergence test present. |
| **B** | Happy path covered. Edge cases sparse. No convergence test. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other Phase 11
> deliverables are explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Prerequisites: DT-52 (`mjd_sub_quat`) landed (needed for Ball/Free spring position derivatives). DT-53 (`forward_skip`) landed (hybrid path uses skip-stage, though position columns run full pipeline). DT-54 (muscle velocity derivatives) landed (no direct dependency but validates parallel structure). Commit hashes for T1 sessions cited. Cross-spec interaction: Spec B (sensor C/D) can use `qDeriv_pos` if available — soft dependency documented. Section ordering: passive_pos → actuator_pos → rne_pos → assembly → hybrid integration. |
| **A** | Order is clear. Minor prerequisite details left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (see EGT-3). Behavioral change: hybrid path position columns switch from FD to analytical — explicitly stated as conformance-neutral (same numerical result, different computation). Existing test impact: all hybrid transition tests should produce identical results (within FD tolerance) — specific test functions named. `IntegrationDerivatives.dqpos_dqpos` activation is a behavioral change (unused → used) — all existing consumers unaffected because it was always computed. New `Data.qDeriv_pos` field: `make_data()` must initialize to zeros(nv, nv), `reset()` must zero it, `Clone` must include it. Backward-compat: `DerivativeConfig` unchanged. Callers of `mjd_transition_hybrid` get same results. `mjd_smooth_pos` is additive (new function, no existing callers). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: `qDeriv_pos` used consistently (not `qDeriv_position` or `pos_deriv`). `mjd_smooth_pos` naming consistent with `mjd_smooth_vel` pattern. Cross-references accurate: AC numbers match traceability matrix, file paths match Files Affected, edge cases in MuJoCo Reference appear in Test Plan. Component names (passive, actuator, RNE) used consistently across sections. Chain-rule formula notation consistent (∂/∂qpos vs ∂/∂q — pick one and stick with it). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Chain-Rule Correctness *(domain-specific)*

> The analytical position derivative formulas are mathematically rigorous.
> Each chain-rule step through the kinematic tree is justified. Per-joint-type
> differentiation formulas are correct. The composition of passive + actuator
> + RNE components produces the correct total `∂qfrc_smooth/∂qpos`.

| Grade | Bar |
|-------|-----|
| **A+** | Each per-joint-type FK position derivative worked out with explicit formula: Hinge rotation matrix derivative `dR/dθ = [axis]× · R(θ)`, Slide translation derivative (identity along axis), Ball quaternion derivative in tangent space (right Jacobian of SO(3)), Free joint decomposition into translation (identity) + rotation (Ball). **Motion subspace position dependence** (per EGT-8): `∂S/∂qpos` for hinge/slide joints through ancestor body orientation changes — explicitly handled in RNE forward pass. **RNE at actual acceleration** (per EGT-10): `mjd_rne_pos` differentiates `RNEA(q,v,qacc)` — NOT `RNEA(q,v,0)`. Forward pass uses full body accelerations (including qacc), not just bias. The `(∂M/∂q)·qacc` term (mass matrix position derivative applied to current acceleration) is captured through `(∂X_b/∂q)·cacc_full[parent]` and `(∂S_b/∂q)·qacc_b` in the recursive structure. This has NO velocity analogue (`∂M/∂v = 0`). RNE position derivatives: gravity torque derivative `∂(r×F)/∂q` where `r = COM − jnt_pos` depends on q through FK — derivative involves `∂COM/∂q` (the COM Jacobian) AND `∂axis_world/∂q` (hinge/slide axis rotation with parent body). Coriolis position derivatives: `∂(C·v)/∂q` involves `∂cinert/∂q` (spatial inertia world-frame rotation) and `∂cvel/∂q` (velocity transport through changed frames). Passive spring derivatives: Hinge/Slide diagonal stiffness (trivial), Ball/Free quaternion spring via `mjd_sub_quat` Jacobian (citing DT-52). **Actuator moment-arm cross-term:** `(∂moment/∂qpos) · force` addressed for each transmission type — zero for Joint/fixed-tendon, nonzero for site/body/slidercrank/spatial-tendon (decision to compute analytically or defer to FD). **Deferred component strategy** explicit: which components are deferred, which models are affected, how the hybrid path handles fallback. Each formula dimensionally checked (nv × nv output). Key insight documented: RNE position derivatives are significantly more complex than velocity derivatives because position changes rotate the entire kinematic tree, affecting body poses, inertias in world frame, motion subspaces, and velocity transforms — AND because the mass matrix itself depends on position (captured by evaluating RNEA at actual qacc). |
| **A** | Formulas correct. One or two derivation steps left implicit. |
| **B** | Formulas stated but derivation gaps (e.g., "differentiate the rotation" without showing how). |
| **C** | Formulas partially incorrect or dimensionally inconsistent. |

> **Boundary with P1:** P1 grades whether the spec correctly identifies that
> this is an extension (no MuJoCo source to match). P9 grades whether the
> extension's mathematical content is rigorous.

### P10. Performance Characterization *(domain-specific)*

> The spec quantifies the expected performance improvement and specifies how
> to measure it. The entire motivation for Spec A is eliminating FD position
> columns — if there's no speedup, the extension has no value.

| Grade | Bar |
|-------|-----|
| **A+** | Cost model explicit: FD position columns cost `nv` simulation steps per column (or `2·nv` for centered). Analytical: O(nbody · nv) matrix operations, no simulation steps. Expected speedup: ≥1.5× for hybrid path (position columns are roughly half the total cost; eliminating them saves ~50% minus the O(nbody·nv) analytical cost). Measurement method specified: wall-clock timing on a reference model (e.g., humanoid with nv ≈ 27). Performance AC with concrete threshold. Discussion of when analytical might NOT be faster (very small nv where FD overhead is minimal). |
| **A** | Speedup claimed with reasonable justification. Measurement method specified. |
| **B** | "Should be faster" without quantification or measurement plan. |
| **C** | No performance discussion. |

> **Boundary with P4:** P4 grades whether the performance AC is testable
> (concrete threshold, measurement method). P10 grades whether the cost
> model and expected improvement are well-reasoned.

### P11. Hybrid Integration Completeness *(domain-specific)*

> The spec fully specifies how analytical position columns integrate into
> `mjd_transition_hybrid()`. The chain rule through integration derivatives
> is complete. The `dqpos_dqpos` activation is correct. The per-integrator
> dispatch for `dvdq = solver(qDeriv_pos)` is specified for all supported
> integrator types.

| Grade | Bar |
|-------|-----|
| **A+** | Complete chain-rule formula: `∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq` and `∂v⁺/∂q = dvdq` where `dvdq` is specified per integrator: Euler (`h · M⁻¹ · qDeriv_pos`), **ISD** (`(M+hD+h²K)⁻¹ · h · qDeriv_pos` — per EGT-7, NO correction term needed; the ISD spring displacement `−h·K·δq` provides its own cancellation when differentiated, unlike velocity columns where `+h·D_diag` cancels damping), ImplicitFast (`h · (M−hD)⁻¹ · qDeriv_pos`), Implicit (`h · LU⁻¹ · qDeriv_pos`), RK4 (fallback to FD — explicitly stated). **ISD formula derivation** from the implicit update equation is required — must differentiate `(M+hD+h²K)·v⁺ = M·v + h·f_ext − h·K·δq` w.r.t. `q` explicitly. The derivation must NOT use velocity-column analogy (the position and velocity formulas are structurally different). Activation row position columns: `∂act⁺/∂q = 0` (activation doesn't depend on position) — explicitly stated. FD position loop (lines 1448–1511) replaced entirely — no remaining FD for position columns in the hybrid path (except for models with deferred components, if selective fallback is chosen). Muscle activation columns: still use FD (lines 1513–1553) — NOT affected by Spec A. `dqpos_dqpos` activation: remove `#[allow(dead_code)]`, verify per-joint-type values (Hinge/Slide = 1.0, Ball/Free = `exp(−[ω·h]×)` from `mjd_quat_integrate`). |
| **A** | Chain rule complete. Per-integrator dispatch present. Minor integration detail implicit. |
| **B** | Chain rule stated but per-integrator formulas incomplete or `dqpos_dqpos` activation not addressed. |
| **C** | Integration into hybrid path not specified. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions, formulas, line
      numbers, and tolerances. Two independent reviewers would agree on the
      grade. P1 names `engine_derivative.c`, `mujoco.h`. P9 names per-joint
      formulas. P11 names per-integrator dispatch.

- [x] **Non-overlap:** P1 (MuJoCo boundary / split mandate) vs P9 (math
      correctness): P1 grades whether the spec GOT the conformance boundary
      right; P9 grades whether the analytical math is correct. P4
      (AC testability) vs P10 (performance model): P4 grades whether
      performance ACs are concrete; P10 grades whether the cost model is
      sound. P2 (algorithm completeness) vs P11 (hybrid integration): P2
      covers the three analytical components; P11 covers the chain-rule
      assembly into the transition matrix.

- [x] **Completeness:** 11 criteria cover: MuJoCo boundary (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), math correctness (P9), performance
      (P10), hybrid integration (P11). All dimensions of the task covered.

- [x] **Gradeability:** P1 → MuJoCo Reference + split mandate statement.
      P2 → Specification sections S1–S5. P3 → Convention Notes. P4 →
      Acceptance Criteria. P5 → Test Plan + Traceability Matrix. P6 →
      Prerequisites + Execution Order. P7 → Files Affected + Behavioral
      Changes. P8 → cross-cutting. P9 → Specification algorithm formulas.
      P10 → Performance section. P11 → Hybrid integration section.

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo source
      file names and the critical finding that `mjd_smooth_pos` does not
      exist. Split mandate clearly delineated. P4 requires FD-validated
      expected values. P5 requires FD convergence tests. The rubric cannot
      produce an A+ spec that diverges from FD (= MuJoCo) behavior.

- [x] **Empirical grounding:** EGT-1 through EGT-10 filled with verified
      MuJoCo source analysis, codebase context, and analytical derivations.
      Every A+ bar referencing MuJoCo behavior has a corresponding EGT entry.
      EGT-7 verified with 1-DOF analytical example. EGT-9/10 grounded in
      existing codebase patterns. No criterion bar written from header-file
      assumptions.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Split Mandate statement, Key Behaviors table |
| P2 | Specification (S1: passive_pos, S2: actuator_pos, S3: rne_pos, S4: assembly, S5: hybrid integration) |
| P3 | Convention Notes, Specification code (tangent-space handling) |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification algorithm formulas (per-joint differentiation, chain-rule derivations) |
| P10 | Performance section, cost model, measurement method |
| P11 | Specification S5 (hybrid integration), chain-rule formula, per-integrator dispatch |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | — | *Graded against spec (not yet written)* |
| P2. Algorithm Completeness | — | |
| P3. Convention Awareness | — | |
| P4. Acceptance Criteria Rigor | — | |
| P5. Test Plan Coverage | — | |
| P6. Dependency Clarity | — | |
| P7. Blast Radius & Risk | — | |
| P8. Internal Consistency | — | |
| P9. Chain-Rule Correctness | — | |
| P10. Performance Characterization | — | |
| P11. Hybrid Integration Completeness | — | |

**Overall: — (Rev 3, stress-tested ×2)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Umbrella spec claimed `mjd_smooth_pos` exists in MuJoCo `engine_derivative.c`. Verified against MuJoCo source: function does NOT exist. All position columns use FD via `mjd_stepFD`. | Rubric Phase 1 (MuJoCo source verification) | Reclassified Spec A as CortenForge extension with split mandate. P1 bar rewritten for split mandate. Scope Adjustment section added. | Rubric Rev 1 |
| R2 | P2 | Original umbrella described only three components (FK, RNE, passive). Actuator position derivatives (`∂force/∂L · moment^T`) were missing — position-dependent gains (Affine, Muscle FL curve) contribute to `∂qfrc_smooth/∂qpos`. | Rubric Phase 1 (codebase research) | Added actuator position derivatives as fourth algorithmic component. Updated scope. | Rubric Rev 1 |
| R3 | P9 | Need domain-specific criterion for chain-rule mathematical rigor. Standard P2 (algorithm completeness) doesn't distinguish "steps are present" from "chain-rule derivation is mathematically correct." | Rubric self-audit (completeness check) | Added P9 (Chain-Rule Correctness) with per-joint-type formula requirements and dimensional checks. | Rubric Rev 1 |
| R4 | P10 | Need domain-specific criterion for performance. Entire motivation is speedup — standard criteria don't cover this. | Rubric self-audit (completeness check) | Added P10 (Performance Characterization) with cost model and measurement requirements. | Rubric Rev 1 |
| R5 | P11 | Need domain-specific criterion for hybrid integration. The chain rule `∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq` and per-integrator `dvdq` formulas are the most integration-sensitive part. Standard P2 and P6 don't capture this depth. | Rubric self-audit (completeness check) | Added P11 (Hybrid Integration Completeness) with per-integrator dispatch and `dqpos_dqpos` activation requirements. | Rubric Rev 1 |
| R6 | P1 | EGT-1 needed: must document the MuJoCo source verification empirically. Without EGT, P1 bar references "MuJoCo doesn't have this" without evidence. | Rubric self-audit (empirical grounding check) | Added EGT-1 with specific source files examined and conclusions. | Rubric Rev 1 |
| R7 | P5 | Edge case list needed tailoring. Generic "nv=0" is relevant but "nu=0" is irrelevant (no actuators doesn't affect position derivatives significantly). Added domain-specific edges: world body, disabled gravity, disabled springs, sleeping bodies, Free joint absolute position. | Rubric self-audit (specificity check) | P5 A+ bar updated with domain-specific edge case inventory. | Rubric Rev 1 |
| R8 | P11 | ~~ISD integrator position column formula omitted the `+h²·K_diag` correction term.~~ **SUPERSEDED by R16:** The velocity-column analogy was incorrect. The ISD position column needs NO correction — see R16 for the correct derivation. | Stress-test audit (ISD integrator analysis) | ~~Added EGT-7 with +h²·K_diag correction~~ → **Corrected in R16.** EGT-7 rewritten with correct derivation showing no correction needed. | Rubric Rev 2 → Rev 3 |
| R9 | P9 | Actuator moment-arm cross-term `(∂moment/∂qpos) · force` missing for site/body/slidercrank/spatial-tendon transmissions. The `moment · (∂force/∂L) · moment^T` formula captures only the force chain-rule term. The geometry change term (`∂moment/∂qpos · force`) is separate and nonzero for position-dependent moment arms. | Stress-test audit (actuator transmission analysis) | Updated EGT-2 Component 2 to document the cross-term per transmission type. Updated P9 A+ bar to require the cross-term be addressed. | Rubric Rev 2 |
| R10 | P9/P2 | Flex force position derivatives not mentioned. Flex edge springs and bending forces contribute to `qfrc_spring` and have position-dependent derivatives. Free-vertex flex springs follow the same `J^T · k · J` pattern as tendon springs. Bending forces (cotangent, Bridson) are more complex. | Stress-test audit (passive force inventory) | Added flex forces to EGT-2 Component 1. | Rubric Rev 2 |
| R11 | P1/P4/P5 | Deferred components (fluid, gravcomp, flex bending, moment-arm cross-terms) cause analytical-vs-FD mismatch for models with those features. The rubric said "may be deferred" without addressing the conformance consequence. If the hybrid path uses analytical position columns that omit these terms, the transition A matrix will differ from FD (and thus from MuJoCo) on affected models. | Stress-test audit (conformance impact analysis) | Added deferred-component warning to EGT-2. Updated P1, P5, P9 A+ bars to require a deferred-component strategy (implement, selective FD fallback, or documented tolerance). | Rubric Rev 2 |
| R12 | P9 | Motion subspace S position dependence not explicitly called out. For hinge/slide joints, `S` depends on body orientation through ancestor joints. `∂S/∂qpos` is nonzero for ancestor perturbations and affects both the RNE forward pass and the projection step. This is the key complexity that makes position derivatives harder than velocity derivatives. | Stress-test audit (RNE derivative analysis) | Added EGT-8 documenting motion subspace position dependence. Updated P9 A+ bar to require `∂S/∂qpos` handling. | Rubric Rev 2 |
| R13 | P1/P5 | Constraint force position derivatives (`∂qfrc_constraint/∂qpos`) excluded from analytical computation but rubric didn't acknowledge this. FD position columns capture constraint response via full `mj_stepSkip(mjSTAGE_NONE)` re-computation. Analytical columns only compute `∂qfrc_smooth/∂qpos`. For models with contacts/constraints, this is the same approximation as existing analytical velocity columns (which exclude `∂qfrc_constraint/∂qvel`). | Stress-test round 2 (transition derivative analysis) | Added EGT-9 documenting constraint exclusion as established pattern. Updated P1 A+ bar to require constraint-exclusion documentation. Updated P5 A+ bar to require `DISABLE_CONTACT` isolation tests. | Rubric Rev 3 |
| R14 | P9 | `mjd_rne_pos` must differentiate RNEA at actual acceleration `qacc`, not at zero (bias only). The mass matrix `M(q)` depends on position, producing a `(∂M/∂q)·qacc` term in the transition derivative that has NO velocity analogue (`∂M/∂v = 0`). EGT-2 Component 3 described only bias force derivatives (gravity, Coriolis, gyroscopic) without this term. In the RNEA recursive structure, the fix manifests as: (a) forward pass uses `cacc_full` (including qacc) not `cacc_bias`, (b) new `∂S_b/∂q · qacc_b` terms appear. For systems at rest the term vanishes; for dynamic motion it can be substantial. | Stress-test round 2 (RNEA derivative analysis against velocity derivative code) | Added EGT-10. Updated EGT-2 Component 3 heading and content. Updated EGT-6 position derivative description. Updated P9 A+ bar to require RNEA at actual acceleration. Updated scope item 1 to describe `qDeriv_pos` content precisely. | Rubric Rev 3 |
| R15 | P5 | No test isolation strategy for clean validation. Contact-free models provide the cleanest validation target where analytical should match FD exactly (modulo deferred passive sub-components). Models with contacts should be tested separately to characterize the constraint-exclusion approximation. | Stress-test round 2 (test strategy analysis) | Updated P5 A+ bar to require `DISABLE_CONTACT` isolation tests and separate contact model characterization. | Rubric Rev 3 |
| R16 | P11/EGT-7 | **ISD position column formula was WRONG.** EGT-7 claimed `dvdq = (M+hD+h²K)⁻¹ · (h·qDeriv_pos + h²·K_diag)` by analogy with velocity columns (`+h·D_diag`). Analytical verification with 1-DOF system (m=1, k=10, h=0.01) proved the `+h²·K_diag` correction produces incorrect results (−0.0988 vs correct −0.0998). The correct formula is `dvdq = (M+hD+h²K)⁻¹ · h·qDeriv_pos` — NO correction needed. The spring displacement term `−h·K·δq` in the ISD RHS provides its own cancellation when differentiated w.r.t. position. The velocity-column analogy fails because `∂(M·v)/∂v = M` has no position analogue, while `∂(−h·K·δq)/∂q = −h·K` cancels the spring stiffness. | Stress-test round 2 (1-DOF analytical verification) | Rewrote EGT-7 with correct derivation and 1-DOF proof. Updated EGT-5 ISD formula. Updated P11 A+ bar. Removed all references to `+h²·K_diag` correction. | Rubric Rev 3 |
