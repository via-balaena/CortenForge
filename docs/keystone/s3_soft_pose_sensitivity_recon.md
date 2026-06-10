# Keystone S3 — soft-pose sensitivity `∂x*/∂(plane pose)` — RECON

*Active recon, opened 2026-06-10. The keystone's deepest leaf: a real `sim-soft` change. Lift
the **explicit** coupled-step Jacobian (S2) to the **total** single-step derivative by adding the
missing **implicit** term — how the soft solve's converged equilibrium `x*` moves as the rigid
contact plane moves. Sibling of `recon.md` (forward) and `s2_differentiability_recon.md` (the
factor chain). `MISSION.md` calls the differentiable coupling "the genuinely open research
problem"; this is the term S2 deliberately deferred.*

> **Establish `∂x*/∂(plane pose)` — the contact-engaged soft equilibrium's sensitivity to the
> rigid plane pose — by extending the soft solver's IFT machinery (`∂x*/∂θ = −A⁻¹ ∂r/∂θ`, today
> LOAD-only) to a contact-RESIDUAL pose derivative `∂x*/∂pose = −A⁻¹ ∂r/∂pose`, reusing the same
> factored tangent `A` the forward Newton step converged with. FD-validated first (perturb the
> plane pose, RE-SOLVE the soft step, central-difference), then analytic against that oracle.**

---

## 1. Where the plane pose enters the soft solve

The dynamic backward-Euler residual (`backward_euler.rs:2143` `residual_into`,
`assemble_global_int_force:1825`) is

```text
    r(x; θ, pose) = (M/Δt²)·(x − x̂)  +  f_int(x)  −  f_ext(θ)        x̂ = x_prev + Δt·v_prev
    f_int(x) = f_elastic(x)  +  Σ_{active pairs} g_v(x; pose)
```

The **plane pose enters ONLY through the contact term** `g_v`. Today's autograd
(`NewtonStepVjp`, `newton_vjp.rs`) differentiates `x*` w.r.t. `θ` (the applied load `f_ext`) —
`∂r/∂θ = −∂f_ext/∂θ`, a `−1` on each loaded vertex's free DOF. The plane is baked into the
contact at solver construction (`PenaltyRigidContact::with_params(vec![plane], κ, d̂)`) and is
**not a tape input** — exactly the gap S3 closes.

Per active pair at soft vertex `v` (penalty contact, `penalty.rs:730` `per_pair_readout` /
`gradient`):

```text
    g_v = (dE/dsd)·n̂            scattered as +f_int          (ContactModel::gradient sign)
    sd  = p_v·n̂ − offset                                     (RigidPlane::signed_distance)
    hard-penalty active:  dE/dsd = −κ(d̂ − sd),   d²E/dsd² = κ
```

**Pose parameterization (the key identity).** The keystone moves the plane by
`height = xpos[body].z − clearance`, rebuilding `RigidPlane::new((0,0,−1), −height)` each step
(`coupling/src/lib.rs:158` `build_contact`). Raising `height` by `δ` = translating the rigid
primitive by `+δ·ẑ`. For ANY rigid SDF translated by `t`, `sd(p; t) = sd₀(p − t)`, so the pose
derivative is a pure normal projection — no per-shape math:

```text
    ∂sd/∂δ  (translate primitive along world û)  =  −∇sd·û  =  −n̂·û
```

For the keystone (`û = +ẑ`, plane `n̂ = (0,0,−1)`): `∂sd/∂δ = −n̂_z = +1`. (Matches the
hand-derivation: `sd = height − z`, `∂sd/∂height = +1`.)

**The contact-residual pose derivative.** The plane normal is constant for a plane
(`∂n̂/∂δ = 0`), so only the `dE/dsd` factor varies:

```text
    ∂g_v/∂δ = d²E/dsd² · (∂sd/∂δ) · n̂ = κ·(−n̂·û)·n̂        (active hard pair)
```

For the keystone: `κ·(+1)·(0,0,−1) = (0,0,−κ)` at each active vertex. So `(∂r/∂δ)_free` is a
sparse vector: `−κ` at each active free vertex's **z** free-DOF, zero elsewhere. This is the
direct structural analog of the load case's `(∂r/∂θ)_free = −1` at loaded z-DOFs — same shape,
different scalar, sourced from the active set instead of the BC loaded set.

> **Scope of `∂n̂/∂δ = 0`.** Exact for planes (the keystone). A curved primitive (sphere, mesh
> SDF) translated by `δ` has `∂n̂/∂δ ≠ 0`, adding a `(dE/dsd)·∂n̂/∂δ` term. The general
> primitive-translation identity `∂sd/∂δ = −n̂·û` still holds; the normal-curvature term is the
> documented deferral, parallel to the penalty active-set cap (IPC the eventual cure for both).

## 2. The IFT forward solve (reusing `A`)

At the converged step `r(x*; pose) = 0`, the implicit function theorem gives, with
`A = ∂r/∂x |_{x*}` (the SAME tangent the forward Newton step converged with — and that
`NewtonStepVjp` already re-factors at `x_final` via `factor_at_position:1024`):

```text
    ∂x*/∂δ = −A⁻¹ · (∂r/∂δ)                solve  A · w_free = −(∂r/∂δ)_free
```

`A` (`assemble_free_hessian_triplets:1894`) is `M/Δt² + K_elastic + K_contact`, symmetric for
NeoHookean — the SAME factor (`FactoredFreeTangent`, Llt happy-path / A2 Lu fallback) the forward
solve and the load-VJP use. S3 reuses it with a different RHS. `w_free` (length `n_free`)
scatters back to a full-DOF `∂x*/∂δ` (zeros on pinned/roller DOFs).

This is a **forward sensitivity** (one column, since pose is a scalar), not a reverse VJP — the
right primitive because the coupling consumes `∂x*/∂δ` forward to lift the explicit Jacobian
(§3). It is the dual of the load VJP through the same `A`; the same code path validates both.

## 3. Lifting the explicit Jacobian to the total single-step derivative

S2 shipped the **explicit** (fixed-soft-position) coupled-step factor. The TOTAL force
sensitivity adds the implicit re-equilibration:

```text
    d(force_z)/d(height) = ∂force_z/∂height|_x   +   (∂force_z/∂x)·(∂x*/∂height)
                           └─ S1: +κ·N_active ─┘     └────── S3 implicit ──────┘
```

`∂force/∂x` for the penalty force `force_on_soft = κ(d̂−sd)·n̂` (active) is
`∂force_z/∂z_v = −κ·n̂_z² = −κ` per active vertex — as a soft vertex rises, `sd` grows, force
falls. So the implicit term is `Σ_active (−κ)·(∂x*/∂height)[v.z]`, and S3 closes the loop the S2
gate explicitly flagged as the open term ("the implicit re-equilibration is S3").

## 4. #1 RISK and the spike

**#1 RISK — does reusing `A` at `x_final` reproduce the re-solve FD for the DYNAMIC single
backward-Euler step, in the engaged regime?** Three sub-risks: **(a) active-set stability** — the
FD perturbation must not flip any pair (differentiate deeply engaged, the documented R3 cap);
**(b) Newton must converge tightly** so `x*` is a true root and the IFT premise holds (coupling
uses `tol = 1e-10` skeleton — fine); **(c) the curvature term** `∂n̂/∂δ` — zero for the plane,
non-zero for curved primitives (scope to planes).

**SPIKE (throwaway, `#[ignore]`, sim-soft)** — mirrors `sim-core`'s `validate_analytical_vs_fd`
discipline. At a deeply-engaged block-on-plane config:
1. Build the solver, `replay_step` → `x*(offset)`.
2. Perturb the plane offset by `±ε` (rebuild contact + solver), `replay_step` → `x*(offset±ε)`;
   central-difference per DOF → **FD `∂x*/∂δ`**. Two `ε`, confirm they agree (smooth) and the
   active-set count is stable across the perturbation.
3. Analytic `−A⁻¹·(∂r/∂δ)` (RHS `−κ·n̂` per active vertex, factor at `x*`); confirm it matches the
   FD oracle to ~1e-6 rel.

Passing retires R1 and yields the oracle the production gate reuses.

**SPIKE DONE 2026-06-10 (throwaway, deleted) — ★ R1 RETIRED.** A `#[ignore]`'d unit test in
`backward_euler.rs` (private access to `factor_at_position` + the free-DOF map), keystone params
(`n_per_edge=4`, `edge=0.1`, `μ=3e4`, `κ=3e4`, `d̂=1e-2`, `dt=1e-3`), settled 400 dynamic steps
to a deeply-engaged near-equilibrium (plane penetrating the top face ~1 mm), then on one more
dynamic step compared analytic `−A⁻¹·(∂r/∂h)` (RHS `κ·(∂sd/∂h)·n̂` per active pair, factor at
`x_final`) to a re-solve central FD:

| quantity | value |
|---|---|
| active set | **25** (5×5 top face), **stable** across `+ε`/`−ε` (25/25) |
| FD two-ε agreement (1e-6 vs 5e-7) | **7.4e-11** (smooth, converged) |
| **analytic vs re-solve FD (rel)** | **9.4e-11** (FD/Newton-tol-limited) |
| `‖∂x*/∂h‖_∞` | 0.94 (top vertices track the rising plane ~1:1 — physical) |

So reusing the converged tangent `A` at `x_final` reproduces the re-solve sensitivity to the
FD/Newton-tolerance floor (the oracle's inner Newton converges to `tol = 1e-10` and the central
difference divides that by `2ε ≈ 2e-6` — not machine epsilon), the active set is stable in the
engaged regime, and the step is smoothly differentiable. The math (§1–2) is confirmed; the
analytic path is ready to productionize.

## 5. Decisions (head-engineer)

- **D1 — Forward sensitivity, not a reverse VJP.** Pose is a scalar; the coupling consumes
  `∂x*/∂δ` forward (§3). It is the dual of the load VJP through the same factored `A`. The
  independent numeric check is the **nonlinear re-solve FD** (§ PR1): the oracle re-runs the full
  Newton solve at `height±ε` with NO access to the analytic `A⁻¹` path, so its agreement is a
  genuine cross-check (not an affine identity like S2's gate). *(Earlier plan: also cross-check the
  same solve on the LOAD parameter against `NewtonStepVjp` — dropped as redundant once the
  nonlinear re-solve FD was confirmed to be the stronger, fully independent guard.)*
- **D2 — Primitive-translation pose, generic over the SDF; normal-curvature term scoped out.**
  `∂sd/∂δ = −n̂·û` is exact for any rigid primitive; `∂n̂/∂δ = 0` is exact for the plane (the
  keystone) and the documented deferral for curved primitives. The API takes a world unit
  direction `û` so the keystone passes `+ẑ` and the math reads generally.
- **D3 — Lives in `sim-soft`, on `CpuNewtonSolver`.** The method needs `A` factored at `x_final`
  (`factor_at_position`), the free-DOF map, and the active set + `κ`/normals — all crate-private
  to `backward_euler.rs`. The coupling holds a concrete
  `PenaltyRigidContactSolver<HandBuiltTetMesh>` and calls the public method. The pose-residual
  derivative is sourced via a small contact hook (default-empty `ContactModel` extension,
  implemented for `PenaltyRigidContact`) so the solver method stays generic.
- **D4 — Differentiate in the contact-ENGAGED regime; FD oracle gates analytic.** Same posture as
  S0–S2 (R3 cap documented, IPC deferred).

## 6. Sub-leaf ladder / slicing

- **Spike (THROWAWAY, `#[ignore]`).** §4 — retire R1, get the oracle.
- **PR1 (S3a) — sim-soft soft-pose sensitivity primitive.** `CpuNewtonSolver` method returning
  `∂x*/∂δ` for translating the contact primitive(s) along a world `û`, reusing `A` at `x_final`;
  contact pose-residual-derivative hook on `PenaltyRigidContact`. Gates (as shipped in
  `tests/soft_pose_sensitivity.rs`): (i) analytic `∂x*/∂δ` vs a **nonlinear re-solve** central FD
  at a deeply-engaged config — the oracle re-runs Newton at `height±ε` touching none of the `A⁻¹`
  machinery, so it independently guards against a self-consistent-but-wrong IFT solve; (ii) a
  `NullContact` zero-sensitivity guard (the default-empty trait hook). grade A.
- **PR2 (S3b) — lift to the total in sim-coupling.** Call the primitive for `∂x*/∂height`,
  assemble the total `d force/d height` (§3), FD-gate the TOTAL against a black-box re-solve
  (rebuild coupling, re-solve the single soft step at `height±ε`, recompute force). grade A.

Each leaf: n+1 cold-read + pre-PR local ultra-review; no push/PR without go-ahead.

## Progress · PR1 (S3a) — sim-soft soft-pose sensitivity primitive DONE (2026-06-10)

The IFT forward-sensitivity primitive, in `sim-soft`:
- `ContactModel::pose_residual_derivative(pair, positions, dir)` — default-empty
  trait method (pose-independent contacts contribute nothing); overridden by
  `PenaltyRigidContact`: `∂r_v/∂δ = d²E/dsd²·(−n̂·dir)·n̂` (translate the rigid SDF
  by `δ·dir` ⇒ `∂sd/∂δ = −∇sd·dir`).
- `CpuNewtonSolver::equilibrium_pose_sensitivity(x_final, dt, dir) → ∂x*/∂δ` —
  gathers `(∂r/∂δ)` from the active set, solves `A·w = −(∂r/∂δ)_free` reusing the
  tangent factored at `x_final` (`factor_at_position`, the same factor the load
  adjoint reuses), scatters to full-DOF. Generic over the contact model; all-zeros
  for `NullContact`.
- **Committed gate `tests/soft_pose_sensitivity.rs`:** at a settled deeply-engaged
  block (25 active, stable across `±ε`), analytic `∂x*/∂h` matches a **nonlinear
  re-solve** central FD to **rel 9.4e-11** (the oracle re-runs Newton at `height±ε`
  — independent of the `A⁻¹` path, so this validates the IFT adjoint itself, not an
  affine identity); plus a `NullContact` zero-sensitivity guard. grade **A**.

## Progress · PR2 (S3b) — total single-step force Jacobian (lifted) DONE (2026-06-10)

In `sim-coupling`, the explicit coupled-step factor is lifted to the total:
- `StaggeredCoupling::resolved_contact_force(height)` — force on the soft body after
  one *re-equilibrated* dynamic soft step (the black-box oracle).
- `StaggeredCoupling::contact_force_height_total_jacobian(height)` —
  `d force/d h = ∂force/∂h|_x (S1 explicit) + (∂force/∂x = −κ·n̂⊗n̂)·(∂x*/∂h)` (S3
  implicit via `equilibrium_pose_sensitivity`).
- **Committed gate `tests/coupled_total_jacobian.rs`:** at the deeply-engaged
  `h=0.099`, the explicit-only S1 factor is **7.5e5** N/m but the **total = 1.62e5**
  N/m — the implicit re-equilibration **cancels ~78%** ("the soft body follows the
  rising plane, so `sd`—hence the force—barely changes"), and the analytic total
  matches the **nonlinear re-solve FD to rel 6.3e-11**. grade **A** (Coverage A+,
  lib smoke test added so the `--lib` coverage tier counts the probes).

★ S3 = the implicit soft-re-equilibration term is CLOSED + machine-validated; the
explicit coupled-step Jacobian is lifted to the total single-step derivative.
**NEXT (beyond S3):** the full soft-tape `VjpOp` crossing (one `tape.backward`
across both engines) + the material-parameter VJP.

## 7. Risks

- **R1 (#1) — IFT-vs-resolve mismatch / active-set instability / convergence.** §4 spike measures
  it directly before any production code.
- **R2 — generic-`C` plumbing.** The pose-residual derivative is plane-specific but the solver is
  generic. MITIGATION: default-empty `ContactModel` extension method, implemented for
  `PenaltyRigidContact`; solver method stays generic, bit-equal on non-pose contacts.
- **R3 — `Data`/factor reuse footguns.** `factor_at_position` re-factors fresh at `x_final` (no
  stash to invalidate); the method is a pure read of `(x_final, dt, active set)` like
  `rigid_step_probe` is a pure rigid probe.
- **R4 — curved-primitive normal curvature** (`∂n̂/∂δ ≠ 0`). Scoped out (planes only); documented.

## 8. Validation gate (S3)

CI-runnable: (PR1) an analytic `∂x*/∂(plane pose)` from the soft solver agrees with a nonlinear
re-solve central FD to tolerance at a deeply-engaged config (plus a `NullContact` zero-sensitivity
guard for the default trait hook); (PR2) the analytically-assembled TOTAL `d force/d height`
agrees with a black-box re-solve FD. Honest scope: smooth/engaged regime,
plane primitive (`∂n̂/∂δ = 0`); curved-primitive curvature + the full soft-tape `VjpOp` crossing
(one `tape.backward` across both engines) + the material-parameter VJP are the leaves beyond S3.

## Key files / pointers

- Soft residual + tangent + factor: `sim/L0/soft/src/solver/backward_euler.rs`
  (`assemble_global_int_force:1825`, `assemble_free_hessian_triplets:1894`,
  `factor_at_position:1024`, `residual_into:2143`, `replay_step:2024`).
- Soft autograd (load-only IFT): `sim/L0/soft/src/differentiable/newton_vjp.rs`.
- Contact: `sim/L0/soft/src/contact/penalty.rs` (`pair_contribution`, `per_pair_readout`,
  `gradient`/`hessian`); `contact/rigid.rs` (`RigidPlane`, `sd = p·n − offset`);
  `contact/mod.rs` (`ContactModel`, `ContactPairReadout`, `ActivePairsFor`).
- Coupling: `sim/L1/coupling/src/lib.rs` (`build_contact`, `contact_readout`,
  `contact_force_height_jacobian` (S1), `rigid_step_probe` (S2), `step`).
- Engaged-contact fixture precedent: `sim/L0/soft/tests/penalty_compressive_block.rs`.
