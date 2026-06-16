# Tangential friction — recon + S0

*Keystone deepening. Written 2026-06-15. The differentiable soft↔rigid contact is
FRICTIONLESS — normal force only (`force_on_soft = −dE/dsd·n̂`). Friction is the one
remaining contact-physics CAPABILITY, and it's convergent: the exo applies shear/hold to
a limb, and Bubby's "warm hug" is a soft body GRIPPING a person — both grip-dominated, both
un-modelable without friction. This is the recon's named follow-on from the rotating-normal
leaf, and a deferral the codebase has flagged since the IPC contact (`null.rs`: "Phase C
adds … friction per spec §8").*

## 1. The model — Li 2020 smoothed Coulomb (spec is COMPLETE)

The model is fully worked out in `docs/studies/soft_body_architecture/src/40-contact/02-friction/`.
It is a **position-space dissipative potential** — a function of `x` (the within-step
tangential displacement), NOT of velocity — so it composes into the backward-Euler Newton
minimization as just another energy term (no integrator change). Per pair:

```
u_T(x) = (Tⁿ)ᵀ(x − xᵗ)                  tangential displacement (2-vec); Tⁿ (3×2) lagged tangent frame, xᵗ = step start
f₁(y)  = −(y/w)² + 2(y/w)  (y≤w), else 1  C¹ kernel; w = h·ε_v (transition width, ε_v ≈ 1e-3·L_bbox m/s)
f₀(y)  = w/3 − y³/(3w²) + y²/w (y≤w), else y   antiderivative (f₀'=f₁), C²; f₀(w)=w
D(x)   = μ_c · λⁿ · f₀(‖u_T‖)            the per-pair friction potential (∈ U_total as h²·Σ D_k)
∇_x D  = μ_c · λⁿ · f₁(‖u_T‖) · Tⁿ·(u_T/‖u_T‖)    friction force = −∇D
λⁿ     = −(κ/h²)·∂b/∂d_k                 lagged normal-force magnitude (from the IPC barrier derivative)
```

`λⁿ` (normal force) and `Tⁿ` (tangent basis) are **lagged** — held CONSTANT across the
within-assembly differentiation (only `u_T` is differentiated). PR1 takes the simplest
lagging that still converges to the correct equilibrium: recompute `(Tⁿ, λⁿ)` from the
current iterate's positions and freeze them for that assembly (so the grad/Hessian see
`λⁿ`/`Tⁿ` as constants). At the converged fixed point `x*` this is self-consistent
(`λⁿ = λⁿ(x*)`), so the forward equilibrium is independent of the lagging detail — a
prior-iterate freeze and this current-iterate freeze are both inexact-Newton variants
reaching the same residual-zero. That lagging is what makes `D` position-space and `C²`
(the inner `‖·‖` non-smoothness is absorbed: near the origin `f₀(‖u‖) ≈ w/3 + ‖u‖²/w`, a
smooth quadratic bowl). No cross-iterate cache is needed for PR1 — see §2.

### S0 (THROWAWAY) — force-law CONFIRMED
Standalone validation of `f₀`/`f₁`/`D`/`∇D`: `∇D` vs central FD rel ~1e-8–1e-10 across
stick/transition/slip; deep-slip force = exactly `μ·λ` (Coulomb cone), never exceeded;
near the origin the force ramps LINEARLY in slip (slope `μλ·2/w`, the smooth bowl — no kink).

## 2. Seam map (where friction plugs in)

- **`ContactModel` (sim-soft `contact/mod.rs`)** — `energy`/`gradient`/`hessian` are
  position-only (perfect: friction is position-space). Friction adds `D`/`∇D`/`∇²D`
  ALONGSIDE the barrier. NEW per-pair data: the lagged `(Tⁿ, λⁿ)` and `xᵗ`. `∇²D` couples
  the two tangent directions (a 3×3 block per pair, vs the barrier's rank-1 `n̂⊗n̂`).
- **Solver (`solver/backward_euler.rs`)** — `assemble_global_int_force` / `assemble_free_hessian_triplets`
  already scatter the contact `gradient`/`hessian`; friction scatters the SAME way once the
  term exists. `xᵗ = x_prev` and `h` are already in scope. PR1 needs NO new persistent
  infrastructure: `λⁿ = ‖contact.gradient‖` and `Tⁿ` are recomputed from the current iterate
  inside `friction_blocks` and frozen for that assembly (see §1). PR2 may formalize a
  per-solve freeze of `(Tⁿ, λⁿ)` if the adjoint needs the lag held at exactly `x*`.
- **Coupling (`sim-coupling/lib.rs`)** — `active_pair_wrench_data` / `contact_wrench`: the
  wrench already routes an arbitrary per-pair force (the off-COM moment handles any
  direction), so the TANGENTIAL reaction enters with no change to the wrench assembly — only
  `force_on_soft` extends from normal-only to `normal + friction`. `λⁿ` derives from the
  barrier curvature the coupling already reads.
- **Differentiability** — friction enters the system tangent `A` (the Hessian), so `∂x*/∂·`
  already flows through it once friction is in the Hessian; the pose-adjoint
  (`pose_residual_derivative` / `RigidTwist`) and the coupled wrench gain friction's
  pose/normal-force dependence (the lagged `λⁿ`/`Tⁿ` are constants per solve, so they are
  NOT differentiated through — the standard IPC-friction approximation; FD-gate validates it).

## 3. Sliced plan (mirrors the normal-contact arc: soft physics → differentiable → coupled)

1. **PR1 (sim-soft) — the friction energy term on a fixed rigid plane.** A `FrictionPair`
   contribution to the contact `energy`/`gradient`/`hessian`, lagged `(Tⁿ, λⁿ)` refreshed
   between Newton solves, `λⁿ` from the barrier derivative. Forward only (the solver
   minimizes elastic + barrier + friction). Gate: FD `∇D`/`∇²D`; the STICK/SLIP physics — a
   soft block under tangential load below `μλ` holds (sticks), above slides. The foundation.
2. **PR2 (sim-soft) — friction differentiability (the forward sensitivities).** ✅ **The
   `∂λⁿ/∂x` term makes the friction adjoint NON-SYMMETRIC** (friction is the first
   non-conservative force in the soft solve; the frozen-lag `∇²D` is its symmetric part, the
   `∂λⁿ/∂x` normal-force coupling its asymmetric rank-1-per-pair part). The keystone factor
   path is symmetric-only (Cholesky on lower-triangle triplets), so rather than tear into it
   we add a **Woodbury low-rank correction AROUND the existing symmetric factor**: the true
   adjoint `A = A_sym + Σₚ aₚ bₚᵀ` (`aₚ = ∇D/λⁿ`, `bₚ = n̂ᵀ∇²E_contact`), solved via
   `A_sym⁻¹` reuse + a `k×k` capacitance solve (`k` = active pairs). Folded into
   `FactoredFreeTangent` so every adjoint consumer is transparent; empty ⇒ bit-identical.
   `factor_at_position` gains `x_prev: Option` (`Some` ⇒ friction-exact, `None` ⇒
   frictionless / the guarded reverse path). **Gate: `∂x*/∂μ` (material) MACHINE-EXACT vs
   re-solve FD (rel ~5e-9; S0 measured the frozen-lag-only adjoint stuck at a systematic
   ~2e-3 — the Woodbury closes it to the FD floor).** The forward sensitivities
   (`equilibrium_material/pose/state`) take the exact adjoint; the reverse-mode tape VJPs
   stay friction-guarded (`x_prev = None` under friction → conditional panic) until PR3.
   **Pose deferred to PR3:** the adjoint `A` is identical for material and pose (so it is
   friction-exact for both), but the POSE *RHS* (`∂r/∂pose` = `assemble_pose_residual_grad`)
   also needs the friction force's explicit pose dependence (moving the plane changes `λⁿ`),
   which lands naturally with the coupling where the pose adjoint is consumed.
3. **PR3 (sim-coupling) — friction in the coupled gradient.** The tangential reaction in the
   wrench; the **pose-residual friction term** (`∂r/∂pose`, deferred from PR2) + the
   reverse-mode VJPs threaded with `x_prev`; the lagged `λⁿ` wired from the contact. FD-gate
   the coupled trajectory gradient on a scene with a tangential load (a tilted/sliding contact
   — where friction holds vs slips). This is where it serves the exo / Bubby.

Follow-on: stick-slip at the true Coulomb limit is already captured by the smooth transition
(the spec's whole point); the open knobs are `ε_v` (transition width) and per-pair `μ_c`.
