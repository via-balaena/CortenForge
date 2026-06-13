# Keystone — the multi-step time-adjoint — RECON

*Active recon, opened 2026-06-12. The trajectory S0 spike (`traj_differentiability_recon.md` §8)
returned the positive branch: the co-design gradient HOLDS across contact make/break, and the **only
real remaining gap is the multi-step adjoint assembly** — composing the validated per-step gradients
into one `dJ/dθ` across an N-step coupled rollout. `Differentiable::time_adjoint` is still
`unimplemented!("skeleton phase 2 — time-adjoint is Phase E+")` (`sim-soft/src/differentiable/`).
This leaf builds it. Sibling of `recon.md`, `s2..s5_*.md`, `traj_differentiability_recon.md`.*

> **Thread the per-step adjoint across the rollout.** Today every soft VJP (`NewtonStepVjp`,
> `MaterialStepVjp`) has a SINGLE parent (load θ / material p); the dependence of `x*_k` on the
> PREVIOUS state `(x_prev, v_prev)` is never on the tape — so one `tape.backward` cannot cross step
> boundaries. The missing primitive is `∂x*/∂x_prev` and `∂x*/∂v_prev`. Both come from the SAME IFT
> machinery and reuse the SAME factored tangent the forward Newton step already produced — only the
> RHS is new. Add that primitive, thread it across steps (and across the soft↔rigid interface, which
> the S1–S5 crossing already wires per step), and one reverse pass gives `dJ/dθ` over the whole
> trajectory. FD-validated against the full-coupled re-rollout oracle (the discipline S3/S4/S5 used).

---

## 1. The single new primitive: `∂x*/∂(x_prev, v_prev)`

The dynamic backward-Euler residual (`backward_euler.rs::residual_into`, line 2351) is

```text
    r(x; x_prev, v_prev, θ, p, dt) = (M/Δt²)·(x − x̂) + f_int(x; p) − f_ext(θ),
        x̂ = x_prev + Δt·v_prev   (predictor)
```

with **lumped diagonal mass** `mass_per_dof` (field, line 259). The previous state enters ONLY through
the predictor `x̂`, so

```text
    ∂r/∂x_prev = −(M/Δt²)·I       ∂r/∂v_prev = −(M/Δt)·I       (both diagonal)
```

At the converged step `r(x*) = 0`, the implicit function theorem gives, reusing the tangent
`A = ∂r/∂x|_{x*}` (the SAME factor the forward Newton, the load adjoint, S3 pose-sensitivity and S5
material-sensitivity all use — `factor_at_position(x_final, dt, ·)`):

```text
    ∂x*/∂x_prev = −A⁻¹·∂r/∂x_prev = +A⁻¹·(M/Δt²)      (an n×n matrix — never materialized)
    ∂x*/∂v_prev = −A⁻¹·∂r/∂v_prev = +A⁻¹·(M/Δt)
```

**Reverse-mode VJP (what the tape needs).** For upstream cotangent `g = ∂L/∂x*`, solve `A·λ = g_free`
ONCE (identical adjoint solve to `NewtonStepVjp`), then — because `∂r/∂x_prev` is the symmetric
diagonal `−(M/Δt²)` —

```text
    ∂L/∂x_prev[i] = +(M/Δt²)[i]·λ_full[i]        ∂L/∂v_prev[i] = +(M/Δt)[i]·λ_full[i]
```

(`λ_full` = λ scattered onto full DOFs, zero on pinned). So ONE factor-reuse solve yields BOTH the
x_prev and v_prev cotangents alongside the existing θ/p cotangent — a multi-parent VJP. This is the
whole new ingredient; everything else is composition.

## 2. Threading the soft side across steps

`v` is not an independent input — backward Euler sets `v_k = (x*_k − x*_{k−1})/Δt` (the velocity
update; `StaggeredCoupling::step` already does this, lib.rs:799). So on the tape:

```text
    x*_k = SoftStep(x_prev = x*_{k−1},  v_prev = v_{k−1},  p)     [StateStepVjp: parents x*_{k−1}, v_{k−1}, p]
    v_k  = (x*_k − x*_{k−1}) / Δt                                 [linear tape node: sub + scale]
```

The chassis tape supports `sub`/`scale`/`neg` (used already in the crossing), so `v_k` is a plain
linear node of two consecutive position vars. Chaining `x*_0 → x*_1 → … → x*_N` then a scalar
`J(x*_N)` and `tape.backward(J)` walks the cotangent back through every step to the leaf p — the soft
trajectory adjoint.

## 3. Threading the soft↔rigid interface across steps (the coupled trajectory)

Per step the S1–S5 crossing is already built: `x*_k →[ContactForceVjp] fz_k →[neg] xfrc_k
→[RigidStepVjp] vz'_k`, and the plane height for step k+1 is the rigid pose `xpos[body].z − clearance`.
For a multi-step COUPLED tape two things thread additionally:
- **rigid state** `s_k = (qpos, qvel)` evolves `s_{k+1} = f_rigid(s_k, xfrc_k)`; its Jacobians
  `∂s_{k+1}/∂s_k` (sim-core `transition_derivatives` `A`) and `∂s_{k+1}/∂xfrc_k` (the S2 finding: xfrc
  is NOT in `B`, so this is the FD/column-built `dt/m` free-body factor `RigidStepVjp` already wraps).
- **plane height** feeds the next soft solve's contact (hence its residual AND tangent A). The S3
  pose-sensitivity `∂x*/∂height` is exactly this cross-term and is already validated.

So the coupled trajectory adjoint composes FOUR already-validated per-step factors (soft state §1,
soft material/load S5/S4, contact-force S3/ContactForceVjp, rigid dt/m S2) plus the rigid-state
carry. No new research — wiring of validated pieces, which is why the trajectory spike rated this
low-risk.

## 4. Ladder (sliced, each FD-gated; the keystone leaf pattern)

- **S0 spike (this turn, throwaway):** validate the ONE new primitive `∂x*/∂x_prev`, `∂x*/∂v_prev`
  in `sim-soft`, in isolation, vs an independent re-solve FD — plus a 2–3-step soft-only composition
  check (chain the per-step state Jacobians, compare to a full multi-step FD). If the new primitive
  and its composition hold, the time-adjoint is assured; the rest is wiring. Prototype the forward
  sensitivity in place (the VJP needs crate-private `factor_at_position`/`mass_per_dof`/`free_dof_indices`,
  so the spike lives as a throwaway test + a temporary method inside `sim-soft`, reverted after).
- **PR1 (sim-soft):** `equilibrium_state_sensitivity` (forward `∂x*/∂x_prev`, `∂x*/∂v_prev`) +
  `StateStepVjp` (reverse, multi-parent: x_prev, v_prev, [θ|p]) mirroring `MaterialStepVjp`; FD gate
  (single-step ∂x*/∂x_prev vs re-solve; multi-parent VJP vs forward-sum). Implement the real
  `time_adjoint` (or document it superseded by `push_custom` like its siblings).
- **PR2 (sim-coupling):** `coupled_trajectory_*_gradient` — build the full coupled multi-step tape
  (soft state thread §2 + interface thread §3 + rigid-state carry), ONE `tape.backward` over N steps
  → `dJ/dθ` (or `dJ/dμ`), gated vs the full-nonlinear-coupled FD oracle (the trajectory spike's
  `dz_N/dμ ≈ 5.334e-9` is a ready reference value). Plug the real `time_adjoint` into the keystone's
  consumer surface.
- **(Deferred):** general objective functionals beyond `z_N`; the policy/RL trajectory objective in
  the co-design optimizer (`project-codesign-optimizer`).

## 5. Risks

- **R1 (low, primary spike target):** the state VJP math / sign / factor-reuse is wrong → the S0
  FD gate catches it immediately (independent re-solve, not an affine identity).
- **R2 (low):** the multi-step composition drifts (accumulating tangent error over N steps) → the
  spike's 2–3-step composition vs full FD measures it; the trajectory spike already showed the
  N-step objective is well-conditioned (FD-converged over 3 decades of ε).
- **R3 (medium, deferred to PR2):** the rigid-state carry (`∂s_{k+1}/∂s_k`) interacts with the
  xfrc-not-in-`B` gap — but the keystone free-body platen makes the rigid step axis-decoupled
  (`dt/m`), so the spike + PR1 sidestep it and PR2 inherits the S2 scope cap.
- **R4 (low):** active-set CHANGES between steps shift which contact DOFs are in A — but the
  trajectory spike measured this directly (worst rel-err 6.1e-9 ON active-set-change steps); the
  per-step factor A is rebuilt each step at that step's active set, so the threading is consistent.

## 6. VERDICT — S0 spike done 2026-06-12 (throwaway, reverted). **PRIMITIVE SOUND + COMPOSES.**

Throwaway `sim-soft` method `debug_state_cotangents` (the exact reverse-VJP contraction §1) + test
`tests/zzz_time_adjoint_spike.rs`, both reverted after. The state cotangents were gated against an
INDEPENDENT re-solve FD (the oracle re-runs the full nonlinear Newton at perturbed prev-state,
touching no A⁻¹ — not an affine identity):

- **Single converged step (`L = g·x*`):** analytic `∂L/∂x_prev` vs re-solve FD = **rel 9.3e-9**;
  `∂L/∂v_prev` = **rel 8.6e-7** (abs 7.5e-10 — at the FD noise floor; this term is `1/Δt` smaller).
- **Two-step composition (`L = g·x*₂`, chain step-2 → step-1 incl. the `v₁=(x₁−x₀)/Δt` linear node):**
  composed `dL/dx₀` vs full two-step re-rollout FD = **rel 1.8e-8.** The per-step state Jacobians
  chain correctly, including the velocity coupling between consecutive positions.

**Design fact surfaced (first FD run caught it):** the world-pinned base is held at the incoming
`x_prev` value (Dirichlet), so perturbing a PINNED `x_prev` DOF moves `x*` — but in the rollout the
base is constant (world-fixed), NOT part of the differentiable state thread. The VJP correctly returns
**0 cotangent on pinned DOFs** (λ scattered to free only); FD validation perturbs FREE DOFs. ⇒ **PR1
note:** `StateStepVjp`'s threaded parents are the FREE state thread; the pinned base is a constant
leaf, not a tape input. (The free-to-free block depends on `x_prev` only through the inertia diagonal —
`f_int` depends on the unknown `x`, not `x_prev` — so the `(M/Δt²)·λ` formula is COMPLETE for it.)

**DECISION:** the one genuinely-new primitive is correct and composes → **the time-adjoint is a
low-risk wiring build, as the trajectory spike predicted. PROCEED to PR1** (sim-soft
`equilibrium_state_sensitivity` forward + `StateStepVjp` reverse multi-parent + FD gate), then PR2
(sim-coupling coupled-trajectory `tape.backward` over N steps vs full-coupled FD). R1/R2 retired
(primitive + composition validated); R3 (rigid-state carry) deferred to PR2 as scoped.

## 7. PR1 + PR2 STATUS — built 2026-06-13 (branch `feat/keystone-traj-differentiability-spike`, NOT pushed)

**PR1 (sim-soft) ✅ grade A** — `equilibrium_state_sensitivity` (forward JVP) + `StateStepVjp`
(reverse, 2-parent x_prev/v_prev) + `state_step_vjp`. Gate `tests/state_sensitivity.rs`: forward JVP
vs re-solve FD (∂/∂x_prev rel 4.1e-10, ∂/∂v_prev 4.8e-8); reverse VJP on a tape vs forward-sum dual
(machine-exact) AND re-solve FD; pinned-base zero guard.

**PR2 (sim-soft + sim-coupling) ✅ grade A both** — sim-soft `TrajectoryStepVjp` (the unified
4-parent VJP fusing the prev-state, material, and contact-pose adjoints into ONE shared `A·λ=g_free`
solve) + `trajectory_step_vjp`. sim-coupling glue VJPs (`VelVjp`, `ContactForceTrajVjp`,
`VzCarryVjp`, `ZCarryVjp`) + `coupled_trajectory_material_gradient(n_steps, param_idx)` — ONE
`tape.backward(z_N)` over the coupled rollout. Gate `tests/coupled_trajectory_gradient.rs`.

**★ RESULT — the gradient is CORRECT, penalty-accuracy-capped (refines the trajectory-spike verdict).**
Each per-step factor is machine-exact — `TrajectoryStepVjp`'s four cotangents (state/material/pose) are
gated against independent re-solve FD in `sim-soft/tests/trajectory_step_vjp.rs`, and the rigid carry
reproduces sim-core — so by the chain rule the composition is exact too. The composed multi-step
gradient matches the full real coupled re-rollout FD to **~3e-4 (rel) while FIRMLY engaged** (`sd ≪ d̂`,
the shipped gate's measured value, gated at 6e-4), and the accuracy **improves as the rollout drives
the contact deeper** (the shipped `converges_with_engagement` gate checks n=8 ≫ n=20; a development
diagnostic sweep measured rel 16%→2.7%→0.03% over n=4→24, and FD-convergence across ~7 decades of ε —
both throwaway-spike figures, not reproduced by the committed gates). The FD oracle is rock-solid, so
`z_N(μ)` is genuinely smooth and the residual is **not** a formula error — it is consistent with the
penalty non-smoothness shrinking as `sd` moves below `d̂`.

**The honest caveat (penalty R3, refines the spike's "IPC not mandatory").** With penalty contact,
static force balance settles at the band edge `sd ≈ d̂` (force `κ(d̂−sd)→0` = weight). The **C⁰ force
keeps `z_N(μ)` smooth** (so the trajectory spike's single-step finding stands), but the force's
**derivative kinks** at the active-set boundary, so a per-step linearization (active set frozen at the
rollout's μ) loses accuracy as the platen reaches **marginal contact** or **bounces** through
make/break (rel climbs to 5–25%). So: the **single-step** gradient holds across make/break (spike), but
the **multi-step COMPOSED** gradient through marginal/bouncing contact is penalty-limited to ~1e-3..1e-4.
That is adequate for co-design gradient descent (which tolerates ~1e-4 gradients) → **IPC stays a
quality upgrade, not a blocker** — but it now has a *measured* number and a clear failure mode (the
marginal active set), which is exactly the gate a future IPC leaf must beat.

GOTCHAS hit: `tape.constant(f64)` makes a shape-`[]` scalar — use `constant_tensor(…,&[1])` for `[1]`
parents; the FD oracle moves `λ=4μ` (constructor tie) so it measures the TOTAL `∂/∂μ+4∂/∂λ` (compare
against that linear combo, the S5 convention); grade Doc-tier F on `[`CpuNewtonSolver::…`]` /
private-glue intra-doc links → plain backticks; grade Safety/Clippy F on unjustified `#[allow]`
(panic/expect/too_many_lines) → `//` justification directly above each.

## 8. Scope & ritual

Per leaf: recon → S0 spike (measure, throwaway/reverted) → sliced PRs → n+1 cold-read → pre-PR local
ultra-review; no push without go-ahead. Honest scope inherited from S1–S5: engaged / stable-within-
step active set / hard-penalty / constant-normal plane / free-body rigid factor. The deliverable is
one `tape.backward` across an N-step coupled rollout, FD-validated on the keystone scene — the
gradient substrate the co-design optimizer's trajectory/policy half consumes.
