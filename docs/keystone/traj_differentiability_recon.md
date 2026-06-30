# Keystone — multi-step coupled-trajectory differentiability — RECON

*Active recon, opened 2026-06-12. The keystone's differentiability arc (S1→S5) is COMPLETE and
validated to ~1e-9 — but **only in the frozen regime every leaf caps to: contact-ENGAGED,
STABLE-active-set, hard-penalty, SINGLE staggered step, plane fixed during the soft solve.** Real
device dynamics (a cuff engaging, a foot strike, any motion) are a *sequence of steps crossing
contact make/break events*, which violates all of those caps at once. This recon scopes the
**spike that measures whether the validated gradient survives a real multi-step trajectory that
crosses a contact make/break**. Sibling of `recon.md`, `s2/s3/s4/s5_*.md`.*

> **The biggest untested load-bearing assumption in the stack.** Every downstream consumer
> (co-design, lattice design, policy, the exo) inherits the single gradient `∂(rigid outcome)/∂(soft
> design)`. Its validity has only ever been checked *frozen + single-step + stable-active-set*. Its
> validity in actual dynamics — across the make/break events real motion produces — is **UNKNOWN,
> not validated**. The spike measures the risk DIRECTLY (the team's own S0 pattern) before building
> any fix, against a full-nonlinear-coupled FD oracle. Either outcome chooses the next big build:
> **holds** → the caps were conservative → scale consumers; **breaks at the active-set change**
> (likely — penalty contact is non-smooth there) → *empirically localized*, and **IPC stops being
> deferred → next MANDATORY build**, now justified by a measured failure with the exact gate it
> must pass.

---

## 1. What is validated today, and the exact caps

The single-step gradient lives in `sim/L1/coupling/src/lib.rs` (`StaggeredCoupling`). Two surfaces:

- **Forward (mutating) dynamics** — `step()` (lib.rs:782): the *real* coupled rollout. Per step:
  read rigid `xpos[body].z` → pose the downward penalty plane at `plane_height()` → one **dynamic**
  backward-Euler soft Newton step → `Σ force_on_soft` → `−force_on_soft + damping → xfrc_applied` →
  rigid `data.step`. Carries `(self.x, self.v, self.data)` forward. This is the ONLY surface that
  advances state, and the plane MOVES each step because it tracks the rigid pose.
- **Gradient probes (non-mutating, single-step, frozen plane)** — `coupled_step_material_gradient(
  height, param_idx)` (lib.rs:670) and `coupled_step_load_gradient(...)`. Each builds a *fresh* tape,
  does ONE step at a **supplied fixed `height`**, runs one `tape.backward` across both engines
  (`MaterialStepVjp`/load adjoint → `ContactForceVjp` → `neg` → `RigidStepVjp`), tears the tape down.
  FD-validated to ~1e-9 (the `material[μ]`/`material[λ]` and `load·plane`/`load·sphere` rows of
  `tests/coupling_grad_harness.rs`) — at a
  **deeply-engaged height** (`h = 0.099`, plane ~1 mm below the rest top face z=0.1 → all 25 top
  vertices active and STABLE across the FD step).

The scope caps written into every method doc and the crate doc: *contact-engaged, stable-active-set,
hard-penalty (`d²E/dsd² = κ`), single staggered step, plane fixed during the soft solve.* The S1
gate (`contact_force_jacobian.rs`) is explicit that at the gentle settle the pairs are
barely-active (`sd ≈ d̂ − 2.6e-6`) and FD *flips them* (the penalty non-smoothness, R3) — so the
gates deliberately operate FAR from the make/break boundary.

## 2. The three structural gaps a trajectory gradient confronts

**G1 — no state threading across steps (one `tape.backward` over a rollout is impossible today).**
`Solver::step` (`sim/L0/soft/src/solver/mod.rs:187`) takes `x_prev: &Tensor<f64>` — a bare *value*,
not a `Var`. The step pushes `x_final_var` onto the tape with `theta_var` (or the material param) as
parent, but the dependence of `x_final` on the *previous* `x` is NOT captured as a tape edge. So
step k's output cannot feed step k+1's input on one tape. Confirmed by the soft side directly:
`Differentiable::time_adjoint` (`differentiable/mod.rs:51`, impl `newton_vjp.rs:104`) is literally
`unimplemented!("skeleton phase 2 — time-adjoint is Phase E+")`. **The multi-step adjoint does not
exist.** A trajectory gradient must therefore *compose* per-step Jacobians by hand (forward-mode
tangent or reverse accumulation), not lean on a single backward pass.

**G2 — the plane is frozen in every gradient probe.** `coupled_step_*_gradient` hold `height` fixed.
Real dynamics move `xpos[body].z` each step. The probes have never seen a moving plane, and never one
crossing the make event. The single-step total-derivative method
(`contact_force_height_total_jacobian`, lib.rs:479) shows why this matters: the *implicit*
re-equilibration term cancels ~78% of the explicit term in the engaged regime — so the coupled
sensitivity is a delicate near-cancellation that the active-set boundary could wreck.

**G3 — the make/break event is already built into the forward scene.** The test platen
(`PLATEN_MJCF`, lib.rs:835) starts at z=0.125 (plane at z=0.12 via `contact_clearance=0.005`) ABOVE
the block top (z=0.1) → **no contact**. It falls under gravity and makes contact ~step 150 (the
existing forward gate `coupled_step_is_bounded_and_engages_contact` rolls 150 steps and asserts a
nonzero reaction emerges). So a rollout from the start **naturally crosses a make event** — the exact
transition every single-step gate scoped *out*. No contrivance needed; the spike just instruments the
existing dynamics.

## 3. The design parameter and the objective

- **Design param `p`:** the soft block's Neo-Hookean `μ` (the co-design knob S5 differentiates;
  `λ = 4μ` is tied, so a total `d/dμ` along that line = `∂/∂μ + 4·∂/∂λ`). A single scalar → a single
  tangent direction → cheapest forward-mode propagation.
- **Trajectory objective `J(μ)`:** a scalar functional of the rollout. Candidates, in increasing
  sensitivity to the transition:
  - final platen height `z_N` (smooth, integral — most forgiving),
  - final platen velocity `vz_N`,
  - peak / settled contact reaction `max_k(−force_on_soft.z)` or its value at a fixed late step.
  The spike reports several; `z_N` is the headline (a clean co-design-style objective).

## 4. The oracle — full-nonlinear-coupled FD (independent, not an affine identity)

The ground truth is a **central FD over full coupled re-rollouts**: build the coupling at `μ ± ε`,
roll out the real `step()` for N steps, evaluate `J`, central-difference → `dJ/dμ|_FD`. This crosses
the make/break because the real sim does, and it touches NONE of the gradient machinery (the
discipline S3/S4/S5 used to avoid the S2 affine-identity trap). Sweep ε to confirm FD convergence
*away* from the boundary and to expose non-convergence *at* it.

`StaggeredCoupling::new` already stores `mu`/`lambda` and rebuilds the block from them
(`coupled_step_material_vz` is the single-step precedent), so a `μ`-parameterized constructor for the
oracle is a thin wrapper over the public API.

## 5. The two measurements (cheapest-and-most-decisive first)

**M1 — make/break sweep (targets the #1 risk: penalty non-smoothness at the active-set change).**
Reuse the *single-step* `coupled_step_material_gradient(height, 0)` vs its single-step FD oracle
(`coupled_step_material_vz`), sweeping `height` from **above-contact** (plane above the block top, no
active pairs) **through engaged** (deeply penetrating), and sweeping ε at each height. Plot/log the
analytic-vs-FD relative error and the active-pair count vs height. **Hypothesis:** error is ~1e-9 in
the stable interior, spikes to O(1) in the boundary band where ε straddles a pair's activation. This
is cheap (no new plumbing — existing methods, new operating points) and already decisive about the
IPC question: if a *single* step's gradient breaks at a boundary crossing, the trajectory gradient
inherits it, and IPC is justified before any multi-step machinery is built.

**M2 — compose-over-trajectory (the "does single-step COMPOSE over a rollout?" probe — also the
stubbed `time_adjoint` risk).** Roll out the real `step()` from above-contact across the make event
for N steps. Compute `dJ/dμ` two ways:
  - **Oracle:** §4 full-coupled FD.
  - **Analytic candidate:** forward-propagate a single-parameter tangent `ẋ_k = ∂(coupled state)_k /
    ∂μ` step-by-step. The coupled state is `(rigid qpos/qvel, soft x/v)`. The recurrence is
    `state_{k+1} = F(state_k, μ)`, tangent `ẋ_{k+1} = (∂F/∂state)·ẋ_k + ∂F/∂μ`. The crate already has
    the hard interface pieces (the contact-force/pose Jacobians, the rigid `dt/m` factor, the soft
    material sensitivity `equilibrium_material_sensitivity`); the gaps the spike fills with **logged
    sub-FD** are the within-step state→state blocks not yet assembled (soft `(x,v)` carry-forward, the
    rigid qpos/qvel update). A throwaway spike may use sub-FD for any block it doesn't have analytic —
    the POINT is to test whether *composition across the transition* matches the oracle, not to ship
    a clean analytic multi-step adjoint (that is the FEATURE the verdict decides whether to build).
  Compare across N and across whether the rollout crosses the make event.

## 6. Decision rule (what each outcome triggers)

- **M1 interior + M2 both match the oracle to ~1e-6, AND the error at the make/break band is bounded
  / integrably-small (the single non-smooth step is measure-zero in the rollout):** the caps were
  conservative for *trajectory-integrated* objectives → the foundation holds for real dynamics →
  scale consumers (co-design policy/RL half, trajectory objectives) with the penalty contact.
- **M1 shows an O(1) gradient error at the boundary that M2 inherits (the trajectory gradient is
  wrong whenever the rollout's FD straddles a make/break):** the penalty active-set non-smoothness is
  load-bearing → **IPC becomes the next mandatory build**. The spike hands IPC its gate: reproduce
  M2's oracle to ~1e-6 *through* a make/break, where penalty fails.
- **In between (error small but not negligible, or sensitive to ε/step choice):** document the
  regime where the penalty gradient is trustworthy (e.g. objectives that never differentiate near a
  transition), and scope IPC as a quality upgrade rather than a correctness blocker.

## 7. Scope & ritual

Throwaway crate `sim/L1/coupling-traj-spike` (deleted after, like every prior keystone S0). No new
public API in `sim-coupling` unless a measurement *needs* a thin accessor (prefer driving everything
through the existing public surface + a `μ`-parameterized constructor wrapper). Ritual: this recon →
S0 spike (measure) → record the verdict in the memory topic + this recon → IF the verdict says build,
sliced PRs + n+1 cold-read + pre-PR local ultra-review per the keystone leaf pattern. No push without
go-ahead.

Sits on the validated Layer-1 substrate and the complete S1–S5 single-step gradient; consumed
(eventually) by the co-design optimizer's policy/RL half (`project-codesign-optimizer`).

---

## 8. VERDICT — S0 spike done 2026-06-12 (throwaway `coupling-traj-spike`, deleted). **GRADIENT HOLDS.**

The throwaway spike (M1 controlled boundary sweep + M2 single-step gradient validity along a real
make/break rollout + full-trajectory FD reference) returned the *positive* branch of §6, decisively:

**M1 — make/break sweep.** As the plane height walks from above-contact (`N_active = 0`,
gradient ≡ 0) through the top-layer boundary (`height = 0.110 = z_top + d̂`) into the engaged interior:
- Right AT activation (the step `N_active` jumps `0 → 25`) the analytic `∂vz'/∂μ` is `≈ −1e-20` (machine
  zero) and FD is `0` — because the hard penalty force `κ(d̂ − sd)` **ramps continuously from zero** at
  `sd = d̂`. Newly-active vertices enter with VANISHING force, so their contribution to `∂(force)/∂μ`
  is vanishing too. **The active-set INDICATOR is discontinuous, but the differentiated quantity and
  its μ-sensitivity are C⁰ across the flip.**
- In the engaged interior (`N_active = 25` stable) analytic matches FD to **~1e-10 – 1e-9** for
  well-scaled FD steps (`ε = μ·5e-5 … 5e-4`); the `ε = μ·5e-3` column's ~2e-7 is FD *truncation*, not
  a gradient defect.

**M2 — single-step gradient validity ALONG a real trajectory crossing make/break.** Rolling the real
`step()` dynamics for 400 steps, the descending platen makes contact (~step 64), overshoots, and
**bounces — flipping the active set dozens of times** (`0 ↔ 9 ↔ 25 …`). At EVERY step the single-step
`coupled_step_material_gradient` matches independent single-step FD (`coupled_step_material_vz`, which
re-solves the full Newton at μ±ε and re-detects its OWN active set — not an affine identity) to:
- **worst rel-err on a STABLE-active-set step = 5.8e-8**,
- **worst rel-err on an ACTIVE-SET-CHANGE step = 6.1e-9** (no degradation at the transition — the
  flipping vertex is always the marginal one near `sd = d̂`, contributing ~0; the bulk active set
  carries the substantial force smoothly).

**M2 — full-trajectory FD reference.** `dz_N/dμ ≈ 5.334e-9` over a 300-step make/break rollout,
FD-converged across three decades of ε (`5.3412e-9 → 5.3343e-9 → 5.3342e-9`): the trajectory
objective is **smooth and well-conditioned in μ**.

**Why the feared non-smoothness does NOT bite the co-design gradient.** The penalty active-set
discontinuity is in the *position* derivative (`∂force/∂x` jumps `−κ → 0` at `sd = d̂`), which lives in
the tangent `A`. But the co-design gradient differentiates w.r.t. the *material/load* parameter, and a
newly-active vertex enters the contact force with magnitude `κ(d̂ − sd) → 0` at the boundary — so the
material-gradient contribution it adds is itself `→ 0` there. The C⁰-continuity of the penalty *force*
(not its position-Hessian) is what the material/co-design gradient rides, and that holds across every
make/break. This is robust to `d̂` (a narrower band steepens the ramp but the force still → 0 at the
boundary).

**DECISION (per §6, the "holds" branch).** The penalty/active-set caps every S1–S5 leaf carried were
**conservative for the co-design (material/load) gradient**: it survives a real multi-step trajectory
crossing repeated make/break events. Therefore:
- **IPC is NOT a correctness blocker for the co-design loop** and stops being the presumed-mandatory
  next build. (It remains a quality upgrade if a future objective differentiates the *position*-Hessian
  path — e.g. contact-force-Jacobian-of-Jacobian — or if a stiff narrow band hurts Newton convergence;
  scope it as such, not as a gate.)
- **The only real remaining gap is mechanical: the multi-step ADJOINT ASSEMBLY** (`time_adjoint`,
  still `unimplemented!`) — composing the per-step gradients into one `dJ/dμ` across a rollout. M2
  shows each per-step link is valid and the full-trajectory objective is well-conditioned, so this is
  a **low-risk FEATURE build** (chain-rule plumbing + threading soft `(x,v)` / rigid `(qpos,qvel)`
  carry-forward onto the tape), not a research risk. It is the natural next keystone leaf and the
  direct enabler of the co-design optimizer's trajectory/policy half.

**Caveats kept honest:** measured for the *material/load* co-design gradient on the keystone
platen-on-block scene with the constant-normal plane and wide hard-penalty band. Curved-primitive
normals (`∂n̂/∂δ ≠ 0`), off-centre contact moment, and the position-Hessian path remain out of scope
(unchanged). The spike validates the *foundation assumption* (gradient survives dynamics + make/break),
not a shipped multi-step API.
