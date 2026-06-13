# Co-design optimizer — RECON

*Active recon, opened 2026-06-10. Mission connective-tissue #2 ("Co-design optimizer — one outer
loop differentiating w.r.t. both design and policy parameters", `MISSION.md`). The first CONSUMER
of the keystone gradient substrate (the differentiable soft↔rigid coupling, S1–S5, all merged): an
outer loop that uses `∂(rigid outcome)/∂(soft design)` to optimize a design parameter toward a
target. v1 differentiates w.r.t. a DESIGN parameter (soft material μ); policy parameters (RL/control)
are the later half of the mission's "both".*

> **Close the first co-design loop: a gradient-based optimizer that consumes the keystone's
> `∂vz'/∂μ` (validated to 7.5e-9 vs FD) to tune the soft body's material so a rigid-side outcome
> hits a target — converging, validated by recovering a known design from its target behavior
> (inverse design).**

---

## 1. What the substrate provides, and what's missing

The keystone (`sim-coupling`, merged) exposes the design gradient:
- `coupled_step_material_gradient(height, param_idx) → (vz', ∂vz'/∂p)` — the rigid outcome `vz'`
  and its sensitivity to the soft material parameter (0=μ, 1=λ), via one `tape.backward` across
  both engines. Evaluated at the coupling's constructed material.
- `coupled_step_material_vz(height, param_idx, value) → vz'` — forward `vz'` at an arbitrary
  material value (rebuilds the soft solver). The black-box.

The chassis (`sim-ml-chassis`, L0) provides the optimizer:
- `OptimizerConfig::adam(lr).build(n) → Box<dyn Optimizer>`; `Optimizer::step_in_place(params,
  gradient, ascent)` — Adam with per-parameter adaptive scaling (handles the μ~3e4 vs gradient~1e-5
  scale mismatch) + gradient clipping. **Reuse it** — `sim-opt` is gradient-FREE (SA/PT), the wrong
  branch; the gradient-based optimizer already exists.

**Missing = the connective tissue:** an outer loop that (a) abstracts a differentiable design
objective, (b) drives the chassis optimizer with the coupling's gradient to convergence, (c) is a
clean, composable, worked-example-backed API (the mission's quality bar). That's the new crate.

## 2. Architecture

A new crate **`tools/cf-codesign`** (depends on `sim-coupling` + `sim-ml-chassis`; tools/ is the
precedent for sim-stack-composing crates — `cf-device-design`, `cf-mjcf-emit` — and the grader has
no L2 tier yet, so a tools/ home avoids new tier infrastructure):

- **`CoDesignProblem` trait** — the differentiable-objective abstraction the optimizer drives:
  `fn n_params(&self) -> usize;` and `fn evaluate(&self, params: &[f64]) -> (f64 /*loss*/, Vec<f64>
  /*grad*/);`. Decouples the optimizer from the coupling — any differentiable design objective
  (material, later geometry/lattice, policy) plugs in.
- **`optimize(problem, x0, OptConfig) → OptResult`** — drives `Adam`: each iter `(loss, grad) =
  problem.evaluate(x); opt.step_in_place(&mut x, &grad, /*ascent=*/false)`; stop on `‖grad‖∞ <
  grad_tol` OR `loss < loss_tol` (absolute) OR `max_iters` (`grad_tol = 0` disables the gradient
  stop — the default, leaving the absolute-loss stop). Records the per-iter `(params, loss, ‖grad‖)` history
  (the gate checks monotone-ish descent + convergence). Optional per-param lower bound (μ > 0)
  clamped after each step.
- **`SoftMaterialTarget` concrete problem** — wraps the coupling: holds the MJCF + block/contact
  params + a target rigid outcome `vz*` + the design `param_idx`. `evaluate([p])` rebuilds the
  coupling at `p` (fresh `Model`/`Data` — `Data` is not `Clone`), calls
  `coupled_step_material_gradient(height, param_idx)`, and returns `loss = ½(vz' − vz*)²`,
  `grad = (vz' − vz*)·∂vz'/∂p`. Uses ONLY the existing public coupling API.

## 3. The first demo / gate — inverse design

`vz'(μ)` is monotone increasing (`∂vz'/∂μ > 0`, keystone-validated: a stiffer soft body pushes the
platen harder), hence invertible — so a target `vz* = vz'(μ*)` has a unique minimizer `μ*`. The
gate: pick `μ*`, compute `vz* = vz'(μ*)`, optimize from `μ₀ ≠ μ*`, and confirm the loop **recovers
`μ*`** (loss → 0, `μ → μ*`). A clean, validatable inverse-design demo: *given a target rigid
behavior, the optimizer recovers the soft material that produces it* — the first closed co-design
loop, end to end through the differentiable coupling.

## 4. #1 RISK and the spike

**#1 RISK — does the gradient-driven loop CONVERGE?** The gradient is keystone-validated, but the
optimization must close: the scale mismatch (`μ ~ 3e4`, `∂vz'/∂μ ~ 2e-5`, so `dL/dμ ~ 1e-5·residual`)
could stall vanilla GD; the loss must be well-conditioned; `μ > 0` must hold. MITIGATION: Adam's
adaptive per-param scaling normalizes the magnitude; the spike measures convergence directly.

**SPIKE (throwaway, `#[ignore]`):** a 1-D Adam loop on μ toward `vz* = vz'(μ*)` for a known `μ*`,
from `μ₀ ≠ μ*` (rebuilding the coupling per eval, the existing API). **Answers:** does it converge
(loss → 0)? does it recover `μ*`? how many iters / what lr? Retires R1 before the crate.

**SPIKE DONE 2026-06-10 (throwaway, deleted) — ★ R1 RETIRED.** A `#[ignore]`'d test in
`sim-coupling` (it has the chassis dep): target `vz* = vz'(μ*=4e4)`, optimize from `μ₀=2e4` with
chassis `Adam(lr=2e3)`, rebuilding the coupling per eval, `grad = (vz'−vz*)·∂vz'/∂μ`. **Result:
recovered `μ = 40000.05` (μ*=40000) to rel 1.2e-6, loss 3e-11, ~190 iters.** The loop CONVERGES —
the gradient-driven co-design optimization closes end-to-end through the differentiable coupling
(given a target rigid behavior, the optimizer recovers the soft material that produces it). Adam's
adaptive scaling handled the `μ~3e4` vs `grad~1e-5` mismatch (some early overshoot, then convergence).
Ready to productionize as `cf-codesign`.

## 5. Decisions (head-engineer)

- **D1 — Reuse the chassis `Adam`; don't reinvent.** Composability is the mission's quality bar;
  `sim-ml-chassis::Optimizer`/`Adam` is the validated gradient optimizer. `sim-opt` is gradient-free.
- **D2 — `tools/cf-codesign`.** Composes L1 (coupling) + L0 (chassis); tools/ is the precedent
  (no L2 grader tier exists). Move to a sim tier later if it becomes core.
- **D3 — `CoDesignProblem` trait decouples optimizer from objective.** The optimizer is generic;
  the coupling is one impl. Future design vars (geometry, lattice, policy) plug into the same loop.
- **D4 — First objective = inverse design (recover μ* from vz*).** Achievable + validatable
  (monotone ⇒ unique minimizer); avoids a contrived target. The optimizer's CORRECTNESS is what v1
  proves; richer objectives (multi-param, multi-step rollouts via the deferred time-adjoint,
  manufacturing constraints) come later.

## 6. Sub-leaf ladder / slicing

- **Spike (THROWAWAY, `#[ignore]`).** §4 — retire R1: a 1-D Adam loop recovers μ* from vz*.
- **PR1 — `cf-codesign` crate.** `CoDesignProblem` + `optimize` (reusing chassis Adam) +
  `SoftMaterialTarget` + the inverse-design gate (recovers μ* to tolerance; loss → 0; converged)
  + a worked example. grade A.
- **(later) PR2+** — multi-parameter (μ AND λ / geometry), box/manufacturing constraints,
  multi-step objectives (needs the deferred soft time-adjoint), and the policy half (RL).

Each leaf: n+1 cold-read + pre-PR local ultra-review; no push/PR without go-ahead.

## Progress · PR1 DONE (2026-06-10) — ★ THE FIRST CLOSED CO-DESIGN LOOP

New crate **`tools/cf-codesign`** (deps: `sim-coupling` + `sim-mjcf` + `sim-ml-chassis`):
- `CoDesignProblem` trait (object-safe: `n_params`, `evaluate → (loss, grad)`, optional
  `lower_bounds`) — the differentiable-objective abstraction.
- `optimize(problem, x0, OptConfig) → OptResult` — drives the **reused** chassis `Adam`
  (`step_in_place`, `ascent=false`); stops on `‖grad‖∞`/`loss`/`max_iters`; clamps lower bounds;
  records the descent history.
- `SoftMaterialTarget` — the concrete coupling problem: tune the soft stiffness `μ` (with `λ=4μ`,
  i.e. scale stiffness at fixed Poisson) so the platen's `vz'` hits a target; rebuilds the coupling
  per eval and uses the **total** `dvz'/dμ = ∂/∂μ + 4·∂/∂λ` (the S5 combo — matches the rebuild-path
  FD, vs the spike's ∂/∂μ-only approximation).
- Gate `tests/inverse_design.rs`: (1) the problem gradient matches an FD of its loss to **6.1e-10**
  (correct for the objective, not just a descent direction); (2) **inverse design recovers
  `μ*=4e4` from `μ₀=2e4` to rel 2.7e-6, loss 9.4e-12, `converged` in 151 iters**. Plus a worked
  `examples/inverse_design.rs`. grade A.
- `xtask/src/grade.rs`: added `cf-codesign` to the tools/ Layer-Integrity exemption list (no SDK
  tier, like the other `tools/cf-*` crates).

★ The body↔device co-design loop is CLOSED end to end: a target rigid behavior → the differentiable
coupling gradient → an optimizer → the soft material that produces it. The mission's flagship loop
runs (single design param / single step v1). NEXT: multi-parameter + geometry/lattice design vars,
constraints, multi-step (time-adjoint), and the policy half (RL co-optimization).

## 7. Risks

- **R1 (#1) — convergence/conditioning.** §4 spike measures it; Adam mitigates the scale mismatch.
- **R2 — eval cost** (rebuild the coupling + a coupled step per iter). MITIGATION: v1 is a small
  scene + ~tens of iters; acceptable. A warm-started / in-place coupling rebuild is a later
  optimization.
- **R3 — single design parameter / single-step objective** (the substrate's current reach). Honest
  scope: v1 proves the LOOP closes on one design var toward a one-step outcome; multi-param,
  multi-step (time-adjoint), and policy co-optimization are documented follow-ons.
- **R4 — local optimum / non-convexity** for richer objectives. v1's inverse-design objective is
  convex-ish (monotone outcome ⇒ unique minimizer); documented as a v1 scoping choice.

## 8. Validation gate (v1)

CI-runnable: the optimizer recovers a known design `μ*` from its target behavior `vz* = vz'(μ*)`
(final `|μ − μ*|/μ*` and loss below tolerance, `converged = true`), with a monotone-ish descent
history. Honest scope: single design parameter, single coupled step, contact-engaged regime
(inherits the keystone caps).

## Key files / pointers

- Keystone gradient: `sim/L1/coupling/src/lib.rs` (`coupled_step_material_gradient`,
  `coupled_step_material_vz`, `StaggeredCoupling::new`).
- Optimizer to reuse: `sim/L0/ml-chassis/src/optimizer.rs` (`OptimizerConfig::adam`, `Optimizer`,
  `Adam`, `step_in_place`).
- Composition precedent: `tools/cf-device-design` (a tools/ crate consuming the sim stack).
- Keystone recons: `docs/keystone/{recon, s2_…, s3_…, s4_…, s5_…}.md`.

---

# v2 — the TRAJECTORY half (opened 2026-06-13)

*The first real consumer of the keystone's now-machine-clean MULTI-step coupled gradient
(`coupled_trajectory_material_gradient`, made machine-exact through contact make/break by the IPC
arc #307). v1 consumed only the single-step gradient; nothing consumed the trajectory gradient yet
— that's the gap this closes.*

> **Close the trajectory co-design loop: a `SoftMaterialTrajectoryTarget` whose cost is
> `½(z_N − z*)²` after an N-step contact-engaged coupled rollout, with the gradient
> `∂z_N/∂μ + 4·∂z_N/∂λ` read from `coupled_trajectory_material_gradient(n, idx)` — recovering a
> known μ* over a whole trajectory. Reuses the Adam `optimize` loop + `CoDesignProblem` trait; the
> only new objective code swaps the single-step gradient for the trajectory one.**

## v2.1 What the substrate now provides

The keystone (merged #306 time-adjoint, #307 IPC fix) exposes the multi-step design gradient:
- `coupled_trajectory_material_gradient(n_steps, param_idx) → (z_N, ∂z_N/∂p)` — the platen's FINAL
  height after `n_steps` of the REAL coupled `step` dynamics, and its sensitivity to the soft
  material parameter (0=μ, 1=λ), via ONE `tape.backward(z_N)` across both engines AND every step
  boundary. **`&mut self`** — it advances the coupling, so each call consumes a fresh build (unlike
  the single-step `&self` gradient, which one build can call twice for idx 0 and 1).
- The block ties `λ = 4μ` at construction, so a rebuild-path FD along that line measures the TOTAL
  `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ` — the documented S5 linear combination. The objective gradient
  uses the same total.

## v2.2 Design — `SoftMaterialTrajectoryTarget`

A new `CoDesignProblem` parallel to `SoftMaterialTarget`, the single-step path left untouched:
- Holds the MJCF + block/contact params + `n_steps` + `target_z`. The platen MJCF starts already
  in contact (e.g. `pos.z = 0.108`, plane at `z − clearance`), so the rollout is engaged from
  step 0 — no separate probe `height` (the rollout reads `self.plane_height()` internally each step).
- `evaluate([μ])`: build a fresh coupling at μ, call `coupled_trajectory_material_gradient(n, 0)` →
  `(z_N, ∂z_N/∂μ)`, then a SECOND fresh build for `(n, 1)` → `∂z_N/∂λ` (two builds because the
  method mutates `self`). `loss = ½(z_N − z*)²`, `grad = (z_N − z*)·(∂z_N/∂μ + 4·∂z_N/∂λ)`.
- `forward_z(μ)` runs the rollout and returns `z_N` (to set up an inverse-design target).

## v2.3 S0 spike (THROWAWAY, deleted) — measured, three findings

A `#[ignore]`'d spike in `cf-codesign` prototyped the problem inline and ran the keystone
platen-on-block scene (start `z = 0.108`, n = 20, damping 60, κ = μ = 3e4, d̂ = 1e-2):

1. **★ Gradient FD-gate: MACHINE-EXACT.** `d/dμ ½(z_N − z*)²` analytic `1.143423e-8` vs central FD
   of the trajectory loss (fresh coupling rebuilt + real `step` loop per μ±ε — an INDEPENDENT
   oracle) `1.143423e-8`, **rel 9.3e-9**. The trajectory gradient is consumed correctly.
2. **Conditioning: monotone & invertible, weakly sensitive.** `z_N(μ)` rises monotonically
   (`z(2e4) = 0.1234 < z(4e4) = 0.1255`) ⇒ a unique minimizer; slope `dz_N/dμ ≈ 1e-7` (n ≤ 20;
   it falls to ~4e-8 by n = 30 as the platen settles). The whole μ ∈ [2e4, 4e4] range moves `z_N`
   by only ~2 mm — weak sensitivity, but that's the physics (a position outcome), not a defect.
3. **★ Inverse-design recovery + the ONE design refinement.** With the chassis Adam at its default
   `eps = 1e-8`, recovery CRAWLED (rel ~5e-2 after 1200 iters). Root cause: the objective gradient
   is `residual·slope ≈ 2e-3 · 1e-7 ≈ 2e-10`, BELOW Adam's `eps`, so `lr·m̂/(√v̂ + eps)` is
   eps-dominated and Adam degenerates to tiny SGD-like steps (loses its scale-invariance). Dropping
   `eps` below the gradient scale restores it: recovery to **rel ~1e-10** (loss ~1e-26) in ~370
   iters, robust across `lr ∈ [2e3, 1e4]` and `eps ∈ [1e-12, 1e-16]`. (The single-step v1 escaped
   this only narrowly — its gradient ~2e-7 sits just above eps.)

## v2.4 Decision (head-engineer) — expose Adam `eps` in `OptConfig`

The spike forced one refinement to the shared loop: **add an `eps` field to `OptConfig`** (default
`1e-8`, so the single-step `SoftMaterialTarget` gate is byte-for-byte unchanged) and have `optimize`
build the optimizer with the full `OptimizerConfig::Adam { … eps … }` instead of the
`OptimizerConfig::adam(lr)` shortcut. Rationale: `eps` is a legitimate, pre-existing Adam knob;
exposing it is NOT reinventing the optimizer, and it's a GENERAL need — any small-sensitivity design
variable (a position-valued trajectory outcome today, geometry/lattice tomorrow) produces gradients
that fall below the default eps. The `optimize` loop logic and the `CoDesignProblem` trait are
otherwise reused verbatim; only the objective impl and the optimizer *construction* change.

## v2.5 Ladder / slicing

- **S0 spike (THROWAWAY, `#[ignore]`).** §v2.3 — DONE: gradient machine-exact, recovery rel ~1e-10
  once eps is tuned.
- **PR (this leaf):** `OptConfig.eps` threaded through `optimize` (default 1e-8) +
  `SoftMaterialTrajectoryTarget` + a trajectory inverse-design gate (gradient-vs-FD machine-exact;
  recovers μ* to tolerance; `converged`) + a **make/break showcase gate** (a platen starting above
  contact that falls, makes contact, rebounds and breaks, then re-makes — gradient stays
  machine-exact across both onsets and the release) + a worked example. grade A. Single-step path
  untouched.
- **(later)** richer trajectory objectives (waypoints / multi-target), multi-parameter, the policy
  half (RL) — the trajectory gradient also feeds a control/policy outer loop.

## v2.6 Validation gate (v2)

CI-runnable, **measured on the shipped gate** (`tests/trajectory_inverse_design.rs`, eps = 1e-12,
loss_tol = 1e-18 via `SoftMaterialTrajectoryTarget::recommended_config`): (1) the problem's analytic
gradient matches a central FD of the trajectory loss to **rel 9.3e-9** (gradient correct for the
objective, not just a descent direction); (2) the optimizer recovers `μ* = 4e4` from `μ₀ = 2e4` to
**rel 2.93e-7** (μ = 39999.9883, loss 5.5e-19, 229 iters, `converged = true`) — descending orders of
magnitude. (The §v2.3 spike figures rel ~1e-10 / loss ~1e-26 / ~370 iters were a tighter throwaway
run, not this gate.) (3) a **make/break showcase** (`trajectory_gradient_matches_fd_through_make_break`):
a platen started above contact (z = 0.125, damping 8) falls, makes contact (~step 64 of 80), rebounds
and breaks (~steps 68–77), then re-makes (~step 78), and the gradient-of-objective stays machine-exact
vs FD (**rel ~4e-8**) across both onsets and the release — the *convergence* scene above is
engaged-from-step-0, so this carries the gradient-correctness-through-make/break proof separately (a
full recovery over an 80-step rollout would be minutes). Honest scope: single design parameter; the
convergence gate's rollout is contact-engaged (deepening/settling) with no make/break event, while the
make/break showcase exercises genuine make/break/re-make at the objective level (and the keystone/IPC
gates establish it independently against a re-rolled FD oracle); the keystone's penalty contact
(machine-clean multi-step after the #307 carry fix); the eval rebuilds the coupling per parameter
(R2 — acceptable at this scene size).

## v2.7 Key files / pointers (v2)

- Trajectory gradient: `sim/L1/coupling/src/lib.rs` (`coupled_trajectory_material_gradient`).
- Keystone trajectory gate (the FD-oracle `final_z` pattern to mirror):
  `sim/L1/coupling/tests/coupled_trajectory_gradient.rs`, `ipc_trajectory_gradient.rs`.
- Optimizer: `sim/L0/ml-chassis/src/optimizer.rs` (`OptimizerConfig::Adam { eps }`).
- Time-adjoint / IPC recons: `docs/keystone/time_adjoint_recon.md`, `docs/ipc/recon.md`.
