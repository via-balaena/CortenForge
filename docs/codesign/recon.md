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
