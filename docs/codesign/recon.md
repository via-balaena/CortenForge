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

> **⚠ SUPERSEDED by §v3** (2026-06-13): the `eps = 1e-12` /
> `SoftMaterialTrajectoryTarget::recommended_config` path below was a conditioning band-aid and has
> been RETIRED. The shipped gate now wraps the target in `Normalized` and recovers μ* with the
> STANDARD `eps = 1e-8` (rel 4.72e-7). This section is kept for history.

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
  `sim/L1/coupling/tests/coupled_trajectory_gradient.rs`, and the `ipc-traj·material` rows of
  `sim/L1/coupling/tests/coupling_grad_harness.rs` (IPC).
- Optimizer: `sim/L0/ml-chassis/src/optimizer.rs` (`OptimizerConfig::Adam { eps }`).
- Time-adjoint / IPC recons: `docs/keystone/time_adjoint_recon.md`, `docs/ipc/recon.md`.

---

# v3 — the NORMALIZED / dimensionless objective (opened 2026-06-13)

*The refinement deferred when shipping v2. v2 shipped a working trajectory loop but only by
papering over a conditioning smell with a tiny Adam `eps`. This session removes the smell: make the
objective WELL-SCALED so `OptConfig::default()` (eps = 1e-8) recovers μ* to ~1e-6 with NO special
eps. Measure-first — the spike's verdict drives the abstraction (bake-in vs general wrapper).*

> **The trajectory loss `½(z_N − z*)²` is dimensionally raw: `z_N` is a position (~0.12 m), so
> `∂z_N/∂μ ~ 1e-7` and the loss gradient `~2e-10` falls BELOW Adam's default `eps = 1e-8`. Adam's
> update `lr·m̂/(√v̂ + eps)` is scale-invariant ONLY when `|grad| ≫ eps`; below eps the `+eps`
> dominates the denominator and Adam degenerates to eps-dominated SGD that crawls. v2 fixed it by
> dropping `eps` to 1e-12 (`recommended_config`). That works but is a band-aid. The clean fix: a
> dimensionless residual (and possibly a relative parameter) so the gradient lands well above the
> default eps and `OptConfig::default()` Just Works.**

## v3.1 The scaling, worked out from the Adam update (the basis for the spike)

`step = lr · m̂ / (√v̂ + eps)` (`optimizer.rs:244`). With `m̂, √v̂ ∝ |grad|`: when `|grad| ≫ eps`
the step is `~ lr · sign(grad)` (scale-invariant — gradient *magnitude* cancels); when `|grad| ≲ eps`
the step is `~ lr · grad / eps` (proportional to grad, i.e. ordinary SGD with effective rate
`lr/eps` — and since `grad` is tiny, it crawls). So the lever that matters is the gradient magnitude
relative to eps, NOT the loss magnitude per se.

Two independent normalizations move the gradient magnitude:

- **Residual normalization** — report `r = (z_N − z*)/L` for a characteristic length `L`, loss
  `½r²`. This scales the loss by `1/L²` and **the loss-gradient by `1/L²`** (loss-grad
  `= r · (∂z_N/∂μ)/L = (z_N − z*)(∂z_N/∂μ)/L²`; the original raw gradient is the `L = 1` case). So
  `L = 0.1` (block edge) → ×100; `L = d̂ = 0.01` (contact band) → ×1e4. Keeps the parameter PHYSICAL
  (μ), so the tuned `lr = 2e3` still applies.
- **Parameter normalization** — optimize in `p = μ/μ_ref` (linear) or `p = ln μ` (log). The chain
  rule multiplies the gradient by `dμ/dp` = `μ_ref` (linear) or `μ` (log) ≈ 3e4. The bigger lever,
  AND it makes Adam steps *relative* (a step in `ln μ` is a fractional change in μ; log also enforces
  μ > 0 for free, no lower-bound clamp). BUT it changes the parameter space, so the tuned `lr = 2e3`
  no longer applies (a step in `p ~ O(1)` wants `lr ~ O(0.01–0.1)`) — a re-tune the wrapper/target
  must supply.

**The subtlety the spike must pin down (worked numbers, μ ≈ 4e4, slope ∂z_N/∂μ ≈ 1e-7):**
a smooth loss has `grad → 0` at the optimum, so with ANY fixed eps the *late* phase eventually
crawls. What makes an objective "work with default eps" is the EARLY/MID phase being above eps (fast
descent) with `loss_tol` stopping it at the target recovery — exactly how the single-step v1 path
(early grad ~2e-7 > eps) reaches 2.7e-6 while the trajectory (grad ~2e-10 < eps from step 0) never
gets the fast phase. So:
- residual-norm with `L = d̂` lifts the EARLY grad to ~1e-6 (≫ eps) → fast early descent restored
  with default eps; but near the optimum grad dips back under eps, so the achievable recovery is set
  by `loss_tol` on the normalized loss (a per-objective stopping choice, not a scaling band-aid).
- log-μ on TOP keeps the gradient `× μ` lifted relative to the residual, holding it above eps down to
  much tighter recovery — at the cost of an `lr` re-tune. The spike measures whether residual-norm
  alone hits ~1e-6, or whether log-μ is needed.

**`loss_tol` re-tune (the flagged gotcha):** a normalized loss has a different scale, so the
absolute `loss_tol = 1e-10` default maps to a different recovery. With `L = d̂`, `loss_tol = 1e-10`
⇒ normalized residual `~1.4e-5` ⇒ μ error `~3.5e-5`; reaching ~1e-6 wants `loss_tol ~ 1e-13`. This
is a stopping threshold, NOT the eps smell — re-tune it for the normalized scale (or scale `L` so the
default lands where we want). The WIN this session banks is **default eps**, the documented smell.

## v3.2 The design fork (the real output)

- **(A) Bake normalization into `SoftMaterialTrajectoryTarget`** — it picks a sensible `L` (block
  edge / `d̂` / `|z*|`) and reports `r = (z_N − z*)/L`. Simplest; zero new surface; but every future
  weakly-sensitive design var (geometry, lattice, policy) re-solves the same scaling wall.
- **(B) A general `CoDesignProblem` normalization wrapper/decorator** that rescales ANY objective's
  loss + gradient (and optionally reparametrizes the design vars). More composable — the mission's
  quality bar (SDK) — and the next design vars hit the EXACT same small-sensitivity wall. Lean = (B)
  if it stays clean; the spike confirms it stays clean and that the wrapped objective recovers ≥ as
  well as a bake-in.

Sub-question for (B): how general? A pure **loss/gradient scaler** (multiply loss & grad by a
constant `k`) is a ~5-line decorator that needs no parameter-space change and preserves `lr` — but
the constant must come from somewhere (caller-supplied `L`, or auto-probed from `|grad(x0)|`). A
**reparametrizing** wrapper (log/linear param transform) is more machinery (un-normalize `x0`,
transform the gradient by `dμ/dp`, denormalize the result) but delivers relative steps. The spike
measures which is actually needed for ~1e-6.

## v3.3 S0 spike (THROWAWAY, `#[ignore]`) — the plan

Inline in `cf-codesign` (`tests/zzz_normalize_spike.rs`, deleted after) — wrap the EXISTING
`SoftMaterialTrajectoryTarget` (already a `CoDesignProblem`) with candidate normalizers and run
`optimize` head-to-head on the engaged scene (n = 20, μ* = 4e4, μ₀ = 2e4). Measure for each: final
rel-μ recovery, loss, iters, converged. Variants:

0. **Baseline-today** — raw problem, `recommended_config()` (eps = 1e-12). Reproduce rel ~2.9e-7.
0b. **Baseline-crawl** — raw problem, `OptConfig::default()` (eps = 1e-8). Reproduce the crawl.
1. **Residual-norm only** (`k = 1/L²`, L ∈ {0.1, 0.01}), `OptConfig::default()` eps = 1e-8, sweep
   `loss_tol`. Does default eps recover ~1e-6?
2. **Log-μ only**, eps = 1e-8, re-tuned `lr`. Does it recover ~1e-6?
3. **Residual-norm + log-μ**, eps = 1e-8. The robust combo.
4. **Auto-probe scaler** — `k = 1/|grad(x0)|`, eps = 1e-8 (the fully-objective-agnostic wrapper).

**FD-gate the normalized gradient** for the chosen variant (analytic grad in the OPTIMIZER's
parameter space vs central FD of the normalized loss in that space) — confirms the chain-rule
bookkeeping (the `1/L²` loss-scale and, if reparametrized, the `dμ/dp` factor). Watch the λ = 4μ
ctor tie: the inner gradient is already the TOTAL `∂/∂μ + 4∂/∂λ`, and the FD must move along that
same line (rebuild-path FD), as the v2 gates do.

**Decision rule:** pick the SIMPLEST variant that recovers ~1e-6 with default eps; then decide A vs B
on whether the general wrapper stays clean and recovers ≥ the bake-in.

## v3.4 ★ S0 SPIKE DONE 2026-06-13 — measured, verdict: FORK B (general wrapper, both levers)

Throwaway `tests/zzz_normalize_spike.rs` (deleted) wrapped the existing `SoftMaterialTrajectoryTarget`
with candidate normalizers and ran `optimize` head-to-head (n = 20, μ* = 4e4, μ₀ = 2e4, target_z =
0.125455). All normalized runs used the **standard eps = 1e-8**:

| Variant | μ recovery (rel) | iters | conv | vs default |
|---|---|---|---|---|
| 0  raw + recommended (eps **1e-12**) | 2.93e-7 | 229 | ✓ | the v2 band-aid |
| 0b raw + **default eps 1e-8** | **0.26 (CRAWL)** | 300 | ✗ | the smell |
| 1  resid L = 0.01, lt = 1e-13 | **1.16e-6** | 186 | ✓ | loss_tol only (lr = 2e3 kept) |
| 1  resid L = 0.1, lt = 1e-13 | 4.6e-6 | 157 | ✓ | loss_tol only |
| 2  **log-μ**, lr = 0.05 | **1.87e-7** | 218 | ✓ | lr + loss_tol |
| 3  resid(L = .01) + log-μ, lr = 0.05 | 4.1e-7 | 180 | ✓ | lr + loss_tol |
| 4  auto-probe k = 1/‖g(x₀)‖, lt = 1e-13 | 5.8e-10 | 316 | ✓ | loss_tol only |

Each row isolates a single lever (or one L). The **shipped gate** uses BOTH levers at the gate scene
— `with_residual_scale(L = 0.1, log_space = true)`, `lr = 0.05`, `loss_tol = 1e-15` — and recovers
**rel 4.717e-7 in 180 iters** with the standard eps (re-tuned from variant 3's `L = 0.01 / lt = 1e-13`
row: the larger L = block-edge makes the loss 100× smaller so `loss_tol` tightens by ~100× for a
comparable recovery). The standout single-lever numbers above (e.g. **1.87e-7 is log-μ ALONE**, k = 1)
isolate *why* each lever helps, not the shipped config.

**Findings:**
1. **The crawl is real** (0b: rel 0.26, never converges) and **every normalized variant fixes it
   with the standard eps = 1e-8** — the eps band-aid is removable.
2. **The PARAMETER (log-μ) normalization is the stronger lever, an inversion of the v3.1 prior.**
   log-μ ALONE recovers to 1.87e-7 (matching the eps = 1e-12 baseline) because `dμ/dp = μ ≈ 3e4`
   lifts the gradient above eps *throughout* descent (relative steps). Residual-norm alone tops out
   ~1e-6: its gradient dips back under eps near the optimum (a smooth loss has `grad → 0` there), so
   only the early phase is fast — it helps but is secondary.
3. **FD bookkeeping exact** (`spike_normalized_gradient_fd`): the loss-scale `k` and the `dμ/dp = μ`
   chain factor each match a central FD to **rel 9.7e-9** (LossScaled grad 1.143e-4, LogParam grad
   4.002e0 — both at target = 0, μ = 3.5e4).
4. Only `lr`/`loss_tol` track the normalized space (lr matches the relative-step scale, loss_tol the
   dimensionless loss scale — the flagged gotcha, handled). **eps stays standard 1e-8.**

**★ VERDICT — Fork B, a general `Normalized` wrapper doing BOTH levers.** The complete dimensionless
objective = `loss_scale` (residual / L → a dimensionless, interpretable loss, so `loss_tol` is a
relative-tolerance²) + `log_space` (optimize positive design vars in ln-space → relative steps that
hold the gradient above the standard eps, and enforce `> 0` structurally). Chosen over bake-in (A)
because: (i) the spike shows B stays clean (one struct, two fields: `loss_scale`, `log_space`; +
`to_physical`/`to_normalized` helpers) and *outperforms* a residual-only bake-in (1.8e-7 vs 1.16e-6);
(ii) composability is the mission's SDK quality bar and the next design vars (geometry / lattice /
policy) hit the EXACT same small-sensitivity wall — log-of-a-positive-scale-parameter + dimensionless
residual is reusable connective tissue, not a one-off; (iii) the eps band-aid (`recommended_config`
eps = 1e-12) is RETIRED — the new path uses the standard eps and the wrapper supplies the
parameter-space-appropriate `lr` (and dimensionless `loss_tol`), which is principled per-objective
tuning rather than a per-scene magic eps tracking an un-normalized gradient.

**Why standard-eps now generalizes (the real improvement, not aesthetics):** eps = 1e-12 was fragile
— it only works while the un-normalized gradient stays above 1e-12; a smaller scene or a different
design var would need 1e-16, etc. A log-parametrized, dimensionless objective keeps an O(1)-ish
gradient across parameter magnitudes and scenes, so the STANDARD eps = 1e-8 holds without per-scene
retuning (`lr` and `loss_tol` remain per-objective tunables — see below). The win is
robustness/generality, not a single config that works everywhere.

## v3.5 Ladder / slicing (post-spike — single PR)

- **S0 spike** (§v3.4) — DONE: verdict Fork B (both levers); FD-exact.
- **PR (this leaf) — SHIPPED:** a general `Normalized` CoDesignProblem wrapper (`new` /
  `with_residual_scale` with finite-positive input guards; `to_physical` / `to_normalized`; a
  `recommended_config` starting point [`lr = 0.05` in log-space, `loss_tol = 1e-15`, standard eps];
  and a bracketing `optimize(x0_physical, cfg)` that maps params/history back to physical units — the
  footgun-free entry point so a `log_space` wrapper can't be driven with an un-transformed `x0`).
  Gate `tests/trajectory_inverse_design.rs`: raw FD (rel 9.3e-9) + **normalized-gradient FD** in
  log-μ space (rel 8.8e-9, confirms the `1/L²` and `dμ/dp = μ` bookkeeping) + a `log_space = false`
  loss-scale-only FD + **a negative control** (raw target + standard eps does NOT recover, 80 iters)
  + recovery to **rel 4.72e-7 with the standard eps** (asserts `cfg.eps == default`) + the make/break
  showcase (unchanged). **Retired** `SoftMaterialTrajectoryTarget::recommended_config` (eps band-aid);
  `OptConfig::eps` stays exposed (general knob) but nothing in cf-codesign overrides it. Single-step
  v1 `SoftMaterialTarget` path + gate UNTOUCHED. grade A. Pre-PR local ultra-review (35 agents, 28
  findings / 0 refuted / 0 correctness defects) — fixes folded in: input guards (the one medium),
  the bracketing `optimize`, the negative control, total `lower_bounds` (non-positive log-floor → −∞,
  no panic), and doc unifications.

## v3.6 Key files / pointers (v3)

- The wrapper + targets: `tools/cf-codesign/src/lib.rs`.
- Adam update (the eps mechanism): `sim/L0/ml-chassis/src/optimizer.rs:244` (`lr·m̂/(√v̂ + eps)`).
- v2 gate to rewrite onto the standard eps: `tools/cf-codesign/tests/trajectory_inverse_design.rs`.

---

# The POLICY half — open-loop control inverse design (opened 2026-06-13)

The DESIGN half (v1/v2/v3) optimizes a soft MATERIAL μ. The mission's outer loop
differentiates w.r.t. *both* design AND policy parameters; this section closes
the policy half: optimize the **control inputs** applied each step so a coupled
soft↔rigid trajectory hits a target. Keystone-side recon:
`docs/keystone/control_gradient_recon.md`.

## P.1 What the substrate now provides

A new keystone primitive `StaggeredCoupling::coupled_trajectory_control_gradient`
(`sim/L1/coupling`, the first leaf of this arc — PR-A): roll the coupled
system forward applying a per-step vertical control force to the platen, on ONE
chassis tape, then a single `tape.backward(z_N)` gives `∂z_N/∂u_k` for EVERY
control input. The control force adds to the platen's `xfrc_applied`, so
`∂vz'/∂u_k = +Δt/m` — it rides the same rigid carry as the contact reaction (a
3-parent `VzControlCarryVjp`), and the rest of the per-step tape is identical to
the material gradient. **S0 spike: machine-exact (~1e-11) vs the independent
re-rolled coupled-FD oracle for all controls; open-loop confirmed tractable.**

## P.2 Design — `ControlScheduleTarget`

`n_params = n_steps`; `evaluate(controls)` builds a fresh coupling (the `&mut`
rollout) and reads `(z_N, [∂z_N/∂u_k])` in ONE backward; loss `½(z_N − z*)²`,
gradient `residual · ∂z_N/∂u`. The soft material is held fixed (joint design+policy
is a follow-on). `lower_bounds = None` — control forces are SIGNED.

**Under-determination (honest):** N controls for one scalar target ⇒ the inverse
is under-determined (and the LAST control has zero effect on `z_N` — its velocity
bump never integrates into a height). The objective is **behavior recovery** (hit
the target platen height), the natural open-loop trajectory-opt framing, not
unique-parameter recovery. All `∂z_N/∂u_k ≥ 0`, so the loss is well-behaved.

## P.3 Conditioning — the SIGNED-parameter case (the real difference from v3)

`z_N` is a position so `∂z_N/∂u_k ~ 1e-5` and the raw loss gradient (`~3e-10`) is
below Adam's standard `eps = 1e-8` — the same scaling smell v3 diagnosed. But a
control force is **signed**, so the v3 **log-space lever does NOT apply** (it needs
positive params). The fix is the `Normalized` **`loss_scale` lever ALONE**
(dimensionless residual `1/L²`) + a control-appropriate `lr` (an O(1)-newton step,
not the default physical-μ step). **Measured: loss_scale-only converges to
|z_N − target| ~1e-13** (the v3 worry that residual-norm alone tops out ~1e-6 did
NOT bite here — the gradient stays above eps until the residual is ~1e-12 with a
large enough loss_scale; `recommended_normalized(1e-3)` → loss_scale 1e6). A
negative-control gate confirms normalization is load-bearing (same lr, raw target
stalls at ~7e-6). `recommended_normalized(residual_scale)` returns the wrapper;
the caller sets `lr ≈ 0.02` explicitly (documented, not band-aided).

## P.4 Ladder / slicing

- **PR-A** (sim-coupling): `coupled_trajectory_control_gradient` +
  `coupled_trajectory_control_z` oracle + `VzControlCarryVjp` + FD gate. grade A.
- **PR-B** (cf-codesign): `ControlScheduleTarget` + inverse-design gate (loss-FD,
  behavior recovery, negative control) + worked example + the bundled
  `Normalized::evaluate` `is_finite` hardening (deferred from the v3 review).
  grade A.

## P.5 Validation gate (policy)

`tools/cf-codesign/tests/control_inverse_design.rs`:
1. `control_loss_gradient_matches_fd` — `residual · ∂z_N/∂u` vs central FD of the
   loss, per control input (~1e-11).
2. `inverse_design_recovers_target_behavior` — from `u = 0`, recover a schedule
   driving `z_N` to a reference behavior (|z_N − target| 9e-14, 249 iters,
   standard eps).
3. `normalization_is_load_bearing` — raw target (same lr, no `Normalized`) does
   NOT reach the target (the load-bearing negative control).

+ worked `examples/control_inverse_design.rs` (the recovered schedule is uniform
with the no-effect final control left at 0 — a clean signature of the rigid
carry).

## P.6 Next

- Closed-loop feedback policy (`u_k = π_θ(state_k)`, differentiate the policy's
  state dependence — a small extra `state_k → u_k` chain on the same tape).
- Joint design+policy (both the μ leaf and the control leaves on one tape, one
  backward — the mission's "both" in one outer loop).
- Then lattice-as-design-var (mission #5), system-ID, the capstone exo loop.

