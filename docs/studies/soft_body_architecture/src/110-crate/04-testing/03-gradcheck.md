# Differentiability tests — gradcheck

Gradcheck is the strongest correctness test in the crate, per [parent Ch 04 claim 1](../04-testing.md): a solver that agrees with finite differences to 5–6 digits is *by construction* consistent between forward and reverse — the same energy gets differentiated analytically and numerically, and the two match. A unit test can check `barrier(d)`; only gradcheck catches a sign error in `d/dd barrier(d)`. A regression against [MuJoCo flex](01-regression.md) compares scalar outputs; only gradcheck compares the *gradient of* those outputs against an independent oracle. And the determinism-in-θ contract the [γ-locked `ForwardMap` from Part 10 Ch 00](../../100-optimization/00-forward.md) commits to is directly testable at the `ForwardMap::evaluate` boundary — the place where that contract is load-bearing for the rest of Part 10.

This sub-leaf names the three tests that compose the gradcheck harness: the `ForwardMap`-level determinism-in-θ check (§01), the FD-vs-IFT analytic-gradient agreement (§02), and the CPU-vs-GPU 5-digit regression (§03). All three run on every PR that touches [`autograd/`](../00-module-layout/07-autograd.md), [`material/`](../00-module-layout/00-material.md), [`contact/`](../00-module-layout/03-contact.md), or [`solver/`](../00-module-layout/04-solver.md), as specified by the [parent Ch 04 spine](../04-testing.md).

## §01 Determinism-in-θ — the `ForwardMap` contract made operational

The [γ-locked `ForwardMap` trait](../../100-optimization/00-forward.md) carries an invariant in its docstring: **repeated calls to `evaluate(theta, tape)` at the same `theta` produce the same [`RewardBreakdown`](../../100-optimization/00-forward.md) modulo cache state — no stochastic sim parameters update between calls.** This contract is what keeps [Part 10 Ch 02's BayesOpt GP](../../100-optimization/02-bayesopt/00-gp.md) cached training data valid under [Part 10 Ch 05's residual-GP online updates](../../100-optimization/05-sim-to-real/01-online.md). A solver drift that silently broke determinism-in-θ would invalidate both downstream paths without surfacing at the solver's own test suite.

The operational test is:

```rust
#[test]
fn forward_map_is_deterministic_in_theta() {
    let mut scene = SoftScene::canonical_cavity_probe();
    let mut fm   = scene.forward_map();
    let theta    = scene.default_theta();

    let mut tape_a = Tape::new();
    let (r_a, _)  = fm.evaluate(&theta, &mut tape_a);

    // fresh tape, same theta, same scene — no intervening mutation of fm
    let mut tape_b = Tape::new();
    let (r_b, _)  = fm.evaluate(&theta, &mut tape_b);

    assert_reward_breakdown_bit_equal(&r_a, &r_b);
}
```

`assert_reward_breakdown_bit_equal` compares every field of `RewardBreakdown` at the bit-pattern level — no tolerance, no $\epsilon$. Same θ, same RewardBreakdown, bit for bit. If the test fails, the breakage is in one of three places: (a) a trait impl on the forward path leaks non-determinism (fix per [unit-test §00's per-trait determinism contracts](00-unit.md)); (b) the solver's Newton-iteration state picks up a timestamp or counter (the [`autograd/` claim 3](../00-module-layout/07-autograd.md) commitment was violated); (c) a cache-state variable leaked across calls in a way the "modulo cache state" exemption does not cover.

Exemption semantics matter. The *"modulo cache state"* phrase means the test can validly change the cached `Llt<f64>` factor (first call factors, second call re-uses) without changing the `RewardBreakdown`. What it does *not* mean is a free pass for any caching that changes results — if the second call returns a subtly different value because a previous call seeded an incremental approximation, the exemption does not apply and the test fails.

The determinism check runs fast — two forward evaluations on a ~100-tet scene complete in under a second — so it runs on every PR, not a weekly-CI slice. A [parent Ch 04](../04-testing.md) PR that renames `RewardBreakdown`'s fields or adds a new one requires the assertion helper to be updated, which is an expected cross-crate edit.

## §02 FD vs IFT — the analytic-gradient agreement

The core gradcheck — the one every soft-body paper means when it says "we gradchecked" — runs the [Differentiable::fd_wrapper from Ch 01 core](../01-traits/00-core.md) alongside the [Part 6 Ch 02 IFT adjoint](../../60-differentiability/02-implicit-function.md) and asserts 5–6-digit agreement on each component. Concretely:

```rust
#[test]
fn ift_matches_central_difference_to_5_digits() {
    let mut scene = SoftScene::hundred_tet_neo_hookean_cube();
    let mut fm    = scene.forward_map();
    let theta     = scene.default_theta();

    // forward evaluates; tape records factor-on-tape
    let mut tape  = Tape::new();
    let (r, _)    = fm.evaluate(&theta, &mut tape);

    // IFT adjoint: one back-substitution on the cached Llt<f64>,
    // composed against the current reward weights into a scalar-gradient
    let (grad_ift, estimate) = fm.gradient(&theta, &tape);
    assert!(matches!(estimate, GradientEstimate::Exact));

    // central difference on the composed scalar reward, h near the
    // catastrophic-cancellation optimum for double precision
    let weights = scene.default_weights();
    let fd      = central_difference(&mut fm, &theta, &weights, 1e-6);

    assert_rel_err!(grad_ift, fd, rel_tol = 1e-5);
}
```

Two substantive points the test embeds:

**The faer `Llt<f64>` factor-on-tape is what makes IFT cheap.** [Part 5 Ch 00 Claim 3](../../50-time-integration/00-backward-euler.md) stores the SPD factorization as a first-class object on the tape; [Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md) retrieves it and applies `factor.solve_in_place` to $-\partial r / \partial \theta$. A test that finds IFT disagreeing with FD while the factor is present on the tape is isolating a bug to either the adjoint registration ([`autograd/` IFT pass](../00-module-layout/07-autograd.md)) or the Jacobian assembly ([`autograd/` FEM VJP](../00-module-layout/07-autograd.md)); neither an FD harness bug nor a line-search diagnostic, because neither of those affect the cached factor or the registered VJPs.

**Rel-tol 1e-5 is the 5-digit bar; 6 digits is possible on simpler scenes.** Neo-Hookean + IPC on a 100-tet cube with clean double-precision assembly passes at 5 digits on most Gauss-point choices; 6 digits requires an `h` chosen near the catastrophic-cancellation optimum (roughly $h \sim \epsilon^{1/3}$ for central difference, $\sim 10^{-5}$ for double precision), which the harness picks adaptively per component. [Parent Ch 04](../04-testing.md)'s "5–6 digits" bar acknowledges both cases; a PR that regresses a previously-6-digit component to 4 digits is flagged as suspicious even if it is nominally above the 5-digit bar.

## §03 CPU-vs-GPU regression — the Phase E adjoint oracle

[Phase E's GPU port](../03-build-order.md#the-committed-order) ports the solver's inner loop onto wgpu and re-registers `autograd/`'s VJPs against the GPU-side tape. The CPU path remains the oracle: on every PR touching `gpu/` or any module whose VJPs feed it, gradcheck runs the same scene through both `CpuNewtonSolver<Tape = CpuTape>` and `GpuNewtonSolver<Tape = GpuTape>` (the [two concrete `Solver` impls from Ch 01 core](../01-traits/00-core.md)) and asserts 5-digit agreement on the composed gradient:

```rust
#[test]
fn gpu_gradient_matches_cpu_to_5_digits() {
    let mut scene = SoftScene::small_gradcheck_scene();
    let theta     = scene.default_theta();
    let weights   = scene.default_weights();

    let grad_cpu  = forward_and_gradient_cpu(&mut scene, &theta, &weights);
    let grad_gpu  = forward_and_gradient_gpu(&mut scene, &theta, &weights);

    assert_rel_err!(grad_cpu, grad_gpu, rel_tol = 1e-5);
}
```

The [associated `Tape` type on `Solver`](../01-traits/00-core.md) is what makes this test well-typed — CPU and GPU solvers each carry their own tape representation, each registers its own VJPs, and the test closes both loops end-to-end rather than testing kernel outputs in isolation. This is the correctness foundation [Phase E's build-order justification](../03-build-order.md#the-committed-order) rests on: "Without that oracle, a GPU gradient bug would surface as 'the optimizer is converging to something different' — weeks of debug time instead of a minute."

Scene size for §03 is capped by GPU warm-up cost in CI — the test runs on a scene small enough that the gradient difference is not dominated by CPU-vs-GPU floating-point accumulation order on dense sums. Reduction order is part of the test: a GPU kernel that sums in a different order from CPU will agree to ~5 digits but not to 8 or more, and the 5-digit bar is tuned around that reality.

## `GradientEstimate` — `Exact` is what gradcheck asserts on; `Noisy` is variance-checked

The [Part 10 Ch 00 95/5 split](../../100-optimization/00-forward.md) commits that ≈95% of gradient calls return `GradientEstimate::Exact` (IFT on the cached factor), and ≈5% return `GradientEstimate::Noisy { variance }` (FD-wrapped through a topology-crossing edit per [Part 6 Ch 05 + Part 7 Ch 04](../../60-differentiability/05-diff-meshing.md)). The gradcheck harness has different assertions for the two:

- **`Exact` path** — §02's 5-digit FD-vs-IFT test is the check. A PR that silently converts an `Exact` return to `Noisy` is a behavior change that fails a companion assertion: `assert!(matches!(estimate, GradientEstimate::Exact))` on scenes the harness expects to be smooth.
- **`Noisy { variance }` path** — the check is that the reported variance is a *calibrated upper bound* on the true FD noise on the scene. A noise report of $\sigma^2 = 10^{-3}$ on a scene whose true central-difference noise is $\sigma^2 = 10^{-5}$ is over-reporting (conservative, passes); under-reporting (variance smaller than the FD noise the wrapper actually produced) fails. Downstream optimizers in [Part 10 Ch 02's noise-tolerant acquisition](../../100-optimization/02-bayesopt.md) read the variance and weight samples accordingly; an under-reported variance is silently wrong data to the optimizer.

Both checks keep the 95/5 contract operationally honest. Scenes tagged as "topology-crossing" in the test fixture exercise the `Noisy` path; all other scenes must produce `Exact`, and the composed-gradient 5-digit agreement holds.

## Phase landing

Each gradcheck sub-test activates at a specific phase in the [build-order commitment](../03-build-order.md#the-committed-order). Activation is non-negotiable — gradcheck is named explicitly in each Phase's deliverable list:

- **Phase A.** Isolated-module gradcheck: [`material/`](../00-module-layout/00-material.md) and [`mesh/`](../00-module-layout/02-mesh.md) each ship with their own FD-vs-analytic-gradient suite before consumer modules compile. No `ForwardMap` yet (no `readout/`, no `solver/`); the tests run at the `Material::first_piola` and the mesh-geometry-Jacobian layers individually.
- **Phase B.** First `ForwardMap`-level gradcheck: 100-tet neo-Hookean cube at static equilibrium, elastic-only, 5-digit FD-vs-IFT agreement. No contact. No determinism-in-θ test yet (the full `ForwardMap` contract lands at Phase D); the contract on the solver-in-isolation does hold, and `solver/`'s own determinism test from [unit §00](00-unit.md) covers the lower layer.
- **Phase C.** Contact gradcheck on a reduced-dimensional scene where central differences are tractable per [parent build-order Phase C commentary](../03-build-order.md#the-committed-order). The FD wrapper is not the scene's gradient oracle — the 5-digit FD-vs-analytic oracle is — but a reduced-dim projection keeps the FD cost bounded.
- **Phase D.** `ForwardMap` lands end-to-end on CPU per [parent build-order Phase D commentary](../03-build-order.md#the-committed-order); §01's determinism-in-θ test activates here, and the composed-reward 5-digit gradcheck runs on the first working synthetic 2-parameter optimization. This is the phase where all three §01–§03 assertions (minus §03's GPU half) begin running together.
- **Phase E.** §03 activates: `Solver`'s CpuTape/GpuTape pairing is exercised by the paired-solver harness on every PR.
- **Phase H.** [Additional `Material` and `Element` impls](../03-build-order.md#the-committed-order) (Tet10, Mooney-Rivlin, Ogden, Prony viscoelasticity, HGO anisotropy) each extend gradcheck with their own fixtures before landing. Phase H is additive; the existing scenes do not stop running, and the new scenes gate their own new trait impls.

## What gradcheck does not test

- **Physical-print residual validation.** [Part 10 Ch 05's `SimToRealCorrection::correct`](../../100-optimization/05-sim-to-real.md) consumes [`MeasurementReport`](../../100-optimization/05-sim-to-real.md) and produces corrected `RewardBreakdown` values; validating that the corrected output matches real-world printed artifacts on held-out θ is a different test class with a different oracle (measured curves, not analytic gradients). Per [Part 10 Ch 06](../../100-optimization/06-full-loop.md)'s phase-staged build and [Part 12 Ch 07's open-questions housing](../../120-roadmap/07-open-questions.md), the `MeasurementReport`-ingestion test harness is post-Phase-I. Gradcheck binds on sim-side ground truth only — FD against IFT, CPU against GPU. The physical-print-loop residual oracle ships as its own suite post-Phase-I rather than being absorbed into this one.
- **Reward-composition weight gradients.** The composed-scalar gradient is checked; weight-gradient sensitivity to individual $w_i$ is a [Part 10 Ch 03 preference-learning](../../100-optimization/03-preference.md) concern, tested against the preference-GP's Laplace-mode convergence rather than against FD. Different oracle, different test harness.
- **Adjoint-method time-trajectory tests.** The [Part 6 Ch 03 time adjoint](../../60-differentiability/03-time-adjoint.md) is exercised by its own phase-landing tests; the static-equilibrium IFT gradcheck in §02 does not span time. A Phase B–C gradcheck does not tell you whether the time-adjoint loop is correct on a rolled-out trajectory.
- **FD gradients through topology-crossing SDF edits at 5 digits.** The [open-problem `Noisy { variance }` path](../../60-differentiability/05-diff-meshing.md) is variance-checked, not 5-digit-checked, because the FD wrapper's noise floor is bounded away from 5-digit agreement by construction. The [parent Ch 05 "open-problem" framing](../../60-differentiability/05-diff-meshing.md) is that clean gradients through topology changes are not yet available; gradcheck does not pretend otherwise.

## What this sub-leaf commits downstream

- **[Every Phase deliverable](../03-build-order.md#the-committed-order)** cites this suite by phase: Phase A isolated-module gradcheck, Phase B 100-tet neo-Hookean 5-digit FD-vs-IFT, Phase C contact reduced-dim, Phase D `ForwardMap` end-to-end + determinism-in-θ, Phase E CPU-vs-GPU 5-digit regression, Phase H each new `Material`/`Element` impl. Phase A's start condition cannot be met without Phase A's gradcheck suite passing, and each subsequent phase's acceptance cannot be signed off without its gradcheck row.
- **[The γ-locked `ForwardMap` determinism-in-θ contract](../../100-optimization/00-forward.md)** becomes merge-blocking here; a PR that breaks determinism-in-θ fails §01 and fails CI, independent of whether downstream tests would notice the drift.
- **[The 95/5 split](../../100-optimization/00-forward.md)** is enforced here via the `Exact`/`Noisy` assertion pair; a silent move outside that envelope (e.g., a change that causes routine parameter edits to return `Noisy`) fails on the assertion that smooth scenes return `Exact`.
- **[Phase E's GPU correctness oracle](../03-build-order.md#the-committed-order)** is this suite's §03; Phase E's shipping criterion includes §03 going green on every PR touching `gpu/` or any VJP-registering module.
