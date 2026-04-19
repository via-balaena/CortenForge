# Energy conservation monitors

A consistent physical system conserves total energy up to dissipation: mechanical energy lost to friction, viscoelastic relaxation, and any explicit damping terms should equal the change in kinetic plus physical-potential energy across the interval, minus the work done by external loads. An energy monitor tracks this ledger across timesteps and raises a trigger when the step-over-step residual exceeds a tolerance. In `sim-soft` it is the adaptive-timestep layer's *backstop*: a coarse invariant that is always true and never tight enough to drive production shrinking on its own. The [CCD-driven path](01-ccd-shrink.md) is the primary mechanism; the energy monitor exists to catch the failure modes that are invisible to CCD.

## The bookkeeping ledger

At each converged step the solver evaluates five scalars:

- **$U_n^\text{phys}$** — *physical* potential energy at $x_n$ (elastic strain energy + contact-barrier potential). Distinct from the step-optimization objective [Ch 00 §01 newton](../00-backward-euler/01-newton.md) minimizes, which adds the backward-Euler inertial quadratic $\tfrac{1}{2\Delta t^2}\|x - \hat x\|_M^2$; that quadratic is a time-discretization artifact, not mechanical energy, and is excluded from the monitor's ledger.
- **$T_n = \tfrac{1}{2} v_n^T M v_n$** — total kinetic energy
- **$D_n^\text{fric}$** — cumulative friction work, summed over contact pairs active during step $n$ with the per-pair, per-step increment discretized per [Part 4 Ch 02 §02 ipc-friction](../../40-contact/02-friction/02-ipc-friction.md) (smoothly blending the stick and slip regimes using the same lagged-state pair the force model uses)
- **$D_n^\text{visco}$** — cumulative viscoelastic dissipation, summed over elements and Prony modes with the per-mode rate and integration detailed in [Part 2 Ch 08 dissipation](../../20-materials/08-thermal-coupling/02-dissipation.md)
- **$E_n^\text{ext}$** — cumulative work done by external loads (prescribed boundary tractions, gravity)

The running invariant is

$$ \Delta_n = (U_n^\text{phys} + T_n) + (D_n^\text{fric} + D_n^\text{visco}) - (U_0^\text{phys} + T_0) - E_n^\text{ext}. $$

$\Delta_n$ should be zero up to step-wise Newton-convergence tolerance and the discretization error of the dissipation-rate integrals. The monitor flags when $|\Delta_n - \Delta_{n-1}| > \tau_\text{energy}$ — an over-the-step mismatch the dissipation terms do not account for.

## Why this is the backstop, not the primary path

Three noise sources prevent energy drift from driving production shrinking:

**Dissipation-discretization noise.** The $D^\text{fric}$ and $D^\text{visco}$ increments are themselves approximations with step-wise error. Friction-work integration over a backward-Euler step uses the same lagged-state pair from [Part 4 Ch 02 §02 ipc-friction](../../40-contact/02-friction/02-ipc-friction.md); Prony-mode dissipation inherits the time-discretization error of the mode-update equation from [Part 2 Ch 07 Prony](../../20-materials/07-viscoelastic/00-prony.md). These errors enter $\Delta_n$ as signed perturbations of bounded magnitude but with no physically meaningful direction, so $\Delta_n$ is noisy even on converged steps.

**Backward Euler's numerical damping is indistinguishable from physical dissipation at this level.** Implicit time integration damps oscillatory modes by design — this is the flip side of the unconditional stability that motivates [Ch 00 §00](../00-backward-euler/00-why-implicit.md)'s choice of the scheme. For a linear oscillatory mode of angular frequency $\omega$, the per-step amplitude ratio is $1/\sqrt{1 + (\omega \Delta t)^2}$, equal at small products to $1 - (\omega \Delta t)^2/2 + O((\omega\Delta t)^4)$ — a non-physical energy leakage that scales as $(\omega \Delta t)^2$ and depends on the current iterate's mode spectrum. The monitor cannot separate this numerical loss from a genuine dissipation-ledger miscount because both appear in $\Delta_n$ as slow drift with the same scaling structure.

**Newton-convergence tolerance contributes to the residual floor.** Newton's inner loop accepts a converged step at $\|r_n\| < \tau_\text{Newton}$, which near a local minimum of $U^\text{phys}_n$ translates to a $U^\text{phys}_n$ uncertainty of order $\tau_\text{Newton}^2 / \|H\|$, with $\|H\|$ the Hessian operator norm at the local minimum. In isolation this source is typically smaller than the first two on production `sim-soft` settings, but it is non-zero and compounds over a simulation.

The net effect: the monitor's floor — the sum of dissipation-discretization noise, numerical damping, and accumulated Newton-tolerance residual — is non-negligible at production `sim-soft` settings, and sits above the step-over-step drift signal a tighter invariant would need to resolve. In a nominal CCD-enabled run, a barrier-crossing or basin-escape event fires the [line search trigger](../00-backward-euler/02-line-search.md) before $|\Delta_n - \Delta_{n-1}|$ exceeds $\tau_\text{energy}$. The monitor is redundant whenever CCD is active and meaningful-only when CCD is disabled or a non-contact failure slips through.

## When the monitor activates

Three modes:

**Debug runs with safety layers disabled.** For bisecting a physics bug — narrowing whether a regression lives in the elastic-tangent assembly, the friction projection, the contact broad-phase, or the time integrator — developers can selectively disable the safety layers (contact barrier, CCD, friction) to isolate sub-solver components. With a safety layer off, the Newton step can make progress in states the disabled layer would otherwise have rejected, and the bug's signature shows as a conservation violation the monitor catches. The outer loop then falls back to halving $\Delta t$. This is the scenario the monitor exists for.

**Validation passes.** Regression runs against [Part 11 Ch 04 §01 flex baseline](../../110-crate/04-testing/01-regression.md) compare `sim-soft` against the MuJoCo flex oracle on canonical-problem cases. The energy monitor runs with a tight $\tau_\text{energy}$ and logs every violation — each logged violation is a candidate for follow-up investigation against the flex regression baseline. These violations are diagnostic data points for the test harness, not triggers for adaptive shrinking in validation runs.

**Constitutive-law and autograd-tape bug hunts.** A forward-path bug in a custom VJP from [Part 6 Ch 01](../../60-differentiability/01-custom-vjps.md), a miscoded per-element tangent $\mathbb{C}$ in [Part 3 Ch 00 Tet4 assembly](../../30-discretization/00-element-choice/00-tet4.md), or any other bug that breaks the elastic potential's frame-indifference or its conservation properties shows up as spurious energy gain or loss over steps. The monitor is the first-line detector; narrower tools (finite-difference gradcheck, frame-rotation tests) then localize the specific root cause.

## Interaction with the outer retry loop

When $|\Delta_n - \Delta_{n-1}| > \tau_\text{energy}$ the monitor emits the same `retry-requested` signal as the CCD path, and the outer loop halves $\Delta t$ and restarts the step from $(x_{n-1}, v_{n-1})$. The monitor does not introduce a new termination state; it shares the halve-and-restart machinery with [§01 ccd-shrink](01-ccd-shrink.md). Only the *signal source* differs, and `sim-soft`'s diagnostic logs record which source triggered the retry — so a debug run with safety layers disabled that exhibits a shrink cascade is traceable to the energy-monitor path specifically.

The autograd-tape treatment is identical: only the successful retry is recorded on the tape; failed attempts — whether CCD-detected or energy-detected — are discarded.

## What this commits

- Energy monitoring is the backstop, off by default in production `sim-soft` runs and on in debug and validation modes
- The $\Delta_n$ ledger's five-term accounting — $U_n^\text{phys}$, $T_n$, $D_n^\text{fric}$, $D_n^\text{visco}$, $E_n^\text{ext}$ — is the load-bearing invariant; the monitor's tolerance sits above the floor set by backward-Euler numerical damping, friction/viscoelastic discretization noise, and accumulated Newton-convergence residual
- The monitor's trigger path is the same halve-and-restart machinery as CCD-driven shrinking, so no extra solver state is introduced
- The monitor is valuable for bug-hunting (constitutive-law bugs, autograd-tape errors, missing-dissipation regressions) and validation-run instrumentation, not for production step control
