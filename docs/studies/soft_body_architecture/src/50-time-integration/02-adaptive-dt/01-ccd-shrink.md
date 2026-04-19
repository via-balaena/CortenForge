# CCD-driven shrinking

The CCD path is `sim-soft`'s primary adaptive-timestep mechanism. It is triggered by a structured failure signal from the [Ch 00 §02 line search](../00-backward-euler/02-line-search.md), remediates by halving $\Delta t$ and restarting the step from the previous step-boundary state, and commits to deterministic replayability — every shrink is traced in the diagnostic log to its signal source (`armijo-stuck` or `ccd-clipped-to-zero`) and, for CCD-clipped cases, to the specific contact pair that forced the clip. The machinery is deliberately simple: a binary trigger on a structured signal, no control-theoretic tuning gains, no smooth error norm.

## The signal contract

[Line search](../00-backward-euler/02-line-search.md) returns a triple $(\alpha, \text{success}, \text{reason})$ with $\text{reason} \in \{\text{ok}, \text{armijo-stuck}, \text{ccd-clipped-to-zero}\}$. The Newton loop aggregates these across its inner iterations: any non-`ok` return from line search marks the entire step failed and the outer loop reacts.

Two trigger conditions and one non-trigger:

- **`armijo-stuck`** — backtracking exhausted its retry budget without satisfying the sufficient-decrease condition on $U_n$. The current iterate sits outside the basin of attraction at this $\Delta t$; no Newton step reduces $U_n$ within the contact barrier's feasibility region. Adaptive response: halve $\Delta t$ and restart.
- **`ccd-clipped-to-zero`** — CCD returned $\text{toi} < \epsilon_\text{ccd}$, meaning the full Newton step was heading into the barrier essentially immediately from the current iterate. The Newton direction and the contact geometry are locally incompatible at this $\Delta t$. Adaptive response: halve $\Delta t$ and restart.
- **`ok`** at every inner Newton iteration followed by $\|r_n\| < \tau_\text{Newton}$ on convergence — the step succeeded. Adaptive response: increment the success counter; on reaching the grow threshold, consider doubling $\Delta t$ for the next step.

Both failure signals remediate by halving, so the outer loop treats them as a single trigger. Diagnostic logs preserve the actual signal so post-hoc analysis can distinguish line-search descent failures from geometry-incompatibility failures. The distinction is diagnostic, not architectural: the retry mechanics are identical.

## Halve-and-restart mechanics

On trigger, the outer loop:

1. **Discards** the failed Newton iteration's state — the partial iterate $x^{(k)}$, any line-search intermediate, and any autograd tape entries recorded during the failed attempt.
2. **Halves** the proposed timestep: $\Delta t \leftarrow \Delta t / 2$. If $\Delta t$ has reached the user-configured minimum $\Delta t_\text{min}$, the step fails permanently and the outer loop raises an error rather than shrinking further.
3. **Restarts** the step from $(x_{n-1}, v_{n-1})$ — the *converged* state at the end of the previous successful step, not the current iterate's mid-step value.
4. **Re-runs** the Newton loop with the halved $\Delta t$. On success, only this successful retry is recorded on the autograd tape — the failed attempts do not leave tape entries.
5. **Resets** the success counter to zero so the grow-after-$N$-successes machinery restarts from scratch.

The restart-from-$(x_{n-1}, v_{n-1})$ commitment is what makes shrinks replayable: a debug run can re-execute a failed step at fixed $\Delta t$ to reproduce the trigger, then halve and re-execute the retry to reproduce the successful step. No intermediate Newton state is load-bearing; only the step-boundary state is.

## Grow policy

After each successful step the outer loop increments a success counter. When the counter reaches $N$ (default $8$), $\Delta t$ doubles for the next step, capped at the user-configured target $\Delta t_\text{target}$ (typically on the order of $16$ ms for experience-mode, $1$ ms for design-mode per the [Ch 02 spine](../02-adaptive-dt.md)). Any shrink resets the counter.

The $N = 8$ default is a practitioner heuristic — not derived from convergence theory. Smaller $N$ grows faster but increases the chance of overshoot into the CCD-clipping regime; larger $N$ is more conservative but leaves throughput on the table when the system enters a quiet phase. `sim-soft` exposes $N$ in the solver config for applications where the default is wrong.

## Why the halve/grow asymmetry

Halve-on-failure costs one wasted failed Newton attempt at the larger $\Delta t$ plus the retry at $\Delta t / 2$, a bounded overhead amortized against the subsequent successful step.

Grow-after-$N$-successes has a long tail. If the doubled $\Delta t$ overshoots into the CCD-clipping regime, the doubled step fails, halves back to the original $\Delta t$, the success counter resets, and it takes another $N$ successes before growing is retried. A single bad grow costs one full Newton-loop failure plus the halving-recovery *and* resets the grow attempt cadence.

Because halve-on-failure is bounded and grow-on-success has a long tail, the prudent asymmetric policy is to fail fast on the halving side and slowly on the growing side. The same qualitative shape — cautious growth, aggressive shrinkage — appears in the PI step controllers of [Gustafsson-Söderlind 1988](../../appendices/00-references/05-time-integration.md#gustafsson-soderlind-1988) and the broader adaptive-RK safety-factor tradition covered in [Hairer-Wanner's ODE textbooks](../../appendices/00-references/05-time-integration.md#hairer-wanner), where the asymmetry is tuned into the proportional and integral gains of a controller acting on a smooth error norm. `sim-soft` uses a binary specialization because the failure mode it reacts to is categorical (did CCD clip? yes/no) rather than a smooth error norm, so there is no gradient for a PI controller to act on.

## Discrete decisions, non-smooth on purpose

The adaptive layer is not a gradient-informed optimizer. It computes no error-norm signal, feeds nothing into a continuous controller, and exposes no tuning gains beyond $\Delta t_\text{target}$, $\Delta t_\text{min}$, and $N$. The trigger is binary, the response is binary.

This simplicity is what makes the shrink/grow decision cleanly non-differentiable. The autograd treatment from the [Ch 02 spine](../02-adaptive-dt.md) commits to this: shrink decisions are discrete events, the tape records only successful retries, the FD wrapper with [`GradientEstimate::Noisy { variance }`](../../60-differentiability/05-diff-meshing.md) handles the case where a design parameter meaningfully affects which steps trigger shrinks. A smooth PI controller would smear the boundary — the shrink decision would become a continuous threshold on a controlled quantity, and the autograd tape would need to differentiate through the controller's internal state across the shrink event. For the Phase D–G commitment, the binary scheme is both simpler and honest about what is non-differentiable.

## Rejected alternatives in detail

Two smooth alternatives the spine names, elaborated:

**PI/PID step controllers on a computed error norm.** [Gustafsson-Söderlind 1988](../../appendices/00-references/05-time-integration.md#gustafsson-soderlind-1988) introduced the proportional-integral framework for ODE step-size selection, and [Söderlind 2002](../../appendices/00-references/05-time-integration.md#soderlind-2002) extended it to a full control-theoretic framework with stability guarantees for stiff solvers. In either form, a PI controller requires a controlled quantity: some error norm $\varepsilon$ that the controller drives toward a setpoint. For the elasticity-only solver, $\varepsilon$ could be a local-truncation-error estimate from a pair of backward-Euler evaluations at $\Delta t$ and $\Delta t / 2$. But for `sim-soft` with IPC, the dominant failure mode is not elasticity-side truncation — it is a contact pair approaching the barrier, a categorical event with no natural smoothing. A PI controller on elasticity-error would not see the contact failure until after it happened, and even then the error norm is poorly defined because the iterate is outside the feasible set. Controller gains would need to be calibrated jointly with contact density, friction coefficient $\mu_c$, and barrier width $\hat d$ in ways that are hard to generalize from a handful of canonical problems. Rejected.

**Runge-Kutta-Fehlberg error estimation.** [Fehlberg 1969](../../appendices/00-references/05-time-integration.md#fehlberg-1969) introduced the embedded-RK4(5) method: two Runge-Kutta solutions share stage evaluations, and their difference estimates local truncation error cheaply. This is the classical non-stiff ODE adaptive-stepping machinery. For `sim-soft`, the framework does not transfer. RKF is explicit, which fails the [Ch 00 §00 unconditional-stability requirement](../00-backward-euler/00-why-implicit.md); and the contact-barrier regime's accuracy limit is CCD topology, not elasticity-side smoothness. An embedded-error estimate for a backward-Euler system could in principle be constructed by step-doubling — one step at $\Delta t$ plus two half-steps at $\Delta t/2$ for comparison — but that triples per-step Newton-loop work, when the adaptive layer's whole point is to economize on that. Rejected.

## What this commits

- The CCD path is the primary adaptive-timestep mechanism; the [energy monitor](00-energy.md) is strictly the backstop
- Trigger is a binary signal from line search (`armijo-stuck` or `ccd-clipped-to-zero`); response is halve-and-restart
- Grow policy is after $N$ successive successful steps, default $N = 8$, tunable, capped at $\Delta t_\text{target}$
- Each shrink is deterministic and replayable — the restart-from-$(x_{n-1}, v_{n-1})$ commit preserves step-boundary state as the only load-bearing quantity
- Only successful retries are recorded on the autograd tape; failed attempts leave no trace
- Rejected alternatives (PI/PID controllers, RKF error estimation) are named with the specific mismatch to the contact-barrier regime that disqualifies them from `sim-soft`, not by blanket "does not apply"
