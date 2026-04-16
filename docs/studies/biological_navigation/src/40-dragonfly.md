# Regime 4: Dragonfly — Predictive Guidance

**Re ~ 10⁵-10⁷ — The Predictive Inertial Regime**

## The Physics

Full inertial regime. Viscosity contributes to drag but not meaningfully to the locomotion control problem. The fluid generates persistent wakes and vortices. The key physical constraint is latency: at these speeds, the time for a reactive signal to propagate through the nervous system and execute a motor command is comparable to or longer than the time in which the target moves a significant fraction of the control distance. A pure reactive strategy fails — by the time the correction executes, it is already wrong.

This is the latency gap regime: you are moving fast enough to lose the distributed local computation advantage (the arm can't adapt fast enough), but not fast enough to generate the vortex-lift control authority that defines the next regime up. It is the most computationally demanding regime for a biological or engineered controller.

## What the Dragonfly Does

Dragonflies achieve a prey capture success rate of 90-95% in the wild — the highest of any known predator. They do this while intercepting aerobatic prey in mid-air at speeds up to several meters per second, in turbulent air, with a nervous system containing roughly 1 million neurons total.

The mechanism was established by Mischiati et al. (Nature, 2015): dragonfly interception steering is driven by forward and inverse internal models of the dragonfly's own body dynamics and of the prey's predicted trajectory. Predictive rotations of the head continuously track the prey's angular position. The head-body angle thereby established guides systematic rotations of the body to align with the prey's predicted flight path. Model-driven control underlies the bulk of maneuvers; reactive control is reserved specifically for unexpected prey movements.

The guidance law was characterized by Brighton et al. (PNAS, 2017, working with peregrine falcons but confirmed analogously for dragonflies): proportional navigation (PN). Under PN, turning rate is commanded proportional to the angular rate of the line-of-sight to target:

$$
\omega_{\text{commanded}} = N \times \frac{d\lambda}{dt}
$$

where λ is the line-of-sight angle and N is the navigation constant (feedback gain). This is the same guidance law used by most visually guided missiles. For dragonflies, N ~ 3, which coincides with the classical linear-quadratic optimal guidance result: PN with N = 3 minimizes control effort to intercept a non-maneuvering target.

Before takeoff, the dragonfly performs a critical pre-selection step: it assesses whether the prey's angular size and velocity co-vary within a privileged range, and times its takeoff to predict when the prey will cross its zenith. The behavioral decision embeds the computational constraints — the dragonfly only pursues prey it has pre-verified it can intercept given its own body dynamics.

The minimum sensory requirement is also remarkable: two local observables — vertical wind acceleration and torque (body rotation rate) — are sufficient to implement the PN guidance law. Global knowledge of the flow field is not required.

## The X-Encoding Principle

**Principle 7:** In the latency-gap regime, the X-encoder must operate predictively. It cannot react to observed drift in the circuit's state because the signal propagation latency is too long for reactive correction to be useful. Instead, it maintains a forward model of the circuit's own relaxation dynamics — predicting where the system will be at time t+Δ and injecting X at the predicted future state rather than the current observed state. The injection leads the relaxation rather than chasing it.

*Applied to thermodynamic circuits:* A forward model of the Langevin dynamics, calibrated from observed circuit statistics, predicts the evolution of the energy landscape for the next several time steps. X-injection is computed against the predicted state. When prediction error exceeds a threshold (unexpected stochastic events), the system falls back to reactive injection temporarily, then returns to model-driven control.

**Principle 8:** Pre-selection and operating regime commitment. Before entering high-throughput operation, the X-encoder should verify that the circuit's current noise profile and relaxation rate are within the regime for which the encoding strategy was designed (analogous to the dragonfly's pre-takeoff assessment). Attempting to encode outside the designed regime is the primary failure mode.

**Principle 9:** Minimum observables sufficiency. Two local scalar cues are sufficient for full trajectory guidance. The X-encoder instrumentation can therefore be minimal: local energy gradient and local curvature (or equivalent observable pair) fed into a PN-style feedback law. This is a tractable instrumentation problem, not a global state estimation problem.

## Hypothesis Given Current Results

Principles 7 (predictive forward model) and 9 (minimum observables) are **statistical-mechanical** in character — they ask how to use noise statistics to calibrate a model, and how few observables suffice for control. Based on the pattern from Chapters 1–3, these should transfer to the Langevin domain.

Principle 8 (pre-selection / regime commitment) is also stat-mech: it's a gate check on operating conditions before committing to a strategy. This should transfer.

**Prediction:** P7 and P9 will validate. The PN guidance law with N ≈ 3 should emerge from Langevin dynamics. Minimum observables (2 local cues) should suffice. These are noise-exploitation strategies, which is what the Langevin framework handles natively.

**Status:** Not yet tested. Infrastructure exists (`experiment_4.rs`).

## Experiment 4 — Dragonfly PN Guidance in Langevin Noise

Implement proportional navigation with N as a free parameter in a simulated Langevin particle navigating a 2D energy landscape toward a target basin. Vary N across [1, 5] and measure convergence speed and convergence fidelity as a function of noise level. Verify the N ~ 3 optimum and characterize its robustness to noise floor variations. Extend to the forward-model variant: precompute the target basin's future position using the Langevin drift term, and measure the improvement in convergence.

> **Platform readiness:** High — [`ThermCircuitEnv`](80-therm-circuit-env.md) builder, `DoubleWellPotential`, `LangevinThermostat`, and all 8 RL/optimization algorithms exist.
>
> **Status:** Plumbing validated — CEM learns state-dependent temperature control. The full PN guidance experiment (N sweep, noise sweep, forward-model variant) is not yet started.
>
> **Code:** [`sim/L0/therm-env/tests/experiment_4.rs`](../../../sim/L0/therm-env/tests/experiment_4.rs)

## Key References

- Mischiati, M. et al. "Internal Models Direct Dragonfly Interception Steering." *Nature* 517 (2015)
- Brighton, C.H. et al. "Terminal Attack Trajectories of Peregrine Falcons are Described by the Proportional Navigation Guidance Law of Missiles." *PNAS* 114 (2017)
- Mills, R. et al. "Physics-Based Simulations of Aerial Attacks by Peregrine Falcons Reveal that Stooping at High Speed Maximizes Catch Success." *PLOS Computational Biology* 14 (2018)
- Combes, S.A. "Neuroscience: Dragonflies Predict and Plan Their Hunts." *Nature* 517 (2015)
- Gonzalez-Bellido, P.T. et al. "Eight Pairs of Descending Visual Neurons in the Dragonfly Give Wing Motor Centers Accurate Population Vector of Prey Direction." *PNAS* 110 (2013)
