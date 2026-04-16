# Regime 1: E. coli — Topological Encoding

**Re < 1 — The Viscosity-Dominated Regime**

## The Physics

At Re < 1, the Navier-Stokes equations simplify to the Stokes equations — linear, time-reversible, with no inertial terms. The fluid has no memory. A swimmer moving forward and then backward through the exact same sequence of shapes returns to exactly its starting position. This is Purcell's scallop theorem (1977): in a time-reversible fluid, any reciprocal motion (one that looks the same played forward and backward) produces zero net displacement. It does not matter how fast or slow the motion is performed — speed is irrelevant.

The implication is profound: at low Re, you cannot encode information in the amplitude or timing of a perturbation. Amplitude scales out. Timing scales out. The only thing that survives is the topology of the motion sequence — whether it traces a closed loop in configuration space that encloses a nonzero area. This is geometric phase, formalized by Shapere and Wilczek (1989) using the language of gauge theory and fiber bundles.

## What E. coli Does

E. coli navigates chemical gradients (chemotaxis) at Re ~ 10⁻⁵ using the run-and-tumble strategy. Multiple flagellar motors rotate counter-clockwise to bundle the flagella into a helical propeller (a "run" — straight motion). When one or more motors switch to clockwise rotation, the bundle unbundles and the cell reorients randomly (a "tumble"). By extending runs when moving up a chemical gradient and shortening them when moving down, the cell executes a biased random walk toward attractants.

The critical insight: the CheY-P signaling molecule that controls motor switching follows Langevin dynamics — it has intrinsic stochastic fluctuations described by:

$$
\frac{dY}{dt} = -\frac{Y - Y_0}{\tau_Y} + \eta(t)
$$

where η(t) is Gaussian white noise. Crucially, signaling noise enhances chemotactic drift. An intermediate level of noise in the slow methylation dynamics improves gradient-climbing performance — not despite the noise, but because of it. The noise is not a contaminant to be suppressed; it is a functional component of the control strategy.

Furthermore, E. coli achieves signal amplification of more than 50-fold: a 2% change in receptor occupancy produces a 100% change in motor output. This nonlinear amplification emerges from the cooperative structure of the receptor-kinase complex operating near a phase transition — a bifurcation point at which sensitivity is maximized.

## The X-Encoding Principle

**Principle 1:** In the viscosity-dominated regime, amplitude and rate are irrelevant. Information must be encoded in the sequence structure — the topology — of the injection. A closed loop in the injection parameter space that encloses nonzero area produces net displacement in the output state; a reciprocal sequence produces nothing.

*Applied to thermodynamic circuits:* At very low throughput rates where the circuit is effectively at quasi-static equilibrium, X-encoding cannot rely on amplitude modulation. The injection must trace a non-reciprocal path in parameter space. The geometric phase associated with that path determines the encoding fidelity. This is a formal statement that connects X-encoding theory to the mathematics of holonomy and gauge theory.

**Principle 2:** Stochastic resonance is available. An optimal noise level enhances encoding fidelity by improving sensitivity near the bifurcation point. The circuit should be tuned to operate in this regime rather than attempting to minimize noise.

## Experiment 1 — E. coli Stochastic Resonance in Chemotactic Navigation

### Scientific Question

Does an intermediate noise temperature maximize a Langevin particle's ability to follow a periodic signal in a biased double-well potential? Does the optimal noise level shift predictably with gradient strength?

This directly tests Principle 2 (stochastic resonance enhances encoding fidelity) and extends the D2 SR study by adding a symmetry-breaking chemical gradient via `ExternalField`.

### Langevin Model of Chemotaxis

The experiment maps E. coli chemotaxis onto the Langevin framework:

| Biology | Langevin Model | Component |
|---------|---------------|-----------|
| Run/tumble states | Bistable wells at ±x₀ | `DoubleWellPotential(ΔV=3, x₀=1)` |
| Periodic chemical signal | Sub-threshold oscillating force | `OscillatingField(A₀=0.3, ω=2π·k_Kramers)` |
| Chemotactic gradient | Linear bias toward one well | `ExternalField([h])` |
| CheY-P signaling noise | Langevin thermal noise | `LangevinThermostat(γ=10, kT=1)` |
| Noise-modulated switching | Temperature as RL action | `.with_ctrl_temperature()` |

The particle "runs" (stays in one well) and "tumbles" (crosses the barrier). Stochastic resonance occurs when the noise-driven switching rate matches the signal frequency — the Kramers rate at `kT ≈ 1.0` equals the signal angular frequency `ω = 2π × 0.01214`.

### Experimental Design

**Synchrony metric** (same as D2):

```
synchrony = sign(qpos[0]) · cos(ω·t)
```

Positive when the particle occupies the correct well at the correct phase of the signal. Averaged per step over an episode.

**Part A — Baseline temperature sweep:** 30 kT multipliers log-spaced in [0.1, 5.0], 20 episodes each. Establishes the SR curve and locates the peak. Repeated at gradient strengths h ∈ {0.0, 0.3} to test whether the gradient shifts the optimum.

**Part B — Scientific controls:**
- Low noise (kT × 0.1): particle trapped in one well, no switching → zero synchrony
- High noise (kT × 5.0): random switching, no correlation with signal → zero synchrony
- No signal (A₀ = 0): metric validates to zero even at the SR-optimal temperature

**Part C — Multi-algorithm training:** Three algorithm classes on the gradient-biased setup (h = 0.3):
- **CEM** (evolutionary, gradient-free) — sim-rl
- **PPO** (policy gradient) — sim-rl
- **SA** (simulated annealing) — sim-opt

Each trains a `LinearPolicy(2, 1)` for 100 epochs on 32-env VecEnv with ctrl-temperature. Evaluation: 20 deterministic episodes with the trained policy.

### Gate System

| Gate | Test | Threshold |
|------|------|-----------|
| **A** | Significant synchrony on eval | One-sample t-test, \|t\| > 2.861 (df=19, α=0.01) |
| **B** | Learning monotonicity | best(last 10 epochs) > mean(first 5 epochs) |
| **C** | Learned kT near SR peak | \|learned_kT - peak_kT\| / peak_kT < 0.5 |

Controls use the 3σ threshold: `|mean| < 3·stderr`.

### Results

**Controls (validated):**

| Control | Synchrony | Stderr | Result |
|---------|-----------|--------|--------|
| Low noise (kT × 0.1) | 0.002 | 0.004 | PASS (indistinguishable from zero) |
| High noise (kT × 5.0) | -0.012 | 0.022 | PASS (indistinguishable from zero) |
| No signal (A₀ = 0) | -0.031 | 0.028 | PASS (indistinguishable from zero) |

**Baseline SR sweep (validated):**

| Metric | Value |
|--------|-------|
| Peak kT multiplier | 1.70 |
| Peak synchrony | 0.090 ± 0.020 |
| t-statistic | 4.52 (critical = 2.861, p < 0.01) |
| Peak location | Interior (not at boundary) — true resonance |

The SR curve shows the predicted bell shape: zero synchrony at low noise (particle trapped), rising to a clear peak at kT ≈ 1.7 (noise-driven switching matches signal frequency), falling back to zero at high noise (random switching destroys correlation).

**Multi-algorithm training:** Not yet run.

> **Platform readiness:** High — all components exist. `ThermCircuitEnv` builder, `DoubleWellPotential`, `OscillatingField`, `ExternalField`, and all 8 algorithms via `Algorithm` trait.
>
> **Status:** Controls validated. Baseline sweep and training in progress.
>
> **Code:** [`sim/L0/therm-env/tests/experiment_1.rs`](../../../sim/L0/therm-env/tests/experiment_1.rs)

## Key References

- Purcell, E.M. "Life at Low Reynolds Number." *American Journal of Physics* 45 (1977)
- Shapere, A. & Wilczek, F. "Geometry of Self-Propulsion at Low Reynolds Number." *Journal of Fluid Mechanics* 198 (1989)
- Flores, M. et al. "Signalling Noise Enhances Chemotactic Drift of E. coli." *Physical Review Letters* 109 (2012)
- Mattingly, H.H. et al. "Escherichia coli Chemotaxis is Information Limited." *Nature Physics* 17 (2021)
