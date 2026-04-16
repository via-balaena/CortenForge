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

**1-particle multi-algorithm training:** Not yet run on the 1-particle setup (superseded by Level 3 below).

> **Code:** [`sim/L0/therm-env/tests/experiment_1.rs`](../../../sim/L0/therm-env/tests/experiment_1.rs)

---

### Level 3 — Ising Chain Stochastic Resonance

Does stochastic resonance scale from 1 particle to coupled multi-cell systems? This is the direct test of Principle 2 on a thermodynamic circuit proxy.

#### The Setup

Four particles, each in a double well with its own oscillating field, connected by nearest-neighbor coupling:

```
Particle 0 ←J→ Particle 1 ←J→ Particle 2 ←J→ Particle 3
  [±x₀]          [±x₀]          [±x₀]          [±x₀]
  + signal       + signal       + signal       + signal
```

| Component | Role |
|-----------|------|
| `DoubleWellPotential(ΔV=3, x₀=1, dof=i)` per particle | Bistable states |
| `OscillatingField(A₀=0.3, ω=2π·k_Kramers, dof=i)` per particle | Periodic signal (same as Level 2) |
| `PairwiseCoupling::chain(4, J)` | Inter-cell coupling (sweep variable) |
| `LangevinThermostat(γ=10, kT=15).with_ctrl_temperature()` | Noise, kT range [0, 15] via RL |

**Reward:** Average synchrony across all particles: `(1/N) × Σ sign(qpos[i]) × cos(ωt)`

**Scientific question:** How does coupling strength J shift the SR-optimal noise temperature? This is the τ_circuit / τ_noise characterization.

#### The Experiment

**Phase 0 — Scout (~30 sec):** J=2 only, 8 kT points in [1, 15], 10 episodes. Validates that the kT=15 ceiling captures the strongest-coupling peak.

**Phase 1 — Temperature sweep at 5 coupling strengths (~35 min):** For each J ∈ {0.0, 0.5, 1.0, 1.5, 2.0}, sweep 25 temperatures log-spaced in [0.1, 15.0], 40 episodes each. Maps the SR curve at each coupling strength. Gates: every J must have a significant (|t| > 2.708, df=39, α=0.01) interior peak.

**Phase 2 — Multi-algorithm training at each J:** CEM, SA, and RicherSA each train a `LinearPolicy(8, 1)` for 100 epochs on 32-env VecEnv. Tests whether gradient-free agents independently discover the SR-optimal temperature at each coupling strength. PPO was dropped — policy gradient methods compute per-timestep advantages, fundamentally wrong when the optimal policy is a constant temperature.

**Controls (same pattern as Level 2):** Low noise (kT × 0.1), high noise (kT × 10), no signal (A₀=0) — all must show zero synchrony.

**Key design decisions:**

- **kT range [0.1, 15.0]:** Coupling raises the effective barrier for individual particle switching (interior: ΔV_eff = 3 + 4J, end: 3 + 2J). Using the calibration ratio ΔV/kT_peak ≈ 1.39, the J=2 peak is predicted at kT ≈ 6.5. Range of 15 gives 2.3× headroom.
- **Training env `k_b_t = 15.0`:** The policy's tanh output [0, 1] maps to kT ∈ [0, 15.0], letting agents reach all predicted peaks (kT 2–7) at ctrl ≈ 0.13–0.47.
- **40 episodes/point:** For the weakest signals (sync ≈ 0.04, σ ≈ 0.06), stderr = 0.06/√40 = 0.0095, giving |t| = 4.2. Well above the 2.708 threshold.

#### Results

**Scout (validated):**

Peak at kT=6.5 (interior, |t|=4.17), exactly matching the effective-barrier prediction. Range [0.1, 15.0] confirmed safe.

**v1 sweep (invalid — range too narrow):**

| J | peak kT | peak sync | |t| | Status |
|---|---|---|---|---|
| 0.00 | 2.16 | 0.075 ± 0.014 | 5.47 | Valid (interior) |
| 0.10 | 5.00 | 0.043 ± 0.010 | 4.50 | **Boundary** |
| 0.50 | 1.64 | 0.039 ± 0.020 | 1.95 | Not significant |
| 1.00 | 5.00 | 0.054 ± 0.008 | 6.74 | **Boundary** |
| 2.00 | 3.78 | 0.049 ± 0.018 | 2.68 | Not significant |

Only J=0 was valid. Peaks for J≥0.1 hit the kT=5 ceiling or were below significance with 15 episodes.

**v2 sweep (validated):**

All 5 coupling strengths produce significant interior peaks. 5,000 episodes, 35 minutes, single run.

| J | peak kT | peak sync | |t| | Status |
|---|---|---|---|---|
| 0.00 | 2.29 | 0.043 ± 0.008 | 5.55 | PASS |
| 0.50 | 2.29 | 0.057 ± 0.008 | 7.63 | PASS |
| 1.00 | 2.82 | 0.065 ± 0.008 | 8.55 | PASS |
| 1.50 | 2.29 | 0.057 ± 0.012 | 4.89 | PASS |
| 2.00 | 4.29 | 0.053 ± 0.007 | 7.23 | PASS |

**Two-regime behavior:** Coupling does not simply shift the SR peak — it reveals two switching modes with a crossover:

- **Weak coupling (J ≤ 1.0):** The SR peak stays near kT ≈ 2.3 (same as uncoupled) but synchrony *increases* with coupling (0.043 → 0.065). Coupling enhances the existing resonance. Individual particle switching dominates — each particle crosses the barrier independently, and coupling merely coordinates them.
- **Strong coupling (J = 2.0):** The peak shifts to kT ≈ 4.3. The higher effective barrier (ΔV_eff = 3 + 2J to 3 + 4J for end/interior particles) requires more noise to drive transitions. The collective switching mode — where coupling forces particles to flip together — takes over.

The crossover between these regimes lies between J = 1.5 and J = 2.0. At J = 1.5, the data shows a primary peak at kT = 2.29 with a secondary bump near kT = 4.3–5.3, suggesting both modes are active and competing.

The effective-barrier model predicted the J = 2 peak at kT ≈ 6.5. The actual peak is at kT ≈ 4.3 — correct direction, right order of magnitude, but the simple model overestimates the barrier because cooperative switching lowers the effective barrier relative to independent switching.

#### Design Rule (Principle 2)

For an N=4 Ising-coupled bistable circuit with coupling strength J:

- **J < 1.5:** Operate at kT ≈ 2.3. Coupling enhances SR without shifting it. Stronger coupling gives better signal fidelity (up to ~50% improvement at J = 1.0).
- **J ≥ 2.0:** Operate at kT ≈ 4.3. The collective switching mode dominates and requires higher noise.
- **J ≈ 1.5–2.0:** Transition zone. Both modes active. Operating at either kT ≈ 2.3 or kT ≈ 4.3 gives similar fidelity.

Operating at the wrong temperature degrades synchrony to noise-floor levels.

#### What Remains

Phase 2 (CEM, SA, RicherSA training at each J) would validate that the optimal temperature can be *learned* from dynamics alone — the agent doesn't need to know J. This is a supplementary result; the sweep data above is the primary contribution.

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs)

## Key References

- Purcell, E.M. "Life at Low Reynolds Number." *American Journal of Physics* 45 (1977)
- Shapere, A. & Wilczek, F. "Geometry of Self-Propulsion at Low Reynolds Number." *Journal of Fluid Mechanics* 198 (1989)
- Flores, M. et al. "Signalling Noise Enhances Chemotactic Drift of E. coli." *Physical Review Letters* 109 (2012)
- Mattingly, H.H. et al. "Escherichia coli Chemotaxis is Information Limited." *Nature Physics* 17 (2021)
