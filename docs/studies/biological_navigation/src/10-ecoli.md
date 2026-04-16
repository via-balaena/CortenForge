# Noise Tuning: E. coli Stochastic Resonance

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

*Applied to thermodynamic circuits:* This principle predicts that at very low throughput rates, X-encoding must rely on injection sequence topology rather than amplitude. **However, testing showed this does NOT transfer to the Langevin domain** — amplitude is a direct lever in time-irreversible systems (see Level 4 below). The scallop theorem requires time-reversibility, which Langevin dynamics lack.

**Principle 2:** Stochastic resonance is available. An optimal noise level enhances encoding fidelity by improving sensitivity near the bifurcation point. The circuit should be tuned to operate in this regime rather than attempting to minimize noise.

## Experiment 1 — E. coli Stochastic Resonance in Chemotactic Navigation

### Scientific Question

Does an intermediate noise temperature maximize a Langevin particle's ability to follow a periodic signal in a biased double-well potential? Does the optimal noise level shift predictably with gradient strength?

This directly tests Principle 2 (stochastic resonance enhances encoding fidelity) and extends prior single-particle SR validation by adding a symmetry-breaking chemical gradient via `ExternalField`.

### Langevin Model of Chemotaxis

The experiment maps E. coli chemotaxis onto the Langevin framework:

| Biology | Langevin Model | Component |
|---------|---------------|-----------|
| Run/tumble states | Bistable wells at ±x₀ | `DoubleWellPotential(ΔV=3, x₀=1)` |
| Periodic chemical signal | Sub-threshold oscillating force | `OscillatingField(A₀=0.3, ω=2π·k_Kramers)` |
| Chemotactic gradient | Linear bias toward one well | `ExternalField([h])` |
| CheY-P signaling noise | Langevin thermal noise | `LangevinThermostat(γ=10, kT=1)` |
| Noise-modulated switching | Temperature as RL action | `.with_ctrl_temperature()` |

The particle "runs" (stays in one well) and "tumbles" (crosses the barrier). Stochastic resonance occurs when the noise-driven switching rate matches the signal frequency — the Kramers rate (the thermally activated barrier-crossing rate, which scales as exp(−ΔV/kT)) at `kT ≈ 1.0` equals the signal angular frequency `ω = 2π × 0.01214`.

### Experimental Design

**Synchrony metric:**

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

Each trains a linear policy (2 inputs → 1 output, mapping particle position and velocity to a temperature control signal) for 100 epochs on a 32-environment parallel batch with ctrl-temperature. Evaluation: 20 deterministic episodes with the trained policy.

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

**1-particle multi-algorithm training:** Skipped (superseded by Level 3 multi-particle validation below).

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

**Phase 2 — Multi-algorithm training at each J:** CEM, SA (simulated annealing), and Richer-SA (SA with adaptive neighborhood) each train a linear policy (8 inputs → 1 output) for 100 epochs on a 32-environment parallel batch. Tests whether gradient-free agents independently discover the SR-optimal temperature at each coupling strength. PPO was dropped — policy gradient methods compute per-timestep advantages, fundamentally wrong when the optimal policy is a constant temperature.

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

#### Design Rule (Noise Tuning)

For an N=4 Ising-coupled bistable circuit with coupling strength J:

- **J < 1.5:** Operate at kT ≈ 2.3. Coupling enhances SR without shifting it. Stronger coupling gives better signal fidelity (up to ~50% improvement at J = 1.0).
- **J ≥ 2.0:** Operate at kT ≈ 4.3. The collective switching mode dominates and requires higher noise.
- **J ≈ 1.5–2.0:** Transition zone. Both modes active. Operating at either kT ≈ 2.3 or kT ≈ 4.3 gives similar fidelity.

Operating at the wrong temperature degrades synchrony to noise-floor levels.

#### Operating Envelope: Barrier Height Tolerance

The Noise Tuning Rule maps the kT axis of the design surface. A complementary sweep maps the ΔV axis: fix kT=2.0 and sweep barrier height ΔV from 0.5 to 10.0 at two signal amplitudes (A₀=0.3 and A₀=0.1), N=4 uncoupled particles (J=0), 40 episodes per point.

**Key finding: the ΔV axis is forgiving.** Unlike the sharp SR peak on the kT axis, the synchrony-vs-ΔV curve is a broad plateau:

| ΔV range | ΔV/kT | Behavior |
|----------|-------|----------|
| 0.5–5.5 | 0.25–2.75 | **Sensitivity plateau** — synchrony stable at 0.03–0.05 |
| 5.5–6.0 | 2.75–3.0 | Sharp drop-off — transitions from detectable to trapped |
| > 6.0 | > 3.0 | **Trapping regime** — synchrony indistinguishable from zero |

The weak signal (A₀=0.1) was below detection threshold at all ΔV values, setting a minimum detectable signal floor for this kT.

**Engineering implication:** The design surface is asymmetric. Temperature requires precision tuning (sharp peak, ±30% of optimal degrades to noise floor). Barrier height is tolerant — any ΔV/kT between 0.25 and 2.75 gives similar performance. This means: **tune kT carefully to the design rule above; ΔV can be approximate.**

The trapping cutoff at ΔV/kT ≈ 3 is a hard constraint: if the barrier exceeds 3× the thermal energy, the circuit cannot respond to signals regardless of other parameters.

#### Why Not Train Agents to Find the Peak?

Phase 2 trained gradient-free agents to discover the SR-optimal temperature from dynamics alone. CEM(J=0) converged to kT ≈ 0.07 — a local optimum that games the synchrony metric rather than finding the SR peak at kT ≈ 2.3. SA(J=0) stalled for 80+ epochs.

The likely cause: a linear policy mapping 8 particle observables to a scalar temperature lacks the capacity to represent the nonlinear relationship between circuit state and optimal noise level.

This does not weaken the design rule. The sweep data directly maps the optimal kT for each coupling strength — an engineer doesn't need an agent to discover this; the rule is the table above. Agent-based discovery becomes relevant when the circuit topology is unknown or changes at runtime, a harder problem deferred to future work.

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs)

---

### Level 4 — Topological Encoding Test (Principle 1)

Does injection sequence topology matter more than amplitude? In the biological E. coli regime (Re < 1), the scallop theorem guarantees that amplitude and rate are irrelevant — only the topology of the motion sequence determines net displacement. Principle 1 claims this extends to thermodynamic circuits: information must be encoded in sequence structure, not amplitude.

#### The Test

Fix J=1.0, kT=2.82 (P2 optimal). Compare two conditions:

- **Condition A (topology):** Metachronal injection δ=0.66 (P4 optimal phase lag) at baseline amplitude A₀=0.3
- **Condition B (amplitude):** Synchronized injection δ=0 at doubled amplitude A₀=0.6

If topology beats doubled amplitude, Principle 1 transfers to the Langevin domain.

Additionally: sweep synchronized amplitude from A₀=0.3 to 2.0 to find the crossover point where brute-force amplitude matches the topological advantage.

#### Results

**Principle 1: NOT VALIDATED.** Amplitude dominates in the Langevin domain.

| Condition | δ | A₀ | Synchrony | Stderr |
|-----------|---|-----|-----------|--------|
| Metachronal (reference) | 0.66 | 0.3 | +0.050 | 0.007 |
| Synchronized | 0 | 0.3 | +0.051 | 0.006 |
| Synchronized | 0 | 0.4 | +0.067 | 0.007 |
| Synchronized | 0 | 0.5 | +0.073 | 0.007 |
| Synchronized | 0 | 0.6 | +0.088 | 0.007 |
| Synchronized | 0 | 0.8 | +0.127 | 0.008 |
| Synchronized | 0 | 1.0 | +0.154 | 0.006 |
| Synchronized | 0 | 1.5 | +0.228 | 0.007 |
| Synchronized | 0 | 2.0 | +0.290 | 0.006 |

80 episodes per condition.

**Gates:**

| Gate | Test | Result |
|------|------|--------|
| 0 (Baseline) | Metachronal reference significant | PASS (\|t\|=7.23) |
| 1 (Core claim) | Metachronal(0.3) > Synchronized(0.6) | **FAIL** (t=−3.81, amplitude wins) |
| 2 (Amp effect) | Synchronized(0.6) > Synchronized(0.3) | PASS (amplitude helps) |

**Why it fails — and why that's informative:**

The scallop theorem applies at Re < 1 where the governing equations are linear and time-reversible. In that regime, amplitude literally cancels out of the physics. The Langevin domain has no such constraint: the oscillating field directly biases the particle's switching rate, and a stronger field produces proportionally more switching. Synchrony scales nearly linearly with amplitude (0.051 → 0.088 → 0.290 across the 0.3–2.0 range).

At matched amplitude (A₀=0.3), metachronal and synchronized injection produce identical synchrony (0.050 vs 0.051). The Injection Timing metachronal advantage (18–37% at certain J values) is a coupling-coordination effect that emerges from specific parameter combinations, not a universal topology-dominance principle.

**What this means for thermodynamic circuit design:**

In the Langevin domain, amplitude IS a design lever. Engineers can — and should — use signal strength to improve encoding fidelity. The topological encoding principle from Stokes-regime biology does not transfer to systems where the dynamics are nonlinear and time-irreversible. This is a boundary condition on the biological analogy, not a weakness: it tells us precisely where the analogy holds (noise tuning, phase coordination) and where it breaks (amplitude scaling).

#### Design Rule (Principle 1)

**Not applicable in the Langevin domain.** Amplitude modulation is effective and scales linearly with synchrony. Use it.

For topology-sensitive encoding, the physics must constrain amplitude from the equations — the Stokes-regime scallop theorem does this, but Langevin dynamics do not. Principle 1 remains valid for systems where the scallop theorem applies (low Re fluids, certain linear circuit topologies), but does not generalize to the broader thermodynamic circuit architecture.

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs) — `ising_topological_sweep`

## Key References

- Purcell, E.M. "Life at Low Reynolds Number." *American Journal of Physics* 45 (1977)
- Shapere, A. & Wilczek, F. "Geometry of Self-Propulsion at Low Reynolds Number." *Journal of Fluid Mechanics* 198 (1989)
- Flores, M. et al. "Signalling Noise Enhances Chemotactic Drift of E. coli." *Physical Review Letters* 109 (2012)
- Mattingly, H.H. et al. "Escherichia coli Chemotaxis is Information Limited." *Nature Physics* 17 (2021)
