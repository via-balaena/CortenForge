# Simulation Experiments and Status

## Why CortenForge

CortenForge's simulation infrastructure — Langevin-capable physics simulation, gradient-free optimization, RL training loops — provides the tools to test biological navigation principles directly on thermodynamic circuit proxies. The organisms are not analogies to be reasoned about abstractly; they are sources of falsifiable, quantitative predictions that can be tested on Ising chain models.

## Infrastructure

| Component | CortenForge Module |
|---|---|
| Langevin dynamics | sim-core with Euler integrator |
| Thermodynamic circuit environments | sim-therm-env (`ThermCircuitEnv` builder) |
| Passive energy landscapes | sim-thermostat (`PassiveComponent` trait) |
| RL algorithm suite | sim-rl (CEM, REINFORCE, PPO, TD3, SAC) |
| Gradient-free optimization | sim-opt (SA, Richer-SA, PT) |

## Validated Principles

| Principle | Regime | Result | Design Rule |
|-----------|--------|--------|-------------|
| **P2 — Stochastic resonance** | E. coli | **Validated** | kT ≈ 2.3 for J < 1.5, kT ≈ 4.3 for J ≥ 2.0 |
| **P4 — Metachronal coordination** | Ctenophore | **Validated** | δ ≈ π/5 for J < 2, synchronized for J ≥ 2 |
| **P6 — Scale-invariant encoding** | Octopus | **Validated** | kT ≈ 2.5 holds at N=4, 8, 16 without retuning |

## Informative Negatives

| Principle | Regime | Result | Why It Failed |
|-----------|--------|--------|---------------|
| **P11 — Deliberate instability** | Peregrine | No sharp bifurcation | ΔV axis is forgiving (broad plateau), not critical. Langevin systems have smooth crossovers, not sharp phase transitions at finite N. |
| **P1 — Topological encoding** | E. coli | Amplitude dominates | Scallop theorem requires time-reversible dynamics. Langevin is time-irreversible — amplitude is a lever. |

## The Boundary

The validated principles are all **statistical-mechanical** questions: optimal noise level, injection timing coordination, extensivity. These transfer because the Langevin framework IS statistical mechanics.

The failures are **dynamical-systems** questions: topological invariants (P1) and bifurcation sensitivity (P11). These require mathematical structures — time-reversibility, sharp phase transitions — that the Langevin domain does not have.

This is the boundary condition on the biological analogy. It tells an engineer exactly which biological literature to mine (noise exploitation, phase coordination) and which to ignore (topology, instability amplification).

## Next Experiments

Four follow-on experiments that deepen the validated results, ordered by impact:

### 1. N-Scaling Law (high priority)

Peak synchrony *increased* with circuit size: 0.041 (N=4) → 0.049 (N=8) → 0.063 (N=16). If this is a real scaling law rather than noise, bigger circuits are not just "the same" — they're *better* at signal following. This would be the strongest possible result for thermodynamic circuit engineering: scale up and fidelity improves for free.

**Experiment:** Sweep N = 4, 8, 12, 16, 24, 32 at J=1.0 with fine kT resolution around the peak (kT 1.5–4.0, 20 points, 40 episodes). Fit sync_peak vs N to a power law. ~2 hours runtime.

**Gate:** If sync_peak ∝ N^α with α significantly > 0 (two-tailed t-test on log-log regression slope), the scaling law is real.

### 2. Coupling Crossover Mapping

P2 showed two regimes: weak coupling (J < 1.5, peak kT ≈ 2.3) and strong coupling (J ≥ 2.0, peak kT ≈ 4.3). Where exactly is the crossover? Is it smooth or sharp?

**Experiment:** Sweep J = 1.0, 1.25, 1.5, 1.75, 2.0 with 25 kT points, 40 episodes each. Map the peak kT vs J curve through the transition. ~3 hours.

**Gate:** If the crossover occupies less than ΔJ = 0.25 (peak shifts by >50% of its full range within one J step), it's sharp — potentially a phase transition in the coupled system.

### 3. Optimal (J, δ) Surface

P4 showed metachronal injection helps at J = 0.5–1.0 but not at J ≥ 2. A finer mesh would map the full optimal phase-lag surface.

**Experiment:** 8 J values × 30 δ values × 80 episodes. ~6 hours.

### 4. Effective Barrier Model Validation

N=16 peaked one grid step higher than N=4/N=8. The effective-barrier model predicts an ~8% shift from the changing end/interior particle ratio. Does the model hold?

**Experiment:** Overlaps with experiment 1 (N-scaling). The kT shift vs N curve either matches the effective-barrier prediction or reveals a different mechanism.

## All Experiment Code

All experiments live in a single test file with shared infrastructure:

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs)
