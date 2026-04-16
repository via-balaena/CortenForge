# Experiments, Status, and Next Steps

## Infrastructure

All experiments run on CortenForge's simulation stack:

| Component | Module |
|---|---|
| Langevin dynamics | sim-core (Euler integrator) |
| Thermodynamic circuit environments | sim-therm-env (`ThermCircuitEnv` builder) |
| Passive energy landscapes | sim-thermostat (`PassiveComponent` trait) |
| RL algorithms | sim-rl (CEM, REINFORCE, PPO, TD3, SAC) |
| Gradient-free optimization | sim-opt (SA, Richer-SA, PT) |

## Completed Validations

| Principle | Regime | Result | Design Rule |
|-----------|--------|--------|-------------|
| **Noise Tuning (P2)** | E. coli | **Validated** | kT ≈ 2.3 for J < 1.5, kT ≈ 4.3 for J ≥ 2.0. ΔV/kT < 3.0 (trapping cutoff). |
| **Injection Timing (P4)** | Ctenophore | **Validated** | δ ≈ π/5 for J < 2 (18–37% improvement). Synchronized for J ≥ 2. |
| **Scale-Invariance (P6)** | Octopus | **Validated** | kT ≈ 2.5 holds at N=4, 8, 16 without retuning. |
| **Topological Encoding (P1)** | E. coli | **Failed** | Amplitude dominates in Langevin domain. Use freely. |
| **Deliberate Instability (P11)** | Peregrine | **Failed** | No sharp bifurcation. ΔV axis is forgiving. |

## The Boundary

**What transfers:** Statistical-mechanical questions — noise tuning, phase coordination, extensivity. The Langevin framework speaks this language natively.

**What doesn't:** Dynamical-systems questions — topological invariants (requires time-reversibility), bifurcation sensitivity (requires sharp phase transitions at finite N). The model doesn't have the vocabulary.

## Remaining Langevin-Ready Principles

These could be tested with the existing `ThermCircuitEnv` infrastructure:

| Principle | Regime | Experiment | Effort |
|-----------|--------|-----------|--------|
| P7 — Predictive forward model | Dragonfly | PN guidance in Langevin noise | Medium |
| P9 — Minimum observables | Dragonfly | Observation ablation study | Medium |
| P5 — Compressed command | Octopus | Single ctrl for heterogeneous circuit | Medium |
| P8 — Pre-selection | Dragonfly | Regime gate check | Small |

## Principles Needing Different Physics

| Principle | Regime | What's needed |
|-----------|--------|--------------|
| P3 — Multimodal switching | Ctenophore | Asymmetric power/recovery strokes need drag model |
| P10 — Paired perturbation structures | Peregrine | Spatial vortex physics |
| P12 — Logarithmic spiral approach | Peregrine | 2D/3D flow field (CFD) |

## Next Experiments

Four follow-on experiments that deepen the validated results, ordered by impact:

### 1. N-Scaling Law (high priority)

Peak synchrony *increased* with circuit size: 0.041 (N=4) → 0.049 (N=8) → 0.063 (N=16). If this is a real scaling law, bigger circuits are *better* at signal following. This would be the strongest possible result for thermodynamic circuit engineering: scale up and fidelity improves for free.

**Experiment:** Sweep N = 4, 8, 12, 16, 24, 32, 48, 64 at J=1.0 with 40 log-spaced kT points in [1.0, 5.0], 40 episodes per point. Fit sync_peak vs N to a power law and validate effective barrier model for kT drift. ~6 hours runtime.

**Gates:**
- sync_peak ∝ N^α with α significantly > 0 (two-tailed t-test on log-log regression slope, df=6, α=0.01)
- Peak kT drift < 50% across N range (scale invariance sanity check)
- Effective barrier model peak_kT = a + b·(N−2)/N achieves R² > 0.80

**Code:** `ising_scale_law_sweep` in `ising_chain.rs`.

### 2. Coupling Crossover Mapping

The Noise Tuning rule showed two regimes: weak coupling (J < 1.5, peak kT ≈ 2.3) and strong coupling (J ≥ 2.0, peak kT ≈ 4.3). Where exactly is the crossover? Is it smooth or sharp?

**Experiment:** Sweep J = 1.0, 1.25, 1.5, 1.75, 2.0 with 25 kT points, 40 episodes each. ~3 hours.

**Gate:** If the crossover occupies less than ΔJ = 0.25, it's sharp — potentially a phase transition in the coupled system.

### 3. Optimal (J, δ) Surface

A finer mesh would map the full optimal phase-lag surface: 8 J values × 30 δ values × 80 episodes. ~6 hours.

### 4. Effective Barrier Model Validation

Merged into experiment 1 (N-Scaling Law) as Gate 3. The effective-barrier model peak_kT = a + b·(N−2)/N is validated against 8 chain sizes.

## All Experiment Code

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs)
