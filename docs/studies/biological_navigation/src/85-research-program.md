# Simulation Experiments and Status

## Why CortenForge

CortenForge's simulation infrastructure — Langevin-capable physics simulation, Hill-type muscles, XPBD deformables, RL training loops, sensor fusion — provides the tools to simulate the biological exemplars directly and extract quantitative design parameters. The organisms are not analogies to be reasoned about abstractly; they are simulatable physical systems from which specific numbers can be extracted.

## Infrastructure Mapping

| Simulation Need | CortenForge Module |
|---|---|
| Langevin dynamics | sim-core with Euler integrator |
| Thermodynamic circuit environments | sim-therm-env (`ThermCircuitEnv` builder) |
| Passive energy landscapes | sim-thermostat (`PassiveComponent` trait) |
| RL algorithm suite | sim-rl (CEM, REINFORCE, PPO, TD3, SAC) |
| Gradient-free optimization | sim-opt (SA, Richer-SA, PT) |
| Hill-type muscle for octopus | sim-physics Hill-type muscle models |
| Sensor array (lateral line) | sensor-fusion hardware-agnostic sensor types |
| GPU-batched simulations | sim-gpu (not yet implemented) |

## Experiment Status

| # | Experiment | Regime | Readiness | Status |
|---|-----------|--------|-----------|--------|
| 1 | E. coli SR in chemotaxis | Viscosity-dominated | **High** | Controls validated, sweep in progress |
| 2 | Ctenophore metachronal phase-lag | Intermediate | Low | Not started |
| 3 | Octopus bend propagation | Inertial distributed | Medium | Not started |
| 4 | Dragonfly PN guidance | Predictive inertial | **High** | Phase 4 validation complete |
| 5 | Peregrine vortex coupling | High-inertial turbulent | Low | Not started |
| 6 | Regime transitions | Cross-regime | Low | Not started |
| 7 | Fish lateral line sensing | Regime 3-4 boundary | Low | Not started |

## Critical Path

The critical path runs through **Experiment 1** and **Experiment 4** — the two experiments with high platform readiness.

**Experiment 4** (Dragonfly PN guidance) has a Phase 4 validation test demonstrating that CEM can learn state-dependent temperature modulation in a double-well landscape. The full experiment — sweeping navigation constant N across [1, 5], varying noise level, and testing the forward-model variant — builds directly on this infrastructure.

**Experiment 1** (E. coli SR) connects directly to the D2 stochastic resonance findings already established in the codebase. The core claim — that an intermediate noise level maximizes encoding fidelity — is a concrete, falsifiable, quantitative prediction.

Experiments 3, 6, and 7 depend on infrastructure that does not yet exist (MJCF tentacle models, distributed sensor plugins, multiple working regime strategies). Experiments 2 and 5 need domain-specific physics models (simplified drag, vortex coupling reformulation).

## Connections to Existing Results

The D2 stochastic resonance study (`sim/L0/thermostat/tests/d2c_cem_training.rs` and related) established that:

- CEM finds the SR band in a double-well + oscillating field setup
- Linear Q is insufficient for TD3/SAC on this task
- Exploration-noise inflation generalizes from D1d

These findings directly inform Experiment 1 (E. coli SR) and provide the baseline infrastructure that Experiment 4 builds on.
