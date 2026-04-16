# Validation Roadmap

## The Key Insight

The biological organisms need fluid dynamics. The thermodynamic circuit analogs mostly don't. A particle in a double well coupled to its neighbors under Langevin noise IS a thermodynamic circuit — it doesn't need a fluid around it. Most principles translate directly to this domain.

The exceptions are principles where the fluid structure itself IS the mechanism (vortex coupling, hydrodynamic drag asymmetry) — or where the mathematics requires constraints the Langevin domain doesn't have (time-reversibility for the scallop theorem, sharp phase transitions for bifurcation sensitivity).

## Completed Validations

| Principle | Regime | Result | Key Finding |
|-----------|--------|--------|-------------|
| **P2 — Stochastic resonance** | E. coli | **Validated** | Two-regime behavior: kT ≈ 2.3 (J < 1.5), kT ≈ 4.3 (J ≥ 2). ΔV axis forgiving. |
| **P4 — Metachronal coordination** | Ctenophore | **Validated** | δ ≈ π/5 for J < 2, 18–37% improvement. Synchronized optimal for J ≥ 2. |
| **P6 — Scale-invariant encoding** | Octopus | **Validated** | kT ≈ 2.5 holds at N=4, 8, 16. Peak indices span 1 on 25-point grid. |
| **P1 — Topological encoding** | E. coli | **Failed** | Amplitude dominates in Langevin domain. Scallop theorem doesn't transfer. |
| **P11 — Deliberate instability** | Peregrine | **Failed** | No sharp bifurcation. Broad plateau, not critical point. |

## The Boundary

**What transfers:** Statistical-mechanical questions — noise tuning (P2), phase coordination (P4), extensivity (P6). The Langevin framework speaks this language natively.

**What doesn't transfer:** Dynamical-systems questions — topological invariants (P1, requires time-reversibility), bifurcation sensitivity (P11, requires sharp phase transitions). The model doesn't have the vocabulary.

## Remaining Langevin-Ready Principles

These could be tested with the existing `ThermCircuitEnv` infrastructure but are lower priority than the micro-experiments on validated results:

| Principle | Regime | Circuit Experiment | Effort |
|-----------|--------|-------------------|--------|
| P7 — Predictive forward model | Dragonfly | PN guidance in Langevin noise | Medium (experiment_4.rs exists) |
| P9 — Minimum observables | Dragonfly | Observation ablation: train with subsets of obs | Medium |
| P5 — Compressed command | Octopus | Single ctrl-temperature for heterogeneous circuit | Medium |
| P8 — Pre-selection | Dragonfly | Gate check: verify noise/relaxation regime | Small |

## Needs Soft-Body / Viscous Coupling

These principles involve drag asymmetry or hydrodynamic interactions. Not testable with current infrastructure.

| Principle | Regime | What's needed |
|-----------|--------|--------------|
| P3 — Multimodal switching | Ctenophore | Asymmetric power/recovery strokes need drag model |
| P10 — Paired perturbation structures | Peregrine | Vortex structure needs spatial physics |

## Needs CFD

These principles require turbulent flow structures that only exist in 2D/3D fluid dynamics.

| Principle | Regime | What's needed |
|-----------|--------|--------------|
| P12 — Logarithmic spiral approach | Peregrine | Scale-invariant trajectory in a flow field |

## Design Rules Summary

Each validated principle produced a concrete, quantitative design rule:

| Principle | Design Rule |
|-----------|-------------|
| P2 — SR | For J < 1.5, operate at kT ≈ 2.3. For J ≥ 2.0, operate at kT ≈ 4.3. ΔV/kT must stay below 3.0 (trapping cutoff). |
| P4 — Metachronal | For J < 2, inject with phase lag δ ≈ π/5. For J ≥ 2, synchronize. |
| P6 — Scale-invariant | For J=1.0, optimal kT ≈ 2.5 regardless of N (tested 4–16). |
| P1 — Topological | Not applicable in Langevin domain. Use amplitude modulation freely. |
| P11 — Instability | Not applicable. ΔV axis is forgiving — tune kT carefully instead. |
