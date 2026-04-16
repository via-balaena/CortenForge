# Validation Roadmap

Which principles can we validate now, which need new physics, and in what order.

## The Key Insight

The biological organisms need fluid dynamics. The thermodynamic circuit analogs mostly don't. A particle in a double well coupled to its neighbors under Langevin noise IS a thermodynamic circuit — it doesn't need a fluid around it. Most principles translate directly to this domain.

The exceptions are principles where the fluid structure itself IS the mechanism (vortex coupling, hydrodynamic drag asymmetry). Those need either soft-body simulation (intermediate viscosity) or full CFD (turbulent flows).

## Langevin-Ready Principles

These can be validated with the existing `ThermCircuitEnv` infrastructure: double wells, pairwise coupling, oscillating fields, Langevin thermostat, and the CEM/SA/RicherSA algorithm suite.

| Priority | Principle | Regime | Circuit Experiment | Effort |
|----------|-----------|--------|-------------------|--------|
| **1** | P2 — Stochastic resonance | E. coli | Coupling sweep: how SR-optimal kT shifts with J | **Running** |
| **2** | P4 — Metachronal coordination | Ctenophore | Phase-lagged injection: sweep phase offset δ between coupled particles | Trivial — one parameter change |
| **3** | P1 — Topological encoding | E. coli | Sequence structure matters, amplitude doesn't: compare phase orderings at matched amplitude | Small |
| **4** | P11 — Deliberate instability | Peregrine | Near-bifurcation sensitivity: tune ΔV near critical point, show sensitivity amplification | Small |
| **5** | P7 — Predictive forward model | Dragonfly | PN guidance in Langevin noise: N sweep, noise sweep, forward-model variant | Medium (experiment_4.rs exists) |
| **6** | P9 — Minimum observables | Dragonfly | Observation ablation: train with subsets of obs, show 2 local cues suffice | Medium |
| **7** | P5 — Compressed command | Octopus | Single ctrl-temperature for heterogeneous circuit (different ΔV per particle) | Medium |
| **8** | P6 — Scale-invariant encoding | Octopus | Same encoding program works at N=4, 8, 16 without retuning | Medium |
| **9** | P8 — Pre-selection | Dragonfly | Gate check: verify noise/relaxation regime before committing to encoding strategy | Small |

### Why this order

P2 first because it's the simplest test of noise-as-resource — the foundational claim.

P4 second because it's the cheapest additional principle to validate (one parameter change to the existing Ising chain experiment) and it crosses into a different regime (Intermediate), which makes the framework look like a general lens rather than a single observation.

P1 third because together with P2 it completes Regime 1 — both E. coli principles validated.

P11 fourth because it's the most surprising claim in the framework (instability is a feature) and maps cleanly onto double-well bifurcation physics we already have.

P7 fifth because the infrastructure exists (experiment_4.rs) and it opens Regime 4.

## Needs Soft-Body / Viscous Coupling

These principles involve drag asymmetry or hydrodynamic interactions between moving elements. MuJoCo's soft-body capabilities could approximate viscous coupling for the intermediate regime without full CFD.

| Principle | Regime | What's needed |
|-----------|--------|--------------|
| P3 — Multimodal switching (temporal asymmetry part) | Ctenophore | Asymmetric power/recovery strokes need drag model |
| P10 — Paired perturbation structures | Peregrine | Correlated noise injection could approximate in Langevin, but vortex structure needs spatial physics |

## Needs CFD

These principles require turbulent flow structures that only exist in 2D/3D fluid dynamics.

| Principle | Regime | What's needed |
|-----------|--------|--------------|
| P12 — Logarithmic spiral approach | Peregrine | Scale-invariant trajectory in a flow field |
| Experiment 2 — Ctene metachronal wave | Ctenophore | Hydrodynamic coupling between beating cilia |
| Experiment 5 — M-shape vortex coupling | Peregrine | LES-validated counter-rotating vortex pairs |

## Design Rules

Each validated principle should produce a concrete, quantitative design rule — something an engineer building a thermodynamic circuit can use directly.

| Principle | Expected design rule form |
|-----------|--------------------------|
| P2 — SR | "For coupling strength J, optimal noise kT ≈ f(J). Operating outside this band degrades fidelity by X%." |
| P4 — Metachronal | "For an N-cell chain with coupling J, optimal injection phase lag δ* ≈ g(J, N)." |
| P1 — Topological | "Injection sequence topology determines encoding fidelity; amplitude modulation has no effect below threshold A_c." |
| P11 — Instability | "Operating at ΔV within ε of the bifurcation point amplifies sensitivity by factor S(ε)." |
| P7 — Predictive | "Forward-model injection with navigation constant N ≈ 3 reduces convergence time by factor F relative to reactive control." |
