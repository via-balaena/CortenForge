# Roadmap

## Mission

**Solve X-encoding through biology-inspired simulation: from Langevin models to hardware design rules.**

## The Root Problem

Given a thermodynamic circuit with energy function F, how do you choose the injection protocol so the system relaxes to the correct output distribution Y?

No formal theory exists. Chip engineers currently tune empirically at every scale. This is the bottleneck blocking thermodynamic computing from becoming an engineered technology.

## What a Solution Looks Like

A design rule that says: *for a circuit with τ_circuit / τ_noise = X, use encoding strategy Y, expect fidelity Z.* Quantitative, testable, generalizable.

## Where We Are

| Step | Progress | Description |
|------|----------|-------------|
| 1. Framework | **Done** | 12 X-encoding principles across 5 Reynolds number regimes, derived from biological navigation strategies |
| 2. 1-particle simulation | **Done** | SR peak validated (kT=1.70, \|t\|=4.52), controls pass, infrastructure works |
| 3. Multi-particle simulation | **Running** | 4-cell Ising chain with coupling sweep J={0, 0.1, 0.5, 1.0, 2.0} |
| 4. τ_circuit / τ_noise mapping | **Running** | Same experiment — coupling strength J is the proxy for τ_circuit / τ_noise |
| 5. Paper | Not started | Framework + principle validation + regime characterization |
| 6. Hardware validation | Needs collaboration | Test predictions on Extropic / Normal Computing hardware |

**Current position:** Steps 3 and 4 are running simultaneously — the coupling sweep *is* the τ_circuit / τ_noise characterization. The 4-cell Ising chain with oscillating fields tests whether SR scales to coupled systems, and sweeping J maps how the optimal noise temperature shifts with coupling strength. Results expected within hours.

## The Path

### Step 1: Finish Experiment 1 cleanly

One complete, statistically rigorous result. Stochastic resonance in a gradient-biased double well, validated with controls, confirmed across multiple algorithm classes (CEM, PPO, SA). This is the table stake — it proves the simulation infrastructure works and produces quantitative predictions.

### Step 2: Scale to coupled multi-particle systems

The infrastructure already supports this. `PairwiseCoupling` provides chain, ring, and fully-connected topologies. `ExternalField` provides per-particle bias. `ThermCircuitEnv` supports N particles. An Ising-like model with N coupled particles in double wells *is* a reasonable proxy for a thermodynamic circuit. This is where the 1-particle toy becomes a real circuit model.

### Step 3: Define a concrete X → Y task

Not just "maximize synchrony" but something a chip engineer would recognize: *given a 4-cell coupled circuit, learn an injection protocol that steers the system from initial state X to target configuration Y.* The injection protocol is the X-encoding. The target configuration is the computation result. The fidelity is measurable.

### Step 4: Measure τ_circuit / τ_noise

This is the scientific headline. Compute the dimensionless ratio τ_circuit / τ_noise for the coupled model. Sweep it by varying coupling strength, noise temperature, and injection rate. Show that different ratios produce different optimal encoding strategies. Show that the boundaries between strategies correspond to the 5 biological regimes. This is the direct test of the framework.

### Step 5: One paper

*"Biology-inspired X-encoding design rules for thermodynamic circuits, validated in Langevin simulation."*

Contents: the framework (5 regimes, 12 principles), one principle validated end-to-end on a multi-cell model, the τ_circuit / τ_noise characterization with regime boundaries, and concrete design rules with quantitative predictions.

### Step 6: Hardware collaboration

Take the paper and predictions to Extropic and Normal Computing. "Here are design rules derived from biological navigation principles. Here are the quantitative predictions. Test them on your chip." Both companies have explicit partnership programs seeking exactly this kind of cross-domain theoretical contribution.

## What's NOT on the Path

- Running all 7 biological experiments before any one is publication-ready
- Building infrastructure for experiments that need physics models we don't have (CFD for ctenophores, tentacle MJCF for octopus)
- Testing all 8 algorithms on every setup — 2-3 algorithm classes is sufficient for validation
- Polishing simulation results beyond what's needed for the paper

The bottleneck isn't more experiments. It's step 3 — moving from a toy model to something that maps onto real hardware.
