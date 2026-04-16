# Transition Zones and the Lateral Line

The five regimes and their principles are the theoretical framework. The experimental results (Chapters 1–3) validated three principles and identified the statistical-mechanics / dynamical-systems boundary. The transition zones between regimes remain an important open question, because:

1. Real thermodynamic circuits will operate across a range of throughput levels, necessarily crossing multiple regime boundaries.
2. The biological exemplars suggest that transitions between regimes are discontinuous — there is no smooth interpolation of strategies, and performance degrades sharply in the transition zone before a new strategy takes over.
3. The intermediate regime (Regime 2) exists precisely because neither low-Re nor high-Re strategies work there, and the organisms that inhabit it (ctenophores, water boatmen) are notably less efficient than those that specialize in either adjacent regime.

**The transition zone hypothesis:** In thermodynamic computing, as injection throughput increases from the regime of one strategy to the next, there will be a characteristic throughput range where neither strategy works well — analogous to the intermediate Reynolds number regime. Identifying these critical throughput values for a given circuit architecture, and designing the mode-switching logic that handles the transitions, is a specific and tractable engineering problem that this research program can address through simulation.

## The Fish Lateral Line

There is a sixth organism that deserves dedicated study for its unique role at the boundary between Regimes 3 and 4: the fish with lateral line. The lateral line is an array of mechanosensory organs distributed along the fish's body that detects local pressure gradients and vortex shedding frequencies with remarkable precision. Fish use the lateral line to perform Karman gaiting — holding station in a turbulent vortex street behind a cylinder by passively synchronizing their body kinematics to the oscillating flow. This is energy harvesting from chaos: exploiting the environmental noise field as a source of free locomotion rather than fighting it. The lateral line provides the sensing architecture that makes this possible. The engineering analog is an in-situ noise characterization system embedded in the circuit itself, providing real-time observables to the X-encoder without interrupting computation.

## Experiment 6 — Regime Transition Characterization

For each pair of adjacent regimes, simulate a Langevin circuit operating at the strategy of the lower regime and systematically increase the injection rate. Measure when performance begins to degrade and what the transition regime looks like. Identify the critical τ_circuit / τ_noise ratio at each transition. Map these to concrete circuit parameters (injection rate, noise floor, relaxation time) to provide actionable design guidance for where to switch encoding strategies.

> **Platform readiness:** Low — this is a meta-experiment that requires 2-3 regime-specific strategies to be working first. It depends on Experiments 1, 4, and ideally 3 or 5.
>
> **Status:** Not started
>
> **Code:** —

## Experiment 7 — Fish Lateral Line In-Situ Sensing

Implement a lateral-line-style sensor array in the simulated circuit: distributed local pressure sensors (equivalent to neuromasts) providing real-time vortex shedding frequency and local gradient information. Test whether this in-situ sensing reduces the number of pre-measurement calibration steps required before the X-encoder can operate effectively. Quantify the steady-state sensing accuracy as a function of array density and sensor placement.

> **Platform readiness:** Low — needs a distributed pressure-gradient sensor plugin. The sim sensor infrastructure exists but does not include the specific sensor type needed.
>
> **Status:** Not started
>
> **Code:** —
