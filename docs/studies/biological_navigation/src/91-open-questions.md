# Open Questions

## Answered by This Work

1. **Does stochastic resonance scale from single particles to coupled circuits?** Yes. The SR peak persists across coupling strengths J = 0–2 and circuit sizes N = 4–16. Two regimes emerge: weak coupling (J < 1.5, peak kT ≈ 2.3) and strong coupling (J ≥ 2, peak kT ≈ 4.3). See the Noise Tuning chapter.

2. **Does the optimal noise level require retuning at larger circuit sizes?** No. Peak kT ≈ 2.8 holds from N=4 to N=64 at J=1.0. Confirmed by both the original 3-size sweep and the expanded 8-size sweep with finer kT resolution. See the Scale-Invariance chapter.

3. **Does phase-lagged injection improve fidelity in coupled chains?** Yes, at weak coupling. Optimal δ ≈ π/5 gives 18–37% improvement over synchronized injection at J = 0.5–1.0. At J ≥ 2, coupling handles coordination and synchronized is optimal. See the Injection Timing chapter.

4. **Does topological encoding dominate amplitude in the Langevin domain?** No. The scallop theorem requires time-reversible dynamics. Langevin dynamics are time-irreversible, so amplitude is a direct lever. Synchrony scales linearly with signal amplitude. See the Noise Tuning chapter, Level 4.

5. **Is there a sharp bifurcation point for sensitivity amplification?** No. The ΔV axis shows a broad sensitivity plateau (ΔV/kT ∈ [0.25, 2.75]) with a gradual trapping cutoff, not a sharp transition. See the Noise Tuning chapter, Operating Envelope.

6. **Does synchrony improve with circuit size?** No. An expanded sweep across N = 4, 8, 12, 16, 24, 32, 48, 64 (40 kT points in [1.0, 5.0], 40 episodes) shows peak synchrony is flat at ~0.058–0.071 with no significant trend (power law α = -0.037, |t| = 1.38, not significant). The preliminary increase from the 3-size sweep was a discretization artifact on the coarse 25-point grid. The system is approximately extensive — the design rules hold without retuning across a 16× scale range, but fidelity does not improve for free. See the Scale-Invariance chapter.

## Still Open

7. **Is the weak/strong coupling crossover a phase transition?** The current data shows two discrete modes with a transition between J = 1.5 and J = 2.0. Whether this is smooth or sharp (and whether it has the character of a thermodynamic phase transition) is unknown. See Next Experiment 2.

8. **What is the correct dimensionless ratio for thermodynamic circuits?** We proposed τ_circuit / τ_noise as the analog of the Reynolds number. The experiments used kT as the control variable and J as the coupling parameter, but the fundamental dimensionless group that governs the behavior remains to be identified. The ratio ΔV/kT ≈ 1.39 at the SR peak may be part of it.

9. **Which biological principles transfer and which don't — is there a general rule?** This work found that statistical-mechanical questions (noise tuning, phase coordination, extensivity) transfer to the Langevin domain, while dynamical-systems questions (topological invariants, bifurcation sensitivity) do not. Is this boundary precise? Does it hold for other Langevin systems beyond Ising chains?

10. **How do these design rules map to real thermodynamic computing hardware?** The experiments use idealized double-well potentials with nearest-neighbor coupling. Real circuits (Josephson junctions, molecular switches, optical bistable elements) have different noise statistics, coupling topologies, and operating timescales. Connecting the design rules to specific hardware parameters is the next major step.
