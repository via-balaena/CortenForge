# Open Questions

The following are the highest-priority open questions identified by this research program, ordered by tractability:

1. **What is the correct dimensionless ratio for thermodynamic circuits?** We proposed τ_circuit / τ_noise as the analog of the Reynolds number. Is this the right formulation? What are τ_circuit and τ_noise precisely in terms of circuit parameters (resistance, inductance, capacitance, temperature)?

2. **Are the five regimes discrete or continuous?** The biological spectrum suggests discrete transitions, but this may be because biological organisms specialize rather than operate across regimes. A thermodynamic circuit might be engineered to have a smooth transition — or the transitions might be genuinely discontinuous (phase transitions in the physics sense).

3. **What is the minimum encoding sequence length for topological encoding?** The scallop theorem says you need at least two degrees of freedom sweeping a nonzero area. What is the minimum area required for a given encoding fidelity? How does this scale with circuit noise temperature?

4. **Does the counter-rotating vortex pair principle have a direct electrical/stochastic analog?** The peregrine's mechanism relies on specific vortex geometry. What is the equivalent concept in Langevin dynamics — two perturbation modes with opposite "rotation" in the energy landscape that cancel each other's dissipation while preserving their net drift?

5. **How does the optimal noise level (stochastic resonance optimum) scale across the spectrum?** In Regime 1 (E. coli), there is an optimal CheY-P noise level. Is there an analogous optimal noise temperature for each regime? Does it increase or decrease as throughput increases?

6. **What are the failure modes at regime transitions?** When a circuit running a Regime 3 strategy is pushed into the Regime 4 throughput range, what specifically breaks? Does the encoding fail gradually or catastrophically?

7. **Can the fish lateral line be implemented as an X-encoder feedback system?** Specifically: can a distributed array of local circuit observables (the lateral line analog) provide sufficient information for real-time X-encoder adjustment without requiring a global measurement step?
