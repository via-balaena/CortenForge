# Synthesis: What Transfers and What Doesn't

## The Results

We tested five biological navigation principles on Ising chain models under Langevin dynamics. Three validated. Two failed.

| Principle | Regime | Question Type | Result |
|-----------|--------|---------------|--------|
| P2 — Stochastic resonance | E. coli | Statistical mechanics | **Validated** |
| P4 — Metachronal coordination | Ctenophore | Statistical mechanics | **Validated** |
| P6 — Scale-invariant encoding | Octopus | Statistical mechanics | **Validated** |
| P1 — Topological encoding | E. coli | Dynamical systems | **Failed** |
| P11 — Deliberate instability | Peregrine | Dynamical systems | **Failed** |

The pattern is not random. It divides cleanly along a line.

## The Boundary

**What transfers:** Principles that ask statistical-mechanical questions — optimal noise level (P2), injection timing coordination (P4), extensivity at scale (P6). These are questions about how to exploit the statistics of a stochastic system. The Langevin framework is statistical mechanics, so it speaks this language natively.

**What doesn't transfer:** Principles that ask dynamical-systems questions — topological invariants of motion sequences (P1), sharp bifurcation sensitivity (P11). These require mathematical structures the Langevin domain does not have: time-reversibility (for the scallop theorem) and sharp phase transitions at finite system size (for bifurcation amplification).

This is not a weakness of the framework. It is the framework's boundary condition — and knowing the boundary is as valuable as knowing the interior.

## The Design Rules

Three quantitative rules emerged, each tested with statistical gates and reproducible code:

**Rule 1 (Noise tuning — P2):** For an Ising-coupled bistable circuit with coupling J:
- J < 1.5: operate at kT ≈ 2.3
- J ≥ 2.0: operate at kT ≈ 4.3
- Keep ΔV/kT < 3.0 (hard trapping cutoff above this)
- The kT axis is sharp (±30% degrades to noise floor). The ΔV axis is forgiving.

**Rule 2 (Injection timing — P4):** For coupled circuits at moderate coupling:
- J < 2: inject with phase lag δ ≈ π/5 between adjacent nodes (18–37% improvement)
- J ≥ 2: synchronize injection (coupling handles coordination)
- The two knobs (temperature and phase lag) are independent

**Rule 3 (Scale-invariance — P6):** The noise tuning rule holds from N=4 to N=16 without retuning:
- Optimal kT ≈ 2.5 at J=1.0 regardless of circuit size
- Preliminary data suggests synchrony may *increase* with N (under investigation)

## What the Failures Tell Us

**P1 (topology fails):** In the Stokes regime (Re < 1), the scallop theorem guarantees amplitude is irrelevant — only sequence topology matters. In the Langevin domain, there is no scallop theorem. Amplitude is a direct lever: synchrony scales linearly with signal strength. An engineer should use amplitude modulation freely.

**P11 (bifurcation fails):** The peregrine exploits pitch instability near a bifurcation point for sensitivity amplification. In the Ising chain, the ΔV axis shows a broad plateau, not a sharp transition. Langevin systems at finite N have smooth crossovers. An engineer should tune kT carefully (sharp peak) and not worry about ΔV (forgiving axis).

Both failures trace to the same root: the Langevin domain is nonlinear and time-irreversible, with smooth crossovers instead of sharp transitions. The mathematical structures that make topology and instability powerful in biology — geometric phase, bifurcation sensitivity — simply don't exist in this physics.

## The Complete Spectrum (Updated)

| Regime | Exemplar | Dominant Strategy | Langevin Transfer? |
|--------|----------|-------------------|--------------------|
| Viscosity-dominated (Re < 1) | E. coli | Noise exploitation (P2 ✓), topology (P1 ✗) | Partial — stat-mech yes, topology no |
| Intermediate (Re 1–1000) | Ctenophore | Phase coordination (P4 ✓) | **Yes** |
| Inertial distributed (Re 10³–10⁵) | Octopus | Scale-invariant encoding (P6 ✓) | **Yes** |
| Predictive inertial (Re 10⁵–10⁷) | Dragonfly | Forward model, min observables | Predicted yes (stat-mech questions) |
| High-inertial turbulent (Re > 10⁷) | Peregrine | Instability (P11 ✗), vortex coupling | Partial — instability no, vortex needs CFD |

The middle of the Reynolds number axis transfers cleanly. The extremes require physics the Langevin model doesn't contain.

## Implications for Thermodynamic Circuit Design

An engineer reading this document should take away three things:

1. **Tune noise, don't suppress it.** There is an optimal operating temperature for your circuit. It depends on coupling strength. The rules are in Chapter 1.

2. **Coordinate injection timing at moderate coupling.** Phase-lagged injection at δ ≈ π/5 gives 18–37% better fidelity than synchronized injection. At strong coupling, synchronize instead. The rules are in Chapter 2.

3. **These rules hold at scale.** You do not need to retune when scaling from 4 to 16 nodes. The rules are in Chapter 3.

And two things NOT to do:

4. **Don't try to operate near a bifurcation point.** The sensitivity-amplification story from biology doesn't apply. The design surface is smooth, not critical.

5. **Don't optimize sequence topology.** Use amplitude instead. It works linearly and doesn't require the time-reversibility constraint that makes topology powerful in Stokes flow.

## Relationship to Current Thermodynamic Computing Research

Extropic and Normal Computing are focused on chip fabrication and algorithm development. The X-encoding problem — how to inject inputs into a stochastic physical system — is acknowledged but not yet formally addressed in their published work. These design rules are complementary: they address the theory gap that enables the next generation of circuit design.

## Open Source Philosophy

This research is released openly. The code that produced every result is in the same repository. The reasoning: becoming the foundational reference for noise-exploiting thermodynamic circuit design creates more value than any IP protection could. The simulation infrastructure (CortenForge) is the durable asset.
