# Regime 2: Ctenophore — Multimodal Switching

**Re ~ 1-1000 — The Intermediate Regime**

## The Physics

This is the most neglected and arguably most important regime. Both inertia and viscosity play significant and comparable roles simultaneously. There is no clean simplification. The governing equations are the full nonlinear Navier-Stokes, but at scales where both viscous and inertial terms contribute non-negligibly to thrust and drag. Crucially: thrust and drag are governed by different forces — at Re ~ 10², thrust is generated primarily by inertial forces while drag is still significantly viscous. No single strategy dominates.

The scallop theorem still technically applies (reciprocal motion produces no net displacement), but the breakdown of the theorem begins in this regime as inertial effects introduce memory into the fluid. A swimmer leaving a wake disturbs the flow it will encounter on its return, breaking exact time-reversibility.

## What Ctenophores Do

Ctenophores are among the oldest animals on Earth and the largest animals that use cilia to swim. Their ctene rows (arrays of fused cilia) beat in a metachronal wave — sequential, coordinated beating with a phase lag between adjacent appendages, creating the appearance of a traveling wave moving down each row. This strategy is notably distinct from either the purely viscous strategy of small ciliates or the inertial undulation of fish.

The metachronal wave achieves several things simultaneously: it creates non-reciprocal motion (satisfying the kinematic constraint of the scallop theorem while inertia begins to provide memory), it generates hydrodynamic interactions between adjacent paddles that increase efficiency beyond what any single paddle could achieve alone, and it provides omnidirectional maneuverability by independently modulating different ctene rows.

Critically: ctenophores can perform tight turns while maintaining forward swimming speeds close to 70% of their maximum — a performance metric comparable to or exceeding vertebrates with far more complex locomotor systems. This is multimodal switching in action: the system does not stop and reorient, it redistributes effort across different control surfaces in real time.

## What Water Boatmen Do

Water boatmen (Corixidae, Re ~ 10-200) operate in the most confused part of the intermediate regime. They use drag-based paddling with asymmetric power and recovery strokes — a direct application of temporal asymmetry to break time-reversibility. Their energetic efficiency is lower than fish of comparable size, but they are trimodal: they can swim, walk, and fly, transitioning rapidly between locomotor modes as the physical environment demands. This mode switching is the characteristic strategy of the intermediate regime.

## The X-Encoding Principle

**Principle 3:** In the intermediate regime, no single encoding strategy dominates. The optimal approach combines temporal asymmetry (power and recovery phases at different rates) with distributed spatial coordination (metachronal waves) and mode switching (dynamically selecting between encoding strategies as local conditions shift).

*Applied to thermodynamic circuits:* At intermediate throughput rates — fast enough that purely topological encoding is insufficient, slow enough that inertial coupling effects haven't emerged — the X-encoder must operate as a multimodal adaptive system. It should maintain multiple encoding strategies simultaneously and switch between them based on local observables of the circuit's stochastic state. This is the regime where a Kalman filter architecture is most directly applicable: continuously estimating which encoding mode is appropriate given the observed noise floor and relaxation rate.

**Principle 4:** Distributed coordination across multiple control points (metachronal wave) produces emergent efficiency that no single control point could achieve. Applied to multi-cell thermodynamic circuits: coordinating injection timing across circuit nodes with a phase lag (analogous to a metachronal wave) may produce higher encoding fidelity than simultaneous or independent injection.

## Experiment 2 — Ctenophore Metachronal Phase-Lag Optimization

Simulate a row of N ctenes beating with a variable phase lag δ in both the viscous and intermediate Re regime. Measure thrust and encoding efficiency as a function of δ and Re. Identify whether there is a characteristic phase lag that maximizes efficiency at the viscous-inertial crossover. This translates directly to a design parameter for multi-node injection timing in thermodynamic circuits.

> **Platform readiness:** Low — needs a simplified drag model (no CFD). The multi-particle `ThermCircuitEnv` supports N particles, but there is no hydrodynamic coupling between them.
>
> **Status:** Not started
>
> **Code:** —

## Key References

- Byron, M.L. "Moving in the In-Between: Locomotion Strategies at Intermediate Reynolds Numbers." *Princeton MAE Seminar* (2022)
- McHenry, M.J. et al. "The Hydrodynamics of Locomotion at Intermediate Reynolds Numbers." *Journal of Experimental Biology* 206 (2003)
- Daniels, J. et al. "The Hydrodynamics of Swimming at Intermediate Reynolds Numbers in the Water Boatman." *Journal of Experimental Biology* 217 (2014)
- Hoover, A.P. et al. "Omnidirectional Propulsion in a Metachronal Swimmer." *PLOS Computational Biology* (2023)
