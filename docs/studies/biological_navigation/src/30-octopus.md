# Regime 3: Octopus — Compressed Command

**Re ~ 10³-10⁵ — The Inertial Distributed Regime**

## The Physics

Inertia dominates. Viscosity still contributes to drag but does not govern the locomotion strategy. The fluid has memory — perturbations persist and propagate. The swimmer generates wakes that interact with the environment on relevant timescales. But this is also the regime of high degrees of freedom: the control problem is not underdetermined (as at low Re where everything scales out) but overdetermined — there are more controllable degrees of freedom than there are dimensions of the desired output, and the control architecture must manage this redundancy efficiently.

## What the Octopus Does

The octopus has eight arms with virtually infinite degrees of freedom. Each arm is a muscular hydrostat — no rigid skeleton, with muscles providing both structural support and actuation. The arm contains approximately 380,000 motor neurons distributed along its length, yet the brain controls all of them via only ~4,000 efferent nerve fibers — a compression ratio of roughly 100:1.

This is the key insight. The central brain does not specify each muscle's activation individually. It sends a compressed command that specifies the goal (reach toward target at location X), and the arm's own distributed neural circuitry handles the decomposition of that command into the specific muscle activations required to execute the motion in the specific stochastic fluid environment the arm is currently in.

The primary locomotion primitive is bend propagation: the brain initiates a bend at the base of the arm, which travels as a wave from base to tip. Different reaching movements vary in speed and distance but maintain a basic invariant velocity profile scaled appropriately. The same motor program, scaled, generates all reaches. This scale-invariance is fundamental — it means the encoding strategy does not need to be re-designed for every target; one parametric program covers the full range.

Sensory information flows back in the opposite direction: ~2.3 million receptors distributed along the arm send information to the brain via only ~17,500 afferent fibers. Most sensory processing happens locally in the arm's peripheral nervous system. The brain receives a summary, not raw data.

The architecture: compressed high-level command → distributed local decoding → precise physical execution → compressed sensory summary → updated command.

## The X-Encoding Principle

**Principle 5:** In the inertial distributed regime, the optimal architecture separates the encoding problem into two layers: a compressed high-level specification of the desired output (what state to reach) and a distributed local computation layer that translates that specification into physical actions appropriate to local noise conditions. The central controller does not need to know the noise floor at every circuit node; the local layer adapts.

*Applied to thermodynamic circuits:* The X-encoder broadcasts a compressed description of the target energy basin to all circuit nodes. Each node runs a local computation (analogous to the arm's ganglion) that determines how to modulate its local Langevin dynamics to steer toward that basin, given the local noise statistics it observes. The circuit's distributed neural computation — its local stochastic dynamics — handles the translation. This is the thermodynamic analog of bend propagation.

**Principle 6:** Scale-invariance of the encoding program. A parametric injection wave that can be scaled in amplitude and velocity to cover a range of target states (analogous to the octopus's invariant velocity profile) is more powerful than a lookup table of specific injection sequences for each target. Scale-invariance is also a prerequisite for scaling the circuit itself — an encoding strategy that must be re-tuned for each circuit size is not an engineering tool.

## Experiment 3 — Octopus Bend Propagation Compression Ratio

Simulate octopus arm reaching using CortenForge's Hill-type muscle model with the 4,000:380,000 efferent compression architecture. Measure the minimum central command dimensionality required to achieve a given reaching accuracy in a stochastic fluid environment. Characterize how this compression ratio scales with arm length and noise level.

> **Platform readiness:** Medium — Hill muscles exist in the sim crate, but needs an MJCF tentacle model with the appropriate compression architecture.
>
> **Status:** Not started
>
> **Code:** —

## Key References

- Gutfreund, Y. et al. "Organization of Octopus Arm Movements: A Model System for Studying the Control of Flexible Arms." *Journal of Neuroscience* 16 (1996)
- Sumbre, G. et al. "Octopuses Use a Human-like Strategy to Control Precise Point-to-Point Arm Movements." *Current Biology* 16 (2006)
- Levy, G. et al. "Motor Control in Soft-Bodied Animals." *Current Biology* 25 (2015)
- Davenport, J.S. et al. "Lessons for Robotics from the Control Architecture of the Octopus." *Frontiers in Robotics and AI* (2022)
- Mischiati, M. et al. "Neural Models and Algorithms for Sensorimotor Control of an Octopus Arm." *arXiv* (2024)
