# Scale-Invariance: Octopus Compressed Command

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

## Experiment 6 — Scale-Invariant Encoding (Principle 6)

### Scientific Question

Does the SR-optimal temperature hold at larger circuit sizes? If the Noise Tuning Rule requires retuning for each circuit size, they're lab curiosities. If they hold from N=4 to N=16 without adjustment, they're engineering tools.

This is the most important result for practical thermodynamic circuit design: **design rules hold at scale without retuning.**

### Experimental Design

Fix coupling J=1.0. Repeat the Noise Tuning temperature sweep at three chain sizes:

| Parameter | Value |
|-----------|-------|
| Chain sizes N | 4, 8, 16 |
| Coupling J | 1.0 (fixed) |
| kT range | [0.1, 15.0], 25 points log-spaced |
| Episodes per point | 40 |
| Total episodes | 3,000 |
| Runtime | 38 minutes (release) |

Same Ising chain setup as the Noise Tuning chapter's multi-particle experiment: double wells, nearest-neighbor coupling, oscillating field, synchrony metric.

### Results

**Principle 6: VALIDATED.** The SR-optimal temperature holds across a 4× range of circuit sizes.

| N | peak kT | peak sync | |t| | Interior? |
|---|---------|-----------|-----|-----------|
| 4 | 2.29 | 0.041 ± 0.011 | 3.77 | YES |
| 8 | 2.29 | 0.049 ± 0.008 | 5.82 | YES |
| 16 | 2.82 | 0.063 ± 0.009 | 7.15 | YES |

**Gates:**

| Gate | Test | Result |
|------|------|--------|
| 0 (Sanity) | All peaks significant and interior | PASS (all \|t\| > 2.708) |
| 1 (Scale-invariance) | Peak indices span ≤ 2 on 25-point grid | PASS (span = 1) |
| 2 (Monotone) | sync(N=16) ≤ sync(N=4) + 3σ | PASS |

**Analysis:**

N=4 and N=8 peak at exactly the same grid point (kT=2.29). N=16 peaks one grid step higher (kT=2.82) — a 23% shift across a 4× increase in circuit size. On the 25-point log-spaced grid, this is within one step of the reference.

The slight upward shift at N=16 is consistent with the effective-barrier model: a 16-particle chain has proportionally more interior particles (14/16 vs 2/4), each feeling coupling from both neighbors. The average effective barrier rises by ~8%, shifting the optimal noise temperature by a corresponding amount. This is a predictable, small correction — not a breakdown of the design rule.

Peak synchrony *appears* to increase with N on this 3-size sweep. However, the expanded N-scaling sweep (below) reveals this was a discretization artifact.

### N-Scaling Expansion (N=4 to N=64)

An expanded sweep tested 8 chain sizes with finer kT resolution to determine whether the apparent sync increase with N was a real scaling law.

| Parameter | Value |
|-----------|-------|
| Chain sizes N | 4, 8, 12, 16, 24, 32, 48, 64 |
| Coupling J | 1.0 (fixed) |
| kT range | [1.0, 5.0], 40 points log-spaced |
| Episodes per point | 40 |
| Total episodes | 12,800 |
| Runtime | 7.5 hours (release) |

| N | peak kT | peak sync | \|t\| |
|---|---------|-----------|-------|
| 4 | 2.28 | 0.071 ± 0.009 | 8.15 |
| 8 | 2.92 | 0.060 ± 0.007 | 8.97 |
| 12 | 2.48 | 0.058 ± 0.008 | 7.14 |
| 16 | 2.81 | 0.061 ± 0.007 | 9.46 |
| 24 | 3.18 | 0.058 ± 0.008 | 7.24 |
| 32 | 2.38 | 0.062 ± 0.007 | 9.28 |
| 48 | 3.18 | 0.064 ± 0.007 | 8.97 |
| 64 | 2.81 | 0.058 ± 0.006 | 9.59 |

**Power law test:** α = -0.037 ± 0.027, |t| = 1.38 (not significant at p < 0.01). There is no scaling law — peak synchrony is flat across a 16× range of circuit sizes.

**Peak kT stability:** The peak kT bounces between 2.28 and 3.18 without systematic drift (17.6% total variation around a mean of 2.75). The SR peak is broad and flat-topped; the exact optimum is noise-dominated, but the operating band is wide enough that this doesn't matter.

**Interpretation:** The system is approximately extensive. The preliminary increase from 0.041 to 0.063 in the 3-size sweep was a discretization artifact — the coarse 25-point grid over [0.1, 15.0] undersampled the peak, and the finer 40-point grid over [1.0, 5.0] resolves higher peak sync values at all chain sizes. The design rules hold without degrading across a 16× scale range, but fidelity does not improve for free. This is the expected behavior of a well-behaved statistical mechanical system.

### Design Rule (Principle 6)

For coupling J=1.0, the SR-optimal temperature is kT ≈ 2.8 (mean across 8 chain sizes), holding from N=4 through N=64 without retuning.

Combined with the Noise Tuning Rule: **For J < 1.5, operate at kT ≈ 2.3–2.8. For J ≥ 2.0, operate at kT ≈ 4.3. These rules apply regardless of circuit size in the range N=4–64.**

An engineer scaling a thermodynamic circuit from a 4-node prototype to a 64-node production system can use the same noise temperature. No retuning required.

> **Code:** [`sim/L0/therm-env/tests/ising_chain.rs`](../../../sim/L0/therm-env/tests/ising_chain.rs) — `ising_scale_invariant_sweep`

## Key References

- Gutfreund, Y. et al. "Organization of Octopus Arm Movements: A Model System for Studying the Control of Flexible Arms." *Journal of Neuroscience* 16 (1996)
- Sumbre, G. et al. "Octopuses Use a Human-like Strategy to Control Precise Point-to-Point Arm Movements." *Current Biology* 16 (2006)
- Levy, G. et al. "Motor Control in Soft-Bodied Animals." *Current Biology* 25 (2015)
- Davenport, J.S. et al. "Lessons for Robotics from the Control Architecture of the Octopus." *Frontiers in Robotics and AI* (2022)
- Mischiati, M. et al. "Neural Models and Algorithms for Sensorimotor Control of an Octopus Arm." *arXiv* (2024)
