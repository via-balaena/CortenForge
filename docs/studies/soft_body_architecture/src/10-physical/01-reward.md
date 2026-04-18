# The reward function

Chapter 00 fixed the geometry; this chapter fixes what "good" means over that geometry. The conformity reward is a weighted composition of four scalar objectives, each one measuring a different failure mode that the naïve objective *contact everywhere* does not catch on its own. A sleeve that grips tight in one band and floats in the next scores perfectly under contact-coverage alone; a sleeve that transmits zero axial force scores perfectly under peak-pressure-bounded alone. Each objective is a regression test for the composition.

The four objectives and the composition rule that folds them into a single scalar for the optimizer are each their own sub-chapter. The separation is deliberate: Part 10's optimization loop treats the reward as a black box, so the definition of "good" has to survive on its own before it enters the optimizer.

| Objective | What it measures | Why it's separate |
|---|---|---|
| [Pressure uniformity](01-reward/00-pressure-uniformity.md) | Standard deviation of the contact pressure field over the intended contact surface | Tight-then-floppy is strictly worse than even-everywhere, even at the same total force |
| [Contact coverage](01-reward/01-coverage.md) | Fraction of the intended contact surface actually in contact | A perfectly uniform pressure over 40% of the surface is not conformity |
| [Peak pressure bounds](01-reward/02-peak-bounds.md) | Smooth barrier penalty on peak pressure as it approaches a damage threshold | Localized spikes damage soft tissue, debond inserts, and predict fatigue failure — a scalar penalty is not a Pareto dimension |
| [Effective stiffness bounds](01-reward/03-stiffness-bounds.md) | Smooth penalty growing as transmitted normal force falls below a minimum for given squeeze displacement | An infinitely compliant sleeve achieves perfect uniformity and does nothing |
| [Multi-objective composition and weighting](01-reward/04-composition.md) | Fixed-weight sum, Pareto front, or preference-learning alternative | The weighting itself is a design choice that interacts with Part 10's surrogate and active learning |

Three claims Part 1 Ch 01 rests on:

1. **Conformity is not coverage.** The four objectives are not redundant. Each one eliminates a sleeve that some other objective would call optimal. The composition rule is where the reward's character comes from.
2. **Every objective is smooth in the design parameters.** Hard constraints are encoded as smooth barriers in the same shape as IPC's contact potential — diverging near the forbidden regime, negligible away from it, differentiable everywhere the optimizer visits. This commits Ch 01 to the [Ch 03 thesis](03-thesis.md) one level up from the constitutive law: the reward is part of the differentiable stack, not a step function sitting outside it. Indicator functions and hard thresholds are rejected on principle.
3. **The reward must survive outside the solver.** Every objective in the table is defined in terms of fields the solver already exports (contact pressure per tet, contact indicator on each surface element, effective transmitted force), so the reward can be evaluated without knowing which solver produced the fields. The reward is a property of the physical outcome, not of the simulator's internals — which lets Part 10 optimize against measurements from a real printed part as naturally as against `sim-soft` output.
