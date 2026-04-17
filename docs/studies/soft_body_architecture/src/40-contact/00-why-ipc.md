# Why IPC beats penalty and impulse

Contact is where `sim-soft`'s [Ch 03 thesis](../10-physical/03-thesis.md) has its sharpest commitment. IPC — the incremental potential contact method of Li et al. 2020 — is the only formulation in the running. Penalty contact and impulse contact, which dominate the games and scientific-FEM branches respectively, both fail in specific ways that are structurally the same failure and that IPC is designed to eliminate. This chapter's sub-chapters walk each failure mode and the IPC guarantee that corresponds to it.

| Section | What it covers |
|---|---|
| [Penalty method failure modes](00-why-ipc/00-penalty.md) | Overlap-proportional force; non-energy-conservative snap-through; stiffness-vs-stability tradeoff that forces either penetration or oscillation |
| [Impulse method failure modes](00-why-ipc/01-impulse.md) | Discrete-time LCP or PGS solvers; no gradient through the contact transition; coefficient-of-restitution picking winners; fragile under stacked contact |
| [IPC guarantees](00-why-ipc/02-ipc-guarantees.md) | Zero penetration (structurally, not approximately); $C^2$ smooth total potential energy; warm-start across Newton iterations; differentiable through contact barrier |

Two claims Ch 00 rests on:

1. **The visible failures from [Part 1 Ch 02](../10-physical/02-what-goes-wrong.md) are the same algorithmic property that breaks differentiability.** Penalty contact pops because its force is a discontinuous function of the separation gap at the contact boundary; that same discontinuity kills autograd (gradients are undefined at the boundary) and kills Newton's convergence (the tangent jumps). Impulse contact pops because its collision response is an event-driven discrete change in velocity; that same event-drivenness has no gradient. IPC's $C^2$-smooth barrier fixes the popping and the non-differentiability with one mechanism. The thesis commitment to "differentiability as a free byproduct of doing the physics right" is, at the contact layer, specifically this: getting contact correct means getting it smooth, and smoothness is what autograd needs.
2. **IPC is not chosen because it is newer; it is chosen because the alternatives are structurally wrong for this stack.** Penalty works in games because games tolerate overlap as a visual artifact; `sim-soft` cannot, because the [reward function](../10-physical/01-reward.md) reads contact pressure per element and overlap corrupts the reward. Impulse works in rigid-body dynamics because rigid bodies have no finite-element-level stress to corrupt; `sim-soft`'s stress field is the primary output. IPC is the only mainstream contact formulation whose failure modes do not cross either of those tripwires.
