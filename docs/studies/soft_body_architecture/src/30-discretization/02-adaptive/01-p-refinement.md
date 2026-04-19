# p-refinement

The [adaptive parent](../02-adaptive.md) named p-refinement as **complementary to h-refinement** and committed `sim-soft` to a **manually-targeted** p-refinement pattern: Tet10 in the [user-authored band](../../00-element-choice/03-mixed.md), Tet4 elsewhere. Fully adaptive p-refinement (let the solver decide which elements get upgraded to higher order at runtime) is deferred beyond Phase H per [Ch 00 Claim 4](../../00-element-choice.md). This leaf writes down what p-refinement is at the discretization level, distinguishes the manual-static version `sim-soft` ships from the dynamic version it does not, and identifies what the dynamic version would require if a future revision adds it.

## What p-refinement is

p-refinement upgrades selected elements from a lower-order shape-function basis to a higher-order one, without subdividing the element geometrically. In `sim-soft`'s Tet4 + Tet10 element family, p-refinement is the Tet4 → Tet10 promotion: same 4 corner vertices, same tet, same volume, with 6 new midpoint nodes added on the edges and the shape functions promoted from linear to quadratic. The element's spatial extent does not change; the polynomial order of the basis representable inside it does.

Compared to h-refinement (subdivide a tet into 8 smaller tets, all with the same Tet4 basis), p-refinement is a different way to spend computational budget for the same accuracy goal:

- **h-refinement** improves accuracy by reducing element size $h$. Per-element cost stays constant per element type; total cost scales with the number of elements.
- **p-refinement** improves accuracy by raising shape-function order $p$. Per-element cost grows with the new order's quadrature and DOF count; total element count is unchanged.

For smooth solutions, p-refinement converges faster than h-refinement — the [Part 1 Ch 02 rim-deformation failure](../../../10-physical/02-what-goes-wrong/04-rim.md) is mostly a smooth-solution problem (smooth bending mode within an element), and Tet10's quadratic basis captures the bending mode without further subdivision. For solutions with stress concentrations (sharp corners, contact discontinuities), h-refinement is more efficient because adding more polynomial orders does not capture a discontinuity any better. The canonical-problem workload mixes both regimes; the [parent's Claim 1](../02-adaptive.md) "complementary, not alternative" framing holds.

## What `sim-soft` ships in Phase H — manual static p-refinement

The Phase H Tet10-in-band commitment is a static, user-authored p-refinement scheme. The user supplies a `band_sdf` ([Ch 00 mixed-element sub-leaf](../../00-element-choice/03-mixed.md)) that defines the contact-band region; tets whose centroids fall inside the band are tagged Tet10 at ingest, tets outside are tagged Tet4. The tagging is fixed for the duration of a simulation run; the [Tet4↔Tet10 conformity machinery](../../00-element-choice/03-mixed.md) handles the band-boundary edges via DOF elimination on midpoint nodes that lie on the boundary.

Static manual p-refinement is enough for the Phase H deliverable because:

- The contact band's spatial extent is predictable from the design — the user knows where the cavity meets the probe, and that band does not migrate during a single simulation episode.
- The tagging cost is paid once, at mesh ingest; per-step solver cost is the steady-state cost for the chosen tagging.
- The [Part 11 Ch 04 testing strategy](../../../110-crate/04-testing.md) regression suite tests against fixed band masks, keeping the regression surface bounded.

What it is not enough for: design-mode optimizer sweeps where the contact band moves between design points (the optimizer changes a probe-shape parameter and the contact area shifts). This is the dynamic p-refinement use case.

## What dynamic p-refinement would require — deferred

A dynamic p-refinement scheme that promotes Tet4 → Tet10 elements during simulation (rather than at ingest) would need:

- **Promotion criterion** — a runtime metric analogous to the [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) stress-gradient criterion that drives h-refinement, but tuned to detect *bending modes that the current Tet4 basis cannot represent*. This is harder to define than the h-refinement criterion: bending-mode error is hard to estimate from a Tet4 solution because the Tet4 solution itself doesn't carry the bending information.
- **Promotion operation** — insert 6 midpoint vertices on the promoted tet's edges, retag the tet as Tet10, update the `midpoint_nodes` constraint table (creating new constraints at any boundary where the promoted Tet10 now neighbors a Tet4), warm-start the solver state by interpolating midpoint displacements from the corner displacements.
- **Demotion operation** — the reverse for elements that no longer need Tet10. Demoting requires discarding the midpoint-node displacements and verifying that the Tet4 solution on the same tet does not have higher residual than a tunable hysteresis tolerance.
- **State-transfer correctness** — the [Part 6 Ch 02 implicit-function](../../../60-differentiability/02-implicit-function.md) gradient through a promotion event has to be defined; the simplest answer is to treat promotion as a non-differentiable event flagged with `GradientEstimate::Noisy { variance }` (mirroring [Part 7 Ch 03's h-refinement treatment](../../../70-sdf-pipeline/03-adaptive-refine.md)), but a stochastic-adjoint treatment via [Part 6 Ch 03](../../../60-differentiability/03-time-adjoint.md) is the post-Phase-H upgrade.
- **Test surface** — the [Part 11 Ch 04 regression suite](../../../110-crate/04-testing.md) has to certify promotion-driven runs against static-tag baselines without requiring per-design-point hand-authored band masks.

These requirements compound. The [parent Claim 4 deferral](../../00-element-choice.md) stands on the cumulative complexity, not on any single requirement being intractable.

## What this sub-leaf commits the book to

- **Phase H ships static manual p-refinement only.** Tet4/Tet10 tagging is determined by the user-supplied `band_sdf` at mesh ingest and does not change during simulation.
- **Dynamic p-refinement is post-Phase-H.** The required machinery — promotion criterion, promotion/demotion operations, state-transfer correctness, expanded test surface — is acknowledged but not in scope through Phase H.
- **p-refinement and h-refinement are complementary, not competing.** Phase H ships both: p-refinement (Tet10 in the contact band) and [h-refinement (stress-driven subdivision in stress-concentration regions)](00-h-refinement.md). The two paths run independently and compose at the same `mesh/` and `element/` modules.
- **The hard problem in dynamic p-refinement is the promotion criterion, not the operation.** Bending-mode error is hard to estimate from a low-order solution; this is the gap that distinguishes dynamic p-refinement from dynamic h-refinement (where the [Part 7 Ch 03 stress-gradient criterion](../../../70-sdf-pipeline/03-adaptive-refine.md) works cleanly off the converged low-order solution).
