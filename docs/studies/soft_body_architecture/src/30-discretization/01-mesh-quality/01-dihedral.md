# Dihedral angle bounds

The [mesh-quality parent](../01-mesh-quality.md) named dihedral-angle bounds as the second ingest-time gate, complementing the [aspect-ratio sibling](00-aspect-ratio.md). This leaf writes down what a tet's dihedral angles are, why two failure modes (angles approaching 0° and angles approaching 180°) need to be checked separately rather than as one combined "non-regular" condition, and why the dihedral-angle gate catches a class of bad tets that the radius-ratio metric does not catch in isolation.

## What the dihedral angle measures

A tetrahedron has 6 dihedral angles, one per edge — the angle between the two triangular faces sharing that edge. For a regular tet (all faces equilateral, all edges equal), every dihedral angle equals $\arccos(1/3) \approx 70.53°$. For a sliver-tet whose four vertices are nearly coplanar, the dihedral angles split into a pattern of very small angles ($\to 0°$) along the "thin" edges and very large angles ($\to 180°$) along the "wide" edges.

Per-tet, `sim-soft` records the minimum and maximum of the 6 dihedral angles:

$$ \theta_\text{min}(t) = \min_{e \in \text{edges}(t)} \theta_e, \qquad \theta_\text{max}(t) = \max_{e \in \text{edges}(t)} \theta_e $$

and gates ingest on both bounds:

$$ \theta_\text{min}(t) \;\geq\; \theta_\text{lower}, \qquad \theta_\text{max}(t) \;\leq\; \theta_\text{upper} $$

with $\theta_\text{lower} > 0°$ and $\theta_\text{upper} < 180°$ both clearly bounded away from the limits. Specific threshold degrees are Phase D measurement deliverables, the same regression-stabilization process the [aspect-ratio sub-leaf](00-aspect-ratio.md) describes.

## Why two separate bounds

A tet with $\theta_\text{min} \to 0°$ has different solver pathology than a tet with $\theta_\text{max} \to 180°$, even when both can occur in the same sliver shape:

- **$\theta_\text{min} \to 0°$ (acute sliver).** The tet has at least one needle-like edge configuration. The shape function gradient $\nabla_X N_i$ for one of the corner nodes opposite this edge becomes very large in magnitude (the gradient is inversely proportional to the perpendicular distance from the corner to the opposite face, which approaches zero as the dihedral collapses). Large gradients inflate the assembled stiffness magnitude on a per-element basis, distorting the global conditioning toward modes that should not be stiff.
- **$\theta_\text{max} \to 180°$ (cap sliver).** The tet has two faces nearly coplanar across an edge. The edge's outward normal direction is poorly defined; numerical issues in any per-edge geometric computation propagate (face-normal-dependent quadrature on Tet10, IPC-edge proximity tests, mesh-improvement edge-flip decisions). The tet itself may have moderate $\nabla_X N_i$ magnitudes but downstream geometric machinery degrades.

Both failure modes can co-occur in the same tet (and frequently do in pure sliver-tets), but a tet can fail one bound and pass the other (e.g., a "needle" tet with all small dihedrals on three edges and one moderate dihedral). Single-bound gates miss the asymmetric failure modes; two-bound gates catch all of them.

## What dihedral catches that radius ratio does not

Radius ratio $\rho = r_\text{ins}/r_\text{circ}$ is the [aspect-ratio sub-leaf](00-aspect-ratio.md)'s primary metric. Dihedral angles add diagnostic dimensions $\rho$ does not separate:

- A "needle" tet (one very long edge, three short edges all roughly equal) has small $\rho$ *and* small $\theta_\text{min}$. Both gates fire; either alone catches it.
- A "cap" or "wedge" tet (one very flat dihedral, others moderate) can have $\rho$ near the threshold (still small but not minimal) yet $\theta_\text{max} \to 180°$. The dihedral gate fires; the radius-ratio gate may not.
- A regular-but-rotated tet has $\rho = 1/3$ (regular) and dihedrals all at $70.53°$ (regular). Both gates pass; rotation does not affect either.

Because the gates catch overlapping but distinct failure regions, `sim-soft` uses both — not because either is insufficient on its own for the average bad tet, but because bad-tet shape distributions in the wild include the asymmetric cases where one metric registers and the other does not.

## Tet4 / Tet10 split — same direction as aspect ratio

The Tet10 thresholds are tighter than the Tet4 thresholds, in the same direction the [aspect-ratio sub-leaf](00-aspect-ratio.md) imposes for the same reasons: Tet10's quadrature on a strongly-skewed element accumulates integration error faster than Tet4's centroid evaluation does. The [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) reads the per-element type tag and applies the appropriate $(\theta_\text{lower}, \theta_\text{upper})$ pair per tet.

## Reporting

Diagnostic statistics returned alongside pass/fail: $\theta_\text{min}$ across the mesh, $\theta_\text{max}$ across the mesh, 5th-percentile of per-tet $\theta_\text{min}$, 95th-percentile of per-tet $\theta_\text{max}$. These four numbers are what mesh authors use to tune upstream parameters; the binary verdict alone is not enough information for re-meshing decisions.

## What this sub-leaf commits the book to

- **Dihedral-angle bounds gate ingest separately from aspect-ratio bounds.** Both gates fire independently; a mesh passes only if both pass. The dihedral gate exists to catch the asymmetric-skew failures the radius-ratio gate alone misses.
- **Two-sided bounds: $\theta_\text{lower} \leq \theta_e \leq \theta_\text{upper}$ on every edge $e$.** Single-sided bounds (only minimum, only maximum) miss one or the other of the two failure-mode classes.
- **Tet10 thresholds are tighter than Tet4 thresholds, same as for radius ratio.** Per-element-type threshold dispatch happens at the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) layer.
- **Specific threshold degrees are Phase D regression-stabilization deliverables.** The [aspect-ratio sub-leaf](00-aspect-ratio.md)'s threshold-setting procedure applies identically here, with Tet4 and Tet10 stabilization runs producing the four threshold values $(\theta_\text{lower}^{\text{Tet4}}, \theta_\text{upper}^{\text{Tet4}}, \theta_\text{lower}^{\text{Tet10}}, \theta_\text{upper}^{\text{Tet10}})$.
