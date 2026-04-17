# Higher-order elements as alternative

The [mixed u-p sibling](01-mixed-up.md) and [F-bar sibling](02-f-bar.md) both keep Tet4 as the element type and cure locking at the formulation layer. Higher-order elements — Tet10 with quadratic shape functions, or Hex8 with trilinear shape functions — approach locking from a different direction: they enlarge the per-element DOF space so the over-constraint that locks Tet4 is less severe. This leaf states what higher-order elements cure, what they don't, and why `sim-soft` does not adopt them as the primary near-incompressibility treatment.

## DOF-to-constraint ratio

The [locking sibling](00-locking.md) named the Tet4 pathology as an over-constrained system: twelve DOFs per element plus effectively one per-tet incompressibility constraint, with the constraint count approaching the DOF count on typical meshes. Higher-order elements change this ratio.

- **Tet10** has 10 nodes × 3 DOFs = 30 DOFs per element, with the same one incompressibility constraint per tet (or a small number per tet if integrated at its standard 4-Gauss-point rule). The DOF-to-constraint ratio is roughly 2.5× what Tet4 provides.
- **Hex8** has 8 nodes × 3 DOFs = 24 DOFs per element, with a small number of constraints per hex integrated at standard $2\times 2\times 2$ Gauss-point rules. Again, a larger ratio than Tet4.

The larger ratio means the over-constraint that produces locking in Tet4 is milder in Tet10 and Hex8 — the deviatoric modes have more room to satisfy boundary conditions while the volumetric constraint holds. Not completely gone: strict incompressibility still requires additional treatment at higher order (selective reduced integration, or mixed-formulation composition), but the severity of the locking and the accuracy loss at moderate near-incompressibility are substantially reduced.

## What higher-order does not cure on its own

**Strict incompressibility ($\nu \to 0.5$) still requires formulation-level treatment.** A pure-displacement Tet10 element at $\nu = 0.499$ still locks, just less severely than Tet4. The cure for the strict-incompressibility limit remains mixed-u-p or equivalent, applied on top of the higher-order element. At Phase H (when the [Tet10 path lands per Part 3 Ch 00](../../30-discretization/00-element-choice.md)), `sim-soft`'s `MixedUP<M>` wrapping continues to apply; the Tet10 element type and the mixed-u-p formulation compose, and the composition cures locking for both the discretization reason (more DOFs) and the formulation reason (separate pressure field).

**Shear locking.** Tet10 and Hex8 under bending can exhibit shear-locking artifacts distinct from volumetric locking. The cure there is a different reduced-integration scheme (B-bar or Bathe-Dvorkin, depending on element type) applied to the deviatoric term. This is beyond the scope of this chapter, but the point is that higher-order elements don't eliminate all locking pathologies by default.

## Cost ratio — why Tet4 + mixed u-p wins

Per-element cost comparisons (order-of-magnitude, on `sim-soft`'s canonical problem):

- **Tet4 + mixed u-p** runs one Gauss point per tet (Tet4's native rule), 12 displacement DOFs per element, one pressure DOF per tet, and the [static-condensation reduced-system Cholesky path](01-mixed-up.md). Per-Gauss-point constitutive cost is the base hyperelastic evaluation plus a small decorator overhead.
- **Tet10** runs 4 Gauss points per tet (Tet10's standard rule, per [Part 3 Ch 00](../../30-discretization/00-element-choice/04-tradeoff.md)), 30 displacement DOFs per element, and the per-element stiffness block is $30\times 30$ instead of $12\times 12$. The per-tet arithmetic is a few-fold larger for shape-function gradient assembly and tangent contraction, and the 4 Gauss points each run the constitutive evaluation separately.

[Part 3 Ch 00 — cost/accuracy tradeoff](../../30-discretization/00-element-choice/04-tradeoff.md) names the textbook ratios: Tet10 assembly is several-fold more per-element than Tet4, and the resulting global stiffness matrix has roughly $2.5\times$ the nonzero count of the equivalent Tet4 mesh (more nodes per element means more per-row contributions in the sparse assembly). The Cholesky factorization cost scales super-linearly with nonzero count, so the solver time per step climbs accordingly.

Tet4 + mixed u-p adds one pressure DOF per tet to the 12 displacement DOFs and a small Schur-complement extension to the CSR pattern — a modest arithmetic overhead on top of plain Tet4. Compared to Tet10, the overhead is substantially smaller than the quadratic-shape-function cost delta, so Tet4 + mixed u-p is cheaper for comparable accuracy in the canonical problem's strain regime. This is the [Ch 05 parent's](../05-incompressibility.md) basis for picking mixed u-p as primary.

## Hex8 and the SDF pipeline

Hex8 has structural advantages: its per-element DOF count is high, its integration rules are well-understood, and it avoids some of the aspect-ratio pathologies Tet4 and Tet10 can exhibit. On meshes generated from CAD B-rep geometry (bricks and blocks), Hex8 is frequently the element of choice.

The problem is mesh generation from SDF. [Part 3 Ch 00's Hex8 rejection](../../30-discretization/00-element-choice/02-hex8.md) names this: robust SDF-to-hex meshing is an unsolved problem. The SDF-driven design primitive `sim-soft` commits to in [Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) does not have a reliable path to a hex mesh of the boundary-conforming quality Tet4 or Tet10 achieve via [fTetWild](../../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md). The state of the art in SDF-to-hex research produces meshes that are either non-boundary-conforming, or locally-non-hex (transition elements), or quality-poor in curvature regions.

Adopting Hex8 would therefore either require a mesh-generation research effort outside `sim-soft`'s scope, or require the design surface to become CAD B-rep rather than SDF — which conflicts with the [cf-design architecture commitment](../../10-physical/03-thesis.md) and the [mesh-IS-render-mesh thesis](../../90-visual/05-sim-bevy.md). The Hex8 alternative is therefore closed.

## When higher-order is chosen anyway

Tet10 lands at Phase H per [Part 11 Ch 03](../../110-crate/03-build-order.md), not because of its locking behavior but because of its accuracy improvement on the canonical problem's [rim-deformation failure mode](../../10-physical/02-what-goes-wrong/04-rim.md). At Phase D (Tet4-only), the rim is under-resolved and the sleeve's bending fidelity is known-limited; the [Part 3 Ch 00 commitment to Tet4 Phase B → Tet10 Phase H](../../30-discretization/00-element-choice.md) sets expectations explicitly.

At Phase H, Tet10 composes with `MixedUP<M>` the same way Tet4 does — the mixed-u-p decorator is element-type-agnostic in its structural form (the Lagrangian formulation, the saddle-point system, the static condensation) and in its implementation (the `Material` trait surface and per-tet assembly). The Phase H transition replaces the element-shape-function gradients in [`element/`](../../110-crate/00-module-layout/01-element.md) without touching `material/`.

## What this sub-leaf commits the book to

- **Higher-order elements mitigate locking, but do not fully cure it at $\nu \to 0.5$.** Tet10's larger DOF-to-constraint ratio reduces the over-constraint severity but does not eliminate it; formulation-level treatment (mixed u-p) remains necessary at strict near-incompressibility even with Tet10.
- **Tet4 + mixed u-p is cheaper than Tet10 for comparable accuracy on the canonical problem's strain regime.** Per-element cost ratios from [Part 3 Ch 00](../../30-discretization/00-element-choice/04-tradeoff.md) plus the modest Schur-complement overhead of static-condensated mixed u-p give Tet4 + mixed u-p the cost advantage; hence the [Ch 05 parent's](../05-incompressibility.md) primary-cure choice.
- **Hex8 is closed by the SDF → hex meshing problem.** Robust hex meshing from SDF is unsolved; adopting Hex8 would require either a mesh-generation research investment or a departure from the SDF design commitment, neither of which is on the Phase A–I roadmap.
- **Tet10 at Phase H composes with `MixedUP<M>` without re-architecting.** The mixed-u-p decorator is element-shape-agnostic; Tet10's landing at Phase H is a shape-function gradient change in [`element/`](../../110-crate/00-module-layout/01-element.md), not a `material/` change. Higher-order is an accuracy-driven choice, not a locking-driven one.
