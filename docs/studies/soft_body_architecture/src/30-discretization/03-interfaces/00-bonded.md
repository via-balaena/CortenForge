# Bonded interfaces

The [interfaces parent](../03-interfaces.md) named bonded as the Phase B default. This leaf writes down what "bonded" means at the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) and [`material/`](../../../110-crate/00-module-layout/00-material.md) module level — the mesh is continuous, the per-tet material assignment varies, and no special interface element is needed — and identifies the regression-test requirement that prevents silent material-discontinuity bugs.

## What "bonded" means at the mesh level

A bonded interface between two materials A and B is represented by a **single continuous mesh** spanning both regions, with each tet tagged with the material that occupies it. There is no double-layer of vertices on the interface (every vertex is shared between A-tets and B-tets that touch it), no explicit interface element, no constraint enforcement — the displacement field is automatically continuous across the interface because the FEM basis functions are continuous across element boundaries by construction.

The data-structure picture:

- The [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module's vertex array is a single shared list; vertices on the A↔B interface have neighbors on both sides.
- The per-tet material assignment from [Part 7 Ch 02](../../../70-sdf-pipeline/02-material-assignment.md) returns the A-material parameters for tets on the A-side and the B-material parameters for tets on the B-side. A tet straddling the interface (centroid inside A but vertices touching B-tets) is tagged with whichever material the centroid sample selects; the [Ch 02 sharp-transitions sub-leaf](02-sharp-transitions.md) handles the within-tet-blending case more carefully.
- The [`element/`](../../../110-crate/00-module-layout/01-element.md) module's per-element loop calls [`Material::tangent()`](../../../20-materials/00-trait-hierarchy/00-trait-surface.md) with the per-tet parameters; A-tets call the A-material impl, B-tets call the B-material impl, and the trait surface does not change.

The result: a silicone-on-rigid-mount scenario — a silicone sleeve bonded to a steel core — is one continuous mesh with steel-elastic tets in the core and silicone-hyperelastic tets around it, all in the same `MixedTetMesh` data structure. No "bond element" or special joint primitive exists.

## The Phase B simplicity argument

Bonded interfaces are the Phase B default for three reasons:

- **No new module surface.** The required machinery is the per-tet material assignment that [Part 7 Ch 02](../../../70-sdf-pipeline/02-material-assignment.md) already covers and the [`Material` trait composability](../../../20-materials/00-trait-hierarchy/01-composition.md) that [Part 2 Ch 00](../../../20-materials/00-trait-hierarchy.md) requires. The mesh doesn't need a new tet-type; the solver doesn't need a new element-type; the `material/` module doesn't need a new trait method.
- **Strong out-of-the-box correctness.** Continuity across the interface is structural, not enforced. The displacement field cannot drift apart across the interface because there is only one displacement field. Any test the solver passes at the bulk-mesh level passes at the interface-mesh level by the same argument.
- **The canonical-problem workload mostly bonds.** The [Part 1 Ch 00 canonical-formulation](../../../10-physical/00-canonical/00-formulation.md) silicone-sleeve-on-rigid-mount is bonded; the [Part 1 Ch 04 carbon-black composite](../../../10-physical/04-material-data/02-carbon-black.md) skin-on-core is bonded; pure-soft-body without internal interfaces is the trivial bonded case. The sliding case ([Ch 01 sliding sub-leaf](01-sliding.md)) is the smaller fraction of the workload and is the Phase H deliverable.

## What can go wrong silently — and how the test gate prevents it

The risk in bonded interfaces is **material-assignment ambiguity at the interface** — a tet straddling A and B is assigned A's material when it should have been B's, or vice versa, and the simulation runs to "completion" with a subtly wrong stiffness distribution at the interface. The wrong assignment is silent: the solver converges, the gradients are well-defined, the visual output looks plausible. The error shows up only as a quantitative discrepancy in the stress field at the interface, which the user may not notice without a regression baseline.

`sim-soft`'s answer is that the [Part 11 Ch 04 regression-test sub-leaf](../../../110-crate/04-testing/01-regression.md) requires bonded-interface test cases to run against an analytic baseline (where one exists — bonded-bilayer beam-bending has analytic solutions for linear-elastic composites) or against a fine-mesh reference. The interface-stress regression catches the assignment-ambiguity case at CI time rather than at end-user-debugging time. A material-assignment bug that doesn't cross any analytic threshold is a Phase H concern only if it causes visible artifacts; bonded-interface validation against a known reference is the standard Phase B gate.

## Composition with the Tet4↔Tet10 mixed-element machinery

A bonded interface and a Tet4↔Tet10 element-type boundary are independent — both can occur in the same mesh, at the same or different locations. A silicone-on-rigid-mount mesh might have:

- The rigid steel core as Tet4-elastic (no Tet10 needed, no high-strain region in steel)
- The silicone sleeve as Tet4-hyperelastic in the bulk and Tet10-hyperelastic in the contact band ([Ch 00 mixed-element pattern](../../00-element-choice/03-mixed.md))
- The bonded steel↔silicone interface running through the Tet4-elastic / Tet4-hyperelastic transition

The [Tet4↔Tet10 conformity machinery](../../00-element-choice/03-mixed.md) handles the element-type boundary; bonded-interface continuity is a free consequence of the shared mesh. The two mechanisms compose because they live at different layers — element type at `element/`, material assignment at `material/` and the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) layer.

## What this sub-leaf commits the book to

- **Bonded interfaces require no new module surface.** The [per-tet material assignment](../../../70-sdf-pipeline/02-material-assignment.md), the continuous mesh, and the [`Material` trait](../../../20-materials/00-trait-hierarchy/00-trait-surface.md) together cover the bonded case. No interface element, no joint primitive, no constraint enforcement.
- **Continuity is structural.** Displacement continuity across the bonded interface is a property of the FEM basis, not of any explicit constraint. Tests do not need to verify it; the construction guarantees it.
- **Material-assignment ambiguity at the interface is the silent failure mode, caught at the regression-test layer.** [Part 11 Ch 04 regression](../../../110-crate/04-testing/01-regression.md) requires interface-stress test cases against analytic or fine-mesh references for any new bonded-multi-material configuration the [`SdfField` library](../../../70-sdf-pipeline/02-material-assignment.md) supports.
- **Bonded composes orthogonally with the Tet4↔Tet10 element-type machinery.** The two live at different module layers and do not interact through any shared data structure beyond the shared `MixedTetMesh`.
