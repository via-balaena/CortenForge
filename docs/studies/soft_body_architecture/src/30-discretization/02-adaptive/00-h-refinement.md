# Stress-driven h-refinement

The [adaptive parent](../02-adaptive.md) deferred the `sim-soft` commitment for h-refinement to [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md). This leaf is the **view from Part 3** — what h-refinement is at the discretization level, what it changes about the per-element data structures the [`element/`](../../../110-crate/00-module-layout/01-element.md) module manages, and the conformity constraint a refinement pass has to maintain. The [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) chapter writes down the trigger criterion, the red-green subdivision operation (Bey 1995), the unrefinement hysteresis, and the optimization-loop integration; this leaf does not duplicate them.

## What h-refinement does to the mesh data structures

A red subdivision of a parent tet (the operation [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) names) inserts vertices at the midpoints of the parent's 6 edges and replaces the parent with 8 child tets. From the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module's perspective, for the **Tet4 → Tet4 refinement case**:

- **Vertex count grows by up to 6 per refined parent** (the new corner-edge midpoint vertices). Fewer if any of the 6 parent edges already has a midpoint vertex from a previously refined neighbor — those midpoints are shared. The vertex array is an append-only list; existing vertices retain their indices.
- **Tet count grows by 7 per refined parent** (1 parent → 8 children, net +7). Parent tet IDs are released or marked invalid; child tet IDs are appended.
- **Per-tet `ElementType` tags** ([mixed-element sibling](../../00-element-choice/03-mixed.md)): each child inherits its parent's tag.

Refinement of a **Tet10-tagged parent** is more expensive in vertex bookkeeping: in addition to the up-to-6 corner-edge midpoints (which become the children's corners, since a Tet10 parent's edges already have midpoints), each of the 8 Tet10 children needs its own 6 edge midpoints — most of which are new vertices at quarter-positions of the parent's edges or on parent face/interior. The `mesh/` module amortizes this through bulk allocation, but the per-event cost is higher than the Tet4 case. Phase H production usage applies refinement primarily to the Tet4 region; Tet10-region refinement is supported by the data structures but not the typical case.

The [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module's data layout is amenable to this — the vertex and tet arrays are append-only lists with indirection through ID maps, so refinement inserts new entries without invalidating the IDs of unrefined elements. The cost is per-refinement-event re-allocation if the underlying `Vec` capacity is exceeded; production usage pre-allocates with a refinement-budget margin.

## The conformity constraint at refinement boundaries

Red subdivision of a single tet leaves its face-neighbors with a **non-conforming face**: the parent's face is now split into 4 sub-triangles by the 3 new midpoint vertices on its edges, but the unrefined neighbor's matching face is still 1 triangle. Without a fix, displacement interpolation across the shared face is discontinuous.

Two standard fixes — the literature is settled, [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) picks (a):

- **(a) Green closure** — refine the neighbor with a transitional pattern (split into 2 or 4 sub-tets depending on how many edges have midpoints) so that the shared face is conformingly subdivided on both sides. The neighbor's children are *transitional* tets, lower-quality than red-subdivision children but with bounded quality loss (per Bey's result the [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) chapter cites). Subsequent red refinements of the neighbor itself promote the green tets to red.
- **(b) Hanging-node constraints** — leave the neighbor unrefined and add a constraint at the hanging midpoint vertex requiring its displacement to equal the linear-interpolation value from the unrefined face's corners. This is the same constraint mechanism the [Tet4↔Tet10 interface in Ch 00](../../00-element-choice/03-mixed.md) uses, applied here to refinement boundaries instead of element-type boundaries.

The [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module supports both representations because the [Tet4↔Tet10 mixed-element infrastructure](../../00-element-choice/03-mixed.md) already provides the constraint-table machinery; green closures are the primary path because they keep the global DOF count bounded and the mesh fully unstructured-tet without special hanging-node handling everywhere downstream.

## Pointer to the implementation chapter

The [Part 7 Ch 03 chapter](../../../70-sdf-pipeline/03-adaptive-refine.md) covers — without duplication here:

- The stress-gradient trigger criterion $r_e = \max_{e' \in \mathcal{N}(e)} \|\sigma^e - \sigma^{e'}\|_F / (\|\sigma^e\|_F + \epsilon)$
- The red-green subdivision operation (Bey 1995)
- The unrefinement hysteresis ($r_e < r_\text{threshold}/4$)
- The integration with the [Part 6 differentiability](../../../60-differentiability/02-implicit-function.md) gradient API
- The "uniform refinement / contact-proximity refinement" rejected alternatives and the "off-by-default in Phase D, on in Phase H" config flag

This Part 3 leaf is the discretization-layer perspective — what refinement *is* at the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) and [`element/`](../../../110-crate/00-module-layout/01-element.md) module level, what conformity constraint it imposes, how it composes with the [mixed-element machinery](../../00-element-choice/03-mixed.md). The why-and-when is at Part 7 Ch 03.

## What this sub-leaf commits the book to

- **h-refinement implementation lives in [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md), driven by [`readout/`](../../../110-crate/00-module-layout/09-readout.md)'s stress output.** The criterion-evaluation and trigger logic are in `readout/`; the subdivision operation is in `mesh/`. No new module is added.
- **Green closure (option a) is the conformity treatment, not hanging-node constraints (option b).** Option (b) machinery exists for the [Tet4↔Tet10 mixed-element interface](../../00-element-choice/03-mixed.md) and is reusable, but the primary refinement path produces fully conforming meshes through green closures.
- **Refinement preserves per-element `ElementType` tags.** A Tet4-tagged parent produces Tet4-tagged children; a Tet10-tagged parent produces Tet10-tagged children. Element-type changes (Tet4 → Tet10) are independent of refinement and live in the [mixed-element band machinery](../../00-element-choice/03-mixed.md).
- **The how-much-and-when policy is at Part 7 Ch 03.** The Part 3 chapter is about taxonomy and data-structure mechanics; Part 7 Ch 03 is about the optimization-loop policy and the off-by-default-in-Phase-D, on-in-Phase-H gating.
