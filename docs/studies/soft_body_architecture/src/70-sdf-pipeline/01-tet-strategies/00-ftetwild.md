# fTetWild

`sim-soft` ships [Hu et al. 2020's fTetWild](../../appendices/00-references/03-diff-sim.md#hu-2020) as the default tet-generator for [design-mode](../../appendices/01-glossary.md) on canonical-scale meshes. This leaf names why — the single-property argument the [Ch 01 parent's Claim 1](../01-tet-strategies.md) rests on — and the adaptations required to drive a mesh-input pipeline from an `SdfField` instead.

## The envelope-based validity guarantee

fTetWild's core contribution is a pipeline that handles *arbitrary* input surface meshes, including self-intersecting, non-manifold, or otherwise-pathological ones, and produces a tet mesh with a formal guarantee: the output surface lies within an $\epsilon$-envelope of the input surface, and every tet is geometrically valid (positive volume, bounded aspect ratio). The envelope is a user-specified geometric tolerance; fTetWild adjusts the mesh until the envelope constraint is met or reports failure if it cannot.

The guarantee matters for `sim-soft` because the input surface is not a hand-authored, clean polygonal mesh — it is the iso-surface of a [composed SDF](../00-sdf-primitive.md) produced by `cf-design`, which can exhibit any surface pathology depending on the composition tree. A designer who subtracts a sphere from a cylinder at the point where they graze tangentially produces an SDF whose iso-surface has near-zero-thickness self-intersecting regions; a designer who composes a smoothly-blended union of three primitives in an unusual configuration can produce an iso-surface with high-genus connectivity that mesh-repair pipelines designed for hand-authored meshes would fail on. fTetWild's envelope pipeline handles both by construction.

Three consequences of the guarantee for `sim-soft`:

- **No upstream mesh-repair step.** The [Ch 00 §00 mesh-first failure modes](../00-sdf-primitive/00-why-sdf.md) (CGAL's self-intersection preconditions, coplanar-face slivers, T-junctions) are fTetWild's problem to solve, and it does. `sim-soft`'s `sdf_bridge/` does not ship a repair pipeline.
- **Iso-surface extraction is allowed to be imperfect.** The surface extracted from the SDF at grid points (standard marching-cubes or surface-nets) can have topology defects; fTetWild's envelope-based pipeline fixes them en route to producing the tet mesh. The extraction step just needs to produce a surface within $\epsilon$ of the true SDF iso-surface.
- **Envelope tolerance is a design parameter.** Tighter tolerances produce more-faithful surfaces but more tets; looser tolerances produce fewer tets but surface approximations that may move the contact geometry enough to matter. `sim-soft` carries an envelope-tolerance hint through the `sdf_bridge/` boundary (defaulted to a fraction of `resolution_hint`) and forwards it to fTetWild.

## Adapting from surface-input to SDF-input

fTetWild as published consumes a polygonal surface mesh as input. `sim-soft` consumes an `SdfField`. The adapter runs in three stages:

1. **Iso-surface extraction.** At the bounding box of the `SdfField` (populated from the field's `bbox`), evaluate the SDF at a grid with spacing set by `resolution_hint / 2` (Nyquist-style factor for feature capture). Run marching-cubes or surface-nets to extract the $\phi = 0$ iso-surface as a triangle mesh. The extraction is permitted to produce topology defects; fTetWild handles them.
2. **fTetWild on the extracted surface.** Feed the triangle mesh into fTetWild with the envelope tolerance and target edge length set from `resolution_hint`. fTetWild produces the interior tet mesh with aspect-ratio and dihedral-angle post-processing passes applied.
3. **Material-field sampling.** Walk the output tet mesh, sample [`MaterialField`](../02-material-assignment.md) at each tet's [centroid or Gauss point](../02-material-assignment/00-sampling.md), and attach the per-element material parameters.

The iso-surface extraction step is the only place the SDF is evaluated at a regular grid; downstream, the tet mesh is the source of truth for geometry and the SDF is only re-queried when [Ch 04's live re-mesh](../04-live-remesh.md) triggers a rebuild.

## Why design-mode, not experience-mode

Design-mode targets ~30k tets at canonical scale with re-meshing triggered only on [topology-changing edits](../04-live-remesh/00-change-detection.md). The meshing cost is paid once per topology change and amortized across many parameter-only edits. fTetWild's meshing time on canonical-scale problems (order-of-seconds) is acceptable at that cadence — a designer sweeping a parameter slider sees the mesh refresh within a perceptible but bounded latency, and the [warm-start machinery](../04-live-remesh/01-warm-start.md) handles the common case where no re-mesh is needed at all.

Experience-mode runs at 60 FPS across a frozen geometry with no re-meshing. fTetWild's robustness is overkill when the input is known-clean (the mesh was produced once at scene load and is never re-generated) and its meshing cost is prohibitive for a one-shot setup whose throughput-per-second is the deliverable. [Delaunay's simpler pipeline](01-delaunay.md) is the better fit for that regime.

## What fTetWild does not solve

fTetWild is a *mesh-generation* guarantee, not a differentiability guarantee. The output tet mesh is a piecewise-constant function of the input — an infinitesimal perturbation of the `SdfField` can produce the exact same mesh, but a large-enough perturbation crosses a topology boundary and the mesh structure changes discretely. This is [Part 6 Ch 05's open problem](../../60-differentiability/05-diff-meshing.md), not something fTetWild addresses. `sim-soft`'s response is the [FD wrapper](../../60-differentiability/05-diff-meshing.md) at the topology-crossing boundary, as the parent chapter commits to.

Nor does fTetWild guarantee the *runtime* of its pipeline. Worst-case pathological SDFs can push meshing time well beyond the design-mode interactive budget. `sim-soft`'s response is a bounded timeout on the meshing call with a fallback to Delaunay on a coarser mesh if fTetWild does not complete in time. The fallback produces a lower-quality mesh but preserves the design-mode interactive loop rather than stalling it.

## What this sub-leaf commits the book to

- **fTetWild is the default meshing pipeline for design-mode.** Its envelope-based validity guarantee on arbitrary input surfaces is the property that matters for SDF-driven meshing where iso-surface topology defects are routine.
- **The adapter is iso-surface extract → fTetWild → material sample.** Three stages, one grid evaluation of the SDF, one call into the fTetWild pipeline, one walk of the output mesh.
- **Envelope tolerance is a named `sdf_bridge/` hint.** Defaulted to a fraction of `resolution_hint`; designers can tighten or loosen per primitive. Carried alongside `SdfField` through the boundary rather than embedded in the struct.
- **fTetWild runs with a timeout.** Pathological SDFs that push meshing time beyond the design-mode budget fall back to Delaunay on a coarser mesh; the interactive loop never stalls on fTetWild.
- **fTetWild does not address differentiability through topology changes.** That is [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md)'s open problem; the FD-wrapper response applies at the topology-crossing boundary.
