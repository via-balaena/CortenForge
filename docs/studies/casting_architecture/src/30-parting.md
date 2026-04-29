# Parting strategy and demoldability

What this part covers (when authored to depth):

## Parting surfaces

The surface along which the mold splits into pieces. Two regimes:

- **Planar parting.** A single flat plane through the part, splitting the mold into two halves. Solvable by inspection for symmetric parts (the equator of a sphere, the midplane of a cylinder). Fast to fab, easy to clamp.
- **Free-form parting.** Curved 2-manifold splitting the mold along a non-planar surface. Required for parts whose silhouette doesn't admit a planar split without trapping the cast. Hard to author by hand; algorithmic candidates include silhouette extraction (camera placed at infinity along an extraction direction, find the silhouette curve in 3D), and convex-hull-based heuristics. Active research area.

## Two-piece vs N-piece molds

For a planar parting, two pieces always work (if the geometry admits planar parting at all). For complex geometries:

- **Three-piece molds.** Two halves plus a core/insert that gets pulled separately. Common for parts with a single deep cavity.
- **Four+-piece molds.** Each piece releases along its own demolding direction. Cost and complexity escalate; rare for bench-scale soft-robotics work.

## Demoldability analysis

The core algorithmic question: given a part geometry and a candidate parting surface, can the cast actually be removed without destroying it?

Inputs:

- Part SDF (or surface mesh).
- Parting surface (a 2-manifold partitioning the mold into pieces).
- Per-piece demolding direction vector.
- Cast material's stretch tolerance (Ecoflex 00-30 will stretch to 600% before tearing; Dragon Skin 30A only ~360%; Smooth-Sil 940 even less).

Output: pass/fail with reasons. A demoldability failure means there's a region of the part whose surface normal opposes the demolding direction beyond a stretch budget the cast material can absorb.

The math: for each surface point `p` on the part, the demoldability score along direction `d̂` is `n̂(p) · d̂` — positive scores mean the surface releases freely; negative scores mean the surface pulls against the mold during demold. An *undercut* is a surface region where `n̂(p) · d̂ ≤ 0` over a connected patch large enough that the local stretch exceeds the material limit.

## Undercut detection

Undercuts are the dominant geometric failure mode for casting. Three responses:

- **Fix the part.** If the undercut is a design accident, remove or smooth it. Trivial when the design loop is live.
- **Fix the parting.** Choose a different parting surface or split the mold into more pieces, eliminating the undercut by topology.
- **Eat the undercut.** For small undercuts within the cast material's stretch budget, accept the bit of demold force. Fine for soft elastomers and small overhang; not fine for rigid casts or large overhangs.

The design loop wants a *gradient* through this analysis — small geometric perturbations of the part should produce small changes in the undercut metric, so the optimizer can route designs away from undercut-heavy regions of the design space. Differentiable demoldability is an open problem; finite-difference wrappers around a discrete pass/fail check are the pragmatic baseline.

## Programmatic surface

`sim-cast::PartingStrategy::analyze(part_sdf, parting_surface, demolding_direction, material)` returning a `DemoldabilityReport` with per-region scores, undercut patches, and an aggregate manufacturability metric suitable as a reward term in the design loop.

## Open questions

- Algorithmic free-form parting surface generation. Existing commercial tools (Solidworks Mold Tools, NX Mold Wizard) treat this as semi-manual; academic work (e.g. silhouette-based methods, convex decomposition) is uneven. Genuine research-frontier territory for highly non-trivial parts.
- Differentiability through demoldability checks. Closely related to [soft-body Part 6 Ch 05](../../soft_body_architecture/src/60-differentiability/05-diff-meshing.md)'s differentiable-meshing problem.
