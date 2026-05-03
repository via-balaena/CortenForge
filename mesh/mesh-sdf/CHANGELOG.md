# Changelog

All notable changes to mesh-sdf will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### v0.9 candidates

These backlog candidates are gated on a real consumer driving them per
the platform's "examples drive gap-fixes" discipline. Each entry names
the consumer-arrival shape that re-opens the work. The platform-wide
list (sixteen candidates spanning four crates) lives in
[Part 10 of the mesh architecture study](../../docs/studies/mesh_architecture/src/100-roadmap.md);
the entries below are the mesh-sdf-specific subset.

- **`SdfError::OutOfBounds` + `InvalidDimensions` resolution.** Two
  unused error variants are dead code unless the grid SDF API ships.
  *Trigger*: a grid-SDF API consumer arrives, OR an explicit decision
  to remove the variants. Effort: ~50 LOC.

- **SDF sign-convention upgrade — pseudo-normal or winding-number at
  vertex / edge regions.** Surfaced by `mesh-sdf-distance-query`
  (mesh book Part 8 Band 3): on the unit octahedron's 1000-point
  bulk-grid scan, `signed_distance < 0` reports 14 inside while
  ray-cast `point_in_mesh` correctly reports 8. The 6 false-positives
  are at points where multiple faces tie on the closest octahedron
  vertex and the strict-`<` tie-break in
  `SignedDistanceField::distance` picks a face whose outward normal
  flips sign relative to the geometric inside-test. The failure
  applies to vertex / edge regions of any mesh, including convex.
  *Trigger*: a user reports SDF sign flipping incorrectly near edges
  or vertices of any geometry. Effort: ~200 LOC; algorithm
  well-documented in the literature (Bærentzen-Aanæs angle-weighted
  vertex normal, or generalized winding number).

- **BVH acceleration of `SignedDistanceField::closest_point`.**
  Currently O(F) brute-force scan over all faces. On the F6
  `generate_infill` connections pass, ~50 lattice nodes × 75 K faces
  ≈ 3.8 M ops per call (tractable in release mode, on the order of
  milliseconds). A BVH would compress this to ~3.8 K ops, three
  orders of magnitude. *Trigger*: any consumer reports SDF query as
  a measured bottleneck. Effort: medium; a BVH primitive already
  exists in `mesh-repair` for self-intersection detection.

- **Consolidate `closest_point_on_triangle` duplication between
  `mesh-sdf` and `mesh-measure`.** Workspace-hygiene refactor; pick
  one home (probably `mesh-sdf` since it is the older surface) and
  re-export to `mesh-measure`. *Trigger*: a maintainer asks "why
  does this exist twice?" Effort: ~50 LOC.

## [0.7.0]

Initial release.
