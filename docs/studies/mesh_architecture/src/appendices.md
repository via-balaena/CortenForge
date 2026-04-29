# References, glossary, conventions

What this part covers (when authored to depth):

## References

To be populated. Candidate categories mirroring the soft-body book's appendices structure:

- **Mesh data structures.** Half-edge, winged-edge, indexed face list — design comparisons; literature on each.
- **Format specs.** Stanford PLY spec, STL spec history, 3MF Consortium spec, ISO 10303 (STEP), Wavefront OBJ legacy reference.
- **Marching cubes literature.** Lorensen-Cline original, Marching Tetrahedra variant, ambiguity resolution papers.
- **SDF computation papers.** Closest-point on triangle (Eberly textbooks), winding-number for inside/outside, distance fields for offset surfaces.
- **Repair literature.** Volino-Magnenat-Thalmann mesh repair, Attene's et al. survey, hole-filling and self-intersection algorithms.
- **TPMS / lattice literature.** Schwarz / Schoen original derivations, density-to-modulus homogenization, manufacturing tradeoffs.
- **Printability literature.** Overhang-angle conventions per technology, support-generation algorithms, orientation optimization.

## Glossary

To be populated. Candidate terms:

- **AABB** — Axis-aligned bounding box; min and max corner of the smallest box that contains a mesh, with edges parallel to coordinate axes.
- **Adjacency** — The data structure mapping each face to its neighbors via shared edges or vertices.
- **Boolean operation** — Union, intersection, or difference of two solids; commonly performed on SDF representations and re-meshed via marching cubes.
- **Boundary edge** — An edge belonging to exactly one face, indicating an open mesh boundary.
- **Closed mesh** — A mesh with no boundary edges (every edge belongs to exactly two faces).
- **Connected component** — A maximal subset of faces connected through shared edges.
- **Demoldability** — Whether a cast can be removed from a mold without destruction; a casting-domain term that the mesh ecosystem feeds with overhang/undercut analysis.
- **Draft angle** — The slope of mold-side surfaces relative to the demolding direction; positive draft = clean release, zero/negative draft = sticking.
- **F-rep / Function representation** — Implicit-surface modeling via scalar functions; SDFs are the most common F-rep variant.
- **Indexed mesh** — A mesh stored as separate vertex array + face index array, vs face-soup which duplicates positions per face.
- **Manifold** — A mesh where every edge belongs to at most two faces; the topological precondition for most surface operations.
- **Marching cubes** — The Lorensen-Cline algorithm for extracting an isosurface from a 3D scalar field, producing a triangle mesh.
- **OBB** — Oriented bounding box; the minimum-volume bounding box, free to rotate; computed via PCA on vertex positions.
- **SDF** — Signed distance field/function; a scalar function returning the signed distance to the nearest surface, with negative inside / positive outside (or the inverse, by convention).
- **SoA / AoS** — Struct-of-arrays / array-of-structs; data-layout choice, with SoA preferred for cache-friendly attribute-by-attribute access.
- **TPMS** — Triply periodic minimal surface; a class of smooth, infinitely-extending surfaces (gyroid, schwarz-P, diamond) used as lattice infill.
- **Watertight** — A mesh that's closed (no boundary edges) and manifold; the precondition for SDF queries to be unambiguous.
- **Welding** — Merging duplicate-position vertices within an epsilon distance, producing a valid indexed mesh from a face-soup-like input.
- **Winding** — The order of vertices in a face; CCW (counter-clockwise from outside) is standard for outward-pointing normals.

## Conventions

| Aspect | Convention | Source |
|---|---|---|
| Coordinate system | Right-handed (X width, Y depth, Z height) | `mesh-types/lib.rs` |
| Face winding | CCW when viewed from outside | `mesh-types/lib.rs` |
| Normal direction | Outward by right-hand rule from CCW winding | `mesh-types/lib.rs` |
| Vertex precision | `f64` for positions, `f32` for attributes | `IndexedMesh`, `AttributedMesh` |
| Vertex index width | `u32` per triangle vertex; mesh size capped at 4G vertices | `IndexedMesh::faces` |
| Default units | Unit-agnostic at type level; downstream operation crates assume mm | `mesh-types/lib.rs` |
| Tier | L0 for pure-compute crates; L0-io for `mesh-io` and the umbrella `mesh` | per-crate `Cargo.toml` |
| Library code policy | `unwrap`/`expect` denied (workspace-wide); test code may use them | per-crate `lib.rs` lints |

The depth pass produces a single conventions appendix that downstream domain books can cite without re-deriving.

## Build and run

To be populated. Standard mdBook conventions for this study; serve via `mdbook serve` from the study root, build via `mdbook build`. The study sits at `docs/studies/mesh_architecture/` alongside the soft-body and casting books at the same level.

## Cross-references to sibling books

- **[Soft-body architecture](../../soft_body_architecture/src/SUMMARY.md)** — the mesh ecosystem is foundational; coupling discussed in [Part 9](90-coupling.md) of this book and [Part 11 Ch 02](../../soft_body_architecture/src/110-crate/02-coupling.md) of the soft-body book.
- **[Casting architecture](../../casting_architecture/src/SUMMARY.md)** — the casting domain is the mesh ecosystem's biggest downstream consumer; coupling discussed in [Part 9](90-coupling.md) of this book and [Part 8](../../casting_architecture/src/80-crate.md) of the casting book.
