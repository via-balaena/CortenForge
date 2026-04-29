# Why this study exists

|  |  |
|---|---|
| **What** | A study in the 10-crate CortenForge mesh ecosystem — the L0 geometric foundation under soft-body simulation, casting/manufacturing, and any future mechanical-design domain. |
| **Why** | The mesh ecosystem is mature, A-grade, and feature-rich (`mesh-types`, `mesh-io`, `mesh-repair`, `mesh-sdf`, `mesh-offset`, `mesh-shell`, `mesh-measure`, `mesh-printability`, `mesh-lattice`, plus the `mesh` umbrella) — but has crate-level doc comments and no architectural narrative tying them together. As CortenForge grows two new domains (soft-body simulation + casting) that lean on the mesh ecosystem, the foundation deserves the same first-class architectural treatment the soft-body and casting books got. |
| **Goal** | Document what each crate is, why its API is shaped the way it is, where the gaps are, what extensions are planned, and how the ecosystem couples to the domain crates that consume it. Retrospective for the existing surface, forward-looking for the gaps and extensions. |
| **Canonical exercises** | (a) per-vertex attribute PLY snapshots from soft-body simulations — drives the `AttributedMesh::extras` extension and the `save_ply_attributed` writer; (b) mold-cavity generation for the layered silicone device — drives the `mesh-shell` → `sim-cast::MoldBuilder` design pattern. |
| **Core thesis** | The mesh ecosystem is the geometric lingua franca of the platform. The same `IndexedMesh` flows from a CAD import (`mesh-io::load_step`), through a repair pass (`mesh-repair::repair_mesh`), through manufacturability validation (`mesh-printability::validate_for_printing`), through shell generation (`mesh-shell::ShellBuilder`), and into a fab-ready file (`mesh-io::save_3mf`). Soft-body and casting are *consumers* of this surface; their value depends on the foundation being solid, extensible, and visually validated. |

## Scope

This book covers the mesh ecosystem at `mesh/`. It does NOT cover:

- **Soft-body tetrahedral meshes** — those live in `sim-soft::mesh` (tet-specific, not in the mesh ecosystem). Tet support could grow into `mesh-types` later (see [Part 10 roadmap](100-roadmap.md)) but is currently out of scope.
- **`cf-geometry`** — the upstream geometric primitives (`Point3`, `Vector3`, `Aabb`, `Triangle`, `IndexedMesh`) that `mesh-types` re-exports. `cf-geometry` is a deeper foundation; this book uses it but doesn't redocument it.
- **Implementation specifics inside each crate** — the crate-level rustdoc and source code remain authoritative for API details. This book focuses on architectural framing, design choices, and inter-crate relationships.
- **Mesh fabrication itself** — converting STL/STEP into actual CNC g-code or printer instructions. The mesh ecosystem stops at the file-export boundary.

## Relationship to the other studies

This is the third sibling study at `docs/studies/`:

- **[soft-body architecture](../../soft_body_architecture/src/SUMMARY.md)** — depth-authored, drives sim-soft. Consumes mesh-types + mesh-io for file I/O at integration boundaries; mesh ecosystem is foundational rather than load-bearing in the simulator's hot path.
- **[casting architecture](../../casting_architecture/src/SUMMARY.md)** — skeleton-only, depth pass deferred. Will lean heavily on `mesh-shell`, `mesh-offset`, `mesh-printability` for mold geometry, parting analysis, and manufacturability.
- **mesh architecture** (this book) — retrospective for what exists, forward-looking for the extensions that soft-body and casting are about to drive.

The three books cross-reference at coupling boundaries. The mesh book documents the mesh-side of each interface; the soft-body and casting books document their respective domain-sides.

## How this study is produced

Same cadence as its siblings: deep recon before commitment, two-pass prose review, ceiling-quality at every leaf, no timeline pressure. The skeleton lands during the mesh examples + soft-body examples branch work; depth-pass authoring happens incrementally over sessions, in tandem with the examples PRs. By the time the mesh examples PR ships, the parts that examples cover have been depth-passed; remaining parts stay skeletal but planned, with depth filled in as later use cases drive it.

## Reading order

Numbered order matches dependency depth. [Part 1 — Core Types](10-types.md) is foundational for every other part; [Part 2 — File I/O](20-io.md) sits on top of types; the operation crates ([Parts 3–5](30-sdf-and-offset.md)) compose types and I/O into useful work; [Parts 6–7](60-measurement.md) are analysis and synthesis; [Part 8 examples](80-examples.md) demonstrates the full surface visually; [Part 9 coupling](90-coupling.md) names the boundaries; [Part 10 roadmap](100-roadmap.md) is forward-looking. The appendices carry conventions and references.
