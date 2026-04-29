# Coupling boundaries — with sim-soft, sim-cast, cf-design, sim-bevy

What this part covers (when authored to depth):

## With `sim-soft`

`sim-soft` consumes the mesh ecosystem at three boundaries:

- **Surface output** (planned): tet-mesh boundary triangle extraction (`sim_soft::mesh::surface::extract_boundary_triangles`) returns an `IndexedMesh` ready for `mesh-io::save_ply_attributed`. Soft-body PLY snapshots flow through this path: tet mesh → boundary triangles → `AttributedMesh` with per-vertex scalars (displacement_magnitude, von_mises_stress, etc.) → PLY frame on disk.
- **SDF round-trip** (potential): if a soft-body example wants to query `mesh-sdf::SignedDistanceField` against the deformed surface (e.g., for collision visualization), the same surface-extraction path feeds in. Less load-bearing than the snapshot path but architecturally clean.
- **Material data co-location** (existing): both `sim-soft` and `mesh-types::AttributedMesh` consume the same material database (Ecoflex/Dragon Skin/etc.) — `sim-soft` uses Lamé parameters and viscoelastic spectra; mesh-types uses the same materials' visual properties for rendering. Single source of truth, two consumer surfaces.

`sim-soft` does NOT depend on `mesh-types` or `mesh-io` directly today — the dependency edge is added when the surface-extraction module lands. The depth pass discusses why this delayed coupling was the right call (kept sim-soft's foundation phases dependency-light) and the migration plan when the edge is added.

## With `sim-cast`

`sim-cast` (planned crate; see [casting book Part 8](../../casting_architecture/src/80-crate.md)) is the mesh ecosystem's biggest downstream consumer. Coupling at:

- **Mold geometry** — `mesh-shell::ShellBuilder` provides the design template for `sim-cast::MoldBuilder`. Same builder pattern, mold-specific concerns (parting surfaces, pour ports) layered on top.
- **Mold cavity inversion** — `mesh-offset::offset_mesh` is the foundational operation for inward-offsetting a part to compute its mold cavity.
- **Manufacturability validation** — `mesh-printability`'s `Config → validate → IssueList` shape transfers to `sim-cast::ManufacturabilityCheck` directly.
- **SDF surface bridging** — `mesh-sdf::SignedDistanceField` for cast-side queries (does this point lie inside the cavity?); `sim-soft::sdf_bridge::Sdf` for design-side authoring (analytic SDF as the design primitive).
- **Lattice infill** — `mesh-lattice` for sacrificial lattice cores or lattice-infilled cast parts. Future-relevant.

The depth pass covers the API-shape transfer in detail and names which `mesh-*` operations `sim-cast` reuses verbatim versus extends.

## With `cf-design`

`cf-design` is the design-authoring layer (see [`project_cf_design_vision.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_cf_design_vision.md) — implicit-surfaces-and-mechanism-as-design-primitive). Coupling at:

- **Design output → mesh** — `cf-design` authors SDFs (analytic, via `sim-soft::sdf_bridge::Sdf` or its eventual `cf-design`-side equivalent); these are tetrahedralized for simulation and triangulated for fab. The triangulated version is `IndexedMesh`, the bridge to `mesh-*` operations.
- **Design parameters → material assignment** — `cf-design`'s parametric material partition feeds both `sim-soft::MaterialField` (for simulation) and `mesh-types::AttributedMesh::zone_ids` (for visualization and fab routing).
- **Live re-meshing under design edits** — when a `cf-design` parameter changes, the mesh updates. The mesh ecosystem's mutation-friendly types (`IndexedMesh::clear`, `AttributedMesh` per-attribute updates) support this.

The depth pass covers the live-update protocol, the change-detection-driven warm-start path that `sim-soft`'s [Part 7 Ch 04](../../soft_body_architecture/src/70-sdf-pipeline/04-live-remesh.md) discusses, and how the mesh ecosystem stays out of the way during incremental design edits.

## With `sim-bevy`

`sim-bevy` is the rendering/visualization layer. Coupling at:

- **PLY frame-sequence playback** (planned) — soft-body and mesh examples produce PLY frame directories; `sim-bevy` will eventually grow a viewer subsystem (under `sim/L1/bevy/src/soft/`) that reads PLY directories and renders them with per-vertex attribute colormaps.
- **Bevy mesh upload** — every `IndexedMesh` produced by the mesh ecosystem can be converted to a `bevy::Mesh` for direct rendering. The conversion is straightforward (positions, indices, optional normals + UVs); the mesh ecosystem's pure-data structures are well-shaped for the upload path.
- **MeshSnapshot interface** (future, [soft-body Part 9 Ch 05](../../soft_body_architecture/src/90-visual/05-sim-bevy.md)) — the eventual production-grade integration uses `MeshSnapshot` (positions + 7 per-vertex physics buffers + indices). The `AttributedMesh::extras` extension is forward-compatible — the same per-vertex attribute shape persists to disk as PLY for the foundation-validation phase, lives in memory as a buffer-pair for the production phase.

The depth pass covers the PLY-viewer plugin design (PR 2 of the examples work; `sim/L1/bevy/src/soft/` module) and the migration path to the full `MeshSnapshot` pipeline at Phase I.

## Tier-system implications

Coupling boundaries respect CortenForge's L0/L0-io/L0-integration/L1 tier system:

| Crate | Tier | Implication for coupling |
|---|---|---|
| `mesh-types` | L0 | Pure data, foundation; everything depends on it. |
| `mesh-io` | L0-io | Adds file-system and parser dependencies; consumers opt in via dependency declarations. |
| `mesh-*` operation crates | L0 | Pure compute, no Bevy/wgpu/network deps. |
| `sim-soft` | L0 | Coupling to mesh-types adds a dependency edge but stays L0. |
| `sim-cast` (planned) | L1 | Coupling to mesh-shell/offset/printability puts it at L1 by depth (same as sim-bevy). |
| `cf-design` | L0/L1 (TBD) | Tier depends on whether it depends on Bevy directly; out of mesh-architecture scope. |
| `sim-bevy` | L1 | Bevy-dependent; reads from the mesh ecosystem's L0 outputs. |

The depth pass covers the tier rationale and the failure modes when a crate's dependencies push it across tier boundaries (e.g., if `mesh-io` ever needed to depend on `bevy`, it would lose its L0-io status — that hasn't happened and isn't planned).
