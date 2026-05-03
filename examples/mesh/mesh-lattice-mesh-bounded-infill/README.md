# mesh-lattice-mesh-bounded-infill

**FDM-style shell + lattice composite via `generate_infill` on a
hand-authored 50 mm watertight cube.** The mesh-bounded counterpart
to §5.8 `mesh-lattice-shape-bounded`: §5.8 trims a lattice with an
analytical SDF (closed-form sphere); §5.9 trims a lattice with a
watertight input mesh (the canonical "shell + lattice" composite for
3D printing). Both paths matter; both ship in v1.0.

## What this example demonstrates

`mesh-lattice` exposes the FDM-infill workflow via `generate_infill`
(`mesh-lattice/src/infill.rs:299`): given a watertight `IndexedMesh`
target and an `InfillParams` configuration, return an `InfillResult`
with three inspectable sub-meshes (`shell`, `lattice`, combined
`mesh`) plus three volume reports (`shell_volume`, `lattice_volume`,
`interior_volume`). The pedagogically-canonical 3D-printing
composite: outer shell from inward offset, lattice in the
mesh-bounded interior, optional bridging connections from lattice
nodes to the inner shell, optional solid caps near the bbox top and
bottom.

The fixture is a 50 mm × 50 mm × 50 mm cube (`min = (0, 0, 0)`,
`max = (50, 50, 50)`; the simplest watertight mesh that exercises
every `InfillParams` field). Hand-authored with 8 vertices and 12
outward-CCW triangles in `src/fixture.rs`, so per-vertex coordinate
anchors and per-face winding-cosine anchors are bit-exact (per
`feedback_math_pass_first_handauthored`). Topology + winding mirror
`mesh-types::unit_cube` (`mesh/mesh-types/src/mesh.rs:20-54`) scaled
by 50.

The `InfillParams::for_fdm` preset (per
`mesh-lattice/src/infill.rs:102`) gives `LatticeParams::cubic(5.0)`
with density `0.2`, `shell_thickness = 1.2`, `shell_layers = 3`,
`infill_percentage = 0.2`, `connect_to_shell = true`,
`connection_thickness = 0.4`, `solid_caps = true`,
`solid_cap_layers = 4`. The fixture overrides `cell_size` to `10.0`
via `.with_cell_size(10.0)` for a coarser, more visually readable
lattice within the 50 mm bbox (the default 5.0 mm cell yields ~514
cells in the post-shell interior; 10 mm cell yields ~54 cells —
clearer in cross-section).

## Numerical contract

`main.rs` locks the post-fix numerical contract on
`for_fdm + cell_size 10` over the 50 mm cube fixture:

| Anchor | Value | What it locks |
|--------|-------|---------------|
| `mesh.vertex_count()` | `226 662` | combined mesh = shell + lattice + caps + connections |
| `mesh.triangle_count()` | `78 308` | combined |
| `shell.vertex_count()` | `224 672` | real inward-offset hollow shell (gap a post-fix) |
| `shell.face_count()` | `74 900` | |
| `lattice.vertex_count()` | `1 456` | 104 cubic struts × 14 verts/cylinder |
| `lattice.face_count()` | `2 496` | 104 × 24 |
| cap geometry | `16` verts / `24` tris | 2 boxes × (8 verts / 12 tris) (gap c post-fix) |
| connection count | `37` | lattice-to-shell bridging struts (gap b post-fix) |
| `shell_volume` | `17 195.179` mm³ | signed-volume integral on hollow shell (gap d post-fix) |
| `lattice_volume` | `28.821` mm³ | signed-volume integral on lattice |
| `interior_volume` | `53 157.376` mm³ | mesh-SDF intersection on offset shell (gap e post-fix) |
| `total_volume()` | `17 223.999_992` mm³ | = shell + lattice |
| `actual_density` | `0.010 402` | cubic-strut volume heuristic |
| top-wide region (z > 33.8) | `0` lattice verts | gap c carving differentiator |

Volume anchors hold to `1e-9` (the integral pipeline is BLAS/LAPACK-
free and platform-deterministic). Count anchors hold bit-exactly.

## Comparison anchor — `solid_caps = true` vs `false`

`main.rs` runs a second `generate_infill` call with
`.with_solid_caps(false)` to differentiate the gap-c carving:

| Anchor | with caps | without caps | Direction |
|--------|----------:|-------------:|-----------|
| `lattice.vertex_count()` | 1 456 | 2 016 | with < without (caps carve cap-band domain) |
| `mesh.triangle_count()` | 78 308 | 79 292 | with < without (un-carved cells_z = 4 strut layer adds ~960 lattice tris, dwarfing the 24 cap-box tris removed) |
| top-wide lattice verts (z > 33.8) | 0 | 448 | gap c witness — without caps, the strut layer at z ≈ 36.2 repopulates |

(The directional triangle-count finding empirically inverts the spec
§5.9 line 800 prediction; the cap-thickness carving removes a strut
layer from the lattice that more than offsets the planar-slab
geometry the caps add.)

## F6 sub-arc historical note

The v0.7 `generate_infill` had five gaps (F6 a–e). Each was fixed in
its own commit during this PR:

| Commit | Gap | Fix |
|--------|-----|-----|
| `§6.2 #24` | **a** | Real shell offset via `mesh_offset::offset_mesh(mesh, -shell_thickness, ...)`; replaces `let shell = mesh.clone();` at `infill.rs:353`. |
| `§6.2 #25` | **d** | Signed-volume integrals via promoted `estimate_mesh_volume`; replaces bbox-heuristic at `infill.rs:363-371`. |
| `§6.2 #26` | **e** | SDF-bounded interior via mesh-SDF intersection on the offset shell; replaces AABB bounds at `infill.rs:337-338`. |
| `§6.2 #27` | **c** | Solid caps at top/bottom — extruded cap boxes carve the lattice domain by `cap_thickness = solid_cap_layers × cell_size / resolution`. |
| `§6.2 #28` | **b** | Lattice-to-shell connections — for each lattice node within `2 × cell_size` of the inner shell (and outside the carved cap bands), emit a `connection_thickness`-radius cylinder bridge to the closest point on the shell. |

The anchors locked in this README and `main.rs` are the post-fix
contract — `cargo run --release` exits 0 against them.

## Public-surface coverage (per spec §5.9)

- `mesh_lattice::InfillParams` — the 8-field struct (`lattice`,
  `shell_thickness`, `shell_layers`, `infill_percentage`,
  `connect_to_shell`, `connection_thickness`, `solid_caps`,
  `solid_cap_layers`).
- `InfillParams::for_fdm` / `for_lightweight` / `for_strong` —
  preset constructors covering the canonical 3D-printing profiles
  (all three preset `validate()` paths anchored in `main.rs`).
- `InfillParams::with_lattice_type` / `with_cell_size` /
  `with_shell_thickness` / `with_infill_percentage` /
  `with_solid_caps` / `with_solid_cap_layers` — six builders;
  `with_shell_thickness` clamps to `max(0.0)` per `infill.rs:169-171`,
  `with_infill_percentage` clamps to `[0.0, 1.0]` per
  `infill.rs:178-181`. `with_solid_caps` is exercised by the
  comparison run.
- `InfillParams::validate` — success path (all three presets) +
  `InvalidShellThickness` error path via direct field assignment per
  HE-3 (the clamping `with_shell_thickness` builder cannot reach
  the negative-`shell_thickness` branch through `validate`).
- `generate_infill(mesh, params) -> Result<InfillResult, LatticeError>`
  per `mesh-lattice/src/infill.rs:299`.
- `InfillResult` fields: `mesh`, `shell`, `lattice`,
  `actual_density`, `shell_volume`, `lattice_volume`,
  `interior_volume`.
- `InfillResult::total_volume` (`shell_volume + lattice_volume`),
  `vertex_count`, `triangle_count`.
- `LatticeError::EmptyMesh` (anchored by feeding `IndexedMesh::new()`)
  and `LatticeError::InteriorTooSmall` (anchored by feeding a 5 mm
  cube under `for_fdm + cell_size 4`, `inset = 4 × 0.5 + 1.2 = 3.2`,
  `5 − 2 × 3.2 = −1.4 < 0`).

## Mesh-bounded vs analytical-SDF contrast (with §5.8)

| | §5.8 `mesh-lattice-shape-bounded` | §5.9 `mesh-lattice-mesh-bounded-infill` |
|---|---|---|
| Trim source | Analytical SDF (closed-form sphere) | Watertight input mesh |
| API | `LatticeParams::with_shape_sdf` + `generate_lattice` | `InfillParams::for_fdm` + `generate_infill` |
| Composite | Single trimmed lattice mesh | Three sub-meshes: shell, lattice, combined |
| Bounds | Caller-supplied bbox + analytical SDF | Derived from input mesh AABB + offset |
| Connections | None (unbroken trimmed lattice) | Bridging struts via `connect_to_shell` |
| Caps | None | Solid caps via `solid_caps` |
| Volume reports | Caller derives from output | `result.{shell, lattice, interior}_volume` reported |
| Visual centerpiece | Sphere-shaped gyroid (boundary-conforming) | FDM-style cutaway: shell + interior lattice + caps + connections |
| Pedagogical use | Design intent where the boundary is mathematical | 3D-printing infill on a custom part geometry |

## Run

```text
cargo run -p example-mesh-mesh-lattice-mesh-bounded-infill --release
```

Outputs (per spec §5.9 lines 802-806):

- `out/input.ply` — the 50 mm cube fixture (8 verts, 12 tris).
- `out/shell.ply` — the inward-offset hollow shell (224 672 verts /
  74 900 tris).
- `out/lattice.ply` — the interior lattice (1 456 verts / 2 496 tris).
- `out/composite.ply` — the combined mesh (226 662 verts / 78 308
  tris = shell + lattice + caps + connections).

## Visuals

⏸ Visual review **recommended** — `composite.ply` opened in f3d
shows the FDM-style cutaway aesthetic (outer shell + interior cubic
lattice + solid caps at top/bottom + bridging connection struts).

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.9 (this example) +
  `§6.5` (F6 gap-fix sub-arc cadence).
- **Sister example**: §5.8 `mesh-lattice-shape-bounded` (the
  analytical-SDF-trimmed counterpart; both ship in v1.0).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth pass at the v1.0 closeout commit).
- **Cadence memos**:
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — example impl recon surfaces in-arc platform fixes (this example
  IS that pattern: 5 gap-fixes between skeleton and post-fix
  anchors);
  [`feedback_baby_steps`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_baby_steps.md)
  — each gap-fix is its own commit + `cargo xtask grade` gate;
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 from `main.rs` IS the math-pass anchor on the
  shipped contract.
