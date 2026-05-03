# mesh-lattice-mesh-bounded-infill

**FDM-style shell + lattice composite via `generate_infill` on a
hand-authored watertight mesh — pre-fix anchors capture the v0.7
`generate_infill` baseline (`shell == mesh.clone()` per
`infill.rs:353`, bbox-heuristic volumes per `infill.rs:363-371`),
then the F6 gap-fix sub-arc (a/d/e/c/b per `§6.5`) replaces those
with a real inward-offset shell, signed-volume integrals on the
shell + lattice, an SDF-bounded interior, solid caps at top/bottom,
and lattice-to-shell connections. The fixture is a 50 mm × 50 mm ×
50 mm cube (`min = (0, 0, 0)`, `max = (50, 50, 50)`; 8 verts / 12
tris, hand-authored with outward-normal winding) fed to
`generate_infill` under the `InfillParams::for_fdm` preset.** The
mesh-bounded counterpart to §5.8 `mesh-lattice-shape-bounded`: §5.8
trims a lattice with an analytical SDF (closed-form sphere); §5.9
trims a lattice with a watertight input mesh (the canonical
"shell + lattice" composite for 3D printing). Both paths matter;
both ship in v1.0.

> Skeleton commit (`§6.2 #23` per the v1.0 examples-coverage arc
> spec at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The pre-fix anchors
> (v0.7 baseline behavior captured), the F6 gap-fix sub-arc commits
> (a/d/e/c/b in dependency order per `§6.5`), and the post-fix
> anchors (real shell `vertex_count`, signed-volume integrals,
> solid caps, connections, comparison runs) all land in subsequent
> commits within this PR. This README is a placeholder; museum-plaque
> content (numerical anchors, visuals, comparison-output callout,
> cross-references) fills in alongside the impl commits.

## What this example will demonstrate

`mesh-lattice` exposes the FDM-infill workflow via `generate_infill`
(`mesh-lattice/src/infill.rs:287`): given a watertight `IndexedMesh`
target and an `InfillParams` configuration, return an `InfillResult`
with three inspectable sub-meshes (`shell`, `lattice`, combined
`mesh`) plus three volume reports (`shell_volume`, `lattice_volume`,
`interior_volume`). The pedagogically-canonical 3D-printing
composite: outer shell from inward offset, lattice in the interior,
optional connections from lattice nodes to the inner shell surface,
optional solid caps near the bbox top/bottom.

The v0.7 implementation has five known gaps (F6 a–e per the v1.0
examples-coverage arc spec §1.2):

- **Gap a**: `shell` is `mesh.clone()` (`infill.rs:353`) — no actual
  offset; the "shell" is identical to the input mesh.
- **Gap b**: `connect_to_shell` / `connection_thickness` parameters
  are publicly exposed but never read in the main path (no bridging
  struts are emitted between lattice and inner shell).
- **Gap c**: `solid_caps` / `solid_cap_layers` parameters are
  publicly exposed but never read (no planar slab geometry near the
  bbox top/bottom).
- **Gap d**: `shell_volume` and `lattice_volume` are bbox-heuristic
  (`bounds_volume - interior_volume` and
  `interior_volume × actual_density` per `infill.rs:363-371`), not
  signed-volume integrals on the shell / lattice meshes.
- **Gap e**: `interior_volume` derives from an axis-aligned bounding
  box (`interior_min` / `interior_max` at `infill.rs:337-338`), so
  for non-box input meshes the lattice can extend through input
  faces.

The pre-fix anchors (next commit) lock the v0.7 behavior (so we
have a reproducible baseline to invalidate). The 5 gap-fix commits
(`§6.2 #24-#28` per `§6.5`) replace each of a/d/e/c/b in dependency
order. The post-fix anchors (`§6.2 #29`) prove every gap closed —
shell has a different vertex count from input, volumes match
signed-volume integrals within 1%, solid caps cover top/bottom
layers (no lattice verts in those regions), and bridging struts
emit when `connect_to_shell == true`.

The fixture is a 50 mm × 50 mm × 50 mm cube (`min = (0, 0, 0)`,
`max = (50, 50, 50)`; the simplest watertight mesh that exercises
every `InfillParams` field). Hand-authored with 8 vertices and 12
outward-normal-winding triangles, so the per-vertex coordinate
anchors and per-face winding-cosine anchors are bit-exact (per
`feedback_math_pass_first_handauthored`). The `InfillParams::for_fdm`
preset (per `mesh-lattice/src/infill.rs:94-105`) gives
`LatticeParams::cubic(5.0)` with density `0.2`, `shell_thickness = 1.2`,
`shell_layers = 3`, `infill_percentage = 0.2`, `connect_to_shell = true`,
`connection_thickness = 0.4`, `solid_caps = true`,
`solid_cap_layers = 4`; the fixture commit overrides `cell_size` to
`10.0` via `.with_cell_size(10.0)` for a coarser, more visually
readable lattice within the 50 mm bbox (the default 5.0 mm cell
yields ~514 cells in the post-shell interior; 10 mm cell yields ~54
cells — clearer in cross-section).

The example will compute (TBD — land across the next ~7 commits):

1. **Hand-authored 50 mm cube fixture** — 8 vertices within `1e-12`
   of `[0, 50]³`; 12 face-winding cross-product cosine-similarity
   anchors > `0.9999` against the outward-normal direction.
2. **Pre-fix anchors** (`expected_pre_fix.rs`, deleted at #29)
   capturing the v0.7 baseline:
   - `result.shell.vertex_count() == input.vertex_count()` (gap a:
     shell IS the input clone).
   - `result.shell_volume == bounds_volume - interior_volume`
     (gap d: bbox heuristic, NOT signed-volume integral).
   - No top/bottom-region cap detection (gap c: lattice extends
     through where caps should sit).
   - No connection geometry between lattice nodes and the shell
     interior surface (gap b).
3. **`InfillParams` builder + validate paths**:
   - `InfillParams::for_fdm().validate() == Ok(())`.
   - **Negative-validate via direct field assignment**: builder
     `with_shell_thickness` clamps to `max(0.0)` per
     `infill.rs:161-164` so the negative-`shell_thickness` path through
     `validate` is reachable only by direct construction —
     `let mut p = InfillParams::for_fdm(); p.shell_thickness = -1.0;`
     then `p.validate()` returns
     `Err(LatticeError::InvalidShellThickness(-1.0))`. The README
     explains: builder is callable-side-defensive (clamp on input);
     `validate()` is the inspection-side check (both paths exist).
4. **`generate_infill` error paths**:
   - Empty input mesh ⇒ `Err(LatticeError::EmptyMesh)`.
   - Input mesh smaller than `2 × shell_thickness + 1 × cell_size`
     ⇒ `Err(LatticeError::InteriorTooSmall)`.
5. **Post-fix anchors** (`§6.2 #29`, after F6 sub-arc closes):
   - `result.shell.vertex_count() != input.vertex_count()` (gap a).
   - `result.shell_volume` within 1% of
     `signed_volume_integral(&result.shell)` (gap d).
   - `result.lattice_volume` within 1% of
     `signed_volume_integral(&result.lattice)` (gap d).
   - `result.actual_density` within ±10% of
     `params.infill_percentage = 0.2`.
   - `result.mesh.vertex_count() > result.shell.vertex_count() + result.lattice.vertex_count()`
     when `connect_to_shell == true` (gap b: the strict-`>` proves
     bridging struts add verts beyond the sub-mesh sum).
   - `solid_caps == true` and `solid_cap_layers == 4` ⇒ count of
     lattice verts with `z > 50 - 4 × layer_height` is **zero**
     (gap c: solid caps cover the top region).
   - With-vs-without solid-caps comparison (two runs, identical
     fixture): the `solid_caps == true` mesh has MORE triangles
     (planar-slab geometry adds tris) and FEWER top/bottom lattice
     verts (gap c).

## Mesh-bounded vs analytical-SDF contrast (with §5.8)

| | §5.8 `mesh-lattice-shape-bounded` | §5.9 `mesh-lattice-mesh-bounded-infill` |
|---|---|---|
| Trim source | Analytical SDF (closed-form sphere) | Watertight input mesh |
| API | `LatticeParams::with_shape_sdf` + `generate_lattice` | `InfillParams::for_fdm` + `generate_infill` |
| Composite | Single trimmed lattice mesh | Three sub-meshes: shell, lattice, combined |
| Bounds | Caller-supplied bbox + analytical SDF | Derived from input mesh AABB + offset |
| Connections | None (unbroken trimmed lattice) | Optional bridging struts (`connect_to_shell`) |
| Caps | None | Optional solid caps near top/bottom |
| Volume reports | Caller derives from output | `result.{shell, lattice, interior}_volume` reported |
| Visual centerpiece | Sphere-shaped gyroid (boundary-conforming) | FDM-style cutaway: outer shell + interior lattice + caps |
| Pedagogical use | Design intent where the boundary is mathematical | 3D-printing infill on a custom part geometry |

## Public-surface coverage (per spec §5.9)

- `mesh_lattice::InfillParams` — the 8-field struct (`lattice`,
  `shell_thickness`, `shell_layers`, `infill_percentage`,
  `connect_to_shell`, `connection_thickness`, `solid_caps`,
  `solid_cap_layers`).
- `InfillParams::for_fdm` / `for_lightweight` / `for_strong` —
  preset constructors covering the canonical 3D-printing profiles.
- `InfillParams::with_lattice_type` / `with_cell_size` /
  `with_shell_thickness` / `with_infill_percentage` /
  `with_solid_caps` / `with_solid_cap_layers` — builders (six total;
  `with_shell_thickness` clamps to `max(0.0)`,
  `with_infill_percentage` clamps to `[0.0, 1.0]`).
- `InfillParams::validate` — success and `InvalidShellThickness`
  error paths.
- `generate_infill(mesh, params) -> Result<InfillResult, LatticeError>`
  per `mesh-lattice/src/infill.rs:287`.
- `InfillResult` fields: `mesh`, `shell`, `lattice`,
  `actual_density`, `shell_volume`, `lattice_volume`,
  `interior_volume`.
- `InfillResult::total_volume` (`shell_volume + lattice_volume`),
  `vertex_count`, `triangle_count`.
- `LatticeError::EmptyMesh` (empty input) and
  `LatticeError::InteriorTooSmall` (input smaller than
  `2 × shell_thickness + 1 × cell_size`).

## F6 gap-fix sub-arc (per `§6.5`)

The 5 gap-fix commits land between this skeleton's pre-fix anchors
and the post-fix anchors, in dependency order:

| Commit | Gap | Description |
|--------|-----|-------------|
| `§6.2 #24` | **a** | Real shell offset via `mesh-offset::offset_mesh(mesh, -shell_thickness)`; replaces `let shell = mesh.clone();` at `infill.rs:353`. |
| `§6.2 #25` | **d** | Signed-volume integrals on shell + lattice via promoted `estimate_mesh_volume`; replaces bbox-heuristic at `infill.rs:363-371`. Depends on gap a (a real shell exists to integrate over). |
| `§6.2 #26` | **e** | SDF-bounded interior via internal `with_shape_sdf` on the offset shell; replaces AABB bounds at `infill.rs:337-338`. Depends on gap a (offset shell defines the boundary). |
| `§6.2 #27` | **c** | Solid caps at top/bottom — planar-slab geometry near `bounds.max.z - solid_cap_layers × layer_height` and the symmetric bottom. Depends on gap e (interior bounded so caps don't extend through mesh faces). |
| `§6.2 #28` | **b** | Lattice-to-shell connections — bridging struts from each lattice node within `connection_thickness` of the inner shell. Depends on gap a (shell exists) + gap e (interior bounded). |

Each fix is its own commit per `feedback_baby_steps`. After each,
`cargo xtask grade mesh-lattice --skip-coverage` confirms A-grade
holds before proceeding.

## Numerical anchors (TBD — land across the F6 sub-arc + post-fix commit)

Pre-fix anchors (next commit, deleted at `§6.2 #29`):

- `result.shell.vertex_count() == 8` (== input cube vertex count;
  gap a ⇒ shell is `mesh.clone()`).
- `result.shell_volume == 125000.0 - interior_volume`
  (bbox-heuristic per `infill.rs:363-371`; explicit numerical match
  with the heuristic formula, NOT a signed-volume integral).

Post-fix anchors (`§6.2 #29`, gates the PR2 closeout):

- 8 hand-authored vertices within `1e-12` of `[0, 50]³`.
- 12 face-winding cosine-similarity anchors > `0.9999`.
- `result.shell.vertex_count() != input.vertex_count()` (gap a).
- `result.shell_volume` within 1% of
  `signed_volume_integral(&result.shell)` (gap d).
- `result.lattice_volume` within 1% of
  `signed_volume_integral(&result.lattice)` (gap d).
- `result.interior_volume > 0.0` (always; bbox-derived).
- `result.actual_density` within ±10% of `0.2`.
- Solid-caps coverage anchor (gap c).
- Connection-strut anchor (gap b).
- With-vs-without solid-caps comparison (gap c).

## Run (TBD — full readout lands at `§6.2 #29`)

```text
cargo run -p example-mesh-mesh-lattice-mesh-bounded-infill --release
```

Output:

- `out/input.ply` — the 50 mm input cube (8 verts, 12 tris).
- `out/shell.ply` — the inward-offset hollow shell (post-gap-a;
  pre-gap-a this is identical to `input.ply`).
- `out/lattice.ply` — the interior lattice (excludes shell).
- `out/composite.ply` — the combined mesh (shell + lattice + caps
  + connections).

Numerical printout includes the three volume reports and every
post-fix anchor's empirical value.

## Visuals (TBD — land at `§6.2 #29`)

⏸ Visual review **recommended** — opening `composite.ply` in f3d
shows the FDM-style cutaway aesthetic (outer shell + interior
lattice + solid caps at top/bottom + bridging connection struts).
The post-fix shell + lattice composite is the visual centerpiece;
pre-fix `composite.ply` would show only the input cube glued onto
a free-floating lattice (no actual shell, no caps, no connections).

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.9 (this example) +
  `§6.5` (F6 gap-fix sub-arc cadence).
- **Sister example**: §5.8 `mesh-lattice-shape-bounded` (the
  analytical-SDF-trimmed counterpart; both ship in v1.0).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at the v1.0 closeout
  commit of the arc).
- **Cadence memos**:
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — example impl recon surfaces in-arc platform fixes (this example
  IS that pattern: 5 gap-fixes in dependency order between skeleton
  and post-fix anchors);
  [`feedback_baby_steps`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_baby_steps.md)
  — each gap-fix is its own commit + `cargo xtask grade` gate;
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 from the post-fix anchors gates the visuals pass.
