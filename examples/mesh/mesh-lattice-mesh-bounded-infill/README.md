# mesh-lattice-mesh-bounded-infill

**FDM-style shell + lattice composite via `generate_infill` on a
hand-authored 50 mm watertight cube. At this commit (the **pre-fix
anchor capture** commit, `§6.2 #23` follow-on), the v0.7 baseline
behavior of `generate_infill` is locked numerically — the F6 sub-arc
gap-a/b/c/d/e witnesses are captured by the audit-trail binary
`bin/pre_fix_check.rs`, which compiles cleanly through every gap-fix
commit (`§6.2 #24-#28`) but is never auto-invoked by CI. Each gap-fix
that lands flips one or more witnesses red — proof that the gap
existed numerically before the fix.** The mesh-bounded counterpart to
§5.8 `mesh-lattice-shape-bounded`: §5.8 trims a lattice with an
analytical SDF (closed-form sphere); §5.9 trims a lattice with a
watertight input mesh (the canonical "shell + lattice" composite for
3D printing). Both paths matter; both ship in v1.0.

> Pre-fix anchor capture commit (`§6.2 #23` follow-on per the v1.0
> examples-coverage arc spec at `mesh/MESH_V1_EXAMPLES_SCOPE.md`).
> The F6 gap-fix sub-arc commits (a/d/e/c/b in dependency order per
> `§6.5`) and the post-fix anchors (real shell `vertex_count`,
> signed-volume integrals on shell + lattice, solid caps, lattice-
> to-shell connections, comparison runs) all land in subsequent
> commits within this PR. The audit-trail binary
> `src/bin/pre_fix_check.rs` and its `[[bin]]` entry in
> `Cargo.toml` are deleted at `§6.2 #29` once the post-fix anchors
> land — `main.rs` is rewritten at the same commit to assert
> post-fix anchors directly.

## What this example demonstrates

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

The fixture is a 50 mm × 50 mm × 50 mm cube (`min = (0, 0, 0)`,
`max = (50, 50, 50)`; the simplest watertight mesh that exercises
every `InfillParams` field). Hand-authored with 8 vertices and 12
outward-CCW triangles in `src/fixture.rs`, so per-vertex coordinate
anchors and per-face winding-cosine anchors are bit-exact (per
`feedback_math_pass_first_handauthored`). Topology + winding mirror
`mesh-types::unit_cube` (`mesh/mesh-types/src/mesh.rs:20-54`) scaled
by 50.

The `InfillParams::for_fdm` preset (per
`mesh-lattice/src/infill.rs:94-105`) gives `LatticeParams::cubic(5.0)`
with density `0.2`, `shell_thickness = 1.2`, `shell_layers = 3`,
`infill_percentage = 0.2`, `connect_to_shell = true`,
`connection_thickness = 0.4`, `solid_caps = true`,
`solid_cap_layers = 4`. The fixture overrides `cell_size` to `10.0`
via `.with_cell_size(10.0)` for a coarser, more visually readable
lattice within the 50 mm bbox (the default 5.0 mm cell yields ~514
cells in the post-shell interior; 10 mm cell yields ~54 cells —
clearer in cross-section).

## v0.7 baseline locked at this commit

`main.rs` runs only **stable** anchors — the math-pass anchors on
the fixture itself, the `InfillParams` preset / negative-validate /
error-path anchors, and smoke anchors on `generate_infill` that the
F6 sub-arc does not invalidate (`actual_density` finite + in
`[0, 1]`, `interior_volume > 0`, `vertex_count > 0`,
`triangle_count > 0`, `total_volume` finite). All stay green at
v0.7 AND through every gap-fix commit.

The **v0.7 baseline witnesses** (numerical values that flip as each
gap-fix lands) live in the audit-trail binary `bin/pre_fix_check.rs`,
runnable via:

```text
cargo run -p example-mesh-mesh-lattice-mesh-bounded-infill --release --bin pre_fix_check
```

The captured stdout at this commit is the audit-trail evidence that
gaps a-e existed numerically before the fixes landed:

| Gap | Witness | Pre-fix value (this commit) | Post-fix flip (commit) |
|-----|---------|-----------------------------|------------------------|
| **a** | `shell.vertex_count` | `8` (= `fixture.vertex_count`; shell IS `mesh.clone()`) | `≠ 8` (offset-shell vertex count differs) at `§6.2 #24` |
| **a** | `shell.signed_volume` | `125 000.0` mm³ (= `50³`; shell IS the input cube) | offset-shell signed volume at `§6.2 #24` |
| **b** | `mesh.vertex_count == shell + lattice` | `2024 == 8 + 2016` (no bridging-strut verts) | `mesh.vertex_count > shell + lattice` at `§6.2 #28` |
| **c** | lattice verts in top region (`z > 33.8`) | `448` (cap region lattice-populated) | `0` at `§6.2 #27` |
| **c** | lattice verts in bottom region (`z < 16.2`) | `592` (cap region lattice-populated) | `0` at `§6.2 #27` |
| **d** | `shell_volume` | `71 842.624` mm³ (bbox-heuristic = `125 000 − 53 157.376`) | signed-volume integral on offset shell at `§6.2 #25` |
| **d** | `lattice_volume` | `609.461907` mm³ (bbox-heuristic = `interior × actual_density`) | signed-volume integral on lattice at `§6.2 #25` |
| **e** | `interior_volume` | `53 157.376` mm³ (= `37.6³`; AABB inset per `infill.rs:337-338`) | mesh-SDF intersection on offset shell at `§6.2 #26` |

(The empirical `actual_density ≈ 0.011_465` reflects the cubic-strut
heuristic at low density on the 50 mm interior; not a load-bearing
witness on its own — it stays in `[0, 1]` and finite through the
sub-arc.)

For the cube fixture specifically, gap-e's fix at `§6.2 #26`
produces no numerical change in `interior_volume` — AABB inset and
mesh-SDF intersection on the offset shell both yield `37.6³` on a
cube. The audit-trail labels gap-e by source-line provenance
(`infill.rs:337-338` pre-fix → mesh-SDF intersection post-fix), not
by a numerical flip; a non-cube fixture would be needed to flip the
scalar value (and to expose the load-bearing geometric consequence:
on a non-cube, the AABB-bounded lattice extends through input mesh
faces). Gap-d is independent of gap-e's degeneracy on the cube and
remains numerically detectable here: `shell_volume` and
`lattice_volume` are bbox-heuristic functions of `interior_volume`
per `infill.rs:363, :371`, and become signed-volume integrals on the
shell + lattice meshes at `§6.2 #25`.

## F6 gap-fix sub-arc (per `§6.5`)

The 5 gap-fix commits land between this pre-fix anchor commit and
the post-fix anchors, in dependency order:

| Commit | Gap | Description |
|--------|-----|-------------|
| `§6.2 #24` | **a** | Real shell offset via `mesh-offset::offset_mesh(mesh, -shell_thickness)`; replaces `let shell = mesh.clone();` at `infill.rs:353`. |
| `§6.2 #25` | **d** | Signed-volume integrals on shell + lattice via promoted `estimate_mesh_volume`; replaces bbox-heuristic at `infill.rs:363-371`. Depends on gap a (a real shell exists to integrate over). |
| `§6.2 #26` | **e** | SDF-bounded interior via internal `with_shape_sdf` on the offset shell; replaces AABB bounds at `infill.rs:337-338`. Depends on gap a (offset shell defines the boundary). |
| `§6.2 #27` | **c** | Solid caps at top/bottom — planar-slab geometry covering the top + bottom `solid_cap_layers` of the post-shell interior (concrete layer-thickness convention TBD at #27, likely derived from `cell_size` or a new `layer_height` param). Depends on gap e (interior bounded so caps don't extend through mesh faces). |
| `§6.2 #28` | **b** | Lattice-to-shell connections — bridging struts from each lattice node within `connection_thickness` of the inner shell. Depends on gap a (shell exists) + gap e (interior bounded). |

Each fix is its own commit per `feedback_baby_steps`. After each,
`cargo xtask grade mesh-lattice --skip-coverage` confirms A-grade
holds before proceeding.

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
  preset constructors covering the canonical 3D-printing profiles
  (all three preset `validate()` paths are anchored in `main.rs`).
- `InfillParams::with_lattice_type` / `with_cell_size` /
  `with_shell_thickness` / `with_infill_percentage` /
  `with_solid_caps` / `with_solid_cap_layers` — six builders;
  `with_shell_thickness` clamps to `max(0.0)` per `infill.rs:161-164`,
  `with_infill_percentage` clamps to `[0.0, 1.0]` per
  `infill.rs:170-173`. The first three are declared `pub fn`, the
  last three are declared `pub const fn`.
- `InfillParams::validate` — success path (all three presets) and
  the `InvalidShellThickness` error path via direct field assignment
  (negative-`shell_thickness` is reachable through `validate` only
  by bypassing the clamping builder per spec HE-3).
- `generate_infill(mesh, params) -> Result<InfillResult, LatticeError>`
  per `mesh-lattice/src/infill.rs:287`.
- `InfillResult` fields: `mesh`, `shell`, `lattice`,
  `actual_density`, `shell_volume`, `lattice_volume`,
  `interior_volume`.
- `InfillResult::total_volume` (`shell_volume + lattice_volume`),
  `vertex_count`, `triangle_count`.
- `LatticeError::EmptyMesh` (empty input — anchored by feeding
  `IndexedMesh::new()`) and `LatticeError::InteriorTooSmall` (input
  smaller than `2 × shell_thickness + 1 × cell_size` — anchored by
  feeding a 5 mm cube under `for_fdm + cell_size 4`,
  `inset = 4 × 0.5 + 1.2 = 3.2`, `5 - 2 × 3.2 = -1.4 < 0`).

## Numerical anchors

**Locked at this commit (pre-fix anchor capture)** — math-pass
anchors on the fixture + `InfillParams` preset / validate / error-
path / smoke-call anchors in `main.rs`; gap-a/b/c/d/e v0.7 baseline
witnesses in `bin/pre_fix_check.rs` (deleted at `§6.2 #29`).

**Locked at `§6.2 #29` (post-fix anchors)**, after F6 sub-arc closes:

- 8 hand-authored vertices within `1e-12` of `[0, 50]³` (already
  locked here via `fixture::verify_fixture`).
- 12 face-winding cosine-similarity anchors > `0.9999` against the
  outward-normal direction (already locked here).
- `result.shell.vertex_count() != input.vertex_count()` (gap a).
- `result.shell_volume` within 1% of
  `signed_volume_integral(&result.shell)` (gap d).
- `result.lattice_volume` within 1% of
  `signed_volume_integral(&result.lattice)` (gap d).
- `result.interior_volume > 0.0` (already locked here).
- `result.actual_density` within ±10% of `0.2`.
- Solid-caps coverage anchor (gap c — count of lattice verts in
  cap regions is zero).
- Connection-strut anchor (gap b — `mesh.vertex_count > shell +
  lattice`).
- With-vs-without solid-caps comparison (gap c — two runs, identical
  fixture; the `solid_caps == true` mesh has more triangles and
  fewer top/bottom lattice verts).

## Run

```text
cargo run -p example-mesh-mesh-lattice-mesh-bounded-infill --release
cargo run -p example-mesh-mesh-lattice-mesh-bounded-infill --release --bin pre_fix_check
```

Output (this commit):

- `out/input.ply` — the 50 mm input cube fixture (8 verts, 12 tris).

Output (post `§6.2 #29` once the post-fix demo lands):

- `out/shell.ply` — the inward-offset hollow shell.
- `out/lattice.ply` — the interior lattice (excludes shell).
- `out/composite.ply` — the combined mesh (shell + lattice + caps
  + connections).

## Visuals

⏸ Visual review **recommended** at `§6.2 #29` — opening
`composite.ply` in f3d will show the FDM-style cutaway aesthetic
(outer shell + interior lattice + solid caps at top/bottom +
bridging connection struts). At this pre-fix-anchor commit, only
`out/input.ply` exists (the 50 mm cube fixture); visual inspection
is optional since `fixture::verify_fixture`'s math-pass anchors
gate the cube's geometric correctness.

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
  IS that pattern: 5 gap-fixes in dependency order between pre-fix
  anchors and post-fix anchors);
  [`feedback_baby_steps`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_baby_steps.md)
  — each gap-fix is its own commit + `cargo xtask grade` gate;
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 from `verify_fixture` IS the math-pass anchor on
  the fixture itself.
