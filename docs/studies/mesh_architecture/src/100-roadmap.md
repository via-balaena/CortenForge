# Roadmap and open questions

What this part covers: where the mesh ecosystem stands today, what's deferred and what triggers it, and the open architectural questions that don't yet have driving forces but stay flagged for when one arrives.

## Phase A — gap fixes + first examples (shipped)

Phase A delivered the foundation extensions soft-body needed (extensible per-vertex attributes; PLY writer for `AttributedMesh`) and validated the mesh ecosystem visually for the first time. Three squash-merged PRs across 2026-04-29 → 2026-05-03:

- **PR #222** (`2dac42b4`, 2026-04-29) — examples + `AttributedMesh` + `mesh-shell` SDF fixes; 22-commit audit trail; mesh + casting book skeletons.
- **PR #223** (`e9cb0d6d`, 2026-05-01) — `mesh-printability` v0.8 fix arc; 13 detector-coverage gaps closed plus 8 printability examples.
- **PR #224** (`e79f2fc2`, 2026-05-02) — first half of the v1.0 examples-coverage arc; 8 examples plus the C15a `BeamLatticeData` octet-truss in-arc gap-fix.
- A fourth PR (in flight on the `feature/mesh-v1-pr2-bounded-infill` branch when these book updates were authored) lands the second half of the v1.0 arc: the F6 `generate_infill` gap-fix sub-arc (gaps a-e, plus post-fix anchor recapture), the `mesh-lattice-mesh-bounded-infill` composite example, the v1.0 examples-mesh aggregator rewrite, and these book updates.

The core deliverables — `AttributedMesh::extras: HashMap<String, Vec<f32>>` extensible per-vertex slot; `mesh-io::save_ply_attributed` PLY writer; complete example coverage of all 10 public crates (see [Part 8 — The mesh examples inventory](80-examples.md)) — are all in place.

## Phase B — broader attribute typing (future-work)

If use cases drive it: extend `extras` to support `Vec<u32>` (material IDs, integer labels) and `Vec<Vector3<f64>>` (vector fields like displacement direction). Probably an enum-typed value:

```rust
pub enum AttributeArray {
    F32(Vec<f32>),
    U32(Vec<u32>),
    Vec3(Vec<Vector3<f64>>),
}
pub extras: HashMap<String, AttributeArray>;
```

Drives PLY writer extensions (typed properties), 3MF metadata extensions (XML-typed values), and downstream consumer updates. Probably bundled with whichever soft-body or sim-cast use case first needs non-`f32` attributes.

**Status at first stable release**: no consumer drove this. The single custom-attribute consumer in the example inventory, `mesh-sdf-distance-query`'s grid PLY with `extras["signed_distance"]: Vec<f32>`, fits the existing `HashMap<String, Vec<f32>>` shape exactly. Defer until a multi-type consumer arrives.

## v0.9 candidates

The first stable release surfaced sixteen near-term candidates: eleven from cross-crate audit and example execution, five from the F6 `generate_infill` gap-fix sub-arc. Each candidate is gated on a real consumer driving it (per `feedback_examples_drive_gap_fixes`); the Trigger line in each entry below states what consumer-arrival looks like. Crate-specific candidates also appear in the corresponding `CHANGELOG.md` `[Unreleased]` block.

For `mesh-printability`-specific candidates surfaced by the v0.8 fix arc (PR #223), see `mesh/mesh-printability/CHANGELOG.md`'s `[Unreleased]` block — that backlog is tracked at the crate level rather than mirrored here, since it's specific to the printability detector inventory.

### `mesh-sdf`

1. **`SdfError::OutOfBounds` + `InvalidDimensions` resolution.** Two unused error variants are dead code unless the grid SDF API ships.
   *Trigger*: a grid-SDF API consumer arrives, OR an explicit decision to remove. ~50 LOC.

2. **SDF sign-convention upgrade — pseudo-normal or winding-number at vertex / edge regions.** Surfaced by `mesh-sdf-distance-query` (Part 8 Band 3): on the unit octahedron's 1000-point bulk-grid scan, `signed_distance < 0` reports 14 inside while ray-cast `point_in_mesh` correctly reports 8. The 6 false-positives are at points where multiple faces tie on the closest octahedron vertex and the strict-`<` tie-break in `SignedDistanceField::distance` picks a face whose outward normal flips sign relative to the geometric inside-test. Failure applies to vertex / edge regions of any mesh, including convex.
   *Trigger*: a user reports SDF sign flipping incorrectly near edges or vertices of any geometry. ~200 LOC; algorithm well-documented in the literature (Bærentzen-Aanæs angle-weighted vertex normal, or generalized winding number).

3. **BVH acceleration of `SignedDistanceField::closest_point`.** Currently O(F) brute-force scan over all faces. On the F6 `generate_infill` connections pass, ~50 lattice nodes × 75 K faces ≈ 3.8 M ops per call (tractable in release mode, on the order of milliseconds). A BVH would compress this to ~3.8 K ops, three orders of magnitude.
   *Trigger*: any consumer reports SDF query as a measured bottleneck. Medium-effort; a BVH primitive already exists in `mesh-repair` for self-intersection detection.

### `mesh-measure`

4. **`MeasureError` / `MeasureResult` adoption.** Currently `Option<T>` carries the same information for callers; tightening to `Result<T, MeasureError>` is API-shape-breaking.
   *Trigger*: a function tightens validation to fail-fast (e.g., empty mesh becomes a hard error), OR a power user reports surprise that `dimensions()` returns `Default` rather than an error on degenerate input. ~80 LOC + breaking-API rename + downstream call-site updates.

5. **Tolerance-aware `OrientedBoundingBox::contains`.** Surfaced by `mesh-measure-bounding-box` (Part 8 Band 4): PCA's iterative `SymmetricEigen` produces `half_extents` and the inverse-rotation mapping with ~1 ULP roundoff; the four input vertices that defined the OBB extremes can fail strict `<=` containment by `~1.78e-15`.
   *Trigger*: a user reports a vertex they passed in failing `obb.contains(v)`. ~30 LOC: add `contains_with_tol(point, eps)` method, keep strict `contains` as alias for `contains_with_tol(p, 0.0)`.

6. **Document "OBB ⊄ AABB in general" + remove the corners-within-AABB anchor pattern from API docs and examples.** Surfaced during `mesh-measure-bounding-box` spec authoring: the folk-intuition "OBB is tighter than AABB so OBB ⊆ AABB" is false for any non-trivial OBB rotation; OBB corners extend OUTSIDE the AABB envelope by `half_extent · sin(rotation_angle)`.
   *Trigger*: write the docstring + audit pass before another consumer encodes the wrong sanity check. ~10 LOC.

7. **Proper polygon centroid in `CrossSection` (shoelace-weighted, not naive average).** Surfaced by `mesh-measure-cross-section` (Part 8 Band 4): the current centroid is `sum(points) / points.len()`, but `chain_segments` produces a contour with one chain-closure-duplicate point, biasing a symmetric polygon's centroid by `V_dup / (N + 1) ≠ (0, 0, 0)`. On the 32-segment cylinder mid-slice the bias is ~0.077 mm in `(x, y)`.
   *Trigger*: a consumer needs ≤ 1e-10 centroid accuracy. ~30 LOC: replace the naive average with `c_x = (1 / (6A)) · Σ (x_i + x_(i+1)) · (x_i · y_(i+1) − x_(i+1) · y_i)`, projecting to 2D via the existing `(u, v)` basis lifted from `calculate_cross_section_area`.

8. **Consolidate `closest_point_on_triangle` duplication between `mesh-sdf` and `mesh-measure`.** Workspace-hygiene refactor; pick one home (probably `mesh-sdf` since it's the older surface) and re-export.
   *Trigger*: a maintainer asks "why does this exist twice?" ~50 LOC.

### `mesh-lattice`

9. **Real Voronoi tessellation.** `LatticeType::Voronoi` is currently perturbed-cubic (a stand-in, not a Voronoi diagram).
   *Trigger*: a user reports cubic-grid artifacts in Voronoi output, OR organic-feeling lattice required for a layered-silicone-device payload. ~200-400 LOC of new Bowyer-Watson or Lloyd's-algorithm code.

10. **Welded TPMS-lattice MC output.** `extract_isosurface` emits un-welded vertex-soup output (the locked signature `vertex_count == 3 × triangle_count`). `mesh-shell` already runs a weld pass via `mesh-repair::weld_vertices`; apply the same pattern.
    *Trigger*: a real consumer needs welded TPMS output for visual aesthetic OR file-size compression. ~30 LOC.

11. **Demo non-default `BeamCap` variants (`Flat`, `Butt`).** The default `BeamCap::Round` is exercised by `mesh-lattice-strut-cubic`; `Flat` and `Butt` are unexercised at the example layer.
    *Trigger*: a user reports surprise that `BeamCap::Flat` / `BeamCap::Butt` aren't visually demonstrated. Augment `mesh-lattice-strut-cubic` with a sub-demo, OR add a separate example. ~80 LOC additive.

12. **0%-infill early-return in `generate_infill` still uses the gap-a `shell = mesh.clone()` pattern.** The 100%-infill case is semantically defensible (solid = no shell distinction); the 0%-infill case is semantically wrong (a hollow part SHOULD have an inward-offset shell, not a clone of the input mesh). Strict-(b) deferral from the F6 gap-a sub-arc.
    *Trigger*: a consumer reports the 0%-infill output looks wrong. ~30 LOC: extend the offset-mesh path to cover the early-return.

13. **Non-convex inputs whose AABB inset includes outside-part regions** (e.g., torus-hole). The F6 gap-e cavity-SDF approach uses face-normal-of-closest-face with a sign convention tuned for the CCW-INWARD inner_offset. For a non-convex input where the AABB inset overlaps outside-part regions, the closest face for outside-part points is on the same side of the cavity, so the negated distance still reads negative — the lattice fills the outside-part region as if it were cavity. Convex inputs unaffected; documented in source.
    *Trigger*: a non-convex-input consumer arrives. ~50-100 LOC: extra explicit-shape-SDF intersection, or analytical-SDF clip on the input bounds.

14. **`shell_thickness == 0` edge case.** Inherited from gap-a: `offset_mesh(mesh, -0, ...)` returns a near-degenerate inner_offset, fed to the cavity-SDF gives wrong signs across the interior, the lattice ends up empty or wrong. Pre-existing, not a regression.
    *Trigger*: a consumer needs `shell_thickness == 0` semantics (probably a hollow-only or solid-only path). ~20 LOC: explicit branch + clamp.

15. **Explicit `LatticeParams::layer_height: f64` field.** The F6 gap-c cap thickness uses `cell_size / resolution` as a heuristic (FDM-typical 0.4-0.6 mm/layer across the three preset constructors). The `resolution` field is validated `>= 2` per `LatticeParams::validate`, so the division can never DIV0, but the heuristic is implicit.
    *Trigger*: a real consumer wants direct control of the layer-height heuristic. ~30 LOC additive.

### `mesh-io`

16. **3MF beam writer.** `BeamLatticeData` is already the data model (populated by `mesh-lattice-strut-cubic`'s `with_beam_export(true)` — see Part 8 Band 6); the writer needs the 3MF Beam Lattice Extension format.
    *Trigger*: 3MF beam-output demand from a printer-driver workflow. ~300-500 LOC.

## Open architectural questions

These don't yet have driving forces. They stay flagged for when one arrives.

### Tet mesh support

`mesh-types` does not currently include a `TetMesh` type; tet meshes live in `sim-soft::mesh`. This was the right call for the soft-body foundation — `sim-soft`'s tet mesh has FEM-specific adjacency layouts and cached invariants. But two future use cases could push tet support upward into `mesh-types`:

1. **`sim-cast` sacrificial cores.** Hollow-cast parts may use lattice tet cores (e.g., PVA-printed gyroid tetrahedralized for stress analysis). If `sim-cast` grows tet meshing, the natural home for the type is `mesh-types`, not within `sim-cast`'s own crate.
2. **Generic tet-mesh interchange.** If `cf-design` ever needs to author tet-mesh primitives (rare), or if future external tools want tet-aware loading from the `mesh-io` family (e.g., a future `.msh` Gmsh format), the type belongs at the foundation tier.

If/when this happens, the architectural question is: does `sim-soft::mesh::HandBuiltTetMesh` move to `mesh-types::TetMesh` (hard refactor; affects every soft-body test), or does `mesh-types::TetMesh` land as a separate type with conversion shims to/from `sim-soft`'s native form (soft introduction; some duplication)? The depth pass enumerates the tradeoffs.

Currently no driving force; flagged as future-when-relevant.

### Typed precondition wrappers

Many operations across the ecosystem assume preconditions that aren't enforced at the type level:

- `mesh-sdf::point_in_mesh` assumes watertight.
- `mesh-offset::offset_mesh` assumes watertight + manifold.
- `mesh-shell::generate_shell` assumes watertight + manifold + closed boundary.
- `mesh-printability::validate_for_printing` assumes manifold.

Each consumer either trusts the input or runs its own repair pass. A type-level alternative: `CleanMesh` (or similar) wrapping `IndexedMesh` with a static guarantee that `mesh-repair::validate_mesh` returned without errors. Operations downstream of repair accept `&CleanMesh` instead of `&IndexedMesh`.

Pro: precondition violations become compile-time errors; clearer API contracts.
Con: every existing call site updates; the wrapper carries no runtime value beyond a marker; the discipline can be circumvented (`unsafe { CleanMesh::new_assumed_clean(mesh) }`).

The depth pass discusses this in detail; current lean is **not yet** — value/cost ratio doesn't justify the migration today.

### Cross-domain reward composition

Both soft-body and casting will eventually contribute to a unified `RewardBreakdown` for the design loop. The mesh ecosystem doesn't directly compute reward terms today, but several operations could feed them:

- `mesh-printability::validate_for_printing` → printability reward term.
- `mesh-shell::ShellBuilder::build` failure → manufacturability classification.
- `mesh-measure::dimensions` → size-budget reward term.

Should the mesh ecosystem grow a reward-aware surface, or should reward composition be entirely the consuming domain's responsibility? Today: latter. Future: open question if the design loop's reward composition grows complicated enough that mesh-side operations should be self-aware about their role in optimization.

### GPU acceleration

All mesh-ecosystem crates are pure CPU. Several operations are computationally heavy and would benefit from GPU acceleration:

- `mesh-offset` marching cubes kernel.
- `mesh-sdf::SignedDistanceField::distance` (per-query closest-point search).
- `mesh-printability::find_optimal_orientation` (search over many orientations).
- `mesh-lattice` TPMS marching-cubes extraction.

Phase E in the soft-body book covers the GPU plumbing (wgpu-based, separate device from Bevy's rendering). The mesh ecosystem could ride the same machinery — `mesh-offset` as the most natural first port. Currently no driving force; flagged for when/if performance pressure surfaces.

### Additional file formats

Current set: STL, OBJ, PLY, 3MF, STEP. Realistic additions:

- **VTK** — scientific visualization standard; preferred by ParaView for time-varying simulation data. Soft-body PLY frame-sequences could equivalently be VTK time-series.
- **USD (Universal Scene Description)** — Pixar's modern interchange format; gaining traction in industrial pipelines.
- **glTF** — modern game-engine-friendly format with PBR materials and animation; relevant if CortenForge's visualization layer wants to export to web viewers.
- **Gmsh `.msh`** — explicit volumetric mesh format; relevant if tet-mesh support matures.

Each is a feature-gated addition similar to the existing 3MF/STEP pattern. No current driving force; depth pass discusses each format's pros/cons.

## What this study does NOT commit to

- **Specific timelines.** The first stable release has shipped; everything past it is gated on use cases driving it.
- **Stable API guarantees beyond what's locked at first stable release.** APIs may evolve as use cases bite. The deprecation policy follows whatever the workspace policy is at API-stability time.
- **Performance targets.** No specific FPS / throughput / memory targets. Foundation-level performance is "fast enough not to bottleneck consumers"; will be calibrated against real workloads as they materialize.
