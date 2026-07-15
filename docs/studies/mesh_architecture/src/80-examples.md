# The mesh examples inventory

The mesh ecosystem ships example coverage of the ten public crates below — each with at least one example demonstrating its load-bearing capabilities; the lattice domain ships one stress-test (five modules) spanning TPMS, strut, density-graded, shape-bounded, and mesh-bounded composite paths. Two further public crates — `mesh-loft` (join two painted contact patches into one watertight bushing) and `mesh-select` (brush-based face selection) — are not yet exampled; like the [v0.9 candidates](100-roadmap.md#v09-candidates), they await a consumer to drive their first example.

Examples are workspace member crates under `examples/mesh/`: standalone examples live flat at `examples/mesh/<name>/{Cargo.toml, src/main.rs, README.md, out/}`, while a consolidated domain (measure, offset, shell-generation, printability, lattice) ships one `examples/mesh/<domain>/stress-test/` validator whose modules each carry a former per-example fixture + oracle. The aggregator at [`examples/mesh/README.md`](../../../../examples/mesh/README.md) is the canonical per-example navigator; per-example READMEs are the depth source for numerical anchors and visuals notes. This part synthesizes those examples into a pedagogical reading order — six bands ascending from foundational types through repair, SDF and offset, measurement, manufacturing-aware operations, and lattice composites — with cross-references back to the rest of the book and forward to the v0.9 candidates [Part 10](100-roadmap.md) tracks.

## Reading order

The examples below are arranged in six pedagogical bands. Within each band, examples are sub-grouped by crate to mirror the workspace layout. The progression matches Parts 1-7 of this book: read this part alongside [Part 1](10-types.md) through [Part 7](70-lattices.md) to see each concept land as runnable code with locked numerical anchors.

## Band 1 — foundational types and I/O

These examples build the working vocabulary for the rest: how to construct an `AttributedMesh`, how to round-trip it through PLY with custom per-vertex scalars, and how the format family (STL / OBJ / PLY) preserves different facets of the same geometry.

### `mesh-types`

- **`attributed-mesh-basics`** — the SoA attributed-mesh shape: built-in slots (positions, normals, colors, zone IDs), the extras `HashMap<String, Vec<f32>>`, and length validation. No I/O, no operation crates — just the type itself.
  Pairs with [Part 1 — Core types](10-types.md).

### `mesh-io`

- **`ply-with-custom-attributes`** — per-vertex custom-attribute PLY round-trip via `save_ply_attributed`. Demonstrates the load-bearing infrastructure for downstream crates that emit per-vertex scalars (`mesh-sdf-distance-query`'s grid below; soft-body PLY snapshots in the wider workspace).
  Pairs with [Part 2 — File I/O](20-io.md).
- **`format-conversion`** — STL ↔ PLY ↔ OBJ. Surfaces what each format preserves (geometry vs topology vs custom scalars) and where round-trip integrity holds vs degrades.
  Pairs with [Part 2 — File I/O](20-io.md).

## Band 2 — repair

A single example covers the canonical broken-to-clean pipeline. Repair sits between "load" and "operate": every operation downstream of `mesh-repair::validate_mesh` either trusts the input or runs its own repair pass — a pattern explored further in the typed-precondition-wrappers open question in [Part 10](100-roadmap.md).

### `mesh-repair`

- **`mesh-repair-walkthrough`** — validate → repair → fill → re-wind → re-validate, on a deliberately-broken fixture (duplicate vertices + reversed-winding face + small hole + degenerate triangle). The repair pipeline made visible end-to-end.
  Pairs with [Part 4 — Repair](40-repair.md).

## Band 3 — SDF and offset

These examples cover the mesh-to-SDF half of the bridge: building signed-distance fields, querying them point-wise and on bulk grids, and using them to generate offset surfaces via marching cubes. The SDF example surfaced two v0.9 candidates (sign-convention robustness at vertex / edge regions, and BVH-acceleration of the closest-point primitive); the offset example locks in the Steiner-vs-polytope volume asymmetry and the genus-0 topological invariant of the offset output.

### `mesh-offset`

- **`offset/stress-test`** — SDF + marching cubes offset in both directions on a closed cube, the `mesh-offset` domain validator (folded from the former `mesh-offset-outward` / `mesh-offset-inward` pair). Anchors the expansion/erosion asymmetry — outward dilation rounds corners (Steiner-Minkowski volume; intrinsic to the operation, not a bug) while inward erosion preserves the sharp polytope (exact `(1 + 2d)³`) — and the **grid-alignment topology pitfall**: an eroded face landing exactly on marching-cubes sample planes stitches spurious handles (genus > 0), which an off-grid resolution avoids. The clean output is now watertight and outward-wound directly (the §Q-5 winding fix plus the MC edge-vertex cache retired the old inside-out vertex-soup output and its per-face-flip workaround).
  Pairs with [Part 3 — SDF and offset](30-sdf-and-offset.md).

### `mesh-sdf`

- **`mesh-sdf-distance-query`** — numerical SDF on a unit octahedron. Combines `SignedDistanceField` cached queries, free-fn one-shots, the underlying primitives (`closest_point_on_triangle`, `ray_triangle_intersect`, `point_segment_distance_squared`), `point_in_mesh` ray-casting, and a 1000-point bulk-grid PLY with `extras["signed_distance"]` colour-mapped scalars.
  Surfaced v0.9 candidates #2 (SDF sign-convention upgrade) and #3 (BVH acceleration). Pairs with [Part 3 — SDF and offset](30-sdf-and-offset.md).

## Band 4 — measurement

One stress-test (`measure/stress-test`) covers the geometric-inspection surface across three modules: bounding boxes (axis-aligned vs PCA-based oriented), planar cross-sections, and point- and mesh-distance composition into Hausdorff. Each module surfaced one or more v0.9 candidates where spec authoring or example execution exposed an FP-stability or formula gap that didn't show up under unit-test coverage.

### `mesh-measure` (folded into `measure/stress-test`)

- **`bounding_box` module** — AABB and OBB on a two-shape fixture (axis-aligned cube + 45°-rotated brick). Demonstrates when AABB and OBB coincide, when they diverge (PCA recovers the brick's `(20, 10, 10)` extents), and that the folk-intuition "OBB ⊆ AABB" is false for any non-trivial OBB rotation — corners extend OUTSIDE the AABB envelope by `half_extent · sin(rotation_angle)`.
  Surfaced v0.9 candidates #5 (tolerance-aware `OrientedBoundingBox::contains` — shipped in #613) and #6 (documenting the OBB ⊄ AABB caveat). Pairs with [Part 6 — Measurement](60-measurement.md).
- **`cross_section` module** — planar slicing of a 32-segment closed cylinder: mid-slice, 10-slice stack, `circumference_at_height` and `area_at_height` helpers, out-of-mesh handling, and plane-normal auto-normalization.
  Surfaced v0.9 candidate #7 (shoelace-weighted polygon centroid — shipped in #614; the former naive average biased chain-closure-duplicate contours by ~0.077 mm on this fixture, now corrected to the true centroid). Pairs with [Part 6 — Measurement](60-measurement.md).
- **`distance` module** — point-to-point + unsigned point-to-mesh + symmetric Hausdorff between two vertex-disjoint unit cubes. Demonstrates that Hausdorff is composition over the exposed primitives, not a built-in — a deliberate API choice deferring the convenience wrapper until a real consumer drives it.
  Pairs with [Part 6 — Measurement](60-measurement.md).

## Band 5 — shell and printability

A `mesh-shell` stress-test and a `mesh-printability` stress-test cover the manufacturing-aware operations: producing wall-thickness shells (the foundational geometry for every casting and printing workflow) and validating a candidate mesh against per-technology printer constraints. The printability stress-test is the densest module set in the inventory because each detector keeps its own pedagogical fixture surfacing one detector at a time, plus a final showcase exercising six detectors on a single multi-shell bracket.

### `mesh-shell`

- **`shell-generation/stress-test`** — the `mesh-shell` domain validator, exercising both wall-generation methods on the same 10mm cube geometry (folded from the former `shell-generation-fast` / `-high-quality` pair). The `fast` module (normal-based, open-topped box) preserves vertex correspondence but skews perpendicular wall thickness by `1/√k` at corners (intrinsic to the normal path, not a bug); the `high_quality` module (SDF + marching cubes, closed cube) holds uniform thickness to within half a voxel at the cost of MC re-triangulation and Steiner-Minkowski rounding at sharp creases. Both now produce consistently-wound shells — the normal path via the rim-winding fix in `generate_rim`, the SDF path via the 11.5.x per-face-flip fix.
  Pairs with [Part 5 — Shell and printability](50-shell-and-print.md).

### `mesh-printability`

The `printability/stress-test` crate is the domain validator, its seven modules mirroring the v0.8 detector inventory documented in [Part 5 — Shell and printability](50-shell-and-print.md). Each module keeps a fixture purpose-built to surface one detector cleanly, plus regression coverage on adjacent detectors that would naturally co-fire on the fixture. Folded from eight former per-crate examples; the former `printability-thin-wall` was dropped as a strict subset of `technology_sweep`'s FDM slice (same hollow-box family, analytical oracles, no coverage lost).

- **`long_bridge`** — `LongBridge` detector. 24-vertex H-shape with a 20 mm middle bridge above FDM's 10 mm `max_bridge_span`; the two 5 mm cantilevers cluster independently. Silent-drop on SLS / MJF (powder support). The bridge midpoint coincides with the middle-overhang centroid (cross-detector co-flag).
- **`trapped_volume`** — `TrappedVolume` detector. Solid cube with a sealed sphere cavity; voxel volume validated against analytical `(4/3)πr³`; `Info` on FDM (extrusion prints through), `Critical` on SLA / SLS / MJF (resin / powder traps).
- **`self_intersecting`** — `SelfIntersecting` detector. Two interpenetrating cylinders without boolean cleanup; the `max_reported` cap biconditional (truncation suffix iff cap hit) plus a single-cylinder convex-mesh no-false-positive control.
- **`small_feature`** — `SmallFeature` detector. 30 mm cube + 0.2 mm hex-prism burr; Warning band; ThinWall co-flag drives `is_printable() == false`.
- **`orientation`** — `PrinterConfig::build_up_direction` parametrization. Leaning cylinder revalidated under three runs (original, manual `R_Y(-60°)` rotation, build-up direction set to the axis); the v0.8 Gap L invariant is mesh-rotation ≡ up-vector-rotation when both are exact.
- **`technology_sweep`** — cross-technology severity divergence. Same hollow box validated under FDM / SLA / SLS / MJF; each technology fails `is_printable()` for a different reason. Its FDM slice is the canonical hollow-box `ThinWall` oracle.
- **`showcase`** — capstone. Single bracket fixture (528 vertices / 1032 faces, five vertex-disjoint shells) exercising six detectors at once; surfaces five deliberate spec deviations as load-bearing pedagogical observations.

All seven modules pair with [Part 5 — Shell and printability](50-shell-and-print.md).

## Band 6 — lattices and composites

The `lattice/stress-test` crate is the `mesh-lattice` domain validator, its five modules covering the three lattice approaches in the workspace: TPMS implicit-surface lattices via marching cubes, strut graphs via cylindrical beams between grid nodes, and the FDM-style composite (shell + lattice + caps + connections) that combines all three with a `mesh-offset`-driven inward shell. Folded from five former per-crate examples; all five carry distinct oracles (no subsumption). Pairs with [Part 7 — Lattices](70-lattices.md).

- **`tpms_gyroid`** — TPMS lattice via the gyroid implicit surface. Direct anchors on `gyroid` / `density_to_threshold` / `make_shell`, then `generate_lattice` for a 30 mm cube at density 0.5, verifying every marching-cubes vertex lands on the analytical shell surface. Currently locks the un-welded MC vertex-soup signature `vertex_count == 3 × triangle_count`; v0.9 candidate #10 (welded MC output) is trigger-gated to a real consumer needing visual aesthetic or file-size compression.
- **`shape_bounded`** — boundary-conforming TPMS via `with_shape_sdf`. Gyroid clipped to an analytical sphere of radius 12 mm; the `is_outside_shape` predicate plus a confinement invariant (every vertex within `radius + voxel_size + cushion` of the SDF zero level-set). The analytical-SDF path — complementary to the mesh-SDF path used by `generate_infill` below.
- **`strut_cubic`** — cubic strut lattice. Direct anchors on `generate_strut` / `combine_struts` / `estimate_strut_volume`, then `generate_lattice` with `with_beam_export(true)` populating the 3MF `BeamLatticeData` precursor (216 deduplicated grid nodes; 540 beams; total length 2700 mm). Surfaces v0.9 candidate #16 (3MF beam writer in `mesh-io`).
- **`density_gradient`** — variable-density octet-truss via `DensityMap::Gradient`. The full `DensityMap` enum plus per-beam `r1 = strut_thickness/2 × √density` modulation; four discrete cell-z strata produce four discrete radius bands visible end-to-end.
- **`mesh_bounded_infill`** — FDM-style composite via `generate_infill` on a hand-authored 50 mm watertight cube. Outer inward-offset shell (the F6 gap-a `mesh-offset` integration, anchored by the resolution-independent Euler invariant `2V−F=8`), interior cubic lattice clipped by mesh-SDF intersection (gap e), bridging connection struts to the nearest shell point (gap b), and solid caps (gap c); the §Q-5 hollow-shell winding guard catches the inner-wall sign flip. Four PLY outputs: input, shell, lattice, composite. Surfaces v0.9 candidates #12-15. Also pairs with [Part 5 — Shell and printability](50-shell-and-print.md).

## Layout convention

Every example is a workspace member crate with dash-case directory names. The standardization arc is migrating mesh examples to the uniform `examples/mesh/<domain>/<crate>` layout that matches `fundamentals/sim-cpu`: each domain consolidates into one headless `stress-test` validator (package `example-<domain>-stress-test`, marked `[package.metadata.cortenforge] example_kind = "validator"`) alongside any visual demos, so the CI `run-validators` gate tracks one physics-validation superset per domain. Flat `example-mesh-<name>` examples not yet folded keep their names until their domain is migrated — `measure/stress-test` (folding the three former `mesh-measure-*` examples) is the first. Generated PLY artifacts go to a per-crate `out/` directory, gitignored via `**/out/` at the `examples/mesh/.gitignore` level.

The canonical per-example README template — including the `## What it does` / `## Numerical anchors` / `## Visuals` / `## Run` sections — lives in [`examples/mesh/README.md`](../../../../examples/mesh/README.md), the single source of truth so it doesn't drift between this book and the examples directory.

## Cadence and review

Per `feedback_one_at_a_time` and `feedback_one_at_a_time_review`, each example is reviewed individually before the next lands: a numerical-anchor pass (`assert_*` calls in `src/main.rs` under `feedback_math_pass_first_handauthored` — exit-0 IS the correctness signal) plus an optional visuals pass on the PLY output. Examples bundle into PRs per `feedback_pr_size_ci_economics`: internal commit segmentation is the splitting lever, not PR fragmentation.

## What this part commits to

- The inventory above is the canonical mesh examples list at the platform's first stable release (originally 25 flat crates; the standardization arc is consolidating them per-domain into `stress-test` validators, so the crate count drops as domains are folded — `measure/stress-test` was the first, folding three into one).
- New examples MUST follow the standardization layout (one headless `stress-test` validator per domain at `examples/mesh/<domain>/stress-test/`, package `example-<domain>-stress-test`, museum-plaque README per `feedback_museum_plaque_readmes`, two-pass reviewed).
- Per-example numerical anchors and expected outputs live in each example's README and `src/main.rs` — the depth source. The aggregator at `examples/mesh/README.md` is index-granularity (link out, no anchor citation, per `feedback_museum_plaque_readmes` extended to docs aggregators); this Part is the pedagogical narrative tying examples to the rest of the book.
- New examples that surface gaps in the platform should add the gap to [Part 10's v0.9 candidates list](100-roadmap.md#v09-candidates) (or an appropriate Open architectural question section), with a trigger statement describing what consumer arrival looks like.
