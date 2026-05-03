# The mesh examples inventory

The mesh ecosystem ships with complete example coverage of all 10 public crates. Every crate has at least one example demonstrating its load-bearing capabilities; the lattice family ships five examples spanning TPMS, strut, density-graded, shape-bounded, and shell-bounded composite paths.

Each example is a workspace member crate at `examples/mesh/<name>/{Cargo.toml, src/main.rs, README.md, out/}`. The aggregator at [`examples/mesh/README.md`](../../../../examples/mesh/README.md) is the canonical per-example navigator; per-example READMEs are the depth source for numerical anchors and visuals notes. This part synthesizes those examples into a pedagogical reading order — six bands ascending from foundational types through repair, SDF and offset, measurement, manufacturing-aware operations, and lattice composites — with cross-references back to the rest of the book and forward to the v0.9 candidates [Part 10](100-roadmap.md) tracks.

## Reading order

The 25 examples below are arranged in six pedagogical bands. Within each band, examples are sub-grouped by crate to mirror the workspace layout. The progression matches Parts 1-7 of this book: read this part alongside [Part 1](10-types.md) through [Part 7](70-lattices.md) to see each concept land as runnable code with locked numerical anchors.

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

These examples cover the mesh-to-SDF half of the bridge: building signed-distance fields, querying them point-wise and on bulk grids, and using them to generate offset surfaces via marching cubes. The SDF example surfaced two v0.9 candidates (sign-convention robustness at vertex / edge regions, and BVH-acceleration of the closest-point primitive); the offset examples lock in the un-welded MC vertex-soup signature.

### `mesh-offset`

- **`mesh-offset-outward`** — SDF + marching cubes outward offset. Surfaces corner rounding (Steiner-Minkowski; intrinsic to the offset operation, not a bug) and the per-face-flip pattern needed to render un-welded MC vertex-soup output correctly in single-sided viewers.
  Pairs with [Part 3 — SDF and offset](30-sdf-and-offset.md).
- **`mesh-offset-inward`** — SDF + marching cubes inward offset on a closed cube. Sharp polytope preservation (no rounding). Pair-companion to `mesh-offset-outward` highlighting the asymmetry between expansion and erosion.
  Pairs with [Part 3 — SDF and offset](30-sdf-and-offset.md).

### `mesh-sdf`

- **`mesh-sdf-distance-query`** — numerical SDF on a unit octahedron. Combines `SignedDistanceField` cached queries, free-fn one-shots, the underlying primitives (`closest_point_on_triangle`, `ray_triangle_intersect`, `point_segment_distance_squared`), `point_in_mesh` ray-casting, and a 1000-point bulk-grid PLY with `extras["signed_distance"]` colour-mapped scalars.
  Surfaced v0.9 candidates #2 (SDF sign-convention upgrade) and #3 (BVH acceleration). Pairs with [Part 3 — SDF and offset](30-sdf-and-offset.md).

## Band 4 — measurement

Three examples cover the geometric-inspection surface: bounding boxes (axis-aligned vs PCA-based oriented), planar cross-sections, and point- and mesh-distance composition into Hausdorff. Each one surfaced one or more v0.9 candidates where spec authoring or example execution exposed an FP-stability or formula gap that didn't show up under unit-test coverage.

### `mesh-measure`

- **`mesh-measure-bounding-box`** — AABB and OBB on a two-shape fixture (axis-aligned cube + 45°-rotated brick). Demonstrates when AABB and OBB coincide, when they diverge (PCA recovers the brick's `(20, 10, 10)` extents), and that the folk-intuition "OBB ⊆ AABB" is false for any non-trivial OBB rotation — corners extend OUTSIDE the AABB envelope by `half_extent · sin(rotation_angle)`.
  Surfaced v0.9 candidates #5 (tolerance-aware `OrientedBoundingBox::contains`) and #6 (documenting the OBB ⊄ AABB caveat). Pairs with [Part 6 — Measurement](60-measurement.md).
- **`mesh-measure-cross-section`** — planar slicing of a 32-segment closed cylinder: mid-slice, 10-slice stack, `circumference_at_height` and `area_at_height` helpers, out-of-mesh handling, and plane-normal auto-normalization.
  Surfaced v0.9 candidate #7 (shoelace-weighted polygon centroid; the current naive average biases on chain-closure-duplicate contours by ~0.077 mm on this fixture). Pairs with [Part 6 — Measurement](60-measurement.md).
- **`mesh-measure-distance-to-mesh`** — point-to-point + unsigned point-to-mesh + symmetric Hausdorff between two vertex-disjoint unit cubes. Demonstrates that Hausdorff is composition over the exposed primitives, not a built-in — a deliberate API choice deferring the convenience wrapper until a real consumer drives it.
  Pairs with [Part 6 — Measurement](60-measurement.md).

## Band 5 — shell and printability

Two `mesh-shell` examples and eight `mesh-printability` examples cover the manufacturing-aware operations: producing wall-thickness shells (the foundational geometry for every casting and printing workflow) and validating a candidate mesh against per-technology printer constraints. The printability set is the densest in the inventory because each detector ships its own pedagogical fixture surfacing one detector at a time, plus a final showcase exercising six detectors on a single multi-shell bracket.

### `mesh-shell`

- **`shell-generation-fast`** — normal-based shell on an open-topped box. Preserves vertex correspondence; perpendicular wall thickness varies as `1/√k` at corners (geometric distortion is intrinsic to the normal-based path, not a bug).
  Pairs with [Part 5 — Shell and printability](50-shell-and-print.md).
- **`shell-generation-high-quality`** — SDF-based shell on a closed cube. Uniform perpendicular wall thickness everywhere; Steiner-Minkowski rounding at sharp creases (the geometric trade for uniformity). Side-by-side comparison with the fast path.
  Pairs with [Part 5 — Shell and printability](50-shell-and-print.md).

### `mesh-printability`

These eight examples mirror the v0.8 detector inventory documented in [Part 5 — Shell and printability](50-shell-and-print.md). Each ships a fixture purpose-built to surface one detector cleanly, plus regression coverage on adjacent detectors that would naturally co-fire on the fixture.

- **`printability-thin-wall`** — `ThinWall` detector. Double-walled hollow box with a 0.4 mm top wall thinned below FDM's 1.0 mm threshold; two Critical clusters plus a cavity-ceiling overhang co-flag.
- **`printability-long-bridge`** — `LongBridge` detector. 24-vertex H-shape with a 20 mm middle bridge above FDM's 10 mm `max_bridge_span`; the two 5 mm cantilevers cluster independently. Silent-drop on SLS / MJF (powder support).
- **`printability-trapped-volume`** — `TrappedVolume` detector. Solid cube with a sealed sphere cavity; `Info` on FDM (extrusion prints through), `Critical` on SLA / SLS / MJF (resin / powder traps).
- **`printability-self-intersecting`** — `SelfIntersecting` detector. Two interpenetrating cylinders without boolean cleanup; flags 100 triangle pairs (the `mesh-repair::IntersectionParams::default` `max_reported` cap) plus a single-cylinder convex-mesh regression.
- **`printability-small-feature`** — `SmallFeature` detector. 30 mm cube + 0.2 mm hex-prism burr; Warning band; ThinWall co-flag drives `is_printable() == false`.
- **`printability-orientation`** — `PrinterConfig::build_up_direction` parametrization. Leaning cylinder revalidated under three runs (original, manual `R_Y(-60°)` rotation, build-up direction set to the axis); the v0.8 Gap L invariant is mesh-rotation ≡ up-vector-rotation when both are exact.
- **`printability-technology-sweep`** — cross-technology severity divergence. Same hollow box validated under FDM / SLA / SLS / MJF; each technology fails `is_printable()` for a different reason.
- **`printability-showcase`** — capstone. Single bracket fixture (528 vertices / 1032 faces, five vertex-disjoint shells) exercising six detectors at once; surfaces five deliberate spec deviations as load-bearing pedagogical observations.

All eight pair with [Part 5 — Shell and printability](50-shell-and-print.md).

## Band 6 — lattices and composites

Five `mesh-lattice` examples cover the three lattice approaches in the workspace: TPMS implicit-surface lattices via marching cubes, strut graphs via cylindrical beams between grid nodes, and the FDM-style composite (shell + lattice + caps + connections) that combines all three with a `mesh-offset`-driven inward shell.

### TPMS path — implicit-surface lattices

- **`mesh-lattice-tpms-gyroid`** — TPMS lattice via the gyroid implicit surface. Direct anchors on `gyroid` / `density_to_threshold` / `make_shell`, then `generate_lattice` for a 30 mm cube at density 0.5; locks the un-welded MC vertex-soup signature `vertex_count == 3 × triangle_count` BIT-EXACT.
  v0.9 candidate #10 (welded MC output) is trigger-gated to a real consumer needing visual aesthetic or file-size compression. Pairs with [Part 7 — Lattices](70-lattices.md).
- **`mesh-lattice-shape-bounded`** — boundary-conforming TPMS via `with_shape_sdf`. Gyroid clipped to an analytical sphere of radius 12 mm; trim drops 72.8% of vertices vs the bbox-filling baseline (87 480 vs 321 084). Demonstrates the analytical-SDF path — complementary to the mesh-SDF path used by `generate_infill` below.
  Pairs with [Part 7 — Lattices](70-lattices.md).

### Strut path — cylindrical beams between grid nodes

- **`mesh-lattice-strut-cubic`** — cubic strut lattice. Direct anchors on `generate_strut` / `combine_struts` / `estimate_strut_volume`, then `generate_lattice` with `with_beam_export(true)` populating the 3MF `BeamLatticeData` precursor (216 deduplicated grid nodes; 540 beams; total length 2700 mm).
  Surfaces v0.9 candidate #16 (3MF beam writer in `mesh-io` — the data model is populated, the writer is the missing half). Pairs with [Part 7 — Lattices](70-lattices.md).
- **`mesh-lattice-density-gradient`** — variable-density octet-truss via `DensityMap::Gradient`. Per-beam `r1 = strut_thickness/2 × √density` modulation; four discrete cell-z strata produce four discrete radius bands visible end-to-end in the output.
  Pairs with [Part 7 — Lattices](70-lattices.md).

### Composite (FDM infill) — shell + lattice + caps + connections

- **`mesh-lattice-mesh-bounded-infill`** — FDM-style composite via `generate_infill` on a hand-authored 50 mm watertight cube. Outer inward-offset shell (the F6 gap-a `mesh-offset` integration), interior cubic lattice clipped by mesh-SDF intersection on the offset shell (gap e), bridging connection struts from each unique lattice grid-node to the nearest shell point (gap b), and solid caps at the build-plate-aligned faces (gap c). Four PLY outputs: input, shell, lattice, composite.
  Surfaces v0.9 candidates #12-15 (0%-infill early-return still has the gap-a pattern; non-convex inputs whose AABB inset includes outside-part regions; `shell_thickness == 0` edge case; explicit `LatticeParams::layer_height` field). Pairs with [Part 7 — Lattices](70-lattices.md) and [Part 5 — Shell and printability](50-shell-and-print.md).

## Layout convention

Every example is a workspace member crate. Directory names are dash-case to match the rest of the workspace (`mesh-pipeline`, `hello-solid`, etc.); package names are `example-mesh-<dash-case-name>`. Generated PLY artifacts go to a per-example `out/` directory, gitignored at the `examples/mesh/.gitignore` level.

The canonical per-example README template — including the `## What it does` / `## Numerical anchors` / `## Visuals` / `## Run` sections — lives in [`examples/mesh/README.md`](../../../../examples/mesh/README.md), the single source of truth so it doesn't drift between this book and the examples directory.

## Cadence and review

Per `feedback_one_at_a_time` and `feedback_one_at_a_time_review`, each example is reviewed individually before the next lands: a numerical-anchor pass (`assert_*` calls in `src/main.rs` under `feedback_math_pass_first_handauthored` — exit-0 IS the correctness signal) plus an optional visuals pass on the PLY output. Examples bundle into PRs per `feedback_pr_size_ci_economics`: internal commit segmentation is the splitting lever, not PR fragmentation.

## What this part commits to

- The 25-example inventory above is the canonical mesh examples list at the platform's first stable release.
- New examples MUST follow the layout convention (workspace member crate at `examples/mesh/<dash-case-name>/`, museum-plaque README per `feedback_museum_plaque_readmes`, two-pass reviewed).
- Per-example numerical anchors and expected outputs live in each example's README and `src/main.rs` — the depth source. The aggregator at `examples/mesh/README.md` is index-granularity (link out, no anchor citation, per `feedback_museum_plaque_readmes` extended to docs aggregators); this Part is the pedagogical narrative tying examples to the rest of the book.
- New examples that surface gaps in the platform should add the gap to [Part 10's v0.9 candidates list](100-roadmap.md#v09-candidates) (or an appropriate Open architectural question section), with a trigger statement describing what consumer arrival looks like.
