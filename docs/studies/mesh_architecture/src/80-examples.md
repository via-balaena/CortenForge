# The mesh examples inventory

## The examples gap

Until the examples PR lands, **the mesh ecosystem has zero `examples/` directories and zero `[[example]]` Cargo entries across all 10 crates** — entirely test-driven, no visual validation. This part documents the planned mesh examples directory at workspace `examples/mesh/`, organized in three tiers by priority.

Cadence per the patience-driven philosophy: each example is a workspace member crate at `examples/mesh/<name>/{Cargo.toml, src/main.rs, README.md, out/}`, dumps PLY frames where applicable, and is reviewed two-pass (numbers from Claude, visuals from the user) before the next example lands. PRs bundle multiple commits per the CI-economics rule.

## Layout convention

Every example is a workspace member crate. Directory names are dash-case to match the rest of the workspace (`hello-solid`, `mesh-pipeline`, etc.); package names are `example-mesh-<dash-case-name>`. Generated PLY artifacts go to a per-example `out/` directory, gitignored at the `examples/mesh/.gitignore` level.

The canonical per-example README template lives in [`examples/mesh/README.md`](../../../../examples/mesh/README.md) — single source of truth so it doesn't drift between this book and the examples directory.

## Tier 1 — first PR (validates the new infrastructure)

1. **`ply-with-custom-attributes`** — Build a unit cube as `AttributedMesh` with a custom `extras["height"]` per-vertex scalar. Save via `save_ply_attributed`. Reload, verify round-trip. Concept: **the platform supports per-vertex custom-attribute PLY snapshots**, which is the load-bearing infrastructure for soft-body PLY output.

## Tier 2 — alongside soft-body, foundation for sim-cast (10 examples)

2. **`format-conversion`** — Load STL, save PLY, load PLY, save OBJ. Verify topology preserved. Concept: **format auto-detection and round-trip integrity across STL/OBJ/PLY**.
3. **`attributed-mesh-basics`** — Build `AttributedMesh` with `compute_normals`, set colors, set zone_ids, set extras. No I/O, no operation crates — just the type itself. Concept: **the SoA attributed-mesh shape**.
4. **`mesh-repair-walkthrough`** — Construct a deliberately-broken mesh (duplicate vertices + reversed-winding face + small hole + degenerate triangle). Run `validate_mesh`, print the report. Apply `repair_mesh`, observe the changes. Re-validate. Concept: **the repair pipeline pattern from broken to clean**.
5. **`mesh-offset-outward`** — Unit cube + `offset_mesh(0.1)` (outward 0.1mm). Save both before/after as PLY. Concept: **expansion via SDF + marching cubes**.
6. **`mesh-offset-inward`** — Unit cube + `offset_mesh(-0.1)` (inward 0.1mm). Concept: **erosion / cavity-prep**, the foundational operation for mold-cavity inversion.
7. **`shell-generation-fast`** — Solid mesh → `ShellBuilder::wall_thickness(2.0).fast().build()`. Save resulting shell as PLY. Concept: **normal-based shell generation** (fast, but wall thickness varies at corners).
8. **`shell-generation-high-quality`** — Same input as #7, but `.high_quality()`. Save side-by-side comparison. Concept: **SDF-based shell generation** (slower, uniform wall thickness).
9. **`printability-fdm-validation`** — Load a deliberately-overhanging mesh, run `validate_for_printing` with `PrinterConfig::fdm_default`, print the issue list. Concept: **manufacturability constraint reporting**.
10. **`printability-orientation`** — Same input as #9, but call `find_optimal_orientation` first; re-validate. Concept: **automatic orientation reduces overhang**.
11. **`cross-section-sweep`** — Take a known body (e.g., the layered silicone device's outer shell), sweep a plane along Z, dump one PLY per slice as 2D contour polylines. Concept: **cross-section visualization** (also serves as a primitive for the soft-body deformation slice viz).

## Tier 3 — deferred (5 examples, useful but not blocking soft-body or casting)

12. **`sdf-distance-field`** — Sample `mesh-sdf::SignedDistanceField` on a 3D grid, dump as a PLY with `extras["signed_distance"]` colormap. Concept: **the numerical SDF visualized**.
13. **`lattice-gyroid-in-box`** — Generate a gyroid lattice in a bounding box. Save as PLY. Concept: **TPMS lattice generation**.
14. **`lattice-density-gradient`** — Same as #13 but with a vertical density gradient. Concept: **variable-density lattices**.
15. **`lattice-inside-shell`** — Generate a shell, generate a lattice in its bounding box, boolean-combine to keep lattice inside shell. Concept: **printable lightweight composite (shell + infill)**.
16. **`self-intersection-detection`** — Construct a self-intersecting mesh (two intersecting cubes), run `detect_self_intersections`, render the intersecting triangles in distinct color. Concept: **the hardest mesh-repair case made visible**.

## Cadence and review

Per `feedback_one_at_a_time` and `feedback_one_at_a_time_review`:

- Each example is reviewed individually before the next lands.
- Two-pass review: Claude verifies numerical anchors (the PLY round-trip succeeded, the manifold check passes, the offset distance is correct); user verifies visuals (the cross-section looks right, the shell wall is uniform, the gyroid is recognizable).
- Tier 1 lands in the same PR as the gap-fix commits (extensible attributes + custom-attribute PLY writer).
- Tier 2 examples land as additive commits in the same PR or a follow-up PR depending on bandwidth.
- Tier 3 examples land later, as their underlying use cases (lattice work for the layered silicone device middle layer; sdf-distance work for casting demoldability analysis) drive them.

## What this part commits to

- The 16-example inventory above is the **canonical mesh examples plan** for the platform.
- New examples added in the future MUST follow the layout convention above (workspace member crate at `examples/mesh/<dash-case-name>/`, museum-plaque README, two-pass reviewed).
- The PLY-output convention is the canonical persistence format for example artifacts; future Bevy viewer (in `sim-bevy`) reads PLY directories produced by examples.
- Per-example expected-output specifications, suggested viewer-side colormap settings, and cross-references between examples (e.g., #4's broken mesh is a good input to #11's cross-section sweep) land alongside each example as it ships, not all at once.
