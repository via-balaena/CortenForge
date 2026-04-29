# The mesh examples inventory

What this part covers (when authored to depth):

## The examples gap

Until the examples PR lands, **the mesh ecosystem has zero `examples/` directories and zero `[[example]]` Cargo entries across all 10 crates** — entirely test-driven, no visual validation. This part documents the planned mesh examples directory at workspace `examples/mesh/`, organized in three tiers by priority.

Cadence per the patience-driven philosophy: each example is `examples/mesh/<name>/{main.rs, README.md}`, dumps PLY frames where applicable, and is reviewed two-pass (numbers from Claude, visuals from the user) before the next example lands. PRs bundle multiple commits per the CI-economics rule.

## Tier 1 — first PR (validates the new infrastructure)

1. **`ply_with_custom_attributes`** — Build a unit cube as `AttributedMesh` with a custom `extras["height"]` per-vertex scalar. Save via `save_ply_attributed`. Reload, verify round-trip. Concept: **the platform supports per-vertex custom-attribute PLY snapshots**, which is the load-bearing infrastructure for soft-body PLY output.

## Tier 2 — alongside soft-body, foundation for sim-cast (10 examples)

2. **`format_conversion`** — Load STL, save PLY, load PLY, save OBJ. Verify topology preserved. Concept: **format auto-detection and round-trip integrity across STL/OBJ/PLY**.
3. **`attributed_mesh_basics`** — Build `AttributedMesh` with `compute_normals`, set colors, set zone_ids, set extras. No I/O, no operation crates — just the type itself. Concept: **the SoA attributed-mesh shape**.
4. **`mesh_repair_walkthrough`** — Construct a deliberately-broken mesh (duplicate vertices + reversed-winding face + small hole + degenerate triangle). Run `validate_mesh`, print the report. Apply `repair_mesh`, observe the changes. Re-validate. Concept: **the repair pipeline pattern from broken to clean**.
5. **`mesh_offset_outward`** — Unit cube + `offset_mesh(0.1)` (outward 0.1mm). Save both before/after as PLY. Concept: **expansion via SDF + marching cubes**.
6. **`mesh_offset_inward`** — Unit cube + `offset_mesh(-0.1)` (inward 0.1mm). Concept: **erosion / cavity-prep**, the foundational operation for mold-cavity inversion.
7. **`shell_generation_fast`** — Solid mesh → `ShellBuilder::wall_thickness(2.0).fast().build()`. Save resulting shell as PLY. Concept: **normal-based shell generation** (fast, but wall thickness varies at corners).
8. **`shell_generation_high_quality`** — Same input as #7, but `.high_quality()`. Save side-by-side comparison. Concept: **SDF-based shell generation** (slower, uniform wall thickness).
9. **`printability_fdm_validation`** — Load a deliberately-overhanging mesh, run `validate_for_printing` with `PrinterConfig::fdm_default`, print the issue list. Concept: **manufacturability constraint reporting**.
10. **`printability_orientation`** — Same input as #9, but call `find_optimal_orientation` first; re-validate. Concept: **automatic orientation reduces overhang**.
11. **`cross_section_sweep`** — Take a known body (e.g., the layered silicone device's outer shell), sweep a plane along Z, dump one PLY per slice as 2D contour polylines. Concept: **cross-section visualization** (also serves as a primitive for the soft-body deformation slice viz).

## Tier 3 — deferred (5 examples, useful but not blocking soft-body or casting)

12. **`sdf_distance_field`** — Sample `mesh-sdf::SignedDistanceField` on a 3D grid, dump as a PLY with `extras["signed_distance"]` colormap. Concept: **the numerical SDF visualized**.
13. **`lattice_gyroid_in_box`** — Generate a gyroid lattice in a bounding box. Save as PLY. Concept: **TPMS lattice generation**.
14. **`lattice_density_gradient`** — Same as #13 but with a vertical density gradient. Concept: **variable-density lattices**.
15. **`lattice_inside_shell`** — Generate a shell, generate a lattice in its bounding box, boolean-combine to keep lattice inside shell. Concept: **printable lightweight composite (shell + infill)**.
16. **`self_intersection_detection`** — Construct a self-intersecting mesh (two intersecting cubes), run `detect_self_intersections`, render the intersecting triangles in distinct color. Concept: **the hardest mesh-repair case made visible**.

## Cadence and review

Per `feedback_one_at_a_time` and `feedback_one_at_a_time_review`:

- Each example is reviewed individually before the next lands.
- Two-pass review: Claude verifies numerical anchors (the PLY round-trip succeeded, the manifold check passes, the offset distance is correct); user verifies visuals (the cross-section looks right, the shell wall is uniform, the gyroid is recognizable).
- Tier 1 lands in the same PR as the gap-fix commits (extensible attributes + custom-attribute PLY writer).
- Tier 2 examples land as additive commits in the same PR or a follow-up PR depending on bandwidth.
- Tier 3 examples land later, as their underlying use cases (lattice work for the layered silicone device middle layer; sdf-distance work for casting demoldability analysis) drive them.

## What this part commits to

- The 16-example inventory above is the **canonical mesh examples plan** for the platform.
- New examples added in the future MUST follow the same format (`<name>/{main.rs, README.md}`, museum-plaque README, two-pass reviewed).
- The PLY-output convention is the canonical persistence format for example artifacts; future Bevy viewer (in `sim-bevy`) reads PLY directories produced by examples.
- The depth pass for this part provides per-example expected-output specifications, suggested viewer-side colormap settings, and the cross-references between examples (e.g., #4's broken mesh is a good input to #11's cross-section sweep, etc.).
