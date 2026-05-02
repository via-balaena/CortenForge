# mesh v1.0 examples-coverage arc — specification

**Status**: drafting (recon + spec only; no code yet)
**Working doc**: deleted in the final commit of the arc per `feedback_code_speaks`. Durable narrative migrates to `docs/studies/mesh_architecture/src/80-examples.md` (Part 8 update) + `examples/mesh/README.md` (canonical per-example index) + per-crate `CHANGELOG.md` `[Unreleased]` blocks.

**Precedent**: structurally mirrors `mesh-printability/V08_FIX_ARC_SPEC.md` (the v0.8 fix arc spec, deleted at row #25 of that arc, recoverable from git at `0ac8c102~1`). Same recon → spec → execute → spec-delete cadence; same §-numbering convention; same triple-tracking discipline for deferred work.

---

## §1. Overview & current state

### What this arc does

Three mesh crates ship A-grade per their COMPLETION.md — `mesh-sdf`, `mesh-measure`, `mesh-lattice` — and have been A-grade since 2026-01-18. None has any `examples/` coverage. The 8 examples shipped in PR #222 (`2dac42b4`, mesh ecosystem skeleton) and the 8 examples in PR #223 (`e9cb0d6d`, mesh-printability v0.8 arc) cover `mesh-types`, `mesh-io`, `mesh-repair`, `mesh-offset`, `mesh-shell`, and `mesh-printability`. The umbrella `mesh` re-exports only — no example needed.

This arc closes the example-coverage gap so v1.0 of the mesh ecosystem ships with **complete example coverage**: every public crate has at least one example demonstrating its load-bearing capabilities, modeled on the rigid-body/mujoco precedent. After this arc, "examples completely reflect the codebase" is true for the mesh tree.

### Crate-by-crate coverage gap

| Crate | LOC | Coverage now | Gap |
|---|---|---|---|
| `mesh-types` | (covered) | `attributed-mesh-basics` | none |
| `mesh-io` | (covered) | `ply-with-custom-attributes`, `format-conversion` | none |
| `mesh-repair` | (covered) | `mesh-repair-walkthrough` | none |
| `mesh-offset` | (covered) | `mesh-offset-{outward, inward}` | none |
| `mesh-shell` | (covered) | `shell-generation-{fast, high-quality}` | none |
| `mesh-printability` | (covered) | 8 examples shipped in PR #223 | none |
| **`mesh-sdf`** | **863** | **none** | **all public surface** |
| **`mesh-measure`** | **1,658** | **none** | **all public surface** |
| **`mesh-lattice`** | **4,523** | **none** | **all public surface** |
| `mesh` (umbrella) | re-exports | implicit via downstream examples | none |

### Pre-flight findings (full audit in §2)

| ID | Finding | In-arc | Treatment |
|----|---------|--------|-----------|
| F1 | `mesh-sdf` public surface is small + clean (1 struct, 5 free fns) — single example sufficient | yes | spec'd as `mesh-sdf-distance-query` (§5.4) |
| F2 | `mesh-measure` exposes 4 result-structs + 8 free fns across 4 modules — natural fit for 3 examples | yes | spec'd as `mesh-measure-bounding-box`, `-cross-section`, `-distance-to-mesh` (§5.1–§5.3) |
| F3 | `mesh-lattice` exposes 6+ lattice types + density maps + beam-data + infill + low-level TPMS evals — propose 4 examples | yes | spec'd as `mesh-lattice-tpms-gyroid`, `-strut-cubic`, `-density-gradient`, `-shape-bounded` (§5.5–§5.8) |
| F4 | `mesh-sdf::SdfError::OutOfBounds`/`InvalidDimensions` exposed but never constructed (anticipates a never-built grid SDF API) | **NO** (defer to v0.9) | triple-track: trigger = grid SDF API consumer arrives |
| F5 | `mesh-measure::MeasureError`/`MeasureResult` exposed but no public function returns them (all return `Option<T>` or default-on-empty) | **NO** (defer to v0.9) | triple-track: trigger = a function tightens validation to fail-fast |
| F6 | `mesh-lattice::generate_infill` is gap-rich: shell is `mesh.clone()` (`infill.rs:353`); `connect_to_shell` + `solid_caps` + `shell_layers` + `connection_thickness` + `solid_cap_layers` parameters publicly exposed but never read in the main path; volumes are bbox-heuristic (`bounds_volume - interior_volume`, lines 363-371), not signed-volume integrals on the shell/lattice meshes; the 100% / 0% infill early-return paths hard-code volumes to `0.0` with a `// Would need proper volume calculation` TODO at line 307 | **YES** (in-arc gap-fix sub-arc — reasoned in §1.2 below; cadence in §6.5) | bundled with §5.9 mesh-lattice-mesh-bounded-infill example; gap-fixes a–e |
| F7 | `mesh-lattice::LatticeType::Voronoi` is "perturbed cubic" (honest pre-disclosure in `LatticeType::Voronoi` doc-comment) — not a true Voronoi tessellation | **NO** (defer to v0.9) | triple-track: trigger = a downstream caller reports cubic-grid artifacts in Voronoi output |
| F8 | `mesh-lattice::density_to_threshold` is approximate — doc-comment says "These are approximate mappings"; quantitative density anchors will be ±10% not bit-exact | **NO** (live with it) | document as a verbal anchor in §5.5; not a stub |
| F9 | `mesh-lattice::estimate_strut_volume(mesh, radius)` is a triangle-count heuristic (`tri_count / 24`); `actual_density` reported by `LatticeResult` is approximate | **NO** (live with it) | examples assert vertex/face counts (bit-exact) not density (approximate) |
| F10 | `mesh-lattice::marching_cubes` produces vertex-soup (explicit comment: "could deduplicate, but this is simpler") — TPMS-lattice output has no welding pass | **NO** (live with it; document as platform truth per `feedback_f3d_winding_callout`) | examples assert `vert/face = 3.0` soup signature (3 unique verts per face; `is_degenerate` filter gates both atomically so the ratio is bit-exact); v0.9 trigger = real consumer needs welded TPMS lattice |
| F11 | `mesh-lattice::BeamLatticeData` (3MF beam data) is publicly exposed but `mesh-io` has no 3MF beam writer | **NO** (defer to v0.9) | triple-track: trigger = 3MF beam writer demand from a printer-driver workflow |
| F12 | `mesh-sdf::SignedDistanceField::is_inside` + queries are O(F) per call (no BVH) — examples must keep mesh sizes small (sphere ≤ ~500 tris) | **NO** | document constraint in §4.6 (point-cloud sample sizes ≤ 1000) |
| F13 | The 16-example inventory in `docs/studies/mesh_architecture/src/80-examples.md` (mesh book Part 8) is partially superseded by the v0.8 printability arc (Tier 2 #9 `printability-fdm-validation` → 8 examples; Tier 3 #16 `self-intersection-detection` already covered by `printability-self-intersecting`) | yes | Part 8 rewrite slot in §9 |

### §1.2 Why `generate_infill` DOES get an in-arc gap-fix sub-arc

Per `feedback_examples_drive_gap_fixes` (saved 2026-05-01 during this spec's authoring): examples are *just as much* for completing and fixing gaps in the codebase as they are for showcasing capabilities. The mesh-printability v0.8 precedent (PR #223 — 5 detectors authored + 5 in-place fixes + 1 architecture change + 8 examples in one 41-commit PR) is the canonical pattern. The mesh-shell SDF engine-fix in PR #222 (commits 11.5.1/11.5.2) is the same pattern at a smaller scale: an example commit attempt revealed `.high_quality()` produced non-printable shells; the spec extended mid-arc with engine-fix commits; the example resumed.

Pre-deferring F6 on "examples should steer around" grounds was the wrong call (initial spec draft made it; corrected after user feedback). It would (a) ship `mesh-lattice-shape-bounded` (§5.8) as a pedagogical workaround that documents `with_shape_sdf` as if it were the canonical mesh-bounded composition path — when `generate_infill` is the canonical API, just gap-rich; (b) leave a future Claude with no recon depth to re-discover the gap; (c) violate `feedback_fix_gaps_before_continuing` ("when examples reveal engine gaps, stop and fix the engine first") + `feedback_improve_mesh_crate`.

**Verdict**: `generate_infill` gets an in-arc gap-fix sub-arc, with a dedicated 9th example (§5.9 `mesh-lattice-mesh-bounded-infill`) that exercises the API and surfaces gaps a–e. Cadence at §6.5.

| Criterion | mesh-printability v0.8 (precedent) | mesh-lattice F6 (this arc) |
|-----------|------------------------------------|---------------------------|
| Gap shape | Variants exposed but never populated | Same — shell-offset never computed; connections + caps + volumes unimplemented |
| Fix complexity | New detector per gap, ~50–200 LOC each (mostly local algorithms) | Per-gap: a (~100-150 LOC shell offset via `mesh-offset`), b (~150-300 LOC connections), c (~100-200 LOC solid caps), d (~50-80 LOC volumes), e (~50-100 LOC SDF-bounded interior). Total ~450-830 LOC. Comparable to ONE printability detector group. |
| Theme alignment | On-theme — example surfaces gap | On-theme — §5.9 example surfaces gap, gap-fix is the engine work the example needs |
| Workaround? | None — the variants ARE the API | `with_shape_sdf` covers shape-conforming for §5.8 — but NOT shell + lattice composite which is the canonical `generate_infill` use case |
| `feedback_pr_size_ci_economics` | 41 commits (5 detectors + supporting + 8 examples) | ~34 commits in single PR (~17 example commits across 9 examples + 5 gap-fix commits + 4 doc commits + workspace bump + spec deletion). Comparable cadence. |
| `feedback_no_reflexive_defer` "off-theme" definition | Same crate, same theme | Same crate, same theme |

The previous draft's "v1.0 examples can demonstrate boundary-conformance via `with_shape_sdf` (which IS implemented) instead of via `generate_infill`" framing confused two pedagogies: `with_shape_sdf` is for **TPMS/lattice trimmed by an arbitrary shape** (sphere, custom SDF); `generate_infill` is for **mesh-bounded composite of shell + lattice + solid-caps** (FDM-style infill workflow). They're complementary, not substitutes.

**Scope cap on the sub-arc**: if recon during execution shows the F6 gap-fix scope ballooning past ~7 commits (e.g., gap b — connections — turns out to need a non-trivial new geometric primitive in mesh-types), **stop and raise**. The user decides whether to expand the arc further or split off a follow-on PR. Mirrors the printability v0.8 §1 scope cap on the Gap A 38-site fallout.

**Other deferrals stay deferred** (F4, F5, F7, F10, F11 — see §10 v0.9 backlog) — those are off-theme: F4/F5 are API-shape-breaking + no current consumer need; F7 is a separate algorithm replacement (~200-400 LOC of new Bowyer-Watson code, not a stub-fix); F10 is documented-as-deliberate (welding TPMS lattice is trigger-gated to a real consumer); F11 is a separate format implementation (3MF beam writer) in a different crate (`mesh-io`).

### Out of scope (declined-with-rationale, all triple-tracked)

Each item below is **triple-tracked** at PR-close: (1) AI cross-session memory at the project memo or a new entry; (2) per-crate `CHANGELOG.md` `[Unreleased] / v0.9 candidates` block; (3) mesh book §10 (Roadmap) or per-part Known limitations subsection. Migration happens in commit 32 (the CHANGELOG migration commit) per §6.2, before spec deletion at commit 34.

- **F4 — `SdfError::OutOfBounds` / `InvalidDimensions`**: anticipates a never-built grid-based SDF API. Triple-tracked. Trigger: a downstream caller asks for `SignedDistanceField::from_grid` or similar.
- **F5 — `MeasureError` / `MeasureResult` adoption**: would tighten validation in current `Option<T>`-returning functions. Currently no caller benefits. Triple-tracked. Trigger: a power user reports surprise that `dimensions()` returns `Default` rather than an error on degenerate input.
- **F7 — Real Voronoi tessellation**: `LatticeType::Voronoi` is perturbed-cubic. Triple-tracked. Trigger: a user reports cubic-grid artifacts in Voronoi output, OR organic-feeling lattice is required for a layered-silicone-device payload.
- **F11 — 3MF beam writer in `mesh-io`**: `BeamLatticeData` exists, no writer. Triple-tracked. Trigger: 3MF output demand from a printer-driver workflow.

Deferred examples from the original Part 8 inventory:
- **`lattice-inside-shell`** (Tier 3 #15): now mapped to `mesh-lattice-mesh-bounded-infill` (§5.9 in this spec), which exercises `generate_infill` after the F6 in-arc gap-fix sub-arc lands (§6.5). Bundled, not deferred.
- **`self-intersection-detection`** (Tier 3 #16): already covered by `printability-self-intersecting` (PR #223). Mark obsolete in Part 8 rewrite (§9); do not author.

### Scope cap

Two-tier scope cap. **Default resolution is fix-in-arc + scope expansion**, NOT defer; per `feedback_examples_drive_gap_fixes` + `feedback_patience_track_record` + `feedback_baby_steps`. The "stop and raise" cadence is for user confirmation of scope expansion, not for negotiating partial-deferrals.

1. **Public-surface expansion**: If recon during execution surfaces ANY public-surface item not enumerated in this spec's §5 per-example coverage tables, **stop and raise**. The expected resolution is "add a new example in this PR with proper math-pass anchors" — scope-expansion. v0.9 deferral is reserved for genuinely off-theme surface (different crate / different API contract).

2. **In-arc gap-fix sub-arc expansion (§6.5 specifically)**: If the F6 gap-fix sub-arc reveals MORE gaps than the listed a–e (e.g., recon shows `LatticeError::NonWatertightMesh` is never returned because the watertight check is missing entirely; or gap b connections need a new `mesh-types` geometric primitive), **stop and raise**. The expected resolution is "expand the sub-arc; the new geometric primitive becomes in-arc work for v1.0." Per the user-affirmed cadence (`feedback_patience_track_record` 2026-04-29: "slow/patient/detail-driven baby-stepping is by far the most successful approach historically"), large-gap-discovery is *expected* in this arc — the spec does not pre-build partial-defer escape hatches.

### Current state on main (2026-05-01, working tree clean)

- All six C1 phases shipped: P0 (#214/215) → P1 (#217) → P2 (#218) → P3 (#219) → P4 (#220) → P5 (#221).
- Mesh PRs #222 (`2dac42b4`) + #223 (`e9cb0d6d`) shipped. 16 mesh examples in `examples/mesh/`.
- `cargo xtask grade-all --skip-coverage` green on main per the PR #223 close.
- Master-architect delegation expired per `feedback_master_architect_delegation` with PR #221 merge; standard counterweights apply (confirm before commit/push/merge/branch/tag, surface project-scope/UX decisions). `cargo doc --no-deps -p <crate>` and read-only recon are autonomous.
- Soft-body Phase 5.5 is the natural follow-on to v1.0 mesh examples (per the project_mesh_v1_examples_coverage memo's framing).

---

## §2. Pre-flight audit

### Audit scope

Read all 17 `.rs` source files across the three uncovered crates (4 in mesh-sdf, 5 in mesh-measure, 11 in mesh-lattice — counting `lib.rs`, error.rs, and 9 module files in lattice). Plus each crate's `Cargo.toml`, `COMPLETION.md`, and the umbrella `mesh/Cargo.toml`. Findings F1–F13 enumerated in §1; per-crate detail below.

### §2.1 mesh-sdf surface (863 LOC)

| Public surface | Type | Where | Notes |
|---------------|------|-------|-------|
| `SignedDistanceField` | struct | `sdf.rs` | precomputed face-normal cache; O(F) per query |
| `SignedDistanceField::new` | fn | `sdf.rs` | returns `SdfResult<Self>`; `EmptyMesh` on empty input |
| `SignedDistanceField::distance` | method | `sdf.rs` | signed (closest-face-normal sign convention) |
| `SignedDistanceField::unsigned_distance` | method | `sdf.rs` | absolute |
| `SignedDistanceField::closest_point` | method | `sdf.rs` | brute-force over faces; returns `Point3<f64>` |
| `SignedDistanceField::is_inside` | method | `sdf.rs` | ray-cast in `+X` |
| `SignedDistanceField::mesh` | method | `sdf.rs` | accessor |
| `signed_distance` | free fn | `sdf.rs` | one-shot equivalent |
| `unsigned_distance` | free fn | `sdf.rs` | one-shot equivalent |
| `closest_point_on_triangle` | free fn | `query.rs` | Ericson RTCD algorithm |
| `ray_triangle_intersect` | free fn | `query.rs` | Möller–Trumbore |
| `point_in_mesh` | free fn | `query.rs` | inside test via ray-cast |
| `point_segment_distance_squared` | free fn | `query.rs` | utility |
| `SdfError`, `SdfResult` | error types | `error.rs` | 3 variants; only `EmptyMesh` ever constructed |

**Stub-shaped findings**: F4 — `OutOfBounds` + `InvalidDimensions` variants exposed but never constructed.

**Sign-convention warning**: `compute_sign` uses face-normal of closest face. For concave geometry near edges/vertices, this can flip sign at the discontinuity. Documented constraint: example fixtures stay convex (sphere, cube). Concave-geometry SDF behavior is a known limitation, not a spec concern for v1.0 examples.

**Performance constraint**: O(F) per query; no BVH. For a ~500-tri sphere fixture and ~1000 query points, ~500k tri-tests per run — sub-second in release. Don't exceed.

### §2.2 mesh-measure surface (1,658 LOC)

| Public surface | Type | Where | Notes |
|---------------|------|-------|-------|
| `Dimensions` | struct | `dimensions.rs` | AABB + width/depth/height/diagonal/center/bounding_volume |
| `Dimensions::min_extent`, `max_extent`, `aspect_ratio`, `is_cubic`, `size` | methods | `dimensions.rs` | utility |
| `dimensions` | free fn | `dimensions.rs` | computes `Dimensions` from `IndexedMesh` |
| `OrientedBoundingBox` | struct | `obb.rs` | PCA-driven; carries 8 corners + half_extents + Rotation3 + volume |
| `OrientedBoundingBox::extents`, `axis_x`/`y`/`z`, `contains`, `surface_area` | methods | `obb.rs` | utility |
| `oriented_bounding_box` | free fn | `obb.rs` | computes OBB via PCA + symmetric eigendecomp |
| `CrossSection` | struct | `cross_section.rs` | points + perimeter + area + centroid + bounds + plane + contour_count |
| `CrossSection::is_empty`, `is_closed` | methods | `cross_section.rs` | utility |
| `cross_section` | free fn | `cross_section.rs` | single-plane slice |
| `cross_sections` | free fn | `cross_section.rs` | regular-spaced slice stack |
| `circumference_at_height`, `area_at_height` | free fns | `cross_section.rs` | Z-axis convenience helpers |
| `DistanceMeasurement` | struct | `distance.rs` | from + to + distance + dx/dy/dz |
| `DistanceMeasurement::direction`, `direction_normalized`, `midpoint` | methods | `distance.rs` | utility |
| `measure_distance` | free fn | `distance.rs` | point-to-point |
| `closest_point_on_mesh` | free fn | `distance.rs` | point-to-mesh; `Option<Point3>` |
| `distance_to_mesh` | free fn | `distance.rs` | unsigned point-to-mesh distance; `Option<f64>` |
| `MeasureError`, `MeasureResult` | error types | `error.rs` | 3 variants; **never returned by any public fn** |

**Stub-shaped findings**: F5 — `MeasureError`/`MeasureResult` are public surface but no fn returns them.

**Code duplication note (informational, not a stub)**: `mesh-measure::distance::closest_point_on_triangle` is a private duplicate of `mesh-sdf::closest_point_on_triangle`. Both crates have their own implementation. Not in scope for v1.0 — could be deduplicated in a future hygiene arc, triple-tracked candidate "consolidate Ericson RTCD into one workspace home".

**Plane-normalization in `cross_section`**: `plane_normal.normalize()` is called inside the function — caller doesn't need to pre-normalize. Document this in the example's prose so users learn the contract.

### §2.3 mesh-lattice surface (4,523 LOC)

The largest crate; widest public surface. Categorized:

**Lattice generation (top-level entry point)**:

| Surface | Type | Where | Notes |
|---------|------|-------|-------|
| `LatticeType` (Cubic, OctetTruss, Gyroid, SchwarzP, Diamond, Voronoi) | enum | `types.rs` | `#[non_exhaustive]`; 4 fns: `name`, `is_tpms`, `is_strut_based`, `recommended_resolution` |
| `LatticeResult` | struct | `types.rs` | mesh + actual_density + cell_count + total_strut_length + beam_data |
| `LatticeResult::new`, `with_strut_length`, `with_beam_data`, `vertex_count`, `triangle_count` | methods | `types.rs` | builder + accessors |
| `LatticeParams` | struct | `params.rs` | 11 fields incl. `Arc<dyn Fn(Point3) -> f64 + Send + Sync>` for `shape_sdf` |
| `LatticeParams::cubic`, `octet_truss`, `gyroid`, `schwarz_p`, `diamond`, `voronoi` | preset constructors | `params.rs` | `gyroid`/`schwarz_p`/`diamond` set `resolution = 15`; others `10` |
| `LatticeParams::with_lattice_type`, `with_cell_size`, `with_strut_thickness`, `with_wall_thickness`, `with_density`, `with_density_map`, `with_min_feature_size`, `with_resolution`, `with_trim_to_bounds`, `with_beam_export`, `with_shape_sdf` | builders | `params.rs` | `with_density` clamps to [0,1]; `with_resolution` clamps to ≥2 |
| `LatticeParams::is_outside_shape`, `density_at`, `validate` | methods | `params.rs` | `validate` returns `Result<(), LatticeError>` |
| `generate_lattice` | free fn | `generate.rs` | dispatches by `LatticeType`; returns `Result<LatticeResult, LatticeError>` |

**Density mapping (variable density)**:

| Surface | Type | Where | Notes |
|---------|------|-------|-------|
| `DensityMap` (Uniform, Gradient, Radial, SurfaceDistance, StressField, Function) | enum | `density.rs` | StressField + Function carry `Arc<dyn Fn>` |
| `DensityMap::evaluate`, `evaluate_with_distance` | methods | `density.rs` | `SurfaceDistance` only works through `evaluate_with_distance` (returns midpoint as fallback in `evaluate`) |
| `DensityMap::from_stress_field`, `from_stress_field_with_exponent`, `from_function` | constructors | `density.rs` | builder |

**Low-level TPMS evaluation (free fns)**:

| Surface | Where | Notes |
|---------|-------|-------|
| `gyroid`, `schwarz_p`, `diamond`, `neovius`, `iwp` | `tpms.rs` | bare implicit-surface evaluators; return scalar at `point` for `cell_size` period |
| `density_to_threshold` | `tpms.rs` | approximate density → threshold; `gyroid`/`diamond` use `(0.5 - density) * 3.0`, `schwarz_p` uses `* 6.0` |
| `make_shell` | `tpms.rs` | `\|f(x)\| - half_thickness` shell SDF wrapper |

**Low-level strut construction (free fns)**:

| Surface | Where | Notes |
|---------|-------|-------|
| `generate_strut`, `generate_strut_tapered` | `strut.rs` | hexagonal-prism strut (6 segs); 14 verts + 24 tris per strut |
| `combine_struts` | `strut.rs` | merge iter of meshes; preserves vertex layout |
| `estimate_strut_volume(length, r1, r2)` | `strut.rs` | analytical truncated-cone formula `(π/3) × h × (r1² + r1·r2 + r2²)` |

**Beam data (3MF export precursor)**:

| Surface | Where | Notes |
|---------|-------|-------|
| `Beam`, `BeamCap` (Sphere/Flat/Butt), `BeamSet`, `BeamLatticeData` | `beam.rs` | full 3MF Beam Lattice Extension data model |
| `Beam::new`, `tapered`, `with_caps`, `length`, `average_radius` | methods | `beam.rs` | utility |
| `BeamSet::new`, `add_beam`, `len`, `is_empty` | methods | `beam.rs` | named beam grouping |
| `BeamLatticeData::new`, `add_vertex`, `add_beam`, `add_beam_with_radius`, `add_tapered_beam`, `total_length`, `vertex_count`, `beam_count`, `estimate_volume`, `remove_short_beams` | methods | `beam.rs` | full API |

**Infill (mesh-bounded lattice composite)**:

| Surface | Where | Notes |
|---------|-------|-------|
| `InfillParams` | `infill.rs` | 8 fields (`shell_thickness`, `shell_layers`, `infill_percentage`, `connect_to_shell`, `connection_thickness`, `solid_caps`, `solid_cap_layers`, `lattice`); F6 — half are unimplemented |
| `InfillParams::for_fdm`, `for_lightweight`, `for_strong` | presets | `infill.rs` | profile constructors |
| `InfillParams::with_*` builders + `validate` | methods | `infill.rs` | builder |
| `InfillResult` | struct | `infill.rs` | combined mesh + shell + lattice + actual_density + 3 volumes |
| `InfillResult::total_volume`, `vertex_count`, `triangle_count` | methods | `infill.rs` | utility |
| `generate_infill` | free fn | `infill.rs` | F6 — gap-rich; **NOT used by any v1.0 example** |

**Marching cubes (re-exported for advanced usage)**:

| Surface | Where | Notes |
|---------|-------|-------|
| `marching_cubes_algorithm::extract_isosurface` | `marching_cubes.rs` | public submodule; vertex-soup output (F10) |

**Errors**:

| Surface | Where | Notes |
|---------|-------|-------|
| `LatticeError` (10 variants, `#[non_exhaustive]`) | `error.rs` | All variants are constructed somewhere |

**Stub-shaped findings**: F6 (`generate_infill`), F7 (Voronoi-as-perturbed-cubic), F8 (approximate `density_to_threshold`), F9 (heuristic `actual_density`), F10 (MC vertex-soup), F11 (no 3MF writer for `BeamLatticeData`).

**Workspace dep on `hashbrown`**: lattice already pulls hashbrown for vertex deduplication in cubic/voronoi paths. No new dep needed for examples.

### §2.4 Workspace-wide consumer enumeration

Each of the 3 crates is consumed by:

- **mesh-sdf**: `mesh-shell` (SDF path), `mesh-offset` (offset via SDF), `mesh` (umbrella). No examples consume it directly.
- **mesh-measure**: `mesh` (umbrella), `examples/mesh/shell-generation-high-quality` (cross-section anchor). No production code consumes it.
- **mesh-lattice**: `mesh` (umbrella). No other workspace crate consumes it. No examples.

Consequence: examples for these 3 crates are the **first downstream consumers** of each crate's public surface (mesh-shell + mesh-offset use mesh-sdf internals only). Pedagogical surface area is high.

### §2.5 A-grade baseline

Per `feedback_grading_rubric` ("A-across-the-board or fix-and-regrade before execution"):

1. Capture v0.7.0 baseline by running `cargo xtask grade mesh-sdf`, `cargo xtask grade mesh-measure`, `cargo xtask grade mesh-lattice` (all three) BEFORE the first example commit. All three are reported A-grade in their COMPLETION.md (2026-01-18); confirm still true.
2. After each example commit, run `cargo xtask grade example-mesh-<name> --skip-coverage` on the new crate (per `feedback_xtask_grade_opacity` — per-criterion checks during iteration). Full grade-all only as the final ship-readiness gate.
3. If any commit drops a criterion below A, stop and fix before proceeding. The grading bar is durable per `feedback_thorough_review_before_commit`.
4. Coverage for the three uncovered crates is **not changed by examples** (examples are separate workspace crates with their own coverage). Their coverage stays where it was on 2026-01-18.

---

## §3. API surface diff — v0.7 → v1.0

**No API changes are in scope for this arc.** v1.0 examples-coverage is purely additive at the workspace level (8 new `examples/mesh/<name>/` crates) and pure-consumer of existing public surfaces in mesh-sdf, mesh-measure, and mesh-lattice.

### v0.9-candidate API hygiene (out of scope, triple-tracked)

| Crate | API change | Reason | Trigger |
|-------|------------|--------|---------|
| mesh-sdf | Remove or wire up `SdfError::OutOfBounds` + `InvalidDimensions` | F4 — never constructed | Grid SDF API consumer arrives |
| mesh-measure | Remove `MeasureError`/`MeasureResult` OR adopt them in fn signatures | F5 — never returned | A function tightens validation to fail-fast |
| mesh-lattice | `generate_infill` real-shell-offset + connections + solid-caps + accurate volumes | F6 | Real consumer needs shell+lattice composite (not solved by `with_shape_sdf`) |
| mesh-lattice | `LatticeType::Voronoi` as Bowyer-Watson tessellation | F7 | User reports cubic-grid artifacts in Voronoi output |
| mesh-io | 3MF beam writer | F11 | 3MF beam output demand from a printer-driver workflow |

### Version bump rationale

| Crate | v0.7 → v1.0? | Reason |
|-------|--------------|--------|
| mesh-sdf | maybe | Workspace `version.workspace = true` candidates; flag for §11 |
| mesh-measure | maybe | Same |
| mesh-lattice | maybe | Same |
| mesh (umbrella) | yes | Conceptual v1.0 ship of "mesh ecosystem with examples completely reflecting the codebase" |

**Recommendation**: bump workspace `[workspace.package].version = "1.0.0"` in commit 33 of §6.2 (the version-bump commit, second-to-last in the arc), after all examples ship. v1.0 is a meaningful semver milestone — "examples completely reflect the codebase" matches the spirit of 1.0 (production-ready, public API stable). Not a hard requirement for the arc; flagged for §11 / user decision.

---

## §4. Cross-cutting policies

These policies apply to all 9 examples uniformly. They live above the per-example specs in §5 to enforce consistency and prevent drift.

### §4.1 Layout convention (mirrors `examples/mesh/README.md`)

Every example is a workspace member crate at:

```
examples/mesh/<dash-case-name>/
├── Cargo.toml      # [package].name = "example-mesh-<dash-case-name>"
├── README.md       # museum-plaque (per `feedback_museum_plaque_readmes`)
├── src/main.rs     # writes PLY artifacts to out/
└── out/            # gitignored; generated artifacts
```

Names are dash-case to match the workspace (`hello-solid`, `mesh-pipeline`, etc.). Cargo `[lints] workspace = true` block included.

### §4.2 Math-pass-first (per `feedback_math_pass_first_handauthored`)

Every example encodes its numerical anchors as `assert!`/`approx::assert_relative_eq!` calls in `main()` (or in a `verify()` / `verify_fixture_geometry()` helper called from `main()`). A clean `cargo run --release` exit-0 == clean visual inspection. **Drops the ⏸ pause-for-visuals gate by default.**

For each example, anchors include:
- **Per-vertex coordinate anchors** (when fixture is hand-authored) within `1e-12` of the analytical expected coordinates.
- **Per-face winding anchors** (when fixture is hand-authored) — cross-product unit normal direction within `1e-12` of expected.
- **Mesh bounding-box anchors** at analytical extents.
- **Function-output anchors** specific to the public surface being demonstrated (signed distance, AABB, OBB volume, cross-section area, gyroid eval at known points, lattice cell count, etc.).

For lattice examples specifically, ⏸ visual review is **recommended but not blocking** because the gyroid surface is itself the pedagogy — math-pass anchors verify correctness, but seeing the gyroid is part of the museum-plaque value. README's "Run" section invites optional `f3d --up=+Z out/lattice.ply` viewing.

### §4.3 Museum-plaque READMEs (per `feedback_museum_plaque_readmes`)

Each README has these sections (mandatory):
1. **Title + 1-paragraph concept statement** — "What this example demonstrates." Names the platform capability + the v1.0 coverage gap it closes.
2. **What it does** (2–4 sentences) — input fixture, transformation/query, output written.
3. **Numerical anchors** — bullet list, each cross-references a constant or assertion in `src/main.rs`.
4. **Visuals** (optional/recommended) — what the user looks for in the PLY (color gradient, periodic surface, density-correlated thickness, etc.); how to invoke f3d.
5. **Run** — single `cargo run -p example-mesh-<name> --release` block. Output paths.
6. **Cross-references** — sister examples, mesh book parts, v0.9 candidates the example surfaces.

`feedback_chamfered_not_rounded` applies: when describing MC tessellation of sharp creases, use "chamfered" / "polygonal" / "angular planar cuts" — never "smooth" / "filleted" / "rounded".

`feedback_f3d_winding_callout` applies: any winding-pair platform-truth artifact (e.g., MC vertex-soup output) gets a callout near the README top, not buried.

### §4.4 Fixture style — bias toward exact-representable inputs

Hand-authored fixture vertex coordinates use FP-exact values (`1.0`, `2.0`, `0.5`, integer multiples) when possible. Avoid `0.1` and similar non-exact decimals; prefer `1.0 / 10.0` only when the assertion tolerance admits it. Sphere fixtures use `f64::sqrt` (correctly-rounded per IEEE-754) for radius math; sphere tessellations use UV with `sin`/`cos` (NOT correctly-rounded — anchor with `1e-12` not bit-exact).

**Rotation-derived geometry**: when a fixture vertex needs `cos(π/4) = sin(π/4) = √2/2`, use `let s = f64::sqrt(0.5);` and apply `s` for both cos and sin coefficients. `f64::sqrt` is correctly-rounded; `f64::sin(π/4)` and `f64::cos(π/4)` are NOT (per stress round 1 finding R6: `f64::sqrt(2)/2 = 0.70710678118654757` vs `f64::sin(π/4) = 0.70710678118654746` — last-bit divergence). Using sqrt-derived values keeps the rotation matrix FP-stable across libm versions on macOS/Linux.

For the printability-showcase precedent on hand-authored fixtures: per-shell coords + per-face winding cross-product unit normal anchors hold to `1e-12` across platforms when inputs are exact-representable.

### §4.5 Dependency policy

Examples depend on:
- `mesh-types` (always; the `IndexedMesh` + `Point3<f64>` carrier).
- The crate(s) being demonstrated.
- `mesh-io` for `save_ply` (always, for the artifact dump).
- `anyhow` for `Result<()>` `main()`.
- `approx` for `assert_relative_eq!`.

Optionally, when a math-pass anchor needs a sister mesh capability:
- `mesh-repair::validate_mesh` for closed-mesh manifold/watertight assertions.
- `mesh-sdf` for SDF-based validation (e.g., the gyroid example may use SDF to verify "all points on surface have `gyroid(p)` within tolerance of 0").
- `mesh-measure` for cross-section / dimensions on a generated mesh.

**No new workspace deps introduced** by any v1.0 example. All listed crates are already workspace members.

`#![allow(clippy::cast_possible_truncation)]` and `#![allow(clippy::cast_precision_loss)]` are applied at the example-`main.rs` top-level when `usize → u32` (mesh indexing) or `usize → f64` (count printout) casts are required, with `//`-style justification per `feedback_xtask_grade_opacity`. The same per-crate `[lints] workspace = true` flip from PR #223 propagates to all new examples.

### §4.6 Performance budget

Every example completes in **< 5 seconds in release mode** on the project's reference machine. Guidelines per crate:

- **mesh-sdf examples**: O(F × Q) where F = face count, Q = query count. Sphere ~ 500 tris × 1000 query points ≈ 500k tri-tests = sub-second. Don't exceed ~1000 tris × 1000 points without re-evaluating.
- **mesh-measure cross-section**: O(F) per slice × N slices. 50 slices × 1000-tri mesh = 50k tri-tests = fast.
- **mesh-measure obb**: O(V) for centroid + covariance + symmetric eigendecomp (3×3 — fast). Linear in vertex count.
- **mesh-lattice generation**: cubic + octet-truss are O(cells^3); TPMS is O(resolution^3). 25mm box × 5mm cells = 125 cells, ~7500 verts via `combine_struts` (540 struts × 14 = 7560; not deduplicated by `combine_struts`). Gyroid at resolution 15 in 30mm × 30mm × 30mm box: total_resolution = 45 voxels along each axis = ~91k cells; emits ~10-20k MC tris, ~30-60k verts (vertex-soup signature 3.0×). Fast.

If recon during execution reveals an example exceeds the budget, raise immediately — examples are pedagogical, slow runs erode the museum-plaque experience.

### §4.7 Cross-platform FP stability

The Phase 4 multi-material work surfaced cross-platform FP drift in faer's block-diagonal LDLᵀ (cf [faer FP drift memo](../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_faer_block_diagonal_fp_drift.md)). Mesh-printability v0.8 surfaced ray-cast / voxel-fill drift mitigated via tolerance bands + asymmetric voxel jitter.

**For v1.0 examples, the FP-drift mitigations are**:
1. **Tolerance-based assertions** — use `assert_relative_eq!` with `epsilon = 1e-12` for hand-authored coords (FP-exact inputs) but `epsilon = 1e-6` to `1e-4` for derived geometric quantities (MC output, OBB eigendecomp) and lattice cell counts.
2. **Exact-representable inputs** — per §4.4.
3. **Cross-os CI matrix** — already covers all examples via the standard CI gate; no per-example CI changes.
4. **No bit-equal assertions on TPMS / MC output** — MC tessellation has FP-fragile behavior at level-set/voxel-corner coincidences; assert vert/face count ratios + bbox + topology checks (manifold/watertight where applicable), not bit-exact vertex coords.

### §4.8 mesh-shell SDF path precedent — does NOT apply

PR #222's commit 11.5.1/11.5.2 fixed an engine-level bug in `mesh-shell::generate_shell_sdf` (vertex-soup outer not welded; MC inside-out unflipped; rim collapse on open input). The MC output from `mesh-lattice::generate_tpms_lattice` has the same vertex-soup behavior (F10) — but it's documented in the source code as a deliberate simplification ("could deduplicate, but this is simpler"), NOT an unintended regression. v1.0 lattice examples treat the soup output as the load-bearing observation (assert `vertex_count == 3 × triangle_count` bit-exact; explain in README that this is intentional); they do NOT introduce a parallel weld-pass fix.

If user feedback later prefers welded TPMS-lattice output (better visual aesthetic, tighter file size), that's a v0.9 trigger captured in F10. For v1.0, vertex-soup is the platform-truth.

---

## §5. Per-example specs

9 examples across 3 crates (8 standard + §5.9 mesh-bounded-infill paired with the F6 in-arc gap-fix sub-arc per §6.5). Each subsection covers: target crate(s), public-surface coverage, fixture, math-pass numerical anchors, expected LOC, dependencies on other examples, optional ⏸ pause-for-visuals slot. PR boundaries in §6.

### §5.1 `mesh-measure-bounding-box`

**Target**: `mesh-measure` (AABB + OBB)

**Concept**: AABB and OBB on the same mesh — when do they coincide, when do they diverge? AABB-aligned vs principal-axes-aligned bounding box on a cube vs a tilted box.

**Public-surface coverage**:
- `mesh_measure::dimensions` — AABB-derived `Dimensions` struct: width, depth, height, diagonal, center, bounding_volume.
- `Dimensions::min_extent`, `max_extent`, `aspect_ratio`, `is_cubic`, `size`.
- `mesh_measure::oriented_bounding_box` — PCA-driven OBB.
- `OrientedBoundingBox::extents`, `axis_x`/`y`/`z`, `contains`, `surface_area`.
- `OrientedBoundingBox.vertices` — 8-corner array.

**Fixture**: two vertex-disjoint shapes side by side as ONE input mesh:
- **Cube A — axis-aligned 10 mm cube** at `[0, 10] × [-5, 5] × [0, 10]` (y-centered on the origin so its `y ∈ [-5, 5]` sits inside the tilted brick's `y ∈ [-7.5√2, 7.5√2]`; combined-mesh AABB then has clean `center.y = 0`. 8 verts + 12 tris, FP-exact).
- **Cube B — 20 × 10 × 10 mm tilted brick** (long axis along local +X, then rotated 45° around Z and translated so its bbox-center sits at `(25, 0, 5)`; 8 verts + 12 tris). The brick is non-cubic so PCA's eigenvalues are distinct (one large + two equal small) and OBB on cube B alone recovers the original `(20, 10, 10)` extents — for a 10×10×10 *cube*, PCA's covariance is `α·I` with all three eigenvalues equal and OBB cannot uniquely recover the rotation, so the spec's pedagogy requires the brick. **The rotation uses `let s = f64::sqrt(0.5)` for both cos and sin coefficients** — `f64::sqrt` is correctly-rounded per IEEE-754 (`f64::sin(π/4)` and `f64::cos(π/4)` are NOT; using sqrt-derived values keeps the rotation matrix FP-stable across libm versions). After rotation+translation: tilted-brick AABB extents are `15·√2` in X and Y, and `10` in Z; brick AABB occupies `[25 - 7.5√2, 25 + 7.5√2] × [-7.5√2, 7.5√2] × [0, 10]`.

The combined mesh's overall AABB: `[0, 25 + 7.5√2] × [-7.5√2, 7.5√2] × [0, 10]` ≈ `[0, 35.61] × [-10.61, 10.61] × [0, 10]`. The tilted-brick AABB has volume `(15√2)² × 10 = 4500` (2.25× the brick's actual `20·10·10 = 2000` mm³); the brick's OBB (computed on the cube B vertex slice as a sub-mesh) recovers the original 2000 volume (PCA finds the principal axis at 45° from world +X). The OBB on the *combined* mesh has principal axis at `atan(3/8)/2 ≈ 10.28°` from world +X (PCA compromise between cube A's spread and the brick's elongation) and recovers a more modest 5% volume reduction — `7169.48` vs AABB `7553.30`.

The two shapes are vertex-disjoint, so the union's manifold is preserved per the printability-showcase precedent.

**Math-pass anchors**:
- 16 hand-authored vertices within `1e-12` of `expected_hand_vertices()` (rotation matrix uses `s = f64::sqrt(0.5)`; the 1-ULP `sin(π/4) ≠ sqrt(2)/2` divergence is sidestepped).
- 24 face-winding cross-product unit-normal anchors within `1e-12`.
- `dimensions(mesh)`:
  - `width = 25 + 7.5√2 ≈ 35.61`, `depth = 15√2 ≈ 21.21`, `height = 10.0` within `1e-12`.
  - `diagonal = sqrt(width² + depth² + 10²) ≈ 42.64` within `1e-12`.
  - `center = ((25 + 7.5√2)/2, 0, 5) ≈ (17.80, 0, 5)`.
  - `bounding_volume = width × depth × 10 = 3750·√2 + 2250 ≈ 7553.30` within `1e-12`.
  - `min_extent = 10`, `max_extent ≈ 35.61`, `aspect_ratio ≈ 3.56`, `is_cubic(0.01) == false`.
  - `size()` returns `Vector3(width, depth, height)` — verified component-wise to `1e-12`.
- `oriented_bounding_box(mesh)` (combined mesh):
  - `volume` within 5% of empirical `7169.48` (PCA-derived; covariance is non-diagonal due to brick elongation).
  - `axis_x.{x, y, z} ≈ (0.984, 0.178, 0)` within `1e-9` (principal axis ≈ `10.28°` from world +X).
  - `axis_x ⊥ axis_y ⊥ axis_z` within `1e-9` (rotation orthogonality).
  - `surface_area > 0`.
  - All 16 input vertices satisfy `|local_offset| ≤ half_extents + 1e-9` (tolerance-aware containment — strict `obb.contains(v)` can fail by 1 ULP for the 4 brick vertices that defined the OBB extremes; v0.9 candidate gap, see §10).
  - **No "8 corners within inflated AABB" anchor** — that anchor only holds when `OBB.rotation == identity` (OBB ⊆ AABB). For a non-degenerate OBB at any non-trivial tilt, OBB corners extend OUTSIDE the AABB envelope (here by ~3.5 mm in y). The contains-input-verts anchor is the correct enclosure test. The "OBB ⊆ AABB" intuition is captured as v0.9 candidate gap in §10.
- `oriented_bounding_box` on cube B sub-mesh (the brick alone — vertices `[8..16]` extracted as a no-faces sub-mesh):
  - `volume = 2000` within `1e-9` (recovers brick's analytical `BRICK_LONG · BRICK_SHORT · BRICK_SHORT`).
  - Sorted extents `(20, 10, 10)` within `1e-9`.
  - `center = (25, 0, 5)` within `1e-9`.
  - `surface_area = 2·(20·10 + 20·10 + 10·10) = 1000` mm² within `1e-9`.
  - All 8 brick vertices satisfy tolerance-aware containment.

**LOC estimate**: ~645 LOC actual (160 fixture, 350 anchors, 100 print/main, 35 const docs explaining empirical OBB constants). Up from the original 280 estimate because the spec now drives three computations (AABB on combined, OBB on combined, OBB on brick sub-mesh) plus structured verifier helpers and the P1 brick pivot that the cold-read on §6.2 #4 surfaced. **Sets the new per-example LOC ceiling for v1.0** (§7 stop-gate item 4 raised from 600 → 700; see §8 Round 4 follow-up).

**⏸ visual review**: not required (math-pass-first); optional `f3d out/mesh.ply` to see the cube + tilted-brick fixture.

**Dependencies on other examples**: none.

### §5.2 `mesh-measure-cross-section`

**Target**: `mesh-measure` (cross-section)

**Concept**: planar slicing of a mesh into 2D contours; perimeter and area extraction; slice-stack along an axis.

**Public-surface coverage**:
- `mesh_measure::cross_section` — single-plane slice.
- `mesh_measure::cross_sections` — regular-spaced slice stack.
- `mesh_measure::circumference_at_height`, `area_at_height` — Z-axis convenience helpers.
- `CrossSection` fields: `points`, `perimeter`, `area`, `centroid`, `bounds`, `plane_origin`, `plane_normal`, `contour_count`.
- `CrossSection::is_empty`, `is_closed`.
- Demonstrates that `plane_normal` is normalized internally (caller need not pre-normalize).

**Fixture**: closed cylinder with hand-authored 32-vertex top/bottom rings + 2 cap centers = **66 verts**; **128 tris** total (32 top-cap fan-tris + 32 bottom-cap fan-tris + 32 wall quads × 2 = 64 wall tris). 32-vert ring uses `f64::sqrt` for unit-circle, accurate to `1e-12`. Height 10 mm, radius 5 mm, axis-aligned along Z.

**Math-pass anchors**:
- 64 ring vertices within `1e-12` of `(5 × cos(2π·i/32), 5 × sin(2π·i/32), z)` (sin/cos: `1e-12` tolerance, not bit-exact).
- 128 face winding anchors (cross-product unit normal cosine similarity > 0.99 with outward direction).
- Single cross-section at `z = 5.0` (mid-cylinder):
  - `area` within `1e-10` of the **32-vertex polygon shoelace area** = `400 × sin(π/16) ≈ 78.0357`. (NOT analytical π·25 ≈ 78.5398 — the 32-segment polygon underestimates by ~0.64% per chord-shrinkage; `cross_section` returns the shoelace area of the polygon, which is FP-stable to `1e-10` for FP-exact ring vertices.)
  - `perimeter` within 0.5% of analytical `2π × 5 ≈ 31.4159...` (32-segment chord-shrinkage on perimeter is ~0.16%, well under 0.5%).
  - `centroid` within `1e-6` of `(0, 0, 5)`.
  - `contour_count == 1`.
  - `is_closed() == true` — note: `is_closed()` is implemented as `contour_count > 0` per `cross_section.rs:73-77`, NOT geometric closure (last point = first point). Document the semantics in the README.
  - `plane_normal` normalized (caller passes `(0, 0, 2)` un-normalized → output has `||plane_normal|| = 1` within `1e-12`).
- Slice stack via `cross_sections(start=(0,0,0.5), normal=(0,0,1), count=10, spacing=1.0)`:
  - 10 slices returned.
  - All 10 have area within 0.5% of analytical (cylinder is z-uniform).
- Convenience helpers:
  - `circumference_at_height(mesh, 5.0)` matches the single-slice perimeter to `1e-12`.
  - `area_at_height(mesh, 5.0)` matches the single-slice area to `1e-12`.
- Out-of-mesh slice at `z = 100.0`:
  - `is_empty() == true`, `contour_count == 0`, `area == 0.0`.

**Output**: `out/cylinder.ply` (the input mesh); optionally one `out/slice_<i>.ply` per slice in the slice-stack visualization (vertex-only PLYs of the 32-vertex contour points). Default is just the cylinder + numerical printout; user can opt-in to slice PLYs via a `--write-slices` arg or a const flag (`WRITE_SLICE_STACK_PLY: bool = false`).

**LOC estimate**: ~340 LOC.

**⏸ visual review**: not required; optional viewing of the cylinder + (if written) slice contour stack.

**Dependencies on other examples**: none.

### §5.3 `mesh-measure-distance-to-mesh`

**Target**: `mesh-measure` (distance)

**Concept**: point-to-mesh distance + closest-point-on-mesh + symmetric Hausdorff distance composed from the public surface.

**Public-surface coverage**:
- `mesh_measure::measure_distance` — point-to-point Euclidean.
- `mesh_measure::closest_point_on_mesh` — point→mesh closest point.
- `mesh_measure::distance_to_mesh` — point→mesh unsigned distance.
- `DistanceMeasurement` fields: `from`, `to`, `distance`, `dx`, `dy`, `dz`.
- `DistanceMeasurement::direction`, `direction_normalized`, `midpoint`.
- **Composition**: symmetric Hausdorff distance between two meshes by iterating verts of A → `distance_to_mesh(B)`, max → `d_AB`; same for B→A → `d_BA`; Hausdorff = `max(d_AB, d_BA)`. Demonstrates that `mesh-measure` exposes the primitives; `mesh↔mesh Hausdorff` is achievable as composition (not built-in).

**Fixture**: two unit cubes constructed as **two SEPARATE `IndexedMesh` instances** (`cube_a` at `[0, 1]³`, `cube_b` at `[2, 3]³`; closest-face distance = 1.0 along `+X`). The combined mesh `out/two_cubes.ply` is for visualization only; `distance_to_mesh` anchors target the individual cubes; Hausdorff iterates verts of cube_a vs cube_b. (Note: `DistanceMeasurement::dx/dy/dz` are **absolute** values per `distance.rs:97-99` — `measure_distance(from=(0,0,0), to=(-3,-4,0))` returns `dx=3.0`, not `-3.0`. Surface this convention in the README.)

**Math-pass anchors**:
- 16 hand-authored vertices within `1e-12` of expected (8 per cube, two separate meshes).
- 24 face-winding anchors (12 per cube).
- Point-to-point:
  - `measure_distance((0,0,0), (3,4,0))` → `distance == 5.0` (Pythagorean), `dx == 3.0`, `dy == 4.0`, `dz == 0.0` within `1e-12`.
  - `direction_normalized()` returns `Some((0.6, 0.8, 0.0))` within `1e-12`.
  - `midpoint()` returns `(1.5, 2.0, 0.0)` within `1e-12`.
- Point-to-mesh on `cube_a` (anchors target cube_a alone, not the combined mesh):
  - 6 query points, one per face direction (e.g., `(2, 0.5, 0.5)` for `+X` face): `distance_to_mesh(&cube_a, p) = 1.0` within `1e-12`.
  - 8 query points, one per cube vertex direction (e.g., `(2, 2, 2)` for `+X+Y+Z` corner): `distance_to_mesh(&cube_a, p) = sqrt(3)` within `1e-12`.
  - 1 query point at center `(0.5, 0.5, 0.5)`: `distance_to_mesh(&cube_a, p) = 0.5` (closest face is at distance 0.5; whether interior or exterior is irrelevant for unsigned).
  - `closest_point_on_mesh(&cube_a, p)` returns the expected face point within `1e-12`.
  - Empty-mesh edge case: `distance_to_mesh(&empty, p) == None`; `closest_point_on_mesh(&empty, p) == None`.
- Hausdorff between cube_a and cube_b (composed via `mesh-measure` primitives, not built-in):
  - **Symmetric Hausdorff**: max over all 8 verts of cube_a of `distance_to_mesh(&cube_b, v)` AND max over verts of cube_b of `distance_to_mesh(&cube_a, v)`. The farthest vertex of cube_a from cube_b is `(0,0,0)` (the corner of A diagonally-opposite from B); the closest point on cube_b from `(0,0,0)` is the (clamped) corner `(2,2,2)` — distance `sqrt(4+4+4) = sqrt(12) = 2√3 ≈ 3.464`. Symmetrically, cube_b's farthest vertex `(3,3,3)` clamps to cube_a's `(1,1,1)` at the same distance `sqrt(12)`. Hausdorff = `sqrt(12)` within `1e-12`.
  - Hausdorff is asymmetric in general; this fixture is designed so both A→B and B→A produce the same value.

  *(R1 stress-round corrected this — original draft incorrectly stated `sqrt(8) ≈ 2.828`. The error: cube B's nearest face from `(0,0,0)` is `x=2`, but the closest point on the face is the (clamped) corner `(2,2,2)` since `(0,0)` is outside the y,z `∈ [2,3]` face bounds — not the face interior at `(2,0,0)`.)*

**Output**: `out/two_cubes.ply` (the input mesh); numerical printout only.

**LOC estimate**: ~310 LOC (40+ anchor cases drive line count up).

**⏸ visual review**: not required; optional viewing of the two-cube fixture.

**Dependencies on other examples**: none.

### §5.4 `mesh-sdf-distance-query`

**Target**: `mesh-sdf` (full public surface)

**Concept**: numerical SDF on a closed mesh; signed distance + closest-point + inside/outside classification; bulk query via `SignedDistanceField` cache vs one-shot `signed_distance` free fn.

**Public-surface coverage**:
- `SignedDistanceField::new` (with empty-mesh error path).
- `SignedDistanceField::distance` (signed).
- `SignedDistanceField::unsigned_distance`.
- `SignedDistanceField::closest_point`.
- `SignedDistanceField::is_inside`.
- `SignedDistanceField::mesh` accessor.
- `signed_distance` free fn (one-shot).
- `unsigned_distance` free fn (one-shot).
- `closest_point_on_triangle` free fn (verified directly).
- `ray_triangle_intersect` free fn (verified directly).
- `point_in_mesh` free fn.
- `point_segment_distance_squared` free fn.
- `SdfError::EmptyMesh` (the only constructed variant).

**Fixture**: octahedron — 6 vertices + 8 triangles. Vertices at `(±r, 0, 0)`, `(0, ±r, 0)`, `(0, 0, ±r)` for `r = 1.0` — all FP-exact.

**Why octahedron not sphere**: octahedron has analytically-known SDF for any query point (faces are 4 triangles with normal `(±1, ±1, ±1)/√3`; signed distance to the surface is `(|x| + |y| + |z|) / √3 - r/√3` for points with `(±x, ±y, ±z)` in the principal octant — rotationally symmetric over 8 octants). UV-tessellated sphere would have `1/√3` chord-error at face centers. Octahedron lets us assert analytical SDF values to `1e-12`.

**Math-pass anchors**:
- 6 hand-authored vertices within `1e-12`.
- 8 face winding anchors (cross-product unit normal direction matches the expected octant outward).
- `SignedDistanceField::new(empty_mesh)` returns `Err(SdfError::EmptyMesh)`.
- `SignedDistanceField::new(octahedron)` returns `Ok(_)`.
- 14 query points (1 generic-interior, 6 face-center directions, 6 vertex-direction off-mesh, 1 far-exterior). **Origin (0,0,0) is NOT used as an inside-test query**: the +X ray from origin hits vertex (1,0,0) shared by 4 faces; Möller-Trumbore returns `Some(t=1)` for all 4, count=4 → even → `is_inside` reports false (degenerate corner-hit). Use an off-axis interior point instead.
  - **Generic interior** `(0.05, 0.07, 0.11)` (off any axis-of-symmetry; +X ray hits the `+x+y+z` face interior at one point): `is_inside == true`, `distance < 0`, `unsigned_distance` analytically `(1 - 0.05 - 0.07 - 0.11) / √3 = 0.77/√3 ≈ 0.4446` within `1e-12`.
  - **Face-center direction** `(0.3, 0.3, 0.3)` (interior, off-vertex): closest point on `+x+y+z` face; `distance = (0.3+0.3+0.3)/√3 - 1/√3 = (0.9-1)/√3 = -0.1/√3 ≈ -0.0577` (negative, inside). `is_inside == true` (+X ray from this point hits `+x+y-z` face plane outside the triangle, hits only `+x+y+z` face interior — 1 intersection → odd → inside).
  - **Outside on +X axis** `(2, 0, 0)`: closest point on the `+x+y+z` / `+x+y-z` / `+x-y+z` / `+x-y-z` faces all converge to the `+X` vertex `(1, 0, 0)`; `distance == 1.0` (Euclidean), `is_inside == false`.
  - **Far outside** `(10, 10, 10)`: `is_inside == false`, `distance > 0`, large.
  - **Documented limitation** (R5 + new): inside-test on the origin (and any point exactly on a symmetry axis whose +X ray hits a shared vertex) reports false due to multi-face counting at degenerate corners. README calls this out as a platform truth, not a fixture bug. v0.9 candidate: switch to winding-number-based inside-test (item #8 of v0.9 backlog).
- `closest_point_on_triangle` direct: 3 sub-cases (interior region, edge region, vertex region) hand-authored against a known triangle.
- `ray_triangle_intersect` direct: 3 sub-cases (hit, miss, parallel).
- `point_in_mesh((0.05, 0.07, 0.11), octahedron) == true` (off-axis interior, avoids the origin's 4-face vertex-coincidence); `point_in_mesh((10,10,10), octahedron) == false`.
- `point_segment_distance_squared` direct: 2 sub-cases (perpendicular drop, beyond-endpoint).
- One-shot equivalents (`signed_distance`/`unsigned_distance` free fns) match `SignedDistanceField` cached values to `1e-12` on the same query points.
- **Bulk query**: 1000-point cubic grid in `[-2, 2]³` (10 × 10 × 10) cached as `Vec<(Point3<f64>, f64)>`. Stats printed: percent inside (analytically `(8/3) × r³ / box_volume = (8/3 × 1) / 64 ≈ 4.17%`), max distance (`distance to (-2,-2,-2)` ≈ `(2+2+2)/√3 - 1/√3 ≈ 5/√3 ≈ 2.89`).

**Output**: `out/octahedron.ply` (input mesh) + `out/sdf_grid.ply` (1000-vertex grid PLY with `extras["signed_distance"]` per-vertex scalar via `save_ply_attributed`). User can colormap by signed distance externally.

**LOC estimate**: ~480 LOC (octahedron geometry + 14 query-point analytical anchors + grid generation).

**⏸ visual review**: optional; the colored grid is the recommended visualization.

**Dependencies on other examples**: builds on `ply-with-custom-attributes` for the `extras["signed_distance"]` PLY pattern.

### §5.5 `mesh-lattice-tpms-gyroid`

**Target**: `mesh-lattice` (TPMS path)

**Concept**: TPMS surface generation via `LatticeParams::gyroid` + `generate_lattice`. Demonstrates the gyroid implicit surface, periodicity, density-to-threshold mapping, and the marching-cubes vertex-soup output.

**Public-surface coverage**:
- `LatticeParams::gyroid` preset constructor.
- `LatticeParams::with_density`, `with_resolution`, `with_wall_thickness`, `with_cell_size`.
- `LatticeParams::validate` (success path).
- `generate_lattice(params, bounds)` returning `Ok(LatticeResult)`.
- `LatticeResult.mesh`, `actual_density`, `cell_count`, `vertex_count`, `triangle_count`.
- `LatticeType::Gyroid::is_tpms() == true`.
- `LatticeType::Gyroid::recommended_resolution() == 15`.
- `mesh_lattice::gyroid` (free TPMS evaluator) — direct evaluation at known points to verify implicit-function correctness.
- `mesh_lattice::density_to_threshold` — for `density = 0.5` returns `0.0` exactly; for `density = 0.3` returns `0.6` (gyroid type).
- `mesh_lattice::make_shell` — wraps the TPMS into a shell SDF at given wall thickness.
- `marching_cubes_algorithm::extract_isosurface` indirectly via `generate_lattice`.

**Fixture**: 30 mm × 30 mm × 30 mm cubic bounding box; cell size 10 mm; resolution 15; density 0.5; wall thickness 1.5 mm.

**Math-pass anchors**:
- TPMS evaluation at known points (verifies `gyroid` free fn, BEFORE generation):
  - `gyroid((0, 0, 0), 10.0) == 0.0` (origin: sin(0)·cos(0)·3 = 0; bit-exact).
  - `gyroid((10, 10, 10), 10.0) == gyroid((0, 0, 0), 10.0) within 1e-10` (periodicity over one cell).
  - `gyroid((2.5, 0, 0), 10.0) == sin(π/2)·cos(0) + sin(0)·cos(0) + sin(0)·cos(π/2) = 1.0` within `1e-12`.
- `density_to_threshold(0.5, "gyroid") == 0.0` bit-exact.
- `density_to_threshold(0.3, "gyroid") == 0.6` within `1e-12`.
- `make_shell(|p| gyroid(p, 10.0), 1.0)((0,0,0)) == 0 - 0.5 = -0.5` (origin is on the gyroid surface, half_thickness inside the shell).
- After `generate_lattice`:
  - `result.cell_count == 27` (3 × 3 × 3 cells in 30mm³ at 10mm cell size).
  - `result.vertex_count > 0` and `result.triangle_count > 0`.
  - **Vertex-soup signature**: `vertex_count == 3 × triangle_count` **bit-exact**. Reason: MC source (`marching_cubes.rs:462-468`) gates both `vertices.push(...)` calls and the `faces.push([...])` inside the same `if !is_degenerate(...)` block — degenerate-tri filtering removes both atomically, so the ratio is invariant. Anchor: exact equality.
  - `result.actual_density` in approximate range `[0.3, 0.7]` for density 0.5 input (heuristic estimate, F9; not tight).
  - **Bounds containment**: all output vertices satisfy `bounds.min.x ≤ v.x ≤ bounds.max.x` (and Y, Z) within `1e-12`. MC interpolates vertices on cell edges between corner positions; the highest-index cell's far corner equals `bounds.max` exactly, so verts are within the closed bbox (no cushion needed).
- **Surface verification via SDF-on-output** (anchor that gyroid surface is what the lattice approximates): for each vertex `v` in `result.mesh.vertices`, `gyroid(v, 10.0).abs() < threshold + voxel_size + epsilon` where `threshold = 0` (density 0.5) and `voxel_size = cell_size / resolution = 10/15 ≈ 0.667`. Anchor tolerance: `1.0` (one voxel + cushion).

**Output**: `out/gyroid_lattice.ply` (the generated lattice mesh); the gyroid is the visual centerpiece.

**LOC estimate**: ~370 LOC.

**⏸ visual review**: **recommended** (gyroid surface is itself the pedagogy). The README has a callout: "even though math-pass anchors verify correctness, opening `out/gyroid_lattice.ply` in f3d shows the iconic 'twisted-saddle' surface of the gyroid TPMS — viewing is part of the museum-plaque experience."

**Dependencies on other examples**: none, but the README cross-references `mesh-offset-{outward,inward}` (which use marching cubes with welded output via mesh-shell engine fixes) to contrast the un-welded lattice MC output (F10).

### §5.6 `mesh-lattice-strut-cubic`

**Target**: `mesh-lattice` (strut-based lattice + beam-data export)

**Concept**: cubic strut lattice generation + the 3MF beam-data export precursor.

**Public-surface coverage**:
- `LatticeParams::cubic` preset constructor.
- `LatticeParams::with_strut_thickness`, `with_density`, `with_beam_export`, `with_trim_to_bounds`.
- `generate_lattice` (strut-based path).
- `LatticeResult.beam_data` populated when `with_beam_export(true)`.
- `BeamLatticeData::vertices`, `beams`, `vertex_count`, `beam_count`, `total_length`, `estimate_volume`.
- `Beam::v1`, `v2`, `r1`, `r2`, `length`, `average_radius`, `cap1`, `cap2`.
- `BeamCap::Sphere` (default).
- `mesh_lattice::generate_strut`, `generate_strut_tapered`, `combine_struts`, `estimate_strut_volume` (direct verification).
- `LatticeType::Cubic::is_strut_based() == true`.
- `LatticeType::Cubic::recommended_resolution() == 10`.

**Fixture**: 25 mm × 25 mm × 25 mm cubic bounding box; cell size 5 mm; strut thickness 1.0 mm; density 1.0 (uniform full-density grid); beam export ON.

**Math-pass anchors**:
- Direct strut tests (BEFORE generate_lattice):
  - `generate_strut((0,0,0), (0,0,10), 0.5)` returns mesh with `vertex_count == 14` (2 cap centers + 6 × 2 ring) and `face_count == 24` (6 sides × 2 + 12 caps).
  - `generate_strut_tapered` same vert/face count for tapered.
  - `generate_strut(p, p, 0.5) == None` (degenerate zero-length).
  - `combine_struts` of N struts = `N × 14` verts, `N × 24` faces.
  - `estimate_strut_volume(10, 1, 1) == π × 10 ≈ 31.416` within `1e-10` (cylinder formula).
  - `estimate_strut_volume(10, 1, 0) == π/3 × 10 ≈ 10.472` within `1e-10` (cone formula).
- After `generate_lattice`:
  - `result.cell_count == 125` (5 × 5 × 5 cells).
  - `result.total_strut_length == Some(2700.0)` within `1e-9` (540 struts × 5.0 mm; bit-exact for integer-spaced grid since each `f64::add` of integer multiples of 5.0 is exact below 2^53).
  - `result.beam_data == Some(_)` (preserve_beam_data on).
- Beam-data assertions (independent path from `result.total_strut_length`):
  - `data.vertex_count() == 6 × 6 × 6 == 216` (cubic grid (cells+1)³ = 6³ = 216 nodes; deduplicated via the `quantize` HashMap pattern at quantization scale `1e6`).
  - `data.beam_count()` = number of edges = `3 × 5 × 6 × 6 == 540` (3 axes × 5 in-axis × 6 × 6 = 540 unique edges in a 5×5×5 cubic lattice).
  - `data.total_length()` exactly `540 × 5.0 = 2700.0` mm within `1e-9` (every edge is exactly cell_size = 5.0 mm).
  - All beams have `r1 == r2 == strut_thickness / 2 == 0.5` within `1e-12` (uniform density 1.0; `density.sqrt() == 1.0`).
  - All beams have `cap1 == cap2 == BeamCap::Sphere` (default).
  - `data.estimate_volume()` within reasonable range (cylinders at 0.5mm radius × 5mm length × 540 beams ≈ 2120 mm³).
  - Per-beam length sanity: every beam in `data.beams` reports `Some(5.0)` for `beam.length(&data.vertices)` within `1e-9`.
- Mesh result counts are bit-exact: `result.vertex_count() == 540 × 14 == 7560`; `result.triangle_count() == 540 × 24 == 12960`. The `combine_struts` helper concatenates per-strut meshes without vertex dedup (each grid node appears in up to 6 struts → ~6× duplication of the 216 unique grid nodes; deduplicated separately into `BeamLatticeData.vertices`). No degenerate struts at integer-spaced grid points.

**Output**: `out/cubic_lattice.ply` (the triangulated mesh); numerical printout includes the beam-data summary.

**LOC estimate**: ~420 LOC.

**⏸ visual review**: optional but recommended; cubic strut grid is visually clear.

**Dependencies on other examples**: none.

### §5.7 `mesh-lattice-density-gradient`

**Target**: `mesh-lattice` (variable density via `DensityMap`)

**Concept**: variable density lattice — gradient from low density at bottom to high density at top.

**Public-surface coverage**:
- `LatticeParams::octet_truss` preset (richer geometry than cubic — better visual for density gradient).
- `LatticeParams::with_density_map`, `with_beam_export(true)` (turned on for per-beam-radius anchors; bottom struts have smaller `r1` than top struts — the load-bearing observation that demonstrates density modulation).
- `DensityMap::Uniform`, `Gradient`, `Radial`, `Function` (4 variants demonstrated; `SurfaceDistance` and `StressField` deferred).
- `DensityMap::evaluate` at known sample points.
- `DensityMap::evaluate_with_distance` for `SurfaceDistance` variant (called explicitly to demonstrate the special-method contract).
- `DensityMap::from_function`, `from_stress_field` constructors.
- `LatticeParams::density_at(point)` accessor (uses density_map when present).
- `generate_lattice` (octet-truss + density-modulated path).
- `LatticeResult.beam_data.beams[i].r1` per-beam radius (target of HE-4 anchor; bottom-of-bbox beams report smaller radii than top-of-bbox beams).

**Fixture**: 30 mm × 30 mm × 30 mm bounding box; cell size 7.5 mm (4 cells per axis); strut thickness 0.6 mm; `with_beam_export(true)`. `Gradient` density map: `from_density = 0.1` at `(0,0,0)`, `to_density = 0.5` at `(0,0,30)`.

**Math-pass anchors**:
- Direct `DensityMap` evaluation (before generation):
  - `DensityMap::Uniform(0.4).evaluate((10, 20, 30)) == 0.4` (ignores point) within `1e-12`.
  - `Gradient` at endpoints: `evaluate((0,0,0)) == 0.1`, `evaluate((0,0,30)) == 0.5` within `1e-12`.
  - `Gradient` at midpoint: `evaluate((0,0,15)) == 0.3` within `1e-12`.
  - `Gradient` clamped beyond endpoint: `evaluate((0,0,-50)) == 0.1`, `evaluate((0,0,150)) == 0.5`.
  - `Radial` at center: at `inner_radius == 0`, `evaluate(center) == inner_density`.
  - `Function::evaluate` clamps to [0, 1]: `from_function(|_| 1.5).evaluate(origin) == 1.0`.
  - `SurfaceDistance::evaluate(p)` returns `(surface_density + core_density) / 2` (midpoint fallback per `density.rs:203-212`).
  - `SurfaceDistance::evaluate_with_distance(0.0) == surface_density`; `evaluate_with_distance(transition_depth) == core_density`.
- After `generate_lattice`:
  - `result.cell_count == 64` (4³).
  - `result.beam_data == Some(_)` (preserve_beam_data ON for this example).
  - **Per-beam radius density anchor (load-bearing observation)**: every cell emits 20 struts × 14 verts/strut = 280 verts regardless of density (geometric tessellation is fixed; density does NOT change vertex count). The density modulation manifests as `r1 = strut_thickness/2 × density.sqrt()` per beam. Anchor: filter `data.beams` by `vertices[v1].z` — bottom-half beams (z ≤ 15) have mean `r1` ≈ `0.3 × sqrt(density at z≈7.5) ≈ 0.3 × sqrt(0.20) ≈ 0.134`; top-half beams (z > 15) have mean `r1` ≈ `0.3 × sqrt(0.40) ≈ 0.190`. Top-half mean `r1` > bottom-half mean `r1` by a factor of `sqrt(0.4/0.2) ≈ 1.41` ± 5%.
  - Spot-check anchor: at least one beam with `vertices[v1].z < 7.5` has `r1 < 0.16`; at least one beam with `vertices[v1].z > 22.5` has `r1 > 0.18`.
- `params.density_at((0,0,0)) == 0.1`; `params.density_at((0,0,30)) == 0.5`.
- Direct demo of `DensityMap::Function`: build `DensityMap::from_function(|p| 0.5 + 0.1 * (p.x / 10.0).sin())`, query at known points, assert.

**Output**: `out/density_gradient_lattice.ply`.

**LOC estimate**: ~380 LOC.

**⏸ visual review**: optional but recommended; the density gradient is visually striking (thin bottom struts → thick top struts).

**Dependencies on other examples**: none.

### §5.8 `mesh-lattice-shape-bounded`

**Target**: `mesh-lattice` (boundary-conforming via `with_shape_sdf`)

**Concept**: lattice clipped to an arbitrary analytical shape via SDF. The pedagogy: TPMS or strut lattice trimmed by a closed-form SDF (sphere, custom function) — useful for design intent where the boundary is mathematical, not a mesh. Complementary to §5.9 `mesh-lattice-mesh-bounded-infill`, which exercises the mesh-bounded composite path via `generate_infill`.

**Public-surface coverage**:
- `LatticeParams::gyroid` (TPMS path, where shape-conforming is most striking visually).
- `LatticeParams::with_shape_sdf` — accepts `Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>`.
- `LatticeParams::is_outside_shape` accessor.
- The `shell_sdf` composition pattern internally: `tpms_value.max(sdf(p))` (SDF intersection).
- `generate_lattice` (TPMS path with shape SDF).
- The `Arc<dyn Fn>` convention for the shape SDF (per the existing `tests/test_gyroid_lattice_with_shape_sdf` precedent).

**Fixture**: 30 mm × 30 mm × 30 mm bounding box **centered at origin** (`min = (-15, -15, -15)`, `max = (15, 15, 15)`, mirroring the in-tree `test_gyroid_lattice_with_shape_sdf` precedent at `generate.rs:695-725`); cell size 10 mm (so bbox is integer multiple of cell — voxel size = `cell_size / resolution` exactly); resolution 15. Shape SDF: sphere of radius 12 mm centered at origin (`p.coords.norm() - 12.0`).

**Math-pass anchors**:
- Direct `shape_sdf` evaluation (before generation):
  - `is_outside_shape((0,0,0)) == false` (interior).
  - `is_outside_shape((20,0,0)) == true` (exterior).
  - `is_outside_shape((11,0,0)) == false` (interior, < 12).
  - `is_outside_shape((13,0,0)) == true` (exterior, > 12).
  - With NO shape_sdf set (default), `is_outside_shape(any_point) == false`.
- Comparison generation (for the "with vs without" anchor, mirroring existing `test_gyroid_lattice_with_shape_sdf`):
  - `result_with_sdf.vertex_count() < result_without_sdf.vertex_count()` (SDF trims output).
  - The trimmed lattice has all vertices within `sphere_radius + voxel_size + cushion` of origin. With bbox 30³ and cell_size 10, `total_resolution = ceil((30/10) × 15) = 45`, so MC voxel size = `30/45 = 2/3 ≈ 0.667 mm` (= `cell_size/resolution = 10/15` since bbox is integer multiple). Every `v` satisfies `(v - origin).norm() < 12 + 0.667 + 1.0 == 13.67`. Anchor tolerance reflects MC voxel size.
- The `result_with_sdf.cell_count` reports the TOTAL bbox cells (not the trimmed cells) — F9-related; document this in the README.
- Edge case: a tiny sphere (radius 1.0) → very few output verts (or empty); demonstrates SDF trim is real.

**Output**: `out/sphere_gyroid.ply` (the trimmed lattice); optionally `out/sphere_gyroid_full.ply` (the un-trimmed comparison) — written behind a `WRITE_COMPARISON: bool = true` constant for reviewer clarity.

**LOC estimate**: ~340 LOC.

**⏸ visual review**: **recommended** (sphere-bounded gyroid is visually iconic; the comparison with vs without SDF tells the boundary-conforming story).

**Dependencies on other examples**: builds conceptually on `mesh-lattice-tpms-gyroid` (must land first; the README cross-references it).

### §5.9 `mesh-lattice-mesh-bounded-infill`

**Target**: `mesh-lattice::generate_infill` — exercises the canonical mesh-bounded composite API and surfaces F6 gaps a–e for in-arc fix per §1.2 + §6.5.

**Concept**: FDM-style infill workflow on a watertight input mesh — outer shell generated by inward offset, lattice generated in the interior, optional connections from lattice to inner shell, optional solid caps at top/bottom. The pedagogically-canonical "shell + lattice" composite for 3D printing. Complementary to §5.8's analytical-SDF-bounded path.

**Public-surface coverage**:
- `mesh_lattice::InfillParams` — the 8-field struct.
- `InfillParams::for_fdm`, `for_lightweight`, `for_strong` preset constructors.
- `InfillParams::with_lattice_type`, `with_cell_size`, `with_shell_thickness`, `with_infill_percentage`, `with_solid_caps`, `with_solid_cap_layers`.
- `InfillParams::validate` (success + error paths).
- `generate_infill(mesh, params) -> Result<InfillResult, LatticeError>`.
- `InfillResult` fields: `mesh`, `shell`, `lattice`, `actual_density`, `shell_volume`, `lattice_volume`, `interior_volume`.
- `InfillResult::total_volume`, `vertex_count`, `triangle_count`.
- Edge cases: `LatticeError::EmptyMesh` (empty input), `LatticeError::InteriorTooSmall` (mesh smaller than `2 × shell_thickness + 1 × cell_size`).

**Fixture**: 50 mm × 50 mm × 50 mm hollow cube target — hand-authored 8-vertex / 12-triangle outer cube fed to `generate_infill`. The fixture is the simplest watertight mesh that exercises all the params; FDM-style preset (`InfillParams::for_fdm` with `cell_size = 10.0`, `infill_percentage = 0.2`).

**Math-pass anchors** (post-gap-fix):
- 8 hand-authored vertices within `1e-12` of `[0, 50]³`.
- 12 face winding anchors (cross-product unit normal cosine similarity > 0.9999 with outward direction).
- After `generate_infill`:
  - `result.shell` is **a real hollow shell**, NOT `mesh.clone()`. Anchor: `result.shell.vertex_count() != input_mesh.vertex_count()` (a properly-offset shell has different vertex count from the input cube). **This anchor only passes after F6 gap a is fixed.**
  - `result.shell_volume == signed_volume_integral(&result.shell)` within 1% (post-gap-d). Pre-gap-d, v0.7 returns `bounds_volume - interior_volume = 125000 - (50 - 2*3.7)³ ≈ 47734` — a bbox-heuristic that does NOT match the actual shell mesh's signed-volume integral. The anchor's intent: prove gap-d replaced the heuristic with the integral. (`shell_volume > 0` alone is satisfied pre-fix; insufficient as a gap-d witness.)
  - `result.lattice_volume == signed_volume_integral(&result.lattice)` within 1% (post-gap-d). Same intent.
  - `result.interior_volume > 0` (always — bbox-derived).
  - `result.actual_density` within ±10% of `params.infill_percentage = 0.2`.
  - `result.mesh.vertex_count() > result.shell.vertex_count() + result.lattice.vertex_count()` if connections were generated, else equality (post-gap b fix).
  - When `solid_caps = true` and `solid_cap_layers = 4`, the resulting top + bottom regions are SOLID (no lattice). Anchor: count lattice verts with `z > 50 - 4×layer_height` should be ZERO. **This anchor only passes after F6 gap c is fixed.**
- `validate` paths:
  - `InfillParams::for_fdm().validate()` returns `Ok(())`.
  - **Negative-validate via direct field assignment** (the builder `with_shell_thickness` clamps to `max(0.0)` per `infill.rs:161-164`, so `with_shell_thickness(-1.0)` cannot reach `validate`'s negative-check path):
    ```rust
    let mut params = InfillParams::for_fdm();
    params.shell_thickness = -1.0;
    assert!(matches!(params.validate(), Err(LatticeError::InvalidShellThickness(t)) if t == -1.0));
    ```
    README explains: the builder is callable-side-defensive (clamp on input); `validate()` is the inspection-side check. Both paths exist; the negative-validate anchor demonstrates the latter via direct construction.
- `generate_infill` error paths:
  - `generate_infill(empty_mesh, params)` returns `Err(LatticeError::EmptyMesh)`.
  - `generate_infill(tiny_5mm_cube, params_with_cell_size_4)` returns `Err(LatticeError::InteriorTooSmall)`.
- Comparison anchor (with vs without solid caps): two runs, one with `solid_caps = true` and one with `false`; the `solid_caps = true` mesh should have MORE triangles (the planar slab geometry adds tris) and fewer top/bottom lattice verts. **Post-gap c fix.**

**Output**:
- `out/input.ply` — the 50mm cube (8 verts, 12 tris).
- `out/shell.ply` — the offset hollow shell (post-gap-a; pre-gap-a this is identical to input.ply).
- `out/lattice.ply` — the interior lattice (excludes shell).
- `out/composite.ply` — the combined mesh (shell + lattice + caps).
- Numerical printout includes the 3 volume reports.

**LOC estimate**: ~480 LOC (largest example; exercises every `InfillParams` field with an anchor or scenario).

**⏸ visual review**: **recommended** (the shell + lattice composite is visually distinctive; opening `composite.ply` in f3d shows the FDM-style cutaway aesthetic).

**Dependencies on other examples**: lands AFTER `mesh-lattice-strut-cubic` (§5.6) and `mesh-lattice-tpms-gyroid` (§5.5) — the lattice-generation primitives it composes are demonstrated separately first. Lands AFTER the F6 gap-fix sub-arc commits (§6.5).

**Why this example exists** (for the README): without it, v1.0 ships demonstrating *every* mesh-lattice public surface EXCEPT `generate_infill` — the canonical FDM-infill workflow API. The §5.8 `mesh-lattice-shape-bounded` covers analytical-SDF-trimmed lattices; `generate_infill` covers mesh-bounded shell + lattice composites. Both paths matter; both ship in v1.0.

---

## §6. PR boundaries + commit ordering

### §6.1 PR boundary

**One PR** (user directive 2026-05-02 close-read session): `feature/mesh-v1-examples-coverage` covers the entire arc — 9 examples + F6 gap-fix sub-arc + book/release work in a single squash-merge.

| PR | Branch | Examples | Estimated commits | Rationale |
|----|--------|----------|-------------------|-----------|
| Single | `feature/mesh-v1-examples-coverage` | All 9: `mesh-measure-{bounding-box, cross-section, distance-to-mesh}`, `mesh-sdf-distance-query`, `mesh-lattice-{tpms-gyroid, strut-cubic, density-gradient, shape-bounded, mesh-bounded-infill}` + F6 gap-fix sub-arc (5 fixes a–e) | ~34-40 | Mirrors mesh-printability v0.8 precedent (41 commits in one PR). One CI cycle (~25 min) instead of two = clear win; squash-merge with one pre-squash tag; review burden heavier but precedent-supported. |

If recon during execution surfaces a smaller in-arc gap-fix beyond the F6 sub-arc (e.g., one missing builder, one off-by-one), it slots in. If it surfaces an additional LARGER gap-fix beyond F6 a–e, **stop and raise** per §1 two-tier scope cap.

### §6.2 Commit ordering — single PR (~36 commits)

| # | Commit | Type | Theme |
|---|--------|------|-------|
| 1 | `docs(mesh): MESH_V1_EXAMPLES_SCOPE.md spec lock + sim-core review plan` | docs | spec post-cold-read-edits + `docs/decisions/SIM_CORE_REVIEW_PLAN_2026-05.md` (cross-track planning artifact, carried on this branch) |
| 2 | `chore(examples-mesh): bbox skeleton — Cargo.toml + main.rs stub` | scaffolding | example-mesh-mesh-measure-bounding-box crate skeleton |
| 3 | `feat(examples-mesh): mesh-measure-bounding-box hand-authored fixture` | impl | two-cube fixture + per-vertex coord anchors + per-face winding anchors |
| 4 | `feat(examples-mesh): mesh-measure-bounding-box AABB + OBB anchors` | impl | `dimensions()` + `oriented_bounding_box()` numerical anchors; README |
| 5 | `chore(examples-mesh): cross-section skeleton` | scaffolding | example-mesh-mesh-measure-cross-section crate skeleton |
| 6 | `feat(examples-mesh): mesh-measure-cross-section cylinder fixture + slice anchors` | impl | hand-authored 32-segment cylinder + single-slice + slice-stack + helpers |
| 7 | `chore(examples-mesh): distance-to-mesh skeleton` | scaffolding | example-mesh-mesh-measure-distance-to-mesh crate skeleton |
| 8 | `feat(examples-mesh): mesh-measure-distance-to-mesh anchors + Hausdorff composition` | impl | two separate cubes (cube_a, cube_b) + point-mesh anchors + Hausdorff demonstration |
| 9 | `chore(examples-mesh): sdf-distance-query skeleton` | scaffolding | example-mesh-mesh-sdf-distance-query crate skeleton |
| 10 | `feat(examples-mesh): mesh-sdf-distance-query octahedron + analytical anchors` | impl | octahedron fixture + 14 query-point analytical anchors (off-axis interior point) + free-fn direct anchors |
| 11 | `feat(examples-mesh): mesh-sdf-distance-query bulk grid PLY` | impl | 1000-point grid + `save_ply_attributed` with `extras["signed_distance"]` |
| 12 | `chore(examples-mesh): tpms-gyroid skeleton` | scaffolding | example-mesh-mesh-lattice-tpms-gyroid crate skeleton |
| 13 | `feat(examples-mesh): mesh-lattice-tpms-gyroid free-fn TPMS anchors` | impl | direct `gyroid()` + `density_to_threshold` + `make_shell` anchors (BEFORE generate_lattice) |
| 14 | `feat(examples-mesh): mesh-lattice-tpms-gyroid generate_lattice + soup signature` | impl | `generate_lattice` + cell_count + `vert_count == 3 × tri_count` bit-exact soup anchor + per-vertex SDF verification |
| 15 | `chore(examples-mesh): strut-cubic skeleton` | scaffolding | example-mesh-mesh-lattice-strut-cubic crate skeleton |
| 16 | `feat(examples-mesh): mesh-lattice-strut-cubic direct strut anchors` | impl | `generate_strut` + `combine_struts` + `estimate_strut_volume` anchors (BEFORE generate_lattice) |
| 17 | `feat(examples-mesh): mesh-lattice-strut-cubic generate_lattice + beam_data` | impl | cubic lattice + `BeamLatticeData` cell_count, beam_count = 540, total_length = 2700, both result.total_strut_length and data.total_length() anchors |
| 18 | `chore(examples-mesh): density-gradient skeleton` | scaffolding | example-mesh-mesh-lattice-density-gradient crate skeleton |
| 19 | `feat(examples-mesh): mesh-lattice-density-gradient DensityMap variants` | impl | direct `DensityMap` evaluation anchors for Uniform/Gradient/Radial/SurfaceDistance/Function (BEFORE generate_lattice) |
| 20 | `feat(examples-mesh): mesh-lattice-density-gradient octet-truss + per-beam radius modulation` | impl | octet-truss generation with `with_beam_export(true)` + density_at + per-beam r1 modulation anchor (top vs bottom mean radius) |
| 21 | `chore(examples-mesh): shape-bounded skeleton` | scaffolding | example-mesh-mesh-lattice-shape-bounded crate skeleton |
| 22 | `feat(examples-mesh): mesh-lattice-shape-bounded SDF + with-vs-without comparison` | impl | sphere SDF + `is_outside_shape` anchors + with-vs-without trimmed-vert-count anchor (bbox -15..15, integer-cell voxel size) |
| **— F6 gap-fix sub-arc (per §6.5) starts here —** | | | |
| 23 | `chore(examples-mesh): mesh-bounded-infill skeleton + pre-fix anchor capture` | scaffolding | example-mesh-mesh-lattice-mesh-bounded-infill crate skeleton; captures v0.7 `generate_infill` baseline (`shell == mesh.clone()`, bbox-heuristic volumes) — anchors in `expected_pre_fix.rs` (deleted at #29) |
| 24 | `fix(mesh-lattice): F6 gap a — real shell offset via mesh-offset` | gap-fix | replace `let shell = mesh.clone();` (`infill.rs:353`) with `mesh_offset::offset_mesh(mesh, -shell_thickness)`; ~100-150 LOC |
| 25 | `fix(mesh-lattice): F6 gap d — signed-volume integrals on shell + lattice` | gap-fix | replace bbox-heuristic at `infill.rs:363-371` with `estimate_mesh_volume(&result.shell)` etc.; promote helper from generate.rs to `pub(crate)` `volume.rs`; early-return paths stay 0.0; ~50-80 LOC |
| 26 | `fix(mesh-lattice): F6 gap e — SDF-bounded interior via with_shape_sdf` | gap-fix | replace AABB interior bounds (lines 337-338) with internal `with_shape_sdf`-style mesh-SDF intersection; `mesh-sdf` becomes a workspace dep on `mesh-lattice`; ~50-100 LOC |
| 27 | `fix(mesh-lattice): F6 gap c — solid caps at top/bottom` | gap-fix | implement planar-slab geometry near top/bottom for `solid_cap_layers`; ~100-200 LOC |
| 28 | `fix(mesh-lattice): F6 gap b — lattice-to-shell connections` | gap-fix | implement `connect_to_shell` bridging struts; ~150-300 LOC. **Highest-LOC fix**; raises §1 two-tier scope cap if balloons past 300 LOC |
| 29 | `feat(examples-mesh): mesh-lattice-mesh-bounded-infill post-fix anchors + composite output` | impl | shell.vertex_count != input.vertex_count; volumes via signed-volume integral; solid caps cover top/bottom; connections present; comparison anchors |
| **— F6 sub-arc closes —** | | | |
| 30 | `docs(examples-mesh): README update for v1.0 coverage` | docs | `examples/mesh/README.md` update (8 new examples added; Tier 1/2/3 framing → category-based) |
| 31 | `docs(mesh-book): Part 8 v1.0 final inventory + Roadmap update` | docs | full Part 8 rewrite + Part 10 (Roadmap) v0.9 candidates list |
| 32 | `docs(mesh-{sdf,measure,lattice}): CHANGELOG [Unreleased] v0.9 candidates + v0.8.0 F6 entry` | docs | v0.9 candidates: F4, F5, F7, F10, F11; v0.8.0 entry: F6 a-e detail |
| (32.5) | _(reserved for in-arc fixes per `feedback_thorough_review_before_commit` cold re-read)_ | hotfix | optional |
| 33 | `chore(workspace): bump version 0.7.0 → 1.0.0 (mesh ecosystem v1.0)` | chore | per §3 version-bump rationale; `[workspace.package].version = "1.0.0"`. **Confirm with user before commit** per §7 R8 |
| 34 | `chore(mesh): delete MESH_V1_EXAMPLES_SCOPE.md (per feedback_code_speaks)` | chore | spec deletion; durable narrative migrated |

**⏸ visual-review slots**: math-pass-first applies; ⏸ optional after each example commit (4, 6, 8, 10, 14, 17, 20, 22, 29). Per-example numerical anchor verification = automatic gate; user opt-in to f3d viewing. Lattice examples (14, 17, 20, 22, 29) recommended for ⏸ since gyroid surface / strut grid / density gradient / sphere-bounded gyroid / shell+lattice composite is itself the pedagogy.

**Per-fix gating** (per `feedback_baby_steps` + `feedback_xtask_grade_opacity`): after each gap-fix commit (24–28), run `cargo xtask grade mesh-lattice --skip-coverage`. If A drops, fix-and-regrade BEFORE proceeding to the next fix. Per-criterion clippy/test checks are fast iteration; full grade is the ship-readiness gate.

### §6.3 Pre-squash audit-trail tag

Per `feedback_pre_squash_tag`, one annotated tag before squash-merge:

- `feature/mesh-v1-examples-coverage-pre-squash` at branch head (~34-36 commits preserved for `git log --reverse` audit).

Per `feedback_post_squash_merge_diff_verify`, after squash-merge: `git diff <squash-commit> <feature-branch-head>` MUST be empty before deleting the feature branch.

### §6.4 mesh-printability v0.8 precedent — what changes for v1.0

| Aspect | v0.8 fix arc | v1.0 examples arc |
|--------|--------------|-------------------|
| New types added | 4 new region types + `DetectorSkipped` | none |
| New detectors / new APIs | 5 detectors | none new; F6 a–e fixes existing `generate_infill` (5 internal fixes; pub surface unchanged) |
| In-place fixes | 5 (Gap B, D, E, F, M) | 5 (F6 a–e in §6.5) |
| Hygiene commits | 2 (Gap A, K) | 0 in-arc; new examples auto-inherit `[lints] workspace = true` |
| Architecture changes | 1 (Gap L: build_up_direction) | 0 |
| Total commits | 41 | ~34 (single PR) |
| Total LOC delta | ~2300 LOC of new src + ~3200 LOC of new examples | ~3400 LOC of new examples + ~450-830 LOC of F6 fixes |
| Mesh book updates | Part 5 depth-pass + §50 v0.9 candidates | Part 8 inventory rewrite + Part 10 Roadmap update |
| API surface change | additive (semver-significant) | additive (`generate_infill` semantics shift from broken to working — semver-significant for callers asserting on pre-fix volumes/shell) |
| Engine fixes mid-arc | yes (5 detectors authored alongside 8 examples) | yes (F6 a–e fixes alongside 5 lattice examples) |
| PR count | 1 | 1 (per user directive 2026-05-02) |

### §6.5 F6 gap-fix sub-arc cadence

Mirrors mesh-printability v0.8's commit-segmentation pattern. The 5 fixes are NOT a single commit; each gets its own commit per `feedback_baby_steps`. Ordering is dependency-driven:

| Commit # | Gap | Description | LOC | Depends on |
|----------|-----|-------------|-----|------------|
| §6.2 #24 | **a** | **Real shell offset** — replace `let shell = mesh.clone();` (`infill.rs:353`) with `mesh_offset::offset_mesh(mesh, -shell_thickness)` to compute an inner offset; combine the original mesh + offset as a hollow shell. `mesh-offset` already a workspace dep (no Cargo.toml addition). Doc-test on `generate_infill` updated to reflect actual offset shell. | ~100-150 | none — independent first |
| §6.2 #25 | **d** | **Signed-volume integrals** — replace bbox-heuristic at `infill.rs:363-371` (`bounds_volume - interior_volume`, `interior_volume * actual_density`) with `estimate_mesh_volume(&result.shell)` and `estimate_mesh_volume(&result.lattice)`. Promote `estimate_mesh_volume` from `generate.rs` (private) to `pub(crate)` in a new `volume.rs` module. Early-return paths (100% / 0% infill) stay at `0.0` (no shell mesh to integrate over). | ~50-80 | gap a (shell exists to integrate over) |
| §6.2 #26 | **e** | **SDF-bounded interior** — current `infill.rs` uses AABB interior bounds (`interior_min` / `interior_max` lines 337-338); for non-box meshes, lattice extends through faces. Fix: pass `mesh` as an internal `shape_sdf` to `generate_lattice` so the lattice gets trimmed to the mesh interior. `mesh-sdf` becomes a workspace dep on `mesh-lattice` (one-line Cargo.toml addition). | ~50-100 | gap a (offset shell defines the interior boundary) |
| §6.2 #27 | **c** | **Solid caps at top/bottom** — when `params.solid_caps == true`, generate planar-slab geometry near `bounds.max.z - solid_cap_layers × layer_height` and `bounds.min.z + solid_cap_layers × layer_height`. Replace lattice in those regions. `layer_height` is currently a printer concept — derive from `params.lattice.cell_size / params.lattice.resolution` per a documented heuristic, OR add an explicit `params.layer_height: f64` field (additive API change). | ~100-200 | gap e (interior bounded so caps don't extend through mesh faces) |
| §6.2 #28 | **b** | **Lattice-to-shell connections** — when `params.connect_to_shell == true`, for each lattice node within `params.connection_thickness` of the inner shell surface, generate a bridging strut. Geometry-heavy. **Highest scope-creep risk** — if BVH is needed for connection-distance lookup, raises §1 two-tier scope cap. | ~150-300 | gap a (shell exists), gap e (interior is mesh-bounded) |

**Why this ordering**: gap a is the foundation (real shell). Gap d is volumes — straightforward signed-volume integral once shells exist. Gap e is the SDF-bounded interior — precondition for caps and connections (both should NOT extend through mesh faces). Gap c is caps — additive geometry on top of the bounded interior. Gap b is connections — most complex, last; if scope balloons, this is where to stop and raise.

**Gap-fix testing**: each fix gets its own unit tests in the existing `infill.rs::tests` module. Existing tests must continue to pass — most assert on `result.is_ok()` and `result.vertex_count() > 0`, both preserved by all 5 fixes. Pre-fix anchors in §5.9's `expected_pre_fix.rs` (commit 23) capture the v0.7 behavior so the §6.2 #29 post-fix anchor can prove the gap-fix landed.

**Decline / triage during the sub-arc**: if recon during gap b reveals connections need a new geometric primitive in `mesh-types` (e.g., a polyline-strip type), that exceeds the 200-LOC off-theme bar of `feedback_no_reflexive_defer`. **Stop and raise**. User decides: defer gap b only (keep a + c + d + e in-arc; mark gap b as a v0.9 candidate with explicit trigger), expand the sub-arc, or accept simpler-but-less-elegant connection geometry.

**Order of attempts within each fix**: per `feedback_thorough_review_before_commit`, do TWO re-read passes per gap-fix commit; first pass for math/algorithm errors, second pass for code-vs-comment consistency. The printability v0.8 precedent: this pattern caught 3-5 small drifts per detector commit.

---

## §7. Pre-flight risk table

Following the §8.1 master-architect cadence — risk-mitigation review per `feedback_risk_mitigation_review`. Each high-tier risk has a stop/proceed decision. Master-architect delegation expired with PR #221; per `feedback_autonomous_delegation`, raise high-stakes calls.

| Risk | Tier | Probability | Impact | Mitigation | Stop/raise gate |
|------|------|-------------|--------|------------|-----------------|
| **R1** — `mesh-lattice` MC vertex-soup ratio is bit-exact `3.0` per the §4.8 source-code analysis; risk is that the assertion is wrong about which crate-internal invariant is tested | low | very low (verified by reading `marching_cubes.rs:462-468`) | low | assert `vertex_count == 3 × triangle_count` exactly; if cross-platform run shows a deviation, the bug is in the per-platform `is_degenerate` predicate — surface as v0.9 trigger | None |
| **R2** — PCA eigendecomposition in `oriented_bounding_box` is FP-fragile on the two-cube fixture (16 verts, two clusters) | medium | medium (eigenvalue separation may swap principal axes near degenerate fixtures) | medium (extents reordering breaks anchor) | use 5% tolerance on volume; assert ordering of extent magnitudes (longest, mid, shortest) NOT axis identity | If eigenvalue separation < 5%, raise — fixture geometry needs adjustment |
| **R3** — `density_to_threshold` is approximate (F8); density anchors in `mesh-lattice-tpms-gyroid` could drift | low | low (anchored at `density = 0.5 → threshold = 0.0` which IS bit-exact) | low | use `density = 0.5` to land on bit-exact threshold = 0; document approximation in README | None |
| **R4** — `LatticeParams::shape_sdf` uses `Arc<dyn Fn>` which is not `Debug`-friendly; example printout looks ugly | low | medium | low (cosmetic) | use `LatticeParams::Debug` impl which prints `<shape_sdf>` placeholder; example separately prints SDF as a name string | None |
| **R5** — `mesh-sdf-distance-query` octahedron sign-convention edge case: face-normal-of-closest-face fails near edges/vertices for concave geometry | low | low (octahedron is convex; inside-test passes) | low | use convex fixture only; document constraint in README | None (constraint stays; documented limitation) |
| **R6** — `mesh-measure-cross-section` UV-cylinder face winding cross-product cosine-similarity anchor is FP-sensitive at chord midpoints | low | low (anchors pass at `> 0.99` not bit-exact) | low | use 32-segment ring; cosine similarity > 0.99 with FP-sin/cos | None |
| **R7** — `cargo doc --no-deps` rustdoc warnings on bracket notation in example doc-comments per `feedback_cargo_doc_pre_commit` | medium | medium (we WILL hit `face[N]` patterns; v0.8 arc had 3 such per commit-12) | medium (CI fails post-PR-open) | run `RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p example-mesh-<name>` locally before each example commit | If warnings persist after the per-commit fix, raise |
| **R8** — Workspace version bump (`0.7.0 → 1.0.0`) breaks downstream consumers via `version.workspace` | medium | medium (every workspace crate inherits) | medium (compile errors elsewhere) | confirm before the bump commit; run `cargo build --workspace` after | **Confirm with user before bumping** — version bump is project-scope decision per master-architect-expired counterweights |
| **R9** — F6 gap-fix sub-arc scope-creep: gap b (lattice-to-shell connections) needs new geometric primitive in `mesh-types` (e.g., polyline-strip), or BVH-shaped acceleration for connection-distance lookup | medium-high | medium | medium-high (PR extends by 5-15 additional commits if a new mesh-types primitive becomes in-arc) | per §6.5, gap b is last in ordering — by the time it lands, gaps a/c/d/e are stable. If scope balloons, raise per §1 two-tier cap; expected resolution is **scope-expansion in-arc** (the new primitive joins the v1.0 work), NOT partial-defer | **Stop and raise** at the §1 cap — patient gap-discovery is the cadence, not deadline-driven scope-shrinkage |
| **R10** — Octahedron normal direction mismatch: the `+x+y+z` octant face needs CCW-when-viewed-from-outside; getting handedness wrong means SDF sign flips | medium | medium (hand-authoring 8 face windings is error-prone) | high (every SDF anchor fails) | use the unit_tetrahedron-from-mesh-sdf-tests pattern (CCW from outside = `[v0, v2, v1]` not `[v0, v1, v2]`); add per-face winding cross-product anchor BEFORE the SDF anchors in commit 10 per §6.2 | If anchors don't pass on commit 10, raise — the bug is in either the fixture or the spec; a single 30-min session fixes one or the other |
| **R11** — Math-pass anchor formula derivation errors (the printability-showcase precedent: 8 formula errors caught in 2 re-read passes) | medium | high (hand-derived analytical formulas for SDF-on-octahedron and Hausdorff are non-trivial) | medium | apply `feedback_thorough_review_before_commit` — TWO re-read passes before each math-pass commit; first pass for math errors, second for code-vs-comment consistency | If second re-read pass surfaces a math error, do a THIRD pass — this is the precedent's lesson |
| **R12** — `examples/mesh/README.md` v1.0 update misses an example or has stale Tier 3 (the README is current source-of-truth + appears in mesh book) | low | medium | low (visible misalignment if missed) | book Part 8 rewrite + README update happen in the same PR; cross-reference each example name letter-for-letter | None |
| **R13** — `cargo xtask grade-all --skip-coverage` on the new examples — the grader requires `[lints] workspace = true` block + per-criterion bar | medium | medium (every new crate has it from boilerplate, but grader is strict) | medium (CI red post-PR-open) | run `cargo xtask grade example-mesh-<name> --skip-coverage` after each example skeleton commit; per `feedback_xtask_grade_opacity`, use per-criterion checks during iteration | If grade falls below A, fix-and-regrade BEFORE proceeding to the next example (not at end of arc) |
| **R14** — Beam-data anchor `beam_count == 540` for cubic 5×5×5 may be wrong if `quantize` HashMap collapse is non-deterministic | low | low (`quantize` uses bit-exact `1e6` scale; deterministic per-platform) | medium (anchor wrong by even 1 fails CI) | derive analytically: 5×5×5 cubic lattice has `3 × cells × (cells+1) × (cells+1) = 3 × 5 × 6 × 6 = 540` edges; verify against actual lattice on first run; if mismatched, audit which edges are skipped (boundary trim) | If actual ≠ 540 by > 1, revisit derivation |

**Stop/raise summary**: 4 high-stakes calls require explicit user confirmation before proceeding —
1. Workspace version bump 0.7.0 → 1.0.0 (R8).
2. Any scope-expansion beyond this spec — public-surface OR new mesh-types primitive needed by F6 sub-arc gap b (per §1 two-tier cap). **Expected outcome is in-arc fix + scope expansion**, not defer.
3. Any deviation from the §6 commit ordering by > 2 commits.
4. Any per-example LOC > 700 (current ceiling: bounding-box at ~645 post-P1-pivot). Suggests a missing decomposition. *(Ceiling raised 2026-05-02 from 600 → 700 after the §5.1 brick-pivot scope expansion: the spec author underestimated the structured-verifier + drift-documentation overhead. The bounding-box example legitimately runs three independent computations and exercises both the recovery case and the multi-cluster compromise case; decomposition into helper modules saves no LOC and harms in-line readability.)*

(Previous draft listed a "partial-defer for F6 gap b" as an acceptable outcome — REMOVED 2026-05-01 per user correction. We accept that gap-discovery may grow scope; we do NOT pre-build escape hatches around that growth.)

---

## §8. Reconciliation table — 6 stress rounds

Per the `project_gameplan.md` "Gates per phase" convention. 6 rounds documented; each surfaces conflicts that revise the spec or get absorbed into per-§ decisions. Stress rounds happen DURING this spec authoring (i.e., now, before user review) — not after-the-fact.

| Round | Lens | Finding | Resolution |
|-------|------|---------|------------|
| **1** | Numerical-anchor verification — every analytical formula in §5 evaluated independently in Python at 1e-15 ULP scale | **Three findings**: (a) **MATH ERROR caught in §5.3 Hausdorff** — original draft said `sqrt(8) ≈ 2.828`, correct value is `sqrt(12) ≈ 3.464`; the closest point on cube B `[2,3]³` from `(0,0,0)` is the (clamped) corner `(2,2,2)` not the face-interior `(2,0,0)`. (b) `density_to_threshold(0.3, 'gyroid') = 0.6 + 1 ULP` (FP normal); §5.5 anchor at `1e-12` is correct. (c) `f64::sin(π/4)` and `f64::sqrt(0.5)` differ in the last bit (R6 below); §5.1 needed clarification — "rotation uses `f64::sqrt(0.5)` not `f64::sin(π/4)`." | (a) §5.3 Hausdorff anchor rewritten with parenthetical noting the original error. (b) Anchor unchanged. (c) §5.1 fixture description updated to specify `s = f64::sqrt(0.5)` for the rotation matrix coefficients. |
| **2** | Public-surface coverage — every public item from §2 walked against §5 per-example coverage tables | Found 3 items missing: `DistanceMeasurement::midpoint` was missing from §5.3; `LatticeType::recommended_resolution` was missing from §5.5; `BeamCap::Flat` and `BeamCap::Butt` are NOT demonstrated by §5.6 (only default `BeamCap::Sphere` is). | (a) Added `midpoint` to §5.3 anchor list. (b) Added `recommended_resolution()` to §5.5 anchor list. (c) Acknowledged in §5.6 that `BeamCap::Flat`/`Butt` are NOT demonstrated; §10 v0.9 candidate #8 added "Demo non-default BeamCap variants" as trigger-gated low-priority. |
| **3** | Dep cycles + tier integrity | New examples depend on `mesh-types`, `mesh-io`, `mesh-sdf`, `mesh-measure`, `mesh-lattice`, optionally `mesh-repair`, `anyhow`, `approx`. All workspace members already. Examples are tier `L0-integration`. Layer Integrity check passes — no sim-* / cf-* / Bevy imports. Beam-data anchor `beam_count == 540` for cubic 5×5×5 verified analytically: `3 × 5 × 6 × 6 = 540` (R2). | Confirmed; no revision. |
| **4** | LOC estimate sanity — sum of §5 estimates against precedent | Initial sum across 9 examples: 280 + 340 + 310 + 480 + 370 + 420 + 380 + 340 + 480 = **3400 LOC** (§5.9 mesh-bounded-infill added per Round 7). Mesh-printability v0.8's 8 examples totaled ~3200 LOC. Per-example range originally 280–480 LOC, all under the (then) 600 ceiling. **Post-P1-pivot follow-up (2026-05-02)**: §5.1 actual ~645 LOC after the cube→brick fixture pivot + 3 computations; ceiling raised to 700 (§7 item 4). Revised sum estimate: ~3600 LOC across 9 examples. | Per-example range now 280–645 LOC. Other 8 examples remain at original estimates pending in-flight cold-read passes. |
| **5** | Reviewer readability — README pacing + per-example single-sitting flow | Walk §4.3 README templates per example. §5.4 sdf-distance-query has densest content (14 analytical anchors + free-fn direct anchors); README will be ~120-150 lines. §5.6 strut-cubic has a beam-data printout that could overwhelm `main()` — extract `print_beam_summary(beam_data)` helper. **Octahedron face-winding verified analytically (R3)**: `+x+y+z` face with vertex order `[(1,0,0), (0,1,0), (0,0,1)]` produces cross-product `(1,1,1)` outward — matches the §5.4 spec's "CCW from outside" convention. | Refactor decision baked into §5.6 spec: `print_beam_summary(beam_data)` helper. §5.4 octahedron face winding is correctly specified. |
| **6** | Downstream consumer — v0.9 backlog absorbs all surfaced findings | Walk each F# from §1: F4 → mesh-sdf CHANGELOG; F5 → mesh-measure CHANGELOG; F6, F7, F10, F11 → mesh-lattice CHANGELOG + book §70 Known limitations. Mesh book Part 10 (Roadmap) gets consolidated v0.9 candidates list. **R6 finding**: `f64::sqrt(2)/2` vs `f64::sin(π/4)` differ at 1 ULP — adds informational note to §4.4 (fixture style "bias toward exact-representable inputs"). | §10 backlog complete; commit 32 per §6.2 (CHANGELOG migration) + commit 31 per §6.2 (book Part 10 update) deliver the triple-tracking. §4.4 not modified (already covered by "use `f64::sqrt(0.5)` rather than `sin(π/4)` when geometry is rotation-derived" — added as part of round 1 §5.1 fix). |
| **7** | **Philosophy correction round** (post-stress-rounds) — user feedback 2026-05-01 mid-spec-authoring: "examples are just as much for completing/fixing gaps as showcasing." Initial draft pre-deferred F6 (`generate_infill`) on `feedback_no_reflexive_defer` >200-LOC-off-theme grounds. **User correction**: examples-arc theme INCLUDES gap-fix; "off-theme" means different crate / different API contract, NOT "the example would surface the gap." Saved as `feedback_examples_drive_gap_fixes`. | F6 moved IN-ARC: 9th example `mesh-lattice-mesh-bounded-infill` (§5.9) added; F6 gap-fix sub-arc spec at §6.5; §6.2 commit ordering revised to interleave 5 gap-fix commits between example skeleton and post-fix anchors; §10 backlog item 3 (was F6) removed; §11 locked decisions reversed; §7 R9 reframed from "user surprise" to "scope-creep risk on gap b". §6.4 precedent table updated to note "engine fixes mid-arc: yes" (was "unlikely"). |
| **8** | **Patient-cadence correction round** — user feedback 2026-05-01 mid-spec-authoring: "we have to accept that we may find large gaps and we can't rush." The partial-defer escape hatch in §10 item #9 + R9 was contradicting the patient-cadence meta-strategy. | **Partial-defer escape hatch removed**: §10 item #9 deleted; §7 R9 reframed from "stop and raise → defer" to "stop and raise → expand in-arc"; §1 two-tier scope cap revised to clarify that the expected resolution is fix-in-arc + scope expansion (NOT defer); §11 locked decisions added "no partial-defer escape hatches." Reaffirms `feedback_examples_drive_gap_fixes` cadence. |

**Cold re-read pass** (separate quality gate from the §8 stress rounds above): per §11, after this spec is locked but before declaring it ready, do an explicit cold re-read pass per `feedback_thorough_review_before_commit`. The cold re-read surfaces drift between this spec and what the user actually agreed to do — caught the soup-signature mis-numbering (1.5 → 3.0) + the cushion-overcautious anchor in §5.5 during this session.

**§-internal conflicts surfaced** (none structural):
- **8.1**: Tilted cube in §5.1 uses `sin(π/4) = sqrt(2)/2 = 0.7071...` — `f64::sqrt(0.5)` is correctly-rounded, so anchors stay at `1e-12`. `f64::sin(π/4)` is NOT correctly-rounded, so for any sin-based fixture vertex (cylinder, sphere) the tolerance relaxes to `1e-12` (which sin/cos satisfy on x86-64 / aarch64 modern libms). NO conflict; both already specified consistently.
- **8.2**: §5.7 lattice-density-gradient has 2 examples worth of demonstration material (gradient + radial + function variants of `DensityMap`). Considered splitting into `mesh-lattice-density-uniform-gradient` + `mesh-lattice-density-radial-function`, rejected — would push lattice example count to 5 against the rough plan's 4. Bundled into §5.7 with a clear "primary: gradient; sub-demos: uniform, radial, function via direct evaluate calls" framing.

---

## §9. examples/mesh/README.md + book Part 8 update plan

### §9.1 examples/mesh/README.md update

Currently (read 2026-05-01):

- "Examples" table has 8 entries (PR #222 set).
- Tier 1: ply-with-custom-attributes.
- Tier 2: 7 entries listed; #9-#10 (`printability-fdm-validation`, `printability-orientation`, `cross-section-sweep`) listed without links — but PR #223 shipped 8 printability examples, NOT just `printability-fdm-validation`. So Tier 2 is OUT-OF-DATE.
- Tier 3: "5 examples deferred. See architecture book Part 8 for the full inventory."

**v1.0 README rewrite plan**:

1. **Single combined "Examples" table** with 24 entries (16 existing + 8 new). Sort by category alphabetical, OR by mesh book Part order:
   - Core types: `attributed-mesh-basics`, `ply-with-custom-attributes`
   - I/O: `format-conversion`
   - Repair: `mesh-repair-walkthrough`
   - SDF + Offset: `mesh-offset-{outward, inward}`, `mesh-sdf-distance-query` *(NEW)*
   - Shell + Print: `shell-generation-{fast, high-quality}`, `printability-{thin-wall, long-bridge, trapped-volume, self-intersecting, small-feature, orientation, technology-sweep, showcase}` (8)
   - Measurement: `mesh-measure-{bounding-box, cross-section, distance-to-mesh}` *(NEW × 3)*
   - Lattices: `mesh-lattice-{tpms-gyroid, strut-cubic, density-gradient, shape-bounded}` *(NEW × 4)*

2. **Deprecate Tier 1/2/3 framing** — replace with the 8-category mesh-book-Part-aligned grouping. The Tier 1/2/3 distinction was a v0.7 planning device for "what ships first"; v1.0 ships everything. The README's task is to be a static index.

3. **Remove the "Tier 3 — deferred" section entirely**. Replace with a small "v0.9 candidates" subsection that names the 5 deferred examples (4 from F4-F11 deferrals + 1 obsolete `self-intersection-detection`) with their re-open triggers from §1.

4. **Per-example README template** (§4.3) stays unchanged — already canonical.

5. **Add a "v1.0 release" note** at the top of the README: "v1.0 ships with **complete example coverage** of the 10-crate mesh ecosystem. Every public crate has at least one example demonstrating its load-bearing capabilities."

### §9.2 mesh book Part 8 update

Currently `docs/studies/mesh_architecture/src/80-examples.md` has the 16-example inventory in Tier 1/2/3 framing (read in this session). **v1.0 rewrite mirror's §9.1's strategy** — replace tier-based with category-based; mark the 5 v0.9 candidates as deferred with triggers; add the "v1.0 ships with complete example coverage" headline.

The tier-based narrative (Tier 1 first PR, Tier 2 alongside soft-body, Tier 3 deferred) was a planning artifact of the v0.7 era. v1.0 keeps the historical record in a "v0.7 → v1.0 evolution" subsection (one paragraph) — preserves the audit trail for future maintainers.

### §9.3 mesh book Part 10 (Roadmap) update

Add a "v0.9 candidates" subsection with the 5 deferred items (F4, F5, F6, F7+F10+F11, original-Part-8-Tier-3-#15-#16-deferrals). Each entry: name, trigger, brief rationale. Mirrors the format already established in mesh-printability v0.9 backlog (memo + CHANGELOG + book § Known limitations).

---

## §10. v0.9 backlog (triple-tracked at PR-close)

Each entry below lands in (1) AI cross-session memory at `project_mesh_v1_examples_coverage.md` (followups subsection) — or a new memo if expansive; (2) per-crate `CHANGELOG.md` `[Unreleased] / v0.9 candidates` block; (3) mesh book Part 10 (Roadmap) v0.9 candidates.

Migration commits per §6.2: commit 31 (mesh book Part 10 Roadmap update); commit 32 (per-crate CHANGELOG `[Unreleased] / v0.9 candidates` blocks for mesh-sdf F4, mesh-measure F5, mesh-lattice F7/F10/F11; v0.8.0 entry on mesh-lattice for F6 a-e detail).

(F6 was previously a v0.9 candidate in this list — moved IN-ARC per `feedback_examples_drive_gap_fixes` correction; gap-fix sub-arc spec at §6.5.)

| # | Title | Crate | Trigger | Rationale | Effort estimate |
|---|-------|-------|---------|-----------|-----------------|
| 1 | `SdfError::OutOfBounds` + `InvalidDimensions` resolution (F4) | mesh-sdf | Grid SDF API consumer arrives, OR explicit decision to remove | 2 unused error variants are dead code unless the grid API ships | small (~50 LOC: either remove or implement the grid API) |
| 2 | `MeasureError`/`MeasureResult` adoption (F5) | mesh-measure | A function tightens validation to fail-fast (e.g., empty mesh becomes hard error) | Currently `Option<T>` carries the same information for callers; change is API-shape (option<T>→Result<T>) and breaking | small (~80 LOC + breaking-API rename + downstream call-site updates) |
| 3 | Real Voronoi tessellation (F7) | mesh-lattice | User reports cubic-grid artifacts in Voronoi output, OR organic-feeling lattice required for layered-silicone-device payload | Bowyer-Watson or Lloyd's algorithm on perturbed seed grid; replaces the "perturbed cubic" stand-in | medium (~200-400 LOC of new algorithm) |
| 4 | Welded TPMS-lattice MC output (F10) | mesh-lattice | Real consumer needs welded TPMS-lattice for visual aesthetic OR file size compression | mesh-shell already has a weld pass via mesh-repair (see PR #222 commit 11.5.2); apply same pattern to `extract_isosurface` | small (~30 LOC: mesh-repair `weld_vertices` after MC) |
| 5 | 3MF beam writer (F11) | mesh-io | 3MF beam output demand from a printer-driver workflow | `BeamLatticeData` is already the data model; writer needs the 3MF Beam Lattice Extension format | medium (~300-500 LOC: 3MF spec implementation) |
| 6 | Demo non-default BeamCap variants (Flat, Butt) | examples | A user reports surprise that `BeamCap::Flat`/`Butt` aren't visually demonstrated | Augment `mesh-lattice-strut-cubic` with a sub-demo, OR add a separate example | small (~80 LOC additive) |
| 7 | Consolidate `closest_point_on_triangle` duplication | mesh-sdf + mesh-measure | A maintainer asks "why does this exist twice?" | Workspace-hygiene refactor; pick one home (probably `mesh-sdf` since it's the older surface) and re-export | small (~50 LOC: one move + one re-export) |
| 8 | Concave-mesh SDF sign-convention upgrade (winding-number or generalized) | mesh-sdf | A user reports SDF sign flipping incorrectly near edges of concave geometry | Current implementation uses face-normal-of-closest-face; winding-number-based is more robust | medium (~200 LOC; algorithm is well-documented in literature) |
| 9 | Tolerance-aware `OrientedBoundingBox::contains` | mesh-measure | Surfaced 2026-05-02 by §5.1 example. PCA's iterative `SymmetricEigen` produces `half_extents` and the inverse-rotation mapping with ~1 ULP roundoff; the 4 input vertices that defined the OBB extremes can fail strict `<=` containment by `~1.78e-15` | Library should accept an FP epsilon (default 1e-9) so callers don't have to re-implement the local-frame projection. Currently `OrientedBoundingBox::contains` uses strict `<=`, leading to surprising `false` returns for the very vertices the OBB was built from | small (~30 LOC: add `contains_with_tol(point, eps)` method, keep strict `contains` as alias for `contains_with_tol(p, 0.0)`) |
| 10 | Document "OBB ⊄ AABB in general" + remove the corners-within-AABB anchor pattern from API docs/examples | mesh-measure | Surfaced 2026-05-02 by §5.1 spec authoring. The folk-intuition "OBB is tighter than AABB so OBB ⊆ AABB" is false for any non-trivial OBB rotation; OBB corners extend OUTSIDE AABB by `half_extent · sin(rotation_angle)` | One-paragraph caveat in `oriented_bounding_box` rustdoc; correct any user-facing example that asserted "8 OBB corners within AABB" as a sanity check | small (~10 LOC of docstring + audit pass on existing tests) |

---

## §11. Spec lifecycle

This file is **ephemeral working doc**. It exists to align user + Claude on scope, ordering, anchors, and risks BEFORE any code lands. It is deleted at the end of the arc (commit 34 per §6.2) per `feedback_code_speaks`.

**Durable narrative migrates to**:
- `examples/mesh/README.md` — the canonical user-facing per-example index (§9.1 update plan).
- `docs/studies/mesh_architecture/src/80-examples.md` — Part 8 rewrite (§9.2 plan).
- `docs/studies/mesh_architecture/src/100-roadmap.md` — Part 10 Roadmap with v0.9 candidates (§9.3 plan).
- Per-crate `CHANGELOG.md` `[Unreleased] / v0.9 candidates` blocks.
- Per-example crate's `src/main.rs` doc-comments + `README.md`.
- Project memos: a v1.0-closed update to `project_mesh_v1_examples_coverage.md`.

**Cold re-read pass — completed 2026-05-02**. Per `feedback_thorough_review_before_commit`. 4 hard math/code errors caught (HE-1 §5.4 inside-test on origin, HE-2 §5.2 area tolerance vs chord error, HE-3 §5.9 negative-validate via clamping builder, HE-4 §5.7 vert-count anchor on geometric tessellation), 6 soft refinements (SE-1..6 wording / volume-anchor / formula clarifications), 3 style refinements (SR-1..4 explicit double-anchor / dx-abs note / is_closed semantics). All 13 fixes applied to spec before any code lands.

**Re-open triggers for the spec itself** (during execution):
- A per-example anchor formula fails verification → revise §5 in this spec OR raise.
- A pre-flight finding from §2 turns out to be wrong (e.g., F8's "approximate density_to_threshold" is actually bit-exact at density 0.5) → update §2 + §5 anchors.
- A reviewer surfaces a `[lints] workspace = true` fallout site (per the printability v0.8 Gap A precedent) → log in §12 reconciliation table OR raise if > 5 sites.
- §1.2 `generate_infill` gap-fix turns out to be tractable in-arc (e.g., shell-offset alone is the bulk of the value) → raise; user decision required to expand scope.

**Locked decisions** (will NOT revisit during execution):
- **9 examples**, not 8 (revised 2026-05-01 mid-spec-authoring per §8 Round 7). (Per §1 + §5.)
- **1 PR** covering the entire arc (revised 2026-05-02 cold-read session per user directive — was 2 PRs in 2026-05-01 draft). (Per §6.1.)
- **In-arc F6 gap-fix sub-arc** with 5 fixes a–e (revised 2026-05-01 mid-spec-authoring per §8 Round 7 — initial draft pre-deferred to v0.9; user correction inverted the verdict). (Per §1.2 + §6.5.)
- **No partial-defer escape hatches.** Patient gap-discovery is the cadence; if scope grows mid-execution, scope expands in-arc. The §1 two-tier scope cap is for user confirmation of scope expansion, NOT for negotiating partial-deferrals. (Revised 2026-05-01 per user correction; previous draft included a v0.9 candidate "F6 gap b conditional defer" — removed.)
- Math-pass-first anchors per `feedback_math_pass_first_handauthored`. (Per §4.2.)
- Octahedron, not sphere, for §5.4 mesh-sdf-distance-query. (Per §5.4 fixture rationale.)
- Two-cube vs single-cube for §5.1 mesh-measure-bounding-box. (Per §5.1 fixture rationale; OBB needs a non-cube to demonstrate vs AABB.)
- §5.8 (`with_shape_sdf`) and §5.9 (`generate_infill`) are **complementary, not substitutes**: §5.8 is for analytical-SDF-trimmed lattices, §5.9 is for mesh-bounded shell+lattice composites. Both ship in v1.0.

---

## §12. Cross-references

- **Precedent**: `mesh-printability/V08_FIX_ARC_SPEC.md` (deleted at row #25; recoverable from git at commit `0ac8c102~1`). 13-section structure mirrored here.
- **Project memos**:
  - [`project_mesh_v1_examples_coverage`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_mesh_v1_examples_coverage.md) — the directive + scope clarifications.
  - [`project_mesh_printability_gaps`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_mesh_printability_gaps.md) — v0.8 closed + v0.9 backlog precedent.
  - [`project_mesh_architecture_book`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_mesh_architecture_book.md) — book skeleton + AttributedMesh history.
  - [`project_gameplan`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_gameplan.md) — six-phase plan + post-C1 sequencing.
- **Feedback memos shaping cadence**:
  - `feedback_baby_steps`, `feedback_one_at_a_time`, `feedback_one_at_a_time_review`
  - `feedback_museum_plaque_readmes`, `feedback_math_pass_first_handauthored`, `feedback_chamfered_not_rounded`, `feedback_f3d_winding_callout`
  - `feedback_pr_size_ci_economics`, `feedback_pre_squash_tag`, `feedback_post_squash_merge_diff_verify`
  - `feedback_thorough_review_before_commit`, `feedback_risk_mitigation_review`, `feedback_cargo_doc_pre_commit`
  - `feedback_xtask_grade_opacity`, `feedback_grading_rubric`, `feedback_code_speaks`, `feedback_no_reflexive_defer`
- **Mesh book**: `docs/studies/mesh_architecture/` — skeleton landed PR #222; depth-pass weaves through example PRs.
- **A-grade STANDARDS**: `docs/STANDARDS.md` — 8 grading criteria post-2026-04-25 L0 refactor.
