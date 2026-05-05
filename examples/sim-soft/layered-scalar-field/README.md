# layered-scalar-field

**Phase 4 multi-material spatial composition — `LayeredScalarField` over a 3-shell concentric `SphereSdf` partition; per-tet `(μ, λ)` sampled at the centroid carries the layer's Lamé pair, visualized as integer `material_layer_id ∈ {0, 1, 2}` in cf-view.** A solid `SphereSdf{ radius: R_OUTER }` body partitions into three concentric shells via two `LayeredScalarField`s (one per Lamé parameter) threaded through `MaterialField::from_fields`; `SdfMeshedTetMesh::from_sdf` samples each per-tet pair at the centroid and caches it in `Mesh::materials()`. The headline cf-view artifact is a per-tet centroid point cloud, **filtered to a thin `|z| < cell_size/2` slab** so the projected disk reads as three concentric color rings on the z=0 plane — cf-view's categorical-colormap heuristic (integer-valued + < 16 unique → tab10) renders the **sharp-boundary spatial composition** `LayeredScalarField` ships. Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/material_layer_assignment.ply` (648 z-slab centroids + per-vertex categorical scalar; the `verify_*` correctness gates run over all 6768 body tets).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The first user-facing example of Tier 3 multi-material per [`EXAMPLE_INVENTORY.md`][inv] — the canonical pattern for spatial composition:

```rust
// Solid body at the IV-5 silicone-device geometry minus the cavity.
// Reuses LAYERED_SPHERE_R_OUTER / _OUTER_INNER / _INNER_OUTER from
// sim-soft for cross-row alignment with row 11 (concentric-lame-shells).
let body = || Box::new(SphereSdf { radius: R_OUTER });

// Two LayeredScalarFields — one per Lamé parameter — keyed on the same
// SphereSdf, with thresholds [PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD]
// on phi = ‖p‖ − R_OUTER and per-shell values.
let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
    body(), vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
    vec![MU_INNER, MU_MIDDLE, MU_OUTER],
));
let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
    body(), vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
    vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
));

// MaterialField::from_fields aggregates the two trait-object stacks.
let material_field = MaterialField::from_fields(mu_field, lambda_field);

// SdfMeshedTetMesh::from_sdf calls materials_from_field at the end of
// the BCC + Isosurface Stuffing pipeline; per-tet (μ, λ) is sampled at
// the centroid and cached in mesh.materials().
let mesh = SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: R_OUTER }, &MeshingHints {
    bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
    cell_size: 0.02,
    material_field: Some(material_field),
})?;
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

The `LayeredScalarField` boundary convention is the partition rule `partition_point(|&t| t <= phi)`: at exactly `phi == threshold[i]`, the point lands in the **outer** shell (`values[i+1]`). Concretely for the 3-shell sphere with `R_OUTER = 0.10`, `R_OUTER_INNER = 0.08`, `R_INNER_OUTER = 0.06`:

```text
phi = ‖p‖ − R_OUTER
PHI_INNER_THRESHOLD  = R_INNER_OUTER  − R_OUTER = -0.04
PHI_MIDDLE_THRESHOLD = R_OUTER_INNER  − R_OUTER = -0.02

phi <  -0.04                 ⇒ inner shell  ‖p‖ ∈ [0,    R_INNER_OUTER)
-0.04 ≤ phi < -0.02          ⇒ middle shell ‖p‖ ∈ [R_INNER_OUTER, R_OUTER_INNER)
-0.02 ≤ phi                  ⇒ outer shell  ‖p‖ ∈ [R_OUTER_INNER, R_OUTER]
```

This is **row 8 of the sim-soft examples arc** — the first Tier 3 multi-material example, generalizing rows 4-6's uniform-material scenes. It's also the **first cf-view consumer in PR1's later half** (rows 4-6 were JSON-only per inventory Q4 visualization assignment).

**Geometry constants reused from `SoftScene::layered_silicone_sphere`.** `LAYERED_SPHERE_R_OUTER` / `_OUTER_INNER` / `_INNER_OUTER` / `_BBOX_HALF_EXTENT` are sim-soft's IV-5 silicone-device geometry; row 8 uses three of the four (skipping `R_CAVITY` — row 8 is solid). Row 11 (`concentric-lame-shells`) will reuse all four constants plus `DifferenceSdf` for the hollow form. Same Lamé pairs across both rows give visual continuity in cf-view.

**Lamé pairs match `sdf_material_tagging.rs` IV-4 deviation.** `(0.5×, 1×, 2×)` the IV-1 baseline, all in the `λ = 4μ` ⇒ `ν = 0.4` compressible regime:

| Shell | μ (Pa) | λ (Pa) | factor |
|---|---|---|---|
| inner  | `5.0e4` | `2.0e5` | `0.5×` |
| middle | `1.0e5` | `4.0e5` | `1.0×` (= IV-1 baseline) |
| outer  | `2.0e5` | `8.0e5` | `2.0×` |

Three distinct values discriminate against shell-swap bugs that Decision J's outer = inner symmetry would mask. `cell_size = 0.02` matches III-1 / IV-4 h/2 canonical → 6768 tets per the III-1 determinism contract; row 8's scene is exactly IV-4's at h/2, exposed user-facing.

**Why a per-tet centroid point cloud (vs majority-vote per-vertex or boundary-shell PLY).** The cf-view viewer's per-vertex scalar surface (per VIEWER_DESIGN.md Q4 — per-face is deferred) needs each per-tet quantity broadcast to a vertex. A majority-vote-per-vertex on the body's tet mesh would smear the layer interfaces (a vertex on `‖p‖ = R_INNER_OUTER` is incident to inner and middle tets — majority arbitrarily picks one). A per-shell boundary surface would render only the exterior of each layer, losing the volumetric distribution. The centroid point cloud puts each per-tet quantity at exactly one vertex with no smearing.

**Why a z-slab cut (vs the full 6768-centroid body).** First-pass visual review surfaced an engine-side density gap: cf-view's commit-3 instanced-sphere radius factor (`bbox.diagonal() * 0.005`) was tuned for sphere-sdf-eval's 1331-point sparse `[-1.5, 1.5]³` grid; on this row's `[-0.10, 0.10]³` body with 3624 outer-shell tets densely packed in a thin annulus, the rendered spheres overlap visually and the inner layers are occluded. Restricting the PLY emit to a thin slab `|centroid.z| < cell_size/2 = 0.01 m` (~10% of the body's diameter) projects the centroid distribution onto a near-flat disk; cf-view renders it as three concentric color rings unmistakably from any orbit angle, mirroring hollow-shell-sdf row 2's z=0 cross-section precedent for cf-view consumers. An octant-cut alternative (`+x ∧ +y ∧ +z`) was considered first — it reveals the cut faces' radial layering well from the cut-face camera angles, but the convex outer-shell surface still dominates the default 3/4-perspective view, so the slab approach wins on first-impression pedagogy. The `verify_per_tet_layer_assignment` gate still runs over all 6768 tets (correctness independent of the visual filter); only the PLY artifact is filtered to 648 z-slab centroids. The engine-side fix (auto-scale point-sphere radius for dense clouds) is a viewer-arc followup rather than row 8's scope.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `layered_field_thresholds_strictly_monotone`

| Anchor | Bound |
|---|---|
| `R_INNER_OUTER < R_OUTER_INNER < R_OUTER` | strict `<` |
| `PHI_INNER_THRESHOLD < PHI_MIDDLE_THRESHOLD` | strict `<` |

The `LayeredScalarField` constructor's runtime invariant (`field/layered.rs:101-110`) re-asserted at the example layer via `const { assert!(...) }` — compile-time enforcement on the geometry constants. A regression that swaps `R_INNER_OUTER` and `R_OUTER_INNER` surfaces at `cargo build`, anchored at this source location.

### 2. `determinism`

| Anchor | Bound |
|---|---|
| Second `from_sdf` call vs first | `equals_structurally` |
| Per-tet `materials()[t].energy(F_probe)` across calls | `to_bits` equality |

Anchors III-1 + Decision N (`I-5` carry-forward) at the user-facing example layer. Per-tet material-cache determinism extends III-1's mesh-topology determinism: `materials_from_field` walks tets in `tet_id` order, single-threaded; same input → bit-equal cache.

### 3. `counts`

| Anchor | Bound |
|---|---|
| `mesh.n_tets()` | exact-pin `6768` |
| `mesh.n_vertices()` | exact-pin `4634` |
| `referenced_vertices.len()` | exact-pin `1483` |
| `referenced.len() < n_vertices` | strict `<` (orphan-rejection invariant) |

Identical to row 3's `N_*_EXACT` — material-field carry doesn't shift mesh topology (BCC + stuffing runs before `materials_from_field` populates the cache). Pinning exactly per the III-1 determinism contract gives the tightest regression net.

### 4. `per_tet_layer_assignment_matches_sdf_predicate`

**Headline gate.** For every tet, `mesh.materials()[t]` is probed at `F_probe = diag(1.2, 1, 1)` against `expected[shell_at(centroid)]`:

| Anchor | Bound |
|---|---|
| Per tet `materials()[t].energy(F_probe)` vs `expected[layer_id].energy(F_probe)` | `epsilon = 0.0` (bit-equal) |
| Per tet `materials()[t].first_piola(F_probe)` vs `expected[layer_id].first_piola(F_probe)` | `epsilon = 0.0` (bit-equal) per entry |

`NeoHookean::{mu, lambda}` are private; the Material-trait probe runs through identical `NeoHookean::first_piola` / `energy` arithmetic on identical `(μ, λ)` pairs and identical `F` — bit-equal by construction on a fixed toolchain. The cross-implementation gate: the test-side `shell_at` reimplements `LayeredScalarField`'s `partition_point(|&t| t <= phi)` rule, so centroid-sampled `materials()` and re-derived `expected[shell_at(...)]` MUST agree on every tet by construction. Drift between mesher partition pass and test re-derivation fires loud.

`F_probe = diag(1.2, 1, 1)` ties to row 6's `LAMBDA_STRETCH = 1.20` for cross-row continuity. Both `energy(F_probe)` (scalar) AND `first_piola(F_probe)` (Matrix3, all 9 entries) are probed — a regression that nudges `μ` and `λ` such that energy happens to coincide while first-Piola differs surfaces in the matrix probe.

### 5. `shell_populations_non_empty`

| Anchor | Bound |
|---|---|
| Inner-shell tet count | strict `>` `0` |
| Middle-shell tet count | strict `>` `0` |
| Outer-shell tet count | strict `>` `0` |

Sanity guard against a degenerate scene (e.g., a middle annulus too thin to admit any centroid). Mirrors IV-4's `iv_4_shell_populations_are_non_empty_at_finest`.

### 6. `shell_populations_exact`

| Anchor | Bound |
|---|---|
| Inner-shell tet count | exact-pin `1344` |
| Middle-shell tet count | exact-pin `1800` |
| Outer-shell tet count | exact-pin `3624` |
| `inner + middle + outer` | exact `== N_TETS_EXACT` |

Captured per-shell counts under the III-1 determinism contract. Mirrors row 3's `N_*_EXACT` shape — same scene + resolution + toolchain, run-to-run + same-toolchain bit-stable. Cross-platform sparse-mesh stability (~3k tets at h/2) is empirically untested; a future CI-matrix expansion may surface drift, in which case follow the IV-1 sparse-tier failure-mode protocol (rule out toolchain drift first; never re-bake without diagnosing).

### 7. `layer_id_categorical_count`

| Anchor | Bound |
|---|---|
| Unique layer IDs across all tets | exact `== 3` |
| Each `rec.layer_id < 3` | strict `<` |

Gates cf-view's Q5 categorical-colormap heuristic (`integer-valued + < 16 unique values` → tab10). If a regression introduced a fourth bucket (e.g., a sentinel ID for "outside the body"), cf-view would still pick categorical (4 < 16), but the example's pedagogy — three sharp concentric shells — would silently slip. The exact `== 3` pins the pedagogy.

### 8. `interface_flags_all_false`

| Anchor | Bound |
|---|---|
| `mesh.interface_flags().len()` | `== n_tets` |
| Count of `true` flags | exact `== 0` |

`MaterialField::with_interface_sdf` was NOT called during construction, so `interface_flags_from_field` short-circuits to `vec![false; n_tets]` (`mesh/mod.rs:166-168`). Pedagogically clarifies the interface flag is the **`BlendedScalarField`** smooth-transition band rule (`|φ(x_c)| < L_e` per Part 7 §02 §01) — not relevant to `LayeredScalarField`'s sharp boundaries. A future `blended-scalar-field` example (row 9) will exercise the populated path.

### 9. `centroid_radii_within_body`

| Anchor | Bound |
|---|---|
| Per tet `‖centroid‖` | strict `<` `R_OUTER` |

Solid-body sanity: the SDF mesher only emits tets inside the body's zero set, so centroids must be strictly interior. Empirically `max ‖c‖ ≈ 9.76e-2` m at this resolution (body radius `0.10` m) — the surface tets' centroids lie within `cell_size / 4` of the surface. A regression that retains exterior tets surfaces here.

### 10. `zslab_visual_populations_exact`

| Anchor | Bound |
|---|---|
| Z-slab inner-shell tet count | exact-pin `216` |
| Z-slab middle-shell tet count | exact-pin `176` |
| Z-slab outer-shell tet count | exact-pin `256` |
| Each z-slab per-shell count | strict `>` `0` (visual-pedagogy guard) |

Gates the visual pedagogy: each layer must have ≥ 1 z-slab centroid for cf-view to render the concentric ring structure. Captured per the III-1 determinism contract (same scene + toolchain + platform as the full-body counts above; see [Capture provenance](#6-shell_populations_exact) inheriting from anchor 6). Total ≈ `648` (≈ `N_TETS · slab_thickness / body_diameter = 6768 · 0.02 / 0.20 = 677` modulo BCC + stuffing layer-alignment effects). The inner-shell count is higher than the middle-shell count even though the body's full-population inner < middle: a z-slab through the solid inner ball (radius `R_INNER_OUTER`) projects to a disk, while a slab through the middle annular shell projects to a thin annular slice — the disk packs more centroids per unit slab thickness than the annulus, hence the ordering inversion.

## Visuals

`out/material_layer_assignment.ply` — the canonical visual artifact.

```text
648 vertices (one per tet centroid in the |z| < cell_size/2 slab cut)
0 faces (point cloud)
extras["material_layer_id"]: f32  // integer 0 / 1 / 2 — categorical
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/layered-scalar-field/out/material_layer_assignment.ply
```

cf-view auto-discovers the single per-vertex scalar `material_layer_id`, default-selects it (alphabetical first; only scalar present), detects categorical via Q5's heuristic (integer-valued + 3 unique values < 16) → renders the tab10 palette. The thin z-slab projects to a near-flat disk in cf-view, reading as three concentric color rings on the z=0 plane: an inner solid disk (layer 0), a middle annular ring (layer 1), and an outer annular ring (layer 2). The pedagogy is unmistakable from any orbit angle — no need to face a specific cut plane. Per `feedback_visual_review_is_the_test` — for cf-view consumers, the visual pass is real (not collapsed to JSON read).

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, full-body + z-slab per-shell tet partitions, max centroid radius).

## Run

```text
cargo run -p example-sim-soft-layered-scalar-field --release
```

Output: `out/material_layer_assignment.ply` and stdout summary.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The exact-pin counts were captured under release-mode build; matching that invocation shape removes one variable from the III-1 determinism contract.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Cross-references

- **Sister sim-soft examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait contract on the same `SphereSdf` primitive used here as the partition key), `sdf-to-tet-sphere` (row 3 — same canonical `R = 0.1, cell_size = 0.02` III-1 scene this row inherits exact-pinned counts from), `multi-element-stretch` (row 6 — uniform-material counterpart; row 8 generalizes from one Lamé pair to three), forthcoming `concentric-lame-shells` (row 11 — same Lamé pairs + same constants + IV-5 hollow form for the closed-form Lamé radial-displacement validation).
- **Internal-fixture template**: `sim/L0/soft/tests/sdf_material_tagging.rs` (IV-4 multi-material centroid-sampling correctness gate). Row 8's `shell_at` reimplements that file's `shell_at` verbatim; the canonical scene at h/2 and Lamé pairs match exactly. Row 8 is the user-facing exposure of IV-4.
- **`LayeredScalarField`**: `sim/L0/soft/src/field/layered.rs:62-132` — the discrete-step composition over `Field<f64>`. Constructor invariants (`thresholds` strictly monotone, `values.len() == thresholds.len() + 1`, `thresholds` finite) re-asserted here at compile time via `const { assert!(...) }`.
- **`MaterialField::from_fields`**: `sim/L0/soft/src/material/material_field.rs` — aggregates one `Box<dyn Field<f64>>` per Lamé parameter. Phase 4 ships `f64` only; `Vec3` (HGO fiber direction) and `Tensor3` (full orthotropy) are Phase H.
- **`materials_from_field` / `interface_flags_from_field`**: `sim/L0/soft/src/mesh/mod.rs:124-190` — the per-tet centroid-sampling pass that populates `Mesh::materials()` and `Mesh::interface_flags()` at construction. Implements Part 7 §02 §00 sampling and §02 §01 composition rules.
- **`SoftScene::layered_silicone_sphere`**: `sim/L0/soft/src/readout/scene.rs:175-301` — the IV-5 hollow-shell version of this scene; row 11 will exercise it. Geometry constants `LAYERED_SPHERE_R_OUTER` / `_OUTER_INNER` / `_INNER_OUTER` / `_BBOX_HALF_EXTENT` are reused here for cross-row continuity.
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` Q4 (per-vertex scalar surface, per-face deferred) + Q5 (categorical-colormap heuristic: integer-valued + < 16 unique values → tab10).
- **Book references**: Part 2 Ch 09 [`00-sdf-valued.md`](../../../docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields/00-sdf-valued.md) (`Field<T>` trait); Part 7 Ch 02 [`00-sampling.md`](../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/00-sampling.md) + [`01-composition.md`](../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/01-composition.md) (centroid sampling + sharp-step composition).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md).
