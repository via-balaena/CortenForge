# blended-scalar-field

**Phase 4 multi-material spatial composition — `BlendedScalarField` cubic-Hermite-smoothstep transition between two uniform Lamé regions ("stiff skin over soft core" idiom from `material_field.rs:130`), with the same SDF threaded through `MaterialField::with_interface_sdf` to populate `Mesh::interface_flags` per the IV-6 `|φ(x_c)| < L_e` rule.** A solid `SphereSdf{ radius: R_OUTER = 0.10 }` body (reused from row 8) is meshed; an interior `SphereSdf{ radius: R_INTERFACE = 0.07 }` (NEW) drives two `BlendedScalarField`s (one per Lamé parameter) blending `(MU_INNER, LAMBDA_INNER)` (inside the interface) into `(MU_OUTER, LAMBDA_OUTER)` (outside) over a band of half-width `BAND_HALF_WIDTH = 0.015 m`. The same interior sphere is then attached via `with_interface_sdf` so `Mesh::interface_flags` populates per the book's `|φ(x_c)| < L_e` rule (Part 7 §02 §01) — **the headline new feature vs row 8's all-false interface flags.** The headline cf-view artifact is a per-tet centroid point cloud, **filtered to a thin `|z| < cell_size/2` z-slab** (row 8 banked pattern reused), carrying three per-vertex scalars: `interface_flag` (categorical tab10 — IV-6 flagged band highlight), `material_mu` (continuous viridis — the physical readout), and `smoothstep_weight` (continuous viridis — the cubic Hermite kernel mechanism). Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/material_blend.ply` (648 z-slab centroids + 3 per-vertex scalars; the `verify_*` correctness gates run over all 6768 body tets).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The second user-facing example of Tier 3 multi-material per [`EXAMPLE_INVENTORY.md`][inv] — the canonical `BlendedScalarField + with_interface_sdf` composition pattern, generalizing row 8's sharp-step `LayeredScalarField` to smooth blending:

```rust
let body = SphereSdf { radius: R_OUTER };                              // 0.10
let interface_sdf = || Box::new(SphereSdf { radius: R_INTERFACE });    // 0.07

// Two BlendedScalarFields — one per Lamé parameter — keyed on the
// interior sphere; cubic Hermite smoothstep over band of half-width
// BAND_HALF_WIDTH straddling the interface zero set.
let mu_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
    interface_sdf(),
    Box::new(ConstantField::new(MU_INNER)),
    Box::new(ConstantField::new(MU_OUTER)),
    BAND_HALF_WIDTH,                                                   // 0.015
));
let lambda_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
    interface_sdf(),
    Box::new(ConstantField::new(LAMBDA_INNER)),
    Box::new(ConstantField::new(LAMBDA_OUTER)),
    BAND_HALF_WIDTH,
));

// MaterialField::from_fields aggregates; with_interface_sdf attaches
// the SAME interior sphere for IV-6 |phi(c)| < L_e flagging — the
// canonical "stiff skin over soft core" pattern.
let material_field = MaterialField::from_fields(mu_field, lambda_field)
    .with_interface_sdf(interface_sdf());

let mesh = SdfMeshedTetMesh::from_sdf(&body, &MeshingHints {
    bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
    cell_size: 0.02,
    material_field: Some(material_field),
})?;
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

The `BlendedScalarField` boundary convention is the cubic Hermite smoothstep `s²(3 − 2s)` over `s = clamp((phi + band) / (2·band), 0, 1)`. With `phi = ‖p‖ − R_INTERFACE`, `R_INTERFACE = 0.07`, `BAND_HALF_WIDTH = 0.015`:

```text
phi = ‖p‖ − R_INTERFACE
PHI_INSIDE_SNAP_THRESHOLD  = -BAND_HALF_WIDTH = -0.015  ⇔ ‖p‖ = 0.055
PHI_OUTSIDE_SNAP_THRESHOLD = +BAND_HALF_WIDTH = +0.015  ⇔ ‖p‖ = 0.085

phi ≤ -0.015                ⇒ inside snap   ‖p‖ ≤ 0.055   (μ = MU_INNER, λ = LAMBDA_INNER)
-0.015 < phi < +0.015       ⇒ smoothstep band 0.055 < ‖p‖ < 0.085
phi ≥ +0.015                ⇒ outside snap  ‖p‖ ≥ 0.085   (μ = MU_OUTER, λ = LAMBDA_OUTER)
```

Boundary inclusivity follows the `clamp(0, 1)` exactly: `s = 0` at `phi = -band` (inside snap inclusive) and `s = 1` at `phi = +band` (outside snap inclusive); cubic Hermite is C¹ at both edges and bit-exact at `s ∈ {0, 0.5, 1}`.

This is **row 9 of the sim-soft examples arc** — the second Tier 3 multi-material example, generalizing row 8's sharp-boundary `LayeredScalarField` to smooth-transition `BlendedScalarField`. It's also the **first user-facing coverage of the `BlendedScalarField + with_interface_sdf` composition path not covered by IV-6** (`tests/interface_band_flagging.rs:53-60` explicitly notes IV-6 uses `LayeredScalarField`, NOT `BlendedScalarField`).

**Geometry constants reused from row 8.** `LAYERED_SPHERE_R_OUTER` (body radius `0.10`) and `LAYERED_SPHERE_BBOX_HALF_EXTENT` (bbox `0.12`) — same body sphere + bbox as row 8 for cross-row visual continuity. `R_INTERFACE = 0.07` is a NEW constant: the visual midpoint between row 8's `R_INNER_OUTER = 0.06` and `R_OUTER_INNER = 0.08`, semantically distinct from those (row 9's `R_INTERFACE` is the BlendedScalarField zero-set "stiff skin over soft core" boundary, NOT a partition boundary). `BAND_HALF_WIDTH = 0.015` is NEW: `≈ 0.75 × CELL_SIZE`, so the full band (`2 × BAND_HALF_WIDTH = 0.03`) is roughly 1.5 BCC cells wide — well-resolved, neither under-sampled nor inside-snap-collapsing.

**Lamé pairs reuse row 8's MU_INNER / LAMBDA_INNER (innermost shell) + MU_OUTER / LAMBDA_OUTER (outermost shell), skipping the middle shell.** Row 9 reads pedagogically as "row 8 with the middle shell dissolved into a smoothstep transition between the inner and outer values." All in `λ = 4μ` ⇒ `ν = 0.4` compressible regime; `0.5×` and `2×` IV-1 baseline:

| Region | μ (Pa) | λ (Pa) | factor |
|---|---|---|---|
| inside snap  | `5.0e4` | `2.0e5` | `0.5×` (= row 8 inner) |
| outside snap | `2.0e5` | `8.0e5` | `2.0×` (= row 8 outer) |

`cell_size = 0.02` matches III-1 / IV-4 h/2 canonical (same as row 8) → 6768 tets per the III-1 determinism contract.

**Why a per-tet centroid point cloud + z-slab cut.** Same rationale as row 8 — see [row 8 README's "Why a per-tet centroid point cloud" + "Why a z-slab cut" subsections][r8] for the long-form discussion. cf-view's commit-3 instanced-sphere radius factor (`bbox.diagonal() * 0.005`) is oversized for dense per-tet clouds in the small `[-0.10, 0.10]³` body bbox; the z-slab pattern (z-slab to ~10% of body diameter, ~648 centroids vs 6768) projects to a near-flat disk readable from any orbit angle. See [`project_cf_viewer_dense_point_cloud_gap.md`][gap] for the engine-side followup.

[r8]: ../layered-scalar-field/README.md
[gap]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_cf_viewer_dense_point_cloud_gap.md

**Why three per-vertex scalars (vs row 8's one categorical).** Row 9's headline contrasts row 8 across THREE mechanism layers: (1) the IV-6 categorical interface flag — the *new feature*; (2) the continuous physical μ readout — the *outcome*; (3) the continuous smoothstep weight — the *blend kernel itself*. cf-view's dropdown switches between the three on the same point cloud; alphabetical-first lands on `interface_flag` so the IV-6 flagged band is the loud first impression on launch.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `band_half_width_validity`

| Anchor | Bound |
|---|---|
| `BAND_HALF_WIDTH > 0.0` | strict `>` |
| `BAND_HALF_WIDTH.is_finite()` | finite |
| `R_INTERFACE - BAND_HALF_WIDTH > 0.0` | inside snap exists in body |
| `R_INTERFACE + BAND_HALF_WIDTH < R_OUTER` | outside snap exists in body |

The `BlendedScalarField` constructor's runtime invariant (`field/layered.rs:196-201`) re-asserted at the example layer via `const { assert!(...) }` — compile-time enforcement on the geometry constants. Plus the example-specific "band fully interior to body" contract that `outside_band_snaps_bit_equal` and `band_populations_all_non_empty` depend on.

### 2. `determinism`

| Anchor | Bound |
|---|---|
| Second `from_sdf` call vs first | `equals_structurally` |
| Per-tet `materials()[t].energy(F_probe)` across calls | `to_bits` equality |
| Per-tet `interface_flags()[t]` across calls | exact `bool` equality |

Anchors III-1 + Decision N (`I-5` carry-forward) at the user-facing example layer. Per-tet material-cache and interface-flag-cache determinism extends III-1's mesh-topology determinism: both `materials_from_field` and `interface_flags_from_field` walk tets in `tet_id` order, single-threaded; same input → bit-equal caches. Sister of IV-6's `iv_6_flag_vector_is_run_to_run_deterministic` for the BlendedScalarField path.

### 3. `counts`

| Anchor | Bound |
|---|---|
| `mesh.n_tets()` | exact-pin `6768` |
| `mesh.n_vertices()` | exact-pin `4634` |
| `referenced_vertices.len()` | exact-pin `1483` |
| `referenced.len() < n_vertices` | strict `<` (orphan-rejection invariant) |

Identical to row 3 / row 8's `N_*_EXACT` — material-field composition does not shift mesh topology (BCC + stuffing runs before `materials_from_field` and `interface_flags_from_field` populate the caches).

### 4. `per_tet_blended_assignment_matches_smoothstep_predicate`

**HEADLINE 1.** For every tet, `mesh.materials()[t]` is probed at `F_probe = diag(1.2, 1, 1)` against `expected = NH::from_lame(blend(MU_INNER, MU_OUTER, w), blend(LAMBDA_INNER, LAMBDA_OUTER, w))` where `w = smoothstep_weight(phi(centroid))`:

| Anchor | Bound |
|---|---|
| Per tet `materials()[t].energy(F_probe)` vs `expected.energy(F_probe)` | `epsilon = 0.0` (bit-equal) |
| Per tet `materials()[t].first_piola(F_probe)` vs `expected.first_piola(F_probe)` | `epsilon = 0.0` (bit-equal) per entry |

The cross-implementation gate: the test-side smoothstep arithmetic (`smoothstep_weight` + `fma_blend`) is a bit-exact mirror of `BlendedScalarField::sample` (`field/layered.rs:212-225`) — identical clamp + cubic Hermite + FMA-blend. `expected.energy(F_probe)` and `mesh.materials()[t].energy(F_probe)` therefore run identical NH arithmetic on identical blended `(μ, λ)` pairs and identical `F` — bit-equal by construction on a fixed toolchain. Drift between the mesher's centroid-sampling pass and the test-side re-derivation fires loud at every tet.

`F_probe = diag(1.2, 1, 1)` ties to row 6's `LAMBDA_STRETCH = 1.20` and row 8's `PROBE_LAMBDA` for cross-row continuity. Both `energy(F_probe)` (scalar) AND `first_piola(F_probe)` (Matrix3, all 9 entries) are probed — energy alone is one linear equation in `(μ, λ)`, `first_piola`'s `P_22 = λ ln J` directly fixes `λ` then `P_11` fixes `μ` (over-determination).

### 5. `outside_band_snaps_bit_equal`

| Anchor | Bound |
|---|---|
| For tets with `phi(c) ≤ -BAND_HALF_WIDTH`, `materials()[t].energy(F_probe)` | `to_bits` equality vs `NH(MU_INNER, LAMBDA_INNER).energy(F_probe)` |
| For tets with `phi(c) ≥ +BAND_HALF_WIDTH`, `materials()[t].energy(F_probe)` | `to_bits` equality vs `NH(MU_OUTER, LAMBDA_OUTER).energy(F_probe)` |

Verifies the BlendedScalarField "snaps cleanly to 0 or 1 outside the band" contract at `field/layered.rs:148-153`. At the boundaries, `s = clamp((phi + band) / (2·band), 0, 1)` evaluates to exactly `0` or `1` (the clamp + IEEE 754 give bit-exact 0 or 1 at the boundary inputs); the FMA-blend `fma(1, MU_INNER, 0 * MU_OUTER) = MU_INNER` and `fma(0, MU_INNER, 1 * MU_OUTER) = MU_OUTER` are then bit-exact. Together these gate the "no float noise leaks across the band into the snap regions" property the BlendedScalarField docstring commits to.

### 6. `inside_band_monotonic_mu_gradient`

| Anchor | Bound |
|---|---|
| For tets with `\|phi(c)\| < BAND_HALF_WIDTH` sorted by `phi(c)` ascending, consecutive `blended_mu` values | non-decreasing (`<=`) |

Inventory row 9's named gate — directly verifies the smooth-gradient property the row's headline ships. The cubic Hermite smoothstep `s²(3 − 2s)` is monotone non-decreasing in `s ∈ [0, 1]`; `s` is monotone non-decreasing in `phi`; the FMA-blend `MU_INNER + w·(MU_OUTER − MU_INNER)` is monotone non-decreasing in `w` since `MU_OUTER > MU_INNER`. Composing the three monotonicities, `blended_mu` must be monotone non-decreasing in `phi` — verified directly across all 2736 band tets.

### 7. `interface_flags_match_book_rule_per_tet`

**HEADLINE 2.** For every tet:

| Anchor | Bound |
|---|---|
| `mesh.interface_flags()[t]` | exact `bool` equality vs `(\|phi(centroid)\| < L_e(t))` |
| `mesh.interface_flags().len()` | `== n_tets` |

First user-facing coverage of the `BlendedScalarField + with_interface_sdf` composition path not covered by IV-6 (`tests/interface_band_flagging.rs:53-60` explicitly notes IV-6 uses `LayeredScalarField` shell-pattern, NOT `BlendedScalarField`). Test-side `mean_edge_length` is a bit-exact mirror of `interface_flags_from_field` at `mesh/mod.rs:180-186` — six norms in the exact same order, divided by 6.0 once. `expected_flag = phi.abs() < l_e` (strict `<`, mirroring `mesh/mod.rs:187`). On a fixed toolchain, observed and expected bit-equal by construction.

Note: `L_e(t)` varies per tet with the BCC + Isosurface Stuffing edge-length distribution and is empirically larger than `BAND_HALF_WIDTH = 0.015` on most tets at this resolution (flagged count `3480` > smoothstep band count `2736`), so the IV-6 interface band and the smoothstep band are **distinct but overlapping** populations.

### 8. `band_populations_all_non_empty`

| Anchor | Bound |
|---|---|
| Inside-snap bucket count | strict `>` `0` |
| Smoothstep band bucket count | strict `>` `0` |
| Outside-snap bucket count | strict `>` `0` |
| IV-6 interface-flagged tet count | strict `>` `0` |

Sanity guard against a degenerate scene + ensures the monotonicity gate (anchor 6) and the per-tet flag gate (anchor 7) have something to verify (the headline new feature vs row 8 fires).

### 9. `band_populations_exact`

| Anchor | Bound |
|---|---|
| Inside-snap bucket count | exact-pin `1056` |
| Smoothstep band bucket count | exact-pin `2736` |
| Outside-snap bucket count | exact-pin `2976` |
| `inside-snap + band + outside-snap` | exact `== N_TETS_EXACT` |
| IV-6 interface-flagged tet count | exact-pin `3480` |

Captured per-bucket counts under the III-1 determinism contract. Mirrors row 3 / row 8's `N_*_EXACT` shape — same scene + resolution + toolchain, run-to-run + same-toolchain bit-stable. Cross-platform sparse-mesh stability (~3k tets at h/2) is empirically untested; a future CI-matrix expansion may surface drift, in which case follow the IV-1 sparse-tier failure-mode protocol (rule out toolchain drift first; never re-bake without diagnosing). The IV-6 flagged count is pinned SEPARATELY because the interface band (`|phi(c)| < L_e(t)`, varies per tet) is a different rule from the smoothstep band (`|phi(c)| < BAND_HALF_WIDTH`, constant).

### 10. `zslab_visual_populations_exact`

| Anchor | Bound |
|---|---|
| Z-slab inside-snap bucket count | exact-pin `200` |
| Z-slab smoothstep band bucket count | exact-pin `248` |
| Z-slab outside-snap bucket count | exact-pin `200` |
| Each z-slab per-bucket count | strict `>` `0` (visual-pedagogy guard) |

Gates the visual pedagogy: each bucket must have ≥ 1 z-slab centroid for cf-view to render the smooth gradient + skin colors. Captured per the III-1 determinism contract (capture provenance inherits anchor 9). Total `648` (≈ `N_TETS · slab_thickness / body_diameter = 6768 · 0.02 / 0.20 = 677` modulo BCC + stuffing layer-alignment effects). The band bucket has the highest z-slab count (248) — at z=0 the band annulus has the largest cross-sectional area of the three regions (`π·(0.085² − 0.055²) ≈ 0.0132 m²`, vs the inside-snap disk's `π·0.055² ≈ 0.0095 m²` and the outside-snap thin annulus capped by the body surface), so it accumulates the most centroids in the slab.

## Visuals

`out/material_blend.ply` — the canonical visual artifact.

```text
648 vertices (one per tet centroid in the |z| < cell_size/2 slab cut)
0 faces (point cloud)
extras["interface_flag"]:    f32  // categorical 0.0 / 1.0 (IV-6 |phi(c)| < L_e flag)
extras["material_mu"]:       f32  // continuous Pa        (physical μ readout)
extras["smoothstep_weight"]: f32  // continuous [0, 1]    (cubic Hermite kernel)
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/blended-scalar-field/out/material_blend.ply
```

cf-view auto-discovers the three per-vertex scalars; alphabetical-first selects `interface_flag` on launch — cf-view's Q5 colormap heuristic detects integer-valued + 2 unique values < 16 → tab10 categorical palette, rendering the IV-6 flagged band as a binary highlight (one color for true, one for false). The thin z-slab projects to a near-flat disk in cf-view, reading as a flagged-band annulus straddling the interface zero set on the z=0 plane.

User dropdowns to `material_mu`: cf-view detects continuous + all-positive → viridis sequential palette, rendering the **physical μ readout** as a smooth color gradient — inside-snap solid-color disk (low μ), smooth gradient ring across the band (μ varies smoothly through the smoothstep), outside-snap solid-color outer ring (high μ). Or to `smoothstep_weight`: viridis sequential on `[0, 1]`, rendering the **cubic Hermite kernel** directly — useful for inspecting the blend mechanism independent of the Lamé values.

Per `feedback_visual_review_is_the_test` — for cf-view consumers, the visual pass is real (not collapsed to JSON read).

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, full-body + per-bucket smoothstep partition + IV-6 flagged count + z-slab partition).

## Run

```text
cargo run -p example-sim-soft-blended-scalar-field --release
```

Output: `out/material_blend.ply` and stdout summary.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The exact-pin counts were captured under release-mode build; matching that invocation shape removes one variable from the III-1 determinism contract.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Cross-references

- **Sister sim-soft examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait contract on the same `SphereSdf` primitive used here as both body and interface), `hollow-shell-sdf` (row 2 — z=0 cross-section precedent for cf-view), `sdf-to-tet-sphere` (row 3 — same canonical `R = 0.1, cell_size = 0.02` III-1 scene this row inherits exact-pinned counts from), `multi-element-stretch` (row 6 — uniform-material counterpart at the IV-1 baseline; row 9 generalizes from one Lamé pair to a smooth blend), `layered-scalar-field` (row 8 — sharp-step counterpart; row 9 dissolves row 8's middle shell into a smoothstep transition between row 8's inner and outer Lamé pairs), forthcoming `concentric-lame-shells` (row 11 — IV-5 hollow form).
- **Internal-fixture templates**: `sim/L0/soft/tests/field_unit.rs` (`BlendedScalarField` boundary-value cases + cubic Hermite midpoint = 0.5 + constructor-panic gates) + `sim/L0/soft/tests/interface_band_flagging.rs` (IV-6 LayeredScalarField + with_interface_sdf coverage; row 9 extends to the BlendedScalarField path that file explicitly notes is uncovered).
- **`BlendedScalarField`**: `sim/L0/soft/src/field/layered.rs:167-226` — the smooth-step composition over `Field<f64>`. Cubic Hermite kernel `s²(3 − 2s)` (C¹ at band edges) + FMA-blend; constructor invariants (`band_half_width > 0` and finite) re-asserted here at compile time via `const { assert!(...) }` plus the example-specific band-fully-interior contract.
- **`MaterialField::with_interface_sdf`**: `sim/L0/soft/src/material/material_field.rs:124-142` — builder method attaching the SDF whose zero set drives `Mesh::interface_flags`. The "stiff skin over soft core" canonical pattern named at `material_field.rs:130-132`.
- **`materials_from_field` / `interface_flags_from_field`**: `sim/L0/soft/src/mesh/mod.rs:124-190` — the per-tet centroid-sampling passes that populate `Mesh::materials()` and `Mesh::interface_flags()` at construction. Implements Part 7 §02 §00 sampling and §02 §01 composition + interface-flag rules. Test-side `smoothstep_weight + fma_blend` and `mean_edge_length` are bit-exact mirrors of these for the cross-impl gate.
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` Q5 (categorical-colormap heuristic — integer-valued + < 16 unique values → tab10; otherwise sequential viridis).
- **Book references**: Part 2 Ch 09 [`00-sdf-valued.md`](../../../docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields/00-sdf-valued.md) (`Field<T>` trait); Part 7 Ch 02 [`00-sampling.md`](../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/00-sampling.md) + [`01-composition.md`](../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/01-composition.md) (centroid sampling + smooth-step composition + `|φ(x_c)| < L_e` interface rule).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md).
