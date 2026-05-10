# scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp

**Row 24 — F4.1 + v3 axial-zoned variation, the first user of `BlendedScalarField` over an axial half-space SDF composed onto row 23's radial `LayeredScalarField` material field.** Same scan + 3-layer sleeve + rigid intrusion probe geometry as [row 23 `scan-fit-3layer-sleeve-yeoh-ramp`](../scan-fit-3layer-sleeve-yeoh-ramp/); constitutive model (Yeoh) and ramp depth (8 mm in 16 × 0.5 mm steps) carry through verbatim — the ONLY differentiator is that each shell's `(μ, C₂, λ)` becomes axially zoned: a "soft tip / stiff anchor" stack with the row 23 anchor set in the proximal (+z, contact end) zone and a one-Shore-step-stiffer sibling stack in the distal (−z, anchored end) zone. See [Yeoh arc memo][arcmemo] Roadmap §"v3 axial-zoned variation" + the [v3 spec memo][spec] for the locked design + decisions.

| Shell | Proximal anchor (+z, contact end) | Distal anchor (−z, anchor end) | μ contrast | Engineering role |
|---|---|---|---|---|
| Inner (φ < 6 mm)            | `ECOFLEX_00_20` (μ = 18 kPa, C₂ = 1.69 kPa)  | `ECOFLEX_00_30` (μ = 23 kPa, C₂ = 2.05 kPa)   | ×1.28 | skin-contact softness |
| Middle (6 mm ≤ φ < 10 mm)   | `DRAGON_SKIN_10A` (μ = 51 kPa, C₂ = 4.46 kPa)| `DRAGON_SKIN_15` (μ = 92 kPa, C₂ = 8.20 kPa)  | ×1.80 | conductive composite proxy |
| Outer (φ ≥ 10 mm)           | `DRAGON_SKIN_20A` (μ = 113 kPa, C₂ = 10.0 kPa)| `DRAGON_SKIN_30A` (μ = 198 kPa, C₂ = 17.6 kPa)| ×1.75 | structural stiffness |

All six anchors are F4 `pub const` Path-1 entries. Per [arc memo §D3][arcmemo] the proximal/distal pair within each shell stays within the same Smooth-On family (Ecoflex inner, Dragon Skin middle + outer) so the smoothstep weight in the band interpolates linearly through the F4 Shore-space convention.

[arcmemo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_yeoh_hyperelastic_arc.md
[spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_24_v3_axial_zoned_spec.md
[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Why axial zoning

Row 23 ships the user-target 8 mm physical intrusion on a uniform 3-shell radial silicone stack. Real soft-actuator designs commonly grade stiffness along the long axis: the contact-end (probe-facing) is softer for compliance, the anchor-end (away from contact) is stiffer for structural support. This row demonstrates that cf-design's `BlendedScalarField` (smoothstep blend over a chosen SDF, between two child `Field<f64>`s) composes cleanly with row 23's `LayeredScalarField` (SDF-thresholded radial layers) to produce a 2-zone-axial × 3-shell-radial material field with **no new `Field<f64>` impl required** — only an inline 5-line `Sdf`-impl `AxialHalfSpace` for the axial blend region.

The composition is per-parameter independent (one `BlendedScalarField` each for μ, C₂, λ) and shares the same axial SDF + band, so the smoothstep weight is identical at every reference-space probe → the band-zone Yeoh sample at any tet is a coherent `(μ_blend, c2_blend, λ_blend)` triple, not a malformed mix of parameters from different axial points.

## Pipeline

```rust
// Inline 5-line half-space SDF for the axial blend SDF.
struct AxialHalfSpace { split_z: f64 }
impl Sdf for AxialHalfSpace {
    fn eval(&self, p: Point3<f64>) -> f64 { p.z - self.split_z }
    fn grad(&self, _: Point3<f64>) -> Vector3<f64> { Vector3::new(0.0, 0.0, 1.0) }
}

// Per-parameter axial-zoned field — three independent BlendedScalarFields
// sharing the same axial SDF + band → smoothstep weight is bit-identical
// at every reference-space probe.
let mu_proximal = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
    [ECOFLEX_00_20.mu, DRAGON_SKIN_10A.mu, DRAGON_SKIN_20A.mu]);
let mu_distal   = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
    [ECOFLEX_00_30.mu, DRAGON_SKIN_15.mu,  DRAGON_SKIN_30A.mu]);
let mu_field    = BlendedScalarField::new(axial, mu_distal, mu_proximal, AXIAL_BAND_HALF_WIDTH);
// ... analogous for c2 and lambda

let material_field = MaterialField::from_yeoh_fields(
    Box::new(mu_field), Box::new(c2_field), Box::new(lambda_field));

// SdfMeshedTetMesh<Yeoh> + chained replay_step over 16 steps to 8 mm — same as row 23.
```

Sign convention (load-bearing — get this wrong and proximal/distal flip): with `phi = p.z − SPLIT_Z`, `phi < 0` means `z < SPLIT` (the distal, −z half of the body). [`BlendedScalarField`]'s doc says "inside_field dominates at SDF-negative samples", so `inside_field = distal` and `outside_field = proximal`.

Peak axial μ-gradient (at the band's interior `|z − SPLIT| < BAND_HALF`) is `(μ_distal − μ_proximal) / (2 · band_half_width) ≈ 4.1 GPa/m` for the middle shell and `8.5 GPa/m` for the outer shell — comparable to or smaller than the existing inner-middle radial step (`(51 − 18) kPa / CELL_SIZE ≈ 8.25 GPa/m`), so Newton's tangent stiffness landscape is no harsher than row 23's already-converged regime.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Outputs are `out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp.json` (final-step scalars + `axial_zoning` metadata + 3-shell × 2-zone Yeoh provenance + per-step `ramp_curve` array + per-active-contact-pair detail at the final step) and `out/sleeve_xslab_final.ply` (x-slab per-tet centroid cloud at the final step rendered at REST positions, with categorical `material_id` + canonical `zone_id` + extra `axial_zone_id` + sequential `displacement_magnitude` + sequential `mu_sampled_pa` + sequential `psi_j_per_m3` strain-energy heatmap).

## Why x-slab over z-slab (vs row 23)

Row 23's z-slab cut at z = 0 (the body equator) catches the propagated radial response of the wrap shell — the cut sits 40 mm BELOW the contact zone at z ≈ +SCAN_HZ. For row 24 the equator IS the AXIAL_SPLIT, INSIDE the smoothstep band — the entire z-slab samples blended-band material, which obliterates the soft-tip / stiff-anchor visualisation that this row demonstrates. An x-slab at x = 0 cuts perpendicular to the long axis and spans the full z range, exposing the proximal-pure / band / distal-pure axial structure as a 2-D centroid cloud. Z-slab tet counts are RETAINED as bit-equal regression gates (cheap centroid filter; not emitted as PLY) — see anchor 3 below.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-shell × 2-zone Yeoh FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 16 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `quality_floors`

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same anchor as row 23. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

### 2. `counts_exact`

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `74_628` | bit-equal to rows 22 + 23 (BCC + IS deterministic on same SDF + hints; material model + axial zoning don't affect discretisation) |
| `n_vertices`          | `31_966` | bit-equal to rows 22 + 23 |
| `n_referenced`        | `17_384` | bit-equal to rows 22 + 23 |
| `n_pinned` (outer-envelope band) | `7_046` | bit-equal to rows 22 + 23 |
| `n_inner_tets`   | `25_892` | bit-equal to rows 22 + 23 |
| `n_middle_tets`  | `16_656` | bit-equal to rows 22 + 23 |
| `n_outer_tets`   | `32_080` | bit-equal to rows 22 + 23 |
| sum                                | `74_628` | partition gate |

Cross-row continuity to rows 22 + 23 IS the gate (pattern (y) — bit-equal cross-row continuity captures regressions; row 24 + row 23 share geometry exactly).

### 3. `zslab_counts_exact` (carry-through; no PLY emit)

| Count | Pinned |
|---|---|
| `n_inner_tets_zslab`  | `768` (bit-equal to rows 22 + 23) |
| `n_middle_tets_zslab` | `432` |
| `n_outer_tets_zslab`  | `892` |

Z-slab tet counts retained as cross-row regression gates at `|cz| < CELL_SIZE / 2 = 0.002 m`. Visualisation cut switches to **x-slab** for v3 — z-slab at z = 0 sits inside the smoothstep band and cannot show the axial gradient.

### 4. `zone_shell_counts_exact` (NEW for v3)

3 zones × 3 shells = 9 cells. Sharp partition at `|centroid.z − AXIAL_SPLIT_Z| = AXIAL_BAND_HALF_WIDTH` ⇒ band, else proximal (cz > +BAND_HALF) / distal (cz < −BAND_HALF). NOTE: zone partition is sharp, NOT smoothstep — band tets are classified as "BAND" but their sampled `(μ, c2, λ)` is the smoothstep interpolation, not the arithmetic mean.

| Count |
|---|
| `n_distal_inner_tets` |
| `n_distal_middle_tets` |
| `n_distal_outer_tets` |
| `n_band_inner_tets` |
| `n_band_middle_tets` |
| `n_band_outer_tets` |
| `n_proximal_inner_tets` |
| `n_proximal_middle_tets` |
| `n_proximal_outer_tets` |
| sum (partition gate) |

### 5. `xslab_zone_shell_counts_exact` (NEW for v3)

Per-zone-shell counts in the `|cx| < CELL_SIZE / 2 = 0.002 m` x-slab cut. The PLY's centroid cloud has exactly this many vertices (sum across all 9 cells).

### 6. `n_ramp_steps_exact`

| Anchor | Bound |
|---|---|
| `len(results) == N_RAMP_STEPS_EXACT` | strict (16) |

### 7. `per_step_solver_converges`

| Anchor | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 150` | per step, all 16 |
| `r.final_residual_norm < 1e-10` | per step, all 16 |

### 8. `per_step_iter_count`

| Anchor | Pinned |
|---|---|
| `IT_COUNT_RAMP_EXACT[k] == r.iter_count` per step | first-bake capture |

NEW capture (different stiffness landscape from row 23 → different Newton path; expect comparable magnitudes to row 23's `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77]`).

### 9. `force_displacement_monotone`

Same shape as row 23: ramp endpoint sanity + strict-adjacent monotone from step 2 onward.

### 10. `per_step_strain_energy_ordering`

Same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 23 — radial partition is unaffected by axial zoning (the per-shell mean averages over both proximal + distal + band tets within the shell).

### 11. `per_step_max_disp_bounded`

| Anchor | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (16) |

### 12. `material_provenance`

For all six anchors used in proximal + distal stacks (`ECOFLEX_00_20`, `ECOFLEX_00_30`, `DRAGON_SKIN_10A`, `DRAGON_SKIN_15`, `DRAGON_SKIN_20A`, `DRAGON_SKIN_30A`): `to_yeoh()` → `Yeoh::energy(I) == 0` and `Yeoh::energy(F_probe) == nh_part + C₂(I₁−3)²` at `EXACT_TOL = 0.0` (additive decomposition per arc memo F1 Spike-1).

### 13. `material_assignment_partition`

Per-tet `mesh.materials()[t].energy(F_probe)` matches the per-zone × per-shell anchor's `to_yeoh()` energy at `EXACT_TOL = 0.0` for tets in the proximal-pure zone (cz > +BAND_HALF) and the distal-pure zone (cz < −BAND_HALF). Band tets are skipped here — see anchor 14.

### 14. `blend_zone_material_provenance` (NEW for v3)

For one tet centroid CLOSEST to z = AXIAL_SPLIT_Z in each radial shell, the sampled material's `Yeoh::energy(F_probe)` matches the analytic smoothstep-blended `Yeoh::energy(F_probe)` at `BLEND_MIDPLANE_TOL = 1e-15`. Reproduces `BlendedScalarField::sample`'s exact FMA pattern (`outside_weight = s² · (3 − 2s)` via `mul_add(-s, 3)`, then lerp via `(1 − w).mul_add(inside, w · outside)`) for bit-exact comparison.

### 15. `n_contact_pairs_final_exact` + `outer_layer_max_psi_final`

| Anchor | Pinned |
|---|---|
| `n_active_pairs` at final step (referenced-only) | first-bake capture (expected ≈ row 23's 50; contact happens entirely in proximal zone) |
| `max Ψ_outer` at final step | bits self-pinned; rel-tol IV-1 sparse-tier |

### 16. `per_step_captured_bits` + `zone_shell_psi_final` — IV-1 sparse-tier rel-tol

Per-step force-displacement + per-shell radial Ψ̄ aggregates self-pinned at first capture (5 quantities × 16 steps = **80 captured-bit anchors** — same shape as row 23). PLUS the final-step 9-cell zone × shell mean Ψ̄ partition (**9 captured-bit anchors**). Compared via `assert_relative_eq!` at `SPARSE_REL_TOL = 1e-12` rel + `SPARSE_EPS_ABS = 1e-12` floor.

Two ordering gates beyond the bit-pin on `zone_shell_psi_final`:

- **Per-zone radial**: `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` within zone.
- **Per-shell axial**: `Ψ̄_proximal > Ψ̄_distal` within shell at the final step. Contact is at the proximal end + softer material there → strain concentrates proximally; the band's blended material sits between by construction.

**Failure-mode protocol per IV-1**: if a rel-tol comparison fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift; (2) if same toolchain, real regression in cf-design's `cuboid` / `offset` / `subtract` plumbing OR sim-soft's BCC + IS + faer hot path OR `BlendedScalarField::sample`'s smoothstep arithmetic OR `MaterialField::sample_yeoh`'s per-parameter sample order OR the chained-`replay_step` path OR Yeoh's `energy` / `first_piola` / `tangent` arithmetic; (3) NEVER re-bake to silence drift.

`CF_CAPTURE_BITS=1` env-var bootstrap pattern (banked at row 19 as pattern (cc)): when set, every captured-anchor check is bypassed and a paste-ready capture block is printed to stderr.

## Visuals

`out/sleeve_xslab_final.ply` — **x-slab** per-tet centroid cloud at the FINAL ramp step (depth = 8 mm) at `|rest_centroid.x| < CELL_SIZE / 2 = 0.002 m`, rendered at **REST positions** (no displacement amplification). Per the [sim-soft viz arc memo][vizarc] this row is the option-1 baby step toward FEM-grade soft-body cf-view rendering: the body keeps its rest-shape rectangle so soft-solid character is preserved, and the contact-zone story shows up as a `psi_j_per_m3` strain-energy heatmap rather than as geometric "explosion." Earlier rows (21/22/23) used `DISPLACEMENT_SCALE = 10.0` to amplify the deformation field for visibility, but at this row's contact intensity (8 mm penetration on a 108 mm body) 10× amplification sent the contact-band tets ~85 mm above the body's rest extent — reading like a fluid spray rather than a soft solid. The displacement field is preserved as the `displacement_magnitude` scalar (true physical magnitude, no scaling).

Six scalars:

- **`material_id`** (categorical extra, 0/1/2 = inner/middle/outer) — radial shell membership.
- **`zone_id`** (canonical `AttributedMesh.zone_ids` slot, written as `property uint zone_id` in the PLY) — sharp axial-zone classification.
- **`axial_zone_id`** (categorical extra mirror of `zone_id`, 0.0/1.0/2.0 = distal/band/proximal) — selectable in cf-view's Scalar dropdown (cf-viewer enumerates `extras` only, not canonical AttributedMesh slots; the disambiguated extra name dodges the reserved-name guard at `mesh-io/src/ply.rs:322`).
- **`displacement_magnitude`** (sequential extra, m) — true physical magnitude, unscaled. Peak is at the contact band on the proximal cap (z ≈ +SCAN_HZ).
- **`mu_sampled_pa`** (sequential extra, Pa) — the per-tet sampled μ, visualises the axial smoothstep blend directly. Within each shell, `mu_sampled_pa` is constant across the proximal-pure zone (e.g. 18 kPa for inner shell), continuously increasing through the band (smoothstep), then constant again across the distal-pure zone (e.g. 23 kPa for inner shell).
- **`psi_j_per_m3`** (sequential extra, J/m³) — per-tet strain-energy density `Ψ = Material::energy(F)`, the FEM stress-concentration **headline scalar**. Spans ~6 orders of magnitude across the body: contact-band tets at the proximal cap reach ~49 200 J/m³ (`MAX_PSI_OUTER_FINAL_REF_BITS` anchor), distal-zone tets sit at ~10⁻⁴ J/m³ (barely deformed). Sequential viridis under cf-view picks out the contact-zone glow against the cool-toned bulk.

[vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_xslab_final.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** + **`axial_zone_id`** are categorical → cf-view picks the **categorical palette**.
- **`displacement_magnitude`** + **`mu_sampled_pa`** + **`psi_j_per_m3`** are unipolar continuous → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D rectangle on `x = 0`, spanning the full z range from −SCAN_HZ − WRAP_THICKNESS to +SCAN_HZ + WRAP_THICKNESS (≈ 108 mm tall). The probe contact zone at z ≈ +SCAN_HZ shows the deformation peak; the soft-tip / stiff-anchor material gradient is visible in `mu_sampled_pa`'s smoothstep transition through the equator band.

**Force-displacement curve via matplotlib.** The `ramp_curve` array carries the per-step force / displacement / Ψ̄ trace. Optional matplotlib post-processing via PEP 723 inline metadata:

```sh
uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/plot_ramp.py
```

Same dual-axis depth × `force_z` + depth × `max_disp` plot as row 23, with Newton iter counts annotated. Force trajectory expected higher than row 23 at deeper depths (distal half is stiffer); max_disp expected lower (distal stiffness compresses the deformation field toward the proximal half).

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required. Per-step runtime is similar to row 23 (~3-6 s per step late in the ramp); 16-step total ~40-90 s release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness; finer cells (e.g. `0.002 m`) trip an SPD pivot at the FIRST ramp step (empirically tested at row-22 v2-spec spike time, applies to row 24 by inheritance — same mesh + meshing pipeline).

## First-time bit capture

```sh
CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp --release
```

Emits a paste-ready block of every `*_EXACT` count (including the 9-cell zone × shell partition + x-slab partition), `IT_COUNT_RAMP_EXACT` array, every `*_RAMP_REF_BITS` array, the 9-cell `MEAN_PSI_*_FINAL_BITS`, `MAX_PSI_OUTER_FINAL_REF_BITS`, and `N_CONTACT_PAIRS_FINAL_EXACT`, bypassing the captured-anchor checks. Use for first-time author-bake and intentional re-bake; the IV-1 protocol forbids using this to silence a drift assertion.

## Roadmap (followups, not in row-24 scope)

This row realises the v3 "axial-zoned variation" entry from row 23's roadmap. Next steps in the queued evolution toward iter-2+ silicone-device design support:

- **v4 (explicit Cu mesh sub-layer)** — 4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A. **Gated on solver-side fix** (faer LU fallback) since CELL_SIZE ≲ 0.25 m is needed to resolve a 0.5 mm sub-layer, well below the 2 mm CELL_SIZE that already trips an SPD pivot at the FIRST ramp step (row 22 v2-spec Spike A).
- **v5+ (3-param Yeoh with measured C₃)** — when post-cast Fork-B calibration data lands, switch from 2-param to 3-param Yeoh per the U1 future-proofing note in the arc memo. The `c3: Option<f64>` field in `SiliconeMaterial` is already in place.
- **vN — real anatomy scan replacing the cuboid fixture** (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
