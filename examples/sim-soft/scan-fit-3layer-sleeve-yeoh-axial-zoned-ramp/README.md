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

// Two more axially-blended fields carry the per-anchor calibrated validity caps
// (0.8·λ_break tensile, 0.20 compressive) — without them the deep solve
// fail-closes on the legacy NH σ=2.0 ceiling near 8 mm (see rows 23/24).
let max_stretch_field = BlendedScalarField::new(axial, max_distal, max_proximal, AXIAL_BAND_HALF_WIDTH);
let min_stretch_field = BlendedScalarField::new(axial, min_distal, min_proximal, AXIAL_BAND_HALF_WIDTH);
let material_field = MaterialField::from_yeoh_fields_with_bounds(
    Box::new(mu_field), Box::new(c2_field), Box::new(lambda_field),
    Box::new(max_stretch_field), Box::new(min_stretch_field));

// SdfMeshedTetMesh<Yeoh> + chained replay_step over 16 steps to 8 mm — same as row 23.
```

Sign convention (load-bearing — get this wrong and proximal/distal flip): with `phi = p.z − SPLIT_Z`, `phi < 0` means `z < SPLIT` (the distal, −z half of the body). [`BlendedScalarField`]'s doc says "inside_field dominates at SDF-negative samples", so `inside_field = distal` and `outside_field = proximal`.

Peak axial μ-gradient (at the band's interior `|z − SPLIT| < BAND_HALF`) is `(μ_distal − μ_proximal) / (2 · band_half_width) ≈ 4.1 GPa/m` for the middle shell and `8.5 GPa/m` for the outer shell — comparable to or smaller than the existing inner-middle radial step (`(51 − 18) kPa / CELL_SIZE ≈ 8.25 GPa/m`), so Newton's tangent stiffness landscape is no harsher than row 23's already-converged regime.

Every gate sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. This row is a Rule-B **validator**: its gates are pipeline-emergent structural + physics invariants read from the real 16-step ramp, not captured-bit self-pins (those were stripped in the Rule-B de-frag — constitutive, blend, and mesher correctness are lib-owned). Outputs are `out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp.json` (final-step scalars + `axial_zoning` metadata + 3-shell × 2-zone Yeoh material params + per-step `ramp_curve` array + per-active-contact-pair detail) and the F1.2 viz PLYs `out/sleeve_boundary_final.ply` + `out/sleeve_design_slab_cut_x0_final.ply` + `out/sleeve_design_surface_deformed_step_01..16.ply` (full 3D body, `x = 0` cross-section spanning the axial gradient, and per-step deformed design surfaces via `sim_soft::viz`), each with categorical `material_id` + `zone_id` and sequential `displacement_magnitude` + `mu_sampled_pa` scalars.

## Why x-slab over z-slab (vs row 23)

Row 23's z-slab cut at z = 0 (the body equator) catches the propagated radial response of the wrap shell — the cut sits 40 mm BELOW the contact zone at z ≈ +SCAN_HZ. For row 24 the equator IS the AXIAL_SPLIT, INSIDE the smoothstep band — the entire z-slab samples blended-band material, which obliterates the soft-tip / stiff-anchor visualisation that this row demonstrates. An x-slab at x = 0 cuts perpendicular to the long axis and spans the full z range, exposing the proximal-pure / band / distal-pure axial structure — emitted as a triangulated cross-section (`sim_soft::viz::slab_cut` / `design_slab_cut`). Z-slab per-shell tet counts are retained as cheap non-empty population gates (centroid filter, not emitted as PLY) — see anchor 3 below.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-shell × 2-zone Yeoh FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each gate is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*` and is called from `main()` in dependency order; `cargo run --release` exit-0 means every gate passed. They are **pipeline-emergent structural + physics invariants** read from the real 16-step ramp — resolution- and toolchain-robust. The pre-Rule-B captured-bit self-pins (exact counts incl. the 9-cell zone × shell + x-slab freezes, the per-step iter-count freeze, the 80 per-step + 9 zone × shell final `to_bits()` Ψ̄/force/displacement pins, the `to_yeoh()` additive-decomposition provenance mirror AND the blend-zone smoothstep self-mirror) were **stripped**: they froze one run's FP trajectory on one toolchain, and the correctness they redundantly implied is lib-owned — Yeoh closed form + additive decomposition + round-trip (`yeoh_contract.rs` + `silicone_table.rs` tests), per-shell routing (`sdf_material_tagging.rs` IV-4), and the `BlendedScalarField` cubic-Hermite blend (`blended_material_composition.rs`). The observed per-step + zone × shell scalars are still emitted to the JSON + stdout for eyes-on inspection — just not self-pinned.

### 1. `quality_floors`

| Gate | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same gate as row 23. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

### 2. `mesh_structure`

| Gate | Bound |
|---|---|
| `n_tets` | `> 0` |
| `n_referenced` | `> 0` and `≤ n_vertices` (referenced ⊆ all vertices) |
| `n_pinned` (outer-envelope band) | non-empty proper subset of `n_referenced` |
| each per-shell tet count | `> 0` |

Structural invariants, not exact counts (the specific counts are a mesher-version artifact). Per-shell routing correctness is checked in gate 9 + owned by lib IV-4.

### 3. `zslab_populations`

| Gate | Bound |
|---|---|
| each z-slab per-shell tet count (`\|cz\| < CELL_SIZE / 2 = 0.002`) | `> 0` |

Non-empty, not exact-count. (The v3 visualization cut switches to **x-slab** — z-slab at z = 0 sits inside the smoothstep band and cannot show the axial gradient.)

### 4. `zone_shell_populations`

| Gate | Bound |
|---|---|
| each of the 9 zone × shell cells (distal/band/proximal × inner/middle/outer) | `> 0` |

The 3-zone × 3-shell decomposition is fully populated — the axial-zoning demonstration is meaningful. Sharp partition at `\|centroid.z − AXIAL_SPLIT_Z\| = AXIAL_BAND_HALF_WIDTH` ⇒ band, else proximal / distal. (Band tets are classified structurally as "band" but sample a smoothstep-blended `(μ, C₂, λ)` — that blend is lib-owned, `blended_material_composition.rs`.)

### 5. `xslab_zone_shell_populations`

| Gate | Bound |
|---|---|
| each of the 9 x-slab (`\|cx\| < CELL_SIZE / 2`) zone × shell cells | `> 0` |

The `x = 0` viz cut populates every zone × shell cell, so the axial material gradient is visible across the full decomposition.

### 6. `per_step_solver_converges`

| Gate | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 150` | per step, all 16 |
| `r.final_residual_norm < 1e-10` | per step, all 16 |

Observed iter counts comparable to row 23's `[8 … 77]`. (Robust 8 mm convergence depends on the calibrated Yeoh validity bounds — routed via `from_yeoh_fields_with_bounds`, blended axially like μ/C₂/λ; without them the solve fail-closes on the legacy NH σ=2.0 ceiling near step 16, exactly as rows 23/24 did before the fix.)

### 7. `force_displacement_monotone`

Same shape as row 23: ramp endpoint sanity (`last > first`) + strict-adjacent monotone from step 2 onward (referenced-only filtered `+z` force).

### 8. `per_step_strain_energy_ordering`

`Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` at every step — radial partition unaffected by axial zoning (the per-shell mean averages over proximal + band + distal tets within the shell).

### 9. `material_routing`

| Gate | Bound |
|---|---|
| `mesh.materials()[t]` per proximal-pure / distal-pure tet | `(μ, C₂, λ)` bit-equal to that zone's F4 Yeoh anchor for the tet's shell, each pure zone exercised ≥ 1 tet |

The scene's axial-zoning routing invariant — reads the real per-tet `Yeoh` via public `.mu()`/`.c2()`/`.lambda()` and compares (exact `to_bits() ==`) to the proximal stack (`ECOFLEX_00_20`/`DRAGON_SKIN_10A`/`DRAGON_SKIN_20A`) for proximal-pure tets and the distal stack (`ECOFLEX_00_30`/`DRAGON_SKIN_15`/`DRAGON_SKIN_30A`) for distal-pure tets. A routing check, not a constitutive mirror. Band tets (smoothstep-blended) are skipped — their blend is lib-owned. Yeoh closed form + `to_yeoh()` round-trip are lib-owned (`yeoh_contract.rs` + `silicone_table.rs::tests::to_yeoh_round_trips_yeoh_fields_for_each_anchor`); the generic routing mechanism is lib-owned (`sdf_material_tagging.rs` IV-4).

### 10. `per_step_max_disp_bounded`

| Gate | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (16) |

### 11. `contact_engaged`

| Gate | Bound |
|---|---|
| `n_active_pairs` at final step | `> 0` |

The final (deepest) ramp step engaged the probe (contact happens entirely in the proximal zone). Non-empty; the exact count was a mesher artifact.

### 12. `zone_shell_psi_final` — the row-24 headline (structural orderings)

The 9-cell zone × shell mean-Ψ̄ table obeys BOTH orderings at the final (deepest) step:

- **Per-zone radial**: `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` within each of distal / band / proximal.
- **Per-shell axial**: `Ψ̄_proximal > Ψ̄_distal` within each shell — the physical consequence of the soft-proximal / stiff-distal material gradient under proximal-end contact (strain concentrates proximally; the band sits between by construction). The 9 captured-bit Ψ̄ pins on this table were stripped; the orderings are read from the real solve.

## Visuals

Post-F1.2-viz-retrofit the final step (depth = 8 mm) emits proper triangulated viz artifacts via `sim_soft::viz` (not the pre-retrofit x-slab centroid cloud `sleeve_xslab_final.ply`): `out/sleeve_boundary_final.ply` (full 3D body) + `out/sleeve_design_slab_cut_x0_final.ply` (the `x = 0` cross-section — chosen over a z-slab because a z-slab at `z = 0` sits inside the smoothstep band and can't show the axial gradient, whereas the `x = 0` cut spans the full ≈108 mm z range) + the per-step deformed-surface sequence `out/sleeve_design_surface_deformed_step_01..16.ply`. The artifacts carry these scalars:

- **`material_id`** (categorical extra, 0/1/2 = inner/middle/outer) — radial shell membership.
- **`zone_id`** (canonical `AttributedMesh.zone_ids` slot, written as `property uint zone_id` in the PLY) — sharp axial-zone classification.
- **`axial_zone_id`** (categorical extra mirror of `zone_id`, 0.0/1.0/2.0 = distal/band/proximal) — selectable in cf-view's Scalar dropdown (cf-viewer enumerates `extras` only, not canonical AttributedMesh slots; the disambiguated extra name dodges the reserved-name guard at `mesh-io/src/ply.rs:322`).
- **`displacement_magnitude`** (sequential extra, m) — true physical magnitude, unscaled. Peak is at the contact band on the proximal cap (z ≈ +SCAN_HZ).
- **`mu_sampled_pa`** (sequential extra, Pa) — the per-tet sampled μ, visualises the axial smoothstep blend directly. Within each shell, `mu_sampled_pa` is constant across the proximal-pure zone (e.g. 18 kPa for inner shell), continuously increasing through the band (smoothstep), then constant again across the distal-pure zone (e.g. 23 kPa for inner shell).
- **`psi_j_per_m3`** (sequential extra, J/m³) — per-tet strain-energy density `Ψ = Material::energy(F)`, the FEM stress-concentration **headline scalar**. Spans ~6 orders of magnitude across the body: contact-band tets at the proximal cap reach ~49 200 J/m³ (observed), distal-zone tets sit at ~10⁻⁴ J/m³ (barely deformed). Sequential viridis under cf-view picks out the contact-zone glow against the cool-toned bulk.

[vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_design_slab_cut_x0_final.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** + **`axial_zone_id`** are categorical → cf-view picks the **categorical palette**.
- **`displacement_magnitude`** + **`mu_sampled_pa`** + **`psi_j_per_m3`** are unipolar continuous → cf-view picks **sequential viridis**.

The `x = 0` slab cut is a triangulated cross-section spanning the full z range from −SCAN_HZ − WRAP_THICKNESS to +SCAN_HZ + WRAP_THICKNESS (≈ 108 mm tall). The probe contact zone at z ≈ +SCAN_HZ shows the deformation peak; the soft-tip / stiff-anchor material gradient is visible in `mu_sampled_pa`'s smoothstep transition through the equator band. (An eyes-on-pixels review of the F1.2 retrofit artifacts is a user-side pass — the boundary/slab-cut meshes replaced the pre-F1.2 centroid cloud this section originally described.)

**Force-displacement curve via matplotlib.** The `ramp_curve` array carries the per-step force / displacement / Ψ̄ trace. Optional matplotlib post-processing via PEP 723 inline metadata:

```sh
uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/plot_ramp.py
```

Same dual-axis depth × `force_z` + depth × `max_disp` plot as row 23, with Newton iter counts annotated. Force trajectory expected higher than row 23 at deeper depths (distal half is stiffer); max_disp expected lower (distal stiffness compresses the deformation field toward the proximal half).

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required. Per-step runtime is similar to row 23 (~3-6 s per step late in the ramp); 16-step total ~40-90 s release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness. Finer cells (e.g. `0.002 m`) were observed to trip an SPD pivot at the first ramp step during the row-22 v2-spec spike (pre-A2 era), and the claim originally propagated here "by inheritance" on the assumption of same mesh + meshing pipeline. **Status (post-A2, 2026-05-11): partially falsified.** Row 25 (the open-mouth fork sharing this row's meshing pipeline) was directly tested at `CELL_SIZE = 0.002` post-A2 and converged cleanly through 3 ramp steps with zero LU fallback engagements (iter counts 8/13/17 at depths 0.5/1.0/1.5 mm; full 8-step ramp not completed due to compute time). This row 24's behavior at `CELL_SIZE = 0.002` post-A2 is **likely similar but not directly tested** — the pre-A2 SPD-trip claim may have applied to a now-superseded geometry or assembly, or the A2 LU fallback may handle the trip without anyone noticing. B2 followup per `docs/SIM_SOFT_ROADMAP.md` Track B.

## Roadmap (followups, not in row-24 scope)

This row realises the v3 "axial-zoned variation" entry from row 23's roadmap. Next steps in the queued evolution toward iter-2+ silicone-device design support:

- **v4 (explicit Cu mesh sub-layer)** — 4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A. **Gated on solver-side fix** (faer LU fallback) since CELL_SIZE ≲ 0.25 m is needed to resolve a 0.5 mm sub-layer, well below the 2 mm CELL_SIZE that already trips an SPD pivot at the FIRST ramp step (row 22 v2-spec Spike A).
- **v5+ (3-param Yeoh with measured C₃)** — when post-cast Fork-B calibration data lands, switch from 2-param to 3-param Yeoh per the U1 future-proofing note in the arc memo. The `c3: Option<f64>` field in `SiliconeMaterial` is already in place.
- **vN — real anatomy scan replacing the cuboid fixture** (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
