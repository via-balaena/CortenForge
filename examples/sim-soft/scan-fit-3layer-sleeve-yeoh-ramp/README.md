# scan-fit-3layer-sleeve-yeoh-ramp

**Row 23 — F4.1 Yeoh consumer, the load-bearing first user of the Yeoh hyperelastic foundation shipped in PR #235 (`e0c2f856`).** Same scan + 3-layer sleeve + rigid intrusion probe geometry as [row 22 `scan-fit-3layer-sleeve-ramp`](../scan-fit-3layer-sleeve-ramp/); constitutive model swapped from Neo-Hookean to **Yeoh** (additive `C₂(I₁−3)²` extension over NH's deviatoric kernel, same NH-style compressibility — see [Yeoh arc memo][arcmemo]); ramp extended from 12 × 0.5 mm = 6 mm to **16 × 0.5 mm = 8 mm** so the `+4` past-NH-wall steps test whether Yeoh's wider validity envelope translates into solver-clean convergence at the user-target physical depth.

**Headline result.** Yeoh reaches **8 mm cleanly** in 77 Newton iters at the final step (row 22 NH's wall was at ~6.5 mm tripping fail-closed on `validate_F_in_domain`). Force at 8 mm: **49.3 N** vs row 22's 25.6 N at 6 mm — Yeoh's `C₂(I₁−3)²` stiffening engages at the higher-strain regime where NH would have refused. The arc thesis (Yeoh fixes the row-22 wall via wider validity envelope) is validated.

| Layer | Thickness (m) | Hardware target | Source material | Yeoh proxy | Construction path | Engineering role |
|---|---|---|---|---|---|---|
| Inner  | 6 mm | 6 mm | Ecoflex 00-30 + 75 % Slacker (effective Shore 00-20) | μ = 18 kPa, C₂ = 1.69 kPa, λ = 72 kPa | **Path 2**: `from_effective_shore(DoubleZero(20.0))?.to_yeoh()` | skin-contact softness |
| Middle | 4 mm | 5 mm | DS10A + Cu mesh + carbon black (effective Shore 15-18A) | μ = 51 kPa, C₂ = 4.46 kPa, λ = 204 kPa | **Path 1**: `DRAGON_SKIN_10A.to_yeoh()` | conductive composite |
| Outer  | 4 mm | 3 mm | DS20A direct | μ = 113 kPa, C₂ = 10.0 kPa, λ = 452 kPa | **Path 1**: `DRAGON_SKIN_20A.to_yeoh()` | structural stiffness |

The inner-layer constructor uses **Path 2** (parametric API) deliberately — at `DoubleZero(20.0)` the bracket is `(ECOFLEX_00_10, ECOFLEX_00_20)` with weight = 1.0, so the produced parameters bit-coincide with the `ECOFLEX_00_20` anchor; the [`ConstructionSource::Interpolated { weight: 1.0, .. }`][src] provenance tag distinguishes the construction path. Production Slacker-softened recipes that don't coincide with a published anchor reach for the same API with non-degenerate bracketing weights.

[arcmemo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_yeoh_hyperelastic_arc.md
[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
[v2spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_22_v2_spec.md
[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md
[src]: ../../../sim/L0/soft/src/material/silicone_table.rs

## Why Yeoh, not Neo-Hookean

Row 22's post-ship investigation (banked at the [v2 spec memo][v2spec]'s "Post-ship investigation" section) identified the wall at `max_disp ≈ 7 mm` as the **Neo-Hookean validity domain** tripping fail-closed. Phase 4 Decision Q's `validate_F_in_domain` refuses to evaluate NH past `max_stretch_deviation < 1.0`; at 6.5 mm cavity-wall displacement, one tet under the probe has principal stretches `[2.06, 1.22, 0.073]` — both extremes are past NH's calibrated faithful range, with the **0.073 (92.7 % compression)** value the load-bearing failure (the symmetric NH gate happens to catch both at the `max(|σᵢ - 1|) > 1.0` threshold; that's a coincidence of NH's specific bound).

Yeoh has per-anchor **asymmetric** validity bounds: `max_principal_stretch ≈ 5-9` (per-family tensile cap, calibrated from Smooth-On TDS elongation-at-break) and `min_principal_stretch = 0.30` (engineering-aggressive default — no published Yeoh-on-silicone compression bound exists per the arc memo §A2 web-search recon). The compressive 0.073 row 22 saw is well inside Yeoh's 0.30 floor. Whether the SOLVER also handles the past-6.5-mm regime (Armijo `α_min = 2⁻²¹` floor was the v2-spike secondary wall, independent of the constitutive model) was the load-bearing unknown the first run answered: **yes, with margin** (final-step 77 iters under `MAX_NEWTON_ITER = 150`, residual 9.87e-11).

## Why a ramp, not a single step

Same rationale as row 22 inherited from row 21: deeper penetration single-stepped on this geometry pushes the iter-0 penalty gradient out of Newton's basin (single-step failure modes were directly observed at related configs — v1.5's capsule + capsule at 4 mm depth tripped a non-PD pivot at iter 3; the earlier-pivot 459 K-tet 2 mm-cell experiment at 8 mm depth inverted tets in the first Newton step). The quasi-static ramp circumvents the single-step basin issue: each step's `x_prev` is the previous step's `x_final`, so the iter-0 gradient is bounded by the per-step delta only.

This row uses the same **quasi-static `replay_step` chaining** pattern row 22 introduced (`x_prev_k+1 = x_final_k`, `v_prev = 0` throughout, no inertia carry-over) — the only differentiators are the Yeoh material model and the extended depth.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Outputs are `out/scan_fit_3layer_sleeve_yeoh_ramp.json` (final-step scalars + 3-material Yeoh provenance + per-step `ramp_curve` array + per-active-contact-pair detail at the final step) and `out/sleeve_zslab_final.ply` (z-slab per-tet centroid cloud at the final step, with categorical `material_id` + sequential `displacement_magnitude`).

## What this example demonstrates

**Workflow** (the architectural payload — Yeoh material model with full F4.0 generic-mesh plumbing):

```rust
// Path 2 (parametric API) for the inner layer; Path 1 (anchor) for middle + outer.
fn inner_silicone() -> SiliconeMaterial {
    SiliconeMaterial::from_effective_shore(
        ShoreReading::DoubleZero(20.0),
        Some("Slacker-softened Ecoflex 00-30 ..."),
    ).expect("DoubleZero(20.0) brackets ECOFLEX family at weight=1.0")
}

// 3-layer Yeoh MaterialField — three scalar fields (μ, C₂, λ); C₁ = μ/2 derived.
let mu_field     = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
                       [inner.mu,     DRAGON_SKIN_10A.mu,     DRAGON_SKIN_20A.mu]);
let c2_field     = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
                       [inner.c2,     DRAGON_SKIN_10A.c2,     DRAGON_SKIN_20A.c2]);
let lambda_field = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
                       [inner.lambda, DRAGON_SKIN_10A.lambda, DRAGON_SKIN_20A.lambda]);
let material_field = MaterialField::from_yeoh_fields(
    Box::new(mu_field), Box::new(c2_field), Box::new(lambda_field));

// SdfMeshedTetMesh<Yeoh> via the F4.0 dedicated constructor.
let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints)?;

// Quasi-static intrusion ramp — chained replay_step over 16 steps to 8 mm.
for k in 0..N_RAMP_STEPS {
    let depth = (k + 1) as f64 * RAMP_STEP_DELTA;
    let probe_k = build_probe_solid_at_depth(depth);
    let solver_k: PenaltyRigidContactYeohSolver<SdfMeshedTetMesh<Yeoh>> =
        CpuNewtonSolver::new(/* ..., contact_k, ... */);
    let step_k = solver_k.replay_step(&x_prev_t, &Tensor::zeros(&[n_dof]), &theta, STATIC_DT);
    x_prev_flat.clone_from(&step_k.x_final);  // chain
}
```

The Yeoh material model composes through the F4.0 generic-mesh refactor without any per-row plumbing: `Mesh<M = NeoHookean>` defaults to NH for legacy consumers, this row writes `<Yeoh>` explicitly to pick the Yeoh slot. Future Yeoh consumers (rows 24+ in the projected ladder) reuse the same constructor pair (`from_yeoh_fields` + `from_sdf_yeoh`) and solver alias (`PenaltyRigidContactYeohSolver`).

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material Yeoh FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 12 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `quality_floors`

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same anchor as row 22. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

### 2. `counts_exact`

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `74_628` | bit-equal to row 22 (BCC + IS deterministic on same SDF + hints; material model doesn't affect discretisation) |
| `n_vertices`          | `31_966` | bit-equal to row 22 |
| `n_referenced`        | `17_384` | bit-equal to row 22 |
| `n_pinned` (outer-envelope band) | `7_046` | bit-equal to row 22 |
| `n_inner_tets`   | `25_892` | bit-equal to row 22 |
| `n_middle_tets`  | `16_656` | bit-equal to row 22 |
| `n_outer_tets`   | `32_080` | bit-equal to row 22 |
| sum                                | `74_628` | partition gate |

Cross-row continuity to row 22 IS the gate (pattern (y) — bit-equal cross-row continuity captures regressions; row 23 + row 22 share geometry exactly, so counts are bit-equal).

### 3. `zslab_counts_exact`

| Count | Pinned |
|---|---|
| `n_inner_tets_zslab`  | `768` (per-shell partition of the z-slab cut at `\|cz\| < CELL_SIZE / 2 = 0.002`) |
| `n_middle_tets_zslab` | `432` |
| `n_outer_tets_zslab`  | `892` |

Bit-equal to row 22 (z-slab geometry + cut axis are unchanged).

### 4. `n_ramp_steps_exact`

| Anchor | Bound |
|---|---|
| `len(results) == N_RAMP_STEPS_EXACT` | strict (16) |

Partition gate — the ramp must produce exactly 16 step results.

### 5. `per_step_solver_converges`

| Anchor | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 150` | per step, all 16 |
| `r.final_residual_norm < 1e-10` | per step, all 16 |

Each step's Newton solve must converge within the iter cap + tolerance bound. Empirical iter counts: `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77]` — step 16 (the deepest at 8 mm) is the closest to the cap with **73 iters of margin**.

### 6. `per_step_iter_count`

| Anchor | Pinned |
|---|---|
| `IT_COUNT_RAMP_EXACT[k] == r.iter_count` per step | `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77]` |

The chained `replay_step` is deterministic on a fixed toolchain (rustc 1.95.0 on macOS arm64); iter-count drift signals real solver-path regression, not noise. Tighter than `per_step_solver_converges` (anchor 5) — that gate accepts any iter count below the cap; this gate pins to specific values.

**Cross-model comparison.** Row 22's NH counts at the same depths 0.5..6 mm were `[8, 8, 9, 11, 11, 13, 14, 16, 19, 22, 30, 61]`. Row 23's Yeoh counts at depths 0.5..6 mm are `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27]` — Yeoh's `C₂(I₁−3)²` stiffening yields a smoother Newton path past row 22's step 11 (e.g. step 12 drops 61 → 27 iters). The past-NH-wall extension `[31, 39, 49, 77]` at depths 6.5..8 mm escalates as expected and remains under the cap with 73-iter margin at step 16.

### 7. `force_displacement_monotone`

| Anchor | Bound |
|---|---|
| `force_total_z[N-1] > force_total_z[0]` | strict (ramp endpoint sanity) |
| `force_total_z[k+1] > force_total_z[k]` for k ≥ 1 | strict-adjacent monotone from step 2 onward |

Force-on-soft summed `+z`-component is in `+z` direction (probe enters from above; wrap-cap material pushed UP) and grows monotonically with deeper penetration as more wrap-cap material engages. Strict-adjacent monotonicity at step 1 → step 2 is NOT required: contact engagement at the shallowest 0.5 mm penetration is in a transient regime where the active-pair set is still settling.

**Sign-convention note (inherited from row 22).** This row uses the `referenced-only` filtering convention — the unfiltered `per_pair_readout` includes ORPHAN BCC vertices inside the empty cavity (no FEM stiffness, ignored by the solver) and would dominate the sum with `-z` normal components, flipping the sign. `sim_soft::filter_pair_readouts_to_referenced` is the helper that enforces the physically meaningful reading. See pattern (xx) at the row 22 patterns memo.

### 8. `per_step_strain_energy_ordering`

| Anchor | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict, per step (16) |
| `Ψ̄_middle > Ψ̄_outer` | strict, per step (16) |

Same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 22's anchor 8, but applied at every ramp step over the deeper 8-mm regime. The compounding (compliance + distance-to-load + distance-to-constraint) holds throughout. Final-step values: `Ψ̄_inner ≈ 606 J/m³`, `Ψ̄_middle ≈ 295 J/m³`, `Ψ̄_outer ≈ 113 J/m³` — all roughly 2-3× row 22's 6-mm values, the extra ~40 % beyond depth-doubling coming from Yeoh's high-strain stiffening.

### 9. `per_step_max_disp_bounded`

| Anchor | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (16) |

Body-wide max displacement must stay under the 14 mm wrap thickness at every step (the cavity wall reaching the outer envelope's Dirichlet pin band would mean wrap collapse). Final-step max_disp ≈ 8.45 mm, ~60 % of the bound.

### 10. `material_provenance` + `material_assignment_partition`

| Material | (μ, C₂, λ) | Identity at `epsilon = 0.0` |
|---|---|---|
| Inner (Path 2)             | μ = 18 kPa, C₂ = 1.69 kPa, λ = 72 kPa  | `yr.energy(I) == 0` and `yr.energy(F_probe) == nh_part + C₂(I₁−3)²` (additive decomposition per arc memo F1 Spike-1) |
| `DRAGON_SKIN_10A` (middle) | μ = 51 kPa, C₂ = 4.46 kPa, λ = 204 kPa | same |
| `DRAGON_SKIN_20A` (outer)  | μ = 113 kPa, C₂ = 10.0 kPa, λ = 452 kPa | same |

Plus per-tet material assignment via `mesh.materials()[t].energy(F_probe)` matches the centroid's shell lookup at `EXACT_TOL = 0.0`. Plus the inner layer's `ConstructionSource::Interpolated { weight: 1.0 }` provenance tag is asserted. Same anchors as row 22's group 10, lifted to Yeoh.

### 11. `n_contact_pairs_final_exact` + `outer_layer_max_psi_final`

| Anchor | Pinned |
|---|---|
| `n_active_pairs` at final step (referenced-only) | `50` |
| `max Ψ_outer` at final step | bits self-pinned (~49 205 J/m³); rel-tol IV-1 sparse-tier |

Pair count grows through the ramp from 9 at step 1 to 50 at step 16 (vs row 22's 37 at step 12 / 6 mm) — the deeper probe pose progressively engages more wrap-cap material. Max Ψ_outer at 8 mm depth is ~5× row 22's 10 487 J/m³ at 6 mm — strain at the contact-band-adjacent outer-shell tets concentrates dramatically as Yeoh's `C₂(I₁−3)²` term contributes nonlinear stiffening at the extra 2 mm of depth.

### 12. `per_step_captured_bits` — IV-1 sparse-tier rel-tol

Per-step force-displacement + per-layer Ψ̄ aggregates self-pinned at first capture, 5 quantities × 16 steps = **80 captured-bit anchors** (plus the final-step `MAX_PSI_OUTER_FINAL_REF_BITS` from anchor 11):

- `FORCE_TOTAL_Z_RAMP_REF_BITS[16]` — `+z`-force-on-soft summed (referenced-only); approximate values `[1.10, 1.89, 2.97, 4.20, 5.64, 7.35, 9.44, 11.92, 14.72, 17.89, 21.49, 25.60, 30.27, 35.51, 41.79, 49.28] N`. Force is in `+z` direction (wrap-cap material pushed UP by the probe) and monotone-growing. The past-6 mm tail (steps 13-16) climbs faster than linear extrapolation from row 22's NH curve — Yeoh's `C₂(I₁−3)²` stiffening engages.
- `MAX_DISP_RAMP_REF_BITS[16]` — body-wide max displacement; approximate values `[1.48, 1.97, 2.47, 2.95, 3.44, 3.93, 4.41, 4.88, 5.35, 5.82, 6.27, 6.72, 7.15, 7.57, 8.00, 8.45] mm`.
- `MEAN_PSI_INNER_RAMP_REF_BITS[16]` — inner-shell mean Ψ.
- `MEAN_PSI_MIDDLE_RAMP_REF_BITS[16]` — middle-shell mean Ψ.
- `MEAN_PSI_OUTER_RAMP_REF_BITS[16]` — outer-shell mean Ψ.

Compared via `assert_relative_eq!` at `SPARSE_REL_TOL = 1e-12` rel + `SPARSE_EPS_ABS = 1e-12` floor. **Failure-mode protocol per IV-1**: if a rel-tol comparison fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture); (2) if same toolchain, real regression in cf-design's `cuboid` / `offset` plumbing OR in sim-soft's BCC + IS + faer hot path OR in the chained-`replay_step` path OR in Yeoh's `energy` / `first_piola` / `tangent` arithmetic OR in the inline `deformation_gradient` arithmetic; (3) NEVER re-bake to silence drift.

`CF_CAPTURE_BITS=1` env-var bootstrap pattern (banked at row 19 as pattern (cc)): when set, every captured-anchor check is bypassed and a paste-ready capture block is printed to stderr. Use for first-time author-bake and intentional re-bake; never for failure silencing.

## Visuals

`out/sleeve_zslab_final.ply` — z-slab per-tet centroid cloud at the FINAL ramp step (depth = 8 mm) (`2_092` centroids in `|rest_centroid.z| < CELL_SIZE / 2 = 0.002 m`) with `DISPLACEMENT_SCALE = 10.0` geometric amplification on positions (`amplified = rest_centroid + SCALE * (deformed_centroid - rest_centroid)`). Same scale factor as row 22; row 23's true displacements are ~26 % larger (8.45 mm vs 6.7 mm peak). Two scalars: categorical `material_id` (0 = inner / 1 = middle / 2 = outer) + sequential `displacement_magnitude` (true physical magnitude, unscaled).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_zslab_final.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** is binary-categorical (3 distinct values 0/1/2) → cf-view picks the **categorical palette**.
- **`displacement_magnitude`** is unipolar continuous → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D annulus on `z = 0`, 40 mm BELOW the contact zone at `z ≈ 0.040 m`. Same z-slab axis as rows 21 + 22; the slab catches the propagated response of the wrap shell, NOT the contact-zone signal directly.

**Force-displacement curve via matplotlib.** The `ramp_curve` array in `out/scan_fit_3layer_sleeve_yeoh_ramp.json` carries the per-step force / displacement / Ψ̄ trace. Optional matplotlib post-processing via PEP 723 inline metadata:

```sh
uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/plot_ramp.py
```

Produces a dual-axis plot of penetration depth (mm) × force_total_z (N) on the left axis and depth × max_disp (mm) on the right axis, with Newton iter counts annotated above each force-curve point. Force_total_z grows smoothly + super-linearly from `~1.1 N` at 0.5 mm to `~49.3 N` at 8 mm (per-step growth `+0.78 → +1.07 → +1.22 → ... → +7.49 N` is monotone-increasing, with the `+7.49 N` jump at step 16 reflecting Yeoh's high-strain stiffening). Newton iter counts climb visibly across the past-NH-wall steps 13-16 (31 → 39 → 49 → 77) — the deep-penetration regime's signature lives in iter-count growth.

**Note on `max_disp` vs depth.** `max_disp` exceeds the rigid `probe penetration depth` at the shallowest steps but the ratio shrinks through the ramp: `~3×` at 0.5 mm to `~1.06×` at 8 mm. Penalty contact's elastic equilibrium can push the cavity wall farther than the rigid penetration depth — same effect rows 21 + 22 documented. Yeoh's high-strain stiffening compresses this ratio at deeper poses since the body becomes harder to push past the rigid penetration target.

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-ramp --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve. The 16-step ramp at ~75 k tets through faer's sparse Cholesky takes ~40-90 s release; debug mode would take many minutes per step × 16 steps. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness; finer cells (e.g. `0.002 m`) trip an SPD pivot at the FIRST ramp step (empirically tested at row-22 v2-spec spike time, applies to row 23 by inheritance — same mesh + meshing pipeline).

## First-time bit capture

```sh
CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-ramp --release
```

Emits a paste-ready block of every `*_EXACT` count, `IT_COUNT_RAMP_EXACT` array, every `*_RAMP_REF_BITS` array, and `MAX_PSI_OUTER_FINAL_REF_BITS`, bypassing the captured-anchor checks. Use for first-time author-bake and intentional re-bake; the IV-1 protocol forbids using this to silence a drift assertion.

## Roadmap (followups, not in row-23 scope)

This row realises the v? "8 mm intrusion via wider-validity hyperelastic" entry from row 22's roadmap (then provisionally Mooney-Rivlin, pivoted to Yeoh after the arc-memo §"Mooney-Rivlin rejection" math falsification). Next steps in the queued evolution toward iter-2+ silicone-device design support:

- **v3 (axial zoned variation)** — proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`. Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4 (explicit Cu mesh sub-layer)** — 4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A.
- **v5+ (3-param Yeoh with measured C₃)** — when post-cast Fork-B calibration data lands, switch from 2-param to 3-param Yeoh per the U1 future-proofing note in the arc memo. Typical silicone fits give `C₂ < 0` + `C₃ > 0` (intermediate-strain softening + high-strain stiffening) that the current 2-param model can't represent. The `c3: Option<f64>` field in `SiliconeMaterial` is already in place.
- **vN — real anatomy scan replacing the cuboid fixture** (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
