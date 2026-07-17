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

Yeoh has per-anchor **asymmetric** validity bounds: `max_principal_stretch = 0.8·λ_break ≈ 5.8-8.8` for the contact-zone shells (per-family tensile cap, calibrated from Smooth-On TDS elongation-at-break) and `min_principal_stretch = 0.20` (engineering-aggressive default — no published Yeoh-on-silicone compression bound exists per the arc memo §A2 web-search recon). These caps are **routed into the per-tet materials** via `MaterialField::from_yeoh_fields_with_bounds` (see the code example below); that is what makes the deep-penetration solve robust. The contact-zone tet that grazes σ ≈ 2.0 at the 8 mm target sits at ~2.9-3.8× margin under its shell's real ~5.8-8.8 cap.

> **Why the bounds matter (and a footgun this row fixes).** The bounds-*less* `MaterialField::from_yeoh_fields` leaves each per-tet Yeoh's validity `None`, so it silently falls through to the **legacy Neo-Hookean symmetric ceiling** `max_stretch_deviation = 1.0` (σ ∈ [0, 2]) — ~3× tighter than silicone's real envelope, and a *Neo-Hookean* gate on a *Yeoh* body. With the σ=2.0 placeholder the 8 mm solve grazes the ceiling near step 16 and fail-closes (it landed at σ ≈ 1.9998 on the authoring toolchain and tipped to σ ≈ 2.0002 under a later rustc minor bump). Routing the calibrated caps via `from_yeoh_fields_with_bounds` — the constructor that closes the "MaterialField-drops-bounds" gap — is the fix. The bound governs only the fail-closed gate, not the energy/force response, so the converged 8 mm solution is unchanged (force_z = 49.3 N, 77 iters, residual 9.87e-11).

## Why a ramp, not a single step

Same rationale as row 22 inherited from row 21: deeper penetration single-stepped on this geometry pushes the iter-0 penalty gradient out of Newton's basin (single-step failure modes were directly observed at related configs — v1.5's capsule + capsule at 4 mm depth tripped a non-PD pivot at iter 3; the earlier-pivot 459 K-tet 2 mm-cell experiment at 8 mm depth inverted tets in the first Newton step). The quasi-static ramp circumvents the single-step basin issue: each step's `x_prev` is the previous step's `x_final`, so the iter-0 gradient is bounded by the per-step delta only.

This row uses the same **quasi-static `replay_step` chaining** pattern row 22 introduced (`x_prev_k+1 = x_final_k`, `v_prev = 0` throughout, no inertia carry-over) — the only differentiators are the Yeoh material model and the extended depth.

Every gate sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. This row is a Rule-B **validator**: its gates are pipeline-emergent structural + physics invariants read from the real 16-step ramp, not captured-bit self-pins (those were stripped in the Rule-B de-frag — constitutive and mesher correctness are lib-owned). Outputs are `out/scan_fit_3layer_sleeve_yeoh_ramp.json` (final-step scalars + per-shell Yeoh `(μ, C₂, λ)` + per-step `ramp_curve` array + per-active-contact-pair detail at the final step), `out/sleeve_boundary_final.ply` + `out/sleeve_design_slab_cut_z0_final.ply` + `out/sleeve_design_surface_deformed_step_01..16.ply` (F1.5 viz: full 3D body, equatorial cross-section, and per-step deformed design surfaces via `sim_soft::viz`), and `out/ramp_curve.png` (from `plot_ramp.py`).

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
// Two more partition fields carry the per-anchor calibrated validity caps
// (0.8·λ_break tensile, 0.20 compressive) — without them the deep solve
// fail-closes on the legacy NH σ=2.0 ceiling near 8 mm (see "Why Yeoh").
let max_stretch_field = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
                       [inner.validity_max_principal_stretch, DRAGON_SKIN_10A.validity_max_principal_stretch,
                        DRAGON_SKIN_20A.validity_max_principal_stretch]);
let min_stretch_field = LayeredScalarField::new(scan, [LAYER_INNER, LAYER_MIDDLE_OUTER],
                       [inner.validity_min_principal_stretch, DRAGON_SKIN_10A.validity_min_principal_stretch,
                        DRAGON_SKIN_20A.validity_min_principal_stretch]);
let material_field = MaterialField::from_yeoh_fields_with_bounds(
    Box::new(mu_field), Box::new(c2_field), Box::new(lambda_field),
    Box::new(max_stretch_field), Box::new(min_stretch_field));

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

The Yeoh material model composes through the F4.0 generic-mesh refactor without any per-row plumbing: `Mesh<M = NeoHookean>` defaults to NH for legacy consumers, this row writes `<Yeoh>` explicitly to pick the Yeoh slot. Future Yeoh consumers (rows 24+ in the projected ladder) reuse the same constructor pair (`from_yeoh_fields_with_bounds` + `from_sdf_yeoh`) and solver alias (`PenaltyRigidContactYeohSolver`).

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material Yeoh FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each gate is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*` and is called from `main()` in dependency order; `cargo run --release` exit-0 means every gate passed. They are **pipeline-emergent structural + physics invariants** read from the real 16-step ramp — resolution- and toolchain-robust. The pre-Rule-B captured-bit self-pins (exact tet/vertex/pair counts, the per-step iter-count freeze, the 80 per-step + final `to_bits()` force/displacement/Ψ̄ pins, the `to_yeoh()` additive-decomposition provenance mirror) were **stripped**: they froze one run's FP trajectory on one toolchain, and the constitutive correctness they redundantly implied is lib-owned (`yeoh_contract.rs` closed-form + additive-decomposition + `silicone_table.rs::tests::to_yeoh_round_trips_yeoh_fields_for_each_anchor` + the inner layer's `from_effective_shore` weight-1.0 provenance test), as is the generic distance-from-scan shell routing (`sdf_material_tagging.rs` IV-4). The observed per-step scalars (force, displacement, per-layer Ψ̄) are still emitted to the JSON `ramp_curve` + stdout for eyes-on inspection — just not self-pinned.

### 1. `quality_floors`

| Gate | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same gate as row 22. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

### 2. `mesh_structure`

| Gate | Bound |
|---|---|
| `n_tets` | `> 0` |
| `n_referenced` | `> 0` and `≤ n_vertices` (referenced ⊆ all vertices) |
| `n_pinned` (outer-envelope band) | non-empty proper subset of `n_referenced` |
| each per-shell tet count | `> 0` (all three material bands populated) |

Structural invariants, not exact counts (the specific tet/vertex counts are a mesher-version artifact). Per-shell routing correctness is checked in gate 9 + owned by lib IV-4.

### 3. `zslab_populations`

| Gate | Bound |
|---|---|
| each z-slab per-shell tet count (`\|cz\| < CELL_SIZE / 2 = 0.002`) | `> 0` |

Each material band contributes ≥ 1 tet to the `z = 0` cut so the viz cross-section shows all three shells. Non-empty, not exact-count.

### 4. `per_step_solver_converges`

| Gate | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 150` | per step, all 16 |
| `r.final_residual_norm < 1e-10` | per step, all 16 |

Each step's Newton solve must converge within the iter cap + tolerance bound. Observed iter counts climb `[8 … 27, 31, 39, 49, 77]` — step 16 (the deepest at 8 mm) is the closest to the cap with **73 iters of margin**. (Robust convergence at 8 mm depends on the calibrated Yeoh validity bounds — see "Why Yeoh" above; without them the solve fail-closes on the legacy NH σ=2.0 ceiling near step 16.)

### 5. `force_displacement_monotone`

| Gate | Bound |
|---|---|
| `force_total_z[N-1] > force_total_z[0]` | strict (ramp endpoint sanity) |
| `force_total_z[k+1] > force_total_z[k]` for k ≥ 1 | strict-adjacent monotone from step 2 onward |

Force-on-soft summed `+z`-component is in `+z` direction (probe enters from above; wrap-cap material pushed UP) and grows monotonically with deeper penetration as more wrap-cap material engages (observed: `~1.1 N` at 0.5 mm → `~49.3 N` at 8 mm). Strict-adjacent monotonicity at step 1 → step 2 is NOT required: contact engagement at the shallowest 0.5 mm penetration is a transient regime where the active-pair set is still settling.

**Sign-convention note (inherited from row 22).** This row uses the `referenced-only` filtering convention — the unfiltered `per_pair_readout` includes ORPHAN BCC vertices inside the empty cavity (no FEM stiffness, ignored by the solver) and would dominate the sum with `-z` normal components, flipping the sign. `sim_soft::filter_pair_readouts_to_referenced` enforces the physically meaningful reading. See pattern (xx) at the row 22 patterns memo.

### 6. `per_step_strain_energy_ordering`

| Gate | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict, per step (16) |
| `Ψ̄_middle > Ψ̄_outer` | strict, per step (16) |

The same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 22, applied at every ramp step over the deeper 8-mm regime. The compounding (compliance + distance-to-load + distance-to-constraint) holds throughout. Observed final-step values: `Ψ̄_inner ≈ 606 J/m³`, `Ψ̄_middle ≈ 295 J/m³`, `Ψ̄_outer ≈ 113 J/m³`.

### 7. `per_step_max_disp_bounded`

| Gate | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (16) |

Body-wide max displacement must stay under the 14 mm wrap thickness at every step (the cavity wall reaching the outer envelope's Dirichlet pin band would mean wrap collapse). Observed final-step max_disp ≈ 8.45 mm, ~60 % of the bound.

### 8. `contact_engaged`

| Gate | Bound |
|---|---|
| `n_active_pairs` at final step | `> 0` |

The final (deepest) ramp step engaged the probe against the cavity wall — at least one active referenced-vertex contact pair. (The exact pair count was a mesher/discretization artifact; non-empty is the invariant that matters.)

### 9. `material_routing`

| Gate | Bound |
|---|---|
| `mesh.materials()[t]` per tet | `(μ, C₂, λ)` bit-equal to the shell's F4 Yeoh entry, each shell exercised ≥ 1 tet |

The scene's multi-material routing invariant — the `MaterialField` assigned every tet the `(μ, C₂, λ)` of the shell its centroid falls in. Reads the real per-tet `Yeoh` from `mesh.materials()` via the public `.mu()` / `.c2()` / `.lambda()` accessors and compares to the shell's F4 entry (`inner_silicone()` / `DRAGON_SKIN_10A` / `DRAGON_SKIN_20A`, all `.to_yeoh()`) with an exact `to_bits()` `==` — a routing check, NOT a constitutive-arithmetic mirror. The Yeoh closed form + additive decomposition + the `to_yeoh()` round-trip are lib-owned (`yeoh_contract.rs` + `silicone_table.rs::tests::to_yeoh_round_trips_yeoh_fields_for_each_anchor`), as is the inner layer's Path-2 `ConstructionSource::Interpolated { weight: 1.0 }` provenance (`..._from_effective_shore_at_anchor_position_returns_anchor_data`); the generic routing mechanism is lib-owned (`sdf_material_tagging.rs` IV-4).

## Visuals

Post-F1.5-viz-retrofit, the final step (depth = 8 mm) emits proper triangulated viz artifacts via `sim_soft::viz` (not the pre-retrofit z-slab centroid cloud): `out/sleeve_boundary_final.ply` (full 3D body via `boundary_surface`), `out/sleeve_slab_cut_z0_final.ply` (equatorial cross-section at `z = 0` via `slab_cut`, 40 mm below the contact zone — catches the propagated wrap-shell response), plus the cf-design design-mesh emits `out/sleeve_design_slab_cut_z0_final.ply` and the per-step deformed-surface sequence `out/sleeve_design_surface_deformed_step_01..16.ply` (scrubbable in cf-view). Each carries a categorical `material_id` (0 = inner / 1 = middle / 2 = outer → categorical palette) + a sequential `displacement_magnitude` (unipolar → viridis), per pattern (u).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_boundary_final.ply
```

(An eyes-on-pixels review of the retrofit artifacts is a user-side pass — the boundary/slab-cut meshes replaced the pre-F1.5 centroid cloud this section originally described.)

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


## Roadmap (followups, not in row-23 scope)

This row realises the v? "8 mm intrusion via wider-validity hyperelastic" entry from row 22's roadmap (then provisionally Mooney-Rivlin, pivoted to Yeoh after the arc-memo §"Mooney-Rivlin rejection" math falsification). Next steps in the queued evolution toward iter-2+ silicone-device design support:

- **v3 (axial zoned variation)** — proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`. Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4 (explicit Cu mesh sub-layer)** — 4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A.
- **v5+ (3-param Yeoh with measured C₃)** — when post-cast Fork-B calibration data lands, switch from 2-param to 3-param Yeoh per the U1 future-proofing note in the arc memo. Typical silicone fits give `C₂ < 0` + `C₃ > 0` (intermediate-strain softening + high-strain stiffening) that the current 2-param model can't represent. The `c3: Option<f64>` field in `SiliconeMaterial` is already in place.
- **vN — real anatomy scan replacing the cuboid fixture** (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
