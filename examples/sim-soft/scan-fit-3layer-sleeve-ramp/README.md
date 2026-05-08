# scan-fit-3layer-sleeve-ramp

**Row 22 — Tier 6 synthesis #3, a multi-step quasi-static intrusion ramp evolution of [row 21 `scan-fit-3layer-sleeve`](../scan-fit-3layer-sleeve/).** Where row 21 v1 computes a single static fit pose at 1 mm probe penetration, this row chains 12 backward-Euler `replay_step` calls in a ramp from rest to **6 mm** penetration (`0.5 mm/step`), bounding each step's Newton iter-0 penalty gradient by the per-step delta only — NOT the full target overlap from rest. Same single-step solver, same SPD-skeleton path, same `STATIC_DT = 1.0 s` collapse of the inertial term. Each step is its own converged static equilibrium; the ramp sweeps the rigid-primitive geometry between converged equilibria.

Every architectural piece is inherited verbatim from row 21 v1 (cuboid scan + sphere probe + z-slab + 6/4/4 mm layered material partition + `CELL_SIZE = 4 mm` + 18-113 kPa ECOFLEX_00_20 / DRAGON_SKIN_10A / DRAGON_SKIN_20A silicone stack). The only differentiator is the ramp.

| Layer | Thickness (v1+v2) | Hardware target | Source material | F4 proxy | Engineering role |
|---|---|---|---|---|---|
| Inner  | 6 mm | 6 mm | Ecoflex 00-30 + 75 % Slacker (effective Shore 00-20) | `ECOFLEX_00_20`   (μ = 18 kPa, λ = 72 kPa)   | skin-contact softness |
| Middle | 4 mm | 5 mm | DS10A + Cu mesh + carbon black (effective Shore 15-18A) | `DRAGON_SKIN_10A` (μ = 51 kPa, λ = 204 kPa)  | conductive composite |
| Outer  | 4 mm | 3 mm | DS20A direct                                      | `DRAGON_SKIN_20A` (μ = 113 kPa, λ = 452 kPa) | structural stiffness |

[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
[v2spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_22_v2_spec.md
[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Why a ramp, not a single step

Row 21 v1's static fit pose at 1 mm penetration is the canonical single-step demonstration of the pipeline. Deeper penetration single-stepped on this geometry is expected to push the iter-0 penalty gradient out of Newton's basin — single-step failure modes were directly observed at related configs (v1.5's capsule + capsule at 4 mm depth tripped a non-PD pivot at iter 3; the earlier-pivot 459 K-tet 2 mm-cell experiment at 8 mm depth inverted tets in the first Newton step), and v2's own ramp observed Armijo line-search collapse at 6.5 mm depth even with a 6.0 mm pre-converged `x_prev` (see [v2 spec memo][v2spec]). The quasi-static ramp circumvents the single-step basin issue — each step's `x_prev` is the previous step's `x_final` (cavity wall already deformed to the previous step's equilibrium), so the iter-0 penalty gradient is bounded by the per-step delta only, not the full target overlap.

The ramp's empirical reach was characterized at v2-spec lock time via five pre-execution spike runs (Runs 1–3 swept step-size + iter-cap; Spike A + Spike B then probed mesh resolution + stiffness gradient — see [v2 spec memo][v2spec]). The wall lies at `max_disp ≈ 7 mm`, where Newton's tangent matrix becomes near-singular and Armijo line-search hits its hardcoded `α_min = 2⁻²¹` floor. **6 mm penetration is this row's target — the deepest reach with comfortable solver margin** (step 12 converges in 61 Newton iters at `MAX_NEWTON_ITER = 100`, leaving a 39-iter safety margin). The user-target 8 mm physical intrusion is structurally unreachable on this contact configuration; it is deferred to a v3 followup that requires either solver-side faer LU fallback OR contact-geometry change (probe placement / shape).

This row is also the **first sim-soft user-facing example to demonstrate quasi-static `replay_step` chaining** — `x_prev_k+1 = x_final_k`, `v_prev = 0` throughout (no inertia carry-over). Distinct from row 17 [`soft-drop-on-plane`](../soft-drop-on-plane/), which chains the full `step` solver method for transient dynamics with inertial damping; v2's chaining is the quasi-static-equilibrium variant. The pattern generalises to any scenario where the single-step Newton basin would otherwise collapse.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Outputs are `out/scan_fit_3layer_sleeve_ramp.json` (final-step scalars + 3-material provenance + per-step `ramp_curve` array + per-active-contact-pair detail at the final step) and `out/sleeve_zslab_final.ply` (z-slab per-tet centroid cloud at the final step, with categorical `material_id` + sequential `displacement_magnitude`).

## What this example demonstrates

**Workflow** (the architectural payload — the multi-step ramp generalises to any quasi-static deep-penetration scenario):

```rust
// 1-5. Build the sleeve body + material field + initial mesh + verifies.
//      Same as row 21 v1.
let scan_solid = Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, SCAN_HZ));
let outer_envelope = scan_solid.clone().offset(WRAP_THICKNESS);
let sleeve_body = outer_envelope.clone().subtract(scan_solid.clone());
// (3-layer MaterialField via F4; SdfMeshedTetMesh::from_sdf via F1+F3.)

// 6. Quasi-static intrusion ramp — chained replay_step over 12 steps.
let mut x_prev_flat = rest_positions_as_flat_vec();
for k in 0..N_RAMP_STEPS {
    let depth = (k + 1) as f64 * RAMP_STEP_DELTA; // 0.5 mm per step
    let probe_k = build_probe_solid_at_depth(depth);
    let contact_k = PenaltyRigidContact::new(vec![probe_k]);
    // Mesh + bc rebuilt per step (deterministic; same SDF + hints).
    let solver_k = CpuNewtonSolver::new(/* ..., contact_k, ... */);
    let step_k = solver_k.replay_step(
        &Tensor::from_slice(&x_prev_flat, &[n_dof]),
        &Tensor::zeros(&[n_dof]),  // v_prev = 0 (quasi-static)
        &empty_theta,
        STATIC_DT,
    );
    // Capture per-step readouts (force_z, max_disp, per-layer Ψ̄, ...).
    results.push(/* RampStepResult */);
    x_prev_flat.clone_from(&step_k.x_final);  // chain
}

// 7. Per-step + final-step verifies (12 anchor groups, see "Numerical anchors").

// 8. JSON + PLY readouts. Final step drives the headline z-slab artifact;
//    the JSON `ramp_curve` array carries the full force-displacement trace
//    for matplotlib post-processing.
```

The ramp pattern composes for any scenario where the single-step Newton basin is too tight. Future rows can reuse `solve_ramp` directly; the only per-row choice is `(N_RAMP_STEPS, PROBE_PENETRATION_FINAL, MAX_NEWTON_ITER)` — all three set empirically from a pre-execution spike on the target geometry.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 12 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `quality_floors`

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same anchor as row 21 v1. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

### 2. `counts_exact`

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `74_628` | bit-equal to row 21 v1 (geometry + BCC + IS deterministic on same SDF + hints) |
| `n_vertices`          | `31_966` | bit-equal to row 21 v1 |
| `n_referenced`        | `17_384` | bit-equal to row 21 v1 |
| `n_pinned` (outer-envelope band) | `7_046` | bit-equal to row 21 v1 |
| `n_inner_tets` (`ECOFLEX_00_20`)   | `25_892` | bit-equal to row 21 v1 |
| `n_middle_tets` (`DRAGON_SKIN_10A`)| `16_656` | bit-equal to row 21 v1 |
| `n_outer_tets` (`DRAGON_SKIN_20A`) | `32_080` | bit-equal to row 21 v1 |
| sum                                | `74_628` | partition gate |

Cross-row continuity to row 21 v1 IS the gate (pattern (y) — bit-equal cross-row continuity captures regressions; row 22 + row 21 v1 share geometry + material stack exactly, so counts are bit-equal).

### 3. `zslab_counts_exact`

| Count | Pinned |
|---|---|
| `n_inner_tets_zslab`  | `768` (per-shell partition of the z-slab cut at `\|cz\| < CELL_SIZE / 2 = 0.002`) |
| `n_middle_tets_zslab` | `432` |
| `n_outer_tets_zslab`  | `892` |

Bit-equal to row 21 v1 (z-slab geometry + cut axis are unchanged).

### 4. `n_ramp_steps_exact`

| Anchor | Bound |
|---|---|
| `len(results) == N_RAMP_STEPS_EXACT` | strict (12) |

Partition gate — the ramp must produce exactly 12 step results.

### 5. `per_step_solver_converges`

| Anchor | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 100` | per step, all 12 |
| `r.final_residual_norm < 1e-10` | per step, all 12 |

Each step's Newton solve must converge within the iter cap + tolerance bound. Empirical iter counts: `[8, 8, 9, 11, 11, 13, 14, 16, 19, 22, 30, 61]` — step 12 (the deepest at 6 mm) is the closest to the cap with 39 iters of margin.

### 6. `per_step_iter_count`

| Anchor | Pinned |
|---|---|
| `IT_COUNT_RAMP_EXACT[k] == r.iter_count` per step | `[8, 8, 9, 11, 11, 13, 14, 16, 19, 22, 30, 61]` |

The chained `replay_step` is deterministic on a fixed toolchain (rustc 1.95.0 on macOS arm64); iter-count drift signals real solver-path regression, not noise. Tighter than `per_step_solver_converges` (anchor 5) — that gate accepts any iter count below the cap; this gate pins to specific values.

### 7. `force_displacement_monotone`

| Anchor | Bound |
|---|---|
| `force_total_z[N-1] > force_total_z[0]` | strict (ramp endpoint sanity) |
| `force_total_z[k+1] > force_total_z[k]` for k ≥ 1 | strict-adjacent monotone from step 2 onward |

Force-on-soft summed `+z`-component is in `+z` direction (probe enters from above; wrap-cap material is pushed UP) and grows monotonically with deeper penetration as more wrap-cap material engages. Strict-adjacent monotonicity at step 1 → step 2 is NOT required: contact engagement at the shallowest 0.5 mm penetration is in a transient regime where the active-pair set is still settling. From step 2 onward the gate IS strict (any inversion in the deeper regime would signal a real defect).

**v2.5 sign-flip note.** Pre-v2.5 anchors had `force_total_z` in the `-z` direction (e.g., `-137 N` at step 1) because the unfiltered `per_pair_readout` includes ORPHAN BCC vertices inside the empty cavity — orphans below the probe equator have `-z` normals and dominated the sum (~95-97% of readout entries were orphans). v2.5 filters readouts to referenced (= solver-active) vertices only, surfacing the physically meaningful `+z` push from wrap-cap-above-probe contacts. See pattern (xx) at the row 22 patterns memo and the v2 spec memo's "Post-ship investigation" section for the orphan-pollution discovery and v2.5 cleanup rationale.

### 8. `per_step_strain_energy_ordering`

| Anchor | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict, per step (12) |
| `Ψ̄_middle > Ψ̄_outer` | strict, per step (12) |

Same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 21 v1's anchor 7, but applied at every ramp step. The compounding (compliance + distance-to-load + distance-to-constraint) holds throughout the ramp, so the ordering is robust at every intermediate equilibrium. Final-step values: `Ψ̄_inner ≈ 302 J/m³`, `Ψ̄_middle ≈ 102 J/m³`, `Ψ̄_outer ≈ 30 J/m³`.

### 9. `per_step_max_disp_bounded`

| Anchor | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (12) |

Body-wide max displacement must stay under the 14 mm wrap thickness at every step (the cavity wall reaching the outer envelope's Dirichlet pin band would mean wrap collapse). Final-step max_disp ≈ 6.7 mm, ~48 % of the bound.

### 10. `material_provenance` + `material_assignment_partition`

| Material | Lamé pair | Identity at `epsilon = 0.0` |
|---|---|---|
| `ECOFLEX_00_20` (inner)   | μ = 18 kPa, λ = 72 kPa  | `nh.energy(I) == 0` and `nh.energy(F_probe) == FMA(λ/2, ...)` |
| `DRAGON_SKIN_10A` (middle)| μ = 51 kPa, λ = 204 kPa | same |
| `DRAGON_SKIN_20A` (outer) | μ = 113 kPa, λ = 452 kPa | same |

Plus per-tet material assignment via `mesh.materials()[t].energy(F_probe)` matches the centroid's shell lookup at `EXACT_TOL = 0.0`. Same anchors as row 21 v1's anchors 5 + 6.

### 11. `n_contact_pairs_final_exact` + `outer_layer_max_psi_final`

| Anchor | Pinned |
|---|---|
| `n_active_pairs` at final step (referenced-only, v2.5) | `37` |
| `max Ψ_outer` at final step | bits self-pinned (~10487 J/m³); rel-tol IV-1 sparse-tier |

Pre-v2.5 the anchored `n_active_pairs` was 273 — but ~95-97 % of those were ORPHAN BCC lattice vertices inside the empty cavity (no FEM stiffness, ignored by the solver). v2.5 filters to referenced vertices only; the count is now physically meaningful. Real pair count GROWS from 9 at 0.5 mm penetration to 37 at 4 mm and PLATEAUS at 37 from step 7 onward — the deepening probe pose progressively engages more wrap-cap material until the active band stabilizes. Max Ψ_outer at 6 mm depth is ~60× row 21 v1's 175.8 J/m³ at 1 mm — strain at the contact-band-adjacent outer-shell tets concentrates dramatically as the radial chain inner → middle → outer transmits the deeper probe load. (Row 21 v1's anchored 294-pair count is also orphan-polluted; preserved as-is for cross-row continuity at the unfiltered convention.)

### 12. `per_step_captured_bits` — IV-1 sparse-tier rel-tol

Per-step force-displacement + per-layer Ψ̄ aggregates self-pinned at first capture, 5 quantities × 12 steps = 60 captured-bit anchors (plus the final-step `MAX_PSI_OUTER_FINAL_REF_BITS` from anchor 11):

- `FORCE_TOTAL_Z_RAMP_REF_BITS[12]` — `+z`-force-on-soft summed (referenced-only post-v2.5); values `~[1.1, 1.9, 2.9, 4.2, 5.6, 7.2, 9.2, 11.5, 14.1, 16.9, 20.0, 23.1] N`. Force is in `+z` direction (wrap-cap material pushed UP by the probe) and monotone-growing. Pre-v2.5 these were `~[-137, -131, ..., -1135] N` — orphan-driven sign reversal documented above.
- `MAX_DISP_RAMP_REF_BITS[12]` — body-wide max displacement; values `~[1.5, 2.0, 2.5, 3.0, 3.4, 3.9, 4.4, 4.9, 5.4, 5.8, 6.3, 6.7] mm`.
- `MEAN_PSI_INNER_RAMP_REF_BITS[12]` — inner-shell mean Ψ.
- `MEAN_PSI_MIDDLE_RAMP_REF_BITS[12]` — middle-shell mean Ψ.
- `MEAN_PSI_OUTER_RAMP_REF_BITS[12]` — outer-shell mean Ψ.

Compared via `assert_relative_eq!` at `SPARSE_REL_TOL = 1e-12` rel + `SPARSE_EPS_ABS = 1e-12` floor. **Failure-mode protocol per IV-1**: if a rel-tol comparison fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture); (2) if same toolchain, real regression in cf-design's `cuboid` / `offset` plumbing OR in sim-soft's BCC + IS + faer hot path OR in the chained-`replay_step` path OR in the inline `deformation_gradient` arithmetic; (3) NEVER re-bake to silence drift.

`CF_CAPTURE_BITS=1` env-var bootstrap pattern (banked at row 19 as pattern (cc)): when set, every captured-anchor check is bypassed and a paste-ready capture block is printed to stderr. Use for first-time author-bake and intentional re-bake; never for failure silencing.

## Visuals

`out/sleeve_zslab_final.ply` — z-slab per-tet centroid cloud at the FINAL ramp step (depth = 6 mm) (`2_092` centroids in `|rest_centroid.z| < CELL_SIZE / 2 = 0.002 m`) with `DISPLACEMENT_SCALE = 10.0` geometric amplification on positions (`amplified = rest_centroid + SCALE * (deformed_centroid - rest_centroid)`). Lower than row 21 v1's `50×` so the rendered displacement-field magnitude stays visually comparable to v1's at the same body-scale: v1's `50× × 1.97 mm peak ≈ 99 mm` rendered; v2's `10× × 6.7 mm peak ≈ 67 mm` rendered — same order of magnitude even though v2's true displacements are ~3.4× larger. Two scalars: categorical `material_id` (0 = inner / 1 = middle / 2 = outer) + sequential `displacement_magnitude` (true physical magnitude, unscaled).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-ramp/out/sleeve_zslab_final.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** is binary-categorical (3 distinct values 0/1/2) → cf-view picks the **categorical palette**.
- **`displacement_magnitude`** is unipolar continuous → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D annulus on `z = 0`, 40 mm BELOW the contact zone at `z ≈ 0.040 m`. Same z-slab axis as row 21 v1; the slab catches the propagated response of the wrap shell, NOT the contact-zone signal directly.

**Regime transition vs row 21 v1.** Row 21 v1's [Visuals section](../scan-fit-3layer-sleeve/README.md#visuals) describes a peak-at-mid-thickness flexural-bending mode at 1 mm penetration: the inner cavity wall and outer envelope are both held near zero (cavity-wall material constrained from moving inward + outer envelope Dirichlet-pinned), so the wrap cross-section interior is the path of least resistance. v2's final-step PLY at 6 mm penetration shows a **qualitatively different** pattern — the displacement field is highest at the inner cavity wall (yellow band against the cavity opening in the viridis sequential render) and decays outward to the Dirichlet-pinned outer envelope (dark purple). The 6× deeper probe pose displaces the inner cavity wall at z=0 (40 mm below the contact zone) substantially through the propagated load chain, exiting the small-strain regime where v1's flexural mode dominated; v2's regime is "inner cavity wall pushed outward as a whole, decaying radially to the outer pin."

The xy-anisotropy v1 documented (longer ±y faces of the cuboid + offset deflect more than the shorter ±x faces) is faintly present in v2's final-step PLY but secondary to the dominant inner-wall-vs-outer-wall radial gradient — the regime change subordinates the flexural-bending detail to the gross radial mode.

**Force-displacement curve via matplotlib.** The `ramp_curve` array in `out/scan_fit_3layer_sleeve_ramp.json` carries the per-step force / displacement / Ψ̄ trace. Optional matplotlib post-processing via PEP 723 inline metadata (mirrors row 5's `plot.py`):

```sh
uv run examples/sim-soft/scan-fit-3layer-sleeve-ramp/plot_ramp.py
```

Produces a dual-axis plot of penetration depth (mm) × force_total_z (N) on the left axis and depth × max_disp (mm) on the right axis, with Newton iter counts annotated above each force-curve point. Force_total_z grows smoothly + convexly from `~1.1 N` at 0.5 mm to `~23 N` at 6 mm (post-v2.5; the per-step growth `+0.78 → +1.07 → +1.22 → ... → +3.19 N` is monotone-increasing, no sharp elbow). Newton iter counts (8 / 8 / 9 / 11 / 11 / 13 / 14 / 16 / 19 / 22 / 30 / 61) climb visibly across the ramp — the deep-penetration regime's signature lives in iter-count growth (not force-curve elbow), since the body stiffens through nonlinear NH approaching the validity-domain boundary.

(The pre-v2.5 plot showed an apparent sharp elbow at 4 → 4.5 mm — force jumping from `-462 N` to `-809 N`. That was an orphan-vertex artifact; the post-v2.5 referenced-only force trajectory has no such elbow. See pattern (xx) + the v2 spec memo's "Post-ship investigation" section.)

**Note on `max_disp` vs depth.** `max_disp` exceeds the rigid `probe penetration depth` at every step (`~1.5 mm` peak displacement at `0.5 mm` penetration; `~6.7 mm` at `6.0 mm`). Penalty contact's elastic equilibrium can push the cavity wall farther than the rigid penetration depth — same effect row 21 v1's anchor 8 prose documents at v1's `~1.97 mm` peak vs `1 mm` penetration. The amplification factor `max_disp / depth` shrinks from `~3×` at 0.5 mm to `~1.1×` at 6 mm as the body stiffens through the non-linear regime.

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-ramp --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve. The 12-step ramp at ~75 k tets through faer's sparse Cholesky takes ~30-60 s release; debug mode would take many minutes per step × 12 steps. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness; finer cells (e.g. `0.002 m`) trip an SPD pivot at the FIRST ramp step (denser per-cell penalty gradient — empirically tested at v2-spec spike time and rejected).

## First-time bit capture

```sh
CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-scan-fit-3layer-sleeve-ramp --release
```

Emits a paste-ready block of every `*_EXACT` count, `IT_COUNT_RAMP_EXACT` array, every `*_RAMP_REF_BITS` array, and `MAX_PSI_OUTER_FINAL_REF_BITS`, bypassing the captured-anchor checks. Use for first-time author-bake and intentional re-bake; the IV-1 protocol forbids using this to silence a drift assertion.

## Roadmap (followups, not in v2 scope)

This row is v2 of a queued evolution toward iter-2+ silicone-device design support:

- **v3** — axial zoned variation (proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`). Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4** — explicit thin copper-mesh sub-layer (4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A).
- **v? — 8 mm intrusion via Mooney-Rivlin**. The user-target physical depth is structurally unreachable on Neo-Hookean. Post-ship investigation (Dirichlet displacement-control spike, see [v2 spec memo's "Post-ship investigation" section][v2spec]) identified the wall as the **Neo-Hookean validity domain** (Phase 4 Decision Q fail-closed at `max_stretch_deviation < 1.0`), tripping at ~6.5 mm cavity-wall displacement when one tet's principal stretches reach `[2.06, 1.22, 0.073]` — past NH's calibrated validity range. faer LU fallback would NOT help; the validity gate is upstream of any linear solve. **The chosen path is Mooney-Rivlin (or higher-order hyperelastic) calibrated from publicly available Smooth-On data sheets** (100% modulus + 200% modulus + tensile strength at break). MR has a wider validity envelope (~2-3× faithful at higher principal stretches) and the data is already on hand — no physical cast required for sim faithfulness at 8 mm. Two alternative paths if MR is insufficient: (b) **local h-refinement** near the contact band so per-tet strain stays smaller; (c) **change the contact geometry** — lateral probe, smaller probe radius, or distributed traction to avoid the point of extreme local strain.
- **vN** — real anatomy scan replacing the cuboid fixture (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
