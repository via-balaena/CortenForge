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

The ramp's empirical reach was characterized at v2-spec lock time via five pre-execution spike runs (Runs 1–3 swept step-size + iter-cap; Spike A + Spike B then probed mesh resolution + stiffness gradient — see [v2 spec memo][v2spec]). The wall lies at `max_disp ≈ 7 mm`. **6 mm penetration is this row's target — the deepest reach with comfortable solver margin** (step 12 converges in 61 Newton iters at `MAX_NEWTON_ITER = 100`, leaving a 39-iter safety margin). The user-target 8 mm physical intrusion is structurally unreachable on Neo-Hookean — post-ship investigation (Dirichlet displacement-control spike, banked at the [v2 spec memo][v2spec]'s "Post-ship investigation" section) identified the wall as the **Neo-Hookean validity domain** tripping fail-closed (Phase 4 Decision Q at `max_stretch_deviation < 1.0`), NOT a Newton-basin / Armijo / faer issue. The chosen path forward is Mooney-Rivlin calibrated from public Smooth-On data — see the v? entry in the Roadmap section below.

This row is also the **first sim-soft user-facing example to demonstrate quasi-static `replay_step` chaining** — `x_prev_k+1 = x_final_k`, `v_prev = 0` throughout (no inertia carry-over). Distinct from row 17 [`soft-drop-on-plane`](../soft-drop-on-plane/), which chains the full `step` solver method for transient dynamics with inertial damping; v2's chaining is the quasi-static-equilibrium variant. The pattern generalises to any scenario where the single-step Newton basin would otherwise collapse.

Every gate sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. This row is a Rule-B **validator**: its gates are pipeline-emergent structural + physics invariants read from the real 12-step ramp, not captured-bit self-pins (those were stripped in the Rule-B de-frag — constitutive and mesher correctness are lib-owned). Outputs are `out/scan_fit_3layer_sleeve_ramp.json` (final-step scalars + per-shell material `(μ, λ)` + per-step `ramp_curve` array + per-active-contact-pair detail at the final step) and the F1.5 viz PLYs `out/sleeve_boundary_final.ply` + `out/sleeve_slab_cut_z0_final.ply` + `out/sleeve_design_surface_deformed_step_01..12.ply` (full 3D body, equatorial cross-section, and per-step deformed design surfaces via `sim_soft::viz`).

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

// 7. Per-step verifies (structural + physics gates, see "Numerical anchors").

// 8. JSON + PLY readouts. Final step drives the headline viz PLYs
//    (boundary surface + z=0 slab cut); the JSON `ramp_curve` array carries
//    the full force-displacement trace for matplotlib post-processing.
```

The ramp pattern composes for any scenario where the single-step Newton basin is too tight. Future rows can reuse `solve_ramp` directly; the only per-row choice is `(N_RAMP_STEPS, PROBE_PENETRATION_FINAL, MAX_NEWTON_ITER)` — all three set empirically from a pre-execution spike on the target geometry.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material FEM → multi-step rigid intrusion ramp"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each gate is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*` and is called from `main()` in dependency order; `cargo run --release` exit-0 means every gate passed. They are **pipeline-emergent structural + physics invariants** read from the real 12-step ramp — resolution- and toolchain-robust. The pre-Rule-B captured-bit self-pins (exact tet/vertex/pair counts, the per-step iter-count freeze, the 60 per-step + final `to_bits()` force/displacement/Ψ̄ pins, the `to_neo_hookean()` provenance mirror) were **stripped**: they froze one run's FP trajectory on one toolchain, and the constitutive correctness they redundantly implied is lib-owned (`neo_hookean.rs` closed-form tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`), as is the generic distance-from-scan shell routing (`sdf_material_tagging.rs` IV-4). The observed per-step scalars (force, displacement, per-layer Ψ̄) are still emitted to the JSON `ramp_curve` + stdout for eyes-on inspection — just not self-pinned.

### 1. `quality_floors`

| Gate | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Same gate as row 21 v1. Pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse()` invariant.

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

Each material band contributes ≥ 1 tet to the `z = 0` cut. Non-empty, not exact-count.

### 4. `per_step_solver_converges`

| Gate | Bound |
|---|---|
| `r.iter_count < MAX_NEWTON_ITER = 100` | per step, all 12 |
| `r.final_residual_norm < 1e-10` | per step, all 12 |

Each step's Newton solve must converge within the iter cap + tolerance bound. Observed iter counts `[8 … 30, 61]` — step 12 (the deepest at 6 mm) is the closest to the cap with 39 iters of margin.

### 5. `force_displacement_monotone`

| Gate | Bound |
|---|---|
| `force_total_z[N-1] > force_total_z[0]` | strict (ramp endpoint sanity) |
| `force_total_z[k+1] > force_total_z[k]` for k ≥ 1 | strict-adjacent monotone from step 2 onward |

Force-on-soft summed `+z`-component is in `+z` direction (probe enters from above; wrap-cap material is pushed UP) and grows monotonically with deeper penetration (observed `~1.1 N` at 0.5 mm → `~25.6 N` at 6 mm). Strict-adjacent monotonicity at step 1 → step 2 is NOT required: contact engagement at the shallowest 0.5 mm penetration is a transient regime where the active-pair set is still settling.

**v2.5 sign-flip note.** Pre-v2.5 `force_total_z` was `-z` (e.g., `-137 N` at step 1) because the unfiltered `per_pair_readout` includes ORPHAN BCC vertices inside the empty cavity — orphans below the probe equator have `-z` normals and dominated the sum. `sim_soft::filter_pair_readouts_to_referenced` restricts to solver-active vertices, surfacing the physically meaningful `+z` push. See pattern (xx) at the row 22 patterns memo.

### 6. `per_step_strain_energy_ordering`

| Gate | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict, per step (12) |
| `Ψ̄_middle > Ψ̄_outer` | strict, per step (12) |

The same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 21 v1, applied at every ramp step. The compounding (compliance + distance-to-load + distance-to-constraint) holds throughout. Observed final-step values: `Ψ̄_inner ≈ 302 J/m³`, `Ψ̄_middle ≈ 102 J/m³`, `Ψ̄_outer ≈ 30 J/m³`.

### 7. `per_step_max_disp_bounded`

| Gate | Bound |
|---|---|
| `max_disp[k] < WRAP_THICKNESS = 0.014 m` | strict, per step (12) |

Body-wide max displacement must stay under the 14 mm wrap thickness at every step (the cavity wall reaching the outer envelope's Dirichlet pin band would mean wrap collapse). Observed final-step max_disp ≈ 6.7 mm, ~48 % of the bound.

### 8. `contact_engaged`

| Gate | Bound |
|---|---|
| `n_active_pairs` at final step | `> 0` |

The final (deepest) ramp step engaged the probe against the cavity wall. (The exact pair count was a mesher/discretization artifact; non-empty is the invariant that matters — the count is over the referenced-only filtered set, so it excludes the orphan cavity vertices discussed in gate 5.)

### 9. `material_routing`

| Gate | Bound |
|---|---|
| `mesh.materials()[t]` per tet | `(μ, λ)` bit-equal to the shell's F4 entry, each shell exercised ≥ 1 tet |

The scene's multi-material routing invariant — the `MaterialField` assigned every tet the `(μ, λ)` of the shell its centroid falls in. Reads the real per-tet `NeoHookean` from `mesh.materials()` via public `.mu()` / `.lambda()` and compares to the shell's F4 entry (`ECOFLEX_00_20` / `DRAGON_SKIN_10A` / `DRAGON_SKIN_20A`, all `.to_neo_hookean()`) with an exact `to_bits()` `==` — a routing check, NOT a constitutive mirror. The closed-form NH energy + the `to_neo_hookean()` round-trip are lib-owned (`neo_hookean.rs` tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`); the generic routing mechanism is lib-owned (`sdf_material_tagging.rs` IV-4).

## Visuals

Post-F1.5-viz-retrofit, the final step (depth = 6 mm) emits proper triangulated viz artifacts via `sim_soft::viz` (not the pre-retrofit z-slab centroid cloud): `out/sleeve_boundary_final.ply` (full 3D body via `boundary_surface`), `out/sleeve_slab_cut_z0_final.ply` (equatorial cross-section at `z = 0` via `slab_cut`, 40 mm below the contact zone — catches the propagated wrap-shell response), plus the cf-design design-mesh emits and the per-step deformed-surface sequence `out/sleeve_design_surface_deformed_step_01..12.ply` (scrubbable in cf-view). Each carries a categorical `material_id` (0 = inner / 1 = middle / 2 = outer → categorical palette) + a sequential `displacement_magnitude` (unipolar → viridis), per pattern (u).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-ramp/out/sleeve_boundary_final.ply
```

(An eyes-on-pixels review of the retrofit artifacts is a user-side pass — the boundary/slab-cut meshes replaced the pre-F1.5 centroid cloud this section originally described.)

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

## Roadmap (followups, not in v2 scope)

This row is v2 of a queued evolution toward iter-2+ silicone-device design support:

- **v3** — axial zoned variation (proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`). Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4** — explicit thin copper-mesh sub-layer (4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A).
- **v? — 8 mm intrusion via Mooney-Rivlin**. The user-target physical depth is structurally unreachable on Neo-Hookean. Post-ship investigation (Dirichlet displacement-control spike, see [v2 spec memo's "Post-ship investigation" section][v2spec]) identified the wall as the **Neo-Hookean validity domain** (Phase 4 Decision Q fail-closed at `max_stretch_deviation < 1.0`), tripping at ~6.5 mm cavity-wall displacement when one tet's principal stretches reach `[2.06, 1.22, 0.073]` — past NH's calibrated validity range. faer LU fallback would NOT help; the validity gate is upstream of any linear solve. **The chosen path is Mooney-Rivlin (or higher-order hyperelastic) calibrated from publicly available Smooth-On data sheets** (100% modulus + 200% modulus + tensile strength at break). MR has a wider validity envelope (~2-3× faithful at higher principal stretches) and the data is already on hand — no physical cast required for sim faithfulness at 8 mm. Two alternative paths if MR is insufficient: (b) **local h-refinement** near the contact band so per-tet strain stays smaller; (c) **change the contact geometry** — lateral probe, smaller probe radius, or distributed traction to avoid the point of extreme local strain.
- **vN** — real anatomy scan replacing the cuboid fixture (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
