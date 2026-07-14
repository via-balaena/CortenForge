# concentric-lame-shells

**Phase 4 IV-5 user-facing wrap — three-shell concentric hollow silicone sphere meshed via the BCC + Labelle-Shewchuk pipeline, internal pressure on the cavity wall, fixed Dirichlet pin on the outer surface, with closed-form piecewise-Lamé radial-displacement comparison per shell + uniform-baseline strict-between cavity-wall gate.** A `DifferenceSdf` of two `SphereSdf`s at radii `(R_OUTER, R_CAVITY) = (0.10, 0.04) m` (hollow shell body, `LAYERED_SPHERE_R_*` constants reused from sim-soft's re-exports) is meshed via `SoftScene::layered_silicone_sphere(material_field, cell_size, pressure)`; a 3-shell `MaterialField` partitions tets into inner Ecoflex (`(MU_INNER, LAMBDA_INNER) = (1e5, 4e5) Pa`, `1×` baseline) / middle carbon-black-composite (`(2e5, 8e5) Pa`, `2×`) / outer Ecoflex (`(1e5, 4e5) Pa`, `1×`) per Decision J's `1× / 2× / 1×` symmetry pattern (compressible regime, all in `λ = 4 μ` ⇒ `ν = 0.4`). The cavity surface receives a per-vertex radially-outward force traction packed as `LoadAxis::FullVector` (`f_v = pressure · n̂_v · A_v`, `A_v = 4π R_CAVITY² / N_loaded`); the outer surface is fully Dirichlet-pinned, killing the 6 rigid-body modes and generalising the Lamé closed-form to fixed-outer BC variant via `u_r^{(3)}(R_b) = 0`. A single backward-Euler `replay_step` at `cfg.dt = STATIC_DT = 1.0 s` (collapses the inertial term `M / Δt²` by ~4 orders of magnitude relative to stiffness — IV-5's static-regime idiom verbatim) converges from rest to the static equilibrium configuration in 3 Newton iters. The headline new capability vs row 10 is **three-shell hollow-body integration** — multi-material (3 shells, not 2) + SDF-meshed (hollow `DifferenceSdf` body, not hand-built) + non-vertex-force BC (radially-outward distributed pressure, not axial tip-force) + piecewise-Lamé closed-form analytic (3-shell composite, not Euler-Bernoulli composite-beam). This is **PR1 finale, row 10 of 10** — the canonical IV-5 multi-material + SDF + hollow-body integration test, exposed as a user-facing demo. Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/concentric_lame_shells.ply` (616 z-slab centroids of the 6456 body tets at amplified deformed positions + 2 per-vertex scalars; the `verify_*` correctness gates run over all 6456 tets / 1480 referenced vertices, with two extra solver runs at uniform-1× and uniform-2× baselines for the IV-2 lens β analog at HEADLINE C).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The fourth user-facing example of Tier 3 multi-material per [`EXAMPLE_INVENTORY.md`][inv] — the canonical `SoftScene::layered_silicone_sphere + 3-shell MaterialField + radially-outward pressure BC` solver-driven scene, generalising row 8's three-shell concentric scene from field-exposition to solver-driven AND row 10's bilayer cantilever-beam EB-composite analytic to a 3-shell hollow sphere piecewise-Lamé composite analytic:

```rust
let scene_field = MaterialField::from_fields(
    Box::new(LayeredScalarField::new(
        Box::new(SphereSdf { radius: LAYERED_SPHERE_R_OUTER }),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],         // [-0.04, -0.02]
        vec![MU_INNER, MU_MIDDLE, MU_OUTER],                     // [1e5, 2e5, 1e5]
    )),
    Box::new(LayeredScalarField::new(
        Box::new(SphereSdf { radius: LAYERED_SPHERE_R_OUTER }),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],         // [4e5, 8e5, 4e5]
    )),
);

// SoftScene constructor packs DifferenceSdf body + half-cell surface-band
// predicates + per-vertex pressure-traction computation under the hood.
let (mesh, bc, initial, theta) =
    SoftScene::layered_silicone_sphere(scene_field, CELL_SIZE, PRESSURE)?;       // (0.02, 5e3 Pa)

let mut cfg = SolverConfig::skeleton();
cfg.dt = 1.0;                                                                    // STATIC_DT — IV-5 idiom
cfg.max_newton_iter = 50;

let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
    CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
let step = solver.replay_step(&initial.x_prev, &initial.v_prev, &theta, cfg.dt);
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

The shell partition lives at the centroid level. `LayeredScalarField`'s `partition_point(|&t| t <= phi)` rule places `phi == threshold[i]` into shell `i + 1` (closed-left, open-right):

```text
phi = ‖x_centroid‖ − R_OUTER

phi <  PHI_INNER_THRESHOLD  (i.e., ‖p‖ < R_INNER_OUTER)               ⇒ inner   shell 0   (μ = MU_INNER,  λ = LAMBDA_INNER)
PHI_INNER_THRESHOLD ≤ phi < PHI_MIDDLE_THRESHOLD                       ⇒ middle  shell 1   (μ = MU_MIDDLE, λ = LAMBDA_MIDDLE)
phi ≥ PHI_MIDDLE_THRESHOLD  (i.e., ‖p‖ ≥ R_OUTER_INNER)                ⇒ outer   shell 2   (μ = MU_OUTER,  λ = LAMBDA_OUTER)
```

This is **row 11 of the sim-soft examples arc** — the fourth Tier 3 multi-material example, the second solver-driven multi-material scene (after row 10's bonded bilayer beam), the first hollow-body solver-driven scene with non-vertex-force BC, and the **PR1 finale** capping the 10-row capability sweep. Row 11 is the canonical IV-5 multi-material + SDF + hollow-body integration test (`sim/L0/soft/tests/concentric_lame_shells.rs`), exposed as a user-facing demo at the IV-5 main convergence-test middle refinement (`cell_size = h/2 = 0.02`).

Three solver runs total: the headline three-shell scene (HEADLINE A + B) plus uniform-1× and uniform-2× baselines for the IV-2 lens β analog at HEADLINE C. The two extra runs are full pipelines through `SoftScene::layered_silicone_sphere` + `replay_step` but only the cavity-wall mean is read out (the per-tet records and per-shell means apply only to the three-shell run); marginal cost is `~10 sec` runtime.

**Lamé pairs** match Phase 4 scope memo Decision J: inner = outer Ecoflex 00-30 baseline (`(μ, λ) = (1e5, 4e5)` ⇒ `ν = 0.4` compressible), middle is `2×` (Ecoflex 00-30 + 15 wt% carbon-black per Part 1 §04 §02, mechanical-only at this phase). Decision J's `outer / middle / inner = Ecoflex / composite / Ecoflex` symmetry commitment is structural — the device's outer and inner skins are the same elastomer, the middle is the composite layer.

| Shell | μ (Pa) | λ (Pa) | factor | radial range |
|---|---|---|---|---|
| inner  | `1.0e5` | `4.0e5` | `1×` baseline   | `[R_CAVITY,         R_INNER_OUTER)` = `[0.04, 0.06)` |
| middle | `2.0e5` | `8.0e5` | `2×` Decision J | `[R_INNER_OUTER,    R_OUTER_INNER)` = `[0.06, 0.08)` |
| outer  | `1.0e5` | `4.0e5` | `1×` baseline   | `[R_OUTER_INNER,    R_OUTER]`       = `[0.08, 0.10]` |

**Why `cfg.dt = 1.0` (STATIC_DT idiom).** Row 6's `cfg.density = 0` and IV-3 / IV-5's `cfg.dt = 1.0` are two ways to suppress the inertial recall term and recover the static equilibrium under prescribed BCs. Row 11 mirrors IV-5 verbatim — same scene + same time-step idiom keeps cross-reference between the example and the internal fixture clean. At `cfg.dt = 1.0` and the cavity-pressure-inflation scene's stiffness scale, `M / dt²` is `~4` orders below the elastic stiffness contribution; a single `replay_step` from rest converges to the static equilibrium far below `cfg.tol = 1e-10`. Newton converges in 3 iters at this refinement (radially-symmetric distributed-pressure load with fixed-outer-pin BC is in the linearisable small-strain regime from rest — no rotations, no tip-force concentration, `~0.71 %` cavity-wall inflation observed / `~0.84 %` analytic) — well within the `max_newton_iter = 50` budget.

**Why a z-slab cut (vs row 10's y-slab).** Same banked rationale as rows 8 + 9 (cf-view's commit-3 instanced-sphere radius factor `bbox.diagonal() * 0.005` is oversized for dense per-tet centroid clouds in small-bbox bodies — see [`project_cf_viewer_dense_point_cloud_gap.md`][gap]) but adapted to row 11's spherically-symmetric body. Radial expansion under internal pressure is azimuthally symmetric, so a `|centroid.z| < cell_size/2` z-slab cut projects centroids onto the x-y plane — reading as three concentric color rings (categorical `material_id`) + a smooth radial gradient (continuous `radial_displacement`) unmistakably from any cf-view orbit angle. Row 10's y-slab was the right adaptation for its axial cantilever-beam geometry; row 11 reverts to the row-8 z-slab pattern for the spherically-symmetric body. The slab keeps 616 of 6456 centroids (~9.5% of the body, since `cell_size / (2 R_OUTER) = 0.10`).

[gap]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_cf_viewer_dense_point_cloud_gap.md

**Why two per-vertex scalars (vs row 9's three).** Row 11's headline is the analytic comparison + per-shell radial-displacement profile, not the visualisation. Two scalars (`material_id` categorical → tab10 for the 3-shell partition; `radial_displacement` continuous → viridis for the inflation profile) cover the structural-composition readout and the radial-decay gradient unambiguously; a third scalar would be ornamentation. Same trade-off framing as row 10's "outcome + feature" pair, here re-shaped to "feature + outcome" (cf-view's alphabetical-first picks `material_id` — the 3 sharp shells in 3 colors is the loud first impression; user dropdowns to `radial_displacement` for the radial-inflation gradient).

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 11 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

| Anchor | Bound |
|---|---|
| `0 < R_CAVITY < R_INNER_OUTER < R_OUTER_INNER < R_OUTER` | strict `<` chain |
| `BBOX_HALF_EXTENT > R_OUTER` | margin for BCC mesher cuts |
| `PRESSURE > 0` | inflation direction |
| `MU_INNER == MU_OUTER`, `LAMBDA_INNER == LAMBDA_OUTER` | Decision J 1×/2×/1× symmetry |
| `MU_INNER < MU_MIDDLE`, `LAMBDA_INNER < LAMBDA_MIDDLE` | 2× ordering |
| `LAMBDA_X == 4 · MU_X` per shell | ν = 0.4 compressible (per IV-3 + IV-5 deviation) |
| `BBOX_HALF_EXTENT / CELL_SIZE >= 6.0` | III-1 / IV-4 BCC stability ratio |
| `PHI_INNER_THRESHOLD < PHI_MIDDLE_THRESHOLD < 0` | `LayeredScalarField` strict-monotone-increasing thresholds invariant |

The `layered_silicone_sphere` constructor's runtime panic invariants re-asserted at the user-facing example layer via `const { assert!(...) }` — compile-time enforcement on the geometry constants. Plus the example-specific Decision J symmetry contract.

### 2. `mesh_topology_exact`

| Anchor | Bound |
|---|---|
| `mesh.n_tets` | exact-pin `6456` |
| `mesh.n_vertices` | exact-pin `4682` (BCC corners + warp-snapped + cut-points) |
| `referenced_vertices(&mesh).len()` | exact-pin `1480` (orphan BCC corners excluded) |
| `referenced.len() < mesh.n_vertices` | orphan-rejection invariant non-vacuous |

Counts captured 2026-05-05 on the row 11 capture platform per the III-1 determinism contract; bit-exact match with IV-5's `iv_5_three_shell_converges_to_piecewise_lame` h/2 reading (`n_tets = 6456`).

### 3. `boundary_partition`

| Anchor | Bound |
|---|---|
| Pinned (outer-surface band, `(‖p‖ - R_OUTER).abs() < cell_size/2`) | exact-pin `734` |
| Loaded (cavity-surface band, `(‖p‖ - R_CAVITY).abs() < cell_size/2`) | exact-pin `134` |
| Pinned + loaded ascending order | strict `<` consecutive |
| `pinned ∩ loaded` | empty (R_OUTER and R_CAVITY are at opposite radii) |
| Every loaded entry's `LoadAxis` | `FullVector` (radially-outward pressure) |

Both vectors come from `pick_vertices_by_predicate` filtered against `referenced_vertices` (BCC orphan rejection — orphan corners would trip the solver's "load on orphan vertex" panic at `backward_euler.rs:274`); ascending is structural here. Disjointness is geometric (R_OUTER and R_CAVITY differ by `2.5×`). Every loaded entry uses `LoadAxis::FullVector` because radially-outward pressure traction is per-component, not axis-aligned — distinct from rows 6 + 10 which use `LoadAxis::AxisZ` for axial loads.

### 4. `per_tet_material_assignment`

**HEADLINE A.** For every tet, `mesh.materials()[t]` is probed at `F_probe = diag(1.2, 1, 1)` against `expected = NH(MU_X, LAMBDA_X)` where `X = shell_at(rest_centroid)`:

| Anchor | Bound |
|---|---|
| Per tet `materials()[t].energy(F_probe)` vs expected | `epsilon = 0.0` (bit-equal) |
| Per tet `materials()[t].first_piola(F_probe)` vs expected | `epsilon = 0.0` (bit-equal) per entry |
| Per-shell tet count (inner / middle / outer) | exact-pin `1032 / 1800 / 3624` |
| `inner + middle + outer` | `== N_TETS_EXACT = 6456` |

The cross-implementation gate: the test-side `shell_at` reimplements `LayeredScalarField`'s `partition_point(|&t| t <= phi)` rule verbatim, so centroid-sampled `materials()` and re-derived `expected[shell_at(...)]` MUST agree on every tet by construction. Drift between mesher partition pass and test re-derivation fires loud. Same `Matrix3` over-determination logic as rows 8 + 9 + 10: `energy(F_probe)` alone is one linear equation in `(μ, λ)`; `first_piola(F_probe)`'s `P_22 = λ ln J` directly fixes `λ`, then `P_11` fixes `μ` (over-determination).

This is the canonical IV-5 cross-impl gate (row 8 anchor 4 generalised from solid 3-shell to hollow 3-shell + the BCC mesher's centroid-tag pass), exposed user-facing at h/2 production-scale mesh resolution.

### 5. `solver_converges`

| Anchor | Bound |
|---|---|
| `step.iter_count` | strict `<` `cfg.max_newton_iter = 50` |
| `step.final_residual_norm` | strict `<` `cfg.tol = 1e-10` |
| Per-tet `max\|σᵢ - 1\|` at `x_final` | strict `<` NH validity bound `1.0` |
| Global max stretch deviation at `x_final` | strict `<` `0.05` (small-strain regime sanity) |
| `mat.validity().max_stretch_deviation` | `to_bits` equality vs `1.0` |
| `mat.validity().inversion` | `RequireOrientation` |

The cavity-pressure-inflation scene at `pressure = 5e3 Pa` deforms by `~0.71 %` of `R_CAVITY` at the cavity wall (observed `2.857e-4 / 0.04 ≈ 0.71 %`; analytic `~0.84 %`); per-tet `max|σ-1|` peaks at `~0.024` (well below the `0.05` small-strain regime sanity). Catches a regression that would push the converged solution toward NH's domain boundary.

### 6. `radial_displacement_per_shell_matches_lame_within_30pct`

**HEADLINE B — inventory's named gate.** Four Saint-Venant-averaged radial-displacement readouts matched against the piecewise-Lamé closed-form `u_r^{(i)}(r) = A_i r + B_i / r²` (with `(A_i, B_i)` solved from the 6×6 system at the row 11 constants — see `solve_three_shell_lame` inlined from IV-5):

| Readout | Definition | observed | analytic | rel-err |
|---|---|---|---|---|
| `cavity_wall_mean`  | mean over `bc.loaded_vertices` of `(\|x_final[v]\| - \|rest_pos[v]\|)`; analytic = `coeffs.u_r(0, R_CAVITY)` (single-r, mirrors IV-5's headline reading verbatim) | `2.857e-4 m` | `3.342e-4 m` | `14.5 %` |
| `inner_shell_mean`  | mean over all 226 referenced vertices in inner shell of the same scalar; analytic = mean of `coeffs.u_r(0, \|rest\|)` over the SAME vertex set | `2.130e-4 m` | `2.420e-4 m` | `12.0 %` |
| `middle_shell_mean` | same shape, 302 middle-shell vertices, `coeffs.u_r(1, \|rest\|)` | `6.004e-5 m` | `7.406e-5 m` | `18.9 %` |
| `outer_shell_mean`  | same shape, 952 outer-shell vertices (734 pinned at `u_r = 0`), `coeffs.u_r(2, \|rest\|)` | `6.724e-6 m` | `1.044e-5 m` | `35.6 %` |

Gate per readout: `assert_relative_eq!(observed, analytic, max_relative = 0.30, epsilon = 5.0e-6 m)`. The `0.30` rel-tol is binding for cavity / inner / middle (mirrors IV-5's `iv_5_uniform_passthrough_at_h2_matches_single_shell_lame` sanity-band gate); the `5e-6 m` eps absolute floor absorbs the outer-shell readout's small-magnitude case (observed abs diff `~3.7e-6 m < 5e-6 m` floor — see [Tet4 caveat](#tet4-caveat--per-shell-convergence-pattern-at-h2) below for why outer-shell rel-err is genuinely larger at h/2).

Inventory's named gate ("assert radial displacement vs Lamé per shell"), generalising IV-5's cavity-wall-only reading to a 3-shell profile gate while preserving the IV-5 cross-reference at the cavity-wall readout (the cavity_wall_mean readout uses single-r analytic at exactly `R_CAVITY` mirroring IV-5; the per-shell readouts use vertex-set-mean of analytic over the same set as observed for apples-to-apples comparison without band-tolerance choices).

### 7. `cavity_wall_three_shell_strictly_between_uniform_bounds`

**HEADLINE C — IV-2 lens β analog at the cavity-wall mean.** Three solver runs: three-shell scene + uniform-1× baseline (every shell at `MU_INNER, LAMBDA_INNER`) + uniform-2× baseline (every shell at `MU_MIDDLE, LAMBDA_MIDDLE`). The cavity-wall Saint-Venant means satisfy a strict-between inequality:

| Anchor | Bound |
|---|---|
| `cavity_uniform_2x > 0`, `cavity_uniform_1x > 0`, `cavity_three_shell > 0` | strict `>` (cavity inflates outward) |
| `cavity_uniform_2x < cavity_uniform_1x` | strict `<` (all-stiff deflects strictly less than all-soft) |
| `cavity_uniform_2x < cavity_three_shell` | strict `<` (lower bound — three-shell more compliant than uniform-2×) |
| `cavity_three_shell < cavity_uniform_1x` | strict `<` (upper bound — three-shell less compliant than uniform-1×) |

Observed at h/2 on the row 11 capture platform:

| Configuration | Cavity-wall mean | Compliance |
|---|---|---|
| `uniform_2x` (every shell at `MU_MIDDLE = 2×`) | `1.626e-4 m` | stiffest — smallest inflation |
| `three-shell` (`1× / 2× / 1×` layered)        | `2.857e-4 m` | intermediate |
| `uniform_1x` (every shell at `MU_INNER = 1×`)  | `3.245e-4 m` | softest — largest inflation |

The directionality follows from compliance composition under fixed-outer pressure-vessel mechanics: by linearity of the Lamé 6×6 system in `(μ, λ)` (uniform-2× has every shell at exactly `2×` the uniform-1× pair), uniform-2× cavity is exactly half of uniform-1× cavity in the analytic and within `~0.24 %` of half at the FEM level (uniform-2× FEM `1.626e-4 m` vs `0.5 × 3.245e-4 m = 1.622e-4 m` half-of-uniform-1× FEM — clean FP noise on the linearity prediction). Three-shell sits between because two of its three shells are at `1×` (more compliant than uniform-2×) and one is at `2×` (less compliant than uniform-1×).

**Why this anchor.** Mirrors row 10 banked pattern (d) — the IV-2 lens β analog on aggregate readout — adapted to row 11's hollow-body cavity-wall mean. Row 10 used `d_uniform_b < d_bilayer < d_uniform_a` on tip displacement; row 11 uses the same shape on cavity-wall mean. Catches dropped-tet-contribution / swapped-materials / mis-assigned bugs that would push `cavity_three_shell` to one of the bounds (or outside) without flipping any per-tet material-probe (anchor 4) or per-shell rel-err gate (anchor 6). The strict-between inequality is structurally tighter than anchor 6's `< 0.30` rel-err gate at downward-bias FEM-assembly regressions in the 17-30% range — anchor 6's cavity-wall reading observed at 14.5% rel-err is 15-percentage-points below the gate; anchor 7 catches a 17-percentage-point downward shift that would push three-shell below uniform-1×.

The 2 extra solver runs (uniform-1× and uniform-2×) are full pipeline runs through `SoftScene::layered_silicone_sphere` + `replay_step`, but only the cavity-wall mean is read out (no per-tet records, no per-shell means) since the between-bounds gate is single-readout. Marginal cost is `~10 sec` runtime + 2 captured bits at IV-1 sparse-tier rel-tol contract.

### 8. `radial_monotonicity_outward`

| Anchor | Bound |
|---|---|
| `cavity_wall_mean > inner_shell_mean` | strict `>` |
| `inner_shell_mean > middle_shell_mean` | strict `>` |
| `middle_shell_mean > outer_shell_mean` | strict `>` |
| `outer_shell_mean ≥ 0` | non-negative (no inward inflation under positive pressure) |

The piecewise-Lamé closed-form predicts strict outward monotone-decay under the internal-pressure-with-fixed-outer geometry: cavity wall is most inflated (`u_r(R_a)` is max), outer wall is pinned at 0 (BC #2 in the 6×6 system: `u_r^{(3)}(R_b) = 0`). Sanity guard against load-direction or BC-sign regressions.

### 9. `captured_bits_radial_displacements`

Six radial-displacement means captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `cavity_wall_mean` (three-shell) vs captured ref | rel `1e-12`, abs `1e-12` |
| `inner_shell_mean` (three-shell) vs captured ref | rel `1e-12`, abs `1e-12` |
| `middle_shell_mean` (three-shell) vs captured ref | rel `1e-12`, abs `1e-12` |
| `outer_shell_mean` (three-shell) vs captured ref | rel `1e-12`, abs `1e-12` |
| `cavity_wall_mean_uniform_1x` vs captured ref | rel `1e-12`, abs `1e-12` |
| `cavity_wall_mean_uniform_2x` vs captured ref | rel `1e-12`, abs `1e-12` |

6456 tets through faer's sparse Cholesky lives between IV-1's dense bit-equal tier (12-24 DOFs) and IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift on faer's per-column FMA-fusion path). 6456 is past IV-1's sparse-at-scale tier; the contract is **relative tolerance, not strict bit-equality** — observed bits captured for regression detection, compared via `assert_relative_eq!` at `1e-12` rel. Capture provenance + failure-mode protocol IV-1-protocol-verbatim above the const blocks (do NOT re-bake without diagnosing toolchain vs real regression first).

This is the row's **tight regression-detection gate** — anchors 6 + 7 document physical reasonableness (analytic comparison at `0.30 / 5e-6`, between-bounds inequality on cavity-wall); anchor 9's `1e-12` rel against pinned bits catches any FEM-numerics-shifting regression at machine precision across all three solver runs.

### 10. `material_populations_exact`

| Anchor | Bound |
|---|---|
| Inner-shell tet count | exact-pin `1032` |
| Middle-shell tet count | exact-pin `1800` |
| Outer-shell tet count | exact-pin `3624` |
| `inner + middle + outer` | `== N_TETS_EXACT = 6456` |

Visual-pedagogy guard: each shell has ≥ 1 tet (the meshing pipeline successfully resolves the body into 3 distinct shells); per-shell counts also exact-pinned per the III-1 determinism contract. Mirrors row 8's banked pattern at the hollow-body extension.

### 11. `zslab_visual_populations_exact`

| Anchor | Bound |
|---|---|
| Z-slab inner-shell count | strict `>` `0` AND exact-pin `184` |
| Z-slab middle-shell count | strict `>` `0` AND exact-pin `176` |
| Z-slab outer-shell count | strict `>` `0` AND exact-pin `256` |

Visual-pedagogy guard: each shell must have ≥ 1 z-slab centroid for the three concentric-ring cf-view rendering to work; per-shell counts also exact-pinned. Total `616 = 184 + 176 + 256` (~9.5% of the 6456 body tets, since `cell_size / (2 R_OUTER) = 0.10`).

## Visuals

`out/concentric_lame_shells.ply` — the canonical visual artifact.

```text
616 vertices (one per tet centroid in the |centroid.z| < cell_size/2 equatorial-plane z-slab cut,
              vertex positions amplified `rest + DISPLACEMENT_SCALE × (deformed - rest)`
              with DISPLACEMENT_SCALE = 50.0 — visualisation-only amplification)
0 faces (point cloud)
extras["material_id"]:           f32  // categorical 0.0/1.0/2.0      (inner/middle/outer)
extras["radial_displacement"]:   f32  // continuous m                  (TRUE physical inflation profile)
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/concentric-lame-shells/out/concentric_lame_shells.ply
```

cf-view auto-discovers the two per-vertex scalars; alphabetical-first selects `material_id` on launch — cf-view's Q5 colormap heuristic detects integer-valued + 3 unique values < 16 → tab10 categorical palette, rendering the three concentric shells as three sharp colors in a flat disk (the z-slab projects to a ~2D x-y disk in cf-view).

User dropdowns to `radial_displacement`: cf-view detects continuous + all-positive → viridis sequential palette, rendering the radial-inflation gradient as a smooth color decay from `~3e-4 m` (cavity wall, brightest) to `~0 m` (outer wall, darkest). Same rendered geometry, different scalar.

### Why `DISPLACEMENT_SCALE = 50.0` (visualisation-only amplification)

Standard FEM-visualisation trick. At small-strain regime the geometric inflation arc is honest-but-subtle: the observed cavity-wall radial displacement is `~2.86e-4 m` on a body of radius `0.10 m` (~0.29% of `R_OUTER`; the analytic predicts `~3.34e-4 m ≈ 0.33 %` and the Tet4 + SDF-meshed three-shell rel-err at this refinement is ~14.5%). The unsmaller-strain magnitudes here (smaller than row 10's ~2.2% bending) makes the geometric inflation invisible at default cf-view orbit even with modest amplification. A `50×` amplifier on the geometric vertex positions puts the visible cavity-wall inflation at `~1.4e-2 m` (~36% of `R_CAVITY`, ~14% of `R_OUTER`), dramatically visible without distorting the spherical-symmetry readability — the inflation is exaggerated but the body still reads as a hollow shell.

**The amplification is geometry-only.** The `radial_displacement` per-vertex scalar carries the TRUE physical displacement (no scale factor); cf-view's color gradient still reflects the true inflation magnitude `[0, ~3e-4 m]`. Every `verify_*` correctness gate operates on the unscaled solver outputs (`step.x_final`, `cavity_wall_mean`, the per-tet records' `f_at_x_final`) — the per-shell rel-err vs analytic, the captured-bits anchor under IV-1 sparse-tier rel-tol, and the monotonicity sanity are all unaffected. Same trade-off framing as row 10's `DISPLACEMENT_SCALE = 20.0` (smaller-strain row 11 → larger amplifier; row 11's `~0.71 %` (observed) cavity-wall inflation needs 50× to read; row 10's `~2.2 %` bending needed 20× to read).

If you want the un-amplified visualisation (TRUE geometric deformation), set `DISPLACEMENT_SCALE = 1.0` in `src/main.rs` and re-run; the z-slab cloud will render at the physical `~3e-4 m` cavity-wall bend, near-invisible from default orbit angles.

Per `feedback_visual_review_is_the_test` — for cf-view consumers, the visual pass is real (not collapsed to JSON read).

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 11 anchor-group names, full-body per-shell split + 4 radial-displacement readouts table + analytic + rel-err per readout + cavity-wall uniform-baseline comparison + z-slab partition).

## Run

```text
cargo run -p example-sim-soft-concentric-lame-shells --release
```

Output: `out/concentric_lame_shells.ply` and stdout summary.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The exact-pin counts and captured radial-displacement bits were captured under release-mode build; matching that invocation shape removes one variable from the determinism contract.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Tet4 caveat — per-shell convergence pattern at h/2

Inventory wording for row 11 ("assert radial displacement vs Lamé per shell") doesn't pin a quantitative bound — unlike row 10's "to 3 digits" wording, no inventory-recalibration is required. The operative gate at h/2 (`cell_size = 0.02`) is the row 11 `assert_relative_eq!` shape: `max_relative = 0.30, epsilon = 5e-6 m` per readout.

**Per-shell rel-err pattern at h/2 (observed):**

| Readout | Rel-err | Gate branch |
|---|---|---|
| `cavity_wall_mean`  | `14.5 %` | rel-tol binding (`< 0.30`) |
| `inner_shell_mean`  | `12.0 %` | rel-tol binding (`< 0.30`) |
| `middle_shell_mean` | `18.9 %` | rel-tol binding (`< 0.30`) |
| `outer_shell_mean`  | `35.6 %` | **eps-floor binding** (abs diff `~3.7e-6 m < 5e-6 m`) |

The pattern: as the shell-mean magnitude decreases (further from cavity), rel-err increases. At h/2 with `~0.71 %` (observed) cavity-wall inflation:
- cavity / inner / middle means are large enough (`6e-5` to `3e-4 m`) that the FEM noise's absolute scale is a small fraction → 12-19% rel-err.
- outer-shell mean is small (`~7e-6 m`, dominated by 734 outer-wall vertices pinned at `u_r = 0` plus 218 outer-shell-interior vertices with small `u_r`) → the same FEM noise's absolute scale is a meaningful fraction → 36% rel-err.

The eps-floor (`5e-6 m`) absorbs this small-magnitude case: `assert_relative_eq!` passes via `|obs - ana| <= max(eps, max_relative * max(|obs|, |ana|))` — for outer the max-branch is `max(5e-6, 0.30 × 1.04e-5 = 3.13e-6) = 5e-6 m`, and observed abs diff `3.7e-6 m < 5e-6 m`, so passes. The floor has `~1.3e-6 m` headroom over observed (~35% margin); IV-1 sparse-tier 3-ULP cross-platform drift is far below that scale at sparse-mesh magnitudes.

**Convergence story.** IV-5's main convergence test (`tests/concentric_lame_shells.rs:iv_5_three_shell_converges_to_piecewise_lame`) ships at three refinement levels (h, h/2, h/4) with super-quadratic empirical fine-end rate (~3.5 per IV-5 docstring + per the test's `eprintln!`). The absolute floor at h/4 is `< 0.20` rel-err on cavity wall; at h/2 the cavity-wall rel-err is `~14.5 %` (this row's empirical capture, bit-exact match with IV-5's middle-refinement cavity-wall reading). For tighter per-shell convergence study see IV-5's `iv_5_three_shell_converges_to_piecewise_lame` — heavier mesh (`~50k` tets at h/4), three-refinement convergence-rate assertion. That's the validation gate; this row is the user-facing wrap at the example-friendly h/2 refinement.

**Cavity-wall mean at the row 11 constants — analytic vs FEM at h/2.** The three-shell `1× / 2× / 1×` cavity-wall analytic is `3.342e-4 m` (~0.84 % of `R_CAVITY = 0.04 m`); the FEM observed at h/2 is `2.857e-4 m` (~14.5 % rel-err — IV-5 sanity-band convergence behavior). At HEADLINE C the FEM-vs-FEM uniform-baseline runs give:

| Configuration | Cavity-wall mean (FEM at h/2) | Analytic (closed-form, single-shell Lamé) |
|---|---|---|
| `uniform_2x` (all shells at MU_MIDDLE = 2×) | `1.626e-4 m` | `~1.91e-4 m` (= 0.5 × uniform-1× by uniform-stiffness linearity) |
| `three-shell` (`1× / 2× / 1×`) | `2.857e-4 m` | `3.342e-4 m` (piecewise-Lamé 6×6 LU) |
| `uniform_1x` (all shells at MU_INNER = 1×) | `3.245e-4 m` | `~3.82e-4 m` (single-shell `u_r(R_a) = A·R_a + B/R_a²` with `A = -p / (K + 4μ R_b³/R_a³)`, `B = -A R_b³`) |

The strict-between inequality holds at both the analytic and FEM levels. By Lamé linearity in `(μ, λ)`, `uniform_2x = 0.5 × uniform_1x` exactly at the analytic level (since uniform-2× has every parameter scaled by 2× from uniform-1×); at the FEM level the linearity holds within `~0.24 %` (1.626e-4 vs `0.5 × 3.245e-4 = 1.622e-4` — clean FP noise). Three-shell sits between because two of three shells are at 1× (more compliant than uniform-2×'s all-2× configuration) and one is at 2× (less compliant than uniform-1×'s all-1× configuration); note that three-shell `≠ 0.5 × uniform_1×` because the middle shell's stiffness change is not a uniform scaling.

**Note: IV-5 docstring stale number.** IV-5's `PRESSURE` constant docstring at `tests/concentric_lame_shells.rs:289-293` quotes `~4.6e-4 m` for the cavity-wall analytic; this is a stale pre-implementation estimate — the actual analytic at the row's constants is `3.342e-4 m` per the LU solve (verified bit-exact against IV-5's own runtime `eprintln!` from `iv_5_three_shell_converges_to_piecewise_lame`). Banked as an IV-5 docstring fixup candidate for a follow-up sim-soft commit.

## Cross-references

- **Sister sim-soft examples** (`sdf/stress-test` modules): `sphere_eval` (row 1 — `Sdf` trait contract on `SphereSdf`); `hollow_shell` (row 2 — `DifferenceSdf` of two `SphereSdf`s, the same hollow-body composition row 11's scene constructor uses internally, exposed at the design surface); `sdf_to_tet` (row 3 — single-material `SdfMeshedTetMesh::from_sdf` on a solid sphere, the FEM-mesh counterpart row 11 generalises to multi-material hollow body); `stretch/stress-test`'s `multi_element` (row 6 — uniform-material Dirichlet-stretch counterpart at the same compressible NH baseline; row 11 generalises to internal-pressure BC + multi-material partition); `scalar-field/stress-test`'s `layered` (row 8 — solid 3-shell concentric scene without solver; row 11 ships the solver-driven counterpart at the hollow-body extension) + `blended` (row 9 — `BlendedScalarField` smooth-transition counterpart without solver); `bonded-bilayer-beam` (row 10 — bilayer cantilever-beam EB-composite analytic counterpart; row 11 generalises to 3-shell hollow sphere piecewise-Lamé composite).
- **Internal-fixture template**: `sim/L0/soft/tests/concentric_lame_shells.rs` (IV-5 — three-refinement convergence study at `h, h/2, h/4` against piecewise-Lamé closed forms; row 11 ships the user-facing wrap at the `h/2` middle-refinement). Row 11 mirrors IV-5's scene + load + BC + STATIC_DT + max_newton_iter exactly; the closed-form 6×6 LU solver (`solve_three_shell_lame` + `LameCoefficients::u_r`) is inlined verbatim.
- **`SoftScene::layered_silicone_sphere`**: `sim/L0/soft/src/readout/scene.rs:175-301` — `(material_field, cell_size, pressure) -> (mesh, bc, initial, theta)`. `DifferenceSdf` body of two `SphereSdf`s + half-cell surface-band predicates filtered against `referenced_vertices` (BCC orphan rejection) + per-vertex pressure-traction computation `f_v = pressure · n̂_v · A_v` packed as `LoadAxis::FullVector`. Per-tet `MaterialField` sampling at rest-config centroids populates the `materials()` cache.
- **Closed-form derivation**: see IV-5 module docstring "Closed-form three-shell Lamé under internal pressure with fixed outer surface" section at `tests/concentric_lame_shells.rs:81-124` for the 6×6 derivation. Row 11 reuses `ShellParams`, `LameCoefficients`, `solve_three_shell_lame`, `cavity_wall_mean_analytic` (single-r reading), and `shell_mean_analytic_radial_displacement` (vertex-set-mean reading) verbatim where applicable.
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` Q5 (categorical-colormap heuristic — integer-valued + < 16 unique values → tab10; otherwise sequential viridis).
- **Book reference**: Part 2 §04 §00 ("Linear elastic regime where Neo-Hookean reduces to Lamé"); Part 7 §00 §01 ("Sharp-CSG difference operator", `DifferenceSdf`); Part 7 §02 §00 ("Centroid evaluation for material partition"). Row 11 is the **regression-test artifact** for the bonded-multi-material-with-internal-pressure-load chapter section — the per-shell radial-displacement vs piecewise-Lamé closed-form comparison IS the bonded-multi-material analytic regression that book chapter prescribes at the user-facing example layer.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md).
