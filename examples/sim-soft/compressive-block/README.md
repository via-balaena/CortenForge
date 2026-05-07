# compressive-block

**Compressive-block user-facing wrap — soft cube quasi-statically compressed by a descending rigid plane against a BC-pinned bottom face; per-refinement reaction force compared against the two pure-BC analytic limits + Cauchy convergence.** A `HandBuiltTetMesh::uniform_block` cube of edge `L = 1 cm` is built at three refinement levels (`n ∈ {2, 4, 8}` → `48 / 384 / 3072` tets); the bottom face is full-pinned via `BoundaryConditions::pinned_vertices` (bottom-bonded rigid base), and a single `RigidPlane(n = −ẑ, offset = δ − L)` represents the descended top plate, penetrated by exactly `δ = 5 × 10⁻⁵ m` at rest. One-way `PenaltyRigidContact` with **fixture-local `(d̂, δ)` override** (`d̂ = 1e-5 m`, default `κ = 1e4 N/m`) drives the cube to static equilibrium under the descended plate; `STATIC_DT = 1.0 s` collapses the inertial Tikhonov term so a single backward-Euler step from rest reaches static equilibrium per refinement.

The headline new capability vs row 13's Hertzian sphere-plane is **`PenaltyRigidContact` under bilateral compression with NH-derived stiffness slope assertion** — the third PR2 example row. Row 14 mirrors `sim/L0/soft/tests/penalty_compressive_block.rs` verbatim on constants + scene + bound-bracket gate selection; the contributions vs the internal fixture are: **(a)** finest-refinement deformed-mesh PLY emit with per-vertex `contact_force_z` extras (cf-view auto-colormap renders the contact patch as a bright disk on the cube's top face), **(b)** three-section JSON emit (scalars + per-active top-face vertex `(v, x, y, sd, force_z)` at finest + 11-point F-vs-ε analytic bound curves) for matplotlib overlay, **(c)** opt-in Bevy static-state visualization under `CF_VISUAL=1` — `VIZ_AMPLIFY = 50×` displacement amplifier (small-strain regime is below visual acuity) + thick Cuboid plates positioned flush against the amplified cube faces (a real compression-test rig look, with the penalty contact-band gap absorbed into the visualization) + HUD reading per-refinement ε / F_R / [F_us, F_strain] / Cauchy ratio / E_eff / rel_pos_in_bounds + amplification disclosure.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output: `out/compressive_block.ply` (finest deformed boundary mesh, 729 vertices, 768 triangles via `Mesh::boundary_faces` + per-vertex `contact_force_z` extra) and `out/compressive_block.json` (scalars + per-active top-face vertex contact data + 11-point F-vs-ε analytic bound curves).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The third user-facing example of Tier 4 penalty contact per [`EXAMPLE_INVENTORY.md`][inv] — bilateral compression on cube geometry, the compressive-block fixture's two-bound-plus-Cauchy gate exposed user-facing (regression-test artifact for [`tests/penalty_compressive_block.rs`](../../../sim/L0/soft/tests/penalty_compressive_block.rs)):

```rust
let (mesh, bc, initial, _default_contact) = SoftScene::compressive_block_on_plane(
    EDGE_LEN, cell_size, DISPLACEMENT, &MaterialField::uniform(MU, LAMBDA),
); // (1e-2, EDGE_LEN/{2,4,8}, 5e-5, NH(1e5, 4e5))

// Fixture-local d̂ override.
// κ stays at PENALTY_KAPPA_DEFAULT = 1e4; only d̂ is overridden.
let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE); // (1e4, 1e-5)

let mut cfg = SolverConfig::skeleton();
cfg.dt = 1.0;            // static — collapse M / dt² Tikhonov
cfg.max_newton_iter = 50;

let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);
let step = solver.replay_step(&x_prev, &v_prev, &empty_theta, cfg.dt);
// ... walk top-face vertices, sum penalty force into F_R_FEM,
//     compute λ_z_avg → ε → F_us(ε) and F_strain(ε) ...
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

This is **row 14 of the sim-soft examples arc** — the third Tier 4 penalty-contact example, the third PR2 example row, and the third user-facing demo of `sim-bevy-soft` (under `CF_VISUAL=1`). Row 14 is also the first user-facing consumer of `<HandBuiltTetMesh as Mesh>::boundary_faces`; rows 12 + 13 consumed the trait method on `SdfMeshedTetMesh`.

## Why `F_R ∈ [F_us, F_strain]` is the headline

The original spec named the gate as `< 5 %` rel-err vs uniaxial-stress small-strain `F_us = E · A · ε`, presupposing **pure uniaxial-stress BC** (z-only pin on the bottom face, x/y free). The Phase 5 commit-6 helper `SoftScene::compressive_block_on_plane` full-pins **every** bottom-face vertex (`BoundaryConditions` only models full-vertex Dirichlet; see [`scene.rs:414-423`][s5]), giving a **mixed BC**: bottom full-pinned (constrained-modulus regime locally), sides free (uniaxial-stress regime), top z-contacted. The deformation field is non-uniform; **no clean closed-form exists** for this BC at general aspect ratio.

This row therefore brackets the FEM response by **two pure-BC bounds** at the equilibrium strain `ε`:

- **Lower bound** — uniaxial-stress small-strain `F_us = E · A · ε` (everywhere lateral free; achievable only with z-only-pin BC). The mixed BC must give `F ≥ F_us` because adding lateral constraints (the bottom full-pin) makes the system stiffer, not softer.
- **Upper bound** — uniaxial-strain small-strain `F_strain = M_c · A · ε` where `M_c = E · (1 - ν) / ((1 + ν)(1 - 2 ν)) = λ + 2 μ` (everywhere lateral pin; achievable only with full-pin throughout). The mixed BC must give `F ≤ F_strain` because removing lateral constraints (sides free, top free) makes the system softer.

With `μ = 1e5`, `λ = 4e5` ⇒ `ν = 0.4`, `E = 2 μ (1 + ν) = 2.8e5 Pa`, `M_c = λ + 2μ = 6.0e5 Pa` (note `M_c / E ≈ 2.14×`); the bounds at `ε ≈ 0.6 %` give `F_us ≈ 0.167 N` and `F_strain ≈ 0.359 N`. The FEM at this scene's aspect ratio (cube) settles at `F_R_n8 ≈ 0.182 N` — close to `F_us`. The bottom-pin's lateral constraint is geometrically confined to a thin Saint-Venant boundary layer; most of the cube interior sits in uniaxial-stress regime. `effective_modulus_n8 = F_R_n8 / (A · ε_n8) ≈ 3.04e5 Pa` sits at `rel_pos_in_bounds ≈ 0.076` in `[E, M_c]` — about 7.6 % of the way from uniaxial-stress to uniaxial-strain.

The "NH-derived stiffness slope" framing in the [`EXAMPLE_INVENTORY`][inv] row 14 spec is **mathematically equivalent** to the two-bound bracket: `F_R / (A · ε) ∈ [E, M_c]` ⟺ `F_R ∈ [E·A·ε, M_c·A·ε]` = `[F_us, F_strain]`. The example reports `effective_modulus_n8` and `rel_pos_in_bounds_n8` in stdout + JSON as derived diagnostics; only the F_R bracket is asserted (the modulus framing is a redundant view of the same constraint).

[s5]: ../../../sim/L0/soft/src/readout/scene.rs

Reference: Sokolnikoff, *Mathematical Theory of Elasticity*, 2nd ed., Ch 4 (Saint-Venant principle for boundary-layer effects); Bonet & Wood, *Nonlinear Continuum Mechanics for Finite Element Analysis*, 2nd ed., Ch 5 (small-strain regime). Phase 4 IV-3 (`bonded_bilayer_beam.rs`) precedent for three-refinement-level single-test-fn structure with monotonic + iter-budget asserts + diagnostic `eprintln!`.

## Why the fixture-local `(d̂, δ)` override

At the original spec parameters (`L = 1 cm, δ = 0.5 mm, κ = 1e4 N/m, d̂ = 1 mm, ν = 0.4`), the cold-start penalty residual is `~κ · d̂ ≈ 10 N` per top-face vertex. The raw Newton step is `~residual / κ ≈ 1 mm` per vertex — **10 % of the cube edge**, past the tet-inversion threshold. Armijo's residual-norm sufficient-decrease condition does NOT check element invertibility; line-search accepts trial steps that push elements into NeoHookean's compressive nonlinearity regime, and the next Newton iter's tangent fails Cholesky. Three remediation paths considered:

- **(a) Multi-step rollout with inertial damping.** Requires `dt ≈ 1e-5 s` to make `M / dt²` competitive with `κ_pen`; `~10 000` steps to reach quasi-static. Infeasible test runtime.
- **(b) Global default `κ` reduction.** The Hertzian fixture has separate authority over defaults under sphere geometry; a global reduction would silently affect every other contact-active scene.
- **(c) Fixture-local `(d̂, δ)` override via `PenaltyRigidContact::with_params`.** Policy: *"if defaults fall on the wrong side of the ceiling, surface as a global retune, not a fix-on-the-fly."* Here the ceiling sits comfortably above defaults at the sphere geometry but below at the cube geometry — so a fixture-local override (NOT a global retune) is the right reconciliation.

This row takes (c). Override `d̂ = 1e-5 m` (100× smaller than default) and `δ = 5e-5 m` (10× smaller than the original spec). At the override parameters: cold-start residual `κ · (d̂ + δ) ≈ 0.6 N` per vertex, raw Newton step `~6 × 10⁻⁵ m ≈ 0.6 %` of edge — safely below tet-inversion threshold. Equilibrium strain `ε ≈ 0.6 %` — deep into small-strain regime where the two pure-BC bounds cleanly bracket the FEM response (`≈ 0.182 N` at finest, between `F_us ≈ 0.167 N` and `F_strain ≈ 0.359 N`).

Other contact-active scenes (Hertzian sphere-plane, drop-and-rest, non-interpenetration, grad-hook + rows 12+13) continue to use the default `(κ, d̂)`; this row's override is local to this example only, never propagated upstream. **`KAPPA` stays at the `PENALTY_KAPPA_DEFAULT = 1e4` value** (no κ change, only `d̂`).

## Why three refinements

The convergence story needs three points: per-level two-bound bracket holds + Cauchy ratio `|Δ_fine| / |Δ_coarse| < 1`. Mirrors the compressive-block fixture's `n_per_edge ∈ {2, 4, 8}` choice (cell sizes `5 / 2.5 / 1.25 mm`; tet counts `48 / 384 / 3072` — `HandBuiltTetMesh::uniform_block` decomposes each unit cell into 6 tets). Sub-second release-mode runtime per refinement at the `(d̂, δ)` override. Captured Newton iter counts: `3 / 3 / 3` — flat across refinements (active-set size grows but per-iter residual decreases at the same Newton rate); the `MAX_NEWTON_ITER = 50` cap exists as headroom against material / load perturbations rather than as a tight working budget.

Captured Cauchy ratio: `0.4296` — comfortably below `1.0`, geometric convergence demonstrated. The `F_R` sequence converges to its asymptotic value at a `~2.3×` rate per refinement halving (consistent with first-order quadrature error in the FEM contact integration).

## Why `cargo run --release` only

Mirrors row 13 + row 12 + the compressive-block fixture precedent. The IV-1 captured-bits contract is platform + build-mode-locked; matching row 13's `--release` invocation removes one variable from the determinism contract. The compressive block is fast enough at finest `n=8` that debug-mode is also viable, but consistency with sister rows wins over the cold-build-time savings. Per [`feedback_release_mode_heavy_tests`][rel].

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

Compile-time `const { assert!(...) }` on all input scalars + cell-size monotonicity + refinement-level monotonicity + `DISPLACEMENT < EDGE_LEN` + Newton-iter cap margin + `SMALL_STRAIN_CEILING` / `LAMBDA_Z_FLOOR` domain (`(0, 1)`).

### 2. `mesh_topology_exact`

| Refinement | `n_tets` | `n_vertices` | `n_loaded` | `n_pinned` |
|---|---|---|---|---|
| n=2 | 48   | 27  | 0 | 9  |
| n=4 | 384  | 125 | 0 | 25 |
| n=8 | 3072 | 729 | 0 | 81 |

III-1 determinism contract per refinement. Captured 2026-05-06 on macOS arm64 in `--release` build. **`n_loaded == 0` exact** is compressive-block-specific — no external traction; load is penalty-mediated. **`n_pinned = (n+1)²`** because the helper full-pins all bottom-face vertices.

### 3. `boundary_partition`

Per refinement: `n_pinned > 0` (bottom-face full-pin must be non-empty) and `n_loaded == 0` exact (compressive-block guarantee — load is penalty-mediated, no external traction).

### 4. `solver_per_step_invariants`

Per refinement: no NaN in `x_final`; `iter_count < NEWTON_ITER_SANITY_CAP = 40` (10-iter margin under `MAX_NEWTON_ITER = 50` per compressive-block policy); finite residual norm. Captured `iter_count`: `3, 3, 3` at (n=2, n=4, n=8).

### 5. `contact_engagement`

Per refinement: `n_active_pairs == (n+1)²` exact. Captured `9, 25, 81` at (n=2, n=4, n=8) — every top-face vertex inside the `d̂`-band at equilibrium per the compressive-block fixture's docstring (cube compresses by `~6e-5 m` from `δ = 5e-5 m` plate displacement plus `d̂_override = 1e-5 m` band engagement, leaving every top vertex with a small positive `sd` strictly less than `D_HAT_OVERRIDE`). Stronger than row 13's `n_active > 0` since the compressive block's geometry guarantees uniform top-face penetration.

### 6. `small_strain_validity`

Per refinement: `0 < ε < SMALL_STRAIN_CEILING = 0.10` (10× cap over expected `ε ≈ 0.6 %`). Catches a regression where the `(d̂, δ)` override no longer produces a small-strain regime. Captured: `0.578 % / 0.593 % / 0.598 %` — well into small-strain.

### 7. `gross_physics_per_level`

Per refinement: `λ_z ∈ (LAMBDA_Z_FLOOR, 1.0) = (0.5, 1.0)` (cube compresses, not extends; `λ_z > 0.5` rules out >50% compression as physically implausible at the `(κ, d̂, δ)` regime); `F_R > 0` (soft body pushes UP on rigid plane — Newton's 3rd-law partner of penalty's DOWN force on top face). Catches sign-flip regressions at gross-physics level before the bound asserts surface them numerically.

### 8. `two_bound_per_level` (HEADLINE)

Per refinement: `F_us ≤ F_R_FEM ≤ F_strain` at that level's equilibrium ε.

| Refinement | `ε`     | `F_us` (N) | `F_R_FEM` (N) | `F_strain` (N) |
|---|---|---|---|---|
| n=2 | 5.78e-3 | 0.1620 | 0.1944 | 0.3470 |
| n=4 | 5.93e-3 | 0.1659 | 0.1856 | 0.3555 |
| n=8 | 5.98e-3 | 0.1674 | **0.1819** | 0.3587 |

The headline analytical-bracket gate. F_R sits close to F_us (Saint-Venant boundary-layer regime — bottom-pin's lateral constraint confined to thin layer; cube interior in uniaxial-stress regime). Equivalently in stiffness-modulus units: `effective_modulus_n8 = 3.04e5 Pa ∈ [E = 2.8e5, M_c = 6.0e5]` with `rel_pos_in_bounds_n8 ≈ 0.076`.

### 9. `cauchy_f_r_convergence`

`|F_R_n4 − F_R_n8| < |F_R_n2 − F_R_n4|` (Cauchy ratio `< 1`). Captured ratio `0.4296`. Stronger than monotonic-only — catches "bounded but non-converging" regressions where the F_R sequence stays bounded but doesn't actually converge.

### 10. `captured_bits_compressive_metrics`

Eleven `f64` compressive metrics + six `usize` counts captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `λ_z_avg` per refinement | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `ε` per refinement | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `F_R_FEM` per refinement | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `cauchy_ratio` | rel `1e-12`, abs `1e-12` (`f64`) |
| `effective_modulus_n8` | rel `1e-12`, abs `1e-12` (`f64`) |
| `n_active_pairs` per refinement | strict equality (3 × `usize`) |
| `iter_count` per refinement | strict equality (3 × `usize`) |

Captured drift-detection contract: `~few thousand tets through faer's sparse Cholesky` lives in IV-1's sparse-at-scale tier. The contract is **relative tolerance, not strict bit-equality** for the floats — observed bits captured for regression detection, compared via `assert_relative_eq!` at `1e-12` rel. Capture provenance + failure-mode protocol (rule out toolchain drift before re-baking) inline above the const blocks. Same precedent as PR1 rows 6+10+11 + rows 12 + 13.

This is the row's **tight regression-detection gate** — anchors 2-9 document physical reasonableness; anchor 10's `1e-12` rel against pinned bits catches any FEM-numerics-shifting regression at machine precision.

## Visuals

### PLY artifact

`out/compressive_block.ply` — finest-refinement (n=8) deformed boundary mesh with per-vertex `contact_force_z` extra:

```text
729 vertices    (finest deformed positions in physics +Z frame; (n+1)³ grid from
                 HandBuiltTetMesh::uniform_block — every vertex referenced by ≥ 1 tet,
                 no BCC-orphan corners unlike rows 12 + 13's SdfMeshedTetMesh)
768 faces       (boundary triangulation via Mesh::boundary_faces — outward-CCW winding
                 from the right-handed-tet `signed_volume > 0` invariant per
                 boundary_faces_from_topology)
normals         : smooth, area-weighted via AttributedMesh::compute_normals
extras["contact_force_z"]
                : f32 per vertex; zero everywhere except 81 active top-face vertices
                  where `force_z = κ · (d̂ - sd) > 0`. cf-view's auto-colormap
                  (sequential viridis on positive scalar) renders the contact patch
                  as a uniform bright disk on the deformed cube's top face.
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/compressive-block/out/compressive_block.ply
```

### Bevy static-state visualization (CF_VISUAL=1)

```text
CF_VISUAL=1 cargo run -p example-sim-soft-compressive-block --release
```

Spawns an `OrbitCamera` scene with four entities (rendered at `RENDER_SCALE = 100×` physics scale per the section below):

- **Soft mesh** (`Mesh3d` + `MeshMaterial3d` + `Trajectory`) — coral PBR cube built via `sim_bevy_soft::mesh::build_soft_mesh` from the rest configuration + `Mesh::boundary_faces()` triangulation, animated to the captured n=8 deformed positions on first `step_replay` tick (single-frame trajectory; `frame_index_at` clamps at end). `Transform::from_scale(100)` so cm-scale physics renders at meter scale.
- **Bottom plate** (`Mesh3d` + `MeshMaterial3d`) — gray PBR cuboid (`Cuboid::new`), `1.5 × EDGE_LEN · RENDER_SCALE = 1.5 m` lateral (xz) × `0.08 × EDGE_LEN · RENDER_SCALE = 0.08 m` thick (y). The cuboid's TOP face sits **flush against the cube bottom** at Bevy `y = 0` (cube bottom is BC-pinned at physics z=0, doesn't move under load), with a sub-mm `PLATE_ZFIGHT_OFFSET` to prevent depth-buffer z-fight. **The bottom plate is purely visual** — physics has no penalty contact at the bottom face; the bottom face is BC-pinned in `BoundaryConditions::pinned_vertices`. The visualization shows two plates because the [`EXAMPLE_INVENTORY`][inv] row 14 framing is "soft cube between two RigidPlanes," and the BC-pin is the nearest-representable analog of a perfectly bonded rigid bottom plate (Phase 5 `BoundaryConditions` only models full-vertex Dirichlet); the bottom plate carries no penalty force in the physics.
- **Top plate** (`Mesh3d` + `MeshMaterial3d`) — same gray PBR cuboid mesh + material; cuboid's BOTTOM face sits **flush against the AMPLIFIED cube top** at Bevy `y ≈ 0.7 m` at `VIZ_AMPLIFY = 50` (cube rest top at `EDGE_LEN · RENDER_SCALE = 1.0 m` minus amplified compression `VIZ_AMPLIFY · ε_n8 · EDGE_LEN · RENDER_SCALE ≈ 0.3 m`), with the same sub-mm z-fight offset. **NOT** at the kinematic `δ`-offset position the physics solver uses — the penalty model's `sd ≈ 9.78 μm` contact-band would inflate to ~5 cm Bevy under amplification and dominate the visual story. The band is an FEM no-penetration-enforcement implementation detail tangential to the row's "two plates squishing a cube" pedagogy, so it's absorbed into the visualization layer. The actual physical `RigidPlane` (penalty contact in the headless solver) sits at `z = EDGE_LEN − δ ≈ 9.95 mm` physics, but its visual rendering is shifted to flush against the amplified cube top so the rig looks like a real compression test.
- **HUD** (`Text` + `Node`) — top-left, ASCII-only multi-line: per-refinement ε / F_R / [F_us, F_strain] / Cauchy ratio / E_eff / rel_pos_in_bounds + `VIZ_AMPLIFY` disclosure line. The first six lines read 1:1 with the asserted scalars; the disclosure line declares the visual amplification so the cube's apparent ~30% z-compression is read as visualization-only, not as a contradiction of the asserted `ε ≈ 0.6 %`.

Plus a directional-light at `12 klx` from upper-front-right + per-camera `AmbientLight` at `80 cd/m²` (mirror rows 12 + 13). Controls overlay sits at bottom-left.

The scene is **static** — single quasi-static step with no temporal evolution. `R` is a no-op for this row (no animation to reset). Default camera distance `~3 m` per pattern (l) full-scene-first — full cube + both plates fit comfortably; HUD readout makes the bound-bracket numbers visible regardless of camera position.

### Controls

| Key | Action |
|---|---|
| **Left-drag** | Orbit camera around the cube |
| **Mouse scroll** | Zoom |
| **Right-drag** | Pan |
| **Close window** | Exit the app |

### Why the rendered scene is `100×` simulation scale

Mirrors rows 12 + 13 verbatim. Bevy 0.18's pipeline defaults (near plane `0.1 m`, OrbitCamera `min_distance = 0.1 m`, AmbientLight brightness, depth precision) were tuned for human-scale (1 m+) scenes. At cm-scale rendering, the camera approaches the default near plane on any zoom-in and clips the geometry; lifting the rendered scene to meter scale puts everything safely past the defaults. **Headless asserts + JSON + PLY are scale-invariant** — they operate on unscaled physics positions, so this is a visualization-only adjustment.

### Why deformations are amplified by `VIZ_AMPLIFY = 50×`

Banked pattern (b) per row 10's [`feedback_visual_review_is_the_test`][vrt]. the compressive block's small-strain regime — `ε ≈ 0.6 %` finest-equilibrium per the headline gate — produces 60 μm cube compression on a 1 cm physical edge. At `RENDER_SCALE = 100×` the visual cube is 1 m tall and the compression is 6 mm — viewed from the default `~3 m` camera distance, this subtends `~2 mrad ≈ 0.1°`, well below the human visual acuity threshold (`~17 mrad ≈ 1 arc-min`). At rest configuration, the cube would appear visually identical to its compressed equilibrium configuration; the asserted physics is invisible.

`VIZ_AMPLIFY = 50` multiplies the displacement-from-rest of every cube vertex (`viz_pos = rest + VIZ_AMPLIFY · (x_final − rest)`) before the trajectory replay writes positions to the Bevy mesh. **Plates are positioned flush against the amplified cube faces** (top plate's bottom face at amplified cube top y; bottom plate's top face at cube bottom y=0; both with a sub-mm z-fight offset) — NOT at the kinematic `δ`-offset positions the physics solver uses. The penalty model's `sd ≈ 9.78 μm` contact-band would inflate to ~5 cm Bevy under amplification and dominate the visual story; the band is an FEM no-penetration-enforcement implementation detail tangential to the row's pedagogy, so it's absorbed into the visualization layer. The amplified scene reads as `~30%` apparent z-compression + `~12%` lateral Poisson bulge with both plates clearly in contact — pedagogically faithful to "two plates squishing a cube."

**Headless asserts + JSON + PLY are NOT amplified** — they read the true `x_final` from the solver. The HUD's last line declares the amplification factor: `VIZ_AMPLIFY = 50× (cube + plates amplified for visibility; plates flush against cube; numbers above are TRUE physics)`. The visual-vs-numerical contract is explicit; the cube's apparent squish is a visualization choice, not a physics claim. If you want to inspect the un-amplified deformation directly, open `out/compressive_block.ply` in cf-view (which renders the true `x_final` positions; the `~60 μm` cube compression at finest equilibrium reads as the contact-force colormap on the top face rather than as a perceptible squish on the boundary mesh — at the cube's 1 cm edge length, 60 μm is 0.6 % and below visual acuity). The `sd ≈ 9.78 μm` penalty-band gap appears in `out/compressive_block.json`'s per-vertex array for anyone wanting numerical inspection.

[vrt]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md

### Matplotlib plot

`uv run examples/sim-soft/compressive-block/plot.py`

Single-panel F-vs-ε comparison: two analytic bound lines (`F_us(ε) = E · A · ε` lower in red, `F_strain(ε) = M_c · A · ε` upper in blue) + 3 FEM scatter points (one per refinement) clustered near `ε_n8 ≈ 0.6 %`. The shaded region between the two bound lines is the **physically valid range** for the mixed BC; FEM scatter points falling inside the shaded region IS the asserted two-bound bracket. Title shows the asserted Cauchy ratio + effective modulus + `rel_pos_in_bounds`.

Per [`feedback_visible_contacts`][vis] — for Tier 4 penalty contact rows, the visual review is real (not collapsed to JSON read).

[vis]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, per-refinement results table, Cauchy convergence + effective modulus summary).

## Run

```text
cargo run -p example-sim-soft-compressive-block --release
```

Output: `out/compressive_block.ply`, `out/compressive_block.json`, and stdout summary.

```text
CF_VISUAL=1 cargo run -p example-sim-soft-compressive-block --release
```

Output: same as above + Bevy windowed static-state visualization.

```text
uv run examples/sim-soft/compressive-block/plot.py
```

Output: `out/compressive_block.png` (F-vs-ε scatter + analytic bound lines).

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for this example. Sub-second release-mode total runtime across the three refinements; debug-mode is also viable but consistency with sister rows wins over the cold-build-time savings.

## Cross-references

- **Sister sim-soft examples**: `soft-drop-on-plane` (row 12 — drop-and-rest), `hertz-sphere-plane` (row 13 — Hertzian sphere-plane analytical-comparison; closest precedent for static-state Bevy rendering + IV-1 sparse-tier captured-bits contract); future `contact-force-readout` (row 18 — quantitative readout for the calibration loop, blocked on `PenaltyRigidContact::contact_pairs()` accessor).
- **Internal-fixture template**: `sim/L0/soft/tests/penalty_compressive_block.rs` (penalty-contact bilateral-compression gate; the simpler-geometry warmup before the Hertzian fixture). Row 14 mirrors the fixture's scene + load + BC + DT + (d̂, δ) override + cell sizes + two-bound + Cauchy gates verbatim. The user-facing extensions are the per-vertex force PLY, three-section JSON + plot.py, and the Bevy static-state visualization.
- **`SoftScene::compressive_block_on_plane`**: `sim/L0/soft/src/readout/scene.rs:377-454` — `(edge_len, cell_size, displacement, &material_field) -> (mesh, bc, initial, contact)`. `HandBuiltTetMesh::uniform_block` cube, full-pin BC on bottom face, no external traction (load is penalty-mediated), default-κ default-d̂ `PenaltyRigidContact` (replaced here by fixture-local `with_params`).
- **`Mesh::boundary_faces`** for `HandBuiltTetMesh`: `sim/L0/soft/src/mesh/hand_built.rs:358-359` — outward-CCW boundary-face cache populated at construction via `boundary_faces_from_topology`. Row 14 is its first user-facing consumer on `HandBuiltTetMesh` (rows 12 + 13 used `SdfMeshedTetMesh`).
- **`sim-bevy-soft`**: `sim/L1/sim-bevy-soft/` — Bevy soft-body trajectory replay (single-frame trajectory at row 14 makes this a static-state render). Row 14 is its third consumer after rows 12 + 13; the row 12-fixed foundation patches (ReplayEpoch + R-key reset + area-weighted normals) are inherited unchanged.
- **`cf-bevy-common`**: `cf-bevy-common/` — shared Bevy 0.18 helpers (`UpAxis` input → Bevy frame swap, `OrbitCamera` + `OrbitCameraPlugin`).
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` — cf-view static-artifact viewer the headless PLY targets (auto-colormap on positive `extras` scalar; `RENDER_SCALE` auto-fit from cf-view C1 handles cm-scale meshes natively).
- **Book reference**: Part 4 §00 §00 ("Penalty contact pathology and validation scope"). Row 14 is the **regression-test artifact** for the bilateral-compression validation chapter section — the two-bound-bracket-plus-Cauchy gate, exposed user-facing at the example layer with the deformed-mesh + per-vertex contact-force visualization that the internal fixture omits. Saint-Venant boundary-layer doctrine cited in-line via Sokolnikoff Ch 4 (above).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visible_contacts`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md),
  [`feedback_release_mode_heavy_tests`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md).
