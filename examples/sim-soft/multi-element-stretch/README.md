# multi-element-stretch

**Phase 2 multi-element FEM assembly under uniform Dirichlet stretch — homogeneous deformation gradient `F = diag(λ, 1, 1)` is uniform across all 48 tets within FP tolerance.** A 27-vertex / 48-tet hex grid (`HandBuiltTetMesh::uniform_block(2, 0.1, ...)`) has its 26 boundary vertices Dirichlet-pinned to `D · X_rest` with `D = diag(λ, 1, 1)`, `λ = 1.20`; the single interior vertex (ID 13 at the cube's center) is left free and starts at REST so Newton has actual work to do. The solver converges to the homogeneous deformation `x = D · X` for all 27 vertices, and per-tet `F` evaluates to `diag(λ, 1, 1)` for every one of the 48 tets — empirically bit-equal on the capture platform across all tets including the 24 that border the interior free DOF. Every claim sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/multi_element_stretch.json` (48-record per-tet trace + uniformity verdict, JSON-only per inventory Q4 row 6).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The "homogeneous Dirichlet boundary data uniquely determines the homogeneous deformation" mechanics result, gated on Phase 2's multi-element FEM solver doing the work end-to-end:

```rust
// 27-vertex / 48-tet hex grid, uniform NH (Ecoflex-class).
let mesh = HandBuiltTetMesh::uniform_block(2, 0.1, &MaterialField::uniform(MU_PA, LAMBDA_PA));

// Pin every boundary vertex to D · X_rest; leave the lone interior vertex free.
let boundary: Vec<VertexId> = pick_vertices_by_predicate(&mesh, on_boundary)
    .into_iter()
    .filter(|v| referenced_vertices(&mesh).contains(v))   // canonical orphan filter
    .collect();
let bc = BoundaryConditions { pinned_vertices: boundary, loaded_vertices: vec![] };

// Quasi-static config: density = 0 to suppress backward-Euler inertia.
let mut cfg = SolverConfig::skeleton();
cfg.density = 0.0;

// x_prev: 26 boundary at D · X_rest, vertex 13 at REST (off-equilibrium).
// Newton iterates → vertex 13 lands at D · v_13_rest within tol; per-tet F bit-equal D.
let solver: CpuTet4NHSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
let step = solver.replay_step(&x_prev, &v_prev, &Tensor::from_slice(&[], &[0]), cfg.dt);
```

For compressible NH with shear modulus `μ` and first Lamé parameter `Λ`, at `F = diag(λ, 1, 1)` the closed forms are:

```text
I_1   = λ² + 2
J     = λ
ψ     = (μ/2)(λ² − 1) + (Λ/2)(ln λ)² − μ ln λ
P_11  = μ(λ − 1/λ) + Λ ln(λ) / λ
P_22  = P_33 = Λ ln(λ)               (NOT zero — constrained, not traction-free)
F_ij  = 0  for i ≠ j                 (diagonal F)
```

This is **row 6 of the sim-soft examples arc** ([`EXAMPLE_INVENTORY.md`][inv]) — the multi-element solver-driven counterpart to row 5 (`neo-hookean-uniaxial`, single-tet direct-eval). Row 5 strips down to `Material::first_piola(F)` direct evaluation; row 6 imposes `F` via Dirichlet pinning of every boundary vertex on a 48-tet mesh and runs the full Phase 2 multi-element FEM pipeline (per-tet mass + sparse pattern build at construction, per-iter `assemble_global_int_force` + `assemble_free_hessian_triplets` + sparse Cholesky on the 3-DOF free block).

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

**Why quasi-static (`density = 0`).** Backward Euler with non-zero mass and `v_prev = 0` would NOT converge to the pure-static homogeneous deformation in one step: the `(M / Δt²)·(x − x_prev)` inertia recall pulls vertex 13 toward its rest position, so the converged interior position would sit between rest and `D · v_13_rest`. Setting `density = 0` zeros the lumped mass entirely and reduces the residual to pure elasticity (`f_int(x) − f_ext = 0`). For homogeneous neo-Hookean under affine Dirichlet boundary data, the static minimizer of NH energy IS the homogeneous deformation `x = D · X` — ∇·P = 0 holds trivially when P is constant, so the equilibrium is uniquely determined by the boundary positions. Recovered to within Newton's `tol = 1e-10` N residual bound.

**Why constrained `F = diag(λ, 1, 1)`** (rather than row 5's traction-free `diag(λ, λ_t, λ_t)`): under Dirichlet pinning we control all boundary positions exactly, so transverse `λ_t = 1` is *prescribed* by the pin — there's no traction-free natural BC to enforce, and the constrained form has clean closed forms with no inner Newton. The non-zero transverse stress `P_22 = Λ ln(λ) ≈ 7.29e4 Pa` is the pedagogical cue that this is constrained-uniaxial (think clamped-side stretching), not row 5's traction-free benchmark.

**Why solver-driven (vs row 5's direct-eval).** Row 5's subject is the constitutive surface — `Material::first_piola(F)` evaluated directly at hand-constructed `F`. Row 6's subject is the multi-element FEM assembly path: per-tet local→global stitch via `mesh.tet_vertices(tet_id)` indexing, mass aggregation across tets sharing vertices, sparse-pattern construction with shared-vertex coupling, Newton iteration on the free-DOF block. With `n_free = 3` (the interior vertex's xyz), the solver's full assembly cache builds at construction, and Newton runs 3 iterations on the 3×3 free Hessian — small but exercises the same code path as larger meshes.

**Why JSON-only (no `cf-view`)**: per inventory Q4 row 6, the per-tet uniformity record IS the readout. 48 tets at essentially the same value with sub-pixel deviations would be a visually trivial bar chart; stdout's max/min/spread summary plus the per-tet JSON array carries the information cleanly. A future plot.py (analogous to row 5's) could histogram per-tet P_11 to show the distribution's tightness, but for `λ = 1.20` the spread is empirically `0.0` Pa — there's no distribution to visualize.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 11 groups are called from `main()` in dependency order; cargo `--release` exit-0 means every assert passed.

### 1. `referenced_vertices_full`

| Anchor | Bound |
|---|---|
| `referenced_vertices(&mesh).len()` | exact-pin to `27` |
| `referenced_vertices(&mesh)[i] == i` | every i in `0..27` |

Hand-built meshes have no orphans by construction (every vertex appears in at least one tet). The orphan filter inside [`build_scene`][bs] is therefore a no-op here, but exercising it keeps the canonical SDF-meshed-scene BC-construction pattern visible: spatial predicate → orphan-filter → pin set. SDF-meshed scenes (Phase 3+) need the filter to reject unreferenced BCC-lattice corners outside the SDF zero set.

[bs]: src/main.rs

### 2. `mesh_topology`

| Anchor | Bound |
|---|---|
| `mesh.n_vertices()` | exact-pin to `27` |
| `mesh.n_tets()` | exact-pin to `48` |
| `mesh.positions()[13]` | center `(L/2, L/2, L/2)` at relative `1e-12` |

Vertex ID 13 is the lone interior vertex — `vid(1, 1, 1) = 1 + 1·3 + 1·9 = 13` per the hex-grid stride formula. Its rest position must be the cube's center.

### 3. `boundary_partition`

| Anchor | Bound |
|---|---|
| `boundary.len()` | exact-pin to `26` (= `N_VERTICES − 1`) |
| `boundary.contains(13)` | must be `false` |
| `boundary[i] < boundary[i+1]` | strict ascending |

Ascending order is load-bearing for the deterministic free-DOF index map the solver builds (`backward_euler.rs:362-376` walks vertices in natural order). Both `pick_vertices_by_predicate` and `referenced_vertices` walk in ascending `VertexId` order, so the filter result should also be ascending — a regression that introduced HashMap-based collection anywhere upstream would surface here.

### 4. `validity_in_bounds_at_x_prev`

| Anchor | Bound |
|---|---|
| `mat.validity().max_stretch_deviation` | exact-pin to `1.0_f64` (`to_bits` equality) |
| `mat.validity().inversion` | `matches!(_, InversionHandling::RequireOrientation)` |
| Per-tet `max\|σᵢ − 1\|` at `x_prev` | strict `<` `1.0` |

The off-equilibrium initial guess (boundary stretched, interior at rest) deforms the 24 v_13-adjacent tets non-uniformly. At `λ = 1.20`, the worst-case adjacent-tet `F` has principal stretches roughly `(1.246, 1, 0.928)` — `max|σᵢ−1| ≈ 0.246`, well inside NH's `1.0` boundary. The strict bound here gates the initial guess — if `LAMBDA_STRETCH` were pushed close to `1.0`, this anchor would fail before the solver ran.

### 5. `solver_converges`

| Anchor | Bound |
|---|---|
| `step.iter_count` | strict `<` `cfg.max_newton_iter` |
| `step.final_residual_norm` | strict `<` `cfg.tol` (= `1e-10` N) |

Empirically converges in **3 Newton iterations** with final residual `≈ 1e-14` N (4 orders below tol) — tight quadratic convergence on the 3-DOF free Hessian.

### 6. `pinned_x_final_exact`

| Anchor | Bound |
|---|---|
| Per pinned DOF: `step.x_final[i].to_bits() == x_prev[i].to_bits()` | bit-equal |

The solver leaves pinned DOFs bit-equal to `x_prev` (mechanism: `armijo_backtrack` only updates `trial_x[full_idx]` for indices in `free_dof_indices`; pinned DOFs are never written to). Bit-equality is the contract here — any drift signals a regression in the pin-vs-free DOF dispatch.

### 7. `interior_x_final_close_to_homogeneous`

| Anchor | Bound |
|---|---|
| `\|x_final[v_13] − D · v_13_rest\|` | per-axis relative `1e-12` |

Vertex 13 lands at `D · v_13_rest = (0.06, 0.05, 0.05)` within Newton's residual-tol-implied position floor (~`1e-15` m on a `~1e4` N/m elastic tangent). Empirically observed at `|Δ| = 0.0` exact — the 3-DOF Newton converges to the homogeneous solution bit-equal at this scale.

### 8. `per_element_F_uniform_diag`

| Anchor | Bound |
|---|---|
| Per tet `F[(0,0)]` vs `LAMBDA_STRETCH` | relative `1e-12` |
| Per tet `F[(1,1)]`, `F[(2,2)]` vs `1.0` | relative `1e-12` |
| Per tet `F[(i,j)]` for `i ≠ j` | absolute `<` `1e-12` |

Every one of the 48 tets — including the 24 that border vertex 13 (whose final position came out of the sparse-Cholesky path) — has `F` matching `diag(λ, 1, 1)` within rel-tol. The 24 isolated tets (no v_13 vertex) have `F` *bit-equal* to `D` exactly, since their 4 verts are pinned and the per-tet `F = J · J_0^-1` is computed entirely from frozen boundary-position bits.

### 9. `per_element_stress_closed_form`

| Anchor | Bound |
|---|---|
| Per tet `P_11` vs `analytic_p11(LAMBDA_STRETCH)` | relative `1e-12` |
| Per tet `P_22`, `P_33` vs `analytic_p22(LAMBDA_STRETCH) = Λ ln(λ)` | relative `1e-12` |
| Per tet `ψ` vs `analytic_psi(LAMBDA_STRETCH)` | relative `1e-12` |
| Per tet `P[(i,j)]` for `i ≠ j` | absolute `<` `1e-12` |

`P_11(1.20) ≈ 9.744e4` Pa, `P_22(1.20) ≈ 7.293e4` Pa (note: non-zero — constrained, not traction-free), `ψ(1.20) ≈ 1.042e4` J/m³.

### 10. `uniformity_spread_bounded`

| Anchor | Bound |
|---|---|
| `max P_11 − min P_11` across all 48 tets | strict `<` `1e-6` Pa |

The headline uniformity claim. Empirically observed at exactly `0.0` Pa — every tet's `P_11` is bit-equal `9.744051893131819e4` on the capture platform. Bound at `1e-6` Pa = 11 orders below the closed-form value, generous slack for any cross-platform sparse-Cholesky drift while catching real regressions at `>= 1` Pa.

### 11. `captured_bits`

| Anchor | Bound |
|---|---|
| Interior `x_final` (3 entries) vs `INTERIOR_X_FINAL_REF_BITS` | relative `1e-12` |
| Representative isolated tet's `F_diag` (3 entries) vs `REPRESENTATIVE_F_DIAG_REF_BITS` | relative `1e-12` |
| Representative isolated tet's `P_diag` (3 entries) vs `REPRESENTATIVE_P_DIAG_REF_BITS` | relative `1e-12` |
| Representative isolated tet's `ψ` vs `REPRESENTATIVE_PSI_REF_BITS` | relative `1e-12` |

10 captured-reference anchors. The representative tet is `tet_id = 6` (verts `[1, 2, 5, 14]` — a tet in cell `(1, 0, 0)` that does NOT contain v_13, so its `F` is computed entirely from frozen pinned positions and is bit-equal `diag(λ, 1, 1)` on a fixed toolchain). The interior `x_final` reference and the representative tet's `P_diag` / `ψ` reference exercise the captured-NH-eval path; the `F_diag` reference exercises the per-tet `F = J · J_0^-1` arithmetic.

**Capture provenance.** Captured on 2026-05-05 at sim-soft `dev` HEAD (preceding row 6 commit), rustc 1.95.0 (`59807616e` 2026-04-14) — the same toolchain IV-1 captured at sim-soft `c3729d4a` per [`invariant_iv_1_uniform_passthrough.rs:138-151`][iv1] — on macOS arm64.

[iv1]: ../../../sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs

**IV-1 sparse-tier contract.** This row's 81-DOF FEM solve through faer's sparse Cholesky lives between IV-1's dense bit-equal tier (12-24 DOFs, `nalgebra::Matrix3` scalar arithmetic, bit-equal across rustc minor versions AND across `(macOS arm64, Linux x86_64)`) and IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift on faer's per-column FMA-fusion path). At this scale the cross-platform behavior is empirically untested, so the contract is **relative tolerance**, not strict bit-equality. The capture-platform observed values ARE bit-equal across all 48 tets (uniformity spread `0.0` Pa) — but the contract reserves `1e-12` rel-tol for cross-platform sparse-solver SIMD/FMA noise.

**Failure-mode protocol** (mirrors IV-1's): if the rel-tol comparison fails, do NOT re-bake. Diagnose in this order:

1. Rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture).
2. If same toolchain, real regression — identify which sim-soft commit altered the multi-element FEM assembly numerics OR the sparse-Cholesky path through faer.
3. NEVER re-bake the reference values to make the test green. Spurious re-capture hollows the contract to a tautology.

## Visuals

`out/multi_element_stretch.json` is the canonical inspection artifact. Top-level shape:

```text
{ "scene":               { mesh, material, solver_config, boundary_condition },
  "expected_homogeneous": { F_diag, P_11, P_22, P_33, psi, v13_stretched },
  "step_result":          { iter_count, final_residual_norm_N, v13_observed, v13_residual_vs_homogeneous_m },
  "uniformity":           { p11_min, p11_max, p11_spread_Pa, p11_spread_bound_Pa },
  "tets":                 [ /* 48 records */ ] }
```

Per-tet record:

```text
{ "tet_id":                  6,
  "verts":                   [1, 2, 5, 14],
  "contains_interior_vertex": false,
  "F_diag":                  [1.2, 1.0, 1.0],
  "F_offdiag_max_abs":       0.0,
  "P_diag":                  [97440.52, 72928.62, 72928.62],
  "P_offdiag_max_abs":       0.0,
  "psi":                     10416.07,
  "max_stretch_deviation":   0.20 }
```

Read directly or via `jq`:

```text
jq '.uniformity'                                                                  out/multi_element_stretch.json
jq '.expected_homogeneous'                                                        out/multi_element_stretch.json
jq '.tets | map({tet_id, contains_interior_vertex, P_11: .P_diag[0]})'            out/multi_element_stretch.json
jq '.tets | group_by(.contains_interior_vertex) | map({key: .[0].contains_interior_vertex, count: length, P_11_spread: ([.[].P_diag[0]] | max - min)})'  out/multi_element_stretch.json
```

The last query buckets tets into adjacent-vs-isolated and reports per-bucket P_11 spread — a cleaner "are the v_13-adjacent tets perturbed by the sparse-solver path" diagnostic than the global spread.

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 11 anchor-group names, expected values, per-tet uniformity verdict).

## Run

```text
cargo run -p example-sim-soft-multi-element-stretch --release
```

Output: `out/multi_element_stretch.json` (schema above) and stdout summary.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The captured-bit reference was captured under release-mode build; matching that invocation shape removes one variable from the (rel-tol) contract. Debug builds may also fire `debug_assert!`s inside `nalgebra` or the solver that release elides.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Cross-references

- **Sister sim-soft examples**: `single-tet-stretch` (row 4 — single-tet `Solver::step` end-to-end with backward-Euler Newton; the smallest FEM-running scene in the arc, which row 6 generalizes from 1 tet to 48), `neo-hookean-uniaxial` (row 5 — single-tet *direct-eval* on the same canonical scene, traction-free `F = diag(λ, λ_t, λ_t)` with inner Newton on `λ_t`; companion to this row's solver-driven constrained `F = diag(λ, 1, 1)` form). See `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 2.
- **`HandBuiltTetMesh::uniform_block`**: `sim/L0/soft/src/mesh/hand_built.rs:281-310` — Phase 5 V-3a constructor; this is its first user-facing example consumer (Phase 5's invariant tests use it for compressive-block scenes).
- **`pick_vertices_by_predicate` + `referenced_vertices`**: `sim/L0/soft/src/readout/scene.rs:899-908` and `sim/L0/soft/src/mesh/mod.rs:212-222` — the canonical BC-construction idiom. Hand-built meshes have no orphans, so the filter is a no-op here; SDF-meshed scenes (rows 8+ Tier 3) use it to reject unreferenced BCC-lattice corners.
- **`CpuNewtonSolver` Phase 2 multi-element assembly**: `sim/L0/soft/src/solver/backward_euler.rs:200-429` — the constructor walks every tet building `mass_per_dof`, `element_geometries`, sparse-pattern `triplet_set`, and the symbolic Cholesky factor. This row is the smallest user-facing example exercising that path on a true multi-tet mesh (rows 4 + 5 are single-tet).
- **Companion gradcheck tests**: `sim/L0/soft/tests/multi_element_grad_scaling.rs` (II-2 gradient aggregation across 2 isolated tets) and `multi_element_isolation.rs` (II-1 multi-element determinism). This row is the Tier-2 Dirichlet-stretch counterpart to those θ-driven gradient tests.
- **IV-1 sparse-tier contract**: `sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs` — the "two-tier contract" framing this row's `captured_bits` anchor inherits, with the rel-tol bar applied per the sparse-tier noise floor.
- **Validity-domain solver enforcement**: `sim/L0/soft/src/solver/backward_euler.rs:431-523` — `check_validity_at_step_start` panics on `max|σᵢ−1| > NH bound 1.0`. Anchor 4 above (`validity_in_bounds_at_x_prev`) replicates this check on the off-equilibrium initial guess to gate the scene before the solver runs.
- **Book references**: Part 2 Ch 04 [00-energy.md](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/00-energy.md) (closed-form ψ derivation for general `F`; the constrained `F = diag(λ, 1, 1)` specialization here is inline in the module docstring); Part 8 §00 (Phase 2 multi-element FEM assembly).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_pass_collapses_for_json_rows`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_pass_collapses_for_json_rows.md).
