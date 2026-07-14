# bonded-bilayer-beam

**Phase 4 IV-3 user-facing wrap — shared-vertex multi-material cantilever beam under tip load, with Euler-Bernoulli composite-beam analytic comparison + uniform-baseline asymmetry gate.** A `(0.5, 0.1, 0.1) m` cantilever beam at `(NX, NY, NZ) = (20, 8, 8)` (1701 verts, 7680 tets) is meshed via `HandBuiltTetMesh::cantilever_bilayer_beam`; an inline half-space `Field<f64>` at `z = HEIGHT / 2` partitions tets into region A (`(MU_A, LAMBDA_A) = (1.0e5, 4.0e5) Pa`, lower half) and region B (`(MU_B, LAMBDA_B) = (2.0e5, 8.0e5) Pa`, upper half — `2×` stiffer per Phase 4 Decision J). A `1.0 N` tip force is distributed uniformly across every `x = LENGTH` vertex via `LoadAxis::AxisZ`; the clamped face is every `x = 0` vertex; a single backward-Euler `replay_step` at `cfg.dt = 1.0` (collapses the inertial term `M / dt²` by ~4 orders relative to stiffness — IV-3's `STATIC_DT` idiom) converges from rest to the static equilibrium configuration. The headline new capability vs row 9 is **shared-vertex multi-material coupling with no inter-layer slip** — both regions' tets share the interface vertices at `z = HEIGHT / 2` by construction, so displacement is C⁰-continuous across the interface (no slip, no penalty contact). This is the *bonded* counterpart of row 9's smooth-blended material gradient: row 9 has continuous material across a smoothstep band; row 10 has continuous *displacement* across a sharp material step. Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/bilayer_beam.ply` (960 y-slab centroids of the 7680 body tets at deformed positions + 2 per-vertex scalars; the `verify_*` correctness gates run over all 7680 tets).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The third user-facing example of Tier 3 multi-material per [`EXAMPLE_INVENTORY.md`][inv] — the canonical `cantilever_bilayer_beam + half-space MaterialField + tip-force BC` solver-driven scene, generalising row 6's uniform-material cantilever and row 9's smooth-blended sphere to a bonded bilayer cantilever beam under tip load:

```rust
let bilayer = MaterialField::from_fields(
    Box::new(HalfSpaceField {
        threshold_z: HEIGHT / 2.0,
        value_below: MU_A,                                                 // 1e5
        value_above: MU_B,                                                 // 2e5
    }),
    Box::new(HalfSpaceField {
        threshold_z: HEIGHT / 2.0,
        value_below: LAMBDA_A,                                             // 4e5
        value_above: LAMBDA_B,                                             // 8e5
    }),
);
let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(
    NX, NY, NZ, LENGTH, BREADTH, HEIGHT, &bilayer,                         // (20, 8, 8, 0.5, 0.1, 0.1)
);

let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| (p.x - LENGTH).abs() < 1e-9);
let bc = BoundaryConditions {
    pinned_vertices: pinned,
    loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
};
let theta = Tensor::from_slice(&[TIP_FORCE_TOTAL / loaded.len() as f64], &[1]);

let mut cfg = SolverConfig::skeleton();
cfg.dt = 1.0;                                                              // STATIC_DT — IV-3 idiom
cfg.max_newton_iter = 50;

let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
    CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

The bilayer split aligns at `z = HEIGHT / 2 = 0.05`; with `NZ = 8` (even), the 8 cell layers split 4 / 4 around the interface plane. `HalfSpaceField::sample`'s strict `<` boundary convention places centroids at `z = INTERFACE_Z` exactly into the upper half:

```text
centroid.z < HEIGHT / 2          ⇒ region A   (μ = MU_A, λ = LAMBDA_A;     softer)
centroid.z >= HEIGHT / 2         ⇒ region B   (μ = MU_B, λ = LAMBDA_B;     2× stiffer)
```

This is **row 10 of the sim-soft examples arc** — the third Tier 3 multi-material example, the first solver-driven multi-material scene (rows 8 + 9 are field-exposition without a solver), and the first user-facing exposure of IV-2's "shared-vertex displacement continuity" claim (`tests/multi_material_continuity.rs:181-234`) at production-scale 7680-tet mesh resolution (IV-2 covers the same tautology on a 2-tet hand-built scene). Row 10 also user-faces IV-3's bilayer-cantilever scene (`tests/bonded_bilayer_beam.rs`) at the sanity-test refinement, with the EB-composite tip-displacement comparison as the headline analytic gate.

**Lamé pairs** match Phase 4 scope memo Decision J: region A is the Ecoflex 00-30 baseline (`(μ, λ) = (1e5, 4e5)` ⇒ `ν = 0.4` compressible), region B is `2×` (Ecoflex 00-30 + 15 wt% carbon-black per Part 1 §04 §02, mechanical-only at this phase).

| Region | μ (Pa) | λ (Pa) | factor | half |
|---|---|---|---|---|
| A | `1.0e5` | `4.0e5` | `1×` baseline | lower (`z < H/2`) |
| B | `2.0e5` | `8.0e5` | `2×` Decision J | upper (`z >= H/2`) |

**Why `cfg.dt = 1.0` (STATIC_DT idiom).** Row 6's `cfg.density = 0` and IV-3's `cfg.dt = 1.0` are two ways to suppress the inertial recall term and recover the static equilibrium under prescribed BCs. Row 10 mirrors IV-3 verbatim — same scene + same time-step idiom keeps cross-reference between the example and the internal fixture clean. At `cfg.dt = 1.0` and the bilayer beam's stiffness scale, `M / dt²` is `~4` orders below the elastic stiffness contribution; a single `replay_step` from rest converges to the static equilibrium far below `cfg.tol = 1e-10`. Newton from rest under full static load takes 23 iters at this refinement (line-search backtracks at the first iters absorb the nonlinearity, then quadratic convergence kicks in) — well within the `max_newton_iter = 50` budget.

**Why a y-slab cut (vs row 8 / 9's z-slab).** Same banked rationale as rows 8 + 9 (cf-view's commit-3 instanced-sphere radius factor `bbox.diagonal() * 0.005` is oversized for dense per-tet centroid clouds in small-bbox bodies — see [`project_cf_viewer_dense_point_cloud_gap.md`][gap]) but adapted to the cantilever-beam axial geometry. The beam is thin in `y` and `z` (`b = H = 0.1`) and long in `x` (`L = 0.5`), so a `|y - b/2| < dy/2` axial mid-plane y-slab cut projects centroids onto the x-z plane — reading as a 2D bending-profile arc with the bilayer split visible as a horizontal mid-line. Z-slab (row 8 / 9 pattern) on a beam projects to an x-y rectangle with no bending visible; y-slab is the right adaptation here. The slab keeps 960 of 7680 centroids (12.5% of the body, since `dy / b = 1/8`).

[gap]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_cf_viewer_dense_point_cloud_gap.md

**Why two per-vertex scalars (vs row 9's three).** Row 10's headline is the analytic comparison + interface continuity, not the visualisation. Two scalars (`displacement_z` continuous → viridis; `material_id` categorical → tab10) cover the bending-profile readout and the bilayer split unambiguously; a third scalar (e.g., per-tet stress norm) would be ornamentation rather than load-bearing pedagogy. Same trade-off framing as row 9's "feature + mechanism + outcome" triple, here collapsed to "outcome + feature" since no novel mechanism kernel is demonstrated.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

| Anchor | Bound |
|---|---|
| `LENGTH > 0`, `BREADTH > 0`, `HEIGHT > 0` | strict `>` |
| `NX > 0`, `NY > 0`, `NZ >= 2`, `NZ % 2 == 0` | `cantilever_bilayer_beam` constructor invariants |
| `MU_A < MU_B`, `LAMBDA_A < LAMBDA_B` | bilayer ordering — Decision J `2.0×` multiplier |
| `0 < INTERFACE_Z < HEIGHT` | interface plane strictly interior to body |

The `cantilever_bilayer_beam` constructor's runtime panic invariants (`NZ` even and ≥ 2; positive geometry) re-asserted at the user-facing example layer via `const { assert!(...) }` — compile-time enforcement on the geometry constants. Plus the example-specific bilayer-ordering contract that `tip_displacement_strictly_between_uniform_bounds` depends on.

### 2. `mesh_topology_exact`

| Anchor | Bound |
|---|---|
| `mesh.n_vertices` | exact-pin `(NX+1)·(NY+1)·(NZ+1) = 1701` |
| `mesh.n_tets` | exact-pin `6·NX·NY·NZ = 7680` (CFK 6-tets-per-cell) |

Counts derived structurally from the constructor's stride formula `vid(i, j, k) = i + j·(NX+1) + k·(NX+1)·(NY+1)` and the CFK 6-tets-per-cell decomposition. III-1 determinism contract analog at the hand-built mesh layer.

### 3. `boundary_partition`

| Anchor | Bound |
|---|---|
| Pinned face vertex count | exact-pin `(NY+1)·(NZ+1) = 81` |
| Loaded face vertex count | exact-pin `(NY+1)·(NZ+1) = 81` |
| Pinned + loaded ascending order | strict `<` consecutive |
| `pinned ∩ loaded` | empty (clamped face and tip face at opposite ends) |

`pick_vertices_by_predicate` walks vertex IDs in ascending order; ascending is structural here. Disjointness is structural since `x = 0` and `x = LENGTH` are at opposite ends of the beam.

### 4. `per_tet_material_assignment`

**HEADLINE A.** For every tet, `mesh.materials()[t]` is probed at `F_probe = diag(1.2, 1, 1)` against `expected = NH(MU_A, LAMBDA_A)` if `rest_centroid.z < INTERFACE_Z` else `NH(MU_B, LAMBDA_B)`:

| Anchor | Bound |
|---|---|
| Per tet `materials()[t].energy(F_probe)` vs expected | `epsilon = 0.0` (bit-equal) |
| Per tet `materials()[t].first_piola(F_probe)` vs expected | `epsilon = 0.0` (bit-equal) per entry |
| Lower-half tet count (`rest_centroid.z < INTERFACE_Z`) | exact-pin `3840` |
| Upper-half tet count (`rest_centroid.z >= INTERFACE_Z`) | exact-pin `3840` |
| `lower + upper` | `== N_TETS_EXACT = 7680` |

The cross-implementation gate: the test-side layer assignment (`if rest_centroid.z < INTERFACE_Z`) is a bit-exact mirror of `HalfSpaceField::sample`'s strict `<` boundary convention. Both sides run identical NH arithmetic on identical `(μ, λ)` pairs and identical `F_probe` — bit-equal by construction on a fixed toolchain. Same `Matrix3` over-determination logic as rows 8 + 9: `energy(F_probe)` alone is one linear equation in `(μ, λ)`; `first_piola(F_probe)`'s `P_22 = λ ln J` directly fixes `λ`, then `P_11` fixes `μ` (over-determination).

The 3840 / 3840 split is structural at `NZ = 8`: cells `k ∈ {0, 1, 2, 3}` have all 4 vertices at `z ≤ H/2`, so every tet's centroid `z` is strictly less than `H/2` (the 4-vertex average is strictly less than the maximum since at least one vertex sits at `z = k·dz < H/2` for `k < 4`); cells `k ∈ {4, 5, 6, 7}` symmetrically place all centroids strictly above. Total partitions over the body exactly.

### 5. `solver_converges_bilayer`

| Anchor | Bound |
|---|---|
| `step.iter_count` | strict `<` `cfg.max_newton_iter = 50` |
| `step.final_residual_norm` | strict `<` `cfg.tol = 1e-10` |
| Per-tet `max\|σᵢ - 1\|` at `x_final` | strict `<` NH validity bound `1.0` |
| Global max stretch deviation at `x_final` | strict `<` `0.1` (small-strain regime sanity) |
| `mat.validity().max_stretch_deviation` | `to_bits` equality vs `1.0` |
| `mat.validity().inversion` | `RequireOrientation` |

NH's `RequireOrientation` declares `max_stretch_deviation = 1.0`; the bilayer beam under `1 N` tip load deforms by `~2.2 %` of `L` at the tip (observed), with per-tet `max|σ-1| ≈ 0.007` (well below the validity boundary; observed value surfaced in stdout). Catches a regression that would push the converged solution into NH's domain boundary.

### 6. `interface_continuity_no_slip`

**HEADLINE B.** For every interface vertex (rest `z = INTERFACE_Z`):

| Anchor | Bound |
|---|---|
| Layer-0 incident tet exists | `Some(_)` (find by `centroid.z < INTERFACE_Z` AND tet contains vertex) |
| Layer-1 incident tet exists | `Some(_)` (find by `centroid.z >= INTERFACE_Z` AND tet contains vertex) |
| `x_final` read via either layer's connectivity | bit-equal |
| Mid-beam interface vertex displacement-z (`x = L/2, y = b/2, z = H/2`) | strict `>` `1e-4 m` (non-trivial) |
| Mid-beam interface vertex rest pos | bit-equal `(L/2, b/2, H/2)` |
| Interface vertex count | exact-pin `(NX+1)·(NY+1) = 189` |

The IV-2-lens-α tautology in shared-vertex FEM: `x_final` is global and indexed by global vertex ID, so reading at vertex `v` via tet 0's connectivity (where `v` appears at some position in `tet0.vertices()`) produces bit-identical bytes to reading via tet 1's connectivity (where `v` appears at a different position). The assertion documents the contract — a contrived bug (per-tet `x_final` shadows, off-by-one indexing) would surface here. Plus non-triviality: at the mid-beam interface vertex, the displacement-z exceeds `1e-4 m` (cantilever cubic profile gives `~1/8` of tip displacement at mid-beam, so `~1.6e-3 m` expected; the threshold is one decade below).

This is the **first user-facing exposure of IV-2's shared-vertex continuity claim** at production-scale 7680-tet mesh resolution; IV-2 covers the same tautology on a 2-tet hand-built scene at `sim/L0/soft/tests/multi_material_continuity.rs:181-234`.

### 7. `tip_displacement_matches_eb_composite_within_30pct`

**HEADLINE C — inventory's named gate.** Saint-Venant-averaged tip-z displacement vs the Euler-Bernoulli composite-beam analytic:

| Anchor | Bound |
|---|---|
| `tip_disp_bilayer > 0` | strict `>` (upward bend under +ẑ load) |
| `\|tip_disp_bilayer - eb_composite\| / eb_composite` | strict `<` `TIP_DISP_REL_TOL = 0.30` |

EB-composite analytic: `δ_eb = F · L³ / (3 · EI_eff)` where `EI_eff = E_A · I_transformed` and `I_transformed = b · H³ · (1 + 14 n + n²) / (96 (1 + n))` (transformed-section, equal-thickness layers, modular ratio `n = E_B / E_A`). At our parameters (`n = 2`, `E_A = 2.8e5 Pa`, `(L, b, H) = (0.5, 0.1, 0.1)`, `F = 1 N`): `EI_eff ≈ 3.208 N·m²`, `δ_eb ≈ 1.299e-2 m`.

Observed `tip_disp_bilayer ≈ 1.086e-2 m` ⇒ rel-err `≈ 0.164` — well under the 30% gate, mirroring IV-3's empirical Tet4 + bilayer convergence behaviour. **See the "Tet4 caveat" section below** for the recalibration story (inventory's "to 3 digits" was off-spec; operative gate is "to 30%" mirroring IV-3 sanity).

### 8. `tip_displacement_strictly_between_uniform_bounds`

The discriminating physical assertion (IV-2 lens β analog on tip displacement). Three solver runs: bilayer + uniform-A baseline (every tet `MU_A, LAMBDA_A`) + uniform-B baseline (every tet `MU_B, LAMBDA_B`):

| Anchor | Bound |
|---|---|
| `tip_disp_uniform_a > 0` | strict `>` (upward bend) |
| `tip_disp_uniform_b > 0` | strict `>` (upward bend) |
| `tip_disp_uniform_b < tip_disp_uniform_a` | strict `<` (region B `2×` stiffer ⇒ less compliance) |
| `tip_disp_uniform_b < tip_disp_bilayer` | strict `<` (one half is region A — softer than uniform-B) |
| `tip_disp_bilayer < tip_disp_uniform_a` | strict `<` (one half is region B — stiffer than uniform-A) |

Catches dropped-tet-contribution / swapped-materials / mis-assigned-material bugs that would push `d_bilayer` to one of the bounds (or outside).

### 9. `captured_bits_tip_displacements`

Three tip displacements (bilayer + uniform-A + uniform-B) captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `tip_disp_bilayer` vs captured ref | rel `1e-12`, abs `1e-12` |
| `tip_disp_uniform_a` vs captured ref | rel `1e-12`, abs `1e-12` |
| `tip_disp_uniform_b` vs captured ref | rel `1e-12`, abs `1e-12` |

7680 tets through faer's sparse Cholesky lives between IV-1's dense bit-equal tier (12-24 DOFs) and IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift on faer's per-column FMA-fusion path). 7680 tets is past IV-1's sparse-at-scale tier; the contract is **relative tolerance, not strict bit-equality** — observed bits captured for regression detection, compared via `assert_relative_eq!` at `1e-12` rel. Capture provenance + failure-mode protocol IV-1-protocol-verbatim above the const blocks (do NOT re-bake without diagnosing toolchain vs real regression first).

### 10. `yslab_visual_populations_exact`

| Anchor | Bound |
|---|---|
| Y-slab lower-layer bucket count | strict `>` `0` AND exact-pin `480` |
| Y-slab upper-layer bucket count | strict `>` `0` AND exact-pin `480` |

Visual-pedagogy guard: each layer must have ≥ 1 y-slab centroid for the bilayer split to read in cf-view's projected x-z curve; per-layer counts also exact-pinned per the III-1 determinism contract. Total `960` (= `N_TETS · dy / b = 7680 · (0.0125 / 0.1) = 960` with the slab symmetric about `y = b/2`).

## Visuals

`out/bilayer_beam.ply` — the canonical visual artifact.

```text
960 vertices (one per tet centroid in the |y - b/2| < dy/2 axial mid-plane y-slab cut,
              vertex positions amplified `rest + DISPLACEMENT_SCALE × (deformed - rest)`
              with DISPLACEMENT_SCALE = 20.0 — visualisation-only amplification)
0 faces (point cloud)
extras["displacement_z"]: f32  // continuous m       (TRUE physical cantilever-bending profile)
extras["material_id"]:    f32  // categorical 0.0 / 1.0 (lower-half = A, upper-half = B)
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/bonded-bilayer-beam/out/bilayer_beam.ply
```

cf-view auto-discovers the two per-vertex scalars; alphabetical-first selects `displacement_z` on launch — cf-view's Q5 colormap heuristic detects continuous + all-positive → viridis sequential palette, rendering the cantilever-bending profile as a smooth color gradient from `0 m` (clamped face) to `~1.3e-2 m` (tip). The amplified y-slab projects to a 2D x-z bending arc in cf-view that traces the beam's deformed shape unmistakably from default orbit angles.

User dropdowns to `material_id`: cf-view detects integer-valued + 2 unique values < 16 → tab10 categorical palette, rendering the bilayer split as a horizontal mid-line bisecting the bending arc — lower half one color (region A), upper half another (region B). Same rendered geometry, different scalar.

### Why `DISPLACEMENT_SCALE = 20.0` (visualisation-only amplification)

Standard FEM-visualisation trick. At small-strain regime the geometric bending arc is honest-but-subtle: the observed bilayer tip displacement is `~1.1 cm` on a `50 cm` beam (`~2.2 %` of `L`; the EB-composite analytic predicts `~1.3 cm` ≈ `~2.6 %` and the Tet4 + bilayer rel-err at this refinement is `~16 %` per the Tet4-caveat section below), well within small-strain Neo-Hookean validity but visually understated at default cf-view orbit even with modest amplification. A `20×` amplifier on the geometric vertex positions puts the visible tip displacement at `~22 cm` (`~43 %` of `L`, `~2.2×` beam thickness `H = 0.1 m`), arcing dramatically in the y-slab projection without distorting the beam-shape readability — the bend is exaggerated but the rectangle still reads as a beam.

**The amplification is geometry-only.** The `displacement_z` per-vertex scalar carries the TRUE physical displacement (no scale factor); cf-view's color gradient still reflects the true bending magnitude `[0, ~1.3e-2 m]`. Every `verify_*` correctness gate operates on the unscaled solver outputs (`step.x_final`, `tip_disp_*`, the per-tet records' `f_at_x_final`) — the convergence anchor at 16% rel-err vs the EB-composite analytic, the captured-bits anchor under IV-1 sparse-tier rel-tol, and the discriminating uniform-bounds anchor are all unaffected. The amplification is a presentation choice in the same family as IV-3's `cfg.dt = 1.0` quasi-static idiom: an authoring decision that reshapes the readout without affecting the underlying physics.

If you want the un-amplified visualisation (TRUE geometric deformation), set `DISPLACEMENT_SCALE = 1.0` in `src/main.rs` and re-run; the y-slab cloud will render at the physical `~1.3 cm` tip bend, visible if you orbit cf-view's camera to a side view (camera in `+ŷ` looking at the X-Z plane edge-on) but subtle from default orbit angles.

Per `feedback_visual_review_is_the_test` — for cf-view consumers, the visual pass is real (not collapsed to JSON read).

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, full-body per-layer split + tip-displacement triple + EB-composite analytic + y-slab partition).

## Run

```text
cargo run -p example-sim-soft-bonded-bilayer-beam --release
```

Output: `out/bilayer_beam.ply` and stdout summary.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The exact-pin counts and captured tip-displacement bits were captured under release-mode build; matching that invocation shape removes one variable from the determinism contract.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Tet4 caveat — convergence gate recalibration

`EXAMPLE_INVENTORY.md` Tier 3 row 10's "assert tip-displacement matches bilayer-beam analytic to **3 digits**" wording predates IV-3's empirical convergence finding. The operative gate here is `< 0.30` rel-err (mirroring IV-3 sanity), not `< 1e-3`.

**Why.** IV-3 (`sim/L0/soft/tests/bonded_bilayer_beam.rs`) documents that **Tet4 + bilayer converges at `~O(h^1.5)` to the EB-composite analytic** — sub-`O(h²)` due to a bilayer-interface discretisation gap that Phase H Tet10 + interface-aware refinement closes. At three refinement levels:

| Refinement | Tet count | `err / analytic_eb` (typical) |
|---|---|---|
| `h     = (10,  4,  4)` |   960 | `~30-40 %` |
| `h/2   = (20,  8,  8)` |  7680 | `~16 %` (this row) |
| `h/4   = (40, 16, 16)` | 61440 | `< 15 %` (IV-3 absolute floor) |

`< 1e-3` (3 digits) is empirically unreachable at Tet4 at any practical refinement: the bilayer-interface discretisation error doesn't asymptote to zero with Tet4. To hit `< 1e-3`, the engine needs Phase H's quadratic Tet10 elements (which represent C¹-linear deformation across the interface) plus interface-aware refinement (which double-resolves the elements straddling `z = H/2`). Both are multi-month FEM efforts parked under Fork B per the `project_layered_silicone_device` memo.

**The operative gate** mirrors IV-3's `iv_3_uniform_passthrough_at_h2_matches_eb_within_30pct` sanity test (`tests/bonded_bilayer_beam.rs:489-527`): at refinement `(20, 8, 8)`, the bilayer tip displacement matches the EB-composite analytic to within 30%. This is a meaningful regression-detection gate — a passing 30% band confirms the constructor + BC + load + the multi-material FEM assembly path are wired correctly; a failure signals either (a) a regression in the multi-material FEM assembly numerics or (b) a refinement / load / material-assignment misconfiguration. The discriminating gate (anchor 8) is the strict-between-uniform-bounds inequality, NOT the absolute analytic comparison.

For tighter convergence study at `h/4` `(40, 16, 16)` 61440 tets see `iv_3_bilayer_converges_to_eb_composite_beam` at `tests/bonded_bilayer_beam.rs:529-662` — heavier mesh, `< 0.15` rel-err absolute floor, three-refinement convergence-rate assertion. That's the validation gate; this row is the user-facing wrap at the example-friendly refinement.

## Cross-references

- **Sister sim-soft examples**: `sdf/stress-test`'s `sphere_eval` (row 1 — `Sdf` trait contract on the `SphereSdf` primitive), `multi-element-stretch` (row 6 — uniform-material Dirichlet-stretch counterpart at the same compressible NH baseline; row 10 generalises from prescribed-stretch BC to tip-force BC, and from uniform to bilayer multi-material), `layered-scalar-field` (row 8 — sharp-step sphere counterpart without solver; row 10 ships the solver-driven counterpart with the same Decision J `2.0×` stiffness ratio), `blended-scalar-field` (row 9 — smooth-blended sphere counterpart without solver; row 9 has continuous material across a smoothstep band, row 10 has continuous *displacement* across a sharp material step), forthcoming `concentric-lame-shells` (row 11 — IV-5 hollow form).
- **Internal-fixture template**: `sim/L0/soft/tests/bonded_bilayer_beam.rs` (IV-3 — three-refinement convergence study at `h, h/2, h/4` against EB-composite + Timoshenko closed forms; row 10 ships the user-facing wrap at the `h/2` sanity-test refinement). Row 10 mirrors IV-3's scene + load + BC + STATIC_DT + max_newton_iter exactly.
- **Sister fixture for HEADLINE B**: `sim/L0/soft/tests/multi_material_continuity.rs` (IV-2 — shared-vertex displacement continuity on a 2-tet `two_tet_shared_face` hand-built scene; row 10 extends IV-2's lens-α tautology to production-scale 7680-tet bilayer-beam mesh).
- **`HandBuiltTetMesh::cantilever_bilayer_beam`**: `sim/L0/soft/src/mesh/hand_built.rs:201-279` — `(nx, ny, nz, length, breadth, height, &MaterialField)`. CFK 6-tets-per-cell decomposition; bilayer interface aligns at cell boundary `k = NZ / 2` (constructor enforces `NZ` even and ≥ 2). Per-tet `MaterialField` sampling at rest-config centroids populates the `materials()` cache.
- **`HalfSpaceField`**: inline test-side `Field<f64>` mirroring IV-3's at `tests/bonded_bilayer_beam.rs:280-294`. Production-side promotion to `sim-soft::field` is deferred to a later consumer (Phase H interface-aware refinement) per the docstring at the IV-3 site; row 10 is the second consumer and the second deferral.
- **Closed-form derivation**: see IV-3 module docstring "Closed-form Timoshenko prediction" + "`I_transformed` derivation" sections at `tests/bonded_bilayer_beam.rs:81-120` for the parallel-axis + transformed-width step-through. Row 10 reuses `young_modulus`, `i_transformed_bilayer`, and `eb_composite_tip_displacement` verbatim.
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` Q5 (categorical-colormap heuristic — integer-valued + < 16 unique values → tab10; otherwise sequential viridis).
- **Book reference**: Part 3 Ch 03 [`30-discretization/03-interfaces/00-bonded.md`](../../../docs/studies/soft_body_architecture/src/30-discretization/03-interfaces/00-bonded.md) — "Bonded interfaces" sub-leaf documenting the C⁰-continuous-displacement-by-construction claim that row 10's HEADLINE B exposes user-facing. The chapter's **regression-test requirement** for bonded-interface analytic-baseline comparison is what HEADLINE C ships at example layer (the `tip_displacement_matches_eb_composite_within_30pct` gate IS the bonded-bilayer beam-bending analytic regression the chapter prescribes).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md).
