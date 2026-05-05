# single-tet-stretch

**`SkeletonSolver` end-to-end on `SoftScene::one_tet_cube` — the first FEM-running example in the sim-soft examples arc. Backward-Euler Newton step on the canonical decimeter-edge tet (spec §2): `(μ, λ) = (1e5, 4e5)` Pa, `ρ = 1030` kg/m³, `Δt = 1e-2` s, `θ = 10` N along `+ẑ` on `v_3` with `v_0..v_2` Dirichlet-pinned.** Newton converges in `iter_count = 3` to `dz ≈ 9.69e-4` m on `v_3` (~1% engineering strain), with every entry of `x_final` matching the IV-1 frozen reference bit pattern at sim-soft `c3729d4a`. Output is `out/force_stretch.json` (no spatial artifact, no `cf-view` rendering — single-tet has trivial topology and the force-stretch readout is the inspection signal per inventory Q4 row 4).

## What this example demonstrates

The walking-skeleton solver step API:

```rust
let cfg = SolverConfig::skeleton();
let (mesh, bc, initial) = SoftScene::one_tet_cube();
let mut solver: SkeletonSolver = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
let mut tape = Tape::new();
let theta_var = tape.param_tensor(Tensor::from_slice(&[10.0], &[1]));
let step = solver.step(&mut tape, &initial.x_prev, &initial.v_prev, theta_var, cfg.dt);
```

`SkeletonSolver = CpuTet4NHSolver<SingleTetMesh>` is `lib.rs:99`'s walking-skeleton specialization of `CpuNewtonSolver<Tet4, SingleTetMesh, NullContact, 4, 1>` — backward-Euler Newton with `NeoHookean` constitutive law (Phase 4 Decision G) on a hand-built 1-tet mesh, no contact. `SoftScene::one_tet_cube` returns the canonical scene per spec §2: vertices at `v_0=(0,0,0)`, `v_1=(0.1,0,0)`, `v_2=(0,0.1,0)`, `v_3=(0,0,0.1)` m with `MaterialField::uniform(1e5, 4e5)`; default `BoundaryConditions` pin `v_0..v_2` and load `v_3` along `LoadAxis::AxisZ`. The Stage-1 θ convention dispatches a length-1 scalar tensor as a `+ẑ` traction magnitude on every loaded vertex's z-DOF (`backward_euler.rs:807-817`).

This is the **first FEM-running** example in the arc — rows 1+2+3 (`sphere-sdf-eval`, `hollow-shell-sdf`, `sdf-to-tet-sphere`) exercised the SDF eval / composition / meshing surface only. Row 4 is `Solver::step` end-to-end: assemble the elastic + inertial + external-force tangent, factor, line-search, converge, return `NewtonStep { x_final, iter_count, final_residual_norm }`.

**Why these scene parameters**: spec §2's walking-skeleton defaults — soft-robotics dimensional regime (`L = 0.1` m, silicone-class density), Ecoflex-class Lamé pair (`λ = 4μ ⇒ ν = 0.4` compressible Neo-Hookean per Phase H deferral), and `θ = 10` N chosen to land the response in the linear / mildly-nonlinear regime (`δz ≈ 1%` strain). `solver_convergence::stage_1_traction_converges` is the original Phase 1 invariant test on this exact configuration; this example wraps it as a user-facing demo.

**Why JSON-only (no cf-view)**: per inventory Q4 row 4, single-tet examples emit force-stretch JSON traces with no spatial artifact — the boundary surface is a regular tetrahedron, well-described by the four corner positions in the JSON record, and a render of a single tet would be visual noise. The constitutive `θ`-sweep curve belongs to row 5 (`neo-hookean-uniaxial`); this row's JSON is the inspection artifact for the walking-skeleton step API itself.

## Numerical anchors

Each anchor is encoded as `assert!` / `assert_eq!` in `src/main.rs` under `verify_*`. Per [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md), a clean `cargo run --release` exit-0 IS the correctness signal.

### Convergence

| Anchor | Bound |
|---|---|
| `iter_count` | exact-pin: 3 |
| `iter_count < cfg.max_newton_iter (= 10)` | strict (Newton-budget contract) |
| `final_residual_norm < cfg.tol (= 1e-10)` | strict (R-1 convergence trigger) |

`iter_count = 3` is exact-pinned per the row-3 banked pattern (every deterministic count gets an exact pin). The `< budget` and `< tol` bounds are the Newton-convergence contract from `solver_convergence.rs`; both are entailed by the exact-pin when held alongside [`v_3` displacement](#loaded-vertex-displacement) and [bit-equal](#bit-equal-pre-phase-4-reference) below, but kept as explicit asserts for diagnostic clarity (the failure messages distinguish "Newton diverged" from "convergence rate moved silently").

### State shape

| Anchor | Bound |
|---|---|
| `step.x_final.len()` | exact-pin: 12 (4 vertices × 3 DOFs, vertex-major + xyz-inner) |

### Pinned-DOF Dirichlet

| Anchor | Bound |
|---|---|
| Per-DOF deviation from rest, `\|x_final[i] − x_prev[i]\|` for `i ∈ {0..9}` | `< 1e-14` m |

`v_0..v_2`'s 9 DOFs are full Dirichlet-pinned. The `1e-14` m bound admits f64 round-to-nearest noise on rest-position storage while catching any real Dirichlet-pin regression (mirrors `solver_convergence.rs:113`).

### Loaded-vertex displacement

| Anchor | Bound | Observed |
|---|---|---|
| `dz = x_final[11] − x_prev[11]` | `> 0` strict | `9.692383522509201e-4` m |
| `\|dx\| = \|x_final[9] − x_prev[9]\|` | `< 1e-5` m | `0` (exact, follows from bit-equal) |
| `\|dy\| = \|x_final[10] − x_prev[10]\|` | `< 1e-5` m | `0` (exact, follows from bit-equal) |
| `dz` | `∈ [5e-4, 1.5e-3]` m | within band by ~30% margin each side |

The `dz > 0` sign assert anchors the load-axis dispatch contract (force is `+ẑ`, displacement must be `+ẑ`). The `|dx|, |dy| < 1e-5` bound is the F=I isotropic-tangent claim — at the rest configuration the NH constitutive tangent is diagonal in the canonical axis-aligned tet, so the Newton step is pure `+ẑ` at the linear level; NH geometric nonlinearity at ~1% strain induces at most `O(strain² · dz) ≈ 1e-7` m off-axis, two orders below the bound. The bit-equal pin below is even tighter (`dx = dy = 0` exact under IEEE-754).

The dimensional band `[5e-4, 1.5e-3]` m is centred on the linear estimate `δz ≈ θ / A_33,zz ≈ 9.6e-4` m from the condensed stiffness `A_33 ≈ diag(2.1e3, 2.1e3, 1.04e4)` N/m at rest (see `solver_convergence.rs:14-26` for the dimensional analysis), with ~30% NH-nonlinearity margin on each side. Catches a constitutive- or assembly-scale regression that would shift the response by multiple band-widths.

### Bit-equal pre-Phase-4 reference

| Anchor | Bound |
|---|---|
| Per-DOF `x_final[i].to_bits()` for `i ∈ {0..12}` | exact-pin to IV-1 reference bits captured at sim-soft `c3729d4a` |

The strongest regression net available on this path — every entry of `x_final` matches the IV-1 frozen reference bit pattern at `invariant_iv_1_uniform_passthrough.rs:138-151`. IV-1's two-tier contract states that dense small-FEM assembly (the 12-DOF path here) is bit-equal across rustc/LLVM minor versions AND across `(macOS arm64, Linux x86_64)` SIMD architectures, because the `nalgebra::Matrix3` arithmetic compiles to scalar-equivalent IEEE-754 ops on every supported target.

**Failure-mode protocol** (mirrors IV-1's): if this assert fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture); (2) if same toolchain, real regression — identify which sim-soft commit altered the canonical-scene numerics; (3) NEVER re-bake the reference values to make the test green. Spurious re-capture hollows the contract to a tautology.

## Visuals

`out/force_stretch.json` (single-record schema) is the inspection artifact — no cf-view rendering this row.

```text
{
  "scene": {
    "mesh_corners_m":      [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
    "boundary_conditions": { "pinned_vertices": [0, 1, 2],
                             "loaded_vertices": [{"vertex": 3, "axis": "AxisZ"}] },
    "material_uniform":    { "mu_Pa": 1e5, "lambda_Pa": 4e5 },
    "density_kg_m3":       1030,
    "dt_s":                0.01,
    "tol":                 1e-10,
    "max_newton_iter":     10
  },
  "input":           { "theta_N": 10.0 },
  "step":            { "iter_count": 3,
                       "final_residual_norm": 9.59e-14,
                       "x_prev":  [...12 entries],
                       "x_final": [...12 entries] },
  "displacement_v3": { "dx_m": 0.0,
                       "dy_m": 0.0,
                       "dz_m": 9.69e-4,
                       "analytic_linear_estimate_m": 9.6e-4,
                       "dimensional_band_m": [5e-4, 1.5e-3] }
}
```

**Read directly** — open `out/force_stretch.json` in any text editor, or pipe through `jq` for slicing (e.g. `jq '.displacement_v3' out/force_stretch.json` for just the `v_3` readout). The full DOF state is captured in `step.x_prev` and `step.x_final` for spot-checking against the IV-1 bit pattern. Stdout's museum-plaque summary covers the same numbers in human-readable form.

The JSON is a **single-record force-stretch trace**, not a θ-sweep curve. Multi-point constitutive curves (the canonical force-vs-stretch shape across loading magnitudes) belong to row 5 `neo-hookean-uniaxial`.

## Run

```text
cargo run -p example-sim-soft-single-tet-stretch --release
```

Output: `out/force_stretch.json` (single-record schema above). Stdout prints input fixture summary, all 8 anchor-group names, NewtonStep results (iter_count, residual norm), and `v_3` displacement readout (dx, dy, dz, linear estimate, dimensional band).

## Cross-references

- **Sister Tier 1 examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait contract on `SphereSdf`), `hollow-shell-sdf` (row 2 — `DifferenceSdf` composition), `sdf-to-tet-sphere` (row 3 — BCC + Isosurface Stuffing on a solid sphere). See `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1.
- **`SkeletonSolver` + `CpuTet4NHSolver` definitions**: `sim/L0/soft/src/lib.rs:89-99` (type aliases); `sim/L0/soft/src/solver/backward_euler.rs` (`CpuNewtonSolver` impl, `SolverConfig::skeleton`).
- **`SoftScene::one_tet_cube`**: `sim/L0/soft/src/readout/scene.rs:72-89` — canonical 1-tet scene per spec §2.
- **`SingleTetMesh::new`**: `sim/L0/soft/src/mesh/single_tet.rs:32-50` — vertex placement + per-tet material sampling.
- **Invariant tests this example wraps**: `sim/L0/soft/tests/solver_convergence.rs::stage_1_traction_converges` (Phase 1 R-1 convergence + dimensional band) and `sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs::iv_1_one_tet_skeleton_x_final` (frozen `c3729d4a` bit-equal contract — the source of the 12-entry reference table at `src/main.rs::ONE_TET_X_FINAL_BITS`).
- **`LoadAxis::AxisZ` θ dispatch**: `sim/L0/soft/src/solver/backward_euler.rs:807-817` — single-scalar broadcast convention.
- **Book reference**: Part 11 Ch 01 01-composition §Solver (the `Solver` trait shape); Part 6 Ch 04 (`replay_step` deterministic-replay contract — used by IV-1's test 3 and the Phase 1 forward-map gradcheck path, not exercised here).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md).
