# solid-to-sim-soft

**PR3 row 16 — `cf_design::Solid` is a first-class SDF for `SdfMeshedTetMesh::from_sdf` via the F1+F3 bridge.** A typed boolean-difference body composed via cf-design's CSG kernel (`Solid::sphere(R_OUTER).subtract(Solid::sphere(R_CAVITY))`) flows into sim-soft's BCC + Labelle-Shewchuk Isosurface Stuffing pipeline through a single trait-object coercion. No per-shape glue: F1 ships `impl Sdf for Solid` and F3 re-exports `cf_design::Sdf` as `sim_soft::Sdf`, so `&solid: &dyn cf_design::Sdf` is exactly what `SdfMeshedTetMesh::from_sdf(sdf: &dyn Sdf, ...)` already accepts. Companion to [`mesh-scan-as-solid`](../mesh-scan-as-solid) (row 15, scan-derived `mesh_sdf::SignedDistanceField` consumed via the same trait): together the two rows close the bridge story PR3 was set up for — typed CSG bodies AND scanned bodies are both first-class `cf_design::Sdf` primitives in sim-soft.

Geometry is identical to PR1 row 11 [`concentric-lame-shells`](../concentric-lame-shells) (`R_OUTER = 0.10 m`, `R_CAVITY = 0.04 m`, `cell_size = 0.02 m`, `pressure = 5e3 Pa`) — single-material variant (`MaterialField::skeleton_default = uniform(1e5, 4e5) Pa`, bit-equal to row 11's `MU_INNER` / `LAMBDA_INNER` uniform-1× baseline). Cross-row continuity: the captured cavity-wall mean bits below match row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS` exactly.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/shell_boundary.ply` (deformed-boundary triangle mesh of the body's inner + outer surfaces at amplified deformed positions + 1 per-vertex `radial_displacement` scalar; `verify_*` correctness gates run over the unscaled solver outputs).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The bridge — typed `cf_design::Solid` operands flow into sim-soft via a single trait-object coercion:

```rust
let solid = Solid::sphere(LAYERED_SPHERE_R_OUTER)
    .subtract(Solid::sphere(LAYERED_SPHERE_R_CAVITY));     // typed cf-design CSG body

let hints = MeshingHints {
    bbox: Aabb3::new(/* ±0.12 m */),
    cell_size: 0.02,
    material_field: None,                                  // skeleton_default = uniform(1e5, 4e5)
};

// `&solid: &dyn cf_design::Sdf` via DST coercion — F1 impl Sdf for Solid
// AND F3 sim_soft::Sdf re-export. No per-shape wrapper.
let mesh = SdfMeshedTetMesh::from_sdf(&solid, &hints)?;
```

The HEADLINE A semantic-equivalence anchor proves the bridge is bit-preserving by meshing the same hollow shell via two surfaces — typed `Solid` AND sim-soft's PR1-era `DifferenceSdf<SphereSdf>` — and asserting `equals_structurally` plus position-by-position `EXACT_TOL = 0.0`:

```rust
let mesh_typed    = mesh_via(&solid,        &hints);
let mesh_baseline = mesh_via(&diff_sdf,     &hints);     // DifferenceSdf::new(SphereSdf, SphereSdf)
assert!(mesh_typed.equals_structurally(&mesh_baseline));
// ...positions match bit-exact at every vertex.
```

The arithmetic is identical: cf-design's `Sphere` evaluates `p.coords.norm() - radius`, `Subtract` is `a.max(-b)` — bit-equal to sim-soft's `SphereSdf::eval` and `DifferenceSdf::eval`, so the BCC + warp + stuffing pipeline samples bit-identical SDF values and produces bit-identical meshes.

Beyond the bridge, the row exercises the **single-material** variant of row 11's three-shell pressurization scene as the minimal end-to-end demo per [`EXAMPLE_INVENTORY.md`][inv] row 16's "minimal pressurization scene" line. Single-material Lamé closed-form derivation under internal pressure `p` on `R_a = R_CAVITY` and fixed outer surface `u_r(R_b) = 0` at `R_b = R_OUTER`:

```text
u_r(r)   = A·r + B/r²
σ_rr(r)  = K·A − 4μ·B/r³,   K = 3λ + 2μ

BCs:
u_r(R_b) = 0           ⇒  B = -A·R_b³
σ_rr(R_a) = -p         ⇒  A·[K + 4μ·R_b³/R_a³] = -p
                       ⇒  A = -p / [K + 4μ·R_b³/R_a³]
                          B = -A·R_b³
                          u_r(R_a) = A·R_a + B/R_a²
```

At the row's constants `(p, μ, λ, R_a, R_b) = (5e3, 1e5, 4e5, 0.04, 0.10)`: `K = 1.4e6`, `(R_b/R_a)³ = 15.625`, `4μ·R_b³/R_a³ = 6.25e6`, denominator `= 7.65e6`, `A ≈ -6.536e-4`, `B ≈ 6.536e-7`, `u_r(R_a) ≈ 3.823e-4 m` — about `0.96 %` of `R_CAVITY`, well inside the small-strain band where Lamé is the leading-order approximation. FEM observed at h/2 is `≈ 3.245e-4 m` (`~15 %` rel-err vs analytic; IV-5 documents super-quadratic empirical convergence at the fine end).

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 6 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `bridge_equivalence` — HEADLINE A

| Anchor | Bound |
|---|---|
| typed-Solid mesh `equals_structurally` `DifferenceSdf<SphereSdf>` baseline | `true` |
| position vector lengths match | `n_vertices_typed == n_vertices_baseline` |
| every vertex position matches `(x, y, z)` | `EXACT_TOL = 0.0` (bit-equal) |

Bit-identical SDF arithmetic between the two paths produces bit-identical meshes. Any drift surfaces as a regression in F1's `impl Sdf for Solid` or in cf-design's `Sphere` / `Subtract` evaluation diverging from sim-soft's `SphereSdf::eval` / `DifferenceSdf::eval`.

### 2. `counts_exact` — III-1 determinism + cross-row continuity vs row 11

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `6456` | bit-equal to row 11's `N_TETS_EXACT` |
| `n_vertices`          | `4682` | bit-equal to row 11's `N_VERTICES_EXACT` |
| `n_referenced`        | `1480` | bit-equal to row 11's `N_REFERENCED_EXACT` |
| `n_pinned` (outer)    | `734`  | bit-equal to row 11's `N_PINNED_EXACT` |
| `n_loaded` (cavity)   | `134`  | bit-equal to row 11's `N_LOADED_EXACT` |

Mesh topology is a function of geometry + `cell_size` only, not of material partitioning — so single-material row 16 produces the same counts as row 11's three-shell scene at identical radii + spacing.

### 3. `quality_floors` — Theorem-1 sanity

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |
| `aspect_ratio ≥ 0.05` per tet | Theorem 1 sanity floor |
| `dihedral ∈ [5°, 175°]` per tet | Theorem 1 sanity floor |

Same Theorem 1 envelope row 3 [`sdf-to-tet-sphere`](../sdf-to-tet-sphere) anchors verbatim; the BCC + stuffing pipeline is material-blind for tet quality.

### 4. `solver_converges`

| Anchor | Bound |
|---|---|
| `step.iter_count` | `< cfg.max_newton_iter = 50` (observed: `3`) |
| `step.final_residual_norm` | `< cfg.tol = 1e-10` (observed: `~3.4e-12`) |

A single backward-Euler `replay_step` at `STATIC_DT = 1.0 s` (collapses inertial term `M / dt²` ~4 orders below stiffness — IV-5's static-regime idiom) converges from rest in 3 Newton iters.

### 5. `cavity_wall_lame` — HEADLINE B

Saint-Venant-averaged cavity-wall mean radial displacement (mean over `bc.loaded_vertices` of `|x_final[v]| - |rest[v]|`) within the row 11 cavity-gate signature:

| Anchor | Bound |
|---|---|
| `assert_relative_eq!(observed, analytic, max_relative = 0.30, epsilon = 5e-6)` | rel-err `~15 %` |

Observed `≈ 3.245e-4 m`, analytic `≈ 3.823e-4 m` — well under the `0.30` gate.

### 6. `captured_cavity_wall_mean_bits` — IV-1 sparse-tier rel-tol

Cross-row continuity guard: observed cavity-wall mean within `1e-12` rel-tol of row 11's captured `CAVITY_WALL_UNIFORM_1X_REF_BITS`:

| Anchor | Bound |
|---|---|
| `assert_relative_eq!(observed, f64::from_bits(0x3f35_43f1_0b56_98c8), max_relative = 1e-12, epsilon = 1e-12)` | bit-equal by construction |

Same geometry + same Lamé pair + same solver path through faer's sparse Cholesky ⇒ bit-equal observation. IV-1 sparse-tier rel-tol admits any future cross-platform 3-ULP drift on faer's per-column FMA-fusion path. **Failure-mode protocol per IV-1**: if the rel-tol comparison fails, do NOT re-bake. Rule out toolchain drift first; if same toolchain, real regression in the typed-`Solid` → sim-soft bridge OR in the shared FEM hot path that row 11 would also catch.

## Visuals

`out/shell_boundary.ply` — deformed-boundary triangle mesh of the body's inner + outer surfaces with `DISPLACEMENT_SCALE = 50.0` geometric amplification on vertex positions (`vertex = rest + SCALE * (deformed - rest)`) — visualisation-only; the `radial_displacement` per-vertex scalar carries the TRUE physical magnitude. Same precedent as row 11's `DISPLACEMENT_SCALE = 50.0` (small-strain spherical-symmetry regime; `50×` puts cavity-wall inflation at `~36 %` of `R_CAVITY` — dramatically visible without distorting spherical-symmetry readability).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/solid-to-sim-soft/out/shell_boundary.ply
```

cf-view default-picks `radial_displacement` (only scalar present); sequential viridis on `[0, ~5e-4]` shows the radial-decay profile from cavity wall (`~3.8e-4 m` peak inflation) outward to outer wall (`0` by Dirichlet pin). The amplified inflation reads as a visibly-bulged inner surface relative to the rest-config outer boundary.

## Run

```sh
cargo run -p example-sim-soft-solid-to-sim-soft --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve at this mesh resolution (~6.5k tets through faer's sparse Cholesky); debug mode would take many minutes for what runs in seconds release.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md
