# layered-silicone-device

**PR3 row 20 — the Tier 6 synthesis row, closing the entire sim-soft examples arc.** A layered silicone device with a scan-derived cavity is composed end-to-end through every PR3 foundation piece: scan SDF (F2 `impl cf_design::Sdf for mesh_sdf::SignedDistanceField`) → typed-Solid heterogeneous CSG (F5 `Solid::from_sdf` sugar over `user_fn`) → BCC + Isosurface Stuffing tet pipeline (F1 `impl Sdf for Solid` + F3 `sim_soft::Sdf` re-export) → 3-shell `MaterialField` from the silicone constants table (F4 `silicone_table.rs`) → static fit pose with the same scan SDF as the rigid indenter (post-PR2 trait unification: any `impl Sdf` is a valid rigid primitive directly). The relative-comparison "fit-tightness" force readout at zero indenter displacement IS the device memo's "feels right" subjective tightness target made quantitative — the cavity walls already kiss the indenter at rest because the cavity IS scan-shaped.

The device geometry is a 3-layer hollow body: outer + inner shells of `ECOFLEX_00_30` (μ = 23 kPa); middle shell of `DRAGON_SKIN_10A` (μ = 51 kPa, the closest pure-silicone proxy in F4 for the conductive-composite middle layer that real builds carry copper mesh + carbon black through). Each shell is 25 mm thick (`R_OUTER = 0.10 m`, `R_MIDDLE_OUTER = 0.075 m`, `R_INNER_OUTER = 0.05 m`); the cavity is a scan-derived asymmetric region carved from the inner shell.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Outputs are `out/layered_silicone_device.json` (fit-pose scalars + 3-material provenance + per-active-contact-pair detail) and `out/device_zslab.ply` (z-slab per-tet centroid cloud with categorical `material_id` + sequential `displacement_magnitude`).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The full PR3 bridge stack composed in one expression — a scan-derived cavity carved from a parametric outer body via heterogeneous CSG in cf-design's typed kernel:

```rust
let scan_sdf = build_scan_fixture();                             // mesh_sdf::SignedDistanceField, F2
let scan_solid = Solid::from_sdf(scan_sdf.clone(), scan_bounds); // typed Solid leaf, F5
let body = Solid::sphere(R_OUTER).subtract(scan_solid);          // typed CSG, cf-design kernel

let hints = MeshingHints {
    bbox: Aabb3::new(/* ±0.12 m */),
    cell_size: 0.02,
    material_field: Some(build_material_field()),                // 3-shell partition, F4 + LayeredScalarField
};

let mesh = SdfMeshedTetMesh::from_sdf(&body, &hints)?;           // F1 + F3, &dyn Sdf trait dispatch
```

The scan-derived cavity carve composes with the parametric outer sphere through `Solid::subtract` — the same boolean operator as parametric-only CSG, no per-shape glue. This is the architectural payload of PR3 — the heterogeneous-CSG bridge between parametric typed bodies (cf-design `Solid` primitives) and scan-derived SDFs (mesh-sdf-backed) lives in cf-design's typed kernel as F5 sugar over the existing `Solid::user_fn` escape hatch, NOT in sim-soft as a `DifferenceSdf<Box<dyn Sdf>>` heterogeneous escape hatch. F5's contract tests (`from_sdf_composes_with_subtract`, `from_sdf_preserves_sign_convention_of_wrapped_sdf`, `from_sdf_plumbs_bounds_through_to_mesher`) cover exactly the sub-cases this row consumes.

The static fit pose makes the relative-comparison fit-tightness story concrete — `PenaltyRigidContact::new(vec![scan_sdf])` with the SAME scan SDF as the rigid indenter (first non-plane consumer of the post-PR2 `Sdf`-trait-unified `PenaltyRigidContact`). At rest the cavity walls already kiss the indenter (the cavity IS scan-shaped, so cavity-wall vertices lie on the scan's zero isosurface at rest); the contact band `sd < d̂` activates immediately, and the body's static equilibrium under the rest-state preload IS the quantitative fit-tightness force.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan-derived rigid indenter" throughout this crate's prose. No anatomical references appear in any tracked surface. The 12-tri cube placeholder is a programmatic synthetic stand-in: the pipeline demonstration is the workflow ("scan → `SignedDistanceField` → cf-design `Solid` → `SdfMeshedTetMesh` → 3-material FEM → contact"), not the cube's specific geometry. Production runs swap the cube fixture for a real scan via row 15's `STL → load_mesh → SignedDistanceField::new` path without any other code change.

[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 8 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `counts_exact`

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `6860` | first-run capture (post-F5, `93f4bfaa`, 2026-05-08, rustc 1.95.0, macOS arm64) |
| `n_vertices`          | `4773` | first-run capture |
| `n_referenced`        | `1576` | `n_vertices - n_orphaned = 3197 orphans` |
| `n_pinned` (R_OUTER band)  | `710` | diverges from rows 11+16's `734` — the cavity-asymmetry reduces the BCC outer-band lattice population by a handful of vertices |
| `n_inner_tets` (ECOFLEX_00_30)  | `856` | inner-shell tet count by centroid radial-bin |
| `n_middle_tets` (DRAGON_SKIN_10A) | `2035` | middle-shell tet count |
| `n_outer_tets` (ECOFLEX_00_30)  | `3969` | outer-shell tet count |
| sum | `856 + 2035 + 3969 = 6860` | partition gate (every tet centroid in exactly one shell) |

Cross-row continuity to rows 11+16 does NOT extend (pattern (y) at row 19's banking memo): row 11+16 use a spherical `R_CAVITY = 0.04 m` cavity; row 20 uses an offset 12-tri-cube cavity. Geometry differs ⇒ counts differ. The `R_OUTER = 0.10 m` value matches rows 11+16 visually but is row-20-specific in this row's constant block; the ~3 % `n_pinned` drop reflects the cavity-carve cutting back the outer-band lattice population near the cube's `+x` face.

### 2. `zslab_counts_exact`

| Count | Pinned |
|---|---|
| `n_inner_tets_zslab`  | `144` (per-shell partition of the z-slab cut) |
| `n_middle_tets_zslab` | `270` |
| `n_outer_tets_zslab`  | `258` |

Z-slab cut is `|centroid.z| < CELL_SIZE / 2 = 0.01`; total z-slab tet count is `672`, ~9.8 % of the body — matches rows 11+16's z-slab fraction (~10 %).

### 3. `quality_floors`

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Every tet has positive signed volume — the BCC + IS pipeline preserves orientation through the heterogeneous-CSG carve; same Theorem-1-sanity envelope row 3 [`sdf-to-tet-sphere`](../sdf-to-tet-sphere) anchors verbatim.

### 4. `solver_converges`

| Anchor | Bound |
|---|---|
| `step.iter_count` | `< MAX_NEWTON_ITER = 50` (observed: `7`) |
| `step.final_residual_norm` | `< 1e-10` (observed: `~1.12e-12`) |

A single backward-Euler `replay_step` at `STATIC_DT = 1.0 s` converges from rest in 7 Newton iters. Convergence is slower than rows 11+16's `3` iters because the contact-bearing nonlinearity (penalty force gradient at active-band vertices) augments the Newton residual relative to the pure-pressure rows.

### 5. `material_provenance`

| Material | Lamé pair | Energy round-trip |
|---|---|---|
| `ECOFLEX_00_30` (outer + inner) | μ = 23 kPa, λ = 92 kPa | `nh.energy(I) == 0.0` AND `nh.energy(diag(1.01, 1, 1))` matches the closed-form FMA chain bit-equally |
| `DRAGON_SKIN_10A` (middle) | μ = 51 kPa, λ = 204 kPa | same identity at `epsilon = 0.0` |

F4 const-fn `to_neo_hookean()` survives the const-table → `NeoHookean::from_lame` bridge bit-equally per F4's contract test (`silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`). Re-asserted here so a regression in F4 trips this row directly. Same precedent as row 19's per-row contract walk.

### 6. `n_contact_pairs_exact`

| Anchor | Pinned |
|---|---|
| `n_active_pairs` at static fit pose | `127` |

At rest the cavity walls already kiss the indenter (the cavity IS scan-shaped, so cavity-wall vertices lie on the scan's zero isosurface). The `PenaltyRigidContact::new` default `d̂` band activates `127` cavity-wall vertices.

### 7. `captured_bits` — IV-1 sparse-tier rel-tol

Rest-state contact reaction force + cavity-wall mean displacement bits self-pinned at first capture:

| Anchor | Pinned bits | Approximate value |
|---|---|---|
| `force_total_z_n` | `0x3fc4_0790_5286_ba74` | `~0.156 N` (axial component of the rest-state preload) |
| `cavity_wall_mean_disp_m` | `0x3f53_d596_7b05_b573` | `~1.21e-3 m` (over the 127-vertex active-contact-pair set) |

Compared via `assert_relative_eq!` at `SPARSE_REL_TOL = 1e-12` rel + `SPARSE_EPS_ABS = 1e-12` floor. **Failure-mode protocol per IV-1**: if the rel-tol comparison fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture); (2) if same toolchain, real regression in cf-design's `Solid::from_sdf` plumbing OR mesh-sdf's `distance` heuristic OR sim-soft's BCC + IS + faer hot path; (3) NEVER re-bake to silence drift.

`CF_CAPTURE_BITS=1` env-var bootstrap pattern (banked at row 19 as pattern (cc)): when set, every captured-anchor check (counts AND bits) is bypassed and a paste-ready capture block is printed to stderr. Use for first-time author-bake and intentional re-bake (e.g., F4 const value updated to a new data-sheet revision); never for failure silencing.

### 8. `sanitization_grep`

Static prose check, run by reviewers at every cold-read pass: every mention of the scanned reference geometry uses "scanned reference geometry" or "scan-derived rigid indenter" framing. Negative grep for anatomical terms across `src/main.rs`, `README.md`, `Cargo.toml`. Per pattern (dd) parallel-source-of-truth amendment: any prose-rewrite during cold-read must grep ALL parallel surfaces (module docstring + README + commit message) before declaring the cold-read pass done.

## Visuals

`out/device_zslab.ply` — z-slab per-tet centroid cloud (`672` centroids in `|rest_centroid.z| < CELL_SIZE / 2 = 0.01 m`) with `DISPLACEMENT_SCALE = 50.0` geometric amplification on positions (`amplified = rest_centroid + SCALE * (deformed_centroid - rest_centroid)`). Same z-slab pattern as rows 11+16, with two scalars: categorical `material_id` (0 = inner / 1 = middle / 2 = outer) + sequential `displacement_magnitude` (true physical magnitude, unscaled).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/layered-silicone-device/out/device_zslab.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** is binary-categorical (3 distinct values 0/1/2) → cf-view picks the **categorical palette** (row 11 + 16 precedent for shell-id readouts).
- **`displacement_magnitude`** is unipolar continuous (range `[0, ~4e-3]` m) → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D annulus on `z = 0`. Three concentric rings (categorical `material_id`) overlap with a continuous radial-decay gradient (sequential `displacement_magnitude`):

- **Inner ring** at `|p_xy| ∈ [~0.025, ~0.05] m` — 144 inner-shell centroids; displacement magnitudes cluster near the cavity-wall preload signal (~1e-3 m mean for active-contact-pair vertices). Cross-readout: HEADLINE 7's `cavity_wall_mean_disp_m ≈ 1.21e-3 m`.
- **Middle ring** at `|p_xy| ∈ [~0.05, ~0.075] m` — 270 middle-shell centroids; intermediate displacement (composite stiffness damps the inner-shell-driven cavity push).
- **Outer ring** at `|p_xy| ∈ [~0.075, ~0.097] m` — 258 outer-shell centroids; displacement decays toward 0 at `R_OUTER` (Dirichlet pin band).

Cavity asymmetry is visible: the offset 12-tri cube cavity at `(0.015, 0, 0)` produces an inner-ring distribution where the cavity-wall vertices on the cube's `-x` face show the highest displacement magnitudes — the cube's `-x` face at `x = -0.010 m` sees a `0.090 m` silicone shell out to the outer pin at `x = -0.10 m`, vs only a `0.060 m` shell from the cube's `+x` face at `x = 0.040 m` to the outer pin at `x = +0.10 m`. The thicker `-x` shell gives the cavity-wall vertices a more compliant elastic response under the rest-state contact preload, so they displace farther; the thinner `+x` shell sits more constrained against the outer Dirichlet pin and displaces less. The eyes-on-pixels review of `out/device_zslab.ply` confirms the high-displacement cluster on the screen-LEFT side of cf-view's default oblique orbit, consistent with the `-x` direction.

**Why z-slab over the full-boundary-surface artifact (row 3 sphere precedent)**: per pattern (aa) banked at row 16 N+3, hollow / interior-cavity / partial-occlusion bodies' full boundary surfaces 360°-occlude the cavity and the inner/middle interfaces from every cf-view orbit angle (cf-view exposes no section-cut UI). The z-slab projects centroids onto a 2-D annulus cut, exposing both the radial material-shell partition and the cavity-wall displacement response. Row 20's geometry is doubly hollow (scan-shaped cavity AND three concentric shells) → z-slab is required by construction; row 16's N+3 pivot is the precedent.

## Run

```sh
cargo run -p example-sim-soft-layered-silicone-device --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve at this mesh resolution (`6860` tets through faer's sparse Cholesky + 7 Newton iters with penalty-contact); debug mode would take many minutes for what runs in seconds release.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## First-time bit capture

```sh
CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-layered-silicone-device --release
```

Emits a paste-ready block of every `*_EXACT` count and every `*_REF_BITS` constant, bypassing the captured-anchor checks. Use for first-time author-bake and intentional re-bake; the IV-1 protocol forbids using this to silence a drift assertion.
