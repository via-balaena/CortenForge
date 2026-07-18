# layered-silicone-device

**PR3 row 20 — the Tier 6 synthesis row, closing the entire sim-soft examples arc.** A layered silicone device with a scan-derived cavity is composed end-to-end through every PR3 foundation piece: scan SDF (F2 `impl cf_design::Sdf for mesh_sdf::Signed<TriMeshDistance, _>`) → typed-Solid heterogeneous CSG (F5 `Solid::from_sdf` sugar over `user_fn`) → BCC + Isosurface Stuffing tet pipeline (F1 `impl Sdf for Solid` + F3 `sim_soft::Sdf` re-export) → 3-shell `MaterialField` from the silicone constants table (F4 `silicone_table.rs`) → static fit pose with the same scan SDF as the rigid indenter (post-PR2 trait unification: any `impl Sdf` is a valid rigid primitive directly). The relative-comparison "fit-tightness" force readout at zero indenter displacement IS the device memo's "feels right" subjective tightness target made quantitative — the cavity walls already kiss the indenter at rest because the cavity IS scan-shaped.

The device geometry is a 3-layer hollow body: outer + inner shells of `ECOFLEX_00_30` (μ = 23 kPa); middle shell of `DRAGON_SKIN_10A` (μ = 51 kPa, the closest pure-silicone proxy in F4 for the conductive-composite middle layer that real builds carry copper mesh + carbon black through). Each shell is 25 mm thick (`R_OUTER = 0.10 m`, `R_MIDDLE_OUTER = 0.075 m`, `R_INNER_OUTER = 0.05 m`); the cavity is a scan-derived asymmetric region — a 12-triangle cube fixture (`R_SCAN = 0.025 m`, offset `(0.015, 0, 0)` so the carve is demonstrably non-spherical) — subtracted from the inner shell.

| Shell | Radial band | F4 material | `(μ, λ)` |
|---|---|---|---|
| Inner  | `‖p‖ ∈ [0, 0.05) m`      | `ECOFLEX_00_30`   | μ = 23 kPa, λ = 92 kPa  |
| Middle | `‖p‖ ∈ [0.05, 0.075) m`  | `DRAGON_SKIN_10A` | μ = 51 kPa, λ = 204 kPa |
| Outer  | `‖p‖ ∈ [0.075, 0.10] m`  | `ECOFLEX_00_30`   | μ = 23 kPa, λ = 92 kPa  |

The material partition is by `‖p‖` radial bin from the origin (parametric, NOT scan-shaped) — the cavity asymmetry does not mix into the material classes. The DS10A proxy for the conductive composite is F4's silicone matrix only; the Cu mesh + carbon black mechanical uplift is deferred to a Fork-B post-cast modulus calibration that absorbs uplift into the effective μ at calibration time.

Every gate sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. This row is a Rule-B **validator**: its gates are pipeline-emergent structural + physics invariants read from the real solve, not captured-bit self-pins (those were stripped in the Rule-B de-frag — constitutive and mesher correctness are lib-owned). Outputs are `out/layered_silicone_device.json` (fit-pose scalars + 3-material provenance + per-active-contact-pair detail) and the F1.5 / F2 viz PLY surfaces (see [Visuals](#visuals)).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The full PR3 bridge stack composed in one expression — a scan-derived cavity carved from a parametric outer body via heterogeneous CSG in cf-design's typed kernel:

```rust
let scan_sdf = build_scan_fixture();                             // Signed<TriMeshDistance, _>, F2
let cavity = Solid::from_sdf(scan_sdf.clone(), scan_bounds());   // typed Solid leaf, F5
let body = Solid::sphere(R_OUTER).subtract(cavity);             // typed CSG, cf-design kernel

let hints = MeshingHints {
    bbox: Aabb3::new(/* ±0.12 m */),
    cell_size: 0.01,
    material_field: Some(build_material_field()),                // 3-shell radial partition, F4 + LayeredScalarField
};

let mesh = SdfMeshedTetMesh::from_sdf(&body, &hints)?;           // F1 + F3, &dyn Sdf trait dispatch
```

The scan-derived cavity carve composes with the parametric outer sphere through `Solid::subtract` — the same boolean operator as parametric-only CSG, no per-shape glue. This is the architectural payload of PR3 — the heterogeneous-CSG bridge between parametric typed bodies (cf-design `Solid` primitives) and scan-derived SDFs (mesh-sdf-backed) lives in cf-design's typed kernel as F5 sugar over the existing `Solid::user_fn` escape hatch, NOT in sim-soft as a `DifferenceSdf<Box<dyn Sdf>>` heterogeneous escape hatch. F5's contract tests (`from_sdf_composes_with_subtract`, `from_sdf_preserves_sign_convention_of_wrapped_sdf`, `from_sdf_plumbs_bounds_through_to_mesher`) cover exactly the sub-cases this row consumes.

The static fit pose makes the relative-comparison fit-tightness story concrete — `PenaltyRigidContact::new(vec![scan_sdf])` with the SAME scan SDF as the rigid indenter (first non-plane consumer of the post-PR2 `Sdf`-trait-unified `PenaltyRigidContact`). At rest the cavity walls already kiss the indenter (the cavity IS scan-shaped, so cavity-wall vertices lie on the scan's zero isosurface at rest); the contact band `sd < d̂` activates immediately, and the body's static equilibrium under the rest-state preload IS the quantitative fit-tightness force.

The same body expression scales — replacing the 12-tri cube fixture with `TriMeshDistance::new(loaded_scan)` + `PseudoNormalSign::from_distance(...)` (row 15's STL-import precedent) lifts a real scan into the pipeline. Production runs swap the fixture builder for an STL load and every downstream code path (`Solid::from_sdf` + `SdfMeshedTetMesh::from_sdf` + the contact `PenaltyRigidContact::new(scan.clone())` line) stays unchanged.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan-derived rigid indenter" throughout this crate's prose. No anatomical references appear in any tracked surface. The 12-tri cube placeholder is a programmatic synthetic stand-in: the pipeline demonstration is the workflow ("scan → `Signed<TriMeshDistance, _>` → cf-design `Solid` → `SdfMeshedTetMesh` → 3-material FEM → contact"), not the cube's specific geometry. Production runs swap the cube fixture for a real scan via row 15's `STL → TriMeshDistance::new` path without any other code change.

[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md

## Numerical anchors

Each gate is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*` and is called from `main()` in dependency order; `cargo run --release` exit-0 means every gate passed. They are **pipeline-emergent structural + physics invariants** read from the real solve — resolution- and toolchain-robust. The pre-Rule-B captured-bit self-pins (exact tet/vertex/pair counts, `to_bits()` force/displacement pins, the `to_neo_hookean()` provenance mirror, and the `CF_CAPTURE_BITS` capture scaffold) were **stripped**: they froze one run's FP trajectory + mesher version on one toolchain (strictly more fragile than the invariants they redundantly implied), and constitutive correctness (`NeoHookean` closed form + `to_neo_hookean()` round-trip) is lib-owned (`neo_hookean.rs` tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`), as is the generic radial-bin shell routing (`sdf_material_tagging.rs` IV-4). The observed scalars (force, displacement) are still emitted to the JSON readout + stdout for eyes-on inspection — just not self-pinned.

### 1. `quality_floors`

| Gate | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Every tet has positive signed volume — the BCC + Isosurface Stuffing pipeline preserves orientation through the heterogeneous-CSG carve; same Theorem-1-sanity envelope row 3 [`sdf_to_tet`](../sdf/stress-test) anchors verbatim. This gate is also the pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse().expect(...)` invariant — `D_rest` is invertible iff the tet has positive signed volume.

### 2. `mesh_structure`

| Gate | Bound |
|---|---|
| `n_tets`                 | `> 0` |
| `n_referenced`           | `> 0` and `≤ n_vertices` (referenced set ⊆ all vertices; the excess are orphan BCC lattice corners the solver skips) |
| `n_pinned` (R_OUTER band) | non-empty proper subset of `n_referenced` (every vertex with `(‖p‖ - R_OUTER).abs() < CELL_SIZE / 2`) |
| each per-shell tet count | `> 0` (all three material bands populated) |

Structural invariants, not exact counts: the specific tet/vertex counts are a mesher-version artifact (they change on any BCC/stuffing improvement), whereas non-emptiness is the property that actually matters. (The three per-shell counts partition every tet exactly once by construction — the caller derives them from a total match over all tets — so a `sum == n_tets` assert would guard nothing; the per-shell routing is checked independently in gate 4.) The generic per-shell routing correctness is lib-owned (`sdf_material_tagging.rs` IV-4). (Observed at `CELL_SIZE = 0.01`: `n_tets = 51_292`, `n_vertices = 32_162`, `n_referenced = 10_013`, `n_pinned = 2_842`, per-shell `5_068 / 14_736 / 31_488` — emitted to stdout, not pinned.)

### 3. `zslab_populations`

| Gate | Bound |
|---|---|
| each z-slab per-shell tet count (`\|cz\| < CELL_SIZE / 2 = 0.005`) | `> 0` |

Each material band contributes at least one tet to the `z = 0` cut, so the cf-view cross-section artifacts show all three shells. Non-empty, not exact-count (mesher-version robust).

### 4. `material_routing`

| Gate | Bound |
|---|---|
| `mesh.materials()[t]` per tet | `(μ, λ)` bit-equal to the shell's F4 table entry (`ECOFLEX_00_30` inner + outer, `DRAGON_SKIN_10A` middle, by radial bin), each shell exercised ≥ 1 tet |

The scene's multi-material routing invariant — verifies that the `MaterialField` assigned every tet the `(μ, λ)` of the radial shell its centroid falls in. Reads the real per-tet `NeoHookean` from `mesh.materials()` via the public `.mu()` / `.lambda()` accessors and compares to the shell's F4 table entry with an exact `to_bits()` `==` (both come from the same const through the same `to_neo_hookean()` const fn, so a correctly routed tet is bit-identical — this is a routing check, NOT a constitutive-arithmetic mirror). A boundary-convention drift between `partition_point(|&t| t <= phi)` and `shell_at` would fire it immediately. The constitutive math and the `to_neo_hookean()` Lamé round-trip are lib-owned (`neo_hookean.rs` tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`); the generic routing mechanism is lib-owned (`sdf_material_tagging.rs` IV-4).

### 5. `solver_converges`

| Gate | Bound |
|---|---|
| `step.iter_count` | `< MAX_NEWTON_ITER = 200` (observed: `37`) |
| `step.final_residual_norm` | `< 1e-10` (observed: `~8.7e-11`) |

A single backward-Euler `replay_step` at `STATIC_DT = 1.0 s` converges from rest in 37 Newton iters. The contact-bearing nonlinearity (penalty force gradient at active-band vertices) augments the Newton residual relative to a pure-pressure row; the `MAX_NEWTON_ITER = 200` cap leaves comfortable headroom at the refined `CELL_SIZE = 0.01` mesh, where the initial cavity-wall/indenter overlap spans more vertices than at the original coarse mesh.

### 6. `contact_engaged`

| Gate | Bound |
|---|---|
| `n_active_pairs` at static fit pose | `> 0` (the cavity walls actually engaged the scan-derived indenter; the exact count was a mesher/discretization artifact) |

At rest the cavity walls already kiss the indenter (the cavity IS scan-shaped, so cavity-wall vertices lie on the scan's zero isosurface). The `PenaltyRigidContact::new` default `d̂` band activates the cavity-wall vertices within `d̂`. (Observed: `442` active pairs; rest-state axial reaction `force_total_z ≈ -2.24e-4 N`, cavity-wall mean / max displacement `~6.12e-4 m` / `~1.11e-3 m` — emitted to the JSON readout for inspection.)

## Visuals

The F1.5 / F2 viz retrofit emits the full 3D body + design-mesh surfaces via the public `sim_soft::viz` API (the pre-F1.5 amplified z-slab centroid cloud was retired). Every artifact carries three per-vertex scalars — `displacement_magnitude`, categorical `material_id` (0 = inner / 1 = middle / 2 = outer), and `psi_j_per_m3` (per-tet strain-energy density):

| Artifact | Emitter | What it shows |
|---|---|---|
| `out/device_boundary.ply` | `viz::boundary_surface` (F1.1) | full 3D analysis-mesh outer surface — the BCC boundary faithfully, sliver-tet zigzag included |
| `out/device_slab_cut_z0.ply` | `viz::slab_cut` (F1.1) | equatorial `z = 0` cross-section — exposes the radial material shells + cavity wall in one cut |
| `out/device_design_slab_cut_z0.ply` | `viz::design_slab_cut` (F2.0) | same cut, marching-squares on the design SDF (smooth circle minus sharp square), decoupled from the analysis discretization |
| `out/device_design_surface.ply` | `viz::design_surface` (F2.1) | full 3D body, marching-cubes on the design SDF — smooth sphere boundary without the BCC zigzag |
| `out/device_design_surface_deformed.ply` | `viz::design_surface_deformed` (F2.3a) | design surface offset by the per-vertex displacement field, `amplify = 10` to make the mm-scale deformation visible on the cm-scale body |
| `out/device_design_scene.ply` | `viz::design_scene` (F2.3b) | body + scan-derived indenter merged, categorical `primitive_id` scalar (0 = body / 1 = indenter) |

Open any of them in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/layered-silicone-device/out/device_slab_cut_z0.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15 — `material_id` is binary-categorical (3 values 0/1/2) → categorical palette; `displacement_magnitude` and `psi_j_per_m3` are unipolar continuous → sequential viridis. The `slab_cut` at `z = 0` exposes the three concentric radial shells; the cavity asymmetry (offset cube at `(0.015, 0, 0)`) shows as a non-circular cavity cross-section, and the `design_surface_deformed` artifact shows the cavity walls squashing inward where the rigid indenter presses while the outer sphere boundary deforms far less (the silicone bulk absorbs the contact force). Pair the F1 `boundary_surface` / `slab_cut` artifacts with their F2 `design_*` counterparts for the analysis-mesh-vs-design-mesh side-by-side that motivated the F2 arc.

## Run

```sh
cargo run -p example-sim-soft-layered-silicone-device --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve at this mesh resolution (`~51 k` tets through faer's sparse Cholesky + 37 Newton iters with penalty-contact); debug mode would take many minutes for what runs in seconds release.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md
