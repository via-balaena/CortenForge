# scan-fit-3layer-sleeve

**Row 21 — Tier 6 synthesis #2, the second end-to-end relative-comparison sim of the [layered silicone device][mem] cavity-fit workflow, after row 20.** Where row 20 carved a 12-tri-cube cavity from a parametric outer sphere, this row models the manufacturing target's geometry directly: a thin-walled 3-layer sleeve wrapping a cuboid scan stand-in, exercised by a spherical rigid intrusion probe under a static overlap pose. The same PR3 bridge stack composes: cf-design typed primitives (`Solid::cuboid`, `Solid::offset`, `Solid::subtract`, `Solid::sphere`, `Solid::translate`) feed the body and the rigid probe; `SdfMeshedTetMesh::from_sdf` (F1+F3) builds the tet mesh; `LayeredScalarField` partitioned by **distance-from-scan** drives the 3-layer `MaterialField` with F4 silicone-table constants; `PenaltyRigidContact::new` takes the typed-Solid probe directly post-PR2 trait unification.

This row is the first sim-soft user-facing example to demonstrate **post-solve per-tet strain energy density extraction** (`Ψ_t = Material::energy(F_t)` with `F_t` reconstructed inline from rest + deformed positions). Per-layer mean Ψ̄ aggregates drive the headline mechanical readout — strict-monotone ordering `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` reflects the compounded effect of compliance (inner is softest), distance-to-load (inner is at the cavity wall directly contacting the probe), and distance-to-constraint (outer is Dirichlet-pinned). All three effects compound in the same direction; the ordering is robust to small drift in the per-tet F arithmetic.

The 3 layers map to the manufacturing build sequence (v1's `6/4/4` mm split is a numerical proxy for the user-target hardware build's `6/5/3` mm split — the v1 cell size needs each layer to carry ≥ 1 BCC cell across thickness, so the middle and outer absorb the ±0.5 mm rebalance; both layers stay meaningfully resolved and the strain-energy ordering is unaffected):

| Layer | Thickness (v1) | Hardware target | Source material | F4 proxy | Engineering role |
|---|---|---|---|---|---|
| Inner  | 6 mm | 6 mm | Ecoflex 00-30 + 75 % Slacker (effective Shore 00-20) | `ECOFLEX_00_20`   (μ = 18 kPa, λ = 72 kPa)   | skin-contact softness |
| Middle | 4 mm | 5 mm | DS10A + Cu mesh + carbon black (effective Shore 15-18A) | `DRAGON_SKIN_10A` (μ = 51 kPa, λ = 204 kPa)  | conductive composite |
| Outer  | 4 mm | 3 mm | DS20A direct                                      | `DRAGON_SKIN_20A` (μ = 113 kPa, λ = 452 kPa) | structural stiffness |

v2's quasi-static intrusion ramp + finer cells will recover the user-target 6/5/3 split; the v1 layout's mechanical story (compliance + distance-to-load + distance-to-constraint compounding ordering) carries through to v2 unchanged.

The DS10A proxy for the conductive composite mirrors row 20's precedent verbatim — F4 models the silicone matrix only, and the Cu mesh + carbon black mechanical uplift is deferred to a Fork-B post-cast modulus calibration that absorbs uplift into the effective μ at calibration time. Cu / CB / Slacker contributions land in F4 the moment a calibrated entry exists.

Probe penetration in v1 is 1 mm (kept gentle so the static overlap pose's iter-0 penalty gradient does not invert tets in the first Newton step — the 18-113 kPa silicone stack with the stiff outer Dirichlet pin is sensitive to deep initial overlap on a single quasi-static replay step). Deeper penetration (the user-target 8 mm physical intrusion) flows through v2's multi-step ramp, where each step's gradient stays within Newton's basin of convergence.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Outputs are `out/scan_fit_3layer_sleeve.json` (fit-pose scalars + 3-material provenance + per-layer Ψ̄ + per-active-contact-pair detail) and `out/sleeve_zslab.ply` (z-slab per-tet centroid cloud with categorical `material_id` + sequential `displacement_magnitude`).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md

## What this example demonstrates

**Workflow** (the architectural payload — the same expression composes for any scan-shaped sleeve with parametric wrap thickness):

```rust
// 1. Scan stand-in (production swaps for a real scan via row 15's STL path).
//    Cuboid is an *exact* SDF — required for offset to produce a true
//    14 mm-thick wrap shell (non-exact SDFs like superellipsoid /
//    ellipsoid shift the level set in non-distance units, producing a
//    paper-thin or empty wrap shell at meshing time).
let scan_solid = Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, SCAN_HZ));

// 2. Outer envelope = scan offset by WRAP_THICKNESS.
let outer_envelope = scan_solid.clone().offset(WRAP_THICKNESS);

// 3. Sleeve body = outer envelope ⊖ scan (homogeneous typed CSG;
//    F5 heterogeneous bridge is NOT exercised — row 20 covers F5).
let sleeve_body = outer_envelope.clone().subtract(scan_solid.clone());

// 4. 3-layer MaterialField partitioned by distance-from-scan.
let mu_field = LayeredScalarField::new(
    Box::new(scan_solid.clone()) as Box<dyn Sdf>,
    vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
    vec![ECOFLEX_00_20.mu, DRAGON_SKIN_10A.mu, DRAGON_SKIN_20A.mu],
);
// (lambda_field built the same way; both feed MaterialField::from_fields)

// 5. Mesh through F1+F3 trait dispatch.
let hints = MeshingHints {
    bbox: Aabb3::new(/* ±BBOX_HALF_X/Y/Z */),
    cell_size: 0.004, // 4 mm — each layer (6/4/4 mm) carries ≥1 cell.
    material_field: Some(material_field),
};
let mesh = SdfMeshedTetMesh::from_sdf(&sleeve_body, &hints)?;

// 6. Static fit pose: outer-envelope Dirichlet pin + spherical rigid
//    intrusion probe via PenaltyRigidContact::new (post-PR2 trait
//    unification: any impl Sdf is a valid rigid primitive directly).
let probe = Solid::sphere(PROBE_RADIUS)
    .translate(Vector3::new(0.0, 0.0, PROBE_CENTER_Z));
let contact = PenaltyRigidContact::new(vec![probe]);
let step = solver.replay_step(/* ... */);

// 7. Per-tet strain energy density (NEW capability).
let f_t = deformation_gradient(verts, &rest_positions, &positions_vec3);
let psi_t = mesh.materials()[t].energy(&f_t);
```

The same body expression scales — replacing the cuboid with `Solid::from_sdf(SignedDistanceField::new(loaded_scan), bounds)` (PR3 F5 path; row 20's precedent) lifts a real scan into the typed-Solid kernel. Production runs swap step 1 for an STL load and the rest stays identical.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The superellipsoid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material FEM → rigid intrusion contact"), not the superellipsoid's specific geometry. Production runs swap the superellipsoid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 9 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `quality_floors`

| Anchor | Bound |
|---|---|
| `signed_volume > 0` per tet | strict (D-10 detector) |

Every tet has positive signed volume — the BCC + Isosurface Stuffing pipeline preserves orientation through the heterogeneous-CSG carve; same Theorem-1-sanity envelope row 3 [`sdf-to-tet-sphere`](../sdf-to-tet-sphere) anchors verbatim. This anchor is also the pre-condition for the per-tet `deformation_gradient` helper's `D_rest.try_inverse().expect(...)` invariant — `D_rest` is invertible iff the tet has positive signed volume.

### 2. `counts_exact`

| Count | Pinned | Source |
|---|---|---|
| `n_tets`              | `74_628` | first-run capture, post-Q7 tip `13e46dad`, 2026-05-08, rustc 1.95.0, macOS arm64 |
| `n_vertices`          | `31_966` | first-run capture |
| `n_referenced`        | `17_384` | `n_vertices - n_orphaned = 14_582` orphan BCC lattice corners excluded from solver participation |
| `n_pinned` (outer-envelope band) | `7_046` | every vertex with `\|outer_envelope.eval(p)\| < CELL_SIZE / 2`, filtered to referenced set |
| `n_inner_tets` (`ECOFLEX_00_20`)   | `25_892` | inner-layer tet count by distance-from-scan centroid bin |
| `n_middle_tets` (`DRAGON_SKIN_10A`)| `16_656` | middle-layer tet count |
| `n_outer_tets` (`DRAGON_SKIN_20A`) | `32_080` | outer-layer tet count |
| sum                                | `74_628` | partition gate (every tet centroid sits in exactly one shell) |

Cross-row continuity to rows 11+16+20 does NOT extend (pattern (y) at row 19's banking memo): all prior rows use spherical body geometries; row 21 uses a cuboid + offset wrap. Geometry differs ⇒ counts differ. Materials also diverge (rows 11+16 use uniform `(μ, λ) = (1e5, 4e5)`; row 20 uses `ECOFLEX_00_30`/`DRAGON_SKIN_10A`/`ECOFLEX_00_30`; row 21 uses `ECOFLEX_00_20`/`DRAGON_SKIN_10A`/`DRAGON_SKIN_20A`).

### 3. `zslab_counts_exact`

| Count | Pinned |
|---|---|
| `n_inner_tets_zslab`  | `768` (per-shell partition of the z-slab cut at `\|cz\| < CELL_SIZE / 2 = 0.002`) |
| `n_middle_tets_zslab` | `432` |
| `n_outer_tets_zslab`  | `892` |

Total z-slab tet count is `2_092`, ~2.8 % of the body — thinner fractional slab than rows 11+16+20 because the body is z-elongated (full z-extent ~108 mm vs cell size 4 mm).

### 4. `solver_converges`

| Anchor | Bound |
|---|---|
| `step.iter_count` | `< MAX_NEWTON_ITER = 50` (observed: `9`) |
| `step.final_residual_norm` | `< 1e-10` (observed: `~3.04e-11`) |

A single backward-Euler `replay_step` at `STATIC_DT = 1.0 s` converges from rest in 9 Newton iters. Slightly slower than row 20's 7 iters because the row-21 `(μ, λ)` stack is stiffer at the outer layer (`DRAGON_SKIN_20A` vs row 20's `ECOFLEX_00_30`), so the penalty-active Newton residual takes more iterations to drop below `1e-10`.

### 5. `material_provenance`

| Material | Lamé pair | Energy round-trip |
|---|---|---|
| `ECOFLEX_00_20` (inner) | μ = 18 kPa, λ = 72 kPa  | `nh.energy(I) == 0.0` AND `nh.energy(diag(1.01, 1, 1))` matches the closed-form FMA chain bit-equally |
| `DRAGON_SKIN_10A` (middle) | μ = 51 kPa, λ = 204 kPa | same identity at `epsilon = 0.0` |
| `DRAGON_SKIN_20A` (outer)  | μ = 113 kPa, λ = 452 kPa | same identity at `epsilon = 0.0` |

F4 const-fn `to_neo_hookean()` survives the const-table → `NeoHookean::from_lame` bridge bit-equally per F4's contract test (`silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`). Re-asserted here so a regression in F4 trips this row directly. Same precedent as rows 19 + 20.

### 6. `material_assignment_partition`

| Anchor | Bound |
|---|---|
| `mesh.materials()[t].energy(F_probe)` per tet | bit-equal to `expected_nh[shell_at(centroid)].energy(F_probe)` at `epsilon = 0.0` (row 8 pattern (a) probe at `F_probe = diag(1.01, 1, 1)`) |

Headline gate of the row's multi-material correctness — verifies that every per-tet `NeoHookean` cached by `MaterialField::sample` at mesh-build time matches the `(μ, λ)` pair the centroid's shell would assign by direct lookup at `silicone_table.rs`. Probe via the `Material::energy` trait at `F = diag(1.01, 1, 1)` (row 8 banked pattern): both sides run identical NH arithmetic on identical `(μ, λ)`, bit-equal by construction on a fixed toolchain. A regression where the LayeredScalarField partition disagrees with the centroid-bin lookup (e.g., a boundary-convention drift between `partition_point(|&t| t <= phi)` and `shell_at_phi`) would fire this anchor immediately.

### 7. `strain_energy_ordering`

| Anchor | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict |
| `Ψ̄_middle > Ψ̄_outer` | strict |

Three effects compound in the same direction: (a) compliance — inner is softest (μ = 18 kPa), outer is stiffest (μ = 113 kPa); under the same probe-driven displacement field (continuous through the bonded multi-material body), strain concentrates in the softer layers; (b) distance to load — inner is at the cavity wall directly contacting the probe; the strain field decays radially outward; (c) distance to constraint — outer is pinned at the Dirichlet band on the outer envelope; the displacement field is forced to zero at the boundary, so outer-layer F stays close to I and Ψ stays small. The strict ordering is robust to small drift in the per-tet F arithmetic.

### 8. `outer_layer_max_psi` + `peak_displacement_bounded`

| Anchor | Pinned |
|---|---|
| `max Ψ_outer`     | bits self-pinned at first capture (~175.8 J/m³); `assert_relative_eq!` at sparse rel-tol |
| `max disp` over active-contact-pair vertices | `< WRAP_THICKNESS = 0.014 m` (strict, geometric sanity — the wrap should not collapse to zero thickness; observed peak ~1.97e-3 m, ~14 % of the bound) |

The outer-layer Ψ peak self-pin catches regressions in the per-tet F-from-positions arithmetic + `Material::energy` chain. The displacement-bounded gate is a hard geometric upper bound: the cavity wall reaching the outer envelope's Dirichlet pin band would mean the wrap collapsed; observed displacement is ~14 % of that bound. (Note: penalty contact's elastic equilibrium can push the cavity wall *farther* than the rigid penetration depth — that's expected behaviour, not a violation.)

### 9. `captured_bits` — IV-1 sparse-tier rel-tol

Static-pose contact reaction force, cavity-wall mean / max displacement, and per-layer Ψ̄ aggregates self-pinned at first capture (7 captured-bit anchors total in `verify_captured_bits`, plus `OUTER_PSI_MAX_REF_BITS` in `verify_outer_layer_max_psi`):

| Anchor | Bits | Approximate value |
|---|---|---|
| `force_total_z_n`             | `0xc060_50ca_d2c1_c858` | ~ -130.5 N (`+z`-component of the force-on-soft summed over active pairs) |
| `cavity_wall_mean_disp_m`     | `0x3f0c_f7ff_0bfc_a80c` | ~ 5.5e-5 m = 55 µm |
| `cavity_wall_max_disp_m`      | `0x3f60_2b04_a1ce_3f13` | ~ 1.97e-3 m = 1.97 mm |
| `mean_psi_inner_j_per_m3`     | `0x4022_6b7f_4bef_57af` | ~ 9.21 J/m³ |
| `mean_psi_middle_j_per_m3`    | `0x3ff8_675a_701a_9886` | ~ 1.53 J/m³ |
| `mean_psi_outer_j_per_m3`     | `0x3fd5_e5a3_dc72_1c2a` | ~ 0.342 J/m³ |
| `max_psi_outer_j_per_m3`      | `0x4065_f999_5fa6_15bd` | ~ 175.8 J/m³ (~ 514× outer-layer mean — peak localises in tets adjacent to the contact band) |

Compared via `assert_relative_eq!` at `SPARSE_REL_TOL = 1e-12` rel + `SPARSE_EPS_ABS = 1e-12` floor. **Failure-mode protocol per IV-1**: if the rel-tol comparison fails, do NOT re-bake. Diagnose in this order: (1) rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture); (2) if same toolchain, real regression in cf-design's `cuboid` / `offset` plumbing OR in sim-soft's BCC + IS + faer hot path OR in the inline `deformation_gradient` arithmetic; (3) NEVER re-bake to silence drift.

`CF_CAPTURE_BITS=1` env-var bootstrap pattern (banked at row 19 as pattern (cc)): when set, every captured-anchor check (counts AND bits) is bypassed and a paste-ready capture block is printed to stderr. Use for first-time author-bake and intentional re-bake (e.g., F4 const value updated to a new data-sheet revision); never for failure silencing.

## Visuals

`out/sleeve_zslab.ply` — z-slab per-tet centroid cloud (`2_092` centroids in `|rest_centroid.z| < CELL_SIZE / 2 = 0.002 m`) with `DISPLACEMENT_SCALE = 50.0` geometric amplification on positions (`amplified = rest_centroid + SCALE * (deformed_centroid - rest_centroid)`). Same z-slab pattern as rows 11+16+20, with two scalars: categorical `material_id` (0 = inner / 1 = middle / 2 = outer) + sequential `displacement_magnitude` (true physical magnitude, unscaled).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve/out/sleeve_zslab.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** is binary-categorical (3 distinct values 0/1/2) → cf-view picks the **categorical palette** (rows 11+16+20 precedent for shell-id readouts).
- **`displacement_magnitude`** is unipolar continuous → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D annulus on `z = 0`. Three concentric bands of categorical `material_id` overlap with a continuous `displacement_magnitude` distribution: the inner-layer cavity-wall side near the probe-penetration zone (`z ≈ 0`, but the probe centre is at `z = 0.036 m`, so the z=0 slab is just outside the high-displacement zone) shows the strongest displacement signal off-axis; outer-layer centroids near the Dirichlet pin band fall toward zero displacement.

**Why z-slab over the full-boundary-surface artifact (row 3 sphere precedent)**: per pattern (aa) banked at row 16 N+3, hollow / interior-cavity / partial-occlusion bodies' full boundary surfaces 360°-occlude the cavity and the inner/middle interfaces from every cf-view orbit angle (cf-view exposes no section-cut UI). The z-slab projects centroids onto a 2-D annulus cut, exposing both the radial material-shell partition and the cavity-wall displacement response under probe intrusion. Row 21's geometry is doubly hollow (scan-shaped cavity AND three concentric shells) → z-slab is required by construction; row 16's N+3 pivot is the precedent and rows 20 + 21 ride the precedent.

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve at this mesh resolution (`74_628` tets through faer's sparse Cholesky + 9 Newton iters with penalty-contact); debug mode would take many minutes for what runs in seconds release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness — coarsening further would erase the middle and outer layers, and finer cells (e.g. `0.002 m`) push the per-cell penalty gradient too high under the static-overlap pose, inverting tets in the first Newton step.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## First-time bit capture

```sh
CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-scan-fit-3layer-sleeve --release
```

Emits a paste-ready block of every `*_EXACT` count and every `*_REF_BITS` constant, bypassing the captured-anchor checks. Use for first-time author-bake and intentional re-bake; the IV-1 protocol forbids using this to silence a drift assertion.

## Roadmap (followups, not in v1 scope)

This row is v1 of a queued evolution toward iter-2+ silicone-device design support:

- **v2** — multi-step force-displacement curve (5-10 step ramp + JSON sidecar with capture provenance). Replaces the single static fit pose with a quasi-static intrusion sweep.
- **v3** — axial zoned variation (proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`). Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4** — explicit thin copper-mesh sub-layer (4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A).
- **vN** — real anatomy scan replacing the superellipsoid fixture (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
