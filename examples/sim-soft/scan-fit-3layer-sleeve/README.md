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

Every gate sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. This row is a Rule-B **validator**: its gates are pipeline-emergent structural + physics invariants read from the real solve, not captured-bit self-pins (those were stripped in the Rule-B de-frag — constitutive and mesher correctness are lib-owned). Outputs are `out/scan_fit_3layer_sleeve.json` (fit-pose scalars + per-shell material `(μ, λ)` + per-layer Ψ̄ + per-active-contact-pair detail) and `out/sleeve_zslab.ply` (z-slab per-tet centroid cloud with categorical `material_id` + sequential `displacement_magnitude`).

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

The same body expression scales — replacing the cuboid with `Solid::from_sdf(flood_filled_sdf(loaded_scan, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT)?.0, bounds)` (PR3 F5 path; row 20's precedent) lifts a real scan into the typed-Solid kernel. Production runs swap step 1 for an STL load and the rest stays identical.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow ("scan-shaped body → wrap by offset → carve cavity → 3-material FEM → rigid intrusion contact"), not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via row 15's STL-import path without any other code change.

## Numerical anchors

Each gate is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*` and is called from `main()` in dependency order; `cargo run --release` exit-0 means every gate passed. They are **pipeline-emergent structural + physics invariants** read from the real solve — resolution- and toolchain-robust. The pre-Rule-B captured-bit self-pins (exact tet/vertex/pair counts, `to_bits()` force/displacement/Ψ̄ pins, the `to_neo_hookean()` provenance mirror) were **stripped**: they froze one run's FP trajectory on one toolchain (strictly more fragile than the invariants they redundantly implied), and constitutive correctness (`NeoHookean` closed form + `to_neo_hookean()` round-trip) is lib-owned (`neo_hookean.rs` tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`), as is the generic distance-from-scan shell routing (`sdf_material_tagging.rs` IV-4). The observed scalars (force, displacement, per-layer Ψ̄) are still emitted to the JSON readout + stdout for eyes-on inspection — just not self-pinned.

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
| `n_pinned` (outer-envelope band) | non-empty proper subset of `n_referenced` (every vertex with `\|outer_envelope.eval(p)\| < CELL_SIZE / 2`) |
| each per-shell tet count | `> 0` (all three material bands populated) |

Structural invariants, not exact counts: the specific tet/vertex counts are a mesher-version artifact (they change on any BCC/stuffing improvement), whereas non-emptiness is the property that actually matters. (The three per-shell counts partition every tet exactly once by construction — the caller derives them from a total match over all tets — so a `sum == n_tets` assert would guard nothing; the per-shell routing is checked independently in gate 5.) The generic per-shell routing correctness is lib-owned (`sdf_material_tagging.rs` IV-4); see gate 5 for this scene's routing check.

### 3. `zslab_populations`

| Gate | Bound |
|---|---|
| each z-slab per-shell tet count (`\|cz\| < CELL_SIZE / 2 = 0.002`) | `> 0` |

Each material band contributes at least one tet to the `z = 0` cut, so the cf-view PLY artifact shows all three shells. Non-empty, not exact-count (mesher-version robust).

### 4. `solver_converges`

| Gate | Bound |
|---|---|
| `step.iter_count` | `< MAX_NEWTON_ITER = 50` (observed: `9`) |
| `step.final_residual_norm` | `< 1e-10` (observed: `~3.04e-11`) |

A single backward-Euler `replay_step` at `STATIC_DT = 1.0 s` converges from rest in 9 Newton iters. Slightly slower than row 20's 7 iters because the row-21 `(μ, λ)` stack is stiffer at the outer layer (`DRAGON_SKIN_20A` vs row 20's `ECOFLEX_00_30`), so the penalty-active Newton residual takes more iterations to drop below `1e-10`.

### 5. `material_routing`

| Gate | Bound |
|---|---|
| `mesh.materials()[t]` per tet | `(μ, λ)` bit-equal to the shell's F4 table entry (`ECOFLEX_00_20` / `DRAGON_SKIN_10A` / `DRAGON_SKIN_20A` by distance-from-scan bin), each shell exercised ≥ 1 tet |

The scene's multi-material routing invariant — verifies that the `MaterialField` assigned every tet the `(μ, λ)` of the shell its centroid falls in. Reads the real per-tet `NeoHookean` from `mesh.materials()` via the public `.mu()` / `.lambda()` accessors and compares to the shell's F4 table entry with an exact `to_bits()` `==` (both come from the same const through the same `to_neo_hookean()` const fn, so a correctly routed tet is bit-identical — this is a routing check, NOT a constitutive-arithmetic mirror). A boundary-convention drift between `partition_point(|&t| t <= phi)` and `shell_at_phi` would fire it immediately. The constitutive math and the `to_neo_hookean()` Lamé round-trip are lib-owned (`neo_hookean.rs` tests + `silicone_table.rs::tests::to_neo_hookean_round_trips_lame_pair`); the generic routing mechanism is lib-owned (`sdf_material_tagging.rs` IV-4).

### 6. `strain_energy_ordering`

| Gate | Bound |
|---|---|
| `Ψ̄_inner > Ψ̄_middle` | strict |
| `Ψ̄_middle > Ψ̄_outer` | strict |

The row's headline mechanical readout. Three effects compound in the same direction: (a) compliance — inner is softest (μ = 18 kPa), outer is stiffest (μ = 113 kPa); under the same probe-driven displacement field (continuous through the bonded multi-material body), strain concentrates in the softer layers; (b) distance to load — inner is at the cavity wall directly contacting the probe; the strain field decays radially outward; (c) distance to constraint — outer is pinned at the Dirichlet band on the outer envelope; the displacement field is forced to zero at the boundary, so outer-layer F stays close to I and Ψ stays small. The strict ordering is robust to small drift in the per-tet F arithmetic. (Observed: `Ψ̄_inner ≈ 9.21`, `Ψ̄_middle ≈ 1.53`, `Ψ̄_outer ≈ 0.34 J/m³`; the outer-layer peak `~176 J/m³` is emitted to the JSON readout for inspection.)

### 7. `contact_engaged` + `peak_displacement_bounded`

| Gate | Bound |
|---|---|
| `n_active_pairs` | `> 0` (the static overlap pose actually engaged the probe against the cavity wall; the exact count was a mesher/discretization artifact) |
| `max disp` over active-contact-pair vertices | `< WRAP_THICKNESS = 0.014 m` (strict, geometric sanity — the wrap should not collapse to zero thickness; observed peak ~1.97e-3 m, ~14 % of the bound) |

`contact_engaged` guards that the fit pose is non-trivial (at least one active referenced-vertex contact pair). The displacement-bounded gate is a hard geometric upper bound: the cavity wall reaching the outer envelope's Dirichlet pin band would mean the wrap collapsed. (Note: penalty contact's elastic equilibrium can push the cavity wall *farther* than the rigid penetration depth — that's expected behaviour, not a violation.)

## Visuals

`out/sleeve_zslab.ply` — z-slab per-tet centroid cloud (`2_092` centroids in `|rest_centroid.z| < CELL_SIZE / 2 = 0.002 m`) with `DISPLACEMENT_SCALE = 50.0` geometric amplification on positions (`amplified = rest_centroid + SCALE * (deformed_centroid - rest_centroid)`). Same z-slab pattern as rows 11+16+20, with two scalars: categorical `material_id` (0 = inner / 1 = middle / 2 = outer) + sequential `displacement_magnitude` (true physical magnitude, unscaled).

Open in cf-view, the workspace's unified visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve/out/sleeve_zslab.ply
```

cf-view auto-picks the colormap per pattern (u) banked at row 15:

- **`material_id`** is binary-categorical (3 distinct values 0/1/2) → cf-view picks the **categorical palette** (rows 11+16+20 precedent for shell-id readouts).
- **`displacement_magnitude`** is unipolar continuous → cf-view picks **sequential viridis**.

The slab projects centroids onto a 2-D annulus on `z = 0`. Three concentric bands of categorical `material_id` (blue inner / orange middle / green outer) overlap with a continuous `displacement_magnitude` distribution.

**`displacement_magnitude` reads as a propagated 3-D shell mode, NOT a radial decay** — both the outer-layer band (Dirichlet-pinned at the outer envelope) AND the inner-cavity-wall band (40 mm below the probe contact zone, no direct load at `z = 0`) read as low-displacement (dark purple), with the peak displacement zone in the middle of the wrap cross-section thickness. The slab is far from the contact zone, so the absolute displacement values here are small (cf-view's auto-range normalises to the slab's local maximum, not the body's global peak); what the slab catches is the SECONDARY response of the wrap shell to the propagated load from the contact zone above. The peak-at-mid-thickness pattern is consistent with a 3-D shell bending mode where the inner-cavity-wall material is constrained from moving inward (the cavity is empty but adjacent material has nowhere to go laterally) and the outer envelope is Dirichlet-fixed, leaving the cross-section interior as the path of least resistance for the propagated displacement field. The cf-view image alone doesn't pin the exact mechanism — it pins the OBSERVED pattern; pattern (gg) bottom-up mechanical re-derivation lands "peak at mid-thickness, anisotropic in xy" as the fact, with the precise driving mechanism (axial-vs-lateral-vs-shell-bending decomposition) deferred to an explicit displacement-vector field readout in v2 if it becomes load-bearing for the relative-comparison story.

This is distinct from the `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` strain-energy ordering — strain energy density is highest in the inner layer when summed across the WHOLE body (softest material absorbs the most strain, dominated by the contact-zone tets at `z ≈ 0.040 m`), but the displacement-magnitude readout at the `z = 0` slab specifically catches the propagated response, not the contact-zone signal directly.

**Visible xy-anisotropy** — the cuboid has `SCAN_HX = 20 mm > SCAN_HY = 15 mm`, so the ±y wrap faces (40 mm long in x) are LONGER than the ±x wrap faces (30 mm long in y); under the same propagated load, the longer face panels deflect more, and the displacement field shows brighter bands along the ±y faces and dimmer regions along the ±x faces. The wrap THICKNESS itself is uniform (14 mm everywhere) by the `cuboid.offset` semantics; the asymmetry is in face-panel dimensions, not wrap thickness.

**Cross-readout caveat** — the cavity-wall mean / max displacement readouts (`~1.24 mm` / `~1.97 mm`, emitted to the JSON) are statistics over the **referenced-filtered active-contact-pair vertex set** at `z ≈ 0.040 m` (the probe contact zone near the scan's `+z` cap), NOT over the z=0 slab visible in cf-view. The slab artifact and the cavity-wall stats describe different regions of the body (equatorial slab vs probe-side cap); they are complementary readouts of the same fit-pose simulation, not redundant. The slab's local `displacement_magnitude` range is automatically normalized by cf-view, so the bright zones in the z=0 image are LOCAL maxima of the propagated flexural field, not the absolute peak (which lives at the cap region not present in the z=0 slab).

**Why z-slab over the full-boundary-surface artifact (row 3 sphere precedent)**: per pattern (aa) banked at row 16 N+3, hollow / interior-cavity / partial-occlusion bodies' full boundary surfaces 360°-occlude the cavity and the inner/middle interfaces from every cf-view orbit angle (cf-view exposes no section-cut UI). The z-slab projects centroids onto a 2-D annulus cut, exposing both the radial material-shell partition and the cavity-wall displacement response under probe intrusion. Row 21's geometry is doubly hollow (scan-shaped cavity AND three concentric shells) → z-slab is required by construction; row 16's N+3 pivot is the precedent and rows 20 + 21 ride the precedent.

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is required for the FEM solve at this mesh resolution (`74_628` tets through faer's sparse Cholesky + 9 Newton iters with penalty-contact); debug mode would take many minutes for what runs in seconds release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness — coarsening further would erase the middle and outer layers, and finer cells (e.g. `0.002 m`) push the per-cell penalty gradient too high under the static-overlap pose, inverting tets in the first Newton step.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Roadmap (followups, not in v1 scope)

This row is v1 of a queued evolution toward iter-2+ silicone-device design support:

- **v2** — multi-step force-displacement curve (5-10 step ramp + JSON sidecar). Replaces the single static fit pose with a quasi-static intrusion sweep.
- **v3** — axial zoned variation (proximal/mid/distal stiffness modifier composed onto the radial `LayeredScalarField`). Needs a custom `Field<f64>` impl OR a `BlendedScalarField` composition over a longitudinal SDF.
- **v4** — explicit thin copper-mesh sub-layer (4-shell `LayeredScalarField`; ~0.5 mm mesh-band at much higher Shore between Ecoflex and DS10A).
- **vN** — real anatomy scan replacing the cuboid fixture (`flood_filled_sdf(scan_indexed_mesh, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT)?.0` lifted via PR3 F2 `impl<D, S> Sdf for Signed<D, S>`, then `Solid::from_sdf` per F5 — exactly row 20's path).
