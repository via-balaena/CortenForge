# scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth

**Row 25 — F1.6 of the sim-soft viz arc; an open-mouth + cuboid-plug fork of [row 24 `scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp`](../scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/).** Same Yeoh constitutive model, axial zoning, 3-layer radial material stack, and BCC + IS pipeline as row 24. Two differentiators:

1. **Wrap geometry** — the cavity used for the boolean subtraction is extended on +z by `MOUTH_EXTENSION_PLUS_Z = WRAP_THICKNESS + 0.001 m` so the cavity pokes through the outer envelope's +z face, leaving the wrap with an open mouth on +z. The +z face becomes a `WRAP_THICKNESS`-wide rim around a `2*SCAN_HX × 2*SCAN_HY` rectangular opening — the production-target geometry of an insertion-cavity device.
2. **Load case** — `Solid::cuboid().offset(PROBE_PLUG_CORNER_RADIUS)` plug with xy interference of `PROBE_INTERFERENCE = 0.1 mm` descends through the open mouth into the cavity, contacting the cavity walls along its descent. Replaces row 24's spherical-probe-into-closed-+z-face with the insertion-into-cavity scenario the open-mouth geometry demands.

Ramp depth is capped at **4 mm in 8 × 0.5 mm steps** (vs row 24's 8 mm in 16 steps); deeper penetration trips the soft-body solver's max-stretch-deviation validity bound at the corner-of-plug × cavity-wall stress concentrations. F1.7+ work item to extend depth via finer mesh near the rim corners or a more aggressive plug chamfer.

[arcmemo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_yeoh_hyperelastic_arc.md
[vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md
[mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Why this row exists (F1.6 viz arc payload)

The F1 viz arc shipped tet-mesh-native primitives [`sim_soft::viz::boundary_surface`] + [`sim_soft::viz::slab_cut`] (see [vizarc memo][vizarc]) and retrofitted rows 20/22/23/24 to consume them. F1's central architectural claim is "tet-mesh-native primitives generalize to any geometry sim-soft can mesh, no per-geometry tuning." Rows 20/22/23/24 all have **closed-body topology** — the boundary surface is a single closed manifold (sphere or cuboid wrap, possibly with an enclosed cavity that has its own closed surface).

Row 25 validates the architecture's generality on **open-boundary topology**: the body is a cup-with-open-mouth, and the boundary mesh now includes the cavity-wall surface joining the outer envelope at the rim — all in one connected boundary mesh, no tagged surfaces or per-row plumbing. The F1 primitives consume this open-boundary mesh with no API changes.

This row also serves as the production-target geometry for the [layered silicone device][mem] C1 target — the user's actual device is a poured-silicone cup that an object inserts into, not a closed bottle.

## Material stack (inherited from row 24)

Six-anchor stack — Path-1 F4 const entries, soft-tip / stiff-anchor pattern within each shell:

| Shell | Proximal anchor (+z, contact end) | Distal anchor (−z, anchor end) | μ contrast |
|---|---|---|---|
| Inner (φ < 6 mm) | `ECOFLEX_00_20` (μ = 18 kPa, C₂ = 1.69 kPa) | `ECOFLEX_00_30` (μ = 23 kPa, C₂ = 2.05 kPa) | ×1.28 |
| Middle (6 mm ≤ φ < 10 mm) | `DRAGON_SKIN_10A` (μ = 51 kPa, C₂ = 4.46 kPa) | `DRAGON_SKIN_15` (μ = 92 kPa, C₂ = 8.20 kPa) | ×1.80 |
| Outer (φ ≥ 10 mm) | `DRAGON_SKIN_20A` (μ = 113 kPa, C₂ = 10.0 kPa) | `DRAGON_SKIN_30A` (μ = 198 kPa, C₂ = 17.6 kPa) | ×1.75 |

Per [arc memo §D3][arcmemo] the proximal/distal pair within each shell stays within the same Smooth-On family (Ecoflex inner, Dragon Skin middle + outer) so the smoothstep weight in the band interpolates linearly through the F4 Shore-space convention. Three independent `BlendedScalarField`s share the same axial SDF + band, so the band-zone Yeoh sample is a coherent `(μ_blend, C₂_blend, λ_blend)` triple.

## Load case — cuboid plug into open mouth

```rust,ignore
const PROBE_INTERFERENCE: f64 = 0.000_1;            // 0.1 mm xy interference
const PROBE_PLUG_HX: f64 = SCAN_HX + PROBE_INTERFERENCE;
const PROBE_PLUG_HY: f64 = SCAN_HY + PROBE_INTERFERENCE;
const PROBE_PLUG_HZ: f64 = 0.020;
const PROBE_PLUG_CORNER_RADIUS: f64 = 0.001;        // 1 mm rounded corners

fn build_probe_solid_at_depth(depth: f64) -> Solid {
    let plug_center_z = PROBE_PLUG_INITIAL_CENTER_Z - depth;
    let r = PROBE_PLUG_CORNER_RADIUS;
    Solid::cuboid(Vector3::new(PROBE_PLUG_HX - r, PROBE_PLUG_HY - r, PROBE_PLUG_HZ - r))
        .offset(r)
        .translate(Vector3::new(0.0, 0.0, plug_center_z))
}
```

The plug starts at depth=0 with its bottom exactly at the rim plane (`z = SCAN_HZ + WRAP_THICKNESS = 0.054 m`), then descends 0.5 mm per step over 8 steps to a final 4 mm penetration. The xy interference engages immediately as the plug's outer faces overlap the cavity walls, generating contact along the descent.

**Sharp-corner cuboid SDFs have C^0 gradient discontinuities** at vertices and edges, which makes penalty contact ill-conditioned (tangent stiffness becomes non-SPD near corners and Newton stalls in line search). The `.offset(r)` chamfer rounds the plug's corners by 1 mm, smoothing the SDF gradient and stabilising contact. This is a contact-physics regularisation, not a geometric approximation — production-scale fabrication tolerances would also produce rounded edges.

## Ramp behavior

Run `cargo run --release`. The `print_summary` table at run end shows per-step `force_z` / `max_disp` / `iter_count` / `n_active_pairs`:

```text
step | depth(mm) | iter | force_z(N) | max_disp(m)
-----|-----------|------|------------|-------------
   1 | 0.500     |    0 | -8.60e1    | 0.000e0
   2 | 1.000     |    0 | -1.95e2    | 0.000e0
   3 | 1.500     |    0 | -2.59e2    | 0.000e0
   4 | 2.000     |    0 | -1.56e2    | 0.000e0
   5 | 2.500     |    8 | -1.59e2    | 2.49e-4
   6 | 3.000     |   12 | -1.65e1    | 6.59e-4
   7 | 3.500     |   15 | -2.48e1    | 1.04e-3
   8 | 4.000     |   22 | -2.86e1    | 1.38e-3
```

`force_z` is **negative** (plug pushes -z on the wrap material in the xy-interference band; `force_on_soft.z` points -z) — opposite of row 24's POSITIVE +z reaction (where the sphere pushed -z on the +z face from above). The early steps trivially converge with `iter=0` because the rest-state penalty residual is below `tol = 1e-10` already at penalty-band threshold; once the plug descends past the rim into the cavity walls (steps 5+), Newton iterates as the material deforms.

## Sanitization

Per the [device memo][mem]'s sanitization directive — the scanned reference geometry is referred to as "scanned reference geometry" or "scan stand-in" throughout this crate's prose. No anatomical references appear in any tracked surface. The cuboid placeholder is a parametric synthetic stand-in: the pipeline demonstration is the workflow, not the cuboid's specific geometry. Production runs swap the cuboid for a real scan via the `mesh_sdf::SignedDistanceField` → `cf_design::Solid::from_sdf` bridge with no other code change.

## Classification — a `demo`, not a validator

This row is a **`demo`** (`example_kind = "demo"`): CI compile-checks it but never runs it, and it **asserts nothing**. That's the honest classification for its load case:

- Its ramp-physics oracles are inherently **inapplicable**. The interference-fit cuboid plug **inverts the force sign** (`force_z` is NEGATIVE — the plug pushes down on the wrap material in the xy-interference band) and **inverts the strain ordering** (`Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` fails, because stress concentrates at the outer-shell-adjacent wrap material in the interference band, not the inner cavity wall). So the monotonicity + ordering gates that make rows 22/23/24 validators cannot hold here.
- The first ~4 ramp steps are **degenerate** — `iter=0` with zero deformation until the plug engages the cavity wall at ~2.5 mm — so a per-step convergence gate would be near-vacuous over half the ramp.
- Its real payload is the **open-boundary-topology viz** (see Visuals) on the production-target open-mouth geometry.

Constitutive / mesher / blend correctness is covered by the sibling validator rows (21/22/23/24) + the sim-soft lib tests (`yeoh_contract.rs`, `silicone_table.rs`, `sdf_material_tagging.rs`, `blended_material_composition.rs`). The demo DOES route the calibrated Yeoh validity bounds (`from_yeoh_fields_with_bounds`), so a clean `cargo run --release` reaches 4 mm without tripping the fail-closed validity gate.

## Visuals

PLY artifacts via the F1/F2 public viz API:

- **`out/sleeve_boundary_final.ply`** — full 3D body via [`sim_soft::viz::boundary_surface`]. Per-vertex `psi_j_per_m3` from volume-weighted averaging over per-tet psi. Includes the cavity-wall surface joining the outer envelope at the rim — open-mouth topology in a single connected boundary mesh.
- **`out/sleeve_slab_cut_x0_final.ply`** — cross-section at `x = 0` via [`sim_soft::viz::slab_cut`]. Marching-tetrahedra cut, per-vertex psi linearly interpolated along cross-edges. Reads as a "U" shape (vs row 24's closed "O") — directly visualises the cup-with-open-top geometry.
- **`out/sleeve_design_slab_cut_x0_final.ply`** + **`out/sleeve_design_surface_final.ply`** — design-mesh siblings via [`sim_soft::viz::design_slab_cut`] + [`sim_soft::viz::design_surface`]. Marching-cubes on the design SDF; per-display-vertex scalars by barycentric transfer from the analysis tet mesh.
- **`out/scene_steps/scene_step_NN.ply`** (NN = 01..=08) — per-step ramp animation via [`sim_soft::viz::design_scene_deformed`]. Each step merges the deformed body (amplify=10) with the rigid cuboid plug at its current depth into one mesh, tagged via a categorical `primitive_id` extra (`0` = body, `1` = plug). Steps 1–4 show the plug descending pre-engagement (body at rest, `iter_count=0`); steps 5–8 show the cup walls deforming inward as the plug contacts cavity walls. **Caveat**: the closed outer envelope is Dirichlet-pinned, so the body's *outer* silhouette is identical across all 8 frames — the deformation lives on the cavity walls inside. See the slab-cut companion below.
- **`out/slab_steps/slab_step_NN.ply`** (NN = 01..=08) — per-step deformed cross-section at `x = 0` via [`sim_soft::viz::slab_cut_deformed`]. F1 analysis-mesh marching-tet through the deformed tet positions at `amplify=10`. The "U" shape's inner cavity walls visibly bulge inward across frames 5–8 — this is where the squish is actually visible.

Each ramp series lives in its own subdirectory so cf-view's S1 D1 sequence player (which lex-sorts every `*_step_<digits>.ply` in the input directory into one scrub) shows exactly one series per launch:

```sh
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/scene_steps
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/slab_steps
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_boundary_final.ply
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_slab_cut_x0_final.ply
```

cf-view auto-picks sequential viridis for `psi_j_per_m3` and toggles to the categorical `primitive_id` scalar to read the body-vs-plug split on the per-step scenes. Use `←` `→` `Home` `End` to scrub.

**Force-displacement curve via matplotlib.** The `ramp_curve` array carries the per-step force / displacement / Ψ̄ trace. Optional matplotlib post-processing via PEP 723 inline metadata:

```sh
uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/plot_ramp.py
```

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode required. 8-step total runtime ~10-30 s release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at least one BCC cell across thickness; finer cells (e.g., `0.002 m`) trip an SPD pivot at the FIRST ramp step (empirically tested at row-22 v2-spec spike time, applies to rows 24/25 by inheritance).

## Roadmap

- **F1.7 follow-up** — re-derive row-25-specific load-case verify gates (force trajectory, ψ̄ ordering under interference-fit loading) once the contact-onset transient is mechanistically understood. Most likely a separate gate set keyed on the cuboid-plug load case rather than reusing row 24's shape.
- **F2 (viz arc continuation, banked at [vizarc memo][vizarc] §F2)** — decouple display mesh from sim mesh. Display surfaces from the design SDF via marching cubes + scalar transfer via barycentric interpolation. Fixes the boundary-surface depth-perception limitation of cf-view's flat shading.
- **Deeper-than-4mm penetration** — refine mesh near rim corners (CELL_SIZE local refinement) or chamfer plug edges more aggressively. Currently capped by max-stretch-deviation validity bound trip ~4.5 mm.
- **vN — real scan replacing the cuboid stand-in** (`mesh_sdf::SignedDistanceField::new(scan_indexed_mesh)` lifted via PR3 F2 `impl Sdf for SignedDistanceField`, then `Solid::from_sdf` per F5 — exactly row 20's path).
