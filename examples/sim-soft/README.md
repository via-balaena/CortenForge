# sim-soft Examples

Focused, single-concept demonstrations of the `sim-soft` soft-body
FEM crate (`sim/L0/soft/`) — SDF primitives + meshing, hyperelastic
constitutive laws, multi-element + multi-material assembly, penalty
contact, and the cf-design / mesh-sdf bridges that make the
layered silicone device cavity-fit workflow possible.

Each example writes static artifacts (PLY + JSON); Tier 4
contact-tier examples additionally render live in Bevy under
`CF_VISUAL=1` — see "Visualization convention" below. Per-example
READMEs document the locked numerical anchors (asserted by
`cargo run --release` exit-0). The
arc-level inventory at
[`sim/L0/soft/EXAMPLE_INVENTORY.md`](../../sim/L0/soft/EXAMPLE_INVENTORY.md)
covers strategy, ordering, and PR sequencing; this directory is the
executable proof.

## Examples by capability tier

Every example shipped per-commit is registered below. The
[inventory file](../../sim/L0/soft/EXAMPLE_INVENTORY.md) is the
source of truth for the row's tier assignment, capability shown,
and PR sequencing rationale.

### Tier 1 — SDF primitives + meshing (Phase 3 surface)

| Example | Concept |
|---------|---------|
| [`sdf/stress-test`](sdf/stress-test/) | The sim-soft SDF surface as one validation superset (rows 1–3 folded — one domain → one stress-test), three modules: **`sphere_eval`** — the `Sdf` trait contract on `SphereSdf` (analytic signed distance + unit gradient, documented `Vec3::z()` origin fallback, 11³ = 1331 grid sweep, FD-Eikonal diagnostic); **`hollow_shell`** — sharp-CSG `SphereSdf{1.0} \ SphereSdf{0.5}` via `DifferenceSdf` (book Part 7 §00 §01), z = 0 slice (49² = 2401 verts) with `signed_distance` + `active_branch` (branch-flip circle at `\|p\| = 0.75`); **`sdf_to_tet`** — `SdfMeshedTetMesh::from_sdf` BCC + Labelle-Shewchuk Isosurface Stuffing at the III-1 scene (`R = 0.1 m`, `cell_size = 0.02 m`, 6768 tets), closed boundary surface (1224 faces, Euler χ = 2) with bimodal `boundary_residual`. |
| [`single-tet-stretch`](single-tet-stretch/) | The arc's first FEM-running example. `SkeletonSolver::step` (backward-Euler Newton with `NeoHookean` on a hand-built 1-tet mesh, no contact) on `SoftScene::one_tet_cube` — canonical decimeter-edge tet with `θ = 10 N` along `+ẑ` on `v_3` and `v_0..v_2` Dirichlet-pinned; converges in 3 iters to `dz ≈ 9.69e-4 m` (~1 % strain) with all 12 DOFs of `x_final` matching the IV-1 frozen-reference bit pattern. JSON-only force-stretch trace (single-tet topology is too trivial for a render). |

### Tier 2 — Constitutive + multi-element (Phase 1–2 surface)

| Example | Concept |
|---------|---------|
| [`neo-hookean-uniaxial`](neo-hookean-uniaxial/) | Direct-eval Neo-Hookean stress-strain curve across a 12-point traction-free uniaxial sweep on a single Tet4 (`F = diag(λ, λ_t, λ_t)` with `λ_t` from a 1-D inner Newton on the traction-free transcendental); analytic NH `P_11`, `P_22`, ψ vs `Material::first_piola` / `Material::energy` at rel-tol 1e-12, plus a `ValidityDomain` declaration check + per-point in-domain `max\|σ−1\|` gate; 48 captured-bit self-pins (4 quantities × 12 points) under the IV-1 dense bit-equal tier. JSON output + optional `uv run plot.py` matplotlib post-hoc 2×2 panel. |
| [`multi-element-stretch`](multi-element-stretch/) | Phase 2 multi-element FEM assembly under uniform Dirichlet stretch on a 27-vertex / 48-tet hex grid (`HandBuiltTetMesh::uniform_block(2)`) at `λ = 1.20`, with one interior vertex left free so Newton has actual work to do (3 iters, residual ~1e-14 N). Per-tet `F` evaluates to `diag(λ, 1, 1)` for every one of the 48 tets (uniformity spread = `0.0` Pa on the capture platform); `P_11`, `P_22`, ψ match closed forms at rel-tol 1e-12. Quasi-static via `cfg.density = 0`. JSON-only per-tet uniformity trace. |

### Tier 3 — Multi-material spatial fields (Phase 4 surface)

| Example | Concept |
|---------|---------|
| [`layered-scalar-field`](layered-scalar-field/) | `LayeredScalarField` over a 3-shell concentric `SphereSdf` partition; per-tet `(μ, λ)` sampled at the centroid via `MaterialField::from_fields` and cached in `Mesh::materials()` — first Tier 3 example, first cf-view consumer in PR1's later half. Z-slab per-tet centroid PLY (648 of 6768 tets in `\|z\| < cell_size/2`) with categorical `material_layer_id ∈ {0, 1, 2}` reading as three concentric color rings on the z=0 plane (cf-view's tab10 categorical heuristic fires on the integer-valued + < 16-unique-values distribution). |
| [`blended-scalar-field`](blended-scalar-field/) | `BlendedScalarField` cubic-Hermite-smoothstep transition between two uniform Lamé regions ("stiff skin over soft core" idiom), with the same SDF threaded through `MaterialField::with_interface_sdf` to populate `Mesh::interface_flags` per the IV-6 `\|φ(x_c)\| < L_e` rule (the headline new feature vs row 8). Three per-vertex scalars on the z-slab PLY: categorical `interface_flag` (the IV-6 flagged band), continuous `material_mu` (the physical readout), continuous `smoothstep_weight` (the kernel mechanism). |
| [`bonded-bilayer-beam`](bonded-bilayer-beam/) | Phase 4 IV-3 bonded multi-material cantilever beam under tip load (`(0.5, 0.1, 0.1) m`, `(NX, NY, NZ) = (20, 8, 8)` → 1701 verts / 7680 tets); inline `HalfSpaceField` partitions tets into region A (`1×` Ecoflex) below the interface plane and region B (`2×` Decision J composite) above. First Tier 3 solver-driven scene + first IV-2 shared-vertex displacement-continuity gate at production-scale. Tip displacement matches Euler-Bernoulli composite-beam analytic within 30 % at h/2 (Tet4 sub-`O(h²)` convergence per IV-3); strict-between-uniform-bounds inequality on tip displacement as the IV-2-lens-β discriminator. Y-slab axial-mid-plane PLY with `displacement_z` + `material_id` scalars + `DISPLACEMENT_SCALE = 20×` viz amplifier on the rendered positions only. |
| [`concentric-lame-shells`](concentric-lame-shells/) | PR1 finale — the IV-5 three-shell concentric hollow silicone sphere meshed via BCC + Labelle-Shewchuk on a `DifferenceSdf` of two `SphereSdf`s (`R_OUTER = 0.10 m`, `R_CAVITY = 0.04 m`, 6456 tets); 3-shell `MaterialField` per Decision J's `1× / 2× / 1×` symmetry (inner Ecoflex / middle composite / outer Ecoflex); per-vertex radially-outward pressure traction (`f_v = pressure · n̂_v · A_v`, `LoadAxis::FullVector`) on the cavity surface, fixed Dirichlet pin on the outer surface. Four Saint-Venant-averaged radial-displacement readouts (cavity-wall + per-shell) match piecewise-Lamé closed-form within 30 % at h/2; IV-2-lens-β strict-between-uniform-bounds gate runs three full solver passes (three-shell + uniform-1× + uniform-2×). Z-slab equatorial PLY with `material_id` + `radial_displacement` scalars + `DISPLACEMENT_SCALE = 50×` viz amplifier. |

### Tier 4 — Penalty contact (Phase 5 surface)

| Example | Concept |
|---------|---------|
| [`soft-drop-on-plane`](soft-drop-on-plane/) | Drop-and-rest user-facing wrap — first PR2 example row + first user-facing demo of `sim-bevy-soft` (under `CF_VISUAL=1`). Soft sphere (`R = 1 cm`) released at `RELEASE_HEIGHT = 5 cm` falls under gravity onto a `RigidPlane(+ẑ, offset = 0)`; one-way `PenaltyRigidContact` (κ = 1e4 N/m, d̂ = 1 mm) damps penalty oscillation `~1300×` per step at backward-Euler `dt = 1 ms`, settling within 1000 steps. New quantitative gate vs the drop-and-rest fixture: COM-at-equilibrium height computed over `referenced_vertices` only (corrects the orphan-bias the fixture explicitly omits). PLY: 648-triangle deformed boundary mesh (no per-vertex scalars at quiescence). Bevy: 1000-frame trajectory replay at `10×` slow-mo + `RENDER_SCALE = 100×` (Bevy's default near plane is tuned for human-scale). |
| [`hertz-sphere-plane`](hertz-sphere-plane/) | Hertzian sphere ↔ plane analytical-validation gate — second PR2 example row. Soft sphere quasi-statically pressed against a `RigidPlane` by `F = 500 mN` axial; three refinement levels (`h = 3 / 1.5 / 0.75 mm`); contact-patch radius `a_FEM` matches the Hertz analytic within 20 % at h/4 with monotonic + Cauchy convergence (`a/R ≈ 0.18` ≪ small-strain limit). Fixture-local `κ = 1e3 N/m` (10× softer than default) lifts the multi-vertex-engagement threshold over the row's `cell_size`. **`δ_FEM` is structurally unreachable in the penalty regime** (penalty's d̂-band caps elastic compliance — Phase H IPC's logarithmic barrier `−κ log(d/d̂)` recovers the rigid limit), so only `a_FEM` is asserted. PLY with per-vertex `contact_force_z`, three-section JSON (scalars + per-active-vertex + 200-point Hertz analytic for matplotlib overlay), Bevy with concentric coral `a_FEM` / white `a_Hertz` annulus rings on the rigid plane reading the `rel_err_a` gap visually. |
| [`compressive-block`](compressive-block/) | Bilateral-compression `PenaltyRigidContact` user-facing wrap — third PR2 example row. Soft cube (`L = 1 cm`) compressed by a descending rigid plane against a bottom-face Dirichlet pin (mixed BC: bottom full-pin + sides free + top z-contact); three refinements (`n ∈ {2, 4, 8}` → `48 / 384 / 3072` tets); fixture-local `(d̂, δ) = (10 μm, 50 μm)` override via `with_params` keeps the cold-start Newton step below the tet-inversion threshold. Headline gate is the two-bound bracket `F_us ≤ F_R_FEM ≤ F_strain` (mixed BC has no clean closed form; `F_us = E·A·ε` and `F_strain = M_c·A·ε` with `M_c = λ + 2μ` are the pure-BC limits); finest `F_R` sits at 7.6 % rel-pos in `[E, M_c]` (Saint-Venant boundary-layer regime — most cube interior is uniaxial-stress). Cauchy ratio `0.43`. PLY with per-vertex `contact_force_z`, JSON + matplotlib F-vs-ε scatter, Bevy with `VIZ_AMPLIFY = 50×` displacement amplifier + plates positioned flush against the amplified cube faces. |
| [`contact-force-readout`](contact-force-readout/) | Per-active-pair contact readout via the `PenaltyRigidContact::per_pair_readout` foundation patch — fourth PR2 example row. Inherits row 14's compressive-block scene + fixture-local `(d̂, δ)` override at single n=8 refinement. Headline accessor-vs-manual gate: `−Σ readouts.force_on_soft.z` bit-equivalent at rel-tol 1e-12 to the row-14-style inline manual reconstruction. The public surface decouples future rows (row 20's scan-derived rigid primitive) from per-shape closed-form code — any `impl Sdf` flows through. PLY with per-vertex `contact_pressure` (Pa, uniform per-pair area approximation `A_top / n_active`); JSON with per-active-pair `(v, x, y, sd, force_x/y/z, pressure)`; matplotlib top-down patch scatter showing the Saint-Venant boundary-layer pressure pattern (interior `~6×` higher than corners under mixed BC). |

### Tier 5 — Bridges + extensions (silicone-device path)

| Example | Concept |
|---------|---------|
| [`mesh-scan-as-solid`](mesh-scan-as-solid/) | `mesh_sdf::SignedDistanceField` satisfies `cf_design::Sdf` (PR3 F2). 12-tri programmatic cube fixture round-tripped through binary STL on disk; closed-form L∞-ball anchors at face / edge / vertex / interior probes via `&dyn cf_design::Sdf`; 17³ = 4913 bulk grid PLY with two scalars — `signed_distance` (analytical SDF) and `inside_raycast` (raycast inside-test, with documented HE-1 diagonal gaps). |
| [`solid-to-sim-soft`](solid-to-sim-soft/) | `cf_design::Solid` is a first-class SDF for `SdfMeshedTetMesh::from_sdf` via the F1+F3 bridge (PR3 row 16). A typed boolean-difference body composed via cf-design's CSG kernel (`Solid::sphere(R_OUTER).subtract(Solid::sphere(R_CAVITY))`) flows into sim-soft's BCC + Labelle-Shewchuk pipeline through one `&dyn cf_design::Sdf` coercion — companion to row 15's scan-SDF bridge. Geometry identical to row 11 (single-material variant, `MaterialField::skeleton_default = uniform(1e5, 4e5)`); HEADLINE A bit-exact bridge-equivalence anchor vs `DifferenceSdf<SphereSdf>` baseline (`equals_structurally` + per-vertex position match at `EXACT_TOL = 0.0`); cavity-wall mean cross-row continuity bit-equal to row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS`. Z-slab PLY with `radial_displacement` scalar (single-material variant, no `material_id`). |
| [`silicone-material-table`](silicone-material-table/) | Engineering-grade Smooth-On platinum-cure silicone Lamé pairs + density via PR3 F4's `silicone_table.rs` const module. Iterates 7 `pub const SiliconeMaterial` entries (`{Ecoflex 00-10/20/30/50, Dragon Skin 10A/20A/30A}`), dispatches each via `SiliconeMaterial::to_neo_hookean()` (`const` bridge into the `Material` trait surface), probes at `F = diag(2.0, 1, 1)` (the data-sheet `σ_100 = 100% engineering strain` anchor). 7 verify groups: `λ.to_bits() == (4·μ).to_bits()` ν=0.40 invariant + rest-config zero + closed-form `P_11` / `P_22` / ψ at rel-tol 1e-12 + transverse symmetry + hardness ordering + 21 captured-bit self-pins (3 quantities × 7 materials). JSON-only per `feedback_visual_pass_collapses_for_json_rows`. |

### Tier 6 — Synthesis (the engineering goal)

| Example | Concept |
|---------|---------|
| [`layered-silicone-device`](layered-silicone-device/) | The PR3 final synthesis row, closing the entire sim-soft examples arc. Composes every PR3 foundation piece (F1–F5) end-to-end: scan-derived rigid indenter (`mesh_sdf::SignedDistanceField`, F2) bridged into typed `cf_design::Solid` via `Solid::from_sdf` (F5 sugar over `user_fn`); 3-shell `MaterialField` from F4's `silicone_table` (outer + inner = `ECOFLEX_00_30`, middle = `DRAGON_SKIN_10A` as the conductive-composite proxy); BCC + Isosurface Stuffing tet pipeline through `SdfMeshedTetMesh::from_sdf` (F1 + F3); static fit pose with the same scan SDF as the rigid indenter (post-PR2 trait-unified `PenaltyRigidContact::new`, first non-plane consumer). 8 anchor groups including F4 const-fn `to_neo_hookean()` Lamé-pair round-trip + per-shell tet-count partition + IV-1 sparse-tier rel-tol on rest-state contact reaction force. Outputs `out/layered_silicone_device.json` (fit-pose scalars + 3-material provenance + per-active-pair detail) and post-F1.5 boundary-surface + slab-cut at z = 0 PLY emits via [`sim_soft::viz`] public API. Mesh refined to `CELL_SIZE = 0.01 m` post-F1.5 visual review (~10 lattice cells across the sphere radius for legible boundary surface). |
| [`scan-fit-3layer-sleeve`](scan-fit-3layer-sleeve/) | Tier 6 synthesis #2 — the manufacturing-target geometry directly: a 3-layer sleeve wrapping a cuboid scan stand-in (40 × 30 × 80 mm), exercised by a sphere probe at 1 mm static-overlap penetration. Engineering-grade silicone stack via F4 (`ECOFLEX_00_20` / `DRAGON_SKIN_10A` / `DRAGON_SKIN_20A` for inner / middle / outer at 18 / 51 / 113 kPa). First sim-soft example to demonstrate post-solve **per-tet strain energy density extraction** (`Ψ_t = Material::energy(F_t)` with `F_t` reconstructed inline from rest + deformed positions); per-layer Ψ̄ aggregates show strict-monotone `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` from compounded compliance + distance-to-load + distance-to-constraint. 9 anchor groups + 7 captured-bit self-pins. JSON + z-slab PLY. |
| [`scan-fit-3layer-sleeve-ramp`](scan-fit-3layer-sleeve-ramp/) | Tier 6 synthesis #3 — multi-step quasi-static intrusion ramp evolution of row 21. Same geometry + materials, but 12 chained `replay_step` calls at 0.5 mm/step from rest to 6 mm penetration, bounding each step's Newton iter-0 penalty gradient by per-step delta only. First sim-soft example to demonstrate `replay_step` chaining (`x_prev_k+1 = x_final_k`); ramp circumvents the single-step Newton-basin collapse that limits row 21 v1 to 1 mm. 12 anchor groups including per-step iter-count + force-displacement monotonicity (from step 2 onward) + per-step Ψ̄ ordering at every intermediate equilibrium + 60 captured-bit self-pins (5 quantities × 12 steps). JSON `ramp_curve` array + post-F1.5 boundary-surface + slab-cut at z = 0 PLY emits via [`sim_soft::viz`] public API + optional `plot_ramp.py` matplotlib post-processing. **Wall: Neo-Hookean validity domain trips at `max_disp ≈ 7 mm`** (Phase 4 Decision Q fail-closed; per-tet stretch reaches `[2.06, 1.22, 0.073]` past NH's calibrated range). 8 mm user-target unblocked by row 23's Yeoh consumer below (Mooney-Rivlin was math-falsified on Smooth-On TDS data, pivoted to 2-param Yeoh). |
| [`scan-fit-3layer-sleeve-yeoh-ramp`](scan-fit-3layer-sleeve-yeoh-ramp/) | Tier 6 synthesis #4 — F4.1 Yeoh consumer of the F4.0 generic-mesh refactor. Same scan + 3-layer sleeve + sphere probe geometry as rows 21 + 22; constitutive model swapped from Neo-Hookean to **Yeoh** (additive `C₂(I₁−3)²` extension over NH's deviatoric kernel, same NH-style compressibility); ramp extended to **16 × 0.5 mm = 8 mm**. First user of `SdfMeshedTetMesh<Yeoh>::from_sdf_yeoh` + `MaterialField::from_yeoh_fields` + `PenaltyRigidContactYeohSolver`. Material stack demonstrates BOTH F4 constructor paths: inner = Path 2 (`from_effective_shore(DoubleZero(20.0))?.to_yeoh()`, parametric API); middle/outer = Path 1 (direct `DRAGON_SKIN_*.to_yeoh()`). **Yeoh reaches 8 mm cleanly in 77 Newton iters at step 16** (cap 150, 73-iter margin; residual 9.87e-11) — `C₂(I₁−3)²` stiffening engages at the high-strain regime where NH would have refused. Force at 8 mm = 49.3 N vs row 22's 25.6 N at 6 mm. 12 anchor groups + 80 captured-bit self-pins (5 quantities × 16 steps). JSON + post-F1.5 boundary-surface + slab-cut at z = 0 PLY emits via [`sim_soft::viz`] public API + `plot_ramp.py`. |
| [`scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp`](scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/) | Tier 6 synthesis #5 — v3 axial-zoned variation. Same geometry + ramp depth as row 23; ONLY differentiator is axial zoning of the material field. First sim-soft user of `BlendedScalarField` over an axial half-space SDF composed onto `LayeredScalarField` — 2-zone axial × 3-shell radial = 6 anchor cells with one-Shore-step contrast per family (Ecoflex 00-20 → 00-30 inner, DS10A → DS15 middle, DS20A → DS30A outer). Soft-tip (+z, contact end) / stiff-anchor (−z) pattern matches the soft-pneumatic-actuator design convention. Per-parameter independent composition (one `BlendedScalarField` each for μ, C₂, λ) sharing the same axial SDF + band → blend-zone Yeoh sample is a coherent triple. **No new `Field<f64>` impl required**; only an inline 5-line `Sdf`-impl `AxialHalfSpace`. 16 anchor groups + 89 captured-bit self-pins (80 row-23-shape + 9 NEW final-step zone × shell mean Ψ̄). Post-F1.5 viz retrofit: emits boundary-surface PLY + slab-cut at x = 0 PLY via [`sim_soft::viz`] public API (replaces the pre-F1.5 x-slab centroid cloud emit). |
| [`scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth`](scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/) | Tier 6 synthesis #6 (F1.6 viz arc payload) — open-mouth fork of row 24. Cavity used for boolean subtraction extended on +z so it pokes through the outer envelope, leaving the wrap with an open mouth on +z (cup-with-open-top — production-target geometry of an insertion-cavity device). Load case replaces row 24's spherical probe with a chamfered cuboid plug (`Solid::cuboid().offset(1 mm)`) at xy interference of 0.1 mm descending through the open mouth. Validates that the F1 public viz primitives (`boundary_surface`, `slab_cut`) handle open-boundary topology — the boundary mesh now includes the cavity-wall surface joining the outer envelope at the rim, all in one connected mesh, no per-row plumbing. Ramp depth capped at 4 mm (vs row 24's 8 mm) — deeper trips the soft-body solver's max-stretch-deviation validity bound at corner-of-plug stress concentrations. Many of row 24's load-case-dependent verify gates (force-displacement monotone, ψ̄ ordering, captured-bits) DISABLED at row 25 because sign convention inverts (force_z is NEGATIVE for plug-on-cavity-wall) and ordering inverts under interference-fit loading; gate fns + captured-bits constants retained under `#![allow(dead_code)]` for F1.7+ row-25-specific re-derivation. |

## Visualization convention

Examples split by tier per
[`EXAMPLE_INVENTORY.md`](../../sim/L0/soft/EXAMPLE_INVENTORY.md)
§Visualization convention:

- **PLY + JSON** for static math-pass-first artifacts (Tiers 1, 2,
  3, 5, 6). Open in [`cf-view`](../../cf-viewer/) — the workspace's
  unified Bevy-CLI viewer with auto-discovered per-vertex scalars
  and auto-colormap detection. Per-example READMEs call out the
  scalar to pre-select and the canonical visual. Some rows
  (`single-tet-stretch` in Tier 1; `neo-hookean-uniaxial` and
  `multi-element-stretch` in Tier 2; `silicone-material-table` in
  Tier 5) ship JSON-only — the table or curve IS the artifact, no
  spatial render is meaningful (per
  [`feedback_visual_pass_collapses_for_json_rows`](../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_pass_collapses_for_json_rows.md)).
- **Bevy `CF_VISUAL=1` opt-in** for contact dynamics (Tier 4).
  Headless asserts + PLY emit always run; setting `CF_VISUAL=1`
  additionally spawns the Bevy app via
  [`sim_bevy_soft::SoftBodyVisualPlugin`](../../sim/L1/sim-bevy-soft/src/plugin.rs)
  for trajectory replay (`soft-drop-on-plane`) or static-state
  rendering (`hertz-sphere-plane`, `compressive-block`,
  `contact-force-readout`). Tier 6's
  [`layered-silicone-device`](layered-silicone-device/) is static
  (PLY + JSON) — the cf-view z-slab artifact (categorical
  `material_id` + sequential `displacement_magnitude`) is the
  canonical visual; a Bevy mode is a possible future extension.

`feedback_visible_contacts` is the hard requirement that lands
contact-tier examples in Bevy. Pure SDF / constitutive / multi-
material / bridge / synthesis examples are static.

## Layout convention

**Standardization target (migration in progress).** Each implicit domain
folds into ONE headless `stress-test` validation superset — the
validation-consistency arc's "one domain → one stress-test" shape, mirroring
`examples/mesh/<domain>/stress-test/`:

```
examples/sim-soft/<domain>/stress-test/
├── Cargo.toml     # [package].name = "example-<domain>-stress-test"
│                  # [package.metadata.cortenforge] example_kind = "validator"
├── README.md      # museum-plaque (template per `examples/mesh/README.md`)
├── src/main.rs    # dispatcher: one module per folded example, each run()
└── out/           # gitignored (**/out/); generated PLY / JSON artifacts
```

Migrated so far: `sdf/stress-test` (rows 1–3 — `sphere-sdf-eval` +
`hollow-shell-sdf` + `sdf-to-tet-sphere`). The remaining examples still use
the pre-standardization flat crate `examples/sim-soft/<name>/` with name
`example-sim-soft-<name>`; they migrate domain-by-domain. Names are dash-case,
matching the rest of the workspace.

## Cadence

Two-pass review per example (per `feedback_one_at_a_time` and
`feedback_one_at_a_time_review`):

1. **Numbers pass (Claude)** — runs the example, verifies the
   numerical anchors. For static-artifact examples authored under
   `feedback_math_pass_first_handauthored`, anchors are
   `assert_relative_eq!` calls in `src/main.rs` and a clean exit-0
   IS the correctness signal.
2. **Visuals pass (user)** — opens the PLY in cf-view (or watches
   the Bevy playback) to confirm the visual matches expectations.

Examples are reviewed individually before the next one lands.
Multiple examples bundle into one PR per
`feedback_pr_size_ci_economics`.
