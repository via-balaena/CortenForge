# bonded — Stress Test (sim-soft bonded-multi-material validation superset)

Headless validation of the sim-soft **bonded multi-material** surface: two
solver-driven scenes whose shared-vertex (C⁰-continuous, no inter-layer slip)
multi-material bodies are validated against a **closed-form elasticity
solution**. No window, no Bevy — self-gating assertions that abort (exit 101) on
any mismatch, so `cargo xtask run-validators` runs it red-or-green.

Folded from two former per-concept examples (`bonded-bilayer-beam` row 10,
`concentric-lame-shells` row 11), each now a module preserving its hand-authored
fixture and oracle checks verbatim. One domain → one stress-test. Both sit in
the inventory's Tier 3; the two modules are complementary, not subsuming — each
is the sole coverage of one distinct mesher + boundary-condition + closed-form
analytic.

## Modules

### `bilayer_beam` (row 10) — bonded bilayer cantilever vs Euler-Bernoulli composite
A shared-vertex bonded bilayer cantilever beam (`(0.5, 0.1, 0.1) m`,
`(NX, NY, NZ) = (20, 8, 8)` → **1701 verts / 7680 tets**) under a `1 N` tip
load, meshed via `HandBuiltTetMesh::cantilever_bilayer_beam` with an inline
half-space `Field<f64>` partitioning tets into region A (`(μ, λ) = (1e5, 4e5)`
Pa, `z < H/2`) and region B (`2×` Decision J composite, `z ≥ H/2`) — an exact
**3840 / 3840** layer split. A single static-regime backward-Euler `replay_step`
(`cfg.dt = 1.0`) converges in **23 Newton iters** to `max|σ-1| ≈ 0.0069`
(well inside NH's `RequireOrientation` domain). The Saint-Venant-averaged tip
displacement `d_bilayer ≈ 1.086e-2 m` matches the **Euler-Bernoulli composite
(transformed-section) beam** analytic `δ_eb ≈ 1.299e-2 m` to **16.4 %** rel-err
(`< 0.30` per IV-3's Tet4 sub-`O(h²)` sanity gate). Discriminators:
`d_uniform_B < d_bilayer < d_uniform_A` (IV-2 lens β); per-interface-vertex
`x_final`-via-either-layer bit-equality (first IV-2 shared-vertex continuity gate
at production scale, HEADLINE B); 3 tip displacements pinned under the IV-1
sparse-tier rel-tol contract. Sole coverage of the hand-built shared-vertex
bilayer + composite-beam-bending path (axial tip force). cf-view artifact:
`out/bilayer_beam.ply` — 960-centroid y-slab point cloud (`displacement_z` +
`material_id` scalars, `DISPLACEMENT_SCALE = 20×` on rendered positions only).

### `lame_shells` (row 11) — three-shell hollow sphere vs piecewise-Lamé
A three-shell concentric hollow silicone sphere (`DifferenceSdf` of two
`SphereSdf`s, `R_OUTER = 0.10 m`, `R_CAVITY = 0.04 m`, **6456 tets / 4682 verts,
1480 referenced**) meshed via `SoftScene::layered_silicone_sphere` (BCC +
Labelle-Shewchuk) with a 3-shell `MaterialField` per Decision J's `1× / 2× / 1×`
symmetry (inner Ecoflex / middle composite / outer Ecoflex — **1032 / 1800 /
3624** tets). A per-vertex radially-outward pressure traction
(`f_v = pressure · n̂_v · A_v`, `LoadAxis::FullVector`, `pressure = 5e3 Pa`) on
the cavity surface (134 verts) and a fixed Dirichlet pin on the outer surface
(734 verts); a single static-regime `replay_step` converges in **3 Newton iters**
to `max|σ-1| ≈ 0.024`. Four Saint-Venant-averaged radial-displacement readouts
(cavity-wall `2.857e-4 m` + inner / middle / outer shell means) match the
**piecewise-Lamé thick-shell 6×6 closed-form** (`u_r^{(i)}(r) = A_i r + B_i / r²`
with fixed-outer BC `u_r^{(3)}(R_b) = 0`) within **30 %** at h/2 (rel-tol binding
for cavity / inner / middle; a `5e-6 m` eps-floor absorbs the small-magnitude
outer-shell mean). Discriminators: `u_r_uniform_2× < u_r_three_shell <
u_r_uniform_1×` on the cavity-wall mean (IV-2 lens β, three full solver passes,
HEADLINE C); strict outward monotone decay `cavity > inner > middle > outer ≥ 0`;
6 means pinned under the IV-1 sparse-tier rel-tol contract. Sole coverage of the
SDF-meshed hollow-body 3-shell + pressurized-shell path (radial `FullVector`
pressure + Dirichlet outer). cf-view artifact:
`out/concentric_lame_shells.ply` — 616-centroid z-slab point cloud
(`material_id` + `radial_displacement` scalars, `DISPLACEMENT_SCALE = 50×`).

## Run

```
cargo run -p example-bonded-stress-test --release
```

Expected: each module prints its anchor-group summary (`Anchor groups (all
assertions exit-0 on success)`) and the binary exits 0 — a clean exit-0 IS the
correctness signal (there is no `PASS` token; a failed assert aborts with 101).
Use `--release`: the FEM Newton solves are ~30× slower in debug. The solves here
are small (a 7680-tet beam and a 6456-tet shell; `lame_shells` runs three full
passes for the uniform-baseline discriminator), so the pair is cheap in release.

## Artifacts

Each module writes a per-tet centroid point-cloud PLY to `out/` (distinct
filenames — `bilayer_beam.ply` and `concentric_lame_shells.ply` — so no
namespacing is needed). Both carry two per-vertex scalars and amplify only the
rendered vertex positions (`vertex = rest + SCALE · (deformed - rest)`); the
continuous scalar carries the TRUE physical displacement and every `verify_*`
runs on unscaled solver outputs. Open in cf-view — the workspace's unified
visual-review viewer:

```
cargo run -p cf-viewer --release -- examples/sim-soft/bonded/stress-test/out/bilayer_beam.ply
cargo run -p cf-viewer --release -- examples/sim-soft/bonded/stress-test/out/concentric_lame_shells.ply
```

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 3 (rows 10–11).
- **Internal-fixture templates**: `sim/L0/soft/tests/bonded_bilayer_beam.rs`
  (IV-3 — three-refinement EB-composite + Timoshenko convergence study) and
  `sim/L0/soft/tests/concentric_lame_shells.rs` (IV-5 — three-refinement
  piecewise-Lamé convergence study). Each module ships the user-facing wrap at
  the fixture's h/2 middle refinement, mirroring its scene + load + BC +
  STATIC_DT + max_newton_iter exactly; the closed-form helpers are inlined
  verbatim.
- **Shared-vertex continuity**: `sim/L0/soft/tests/multi_material_continuity.rs`
  (IV-2 — the lens-α / lens-β claims `bilayer_beam` exposes at production scale).
- **Constructors**: `sim/L0/soft/src/mesh/hand_built.rs`
  (`cantilever_bilayer_beam`); `sim/L0/soft/src/readout/scene.rs`
  (`SoftScene::layered_silicone_sphere`); `sim/L0/soft/src/field/layered.rs`
  (`LayeredScalarField`).
- **Sister sim-soft folds**: `sdf/stress-test` (rows 1–3), `stretch/stress-test`
  (rows 4–6), `scalar-field/stress-test` (rows 8–9 — the field-exposition
  multi-material counterpart, no solver).
- **Book**: Part 3 Ch 03 §03 `00-bonded.md` (bonded interfaces — the
  C⁰-continuous-displacement-by-construction claim); Part 2 §04 (linear-elastic
  regime where Neo-Hookean reduces to Lamé).
