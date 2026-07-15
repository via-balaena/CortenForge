# sdf-bridge — Stress Test (sim-soft SDF-bridge validation superset)

Headless validation of both directions of the **`cf_design::Sdf` trait as the
SDF interop spine** — the example counterpart to the sim-soft library's
`sim/L0/soft/src/sdf_bridge/` module. No window, no Bevy — self-gating
assertions that abort (exit 101) on any mismatch, so `cargo xtask
run-validators` runs it red-or-green.

Folded from two former per-concept examples (`mesh-scan-as-solid` row 15,
`solid-to-sim-soft` row 16), each now a module preserving its hand-authored
fixture and oracle checks verbatim. One domain → one stress-test. Both sit in
the inventory's Tier 5; the two modules are complementary, not subsuming — each
is the sole coverage of one direction across the trait boundary.

## Modules

### `mesh_scan` (row 15) — scan → design: a mesh-SDF satisfies `cf_design::Sdf`
A mesh-derived SDF (`mesh_sdf::Signed<TriMeshDistance, PseudoNormalSign>`)
satisfies `cf_design::Sdf` (PR3 F2), so a scanned body is a first-class design
primitive. A **programmatic 12-triangle cube** (`R = 1.0`, 8 verts / 12 faces)
is round-tripped through a **runtime-written binary STL** on disk (`save_stl` →
`load_mesh` — no checked-in asset; the "scan" is the workflow, not the geometry),
then every anchor dispatches through `&dyn cf_design::Sdf`:

- **Closed-form L∞-ball SDF** — face-region probes `= +0.5` bit-exact,
  edge-region `= √0.5` and vertex-region `= √0.75` within `1e-12`, five interior
  probes bit-exact, and a finite-difference gradient face-band check
  (`‖grad‖ ∈ [0.9, 1.1]`, aligned with the outward normal).
- **STL round-trip** — the reloaded `ScanSdf` agrees with the in-memory original
  at every named face + origin probe (integer `±R` round-trips losslessly
  through STL's f32).
- **Bulk grid** (17³ = 4913 points in `[−2, 2]³`) — closed-form-vs-trait
  identity at every point (`1e-12`); heuristic strict-interior coverage (343
  points, `|coord| ≤ 0.75`, all `eval < 0`); off-diagonal strict-interior
  raycast coverage (294 = 343 − 49); `raycast_inside` **pinned at 729** (all
  `|coord| ≤ R` including the boundary shell) as a parry-pseudo-normal drift
  tripwire; F2-caveat-absent identity (`heuristic_inside == 343`). The **HE-1
  ray-edge diagonal degeneracy** (49 `y = z` interior probes that Möller-Trumbore
  may miss) is documented and **excluded from assertion** — never locked to a
  wrong value; the `heuristic ↔ raycast divergence` (≈386, dominated by HE-1) is
  printed as a diagnostic, not pinned.

Sole coverage of the mesh-SDF → `cf_design::Sdf` direction. cf-view artifacts:
`out/cube_scan.stl` (the STL "scan" input) + `out/sdf_grid.ply` (4913-point cloud,
`signed_distance` divergent + `inside_raycast` categorical).

### `solid_to_sim` (row 16) — design → sim: a typed `Solid` drives sim-soft meshing
A typed `cf_design::Solid` CSG body (`Solid::sphere(0.10).subtract(Solid::sphere(
0.04))` — a hollow shell) coerces to `&dyn Sdf` and drives
`sim_soft::SdfMeshedTetMesh::from_sdf` (PR3 F1 `impl Sdf for Solid` + F3
`sim_soft::Sdf` re-export — no per-shape glue). Anchor groups:

- **`bridge_equivalence`** (HEADLINE A) — the typed-`Solid` mesh
  `equals_structurally` a `DifferenceSdf<SphereSdf>` baseline with per-vertex
  positions bit-equal (`EXACT_TOL = 0.0`); cf-design's `Sphere`/`Subtract`
  arithmetic is bit-identical to sim-soft's `SphereSdf`/`DifferenceSdf`, so the
  BCC + stuffing pipeline produces bit-identical meshes.
- **`counts_exact`** — `n_tets = 6456`, `n_vertices = 4682`, `n_referenced =
  1480`, `n_pinned = 734`, `n_loaded = 134` — bit-equal to row 11's counts
  (topology is a function of geometry + `cell_size` only, not material).
- **`quality_floors`** — per-tet `signed_volume > 0`, `aspect_ratio ≥ 0.05`,
  dihedral `∈ [5°, 175°]` (Theorem-1 envelope).
- **`solver_converges`** — a single static-regime `replay_step` converges in 3
  Newton iters (residual `~3.4e-12 < tol = 1e-10`).
- **`cavity_wall_lame`** (HEADLINE B) — Saint-Venant cavity-wall mean radial
  displacement `≈ 3.245e-4 m` matches the single-material Lamé closed-form
  `≈ 3.823e-4 m` within 30 % (`~15 %` rel-err at h/2; IV-5 super-quadratic
  convergence at the fine end).
- **`captured_cavity_wall_bits`** — the cavity-wall mean is pinned bit-equal
  (IV-1 sparse-tier `1e-12` rel) to row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS`:
  same geometry + same Lamé pair + same faer path ⇒ bit-equal observation
  (cross-row continuity guard).

Sole coverage of the typed-`Solid` → sim-soft meshing + FEM direction. cf-view
artifact: `out/shell_zslab.ply` (616-centroid z-slab cloud, `radial_displacement`
scalar, `DISPLACEMENT_SCALE = 50×`).

## Run

```
cargo run -p example-sdf-bridge-stress-test --release
```

Expected: each module prints its anchor-group summary + stats and the binary
exits 0 — a clean exit-0 IS the correctness signal (there is no `PASS` token; a
failed assert aborts with 101). Use `--release`: the FEM mesh-and-solve in
`solid_to_sim` is ~30× slower in debug. The solves are small (a 6456-tet shell,
a 4913-point grid), so the pair is cheap in release.

## Artifacts

Each module writes to `out/` with distinct filenames (no namespacing needed):
`cube_scan.stl` (binary, 12 facets — the round-trip input), `sdf_grid.ply`
(4913-point cloud, two per-vertex scalars), and `shell_zslab.ply` (616-centroid
z-slab, one scalar; only rendered positions are `50×`-amplified — the scalar
carries the TRUE physical displacement and every `verify_*` runs on unscaled
solver outputs). Open in cf-view:

```
cargo run -p cf-viewer --release -- examples/sim-soft/sdf-bridge/stress-test/out/sdf_grid.ply
cargo run -p cf-viewer --release -- examples/sim-soft/sdf-bridge/stress-test/out/shell_zslab.ply
```

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 5 (rows 15–16).
- **Bridge code these validate**: `sim/L0/soft/src/sdf_bridge/` (F3 re-export +
  `DifferenceSdf` baseline) and `design/cf-design/src/sdf.rs` (F1 `impl Sdf for
  Solid`, F2 `impl Sdf for SignedDistanceField`).
- **F2 sign-heuristic caveat fixture**: `examples/mesh/mesh-sdf-distance-query`
  (octahedron — 6 vertex-region false-positives at the bbox boundary, the
  fixture that DOES trigger the F2 caveat that `mesh_scan`'s cube does not).
- **Sister sim-soft folds**: `sdf/stress-test`'s `sdf_to_tet` (row 3 — the
  single-material `SdfMeshedTetMesh::from_sdf` counterpart `solid_to_sim`
  generalises to a typed `Solid` source); `bonded/stress-test`'s `lame_shells`
  (row 11 — the three-shell scene whose counts + cavity-wall bits `solid_to_sim`
  pins against for cross-row continuity).
- **Book**: Part 7 §00 §01 (sharp-CSG difference operator); Part 3 (BCC +
  Isosurface Stuffing meshing).
