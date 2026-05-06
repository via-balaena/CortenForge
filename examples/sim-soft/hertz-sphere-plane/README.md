# hertz-sphere-plane

**Phase 5 V-3 user-facing wrap — soft sphere quasi-statically pressed against a `RigidPlane` by an axial force; contact-patch radius compared against the Hertzian closed-form.** A `SphereSdf` body of radius `R = 1 cm` is BCC-meshed via `SdfMeshedTetMesh::from_sdf` at three refinement levels (`h, h/2, h/4 = 3, 1.5, 0.75 mm`); four cardinal-direction equator pins remove the rigid-body modes, the top-of-sphere band carries `LoadAxis::AxisZ` traction summing to `−F = −500 mN`. A single `RigidPlane(+ẑ, offset = -(R + d̂))` resists the press with **V-3-LOCAL `κ = 1e3 N/m`** (10× softer than `PENALTY_KAPPA_DEFAULT = 1e4`, applied via `PenaltyRigidContact::with_params`); `STATIC_DT = 1.0 s` collapses the inertial term so a single backward-Euler step from rest reaches static equilibrium per refinement.

The headline new capability vs row 12's drop-and-rest is **analytical contact-pressure validation (V-3)** — the second PR2 example row. Row 13 mirrors `sim/L0/soft/tests/hertz_sphere_plane.rs` (V-3) verbatim on constants + scene + plan-change-2 gate selection; the contributions vs the internal fixture are: **(a)** finest-refinement deformed-mesh PLY emit with per-vertex `contact_force_z` extras (cf-view auto-colormap renders the contact patch as a bright disk), **(b)** three-section JSON emit (scalars + per-active-vertex `(r, sd, force_z)` at finest + 200-point Hertz analytic curve) for matplotlib overlay, **(c)** opt-in Bevy static-state visualization under `CF_VISUAL=1` showing the deformed sphere + coral `a_FEM` annulus + white `a_Hertz` annulus on the plane (patch-radius gap reads visually 1:1 with the asserted `rel_err_a` number).

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output: `out/hertz_sphere_plane.ply` (finest deformed boundary mesh, 139936 vertices including BCC orphans, 9480 triangles via `Mesh::boundary_faces` + per-vertex `contact_force_z` extra) and `out/hertz_sphere_plane.json` (scalars + per-vertex contact data + 200-point Hertz analytic).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The second user-facing example of Tier 4 penalty contact per [`EXAMPLE_INVENTORY.md`][inv] — Hertzian quasi-static contact mechanics on curved geometry, the canonical analytical-validation gate for `PenaltyRigidContact` (V-3 per `phase_5_penalty_contact_scope.md`):

```rust
let (mesh, bc, initial, _default_contact, theta) = SoftScene::sphere_on_plane(
    RADIUS, cell_size, FORCE, MaterialField::uniform(MU, LAMBDA),
)?; // (1e-2, {3, 1.5, 0.75}e-3, 0.5, NH(2e5, 8e5))

// V-3-LOCAL κ override per scope memo Decision J's V-3-may-tune authority.
let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), -(RADIUS + PENALTY_DHAT));
let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, PENALTY_DHAT); // (1e3, 1e-3)

let mut cfg = SolverConfig::skeleton();
cfg.dt = 1.0;            // static — collapse M / dt² Tikhonov
cfg.max_newton_iter = 50;

let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);
let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
// ... walk active vertices, compute a_FEM = max sqrt(x² + y²) ...
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

This is **row 13 of the sim-soft examples arc** — the second Tier 4 penalty-contact example, the second PR2 example row, and the second user-facing demo of `sim-bevy-soft` (under `CF_VISUAL=1`). Row 13 is the canonical V-3 analytical-validation gate (`sim/L0/soft/tests/hertz_sphere_plane.rs`), exposed as a user-facing demo with deformed-mesh + per-vertex contact-force PLY, three-section JSON, and a static-state Bevy comparison visualization.

**Why `a_FEM` is the headline (and `δ_FEM` is diagnostic-only).** V-3 commit 9 surfaced empirically that the rigid-plane Hertz indentation `δ_Hertz` is **structurally unreachable** in the penalty regime: penalty's contact-band depth `d̂ = 1 mm` provides ~1 mm of "soft compliance" before saturating, and at the V-3 force `F = 500 mN` + Ecoflex stiffness, the sphere reaches penalty-band equilibrium at ~219 μm COM descent — far short of the rigid-limit `R + d̂ + δ_Hertz ≈ 11.3 mm`. Higher κ doesn't fix it (penalty stiffer = even less descent); lower κ allows more descent but elastic compliance caps at ~75 μm regardless. Penalty's compliance band fundamentally prevents the `force = ∞ at sd ≤ 0` rigid-wall behavior Hertz assumes. Phase H IPC's logarithmic barrier `−κ log(d/d̂)` recovers the rigid limit at `κ → ∞`; until then, the `δ` comparison fails not from FEM error but from the model's stepping-stone design (BF-12 amendment).

`a_FEM` (the contact-patch radius) is the Hertz-physical quantity that DOES match in the penalty regime. Hertz `a ∝ F^{1/3} R^{1/3} / E*^{1/3}` is independent of the local pressure profile that penalty distorts; the FEM finds the correct contact-patch radius scaling within mesh-bound discretisation error. Both `δ_FEM` and `a_FEM` appear in stdout + JSON for inspection; **only `a_FEM` is asserted**. Row 13 mirrors V-3 fixture's "Plan change 2" reframe verbatim.

**Why the V-3-LOCAL `κ = 1e3` override.** At `PENALTY_KAPPA_DEFAULT = 1e4` and the scope-memo `F` range (50-200 mN), the multi-vertex contact threshold `h < sqrt(2 R · F/κ)` is `≈ 0.45 mm` — finer than the §9 finest cell. Only the south pole engages, equilibrating against the full `F` in a linear-spring relation (not Hertzian pressure). Softening to `κ = 1e3` raises the multi-vertex threshold to `≈ 3.16 mm` at `F = 500 mN`, engaging multi-vertex Hertz contact at all three V-3 refinement levels — confirmed empirically: 1, 5, 45 active pairs at `(h, h/2, h/4)`. Penalty bias `pen_avg = F/(κ · N_active) ≈ 9 μm` at h/4 is `~3%` of `δ_Hertz` — well below the 20% rel-err gate. Production scenes (V-1 / V-3a / V-4 / V-5 / row 12) keep the default κ; this constant only enters via `PenaltyRigidContact::with_params` in `run_at_refinement` and must NOT be propagated upstream.

**Why three refinements.** The convergence story needs three points: monotonic decrease, finite Cauchy ratio, finest-level rel-err. V-3 establishes the empirical-feasible upper bound at `h/4 = 0.75 mm` (~2 min release-mode total runtime; debug-mode unviable at 10-20 min). Coarser than `h = 3 mm` enters the single-vertex regime where `a_FEM ≈ 0` (rel-err ≈ 100%); finer than `h/4 = 0.75 mm` runs into Newton convergence with multi-vertex active-set churn at large N (>12 min release-mode at `h/4 = 0.5 mm`).

**Why `cargo run --release` only.** At `h/4 = 0.75 mm` with multi-vertex contact-Newton, debug-mode runtime extrapolates to ~10-20 minutes — over the CI 30-minute budget if added to the test matrix. The example mirrors V-3 fixture's release-mode-only policy via the `--release` invocation (ship-time). Per [`feedback_release_mode_heavy_tests`][rel].

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

Compile-time `const { assert!(...) }` on all input scalars + cell-size monotonicity + Newton-iter cap margin + REL_ERR_GATE / SMALL_STRAIN_CEILING domain (`(0, 1)`).

### 2. `mesh_topology_exact`

| Refinement | `n_tets` | `n_vertices` | `n_referenced` | `n_loaded` | `n_pinned` |
|---|---|---|---|---|---|
| h    | 2208   | 18696  | 561   | 22  | 4 |
| h/2  | 15384  | 39732  | 3155  | 45  | 4 |
| h/4  | 124344 | 139936 | 23353 | 118 | 4 |

III-1 determinism contract per refinement. Captured 2026-05-06 on the row 13 capture platform; the BCC + bbox-margin orphan ratio (~97% at h, falling toward ~83% at h/4 as the surface-cut tet shell densifies relative to bbox-corner allocations) is documented in V-3 docstring.

### 3. `boundary_partition`

Per refinement: `n_pinned ≥ 3` (four cardinal-direction equator points, deduplicated; under generic BCC resolution all four hit distinct vertices = 4) and `n_loaded > 0` (top-of-sphere band, `band_tol = 0.5 · cell_size`).

### 4. `solver_per_step_invariants`

Per refinement: no NaN in `x_final`; `iter_count < NEWTON_ITER_SANITY_CAP = 40` (10-iter margin under `MAX_NEWTON_ITER = 50` per V-3 policy); finite residual norm. Captured `iter_count`: 3, 4, 10 at (h, h/2, h/4) — well below the cap.

### 5. `contact_engagement`

Per refinement: `n_active_pairs > 0`. Captured 1, 5, 45 at (h, h/2, h/4) — the h-level single-vertex regime (south pole only) is the documented baseline; multi-vertex engagement at h/2 and h/4 is the asserted Hertzian regime.

### 6. `small_strain_validity`

Per refinement: `0 < a_FEM / R < SMALL_STRAIN_CEILING = 0.30` (Hertz validity domain). At V-3 parameters expected `a_FEM / R ≈ 0.18` (textbook `a/R ≲ 30%`). Catches a regression where `a_FEM` exceeds the Hertz domain (would invalidate the comparison).

### 7. `monotonic_a_convergence`

`rel_err_a` decreases across `(h → h/2 → h/4)`. Captured: 99.9998% → 40.17% → 15.84%. Catches resolution-induced scatter or a regression in contact-patch resolution with mesh density.

### 8. `cauchy_a_convergence`

`|a_h2 − a_h4| < |a_h − a_h2|` (Cauchy ratio `< 1`). Captured ratio `0.4068`. Stronger than monotonic-only — catches "bounded but non-converging" regressions where the rel-err sequence wanders without settling.

### 9. `finest_level_hertz_match`

| Anchor | Bound |
|---|---|
| `rel_err_a` at h/4 | `< REL_ERR_GATE = 20%` |

The headline analytical-comparison gate. Captured `15.84%` — `~4 percentage points` headroom over the gate. Mesh-bound; tightening to `<10%` is Phase H Tet10 + adaptive refinement work, not Phase 5.

### 10. `captured_bits_hertz_metrics`

Four `f64` Hertz metrics + six `usize` counts captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `a_fem` at h, h/2, h/4 | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `delta_fem` at h/4    | rel `1e-12`, abs `1e-12` (`f64`, diagnostic) |
| `n_active_pairs` per refinement | strict equality (3 × `usize`) |
| `iter_count` per refinement     | strict equality (3 × `usize`) |

Captured drift-detection contract: `~few thousand tets through faer's sparse Cholesky` lives in IV-1's sparse-at-scale tier. The contract is **relative tolerance, not strict bit-equality** for the floats — observed bits captured for regression detection, compared via `assert_relative_eq!` at `1e-12` rel. Capture provenance + failure-mode protocol (rule out toolchain drift before re-baking) inline above the const blocks. Same precedent as PR1 rows 6+10+11 + row 12.

This is the row's **tight regression-detection gate** — anchors 4-9 document physical reasonableness; anchor 10's `1e-12` rel against pinned bits catches any FEM-numerics-shifting regression at machine precision.

## Visuals

### PLY artifact

`out/hertz_sphere_plane.ply` — finest-refinement deformed boundary mesh with per-vertex `contact_force_z` extra:

```text
139936 vertices  (finest deformed positions in physics +Z frame; ~85% are unreferenced
                  BCC-orphan corners staying frozen at rest, the 23353 referenced cohort
                  carries the actual settled geometry)
9480 faces       (boundary triangulation via Mesh::boundary_faces — outward-CCW winding
                  from the right-handed-tet `signed_volume > 0` invariant per
                  boundary_faces_from_topology)
normals          : smooth, area-weighted via AttributedMesh::compute_normals
extras["contact_force_z"]
                 : f32 per vertex; zero everywhere except 45 active vertices
                   where `force_z = κ · (d̂ - sd) > 0`. cf-view's auto-colormap
                   (sequential viridis on positive scalar) renders the contact patch
                   as a bright disk on the deformed sphere — fills row 12's
                   "contact-band membership is a row-13 Hertz-patch concern" deferral.
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/hertz-sphere-plane/out/hertz_sphere_plane.ply
```

### Bevy static-state visualization (CF_VISUAL=1)

```text
CF_VISUAL=1 cargo run -p example-sim-soft-hertz-sphere-plane --release
```

Spawns an `OrbitCamera` scene with four entities (rendered at `RENDER_SCALE = 100×` physics scale per the section below):

- **Soft mesh** (`Mesh3d` + `MeshMaterial3d` + `Trajectory`) — coral PBR sphere built via `sim_bevy_soft::mesh::build_soft_mesh` from the rest configuration + `Mesh::boundary_faces()` triangulation, animated to the captured h/4 deformed positions on first `step_replay` tick (single-frame trajectory; `frame_index_at` clamps at end). `Transform::from_scale(100)` so cm-scale physics renders at meter scale.
- **Rigid plane** (`Mesh3d` + `MeshMaterial3d`) — `8 m × 8 m` total (`4 × R × RENDER_SCALE = 4 m` half-size) gray PBR quad at Bevy `y = -(R + d̂) · 100 = -1.1 m` (= physics `z = -(R + d̂)` under `UpAxis::PlusZ`'s `(x,y,z) → (x,z,y)` swap), normal `+Y` Bevy. Static; carries no `Trajectory`.
- **`a_FEM` annulus** (`Mesh3d` + `MeshMaterial3d`) — coral thin ring at `r = a_FEM = 1.497 mm`, thickness `±1%`. Lays flat on the plane (rotated `-π/2` around X). Unlit material so it reads identical regardless of camera angle.
- **`a_Hertz` annulus** (`Mesh3d` + `MeshMaterial3d`) — white thin ring at `r = a_Hertz = 1.778 mm`, same thickness. Slightly above the `a_FEM` ring (1 mm physics offset) to avoid z-fighting at extreme camera angles.

Plus a directional-light at `12 klx` from upper-front-right + per-camera `AmbientLight` at `80 cd/m²` (mirror row 12). The HUD shows numeric `a_FEM`, `a_Hertz`, `rel_err_a` values top-right so the visual ring gap reads 1:1 with the asserted scalar.

The scene is **static** — single quasi-static step with no temporal evolution. `R` is a no-op for this row (no animation to reset).

### Controls

| Key | Action |
|---|---|
| **Left-drag** | Orbit camera around the contact zone |
| **Mouse scroll** | Zoom |
| **Right-drag** | Pan |
| **Close window** | Exit the app |

### Why the rendered scene is `100×` simulation scale

Mirrors row 12 verbatim. Bevy 0.18's pipeline defaults (near plane `0.1 m`, OrbitCamera `min_distance = 0.1 m`, AmbientLight brightness, depth precision) were tuned for human-scale (1 m+) scenes. At cm-scale rendering, the camera approaches the default near plane on any zoom-in and clips the geometry; lifting the rendered scene to meter scale puts everything safely past the defaults. **Headless asserts + JSON + PLY are scale-invariant** — they operate on unscaled physics positions, so this is a visualization-only adjustment.

### Matplotlib plot

`uv run examples/sim-soft/hertz-sphere-plane/plot.py`

Single-panel comparison: per-active-vertex penalty `force_z` (right axis, mN, coral scatter) overlaid on Hertz analytic `p(r)` curve (left axis, kPa, blue line). Vertical dashed lines at `r = a_FEM` (coral) and `r = a_Hertz` (blue) — the gap is the asserted `rel_err_a`. Title shows `rel_err_a` + `Cauchy ratio` numerically.

The two y-axes carry different units (force per vertex vs continuous pressure) — the comparison is **qualitative shape match** (radial decay profile + finite at r = 0 + zero at r = a). Quantitative pressure reconstruction would require per-vertex Voronoi cell areas, an honest follow-on. Per [`feedback_visible_contacts`][vis] — for Tier 4 penalty contact rows, the visual review is real (not collapsed to JSON read).

[vis]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, per-refinement results table, Hertz convergence summary).

## Run

```text
cargo run -p example-sim-soft-hertz-sphere-plane --release
```

Output: `out/hertz_sphere_plane.ply`, `out/hertz_sphere_plane.json`, and stdout summary.

```text
CF_VISUAL=1 cargo run -p example-sim-soft-hertz-sphere-plane --release
```

Output: same as above + Bevy windowed static-state visualization.

```text
uv run examples/sim-soft/hertz-sphere-plane/plot.py
```

Output: `out/hertz_sphere_plane.png` (Hertz pressure profile + per-vertex penalty force overlay).

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for this example (~2 min release-mode total runtime across the three refinements; debug-mode is 10-20 min and unviable). The exact-pin counts and captured Hertz-metric bits were captured under release-mode build; matching that invocation shape removes one variable from the determinism contract.

## Cross-references

- **Sister sim-soft examples**: `soft-drop-on-plane` (row 12 — V-5 dynamic-integration drop-and-rest, the immediate precedent with the same V-3-LOCAL-vs-default κ + d̂ structure but transient-Newton instead of static; sim-bevy-soft + cf-bevy-common foundation phase shipped through it); future `compressive-block` (row 14 — V-3a compressive-load variant at the rigid-plane interface, third PR2 row) and `contact-force-readout` (row 18 — quantitative readout for the calibration loop).
- **Internal-fixture template**: `sim/L0/soft/tests/hertz_sphere_plane.rs` (V-3 — analytical contact-mechanics gate; the load-bearing scientific gate of Phase 5 per scope memo §1 V-3 + §8 commit 9 + Decision D). Row 13 mirrors V-3's scene + load + BC + DT + κ override + cell sizes + a_FEM gate verbatim. The user-facing extensions are the per-vertex force PLY, three-section JSON + plot.py, and the Bevy static-state visualization.
- **`SoftScene::sphere_on_plane`**: `sim/L0/soft/src/readout/scene.rs:557-698` — `(radius, cell_size, force, material_field) -> (mesh, bc, initial, contact, theta)`. `SphereSdf` body BCC-meshed at `cell_size`, four cardinal-direction equator pins, top-of-sphere band loaded with `LoadAxis::AxisZ` traction summing to `−F`, default-κ `PenaltyRigidContact` (replaced here by V-3-LOCAL `with_params`).
- **`Mesh::boundary_faces`**: `sim/L0/soft/src/mesh/mod.rs:113-132` — outward-CCW boundary-face cache populated at construction via `boundary_faces_from_topology`. Row 12 was its first user-facing headless consumer; row 13 is its second.
- **`sim-bevy-soft`**: `sim/L1/sim-bevy-soft/` — Bevy soft-body trajectory replay (single-frame trajectory at row 13 makes this a static-state render). Row 13 is its second consumer after row 12; the row 12-fixed foundation patches (ReplayEpoch + R-key reset + area-weighted normals) are inherited unchanged.
- **`cf-bevy-common`**: `cf-bevy-common/` — shared Bevy 0.18 helpers (`UpAxis` input → Bevy frame swap, `OrbitCamera` + `OrbitCameraPlugin`).
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` — cf-view static-artifact viewer the headless PLY targets (auto-colormap on positive `extras` scalar).
- **Phase 5 scope memo**: `sim/docs/todo/phase_5_penalty_contact_scope.md` — V-3 spec at §1 V-3 + Decision J (V-3-may-tune authority for κ override) + Decision E (penalty has no CCD; Phase H IPC will).
- **Book reference**: Part 4 §00 §00 ("Penalty contact pathology and validation scope") + Part 5 §00 §02 ("Hertzian contact analytic"). Row 13 is the **regression-test artifact** for the analytical-validation chapter section — V-3's plan-change-2 reframe gate, exposed user-facing at the example layer with the deformed-mesh + per-vertex contact-force visualization that the internal fixture omits.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visible_contacts`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md),
  [`feedback_release_mode_heavy_tests`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md).
