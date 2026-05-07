# contact-force-readout

**Phase 5 V-3a scene + per-active-pair contact readout via the [`PenaltyRigidContact::per_pair_readout`][readout] foundation patch (commit `995fb0bf`). Single-refinement (n=8) headline: the public per-pair readout surface returns the same per-vertex forces row 14's `compressive-block` reconstructs inline from known plane geometry — bit-equivalent at 1e-12 relative tolerance.** Inherits row 14's V-3a-LOCAL `(d̂, δ)` override + the bilateral-compression scene + the F_R bound bracket; the new capability is the public per-pair readout surface that decouples the example from the axis-aligned-plane closed-form, letting future rows (row 20 silicone-device with scan-derived `MeshSdf` rigid primitive) read out per-pair forces without per-shape geometry duplication.

[readout]: ../../../sim/L0/soft/src/contact/penalty.rs

The contributions vs the V-3a fixture + row 14 are: **(a)** the headline accessor-vs-manual consistency gate (`Σ readouts.force_on_soft.z` bit-equivalent to the row-14-style manual reconstruction), **(b)** finest-refinement deformed-mesh PLY emit with per-vertex `contact_pressure` extras (Pa, uniform per-pair area approximation; cf-view auto-colormap renders the contact patch in pressure units), **(c)** two-section JSON emit (scalars + per-active-pair `(vertex_id, primitive_id, x, y, z, sd, force_x/y/z, pressure)`) for matplotlib top-down patch scatter, **(d)** opt-in Bevy static-state visualization under `CF_VISUAL=1` mirroring row 14's harness verbatim with HUD swapped to per-pair statistics.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output: `out/contact_force_readout.ply` (finest deformed boundary mesh, 729 vertices, 768 triangles via `Mesh::boundary_faces` + per-vertex `contact_pressure` extra) and `out/contact_force_readout.json` (scalars + per-active-pair contact data).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The fourth user-facing example of Tier 4 penalty contact per [`EXAMPLE_INVENTORY.md`][inv] — quantitative readout for the relative-comparison sim + post-cast empirical-calibration loop (Fork B). Row 18 inherits row 14's bilateral-compression scene (V-3a) verbatim; the new code is the per-pair readout surface and the accessor-vs-manual gate that pins it.

```rust
let (mesh, bc, initial, _default_contact) = SoftScene::compressive_block_on_plane(
    EDGE_LEN, CELL_SIZE, DISPLACEMENT, &MaterialField::uniform(MU, LAMBDA),
); // (1e-2, 1.25e-3, 5e-5, NH(1e5, 4e5))

let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE);
// ... move contact + mesh into solver, run replay_step, capture x_final ...

// Foundation patch — public per-pair readout surface. The mesh +
// contact were moved into the solver; build a fresh inspection
// pair (compressive_block_on_plane is deterministic) for the
// readout call.
let (inspection_mesh, _, _, _) = SoftScene::compressive_block_on_plane(...);
let inspection_contact = PenaltyRigidContact::with_params(...);
let readouts: Vec<ContactPairReadout> =
    inspection_contact.per_pair_readout(&inspection_mesh, &positions_vec3);
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

This is **row 18 of the sim-soft examples arc** — the fourth and final Tier 4 penalty-contact example, the fourth PR2 example row, and the fourth user-facing demo of `sim-bevy-soft` (under `CF_VISUAL=1`).

## Why per-pair readout (vs. row 14's manual reconstruction)

Row 14 reconstructs each top-face vertex's penalty force inline at [`compressive-block/src/main.rs:794-814`][r14main]:

[r14main]: ../compressive-block/src/main.rs

```rust
for &v in &top_face_vertices {
    let z_v = step.x_final[3 * v as usize + 2];
    let sd = EDGE_LEN - DISPLACEMENT - z_v;          // closed-form, axis-aligned
    if sd < D_HAT_OVERRIDE {
        let force_z = KAPPA * (D_HAT_OVERRIDE - sd); // duplicates penalty.rs
        f_r_fem += force_z;
    }
}
```

This works because the V-3a plane is axis-aligned (`n = -ẑ`) and there's exactly one rigid primitive — but it **duplicates `penalty.rs`'s gradient formula** at the example layer, and breaks the moment the row switches to a non-axis-aligned primitive (sphere, scan-derived `MeshSdf`, cf-design `Solid`). Row 20's silicone-device E2E scene needs exactly that — a scan-derived `MeshSdf` rigid primitive — and would have to author per-shape signed-distance code at the example layer to read out per-pair forces.

Row 18 lifts the reconstruction into a public surface: [`PenaltyRigidContact::per_pair_readout(mesh, positions)`][readout] returns a `Vec<ContactPairReadout>` with `(pair, position, sd, normal, force_on_soft)` per active pair, **primitive-agnostic**. Any `impl Sdf` (plane, sphere, MeshSdf) flows through — the readout calls `Sdf::eval` for `sd` and `Sdf::grad` for the outward normal, computes `force_on_soft = +κ·(d̂ - sd)·n`, and assembles. The headline gate at `verify_accessor_vs_manual_consistency` asserts `−Σ readouts.force_on_soft.z` is bit-equivalent at 1e-12 rel to the row-14-style manual reconstruction (sign flip for the rigid-reaction-vs-soft-side convention) — the structural anchor that the new public surface returns the same arithmetic the inline duplication produces.

The readout type docs in [`contact/mod.rs`][modrs] cover the sign convention: `force_on_soft` is the force the contact model exerts on the soft side, equal to `−gradient` by the force-as-`−∇U` identity. For V-3a's outward normal `n = -ẑ` and active gap `(d̂ − sd) > 0`, the soft-side z-component is **negative** (vertex pushed DOWN, out of the rigid); the rigid reaction is its negation (positive UP, the row-14-style `F_R_FEM`).

[modrs]: ../../../sim/L0/soft/src/contact/mod.rs

## Why single-refinement n=8

Row 14's headline is convergence (Cauchy ratio across n=2/4/8); row 18's headline is per-pair readout (an API/structural property of a single configuration). Re-running n=2 and n=4 would add no signal — the accessor-vs-manual gate works at any single refinement, and n=8 gives the maximum 81 active pairs for the per-pair JSON to be informative. The IV-1 captured-bits triplet collapses to a single-value scope (10 f64 captures + 6 usize); `lambda_z_avg`, `eps`, and the bit-equivalent of `f_r_total` are the same numbers row 14 captures at its `*_N8_REF_BITS`, so cross-row regression detection holds (a regression at the V-3a `(d̂, δ)` override would fail BOTH rows simultaneously).

Captured Newton iter count: `3` — bit-equivalent to row 14's `ITER_COUNT_N8_REF`.

## Why uniform per-pair area approximation (vs. Voronoi)

Pressure (Pa) requires per-vertex area to convert from per-pair force (N). The exact derivation is per-vertex Voronoi-cell or barycentric area on the boundary triangulation — non-trivial enough that V-3 (row 13) deferred per-vertex pressure entirely, emitting raw `force_z` to PLY instead.

Row 18 uses a **uniform per-pair area approximation**: `A_per_pair_uniform = A_top_face / n_active_pairs = 1e-4 m² / 81 ≈ 1.235e-6 m²`. Per-pair pressure is then `|force_z| / A_per_pair_uniform`. The **aggregate** `mean_pressure = sum(|force_z|) / A_top_face` is exact engineering stress σ_z (independent of the per-vertex area choice, because the per-pair areas sum to the full top-face area by construction); per-vertex pressure varies because `force_z` varies (interior-vs-corner penetration depth differs by a few μm under the mixed BC's Saint-Venant boundary-layer pattern — **interior** vertices are laterally surrounded by material on all four sides → locally uniaxial-strain → vertically stiffer → equilibrium reached at less compliance → vertex stays close to its rest z → larger penetration into the plate → ~6× larger `|force_z|` and pressure than corners. **Corner** vertices have free side-faces in two directions → can bulge laterally → locally uniaxial-stress → vertically softer → vertex moves DOWN to nearly match the plate level → smallest penetration). The visual in `plot.py` reads as a uniform-yellow interior framed by darker edges with the four corners darkest.

Honest "approximate per-vertex pressure" — not "exact Voronoi." Voronoi-cell area is banked as a live followup (a per-vertex-area helper on `Mesh` that consumes `boundary_faces` would lift the approximation; out of scope for row 18).

## Why `cargo run --release` only

Mirrors row 14 + V-3a fixture precedent. The IV-1 captured-bits contract is platform + build-mode-locked; matching row 14's `--release` invocation removes one variable from the determinism contract. V-3a is fast enough at single n=8 that debug-mode is also viable, but consistency with sister rows wins over the cold-build-time savings. Per [`feedback_release_mode_heavy_tests`][rel].

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 10 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

Compile-time `const { assert!(...) }` on all input scalars + `DISPLACEMENT < EDGE_LEN` + Newton-iter cap margin + `SMALL_STRAIN_CEILING` / `LAMBDA_Z_FLOOR` / `LATERAL_FORCE_TOL` / `A_TOP_FACE` positivity + `(0, 1)` domain checks.

### 2. `mesh_topology_exact`

| Refinement | `n_tets` | `n_vertices` | `n_loaded` | `n_pinned` |
|---|---|---|---|---|
| n=8 | 3072 | 729 | 0 | 81 |

III-1 determinism contract for the single n=8 refinement. Captured 2026-05-06 on macOS arm64 in `--release` build. **`n_loaded == 0` exact** is V-3a-specific — no external traction; load is penalty-mediated. **`n_pinned = (n+1)² = 81`** because the helper full-pins all bottom-face vertices.

### 3. `solver_per_step_invariants`

No NaN in `x_final`; `iter_count < NEWTON_ITER_SANITY_CAP = 40` (10-iter margin under `MAX_NEWTON_ITER = 50`); finite residual norm. Captured `iter_count = 3` — bit-equivalent to row 14's n=8 capture.

### 4. `contact_engagement`

`n_active_pairs == 81 = (n+1)²` exact. Every top-face vertex inside the `d̂`-band at equilibrium per V-3a fixture's docstring (cube compresses by `~6e-5 m` from `δ = 5e-5 m` plate displacement plus `d̂_override = 1e-5 m` band engagement, leaving every top vertex with a small positive `sd` strictly less than `D_HAT_OVERRIDE`). Plus `readouts.len() == n_active_pairs` (per_pair_readout walks the same active set as `active_pairs` by construction).

### 5. `small_strain_validity`

`0 < ε < SMALL_STRAIN_CEILING = 0.10` (10× cap over expected `ε ≈ 0.6 %`). Catches a regression where the V-3a `(d̂, δ)` override no longer produces a small-strain regime. Captured: `0.598 %` (bit-equivalent to row 14's `EPS_N8`).

### 6. `gross_physics`

`λ_z ∈ (0.5, 1.0)` (cube compresses, not extends; rules out >50% compression as physically implausible at the V-3a `(κ, d̂, δ)` regime); `f_r_total < 0` (soft-side total along outward normal `n = -ẑ`); every `readout.force_on_soft.z < 0` (per-pair sign sanity). Catches sign-flip regressions at gross-physics + per-pair levels before the bound-bracket and accessor-vs-manual gates surface them numerically.

### 7. `force_bound_bracket`

`F_us ≤ |f_r_total| ≤ F_strain` at the equilibrium ε. Inherited from row 14 V-3a — the mixed BC must give a response between the two pure-BC limits (adding lateral constraint stiffens, removing it softens).

| `ε` | `F_us` (N) | `|f_r_total|` (N) | `F_strain` (N) |
|---|---|---|---|
| 5.978e-3 | 0.1674 | **0.1819** | 0.3587 |

`|f_r_total|` sits close to `F_us` (Saint-Venant boundary-layer regime). Equivalently in stiffness-modulus units: `effective_modulus = 3.04e5 Pa ∈ [E = 2.8e5, M_c = 6.0e5]` with `rel_pos_in_bounds ≈ 0.076`.

### 8. `accessor_vs_manual_consistency` (HEADLINE)

`assert_relative_eq!(−Σ readouts.force_on_soft.z, manual_reaction_sum, max_relative=1e-12, epsilon=1e-12)`.

The structural anchor for row 18: the new public surface (`per_pair_readout`) returns the same arithmetic the inline duplication in row 14 produces. Bit-equivalence at the per-pair level is verified by [`tests/penalty_pair_readout.rs`][test] (foundation patch's integration test); this gate verifies it holds at the aggregated-sum level for the V-3a scene. Both walks iterate top-face vertices in the same `pick_vertices_by_predicate` insertion order, so floating-point summation parity is guaranteed at the bit level.

[test]: ../../../sim/L0/soft/tests/penalty_pair_readout.rs

### 9. `per_pair_invariants`

For every readout: `sd < D_HAT_OVERRIDE` (active-band gate); `|force_on_soft.x|` and `|force_on_soft.y|` near zero (`< LATERAL_FORCE_TOL = 1e-15`); `normal` drift from `(0, 0, -1)` near zero (axis-aligned plane); `vertex_id` in valid range; `pair` is the `Vertex` variant (Phase 5 ships only this variant; future IPC will add `EdgeEdge` / `VertexFace`).

### 10. `captured_bits_readout_metrics`

Ten `f64` readout metrics + six `usize` counts captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `lambda_z_avg`, `eps`, `f_r_total` | rel `1e-12`, abs `1e-12` (3 × `f64`; `lambda_z_avg` + `eps` bit-equivalent to row 14 n=8) |
| `mean_force_z`, `max_force_z`, `min_force_z` | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `mean_pressure`, `max_pressure`, `min_pressure` | rel `1e-12`, abs `1e-12` (3 × `f64`) |
| `effective_modulus` | rel `1e-12`, abs `1e-12` (`f64`; bit-equivalent to row 14 n=8) |
| `n_active_pairs`, `iter_count`, `n_tets`, `n_vertices`, `n_loaded`, `n_pinned` | strict equality (6 × `usize`) |

Capture provenance + failure-mode protocol (rule out toolchain drift before re-baking) inline above the const blocks. Same precedent as PR1 rows 6+10+11 + rows 12+13+14. **NEVER re-bake the reference values to make the test green.**

## Visuals

### PLY artifact

`out/contact_force_readout.ply` — finest-refinement (n=8) deformed boundary mesh with per-vertex `contact_pressure` extra:

```text
729 vertices    (finest deformed positions in physics +Z frame; (n+1)³ grid from
                 HandBuiltTetMesh::uniform_block — every vertex referenced by ≥ 1 tet,
                 no BCC-orphan corners)
768 faces       (boundary triangulation via Mesh::boundary_faces — outward-CCW winding
                 from the right-handed-tet `signed_volume > 0` invariant)
normals         : smooth, area-weighted via AttributedMesh::compute_normals
extras["contact_pressure"]
                : f32 per vertex; zero everywhere except 81 active top-face vertices
                  where `pressure = |force_z| / A_per_pair_uniform > 0`. cf-view's
                  auto-colormap (sequential viridis on positive scalar) renders the
                  contact patch as a bright top-face cap; colorbar reads in pressure
                  units (Pa) vs row 14's force_z.
```

**Top-face-only coloring is the intended physics asymmetry, not a viewer bug.** The bottom face is BC-pinned (Dirichlet, via `BoundaryConditions::pinned_vertices`), not penalty-contacted, so it carries no `contact_pressure` scalar. Only top-face vertices enter the d̂-band under the descended top plate; side faces are free surfaces. Sister of the Bevy section's "the bottom plate is purely visual" caveat at the colormap layer.

The interior-vs-corner pressure gradient (~6× from corner 374 Pa to interior 2383 Pa per the Saint-Venant boundary-layer pattern: laterally-constrained interior is locally uniaxial-strain → stiffer → larger penetration; free-edge-adjacent corners are locally uniaxial-stress → softer → smaller penetration) is smoothed out by smooth-normal interpolation across the boundary triangulation — cf-view best surfaces the **engagement boundary** (top vs not-top) and the **mean pressure level**. The per-pair gradient is plot.py's headline (no smoothing — each active pair is a discrete marker).

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/contact-force-readout/out/contact_force_readout.ply
```

### Bevy static-state visualization (CF_VISUAL=1)

```text
CF_VISUAL=1 cargo run -p example-sim-soft-contact-force-readout --release
```

Spawns an `OrbitCamera` scene with four entities (rendered at `RENDER_SCALE = 100×` physics scale per the section below), mirroring row 14's harness verbatim:

- **Soft mesh** (`Mesh3d` + `MeshMaterial3d` + `Trajectory`) — coral PBR cube built from the rest configuration + `Mesh::boundary_faces()` triangulation, animated to the captured n=8 deformed positions on first `step_replay` tick.
- **Bottom plate** (`Mesh3d` + `MeshMaterial3d`) — gray PBR cuboid, flush against the BC-pinned cube bottom at Bevy y=0. **Purely visual** — physics has no penalty contact at the bottom face.
- **Top plate** (`Mesh3d` + `MeshMaterial3d`) — same gray cuboid, flush against the AMPLIFIED cube top (NOT the kinematic `δ`-offset position). Mirror row 14.
- **HUD** (`Text` + `Node`) — top-left, ASCII-only multi-line: per-pair statistics (max/mean/min force_z, max/mean/min pressure), n_active_pairs, ε, λ_z, e_eff, plus the `VIZ_AMPLIFY` disclosure line.

Plus a directional-light at `12 klx` from upper-front-right + per-camera `AmbientLight` at `80 cd/m²` (mirror rows 12 + 13 + 14). Controls overlay sits at bottom-left.

The scene is **static** — single quasi-static step with no temporal evolution. `R` is a no-op for this row. Default camera distance `~3 m` per pattern (l) full-scene-first.

### Controls

| Key | Action |
|---|---|
| **Left-drag** | Orbit camera around the cube |
| **Mouse scroll** | Zoom |
| **Right-drag** | Pan |
| **Close window** | Exit the app |

### Why the rendered scene is `100×` simulation scale

Mirrors rows 12 + 13 + 14 verbatim. Bevy 0.18's pipeline defaults (near plane `0.1 m`, OrbitCamera `min_distance = 0.1 m`, AmbientLight brightness, depth precision) were tuned for human-scale (1 m+) scenes. At cm-scale rendering, the camera approaches the default near plane on any zoom-in and clips the geometry; lifting the rendered scene to meter scale puts everything safely past the defaults. **Headless asserts + JSON + PLY are scale-invariant** — they operate on unscaled physics positions, so this is a visualization-only adjustment.

### Why deformations are amplified by `VIZ_AMPLIFY = 50×`

Banked pattern (b) per row 10's [`feedback_visual_review_is_the_test`][vrt]. V-3a's small-strain regime — `ε ≈ 0.6 %` finest-equilibrium per the bound-bracket gate — produces 60 μm cube compression on a 1 cm physical edge. At `RENDER_SCALE = 100×` the visual cube is 1 m tall and the compression is 6 mm — viewed from the default `~3 m` camera distance, this subtends `~2 mrad ≈ 0.1°`, well below the human visual acuity threshold (`~17 mrad ≈ 1 arc-min`). Row 18 mirrors row 14's `VIZ_AMPLIFY = 50×` verbatim so the cube's compression + lateral Poisson bulge become visually perceptible (`~30 %` apparent z-squish + `~12 %` lateral expansion at `ε = 0.6 %`, `ν = 0.4`).

**Plates are positioned flush against the amplified cube faces** (top plate's bottom face at amplified cube top y; bottom plate's top face at cube bottom y=0; both with a sub-mm z-fight offset) — NOT at the kinematic `δ`-offset positions the physics solver uses. The penalty model's `sd ≈ 9.78 μm` contact-band would inflate to ~5 cm Bevy under amplification and dominate the visual story; the band is an FEM no-penetration-enforcement implementation detail tangential to the row's pedagogy, so it's absorbed into the visualization layer. The amplified scene reads as `~30%` apparent z-compression + `~12%` lateral Poisson bulge with both plates clearly in contact.

**Headless asserts + JSON + PLY are NOT amplified** — they read the true `x_final` from the solver. The HUD's last line declares the amplification factor: `VIZ_AMPLIFY = 50× (cube + plates amplified for visibility; numbers above are TRUE physics)`.

[vrt]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md

### Matplotlib plot

`uv run examples/sim-soft/contact-force-readout/plot.py`

Single-panel top-down 2D scatter of the 81 active top-face vertices in (x, y) mm, with marker color encoding per-pair pressure (Pa, viridis colormap) and marker size encoding per-pair `|force_z|` (mN). The mixed BC produces a non-uniform pressure pattern under the Saint-Venant boundary-layer principle: **interior vertices are laterally surrounded by material on all four sides** → locally uniaxial-strain → vertically stiffer → equilibrium reached at less compliance → vertex stays close to its rest z → larger penetration into the descended plate → larger `|force_z|` and pressure. **Corner / edge vertices are adjacent to free side-faces** → can bulge laterally → locally uniaxial-stress → vertically softer → vertex moves DOWN toward the plate level → smaller penetration → smaller pressure. The visual reads as a uniform-yellow 7×7 interior block framed by darker edge-rows and the four darkest corners (~6× pressure ratio from corners 374 Pa to interior 2383 Pa) — a visual readout of how the cube's free-edge geometry interacts with the descending plate to concentrate vertical load at the laterally-constrained interior.

Per [`feedback_visible_contacts`][vis] — for Tier 4 penalty contact rows, the visual review is real (not collapsed to JSON read).

[vis]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 10 anchor-group names, per-pair statistics table, force-bound bracket, effective modulus summary).

## Run

```text
cargo run -p example-sim-soft-contact-force-readout --release
```

Output: `out/contact_force_readout.ply`, `out/contact_force_readout.json`, and stdout summary.

```text
CF_VISUAL=1 cargo run -p example-sim-soft-contact-force-readout --release
```

Output: same as above + Bevy windowed static-state visualization.

```text
uv run examples/sim-soft/contact-force-readout/plot.py
```

Output: `out/contact_force_readout.png` (top-down contact-patch scatter).

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for this example. Sub-second release-mode total runtime for the single n=8 refinement.

## Cross-references

- **Sister sim-soft examples**: `soft-drop-on-plane` (row 12 — V-5 dynamic-integration drop-and-rest), `hertz-sphere-plane` (row 13 — V-3 Hertzian sphere-plane analytical-comparison), `compressive-block` (row 14 — V-3a bilateral compression with three-refinement Cauchy convergence; closest precedent — row 18 reuses row 14's scene + V-3a-LOCAL `(d̂, δ)` override + bound-bracket gate verbatim, replacing the manual force reconstruction with the public per-pair readout surface).
- **Foundation patch**: `sim/L0/soft/src/contact/penalty.rs` (`PenaltyRigidContact::per_pair_readout`) + `sim/L0/soft/src/contact/mod.rs` (`ContactPairReadout`) + `sim/L0/soft/tests/penalty_pair_readout.rs` (integration test pinning bit-equivalence to `ContactGradient`). Shipped at commit `995fb0bf` as the row 18 prerequisite (pattern (j) foundation patch).
- **Internal-fixture template**: `sim/L0/soft/tests/penalty_compressive_block.rs` (V-3a — penalty-contact bilateral-compression gate; the load-bearing scientific gate of Phase 5 commit 8). Row 18 inherits V-3a's scene + load + BC + DT + (d̂, δ) override verbatim through the row 14 wrap.
- **`SoftScene::compressive_block_on_plane`**: `sim/L0/soft/src/readout/scene.rs:377-454` — `(edge_len, cell_size, displacement, &material_field) -> (mesh, bc, initial, contact)`.
- **`Mesh::boundary_faces`** for `HandBuiltTetMesh`: `sim/L0/soft/src/mesh/hand_built.rs:358-359` — outward-CCW boundary-face cache populated at construction via `boundary_faces_from_topology`.
- **`sim-bevy-soft`**: `sim/L1/sim-bevy-soft/` — Bevy soft-body trajectory replay (single-frame trajectory at row 18 makes this a static-state render). Row 18 is its fourth consumer after rows 12 + 13 + 14.
- **`cf-bevy-common`**: `cf-bevy-common/` — shared Bevy 0.18 helpers (`UpAxis`, `OrbitCamera` + `OrbitCameraPlugin`).
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` — cf-view static-artifact viewer the headless PLY targets.
- **Phase 5 scope memo**: `sim/docs/todo/phase_5_penalty_contact_scope.md` — V-3a spec.
- **Book reference**: Part 4 §00 §00 ("Penalty contact pathology and validation scope") + future calibration-loop chapter (Fork B). Row 18 is the **regression-test artifact** for the per-pair readout surface — the public API row 20's silicone-device E2E will consume to read out per-pair contact forces against a scan-derived `MeshSdf` rigid primitive (no axis-aligned-plane closed-form available).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visible_contacts`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md),
  [`feedback_release_mode_heavy_tests`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md).
