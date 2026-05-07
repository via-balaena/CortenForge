# soft-drop-on-plane

**Drop-and-rest user-facing wrap — soft sphere released above a `RigidPlane`, integrated under gravity until it settles into quiescence on the plane.** A `SphereSdf` body of radius `R = 1 cm` is BCC-meshed via `SdfMeshedTetMesh::from_sdf` at `cell_size = 3 mm`, with rest configuration shifted upward to `(0, 0, RELEASE_HEIGHT = 5 cm)` (zero-strain rigid translation). A single `RigidPlane(+ẑ, offset=0)` at `z = 0` sits in the `−ẑ` half-space; the soft sphere falls into the contact band `d̂ = 1 mm` and one-way `PenaltyRigidContact` (κ = 1e4 N/m, d̂ = 1e-3 m, defaults pinned) bounces it into rest. Gravity (`SolverConfig::gravity_z = -9.81 m/s²`) drives the dynamics; backward-Euler at `dt = 1 ms` damps penalty oscillation `~1300×` per step at the scene's `(κ, m_v)` parameters, so a few hundred post-contact steps drop kinetic energy below the `1 cm/s`-magnitude `KE_REST_THRESHOLD`. `n_steps = 1000` (1 s simulated total) gives `~5×` headroom over the analytic time-to-impact `t_c = sqrt(2 (h-R-d̂) / |g|) / dt ≈ 89 steps`.

The headline new capability vs PR1's user-facing rows is **dynamic integration with one-way penalty contact + rest-state convergence** — the first PR2 example row, opening the soft↔rigid contact arc. Row 12 mirrors `sim/L0/soft/tests/contact_drop_rest.rs` verbatim on constants + scene setup; the contributions vs the internal fixture are: **(a)** deformed-mesh PLY emit at the final settled frame via `Mesh::boundary_faces()` (this row is its first user-facing headless consumer per the C2a foundation commit), **(b)** optional Bevy trajectory replay under `CF_VISUAL=1` (this row is `sim-bevy-soft`'s first consumer per the C2c foundation commit), **(c)** a NEW `com_at_equilibrium_height` gate that the V-5 fixture explicitly omits ("No quantitative bound — mean z is biased by orphan vertices"), here corrected by computing COM over `referenced_vertices` only.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/soft_drop_on_plane.ply` (deformed boundary mesh — 18696 vertices including unreferenced BCC-orphan corners, 648 triangles via `Mesh::boundary_faces()`, no per-vertex scalars).

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The first user-facing example of Tier 4 penalty contact per [`EXAMPLE_INVENTORY.md`][inv] — the canonical `SoftScene::dropping_sphere` + `PenaltyRigidContact` + gravity dynamic-integration scene, generalising PR1's static-equilibrium rows (row 6 `cfg.density = 0` quasi-static, IV-3 / IV-5 `cfg.dt = 1.0 s` static) to **transient-dynamics with active contact**:

```rust
let (mesh, bc, initial, contact) = SoftScene::dropping_sphere(
    RADIUS, CELL_SIZE, RELEASE_HEIGHT, MaterialField::uniform(MU, LAMBDA),
)?; // (1e-2, 3e-3, 5e-2, NH(2e5, 8e5))

let mut cfg = SolverConfig::skeleton();
cfg.dt = 1e-3;            // 1 ms — penalty-resolved transient regime
cfg.gravity_z = -9.81;    // body force
cfg.max_newton_iter = 50; // headroom for penalty-oscillation iters

let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

// 1000-step BE rollout; capture trajectory + step diagnostics.
for step_idx in 0..N_STEPS {
    let step = solver.replay_step(&x_in, &v_in, &theta, DT);
    // ... v_final = (x_final - x_state) / DT; per-step asserts; trajectory.push(x_final.clone()) ...
}
```

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

This is **row 12 of the sim-soft examples arc** — the first Tier 4 penalty-contact example, the first PR2 example row, and the first user-facing demo of `sim-bevy-soft` (under `CF_VISUAL=1`). Row 12 is the canonical V-5 hygiene gate (`sim/L0/soft/tests/contact_drop_rest.rs`), exposed as a user-facing demo at the V-5 canonical refinement (`cell_size = 3 mm`).

**Why `cfg.dt = 1 ms`.** PR1's static-regime idiom (`cfg.dt = 1.0 s` or `cfg.density = 0`) collapses the inertial term `M / dt²` so a single `replay_step` from rest converges to the static equilibrium. Row 12 is the inverse — the dynamics ARE the headline, so `dt` must resolve the integration timescale. At penalty stiffness `κ = 1e4 N/m`, the sphere's per-vertex lumped mass `m_v ≈ M / n_referenced ≈ 7.7e-6 kg` puts the penalty oscillator at `ω = sqrt(κ / m_v) ≈ 3.6e4 rad/s`. Backward-Euler at `dt = 1 ms` doesn't resolve the `~0.17 ms` oscillation period (under-resolution gives `dt × ω / (2π) ≈ 5.8` periods per step, i.e., gross dynamics captured + fine penalty-oscillation structure unresolved — permitted as a documented behavior, not a failure). What BE *does* deliver at this `(κ, m_v, dt)` is per-step amplitude factor `1 / (1 + ω²·dt²) ≈ 7.6e-4` — penalty oscillation amplitude shrinks `~1300×` per step in the post-contact regime, so a few hundred post-contact steps brings the system to rest.

**Why `RELEASE_HEIGHT = 5 cm` (5× the contact-band-clear threshold).** The `dropping_sphere` constructor's panic contract requires `release_height > radius + d̂ = 1.1 cm` (sphere starts clear of the contact band, Newton's first iteration sees zero active pairs, dynamics are pure free-fall until contact). `5×` the floor gives a long freefall trajectory before contact, exercising gravity wiring across many steps before the contact dispatch fires — analytic time-to-impact-of-sphere-bottom in pure freefall is `t_c = sqrt(2 (h-R-d̂) / |g|) ≈ 89 ms` ≈ step 89 (captured `n_step_first_contact = 88` sits one step early under sub-step interpolation as penalty's bounded-oscillation onset begins).

**Why no Bevy in headless asserts.** Per inventory iter-7 Q5 lock, the Bevy app is **opt-in only** — headless asserts + PLY emit always run, `CF_VISUAL=1` additionally spawns the Bevy trajectory replay. CI runs the binary without `CF_VISUAL`, so no display / winit / OpenGL is required for the green-build gate. Row 12 sets the `CF_VISUAL=1` convention for sim-bevy-soft consumers; rows 13 / 14 / 18 inherit. Documented in [`examples/EXAMPLES.md`][examples-md].

[examples-md]: ../../EXAMPLES.md

**Why no per-vertex scalars in the PLY.** At quiescence `|v| ≈ 0` everywhere by construction, so a velocity colormap would render FP-noise. Contact-band membership (which vertices are in `d̂`) is a row-13 Hertz-patch concern where contact patch IS the headline. Row 12 ships the deformed boundary mesh with positions only — cf-view's PBR shading on the smooth normals computed by `AttributedMesh::compute_normals` reads the settle's barely-perceptible flattening at the contact zone clearly enough.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 9 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `geometry_invariants`

Compile-time `const { assert!(...) }` on:

| Anchor | Bound |
|---|---|
| `RADIUS > 0`, `CELL_SIZE > 0`, `DT > 0` | strict positivity |
| `RELEASE_HEIGHT > RADIUS + D_HAT` | `dropping_sphere` panic-contract sphere-clear-of-band |
| `N_STEPS > 0`, `MAX_NEWTON_ITER > 0` | strict positivity |
| `KE_REST_THRESHOLD > 0`, `D_HAT > 0` | strict positivity |
| `MU > 0`, `LAMBDA > 0` | NeoHookean Lamé positivity |
| `GRAVITY < 0` | downward direction |
| `COM_TOLERANCE >= 2 × D_HAT` | absorbs Hertz indentation under self-weight |
| `ENERGY_BOUND_SAFETY > 1.0` | allows penalty oscillation overshoot |

`SoftScene::dropping_sphere`'s runtime panic invariants re-asserted at the user-facing example layer at compile time; plus the row-specific `COM_TOLERANCE` / `ENERGY_BOUND_SAFETY` / `GRAVITY < 0` contracts.

### 2. `mesh_topology_exact`

| Anchor | Bound |
|---|---|
| `mesh.n_tets` | exact-pin `2208` |
| `mesh.n_vertices` | exact-pin `18696` (BCC corners + warp-snapped + cut-points) |
| `referenced_vertices(&mesh).len()` | exact-pin `561` |
| `referenced.len() < mesh.n_vertices` | orphan-rejection invariant non-vacuous |

Counts captured 2026-05-06 on the row 12 capture platform per the III-1 determinism contract. The 96% orphan ratio at this `(R, h)` is a BCC + bbox-margin artifact (the mesher allocates corners for the full 6-cell-radius bbox per `SPHERE_BBOX_MARGIN_RATIO`, then references only the surface-cut tets); orphan auto-pin at `CpuNewtonSolver::new` keeps the free-DOF Hessian SPD without touching the orphan cohort.

### 3. `boundary_partition`

| Anchor | Bound |
|---|---|
| `bc.pinned_vertices.is_empty()` | `true` (free-flight) |
| `bc.loaded_vertices.is_empty()` | `true` (gravity is solver-level body force) |

The `dropping_sphere` helper builds an empty `BoundaryConditions` — orphan auto-pin by `CpuNewtonSolver::new`'s `effective_pinned` step is the only Dirichlet constraint, and the mass-driven `M / dt²` Tikhonov regulariser keeps the free-DOF Hessian SPD without an equator pin. Distinct from V-1 / V-3a / V-3 / IV-5 which use explicit pinned + loaded sets.

### 4. `solver_per_step_invariants`

For every step (mirrors V-5):

| Anchor | Bound |
|---|---|
| `x_final[i].is_finite()` per DOF | `true` (no NaN / inf — Newton blowup masked by Armijo) |
| `iter_count` per step | `≤ MAX_NEWTON_ITER = 50` |
| `\|v\|_max` per step | `< sqrt(2 g h) × ENERGY_BOUND_SAFETY = 1.5 × 0.99 m/s ≈ 1.49 m/s` |

The energy gate catches sign-flipped gravity, inverted contact gradient, or runaway Newton step that would explode `|v|` by orders of magnitude. Penalty's bounded oscillation produces overshoot within the safety margin (BE damping pulls envelope down each step). Captured peak: `|v|_max ≈ 0.86 m/s` (penalty-oscillation onset at impact).

### 5. `contact_engagement`

| Anchor | Bound |
|---|---|
| `n_step_first_contact > 0` | sphere did not start in contact band |
| `n_step_first_contact ≤ N_STEPS / 4 = 250` | sphere reached the plane within ~2.8× the analytic freefall step |

Catches a sphere-flying-sideways or gravity-magnitude regression that wouldn't trip the energy gate. Analytic time-for-sphere-bottom-to-band: `t_c = sqrt(2 (h-R-d̂) / |g|) / dt ≈ 89 steps`; observed `88` (one step early under sub-step interpolation as penalty's bounded-oscillation onset begins). The `N_STEPS / 4 = 250` upper bound has `2.8×` headroom.

### 6. `reaches_rest`

| Anchor | Bound |
|---|---|
| `final \|v\|_max` | `< KE_REST_THRESHOLD = 1 cm/s` |

Mirror V-5. Backward-Euler is dissipative; numerical damping carries the system to rest with the high per-step damping fraction at our `(κ, m_v, dt)`. Failure suggests either BE damping rate is much lower than expected, `dt × N_STEPS` doesn't cover impact + decay, or penalty oscillation amplitude is not decaying. Captured `1.187e-4 m/s` — `~84×` headroom over the threshold.

### 7. `com_descended`

| Anchor | Bound |
|---|---|
| `final mean_z` | strict `<` `initial mean_z` |

Mirror V-5. Both means computed over `referenced_vertices` (orphan auto-pin makes referenced-only mean meaningful — full-mesh mean would be biased toward the static auto-pinned cohort at `initial_mean_z + small_shift`). Sanity gate against frozen-x bug (solver returns `x_prev` unmodified).

### 8. `com_at_equilibrium_height`

**NEW — fills the V-5 gap-to-land per inventory row 12 spec.** V-5 explicitly omits a quantitative bound on COM-at-rest because mean-z is biased by orphan vertices; computing over `referenced_vertices` corrects this and lets us assert the settled height directly.

| Anchor | Bound |
|---|---|
| `\|com_z_final - (R + D_HAT)\|` | `< COM_TOLERANCE = 2 × D_HAT = 2 mm` |

The settled sphere center sits at `R + d̂ = 1.1 cm` above the plane (sphere bottom just below the band's upper edge; surface deformed to wrap the band's width). Hertz self-weight indentation at Ecoflex stiffness `E ≈ 600 kPa` and `m·g ≈ 4.24e-2 N` over `~mm` contact radius is `O(10s of µm)` — well inside the `2 mm` floor. The `2 mm` bound catches a sphere-at-wrong-height regression (gravity sign flip, dt under-resolves freefall, plane-offset misconfigured) without flake from the Hertz floor. Tighter contact-mechanics gates belong at row 13 (Hertz) where contact patch IS the headline. Captured `|Δ| = 1.31e-4 m` (~0.13 mm) — `~15×` headroom over `COM_TOLERANCE`.

### 9. `captured_bits_drop_metrics`

Five drop-metric scalars captured under the IV-1 sparse-tier rel-tol contract:

| Anchor | Bound |
|---|---|
| `final_v_max` vs `FINAL_V_MAX_REF_BITS` | rel `1e-12`, abs `1e-12` |
| `final_mean_z` vs `COM_Z_AT_REST_REF_BITS` | rel `1e-12`, abs `1e-12` |
| `com_descent` vs `COM_DESCENT_REF_BITS` | rel `1e-12`, abs `1e-12` |
| `n_step_first_contact` vs `N_STEP_FIRST_CONTACT_REF` | strict equality (usize) |
| `max_iter_observed` vs `MAX_ITER_OBSERVED_REF` | strict equality (usize) |

`~2k` tets through faer's sparse Cholesky lives between IV-1's dense bit-equal tier (12-24 DOFs) and IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift on faer's per-column FMA-fusion path); 1 ms integrator step layered on top adds another arithmetic stage. The contract is **relative tolerance, not strict bit-equality** for the floats — observed bits captured for regression detection, compared via `assert_relative_eq!` at `1e-12` rel. Capture provenance + failure-mode protocol IV-1-protocol-verbatim above the const blocks (do NOT re-bake without diagnosing toolchain vs real regression first).

This is the row's **tight regression-detection gate** — anchors 4-8 document physical reasonableness; anchor 9's `1e-12` rel against pinned bits catches any FEM-numerics-shifting regression at machine precision.

## Visuals

### PLY artifact

`out/soft_drop_on_plane.ply` — final-frame deformed boundary mesh (the canonical visual artifact for the headless review pass).

```text
18696 vertices (final-frame deformed positions in physics +Z frame; 18135 are unreferenced
                BCC-orphan corners staying frozen at their initial release-height
                positions, the 561 referenced-vertex cohort carries the actual
                settled sphere geometry)
648 faces      (boundary triangulation via Mesh::boundary_faces — outward-CCW winding
                from the right-handed-tet `signed_volume > 0` invariant per
                boundary_faces_from_topology)
normals        : smooth, area-weighted via AttributedMesh::compute_normals
no extras      : at quiescence |v| ≈ 0; no per-vertex scalars surface a meaningful field
```

**cf-view command:**

```text
cargo run -p cf-viewer --release -- examples/sim-soft/soft-drop-on-plane/out/soft_drop_on_plane.ply
```

cf-view renders the smoothly shaded boundary mesh; the settled sphere reads as a slightly flattened sphere sitting on the (implicit, not rendered in this static artifact) plane. The unreferenced orphan cohort sits at the rest configuration shifted to release-height — they appear as scattered floating vertices in the bbox above the settled sphere, but only the boundary-faces triangles are drawn (the orphans are not topologically connected by any face), so visual review focuses on the settled-sphere geometry.

### Bevy replay (CF_VISUAL=1)

```text
CF_VISUAL=1 cargo run -p example-sim-soft-soft-drop-on-plane --release
```

Spawns an `OrbitCamera` scene with three entities (rendered at `RENDER_SCALE = 100×` physics scale per the section below):

- **Soft mesh** (`Mesh3d` + `MeshMaterial3d` + `Trajectory`) — a coral PBR sphere built via `sim_bevy_soft::mesh::build_soft_mesh` from the rest configuration + `Mesh::boundary_faces()` triangulation, animated by `sim_bevy_soft::trajectory::step_replay` reading the captured 1000-frame trajectory each frame. Carries `Transform::from_scale(100)` so the cm-scale physics positions render at meter scale.
- **Rigid plane** (`Mesh3d` + `MeshMaterial3d`) — `8 m × 8 m` total (`4 m` half-size on each axis per Bevy's `Plane3d::new`, sized by `4 × R × RENDER_SCALE = 4 m`) gray PBR quad at Bevy `y = 0` (= physics `z = 0` under `UpAxis::PlusZ`'s `(x,y,z) → (x,z,y)` swap), normal `+Y` Bevy. Static; no `Trajectory`.
- **Camera** (`Camera3d` + `OrbitCamera`) — target near rest COM `(0, R + d̂, 0)` Bevy scaled by `RENDER_SCALE` to `(0, 1.1, 0)` Bevy, distance `15 m` (~3× release_height for full trajectory bbox visible), angles `(0.6, 0.4) rad` (~34° azimuth, 23° elevation). Per-camera `AmbientLight` Component (Bevy 0.18 moved AmbientLight from a global Resource to a per-camera Component — `#[require(Camera)]`) at `80 cd/m²`.

Plus a separate directional-light entity at `12 klx` illuminance, oriented from upper-front-right via `Transform::from_xyz(0.5, 1.0, 0.5).looking_at(Vec3::ZERO, Vec3::Y)`.

Replay runs at `SLOW_MO_FACTOR = 10×` slow-motion: 1000 frames × `dt = SLOW_MO_FACTOR × DT = 10 ms` per replay frame = **10.000 s replay duration** for the 1.000 s simulated trajectory. The default is slow-motion because the analytic time-to-impact `t_c ≈ 89 ms` is blink-and-miss-it at 1× wall-clock — `10×` puts the freefall + contact-onset arc at `~890 ms` (clearly observable, contact-pair onset reads as a distinct beat) while the settle-and-rest phase fits under `9 s`. Pure visualization knob; has no effect on the headless asserts or the captured PLY.

The trajectory's playback clock is captured per-entity via `sim_bevy_soft::trajectory::ReplayEpoch` at the first `step_replay` tick — NOT at `App::new()` instantiation. This isolates the trajectory's playback budget from `DefaultPlugins` startup time (winit + render-pipeline init, `~1-2 s` on first run on Apple Silicon), which would otherwise consume the replay window before the user sees the first frame. (sim-bevy-soft fix: previously `step_replay` used `Time<Real>::elapsed_secs_f64()` from app start, which clamped row 12's first visual review to the final settled frame before the window became visible.)

### Controls

| Key | Action |
|---|---|
| **`R`** | Reset and replay from frame 0 (clears `ReplayEpoch.epoch_secs` so the next `step_replay` tick captures a fresh epoch — see [`sim_bevy_soft::trajectory::reset_replay_on_keypress`](../../../sim/L1/sim-bevy-soft/src/trajectory.rs)) |
| **Left-drag** | Orbit camera around the soft body |
| **Mouse scroll** | Zoom |
| **Right-drag** | Pan |
| **Close window** | Exit the app |

### Why the rendered scene is `100×` simulation scale

The visual entities (soft mesh, plane, camera target, camera distance) are rendered at `RENDER_SCALE = 100×` the physics scale via a `Transform::from_scale` on the soft mesh + correspondingly scaled plane half-size and camera distance. Rendered sphere is `1 m` radius (instead of `1 cm`), plane is `4 m` half-size (instead of `4 cm`), camera distance is `15 m` (instead of `15 cm`). Bevy 0.18's pipeline defaults were tuned for human-scale (1 m+) scenes — at cm-scale the camera approaches the default `0.1 m` near plane on any zoom-in (geometry past the near plane gets clipped, producing a hole through the sphere or worse, the entire sphere disappears). Lifting the rendered scene to meter scale puts everything safely past the defaults. Headless asserts + PLY emit are scale-invariant — they operate on the unscaled physics positions — so this is a visualization-only adjustment.

The replay clamps at end (no looping); press `R` to watch again, or close the window to exit. To tune the replay rate edit `SLOW_MO_FACTOR` in `src/main.rs` (`1.0` = real-time, larger = slower). No in-app pause/scrub controls — defer to a future row that needs them per `sim_bevy_soft::trajectory::step_replay` docs.

Per [`feedback_visible_contacts`][vis] — for Tier 4 penalty contact rows, the visual review is real (not collapsed to JSON read).

[vis]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 9 anchor-group names, |v|_max trace summary, COM trace, contact-engagement step + analytic).

## Run

```text
cargo run -p example-sim-soft-soft-drop-on-plane --release
```

Output: `out/soft_drop_on_plane.ply` and stdout summary.

```text
CF_VISUAL=1 cargo run -p example-sim-soft-soft-drop-on-plane --release
```

Output: same as above + Bevy windowed trajectory replay.

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The exact-pin counts and captured drop-metric bits were captured under release-mode build; matching that invocation shape removes one variable from the determinism contract.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Cross-references

- **Sister sim-soft examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait contract on `SphereSdf`, the SDF body row 12's `dropping_sphere` constructor uses); `sdf-to-tet-sphere` (row 3 — single-material `SdfMeshedTetMesh::from_sdf` on a solid sphere, the FEM-meshing pipeline row 12 reuses); future `hertz-sphere-plane` (row 13 — Hertz analytic gate at the same sphere-on-plane geometry, contact-patch IS the headline) and `compressive-block` (row 14 — V-3a compressive-load variant at the rigid-plane interface).
- **Internal-fixture template**: `sim/L0/soft/tests/contact_drop_rest.rs` (V-5 — drop-and-rest hygiene gate at three-property structure: per-step energy bound, no NaN, reaches rest within budget, sphere descended). Row 12 mirrors V-5's scene + load + BC + DT + N_STEPS exactly; the per-step + final asserts mirror V-5 verbatim. The NEW `com_at_equilibrium_height` gate fills V-5's explicit "no quantitative bound on COM" gap by computing the mean over `referenced_vertices` only (V-5's `mean_referenced_z` helper, here adapted to assert against `R + d̂ ± COM_TOLERANCE`).
- **`SoftScene::dropping_sphere`**: `sim/L0/soft/src/readout/scene.rs:753-817` — `(radius, cell_size, release_height, material_field) -> (mesh, bc, initial, contact)`. `SphereSdf` body BCC-meshed at `cell_size`, rest-config positions shifted upward by `release_height`, single `RigidPlane(+ẑ, offset=0)` packed into `PenaltyRigidContact::new` with defaults pinned.
- **`Mesh::boundary_faces`**: `sim/L0/soft/src/mesh/mod.rs:113-132` — outward-CCW boundary-face cache populated at construction via `boundary_faces_from_topology`. Row 12 is its first user-facing headless consumer (PR2 C2a foundation commit shipped this trait method).
- **`sim-bevy-soft`**: `sim/L1/sim-bevy-soft/` — Bevy soft-body trajectory replay. Row 12 is its first consumer (PR2 C2c foundation commit shipped the skeleton crate). Module structure: `mesh::build_soft_mesh` + `mesh::apply_soft_positions` (per-frame position + smooth-normal write); `trajectory::Trajectory` Component + `trajectory::step_replay` system; `plugin::SoftBodyVisualPlugin` wires step_replay into Update.
- **`cf-bevy-common`**: `cf-bevy-common/` — shared Bevy 0.18 helpers (`UpAxis` input → Bevy frame swap, `OrbitCamera` + `OrbitCameraPlugin`). Row 12 consumes both via the `BevyVec3` / `BevyMesh` aliases (sim-soft's `Vec3` / `Mesh` shadow Bevy's prelude under workspace-default re-exports).
- **VIEWER_DESIGN.md**: `docs/VIEWER_DESIGN.md` — cf-view static-artifact viewer the headless PLY targets.
- **Book reference**: Part 4 §00 §00 ("Penalty contact pathology and validation scope"); Part 5 §00 §00 ("Backward-Euler dissipation in transient FEM"). Row 12 is the **regression-test artifact** for the rigid-soft penalty-contact-with-gravity chapter section — V-5's three-property hygiene gate, exposed user-facing at the example layer, plus the COM-at-equilibrium quantitative bound that V-5 explicitly omits.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visible_contacts`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visible_contacts.md),
  [`feedback_visual_review_is_the_test`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_review_is_the_test.md).
