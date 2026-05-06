//! soft-drop-on-plane — Phase 5 V-5 user-facing wrap: soft sphere released
//! above a `RigidPlane`, integrated under gravity until it settles into
//! quiescence on the plane.
//!
//! `SoftScene::dropping_sphere(radius, cell_size, release_height,
//! material_field)` (sim-soft `c3729d4a` per Phase 5 commit-6 scaffolding +
//! commit-10 gravity wiring) is the production scene constructor — a
//! `SphereSdf` body BCC-meshed via `SdfMeshedTetMesh::from_sdf` at
//! `cell_size = 3 mm`, with rest configuration shifted upward to `(0, 0,
//! release_height)` (zero-strain rigid translation). A single
//! `RigidPlane(+ẑ, offset=0)` at `z = 0` sits in the `−ẑ` half-space; the
//! soft sphere falls into the contact band `d̂ = 1 mm` and one-way
//! [`PenaltyRigidContact`](sim_soft::PenaltyRigidContact) (κ = 1e4 N/m, d̂ = 1e-3 m, defaults pinned) bounces
//! it into rest. Gravity (`SolverConfig::gravity_z = -9.81 m/s²`) drives
//! the dynamics; backward-Euler at `dt = 1 ms` damps penalty oscillation
//! `1300×` per step at the `(κ, m_v)` parameters, so a few hundred
//! post-contact steps drop kinetic energy below the `1 cm/s`-magnitude
//! `KE_REST_THRESHOLD`. `n_steps = 1000` (1 s simulated total) gives
//! ample headroom over the analytic time-to-impact for the sphere
//! bottom to enter the contact band: `t_c = sqrt(2 (h-R-d̂) / |g|) ≈
//! 89 ms ≈ step 89`.
//!
//! The headline new capability vs PR1's user-facing rows is **dynamic
//! integration with one-way penalty contact + rest-state convergence** —
//! the first PR2 example row, opening the soft↔rigid contact arc. Row 12
//! mirrors `sim/L0/soft/tests/contact_drop_rest.rs` (V-5) verbatim on
//! constants + scene setup; the contributions vs the internal fixture
//! are: (a) deformed-mesh PLY emit at the final settled frame via
//! [`Mesh::boundary_faces`] (this row is its first user-facing headless
//! consumer), (b) optional Bevy trajectory replay under `CF_VISUAL=1`
//! (this row is sim-bevy-soft's first consumer), (c) a NEW
//! [`com_at_equilibrium_height`](verify_com_at_equilibrium_height) gate
//! that the V-5 fixture explicitly omits ("No quantitative bound — mean
//! z is biased by orphan vertices"), here corrected by computing COM
//! over `referenced_vertices` only.
//!
//! ## Visual-mode opt-in: `CF_VISUAL=1`
//!
//! Headless asserts + PLY emit ALWAYS run. Setting `CF_VISUAL=1` (any
//! non-empty value) additionally spawns a Bevy app that replays the
//! captured 1000-frame trajectory at `SLOW_MO_FACTOR = 10×` slow-motion
//! (10 s replay duration); the replay clamps at end (no looping).
//! Slow-motion is the default because the analytic 89 ms freefall +
//! contact onset is blink-and-miss-it at 1× wall-clock — `10×` puts
//! the freefall arc at `~890 ms` (clearly observable, contact-onset
//! reads as a distinct beat) while keeping the settle-and-rest phase
//! under `9 s`. The trajectory's playback clock starts at the first
//! [`step_replay`](sim_bevy_soft::trajectory::step_replay) tick
//! (per-entity [`ReplayEpoch`](sim_bevy_soft::trajectory::ReplayEpoch)
//! capture), NOT at app start, so `DefaultPlugins` startup time does
//! not consume playback budget.
//!
//! ### Controls
//!
//! - **`R`** — reset and replay from frame 0
//!   ([`reset_replay_on_keypress`](sim_bevy_soft::trajectory::reset_replay_on_keypress)).
//! - **Left-drag / scroll / right-drag** — orbit / zoom / pan (via
//!   [`OrbitCameraPlugin`]).
//! - **Close window** — exit.
//!
//! ### Why the rendered scene is `100×` simulation scale
//!
//! The visual entities (soft mesh, plane, camera target, camera
//! distance) are rendered at `RENDER_SCALE = 100×` the physics scale
//! via a `Transform::from_scale` on the soft mesh + corresponding
//! scaled plane half-size and camera distance. Rendered sphere is
//! `1 m` radius (instead of `1 cm`), plane is `4 m` half-size (instead
//! of `4 cm`), camera distance is `15 m` (instead of `15 cm`).
//! Reason: Bevy 0.18's pipeline defaults — near plane `0.1 m`,
//! OrbitCamera `min_distance = 0.1 m`, AmbientLight brightness, depth
//! precision — were tuned for human-scale (1 m+) scenes. At cm-scale
//! rendering the camera approaches the near plane on any zoom-in
//! (geometry past `0.1 m` from camera gets clipped, producing a hole
//! through the sphere or, at extreme zoom, the entire sphere
//! disappearing). Lifting the rendered scene to meter scale puts
//! everything safely past the defaults. Headless asserts + PLY emit
//! are scale-invariant — they operate on the unscaled physics
//! positions in the captured `DropSnapshot`, so this is a
//! visualization-only adjustment.
//!
//! (Separately, sim-bevy-soft `cc2e6324` swapped Bevy's
//! `Mesh::compute_smooth_normals` for `compute_area_weighted_normals`
//! to dodge a Bevy 0.18 issue where the corner-angle-weighted variant
//! has a hardcoded `EPS = f32::EPSILON` gate that zeros out per-vertex
//! weights at sub-mm edge lengths. That fix is in sim-bevy-soft
//! itself and independent of the `100×` render-scale lift here.)
//!
//! Camera target = rest COM (physics `(0, 0, R + d̂)` → Bevy
//! `(0, R + d̂, 0)` under `UpAxis::PlusZ`'s physics-`(x, y, z)` →
//! Bevy-`(x, z, y)` swap), scaled into Bevy-frame meters by
//! `RENDER_SCALE` so target lands at `(0, 1.1, 0)` Bevy (vs
//! `(0, 0.011, 0)` at 1×). Distance `15 m` (~3× release_height for
//! full trajectory bbox visible), angles `(0.6, 0.4) rad`
//! (~34° azimuth, ~23° elevation). Plane is a `4 m × 4 m` half-size
//! gray PBR quad at Bevy `y = 0` (= physics `z = 0`), normal `+Y`
//! Bevy. To tune the replay rate edit [`SLOW_MO_FACTOR`] (`1.0` =
//! real-time, larger = slower); pause/scrub controls are out of scope
//! (defer to a future row that wants them — see
//! `sim_bevy_soft::trajectory::step_replay` docs).
//!
//! Row 12 sets the `CF_VISUAL=1` convention for sim-bevy-soft consumers
//! (rows 13 / 14 / 18 inherit). Documented in `examples/EXAMPLES.md`.
//!
//! ## PLY artifact
//!
//! `out/soft_drop_on_plane.ply` — final-frame deformed boundary mesh,
//! triangulated via [`Mesh::boundary_faces`]'s outward-CCW winding (the
//! right-handed-tet `signed_volume > 0` invariant gives outward faces
//! topologically). No per-vertex scalars: at quiescence `|v| ≈ 0`
//! everywhere by construction so a velocity colormap would be FP-noise;
//! contact-band membership is a row-13 Hertz-patch concern. Open in
//! cf-view: `cargo run -p cf-viewer --release -- <path>`.
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`geometry_invariants`** — compile-time `const { assert!(...) }` on
//!   `RADIUS > 0`, `CELL_SIZE > 0`, `RELEASE_HEIGHT > RADIUS + D_HAT`
//!   (sphere starts clear of contact band per `dropping_sphere` panic
//!   contract), `DT > 0`, `N_STEPS > 0`, `KE_REST_THRESHOLD > 0`,
//!   `MU > 0`, `LAMBDA > 0`.
//! - **`mesh_topology_exact`** — `mesh.n_tets`, `mesh.n_vertices`,
//!   `referenced_vertices(&mesh).len()` exact-pinned per the III-1
//!   determinism contract at `cell_size = 3 mm` on the canonical
//!   `SphereSdf` body.
//! - **`boundary_partition`** — `bc.pinned_vertices.is_empty()` AND
//!   `bc.loaded_vertices.is_empty()`. Free-flight scene; orphan auto-pin
//!   by `CpuNewtonSolver::new`'s `effective_pinned` step is the only
//!   Dirichlet constraint, and the mass-driven `M / dt²` Tikhonov
//!   regulariser keeps the free-DOF Hessian SPD without an equator pin.
//! - **`solver_per_step_invariants`** — for every step (mirrors V-5):
//!   no NaN in `x_final`; `iter_count ≤ MAX_NEWTON_ITER = 50`; per-step
//!   `|v|_max < sqrt(2 g h) × ENERGY_BOUND_SAFETY = 1.5` m/s (no energy
//!   injection on top of the gravitational-freefall bound + penalty's
//!   bounded-oscillation overshoot per scope memo §1 V-5).
//! - **`contact_engagement`** — at least one step `k ∈ [0, N_STEPS / 4]`
//!   has the sphere bottom in the contact band (`min_z(frame[k]) < D_HAT`).
//!   Catches a sphere-flying-sideways or gravity-magnitude regression that
//!   wouldn't trip the energy gate. Analytic time-for-sphere-bottom-to-band
//!   in pure freefall is `t_c = sqrt(2 (h-R-d̂) / |g|) ≈ 89 ms ≈ step 89`
//!   (the bottom falls `RELEASE_HEIGHT - R - d̂ ≈ 3.9 cm` before crossing
//!   into d̂); the `N_STEPS / 4 = 250` upper bound gives `~2.8×` headroom.
//! - **`reaches_rest`** — final `|v|_max < KE_REST_THRESHOLD = 1 cm/s`
//!   (mirrors V-5).
//! - **`com_descended`** — final referenced-vertex mean-z `<` initial
//!   referenced-vertex mean-z (mirrors V-5; orphan auto-pin makes
//!   referenced-only mean meaningful, full-mesh mean would be biased
//!   downward by the static auto-pinned cohort).
//! - **`com_at_equilibrium_height`** — NEW: `|com_z_final - (R + D_HAT)|
//!   < COM_TOLERANCE = 2 × D_HAT = 2 mm`. The settled sphere center sits
//!   at `R + d̂` above the plane (sphere bottom in contact band, surface
//!   at `z = d̂`); Hertz indentation under self-weight at Ecoflex
//!   stiffness is `O(10s of µm)` — well inside the `2 mm` floor. Tighter
//!   contact-mechanics analytics belong at row 13 (Hertz) where contact
//!   patch IS the headline.
//! - **`captured_bits_drop_metrics`** — five drop-metric scalars captured
//!   under the IV-1 sparse-tier rel-tol contract (`1e-12` rel, `1e-12`
//!   abs floor). `final_v_max`, `com_z_at_rest`, `com_descent`,
//!   `n_step_first_contact`, `max_iter_observed` — bit-pinned at first run
//!   for cross-platform regression detection.

// PLY field-data is single-precision on disk; converting f64 quantities
// to f32 for the AttributedMesh emit is intrinsic to the PLY format. Same
// precedent as PR1 rows 1+2+3+8+9+10+11.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (~2k here, ≪ u32::MAX) — the
// standard Mesh-trait API tax mirrored across the workspace. Also covers
// `step_idx as f64` for the analytic time-to-impact comparison.
#![allow(clippy::cast_possible_wrap)]
// `usize as f64` casts for averaging (referenced_vertices.len(), step
// counts). Counts ≤ ~1k here, well within f64 mantissa exact range.
#![allow(clippy::cast_precision_loss)]
// `dropping_sphere(...).expect(...)` on the helper signature. Mirror of
// `concentric_lame_shells.rs` precedent.
#![allow(clippy::expect_used)]
// `print_summary` is a single museum-plaque stdout writer; splitting into
// sub-helpers fragments the visual format without information gain. Same
// allowance as PR1 rows 4 + 5 + 6 + 9 + 10 + 11.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars-and-collections; threading them
// through a struct adds indirection without information gain. Same
// allowance as row 11.
#![allow(clippy::too_many_arguments)]
// `doc_markdown` flags Unicode math notation (`σ`, `κ`, `λ`, `μ`) as if
// they were unbacktrick-quoted code identifiers. Same allowance as PR1
// rows 5+6+8+9+10+11.
#![allow(clippy::doc_markdown)]
// Bevy systems take Resources by value or by `Res<T>`, both flagged as
// pass-by-value. Same allowance as `sim/L1/sim-bevy-soft/src/trajectory.rs`.
#![allow(clippy::needless_pass_by_value)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
// `bevy::prelude::Vec3` (f32 3-vec, Bevy math) vs `sim_soft::Vec3`
// (nalgebra f64 3-vec) collide; same for `bevy::prelude::Mesh` (asset
// struct) vs `sim_soft::Mesh` (tet-mesh trait). Alias both Bevy items so
// the physics-side imports below stay ergonomic for solver inputs.
use bevy::prelude::Mesh as BevyMesh;
use bevy::prelude::Vec3 as BevyVec3;
use bevy::prelude::*;
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_bevy_soft::mesh::build_soft_mesh;
use sim_bevy_soft::plugin::SoftBodyVisualPlugin;
use sim_bevy_soft::trajectory::Trajectory;
use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, MaterialField, Mesh, NewtonStep, PenaltyRigidContactSolver, SceneInitial,
    SdfMeshedTetMesh, SoftScene, Solver, SolverConfig, Tet4, Vec3, VertexId, referenced_vertices,
};

// =============================================================================
// Scene constants — mirror V-5 (`sim/L0/soft/tests/contact_drop_rest.rs`)
// verbatim. Re-deriving here keeps the example self-contained AND
// captures-platform-locked; any regression in the V-5 helper that shifts
// these implicitly would surface at first sim-bevy-soft visual review.
// =============================================================================

/// Sphere radius (1 cm). Mirror V-5's `RADIUS` for parameter consistency
/// across the contact-active regression net (V-3 / V-3a / V-5 share the
/// 1-cm sphere).
const RADIUS: f64 = 1.0e-2;

/// BCC cell size (3 mm). Mirror V-5's `CELL_SIZE` — release-mode-feasible
/// step latency (~22 s for 1000 steps × dynamic Newton) at the scene's
/// `(κ, m_v)`. Finer resolution is gratuitous for V-5's hygiene scope and
/// row 12's quiescence-on-plane scope; if visual review surfaces a
/// "too-chunky-sphere" finding (~hundreds-of-triangles boundary at this
/// `(R, h)`), tune to 2 mm in a follow-up commit and recapture
/// `N_TETS_EXACT` + the IV-1 drop-metric bits.
const CELL_SIZE: f64 = 3.0e-3;

/// Release height above the rigid plane (5 cm). Must exceed `RADIUS +
/// D_HAT = 1.1 cm` per `SoftScene::dropping_sphere` panic contract; `5×`
/// the contact-band-clear threshold gives a long freefall trajectory
/// before contact, exercising gravity wiring across many steps before
/// the contact dispatch fires.
const RELEASE_HEIGHT: f64 = 5.0e-2;

/// Lamé pair `(μ, λ)` — mirror V-5 (Ecoflex 00-30 + 15 wt% carbon-black
/// composite, `ν = 0.4` compressible Neo-Hookean per Phase 4 IV-3 / IV-5
/// + V-3 precedent).
const MU: f64 = 2.0e5;
const LAMBDA: f64 = 8.0e5;

/// Gravitational acceleration along `+ẑ` (`m/s²`). Negative = downward.
/// Earth standard `9.81 m/s²` per scope memo §1 V-5's "gravity loaded"
/// framing.
const GRAVITY: f64 = -9.81;

/// Time step (1 ms). At penalty stiffness `κ = 1e4 N/m`, sphere total
/// mass `M ≈ 4.32e-3 kg` (silicone-class `ρ = 1030 kg/m³` at `R = 1 cm`),
/// `~561` referenced vertices, the per-vertex lumped mass `m_v ≈ M /
/// n_ref ≈ 7.7e-6 kg`. Penalty-oscillator frequency `ω = sqrt(κ / m_v)
/// ≈ 3.6e4 rad/s`; backward-Euler per-step amplitude factor
/// `1 / (1 + ω²·dt²) ≈ 7.6e-4` knocks oscillation amplitude down `~1300×`
/// per step in the post-contact regime.
const DT: f64 = 1.0e-3;

/// Total step count (1000 → 1 s simulated). Sphere-bottom-to-band
/// freefall time `t_c = sqrt(2 (h-R-d̂) / |g|) ≈ 89 ms ≈ 89 steps`
/// (captured `n_step_first_contact = 88`, one step early under
/// sub-step interpolation). Post-contact decay: `~few hundred steps`.
/// `1000` gives `~10×` headroom on the rest gate.
const N_STEPS: usize = 1000;

/// Newton iter cap (mirror V-5). Bumped from skeleton's 10 to 50 for
/// transient-integration headroom (penalty oscillation during contact +
/// Newton step under steep `M / dt²` regularization can take 5-15 iters
/// per step).
const MAX_NEWTON_ITER: usize = 50;

/// Per-vertex velocity magnitude floor for "rest" (`m/s`). Mirror V-5
/// verbatim. `1 cm/s` is generous — at the scene's `(κ, m_v)` the
/// BE-damped envelope falls below `1e-4 m/s` long before step 1000.
const KE_REST_THRESHOLD: f64 = 1.0e-2;

/// Multiplier on the freefall velocity bound `sqrt(2 g h)` for the
/// per-step "no energy injection" gate. Mirror V-5 verbatim. `1.5×`
/// accommodates penalty's documented bounded-oscillation overshoot.
const ENERGY_BOUND_SAFETY: f64 = 1.5;

/// Penalty contact band `d̂` (m). `PenaltyRigidContact::new` defaults
/// pinned per Phase 5 scope memo Decision J — re-asserted here as a
/// named const for the [`com_at_equilibrium_height`](verify_com_at_equilibrium_height)
/// and [`contact_engagement`](verify_contact_engagement) gates' algebra.
/// MUST equal `sim_soft::contact::penalty::PENALTY_DHAT_DEFAULT`; if the
/// helper's default is ever tuned, this const and the captured bits below
/// must move in lockstep.
const D_HAT: f64 = 1.0e-3;

/// COM-at-equilibrium tolerance (m). `2 × D_HAT = 2 mm`. The settled
/// sphere center sits at `R + d̂ = 1.1 cm` above the plane (sphere bottom
/// just below the band's upper bound; surface deformed to wrap the band's
/// width); Hertz self-weight indentation at Ecoflex stiffness `E ≈ 600
/// kPa` and `m·g ≈ 4.24e-2 N` over `~mm` contact radius is
/// `O(10s of µm)`. The `2 mm` bound catches a sphere-at-wrong-height
/// regression (e.g., gravity-direction sign flip, a dt that under-resolves
/// freefall, a mis-configured plane offset) without flake from the
/// Hertz floor. Tighter contact-mechanics gates belong at row 13 (Hertz)
/// where contact patch IS the headline.
const COM_TOLERANCE: f64 = 2.0e-3;

/// IV-1 sparse-tier rel-tol for captured drop-metric bits. ~2k tets
/// through faer's sparse Cholesky lives between IV-1's dense bit-equal
/// tier (12-24 DOFs) and IV-1's sparse-at-scale tier (~3k tets, 3-ULP
/// cross-platform drift on faer's per-column FMA-fusion path); 1 ms
/// integrator step layered on top adds another arithmetic stage. `1e-12`
/// admits sparse-solver SIMD/FMA noise while catching any real
/// regression. Same precedent as PR1 rows 6 + 10 + 11.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for relative comparisons that touch zero. Below typical
/// drop-metric magnitudes by 8+ orders of magnitude. Same precedent as
/// PR1 rows 6 + 10 + 11.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Visual-mode replay rate multiplier on `Trajectory.dt`. `10.0×`
/// stretches the 1-s simulated trajectory to 10 s wall-clock replay
/// (`SLOW_MO_FACTOR × N_STEPS × DT = 10 × 1000 × 1e-3 = 10 s`). Default
/// rationale: the analytic time-to-impact `t_c = sqrt(2 (h-R-d̂) / |g|)
/// ≈ 89 ms` is blink-and-miss-it at 1× wall-clock; `10×` puts the
/// freefall + contact-onset arc at `~890 ms` (clearly observable, and
/// the contact-pair onset reads as a distinct beat) while the
/// settle-and-rest phase fits under `9 s`. Press `R` mid-replay to
/// reset and watch again from frame 0 — see
/// [`reset_replay_on_keypress`](sim_bevy_soft::trajectory::reset_replay_on_keypress).
/// Pure visualization knob — has no effect on the headless asserts or
/// the captured PLY.
const SLOW_MO_FACTOR: f64 = 10.0;

// =============================================================================
// Exact-pinned mesh counts (III-1 determinism contract)
// =============================================================================
//
// **Capture provenance** — captured 2026-05-06 at sim-soft `dev` tip
// `8401eed6` (post-PR2 foundation phase: C1 trait unification + C2a
// boundary_faces + C2b cf-bevy-common factor-out + C2c sim-bevy-soft
// skeleton + C2d sim-bevy migration), rustc 1.95.0 (`59807616e`
// 2026-04-14) on macOS arm64.

/// Total tet count at `cell_size = 3 mm` on the `SphereSdf(radius =
/// 1 cm)` body via `SdfMeshedTetMesh::from_sdf` (BCC + Labelle-Shewchuk
/// pipeline).
const N_TETS_EXACT: usize = 2208;

/// Total mesh vertex count, including BCC lattice corners not referenced
/// by any tet (orphans). The 96% orphan ratio at this `(R, h)` is a
/// BCC-plus-bbox-margin artifact (the mesher allocates corners for the
/// full 6-cell-radius bbox per `SPHERE_BBOX_MARGIN_RATIO`, then references
/// only those that participate in the surface-cut tets); orphan auto-pin
/// at `CpuNewtonSolver::new` keeps the free-DOF Hessian SPD without
/// touching the orphan cohort.
const N_VERTICES_EXACT: usize = 18696;

/// Vertices referenced by at least one tet. `N_VERTICES_EXACT -
/// N_REFERENCED_EXACT = 18135` orphans auto-pinned by
/// `CpuNewtonSolver::new`'s `effective_pinned` step.
const N_REFERENCED_EXACT: usize = 561;

// =============================================================================
// Captured drop-metric bits (IV-1 sparse-tier contract)
// =============================================================================
//
// **Failure-mode protocol** (mirrors IV-1's): if the rel-tol comparison
// fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version delta
//      vs the rustc 1.95.0 capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the `SoftScene::dropping_sphere` constructor, the
//      `SolverConfig::gravity_z` wiring, the `PenaltyRigidContact`
//      defaults, OR the SDF-meshed FEM assembly path through faer.
//   3. NEVER re-bake the reference values to make the test green.

/// Final-step `|v|_max` (m/s) — peak vertex velocity at step 1000, after
/// BE-damped post-contact decay.
/// `f64::from_bits(0x3f1f_2042_c744_8e2d) ≈ 1.187_363_394_064_492_3e-4`.
/// Below `KE_REST_THRESHOLD = 1 cm/s` by `~84×` headroom — the rest gate
/// has wide margin; this bit captures the actual residual quiescence
/// magnitude for regression detection.
const FINAL_V_MAX_REF_BITS: u64 = 0x3f1f_2042_c744_8e2d;

/// COM z-coordinate at rest (m) — final referenced-vertex mean-z.
/// `f64::from_bits(0x3f86_4276_ba7d_682c) ≈ 1.086_895_710_584_564_3e-2`.
/// Sits `~0.13 mm` below `R + D_HAT = 1.1 cm` (Hertz self-weight
/// indentation under Ecoflex stiffness — well inside `COM_TOLERANCE = 2
/// mm`).
const COM_Z_AT_REST_REF_BITS: u64 = 0x3f86_4276_ba7d_682c;

/// COM descent magnitude (m) — `initial_mean_z - final_mean_z` over the
/// referenced-vertex set.
/// `f64::from_bits(0x3fa4_08fb_eafa_3fbc) ≈ 3.913_104_289_415_467e-2`.
/// Approximately `RELEASE_HEIGHT - (R + D_HAT) = 5 cm - 1.1 cm = 3.9 cm`
/// (initial mean-z sits at the rest-config sphere center, shifted to
/// `RELEASE_HEIGHT`; final settles `0.13 mm` below the geometric R+d̂
/// floor under self-weight).
const COM_DESCENT_REF_BITS: u64 = 0x3fa4_08fb_eafa_3fbc;

/// First step where any referenced vertex enters the contact band `d̂`.
/// Captured at 88. Analytic freefall time for the sphere *bottom* (not
/// center) to reach the band's upper edge: `t_c = sqrt(2 (h - R - d̂)
/// / |g|) / dt = sqrt(2 × 0.039 / 9.81) / 1e-3 ≈ 89 steps` — captured
/// 88 sits one step early (sub-step interpolation under penalty's
/// bounded-oscillation onset). Captures gravity-magnitude regressions
/// distinct from the per-step energy gate.
const N_STEP_FIRST_CONTACT_REF: usize = 88;

/// Maximum Newton iter count observed across all `N_STEPS` solver
/// invocations. Captured at 8 — well below `MAX_NEWTON_ITER = 50`'s wide
/// headroom. Captures Newton-convergence regressions distinct from the
/// per-step `iter ≤ cap` gate.
const MAX_ITER_OBSERVED_REF: usize = 8;

// =============================================================================
// Helpers — math
// =============================================================================

/// Maximum per-vertex velocity magnitude over a flat `[3·N]` velocity
/// buffer. Mirror V-5 helper verbatim.
fn max_vertex_velocity(v_flat: &[f64]) -> f64 {
    debug_assert!(v_flat.len().is_multiple_of(3));
    let mut max = 0.0_f64;
    let n_vertices = v_flat.len() / 3;
    for v in 0..n_vertices {
        let vx = v_flat[3 * v];
        let vy = v_flat[3 * v + 1];
        let vz = v_flat[3 * v + 2];
        let mag = vx.mul_add(vx, vy.mul_add(vy, vz * vz)).sqrt();
        if mag > max {
            max = mag;
        }
    }
    max
}

/// Mean z-coordinate over a referenced-vertices subset of a flat `[3·N]`
/// position buffer. Mirror V-5 helper verbatim — orphan vertices
/// (auto-pinned at `x_prev` by `CpuNewtonSolver::new`'s `effective_pinned`
/// step) are excluded so the mean tracks the actual sphere body, not the
/// static auto-pinned orphan cohort.
fn mean_referenced_z(x_flat: &[f64], referenced: &[VertexId]) -> f64 {
    debug_assert!(x_flat.len().is_multiple_of(3));
    let z_sum: f64 = referenced
        .iter()
        .map(|&v| x_flat[3 * (v as usize) + 2])
        .sum();
    z_sum / referenced.len() as f64
}

/// Minimum z-coordinate over a referenced-vertices subset. Used by the
/// contact-engagement gate: a vertex with `z < D_HAT` is in the penalty
/// contact band (since the rigid plane sits at `z = 0` with outward normal
/// `+ẑ`, signed distance for vertex above the plane is `p.z`; in band when
/// `p.z < d̂`). Restricting to referenced vertices avoids tracking
/// auto-pinned orphans whose `z` stays at the release-height initial.
fn min_referenced_z(x_flat: &[f64], referenced: &[VertexId]) -> f64 {
    debug_assert!(x_flat.len().is_multiple_of(3));
    referenced
        .iter()
        .map(|&v| x_flat[3 * (v as usize) + 2])
        .fold(f64::INFINITY, f64::min)
}

// =============================================================================
// Snapshot — captured solver outputs after the full N_STEPS run
// =============================================================================

/// Carries everything the verify_* gates + PLY emit + Bevy replay need
/// from one full `dropping_sphere` rollout. Built once by
/// [`run_dropping_sphere`]; immutable thereafter.
struct DropSnapshot {
    /// Total mesh vertex count (including orphans).
    n_vertices: usize,
    /// Total tet count.
    n_tets: usize,
    /// Vertices referenced by at least one tet (the solver's free-DOF
    /// candidate set; orphan auto-pin excludes the rest).
    referenced: Vec<VertexId>,
    /// Boundary-face triangulation (cached by `Mesh::boundary_faces`'s
    /// cache, here lifted to an owned `Vec` for the PLY emit + Bevy
    /// build paths).
    boundary_faces: Vec<[VertexId; 3]>,
    /// Rest-configuration positions (mesh.positions() snapshot — used for
    /// `build_soft_mesh`'s initial pose).
    rest_positions: Vec<Vec3>,
    /// Whether the boundary partition is empty (free-flight scene).
    /// Captured at `BoundaryConditions` build, since `bc` is consumed by
    /// `CpuNewtonSolver::new`.
    pinned_was_empty: bool,
    loaded_was_empty: bool,
    /// Initial referenced-vertex mean-z (rest configuration shifted to
    /// `RELEASE_HEIGHT`).
    initial_mean_z: f64,
    /// Per-step trajectory: `frames[k]` is the converged `x_final` at
    /// step `k` (vertex-major + xyz-inner DOF layout).
    trajectory: Vec<Vec<f64>>,
    /// Per-step Newton iteration count.
    iter_counts: Vec<usize>,
    /// Per-step `|v|_max` envelope (peak vertex velocity at each step's
    /// converged config).
    v_max_envelope: Vec<f64>,
    /// Maximum `|v|_max` observed across the full run.
    max_v_observed: f64,
    /// Maximum Newton iter-count observed across the full run.
    max_iter_observed: usize,
    /// First step index at which the sphere enters the contact band
    /// (`min_referenced_z(frame) < D_HAT`).
    n_step_first_contact: usize,
    /// Final-step `|v|_max`.
    final_v_max: f64,
    /// Final referenced-vertex mean-z.
    final_mean_z: f64,
}

impl DropSnapshot {
    fn final_frame(&self) -> &[f64] {
        self.trajectory
            .last()
            .expect("trajectory always has at least one frame after run")
    }

    fn com_descent(&self) -> f64 {
        self.initial_mean_z - self.final_mean_z
    }
}

// =============================================================================
// Run — build solver, integrate N_STEPS, capture trajectory + diagnostics
// =============================================================================

fn run_dropping_sphere() -> DropSnapshot {
    let (mesh, bc, initial, contact) = SoftScene::dropping_sphere(
        RADIUS,
        CELL_SIZE,
        RELEASE_HEIGHT,
        MaterialField::uniform(MU, LAMBDA),
    )
    .expect("dropping_sphere should mesh successfully at canonical params");

    let n_vertices = mesh.n_vertices();
    let n_tets = mesh.n_tets();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let pinned_was_empty = bc.pinned_vertices.is_empty();
    let loaded_was_empty = bc.loaded_vertices.is_empty();
    let boundary_faces: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();

    let n_dof = 3 * n_vertices;

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAVITY;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let SceneInitial { x_prev, v_prev } = initial;
    let initial_mean_z = mean_referenced_z(x_prev.as_slice(), &referenced);

    let mut x_state: Vec<f64> = x_prev.as_slice().to_vec();
    let mut v_state: Vec<f64> = v_prev.as_slice().to_vec();

    let theta = Tensor::from_slice(&[], &[0]);

    let mut trajectory: Vec<Vec<f64>> = Vec::with_capacity(N_STEPS);
    let mut iter_counts: Vec<usize> = Vec::with_capacity(N_STEPS);
    let mut v_max_envelope: Vec<f64> = Vec::with_capacity(N_STEPS);
    let mut max_v_observed = 0.0_f64;
    let mut max_iter_observed = 0_usize;
    let mut n_step_first_contact: usize = N_STEPS; // sentinel: never engaged

    for step_idx in 0..N_STEPS {
        let x_in = Tensor::from_slice(&x_state, &[n_dof]);
        let v_in = Tensor::from_slice(&v_state, &[n_dof]);
        let step: NewtonStep<_> = solver.replay_step(&x_in, &v_in, &theta, DT);

        // Backward-Euler velocity update: v_final = (x_final - x_prev) / dt.
        let v_new: Vec<f64> = step
            .x_final
            .iter()
            .zip(x_state.iter())
            .map(|(xf, xp)| (xf - xp) / DT)
            .collect();

        let v_max_step = max_vertex_velocity(&v_new);
        if v_max_step > max_v_observed {
            max_v_observed = v_max_step;
        }
        if step.iter_count > max_iter_observed {
            max_iter_observed = step.iter_count;
        }

        if n_step_first_contact == N_STEPS && min_referenced_z(&step.x_final, &referenced) < D_HAT {
            n_step_first_contact = step_idx;
        }

        iter_counts.push(step.iter_count);
        v_max_envelope.push(v_max_step);
        x_state = step.x_final;
        v_state = v_new;
        trajectory.push(x_state.clone());
    }

    let final_v_max = v_max_envelope
        .last()
        .copied()
        .expect("trajectory has N_STEPS > 0 frames");
    let final_mean_z = mean_referenced_z(&x_state, &referenced);

    DropSnapshot {
        n_vertices,
        n_tets,
        referenced,
        boundary_faces,
        rest_positions,
        pinned_was_empty,
        loaded_was_empty,
        initial_mean_z,
        trajectory,
        iter_counts,
        v_max_envelope,
        max_v_observed,
        max_iter_observed,
        n_step_first_contact,
        final_v_max,
        final_mean_z,
    }
}

// =============================================================================
// Anchor 1 — geometry_invariants (compile-time)
// =============================================================================

const fn verify_geometry_invariants() {
    const { assert!(RADIUS > 0.0, "RADIUS must be positive") };
    const { assert!(CELL_SIZE > 0.0, "CELL_SIZE must be positive") };
    const {
        assert!(
            RELEASE_HEIGHT > RADIUS + D_HAT,
            "RELEASE_HEIGHT must exceed RADIUS + D_HAT (sphere starts clear of contact band)",
        );
    };
    const { assert!(DT > 0.0, "DT must be positive") };
    const { assert!(N_STEPS > 0, "N_STEPS must be positive") };
    const {
        assert!(
            KE_REST_THRESHOLD > 0.0,
            "KE_REST_THRESHOLD must be positive",
        );
    };
    const { assert!(MU > 0.0, "MU must be positive") };
    const { assert!(LAMBDA > 0.0, "LAMBDA must be positive") };
    const { assert!(GRAVITY < 0.0, "GRAVITY must be downward (negative)") };
    const { assert!(D_HAT > 0.0, "D_HAT must be positive") };
    const {
        assert!(
            COM_TOLERANCE >= 2.0 * D_HAT,
            "COM_TOLERANCE should be ≥ 2 × D_HAT to absorb Hertz indentation under self-weight",
        );
    };
    const {
        assert!(
            ENERGY_BOUND_SAFETY > 1.0,
            "ENERGY_BOUND_SAFETY must allow penalty oscillation overshoot above pure freefall",
        );
    };
    const { assert!(MAX_NEWTON_ITER > 0, "MAX_NEWTON_ITER must be positive") };
}

// =============================================================================
// Anchor 2 — mesh_topology_exact
// =============================================================================

fn verify_mesh_topology_exact(snapshot: &DropSnapshot) {
    assert_eq!(
        snapshot.n_tets, N_TETS_EXACT,
        "n_tets drift: expected {N_TETS_EXACT}, got {} — III-1 determinism contract violated",
        snapshot.n_tets,
    );
    assert_eq!(
        snapshot.n_vertices, N_VERTICES_EXACT,
        "n_vertices drift: expected {N_VERTICES_EXACT}, got {}",
        snapshot.n_vertices,
    );
    assert_eq!(
        snapshot.referenced.len(),
        N_REFERENCED_EXACT,
        "referenced count drift: expected {N_REFERENCED_EXACT}, got {}",
        snapshot.referenced.len(),
    );
    assert!(
        snapshot.referenced.len() < snapshot.n_vertices,
        "BCC orphan-rejection invariant non-vacuous: referenced ({}) < n_vertices ({})",
        snapshot.referenced.len(),
        snapshot.n_vertices,
    );
}

// =============================================================================
// Anchor 3 — boundary_partition (free-flight scene)
// =============================================================================

fn verify_boundary_partition(snapshot: &DropSnapshot) {
    assert!(
        snapshot.pinned_was_empty,
        "free-flight scene must have empty pinned_vertices (orphan auto-pin is the only Dirichlet)",
    );
    assert!(
        snapshot.loaded_was_empty,
        "free-flight scene must have empty loaded_vertices (gravity is solver-level body force, not external traction)",
    );
}

// =============================================================================
// Anchor 4 — solver_per_step_invariants (mirror V-5)
// =============================================================================

fn verify_solver_per_step_invariants(snapshot: &DropSnapshot) {
    let v_freefall_bound = (-2.0 * GRAVITY * RELEASE_HEIGHT).sqrt() * ENERGY_BOUND_SAFETY;

    for (step_idx, frame) in snapshot.trajectory.iter().enumerate() {
        for (i, &x) in frame.iter().enumerate() {
            assert!(
                x.is_finite(),
                "step {step_idx}: x_final[{i}] = {x} is not finite — Newton diverged",
            );
        }
        let v_max_step = snapshot.v_max_envelope[step_idx];
        assert!(
            v_max_step < v_freefall_bound,
            "step {step_idx}: |v|_max = {v_max_step:.4} m/s exceeds freefall bound {v_freefall_bound:.4} m/s — energy injection detected",
        );
        let iter_count = snapshot.iter_counts[step_idx];
        assert!(
            iter_count <= MAX_NEWTON_ITER,
            "step {step_idx}: iter_count = {iter_count} exceeded MAX_NEWTON_ITER = {MAX_NEWTON_ITER}",
        );
    }
}

// =============================================================================
// Anchor 5 — contact_engagement
// =============================================================================

fn verify_contact_engagement(snapshot: &DropSnapshot) {
    let upper_bound = N_STEPS / 4;
    assert!(
        snapshot.n_step_first_contact > 0,
        "n_step_first_contact = 0 — sphere started in contact band (release_height misconfigured)",
    );
    assert!(
        snapshot.n_step_first_contact <= upper_bound,
        "n_step_first_contact = {} exceeds upper bound {upper_bound} (= N_STEPS / 4) — sphere never reached the plane within ~2.8× the analytic freefall time-to-band (~step 89, sphere-bottom). Likely cause: gravity wiring inactive OR sphere flying sideways through plane.",
        snapshot.n_step_first_contact,
    );
}

// =============================================================================
// Anchor 6 — reaches_rest (mirror V-5)
// =============================================================================

fn verify_reaches_rest(snapshot: &DropSnapshot) {
    assert!(
        snapshot.final_v_max < KE_REST_THRESHOLD,
        "final-step |v|_max = {:.4e} m/s ≥ rest threshold {KE_REST_THRESHOLD:.0e} m/s — sphere has not reached steady state within N_STEPS = {N_STEPS}",
        snapshot.final_v_max,
    );
}

// =============================================================================
// Anchor 7 — com_descended (mirror V-5)
// =============================================================================

fn verify_com_descended(snapshot: &DropSnapshot) {
    assert!(
        snapshot.final_mean_z < snapshot.initial_mean_z,
        "final mean_z = {:.4e} m not below initial mean_z = {:.4e} m — sphere did not descend under gravity",
        snapshot.final_mean_z,
        snapshot.initial_mean_z,
    );
}

// =============================================================================
// Anchor 8 — com_at_equilibrium_height (NEW — fills V-5 gap-to-land)
// =============================================================================

fn verify_com_at_equilibrium_height(snapshot: &DropSnapshot) {
    let expected = RADIUS + D_HAT;
    let observed = snapshot.final_mean_z;
    let abs_diff = (observed - expected).abs();
    assert!(
        abs_diff < COM_TOLERANCE,
        "settled COM z = {observed:.6e} m differs from expected R + D_HAT = {expected:.6e} m by {abs_diff:.4e} m — outside COM_TOLERANCE = {COM_TOLERANCE:.0e} m. Likely cause: gravity-direction sign flip, dt under-resolves freefall, or plane-offset misconfigured.",
    );
}

// =============================================================================
// Anchor 9 — captured_bits_drop_metrics (IV-1 sparse-tier rel-tol)
// =============================================================================

fn verify_captured_bits_drop_metrics(snapshot: &DropSnapshot) {
    let final_v_max_ref = f64::from_bits(FINAL_V_MAX_REF_BITS);
    let com_z_ref = f64::from_bits(COM_Z_AT_REST_REF_BITS);
    let com_descent_ref = f64::from_bits(COM_DESCENT_REF_BITS);

    assert_relative_eq!(
        snapshot.final_v_max,
        final_v_max_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.final_mean_z,
        com_z_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.com_descent(),
        com_descent_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_eq!(
        snapshot.n_step_first_contact, N_STEP_FIRST_CONTACT_REF,
        "n_step_first_contact drift: expected {N_STEP_FIRST_CONTACT_REF}, got {}",
        snapshot.n_step_first_contact,
    );
    assert_eq!(
        snapshot.max_iter_observed, MAX_ITER_OBSERVED_REF,
        "max_iter_observed drift: expected {MAX_ITER_OBSERVED_REF}, got {}",
        snapshot.max_iter_observed,
    );
}

// =============================================================================
// PLY emit — final-frame deformed boundary mesh
// =============================================================================

fn save_final_frame_ply(snapshot: &DropSnapshot, path: &Path) -> Result<()> {
    let frame = snapshot.final_frame();
    let n_vertices = snapshot.n_vertices;

    // Build positions in physics +Z frame (no swap — cf-view handles orientation).
    let vertices: Vec<Point3<f64>> = (0..n_vertices)
        .map(|i| Point3::new(frame[3 * i], frame[3 * i + 1], frame[3 * i + 2]))
        .collect();

    // Boundary-face indices as CCW-wound triangles (outward winding from
    // the right-handed-tet `signed_volume > 0` invariant per
    // `boundary_faces_from_topology`).
    let faces: Vec<[u32; 3]> = snapshot
        .boundary_faces
        .iter()
        .map(|f| [f[0], f[1], f[2]])
        .collect();

    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut mesh = AttributedMesh::new(geometry);
    // No per-vertex scalars: at quiescence |v| ≈ 0 everywhere; contact-band
    // membership is a row-13 Hertz-patch concern. Compute smooth normals so
    // cf-view's PBR shading isn't flat across faces.
    mesh.compute_normals();

    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// Bevy visual mode — opt-in via `CF_VISUAL=1`
// =============================================================================

/// Resource carrying the headless-harness outputs the Bevy `Startup`
/// system needs to spawn the soft-mesh + plane + camera entities. Wrapped
/// in `Option` so `setup_visual_scene` can `.take()` it once, freeing the
/// trajectory frames after the entity is spawned.
#[derive(Resource)]
struct VisualSetup(Option<VisualSetupInner>);

struct VisualSetupInner {
    trajectory: Trajectory,
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
}

fn run_visual_mode(snapshot: &DropSnapshot) {
    let visual_setup = VisualSetupInner {
        trajectory: Trajectory {
            frames: snapshot.trajectory.clone(),
            dt: DT * SLOW_MO_FACTOR,
        },
        rest_positions: snapshot.rest_positions.clone(),
        boundary_faces: snapshot.boundary_faces.clone(),
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(SoftBodyVisualPlugin)
        .insert_resource(UpAxis::PlusZ)
        .insert_resource(VisualSetup(Some(visual_setup)))
        .add_systems(Startup, setup_visual_scene)
        .run();
}

/// Render-side scale factor on visual entities. The simulation runs at
/// physics scale (sphere radius `1 cm`); the rendered Bevy entities are
/// `100×` larger so geometry lands in Bevy 0.18's pipeline-default
/// human-scale (m) regime (near plane `0.1 m`, OrbitCamera
/// `min_distance = 0.1 m`, AmbientLight brightness tuned for `1 m+`
/// scenes). At cm-scale rendering, the camera approached the near
/// plane on any zoom-in and clipped the front of the sphere; at `100×`
/// the rendered sphere is `1 m` radius and all defaults Just Work.
/// Headless asserts + PLY emit are scale-invariant — they operate on
/// the unscaled physics positions, so this is visualization-only.
const RENDER_SCALE: f32 = 100.0;

fn setup_visual_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<BevyMesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut visual_setup: ResMut<VisualSetup>,
    up: Res<UpAxis>,
) {
    let Some(VisualSetupInner {
        trajectory,
        rest_positions,
        boundary_faces,
    }) = visual_setup.0.take()
    else {
        return;
    };

    // Soft-mesh entity: built from rest config, animated by step_replay
    // reading the Trajectory each frame. Per `#[require(Mesh3d)]` on
    // Trajectory, the spawn bundle MUST include Mesh3d — provided here.
    // Coral PBR base — warm, light, high contrast against the dark gray
    // Bevy clear color so the deformation arc reads cleanly without the
    // user squinting. Slight gloss (perceptual_roughness 0.5) catches
    // the directional light's specular highlight at the contact patch
    // without going full glossy. The Transform's scale is `RENDER_SCALE`
    // (see const docs above) so the cm-scale physics positions render
    // at human-scale.
    let soft_mesh_handle = meshes.add(build_soft_mesh(&rest_positions, &boundary_faces, *up));
    let soft_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.55, 0.4),
        perceptual_roughness: 0.5,
        metallic: 0.05,
        ..default()
    });
    commands.spawn((
        Mesh3d(soft_mesh_handle),
        MeshMaterial3d(soft_material),
        Transform::from_scale(BevyVec3::splat(RENDER_SCALE)),
        trajectory,
    ));

    // Rigid plane visualization — gray PBR quad at Bevy y=0 (= physics z=0
    // under PlusZ swap). Plane mesh is sized at Bevy-frame scale
    // directly (no physics-positions to scale), so half_size already
    // accounts for RENDER_SCALE: 4 × R × RENDER_SCALE = 4 m.
    let plane_half_size = 4.0 * RADIUS as f32 * RENDER_SCALE;
    let plane_mesh = meshes.add(Plane3d::new(BevyVec3::Y, Vec2::splat(plane_half_size)));
    let plane_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.9,
        ..default()
    });
    commands.spawn((
        Mesh3d(plane_mesh),
        MeshMaterial3d(plane_material),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Camera target = rest COM (physics `(0, 0, R+d̂)` → Bevy `(0,
    // R+d̂, 0)` under PlusZ swap), scaled into Bevy-frame meters by
    // RENDER_SCALE so target lands at `(0, 1.1, 0)` Bevy (vs
    // `(0, 0.011, 0)` at 1×). Distance = 15 m (was 15 cm); OrbitCamera
    // defaults (min_distance = 0.1 m, near plane 0.1 m, etc.) all work
    // at this scale.
    let target_y = (RADIUS + D_HAT) as f32 * RENDER_SCALE;
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        OrbitCamera::new()
            .with_target(BevyVec3::new(0.0, target_y, 0.0))
            .with_distance(0.15 * RENDER_SCALE)
            .with_angles(0.6, 0.4),
        // AmbientLight is a per-camera Component in Bevy 0.18 (not a
        // global Resource). Low-but-nonzero (80 cd/m², near Bevy
        // default) keeps the unlit hemisphere readable without
        // washing out the spherical NdotL gradient on the lit side.
        AmbientLight {
            color: Color::WHITE,
            brightness: 80.0,
            ..default()
        },
    ));

    // Directional light positioned above-and-camera-side so the
    // sphere's camera-facing hemisphere is the LIT one. Directional
    // lights have no position (only orientation), so we don't need to
    // scale here — only the looking_at source position needs to be
    // far enough that the look-direction calculation is well-conditioned.
    // 12 klx illuminance is the overcast-bright-room range; keeps the
    // plane (gray albedo 0.7) reading as a clean mid-gray rather than
    // blowing out to white, while the coral sphere's lit-side reads at
    // full saturation.
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(0.5, 1.0, 0.5).looking_at(BevyVec3::ZERO, BevyVec3::Y),
    ));

    // HUD — Text overlay listing the controls so users discover them
    // without reading the README. Bottom-left corner, light-yellow on
    // dark-gray default Bevy clear color reads cleanly. Default font
    // ships via the `default_font` Cargo feature already in our set.
    // ASCII `|` separators avoid the default font's missing-glyph
    // boxes for U+00B7 middle-dot.
    commands.spawn((
        Text::new("Press R to reset  |  Mouse: drag orbit | scroll zoom  |  close window to exit"),
        TextFont {
            font_size: 14.0,
            ..default()
        },
        TextColor(Color::srgb(0.95, 0.95, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// =============================================================================
// Stdout museum-plaque
// =============================================================================

fn print_summary(snapshot: &DropSnapshot, ply_path: &Path) {
    let v_freefall_bound = (-2.0 * GRAVITY * RELEASE_HEIGHT).sqrt() * ENERGY_BOUND_SAFETY;
    let sim_t = DT * N_STEPS as f64;
    // Analytic time-for-sphere-BOTTOM-to-band: bottom falls
    // `RELEASE_HEIGHT - RADIUS - D_HAT` before crossing into d̂ (sphere
    // bottom is at z = release_height − R initially, band's upper edge
    // sits at z = d̂ above plane). This matches what
    // `n_step_first_contact` measures (`min_referenced_z < D_HAT` is the
    // sphere-bottom-touches-band predicate, not center-touches-plane).
    let bottom_fall_distance = RELEASE_HEIGHT - RADIUS - D_HAT;
    let analytic_step_first_contact = ((-2.0 * bottom_fall_distance / GRAVITY).sqrt() / DT).round();
    let com_descent = snapshot.com_descent();
    let com_z_expected = RADIUS + D_HAT;
    let com_z_diff = (snapshot.final_mean_z - com_z_expected).abs();

    println!("==== soft-drop-on-plane ====");
    println!();
    println!("Scene: SoftScene::dropping_sphere");
    println!(
        "  geometry             : SphereSdf body, RADIUS = {RADIUS} m, RELEASE_HEIGHT = {RELEASE_HEIGHT} m"
    );
    println!(
        "  cell_size            : {CELL_SIZE} m (V-5 mirror — release-mode-feasible step latency)"
    );
    println!(
        "                         = {} verts ({} referenced), {} tets",
        snapshot.n_vertices,
        snapshot.referenced.len(),
        snapshot.n_tets,
    );
    println!(
        "  material             : NH(MU = {MU:e}, LAMBDA = {LAMBDA:e})  Ecoflex 00-30 + 15 wt% carbon-black, ν = 0.4"
    );
    println!(
        "  rigid plane          : RigidPlane(normal = +ẑ, offset = 0) at z = 0 (kinematic, one-way)"
    );
    println!("  contact              : PenaltyRigidContact defaults (κ = 1e4 N/m, d̂ = {D_HAT} m)");
    println!(
        "  solver config         : Δt = {DT} s, gravity_z = {GRAVITY} m/s², max_newton_iter = {MAX_NEWTON_ITER}",
    );
    println!("  integration          : N_STEPS = {N_STEPS} (simulated time = {sim_t:.3} s)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  geometry_invariants                    : compile-time const asserts");
    println!(
        "  mesh_topology_exact                    : n_tets = {N_TETS_EXACT}, n_vertices = {N_VERTICES_EXACT}, referenced = {N_REFERENCED_EXACT}",
    );
    println!("  boundary_partition                     : pinned + loaded both empty (free-flight)");
    println!(
        "  solver_per_step_invariants             : no NaN, iter ≤ {MAX_NEWTON_ITER}, |v|_max < freefall bound × {ENERGY_BOUND_SAFETY}"
    );
    println!(
        "  contact_engagement                     : sphere enters contact band within N_STEPS / 4 = {} steps",
        N_STEPS / 4
    );
    println!(
        "  reaches_rest                           : final |v|_max < KE_REST_THRESHOLD = {KE_REST_THRESHOLD:.0e} m/s"
    );
    println!("  com_descended                          : final referenced mean-z < initial");
    println!(
        "  com_at_equilibrium_height              : |com_z - (R + d̂)| < COM_TOLERANCE = {COM_TOLERANCE:.0e} m"
    );
    println!(
        "  captured_bits_drop_metrics             : 5 metrics within IV-1 sparse-tier rel-tol = {SPARSE_REL_TOL:.0e}"
    );
    println!();
    println!("Solver result:");
    println!("  freefall bound × safety  : {v_freefall_bound:>13.6e} m/s");
    println!(
        "  |v|_max across all steps : {:>13.6e} m/s",
        snapshot.max_v_observed,
    );
    println!(
        "  |v|_max final step       : {:>13.6e} m/s   (rest threshold = {KE_REST_THRESHOLD:.0e})",
        snapshot.final_v_max,
    );
    println!(
        "  max Newton iters / step  : {:>13}     (cap = {MAX_NEWTON_ITER})",
        snapshot.max_iter_observed,
    );
    println!();
    println!("Drop kinematics (referenced-vertex mean-z over the run):");
    println!(
        "  initial mean-z           : {:>13.6e} m   (≈ RELEASE_HEIGHT = {RELEASE_HEIGHT:.4e} m)",
        snapshot.initial_mean_z,
    );
    println!(
        "  final   mean-z           : {:>13.6e} m   (expected R + d̂ = {com_z_expected:.4e} m)",
        snapshot.final_mean_z,
    );
    println!("  Δz (descent)             : {com_descent:>13.6e} m");
    println!(
        "  |Δ vs equilibrium|       : {com_z_diff:>13.6e} m   (tolerance {COM_TOLERANCE:.0e} m)",
    );
    println!();
    println!("Contact dynamics:");
    println!(
        "  n_step_first_contact     : {:>13}     (analytic ~{} for sqrt(2 (h-R-d̂) / |g|) / dt = sphere-bottom impact)",
        snapshot.n_step_first_contact, analytic_step_first_contact as i64,
    );
    println!();
    println!("PLY    : {}", ply_path.display());
    println!(
        "         deformed boundary mesh ({} vertices, {} triangles via Mesh::boundary_faces);",
        snapshot.n_vertices,
        snapshot.boundary_faces.len(),
    );
    println!(
        "         positions in physics +Z frame; no per-vertex scalars (quiescence ⇒ |v| ≈ 0)."
    );
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!();
    println!("Bevy replay (CF_VISUAL=1):");
    println!("           CF_VISUAL=1 cargo run -p example-sim-soft-soft-drop-on-plane --release");
    println!(
        "         spawns an OrbitCamera scene with the deforming sphere + gray plane + light;"
    );
    let replay_duration = sim_t * SLOW_MO_FACTOR;
    println!(
        "         replay at {SLOW_MO_FACTOR:.0}× slow-motion ({replay_duration:.1} s wall-clock for {sim_t:.3} s simulated;"
    );
    println!(
        "         clamp at end). Press R to replay from frame 0, mouse drag to orbit, scroll to"
    );
    println!(
        "         zoom, close window to exit. Per-entity ReplayEpoch defers playback clock to"
    );
    println!(
        "         first step_replay tick so DefaultPlugins startup (~1-2 s on first run) does"
    );
    println!("         not consume budget.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_geometry_invariants();

    let snapshot = run_dropping_sphere();

    verify_mesh_topology_exact(&snapshot);
    verify_boundary_partition(&snapshot);
    verify_solver_per_step_invariants(&snapshot);
    verify_contact_engagement(&snapshot);
    verify_reaches_rest(&snapshot);
    verify_com_descended(&snapshot);
    verify_com_at_equilibrium_height(&snapshot);
    verify_captured_bits_drop_metrics(&snapshot);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let ply_path = out_dir.join("soft_drop_on_plane.ply");
    save_final_frame_ply(&snapshot, &ply_path)?;

    print_summary(&snapshot, &ply_path);

    if std::env::var("CF_VISUAL").is_ok() {
        println!();
        println!("CF_VISUAL set — spawning Bevy trajectory replay (close window to exit) ...");
        run_visual_mode(&snapshot);
    }

    Ok(())
}
