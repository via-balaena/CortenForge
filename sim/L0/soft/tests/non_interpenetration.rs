//! V-4 — non-interpenetration to documented penalty tolerance.
//!
//! Phase 5 scope memo §1 V-4 + §8 commit 10 (`phase_5_penalty_contact_scope.md`).
//! At converged steady-state of any V-3a / V-3 / V-5 scene, every soft vertex
//! satisfies `signed_distance ≥ -δ_pen` for the documented penalty tolerance
//! `δ_pen = F_max / κ_pen`. Penalty cannot enforce zero overlap (book §00 §00
//! cited); V-4 is the gate documenting what penalty *can* enforce — bounded
//! overlap.
//!
//! ## Why V-4 matters
//!
//! V-3a / V-3 / V-5 each verify a *different* scientific property:
//! V-3a force-pumping correctness, V-3 Hertz contact-patch geometry, V-5
//! drop-and-rest dynamics under gravity. None of them directly checks the
//! non-penetration property the book §00 §00 explicitly names as penalty's
//! structural failure mode. V-4 fills that gap with a unified all-vertex
//! signed-distance walk against each scene's converged state.
//!
//! ## `δ_pen` formula and scene-specific values
//!
//! Per scope memo §1 V-4: `δ_pen = F_max / κ_pen` is the single-vertex
//! worst-case penalty overlap tolerance — derived from the equilibrium
//! force balance `κ · pen_v = F_v` where a single active vertex bears
//! the entire applied load. In practice multi-vertex contact distributes
//! the load (`pen_v ≈ F / (κ · N_active)` per active vertex) and actual
//! overlap is much smaller; the worst-case bound is conservative by
//! construction.
//!
//! - **V-3a compressive-block** — local override `κ = 1e4 N/m`,
//!   `F_max ≈ 0.18 N` (the integrated reaction force per V-3a commit-8
//!   empirical at canonical NH params); `δ_pen ≈ 1.8e-5 m`. Multi-vertex
//!   distribution over `(n+1)² = 25` active pairs at `n = 4` gives
//!   `pen_v ≈ 7.2e-7 m` per vertex; the worst-case bound has > 25× headroom.
//! - **V-3 Hertz sphere** — V-3-local override `κ = 1e3 N/m`, `F = 0.5 N`;
//!   `δ_pen = 5e-4 m`. Multi-vertex distribution at h/2 (~5 active pairs)
//!   gives `pen_v ≈ 1e-4 m`; worst-case bound has 5× headroom. (V-4
//!   re-uses V-3 commit-9's coarsest refinement `h = 3 mm` for fastest
//!   release-mode runtime; at `h` only 1 vertex is active, equivalent to
//!   the worst-case bound.)
//! - **V-5 drop-and-rest** — default `κ = 1e4 N/m`, `F_max = m · g` where
//!   `m = ρ · V_sphere ≈ 4.32e-3 kg` (silicone-class `ρ = 1030` kg/m³ at
//!   `R = 1 cm`) and `g = 9.81` m/s² → `F_max = 4.24e-2 N`. `δ_pen ≈
//!   4.24e-6 m`. The static-equilibrium contact patch at rest absorbs
//!   gravity through ~5-10 active vertices; per-vertex pen is sub-μm.
//!
//! ## Three test fns, scene-specific gating
//!
//! V-3a is debug-feasible (cube at `n = 4` is sub-second release-mode);
//! V-3 and V-5 inherit their parent test's `#[cfg_attr(debug_assertions,
//! ignore)]` release-only gate per `feedback_release_mode_heavy_tests`.
//! Each test re-runs its scene at the canonical V-* parameters and walks
//! every referenced vertex at converged `x_final` to assert
//! `signed_distance ≥ -δ_pen`.
//!
//! ## Why V-4 doesn't co-locate with V-3a / V-3 / V-5
//!
//! Per scope memo §0 site table, V-4 lives at `tests/non_interpenetration.rs`
//! as a stand-alone file. Co-locating non-penetration assertions inside
//! V-3a / V-3 / V-5 would couple the gates — a V-3a force-pumping
//! regression would mask a V-4 non-penetration regression and vice versa.
//! Keeping V-4 separate isolates failure modes for diagnosis.

#![allow(
    // Helpers return `Result<tuple, MeshingError>`; `expect_used` is
    // needed at the helper-call site. Mirrors V-3 / V-5 / Phase 4
    // precedent.
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver,
    SolverConfig, Tet4, Vec3, VertexId, referenced_vertices,
};

// ── V-3a non-penetration: cube against plane at converged equilibrium ────

/// V-3a-mirroring scene constants — must track
/// `penalty_compressive_block.rs` exactly so the `δ_pen` budget remains
/// a faithful representation of V-3a's actual converged state.
mod v_3a {
    pub const EDGE_LEN: f64 = 0.01;
    pub const DISPLACEMENT: f64 = 5.0e-5;
    pub const MU: f64 = 1.0e5;
    pub const LAMBDA: f64 = 4.0e5;
    pub const KAPPA: f64 = 1.0e4;
    pub const D_HAT_OVERRIDE: f64 = 1.0e-5;
    pub const STATIC_DT: f64 = 1.0;
    pub const MAX_NEWTON_ITER: usize = 50;
    /// Refinement: `n = 4` mid-level (matches V-3a commit-8's middle
    /// refinement; faster than `n = 8` for V-4's hygiene scope, finer
    /// than `n = 2` for adequate vertex coverage).
    pub const N_PER_EDGE: usize = 4;
    /// Conservative `δ_pen = F_max / κ` per scope memo §1 V-4.
    /// `F_max` per V-3a commit-8 finest-level empirical (~0.18 N at
    /// n=8); n=4 is similar by Cauchy convergence (~0.186 N).
    pub const F_MAX: f64 = 0.20;
    pub const DELTA_PEN: f64 = F_MAX / KAPPA;
}

#[test]
fn v_4_compressive_block_non_interpenetration() {
    // Cast safe: N_PER_EDGE is a small literal; conversion to f64 is
    // loss-free.
    #[allow(clippy::cast_precision_loss)]
    let cell_size = v_3a::EDGE_LEN / (v_3a::N_PER_EDGE as f64);
    let material_field = MaterialField::uniform(v_3a::MU, v_3a::LAMBDA);
    let (mesh, bc, initial, _default_contact) = SoftScene::compressive_block_on_plane(
        v_3a::EDGE_LEN,
        cell_size,
        v_3a::DISPLACEMENT,
        &material_field,
    );
    // V-3a-local override: κ default but d̂ = 1e-5 (V-3a's deviation 2
    // per `penalty_compressive_block.rs` module docstring). Plane
    // reconstructed identically to `SoftScene::compressive_block_on_plane`'s
    // helper construction.
    let plane = RigidPlane::new(
        Vec3::new(0.0, 0.0, -1.0),
        v_3a::DISPLACEMENT - v_3a::EDGE_LEN,
    );
    let contact = PenaltyRigidContact::with_params(vec![plane], v_3a::KAPPA, v_3a::D_HAT_OVERRIDE);

    let n_dof = 3 * mesh.n_vertices();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = v_3a::STATIC_DT;
    cfg.max_newton_iter = v_3a::MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;

    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let empty_theta = Tensor::from_slice(&[], &[0]);
    let step = solver.replay_step(&x_prev, &v_prev, &empty_theta, cfg.dt);

    // signed_distance(p) = p · normal - offset, with normal = -ẑ and
    // offset = DISPLACEMENT - EDGE_LEN. So sd = -p_z - (DISPLACEMENT -
    // EDGE_LEN) = EDGE_LEN - DISPLACEMENT - p_z. At rest top face (p_z
    // = EDGE_LEN), sd = -DISPLACEMENT. At equilibrium top face has
    // descended by penalty deformation, sd → -DISPLACEMENT + δ_eq ≈
    // small positive.
    let plane_offset = v_3a::DISPLACEMENT - v_3a::EDGE_LEN;
    let n_vertices = n_dof / 3;
    let mut min_sd = f64::INFINITY;
    let mut min_sd_vertex: usize = 0;
    for v in 0..n_vertices {
        let pz = step.x_final[3 * v + 2];
        let sd = -pz - plane_offset;
        if sd < min_sd {
            min_sd = sd;
            min_sd_vertex = v;
        }
    }

    eprintln!(
        "v_4 V-3a non-penetration: n_per_edge = {n}, n_vertices = {nv}, \
         κ = {kappa:e} N/m, d̂_override = {dhat:e} m, F_max = {fmax} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        n = v_3a::N_PER_EDGE,
        nv = n_vertices,
        kappa = v_3a::KAPPA,
        dhat = v_3a::D_HAT_OVERRIDE,
        fmax = v_3a::F_MAX,
        dp = v_3a::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -v_3a::DELTA_PEN,
        "V-3a converged state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F_max / κ). Penalty cannot enforce \
         zero overlap (book §00 §00) but should bound overlap by F_max/κ at single-vertex \
         worst-case; multi-vertex contact at this scene's (n+1)² = {expected_active} active \
         pairs makes per-vertex penetration much smaller than worst-case in practice.",
        v = min_sd_vertex,
        sd = min_sd,
        dp = v_3a::DELTA_PEN,
        expected_active = (v_3a::N_PER_EDGE + 1) * (v_3a::N_PER_EDGE + 1),
    );
}

// ── V-3 non-penetration: Hertz sphere on plane at converged equilibrium ──

/// V-3-mirroring scene constants — must track `hertz_sphere_plane.rs`
/// exactly. V-4 re-uses the coarsest refinement (h = 3 mm) only;
/// finer levels are gratuitous for hygiene scope.
mod v_3 {
    pub const RADIUS: f64 = 1.0e-2;
    pub const FORCE: f64 = 0.5;
    pub const MU: f64 = 2.0e5;
    pub const LAMBDA: f64 = 8.0e5;
    pub const CELL_SIZE: f64 = 3.0e-3;
    pub const STATIC_DT: f64 = 1.0;
    pub const MAX_NEWTON_ITER: usize = 50;
    pub const PENALTY_DHAT: f64 = 1.0e-3;
    pub const KAPPA: f64 = 1.0e3;
    /// Conservative `δ_pen = F / κ` per scope memo §1 V-4. Single-
    /// vertex worst-case overlap budget; at h = 3 mm only one vertex
    /// is typically active per V-3 commit-9 (single-pole regime),
    /// equivalent to the worst-case bound.
    pub const DELTA_PEN: f64 = FORCE / KAPPA;
}

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — V-3 sphere mesh + Hertz Newton at h = 3 mm coarsest level (~7 s \
              release per V-4 / V-3 commit-9 empirical, multi-minute debug)"
)]
#[test]
fn v_4_hertz_sphere_plane_non_interpenetration() {
    let material_field = MaterialField::uniform(v_3::MU, v_3::LAMBDA);
    let (mesh, bc, initial, _default_contact, theta) =
        SoftScene::sphere_on_plane(v_3::RADIUS, v_3::CELL_SIZE, v_3::FORCE, material_field)
            .expect("sphere_on_plane should mesh successfully at canonical params");
    // V-3-local κ override per `hertz_sphere_plane.rs` module docstring's
    // "Material plan change" section.
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), -(v_3::RADIUS + v_3::PENALTY_DHAT));
    let contact = PenaltyRigidContact::with_params(vec![plane], v_3::KAPPA, v_3::PENALTY_DHAT);

    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = v_3::STATIC_DT;
    cfg.max_newton_iter = v_3::MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    // signed_distance(p) = p · normal - offset, with normal = +ẑ and
    // offset = -(R + d̂). So sd = p_z - offset = p_z + R + d̂.
    let plane_offset = -(v_3::RADIUS + v_3::PENALTY_DHAT);
    let mut min_sd = f64::INFINITY;
    let mut min_sd_vertex: VertexId = 0;
    for &v in &referenced {
        let pz = step.x_final[3 * (v as usize) + 2];
        let sd = pz - plane_offset;
        if sd < min_sd {
            min_sd = sd;
            min_sd_vertex = v;
        }
    }

    eprintln!(
        "v_4 V-3 non-penetration: cell_size = {cs:e} m, n_referenced = {nr}, \
         κ = {kappa:e} N/m, d̂ = {dhat:e} m, F = {f} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        cs = v_3::CELL_SIZE,
        nr = n_referenced,
        kappa = v_3::KAPPA,
        dhat = v_3::PENALTY_DHAT,
        f = v_3::FORCE,
        dp = v_3::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -v_3::DELTA_PEN,
        "V-3 converged state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F / κ).",
        v = min_sd_vertex,
        sd = min_sd,
        dp = v_3::DELTA_PEN,
    );
}

// ── V-5 non-penetration: dropping sphere at static rest ──────────────────

/// V-5-mirroring scene constants — must track `contact_drop_rest.rs`
/// exactly so the `δ_pen` budget remains a faithful representation of
/// V-5's actual converged rest state.
mod v_5 {
    pub const RADIUS: f64 = 1.0e-2;
    pub const CELL_SIZE: f64 = 3.0e-3;
    pub const RELEASE_HEIGHT: f64 = 5.0e-2;
    pub const MU: f64 = 2.0e5;
    pub const LAMBDA: f64 = 8.0e5;
    pub const GRAVITY: f64 = -9.81;
    pub const DT: f64 = 1.0e-3;
    pub const N_STEPS: usize = 1000;
    pub const MAX_NEWTON_ITER: usize = 50;
    /// `F_max = m · g` for sphere body. Density carried by
    /// `SolverConfig::skeleton().density = 1030` kg/m³, sphere volume
    /// `4/3 π R³ ≈ 4.19e-6 m³` at `R = 1 cm` → `m ≈ 4.32e-3 kg`,
    /// `F_max ≈ 4.24e-2 N`.
    pub const F_MAX: f64 = 5.0e-2;
    pub const KAPPA: f64 = 1.0e4;
    pub const DELTA_PEN: f64 = F_MAX / KAPPA;
}

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — V-5 drop-and-rest at 1000 steps × dynamic Newton (~22 s release \
              per V-5 commit-10 empirical, multi-minute debug)"
)]
#[test]
fn v_4_dropping_sphere_non_interpenetration() {
    let material_field = MaterialField::uniform(v_5::MU, v_5::LAMBDA);
    let (mesh, bc, initial, contact) = SoftScene::dropping_sphere(
        v_5::RADIUS,
        v_5::CELL_SIZE,
        v_5::RELEASE_HEIGHT,
        material_field,
    )
    .expect("dropping_sphere should mesh successfully at canonical params");

    let n_dof = 3 * mesh.n_vertices();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = v_5::DT;
    cfg.gravity_z = v_5::GRAVITY;
    cfg.max_newton_iter = v_5::MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let SceneInitial { x_prev, v_prev } = initial;
    let mut x_state: Vec<f64> = x_prev.as_slice().to_vec();
    let mut v_state: Vec<f64> = v_prev.as_slice().to_vec();
    let theta = Tensor::from_slice(&[], &[0]);

    for _ in 0..v_5::N_STEPS {
        let x_in = Tensor::from_slice(&x_state, &[n_dof]);
        let v_in = Tensor::from_slice(&v_state, &[n_dof]);
        let step = solver.replay_step(&x_in, &v_in, &theta, v_5::DT);
        let v_new: Vec<f64> = step
            .x_final
            .iter()
            .zip(x_state.iter())
            .map(|(xf, xp)| (xf - xp) / v_5::DT)
            .collect();
        x_state = step.x_final;
        v_state = v_new;
    }

    // V-5 plane: normal = +ẑ, offset = 0 (table at z = 0). sd = p_z.
    let mut min_sd = f64::INFINITY;
    let mut min_sd_vertex: VertexId = 0;
    for &v in &referenced {
        let pz = x_state[3 * (v as usize) + 2];
        let sd = pz;
        if sd < min_sd {
            min_sd = sd;
            min_sd_vertex = v;
        }
    }

    eprintln!(
        "v_4 V-5 non-penetration: cell_size = {cs:e} m, n_referenced = {nr}, \
         κ = {kappa:e} N/m (default), F_max = m·g ≈ {fmax} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        cs = v_5::CELL_SIZE,
        nr = n_referenced,
        kappa = v_5::KAPPA,
        fmax = v_5::F_MAX,
        dp = v_5::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -v_5::DELTA_PEN,
        "V-5 rest state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F_max / κ where F_max = m·g).",
        v = min_sd_vertex,
        sd = min_sd,
        dp = v_5::DELTA_PEN,
    );
}
