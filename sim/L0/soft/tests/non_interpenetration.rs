//! Non-interpenetration to documented penalty tolerance.
//!
//! At converged steady-state of any contact-active scene
//! (compressive block, Hertzian sphere, drop-and-rest), every soft
//! vertex satisfies `signed_distance ≥ -δ_pen` for the documented
//! penalty tolerance `δ_pen = F_max / κ_pen`. Penalty cannot enforce
//! zero overlap (book §00 §00 cited); this fixture is the gate
//! documenting what penalty *can* enforce — bounded overlap.
//!
//! ## Why this matters
//!
//! The compressive-block / Hertzian / drop-and-rest fixtures each
//! verify a *different* scientific property: force-pumping correctness,
//! Hertz contact-patch geometry, drop-and-rest dynamics under gravity.
//! None of them directly checks the non-penetration property the book
//! §00 §00 explicitly names as penalty's structural failure mode. This
//! fixture fills that gap with a unified all-vertex signed-distance
//! walk against each scene's converged state.
//!
//! ## `δ_pen` formula and scene-specific values
//!
//! `δ_pen = F_max / κ_pen` is the single-vertex worst-case penalty
//! overlap tolerance — derived from the equilibrium force balance
//! `κ · pen_v = F_v` where a single active vertex bears the entire
//! applied load. In practice multi-vertex contact distributes the
//! load (`pen_v ≈ F / (κ · N_active)` per active vertex) and actual
//! overlap is much smaller; the worst-case bound is conservative by
//! construction.
//!
//! - **Compressive block** — local override `κ = 1e4 N/m`,
//!   `F_max ≈ 0.18 N` (the integrated reaction force at the
//!   compressive-block fixture's canonical NH params);
//!   `δ_pen ≈ 1.8e-5 m`. Multi-vertex distribution over
//!   `(n+1)² = 25` active pairs at `n = 4` gives
//!   `pen_v ≈ 7.2e-7 m` per vertex; the worst-case bound has > 25×
//!   headroom.
//! - **Hertzian sphere** — fixture-local override `κ = 1e3 N/m`,
//!   `F = 0.5 N`; `δ_pen = 5e-4 m`. Multi-vertex distribution at h/2
//!   (~5 active pairs) gives `pen_v ≈ 1e-4 m`; worst-case bound has
//!   5× headroom. (This fixture re-uses the Hertzian fixture's
//!   coarsest refinement `h = 3 mm` for fastest release-mode runtime;
//!   at `h` only 1 vertex is active, equivalent to the worst-case
//!   bound.)
//! - **Drop-and-rest** — default `κ = 1e4 N/m`, `F_max = m · g` where
//!   `m = ρ · V_sphere ≈ 4.32e-3 kg` (silicone-class `ρ = 1030` kg/m³
//!   at `R = 1 cm`) and `g = 9.81` m/s² → `F_max = 4.24e-2 N`.
//!   `δ_pen ≈ 4.24e-6 m`. The static-equilibrium contact patch at
//!   rest absorbs gravity through ~5-10 active vertices; per-vertex
//!   pen is sub-μm.
//!
//! ## Three test fns, scene-specific gating
//!
//! The compressive-block scene is debug-feasible (cube at `n = 4` is
//! sub-second release-mode); the Hertzian and drop-and-rest scenes
//! inherit their parent fixtures' `#[cfg_attr(debug_assertions,
//! ignore)]` release-only gate per `feedback_release_mode_heavy_tests`.
//! Each test re-runs its scene at the canonical parameters and walks
//! every referenced vertex at converged `x_final` to assert
//! `signed_distance ≥ -δ_pen`.
//!
//! ## Why this doesn't co-locate with the contact-active fixtures
//!
//! Non-penetration lives in this stand-alone file rather than as
//! assertions tacked onto the compressive-block / Hertzian /
//! drop-and-rest fixtures. Co-locating would couple the gates — a
//! force-pumping regression would mask a non-penetration regression
//! and vice versa. Keeping them separate isolates failure modes for
//! diagnosis.

#![allow(
    // Helpers return `Result<tuple, MeshingError>`; `expect_used` is
    // needed at the helper-call site. Mirrors `hertz_sphere_plane.rs`
    // / `contact_drop_rest.rs` / Phase 4
    // precedent.
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver,
    SolverConfig, Tet4, Vec3, VertexId, referenced_vertices,
};

// ── Compressive-block non-penetration: cube against plane at equilibrium ─

/// compressive-block-mirroring scene constants — must track
/// `penalty_compressive_block.rs` exactly so the `δ_pen` budget remains
/// a faithful representation of the compressive block's actual
/// converged state.
mod compressive_block {
    pub const EDGE_LEN: f64 = 0.01;
    pub const DISPLACEMENT: f64 = 5.0e-5;
    pub const MU: f64 = 1.0e5;
    pub const LAMBDA: f64 = 4.0e5;
    pub const KAPPA: f64 = 1.0e4;
    pub const D_HAT_OVERRIDE: f64 = 1.0e-5;
    pub const STATIC_DT: f64 = 1.0;
    pub const MAX_NEWTON_ITER: usize = 50;
    /// Refinement: `n = 4` mid-level (matches the compressive block's
    /// middle refinement; faster than `n = 8` for hygiene scope, finer
    /// than `n = 2` for adequate vertex coverage).
    pub const N_PER_EDGE: usize = 4;
    /// Conservative `δ_pen = F_max / κ`. `F_max` per the compressive
    /// block's finest-level empirical (~0.18 N at n=8); n=4 is similar
    /// by Cauchy convergence (~0.186 N).
    pub const F_MAX: f64 = 0.20;
    pub const DELTA_PEN: f64 = F_MAX / KAPPA;
}

#[test]
fn compressive_block_non_interpenetration() {
    // Cast safe: N_PER_EDGE is a small literal; conversion to f64 is
    // loss-free.
    #[allow(clippy::cast_precision_loss)]
    let cell_size = compressive_block::EDGE_LEN / (compressive_block::N_PER_EDGE as f64);
    let material_field = MaterialField::uniform(compressive_block::MU, compressive_block::LAMBDA);
    let (mesh, bc, initial, _default_contact) = SoftScene::compressive_block_on_plane(
        compressive_block::EDGE_LEN,
        cell_size,
        compressive_block::DISPLACEMENT,
        &material_field,
    );
    // Fixture-local override: κ default but d̂ = 1e-5 (the compressive
    // block's deviation 2
    // per `penalty_compressive_block.rs` module docstring). Plane
    // reconstructed identically to `SoftScene::compressive_block_on_plane`'s
    // helper construction.
    let plane = RigidPlane::new(
        Vec3::new(0.0, 0.0, -1.0),
        compressive_block::DISPLACEMENT - compressive_block::EDGE_LEN,
    );
    let contact = PenaltyRigidContact::with_params(
        vec![plane],
        compressive_block::KAPPA,
        compressive_block::D_HAT_OVERRIDE,
    );

    let n_dof = 3 * mesh.n_vertices();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = compressive_block::STATIC_DT;
    cfg.max_newton_iter = compressive_block::MAX_NEWTON_ITER;

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
    let plane_offset = compressive_block::DISPLACEMENT - compressive_block::EDGE_LEN;
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
        "compressive-block non-penetration: n_per_edge = {n}, n_vertices = {nv}, \
         κ = {kappa:e} N/m, d̂_override = {dhat:e} m, F_max = {fmax} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        n = compressive_block::N_PER_EDGE,
        nv = n_vertices,
        kappa = compressive_block::KAPPA,
        dhat = compressive_block::D_HAT_OVERRIDE,
        fmax = compressive_block::F_MAX,
        dp = compressive_block::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -compressive_block::DELTA_PEN,
        "compressive-block converged state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F_max / κ). Penalty cannot enforce \
         zero overlap (book §00 §00) but should bound overlap by F_max/κ at single-vertex \
         worst-case; multi-vertex contact at this scene's (n+1)² = {expected_active} active \
         pairs makes per-vertex penetration much smaller than worst-case in practice.",
        v = min_sd_vertex,
        sd = min_sd,
        dp = compressive_block::DELTA_PEN,
        expected_active = (compressive_block::N_PER_EDGE + 1) * (compressive_block::N_PER_EDGE + 1),
    );
}

// ── Hertzian non-penetration: sphere on plane at converged equilibrium ───

/// Hertzian-mirroring scene constants — must track `hertz_sphere_plane.rs`
/// exactly. This fixture re-uses the coarsest refinement (h = 3 mm) only;
/// finer levels are gratuitous for hygiene scope.
mod hertz_sphere {
    pub const RADIUS: f64 = 1.0e-2;
    pub const FORCE: f64 = 0.5;
    pub const MU: f64 = 2.0e5;
    pub const LAMBDA: f64 = 8.0e5;
    pub const CELL_SIZE: f64 = 3.0e-3;
    pub const STATIC_DT: f64 = 1.0;
    pub const MAX_NEWTON_ITER: usize = 50;
    pub const PENALTY_DHAT: f64 = 1.0e-3;
    pub const KAPPA: f64 = 1.0e3;
    /// Conservative `δ_pen = F / κ`. Single-vertex worst-case overlap
    /// budget; at h = 3 mm only one vertex is typically active in the
    /// Hertzian fixture's coarsest refinement (single-pole regime),
    /// equivalent to the worst-case bound.
    pub const DELTA_PEN: f64 = FORCE / KAPPA;
}

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — Hertzian sphere mesh + Hertz Newton at h = 3 mm coarsest level (~7 s \
              release empirical, multi-minute debug)"
)]
#[test]
fn hertz_sphere_plane_non_interpenetration() {
    let material_field = MaterialField::uniform(hertz_sphere::MU, hertz_sphere::LAMBDA);
    let (mesh, bc, initial, _default_contact, theta) = SoftScene::sphere_on_plane(
        hertz_sphere::RADIUS,
        hertz_sphere::CELL_SIZE,
        hertz_sphere::FORCE,
        material_field,
    )
    .expect("sphere_on_plane should mesh successfully at canonical params");
    // Fixture-local κ override per `hertz_sphere_plane.rs` module docstring's
    // "Material plan change" section.
    let plane = RigidPlane::new(
        Vec3::new(0.0, 0.0, 1.0),
        -(hertz_sphere::RADIUS + hertz_sphere::PENALTY_DHAT),
    );
    let contact = PenaltyRigidContact::with_params(
        vec![plane],
        hertz_sphere::KAPPA,
        hertz_sphere::PENALTY_DHAT,
    );

    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = hertz_sphere::STATIC_DT;
    cfg.max_newton_iter = hertz_sphere::MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    // signed_distance(p) = p · normal - offset, with normal = +ẑ and
    // offset = -(R + d̂). So sd = p_z - offset = p_z + R + d̂.
    let plane_offset = -(hertz_sphere::RADIUS + hertz_sphere::PENALTY_DHAT);
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
        "Hertzian non-penetration: cell_size = {cs:e} m, n_referenced = {nr}, \
         κ = {kappa:e} N/m, d̂ = {dhat:e} m, F = {f} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        cs = hertz_sphere::CELL_SIZE,
        nr = n_referenced,
        kappa = hertz_sphere::KAPPA,
        dhat = hertz_sphere::PENALTY_DHAT,
        f = hertz_sphere::FORCE,
        dp = hertz_sphere::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -hertz_sphere::DELTA_PEN,
        "Hertzian converged state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F / κ).",
        v = min_sd_vertex,
        sd = min_sd,
        dp = hertz_sphere::DELTA_PEN,
    );
}

// ── Drop-and-rest non-penetration: dropping sphere at static rest ────────

/// drop-and-rest-mirroring scene constants — must track `contact_drop_rest.rs`
/// exactly so the `δ_pen` budget remains a faithful representation of
/// the drop-and-rest fixture's actual converged rest state.
mod drop_rest {
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
    ignore = "release-only — drop-and-rest at 1000 steps × dynamic Newton (~22 s release \
              empirical, multi-minute debug)"
)]
#[test]
fn dropping_sphere_non_interpenetration() {
    let material_field = MaterialField::uniform(drop_rest::MU, drop_rest::LAMBDA);
    let (mesh, bc, initial, contact) = SoftScene::dropping_sphere(
        drop_rest::RADIUS,
        drop_rest::CELL_SIZE,
        drop_rest::RELEASE_HEIGHT,
        material_field,
    )
    .expect("dropping_sphere should mesh successfully at canonical params");

    let n_dof = 3 * mesh.n_vertices();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = drop_rest::DT;
    cfg.gravity_z = drop_rest::GRAVITY;
    cfg.max_newton_iter = drop_rest::MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let SceneInitial { x_prev, v_prev } = initial;
    let mut x_state: Vec<f64> = x_prev.as_slice().to_vec();
    let mut v_state: Vec<f64> = v_prev.as_slice().to_vec();
    let theta = Tensor::from_slice(&[], &[0]);

    for _ in 0..drop_rest::N_STEPS {
        let x_in = Tensor::from_slice(&x_state, &[n_dof]);
        let v_in = Tensor::from_slice(&v_state, &[n_dof]);
        let step = solver.replay_step(&x_in, &v_in, &theta, drop_rest::DT);
        let v_new: Vec<f64> = step
            .x_final
            .iter()
            .zip(x_state.iter())
            .map(|(xf, xp)| (xf - xp) / drop_rest::DT)
            .collect();
        x_state = step.x_final;
        v_state = v_new;
    }

    // Plane: normal = +ẑ, offset = 0 (table at z = 0). sd = p_z.
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
        "drop-and-rest non-penetration: cell_size = {cs:e} m, n_referenced = {nr}, \
         κ = {kappa:e} N/m (default), F_max = m·g ≈ {fmax} N, δ_pen = {dp:e} m; \
         min sd = {sd:.4e} m at vertex {v} (penetration = {pen:.4e} m)",
        cs = drop_rest::CELL_SIZE,
        nr = n_referenced,
        kappa = drop_rest::KAPPA,
        fmax = drop_rest::F_MAX,
        dp = drop_rest::DELTA_PEN,
        sd = min_sd,
        v = min_sd_vertex,
        pen = (-min_sd).max(0.0),
    );

    assert!(
        min_sd >= -drop_rest::DELTA_PEN,
        "drop-and-rest rest state violates non-penetration: vertex {v} sd = {sd:.4e} m exceeds \
         documented penalty tolerance δ_pen = {dp:.4e} m (F_max / κ where F_max = m·g).",
        v = min_sd_vertex,
        sd = min_sd,
        dp = drop_rest::DELTA_PEN,
    );
}
