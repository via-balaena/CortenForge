//! Keystone friction leaf `PR3a` (sim-soft half) — the MOVING-COLLIDER friction drift.
//!
//! PR1's friction is one-way: it measures a soft vertex's tangential slip against a
//! STATIC rigid surface, so a collider that slides tangentially never drags the soft
//! body (a tangentially-invariant half-space exerts no signal). `PR3a`'s grip needs the
//! collider's within-step tangential drift `Δ_surf` in the friction reference,
//! `u_T = Tⁿᵀ((x_v − xᵗ) − Δ_surf)`, so a moving collider DRAGS the soft body — the
//! two-way grip a sliding device exerts on a held limb.
//!
//! This isolates that term in a pure sim-soft forward solve (no coupling): a pinned-bottom
//! block pressed into a ceiling plane by upward gravity, with the ceiling given a lateral
//! drift `Δ_surf`. With no drift the top does not move tangentially; with drift it is
//! dragged toward `Δ_surf`, monotonically in `μ`. Hand-checked sign + magnitude of the
//! per-pair force live in `contact::friction`'s `drift_drags_resting_vertex_in_drift_direction`.
//! Forward-only (the friction-coupled gradient is `PR3b`). See `project-friction-leaf.md`.

// A contact solve on a valid hand-built scene does not fail; index/count casts on a small
// block are exact.
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    PenaltyRigidContact, RigidPlane, Solver, SolverConfig, Tet4, Vec3, pick_vertices_by_predicate,
};

const EDGE: f64 = 0.1;
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 5.0e3;
const GRAV_UP: f64 = 10.0; // +z gravity presses the block into the ceiling (normal load)
const EPS_V: f64 = 0.1; // w = dt·ε_v = 1e-4 m (PR1's converging stick-band choice)

/// Mean tangential (x) displacement of the contact (top) vertices after ONE forward step
/// with Coulomb coefficient `mu` and a lateral ceiling drift `drift_x` (m, the within-step
/// tangential sweep of the collider). No external tangential load — the only tangential
/// drive is the collider's drift.
fn top_drag(mu: f64, drift_x: f64) -> f64 {
    let field = MaterialField::uniform(3.0e4, 1.2e5);
    let mesh = HandBuiltTetMesh::uniform_block(4, EDGE, &field);
    let n = mesh.n_vertices();
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top: Vec<usize> = (0..n)
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect();

    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -EDGE); // ceiling at the block top
    let contact = PenaltyRigidContact::with_params([plane], KAPPA, 5.0e-3);
    let bc = BoundaryConditions::new(pinned, Vec::new());
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = mu;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    let solver = CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(drift_x, 0.0, 0.0));

    let mut x0 = vec![0.0_f64; 3 * n];
    for (i, p) in rest.iter().enumerate() {
        x0[3 * i] = p.x;
        x0[3 * i + 1] = p.y;
        x0[3 * i + 2] = p.z;
    }
    let v0 = vec![0.0_f64; 3 * n];
    let x_final = solver
        .replay_step(
            &Tensor::from_slice(&x0, &[3 * n]),
            &Tensor::from_slice(&v0, &[3 * n]),
            &Tensor::zeros(&[0]),
            DT,
        )
        .x_final;
    let sum: f64 = top.iter().map(|&i| x_final[3 * i] - rest[i].x).sum();
    sum / top.len() as f64
}

/// The drift-INDUCED tangential drag at coefficient `mu`: the top's x-displacement WITH the
/// collider drift minus the no-drift baseline. Subtracting the baseline removes the tiny
/// BCC-mesh settling asymmetry (PR1's `induced_slide` pattern), isolating the moving-collider
/// term. At `mu = 0` this is identically zero (the drift only enters via friction).
fn induced_drag(mu: f64, drift: f64) -> f64 {
    top_drag(mu, drift) - top_drag(mu, 0.0)
}

#[test]
fn moving_collider_drags_the_block_via_friction() {
    let drift = 5.0e-4; // 5·w into the saturated (full-cone) slip regime
    // With friction OFF the collider's tangential drift does nothing (PR1 floor): the drift
    // enters only through the friction term, so it cancels exactly in the induced metric.
    let off = induced_drag(0.0, drift);
    // With friction ON the moving collider DRAGS the top toward +x.
    let on = induced_drag(2.0, drift);
    println!("drift-induced drag: frictionless = {off:.3e} m;  μ=2 = {on:.3e} m");
    assert!(
        off.abs() < 1e-12,
        "frictionless: a sliding collider must not drag the block, got {off:.3e}"
    );
    assert!(
        on > 1e-5,
        "friction: the moving collider must drag the top in +x, got {on:.3e}"
    );
}

#[test]
fn drag_increases_monotonically_with_friction() {
    let drift = 5.0e-4;
    let drags: Vec<f64> = [0.0, 0.5, 1.0, 2.0, 3.0]
        .iter()
        .map(|&m| induced_drag(m, drift))
        .collect();
    println!("induced collider-drag vs μ: {drags:?}");
    assert!(
        drags[0].abs() < 1e-12,
        "μ=0 induced drag must be exactly zero"
    );
    for w in drags.windows(2) {
        assert!(
            w[1] >= w[0] - 1e-9,
            "more friction must not decrease the collider drag: {drags:?}"
        );
    }
    assert!(drags[4] > 1e-5, "μ=3 must drag the top materially");
}

/// Forward-only guard: a DIFFERENTIABLE path (here the forward material sensitivity, which
/// passes `Some(x_prev)` and builds the IFT-adjoint factor) must PANIC when a nonzero
/// collider drift is combined with friction — the adjoint's Woodbury term is built at the
/// un-drifted reference, so it would return a gradient inconsistent with the drift-shifted
/// forward solve. `PR3b` threads the drift through the adjoint; until then `replay_step`
/// (forward, exercised by the gates above) is the only drift-supporting path.
#[test]
#[should_panic(expected = "moving-collider friction drift gradient is not supported")]
fn drift_gradient_path_panics_until_pr3b() {
    let field = MaterialField::uniform(3.0e4, 1.2e5);
    let mesh = HandBuiltTetMesh::uniform_block(4, EDGE, &field);
    let n = mesh.n_vertices();
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -EDGE);
    let contact = PenaltyRigidContact::with_params([plane], KAPPA, 5.0e-3);
    let bc = BoundaryConditions::new(pinned, Vec::new());
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = 2.0;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    let solver = CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(5.0e-4, 0.0, 0.0));

    let mut x0 = vec![0.0_f64; 3 * n];
    for (i, p) in rest.iter().enumerate() {
        x0[3 * i] = p.x;
        x0[3 * i + 1] = p.y;
        x0[3 * i + 2] = p.z;
    }
    let v0 = vec![0.0_f64; 3 * n];
    let x_final = solver
        .replay_step(
            &Tensor::from_slice(&x0, &[3 * n]),
            &Tensor::from_slice(&v0, &[3 * n]),
            &Tensor::zeros(&[0]),
            DT,
        )
        .x_final;
    // Passing `Some(x_prev)` reaches the adjoint factor → the drift guard fires.
    let _sensitivity = solver.equilibrium_material_sensitivity(&x_final, Some(&x0), DT, 0);
}
