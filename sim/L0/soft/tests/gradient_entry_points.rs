//! Which public gradient methods of `CpuNewtonSolver` run and which panic, with friction or
//! F-bar on.
//!
//! The rules, stated on `SolverConfig::friction_mu` and `SolverConfig::fbar`: with a nonzero
//! `friction_mu` the adjoint tangent needs the step-start position `x_prev`, so a method that
//! factors it without `x_prev` panics, and on Tet10 every method that factors it panics. With
//! `fbar` set, every method that factors it panics. This file calls every public
//! `CpuNewtonSolver` method that factors the adjoint tangent, with each setting on and off,
//! and checks each outcome against those rules. It checks only whether a call panics, not the
//! gradient's value. (`ReducedNewtonSolver::adjoint` factors its own tangent and has no such
//! checks; it is not called here.)

// let_underscore_must_use: each call is made only to see whether it panics, so its result
// is discarded on purpose.
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::too_many_lines
)]

use std::panic::{AssertUnwindSafe, catch_unwind};

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::element::Tet10;
use sim_soft::{
    ActivePairsFor, BoundaryConditions, ContactModel, CpuNewtonSolver, Element, HandBuiltTetMesh,
    LoadAxis, Material, MaterialField, Mesh, NullContact, PenaltyRigidContact, RigidPlane,
    RigidTwist, Solver, SolverConfig, Tet4, Tet10Mesh, Vec3, pick_vertices_by_predicate,
};

const EDGE: f64 = 0.1;
const DT: f64 = 1.0e-3;
const FRICTION_MU: f64 = 3.0;

const WITHOUT_X_PREV: &str = "friction-exact gradient requested without x_prev";
const TET10_FRICTION: &str = "Tet10 friction-exact gradients";
const FBAR: &str = "F-bar differentiable gradients are not yet supported";
const POSE_TRANSLATION_ONLY: &str = "friction pose sensitivity supports only a pure translation";

/// One call's outcome: the method, whether the call passed `x_prev`, and its panic message.
struct Outcome {
    method: &'static str,
    passes_x_prev: bool,
    panic: Option<String>,
}

fn record(out: &mut Vec<Outcome>, method: &'static str, passes_x_prev: bool, f: impl FnOnce()) {
    let panic = catch_unwind(AssertUnwindSafe(f)).err().map(|e| {
        e.downcast_ref::<String>()
            .cloned()
            .or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_string()))
            .unwrap_or_default()
    });
    out.push(Outcome {
        method,
        passes_x_prev,
        panic,
    });
}

/// Calls every public method that factors the adjoint tangent: `Solver::step`,
/// `Solver::try_step`, and the sensitivity and VJP methods.
fn call_every_gradient_path<E, Msh, C, M, const N: usize, const G: usize>(
    solver: &mut CpuNewtonSolver<E, Msh, C, M, N, G>,
    x: &[f64],
    x_prev: &[f64],
    theta: &[f64],
) -> Vec<Outcome>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    let n_dof = x.len();
    let zeros = vec![0.0_f64; n_dof];
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let up = Vec3::new(0.0, 0.0, 1.0);
    let twist = RigidTwist::translation(up);
    let weights = [1.0, 0.0];
    let mut out = Vec::new();

    record(&mut out, "Solver::step", false, || {
        let mut tape = Tape::new();
        let theta_var = tape.param_tensor(Tensor::from_slice(theta, &[theta.len()]));
        let _ = solver.step(
            &mut tape,
            &Tensor::from_slice(x_prev, &[n_dof]),
            &Tensor::from_slice(&zeros, &[n_dof]),
            theta_var,
            DT,
        );
    });
    record(&mut out, "Solver::try_step", false, || {
        let mut tape = Tape::new();
        let theta_var = tape.param_tensor(Tensor::from_slice(theta, &[theta.len()]));
        let _ = solver.try_step(
            &mut tape,
            &Tensor::from_slice(x_prev, &[n_dof]),
            &Tensor::from_slice(&zeros, &[n_dof]),
            theta_var,
            DT,
        );
    });

    let s = &*solver;
    record(&mut out, "material_step_vjp", false, || {
        let _ = s.material_step_vjp(x, DT, 0);
    });
    record(&mut out, "state_step_vjp", false, || {
        let _ = s.state_step_vjp(x, DT);
    });
    record(&mut out, "trajectory_step_vjp", false, || {
        let _ = s.trajectory_step_vjp(x, DT, 0, up);
    });
    record(&mut out, "trajectory_step_vjp_twist", false, || {
        let _ = s.trajectory_step_vjp_twist(x, DT, 0, &[twist]);
    });
    record(&mut out, "trajectory_step_vjp_combined", false, || {
        let _ = s.trajectory_step_vjp_combined(x, DT, &weights, up);
    });
    record(
        &mut out,
        "equilibrium_dirichlet_reaction_sensitivity",
        false,
        || {
            let _ = s.equilibrium_dirichlet_reaction_sensitivity(x, DT, &zeros);
        },
    );
    record(
        &mut out,
        "equilibrium_dirichlet_reaction_vjp",
        false,
        || {
            let _ = s.equilibrium_dirichlet_reaction_vjp(x, DT, &zeros);
        },
    );
    record(
        &mut out,
        "equilibrium_pose_sensitivity(None)",
        false,
        || {
            let _ = s.equilibrium_pose_sensitivity(x, None, DT, twist);
        },
    );
    record(
        &mut out,
        "equilibrium_material_sensitivity(None)",
        false,
        || {
            let _ = s.equilibrium_material_sensitivity(x, None, DT, 0);
        },
    );
    record(
        &mut out,
        "equilibrium_state_sensitivity(None)",
        false,
        || {
            let _ = s.equilibrium_state_sensitivity(x, None, DT, &zeros, &zeros);
        },
    );

    record(&mut out, "equilibrium_pose_sensitivity(Some)", true, || {
        let _ = s.equilibrium_pose_sensitivity(x, Some(x_prev), DT, twist);
    });
    record(
        &mut out,
        "equilibrium_material_sensitivity(Some)",
        true,
        || {
            let _ = s.equilibrium_material_sensitivity(x, Some(x_prev), DT, 0);
        },
    );
    record(
        &mut out,
        "equilibrium_state_sensitivity(Some)",
        true,
        || {
            let _ = s.equilibrium_state_sensitivity(x, Some(x_prev), DT, &zeros, &zeros);
        },
    );
    record(&mut out, "equilibrium_drift_sensitivity", true, || {
        let _ = s.equilibrium_drift_sensitivity(x, x_prev, DT, dir);
    });
    record(
        &mut out,
        "equilibrium_friction_coeff_sensitivity",
        true,
        || {
            let _ = s.equilibrium_friction_coeff_sensitivity(x, x_prev, DT);
        },
    );
    record(&mut out, "trajectory_step_vjp_grip", true, || {
        let _ = s.trajectory_step_vjp_grip(x, x_prev, DT, 0, up, dir);
    });
    record(&mut out, "trajectory_step_vjp_grip_centre", true, || {
        let _ = s.trajectory_step_vjp_grip_centre(x, x_prev, DT, 0, &[up], dir);
    });
    record(
        &mut out,
        "trajectory_step_vjp_grip_fric_coeff",
        true,
        || {
            let _ = s.trajectory_step_vjp_grip_fric_coeff(x, x_prev, DT, up, dir);
        },
    );
    record(
        &mut out,
        "trajectory_step_vjp_grip_fric_coeff_centre",
        true,
        || {
            let _ = s.trajectory_step_vjp_grip_fric_coeff_centre(x, x_prev, DT, &[up], dir);
        },
    );
    record(&mut out, "trajectory_step_vjp_grip_combined", true, || {
        let _ = s.trajectory_step_vjp_grip_combined(x, x_prev, DT, &weights, up, dir);
    });
    record(
        &mut out,
        "trajectory_step_vjp_grip_combined_centre",
        true,
        || {
            let _ = s.trajectory_step_vjp_grip_combined_centre(x, x_prev, DT, &weights, &[up], dir);
        },
    );
    out
}

/// A Tet4 block pressed into a ceiling plane and dragged sideways, so friction is active
/// at the converged step (the `tests/friction_diff.rs` scene).
struct Tet4Scene {
    solver: CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact>,
    x_rest: Vec<f64>,
    theta: Vec<f64>,
}

fn tet4_scene(friction_mu: f64, fbar: bool) -> Tet4Scene {
    let mesh = HandBuiltTetMesh::uniform_block(4, EDGE, &MaterialField::uniform(3.0e4, 1.2e5));
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top: Vec<usize> = (0..rest.len())
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect();
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -EDGE);
    let contact = PenaltyRigidContact::with_params([plane], 5.0e3, 5.0e-3);
    let loaded: Vec<(u32, LoadAxis)> = top
        .iter()
        .map(|&i| (i as u32, LoadAxis::FullVector))
        .collect();
    let bc = BoundaryConditions::new(pinned, loaded);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = 10.0;
    cfg.friction_mu = friction_mu;
    cfg.friction_eps_v = 0.1;
    cfg.fbar = fbar;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;

    let x_rest = rest.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let mut theta = vec![0.0_f64; 3 * top.len()];
    for k in 0..top.len() {
        theta[3 * k] = 40.0 / top.len() as f64;
    }
    Tet4Scene {
        solver: CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc),
        x_rest,
        theta,
    }
}

/// Runs every gradient path on the Tet4 scene at its converged step.
fn tet4_outcomes(friction_mu: f64) -> Vec<Outcome> {
    let mut scene = tet4_scene(friction_mu, false);
    let n_dof = scene.x_rest.len();
    let x_final = scene
        .solver
        .replay_step(
            &Tensor::from_slice(&scene.x_rest, &[n_dof]),
            &Tensor::from_slice(&vec![0.0_f64; n_dof], &[n_dof]),
            &Tensor::from_slice(&scene.theta, &[scene.theta.len()]),
            DT,
        )
        .x_final;
    if friction_mu > 0.0 {
        let forces = scene
            .solver
            .friction_forces_on_soft(&x_final, &scene.x_rest, DT);
        assert!(
            forces.iter().any(|(_, f)| f.norm() > 0.0),
            "the fixture must have friction acting at its converged step"
        );
    }
    call_every_gradient_path(&mut scene.solver, &x_final, &scene.x_rest, &scene.theta)
}

/// Every call that passes no `x_prev` panics with friction on; every call that passes it runs.
#[test]
fn tet4_with_friction_panics_exactly_where_x_prev_is_missing() {
    let outcomes = tet4_outcomes(FRICTION_MU);
    for o in &outcomes {
        if o.passes_x_prev {
            assert!(o.panic.is_none(), "{} panicked: {:?}", o.method, o.panic);
        } else {
            let msg = o.panic.as_deref().unwrap_or_default();
            assert!(
                msg.contains(WITHOUT_X_PREV),
                "{} should panic for want of x_prev, got {:?}",
                o.method,
                o.panic
            );
        }
    }
    assert!(outcomes.iter().any(|o| o.passes_x_prev));
    assert!(outcomes.iter().any(|o| !o.passes_x_prev));
}

/// The control: with friction off, the same calls on the same scene all run, so the panics
/// above come from friction.
#[test]
fn tet4_without_friction_runs_every_path() {
    for o in &tet4_outcomes(0.0) {
        assert!(o.panic.is_none(), "{} panicked: {:?}", o.method, o.panic);
    }
}

/// With friction, the pose sensitivity given `Some(x_prev)` takes a translation twist only:
/// an angular twist panics. Without friction the same call runs.
#[test]
fn tet4_friction_pose_sensitivity_takes_translations_only() {
    let angular = RigidTwist {
        linear: Vec3::zeros(),
        angular: Vec3::new(1.0, 0.0, 0.0),
    };
    for (friction_mu, expected) in [(FRICTION_MU, Some(POSE_TRANSLATION_ONLY)), (0.0, None)] {
        let scene = tet4_scene(friction_mu, false);
        let x = &scene.x_rest;
        let mut out = Vec::new();
        record(
            &mut out,
            "equilibrium_pose_sensitivity(Some, angular)",
            true,
            || {
                let _ = scene
                    .solver
                    .equilibrium_pose_sensitivity(x, Some(x), DT, angular);
            },
        );
        let panic = out[0].panic.as_deref();
        match expected {
            Some(msg) => assert!(
                panic.is_some_and(|p| p.contains(msg)),
                "μ = {friction_mu}: expected the translation-only panic, got {panic:?}"
            ),
            None => assert!(panic.is_none(), "μ = {friction_mu}: panicked: {panic:?}"),
        }
    }
}

/// With `fbar` set, every path panics on the Tet4 scene, with or without `x_prev`. The
/// calls are made at the rest positions: the guard fires before the position is used.
#[test]
fn tet4_with_fbar_panics_on_every_path() {
    let mut scene = tet4_scene(0.0, true);
    let x = scene.x_rest.clone();
    for o in &call_every_gradient_path(&mut scene.solver, &x, &x, &scene.theta) {
        let msg = o.panic.as_deref().unwrap_or_default();
        assert!(
            msg.contains(FBAR),
            "{} (passes x_prev: {}) should refuse F-bar, got {:?}",
            o.method,
            o.passes_x_prev,
            o.panic
        );
    }
}

/// A Tet10 block on its bottom face, with no contact. Friction is set, so every path that
/// factors the adjoint tangent must refuse, whether or not it is given `x_prev`.
fn tet10_outcomes(friction_mu: f64) -> Vec<Outcome> {
    let cube = HandBuiltTetMesh::uniform_block(2, EDGE, &MaterialField::uniform(1.0e5, 4.0e5));
    let corners = cube.positions().to_vec();
    let pinned: Vec<u32> = (0..corners.len() as u32)
        .filter(|&v| corners[v as usize].z < 0.25 * EDGE)
        .collect();
    let mesh = Tet10Mesh::from_tet4(&cube);
    let x_rest: Vec<f64> = mesh
        .positions()
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e-2;
    cfg.friction_mu = friction_mu;
    let mut solver = CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
    call_every_gradient_path(&mut solver, &x_rest, &x_rest, &[])
}

#[test]
fn tet10_with_friction_panics_on_every_path() {
    for o in &tet10_outcomes(FRICTION_MU) {
        let msg = o.panic.as_deref().unwrap_or_default();
        assert!(
            msg.contains(TET10_FRICTION),
            "{} (passes x_prev: {}) should refuse Tet10 friction, got {:?}",
            o.method,
            o.passes_x_prev,
            o.panic
        );
    }
}

/// The control: the same Tet10 calls all run with friction off.
#[test]
fn tet10_without_friction_runs_every_path() {
    for o in &tet10_outcomes(0.0) {
        assert!(o.panic.is_none(), "{} panicked: {:?}", o.method, o.panic);
    }
}
