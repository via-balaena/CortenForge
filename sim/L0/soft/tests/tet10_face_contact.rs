//! Tet10 ladder rung 8b — the surface-integrated **face** contact barrier.
//!
//! A quadratic (Tet10) mesh surfaces six-node boundary faces, so
//! [`IpcRigidContact::active_pairs`] emits [`ContactPair::Face`] pairs and the
//! barrier is integrated over each face (`E = A_rest · Σ_q ŵ_q b(sd(x_q))`,
//! weighted by the **rest** area so the force is purely along the SDF normal —
//! the consistent P2 load) instead of collocated per vertex. A linear (Tet4)
//! mesh keeps the per-vertex path, byte-identical to pre-rung-8b.
//!
//! These are the *integration* gates on top of the `contact::face` unit gates
//! (which FD-verify the primitive gradient and Hessian directly):
//!
//! 1. **Dispatch** — a Tet10 mesh emits `Face` pairs, a Tet4 mesh emits
//!    `Vertex` pairs, off the same primitive.
//! 2. **Solve** — a Tet10 block whose bottom face sits in the barrier band
//!    (top pinned) converges, the barrier pushes the free bottom nodes off the
//!    plane (all `sd > 0`), and the net contact force is `+z` (away from the
//!    plane).
//! 3. **Adjoint (gate iii)** — the differentiable Dirichlet-reaction JVP, which
//!    factors the tangent through `factor_at_position` (so its `A_ff` carries
//!    the face-barrier Hessian), matches a re-solve central FD. This is the
//!    integration counterpart of the primitive Hessian FD gate, exercising the
//!    real scatter + symbolic sparsity pattern.
//! 4. **Frictionless guard** — rung 8b ships face contact frictionless; a
//!    Tet10 face contact with `friction_mu > 0` fails loud.

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::{
    ActivePairsFor, BoundaryConditions, ContactModel, ContactPair, CpuNewtonSolver,
    HandBuiltTetMesh, IpcRigidContact, MaterialField, Mesh, RigidPlane, Solver, SolverConfig,
    Tet10Mesh, Vec3, VertexId,
};

const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
const STATIC_DT: f64 = 1.0e3;
const D_HAT: f64 = 0.01;
const KAPPA: f64 = 1.0e4;
// Plane below z = 0 so the bottom face rests at sd = STANDOFF ∈ (0, d̂).
const STANDOFF: f64 = 0.4 * D_HAT;

fn field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

/// Ground plane (normal +z) whose surface sits `STANDOFF` below the block's
/// bottom face, so the bottom nodes are inside the barrier band at rest.
fn plane() -> RigidPlane {
    RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), -STANDOFF)
}

/// Rest DOF vector of a mesh, flattened `[x,y,z,...]`.
fn rest_dofs(mesh: &dyn Mesh) -> Vec<f64> {
    let p = mesh.positions();
    let mut rest = vec![0.0_f64; 3 * p.len()];
    for (v, pt) in p.iter().enumerate() {
        rest[3 * v] = pt.x;
        rest[3 * v + 1] = pt.y;
        rest[3 * v + 2] = pt.z;
    }
    rest
}

/// Top-face corner ids of the base Tet4 cube (the pinned face).
fn top_corners(cube: &HandBuiltTetMesh) -> Vec<VertexId> {
    let pos = cube.positions();
    (0..cube.n_vertices() as VertexId)
        .filter(|&v| (pos[v as usize].z - EDGE).abs() < 1e-9)
        .collect()
}

#[test]
fn tet10_emits_face_pairs_and_tet4_emits_vertex_pairs() {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &field());
    let contact = IpcRigidContact::with_params(vec![plane()], KAPPA, D_HAT);

    // Tet4: per-vertex path.
    let pos4 = cube.positions().to_vec();
    let pairs4 =
        ActivePairsFor::<sim_soft::material::NeoHookean>::active_pairs(&contact, &cube, &pos4);
    assert!(!pairs4.is_empty(), "Tet4 must engage the plane");
    assert!(
        pairs4
            .iter()
            .all(|p| matches!(p, ContactPair::Vertex { .. })),
        "a linear mesh must emit only Vertex pairs",
    );

    // Tet10: face path.
    let mesh = Tet10Mesh::from_tet4(&cube);
    let pos10 = mesh.positions().to_vec();
    let pairs10 =
        ActivePairsFor::<sim_soft::material::NeoHookean>::active_pairs(&contact, &mesh, &pos10);
    assert!(!pairs10.is_empty(), "Tet10 must engage the plane");
    assert!(
        pairs10
            .iter()
            .all(|p| matches!(p, ContactPair::Face { .. })),
        "a quadratic mesh must emit only Face pairs",
    );
    // Every emitted face carries a positive rest-area weight.
    for p in &pairs10 {
        if let ContactPair::Face { rest_area, .. } = p {
            assert!(*rest_area > 0.0, "face rest area must be positive");
        }
    }
}

#[test]
fn tet10_face_contact_solve_pushes_off_plane() {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &field());
    let mesh = Tet10Mesh::from_tet4(&cube);
    let rest = rest_dofs(&mesh);
    let n = rest.len();

    let bc = BoundaryConditions {
        pinned_vertices: top_corners(&cube),
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 80;
    let contact = IpcRigidContact::with_params(vec![plane()], KAPPA, D_HAT);
    let solver = CpuNewtonSolver::new(Tet10, mesh, contact, cfg, bc);

    let v0 = vec![0.0_f64; n];
    let theta = Tensor::from_slice(&[], &[0]);
    let step = solver.replay_step(
        &Tensor::from_slice(&rest, &[n]),
        &Tensor::from_slice(&v0, &[n]),
        &theta,
        STATIC_DT,
    );
    let x = step.x_final;
    assert_eq!(x.len(), n, "solve produced a full DOF vector");

    // The barrier is finite only for sd > 0 — a converged solve keeps every node
    // strictly above the plane.
    let pl = plane();
    let mut min_sd = f64::INFINITY;
    for v in 0..n / 3 {
        let z = x[3 * v + 2];
        let sd = z + STANDOFF; // plane at z = -STANDOFF, normal +z
        let _ = pl;
        min_sd = min_sd.min(sd);
    }
    assert!(
        min_sd > 0.0,
        "a converged barrier solve must keep every node above the plane, min sd = {min_sd:e}",
    );

    // Net contact force on the soft body points away from the plane (+z).
    let positions: Vec<Vec3> = (0..n / 3)
        .map(|v| Vec3::new(x[3 * v], x[3 * v + 1], x[3 * v + 2]))
        .collect();
    let contact = IpcRigidContact::with_params(vec![plane()], KAPPA, D_HAT);
    let pairs = ActivePairsFor::<sim_soft::material::NeoHookean>::active_pairs(
        &contact,
        // rebuild a mesh view for active_pairs (rest positions drive rest area)
        &Tet10Mesh::from_tet4(&HandBuiltTetMesh::uniform_block(N, EDGE, &field())),
        &positions,
    );
    assert!(
        !pairs.is_empty(),
        "contact must still be engaged at equilibrium"
    );
    let mut net = Vec3::zeros();
    for pair in &pairs {
        // force_on_soft = -gradient; gradient = +∂E/∂x.
        for (_, g) in contact.gradient(pair, &positions).contributions {
            net -= g;
        }
    }
    assert!(
        net.z > 0.0 && net.z.abs() > net.x.abs() && net.z.abs() > net.y.abs(),
        "net contact force must push the body off the plane (+z dominant), got {net:?}",
    );
}

#[test]
fn tet10_face_contact_reaction_jvp_matches_resolve_fd() {
    // Gate (iii): the differentiable tangent (factor_at_position → A_ff carries
    // the face-barrier Hessian) is FD-consistent with a re-solve.
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &field());
    let mesh = Tet10Mesh::from_tet4(&cube);
    let rest = rest_dofs(&mesh);
    let n = rest.len();
    let top = top_corners(&cube);

    let bc = BoundaryConditions {
        pinned_vertices: top.clone(),
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 80;
    let contact = IpcRigidContact::with_params(vec![plane()], KAPPA, D_HAT);
    let solver = CpuNewtonSolver::new(Tet10, mesh, contact, cfg, bc);

    let v0 = vec![0.0_f64; n];
    let theta = Tensor::from_slice(&[], &[0]);
    let solve = |xp: &[f64]| -> Vec<f64> {
        solver
            .replay_step(
                &Tensor::from_slice(xp, &[n]),
                &Tensor::from_slice(&v0, &[n]),
                &theta,
                STATIC_DT,
            )
            .x_final
    };

    // Operating point: press the pinned top down slightly so the block is
    // compressed onto the plane (contact firmly active in the free block).
    let mut x_op = rest;
    for &v in &top {
        x_op[3 * v as usize + 2] -= 0.3 * D_HAT;
    }
    let x1 = solve(&x_op);

    // Perturb the pinned top targets; zero on the free DOFs (the JVP contract).
    let mut dir = vec![0.0_f64; n];
    for (k, &v) in top.iter().enumerate() {
        dir[3 * v as usize] = 0.2 * (((k % 3) as f64) - 1.0);
        dir[3 * v as usize + 2] = -0.5;
    }
    let an = solver.equilibrium_dirichlet_reaction_sensitivity(&x1, STATIC_DT, &dir);

    let pinned_dofs: Vec<usize> = top
        .iter()
        .flat_map(|&v| [3 * v as usize, 3 * v as usize + 1, 3 * v as usize + 2])
        .collect();

    let reaction = |xp: &[f64]| -> Vec<f64> {
        let xr = solve(xp);
        solver.nodal_reaction_forces(&xr, xp, STATIC_DT)
    };
    let rel_on = |idx: &[usize], a: &[f64], b: &[f64]| -> f64 {
        let num: f64 = idx.iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
        let den: f64 = idx.iter().map(|&i| b[i] * b[i]).sum();
        (num / den.max(1e-300)).sqrt()
    };
    let mut best = f64::INFINITY;
    for &e in &[1e-4, 1e-5, 1e-6, 1e-7] {
        let eps = e * EDGE;
        let xp: Vec<f64> = x_op.iter().zip(&dir).map(|(a, d)| a + eps * d).collect();
        let xm: Vec<f64> = x_op.iter().zip(&dir).map(|(a, d)| a - eps * d).collect();
        let rp = reaction(&xp);
        let rm = reaction(&xm);
        let fd: Vec<f64> = rp
            .iter()
            .zip(&rm)
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect();
        best = best.min(rel_on(&pinned_dofs, &an, &fd));
    }
    let mag = pinned_dofs.iter().map(|&i| an[i].abs()).fold(0.0, f64::max);
    eprintln!("Tet10 face-contact reaction JVP: ‖·‖_∞={mag:.3e} best_rel={best:.3e}");
    assert!(
        mag > 1.0,
        "reaction sensitivity implausibly small (contact inactive?)"
    );
    assert!(
        best < 1e-5,
        "Tet10 face-contact reaction JVP disagrees with re-solve FD: best_rel={best:e}",
    );
}

#[test]
#[should_panic(expected = "face contact with friction")]
fn tet10_face_contact_with_friction_panics() {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &field());
    let mesh = Tet10Mesh::from_tet4(&cube);
    let rest = rest_dofs(&mesh);
    let n = rest.len();

    let bc = BoundaryConditions {
        pinned_vertices: top_corners(&cube),
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 80;
    cfg.friction_mu = 0.3; // friction on a Tet10 face path is unreconciled
    let contact = IpcRigidContact::with_params(vec![plane()], KAPPA, D_HAT);
    let solver = CpuNewtonSolver::new(Tet10, mesh, contact, cfg, bc);

    let v0 = vec![0.0_f64; n];
    let theta = Tensor::from_slice(&[], &[0]);
    // The forward friction assembly fails loud on the first active Face pair.
    let _ = solver.replay_step(
        &Tensor::from_slice(&rest, &[n]),
        &Tensor::from_slice(&v0, &[n]),
        &theta,
        STATIC_DT,
    );
}
