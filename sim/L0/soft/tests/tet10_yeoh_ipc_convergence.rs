//! Renovation item 3, step 0 — does `Tet10Mesh<Yeoh>` converge under the IPC
//! face barrier at all?
//!
//! Item 1 (`6dc7ee03`) made `Tet10Mesh<M>` generic, so `Tet10Mesh<Yeoh>` is
//! **constructible**. Constructible is not converged: the only Tet10 + IPC
//! solver instantiation in the tree is `tests/tet10_indentation_demand1.rs`,
//! and its material is `NeoHookean`. Before item 3 rewires the 6 416-line
//! `tools/cf-sim-research/src/insertion_sim.rs`, this file answers the cheap
//! question on a fixture that runs in seconds: does the triple (Tet10 element
//! × Yeoh material × IPC face barrier) solve, and at what Newton cost relative
//! to the two baselines that already work — Tet4 × Yeoh, and Tet10 ×
//! `NeoHookean`?
//!
//! ## The fixture
//!
//! A 40 × 40 × 12 mm plate of Ecoflex 00-30, SDF-meshed through the same
//! `SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh` path `insertion_sim` uses, then
//! enriched to Tet10 and pressed onto a rigid plane held inside the barrier
//! band. Top face pinned, one quasi-static step.
//!
//! ## What it found, measured 2026-09-20 at `d12e3bf9`
//!
//! - **It converges.** `Tet10Mesh<Yeoh>` + `IpcRigidContact` solves at rest
//!   contact in 2 Newton iterations, residual 5.9e-13, over 4 240 tets /
//!   24 993 DOF, with 961 active face pairs and a standoff of 0.483 mm.
//! - **The face path is genuinely selected**, asserted on
//!   `Mesh::boundary_faces6` — the selector, not the types, per the recon
//!   doc's §9 correction.
//! - **At rest contact the constitutive model does not matter yet.** Yeoh and
//!   Neo-Hookean agree to four significant figures (7.0 µm vs 7.0 µm,
//!   0.038321 N vs 0.038307 N) because 7 µm on a 12 mm plate is 0.06 % strain.
//!   So "it converges" is, on its own, not yet a statement about Yeoh.
//! - **Under a marched compression ramp it walls at `ArmijoStall(iter 0)`** —
//!   at 0.0995 mm of deflection with 0.1 mm increments, 0.1455 mm with
//!   0.025 mm increments. Refining the step buys 46 % and does not remove the
//!   wall.
//! - **`Ok` plus a tiny residual is not a contact solve.** The Tet4 per-vertex
//!   arm at the same `(κ, d̂)` returns `Ok` at residual 9.3e-13 while resting
//!   3.52 mm *inside* the plane, because the barrier clamps at
//!   `d̂ · 1e-6`. Only `min_sd > 0` distinguishes the two, and every gate here
//!   asserts it.
//!
//! ## What this cannot see
//!
//! - **Curvature.** A flat face on a flat plane, so `Sdf::hessian` is the
//!   default zero matrix and the curvature term rung 8c added is never
//!   exercised. `insertion_sim` presses a curved intruder into a curved cavity.
//! - **Graded materials.** One anchor everywhere. `insertion_sim` carries a
//!   layered per-tet Yeoh field, and the material-validity wall row 23 hit was
//!   a per-tet event at one tet.
//! - **Large strain.** The ramp stops at ~1 % engineering compression, which is
//!   below where Yeoh's `C₂` term separates from Neo-Hookean. Whether
//!   Tet10 × Yeoh converges in the regime that motivates Yeoh is still open.
//! - **Friction.** `SolverConfig::friction_mu` defaults to `0.0` and this
//!   fixture leaves it there — the same frictionless regime `insertion_sim`
//!   already runs in, and the regime rung 8b's face path requires.
//! - **Tuned barrier parameters.** `κ` and `d̂` are carried over from the
//!   rung-8b tests, scaled only for plate thickness. The recon doc is explicit
//!   that they do not transfer, and nothing here has re-derived them.
//!
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use nalgebra::Point3;
use sim_ml_chassis::Tensor;
use sim_soft::element::{Tet4, Tet10};
use sim_soft::material::silicone_table::ECOFLEX_00_30;
use sim_soft::{
    Aabb3, ActivePairsFor, BoundaryConditions, ConstantField, ContactPair, CpuNewtonSolver,
    IpcRigidContact, MaterialField, Mesh, MeshingHints, NeoHookean, RigidPlane, Sdf,
    SdfMeshedTetMesh, Solver, SolverConfig, SolverFailure, Tet10Mesh, Vec3, VertexId, Yeoh,
    peak_contact_pressure,
};

// ── fixture geometry ────────────────────────────────────────────────

/// Plate half-extents (m): 40 × 40 × 12 mm. The 12 mm thickness is the
/// sleeve-wall scale the renovation targets (NH walls at ~7 mm, the sleeve
/// needs 8 mm), not the 100 mm block the rung-8b tests use.
const HALF: [f64; 3] = [0.020, 0.020, 0.006];

/// BCC-stuffing cell size (m). Coarse on purpose — this fixture is a
/// convergence probe, not a resolution study.
const CELL: f64 = 0.004;

/// Barrier band (m). Scaled to the plate: the rung-8b tests use `d̂ = 0.01` on a
/// 100 mm block, i.e. `d̂ / thickness = 0.1`; 1.2 mm on a 12 mm plate is the
/// same ratio.
const D_HAT: f64 = 0.0012;

/// Barrier stiffness. The rung-8b face-contact tests' value, carried over
/// unchanged — the recon doc is explicit that penalty's tuned `κ` does *not*
/// transfer to IPC, and this fixture is where a Yeoh-appropriate value gets
/// measured rather than assumed.
const KAPPA: f64 = 1.0e4;

/// Plane standoff inside the band, so the bottom face is engaged at rest.
const STANDOFF: f64 = 0.4 * D_HAT;

/// Large `dt` ⇒ quasi-static: inertia negligible, contact and elasticity
/// balance. Matches the rung-8b tests' `STATIC_DT`.
const STATIC_DT: f64 = 1.0e3;

const MAX_NEWTON_ITER: usize = 60;

/// Axis-aligned box, exact signed distance (`q = |p − c| − h`), analytic
/// gradient.
///
/// Written here because the repo ships no box primitive — `cf_geometry`
/// exports a sphere, a translation wrapper and a difference. `grad` is checked
/// against central differences by [`box_sdf_gradient_matches_finite_differences`]
/// so the fixture's own geometry is verified rather than trusted.
struct BoxSdf {
    center: Vec3,
    half: Vec3,
}

impl BoxSdf {
    /// `q_i = |p_i − c_i| − h_i`: per-axis signed distance to the slab pair.
    /// Positive outside that slab, negative inside it.
    fn q(&self, p: Point3<f64>) -> Vec3 {
        Vec3::new(
            (p.x - self.center.x).abs() - self.half.x,
            (p.y - self.center.y).abs() - self.half.y,
            (p.z - self.center.z).abs() - self.half.z,
        )
    }

    /// Componentwise sign of `p − c`, with `+1` at exactly zero. Folds the
    /// first-octant gradient back onto the octant `p` is actually in.
    fn fold_sign(&self, p: Point3<f64>) -> Vec3 {
        let s = |d: f64| if d < 0.0 { -1.0 } else { 1.0 };
        Vec3::new(
            s(p.x - self.center.x),
            s(p.y - self.center.y),
            s(p.z - self.center.z),
        )
    }
}

impl Sdf for BoxSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let q = self.q(p);
        // Outside: Euclidean distance to the box, counting only the axes the
        // point overshoots. Inside: every `q_i < 0`, and the nearest face is
        // the least-negative one.
        let outside = Vec3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
        let inside = q.x.max(q.y).max(q.z).min(0.0);
        outside + inside
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        let q = self.q(p);
        let sign = self.fold_sign(p);
        let outside = Vec3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0));
        let n = if outside.norm() > 0.0 {
            // Outside (or on an edge/corner): the direction away from the
            // nearest point on the box, which is the overshoot vector.
            outside.normalize()
        } else {
            // Strictly inside: the gradient points at the nearest face, i.e.
            // along the single axis whose `q` is largest.
            let mut axis = 0;
            if q.y > q[axis] {
                axis = 1;
            }
            if q.z > q[axis] {
                axis = 2;
            }
            let mut e = Vec3::zeros();
            e[axis] = 1.0;
            e
        };
        Vec3::new(sign.x * n.x, sign.y * n.y, sign.z * n.z)
    }
}

/// The plate, sitting with its bottom face on `z = 0`.
const fn plate() -> BoxSdf {
    BoxSdf {
        center: Vec3::new(0.0, 0.0, HALF[2]),
        half: Vec3::new(HALF[0], HALF[1], HALF[2]),
    }
}

/// Ground plane (normal `+z`) whose surface sits at height `plane_h`.
/// `RigidPlane`'s offset *is* the plane height, since `sd(p) = p·n̂ − offset`.
///
/// `plane_h < 0` holds the plate in the barrier band without compressing it;
/// `plane_h > 0` drives the plane up through where the plate's bottom face
/// rests, and with the top face pinned that is a compression of `plane_h` on a
/// `2 · HALF[2]` plate.
fn ground_at(plane_h: f64) -> RigidPlane {
    RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), plane_h)
}

/// The resting plane: `STANDOFF` below the plate's bottom face, so the bottom
/// is inside the barrier band but the plate is essentially unloaded.
const REST_PLANE_H: f64 = -STANDOFF;

/// Uniform Ecoflex 00-30, through the five-field bounded constructor — the
/// same `from_yeoh_fields_with_bounds` path `insertion_sim` builds its layered
/// field with, so the per-tet validity caps reach the solver gate here too.
fn yeoh_field() -> MaterialField {
    let m = ECOFLEX_00_30;
    MaterialField::from_yeoh_fields_with_bounds(
        Box::new(ConstantField::new(m.mu)),
        Box::new(ConstantField::new(m.c2)),
        Box::new(ConstantField::new(m.lambda)),
        Box::new(ConstantField::new(m.validity_max_principal_stretch)),
        Box::new(ConstantField::new(m.validity_min_principal_stretch)),
    )
}

/// Uniform Ecoflex 00-30 as Neo-Hookean — the Tet10 × NH baseline's material,
/// built from the same anchor so the two differ in constitutive model alone.
fn nh_field() -> MaterialField {
    let m = ECOFLEX_00_30;
    MaterialField::uniform(m.mu, m.lambda)
}

/// Tet4 plate carrying per-tet Yeoh materials.
fn tet4_yeoh() -> SdfMeshedTetMesh<Yeoh> {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-HALF[0] - CELL, -HALF[1] - CELL, -CELL),
            Vec3::new(HALF[0] + CELL, HALF[1] + CELL, 2.0 * HALF[2] + CELL),
        ),
        cell_size: CELL,
        material_field: Some(yeoh_field()),
    };
    SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&plate(), &hints).expect("mesh the Yeoh plate")
}

/// Tet4 plate carrying per-tet Neo-Hookean materials, same geometry.
fn tet4_nh() -> SdfMeshedTetMesh<NeoHookean> {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-HALF[0] - CELL, -HALF[1] - CELL, -CELL),
            Vec3::new(HALF[0] + CELL, HALF[1] + CELL, 2.0 * HALF[2] + CELL),
        ),
        cell_size: CELL,
        material_field: Some(nh_field()),
    };
    SdfMeshedTetMesh::from_sdf(&plate(), &hints).expect("mesh the NH plate")
}

/// Top-face vertex ids of `mesh` — the pinned set. Picked by position so it
/// works on both the Tet4 corner set and the Tet10 corner-plus-midside set.
fn top_face_pins<M: sim_soft::Material>(mesh: &dyn Mesh<M>) -> Vec<VertexId> {
    let top = 2.0 * HALF[2];
    mesh.positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| (p.z - top).abs() < 0.25 * CELL)
        .map(|(v, _)| v as VertexId)
        .collect()
}

/// Rest DOF vector, flattened `[x, y, z, ...]`.
fn rest_dofs<M: sim_soft::Material>(mesh: &dyn Mesh<M>) -> Vec<f64> {
    let mut rest = Vec::with_capacity(3 * mesh.n_vertices());
    for p in mesh.positions() {
        rest.extend_from_slice(&[p.x, p.y, p.z]);
    }
    rest
}

/// `skeleton()` already carries `gravity_z = 0.0`, `friction_mu = 0.0` and
/// `fbar = false`, which is exactly the regime this fixture wants: no body
/// load, so the barrier pressing the bottom face against the pinned top is the
/// only thing driving deformation; frictionless, as rung 8b's face path
/// requires; and no locking cure, matching `insertion_sim` (where `fbar`
/// appears zero times).
const fn config() -> SolverConfig {
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg
}

// ── the fixture's own geometry, verified ────────────────────────────

#[test]
fn box_sdf_gradient_matches_finite_differences() {
    let b = plate();
    let h = 1.0e-7;
    // Interior, each face's outside, an edge and a corner — the branches
    // `grad` distinguishes.
    //
    // ⚠ Every interior probe is deliberately OFF the medial axis. At the
    // plate's centre `(0, 0, 0.006)` the two z-faces are equidistant, the
    // signed distance has a kink, and the gradient does not exist: the central
    // difference reads exactly 0 while `grad` picks a side. That is a property
    // of the distance function, not a defect in this impl, and it is why the
    // interior probe sits at `z = 0.002` — nearest face unambiguous.
    let probes = [
        Vec3::new(0.0, 0.0, 0.002),       // interior, below the medial plane
        Vec3::new(0.0, 0.0, 0.010),       // interior, above it
        Vec3::new(0.0, 0.0, -0.002),      // below, face
        Vec3::new(0.030, 0.0, HALF[2]),   // +x, face
        Vec3::new(0.030, 0.030, HALF[2]), // +x+y, edge
        Vec3::new(0.030, 0.030, 2.0 * HALF[2] + 0.010), // corner
    ];
    for p in probes {
        let pt = Point3::from(p);
        let analytic = b.grad(pt);
        let mut fd = Vec3::zeros();
        for i in 0..3 {
            let mut lo = p;
            let mut hi = p;
            lo[i] -= h;
            hi[i] += h;
            fd[i] = (b.eval(Point3::from(hi)) - b.eval(Point3::from(lo))) / (2.0 * h);
        }
        assert!(
            (analytic - fd).norm() < 1.0e-5,
            "box SDF gradient at {p:?}: analytic {analytic:?} vs central-difference {fd:?}",
        );
    }
}

// ── the selector, not the types ─────────────────────────────────────

/// The recon doc's §9 correction, applied as a gate.
///
/// A compile probe for `SdfMeshedTetMesh<Yeoh>` in the solver went green for
/// the wrong reason: the type checks, but `boundary_faces6()` returns `None`
/// and contact silently takes the per-vertex path. `IpcRigidContact::active_pairs`
/// selects on that `Option` and nothing else — so the only way to know
/// `Tet10Mesh<Yeoh>` reaches the face barrier is to ask the selector. The Tet4
/// arm is the negative control that keeps this from passing vacuously.
#[test]
fn tet10_yeoh_takes_the_face_path_and_tet4_yeoh_takes_the_vertex_path() {
    let contact = IpcRigidContact::with_params(vec![ground_at(REST_PLANE_H)], KAPPA, D_HAT);

    let tet4 = tet4_yeoh();
    assert!(
        Mesh::<Yeoh>::boundary_faces6(&tet4).is_none(),
        "a linear mesh must not surface six-node faces",
    );
    let pos4 = tet4.positions().to_vec();
    let pairs4 = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet4, &pos4);
    assert!(!pairs4.is_empty(), "Tet4 × Yeoh must engage the ground");
    assert!(
        pairs4
            .iter()
            .all(|p| matches!(p, ContactPair::Vertex { .. })),
        "a linear mesh must emit only Vertex pairs",
    );

    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    assert!(
        Mesh::<Yeoh>::boundary_faces6(&tet10).is_some(),
        "a quadratic mesh must surface six-node faces",
    );
    let pos10 = tet10.positions().to_vec();
    let pairs10 = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet10, &pos10);
    assert!(!pairs10.is_empty(), "Tet10 × Yeoh must engage the ground");
    assert!(
        pairs10
            .iter()
            .all(|p| matches!(p, ContactPair::Face { .. })),
        "a quadratic mesh must emit only Face pairs — if this reads Vertex, the \
         face barrier is not being exercised and every convergence number below \
         describes the wrong code path",
    );
}

// ── the measurement ─────────────────────────────────────────────────

/// What one quasi-static press did — the convergence numbers **and** the
/// loading witness that says whether they describe a real load case.
///
/// A two-iteration solve at residual 1e-13 is a fine result and is also
/// precisely what a no-op looks like: a plate that never moved satisfies
/// equilibrium immediately. The last three fields are what separate the two.
/// Without them "Tet10 × Yeoh converges" would be a claim about an unloaded
/// plate wearing the words of a claim about contact.
#[derive(Debug)]
struct Press {
    iters: usize,
    residual: f64,
    /// Largest nodal displacement from rest (m).
    max_disp: f64,
    /// Net contact force on the solid along `+z` (N) at the converged pose.
    /// The plane is below with normal `+z`, so a loaded step is positive.
    net_force_z: f64,
    /// Peak contact pressure (Pa) over `n_pairs` active pairs.
    peak_pressure: f64,
    n_pairs: usize,
    /// Smallest signed distance over the active pairs at the converged pose (m).
    ///
    /// The interior-point guarantee is `sd > 0`, but `IpcRigidContact::barrier`
    /// clamps its argument (`d = sd.max(d_hat * 1e-6)`), so a solve CAN come to
    /// rest with nodes at or below the plane and still report a tiny residual —
    /// the clamp makes the energy finite where the true barrier is infinite.
    /// This is the field that tells a physically-standing-off solve apart from
    /// one resting on the clamp.
    min_sd: f64,
}

/// Displacement and contact readout at the converged pose.
fn summarize<M: sim_soft::Material>(
    mesh: &dyn Mesh<M>,
    plane_h: f64,
    rest: &[f64],
    x_final: &[f64],
    iters: usize,
    residual: f64,
) -> Press {
    let max_disp = rest
        .chunks_exact(3)
        .zip(x_final.chunks_exact(3))
        .map(|(r, x)| Vec3::new(x[0] - r[0], x[1] - r[1], x[2] - r[2]).norm())
        .fold(0.0_f64, f64::max);
    let positions: Vec<Vec3> = x_final
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    let readouts = IpcRigidContact::with_params(vec![ground_at(plane_h)], KAPPA, D_HAT)
        .per_pair_readout(mesh, &positions);
    Press {
        iters,
        residual,
        max_disp,
        net_force_z: readouts.iter().map(|r| r.force_on_soft.z).sum(),
        peak_pressure: peak_contact_pressure(&readouts),
        n_pairs: readouts.len(),
        min_sd: readouts.iter().map(|r| r.sd).fold(f64::INFINITY, f64::min),
    }
}

/// One-line label for a fail-close surface.
///
/// `SolverFailure`'s derived `Debug` embeds `x_partial`, which on this fixture
/// is 24 993 floats — 2.3 MB of log per failed rung, which buries the one thing
/// the rung is being run to learn. This keeps the variant and its diagnostic
/// scalars and drops the state vector.
fn failure_label(e: &SolverFailure) -> String {
    match e {
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => format!("ArmijoStall(iter {last_iter}, r {last_r_norm:.3e})"),
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => format!("NewtonIterCap(max {max_iter}, r {last_r_norm:.3e})"),
        SolverFailure::DoublyFailedFactor {
            last_iter, context, ..
        } => format!("DoublyFailedFactor(iter {last_iter}, {context})"),
        SolverFailure::ValidityViolation {
            tet_id, message, ..
        } => format!("ValidityViolation(tet {tet_id}: {message})"),
    }
}

/// Zero-velocity, zero-θ arguments shared by all three presses.
fn step_inputs(rest: &[f64]) -> (Tensor<f64>, Tensor<f64>, Tensor<f64>) {
    let n_dof = rest.len();
    (
        Tensor::from_slice(rest, &[n_dof]),
        Tensor::from_slice(&vec![0.0; n_dof], &[n_dof]),
        Tensor::from_slice(&[], &[0]),
    )
}

/// Tet10 × Yeoh — the configuration item 3 needs and nothing has ever solved.
///
/// Uses `try_replay_step`, not `replay_step`: the four fail-close surfaces
/// (`ArmijoStall`, `NewtonIterCap`, `DoublyFailedFactor`, `ValidityViolation`)
/// come back as an `Err` variant naming which one fired, where the panicking
/// path would only abort. Which surface fires is the finding.
fn press_tet10_yeoh(plane_h: f64) -> Result<Press, String> {
    let tet4 = tet4_yeoh();
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    assert!(!pins.is_empty(), "the pinned top face must not be empty");
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
        CpuNewtonSolver::new(
            Tet10,
            mesh.clone(),
            IpcRigidContact::with_params(vec![ground_at(plane_h)], KAPPA, D_HAT),
            config(),
            BoundaryConditions::new(pins, Vec::new()),
        );
    solver
        .try_replay_step(&x, &v, &theta, STATIC_DT)
        .map(|s| {
            summarize(
                &mesh,
                plane_h,
                &rest,
                &s.x_final,
                s.iter_count,
                s.final_residual_norm,
            )
        })
        .map_err(|e| failure_label(&e))
}

/// Tet4 × Yeoh — the per-vertex barrier on the same plate, same `(κ, d̂)`.
///
/// ⛔ **Not a valid baseline at these parameters, and kept because that is the
/// finding.** It reports `Ok`, 10 Newton iterations and residual 9.3e-13 —
/// and `min_sd = −3.52 mm`, i.e. the converged pose is three and a half
/// millimetres *through* the plane, with a net "contact force" of 4.06e9 N
/// where the face path reads 0.038 N. `IpcRigidContact::barrier` clamps
/// `d = sd.max(d_hat * 1e-6)`, which is what makes a penetrating state finite
/// enough to converge onto.
///
/// What is established: at one shared `(κ, d̂)` the per-vertex and per-face
/// barriers are not in the same regime, by eleven orders of magnitude in
/// force, and only the face path holds standoff. What is NOT established is
/// why — whether `κ = 1e4` is simply far too soft for 607 vertex pairs on this
/// plate, or something else. That has not been isolated, and item 3 has to
/// re-derive `(κ, d̂)` for the face path regardless.
fn press_tet4_yeoh(plane_h: f64) -> Result<Press, String> {
    let mesh = tet4_yeoh();
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet4, SdfMeshedTetMesh<Yeoh>, IpcRigidContact, Yeoh> =
        CpuNewtonSolver::new(
            Tet4,
            mesh.clone(),
            IpcRigidContact::with_params(vec![ground_at(plane_h)], KAPPA, D_HAT),
            config(),
            BoundaryConditions::new(pins, Vec::new()),
        );
    solver
        .try_replay_step(&x, &v, &theta, STATIC_DT)
        .map(|s| {
            summarize(
                &mesh,
                plane_h,
                &rest,
                &s.x_final,
                s.iter_count,
                s.final_residual_norm,
            )
        })
        .map_err(|e| failure_label(&e))
}

/// Tet10 × Neo-Hookean — the element baseline, and the configuration
/// `tet10_indentation_demand1` already exercises. Same plate, same contact.
fn press_tet10_nh(plane_h: f64) -> Result<Press, String> {
    let tet4 = tet4_nh();
    let mesh = Tet10Mesh::<NeoHookean>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet10, Tet10Mesh<NeoHookean>, IpcRigidContact, NeoHookean, 10, 4> =
        CpuNewtonSolver::new(
            Tet10,
            mesh.clone(),
            IpcRigidContact::with_params(vec![ground_at(plane_h)], KAPPA, D_HAT),
            config(),
            BoundaryConditions::new(pins, Vec::new()),
        );
    solver
        .try_replay_step(&x, &v, &theta, STATIC_DT)
        .map(|s| {
            summarize(
                &mesh,
                plane_h,
                &rest,
                &s.x_final,
                s.iter_count,
                s.final_residual_norm,
            )
        })
        .map_err(|e| failure_label(&e))
}

#[test]
fn tet10_yeoh_converges_under_the_ipc_face_barrier() {
    let tet4 = tet4_yeoh();
    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    eprintln!(
        "fixture: {} tets, Tet4 {} nodes -> Tet10 {} nodes ({} DOF)",
        tet4.n_tets(),
        tet4.n_vertices(),
        tet10.n_vertices(),
        3 * tet10.n_vertices(),
    );
    eprintln!(
        "  Tet4  x Yeoh        : {:?}",
        press_tet4_yeoh(REST_PLANE_H)
    );
    eprintln!("  Tet10 x NeoHookean  : {:?}", press_tet10_nh(REST_PLANE_H));

    let p = press_tet10_yeoh(REST_PLANE_H)
        .expect("Tet10 x Yeoh must converge under the IPC face barrier");
    eprintln!("  Tet10 x Yeoh        : {p:?}");

    assert!(
        p.residual.is_finite(),
        "converged step reported a non-finite residual {:e}",
        p.residual,
    );
    assert!(
        p.iters <= MAX_NEWTON_ITER,
        "converged in {} iterations, past the {MAX_NEWTON_ITER} cap",
        p.iters,
    );

    // The loading witness. Without these three, the iteration count above is
    // compatible with a plate that never touched the plane.
    assert!(
        p.n_pairs > 0,
        "converged with no active contact pairs — nothing was solved",
    );
    assert!(
        p.net_force_z > 0.0 && p.net_force_z.is_finite(),
        "the barrier must push the plate up; net force along +z was {:e} N",
        p.net_force_z,
    );
    assert!(
        p.max_disp > 0.0,
        "the plate did not move: max displacement {:e} m",
        p.max_disp,
    );
    assert!(
        p.peak_pressure > 0.0 && p.peak_pressure.is_finite(),
        "peak contact pressure must be positive and finite; got {:e} Pa",
        p.peak_pressure,
    );
    // The assertion that actually certifies a CONTACT solve. Everything above
    // is satisfied by the Tet4 arm printed alongside, which "converges" to
    // residual 9.3e-13 while 3.5 mm INSIDE the plane.
    assert!(
        p.min_sd > 0.0,
        "the converged pose penetrates the plane by {:e} m — the barrier clamp \
         made that finite, it does not make it a contact solve",
        -p.min_sd,
    );
}

/// Plane travel per ramp increment (m).
///
/// Must stay below the standing barrier gap: each increment lifts the plane by
/// `RAMP_STEP` while the previous step left the deformed bottom face a gap
/// above it, so `RAMP_STEP < gap` is what keeps the next step's initial guess
/// intersection-free. 0.1 mm against `d̂ = 1.2` mm.
const RAMP_STEP: f64 = 0.0001;

/// Highest the plane is driven (m) — 3 mm into a 12 mm plate, 25 % engineering
/// compression, well past where Yeoh and Neo-Hookean part company.
const RAMP_MAX_PLANE_H: f64 = 0.0030;

/// March the plane up through the plate, re-solving from each converged state,
/// and stop at the first rung that fails.
///
/// Returns one entry per attempted plane height. The mesh, pins and rest
/// configuration are built once; only the contact primitive changes per rung,
/// which is the same per-increment rebuild `tet10_indentation_demand1` uses.
fn ramp_tet10_yeoh(step: f64) -> Vec<(f64, Result<Press, String>)> {
    let tet4 = tet4_yeoh();
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let n_dof = rest.len();
    let zero_v = Tensor::from_slice(&vec![0.0; n_dof], &[n_dof]);
    let theta = Tensor::from_slice(&[], &[0]);

    let mut x_prev = rest.clone();
    let mut out = Vec::new();
    // Indexed, not accumulated: `h += step` drifts (the first cut of this loop
    // printed a +0.020000000000000025 mm plane height) and a float loop
    // condition is a lint besides.
    let n_steps = ((RAMP_MAX_PLANE_H - REST_PLANE_H) / step).ceil() as usize;
    for i in 0..=n_steps {
        let h = REST_PLANE_H + (i as f64) * step;
        let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
            CpuNewtonSolver::new(
                Tet10,
                mesh.clone(),
                IpcRigidContact::with_params(vec![ground_at(h)], KAPPA, D_HAT),
                config(),
                BoundaryConditions::new(pins.clone(), Vec::new()),
            );
        match solver.try_replay_step(
            &Tensor::from_slice(&x_prev, &[n_dof]),
            &zero_v,
            &theta,
            STATIC_DT,
        ) {
            Ok(step) => {
                let p = summarize(
                    &mesh,
                    h,
                    &rest,
                    &step.x_final,
                    step.iter_count,
                    step.final_residual_norm,
                );
                x_prev = step.x_final;
                out.push((h, Ok(p)));
            }
            Err(e) => {
                out.push((h, Err(failure_label(&e))));
                break;
            }
        }
    }
    out
}

/// Where does Tet10 × Yeoh stop converging?
///
/// The resting press above answers "does it solve", and the answer is yes — at
/// 7 µm of deflection, 0.06 % strain. At that amplitude Yeoh and Neo-Hookean
/// agree to four significant figures, so the resting press does not exercise
/// the constitutive model that is the whole reason item 1 made `Tet10Mesh`
/// generic. This walks the plane up through the plate under a pinned top face,
/// which is a direct compression, and reports every rung.
///
/// ⚠ **The plane must be MARCHED, not placed.** A first cut at this test set
/// the plane straight to each target depth from the rest configuration and got
/// `ArmijoStall(iter 0)` at every depth from 1.67 % to 33 % strain — *and
/// identical residuals for Yeoh and Neo-Hookean*, which is the tell: a failure
/// that does not move when the constitutive model changes is not a
/// constitutive failure. The IPC barrier is undefined at `d ≤ 0`, so a plane
/// placed inside the material starts the very first line search from an
/// infeasible point. Every increment here therefore re-solves from the
/// previous converged state, the same way `tet10_indentation_demand1` and
/// `insertion_sim` march theirs.
///
/// ⚠ This is a **probe, not a gate on a number**. It asserts that the ramp
/// gets off the ground and that each converged rung is loaded; the depth at
/// which it stops is printed, not asserted. Pinning that depth would pin a
/// number that `κ`, `d̂`, mesh resolution and element order all move — and
/// moving exactly those is item 3's job.
#[test]
fn tet10_yeoh_convergence_envelope_under_increasing_compression() {
    let thickness = 2.0 * HALF[2];
    eprintln!(
        "compression ramp, {:.0} mm plate, kappa {KAPPA:e}, d_hat {D_HAT:e}, \
         step {:.2} mm",
        thickness * 1e3,
        RAMP_STEP * 1e3,
    );

    let rungs = ramp_tet10_yeoh(RAMP_STEP);
    for (h, r) in &rungs {
        let strain = (h / thickness).max(0.0) * 100.0;
        match r {
            Ok(p) => eprintln!(
                "  plane {:+.2} mm (strain {:5.2} %): iters {:2}, r {:.2e}, \
                 disp {:.4} mm, Fz {:.4e} N, p_peak {:.3e} Pa, pairs {}",
                h * 1e3,
                strain,
                p.iters,
                p.residual,
                p.max_disp * 1e3,
                p.net_force_z,
                p.peak_pressure,
                p.n_pairs,
            ),
            Err(e) => eprintln!("  plane {:+.2} mm (strain {:5.2} %): {e}", h * 1e3, strain),
        }
    }

    let converged: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
    assert!(
        converged.len() >= 2,
        "the ramp must clear at least two rungs before it stops; it cleared {}",
        converged.len(),
    );
    for p in &converged {
        assert!(
            p.n_pairs > 0 && p.net_force_z > 0.0 && p.net_force_z.is_finite(),
            "a converged rung must be loaded: {p:?}",
        );
        assert!(
            p.min_sd > 0.0,
            "a converged rung must not penetrate the plane: {p:?}",
        );
    }
    let deepest = converged.last().expect("checked non-empty above");
    assert!(
        deepest.max_disp > 10.0 * converged[0].max_disp,
        "the ramp must actually load the plate: deepest rung moved {:e} m against \
         the first rung's {:e} m",
        deepest.max_disp,
        converged[0].max_disp,
    );
}

/// Is the envelope's end a step-size artefact, or a wall?
///
/// `#[ignore]` — two full ramps, ~40 s. This is the discriminating experiment
/// for the one question the envelope test cannot answer from a single step
/// size: refining the increment either walks the ramp arbitrarily deep (in
/// which case the end is bookkeeping) or moves it a little and hits the same
/// fail-close surface (in which case something real is there).
///
/// Measured 2026-09-20 at `d12e3bf9` + this fixture, 4 240 tets / 24 993 DOF:
///
/// | step | rungs | deepest plane | max displacement | ends with |
/// |------|-------|---------------|------------------|-----------|
/// | 0.100 mm | 6 | +0.020 mm | 0.0995 mm | `ArmijoStall(iter 0, r 7.773e2)` |
/// | 0.025 mm | 23 | +0.070 mm | 0.1455 mm | `ArmijoStall(iter 0, r 5.747e2)` |
///
/// ⇒ 4× refinement buys 46 % more deflection and does **not** remove the wall.
/// The stall is at Newton iteration 0 both times: the first line search out of
/// the increment's initial guess cannot find a decrease.
///
/// ⚠ Not the same signature as the open `insertion_sim` pathology.
/// `sliding_insertion_ramp_converges_on_synthetic_icosphere` stalls at
/// **iter 23, r 6.143e-1** — deep into Newton at a small residual. This stalls
/// at **iter 0** at a residual three orders larger. Whether they share a cause
/// has not been established, and this fixture is not evidence that they do.
#[test]
#[ignore = "two full compression ramps, ~40 s — run it when (kappa, d_hat), \
           mesh resolution or element order change"]
fn ramp_increment_refinement_extends_the_envelope_without_removing_it() {
    let mut reached = Vec::new();
    for step in [RAMP_STEP, RAMP_STEP / 4.0] {
        let rungs = ramp_tet10_yeoh(step);
        let ok: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
        let err = rungs.iter().find_map(|(_, r)| r.as_ref().err()).cloned();
        let deepest_plane = rungs.iter().rev().find(|(_, r)| r.is_ok()).map(|(h, _)| *h);
        let max_disp = ok.last().map_or(0.0, |p| p.max_disp);
        eprintln!(
            "  step {:.3} mm: {} rungs, deepest plane {:+.3} mm, max disp {:.4} mm, ends {}",
            step * 1e3,
            ok.len(),
            deepest_plane.unwrap_or(f64::NAN) * 1e3,
            max_disp * 1e3,
            err.clone()
                .unwrap_or_else(|| "(ran to RAMP_MAX_PLANE_H)".into()),
        );
        reached.push((max_disp, err));
    }

    let (coarse_disp, coarse_err) = &reached[0];
    let (fine_disp, fine_err) = &reached[1];
    assert!(
        fine_disp > coarse_disp,
        "refining the increment must reach deeper: fine {fine_disp:e} m vs coarse {coarse_disp:e} m",
    );
    // The wall is the finding. If refinement ever runs a ramp clean to
    // `RAMP_MAX_PLANE_H`, this assertion is what says so out loud rather than
    // letting the table above quietly go stale.
    assert!(
        coarse_err.is_some() && fine_err.is_some(),
        "both ramps were expected to stop early; coarse {coarse_err:?}, fine {fine_err:?}",
    );
}
