//! R1.0 — does a linear subspace represent this material's deformation?
//!
//! The kill-or-confirm rung of `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md`. No reduced
//! solver exists yet, and deliberately so: if a POD basis cannot represent the
//! deformation, the whole reduced-order ladder is wrong for this material class and no
//! solver should be written.
//!
//! ## The fixture, and why the load is a traction rather than a prescribed indenter
//!
//! A bonded elastic layer (bottom face pinned) under a **Gaussian traction patch** on
//! its top face, ramped up over a few steps. The patch's centre, width and peak vary
//! across trajectories — this is a stand-in for a device pressing on tissue, which is
//! the deformation class the mission actually cares about.
//!
//! The obvious alternative — prescribing a *displacement* on an indenter patch — does
//! not work for a basis study, and the reason is worth recording: moving the patch
//! changes which DOFs are Dirichlet-constrained, which changes `free_dof_indices`,
//! which changes the snapshot width. Trajectories with the indenter in different places
//! could not share a basis at all. Fixing the patch instead would leave depth as the
//! only ensemble parameter, and §2 below shows exactly how badly that misleads. Loading
//! through `theta` keeps every loaded vertex FREE, so the free-DOF map is identical
//! across the whole ensemble and one solver serves every trajectory.
//!
//! ## What is asserted, and what is only reported
//!
//! **Asserted**: held-out *interpolation* error below 1 %, at an `r` inside the plan's
//! ceiling; and `ΦᵀMΦ = I`.
//!
//! **Reported, not asserted**: held-out *extrapolation* error. It is large by design
//! (35–60 %) and that is the point — it is the first evidence about how the validity
//! domain behaves at its edge, which R3 has to formalise. Asserting a number on it now
//! would pin an arbitrary bound to a quantity nobody has designed yet.
//!
//! ## Two findings from the pilot that shaped this gate
//!
//! 1. **A one-parameter ensemble gives a false confirm.** Varying only the load
//!    magnitude yields `r = 2` at 100.0000 % retained energy — a basis that looks
//!    perfect and means nothing, because a load ramp produces nearly proportional
//!    displacement fields. Effective rank tracks the number of *trajectories*, not the
//!    number of snapshots. Hence 48 trajectories of 5 steps rather than 5 of 48.
//! 2. **Training energy does not predict generalisation.** The 99.99 %-energy criterion
//!    selects `r = 6`, at which held-out error is 2.7 %. Reaching 1 % needs `r = 40`
//!    *and* a dense ensemble; either alone plateaus. `r` is therefore pinned here
//!    rather than chosen by an energy threshold.

#![allow(
    clippy::expect_used,
    clippy::too_many_lines,
    // The LCG and the ramp fraction convert small integer counters to f64; every value
    // is far below 2^53, so the documented mantissa concern cannot arise here.
    clippy::cast_precision_loss,
    // `a`/`b`/`c`/`d` are the four unit-interval draws feeding the four trajectory
    // parameters, and `w`/`s` mirror the maths they implement; longer names would
    // obscure the correspondence rather than clarify it.
    clippy::many_single_char_names
)]

use sim_ml_chassis::Tensor;
use sim_soft::solver::backward_euler::reduced::{Inner, PodBasis, SnapshotSet};
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

const LX: f64 = 0.020;
const LY: f64 = 0.020;
const H: f64 = 0.006;
const MU: f64 = 1.0e5;
/// `nu = 0.4` — the Tet4-safe ratio the crate's other Tet4 gates use (volumetric
/// locking precludes `nu -> 0.5` without F-bar).
const LAMBDA: f64 = 4.0 * MU;
const DENSITY: f64 = 1030.0;
const DT: f64 = 1.0 / 60.0;

/// Training trajectories. Pilot-measured: 16 gives 2.1 % held-out, 32 gives 1.2 %,
/// 48 gives 0.79 %. Below ~24 the ensemble is too sparse to span the parameter box and
/// the gate would fail for want of training data rather than for want of a subspace.
const N_TRAIN: usize = 48;
/// Steps per trajectory. Raising this does **not** help — effective rank tracks
/// trajectory count (10 steps measured 0.81 %, 5 steps 0.79 %) — so it is kept low.
const STEPS: usize = 5;
/// Retained modes. Inside the plan's ceiling of `min(n_free / 50, 200)`, which is 104
/// for this fixture's 5 202 free DOF; `r = 40` is a 130x reduction.
const R_MODES: usize = 40;
/// Held-out interpolation gate.
const MAX_HELDOUT_ERR: f64 = 1.0e-2;

struct Rig {
    solver: CpuTet4NHSolver<HandBuiltTetMesh>,
    x_rest: Vec<f64>,
    loaded: Vec<VertexId>,
    n_dof: usize,
}

/// One loading trajectory: a Gaussian traction patch at `(cx, cy)` of width `w` and
/// peak `p0`, ramped linearly to full magnitude.
#[derive(Clone, Copy, Debug)]
struct Traj {
    cx: f64,
    cy: f64,
    w: f64,
    p0: f64,
}

fn rig() -> Rig {
    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(16, 16, 6, LX, LY, H, &field);
    let n_dof = 3 * mesh.n_vertices();
    let mut x_rest = vec![0.0; n_dof];
    for (c, p) in x_rest.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-12);
    let loaded: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p: &Vec3| (p.z - H).abs() < 1e-12);
    let bc = BoundaryConditions {
        pinned_vertices: pins,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.max_newton_iter = 80;
    cfg.tol = 1.0e-6;
    Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    }
}

/// Deterministic sample of the parameter box — an LCG, so the ensemble is reproducible
/// without a dependency and without a golden file.
fn sample(k: u64) -> Traj {
    let mut s = k
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    let mut next = || {
        s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        ((s >> 33) as f64) / ((1u64 << 31) as f64)
    };
    let (a, b, c, d) = (next(), next(), next(), next());
    Traj {
        cx: 0.007 + 0.006 * a,
        cy: 0.007 + 0.006 * b,
        w: 0.0025 + 0.0015 * c,
        p0: 0.10 + 0.20 * d,
    }
}

fn run(r: &Rig, t: Traj, steps: usize) -> Vec<Vec<f64>> {
    let fd = r.solver.free_dof_indices().to_vec();
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    let mut out = Vec::new();
    for s in 1..=steps {
        let frac = s as f64 / steps as f64;
        let mut th = vec![0.0; 3 * r.loaded.len()];
        for (i, &vid) in r.loaded.iter().enumerate() {
            let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
            let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
            th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
        }
        let step = r.solver.replay_step(
            &Tensor::from_slice(&x, &[r.n_dof]),
            &Tensor::from_slice(&v, &[r.n_dof]),
            &Tensor::from_slice(&th, &[th.len()]),
            DT,
        );
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
        out.push(SnapshotSet::free_displacement(&x, &r.x_rest, &fd));
    }
    out
}

// Release-only: 52 trajectories x 5 dynamic Newton solves at ~5.2k free DOF. Mirrors
// the `contact_drop_rest` / `hertz_sphere_plane` heavy-gate pattern.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 52 trajectories x 5 dynamic solves at ~5.2k free DOF; \
              rerun with `cargo test --release` to include"
)]
#[test]
fn linear_subspace_represents_indentation_within_one_percent() {
    let r = rig();
    let n_free = r.solver.free_dof_indices().len();
    let mass = r.solver.mass_per_free_dof();

    let ceiling = (n_free / 50).min(200);
    assert!(
        R_MODES <= ceiling,
        "r = {R_MODES} exceeds the plan's ceiling of min(n_free/50, 200) = {ceiling}. \
         Raising r until the error looks acceptable is the failure mode that ceiling \
         exists to prevent."
    );

    let mut train = SnapshotSet::new(n_free);
    for k in 0..N_TRAIN as u64 {
        for u in &run(&r, sample(k), STEPS) {
            train.push(u);
        }
    }

    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, R_MODES).expect("basis fits");
    assert_eq!(
        basis.n_modes(),
        R_MODES,
        "expected a full-rank {R_MODES}-mode basis"
    );

    // ΦᵀMΦ = I. This is the invariant that caught the pilot's mass-weighting bug, where
    // modes were re-weighted after fitting and every mass-inner projection collapsed to
    // zero — which the error metric reported as a suspiciously exact 1.0.
    let mut worst_orth = 0.0_f64;
    for i in 0..basis.n_modes() {
        let mut e = vec![0.0; basis.n_modes()];
        e[i] = 1.0;
        for (j, qj) in basis.project(&basis.reconstruct(&e)).iter().enumerate() {
            let target = if i == j { 1.0 } else { 0.0 };
            worst_orth = worst_orth.max((qj - target).abs());
        }
    }
    assert!(
        worst_orth < 1.0e-8,
        "basis is not M-orthonormal: max |ΦᵀMΦ − I| = {worst_orth:.3e}"
    );

    // Held-out INTERPOLATION — inside the training box, never trained on. This is the
    // gate. Measuring on training data instead would be near-vacuous.
    let mut worst = 0.0_f64;
    for k in 900..903_u64 {
        for u in &run(&r, sample(k), STEPS) {
            worst = worst.max(basis.projection_error(u));
        }
    }

    // Held-out EXTRAPOLATION — outside the box. Reported only; see the module docs.
    let outside = Traj {
        cx: 0.0060,
        cy: 0.0140,
        w: 0.0020,
        p0: 0.45,
    };
    let extrap = run(&r, outside, STEPS)
        .iter()
        .map(|u| basis.projection_error(u))
        .fold(0.0_f64, f64::max);

    println!(
        "R1.0: n_free={n_free} r={R_MODES} train_traj={N_TRAIN} snaps={} \
         retained_energy={:.6}",
        train.len(),
        basis.retained_energy_fraction()
    );
    println!("R1.0: held-out INTERP worst = {worst:.3e}   EXTRAP worst = {extrap:.3e} (reported)");

    assert!(
        worst < MAX_HELDOUT_ERR,
        "held-out interpolation projection error {worst:.3e} exceeds the {MAX_HELDOUT_ERR:.0e} \
         gate at r = {R_MODES}. Before raising r, check the ensemble: pilot-measured, this \
         fixture needs ~24+ training trajectories, and below that the gate fails for want \
         of training data rather than for want of a subspace."
    );
}
