//! Does the predictor's iteration-count gain SURVIVE reduction?
//!
//! ## The assumption this exists to test
//!
//! Recon §2f reports that a Newton predictor cuts iteration count ~1.95× on the
//! representative workload, and concludes that R3's required per-iteration gain
//! is roughly halved. That conclusion multiplies two factors:
//!
//! ```text
//!     R1's ~2× PER-ITERATION saving  ×  the predictor's ~2× ITERATION-COUNT saving
//! ```
//!
//! They compose only if the reduced solve inherits the iteration-count saving.
//! **Nobody had measured that.** R1.1 established that oracle and reduced take
//! *identical* iteration counts with no predictor — but "identical without one"
//! does not imply "identically improved by one". If the reduced solve's count
//! does not drop, the second factor is already spent by the first and §2f's
//! halving evaporates.
//!
//! So this is not a regression test for the port. It is the measurement that
//! decides whether a headline already written down is true.
//!
//! ## The matrix
//!
//! | | `PreviousState` | `Inertial` |
//! |---|---|---|
//! | **oracle** (full order) | baseline | oracle gain |
//! | **reduced** (Galerkin, `r = 40`) | R1.1's baseline | reduced gain |
//!
//! Four arms, one shared held-out trajectory, iteration counts summed per arm.
//! The quantity of interest is **`oracle gain ÷ reduced gain`**: at 1.0 the two
//! factors compose cleanly, and §2f's arithmetic stands as written.
//!
//! `InertialWithLoad` is absent by design — recon §2f's pre-registered rule
//! killed it (step-0 failure on contact plus a body load) and the reduced loop
//! refuses it outright.
//!
//! ⚠ **Precondition, checked before anything is concluded.** A gentle fixture
//! that converges in 2 iterations per step cannot show an iteration-count gain
//! whatever the predictor does. The report prints the oracle baseline's mean
//! iterations per step first, and says so explicitly when that number is too
//! low for the comparison to mean anything.
//!
//! ## Running it
//!
//! An instrument, not a gate — `#[ignore]`d, so it costs CI nothing while still
//! being compiled.
//!
//! ```text
//! cargo test --release -p sim-soft --test reduced_predictor -- --ignored --nocapture
//! ```

#![allow(
    // A failed oracle or reduced step is a broken instrument, not a runtime
    // condition to recover from.
    clippy::panic,
    clippy::expect_used,
    clippy::unwrap_used,
    // Step counters and iteration tallies convert small integer counts to f64
    // for means and ratios; every value is far below f64's exact-integer range.
    clippy::cast_precision_loss,
    // The LCG and the ramp fraction mirror `reduced_newton_trajectory`'s rig,
    // whose single-letter names track the maths they implement.
    clippy::many_single_char_names,
    // Each body is one linear narrative: build, train, run the arms, compare.
    clippy::too_many_lines
)]

use sim_ml_chassis::Tensor;
use sim_soft::solver::backward_euler::reduced::{
    Inner, PodBasis, ReducedNewtonSolver, SnapshotSet,
};
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, HandBuiltTetMesh, InitialGuess, LoadAxis, MaterialField,
    Mesh, NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

// ── The R1.1 rig, reproduced so the comparison lands on the fixture R1.1's
//    "identical iteration counts" claim was measured on ────────────────────

const LX: f64 = 0.020;
const LY: f64 = 0.020;
const H: f64 = 0.006;
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0 * MU;
const DENSITY: f64 = 1030.0;
const DT: f64 = 1.0 / 60.0;
/// Training trajectories — R1.1's count, which reaches 0.79 % held-out at `r = 40`.
const N_TRAIN: usize = 48;
/// Steps per trajectory. Held at R1.1's value so the evaluation stays INSIDE the
/// training box; extrapolating outside it costs 12–39 % and would confound the
/// iteration-count comparison with a basis-quality effect.
const STEPS: usize = 5;
/// R1.1's retained-mode count — the default cell's `r`.
const R_MODES: usize = 40;
/// Held-out trajectory ids, disjoint from `0..N_TRAIN`.
const EVAL_IDS: [u64; 3] = [900, 901, 902];
/// Below this mean, the fixture has no room to show a gain and the comparison
/// is reported as inconclusive rather than as a result.
const MIN_INFORMATIVE_ITERS_PER_STEP: f64 = 3.0;

struct Rig {
    solver: CpuTet4NHSolver<HandBuiltTetMesh>,
    x_rest: Vec<f64>,
    loaded: Vec<VertexId>,
    n_dof: usize,
}

fn rig(guess: InitialGuess) -> Rig {
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
    cfg.initial_guess = guess;
    Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    }
}

#[derive(Clone, Copy, Debug)]
struct Traj {
    cx: f64,
    cy: f64,
    w: f64,
    p0: f64,
}

/// Deterministic sample of the parameter box — R1.1's LCG verbatim, so training
/// and held-out ids mean the same thing here as they do there.
fn sample_scaled(k: u64, load_scale: f64) -> Traj {
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
        p0: load_scale * (0.10 + 0.20 * d),
    }
}

/// The θ traction vector at step `s` of trajectory `t` — the one thing all four
/// arms must share exactly, so it is computed in one place.
fn theta_at(r: &Rig, t: Traj, s: usize) -> Vec<f64> {
    let frac = s as f64 / STEPS as f64;
    let mut th = vec![0.0; 3 * r.loaded.len()];
    for (i, &vid) in r.loaded.iter().enumerate() {
        let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
        let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
        th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
    }
    th
}

/// Full-order trajectory. Returns the per-step states and the total iterations.
fn run_oracle(r: &Rig, t: Traj) -> (Vec<Vec<f64>>, Option<usize>) {
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    let mut out = Vec::new();
    let mut iters = 0usize;
    for s in 1..=STEPS {
        let th = theta_at(r, t, s);
        let Ok(step) = r.solver.try_replay_step(
            &Tensor::from_slice(&x, &[r.n_dof]),
            &Tensor::from_slice(&v, &[r.n_dof]),
            &Tensor::from_slice(&th, &[th.len()]),
            DT,
        ) else {
            // A load scale can leave the MATERIAL's validity domain, which is a
            // physics limit on the fixture, not a Newton-difficulty result.
            // Report it rather than crash the sweep.
            return (out, None);
        };
        iters += step.iter_count;
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
        out.push(x.clone());
    }
    (out, Some(iters))
}

/// Reduced trajectory, carried end to end in reduced coordinates.
fn run_reduced(
    r: &Rig,
    reduced: &ReducedNewtonSolver<
        '_,
        Tet4,
        HandBuiltTetMesh,
        NullContact,
        sim_soft::NeoHookean,
        4,
        1,
    >,
    t: Traj,
    n_modes: usize,
) -> Option<usize> {
    let mut q = vec![0.0; n_modes];
    let mut qdot = vec![0.0; n_modes];
    let mut iters = 0usize;
    for s in 1..=STEPS {
        let th = theta_at(r, t, s);
        let Ok(step) = reduced.step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT) else {
            return None;
        };
        iters += step.iter_count;
        q = step.q;
        qdot = step.qdot;
    }
    Some(iters)
}

#[test]
#[ignore = "reduced-predictor instrument — run explicitly, see module docs"]
fn does_the_predictor_gain_survive_reduction() {
    // ⚠ TWO load scales, and the second is the informative one. At ×1 the
    // oracle baseline is exactly 3.00 iterations/step — the floor below which
    // this comparison cannot mean anything, which is far too close to call a
    // result. ×4 traction makes Newton actually work on the same geometry, and
    // the ensemble is trained AND evaluated at the same scale so the basis is
    // never asked to extrapolate outside its box.
    // (load, r). The third cell is the CONFOUND CONTROL: ×2 load again, but with
    // enough modes to bring the subspace's own projection floor back to roughly
    // what it was at ×1. If the composition ratio returns to ~1.0 there, the dip
    // at (×2, r=40) was the basis getting worse, not the predictor helping less.
    for (load_scale, r_modes) in [(1.0_f64, R_MODES), (2.0, R_MODES), (2.0, 2 * R_MODES)] {
        measure(load_scale, r_modes);
    }
}

fn measure(load_scale: f64, r_modes: usize) {
    // Two full solvers: the guess lives on the config, and `ReducedNewtonSolver`
    // BORROWS its full solver, so the only way to give the reduced arms
    // different guesses is to give them different full solvers.
    let base = rig(InitialGuess::PreviousState);
    let pred = rig(InitialGuess::Inertial);

    let fd = base.solver.free_dof_indices().to_vec();
    let mass = base.solver.mass_per_free_dof();

    // Trained from the BASELINE solver. The converged states are the same to
    // tolerance either way, and training off the predicted arm would make the
    // basis a function of the thing under test.
    let mut train = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        let (states, ok) = run_oracle(&base, sample_scaled(k, load_scale));
        if ok.is_none() {
            println!(
                "\n╔═ load ×{load_scale:.1}: SKIPPED — training trajectory {k} left the \
                 material's\n║   validity domain. A physics limit on the fixture, not a \
                 result about the predictor.\n╚═\n"
            );
            return;
        }
        for x in &states {
            train.push(&SnapshotSet::free_displacement(x, &base.x_rest, &fd));
        }
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, r_modes).expect("basis fits");
    let red_base = ReducedNewtonSolver::new(&base.solver, &basis, &base.x_rest);
    let red_pred = ReducedNewtonSolver::new(&pred.solver, &basis, &pred.x_rest);

    let (mut o_base, mut o_pred, mut r_base, mut r_pred) = (0usize, 0usize, 0usize, 0usize);
    for &k in &EVAL_IDS {
        let t = sample_scaled(k, load_scale);
        let cells = (
            run_oracle(&base, t).1,
            run_oracle(&pred, t).1,
            run_reduced(&base, &red_base, t, basis.n_modes()),
            run_reduced(&pred, &red_pred, t, basis.n_modes()),
        );
        if let (Some(a), Some(b), Some(c), Some(d)) = cells {
            o_base += a;
            o_pred += b;
            r_base += c;
            r_pred += d;
        } else {
            println!(
                "\n╔═ load ×{load_scale:.1}: SKIPPED — an arm left the material's validity \
                     domain\n║   (a physics limit on this fixture, not a Newton-difficulty \
                     result)\n╚═\n"
            );
            return;
        }
    }

    // ⚠ CONFOUND CHECK, before any composition number is read as a predictor
    // effect. A larger load deforms further, so a fixed `r` may simply represent
    // it worse — and a reduced trajectory that drifts makes Newton work harder
    // for reasons that have nothing to do with the initial guess. Measure the
    // subspace's own ceiling at this scale so the two can be told apart.
    let mut floor_sum = 0.0_f64;
    let mut floor_n = 0usize;
    for &k in &EVAL_IDS {
        let t = sample_scaled(k, load_scale);
        for x in &run_oracle(&base, t).0 {
            let u = SnapshotSet::free_displacement(x, &base.x_rest, &fd);
            floor_sum += basis.projection_error(&u);
            floor_n += 1;
        }
    }
    let mean_floor = floor_sum / floor_n as f64;

    let steps_total = (EVAL_IDS.len() * STEPS) as f64;
    let per_step = o_base as f64 / steps_total;

    println!(
        "\n╔═ Does the predictor gain survive reduction?  (load ×{load_scale:.0}, r = {r_modes})"
    );
    println!("║ fixture: R1.1 bilayer beam, 5 202 free DOF, r = {r_modes}");
    println!(
        "║ {} held-out trajectories × {STEPS} steps = {steps_total:.0} steps per arm",
        EVAL_IDS.len()
    );
    println!("║");
    println!(
        "║ {:<24} {:>12} {:>12} {:>10}",
        "", "PreviousState", "Inertial", "gain"
    );
    for (label, b, p) in [
        ("oracle (full order)", o_base, o_pred),
        ("reduced (Galerkin)", r_base, r_pred),
    ] {
        let gain = if p == 0 {
            f64::INFINITY
        } else {
            b as f64 / p as f64
        };
        println!("║ {label:<24} {b:>12} {p:>12} {gain:>9.3}×");
    }
    let o_gain = o_base as f64 / o_pred as f64;
    let r_gain = r_base as f64 / r_pred as f64;
    println!("║");
    println!(
        "║ ★ COMPOSITION: reduced gain / oracle gain = {:.3}  (1.0 = composes cleanly)",
        r_gain / o_gain
    );
    println!("║   R1.1's 'identical counts' with a predictor: reduced {r_pred} vs oracle {o_pred}");
    println!(
        "║   basis held-out projection floor here: {:.3} %  ← the CONFOUND: a worse \
         subspace costs iterations on its own",
        100.0 * mean_floor
    );
    println!("║");
    if per_step < MIN_INFORMATIVE_ITERS_PER_STEP {
        println!(
            "║ ⚠⚠ INCONCLUSIVE — the oracle baseline runs {per_step:.2} iterations/step, below \n\
             ║    the {MIN_INFORMATIVE_ITERS_PER_STEP:.1} this comparison needs. A fixture that \n\
             ║    converges in a couple of iterations has no room to show an iteration-count\n\
             ║    gain whatever the predictor does. Do NOT read the ratios above as a result;\n\
             ║    re-run on a harder trajectory with a basis trained on it."
        );
    } else {
        println!(
            "║ oracle baseline {per_step:.2} iters/step — above the {MIN_INFORMATIVE_ITERS_PER_STEP:.1} floor, so the ratios are readable."
        );
    }
    println!("╚═\n");
}

/// The reduced loop REFUSES `InertialWithLoad` rather than half-supporting it.
///
/// Round 3 of review asked for this and it was deferred because the rig was
/// expensive; the rig now exists, so there is no excuse. Guards the construction
/// boundary specifically — the documented contract is that this is a
/// configuration error caught at `new`, before the snapshots and the POD fit are
/// paid for, not something that surfaces mid-trajectory.
#[test]
#[ignore = "reduced-predictor instrument — run explicitly, see module docs"]
#[should_panic(expected = "does not implement")]
fn reduced_refuses_inertial_with_load_at_construction() {
    let r = rig(InitialGuess::InertialWithLoad);
    let fd = r.solver.free_dof_indices().to_vec();
    let mass = r.solver.mass_per_free_dof();
    // A one-snapshot basis is enough: the refusal must fire on the CONFIG, long
    // before anything about the subspace matters.
    let mut train = SnapshotSet::new(fd.len());
    let mut u = vec![0.0; fd.len()];
    u[0] = 1.0e-4;
    train.push(&u);
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, 1).expect("one-mode basis fits");
    // `drop` rather than `let _`: the value is `#[must_use]` and the point is that
    // construction PANICS, so nothing is ever returned to use.
    drop(ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest));
}
