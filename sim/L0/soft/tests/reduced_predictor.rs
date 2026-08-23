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
//! Four arms per cell, over 3 shared held-out trajectories, iteration counts
//! summed per arm; three cells (see `measure`'s call site).
//! The quantity of interest is **`reduced gain ÷ oracle gain`** (the orientation the
//! report prints): at 1.0 the two
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
    Mesh, NullContact, Solver, SolverConfig, SolverFailure, Tet4, Vec3, VertexId,
    pick_vertices_by_predicate,
};

/// Why an arm stopped — and it matters enormously which.
///
/// ⚠ The first version of this instrument collapsed every `SolverFailure` into
/// "left the material's validity domain" and skipped the whole cell. That is
/// only true of `ValidityViolation`. `try_replay_step` also returns
/// `NewtonIterCap` and `ArmijoStall`, which are **exactly what a bad predictor
/// produces** — recon §2f's entire case for killing `InertialWithLoad` is a
/// step-0 Armijo stall. Collapsing them would have printed "SKIPPED — physics
/// limit" over the finding that the predictor HURTS, discarding the result as a
/// fixture problem. The variant is the whole signal; keep it.
enum ArmOutcome {
    Converged(usize),
    /// The FIXTURE cannot take this load — a physics limit, nothing to do with
    /// the initial guess. Skip the cell.
    LeftValidityDomain(String),
    /// Newton failed to converge. This IS a result about the guess.
    DidNotConverge(String),
}

fn classify(fail: &SolverFailure) -> ArmOutcome {
    match fail {
        SolverFailure::ValidityViolation { tet_id, .. } => {
            ArmOutcome::LeftValidityDomain(format!("validity violation at tet {tet_id}"))
        }
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => ArmOutcome::DidNotConverge(format!(
            "NewtonIterCap @ {max_iter}, ‖r‖ = {last_r_norm:.3e}"
        )),
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => ArmOutcome::DidNotConverge(format!(
            "ArmijoStall @ iter {last_iter}, ‖r‖ = {last_r_norm:.3e}"
        )),
        SolverFailure::DoublyFailedFactor { last_iter, .. } => {
            ArmOutcome::DidNotConverge(format!("DoublyFailedFactor @ iter {last_iter}"))
        }
    }
}

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
fn run_oracle(r: &Rig, t: Traj) -> (Vec<Vec<f64>>, ArmOutcome) {
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    let mut out = Vec::new();
    let mut iters = 0usize;
    for s in 1..=STEPS {
        let th = theta_at(r, t, s);
        let step = match r.solver.try_replay_step(
            &Tensor::from_slice(&x, &[r.n_dof]),
            &Tensor::from_slice(&v, &[r.n_dof]),
            &Tensor::from_slice(&th, &[th.len()]),
            DT,
        ) {
            Ok(step) => step,
            Err(fail) => return (out, classify(&fail)),
        };
        iters += step.iter_count;
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
        out.push(x.clone());
    }
    (out, ArmOutcome::Converged(iters))
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
) -> ArmOutcome {
    let mut q = vec![0.0; n_modes];
    let mut qdot = vec![0.0; n_modes];
    let mut iters = 0usize;
    for s in 1..=STEPS {
        let th = theta_at(r, t, s);
        let step = match reduced.step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT) {
            Ok(step) => step,
            Err(fail) => return classify(&fail),
        };
        iters += step.iter_count;
        q = step.q;
        qdot = step.qdot;
    }
    ArmOutcome::Converged(iters)
}

#[test]
#[ignore = "reduced-predictor instrument — run explicitly, see module docs"]
fn does_the_predictor_gain_survive_reduction() {
    // THREE cells. At ×1 (R1.1's own box) the oracle baseline is exactly 3.00
    // iterations/step, which is thin — the gain magnitudes there are weak even
    // though the composition ratio is exact. ×2 makes Newton work harder on the
    // same geometry, trained AND evaluated at the same scale so the basis is
    // never asked to extrapolate outside its box.
    //
    // ⚠ ×4 was the original second cell and is NOT reachable: it leaves the
    // material's validity domain (σ_max = 2.100 against a bound of 2), which is
    // a limit of the FIXTURE, not of Newton. ×2 is the hardest load this
    // geometry admits, and that is a real constraint on how strongly this
    // instrument can test anything.
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
    // ⚠ Cells 2 and 3 share `load_scale`, and `sample_scaled` is a pure function
    // of `(k, load_scale)`, so the 240 full-order training solves below are
    // byte-identical between them — about a third of this instrument's runtime,
    // spent twice. Left as a straight recompute rather than cached because the
    // cache key would be the whole rig configuration and a stale hit here would
    // silently compare two cells against different bases, which is exactly the
    // class of error this instrument exists to avoid. Noted, not hidden.
    let mut train = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        let (states, outcome) = run_oracle(&base, sample_scaled(k, load_scale));
        match outcome {
            ArmOutcome::Converged(_) => {}
            ArmOutcome::LeftValidityDomain(why) => {
                println!(
                    "\n╔═ load ×{load_scale:.1}: SKIPPED — training trajectory {k}: {why}.\n\
                     ║   A physics limit on the FIXTURE, not a result about the predictor.\n╚═\n"
                );
                return;
            }
            ArmOutcome::DidNotConverge(why) => {
                println!(
                    "\n╔═ load ×{load_scale:.1}: ⛔ BASELINE ORACLE FAILED on training \
                     trajectory {k}: {why}.\n║   That is a RESULT, not a skip — the \
                     unpredicted full-order solve cannot converge here, so nothing \
                     downstream\n║   of it means anything.\n╚═\n"
                );
                return;
            }
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
        let cells = [
            ("oracle/PreviousState", run_oracle(&base, t).1),
            ("oracle/Inertial", run_oracle(&pred, t).1),
            (
                "reduced/PreviousState",
                run_reduced(&base, &red_base, t, basis.n_modes()),
            ),
            (
                "reduced/Inertial",
                run_reduced(&pred, &red_pred, t, basis.n_modes()),
            ),
        ];
        let mut counts = [0usize; 4];
        let mut bail: Option<String> = None;
        for (i, (name, outcome)) in cells.iter().enumerate() {
            match outcome {
                ArmOutcome::Converged(n) => counts[i] = *n,
                ArmOutcome::LeftValidityDomain(why) => {
                    bail = Some(format!(
                        "SKIPPED — {name} on trajectory {k}: {why}. A physics limit on the \
                         FIXTURE, not a result about the predictor."
                    ));
                }
                ArmOutcome::DidNotConverge(why) => {
                    bail = Some(format!(
                        "⛔ {name} FAILED TO CONVERGE on trajectory {k}: {why}. This is a \
                         RESULT about that arm, NOT a fixture problem — the predictor put \
                         Newton somewhere it could not recover from."
                    ));
                }
            }
        }
        if let Some(msg) = bail {
            println!("\n╔═ load ×{load_scale:.1}, r = {r_modes}: {msg}\n╚═\n");
            return;
        }
        {
            let [a, b, c, d] = counts;
            o_base += a;
            o_pred += b;
            r_base += c;
            r_pred += d;
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
        let (states, outcome) = run_oracle(&base, t);
        // The eval loop above already returned on any non-convergence, so this
        // cannot fire — assert rather than average over a truncated `states`,
        // which would silently bias the very number the confound control reads.
        assert!(
            matches!(outcome, ArmOutcome::Converged(_)),
            "baseline oracle changed outcome between the eval and confound passes",
        );
        for x in &states {
            let u = SnapshotSet::free_displacement(x, &base.x_rest, &fd);
            floor_sum += basis.projection_error(&u);
            floor_n += 1;
        }
    }
    assert!(
        floor_n > 0,
        "no snapshots to measure the projection floor over"
    );
    let mean_floor = floor_sum / floor_n as f64;

    let steps_total = (EVAL_IDS.len() * STEPS) as f64;
    let per_step = o_base as f64 / steps_total;

    println!(
        "\n╔═ Does the predictor gain survive reduction?  (load ×{load_scale:.0}, r = {r_modes})"
    );
    println!(
        "║ fixture: R1.1 bilayer beam, 5 202 free DOF, r = {} retained (requested {r_modes})",
        basis.n_modes()
    );
    assert!(
        basis.n_modes() == r_modes,
        "PodBasis::fit retained {} of the {r_modes} modes requested — it truncates below a \
         relative singular-value floor. The confound-control cell exists to ask whether ADDING \
         modes restores the projection floor, so a silent truncation would make its negative \
         result unfalsifiable: the reader concludes 'more modes did not help' when no modes \
         were added.",
        basis.n_modes(),
    );
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
    // Guarded like the table rows: `iter_count` is 0-based, so an arm that lands
    // inside `tol` at iteration 0 on every step totals zero.
    let ratio = |b: usize, p: usize| {
        if p == 0 {
            f64::NAN
        } else {
            b as f64 / p as f64
        }
    };
    let o_gain = ratio(o_base, o_pred);
    let r_gain = ratio(r_base, r_pred);
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
    // ⚠ How well is the composition ratio actually RESOLVED? These are integer
    // counts, so the honest statement is not "above an arbitrary floor" but "a
    // one-iteration change in any arm moves the answer by this much". The first
    // version of this check compared `per_step < 3.0` against a baseline of
    // exactly 3.00 and duly printed "readable" over the one cell its own comment
    // called degenerate.
    let worst_wobble = [
        (o_base + 1, o_pred, r_base, r_pred),
        (o_base, o_pred + 1, r_base, r_pred),
        (o_base, o_pred, r_base + 1, r_pred),
        (o_base, o_pred, r_base, r_pred + 1),
    ]
    .into_iter()
    .map(|(ob, op, rb, rp)| ((ratio(rb, rp) / ratio(ob, op)) - r_gain / o_gain).abs())
    .fold(0.0_f64, f64::max);
    println!("║   resolution: ±1 iteration in any arm moves the composition by {worst_wobble:.3}");
    let r_per_step = r_base as f64 / steps_total;
    if per_step <= MIN_INFORMATIVE_ITERS_PER_STEP || r_per_step <= MIN_INFORMATIVE_ITERS_PER_STEP {
        println!(
            "║ ⚠⚠ THIN — baselines run {per_step:.2} (oracle) / {r_per_step:.2} (reduced) \n\
             ║    iterations/step, at or below the {MIN_INFORMATIVE_ITERS_PER_STEP:.1} floor. The \n\
             ║    GAIN MAGNITUDES are weak here. ★ The COMPOSITION ratio can still be read when\n\
             ║    the two arms' counts match exactly — that is a structural agreement, not a\n\
             ║    magnitude — but weigh it against the resolution line above."
        );
    } else {
        println!(
            "║ baselines {per_step:.2} (oracle) / {r_per_step:.2} (reduced) iters/step — \
             both above the {MIN_INFORMATIVE_ITERS_PER_STEP:.1} floor."
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
// NOT `#[ignore]`d, unlike the measurement above. The module doc's rationale
// ("an instrument, costs CI nothing") applies to the 80-second sweep, not to a
// guard: this builds one mesh, fits a one-snapshot basis, runs no trajectory,
// and costs CI about a second. Review found the port shipping with the refusal
// and the `Inertial` arm both covered only by `#[ignore]`d code — i.e. by
// nothing.
#[test]
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

/// A small, fast rig for the CI-covered tests: same physics, a mesh small enough
/// to train and solve in well under a second.
fn small_basis_rig(guess: InitialGuess) -> (Rig, PodBasis) {
    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(4, 4, 2, LX, LY, H, &field);
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
    let r = Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    };
    let fd = r.solver.free_dof_indices().to_vec();
    let mass = r.solver.mass_per_free_dof();
    let mut train = SnapshotSet::new(fd.len());
    for k in 0..6_u64 {
        let (states, outcome) = run_oracle(&r, sample_scaled(k, 1.0));
        assert!(
            matches!(outcome, ArmOutcome::Converged(_)),
            "small rig trains"
        );
        for x in &states {
            train.push(&SnapshotSet::free_displacement(x, &r.x_rest, &fd));
        }
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, 8).expect("small basis fits");
    (r, basis)
}

/// The reduced loop's `Inertial` arm reaches the same root as `PreviousState`.
///
/// The port's whole claim is that the guess changes the PATH the reduced Newton
/// walks, not the `q` it walks to. This is the reduced-coordinate sibling of the
/// full-order `initial_guess_variants_converge_to_the_same_root`, and it is the
/// only automated coverage the `Inertial` match arm in `reduced::newton::step`
/// has — the sweep above is `#[ignore]`d.
#[test]
fn reduced_inertial_reaches_the_same_root_as_previous_state() {
    /// Well inside what `tol = 1e-6` on `‖Φᵀr‖` pins down, loose enough not to
    /// pin bits: the two solves take different paths and need not agree exactly.
    const SAME_ROOT_REL_TOL: f64 = 1.0e-6;

    let (base, basis) = small_basis_rig(InitialGuess::PreviousState);
    let (pred, _) = small_basis_rig(InitialGuess::Inertial);
    let red_base = ReducedNewtonSolver::new(&base.solver, &basis, &base.x_rest);
    let red_pred = ReducedNewtonSolver::new(&pred.solver, &basis, &pred.x_rest);

    let t = sample_scaled(900, 1.0);
    let n = basis.n_modes();
    let (mut qb, mut qdb) = (vec![0.0; n], vec![0.0; n]);
    let (mut qp, mut qdp) = (vec![0.0; n], vec![0.0; n]);
    let mut moved = false;
    for s in 1..=STEPS {
        let th = Tensor::from_slice(&theta_at(&base, t, s), &[3 * base.loaded.len()]);
        let sb = red_base
            .step(&qb, &qdb, &th, DT)
            .expect("baseline reduced step");
        let sp = red_pred
            .step(&qp, &qdp, &th, DT)
            .expect("predicted reduced step");
        qb = sb.q;
        qdb = sb.qdot;
        qp = sp.q;
        qdp = sp.qdot;
        // Negative control on the test itself: with a zero initial velocity the
        // two guesses coincide, so a run where `qdot` never leaves zero would
        // compare an arm against itself.
        if qdb.iter().any(|v| v.abs() > 0.0) {
            moved = true;
        }
    }
    assert!(
        moved,
        "qdot never left zero, so Inertial and PreviousState were the same point \
         throughout and this test compared an arm against itself",
    );
    let den = qb.iter().map(|a| a * a).sum::<f64>().sqrt();
    assert!(den > 0.0, "the baseline reduced solve never moved");
    let diff = qb
        .iter()
        .zip(&qp)
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt();
    assert!(
        diff / den < SAME_ROOT_REL_TOL,
        "reduced Inertial converged to a DIFFERENT q than PreviousState: relative \
         separation {:.3e} exceeds {SAME_ROOT_REL_TOL:.0e}",
        diff / den,
    );
}

/// `Inertial` produces EXACTLY the documented reduced first iterate,
/// `q⁰ = q_prev + Δt·q̇_prev`.
///
/// ⚠ **The same-root test above cannot catch a no-op predictor** — if the
/// `Inertial` arm silently fell back to `q_prev`, both arms would be identical
/// and a "they agree" assertion would pass with flying colours. This is the
/// discriminating one, and it uses the same trick as the full-order sibling: a
/// huge `tol` makes the reduced Newton converge at iteration 0, so the returned
/// `q` **is** the initial iterate and the algebra itself becomes observable.
///
/// Needs no trajectory at all — one step with a nonzero `q̇_prev` is the whole
/// experiment.
#[test]
fn reduced_inertial_produces_the_documented_first_iterate() {
    const HUGE_TOL: f64 = 1.0e30;

    let build = |guess: InitialGuess| {
        let (mut r, basis) = small_basis_rig(guess);
        // Rebuild the solver with the enormous tolerance; everything else is the
        // small rig's configuration unchanged.
        let field = MaterialField::uniform(MU, LAMBDA);
        let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(4, 4, 2, LX, LY, H, &field);
        let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-12);
        let loaded: Vec<VertexId> =
            pick_vertices_by_predicate(&mesh, |p: &Vec3| (p.z - H).abs() < 1e-12);
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = DT;
        cfg.density = DENSITY;
        cfg.max_newton_iter = 80;
        cfg.tol = HUGE_TOL;
        cfg.initial_guess = guess;
        r.solver = CpuTet4NHSolver::new(
            Tet4,
            mesh,
            NullContact,
            cfg,
            BoundaryConditions {
                pinned_vertices: pins,
                roller_vertices: Vec::new(),
                loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
            },
        );
        (r, basis)
    };

    let (base, basis) = build(InitialGuess::PreviousState);
    let (pred, _) = build(InitialGuess::Inertial);
    let n = basis.n_modes();
    assert!(n >= 2, "need a couple of modes for this to say anything");

    // Nonzero in every mode — the input the predictor is supposed to consume.
    //
    // ⚠ These look absurdly small and are not. The basis is M-ORTHONORMAL
    // (`ΦᵀMΦ = I`), so a reduced coordinate is a mass-weighted amplitude, not a
    // displacement: `‖Φq‖_M = ‖q‖`, and this beam masses ~2.5e-3 kg, so a `q` of
    // 1e-5 is already a sub-millimetre field on a 6 mm layer — enough, summed
    // over modes, to inflict `det F < 0` and trip the Decision Q validity gate
    // before any of the algebra below can be read. Measured, not guessed: the
    // first draft used 1e-5 and died on tet 33 with `det F = -0.086`.
    let q_prev: Vec<f64> = (0..n).map(|i| 1.0e-9 * (i as f64 + 1.0)).collect();
    let qdot_prev: Vec<f64> = (0..n).map(|i| 1.0e-7 * (i as f64 + 1.0)).collect();
    let th = Tensor::from_slice(&vec![0.0; 3 * base.loaded.len()], &[3 * base.loaded.len()]);

    let red_base = ReducedNewtonSolver::new(&base.solver, &basis, &base.x_rest);
    let red_pred = ReducedNewtonSolver::new(&pred.solver, &basis, &pred.x_rest);
    let sb = red_base
        .step(&q_prev, &qdot_prev, &th, DT)
        .expect("baseline");
    let sp = red_pred
        .step(&q_prev, &qdot_prev, &th, DT)
        .expect("predicted");

    assert_eq!(sb.iter_count, 0, "tol 1e30 must converge at iteration 0");
    assert_eq!(sp.iter_count, 0, "tol 1e30 must converge at iteration 0");
    assert_eq!(sb.q, q_prev, "PreviousState must return q_prev untouched");

    let want: Vec<f64> = q_prev
        .iter()
        .zip(&qdot_prev)
        .map(|(&q, &qd)| DT.mul_add(qd, q))
        .collect();
    assert_eq!(
        sp.q, want,
        "Inertial must return exactly q_prev + dt*qdot_prev; getting q_prev back \
         would mean the arm is a silent no-op, which the same-root test cannot see",
    );
}

// ── WHY does reduction cost iterations under load? ────────────────────────

/// A reduced step's converged diagnostics, which `ReducedStep` already exposes.
struct RedDiag {
    iters: usize,
    /// `‖Φᵀr‖` at convergence — the quantity the reduced Newton descends.
    proj: f64,
    /// `‖r_free‖` at convergence — the full-order residual the subspace CANNOT
    /// remove. A Galerkin solve makes the residual orthogonal to the basis, not
    /// zero, so this is expected to be large; what matters is how it MOVES.
    full: f64,
}

fn run_reduced_diag(
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
) -> Option<RedDiag> {
    let mut q = vec![0.0; n_modes];
    let mut qdot = vec![0.0; n_modes];
    let (mut iters, mut proj, mut full) = (0usize, 0.0_f64, 0.0_f64);
    for s in 1..=STEPS {
        let th = theta_at(r, t, s);
        let step = reduced
            .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
            .ok()?;
        iters += step.iter_count;
        proj += step.projected_residual_norm;
        full += step.full_residual_norm;
        q = step.q;
        qdot = step.qdot;
    }
    Some(RedDiag {
        iters,
        proj: proj / STEPS as f64,
        full: full / STEPS as f64,
    })
}

/// Why does the reduced solve need MORE Newton iterations than the oracle once
/// the load rises — and why doesn't rank fix it?
///
/// The companion measurement found that at ×2 load the reduced solve takes ~17 %
/// more iterations than the oracle with no predictor involved, and that DOUBLING
/// the modes makes it slightly worse while making the subspace strictly better
/// (projection floor 0.352 % against ×1's 0.455 %). So it is not truncation.
///
/// ## What this separates
///
/// The reduced loop converges on `‖Φᵀr‖ < tol` — an ABSOLUTE tolerance on a
/// projected quantity, descended by its own Armijo. Two candidate mechanisms
/// make opposite predictions under a `tol` sweep at fixed load:
///
/// - **A residual FLOOR.** Nonlinearity couples content the subspace cannot
///   represent back into `Φᵀr`, so the last iterations grind against a floor.
///   ⇒ tightening `tol` blows the reduced count up relative to the oracle's.
/// - **A RATE problem.** The Galerkin direction is simply a worse descent
///   direction under load, costing a roughly constant factor per digit.
///   ⇒ the reduced/oracle ratio stays flat as `tol` tightens.
///
/// Load and `r` are held fixed; only `tol` moves. The basis is trained ONCE at
/// the tightest tolerance and shared, so the subspace is identical across cells
/// and cannot itself explain a trend.
#[test]
#[ignore = "reduced-predictor instrument — run explicitly, see module docs"]
fn why_does_reduction_cost_iterations_under_load() {
    const LOAD: f64 = 2.0;
    const TOLS: [f64; 3] = [1.0e-4, 1.0e-6, 1.0e-8];
    /// Basis training runs at the TIGHTEST tolerance so every cell shares one
    /// subspace; a per-cell basis would confound the trend with its own quality.
    const TRAIN_TOL: f64 = 1.0e-8;

    let rig_at = |guess: InitialGuess, tol: f64| {
        let mut r = rig(guess);
        let field = MaterialField::uniform(MU, LAMBDA);
        let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(16, 16, 6, LX, LY, H, &field);
        let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-12);
        let loaded: Vec<VertexId> =
            pick_vertices_by_predicate(&mesh, |p: &Vec3| (p.z - H).abs() < 1e-12);
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = DT;
        cfg.density = DENSITY;
        cfg.max_newton_iter = 200;
        cfg.tol = tol;
        cfg.initial_guess = guess;
        r.solver = CpuTet4NHSolver::new(
            Tet4,
            mesh,
            NullContact,
            cfg,
            BoundaryConditions {
                pinned_vertices: pins,
                roller_vertices: Vec::new(),
                loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
            },
        );
        r
    };

    let train_rig = rig_at(InitialGuess::PreviousState, TRAIN_TOL);
    let fd = train_rig.solver.free_dof_indices().to_vec();
    let mass = train_rig.solver.mass_per_free_dof();
    let mut train = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        let (states, outcome) = run_oracle(&train_rig, sample_scaled(k, LOAD));
        assert!(
            matches!(outcome, ArmOutcome::Converged(_)),
            "training must converge at the tightest tolerance",
        );
        for x in &states {
            train.push(&SnapshotSet::free_displacement(x, &train_rig.x_rest, &fd));
        }
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, R_MODES).expect("basis fits");

    println!(
        "\n╔═ Why does reduction cost iterations under load?  (load ×{LOAD:.0}, r = {})",
        basis.n_modes()
    );
    println!("║ one shared basis, trained at tol {TRAIN_TOL:.0e}; only `tol` varies");
    println!(
        "║ {:>8} {:>8} {:>9} {:>9} {:>12} {:>12}",
        "tol", "oracle", "reduced", "red/orc", "‖Φᵀr‖ conv", "‖r‖ conv"
    );
    let mut ratios = Vec::new();
    for tol in TOLS {
        let base = rig_at(InitialGuess::PreviousState, tol);
        let red = ReducedNewtonSolver::new(&base.solver, &basis, &base.x_rest);
        let (mut o, mut ri) = (0usize, 0usize);
        let (mut pj, mut fl) = (0.0_f64, 0.0_f64);
        let mut ok = true;
        for &k in &EVAL_IDS {
            let t = sample_scaled(k, LOAD);
            match run_oracle(&base, t).1 {
                ArmOutcome::Converged(n) => o += n,
                _ => ok = false,
            }
            match run_reduced_diag(&base, &red, t, basis.n_modes()) {
                Some(d) => {
                    ri += d.iters;
                    pj += d.proj;
                    fl += d.full;
                }
                None => ok = false,
            }
        }
        if !ok {
            println!("║ {tol:>8.0e}   an arm failed to converge — cell dropped");
            continue;
        }
        let n = EVAL_IDS.len() as f64;
        let ratio = ri as f64 / o as f64;
        ratios.push((tol, ratio));
        println!(
            "║ {tol:>8.0e} {o:>8} {ri:>9} {ratio:>8.3}× {:>12.2e} {:>12.2e}",
            pj / n,
            fl / n
        );
    }
    println!("║");
    if let (Some(&(lo_t, lo)), Some(&(hi_t, hi))) = (ratios.first(), ratios.last()) {
        let growth = hi / lo;
        println!(
            "║ reduced/oracle ratio {lo:.3}× at tol {lo_t:.0e} → {hi:.3}× at tol {hi_t:.0e} \
             (growth {growth:.3}×)"
        );
        if growth > 1.15 {
            println!(
                "║ ⇒ FLOOR: the ratio grows as tol tightens. The reduced solve is grinding \
                 against a\n║   residual floor the subspace cannot get under — nonlinear \
                 coupling of content\n║   Φ does not span back into Φᵀr."
            );
        } else {
            println!(
                "║ ⇒ RATE, not a floor: the ratio is ~flat in tol, so the reduced solve pays a \
                 roughly\n║   CONSTANT factor per digit. The Galerkin direction is simply worse \
                 under load;\n║   it is not stalling against something it cannot represent."
            );
        }
    }
    println!("╚═\n");
}
