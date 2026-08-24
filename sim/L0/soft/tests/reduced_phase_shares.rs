//! Where does a REDUCED frame's time go, and what does that cap R3 at?
//!
//! ## Why this must run before R3, not after
//!
//! Recon §2f puts R3's requirement at **13.5–15.8×** on the reference box. §4b
//! justifies picking ECSW by a literature claim of 2–3 orders of magnitude — but
//! that is a speedup on the *element integration*, and R3's whole-step gain is
//! capped by whatever fraction of a reduced frame that actually is. Amdahl, and
//! the exact trap that made R0's credit wrong by 37 % (§2d finding 3).
//!
//! That fraction has never been measured. §2d's table is the FULL-ORDER path;
//! `src/profile.rs` said in so many words that the reduced solver carried no
//! timers. So R3 would have been built without knowing its own ceiling, and the
//! ceiling would have arrived at the end of the most expensive rung on the
//! ladder.
//!
//! ```text
//! cargo test --release -p sim-soft --features phase-timing \
//!   --test reduced_phase_shares -- --ignored --nocapture --test-threads=1
//! ```
//!
//! ## What it answers
//!
//! ECSW replaces assembly-and-projection over every element with a weighted sum
//! over a sampled subset, so [`Phase::ecsw_reducible`] splits the slots. With
//! `f` the reducible share, **R3's whole-step ceiling is `1/(1 − f)`** — the
//! limit as the sampled subset goes to zero cost.
//!
//! `f` is reported as a BRACKET, because some of the frame is removable only if
//! R3's own `ReducedValidityDomain` works ([`Reducible::PlannedByR3`]); folding
//! that in would assume the success of the thing being sized. Measured, the
//! bracket first read `8.3×–37.1×` against a `13.5–15.8×` requirement and a `10×`
//! floor — straddling both, with the validity element sweep alone deciding it.
//! Parallelising that sweep took it to `20.2–20.5×–≳32×` over two runs, clearing on
//! either bound
//! (recon §2i, v2.7). The straddle is why this harness exists; keep it, because
//! `project_tangent` and R3 itself will both move the mix again.
//!
//! ## ★★ What the ceiling ALONE cannot tell you (recon §2j, v2.8)
//!
//! `C = T/I` and the requirement `R = T/B` share a numerator, so `C/R = B/I` and
//! **R3 clears iff `I ≤ B`** — the frame's IRREDUCIBLE time against the budget.
//! Two consequences for how this harness's output gets read:
//!
//! 1. **Speeding up any [`Reducible::Yes`] phase leaves R3's margin exactly
//!    unchanged**, because it lowers `C` and `R` by the same factor. The
//!    `red proj K` row is `56.8 %` of the frame and moves R3's verdict by zero.
//!    A ceiling re-quoted after such a change MUST carry its own re-measured
//!    requirement, or it reads as a regression that did not happen.
//! 2. **The rows that decide R3 are the irreducible ones**, and they are small:
//!    `I = 3.223 ms`, `61 %` of it `contact` marshalling on a `NullContact`
//!    scene. ⚠ Which is also the limit of this fixture — `I` has never been
//!    measured on a contact fixture, where the requirement actually lives.
//!
//! ⚠ The `10×` floor named above was restated in v2.8 and is now a complexity
//! heuristic, not a gate.
//!
//! ## Controls
//!
//! ⚠ There is **no published reduced-path row to reproduce**, so the positive
//! control `tests/phase_shares.rs` relies on is not available here. Two weaker
//! checks stand in, and both assert:
//!
//! 1. **Cross-path assembly cost.** The reduced solver calls the SAME
//!    `assemble_global_int_force` on the SAME mesh as a full-order solve, so the
//!    per-call cost must agree. If it does not, the timers are not measuring
//!    what their names claim.
//! 2. **Coverage.** Instrumented time over wall time is reported; a low figure
//!    means the slots are missing a cost centre and `f` is computed against an
//!    incomplete denominator.

#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::too_many_lines,
    // `sample` is reproduced verbatim from `reduced_predictor.rs` so the two
    // fixtures cannot drift; renaming its locals would defeat that.
    clippy::many_single_char_names
)]

mod reduced_report;
mod refbox;

use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::profile::{self, Phase};
use sim_soft::solver::backward_euler::reduced::{
    Inner, PodBasis, ReducedNewtonSolver, SnapshotSet,
};
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, HandBuiltTetMesh, InitialGuess, LoadAxis, MaterialField,
    Mesh, NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

const LX: f64 = 0.020;
const LY: f64 = 0.020;
const H: f64 = 0.006;
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0 * MU;
const DENSITY: f64 = 1030.0;
const DT: f64 = 1.0 / 60.0;
const NEWTON_CAP: usize = 200;
const TOL: f64 = 1.0e-8;
/// R1.1's operating point.
const N_LAT: usize = 16;
const NZ: usize = 6;
const R_MODES: usize = 40;
/// Trajectories in the training set; each contributes [`TRAJ_STEPS`] snapshots.
const N_TRAIN: usize = 12;
/// Steps per trajectory — R1.1's value, and what `frac` below normalises against.
const TRAJ_STEPS: usize = 5;
const WARMUP: usize = 1;
const STEPS: usize = 4;

struct Rig {
    solver: CpuTet4NHSolver<HandBuiltTetMesh>,
    x_rest: Vec<f64>,
    loaded: Vec<VertexId>,
    n_dof: usize,
}

fn rig() -> Rig {
    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(N_LAT, N_LAT, NZ, LX, LY, H, &field);
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
    cfg.max_newton_iter = NEWTON_CAP;
    cfg.tol = TOL;
    cfg.initial_guess = InitialGuess::PreviousState;
    Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    }
}

/// One loading trajectory: a Gaussian pressure patch on the top face.
#[derive(Clone, Copy)]
struct Traj {
    cx: f64,
    cy: f64,
    w: f64,
    p0: f64,
}

/// `reduced_predictor::sample_scaled`, reproduced so the fixture is R1.1's and
/// not a lookalike. ⚠ An earlier version of this harness invented its own ramp
/// (`-4e-3 · s` of displacement) and drove the beam straight out of the validity
/// domain at training step 48. The load here is a PRESSURE, `p0 ≈ 0.10–0.30`,
/// ramped over `TRAJ_STEPS`.
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

fn theta_at(r: &Rig, t: Traj, s: usize) -> Vec<f64> {
    let frac = s as f64 / TRAJ_STEPS as f64;
    let mut th = vec![0.0; 3 * r.loaded.len()];
    for (i, &vid) in r.loaded.iter().enumerate() {
        let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
        let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
        th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
    }
    th
}

#[test]
#[ignore = "reduced phase-share instrument — needs --features phase-timing"]
fn reduced_phase_shares() {
    refbox::require_quiet_box();

    let r = rig();
    let fd = r.solver.free_dof_indices().to_vec();
    let mass = r.solver.mass_per_free_dof();

    // ── train the basis on a full-order run ──
    let mut train = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        let t = sample(k);
        let mut x = r.x_rest.clone();
        let mut v = vec![0.0; r.n_dof];
        for s in 1..=TRAJ_STEPS {
            let th = theta_at(&r, t, s);
            let step = r
                .solver
                .try_replay_step(
                    &Tensor::from_slice(&x, &[r.n_dof]),
                    &Tensor::from_slice(&v, &[r.n_dof]),
                    &Tensor::from_slice(&th, &[th.len()]),
                    DT,
                )
                .expect("training step converges");
            for i in 0..r.n_dof {
                v[i] = (step.x_final[i] - x[i]) / DT;
            }
            x = step.x_final;
            train.push(&SnapshotSet::free_displacement(&x, &r.x_rest, &fd));
        }
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, R_MODES).expect("basis fits");
    let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);

    // ── the measurement ──
    let run = sample(N_TRAIN as u64); // a trajectory NOT in the training set
    let mut q = vec![0.0; basis.n_modes()];
    let mut qdot = vec![0.0; basis.n_modes()];
    let (mut wall, mut iters, mut measured) = (0.0_f64, 0usize, 0usize);
    for s in 1..=(STEPS + WARMUP) {
        if s == WARMUP + 1 {
            profile::reset();
        }
        let th = theta_at(&r, run, s);
        let t0 = Instant::now();
        let step = reduced
            .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
            .expect("reduced step converges");
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        if s > WARMUP {
            wall += ms;
            iters += step.iter_count;
            measured += 1;
        }
        q = step.q;
        qdot = step.qdot;
    }
    let red = profile::snapshot();
    let red_wall = wall;
    reduced_report::main_report("REDUCED (R1.1 operating point)", wall, measured, iters);

    // ── control 1: the same assembly, driven full-order, must cost the same ──
    profile::reset();
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    for s in 1..=3 {
        let th = theta_at(&r, run, s);
        let step = r
            .solver
            .try_replay_step(
                &Tensor::from_slice(&x, &[r.n_dof]),
                &Tensor::from_slice(&v, &[r.n_dof]),
                &Tensor::from_slice(&th, &[th.len()]),
                DT,
            )
            .expect("full step converges");
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
    }
    let full = profile::snapshot();

    println!("\n★ CONTROL — same assembly, same mesh, driven both ways (per CALL):");
    let mut worst = 0.0_f64;
    for ph in [Phase::AssembleForce, Phase::AssembleTangent] {
        let rc = red.calls(ph);
        let fc = full.calls(ph);
        assert!(
            rc > 0 && fc > 0,
            "{}: {rc} reduced / {fc} full calls — a zero means the phase never ran, \
             so this control cannot fail for the reason it claims",
            ph.label()
        );
        let rp = red.millis(ph) / rc as f64;
        let fp = full.millis(ph) / fc as f64;
        let ratio = rp / fp;
        worst = worst.max(if ratio > 1.0 { ratio } else { 1.0 / ratio });
        println!(
            "  {:<16} reduced {:.4} ms/call   full {:.4} ms/call   ratio {ratio:.2}×",
            ph.label(),
            rp,
            fp
        );
    }
    assert!(
        worst <= 1.35,
        "assembly cost disagrees by {worst:.2}× between the reduced and full paths on \
         the SAME mesh. They call the same function, so the timers are not measuring \
         what their names say and the reducible share above cannot be read."
    );

    // ★ How much reduction raises the share of what it does not touch. Same
    // fixture, same mesh, both paths, so this is measured rather than inferred —
    // and it came out SMALLER than the cross-fixture figures suggest. Against
    // `cantilever 80x8` (19 440 DOF) the validity sweep is 0.5 % of a full frame,
    // which invites the story "reduction inflates it 20x". On one fixture it goes
    // 7.6 % -> 9.6 %. The 0.5 % is an artifact of a much larger mesh where the
    // sparse factorization is 72 % and squeezes everything else.
    println!("\n★ FIXED COSTS — the same work as a share of each path's frame:");
    let red_total: f64 = Phase::ALL
        .iter()
        .filter(|ph| !matches!(ph, Phase::Contact))
        .map(|ph| red.millis(*ph))
        .sum();
    let full_total: f64 = Phase::ALL
        .iter()
        .filter(|ph| !matches!(ph, Phase::Contact))
        .map(|ph| full.millis(*ph))
        .sum();
    for ph in [Phase::ValidityCheck, Phase::AssembleTangent] {
        println!(
            "  {:<16} full {:5.1} %   reduced {:5.1} %",
            ph.label(),
            100.0 * full.millis(ph) / full_total,
            100.0 * red.millis(ph) / red_total
        );
    }
    println!(
        "  ⇒ modest inflation (~1.3x on share). Both figures are POST the parallel\n\
         \x20   sweep (recon §2i v2.7); before it they read 7.6 % / 9.6 %, and that\n\
         \x20   9.3 % of a reduced frame was what made R3's ceiling straddle."
    );

    let instr: f64 = Phase::ALL
        .iter()
        .filter(|ph| !matches!(ph, Phase::Contact))
        .map(|ph| red.millis(*ph))
        .sum();
    assert!(
        instr / red_wall >= 0.85,
        "only {:.1} % of the reduced frame is instrumented, so the ECSW-reducible \
         share is computed against an incomplete denominator and its Amdahl ceiling \
         is not trustworthy. Add the missing cost centre before reading it.",
        100.0 * instr / red_wall
    );
}
