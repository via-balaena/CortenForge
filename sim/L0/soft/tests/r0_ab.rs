//! Direct A/B of wall ms/step across R0, on ONE box, with NO instrument.
//!
//! ## Why this exists
//!
//! §2d finding 4 derived R0's whole-step credit by differencing phase SHARES
//! measured in two different sessions, on two different boxes, with two
//! different instruments (the pre-R0 one was scratch patches, since reverted).
//! That derivation rests on "R0 touched only tangent assembly, so every other
//! phase's absolute cost is unchanged" — and the data falsify it: across the
//! two sessions `asm force` moves 2.4x and `tri solve` 1.7x, where the premise
//! predicts they move by the credit (~1.19x). The two large phases agree and
//! the two small ones do not, which is what a phase-boundary or denominator
//! difference between two instruments looks like. It is not recoverable after
//! the fact.
//!
//! So stop differencing shares. R0's credit is a ratio of WALL TIMES, and both
//! trees are in git: check out each, run the same fixture on the same box, and
//! divide. No shares, no cross-session transfer, no instrument to trust.
//!
//! ## Portability is the whole point
//!
//! This file must compile UNCHANGED on both trees, so it imports nothing that
//! postdates the pre-R0 commit — in particular no `sim_soft::profile` (added
//! after R0) and no `usize::is_multiple_of` (a later MSRV than that tree pins).
//!
//! ## Running it
//!
//! Build the pre-R0 arm as a detached worktree, so it keeps its own `target/` and
//! neither arm rebuilds when you alternate:
//!
//! ```text
//! git worktree add --detach /tmp/pre-r0 ecf4cfef^   # R0 landed as ecf4cfef (#742)
//! cp sim/L0/soft/tests/r0_ab.rs /tmp/pre-r0/sim/L0/soft/tests/
//! ```
//!
//! Then run once per arm, INTERLEAVED (A B A B A B, never AAA BBB) so thermal
//! drift and background load land on both arms instead of being confounded with
//! R0:
//!
//! ```text
//! cargo test --release -p sim-soft --test r0_ab -- --ignored --nocapture --test-threads=1
//! ```
//!
//! ⚠ Both trees pin the same toolchain (1.96.0) via `rust-toolchain.toml`; if that
//! ever stops being true the arms are measuring the compiler, not R0. And check
//! `iters_per_step` MATCHES across arms before reading any timing — R0 was
//! byte-identical, so a difference there means the two arms are not on the same
//! trajectory and the ratio is meaningless.

#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, IpcRigidContact,
    IpcRigidContactSolver, MaterialField, Mesh, NullContact, Solver, SolverConfig, SphereSdf, Tet4,
    TranslatedSdf, Vec3, pick_vertices_by_predicate,
};

const DT: f64 = 1.0 / 60.0;
const DENSITY: f64 = 1030.0;
const NEWTON_CAP: usize = 200;
const WARMUP_STEPS: usize = 2;

const RADIUS: f64 = 1.0e-2;
const DELTA: f64 = 5.0e-4;
const KAPPA: f64 = 1.0e4;
const BAND_FRAC: f64 = 0.05;
const LAYER_MU: f64 = 1.0e5;
const CHI: f64 = 0.35;

/// `bonded_layer_indentation::dims_for`, reproduced verbatim except for the
/// even-ness test.
//
// manual_is_multiple_of: `n % 2 == 0` is deliberate and must NOT be "fixed".
// `usize::is_multiple_of` postdates the MSRV of the pre-R0 tree this file is
// copied into to run the A/B, and the arms are only comparable if the harness
// is byte-identical on both. Taking clippy's suggestion would silently make the
// file un-runnable on the arm that matters.
#[allow(clippy::manual_is_multiple_of)]
fn dims_for(a_over_cell: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / a_over_cell;
    let h = a / CHI;
    let lateral = 8.0 * a;
    let even = |n: usize| if n % 2 == 0 { n } else { n + 1 };
    (
        even(((lateral / cell).round() as usize).max(2)),
        even(((h / cell).round() as usize).max(2)),
        lateral,
        h,
    )
}

fn indenter(lx: f64, ly: f64, z: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(lx / 2.0, ly / 2.0, z),
    }
}

/// One fixture: the IPC indentation ramp, timed with `Instant` only.
fn measure(a_over_cell: f64, label: &str) {
    let (n_lat, nz, lateral, h) = dims_for(a_over_cell);
    let d_hat = BAND_FRAC * DELTA;
    let z_start = h + RADIUS + 1.2 * d_hat;
    let z_end = h + RADIUS - DELTA;
    let inc = 0.3 * d_hat;
    let n_steps = ((z_start - z_end) / inc).ceil() as usize;

    let field = MaterialField::uniform(LAYER_MU, 4.0 * LAYER_MU);
    let mesh =
        HandBuiltTetMesh::cantilever_bilayer_beam(n_lat, n_lat, nz, lateral, lateral, h, &field);
    let n_dof = 3 * mesh.n_vertices();
    let mut x = vec![0.0; n_dof];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-9);
    let free = n_dof - 3 * pins.len();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.max_newton_iter = NEWTON_CAP;
    let mut solver: IpcRigidContactSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        mesh,
        IpcRigidContact::with_params(vec![indenter(lateral, lateral, z_start)], KAPPA, d_hat),
        cfg,
        BoundaryConditions::new(pins, Vec::new()),
    );

    let theta = Tensor::from_slice(&[], &[0]);
    let mut v = vec![0.0; n_dof];
    let mut per_step: Vec<f64> = Vec::new();
    let mut iters = 0usize;

    for k in 0..n_steps {
        let z = (z_start - (k + 1) as f64 * inc).max(z_end);
        solver.replace_contact(IpcRigidContact::with_params(
            vec![indenter(lateral, lateral, z)],
            KAPPA,
            d_hat,
        ));
        let t0 = Instant::now();
        let step = solver
            .try_replay_step(
                &Tensor::from_slice(&x, &[n_dof]),
                &Tensor::from_slice(&v, &[n_dof]),
                &theta,
                DT,
            )
            .unwrap_or_else(|e| panic!("IPC step {k} failed: {e:?}"));
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        if k >= WARMUP_STEPS {
            per_step.push(ms);
            iters += step.iter_count;
        }
        for i in 0..n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
    }

    assert!(
        !per_step.is_empty(),
        "{label}: no measured steps — the ramp produced no more than WARMUP_STEPS"
    );
    let n = per_step.len();
    let mean = per_step.iter().sum::<f64>() / n as f64;
    let mut sorted = per_step.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("no NaN timings"));
    let p50 = sorted[n / 2];
    // A single machine-parseable line per fixture: the comparison across arms
    // is done by diffing these, so they must not need re-typing by hand.
    // ⚠ LEADING NEWLINE, and it is not cosmetic. With `--nocapture` cargo writes
    // `test <name> ... ` with NO trailing newline, so the FIRST println of a run
    // is appended to that line and a `^R0AB`-anchored filter silently drops it.
    // That cost a whole fixture's data on the first run of this harness.
    println!(
        "\nR0AB\t{label}\tfree_dof={free}\tsteps={n}\tmean_ms={mean:.2}\tp50_ms={p50:.2}\tmin_ms={:.2}\tmax_ms={:.2}\titers_per_step={:.2}",
        sorted[0],
        sorted[n - 1],
        iters as f64 / n as f64
    );
}

#[test]
#[ignore = "measurement, minutes long — run explicitly"]
fn r0_ab_wall_time() {
    for (a_over_cell, label) in [(2.0, "IPC_5202"), (3.0, "IPC_18750")] {
        measure(a_over_cell, label);
    }
}

// ── the positive control for this harness ──────────────────────────────────

/// `cantilever 80×8`, the one fixture the recon published on ITS OWN box on
/// BOTH sides of R0: §2a read `11 272 ms` at `b0f4aa21` (pre-R0) and reads
/// `4 452 ms` today (post-R0), i.e. a published R0 credit of **2.53×**.
///
/// That makes it a positive control for this A/B *method*: the credit is a
/// ratio measured within one box, so it should transfer even though the two
/// boxes' absolute times do not. If this harness reproduces ~2.53× here, its
/// IPC credits can be read; if it does not, they cannot — the same standard
/// `tests/phase_shares.rs` applies to the phase-share instrument.
fn cantilever(nx: usize, n_cross: usize) -> (CpuTet4NHSolver<HandBuiltTetMesh>, Vec<f64>, usize) {
    const MU: f64 = 3.5e5;
    const LAMBDA: f64 = 1.4e6;
    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh =
        HandBuiltTetMesh::cantilever_bilayer_beam(nx, n_cross, n_cross, 0.20, 0.02, 0.02, &field);
    let n_dof = 3 * mesh.n_vertices();
    let mut x0 = vec![0.0; n_dof];
    for (c, p) in x0.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.x.abs() < 1e-9);
    let free = n_dof - 3 * pins.len();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.gravity_z = -9.81;
    cfg.max_newton_iter = NEWTON_CAP;
    (
        CpuNewtonSolver::new(
            Tet4,
            mesh,
            NullContact,
            cfg,
            BoundaryConditions::new(pins, Vec::new()),
        ),
        x0,
        free,
    )
}

#[test]
#[ignore = "measurement, minutes long — run explicitly"]
fn r0_ab_cantilever_control() {
    let (solver, x0, free) = cantilever(80, 8);
    let n = x0.len();
    let theta = Tensor::from_slice(&[], &[0]);
    let (mut x, mut v) = (x0, vec![0.0; n]);
    let mut per_step: Vec<f64> = Vec::new();
    let mut iters = 0usize;
    for k in 0..12 {
        let t0 = Instant::now();
        let step = solver.replay_step(
            &Tensor::from_slice(&x, &[n]),
            &Tensor::from_slice(&v, &[n]),
            &theta,
            DT,
        );
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        if k >= WARMUP_STEPS {
            per_step.push(ms);
            iters += step.iter_count;
        }
        for i in 0..n {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
    }
    let cnt = per_step.len();
    assert!(cnt > 0, "cantilever control: no measured steps");
    let mean = per_step.iter().sum::<f64>() / cnt as f64;
    let mut sorted = per_step.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("no NaN timings"));
    println!(
        "\nR0AB\tCANTILEVER_80x8\tfree_dof={free}\tsteps={cnt}\tmean_ms={mean:.2}\tp50_ms={:.2}\tmin_ms={:.2}\tmax_ms={:.2}\titers_per_step={:.2}",
        sorted[cnt / 2],
        sorted[0],
        sorted[cnt - 1],
        iters as f64 / cnt as f64
    );
}
