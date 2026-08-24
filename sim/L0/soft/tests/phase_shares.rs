//! Re-measure recon §2d's phase shares — and specifically its two `pre-R0` rows.
//!
//! ## What is stale and why it blocks R3
//!
//! §2d's table is post-R0 for every fixture **except the two IPC contact rows**,
//! which are labelled `pre-R0` and were never re-measured. §2a's `771 ms/step` at
//! IPC 18 750 — the source of the `46.2×` frame-budget gap — is pre-R0 too.
//!
//! R3's requirement is `gap ÷ R0 ÷ R1 ÷ predictor`, and R0's credit is the only
//! factor *inferred* rather than measured on that fixture: it comes from an
//! Amdahl bound over the pre-R0 assembly share. The inference forks the answer —
//! best-case R0 gives R3 5.8–8.2×, measured-R0 gives 6.5–10.1× against a 10×
//! kill floor — so it decides whether R3 can clear its own gate. This replaces
//! the inference with a measurement.
//!
//! ## Why single-arm
//!
//! `tests/predictor_spike.rs` runs four solvers in lockstep and measured IPC
//! 18 750 at **894.95 ms** p50 against §2a's pre-R0 **771 ms** — 1.16× SLOWER
//! after an optimisation that should have made it faster. The lockstep harness
//! (four live solvers, four factorizations resident) is the prime suspect, so
//! this runs **exactly one solver at a time** and holds nothing else.
//!
//! ## ★ The positive control is not optional
//!
//! `cantilever 80×8`'s §2d row is ALREADY post-R0. If this instrument cannot
//! reproduce a row that is known-good, its IPC numbers mean nothing — a wrong
//! reading there is indistinguishable from a real finding. The cantilever case
//! runs first and prints §2d's published row beside the measured one.
//!
//! ## Running it
//!
//! ⚠ Requires the feature; without it every slot reads zero and a table of
//! zeros looks like a successful run. The harness detects that and fails.
//!
//! ```text
//! cargo test --release -p sim-soft --features phase-timing \
//!   --test phase_shares -- --ignored --nocapture --test-threads=1
//! ```

#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]

mod refbox;

use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::profile::{self, Phase};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, IpcRigidContact,
    IpcRigidContactSolver, MaterialField, Mesh, NullContact, Solver, SolverConfig, SphereSdf, Tet4,
    TranslatedSdf, Vec3, pick_vertices_by_predicate,
};

const DT: f64 = 1.0 / 60.0;
const DENSITY: f64 = 1030.0;
const NEWTON_CAP: usize = 200;
/// Steps to discard before measuring. The first step of a fresh solver pays
/// one-off allocation and first-touch page faults that no later step repeats;
/// folding those into a per-step mean overstates every phase unevenly.
const WARMUP_STEPS: usize = 2;

/// One fixture's measurement.
struct Measured {
    label: String,
    free_dof: usize,
    steps: usize,
    total_ms: f64,
    iters: usize,
    phases: profile::Phases,
}

fn report(m: &Measured, published: Option<[f64; 5]>) {
    // Without this, `snapshot()` would still hold the PREVIOUS fixture's counters
    // (reset never fired) and every per-step figure would divide by zero — a
    // plausible-looking table attributed to the wrong fixture.
    assert!(
        m.steps > 0,
        "{}: zero measured steps — the fixture produced no more than WARMUP_STEPS \
         ({WARMUP_STEPS}), so the timing slots were never reset and hold the \
         previous fixture's data",
        m.label,
    );
    println!(
        "\n╔═ {} — {} free DOF, {} measured steps",
        m.label, m.free_dof, m.steps
    );
    println!(
        "║ {:.2} ms/step wall · {:.2} Newton iters/step",
        m.total_ms / m.steps as f64,
        m.iters as f64 / m.steps as f64
    );
    println!("║");
    println!(
        "║ {:<16} {:>10} {:>9} {:>11} {:>9}",
        "phase", "ms total", "ms/step", "share", "calls"
    );
    // ⚠ Shares of WALL step time, matching §2d's stated denominator ("phase
    // shares of total step time"). `Phases::share` divides by instrumented work
    // instead, which sums to 100 % BY CONSTRUCTION and is therefore a different
    // quantity from the rows in that table — subtracting one from the other, as
    // an R0-credit derivation does, silently compares two denominators.
    for p in Phase::ALL {
        println!(
            "║ {:<16} {:>10.2} {:>9.3} {:>10.1} % {:>9}",
            p.label(),
            m.phases.millis(p),
            m.phases.millis(p) / m.steps as f64,
            100.0 * m.phases.millis(p) / m.total_ms,
            m.phases.calls(p),
        );
    }
    println!(
        "║ instrumented / wall = {:.1} % — the remainder is residual evaluation, line \
         search and allocation",
        100.0
            * Phase::ALL
                .iter()
                .filter(|p| !matches!(p, Phase::Contact))
                .map(|p| m.phases.millis(*p))
                .sum::<f64>()
            / m.total_ms
    );
    if let Some(pub_row) = published {
        println!("║");
        println!("║ ★ POSITIVE CONTROL — §2d's published row for this fixture is post-R0:");
        println!(
            "║ {:<16} {:>12} {:>12} {:>10}",
            "phase", "published", "measured", "ratio"
        );
        for (p, want) in Phase::ALL.iter().zip(pub_row) {
            let got = 100.0 * m.phases.share(*p);
            let ratio = if want > 0.0 { got / want } else { f64::NAN };
            println!(
                "║ {:<16} {:>11.1} % {:>11.1} % {:>9.2}×",
                p.label(),
                want,
                got,
                ratio
            );
        }
        println!(
            "║ ⚠ If these disagree materially, the INSTRUMENT is suspect and nothing \
             below it can be read."
        );
        // ★ ASSERTED, not merely printed. An earlier version only printed these
        // ratios, so a run where the instrument was 3× wrong on the known-good
        // row still exited 0 and the doc quoted the printout as validation — a
        // control that cannot fail is not a control.
        for (p, want) in Phase::ALL.iter().zip(pub_row) {
            if want <= 0.0 {
                continue;
            }
            let got = 100.0 * m.phases.millis(*p) / m.total_ms;
            let ratio = got / want;
            assert!(
                (0.80..=1.25).contains(&ratio),
                "POSITIVE CONTROL FAILED on {}: published {want:.1} %, measured \
                 {got:.1} % ({ratio:.2}×). The instrument does not reproduce a row \
                 that is already post-R0, so nothing it reports elsewhere can be \
                 read.",
                p.label(),
            );
        }
    }
    println!("╚═\n");
}

// ── fixtures ───────────────────────────────────────────────────────────────

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

const RADIUS: f64 = 1.0e-2;
const DELTA: f64 = 5.0e-4;
const KAPPA: f64 = 1.0e4;
const BAND_FRAC: f64 = 0.05;
const LAYER_MU: f64 = 1.0e5;
const CHI: f64 = 0.35;

/// `bonded_layer_indentation::dims_for`, reproduced so the DOF counts line up
/// with §2a/§2d rather than approximately matching them.
fn dims_for(a_over_cell: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / a_over_cell;
    let h = a / CHI;
    let lateral = 8.0 * a;
    let even = |n: usize| if n.is_multiple_of(2) { n } else { n + 1 };
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

/// The IPC indentation ramp, run single-arm with phase timing.
fn measure_ipc(a_over_cell: f64, label: &str) -> Measured {
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
    let (mut total_ms, mut iters, mut measured) = (0.0_f64, 0usize, 0usize);

    for k in 0..n_steps {
        let z = (z_start - (k + 1) as f64 * inc).max(z_end);
        solver.replace_contact(IpcRigidContact::with_params(
            vec![indenter(lateral, lateral, z)],
            KAPPA,
            d_hat,
        ));
        // Reset AFTER warmup so the discarded steps contribute nothing.
        if k == WARMUP_STEPS {
            profile::reset();
        }
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
            total_ms += ms;
            iters += step.iter_count;
            measured += 1;
        }
        for i in 0..n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
    }
    Measured {
        label: label.to_string(),
        free_dof: free,
        steps: measured,
        total_ms,
        iters,
        phases: profile::snapshot(),
    }
}

/// The instrument must be ON. Without the feature every slot reads zero and a
/// table of zeros is indistinguishable from a clean run — the exact shape of a
/// gate that cannot fail for the reason it claims.
fn require_timing_enabled() {
    let (solver, x0, _) = cantilever(8, 2);
    profile::reset();
    let n = x0.len();
    // `drop`, not `let _`: the result is `#[must_use]` and irrelevant here — the
    // point is only that a real solve ran, so the slots have something to read.
    drop(solver.try_replay_step(
        &Tensor::from_slice(&x0, &[n]),
        &Tensor::from_slice(&vec![0.0; n], &[n]),
        &Tensor::from_slice(&[], &[0]),
        DT,
    ));
    let p = profile::snapshot();
    assert!(
        p.total_nanos() > 0,
        "every timing slot read ZERO after a real solve — the `phase-timing` \
         feature is OFF. Re-run with `--features phase-timing`; without it this \
         harness prints a table of zeros that looks like a successful run.",
    );
}

#[test]
#[ignore = "phase-share instrument — run explicitly with --features phase-timing"]
fn remeasure_2d_phase_shares() {
    refbox::require_quiet_box();
    require_timing_enabled();

    // ── positive control: §2d's cantilever row is ALREADY post-R0 ──
    {
        let (solver, x0, free) = cantilever(80, 8);
        let n = x0.len();
        let theta = Tensor::from_slice(&[], &[0]);
        let (mut x, mut v) = (x0, vec![0.0; n]);
        let (mut total_ms, mut iters, mut measured) = (0.0_f64, 0usize, 0usize);
        for k in 0..12 {
            if k == WARMUP_STEPS {
                profile::reset();
            }
            let t0 = Instant::now();
            let step = solver.replay_step(
                &Tensor::from_slice(&x, &[n]),
                &Tensor::from_slice(&v, &[n]),
                &theta,
                DT,
            );
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            if k >= WARMUP_STEPS {
                total_ms += ms;
                iters += step.iter_count;
                measured += 1;
            }
            for i in 0..n {
                v[i] = (step.x_final[i] - x[i]) / DT;
            }
            x = step.x_final;
        }
        let m = Measured {
            label: "cantilever 80×8 (POSITIVE CONTROL, §2d row is post-R0)".to_string(),
            free_dof: free,
            steps: measured,
            total_ms,
            iters,
            phases: profile::snapshot(),
        };
        // §2d's published row: asmF, asmK, numF, tri, contact.
        report(&m, Some([4.7, 18.6, 72.3, 3.7, 0.0]));
    }

    // ── the two stale rows ──
    for (a_over_cell, label) in [
        (2.0, "IPC 5 202 (§2d row is PRE-R0)"),
        (3.0, "IPC 18 750 (§2d row is PRE-R0)"),
    ] {
        let m = measure_ipc(a_over_cell, label);
        report(&m, None);
    }
}
