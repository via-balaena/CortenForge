//! Does the REDUCED path run with contact at all — and does it interpenetrate?
//!
//! ## Why this exists
//!
//! The ladder defines R1 as "no contact", and every reduced measurement to date
//! is on a contact-free cantilever. But R3's requirement lives on the IPC
//! indentation fixture, and by `C/R = B/I` (recon §2j) the only quantity that
//! decides a rung is the frame's IRREDUCIBLE time — of which `contact` is the
//! largest term. So the deciding number has never been measured on a scene that
//! has any contact in it.
//!
//! Timing that scene needs a solver that produces the right answer there first.
//! **This harness is that producer check**; the timing follows separately in
//! `tests/reduced_phase_shares.rs`.
//!
//! ```text
//! cargo test --release -p sim-soft --test reduced_contact -- --ignored --nocapture
//! ```
//!
//! ## No new physics is involved
//!
//! `assemble_global_int_force` and `assemble_free_hessian_triplets` both fold the
//! contact gradient and Hessian in, and the reduced solver projects whatever they
//! emit. It is generic over `C: ContactModel` today. Nothing here needed writing
//! — which is exactly why the question is open: it *compiles*, so no one has had
//! to think about whether it *works*.
//!
//! ## ★★ The specific failure mode this is built to catch
//!
//! `IpcRigidContact::barrier` clamps its argument — `let d = sd.max(d_hat * 1e-6)`
//! (`contact/ipc.rs`). **Penetration is therefore finite-energy, not infinite.**
//! A configuration that has driven vertices through the collider still has a
//! finite residual and can converge.
//!
//! Now compose that with the reduced line search, which backtracks on `‖Φᵀr‖` —
//! the quantity the reduced solve descends. A barrier spike is *localised*: a few
//! vertices under the sphere. Localised is precisely what a global POD basis
//! represents worst, so the spike can be largely orthogonal to `Φ` and project
//! small. **The reduced Armijo can then accept a step the full-order Armijo would
//! have rejected**, and the step converges, and nothing complains.
//!
//! ⇒ Convergence is NOT evidence of correctness here. Non-penetration has to be
//! checked directly, and it is checked with geometry computed in this file
//! (`min_signed_distance`) rather than through `active_pairs`, so a defect in the
//! contact model's own pair search cannot make a penetrating configuration look
//! clean.
//!
//! ## Pre-registration (written before the first run)
//!
//! 1. The reduced path converges at `r = 40`, because the indentation ramp is a
//!    near-1-parameter family and POD should represent it in far fewer than 40
//!    modes.
//! 2. **Iteration count is the number that matters.** From the published rows,
//!    contact costs `1.43 ms` per Newton iteration at 18 750 free DOF
//!    (`927.7 ms × 1.0 % ÷ 6.51`), all of it irreducible. Against a `16.7 ms`
//!    budget that gives R3 a `1.8×` margin at the full-order iteration count and
//!    **kills it above ~12 iterations/step**. So an iteration count that holds is
//!    a pass for R3; one that inflates is the whole answer.
//! 3. The genuinely uncertain one: **whether `min_sd` stays positive.** No
//!    prediction — the clamp plus the projected Armijo is a real mechanism, and
//!    the `r` ladder is here to tell the two explanations apart. If low `r`
//!    penetrates and high `r` does not, it is subspace capacity. If penetration
//!    is flat in `r`, it is the projected line search, which no rank fixes.
//!
//! ## Knobs
//!
//! Swept: `r ∈ {10, 20, 40, 60}` × `InitialGuess ∈ {PreviousState, Inertial}`.
//! Both are cheap because the single full-order oracle trajectory dominates the
//! run. `Inertial` is in because it is the settled predictor AND because it
//! changes iteration count, which is the deciding quantity above.
//!
//! Held: Tet4, `a/cell = 2` (5 202 free DOF), frictionless, `gravity_z = 0`,
//! `dt = 1/60`, in-sample basis (trained on the trajectory it is tested on).
//!
//! ⚠ **The in-sample basis is deliberate and it limits what this can claim.** It
//! isolates "does the algebra work against a barrier" from "does POD generalise
//! across contact configurations". The second question — a laterally *moving*
//! contact patch, which is the classic advection-like POD failure — is an R1
//! question, is not asked here, and is not answered by anything below.
//!
//! ## Measured — first run, 2026-08-24, `a/cell = 2` (5 202 free DOF)
//!
//! **The reduced path runs with IPC contact, and prediction 3's failure mode did
//! not occur.** Every arm completed all 71 ramp steps and none penetrated —
//! including `r = 2`, whose displacement field is `48 %` wrong.
//!
//! | `r` | iters/step (prev / inertial) | rel-L2 | `gap_dev` |
//! |---:|---|---:|---:|
//! | ORACLE | 6.10 / — | — | — |
//! | 2 | 6.04 / 3.04 | 4.83e-1 | 2.04e-1 d̂ |
//! | 4 | 6.25 / 3.28 | 8.42e-2 | 3.25e-2 d̂ |
//! | 10 | 6.42 / 3.48 | 5.12e-3 | 1.35e-6 d̂ |
//! | 20 | 6.49 / 3.51 | 6.52e-5 | 8.56e-10 d̂ |
//! | 36 (full rank) | 6.44 / 3.55 | 2.77e-7 | 6.94e-14 d̂ |
//!
//! - ★ **The equilibrium gap converges in `r` far faster than the displacement
//!   field does**, and increasingly so: the two are within `1.7×` of each other
//!   at `r = 2`, but the gap is `~2 700×` better determined at `r = 10` and
//!   `~54 000×` at `r = 20`. Once the subspace is adequate at all, the barrier
//!   pins the contact state much harder than the field around it.
//! - ★ **The mechanism prediction 3 named is real and measured — it just has no
//!   consequence here.** At `r = 10` the reduced solve converges on
//!   `‖Φᵀr‖ < 1e-10` while `‖r_free‖` is `1.49e-4`: the projection hides six
//!   orders of magnitude of residual, exactly as argued. It does not become
//!   penetration, because the barrier is stiff enough to pin the gap anyway.
//! - ⚠ **Iteration count is flat in `r`** — `6.04–6.49` across a subspace range
//!   over which the answer goes from `48 %` wrong to `2.8e-7`. Newton's behaviour
//!   is a COST signal and not a quality signal; a bad basis is invisible to it.
//! - **Prediction 2 held.** `Inertial` is `1.75×` fewer iterations than
//!   `PreviousState` (`3.55` vs `6.44`), and both are far under the `~12` that
//!   would kill R3. The oracle's `6.10` reproduces `r0_ab`'s published `6.12` for
//!   this fixture to `0.3 %`, which is the cross-check that this is that fixture.
//!
//! ⚠ What this does NOT establish: anything at 18 750 free DOF, out-of-sample,
//! with friction, or with a body load (recon §2f killed `InertialWithLoad` on
//! contact-plus-load, and everything here runs `gravity_z = 0`). And it is not a
//! timing run — `I` still has to be measured.

#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::cast_precision_loss,
    // `dims_for` and `n_steps` round a positive geometric ratio to a mesh
    // resolution or a step count — the same conversions, on the same fixture,
    // that `predictor_spike.rs` allows for the same reason.
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines,
    // `r` is the reduced rank, `h` the layer thickness, `a` the contact patch —
    // the fixture's own symbols, matching `predictor_spike.rs`.
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
    BoundaryConditions, HandBuiltTetMesh, InitialGuess, IpcRigidContact, IpcRigidContactSolver,
    MaterialField, Mesh, Solver, SolverConfig, SolverFailure, SphereSdf, Tet4, TranslatedSdf, Vec3,
    pick_vertices_by_predicate,
};

// ── the fixture, reproduced from `predictor_spike.rs`'s IPC block ──────────
//
// Reproduced rather than shared for the reason its own `sample` is: these are
// instruments, and an instrument that drifts because a helper it borrows changed
// is worse than a duplicated constant. ⚠ This is the THIRD copy (`r0_ab.rs` has
// the second). A fourth should hoist instead.

/// Sphere radius (10 mm).
const RADIUS: f64 = 1.0e-2;
/// Final indentation depth.
const DELTA: f64 = 5.0e-4;
/// IPC barrier stiffness.
const KAPPA: f64 = 1.0e4;
/// Barrier band as a fraction of `δ`.
const BAND_FRAC: f64 = 0.05;
/// Layer material (μ, λ) — `ν = 0.4`.
const LAYER_MU: f64 = 1.0e5;
const LAYER_LAMBDA: f64 = 4.0 * LAYER_MU;
/// Lateral extent as a multiple of the contact patch `a`.
const LATERAL_FACTOR: f64 = 8.0;
/// Layer aspect `χ = a/h`.
const CHI: f64 = 0.35;
const DT: f64 = 1.0 / 60.0;
const DENSITY: f64 = 1030.0;
const MAX_NEWTON_ITER: usize = 120;

/// `a/cell = 2` — 5 202 free DOF, the recon's small IPC row and R1.1's DOF count.
const A_OVER_CELL: f64 = 2.0;

/// Reduced ranks. Spans well below and well above the point where the subspace
/// should contain the whole ramp, so a penetration that is flat in `r` is
/// distinguishable from one the rank cures.
///
/// ★ The bottom two rungs are the two-sided half. They are far too crude to be
/// a serious operating point, and they exist so the table SHOWS what a subspace
/// that cannot hold the trajectory does to the contact equilibrium — otherwise
/// [`MAX_GAP_DEV_BANDS`] below is a threshold nothing in the run ever approaches
/// and the gate is green for a reason it never demonstrates.
const R_LADDER: [usize; 6] = [2, 4, 10, 20, 40, 60];
/// The rank the pass/fail assertions are stated at — R1.1's operating point.
const R_REFERENCE: usize = 40;

/// The always-on gate's resolution. One step coarser than the instrument, which
/// is what makes it cheap enough to run unconditionally; the ramp itself is
/// resolution-independent, so it exercises the same barrier over the same
/// `71` poses.
const GATE_A_OVER_CELL: f64 = 1.0;
/// Requested rank for the gate. Above the coarse ramp's numerical rank on
/// purpose — [`ladder`] reports what was actually kept, and the gate wants the
/// richest subspace the trajectory supports.
const GATE_RANK: usize = 40;

/// How far a reduced arm may move the converged contact gap, in units of the
/// barrier band `d̂`.
///
/// ★ PILOTED, not chosen: measured `1.345e-6` at `r = 10`, `8.6e-10` at `r = 20`,
/// `6.9e-14` at the trajectory's full rank, and `2.8e-13` on the gate's coarse
/// mesh. The threshold is the CRUDEST rung's value rounded up — "a reduction may
/// not disturb the contact equilibrium more than a 10-mode basis does" — which
/// anchors it to something measured rather than to a round number, and keeps ~7
/// orders of headroom over floating-point drift at the ranks that matter.
///
/// ⚠ **Its sensitivity to a given force error is NOT characterised, and an
/// earlier version of this comment claimed it was.** `gap_dev` falls
/// superlinearly in basis quality — `gap_dev / relL2` is `0.39` at `r = 4`,
/// `2.6e-4` at `r = 10`, `1.3e-5` at `r = 20` — so extrapolating a
/// "trips at ~1 % force error" figure from the one rung nearest the threshold
/// crosses four orders of curvature. What the ladder DEMONSTRATES it catches is
/// the `r ≤ 4` class, four to five orders over the limit.
///
/// ⚠ This is the assertion `min_sd > 0` cannot make. That one only asks which
/// SIDE of the collider the material ended up on; R3's hyper-reduced assembly
/// changes the forces the barrier is balanced against, and the failure mode
/// there is a shifted equilibrium, not a tunnelled one.
const MAX_GAP_DEV_BANDS: f64 = 2.0e-6;

/// The timing sweep. `a/cell = 3` is 18 750 free DOF — the row §2f's requirement
/// is stated on, and the source of the `53.2×` full-order gap. `a/cell = 2` is
/// 5 202, and it is here for ONE reason: the reduced path's iteration count has
/// to be compared across sizes IN THE SAME WINDOW.
///
/// ⚠ The first version of this harness did not do that, and the review pass
/// caught it. It compared 5 202's whole-ramp average (`6.44`) against 18 750's
/// last-8-steps average (`9.00`) and read the difference as a size effect —
/// but the last steps are the DEEPEST indentation, where the count would be
/// highest anyway. Two variables moved at once. Both rows here now use the same
/// trailing window, so the difference is size.
const TIMING_A_OVER_CELL: [f64; 2] = [2.0, 3.0];
/// Requested rank for the timing fixture; the ramp's own numerical rank caps it.
const TIMING_RANK: usize = 40;
/// How many of the ramp's trailing steps carry the timing measurement.
///
/// ⚠ THE LAST ones, and that choice is load-bearing. The indenter starts above
/// the barrier band and descends, so a frame early in the ramp has no active
/// pairs at all — timing those would report `I` for a scene with no contact in
/// it, which is precisely the thing this fixture exists to stop doing. The
/// deepest steps carry the largest contact patch, and that is the workload the
/// frame budget is claimed against.
///
/// ⚠ Whether this leaves any warmup is checked at RUNTIME, against the ramp's
/// actual length. A `const` assertion here could only compare it to a hardcoded
/// step count, and the ramp's length is derived from `d̂` and `δ` — so the
/// constant would go stale in silence the moment either moved, which is the
/// shape of a guard that cannot fail for the reason it claims.
const TIMING_STEPS: usize = 8;

const GUESSES: [(&str, InitialGuess); 2] = [
    ("prev", InitialGuess::PreviousState),
    ("inertial", InitialGuess::Inertial),
];

/// `(n_lat, nz, lateral, h)` for a given `a/cell`, holding the contact patch
/// fixed. Reproduces `bonded_layer_indentation::dims_for` exactly.
fn dims_for(a_over_cell: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / a_over_cell;
    let h = a / CHI;
    let lateral = LATERAL_FACTOR * a;
    let even = |n: usize| if n.is_multiple_of(2) { n } else { n + 1 };
    let n_lat = even(((lateral / cell).round() as usize).max(2));
    let nz = even(((h / cell).round() as usize).max(2));
    (n_lat, nz, lateral, h)
}

fn indenter(lx: f64, ly: f64, z_center: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(lx / 2.0, ly / 2.0, z_center),
    }
}

#[derive(Clone, Copy)]
struct Scene {
    a_over_cell: f64,
    n_lat: usize,
    nz: usize,
    lateral: f64,
    h: f64,
    d_hat: f64,
}

impl Scene {
    fn new(a_over_cell: f64) -> Self {
        let (n_lat, nz, lateral, h) = dims_for(a_over_cell);
        Self {
            a_over_cell,
            n_lat,
            nz,
            lateral,
            h,
            d_hat: BAND_FRAC * DELTA,
        }
    }

    fn mesh(self) -> HandBuiltTetMesh {
        let field = MaterialField::uniform(LAYER_MU, LAYER_LAMBDA);
        HandBuiltTetMesh::cantilever_bilayer_beam(
            self.n_lat,
            self.n_lat,
            self.nz,
            self.lateral,
            self.lateral,
            self.h,
            &field,
        )
    }

    fn solver(self, guess: InitialGuess) -> IpcRigidContactSolver<HandBuiltTetMesh> {
        let mesh = self.mesh();
        let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-9);
        assert!(!pins.is_empty(), "bonded bottom face has no vertices");
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = DT;
        cfg.density = DENSITY;
        cfg.max_newton_iter = MAX_NEWTON_ITER;
        cfg.initial_guess = guess;
        cfg.gravity_z = 0.0;
        sim_soft::CpuNewtonSolver::new(
            Tet4,
            mesh,
            IpcRigidContact::with_params(
                vec![indenter(self.lateral, self.lateral, self.z_start())],
                KAPPA,
                self.d_hat,
            ),
            cfg,
            BoundaryConditions::new(pins, Vec::new()),
        )
    }

    /// South pole starts a full `1.2·d̂` above the top face (barrier inactive).
    fn z_start(self) -> f64 {
        self.h + RADIUS + 1.2 * self.d_hat
    }

    fn z_end(self) -> f64 {
        self.h + RADIUS - DELTA
    }

    fn increment(self) -> f64 {
        0.3 * self.d_hat
    }

    fn n_steps(self) -> usize {
        ((self.z_start() - self.z_end()) / self.increment()).ceil() as usize
    }

    /// Sphere-centre height at step `k`. Both arms call THIS, so "same pose
    /// schedule" is true by construction rather than by two edits agreeing.
    fn z_at(self, k: usize) -> f64 {
        (self.z_start() - (k + 1) as f64 * self.increment()).max(self.z_end())
    }

    fn centre_at(self, k: usize) -> Vec3 {
        Vec3::new(self.lateral / 2.0, self.lateral / 2.0, self.z_at(k))
    }

    fn contact_at(self, k: usize) -> IpcRigidContact {
        IpcRigidContact::with_params(
            vec![indenter(self.lateral, self.lateral, self.z_at(k))],
            KAPPA,
            self.d_hat,
        )
    }
}

/// Minimum signed distance from any mesh vertex to the sphere.
///
/// ★ Computed from the sphere's geometry HERE, not read out of the contact
/// model. The property under test is "the solver did not drive material through
/// the collider", and a check routed through `active_pairs` would share a
/// failure mode with the thing it is checking — the same reason the oracle in
/// `reduced/tests.rs` is written from the other matrix layout.
fn min_signed_distance(x: &[f64], centre: Vec3) -> f64 {
    x.chunks_exact(3)
        .map(|c| (Vec3::new(c[0], c[1], c[2]) - centre).norm() - RADIUS)
        .fold(f64::INFINITY, f64::min)
}

fn rest_positions(mesh: &HandBuiltTetMesh) -> Vec<f64> {
    let mut x = vec![0.0_f64; 3 * mesh.n_vertices()];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

/// The failure's identity and the geometry AT the failure — enough to tell
/// "diverged before touching" from "tunnelled, then diverged". `SolverFailure`
/// derives only `Debug`, and its variants carry `x_partial`, so `{e:?}` would
/// print the entire configuration.
fn describe(e: &SolverFailure, scene: Scene, k: usize) -> String {
    let at = |x: &[f64]| min_signed_distance(x, scene.centre_at(k));
    match e {
        SolverFailure::ArmijoStall {
            x_partial,
            last_iter,
            last_r_norm,
        } => format!(
            "ArmijoStall at iter {last_iter}, ‖r‖ = {last_r_norm:.3e}, min_sd = {:.3e}",
            at(x_partial)
        ),
        SolverFailure::NewtonIterCap {
            x_partial,
            max_iter,
            last_r_norm,
        } => format!(
            "NewtonIterCap at {max_iter}, ‖r‖ = {last_r_norm:.3e}, min_sd = {:.3e}",
            at(x_partial)
        ),
        SolverFailure::DoublyFailedFactor {
            x_partial, context, ..
        } => format!(
            "DoublyFailedFactor ({context}), min_sd = {:.3e}",
            at(x_partial)
        ),
        SolverFailure::ValidityViolation { tet_id, message } => {
            format!("ValidityViolation at tet {tet_id}: {message}")
        }
    }
}

/// The timed sub-window of a trajectory. Kept apart from the trajectory-level
/// tallies because the correctness rows want every step and the timing rows want
/// only the deepest ones, where the contact patch is largest.
#[derive(Clone, Copy, Default)]
struct Measured {
    steps: usize,
    iters: usize,
    wall_ms: f64,
}

impl Measured {
    fn record(&mut self, ms: f64, iters: usize) {
        self.steps += 1;
        self.iters += iters;
        self.wall_ms += ms;
    }
}

/// One arm's trajectory-level summary. `min_sd` is the worst over all steps —
/// the closest any vertex came to the collider, negative meaning it went through.
struct Arm {
    label: String,
    completed: usize,
    iters: usize,
    min_sd: f64,
    /// Rel-L2 free-DOF displacement error against the oracle, worst over steps.
    max_rel_err: f64,
    /// `‖r_free‖` at the last converged step — the FULL residual, which the
    /// reduced solve does not descend and does not gate on.
    last_full_r: f64,
    /// `|min_sd − oracle's min_sd|` in units of the barrier band `d̂`. The sharp
    /// form of the non-penetration check: `min_sd > 0` only asks which side of
    /// the collider the material is on, while this asks whether the reduced arm
    /// reaches the SAME equilibrium gap. A hyper-reduced assembly that got the
    /// internal forces slightly wrong would shift this while staying positive.
    gap_dev: f64,
    /// The timed window, empty unless the arm was run with a `measure_from`.
    measured: Measured,
    failure: Option<String>,
}

impl Arm {
    fn print(&self, n_steps: usize, d_hat: f64) {
        let iters_per = if self.completed == 0 {
            0.0
        } else {
            self.iters as f64 / self.completed as f64
        };
        println!(
            "RC\t{:<18}\tsteps={:>3}/{n_steps}\titers/step={iters_per:>5.2}\t\
             min_sd={:>16.9e}\t(band d̂={d_hat:.2e})\tmax_relL2={:>9.3e}\t‖r‖full={:>9.3e}\tgap_dev={:>9.3e} d̂\t{}",
            self.label,
            self.completed,
            self.min_sd,
            self.max_rel_err,
            self.last_full_r,
            self.gap_dev,
            self.failure.as_deref().unwrap_or("ok"),
        );
    }
}

/// Free-DOF displacement from rest.
fn free_disp(x: &[f64], x_rest: &[f64], fd: &[usize]) -> Vec<f64> {
    fd.iter().map(|&i| x[i] - x_rest[i]).collect()
}

fn rel_l2(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = a
        .iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt();
    let den: f64 = b.iter().map(|y| y * y).sum::<f64>().sqrt();
    if den > 0.0 { num / den } else { f64::NAN }
}

/// The full-order oracle: one pass down the ramp, recording the converged
/// configuration at every step. Doubles as the snapshot source — which is what
/// makes every basis below IN-SAMPLE, see the module docs.
struct Oracle {
    x: Vec<Vec<f64>>,
    arm: Arm,
}

/// Step index at which to `profile::reset()` and start accumulating wall time.
/// [`NO_TIMING`] runs the trajectory without touching the profiler at all, which
/// is what the correctness tests want — they must not disturb a measurement, and
/// they must not pay for one.
type MeasureFrom = usize;
const NO_TIMING: MeasureFrom = usize::MAX;

fn run_oracle(scene: Scene, x_rest: &[f64], measure_from: MeasureFrom) -> Oracle {
    let mut solver = scene.solver(InitialGuess::PreviousState);
    let n_dof = x_rest.len();
    let theta = Tensor::from_slice(&[], &[0]);

    let mut x = x_rest.to_vec();
    let mut v = vec![0.0; n_dof];
    let mut out = Vec::with_capacity(scene.n_steps());
    let (mut iters, mut min_sd, mut last_full_r) = (0usize, f64::INFINITY, f64::NAN);
    let mut m = Measured::default();
    let mut failure = None;

    for k in 0..scene.n_steps() {
        solver.replace_contact(scene.contact_at(k));
        if k == measure_from {
            profile::reset();
        }
        let t0 = Instant::now();
        let step = solver.try_replay_step(
            &Tensor::from_slice(&x, &[n_dof]),
            &Tensor::from_slice(&v, &[n_dof]),
            &theta,
            DT,
        );
        match step {
            Ok(s) => {
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                if k >= measure_from {
                    m.record(ms, s.iter_count);
                }
                for i in 0..n_dof {
                    v[i] = (s.x_final[i] - x[i]) / DT;
                }
                x = s.x_final;
                iters += s.iter_count;
                last_full_r = s.final_residual_norm;
                min_sd = min_sd.min(min_signed_distance(&x, scene.centre_at(k)));
                out.push(x.clone());
            }
            Err(e) => {
                failure = Some(describe(&e, scene, k));
                break;
            }
        }
    }

    let completed = out.len();
    Oracle {
        x: out,
        arm: Arm {
            label: "ORACLE full-order".to_string(),
            completed,
            iters,
            min_sd,
            max_rel_err: 0.0,
            last_full_r,
            gap_dev: 0.0,
            measured: m,
            failure,
        },
    }
}

/// What every reduced arm on one fixture shares: the rest configuration, the
/// free-DOF map, and the trajectory it is scored against. Grouped because they
/// are three views of ONE full-order run and passing them separately invites an
/// arm being scored against a different oracle than it was fitted to.
struct Ctx<'a> {
    x_rest: &'a [f64],
    fd: &'a [usize],
    oracle: &'a Oracle,
}

fn run_reduced(
    scene: Scene,
    basis: &PodBasis,
    guess: InitialGuess,
    label: String,
    ctx: &Ctx<'_>,
    measure_from: MeasureFrom,
) -> Arm {
    let Ctx { x_rest, fd, oracle } = *ctx;
    let mut solver = scene.solver(guess);
    let theta = Tensor::from_slice(&[], &[0]);
    let mut q = vec![0.0; basis.n_modes()];
    let mut qdot = vec![0.0; basis.n_modes()];
    let (mut iters, mut min_sd, mut max_rel_err, mut last_full_r) =
        (0usize, f64::INFINITY, 0.0_f64, f64::NAN);
    let mut m = Measured::default();
    let (mut completed, mut failure) = (0usize, None);

    for k in 0..scene.n_steps() {
        solver.replace_contact(scene.contact_at(k));
        let reduced = ReducedNewtonSolver::new(&solver, basis, x_rest);
        if k == measure_from {
            profile::reset();
        }
        let t0 = Instant::now();
        match reduced.step(&q, &qdot, &theta, DT) {
            Ok(s) => {
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                if k >= measure_from {
                    m.record(ms, s.iter_count);
                }
                let x = reduced.expand_to_full(&s.q);
                min_sd = min_sd.min(min_signed_distance(&x, scene.centre_at(k)));
                if let Some(ox) = oracle.x.get(k) {
                    max_rel_err = max_rel_err.max(rel_l2(
                        &free_disp(&x, x_rest, fd),
                        &free_disp(ox, x_rest, fd),
                    ));
                }
                iters += s.iter_count;
                last_full_r = s.full_residual_norm;
                completed += 1;
                q = s.q;
                qdot = s.qdot;
            }
            Err(e) => {
                failure = Some(describe(&e, scene, k));
                break;
            }
        }
    }

    Arm {
        label,
        completed,
        iters,
        min_sd,
        max_rel_err,
        last_full_r,
        gap_dev: (min_sd - oracle.arm.min_sd).abs() / scene.d_hat,
        measured: m,
        failure,
    }
}

/// One oracle trajectory, one basis per requested rank, both predictors on each.
/// The oracle dominates the cost, so the ladder is nearly free — and the SHAPE
/// across `r` is what distinguishes a subspace-capacity failure from a
/// projected-line-search one.
struct Ladder {
    oracle: Oracle,
    arms: Vec<(usize, &'static str, Arm)>,
}

impl Ladder {
    /// Every arm at the requested rank that failed a non-negotiable. Collected
    /// rather than asserted in place: the shape across `r` is the finding, and
    /// asserting inside the loop would discard every row past the first failure.
    fn faults(&self, at_rank: usize, oracle_min_sd: f64) -> Vec<String> {
        let mut out = Vec::new();
        for (r, name, arm) in &self.arms {
            if *r != at_rank {
                continue;
            }
            if let Some(f) = &arm.failure {
                out.push(format!("r={r} {name}: did not complete the ramp — {f}"));
            }
            if arm.min_sd <= 0.0 {
                out.push(format!(
                    "r={r} {name}: PENETRATED, min_sd = {:.3e} (oracle {oracle_min_sd:.3e})",
                    arm.min_sd,
                ));
            }
            if arm.gap_dev > MAX_GAP_DEV_BANDS {
                out.push(format!(
                    "r={r} {name}: contact equilibrium moved {:.3e} d̂ (limit \
                     {MAX_GAP_DEV_BANDS:.1e}) — it stayed outside the collider but not \
                     where the oracle puts it",
                    arm.gap_dev,
                ));
            }
        }
        out
    }
}

fn ladder(scene: Scene, ranks: &[usize]) -> Ladder {
    let probe = scene.mesh();
    let x_rest = rest_positions(&probe);
    let solver = scene.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let n_steps = scene.n_steps();

    println!(
        "\nRC\tfixture: IPC indentation a/cell={:.1}, {} free DOF, {} tets, \
         {n_steps} ramp steps, d̂={:.2e}, δ={DELTA:.1e}",
        scene.a_over_cell,
        fd.len(),
        probe.n_tets(),
        scene.d_hat,
    );

    // ── the oracle, and the snapshots every basis below is fitted to ──
    let oracle = run_oracle(scene, &x_rest, NO_TIMING);
    oracle.arm.print(n_steps, scene.d_hat);
    assert!(
        oracle.arm.failure.is_none(),
        "the FULL-ORDER arm did not survive its own ramp ({}) — nothing below is \
         interpretable until the fixture itself is fixed",
        oracle.arm.failure.as_deref().unwrap_or(""),
    );

    // ⚠ VACUITY GUARD. Every claim here is about behaviour against a barrier; if
    // the sphere never entered the band, all of them pass on a scene with no
    // contact in it. `d̂` is the band, so `min_sd < d̂` is "the barrier was live".
    assert!(
        oracle.arm.min_sd < scene.d_hat,
        "contact never engaged: closest approach {:.3e} never entered the band {:.3e}",
        oracle.arm.min_sd,
        scene.d_hat,
    );
    // ★ NEGATIVE CONTROL for every `min_sd` in the table. At the ramp's last pose
    // the sphere's south pole sits `δ` BELOW the undeformed top face, so a solver
    // that deformed nothing reads `-δ`. This is what stops a positive `min_sd`
    // being read as a property of the geometry: the rows sit ~20 band-widths from
    // the do-nothing value, and on the other side of zero.
    let do_nothing = min_signed_distance(&x_rest, scene.centre_at(n_steps - 1));
    println!(
        "RC\tNEGATIVE CONTROL — undeformed mesh at the final pose: min_sd = {do_nothing:.6e} \
         (= -δ); every row here is the solve moving material out of the way"
    );
    assert!(
        do_nothing < 0.0,
        "the do-nothing configuration does not penetrate ({do_nothing:.3e}), so a \
         positive min_sd would prove nothing about the solve"
    );
    // The two-sided half: if the ORACLE penetrates, penetration is a property of
    // the fixture and not of the reduced path, and the reduced rows would be
    // evidence about the wrong thing.
    assert!(
        oracle.arm.min_sd > 0.0,
        "the full-order oracle itself penetrates ({:.3e}) — the barrier clamp is \
         being hit at full order",
        oracle.arm.min_sd,
    );

    let mut train = SnapshotSet::new(fd.len());
    for x in &oracle.x {
        train.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
    }

    let mut arms = Vec::new();
    let mut achieved = std::collections::BTreeSet::new();
    for &r in ranks {
        let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, r).expect("basis fits");
        let got = basis.n_modes();
        // ★ `fit` drops modes below `σ/σ_max < 1e-8`, so the ladder cannot climb
        // above the trajectory's own numerical rank. Asking for more and silently
        // getting the SAME basis back would print two identical rows that read as
        // two independent pieces of evidence.
        if !achieved.insert(got) {
            println!("RC\tr={r:<3} requested — same {got}-mode basis as a lower rung, skipped");
            continue;
        }
        if got < r {
            println!(
                "RC\tr={r:<3} requested, {got} kept — the ramp's numerical rank at \
                 σ/σ_max ≥ 1e-8 is {got} of {} snapshots",
                train.len(),
            );
        }
        for (name, guess) in GUESSES {
            let arm = run_reduced(
                scene,
                &basis,
                guess,
                format!("r={r}→{got} {name}"),
                &Ctx {
                    x_rest: &x_rest,
                    fd: &fd,
                    oracle: &oracle,
                },
                NO_TIMING,
            );
            arm.print(n_steps, scene.d_hat);
            arms.push((r, name, arm));
        }
    }

    Ladder { oracle, arms }
}

/// Step 1 of the contact arc: produce the thing `reduced_phase_shares` would
/// time, and check the properties that decide whether timing it is meaningful —
/// it completes the ramp, and it does not put material inside the collider.
#[test]
#[ignore = "reduced-path contact producer check — ~30 s, run explicitly (see module docs)"]
fn reduced_path_with_ipc_contact() {
    let l = ladder(Scene::new(A_OVER_CELL), &R_LADDER);
    println!(
        "\nRC\t⇒ contact costs 1.43 ms per Newton iteration at 18 750 (927.7 ms × 1.0 % ÷ 6.51),\n\
         RC\t  all of it irreducible ⇒ R3 needs iters/step ≲ 12 there. The reduced arm\n\
         RC\t  inheriting the ORACLE's count is the pass; read the two rows together."
    );
    let faults = l.faults(R_REFERENCE, l.oracle.arm.min_sd);
    assert!(faults.is_empty(), "{}", faults.join("\n"));
}

/// ★ The same property, cheap enough to run on every CI job.
///
/// The instrument above is `#[ignore]`d, so it guards nothing by itself — a
/// release-only or ignored test runs in NO job unless something names it, and
/// green then means SKIPPED. This is the always-on half: one coarse mesh, the
/// trajectory's full rank, both predictors, asserting exactly the two
/// non-negotiables (the ramp completes, and nothing ends up inside the sphere)
/// plus the vacuity and negative controls inside [`ladder`].
///
/// What would trip it: any future change to the reduced line search, the
/// convergence quantity, or — the one it is really here for — R3's hyper-reduced
/// assembly, which changes the forces the barrier is balanced against.
#[test]
fn reduced_contact_does_not_tunnel_through_the_barrier() {
    let l = ladder(Scene::new(GATE_A_OVER_CELL), &[GATE_RANK]);
    let faults = l.faults(GATE_RANK, l.oracle.arm.min_sd);
    assert!(faults.is_empty(), "{}", faults.join("\n"));
}

/// Step 2 of the contact arc: **`I` on the fixture the requirement is stated
/// for.**
///
/// ```text
/// cargo test --release -p sim-soft --features phase-timing \
///   --test reduced_contact -- --ignored --nocapture --test-threads=1
/// ```
///
/// Margin is `B / I`, so only the irreducible part of a frame decides R3 — and
/// until this ran, `I` had only ever been measured on a contact-free cantilever
/// where `61 %` of it was `contact` marshalling on a `NullContact` scene. The
/// largest term in the deciding quantity was a placeholder for the thing that
/// was absent.
///
/// Three arms over one ramp: the full-order solve (which is also the snapshot
/// source and the oracle), then the reduced solve under each predictor. The
/// full-order arm is not decoration — every projection made so far divided a
/// published `1.0 %` share by a published `927.7 ms` to get `1.43 ms` of contact
/// per Newton iteration, and this measures that directly, on this box, in the
/// same run as the thing it is compared against.
///
/// ⚠ `--test-threads=1` is not optional: the profiler's slots are process-global
/// statics, and a second test running concurrently would have its time land in
/// this one's snapshot.
///
/// ## Measured — six runs at 18 750, two at 5 202, 2026-08-24, reference box
///
/// | arm | ms/step | iters/step | `I` ms | `B/I` |
/// |---|---:|---:|---:|---:|
/// | full-order (`PreviousState`) | 812.4–825.0 | 6.00 | 666.6–677.9 | 0.02–0.03× |
/// | reduced, `PreviousState` | 303.8–314.5 | 9.00 | 17.1–18.2 | **0.92–0.98×** ⛔ |
/// | reduced, `Inertial` | 181.9–184.9 | 5.25 | 11.3–12.3 | **1.36–1.47×** ✓ |
///
/// - ★★ **R3 clears here, but on `1.4×`, not the `5.6×` the contact-free fixture
///   reports.** The old number was not wrong, it was measured somewhere the
///   requirement does not live; it overstated the margin by about `4×`.
/// - ★★ **The predictor is now load-bearing for R3, not merely an optimisation.**
///   `PreviousState` needs `9.00` iterations against `Inertial`'s `5.25`, and `I`
///   is dominated by per-iteration cost, so the same rung passes under one
///   predictor and fails under the other. It fails by only `3–8 %`, which is
///   close — but it fails in all six runs.
/// - ★ **The reduced path's iteration PENALTY grows with size, and the
///   full-order path's does not.** Same trailing window at both sizes:
///
///   | free DOF | full | prev | inertial | prev/full | inert/full |
///   |---:|---:|---:|---:|---:|---:|
///   | 5 202 | 6.00 | 6.88 | 4.12 | 1.15× | 0.69× |
///   | 18 750 | 6.00 | 9.00 | 5.25 | **1.50×** | **0.88×** |
///
///   Full-order is flat; reduction costs progressively more iterations as the
///   problem grows. The counts are exact integers per step, so the ratios
///   reproduce run to run — it is the margins below them that need repeats.
///
///   ⚠ **The first version of this harness got the right answer from the wrong
///   comparison, and the review pass caught it.** It read 5 202's WHOLE-RAMP
///   average (`6.44`) against 18 750's LAST-8-STEPS average (`9.00`) and called
///   the difference size — but the last steps are the deepest indentation, where
///   the count is highest anyway. Two variables had moved. Re-measured in one
///   window the effect survives and is larger than it looked on the reduced
///   side (`1.15 → 1.50`, not `1.06 → 1.50`).
/// - ★★ **The margin is not a constant of the ladder — it falls with size.**
///   `I` is `3.75–3.80 ms` at 5 202 and `11.3–12.3` at 18 750, i.e. `∝ n^0.88`
///   against a fixed `16.7 ms` budget, so `B/I` goes `4.4× → 1.4×`.
///   Extrapolated, **R3's headroom runs out near `28 k` free DOF** — about
///   `1.5×` the requirement's own fixture. ⚠ Two points and a power law; treat
///   it as a marker for where to measure next, not as a number.
/// - `I` is `69 %` contact and `29 %` the validity sweep, the rest negligible.
///   The sweep is [`Reducible::PlannedByR3`], so R3's own `ReducedValidityDomain`
///   is worth about `+0.5×` of margin here (`I` would fall to `~8.7 ms`). That
///   does NOT revive it as a prerequisite — the rung clears either way — but on a
///   `1.4×` margin it is no longer irrelevant, which it was at `5.6×`.
/// - The `1.43 ms` of contact per Newton iteration this arc had been projecting
///   from two published tables measures **`1.26–1.75`** here (a wide spread:
///   `contact` is ~1 % of a full-order frame, so its per-iteration figure is a
///   small difference of large numbers). The projection was
///   sound; the iteration count it was multiplied by was not.
#[test]
#[ignore = "reduced-path contact TIMING — needs --features phase-timing, ~2 min"]
fn reduced_contact_phase_shares() {
    refbox::require_quiet_box();

    let sizes: Vec<SizeRow> = TIMING_A_OVER_CELL.into_iter().map(timing_fixture).collect();

    // ── the cross-size comparison, which is the whole reason 5 202 is here ──
    //
    // Both rows are the SAME trailing window on the SAME ramp, so the only thing
    // that differs is the mesh. Compare `reduced / full-order` rather than the
    // reduced count alone: the full-order arm absorbs whatever the window's
    // depth does to Newton, and what is being asked is whether REDUCTION costs
    // more iterations as the problem grows.
    println!("\nRC\t╔═ does the reduced path's iteration PENALTY grow with size?");
    println!(
        "RC\t║ {:<10} {:>8} {:>8} {:>10} {:>12} {:>12}",
        "free DOF", "full", "prev", "inertial", "prev/full", "inert/full"
    );
    for s in &sizes {
        println!(
            "RC\t║ {:<10} {:>8.2} {:>8.2} {:>10.2} {:>11.2}× {:>11.2}×",
            s.free_dof,
            s.full_iters,
            s.prev_iters,
            s.inertial_iters,
            s.prev_iters / s.full_iters,
            s.inertial_iters / s.full_iters,
        );
    }
    println!("RC\t╚═");
}

/// One size's three arms, reduced to the iteration counts the size comparison
/// needs. The per-size verdicts are printed by [`timing_fixture`] as they run.
struct SizeRow {
    free_dof: usize,
    full_iters: f64,
    prev_iters: f64,
    inertial_iters: f64,
}

fn timing_fixture(a_over_cell: f64) -> SizeRow {
    let scene = Scene::new(a_over_cell);
    let probe = scene.mesh();
    let x_rest = rest_positions(&probe);
    let solver = scene.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let n_steps = scene.n_steps();
    // Checked here, against the ramp's real length, for the reason
    // `TIMING_STEPS` gives: a `const` form could only compare against a
    // hardcoded count and would go stale in silence.
    assert!(
        TIMING_STEPS < n_steps,
        "the timed window is {TIMING_STEPS} of {n_steps} steps — it would leave \
         no warmup, and `n_steps - TIMING_STEPS` would underflow",
    );
    let measure_from = n_steps - TIMING_STEPS;

    println!(
        "\nRC\tTIMING fixture: IPC indentation a/cell={a_over_cell:.1}, {} free DOF, \
         {} tets, {n_steps} ramp steps, timing the last {TIMING_STEPS}",
        fd.len(),
        probe.n_tets(),
    );

    // ── arm 0: full-order. Oracle, snapshot source, and the measured contact
    // cost that replaces the extrapolation. ⚠ It runs `PreviousState`, so the
    // "full-order" row is that predictor's; the reduced rows are labelled with
    // theirs, and only the reduced rows carry a verdict. ──
    let oracle = run_oracle(scene, &x_rest, measure_from);
    assert!(
        oracle.arm.failure.is_none(),
        "the full-order arm did not survive its ramp: {}",
        oracle.arm.failure.as_deref().unwrap_or(""),
    );
    let full_snap = profile::snapshot();
    let full = reduced_report::main_report(
        &format!("FULL-ORDER ({} free DOF), PreviousState", fd.len()),
        oracle.arm.measured.wall_ms,
        oracle.arm.measured.steps,
        oracle.arm.measured.iters,
    );

    // ⚠ VACUITY GUARD ON THE WINDOW, not on the ramp. The correctness tests
    // already check that contact engages SOMEWHERE; what matters here is that it
    // is engaged in the steps that were TIMED. An early-ramp window would report
    // `I` for a scene with no active pairs — the exact defect this fixture
    // exists to retire, reintroduced one level down.
    let deepest = min_signed_distance(
        oracle.x.last().expect("the ramp produced steps"),
        scene.centre_at(n_steps - 1),
    );
    println!(
        "RC\tWINDOW CHECK — gap at the last timed step {deepest:.3e} vs band {:.2e}: \
         contact is {} during the measurement",
        scene.d_hat,
        if deepest < scene.d_hat {
            "ACTIVE"
        } else {
            "INACTIVE — the timing is of a contact-free frame"
        },
    );
    assert!(
        deepest < scene.d_hat,
        "the timed window has no active contact ({deepest:.3e} ≥ {:.2e})",
        scene.d_hat,
    );
    assert!(
        full_snap.calls(Phase::Contact) > 0,
        "the contact slot recorded no calls in the timed window",
    );

    // ── the basis, from the same trajectory (in-sample; see the module docs) ──
    let mut train = SnapshotSet::new(fd.len());
    for x in &oracle.x {
        train.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, TIMING_RANK).expect("basis fits");
    let r = basis.n_modes();
    println!(
        "RC\tbasis: r={r} kept of {TIMING_RANK} requested, {} snapshots",
        train.len()
    );

    // ── arms 1 and 2: the reduced solve under each predictor ──
    let mut rows = vec![("FULL-ORDER", full, oracle.arm.measured, full_snap)];
    for (name, guess) in GUESSES {
        let arm = run_reduced(
            scene,
            &basis,
            guess,
            format!("r={r} {name}"),
            &Ctx {
                x_rest: &x_rest,
                fd: &fd,
                oracle: &oracle,
            },
            measure_from,
        );
        assert!(
            arm.failure.is_none(),
            "reduced {name} did not survive its ramp: {}",
            arm.failure.as_deref().unwrap_or(""),
        );
        assert!(
            arm.min_sd > 0.0,
            "reduced {name} PENETRATED at {} free DOF (min_sd {:.3e}) — the timing \
             would be of a configuration inside the collider",
            fd.len(),
            arm.min_sd,
        );
        let snap = profile::snapshot();
        let v = reduced_report::main_report(
            &format!("REDUCED r={r}, {name} predictor"),
            arm.measured.wall_ms,
            arm.measured.steps,
            arm.measured.iters,
        );
        rows.push((name, v, arm.measured, snap));
    }

    // ── the per-size comparison, and the control ──
    println!("\nRC\t╔═ {} free DOF — what decides R3", fd.len());
    println!(
        "RC\t║ {:<14} {:>10} {:>10} {:>10} {:>9} {:>12}",
        "arm", "ms/step", "iters/st", "I ms", "B/I", "contact/iter"
    );
    for (label, v, m, snap) in &rows {
        let per_iter = if snap.calls(Phase::Contact) > 0 {
            snap.millis(Phase::Contact) / m.iters as f64
        } else {
            f64::NAN
        };
        println!(
            "RC\t║ {label:<14} {:>10.2} {:>10.2} {:>10.3} {:>8.2}× {:>12.3}",
            v.per_step_ms,
            m.iters as f64 / m.steps as f64,
            v.irreducible_ms,
            v.margin,
            per_iter,
        );
    }
    println!("RC\t╚═");

    // ★ CROSS-PATH CONTROL. `contact` is the same code on the same mesh at
    // nearly the same configuration whichever way the solve is driven, so the
    // per-CALL cost must agree. If it does not, the slot is not measuring what
    // its name claims and every `I` above is built on it.
    let per_call = |snap: &profile::Phases| -> f64 {
        let c = snap.calls(Phase::Contact);
        assert!(c > 0, "no contact calls recorded — cannot form the control");
        snap.millis(Phase::Contact) / c as f64
    };
    let base = per_call(&rows[0].3);
    let mut off = Vec::new();
    for (label, _, _, snap) in rows.iter().skip(1) {
        let ratio = per_call(snap) / base;
        println!(
            "RC\t★ control: contact per call, {label} / full-order = {ratio:.3}× \
             ({:.4} vs {base:.4} ms)",
            per_call(snap),
        );
        // PILOTED at 0.80–1.17× over sixteen arm-comparisons across both
        // sizes. The spread is wide for a control because `contact` is ~1 % of a full-order frame, so the
        // DENOMINATOR is a small difference of large numbers; the band is set
        // for that, not for the agreement the two paths actually show. What it
        // still catches is the failure it is for — a slot that double-counts,
        // misses the broad phase, or books a different set of calls on one of
        // the two paths, all of which land ≥2× out.
        if !(0.6..1.7).contains(&ratio) {
            off.push(format!(
                "{label}: contact per call is {ratio:.3}× full-order"
            ));
        }
    }
    assert!(
        off.is_empty(),
        "the contact slot does not measure the same work on both paths, so every \
         `I` above is built on it:\n{}",
        off.join("\n"),
    );

    let iters_of = |i: usize| rows[i].2.iters as f64 / rows[i].2.steps as f64;
    SizeRow {
        free_dof: fd.len(),
        full_iters: iters_of(0),
        prev_iters: iters_of(1),
        inertial_iters: iters_of(2),
    }
}
