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
//!
//! ## The other studies in this file
//!
//! The producer check above is the first of SIX `#[ignore]`d studies, and each
//! later one answers what its predecessor's caveat leaves open.
//!
//! ⚠ They do NOT all share one fixture. The first two carry their own ladders —
//! [`R_LADDER`] for the producer check, [`TIMING_A_OVER_CELL`] (which reaches
//! `a/cell = 3`) for the timing run. **The last FOUR share
//! [`GEN_A_OVER_CELL`] and [`BASIS_RANKS`] deliberately**, so each is scored
//! against the matrix the previous rung published rather than a lookalike re-run;
//! §2n and §2o then vary the training ENSEMBLE on that matrix — and §2n adds one
//! off-lattice TEST position — which is exactly what each is built to change.
//!
//! - [`reduced_contact_phase_shares`] (recon §2k) — step 2 of the contact arc:
//!   `I` measured on the fixture R3's requirement is stated for.
//! - [`reduced_basis_generalises`] (recon §2l) — does the basis hold up when the
//!   contact patch MOVES? **No**, and rank does not fix it. Failure is SILENT in
//!   **7 of the 8** extrapolation arms: they converge, complete all 71 steps, and
//!   do not penetrate while `14.8 %`–`109 %` wrong. (The eighth, `+2.00a` at
//!   `r=20`, stalls at step 65 — the only one the solver itself flags, and the
//!   highest cell in the matrix at `119.6 %`.) That is what makes §4c's validity
//!   gate a CORRECTNESS prerequisite.
//! - [`an_online_signal_separates_out_of_domain`] (recon §2m) — given that, what
//!   can a gate actually WATCH at runtime? Four oracle-free candidates scored on
//!   the same matrix. `gap_dev`, the detector §2l found, needs the oracle and is
//!   disqualified; one candidate survives a per-step test at `~2×`, on a FITTED
//!   threshold — there is no held-out position in that matrix.
//! - [`the_signal_margin_on_a_held_out_position`] (recon §2n) — supplies one, and
//!   it disqualifies the survivor: across four ranks the error spans `31.9×`
//!   while the signal spans `1.005×`. **Rank-independence and
//!   accuracy-sensitivity are mutually exclusive.**
//! - [`how_dense_the_training_ensemble_must_be`] (recon §2o) — "density" is TWO
//!   factors, and the signals read the weaker one: ENSEMBLE SIZE beats GAP
//!   `2.1–2.7×` at matched rank, and no online candidate can express size.
//!
//! ⚠ The always-on [`reduced_contact_does_not_tunnel_through_the_barrier`] is
//! not one of the six — it is the CI half of the producer check, not a study.

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
    Inner, PodBasis, ReducedNewtonSolver, ReducedStep, SnapshotSet,
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
/// ⚠ THE LAST ones, and that choice is load-bearing — though **not for the
/// reason an earlier version of this comment gave.** It claimed early frames
/// have "no active pairs at all". They do: `z_start` is `1.2·d̂` above the
/// surface and the first increment is `0.3·d̂`, so step 0 already sits at
/// `0.90·d̂` and the barrier is live for the entire ramp.
///
/// The real reason is the PATCH, not the presence of contact. It grows as the
/// indenter descends — step 0 is `+0.90 d̂` of clearance against `-18 d̂` of
/// interference at the first timed step — so contact cost rises monotonically
/// and the trailing window is the conservative choice for `I`. Timing an early
/// window would understate the deciding quantity, which is the same error as
/// timing a contact-free fixture, just smaller.
///
/// ⚠ Whether this leaves any warmup is checked at RUNTIME, against the ramp's
/// actual length. A `const` assertion here could only compare it to a hardcoded
/// step count, and the ramp's length is derived from `d̂` and `δ` — so the
/// constant would go stale in silence the moment either moved, which is the
/// shape of a guard that cannot fail for the reason it claims.
const TIMING_STEPS: usize = 8;

/// How deep the ramp must already be at the first timed step, in units of `d̂`,
/// as pose interference against the undeformed surface.
///
/// ★ PILOTED and DISCRIMINATING, which the "contact is active" check beside it
/// is not: the ramp reads `+0.90 d̂` at step 0 and `-18.0 d̂` at the first timed
/// step, so a floor of `-10 d̂` sits between them and moving the window forward
/// trips it. Both `a/cell` rows use the same ramp, so one constant covers both.
const MIN_WINDOW_DEPTH_BANDS: f64 = -10.0;

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

/// The Hertzian contact patch radius `a = √(Rδ)`. The natural length for a
/// LATERAL offset, because it is the width of the thing being translated.
fn patch_radius() -> f64 {
    (RADIUS * DELTA).sqrt()
}

fn indenter(lx: f64, ly: f64, dx: f64, z_center: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(dx.mul_add(patch_radius(), lx / 2.0), ly / 2.0, z_center),
    }
}

#[derive(Clone, Copy)]
struct Scene {
    /// Lateral offset of the indenter from the plate's centre, in units of the
    /// contact patch radius `a`. Zero for every fixture except the basis
    /// generalisation sweep, which translates the patch to ask whether a global
    /// POD basis can follow it.
    dx: f64,
    a_over_cell: f64,
    n_lat: usize,
    nz: usize,
    lateral: f64,
    h: f64,
    d_hat: f64,
}

impl Scene {
    fn new(a_over_cell: f64) -> Self {
        Self::at_offset(a_over_cell, 0.0)
    }

    /// The same scene with the indenter moved sideways by `dx` patch radii.
    fn at_offset(a_over_cell: f64, dx: f64) -> Self {
        let (n_lat, nz, lateral, h) = dims_for(a_over_cell);
        Self {
            dx,
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
                vec![indenter(
                    self.lateral,
                    self.lateral,
                    self.dx,
                    self.z_start(),
                )],
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
        Vec3::new(
            self.dx.mul_add(patch_radius(), self.lateral / 2.0),
            self.lateral / 2.0,
            self.z_at(k),
        )
    }

    fn contact_at(self, k: usize) -> IpcRigidContact {
        IpcRigidContact::with_params(
            vec![indenter(self.lateral, self.lateral, self.dx, self.z_at(k))],
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
    // ⚠⚠ FINITENESS FIRST, and it is not defensive padding. `f64::min(x, NaN)`
    // returns `x`, so a diverged configuration full of NaN would reduce to a
    // perfectly healthy-looking positive gap and sail through the
    // non-penetration assertion this function exists to feed. `contact/ipc.rs`
    // carries a comment warning about exactly this reduction, and
    // `predictor_spike.rs` filters non-finite `x_final` for the same reason.
    assert!(
        x.iter().all(|v| v.is_finite()),
        "non-finite position in the configuration being measured — the solve \
         diverged, and `f64::min` would have hidden it",
    );
    x.chunks_exact(3)
        .map(|c| (Vec3::new(c[0], c[1], c[2]) - centre).norm() - RADIUS)
        .fold(f64::INFINITY, f64::min)
}

/// The two reductions this file's verdicts rest on, checked directly.
///
/// ★ Both are `f64::min`/`f64::max` folds, which SWALLOW `NaN` — the failure the
/// guard in [`min_signed_distance`] exists for. A guard that has never been
/// shown to fire is not a guard, so it is negative-controlled here rather than
/// left to a diverged solve that may never happen.
#[test]
#[should_panic(expected = "non-finite position")]
fn min_signed_distance_refuses_a_diverged_configuration() {
    let mut x = vec![0.0; 9];
    x[4] = f64::NAN;
    let _ = min_signed_distance(&x, Vec3::zeros());
}

/// The two-sided half: the same shape of input, finite, reduces to the NEAREST
/// vertex's gap — so the test above is failing on the `NaN` and not on the fold.
#[test]
fn min_signed_distance_takes_the_nearest_vertex() {
    // Two vertices on the x axis at 3·RADIUS and 2·RADIUS from a sphere at the
    // origin, so the gaps are 2·RADIUS and RADIUS and the second one wins.
    let x = vec![3.0 * RADIUS, 0.0, 0.0, 2.0 * RADIUS, 0.0, 0.0];
    let got = min_signed_distance(&x, Vec3::zeros());
    assert!(
        (got - RADIUS).abs() < 1e-15 * RADIUS,
        "expected the nearer vertex's gap {RADIUS:e}, got {got:e}",
    );
}

/// `rel_l2` returns `NaN` on a zero-displacement reference, which the ramp's first
/// steps produce before the indenter reaches the band. The caller SKIPS those
/// rather than folding them into a `f64::max` that would drop them silently —
/// this pins the value the skip is keyed on.
#[test]
fn rel_l2_is_nan_against_a_zero_reference() {
    assert!(rel_l2(&[1.0, 0.0], &[0.0, 0.0]).is_nan());
    assert!(rel_l2(&[3.0, 4.0], &[0.0, 0.0]).is_nan());
    let e = rel_l2(&[1.0, 1.0], &[1.0, 0.0]);
    assert!(e.is_finite() && (e - 1.0).abs() < 1e-15, "got {e}");
}

/// A clean arm, for exercising the verdict logic without running a solve.
#[cfg(test)]
fn healthy_arm(label: &str) -> Arm {
    Arm {
        label: label.to_owned(),
        completed: 71,
        iters: 426,
        min_sd: 1.0e-5,
        max_rel_err: 1.0e-7,
        last_full_r: 1.0e-11,
        gap_dev: 0.0,
        signals: None,
        measured: Measured::default(),
        failure: None,
    }
}

/// ★ Negative control for [`faults_at`]'s vacuity guard. Asserting on a rank
/// that recorded no arm would return an empty verdict and pass — the failure the
/// guard exists for, and one no passing run can demonstrate.
#[test]
#[should_panic(expected = "no arm was recorded at r=40")]
fn faults_at_refuses_a_rank_with_no_arm() {
    let arms = vec![(20, "prev", healthy_arm("r=20 prev"))];
    let _ = faults_at(&arms, 40, 1.0e-5);
}

/// The two-sided half: a rank that IS present, and clean, reports nothing — so
/// the panic above is the missing rank and not the machinery.
#[test]
fn faults_at_passes_a_clean_arm() {
    let arms = vec![(40, "inertial", healthy_arm("r=40 inertial"))];
    assert!(faults_at(&arms, 40, 1.0e-5).is_empty());
}

/// ★ And that the checks themselves fire, so "clean arm reports nothing" is not
/// "nothing is ever reported".
#[test]
fn faults_at_catches_penetration_and_a_moved_equilibrium() {
    let mut sunk = healthy_arm("r=40 prev");
    sunk.min_sd = -1.0e-9;
    let mut drifted = healthy_arm("r=40 inertial");
    drifted.gap_dev = 10.0 * MAX_GAP_DEV_BANDS;
    let arms = vec![(40, "prev", sunk), (40, "inertial", drifted)];
    let out = faults_at(&arms, 40, 1.0e-5);
    assert_eq!(out.len(), 2, "{out:#?}");
    assert!(out[0].contains("PENETRATED"), "{}", out[0]);
    assert!(out[1].contains("contact equilibrium moved"), "{}", out[1]);
}

/// ★ Negative control for the OTHER half of the finiteness split: the gate
/// refuses a non-finite configuration, but the failure formatter must survive
/// one — a diverged `x_partial` is exactly where a `NaN` lives, and a panic there
/// would replace "which arm died, and where" with nothing.
#[test]
fn describe_survives_a_diverged_partial() {
    let scene = Scene::new(GATE_A_OVER_CELL);
    let e = SolverFailure::ArmijoStall {
        x_partial: vec![f64::NAN; 9],
        last_iter: 7,
        last_r_norm: 1.0e-3,
    };
    let msg = describe(&e, scene, 0);
    assert!(msg.contains("ArmijoStall at iter 7"), "{msg}");
    assert!(msg.contains("non-finite (diverged)"), "{msg}");
}

/// ★ Negative control for [`MIN_WINDOW_DEPTH_BANDS`], and the check that the
/// "contact is active in the timed window" assertion beside it CANNOT make.
///
/// The barrier is live for this ramp's whole length — step 0 already sits at
/// `0.90 d̂` — so an active-contact test passes at every step and discriminates
/// nothing. The depth floor does: it rejects step 0 and accepts the real window.
/// This also pins the "step 0 is already inside the band" fact that the window
/// comment now rests on, after an earlier version claimed the opposite.
#[test]
fn the_window_depth_guard_discriminates_early_from_deep() {
    let scene = Scene::new(GATE_A_OVER_CELL);
    let x_rest = rest_positions(&scene.mesh());
    let bands = |k: usize| min_signed_distance(&x_rest, scene.centre_at(k)) / scene.d_hat;
    let early = bands(0);
    let deep = bands(scene.n_steps() - TIMING_STEPS);
    assert!(
        (0.0..1.0).contains(&early),
        "step 0 should be inside the band ({early} d̂) but clear of the surface",
    );
    assert!(
        early > MIN_WINDOW_DEPTH_BANDS,
        "the depth floor must REJECT an early window, but {early} d̂ passes it",
    );
    assert!(
        deep <= MIN_WINDOW_DEPTH_BANDS,
        "the depth floor must ACCEPT the window actually used, but {deep} d̂ fails it",
    );
}

/// ★ Negative controls for [`in_sample_advantage`]'s three guards. Each is a
/// state no passing run of the instrument can reach, which is exactly why they
/// need exercising here instead.
#[cfg(test)]
fn rows_for(pairs: &[(&'static str, usize, f64)]) -> Vec<(&'static str, usize, Arm)> {
    pairs
        .iter()
        .map(|&(l, r, err)| {
            let mut a = healthy_arm(l);
            a.max_rel_err = err;
            (l, r, a)
        })
        .collect()
}

/// Every arm that took a step must have PRODUCED both a signal and a usable
/// error — and the full matrix must be present.
///
/// ★ Extracted from [`an_online_signal_separates_out_of_domain`] ONLY so these
/// three guards can be exercised, exactly as [`faults_at`] is. All three are the
/// shape this file keeps producing: **a quantity whose healthy reading is
/// ZERO**, where "never measured" and "perfect" are the same number.
///
/// - an arm run without a [`Probe`] has no signal at all;
/// - an arm that recorded nothing keeps an EMPTY [`SignalTrace`], which would
///   summarise to nothing at all — and every domain signal's healthy reading is
///   zero, so it must be refused rather than folded;
/// - `max_rel_err` initialises to `0.0` and only updates for FINITE samples, so
///   an arm with no usable error sample reads as an EXACT answer. A completed
///   arm cannot be bit-identical to its oracle — full rank is `2.57e-6`.
/// - and the whole loop skips arms that completed no steps, so without the count
///   it goes VACUOUS when nothing completed.
fn assert_every_arm_produced(rows: &[(&'static str, usize, Arm)], expected: usize) {
    let mut checked = 0usize;
    for (label, got, arm) in rows {
        if arm.completed == 0 {
            continue;
        }
        let s = arm
            .signals
            .as_ref()
            .unwrap_or_else(|| panic!("{label} r={got} was run with NO probe attached"));
        assert!(
            s.is_recorded(),
            "{label} r={got} completed {} steps and recorded NO signal — every \
             domain signal's healthy value is zero, so an unrecorded arm would \
             read as perfectly in domain",
            arm.completed,
        );
        assert!(
            arm.max_rel_err > 0.0,
            "{label} r={got} completed {} steps with max_rel_err exactly 0.0 — \
             that is 'no usable sample', not a perfect answer",
            arm.completed,
        );
        checked += 1;
    }
    assert_eq!(
        checked, expected,
        "only {checked} of {expected} arms were signal-checked; the rest \
         completed no steps, so the tables are not a full matrix",
    );
}

/// ★ The three negative controls for it. Every one is a state no passing run of
/// the instrument can reach — which is the whole reason they live here.
#[cfg(test)]
fn probed_rows(errs: &[f64]) -> Vec<(&'static str, usize, Arm)> {
    errs.iter()
        .enumerate()
        .map(|(i, &err)| {
            let mut a = healthy_arm("probed");
            a.max_rel_err = err;
            let mut t = SignalTrace::new();
            t.record(
                Signals {
                    residual_excess: 1.0e6 + i as f64,
                    envelope_excursion: 0.0,
                    snapshot_distance: 0.25,
                    active_novelty: 0.0,
                },
                7,
            );
            a.signals = Some(t);
            ("probed", 40, a)
        })
        .collect()
}

#[test]
fn assert_every_arm_produced_passes_a_full_matrix() {
    assert_every_arm_produced(&probed_rows(&[1.0e-3, 2.0e-3]), 2);
}

#[test]
#[should_panic(expected = "NO probe attached")]
fn assert_every_arm_produced_refuses_an_unprobed_arm() {
    let mut rows = probed_rows(&[1.0e-3]);
    rows[0].2.signals = None;
    assert_every_arm_produced(&rows, 1);
}

#[test]
#[should_panic(expected = "recorded NO signal")]
fn assert_every_arm_produced_refuses_an_arm_that_recorded_nothing() {
    let mut rows = probed_rows(&[1.0e-3]);
    rows[0].2.signals = Some(SignalTrace::new());
    assert_every_arm_produced(&rows, 1);
}

#[test]
#[should_panic(expected = "not a perfect answer")]
fn assert_every_arm_produced_refuses_a_zero_error_arm() {
    assert_every_arm_produced(&probed_rows(&[0.0]), 1);
}

/// ★ The OTHER side of the count, which a mutation round found unexercised:
/// `checked > expected` is unreachable today (`checked` counts a subset of
/// `rows`), so relaxing the equality to `<=` survived the whole suite. It stops
/// being unreachable the moment someone pushes a row the expectation does not
/// know about — which is precisely what the count exists to catch.
#[test]
#[should_panic(expected = "not a full matrix")]
fn assert_every_arm_produced_refuses_more_arms_than_expected() {
    assert_every_arm_produced(&probed_rows(&[1.0e-3, 2.0e-3]), 1);
}

#[test]
#[should_panic(expected = "not a full matrix")]
fn assert_every_arm_produced_refuses_a_vacuous_matrix() {
    // ⚠⚠ THE dangerous one: every arm completed nothing, so the loop body never
    // runs and every guard above passes having checked NOTHING.
    let mut rows = probed_rows(&[1.0e-3, 2.0e-3]);
    for r in &mut rows {
        r.2.completed = 0;
    }
    assert_every_arm_produced(&rows, 2);
}

/// The remaining `top_row` guard. Cheap, and it closes the sweep: every
/// assertion in the generalisation path is now either exercised by a test or
/// DECLARED structural below.
#[test]
#[should_panic(expected = "no rows recorded for `absent`")]
fn top_row_refuses_a_label_with_no_rows() {
    let rows = rows_for(&[("in", 40, 1.0e-6)]);
    let _ = top_row(&rows, "absent");
}

/// ⚠ **Declared UNCONTROLLED, deliberately — and the rule, not a list.** This
/// file carries **22 guards outside test bodies**; six have `#[should_panic]`
/// controls and two more (`MIN_WINDOW_DEPTH_BANDS`, `MAX_GAP_DEV_BANDS`) are
/// exercised two-sidedly by ordinary tests. The remaining fourteen all share one
/// property: **they fire only if the FIXTURE or the INSTRUMENT is broken** — a
/// mesh with no pinned face, a full-order oracle failing its own ramp, a
/// profiler slot recording no calls, a timed window with no contact in it.
/// Demonstrating those means injecting a fault into a multi-minute instrument
/// for no diagnostic gain, so they stay uncontrolled and this comment is the
/// record of that choice.
///
/// The `≥10×` floor is separately not controlled and does not need to be: the
/// number it compares is pinned by [`advantage_is_the_ratio_of_the_two_top_rows`]
/// and the comparison is a `>=`.
///
/// ⚠ An earlier version of this note said "three guards", which read as though
/// the other nineteen were controlled. The count above came from walking the
/// file and separating guards from assertions *inside* test bodies — two
/// earlier scripted audits got that wrong in opposite directions.
#[test]
#[should_panic(expected = "passes having measured NOTHING")]
fn advantage_refuses_a_zero_denominator() {
    // `max_rel_err` initialises to 0.0 and is only updated for finite samples,
    // so "recorded nothing" and "was perfect" are the same value.
    let rows = rows_for(&[("in", 40, 0.0), ("ip", 40, 1.0e-3)]);
    let _ = in_sample_advantage(&rows, "in", "ip");
}

#[test]
#[should_panic(expected = "scored at different ranks")]
fn advantage_refuses_a_rank_mismatch() {
    let rows = rows_for(&[("in", 80, 1.0e-6), ("ip", 40, 1.0e-3)]);
    let _ = in_sample_advantage(&rows, "in", "ip");
}

#[test]
#[should_panic(expected = "TRUNCATED trajectory")]
fn advantage_refuses_a_truncated_interpolation_arm() {
    let mut rows = rows_for(&[("in", 40, 1.0e-6), ("ip", 40, 1.0e-9)]);
    rows[1].2.failure = Some("ArmijoStall at iter 3".to_owned());
    let _ = in_sample_advantage(&rows, "in", "ip");
}

/// The two-sided half: clean inputs give the ratio, so the three panics above
/// are their stated causes and not the machinery.
#[test]
fn advantage_is_the_ratio_of_the_two_top_rows() {
    let rows = rows_for(&[("in", 40, 2.0e-6), ("ip", 40, 1.4e-3), ("in", 20, 9.9)]);
    let got = in_sample_advantage(&rows, "in", "ip");
    assert!((got - 700.0).abs() < 1e-9, "expected 700x, got {got}");
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
    // ⚠ NON-ASSERTING, deliberately. `min_signed_distance` refuses a non-finite
    // configuration because it feeds the non-penetration gate — but a diverged
    // solve's `x_partial` is EXACTLY where a non-finite position lives, so
    // calling it here would replace "which arm died, and where" with a panic
    // from inside the diagnostic. The guard is right for the gate and wrong for
    // the report; this reports the condition instead of asserting on it.
    let at = |x: &[f64]| -> String {
        if x.iter().all(|v| v.is_finite()) {
            format!("{:.3e}", min_signed_distance(x, scene.centre_at(k)))
        } else {
            "non-finite (diverged)".to_owned()
        }
    };
    match e {
        SolverFailure::ArmijoStall {
            x_partial,
            last_iter,
            last_r_norm,
        } => format!(
            "ArmijoStall at iter {last_iter}, ‖r‖ = {last_r_norm:.3e}, min_sd = {}",
            at(x_partial)
        ),
        SolverFailure::NewtonIterCap {
            x_partial,
            max_iter,
            last_r_norm,
        } => format!(
            "NewtonIterCap at {max_iter}, ‖r‖ = {last_r_norm:.3e}, min_sd = {}",
            at(x_partial)
        ),
        SolverFailure::DoublyFailedFactor {
            x_partial, context, ..
        } => format!("DoublyFailedFactor ({context}), min_sd = {}", at(x_partial)),
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
    /// Every step's ONLINE validity readings, `None` unless the arm was run with
    /// a [`Probe`]. Absent by default because recording them costs a
    /// `O(n_vertices)` band scan plus a `O(n_train · r)` nearest-snapshot search
    /// per step, and neither may land in the timing arms.
    signals: Option<SignalTrace>,
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
             min_sd={:>16.9e}\t(band d̂={d_hat:.2e})\tmax_relL2={:>9.3e}\t‖r‖full(LAST step)={:>9.3e}\tgap_dev={:>9.3e} d̂\t{}",
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
            signals: None,
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
    /// What the training ensemble SAW, for the §4c signal study. `None` for
    /// every arm that is not part of it — the recording is opt-in precisely so
    /// the timing and phase-share arms pay nothing for it, the same shape as
    /// [`NO_TIMING`].
    probe: Option<&'a Probe<'a>>,
}

fn run_reduced(
    scene: Scene,
    basis: &PodBasis,
    guess: InitialGuess,
    label: String,
    ctx: &Ctx<'_>,
    measure_from: MeasureFrom,
) -> Arm {
    let Ctx {
        x_rest,
        fd,
        oracle,
        probe,
    } = *ctx;
    let mut signals = probe.map(|_| SignalTrace::new());
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
                let centre = scene.centre_at(k);
                min_sd = min_sd.min(min_signed_distance(&x, centre));
                if let (Some(p), Some(acc)) = (probe, signals.as_mut()) {
                    let (reading, active) = p.read(&s, &x, centre);
                    acc.record(reading, active);
                }
                if let Some(ox) = oracle.x.get(k) {
                    let e = rel_l2(&free_disp(&x, x_rest, fd), &free_disp(ox, x_rest, fd));
                    // Same trap as `min_signed_distance`: `f64::max(x, NaN)` is
                    // `x`, so a NaN here would drop out of the running maximum
                    // instead of surfacing. `rel_l2` returns NaN when the oracle
                    // displacement is exactly zero, which the ramp's first steps
                    // can produce before the indenter reaches the band — so skip
                    // those deliberately rather than folding them in blind.
                    if e.is_finite() {
                        max_rel_err = max_rel_err.max(e);
                    }
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
        signals,
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
        faults_at(&self.arms, at_rank, oracle_min_sd)
    }
}

/// ⚠⚠ **The empty result is the dangerous one.** Callers do
/// `assert!(faults.is_empty())`, so a rank with NO recorded arm passes every
/// check silently. That is reachable: [`ladder`] SKIPS a requested rank whose
/// basis duplicates a lower rung's, so asking for `40` on a mesh whose ramp caps
/// at `20` records nothing at `40` and the gate goes green having tested
/// nothing. Refuse it rather than return an empty verdict.
fn faults_at(
    arms: &[(usize, &'static str, Arm)],
    at_rank: usize,
    oracle_min_sd: f64,
) -> Vec<String> {
    assert!(
        arms.iter().any(|(r, ..)| *r == at_rank),
        "no arm was recorded at r={at_rank}, so asserting on it would pass \
         having checked nothing — the rank was skipped as a duplicate basis, or \
         it is not in the ladder",
    );
    {
        let mut out = Vec::new();
        for (r, name, arm) in arms {
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
                    probe: None,
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
/// ## Measured — nine runs at 18 750, five at 5 202, 2026-08-24, reference box
///
/// (Four of the 18 750 runs predate the refactor into a two-size sweep and
/// measure the same window the same way; all eight sit inside the ranges below.
/// ⚠⚠ Every range here is min–max OBSERVED, not a bound — a ninth run landed
/// outside two of the wall-clock ones. What has NOT moved across any run is
/// `I`, `B/I`, and the iteration counts, which are exact integers per step, and
/// those are what the rung turns on. The 5 202 figures are rounded to one
/// decimal on purpose: they are context, and quoting context tight put these
/// documents on a treadmill where every verification run invalidated a figure.)
///
/// | arm | ms/step | iters/step | `I` ms | `B/I` |
/// |---|---:|---:|---:|---:|
/// | full-order (`PreviousState`) | 812.4–844.2 | 6.00 | 666.6–694.8 | 0.02–0.03× |
/// | reduced, `PreviousState` | 303.8–314.5 | 9.00 | 17.1–18.2 | **0.92–0.98×** ⛔ |
/// | reduced, `Inertial` | 180.7–184.9 | 5.25 | 11.3–12.3 | **1.36–1.47×** ✓ |
///
/// - ★★ **R3 clears here, but on `1.4×`, not the `5.6×` the contact-free fixture
///   reports.** The old number was not wrong, it was measured somewhere the
///   requirement does not live; it overstated the margin by about `4×`.
/// - ★★ **The predictor is now load-bearing for R3, not merely an optimisation.**
///   `PreviousState` needs `9.00` iterations against `Inertial`'s `5.25`, and `I`
///   is dominated by per-iteration cost, so the same rung passes under one
///   predictor and fails under the other. It fails by only `2–8 %`, which is
///   close — but it fails in all nine runs.
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
///   `I` is `3.6–3.8 ms` at 5 202 and `11.3–12.3` at 18 750 against a fixed
///   `16.7 ms` budget, so `B/I` goes `~4.5× → 1.36–1.47×`.
///
///   ⚠ **Do not fit one exponent to that.** `I`'s two real terms move in
///   opposite directions, and the blend (`n^0.89`) is an artefact of where the
///   crossover currently sits:
///
///   | term | 5 202 | 18 750 | scaling | share of `I` |
///   |---|---:|---:|---|---|
///   *(one run at each size — the ranges above are over three and six)*
///   | `contact` | 1.998 | 8.468 | **`n^1.13`** | 53 % → **69 %** |
///   | `validity check` | 1.687 | 3.561 | `n^0.58` | 45 % → 29 % |
///
///   Contact is taking over, so the blended exponent will keep rising toward
///   contact's. Summing the two terms separately puts `I = 16.7 ms` at
///   **`~26 k` free DOF** (the single blended fit says `~28 k`) — about `1.4×`
///   the requirement's own fixture. Two points per term, so it is a marker for
///   where to measure next, not a number. It does mean `1.4×` must not be read
///   as "R3 clears for soft bodies"; it clears **at this size**.
/// - ★★★ **And that identifies where the one available lever on R3's margin
///   is.** Contact is `65–69 %` of `I`, and by `C/R = B/I` only `I` moves a rung's
///   verdict — so `asm tangent`, at `66.7–68.7 %` of the reduced FRAME, moves it by
///   exactly zero, the same trap §2j's corollary caught for `red proj K`.
///
///   What the `Phase::Contact` slot actually wraps (`assembly.rs`) is **two**
///   `O(n)` passes plus the per-pair work: `slice_to_vec3s`, which allocates
///   and copies EVERY position into a fresh `Vec<Vec3>` on every call, and then
///   `active_pairs`. On a linear mesh `active_pairs` resolves to
///   `active_vertex_pairs`, which evaluates every primitive at every vertex
///   with no broad phase. Neither term has anything to do with how many pairs
///   are actually active.
///
///   ★★ **Recon §2d finding 4 already described both passes** — "marshal every
///   vertex into a fresh `Vec<Vec3>` and run the broad-phase scan before any
///   pair exists", twice per Newton iteration — and then set them aside:
///   *"Neither is on R3's path."* Under `C/R = B/I` that is overturned. The
///   cost is `1–2 %` of a whole frame, which is why it looked ignorable, and
///   `65–69 %` of `I`, which is the only quantity that decides the rung.
///
///   ⚠ **Two scoping corrections to the paragraph above, both from round 3.**
///   (a) The `O(n_vertices)` walk is the LINEAR-mesh path; a Tet10 mesh
///   dispatches to `active_face_pairs`, which iterates boundary faces.
///   (b) The `n^1.13` is NOT a per-call scaling. Contact's `4.24×` growth
///   factors as `1.37×` more calls (Newton iterations went `4.12 → 5.25`) ×
///   `3.08×` more work per call — and per call against `3.97×` more vertices
///   that is `n_vert^0.88`, sublinear, not linear. The `O(n)` structure is
///   real and verified in the source; the exponent was over-attributed to it.
///
///   ⚠ No size of win is claimed for removing either term.
/// - `I` is `65–69 %` contact and `29–33 %` the validity sweep, the rest negligible.
///   The sweep is [`Reducible::PlannedByR3`], so R3's own `ReducedValidityDomain`
///   is worth about `+0.5–0.8×` of margin here (`I` would fall to `7.7–8.7 ms`). That
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

    // ⚠ WINDOW GUARD — two claims, because the obvious one is nearly vacuous
    // here. The barrier is live for this ramp's whole length (step 0 is already
    // at `0.90 d̂`), so "contact is active in the timed steps" can only fail if
    // the geometry changes radically. What actually has to hold is that the
    // window sits in the DEEP part, where the patch and therefore the cost are
    // largest — an early window would understate `I`, which is the same error as
    // timing a contact-free fixture, just smaller. Both are checked; the depth
    // one is the one with teeth.
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
    // ★ The guard with teeth: pose depth at the FIRST timed step, measured on
    // the undeformed mesh so it is a property of the window's placement and not
    // of the solve. Step 0 reads `+0.90 d̂`; the first timed step reads `-18 d̂`.
    // `MIN_WINDOW_DEPTH_BANDS` sits between them, so moving the window forward
    // trips this — which is what makes it a check rather than a restatement.
    let window_depth = min_signed_distance(&x_rest, scene.centre_at(measure_from)) / scene.d_hat;
    println!(
        "RC\tWINDOW DEPTH — pose interference at the first timed step is \
         {window_depth:.1} d̂ (step 0 is +0.90 d̂); floor {MIN_WINDOW_DEPTH_BANDS:.1} d̂"
    );
    assert!(
        window_depth <= MIN_WINDOW_DEPTH_BANDS,
        "the timed window is not in the ramp's deep part ({window_depth:.1} d̂ > \
         {MIN_WINDOW_DEPTH_BANDS:.1} d̂) — contact cost rises with the patch, so \
         an early window UNDERSTATES `I`, the quantity the verdict turns on",
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
                probe: None,
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
        // PILOTED at 0.764–1.173× over 28 arm-comparisons across both
        // sizes. ⚠ An earlier comment said `0.80–1.17` — the low end was read
        // off a subset. The spread is wide for a control because `contact` is ~1 % of a full-order frame, so the
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

    // ⚠ BY NAME, not by position. `rows[1]` / `rows[2]` would silently swap the
    // two predictors' columns the day `GUESSES` is reordered, and the size table
    // would still look entirely reasonable.
    let iters_of = |want: &str| -> f64 {
        let (_, _, m, _) = rows
            .iter()
            .find(|(label, ..)| *label == want)
            .unwrap_or_else(|| panic!("no `{want}` arm in this fixture's rows"));
        m.iters as f64 / m.steps as f64
    };
    SizeRow {
        free_dof: fd.len(),
        full_iters: iters_of("FULL-ORDER"),
        prev_iters: iters_of("prev"),
        inertial_iters: iters_of("inertial"),
    }
}

// ── R1's open question: does the basis GENERALISE across contact positions? ──

/// Lateral indenter offsets the basis is TRAINED on, in patch radii.
const TRAIN_OFFSETS: [f64; 5] = [-1.0, -0.5, 0.0, 0.5, 1.0];

/// Offsets it is SCORED at: one inside the training set, one interpolating
/// between training points, and two extrapolating past the training edge.
///
/// The IN-SAMPLE row is the two-sided control — if it is not far better than the
/// rest, the rig is broken and no other row says anything.
///
/// ⚠ TWO extrapolation points, and the pair is the de-confound. `+2.00a` leaves
/// only `1a` of clearance to the plate's free edge, so a failure there mixes
/// "outside the training span" with "a different boundary regime". `+1.50a` is
/// half the extrapolation distance with `1.5a` of clearance; if both fail alike,
/// the free edge is not the cause.
const TEST_OFFSETS: [(&str, f64); 4] = [
    ("IN-SAMPLE  +0.00a", 0.0),
    ("INTERP     +0.25a", 0.25),
    ("EXTRAP     +1.50a", 1.5),
    ("EXTRAP     +2.00a", 2.0),
];

/// Requested ranks. Wide on purpose: the whole discriminator is whether error
/// FALLS with rank (a capacity problem, which rank fixes) or is FLAT in it (the
/// subspace is wrong, which no rank fixes).
const BASIS_RANKS: [usize; 4] = [20, 40, 80, 160];

/// 5 202 free DOF — R1.1's operating point, and cheap enough for the eight
/// full-order trajectories this needs (five training, three more to score
/// against).
const GEN_A_OVER_CELL: f64 = 2.0;

/// **R1's open question, and R3 is blocked on it.**
///
/// ```text
/// cargo test --release -p sim-soft --test reduced_contact \
///   reduced_basis_generalises -- --ignored --nocapture
/// ```
///
/// Every reduced measurement in this crate uses an IN-SAMPLE basis — fitted to
/// the trajectory it is then scored against. That was the right isolation for
/// "does the algebra survive a barrier", and it leaves the property R3 actually
/// depends on untested: that the basis represents scenes it was NOT trained on.
///
/// A ceiling raiser inherits its floor's soundness. If a global POD basis cannot
/// follow a contact patch as it MOVES, then hyper-reducing that basis is a faster
/// wrong answer, and the `1.4×` margin §2k measured is a number about the wrong
/// thing. This is the classic advection-like POD failure: a localised feature
/// that translates needs a rank explosion to represent, because each position
/// is nearly orthogonal to the last.
///
/// The indenter is swept LATERALLY, which on this fixture is the cleanest
/// available form of that question — and it is the same question a puck sliding
/// along a stick blade asks.
///
/// ## Pre-registration (before the first run)
///
/// 1. **In-sample reproduces** the earlier result — `~1e-7` rel-L2 at high rank.
///    If not, stop: the training set or the scoring is wired wrong.
/// 2. **Interpolation is usable** — somewhere in `1e-4 … 1e-2`. Five positions
///    over `±1a` is a fine grid relative to the patch width.
/// 3. **Extrapolation is where it breaks.** No number predicted.
/// 4. ★ **The discriminator is the RANK TREND, not any single error.** Falling
///    with rank ⇒ capacity, and more modes (or goal-oriented enrichment, plan
///    §14) fixes it. Flat in rank ⇒ the subspace is wrong for translated
///    contact, and R1 needs a different basis, not a bigger one.
///
/// How much better the IN-SAMPLE row must be than the INTERPOLATED one, at the
/// highest achieved rank, for the table to mean anything.
///
/// ★ A RATIO, not a floor. An absolute bound would have to sit near `1e-4`, and
/// in-sample reads `1.1e-4` at `r=80` — so a run whose top rank came out lower
/// would trip it spuriously. The ratio is scale-free and encodes the property
/// that actually matters: the rig can score a basis it DID fit far better than
/// one it did not. PILOTED at `734×` (`1.885e-3 / 2.569e-6`) at `r=142` and
/// `84.5×` (`9.484e-3 / 1.122e-4`) at `r=80`, so `10×` keeps one to two orders of
/// headroom and still fails long before the two rows become indistinguishable.
///
/// ⛔ **`86×` was published here and is `84.5×`** — corrected 2026-08-25. It was
/// computed from §2l's already-ROUNDED table (`9.5e-3 / 1.1e-4` = `86.4`) rather
/// than from the run. A second instance of the same defect §2l's own note
/// records, in the docstring that note points at. ⇒ **take a figure from the
/// highest-precision output the run produced, never from a printed table.**
const MIN_IN_SAMPLE_ADVANTAGE: f64 = 10.0;

/// ⚠ `+2.00a` leaves only `1a` of clearance to the plate's free edge, so its
/// deformation field differs in character as well as in position. A failure
/// there is not purely an extrapolation result — which is why `+1.50a` is also
/// scored, at half the distance with `1.5a` of clearance.
///
/// ## Measured — 2026-08-24, 5 202 free DOF, 355 training snapshots
///
/// rel-L2 against each position's OWN full-order oracle:
///
/// | position | r=20 | r=40 | r=80 | r=142 |
/// |---|---:|---:|---:|---:|
/// | IN-SAMPLE `+0.00a` | 2.4e-2 | 4.7e-3 | 1.1e-4 | **2.6e-6** |
/// | INTERP `+0.25a` | 3.2e-2 | 1.8e-2 | 9.5e-3 | **1.9e-3** |
/// | EXTRAP `+1.50a` | 3.1e-1 | 1.5e-1 | 3.2e-1 | **2.8e-1** |
/// | EXTRAP `+2.00a` | DIVERGED | 7.6e-1 | 1.0e0 | **1.1e0** |
///
/// ⚠ **`+1.50a` at `r=20` was published as `3.2e-1` and is `3.1e-1`**, corrected
/// 2026-08-25. The true value is `3.149e-1`; the discriminator below prints
/// `{:.2e}` = `3.15e-1`, and the table was rounded a SECOND time from that. A
/// figure derived from an already-rounded figure. The other 15 reproduce exactly.
///
/// - ★★★ **The subspace is wrong for a translated patch, and RANK DOES NOT FIX
///   IT.** In-sample buys four orders across the ladder. Both extrapolations are
///   FLAT in rank — `+1.50a` wanders `0.15–0.32` with no trend, `+2.00a` gets
///   WORSE. The predicted advection-like failure, confirmed on the clean point:
///   `+1.50a` has `1.5a` of edge clearance, so the free edge is exonerated and
///   extrapolation itself is the cause.
/// - ★★ **Even INTERPOLATION costs three orders.** Between training points
///   `0.5a` apart, `+0.25a` reaches `1.9e-3` where in-sample reaches `2.6e-6` at
///   the same rank, and it improves only ~1.2 orders across a 7× rank increase
///   against in-sample's four. The reduction advantage dies long before the
///   accuracy does.
/// - ★★★ **Out-of-domain is SILENT in 7 of the 8 arms.** Seven converge,
///   complete all 71 steps, and do not penetrate — `min_sd` stays positive and
///   inside the band — while being `14.8 %`–`109 %` wrong. Convergence plus
///   non-penetration is NOT a validity check. ⚠ The eighth (`+2.00a` at `r=20`)
///   stalls at step 65 and IS announced, which is the honest bound on the
///   finding: the failure mode is silent, not universally silent. `28 %` and
///   `109 %` are the `r=142` column, not the range.
/// - ★★ **`gap_dev` is**, and that was not what it was built for. Across all
///   eight failing arms it reads `0.161–0.583 d̂` against `3.1e-11` in-sample at
///   the same rank. ⚠ **But it is an ERROR indicator, not an in-domain one**: at
///   `r = 20` the IN-SAMPLE arm reads `1.6e-4`, over the threshold, because the
///   answer is genuinely poor there (`relL2 = 2.4e-2`). It conflates "outside
///   the hull" with "basis too small". Nor is it monotone in `relL2` within a
///   regime, so it separates regimes by orders of magnitude and ranks nothing
///   finer.
///
///   ⛔ **AND IT CANNOT BE THE §4c GATE — an earlier version of this list said
///   it arguably was.** `gap_dev` is `|min_sd − ORACLE min_sd|`
///   (see its field on [`Arm`]), so it differences against the full-order
///   answer: a runtime check that needs the solve it exists to avoid protects
///   nothing. It stays here as a diagnostic and as the reference column the
///   oracle-free candidates are measured against. The signals that CAN gate are
///   in [`an_online_signal_separates_out_of_domain`], which also confirms the
///   ERROR/in-domain conflation above in an oracle-free signal — which is why
///   the gate needs a DOMAIN signal beside it, not a better error one.
/// - ⇒ **This REVIVES `ReducedValidityDomain` (§4c) as a CORRECTNESS
///   prerequisite.** v2.7 retracted it as a performance prerequisite and that
///   retraction stands. But a reduced solver that returns a converged,
///   non-penetrating, 100 %-wrong answer outside its training hull cannot ship
///   without a domain gate.
#[test]
#[ignore = "R1 basis generalisation — ~5 min, run explicitly (see the fn docs)"]
fn reduced_basis_generalises() {
    let base = Scene::new(GEN_A_OVER_CELL);
    let probe = base.mesh();
    let x_rest = rest_positions(&probe);
    let solver = base.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let n_steps = base.n_steps();

    println!(
        "\nRC\tGENERALISATION: IPC indentation a/cell={GEN_A_OVER_CELL:.1}, {} free DOF, \
         {n_steps} steps/trajectory\nRC\ttrain offsets {TRAIN_OFFSETS:?} a  |  patch radius \
         a = {:.3e} m, plate = 8a",
        fd.len(),
        patch_radius(),
    );

    // ── one full-order trajectory per TRAINING offset; snapshots pooled ──
    let mut train = SnapshotSet::new(fd.len());
    let mut train_oracles = Vec::new();
    for dx in TRAIN_OFFSETS {
        let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
        let o = run_oracle(sc, &x_rest, NO_TIMING);
        assert!(
            o.arm.failure.is_none(),
            "training trajectory at {dx:+.2}a failed: {}",
            o.arm.failure.as_deref().unwrap_or(""),
        );
        for x in &o.x {
            train.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
        }
        train_oracles.push((dx, o));
    }
    println!("RC\ttraining set: {} snapshots", train.len());

    // ── score at each test offset ──
    // ★ Fitted ONCE per rank, outside the scoring loop. The basis is a function
    // of the training set and the rank only — refitting it per test offset costs
    // 16 eigendecompositions where 4 do, and reads as though the subspace
    // depended on the point being scored, which is the opposite of the claim.
    let mut bases = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for r in BASIS_RANKS {
        let fitted = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, r).expect("basis fits");
        if seen.insert(fitted.n_modes()) {
            bases.push((r, fitted));
        }
    }
    println!(
        "RC\tbases: {}",
        bases
            .iter()
            .map(|(r, b)| format!("r={r}→{}", b.n_modes()))
            .collect::<Vec<_>>()
            .join(", ")
    );

    let mut rows: Vec<(&str, usize, Arm)> = Vec::new();
    for (label, dx) in TEST_OFFSETS {
        let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
        // Reuse the training oracle when the test point IS a training point —
        // that guarantees the in-sample control is scored against exactly the
        // trajectory its snapshots came from, rather than a lookalike re-run.
        let owned;
        let oracle =
            if let Some((_, o)) = train_oracles.iter().find(|(t, _)| (t - dx).abs() < 1e-12) {
                o
            } else {
                owned = run_oracle(sc, &x_rest, NO_TIMING);
                assert!(
                    owned.arm.failure.is_none(),
                    "scoring oracle at {dx:+.2}a failed: {}",
                    owned.arm.failure.as_deref().unwrap_or(""),
                );
                &owned
            };

        for (r, basis) in &bases {
            let got = basis.n_modes();
            let arm = run_reduced(
                sc,
                basis,
                InitialGuess::Inertial,
                format!("{label}  r={r}→{got}"),
                &Ctx {
                    x_rest: &x_rest,
                    fd: &fd,
                    oracle,
                    probe: None,
                },
                NO_TIMING,
            );
            arm.print(n_steps, sc.d_hat);
            rows.push((label, got, arm));
        }
    }

    // ── the discriminator ──
    println!("\nRC\t╔═ does rank fix it? (rel-L2 vs each position's OWN full-order oracle)");
    for (label, _) in TEST_OFFSETS {
        let mut line = format!("RC\t║ {label:<18}");
        for (l, got, arm) in &rows {
            if *l == label {
                let e = if arm.failure.is_some() {
                    "   DIVERGED".to_owned()
                } else {
                    format!(" r{got:<3}{:>9.2e}", arm.max_rel_err)
                };
                line.push_str(&e);
            }
        }
        println!("{line}");
    }
    println!(
        "RC\t║ FALLING with rank ⇒ capacity, and more modes fix it.\n\
         RC\t║ FLAT in rank ⇒ the subspace is wrong for a translated patch, and\n\
         RC\t║ no rank fixes it — R1 would need a different basis, not a bigger one."
    );
    println!("RC\t╚═");

    // ── the only assertion: the two-sided control ──
    //
    // The held-out rows are the MEASUREMENT; a bad number there is the finding,
    // not a test failure. What must hold is that the rig can score a basis it
    // did fit — otherwise every row is measuring the harness.
    let control = top_row(&rows, TEST_OFFSETS[0].0);
    assert!(
        control.2.failure.is_none() && control.2.min_sd > 0.0,
        "the IN-SAMPLE control did not survive its own trajectory — nothing above \
         is interpretable",
    );
    let advantage = in_sample_advantage(&rows, TEST_OFFSETS[0].0, TEST_OFFSETS[1].0);
    println!(
        "RC\t★ control: in-sample is {advantage:.0}× better than interpolation at \
         r={} (floor {MIN_IN_SAMPLE_ADVANTAGE:.0}×)",
        control.1,
    );
    assert!(
        advantage >= MIN_IN_SAMPLE_ADVANTAGE,
        "in-sample is only {advantage:.1}× better than interpolation — the rig \
         cannot tell a basis it FIT from one it did not, so the generalisation \
         reading above is not supported",
    );
}

/// The highest-rank row recorded for `label`.
fn top_row<'a>(
    rows: &'a [(&'static str, usize, Arm)],
    label: &str,
) -> &'a (&'static str, usize, Arm) {
    rows.iter()
        .filter(|(l, ..)| *l == label)
        .max_by_key(|(_, got, _)| *got)
        .unwrap_or_else(|| panic!("no rows recorded for `{label}`"))
}

/// How many times better the in-sample row is than the interpolated one, with
/// the three guards that make the number mean something.
///
/// ★ Extracted from the test body ONLY so those guards can be exercised. A
/// control that has run exclusively on good data is not a control — and the
/// first of these is the exact shape this file keeps producing: the ratio's
/// denominator is `max_rel_err`, which INITIALISES TO ZERO and is only updated
/// for finite samples, so a run that recorded no usable error at all would
/// divide by zero, report `inf×`, and sail past the floor having measured
/// nothing.
fn in_sample_advantage(
    rows: &[(&'static str, usize, Arm)],
    in_label: &str,
    interp_label: &str,
) -> f64 {
    let (_, fitted_rank, fitted) = top_row(rows, in_label);
    let (_, held_rank, held) = top_row(rows, interp_label);
    assert_eq!(
        fitted_rank, held_rank,
        "the two rows were scored at different ranks, so their ratio is not a \
         generalisation measurement",
    );
    assert!(
        held.failure.is_none(),
        "the interpolation arm did not complete ({}), so its error is over a \
         TRUNCATED trajectory — a small ratio would then indict the basis when \
         the arm simply stopped early",
        held.failure.as_deref().unwrap_or(""),
    );
    assert!(
        fitted.max_rel_err.is_finite() && fitted.max_rel_err > 0.0,
        "in-sample error is {:.3e}; a zero or non-finite denominator makes the \
         ratio `inf` and the control passes having measured NOTHING",
        fitted.max_rel_err,
    );
    held.max_rel_err / fitted.max_rel_err
}

// ── §4c rung 1: is there an ONLINE signal, and does it separate the two ways ──
// ── a reduced answer can be wrong?                                           ──

/// The candidate online validity signals, worst-over-steps, on one reduced arm.
///
/// ★★ **Every one is computable ONLINE.** No full-order oracle appears in any of
/// them, and that is the whole point — it is exactly what `gap_dev` is not.
/// [`Arm::gap_dev`] differences `min_sd` against the ORACLE's `min_sd`
/// (`run_reduced`), so however well it detects the failure, it is a *diagnostic*
/// and can never be a *gate*: a runtime check cannot run the full-order solve it
/// exists to avoid.
///
/// ★ They are deliberately of two KINDS, because the thing §4c has to
/// disentangle is that a reduced answer can be wrong for two unrelated reasons:
///
/// - an **ERROR** signal rises when the answer is poor, from any cause;
/// - a **DOMAIN** signal rises when the scene is outside what the ensemble saw,
///   whether or not the answer is poor.
///
/// `gap_dev` is an error signal, which is why §2l found it reading `1.6e-4` on
/// the IN-SAMPLE arm at `r = 20` — in domain, and simply solved badly. A gate
/// built on an error signal alone cannot say which of the two it caught, and
/// the two have opposite remedies (more rank vs. more training).
#[derive(Clone, Copy)]
struct Signals {
    /// `‖r_free‖ / tol` — the residual the Galerkin condition cannot see, in
    /// units of the tolerance the solve declares itself converged at. **ERROR.**
    ///
    /// Free: `r_free` is already assembled at the convergence check and
    /// [`ReducedStep::full_residual_norm`] already returns its norm. That
    /// field's own docs state this study's question — "no useful bound on it is
    /// known yet" — and `tol` is the natural unit, since `‖Φᵀr‖ < tol` is what
    /// the solve stopped on, so the ratio reads as *how many tolerances of
    /// residual the basis is blind to*. §2c measured `1.49e-4` against a `1e-10`
    /// tol in-sample at `r = 10`, i.e. `1.5e6`, so the scale is large and the
    /// column is read in orders of magnitude.
    ///
    /// ⚠ **It is NOT normalised by the load.** `‖r_free‖` is a force norm, so
    /// the worst-over-steps reading is taken at the deepest pose, where the
    /// barrier force is largest. That makes the column comparable ACROSS arms —
    /// every one runs the same `z` ramp over the same `71` poses — and it does
    /// NOT make a single cell comparable to anything outside this fixture. A
    /// shippable threshold would have to divide by something; this pilot's job
    /// is to say whether the quantity separates at all.
    residual_excess: f64,
    /// How far `q` lies outside the training envelope, in half-widths of that
    /// envelope, worst over modes. Zero inside the box. **DOMAIN.**
    ///
    /// §4c's STATED signal — "a bound on reduced-coordinate magnitude `‖q‖`
    /// relative to the training envelope". Recon §5 lists "the `‖q‖` gate does
    /// not detect that the indenter moved somewhere the ensemble never saw"
    /// under *honestly open research*: ARGUED, never measured. This measures it.
    ///
    /// ⚠ A per-mode box is the LOOSE hull — the bounding box of the training
    /// coordinates, not their convex hull — so a translated patch can sit inside
    /// it while being far from every training point. That looseness is not an
    /// oversight; it is the predicted failure mode, and [`Signals::snapshot_distance`]
    /// is the tight companion that says whether looseness is the cause.
    envelope_excursion: f64,
    /// Distance from `q` to the NEAREST training snapshot, in units of the
    /// training cloud's own radius. **DOMAIN.**
    ///
    /// The tight hull, paired with `envelope_excursion` so that a box that fails
    /// to separate can be told apart from a `q`-space that fails to separate. If
    /// the box is flat across positions and this is not, the signal is real and
    /// the box was merely the wrong statistic.
    ///
    /// ★ Normalised by the cloud RADIUS (RMS spread about the mean), not by the
    /// nearest-neighbour pitch. On a ramp the nearest neighbour of any snapshot
    /// is its own predecessor one time step earlier, so a pitch normaliser would
    /// measure `dt` and not the offset spacing the ensemble was built on.
    snapshot_distance: f64,
    /// Fraction of contact-active vertices that were active in NO training
    /// trajectory. **DOMAIN.**
    ///
    /// ★ The one candidate that barely involves the basis: it is a statement
    /// about where the collider is, not about how well the subspace resolves it.
    /// That gives it a falsifiable signature the others do not have — it should
    /// be **FLAT IN RANK** and **STEPPED IN POSITION**. If it moves strongly
    /// with rank it is contaminated by the solution and is not the
    /// basis-independent domain check it claims to be.
    active_novelty: f64,
}

impl Signals {
    /// The four values, in table order, paired with their names for reporting.
    const fn named(&self) -> [(&'static str, f64); 4] {
        [
            ("residual_excess", self.residual_excess),
            ("envelope_excursion", self.envelope_excursion),
            ("snapshot_distance", self.snapshot_distance),
            ("active_novelty", self.active_novelty),
        ]
    }

    /// ★ Checked on the way IN, not on the way out. Every summary below is a
    /// fold, and a non-finite sample would either be swallowed by `min`/`max` or
    /// poison a percentile — either way the arm would report the wrong step as
    /// its worst case.
    fn assert_finite(&self) {
        for (name, v) in self.named() {
            assert!(
                v.is_finite(),
                "{name} read {v} on a converged step — a non-finite sample cannot \
                 be summarised, and a fold would hide it rather than surface it",
            );
        }
    }
}

/// Every step's reading for one arm, kept WHOLE.
///
/// ⚠⚠ **This replaces a worst-over-steps accumulator, and the reason is the
/// central limitation of the first pilot.** A max over the trajectory is an
/// extreme-value statistic; a runtime gate reads ONE step. If a signal's max is
/// a spike at the deepest pose, a per-step gate fires late or not at all; if it
/// is a plateau, the trajectory statistic transfers. Nothing in a max-only table
/// can tell those apart, so the summary is no longer the measurement — the
/// distribution is, and the max is one quantile of it.
struct SignalTrace {
    steps: Vec<Signals>,
    /// How many vertices were contact-active at each step — `active_novelty`'s
    /// DENOMINATOR, without which the fraction cannot be read at all.
    active_count: Vec<usize>,
}

impl SignalTrace {
    const fn new() -> Self {
        Self {
            steps: Vec::new(),
            active_count: Vec::new(),
        }
    }

    fn record(&mut self, step: Signals, active: usize) {
        step.assert_finite();
        self.steps.push(step);
        self.active_count.push(active);
    }

    /// An arm that recorded NOTHING has an EMPTY trace, which no summary can be
    /// taken of. That is deliberate: every domain signal's healthy reading is
    /// zero, so "never measured" must not be representable as a number.
    const fn is_recorded(&self) -> bool {
        !self.steps.is_empty()
    }

    /// `(min, median, max)` of one component over the steps recorded.
    ///
    /// ★ The median is the point of this whole structure. Reported beside the
    /// max so a spike and a plateau are distinguishable, which is the difference
    /// between a trajectory statistic and something a runtime gate could use.
    fn spread(&self, pick: fn(&Signals) -> f64) -> (f64, f64, f64) {
        assert!(
            self.is_recorded(),
            "spread over an EMPTY trace — there is no worst step, and returning \
             zero would read as a perfectly in-domain arm",
        );
        let mut v: Vec<f64> = self.steps.iter().map(pick).collect();
        v.sort_by(f64::total_cmp);
        (v[0], v[v.len() / 2], v[v.len() - 1])
    }

    /// The active-set size range — `active_novelty`'s denominator.
    fn active_range(&self) -> (usize, usize) {
        assert!(self.is_recorded(), "active range over an EMPTY trace");
        (
            self.active_count.iter().copied().min().unwrap_or(0),
            self.active_count.iter().copied().max().unwrap_or(0),
        )
    }
}

/// What the training ensemble SAW — the reference every online signal is scored
/// against.
///
/// Built once per basis and shared by every arm at that rank, so two arms cannot
/// be scored against different hulls, which is the same reason [`Ctx`] groups
/// the oracle with the rest configuration.
struct Probe<'a> {
    /// Per-mode `[lo, hi]` of `q = ΦᵀMu` over every training snapshot.
    envelope: Vec<(f64, f64)>,
    /// Every training snapshot's reduced coordinates — the tight hull.
    train_q: Vec<Vec<f64>>,
    /// RMS spread of `train_q` about its mean; `snapshot_distance`'s unit.
    cloud_radius: f64,
    /// Every vertex contact-active at any step of any training trajectory.
    train_active: &'a std::collections::BTreeSet<usize>,
    /// The solve's convergence tolerance — `residual_excess`'s unit.
    tol: f64,
    /// The barrier band, which is what "contact-active" means.
    d_hat: f64,
}

impl Probe<'_> {
    /// One step's four readings.
    fn read(&self, s: &ReducedStep, x: &[f64], centre: Vec3) -> (Signals, usize) {
        let active = active_vertices(x, centre, self.d_hat);
        let reading = Signals {
            residual_excess: s.full_residual_norm / self.tol,
            envelope_excursion: envelope_excursion(&s.q, &self.envelope),
            snapshot_distance: nearest_training_distance(&s.q, &self.train_q) / self.cloud_radius,
            active_novelty: active_novelty(&active, self.train_active),
        };
        (reading, active.len())
    }
}

/// A mode with no variation across training supplies no scale of its own. Floor
/// its half-width at this fraction of the WIDEST mode's, so an excursion there
/// reads as enormous rather than as `inf` — which [`Signals::record`] would
/// refuse, killing the whole run over one dead mode at the tail of the spectrum.
const DEGENERATE_MODE_FLOOR: f64 = 1.0e-12;

/// Per-mode `[min, max]` of the training snapshots' reduced coordinates.
fn training_envelope(train_q: &[Vec<f64>]) -> Vec<(f64, f64)> {
    assert!(
        !train_q.is_empty(),
        "an envelope over ZERO training snapshots would be empty, and \
         `envelope_excursion` would then fold over nothing and return 0 — every \
         arm perfectly in domain, having measured nothing",
    );
    let r = train_q[0].len();
    (0..r)
        .map(|i| {
            train_q
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
                    assert_eq!(t.len(), r, "training snapshots have inconsistent rank");
                    // ⚠ `f64::min`/`max` SWALLOW NaN — the trap `min_signed_distance`
                    // carries its own guard for. A non-finite training coordinate
                    // would drop silently out of the envelope, NARROWING it, and
                    // every excursion measured against it would then read high.
                    assert!(
                        t[i].is_finite(),
                        "training coordinate {i} is {}, which the min/max fold \
                         would have swallowed",
                        t[i],
                    );
                    (lo.min(t[i]), hi.max(t[i]))
                })
        })
        .collect()
}

/// How far `q` lies outside `envelope`, in half-widths, worst over modes.
///
/// Zero inside the box — which is the healthy reading, and the reason
/// [`Signals::UNRECORDED`] starts at `NaN` rather than at zero.
fn envelope_excursion(q: &[f64], envelope: &[(f64, f64)]) -> f64 {
    assert_eq!(
        q.len(),
        envelope.len(),
        "q has {} entries against an envelope of {} modes — the two were built \
         from different bases and the comparison is meaningless",
        q.len(),
        envelope.len(),
    );
    let widest = envelope
        .iter()
        .map(|&(lo, hi)| (hi - lo) / 2.0)
        .fold(0.0_f64, f64::max);
    assert!(
        widest > 0.0,
        "every mode has a zero-width training envelope — the training set is a \
         single point, or was projected onto the wrong basis",
    );
    let floor = DEGENERATE_MODE_FLOOR * widest;
    // ⚠ The per-element `.max(0.0)` and the fold's `0.0` seed are MUTUALLY
    // REDUNDANT, and that is measured, not assumed: deleting either one alone
    // survives the whole suite, deleting BOTH is killed by
    // `envelope_excursion_is_zero_inside_the_box`. Kept as a pair on purpose —
    // the clamp says "inside the box contributes nothing" and the seed says "no
    // modes, no excursion" — but a future edit must not read either as free.
    q.iter()
        .zip(envelope)
        .map(|(&v, &(lo, hi))| (lo - v).max(v - hi).max(0.0) / ((hi - lo) / 2.0).max(floor))
        .fold(0.0_f64, f64::max)
}

/// Distance from `q` to the nearest training snapshot, in `q`-space.
///
/// ★ Plain Euclidean, and that is not an approximation: the basis is fitted
/// under [`Inner::Mass`], so `ΦᵀMΦ = I` and the Euclidean distance between two
/// coordinate vectors IS the mass-norm distance between the displacements they
/// reconstruct. Any other metric here would be measuring something the solve
/// does not use.
fn nearest_training_distance(q: &[f64], train_q: &[Vec<f64>]) -> f64 {
    assert!(
        !train_q.is_empty(),
        "no training snapshots, so the nearest one is `inf` and the arm reads \
         maximally out of domain for a reason that is about the rig",
    );
    train_q
        .iter()
        .map(|t| {
            assert_eq!(
                t.len(),
                q.len(),
                "training snapshot has a different rank than q"
            );
            q.iter()
                .zip(t)
                .map(|(a, b)| (a - b) * (a - b))
                .sum::<f64>()
                .sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}

/// RMS spread of the training coordinates about their mean — the cloud's radius.
fn cloud_radius(train_q: &[Vec<f64>]) -> f64 {
    assert!(
        !train_q.is_empty(),
        "no training snapshots to take a radius of"
    );
    let r = train_q[0].len();
    let n = train_q.len() as f64;
    let mut mean = vec![0.0; r];
    for t in train_q {
        assert_eq!(t.len(), r, "training snapshots have inconsistent rank");
        for (m, v) in mean.iter_mut().zip(t) {
            *m += v / n;
        }
    }
    let sq: f64 = train_q
        .iter()
        .map(|t| {
            t.iter()
                .zip(&mean)
                .map(|(a, m)| (a - m) * (a - m))
                .sum::<f64>()
        })
        .sum();
    (sq / n).sqrt()
}

/// Vertices within the barrier band `d̂` of the collider — what "contact-active"
/// means to IPC, and computed from the sphere's geometry HERE for the same
/// reason [`min_signed_distance`] is: a set read out of the contact model would
/// share a failure mode with the thing being checked.
fn active_vertices(x: &[f64], centre: Vec3, d_hat: f64) -> std::collections::BTreeSet<usize> {
    x.chunks_exact(3)
        .enumerate()
        .filter(|(_, c)| (Vec3::new(c[0], c[1], c[2]) - centre).norm() - RADIUS < d_hat)
        .map(|(i, _)| i)
        .collect()
}

/// Fraction of `active` that appears in no training trajectory.
///
/// ⚠ An EMPTY active set reads `0.0` — nothing is touching, so nothing is novel.
/// That is right for a max-fold over a ramp whose first steps have the sphere a
/// full `1.2 d̂` clear of the surface, and it means the column is driven by the
/// deep steps, which is where the question lives.
fn active_novelty(
    active: &std::collections::BTreeSet<usize>,
    seen: &std::collections::BTreeSet<usize>,
) -> f64 {
    if active.is_empty() {
        return 0.0;
    }
    active.iter().filter(|v| !seen.contains(v)).count() as f64 / active.len() as f64
}

// ── the signal helpers' two-sided controls ────────────────────────────────
//
// ★ Every one of these is a fold or a set operation whose HEALTHY reading is
// `0.0`, on a study whose whole question is whether a non-zero reading means
// anything. A helper that silently returns zero is therefore indistinguishable
// from a helper that works, on every arm that passes — so the "it can read
// non-zero" half is exercised here rather than left to a measurement run that
// may never produce one.

#[test]
fn envelope_excursion_is_zero_inside_the_box() {
    let env = [(-1.0, 1.0), (0.0, 4.0)];
    assert!(envelope_excursion(&[0.0, 2.0], &env).abs() < f64::EPSILON);
    // The boundary belongs to the box.
    assert!(envelope_excursion(&[1.0, 0.0], &env).abs() < f64::EPSILON);
}

#[test]
fn envelope_excursion_counts_half_widths_and_is_two_sided() {
    // Half-width 1 on mode 0: two half-widths past either end reads 2.
    let env = [(-1.0, 1.0), (0.0, 4.0)];
    assert!((envelope_excursion(&[3.0, 2.0], &env) - 2.0).abs() < 1e-12);
    assert!((envelope_excursion(&[-3.0, 2.0], &env) - 2.0).abs() < 1e-12);
    // Worst over modes, not the first: mode 1's half-width is 2, so `10.0` is
    // three half-widths out and must beat mode 0's zero.
    assert!((envelope_excursion(&[0.0, 10.0], &env) - 3.0).abs() < 1e-12);
}

#[test]
fn envelope_excursion_floors_a_dead_mode_instead_of_returning_inf() {
    // A mode that never moved in training has no scale of its own. Dividing by
    // its true zero width gives `inf`, which `Signals::record` refuses — one
    // dead mode at the tail of the spectrum would then kill the whole run.
    let got = envelope_excursion(&[1.0e-6, 0.0], &[(0.0, 0.0), (-1.0, 1.0)]);
    assert!(
        got.is_finite(),
        "a dead mode produced {got}, not a finite reading"
    );
    assert!(
        got > 1.0e5,
        "the dead mode must still read as a large excursion, got {got}"
    );
}

#[test]
#[should_panic(expected = "different bases")]
fn envelope_excursion_refuses_a_rank_mismatch() {
    let _ = envelope_excursion(&[0.0, 0.0, 0.0], &[(-1.0, 1.0)]);
}

#[test]
#[should_panic(expected = "single point")]
fn envelope_excursion_refuses_an_all_zero_width_envelope() {
    let _ = envelope_excursion(&[1.0], &[(0.0, 0.0)]);
}

#[test]
#[should_panic(expected = "ZERO training snapshots")]
fn training_envelope_refuses_an_empty_training_set() {
    let _ = training_envelope(&[]);
}

/// ★ Added because a MUTATION round killed nothing when this guard was deleted.
/// It was written in review round 1, shipped, and then survived its own mutant —
/// a guard with no negative control is not a guard, however right it looks.
#[test]
#[should_panic(expected = "min/max fold")]
fn training_envelope_refuses_a_non_finite_coordinate() {
    let _ = training_envelope(&[vec![1.0, f64::NAN], vec![2.0, 3.0]]);
}

#[test]
fn training_envelope_brackets_every_snapshot() {
    let train = vec![vec![1.0, -3.0], vec![-2.0, 0.0], vec![0.5, 5.0]];
    let env = training_envelope(&train);
    assert_eq!(env, vec![(-2.0, 1.0), (-3.0, 5.0)]);
    // The defining property, checked through the consumer: a training point is
    // in its own envelope by construction, and if it is not the two disagree.
    for t in &train {
        assert!(envelope_excursion(t, &env).abs() < f64::EPSILON);
    }
}

#[test]
fn nearest_training_distance_takes_the_nearest_not_the_first() {
    let train = vec![vec![10.0, 0.0], vec![0.0, 1.0], vec![-10.0, 0.0]];
    assert!((nearest_training_distance(&[0.0, 0.0], &train) - 1.0).abs() < 1e-12);
    assert!(nearest_training_distance(&train[0], &train).abs() < f64::EPSILON);
}

#[test]
#[should_panic(expected = "no training snapshots")]
fn nearest_training_distance_refuses_an_empty_training_set() {
    let _ = nearest_training_distance(&[0.0], &[]);
}

#[test]
fn cloud_radius_is_the_rms_spread_about_the_mean() {
    // Mean `(1, 0)`, each point one unit away ⇒ radius exactly 1.
    assert!((cloud_radius(&[vec![0.0, 0.0], vec![2.0, 0.0]]) - 1.0).abs() < 1e-12);
    // A single point has no spread — which is a real reading, not an error, and
    // the caller is what must refuse to divide by it.
    assert!(cloud_radius(&[vec![3.0, 4.0]]).abs() < f64::EPSILON);
}

#[test]
fn active_vertices_takes_the_barrier_band_and_not_the_surface() {
    let d_hat = 1.0e-5;
    let centre = Vec3::new(0.0, 0.0, 0.0);
    // Three vertices along +x: inside the sphere, inside the band, outside it.
    let x = vec![
        RADIUS * 0.5,
        0.0,
        0.0,
        RADIUS + 0.5 * d_hat,
        0.0,
        0.0,
        RADIUS + 1.5 * d_hat,
        0.0,
        0.0,
    ];
    let got = active_vertices(&x, centre, d_hat);
    assert_eq!(got, [0usize, 1].into_iter().collect(), "got {got:?}");
}

#[test]
fn active_novelty_is_two_sided() {
    let seen: std::collections::BTreeSet<usize> = [1usize, 2, 3].into_iter().collect();
    let subset: std::collections::BTreeSet<usize> = [2usize, 3].into_iter().collect();
    let disjoint: std::collections::BTreeSet<usize> = [7usize, 8].into_iter().collect();
    let half: std::collections::BTreeSet<usize> = [3usize, 9].into_iter().collect();
    assert!(active_novelty(&subset, &seen).abs() < f64::EPSILON);
    assert!((active_novelty(&disjoint, &seen) - 1.0).abs() < f64::EPSILON);
    assert!((active_novelty(&half, &seen) - 0.5).abs() < f64::EPSILON);
    // Nothing touching ⇒ nothing novel, and it must not be `NaN` — a `0/0` here
    // would be swallowed by the max-fold in `Signals::record`.
    assert!(active_novelty(&std::collections::BTreeSet::new(), &seen).abs() < f64::EPSILON);
}

#[cfg(test)]
const fn step(
    residual_excess: f64,
    envelope_excursion: f64,
    snapshot_distance: f64,
    active_novelty: f64,
) -> Signals {
    Signals {
        residual_excess,
        envelope_excursion,
        snapshot_distance,
        active_novelty,
    }
}

#[test]
fn an_unrecorded_arm_does_not_read_as_in_domain() {
    // ★★ THE control for this whole instrument. Every domain signal's healthy
    // reading is `0.0`, so an accumulator initialised to zero and never fed
    // would report a perfectly in-domain arm. An empty trace must be
    // distinguishable from a clean one, and must not be summarisable at all.
    let empty = SignalTrace::new();
    assert!(!empty.is_recorded());
    let mut t = SignalTrace::new();
    t.record(step(1.0e6, 0.0, 0.25, 0.0), 7);
    assert!(t.is_recorded());
}

#[test]
#[should_panic(expected = "EMPTY trace")]
fn an_empty_trace_refuses_to_be_summarised() {
    // ⚠ The dangerous alternative is a summary that returns `0.0` — which on a
    // domain signal reads as PERFECTLY IN DOMAIN.
    let _ = SignalTrace::new().spread(|s| s.snapshot_distance);
}

#[test]
fn the_trace_reports_a_spread_and_not_just_its_worst_step() {
    // ★ The whole reason the trace replaced a max-only accumulator: a SPIKE and
    // a PLATEAU have the same max and completely different meanings for a check
    // that reads one step.
    let mut spike = SignalTrace::new();
    let mut plateau = SignalTrace::new();
    for i in 0..5 {
        spike.record(step(1.0, 0.0, if i == 4 { 10.0 } else { 0.1 }, 0.0), 7);
        plateau.record(step(1.0, 0.0, f64::from(i).mul_add(-0.01, 10.0), 0.0), 7);
    }
    let (s_min, s_med, s_max) = spike.spread(|s| s.snapshot_distance);
    let (p_min, p_med, p_max) = plateau.spread(|s| s.snapshot_distance);
    assert!(
        (s_max - 10.0).abs() < 1e-12 && (p_max - 10.0).abs() < 1e-12,
        "same max"
    );
    assert!(
        (s_med - 0.1).abs() < 1e-12,
        "spike median is the floor, got {s_med}"
    );
    assert!(p_med > 9.9, "plateau median is near the max, got {p_med}");
    assert!((s_min - 0.1).abs() < 1e-12 && p_min > 9.9);
}

#[test]
fn the_trace_reports_the_active_set_size() {
    // `active_novelty` is a FRACTION; without its denominator a reading of
    // `1.000` cannot be told from `1/1`.
    let mut t = SignalTrace::new();
    t.record(step(1.0, 0.0, 0.0, 0.0), 4);
    t.record(step(1.0, 0.0, 0.0, 1.0), 19);
    assert_eq!(t.active_range(), (4, 19));
}

#[test]
#[should_panic(expected = "cannot be summarised")]
fn a_trace_refuses_a_non_finite_sample() {
    SignalTrace::new().record(step(f64::NAN, 0.0, 0.0, 0.0), 7);
}

/// **Can ONE threshold on this signal implement `accept ⟺ max_rel_err ≤ τ`?**
///
/// ★★★ **The question a gate is actually asked — and the first version of this
/// pilot did not ask it.** It asked whether a signal separates the TRAINING SPAN
/// from outside it. That is a different partition, and it flatters any signal
/// that merely tracks the indenter's position, because on this fixture position
/// and error happen to be monotone together. A gate does not refuse an answer for
/// being far from home; it refuses one for being too WRONG.
///
/// So: sweep τ, and for each signal ask whether the accept side's worst reading
/// is below the refuse side's best. A `FAIL` means no single threshold exists at
/// that τ — the classes overlap or invert.
///
/// ⚠ Every margin printed here is FITTED, not validated: the threshold interval
/// is derived from the same 16 cells it is scored on, and there is no held-out
/// position anywhere in this matrix.
fn print_gate_table(rows: &[(&'static str, usize, Arm)], taus: &[f64], q: Quantile) {
    let cells: Vec<(f64, &Arm)> = rows
        .iter()
        .filter(|(_, _, a)| a.max_rel_err > 0.0)
        .map(|(_, _, a)| (a.max_rel_err, a))
        .collect();
    assert!(
        cells.len() == rows.len(),
        "{} of {} arms have no usable error and cannot be placed on either side \
         of an accuracy threshold",
        rows.len() - cells.len(),
        rows.len(),
    );
    let picks: [(&str, fn(&Signals) -> f64); 4] = [
        ("S1", |s| s.residual_excess),
        ("S2", |s| s.envelope_excursion),
        ("S3", |s| s.snapshot_distance),
        ("S4", |s| s.active_novelty),
    ];
    println!(
        "\nRC\t╔═ GATE TEST ({}) — does ONE threshold give `accept ⟺ max_rel_err ≤ τ` ?",
        match q {
            Quantile::Max => "worst step — the decision a fire-if-ever gate reaches",
            Quantile::Median => "median step — how much of the trajectory SUPPORTS that decision",
        }
    );
    let mut head = format!("RC\t║ {:>9}", "τ");
    for (n, _) in &picks {
        let padded = format!("{n:>20}");
        head.push_str(&padded);
    }
    println!("{head}");
    for &tau in taus {
        let mut line = format!("RC\t║ {tau:>9.0e}");
        for (_, pick) in picks {
            let side = |keep: bool| -> Vec<f64> {
                cells
                    .iter()
                    .filter(|(e, _)| (*e <= tau) == keep)
                    .filter_map(|(_, a)| signal(a, pick, q))
                    .collect()
            };
            let (acc, refu) = (side(true), side(false));
            let cell = if acc.is_empty() || refu.is_empty() {
                "(one side empty)".to_owned()
            } else {
                let hi = acc.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                let lo = refu.iter().copied().fold(f64::INFINITY, f64::min);
                if hi >= lo {
                    "FAIL".to_owned()
                } else if hi > 0.0 {
                    format!("OK {:.2}x", lo / hi)
                } else {
                    "OK inf".to_owned()
                }
            };
            let padded = format!("{cell:>20}");
            line.push_str(&padded);
        }
        println!("{line}");
    }
    println!(
        "RC\t║ A signal that FAILs at some τ cannot gate on accuracy there, however \
         well\nRC\t║ it separates positions. ⚠ FITTED margins — no held-out position exists.\nRC\t╚═"
    );
}

/// One quantity's matrix: positions down, rank across. The LAYOUT is the
/// argument — an error signal and a domain signal are told apart by their SHAPE,
/// not by any single cell, and only a table shows shape.
///
/// `pick` returns `None` for a cell that was never measured. That is NOT
/// printed as a zero: every domain signal's healthy reading IS zero, so a blank
/// rendered as `0.0` would be a missing measurement disguised as a clean bill of
/// health — the same trap [`Signals::UNRECORDED`] exists for.
fn print_matrix(
    title: &str,
    note: &str,
    rows: &[(&'static str, usize, Arm)],
    pick: impl Fn(&Arm) -> Option<f64>,
) {
    println!("\nRC\t╔═ {title}");
    for (label, _) in TEST_OFFSETS {
        let mut line = format!("RC\t║ {label:<18}");
        for (l, got, arm) in rows {
            if *l != label {
                continue;
            }
            // Bound rather than pushed inline: `push_str(&format!(..))` is a
            // clippy `format_push_string`, and this file's other table builder
            // binds for the same reason.
            let cell = pick(arm).map_or_else(
                || format!(" r{got:<4}{:>10}", "—"),
                |v| format!(" r{got:<4}{v:>10.3e}"),
            );
            line.push_str(&cell);
        }
        println!("{line}");
    }
    println!("RC\t║ {note}\nRC\t╚═");
}

/// Which quantile of a signal's per-step distribution a table is reporting.
///
/// ★ The pilot's first version reported only [`Quantile::Max`], and that is an
/// extreme-value statistic where the conclusion is about a check that reads ONE
/// step. Reporting the median beside it is what makes a spike distinguishable
/// from a plateau.
#[derive(Clone, Copy)]
enum Quantile {
    Median,
    Max,
}

/// One signal off an arm at one quantile, or `None` if the arm recorded nothing.
fn signal(arm: &Arm, pick: fn(&Signals) -> f64, q: Quantile) -> Option<f64> {
    let t = arm.signals.as_ref().filter(|s| s.is_recorded())?;
    let (_, med, max) = t.spread(pick);
    Some(match q {
        Quantile::Median => med,
        Quantile::Max => max,
    })
}

/// **§4c rung 1 — is there an ONLINE signal, and does it separate the two ways
/// a reduced answer can be wrong?**
///
/// ```text
/// cargo test --release -p sim-soft --test reduced_contact \
///   an_online_signal_separates_out_of_domain -- --ignored --nocapture
/// ```
///
/// ## Why this is the next thing after [`reduced_basis_generalises`]
///
/// §2l established that a reduced solve outside its training hull **fails
/// silently**: 7 of its 8 extrapolation arms converge, complete all `71` steps,
/// and do not penetrate, while being `14.8 %`–`109 %` wrong. Convergence plus
/// non-penetration is not a validity check, so `ReducedValidityDomain` (§4c) is
/// a CORRECTNESS prerequisite and not a nicety.
///
/// The one quantity measured to catch it was `gap_dev` — and `gap_dev` is
/// `|min_sd − ORACLE min_sd|`. **It cannot be a gate.** A runtime check that
/// needs the full-order answer has nothing left to protect. So the detector we
/// have does not exist online, and §4c's *stated* online signal (`‖q‖` against
/// the training envelope) is listed in recon §5 under honestly-open research as
/// not detecting a moved contact patch — argued, never measured.
///
/// This measures four candidates on the matrix where the ground truth is already
/// known, and none of them may look at an oracle. It sets no thresholds: those
/// are learned, and this is the pilot they get learned from.
///
/// ## The matrix — shared with [`reduced_basis_generalises`], deliberately
///
/// The same [`TRAIN_OFFSETS`], [`TEST_OFFSETS`], [`BASIS_RANKS`] and
/// [`GEN_A_OVER_CELL`], so the ground truth this is scored against is the one
/// §2l published rather than a lookalike re-run. The trajectories are re-run
/// (both tests are `#[ignore]`, so nothing pays for it twice in CI) but the
/// constants are shared, which is where a drift would actually invalidate the
/// comparison.
///
/// ## Pre-registration (before the first run)
///
/// 1. **`residual_excess` separates BUT CONFLATES.** It should rise
///    out-of-domain and it should *also* be large on the IN-SAMPLE arm at
///    `r = 20`, which is in domain and merely solved badly (`relL2 = 2.4e-2`) —
///    the same defect §2l found in `gap_dev`, for the same reason. Signature:
///    **falls with rank in-sample, flat with rank out-of-domain.** If so it
///    cannot be the gate alone.
/// 2. **`envelope_excursion` does NOT separate** — the recon's argument is that
///    a translated patch excites mode amplitudes the ensemble already covers, so
///    the box contains it while the answer is `109 %` wrong. Predicted `0.0` or
///    near it at `+1.50a` and `+2.00a`. ★ If it DOES separate, recon §5's open
///    item 3 is answered in the cheap direction and rung 2 is nearly free.
/// 3. **`snapshot_distance` separates where the box does not** — same coordinates,
///    tight hull instead of loose. It is here so that a flat `envelope_excursion`
///    can be attributed to the *statistic* rather than to `q`-space itself; if
///    both are flat, `q` genuinely carries no domain information and only the
///    geometric signal is left.
/// 4. **`active_novelty` separates and does NOT conflate.** ★ Its falsifiable
///    signature is structural: **FLAT IN RANK, STEPPED IN POSITION**, because it
///    is a statement about where the collider is and barely involves the basis.
///    If it moves strongly with rank, it is contaminated by the solution and is
///    not the basis-independent check it claims to be. Predicted `0.0` at
///    `+0.00a` AND at `+0.25a`, rising at `+1.50a`, higher at `+2.00a`.
/// 5. ★★ **The consequence, if 1 and 4 both hold: the gate needs TWO signals.**
///    A domain signal says whether the scene is in the hull; an error signal says
///    whether it is being solved well. `gap_dev` conflates them because it is
///    sensitive to both. They have opposite remedies — more training vs. more
///    rank — so a gate that cannot say which it caught cannot say what to do.
/// 6. **Genuinely uncertain, and it is the product question:** which side
///    `+0.25a` lands on. It is `1.9e-3` — three orders worse than in-sample and
///    arguably still usable. A gate that refuses interpolation is useless; one
///    that accepts it owes an account of the three orders. No prediction.
///
/// ## Measured — 2026-08-25, `a/cell = 2` (5 202 free DOF, 2 023 vertices)
///
/// ⚠⚠ **This section was rewritten after a cold re-derivation. The first version
/// reported a trajectory MAXIMUM for a question about a per-step check, and that
/// single choice inverted three of its four verdicts.** What follows is measured
/// on an instrument that keeps every step. The superseded claims are named at the
/// bottom, because a reader who saw them needs to know which ones moved.
///
/// Ground truth reproduces §2l at 15 of 16 cells and corrects the sixteenth (see
/// [`reduced_basis_generalises`]). Training set `355` snapshots, `41` of `2 023`
/// vertices ever contact-active. Wiring control: `0.000` novel against its own
/// union, `0.476` against the `-1.00a` union alone.
///
/// ### The question a gate is actually asked
///
/// **Not** "does this signal separate the training span from outside it" — that
/// is a partition over POSITION, and on this fixture position and error are
/// monotone together, so it flatters anything tracking the indenter. A gate
/// refuses an answer for being too WRONG. So: is there ONE threshold `θ` with
/// `accept ⟺ max_rel_err ≤ τ`? Swept over `τ`, printed by the harness:
///
/// | | `τ=1e-1` | `5e-2` | `3e-2` | `2e-2` | `1e-2` | `5e-3` | `3e-3` | `2e-3` | `1e-3` |
/// |---|---|---|---|---|---|---|---|---|---|
/// | **S1** worst step | 6.80× | 6.80× | 2.37× | 1.37× | 2.71× | 2.69× | FAIL | FAIL | 98.8× |
/// | **S1** median step | FAIL | FAIL | 1.35× | FAIL | 1.13× | 5.04× | FAIL | FAIL | 85.2× |
/// | **S2** either | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
/// | **S3** worst step | 2.30× | 2.30× | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | 151× |
/// | **S3** median step | 1.94× | 1.94× | 1.00× | FAIL | FAIL | FAIL | FAIL | FAIL | 56.4× |
/// | **S4** worst step | **∞** | **∞** | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
/// | **S4** median step | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
///
/// ★ **Read the two rows per signal as a pair.** A gate that fires when ANY step
/// crosses `θ` reaches its verdict on the **worst step**, so that row is the
/// accept/refuse decision. The **median** row says how much of the trajectory is
/// behind it: separation on both is a plateau; separation only at the max rests
/// on a minority of steps and is one quiet step from silence.
///
/// ### The four verdicts
///
/// - ⛔ **S2 is dead.** It fails at every `τ` on both rows. §5's open item 3 was
///   still wrong to say `q`-space carries no position information — S2 separates
///   POSITIONS strongly at every fixed rank — but the per-mode box cannot convert
///   that into an accuracy rule at any tolerance.
/// - ⛔ **S4's `∞` margin is an ARTIFACT of the maximum.** At `+1.50a` its median
///   is **exactly `0.000` at all four ranks** while the answer is `14.8 %–31.8 %`
///   wrong; the max of `0.111–0.286` comes from a minority of steps. And the
///   denominator, now printed rather than inferred, is **10–18 active vertices at
///   `+1.50a` and 4–12 at `+2.00a`** against 21 in-sample. ⇒ the whole signal is
///   one or two vertices on a few steps. ★ Worse, it is confounded: the active
///   set SHRINKS out of domain (`21 → 4`), so the fraction rises partly because
///   the contact patch is disappearing, not only because it is novel.
/// - ⚠ **S1 gates on accuracy at 7 of 9 tolerances on the worst step — and its
///   separation does not survive per-step.** At the median, `+0.25a` at `r=20`
///   reads `7.923e7` against `+1.50a` at `r=142`'s `5.838e7`: **inverted, 0.737×**.
///   It is also **not free at R3**: `‖r_free‖` exists here only because R1.1
///   sweeps every element, which is exactly what ECSW removes.
/// - ✅ **S3 is the only candidate that survives both rows.** `2.30×` on the worst
///   step, `1.94×` on the median — a plateau. Its interpolation row reads
///   `5.062e-1 / 5.063e-1 / 5.063e-1 / 5.069e-1`, a spread of **`0.138 %`**
///   across a `7.1×` rank change.
///
/// ★★ **The flatness has its own two-sided control, in the same column.** It
/// would prove nothing if the statistic were trivially rank-invariant. It is not:
/// the IN-SAMPLE row of the same quantity falls **`5 168×`** (`3.59e-3 → 6.94e-7`)
/// over the same range — as it must, since in-sample the reduced solution
/// converges onto trajectories that ARE training points.
///
/// ### Why S3 and not S2 — and it is NOT max-versus-norm
///
/// Both read the same coordinates. S2 normalises **per mode** and S3 by **one
/// global scale** (the training cloud's radius). A per-mode divisor is set by the
/// thinnest retained axis, which is always the newest tail mode, so S2's
/// IN-SAMPLE row climbs `104×` with rank on the one row that is in-domain by
/// definition. ⇒ **use ONE GLOBAL normaliser, not a per-mode one.**
///
/// ⛔ An earlier version of this section drew the rule as "take a NORM over modes,
/// never a MAX", and that is **provably wrong**: `‖d‖₂ ≥ ‖d‖∞` pointwise, so a
/// norm over the SAME per-mode-normalised deviations is at least as rank-sensitive
/// as the max — measured at `2.06× → 3.48×` over `r = 20 → 142` on random
/// deviations. The operation was never the problem; the normaliser was. §7 had
/// already recorded the wrong rule as rung 2's design.
///
/// ### The spectrum, which settles what `r=160 → 142` means
///
/// `142` modes of `355` snapshots, `σ_last/σ_0 = 1.012e-8` — so the truncation is
/// `SIGMA_FLOOR_REL`, not anything structural. But **retained energy is
/// `1.000000000`**: the top rung already extracts everything this ensemble
/// contains energetically while spanning 142 of 355 directions. ⇒ "flat in rank
/// out of domain" is not a truncation statement. Asking for more modes cannot
/// help, because there is no more energy to get — only different snapshots can.
///
/// ### Pre-registration, scored honestly
///
/// 1. **HELD** on the worst step (S1 falls `5.8` orders in-sample, `0.77`/`0.58`
///    out of domain) — and its per-step separation inverts, which was not asked.
/// 2. ⛔ **FALSIFIED.** S2 separates positions strongly at fixed rank.
/// 3. **HELD**, and S3 is the only survivor — but at `~2×`, not the `141×` the
///    first version advertised (see the superseded list).
/// 4. ⛔ **FALSIFIED, and more completely than first recorded.** S4 is not merely
///    rank-dependent; its separation is a per-trajectory spike over a handful of
///    vertices.
/// 5. ⚠ **VOID, not HELD.** Item 5 was written as a conditional on items 1 and 4
///    both holding. Item 4 did not hold. The first version marked it HELD anyway —
///    the one place the pre-registration was designed to bind, and it did not.
/// 6. **Answered, with a caveat that undercuts it:** interpolation reads in-domain
///    on both geometric signals. ⚠ But `+0.25a` is `0.5` cells off-lattice
///    (`a/cell = 2`, so training sits at `0, ±1, ±2` cells and the extrapolations
///    at `3` and `4`) — **the only offset in the experiment not on a lattice
///    site**, and the one the whole question rests on.
///
/// ### ⇒ What this rung actually delivered
///
/// **Not a gate, and not a gate design.** A narrowed candidate set and a measured
/// reason each of the others fails: S2 cannot convert position information into an
/// accuracy rule; S4's margin is a maximum over a few vertices on a few steps;
/// S1's is real per-trajectory but absent per-step and unavailable under ECSW. S3
/// is left standing at `~2×`, on a fitted threshold, on one fixture.
///
/// ⇒ ⛔ **AND S3 DID NOT SURVIVE THE NEXT TWO RUNGS. Read them before building on
/// anything above.** [`the_signal_margin_on_a_held_out_position`] (§2n) supplies
/// the held-out position this section says it lacks, and S3's rank-independence —
/// named above as the signature that made it a domain signal — is exactly what
/// makes it blind to resolution. [`how_dense_the_training_ensemble_must_be`]
/// (§2o) then shows it is blind to ENSEMBLE SIZE as well. The two-signal
/// conclusion this section withdraws returns in §2n on held-out evidence rather
/// than the confounded position partition that was withdrawn.
///
/// ### ⚠ What this does NOT establish
///
/// - **No held-out position exists.** Every margin is computed on the same 16
///   cells that chose the threshold. The hull edge is `1.0a` and the nearest
///   scored out-of-domain point is `1.5a`, with **nothing sampled between** — so
///   every window here is an upper bound on an unmeasured quantity.
/// - **The DOMAIN/ERROR taxonomy is not falsifiable on this fixture.**
///   Distance-from-training and error are monotone together across all four
///   positions, so no cell could have distinguished the two kinds. The labels are
///   definitions.
/// - **Extrapolation is confounded with the free edge.** The plate is `8a`; the
///   scored extrapolations keep `1.5a` and `1a` of clearance.
/// - **The in-sample column measures RECONSTRUCTION, not prediction** — its 71
///   steps are all training snapshots — and it is the anchor every ratio is taken
///   against.
/// - **One basis construction.** A single pooled POD under a mass inner product.
///   Nothing here speaks to per-offset, parametric, or contact-local bases.
/// - **`snapshot_distance` assumes [`Inner::Mass`]** so that Euclidean distance in
///   `q` is mass-norm distance in `u`. Nothing checks it; `PodBasis` exposes no
///   `inner()`.
/// - **Every arm ran `Inertial`,** and the predictor exposure is NOT S1's alone —
///   `snapshot_distance` reads `s.q` and `active_novelty` reads `x_rest + Φq`,
///   both of which are the solved state.
/// - **No timing.** Whether any of this is affordable is unmeasured.
/// - **No threshold is set.** One fixture, one mesh, one material, one ramp.
///
/// ### ⛔ Superseded by this rewrite — named so the earlier text is traceable
///
/// | first published | corrected |
/// |---|---|
/// | "the same value to FOUR significant figures" | distinct at 4 and 3 s.f.; a `0.138 %` spread, same at TWO |
/// | "take a NORM over modes, never a MAX" | the per-mode NORMALISER was the fault; a norm is at least as rank-sensitive |
/// | "S4 wins that cut outright, `0` vs `≥0.111`" | a maximum-only artifact; median `0.000` at `+1.50a` |
/// | "S3's margin is `141×`" | `2.30×` worst step, `1.94×` median; `141×` was in-sample-vs-everything |
/// | "the gate needs TWO signals" | not shown for GATING; that argument is about DIAGNOSIS. ⚠ The CONCLUSION returns in §2n on held-out evidence — only the argument stays withdrawn |
/// | "`‖r_free‖`… and it is free" | free at R1.1, NOT under ECSW at R3 |
/// | "out of domain that field is `28 %`–`109 %` wrong" | that is the `r=142` column; the range is `14.8 %`–`119.6 %` across all eight cells, `14.8 %`–`109 %` across the seven that complete |
/// | "`≥ 1.17` extrapolating" | the minimum is `1.165` |
/// | "byte-identical output" (non-regression) | the 37 measurement ROWS are identical; the files differ |
/// | pre-registration item 5 "HELD" | VOID — its antecedent was falsified |
#[test]
#[ignore = "§4c online-signal pilot — 8 full-order trajectories, ~5 min (see the fn docs)"]
fn an_online_signal_separates_out_of_domain() {
    let base = Scene::new(GEN_A_OVER_CELL);
    let probe_mesh = base.mesh();
    let x_rest = rest_positions(&probe_mesh);
    let n_vertices = x_rest.len() / 3;
    let solver = base.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let n_steps = base.n_steps();
    // ⚠ Read from the same constructor `Scene::solver` builds on. That fn
    // overrides `dt`, `density`, `max_newton_iter`, `initial_guess` and
    // `gravity_z` and NOT `tol`, so this is the tolerance the arms actually
    // converged at — but the coupling is by inspection, not by the compiler,
    // which is why the figure is printed rather than left implicit.
    let tol = SolverConfig::skeleton().tol;
    // ⚠ THREE things below hard-wire `0.0` as "the in-sample point": the oracle
    // reused for `TEST_OFFSETS[0]`, the wiring control's `|dx|` choice of the
    // FAR offset, and `top_row(rows, TEST_OFFSETS[0].0)` as the two-sided
    // control. Change `TEST_OFFSETS[0]` to a held-out offset and all three go on
    // reading as though it were in-sample — silently, since a held-out arm still
    // completes. Nothing else pins them together, so this does.
    assert!(
        TRAIN_OFFSETS.contains(&TEST_OFFSETS[0].1),
        "TEST_OFFSETS[0] is {:+.2}a, which is NOT a training offset — the row \
         this test calls its IN-SAMPLE control would be held out",
        TEST_OFFSETS[0].1,
    );
    assert!(
        tol > 0.0,
        "tol is {tol}, so `residual_excess` would be `inf` or negative — a unit \
         of zero is not a unit",
    );

    println!(
        "\nRC\t§4c SIGNAL PILOT: IPC indentation a/cell={GEN_A_OVER_CELL:.1}, {} free DOF, \
         {n_vertices} vertices, {n_steps} steps/trajectory\nRC\ttrain offsets {TRAIN_OFFSETS:?} a \
         |  patch radius a = {:.3e} m  |  tol = {tol:.1e}  |  band d̂ = {:.3e} m",
        fd.len(),
        patch_radius(),
        base.d_hat,
    );

    // ── training: the ensemble, and everything it SAW ──
    let mut train = SnapshotSet::new(fd.len());
    let mut train_active = std::collections::BTreeSet::new();
    // The single training offset FARTHEST from the in-sample point, kept for the
    // wiring control below. Chosen by `|dx|` rather than by position in
    // `TRAIN_OFFSETS`, so reordering that constant cannot quietly hand the
    // control the in-sample set itself.
    let mut far: Option<(f64, std::collections::BTreeSet<usize>)> = None;
    let mut in_sample_oracle = None;
    for dx in TRAIN_OFFSETS {
        let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
        assert!(
            (sc.d_hat - base.d_hat).abs() < f64::EPSILON,
            "the barrier band changed with the offset, so the training union is \
             built with a different definition of contact-active than the probes \
             score against",
        );
        let o = run_oracle(sc, &x_rest, NO_TIMING);
        assert!(
            o.arm.failure.is_none(),
            "training trajectory at {dx:+.2}a failed: {}",
            o.arm.failure.as_deref().unwrap_or(""),
        );
        let mut mine = std::collections::BTreeSet::new();
        for (k, x) in o.x.iter().enumerate() {
            assert_eq!(
                x.len() / 3,
                n_vertices,
                "the mesh changed with the indenter offset, so vertex indices are \
                 not comparable across trajectories and every novelty reading below \
                 compares different bodies",
            );
            train.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
            mine.extend(active_vertices(x, sc.centre_at(k), sc.d_hat));
        }
        train_active.extend(mine.iter().copied());
        if far.as_ref().is_none_or(|(d, _)| dx.abs() > d.abs()) {
            far = Some((dx, mine));
        }
        if dx == 0.0 {
            in_sample_oracle = Some(o);
        }
    }
    let in_sample_oracle = in_sample_oracle.expect("+0.00a is one of the training offsets");
    println!(
        "RC\ttraining set: {} snapshots, {} of {n_vertices} vertices ever contact-active",
        train.len(),
        train_active.len(),
    );

    // ── control: is the novelty signal WIRED TO THE SCENE? ──
    //
    // ★★ The unit tests prove `active_novelty` computes a fraction. They cannot
    // prove this probe is reading THIS fixture. So: the in-sample trajectory
    // must read `0` against the union it is part of (by construction), and it
    // must read NON-ZERO against a union built from the far training offset
    // alone. Without the second half, a probe wired to a set that happens to
    // contain every vertex would pass the first half and every arm below would
    // read "in domain" for a reason about the rig.
    assert!(
        !train_active.is_empty() && train_active.len() < n_vertices,
        "{} of {n_vertices} vertices are in the training active set — an empty set \
         makes every arm maximally novel and a FULL one makes every arm perfectly \
         in domain, and neither is a measurement",
        train_active.len(),
    );
    let (far_dx, far_only) = far.expect("TRAIN_OFFSETS is not empty");
    let (mut self_novelty, mut far_novelty) = (0.0_f64, 0.0_f64);
    for (k, x) in in_sample_oracle.x.iter().enumerate() {
        let a = active_vertices(x, base.centre_at(k), base.d_hat);
        self_novelty = self_novelty.max(active_novelty(&a, &train_active));
        far_novelty = far_novelty.max(active_novelty(&a, &far_only));
    }
    assert!(
        self_novelty.abs() < f64::EPSILON,
        "the +0.00a trajectory reads {self_novelty:.3e} novel against a union it is \
         PART OF — the probe is not reading the scene it thinks it is",
    );
    assert!(
        far_novelty > 0.0,
        "the +0.00a trajectory reads ZERO novel against the {far_dx:+.2}a active set \
         alone — the signal cannot distinguish two patch positions a full {:.1}a \
         apart, so a zero anywhere below means nothing",
        far_dx.abs(),
    );
    println!(
        "RC\t★ wiring control: +0.00a novelty = {self_novelty:.3e} vs its own union, \
         {far_novelty:.3} vs the {far_dx:+.2}a union alone"
    );

    // ── the bases, and one probe per basis, both built ONCE ──
    //
    // ★ Outside the position loop, for the reason `reduced_basis_generalises`
    // fits its bases outside it: the hull is a function of the TRAINING set and
    // the rank alone. Building it per scored position would cost four times the
    // projections and — worse — would read as though what counts as "in domain"
    // depended on the point being asked about, which is the opposite of what a
    // validity domain claims to be.
    let mut bases = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for r in BASIS_RANKS {
        let fitted = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, r).expect("basis fits");
        if seen.insert(fitted.n_modes()) {
            bases.push((r, fitted));
        }
    }
    let probes: Vec<Probe<'_>> = bases
        .iter()
        .map(|(_, basis)| {
            let train_q: Vec<Vec<f64>> = train.columns().iter().map(|u| basis.project(u)).collect();
            let radius = cloud_radius(&train_q);
            assert!(
                radius > 0.0,
                "the training cloud at r={} has zero radius, so `snapshot_distance` \
                 would divide by zero",
                basis.n_modes(),
            );
            Probe {
                envelope: training_envelope(&train_q),
                train_q,
                cloud_radius: radius,
                train_active: &train_active,
                tol,
                // Constant across scenes: `Scene::at_offset` moves the indenter
                // and changes nothing else, which the loop below re-checks.
                d_hat: base.d_hat,
            }
        })
        .collect();
    println!(
        "RC\tbases: {}",
        bases
            .iter()
            .map(|(r, b)| format!("r={r}→{}", b.n_modes()))
            .collect::<Vec<_>>()
            .join(", ")
    );
    // ⚠ The top rung asks for 160 and gets 142, and until this printed, nothing
    // said WHY. It matters: if 142 is the training set's numerical rank, then the
    // top rung already spans everything the ensemble contains, and "flat in rank"
    // out of domain is not a truncation statement at all — no achievable rank
    // could fix it, only different snapshots.
    let top = &bases.last().expect("at least one basis").1;
    let sv = top.singular_values();
    println!(
        "RC\tSPECTRUM: {} modes retained of {} snapshots; sigma_last/sigma_0 = {:.3e}; \
         retained energy = {:.9}\nRC\t  => the top rung {} the training span ({} modes vs {} snapshots)",
        top.n_modes(),
        train.len(),
        sv.get(top.n_modes() - 1).copied().unwrap_or(f64::NAN) / sv.first().copied().unwrap_or(1.0),
        top.retained_energy_fraction(),
        if top.n_modes() >= train.len() {
            "SPANS"
        } else {
            "does NOT span"
        },
        top.n_modes(),
        train.len(),
    );

    // ── score every position against every rank, with the probe attached ──
    let mut rows: Vec<(&'static str, usize, Arm)> = Vec::new();
    for (label, dx) in TEST_OFFSETS {
        let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
        assert!(
            (sc.d_hat - base.d_hat).abs() < f64::EPSILON,
            "the barrier band changed with the offset, so the probes were built \
             against a different definition of contact-active than the arms use",
        );
        let owned;
        let oracle = if dx == 0.0 {
            &in_sample_oracle
        } else {
            owned = run_oracle(sc, &x_rest, NO_TIMING);
            assert!(
                owned.arm.failure.is_none(),
                "scoring oracle at {dx:+.2}a failed: {}",
                owned.arm.failure.as_deref().unwrap_or(""),
            );
            &owned
        };

        for ((r, basis), probe) in bases.iter().zip(&probes) {
            let got = basis.n_modes();
            let arm = run_reduced(
                sc,
                basis,
                InitialGuess::Inertial,
                format!("{label}  r={r}→{got}"),
                &Ctx {
                    x_rest: &x_rest,
                    fd: &fd,
                    oracle,
                    probe: Some(probe),
                },
                NO_TIMING,
            );
            arm.print(n_steps, sc.d_hat);
            rows.push((label, got, arm));
        }
    }

    // ── control: no PRODUCER, no measurement ──
    assert_every_arm_produced(&rows, TEST_OFFSETS.len() * bases.len());

    // ── control: the rig can still score a basis it DID fit ──
    //
    // Carried over from `reduced_basis_generalises` unchanged. The held-out rows
    // are the MEASUREMENT and a bad number there is the finding; what must hold
    // is that the in-sample arm survives its own trajectory, or nothing above is
    // interpretable.
    let control = top_row(&rows, TEST_OFFSETS[0].0);
    assert!(
        control.2.failure.is_none() && control.2.min_sd > 0.0,
        "the IN-SAMPLE control did not survive its own trajectory — nothing above \
         is interpretable",
    );

    // ── the tables ──
    print_matrix(
        "GROUND TRUTH — rel-L2 vs each position's OWN full-order oracle",
        "This is the column every signal below tries to predict with NO oracle. \
         A `—` here is an arm that DIVERGED; its signal cells below are the \
         reading over the steps it did complete, which is what a gate would have \
         had to catch.",
        &rows,
        |a| a.failure.is_none().then_some(a.max_rel_err),
    );
    print_matrix(
        "REFERENCE — gap_dev (d̂), §2l's detector. ⚠ NEEDS THE ORACLE, cannot gate",
        "Shown to be beaten, not to be used: it differences against `oracle.min_sd`.",
        &rows,
        |a| a.failure.is_none().then_some(a.gap_dev),
    );
    // ── worst-over-steps, the statistic the first pilot reported ──
    for (name, note, pick) in SIGNAL_TABLES {
        print_matrix(&format!("{name}   worst over steps"), note, &rows, |a| {
            signal(a, pick, Quantile::Max)
        });
    }

    // ── the MEDIAN step, which is what a per-step check would mostly see ──
    //
    // ★★ A max over 71 steps is an extreme-value statistic. A gate reads ONE
    // step. Where the median tracks the max the trajectory figure transfers;
    // where it collapses toward the in-domain value, the max is a spike and the
    // separation above is not available to a runtime check at most steps.
    for (name, _, pick) in SIGNAL_TABLES {
        print_matrix(&format!("{name}   MEDIAN step"), MEDIAN_NOTE, &rows, |a| {
            signal(a, pick, Quantile::Median)
        });
    }

    // ── active_novelty's denominator, without which its fractions are unreadable ──
    print_matrix(
        "active-set SIZE (max over steps) — S4's DENOMINATOR",
        "A novelty of `1.000` over 4 active vertices and over 40 are different \
         readings. The first pilot printed the fraction and not this.",
        &rows,
        |a| {
            a.signals
                .as_ref()
                .filter(|t| t.is_recorded())
                .map(|t| t.active_range().1 as f64)
        },
    );

    // ★★ BOTH quantiles, and the pair is the argument. A gate that fires when
    // ANY step exceeds its threshold is decided by the MAX — so the max pass is
    // the accept/refuse verdict. The MEDIAN pass says how much of the trajectory
    // is behind that verdict: a signal that separates at both is a plateau, one
    // that separates only at the max rests on a minority of steps and is one bad
    // step away from silence.
    print_gate_table(&rows, &GATE_TAUS, Quantile::Max);
    print_gate_table(&rows, &GATE_TAUS, Quantile::Median);
    println!(
        "\nRC\t★ HOW TO READ THIS:\n\
         RC\t   The four matrices show SHAPE across rank and position.\n\
         RC\t   The GATE TEST shows whether any of it converts into a rule.\n\
         RC\t   ⚠ The [ERROR]/[DOMAIN] labels are DEFINITIONS, not findings — on \
         this fixture\n\
         RC\t   distance-from-training and error are monotone together, so no cell \
         here could\n\
         RC\t   have distinguished the two kinds. NO THRESHOLD IS SET."
    );
}

/// The four candidates, their table captions, and their accessors — one list so
/// the worst-step and median passes cannot drift apart.
const SIGNAL_TABLES: [(&str, &str, fn(&Signals) -> f64); 4] = [
    (
        "S1  residual_excess = ‖r_free‖ / tol   [ERROR by definition]",
        "⚠ NOT normalised by load, and NOT free at R3: `r_free` exists here only \
         because R1.1 sweeps every element, which is what ECSW removes.",
        |s| s.residual_excess,
    ),
    (
        "S2  envelope_excursion (half-widths outside the per-mode box)   [DOMAIN by definition]",
        "§4c's STATED signal. Normalised PER MODE, so the thinnest retained axis \
         sets it.",
        |s| s.envelope_excursion,
    ),
    (
        "S3  snapshot_distance (cloud radii to the NEAREST TRAINING SNAPSHOT)   [DOMAIN by definition]",
        "⚠ NOT a hull distance: a point deep inside the convex hull but in a GAP \
         between snapshots reads large. Normalised by ONE global scale.",
        |s| s.snapshot_distance,
    ),
    (
        "S4  active_novelty (fraction of active vertices never active in training)   [DOMAIN by definition]",
        "⚠ Returns 0 for an EMPTY active set, so a solve that loses contact reads \
         maximally in-domain — fail-open. Read with the denominator below.",
        |s| s.active_novelty,
    ),
];

const MEDIAN_NOTE: &str = "Compare with the worst-step matrix above: tracking ⇒ plateau (transfers to a \
     per-step check); collapsing ⇒ spike (does not).";

/// Accuracy thresholds the gate test sweeps. Spans four orders, and deliberately
/// brackets `1e-2` — a 1 % field error is the loosest tolerance anyone would call
/// acceptable, so a signal that cannot gate there cannot gate where it matters.
const GATE_TAUS: [f64; 9] = [1e-1, 5e-2, 3e-2, 2e-2, 1e-2, 5e-3, 3e-3, 2e-3, 1e-3];

// ── §4c rung 1b: is the margin FITTED or VALIDATED, and is `+0.25a` confounded? ──

/// The offset dropped from training and then scored — the held-out position the
/// first pilot did not have.
const LOO_HELD_OUT: f64 = 0.0;

/// An OFF-lattice extrapolation point, at `3.5` cells. `+1.50a` and `+2.00a` are
/// at `3` and `4` cells and bracket it in extrapolation distance, so if mesh
/// alignment were a large effect this would fall OUTSIDE their bracket.
const OFF_LATTICE_EXTRAP: f64 = 1.75;

/// **Every margin in [`an_online_signal_separates_out_of_domain`] is FITTED.**
///
/// ```text
/// cargo test --release -p sim-soft --test reduced_contact \
///   the_signal_margin_on_a_held_out_position -- --ignored --nocapture
/// ```
///
/// The threshold interval there is derived from the same 16 cells it is scored
/// on, and there is no held-out position anywhere in that matrix. A margin
/// measured on the points that chose it is an upper bound on an unmeasured
/// quantity, not a measurement — which is the largest single hole a cold
/// re-derivation found in §2m.
///
/// Two arms, and the pair costs one extra oracle over the main pilot:
///
/// **A — LEAVE-ONE-OUT.** Refit on `[-1, -0.5, +0.5, +1]a` and score at
/// `+0.00a`, whose oracle already exists. This is the first genuinely held-out
/// position in the arc.
///
/// ★★★ **And it lands on a coincidence worth more than the validation.**
/// Held-out `+0.00a` sits `0.5a` from its nearest training snapshot and is ON the
/// mesh lattice. So does `+1.50a`. **Matched gap, matched lattice status, one
/// INTERPOLATING and one EXTRAPOLATING** — a direct test of whether "inside the
/// convex hull" carries any information beyond "close to a training snapshot". If
/// the two arms land together, then the in-hull/out-of-hull framing this whole
/// rung is built on is really a distance framing wearing a topological name.
///
/// **B — THE LATTICE CONTROL.** `a/cell = 2`, so the training offsets sit at
/// `0, ±1, ±2` cells and `+0.25a` sits at `0.5` — the ONLY scored offset off the
/// lattice, and the arm the product question rests on. That confound cannot be
/// broken by moving an interpolation point, because with `cell = 0.5a` every
/// on-lattice point inside the training span IS a training point. So it is tested
/// at the extrapolation end instead: `+1.75a` is off-lattice at `3.5` cells and
/// its extrapolation distance is bracketed by `+1.50a` and `+2.00a`, both on it.
///
/// ## Pre-registration (before the first run)
///
/// 1. **No prediction on arm A's error**, and it is the interesting one. Gap
///    `0.5a` is twice `+0.25a`'s.
/// 2. ★ **If held-out `+0.00a` and `+1.50a` land within ~an order of each other,
///    the hull framing is refuted** and `snapshot_distance` is measuring the only
///    thing that ever mattered.
/// 3. **`snapshot_distance` at held-out `+0.00a` reads `~1.0`** if it tracks gap
///    linearly — `+0.25a` reads `0.506` at half the gap.
/// 4. ★ **`active_novelty` at held-out `+0.00a` reads `0.000`**: the contact patch
///    is inside the union of the four remaining trajectories. **If its error is
///    also large, S4 is falsified as a gate in a single cell** — a fail-open
///    exactly where a gate must not have one.
/// 5. **`+1.75a` lands INSIDE the `+1.50a`/`+2.00a` bracket** if alignment is not
///    a large effect. Outside it, `+0.25a`'s three orders are partly a mesh
///    artifact and §2l's interpolation finding needs re-scoping.
///
/// ## Measured — 2026-08-25, 8 oracles, `140.8 s`
///
/// ⚠ Two runs exist. The first (`139.7 s`) recorded the signals and printed none
/// of them; every signal figure below is from the second, which is why its wall
/// time is the one quoted. The arm rows are identical across both.
///
/// ### ★★★ The headline: RANK-INDEPENDENCE IS WHY IT CANNOT GATE
///
/// §2m left one candidate standing and called its rank-independence the signature
/// that made it a domain signal. **On a held-out ensemble that property is exactly
/// what disqualifies it.** Four ranks, one basis family, one hull — what a
/// deployed gate actually sees:
///
/// | | across the four ranks |
/// |---|---|
/// | `max_rel_err` | **31.9×** (`3.733e-1 → 1.172e-2`) |
/// | `snapshot_distance` worst step | **1.005×** (`0.9756 → 0.9704`) |
/// | `snapshot_distance` median step | 1.023× |
/// | `residual_excess` worst step | 6.409× |
/// | `active_novelty` | `0.000` at every rank |
///
/// A signal that does not move with rank **cannot report how well the basis
/// resolved the answer**, because that is the thing rank changes. Its
/// single-ensemble threshold margins are `1.002×`, `1.002×`, `1.002×`, `1.000×`
/// across `τ = 1e-1 … 2e-2` — noise. `residual_excess` gets `2.883×` on the same
/// cells because it is an error signal and tracks rank by construction.
///
/// ⇒ **Rank-independence and accuracy-sensitivity are mutually exclusive.** That
/// is not a design preference; it follows from what each quantity measures.
///
/// ### ⇒ Which RESURRECTS the two-signal conclusion — for the opposite reason
///
/// §2m withdrew "the gate needs TWO signals" because the argument given for it was
/// about DIAGNOSIS and was sold as gating. The conclusion returns here on
/// different evidence: a **held-out** position, not a confounded position
/// partition. `snapshot_distance` reads position and is blind to resolution;
/// `residual_excess` reads resolution and is per-step-blind to position (§2m) and
/// unavailable under ECSW. **Neither alone gates**, and now that is measured.
///
/// ### ⛔ §2m's margin was FITTED, and it does not survive
///
/// The load-bearing statement is WITHIN arm A, so no normaliser question touches
/// it: its four readings are `0.9704–0.9756` while its errors run `1.2 %` to
/// `37 %`. **No threshold separates them** — one below `0.9704` refuses all four,
/// one above `0.9756` accepts all four, and there is nothing in between to place
/// one at. A `32×` error spread sits at one signal value.
///
/// ⚠ §2m's interval was `[0.5069, 1.165)` and arm A's readings land inside it, in
/// the band nothing was ever sampled in — but that comparison is **DIRECTIONAL,
/// not numeric**: §2m's interval is in the 355-snapshot ensemble's units and arm
/// A's readings in the 284-snapshot ensemble's, and `snapshot_distance` divides by
/// each ensemble's OWN cloud radius. See the caveats.
///
/// ### The matched-gap pair — the comparison this test was built for
///
/// Held-out `+0.00a` and `+1.50a` sit `0.5a` from their nearest training snapshot,
/// both ON the lattice, one interpolating and one extrapolating:
///
/// | rank | HELD-OUT `+0.00a` | `+1.50a` | ratio |
/// |---|---:|---:|---:|
/// | 20 | 3.733e-1 | 3.149e-1 | **0.84×** |
/// | 40 | 1.406e-1 | 1.478e-1 | **1.05×** |
/// | 80 | 2.902e-2 | 3.177e-1 | 10.95× |
/// | 116 / 142 | 1.172e-2 | 2.822e-1 | 24.08× |
///
/// ★★★ **At `r=20` extrapolation is the BETTER of the two, and at `r=40` they are
/// indistinguishable.** Being inside the hull does not make the answer better at a
/// given rank — it makes the answer **improvable by rank**. Interpolation falls
/// `31.9×` across the sweep; extrapolation falls `1.12×`. ⇒ This sharpens §2l's
/// "rank does not fix it": rank does not fix it OUTSIDE the hull, and does fix it
/// inside, at a rate the gap degrades.
///
/// ⚠ And at that matched gap, `snapshot_distance`'s **median reads `4.117e-1` vs
/// `4.271e-1` — `1.04×` apart for a `24×` error difference.** ⚠⚠ Those two
/// medians come from DIFFERENT ensembles and therefore different normalisers, so
/// `1.04×` is a DIRECTION and not a number; the ratio between the normalisers is
/// unmeasured. The within-arm version needs no such comparison and says the same
/// thing: inside arm A alone the median moves `1.023×` across a `31.9×` error
/// range.
///
/// ### The lattice control — the mesh confound is largely defused
///
/// `+1.75a` (off-lattice, `3.5` cells) falls **INSIDE** the `+1.50a`/`+2.00a`
/// bracket at all four ranks (`0.720` in `[0.315, 1.196]`; `0.442` in
/// `[0.148, 0.764]`; `0.674` in `[0.318, 1.037]`; `0.626` in `[0.282, 1.093]`).
/// Alignment is not a large effect at the extrapolation end, and error tracks gap
/// smoothly across `0.5a → 0.75a → 1.0a` with no lattice jump. ⚠ Evidence, not
/// proof, for the interpolation end — `+0.25a` remains the only off-lattice
/// interpolation point, and with `cell = 0.5a` no on-lattice one can exist inside
/// the training span.
///
/// ### Pre-registration
///
/// 1. No prediction was made. Held-out `+0.00a`: `3.733e-1 → 1.172e-2`.
/// 2. ⚠ **Half-refuted, and the half that fails is the interesting one.** The two
///    arms land within `1.05×` at `r=40` and INVERT at `r=20`, so at low rank the
///    hull framing carries nothing. It is upheld only at high rank, and its
///    content is the TREND rather than the level.
/// 3. ✅ **`~1.0` predicted, `0.970` measured.** ⚠ But score it weakly: the
///    prediction was extrapolated from `0.506` at `0.25a`, which is §2m's
///    FULL-ensemble reading, and `0.970` is arm A's LOO-ensemble reading. The
///    two divide by different cloud radii, so "close to linear in gap" is a
///    plausible reading of two points in two unit systems and not a measured
///    slope. Nothing downstream rests on it.
/// 4. ✅ **`active_novelty` reads `0.000` at the held-out point at every rank and
///    both quantiles**, while the error runs `1.2 %`–`37 %`. FAIL at every `τ`.
///    Fail-open at a held-out in-hull position, exactly as pre-registered.
/// 5. ✅ Confirmed at all four ranks.
///
/// ### ⚠ What this does NOT establish
///
/// - **One held-out position.** Better than none, which is what §2m had, and still
///   a single point.
/// - ⚠⚠ **The two ensembles have different normalisers, and that disqualifies
///   EVERY cross-arm number here — the median comparison included.** The LOO
///   basis is fitted on 284 snapshots and truncates at `r=116`; the full one on
///   355 at `r=142`, and `snapshot_distance` divides by its own ensemble's cloud
///   radius (`cloud_radius(&train_q)`, computed per arm). So the sound readings
///   are the WITHIN-arm spans — `1.005×` worst step, `1.023×` median, against
///   `31.9×` in error — and every cross-arm figure above (`0.9704` against §2m's
///   `[0.5069, 1.165)`; `4.117e-1` against `4.271e-1`; `0.506` at `0.25a` against
///   `0.970` at `0.50a`) is a DIRECTION. ⚠ The ratio between the two normalisers
///   is not measured; measuring it would take one re-run printing
///   `cloud_radius` per arm, and none of this section's conclusions wait on it.
///   ✅ §2o carries one cross-ensemble reading (`~1.1×`) and labels it a direction
///   rather than a number, which is the form this section should have used.
/// - **The lattice control tests the EXTRAPOLATION end only.**
/// - No timing, no threshold, one fixture, one basis construction.
#[test]
#[ignore = "§4c margin validation — 8 full-order trajectories, ~3 min (see the fn docs)"]
fn the_signal_margin_on_a_held_out_position() {
    let base = Scene::new(GEN_A_OVER_CELL);
    let x_rest = rest_positions(&base.mesh());
    let solver = base.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let tol = SolverConfig::skeleton().tol;
    let n_steps = base.n_steps();
    println!(
        "\nRC\t§4c MARGIN VALIDATION: {} free DOF, {n_steps} steps\nRC\tarm A: train {:?} \\ {{{LOO_HELD_OUT:+.2}}}, score at {LOO_HELD_OUT:+.2}a  (gap 0.50a, ON lattice)\
         \nRC\tarm B: full train, score at {OFF_LATTICE_EXTRAP:+.2}a (3.5 cells, OFF lattice) vs {:+.2}a / {:+.2}a (ON)",
        fd.len(),
        TRAIN_OFFSETS,
        TEST_OFFSETS[2].1,
        TEST_OFFSETS[3].1,
    );

    // ── every full-order trajectory this needs, run once ──
    let mut oracles: Vec<(f64, Oracle)> = Vec::new();
    let mut wanted: Vec<f64> = TRAIN_OFFSETS.to_vec();
    for dx in [OFF_LATTICE_EXTRAP, TEST_OFFSETS[2].1, TEST_OFFSETS[3].1] {
        wanted.push(dx);
    }
    for dx in wanted {
        let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
        let o = run_oracle(sc, &x_rest, NO_TIMING);
        assert!(
            o.arm.failure.is_none(),
            "oracle at {dx:+.2}a failed: {}",
            o.arm.failure.as_deref().unwrap_or(""),
        );
        oracles.push((dx, o));
    }
    let at = |dx: f64| -> &Oracle {
        &oracles
            .iter()
            .find(|(t, _)| (t - dx).abs() < 1e-12)
            .unwrap_or_else(|| panic!("no oracle at {dx:+.2}a"))
            .1
    };

    // ── build one (snapshots, active-union) pair from a chosen set of offsets ──
    let ensemble = |offsets: &[f64]| -> (SnapshotSet, std::collections::BTreeSet<usize>) {
        let mut snaps = SnapshotSet::new(fd.len());
        let mut active = std::collections::BTreeSet::new();
        for &dx in offsets {
            let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
            for (k, x) in at(dx).x.iter().enumerate() {
                snaps.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
                active.extend(active_vertices(x, sc.centre_at(k), sc.d_hat));
            }
        }
        (snaps, active)
    };

    let loo_offsets: Vec<f64> = TRAIN_OFFSETS
        .into_iter()
        .filter(|d| (d - LOO_HELD_OUT).abs() > 1e-12)
        .collect();
    assert_eq!(
        loo_offsets.len(),
        TRAIN_OFFSETS.len() - 1,
        "the held-out offset is not in TRAIN_OFFSETS, so nothing was held out",
    );

    let mut rows: Vec<(&'static str, usize, Arm)> = Vec::new();
    for (tag, train_offsets, scored) in [
        ("A LOO-HELD-OUT", loo_offsets, vec![LOO_HELD_OUT]),
        (
            "B FULL-TRAIN",
            TRAIN_OFFSETS.to_vec(),
            vec![OFF_LATTICE_EXTRAP, TEST_OFFSETS[2].1, TEST_OFFSETS[3].1],
        ),
    ] {
        let (snaps, active) = ensemble(&train_offsets);
        println!(
            "\nRC\t── {tag}: {} offsets, {} snapshots, {} vertices ever active ──",
            train_offsets.len(),
            snaps.len(),
            active.len(),
        );
        let mut seen = std::collections::BTreeSet::new();
        for r in BASIS_RANKS {
            let basis = PodBasis::fit(&snaps, Inner::Mass, &mass, 1.0, r).expect("basis fits");
            if !seen.insert(basis.n_modes()) {
                continue;
            }
            let train_q: Vec<Vec<f64>> = snaps.columns().iter().map(|u| basis.project(u)).collect();
            let radius = cloud_radius(&train_q);
            assert!(radius > 0.0, "zero cloud radius at r={}", basis.n_modes());
            let probe = Probe {
                envelope: training_envelope(&train_q),
                train_q,
                cloud_radius: radius,
                train_active: &active,
                tol,
                d_hat: base.d_hat,
            };
            for &dx in &scored {
                let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
                let arm = run_reduced(
                    sc,
                    &basis,
                    InitialGuess::Inertial,
                    format!("{tag} {dx:+.2}a r={r}→{}", basis.n_modes()),
                    &Ctx {
                        x_rest: &x_rest,
                        fd: &fd,
                        oracle: at(dx),
                        probe: Some(&probe),
                    },
                    NO_TIMING,
                );
                arm.print(n_steps, sc.d_hat);
                rows.push((tag, basis.n_modes(), arm));
            }
        }
    }
    assert_every_arm_produced(&rows, rows.len());

    // ⚠ The first version of this test recorded the signals and PRINTED NONE of
    // them, so pre-registration items 3 and 4 — both about what a signal reads at
    // the held-out point — were unanswerable from its own output. A prediction
    // whose data is not emitted is not a prediction.
    println!(
        "\nRC\t╔═ SIGNALS at each scored arm (worst step / median step)\nRC\t║ {:<30}{:>12}{:>22}{:>22}{:>18}",
        "arm", "relL2", "S1 resid", "S3 snapshot-dist", "S4 novelty"
    );
    for (tag, got, arm) in &rows {
        let cell = |pick: fn(&Signals) -> f64, w: usize| -> String {
            let m = signal(arm, pick, Quantile::Max);
            let d = signal(arm, pick, Quantile::Median);
            match (m, d) {
                (Some(m), Some(d)) => format!("{:>w$}", format!("{m:.3e}/{d:.3e}"), w = w),
                _ => format!("{:>w$}", "—", w = w),
            }
        };
        println!(
            "RC\t║ {:<30}{:>12.3e}{}{}{}",
            format!("{tag} r={got}"),
            arm.max_rel_err,
            cell(|s| s.residual_excess, 22),
            cell(|s| s.snapshot_distance, 22),
            cell(|s| s.active_novelty, 18),
        );
    }
    println!("RC\t╚═");
    println!(
        "\nRC\t★ READ IT AS: (a) does HELD-OUT +0.00a land with +1.50a — same gap, same\n\
         RC\t   lattice status, one interpolating and one extrapolating? If so the\n\
         RC\t   hull framing is a DISTANCE framing wearing a topological name.\n\
         RC\t   (b) does +1.75a fall inside the +1.50a/+2.00a bracket? If not, +0.25a's\n\
         RC\t   three orders are partly a MESH artifact.\n\
         RC\t   (c) does active_novelty read 0 at HELD-OUT +0.00a while the error is\n\
         RC\t   large? That is a fail-open, in one cell."
    );
}

// ── §4c rung 1c: how DENSE must the ensemble be, and does density mean SPACING ──
// ── or COUNT?                                                                  ──

/// The three training sets, all scored at the same held-out point. Chosen as a
/// **2-factor design over the offsets that already have oracles**, so this costs
/// no full-order trajectory the arc has not already paid for:
///
/// | | offsets | trajectories | gap to `+0.00a` |
/// |---|---|---|---|
/// | **A** | `[-1, -0.5, +0.5, +1]` | 4 | `0.5a` |
/// | **B** | `[-0.5, +0.5]` | 2 | `0.5a` |
/// | **C** | `[-1, +1]` | 2 | `1.0a` |
///
/// **A vs B isolates ENSEMBLE SIZE at fixed gap. B vs C isolates GAP at fixed
/// size.** Without both comparisons "denser is better" cannot be resolved into a
/// statement a `ReducedValidityDomain` could carry, because spacing and count move
/// together in every ensemble anyone builds by refining a grid.
const DENSITY_SETS: [(&str, &[f64]); 3] = [
    ("A  4 traj, gap 0.50a", &[-1.0, -0.5, 0.5, 1.0]),
    ("B  2 traj, gap 0.50a", &[-0.5, 0.5]),
    ("C  2 traj, gap 1.00a", &[-1.0, 1.0]),
];

/// **How dense must the training ensemble be — and is "density" SPACING or COUNT?**
///
/// ```text
/// cargo test --release -p sim-soft --test reduced_contact \
///   how_dense_the_training_ensemble_must_be -- --ignored --nocapture
/// ```
///
/// §10 risk 0 carries the domain-coverage half of §4c as untouched, and §5's open
/// item 3 was re-scoped to exactly this question. It is the one a validity domain
/// has to answer in its **Stated** clause: a parameter box is only meaningful if
/// you can say how finely the box was sampled and what that buys.
///
/// Every reduced result in this arc conflates two things, because refining a
/// training grid changes both at once: the SPACING between neighbouring
/// trajectories, and the NUMBER of them. §2n measured one ensemble at one spacing.
/// This separates the two factors on the offsets whose oracles already exist.
///
/// ## Pre-registration (before the first run)
///
/// 1. **If only GAP matters, A ≈ B and C is worse.** Then a validity domain need
///    only state its sampling PITCH, which is a cheap thing to state and to check
///    online — the nearest-snapshot distance already measures it.
/// 2. **If SIZE matters, A beats B at the same gap.** Then the domain must state
///    the ensemble's cardinality too, and `snapshot_distance` — which sees only the
///    nearest neighbour — is blind to the thing that matters, which would be a
///    second strike against the candidate §2n already disqualified as an accuracy
///    signal.
/// 3. ★ **No prediction on which.** The POD basis grows richer with more
///    trajectories even at fixed spacing, so both mechanisms are real; which
///    dominates at this gap is the measurement.
/// 4. **`snapshot_distance` should read the SAME for A and B and larger for C**,
///    since it is close to linear in gap (§2n measured `0.506` at `0.25a` and
///    `0.970` at `0.50a`). ⚠ Across ensembles its normaliser changes, so this is
///    the weakest of the four and is read as a direction, not a number.
///
/// ## Measured — 2026-08-25, 5 oracles, `82.7 s`
///
/// | ensemble | `r=20` | `r=40` | at its own ceiling |
/// |---|---:|---:|---:|
/// | **A** 4 traj, gap `0.50a` | 3.733e-1 | 1.406e-1 | 1.172e-2 (`r=116`) |
/// | **B** 2 traj, gap `0.50a` | 7.949e-1 | 3.790e-1 | 1.394e-1 (`r=64`) |
/// | **C** 2 traj, gap `1.00a` | 8.632e-1 | 4.448e-1 | 3.058e-1 (`r=65`) |
///
/// ### ★★★ SIZE dominates GAP, and pre-registration item 2 is the branch taken
///
/// At matched rank:
///
/// - **A vs B — doubling the ENSEMBLE at fixed gap buys `2.13×` / `2.70×`.**
/// - **B vs C — doubling the GAP at fixed size costs `1.09×` / `1.17×`.**
///
/// So "denser is better" resolves into two effects of unequal weight, and the one
/// every online signal here measures — the gap — is the WEAKER of the two.
///
/// ★★ **And size compounds, because it sets the rank CEILING.** A's 284 snapshots
/// retain 116 modes; B's 142 retain 64. (C's 142 retain 65 — the ceiling is set by
/// the spectrum, not by the snapshot count alone, which is why the two 142-snapshot
/// ensembles do not land on the same one.) B cannot go past `r=64` no matter what is
/// asked of it, so at their respective ceilings A beats B by **`11.9×`** — far
/// more than the `2.1–2.7×` the basis quality alone accounts for. A small ensemble
/// is penalised twice: a worse basis at every rank, and a lower rank to stop at.
///
/// ### ⇒ Second strike against the surviving candidate
///
/// A and B are **the same gap** and differ `2.13–2.70×` in error. What the domain
/// signals read:
///
/// - `active_novelty`: **`0.000` for both, at every rank.** Identical.
/// - `snapshot_distance` median: A `0.411–0.420`, B `0.447–0.500` — about `1.1×`,
///   ⚠ and across ensembles its normaliser changes, so even that is weak evidence
///   rather than a measurement.
///
/// **Neither can express ensemble size.** §2n showed `snapshot_distance` is blind
/// to RESOLUTION (rank-independent by construction); this shows it is blind to
/// ENSEMBLE SIZE as well. It reads the sampling PITCH, which is measured here to be
/// the smaller of the two things that determine the error.
///
/// ⇒ **A `ReducedValidityDomain` must state CARDINALITY, not only pitch** — and
/// the online candidate cannot check the clause that matters more.
///
/// ### ✅ C is the positive control, and it tells `active_novelty` apart from noise
///
/// C fires: `0.250 / 0.182 / 0.167`, where A and B read exactly zero. `[-1, +1]`
/// leaves a genuine hole at the centre that `±0.5a` sweeps through. So
/// `active_novelty` does measure something real — **COVERAGE** — and it fires
/// exactly when coverage fails. That is the correct semantics for it, and this
/// sweep measures coverage to be the thing that is NOT dominating.
///
/// ### Pre-registration
///
/// 1. ⛔ Not the branch taken — gap is not the only factor.
/// 2. ✅ **SIZE matters, and dominates.** With the consequence stated there: the
///    domain must carry cardinality, and a nearest-neighbour signal cannot.
/// 3. Both mechanisms are real; SIZE won at this gap by `~2×`.
/// 4. ⚠ Read as a direction only, as flagged: B and C read higher than A, but the
///    cross-ensemble normaliser makes the numbers non-comparable.
///
/// ### ⚠ What this does NOT establish
///
/// - **One held-out point, one gap pair, one size pair.** The `2.13–2.70×` and
///   `1.09–1.17×` are two points each, not a scaling law.
/// - **Size and snapshot COUNT are confounded here** — B and C have 142 snapshots
///   because they have 2 trajectories of 71 steps. Nothing separates "more
///   trajectories" from "more snapshots"; a longer ramp at fewer offsets would.
/// - **The rank ceilings differ**, so only the matched-rank rows isolate a factor.
///   The own-ceiling column mixes basis quality with the ceiling itself, which is
///   why it is reported separately and named as compounding.
#[test]
#[ignore = "§4c ensemble-density sweep — 5 full-order trajectories, ~2 min (see the fn docs)"]
fn how_dense_the_training_ensemble_must_be() {
    let base = Scene::new(GEN_A_OVER_CELL);
    let x_rest = rest_positions(&base.mesh());
    let solver = base.solver(InitialGuess::PreviousState);
    let fd = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();
    let tol = SolverConfig::skeleton().tol;
    let n_steps = base.n_steps();
    let scored_at = LOO_HELD_OUT;

    println!(
        "\nRC\t§4c DENSITY SWEEP: {} free DOF, all arms scored at {scored_at:+.2}a\n\
         RC\tA vs B isolates ENSEMBLE SIZE at fixed gap; B vs C isolates GAP at fixed size.",
        fd.len(),
    );

    let mut oracles: Vec<(f64, Oracle)> = Vec::new();
    for dx in TRAIN_OFFSETS {
        let o = run_oracle(Scene::at_offset(GEN_A_OVER_CELL, dx), &x_rest, NO_TIMING);
        assert!(
            o.arm.failure.is_none(),
            "oracle at {dx:+.2}a failed: {}",
            o.arm.failure.as_deref().unwrap_or(""),
        );
        oracles.push((dx, o));
    }
    let at = |dx: f64| -> &Oracle {
        &oracles
            .iter()
            .find(|(t, _)| (t - dx).abs() < 1e-12)
            .unwrap_or_else(|| panic!("no oracle at {dx:+.2}a"))
            .1
    };

    let mut rows: Vec<(&'static str, usize, Arm)> = Vec::new();
    for (tag, offsets) in DENSITY_SETS {
        assert!(
            offsets.iter().all(|d| (d - scored_at).abs() > 1e-12),
            "{tag} contains the scored offset, so it is not held out at all",
        );
        let mut snaps = SnapshotSet::new(fd.len());
        let mut active = std::collections::BTreeSet::new();
        for &dx in offsets {
            let sc = Scene::at_offset(GEN_A_OVER_CELL, dx);
            for (k, x) in at(dx).x.iter().enumerate() {
                snaps.push(&SnapshotSet::free_displacement(x, &x_rest, &fd));
                active.extend(active_vertices(x, sc.centre_at(k), sc.d_hat));
            }
        }
        let gap = offsets
            .iter()
            .map(|d| (d - scored_at).abs())
            .fold(f64::INFINITY, f64::min);
        println!(
            "\nRC\t── {tag}: {} snapshots, {} vertices ever active, true gap {gap:.2}a ──",
            snaps.len(),
            active.len(),
        );
        let mut seen = std::collections::BTreeSet::new();
        for r in BASIS_RANKS {
            let basis = PodBasis::fit(&snaps, Inner::Mass, &mass, 1.0, r).expect("basis fits");
            if !seen.insert(basis.n_modes()) {
                continue;
            }
            let train_q: Vec<Vec<f64>> = snaps.columns().iter().map(|u| basis.project(u)).collect();
            let radius = cloud_radius(&train_q);
            assert!(radius > 0.0, "zero cloud radius at r={}", basis.n_modes());
            let probe = Probe {
                envelope: training_envelope(&train_q),
                train_q,
                cloud_radius: radius,
                train_active: &active,
                tol,
                d_hat: base.d_hat,
            };
            let sc = Scene::at_offset(GEN_A_OVER_CELL, scored_at);
            let arm = run_reduced(
                sc,
                &basis,
                InitialGuess::Inertial,
                format!("{tag} r={r}→{}", basis.n_modes()),
                &Ctx {
                    x_rest: &x_rest,
                    fd: &fd,
                    oracle: at(scored_at),
                    probe: Some(&probe),
                },
                NO_TIMING,
            );
            arm.print(n_steps, sc.d_hat);
            rows.push((tag, basis.n_modes(), arm));
        }
    }
    assert_every_arm_produced(&rows, rows.len());

    println!(
        "\nRC\t╔═ held-out error at {scored_at:+.2}a, and what the domain signal reads\n\
         RC\t║ {:<26}{:>12}{:>22}{:>16}",
        "ensemble", "relL2", "snapshot-dist max/med", "novelty max"
    );
    for (tag, got, arm) in &rows {
        let s3 = signal(arm, |s| s.snapshot_distance, Quantile::Max);
        let s3m = signal(arm, |s| s.snapshot_distance, Quantile::Median);
        let s4 = signal(arm, |s| s.active_novelty, Quantile::Max);
        println!(
            "RC\t║ {:<26}{:>12.3e}{:>22}{:>16}",
            format!("{tag} r={got}"),
            arm.max_rel_err,
            match (s3, s3m) {
                (Some(a), Some(b)) => format!("{a:.3e}/{b:.3e}"),
                _ => "—".to_owned(),
            },
            s4.map_or_else(|| "—".to_owned(), |v| format!("{v:.3e}")),
        );
    }
    println!(
        "RC\t║ A vs B at the SAME gap ⇒ what ENSEMBLE SIZE alone buys.\n\
         RC\t║ B vs C at the SAME size ⇒ what GAP alone costs.\n\
         RC\t╚═"
    );
}
