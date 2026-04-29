//! V-5 — drop-and-rest hygiene: soft sphere released above a rigid plane,
//! integrated under gravity until kinetic energy below threshold.
//!
//! Phase 5 scope memo §1 V-5 + §8 commit 10 (`phase_5_penalty_contact_scope.md`).
//! First Phase 5 test that exercises **dynamic** integration with gravity —
//! V-1 / V-3a / V-3 are quasi-static (`STATIC_DT = 1.0`), V-5 is dt-resolved
//! transient. The gravity wiring (`SolverConfig::gravity_z`) introduced in
//! commit 10 is exercised end-to-end here for the first time.
//!
//! ## What V-5 catches
//!
//! Per scope memo §1 V-5: "Soft sphere released above a `RigidPlane`, gravity
//! loaded, integrated for `n_steps` until kinetic energy `< ε_KE_threshold`.
//! Steady state reached within documented step count. ... No energy
//! *injection* — total energy at any step is bounded above by initial
//! potential + work done by external loads. Penalty's known oscillation
//! pathology (book §00 §00) is permitted as a documented behavior; not a
//! failure."
//!
//! Three properties asserted, each isolating a distinct failure mode:
//!
//! - **No energy injection per step** — at every step the maximum
//!   per-vertex velocity magnitude stays below the gravitational
//!   freefall bound `sqrt(2 g h)` plus a generous safety margin. A
//!   sign-flipped gravity, an inverted contact gradient, or a faulty
//!   Newton step would inject kinetic energy and trip this assert
//!   long before reaching the rest gate. Penalty's oscillation
//!   pathology produces *bounded* (not growing) overshoot during
//!   contact — backward-Euler's numerical damping pulls magnitudes
//!   back down each step.
//! - **Reaches rest within budget** — at the final step every vertex's
//!   velocity magnitude is below `KE_REST_THRESHOLD`. Backward-Euler
//!   is dissipative; the analytic per-step amplitude factor at our
//!   parameters is `1 / (1 + ω²·dt²) ≈ 7.6e-4` (oscillation amplitude
//!   shrinks `~1300×` per step at the penalty oscillator frequency),
//!   so a few hundred post-contact steps drop kinetic energy below
//!   the `1e-2 m/s`-magnitude rest threshold.
//! - **Sphere descended** — final mean z-coordinate is below the
//!   initial release height (gravity acted as expected). Sanity gate
//!   catching a frozen-x configuration where the solver returned
//!   `x_prev` unmodified.
//!
//! ## Why `dt = 1e-3` s, `n_steps = 1000`
//!
//! Time-to-impact in pure freefall from `h = 5 cm`: `t_c = sqrt(2h/g) ≈
//! 0.10 s = 100 steps at dt = 1e-3 s`. Post-contact decay: at penalty
//! stiffness `κ = 1e4 N/m`, sphere total mass `M ≈ ρ · V_sphere ≈
//! 4.32e-3 kg` (silicone-class `ρ = 1030 kg/m³` at `R = 1 cm`), and
//! `~561` referenced vertices, the per-vertex lumped mass `m_v ≈ M /
//! n_ref ≈ 7.7e-6 kg`. Penalty-oscillator frequency `ω = sqrt(κ /
//! m_v) ≈ 3.6e4 rad/s`; backward-Euler per-step amplitude factor `1
//! / (1 + ω²·dt²) ≈ 7.6e-4` knocks oscillation amplitude down `~1300×`
//! per step in the post-contact regime, so a few hundred steps brings
//! the system to rest. `n_steps = 1000` (1 s simulated total) gives
//! wide headroom.
//!
//! Penalty oscillation period at this `(κ, m_v)`: `2π / ω ≈ 1.7e-4 s ≈
//! 0.17 ms`, i.e., `dt / T ≈ 5.8` oscillation periods per integrator
//! step — gross dynamics captured, fine penalty-oscillation structure
//! unresolved (and per scope memo "permitted as a documented behavior").
//!
//! ## RB modes — auto-pin handles them
//!
//! The `dropping_sphere` helper ships empty `pinned_vertices`. In
//! statics that produced the V-3 `sphere_on_plane` stall (commit 9
//! lesson) — but in dynamics the `M / dt²` Tikhonov regulariser is per-
//! DOF positive on every referenced vertex, so the free-DOF Hessian is
//! SPD even at rest config with zero contact engagement. RB modes don't
//! need a four-pin equator set in V-5 (orphan vertices auto-pin via
//! `CpuNewtonSolver::new`'s `effective_pinned` step; referenced
//! vertices stay free with mass-driven inertial regularization).
//!
//! ## Helper extensions surfaced at first use
//!
//! Per `feedback_no_reflexive_defer` and the commit-9 V-3 / commit-8
//! V-3a precedent (helper docstring claims may need empirical
//! validation), the `dropping_sphere` helper at
//! `SoftScene::dropping_sphere` was validated end-to-end here. No
//! drift surfaced — the empty-pinned design works under dynamics for
//! the documented reasons (M/dt² Tikhonov regularization).

#![allow(
    // The helper signature returns a `Result<4-tuple, MeshingError>`.
    // Mirror of `concentric_lame_shells.rs` / `hertz_sphere_plane.rs`
    // / V-3a precedent.
    clippy::expect_used,
    // V-5 single-fn structure with (a) constants, (b) setup, (c)
    // step-loop with per-step asserts, (d) final asserts, (e)
    // diagnostic eprintln legitimately exceeds clippy's 100-line cap
    // once the loop logic is inlined. Mirrors V-3 / V-3a precedent.
    clippy::too_many_lines
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, MaterialField, Mesh, PenaltyRigidContactSolver, SceneInitial,
    SdfMeshedTetMesh, SoftScene, Solver, SolverConfig, Tet4, VertexId, referenced_vertices,
};

// ── Scene constants ──────────────────────────────────────────────────────

/// Sphere radius (1 cm) — mirror V-3 commit-9 RADIUS for parameter
/// consistency across the contact-active regression net.
const RADIUS: f64 = 1.0e-2;

/// Cell size (3 mm) — matches V-3 commit-9 coarsest level (~2.2k tets
/// at this `R` per V-3 empirical). Debug-mode-feasible step latency at
/// this resolution; finer resolution is gratuitous for V-5's hygiene
/// scope (this isn't an analytic-comparison gate).
const CELL_SIZE: f64 = 3.0e-3;

/// Release height above the rigid plane (5 cm). Must exceed `RADIUS +
/// PENALTY_DHAT_DEFAULT = 1.1 cm` — picked at 5× to give a long
/// freefall trajectory before contact, exercising gravity wiring
/// across many steps before the contact dispatch fires.
const RELEASE_HEIGHT: f64 = 5.0e-2;

/// Lamé pair `(μ, λ)` — mirror V-3 commit-9 (Ecoflex 00-30 + 15 wt%
/// carbon-black composite, `ν = 0.4` compressible Neo-Hookean per
/// Phase 4 IV-3 / IV-5 + V-3 precedent).
const MU: f64 = 2.0e5;
const LAMBDA: f64 = 8.0e5;

/// Gravitational acceleration along `+ẑ` (`m/s²`). Negative = downward.
/// Earth standard `9.81 m/s²` per scope memo §1 V-5's "gravity loaded"
/// framing.
const GRAVITY: f64 = -9.81;

/// Time step (1 ms). See module docstring "Why dt = 1e-3 s" section.
const DT: f64 = 1.0e-3;

/// Total step count (1000 → 1 s simulated). Time-to-impact ~100 ms;
/// post-contact decay ~few hundred steps; ~5× headroom on the rest
/// gate per the docstring sizing.
const N_STEPS: usize = 1000;

/// Newton iteration cap — bumped from skeleton's 10 to 50 for transient
/// integration headroom (penalty oscillation during contact + Newton
/// step under steep `M/dt²` regularization can take 5-15 iters per
/// step). Mirrors V-3a's `MAX_NEWTON_ITER = 50` for consistency.
const MAX_NEWTON_ITER: usize = 50;

/// Per-vertex velocity magnitude floor for "rest" (`m/s`). Below this
/// the system is judged to have reached steady state. `1 cm/s` is
/// generous — at our `(κ, m_v)` the BE-damped envelope falls below
/// `1e-4 m/s` long before step 1000.
const KE_REST_THRESHOLD: f64 = 1.0e-2;

/// Multiplier on the freefall velocity bound `sqrt(2 g h)` for the
/// per-step "no energy injection" gate. `1.5×` accommodates penalty's
/// documented bounded-oscillation overshoot (book §00 §00) during
/// contact. A faulty integrator that injects energy would blow past
/// any reasonable multiplier; this isn't a tight gate, just a sanity
/// gate.
const ENERGY_BOUND_SAFETY: f64 = 1.5;

// ── Helpers ──────────────────────────────────────────────────────────────

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

/// Maximum per-vertex velocity magnitude over a flat `[3·N]` velocity
/// buffer. Walks vertices in natural order; takes per-vertex magnitude
/// `sqrt(vx² + vy² + vz²)`.
fn max_vertex_velocity(v_flat: &[f64]) -> f64 {
    debug_assert!(v_flat.len().is_multiple_of(3));
    let mut max = 0.0_f64;
    let n_vertices = v_flat.len() / 3;
    for v in 0..n_vertices {
        let vx = v_flat[3 * v];
        let vy = v_flat[3 * v + 1];
        let vz = v_flat[3 * v + 2];
        let mag = vx.mul_add(vx, vy.mul_add(vy, vz * vz)).sqrt();
        if mag > max {
            max = mag;
        }
    }
    max
}

/// Mean z-coordinate over a referenced-vertices subset of a flat
/// `[3·N]` position buffer. Walks the supplied `referenced` slice;
/// orphan vertices (auto-pinned at `x_prev` by `CpuNewtonSolver::new`'s
/// `effective_pinned` step) are excluded so the mean tracks the
/// actual sphere body, not the static auto-pinned orphan cohort.
/// Mirrors V-3 commit-9's `referenced_vertices` filter pattern.
fn mean_referenced_z(x_flat: &[f64], referenced: &[VertexId]) -> f64 {
    debug_assert!(x_flat.len().is_multiple_of(3));
    let z_sum: f64 = referenced
        .iter()
        .map(|&v| x_flat[3 * (v as usize) + 2])
        .sum();
    // Referenced count fits f64 exactly for any sim-soft mesh size.
    #[allow(clippy::cast_precision_loss)]
    let mean = z_sum / referenced.len() as f64;
    mean
}

// ── Tests ────────────────────────────────────────────────────────────────

// Release-mode-only gate per `feedback_release_mode_heavy_tests` and the
// V-3 commit-9 precedent. V-5 runtime is ~22 s release-mode at the
// canonical (CELL_SIZE = 3 mm, N_STEPS = 1000) parameters; debug-mode
// inflation at this resolution would push runtime into the multi-minute
// range, against the CI 30-min total budget per scope memo §4. The
// `#[cfg_attr(debug_assertions, ignore)]` pattern mirrors
// `hertz_sphere_plane.rs:621-625`. Like V-3, sim-soft is NOT in the CI
// tests-release matrix today; CI followup to add `cargo test --release
// -p sim-soft --test contact_drop_rest` to `quality-gate.yml`'s tests-
// release job is deferred to commit 11 alongside the V-3 followup.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — heavy drop-and-rest at 1000 steps × dynamic Newton (~22 s release, \
              multi-minute debug); rerun with `cargo test --release` to include"
)]
#[test]
fn v_5_dropping_sphere_reaches_rest_no_energy_injection() {
    let (mesh, bc, initial, contact) =
        SoftScene::dropping_sphere(RADIUS, CELL_SIZE, RELEASE_HEIGHT, material_field())
            .expect("dropping_sphere should mesh successfully at canonical params");

    let n_dof = 3 * mesh.n_vertices();
    let n_tets = mesh.n_tets();

    // Snapshot referenced (non-orphan) vertices BEFORE moving the mesh
    // into the solver. Orphans auto-pin per `backward_euler.rs:270-
    // 298`'s `effective_pinned`; their z stays frozen at `x_prev` and
    // would bias the mean-z descent gate downward by orders of
    // magnitude relative to the actual sphere descent. Mirrors V-3
    // commit-9's `referenced` snapshot at `hertz_sphere_plane.rs:519`.
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();
    assert!(
        n_referenced > 0,
        "dropping_sphere mesh has zero referenced vertices at cell_size = {CELL_SIZE}",
    );

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAVITY;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let SceneInitial { x_prev, v_prev } = initial;
    let initial_mean_z = mean_referenced_z(x_prev.as_slice(), &referenced);
    let mut x_state: Vec<f64> = x_prev.as_slice().to_vec();
    let mut v_state: Vec<f64> = v_prev.as_slice().to_vec();

    // Freefall velocity bound: `|v|_z ≤ sqrt(2 g h)` at impact in pure
    // freefall (energy conservation). Penalty + BE adds bounded
    // oscillation overshoot on top — see ENERGY_BOUND_SAFETY const.
    let v_freefall_bound = (-2.0 * GRAVITY * RELEASE_HEIGHT).sqrt() * ENERGY_BOUND_SAFETY;

    let theta = Tensor::from_slice(&[], &[0]);

    let mut max_v_observed = 0.0_f64;
    let mut max_iter_observed = 0_usize;

    for step_idx in 0..N_STEPS {
        let x_in = Tensor::from_slice(&x_state, &[n_dof]);
        let v_in = Tensor::from_slice(&v_state, &[n_dof]);
        let step = solver.replay_step(&x_in, &v_in, &theta, DT);

        // v_final = (x_final - x_state) / dt — backward-Euler velocity
        // update.
        let v_new: Vec<f64> = step
            .x_final
            .iter()
            .zip(x_state.iter())
            .map(|(xf, xp)| (xf - xp) / DT)
            .collect();

        let v_max_step = max_vertex_velocity(&v_new);
        if v_max_step > max_v_observed {
            max_v_observed = v_max_step;
        }
        if step.iter_count > max_iter_observed {
            max_iter_observed = step.iter_count;
        }

        // ── Per-step "no energy injection" gate ─────────────────────
        //
        // |v|_max at any step must stay below the gravitational
        // freefall bound × safety. A sign-flipped gravity, inverted
        // contact gradient, or runaway Newton step would explode |v|
        // by orders of magnitude relative to this bound; penalty's
        // documented bounded oscillation produces overshoot within the
        // safety margin (BE damping pulls envelope down each step).
        assert!(
            v_max_step < v_freefall_bound,
            "step {step_idx}: |v|_max = {v_max_step:.4} m/s exceeds freefall bound \
             {v_freefall_bound:.4} m/s — energy injection detected. The dynamics should be \
             bounded by gravitational free fall plus penalty's bounded-oscillation overshoot.",
        );

        // ── Per-step finite-state sanity ──────────────────────────────
        //
        // NaN / inf in x_final flags Newton blowup masked by Armijo
        // backtrack (residual decreases but config is garbage).
        for (i, &x) in step.x_final.iter().enumerate() {
            assert!(
                x.is_finite(),
                "step {step_idx}: x_final[{i}] = {x} is not finite — Newton diverged",
            );
        }

        x_state = step.x_final;
        v_state = v_new;
    }

    let v_max_final = max_vertex_velocity(&v_state);
    let final_mean_z = mean_referenced_z(&x_state, &referenced);

    // Cast safe: `N_STEPS` is a small literal; conversion to f64 is
    // loss-free.
    #[allow(clippy::cast_precision_loss)]
    let sim_t = DT * N_STEPS as f64;

    eprintln!(
        "v_5 drop-and-rest: n_tets = {n_tets}, n_dof = {n_dof}, n_referenced = {n_referenced}, \
         GRAVITY = {GRAVITY} m/s², DT = {DT} s, N_STEPS = {N_STEPS} \
         (simulated time = {sim_t:.3} s); \
         RELEASE_HEIGHT = {RELEASE_HEIGHT} m, freefall bound × safety = {fb:.3} m/s; \
         |v|_max observed across all steps = {max_v_observed:.4} m/s; \
         max Newton iters per step = {max_iter_observed}; \
         |v|_max at final step = {v_max_final:.6e} m/s (rest threshold = {KE_REST_THRESHOLD:.0e}); \
         referenced mean z initial = {initial_mean_z:.4e} m → final = {final_mean_z:.4e} m \
         (Δz = {dz:.4e} m)",
        fb = v_freefall_bound,
        dz = final_mean_z - initial_mean_z,
    );

    // ── Reaches rest within budget ─────────────────────────────────────
    //
    // Backward-Euler is dissipative; numerical damping carries the
    // system to rest with high per-step damping fraction at our `(κ,
    // m_v, dt)` parameters. Failure to reach rest within `N_STEPS`
    // suggests either (a) BE damping rate is much lower than expected
    // (κ misconfigured?), (b) dt × N_STEPS doesn't cover impact +
    // decay (sizing mistake), or (c) penalty oscillation amplitude is
    // not decaying (numerical-damping bug in the contact dispatch
    // path).
    assert!(
        v_max_final < KE_REST_THRESHOLD,
        "final-step |v|_max = {v_max_final:.4e} m/s ≥ rest threshold {KE_REST_THRESHOLD:.0e} m/s \
         — sphere has not reached steady state within N_STEPS = {N_STEPS} (simulated time = \
         {sim_t:.3} s). Investigate BE damping rate / contact decay envelope.",
    );

    // ── Sphere descended ──────────────────────────────────────────────
    //
    // Final mean z below initial mean z. Strict `<` rather than `<=`
    // catches a frozen-x bug where solver returns x_prev unmodified.
    // No quantitative bound — mean z is biased by orphan vertices
    // staying auto-pinned at initial release height; the qualitative
    // gate is just "moved at all in -ẑ direction."
    assert!(
        final_mean_z < initial_mean_z,
        "final mean_z = {final_mean_z:.4e} m not below initial mean_z = {initial_mean_z:.4e} m \
         — sphere did not descend under gravity. Likely cause: gravity wiring not active OR \
         solver returning frozen x_prev.",
    );
}
