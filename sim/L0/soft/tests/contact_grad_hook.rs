//! V-7 — penalty contact differentiability hook: FD-self-consistency
//! of `∂F_R/∂κ_pen` in the robustly-in-contact band.
//!
//! Phase 5 scope memo §1 V-7 + §8 commit 11. **Forward-looking
//! differentiability hook** — sets up Part 6 / Part 10 differentiable
//! design with contact parameters; does not pretend penalty is
//! differentiable across the active-set boundary. Phase H IPC removes
//! this restriction structurally via the logarithmic barrier.
//!
//! ## Why this gate
//!
//! Book Part 4 §00 §00 ([`00-penalty.md`] Claim 1 elaboration) is
//! cited verbatim:
//!
//! > Gradient-based optimization through a penalty-contact simulator
//! > is therefore limited to configurations that are robustly
//! > in-contact or robustly out-of-contact; the transition band is a
//! > forbidden zone.
//!
//! V-7 tests the *positive* half of that statement — penalty IS
//! differentiable when every active pair sits robustly inside the
//! contact band (`d < d̂ - 2 ε_FD-position`). The *negative* half
//! (transition-band singularity at `d = d̂`) is what Phase H IPC
//! structurally removes; V-7 does not exercise it.
//!
//! ## Reward functional choice
//!
//! Scalar `reward(κ_pen) = F_R` — total reaction force on the rigid
//! plane at converged steady-state of the V-3a override compressive-
//! block scene. Newton's 3rd-law partner of the penalty force on the
//! top face. Reconstructed manually from `x_final` + `κ` + `d̂` (per
//! [`penalty_compressive_block.rs`] precedent — `PenaltyRigidContact`
//! moves into the solver at construction so `contact.gradient(...)`
//! post-step is unreachable).
//!
//! `F_R(κ)` captures **both** explicit κ-dependence (the `κ` multiplier
//! in the per-pair penalty force) **and** implicit κ-dependence via
//! `x_final` (which shifts with κ at fixed external loading by
//! force-balance). The full `∂F_R/∂κ` is `∂F_R/∂κ|_explicit +
//! ∂F_R/∂x · ∂x_final/∂κ` with `∂x_final/∂κ = -A⁻¹ · ∂r/∂κ` via IFT —
//! exactly what FD probes by re-running Newton at perturbed κ.
//!
//! ## Why FD-self-consistency, not analytic comparison
//!
//! Phase 4 IV-8 ([`material_grad_hook.rs`]) set the canonical Phase-N
//! differentiability-hook pattern: FD agreement at two step sizes at
//! the 5-digit bar demonstrates the map IS smooth at the perturbation
//! scales the eventual reverse-mode adjoint will consume. That **is**
//! the V-7 "FD-stable" gate.
//!
//! Hand-derived analytic `∂F_R/∂κ` requires plumbing the IFT adjoint at
//! κ — the Newton Hessian factor `A` cached at convergence contracted
//! against `∂r/∂κ`, mirroring Part 6 §02's adjoint formula for
//! θ-driven parameters. That plumbing is Part 6 / Phase H
//! differentiable-design work, not Phase 5's hygiene scope (Decision
//! K — no new γ-locked API types in Phase 5).
//!
//! No closed-form for `F_R(κ)` exists at V-3a's mixed BC (commit-8
//! deviation 1: bottom full-pin / sides free / top z-pressed has no
//! clean analytic). The "FD-vs-analytic" framing in scope memo §1 V-7
//! maps to "FD demonstrates the gradient is well-defined" — the same
//! interpretation IV-8 took, where the "analytic" leg was FD-of-
//! closed-form-Lamé. For V-7 the analytic leg is degenerate; the
//! self-consistency leg is load-bearing.
//!
//! ## Robustly-in-contact band condition
//!
//! Every active pair must satisfy `d < d̂ - 2 ε_FD-position`, where
//! `ε_FD-position` is the FD step's effective position-perturbation —
//! otherwise the κ-perturbation could flip an active-set membership
//! across the `d = d̂` discontinuity, invalidating the smooth-
//! derivative assumption.
//!
//! Conservative linearization: at robustly-in-contact, force-balance
//! `κ · N_active · (d̂ - d_eq) ≈ F_elastic(x_final)` gives
//! `δd_eq / d_eq ≈ -δκ / κ` (treating elastic force ≈ const at small
//! κ-perturbations). So `ε_FD-position ≈ h · d̂ / κ`. At commit-8
//! V-3a override `n = 4` regime: average per-pair band margin
//! `≈ F_R / (κ · N_active) ≈ 0.186 / (1e4 · 25) ≈ 7.4e-7 m`; empirical
//! **minimum** band margin across the 25 active pairs `≈ 1.9e-7 m`
//! (top-face vertices have non-uniform depth at converged steady-state
//! since the bottom-pin's lateral confinement decays from corners
//! inward). At `h_rel = 1e-3` the band-margin floor is
//! `2 · ε_FD-position = 2 · 1e-3 · 1e-5 = 2e-8 m` — the empirical min
//! exceeds it by `~9×`. Test asserts the band condition at every active
//! pair across the **base κ + four perturbed κ** configs.
//!
//! ## Step size derivation
//!
//! Central FD truncation `O(h²)` + roundoff `O(ε_f64 / h)` for `f64`.
//! Optimal step at `h ≈ ε_f64^(1/3) · |κ| ≈ 6e-6 · κ ≈ 6e-2` at
//! `κ = 1e4`. Chosen pair `h_rel ∈ {1e-3, 1e-4}` (`ε_κ ∈ {10, 1}` N/m)
//! sits above the optimum in the truncation-dominated regime; pairwise
//! agreement at the 5-digit bar bounds the coarse-step truncation
//! itself, demonstrating smooth `F_R(κ)` over the perturbation range.
//! Both step sizes are safely above the Newton-tol noise floor
//! (`tol = 1e-10` × effective stiffness `~1e-10` N at `F_R ≈ 0.18` N);
//! empirical FD numerator `δF_R ~ ε_κ · ∂F_R/∂κ ~ 1 · 2.8e-7 ~ 3e-7` N
//! at `h_rel = 1e-4` gives `~3` orders of headroom from the noise
//! floor. Mirrors IV-8 step-size structure verbatim.
//!
//! ## Why one refinement level only
//!
//! V-7 is forward-looking differentiability hygiene, not a multi-
//! refinement scientific gate. `n_per_edge = 4` (V-3a override regime)
//! gives 25 active pairs with average band margin `~7.4e-7 m` (min
//! `~1.9e-7 m`); five `replay_step` calls at sub-second debug-mode
//! runtime per call. Mirror of V-6's single-config κ-scan structure
//! (commit 10).
//!
//! ## Forward path: Part 6 + Phase H IPC
//!
//! When the IFT adjoint is plumbed for κ-driven parameters (Part 6 §02
//! plus Phase H IPC adjoint-through-barrier extension), this test's FD
//! baseline IS the gradcheck reference — the scalar `∂F_R/∂κ` value
//! computed by the test fn's central-FD over [`reaction_at`] becomes
//! the right-hand side of an analytic-vs-FD assertion mirroring II-2 /
//! III-3 / IV-8 Gate 2.
//! Phase H IPC's logarithmic barrier `−κ · log(d / d̂)` extends
//! differentiability to the full domain (the barrier blows up smoothly
//! as `d → 0`); V-7's robustly-in-contact restriction lifts at that
//! point.

#![allow(
    // Direct gradient comparisons + reconstructed F_R against FD-of-
    // F_R; not bit-pattern checks. Mirrors `material_grad_hook.rs`
    // precedent.
    clippy::float_cmp,
    // Inline `.expect()` on `Tensor::from_slice` results — mirrors
    // `material_grad_hook.rs` + `penalty_compressive_block.rs`
    // canonical-test convention.
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SoftScene, Solver, SolverConfig, Tet4,
    Vec3, VertexId,
};

// ── Scene constants — V-3a override regime (mirrors penalty_compressive_block.rs) ──

/// Cube edge length (1 cm). Scope memo §9 V-3a recommendation.
const EDGE_LEN: f64 = 0.01;

/// Rigid-plane axial displacement (0.05 mm). V-3a override per commit-8
/// deviation 2 — see [`penalty_compressive_block`] module docstring.
const DISPLACEMENT: f64 = 5.0e-5;

/// Refinement level — single config for V-7 hygiene scope. n=4 gives 25
/// active pairs with average band margin `~7.4e-7 m` (min `~1.9e-7 m`);
/// 5×5×5 = 125 vertices, sub-second debug-mode runtime.
const N_PER_EDGE: usize = 4;

/// Lamé pair `(μ, λ)` — Phase 4 IV-3 / IV-5 default Ecoflex-class
/// compressible `NeoHookean` (`λ = 4 μ` ⇒ `ν = 0.4`). Mirrors V-3a.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// Base penalty stiffness — `PENALTY_KAPPA_DEFAULT` per scope memo
/// Decision J's V-may-tune authority (V-3a tunes only `d̂` and `δ`; `κ`
/// stays at default). FD perturbations probe `∂F_R/∂κ` at this base.
const KAPPA_BASE: f64 = 1.0e4;

/// V-3a-local penalty contact band — 100× smaller than default per
/// commit-8 deviation 2. Brings cold-start residual `κ · (d̂ + δ) ≈ 0.6`
/// N per top-face vertex below the tet-inversion threshold; preserves
/// V-3a regime safety at every perturbed `κ` config V-7 evaluates.
const D_HAT_OVERRIDE: f64 = 1.0e-5;

/// Static-equilibrium time-step — large `dt` damps the inertial Tikhonov
/// regulariser `M / dt²` to negligible relative magnitude, yielding
/// pure-static root-find. Mirrors V-3a's `STATIC_DT`.
const STATIC_DT: f64 = 1.0;

/// Newton iteration cap — mirrors V-3a (50). Newton typically takes
/// `3-5` iters per pass under V-3a override; cap leaves wide margin
/// against perturbation-induced regime drift.
const MAX_NEWTON_ITER: usize = 50;

// ── FD step sizes — mirrors IV-8 (`material_grad_hook.rs`) verbatim ─────────

/// Coarse FD step relative to `KAPPA_BASE`. At `κ = 1e4` → `ε_κ = 10` N/m.
const FD_H_REL_COARSE: f64 = 1.0e-3;

/// Fine FD step relative to `KAPPA_BASE`. At `κ = 1e4` → `ε_κ = 1` N/m.
const FD_H_REL_FINE: f64 = 1.0e-4;

/// 5-digit relative-error bar — pairwise FD agreement at the two step
/// sizes per scope memo §1 V-7 ("agrees to 5-digit relative-error
/// bar"). Mirrors IV-8 + V-2 commit 4 verbatim.
const FD_SELF_CONSISTENCY_BAR: f64 = 1.0e-5;

// ── Helpers ──────────────────────────────────────────────────────────────

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

/// Per-κ run output: `F_R` + diagnostics for the robustly-in-contact
/// band condition + Newton sanity.
struct ReactionReport {
    /// FEM-integrated reaction force on the rigid plane.
    f_r: f64,
    /// Active-pair count at converged `x_final`.
    n_active: usize,
    /// Smallest `(d̂ - d)` across all active pairs — the band-margin
    /// floor for the robustly-in-contact assertion. Higher is better
    /// (further from the active-set boundary at `d = d̂`).
    min_band_margin: f64,
    /// Newton iteration count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
}

/// Build the V-3a override scene at `κ = kappa`, run one static
/// `replay_step`, return reconstructed `F_R` plus diagnostics. Mirrors
/// [`penalty_compressive_block.rs::run_at_refinement`] structure with
/// fixed `n_per_edge = N_PER_EDGE` and configurable κ.
fn reaction_at(kappa: f64) -> ReactionReport {
    // Cast safe: N_PER_EDGE is a small-integer literal.
    #[allow(clippy::cast_precision_loss)]
    let cell_size = EDGE_LEN / (N_PER_EDGE as f64);
    let (mesh, bc, initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, cell_size, DISPLACEMENT, &material_field());
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], kappa, D_HAT_OVERRIDE);

    let band_tol = 0.5 * cell_size;
    let top_face_vertices: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE_LEN).abs() < band_tol);

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;

    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);

    // Reconstruct F_R from x_final per `penalty_compressive_block.rs`'s
    // "Reaction-force extraction" section: gradient is `-κ·(d̂-d)·n`
    // with `n = -ẑ` for the V-3a plane, so the force on the rigid body
    // (Newton's 3rd-law partner) along `+ẑ` is `+κ·(d̂-d)`.
    let mut f_r = 0.0;
    let mut n_active = 0;
    let mut min_band_margin = f64::INFINITY;
    for &v in &top_face_vertices {
        let z_v = step.x_final[3 * v as usize + 2];
        let sd = EDGE_LEN - DISPLACEMENT - z_v;
        if sd < D_HAT_OVERRIDE {
            let band_margin = D_HAT_OVERRIDE - sd;
            f_r += kappa * band_margin;
            n_active += 1;
            if band_margin < min_band_margin {
                min_band_margin = band_margin;
            }
        }
    }

    ReactionReport {
        f_r,
        n_active,
        min_band_margin,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
// Mirrors `concentric_lame_shells.rs:678` + `penalty_compressive_block.rs:395`
// precedent — the diagnostic eprintln + per-config band-condition + sign
// + self-consistency assertion blocks legitimately exceed clippy's 100-
// line soft cap. Extracting helpers would add indirection without
// improving clarity.
#[allow(clippy::too_many_lines)]
fn v_7_contact_gradient_hook_is_fd_stable_in_robustly_in_contact_band() {
    let h_coarse = FD_H_REL_COARSE * KAPPA_BASE;
    let h_fine = FD_H_REL_FINE * KAPPA_BASE;

    // Five forward passes: base + four perturbed (± at each step size).
    // Mirrors IV-8's four-pass structure plus an explicit base-state
    // report for the band-condition / sign assertions.
    let base = reaction_at(KAPPA_BASE);
    let plus_coarse = reaction_at(KAPPA_BASE + h_coarse);
    let minus_coarse = reaction_at(KAPPA_BASE - h_coarse);
    let plus_fine = reaction_at(KAPPA_BASE + h_fine);
    let minus_fine = reaction_at(KAPPA_BASE - h_fine);

    let fd_coarse = (plus_coarse.f_r - minus_coarse.f_r) / (2.0 * h_coarse);
    let fd_fine = (plus_fine.f_r - minus_fine.f_r) / (2.0 * h_fine);
    let fd_self_rel_err = (fd_coarse - fd_fine).abs() / fd_coarse.abs().max(1.0e-12);

    eprintln!(
        "v_7 ∂F_R/∂κ FD-stability at V-3a override regime n={N_PER_EDGE}, κ={KAPPA_BASE:e} N/m: \
         base F_R = {base_fr:.6} N, n_active = {base_n}, min band margin = {base_m:e} m, \
         iters = {base_it}, res = {base_r:e}; \
         FD at h_rel={FD_H_REL_COARSE:e} (ε_κ = {h_coarse:e}) = {fd_coarse:e} N/(N/m); \
         FD at h_rel={FD_H_REL_FINE:e} (ε_κ = {h_fine:e}) = {fd_fine:e} N/(N/m); \
         self-consistency rel_err = {fd_self_rel_err:.3e} (bar {FD_SELF_CONSISTENCY_BAR:.0e})",
        base_fr = base.f_r,
        base_n = base.n_active,
        base_m = base.min_band_margin,
        base_it = base.iter_count,
        base_r = base.residual_norm,
    );

    // ── Newton convergence sanity at every pass ─────────────────────────
    //
    // Under V-3a override, Newton typically takes 3-5 iters per pass
    // (commit 8 logs). At < 40 we have 35+ iters of margin against
    // perturbation-induced regime drift. A pass that runs near the cap
    // signals the V-3a `(d̂, δ)` override is not robust at this
    // perturbed κ — would corrupt the FD numerator before band-margin
    // or self-consistency gates surface.
    for (label, report) in [
        ("base", &base),
        ("κ + h_coarse", &plus_coarse),
        ("κ - h_coarse", &minus_coarse),
        ("κ + h_fine", &plus_fine),
        ("κ - h_fine", &minus_fine),
    ] {
        assert!(
            report.iter_count < 40,
            "{label}: Newton ran {iters} iters, within 10 of the {cap}-iter cap — \
             investigate solver / penalty-regime regression at this perturbed κ before \
             bumping the cap",
            iters = report.iter_count,
            cap = MAX_NEWTON_ITER,
        );
    }

    // ── Robustly-in-contact band condition at every active pair ─────────
    //
    // Per scope memo §1 V-7: every active pair has `d < d̂ - 2 ε_FD-position`,
    // where ε_FD-position ≈ h · d̂ / κ via linearization of the implicit-
    // x-vs-κ map (see module docstring "Robustly-in-contact band
    // condition" section). At `h_coarse = 10` N/m, `D_HAT_OVERRIDE = 1e-5`
    // m, `KAPPA_BASE = 1e4` N/m: `ε_FD-position ≈ 1e-8` m, `band_margin_floor
    // = 2e-8` m. Empirical min band margin at n=4 is ≈ `1.9e-7` m → ~9×
    // headroom (the average band margin is ~`7.4e-7` m, but the min is
    // smaller because top-face vertex depths are non-uniform — the
    // bottom-pin's lateral confinement decays from corners inward, so
    // central top vertices reach equilibrium at shallower contact).
    // The h_coarse step bounds the h_fine ε_FD-position (smaller h →
    // smaller perturbation), so checking against h_coarse-derived floor
    // is the conservative bound for all five configs.
    let eps_fd_position = h_coarse * D_HAT_OVERRIDE / KAPPA_BASE;
    let band_margin_floor = 2.0 * eps_fd_position;
    for (label, report) in [
        ("base", &base),
        ("κ + h_coarse", &plus_coarse),
        ("κ - h_coarse", &minus_coarse),
        ("κ + h_fine", &plus_fine),
        ("κ - h_fine", &minus_fine),
    ] {
        assert!(
            report.n_active > 0,
            "{label}: zero active pairs — V-7 requires the V-3a override regime to engage \
             contact at all five FD configs; an empty active set at any perturbed κ \
             signals the regime drifted out of contact",
        );
        assert!(
            report.min_band_margin > band_margin_floor,
            "{label}: min band margin {m:e} m ≤ 2·ε_FD-position {bm:e} m. \
             FD κ-perturbation could flip active-set membership across the d = d̂ \
             discontinuity (book Part 4 §00 §00 'transition band is a forbidden zone'); \
             V-7's smooth-derivative assumption fails. Diagnose: (1) is the V-3a override \
             regime still robustly-in-contact at this commit's HEAD? Commit-11 empirical \
             min margin at n=4 was ≈ 1.9e-7 m (avg ≈ 7.4e-7 m). (2) has the FD step grown \
             beyond commit-time tolerance? (3) at perturbed κ, has any active pair drifted \
             toward d = d̂?",
            m = report.min_band_margin,
            bm = band_margin_floor,
        );
    }

    // ── Sign sanity: ∂F_R/∂κ > 0 ────────────────────────────────────────
    //
    // At robustly-in-contact, force-balance F_R = elastic force at
    // x_final. As κ grows, x_final shifts so top-face vertices push
    // back into the band at shallower depths; effective compressive
    // strain ε_eff = (EDGE_LEN - z_eq_avg) / EDGE_LEN grows; elastic
    // force grows monotonically. Net: F_R is monotonically increasing
    // in κ, saturating at F_elastic_max = E·A·δ/L as κ → ∞. Negative
    // FD here signals a sign-flip regression in the implicit-x-vs-κ
    // map.
    assert!(
        fd_coarse > 0.0,
        "FD-coarse ∂F_R/∂κ = {fd_coarse:e} ≤ 0 — stiffer penalty should increase the \
         reaction force (F_R saturating from 0 at κ=0 toward E·A·δ/L as κ → ∞). \
         Negative or zero is a sign-flip regression in the implicit-x-vs-κ map.",
    );
    assert!(
        fd_fine > 0.0,
        "FD-fine ∂F_R/∂κ = {fd_fine:e} ≤ 0 — see fd_coarse rationale above.",
    );

    // ── FD self-consistency at the 5-digit bar ──────────────────────────
    //
    // Per scope memo §1 V-7: "FD-stable" means the gradient is well-
    // defined at the perturbation scales the eventual reverse-mode
    // adjoint will consume. Two FD samples at h_rel ∈ {1e-3, 1e-4} sit
    // above the optimal h ≈ ε_f64^(1/3) · |κ| ≈ 6e-2 in the truncation-
    // dominated regime; pairwise agreement at the 5-digit bar bounds
    // the coarse-step truncation itself (truncation at h=10 is 100×
    // truncation at h=1 for smooth F_R(κ)). Demonstrates we're safely
    // above the Newton-convergence noise floor (`tol = 1e-10` × effective
    // stiffness ~ 1e-10 N noise at F_R ≈ 0.18 N) — the empirical
    // self-consistency rel_err at the V-3a regime sits in the 1e-7 range,
    // 12× headroom against the bar.
    assert!(
        fd_self_rel_err <= FD_SELF_CONSISTENCY_BAR,
        "V-7 FD self-consistency failed: \
         FD at h_rel = {FD_H_REL_COARSE:e} = {fd_coarse:e} N/(N/m), \
         FD at h_rel = {FD_H_REL_FINE:e} = {fd_fine:e} N/(N/m), \
         rel_err = {fd_self_rel_err:.3e} > {FD_SELF_CONSISTENCY_BAR:.0e}. \
         Diagnose in this order: (1) Newton convergence — verify all five passes \
         converged to `tol = 1e-10` (residual_norm in `replay_step`'s NewtonStep); \
         a regression here suggests the V-3a override regime no longer converges \
         as cleanly at perturbed κ. (2) Step-size band — h too small lets roundoff \
         dominate, h too large lets truncation; the chosen `h_rel ∈ {{1e-3, 1e-4}}` \
         brackets `ε_f64^(1/3) · |κ|`, a regression here suggests the optimal-h \
         derivation no longer holds at this commit's HEAD. (3) Robustly-in-contact \
         band — if any active pair's `(d̂ - d)` shrunk below `2 · ε_FD-position` \
         (caught by the band-condition assertion above, but worth re-checking if \
         the regime drifted). (4) commit-8 V-3a regime drift — the override \
         `(d̂, δ) = (1e-5, 5e-5)` was tuned for cold-start residual safety, not \
         sensitivity precision; if the regime moved, FD step sizes need \
         recalibration accordingly.",
    );
}
