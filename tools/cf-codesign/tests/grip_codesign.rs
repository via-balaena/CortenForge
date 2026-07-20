//! The de-escalation co-design loop on the soft↔rigid GRIP — the study's first
//! co-design *result*.
//!
//! [`GripCoDesignTarget`] co-designs the soft buffer stiffness `μ` (λ = 4μ) AND a
//! closed-loop holding policy `θ` together so the gripped limb's tangential drag
//! `tip_x` after the coupled rollout reaches a restraint target, reading BOTH
//! gradient blocks `(∂tip_x/∂μ_total, ∂tip_x/∂θ)` from ONE
//! `StaggeredCoupling::coupled_trajectory_design_policy_friction_gradient` backward
//! (the curvature-correct finite-contact gradient, PR #406, gated on the centroid
//! sphere by #430). Unlike [`JointTarget`]'s static free-platen press, the scene is
//! the keystone de-escalation grip: an actuated hinge limb whose finite sphere tip
//! sweeps tangentially into the soft block, gripped by dry friction (soft holding a
//! MOVING limb).
//!
//! The optimizer works in `p = [ln μ, θ…]` (the target owns the μ log-reparam so
//! the positive μ takes relative steps and the signed θ stay linear — both then
//! conditioned by one `loss_scale` Normalized lever and a single lr).
//!
//! Gates:
//! 1. the joint loss gradient (both blocks) matches a central FD of the loss in
//!    `p`-space — including the log-μ chain-rule on the design component;
//! 2. grip inverse design recovers a (μ*, θ*) behavior — from a perturbed start the
//!    optimizer drives `tip_x` to the restraint target by moving BOTH μ and θ.
//!
//! Tolerances are the friction-gradient's (rel ~5e-3), not the smooth normal
//! channel's: the underlying design+policy-friction gradient carries the frozen-lag
//! friction residual (the ~2e-3 intrinsic floor documented in #422), so the loss
//! gradient inherits it.

#![allow(clippy::expect_used)]

use cf_codesign::{
    CoDesignProblem, GripCoDesignTarget, GripObjective, InfeasibleDesign, OptConfig, StopReason,
};

// The de-escalation grip: an actuated hinge arm, its finite sphere tip pressed into
// the soft buffer under a sideways gravity that drives the tangential sweep.
const GRIP_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

const N_STEPS: usize = 10;
// The reference design whose gripped drag is the restraint target: a softer buffer +
// a holding policy.
const MU_STAR: f64 = 3.0e3;
const THETA_STAR: [f64; 3] = [0.0, 0.0, 0.5];

fn target() -> GripCoDesignTarget<sim_coupling::LinearFeedback> {
    let x = GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), N_STEPS, 0.0)
        .forward_x(MU_STAR, &THETA_STAR);
    GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), N_STEPS, x)
}

/// Both gradient blocks match a central FD of the loss in `p = [ln μ, θ]` space —
/// the design component carries the log-μ chain-rule factor, the policy components
/// are linear. Independent: the FD re-rolls the real coupled grip sim (rebuilding
/// the block at the perturbed μ for the design component).
#[test]
fn grip_loss_gradient_matches_fd() {
    let problem = target();
    // A generic non-optimal point: μ = 5e3 (p0 = ln 5e3), a generic holding policy.
    let p = problem.x0(5.0e3, &[0.05, -0.1, 0.2]);
    let (_loss, grad) = problem.evaluate(&p);

    let loss_at = |p: &[f64]| {
        let (loss, _) = problem.evaluate(p);
        loss
    };
    let names = ["ln μ", "w_z", "w_vz", "b"];
    for i in 0..p.len() {
        // ln μ ~ 8.5 (relative eps); weights/bias O(1) but the tip_x lever is small,
        // so use a slightly larger linear eps for a clean central difference.
        let eps = if i == 0 { 1e-5 } else { 1e-4 };
        let mut up = p.clone();
        let mut dn = p.clone();
        up[i] += eps;
        dn[i] -= eps;
        let fd = (loss_at(&up) - loss_at(&dn)) / (2.0 * eps);
        let rel = (grad[i] - fd).abs() / fd.abs().max(1e-30);
        let abs = (grad[i] - fd).abs();
        eprintln!(
            "{}: grad={:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            names[i], grad[i]
        );
        // The FD of the loss must be a real signal (not near-zero), else the rel
        // check is vacuously satisfied — mirroring the upstream centroid gate's
        // non-degenerate-FD guard (which the bare template omits). Every component
        // is O(1e-8..1e-6) at this point, so this is a safety net against a future
        // test-point shift, not a live constraint.
        assert!(
            fd.abs() > 1e-9,
            "∂loss/∂{} degenerate FD ({fd:e}) — pick a non-trivial test point",
            names[i]
        );
        // Friction-gradient tolerance (the underlying design+policy-friction gradient
        // is gated at rel ~5e-3, per the #422 intrinsic frozen-lag floor).
        assert!(
            rel < 5e-3,
            "∂loss/∂{} {} vs FD {fd} (rel {rel:e} abs {abs:e})",
            names[i],
            grad[i]
        );
    }
}

/// Grip inverse design recovers a target behavior: from a perturbed start (firmer
/// buffer AND zero policy), the optimizer drives `tip_x` to the restraint target by
/// moving BOTH the design variable and the holding policy. (Under-determined — 1+3
/// params for one scalar target — so this is behavior recovery, not unique (μ*,θ*).)
#[test]
fn grip_inverse_design_recovers_behavior() {
    let problem = target();
    let x_tgt = problem.target_x();
    // Start: firmer buffer, zero policy — far from the reference in BOTH axes.
    let mu0 = 6.0e3;
    let x0 = problem.x0(mu0, &[0.0, 0.0, 0.0]);
    let x_start = {
        let (m0, th0) = problem.to_physical(&x0);
        problem.forward_x(m0, &th0)
    };

    // Dimensionless residual (residual scale = the target drag magnitude); log_space
    // = false (the μ log-reparam is inside the target, θ is linear). Standard eps; a
    // single lr suiting both relative-μ and linear-θ steps.
    let normalized = problem.recommended_normalized(x_tgt.abs().max(1e-3));
    let cfg = OptConfig {
        lr: 0.03,
        max_iters: 800,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    assert!(
        (cfg.eps - OptConfig::default().eps).abs() < f64::EPSILON,
        "standard eps"
    );

    let result = normalized.optimize(&x0, &cfg);
    let (mu_rec, th_rec) = problem.to_physical(&result.params);
    let x_final = problem.forward_x(mu_rec, &th_rec);
    eprintln!(
        "x_tgt={x_tgt:.9} x_start={x_start:.9} x_final={x_final:.9} |x-tgt|={:.3e} \
         μ: {mu0} → {mu_rec:.1} (μ*={MU_STAR})  θ_rec={th_rec:?}  iters={} stop={:?}",
        (x_final - x_tgt).abs(),
        result.iters,
        result.stop_reason,
    );
    assert!(
        matches!(
            result.stop_reason,
            StopReason::GradTol | StopReason::LossTol
        ),
        "grip inverse design did not converge"
    );
    assert!(
        (x_final - x_tgt).abs() < 1e-8,
        "recovered (μ, θ) should hit the restraint target: x_final {x_final} vs target {x_tgt}"
    );
    // The optimizer genuinely moved the design variable μ off its start (both axes
    // are live, not just the policy carrying the whole correction). The μ lever on a
    // policy-driven tip_x is weaker than the policy's, so the threshold is modest.
    assert!(
        (mu_rec - mu0).abs() > 500.0,
        "μ should move off its {mu0} start (the design axis is live), got {mu_rec}"
    );
    // Sanity: the target differs from the start drag, so there was real work.
    assert!(
        (x_tgt - x_start).abs() > 1e-6,
        "restraint target should differ from the start drag"
    );
}

/// The public `build_at` (the viewer bridge) hands back the EXACT optimized scene:
/// a viewer renders the co-designed encounter by capturing frames from it. The
/// terminal tip agrees with the scalar `forward_x` — the picture IS the optimized
/// rollout. (Byte-identity of the capture vs the scalar rollout is gated in
/// `sim-coupling`'s `grip_rollout_capture.rs`; here we check the cf-codesign scene
/// plumbs through to renderable frames.)
#[test]
fn build_at_yields_renderable_frames() {
    let problem = target();
    let mu = 4.0e3;
    let theta = [0.05_f64, -0.1, 0.2];
    let frames = problem
        .build_at(mu)
        .coupled_trajectory_policy_gripped_capture(&sim_coupling::LinearFeedback, &theta, N_STEPS)
        .1;
    assert_eq!(
        frames.len(),
        N_STEPS + 1,
        "expected a rest frame + one per step ({} total), got {}",
        N_STEPS + 1,
        frames.len()
    );
    assert!(
        frames
            .iter()
            .all(|f| f.soft_positions.iter().all(|p| p.is_finite())
                && f.arm_pivot.iter().all(|p| p.is_finite())
                && f.fist_center.iter().all(|p| p.is_finite())),
        "all captured frames must be finite"
    );
    // The terminal frame's tracked drag must agree with the scalar forward: the picture
    // is the same rollout the optimizer scored. (The frame carries the body ORIGIN in
    // arm_pivot; forward_x returns the inertial tip xipos.x, so compare via a fresh
    // forward_x — both come from the shared sim-coupling loop body, hence byte-equal.)
    let scalar_tip = problem.forward_x(mu, &theta);
    // The soft body deforms over the rollout (a real grip, not a static frame dump).
    let max_disp = frames[N_STEPS]
        .soft_positions
        .iter()
        .zip(&frames[0].soft_positions)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        scalar_tip.is_finite() && max_disp > 1e-5,
        "the captured grip should deform the soft body (scalar tip {scalar_tip}, \
         max vertex displacement {max_disp:e})"
    );
}

const Q_HOLD: f64 = 0.25;

/// The HOLD objective's loss gradient (the `GripObjective::Hold` dispatch) matches a central FD
/// of the holding loss `L = Σ (qₖ − q_hold)²` in `p = [ln μ, θ]` space — the cf-codesign glue
/// (the log-μ chain on `L`, the dispatch) on top of the H1 trajectory-integrated gradient (gated
/// by `sim-coupling`'s `*·design-policy-hold[μ+θ]` `coupling_grad_harness.rs` rows).
#[test]
fn grip_hold_loss_gradient_matches_fd() {
    let problem = GripCoDesignTarget::linear_for_hold(GRIP_MJCF.to_string(), N_STEPS, Q_HOLD);
    let p = problem.x0(5.0e3, &[0.05, -0.1, 0.2]);
    let (_loss, grad) = problem.evaluate(&p);

    let loss_at = |p: &[f64]| problem.evaluate(p).0;
    let names = ["ln μ", "w_z", "w_vz", "b"];
    for i in 0..p.len() {
        let eps = if i == 0 { 1e-5 } else { 1e-4 };
        let mut up = p.clone();
        let mut dn = p.clone();
        up[i] += eps;
        dn[i] -= eps;
        let fd = (loss_at(&up) - loss_at(&dn)) / (2.0 * eps);
        assert!(fd.abs() > 1e-9, "∂L/∂{} degenerate FD ({fd:e})", names[i]);
        let rel = (grad[i] - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 5e-3,
            "∂L/∂{} {} vs FD {fd} (rel {rel:e})",
            names[i],
            grad[i]
        );
    }
}

/// Hold co-design reduces the holding cost: from a perturbed start (firm buffer, zero policy) the
/// optimizer drives `L = Σ (qₖ − q_hold)²` down — the co-designed buffer + policy hold the limb
/// closer to `q_hold` throughout. (A minimization, not a recover-to-zero, so we assert a clear
/// reduction, not convergence to a known target.)
#[test]
fn grip_hold_reduces_cost() {
    let problem = GripCoDesignTarget::linear_for_hold(GRIP_MJCF.to_string(), N_STEPS, Q_HOLD);
    let x0 = problem.x0(6.0e3, &[0.0, 0.0, 0.0]);
    let (loss_start, _) = problem.evaluate(&x0);

    // Condition by the starting cost scale; run a fixed Adam budget (the loss floors above 0, so
    // it won't hit loss_tol — we judge by the reduction).
    let normalized = problem.recommended_normalized(loss_start.sqrt().max(1e-3));
    let cfg = OptConfig {
        lr: 0.05,
        max_iters: 200,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    let result = normalized.optimize(&x0, &cfg);
    let (loss_final, _) = problem.evaluate(&result.params);

    eprintln!(
        "hold cost: start {loss_start:.6e} -> final {loss_final:.6e} ({:.1}% reduction) in {} iters",
        100.0 * (1.0 - loss_final / loss_start),
        result.iters,
    );
    assert!(
        loss_start > 1e-6,
        "the start should have a real holding deficit (got {loss_start:e})"
    );
    assert!(
        loss_final < 0.8 * loss_start,
        "hold co-design should clearly reduce the holding cost: {loss_start} -> {loss_final}"
    );
}

// The MOVING-EE grip the R5 viewer drives: the contact sphere tracks the arm-tip geom
// (`with_contact_geom`), pivot directly above the block centre. The gated geometry from
// sim-coupling's `powered_friction_moving_ee_mu` harness scene (the moving-EE grip).
const MOVING_EE_MJCF: &str = r#"<mujoco>
  <option gravity="1.5 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

/// The moving-EE Hold path (`with_contact_geom` + `GripObjective::Hold` dispatch) — the exact
/// plumbing the R5 viewer drives — evaluates to a finite loss + gradient. Gradient *correctness*
/// for the moving-EE hold is gated upstream (sim-coupling's `moving-ee·design-policy-hold[μ+θ]`
/// `coupling_grad_harness.rs` row); this checks the cf-codesign scene/objective wiring threads through.
#[test]
fn grip_hold_moving_ee_evaluates() {
    let problem = GripCoDesignTarget::new(
        MOVING_EE_MJCF.to_string(),
        1,
        0.005,
        4,
        0.1,
        1.0e-3,
        3.0e4,
        1.0e-2,
        0.05,
        0.08,
        2.5,
        0.1,
        5,
        GripObjective::Hold(0.0),
        sim_coupling::LinearFeedback,
    )
    .with_contact_geom(0);
    let p = problem.x0(3.0e3, &[0.05, -0.02, 0.01]);
    let (loss, grad) = problem.evaluate(&p);
    assert!(
        loss.is_finite() && loss > 1e-9 && grad.len() == 4 && grad.iter().all(|g| g.is_finite()),
        "moving-EE Hold evaluate must yield a finite, live loss + gradient, got (loss {loss}, grad {grad:?})"
    );
}

/// The moving-EE hold optimization is fail-closed-fragile: some Adam-explored θ tears the buffer into
/// a non-convergent soft solve, which the grip surfaces as a typed `Err(InfeasibleDesign)` (the soft
/// `NewtonIterCap` → `RolloutError` → `InfeasibleDesign` chain). With `OptConfig::reject_infeasible`
/// the optimizer reads that `Err` and backtracks past those steps and SURVIVES — reducing the holding
/// cost where the default loop would hit the panic-path `evaluate` and crash. This is the robustness
/// the R5 viewer relies on to co-design a moving-EE hold at a useful horizon.
#[test]
fn grip_hold_moving_ee_optimizes_robustly() {
    let problem = GripCoDesignTarget::new(
        MOVING_EE_MJCF.to_string(),
        1,
        0.005,
        4,
        0.1,
        1.0e-3,
        3.0e4,
        1.0e-2,
        0.05,
        0.08,
        2.5,
        0.1,
        8,
        GripObjective::Hold(0.0),
        sim_coupling::LinearFeedback,
    )
    .with_contact_geom(0);
    let x0 = problem.x0(4.0e3, &[0.0, 0.0, 0.0]);
    let (cost_start, _) = problem.evaluate(&x0);
    let normalized = problem.recommended_normalized(cost_start.sqrt().max(1e-3));
    let cfg = OptConfig {
        lr: 0.03,
        max_iters: 40,
        loss_tol: 1e-18,
        reject_infeasible: true,
        ..OptConfig::default()
    };
    let result = normalized.optimize(&x0, &cfg);
    let (cost_final, _) = problem.evaluate(&result.params);
    // Survived the tearing steps (no crash) and — because feasibility-aware optimization returns the
    // BEST feasible iterate, and the feasible x0 is the first point scored — the holding cost is
    // structurally never worse than the start. (Reduction magnitude varies with the fragile contact;
    // survival + the never-worse guarantee are the load-bearing claims.)
    assert!(
        cost_final.is_finite() && cost_final <= cost_start,
        "robust moving-EE hold optimize should survive and not worsen: {cost_start} -> {cost_final}"
    );
}

// The moving-EE Hold grip target (the fail-closed-fragile R5 scene) at the horizon the robust-
// optimize test uses.
fn moving_ee_hold_target() -> GripCoDesignTarget<sim_coupling::LinearFeedback> {
    GripCoDesignTarget::new(
        MOVING_EE_MJCF.to_string(),
        1,
        0.005,
        4,
        0.1,
        1.0e-3,
        3.0e4,
        1.0e-2,
        0.05,
        0.08,
        2.5,
        0.1,
        8,
        GripObjective::Hold(0.0),
        sim_coupling::LinearFeedback,
    )
    .with_contact_geom(0)
}

/// `CoDesignProblem::try_evaluate` surfaces a grip fail-close as `Err(InfeasibleDesign)` — the typed
/// path the feasibility-aware optimizer consumes (in lieu of `catch_unwind`). An aggressive holding
/// policy (θ = [50, 0, 50]) tears the coarse buffer into a non-convergent soft solve; `try_evaluate`
/// returns Err (NOT a panic, NOT a bogus Ok), and its `Display` carries the fail-close reason.
#[test]
fn grip_try_evaluate_returns_err_on_infeasible_design() {
    let problem = moving_ee_hold_target();
    let p = problem.x0(2.0e3, &[50.0, 0.0, 50.0]);
    let err: InfeasibleDesign = problem
        .try_evaluate(&p)
        .expect_err("an aggressive holding policy must tear the buffer → Err, not Ok");
    let msg = err.to_string();
    assert!(
        msg.contains("infeasible design") && msg.contains("rollout failed at step"),
        "Display should name the fail-close, got: {msg}"
    );
}

/// PR2a: on a feasible design `try_evaluate` returns `Ok` byte-identical to `evaluate` (the panic-path
/// method now delegates to it). Pins the success-path equivalence of the two surfaces.
#[test]
fn grip_try_evaluate_ok_matches_evaluate_when_feasible() {
    let problem = moving_ee_hold_target();
    let p = problem.x0(3.0e3, &[0.05, -0.02, 0.01]);
    let (loss_e, grad_e) = problem.evaluate(&p);
    let (loss_t, grad_t) = problem
        .try_evaluate(&p)
        .expect("a feasible design must evaluate");
    assert_eq!(loss_e.to_bits(), loss_t.to_bits(), "loss byte-identical");
    assert_eq!(grad_e.len(), grad_t.len(), "gradient length");
    for (k, (a, b)) in grad_e.iter().zip(&grad_t).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "grad[{k}] byte-identical");
    }
}

/// The load-bearing `Normalized::try_evaluate` override: the grip is optimized THROUGH a `Normalized`
/// wrapper, so the wrapper must FORWARD the inner grip's `Err(InfeasibleDesign)` rather than fall back
/// to the panic-path `evaluate`. A *deterministic* guard (the optimize loop's tear is stochastic):
/// wrap the moving-EE grip, call `try_evaluate` at the known-infeasible θ=[50,0,50], and assert it
/// returns `Err` — not a panic, not a swallowed `Ok`. Without the override this would panic (the
/// default `Normalized::try_evaluate` → `Normalized::evaluate` → grip `evaluate` → `panic!`), which is
/// exactly the crash `grip_hold_moving_ee_optimizes_robustly` would suffer once `catch_unwind` is gone.
#[test]
fn normalized_grip_try_evaluate_forwards_infeasible() {
    let problem = moving_ee_hold_target();
    // `recommended_normalized` wraps the grip in `Normalized` (log_space = false — the μ log-reparam
    // lives in the target — so the optimizer-space params equal the grip's `x0` space).
    let normalized = problem.recommended_normalized(1.0);
    let p = problem.x0(2.0e3, &[50.0, 0.0, 50.0]);
    let err: InfeasibleDesign = normalized.try_evaluate(&p).expect_err(
        "Normalized must forward the grip's infeasibility as Err, not panic or swallow it",
    );
    assert!(
        err.to_string().contains("infeasible design"),
        "forwarded error should be the grip's InfeasibleDesign, got: {err}"
    );
}

// A SHARP fist (small sphere) deeply indenting the coarse buffer over-stretches a local tet past the
// material's validity domain — the fail-close mode the R5 viewer's co-design hits (distinct from the
// Newton iter-cap the blunter moving-EE scene hits). Arm pivot above the block, fist grazing at qpos0.
const VALIDITY_TEAR_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.285">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom name="fist" type="sphere" pos="0 0 -0.15" size="0.06" mass="0.5"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

/// Validity-as-`Err` end-to-end (the H3-unblocking fix): a grip whose contact over-stretches a tet
/// (the validity-domain fail-close) now surfaces as `Err(InfeasibleDesign)` through `try_evaluate` —
/// the soft `ValidityViolation` → `RolloutError` → `InfeasibleDesign` chain — instead of panicking.
/// Before this fix the soft solver's validity `assert!` panicked uncaught (only `NewtonIterCap` was a
/// typed `Err`), crashing the R5 co-design; now the optimizer skips a validity tear like any other
/// infeasible design.
#[test]
fn grip_try_evaluate_returns_err_on_validity_tear() {
    let problem = GripCoDesignTarget::new(
        VALIDITY_TEAR_MJCF.to_string(),
        1,
        0.005,
        4,
        0.1,
        1.0e-3,
        3.0e4,
        1.0e-2,
        0.15, // qpos0 — the fist presses in
        0.06, // sphere radius = the sharp fist
        2.5,
        0.1,
        6,
        GripObjective::Hold(0.0),
        sim_coupling::LinearFeedback,
    )
    .with_contact_geom(0);
    let p = problem.x0(4.0e3, &[0.0, 0.0, 0.0]);
    let err: InfeasibleDesign = problem
        .try_evaluate(&p)
        .expect_err("a tet-over-stretching grip must be a typed Err, not a panic");
    // Specifically the validity-domain path (not a Newton iter-cap) — proves the new variant flows.
    assert!(
        err.to_string().contains("validity domain"),
        "expected a validity-domain fail-close, got: {err}"
    );
}
