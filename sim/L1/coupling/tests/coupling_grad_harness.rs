//! Differential-testing harness for the keystone soft↔rigid co-design gradients.
//!
//! The coupling layer's correctness has historically rested on dozens of
//! *separate* hand-authored gradient gates, each FD-checking one channel on one
//! bespoke scene. That is the "correctness depends on hand-picked adversarial scenes —
//! doesn't scale, is how bugs ship" pattern that the sim-core
//! `transition_matrix_harness` was built to retire at Layer 0. This file is its
//! Layer-1 counterpart: a single channel-agnostic harness that FD-checks a
//! *matrix* of `(scene × gradient channel)` against an independent re-rollout
//! oracle, so a new gradient leaf becomes one [`GradCase`] row rather than a new
//! bespoke file.
//!
//! ## The unifying abstraction
//!
//! Every coupling gradient — however heterogeneous its internals — reduces to a
//! flat parameter vector plus two closures:
//! - `analytic`: returns `(loss, grad)`, the tape gradient over the baseline
//!   params (one `tape.backward` across both engines);
//! - `value_at`: returns the scalar loss at an arbitrary full parameter vector,
//!   re-running the REAL nonlinear coupled rollout (a fresh Newton solve + rigid
//!   step per evaluation) — so the FD oracle is genuinely independent of the
//!   tape/VjpOp machinery, not an affine identity.
//!
//! The adapter for each channel injects each flat param into its correct site —
//! a *scene-construction* arg (e.g. the soft stiffness μ, FD'd by rebuilding) or
//! a *per-step call* arg (e.g. a control schedule / policy θ, FD'd by re-calling)
//! — and the analytic gradient is returned in the same flat ordering. Per-channel
//! `eps`/`tol`/`floor` let a relative-eps stiffness param and an absolute-eps
//! control schedule share one harness core ([`fd_check`]).
//!
//! ## PR1 scope
//!
//! The four co-design channels — `control`, `policy`, `joint` (the heterogeneous
//! μ+θ case), and `material` (the single-step S5 gradient, split into μ and λ) —
//! on the keystone platen scene(s). Broadening the *scene* axis (articulated /
//! sphere / free-body topologies) and retiring the now-redundant bespoke gates
//! follow in later PRs. The existing per-channel gates stay in place meanwhile;
//! this harness runs alongside them as the scalable guard.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

// ── Scene library (the seed; grows along the scene axis in later PRs) ──

const MU0: f64 = 3.0e4;
const LAMBDA0: f64 = 1.2e5; // = 4·μ (the constructor's Lamé relation)

/// Deeply-engaged platen for the *trajectory* channels: the soft block's top
/// face sits inside the platen's contact band so the rollout is firmly engaged —
/// the regime the keystone gradients are scoped to. A fresh build per call is
/// REQUIRED: the rollout mutates coupling state.
fn traj_coupling(mu: f64) -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("trajectory platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // body=1, clearance=5mm, n_per_edge=4, edge=0.1, mu, dt=1e-3, κ=3e4, d̂=1e-2, damping=60.
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 60.0,
    )
}

/// Platen for the *single-step* S5 material gradient — a higher start so the
/// `height`-parameterized step engages 25 top vertices (the material gate's
/// fixture). Lighter damping (12) is the single-step scope.
fn step_coupling() -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("step platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 12.0,
    )
}

// ── Harness core ──

/// One gradient channel reduced to the harness's uniform interface.
struct GradCase {
    /// Display label (`channel` or `channel[block]`).
    name: &'static str,
    /// Baseline parameter vector the analytic gradient is taken at.
    baseline: Vec<f64>,
    /// Per-component finite-difference step (absolute). Per-component so a
    /// relative-eps stiffness param and an absolute-eps schedule coexist.
    eps: Vec<f64>,
    /// Relative-error gate; a component passes if `rel < tol` OR `abs < floor`.
    tol: f64,
    /// Absolute-error floor for components whose FD bottoms near its float floor
    /// (e.g. a physically zero-effect terminal control).
    floor: f64,
    /// Per-component liveness expectation: `true` requires that component's FD be
    /// non-degenerate (`|fd| > floor`), so a channel that silently stopped
    /// coupling there can't pass vacuously. This is the per-component
    /// generalization of the bespoke gates' liveness checks (e.g. policy's
    /// `feedback_weights_carry_gradient`). `false` marks a component that is
    /// physically zero-effect (e.g. the terminal control under the off-by-one
    /// rigid carry), where degeneracy is expected — the harness does NOT assert
    /// it is *exactly* zero (that physical invariant stays in the targeted gate).
    expect_live: Vec<bool>,
    /// Analytic `(loss, grad)` over `baseline` — one `tape.backward`.
    analytic: Box<dyn Fn() -> (f64, Vec<f64>)>,
    /// Scalar loss at an arbitrary full parameter vector, via a fresh REAL
    /// coupled rollout (the independent FD oracle).
    value_at: Box<dyn Fn(&[f64]) -> f64>,
}

/// The channel-agnostic core: analytic tape gradient vs per-component central FD
/// through `value_at`. Asserts each component matches, and that every component
/// flagged `expect_live` has a non-degenerate FD (so a channel that silently
/// stopped coupling to the loss can't pass vacuously).
fn fd_check(case: &GradCase) {
    let (loss, grad) = (case.analytic)();
    assert!(loss.is_finite(), "{}: loss not finite", case.name);
    assert_eq!(
        grad.len(),
        case.baseline.len(),
        "{}: grad len {} != baseline len {}",
        case.name,
        grad.len(),
        case.baseline.len()
    );
    assert_eq!(
        case.eps.len(),
        case.baseline.len(),
        "{}: eps len {} != baseline len {}",
        case.name,
        case.eps.len(),
        case.baseline.len()
    );
    assert_eq!(
        case.expect_live.len(),
        case.baseline.len(),
        "{}: expect_live len {} != baseline len {}",
        case.name,
        case.expect_live.len(),
        case.baseline.len()
    );

    for i in 0..case.baseline.len() {
        let mut up = case.baseline.clone();
        let mut dn = case.baseline.clone();
        up[i] += case.eps[i];
        dn[i] -= case.eps[i];
        let fd = ((case.value_at)(&up) - (case.value_at)(&dn)) / (2.0 * case.eps[i]);
        let g = grad[i];
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!(
            "  {} [{i}]: tape={g:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            case.name
        );
        // Per-component liveness: a component expected to couple must have a
        // non-degenerate FD, else a silently-decoupled channel passes vacuously
        // (analytic 0 vs oracle 0). Generalizes the bespoke per-weight guards.
        assert!(
            !case.expect_live[i] || fd.abs() > case.floor,
            "{} [{i}]: expected a live coupling but |fd|={:e} ≤ floor — the channel is not \
             coupling to the loss here (degenerate gate)",
            case.name,
            fd.abs()
        );
        assert!(
            rel < case.tol || abs < case.floor,
            "{} [{i}]: tape {g} vs full-coupled FD {fd} (rel {rel:e} abs {abs:e})",
            case.name
        );
    }
}

// ── Channel adapters ──

/// CONTROL — per-step vertical force schedule on the platen (pure call-slice).
fn control_case() -> GradCase {
    let controls: Vec<f64> = (0..10)
        .map(|k| if k % 2 == 0 { -1.5 } else { 1.0 })
        .collect();
    let base = controls.clone();
    GradCase {
        name: "control",
        eps: vec![1e-2; base.len()],
        tol: 1e-6,
        floor: 1e-12,
        // Every control couples EXCEPT the last: sim-core integrates height with
        // the step's STARTING velocity, so the terminal control never moves z_N
        // (the off-by-one rigid carry). Its exact-zero invariant is the bespoke
        // `last_control_has_no_effect` gate's job.
        expect_live: (0..base.len()).map(|k| k + 1 != base.len()).collect(),
        baseline: base.clone(),
        analytic: Box::new(move || traj_coupling(MU0).coupled_trajectory_control_gradient(&base)),
        value_at: Box::new(|p| traj_coupling(MU0).coupled_trajectory_control_z(p)),
    }
}

/// POLICY — closed-loop `LinearFeedback` params θ = [w_z, w_vz, b] (call-slice
/// through the recurrence).
fn policy_case() -> GradCase {
    const THETA: [f64; 3] = [-20.0, -5.0, 2.0];
    const N: usize = 12;
    GradCase {
        name: "policy(θ)",
        baseline: THETA.to_vec(),
        eps: vec![1e-2, 1e-2, 1e-3],
        tol: 1e-5,
        floor: 1e-12,
        // All three live, including the feedback weights w_z/w_vz whose gradient
        // flows ONLY through the recurrence (subsumes `feedback_weights_carry_gradient`).
        expect_live: vec![true, true, true],
        analytic: Box::new(|| {
            traj_coupling(MU0).coupled_trajectory_policy_gradient(&LinearFeedback, &THETA, N)
        }),
        value_at: Box::new(|p| {
            traj_coupling(MU0).coupled_trajectory_policy_z(&LinearFeedback, p, N)
        }),
    }
}

/// JOINT — the heterogeneous co-design case: param 0 is the soft stiffness μ (a
/// scene-construction arg → FD by REBUILDING), params 1..4 are the policy θ (a
/// call-arg slice → FD by re-calling on the same builder). Proves the harness
/// spans both parameter kinds within one channel.
fn joint_case() -> GradCase {
    const THETA: [f64; 3] = [-20.0, -5.0, 2.0];
    const N: usize = 12;
    GradCase {
        name: "joint(μ+θ)",
        baseline: vec![MU0, THETA[0], THETA[1], THETA[2]],
        eps: vec![MU0 * 1e-6, 1e-2, 1e-2, 1e-3],
        tol: 1e-5,
        floor: 1e-12,
        // Soft stiffness μ AND all three policy params couple.
        expect_live: vec![true, true, true, true],
        analytic: Box::new(|| {
            let (z, dmu, dth) =
                traj_coupling(MU0).coupled_trajectory_joint_gradient(&LinearFeedback, &THETA, N);
            (z, vec![dmu, dth[0], dth[1], dth[2]])
        }),
        value_at: Box::new(|p| {
            let theta = [p[1], p[2], p[3]];
            traj_coupling(p[0]).coupled_trajectory_policy_z(&LinearFeedback, &theta, N)
        }),
    }
}

/// MATERIAL — the single-step S5 co-design gradient `∂vz'/∂(material param)` at a
/// deeply-engaged height. `idx` selects μ (0) or λ (1); each is a 1-param case
/// FD'd via the dedicated value oracle `coupled_step_material_vz`.
fn material_case(idx: usize, name: &'static str, p0: f64) -> GradCase {
    const H: f64 = 0.099;
    GradCase {
        name,
        baseline: vec![p0],
        eps: vec![p0 * 1e-6],
        tol: 1e-5,
        floor: 1e-12,
        expect_live: vec![true],
        analytic: Box::new(move || {
            let (vz, g) = step_coupling().coupled_step_material_gradient(H, idx);
            (vz, vec![g])
        }),
        value_at: Box::new(move |p| step_coupling().coupled_step_material_vz(H, idx, p[0])),
    }
}

/// The PR1 matrix: four co-design channels (material split μ/λ) on the keystone
/// platen scene(s). Adding a channel or scene is one row here.
fn cases() -> Vec<GradCase> {
    vec![
        control_case(),
        policy_case(),
        joint_case(),
        material_case(0, "material[μ]", MU0),
        material_case(1, "material[λ]", LAMBDA0),
    ]
}

#[test]
fn coupling_gradient_matrix_matches_full_coupled_fd() {
    for case in cases() {
        eprintln!("=== {} ===", case.name);
        fd_check(&case);
    }
}
