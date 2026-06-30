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
//! ## Coverage
//!
//! The harness covers a set of `(scene, channel)` cells — not a dense
//! cross-product, since not every channel applies to every rigid topology. The
//! cells live in [`cases`], grouped by scene:
//! - **free platen** (`traj_coupling` / `step_coupling`): `control`, `policy`,
//!   `joint` (the heterogeneous μ+θ case), `material[μ]`/`material[λ]`;
//! - **passive Y-hinge arm** (`hinge_coupling`): `articulated-material` (the
//!   moment-routed `Δt·M⁻¹·Jᵀ` carry, FD'd along the λ = 4μ tie);
//! - **actuated Y-hinge** (`actuated_hinge_coupling`): `actuator` (a real
//!   `<motor>`'s per-step control driving the arm through the contact);
//! - **curved `SphereSdf` collider** (`sphere_coupling`): `material` (the
//!   rotating-normal `f_mag·H` curvature carry a plane can't exercise);
//! - **off-COM free body, contact moment ON** (`freebody_coupling`):
//!   `orientation` (a final quaternion component — gates the wrench carry's
//!   POSITION rows `G_pos`, read via a `qpos` readout closure).
//!
//! ## Oracle independence — two kinds
//!
//! Every channel's FD oracle re-runs the REAL coupled physics, tape-free. Two
//! shapes, by what the parameter is:
//! - **scene-construction params** (material μ, …) → [`step_rollout`]: a generic
//!   `n`-step rollout via the public `step()` + a `readout` closure. Works on ANY
//!   scene with NO bespoke evaluator (the platen/hinge rows predate it and still
//!   use their dedicated `_z` methods, which are equally tapeless);
//! - **per-step call params** (control / policy / actuator) → the channel's
//!   dedicated `_z` method, since `step()` takes no per-step input.
//!
//! Adding a channel or scene is one [`GradCase`] row.
//!
//! ## Retiring the bespoke gates
//!
//! The end goal is for this matrix to REPLACE the per-scene gates, not run beside
//! them. A gate is retired once its row asserts everything the gate did. Each
//! gate's asserts are one of three kinds, all now expressible here: the per-component
//! FD-match (the row's core), the tape-vs-oracle forward match ([`FORWARD_TOL`],
//! folded into [`fd_check`] for every row), and per-component physical invariants
//! (liveness / zero-effect, via [`Comp`]). Retired so far: `control`
//! (`coupled_control_gradient.rs` — FD-match + forward match + the terminal-control
//! `Comp::Zero` invariant). The remaining bespoke gates retire row-by-row as their
//! coverage is confirmed folded; further scenes (free-body ang-vel, moving-EE) and
//! channels (friction / tangential / peak-pressure) join the same way.

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

/// Passive Y-hinge arm: a tilted link with a point mass on the tip pressing the
/// soft block (a contact force at the tip maps through the joint as
/// `Δt·M⁻¹·Jᵀ`, ≠ the platen's scalar `Δt/m`). Started off-vertical (qpos = 0.3)
/// so the tip arcs and gravity sustains engagement. Bare-M⁻¹ scope (no damping).
fn hinge_coupling(mu: f64) -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical so the moment arm is live
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// Actuated Y-hinge: the same arm driven by a swappable `<actuator>` on joint
/// `j` — the actuator torque drives the arm THROUGH the contact, so the per-step
/// control is the differentiated parameter.
fn actuated_hinge_coupling(actuator: &str) -> StaggeredCoupling {
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    {actuator}
  </actuator>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("actuated hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// Curved-collider platen: the soft block contacted by a finite `SphereSdf`
/// (`with_sphere_collider`) instead of a half-plane, so the contact normal
/// rotates over the curved face — the `f_mag·H` curvature term the plane gates
/// can't exercise. Started deeply engaged (platen z = 0.103) and damped so it
/// settles into the curved contact without tearing.
fn sphere_coupling(mu: f64) -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.103">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("sphere platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 40.0,
    )
    .with_sphere_collider(0.08)
}

/// Off-centre free body with the contact MOMENT enabled: the platen's COM sits at
/// x = 0.07 (off the +x contact patch), pre-penetrated and released from rest, so
/// the off-COM contact wrench tumbles it about body-y. This is the only scene with
/// `with_contact_moment(true)` — it exercises the free-joint wrench carry's
/// POSITION rows `G_pos = Δt·J_r·G_vel` (the SO(3) right-Jacobian integrating
/// `qvel'` into the quaternion), the rung the orientation target gates.
fn freebody_coupling(mu: f64) -> StaggeredCoupling {
    let com_z = 0.1 + 0.005 - 1.0e-4; // EDGE + HALF_Z − penetration
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0.07 0.05 {com_z}">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("free-body platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_contact_moment(true)
}

// ── Harness core ──

/// Per-component physical expectation on the gradient — the generalization of
/// the bespoke gates' liveness / zero-effect invariants.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Comp {
    /// The component MUST couple: its FD is non-degenerate (`|fd| > floor`), so a
    /// channel that silently stopped coupling there can't pass vacuously
    /// (generalizes e.g. policy's `feedback_weights_carry_gradient`).
    Live,
    /// The component MUST be physically zero-effect: both the tape gradient AND
    /// the oracle FD are `< floor` (generalizes e.g. control's
    /// `last_control_has_no_effect` — the terminal control under the off-by-one
    /// rigid carry never moves the final state).
    Zero,
}

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
    /// Per-component physical expectation ([`Comp`]) — `Live` (must couple) or
    /// `Zero` (must be exactly zero-effect). Folds the bespoke gates' liveness /
    /// zero invariants into the matrix.
    expect: Vec<Comp>,
    /// Analytic `(loss, grad)` over `baseline` — one `tape.backward`.
    analytic: Box<dyn Fn() -> (f64, Vec<f64>)>,
    /// Scalar loss at an arbitrary full parameter vector, via a fresh REAL
    /// coupled rollout (the independent FD oracle).
    value_at: Box<dyn Fn(&[f64]) -> f64>,
}

/// Forward-consistency tolerance: the analytic tape's loss (`analytic().0`) must
/// equal the independent oracle at the baseline to this bound. Every bespoke gate
/// asserts the same `< 1e-12` tape-vs-oracle forward match; folding it here gives
/// it to all rows (the tape's forward carries the real `step` values, so a glue
/// bug that diverges the tape forward from the real rollout is caught even when
/// the gradient FD doesn't expose it).
const FORWARD_TOL: f64 = 1e-12;

/// The channel-agnostic core. Three checks per case: (1) the tape's forward loss
/// matches the independent oracle at the baseline ([`FORWARD_TOL`]); (2) each
/// component's analytic gradient matches the central FD of `value_at`; (3) each
/// component meets its [`Comp`] expectation (live / zero).
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
        case.expect.len(),
        case.baseline.len(),
        "{}: expect len {} != baseline len {}",
        case.name,
        case.expect.len(),
        case.baseline.len()
    );

    // (1) Forward consistency: the tape's primal must reproduce the real rollout.
    let fwd = (case.value_at)(&case.baseline);
    let fwd_err = (loss - fwd).abs();
    assert!(
        fwd_err < FORWARD_TOL,
        "{}: tape forward loss {loss} != oracle rollout {fwd} (|Δ|={fwd_err:e} ≥ {FORWARD_TOL:e})",
        case.name
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
        // (2) Gradient match.
        assert!(
            rel < case.tol || abs < case.floor,
            "{} [{i}]: tape {g} vs full-coupled FD {fd} (rel {rel:e} abs {abs:e})",
            case.name
        );
        // (3) Per-component physical expectation.
        match case.expect[i] {
            Comp::Live => assert!(
                fd.abs() > case.floor,
                "{} [{i}]: expected a live coupling but |fd|={:e} ≤ floor — the channel is not \
                 coupling to the loss here (degenerate gate)",
                case.name,
                fd.abs()
            ),
            Comp::Zero => assert!(
                g.abs() < case.floor && fd.abs() < case.floor,
                "{} [{i}]: expected zero-effect but |tape|={:e} / |fd|={:e} ≥ floor",
                case.name,
                g.abs(),
                fd.abs()
            ),
        }
    }
}

/// Generic tapeless FD oracle for scene-construction-param channels: roll the
/// REAL coupled dynamics forward `n` steps via the public `step()` (which builds
/// NO tape), then read a scalar from the final state. `step()` is SDF-generic, so
/// this drives the actual posed-collider contact for ANY scene — a genuinely
/// independent oracle (it shares no code with the analytic tape/VJP path) without
/// needing a bespoke per-channel `_z` evaluator. Channels whose parameter is a
/// per-STEP call arg (control / policy / actuator) can't use this — `step()` takes
/// no per-step input — and route through their dedicated `_z` methods instead.
fn step_rollout(
    mut c: StaggeredCoupling,
    n: usize,
    readout: impl Fn(&StaggeredCoupling) -> f64,
) -> f64 {
    for _ in 0..n {
        c.step();
    }
    readout(&c)
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
        // (the off-by-one rigid carry). Marked `Comp::Zero` — which asserts its
        // exact-zero invariant here (absorbed from the retired bespoke
        // `last_control_has_no_effect` gate).
        expect: (0..base.len())
            .map(|k| {
                if k + 1 == base.len() {
                    Comp::Zero
                } else {
                    Comp::Live
                }
            })
            .collect(),
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
        expect: vec![Comp::Live, Comp::Live, Comp::Live],
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
        expect: vec![Comp::Live, Comp::Live, Comp::Live, Comp::Live],
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
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (vz, g) = step_coupling().coupled_step_material_gradient(H, idx);
            (vz, vec![g])
        }),
        value_at: Box::new(move |p| step_coupling().coupled_step_material_vz(H, idx, p[0])),
    }
}

/// ARTICULATED-MATERIAL — the material gradient on the passive Y-hinge scene: a
/// contact force at the arm tip routed through the joint (`Δt·M⁻¹·Jᵀ`). The lone
/// param is the soft stiffness μ with λ slaved (the constructor's λ = 4μ tie), so
/// the analytic total is `∂tip_z/∂μ + 4·∂tip_z/∂λ` and the FD rebuilds along the
/// same line via `coupled_trajectory_articulated_z`.
fn articulated_material_case() -> GradCase {
    const N: usize = 6; // spans the violent make, a liftoff, and a re-touch
    GradCase {
        name: "hinge·material[μ]",
        baseline: vec![MU0],
        eps: vec![MU0 * 1e-4],
        tol: 1e-7,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (tip_z, g_mu) =
                hinge_coupling(MU0).coupled_trajectory_material_gradient_articulated(N, 0);
            let (_t, g_la) =
                hinge_coupling(MU0).coupled_trajectory_material_gradient_articulated(N, 1);
            (tip_z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| hinge_coupling(p[0]).coupled_trajectory_articulated_z(N)),
    }
}

/// ACTUATOR — a real `<motor>`'s per-step control schedule on the actuated hinge
/// (`force = gear·ctrl`). All controls are live (the torque drives the arm
/// through the contact), FD'd via `coupled_trajectory_actuated_z`.
fn actuator_motor_case() -> GradCase {
    const MOTOR: &str = r#"<motor name="a" joint="j" gear="4"/>"#;
    let controls = vec![0.3, -0.2, 0.25, -0.15, 0.1, 0.2];
    let base = controls.clone();
    GradCase {
        name: "hinge·actuator(motor)",
        eps: vec![1e-3; base.len()],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live; base.len()],
        baseline: base.clone(),
        analytic: Box::new(move || {
            actuated_hinge_coupling(MOTOR).coupled_trajectory_actuator_gradient(&base)
        }),
        value_at: Box::new(|p| actuated_hinge_coupling(MOTOR).coupled_trajectory_actuated_z(p)),
    }
}

/// SPHERE-MATERIAL — the material gradient on the curved `SphereSdf` collider: the
/// rotating contact normal exercises the `f_mag·H` curvature carry (zero for a
/// plane). Tied-μ param (λ = 4μ), so the analytic total is `∂z/∂μ + 4·∂z/∂λ`. ★
/// FD oracle = the GENERIC tapeless `step_rollout` (no bespoke `_z` method),
/// reading the platen's world height — the first row to use the readout oracle.
fn sphere_material_case() -> GradCase {
    const N: usize = 8;
    GradCase {
        name: "sphere·material[μ]",
        baseline: vec![MU0],
        eps: vec![MU0 * 5e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (z, g_mu) = sphere_coupling(MU0).coupled_trajectory_material_gradient(N, 0);
            let g_la = sphere_coupling(MU0)
                .coupled_trajectory_material_gradient(N, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| step_rollout(sphere_coupling(p[0]), N, |c| c.data().xpos[1].z)),
    }
}

/// FREEBODY-ORIENTATION — `∂(final quaternion component qy)/∂(material)` on the
/// tumbling off-COM free body. Unlike the height/tip targets, the loss is an
/// ORIENTATION component, so this gates the wrench carry's POSITION rows
/// (`G_pos`, the SO(3) right-Jacobian). Tied-μ (`∂qy/∂μ + 4·∂qy/∂λ`); FD oracle =
/// `step_rollout` reading `qpos[4 + AXIS]` (the free joint's quaternion vector
/// component), the contact-moment analogue of the sphere row's height readout.
fn freebody_orientation_case() -> GradCase {
    const N: usize = 16;
    const AXIS: usize = 1; // qy — the off-centre +x strike tumbles about body-y
    GradCase {
        name: "freebody·orientation[μ]",
        baseline: vec![MU0],
        eps: vec![MU0 * 5e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (q, g_mu) =
                freebody_coupling(MU0).coupled_trajectory_orientation_gradient(N, 0, AXIS);
            let g_la = freebody_coupling(MU0)
                .coupled_trajectory_orientation_gradient(N, 1, AXIS)
                .1;
            (q, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| {
            step_rollout(freebody_coupling(p[0]), N, |c| c.data().qpos[4 + AXIS])
        }),
    }
}

/// The coverage matrix: `(scene, channel)` cells, grouped by scene. Adding a
/// channel or scene is one row here.
fn cases() -> Vec<GradCase> {
    vec![
        // free platen
        control_case(),
        policy_case(),
        joint_case(),
        material_case(0, "material[μ]", MU0),
        material_case(1, "material[λ]", LAMBDA0),
        // passive Y-hinge arm
        articulated_material_case(),
        // actuated Y-hinge
        actuator_motor_case(),
        // curved SphereSdf collider (generic tapeless step_rollout oracle)
        sphere_material_case(),
        // off-COM free body, contact moment ON (gates G_pos via orientation)
        freebody_orientation_case(),
    ]
}

#[test]
fn coupling_gradient_matrix_matches_full_coupled_fd() {
    for case in cases() {
        eprintln!("=== {} ===", case.name);
        fd_check(&case);
    }
}
