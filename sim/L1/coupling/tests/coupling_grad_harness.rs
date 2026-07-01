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
//!   `joint` (the heterogeneous μ+θ case), `material[μ]`/`material[λ]` (single-step),
//!   `platen·material[μ]` (the 20-step material trajectory), `load·plane` (the
//!   single-step S4 cross-engine load crossing, two operating points);
//! - **passive Y-hinge arm** (`hinge_coupling`): `articulated-material` (the
//!   moment-routed `Δt·M⁻¹·Jᵀ` carry, FD'd along the λ = 4μ tie);
//! - **seeded serial arms** (`seeded_arm_coupling`): `material` on damped arms — a single
//!   hinge (`damping=0.5`, analytic damped `J_state`) and a 2-link chain (per-joint damping,
//!   off-diagonal FD `J_state`), the `M_impl = M + Δt·D` velocity coupling a damping-free
//!   scene can't exercise — AND on undamped chains routed through the analytic chain
//!   `J_state`: a 2-link (nv = 2, off-diagonal `M` + Coriolis) and a 3-link multi-hop chain
//!   (nv = 3, the bias-acceleration transport a 2-link never reaches);
//! - **rotating-normal arms** (`rotating_coupling`): `material` on a tilted ball joint
//!   and a free base with `with_rotating_normal(true)`, so the carry flows through the
//!   contact normal's `q`-dependence (`δn̂ = ω×n̂` + the wrench redirect + the pose-twist
//!   seam) — terms a fixed-normal scene zeroes;
//! - **actuated Y-hinge** (`actuated_hinge_coupling`): `actuator` (real `<motor>` and
//!   state-feedback `<position>`/`<velocity>`/PD servos, per-step control driving the
//!   arm through the contact);
//! - **curved `SphereSdf` collider** (`sphere_coupling` / `load_sphere_coupling` /
//!   `sphere_articulated_coupling`): `material` (the `f_mag·H` curvature carry a plane
//!   can't exercise) on the free platen and the articulated hinge (the curved wrench
//!   routed through the joint), plus `load·sphere` (the load crossing on the curved normal);
//! - **moving end-effector** (`moving_ee_coupling`): `material` on a finite sphere
//!   tracking the arm geom (`with_contact_geom`), so the contact centre moves laterally
//!   as the arm swings — the 3-vector `∂centre/∂q` channel a height-only carry drops;
//! - **off-COM free body, contact moment ON** (`freebody_coupling`):
//!   `orientation` (a final quaternion component — gates the wrench carry's
//!   POSITION rows `G_pos`, read via a `qpos` readout closure);
//! - **IPC (C²-barrier) contact model** (`ipc_coupling`, a `StaggeredCoupling<IpcRigidContact>`
//!   type-parameter variant): single-step `material` (the isolated per-step factor) and the
//!   multi-step `material` trajectory through the contact make/break event across the barrier-
//!   stiffness sweep (κ from 3e4 to 3e2).
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
//! them. A gate is retired once its row asserts everything the gate did. Every
//! bespoke assert is now expressible here, all folded into [`fd_check`] so each
//! serves every row:
//! - the per-component **FD-match** (the row's core);
//! - the tape-vs-oracle **forward match** ([`FORWARD_TOL`]);
//! - **engagement** — either a state-level band on the baseline loss
//!   ([`GradCase::value_bounds`], folding e.g. `tip_z ∈ (0.10, 0.115)`) or a
//!   gradient-level [`Comp::Live`] floor ([`LIVE_FLOOR`], folding `grad > 1e-9`),
//!   so a fixture that drifts to non-engagement can't pass vacuously;
//! - per-component **zero-effect** invariants ([`Comp::Zero`]).
//!
//! Retired/folded so far. FULLY FOLDED (bespoke file deleted, nothing the matrix
//! can't express): `control`, `material-step`, `policy`, the single-step `load` channel
//! (plane + sphere), the `actuator` channel (motor + position / velocity / PD servos),
//! the `sphere-articulated` material trajectory, and the `IPC` contact-model trajectory
//! (single-step + the multi-step make/break rollout). SLIMMED (the file keeps the one
//! invariant the FD matrix structurally can't hold): `joint` (a tape-vs-tape
//! fusion-soundness check), `freebody-orientation` and the platen `material-trajectory`
//! (all-lengths machine-exact sweeps), `damped` (a damped-vs-undamped materiality
//! cross-check), `rotating-normal` (a rotating-vs-flat materiality check + a single-step
//! height-probe Jacobian), `moving-EE` (a `#[should_panic]` free-body contract guard + a
//! plane-collider byte-identity no-op). The remaining bespoke gates retire row-by-row as
//! their coverage is confirmed folded; further scenes (free-body ang-vel) and
//! channels (friction / tangential / peak-pressure) join the same way.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, PlaneContact, StaggeredCoupling};
use sim_mjcf::load_model;
use sim_soft::{
    HandBuiltTetMesh, IpcRigidContact, MaterialField, VertexId, pick_vertices_by_predicate,
};

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

/// The single-step LOAD channel's sphere variant: the same z=0.125 platen fixture as
/// [`step_coupling`] (the single-step gates pass an explicit `height`, so the platen pose is
/// irrelevant), with the half-plane swapped for a finite `SphereSdf`. The curved normal makes
/// the load crossing carry the `f_mag·H` term a plane can't (mirrors the retired
/// `sphere_trajectory_gradient.rs` load gate).
fn load_sphere_coupling() -> StaggeredCoupling {
    step_coupling().with_sphere_collider(0.08)
}

/// The block's top-face vertex ids (the loaded set for the single-step load gate) — recovered
/// from a mesh built with the SAME block params the coupling fixtures use (n_per_edge = 4,
/// edge = 0.1, μ/λ = MU0/LAMBDA0).
fn load_top_face() -> Vec<VertexId> {
    let mesh = HandBuiltTetMesh::uniform_block(4, 0.1, &MaterialField::uniform(MU0, LAMBDA0));
    pick_vertices_by_predicate(&mesh, |p| (p.z - 0.1).abs() < 1e-9)
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

/// The passive Y-hinge arm with a finite `SphereSdf` end-effector instead of the
/// half-plane (the articulated counterpart of [`sphere_coupling`]): the SAME
/// [`hinge_coupling`] scene + `with_sphere_collider`, posed over the block centroid. The
/// contact normal turns as a soft vertex slides over the curved face AND as the primitive
/// translates with tip height, so the wrench Jacobian carries the geometric-stiffness term
/// `f_mag·H` (zero for the plane) routed through the joint as `Δt·M⁻¹·Jᵀ`.
fn sphere_articulated_coupling(mu: f64) -> StaggeredCoupling {
    hinge_coupling(mu).with_sphere_collider(0.08)
}

/// A seeded serial articulated arm: an MJCF hinge / chain (with or without MuJoCo
/// `damping=` on its joints) seeded off-vertical and pressing the soft block. `body`/`seed`
/// select the contacting body and the off-vertical pose; the joint torque routing is the
/// articulated `Δt·M⁻¹·Jᵀ` carry. When the MJCF carries `damping=`, the default Euler
/// integrator `eulerdamp` solves `(M + Δt·D)·qacc = F`, so the contact wrench reaches
/// `qvel'` through `M_impl = M + Δt·D`, not the bare `M` — a channel a damping-free scene
/// can't exercise; undamped chains route through the bare `M`. The builder body itself is
/// damping-agnostic (the damping lives entirely in the MJCF), so it serves the damped hinge /
/// chain AND the undamped 2-link / 3-link chains. Bare-M⁻¹ scope otherwise matches
/// [`hinge_coupling`].
fn seeded_arm_coupling(
    mjcf: &str,
    mu: f64,
    body: usize,
    seed: &[(usize, f64)],
) -> StaggeredCoupling {
    let model = load_model(mjcf).expect("seeded arm MJCF loads");
    let mut data = model.make_data();
    for &(i, q) in seed {
        data.qpos[i] = q;
    }
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, body, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// Rotating-normal articulated scene: a body tilted off-vertical by `half` radians
/// about Y (a quaternion seed `qpos[w_idx]=cos(half)`, `qpos[v_idx]=sin(half)`)
/// pressing the soft block with `with_rotating_normal(true)`, so the contact normal
/// tracks the body orientation (`n̂ = R(q)·(0,0,−1)`). The coupled gradient then flows
/// through the normal's `q`-dependence (the pose-adjoint `δn̂ = ω×n̂`, the wrench's
/// `∂w/∂T` redirect, the `PoseTwistSeamVjp` seam). A GENTLE tilt (`half ≈ 0.02–0.05`)
/// keeps both the flat and rotating models stable — the ball diverges at θ = 0.3.
/// `body=1` for both seeds; the FD `loaded_state_jacobian` carries the ball/free joint
/// (neither has a single-hinge analytic `J_state`).
fn rotating_coupling(
    mjcf: &str,
    mu: f64,
    w_idx: usize,
    v_idx: usize,
    half: f64,
) -> StaggeredCoupling {
    let model = load_model(mjcf).expect("rotating-normal MJCF loads");
    let mut data = model.make_data();
    data.qpos[w_idx] = half.cos();
    data.qpos[v_idx] = half.sin();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_rotating_normal(true)
}

/// Moving-end-effector scene: a Y-hinge arm whose long link carries a finite sphere
/// end-effector (`with_sphere_collider` + `with_contact_geom(0)`) posed DIRECTLY over the
/// block centroid, so the sphere centre TRACKS the geom (`centre = geom_xpos(q)`) and
/// translates laterally in x as the tilted arm swings — not just vertically. The coupled
/// gradient must thread the 3-vector centre channel (`∂centre/∂q = J_geom`, the
/// `PoseCentreVjp` seam, `WrenchPose::Centre`); a height-only carry drops the lateral
/// feedback and is wrong here. Tilted (qpos = 0.15) for the off-COM moment + lateral travel,
/// large gentle sphere (r = 0.08, low curvature ⇒ a definite forward tangent). No damping.
fn moving_ee_coupling(mu: f64) -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("moving-EE hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.15; // tilt → off-COM moment + lateral centre travel
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(0.08)
    .with_contact_geom(0)
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

/// IPC-contact platen: the same z=0.125 free platen as [`step_coupling`] but running the
/// coupling on `IpcRigidContact` (the C²-barrier contact model) instead of the default
/// penalty, with the barrier stiffness `kappa` as a scene arg. The coupling is generic over
/// the contact model (a type parameter), so the SAME `(loss, grad)` methods and the SAME
/// tapeless `step()` oracle drive it — the `GradCase` closures type-erase the concrete
/// `StaggeredCoupling<IpcRigidContact>`. Exercises that the generic refactor + IPC contact +
/// the per-pair-curvature gradient factors compose, single- and multi-step.
fn ipc_coupling(mu: f64, kappa: f64) -> StaggeredCoupling<IpcRigidContact> {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("IPC platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::<IpcRigidContact>::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, kappa, 1.0e-2, 8.0,
    )
}

// ── Harness core ──

/// Per-component physical expectation on the gradient — the generalization of
/// the bespoke gates' liveness / zero-effect invariants.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Comp {
    /// The component MUST couple MEANINGFULLY: its FD is well above the float
    /// floor (`|fd| > LIVE_FLOOR`, not merely the FD-noise `floor`), so a channel
    /// that silently stopped coupling — or whose scene quietly de-engaged toward
    /// noise — can't pass vacuously (folds the bespoke gates' `grad > 1e-9`
    /// liveness sanities, e.g. sphere/free-body; cf. the loss-band [`value_bounds`]
    /// guard for scenes that pin engagement on the state instead).
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
    /// Optional engagement band on the baseline loss `(min, max)`. `Some` folds a
    /// bespoke gate's "the scene stays in its contact regime" assert (e.g.
    /// articulated/actuator's `tip_z ∈ (0.10, 0.115)`): the baseline rollout must
    /// land in-band, catching a fixture that drifts to non-engagement (loss too
    /// high) OR over-penetration/tearing (loss too low) — the direct state-level
    /// guard against fixture-rot in a consolidated harness. `None` for scenes that
    /// pin engagement via the gradient floor ([`Comp::Live`]) instead.
    value_bounds: Option<(f64, f64)>,
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

/// Liveness floor for [`Comp::Live`] — a gradient is "meaningfully coupling" only
/// if `|fd|` clears this, well above the FD-noise `floor`. Matches the bespoke
/// gates' `grad > 1e-9` sanities; every live component in the matrix clears it
/// with ≥10× margin (the smallest, the damped-hinge tied-μ, is ~1.2e-8 — the
/// damped chains' tied gradients are intrinsically near this floor, so their rows
/// run a longer horizon to lift clear of it). A scene that quietly de-engaged
/// toward noise would drop below it and fail.
const LIVE_FLOOR: f64 = 1e-9;

/// The channel-agnostic core. Four checks per case: (1) the tape's forward loss
/// matches the independent oracle at the baseline ([`FORWARD_TOL`]) and sits in
/// the optional engagement band; (2) each component's analytic gradient matches
/// the central FD of `value_at`; (3) each component meets its [`Comp`] expectation
/// (live / zero).
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
    // (1b) Engagement band: the baseline rollout must stay in its contact regime,
    // so a fixture that drifts to non-engagement / tearing can't pass vacuously.
    if let Some((lo, hi)) = case.value_bounds {
        assert!(
            loss >= lo && loss <= hi,
            "{}: baseline loss {loss} outside engagement band ({lo}, {hi}) — fixture not in \
             its intended contact regime",
            case.name
        );
    }

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
                fd.abs() > LIVE_FLOOR,
                "{} [{i}]: expected a live coupling but |fd|={:e} ≤ LIVE_FLOOR — the channel is \
                 not meaningfully coupling here (degenerate gate / de-engaged scene)",
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
fn step_rollout<C: PlaneContact>(
    mut c: StaggeredCoupling<C>,
    n: usize,
    readout: impl Fn(&StaggeredCoupling<C>) -> f64,
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
        value_bounds: None,
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
        value_bounds: None,
        baseline: THETA.to_vec(),
        eps: vec![1e-2, 1e-2, 1e-3],
        tol: 1e-5,
        floor: 1e-12,
        // All three live, including the feedback weights w_z/w_vz whose gradient
        // flows ONLY through the recurrence (subsumes the retired policy gate's
        // feedback-weights-live check).
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
        value_bounds: None,
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
        value_bounds: None,
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

/// LOAD — the single-step S4 cross-engine load gradient `∂vz'/∂theta`: a scalar load handle
/// `theta` on the block's top face drives the contact force hence the rigid step. `build`
/// selects the collider (plane via [`step_coupling`] or curved via [`load_sphere_coupling`]);
/// `(h, theta0)` is the operating point (the single-step gate passes `h` explicitly and freezes
/// the plane during the soft solve). `theta` is a per-call arg, so the FD oracle is the
/// dedicated `coupled_step_load_vz` (not `step_rollout`). Folds the retired
/// `coupled_load_gradient.rs` (plane) and the load test of `sphere_trajectory_gradient.rs`
/// (sphere); each gate ran two operating points, kept here as two rows per collider.
fn load_case(
    name: &'static str,
    build: fn() -> StaggeredCoupling,
    h: f64,
    theta0: f64,
) -> GradCase {
    let loaded_a = load_top_face();
    let loaded_v = loaded_a.clone();
    GradCase {
        name,
        value_bounds: None,
        baseline: vec![theta0],
        eps: vec![1e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (vz, g) = build().coupled_step_load_gradient(h, &loaded_a, theta0);
            (vz, vec![g])
        }),
        value_at: Box::new(move |p| build().coupled_step_load_vz(h, &loaded_v, p[0])),
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
        value_bounds: Some((0.10, 0.115)),
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

/// The damped single-hinge keystone scene (`damping=0.5`): the analytic damped
/// `J_state` (the `M → M_impl` correction) makes its per-step Jacobian machine-exact.
const DAMPED_HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

/// A 2-link chain with per-joint damping — off-diagonal `M`, so the `M + Δt·D`
/// coupling is genuinely live ACROSS joints (the FD `J_state` path under damping).
const DAMPED_2LINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0" damping="0.3"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

// An UNDAMPED 2-link hinge chain (nv = 2): the distal tip presses the block, so the
// contact wrench couples ACROSS both joints (off-diagonal M, Coriolis velocity coupling)
// through the analytic chain `J_state` — the bare-M⁻¹ (no-damping) counterpart of
// DAMPED_2LINK_MJCF. Started slightly off-centre so the joint coupling is live.
const TWOLINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

// An UNDAMPED 3-link hinge chain (nv = 3): the MULTI-HOP case — the tip joint's ancestor
// is two links up, exercising the sim-core Coriolis bias-acceleration transport (bug #3)
// that a 2-link never reaches. Planar (parallel axes) so the rollout stays
// well-conditioned; the analytic chain carry's multi-hop correctness is under test.
const THREELINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.02" size="0.004" mass="0.3"/>
      <body name="mid" pos="0 0 -0.04">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.02" size="0.004" mass="0.35"/>
        <body name="lower" pos="0 0 -0.04">
          <joint type="hinge" axis="0 1 0"/>
          <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.4"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

/// ARM-MATERIAL — the articulated material-trajectory gradient on a seeded serial arm
/// ([`seeded_arm_coupling`]): a single hinge / chain, damped or undamped, its tip pressing
/// the block, the contact wrench routed through the joint(s) as `Δt·M⁻¹·Jᵀ`. The general
/// counterpart of [`articulated_material_case`] across the seeded-arm scene family. Tied-μ
/// (λ = 4μ → `∂tip_z/∂μ + 4·∂tip_z/∂λ`); FD oracle = `coupled_trajectory_articulated_z` (the
/// same tapeless articulated rollout, automatically damping-correct because the engine's Euler
/// `step` runs `eulerdamp`). `(mjcf, body, seed)` selects the scene; per-scene `(n, band, eps_rel,
/// tol)` keep each fixture in its well-conditioned engaged horizon (the damped chains' tied-μ
/// gradient is intrinsically ~1e-8, so a longer horizon + the state `band` — not just the
/// gradient floor — guard against a de-engaged fixture; the undamped chains are machine-exact
/// via the analytic chain carry but whippy, so the horizon is capped in the stable regime).
/// Folds the FD cells of `damped_joint_gradient.rs` (damped hinge + chain) and the 2-link /
/// 3-link FD cells of `articulated_trajectory_gradient.rs` (undamped chains); those files keep
/// only the invariants a single-scene FD row can't express (the damped-vs-undamped materiality
/// cross-check; the single-hinge / free-platen machine-exact-at-all-lengths sweeps).
#[allow(clippy::too_many_arguments)]
fn arm_material_case(
    name: &'static str,
    mjcf: &'static str,
    body: usize,
    seed: &'static [(usize, f64)],
    n: usize,
    band: (f64, f64),
    eps_rel: f64,
    tol: f64,
) -> GradCase {
    GradCase {
        name,
        value_bounds: Some(band),
        baseline: vec![MU0],
        eps: vec![MU0 * eps_rel],
        tol,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (tip_z, g_mu) = seeded_arm_coupling(mjcf, MU0, body, seed)
                .coupled_trajectory_material_gradient_articulated(n, 0);
            let g_la = seeded_arm_coupling(mjcf, MU0, body, seed)
                .coupled_trajectory_material_gradient_articulated(n, 1)
                .1;
            (tip_z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(move |p| {
            seeded_arm_coupling(mjcf, p[0], body, seed).coupled_trajectory_articulated_z(n)
        }),
    }
}

/// The rotating-normal ball joint at a gentle Y-tilt (θ = 0.1): the off-COM tip presses
/// the block and the rotating normal redirects the reaction.
const ROT_BALL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="ball"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

/// The rotating-normal free joint (floating base) with an off-COM mass at a gentle
/// Y-tilt (θ = 0.04): the full SE(3) successor, started just above the block on a broad
/// base plate so the unconstrained base stays stable under the tangential redirect.
const ROT_FREE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="body" pos="0 0 0.123">
      <freejoint/>
      <geom type="box" pos="0 0 0" size="0.06 0.06 0.003" mass="0.2"/>
      <geom type="sphere" pos="0.02 0 -0.02" size="0.004" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>"#;

/// ROTATING-NORMAL-MATERIAL — the articulated material-trajectory gradient with the
/// contact normal tracking body orientation (`with_rotating_normal(true)`), so the carry
/// flows through the normal's `q`-dependence (the `δn̂ = ω×n̂` pose-adjoint + the wrench
/// `∂w/∂T` redirect + the `PoseTwistSeamVjp` seam) — terms a fixed-normal scene zeroes.
/// Same tied-μ + `coupled_trajectory_articulated_z` oracle as [`articulated_material_case`];
/// the oracle is built through the SAME `with_rotating_normal`-gated `build_contact`, so the
/// FD is rotating-normal too (a matched pair). `(w_idx, v_idx, half)` is the quaternion tilt
/// seed; the per-scene `band`/`n` keep the gentle-tilt fixture in its stable engaged regime
/// (the ball climbs out of contact past n ≈ 4, the free body is slow-climbing). Folds the FD
/// cells of `rotating_normal_gradient.rs`; that file keeps the rotating-vs-flat materiality
/// check and the single-step height-probe JACOBIAN (a different, Bucket-B shape).
#[allow(clippy::too_many_arguments)]
fn rotating_material_case(
    name: &'static str,
    mjcf: &'static str,
    w_idx: usize,
    v_idx: usize,
    half: f64,
    n: usize,
    band: (f64, f64),
) -> GradCase {
    GradCase {
        name,
        value_bounds: Some(band),
        baseline: vec![MU0],
        eps: vec![MU0 * 1e-4],
        tol: 1e-5,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (z, g_mu) = rotating_coupling(mjcf, MU0, w_idx, v_idx, half)
                .coupled_trajectory_material_gradient_articulated(n, 0);
            let g_la = rotating_coupling(mjcf, MU0, w_idx, v_idx, half)
                .coupled_trajectory_material_gradient_articulated(n, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(move |p| {
            rotating_coupling(mjcf, p[0], w_idx, v_idx, half).coupled_trajectory_articulated_z(n)
        }),
    }
}

/// MOVING-EE-MATERIAL — the articulated material-trajectory gradient with the finite sphere
/// end-effector tracking the arm geom (`with_contact_geom`), so the contact centre moves
/// LATERALLY (x) as well as vertically as the arm swings. The lateral generalization of
/// [`sphere_material_case`]'s curved-normal carry: the gradient threads the 3-vector centre
/// channel (`∂centre/∂q = J_geom`, the `PoseCentreVjp` seam) a height-only carry drops. Same
/// tied-μ + `coupled_trajectory_articulated_z` oracle as the other articulated rows (forward and
/// adjoint pose at the SAME geom each step, so the FD is moving-EE too). n=8: the gradient is
/// intrinsically near `LIVE_FLOOR` (~1.4e-9 at n=2) and grows with horizon, so a longer rollout
/// lifts it clear (~1.1e-8, ~11×) while the arm-reference height stays ~0.176. Folds the FD +
/// engagement tests of `sphere_moving_ee_trajectory_gradient.rs`; that file keeps the two
/// invariants the matrix can't express — the `#[should_panic]` free-body-friction contract guard
/// and the plane-collider byte-identity no-op cross-check.
fn moving_ee_material_case() -> GradCase {
    const N: usize = 8;
    GradCase {
        name: "sphere-moving-ee·material[μ]",
        value_bounds: Some((0.17, 0.185)),
        baseline: vec![MU0],
        eps: vec![MU0 * 1e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (z, g_mu) =
                moving_ee_coupling(MU0).coupled_trajectory_material_gradient_articulated(N, 0);
            let g_la = moving_ee_coupling(MU0)
                .coupled_trajectory_material_gradient_articulated(N, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| moving_ee_coupling(p[0]).coupled_trajectory_articulated_z(N)),
    }
}

/// SPHERE-ARTICULATED-MATERIAL — the material-trajectory gradient on the passive Y-hinge
/// arm pressing the block through a finite `SphereSdf` end-effector: the curved-normal
/// `f_mag·H` geometric-stiffness wrench term (zero for the plane `articulated_material_case`)
/// routed through the joint. The articulated counterpart of [`sphere_material_case`]'s free
/// platen and the curved sibling of [`articulated_material_case`]'s plane hinge, sharing the
/// tied-μ `coupled_trajectory_articulated_z` oracle. n=8: the gradient is intrinsically near
/// `LIVE_FLOOR` (~1.8e-9 at n=2) and grows with horizon, so a longer rollout lifts it clear
/// (~1.6e-8, ~16×) while the tip stays engaged near the block top. Folds the FD + engagement
/// tests of `sphere_articulated_trajectory_gradient.rs` (a clean full fold — no bespoke
/// invariant the matrix can't express, so that file is deleted).
fn sphere_articulated_material_case() -> GradCase {
    const N: usize = 8;
    GradCase {
        name: "sphere-articulated·material[μ]",
        value_bounds: Some((0.10, 0.118)),
        baseline: vec![MU0],
        eps: vec![MU0 * 1e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (z, g_mu) = sphere_articulated_coupling(MU0)
                .coupled_trajectory_material_gradient_articulated(N, 0);
            let g_la = sphere_articulated_coupling(MU0)
                .coupled_trajectory_material_gradient_articulated(N, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| {
            sphere_articulated_coupling(p[0]).coupled_trajectory_articulated_z(N)
        }),
    }
}

/// ACTUATOR — a real `<actuator>`'s per-step control schedule on the actuated hinge, FD'd via
/// `coupled_trajectory_actuated_z`. The same machinery covers any AFFINE actuator: a direct-torque
/// MOTOR (`force = gear·ctrl`) and the state-feedback SERVOS (position `kp·(ctrl−qpos)`, velocity
/// `kv·(ctrl−qvel)`, PD combining both) — a servo's `∂force/∂(qpos,qvel)` is a constant explicit
/// slope already carried by the analytic `J_state`, so no `ctrl`-replication is needed and all
/// controls are live (the torque drives the arm through the contact).
fn actuator_case(name: &'static str, actuator: &'static str, controls: Vec<f64>) -> GradCase {
    let n = controls.len();
    let base = controls.clone();
    GradCase {
        name,
        value_bounds: Some((0.10, 0.115)),
        eps: vec![1e-3; n],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live; n],
        baseline: controls,
        analytic: Box::new(move || {
            actuated_hinge_coupling(actuator).coupled_trajectory_actuator_gradient(&base)
        }),
        value_at: Box::new(move |p| {
            actuated_hinge_coupling(actuator).coupled_trajectory_actuated_z(p)
        }),
    }
}

/// PLATEN-MATERIAL — the multi-step material trajectory gradient on the free platen
/// (the half-plane counterpart of [`sphere_material_case`]): `∂z_N/∂μ` over a 20-step
/// engaged rollout via one `tape.backward`. Tied-μ (λ = 4μ → `∂z/∂μ + 4·∂z/∂λ`); FD
/// oracle = the generic tapeless `step_rollout` reading the platen's world height. The
/// fixture is the shared `traj_coupling` (platen 0.108, damping 60). Folds the
/// engaged-FD + forward tests of `coupled_trajectory_gradient.rs`; that file keeps its
/// machine-exact-at-all-lengths sweep, the one invariant a single-length row can't hold.
fn platen_material_case() -> GradCase {
    const N: usize = 20;
    GradCase {
        name: "platen·material[μ]",
        value_bounds: None,
        baseline: vec![MU0],
        eps: vec![MU0 * 5e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(|| {
            let (z, g_mu) = traj_coupling(MU0).coupled_trajectory_material_gradient(N, 0);
            let g_la = traj_coupling(MU0)
                .coupled_trajectory_material_gradient(N, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(|p| step_rollout(traj_coupling(p[0]), N, |c| c.data().xpos[1].z)),
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
        value_bounds: None,
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
        value_bounds: None,
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

/// IPC-STEP-MATERIAL — the single-step co-design gradient `∂vz'/∂p` on the `IpcRigidContact`
/// (C²-barrier) contact model: the IPC counterpart of [`material_case`], isolating the per-step
/// factor (the generic coupling + IPC contact + per-pair-curvature factors compose for ONE step)
/// from the multi-step carry. `idx` selects μ (0) or λ (1); `κ = 3e4`, engaged height `h = 0.105`
/// (`0 < sd < d̂`). Folds the single-step half of `ipc_trajectory_gradient.rs` at one representative
/// engaged height (this harness tests one operating point per cell; the deleted gate additionally
/// spot-checked `h = 0.103` / `0.107`, sample points of this same single-step FD-match).
fn ipc_step_material_case(idx: usize, name: &'static str, p0: f64) -> GradCase {
    const H: f64 = 0.105;
    const KAPPA: f64 = 3.0e4;
    GradCase {
        name,
        value_bounds: None,
        baseline: vec![p0],
        eps: vec![p0 * 5e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (vz, g) = ipc_coupling(MU0, KAPPA).coupled_step_material_gradient(H, idx);
            (vz, vec![g])
        }),
        value_at: Box::new(move |p| {
            ipc_coupling(MU0, KAPPA).coupled_step_material_vz(H, idx, p[0])
        }),
    }
}

/// IPC-TRAJ-MATERIAL — the multi-step material-trajectory gradient `dz_N/dμ` on the
/// `IpcRigidContact` contact model, over a 90-step rollout that descends through the contact
/// make/break boundary (the regime the keystone time-adjoint once reported as a 5–25% penalty
/// degradation; machine-exact once the rigid position-carry off-by-one was fixed — shared with
/// penalty, not an IPC effect). Tied-μ (λ = 4μ); FD oracle = the generic tapeless `step_rollout`
/// reading the platen height (bit-identical to the bespoke gate's `step().rigid_z` rollout). One
/// row per `kappa` — the harness keeps the deleted gate's full barrier-stiffness sweep
/// (κ ∈ {3e4, 3e3, 1e3, 3e2}, a 100× range) so a κ-dependent regression can't hide. n = 90 is
/// load-bearing: it is what carries the rollout through the make/break event.
fn ipc_traj_material_case(name: &'static str, kappa: f64) -> GradCase {
    const N: usize = 90;
    GradCase {
        name,
        value_bounds: Some((0.112, 0.117)),
        baseline: vec![MU0],
        eps: vec![MU0 * 5e-4],
        tol: 1e-6,
        floor: 1e-12,
        expect: vec![Comp::Live],
        analytic: Box::new(move || {
            let (z, g_mu) = ipc_coupling(MU0, kappa).coupled_trajectory_material_gradient(N, 0);
            let g_la = ipc_coupling(MU0, kappa)
                .coupled_trajectory_material_gradient(N, 1)
                .1;
            (z, vec![g_mu + 4.0 * g_la])
        }),
        value_at: Box::new(move |p| {
            step_rollout(ipc_coupling(p[0], kappa), N, |c| c.data().xpos[1].z)
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
        platen_material_case(),
        // single-step LOAD channel — two operating points per collider (shallow/deep),
        // mirroring the retired plane + sphere load gates
        load_case("load·plane[shallow]", step_coupling, 0.099, 5.0),
        load_case("load·plane[deep]", step_coupling, 0.097, 9.0),
        load_case("load·sphere[shallow]", load_sphere_coupling, 0.099, 5.0),
        load_case("load·sphere[deep]", load_sphere_coupling, 0.097, 9.0),
        // passive Y-hinge arm
        articulated_material_case(),
        // damped articulated arms (the M_impl = M + Δt·D channel): single hinge
        // (analytic damped J_state) and a 2-link chain (off-diagonal FD J_state)
        arm_material_case(
            "damped-hinge·material[μ]",
            DAMPED_HINGE_MJCF,
            1,
            &[(0, 0.3)],
            6,
            (0.10, 0.115),
            1e-4,
            1e-5,
        ),
        // n=20: the 2-link's tied-μ gradient grows with horizon (~5e-10 at n=2), so a
        // longer rollout lifts it clear of LIVE_FLOOR while the tip stays in the band.
        arm_material_case(
            "damped-2link·material[μ]",
            DAMPED_2LINK_MJCF,
            2,
            &[(0, 0.1), (1, -0.05)],
            20,
            (0.10, 0.115),
            1e-4,
            1e-5,
        ),
        // undamped serial chains (bare M⁻¹): the analytic chain J_state on a 2-link
        // (nv=2, off-diagonal M + Coriolis) and a 3-link multi-hop chain (nv=3, the
        // bias-acceleration transport a 2-link never reaches). Machine-exact at all
        // horizons (analytic carry, probed flat rel ~7.6e-10 to n=12 for the 2-link),
        // so one operating point per scene captures the carry; n stays in the stable
        // engaged regime (the whippy undamped 3-link launches off the block by n≈3, so
        // n=2 is its sole well-conditioned horizon, with eps=μ·1e-5 for the tighter FD).
        arm_material_case(
            "2link·material[μ]",
            TWOLINK_MJCF,
            2,
            &[(0, 0.1), (1, -0.05)],
            6,
            (0.130, 0.145),
            1e-4,
            1e-7,
        ),
        arm_material_case(
            "3link·material[μ]",
            THREELINK_MJCF,
            3,
            &[(0, 0.08), (1, -0.05), (2, -0.03)],
            2,
            (0.220, 0.232),
            1e-5,
            1e-4,
        ),
        // rotating contact normal (n̂ = R(q)·(0,0,−1)): ball joint and free base, gentle
        // Y-tilt. n/band are per-scene (the ball climbs out of contact past n≈4; the free
        // base sits in a higher, slow-climbing band than the hinge scenes).
        rotating_material_case(
            "ball-rot·material[μ]",
            ROT_BALL_MJCF,
            0,
            2,
            0.05,
            2,
            (0.10, 0.115),
        ),
        rotating_material_case(
            "free-rot·material[μ]",
            ROT_FREE_MJCF,
            3,
            5,
            0.02,
            6,
            (0.114, 0.123),
        ),
        // actuated Y-hinge
        actuator_case(
            "hinge·actuator(motor)",
            r#"<motor name="a" joint="j" gear="4"/>"#,
            vec![0.3, -0.2, 0.25, -0.15, 0.1, 0.2],
        ),
        actuator_case(
            "hinge·actuator(position)",
            r#"<position name="a" joint="j" kp="8"/>"#,
            vec![0.35, 0.25, 0.4, 0.2, 0.3, 0.28],
        ),
        actuator_case(
            "hinge·actuator(velocity)",
            r#"<velocity name="a" joint="j" kv="0.5"/>"#,
            vec![0.5, -0.3, 0.4, -0.2, 0.3, -0.1],
        ),
        actuator_case(
            "hinge·actuator(pd)",
            r#"<position name="a" joint="j" kp="8" kv="0.4"/>"#,
            vec![0.35, 0.25, 0.4, 0.2, 0.3, 0.28],
        ),
        // curved SphereSdf collider (generic tapeless step_rollout oracle)
        sphere_material_case(),
        // articulated finite sphere: the f_mag·H curved-wrench term routed through the joint
        sphere_articulated_material_case(),
        // moving end-effector: finite sphere tracking the arm geom (lateral centre channel)
        moving_ee_material_case(),
        // off-COM free body, contact moment ON (gates G_pos via orientation)
        freebody_orientation_case(),
        // IPC (C²-barrier) contact model, type-parameter variant: single-step material
        // (the isolated per-step factor) + the multi-step make/break rollout across the full
        // barrier-stiffness sweep the headline gate validated (κ = 3e4 … 3e2, 100×).
        ipc_step_material_case(0, "ipc-step·material[μ]", MU0),
        ipc_step_material_case(1, "ipc-step·material[λ]", LAMBDA0),
        ipc_traj_material_case("ipc-traj·material[μ,κ=3e4]", 3.0e4),
        ipc_traj_material_case("ipc-traj·material[μ,κ=3e3]", 3.0e3),
        ipc_traj_material_case("ipc-traj·material[μ,κ=1e3]", 1.0e3),
        ipc_traj_material_case("ipc-traj·material[μ,κ=3e2]", 3.0e2),
    ]
}

#[test]
fn coupling_gradient_matrix_matches_full_coupled_fd() {
    for case in cases() {
        eprintln!("=== {} ===", case.name);
        fd_check(&case);
    }
}
