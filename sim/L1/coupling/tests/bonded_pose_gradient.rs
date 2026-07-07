//! Rung 6d PR2 — FD gate for the differentiable bond: the reverse-mode gradient of the
//! bonded reaction wrenches w.r.t. the two body poses
//! ([`BondedSandwich::probe_with_pose_gradient`]).
//!
//! A rich scalar loss `L = cot_lower·w_lower + cot_upper·w_upper` (force + moment
//! cotangents on both endplates) is differentiated w.r.t. each body's world-frame twist
//! `[ω; v]`. The analytic gradient (Dirichlet reaction VJP + wrench-readout adjoint +
//! pose→target Jacobian) is compared against an INDEPENDENT re-probe FD: each twist DOF is
//! perturbed on a FRESH sandwich (linear = shift `xpos`, angular = world-frame rotation),
//! re-solved, and the loss finite-differenced — the FD never touches the analytic factor.
//! Evaluated at a flexed + compressed operating point so force AND moment (hence the
//! moment-arm term) are live. This is the reverse dual of the block spike that measured
//! `∂F_up.z/∂z = −9061.966`.
//!
//! Run on BOTH a centered body (`xipos = xpos`) and an OFF-CENTRE one (geom COM offset from
//! the joint origin, as an anatomical vertebra has) — the off-centre case exercises the
//! moment-arm (`xipos`) vs. target-rotation (`xpos`) reference-point split.

#![allow(
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::similar_names
)]

use nalgebra::{UnitQuaternion, Vector6};
use sim_coupling::{BondStep, BondedSandwich};
use sim_mjcf::load_model;
use sim_soft::Vec3;

/// `sim-core`'s `SpatialVector` (`[ang(3); lin(3)]` layout) — a `Vector6<f64>`.
type SpatialVector = Vector6<f64>;

const EDGE: f64 = 0.02;
const N: usize = 2;
const MU: f64 = 1.0e5;
const STATIC_DT: f64 = 1.0e3;
const H: f64 = 0.01;
const LOWER: usize = 1;
const UPPER: usize = 2;
const C: f64 = EDGE / 2.0;
const UPPER_REST_Z: f64 = EDGE + H;

/// FSU scene. `goff` offsets each box geom's centre along `+z` in its body frame, so the
/// auto-computed COM (`xipos`) sits `goff` off the joint origin (`xpos`) — `goff = 0` is
/// the centered body, `goff ≠ 0` an anatomical off-centre one. The bond (disc↔body) is
/// unchanged by `goff` (it references `xpos`), so the forward disc solve is identical; only
/// the wrench's moment reference (`xipos`) moves.
fn fsu_mjcf(goff: f64) -> String {
    format!(
        r#"<mujoco>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="L5" pos="{c} {c} {z5}"><freejoint/><geom type="box" pos="0 0 {goff}" size="{hx} {hx} {h}" mass="0.05"/></body>
    <body name="L4" pos="{c} {c} {z4}"><freejoint/><geom type="box" pos="0 0 {goff}" size="{hx} {hx} {h}" mass="0.05"/></body>
  </worldbody>
</mujoco>"#,
        c = C,
        z5 = -H,
        z4 = UPPER_REST_Z,
        hx = EDGE / 2.0,
        h = H,
        goff = goff,
    )
}

fn build(goff: f64) -> BondedSandwich {
    let model = load_model(&fsu_mjcf(goff)).expect("mjcf");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    BondedSandwich::new(model, data, LOWER, UPPER, N, EDGE, MU, STATIC_DT)
}

/// The operating-point poses: lower at rest; upper flexed θ about the medio-lateral axis
/// through the disc top-face centre and compressed δ axially.
fn operating_poses() -> ((Vec3, UnitQuaternion<f64>), (Vec3, UnitQuaternion<f64>)) {
    let lower = (Vec3::new(C, C, -H), UnitQuaternion::identity());
    let theta = 0.06;
    let delta = 0.05 * EDGE;
    let pivot = Vec3::new(C, C, EDGE);
    let rot = UnitQuaternion::from_axis_angle(&Vec3::x_axis(), theta);
    let rest_com = Vec3::new(C, C, UPPER_REST_Z - delta);
    let upper = (pivot + rot * (rest_com - pivot), rot);
    (lower, upper)
}

/// Rich cotangents on the two endplate wrenches (`[ang = ∂L/∂moment; lin = ∂L/∂force]`).
fn cotangents() -> (SpatialVector, SpatialVector) {
    let cl = SpatialVector::from_row_slice(&[0.1, 0.4, -0.2, -0.3, 0.1, 0.5]);
    let cu = SpatialVector::from_row_slice(&[0.5, -0.3, 0.2, 0.1, 0.2, 1.0]);
    (cl, cu)
}

fn loss(step: &BondStep, cl: SpatialVector, cu: SpatialVector) -> f64 {
    let dot = |ang: Vec3, lin: Vec3, m: Vec3, f: Vec3| {
        ang.x * m.x + ang.y * m.y + ang.z * m.z + lin.x * f.x + lin.y * f.y + lin.z * f.z
    };
    dot(
        Vec3::new(cl[0], cl[1], cl[2]),
        Vec3::new(cl[3], cl[4], cl[5]),
        step.moment_lower,
        step.force_lower,
    ) + dot(
        Vec3::new(cu[0], cu[1], cu[2]),
        Vec3::new(cu[3], cu[4], cu[5]),
        step.moment_upper,
        step.force_upper,
    )
}

/// Perturb `body`'s pose along world-frame twist DOF `m` (0..3 angular, 3..6 linear) by
/// `eps` on a fresh sandwich, re-probe, and return the loss.
fn loss_perturbed(body: usize, m: usize, eps: f64, goff: f64) -> f64 {
    let (lower, upper) = operating_poses();
    let (cl, cu) = cotangents();
    let mut c = build(goff);
    let mut poses = [lower, upper];
    let slot = if body == LOWER { 0 } else { 1 };
    let (pos, quat) = poses[slot];
    poses[slot] = if m < 3 {
        // angular: world-frame rotation about xpos (left-multiply).
        let mut axis = Vec3::zeros();
        axis[m] = 1.0;
        let dq = UnitQuaternion::from_scaled_axis(axis * eps);
        (pos, dq * quat)
    } else {
        // linear: shift xpos.
        let mut dp = Vec3::zeros();
        dp[m - 3] = eps;
        (pos + dp, quat)
    };
    c.set_body_pose(LOWER, poses[0].0, poses[0].1);
    c.set_body_pose(UPPER, poses[1].0, poses[1].1);
    let step = c.probe();
    loss(&step, cl, cu)
}

fn rel(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum();
    let den: f64 = b.iter().map(|y| y * y).sum();
    (num / den.max(1e-300)).sqrt()
}

/// FD the 6-twist gradient of L w.r.t. `body`'s pose at eps, via re-probe.
fn fd_gradient(body: usize, eps: f64, goff: f64) -> [f64; 6] {
    let mut g = [0.0; 6];
    for (m, gm) in g.iter_mut().enumerate() {
        *gm = (loss_perturbed(body, m, eps, goff) - loss_perturbed(body, m, -eps, goff))
            / (2.0 * eps);
    }
    g
}

/// The analytic pose gradient (both bodies) vs re-probe FD at COM offset `goff`.
fn run_gate(goff: f64) {
    let (lower, upper) = operating_poses();
    let (cl, cu) = cotangents();

    let mut c = build(goff);
    c.set_body_pose(LOWER, lower.0, lower.1);
    c.set_body_pose(UPPER, upper.0, upper.1);
    let (_step, grads) = c.probe_with_pose_gradient(cl, cu);
    let an_lower: [f64; 6] = std::array::from_fn(|k| grads[0][k]);
    let an_upper: [f64; 6] = std::array::from_fn(|k| grads[1][k]);

    let live = |g: &[f64; 6]| g.iter().map(|x| x.abs()).fold(0.0, f64::max);
    assert!(live(&an_upper) > 1.0, "upper pose gradient degenerate");
    assert!(live(&an_lower) > 1.0, "lower pose gradient degenerate");

    for (name, body, an) in [("upper", UPPER, &an_upper), ("lower", LOWER, &an_lower)] {
        let mut best = f64::INFINITY;
        for e in [1e-4, 1e-5, 1e-6, 1e-7] {
            let eps = e * EDGE;
            let fd = fd_gradient(body, eps, goff);
            let r = rel(an, &fd);
            eprintln!("[goff={goff:.3} {name}] eps={e:.1e}·EDGE  rel = {r:.3e}");
            best = best.min(r);
        }
        eprintln!("[goff={goff:.3} {name}] analytic = {an:?}");
        assert!(
            best < 1e-5,
            "{name} pose gradient must match re-probe FD (goff={goff}); best rel = {best:.3e}"
        );
    }
}

/// Centered body (`xipos = xpos`): the moment arm and target-rotation share a reference.
#[test]
fn bonded_pose_gradient_matches_reprobe_fd() {
    run_gate(0.0);
}

/// Off-centre body (`xipos ≠ xpos`, geom COM 5 mm off the joint origin) — exercises the
/// moment-arm (`xipos`) vs. target-rotation (`xpos`) reference split, the path an
/// anatomical vertebra (rung 6d PR3) takes.
#[test]
fn bonded_pose_gradient_offcenter_matches_reprobe_fd() {
    run_gate(0.005);
}
