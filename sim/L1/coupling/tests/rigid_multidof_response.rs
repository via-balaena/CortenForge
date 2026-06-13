//! Keystone multi-DOF rigid coupling, PR1 — the rigid factor in isolation.
//!
//! `rigid_xfrc_column` returns `∂qvel'/∂xfrc_applied[body] = Δt·M⁻¹·J_comᵀ`, the
//! matrix successor to the scalar free-body `∂vz'/∂fz = dt/m` the merged coupling
//! caps to. This gate validates it against an INDEPENDENT finite difference over a
//! real sim-core step (perturb each of the 6 spatial-force components, read the
//! next joint velocity) on:
//!   - a single HINGE — where the tip arcs, so a contact force maps to joint torque
//!     through the moment arm and the effective inertia `J·M⁻¹·Jᵀ ≠ m` (the scalar
//!     `dt/m` is simply wrong);
//!   - a 2-LINK arm — the genuine off-diagonal coupling (a force on the distal link
//!     accelerates the proximal joint too);
//!   - a FREE body — where the column collapses to the scalar `dt/m` on the contact
//!     axis, tying back to the merged platen path.

#![allow(clippy::expect_used)]

use nalgebra::DVector;
use sim_core::{Model, SpatialVector, max_relative_error};
use sim_coupling::rigid_xfrc_column;

/// Step a fresh scratch `Data` from `(qpos, qvel)` with a spatial force
/// `[τ(3); f(3)]` on `body`, returning the next joint-velocity vector.
fn next_qvel(
    model: &Model,
    qpos: &DVector<f64>,
    qvel: &DVector<f64>,
    body: usize,
    sf: [f64; 6],
) -> DVector<f64> {
    let mut d = model.make_data();
    d.qpos.copy_from(qpos);
    d.qvel.copy_from(qvel);
    let mut s = SpatialVector::zeros();
    for (i, &c) in sf.iter().enumerate() {
        s[i] = c;
    }
    d.xfrc_applied[body] = s;
    d.step(model).expect("scratch step");
    d.qvel.clone()
}

/// Central-difference `∂qvel'/∂xfrc[body]` (nv × 6) over the real step, evaluated
/// at the SAME configuration `(qpos, qvel)` the analytic column is taken at.
fn fd_column(
    model: &Model,
    qpos: &DVector<f64>,
    qvel: &DVector<f64>,
    body: usize,
    eps: f64,
) -> nalgebra::DMatrix<f64> {
    let mut fd = nalgebra::DMatrix::zeros(model.nv, 6);
    for c in 0..6 {
        let mut sp = [0.0; 6];
        let mut sm = [0.0; 6];
        sp[c] = eps;
        sm[c] = -eps;
        let vp = next_qvel(model, qpos, qvel, body, sp);
        let vm = next_qvel(model, qpos, qvel, body, sm);
        for r in 0..model.nv {
            fd[(r, c)] = (vp[r] - vm[r]) / (2.0 * eps);
        }
    }
    fd
}

#[test]
fn single_hinge_xfrc_column_matches_fd() {
    let model = Model::n_link_pendulum(1, 1.0, 2.0);
    let mut data = model.make_data();
    // Tilt off the axis so the moment arm couples all in-plane components.
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");

    let analytic = rigid_xfrc_column(&model, &data, 1); // 1 × 6
    let fd = fd_column(&model, &data.qpos, &data.qvel, 1, 1e-4);
    let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
    assert!(
        err < 1e-6,
        "hinge xfrc column must match FD, got rel {err:.3e}"
    );

    // It is genuinely NOT the scalar dt/m: the Y-hinge responds to τ_y, f_x AND f_z
    // (the moment-arm coupling), not a lone f_z·(dt/m).
    assert!(analytic[(0, 1)].abs() > 1e-6, "hinge must respond to τ_y");
    assert!(
        analytic[(0, 3)].abs() > 1e-6,
        "hinge must respond to f_x (moment arm)"
    );
    assert!(analytic[(0, 5)].abs() > 1e-6, "hinge must respond to f_z");
}

#[test]
fn two_link_xfrc_column_matches_fd_and_couples_offdiagonal() {
    let model = Model::n_link_pendulum(2, 0.7, 1.5);
    let mut data = model.make_data();
    data.qpos[0] = 0.2;
    data.qpos[1] = -0.4;
    data.forward(&model).expect("forward");

    let analytic = rigid_xfrc_column(&model, &data, 2); // 2 × 6
    let fd = fd_column(&model, &data.qpos, &data.qvel, 2, 1e-4);
    let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
    assert!(
        err < 1e-6,
        "2-link xfrc column must match FD, got rel {err:.3e}"
    );

    // Off-diagonal: a force on the DISTAL link (body 2) accelerates the PROXIMAL
    // joint (row 0) — the multi-DOF coupling the scalar dt/m cannot represent.
    assert!(
        analytic[(0, 5)].abs() > 1e-6,
        "+z force on distal link must move proximal joint, got {:.3e}",
        analytic[(0, 5)]
    );
}

#[test]
fn free_body_column_collapses_to_dt_over_m() {
    // A single free-joint box (the merged platen): the column must reduce to the
    // scalar dt/m on the contact axis (qvel[2] = vz vs xfrc[5] = f_z).
    const PLATEN: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = sim_mjcf::load_model(PLATEN).expect("platen loads");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let analytic = rigid_xfrc_column(&model, &data, 1); // 6 × 6
    let dt_over_m = model.timestep / 0.2;
    assert!(
        (analytic[(2, 5)] - dt_over_m).abs() < 1e-12,
        "free-body ∂vz'/∂f_z must be dt/m = {dt_over_m:.6e}, got {:.6e}",
        analytic[(2, 5)]
    );

    // And it agrees with FD across the full 6×6.
    let fd = fd_column(&model, &data.qpos, &data.qvel, 1, 1e-3);
    let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
    assert!(
        err < 1e-6,
        "free-body column must match FD, got rel {err:.3e}"
    );
}
