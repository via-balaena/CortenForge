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
    // ...and is exactly DEAF to the components orthogonal to the Y-hinge axis
    // (τ_x, τ_z, f_y produce no rotation about Y) — the structure the scalar can't
    // express. These rows are ~0 in the FD too, but assert them directly so a
    // spurious cross-coupling can't hide behind the aggregate FD match.
    for &(comp, name) in &[(0, "τ_x"), (2, "τ_z"), (4, "f_y")] {
        assert!(
            analytic[(0, comp)].abs() < 1e-12,
            "Y-hinge must be deaf to {name}, got {:.3e}",
            analytic[(0, comp)]
        );
    }
}

#[test]
fn two_link_xfrc_column_matches_fd_and_couples_offdiagonal() {
    let model = Model::n_link_pendulum(2, 0.7, 1.5);
    let mut data = model.make_data();
    data.qpos[0] = 0.2;
    data.qpos[1] = -0.4;
    // Nonzero joint velocities: the column is qvel-independent by construction
    // (Δt·M⁻¹·Jᵀ) and the central FD over xfrc cancels the Coriolis/bias baseline,
    // so the match must hold away from rest too — exercises that invariance.
    data.qvel[0] = 0.7;
    data.qvel[1] = -1.3;
    data.forward(&model).expect("forward");

    let analytic = rigid_xfrc_column(&model, &data, 2); // 2 × 6
    let fd = fd_column(&model, &data.qpos, &data.qvel, 2, 1e-4);
    let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
    assert!(
        err < 1e-6,
        "2-link xfrc column must match FD, got rel {err:.3e}"
    );

    // Off-diagonal: a +z force on the DISTAL link (body 2) accelerates BOTH the
    // proximal joint (row 0) and the distal joint (row 1) — the multi-DOF coupling
    // the scalar dt/m cannot represent (a scalar would touch one DOF only).
    assert!(
        analytic[(0, 5)].abs() > 1e-6,
        "+z force on distal link must move proximal joint, got {:.3e}",
        analytic[(0, 5)]
    );
    assert!(
        analytic[(1, 5)].abs() > 1e-6,
        "+z force on distal link must move distal joint, got {:.3e}",
        analytic[(1, 5)]
    );
}

/// With joint DAMPING the Euler `eulerdamp` solves `(M + Δt·D)·qacc = F`, so the
/// wrench→velocity factor is `Δt·(M + Δt·D)⁻¹·Jᵀ` — `rigid_xfrc_column` must use the
/// implicit mass matrix, not bare `M`. A damped hinge + a damped 2-link chain
/// (off-diagonal `M`, so the `M + Δt·D` coupling is live) match the FD over the real
/// (damped) step; bare `M⁻¹` is materially wrong (≈28% hinge, ≈10× chain — asserted in
/// the S0 spike). FD over the real step is the ground truth since `step` runs eulerdamp.
#[test]
fn damped_xfrc_column_matches_fd() {
    const DAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    const DAMPED_2LINK: &str = r#"<mujoco>
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
    for (mjcf, body, seed, label) in [
        (DAMPED_HINGE, 1usize, vec![(0usize, 0.3)], "hinge"),
        (DAMPED_2LINK, 2, vec![(0, 0.1), (1, -0.05)], "2link"),
    ] {
        let model = sim_mjcf::load_model(mjcf).expect("damped model loads");
        let mut data = model.make_data();
        for (i, q) in seed {
            data.qpos[i] = q;
        }
        data.forward(&model).expect("forward");
        assert!(
            model.implicit_damping.iter().any(|&d| d > 0.0),
            "{label}: damping must be live in the model"
        );
        let analytic = rigid_xfrc_column(&model, &data, body);
        let fd = fd_column(&model, &data.qpos, &data.qvel, body, 1e-4);
        let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
        assert!(
            err < 1e-6,
            "{label}: damped xfrc column Δt·(M+Δt·D)⁻¹·Jᵀ must match FD, got rel {err:.3e}"
        );
    }
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
    let dt_over_m = model.timestep / model.body_mass[1]; // mass read from the model, not hardcoded
    assert!(
        (analytic[(2, 5)] - dt_over_m).abs() < 1e-12,
        "free-body ∂vz'/∂f_z must be dt/m = {dt_over_m:.6e}, got {:.6e}",
        analytic[(2, 5)]
    );

    // And it agrees with FD across the full 6×6.
    let fd = fd_column(&model, &data.qpos, &data.qvel, 1, 1e-4);
    let (err, _) = max_relative_error(&analytic, &fd, 1e-10);
    assert!(
        err < 1e-6,
        "free-body column must match FD, got rel {err:.3e}"
    );
}
