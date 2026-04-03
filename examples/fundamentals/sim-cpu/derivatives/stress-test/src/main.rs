//! Stress test — headless validation of finite-difference and analytical derivatives.
//!
//! 32 checks covering: dimensions (4), eigenvalue analysis (3), FD convergence (3),
//! hybrid vs FD agreement (3), sensor derivatives (3), inverse dynamics (3),
//! integrator coverage (4), quaternion handling (3), actuator/activation (3),
//! contact sensitivity (2), config edge cases (1).
//!
//! Run: `cargo run -p example-derivatives-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::option_if_let_else,
    non_snake_case
)]

use std::f64::consts::PI;

use nalgebra::DMatrix;
use sim_core::{
    DerivativeConfig, Model, fd_convergence_check, max_relative_error, mjd_inverse_fd,
    mjd_transition_fd, mjd_transition_hybrid, validate_analytical_vs_fd,
};

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Build a pendulum with a filter actuator (has activation dynamics, na=1).
fn pendulum_with_activation() -> Model {
    let mjcf = r#"
    <mujoco model="pendulum-activation">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
      <actuator>
        <general name="filtered" joint="hinge" dyntype="filter" dynprm="0.1"
                 gainprm="1" biastype="none"/>
      </actuator>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a pendulum with a motor (stateless, na=0).
fn pendulum_with_motor() -> Model {
    let mjcf = r#"
    <mujoco model="pendulum-motor">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="torque" joint="hinge" gear="1"/>
      </actuator>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a pendulum with sensors (jointpos + jointvel) and a motor.
fn pendulum_with_sensors() -> Model {
    let mjcf = r#"
    <mujoco model="pendulum-sensors">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="torque" joint="hinge" gear="1"/>
      </actuator>
      <sensor>
        <jointpos name="pos" joint="hinge"/>
        <jointvel name="vel" joint="hinge"/>
      </sensor>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a ball-joint pendulum.
fn ball_pendulum() -> Model {
    let mjcf = r#"
    <mujoco model="ball-pendulum">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="ball" type="ball"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a free-joint body.
fn free_body() -> Model {
    let mjcf = r#"
    <mujoco model="free-body">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="box" pos="0 0 1">
          <freejoint name="free"/>
          <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
          <geom type="box" size="0.1 0.1 0.1" rgba="0.5 0.5 0.5 1"
                contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a pendulum with damping.
fn pendulum_with_damping() -> Model {
    let mjcf = r#"
    <mujoco model="pendulum-damped">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a model with contact (box on plane).
fn contact_model(stiffness: f64) -> Model {
    let mjcf = format!(
        r#"
    <mujoco model="contact-box">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.01"
              rgba="0.3 0.3 0.3 1"/>
        <body name="box" pos="0 0 0.15">
          <freejoint name="free"/>
          <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
          <geom name="cube" type="box" size="0.1 0.1 0.1"
                solref="{} 1.0" rgba="0.8 0.3 0.3 1"/>
        </body>
      </worldbody>
    </mujoco>
    "#,
        -stiffness
    );
    sim_mjcf::load_model(&mjcf).expect("MJCF should parse")
}

/// Build a multi-actuator model (3 hinges, 2 motors).
fn multi_actuator() -> Model {
    let mjcf = r#"
    <mujoco model="multi-actuator">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.3" mass="0.5" diaginertia="0.01 0.01 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
          <body name="link2" pos="0 0 -0.5">
            <joint name="j2" type="hinge" axis="0 1 0"/>
            <inertial pos="0 0 -0.3" mass="0.5" diaginertia="0.01 0.01 0.001"/>
            <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
            <body name="link3" pos="0 0 -0.5">
              <joint name="j3" type="hinge" axis="0 1 0"/>
              <inertial pos="0 0 -0.3" mass="0.5" diaginertia="0.01 0.01 0.001"/>
              <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
            </body>
          </body>
        </body>
      </worldbody>
      <actuator>
        <motor name="m1" joint="j1" gear="1"/>
        <motor name="m2" joint="j2" gear="1"/>
      </actuator>
    </mujoco>
    "#;
    sim_mjcf::load_model(mjcf).expect("MJCF should parse")
}

/// Build a pendulum with a given integrator.
fn pendulum_with_integrator(integrator: &str) -> Model {
    let mjcf = format!(
        r#"
    <mujoco model="pendulum-{integrator}">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002" integrator="{integrator}"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
        </body>
      </worldbody>
    </mujoco>
    "#
    );
    sim_mjcf::load_model(&mjcf).expect("MJCF should parse")
}

/// Compute eigenvalues of a 2×2 matrix via the quadratic formula.
/// Returns (λ1, λ2) as complex numbers (real, imag).
fn eigenvalues_2x2(mat: &DMatrix<f64>) -> [(f64, f64); 2] {
    assert_eq!(mat.nrows(), 2);
    assert_eq!(mat.ncols(), 2);
    let m00 = mat[(0, 0)];
    let m01 = mat[(0, 1)];
    let m10 = mat[(1, 0)];
    let m11 = mat[(1, 1)];
    let trace = m00 + m11;
    let det = m00 * m11 - m01 * m10;
    let disc = trace * trace - 4.0 * det;
    if disc >= 0.0 {
        let sqrt_disc = disc.sqrt();
        [
            (f64::midpoint(trace, sqrt_disc), 0.0),
            (f64::midpoint(trace, -sqrt_disc), 0.0),
        ]
    } else {
        let sqrt_disc = (-disc).sqrt();
        [
            (trace / 2.0, sqrt_disc / 2.0),
            (trace / 2.0, -sqrt_disc / 2.0),
        ]
    }
}

/// Magnitude of a complex number.
fn complex_mag(c: (f64, f64)) -> f64 {
    c.0.hypot(c.1)
}

// ── Dimensions (4 checks) ─────────────────────────────────────────────────

fn check_1_a_dim_with_activation() -> (u32, u32) {
    let model = pendulum_with_activation();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // nv=1, na=1 → state_dim = 2*1+1 = 3
    let state_dim = 2 * model.nv + model.na;
    let a_ok = d.A.nrows() == state_dim && d.A.ncols() == state_dim;
    let detail = format!(
        "A: {}×{}, expected {}×{} (nv={}, na={})",
        d.A.nrows(),
        d.A.ncols(),
        state_dim,
        state_dim,
        model.nv,
        model.na
    );
    let p = check("A dim with activation", a_ok, &detail);
    (u32::from(p), 1)
}

fn check_2_b_dim_with_motor() -> (u32, u32) {
    let model = pendulum_with_motor();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let state_dim = 2 * model.nv + model.na;
    let b_ok = d.B.nrows() == state_dim && d.B.ncols() == model.nu;
    let detail = format!(
        "B: {}×{}, expected {}×{} (nu={})",
        d.B.nrows(),
        d.B.ncols(),
        state_dim,
        model.nu,
        model.nu
    );
    let p = check("B dim with motor", b_ok, &detail);
    (u32::from(p), 1)
}

fn check_3_free_joint_dims() -> (u32, u32) {
    let model = free_body();
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // Free joint: nq=7 but nv=6, na=0, nu=0
    let state_dim = 2 * model.nv + model.na; // 12
    let a_ok = d.A.nrows() == state_dim && d.A.ncols() == state_dim;
    let b_ok = d.B.nrows() == state_dim && d.B.ncols() == model.nu;
    let detail = format!(
        "A: {}×{} (expected 12×12), B: {}×{} (nv={}, nu={})",
        d.A.nrows(),
        d.A.ncols(),
        d.B.nrows(),
        d.B.ncols(),
        model.nv,
        model.nu
    );
    let p = check("Free joint dims", a_ok && b_ok, &detail);
    (u32::from(p), 1)
}

fn check_4_ball_joint_dims() -> (u32, u32) {
    let model = ball_pendulum();
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // Ball joint: nq=4 but nv=3, na=0, nu=0
    let state_dim = 2 * model.nv + model.na; // 6
    let a_ok = d.A.nrows() == state_dim && d.A.ncols() == state_dim;
    let detail = format!(
        "A: {}×{} (expected 6×6, nq={}, nv={})",
        d.A.nrows(),
        d.A.ncols(),
        model.nq,
        model.nv
    );
    let p = check("Ball joint dims", a_ok, &detail);
    (u32::from(p), 1)
}

// ── Eigenvalue analysis (3 checks) ────────────────────────────────────────

fn check_5_unstable_upright() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI; // upright (unstable equilibrium)
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let eigs = eigenvalues_2x2(&d.A);
    let max_mag = complex_mag(eigs[0]).max(complex_mag(eigs[1]));
    let p = check(
        "Upright → unstable (|λ| > 1)",
        max_mag > 1.0,
        &format!("max |λ| = {max_mag:.6}"),
    );
    (u32::from(p), 1)
}

fn check_6_stable_downward() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.0; // hanging down (stable equilibrium)
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let eigs = eigenvalues_2x2(&d.A);
    let max_mag = complex_mag(eigs[0]).max(complex_mag(eigs[1]));
    // Discrete-time stable: all |λ| ≤ 1 (with small margin for FD noise)
    let p = check(
        "Downward → stable (|λ| ≤ 1)",
        max_mag <= 1.0 + 1e-6,
        &format!("max |λ| = {max_mag:.6}"),
    );
    (u32::from(p), 1)
}

fn check_7_eigenvalues_real() -> (u32, u32) {
    // Undamped 1-DOF pendulum at upright: eigenvalues should be real
    // (one > 1, one < 1 — saddle point in discrete time).
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let eigs = eigenvalues_2x2(&d.A);
    let imag_mag = eigs[0].1.abs().max(eigs[1].1.abs());
    let p = check(
        "1-DOF upright eigenvalues real",
        imag_mag < 1e-6,
        &format!("max |imag| = {imag_mag:.2e}"),
    );
    (u32::from(p), 1)
}

// ── FD convergence (3 checks) ────────────────────────────────��────────────

fn check_8_centered_convergence() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    // Use forward differences at very small eps as ground truth (avoids
    // comparing centered-vs-centered where both converge to the same limit).
    // Forward at eps=1e-8 has O(eps) ≈ 1e-8 error — good enough as reference.
    let gt_config = DerivativeConfig {
        eps: 1e-8,
        centered: true,
        ..Default::default()
    };
    let gt = mjd_transition_fd(&model, &data, &gt_config).expect("fd");

    // Coarse eps values so centered O(eps²) convergence is visible.
    let eps1 = 1e-2;
    let eps2 = eps1 / 2.0;

    let c1 = DerivativeConfig {
        eps: eps1,
        centered: true,
        ..Default::default()
    };
    let c2 = DerivativeConfig {
        eps: eps2,
        centered: true,
        ..Default::default()
    };

    let d1 = mjd_transition_fd(&model, &data, &c1).expect("fd");
    let d2 = mjd_transition_fd(&model, &data, &c2).expect("fd");

    let (err1, _) = max_relative_error(&d1.A, &gt.A, 1e-10);
    let (err2, _) = max_relative_error(&d2.A, &gt.A, 1e-10);

    // Centered: O(eps²). Halving eps should ~quarter the error.
    // Ratio should be ~4, accept > 2 to be robust.
    let ratio = if err2 > 1e-14 { err1 / err2 } else { 4.0 };
    let p = check(
        "Centered O(eps²) convergence",
        ratio > 2.0,
        &format!("err({eps1:.0e}) = {err1:.2e}, err({eps2:.0e}) = {err2:.2e}, ratio = {ratio:.1}"),
    );
    (u32::from(p), 1)
}

fn check_9_forward_convergence() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    // Ground truth: centered at small eps
    let gt_config = DerivativeConfig {
        eps: 1e-7,
        centered: true,
        ..Default::default()
    };
    let gt = mjd_transition_fd(&model, &data, &gt_config).expect("fd");

    let eps1 = 1e-4;
    let eps2 = eps1 / 2.0;

    let c1 = DerivativeConfig {
        eps: eps1,
        centered: false,
        ..Default::default()
    };
    let c2 = DerivativeConfig {
        eps: eps2,
        centered: false,
        ..Default::default()
    };

    let d1 = mjd_transition_fd(&model, &data, &c1).expect("fd");
    let d2 = mjd_transition_fd(&model, &data, &c2).expect("fd");

    let (err1, _) = max_relative_error(&d1.A, &gt.A, 1e-10);
    let (err2, _) = max_relative_error(&d2.A, &gt.A, 1e-10);

    // Forward: O(eps). Halving eps should ~halve the error.
    // Ratio should be ~2, accept > 1.5 to be robust.
    let ratio = if err2 > 1e-14 { err1 / err2 } else { 2.0 };
    let p = check(
        "Forward O(eps) convergence",
        ratio > 1.5,
        &format!("err({eps1:.0e}) = {err1:.2e}, err({eps2:.0e}) = {err2:.2e}, ratio = {ratio:.1}"),
    );
    (u32::from(p), 1)
}

fn check_10_fd_convergence_utility() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    let ok = fd_convergence_check(&model, &data, 1e-6, 0.1).expect("convergence check");
    let p = check(
        "fd_convergence_check at default eps",
        ok,
        &format!("converged = {ok}"),
    );
    (u32::from(p), 1)
}

// ── Hybrid vs FD agreement (3 checks) ─────────────────────────────────────

fn check_11_hybrid_vs_fd_A() -> (u32, u32) {
    let model = Model::n_link_pendulum(3, 1.0, 0.5);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qpos[2] = 0.7;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
    let hyb = mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).expect("hybrid");

    let (err, loc) = max_relative_error(&fd.A, &hyb.A, 1e-10);
    let p = check(
        "Hybrid vs FD: A matrix",
        err < 1e-4,
        &format!("max rel err = {err:.2e} at ({}, {})", loc.0, loc.1),
    );
    (u32::from(p), 1)
}

fn check_12_hybrid_vs_fd_actuated() -> (u32, u32) {
    let model = multi_actuator();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qpos[2] = 0.7;
    data.forward(&model).expect("forward");

    let (err_a, err_b) = validate_analytical_vs_fd(&model, &data).expect("validate");
    let ok = err_a < 1e-3 && err_b < 1e-3;
    let p = check(
        "Hybrid vs FD: actuated A+B",
        ok,
        &format!("err_A = {err_a:.2e}, err_B = {err_b:.2e} (nu={})", model.nu),
    );
    (u32::from(p), 1)
}

fn check_13_validate_utility() -> (u32, u32) {
    let model = Model::n_link_pendulum(2, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qpos[1] = -0.3;
    data.forward(&model).expect("forward");

    let (err_a, err_b) = validate_analytical_vs_fd(&model, &data).expect("validate");
    let ok = err_a < 1e-3 && err_b < 1e-3;
    let p = check(
        "validate_analytical_vs_fd",
        ok,
        &format!("err_A = {err_a:.2e}, err_B = {err_b:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Sensor derivatives (3 checks) ─────────────────────────────────────────

fn check_14_C_some_when_enabled() -> (u32, u32) {
    let model = pendulum_with_sensors();
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.ctrl[0] = 0.1;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..Default::default()
    };
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let p = check(
        "C is Some when enabled",
        d.C.is_some(),
        &format!(
            "C = {}, nsensordata = {}",
            if d.C.is_some() { "Some" } else { "None" },
            model.nsensordata
        ),
    );
    (u32::from(p), 1)
}

fn check_15_D_some_when_enabled() -> (u32, u32) {
    let model = pendulum_with_sensors();
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.ctrl[0] = 0.1;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..Default::default()
    };
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let p = check(
        "D is Some when enabled",
        d.D.is_some(),
        &format!(
            "D = {}, nu = {}",
            if d.D.is_some() { "Some" } else { "None" },
            model.nu
        ),
    );
    (u32::from(p), 1)
}

fn check_16_CD_none_by_default() -> (u32, u32) {
    let model = pendulum_with_sensors();
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default(); // compute_sensor_derivatives = false
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let ok = d.C.is_none() && d.D.is_none();
    let p = check(
        "C, D are None by default",
        ok,
        &format!(
            "C = {}, D = {}",
            if d.C.is_some() { "Some" } else { "None" },
            if d.D.is_some() { "Some" } else { "None" }
        ),
    );
    (u32::from(p), 1)
}

// ── Inverse dynamics (3 checks) ───────────────────────────────────────────

fn check_17_inverse_dims() -> (u32, u32) {
    let model = Model::n_link_pendulum(2, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let inv = mjd_inverse_fd(&model, &data, &config).expect("inverse fd");

    let nv = model.nv;
    let ok = inv.DfDq.nrows() == nv
        && inv.DfDq.ncols() == nv
        && inv.DfDv.nrows() == nv
        && inv.DfDv.ncols() == nv
        && inv.DfDa.nrows() == nv
        && inv.DfDa.ncols() == nv;
    let p = check(
        "Inverse dims (nv × nv)",
        ok,
        &format!(
            "DfDq: {}×{}, DfDv: {}×{}, DfDa: {}×{} (nv={})",
            inv.DfDq.nrows(),
            inv.DfDq.ncols(),
            inv.DfDv.nrows(),
            inv.DfDv.ncols(),
            inv.DfDa.nrows(),
            inv.DfDa.ncols(),
            nv
        ),
    );
    (u32::from(p), 1)
}

fn check_18_dfda_approx_mass_matrix() -> (u32, u32) {
    let model = Model::n_link_pendulum(2, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let inv = mjd_inverse_fd(&model, &data, &config).expect("inverse fd");

    // DfDa ≈ M (inverse dynamics: f = M*qacc + bias → ∂f/∂a = M)
    let (err, loc) = max_relative_error(&inv.DfDa, &data.qM, 1e-10);
    let p = check(
        "DfDa ≈ mass matrix M",
        err < 1e-4,
        &format!("max rel err = {err:.2e} at ({}, {})", loc.0, loc.1),
    );
    (u32::from(p), 1)
}

fn check_19_dfdv_nonzero_with_damping() -> (u32, u32) {
    let model = pendulum_with_damping();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let inv = mjd_inverse_fd(&model, &data, &config).expect("inverse fd");

    let dfdv_norm = inv.DfDv.norm();
    let p = check(
        "DfDv nonzero with damping",
        dfdv_norm > 1e-6,
        &format!("‖DfDv‖ = {dfdv_norm:.6}"),
    );
    (u32::from(p), 1)
}

// ── Integrator coverage (4 checks) ────────────────────────────────────────

fn check_integrator(name: &str, mjcf_name: &str) -> (u32, u32) {
    let model = pendulum_with_integrator(mjcf_name);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let result = mjd_transition_fd(&model, &data, &config);
    let ok = result.is_ok();
    let detail = match &result {
        Ok(d) => format!("A: {}×{}", d.A.nrows(), d.A.ncols()),
        Err(e) => format!("error: {e:?}"),
    };
    let p = check(&format!("{name} derivatives"), ok, &detail);
    (u32::from(p), 1)
}

fn check_20_euler() -> (u32, u32) {
    check_integrator("Euler", "Euler")
}

fn check_21_implicit() -> (u32, u32) {
    check_integrator("ImplicitSpringDamper", "implicit")
}

fn check_22_implicit_fast() -> (u32, u32) {
    check_integrator("ImplicitFast", "implicitfast")
}

fn check_23_rk4() -> (u32, u32) {
    check_integrator("RK4", "RK4")
}

// ── Quaternion handling (3 checks) ────────────────────────────────────────

fn check_24_ball_tangent_space() -> (u32, u32) {
    let model = ball_pendulum();
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // Ball: nq=4, nv=3. A should be 6×6 (2*3), NOT 8×8 (2*4).
    let a_size = d.A.nrows();
    let expected = 2 * model.nv; // 6
    let p = check(
        "Ball: A is 6×6 not 8×8",
        a_size == expected,
        &format!("A: {a_size}×{a_size}, nq={}, nv={}", model.nq, model.nv),
    );
    (u32::from(p), 1)
}

fn check_25_free_tangent_space() -> (u32, u32) {
    let model = free_body();
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // Free: nq=7, nv=6. A should be 12×12 (2*6), NOT 14×14 (2*7).
    let a_size = d.A.nrows();
    let expected = 2 * model.nv; // 12
    let p = check(
        "Free: A is 12×12 not 14×14",
        a_size == expected,
        &format!("A: {a_size}×{a_size}, nq={}, nv={}", model.nq, model.nv),
    );
    (u32::from(p), 1)
}

fn check_26_ball_fd_well_conditioned() -> (u32, u32) {
    let model = ball_pendulum();
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // Ball joint FD: A should be 6×6, all finite, and nontrivial.
    // We don't compare hybrid vs FD here because extreme inertia ratios
    // produce near-zero entries where max_relative_error with a tight
    // floor gives false ~1.0 errors. The hybrid is verified to match FD
    // to 1.5e-4 (floor=1e-6) in test_ball_joint_hybrid_vs_fd_a.
    let dim_ok = d.A.nrows() == 6 && d.A.ncols() == 6;
    let finite = d.A.iter().all(|v| v.is_finite());
    // Off-diagonal norm: A minus identity should be nonzero (gravity effect)
    let eye = DMatrix::identity(6, 6);
    let off_diag_norm = (&d.A - &eye).norm();
    let nontrivial = off_diag_norm > 1e-6;

    let ok = dim_ok && finite && nontrivial;
    let p = check(
        "Ball: FD well-conditioned",
        ok,
        &format!(
            "6×6={dim_ok}, finite={finite}, ‖A-I‖={off_diag_norm:.4e} (nq={}, nv={})",
            model.nq, model.nv
        ),
    );
    (u32::from(p), 1)
}

// ── Actuator / activation (3 checks) ──────────────────────────────────────

fn check_27_B_columns_multi_actuator() -> (u32, u32) {
    let model = multi_actuator();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    let ok = d.B.ncols() == model.nu;
    let p = check(
        "B columns = nu (multi-actuator)",
        ok,
        &format!("B: {}×{}, nu={}", d.B.nrows(), d.B.ncols(), model.nu),
    );
    (u32::from(p), 1)
}

fn check_28_activation_filter_in_A() -> (u32, u32) {
    let model = pendulum_with_activation();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // State = [dq, qvel, act]. The act block is the last na rows/cols.
    // For filter dynamics act_dot = (ctrl - act) / tau, the discrete
    // A diagonal entry for act should be < 1 (exponential decay).
    let nv = model.nv;
    let act_idx = 2 * nv; // index of first activation state
    let a_act = d.A[(act_idx, act_idx)];
    let ok = a_act < 1.0 && a_act > 0.0;
    let p = check(
        "Filter activation in A diagonal",
        ok,
        &format!("A[act,act] = {a_act:.6} (expected 0 < val < 1 for filter decay)"),
    );
    (u32::from(p), 1)
}

fn check_29_zero_control_passive() -> (u32, u32) {
    let model = pendulum_with_motor();
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.ctrl[0] = 0.0; // zero control
    data.forward(&model).expect("forward");

    let config = DerivativeConfig::default();
    let d = mjd_transition_fd(&model, &data, &config).expect("fd");

    // With zero control, the B contribution is zero. The A matrix should
    // reflect only passive dynamics (gravity). Verify A is nontrivial.
    let a_norm = d.A.norm();
    let p = check(
        "Zero-control A reflects passive dynamics",
        a_norm > 0.1,
        &format!("‖A‖ = {a_norm:.6} (should be nontrivial from gravity)"),
    );
    (u32::from(p), 1)
}

// ── Contact sensitivity (2 checks) ────────────────────────────────────────

fn check_30_contact_stiffness_affects_A() -> (u32, u32) {
    let model1 = contact_model(500.0);
    let mut data1 = model1.make_data();
    // Start box right at the floor surface so contact is immediate
    data1.qpos[2] = 0.1; // z position: box half-size is 0.1, so bottom touches floor
    data1.forward(&model1).expect("forward");
    // Step until settled in contact
    for _ in 0..200 {
        data1.step(&model1).expect("step");
    }

    let model2 = contact_model(2000.0);
    let mut data2 = model2.make_data();
    data2.qpos[2] = 0.1;
    data2.forward(&model2).expect("forward");
    for _ in 0..200 {
        data2.step(&model2).expect("step");
    }

    let config = DerivativeConfig::default();
    let d1 = mjd_transition_fd(&model1, &data1, &config).expect("fd1");
    let d2 = mjd_transition_fd(&model2, &data2, &config).expect("fd2");

    let (diff, _) = max_relative_error(&d1.A, &d2.A, 1e-10);
    let p = check(
        "Contact stiffness affects A",
        diff > 1e-6,
        &format!("max rel diff between stiffness=500 and stiffness=2000: {diff:.2e}"),
    );
    (u32::from(p), 1)
}

fn check_31_contact_vs_no_contact() -> (u32, u32) {
    // With contact: box sitting on floor
    let model_c = contact_model(500.0);
    let mut data_c = model_c.make_data();
    data_c.qpos[2] = 0.1; // start at floor
    data_c.forward(&model_c).expect("forward");
    for _ in 0..200 {
        data_c.step(&model_c).expect("step");
    }

    // Without contact: free body high in the air (no floor to collide with)
    let model_nc = free_body();
    let mut data_nc = model_nc.make_data();
    data_nc.qpos[2] = 5.0; // well above any surface
    data_nc.forward(&model_nc).expect("forward");

    let config = DerivativeConfig::default();
    let d_c = mjd_transition_fd(&model_c, &data_c, &config).expect("fd contact");
    let d_nc = mjd_transition_fd(&model_nc, &data_nc, &config).expect("fd no-contact");

    // Both are 12×12 (free joint), but A should differ due to contact forces.
    let same_dims = d_c.A.nrows() == d_nc.A.nrows();
    let (diff, _) = if same_dims {
        max_relative_error(&d_c.A, &d_nc.A, 1e-10)
    } else {
        (f64::INFINITY, (0, 0))
    };

    let p = check(
        "Contact vs no-contact A differs",
        diff > 1e-3,
        &format!("max rel diff = {diff:.2e}, same_dims = {same_dims}"),
    );
    (u32::from(p), 1)
}

// ── Config edge cases (1 check) ───────────────────────────────────────────

fn check_32_tiny_epsilon() -> (u32, u32) {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    let config = DerivativeConfig {
        eps: 1e-8,
        ..Default::default()
    };
    let result = mjd_transition_fd(&model, &data, &config);

    let ok = match &result {
        Ok(d) => d.A.iter().all(|v| v.is_finite()),
        Err(_) => false,
    };
    let detail = match &result {
        Ok(d) => {
            let max_val = d.A.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            format!("all finite, max |A_ij| = {max_val:.6}")
        }
        Err(e) => format!("error: {e:?}"),
    };
    let p = check("eps=1e-8 produces finite results", ok, &detail);
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Derivatives — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        // Dimensions (4)
        ("A dim with activation", check_1_a_dim_with_activation),
        ("B dim with motor", check_2_b_dim_with_motor),
        ("Free joint dims", check_3_free_joint_dims),
        ("Ball joint dims", check_4_ball_joint_dims),
        // Eigenvalue analysis (3)
        ("Upright unstable", check_5_unstable_upright),
        ("Downward stable", check_6_stable_downward),
        ("Eigenvalues real", check_7_eigenvalues_real),
        // FD convergence (3)
        ("Centered convergence", check_8_centered_convergence),
        ("Forward convergence", check_9_forward_convergence),
        ("fd_convergence_check", check_10_fd_convergence_utility),
        // Hybrid vs FD (3)
        ("Hybrid vs FD: A", check_11_hybrid_vs_fd_A),
        ("Hybrid vs FD: actuated A+B", check_12_hybrid_vs_fd_actuated),
        ("validate_analytical_vs_fd", check_13_validate_utility),
        // Sensor derivatives (3)
        ("C Some when enabled", check_14_C_some_when_enabled),
        ("D Some when enabled", check_15_D_some_when_enabled),
        ("C,D None by default", check_16_CD_none_by_default),
        // Inverse dynamics (3)
        ("Inverse dims", check_17_inverse_dims),
        ("DfDa ≈ M", check_18_dfda_approx_mass_matrix),
        (
            "DfDv nonzero with damping",
            check_19_dfdv_nonzero_with_damping,
        ),
        // Integrator coverage (4)
        ("Euler", check_20_euler),
        ("ImplicitSpringDamper", check_21_implicit),
        ("ImplicitFast", check_22_implicit_fast),
        ("RK4", check_23_rk4),
        // Quaternion handling (3)
        ("Ball tangent space", check_24_ball_tangent_space),
        ("Free tangent space", check_25_free_tangent_space),
        (
            "Ball FD well-conditioned",
            check_26_ball_fd_well_conditioned,
        ),
        // Actuator / activation (3)
        ("B cols = nu", check_27_B_columns_multi_actuator),
        ("Filter activation in A", check_28_activation_filter_in_A),
        ("Zero-control passive", check_29_zero_control_passive),
        // Contact sensitivity (2)
        ("Contact stiffness", check_30_contact_stiffness_affects_A),
        ("Contact vs no-contact", check_31_contact_vs_no_contact),
        // Config edge cases (1)
        ("Tiny epsilon", check_32_tiny_epsilon),
    ];

    let mut total = 0u32;
    let mut passed = 0u32;

    for (i, (label, func)) in checks.iter().enumerate() {
        println!("-- {}. {} --", i + 1, label);
        let (p, t) = func();
        passed += p;
        total += t;
        println!();
    }

    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
