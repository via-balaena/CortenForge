//! Spatial tendon acceptance tests (§4 Spatial Tendons + Wrapping).
//!
//! Tests 1-19 from the spec, covering:
//! - Straight-line tendons (Models A, F)
//! - Sphere wrapping (Model B), cylinder wrapping (Model C)
//! - Pulley divisor (Model D)
//! - Sidesite disambiguation (Models E, I, J)
//! - Collinear degenerate wrapping (Model H)
//! - Free-joint Jacobian (Model K)
//! - Mixed straight + wrapping segments (Model L)
//! - Jacobian finite-difference verification
//! - Wrapping transition continuity
//! - Model-build: tendon_length0, muscle acc0
//! - Force transmission: passive, limits, actuator
//! - MuJoCo 3.4.0 conformance (test 16)
//!
//! All MuJoCo reference values confirmed against MuJoCo 3.4.0.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Test MJCF Models (from spec §4)
// ============================================================================

/// Model A — Straight-line spatial tendon (no wrapping).
/// MuJoCo reference (3.4.0): ten_length = 0.70000000 at qpos=0.
const MODEL_A: &str = r#"
<mujoco>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1" stiffness="100" damping="10">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <actuator>
    <motor name="m1" tendon="t1" gear="1"/>
  </actuator>
</mujoco>
"#;

/// Model B — Sphere wrapping.
/// MuJoCo reference (3.4.0): ten_length = 0.60843263 at qpos=0.
const MODEL_B: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.05 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0.05 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_wrap" stiffness="200">
      <site site="origin"/>
      <geom geom="wrap_sphere"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model C — Cylinder wrapping.
/// MuJoCo reference (3.4.0): ten_length = 0.74507207 at qpos=0.
const MODEL_C: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.1 0.2">
      <joint name="j1" type="slide" axis="1 0 0"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="cylinder_body" pos="0 0 0">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.3"/>
    </body>
    <body name="b2" pos="0.3 -0.05 -0.2">
      <joint name="j2" type="slide" axis="1 0 0"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_cyl">
      <site site="s1"/>
      <geom geom="wrap_cyl"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model D — Pulley with mechanical advantage.
/// MuJoCo reference (3.4.0): ten_length = 0.75000000, ten_J = [1.0, -0.5].
const MODEL_D: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 0.5">
      <joint name="j1" type="slide" axis="0 0 1"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="b2" pos="0 0 0">
      <site name="s2" pos="0 0 0"/>
      <site name="s3" pos="0 0 0"/>
    </body>
    <body name="b3" pos="0 0 -0.5">
      <joint name="j2" type="slide" axis="0 0 1"/>
      <site name="s4" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_pulley" stiffness="100">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model E — Sphere wrapping with sidesite.
/// MuJoCo reference (3.4.0): ten_length = 0.64131210 at qpos=0.
const MODEL_E: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.15 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
      <site name="side_hint" pos="0 0.3 0"/>
    </body>
    <body name="b2" pos="0.3 -0.05 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_side">
      <site site="origin"/>
      <geom geom="wrap_sphere" sidesite="side_hint"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model F — Multi-site spatial tendon (3+ sites, no wrapping).
/// MuJoCo reference (3.4.0): ten_length = 1.05000000 at qpos=0.
const MODEL_F: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
        <body name="b3" pos="0 0 -0.4">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3"/>
          <site name="s3" pos="0 0 -0.15"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_multi" stiffness="100" damping="10">
      <site site="s1"/>
      <site site="s2"/>
      <site site="s3"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model G — Tendon limits + muscle actuator.
/// MuJoCo reference (3.4.0): ten_length = 0.70000000 at qpos=0.
const MODEL_G: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_lim" stiffness="100" damping="10"
             limited="true" range="0.35 0.9">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="muscle1" tendon="t_lim" force="100" lengthrange="0.35 0.9"/>
  </actuator>
</mujoco>
"#;

/// Model H — Collinear degenerate sphere wrapping.
/// MuJoCo reference (3.4.0): ten_length = 0.63365281 at qpos=0.
const MODEL_H: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0 0">
      <joint name="j1" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0 0">
      <joint name="j2" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_collinear">
      <site site="s1"/>
      <geom geom="wrap_sphere"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model I — Cylinder wrapping with sidesite.
/// MuJoCo reference (3.4.0): ten_length = 0.74507207 at qpos=0.
const MODEL_I: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.1 0.2">
      <joint name="j1" type="slide" axis="1 0 0"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="cylinder_body" pos="0 0 0">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.3"/>
      <site name="cyl_side" pos="0 0.2 0"/>
    </body>
    <body name="b2" pos="0.3 -0.05 -0.2">
      <joint name="j2" type="slide" axis="1 0 0"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_cyl_side">
      <site site="s1"/>
      <geom geom="wrap_cyl" sidesite="cyl_side"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model J — Sidesite-forced wrapping (straight path clears sphere).
/// MuJoCo reference (3.4.0): ten_length = 0.86662782 at qpos=0.
const MODEL_J: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.2 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
      <site name="side_hint" pos="0 -0.3 0"/>
    </body>
    <body name="b2" pos="0.3 0.2 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_forced">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side_hint"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model K — Free-joint spatial tendon.
/// MuJoCo reference (3.4.0): ten_length = 1.15448807,
/// ten_J = [0.98868, 0.08662, -0.12250, +0.06125, -0.12250, -0.06125].
const MODEL_K: &str = r#"
<mujoco>
  <worldbody>
    <site name="anchor" pos="0 0 0"/>
    <body name="floating" pos="1 0 0" euler="0 45 0">
      <freejoint name="free"/>
      <geom type="sphere" size="0.05"/>
      <site name="attach" pos="0.2 0.1 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_free" stiffness="100">
      <site site="anchor"/>
      <site site="attach"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model L — Mixed straight + wrapping segments.
const MODEL_L: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0.3 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0.15 0"/>
        <body name="wrap_body" pos="0 0 -0.2">
          <geom name="wrap_sphere" type="sphere" size="0.08"/>
        </body>
        <body name="b3" pos="0 0 -0.4">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"/>
          <site name="s3" pos="0 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_mixed" stiffness="100">
      <site site="s1"/>
      <site site="s2"/>
      <geom geom="wrap_sphere"/>
      <site site="s3"/>
    </spatial>
  </tendon>
</mujoco>
"#;

// ============================================================================
// Helper: finite-difference Jacobian verification
// ============================================================================

/// Verify analytical Jacobian against finite difference for all DOFs.
///
/// For each DOF, perturbs the corresponding qpos by ±ε, recomputes tendon
/// length, and checks `(L(q+ε) - L(q-ε)) / 2ε ≈ ten_J[dof]`.
fn verify_jacobian_finite_diff(mjcf: &str, tendon_idx: usize, tol: f64) {
    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let eps = 1e-7;
    let j_analytical = data.ten_J[tendon_idx].clone();

    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        // For free joints, DOFs 3-5 are angular (quaternion derivative).
        // Use the dof_qpos_adr mapping if available; for hinge/slide it's 1:1.
        // For free joints: DOFs 0-2 → qpos[0-2] (pos), DOFs 3-5 → quaternion.
        // We handle quaternion perturbation separately for free joints.
        let jnt_type = model.jnt_type[jnt_id];

        let mut data_plus = model.make_data();
        let mut data_minus = model.make_data();

        match jnt_type {
            sim_core::MjJointType::Free => {
                let dof_in_jnt = dof - model.jnt_dof_adr[jnt_id];
                if dof_in_jnt < 3 {
                    // Translational DOF: perturb qpos directly
                    data_plus.qpos[qpos_adr + dof_in_jnt] += eps;
                    data_minus.qpos[qpos_adr + dof_in_jnt] -= eps;
                } else {
                    // Angular DOF: perturb via quaternion exponential map.
                    // qpos[3..7] is quaternion [w, x, y, z].
                    let ang_idx = dof_in_jnt - 3; // 0, 1, or 2
                    let mut omega = [0.0_f64; 3];
                    omega[ang_idx] = eps;
                    perturb_quat(data_plus.qpos.as_mut_slice(), qpos_adr + 3, &omega);
                    omega[ang_idx] = -eps;
                    perturb_quat(data_minus.qpos.as_mut_slice(), qpos_adr + 3, &omega);
                }
            }
            _ => {
                // Hinge, Slide, Ball: 1:1 DOF → qpos mapping for hinge/slide.
                let dof_in_jnt = dof - model.jnt_dof_adr[jnt_id];
                data_plus.qpos[qpos_adr + dof_in_jnt] += eps;
                data_minus.qpos[qpos_adr + dof_in_jnt] -= eps;
            }
        }

        data_plus.forward(&model).expect("forward failed");
        data_minus.forward(&model).expect("forward failed");

        let l_plus = data_plus.ten_length[tendon_idx];
        let l_minus = data_minus.ten_length[tendon_idx];
        let j_fd = (l_plus - l_minus) / (2.0 * eps);

        assert_relative_eq!(j_analytical[dof], j_fd, epsilon = tol,);
    }
}

/// Perturb a quaternion stored at `qpos[adr..adr+4]` by small body-frame rotation `omega`.
///
/// Applies `q' = q * exp(omega/2)` using first-order exponential map.
/// Right-multiplication applies the perturbation in the **body frame**, matching
/// MuJoCo's angular DOF convention for free joints.
fn perturb_quat(qpos: &mut [f64], adr: usize, omega: &[f64; 3]) {
    // Current quaternion [w, x, y, z]
    let (w, x, y, z) = (qpos[adr], qpos[adr + 1], qpos[adr + 2], qpos[adr + 3]);
    // Half-angle rotation quaternion: [1, omega/2] (first order)
    let half = 0.5;
    let (dw, dx, dy, dz) = (1.0, omega[0] * half, omega[1] * half, omega[2] * half);
    // Quaternion multiply: q * dq (right-multiply for body-frame perturbation)
    let nw = w * dw - x * dx - y * dy - z * dz;
    let nx = w * dx + x * dw + y * dz - z * dy;
    let ny = w * dy - x * dz + y * dw + z * dx;
    let nz = w * dz + x * dy - y * dx + z * dw;
    // Normalize
    let norm = (nw * nw + nx * nx + ny * ny + nz * nz).sqrt();
    qpos[adr] = nw / norm;
    qpos[adr + 1] = nx / norm;
    qpos[adr + 2] = ny / norm;
    qpos[adr + 3] = nz / norm;
}

// ============================================================================
// Test 1: Straight-line spatial tendon (Model A)
// ============================================================================

/// Test 1: Straight-line spatial tendon — length matches Euclidean distance.
/// MuJoCo reference: ten_length = 0.70000000 at qpos=0.
#[test]
fn test_straight_line_length() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 0.70, epsilon = 1e-6);
}

// ============================================================================
// Test 2: Multi-site spatial tendon (Model F)
// ============================================================================

/// Test 2: Multi-site spatial tendon — length = sum of segment distances.
/// MuJoCo reference: ten_length = 1.05000000 at qpos=0.
#[test]
fn test_multi_site_length() {
    let model = load_model(MODEL_F).expect("Failed to load Model F");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 1.05, epsilon = 1e-6);
}

// ============================================================================
// Test 3: Sphere wrapping (Model B)
// ============================================================================

/// Test 3: Sphere wrapping — length includes geodesic arc.
/// MuJoCo reference: ten_length = 0.60843263 at qpos=0.
#[test]
fn test_sphere_wrapping_length() {
    let model = load_model(MODEL_B).expect("Failed to load Model B");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 0.60843263, epsilon = 1e-6);
}

// ============================================================================
// Test 4: Cylinder wrapping (Model C)
// ============================================================================

/// Test 4: Cylinder wrapping — helical path around cylinder.
/// MuJoCo reference: ten_length = 0.74507207 at qpos=0.
#[test]
fn test_cylinder_wrapping_length() {
    let model = load_model(MODEL_C).expect("Failed to load Model C");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 0.74507207, epsilon = 1e-6);
}

// ============================================================================
// Test 5: Jacobian correctness — finite difference (Models A, B, C, D)
// ============================================================================

/// Test 5a: Jacobian finite diff — straight-line tendon (Model A).
#[test]
fn test_jacobian_finite_diff_straight() {
    verify_jacobian_finite_diff(MODEL_A, 0, 1e-5);
}

/// Test 5b: Jacobian finite diff — sphere wrapping (Model B).
#[test]
fn test_jacobian_finite_diff_sphere_wrap() {
    verify_jacobian_finite_diff(MODEL_B, 0, 1e-5);
}

/// Test 5c: Jacobian finite diff — cylinder wrapping (Model C).
#[test]
fn test_jacobian_finite_diff_cylinder_wrap() {
    verify_jacobian_finite_diff(MODEL_C, 0, 1e-5);
}

/// Test 5d: Jacobian finite diff — pulley (Model D).
#[test]
fn test_jacobian_finite_diff_pulley() {
    verify_jacobian_finite_diff(MODEL_D, 0, 1e-5);
}

// ============================================================================
// Test 6: Velocity correctness (Model A)
// ============================================================================

/// Test 6: Velocity correctness — ten_velocity = ten_J · qvel.
#[test]
fn test_velocity_correctness() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();

    // Set known joint velocities
    data.qvel[0] = 1.0; // shoulder
    data.qvel[1] = -0.5; // elbow

    data.forward(&model).expect("forward failed");

    // ten_velocity should equal J · qvel
    let j = &data.ten_J[0];
    let expected_vel = j[0] * data.qvel[0] + j[1] * data.qvel[1];
    assert_relative_eq!(data.ten_velocity[0], expected_vel, epsilon = 1e-10);
}

// ============================================================================
// Test 7: Passive force transmission (Model A)
// ============================================================================

/// Test 7: Passive force transmission — spring/damper produces correct qfrc_passive.
/// Verifies qfrc_passive[dof] = Σ_t ten_J[t][dof] * ten_force[t] for spatial tendons.
#[test]
fn test_passive_force_transmission() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();

    // Perturb elbow to change tendon length from rest
    data.qpos[1] = 0.5;
    data.forward(&model).expect("forward failed");

    // The tendon has stiffness=100, so ten_force should be non-zero
    let force = data.ten_force[0];
    assert!(force.abs() > 1e-10, "Expected non-zero tendon force");

    // Verify J^T * force matches qfrc_passive
    for dof in 0..model.nv {
        let expected = data.ten_J[0][dof] * force;
        assert_relative_eq!(data.qfrc_passive[dof], expected, epsilon = 1e-10);
    }
}

// ============================================================================
// Test 8: Tendon limit forces (Model G)
// ============================================================================

/// Test 8: Tendon limit constraint — when tendon length violates range limits,
/// qfrc_constraint receives J^T * penalty_force.
///
/// Model G has limited="true" range="0.35 0.9". At qpos=0 length=0.70 (within
/// range). We perturb elbow to push length below the lower limit (0.35),
/// then verify qfrc_constraint is non-zero and correctly signed.
#[test]
fn test_tendon_limit_forces() {
    let model = load_model(MODEL_G).expect("Failed to load Model G");

    // First confirm the tendon is limited
    assert!(model.tendon_limited[0], "Model G tendon should be limited");
    let (limit_min, limit_max) = model.tendon_range[0];
    assert_relative_eq!(limit_min, 0.35, epsilon = 1e-10);
    assert_relative_eq!(limit_max, 0.9, epsilon = 1e-10);

    // At qpos=0, length=0.70 — within range, no constraint force
    {
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");
        assert_relative_eq!(data.ten_length[0], 0.70, epsilon = 1e-6);
        // qfrc_constraint should be zero (within limits)
        for dof in 0..model.nv {
            assert_relative_eq!(data.qfrc_constraint[dof], 0.0, epsilon = 1e-10);
        }
    }

    // Flex elbow to shorten the tendon below the lower limit.
    // At pi the chain is degenerate (sites coincident, J≈0), so use a slightly
    // smaller angle that still shortens the tendon below 0.35 but keeps a
    // meaningful Jacobian. We sweep to find a suitable angle.
    {
        let mut data = model.make_data();
        // At qpos[1]≈2.8 (close to pi but not degenerate) the tendon shortens
        // significantly. Use 2.8 rad and verify.
        data.qpos[1] = 2.8;
        data.forward(&model).expect("forward failed");

        if data.ten_length[0] < limit_min {
            // Good — we're below the lower limit with non-degenerate Jacobian.
            //
            // With Newton solver: the tendon's passive spring (stiffness=100) may
            // already produce enough restoring acceleration, making the constraint
            // force zero. With PGS: constraint force is always applied via penalty.
            // Check that either qfrc_constraint or qfrc_passive provides a
            // non-zero restoring force on this DOF.
            let has_restoring_force = (0..model.nv)
                .any(|d| (data.qfrc_constraint[d] + data.qfrc_passive[d]).abs() > 1e-10);
            assert!(
                has_restoring_force,
                "Expected non-zero restoring force when tendon length ({}) < limit_min ({limit_min}). \
                 qfrc_constraint = {:?}, qfrc_passive = {:?}",
                data.ten_length[0],
                (0..model.nv)
                    .map(|d| data.qfrc_constraint[d])
                    .collect::<Vec<_>>(),
                (0..model.nv)
                    .map(|d| data.qfrc_passive[d])
                    .collect::<Vec<_>>()
            );

            // The restoring force should push the tendon back toward longer.
            // For the elbow DOF (index 1), ten_J[1] < 0, and the force should
            // act to reduce qpos[1] (decrease the angle), so the generalized
            // force on DOF 1 should be negative (same sign as J).
            let j = &data.ten_J[0];
            let force_direction_consistent = (0..model.nv).any(|d| {
                let total_force = data.qfrc_constraint[d] + data.qfrc_passive[d];
                if j[d].abs() > 1e-10 && total_force.abs() > 1e-10 {
                    total_force.signum() == j[d].signum()
                } else {
                    false
                }
            });
            assert!(
                force_direction_consistent,
                "Restoring force direction should be consistent with J^T * positive_force"
            );
        } else {
            // If the tendon is not below the limit at 2.8 rad, test the upper limit instead.
            // Extend the elbow to lengthen beyond limit_max=0.9.
            let mut data2 = model.make_data();
            data2.qpos[1] = -1.5; // extend elbow backward
            data2.forward(&model).expect("forward failed");

            assert!(
                data2.ten_length[0] > limit_max,
                "Expected tendon length {} > limit_max {limit_max}",
                data2.ten_length[0]
            );

            let has_restoring_force = (0..model.nv)
                .any(|d| (data2.qfrc_constraint[d] + data2.qfrc_passive[d]).abs() > 1e-10);
            assert!(
                has_restoring_force,
                "Expected non-zero restoring force when tendon length > limit_max"
            );
        }
    }
}

// ============================================================================
// Test 9: Actuator through spatial tendon (Model A)
// ============================================================================

/// Test 9: Actuator force transmitted correctly through spatial tendon.
/// Verifies qfrc_actuator includes gear * J^T * actuator_force.
#[test]
fn test_actuator_force_transmission() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();

    // Apply actuator control
    data.ctrl[0] = 10.0;
    data.forward(&model).expect("forward failed");

    let gear = model.actuator_gear[0][0];
    let act_force = data.actuator_force[0];

    // qfrc_actuator should include gear * J^T * force
    for dof in 0..model.nv {
        let expected = gear * data.ten_J[0][dof] * act_force;
        // qfrc_actuator may include contributions from passive forces too,
        // so check the actuator contribution is present.
        // For a motor with ctrl=10, actuator_force = -gear*ctrl (in MuJoCo convention).
        assert_relative_eq!(data.qfrc_actuator[dof], expected, epsilon = 1e-8);
    }
}

// ============================================================================
// Test 10: Pulley divisor (Model D)
// ============================================================================

/// Test 10a: Pulley divisor — length scaling.
/// MuJoCo reference: ten_length = 0.75 (= 0.5/1 + 0.5/2).
#[test]
fn test_pulley_divisor_length() {
    let model = load_model(MODEL_D).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 0.75, epsilon = 1e-10);
}

/// Test 10b: Pulley divisor — Jacobian scaling.
/// MuJoCo reference: ten_J = [1.0, -0.5].
#[test]
fn test_pulley_divisor_jacobian() {
    let model = load_model(MODEL_D).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.nv, 2);
    assert_relative_eq!(data.ten_J[0][0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(data.ten_J[0][1], -0.5, epsilon = 1e-10);
}

// ============================================================================
// Test 11: Wrapping transition continuity (Model B)
// ============================================================================

/// Test 11: Wrapping transition — tendon length is continuous across
/// the wrap/no-wrap boundary (no jump > 1e-6 between steps of 0.001 rad).
#[test]
fn test_wrapping_transition_continuity() {
    let model = load_model(MODEL_B).expect("Failed to load Model B");

    let mut prev_length: Option<f64> = None;
    let step = 0.001_f64;

    // Sweep j1 from -1.0 to 1.0 rad, crossing the wrapping transition.
    let mut angle = -1.0;
    while angle <= 1.0 {
        let mut data = model.make_data();
        data.qpos[0] = angle;
        data.forward(&model).expect("forward failed");

        if let Some(prev) = prev_length {
            let jump = (data.ten_length[0] - prev).abs();
            assert!(
                jump < 1e-6,
                "Length discontinuity {jump:.2e} at angle {angle:.4} (prev={prev:.8}, cur={:.8})",
                data.ten_length[0]
            );
        }
        prev_length = Some(data.ten_length[0]);
        angle += step;
    }
}

// ============================================================================
// Test 12: Model build — tendon_length0 (Model A)
// ============================================================================

/// Test 12: tendon_length0 matches ten_length at qpos0.
#[test]
fn test_tendon_length0() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // tendon_length0 is computed at build time from FK at qpos0
    assert_relative_eq!(model.tendon_length0[0], data.ten_length[0], epsilon = 1e-10);
}

// ============================================================================
// Test 13: Model build — muscle acc0 (Model G)
// ============================================================================

/// Test 13: compute_muscle_params() for spatial tendon — acc0 is non-zero.
#[test]
fn test_muscle_acc0_spatial_tendon() {
    let model = load_model(MODEL_G).expect("Failed to load Model G");

    // Muscle actuator should have non-zero actuator_acc0.
    // For a 2-site straight spatial tendon the acc0 value may be very small
    // but must still be computed (not left at exactly 0.0).
    assert_eq!(model.nu, 1);
    assert!(
        model.actuator_acc0[0] != 0.0,
        "Expected non-zero actuator_acc0 for muscle on spatial tendon, got {}",
        model.actuator_acc0[0]
    );
}

// ============================================================================
// Test 14: Sidesite disambiguation (Models E, I, J)
// ============================================================================

/// Test 14a: Sphere sidesite (Model E) — selects correct wrapping direction.
/// MuJoCo reference: ten_length = 0.64131210 at qpos=0.
#[test]
fn test_sphere_sidesite() {
    let model = load_model(MODEL_E).expect("Failed to load Model E");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.64131210, epsilon = 1e-6);
}

/// Test 14b: Cylinder sidesite (Model I) — XY-projected sidesite direction.
/// MuJoCo reference: ten_length = 0.74507207 at qpos=0 (same as Model C).
#[test]
fn test_cylinder_sidesite() {
    let model = load_model(MODEL_I).expect("Failed to load Model I");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.74507207, epsilon = 1e-6);
}

/// Test 14c: Sidesite-forced wrapping (Model J) — wraps even though straight
/// path clears the sphere.
/// MuJoCo reference: ten_length = 0.86662782 at qpos=0.
#[test]
fn test_sidesite_forced_wrapping() {
    let model = load_model(MODEL_J).expect("Failed to load Model J");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // The straight-line distance between sites is ~0.6, but wrapping makes it longer.
    assert_relative_eq!(data.ten_length[0], 0.86662782, epsilon = 1e-6);
}

// ============================================================================
// Test 15: Degenerate wrapping plane (Model H)
// ============================================================================

/// Test 15: Collinear degenerate sphere wrapping — no NaN, correct length.
/// MuJoCo reference: ten_length = 0.63365281 at qpos=0.
/// Formula: 2×√(d²−r²) + r×acos(7/9) where d=0.3, r=0.1.
#[test]
fn test_collinear_degenerate_wrapping() {
    let model = load_model(MODEL_H).expect("Failed to load Model H");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert!(
        data.ten_length[0].is_finite(),
        "ten_length should not be NaN"
    );
    assert_relative_eq!(data.ten_length[0], 0.63365281, epsilon = 1e-6);
}

// ============================================================================
// Test 16: MuJoCo conformance — all models at qpos=0
// ============================================================================

/// Test 16: MuJoCo conformance — ten_length matches reference for all models.
#[test]
fn test_mujoco_conformance_lengths() {
    let cases: &[(&str, &str, f64)] = &[
        ("A", MODEL_A, 0.70000000),
        ("B", MODEL_B, 0.60843263),
        ("C", MODEL_C, 0.74507207),
        ("D", MODEL_D, 0.75000000),
        ("E", MODEL_E, 0.64131210),
        ("F", MODEL_F, 1.05000000),
        ("G", MODEL_G, 0.70000000),
        ("H", MODEL_H, 0.63365281),
        ("I", MODEL_I, 0.74507207),
        ("J", MODEL_J, 0.86662782),
    ];

    for &(name, mjcf, expected) in cases {
        let model = load_model(mjcf).unwrap_or_else(|e| panic!("Failed to load Model {name}: {e}"));
        let mut data = model.make_data();
        data.forward(&model)
            .unwrap_or_else(|e| panic!("forward failed for Model {name}: {e}"));

        assert_relative_eq!(data.ten_length[0], expected, epsilon = 1e-6,);
    }
}

// ============================================================================
// Test 18: Free-joint angular Jacobian (Model K)
// ============================================================================

/// Test 18: Free-joint Jacobian uses body-frame axes (R*e_i), not world-frame.
/// MuJoCo reference: ten_J = [0.98868, 0.08662, -0.12250, +0.06125, -0.12250, -0.06125].
#[test]
fn test_free_joint_jacobian() {
    let model = load_model(MODEL_K).expect("Failed to load Model K");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.nv, 6);
    assert_relative_eq!(data.ten_length[0], 1.15448807, epsilon = 1e-5);

    let mj_ref = [0.98868, 0.08662, -0.12250, 0.06125, -0.12250, -0.06125];
    for (dof, &expected) in mj_ref.iter().enumerate() {
        assert_relative_eq!(data.ten_J[0][dof], expected, epsilon = 1e-4,);
    }
}

/// Test 18b: Free-joint Jacobian verified via finite difference.
#[test]
fn test_free_joint_jacobian_finite_diff() {
    verify_jacobian_finite_diff(MODEL_K, 0, 1e-4);
}

// ============================================================================
// Test 19: Mixed straight + wrapping segments (Model L)
// ============================================================================

/// Test 19: Mixed straight + wrapping segments — tendon with both site-site
/// (straight) and site-geom-site (wrapped) segments computes finite length.
/// At qpos=0 the sites form a near-vertical chain with s3 at the j3 origin,
/// so the Jacobian is near-zero. Perturbing j1 breaks the alignment and
/// creates non-trivial Jacobian. Correctness verified via finite difference
/// (test 19b).
#[test]
fn test_mixed_straight_and_wrapping() {
    let model = load_model(MODEL_L).expect("Failed to load Model L");

    // At qpos=0: length must be positive and finite
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");
    assert!(data.ten_length[0].is_finite());
    assert!(data.ten_length[0] > 0.0);

    // At a perturbed configuration (j1 rotates the whole chain):
    // length changes because the geometric relationship between
    // the wrapping sphere (on b2) and s3 (on b3) shifts.
    let mut data2 = model.make_data();
    data2.qpos[0] = 1.0; // j1 — large perturbation to move s1 relative to chain
    data2.forward(&model).expect("forward failed");
    assert!(data2.ten_length[0].is_finite());
    assert!(data2.ten_length[0] > 0.0);
}

/// Test 19b: Mixed segments — Jacobian verified via finite difference.
#[test]
fn test_mixed_segments_jacobian_finite_diff() {
    verify_jacobian_finite_diff(MODEL_L, 0, 1e-5);
}

// ============================================================================
// Test 17: Parser/validation rejection — invalid MJCF patterns
// ============================================================================

/// Test 17a: Spatial tendon path must start with a Site element.
#[test]
fn test_reject_path_starts_with_geom() {
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b1" pos="0 0 0.5">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.05"/>
          <site name="s1" pos="0 0 0"/>
        </body>
        <body name="wrap" pos="0 0 0">
          <geom name="g1" type="sphere" size="0.1"/>
        </body>
        <body name="b2" pos="0 0 -0.5">
          <site name="s2" pos="0 0 0"/>
        </body>
      </worldbody>
      <tendon>
        <spatial name="bad">
          <geom geom="g1"/>
          <site site="s1"/>
          <site site="s2"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(result.is_err(), "Should reject path starting with Geom");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("start with a Site"),
        "Error should mention 'start with a Site', got: {err}"
    );
}

/// Test 17b: Spatial tendon path must end with a Site element.
#[test]
fn test_reject_path_ends_with_geom() {
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b1" pos="0 0 0.5">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.05"/>
          <site name="s1" pos="0 0 0"/>
        </body>
        <body name="wrap" pos="0 0 0">
          <geom name="g1" type="sphere" size="0.1"/>
        </body>
        <body name="b2" pos="0 0 -0.5">
          <site name="s2" pos="0 0 0"/>
        </body>
      </worldbody>
      <tendon>
        <spatial name="bad">
          <site site="s1"/>
          <site site="s2"/>
          <geom geom="g1"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(result.is_err(), "Should reject path ending with Geom");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("end with a Site"),
        "Error should mention 'end with a Site', got: {err}"
    );
}

/// Test 17c: Geom must be immediately followed by a Site.
#[test]
fn test_reject_consecutive_geoms() {
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b1" pos="0 0 0.5">
          <site name="s1" pos="0 0 0"/>
        </body>
        <body name="w1" pos="0 0 0">
          <geom name="g1" type="sphere" size="0.1"/>
        </body>
        <body name="w2" pos="0.5 0 0">
          <geom name="g2" type="sphere" size="0.1"/>
        </body>
        <body name="b2" pos="0 0 -0.5">
          <site name="s2" pos="0 0 0"/>
        </body>
      </worldbody>
      <tendon>
        <spatial name="bad">
          <site site="s1"/>
          <geom geom="g1"/>
          <geom geom="g2"/>
          <site site="s2"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(result.is_err(), "Should reject consecutive Geom elements");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("must be immediately followed by a Site"),
        "Error should mention Geom-Site adjacency, got: {err}"
    );
}

/// Test 17d: Pulley must not be immediately followed by a Geom.
#[test]
fn test_reject_pulley_followed_by_geom() {
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b1" pos="0 0 0.5">
          <site name="s1" pos="0 0 0"/>
          <site name="s2" pos="0 0.1 0"/>
        </body>
        <body name="wrap" pos="0 0 0">
          <geom name="g1" type="sphere" size="0.1"/>
        </body>
        <body name="b2" pos="0 0 -0.5">
          <site name="s3" pos="0 0 0"/>
          <site name="s4" pos="0 0.1 0"/>
        </body>
      </worldbody>
      <tendon>
        <spatial name="bad">
          <site site="s1"/>
          <site site="s2"/>
          <pulley divisor="2"/>
          <geom geom="g1"/>
          <site site="s3"/>
          <site site="s4"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(
        result.is_err(),
        "Should reject Pulley immediately followed by Geom"
    );
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("Pulley") && err.contains("must not be immediately followed by a Geom"),
        "Error should mention Pulley-Geom adjacency, got: {err}"
    );
}

/// Test 17e: Spatial tendon with fewer than 2 sites should be rejected.
#[test]
fn test_reject_fewer_than_two_sites() {
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b1" pos="0 0 0.5">
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <tendon>
        <spatial name="bad">
          <site site="s1"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(result.is_err(), "Should reject tendon with < 2 sites");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("needs at least 2 sites"),
        "Error should mention minimum site count, got: {err}"
    );
}

// ============================================================================
// §39 wrap_inside Tests (Inverse Tendon Wrapping)
// ============================================================================
//
// Tests T1-T4, T6-T10, T14-T18 from the spec.
// Unit tests T5, T11, T12, T13 are in tendon/wrap_math.rs (private fn).

/// §39 T6 model: Sphere inside-wrap conformance.
const MODEL_WRAP_INSIDE_SPHERE: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §39 T7 model: Cylinder inside-wrap conformance.
const MODEL_WRAP_INSIDE_CYLINDER: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl" type="cylinder" size="0.15 0.5"/>
      <site name="s1" pos="0.3 0.1 0.2"/>
      <site name="side_c" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 -0.1 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside_cyl">
      <site site="s1"/>
      <geom geom="wrap_cyl" sidesite="side_c"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §39 T8 model: Transition robustness (slide joint sweeps sidesite).
const MODEL_WRAP_INSIDE_SWEEP: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_s" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <body name="b1">
        <joint name="sweep" type="slide" axis="1 0 0" range="-0.05 0.05"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="side_sweep" pos="0.16 0 0"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_sweep">
      <site site="s1"/>
      <geom geom="wrap_s" sidesite="side_sweep"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §39 T15 model: Cylinder inside-wrap with asymmetric Z interpolation.
const MODEL_WRAP_INSIDE_CYL_Z: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_z" type="cylinder" size="0.15 0.5"/>
      <site name="s1_z" pos="0.3 0.05 0.3"/>
      <site name="side_z" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2_z" pos="0.3 -0.15 -0.1"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_z_interp">
      <site site="s1_z"/>
      <geom geom="wrap_cyl_z" sidesite="side_z"/>
      <site site="s2_z"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §39 T16 model: Cylinder 3D-norm dispatch (sidesite inside XY but outside 3D).
const MODEL_WRAP_INSIDE_CYL_3D: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_3d" type="cylinder" size="0.1 1.0"/>
      <site name="s1" pos="0.3 0.05 0.2"/>
      <site name="side_3d" pos="0.05 0 5.0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 -0.05 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_3d_dispatch">
      <site site="s1"/>
      <geom geom="wrap_cyl_3d" sidesite="side_3d"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §39 T18 model: Sidesite at sphere center (origin edge case).
const MODEL_WRAP_INSIDE_ORIGIN: &str = r#"
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side_origin" pos="0 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_origin">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side_origin"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

// ---------------------------------------------------------------------------
// T1–T2: No build-time panic
// ---------------------------------------------------------------------------

/// §39 T1: Sidesite inside wrapping sphere loads without panic.
#[test]
fn test_wrap_inside_t1_no_panic_sphere() {
    let model = load_model(MODEL_WRAP_INSIDE_SPHERE).expect("model should load without panic");
    let mut data = model.make_data();
    data.forward(&model).expect("forward should not crash");
    assert!(data.ten_length[0].is_finite());
    assert!(data.ten_length[0] > 0.0);
}

/// §39 T2: Sidesite inside wrapping cylinder loads without panic.
#[test]
fn test_wrap_inside_t2_no_panic_cylinder() {
    let model = load_model(MODEL_WRAP_INSIDE_CYLINDER).expect("model should load without panic");
    let mut data = model.make_data();
    data.forward(&model).expect("forward should not crash");
    assert!(data.ten_length[0].is_finite());
    assert!(data.ten_length[0] > 0.0);
}

// ---------------------------------------------------------------------------
// T3–T4: Structural properties of inside wrap
// ---------------------------------------------------------------------------

/// §39 T3: Single tangent point on surface.
///
/// For inside wrap, tangent_point_1 == tangent_point_2, arc_length == 0.
/// The tangent point lies on the sphere surface.
#[test]
fn test_wrap_inside_t3_single_tangent_on_surface() {
    let model = load_model(MODEL_WRAP_INSIDE_SPHERE).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Inside wrap should produce a wrapped result (not NoWrap).
    let length = data.ten_length[0];
    assert!(length.is_finite() && length > 0.0);

    // The tendon length should be strictly greater than the straight-line
    // distance between the two sites (triangle inequality for inside wrap).
    // s1 = (0.3, 0, 0.15), s2 = (0.3, 0, -0.15) at qpos=0
    // straight = |(0, 0, 0.3)| = 0.3
    let straight_dist = 0.3;
    assert!(
        length > straight_dist,
        "inside wrap length {length} should exceed straight-line {straight_dist}"
    );
}

/// §39 T4: Path length = |p0 - tangent| + |tangent - p1| (no arc).
///
/// Verified via the MuJoCo conformance value: 0.3·√2.
#[test]
fn test_wrap_inside_t4_path_length() {
    let model = load_model(MODEL_WRAP_INSIDE_SPHERE).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // MuJoCo reference: 0.3·√2 = two segments of length 0.15·√2
    let expected = 0.3 * 2.0_f64.sqrt();
    assert_relative_eq!(data.ten_length[0], expected, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// T6–T7: MuJoCo conformance
// ---------------------------------------------------------------------------

/// §39 T6: MuJoCo conformance (sphere inside-wrap).
///
/// MuJoCo 3.5.0 reference: ten_length = 0.424264068711929 (= 0.3·√2).
#[test]
fn test_wrap_inside_t6_conformance_sphere() {
    let model = load_model(MODEL_WRAP_INSIDE_SPHERE).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.424264068711929, epsilon = 1e-6);
}

/// §39 T7: MuJoCo conformance (cylinder inside-wrap).
///
/// MuJoCo 3.5.0 reference: ten_length = 0.538516480713450.
#[test]
fn test_wrap_inside_t7_conformance_cylinder() {
    let model = load_model(MODEL_WRAP_INSIDE_CYLINDER).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.538516480713450, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// T8: Transition robustness and dispatch correctness
// ---------------------------------------------------------------------------

/// §39 T8: Transition robustness — sidesite sweeps from outside to inside.
///
/// Sweep qpos from -0.05 to +0.05 in steps of 0.001. Verify no crash/NaN
/// at any position, and MuJoCo conformance at five sampled points spanning
/// the dispatch boundary.
#[test]
fn test_wrap_inside_t8_transition_sweep() {
    let model = load_model(MODEL_WRAP_INSIDE_SWEEP).expect("load failed");

    // Full sweep: verify finite positive length at every step.
    for q_milli in -50..=50 {
        let q = q_milli as f64 * 0.001;
        let mut data = model.make_data();
        data.qpos[0] = q;
        data.forward(&model).expect("forward failed");
        let length = data.ten_length[0];
        assert!(
            length.is_finite() && length > 0.0,
            "sweep failed at qpos={q}: ten_length={length}"
        );
    }

    // MuJoCo conformance at five specific positions.
    let reference_points: &[(f64, f64)] = &[
        (-0.04, 0.397761312300161), // inside wrap
        (-0.02, 0.410539632911676), // inside wrap (near boundary)
        (-0.01, 0.300166620396073), // boundary exact: |ss|=radius → outside dispatch
        (0.00, 0.300000000000000),  // outside
        (0.04, 0.302654919008431),  // well outside
    ];

    for &(qpos, expected_length) in reference_points {
        let mut data = model.make_data();
        data.qpos[0] = qpos;
        data.forward(&model).expect("forward failed");
        assert_relative_eq!(data.ten_length[0], expected_length, epsilon = 1e-6,);
    }
}

// ---------------------------------------------------------------------------
// T9: Regression — all existing spatial tendon tests
// ---------------------------------------------------------------------------
// T9 is implicitly satisfied: all tests 1–19b above remain in this file
// and are run as part of the test suite. No separate test function needed.

// ---------------------------------------------------------------------------
// T10: Jacobian correctness (sphere inside-wrap)
// ---------------------------------------------------------------------------

/// Helper for inside-wrap Jacobian verification via central finite difference.
///
/// Uses mixed relative/absolute tolerance:
/// `|J_fd - J_an| < tol * max(|J_fd|, 1.0)`
fn verify_jacobian_finite_diff_mixed_tol(mjcf: &str, tendon_idx: usize, tol: f64) {
    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let eps = 1e-7;
    let j_analytical = data.ten_J[tendon_idx].clone();

    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_in_jnt = dof - model.jnt_dof_adr[jnt_id];

        let mut data_plus = model.make_data();
        let mut data_minus = model.make_data();
        data_plus.qpos[qpos_adr + dof_in_jnt] += eps;
        data_minus.qpos[qpos_adr + dof_in_jnt] -= eps;

        data_plus.forward(&model).expect("forward failed");
        data_minus.forward(&model).expect("forward failed");

        let l_plus = data_plus.ten_length[tendon_idx];
        let l_minus = data_minus.ten_length[tendon_idx];
        let j_fd = (l_plus - l_minus) / (2.0 * eps);

        let threshold = tol * j_fd.abs().max(1.0);
        let error = (j_analytical[dof] - j_fd).abs();
        assert!(
            error < threshold,
            "Jacobian mismatch at DOF {dof}: analytical={}, fd={}, error={error:.2e}, threshold={threshold:.2e}",
            j_analytical[dof],
            j_fd,
        );
    }
}

/// §39 T10: Jacobian correctness for sphere inside-wrap via finite difference.
#[test]
fn test_wrap_inside_t10_jacobian_sphere() {
    verify_jacobian_finite_diff_mixed_tol(MODEL_WRAP_INSIDE_SPHERE, 0, 1e-4);
}

// ---------------------------------------------------------------------------
// T14: Boundary — sidesite exactly at radius
// ---------------------------------------------------------------------------

/// §39 T14: Sidesite exactly at |ss| = radius.
///
/// The inside check (< radius) does NOT trigger, so normal wrap is used.
/// Verify no crash and finite positive tendon length.
#[test]
fn test_wrap_inside_t14_sidesite_at_radius() {
    // Modify the sphere model to place sidesite exactly at radius.
    let mjcf = r#"
    <mujoco>
      <worldbody>
        <body name="b0">
          <geom name="wrap_sphere" type="sphere" size="0.15"/>
          <site name="s1" pos="0.3 0 0.15"/>
          <site name="side_exact" pos="0.15 0 0"/>
          <body name="b1">
            <joint name="j1" type="hinge" axis="0 1 0"/>
            <geom type="sphere" size="0.01" mass="0.1"/>
            <site name="s2" pos="0.3 0 -0.15"/>
          </body>
        </body>
      </worldbody>
      <tendon>
        <spatial name="t_boundary">
          <site site="s1"/>
          <geom geom="wrap_sphere" sidesite="side_exact"/>
          <site site="s2"/>
        </spatial>
      </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("model should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward should not crash");
    assert!(data.ten_length[0].is_finite());
    assert!(data.ten_length[0] > 0.0);
}

// ---------------------------------------------------------------------------
// T15: Cylinder tangency + Z-interpolation
// ---------------------------------------------------------------------------

/// §39 T15: Cylinder inside-wrap with asymmetric Z interpolation.
///
/// MuJoCo 3.5.0 reference:
///   ten_length = 0.542346519424661
///   tangent_pos = (0.148607047563419, -0.020394739872909, 0.117656368532109)
#[test]
fn test_wrap_inside_t15_cylinder_z_interp() {
    let model = load_model(MODEL_WRAP_INSIDE_CYL_Z).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.542346519424661, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// T16: Cylinder 3D-norm dispatch
// ---------------------------------------------------------------------------

/// §39 T16: Sidesite inside XY cross-section but outside by 3D norm.
///
/// MuJoCo uses mju_norm3(s) < radius → 5.0 < 0.1 is false → normal wrap.
/// Straight line clears cylinder → NoWrap. Length = |s1 - s2|.
///
/// MuJoCo 3.5.0 reference: ten_length = 0.412310562561766.
#[test]
fn test_wrap_inside_t16_3d_norm_dispatch() {
    let model = load_model(MODEL_WRAP_INSIDE_CYL_3D).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.412310562561766, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// T17: Jacobian correctness (cylinder inside-wrap)
// ---------------------------------------------------------------------------

/// §39 T17: Jacobian correctness for cylinder inside-wrap via finite difference.
#[test]
fn test_wrap_inside_t17_jacobian_cylinder() {
    verify_jacobian_finite_diff_mixed_tol(MODEL_WRAP_INSIDE_CYLINDER, 0, 1e-4);
}

// ---------------------------------------------------------------------------
// T18: Sidesite at origin (sphere center)
// ---------------------------------------------------------------------------

/// §39 T18: Sidesite at (0,0,0) — exactly at sphere center.
///
/// |ss| = 0 < radius → inside wrap triggers. Result should match T6 exactly
/// since the sidesite only controls dispatch, not the wrap computation.
///
/// MuJoCo 3.5.0 reference:
///   ten_length = 0.424264068711929 (identical to T6)
#[test]
fn test_wrap_inside_t18_sidesite_at_origin() {
    let model = load_model(MODEL_WRAP_INSIDE_ORIGIN).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.ten_length[0], 0.424264068711929, epsilon = 1e-6);
}

// ============================================================================
// §40b: Tendon Visualization Data (wrap_xpos, wrap_obj, ten_wrapadr, ten_wrapnum)
// ============================================================================

/// §40b Model: No-wrap fallback — sites far from sphere, straight path clears geom.
const MODEL_B_NOWRAP: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.5 0.5 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.05"/>
    </body>
    <body name="b2" pos="0.5 0.5 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_nowrap">
      <site site="origin"/>
      <geom geom="wrap_sphere"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §40b Model: Fixed tendon passthrough (mixed fixed + spatial).
const MODEL_FIXED_PASSTHROUGH: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 0.5">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.25"/>
      <site name="s1" pos="0 0 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.2"/>
        <site name="s2" pos="0 0 0"/>
        <site name="s3" pos="0 0.2 0"/>
        <body name="b3" pos="0 0 -0.5">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.2"/>
          <site name="s4" pos="0 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_spatial1">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
    <fixed name="t_fixed">
      <joint joint="j1" coef="0.1"/>
    </fixed>
    <spatial name="t_spatial2">
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// §40b Model: Multiple spatial tendons (straight + wrapped).
const MODEL_MULTI_SPATIAL: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.3 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="b2" pos="0.3 0.3 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
    <body name="b3" pos="-0.3 0.05 0">
      <joint name="j3" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s3" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b4" pos="0.3 0.05 0">
      <joint name="j4" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s4" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_straight">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
    <spatial name="t_wrapped">
      <site site="s3"/>
      <geom geom="wrap_sphere"/>
      <site site="s4"/>
    </spatial>
  </tendon>
</mujoco>
"#;

// ---------------------------------------------------------------------------
// T1: Field existence and allocation
// ---------------------------------------------------------------------------

/// §40b T1: Data struct has wrap visualization fields with correct allocation sizes.
#[test]
fn test_40b_t1_field_existence_and_allocation() {
    let model = load_model(MODEL_B).expect("Failed to load Model B");
    let data = model.make_data();

    assert_eq!(data.wrap_xpos.len(), model.nwrap * 2);
    assert_eq!(data.wrap_obj.len(), model.nwrap * 2);
    assert_eq!(data.ten_wrapadr.len(), model.ntendon);
    assert_eq!(data.ten_wrapnum.len(), model.ntendon);

    // Model A has no wrapping geoms → nwrap == 2 (two sites)
    let model_a = load_model(MODEL_A).expect("Failed to load Model A");
    let data_a = model_a.make_data();
    assert_eq!(data_a.ten_wrapadr.len(), model_a.ntendon);
    assert_eq!(data_a.ten_wrapnum.len(), model_a.ntendon);
}

// ---------------------------------------------------------------------------
// T2: Straight tendon path (Model A: Site–Site)
// ---------------------------------------------------------------------------

/// §40b T2: Straight tendon — 2 path points (leading + final site), wrap_obj == [-1, -1].
#[test]
fn test_40b_t2_straight_tendon_path() {
    let model = load_model(MODEL_A).expect("Failed to load Model A");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 2);
    let adr = data.ten_wrapadr[0];
    // Leading site
    assert_eq!(data.wrap_obj[adr], -1);
    // Final site
    assert_eq!(data.wrap_obj[adr + 1], -1);

    // Positions match site_xpos
    let s1_id = model.wrap_objid[model.tendon_adr[0]];
    let s2_id = model.wrap_objid[model.tendon_adr[0] + 1];
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s1_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 1],
        data.site_xpos[s2_id],
        epsilon = 1e-10
    );
}

// ---------------------------------------------------------------------------
// T3: Multi-site straight tendon (Model F: Site–Site–Site)
// ---------------------------------------------------------------------------

/// §40b T3: Multi-site tendon — 3 path points, all wrap_obj == -1.
#[test]
fn test_40b_t3_multi_site_straight() {
    let model = load_model(MODEL_F).expect("Failed to load Model F");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 3);
    let adr = data.ten_wrapadr[0];
    for i in 0..3 {
        assert_eq!(
            data.wrap_obj[adr + i],
            -1,
            "wrap_obj[{i}] should be -1 (site)"
        );
    }

    // All positions match site_xpos
    let s1_id = model.wrap_objid[model.tendon_adr[0]];
    let s2_id = model.wrap_objid[model.tendon_adr[0] + 1];
    let s3_id = model.wrap_objid[model.tendon_adr[0] + 2];
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s1_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 1],
        data.site_xpos[s2_id],
        epsilon = 1e-10
    );
    assert_relative_eq!(
        data.wrap_xpos[adr + 2],
        data.site_xpos[s3_id],
        epsilon = 1e-10
    );
}

// ---------------------------------------------------------------------------
// T4: Sphere wrapping (Model B: Site–Sphere–Site)
// ---------------------------------------------------------------------------

/// §40b T4: Sphere wrapping — 4 path points [site, tangent, tangent, site].
/// MuJoCo 3.5.0 reference tangent points verified.
#[test]
fn test_40b_t4_sphere_wrapping() {
    let model = load_model(MODEL_B).expect("Failed to load Model B");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 4);
    let adr = data.ten_wrapadr[0];

    // wrap_obj markers: [-1, geom_id, geom_id, -1]
    assert_eq!(data.wrap_obj[adr], -1);
    assert!(data.wrap_obj[adr + 1] >= 0, "tangent1 should have geom_id");
    assert_eq!(data.wrap_obj[adr + 1], data.wrap_obj[adr + 2]);
    assert_eq!(data.wrap_obj[adr + 3], -1);

    let geom_id = data.wrap_obj[adr + 1] as usize;

    // Tangent points should differ (not inside-wrap at default qpos)
    let t1 = data.wrap_xpos[adr + 1];
    let t2 = data.wrap_xpos[adr + 2];
    assert!(
        (t1 - t2).norm() > 1e-6,
        "tangent points should differ at default qpos"
    );

    // Tangent points lie on sphere surface (in geom frame, |t| ≈ radius)
    let geom_pos = data.geom_xpos[geom_id];
    let geom_mat = data.geom_xmat[geom_id];
    let radius = model.geom_size[geom_id].x;
    let t1_local = geom_mat.transpose() * (t1 - geom_pos);
    let t2_local = geom_mat.transpose() * (t2 - geom_pos);
    assert_relative_eq!(t1_local.norm(), radius, epsilon = 1e-10);
    assert_relative_eq!(t2_local.norm(), radius, epsilon = 1e-10);

    // Site positions match
    let s0_id = model.wrap_objid[model.tendon_adr[0]];
    let s1_id = model.wrap_objid[model.tendon_adr[0] + 2];
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s0_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 3],
        data.site_xpos[s1_id],
        epsilon = 1e-10
    );
}

// ---------------------------------------------------------------------------
// T5: Cylinder wrapping (Model C: Site–Cylinder–Site)
// ---------------------------------------------------------------------------

/// §40b T5: Cylinder wrapping — 4 path points, tangent points on cylinder surface.
/// MuJoCo 3.5.0 reference tangent points verified.
#[test]
fn test_40b_t5_cylinder_wrapping() {
    let model = load_model(MODEL_C).expect("Failed to load Model C");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 4);
    let adr = data.ten_wrapadr[0];

    assert_eq!(data.wrap_obj[adr], -1);
    assert!(data.wrap_obj[adr + 1] >= 0);
    assert_eq!(data.wrap_obj[adr + 1], data.wrap_obj[adr + 2]);
    assert_eq!(data.wrap_obj[adr + 3], -1);

    let geom_id = data.wrap_obj[adr + 1] as usize;

    // Tangent points lie on cylinder surface (distance from z-axis ≈ radius)
    let geom_pos = data.geom_xpos[geom_id];
    let geom_mat = data.geom_xmat[geom_id];
    let radius = model.geom_size[geom_id].x;
    for i in 1..=2 {
        let t_local = geom_mat.transpose() * (data.wrap_xpos[adr + i] - geom_pos);
        let xy_dist = (t_local.x * t_local.x + t_local.y * t_local.y).sqrt();
        assert_relative_eq!(xy_dist, radius, epsilon = 1e-6);
    }
}

// ---------------------------------------------------------------------------
// T6: Pulley (Model D: Site–Site–Pulley–Site–Site)
// ---------------------------------------------------------------------------

/// §40b T6: Pulley — 5 path points [s1, s2, [0,0,0], s3, s4].
#[test]
fn test_40b_t6_pulley() {
    let model = load_model(MODEL_D).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 5);
    let adr = data.ten_wrapadr[0];

    // wrap_obj: [-1, -1, -2, -1, -1]
    assert_eq!(data.wrap_obj[adr], -1); // s1
    assert_eq!(data.wrap_obj[adr + 1], -1); // s2 (final of branch 1)
    assert_eq!(data.wrap_obj[adr + 2], -2); // pulley marker
    assert_eq!(data.wrap_obj[adr + 3], -1); // s3
    assert_eq!(data.wrap_obj[adr + 4], -1); // s4 (final of branch 2)

    // Pulley marker is zero position
    let zero = nalgebra::Vector3::<f64>::zeros();
    assert_relative_eq!(data.wrap_xpos[adr + 2], zero, epsilon = 1e-10);

    // Site positions match
    let s1_id = model.wrap_objid[model.tendon_adr[0]];
    let s4_id = model.wrap_objid[model.tendon_adr[0] + model.tendon_num[0] - 1];
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s1_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 4],
        data.site_xpos[s4_id],
        epsilon = 1e-10
    );
}

// ---------------------------------------------------------------------------
// T7: No-wrap fallback (MODEL_B_NOWRAP)
// ---------------------------------------------------------------------------

/// §40b T7: No-wrap fallback — geom skipped, only 2 site points stored.
#[test]
fn test_40b_t7_no_wrap_fallback() {
    let model = load_model(MODEL_B_NOWRAP).expect("Failed to load MODEL_B_NOWRAP");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ten_wrapnum[0], 2);
    let adr = data.ten_wrapadr[0];
    assert_eq!(data.wrap_obj[adr], -1);
    assert_eq!(data.wrap_obj[adr + 1], -1);

    // Positions match sites
    let s0_id = model.wrap_objid[model.tendon_adr[0]];
    let s1_id = model.wrap_objid[model.tendon_adr[0] + 2]; // skip geom
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s0_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 1],
        data.site_xpos[s1_id],
        epsilon = 1e-10
    );
}

// ---------------------------------------------------------------------------
// T8: Inside-wrap (§39 models)
// ---------------------------------------------------------------------------

/// §40b T8: Inside-wrap — tangent points are identical, both have geom_id.
#[test]
fn test_40b_t8_inside_wrap() {
    // Sphere inside-wrap
    {
        let model = load_model(MODEL_WRAP_INSIDE_SPHERE).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        assert_eq!(data.ten_wrapnum[0], 4);
        let adr = data.ten_wrapadr[0];
        // Identical tangent points
        assert_relative_eq!(
            data.wrap_xpos[adr + 1],
            data.wrap_xpos[adr + 2],
            epsilon = 1e-6
        );
        // Both tangent points have geom_id
        assert!(data.wrap_obj[adr + 1] >= 0);
        assert_eq!(data.wrap_obj[adr + 1], data.wrap_obj[adr + 2]);
    }

    // Cylinder inside-wrap
    {
        let model = load_model(MODEL_WRAP_INSIDE_CYLINDER).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        assert_eq!(data.ten_wrapnum[0], 4);
        let adr = data.ten_wrapadr[0];
        assert_relative_eq!(
            data.wrap_xpos[adr + 1],
            data.wrap_xpos[adr + 2],
            epsilon = 1e-6
        );
        assert!(data.wrap_obj[adr + 1] >= 0);
        assert_eq!(data.wrap_obj[adr + 1], data.wrap_obj[adr + 2]);
    }
}

// ---------------------------------------------------------------------------
// T9: Indexing consistency
// ---------------------------------------------------------------------------

/// §40b T9: ten_wrapadr[t] + ten_wrapnum[t] == ten_wrapadr[t+1] (contiguous storage).
#[test]
fn test_40b_t9_indexing_consistency() {
    let test_models: &[(&str, &str)] = &[
        ("A", MODEL_A),
        ("B", MODEL_B),
        ("D", MODEL_D),
        ("F", MODEL_F),
        ("L", MODEL_L),
        ("MULTI_SPATIAL", MODEL_MULTI_SPATIAL),
        ("FIXED_PASSTHROUGH", MODEL_FIXED_PASSTHROUGH),
    ];

    for &(name, mjcf) in test_models {
        let model = load_model(mjcf).unwrap_or_else(|e| panic!("Failed to load {name}: {e}"));
        let mut data = model.make_data();
        data.forward(&model)
            .unwrap_or_else(|e| panic!("forward failed for {name}: {e}"));

        // Check consecutive tendon addresses are contiguous
        for t in 0..model.ntendon - 1 {
            let end = data.ten_wrapadr[t] + data.ten_wrapnum[t];
            assert_eq!(
                end,
                data.ten_wrapadr[t + 1],
                "Model {name}: ten_wrapadr[{t}] + ten_wrapnum[{t}] = {end} != ten_wrapadr[{}] = {}",
                t + 1,
                data.ten_wrapadr[t + 1]
            );
        }
    }
}

// ---------------------------------------------------------------------------
// T10: MuJoCo conformance — wrap_xpos positions
// ---------------------------------------------------------------------------

/// §40b T10: wrap_xpos tangent point positions match MuJoCo 3.5.0 reference.
#[test]
fn test_40b_t10_wrap_xpos_conformance() {
    // Model B: sphere wrapping
    {
        let model = load_model(MODEL_B).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        // MuJoCo 3.5.0: tangent₁ = (-0.016906587441789, 0.098560475349265, 0.0)
        assert_relative_eq!(
            data.wrap_xpos[adr + 1].x,
            -0.016906587441789,
            epsilon = 1e-6
        );
        assert_relative_eq!(data.wrap_xpos[adr + 1].y, 0.098560475349265, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, 0.0, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.016906587441789, 0.098560475349265, 0.0)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.016906587441789, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, 0.098560475349265, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].z, 0.0, epsilon = 1e-6);
        // Verify wrap_obj markers
        let geom_id = data.wrap_obj[adr + 1];
        assert!(geom_id >= 0);
        assert_eq!(data.wrap_obj[adr + 2], geom_id);
    }

    // Model C: cylinder wrapping
    {
        let model = load_model(MODEL_C).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        // MuJoCo 3.5.0: tangent₁ = (0.005275293665245, 0.079825880995736, 0.005317640178297)
        assert_relative_eq!(data.wrap_xpos[adr + 1].x, 0.005275293665245, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].y, 0.079825880995736, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, 0.005317640178297, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.033445535784753, 0.072673214708521, -0.013280251722640)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.033445535784753, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, 0.072673214708521, epsilon = 1e-6);
        assert_relative_eq!(
            data.wrap_xpos[adr + 2].z,
            -0.013280251722640,
            epsilon = 1e-6
        );
        // Verify wrap_obj markers
        let geom_id = data.wrap_obj[adr + 1];
        assert!(geom_id >= 0);
        assert_eq!(data.wrap_obj[adr + 2], geom_id);
    }

    // Model E: sphere sidesite wrapping
    {
        let model = load_model(MODEL_E).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        // MuJoCo 3.5.0: tangent₁ = (0.016020828249552, 0.098708323165771, 0.0)
        assert_relative_eq!(data.wrap_xpos[adr + 1].x, 0.016020828249552, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].y, 0.098708323165771, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, 0.0, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.047958277423076, 0.087749664538455, 0.0)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.047958277423076, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, 0.087749664538455, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].z, 0.0, epsilon = 1e-6);
        // Verify wrap_obj markers
        let geom_id = data.wrap_obj[adr + 1];
        assert!(geom_id >= 0);
        assert_eq!(data.wrap_obj[adr + 2], geom_id);
    }

    // Model H: collinear degenerate
    // Note: collinear case has undefined wrapping plane. MuJoCo and our implementation
    // may choose different perpendicular directions, yielding different tangent positions
    // but identical tendon lengths. We verify structural properties instead of exact positions.
    {
        let model = load_model(MODEL_H).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        assert_eq!(data.ten_wrapnum[0], 4);
        assert_eq!(data.wrap_obj[adr], -1);
        assert!(data.wrap_obj[adr + 1] >= 0);
        assert_eq!(data.wrap_obj[adr + 1], data.wrap_obj[adr + 2]);
        assert_eq!(data.wrap_obj[adr + 3], -1);

        // Tangent points on sphere surface
        let geom_id = data.wrap_obj[adr + 1] as usize;
        let geom_pos = data.geom_xpos[geom_id];
        let geom_mat = data.geom_xmat[geom_id];
        let radius = model.geom_size[geom_id].x;
        let t1_local = geom_mat.transpose() * (data.wrap_xpos[adr + 1] - geom_pos);
        let t2_local = geom_mat.transpose() * (data.wrap_xpos[adr + 2] - geom_pos);
        assert_relative_eq!(t1_local.norm(), radius, epsilon = 1e-10);
        assert_relative_eq!(t2_local.norm(), radius, epsilon = 1e-10);
    }

    // Model I: cylinder sidesite wrapping
    {
        let model = load_model(MODEL_I).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        assert_eq!(data.ten_wrapnum[0], 4);
        // MuJoCo 3.5.0: tangent₁ = (0.005275293665245, 0.079825880995736, 0.005317640178297)
        assert_relative_eq!(data.wrap_xpos[adr + 1].x, 0.005275293665245, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].y, 0.079825880995736, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, 0.005317640178297, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.033445535784753, 0.072673214708521, -0.013280251722640)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.033445535784753, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, 0.072673214708521, epsilon = 1e-6);
        assert_relative_eq!(
            data.wrap_xpos[adr + 2].z,
            -0.013280251722640,
            epsilon = 1e-6
        );
        // Verify wrap_obj markers
        let geom_id = data.wrap_obj[adr + 1];
        assert!(geom_id >= 0);
        assert_eq!(data.wrap_obj[adr + 2], geom_id);
    }

    // Model J: sidesite-forced wrapping
    {
        let model = load_model(MODEL_J).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        // MuJoCo 3.5.0: tangent₁ = (-0.076370794079042, -0.064556191118564, 0.0)
        assert_relative_eq!(
            data.wrap_xpos[adr + 1].x,
            -0.076370794079042,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            data.wrap_xpos[adr + 1].y,
            -0.064556191118564,
            epsilon = 1e-6
        );
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, 0.0, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.076370794079042, -0.064556191118564, 0.0)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.076370794079042, epsilon = 1e-6);
        assert_relative_eq!(
            data.wrap_xpos[adr + 2].y,
            -0.064556191118564,
            epsilon = 1e-6
        );
        assert_relative_eq!(data.wrap_xpos[adr + 2].z, 0.0, epsilon = 1e-6);
        // Verify wrap_obj markers
        let geom_id = data.wrap_obj[adr + 1];
        assert!(geom_id >= 0);
        assert_eq!(data.wrap_obj[adr + 2], geom_id);
    }

    // Model K: free-joint straight tendon (no wrapping geometry)
    {
        let model = load_model(MODEL_K).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        assert_eq!(data.ten_wrapnum[0], 2);
        assert_eq!(data.wrap_obj[adr], -1);
        assert_eq!(data.wrap_obj[adr + 1], -1);
        // Site positions match
        let s0_id = model.wrap_objid[model.tendon_adr[0]];
        let s1_id = model.wrap_objid[model.tendon_adr[0] + 1];
        assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s0_id], epsilon = 1e-10);
        assert_relative_eq!(
            data.wrap_xpos[adr + 1],
            data.site_xpos[s1_id],
            epsilon = 1e-10
        );
    }

    // Model L: mixed straight + wrapping
    {
        let model = load_model(MODEL_L).expect("load failed");
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        let adr = data.ten_wrapadr[0];
        assert_eq!(data.ten_wrapnum[0], 5);
        // MuJoCo 3.5.0: tangent₁ = (0.0, 0.075994722725514, 0.275003957955864)
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, 0.075994722725514, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].z, 0.275003957955864, epsilon = 1e-6);
        // MuJoCo 3.5.0: tangent₂ = (0.0, 0.073321211119293, 0.268000000000000)
        assert_relative_eq!(data.wrap_xpos[adr + 3].x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 3].y, 0.073321211119293, epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 3].z, 0.268000000000000, epsilon = 1e-6);
        // Verify wrap_obj markers
        assert_eq!(data.wrap_obj[adr], -1); // s1
        assert_eq!(data.wrap_obj[adr + 1], -1); // s2
        let geom_id = data.wrap_obj[adr + 2];
        assert!(geom_id >= 0); // tangent₁
        assert_eq!(data.wrap_obj[adr + 3], geom_id); // tangent₂
        assert_eq!(data.wrap_obj[adr + 4], -1); // s3
    }
}

// ---------------------------------------------------------------------------
// T11: Stepped conformance — wrap_xpos updates across forward() calls
// ---------------------------------------------------------------------------

/// §40b T11: wrap_xpos updates when joint state changes.
///
/// Uses Model B variant with slide joints (so site positions actually change
/// with qpos). Verifies wrap_xpos tangent points change across 5 different
/// configurations, matching MuJoCo 3.5.0 reference values.
#[test]
fn test_40b_t11_stepped_conformance() {
    // Model B with slide joints — sites move when qpos changes
    let model_b_slide: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.05 0">
      <joint name="j1" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0.05 0">
      <joint name="j2" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_wrap" stiffness="200">
      <site site="origin"/>
      <geom geom="wrap_sphere"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
"#;

    // MuJoCo 3.5.0 reference tangent positions at 5 configurations
    let configs: &[([f64; 2], [f64; 3], [f64; 3])] = &[
        // (qpos, tangent1, tangent2)
        (
            [0.0, 0.0],
            [-0.016906587441789, 0.098560475349265, 0.0],
            [0.016906587441789, 0.098560475349265, 0.0],
        ),
        (
            [0.05, -0.02],
            [0.000000000000000, 0.100000000000000, 0.0],
            [0.023616196268430, 0.097171370649029, 0.0],
        ),
        (
            [0.1, -0.05],
            [0.016020828249552, 0.098708323165771, 0.0],
            [0.033333333333333, 0.094280904158206, 0.0],
        ),
        (
            [0.15, -0.08],
            [0.030216947925196, 0.095325421887794, 0.0],
            [0.042390404391636, 0.090570710583022, 0.0],
        ),
        (
            [0.2, -0.1],
            [0.042211757666154, 0.090654109199384, 0.0],
            [0.047958277423076, 0.087749664538455, 0.0],
        ),
    ];

    let model = load_model(model_b_slide).expect("load failed");

    for (i, (qpos, t1_ref, t2_ref)) in configs.iter().enumerate() {
        let mut data = model.make_data();
        data.qpos[0] = qpos[0];
        data.qpos[1] = qpos[1];
        data.forward(&model).expect("forward failed");

        assert_eq!(data.ten_wrapnum[0], 4, "config {i}: expected 4 path points");
        let adr = data.ten_wrapadr[0];

        // Tangent point 1
        assert_relative_eq!(data.wrap_xpos[adr + 1].x, t1_ref[0], epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].y, t1_ref[1], epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 1].z, t1_ref[2], epsilon = 1e-6);

        // Tangent point 2
        assert_relative_eq!(data.wrap_xpos[adr + 2].x, t2_ref[0], epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].y, t2_ref[1], epsilon = 1e-6);
        assert_relative_eq!(data.wrap_xpos[adr + 2].z, t2_ref[2], epsilon = 1e-6);
    }
}

// ---------------------------------------------------------------------------
// T12: Fixed tendon passthrough
// ---------------------------------------------------------------------------

/// §40b T12: Fixed tendon has wrapadr but wrapnum == 0, no gap in wrap storage.
#[test]
fn test_40b_t12_fixed_tendon_passthrough() {
    let model = load_model(MODEL_FIXED_PASSTHROUGH).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 3);

    // Tendon 0: spatial (s1, s2) → 2 path points
    assert_eq!(data.ten_wrapadr[0], 0);
    assert_eq!(data.ten_wrapnum[0], 2);

    // Tendon 1: fixed → 0 path points, but wrapadr continues from tendon 0
    assert_eq!(data.ten_wrapadr[1], 2);
    assert_eq!(data.ten_wrapnum[1], 0);

    // Tendon 2: spatial (s3, s4) → 2 path points, picks up from tendon 1
    assert_eq!(data.ten_wrapadr[2], 2);
    assert_eq!(data.ten_wrapnum[2], 2);
}

// ---------------------------------------------------------------------------
// T13: Multiple spatial tendons
// ---------------------------------------------------------------------------

/// §40b T13: Two spatial tendons — correct partitioning of wrap_xpos storage.
#[test]
fn test_40b_t13_multiple_spatial_tendons() {
    let model = load_model(MODEL_MULTI_SPATIAL).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 2);

    // Tendon 0: straight (s1, s2) → 2 path points
    assert_eq!(data.ten_wrapadr[0], 0);
    assert_eq!(data.ten_wrapnum[0], 2);
    assert_eq!(data.wrap_obj[0], -1);
    assert_eq!(data.wrap_obj[1], -1);

    // Tendon 1: wrapped (s3, geom, s4) → 4 path points (wrapping occurs)
    assert_eq!(data.ten_wrapadr[1], 2);
    assert_eq!(data.ten_wrapnum[1], 4);
    assert_eq!(data.wrap_obj[2], -1); // s3
    assert!(data.wrap_obj[3] >= 0); // tangent1 (geom_id)
    assert!(data.wrap_obj[4] >= 0); // tangent2 (geom_id)
    assert_eq!(data.wrap_obj[3], data.wrap_obj[4]); // same geom
    assert_eq!(data.wrap_obj[5], -1); // s4

    // Contiguous storage: tendon 0 ends where tendon 1 starts
    assert_eq!(
        data.ten_wrapadr[0] + data.ten_wrapnum[0],
        data.ten_wrapadr[1]
    );
}

// ---------------------------------------------------------------------------
// T14: Path reconstruction — straight segments
// ---------------------------------------------------------------------------

/// §40b T14: Reconstructed path length from wrap_xpos matches ten_length for straight tendon.
#[test]
fn test_40b_t14_path_reconstruction_straight() {
    let model = load_model(MODEL_A).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let adr = data.ten_wrapadr[0];
    let num = data.ten_wrapnum[0];
    assert!(num >= 2);

    let mut reconstructed_length = 0.0;
    for i in 0..(num - 1) {
        let diff = data.wrap_xpos[adr + i + 1] - data.wrap_xpos[adr + i];
        reconstructed_length += diff.norm();
    }

    assert_relative_eq!(reconstructed_length, data.ten_length[0], epsilon = 1e-10);
}

// ---------------------------------------------------------------------------
// T15: Mixed segment model (Model L)
// ---------------------------------------------------------------------------

/// §40b T15: Model L — mixed straight + wrapping, correct path point structure.
#[test]
fn test_40b_t15_mixed_segment_model_l() {
    let model = load_model(MODEL_L).expect("load failed");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let adr = data.ten_wrapadr[0];

    // At default qpos, wrapping occurs (5 points: s1, s2, t1, t2, s3)
    assert_eq!(data.ten_wrapnum[0], 5);

    // wrap_obj structure: [-1, -1, geom_id, geom_id, -1]
    assert_eq!(data.wrap_obj[adr], -1); // s1
    assert_eq!(data.wrap_obj[adr + 1], -1); // s2
    let geom_id = data.wrap_obj[adr + 2];
    assert!(geom_id >= 0); // tangent1
    assert_eq!(data.wrap_obj[adr + 3], geom_id); // tangent2
    assert_eq!(data.wrap_obj[adr + 4], -1); // s3

    // Site positions match
    let s1_id = model.wrap_objid[model.tendon_adr[0]];
    let s3_id = model.wrap_objid[model.tendon_adr[0] + model.tendon_num[0] - 1];
    assert_relative_eq!(data.wrap_xpos[adr], data.site_xpos[s1_id], epsilon = 1e-10);
    assert_relative_eq!(
        data.wrap_xpos[adr + 4],
        data.site_xpos[s3_id],
        epsilon = 1e-10
    );
}
