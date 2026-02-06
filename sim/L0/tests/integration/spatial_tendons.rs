//! Spatial tendon tests — acceptance test 10 (pulley divisor).
//!
//! Validates that `<pulley divisor="D">` elements correctly scale all
//! subsequent path length and Jacobian contributions by `1/D`, matching
//! MuJoCo 3.4.0 reference values.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Model D from spec §4: Pulley with mechanical advantage.
///
/// Two branches separated by `<pulley divisor="2">`:
///   Branch 1 (divisor=1): s1 (0,0,0.5) → s2 (0,0,0)  → length = 0.5
///   Branch 2 (divisor=2): s3 (0,0,0)   → s4 (0,0,-0.5) → length = 0.5/2 = 0.25
///
/// MuJoCo reference (3.4.0): `ten_length = 0.75000000`, `ten_J = [1.0, -0.5]`.
const MODEL_D_PULLEY: &str = r#"
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

/// Test 10: Pulley divisor — length scaling.
///
/// Segments after `<pulley divisor="D">` contribute `length/D`.
/// MuJoCo reference: `ten_length = 0.75` (= 0.5/1 + 0.5/2).
#[test]
fn test_pulley_divisor_length() {
    let model = load_model(MODEL_D_PULLEY).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_relative_eq!(data.ten_length[0], 0.75, epsilon = 1e-10);
}

/// Test 10: Pulley divisor — Jacobian scaling.
///
/// Branch 1 (divisor=1): j1 is a slide along Z, s1 moves with b1.
///   J[j1] = 1.0/1 = 1.0  (s1 moves +Z when j1 increases, increasing distance to s2)
/// Branch 2 (divisor=2): j2 is a slide along Z, s4 moves with b3.
///   J[j2] = -1.0/2 = -0.5  (s4 moves +Z when j2 increases, decreasing distance from s3)
///
/// MuJoCo reference: `ten_J = [1.0, -0.5]`.
#[test]
fn test_pulley_divisor_jacobian() {
    let model = load_model(MODEL_D_PULLEY).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(model.ntendon, 1);
    assert_eq!(model.nv, 2);

    // ten_J[0] is a DVector<f64> of length nv
    assert_relative_eq!(data.ten_J[0][0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(data.ten_J[0][1], -0.5, epsilon = 1e-10);
}

/// Test 10: Pulley divisor — Jacobian correctness via finite difference.
///
/// For each DOF, perturb qpos by ±ε, recompute tendon length, verify
/// `(L(q+ε) - L(q-ε)) / 2ε ≈ ten_J[dof]` within tolerance.
#[test]
fn test_pulley_divisor_jacobian_finite_diff() {
    let model = load_model(MODEL_D_PULLEY).expect("Failed to load Model D");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let eps = 1e-7;
    let j_analytical = data.ten_J[0].clone();

    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];

        // +ε
        let mut data_plus = model.make_data();
        data_plus.qpos[qpos_adr] += eps;
        data_plus.forward(&model).expect("forward failed");
        let l_plus = data_plus.ten_length[0];

        // -ε
        let mut data_minus = model.make_data();
        data_minus.qpos[qpos_adr] -= eps;
        data_minus.forward(&model).expect("forward failed");
        let l_minus = data_minus.ten_length[0];

        let j_fd = (l_plus - l_minus) / (2.0 * eps);
        assert_relative_eq!(j_analytical[dof], j_fd, epsilon = 1e-5,);
    }
}
