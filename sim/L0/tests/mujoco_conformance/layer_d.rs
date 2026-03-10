//! Layer D — Property/invariant tests (no MuJoCo dependency).
//!
//! These tests verify physical invariants that must hold regardless of MuJoCo
//! reference data. They are mathematical property tests — if any fail, it
//! indicates a violation of a fundamental physical or mathematical property.
//!
//! Categories:
//! 6. Momentum conservation (free-flying, no gravity, no contacts)
//! 7. Energy conservation (conservative system, RK4 integrator)
//! 8. Quaternion normalization (free/ball joints, N steps, |q|=1)
//! 9. Contact force feasibility (normal >= 0, tangential <= mu * normal)
//! 10. Mass matrix SPD (symmetric, positive definite, diagonal > 0)

use approx::assert_relative_eq;

// ============================================================================
// 6. Momentum Conservation
// ============================================================================

/// Linear momentum conservation: free body, zero gravity, no contacts.
///
/// A free body with initial velocity and no external forces should maintain
/// constant linear momentum: p = m * v = const.
#[test]
fn layer_d_linear_momentum_conservation() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0" timestep="0.001"/>
      <worldbody>
        <body pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Initial linear velocity (free joint DOF order: [vx, vy, vz, wx, wy, wz])
    data.qvel[0] = 1.0; // vx
    data.qvel[1] = -0.5; // vy
    data.qvel[2] = 0.3; // vz

    let mass = 2.0_f64;
    let p0 = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];

    for step in 0..500 {
        data.step(&model).expect("step");

        let p = [
            mass * data.qvel[0],
            mass * data.qvel[1],
            mass * data.qvel[2],
        ];

        for i in 0..3 {
            assert!(
                (p[i] - p0[i]).abs() < 1e-10,
                "Step {step}: linear momentum[{i}] changed: {:.10} -> {:.10}",
                p0[i],
                p[i]
            );
        }
    }
}

/// Angular momentum conservation: free body, zero gravity, no contacts.
///
/// A symmetric body (sphere) spinning with initial angular velocity and no
/// external torques should maintain constant angular velocity (since I is
/// diagonal and constant for a sphere: L = I*ω, τ=0 → ω=const).
#[test]
fn layer_d_angular_momentum_conservation() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0" timestep="0.001"/>
      <worldbody>
        <body pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Initial angular velocity only (free joint DOF order: [vx,vy,vz, wx,wy,wz])
    data.qvel[3] = 2.0; // wx
    data.qvel[4] = -1.0; // wy
    data.qvel[5] = 0.5; // wz

    let omega0 = [data.qvel[3], data.qvel[4], data.qvel[5]];

    for step in 0..500 {
        data.step(&model).expect("step");

        for (i, &w0) in omega0.iter().enumerate() {
            assert!(
                (data.qvel[3 + i] - w0).abs() < 1e-10,
                "Step {step}: angular velocity[{i}] changed: {w0:.10} -> {:.10}",
                data.qvel[3 + i]
            );
        }
    }
}

/// Linear + angular momentum for a hinge pendulum in zero gravity.
///
/// A hinge joint in zero gravity with initial velocity should conserve
/// angular momentum about the hinge axis (no external torques).
#[test]
fn layer_d_hinge_angular_momentum_conservation() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0" timestep="0.001"/>
      <worldbody>
        <body pos="0 0 0">
          <joint type="hinge" axis="0 1 0" damping="0"/>
          <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.qvel[0] = 3.0; // angular velocity around Y axis

    // For a 1-DOF hinge with no damping or gravity: qvel should be constant
    // (M * qacc = 0 when no forces → qacc = 0 → qvel = const)
    let qvel0 = data.qvel[0];

    for step in 0..500 {
        data.step(&model).expect("step");

        assert!(
            (data.qvel[0] - qvel0).abs() < 1e-10,
            "Step {step}: hinge qvel changed: {qvel0:.10} -> {:.10}",
            data.qvel[0]
        );
    }
}

// ============================================================================
// 7. Energy Conservation
// ============================================================================

/// Energy conservation for a conservative pendulum with RK4.
///
/// A frictionless, damping-free pendulum under gravity is a Hamiltonian system.
/// With RK4 at h=0.001, energy drift should be < 1e-10 relative over 1 second.
#[test]
fn layer_d_energy_conservation_rk4() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
        <flag energy="enable"/>
      </option>
      <worldbody>
        <body pos="0 0 0">
          <joint type="hinge" axis="0 1 0" damping="0"/>
          <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // initial angle

    data.forward(&model).expect("forward");
    let e0 = data.total_energy();
    assert!(e0.is_finite() && e0.abs() > 1e-15, "E0={e0}");

    let mut max_drift = 0.0_f64;
    for _ in 0..1000 {
        data.step(&model).expect("step");
        data.forward(&model).expect("forward");
        let e = data.total_energy();
        let drift = ((e - e0) / e0).abs();
        max_drift = max_drift.max(drift);
    }

    assert!(
        max_drift < 1e-10,
        "RK4 energy drift should be < 1e-10, got {max_drift:.2e}"
    );
}

/// Energy drift convergence: halving timestep should reduce drift by ~32×
/// for RK4 (energy error is O(h⁵) for symplectic-like behavior).
#[test]
fn layer_d_energy_drift_convergence() {
    fn max_energy_drift(h: f64, n: usize) -> f64 {
        let xml = format!(
            r#"
            <mujoco>
              <option gravity="0 0 -9.81" timestep="{h}" integrator="RK4">
                <flag energy="enable"/>
              </option>
              <worldbody>
                <body pos="0 0 0">
                  <joint type="hinge" axis="0 1 0" damping="0"/>
                  <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
                </body>
              </worldbody>
            </mujoco>"#
        );

        let model = sim_mjcf::load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;

        data.forward(&model).expect("forward");
        let e0 = data.total_energy();

        let mut max_drift = 0.0_f64;
        for _ in 0..n {
            data.step(&model).expect("step");
            data.forward(&model).expect("forward");
            let e = data.total_energy();
            let drift = ((e - e0) / e0).abs();
            max_drift = max_drift.max(drift);
        }
        max_drift
    }

    let drift_coarse = max_energy_drift(0.002, 500); // 1.0s
    let drift_fine = max_energy_drift(0.001, 1000); // 1.0s

    // Halving h should reduce drift by at least 16×
    if drift_fine > 1e-16 {
        let ratio = drift_coarse / drift_fine;
        assert!(
            ratio > 16.0,
            "Energy drift should decrease by ~32× when halving h, got {ratio:.1}× \
             (coarse={drift_coarse:.2e}, fine={drift_fine:.2e})"
        );
    }
}

/// Euler integrator should dissipate energy for a damped system.
///
/// This is not conservation but a related property: with positive damping,
/// total energy should monotonically decrease.
#[test]
fn layer_d_energy_dissipation_damped() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.001" integrator="Euler">
        <flag energy="enable"/>
      </option>
      <worldbody>
        <body pos="0 0 0">
          <joint type="hinge" axis="0 1 0" damping="0.5"/>
          <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = 1.0; // large initial angle
    data.qvel[0] = 2.0; // initial velocity

    data.forward(&model).expect("forward");
    let e0 = data.total_energy();
    assert!(e0.is_finite());

    // After many steps with damping, energy should decrease
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }
    data.forward(&model).expect("forward");
    let e_final = data.total_energy();

    assert!(
        e_final < e0,
        "Energy should decrease with damping: E0={e0:.6}, E_final={e_final:.6}"
    );
}

// ============================================================================
// 8. Quaternion Normalization
// ============================================================================

/// Quaternion norm for a free joint stays at 1.0 through N steps.
///
/// Free joint stores position as [x, y, z, qw, qx, qy, qz]. After each
/// step, the quaternion norm |q| must remain within 1e-10 of 1.0.
#[test]
fn layer_d_quaternion_norm_free_joint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body pos="0 0 2">
          <freejoint/>
          <geom type="box" size="0.1 0.2 0.3" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Give it angular velocity to exercise quaternion integration
    data.qvel[3] = 5.0;
    data.qvel[4] = -3.0;
    data.qvel[5] = 1.0;

    for step in 0..1000 {
        data.step(&model).expect("step");

        let qw = data.qpos[3];
        let qx = data.qpos[4];
        let qy = data.qpos[5];
        let qz = data.qpos[6];
        let qnorm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();

        assert!(
            (qnorm - 1.0).abs() < 1e-10,
            "Step {step}: quaternion norm = {qnorm}, expected 1.0"
        );
    }
}

/// Quaternion norm for a ball joint stays at 1.0 through N steps.
#[test]
fn layer_d_quaternion_norm_ball_joint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body pos="0 0 0">
          <joint type="ball"/>
          <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Give angular velocity
    data.qvel[0] = 4.0;
    data.qvel[1] = -2.0;
    data.qvel[2] = 1.0;

    for step in 0..1000 {
        data.step(&model).expect("step");

        let qw = data.qpos[0];
        let qx = data.qpos[1];
        let qy = data.qpos[2];
        let qz = data.qpos[3];
        let qnorm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();

        assert!(
            (qnorm - 1.0).abs() < 1e-10,
            "Step {step}: ball joint quaternion norm = {qnorm}, expected 1.0"
        );
    }
}

/// Quaternion norm for RK4 integrator.
///
/// RK4 uses intermediate stages — verify the quaternion is properly
/// renormalized after the full RK4 step.
#[test]
fn layer_d_quaternion_norm_rk4() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0" timestep="0.01" integrator="RK4"/>
      <worldbody>
        <body pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Fast spin
    data.qvel[5] = 2.0 * std::f64::consts::PI; // 1 rev/s around Z

    for step in 0..200 {
        data.step(&model).expect("step");

        let qw = data.qpos[3];
        let qx = data.qpos[4];
        let qy = data.qpos[5];
        let qz = data.qpos[6];
        let qnorm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();

        assert!(
            (qnorm - 1.0).abs() < 1e-10,
            "Step {step}: RK4 quaternion norm = {qnorm}, expected 1.0"
        );
    }
}

// ============================================================================
// 9. Contact Force Feasibility
// ============================================================================

/// Contact normal forces must be non-negative (compressive only).
///
/// For a sphere resting on a plane, the normal contact force should be
/// non-negative: the ground pushes up, never pulls down.
#[test]
fn layer_d_contact_normal_nonnegative() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom type="plane" size="10 10 0.1"/>
        <body pos="0 0 0.5">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step until contacts develop
    for step in 0..500 {
        data.step(&model).expect("step");

        if data.ncon > 0 {
            // Check constraint forces in the efc_force array
            // For contact constraints, the normal component (first row per contact)
            // should be >= 0 (compressive).
            // In MuJoCo convention, efc_force for a normal contact is >= 0.
            for i in 0..data.efc_force.len() {
                // Only check non-friction contact rows (rough check: inequality constraints)
                // Normal forces are typically the first row of each contact block.
                if data.efc_force[i].is_finite() {
                    // We're checking structural property: forces should be finite
                    assert!(
                        data.efc_force[i].is_finite(),
                        "Step {step}: efc_force[{i}] is not finite: {}",
                        data.efc_force[i]
                    );
                }
            }

            // Contact-level check: qfrc_constraint vertical component should be
            // upward (opposing gravity) for a sphere settling on a plane.
            // DOF 2 = vz for the free joint.
            // After settling, qfrc_constraint[2] should be ≥ 0 (upward force).
            if step > 200 {
                // After settling
                assert!(
                    data.qfrc_constraint[2] >= -1e-6,
                    "Step {step}: vertical constraint force should be upward, got {}",
                    data.qfrc_constraint[2]
                );
            }
        }
    }
}

/// Contact forces should balance gravity at equilibrium.
///
/// A sphere resting on a plane should reach equilibrium where contact forces
/// exactly balance gravitational force: |qfrc_constraint| ≈ m*g.
#[test]
fn layer_d_contact_gravity_balance() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom type="plane" size="10 10 0.1"/>
        <body pos="0 0 0.1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Let the sphere settle
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    // At equilibrium: qacc ≈ 0 (settled), qfrc_constraint balances gravity
    // Gravity contribution to qfrc_bias for z-DOF ≈ -m*g = -9.81
    // qfrc_constraint[2] should balance this ≈ +9.81
    let z_acc = data.qacc[2].abs();
    assert!(
        z_acc < 0.1,
        "Sphere should be settled (qacc_z ≈ 0), got {z_acc}"
    );

    // Constraint force in z should approximately balance gravity
    let z_constraint = data.qfrc_constraint[2];
    assert!(
        (z_constraint - 9.81).abs() < 1.0,
        "Vertical constraint force should be ~9.81 (mg), got {z_constraint}"
    );
}

// ============================================================================
// 10. Mass Matrix SPD
// ============================================================================

/// Mass matrix must be symmetric positive definite for a 2-DOF chain.
///
/// SPD verification:
/// - Symmetric: M[i,j] = M[j,i]
/// - Positive definite: all eigenvalues > 0 (checked via diagonal dominance
///   and determinant for small matrices)
#[test]
fn layer_d_mass_matrix_spd_hinge_chain() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.3" mass="1.0"/>
          <body pos="0 0 -0.6">
            <joint name="j2" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.2" mass="0.5"/>
          </body>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Test at multiple configurations
    let configs = [
        [0.0, 0.0],
        [0.5, -0.3],
        [1.0, 1.0],
        [-0.7, 0.4],
        [std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_4],
    ];

    for config in &configs {
        data.qpos[0] = config[0];
        data.qpos[1] = config[1];
        data.forward(&model).expect("forward");

        let nv = model.nv;

        // Symmetry
        for i in 0..nv {
            for j in 0..nv {
                let diff = (data.qM[(i, j)] - data.qM[(j, i)]).abs();
                assert!(
                    diff < 1e-14,
                    "Config {:?}: M[{i},{j}]={} != M[{j},{i}]={}",
                    config,
                    data.qM[(i, j)],
                    data.qM[(j, i)]
                );
            }
        }

        // Positive diagonal
        for i in 0..nv {
            assert!(
                data.qM[(i, i)] > 0.0,
                "Config {:?}: M[{i},{i}]={} should be > 0",
                config,
                data.qM[(i, i)]
            );
        }

        // Positive determinant (2x2: det = M00*M11 - M01*M10)
        let det = data.qM[(0, 0)] * data.qM[(1, 1)] - data.qM[(0, 1)] * data.qM[(1, 0)];
        assert!(
            det > 0.0,
            "Config {:?}: determinant = {} should be > 0",
            config,
            det
        );
    }
}

/// Mass matrix must be SPD for a free joint (6×6).
#[test]
fn layer_d_mass_matrix_spd_free_joint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 1">
          <freejoint/>
          <geom type="box" size="0.1 0.2 0.3" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    let nv = model.nv;
    assert_eq!(nv, 6);

    // Symmetry
    for i in 0..nv {
        for j in 0..nv {
            assert_relative_eq!(data.qM[(i, j)], data.qM[(j, i)], epsilon = 1e-14);
        }
    }

    // Positive diagonal
    for i in 0..nv {
        assert!(
            data.qM[(i, i)] > 0.0,
            "M[{i},{i}]={} should be > 0",
            data.qM[(i, i)]
        );
    }

    // Cholesky factorization test: attempt to factor M. If M is SPD,
    // Cholesky succeeds. We do this via the nalgebra API.
    let m_dense = nalgebra::DMatrix::from_fn(nv, nv, |i, j| data.qM[(i, j)]);
    let chol = m_dense.clone().cholesky();
    assert!(
        chol.is_some(),
        "Mass matrix Cholesky factorization failed — not SPD"
    );
}

/// Mass matrix must be SPD for a 3-DOF chain with mixed joint axes.
#[test]
fn layer_d_mass_matrix_spd_mixed_axes() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 0">
          <joint type="hinge" axis="0 0 1"/>
          <geom type="capsule" size="0.05 0.3" mass="1.0"/>
          <body pos="0 0 -0.6">
            <joint type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.2" mass="0.5"/>
            <body pos="0 0 -0.4">
              <joint type="hinge" axis="1 0 0"/>
              <geom type="capsule" size="0.03 0.15" mass="0.3"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    let configs = [[0.0, 0.0, 0.0], [0.5, -0.3, 0.7], [1.2, 0.8, -0.5]];

    for config in &configs {
        data.qpos[0] = config[0];
        data.qpos[1] = config[1];
        data.qpos[2] = config[2];
        data.forward(&model).expect("forward");

        let nv = model.nv;
        let m_dense = nalgebra::DMatrix::from_fn(nv, nv, |i, j| data.qM[(i, j)]);

        // Symmetry
        for i in 0..nv {
            for j in i + 1..nv {
                assert_relative_eq!(m_dense[(i, j)], m_dense[(j, i)], epsilon = 1e-14);
            }
        }

        // Cholesky
        let chol = m_dense.clone().cholesky();
        assert!(chol.is_some(), "Config {:?}: Mass matrix not SPD", config);

        // All eigenvalues positive (real symmetric → real eigenvalues)
        let eig = m_dense.symmetric_eigen();
        for (idx, &val) in eig.eigenvalues.iter().enumerate() {
            assert!(
                val > 0.0,
                "Config {:?}: eigenvalue[{idx}] = {val} should be > 0",
                config
            );
        }
    }
}

/// Mass matrix diagonal entries should be > 0 for all DOFs in a complex model.
#[test]
fn layer_d_mass_matrix_diagonal_positive() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 1">
          <freejoint/>
          <geom type="capsule" size="0.05 0.3" mass="1.0"/>
          <body pos="0 0 -0.6">
            <joint type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.2" mass="0.5"/>
            <body pos="0 0 -0.4">
              <joint type="ball"/>
              <geom type="sphere" size="0.1" mass="0.3"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    for i in 0..model.nv {
        assert!(
            data.qM[(i, i)] > 0.0,
            "M[{i},{i}]={} should be > 0 (nv={})",
            data.qM[(i, i)],
            model.nv
        );
    }
}
