//! RK4 Integration Tests.
//!
//! Validates the true 4-stage Runge-Kutta integrator against acceptance criteria
//! from future_work_1 §8:
//!
//! - **AC #2**: Order of convergence (O(h⁴) for RK4 vs O(h) for symplectic Euler)
//! - **AC #3**: Euler/RK4 consistency at small timestep
//! - **AC #4**: Energy conservation with quantitative bound
//! - **AC #5**: Contact handling (falling ball with ground contact)
//! - **AC #6**: Quaternion correctness (free-body rotation)
//! - **AC #9**: Sensor non-corruption
//! - **AC #10**: Warmstart efc_lambda preservation

use approx::assert_relative_eq;
use sim_core::Integrator;
use sim_mjcf::load_model;
use std::f64::consts::PI;

// ============================================================================
// Helper: build a simple pendulum MJCF string
// ============================================================================

/// Builds MJCF for a frictionless pendulum (hinge on Y axis, point mass at tip).
/// Gravity pulls straight down (-Z). No damping, no springs — pure conservative.
/// Uses sphere geom with pos offset for proper center-of-mass torque.
fn pendulum_mjcf(timestep: f64, integrator: &str) -> String {
    format!(
        r#"
        <mujoco model="pendulum">
            <option gravity="0 0 -9.81" timestep="{timestep}" integrator="{integrator}"/>
            <worldbody>
                <body name="link" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0" damping="0"/>
                    <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#
    )
}

/// Runs a pendulum simulation for `n_steps` and returns the final qpos[0] (angle).
fn run_pendulum(timestep: f64, integrator: &str, initial_angle: f64, n_steps: usize) -> f64 {
    let mjcf = pendulum_mjcf(timestep, integrator);
    let model = load_model(&mjcf).expect("Failed to load pendulum model");
    let mut data = model.make_data();
    data.qpos[0] = initial_angle;
    data.qvel[0] = 0.0;

    for _ in 0..n_steps {
        data.step(&model).expect("step failed");
    }
    data.qpos[0]
}

// ============================================================================
// AC #2: Order of convergence
// ============================================================================

/// Verify RK4 is O(h⁴) and symplectic Euler is lower order on a pendulum.
///
/// For a smooth Hamiltonian system (frictionless pendulum), halving h should:
/// - Reduce RK4 error by ~16× (2⁴)
/// - Reduce symplectic Euler error by ~2×–4×
///
/// We use a small initial angle (0.1 rad) to stay in the near-linear regime
/// where the order-of-convergence is cleanest.
#[test]
fn test_rk4_order_of_convergence() {
    let initial_angle = 0.1; // small angle for clean convergence
    let t_final = 1.0; // simulate 1 second

    // Reference: very fine timestep RK4 as "ground truth"
    let h_ref: f64 = 1e-5;
    let n_ref = (t_final / h_ref).round() as usize;
    let q_ref = run_pendulum(h_ref, "RK4", initial_angle, n_ref);

    // RK4 at h=0.01 and h=0.005
    let h1 = 0.01;
    let h2 = 0.005;
    let n1 = (t_final / h1).round() as usize;
    let n2 = (t_final / h2).round() as usize;
    let q_rk4_h1 = run_pendulum(h1, "RK4", initial_angle, n1);
    let q_rk4_h2 = run_pendulum(h2, "RK4", initial_angle, n2);

    let err_rk4_h1 = (q_rk4_h1 - q_ref).abs();
    let err_rk4_h2 = (q_rk4_h2 - q_ref).abs();

    // RK4 convergence ratio should be ~16 (2⁴ = 16 for O(h⁴))
    let rk4_ratio = err_rk4_h1 / err_rk4_h2;
    assert!(
        rk4_ratio > 10.0 && rk4_ratio < 25.0,
        "RK4 convergence ratio should be ~16 (O(h⁴)), got {rk4_ratio:.2} \
         (err_h1={err_rk4_h1:.2e}, err_h2={err_rk4_h2:.2e})"
    );

    // Euler at h=0.01 and h=0.005
    let q_euler_h1 = run_pendulum(h1, "Euler", initial_angle, n1);
    let q_euler_h2 = run_pendulum(h2, "Euler", initial_angle, n2);

    let err_euler_h1 = (q_euler_h1 - q_ref).abs();
    let err_euler_h2 = (q_euler_h2 - q_ref).abs();

    // Symplectic Euler: O(h) in general, O(h²) on separable Hamiltonian systems.
    // In practice the convergence ratio is between ~2 (O(h)) and ~4 (O(h²))
    // depending on the observable and system structure.
    let euler_ratio = err_euler_h1 / err_euler_h2;
    assert!(
        euler_ratio > 1.5 && euler_ratio < 6.0,
        "Symplectic Euler convergence ratio should be ~2–4, got {euler_ratio:.2} \
         (err_h1={err_euler_h1:.2e}, err_h2={err_euler_h2:.2e})"
    );

    // AC #1: RK4 must not produce identical results to Euler
    assert!(
        (q_rk4_h1 - q_euler_h1).abs() > 1e-10,
        "RK4 and Euler must produce different results"
    );
}

// ============================================================================
// AC #3: Euler/RK4 consistency at small h
// ============================================================================

/// At very small timestep, RK4 and Euler should converge to the same trajectory.
#[test]
fn test_rk4_euler_consistency_small_h() {
    let initial_angle = 0.1;
    let h = 1e-5;
    let n_steps = 10;

    let q_rk4 = run_pendulum(h, "RK4", initial_angle, n_steps);
    let q_euler = run_pendulum(h, "Euler", initial_angle, n_steps);

    let diff = (q_rk4 - q_euler).abs();
    assert!(
        diff < 1e-8,
        "RK4 and Euler should converge at small h: diff={diff:.2e}"
    );
}

// ============================================================================
// AC #4: Energy conservation
// ============================================================================

/// RK4 energy drift should be < 1e-10 relative to initial energy at h=0.001.
/// Additionally, halving h should reduce drift by ~32×–64× (O(h⁵)–O(h⁶)).
#[test]
fn test_rk4_energy_conservation() {
    let initial_angle = 0.3; // moderate angle

    // --- Test 1: Absolute energy drift at h=0.001 ---
    let h_fine = 0.001;
    let n_fine = 1000; // 1.0 s simulated time

    let mjcf_fine = pendulum_mjcf(h_fine, "RK4");
    let model_fine = load_model(&mjcf_fine).expect("Failed to load pendulum model");
    let mut data_fine = model_fine.make_data();
    data_fine.qpos[0] = initial_angle;
    data_fine.qvel[0] = 0.0;

    // Get initial energy using total_energy() (requires forward() to populate)
    data_fine.forward(&model_fine).expect("forward failed");
    let e0_fine = data_fine.total_energy();
    assert!(
        e0_fine.is_finite() && e0_fine.abs() > 1e-15,
        "Initial energy should be nonzero, got {e0_fine}"
    );

    let mut max_drift_fine = 0.0_f64;
    for _ in 0..n_fine {
        data_fine.step(&model_fine).expect("step failed");
        // After RK4 step(), derived quantities are stale from stage 3.
        // Need a fresh forward() for accurate energy.
        data_fine.forward(&model_fine).expect("forward failed");
        let e = data_fine.total_energy();
        let drift = ((e - e0_fine) / e0_fine).abs();
        max_drift_fine = max_drift_fine.max(drift);
    }

    assert!(
        max_drift_fine < 1e-10,
        "RK4 energy drift at h={h_fine} should be < 1e-10, got {max_drift_fine:.2e}"
    );

    // --- Test 2: Order-of-convergence of energy drift ---
    let h_coarse = 0.002;
    let n_coarse = 500; // same simulated time (1.0 s)

    let mjcf_coarse = pendulum_mjcf(h_coarse, "RK4");
    let model_coarse = load_model(&mjcf_coarse).expect("Failed to load model");
    let mut data_coarse = model_coarse.make_data();
    data_coarse.qpos[0] = initial_angle;
    data_coarse.qvel[0] = 0.0;

    data_coarse.forward(&model_coarse).expect("forward failed");
    let e0_coarse = data_coarse.total_energy();

    let mut max_drift_coarse = 0.0_f64;
    for _ in 0..n_coarse {
        data_coarse.step(&model_coarse).expect("step failed");
        data_coarse.forward(&model_coarse).expect("forward failed");
        let e = data_coarse.total_energy();
        let drift = ((e - e0_coarse) / e0_coarse).abs();
        max_drift_coarse = max_drift_coarse.max(drift);
    }

    // Halving h should reduce energy drift by ~32×–64× (O(h⁵)–O(h⁶))
    if max_drift_fine > 1e-16 {
        let drift_ratio = max_drift_coarse / max_drift_fine;
        assert!(
            drift_ratio > 16.0,
            "Energy drift should decrease by ~32×–64× when halving h, got {drift_ratio:.1}× \
             (drift_coarse={max_drift_coarse:.2e}, drift_fine={max_drift_fine:.2e})"
        );
    }
}

// ============================================================================
// AC #5: Contact handling (ball + ground plane)
// ============================================================================

/// Ball falling under gravity with RK4: ball should reach ground, make contact,
/// and not tunnel or explode. Energy should remain bounded.
///
/// Note: With constraint-based contact (no restitution coefficient), the ball
/// does not bounce — it settles at the ground. This is correct behavior for the
/// PGS contact solver without restitution.
#[test]
fn test_rk4_contact_handling() {
    let mjcf = r#"
        <mujoco model="contact_ball">
            <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
            <worldbody>
                <geom name="ground" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 2">
                    <freejoint name="ball_joint"/>
                    <geom name="ball_geom" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load contact ball model");
    assert_eq!(model.integrator, Integrator::RungeKutta4);
    let mut data = model.make_data();

    let initial_z = data.qpos[2];
    assert!(initial_z > 1.5, "Initial z should be ~2.0, got {initial_z}");

    // Step for 2 seconds (2000 steps at h=0.001)
    let mut max_z = initial_z;
    let mut had_contact = false;

    for step in 0..2000 {
        data.step(&model).expect("step failed");
        let z = data.qpos[2];
        max_z = max_z.max(z);

        if data.ncon > 0 {
            had_contact = true;
        }

        // Ball should not explode
        assert!(
            z.is_finite() && z.abs() < 100.0,
            "Ball position should be finite and bounded, got z={z} at step {step}"
        );

        // Ball should not tunnel through ground (sphere radius = 0.1)
        assert!(
            z > -0.5,
            "Ball should not tunnel through ground, got z={z} at step {step}"
        );
    }

    // Ball should have reached the ground and made contact
    assert!(had_contact, "Ball should have made contact with ground");

    // Final position should be near ground (sphere radius = 0.1, so z ≈ 0.1)
    let final_z = data.qpos[2];
    assert!(
        final_z < 0.5,
        "Ball should have settled near ground, got z={final_z}"
    );

    // Ball should not have gained energy (max height ≤ initial + small tolerance)
    assert!(
        max_z <= initial_z + 0.01,
        "Ball should not gain energy: max_z={max_z}, initial_z={initial_z}"
    );
}

// ============================================================================
// AC #6: Quaternion correctness (free-body rotation)
// ============================================================================

/// Free-body with known angular velocity: verify final orientation matches
/// analytic quaternion exponential, and quaternion norm stays near 1.0.
#[test]
fn test_rk4_quaternion_free_body() {
    // Free body in zero gravity, spinning around Z axis
    let mjcf = r#"
        <mujoco model="spinning_body">
            <option gravity="0 0 0" timestep="0.01" integrator="RK4"/>
            <worldbody>
                <body name="spinner" pos="0 0 1">
                    <freejoint name="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load spinning body model");
    assert_eq!(model.integrator, Integrator::RungeKutta4);
    let mut data = model.make_data();

    // Set angular velocity around Z axis: ω = (0, 0, 2π) rad/s
    // After 1 second, body should have rotated exactly one full turn.
    let omega_z = 2.0 * PI;
    // Free joint DOF order: [vx, vy, vz, wx, wy, wz]
    data.qvel[3] = 0.0;
    data.qvel[4] = 0.0;
    data.qvel[5] = omega_z;

    let n_steps = 100; // 100 steps at h=0.01 = 1.0 second

    for step in 0..n_steps {
        data.step(&model).expect("step failed");

        // Check quaternion norm at every step
        // Free joint qpos: [x, y, z, qw, qx, qy, qz]
        let qw = data.qpos[3];
        let qx = data.qpos[4];
        let qy = data.qpos[5];
        let qz = data.qpos[6];
        let qnorm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
        assert!(
            (qnorm - 1.0).abs() < 1e-10,
            "Quaternion norm should be ~1.0, got {qnorm} at step {step}"
        );
    }

    // After exactly 1 full revolution (ωz = 2π, t = 1s):
    // q = [cos(π), 0, 0, sin(π)] = [-1, 0, 0, 0] or [1, 0, 0, 0]
    let qw = data.qpos[3];
    let qx = data.qpos[4];
    let qy = data.qpos[5];
    let qz = data.qpos[6];

    let orientation_error = 1.0 - qw.abs(); // distance from ±identity
    assert!(
        orientation_error < 1e-6,
        "After one full revolution, quaternion should be near ±identity: \
         qw={qw:.8}, qx={qx:.8}, qy={qy:.8}, qz={qz:.8}, error={orientation_error:.2e}"
    );
    assert!(qx.abs() < 1e-6, "qx should be ~0, got {qx:.8}");
    assert!(qy.abs() < 1e-6, "qy should be ~0, got {qy:.8}");
    assert!(qz.abs() < 1e-6, "qz should be ~0, got {qz:.8}");

    // Position should remain unchanged (no linear velocity, no gravity)
    assert_relative_eq!(data.qpos[0], 0.0, epsilon = 1e-8);
    assert_relative_eq!(data.qpos[1], 0.0, epsilon = 1e-8);
    assert_relative_eq!(data.qpos[2], 1.0, epsilon = 1e-8);
}

// ============================================================================
// AC #9: Sensor non-corruption
// ============================================================================

/// After step() with RK4, sensordata should contain values from the pre-step
/// forward() call. Intermediate RK4 stages should not corrupt sensor data.
#[test]
fn test_rk4_sensor_non_corruption() {
    let mjcf = r#"
        <mujoco model="sensor_test">
            <option gravity="0 0 -9.81" timestep="0.01" integrator="RK4"/>
            <worldbody>
                <body name="link">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0" pos="0 0 -0.5"/>
                    <site name="imu_site" pos="0 0 -0.5"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="j1_pos" joint="j1"/>
                <jointvel name="j1_vel" joint="j1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load sensor test model");
    assert_eq!(model.integrator, Integrator::RungeKutta4);
    let mut data = model.make_data();

    // Set initial state
    data.qpos[0] = 0.5; // 0.5 rad initial angle
    data.qvel[0] = 1.0; // 1 rad/s initial velocity

    // Run forward to populate sensordata with pre-step values
    data.forward(&model).expect("forward failed");

    // Verify sensors read the correct pre-step state
    if data.sensordata.len() >= 2 {
        assert_relative_eq!(data.sensordata[0], 0.5, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 1.0, epsilon = 1e-10);
    }

    // Now step (which calls forward() then mj_runge_kutta())
    data.step(&model).expect("step failed");

    // sensordata should be finite and not NaN (no corruption from intermediate stages)
    for (i, &val) in data.sensordata.iter().enumerate() {
        assert!(
            val.is_finite(),
            "sensordata[{i}] should be finite after RK4 step, got {val}"
        );
    }

    // After step(), sensordata should reflect the state at the start of step()
    // (forward() with sensors runs first, then mj_runge_kutta uses forward_skip_sensors).
    if !data.sensordata.is_empty() {
        // qpos has changed after step, but sensor should still reflect pre-step qpos (0.5)
        let post_step_qpos = data.qpos[0];
        assert!(
            (post_step_qpos - 0.5).abs() > 1e-6,
            "qpos should have changed after step"
        );
        assert_relative_eq!(data.sensordata[0], 0.5, epsilon = 1e-10);
    }
}

// ============================================================================
// AC #10: Warmstart efc_lambda preservation
// ============================================================================

/// efc_lambda after step() should reflect the initial state's contact solve,
/// not stage 3's. Verified by clone-before, compare-after.
#[test]
fn test_rk4_warmstart_efc_lambda_preservation() {
    let mjcf = r#"
        <mujoco model="warmstart_test">
            <option gravity="0 0 -9.81" timestep="0.005" integrator="RK4"/>
            <worldbody>
                <geom name="ground" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.15">
                    <freejoint name="ball_joint"/>
                    <geom name="ball_geom" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load warmstart test model");
    assert_eq!(model.integrator, Integrator::RungeKutta4);
    let mut data = model.make_data();

    // Place ball at ground contact (radius = 0.1)
    data.qpos[2] = 0.1;

    // Test the save/restore mechanism:
    // 1. Call forward() to populate efc_lambda from the contact solve
    data.forward(&model).expect("forward failed");
    let lambda_from_forward = data.efc_lambda.clone();

    // 2. Now step() from the same state. step() will:
    //    a. Call forward() → produces same efc_lambda (same state)
    //    b. Call mj_runge_kutta() → saves efc_lambda, runs 3 stages, restores
    data.step(&model).expect("step failed");
    let lambda_after_step = data.efc_lambda.clone();

    // 3. efc_lambda after step should be bitwise identical to what forward() produced
    assert_eq!(
        lambda_from_forward, lambda_after_step,
        "efc_lambda after RK4 step should be bitwise identical to pre-step forward() result \
         (save/restore mechanism)"
    );

    // Verify the efc_lambda values are finite
    for (key, lambda) in &lambda_after_step {
        for &l in lambda {
            assert!(
                l.is_finite(),
                "efc_lambda[({}, {})] contains non-finite value: {l}",
                key.geom_lo,
                key.geom_hi
            );
        }
    }
}
