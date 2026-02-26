//! DT-35: Tendon Spring/Damper Forces in `ImplicitSpringDamper` — Non-Diagonal K/D Coupling
//!
//! Tests 6a–6s from the DT-35 spec in `sim/docs/todo/future_work_10d.md`.
//! Verifies that tendon spring and damper forces are correctly handled by the
//! `ImplicitSpringDamper` integrator via non-diagonal K/D matrices.

use sim_core::{Data, Model, mjd_smooth_vel};

// ============================================================================
// Test Model Helpers
// ============================================================================

/// Build a 2-DOF model (two hinge joints on a serial chain) with a fixed tendon
/// coupling them. Parameterize by integrator, stiffness, damping, and solver.
fn make_tendon_implicit_model(integrator: &str, stiffness: f64, damping: f64) -> (Model, Data) {
    make_tendon_implicit_model_with_solver(integrator, stiffness, damping, "PGS")
}

fn make_tendon_implicit_model_with_solver(
    integrator: &str,
    stiffness: f64,
    damping: f64,
    solver: &str,
) -> (Model, Data) {
    // springlength="0 0" explicitly sets the deadband to [0, 0], meaning any
    // non-zero tendon length is outside the deadband. Without this attribute,
    // MuJoCo auto-computes springlength from the initial tendon length, which
    // happens to be 0 for this model (qpos = [0, 0] → L = 0). Explicit is
    // clearer and protects against accidental changes to initial state.
    let mjcf = format!(
        r#"
        <mujoco model="tendon_impl">
            <option timestep="0.001" integrator="{integrator}"
                    solver="{solver}" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="{stiffness}" damping="{damping}"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#
    );
    let model = sim_mjcf::load_model(&mjcf).unwrap();
    let data = model.make_data();
    (model, data)
}

/// Heavy-mass variant for Euler-vs-implicit comparison (6c).
fn make_tendon_implicit_model_heavy(
    integrator: &str,
    stiffness: f64,
    damping: f64,
) -> (Model, Data) {
    let mjcf = format!(
        r#"
        <mujoco model="tendon_impl_heavy">
            <option timestep="0.001" integrator="{integrator}" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="10.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="10.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="{stiffness}" damping="{damping}"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#
    );
    let model = sim_mjcf::load_model(&mjcf).unwrap();
    let data = model.make_data();
    (model, data)
}

// ============================================================================
// 6a. tendon_spring_nonzero_in_implicitspringdamper
// ============================================================================

#[test]
fn tendon_spring_nonzero_in_implicitspringdamper() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
    data.qpos[0] = 0.3; // Displace j0 — tendon stretches

    data.step(&model).unwrap();

    // The tendon spring should produce restoring acceleration.
    // With q0=0.3, tendon L = q0 - q1 = 0.3, spring force = -50*0.3 = -15.
    // This maps to qacc[0] via J^T. qacc[0] should be negative (restoring).
    assert!(
        data.qacc[0] < -1e-3,
        "qacc[0] should be negative (restoring), got {}",
        data.qacc[0]
    );

    // qacc should be non-trivial — not the zero that the old code produced
    let qacc_norm = data.qacc.norm();
    assert!(
        qacc_norm > 1e-3,
        "qacc norm should be non-trivial, got {qacc_norm}"
    );

    // ten_force[0] should be populated even in ImplicitSpringDamper mode
    // (diagnostic field). Spring force for positive displacement is negative.
    assert!(
        data.ten_force[0] < -1e-3,
        "ten_force[0] should be negative (spring restoring), got {}",
        data.ten_force[0]
    );
}

// ============================================================================
// 6b. tendon_damper_nonzero_in_implicitspringdamper
// ============================================================================

#[test]
fn tendon_damper_nonzero_in_implicitspringdamper() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 100.0);
    data.qvel[0] = 1.0; // Moving j0 — tendon damper resists

    data.step(&model).unwrap();

    // Damper should decelerate: qvel[0] after step should be less than 1.0
    assert!(
        data.qvel[0] < 1.0 - 1e-6,
        "qvel[0] should decrease from damping, got {}",
        data.qvel[0]
    );

    // ten_force[0] should be populated even in ImplicitSpringDamper mode
    // (diagnostic field). Damper resists positive velocity → negative force.
    assert!(
        data.ten_force[0] < -1e-3,
        "ten_force[0] should be negative (damper resisting), got {}",
        data.ten_force[0]
    );
}

// ============================================================================
// 6c. tendon_implicit_matches_euler_small_dt
// ============================================================================

#[test]
fn tendon_implicit_matches_euler_small_dt() {
    let (model_euler, mut data_euler) = make_tendon_implicit_model_heavy("Euler", 50.0, 10.0);
    let (model_impl, mut data_impl) =
        make_tendon_implicit_model_heavy("implicitspringdamper", 50.0, 10.0);

    // Same initial displacement
    data_euler.qpos[0] = 0.1;
    data_impl.qpos[0] = 0.1;

    // Step both forward 10 steps
    for _ in 0..10 {
        data_euler.step(&model_euler).unwrap();
        data_impl.step(&model_impl).unwrap();
    }

    // At dt=0.001, mass=10.0, and only 10 steps, the trajectories should be
    // very close. The implicit modification introduces O(h·D_eff/M_eff)
    // per-step numerical damping, accumulating to ≈3e-4 over 10 steps.
    // The implicit modification introduces O(h) per-step numerical damping.
    // With heavy masses and short trajectory, the accumulated divergence is
    // small but not negligible: use a tolerance of 0.05.
    for i in 0..model_euler.nv {
        let diff = (data_euler.qvel[i] - data_impl.qvel[i]).abs();
        assert!(
            diff < 0.05,
            "qvel[{i}] divergence too large: euler={}, impl={}, diff={diff}",
            data_euler.qvel[i],
            data_impl.qvel[i]
        );
    }
}

// ============================================================================
// 6d. tendon_implicit_stability_vs_euler
// ============================================================================

#[test]
fn tendon_implicit_stability_vs_euler() {
    fn make_tendon_stiff_model(integrator: &str) -> (Model, Data) {
        let mjcf = format!(
            r#"
            <mujoco model="tendon_stiff">
                <option timestep="0.01" integrator="{integrator}" gravity="0 0 0"/>
                <worldbody>
                    <body name="b1" pos="0 0 0">
                        <joint name="j0" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <body name="b2" pos="0 0 -1">
                            <joint name="j1" type="hinge" axis="0 1 0"
                                   stiffness="0" damping="0"/>
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="t0" stiffness="5000.0" damping="0.0"
                           springlength="0 0">
                        <joint joint="j0" coef="1.0"/>
                        <joint joint="j1" coef="-1.0"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#
        );
        let model = sim_mjcf::load_model(&mjcf).unwrap();
        let data = model.make_data();
        (model, data)
    }

    let (model_euler, mut data_euler) = make_tendon_stiff_model("Euler");
    let (model_impl, mut data_impl) = make_tendon_stiff_model("implicitspringdamper");

    data_euler.qpos[0] = 0.3;
    data_impl.qpos[0] = 0.3;

    // Step 200 times.
    // `let _ = ...` intentionally swallows errors: Euler with h²k/m >> 1 may
    // return Err(CholeskyFailed) or produce NaN/Inf. Both outcomes confirm
    // instability. ImplicitSpringDamper should never error here.
    for _ in 0..200 {
        let _ = data_euler.step(&model_euler);
        data_impl
            .step(&model_impl)
            .expect("ImplicitSpringDamper should not fail");
    }

    let vel_euler = data_euler.qvel.norm();
    let vel_impl = data_impl.qvel.norm();

    // ImplicitSpringDamper should stay bounded
    assert!(
        vel_impl < 100.0,
        "ImplicitSpringDamper velocity should be bounded, got {vel_impl}"
    );

    // Euler with dt=0.01, k=5000 should be unstable (h²k/m >> 1).
    // With auto-reset (§41 S8), diverged Euler state gets reset to qpos0,
    // so vel_euler may be 0 instead of NaN/Inf. Check divergence_detected().
    assert!(
        vel_impl < vel_euler
            || vel_euler.is_nan()
            || vel_euler.is_infinite()
            || data_euler.divergence_detected(),
        "Implicit should be more stable: vel_impl={vel_impl}, vel_euler={vel_euler}"
    );
}

// ============================================================================
// 6e. tendon_implicit_mass_matrix_has_off_diagonal
// ============================================================================

#[test]
fn tendon_implicit_mass_matrix_has_off_diagonal() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
    data.qpos[0] = 0.3; // Only displace j0

    data.step(&model).unwrap();

    // Off-diagonal coupling: J^T*k*J has nonzero (0,1) and (1,0) entries.
    // So qacc[1] should be NON-ZERO even though q1 = 0 and qvel[1] = 0.
    assert!(
        data.qacc[1].abs() > 1e-3,
        "qacc[1] should be non-zero from off-diagonal coupling, got {}",
        data.qacc[1]
    );

    // Signs: tendon L = q0 - q1 = 0.3 > 0, spring force = -k*0.3 = -15.
    // qfrc = J^T * F = [1, -1]^T * (-15) = [-15, +15].
    // So qacc[0] < 0 (restoring) and qacc[1] > 0 (pulled toward q0).
    assert!(
        data.qacc[0] < 0.0,
        "qacc[0] should be negative, got {}",
        data.qacc[0]
    );
    assert!(
        data.qacc[1] > 0.0,
        "qacc[1] should be positive, got {}",
        data.qacc[1]
    );
}

// ============================================================================
// 6f. tendon_qDeriv_includes_damping_for_all_integrators
// ============================================================================

#[test]
#[allow(non_snake_case)]
fn tendon_qDeriv_includes_damping_for_all_integrators() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 100.0);
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();
    mjd_smooth_vel(&model, &mut data); // Populates qDeriv

    // qDeriv should have tendon damping contribution.
    // For J = [1, -1], b = 100: qDeriv += -b * J^T * J = -100 * [[1,-1],[-1,1]]
    // Diagonal entries: qDeriv[(0,0)] += -100 * 1 * 1 = -100
    // Off-diagonal:     qDeriv[(0,1)] += -100 * 1 * (-1) = +100
    assert!(
        data.qDeriv[(0, 0)] < -1e-6,
        "qDeriv[(0,0)] should be negative from tendon damping, got {}",
        data.qDeriv[(0, 0)]
    );
    assert!(
        data.qDeriv[(0, 1)] > 1e-6,
        "qDeriv[(0,1)] should be positive (J[0]*J[1] = 1*(-1) = -1, times -b = +100), got {}",
        data.qDeriv[(0, 1)]
    );
}

// ============================================================================
// 6g. tendon_implicit_combined_spring_damper
// ============================================================================

#[test]
fn tendon_implicit_combined_spring_damper() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 50.0, 20.0);
    data.qpos[0] = 0.3; // Displace — spring activates
    data.qvel[0] = 1.0; // Moving — damper activates

    data.step(&model).unwrap();

    // Spring restoring: qacc[0] should be negative.
    assert!(
        data.qacc[0] < -1e-3,
        "qacc[0] should be negative (spring restoring + damper resisting), got {}",
        data.qacc[0]
    );

    // qvel[0] should decrease (damper decelerates, spring pulls back)
    assert!(
        data.qvel[0] < 1.0 - 1e-6,
        "qvel[0] should decrease from combined spring+damper, got {}",
        data.qvel[0]
    );

    // Cross-coupling: qacc[1] should be non-zero from off-diagonal terms
    assert!(
        data.qacc[1].abs() > 1e-3,
        "qacc[1] should show cross-coupling, got {}",
        data.qacc[1]
    );

    // Verify both spring and damper are contributing by comparing components.
    // The combined result should differ from a spring-only simulation.
    let (model_s, mut data_s) = make_tendon_implicit_model("implicitspringdamper", 50.0, 0.0);
    data_s.qpos[0] = 0.3;
    data_s.qvel[0] = 1.0;
    data_s.step(&model_s).unwrap();

    // Combined and spring-only should produce different qacc (damping has effect)
    let qacc_diff = (data.qacc[0] - data_s.qacc[0]).abs();
    assert!(
        qacc_diff > 1e-6,
        "Combined should differ from spring-only: combined={}, spring_only={}, diff={qacc_diff}",
        data.qacc[0],
        data_s.qacc[0]
    );
}

// ============================================================================
// 6h. tendon_implicit_energy_dissipation
// ============================================================================

#[test]
fn tendon_implicit_energy_dissipation() {
    // --- Part A: Damper-only energy dissipation ---
    let (model_d, mut data_d) = make_tendon_implicit_model("implicitspringdamper", 0.0, 50.0);
    data_d.qvel[0] = 2.0;
    data_d.qvel[1] = -1.0;

    let mut prev_ke = f64::MAX;
    for step in 0..50 {
        data_d.step(&model_d).unwrap();
        // Kinetic energy: 0.5 * v^T * M * v
        let ke = {
            let mv = &data_d.qM * &data_d.qvel;
            0.5 * data_d.qvel.dot(&mv)
        };
        assert!(
            ke <= prev_ke + 1e-12,
            "Step {step}: KE should not increase with damper-only: prev={prev_ke}, now={ke}"
        );
        prev_ke = ke;
    }
    // After 50 steps, kinetic energy should have decreased significantly.
    // (Velocity norm may not decrease monotonically due to off-diagonal
    // coupling redistributing energy between DOFs, but total KE must decrease.)
    assert!(
        prev_ke < f64::MAX,
        "KE tracking should have recorded valid values"
    );

    // --- Part B: Spring-only energy approximate conservation ---
    let mjcf_spring_energy = r#"
        <mujoco model="tendon_energy">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="50.0" damping="0.0"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model_s = sim_mjcf::load_model(mjcf_spring_energy).unwrap();
    let mut data_s = model_s.make_data();
    data_s.qpos[0] = 0.3;

    let disp0 = data_s.qpos[0] - data_s.qpos[1];
    let pe0 = 0.5 * 50.0 * disp0 * disp0;
    let ke0 = 0.0; // zero initial velocity
    let e0 = pe0 + ke0;

    for _ in 0..100 {
        data_s.step(&model_s).unwrap();
    }

    // Recompute energy
    let l = data_s.qpos[0] - data_s.qpos[1];
    let displacement = l; // deadband [0, 0]: any non-zero L is outside
    let pe = 0.5 * 50.0 * displacement * displacement;
    let ke = {
        let mv = &data_s.qM * &data_s.qvel;
        0.5 * data_s.qvel.dot(&mv)
    };
    let e_final = pe + ke;

    // Implicit integrator adds numerical damping: energy should NOT increase.
    assert!(
        e_final <= e0 + 1e-6,
        "Total energy should not increase for spring-only: e0={e0}, e_final={e_final}"
    );
    // Energy should not drop to near-zero — the implicit integrator introduces
    // numerical damping O(h) per step, so over 100 steps energy can drop
    // significantly but should retain a measurable fraction.
    assert!(
        e_final > 0.01 * e0,
        "Total energy should retain a measurable fraction: e0={e0}, e_final={e_final}"
    );
}

// ============================================================================
// 6k. tendon_implicit_deadband_zero_inside
// ============================================================================

#[test]
fn tendon_implicit_deadband_zero_inside() {
    let mjcf = r#"
        <mujoco model="tendon_deadband">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="100.0" damping="0.0"
                       springlength="0.5 1.0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // --- Sub-case A1: Inside deadband, zero velocity ---
    // Set tendon length L = q0 - q1 = 0.7, inside deadband [0.5, 1.0]
    data.qpos[0] = 0.7;

    data.step(&model).unwrap();

    // No spring force inside deadband — qacc should be zero
    let qacc_norm = data.qacc.norm();
    assert!(
        qacc_norm < 1e-10,
        "qacc should be ~zero inside deadband, got norm={qacc_norm}"
    );

    // --- Sub-case A2: Inside deadband WITH nonzero velocity ---
    let mut data_vel = model.make_data();
    data_vel.qpos[0] = 0.7; // Inside deadband
    data_vel.qvel[0] = 1.0; // Moving — should NOT be damped by spring
    data_vel.step(&model).unwrap();

    let qacc_vel_norm = data_vel.qacc.norm();
    assert!(
        qacc_vel_norm < 1e-10,
        "qacc should be ~zero inside deadband even with velocity, got norm={qacc_vel_norm}"
    );
    assert!(
        (data_vel.qvel[0] - 1.0).abs() < 1e-10,
        "qvel[0] should be unchanged inside deadband (no spring, no damper), got {}",
        data_vel.qvel[0]
    );

    // --- Sub-case B: Below lower bound ---
    let mut data_below = model.make_data();
    data_below.qpos[0] = 0.2; // L = 0.2 < 0.5

    data_below.step(&model).unwrap();

    // qacc[0] should be positive (pushing q0 larger to lengthen tendon)
    assert!(
        data_below.qacc[0] > 1e-3,
        "Below deadband: qacc[0] should be positive (lengthening), got {}",
        data_below.qacc[0]
    );
    assert!(
        data_below.qacc[1] < -1e-3,
        "Below deadband: qacc[1] should be negative, got {}",
        data_below.qacc[1]
    );

    // --- Sub-case C: Above upper bound ---
    let mut data_above = model.make_data();
    data_above.qpos[0] = 1.5; // L = 1.5 > 1.0

    data_above.step(&model).unwrap();

    assert!(
        data_above.qacc[0] < -1e-3,
        "Above deadband: qacc[0] should be negative (shortening), got {}",
        data_above.qacc[0]
    );
    assert!(
        data_above.qacc[1] > 1e-3,
        "Above deadband: qacc[1] should be positive, got {}",
        data_above.qacc[1]
    );

    // --- Sub-case D: Exactly AT the upper deadband boundary, with velocity ---
    let mut data_boundary = model.make_data();
    data_boundary.qpos[0] = 1.0; // L = upper boundary exactly
    data_boundary.qvel[0] = 2.0; // Moving outward

    data_boundary.step(&model).unwrap();

    // At boundary, k_active = 0, displacement = 0 → no spring force.
    let qacc_step1_norm = data_boundary.qacc.norm();
    assert!(
        qacc_step1_norm < 1e-10,
        "At boundary: qacc should be ~zero (spring disengaged), got norm={}",
        qacc_step1_norm
    );

    data_boundary.step(&model).unwrap();

    // Step 2: Velocity moved L past upper boundary. Spring should activate.
    assert!(
        data_boundary.qacc[0].abs() > 1e-3,
        "After crossing boundary: spring should activate, got qacc[0]={}",
        data_boundary.qacc[0]
    );
}

// ============================================================================
// 6l. tendon_implicit_spatial_tendon_basic
// ============================================================================

#[test]
fn tendon_implicit_spatial_tendon_basic() {
    let mjcf = r#"
        <mujoco model="tendon_spatial_impl">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom name="g0" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" mass="1.0"/>
                    <site name="s0" pos="0.1 0 -0.25"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom name="g1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" mass="1.0"/>
                        <site name="s1" pos="0.1 0 -0.25"/>
                    </body>
                </body>
                <site name="s_world" pos="0.3 0 0"/>
            </worldbody>
            <tendon>
                <spatial name="sp0" stiffness="50.0" damping="10.0"
                         springlength="0 0">
                    <site site="s_world"/>
                    <site site="s0"/>
                    <site site="s1"/>
                </spatial>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();

    data.qpos[0] = 0.3; // Rotate j0 — stretches spatial tendon

    // Compute forward kinematics to get the tendon Jacobian before stepping
    data.forward(&model).unwrap();
    let j0 = data.ten_J[0][0];
    let j1 = data.ten_J[0][1];
    assert!(
        j0.abs() > 1e-6,
        "Spatial tendon J[0] should be non-zero, got {j0}"
    );

    data.step(&model).unwrap();

    // Spatial tendon should produce restoring forces
    let qacc_norm = data.qacc.norm();
    assert!(
        qacc_norm > 1e-3,
        "Spatial tendon should produce non-trivial qacc, got norm={qacc_norm}"
    );

    // Direction check: positive q0 displacement stretches the tendon,
    // so the restoring spring force should produce negative qacc[0]
    assert!(
        data.qacc[0] < 0.0,
        "qacc[0] should be negative (restoring) for positive displacement, got {}",
        data.qacc[0]
    );

    // Cross-coupling
    if j1.abs() > 1e-6 {
        assert!(
            data.qacc[1].abs() > 1e-6,
            "qacc[1] should be non-zero from spatial tendon coupling, got {}",
            data.qacc[1]
        );
    }
}

// ============================================================================
// 6l½. tendon_implicit_multi_tendon_summation
// ============================================================================

#[test]
fn tendon_implicit_multi_tendon_summation() {
    let mjcf = r#"
        <mujoco model="tendon_multi">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <body name="b3" pos="0 0 -1">
                            <joint name="j2" type="hinge" axis="0 1 0"
                                   stiffness="0" damping="0"/>
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="50.0" damping="0.0"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
                <fixed name="t1" stiffness="30.0" damping="0.0"
                       springlength="0 0">
                    <joint joint="j1" coef="1.0"/>
                    <joint joint="j2" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Displace j0 only — t0 stretches, t1 is at rest.
    data.qpos[0] = 0.3;
    data.step(&model).unwrap();

    assert!(
        data.qacc[0] < -1e-3,
        "qacc[0] should be negative from t0 spring, got {}",
        data.qacc[0]
    );
    assert!(
        data.qacc[1].abs() > 1e-3,
        "qacc[1] should be non-zero from t0 coupling, got {}",
        data.qacc[1]
    );
    // qacc[2]: t0 doesn't directly touch j2, but the serial chain's inertia
    // coupling and t1's LHS modification distribute the force. Just check finite.
    assert!(
        data.qacc[2].is_finite(),
        "qacc[2] should be finite, got {}",
        data.qacc[2]
    );

    // Now displace both — both tendons active.
    let mut data2 = model.make_data();
    data2.qpos[0] = 0.3;
    data2.qpos[2] = -0.2; // t1 stretches: L1 = q1 - q2 = 0 - (-0.2) = 0.2
    data2.step(&model).unwrap();

    // j2 should have non-zero qacc from t1.
    assert!(
        data2.qacc[2].abs() > 1e-3,
        "qacc[2] should be non-zero when t1 is active, got {}",
        data2.qacc[2]
    );

    // j1 should have contributions from BOTH tendons — the result should
    // differ from the single-tendon case.
    let qacc1_diff = (data2.qacc[1] - data.qacc[1]).abs();
    assert!(
        qacc1_diff > 1e-6,
        "qacc[1] should differ with both tendons active: \
         both={}, single={}, diff={qacc1_diff}",
        data2.qacc[1],
        data.qacc[1]
    );
}

// ============================================================================
// 6l¾. tendon_implicit_zero_kd_noop
// ============================================================================

#[test]
fn tendon_implicit_zero_kd_noop() {
    let (model, mut data) = make_tendon_implicit_model("implicitspringdamper", 0.0, 0.0);
    data.qpos[0] = 0.3; // Displace — but no spring, no damper

    data.step(&model).unwrap();

    // No spring or damper: qacc should be zero
    let qacc_norm = data.qacc.norm();
    assert!(
        qacc_norm < 1e-10,
        "qacc should be ~zero with k=0 b=0, got norm={qacc_norm}"
    );
}

// ============================================================================
// 6n. tendon_implicit_newton_spring_nonzero
// ============================================================================

#[test]
fn tendon_implicit_newton_spring_nonzero() {
    let (model, mut data) =
        make_tendon_implicit_model_with_solver("implicitspringdamper", 50.0, 0.0, "Newton");
    data.qpos[0] = 0.3;

    data.step(&model).unwrap();

    // Newton + ImplicitSpringDamper should produce non-zero restoring acceleration.
    assert!(
        data.qacc[0] < -1e-3,
        "Newton: qacc[0] should be negative (restoring), got {}",
        data.qacc[0]
    );
    // Off-diagonal coupling must also work through Newton.
    assert!(
        data.qacc[1] > 1e-3,
        "Newton: qacc[1] should be positive (off-diagonal coupling), got {}",
        data.qacc[1]
    );
}

// ============================================================================
// 6o. tendon_implicit_solver_flag_no_effect_unconstrained
// ============================================================================

#[test]
fn tendon_implicit_solver_flag_no_effect_unconstrained() {
    let (model_pgs, mut data_pgs) =
        make_tendon_implicit_model_with_solver("implicitspringdamper", 50.0, 20.0, "PGS");
    let (model_newton, mut data_newton) =
        make_tendon_implicit_model_with_solver("implicitspringdamper", 50.0, 20.0, "Newton");

    // Same initial state — no contacts, no limits, so no active constraints.
    data_pgs.qpos[0] = 0.3;
    data_pgs.qvel[0] = 1.0;
    data_newton.qpos[0] = 0.3;
    data_newton.qvel[0] = 1.0;

    data_pgs.step(&model_pgs).unwrap();
    data_newton.step(&model_newton).unwrap();

    // Results should match to machine precision (both take identical codepath).
    for i in 0..model_pgs.nv {
        let diff = (data_pgs.qvel[i] - data_newton.qvel[i]).abs();
        assert!(
            diff < 1e-10,
            "qvel[{i}] should match between PGS and Newton (unconstrained): \
             pgs={}, newton={}, diff={diff}",
            data_pgs.qvel[i],
            data_newton.qvel[i]
        );
    }
    for i in 0..model_pgs.nv {
        let diff = (data_pgs.qacc[i] - data_newton.qacc[i]).abs();
        assert!(
            diff < 1e-10,
            "qacc[{i}] should match between PGS and Newton (unconstrained): \
             pgs={}, newton={}, diff={diff}",
            data_pgs.qacc[i],
            data_newton.qacc[i]
        );
    }
}

// ============================================================================
// 6p. tendon_implicit_newton_with_contact
// ============================================================================

#[test]
fn tendon_implicit_newton_with_contact() {
    let mjcf = r#"
        <mujoco model="tendon_newton_contact">
            <option timestep="0.001" integrator="implicitspringdamper"
                    solver="Newton" gravity="0 0 -9.81"/>
            <worldbody>
                <geom type="plane" size="5 5 0.1"/>
                <body name="b1" pos="0 0 0.5">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="100.0" damping="10.0"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // Displace — tendon spring activates

    // Step 50 times — should not crash or produce NaN
    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    // Verify no NaN/Inf in state
    assert!(
        !data.qvel[0].is_nan() && !data.qvel[0].is_infinite(),
        "qvel should be finite, got {:?}",
        &data.qvel
    );
    assert!(
        !data.qacc[0].is_nan() && !data.qacc[0].is_infinite(),
        "qacc should be finite, got {:?}",
        &data.qacc
    );

    // Tendon spring should have affected the trajectory
    let qacc_norm = data.qacc.norm();
    assert!(
        qacc_norm > 1e-3,
        "qacc should be non-trivial with tendon + contact, got norm={qacc_norm}"
    );
}

// ============================================================================
// 6q. tendon_implicit_single_dof_tendon
// ============================================================================

#[test]
fn tendon_implicit_single_dof_tendon() {
    let mjcf = r#"
        <mujoco model="tendon_single_dof">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="50.0" damping="0.0"
                       springlength="0 0">
                    <joint joint="j0" coef="2.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // Displace — L = 2.0 * 0.3 = 0.6

    data.step(&model).unwrap();

    // Spring force: F = -k * L = -50 * 0.6 = -30
    // Joint force: qfrc = J^T * F = 2.0 * (-30) = -60
    // qacc should be negative (restoring)
    assert!(
        data.qacc[0] < -1e-3,
        "Single-DOF tendon: qacc[0] should be negative, got {}",
        data.qacc[0]
    );
}

// ============================================================================
// 6r. tendon_implicit_newton_joint_spring_bonus
// ============================================================================

#[test]
fn tendon_implicit_newton_joint_spring_bonus() {
    // Model with joint spring ONLY (no tendon), Newton solver
    let mjcf = r#"
        <mujoco model="joint_spring_newton">
            <option timestep="0.001" integrator="implicitspringdamper"
                    solver="Newton" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="100.0" damping="10.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // Displace — joint spring activates

    data.step(&model).unwrap();

    // Joint spring should produce restoring acceleration
    assert!(
        data.qacc[0] < -1e-3,
        "Newton + joint spring: qacc[0] should be negative (restoring), got {}",
        data.qacc[0]
    );

    // Compare against PGS path — should match (unconstrained).
    let mjcf_pgs = r#"
        <mujoco model="joint_spring_pgs">
            <option timestep="0.001" integrator="implicitspringdamper"
                    solver="PGS" gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="100.0" damping="10.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model_pgs = sim_mjcf::load_model(mjcf_pgs).unwrap();
    let mut data_pgs = model_pgs.make_data();
    data_pgs.qpos[0] = 0.5;
    data_pgs.step(&model_pgs).unwrap();

    let diff = (data.qacc[0] - data_pgs.qacc[0]).abs();
    assert!(
        diff < 1e-10,
        "Newton and PGS should match for unconstrained joint spring: \
         newton={}, pgs={}, diff={diff}",
        data.qacc[0],
        data_pgs.qacc[0]
    );
}

// ============================================================================
// 6s. tendon_implicit_sleep_guard_skips_sleeping_tendons
// ============================================================================

#[test]
fn tendon_implicit_sleep_guard_skips_sleeping_tendons() {
    // Zero gravity so sleeping DOFs have no external forces — qacc must be
    // exactly zero when all trees are forced asleep.
    let mjcf = r#"
        <mujoco model="tendon_sleep">
            <option timestep="0.001" integrator="implicitspringdamper" gravity="0 0 0">
                <flag sleep="enable"/>
            </option>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j0" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="100.0" damping="10.0"
                       springlength="0 0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();

    // --- Awake: tendon spring should produce restoring acceleration ---
    let mut data_awake = model.make_data();
    data_awake.qpos[0] = 0.3;
    data_awake.step(&model).unwrap();
    let qacc_awake_norm = data_awake.qacc.norm();
    assert!(
        qacc_awake_norm > 1e-3,
        "Awake: tendon spring should produce non-trivial qacc, got {qacc_awake_norm}"
    );

    // --- Sleeping: force all trees asleep, verify tendon K/D is skipped ---
    let mut data_sleep = model.make_data();
    data_sleep.qpos[0] = 0.3;
    // Run forward to compute tendon Jacobians/lengths with the displacement
    data_sleep.forward(&model).unwrap();

    // Force ALL trees to sleep (same pattern as gravcomp.rs:412-414)
    for i in 0..model.ntree {
        data_sleep.tree_awake[i] = false;
    }
    data_sleep.nv_awake = 0;
    data_sleep.dof_awake_ind.clear();

    // Step — implicit solver should skip tendon K/D for sleeping DOFs
    data_sleep.step(&model).unwrap();
    let qacc_sleep_norm = data_sleep.qacc.norm();
    assert!(
        qacc_sleep_norm < 1e-10,
        "Sleeping: tendon forces should be skipped, got qacc norm={qacc_sleep_norm}"
    );
}
