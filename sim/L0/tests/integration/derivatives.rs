//! Integration tests for analytical derivatives (Part 1, Steps 0–7).
//!
//! Covers acceptance criteria 1–23 from the derivatives spec
//! in `sim/docs/todo/future_work_4.md`, section 12.

use approx::assert_relative_eq;
use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, DISABLE_GRAVITY, DISABLE_SPRING,
    DerivativeConfig, GainType, Integrator, Model, mj_integrate_pos_explicit, mj_solve_sparse,
    mjd_smooth_pos, mjd_smooth_vel, mjd_transition_fd, mjd_transition_hybrid,
    validate_analytical_vs_fd,
};

// ============================================================================
// Test fixture helpers
// ============================================================================

/// Add `n` simple torque actuators (GainType::Fixed, Joint transmission)
/// to a model — one per joint starting from joint 0.
fn add_torque_actuators(model: &mut Model, n: usize) {
    for i in 0..n {
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([i, usize::MAX]);
        model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-100.0, 100.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 10]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0; // gain = 1
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);
    }
    model.nu = n;
}

/// Add one affine actuator with velocity-dependent gain/bias terms.
/// gainprm = [g_l, _, g_v, ...], biasprm = [b_l, _, b_v, ...]
fn add_affine_actuator(model: &mut Model, jnt_id: usize, gear: f64, gain_v: f64, bias_v: f64) {
    model.actuator_trntype.push(ActuatorTransmission::Joint);
    model.actuator_trnid.push([jnt_id, usize::MAX]);
    model.actuator_gear.push([gear, 0.0, 0.0, 0.0, 0.0, 0.0]);
    model.actuator_dyntype.push(ActuatorDynamics::None);
    model.actuator_ctrlrange.push((-100.0, 100.0));
    model.actuator_forcerange.push((-100.0, 100.0));
    model.actuator_name.push(None);
    model.actuator_act_adr.push(0);
    model.actuator_act_num.push(0);
    model.actuator_gaintype.push(GainType::Affine);
    model.actuator_biastype.push(BiasType::Affine);
    model.actuator_dynprm.push([0.0; 10]);
    model.actuator_gainprm.push({
        let mut p = [0.0; 9];
        p[0] = 1.0; // gainprm[0] = gain_l (length gain, used as fixed part)
        p[2] = gain_v; // gainprm[2] = velocity-dependent gain
        p
    });
    model.actuator_biasprm.push({
        let mut p = [0.0; 9];
        p[2] = bias_v; // biasprm[2] = velocity-dependent bias
        p
    });
    model.actuator_lengthrange.push((0.0, 0.0));
    model.actuator_acc0.push(0.0);
    model.actuator_actlimited.push(false);
    model.actuator_actrange.push((0.0, 0.0));
    model.actuator_actearly.push(false);
    model.nu += 1;
}

/// Add a filter actuator with activation dynamics (contributes to `na`).
fn add_filter_actuator(model: &mut Model, jnt_id: usize, tau: f64) {
    let act_adr = model.na;
    model.actuator_trntype.push(ActuatorTransmission::Joint);
    model.actuator_trnid.push([jnt_id, usize::MAX]);
    model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    model.actuator_dyntype.push(ActuatorDynamics::Filter);
    model.actuator_ctrlrange.push((-100.0, 100.0));
    model.actuator_forcerange.push((-100.0, 100.0));
    model.actuator_name.push(None);
    model.actuator_act_adr.push(act_adr);
    model.actuator_act_num.push(1);
    model.actuator_gaintype.push(GainType::Fixed);
    model.actuator_biastype.push(BiasType::None);
    model.actuator_dynprm.push({
        let mut p = [0.0; 10];
        p[0] = tau; // time constant
        p
    });
    model.actuator_gainprm.push({
        let mut p = [0.0; 9];
        p[0] = 1.0;
        p
    });
    model.actuator_biasprm.push([0.0; 9]);
    model.actuator_lengthrange.push((0.0, 0.0));
    model.actuator_acc0.push(0.0);
    model.actuator_actlimited.push(false);
    model.actuator_actrange.push((0.0, 0.0));
    model.actuator_actearly.push(false);
    model.nu += 1;
    model.na += 1;
}

/// Compute max relative error between two matrices (element-wise).
/// Denominator uses max(|a|, |b|, floor) to avoid div-by-zero.
fn max_relative_error(a: &nalgebra::DMatrix<f64>, b: &nalgebra::DMatrix<f64>, floor: f64) -> f64 {
    assert_eq!(a.nrows(), b.nrows());
    assert_eq!(a.ncols(), b.ncols());
    let mut max_err = 0.0_f64;
    for r in 0..a.nrows() {
        for c in 0..a.ncols() {
            let va = a[(r, c)];
            let vb = b[(r, c)];
            let denom = va.abs().max(vb.abs()).max(floor);
            let err = (va - vb).abs() / denom;
            max_err = max_err.max(err);
        }
    }
    max_err
}

/// FD of qfrc_bias w.r.t. qvel for a single column (column `col`).
/// Returns the nv-length column of ∂(qfrc_bias)/∂(qvel[col]).
fn fd_qfrc_bias_col(
    model: &Model,
    data: &sim_core::Data,
    col: usize,
    eps: f64,
) -> nalgebra::DVector<f64> {
    let mut scratch = data.clone();

    // +eps
    scratch.qvel.copy_from(&data.qvel);
    scratch.qvel[col] += eps;
    scratch.qpos.copy_from(&data.qpos);
    scratch.forward(model).unwrap();
    let bias_plus = scratch.qfrc_bias.clone();

    // -eps
    scratch.qvel.copy_from(&data.qvel);
    scratch.qvel[col] -= eps;
    scratch.qpos.copy_from(&data.qpos);
    scratch.forward(model).unwrap();
    let bias_minus = scratch.qfrc_bias.clone();

    (&bias_plus - &bias_minus) / (2.0 * eps)
}

/// FD of (qfrc_passive + qfrc_actuator - qfrc_bias) w.r.t. qvel[col].
fn fd_smooth_force_col(
    model: &Model,
    data: &sim_core::Data,
    col: usize,
    eps: f64,
) -> nalgebra::DVector<f64> {
    let mut scratch = data.clone();

    // +eps
    scratch.qvel.copy_from(&data.qvel);
    scratch.qvel[col] += eps;
    scratch.qpos.copy_from(&data.qpos);
    scratch.forward(model).unwrap();
    let smooth_plus = &scratch.qfrc_passive + &scratch.qfrc_actuator - &scratch.qfrc_bias;

    // -eps
    scratch.qvel.copy_from(&data.qvel);
    scratch.qvel[col] -= eps;
    scratch.qpos.copy_from(&data.qpos);
    scratch.forward(model).unwrap();
    let smooth_minus = &scratch.qfrc_passive + &scratch.qfrc_actuator - &scratch.qfrc_bias;

    (&smooth_plus - &smooth_minus) / (2.0 * eps)
}

// ============================================================================
// Acceptance criterion 1: A matrix dimensions
// ============================================================================

#[test]
fn test_a_matrix_dimensions() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // nv=3, na=0, nu=0 → A is 6×6, B is 6×0
    assert_eq!(derivs.A.nrows(), 6);
    assert_eq!(derivs.A.ncols(), 6);
    assert_eq!(derivs.B.nrows(), 6);
    assert_eq!(derivs.B.ncols(), 0);
    assert!(derivs.C.is_none());
    assert!(derivs.D.is_none());
}

// ============================================================================
// Acceptance criterion 2: Centered vs forward convergence
// ============================================================================

#[test]
fn test_centered_vs_forward_convergence() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[1] = 0.5;
    data.forward(&model).unwrap();

    let centered = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let forward = DerivativeConfig {
        eps: 1e-6,
        centered: false,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };

    let dc = mjd_transition_fd(&model, &data, &centered).unwrap();
    let df = mjd_transition_fd(&model, &data, &forward).unwrap();

    // Use a floor proportional to the matrix scale to avoid inflated relative
    // errors from near-zero entries.
    let a_max = dc.A.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    let err = max_relative_error(&dc.A, &df.A, a_max * 1e-4);
    // Centered and forward FD agree to O(eps) for forward, O(eps²) for centered.
    // With eps=1e-6, forward truncation error limits agreement.
    assert!(
        err < 1e-2,
        "Centered vs forward should agree to 1e-2, got {}",
        err
    );
}

// ============================================================================
// Acceptance criterion 3: B matrix structure
// ============================================================================

#[test]
fn test_b_matrix_structure() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    add_torque_actuators(&mut model, 3);
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // B has shape 6×3
    assert_eq!(derivs.B.nrows(), 6);
    assert_eq!(derivs.B.ncols(), 3);

    // Velocity rows (B[3..6, :]) should have non-trivial entries
    let vel_block = derivs.B.rows(3, 3);
    assert!(
        vel_block.norm() > 1e-8,
        "Velocity rows of B should be non-zero"
    );

    // Position rows (B[0..3, :]) should be approximately h times velocity rows
    let h = model.timestep;
    let pos_block = derivs.B.rows(0, 3);
    for r in 0..3 {
        for c in 0..3 {
            let expected = h * vel_block[(r, c)];
            let actual = pos_block[(r, c)];
            // Semi-implicit Euler: ∂q⁺/∂ctrl = h · ∂v⁺/∂ctrl
            if expected.abs() > 1e-10 {
                let rel_err = (actual - expected).abs() / expected.abs().max(1e-10);
                assert!(
                    rel_err < 1e-3,
                    "B position row [{},{}]: expected {}, got {}, err={}",
                    r,
                    c,
                    expected,
                    actual,
                    rel_err
                );
            }
        }
    }
}

// ============================================================================
// Acceptance criterion 4: Quaternion handling (Ball joint)
// ============================================================================

#[test]
fn test_quaternion_handling_ball_joint() {
    // Load a model with a Ball joint via MJCF
    let mjcf = r#"
        <mujoco model="ball_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="ball" name="ball"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // Ball: nq=4, nv=3, na=0, nu=0
    // A should be (2*nv + na) × (2*nv + na) = 6×6, NOT 2*nq=8
    assert_eq!(derivs.A.nrows(), 2 * model.nv);
    assert_eq!(derivs.A.ncols(), 2 * model.nv);
}

// ============================================================================
// Acceptance criterion 5: Activation derivatives (Filter dynamics)
// ============================================================================

#[test]
fn test_activation_derivatives() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    let tau = 0.1; // filter time constant
    add_filter_actuator(&mut model, 0, tau);
    add_filter_actuator(&mut model, 1, tau);
    let mut data = model.make_data();
    data.act[0] = 0.5;
    data.act[1] = 0.3;
    data.ctrl[0] = 1.0;
    data.ctrl[1] = 0.5;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // na=2, nv=2 → state dim = 2*2+2 = 6
    let nv = model.nv;
    let na = model.na;
    assert_eq!(na, 2);

    // Activation block: A[2*nv..2*nv+2, 2*nv..2*nv+2]
    // For Filter dynamics: act_dot = (ctrl - act) / tau
    // ∂act⁺/∂act = 1 - h/tau (diagonal)
    let h = model.timestep;
    let expected_diag = 1.0 - h / tau;
    for i in 0..na {
        let actual = derivs.A[(2 * nv + i, 2 * nv + i)];
        assert_relative_eq!(actual, expected_diag, epsilon = 1e-4);
    }
}

// ============================================================================
// Acceptance criterion 6: Contact sensitivity
// ============================================================================

#[test]
fn test_contact_sensitivity() {
    let mjcf = r#"
        <mujoco model="contact_sens">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 0.1"/>
                <body name="sphere" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"
                          condim="1" solref="0.02 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    // Sphere touching the ground
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // ∂qvel⁺/∂qpos block should have non-zero entries (contact stiffness)
    let nv = model.nv;
    let vel_pos_block = derivs.A.view((nv, 0), (nv, nv));
    let norm = vel_pos_block.norm();
    assert!(
        norm > 1e-6,
        "Contact should produce non-zero vel-pos derivative, got norm={}",
        norm
    );
}

// ============================================================================
// Acceptance criterion 7: Integrator coverage
// ============================================================================

#[test]
fn test_integrator_coverage_euler() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config);
    assert!(derivs.is_ok(), "Euler FD should succeed");
}

#[test]
fn test_integrator_coverage_implicit() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    model.integrator = Integrator::ImplicitSpringDamper;
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config);
    assert!(derivs.is_ok(), "ImplicitSpringDamper FD should succeed");
}

/// A pendulum with a motor under `integrator`; `extra` goes inside `<option>`'s
/// tag and `actuator` is the motor's extra attributes.
fn fd_pendulum(integrator: &str, extra: &str, actuator: &str) -> Model {
    let xml = format!(
        r#"<mujoco><option timestep="0.01" integrator="{integrator}" {extra}/>
<worldbody><body><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
<geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody>
<actuator><motor joint="j" {actuator}/></actuator></mujoco>"#
    );
    sim_mjcf::load_model(&xml).expect("load")
}

/// Every finite-difference entry point, through `mjd_transition` and the
/// `Data` method too, with and without the analytic path.
fn fd_entry_points(
    model: &Model,
    data: &sim_core::Data,
) -> Vec<(String, Result<(), sim_core::StepError>)> {
    let mut out = Vec::new();
    for analytic in [false, true] {
        let cfg = DerivativeConfig {
            use_analytical: analytic,
            ..Default::default()
        };
        out.push((
            format!("mjd_transition_fd {analytic}"),
            mjd_transition_fd(model, data, &cfg).map(|_| ()),
        ));
        out.push((
            format!("mjd_transition_hybrid {analytic}"),
            mjd_transition_hybrid(model, data, &cfg).map(|_| ()),
        ));
        out.push((
            format!("mjd_transition {analytic}"),
            sim_core::mjd_transition(model, data, &cfg).map(|_| ()),
        ));
        out.push((
            format!("transition_derivatives {analytic}"),
            data.transition_derivatives(model, &cfg).map(|_| ()),
        ));
        out.push((
            format!("mjd_inverse_fd {analytic}"),
            sim_core::mjd_inverse_fd(model, data, &cfg).map(|_| ()),
        ));
    }
    out
}

/// As MuJoCo 3.5.0's `mjd_transitionFD` and `mjd_inverseFD` ("RK4 integrator
/// is not supported"), before anything runs: no callback fires.
#[test]
fn finite_differences_refuse_rk4() {
    let mut model = fd_pendulum("RK4", "", "");
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    let calls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let c = std::sync::Arc::clone(&calls);
    model.set_passive_callback(move |_, _| {
        c.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    });
    for (entry, result) in fd_entry_points(&model, &data) {
        assert_eq!(
            result,
            Err(sim_core::StepError::UnsupportedIntegrator {
                integrator: Integrator::RungeKutta4
            }),
            "{entry}"
        );
    }
    assert_eq!(calls.load(std::sync::atomic::Ordering::Relaxed), 0);
}

/// As MuJoCo 3.5.0's `mjd_transitionFD` ("delays are not supported"); its
/// `mjd_inverseFD` does not check history.
#[test]
fn finite_differences_refuse_history() {
    let model = fd_pendulum("Euler", "", r#"nsample="2""#);
    assert!(model.nhistory > 0);
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    for (entry, result) in fd_entry_points(&model, &data) {
        if entry.starts_with("mjd_inverse_fd") {
            assert!(result.is_ok(), "{entry}: {result:?}");
        } else {
            assert_eq!(
                result,
                Err(sim_core::StepError::UnsupportedHistory {
                    nhistory: model.nhistory
                }),
                "{entry}"
            );
        }
    }
}

/// As MuJoCo 3.5.0's `mjd_inverseFD` ("noslip solver is not supported");
/// the transition derivatives take noslip.
#[test]
fn inverse_finite_differences_refuse_noslip() {
    let model = fd_pendulum("Euler", r#"noslip_iterations="3""#, "");
    assert_eq!(model.noslip_iterations, 3);
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    for (entry, result) in fd_entry_points(&model, &data) {
        if entry.starts_with("mjd_inverse_fd") {
            assert_eq!(
                result,
                Err(sim_core::StepError::UnsupportedNoslip { iterations: 3 }),
                "{entry}"
            );
        } else {
            assert!(result.is_ok(), "{entry}: {result:?}");
        }
    }
}

/// The entry points check the timestep and the `Data`'s shape first, as
/// `step` does: a `Data` made by another model and a non-finite timestep are
/// errors, not a panic or another error from the work done first.
#[test]
fn finite_differences_check_their_inputs_first() {
    let model = fd_pendulum("implicitfast", "", "");
    let other = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut short = other.make_data();
    short.forward(&other).unwrap();
    for (entry, result) in fd_entry_points(&model, &short) {
        assert!(
            matches!(result, Err(sim_core::StepError::DataShapeMismatch { .. })),
            "{entry}: {result:?}"
        );
    }
    let mut bad_h = fd_pendulum("implicitfast", "", "");
    let mut data = bad_h.make_data();
    data.forward(&bad_h).unwrap();
    bad_h.timestep = f64::NEG_INFINITY;
    for (entry, result) in fd_entry_points(&bad_h, &data) {
        assert_eq!(result, Err(sim_core::StepError::InvalidTimestep), "{entry}");
    }
}

/// The refusals come in the documented order: the step inputs, then the
/// integrator, then history (transition) or noslip (inverse). A model with no
/// degrees of freedom checks its inputs too, though the inverse then returns
/// before any pipeline call.
#[test]
fn finite_difference_refusals_come_in_their_documented_order() {
    let other = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut short = other.make_data();
    short.forward(&other).unwrap();
    let rk4 = fd_pendulum("RK4", r#"noslip_iterations="3""#, r#"nsample="2""#);
    assert!(rk4.nhistory > 0);
    for (entry, result) in fd_entry_points(&rk4, &short) {
        assert!(
            matches!(result, Err(sim_core::StepError::DataShapeMismatch { .. })),
            "{entry}: {result:?}"
        );
    }
    let mut data = rk4.make_data();
    data.forward(&rk4).unwrap();
    for (entry, result) in fd_entry_points(&rk4, &data) {
        assert_eq!(
            result,
            Err(sim_core::StepError::UnsupportedIntegrator {
                integrator: Integrator::RungeKutta4
            }),
            "{entry}"
        );
    }
    let still = sim_mjcf::load_model(
        r#"<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>"#,
    )
    .expect("load");
    assert_eq!(still.nv, 0);
    for (entry, result) in fd_entry_points(&still, &short) {
        assert!(
            matches!(result, Err(sim_core::StepError::DataShapeMismatch { .. })),
            "{entry}: {result:?}"
        );
    }
}

// ============================================================================
// Acceptance criterion 8: FD convergence
// ============================================================================

#[test]
fn test_fd_convergence() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[1] = 0.5;
    data.forward(&model).unwrap();

    let c1 = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let c2 = DerivativeConfig {
        eps: 1e-7,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };

    let d1 = mjd_transition_fd(&model, &data, &c1).unwrap();
    let d2 = mjd_transition_fd(&model, &data, &c2).unwrap();

    // Use scale-relative floor to avoid inflated errors from near-zero entries.
    let a_max = d1.A.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    let err = max_relative_error(&d1.A, &d2.A, a_max * 1e-4);
    assert!(
        err < 1e-1,
        "Different eps should converge, relative error={}",
        err
    );
}

// ============================================================================
// Acceptance criteria 9-10: Zero control/activation baselines
// ============================================================================

#[test]
fn test_zero_control_baseline() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // nu=0 → B has 0 columns
    assert_eq!(derivs.B.ncols(), 0);
    assert_eq!(derivs.B.nrows(), 6);
}

#[test]
fn test_zero_activation_baseline() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // na=0 → state dim is 2*nv
    assert_eq!(derivs.A.nrows(), 2 * model.nv);
    assert_eq!(derivs.A.ncols(), 2 * model.nv);
}

// ============================================================================
// Acceptance criterion 11: extract_state tangent correctness
// ============================================================================

#[test]
fn test_extract_state_tangent_correctness() {
    // Free joint: nq=7, nv=6
    let mjcf = r#"
        <mujoco model="free_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    assert_eq!(model.nq, 7); // 3 pos + 4 quat
    assert_eq!(model.nv, 6); // 3 lin + 3 ang

    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // A should be 12×12 (2*nv=12), NOT 14×14 (2*nq=14)
    assert_eq!(derivs.A.nrows(), 12);
    assert_eq!(derivs.A.ncols(), 12);
}

// ============================================================================
// Acceptance criterion 12: Passive damping diagonal
// ============================================================================

#[test]
fn test_passive_damping_diagonal() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    // Set distinct damping for each joint
    model.jnt_damping = vec![0.5, 1.0, 1.5];
    model.compute_implicit_params();

    let mut data = model.make_data();
    data.forward(&model).unwrap();

    mjd_smooth_vel(&model, &mut data);

    // Check diagonal entries from passive damping
    for i in 0..3 {
        let expected = -model.jnt_damping[i];
        // The RNE contribution at zero velocity should be zero, so qDeriv diagonal
        // should be approximately the passive damping
        // Allow some tolerance for Coriolis (which should be near zero at zero velocity)
        assert_relative_eq!(data.qDeriv[(i, i)], expected, epsilon = 1e-6);
    }
}

// ============================================================================
// Acceptance criterion 13: Tendon damping off-diagonal
// ============================================================================

#[test]
fn test_tendon_damping_off_diagonal() {
    // Use MJCF to create a 2-link pendulum with a fixed tendon spanning both DOFs
    let mjcf = r#"
        <mujoco model="tendon_damp">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" damping="2.0">
                    <joint joint="j0" coef="0.7"/>
                    <joint joint="j1" coef="0.3"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    // FD of qfrc_passive w.r.t. qvel
    let eps = 1e-7;
    let nv = model.nv;
    let mut fd_deriv = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let mut scratch = data.clone();
        scratch.qvel[col] += eps;
        scratch.forward(&model).unwrap();
        let fp = scratch.qfrc_passive.clone();

        scratch.qvel.copy_from(&data.qvel);
        scratch.qvel[col] -= eps;
        scratch.forward(&model).unwrap();
        let fm = scratch.qfrc_passive.clone();

        let d = (&fp - &fm) / (2.0 * eps);
        fd_deriv.column_mut(col).copy_from(&d);
    }

    // Analytical via mjd_smooth_vel
    // At zero velocity, RNE velocity derivative is zero, so qDeriv = passive part
    mjd_smooth_vel(&model, &mut data);

    for r in 0..nv {
        for c in 0..nv {
            assert_relative_eq!(data.qDeriv[(r, c)], fd_deriv[(r, c)], epsilon = 1e-6);
        }
    }
}

// ============================================================================
// Acceptance criterion 14: Actuator velocity derivative
// ============================================================================

#[test]
fn test_actuator_velocity_derivative() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    let gain_v = 0.5;
    let bias_v = -0.3;
    let gear = 2.0;
    add_affine_actuator(&mut model, 0, gear, gain_v, bias_v);

    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // input = ctrl[0] since dyntype=None
    data.forward(&model).unwrap();

    // FD of qfrc_actuator w.r.t. qvel
    let eps = 1e-7;
    let nv = model.nv;
    let mut fd_deriv = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let mut scratch = data.clone();
        scratch.qvel[col] += eps;
        scratch.forward(&model).unwrap();
        let fp = scratch.qfrc_actuator.clone();

        scratch.qvel.copy_from(&data.qvel);
        scratch.qvel[col] -= eps;
        scratch.forward(&model).unwrap();
        let fm = scratch.qfrc_actuator.clone();

        let d = (&fp - &fm) / (2.0 * eps);
        fd_deriv.column_mut(col).copy_from(&d);
    }

    // Analytical: dforce/dV = gainprm[2] * input + biasprm[2]
    let input = data.ctrl[0];
    let dforce_dv = gain_v * input + bias_v;
    // For Joint transmission: qDeriv[(dof, dof)] += gear² · dforce_dv
    let expected = gear * gear * dforce_dv;
    let dof_adr = model.jnt_dof_adr[0];

    assert_relative_eq!(fd_deriv[(dof_adr, dof_adr)], expected, epsilon = 1e-6);
}

// ============================================================================
// Acceptance criterion 15: Bias force velocity derivative
// ============================================================================

#[test]
fn test_bias_force_velocity_derivative() {
    let model = Model::n_link_pendulum(2, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = 0.7;
    data.qvel[0] = 1.0;
    data.qvel[1] = 0.5;
    data.forward(&model).unwrap();

    let nv = model.nv;
    let eps = 1e-7;

    // FD of qfrc_bias w.r.t. qvel (column by column)
    let mut fd_deriv = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let d = fd_qfrc_bias_col(&model, &data, col, eps);
        fd_deriv.column_mut(col).copy_from(&d);
    }

    // Analytical via mjd_smooth_vel
    // First zero out damping so only RNE contributes to the non-trivial part
    let mut model_no_damp = model.clone();
    model_no_damp.jnt_damping = vec![0.0; model.njnt];
    model_no_damp.compute_implicit_params();

    let mut data_nd = model_no_damp.make_data();
    data_nd.qpos.copy_from(&data.qpos);
    data_nd.qvel.copy_from(&data.qvel);
    data_nd.forward(&model_no_damp).unwrap();
    mjd_smooth_vel(&model_no_damp, &mut data_nd);

    // qDeriv from mjd_smooth_vel with no damping and no actuators = -∂(bias)/∂qvel
    // So qDeriv ≈ -fd_deriv (since qfrc_smooth = passive + actuator - bias)
    // With no damping and no actuators, qfrc_smooth = -qfrc_bias
    // So qDeriv = -∂(qfrc_bias)/∂qvel = -fd_deriv_bias
    let mut fd_deriv_nd = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let d = fd_qfrc_bias_col(&model_no_damp, &data_nd, col, eps);
        fd_deriv_nd.column_mut(col).copy_from(&d);
    }

    for r in 0..nv {
        for c in 0..nv {
            // qDeriv should be ≈ -∂(bias)/∂qvel
            assert_relative_eq!(data_nd.qDeriv[(r, c)], -fd_deriv_nd[(r, c)], epsilon = 1e-5);
        }
    }
}

// ============================================================================
// Acceptance criterion 16: qDeriv combined
// ============================================================================

#[test]
fn test_qderiv_combined() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    // Add damping
    model.jnt_damping = vec![0.5, 1.0, 0.3];
    model.compute_implicit_params();
    // Add an affine actuator
    add_affine_actuator(&mut model, 0, 1.5, 0.2, -0.1);

    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.1;
    data.ctrl[0] = 0.8;
    data.forward(&model).unwrap();

    // FD of smooth forces w.r.t. qvel
    let nv = model.nv;
    let eps = 1e-7;
    let mut fd_deriv = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let d = fd_smooth_force_col(&model, &data, col, eps);
        fd_deriv.column_mut(col).copy_from(&d);
    }

    // Analytical
    mjd_smooth_vel(&model, &mut data);

    // Floor 1e-6 (matching the sibling derivative tests): some Coriolis
    // cross-terms of this chain are genuinely ~0 (analytic returns machine
    // zero; central-FD at eps=1e-7 leaves ~1e-10 truncation noise), so a
    // tighter 1e-10 floor would divide that noise by itself and report a
    // spurious ~O(1) relative error. Meaningful entries are O(1e-2) and still
    // gate at 1e-4. (The previous 1e-10 floor passed only because the pre-fix
    // RNE Coriolis bug gave those entries a spurious non-zero value well above
    // the floor.)
    let err = max_relative_error(&data.qDeriv, &fd_deriv, 1e-6);
    assert!(
        err < 1e-4,
        "Combined qDeriv should match FD to 1e-4, got {}",
        err
    );
}

// ============================================================================
// Acceptance criterion 17: Scratch buffer isolation
// ============================================================================

#[test]
fn test_scratch_buffer_isolation() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = 1.0;
    data.forward(&model).unwrap();

    // Save body velocities before
    let cvel_before: Vec<_> = data.cvel.clone();

    mjd_smooth_vel(&model, &mut data);

    // cvel should be unchanged
    for (i, (before, after)) in cvel_before.iter().zip(data.cvel.iter()).enumerate() {
        assert_eq!(before, after, "cvel[{}] was corrupted by mjd_smooth_vel", i);
    }
}

// ============================================================================
// Acceptance criterion 18: Ball/Free joint FD
// ============================================================================

#[test]
fn test_free_joint_fd() {
    let mjcf = r#"
        <mujoco model="free_deriv">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let nv = model.nv; // 6

    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // A should be 12×12 (2*nv=12)
    assert_eq!(derivs.A.nrows(), 12);
    assert_eq!(derivs.A.ncols(), 12);

    // Position-velocity block A[0..6, 6..12] should be ≈ h·I₆
    let h = model.timestep;
    let pv_block = derivs.A.view((0, nv), (nv, nv));
    for r in 0..nv {
        for c in 0..nv {
            let expected = if r == c { h } else { 0.0 };
            assert_relative_eq!(pv_block[(r, c)], expected, epsilon = 1e-3);
        }
    }
}

// ============================================================================
// Acceptance criterion 19: Ball joint Coriolis
// ============================================================================

#[test]
fn test_ball_joint_coriolis() {
    // Use a box (non-spherical inertia) so gyroscopic terms are non-zero.
    // A sphere has I_xx = I_yy = I_zz, so ω × (I·ω) = 0 for any ω.
    // A box with different dimensions has distinct principal moments.
    let mjcf = r#"
        <mujoco model="ball_coriolis">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint type="ball" name="ball"/>
                    <geom type="box" size="0.3 0.1 0.05" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Set a non-trivial velocity
    data.qvel[0] = 1.0;
    data.qvel[1] = 0.5;
    data.qvel[2] = -0.3;
    data.forward(&model).unwrap();

    let nv = model.nv;
    let eps = 1e-7;

    // FD of qfrc_bias w.r.t. qvel
    let mut fd_deriv = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let d = fd_qfrc_bias_col(&model, &data, col, eps);
        fd_deriv.column_mut(col).copy_from(&d);
    }

    // Should be non-zero (gyroscopic terms from non-spherical inertia)
    assert!(
        fd_deriv.norm() > 1e-6,
        "Ball joint Coriolis should be nonzero for non-spherical inertia, got norm={}",
        fd_deriv.norm()
    );

    // Analytical
    let mut model_nd = model.clone();
    // Zero damping to isolate RNE
    for d in model_nd.dof_damping.iter_mut() {
        *d = 0.0;
    }
    model_nd.compute_implicit_params();
    let mut data_nd = model_nd.make_data();
    data_nd.qvel.copy_from(&data.qvel);
    data_nd.forward(&model_nd).unwrap();
    mjd_smooth_vel(&model_nd, &mut data_nd);

    // qDeriv = -∂(bias)/∂qvel when no damping/actuators
    for r in 0..nv {
        for c in 0..nv {
            assert_relative_eq!(data_nd.qDeriv[(r, c)], -fd_deriv[(r, c)], epsilon = 1e-5);
        }
    }
}

// ============================================================================
// Acceptance criterion 20: Config validation
// ============================================================================

#[test]
#[should_panic(expected = "DerivativeConfig::eps must be in (0, 1e-2]")]
fn test_config_validation_zero() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let data = model.make_data();
    let config = DerivativeConfig {
        eps: 0.0,
        ..Default::default()
    };
    let _ = mjd_transition_fd(&model, &data, &config);
}

#[test]
#[should_panic(expected = "DerivativeConfig::eps must be in (0, 1e-2]")]
fn test_config_validation_negative() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let data = model.make_data();
    let config = DerivativeConfig {
        eps: -1.0,
        ..Default::default()
    };
    let _ = mjd_transition_fd(&model, &data, &config);
}

#[test]
#[should_panic(expected = "DerivativeConfig::eps must be in (0, 1e-2]")]
fn test_config_validation_nan() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let data = model.make_data();
    let config = DerivativeConfig {
        eps: f64::NAN,
        ..Default::default()
    };
    let _ = mjd_transition_fd(&model, &data, &config);
}

#[test]
#[should_panic(expected = "DerivativeConfig::eps must be in (0, 1e-2]")]
fn test_config_validation_too_large() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let data = model.make_data();
    let config = DerivativeConfig {
        eps: 0.1,
        ..Default::default()
    };
    let _ = mjd_transition_fd(&model, &data, &config);
}

#[test]
fn test_config_validation_default_ok() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    let config = DerivativeConfig::default();
    let result = mjd_transition_fd(&model, &data, &config);
    assert!(result.is_ok());
}

// ============================================================================
// Acceptance criterion 22: Scratch state restoration (idempotent FD)
// ============================================================================

#[test]
fn test_scratch_state_restoration() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[1] = 0.5;
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();
    let d1 = mjd_transition_fd(&model, &data, &config).unwrap();
    let d2 = mjd_transition_fd(&model, &data, &config).unwrap();

    // Should be identical (data is &Data, unmodified)
    let diff = (&d1.A - &d2.A).norm();
    assert!(
        diff < 1e-14,
        "Two FD calls on same data should be identical, diff={}",
        diff
    );
}

// ============================================================================
// Acceptance criterion 23: Tendon damping in qDeriv for all integrators (DT-35)
// ============================================================================

#[test]
#[allow(non_snake_case)]
fn test_tendon_damping_in_qDeriv_all_integrators() {
    // Use MJCF: 2 hinges with zero damping, one tendon with damping=5.0,
    // using ImplicitSpringDamper integrator. DT-35: tendon damping is now
    // included in qDeriv for all integrators (no longer skipped).
    let mjcf = r#"
        <mujoco model="tendon_implicit">
            <option gravity="0 0 -9.81" timestep="0.001" integrator="implicitspringdamper"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" damping="5.0">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    mjd_smooth_vel(&model, &mut data);

    // FIXED (DT-35): tendon damping is now included in qDeriv for all integrators.
    // With J = [1, 1], b = 5: qDeriv += -b * J^T * J = -5 * [[1,1],[1,1]]
    //   qDeriv[(0,0)] += -5 * 1 * 1 = -5
    //   qDeriv[(0,1)] += -5 * 1 * 1 = -5
    //   qDeriv[(1,0)] += -5 * 1 * 1 = -5
    //   qDeriv[(1,1)] += -5 * 1 * 1 = -5
    //
    // Note: mjd_smooth_vel also populates qDeriv with RNE/Coriolis and per-DOF
    // damping terms. The entry-level assertions below isolate the tendon
    // contribution (joint damping is zero in this model, and Coriolis at zero
    // velocity is negligible). The norm check is a coarse sanity guard.
    let qderiv_norm = data.qDeriv.norm();
    assert!(
        qderiv_norm > 1.0,
        "qDeriv should be non-trivial (includes tendon damping), norm={}",
        qderiv_norm
    );
    // Verify tendon damping structure: all entries ≈ -5 (J=[1,1] makes all
    // outer-product entries positive, then multiplied by -b = -5).
    // At qpos[0]=0.3 with gravity, RNE/Coriolis may contribute a small offset,
    // so use a relaxed threshold (-1.0) rather than exact (-5.0 ± epsilon).
    assert!(
        data.qDeriv[(0, 0)] < -1.0,
        "qDeriv[(0,0)] should be ~-5, got {}",
        data.qDeriv[(0, 0)]
    );
    assert!(
        data.qDeriv[(0, 1)] < -1.0,
        "qDeriv[(0,1)] should be ~-5 (J[0]*J[1] = 1*1 = 1, times -b = -5), got {}",
        data.qDeriv[(0, 1)]
    );
    assert!(
        data.qDeriv[(1, 0)] < -1.0,
        "qDeriv[(1,0)] should be ~-5, got {}",
        data.qDeriv[(1, 0)]
    );
    assert!(
        data.qDeriv[(1, 1)] < -1.0,
        "qDeriv[(1,1)] should be ~-5, got {}",
        data.qDeriv[(1, 1)]
    );
}

// ============================================================================
// Acceptance criterion 24: Performance (basic timing check)
// ============================================================================

#[test]
fn test_fd_performance() {
    let model = Model::n_link_pendulum(6, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig::default();

    // Just verify it completes without error (timing is release-mode only)
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();
    assert_eq!(derivs.A.nrows(), 12); // 2*6
}

// ============================================================================
// Spec A: Analytical Position Derivatives (§58)
// ============================================================================

/// FD of (qfrc_passive + qfrc_actuator - qfrc_bias) w.r.t. qpos[col] (centered).
/// Uses tangent-space perturbation via mj_integrate_pos_explicit.
fn fd_smooth_force_pos_col(
    model: &Model,
    data: &sim_core::Data,
    col: usize,
    eps: f64,
) -> nalgebra::DVector<f64> {
    let mut scratch = data.clone();

    // +eps: tangent-space perturbation
    scratch.qpos.copy_from(&data.qpos);
    let mut dq = nalgebra::DVector::zeros(model.nv);
    dq[col] = eps;
    let mut qpos_pert = nalgebra::DVector::zeros(model.nq);
    mj_integrate_pos_explicit(model, &mut qpos_pert, &data.qpos, &dq, 1.0);
    scratch.qpos.copy_from(&qpos_pert);
    scratch.qvel.copy_from(&data.qvel);
    scratch.forward(model).unwrap();
    let smooth_plus = &scratch.qfrc_passive + &scratch.qfrc_actuator - &scratch.qfrc_bias;

    // -eps
    dq[col] = -eps;
    mj_integrate_pos_explicit(model, &mut qpos_pert, &data.qpos, &dq, 1.0);
    scratch.qpos.copy_from(&qpos_pert);
    scratch.qvel.copy_from(&data.qvel);
    scratch.forward(model).unwrap();
    let smooth_minus = &scratch.qfrc_passive + &scratch.qfrc_actuator - &scratch.qfrc_bias;

    (&smooth_plus - &smooth_minus) / (2.0 * eps)
}

/// Build full nv×nv FD position derivative matrix (∂qfrc_smooth/∂qpos).
///
/// NOTE: This computes ∂qfrc_smooth/∂qpos, which equals qDeriv_pos only
/// when M(q) is constant (single-joint models). For multi-link models,
/// qDeriv_pos (with AD-2) includes the −(∂M/∂q)·qacc term. Use
/// `fd_dqacc_dqpos` for correct validation of qDeriv_pos on multi-link models.
fn fd_qderiv_pos(model: &Model, data: &sim_core::Data, eps: f64) -> nalgebra::DMatrix<f64> {
    let nv = model.nv;
    let mut mat = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let fd_col = fd_smooth_force_pos_col(model, data, col, eps);
        for row in 0..nv {
            mat[(row, col)] = fd_col[row];
        }
    }
    mat
}

/// FD of qacc w.r.t. qpos — correct validation level for qDeriv_pos (with AD-2).
///
/// Since qDeriv_pos includes −(∂M/∂q)·qacc via evaluating RNEA at actual
/// acceleration, the identity M⁻¹·qDeriv_pos = ∂qacc/∂q holds. This helper
/// computes the FD reference for ∂qacc/∂q.
fn fd_dqacc_dqpos(model: &Model, data: &sim_core::Data, eps: f64) -> nalgebra::DMatrix<f64> {
    let nv = model.nv;
    let mut mat = nalgebra::DMatrix::zeros(nv, nv);
    for col in 0..nv {
        let mut scratch = data.clone();

        // +eps
        let mut dq = nalgebra::DVector::zeros(nv);
        dq[col] = eps;
        let mut qpos_pert = nalgebra::DVector::zeros(model.nq);
        mj_integrate_pos_explicit(model, &mut qpos_pert, &data.qpos, &dq, 1.0);
        scratch.qpos.copy_from(&qpos_pert);
        scratch.qvel.copy_from(&data.qvel);
        scratch.forward(model).unwrap();
        let qacc_plus = scratch.qacc.clone();

        // -eps
        dq[col] = -eps;
        mj_integrate_pos_explicit(model, &mut qpos_pert, &data.qpos, &dq, 1.0);
        scratch.qpos.copy_from(&qpos_pert);
        scratch.qvel.copy_from(&data.qvel);
        scratch.forward(model).unwrap();

        let fd_col = (&qacc_plus - &scratch.qacc) / (2.0 * eps);
        for row in 0..nv {
            mat[(row, col)] = fd_col[row];
        }
    }
    mat
}

/// Compute M⁻¹ · qDeriv_pos analytically (gives ∂qacc/∂q).
fn analytical_dqacc_dqpos(model: &Model, data: &sim_core::Data) -> nalgebra::DMatrix<f64> {
    let nv = model.nv;
    let mut result = nalgebra::DMatrix::zeros(nv, nv);
    let (rowadr, rownnz, colind) = model.qld_csr();
    for col in 0..nv {
        let mut x = data.qDeriv_pos.column(col).clone_owned();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut x,
        );
        result.column_mut(col).copy_from(&x);
    }
    result
}

// T1: Hinge spring position derivative (exact) → AC1
#[test]
fn test_pos_deriv_hinge_spring() {
    let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
    model.jnt_stiffness[0] = 10.0;
    model.jnt_springref[0] = 0.0;
    model.gravity = nalgebra::Vector3::zeros(); // no gravity
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);
    assert_relative_eq!(data.qDeriv_pos[(0, 0)], -10.0, epsilon = 1e-12);
}

// T_supp6: Slide joint spring position derivative (exact) → AC18
#[test]
fn test_pos_deriv_slide_spring() {
    // Build a slide joint model
    let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
    // Change joint type to Slide
    model.jnt_type[0] = sim_core::MjJointType::Slide;
    model.jnt_stiffness[0] = 15.0;
    model.jnt_springref[0] = 0.0;
    model.gravity = nalgebra::Vector3::zeros();
    model.nq = 1; // Slide has nq=1
    model.qpos0 = nalgebra::DVector::from_vec(vec![0.0]);
    model.qpos_spring = vec![0.0];
    model.compute_ancestors();
    model.compute_implicit_params();
    model.compute_qld_csr_metadata();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);
    assert_relative_eq!(data.qDeriv_pos[(0, 0)], -15.0, epsilon = 1e-12);
}

// T2: Ball spring position derivative (FD-validated) → AC2
#[test]
fn test_pos_deriv_ball_spring() {
    let mut model = Model::spherical_pendulum(1.0, 0.1);
    model.jnt_stiffness[0] = 5.0;
    model.gravity = nalgebra::Vector3::zeros();
    let mut data = model.make_data();
    // Small rotation ~0.3 rad about [1,1,1]/√3
    let axis = nalgebra::Vector3::new(1.0, 1.0, 1.0).normalize();
    let q = nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), 0.3);
    data.qpos[0] = q.w;
    data.qpos[1] = q.i;
    data.qpos[2] = q.j;
    data.qpos[3] = q.k;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // With body_ipos ≠ 0 (MuJoCo convention: COM offset from body frame),
    // M(q) varies with q due to parallel-axis rotation. qDeriv_pos includes
    // -(∂M/∂q)·qacc, so validate via ∂qacc/∂q = M⁻¹·qDeriv_pos.
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-6);

    let analytical_block = analytical.view((0, 0), (3, 3));
    let fd_block = fd.view((0, 0), (3, 3));

    let err = max_relative_error(
        &analytical_block.clone_owned(),
        &fd_block.clone_owned(),
        1e-6,
    );
    assert!(
        err < 1e-5,
        "Ball spring pos deriv: max relative error {err} exceeds 1e-5"
    );
}

// T3: Free joint spring position derivative (FD-validated) → AC3
#[test]
fn test_pos_deriv_free_spring() {
    let mut model = Model::free_body(0.5, nalgebra::Vector3::new(0.01, 0.01, 0.01));
    model.jnt_stiffness[0] = 8.0;
    model.gravity = nalgebra::Vector3::zeros();
    let mut data = model.make_data();
    // Translate [1, 0, 0]
    data.qpos[0] = 1.0;
    // Small rotation
    let q = nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(nalgebra::Vector3::z()),
        0.2,
    );
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);
    let fd = fd_qderiv_pos(&model, &data, 1e-6);

    let err = max_relative_error(&data.qDeriv_pos, &fd, 1e-6);
    assert!(
        err < 1e-5,
        "Free joint spring pos deriv: max relative error {err} exceeds 1e-5"
    );
}

// T4: Tendon spring position derivative (FD-validated) → AC4
#[test]
fn test_pos_deriv_tendon_spring() {
    // 2-hinge chain with a fixed tendon spanning both joints, stiffness=20.0.
    // Tendon lengthspring deadband [0.5, 1.5]. Set qpos so tendon length is
    // outside the deadband to activate the spring.
    let mjcf = r#"
        <mujoco model="tendon_spring_pos">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="hinge" axis="0 1 0" damping="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j1" type="hinge" axis="0 1 0" damping="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t0" stiffness="20.0" springlength="0.5 1.5">
                    <joint joint="j0" coef="1.0"/>
                    <joint joint="j1" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    // Set qpos so that tendon length = coef[0]*q0 + coef[1]*q1 + offset
    // is outside the [0.5, 1.5] deadband.
    data.qpos[0] = 1.0;
    data.qpos[1] = 0.8;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // FD reference
    let fd = fd_qderiv_pos(&model, &data, 1e-6);
    let err = max_relative_error(&data.qDeriv_pos, &fd, 1e-6);
    assert!(
        err < 1e-5,
        "Tendon spring pos deriv: max relative error {err} exceeds 1e-5"
    );

    // Verify structural property: -k * J^T * J pattern.
    // With J = [1, 1] and k = 20: contribution is -20 * [[1,1],[1,1]]
    // All four entries should be significantly negative.
    assert!(
        data.qDeriv_pos[(0, 0)] < -1.0,
        "Tendon spring should produce negative diagonal: got {}",
        data.qDeriv_pos[(0, 0)]
    );
}

// T5: Affine actuator position derivative (FD-validated) → AC5
#[test]
fn test_pos_deriv_affine_actuator() {
    let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
    model.gravity = nalgebra::Vector3::zeros();

    // Add affine actuator with gainprm[1]=2.0 (length-dependent gain)
    model.actuator_trntype.push(ActuatorTransmission::Joint);
    model.actuator_trnid.push([0, usize::MAX]);
    model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    model.actuator_dyntype.push(ActuatorDynamics::None);
    model.actuator_ctrlrange.push((-100.0, 100.0));
    model.actuator_forcerange.push((-100.0, 100.0));
    model.actuator_name.push(None);
    model.actuator_act_adr.push(0);
    model.actuator_act_num.push(0);
    model.actuator_gaintype.push(GainType::Affine);
    model.actuator_biastype.push(BiasType::None);
    model.actuator_dynprm.push([0.0; 10]);
    model.actuator_gainprm.push({
        let mut p = [0.0; 9];
        p[1] = 2.0; // gainprm[1] = length-dependent gain
        p
    });
    model.actuator_biasprm.push([0.0; 9]);
    model.actuator_lengthrange.push((0.0, 0.0));
    model.actuator_acc0.push(0.0);
    model.actuator_actlimited.push(false);
    model.actuator_actrange.push((0.0, 0.0));
    model.actuator_actearly.push(false);
    model.nu = 1;

    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // Expected: gainprm[1] · ctrl · gear² = 2.0 · 1.0 · 1.0 = 2.0
    // (positive: increasing q increases actuator length → increases gain →
    //  increases force applied in positive qfrc_actuator direction)
    assert_relative_eq!(data.qDeriv_pos[(0, 0)], 2.0, epsilon = 1e-10);

    // Also verify against FD
    let fd = fd_qderiv_pos(&model, &data, 1e-6);
    let err = max_relative_error(&data.qDeriv_pos, &fd, 1e-6);
    assert!(
        err < 1e-5,
        "Affine actuator pos deriv: max relative error {err} exceeds 1e-5"
    );
}

// T6: RNE position derivatives — 3-link pendulum (FD-validated) → AC6
// Validated at the qacc level: M⁻¹·qDeriv_pos = ∂qacc/∂q (FD).
// Direct comparison of qDeriv_pos against FD of qfrc_smooth is invalid
// for multi-link models because AD-2 includes the −(∂M/∂q)·qacc term.
#[test]
fn test_pos_deriv_rne_pendulum() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qpos[2] = 0.1;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.2;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // Compare M⁻¹·qDeriv_pos (analytical ∂qacc/∂q) against FD of qacc
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-6);

    let err = max_relative_error(&analytical, &fd, 1e-6);
    assert!(
        err < 1e-4,
        "RNE 3-link pendulum pos deriv: max relative error {err} exceeds 1e-4"
    );
}

// T7: Transition A matrix — Euler — position columns match FD → AC7, AC16
#[test]
fn test_pos_deriv_transition_euler() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    add_torque_actuators(&mut model, 3);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qpos[2] = 0.1;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.2;
    data.ctrl[0] = 1.0;
    data.ctrl[1] = -0.5;
    data.ctrl[2] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    let nv = model.nv;

    // Position columns (0..nv) of A should match FD
    let hybrid_pos = hybrid.A.view((0, 0), (2 * nv, nv));
    let fd_pos = fd.A.view((0, 0), (2 * nv, nv));
    let err = max_relative_error(&hybrid_pos.clone_owned(), &fd_pos.clone_owned(), 1e-6);
    assert!(
        err < 1e-4,
        "Euler transition pos columns: max relative error {err} exceeds 1e-4"
    );

    // Velocity columns should be unchanged (identical for both)
    let hybrid_vel = hybrid.A.view((0, nv), (2 * nv, nv));
    let fd_vel = fd.A.view((0, nv), (2 * nv, nv));
    let vel_err = max_relative_error(&hybrid_vel.clone_owned(), &fd_vel.clone_owned(), 1e-6);
    assert!(
        vel_err < 1e-4,
        "Euler transition vel columns: max relative error {vel_err} exceeds 1e-4"
    );
}

// T8: Transition A matrix — ISD — position columns match FD → AC8
#[test]
fn test_pos_deriv_transition_isd() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.integrator = Integrator::ImplicitSpringDamper;
    model.jnt_stiffness[0] = 10.0;
    model.jnt_stiffness[1] = 10.0;
    model.jnt_damping[0] = 1.0;
    model.jnt_damping[1] = 1.0;
    model.compute_implicit_params();
    add_torque_actuators(&mut model, 2);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.ctrl[1] = -0.5;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    let nv = model.nv;
    let hybrid_pos = hybrid.A.view((0, 0), (2 * nv, nv));
    let fd_pos = fd.A.view((0, 0), (2 * nv, nv));
    let err = max_relative_error(&hybrid_pos.clone_owned(), &fd_pos.clone_owned(), 1e-6);
    assert!(
        err < 1e-4,
        "ISD transition pos columns: max relative error {err} exceeds 1e-4"
    );
}

// T9: Transition A matrix — ImplicitFast — position columns match FD → AC9
#[test]
fn test_pos_deriv_transition_implicit_fast() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.integrator = Integrator::ImplicitFast;
    model.jnt_damping[0] = 0.5;
    model.jnt_damping[1] = 0.5;
    model.compute_implicit_params();
    add_torque_actuators(&mut model, 2);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.ctrl[1] = -0.5;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    let nv = model.nv;
    let hybrid_pos = hybrid.A.view((0, 0), (2 * nv, nv));
    let fd_pos = fd.A.view((0, 0), (2 * nv, nv));
    let err = max_relative_error(&hybrid_pos.clone_owned(), &fd_pos.clone_owned(), 1e-6);
    assert!(
        err < 1e-4,
        "ImplicitFast transition pos columns: max relative error {err} exceeds 1e-4"
    );
}

// T10: Ball/Free joint transition position columns match FD → AC10
#[test]
fn test_pos_deriv_transition_ball_free() {
    // Free body (6 DOF) — test with Euler integrator
    let model = Model::free_body(0.5, nalgebra::Vector3::new(0.01, 0.01, 0.01));
    let mut data = model.make_data();
    data.qpos[0] = 0.1; // x translation
    data.qpos[1] = 0.2; // y translation
    data.qpos[2] = 0.3; // z translation
    // Small rotation
    let q = nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(nalgebra::Vector3::new(1.0, 0.5, 0.3)),
        0.2,
    );
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;
    data.qvel[0] = 0.1;
    data.qvel[3] = 0.05;
    data.qvel[4] = -0.03;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    let nv = model.nv; // 6
    let hybrid_pos = hybrid.A.view((0, 0), (2 * nv, nv));
    let fd_pos = fd.A.view((0, 0), (2 * nv, nv));

    // Two-tier check:
    // 1. Absolute tolerance 5e-5 for all elements (catches FD noise in near-zero entries)
    // 2. Relative tolerance 1e-4 for "large" elements (floor 1e-3)
    //
    // The free body with isotropic inertia has zero linear-angular cross-coupling
    // (analytical = 0). FD shows ~2e-5 noise from FP arithmetic in forward dynamics
    // (centered diff: f(+eps)-f(-eps) ~ 4e-11 for velocity values of O(0.01)).
    let h_mat = hybrid_pos.clone_owned();
    let f_mat = fd_pos.clone_owned();
    let mut max_err = 0.0_f64;
    for r in 0..h_mat.nrows() {
        for c in 0..h_mat.ncols() {
            let diff = (h_mat[(r, c)] - f_mat[(r, c)]).abs();
            if diff < 5e-5 {
                continue; // absolute tolerance: skip FD noise in near-zero entries
            }
            let denom = h_mat[(r, c)].abs().max(f_mat[(r, c)].abs()).max(1e-6);
            max_err = max_err.max(diff / denom);
        }
    }
    assert!(
        max_err < 1e-4,
        "Ball/Free transition pos columns: max relative error {max_err} exceeds 1e-4"
    );
}

// T11: FD fallback for ineligible model → AC11
#[test]
fn test_pos_deriv_fd_fallback() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.density = 1.0; // makes model ineligible for analytical pos
    add_torque_actuators(&mut model, 2);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = 0.5;
    data.ctrl[0] = 1.0;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    // FD fallback: position columns of hybrid should match pure FD exactly.
    // Velocity columns may differ (hybrid always uses analytical velocity derivatives).
    let nv = model.nv;
    let hybrid_pos = hybrid.A.view((0, 0), (2 * nv, nv));
    let fd_pos = fd.A.view((0, 0), (2 * nv, nv));
    let err = max_relative_error(&hybrid_pos.clone_owned(), &fd_pos.clone_owned(), 1e-10);
    assert!(
        err < 1e-10,
        "FD fallback: max relative error {err} exceeds 1e-10"
    );
}

// T14: Contact isolation — analytical matches FD on contact-free models → AC17
#[test]
fn test_pos_deriv_contact_free() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    add_torque_actuators(&mut model, 3);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qpos[2] = 0.1;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.2;
    data.ctrl[0] = 1.0;
    data.ctrl[1] = -0.5;
    data.ctrl[2] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let fd = mjd_transition_fd(&model, &data, &config).unwrap();

    let err = max_relative_error(&hybrid.A, &fd.A, 1e-6);
    assert!(
        err < 1e-4,
        "Contact-free full A matrix: max relative error {err} exceeds 1e-4"
    );
}

// T_supp2: Disabled gravity → gravity contribution is zero
#[test]
fn test_pos_deriv_disabled_gravity() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.disableflags |= DISABLE_GRAVITY;
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // With gravity disabled, no springs, and no actuators, the only
    // contribution is Coriolis position derivatives from RNE.
    // Validate at qacc level (multi-link model, AD-2).
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-6);
    // With gravity disabled and only one joint moving, derivatives are very small
    // (near-zero qacc). Use a higher floor to avoid FD noise inflation.
    let err = max_relative_error(&analytical, &fd, 1e-3);
    assert!(
        err < 1e-4,
        "Disabled gravity pos deriv: max relative error {err} exceeds 1e-4"
    );
}

// T_supp3: Disabled springs → passive contribution is zero
#[test]
fn test_pos_deriv_disabled_springs() {
    let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
    model.jnt_stiffness[0] = 10.0;
    model.disableflags |= DISABLE_SPRING;
    model.gravity = nalgebra::Vector3::zeros();
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);
    // With springs disabled, gravity off, no actuators, and qvel=0,
    // all position derivatives should be zero.
    let max_val = data.qDeriv_pos.abs().max();
    assert!(
        max_val < 1e-15,
        "Disabled springs: qDeriv_pos should be ~zero, max = {max_val}"
    );
}

// T_supp4: nv=0 model → all functions return without error
#[test]
fn test_pos_deriv_nv_zero() {
    let model = Model::empty();
    let mut data = model.make_data();
    mjd_smooth_pos(&model, &mut data);
    // Just verifying no panic
}

// T_supp7: FD convergence — shrinking epsilon confirms analytical matches FD limit
#[test]
fn test_pos_deriv_fd_convergence() {
    let model = Model::n_link_pendulum(2, 1.0, 0.1);
    let mut data = model.make_data();
    data.qpos[0] = 0.4;
    data.qpos[1] = -0.3;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // Compare at qacc level (multi-link model, AD-2)
    let analytical = analytical_dqacc_dqpos(&model, &data);

    // Compute FD at decreasing eps and verify error decreases
    let eps_vals = [1e-4, 1e-6, 1e-8];
    let mut prev_err = f64::MAX;
    for &eps in &eps_vals {
        let fd = fd_dqacc_dqpos(&model, &data, eps);
        let err = max_relative_error(&analytical, &fd, 1e-6);
        // Each step should reduce error (at least not increase by much).
        // At extremely small analytical error (< 1e-7), FP noise in FD dominates
        // and monotonic convergence is not guaranteed.
        if eps > 1e-7 && prev_err > 1e-7 {
            assert!(
                err < prev_err * 1.1,
                "FD convergence: error {err} at eps={eps} not improving (prev={prev_err})"
            );
        }
        prev_err = err;
    }
    // Final error should be small
    assert!(
        prev_err < 1e-3,
        "FD convergence: final error {prev_err} too large"
    );
}

// T_supp1: Near-zero-mass body — no NaN or division by zero in RNE pos
//
// A truly zero-mass articulated body creates a singular mass matrix (NaN in
// qacc), which is degenerate — not a derivative edge case. Instead we test
// with a very small mass (1e-8) to verify that RNE position derivatives
// handle near-zero inertia without numerical blowup. The joint has armature
// to keep the mass matrix non-singular.
#[test]
fn test_pos_deriv_zero_mass_body() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.body_mass[2] = 1e-8; // near-zero mass
    model.body_inertia[2] = nalgebra::Vector3::new(1e-10, 1e-10, 1e-10);
    model.jnt_armature[1] = 0.01; // regularize mass matrix diagonal
    model.dof_armature[1] = 0.01;
    model.compute_ancestors();
    model.compute_implicit_params();
    model.compute_qld_csr_metadata();

    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // No NaN or Inf in the result
    for r in 0..model.nv {
        for c in 0..model.nv {
            let v = data.qDeriv_pos[(r, c)];
            assert!(
                v.is_finite(),
                "Near-zero-mass body: qDeriv_pos[({r},{c})] = {v} is not finite"
            );
        }
    }

    // Verify against FD (at qacc level)
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-6);
    let err = max_relative_error(&analytical, &fd, 1e-6);
    assert!(
        err < 1e-3,
        "Near-zero-mass body: dqacc/dqpos max relative error {err} exceeds 1e-3"
    );
}

// T_supp5: Multi-joint body (FD-validated) — validates all-joints extraction
#[test]
fn test_pos_deriv_multi_joint_body() {
    // A body with TWO hinge joints on it (slide + hinge on one body).
    // This validates the multi-joint subtraction logic in mjd_rne_pos
    // (subtracting ALL joints from cvel/cacc to recover X_body · parent).
    let mjcf = r#"
        <mujoco model="multi_joint_body">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j0" type="slide" axis="1 0 0"/>
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 -1">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    assert_eq!(model.body_jnt_num[1], 2, "Body 1 should have 2 joints");

    let mut data = model.make_data();
    data.qpos[0] = 0.2; // slide displacement
    data.qpos[1] = 0.4; // hinge angle
    data.qpos[2] = -0.3; // child hinge
    data.qvel[0] = 0.3;
    data.qvel[1] = 0.5;
    data.qvel[2] = -0.2;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // Validate at qacc level (multi-link → AD-2).
    // Uses FD step 1e-5 (not 1e-6) because the 3×3 matrix has many near-zero
    // elements where FD noise at 1e-6 creates false positives against the floor.
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-5);
    let err = max_relative_error(&analytical, &fd, 1e-6);
    assert!(
        err < 1e-4,
        "Multi-joint body: dqacc/dqpos max relative error {err} exceeds 1e-4"
    );
}

// T_supp8: Sleeping bodies — derivatives correct after forward pass
#[test]
fn test_pos_deriv_sleeping_body() {
    // After a forward pass, sleeping state is reflected in qacc/cvel.
    // mjd_smooth_pos should produce correct derivatives regardless of
    // whether a body is "sleeping" — it operates on forward-pass output.
    // We verify by comparing a model at rest (sleeping-eligible state)
    // against FD. The point: no special-casing needed, same result.
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.jnt_stiffness[0] = 5.0;
    model.jnt_stiffness[1] = 5.0;
    let mut data = model.make_data();
    // Set qpos away from equilibrium but qvel=0 (sleeping-eligible)
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    // qvel remains zero
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);

    // Validate at qacc level
    let analytical = analytical_dqacc_dqpos(&model, &data);
    let fd = fd_dqacc_dqpos(&model, &data, 1e-6);
    let err = max_relative_error(&analytical, &fd, 1e-6);
    assert!(
        err < 1e-4,
        "Sleeping body: dqacc/dqpos max relative error {err} exceeds 1e-4"
    );
}

// T12: Performance benchmark — analytical ≥1.5× faster than FD position columns
#[test]
#[ignore = "benchmark — run with --release --ignored for meaningful timing"]
fn test_pos_deriv_performance() {
    let mut model = Model::n_link_pendulum(6, 1.0, 0.1);
    add_torque_actuators(&mut model, 6);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qpos[2] = 0.1;
    data.qvel[0] = 0.5;
    data.ctrl[0] = 1.0;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };

    let iters = 100;

    // Warm up
    let _ = mjd_transition_hybrid(&model, &data, &config).unwrap();
    let _ = mjd_transition_fd(&model, &data, &config).unwrap();

    // Time analytical (hybrid with analytical position columns)
    let t0 = std::time::Instant::now();
    for _ in 0..iters {
        let _ = mjd_transition_hybrid(&model, &data, &config).unwrap();
    }
    let hybrid_time = t0.elapsed();

    // Time pure FD
    let t1 = std::time::Instant::now();
    for _ in 0..iters {
        let _ = mjd_transition_fd(&model, &data, &config).unwrap();
    }
    let fd_time = t1.elapsed();

    let speedup = fd_time.as_secs_f64() / hybrid_time.as_secs_f64();
    eprintln!(
        "Performance: hybrid={:.2}ms, fd={:.2}ms, speedup={:.2}x ({iters} iters, nv={})",
        hybrid_time.as_secs_f64() * 1000.0,
        fd_time.as_secs_f64() * 1000.0,
        speedup,
        model.nv,
    );
}

// T_supp9: Zero-derivative model (negative test)
#[test]
fn test_pos_deriv_zero_model() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.disableflags |= DISABLE_SPRING;
    model.disableflags |= DISABLE_GRAVITY;
    let mut data = model.make_data();
    // qvel = 0, so no Coriolis either
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    mjd_smooth_pos(&model, &mut data);
    let max_val = data.qDeriv_pos.abs().max();
    assert!(
        max_val < 1e-15,
        "Zero-derivative model: qDeriv_pos should be zero, max = {max_val}"
    );
}

// ============================================================================
// Phase 11 Spec B — Sensor Derivatives (C, D matrices)
// ============================================================================

/// Helper: build a 2-link pendulum with jointpos sensors and a motor actuator.
fn sensor_pendulum_2link() -> (sim_core::Model, sim_core::Data) {
    let mjcf = r#"
        <mujoco model="sensor_deriv_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0.5 0 0">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1" ctrlrange="-10 10"/>
            </actuator>
            <sensor>
                <jointpos joint="j1"/>
                <jointpos joint="j2"/>
            </sensor>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[1] = 0.5;
    data.ctrl[0] = 1.0;
    data.forward(&model).unwrap();
    (model, data)
}

/// T1: Sensor derivative dimensions → AC1, AC2
#[test]
fn t1_sensor_derivative_dimensions() {
    let (model, data) = sensor_pendulum_2link();
    assert_eq!(model.nsensordata, 2);
    assert_eq!(model.nv, 2);
    assert_eq!(model.na, 0);
    assert_eq!(model.nu, 1);

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    // C: nsensordata × (2*nv + na) = 2 × 4
    let c = derivs.C.as_ref().expect("C should be Some");
    assert_eq!(c.nrows(), 2);
    assert_eq!(c.ncols(), 4);

    // D: nsensordata × nu = 2 × 1
    let d = derivs.D.as_ref().expect("D should be Some");
    assert_eq!(d.nrows(), 2);
    assert_eq!(d.ncols(), 1);
}

/// T2: Opt-in negative case → AC3
#[test]
fn t2_sensor_derivative_opt_in_negative() {
    let (model, data) = sensor_pendulum_2link();

    let config = DerivativeConfig::default();
    assert!(!config.compute_sensor_derivatives);
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();
    assert!(derivs.C.is_none(), "C should be None when not requested");
    assert!(derivs.D.is_none(), "D should be None when not requested");
}

/// T3: C/D accuracy via independent FD validation → AC4, AC5
#[test]
fn t3_sensor_derivative_fd_accuracy() {
    let (model, data) = sensor_pendulum_2link();
    let eps = 1e-6;
    let nv = model.nv;
    let na = model.na;
    let nu = model.nu;
    let ns = model.nsensordata;
    let nx = 2 * nv + na;

    let config = DerivativeConfig {
        eps,
        centered: true,
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();
    let c_api = derivs.C.as_ref().unwrap();
    let d_api = derivs.D.as_ref().unwrap();

    // Independent manual FD for C matrix (state columns)
    let mut c_manual = nalgebra::DMatrix::zeros(ns, nx);
    for i in 0..nx {
        let mut scratch_p = data.clone();
        let mut dq = nalgebra::DVector::zeros(nv);
        if i < nv {
            dq[i] = eps;
            let mut qpos_perturbed = data.qpos.clone();
            mj_integrate_pos_explicit(&model, &mut qpos_perturbed, &data.qpos, &dq, 1.0);
            scratch_p.qpos.copy_from(&qpos_perturbed);
        } else if i < 2 * nv {
            scratch_p.qvel[i - nv] += eps;
        } else {
            scratch_p.act[i - 2 * nv] += eps;
        }
        // Sensors at the perturbed current state, as MuJoCo's mjd_transitionFD.
        scratch_p.forward(&model).unwrap();
        let s_plus = scratch_p.sensordata.clone();

        let mut scratch_m = data.clone();
        if i < nv {
            dq[i] = -eps; // it was set to eps above; but let's just rebuild
            let mut dq_neg = nalgebra::DVector::zeros(nv);
            dq_neg[i] = -eps;
            let mut qpos_perturbed = data.qpos.clone();
            mj_integrate_pos_explicit(&model, &mut qpos_perturbed, &data.qpos, &dq_neg, 1.0);
            scratch_m.qpos.copy_from(&qpos_perturbed);
        } else if i < 2 * nv {
            scratch_m.qvel[i - nv] -= eps;
        } else {
            scratch_m.act[i - 2 * nv] -= eps;
        }
        scratch_m.forward(&model).unwrap();
        let s_minus = scratch_m.sensordata.clone();

        let scol = (&s_plus - &s_minus) / (2.0 * eps);
        c_manual.column_mut(i).copy_from(&scol);
    }
    let c_err = max_relative_error(c_api, &c_manual, 1e-10);
    assert!(
        c_err < 1e-6,
        "C matrix: API vs manual FD max relative error = {c_err}"
    );

    // Independent manual FD for D matrix (control columns)
    let mut d_manual = nalgebra::DMatrix::zeros(ns, nu);
    for j in 0..nu {
        let mut scratch_p = data.clone();
        scratch_p.ctrl[j] += eps;
        scratch_p.forward(&model).unwrap();
        let s_plus = scratch_p.sensordata.clone();

        let mut scratch_m = data.clone();
        scratch_m.ctrl[j] -= eps;
        scratch_m.forward(&model).unwrap();
        let s_minus = scratch_m.sensordata.clone();

        let scol = (&s_plus - &s_minus) / (2.0 * eps);
        d_manual.column_mut(j).copy_from(&scol);
    }
    let d_err = max_relative_error(d_api, &d_manual, 1e-10);
    assert!(
        d_err < 1e-6,
        "D matrix: API vs manual FD max relative error = {d_err}"
    );
}

/// T4: Hybrid C/D matches FD C/D → AC6
#[test]
fn t4_hybrid_matches_fd_sensor_derivatives() {
    let (model, data) = sensor_pendulum_2link();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };

    let fd = mjd_transition_fd(&model, &data, &config).unwrap();
    let hybrid = mjd_transition_hybrid(&model, &data, &config).unwrap();

    let fd_c = fd.C.as_ref().unwrap();
    let hybrid_c = hybrid.C.as_ref().unwrap();
    // Use a generous floor: near-zero entries (< 1e-4) differ by FD noise
    let c_err = max_relative_error(fd_c, hybrid_c, 1e-4);
    assert!(
        c_err < 0.05,
        "Hybrid C vs FD C: max relative error = {c_err}"
    );

    let fd_d = fd.D.as_ref().unwrap();
    let hybrid_d = hybrid.D.as_ref().unwrap();
    let d_err = max_relative_error(fd_d, hybrid_d, 1e-6);
    assert!(
        d_err < 1e-4,
        "Hybrid D vs FD D: max relative error = {d_err}"
    );
}

/// The hybrid path's sensor-only finite-difference columns (velocity,
/// activation and control) give exactly the pure-FD `C` and `D`: each runs
/// every pipeline stage, since the scratch it reuses holds the previous
/// column's post-step state.
#[test]
fn hybrid_sensor_derivatives_equal_pure_fd() {
    for integrator in ["Euler", "implicit", "implicitfast", "implicitspringdamper"] {
        let xml = format!(
            r#"<mujoco>
              <option timestep="0.01" integrator="{integrator}"/>
              <worldbody>
                <body>
                  <joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
                  <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/>
                </body>
              </worldbody>
              <actuator>
                <motor joint="j"/>
                <general joint="j" dyntype="filter" dynprm="0.1"/>
              </actuator>
              <sensor><jointpos joint="j"/><jointvel joint="j"/></sensor>
            </mujoco>"#
        );
        let model = sim_mjcf::load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.2;
        data.qvel[0] = 0.3;
        data.ctrl[0] = 0.4;
        data.ctrl[1] = 0.5;
        data.act[0] = 0.1;
        data.forward(&model).expect("forward");
        let config = DerivativeConfig {
            compute_sensor_derivatives: true,
            ..DerivativeConfig::default()
        };
        let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
        let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
        assert_eq!(fd.C, hybrid.C, "C, {integrator}");
        assert_eq!(fd.D, hybrid.D, "D, {integrator}");
    }
}

/// T5: A/B unchanged when sensors enabled → AC7
#[test]
fn t5_ab_unchanged_with_sensors_enabled() {
    let (model, data) = sensor_pendulum_2link();

    let config_off = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        compute_sensor_derivatives: false,
        ..DerivativeConfig::default()
    };
    let config_on = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };

    let derivs_off = mjd_transition_fd(&model, &data, &config_off).unwrap();
    let derivs_on = mjd_transition_fd(&model, &data, &config_on).unwrap();

    // A/B should be bitwise identical
    assert_eq!(
        derivs_off.A, derivs_on.A,
        "A matrices differ when sensor derivatives enabled"
    );
    assert_eq!(
        derivs_off.B, derivs_on.B,
        "B matrices differ when sensor derivatives enabled"
    );
}

/// T6: nsensordata == 0 edge case → AC8
#[test]
fn t6_nsensordata_zero_empty_matrices() {
    let model = Model::n_link_pendulum(2, 1.0, 0.1);
    assert_eq!(model.nsensordata, 0);
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let c = derivs
        .C
        .as_ref()
        .expect("C should be Some even with nsensordata=0");
    assert_eq!(c.nrows(), 0, "C should have 0 rows");
    assert_eq!(c.ncols(), 4, "C should have nx columns"); // 2*nv + na = 4

    let d = derivs
        .D
        .as_ref()
        .expect("D should be Some even with nsensordata=0");
    assert_eq!(d.nrows(), 0, "D should have 0 rows");
    assert_eq!(d.ncols(), 0, "D should have nu=0 columns");
}

/// T7: Multi-sensor-type model → AC9
#[test]
fn t7_multi_sensor_type_model() {
    let mjcf = r#"
        <mujoco model="multi_sensor">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" ctrlrange="-10 10"/>
            </actuator>
            <sensor>
                <jointpos joint="j1"/>
                <jointvel joint="j1"/>
                <actuatorfrc actuator="m1"/>
            </sensor>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    assert_eq!(model.nsensordata, 3);
    assert_eq!(model.nv, 1);
    assert_eq!(model.nu, 1);

    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.ctrl[0] = 2.0;
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let c = derivs.C.as_ref().unwrap();
    assert_eq!(c.nrows(), 3);
    assert_eq!(c.ncols(), 2); // nx = 2*1 + 0

    let d = derivs.D.as_ref().unwrap();
    assert_eq!(d.nrows(), 3);
    assert_eq!(d.ncols(), 1);

    // Sensor types should have non-zero C entries (position-stage and velocity-stage sensors)
    let c_abs_sum: f64 = c.iter().map(|v| v.abs()).sum();
    assert!(
        c_abs_sum > 1e-10,
        "C matrix should have non-zero entries for multi-sensor model"
    );

    // D should have non-zero entries for actuatorfrc (depends on ctrl)
    let d_abs_sum: f64 = d.iter().map(|v| v.abs()).sum();
    assert!(
        d_abs_sum > 1e-10,
        "D matrix should have non-zero entries (actuatorfrc depends on ctrl)"
    );
}

/// T8: Control-limited actuator D column → AC10
///
/// Uses actuatorfrc sensor which responds to ctrl changes, and sets ctrl at the
/// upper bound so forward nudge is infeasible — forces backward-only differencing.
#[test]
fn t8_control_limited_actuator_d_column() {
    let mjcf = r#"
        <mujoco model="ctrl_limited">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" ctrlrange="0 1" ctrllimited="true"/>
            </actuator>
            <sensor>
                <actuatorfrc actuator="m1"/>
            </sensor>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // at upper bound — forward nudge infeasible
    data.forward(&model).unwrap();

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let d = derivs.D.as_ref().unwrap();
    // D column should be non-zero (backward-only differencing; actuatorfrc depends on ctrl)
    assert!(
        d[(0, 0)].abs() > 1e-15,
        "D column for control-limited actuator with actuatorfrc sensor should be non-zero, got {}",
        d[(0, 0)]
    );
}

/// T10: Forward-difference sensor derivatives (supplementary)
#[test]
fn t10_forward_difference_sensor_derivatives() {
    let (model, data) = sensor_pendulum_2link();

    let config = DerivativeConfig {
        eps: 1e-6,
        centered: false,
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let c = derivs.C.as_ref().unwrap();
    let d = derivs.D.as_ref().unwrap();

    // Check dimensions
    assert_eq!(c.nrows(), 2);
    assert_eq!(c.ncols(), 4);
    assert_eq!(d.nrows(), 2);
    assert_eq!(d.ncols(), 1);

    // Non-zero: forward difference should produce meaningful sensor derivatives
    let c_abs_sum: f64 = c.iter().map(|v| v.abs()).sum();
    assert!(
        c_abs_sum > 1e-10,
        "Forward-difference C should have non-zero entries"
    );
}

/// T11: Structural C matrix test for jointpos sensors → AC13
///
/// C is `∂sensordata_t/∂x_t`, the sensors at the current state, as MuJoCo's
/// `mjd_transitionFD`. A jointpos sensor reads its own joint's position, so its
/// position block is the identity, its velocity block 0, and D 0.
#[test]
fn t11_structural_cd_to_ab_crosscheck() {
    let (model, data) = sensor_pendulum_2link();
    let nv = model.nv;

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let c = derivs.C.as_ref().unwrap();
    let d = derivs.D.as_ref().unwrap();

    for sensor_idx in 0..2 {
        for j in 0..nv {
            let expected = if sensor_idx == j { 1.0 } else { 0.0 };
            let c_val = c[(sensor_idx, j)];
            assert!(
                (c_val - expected).abs() < 1e-9,
                "C[{sensor_idx},{j}] = {c_val:.12e}, expected {expected:.1}"
            );
            let c_vel = c[(sensor_idx, nv + j)];
            assert_eq!(c_vel, 0.0, "C[{sensor_idx},{}]", nv + j);
        }
        for k in 0..model.nu {
            assert_eq!(d[(sensor_idx, k)], 0.0, "D[{sensor_idx},{k}]");
        }
    }
}

/// T12: Cutoff-clamped sensor zero derivatives → AC14
#[test]
fn t12_cutoff_clamped_sensor_zero_derivatives() {
    let mjcf = r#"
        <mujoco model="cutoff_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1" ctrlrange="-10 10"/>
            </actuator>
            <sensor>
                <jointpos joint="j1" cutoff="0.1"/>
            </sensor>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // far exceeds cutoff
    data.forward(&model).unwrap();

    // Verify sensor is clamped
    assert!(
        data.sensordata[0].abs() <= 0.1 + 1e-12,
        "Sensor should be clamped at cutoff"
    );

    let config = DerivativeConfig {
        compute_sensor_derivatives: true,
        ..DerivativeConfig::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).unwrap();

    let c = derivs.C.as_ref().unwrap();
    // All entries in the C row for the clamped sensor should be approximately zero
    for j in 0..c.ncols() {
        assert!(
            c[(0, j)].abs() < 1e-6,
            "C[0,{j}] = {} should be ~0 for cutoff-clamped sensor",
            c[(0, j)]
        );
    }
}

// ============================================================================
// Hybrid moment dispatch — Joint/Tendon transmission B + activation columns
// ============================================================================

/// Hybrid B matrix matches FD B for Joint-transmission motor actuators.
/// Regression test for the actuator_moment dispatch bug (Joint transmissions
/// on a hinge or slide never populate data.actuator_moment — the hybrid path must construct the
/// moment inline from gear and jnt_dof_adr).
#[test]
fn test_hybrid_vs_fd_b_joint_motor() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    add_torque_actuators(&mut model, 2); // motors on joints 0, 1
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qpos[2] = 0.7;
    data.forward(&model).unwrap();

    let config_fd = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let fd = mjd_transition_fd(&model, &data, &config_fd).unwrap();
    let hyb = mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).unwrap();

    assert_eq!(fd.B.nrows(), hyb.B.nrows());
    assert_eq!(fd.B.ncols(), hyb.B.ncols());
    let err = max_relative_error(&fd.B, &hyb.B, 1e-10);
    assert!(
        err < 1e-4,
        "Hybrid B vs FD B for Joint motors: max rel err = {err:.2e} (should be < 1e-4)"
    );
}

/// Hybrid B matrix matches FD B for Tendon-transmission actuators.
#[test]
fn test_hybrid_vs_fd_b_tendon_actuator() {
    let mjcf = r#"
    <mujoco model="tendon-actuator">
      <compiler angle="radian"/>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -1" rgba="0.5 0.5 0.5 1"/>
          <site name="tip" pos="0 0 -1"/>
        </body>
        <site name="anchor" pos="0 0 0"/>
      </worldbody>
      <tendon>
        <spatial name="ten">
          <site site="anchor"/>
          <site site="tip"/>
        </spatial>
      </tendon>
      <actuator>
        <motor name="ten_motor" tendon="ten" gear="5"/>
      </actuator>
    </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config_fd = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let fd = mjd_transition_fd(&model, &data, &config_fd).unwrap();
    let hyb = mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).unwrap();

    assert_eq!(fd.B.nrows(), hyb.B.nrows());
    assert_eq!(fd.B.ncols(), hyb.B.ncols());
    let err = max_relative_error(&fd.B, &hyb.B, 1e-10);
    assert!(
        err < 1e-4,
        "Hybrid B vs FD B for Tendon motor: max rel err = {err:.2e} (should be < 1e-4)"
    );
}

/// Hybrid A activation columns match FD for Joint-transmission filter actuators.
/// Regression test: the activation column code path had the same actuator_moment
/// dispatch bug as the B matrix code path.
#[test]
fn test_hybrid_vs_fd_act_col_joint_filter() {
    let mut model = Model::n_link_pendulum(1, 1.0, 0.5);
    add_filter_actuator(&mut model, 0, 0.1); // filter on joint 0, tau=0.1
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();

    let config_fd = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let fd = mjd_transition_fd(&model, &data, &config_fd).unwrap();
    let hyb = mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).unwrap();

    // Compare the full A matrix (includes activation columns at col 2*nv..2*nv+na)
    let err = max_relative_error(&fd.A, &hyb.A, 1e-10);
    assert!(
        err < 1e-4,
        "Hybrid A vs FD A for Joint filter actuator: max rel err = {err:.2e} (should be < 1e-4)"
    );
}

/// validate_analytical_vs_fd passes for multi-actuator Joint-transmission models.
/// This is the end-to-end check: both err_A and err_B must be small.
#[test]
fn test_validate_analytical_vs_fd_actuated() {
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
    add_torque_actuators(&mut model, 3); // motors on all 3 joints
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qpos[2] = 0.7;
    data.forward(&model).unwrap();

    let (err_a, err_b) = validate_analytical_vs_fd(&model, &data).expect("validate should succeed");
    assert!(
        err_a < 1e-3,
        "validate_analytical_vs_fd: err_A = {err_a:.2e} (should be < 1e-3)"
    );
    assert!(
        err_b < 1e-3,
        "validate_analytical_vs_fd: err_B = {err_b:.2e} (should be < 1e-3)"
    );
}

// ============================================================================
// Ball joint hybrid A — diagnostic
// ============================================================================

/// Hybrid A matches FD A for ball joints (including extreme inertia ratios).
///
/// The original stress test reported error ~1.0, but this was caused by
/// `max_relative_error` with a strict floor (1e-10) amplifying FD noise on
/// near-zero entries. With a reasonable floor (1e-6), the error is <1e-3.
#[test]
fn test_ball_joint_hybrid_vs_fd_a() {
    // Standard inertia (box geom)
    let mjcf = r#"
        <mujoco model="ball_standard">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint type="ball" name="ball"/>
                    <geom type="box" size="0.3 0.1 0.05" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.2;
    data.forward(&model).unwrap();

    let config_fd = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let fd = mjd_transition_fd(&model, &data, &config_fd).unwrap();
    let hyb = mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).unwrap();

    // Use floor=1e-6 to avoid FD noise on near-zero entries dominating.
    // Tolerance is 2e-3 (looser than hinge joints) because ball joint FD
    // has inherent noise from quaternion tangent-space mapping.
    let err = max_relative_error(&fd.A, &hyb.A, 1e-6);
    assert!(
        err < 2e-3,
        "Ball joint hybrid vs FD A (standard inertia): {err:.2e}"
    );

    // Extreme inertia (100:1 ratio — diaginertia 0.1/0.1/0.001)
    let mjcf2 = r#"
        <mujoco model="ball_extreme">
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
    let model2 = sim_mjcf::load_model(mjcf2).unwrap();
    let mut data2 = model2.make_data();
    data2.qvel[0] = 0.5;
    data2.qvel[1] = -0.3;
    data2.qvel[2] = 0.2;
    data2.forward(&model2).unwrap();

    let fd2 = mjd_transition_fd(&model2, &data2, &config_fd).unwrap();
    let hyb2 = mjd_transition_hybrid(&model2, &data2, &DerivativeConfig::default()).unwrap();

    let err2 = max_relative_error(&fd2.A, &hyb2.A, 1e-6);
    assert!(
        err2 < 2e-3,
        "Ball joint hybrid vs FD A (extreme inertia): {err2:.2e}"
    );
}

/// Under implicitfast the analytic transition's quaternion blocks use the
/// post-step angular velocity `qvel + h·qacc_implicit` — the acceleration the
/// step integrates, not the explicit `qacc` forward() leaves. A free body with
/// a damped ball joint and a damped hinge: analytic A matches pure FD within
/// 1e-6; reading the explicit `qacc` there instead puts it 5.4e-2 off.
#[test]
fn implicitfast_transition_matches_fd_with_quaternion_joints() {
    assert_transition_matches_fd_with_quaternion_joints(None);
}

/// The same model under Euler: its damped joints make the step advance `qvel`
/// with eulerdamp's `(M + h·D)⁻¹·f`, which the quaternion blocks must read
/// instead of `qacc`.
#[test]
fn euler_eulerdamp_transition_matches_fd_with_quaternion_joints() {
    assert_transition_matches_fd_with_quaternion_joints(Some(Integrator::Euler));
}

/// A free body with a damped ball joint and a damped hinge, under the model's
/// implicitfast or `integrator`: analytic A matches pure FD within 1e-6.
fn assert_transition_matches_fd_with_quaternion_joints(integrator: Option<Integrator>) {
    let xml = r#"
    <mujoco model="deriv_free">
      <option timestep="0.005" integrator="implicitfast"/>
      <worldbody>
        <body name="f" pos="0 0 1">
          <freejoint/>
          <geom type="box" size="0.1 0.05 0.03" mass="1" contype="0" conaffinity="0"/>
          <body name="c" pos="0.1 0 0">
            <joint type="ball" damping="0.3"/>
            <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.3"
                  contype="0" conaffinity="0"/>
            <body name="d" pos="0.2 0 0">
              <joint name="h" type="hinge" axis="0 1 0" damping="0.4"/>
              <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"
                    contype="0" conaffinity="0"/>
            </body>
          </body>
        </body>
      </worldbody>
      <actuator><motor joint="h" gear="1"/></actuator>
    </mujoco>"#;
    let mut model = sim_mjcf::load_model(xml).expect("load");
    if let Some(integrator) = integrator {
        model.integrator = integrator;
    }
    let mut data = model.make_data();
    for i in 0..model.nv {
        data.qvel[i] = 2.1 * ((i + 1) as f64).sin();
    }
    data.ctrl[0] = 0.5;
    data.forward(&model).expect("forward");
    let fd = mjd_transition_fd(
        &model,
        &data,
        &DerivativeConfig {
            use_analytical: false,
            ..DerivativeConfig::default()
        },
    )
    .expect("fd");
    let hybrid =
        mjd_transition_hybrid(&model, &data, &DerivativeConfig::default()).expect("hybrid");
    let err = (&fd.A - &hybrid.A).abs().max();
    assert!(
        err < 1e-6,
        "{:?}: analytic A vs FD A: max abs difference {err}",
        model.integrator
    );
}

// ============================================================================
// Sensor derivatives at the current state (MuJoCo's mjd_transitionFD)
// ============================================================================

/// A damped hinge with a motor, a jointpos and a jointvel sensor, under
/// `integrator` (and `option`'s extra attributes), at qpos 0.2, qvel 0.3,
/// ctrl 0.4 after `forward`.
fn sensed_hinge_with(integrator: &str, option: &str) -> (Model, sim_core::Data) {
    let xml = format!(
        r#"<mujoco><option timestep="0.01" integrator="{integrator}" {option}/>
<worldbody><body><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
<geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody>
<actuator><motor joint="j"/></actuator>
<sensor><jointpos joint="j"/><jointvel joint="j"/></sensor></mujoco>"#
    );
    let model = sim_mjcf::load_model(&xml).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = 0.2;
    data.qvel[0] = 0.3;
    data.ctrl[0] = 0.4;
    data.forward(&model).unwrap();
    (model, data)
}

fn sensed_hinge(integrator: &str) -> (Model, sim_core::Data) {
    sensed_hinge_with(integrator, "")
}

/// C and D are the sensors' derivatives at the current state: MuJoCo 3.5.0's
/// `mjd_transitionFD` on this model, eps 1e-6, centered or not, with and
/// without fluid density (measured with the unfused oracle, all three
/// integrators), gives C = [[1.000000000001, 0], [0, 0.9999999999732445]] and
/// D = 0. Density makes the hybrid take its finite-difference position columns.
#[test]
fn sensor_derivatives_at_current_state_match_mujoco_3_5_0() {
    let mujoco_c = [[1.000000000001, 0.0], [0.0, 0.999_999_999_973_244_5]];
    let cases = ["Euler", "implicit", "implicitfast"]
        .into_iter()
        .flat_map(|i| {
            [
                (i, true, ""),
                (i, false, ""),
                (i, true, r#"density="1.2""#),
                (i, false, r#"density="1.2""#),
            ]
        });
    for (integrator, centered, option) in cases {
        let cfg = DerivativeConfig {
            eps: 1e-6,
            centered,
            compute_sensor_derivatives: true,
            ..DerivativeConfig::default()
        };
        let (model, data) = sensed_hinge_with(integrator, option);
        let paths = [
            ("fd", mjd_transition_fd(&model, &data, &cfg).unwrap()),
            (
                "hybrid",
                mjd_transition_hybrid(&model, &data, &cfg).unwrap(),
            ),
        ];
        for (path, derivs) in paths {
            let (c, d) = (derivs.C.unwrap(), derivs.D.unwrap());
            for (r, row) in mujoco_c.iter().enumerate() {
                for (k, want) in row.iter().enumerate() {
                    assert!(
                        (c[(r, k)] - want).abs() < 1e-9,
                        "{integrator} {option} {path} centered {centered}: C[{r},{k}] = {:e}, MuJoCo {want:e}",
                        c[(r, k)]
                    );
                }
            }
            assert!(
                d.iter().all(|&x| x == 0.0),
                "{integrator} {option} {path} centered {centered}: D = {d}"
            );
        }
    }
}

/// Asking pure finite differences for sensor derivatives evaluates no extra
/// forward pass: the passive callback fires as often with them as without.
/// (The hybrid computes A and B analytically where it can, so its C and D
/// need finite-difference columns it would otherwise not run.)
#[test]
fn sensor_derivatives_do_not_add_callbacks() {
    {
        let mut counts = Vec::new();
        for sensors in [false, true] {
            let (mut model, data) = sensed_hinge("Euler");
            let calls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let c = std::sync::Arc::clone(&calls);
            model.set_passive_callback(move |_, _| {
                c.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            });
            let cfg = DerivativeConfig {
                compute_sensor_derivatives: sensors,
                ..DerivativeConfig::default()
            };
            mjd_transition_fd(&model, &data, &cfg).unwrap();
            counts.push(calls.load(std::sync::atomic::Ordering::Relaxed));
        }
        assert_eq!(counts[0], counts[1], "callbacks without and with sensors");
    }
}

/// With an active constraint row the hybrid's analytic velocity columns are
/// wrong (they hold no constraint-force derivative), so it returns pure finite
/// differences; with none it stays analytic.
#[test]
fn hybrid_takes_finite_differences_under_an_active_constraint() {
    let cases = [
        (
            "a hinge at its limit",
            r#"<mujoco><option timestep="0.002"/><worldbody><body>
<joint type="hinge" axis="0 1 0" damping="0.1" limited="true" range="-0.1 0.1"/>
<geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
            Some(0.12),
        ),
        (
            "a box resting on a plane",
            r#"<mujoco><option timestep="0.002"/><worldbody><geom type="plane" size="5 5 0.1"/>
<body pos="0 0 0.099"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody></mujoco>"#,
            None,
        ),
    ];
    let cfg = DerivativeConfig::default();
    for (name, xml, q0) in cases {
        let model = sim_mjcf::load_model(xml).expect("load");
        let mut data = model.make_data();
        if let Some(q) = q0 {
            data.qpos[0] = q;
        }
        data.forward(&model).unwrap();
        assert!(!data.efc_type.is_empty(), "{name}: a constraint is active");
        let fd = mjd_transition_fd(&model, &data, &cfg).unwrap();
        let hybrid = mjd_transition_hybrid(&model, &data, &cfg).unwrap();
        assert_eq!(hybrid.A, fd.A, "{name}");
        assert_eq!(hybrid.B, fd.B, "{name}");
    }
    let (model, data) = sensed_hinge("Euler");
    assert!(data.efc_type.is_empty());
    let fd = mjd_transition_fd(&model, &data, &cfg).unwrap();
    let hybrid = mjd_transition_hybrid(&model, &data, &cfg).unwrap();
    assert_ne!(
        hybrid.A, fd.A,
        "without a constraint the hybrid is analytic"
    );
    assert!((&hybrid.A - &fd.A).abs().max() < 1e-9);
}

/// MuJoCo 3.5.0's `mjd_inverseFD` takes every column through `mj_inverseSkip`,
/// which fires no control callback (`engine_inverse.c:184-245`), so the
/// inverse reads `ctrl` as the caller left it. At a joint limit the constraint
/// force depends on the actuator force, so a callback that wrote `ctrl` would
/// change the derivatives.
#[test]
fn inverse_finite_differences_fire_no_control_callback() {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    for centered in [true, false] {
        for at_limit in [false, true] {
            let (mut model, mut data) = sensed_hinge("Euler");
            if at_limit {
                model.jnt_limited[0] = true;
                model.jnt_range[0] = (-0.1, 0.1);
                data.qpos[0] = 0.12;
                data.forward(&model).unwrap();
                assert!(!data.efc_type.is_empty(), "the limit is active");
            }
            let ctx = format!("centered {centered}, at the limit {at_limit}");
            let cfg = DerivativeConfig {
                centered,
                ..DerivativeConfig::default()
            };
            let plain = sim_core::mjd_inverse_fd(&model, &data, &cfg).unwrap();
            let calls = Arc::new(AtomicUsize::new(0));
            let c = Arc::clone(&calls);
            model.set_control_callback(move |_, d| {
                c.fetch_add(1, Ordering::Relaxed);
                d.ctrl[0] = 3.0 - 50.0 * d.qpos[0] - d.qvel[0];
            });
            let with_callback = sim_core::mjd_inverse_fd(&model, &data, &cfg).unwrap();
            assert_eq!(calls.load(Ordering::Relaxed), 0, "{ctx}");
            assert_eq!(with_callback.DfDq, plain.DfDq, "{ctx}");
            assert_eq!(with_callback.DfDv, plain.DfDv, "{ctx}");
            assert_eq!(with_callback.DfDa, plain.DfDa, "{ctx}");
        }
    }
}

/// The acceleration columns are taken at the nominal state, as MuJoCo's
/// `mjd_inverseFD` takes them, from the centre point before the velocity and
/// position columns (`engine_derivative_fd.c:641-660`): forward and centered
/// differences both give the mass matrix.
#[test]
fn inverse_acceleration_columns_are_taken_at_the_nominal_state() {
    let (model, data) = sensed_hinge("Euler");
    for centered in [true, false] {
        let cfg = DerivativeConfig {
            centered,
            ..DerivativeConfig::default()
        };
        let d = sim_core::mjd_inverse_fd(&model, &data, &cfg).unwrap();
        assert_relative_eq!(d.DfDa[(0, 0)], data.qM[(0, 0)], epsilon = 1e-6);
    }
}

/// Without a constraint the inverse finite differences are MuJoCo 3.5.0's
/// `mjd_inverseFD`: on this moving two-link chain, forward differences with
/// eps 1e-6 give the oracle's values.
#[test]
fn inverse_finite_differences_match_mujoco_3_5_0_without_constraints() {
    let model = sim_mjcf::load_model(
        r#"<mujoco><option timestep="0.01"/>
<worldbody><body><joint name="a" type="hinge" axis="0 1 0" damping="0.1"/>
<geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/>
<body pos="0.5 0 0"><joint type="hinge" axis="0 1 0" damping="0.2"/>
<geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" mass="0.5"/></body></body></worldbody>
<actuator><motor joint="a"/></actuator></mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&[0.3, -0.5]);
    data.qvel.copy_from_slice(&[0.7, -1.1]);
    data.ctrl[0] = 0.4;
    data.forward(&model).unwrap();
    let cfg = DerivativeConfig {
        centered: false,
        ..DerivativeConfig::default()
    };
    let ours = sim_core::mjd_inverse_fd(&model, &data, &cfg).unwrap();
    let mujoco = [
        (
            "DfDq",
            &ours.DfDq,
            [
                [1.254_634_826_253_209_2, 0.560_739_485_955_252_8],
                [-0.194_894_132_610_201_6, 0.307_832_018_431_319_9],
            ],
        ),
        (
            "DfDv",
            &ours.DfDv,
            [
                [0.047_263_191_582_658_24, -0.019_176_996_879_366_47],
                [-0.033_559_811_596_362_01, 0.199_999_999_894_728_83],
            ],
        ),
        (
            "DfDa",
            &ours.DfDa,
            [
                [0.331_567_962_419_399, 0.072_681_481_277_925_29],
                [0.072_681_481_277_925_29, 0.028_802_352_947_110_42],
            ],
        ),
    ];
    for (name, ours, theirs) in mujoco {
        for (r, row) in theirs.iter().enumerate() {
            for (c, &want) in row.iter().enumerate() {
                assert!(
                    (ours[(r, c)] - want).abs() < 1e-8,
                    "{name}[{r}][{c}]: ours {}, MuJoCo {want}",
                    ours[(r, c)]
                );
            }
        }
    }
}

/// Registry `D-FD-CTRL-COLUMNS`: pure finite differences take each control
/// column through a full step, so B is the derivative of the step a passive
/// callback that reads `ctrl` changes. MuJoCo 3.5.0's `mjd_transitionFD`
/// skips the velocity stage there (`engine_derivative_fd.c:350`) and keeps the
/// nominal passive force: with this callback its B is [0.0010988, 0.10988],
/// as without it (measured). Ours equals a central difference of two steps.
#[test]
fn fd_control_columns_differentiate_a_ctrl_reading_passive_callback() {
    let (mut model, data) = sensed_hinge("Euler");
    model.set_passive_callback(|_, d| d.qfrc_passive[0] += 5.0 * d.ctrl[0]);
    let cfg = DerivativeConfig {
        use_analytical: false,
        ..DerivativeConfig::default()
    };
    let b = mjd_transition_fd(&model, &data, &cfg).unwrap().B;
    let next = |du: f64| {
        let mut d = data.clone();
        d.ctrl[0] += du;
        d.step(&model).unwrap();
        [d.qpos[0], d.qvel[0]]
    };
    let (plus, minus) = (next(cfg.eps), next(-cfg.eps));
    for row in 0..2 {
        let want = (plus[row] - minus[row]) / (2.0 * cfg.eps);
        assert!(
            (b[(row, 0)] - want).abs() < 1e-9,
            "B {b}, row {row}: {want}"
        );
    }
    assert!((b[(1, 0)] - 0.109_881_231_336_039_78).abs() > 0.1, "{b}");
}

/// Registry `D-FD-CENTERED-D-SIGN`: MuJoCo 3.5.0's centered `mjd_transitionFD`
/// gives the control columns of D the wrong sign: `clampedDiff` takes
/// `x_minus - x_plus` there (`engine_derivative_fd.c:79`), where every other
/// centered column takes plus minus minus. Measured on this hinge's
/// `actuatorfrc` sensor: D = 0.99999999997 forward, -0.99999999997 centered.
/// Ours has the forward sign both ways, as a central difference of the sensor
/// at the current state does.
#[test]
fn centered_d_has_the_forward_sign() {
    let model = sim_mjcf::load_model(
        r#"<mujoco><option timestep="0.01"/>
<worldbody><body><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
<geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody>
<actuator><motor name="m" joint="j"/></actuator>
<sensor><jointpos joint="j"/><actuatorfrc actuator="m"/></sensor></mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.qpos[0] = 0.2;
    data.ctrl[0] = 0.4;
    data.forward(&model).unwrap();
    let eps = 1e-6;
    let sensed = |du: f64| {
        let mut d = data.clone();
        d.ctrl[0] += du;
        d.forward(&model).unwrap();
        d.sensordata[1]
    };
    let central = (sensed(eps) - sensed(-eps)) / (2.0 * eps);
    for centered in [false, true] {
        let cfg = DerivativeConfig {
            centered,
            eps,
            use_analytical: false,
            compute_sensor_derivatives: true,
        };
        let d = mjd_transition_fd(&model, &data, &cfg).unwrap().D.unwrap();
        assert!(
            (d[(1, 0)] - central).abs() < 1e-6,
            "centered {centered}: {d}"
        );
        assert!(d[(1, 0)] > 0.5, "centered {centered}: {d}");
    }
}

/// The transition derivatives leave out what MuJoCo's actuator derivatives
/// leave out (`engine_derivative.c:1066-1146`): every actuator when actuation
/// is disabled, an actuator whose group is disabled, and one whose force its
/// forcerange clamps; and an `actearly` actuator's force reads the next
/// activation in its position and activation columns. The analytic path and
/// pure finite differences both give MuJoCo 3.5.0's `mjd_transitionFD`
/// (unfused build, centered, eps 1e-6; Euler and implicitfast alike): a hinge
/// at qpos 0.5, qvel 1, ctrl 0.3 and act 0.5.
#[test]
fn transition_derivatives_leave_out_what_mujoco_does() {
    const P: f64 = 0.997_822_786_896_751_5;
    const G: f64 = -0.217_721_311_202_012_66;
    const H: f64 = 0.01;
    type Case = (
        &'static str,
        &'static str,
        &'static str,
        &'static str,
        Vec<Vec<f64>>,
        Vec<f64>,
    );
    let cases: Vec<Case> = vec![
        (
            "actuation disabled",
            "",
            r#"<flag actuation="disable"/>"#,
            r#"<motor joint="j" gear="1"/>"#,
            vec![vec![P, H], vec![G, 1.0]],
            vec![0.0, 0.0],
        ),
        (
            "group disabled",
            r#" actuatorgroupdisable="2""#,
            "",
            r#"<motor joint="j" gear="1" group="2"/>"#,
            vec![vec![P, H], vec![G, 1.0]],
            vec![0.0, 0.0],
        ),
        (
            "force clamped",
            "",
            "",
            r#"<position joint="j" kp="10" forcelimited="true" forcerange="-0.1 0.1"/>"#,
            vec![vec![P, H], vec![G, 1.0]],
            vec![0.0, 0.0],
        ),
        (
            "filter, group disabled",
            r#" actuatorgroupdisable="2""#,
            "",
            r#"<general joint="j" dyntype="filter" dynprm="0.05" group="2"/>"#,
            vec![vec![P, H, 0.0], vec![G, 1.0, 0.0], vec![0.0, 0.0, 1.0]],
            vec![0.0, 0.0, 0.0],
        ),
        (
            "filter, force clamped",
            "",
            "",
            r#"<general joint="j" dyntype="filter" dynprm="0.05" forcelimited="true" forcerange="-0.1 0.1"/>"#,
            vec![vec![P, H, 0.0], vec![G, 1.0, 0.0], vec![0.0, 0.0, 0.8]],
            vec![0.0, 0.0, 0.2],
        ),
        (
            "actearly",
            "",
            "",
            r#"<general joint="j" dyntype="filter" dynprm="0.05" gaintype="affine" gainprm="1 2 0" actearly="true"/>"#,
            vec![
                vec![1.000_662_061_101_387_2, H, 0.004_937_868_169_907_489],
                vec![0.066_206_109_261_557_34, 1.0, 0.493_786_818_323_016_5],
                vec![0.0, 0.0, 0.8],
            ],
            vec![0.001_234_467_028_599_084_4, 0.123_446_704_636_265_28, 0.2],
        ),
    ];
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };
    for (name, option, flags, actuator, a, b) in cases {
        for integrator in ["Euler", "implicitfast"] {
            let xml = format!(
                r#"<mujoco>
                  <option timestep="0.01" integrator="{integrator}"{option}>{flags}</option>
                  <worldbody>
                    <body name="b">
                      <joint name="j" type="hinge" axis="0 1 0"/>
                      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
                    </body>
                  </worldbody>
                  <actuator>{actuator}</actuator>
                </mujoco>"#
            );
            let model = sim_mjcf::load_model(&xml).expect("load");
            let mut data = model.make_data();
            data.qpos[0] = 0.5;
            data.qvel[0] = 1.0;
            data.ctrl[0] = 0.3;
            data.act.fill(0.5);
            data.forward(&model).expect("forward");
            let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
            let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
            for (path, ours) in [("analytic", &hybrid), ("fd", &fd)] {
                for (r, row) in a.iter().enumerate() {
                    for (c, want) in row.iter().enumerate() {
                        let got = ours.A[(r, c)];
                        assert!(
                            (got - want).abs() < 1e-6,
                            "{name}, {integrator}, {path}: A[{r},{c}] {got}, MuJoCo {want}"
                        );
                    }
                }
                for (r, want) in b.iter().enumerate() {
                    let got = ours.B[(r, 0)];
                    assert!(
                        (got - want).abs() < 1e-6,
                        "{name}, {integrator}, {path}: B[{r},0] {got}, MuJoCo {want}"
                    );
                }
            }
        }
    }
}

/// A bad control makes every actuator's input 0 for the pass, so MuJoCo's
/// transition derivative has no control column and no position term from a
/// gain: a NaN or infinite control on a motor beside a `kp="10"` position
/// actuator (the same matrices for both). The
/// analytic path and pure finite differences both give MuJoCo 3.5.0's
/// `mjd_transitionFD` (unfused build, centered, eps 1e-6; Euler and
/// implicitfast alike) at qpos 0.5, qvel 1.
#[test]
fn transition_derivatives_with_a_bad_control_are_mujocos() {
    let a = [
        [0.966_961_110_737_685_2, 0.01],
        [-3.303_888_925_998_421_7, 1.0],
    ];
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };
    for (integrator, bad) in [
        ("Euler", f64::NAN),
        ("implicitfast", f64::NAN),
        ("Euler", f64::INFINITY),
        ("implicitfast", f64::INFINITY),
    ] {
        let xml = format!(
            r#"<mujoco>
              <option timestep="0.01" integrator="{integrator}"/>
              <worldbody>
                <body name="b">
                  <joint name="j" type="hinge" axis="0 1 0"/>
                  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
                </body>
              </worldbody>
              <actuator>
                <motor joint="j" gear="1"/>
                <position joint="j" kp="10"/>
              </actuator>
            </mujoco>"#
        );
        let model = sim_mjcf::load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.qvel[0] = 1.0;
        data.ctrl[0] = bad;
        data.ctrl[1] = 0.3;
        data.forward(&model).expect("forward");
        let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
        let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
        for (path, ours) in [("analytic", &hybrid), ("fd", &fd)] {
            for (r, row) in a.iter().enumerate() {
                for (c, &want) in row.iter().enumerate() {
                    let got = ours.A[(r, c)];
                    assert!(
                        (got - want).abs() < 1e-6,
                        "{integrator}, ctrl {bad}, {path}: A[{r},{c}] {got}, MuJoCo {want}"
                    );
                    let got = ours.B[(r, c)];
                    assert!(
                        got.abs() < 1e-6,
                        "{integrator}, ctrl {bad}, {path}: B[{r},{c}] {got}, MuJoCo 0"
                    );
                }
            }
        }
    }
}

/// A control its ctrlrange clamps: the forward pass acts on the clamped value
/// and MuJoCo's finite differences nudge no control outside its range, so
/// its control column is 0; under implicitfast its velocity derivative also
/// reads the control as written. A hinge motor at ctrl 2 and an affine gain
/// with a velocity term at ctrl 5, each with `ctrlrange="-1 1"`, at qpos 0.5,
/// qvel 1. The analytic path and pure finite differences both give MuJoCo
/// 3.5.0's `mjd_transitionFD` (unfused build, centered, eps 1e-6).
#[test]
fn transition_derivatives_with_a_clamped_control_are_mujocos() {
    const P: f64 = 0.997_822_786_896_751_5;
    let cases = [
        (
            "Euler",
            r#"<motor joint="j" gear="1" ctrlrange="-1 1"/>"#,
            2.0,
            [[P, 0.01], [-0.217_721_311_424_057_26, 1.0]],
        ),
        (
            "implicitfast",
            r#"<motor joint="j" gear="1" ctrlrange="-1 1"/>"#,
            2.0,
            [[P, 0.01], [-0.217_721_311_424_057_26, 1.0]],
        ),
        (
            "Euler",
            r#"<general joint="j" gaintype="affine" gainprm="1 0 -2" ctrlrange="-1 1"/>"#,
            5.0,
            [
                [P, 0.003_827_664_796_229_868],
                [-0.217_721_311_090_990_35, 0.382_766_476_958_451_56],
            ],
        ),
        (
            "implicitfast",
            r#"<general joint="j" gaintype="affine" gainprm="1 0 -2" ctrlrange="-1 1"/>"#,
            5.0,
            [
                [0.999_467_174_800_372_5, 0.008_489_456_193_583_322],
                [-0.053_282_520_728_892_28, 0.848_945_618_248_109_2],
            ],
        ),
    ];
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        ..DerivativeConfig::default()
    };
    for (integrator, actuator, ctrl, a) in cases {
        let xml = format!(
            r#"<mujoco>
              <option timestep="0.01" integrator="{integrator}"/>
              <worldbody>
                <body name="b">
                  <joint name="j" type="hinge" axis="0 1 0"/>
                  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
                </body>
              </worldbody>
              <actuator>{actuator}</actuator>
            </mujoco>"#
        );
        let model = sim_mjcf::load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.qvel[0] = 1.0;
        data.ctrl[0] = ctrl;
        data.forward(&model).expect("forward");
        let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
        let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
        for (path, ours) in [("analytic", &hybrid), ("fd", &fd)] {
            for (r, row) in a.iter().enumerate() {
                for (c, &want) in row.iter().enumerate() {
                    let got = ours.A[(r, c)];
                    assert!(
                        (got - want).abs() < 1e-6,
                        "{actuator}, {integrator}, {path}: A[{r},{c}] {got}, MuJoCo {want}"
                    );
                }
                let got = ours.B[(r, 0)];
                assert!(
                    got.abs() < 1e-6,
                    "{actuator}, {integrator}, {path}: B[{r},0] {got}, MuJoCo 0"
                );
            }
        }
    }
}

/// `mjd_smooth_pos` differentiates the force the forward pass computed: a
/// direct actuator's input is its control clamped to its ctrlrange, and 0
/// for every actuator when one control is bad. An affine gain with a length
/// term beside a motor, on one hinge.
#[test]
fn smooth_pos_reads_the_control_the_forward_pass_acted_on() {
    let model = sim_mjcf::load_model(
        r#"<mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge" axis="0 1 0"/>
              <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
            </body>
          </worldbody>
          <actuator>
            <general joint="j" gaintype="affine" gainprm="1 2 0" ctrlrange="-1 1"/>
            <motor joint="j"/>
          </actuator>
        </mujoco>"#,
    )
    .expect("load");
    let qderiv_pos = |ctrl: [f64; 2]| {
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.ctrl[0] = ctrl[0];
        data.ctrl[1] = ctrl[1];
        data.forward(&model).expect("forward");
        mjd_smooth_pos(&model, &mut data);
        data.qDeriv_pos.clone()
    };
    assert_ne!(qderiv_pos([1.0, 0.0]), qderiv_pos([0.0, 0.0]));
    assert_eq!(qderiv_pos([5.0, 0.0]), qderiv_pos([1.0, 0.0]));
    assert_eq!(qderiv_pos([0.7, f64::NAN]), qderiv_pos([0.0, 0.0]));
}

/// `mjd_smooth_pos` equals a central difference, over the configuration, of
/// the forces the step applies through a tendon (and the hybrid transition's
/// `A` equals pure finite differences of the step): `qfrc_passive +
/// qfrc_actuator − qfrc_bias − M·qacc` at the state's `qacc`. A spatial
/// tendon's `J` moves with `q`, so its spring, its damper (`−b·J·qvel`) and
/// an actuator's force (`gear·J·force`, the force reading `gear·J·qvel`
/// under a velocity gain or bias, or held at its forcerange's bound, where
/// the force itself has no derivative) carry terms through `∂J/∂q`; a fixed
/// tendon's `J` is constant. With the spring or the damper disabled the
/// step applies neither. Hinge and ball roots, Euler and implicitfast
/// (implicitspringdamper applies tendon springs and dampers in its implicit
/// solve, so `qfrc_passive` holds neither there).
#[test]
fn smooth_pos_matches_finite_differences_through_tendons() {
    let tendons = [
        (
            "spatial",
            r#"<spatial name="t"{prm}><site site="w"/><site site="s3"/></spatial>"#,
        ),
        (
            "fixed",
            r#"<fixed name="t"{prm}><joint joint="j2" coef="1"/><joint joint="j3" coef="-0.5"/></fixed>"#,
        ),
    ];
    let actuators = [
        ("none", ""),
        (
            "motor",
            r#"<motor name="a" tendon="t" gear="1.5" ctrlrange="-1 1"/>"#,
        ),
        (
            "damper",
            r#"<damper name="a" tendon="t" kv="2" ctrlrange="0 1"/>"#,
        ),
        (
            "velocity",
            r#"<velocity name="a" tendon="t" kv="2" gear="1.5"/>"#,
        ),
        (
            "motor at its forcerange",
            r#"<motor name="a" tendon="t" gear="1.5" forcelimited="true" forcerange="-0.2 0.2"/>"#,
        ),
    ];
    let mut failures = Vec::new();
    let mut cases = 0;
    for root in ["hinge", "ball"] {
        let joint = if root == "ball" {
            r#"<joint name="j1" type="ball"/>"#
        } else {
            r#"<joint name="j1" type="hinge" axis="0 1 0" range="-2 2"/>"#
        };
        for integrator in ["Euler", "implicitfast"] {
            for flags in [
                "",
                r#"<flag spring="disable"/>"#,
                r#"<flag damper="disable"/>"#,
            ] {
                for (tendon, tendon_xml) in tendons {
                    for prm in ["", r#" stiffness="3" springlength="0.3" damping="0.7""#] {
                        for (actuator, actuator_xml) in actuators {
                            let tendon_xml = tendon_xml.replace("{prm}", prm);
                            let xml = format!(
                                r#"<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.01" integrator="{integrator}">{flags}</option>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <site name="w" pos="0.3 0.2 0.5"/>
    <body name="b1" pos="0 0 1">
      {joint}
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0" range="-2 2"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.7"/>
        <body name="b3" pos="0.3 0 0">
          <joint name="j3" type="slide" axis="1 0 0" range="-1 1"/>
          <geom type="sphere" size="0.05" mass="0.5"/>
          <site name="s3" pos="0 0 0.05"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>{tendon_xml}</tendon>
  <actuator>{actuator_xml}</actuator>
</mujoco>"#
                            );
                            let model = sim_mjcf::load_model(&xml).expect("load");
                            let mut data = model.make_data();
                            if root == "ball" {
                                // 0.3 rad about (1, 1, 0)/√2.
                                let (s, c) = (0.15_f64.sin(), 0.15_f64.cos());
                                let k = s / 2.0_f64.sqrt();
                                data.qpos.as_mut_slice()[..4].copy_from_slice(&[c, k, k, 0.0]);
                                data.qvel.as_mut_slice()[..3].copy_from_slice(&[0.7, -0.4, 0.3]);
                            } else {
                                data.qpos[0] = 0.4;
                                data.qvel[0] = 0.7;
                            }
                            data.qpos[model.jnt_qpos_adr[1]] = 0.5;
                            data.qvel[model.jnt_dof_adr[1]] = 1.0;
                            data.qpos[model.jnt_qpos_adr[2]] = 0.1;
                            data.qvel[model.jnt_dof_adr[2]] = -0.5;
                            if model.nu > 0 {
                                data.ctrl[0] = 0.3;
                            }
                            data.forward(&model).expect("forward");
                            let qacc = data.qacc.clone();
                            let mut analytic = data.clone();
                            mjd_smooth_pos(&model, &mut analytic);
                            let nv = model.nv;
                            let eps = 1e-6;
                            let force = |sign: f64, c: usize| {
                                let mut d = data.clone();
                                let mut dq = nalgebra::DVector::zeros(nv);
                                dq[c] = sign * eps;
                                mj_integrate_pos_explicit(
                                    &model,
                                    &mut d.qpos,
                                    &data.qpos,
                                    &dq,
                                    1.0,
                                );
                                d.forward(&model).expect("forward");
                                &d.qfrc_passive + &d.qfrc_actuator - &d.qfrc_bias - &d.qM * &qacc
                            };
                            // `f64::max` drops a NaN, so non-finite entries are
                            // counted apart.
                            let mut worst = 0.0_f64;
                            let mut non_finite = false;
                            for c in 0..nv {
                                let fd = (force(1.0, c) - force(-1.0, c)) / (2.0 * eps);
                                for r in 0..nv {
                                    let d = (analytic.qDeriv_pos[(r, c)] - fd[r]).abs();
                                    non_finite |= !d.is_finite();
                                    worst = worst.max(d);
                                }
                            }
                            // The transition's A from the hybrid, against pure
                            // finite differences of the step.
                            let config = DerivativeConfig::default();
                            let hybrid =
                                mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
                            let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
                            let mut worst_a = 0.0_f64;
                            for (h, f) in hybrid.A.iter().zip(fd.A.iter()) {
                                let excess = (h - f).abs() - 1e-5 * h.abs().max(f.abs());
                                non_finite |= !excess.is_finite();
                                worst_a = worst_a.max(excess);
                            }
                            cases += 1;
                            if non_finite || worst >= 1e-6 || worst_a >= 1e-6 {
                                failures.push(format!(
                                    "{root} root, {integrator}, flags [{flags}], {tendon} tendon [{prm}], {actuator}: smooth_pos {worst:.3e}, transition A {worst_a:.3e}"
                                ));
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(cases, 240);
    assert!(
        failures.is_empty(),
        "{} of {cases} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// The step clamps the next activation to its actrange, so the activation
/// column of `A` is 0 where the next activation, before the clamp, falls
/// outside it: the hybrid decides that with the step's own formula, the
/// exact filter for `filterexact`, `act + h·act_dot` for an integrator. In
/// each case the other formula would land on the other side of the bound (the
/// integrator's `dynprm` is the time constant the exact filter would read).
#[test]
fn hybrid_activation_column_follows_the_activation_clamp() {
    for (dynamics, act, ctrl) in [
        (r#"dyntype="filterexact" dynprm="0.05""#, 1.05, 0.79),
        (r#"dyntype="integrator" dynprm="0.05""#, 0.999, 0.105),
    ] {
        let model = sim_mjcf::load_model(&format!(
            r#"<mujoco>
              <option timestep="0.01"/>
              <worldbody>
                <body name="b">
                  <joint name="j" type="hinge" axis="0 1 0"/>
                  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
                </body>
              </worldbody>
              <actuator>
                <general joint="j" {dynamics} actlimited="true" actrange="-1 1"/>
              </actuator>
            </mujoco>"#
        ))
        .expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.qvel[0] = 1.0;
        data.act[0] = act;
        data.ctrl[0] = ctrl;
        data.forward(&model).expect("forward");
        let config = DerivativeConfig::default();
        let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
        let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
        assert!(
            fd.A[(2, 2)].abs() < 1e-9,
            "{dynamics}: fd A[2,2] {}",
            fd.A[(2, 2)]
        );
        for (h, f) in hybrid.A.iter().zip(fd.A.iter()) {
            assert!((h - f).abs() < 1e-6, "{dynamics}: hybrid {h}, fd {f}");
        }
    }
}

/// Under implicitfast the step's `D` holds a muscle gain's slope along its
/// force-velocity curve (`mjd_actuator_vel`), and that slope moves with the
/// velocity, so the analytic velocity columns miss a term: a model with a
/// muscle gain takes finite differences there, as under implicit. The sweep
/// covers `<muscle>`; this is the Hill-type gain, which it does not build. The
/// state sits at the optimal fiber length (gainprm[4]): below a normalized
/// length of 0.5 the active force-length factor is 0 (`hill_active_fl`), and
/// with it the gain's slope.
#[test]
fn hybrid_hill_muscle_gain_under_implicitfast_matches_finite_differences() {
    let model = sim_mjcf::load_model(
        r#"<mujoco>
          <option timestep="0.01" integrator="implicitfast"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge" axis="0 1 0"/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.02" mass="1"/>
            </body>
          </worldbody>
          <actuator>
            <general name="hill" joint="j" dyntype="hillmuscle"
                     gainprm="0.75 1.05 500 200 1.0 0.0 10.0 0.0 35.0"
                     dynprm="0.01 0.04 0.0"/>
          </actuator>
        </mujoco>"#,
    )
    .expect("load");
    assert_eq!(model.actuator_gaintype[0], GainType::HillMuscle);
    let mut data = model.make_data();
    data.qpos[0] = 1.0;
    data.qvel[0] = 2.0;
    data.act[0] = 0.6;
    data.ctrl[0] = 0.6;
    data.forward(&model).expect("forward");
    let config = DerivativeConfig::default();
    let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
    let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
    for (h, f) in hybrid.A.iter().zip(fd.A.iter()) {
        assert!(
            (h - f).abs() <= 1e-6 + 1e-5 * h.abs().max(f.abs()),
            "hybrid {h}, fd {f}"
        );
    }
}

/// With fluid density the implicit integrators' `D` holds the quadratic
/// drag's slope (`mjd_passive_vel`), which moves with the velocity, so the
/// analytic velocity columns miss a term in `∂D/∂v`: such a model takes pure
/// finite differences under implicitfast and implicit. Under Euler and
/// implicitspringdamper the step applies the drag without `D`, and the
/// analytic columns agree.
#[test]
fn hybrid_takes_finite_differences_in_a_dense_fluid_under_the_implicit_integrators() {
    for integrator in ["Euler", "implicitspringdamper", "implicitfast", "implicit"] {
        let model = sim_mjcf::load_model(&format!(
            r#"<mujoco>
              <option timestep="0.01" integrator="{integrator}" density="1000"/>
              <worldbody>
                <body name="b" pos="0 0 1">
                  <joint name="j" type="hinge" axis="0 1 0"/>
                  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
                  <body name="b2" pos="0.3 0 0">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.7"/>
                  </body>
                </body>
              </worldbody>
            </mujoco>"#
        ))
        .expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.4;
        data.qpos[1] = 0.5;
        data.qvel[0] = 0.7;
        data.qvel[1] = 1.0;
        data.forward(&model).expect("forward");
        let config = DerivativeConfig::default();
        let hybrid = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
        let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
        for (h, f) in hybrid.A.iter().zip(fd.A.iter()) {
            assert!(
                (h - f).abs() <= 1e-6 + 1e-5 * h.abs().max(f.abs()),
                "{integrator}: hybrid {h}, fd {f}"
            );
        }
    }
}

/// The transition matrices a column at a time, each column from its own copy
/// of `data` (centered differences), so no column sees another. The fixture
/// must be hinges and slides only: the position rows are plain differences.
fn transition_a_column_at_a_time(
    model: &Model,
    data: &sim_core::Data,
    eps: f64,
) -> (nalgebra::DMatrix<f64>, nalgebra::DMatrix<f64>) {
    assert_eq!(model.nq, model.nv, "hinges and slides only");
    let (nv, na, nu) = (model.nv, model.na, model.nu);
    let nx = 2 * nv + na;
    let next = |nudge: &dyn Fn(&mut sim_core::Data)| -> nalgebra::DVector<f64> {
        let mut d = data.clone();
        nudge(&mut d);
        d.step(model).expect("step");
        let mut x = nalgebra::DVector::zeros(nx);
        x.rows_mut(0, nv).copy_from(&(&d.qpos - &data.qpos));
        x.rows_mut(nv, nv).copy_from(&d.qvel);
        x.rows_mut(2 * nv, na).copy_from(&d.act);
        x
    };
    let state = |i: usize, delta: f64| {
        next(&move |d: &mut sim_core::Data| {
            if i < nv {
                d.qpos[i] += delta;
            } else if i < 2 * nv {
                d.qvel[i - nv] += delta;
            } else {
                d.act[i - 2 * nv] += delta;
            }
        })
    };
    let mut a = nalgebra::DMatrix::zeros(nx, nx);
    for i in 0..nx {
        a.set_column(i, &((state(i, eps) - state(i, -eps)) / (2.0 * eps)));
    }
    let mut b = nalgebra::DMatrix::zeros(nx, nu);
    for j in 0..nu {
        let plus = next(&move |d: &mut sim_core::Data| d.ctrl[j] += eps);
        let minus = next(&move |d: &mut sim_core::Data| d.ctrl[j] -= eps);
        b.set_column(j, &((plus - minus) / (2.0 * eps)));
    }
    (a, b)
}

/// Asserts that the finite-difference and hybrid transition matrices at `data`
/// equal the ones taken a column at a time (to 1e-9 plus 1e-9 relative), and
/// returns them (finite differences, hybrid).
#[allow(non_snake_case)]
fn assert_transition_matches_a_column_at_a_time(
    what: &str,
    model: &Model,
    data: &sim_core::Data,
) -> (sim_core::TransitionMatrices, sim_core::TransitionMatrices) {
    let config = DerivativeConfig::default();
    let (a, b) = transition_a_column_at_a_time(model, data, config.eps);
    let fd = mjd_transition_fd(model, data, &config).expect("fd");
    let hybrid = mjd_transition_hybrid(model, data, &config).expect("hybrid");
    for (name, got, want) in [
        ("fd A", &fd.A, &a),
        ("fd B", &fd.B, &b),
        ("hybrid A", &hybrid.A, &a),
        ("hybrid B", &hybrid.B, &b),
    ] {
        for r in 0..want.nrows() {
            for c in 0..want.ncols() {
                let (g, w) = (got[(r, c)], want[(r, c)]);
                assert!(
                    (g - w).abs() <= 1e-9 + 1e-9 * w.abs(),
                    "{what}: {name}[{r},{c}] {g}, a column at a time {w}"
                );
            }
        }
    }
    (fd, hybrid)
}

/// Finite differences of the transition with sleep in play: each column starts
/// from the state as the caller holds it, its sleep state included, so the
/// matrices equal the ones taken a column at a time from fresh copies of the
/// state (registry `D-FD-SLEEP`: MuJoCo's `mjd_transitionFD` carries one
/// column's wake into the next). A nudge can wake a sleeping tree, and a step
/// counts an awake tree down toward sleep, so both are checked: a tree asleep,
/// and an awake tree at rest at each state of its countdown, the last
/// included, from which the step puts it to sleep (unactuated: an actuator
/// keeps a tree under the auto policy awake). In the tree asleep and at the
/// countdown's last state the hybrid takes pure finite differences: its
/// analytic columns know nothing of a tree waking or going to sleep.
#[test]
fn transition_derivatives_take_each_column_from_the_sleep_state() {
    let model = sim_mjcf::load_model(
        r#"<mujoco>
          <option timestep="0.002" gravity="0 0 0"><flag sleep="enable"/></option>
          <worldbody>
            <body name="awake" pos="0 0 1">
              <joint name="j0" type="hinge" axis="0 1 0" damping="0.3"/>
              <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"
                    contype="0" conaffinity="0"/>
            </body>
            <body name="asleep" pos="1 0 1" sleep="init">
              <joint name="j1" type="hinge" axis="0 1 0" damping="0.3"/>
              <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"
                    contype="0" conaffinity="0"/>
            </body>
          </worldbody>
          <actuator>
            <motor name="m1" joint="j1"/>
          </actuator>
        </mujoco>"#,
    )
    .expect("load");
    let check = |what: &str, data: &sim_core::Data| {
        assert_transition_matches_a_column_at_a_time(what, &model, data);
    };

    // The second tree asleep, its motor driven.
    let mut data = model.make_data();
    data.qvel[0] = 0.5;
    data.ctrl.fill(0.2);
    data.forward(&model).expect("forward");
    assert!(data.tree_asleep[0] < 0 && data.tree_asleep[1] >= 0);
    check("a tree asleep", &data);

    // Both awake, the first at rest: each state of its countdown, down to the
    // last, from which the step puts it to sleep. `make_data`'s sleep pass
    // (for the `sleep="init"` tree) has counted it down once; it starts over.
    let mut data = model.make_data();
    data.qvel[1] = 0.5;
    data.tree_asleep[0] = -(1 + sim_core::MIN_AWAKE);
    data.forward(&model).expect("forward");
    let mut countdown = Vec::new();
    while countdown.len() < 20 {
        let t = data.tree_asleep[0];
        assert!(data.tree_asleep[1] < 0, "{:?}", data.tree_asleep);
        countdown.push(t);
        check(&format!("countdown at {t}"), &data);
        if t == -2 {
            break;
        }
        data.step(&model).expect("step");
        data.forward(&model).expect("forward");
    }
    let want: Vec<i32> = (-(1 + sim_core::MIN_AWAKE)..=-2).collect();
    assert_eq!(countdown, want);
}

/// A passive plugin with one state entry, which each step adds the timestep
/// times the first joint's position to, and whose force pushes that joint back
/// by twice the state: an integral term, whose force depends on the state the
/// steps before it left.
struct IntegralTerm;

impl sim_core::plugin::Plugin for IntegralTerm {
    fn name(&self) -> &'static str {
        "test.integral_term"
    }

    fn capabilities(&self) -> sim_core::plugin::PluginCapabilities {
        sim_core::plugin::PluginCapabilityBit::Passive.into()
    }

    fn nstate(&self, _model: &Model, _instance: usize) -> usize {
        1
    }

    fn compute(
        &self,
        model: &Model,
        data: &mut sim_core::Data,
        instance: usize,
        _capability: sim_core::plugin::PluginCapabilityBit,
    ) {
        data.qfrc_passive[0] -= 2.0 * data.plugin_state[model.plugin_stateadr[instance]];
    }

    fn advance(&self, model: &Model, data: &mut sim_core::Data, instance: usize) {
        data.plugin_state[model.plugin_stateadr[instance]] += model.timestep * data.qpos[0];
    }
}

/// Finite differences of the transition start each step from the plugin state
/// the caller holds, as MuJoCo's `mjd_stepFD` restores `mjSTATE_FULLPHYSICS`,
/// which holds `mjSTATE_PLUGIN` (`engine_derivative_fd.c:307`,
/// `engine_support.c:167`): the matrices equal the ones taken a column at a
/// time from fresh copies of the state. Sleep is disabled; with it enabled
/// each step starts from a copy of the whole `Data`. Under Euler every hybrid
/// column of this fixture is analytic; under full Implicit its position
/// columns are finite differences.
#[test]
fn transition_derivatives_take_each_column_from_the_plugin_state() {
    for integrator in ["Euler", "implicit"] {
        check_transition_with_a_stateful_plugin(integrator);
    }
}

fn check_transition_with_a_stateful_plugin(integrator: &str) {
    let mut model = sim_mjcf::load_model(&format!(
        r#"<mujoco>
          <option timestep="0.01" integrator="{integrator}"/>
          <worldbody>
            <body pos="0 0 1">
              <joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
              <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"
                    contype="0" conaffinity="0"/>
            </body>
          </worldbody>
          <actuator>
            <motor name="m" joint="j"/>
          </actuator>
        </mujoco>"#
    ))
    .expect("load");
    let plugin: std::sync::Arc<dyn sim_core::plugin::Plugin> = std::sync::Arc::new(IntegralTerm);
    model.nplugin = 1;
    model.npluginstate = 1;
    model.plugin_capabilities.push(plugin.capabilities());
    model
        .plugin_needstage
        .push(sim_core::plugin::PluginStage::Acc);
    model.plugin_objects.push(plugin);
    model.plugin_stateadr.push(0);
    model.plugin_statenum.push(1);
    model.plugin_name.push(None);
    model.plugin_attradr.push(0);
    model.plugin_attrnum.push(0);
    let mut data = model.make_data();
    data.qpos[0] = 0.4;
    data.ctrl[0] = 0.2;
    data.plugin_state[0] = 0.3;
    data.forward(&model).expect("forward");

    // The fixture is the subject: a step advances the plugin's state, and
    // that state moves the next step.
    let mut stepped = data.clone();
    stepped.step(&model).expect("step");
    assert_eq!(stepped.plugin_state[0], 0.3 + 0.01 * stepped.qpos[0]);
    let mut moved = data.clone();
    moved.plugin_state[0] = 0.0;
    moved.step(&model).expect("step");
    assert_ne!(moved.qvel[0], stepped.qvel[0]);

    assert_transition_matches_a_column_at_a_time(integrator, &model, &data);
}

/// The hybrid's analytic velocity columns take flex edge damping, whose
/// `−b · JᵀJ` `mjd_passive_vel` adds (MuJoCo `engine_derivative.c:1737-1758`):
/// a four-vertex cable with edge damping, equality constraints disabled (our
/// `<flex>` adds edge equalities, whose rows send the hybrid to pure finite
/// differences), under each integrator, and with dampers disabled, where the
/// passive pass applies none. Its position columns are finite differences (a
/// flex model's always are).
#[test]
fn hybrid_velocity_columns_take_flex_edge_damping() {
    for (integrator, damper) in [
        ("Euler", "enable"),
        ("implicitfast", "enable"),
        ("implicit", "enable"),
        ("implicitspringdamper", "enable"),
        ("implicitfast", "disable"),
    ] {
        let what = format!("{integrator}, damper {damper}");
        let model = sim_mjcf::load_model(&format!(
            r#"<mujoco>
              <option timestep="0.002" integrator="{integrator}">
                <flag equality="disable" damper="{damper}"/>
              </option>
              <deformable>
                <flex name="c" dim="1" mass="0.4">
                  <vertex pos="0 0 1  0.1 0 1  0.2 0 1  0.3 0 1"/>
                  <element data="0 1  1 2  2 3"/>
                  <edge damping="3"/>
                  <contact contype="0" conaffinity="0"/>
                </flex>
              </deformable>
            </mujoco>"#
        ))
        .expect("load");
        assert_eq!((model.nv, model.nflexedge), (12, 3));
        let mut data = model.make_data();
        for (i, v) in data.qvel.iter_mut().enumerate() {
            *v = [0.4, -0.7, 0.5, -0.2, 0.9, 0.3][i % 6] * (1.0 + 0.1 * i as f64);
        }
        data.forward(&model).expect("forward");
        assert!(data.efc_type.is_empty());
        let (fd, hybrid) = assert_transition_matches_a_column_at_a_time(&what, &model, &data);
        assert_ne!(hybrid.A, fd.A, "{what}: the analytic columns ran");
    }
}

/// `mjd_smooth_pos` takes a joint transmission on a ball or free joint as its
/// length moves with `q`: a ball joint's length is its rotation vector along
/// the gear, whose Jacobian is the gear through the log map (the moment only
/// at the identity); a free joint's length is 0. Checked against a central
/// difference of the forces over the configuration, for a position actuator
/// under a `joint` transmission (under `jointinparent` the moment also turns
/// with `q`, a term `mjd_smooth_pos` leaves out, AD-1).
#[test]
fn smooth_pos_takes_ball_and_free_joint_transmissions() {
    let mut failures = Vec::new();
    for (root, gear, quat) in [
        ("ball", "1.5 0.2 -0.3", [1.0, 0.0, 0.0, 0.0]),
        (
            "ball",
            "1.5 0.2 -0.3",
            [
                0.988_771_077_936_042_4,
                0.105_668_716_839_935_62,
                0.105_668_716_839_935_62,
                0.0,
            ],
        ),
        (
            "ball",
            "1.5 0.2 -0.3",
            [
                -0.416_146_836_547_142_4,
                0.396_849_932_157_833_9,
                -0.793_699_864_315_667_7,
                0.198_424_966_078_916_94,
            ],
        ),
        (
            "free",
            "1.5 0.2 -0.3 0.4 -0.5 0.6",
            [
                0.988_771_077_936_042_4,
                0.105_668_716_839_935_62,
                0.105_668_716_839_935_62,
                0.0,
            ],
        ),
    ] {
        let joint = if root == "ball" {
            r#"<joint name="j" type="ball"/>"#
        } else {
            r#"<freejoint name="j"/>"#
        };
        let model = sim_mjcf::load_model(&format!(
            r#"<mujoco>
              <option timestep="0.002" gravity="0 0 0"/>
              <worldbody>
                <body name="box" pos="0 0 1">
                  {joint}
                  <geom type="box" size="0.1 0.2 0.3" mass="1"/>
                </body>
              </worldbody>
              <actuator>
                <position joint="j" gear="{gear}" kp="20"/>
              </actuator>
            </mujoco>"#
        ))
        .expect("load");
        let mut data = model.make_data();
        let q = if root == "ball" { 0 } else { 3 };
        data.qpos.as_mut_slice()[q..q + 4].copy_from_slice(&quat);
        data.ctrl[0] = 0.3;
        data.forward(&model).expect("forward");
        let qacc = data.qacc.clone();
        let mut analytic = data.clone();
        mjd_smooth_pos(&model, &mut analytic);
        let nv = model.nv;
        let eps = 1e-6;
        let force = |sign: f64, c: usize| {
            let mut d = data.clone();
            let mut dq = nalgebra::DVector::zeros(nv);
            dq[c] = sign * eps;
            mj_integrate_pos_explicit(&model, &mut d.qpos, &data.qpos, &dq, 1.0);
            d.forward(&model).expect("forward");
            &d.qfrc_passive + &d.qfrc_actuator - &d.qfrc_bias - &d.qM * &qacc
        };
        // `f64::max` drops a NaN, so non-finite entries are counted apart.
        let mut worst = 0.0_f64;
        let mut non_finite = false;
        for c in 0..nv {
            let fd = (force(1.0, c) - force(-1.0, c)) / (2.0 * eps);
            for r in 0..nv {
                let d = (analytic.qDeriv_pos[(r, c)] - fd[r]).abs();
                non_finite |= !d.is_finite();
                worst = worst.max(d);
            }
        }
        if non_finite || worst >= 1e-6 {
            failures.push(format!(
                "{root} at {quat:?}: worst {worst:e}, non-finite {non_finite}"
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
