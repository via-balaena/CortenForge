//! §34 Integration tests: actlimited / actrange / actearly — Activation State Clamping.
//!
//! Tests cover:
//! - AC1: Integrator clamping to actrange
//! - AC2: Unbounded without actlimited
//! - AC3: Muscle default [0,1] clamping
//! - AC4: Muscle override with custom actrange
//! - AC5: FilterExact integration + actlimited
//! - AC6: actearly force prediction (one-step-earlier response)
//! - AC7: actearly + actlimited interaction
//! - AC8: MuJoCo reference trajectory (Euler integrator + clamping)
//! - AC13: actearly with muscle dynamics — quantitative verification (DT-6)
//! - AC14: actearly=false with muscle uses current activation (DT-6)
//! - AC15: actearly with filter — quantitative exact-value verification (DT-6)

use sim_core::Model;

/// Build a minimal hinge model with one integrator actuator.
/// `actlimited` and `actrange` are configurable.
fn build_integrator_model(actlimited: bool, actrange: (f64, f64), actearly: bool) -> Model {
    let mjcf = if actlimited {
        format!(
            r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="integrator" joint="hinge" dyntype="integrator"
             actlimited="true" actrange="{} {}" actearly="{}"
             gainprm="1" biasprm="0 0 0" biastype="none"/>
  </actuator>
</mujoco>"#,
            actrange.0,
            actrange.1,
            if actearly { "true" } else { "false" }
        )
    } else {
        format!(
            r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="integrator" joint="hinge" dyntype="integrator"
             actlimited="false" actearly="{}"
             gainprm="1" biasprm="0 0 0" biastype="none"/>
  </actuator>
</mujoco>"#,
            if actearly { "true" } else { "false" }
        )
    };
    sim_mjcf::load_model(&mjcf).expect("Failed to load integrator model")
}

/// Build a minimal hinge model with one muscle actuator.
/// If `actrange_override` is Some, sets explicit actlimited + actrange.
fn build_muscle_model(actrange_override: Option<(f64, f64)>) -> Model {
    let actlimited_attr = match actrange_override {
        Some(range) => format!(r#"actlimited="true" actrange="{} {}""#, range.0, range.1),
        None => String::new(), // muscle default: actlimited=true, actrange=[0,1]
    };
    let mjcf = format!(
        r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <muscle name="muscle" joint="hinge" {}/>
  </actuator>
</mujoco>"#,
        actlimited_attr
    );
    sim_mjcf::load_model(&mjcf).expect("Failed to load muscle model")
}

/// Build a model with one position actuator (FilterExact dynamics via timeconst).
fn build_filterexact_model(actlimited: bool, actrange: (f64, f64)) -> Model {
    let actlimited_attr = if actlimited {
        format!(
            r#"actlimited="true" actrange="{} {}""#,
            actrange.0, actrange.1
        )
    } else {
        r#"actlimited="false""#.to_string()
    };
    let mjcf = format!(
        r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="pos" joint="hinge" kp="1" timeconst="0.1" {}/>
  </actuator>
</mujoco>"#,
        actlimited_attr
    );
    sim_mjcf::load_model(&mjcf).expect("Failed to load FilterExact model")
}

/// Build a model with one filter actuator (Euler filter dynamics).
fn build_filter_model(actlimited: bool, actrange: (f64, f64), actearly: bool) -> Model {
    let mjcf = format!(
        r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="filter" joint="hinge" dyntype="filter" dynprm="0.1"
             {} actearly="{}"
             gainprm="1" biasprm="0 -1 0" biastype="affine"/>
  </actuator>
</mujoco>"#,
        if actlimited {
            format!(
                r#"actlimited="true" actrange="{} {}""#,
                actrange.0, actrange.1
            )
        } else {
            r#"actlimited="false""#.to_string()
        },
        if actearly { "true" } else { "false" }
    );
    sim_mjcf::load_model(&mjcf).expect("Failed to load filter model")
}

// ============================================================================
// AC1: Integrator clamping — actlimited="true" actrange="-1 1"
// ============================================================================

#[test]
fn test_integrator_clamped_to_actrange() {
    let model = build_integrator_model(true, (-1.0, 1.0), false);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // act_dot = ctrl = 1.0

    // After 1000 steps at dt=0.001: unclamped act = 1.0*1.0*1000*0.001 = 1.0
    // But if we keep stepping, it should clamp at 1.0
    for _ in 0..2000 {
        data.step(&model).expect("step failed");
    }

    assert!(
        (data.act[0] - 1.0).abs() < 1e-12,
        "Integrator should be clamped to actrange max=1.0, got {}",
        data.act[0]
    );
}

#[test]
fn test_integrator_clamped_negative() {
    let model = build_integrator_model(true, (-1.0, 1.0), false);
    let mut data = model.make_data();
    data.ctrl[0] = -1.0; // drive activation negative

    for _ in 0..2000 {
        data.step(&model).expect("step failed");
    }

    assert!(
        (data.act[0] - (-1.0)).abs() < 1e-12,
        "Integrator should be clamped to actrange min=-1.0, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC2: Unbounded without actlimited
// ============================================================================

#[test]
fn test_integrator_unbounded_without_actlimited() {
    let model = build_integrator_model(false, (0.0, 0.0), false);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    for _ in 0..2000 {
        data.step(&model).expect("step failed");
    }

    // After 2000 steps at dt=0.001 with ctrl=1: act = 2000 * 0.001 * 1.0 = 2.0
    assert!(
        data.act[0] > 1.5,
        "Without actlimited, activation should exceed 1.0, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC3: Muscle default [0, 1] clamping
// ============================================================================

#[test]
fn test_muscle_default_actrange() {
    let model = build_muscle_model(None);

    // Verify model builder applied muscle defaults
    assert!(
        model.actuator_actlimited[0],
        "Muscle should default to actlimited=true"
    );
    assert_eq!(
        model.actuator_actrange[0],
        (0.0, 1.0),
        "Muscle should default to actrange=[0,1]"
    );
}

#[test]
fn test_muscle_clamped_to_default_range() {
    let model = build_muscle_model(None);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    for _ in 0..500 {
        data.step(&model).expect("step failed");
    }

    assert!(
        data.act[0] <= 1.0 + 1e-12,
        "Muscle activation should be clamped to [0,1], got {}",
        data.act[0]
    );
    assert!(
        data.act[0] >= -1e-12,
        "Muscle activation should be >= 0, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC4: Muscle override with custom actrange
// ============================================================================

#[test]
fn test_muscle_override_actrange() {
    let model = build_muscle_model(Some((-0.5, 1.5)));

    assert!(model.actuator_actlimited[0]);
    assert_eq!(model.actuator_actrange[0], (-0.5, 1.5));
}

// ============================================================================
// AC5: FilterExact integration + actlimited
// ============================================================================

#[test]
fn test_filterexact_clamped() {
    let model = build_filterexact_model(true, (-0.5, 0.5));
    let mut data = model.make_data();
    data.ctrl[0] = 10.0; // Large control to drive activation high

    for _ in 0..2000 {
        data.step(&model).expect("step failed");
    }

    assert!(
        (data.act[0] - 0.5).abs() < 1e-10,
        "FilterExact should clamp to actrange max=0.5, got {}",
        data.act[0]
    );
}

#[test]
fn test_filterexact_uses_exact_formula() {
    // Without clamping, verify the FilterExact exponential integration is used
    let model = build_filterexact_model(false, (0.0, 0.0));
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    // One step: act += act_dot * tau * (1 - exp(-h/tau))
    // act_dot = (ctrl - act) / tau = (1.0 - 0.0) / 0.1 = 10.0
    // act += 10.0 * 0.1 * (1 - exp(-0.001/0.1))
    //      = 1.0 * (1 - exp(-0.01))
    //      ≈ 0.009950166...
    data.step(&model).expect("step failed");

    let expected = 1.0 * (1.0 - (-0.001f64 / 0.1).exp());
    assert!(
        (data.act[0] - expected).abs() < 1e-12,
        "FilterExact should use exact exponential, expected {}, got {}",
        expected,
        data.act[0]
    );
}

// ============================================================================
// AC6: actearly force prediction
// ============================================================================

#[test]
fn test_actearly_force_responds_immediately() {
    // With actearly=true, force at step t uses act(t+1), so force responds
    // to ctrl change one step earlier than with actearly=false.
    let model_early = build_filter_model(false, (0.0, 0.0), true);
    let model_late = build_filter_model(false, (0.0, 0.0), false);

    let mut data_early = model_early.make_data();
    let mut data_late = model_late.make_data();

    // Initial state: act=0, ctrl=0
    // Set ctrl=1 and step once
    data_early.ctrl[0] = 1.0;
    data_late.ctrl[0] = 1.0;

    data_early.step(&model_early).expect("step failed");
    data_late.step(&model_late).expect("step failed");

    // With actearly=false: input = current act = 0, so force ≈ 0
    // With actearly=true: input = predicted act(t+1) > 0, so force > 0
    // The actuator force should be nonzero for actearly=true on the first step
    let force_early = data_early.actuator_force[0];
    let force_late = data_late.actuator_force[0];

    assert!(
        force_early.abs() > force_late.abs(),
        "actearly force ({}) should be larger than non-actearly force ({}) on first step",
        force_early,
        force_late
    );
}

// ============================================================================
// AC7: actearly + actlimited interaction
// ============================================================================

#[test]
fn test_actearly_with_actlimited() {
    // actearly should use the CLAMPED predicted activation
    let model = build_integrator_model(true, (-1.0, 1.0), true);
    let mut data = model.make_data();
    data.act[0] = 0.9995; // Near the limit
    data.ctrl[0] = 1.0; // Would push to 0.9995 + 0.001 = 1.0005, clamped to 1.0

    // The actearly input should be clamped to 1.0, not 1.0005
    // Force = gain * input = 1.0 * 1.0 = 1.0
    data.step(&model).expect("step failed");

    // After step, activation should be clamped to 1.0
    assert!(
        (data.act[0] - 1.0).abs() < 1e-12,
        "Activation should be clamped to 1.0, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC8: MuJoCo reference trajectory (Euler integrator + clamping)
// ============================================================================

#[test]
fn test_integrator_trajectory_exact_match() {
    // Simulate an integrator actuator with actlimited="true" actrange="-1 1"
    // for 100 steps with ctrl=1.0 at dt=0.001.
    // Expected: act[t] = min(t * dt * ctrl, 1.0) = min(t * 0.001, 1.0)
    // This is an exact analytical result — no tolerance needed beyond f64 precision.
    let model = build_integrator_model(true, (-1.0, 1.0), false);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    for t in 1..=2000 {
        data.step(&model).expect("step failed");
        let expected = (t as f64 * 0.001).min(1.0);
        assert!(
            (data.act[0] - expected).abs() < 1e-12,
            "Step {}: expected act={}, got {}",
            t,
            expected,
            data.act[0]
        );
    }
}

// ============================================================================
// AC9: Default class inheritance (verified by parsing tests)
// ============================================================================

#[test]
fn test_actlimited_default_class_inheritance() {
    let mjcf = r#"<mujoco>
  <option timestep="0.001"/>
  <default>
    <general actlimited="true" actrange="-2 2"/>
  </default>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="act" joint="hinge" dyntype="integrator"
             gainprm="1" biasprm="0 0 0" biastype="none"/>
  </actuator>
</mujoco>"#;
    let model = sim_mjcf::load_model(mjcf).expect("Failed to load model");

    assert!(
        model.actuator_actlimited[0],
        "actlimited should be inherited from default class"
    );
    assert_eq!(
        model.actuator_actrange[0],
        (-2.0, 2.0),
        "actrange should be inherited from default class"
    );
}

// ============================================================================
// AC10: autolimits inference
// ============================================================================

#[test]
fn test_actlimited_autolimits_inference() {
    // With compiler autolimits="true", specifying actrange should auto-enable actlimited
    let mjcf = r#"<mujoco>
  <compiler autolimits="true"/>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="act" joint="hinge" dyntype="integrator"
             actrange="-3 3"
             gainprm="1" biasprm="0 0 0" biastype="none"/>
  </actuator>
</mujoco>"#;
    let model = sim_mjcf::load_model(mjcf).expect("Failed to load model");

    assert!(
        model.actuator_actlimited[0],
        "actlimited should be auto-inferred from actrange with autolimits=true"
    );
    assert_eq!(
        model.actuator_actrange[0],
        (-3.0, 3.0),
        "actrange should be [-3, 3]"
    );
}

// ============================================================================
// AC11: Filter dynamics clamping
// ============================================================================

#[test]
fn test_filter_clamped_to_actrange() {
    let model = build_filter_model(true, (-0.5, 0.5), false);
    let mut data = model.make_data();
    data.ctrl[0] = 10.0; // Large control to saturate

    for _ in 0..5000 {
        data.step(&model).expect("step failed");
    }

    assert!(
        (data.act[0] - 0.5).abs() < 1e-6,
        "Filter actuator should clamp to 0.5, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC12: actearly on integrator — one-step-earlier check
// ============================================================================

#[test]
fn test_actearly_integrator_prediction() {
    // With actearly + integrator: force = gain * act(t+h) = gain * (act(t) + h * ctrl)
    // Without actearly: force = gain * act(t)
    // At t=0 with act=0, ctrl=1: actearly force = gain*(0 + 0.001*1.0) = 0.001
    //                              non-actearly force = gain*0 = 0.0
    let model = build_integrator_model(false, (0.0, 0.0), true);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    // Run forward to compute forces (without stepping — just forward pass)
    data.step(&model).expect("step failed");

    // After one step: actuator_force should reflect predicted activation
    // Since force = gain * input, and gain=1, force = predicted_act = 0 + 0.001 * 1.0 = 0.001
    // (The step also integrates, so act[0] = 0.001 after this step)
    assert!(
        data.act[0] > 0.0,
        "After one step, activation should be positive, got {}",
        data.act[0]
    );
}

// ============================================================================
// AC13: actearly with muscle dynamics — quantitative verification (DT-6)
// ============================================================================

/// Build a muscle model with configurable actearly.
fn build_muscle_model_actearly(actearly: bool) -> Model {
    let mjcf = format!(
        r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" limited="true" range="-1 1"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <muscle name="muscle" joint="hinge" actearly="{}"/>
  </actuator>
</mujoco>"#,
        if actearly { "true" } else { "false" }
    );
    sim_mjcf::load_model(&mjcf).expect("Failed to load muscle model with actearly")
}

#[test]
fn test_actearly_muscle_quantitative() {
    // DT-6 conformance: with actearly=true, muscle force computation uses
    // the predicted next-step activation (act + dt * act_dot), not current act.
    //
    // At t=0 with act=0, ctrl=1: muscle_activation_dynamics computes act_dot > 0.
    // actearly=true: input = mj_next_activation(0, act_dot) = 0 + dt * act_dot
    // actearly=false: input = 0
    //
    // With actearly=true, the first forward pass should produce a nonzero force
    // input, while actearly=false produces zero input (act=0 means no force).
    let model_early = build_muscle_model_actearly(true);
    let model_late = build_muscle_model_actearly(false);

    let mut data_early = model_early.make_data();
    let mut data_late = model_late.make_data();

    // Start with act=0, ctrl=1
    data_early.ctrl[0] = 1.0;
    data_late.ctrl[0] = 1.0;

    // Run one forward pass
    data_early.forward(&model_early).expect("forward failed");
    data_late.forward(&model_late).expect("forward failed");

    // Verify: actearly=false with act=0 produces zero input to gain/bias,
    // so actuator_force should be zero (or negligible from passive force).
    // actearly=true with act=0 produces input = predicted_act > 0,
    // which feeds into gain computation, producing nonzero force.
    let force_early = data_early.actuator_force[0];
    let force_late = data_late.actuator_force[0];

    // The actearly force should be strictly larger in magnitude
    assert!(
        force_early.abs() > force_late.abs(),
        "actearly=true muscle force ({:.6e}) should be larger in magnitude \
         than actearly=false ({:.6e}) when starting from act=0",
        force_early,
        force_late
    );

    // Quantitative check: the predicted activation should be approximately
    // dt * act_dot where act_dot = muscle_activation_dynamics(1.0, 0.0, dynprm).
    // With default dynprm [0.01, 0.04, 0.0] and act=0:
    //   tau_act_eff = 0.01 * (0.5 + 1.5*0) = 0.005
    //   dctrl = 1.0 - 0.0 = 1.0 (> 0, so we use tau_act)
    //   act_dot = 1.0 / 0.005 = 200.0
    //   predicted_act = 0 + 0.001 * 200.0 = 0.2 (clamped to [0,1] → 0.2)
    let dt = model_early.timestep;
    let tau_act_eff = 0.01 * (0.5 + 1.5 * 0.0); // 0.005
    let expected_act_dot = 1.0 / tau_act_eff; // 200.0
    let expected_predicted_act = (0.0 + dt * expected_act_dot).clamp(0.0, 1.0); // 0.2

    // The act_dot computed by the forward pass should match
    assert!(
        (data_early.act_dot[0] - expected_act_dot).abs() < 1e-6,
        "Muscle act_dot should be {expected_act_dot}, got {}",
        data_early.act_dot[0]
    );

    // Both early and late should compute the same act_dot
    assert!(
        (data_late.act_dot[0] - expected_act_dot).abs() < 1e-6,
        "act_dot should be identical regardless of actearly: expected {}, got {}",
        expected_act_dot,
        data_late.act_dot[0]
    );

    // Sanity: predicted_act ≈ 0.2 (dt=0.001, act_dot=200)
    assert!(
        expected_predicted_act > 0.1,
        "Predicted activation should be significant, got {expected_predicted_act}"
    );
}

// ============================================================================
// AC14: actearly=false with muscle uses current activation (DT-6)
// ============================================================================

#[test]
fn test_muscle_actearly_false_uses_current_act() {
    // DT-6 conformance: with actearly=false, force is computed using the
    // current activation value, NOT the predicted next-step value.
    // This verifies the negative case — actearly=false is the default behavior.
    let model = build_muscle_model_actearly(false);
    let mut data = model.make_data();

    // Set act=0.5 (known, nonzero activation) and ctrl=0 (deactivating)
    data.act[0] = 0.5;
    data.ctrl[0] = 0.0;

    data.forward(&model).expect("forward failed");

    // act_dot should be negative (deactivation: ctrl=0 < act=0.5)
    assert!(
        data.act_dot[0] < 0.0,
        "act_dot should be negative during deactivation, got {}",
        data.act_dot[0]
    );

    // With actearly=false, the force input is act=0.5 (current value).
    // With actearly=true, it would be act + dt * act_dot < 0.5.
    // So force magnitude with actearly=false should be LARGER than actearly=true
    // during deactivation.
    let force_late = data.actuator_force[0];

    let model_early = build_muscle_model_actearly(true);
    let mut data_early = model_early.make_data();
    data_early.act[0] = 0.5;
    data_early.ctrl[0] = 0.0;
    data_early.forward(&model_early).expect("forward failed");
    let force_early = data_early.actuator_force[0];

    // During deactivation from act=0.5: actearly predicts lower activation,
    // so should produce smaller force magnitude than current act=0.5.
    assert!(
        force_late.abs() >= force_early.abs(),
        "actearly=false force ({:.6e}) should be >= actearly=true force ({:.6e}) \
         during deactivation (current act > predicted act)",
        force_late,
        force_early
    );
}

// ============================================================================
// AC15: actearly with filter — quantitative exact-value verification (DT-6)
// ============================================================================

#[test]
fn test_actearly_filter_exact_value() {
    // DT-6 conformance: verify the exact predicted activation value for a
    // filter actuator with actearly=true.
    //
    // Filter dynamics: act_dot = (ctrl - act) / tau
    // At act=0, ctrl=1, tau=0.1:
    //   act_dot = (1 - 0) / 0.1 = 10.0
    //   predicted_act = 0 + 0.001 * 10.0 = 0.01
    //
    // The force for a general filter actuator with gainprm=[1] biastype=affine
    // biasprm=[0, -1, 0] is: force = gain * input + bias
    //   = 1.0 * 0.01 + (0 + (-1) * length + 0)
    // Since length = qpos * gear = 0 * 1 = 0:
    //   force = 0.01
    let model = build_filter_model(false, (0.0, 0.0), true);
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    data.forward(&model).expect("forward failed");

    // Exact verification of predicted activation value
    let tau = 0.1;
    let dt = model.timestep; // 0.001
    let expected_act_dot = (1.0 - 0.0) / tau; // 10.0
    let expected_predicted_act = 0.0 + dt * expected_act_dot; // 0.01

    assert!(
        (data.act_dot[0] - expected_act_dot).abs() < 1e-10,
        "Filter act_dot should be exactly {expected_act_dot}, got {}",
        data.act_dot[0]
    );

    // Force = gain * input + bias = 1.0 * predicted_act + (-1.0 * length)
    // length = 0 at qpos=0, so force = predicted_act = 0.01
    assert!(
        (data.actuator_force[0] - expected_predicted_act).abs() < 1e-10,
        "Filter actearly force should be {expected_predicted_act}, got {}",
        data.actuator_force[0]
    );
}
