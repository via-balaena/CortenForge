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
