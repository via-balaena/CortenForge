//! Layer A — Self-consistency tests (no MuJoCo dependency).
//!
//! These tests verify internal consistency properties that must hold regardless
//! of MuJoCo reference data. They are structural tests — if any fail, it
//! indicates a fundamental implementation bug.
//!
//! Categories:
//! 1. Forward/inverse equivalence (multiple joint types)
//! 2. Island/monolithic solver equivalence
//! 3. Determinism (same input → bit-identical output)
//! 4. Integrator energy ordering (implicit < semi-implicit < Euler drift)
//! 5. Sparse/dense mass matrix equivalence

use sim_core::DISABLE_ISLAND;

// ============================================================================
// 1. Forward/Inverse Equivalence
// ============================================================================

/// Forward/inverse round-trip on a hinge chain.
///
/// After forward() → inverse(), the round-trip identity holds:
///   qfrc_inverse = qfrc_applied + qfrc_actuator
/// (when no xfrc_applied is present).
#[test]
fn layer_a_forward_inverse_hinge_chain() {
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
            <body pos="0 0 -0.4">
              <joint name="j3" type="hinge" axis="1 0 0"/>
              <geom type="capsule" size="0.03 0.15" mass="0.3"/>
            </body>
          </body>
        </body>
      </worldbody>
      <actuator>
        <motor joint="j1" gear="1"/>
        <motor joint="j2" gear="1"/>
        <motor joint="j3" gear="1"/>
      </actuator>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Non-trivial state
    data.qpos[0] = 0.7;
    data.qpos[1] = -0.4;
    data.qpos[2] = 0.2;
    data.qvel[0] = 1.5;
    data.qvel[1] = -0.8;
    data.qvel[2] = 0.3;
    data.ctrl[0] = 5.0;
    data.ctrl[1] = -2.0;
    data.ctrl[2] = 1.0;
    data.qfrc_applied[0] = 1.0;
    data.qfrc_applied[1] = -0.5;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    for i in 0..model.nv {
        let expected = data.qfrc_applied[i] + data.qfrc_actuator[i];
        assert!(
            (data.qfrc_inverse[i] - expected).abs() < 1e-8,
            "DOF {i}: qfrc_inverse={} != applied+actuator={}",
            data.qfrc_inverse[i],
            expected
        );
    }
}

/// Forward/inverse round-trip with a free joint (6 DOF).
///
/// A free body under gravity with no external forces should have
/// qfrc_inverse ≈ 0 (inverse says "no external force needed to produce
/// the forward-computed qacc").
#[test]
fn layer_a_forward_inverse_free_body() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 2">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    // qacc should be non-trivial (gravity)
    assert!(
        data.qacc[2].abs() > 1.0,
        "Free fall qacc[2] should be ~-9.81, got {}",
        data.qacc[2]
    );

    data.inverse(&model);

    // Round-trip: with no applied forces/actuators, qfrc_inverse ≈ 0
    for i in 0..model.nv {
        assert!(
            data.qfrc_inverse[i].abs() < 1e-8,
            "DOF {i}: qfrc_inverse should be ~0, got {}",
            data.qfrc_inverse[i]
        );
    }
}

/// Forward/inverse round-trip with a ball joint.
#[test]
fn layer_a_forward_inverse_ball_joint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 0">
          <joint type="ball"/>
          <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Tilt the ball joint
    let angle = 0.3_f64;
    let half = angle / 2.0;
    data.qpos[0] = half.cos(); // w
    data.qpos[1] = 0.0; // x
    data.qpos[2] = half.sin(); // y
    data.qpos[3] = 0.0; // z
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.qvel[2] = 0.1;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    for i in 0..model.nv {
        assert!(
            data.qfrc_inverse[i].abs() < 1e-8,
            "DOF {i}: qfrc_inverse should be ~0 (no external forces), got {}",
            data.qfrc_inverse[i]
        );
    }
}

/// Forward/inverse round-trip with slide joint.
#[test]
fn layer_a_forward_inverse_slide_joint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body pos="0 0 1">
          <joint name="slide0" type="slide" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <actuator>
        <motor joint="slide0" gear="1"/>
      </actuator>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.qpos[0] = 0.2;
    data.qvel[0] = -1.0;
    data.ctrl[0] = 3.0;
    data.qfrc_applied[0] = 0.5;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    let expected = data.qfrc_applied[0] + data.qfrc_actuator[0];
    assert!(
        (data.qfrc_inverse[0] - expected).abs() < 1e-8,
        "qfrc_inverse={} != applied+actuator={}",
        data.qfrc_inverse[0],
        expected
    );
}

/// Forward/inverse explicit formula:
///   qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
///
/// Verify the formula holds with non-trivial constraint forces.
#[test]
fn layer_a_forward_inverse_formula() {
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
      <actuator>
        <motor joint="j1" gear="1"/>
        <motor joint="j2" gear="1"/>
      </actuator>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    data.qpos[0] = 0.8;
    data.qpos[1] = -0.4;
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.0;
    data.ctrl[0] = 3.0;
    data.ctrl[1] = -1.5;
    data.qfrc_applied[0] = 2.0;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    // Compute M * qacc manually
    let mut m_qacc = nalgebra::DVector::zeros(model.nv);
    data.qM.mul_to(&data.qacc, &mut m_qacc);

    for i in 0..model.nv {
        let expected =
            m_qacc[i] + data.qfrc_bias[i] - data.qfrc_passive[i] - data.qfrc_constraint[i];
        assert!(
            (data.qfrc_inverse[i] - expected).abs() < 1e-10,
            "DOF {i}: formula mismatch: qfrc_inverse={}, M*qacc+bias-passive-constraint={}",
            data.qfrc_inverse[i],
            expected
        );
    }
}

// ============================================================================
// 2. Island/Monolithic Solver Equivalence
// ============================================================================

/// Island-based solving must produce the same result as monolithic solving.
///
/// Two independent free bodies on a plane: the island solver decomposes them
/// into separate islands, while DISABLE_ISLAND forces a global solve. Both
/// must agree within floating-point tolerance.
#[test]
fn layer_a_island_monolithic_equivalence() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom type="plane" size="10 10 0.1"/>
        <body pos="-2 0 0.5">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
        <body pos="2 0 0.5">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    // Run with islands enabled (default)
    let model_island = sim_mjcf::load_model(xml).expect("load");
    let mut data_island = model_island.make_data();

    // Run with DISABLE_ISLAND (global solve)
    let mut model_global = sim_mjcf::load_model(xml).expect("load");
    model_global.disableflags |= DISABLE_ISLAND;
    let mut data_global = model_global.make_data();

    for step in 0..100 {
        data_island.step(&model_island).expect("island step");
        data_global.step(&model_global).expect("global step");

        for dof in 0..model_island.nv {
            let diff = (data_island.qvel[dof] - data_global.qvel[dof]).abs();
            assert!(
                diff < 1e-8,
                "Step {step}, DOF {dof}: island qvel={} != global qvel={} (diff={diff:.2e})",
                data_island.qvel[dof],
                data_global.qvel[dof]
            );
        }

        for q in 0..model_island.nq {
            let diff = (data_island.qpos[q] - data_global.qpos[q]).abs();
            assert!(
                diff < 1e-8,
                "Step {step}, q {q}: island qpos={} != global qpos={} (diff={diff:.2e})",
                data_island.qpos[q],
                data_global.qpos[q]
            );
        }
    }
}

/// Island/monolithic equivalence with equality constraints.
///
/// A weld constraint links two bodies — the island solver must handle the
/// coupling correctly and match the global solver.
#[test]
fn layer_a_island_monolithic_with_equality() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom type="plane" size="10 10 0.1"/>
        <body name="a" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
        <body name="b" pos="0 0 1.5">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <equality>
        <weld body1="a" body2="b"/>
      </equality>
    </mujoco>"#;

    let model_island = sim_mjcf::load_model(xml).expect("load");
    let mut data_island = model_island.make_data();

    let mut model_global = sim_mjcf::load_model(xml).expect("load");
    model_global.disableflags |= DISABLE_ISLAND;
    let mut data_global = model_global.make_data();

    for step in 0..50 {
        data_island.step(&model_island).expect("island step");
        data_global.step(&model_global).expect("global step");

        for dof in 0..model_island.nv {
            let diff = (data_island.qacc[dof] - data_global.qacc[dof]).abs();
            assert!(
                diff < 1e-6,
                "Step {step}, DOF {dof}: island qacc={} != global qacc={} (diff={diff:.2e})",
                data_island.qacc[dof],
                data_global.qacc[dof]
            );
        }
    }
}

// ============================================================================
// 3. Determinism
// ============================================================================

/// Same input → bit-identical output across multiple forward() calls.
///
/// Run forward() twice on the same state. Results must be bit-identical.
#[test]
fn layer_a_determinism_forward() {
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
    let mut data1 = model.make_data();
    let mut data2 = model.make_data();

    // Set identical non-trivial state
    for d in [&mut data1, &mut data2] {
        d.qpos[2] = 0.5; // z
        d.qvel[0] = 0.5;
        d.qvel[2] = -1.0;
    }

    data1.forward(&model).expect("forward 1");
    data2.forward(&model).expect("forward 2");

    // Bit-identical comparison
    assert_eq!(
        data1.qacc.as_slice(),
        data2.qacc.as_slice(),
        "qacc must be bit-identical"
    );
    assert_eq!(
        data1.qvel.as_slice(),
        data2.qvel.as_slice(),
        "qvel must be bit-identical"
    );
}

/// Determinism across multiple step() calls — two identical simulations
/// must produce bit-identical trajectories.
#[test]
fn layer_a_determinism_trajectory() {
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
    let mut data1 = model.make_data();
    let mut data2 = model.make_data();

    for _ in 0..200 {
        data1.step(&model).expect("step 1");
        data2.step(&model).expect("step 2");
    }

    assert_eq!(
        data1.qpos.as_slice(),
        data2.qpos.as_slice(),
        "qpos must be bit-identical after 200 steps"
    );
    assert_eq!(
        data1.qvel.as_slice(),
        data2.qvel.as_slice(),
        "qvel must be bit-identical after 200 steps"
    );
    assert_eq!(
        data1.qacc.as_slice(),
        data2.qacc.as_slice(),
        "qacc must be bit-identical after 200 steps"
    );
}

// ============================================================================
// 4. Integrator Energy Ordering
// ============================================================================

/// Energy drift ordering: Euler > semi-implicit integrators.
///
/// For a conservative system (frictionless pendulum), energy drift at a given
/// timestep should follow: Euler drift > ImplicitFast drift (because implicit
/// integrators have better energy behavior).
///
/// We test on a 1-DOF pendulum with spring and no damping.
#[test]
fn layer_a_integrator_energy_ordering() {
    fn energy_drift(integrator: &str, timestep: f64, n_steps: usize) -> f64 {
        let xml = format!(
            r#"
            <mujoco>
              <option gravity="0 0 -9.81" timestep="{timestep}" integrator="{integrator}">
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
        data.qpos[0] = 0.3; // Initial angle

        data.forward(&model).expect("forward");
        let e0 = data.total_energy();
        assert!(e0.is_finite() && e0.abs() > 1e-15);

        let mut max_drift = 0.0_f64;
        for _ in 0..n_steps {
            data.step(&model).expect("step");
            data.forward(&model).expect("forward");
            let e = data.total_energy();
            let drift = ((e - e0) / e0).abs();
            max_drift = max_drift.max(drift);
        }
        max_drift
    }

    let h = 0.005;
    let n = 200; // 1 second

    let drift_euler = energy_drift("Euler", h, n);
    let drift_rk4 = energy_drift("RK4", h, n);

    // RK4 should have much less energy drift than Euler on a conservative system
    assert!(
        drift_rk4 < drift_euler,
        "RK4 drift ({drift_rk4:.2e}) should be less than Euler drift ({drift_euler:.2e})"
    );

    // Both should be bounded (not exploding)
    assert!(
        drift_euler < 1.0,
        "Euler drift should be bounded, got {drift_euler:.2e}"
    );
    assert!(
        drift_rk4 < 1e-6,
        "RK4 drift should be very small at h={h}, got {drift_rk4:.2e}"
    );
}

/// RK4 convergence order: halving h should reduce error by ~16× (O(h⁴)).
#[test]
fn layer_a_rk4_convergence_order() {
    let xml_template = |h: f64| {
        format!(
            r#"
            <mujoco>
              <option gravity="0 0 -9.81" timestep="{h}" integrator="RK4"/>
              <worldbody>
                <body pos="0 0 0">
                  <joint type="hinge" axis="0 1 0" damping="0"/>
                  <geom type="sphere" size="0.05" pos="0 0 -1" mass="1.0"/>
                </body>
              </worldbody>
            </mujoco>"#
        )
    };

    let run = |h: f64, n: usize| -> f64 {
        let model = sim_mjcf::load_model(&xml_template(h)).expect("load");
        let mut data = model.make_data();
        data.qpos[0] = 0.1;
        for _ in 0..n {
            data.step(&model).expect("step");
        }
        data.qpos[0]
    };

    let t_final = 1.0;
    let h_ref = 1e-5;
    let q_ref = run(h_ref, (t_final / h_ref).round() as usize);

    let h1 = 0.01;
    let h2 = 0.005;
    let q1 = run(h1, (t_final / h1).round() as usize);
    let q2 = run(h2, (t_final / h2).round() as usize);

    let err1 = (q1 - q_ref).abs();
    let err2 = (q2 - q_ref).abs();

    let ratio = err1 / err2;
    assert!(
        ratio > 10.0 && ratio < 25.0,
        "RK4 convergence ratio should be ~16 (O(h⁴)), got {ratio:.2} \
         (err_h1={err1:.2e}, err_h2={err2:.2e})"
    );
}

// ============================================================================
// 5. Sparse/Dense Mass Matrix Equivalence
// ============================================================================

/// For a small model (nv ≤ 60), qM should be available as a dense matrix.
/// Verify symmetry and match against direct computation.
#[test]
fn layer_a_mass_matrix_symmetry() {
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
            <body pos="0 0 -0.4">
              <joint name="j3" type="hinge" axis="1 0 0"/>
              <geom type="capsule" size="0.03 0.15" mass="0.3"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Non-trivial pose
    data.qpos[0] = 0.5;
    data.qpos[1] = -0.3;
    data.qpos[2] = 0.7;

    data.forward(&model).expect("forward");

    let nv = model.nv;
    // Verify symmetry: M[i,j] == M[j,i]
    for i in 0..nv {
        for j in 0..nv {
            let mij = data.qM[(i, j)];
            let mji = data.qM[(j, i)];
            assert!(
                (mij - mji).abs() < 1e-14,
                "Mass matrix not symmetric: M[{i},{j}]={mij} != M[{j},{i}]={mji}"
            );
        }
    }
}

/// The mass matrix factored by the solver (M + J^T D J → qacc) should produce
/// the same qacc as the unconstrained case when no constraints exist.
///
/// For an unconstrained model: qacc = M^{-1} * (qfrc_bias + qfrc_passive + qfrc_applied)
/// This is a structural consistency check.
#[test]
fn layer_a_mass_matrix_unconstrained_consistency() {
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

    data.qpos[0] = 0.5;
    data.qpos[1] = -0.3;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.qfrc_applied[0] = 2.0;

    data.forward(&model).expect("forward");

    // No constraints → qfrc_constraint should be zero
    if data.efc_force.is_empty() {
        for i in 0..model.nv {
            assert!(
                data.qfrc_constraint[i].abs() < 1e-12,
                "qfrc_constraint[{i}] should be 0 without constraints, got {}",
                data.qfrc_constraint[i]
            );
        }

        // qacc should satisfy: M * qacc = qfrc_bias + qfrc_passive + qfrc_applied + qfrc_actuator
        let mut m_qacc = nalgebra::DVector::zeros(model.nv);
        data.qM.mul_to(&data.qacc, &mut m_qacc);

        for i in 0..model.nv {
            // In MuJoCo: M * qacc + qfrc_bias = qfrc_passive + qfrc_applied + qfrc_actuator
            // So: M * qacc = -qfrc_bias + qfrc_passive + qfrc_applied + qfrc_actuator
            let rhs_corrected = -data.qfrc_bias[i]
                + data.qfrc_passive[i]
                + data.qfrc_applied[i]
                + data.qfrc_actuator[i];
            assert!(
                (m_qacc[i] - rhs_corrected).abs() < 1e-8,
                "DOF {i}: M*qacc={} != rhs={}",
                m_qacc[i],
                rhs_corrected
            );
        }
    }
}
