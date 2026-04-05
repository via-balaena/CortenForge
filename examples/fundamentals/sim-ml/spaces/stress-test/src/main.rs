#![allow(missing_docs, clippy::expect_used, clippy::cast_precision_loss)]

use std::sync::Arc;

use sim_core::Model;
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{ActionSpace, ObservationSpace, SpaceError, Tensor};

// ── MJCF Models ─────────────────────────────────────────────────────────────

// Cart-pole: 2 joints (slide + hinge), 3 bodies (world + cart + pole),
// 4 named sensors, 1 actuator, ground plane for contacts.
const CARTPOLE_MJCF: &str = r#"
<mujoco model="cartpole-stress">
  <option timestep="0.002">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="cart" pos="0 0 0.5">
      <joint name="cart_slide" type="slide" axis="1 0 0" damping="0.1"/>
      <geom type="box" size="0.2 0.1 0.05" mass="1"/>
      <body name="pole" pos="0 0 0.05">
        <joint name="pole_hinge" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.5" mass="0.1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="cart_slide" name="force" ctrllimited="true" ctrlrange="-10 10"/>
  </actuator>
  <sensor>
    <jointpos joint="cart_slide" name="cart_pos"/>
    <jointvel joint="cart_slide" name="cart_vel"/>
    <jointpos joint="pole_hinge" name="pole_angle"/>
    <jointvel joint="pole_hinge" name="pole_angvel"/>
  </sensor>
</mujoco>
"#;

// Mocap-enabled model: a mocap target body + a pendulum with actuator.
// Enables all 5 injector types: ctrl, qfrc_applied, xfrc_applied,
// mocap_pos, mocap_quat.
const MOCAP_MJCF: &str = r#"
<mujoco model="mocap-stress">
  <option timestep="0.002"/>
  <worldbody>
    <body name="target" mocap="true" pos="0.5 0 0.5">
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
</mujoco>
"#;

fn load(mjcf: &str) -> Arc<Model> {
    Arc::new(sim_mjcf::load_model(mjcf).expect("MJCF parse"))
}

// ── Check 1: Realistic Model ────────────────────────────────────────────────

fn check_1_realistic_model() -> Check {
    let model = load(CARTPOLE_MJCF);
    let mut data = model.make_data();

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    let mut any_nan = false;
    for i in 0_u32..100 {
        let o = obs.extract(&data);
        if o.as_slice().iter().any(|v| v.is_nan()) {
            any_nan = true;
            break;
        }
        let val = (i as f32 * 0.1).sin();
        let action = Tensor::from_slice(&[val], &[1]);
        act.apply(&action, &mut data, &model);
        data.step(&model).expect("step");
    }

    Check {
        name: "Realistic model",
        pass: !any_nan,
        detail: format!(
            "100 steps, {}",
            if any_nan {
                "NaN detected"
            } else {
                "no panic/NaN"
            }
        ),
    }
}

// ── Check 2: All 13 Extractors ──────────────────────────────────────────────

fn check_2_all_extractors() -> Check {
    let model = load(CARTPOLE_MJCF);
    let data = model.make_data();

    // Cart-pole dims: nq=2, nv=2, nu=1, nsensordata=4, nbody=3
    // 13 extractor types with expected element counts:
    //   qpos(0..2)=2, qvel(0..2)=2, qacc(0..2)=2, ctrl(0..1)=1,
    //   sensordata(0..4)=4, actuator_force(0..1)=1, qfrc_constraint(0..2)=2,
    //   xpos(0..3)=9, xquat(0..3)=12, cvel(0..3)=18,
    //   sensor("cart_pos")=1, contact_count=1, time=1, energy=2
    //   Total = 58
    let expected_dim = 2 + 2 + 2 + 1 + 4 + 1 + 2 // flat fields
        + 9 + 12 + 18                              // per-body (3 bodies)
        + 1                                         // sensor("cart_pos")
        + 1 + 1 + 2; // contact_count, time, energy

    let obs = ObservationSpace::builder()
        .qpos(0..model.nq)
        .qvel(0..model.nv)
        .qacc(0..model.nv)
        .ctrl(0..model.nu)
        .sensordata(0..model.nsensordata)
        .actuator_force(0..model.nu)
        .qfrc_constraint(0..model.nv)
        .xpos(0..model.nbody)
        .xquat(0..model.nbody)
        .cvel(0..model.nbody)
        .sensor("cart_pos")
        .contact_count()
        .time()
        .energy()
        .build(&model)
        .expect("build with all 13 extractor types");

    let tensor = obs.extract(&data);
    let dim_ok = obs.dim() == expected_dim;
    let shape_ok = tensor.shape() == [expected_dim];
    let no_nan = tensor.as_slice().iter().all(|v| v.is_finite());

    let pass = dim_ok && shape_ok && no_nan;
    Check {
        name: "All 13 extractors",
        pass,
        detail: format!(
            "dim={}, expected={expected_dim}, shape_ok={shape_ok}, no_nan={no_nan}",
            obs.dim(),
        ),
    }
}

// ── Check 3: All 5 Injectors ────────────────────────────────────────────────

fn check_3_all_injectors() -> Check {
    let model = load(MOCAP_MJCF);
    let mut data = model.make_data();

    // Mocap model: nu=1, nv=1, nbody=3 (world + target + pendulum), nmocap=1
    // 5 injector types:
    //   ctrl(0..1)=1, qfrc_applied(0..1)=1, xfrc_applied(1..3)=12,
    //   mocap_pos(0..1)=3, mocap_quat(0..1)=4
    //   Total = 21
    let expected_dim = 1 + 1 + 12 + 3 + 4;

    let act = ActionSpace::builder()
        .ctrl(0..model.nu)
        .qfrc_applied(0..model.nv)
        .xfrc_applied(1..model.nbody)
        .mocap_pos(0..model.nmocap)
        .mocap_quat(0..model.nmocap)
        .build(&model)
        .expect("build with all 5 injector types");

    let dim_ok = act.dim() == expected_dim;
    let action = Tensor::zeros(&[expected_dim]);
    act.apply(&action, &mut data, &model);

    Check {
        name: "All 5 injectors",
        pass: dim_ok,
        detail: format!(
            "dim={}, expected={expected_dim}, apply succeeded",
            act.dim()
        ),
    }
}

// ── Check 4: 1000-Step Endurance ────────────────────────────────────────────

fn check_4_endurance() -> Check {
    let model = load(CARTPOLE_MJCF);
    let mut data = model.make_data();

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .sensor("cart_pos")
        .sensor("pole_angle")
        .energy()
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    let expected_shape = obs.dim();
    let mut any_nan = false;
    let mut shape_drift = false;

    for i in 0..1000_u32 {
        let o = obs.extract(&data);
        if o.shape() != [expected_shape] {
            shape_drift = true;
            break;
        }
        if o.as_slice().iter().any(|v| v.is_nan()) {
            any_nan = true;
            break;
        }
        let val = (i as f32 * 0.01).sin();
        let action = Tensor::from_slice(&[val], &[1]);
        act.apply(&action, &mut data, &model);
        data.step(&model).expect("step");
    }

    let pass = !any_nan && !shape_drift;
    Check {
        name: "1000-step endurance",
        pass,
        detail: format!(
            "1000 steps, shapes {}, {}",
            if shape_drift { "drifted" } else { "stable" },
            if any_nan { "NaN detected" } else { "no NaN" }
        ),
    }
}

// ── Check 5: Overlapping Ranges ─────────────────────────────────────────────

fn check_5_overlapping_ranges() -> Check {
    let model = load(CARTPOLE_MJCF);
    let mut data = model.make_data();

    // Cart-pole nq=2: qpos[0] = cart position, qpos[1] = pole angle.
    // qpos(0..2) gives [q0, q1], qpos(1..2) gives [q1].
    // Result: [q0, q1, q1] — 3 elements with q1 duplicated.
    let obs = ObservationSpace::builder()
        .qpos(0..2)
        .qpos(1..2)
        .build(&model)
        .expect("overlapping qpos build");

    assert_eq!(obs.dim(), 3, "dim should be 3 (2 + 1)");

    data.qpos[0] = 1.11;
    data.qpos[1] = 2.22;
    data.forward(&model).expect("forward");

    let tensor = obs.extract(&data);
    let s = tensor.as_slice();
    let first_ok = (s[0] - 1.11_f32).abs() < 1e-5;
    let second_ok = (s[1] - 2.22_f32).abs() < 1e-5;
    let dup_ok = (s[2] - 2.22_f32).abs() < 1e-5;

    let pass = first_ok && second_ok && dup_ok;
    Check {
        name: "Overlapping ranges",
        pass,
        detail: format!(
            "[{:.2}, {:.2}, {:.2}] (expected [1.11, 2.22, 2.22])",
            s[0], s[1], s[2]
        ),
    }
}

// ── Check 6: Empty Space ────────────────────────────────────────────────────

fn check_6_empty_space() -> Check {
    let model = load(CARTPOLE_MJCF);
    let data = model.make_data();

    let obs = ObservationSpace::builder()
        .build(&model)
        .expect("empty build");

    let dim_ok = obs.dim() == 0;
    let tensor = obs.extract(&data);
    let shape_ok = tensor.shape() == [0];
    let empty_ok = tensor.as_slice().is_empty();

    let pass = dim_ok && shape_ok && empty_ok;
    Check {
        name: "Empty space",
        pass,
        detail: format!("dim={}, shape={:?}", obs.dim(), tensor.shape()),
    }
}

// ── Check 7: Round-Trip Fidelity ────────────────────────────────────────────

fn check_7_round_trip() -> Check {
    let model = load(CARTPOLE_MJCF);
    let mut data = model.make_data();

    let obs = ObservationSpace::builder()
        .ctrl(0..model.nu)
        .build(&model)
        .expect("obs with ctrl");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    // Initial ctrl should be 0.
    let before = obs.extract(&data);
    let ctrl_before = before.as_slice()[0];

    // Inject a known value (within ctrlrange [-10, 10]).
    let test_val = 3.125_f32;
    let action = Tensor::from_slice(&[test_val], &[1]);
    act.apply(&action, &mut data, &model);

    // Extract again — ctrl should reflect the injected value.
    let after = obs.extract(&data);
    let ctrl_after = after.as_slice()[0];

    let before_ok = ctrl_before.abs() < 1e-6;
    let after_ok = (ctrl_after - test_val).abs() < 1e-5;

    let pass = before_ok && after_ok;
    Check {
        name: "Round-trip fidelity",
        pass,
        detail: format!("before={ctrl_before:.4}, after={ctrl_after:.4} (expected {test_val})"),
    }
}

// ── Check 8: Batch Parity ───────────────────────────────────────────────────

fn check_8_batch_parity() -> Check {
    let model = load(CARTPOLE_MJCF);
    let n_envs: usize = 8;

    // Create N data instances with different states.
    let mut datas: Vec<_> = (0..n_envs)
        .map(|i| {
            let mut d = model.make_data();
            let fi = i as f64;
            d.qpos[0] = fi * 0.1;
            d.qpos[1] = -fi * 0.05;
            d.qvel[0] = fi * 0.2;
            d.forward(&model).expect("forward");
            d
        })
        .collect();

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .energy()
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    // ── Observation parity ──
    let individual_obs: Vec<_> = datas.iter().map(|d| obs.extract(d)).collect();
    let batch_obs = obs.extract_batch(datas.iter());

    let mut obs_match = true;
    for (i, ind) in individual_obs.iter().enumerate() {
        if batch_obs.row(i) != ind.as_slice() {
            obs_match = false;
            break;
        }
    }

    // ── Action parity ──
    let actions_flat: Vec<f32> = (0..n_envs).map(|i| (i as f32).mul_add(0.5, -2.0)).collect();
    let actions = Tensor::from_slice(&actions_flat, &[n_envs, 1]);

    // Apply individually to clones.
    let mut individual_datas = datas.clone();
    for (i, d) in individual_datas.iter_mut().enumerate() {
        let a = Tensor::from_slice(&[actions_flat[i]], &[1]);
        act.apply(&a, d, &model);
    }

    // Apply batch to originals.
    act.apply_batch(&actions, datas.iter_mut(), &model);

    let mut act_match = true;
    for (i, d) in datas.iter().enumerate() {
        if (d.ctrl[0] - individual_datas[i].ctrl[0]).abs() > 1e-12 {
            act_match = false;
            break;
        }
    }

    let pass = obs_match && act_match;
    Check {
        name: "Batch parity",
        pass,
        detail: format!("{n_envs} envs, obs_match={obs_match}, act_match={act_match}"),
    }
}

// ── Check 9: Builder Errors ─────────────────────────────────────────────────

fn check_9_builder_errors() -> Check {
    let model = load(CARTPOLE_MJCF);

    // 9a: qpos range out of bounds (nq=2, request 0..10).
    let err_a = ObservationSpace::builder().qpos(0..10).build(&model);
    let a_ok = matches!(err_a, Err(SpaceError::RangeOutOfBounds { .. }));

    // 9b: xpos body range out of bounds (nbody=3, request 0..100).
    let err_b = ObservationSpace::builder().xpos(0..100).build(&model);
    let b_ok = matches!(err_b, Err(SpaceError::BodyRangeOutOfBounds { .. }));

    // 9c: nonexistent sensor name on a model with real sensors.
    let err_c = ObservationSpace::builder()
        .sensor("nonexistent_sensor_xyz")
        .build(&model);
    let c_ok = matches!(err_c, Err(SpaceError::SensorNotFound { .. }));

    let pass = a_ok && b_ok && c_ok;
    Check {
        name: "Builder errors",
        pass,
        detail: format!("range_oob={a_ok}, body_oob={b_ok}, sensor_missing={c_ok} (3/3 caught)"),
    }
}

// ── Main ────────────────────────────────────────────────────────────────────

fn main() {
    let checks = vec![
        check_1_realistic_model(),
        check_2_all_extractors(),
        check_3_all_injectors(),
        check_4_endurance(),
        check_5_overlapping_ranges(),
        check_6_empty_space(),
        check_7_round_trip(),
        check_8_batch_parity(),
        check_9_builder_errors(),
    ];

    let all_ok = print_report("Spaces Stress Test", &checks);
    if !all_ok {
        std::process::exit(1);
    }
}
