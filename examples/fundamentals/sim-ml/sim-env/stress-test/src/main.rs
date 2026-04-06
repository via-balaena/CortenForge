#![allow(missing_docs, clippy::expect_used, clippy::cast_precision_loss)]

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use sim_core::Model;
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor};

// ── MJCF Model ─────────────────────────────────────────────────────────────

const PENDULUM_MJCF: &str = r#"
<mujoco model="pendulum-stress">
  <option timestep="0.01"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

fn load() -> Arc<Model> {
    Arc::new(sim_mjcf::load_model(PENDULUM_MJCF).expect("MJCF parse"))
}

fn build_spaces(model: &Model) -> (ObservationSpace, ActionSpace) {
    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(model)
        .expect("act build");
    (obs, act)
}

// ── Check 1: 1000-Step Episode ─────────────────────────────────────────────

fn check_1_1000_step_episode() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    let mut any_nan = false;
    for i in 0..1000_u32 {
        let val = (i as f32 * 0.01).sin();
        let action = Tensor::from_slice(&[val], &[1]);
        let result = env.step(&action).expect("step");
        if result.observation.as_slice().iter().any(|v| v.is_nan()) {
            any_nan = true;
            break;
        }
    }

    Check {
        name: "1000-step episode",
        pass: !any_nan,
        detail: format!(
            "1000 steps, {}",
            if any_nan {
                "NaN detected"
            } else {
                "no panic/NaN"
            }
        ),
    }
}

// ── Check 2: Multi-Episode Endurance ───────────────────────────────────────

fn check_2_multi_episode() -> Check {
    let model = load();
    let initial_qpos = model.make_data().qpos[0];
    let (obs, act) = build_spaces(&model);

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, d| d.qpos[0].abs() > 2.0)
        .truncated(|_m, d| d.time > 1.0)
        .build()
        .expect("env build");

    let mut completed = 0_u32;
    let mut resets_ok = true;

    for _ in 0..50 {
        env.reset().expect("reset");
        let qpos = env.data().qpos[0];
        if (qpos - initial_qpos).abs() > 1e-10 {
            resets_ok = false;
            break;
        }

        loop {
            let action = Tensor::from_slice(&[1.0_f32], &[1]);
            let result = env.step(&action).expect("step");
            if result.done || result.truncated {
                break;
            }
        }
        completed += 1;
    }

    let pass = resets_ok && completed == 50;
    Check {
        name: "Multi-episode endurance",
        pass,
        detail: format!(
            "{completed}/50 episodes, state_leaks={}",
            if resets_ok { "none" } else { "detected" },
        ),
    }
}

// ── Check 3: Early Termination Timing ──────────────────────────────────────

fn check_3_early_termination() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, d| d.qpos[0].abs() > 0.1)
        .truncated(|_m, _d| false)
        .sub_steps(100)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    // Large torque pushes pendulum past threshold within a few sub-steps.
    let action = Tensor::from_slice(&[10.0_f32], &[1]);
    let result = env.step(&action).expect("step");

    let time = env.data().time;
    let max_time = 100.0 * 0.01; // sub_steps * timestep
    let early = time < max_time;

    Check {
        name: "Early termination",
        pass: result.done && early,
        detail: format!(
            "done={}, time={time:.2}s (< {max_time:.1}s = {})",
            result.done,
            if early { "early" } else { "NOT early" },
        ),
    }
}

// ── Check 4: on_reset Stateful Closure ─────────────────────────────────────

fn check_4_on_reset_counter() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    let counter = Arc::new(AtomicUsize::new(0));
    let counter_hook = counter.clone();

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, d| d.qpos[0].abs() > 0.5)
        .truncated(|_m, _d| false)
        .on_reset(move |_m, _d| {
            counter_hook.fetch_add(1, Ordering::Relaxed);
        })
        .build()
        .expect("env build");

    for _ in 0..20 {
        env.reset().expect("reset");
        let action = Tensor::from_slice(&[5.0_f32], &[1]);
        loop {
            let result = env.step(&action).expect("step");
            if result.done || result.truncated {
                break;
            }
        }
    }

    let count = counter.load(Ordering::Relaxed);
    Check {
        name: "on_reset counter",
        pass: count == 20,
        detail: format!("counter={count}, expected=20"),
    }
}

// ── Check 5: on_reset + forward() Interaction ──────────────────────────────

fn check_5_on_reset_forward() -> Check {
    let model = load();

    // Use sensor extractor — sensors are derived quantities recomputed by
    // forward(), so this verifies the reset→on_reset→forward() chain.
    let obs = ObservationSpace::builder()
        .sensor("angle")
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    let test_angle = 1.5;

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .on_reset(move |_m, d| {
            d.qpos[0] = test_angle;
        })
        .build()
        .expect("env build");

    let obs = env.reset().expect("reset");
    let sensor_val = obs.as_slice()[0];
    #[allow(clippy::cast_possible_truncation)]
    let expected = test_angle as f32;
    let ok = (sensor_val - expected).abs() < 1e-4;

    Check {
        name: "on_reset + forward()",
        pass: ok,
        detail: format!("sensor={sensor_val:.4}, expected={expected:.4}"),
    }
}

// ── Check 6: data_mut() Round-Trip ─────────────────────────────────────────

fn check_6_data_mut() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    // Modify qpos via data_mut().
    let test_val = 0.75;
    env.data_mut().qpos[0] = test_val;

    // observe() should reflect the modified qpos (all_qpos reads qpos directly).
    let obs = env.observe();
    let obs_qpos = f64::from(obs.as_slice()[0]);
    let obs_ok = (obs_qpos - test_val).abs() < 1e-4;

    // step() should run from the modified state (not snap back to 0).
    let result = env.step(&Tensor::zeros(&[1])).expect("step");
    let post_qpos = env.data().qpos[0];
    let step_ok = post_qpos.abs() > 1e-4; // didn't reset to 0
    let no_nan = result.observation.as_slice().iter().all(|v| v.is_finite());

    let pass = obs_ok && step_ok && no_nan;
    Check {
        name: "data_mut() round-trip",
        pass,
        detail: format!(
            "obs_qpos={obs_qpos:.4} (expect {test_val}), stepped_from_modified={step_ok}",
        ),
    }
}

// ── Check 7: Reward Sign Under Known State ─────────────────────────────────

fn check_7_reward_sign() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    // reward = -qpos^2 (always non-positive, negative for non-zero qpos)
    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    // Set known state: qpos = 1.0 (far from zero → large negative reward).
    env.data_mut().qpos[0] = 1.0;

    let result = env.step(&Tensor::zeros(&[1])).expect("step");

    // Reward is evaluated on the post-step state. qpos ≈ 1.0 after one
    // small timestep, so reward ≈ -1.0.
    let reward = result.reward;
    let manual = -(env.data().qpos[0].powi(2));
    let sign_ok = reward < 0.0;
    let match_ok = (reward - manual).abs() < 1e-10;

    let pass = sign_ok && match_ok;
    Check {
        name: "Reward sign",
        pass,
        detail: format!("reward={reward:.4}, manual={manual:.4}, sign_ok={sign_ok}"),
    }
}

// ── Check 8: Done Precedence Over Truncated ────────────────────────────────

fn check_8_done_precedence() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    // Recommended pattern: truncated_fn guards on !done_condition so that
    // done takes precedence when both conditions are met simultaneously.
    let threshold = 2.0;
    let time_limit = 0.5;

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(move |_m, d| d.qpos[0].abs() > threshold)
        .truncated(move |_m, d| d.qpos[0].abs() <= threshold && d.time > time_limit)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    // Force state where both conditions would trigger naively:
    // qpos > threshold AND time > time_limit.
    env.data_mut().qpos[0] = 3.0;
    env.data_mut().time = 1.0;

    let result = env.step(&Tensor::zeros(&[1])).expect("step");

    let pass = result.done && !result.truncated;
    Check {
        name: "Done precedence",
        pass,
        detail: format!(
            "done={}, truncated={} (expected true, false)",
            result.done, result.truncated,
        ),
    }
}

// ── Check 9: Sub-Steps Parity ──────────────────────────────────────────────

fn check_9_sub_steps_parity() -> Check {
    let model = load();

    let (obs1, act1) = build_spaces(&model);
    let mut env1 = SimEnv::builder(model.clone())
        .observation_space(obs1)
        .action_space(act1)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .sub_steps(1)
        .build()
        .expect("env1 build");

    let (obs5, act5) = build_spaces(&model);
    let mut env5 = SimEnv::builder(model)
        .observation_space(obs5)
        .action_space(act5)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .sub_steps(5)
        .build()
        .expect("env5 build");

    env1.reset().expect("reset");
    env5.reset().expect("reset");

    let action = Tensor::zeros(&[1]);

    // 5 individual steps with sub_steps=1 …
    for _ in 0..5 {
        env1.step(&action).expect("step");
    }
    // … vs 1 step with sub_steps=5.
    env5.step(&action).expect("step");

    let qpos_diff = (env1.data().qpos[0] - env5.data().qpos[0]).abs();
    let qvel_diff = (env1.data().qvel[0] - env5.data().qvel[0]).abs();
    let time_diff = (env1.data().time - env5.data().time).abs();

    let pass = qpos_diff < 1e-12 && qvel_diff < 1e-12 && time_diff < 1e-12;
    Check {
        name: "Sub-steps parity",
        pass,
        detail: format!(
            "qpos_diff={qpos_diff:.2e}, qvel_diff={qvel_diff:.2e}, time_diff={time_diff:.2e}",
        ),
    }
}

// ── Check 10: observe() Consistency ────────────────────────────────────────

fn check_10_observe_consistency() -> Check {
    let model = load();
    let (obs, act) = build_spaces(&model);

    let mut env = SimEnv::builder(model)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    // Step a few times to reach a non-trivial state.
    for i in 0..10_u32 {
        let val = (i as f32 * 0.3).sin();
        let action = Tensor::from_slice(&[val], &[1]);
        env.step(&action).expect("step");
    }

    let obs1 = env.observe();
    let obs2 = env.observation_space().extract(env.data());

    let match_ok = obs1.as_slice() == obs2.as_slice();

    Check {
        name: "observe() consistency",
        pass: match_ok,
        detail: format!("observe() == extract(): {match_ok}"),
    }
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let checks = vec![
        check_1_1000_step_episode(),
        check_2_multi_episode(),
        check_3_early_termination(),
        check_4_on_reset_counter(),
        check_5_on_reset_forward(),
        check_6_data_mut(),
        check_7_reward_sign(),
        check_8_done_precedence(),
        check_9_sub_steps_parity(),
        check_10_observe_consistency(),
    ];

    let all_ok = print_report("SimEnv Stress Test", &checks);
    if !all_ok {
        std::process::exit(1);
    }
}
