#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::float_cmp
)]

use std::sync::Arc;

use sim_core::Model;
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::{
    ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv, VecStepError,
};

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

// ── Check 1: Bit-Exact Parity (100 Steps) ─────────────────────────────────

fn check_1_bit_exact_parity() -> Check {
    let model = load();
    let n = 4;

    // Build VecEnv
    let (obs_v, act_v) = build_spaces(&model);
    let mut vec_env = VecEnv::builder(model.clone(), n)
        .observation_space(obs_v)
        .action_space(act_v)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, d| d.qpos[0].abs() > 3.0)
        .truncated(|_m, d| d.time > 10.0)
        .build()
        .expect("vec_env build");

    // Build N individual SimEnvs
    let mut sim_envs: Vec<SimEnv> = (0..n)
        .map(|_| {
            let (obs, act) = build_spaces(&model);
            SimEnv::builder(model.clone())
                .observation_space(obs)
                .action_space(act)
                .reward(|_m, d| -d.qpos[0].powi(2))
                .done(|_m, d| d.qpos[0].abs() > 3.0)
                .truncated(|_m, d| d.time > 10.0)
                .build()
                .expect("sim_env build")
        })
        .collect();

    vec_env.reset_all().expect("reset");
    for env in &mut sim_envs {
        env.reset().expect("reset");
    }

    let action_vals: [f32; 4] = [0.0, 0.5, 1.0, 1.5];
    let actions = Tensor::from_slice(&action_vals, &[n, 1]);

    let mut mismatches = 0_u32;
    for _ in 0..100 {
        let vr = vec_env.step(&actions).expect("vec step");

        for (i, env) in sim_envs.iter_mut().enumerate() {
            let a = Tensor::from_slice(&[action_vals[i]], &[1]);
            let sr = env.step(&a).expect("sim step");

            if vr.observations.row(i) != sr.observation.as_slice() {
                mismatches += 1;
            }
            if vr.rewards[i] != sr.reward {
                mismatches += 1;
            }
            if vr.dones[i] != sr.done {
                mismatches += 1;
            }
            if vr.truncateds[i] != sr.truncated {
                mismatches += 1;
            }
        }
    }

    Check {
        name: "Bit-exact parity (100 steps)",
        pass: mismatches == 0,
        detail: format!("4 envs × 100 steps, mismatches={mismatches}"),
    }
}

// ── Check 2: 100-Step Endurance With Auto-Resets ──────────────────────────

fn check_2_endurance_auto_resets() -> Check {
    let model = load();
    let n = 64;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, d| d.qpos[0].abs() > 1.5)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    // Constant torque = 5.0 to trigger fast done
    let action_data: Vec<f32> = vec![5.0; n];
    let actions = Tensor::from_slice(&action_data, &[n, 1]);

    let mut total_resets = 0_u32;
    let mut any_nan = false;

    for _ in 0..100 {
        let result = env.step(&actions).expect("step");
        for &d in &result.dones {
            if d {
                total_resets += 1;
            }
        }
        if result.observations.as_slice().iter().any(|v| v.is_nan()) {
            any_nan = true;
            break;
        }
    }

    let pass = !any_nan && total_resets > 0;
    Check {
        name: "100-step endurance + auto-resets",
        pass,
        detail: format!(
            "64 envs, 100 steps, resets={total_resets}, nan={}",
            if any_nan { "yes" } else { "no" },
        ),
    }
}

// ── Check 3: Auto-Reset terminal_obs Correctness ─────────────────────────

fn check_3_terminal_obs() -> Check {
    let model = load();
    let n = 4;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, d| d.qpos[0].abs() > 1.0)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    // Push env 0 past threshold
    if let Some(data) = env.batch_mut().env_mut(0) {
        data.qpos[0] = 1.5;
    }

    let actions = Tensor::zeros(&[n, 1]);
    let result = env.step(&actions).expect("step");

    let term_some = result.terminal_observations[0].is_some();
    let term_qpos_ok = result.terminal_observations[0]
        .as_ref()
        .is_some_and(|t| t.as_slice()[0].abs() > 1.0);
    let post_reset_ok = result.observations.row(0)[0].abs() < 0.1;
    let others_none = (1..n).all(|i| result.terminal_observations[i].is_none());

    let pass = term_some && term_qpos_ok && post_reset_ok && others_none;
    Check {
        name: "Auto-reset terminal_obs",
        pass,
        detail: format!(
            "term_some={term_some}, term_qpos_ok={term_qpos_ok}, \
             post_reset_ok={post_reset_ok}, others_none={others_none}",
        ),
    }
}

// ── Check 4: Non-Reset Env terminal_obs is None ──────────────────────────

fn check_4_no_reset_no_terminal() -> Check {
    let model = load();
    let n = 4;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, d| d.qpos[0].abs() > 10.0)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    let actions = Tensor::zeros(&[n, 1]);
    let mut all_none = true;

    for _ in 0..10 {
        let result = env.step(&actions).expect("step");
        if result.terminal_observations.iter().any(Option::is_some) {
            all_none = false;
            break;
        }
    }

    Check {
        name: "Non-reset terminal_obs = None",
        pass: all_none,
        detail: format!(
            "10 steps, 4 envs, all terminal_obs None = {}",
            if all_none { "yes" } else { "no" },
        ),
    }
}

// ── Check 5: on_reset env_index Correctness ──────────────────────────────

fn check_5_on_reset_env_index() -> Check {
    let model = load();
    let n = 8;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .on_reset(|_m, data, env_idx| {
            #[allow(clippy::cast_precision_loss)]
            {
                data.qpos[0] = env_idx as f64 * 0.1;
            }
        })
        .build()
        .expect("build");

    let obs = env.reset_all().expect("reset");

    let mut all_ok = true;
    for i in 0..n {
        #[allow(clippy::cast_precision_loss)]
        let expected = (i as f64 * 0.1) as f32;
        let actual = obs.row(i)[0];
        if (actual - expected).abs() > 1e-4 {
            all_ok = false;
            break;
        }
    }

    Check {
        name: "on_reset env_index",
        pass: all_ok,
        detail: format!(
            "8 envs, qpos[i] = i*0.1, match = {}",
            if all_ok { "yes" } else { "no" },
        ),
    }
}

// ── Check 6: Sub-Stepping: All Sub-Steps Run ─────────────────────────────

fn check_6_sub_steps() -> Check {
    let model = load();
    let n = 4;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, d| d.qpos[0].abs() > 100.0)
        .truncated(|_m, _d| false)
        .sub_steps(10)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    let actions = Tensor::zeros(&[n, 1]);
    env.step(&actions).expect("step");

    let expected_time = 10.0 * 0.01; // 0.10
    let mut all_ok = true;

    for i in 0..n {
        let time = env.batch().env(i).expect("env").time;
        if (time - expected_time).abs() > 1e-10 {
            all_ok = false;
            break;
        }
    }

    Check {
        name: "Sub-steps all run",
        pass: all_ok,
        detail: format!(
            "sub_steps=10, expected time={expected_time}, match = {}",
            if all_ok { "yes" } else { "no" },
        ),
    }
}

// ── Check 7: Wrong Action Shape ──────────────────────────────────────────

fn check_7_wrong_action_shape() -> Check {
    let model = load();
    let n = 4;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    // Wrong number of envs: [3, 1] instead of [4, 1]
    let bad1 = Tensor::zeros(&[3, 1]);
    let err1 = env.step(&bad1);
    let err1_ok = matches!(err1, Err(VecStepError::ActionShapeMismatch { .. }));

    // Wrong action dim: [4, 2] instead of [4, 1]
    let bad2 = Tensor::zeros(&[n, 2]);
    let err2 = env.step(&bad2);
    let err2_ok = matches!(err2, Err(VecStepError::ActionShapeMismatch { .. }));

    let pass = err1_ok && err2_ok;
    Check {
        name: "Wrong action shape",
        pass,
        detail: format!("wrong_n_envs={err1_ok}, wrong_dim={err2_ok}"),
    }
}

// ── Check 8: Per-Env Action Isolation (50 Steps) ─────────────────────────

fn check_8_action_isolation() -> Check {
    let model = load();
    let n = 4;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    let action_vals: [f32; 4] = [0.0, 1.0, -1.0, 2.0];
    let actions = Tensor::from_slice(&action_vals, &[n, 1]);

    let mut result = env.step(&actions).expect("step");
    for _ in 1..50 {
        result = env.step(&actions).expect("step");
    }

    // All 4 envs should have distinct observations after 50 steps
    let mut all_distinct = true;
    for i in 0..n {
        for j in (i + 1)..n {
            if result.observations.row(i) == result.observations.row(j) {
                all_distinct = false;
            }
        }
    }

    // Env 3 (action=2.0) should have largest magnitude qpos
    let qpos: Vec<f32> = (0..n).map(|i| result.observations.row(i)[0]).collect();
    let max_mag_env = qpos
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).expect("no NaN"))
        .map(|(i, _)| i)
        .expect("non-empty");

    let pass = all_distinct && max_mag_env == 3;
    Check {
        name: "Per-env action isolation (50 steps)",
        pass,
        detail: format!(
            "all_distinct={all_distinct}, max_mag_env={max_mag_env} (expected 3), \
             qpos=[{:.2}, {:.2}, {:.2}, {:.2}]",
            qpos[0], qpos[1], qpos[2], qpos[3],
        ),
    }
}

// ── Check 9: Done Precedence Over Truncated ──────────────────────────────

fn check_9_done_precedence() -> Check {
    let model = load();
    let n = 2;
    let threshold = 2.0;
    let time_limit = 0.5;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(move |_m, d| d.qpos[0].abs() > threshold)
        .truncated(move |_m, d| d.qpos[0].abs() <= threshold && d.time > time_limit)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    // Force state where both conditions would trigger naively
    if let Some(data) = env.batch_mut().env_mut(0) {
        data.qpos[0] = 3.0;
        data.time = 1.0;
    }

    let actions = Tensor::zeros(&[n, 1]);
    let result = env.step(&actions).expect("step");

    let pass = result.dones[0] && !result.truncateds[0];
    Check {
        name: "Done precedence",
        pass,
        detail: format!(
            "done={}, truncated={} (expected true, false)",
            result.dones[0], result.truncateds[0],
        ),
    }
}

// ── Check 10: reset_all Correctness ──────────────────────────────────────

fn check_10_reset_all() -> Check {
    let model = load();
    let n = 8;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    // Step a few times to reach non-trivial state
    let action_data: Vec<f32> = (0..n).map(|i| (i as f32) * 0.5).collect();
    let actions = Tensor::from_slice(&action_data, &[n, 1]);
    for _ in 0..20 {
        env.step(&actions).expect("step");
    }

    // Now reset
    let obs = env.reset_all().expect("reset_all");

    let shape_ok = obs.shape() == [n, 2];
    let mut all_near_zero = true;
    for i in 0..n {
        if obs.row(i)[0].abs() > 1e-4 {
            all_near_zero = false;
            break;
        }
    }

    let pass = shape_ok && all_near_zero;
    Check {
        name: "reset_all correctness",
        pass,
        detail: format!(
            "shape={:?} (expected [8,2]), all_near_zero={all_near_zero}",
            obs.shape()
        ),
    }
}

// ── Check 11: VecStepResult Field Lengths ────────────────────────────────

fn check_11_field_lengths() -> Check {
    let model = load();
    let n = 16;

    let (obs, act) = build_spaces(&model);
    let mut env = VecEnv::builder(model, n)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(|_m, _d| false)
        .build()
        .expect("build");

    env.reset_all().expect("reset");

    let actions = Tensor::zeros(&[n, 1]);
    let result = env.step(&actions).expect("step");

    let obs_ok = result.observations.shape() == [n, 2];
    let rew_ok = result.rewards.len() == n;
    let don_ok = result.dones.len() == n;
    let tru_ok = result.truncateds.len() == n;
    let ter_ok = result.terminal_observations.len() == n;
    let err_ok = result.errors.len() == n;

    let pass = obs_ok && rew_ok && don_ok && tru_ok && ter_ok && err_ok;
    Check {
        name: "VecStepResult field lengths",
        pass,
        detail: format!(
            "obs={obs_ok}, rew={rew_ok}, don={don_ok}, \
             tru={tru_ok}, ter={ter_ok}, err={err_ok} (all == {n})",
        ),
    }
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let checks = vec![
        check_1_bit_exact_parity(),
        check_2_endurance_auto_resets(),
        check_3_terminal_obs(),
        check_4_no_reset_no_terminal(),
        check_5_on_reset_env_index(),
        check_6_sub_steps(),
        check_7_wrong_action_shape(),
        check_8_action_isolation(),
        check_9_done_precedence(),
        check_10_reset_all(),
        check_11_field_lengths(),
    ];

    let all_ok = print_report("VecEnv Stress Test", &checks);
    if !all_ok {
        std::process::exit(1);
    }
}
