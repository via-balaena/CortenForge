//! 6-DOF Stress Test — headless integration checks
//!
//! Validates 6-DOF reaching task infrastructure before investing in visual
//! examples. Covers `VecEnv` parity, observation scaling, done/truncated
//! conditions, policy construction for all three architectures (linear,
//! MLP, autograd), and endurance.
//!
//! Run: `cargo run -p example-ml-6dof-stress-test --release`

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::needless_range_loop,
    clippy::similar_names,
    clippy::unwrap_used,
    clippy::suboptimal_flops
)]

use std::sync::Arc;

use rand::SeedableRng;
use rand::rngs::StdRng;
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::{
    Activation, AutogradStochasticPolicy, Environment, LinearPolicy, MlpPolicy, Policy, SimEnv,
    StochasticPolicy, Tensor, VecEnv, reaching_6dof,
};

// ── Check 1: VecEnv parity ──────────────────────────────────────────────

fn check_vec_env_parity() -> Check {
    let task = reaching_6dof();
    let n = 50;

    let mut vec_env = task.build_vec_env(n, 0).expect("build vec_env");
    vec_env.reset_all().expect("reset vec_env");

    // Build N sequential SimEnvs with identical config.
    let model = Arc::new(vec_env.model().clone());
    let _obs = sim_ml_chassis::ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs");
    let _act = sim_ml_chassis::ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act");

    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];

    let mut sim_envs: Vec<SimEnv> = (0..n)
        .map(|_| {
            let o = sim_ml_chassis::ObservationSpace::builder()
                .all_qpos()
                .all_qvel()
                .build(&model)
                .expect("obs");
            let a = sim_ml_chassis::ActionSpace::builder()
                .all_ctrl()
                .build(&model)
                .expect("act");
            let tq = target_joints;
            let mut env = SimEnv::builder(model.clone())
                .observation_space(o)
                .action_space(a)
                .reward(move |_m, d| {
                    let mut err = 0.0;
                    for j in 0..6 {
                        err += (d.qpos[j] - tq[j]).powi(2);
                    }
                    -err
                })
                .done(|_m, d| {
                    let dist_sq = d
                        .site_xpos
                        .first()
                        .map_or(f64::MAX, |s| s.x * s.x + s.y * s.y + s.z * s.z);
                    dist_sq < 0.05 * 0.05
                })
                .truncated(|_m, d| d.time > 5.0)
                .sub_steps(5)
                .build()
                .expect("sim_env");
            env.reset().expect("reset");
            env
        })
        .collect();

    // Step both with zero actions for 20 steps
    let actions = Tensor::zeros(&[n, 6]);
    let sim_action = Tensor::zeros(&[1, 6]);
    let mut parity_ok = true;

    for _ in 0..20 {
        let vr = vec_env.step(&actions).expect("vec step");
        for (i, env) in sim_envs.iter_mut().enumerate() {
            let sr = env.step(&sim_action).expect("sim step");
            let vec_obs = vr.observations.row(i);
            let sim_obs = sr.observation.as_slice();
            if vec_obs != sim_obs {
                parity_ok = false;
            }
        }
    }

    Check {
        name: "6-DOF VecEnv parity (50 envs, 20 steps)",
        pass: parity_ok,
        detail: format!("parity_ok={parity_ok}"),
    }
}

// ── Check 2: Obs scaling sanity ─────────────────────────────────────────

fn check_obs_scaling() -> Check {
    let task = reaching_6dof();
    let mut env = task.build_vec_env(4, 0).expect("build");
    env.reset_all().expect("reset");

    // Step 100 times with random actions
    let mut rng = StdRng::seed_from_u64(42);
    let mut max_obs = [f32::NEG_INFINITY; 12];

    for _ in 0..100 {
        let action_data: Vec<f32> = (0..24)
            .map(|_| {
                use rand::Rng;
                rng.random_range(-1.0_f32..1.0_f32)
            })
            .collect();
        let actions = Tensor::from_slice(&action_data, &[4, 6]);
        let result = env.step(&actions).expect("step");

        for i in 0..4 {
            let obs = result.observations.row(i);
            for (j, &v) in obs.iter().enumerate() {
                if v.abs() > max_obs[j] {
                    max_obs[j] = v.abs();
                }
            }
        }
    }

    // obs_scale is [1/pi x6, 0.1 x6]. Under random torques, joints can
    // exceed soft limits and velocities spike. The scaling keeps values
    // from being astronomical. Check: all finite and qpos scaled < 10.
    let all_reasonable =
        max_obs.iter().all(|v| v.is_finite()) && max_obs[..6].iter().all(|&v| v < 10.0);

    Check {
        name: "Obs scaling sanity (100 steps, random actions)",
        pass: all_reasonable,
        detail: format!(
            "max_obs=[{:.2}, {:.2}, ..., {:.2}, {:.2}]",
            max_obs[0], max_obs[1], max_obs[10], max_obs[11]
        ),
    }
}

// ── Check 3: Done condition reachable ───────────────────────────────────

fn check_done_reachable() -> Check {
    let task = reaching_6dof();
    let mut env = task.build_vec_env(1, 0).expect("build");

    // Set qpos to target_joints via data_mut on the batch
    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];
    {
        let model = env.model().clone();
        let data = env.batch_mut().env_mut(0).expect("env 0");
        for (j, &q) in target_joints.iter().enumerate() {
            data.qpos[j] = q;
        }
        for j in 0..6 {
            data.qvel[j] = 0.0;
        }
        data.forward(&model).expect("forward");
    }

    let actions = Tensor::zeros(&[1, 6]);
    let result = env.step(&actions).expect("step");
    let done = result.dones[0];

    Check {
        name: "Done condition reachable (from target_joints)",
        pass: done,
        detail: format!("done={done} after setting qpos=target_joints, qvel=0"),
    }
}

// ── Check 4: Truncated timing ───────────────────────────────────────────

fn check_truncated_timing() -> Check {
    let task = reaching_6dof();
    let mut env = task.build_vec_env(1, 0).expect("build");
    env.reset_all().expect("reset");

    // Step until truncated fires. dt=0.002, sub_steps=5, so 0.01s/step.
    // Truncation at 5.0s = 500 steps.
    let actions = Tensor::zeros(&[1, 6]);
    let mut truncated_at = None;

    for step in 0..600 {
        let result = env.step(&actions).expect("step");
        if result.truncateds[0] {
            truncated_at = Some(step);
            break;
        }
    }

    let pass = truncated_at == Some(500);

    Check {
        name: "Truncated fires at t=5.0s (step 500)",
        pass,
        detail: format!("truncated_at={truncated_at:?}"),
    }
}

// ── Check 5: Reward gradient ────────────────────────────────────────────

fn check_reward_gradient() -> Check {
    let task = reaching_6dof();
    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];

    // Reward at qpos=0 (far from target)
    let mut env_far = task.build_vec_env(1, 0).expect("build");
    env_reset_with_qpos(&mut env_far, &[0.0; 6]);
    let actions = Tensor::zeros(&[1, 6]);
    let r_far = env_far.step(&actions).expect("step").rewards[0];

    // Reward at qpos=target (at target)
    let mut env_near = task.build_vec_env(1, 0).expect("build");
    env_reset_with_qpos(&mut env_near, &target_joints);
    let r_near = env_near.step(&actions).expect("step").rewards[0];

    let pass = r_near > r_far;

    Check {
        name: "Reward improves as qpos → target",
        pass,
        detail: format!("r_far={r_far:.4}, r_near={r_near:.4}"),
    }
}

fn env_reset_with_qpos(env: &mut VecEnv, qpos: &[f64; 6]) {
    env.reset_all().expect("reset");
    let model = env.model().clone();
    let data = env.batch_mut().env_mut(0).expect("env 0");
    for (j, &q) in qpos.iter().enumerate() {
        data.qpos[j] = q;
    }
    for j in 0..6 {
        data.qvel[j] = 0.0;
    }
    data.forward(&model).expect("forward");
}

// ── Check 6: Auto-reset terminal_obs ────────────────────────────────────

fn check_terminal_obs() -> Check {
    let task = reaching_6dof();
    let mut env = task.build_vec_env(1, 0).expect("build");
    env.reset_all().expect("reset");

    // Step until truncated at step 500
    let actions = Tensor::zeros(&[1, 6]);
    let mut terminal_populated = false;

    for _ in 0..510 {
        let result = env.step(&actions).expect("step");
        if result.truncateds[0] || result.dones[0] {
            terminal_populated = result.terminal_observations[0].is_some();
            break;
        }
    }

    Check {
        name: "Auto-reset terminal_obs populated",
        pass: terminal_populated,
        detail: format!("terminal_populated={terminal_populated}"),
    }
}

// ── Check 7: Linear policy construction ─────────────────────────────────

fn check_linear_policy() -> Check {
    let task = reaching_6dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());

    let expected_params = 6 * 12 + 6; // W[6x12] + b[6] = 78
    let n = policy.n_params();

    // Forward pass with dummy obs
    let obs = vec![0.1_f32; 12];
    let actions = policy.forward(&obs);
    let all_valid = actions.iter().all(|&a| a.abs() <= 1.0 && !a.is_nan());

    Check {
        name: "Linear policy: 78 params, valid actions",
        pass: n == expected_params && all_valid,
        detail: format!("n_params={n}, expected={expected_params}, actions_valid={all_valid}"),
    }
}

// ── Check 8: MLP policy construction ────────────────────────────────────

fn check_mlp_policy() -> Check {
    let task = reaching_6dof();
    let policy = MlpPolicy::new(task.obs_dim(), 32, task.act_dim(), task.obs_scale());

    let n = policy.n_params();
    // W1[32x12] + b1[32] + W2[6x32] + b2[6] = 384 + 32 + 192 + 6 = 614
    let expected = 614;

    let obs = vec![0.1_f32; 12];
    let actions = policy.forward(&obs);
    let all_valid = actions.iter().all(|&a| a.abs() <= 1.0 && !a.is_nan());

    Check {
        name: "MLP policy: 614 params, valid actions",
        pass: n == expected && all_valid,
        detail: format!("n_params={n}, expected={expected}, actions_valid={all_valid}"),
    }
}

// ── Check 9: Autograd policy + gradient finite ──────────────────────────

fn check_autograd_policy() -> Check {
    let task = reaching_6dof();
    let mut rng = StdRng::seed_from_u64(42);

    let policy = AutogradStochasticPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        -0.5,
        Activation::Relu,
        &mut rng,
    );

    let n = policy.n_params();

    let obs = vec![0.1_f32; 12];
    let (mu, log_std) = policy.forward_stochastic(&obs);
    let actions_valid = mu.iter().all(|&a| a.abs() <= 1.0 && !a.is_nan());
    let log_std_valid = log_std.iter().all(|v| v.is_finite());

    // Test gradient: use reparameterized VJP
    let action: Vec<f64> = mu
        .iter()
        .zip(&log_std)
        .map(|(&m, &ls)| ls.exp().mul_add(0.1, m))
        .collect();
    let upstream = vec![1.0; task.act_dim()];
    let grad = policy.reparameterized_vjp(&obs, &action, &upstream);
    let grad_finite = grad.iter().all(|g| g.is_finite());

    Check {
        name: "Autograd policy: gradient finite after backward",
        pass: actions_valid && log_std_valid && grad_finite,
        detail: format!(
            "n_params={n}, actions_valid={actions_valid}, log_std_valid={log_std_valid}, grad_finite={grad_finite}"
        ),
    }
}

// ── Check 10: No NaN after 500 steps ────────────────────────────────────

fn check_endurance() -> Check {
    let task = reaching_6dof();
    let mut env = task.build_vec_env(4, 0).expect("build");
    env.reset_all().expect("reset");

    let mut rng = StdRng::seed_from_u64(42);
    let mut any_nan = false;

    for _ in 0..500 {
        let action_data: Vec<f32> = (0..24)
            .map(|_| {
                use rand::Rng;
                rng.random_range(-1.0_f32..1.0_f32)
            })
            .collect();
        let actions = Tensor::from_slice(&action_data, &[4, 6]);
        let result = env.step(&actions).expect("step");

        for i in 0..4 {
            for &v in result.observations.row(i) {
                if v.is_nan() {
                    any_nan = true;
                }
            }
            if result.rewards[i].is_nan() {
                any_nan = true;
            }
        }
    }

    Check {
        name: "No NaN after 500 steps (4 envs, random actions)",
        pass: !any_nan,
        detail: format!("any_nan={any_nan}"),
    }
}

// ── Main ─────────────────────────────────────────────────────────────────

fn main() {
    println!("=== 6-DOF Stress Test ===\n");

    let checks = vec![
        check_vec_env_parity(),
        check_obs_scaling(),
        check_done_reachable(),
        check_truncated_timing(),
        check_reward_gradient(),
        check_terminal_obs(),
        check_linear_policy(),
        check_mlp_policy(),
        check_autograd_policy(),
        check_endurance(),
    ];

    let all_pass = print_report("6-DOF Integration", &checks);

    if !all_pass {
        std::process::exit(1);
    }
}
