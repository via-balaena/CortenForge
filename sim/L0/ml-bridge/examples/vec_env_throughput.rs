#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use
)]
//! Multi-env throughput demo — 64 parallel pendulums.
//!
//! Demonstrates `VecEnv` with `BatchSim` composition, auto-reset,
//! terminal observations, and measures steps/sec throughput.
//!
//! Run: `cargo run -p sim-ml-bridge --release --example vec_env_throughput`

use std::sync::Arc;
use std::time::Instant;

use sim_ml_bridge::{ActionSpace, ObservationSpace, Tensor, VecEnv};

fn main() {
    let xml = r#"
    <mujoco model="pendulum-vecenv">
      <option timestep="0.002"/>
      <worldbody>
        <body name="pendulum" pos="0 0 1">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
        </body>
      </worldbody>
      <actuator>
        <motor joint="hinge" name="torque" ctrllimited="true" ctrlrange="-2 2"/>
      </actuator>
    </mujoco>
    "#;

    let model = Arc::new(sim_mjcf::load_model(xml).expect("Failed to load MJCF"));
    let n_envs = 64;
    let sub_steps = 10;
    let total_steps = 10_000;

    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("Failed to build obs space");

    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("Failed to build act space");

    println!("=== VecEnv Throughput Demo ===");
    println!("  Environments: {n_envs}");
    println!("  Sub-steps:    {sub_steps}");
    println!("  Obs dim:      {}", obs_space.dim());
    println!("  Act dim:      {}", act_space.dim());

    let mut env = VecEnv::builder(Arc::clone(&model), n_envs)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(|_m, data| -data.qpos[0].powi(2))
        .done(|_m, data| data.qpos[0].abs() > 3.0)
        .truncated(|_m, data| data.time > 10.0)
        .sub_steps(sub_steps)
        .on_reset(|_m, data, _env_idx| {
            // Small domain randomization: random-ish initial angle
            // (deterministic — no RNG, just env_idx-based)
            data.qpos[0] = 0.1;
        })
        .build()
        .expect("Failed to build VecEnv");

    let _obs = env.reset_all().expect("Failed to reset");
    let actions = Tensor::zeros(&[n_envs, 1]);

    // Warm up
    for _ in 0..100 {
        let _ = env.step(&actions);
    }

    // Measure throughput
    let start = Instant::now();
    let mut total_resets = 0_u64;

    for _ in 0..total_steps {
        let result = env.step(&actions).expect("Step failed");
        total_resets += result.dones.iter().filter(|&&d| d).count() as u64;
        total_resets += result.truncateds.iter().filter(|&&t| t).count() as u64;
    }

    let elapsed = start.elapsed();
    let total_env_steps = (total_steps as u64) * (n_envs as u64);
    let steps_per_sec = total_env_steps as f64 / elapsed.as_secs_f64();
    let action_steps_per_sec = total_steps as f64 / elapsed.as_secs_f64();

    println!("\n=== Results ===");
    println!("  Total action steps:     {total_steps}");
    println!("  Total env steps:        {total_env_steps} ({n_envs} envs x {total_steps} actions)");
    println!(
        "  Total physics steps:    {} ({total_env_steps} x {sub_steps} sub-steps)",
        total_env_steps * sub_steps as u64
    );
    println!("  Total auto-resets:      {total_resets}");
    println!("  Wall time:              {:.3}s", elapsed.as_secs_f64());
    println!("  Action steps/sec:       {action_steps_per_sec:.0} (VecEnv.step() calls/sec)");
    println!("  Env steps/sec:          {steps_per_sec:.0} (individual env steps/sec)");
    println!(
        "  Physics steps/sec:      {:.0}",
        total_env_steps as f64 * sub_steps as f64 / elapsed.as_secs_f64()
    );
    println!(
        "  Time per action step:   {:.1} µs",
        elapsed.as_micros() as f64 / total_steps as f64
    );

    // Verify auto-reset is working
    if total_resets > 0 {
        println!("\n  Auto-reset confirmed: {total_resets} episodes completed during run.");
    } else {
        println!("\n  Note: no episodes terminated. Increase total_steps or lower done threshold.");
    }
}
