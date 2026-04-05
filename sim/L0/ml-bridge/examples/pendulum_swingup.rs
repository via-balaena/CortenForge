#![allow(clippy::expect_used, clippy::unwrap_used)]
//! Pendulum swing-up — simplest possible RL environment.
//!
//! One hinge joint, one actuator, scalar observation (angle + angular velocity).
//! Demonstrates `SimEnv` with reward, done, truncated, sub-stepping, and reset.
//!
//! Run: `cargo run -p sim-ml-bridge --example pendulum_swingup`

use std::sync::Arc;

use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor};

fn main() {
    let xml = r#"
    <mujoco model="pendulum-swingup">
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
      <sensor>
        <jointpos joint="hinge" name="angle"/>
        <jointvel joint="hinge" name="angvel"/>
      </sensor>
    </mujoco>
    "#;

    let model = Arc::new(sim_mjcf::load_model(xml).expect("Failed to load MJCF"));
    println!("Model: nq={}, nv={}, nu={}", model.nq, model.nv, model.nu);

    // Observation: joint angle + angular velocity
    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("Failed to build obs space");

    // Action: single torque
    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("Failed to build act space");

    println!("Obs dim: {}", obs_space.dim());
    println!("Act dim: {}", act_space.dim());
    println!("Act bounds: {:?}", act_space.spec(&model));

    // Build environment
    // Reward: penalize angle deviation from upright (pi) and angular velocity
    // Done: never (swing-up is a continuous task)
    // Truncated: after 5 seconds
    let mut env = SimEnv::builder(model)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(|_m, data| {
            let angle = data.qpos[0];
            let angvel = data.qvel[0];
            // Reward being near upright (angle=pi) with low velocity
            let angle_cost = (angle - std::f64::consts::PI).powi(2);
            let vel_cost = 0.1 * angvel.powi(2);
            -(angle_cost + vel_cost)
        })
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > 5.0)
        .sub_steps(10) // 500 Hz physics, 50 Hz actions
        .build()
        .expect("Failed to build SimEnv");

    // Run one episode with a random-ish policy (sinusoidal torque)
    let obs = env.reset().expect("Failed to reset");
    println!("\nInitial obs: {:?}", obs.as_slice());

    let mut total_reward = 0.0;
    let mut steps = 0;

    loop {
        // Simple sinusoidal "policy" — not a real controller
        let t = env.data().time;
        #[allow(clippy::cast_possible_truncation)]
        let action_val = (t * 2.0 * std::f64::consts::PI).sin() as f32;
        let action = Tensor::from_slice(&[action_val], &[1]);

        let result = env.step(&action).expect("Step failed");
        total_reward += result.reward;
        steps += 1;

        if result.done || result.truncated {
            println!("\nEpisode finished after {steps} steps");
            println!("  Total reward: {total_reward:.2}");
            println!("  Final time:   {:.3}s", env.data().time);
            println!("  Done:         {}", result.done);
            println!("  Truncated:    {}", result.truncated);
            println!("  Final obs:    {:?}", result.observation.as_slice());
            break;
        }
    }
}
