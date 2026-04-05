#![allow(clippy::expect_used, clippy::unwrap_used, clippy::suboptimal_flops)]
//! Cart-pole balance — classic RL benchmark.
//!
//! 4D observation (cart position, cart velocity, pole angle, pole angular velocity),
//! 1D action (horizontal force on cart).  Demonstrates `SimEnv` with a more
//! complex observation space using named sensors.
//!
//! Run: `cargo run -p sim-ml-bridge --example cartpole_balance`

use std::sync::Arc;

use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor};

fn main() {
    // Cart-pole: slide joint for cart, hinge joint for pole
    let xml = r#"
    <mujoco model="cartpole">
      <option timestep="0.002"/>
      <worldbody>
        <body name="cart" pos="0 0 0.5">
          <joint name="cart_slide" type="slide" axis="1 0 0" damping="0.1"/>
          <geom type="box" size="0.2 0.1 0.05" mass="1.0"/>
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

    let model = Arc::new(sim_mjcf::load_model(xml).expect("Failed to load MJCF"));
    println!(
        "Cart-pole model: nq={}, nv={}, nu={}, nsensordata={}",
        model.nq, model.nv, model.nu, model.nsensordata
    );

    // Observation: all 4 sensor readings (cart_pos, cart_vel, pole_angle, pole_angvel)
    let obs_space = ObservationSpace::builder()
        .sensor("cart_pos")
        .sensor("cart_vel")
        .sensor("pole_angle")
        .sensor("pole_angvel")
        .build(&model)
        .expect("Failed to build obs space");

    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("Failed to build act space");

    println!(
        "Obs dim: {} (cart_pos, cart_vel, pole_angle, pole_angvel)",
        obs_space.dim()
    );
    println!("Act dim: {} (horizontal force)", act_space.dim());

    // Build environment
    // Reward: +1 for each step the pole stays upright
    // Done: pole angle > 0.2 rad or cart off track (|x| > 2.4)
    // Truncated: after 10 seconds (500 steps at 50 Hz)
    let mut env = SimEnv::builder(model)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(|_m, _d| 1.0) // survival reward
        .done(|_m, data| {
            let cart_x = data.qpos[0];
            let pole_angle = data.qpos[1];
            cart_x.abs() > 2.4 || pole_angle.abs() > 0.2
        })
        .truncated(|_m, data| data.time > 10.0)
        .sub_steps(10) // 500 Hz physics, 50 Hz actions
        .build()
        .expect("Failed to build SimEnv");

    // Run 5 episodes with a simple proportional controller
    println!("\n--- Running 5 episodes with proportional controller ---\n");

    for ep in 0..5 {
        let obs = env.reset().expect("Failed to reset");
        let mut total_reward = 0.0;
        let mut steps = 0;

        loop {
            // Simple proportional controller: push opposite to pole angle
            let pole_angle = obs.as_slice()[2]; // sensor index 2 = pole_angle
            let pole_angvel = obs.as_slice()[3]; // sensor index 3 = pole_angvel
            #[allow(clippy::cast_possible_truncation)]
            let action_val =
                ((-20.0 * f64::from(pole_angle)) - (5.0 * f64::from(pole_angvel))) as f32;
            let action = Tensor::from_slice(&[action_val], &[1]);

            let result = env.step(&action).expect("Step failed");
            total_reward += result.reward;
            steps += 1;

            if result.done || result.truncated {
                println!(
                    "Episode {ep}: {steps:>4} steps, reward={total_reward:>7.1}, \
                     done={}, truncated={}",
                    result.done, result.truncated
                );
                break;
            }
        }
    }
}
