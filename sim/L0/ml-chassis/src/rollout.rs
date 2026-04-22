//! Episodic rollout collector for on-policy algorithms.
//!
//! Steps a [`VecEnv`] until all environments complete an
//! episode (done or truncated), recording observations, actions, rewards,
//! and terminal state per environment.
//!
//! Used by CEM, REINFORCE, PPO — any algorithm that processes complete
//! episodes rather than individual transitions.

use crate::tensor::Tensor;
use crate::vec_env::VecEnv;

// ── Trajectory ────────────────────────────────────────────────────────────

/// One environment's episode data.
///
/// Records the sequence of observations, actions, and rewards from a single
/// episode. The `done` flag distinguishes true terminal states from timeouts.
#[derive(Debug, Clone)]
pub struct Trajectory {
    /// Observations at each step, `[steps][obs_dim]`.
    pub obs: Vec<Vec<f32>>,
    /// Actions taken at each step, `[steps][act_dim]`.
    pub actions: Vec<Vec<f64>>,
    /// Rewards received at each step.
    pub rewards: Vec<f64>,
    /// `true` if the episode ended at a true terminal state,
    /// `false` if truncated (timeout or `max_steps`).
    pub done: bool,
    /// Observation at the end of the episode. Used for GAE bootstrapping:
    /// if truncated, `V(terminal_obs)` is the bootstrap value.
    pub terminal_obs: Option<Vec<f32>>,
}

impl Trajectory {
    /// Number of recorded steps.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.rewards.len()
    }

    /// Whether the trajectory has zero steps.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.rewards.is_empty()
    }
}

// ── EpisodicRollout ───────────────────────────────────────────────────────

/// One epoch's worth of per-environment trajectories.
///
/// Returned by [`collect_episodic_rollout`]. Contains one [`Trajectory`]
/// per environment.
#[derive(Debug, Clone)]
pub struct EpisodicRollout {
    /// Per-environment trajectories, length `n_envs`.
    pub trajectories: Vec<Trajectory>,
}

// ── collect_episodic_rollout ──────────────────────────────────────────────

/// Collect one complete episode from every environment.
///
/// Resets the environment, then steps until all envs have either reached
/// a true terminal state (`done`) or been truncated (timeout / `max_steps`).
///
/// # Arguments
///
/// - `env` — vectorized environment.
/// - `act_fn(env_index, obs) -> action` — caller-provided action selection.
///   This is where the policy forward pass + exploration noise happens.
///   The action vector length must match the environment's action dimension.
/// - `max_steps` — safety limit. Any env still running after this many
///   steps is force-truncated.
///
/// # Panics
///
/// Panics if `env.reset_all()` or `env.step()` fails (physics errors in
/// the environment setup are programming errors, not runtime conditions).
// env errors surface construction bugs (bad MJCF, malformed model); the
// rollout loop has no sensible recovery path mid-episode.
#[allow(clippy::panic)]
pub fn collect_episodic_rollout(
    env: &mut VecEnv,
    act_fn: &mut dyn FnMut(usize, &[f32]) -> Vec<f64>,
    max_steps: usize,
) -> EpisodicRollout {
    let n_envs = env.n_envs();

    // Reset and get initial observations.
    let mut current_obs = env
        .reset_all()
        .unwrap_or_else(|e| panic!("collect_episodic_rollout: reset_all failed: {e}"));
    // Initialize per-env trajectories and completion flags.
    let mut trajectories: Vec<Trajectory> = (0..n_envs)
        .map(|_| Trajectory {
            obs: Vec::new(),
            actions: Vec::new(),
            rewards: Vec::new(),
            done: false,
            terminal_obs: None,
        })
        .collect();
    let mut complete = vec![false; n_envs];

    // Get act_dim from the first action.
    let first_action = act_fn(0, current_obs.row(0));
    let act_dim = first_action.len();

    // Record first env's step and prepare action tensor.
    trajectories[0].obs.push(current_obs.row(0).to_vec());
    trajectories[0].actions.push(first_action.clone());

    let mut action_data = vec![0.0_f32; n_envs * act_dim];
    for (j, &a) in first_action.iter().enumerate() {
        // f64 → f32 at the sim/ML boundary (see module doc).
        #[allow(clippy::cast_possible_truncation)]
        {
            action_data[j] = a as f32;
        }
    }

    // Get actions for remaining envs.
    for i in 1..n_envs {
        let obs = current_obs.row(i);
        trajectories[i].obs.push(obs.to_vec());
        let action = act_fn(i, obs);
        for (j, &a) in action.iter().enumerate() {
            // f64 → f32 at the sim/ML boundary (see module doc).
            #[allow(clippy::cast_possible_truncation)]
            {
                action_data[i * act_dim + j] = a as f32;
            }
        }
        trajectories[i].actions.push(action);
    }

    // First step.
    let actions = Tensor::from_slice(&action_data, &[n_envs, act_dim]);
    let result = env
        .step(&actions)
        .unwrap_or_else(|e| panic!("collect_episodic_rollout: step failed: {e}"));

    for i in 0..n_envs {
        trajectories[i].rewards.push(result.rewards[i]);
        if result.dones[i] || result.truncateds[i] {
            complete[i] = true;
            trajectories[i].done = result.dones[i];
            trajectories[i].terminal_obs = result.terminal_observations[i]
                .as_ref()
                .map(|t| t.as_slice().to_vec());
        }
    }
    current_obs = result.observations;

    // Remaining steps.
    for _step in 1..max_steps {
        if complete.iter().all(|&c| c) {
            break;
        }

        // Collect actions for non-complete envs.
        action_data.fill(0.0);
        for i in 0..n_envs {
            if !complete[i] {
                let obs = current_obs.row(i);
                trajectories[i].obs.push(obs.to_vec());
                let action = act_fn(i, obs);
                for (j, &a) in action.iter().enumerate() {
                    // f64 → f32 at the sim/ML boundary (see module doc).
                    #[allow(clippy::cast_possible_truncation)]
                    {
                        action_data[i * act_dim + j] = a as f32;
                    }
                }
                trajectories[i].actions.push(action);
            }
        }

        let actions = Tensor::from_slice(&action_data, &[n_envs, act_dim]);
        let result = env
            .step(&actions)
            .unwrap_or_else(|e| panic!("collect_episodic_rollout: step failed: {e}"));

        for i in 0..n_envs {
            if !complete[i] {
                trajectories[i].rewards.push(result.rewards[i]);
                if result.dones[i] || result.truncateds[i] {
                    complete[i] = true;
                    trajectories[i].done = result.dones[i];
                    trajectories[i].terminal_obs = result.terminal_observations[i]
                        .as_ref()
                        .map(|t| t.as_slice().to_vec());
                }
            }
        }
        current_obs = result.observations;
    }

    // Force-truncate any env that didn't finish within max_steps.
    for i in 0..n_envs {
        if !complete[i] {
            complete[i] = true;
            // Not done — truncated by max_steps.
            trajectories[i].terminal_obs = Some(current_obs.row(i).to_vec());
        }
    }

    EpisodicRollout { trajectories }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::space::{ActionSpace, ObservationSpace};
    use crate::vec_env::VecEnv;

    const N_ENVS: usize = 3;

    fn pendulum_xml() -> &'static str {
        r#"
        <mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
        </mujoco>
        "#
    }

    fn make_model() -> Arc<sim_core::Model> {
        Arc::new(sim_mjcf::load_model(pendulum_xml()).unwrap())
    }

    fn make_env(n_envs: usize, timeout: f64) -> VecEnv {
        let model = make_model();
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        VecEnv::builder(model, n_envs)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| -1.0)
            .done(|_m, _d| false) // never done — only truncated
            .truncated(move |_m, d| d.time > timeout)
            .sub_steps(1)
            .build()
            .unwrap()
    }

    fn make_env_with_done(n_envs: usize) -> VecEnv {
        let model = make_model();
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        VecEnv::builder(model, n_envs)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| -1.0)
            .done(|_m, d| d.time > 0.05) // done after ~5 steps at dt=0.01
            .truncated(|_m, d| d.time > 1.0)
            .sub_steps(1)
            .build()
            .unwrap()
    }

    #[test]
    fn rollout_collects_all_envs() {
        let mut env = make_env(N_ENVS, 0.05);
        let rollout = collect_episodic_rollout(&mut env, &mut |_i, _obs| vec![0.0], 1000);

        assert_eq!(rollout.trajectories.len(), N_ENVS);
        for traj in &rollout.trajectories {
            assert!(!traj.is_empty());
        }
    }

    #[test]
    fn rollout_records_observations_and_actions() {
        let mut env = make_env(N_ENVS, 0.03);
        let rollout = collect_episodic_rollout(&mut env, &mut |_i, _obs| vec![0.5], 1000);

        for traj in &rollout.trajectories {
            // obs_dim = 2 (qpos=1 + qvel=1), act_dim = 1
            assert_eq!(traj.obs.len(), traj.len());
            assert_eq!(traj.actions.len(), traj.len());
            for obs in &traj.obs {
                assert_eq!(obs.len(), 2, "obs_dim should be 2");
            }
            for action in &traj.actions {
                assert_eq!(action.len(), 1, "act_dim should be 1");
            }
        }
    }

    #[test]
    fn rollout_stops_on_done() {
        let mut env = make_env_with_done(N_ENVS);
        let rollout = collect_episodic_rollout(&mut env, &mut |_i, _obs| vec![0.0], 1000);

        for traj in &rollout.trajectories {
            assert!(traj.done, "env should reach done state");
            // At dt=0.01 with done at t>0.05, should be ~5-6 steps.
            assert!(traj.len() <= 10, "trajectory too long: {}", traj.len());
        }
    }

    #[test]
    fn rollout_truncates_at_max_steps() {
        // Very high timeout (won't trigger), but low max_steps.
        let mut env = make_env(N_ENVS, 100.0);
        let max_steps = 5;
        let rollout = collect_episodic_rollout(&mut env, &mut |_i, _obs| vec![0.0], max_steps);

        for traj in &rollout.trajectories {
            assert!(!traj.done, "should be truncated, not done");
            assert_eq!(traj.len(), max_steps);
        }
    }

    #[test]
    fn rollout_stores_terminal_obs() {
        let mut env = make_env(N_ENVS, 0.03);
        let rollout = collect_episodic_rollout(&mut env, &mut |_i, _obs| vec![0.0], 1000);

        for traj in &rollout.trajectories {
            assert!(
                traj.terminal_obs.is_some(),
                "terminal_obs should be set for completed env"
            );
            let t_obs = traj.terminal_obs.as_ref().unwrap();
            assert_eq!(t_obs.len(), 2, "terminal obs should have obs_dim=2");
        }
    }

    #[test]
    fn act_fn_receives_correct_env_index() {
        use std::sync::Mutex;

        let seen_indices = Arc::new(Mutex::new(Vec::new()));
        let seen = Arc::clone(&seen_indices);

        let mut env = make_env(N_ENVS, 0.02);
        let _rollout = collect_episodic_rollout(
            &mut env,
            &mut |i, _obs| {
                seen.lock().unwrap().push(i);
                vec![0.0]
            },
            1000,
        );

        // Every env index 0..N_ENVS should appear at least once.
        let indices = seen_indices.lock().unwrap().clone();
        for env_idx in 0..N_ENVS {
            assert!(
                indices.contains(&env_idx),
                "env index {env_idx} was never passed to act_fn"
            );
        }
    }
}
