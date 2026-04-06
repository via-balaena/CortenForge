//! Replay buffer for off-policy algorithms (SAC, TD3).
//!
//! Ring buffer storing `(obs, action, reward, next_obs, done)` transitions.
//! Pre-allocates all storage at construction — no per-push allocations.
//! Uniform random sampling with replacement.

use rand::Rng;

// ── TransitionBatch ───────────────────────────────────────────────────────

/// A batch of transitions sampled from a [`ReplayBuffer`].
///
/// All fields are flat vectors. To access transition `i`:
/// - obs: `batch.obs[i * batch.obs_dim..(i + 1) * batch.obs_dim]`
/// - actions: `batch.actions[i * batch.act_dim..(i + 1) * batch.act_dim]`
/// - rewards/dones: `batch.rewards[i]`, `batch.dones[i]`
#[derive(Debug, Clone)]
pub struct TransitionBatch {
    /// Observations, flat `[batch_size × obs_dim]`.
    pub obs: Vec<f32>,
    /// Actions, flat `[batch_size × act_dim]`.
    pub actions: Vec<f64>,
    /// Rewards, `[batch_size]`.
    pub rewards: Vec<f64>,
    /// Next observations, flat `[batch_size × obs_dim]`.
    pub next_obs: Vec<f32>,
    /// Terminal flags, `[batch_size]`.
    pub dones: Vec<bool>,
    /// Number of transitions in this batch.
    pub batch_size: usize,
    /// Observation dimensionality.
    pub obs_dim: usize,
    /// Action dimensionality.
    pub act_dim: usize,
}

// ── ReplayBuffer ──────────────────────────────────────────────────────────

/// Ring buffer for off-policy experience replay.
///
/// Used by SAC, TD3, and all off-policy methods. Stores transitions in
/// pre-allocated flat arrays. When full, new transitions overwrite the
/// oldest.
pub struct ReplayBuffer {
    capacity: usize,
    obs_dim: usize,
    act_dim: usize,
    obs: Vec<f32>,
    actions: Vec<f64>,
    rewards: Vec<f64>,
    next_obs: Vec<f32>,
    dones: Vec<bool>,
    len: usize,
    pos: usize,
}

impl ReplayBuffer {
    /// Create a new replay buffer with pre-allocated storage.
    #[must_use]
    pub fn new(capacity: usize, obs_dim: usize, act_dim: usize) -> Self {
        Self {
            capacity,
            obs_dim,
            act_dim,
            obs: vec![0.0; capacity * obs_dim],
            actions: vec![0.0; capacity * act_dim],
            rewards: vec![0.0; capacity],
            next_obs: vec![0.0; capacity * obs_dim],
            dones: vec![false; capacity],
            len: 0,
            pos: 0,
        }
    }

    /// Store a transition.
    ///
    /// When the buffer is full, overwrites the oldest transition.
    ///
    /// # Panics
    ///
    /// Panics if `obs.len() != obs_dim`, `next_obs.len() != obs_dim`,
    /// or `action.len() != act_dim`.
    pub fn push(&mut self, obs: &[f32], action: &[f64], reward: f64, next_obs: &[f32], done: bool) {
        assert!(
            obs.len() == self.obs_dim,
            "ReplayBuffer::push: obs.len() ({}) != obs_dim ({})",
            obs.len(),
            self.obs_dim,
        );
        assert!(
            action.len() == self.act_dim,
            "ReplayBuffer::push: action.len() ({}) != act_dim ({})",
            action.len(),
            self.act_dim,
        );
        assert!(
            next_obs.len() == self.obs_dim,
            "ReplayBuffer::push: next_obs.len() ({}) != obs_dim ({})",
            next_obs.len(),
            self.obs_dim,
        );

        let i = self.pos;
        self.obs[i * self.obs_dim..(i + 1) * self.obs_dim].copy_from_slice(obs);
        self.actions[i * self.act_dim..(i + 1) * self.act_dim].copy_from_slice(action);
        self.rewards[i] = reward;
        self.next_obs[i * self.obs_dim..(i + 1) * self.obs_dim].copy_from_slice(next_obs);
        self.dones[i] = done;

        self.pos = (self.pos + 1) % self.capacity;
        if self.len < self.capacity {
            self.len += 1;
        }
    }

    /// Sample a batch of transitions uniformly at random (with replacement).
    ///
    /// # Panics
    ///
    /// Panics if `self.len() < batch_size`.
    pub fn sample(&self, batch_size: usize, rng: &mut impl Rng) -> TransitionBatch {
        assert!(
            self.len >= batch_size,
            "ReplayBuffer::sample: buffer has {} transitions, need at least {batch_size}",
            self.len,
        );

        let mut obs = vec![0.0_f32; batch_size * self.obs_dim];
        let mut actions = vec![0.0_f64; batch_size * self.act_dim];
        let mut rewards = vec![0.0_f64; batch_size];
        let mut next_obs = vec![0.0_f32; batch_size * self.obs_dim];
        let mut dones = vec![false; batch_size];

        for b in 0..batch_size {
            let i = rng.random_range(0..self.len);
            obs[b * self.obs_dim..(b + 1) * self.obs_dim]
                .copy_from_slice(&self.obs[i * self.obs_dim..(i + 1) * self.obs_dim]);
            actions[b * self.act_dim..(b + 1) * self.act_dim]
                .copy_from_slice(&self.actions[i * self.act_dim..(i + 1) * self.act_dim]);
            rewards[b] = self.rewards[i];
            next_obs[b * self.obs_dim..(b + 1) * self.obs_dim]
                .copy_from_slice(&self.next_obs[i * self.obs_dim..(i + 1) * self.obs_dim]);
            dones[b] = self.dones[i];
        }

        TransitionBatch {
            obs,
            actions,
            rewards,
            next_obs,
            dones,
            batch_size,
            obs_dim: self.obs_dim,
            act_dim: self.act_dim,
        }
    }

    /// Number of transitions currently stored (capped at capacity).
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the buffer is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_lossless
)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    const OBS_DIM: usize = 3;
    const ACT_DIM: usize = 2;

    fn make_obs(seed: f32) -> Vec<f32> {
        vec![seed, seed + 0.1, seed + 0.2]
    }

    fn make_action(seed: f64) -> Vec<f64> {
        vec![seed, seed + 0.01]
    }

    #[test]
    fn push_and_len() {
        let mut buf = ReplayBuffer::new(100, OBS_DIM, ACT_DIM);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);

        buf.push(
            &make_obs(1.0),
            &make_action(0.5),
            1.0,
            &make_obs(2.0),
            false,
        );
        assert_eq!(buf.len(), 1);

        buf.push(&make_obs(3.0), &make_action(0.7), 2.0, &make_obs(4.0), true);
        assert_eq!(buf.len(), 2);
        assert!(!buf.is_empty());
    }

    #[test]
    fn ring_buffer_wraps() {
        let mut buf = ReplayBuffer::new(3, OBS_DIM, ACT_DIM);

        for i in 0..5 {
            buf.push(
                &make_obs(i as f32),
                &make_action(i as f64),
                i as f64,
                &make_obs(i as f32 + 10.0),
                false,
            );
        }

        // Capacity is 3 — len should cap at 3.
        assert_eq!(buf.len(), 3);

        // The oldest transitions (0, 1) should be overwritten.
        // Remaining: transitions 2, 3, 4 at positions 2, 0, 1.
        // Verify by sampling all 3 with a fixed seed.
        let mut rng = StdRng::seed_from_u64(42);
        let batch = buf.sample(3, &mut rng);
        assert_eq!(batch.batch_size, 3);

        // All sampled rewards should be from {2.0, 3.0, 4.0}.
        for &r in &batch.rewards {
            assert!(
                (2.0..=4.0).contains(&r),
                "reward {r} not in expected range [2.0, 4.0]"
            );
        }
    }

    #[test]
    fn sample_returns_correct_shape() {
        let mut buf = ReplayBuffer::new(100, OBS_DIM, ACT_DIM);
        for i in 0..10 {
            buf.push(
                &make_obs(i as f32),
                &make_action(i as f64),
                i as f64,
                &make_obs(i as f32 + 1.0),
                i % 3 == 0,
            );
        }

        let mut rng = StdRng::seed_from_u64(0);
        let batch = buf.sample(5, &mut rng);

        assert_eq!(batch.batch_size, 5);
        assert_eq!(batch.obs_dim, OBS_DIM);
        assert_eq!(batch.act_dim, ACT_DIM);
        assert_eq!(batch.obs.len(), 5 * OBS_DIM);
        assert_eq!(batch.actions.len(), 5 * ACT_DIM);
        assert_eq!(batch.rewards.len(), 5);
        assert_eq!(batch.next_obs.len(), 5 * OBS_DIM);
        assert_eq!(batch.dones.len(), 5);
    }

    #[test]
    fn sample_reproduces_pushed_data() {
        let mut buf = ReplayBuffer::new(100, OBS_DIM, ACT_DIM);
        let obs = [1.0_f32, 2.0, 3.0];
        let action = [0.5_f64, -0.3];
        let reward = 7.5;
        let next_obs = [4.0_f32, 5.0, 6.0];

        buf.push(&obs, &action, reward, &next_obs, true);

        // With only 1 transition, every sample must return it.
        let mut rng = StdRng::seed_from_u64(0);
        let batch = buf.sample(1, &mut rng);

        assert_eq!(&batch.obs[..], &obs[..]);
        assert_eq!(&batch.actions[..], &action[..]);
        assert_eq!(batch.rewards[0], reward);
        assert_eq!(&batch.next_obs[..], &next_obs[..]);
        assert!(batch.dones[0]);
    }

    #[test]
    #[should_panic(expected = "obs.len() (2) != obs_dim (3)")]
    fn push_wrong_obs_length_panics() {
        let mut buf = ReplayBuffer::new(10, OBS_DIM, ACT_DIM);
        buf.push(&[1.0, 2.0], &make_action(0.0), 0.0, &make_obs(0.0), false);
    }

    #[test]
    #[should_panic(expected = "action.len() (1) != act_dim (2)")]
    fn push_wrong_action_length_panics() {
        let mut buf = ReplayBuffer::new(10, OBS_DIM, ACT_DIM);
        buf.push(&make_obs(0.0), &[1.0], 0.0, &make_obs(0.0), false);
    }

    #[test]
    #[should_panic(expected = "buffer has 2 transitions, need at least 5")]
    fn sample_from_underfilled_panics() {
        let mut buf = ReplayBuffer::new(100, OBS_DIM, ACT_DIM);
        buf.push(
            &make_obs(0.0),
            &make_action(0.0),
            0.0,
            &make_obs(1.0),
            false,
        );
        buf.push(
            &make_obs(1.0),
            &make_action(1.0),
            1.0,
            &make_obs(2.0),
            false,
        );

        let mut rng = StdRng::seed_from_u64(0);
        buf.sample(5, &mut rng);
    }
}
