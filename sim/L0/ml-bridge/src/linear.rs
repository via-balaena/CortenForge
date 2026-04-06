//! Linear policy, value, and Q-function implementations.
//!
//! The "stock engine" — simplest possible architecture with hand-coded
//! gradients. Every future implementation (MLP, burn, candle) must pass
//! the same gradient finite-difference tests.
//!
//! # Parameter layouts
//!
//! All parameters are stored as a contiguous `Vec<f64>`:
//!
//! - [`LinearPolicy`]: `W[act_dim × obs_dim]` row-major, then `b[act_dim]`.
//! - [`LinearValue`]: `w[obs_dim]`, then `b` (scalar).
//! - [`LinearQ`]: `w[obs_dim + act_dim]`, then `b` (scalar).
//!
//! # Observation scaling
//!
//! All three structs accept an `obs_scale` slice at construction. The
//! scaled observation `s_scaled[i] = f64::from(obs[i]) * obs_scale[i]`
//! keeps pre-activation values moderate, which matters for `tanh` (policy)
//! and for gradient magnitude (all three).

use crate::policy::{DifferentiablePolicy, Policy};
use crate::value::{QFunction, ValueFn};

// ── LinearPolicy ──────────────────────────────────────────────────────────

/// Linear policy: `μ(s) = tanh(W · s_scaled + b)`.
///
/// Implements [`Policy`] + [`DifferentiablePolicy`]. Enough for CEM,
/// REINFORCE, PPO, and TD3. Does NOT implement [`StochasticPolicy`]
/// (SAC needs learned `log_std` — that's a separate struct).
///
/// This is the exact architecture used in the reaching-arm examples,
/// extracted into a trait impl so algorithms can swap it for MLP or
/// autograd-backed policies without code changes.
pub struct LinearPolicy {
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
}

impl LinearPolicy {
    /// Create a new linear policy with zero-initialized parameters.
    ///
    /// `obs_scale` must have length `obs_dim`. Each observation element is
    /// multiplied by its corresponding scale before the linear transform.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, act_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "LinearPolicy::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        let n_params = act_dim * (obs_dim + 1);
        Self {
            obs_dim,
            act_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
        }
    }

    /// Compute scaled observation (shared by forward and gradient methods).
    fn scaled_obs(&self, obs: &[f32]) -> Vec<f64> {
        obs.iter()
            .zip(&self.obs_scale)
            .map(|(&o, &s)| f64::from(o) * s)
            .collect()
    }
}

impl Policy for LinearPolicy {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "LinearPolicy::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32]) -> Vec<f64> {
        let s = self.scaled_obs(obs);
        let mut mu = Vec::with_capacity(self.act_dim);
        for a in 0..self.act_dim {
            let mut z = self.params[self.act_dim * self.obs_dim + a]; // bias
            for (o, &so) in s.iter().enumerate() {
                z += self.params[a * self.obs_dim + o] * so;
            }
            mu.push(z.tanh());
        }
        mu
    }
}

impl DifferentiablePolicy for LinearPolicy {
    fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64> {
        let s = self.scaled_obs(obs);
        let mu = self.forward(obs);
        let sigma2 = sigma * sigma;
        let mut grad = vec![0.0; self.params.len()];

        for a in 0..self.act_dim {
            let tanh_deriv = mu[a].mul_add(-mu[a], 1.0);
            let score = (action[a] - mu[a]) / sigma2 * tanh_deriv;

            for o in 0..self.obs_dim {
                grad[a * self.obs_dim + o] = score * s[o];
            }
            grad[self.act_dim * self.obs_dim + a] = score;
        }
        grad
    }

    fn forward_vjp(&self, obs: &[f32], v: &[f64]) -> Vec<f64> {
        let s = self.scaled_obs(obs);
        let mu = self.forward(obs);
        let mut vjp = vec![0.0; self.params.len()];

        for a in 0..self.act_dim {
            let tanh_deriv = mu[a].mul_add(-mu[a], 1.0);

            for o in 0..self.obs_dim {
                vjp[a * self.obs_dim + o] = v[a] * tanh_deriv * s[o];
            }
            vjp[self.act_dim * self.obs_dim + a] = v[a] * tanh_deriv;
        }
        vjp
    }
}

// ── LinearValue ───────────────────────────────────────────────────────────

/// Linear value function: `V(s) = w · s_scaled + b`.
///
/// Implements [`ValueFn`]. Used by PPO for advantage estimation.
pub struct LinearValue {
    obs_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
}

impl LinearValue {
    /// Create a new linear value function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "LinearValue::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        Self {
            obs_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; obs_dim + 1],
        }
    }

    fn scaled_obs(&self, obs: &[f32]) -> Vec<f64> {
        obs.iter()
            .zip(&self.obs_scale)
            .map(|(&o, &s)| f64::from(o) * s)
            .collect()
    }
}

impl ValueFn for LinearValue {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "LinearValue::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32]) -> f64 {
        let s = self.scaled_obs(obs);
        let mut v = self.params[self.obs_dim]; // bias
        for (o, &so) in s.iter().enumerate() {
            v += self.params[o] * so;
        }
        v
    }

    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64> {
        let s = self.scaled_obs(obs);
        let v = self.forward(obs);
        let residual = 2.0 * (v - target);
        let mut grad = vec![0.0; self.params.len()];
        for o in 0..self.obs_dim {
            grad[o] = residual * s[o];
        }
        grad[self.obs_dim] = residual;
        grad
    }
}

// ── LinearQ ───────────────────────────────────────────────────────────────

/// Linear Q-function: `Q(s, a) = w · [s_scaled; a] + b`.
///
/// Implements [`QFunction`]. Used by SAC and TD3 (twin Q-networks).
pub struct LinearQ {
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
}

impl LinearQ {
    /// Create a new linear Q-function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, act_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "LinearQ::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        Self {
            obs_dim,
            act_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; obs_dim + act_dim + 1],
        }
    }

    fn scaled_obs(&self, obs: &[f32]) -> Vec<f64> {
        obs.iter()
            .zip(&self.obs_scale)
            .map(|(&o, &s)| f64::from(o) * s)
            .collect()
    }

    /// Build the concatenated input vector `[s_scaled; action]`.
    fn input_vec(&self, obs: &[f32], action: &[f64]) -> Vec<f64> {
        let mut input = self.scaled_obs(obs);
        input.extend_from_slice(action);
        input
    }
}

impl QFunction for LinearQ {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "LinearQ::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32], action: &[f64]) -> f64 {
        let input = self.input_vec(obs, action);
        let bias_idx = self.obs_dim + self.act_dim;
        let mut q = self.params[bias_idx]; // bias
        for (i, &x) in input.iter().enumerate() {
            q += self.params[i] * x;
        }
        q
    }

    fn mse_gradient(&self, obs: &[f32], action: &[f64], target: f64) -> Vec<f64> {
        let input = self.input_vec(obs, action);
        let q = self.forward(obs, action);
        let residual = 2.0 * (q - target);
        let mut grad = vec![0.0; self.params.len()];
        for (i, &x) in input.iter().enumerate() {
            grad[i] = residual * x;
        }
        grad[self.obs_dim + self.act_dim] = residual;
        grad
    }

    fn action_gradient(&self, _obs: &[f32], _action: &[f64]) -> Vec<f64> {
        // dQ/da[j] = w[obs_dim + j]
        self.params[self.obs_dim..self.obs_dim + self.act_dim].to_vec()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;

    // ── Test dimensions ───────────────────────────────────────────────

    const OBS_DIM: usize = 4;
    const ACT_DIM: usize = 2;
    const OBS_SCALE: [f64; OBS_DIM] = [0.5, 1.0, 0.1, 2.0];
    const FD_EPS: f64 = 1e-6;
    const FD_TOL: f64 = 1e-5;

    /// Deterministic test parameters (non-trivial values).
    fn test_params_policy() -> Vec<f64> {
        // W[2x4] + b[2] = 10 params
        vec![0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, 0.05, -0.1]
    }

    fn test_obs() -> Vec<f32> {
        vec![1.0_f32, -0.5, 3.0, 0.2]
    }

    fn test_action() -> Vec<f64> {
        vec![0.3, -0.7]
    }

    // ── LinearPolicy smoke tests ──────────────────────────────────────

    #[test]
    fn linear_policy_n_params() {
        let p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        assert_eq!(p.n_params(), ACT_DIM * (OBS_DIM + 1));
    }

    #[test]
    fn linear_policy_params_roundtrip() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let vals = test_params_policy();
        p.set_params(&vals);
        assert_eq!(p.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 10 params, got 3")]
    fn linear_policy_set_params_wrong_length() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&[1.0, 2.0, 3.0]);
    }

    #[test]
    fn linear_policy_forward_in_tanh_range() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());
        let mu = p.forward(&test_obs());
        assert_eq!(mu.len(), ACT_DIM);
        for &m in &mu {
            assert!((-1.0..=1.0).contains(&m), "tanh output {m} out of [-1,1]");
        }
    }

    // ── LinearPolicy FD: log_prob_gradient ────────────────────────────

    /// Compute log probability of action under Gaussian policy.
    fn log_prob(policy: &LinearPolicy, obs: &[f32], action: &[f64], sigma: f64) -> f64 {
        let mu = policy.forward(obs);
        let sigma2 = sigma * sigma;
        let mut lp = 0.0;
        for (a, m) in action.iter().zip(&mu) {
            lp -= (a - m) * (a - m) / (2.0 * sigma2);
        }
        lp
    }

    #[test]
    fn linear_policy_log_prob_gradient_matches_fd() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());
        let obs = test_obs();
        let action = test_action();
        let sigma = 0.5;

        let analytical = p.log_prob_gradient(&obs, &action, sigma);
        let mut base_params = p.params().to_vec();

        for i in 0..p.n_params() {
            let orig = base_params[i];

            base_params[i] = orig + FD_EPS;
            p.set_params(&base_params);
            let lp_plus = log_prob(&p, &obs, &action, sigma);

            base_params[i] = orig - FD_EPS;
            p.set_params(&base_params);
            let lp_minus = log_prob(&p, &obs, &action, sigma);

            base_params[i] = orig;

            let fd = (lp_plus - lp_minus) / (2.0 * FD_EPS);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < FD_TOL,
                "log_prob_gradient[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }

        // Restore original params.
        p.set_params(&base_params);
    }

    // ── LinearPolicy FD: forward_vjp ──────────────────────────────────

    #[test]
    fn linear_policy_forward_vjp_matches_fd() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());
        let obs = test_obs();
        let v = vec![0.7, -0.3]; // arbitrary tangent vector

        let analytical = p.forward_vjp(&obs, &v);
        let mut base_params = p.params().to_vec();

        for i in 0..p.n_params() {
            let orig = base_params[i];

            base_params[i] = orig + FD_EPS;
            p.set_params(&base_params);
            let fwd_plus = p.forward(&obs);

            base_params[i] = orig - FD_EPS;
            p.set_params(&base_params);
            let fwd_minus = p.forward(&obs);

            base_params[i] = orig;

            // v^T · (f(θ+ε) - f(θ-ε)) / 2ε
            let fd: f64 = v
                .iter()
                .zip(fwd_plus.iter().zip(&fwd_minus))
                .map(|(&vi, (&fp, &fm))| vi * (fp - fm) / (2.0 * FD_EPS))
                .sum();
            let err = (analytical[i] - fd).abs();
            assert!(
                err < FD_TOL,
                "forward_vjp[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }

        p.set_params(&base_params);
    }

    // ── LinearValue smoke tests ───────────────────────────────────────

    #[test]
    fn linear_value_n_params() {
        let v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        assert_eq!(v.n_params(), OBS_DIM + 1);
    }

    #[test]
    fn linear_value_params_roundtrip() {
        let mut v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        let vals = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        v.set_params(&vals);
        assert_eq!(v.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 5 params, got 2")]
    fn linear_value_set_params_wrong_length() {
        let mut v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        v.set_params(&[1.0, 2.0]);
    }

    // ── LinearValue FD: mse_gradient ──────────────────────────────────

    #[test]
    fn linear_value_mse_gradient_matches_fd() {
        let mut vf = LinearValue::new(OBS_DIM, &OBS_SCALE);
        let init_params = vec![0.3, -0.5, 0.8, -0.2, 0.1];
        vf.set_params(&init_params);
        let obs = test_obs();
        let target = 1.5;

        let analytical = vf.mse_gradient(&obs, target);
        let mut base_params = vf.params().to_vec();

        for i in 0..vf.n_params() {
            let orig = base_params[i];

            base_params[i] = orig + FD_EPS;
            vf.set_params(&base_params);
            let v_plus = vf.forward(&obs);
            let loss_plus = (v_plus - target) * (v_plus - target);

            base_params[i] = orig - FD_EPS;
            vf.set_params(&base_params);
            let v_minus = vf.forward(&obs);
            let loss_minus = (v_minus - target) * (v_minus - target);

            base_params[i] = orig;

            let fd = (loss_plus - loss_minus) / (2.0 * FD_EPS);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < FD_TOL,
                "value mse_gradient[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }

        vf.set_params(&base_params);
    }

    // ── LinearQ smoke tests ───────────────────────────────────────────

    #[test]
    fn linear_q_n_params() {
        let q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        assert_eq!(q.n_params(), OBS_DIM + ACT_DIM + 1);
    }

    #[test]
    fn linear_q_params_roundtrip() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let vals = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        q.set_params(&vals);
        assert_eq!(q.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 7 params, got 2")]
    fn linear_q_set_params_wrong_length() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        q.set_params(&[1.0, 2.0]);
    }

    // ── LinearQ FD: mse_gradient ──────────────────────────────────────

    #[test]
    fn linear_q_mse_gradient_matches_fd() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let init_params = vec![0.3, -0.5, 0.8, -0.2, 0.4, -0.6, 0.1];
        q.set_params(&init_params);
        let obs = test_obs();
        let action = test_action();
        let target = 2.0;

        let analytical = q.mse_gradient(&obs, &action, target);
        let mut base_params = q.params().to_vec();

        for i in 0..q.n_params() {
            let orig = base_params[i];

            base_params[i] = orig + FD_EPS;
            q.set_params(&base_params);
            let q_plus = q.forward(&obs, &action);
            let loss_plus = (q_plus - target) * (q_plus - target);

            base_params[i] = orig - FD_EPS;
            q.set_params(&base_params);
            let q_minus = q.forward(&obs, &action);
            let loss_minus = (q_minus - target) * (q_minus - target);

            base_params[i] = orig;

            let fd = (loss_plus - loss_minus) / (2.0 * FD_EPS);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < FD_TOL,
                "Q mse_gradient[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }

        q.set_params(&base_params);
    }

    // ── LinearQ FD: action_gradient ───────────────────────────────────

    #[test]
    fn linear_q_action_gradient_matches_fd() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let init_params = vec![0.3, -0.5, 0.8, -0.2, 0.4, -0.6, 0.1];
        q.set_params(&init_params);
        let obs = test_obs();
        let action = test_action();

        let analytical = q.action_gradient(&obs, &action);
        let mut base_action = action;

        for j in 0..ACT_DIM {
            let orig = base_action[j];

            base_action[j] = orig + FD_EPS;
            let q_plus = q.forward(&obs, &base_action);

            base_action[j] = orig - FD_EPS;
            let q_minus = q.forward(&obs, &base_action);

            base_action[j] = orig;

            let fd = (q_plus - q_minus) / (2.0 * FD_EPS);
            let err = (analytical[j] - fd).abs();
            assert!(
                err < FD_TOL,
                "Q action_gradient[{j}]: analytical={}, fd={fd}, err={err}",
                analytical[j],
            );
        }
    }

    // ── Cross-check: LinearPolicy matches PPO example ─────────────────

    #[test]
    fn linear_policy_matches_ppo_example_math() {
        // Reproduce the exact math from the PPO example with obs_dim=4, act_dim=2.
        let ppo_obs_scale = [
            1.0 / std::f64::consts::PI,
            1.0 / std::f64::consts::PI,
            0.1,
            0.1,
        ];
        let mut p = LinearPolicy::new(4, 2, &ppo_obs_scale);
        let params = vec![0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, 0.05, -0.1];
        p.set_params(&params);

        let obs = [1.0_f32, -0.5, 3.0, 0.2];

        // Manual computation: z[a] = bias + Σ W[a,o] * obs[o] * scale[o]
        let s0 = f64::from(obs[0]) * ppo_obs_scale[0];
        let s1 = f64::from(obs[1]) * ppo_obs_scale[1];
        let s2 = f64::from(obs[2]) * ppo_obs_scale[2];
        let s3 = f64::from(obs[3]) * ppo_obs_scale[3];

        let z0 = params[3].mul_add(
            s3,
            params[2].mul_add(s2, params[1].mul_add(s1, params[0].mul_add(s0, params[8]))),
        );
        let z1 = params[7].mul_add(
            s3,
            params[6].mul_add(s2, params[5].mul_add(s1, params[4].mul_add(s0, params[9]))),
        );

        let expected = [z0.tanh(), z1.tanh()];
        let actual = p.forward(&obs);

        for (i, (&e, &a)) in expected.iter().zip(&actual).enumerate() {
            assert!(
                (e - a).abs() < 1e-12,
                "forward[{i}]: expected={e}, actual={a}"
            );
        }
    }

    // ── Batch method tests ────────────────────────────────────────────

    fn test_obs2() -> Vec<f32> {
        vec![-0.3_f32, 2.0, 0.5, -1.0]
    }

    fn batch_obs() -> Vec<f32> {
        let mut batch = test_obs();
        batch.extend(test_obs2());
        batch
    }

    #[test]
    fn linear_policy_forward_batch_matches_loop() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());

        let obs_batch = batch_obs();
        let batch_result = p.forward_batch(&obs_batch, OBS_DIM);

        let single0 = p.forward(&test_obs());
        let single1 = p.forward(&test_obs2());

        assert_eq!(batch_result.len(), ACT_DIM * 2);
        for i in 0..ACT_DIM {
            assert!((batch_result[i] - single0[i]).abs() < 1e-12);
            assert!((batch_result[ACT_DIM + i] - single1[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn linear_policy_forward_vjp_batch_matches_loop() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());

        let obs_batch = batch_obs();
        let v_batch = vec![0.5, -0.3, 0.2, 0.8]; // 2 samples × 2 act_dim

        let batch_result = p.forward_vjp_batch(&obs_batch, &v_batch, OBS_DIM);

        let vjp0 = p.forward_vjp(&test_obs(), &v_batch[..ACT_DIM]);
        let vjp1 = p.forward_vjp(&test_obs2(), &v_batch[ACT_DIM..]);

        assert_eq!(batch_result.len(), p.n_params());
        for i in 0..p.n_params() {
            let expected = f64::midpoint(vjp0[i], vjp1[i]);
            assert!(
                (batch_result[i] - expected).abs() < 1e-12,
                "vjp_batch[{i}]: batch={}, manual={}",
                batch_result[i],
                expected,
            );
        }
    }

    #[test]
    fn linear_value_forward_batch_matches_loop() {
        let mut v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        v.set_params(&[0.3, -0.5, 0.8, -0.2, 0.1]);

        let obs_batch = batch_obs();
        let batch_result = v.forward_batch(&obs_batch, OBS_DIM);

        let single0 = v.forward(&test_obs());
        let single1 = v.forward(&test_obs2());

        assert_eq!(batch_result.len(), 2);
        assert!((batch_result[0] - single0).abs() < 1e-12);
        assert!((batch_result[1] - single1).abs() < 1e-12);
    }

    #[test]
    fn linear_value_mse_gradient_batch_matches_loop() {
        let mut v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        v.set_params(&[0.3, -0.5, 0.8, -0.2, 0.1]);

        let obs_batch = batch_obs();
        let targets = [1.0, -0.5];

        let batch_grad = v.mse_gradient_batch(&obs_batch, &targets, OBS_DIM);

        let grad0 = v.mse_gradient(&test_obs(), targets[0]);
        let grad1 = v.mse_gradient(&test_obs2(), targets[1]);

        assert_eq!(batch_grad.len(), v.n_params());
        for i in 0..v.n_params() {
            let expected = f64::midpoint(grad0[i], grad1[i]);
            assert!(
                (batch_grad[i] - expected).abs() < 1e-12,
                "mse_grad_batch[{i}]: batch={}, manual={}",
                batch_grad[i],
                expected,
            );
        }
    }

    #[test]
    fn linear_q_forward_batch_matches_loop() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let q_params: Vec<f64> = (0..q.n_params())
            .map(|i| (i as f64).mul_add(0.1, -0.3))
            .collect();
        q.set_params(&q_params);

        let obs_batch = batch_obs();
        let act0 = test_action();
        let act1 = vec![-0.2, 0.5];
        let actions: Vec<f64> = act0.iter().chain(act1.iter()).copied().collect();

        let batch_result = q.forward_batch(&obs_batch, &actions, OBS_DIM, ACT_DIM);

        let single0 = q.forward(&test_obs(), &act0);
        let single1 = q.forward(&test_obs2(), &act1);

        assert_eq!(batch_result.len(), 2);
        assert!((batch_result[0] - single0).abs() < 1e-12);
        assert!((batch_result[1] - single1).abs() < 1e-12);
    }

    #[test]
    fn linear_q_mse_gradient_batch_matches_loop() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let q_params: Vec<f64> = (0..q.n_params())
            .map(|i| (i as f64).mul_add(0.1, -0.3))
            .collect();
        q.set_params(&q_params);

        let obs_batch = batch_obs();
        let act0 = test_action();
        let act1 = vec![-0.2, 0.5];
        let actions: Vec<f64> = act0.iter().chain(act1.iter()).copied().collect();
        let targets = [2.0, -1.0];

        let batch_grad = q.mse_gradient_batch(&obs_batch, &actions, &targets, OBS_DIM, ACT_DIM);

        let grad0 = q.mse_gradient(&test_obs(), &act0, targets[0]);
        let grad1 = q.mse_gradient(&test_obs2(), &act1, targets[1]);

        assert_eq!(batch_grad.len(), q.n_params());
        for i in 0..q.n_params() {
            let expected = f64::midpoint(grad0[i], grad1[i]);
            assert!(
                (batch_grad[i] - expected).abs() < 1e-12,
                "q_mse_grad_batch[{i}]: batch={}, manual={}",
                batch_grad[i],
                expected,
            );
        }
    }

    #[test]
    fn linear_q_action_gradient_batch_matches_loop() {
        let mut q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        let q_params: Vec<f64> = (0..q.n_params())
            .map(|i| (i as f64).mul_add(0.1, -0.3))
            .collect();
        q.set_params(&q_params);

        let obs_batch = batch_obs();
        let act0 = test_action();
        let act1 = vec![-0.2, 0.5];
        let actions: Vec<f64> = act0.iter().chain(act1.iter()).copied().collect();

        let batch_result = q.action_gradient_batch(&obs_batch, &actions, OBS_DIM, ACT_DIM);

        let single0 = q.action_gradient(&test_obs(), &act0);
        let single1 = q.action_gradient(&test_obs2(), &act1);

        assert_eq!(batch_result.len(), ACT_DIM * 2);
        for i in 0..ACT_DIM {
            assert!((batch_result[i] - single0[i]).abs() < 1e-12);
            assert!((batch_result[ACT_DIM + i] - single1[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn batch_empty_returns_empty_or_zeros() {
        let p = LinearPolicy::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        assert!(p.forward_batch(&[], OBS_DIM).is_empty());
        assert_eq!(
            p.forward_vjp_batch(&[], &[], OBS_DIM),
            vec![0.0; p.n_params()]
        );

        let v = LinearValue::new(OBS_DIM, &OBS_SCALE);
        assert!(v.forward_batch(&[], OBS_DIM).is_empty());
        assert_eq!(
            v.mse_gradient_batch(&[], &[], OBS_DIM),
            vec![0.0; v.n_params()]
        );

        let q = LinearQ::new(OBS_DIM, ACT_DIM, &OBS_SCALE);
        assert!(q.forward_batch(&[], &[], OBS_DIM, ACT_DIM).is_empty());
        assert_eq!(
            q.mse_gradient_batch(&[], &[], &[], OBS_DIM, ACT_DIM),
            vec![0.0; q.n_params()]
        );
        assert!(
            q.action_gradient_batch(&[], &[], OBS_DIM, ACT_DIM)
                .is_empty()
        );
    }
}
