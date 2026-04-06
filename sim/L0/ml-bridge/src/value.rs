//! Value function traits — critics for actor-critic algorithms.
//!
//! - [`ValueFn`] — state value V(s). Used by PPO (advantage estimation).
//! - [`QFunction`] — state-action value Q(s, a). Used by SAC and TD3 (twin
//!   Q-networks, target network soft updates, deterministic/reparameterized
//!   policy gradients).
//!
//! Also provides [`soft_update`] — Polyak averaging for target network
//! updates, used by all off-policy actor-critic methods.

// ── State value function ───────────────────────────────────────────────────

/// State value function V(s).
///
/// Used by PPO for advantage estimation: A(s) = R − V(s).
/// Optionally used by A2C, PPG, and other on-policy methods.
///
/// # Parameter contract
///
/// Same as [`Policy`](crate::Policy): `params()` returns `&[f64]` of length
/// `n_params()`, `set_params()` panics on wrong length.
pub trait ValueFn: Send + Sync {
    /// Number of learnable parameters.
    fn n_params(&self) -> usize;

    /// Current parameter values as a flat slice.
    fn params(&self) -> &[f64];

    /// Replace all parameters.
    ///
    /// # Panics
    ///
    /// Panics if `params.len() != self.n_params()`.
    fn set_params(&mut self, params: &[f64]);

    /// Predict V(s) — expected return from state `obs`.
    fn forward(&self, obs: &[f32]) -> f64;

    /// Gradient of MSE loss w.r.t. parameters.
    ///
    /// Returns d/d(params) [(V(obs) − target)²], a vector of length
    /// `n_params()`. The factor of 2 is included.
    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64>;
}

// ── State-action value function ────────────────────────────────────────────

/// State-action value function Q(s, a).
///
/// Used by SAC and TD3 (twin Q-networks). Each algorithm maintains two
/// Q-functions (Q1, Q2) and takes the minimum for target computation
/// (clipped double-Q, reduces overestimation bias).
///
/// # Parameter contract
///
/// Same as [`ValueFn`]: `set_params()` panics on wrong length.
pub trait QFunction: Send + Sync {
    /// Number of learnable parameters.
    fn n_params(&self) -> usize;

    /// Current parameter values as a flat slice.
    fn params(&self) -> &[f64];

    /// Replace all parameters.
    ///
    /// # Panics
    ///
    /// Panics if `params.len() != self.n_params()`.
    fn set_params(&mut self, params: &[f64]);

    /// Predict Q(s, a) — expected return from state `obs` taking action `action`.
    fn forward(&self, obs: &[f32], action: &[f64]) -> f64;

    /// Gradient of MSE loss w.r.t. Q-function parameters.
    ///
    /// Returns d/d(params) [(Q(obs, action) − target)²], a vector of length
    /// `n_params()`. The factor of 2 is included.
    fn mse_gradient(&self, obs: &[f32], action: &[f64], target: f64) -> Vec<f64>;

    /// Gradient of Q(s, a) w.r.t. action.
    ///
    /// Returns dQ/da, a vector of length `action.len()`.
    ///
    /// Used by TD3 for the deterministic policy gradient:
    ///   dJ/dθ = dQ/da · dμ/dθ
    ///
    /// Used by SAC for the reparameterized policy gradient.
    fn action_gradient(&self, obs: &[f32], action: &[f64]) -> Vec<f64>;
}

// ── Target network utilities ───────────────────────────────────────────────

/// Polyak averaging: target = τ · source + (1 − τ) · target.
///
/// Used by SAC and TD3 for target network soft updates. Typical τ = 0.005.
///
/// This is a free function (not a trait method) because it's a pattern used
/// across algorithms, not an inherent capability of Q-functions.
pub fn soft_update(target: &mut dyn QFunction, source: &dyn QFunction, tau: f64) {
    let src = source.params();
    let tgt = target.params().to_vec();
    let updated: Vec<f64> = tgt
        .iter()
        .zip(src)
        .map(|(&t, &s)| tau.mul_add(s, (1.0 - tau) * t))
        .collect();
    target.set_params(&updated);
}

/// Polyak averaging for [`ValueFn`] targets.
///
/// Same as [`soft_update`] but for state value functions.
pub fn soft_update_value(target: &mut dyn ValueFn, source: &dyn ValueFn, tau: f64) {
    let src = source.params();
    let tgt = target.params().to_vec();
    let updated: Vec<f64> = tgt
        .iter()
        .zip(src)
        .map(|(&t, &s)| tau.mul_add(s, (1.0 - tau) * t))
        .collect();
    target.set_params(&updated);
}

// ── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn boxed_value_types_are_send_sync() {
        fn require<T: Send + Sync>() {}
        require::<Box<dyn ValueFn>>();
        require::<Box<dyn QFunction>>();
    }

    // ── Mock Q-function for soft_update tests ──────────────────────────

    struct MockQ {
        p: Vec<f64>,
    }

    impl MockQ {
        fn new(p: Vec<f64>) -> Self {
            Self { p }
        }
    }

    impl QFunction for MockQ {
        fn n_params(&self) -> usize {
            self.p.len()
        }
        fn params(&self) -> &[f64] {
            &self.p
        }
        fn set_params(&mut self, params: &[f64]) {
            assert!(
                params.len() == self.p.len(),
                "MockQ::set_params: expected {} params, got {}",
                self.p.len(),
                params.len(),
            );
            self.p.copy_from_slice(params);
        }
        fn forward(&self, _obs: &[f32], _action: &[f64]) -> f64 {
            0.0
        }
        fn mse_gradient(&self, _obs: &[f32], _action: &[f64], _target: f64) -> Vec<f64> {
            vec![0.0; self.p.len()]
        }
        fn action_gradient(&self, _obs: &[f32], _action: &[f64]) -> Vec<f64> {
            vec![0.0]
        }
    }

    #[test]
    fn soft_update_interpolates() {
        let source = MockQ::new(vec![10.0, 20.0, 30.0]);
        let mut target = MockQ::new(vec![0.0, 0.0, 0.0]);

        soft_update(&mut target, &source, 0.1);

        // target = 0.1 * source + 0.9 * target_old
        assert_eq!(target.params(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn soft_update_tau_one_copies() {
        let source = MockQ::new(vec![5.0, 10.0]);
        let mut target = MockQ::new(vec![100.0, 200.0]);

        soft_update(&mut target, &source, 1.0);

        assert_eq!(target.params(), &[5.0, 10.0]);
    }

    #[test]
    fn soft_update_tau_zero_preserves() {
        let source = MockQ::new(vec![5.0, 10.0]);
        let mut target = MockQ::new(vec![100.0, 200.0]);

        soft_update(&mut target, &source, 0.0);

        assert_eq!(target.params(), &[100.0, 200.0]);
    }
}
