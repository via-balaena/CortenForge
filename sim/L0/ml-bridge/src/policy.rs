//! Policy traits — the chassis rails for every RL algorithm.
//!
//! Three tiers, each extending the one below:
//!
//! - [`Policy`] — base trait. Forward pass + parameter access. Enough for
//!   evolutionary methods (CEM, CMA-ES) that perturb params directly.
//! - [`DifferentiablePolicy`] — adds gradient computation. Required by all
//!   gradient-based algorithms (REINFORCE, PPO, TD3).
//! - [`StochasticPolicy`] — adds learned exploration (`log_std` as a parameter).
//!   Required by entropy-based methods (SAC).
//!
//! Algorithms declare which tier they need via trait bounds on their
//! constructor. Swapping one policy implementation for another is a
//! one-line change — the algorithm never knows the difference.

// ── Base policy ────────────────────────────────────────────────────────────

/// Base policy — enough for evolutionary methods (CEM, CMA-ES).
///
/// Does not require gradient computation. CEM perturbs `params()` directly,
/// evaluates fitness via `forward()`, and selects elites.
///
/// # Parameter contract
///
/// - `params()` returns a contiguous `&[f64]` of length `n_params()`.
/// - `set_params()` accepts a slice of the same length. **Panics** if the
///   length doesn't match — wrong-length params is always a programming error.
/// - Parameters are `f64` (policy-internal precision). Observations are `f32`
///   (ML convention). The policy owns this conversion.
pub trait Policy: Send + Sync {
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

    /// Deterministic forward pass: observation → mean action.
    fn forward(&self, obs: &[f32]) -> Vec<f64>;
}

// ── Differentiable policy ──────────────────────────────────────────────────

/// Extends [`Policy`] with gradient computation.
///
/// Required by all gradient-based algorithms: REINFORCE, PPO, TD3.
///
/// At levels 0-1, gradients are hand-coded per policy architecture.
/// At level 2+, an autograd backend (burn, candle) computes them
/// automatically from the forward pass — the algorithm never knows
/// the difference.
pub trait DifferentiablePolicy: Policy {
    /// Gradient of log π(a|s) w.r.t. policy parameters.
    ///
    /// Assumes a Gaussian policy: π(a|s) = N(a; forward(s), σ²I).
    /// Returns d/dθ log π(a|s), a vector of length `n_params()`.
    ///
    /// Used by REINFORCE and PPO for the policy gradient:
    ///   ∇J ≈ Σ [∇log π(a|s) · advantage]
    fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64>;

    /// Vector-Jacobian product: v^T · d(forward(obs))/d(params).
    ///
    /// TD3 passes dQ/da as `v` to compute the deterministic policy gradient:
    ///   dJ/dθ = dQ/da · d(μ)/d(θ)
    ///
    /// Returns a vector of length `n_params()`.
    ///
    /// At levels 0-1, hand-coded per policy architecture.
    /// At level 2+, autograd computes this via `backward()`.
    fn forward_vjp(&self, obs: &[f32], v: &[f64]) -> Vec<f64>;
}

// ── Stochastic policy ──────────────────────────────────────────────────────

/// Extends [`DifferentiablePolicy`] with learned exploration.
///
/// Required by SAC (entropy-regularized exploration). Algorithms that use
/// fixed sigma schedules (REINFORCE, PPO) use [`DifferentiablePolicy`]
/// instead — they don't need learned `log_std`.
///
/// The key addition: `log_std` is part of the policy's parameters, not an
/// external schedule. The policy learns how much to explore.
pub trait StochasticPolicy: DifferentiablePolicy {
    /// Forward pass returning `(mean, log_std)` per action dimension.
    fn forward_stochastic(&self, obs: &[f32]) -> (Vec<f64>, Vec<f64>);

    /// Gradient of log π(a|s) w.r.t. all parameters (including `log_std`).
    ///
    /// Unlike [`DifferentiablePolicy::log_prob_gradient`], sigma is not an
    /// external parameter — it's derived from the policy's own `log_std` output.
    fn log_prob_gradient_stochastic(&self, obs: &[f32], action: &[f64]) -> Vec<f64>;

    /// Entropy of the policy at a given state: H[π(·|s)].
    fn entropy(&self, obs: &[f32]) -> f64;

    /// Vector-Jacobian product for the reparameterized action.
    ///
    /// Given obs and noise ε, the reparameterized action is:
    ///   a = μ(obs) + exp(`log_std`) · ε
    ///
    /// Returns v^T · d(a)/d(params), where params includes `log_std`.
    ///
    /// SAC uses this for its actor gradient:
    ///   dJ/dθ = α · d(log π)/dθ − dQ/da · d(a)/dθ
    ///
    /// `dQ/da` comes from [`QFunction::action_gradient`](crate::QFunction::action_gradient).
    /// `d(log π)/dθ` comes from [`log_prob_gradient_stochastic`](StochasticPolicy::log_prob_gradient_stochastic).
    /// `d(a)/dθ` comes from this method.
    fn reparameterized_vjp(&self, obs: &[f32], eps: &[f64], v: &[f64]) -> Vec<f64>;
}

// ── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Boxed trait objects must be Send + Sync (required for Bevy resources
    // and async training loops).  This also proves object safety.
    #[test]
    fn boxed_policy_is_send_sync() {
        fn require<T: Send + Sync>() {}
        require::<Box<dyn Policy>>();
        require::<Box<dyn DifferentiablePolicy>>();
        require::<Box<dyn StochasticPolicy>>();
    }
}
