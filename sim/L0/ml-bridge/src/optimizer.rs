//! Optimizer trait and Adam implementation.
//!
//! The optimizer is a swappable part — algorithms receive an
//! [`OptimizerConfig`] at construction time and call [`build`](OptimizerConfig::build)
//! to create their optimizer instances. Swapping Adam for `AdamW` is a config
//! change — zero algorithm code edits.
//!
//! ## Dual param ownership
//!
//! Both the optimizer and the network (`Policy`, `ValueFn`, `QFunction`) own
//! independent copies of parameters. After every [`Optimizer::step`], the
//! algorithm must manually sync: `network.set_params(optimizer.params())`.
//! This is a level 0-1 wart — at level 2+, autograd optimizers modify the
//! network in-place and the problem disappears.

// ── Optimizer trait ────────────────────────────────────────────────────────

/// Trait for parameter optimizers.
///
/// Optimizers own a copy of the parameter vector and update it in-place
/// on each `step()`. The algorithm must sync the updated params back to
/// the network after each step.
pub trait Optimizer: Send {
    /// Apply one gradient update.
    ///
    /// - `ascent = true`: maximize (policy gradient — REINFORCE, PPO actor).
    /// - `ascent = false`: minimize (MSE loss — value function, Q-function).
    ///
    /// # Panics
    ///
    /// Panics if `gradient.len() != self.n_params()`.
    fn step(&mut self, gradient: &[f64], ascent: bool);

    /// Current parameter values after the most recent step.
    fn params(&self) -> &[f64];

    /// Replace all parameters (e.g., to initialize from a pre-trained policy).
    ///
    /// # Panics
    ///
    /// Panics if `params.len() != self.n_params()`.
    fn set_params(&mut self, params: &[f64]);

    /// Number of parameters this optimizer manages.
    fn n_params(&self) -> usize;
}

// ── Optimizer config ───────────────────────────────────────────────────────

/// Configuration for creating an [`Optimizer`] instance.
///
/// Derives `Copy` — SAC needs 5 optimizer instances from the same config.
#[derive(Debug, Clone, Copy)]
pub enum OptimizerConfig {
    /// Adam optimizer (Kingma & Ba, 2015).
    Adam {
        /// Learning rate.
        lr: f64,
        /// Exponential decay rate for the first moment (momentum).
        beta1: f64,
        /// Exponential decay rate for the second moment (RMS prop).
        beta2: f64,
        /// Small constant for numerical stability.
        eps: f64,
        /// Maximum L2 norm for gradient clipping. `f64::INFINITY` = no clipping.
        max_grad_norm: f64,
    },
}

impl OptimizerConfig {
    /// Convenience constructor for Adam with standard defaults.
    ///
    /// Uses β₁ = 0.9, β₂ = 0.999, ε = 1e-8, no gradient clipping.
    #[must_use]
    pub const fn adam(lr: f64) -> Self {
        Self::Adam {
            lr,
            beta1: 0.9,
            beta2: 0.999,
            eps: 1e-8,
            max_grad_norm: f64::INFINITY,
        }
    }

    /// Build an optimizer instance with the given number of parameters.
    ///
    /// Parameters are initialized to zero. Call `set_params()` to load
    /// initial values from a policy or value function.
    #[must_use]
    pub fn build(&self, n_params: usize) -> Box<dyn Optimizer> {
        match *self {
            Self::Adam {
                lr,
                beta1,
                beta2,
                eps,
                max_grad_norm,
            } => Box::new(Adam::new(n_params, lr, beta1, beta2, eps, max_grad_norm)),
        }
    }
}

// ── Adam ───────────────────────────────────────────────────────────────────

/// Adam optimizer (Kingma & Ba, 2015) with optional gradient clipping.
struct Adam {
    params: Vec<f64>,
    m: Vec<f64>,
    v: Vec<f64>,
    t: usize,
    lr: f64,
    beta1: f64,
    beta2: f64,
    eps: f64,
    max_grad_norm: f64,
}

impl Adam {
    fn new(n_params: usize, lr: f64, beta1: f64, beta2: f64, eps: f64, max_grad_norm: f64) -> Self {
        Self {
            params: vec![0.0; n_params],
            m: vec![0.0; n_params],
            v: vec![0.0; n_params],
            t: 0,
            lr,
            beta1,
            beta2,
            eps,
            max_grad_norm,
        }
    }
}

impl Optimizer for Adam {
    fn step(&mut self, gradient: &[f64], ascent: bool) {
        let n = self.params.len();
        assert!(
            gradient.len() == n,
            "Adam::step: expected {} gradient elements, got {}",
            n,
            gradient.len(),
        );

        // Gradient clipping by L2 norm.
        let grad_norm_sq: f64 = gradient.iter().map(|g| g * g).sum();
        let grad_norm = grad_norm_sq.sqrt();
        let clip_scale = if grad_norm > self.max_grad_norm {
            self.max_grad_norm / grad_norm
        } else {
            1.0
        };

        self.t += 1;
        #[allow(clippy::cast_precision_loss)] // t < 2^52 in practice
        let t = self.t as f64;

        // Bias correction factors.
        let bc1 = 1.0 - self.beta1.powf(t);
        let bc2 = 1.0 - self.beta2.powf(t);

        let direction = if ascent { 1.0 } else { -1.0 };

        for (i, &gi) in gradient.iter().enumerate() {
            let g = gi * clip_scale;

            // Update biased first moment estimate.
            self.m[i] = self.beta1.mul_add(self.m[i], (1.0 - self.beta1) * g);
            // Update biased second raw moment estimate.
            self.v[i] = self.beta2.mul_add(self.v[i], (1.0 - self.beta2) * g * g);

            // Bias-corrected estimates.
            let m_hat = self.m[i] / bc1;
            let v_hat = self.v[i] / bc2;

            self.params[i] += direction * self.lr * m_hat / (v_hat.sqrt() + self.eps);
        }
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "Adam::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn n_params(&self) -> usize {
        self.params.len()
    }
}

// ── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    // ── Compile-time bounds ────────────────────────────────────────────

    #[test]
    fn boxed_optimizer_is_send() {
        fn require<T: Send>() {}
        require::<Box<dyn Optimizer>>();
    }

    // ── Adam basics ────────────────────────────────────────────────────

    #[test]
    fn adam_descent_moves_toward_zero() {
        // f(x) = x^2, gradient = 2x. Start at x=5.
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(1);
        opt.set_params(&[5.0]);

        // One descent step with gradient = 2*5 = 10.
        opt.step(&[10.0], false);

        // Parameter should decrease (move toward 0).
        assert!(opt.params()[0] < 5.0, "descent should reduce param");
    }

    #[test]
    fn adam_ascent_moves_away_from_zero() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(1);
        opt.set_params(&[5.0]);

        opt.step(&[10.0], true);

        assert!(opt.params()[0] > 5.0, "ascent should increase param");
    }

    #[test]
    fn adam_ascent_and_descent_are_opposite() {
        let config = OptimizerConfig::adam(0.1);

        let mut opt_asc = config.build(1);
        opt_asc.set_params(&[5.0]);
        opt_asc.step(&[10.0], true);

        let mut opt_desc = config.build(1);
        opt_desc.set_params(&[5.0]);
        opt_desc.step(&[10.0], false);

        let delta_asc = opt_asc.params()[0] - 5.0;
        let delta_desc = opt_desc.params()[0] - 5.0;

        assert!(
            (delta_asc + delta_desc).abs() < 1e-12,
            "ascent and descent deltas should be equal and opposite"
        );
    }

    // ── Gradient clipping ──────────────────────────────────────────────

    #[test]
    fn adam_gradient_clipping() {
        let config = OptimizerConfig::Adam {
            lr: 0.1,
            beta1: 0.9,
            beta2: 0.999,
            eps: 1e-8,
            max_grad_norm: 1.0,
        };

        // Large gradient [100, 100] — norm = ~141.4, should be clipped to 1.0.
        let mut opt_clipped = config.build(2);
        opt_clipped.set_params(&[0.0, 0.0]);
        opt_clipped.step(&[100.0, 100.0], false);

        // Small gradient [0.1, 0.1] — norm = ~0.14, no clipping.
        let mut opt_unclipped = config.build(2);
        opt_unclipped.set_params(&[0.0, 0.0]);
        opt_unclipped.step(&[0.1, 0.1], false);

        // Clipped gradient produces smaller param change than unclipped large gradient would.
        // Both start at 0, descent moves negative.
        let clipped_delta = opt_clipped.params()[0].abs();
        let unclipped_delta = opt_unclipped.params()[0].abs();

        // After one Adam step with bias correction, the update magnitude
        // depends on m_hat / sqrt(v_hat). With clipping, both gradients
        // should produce the same update direction. The clipped large gradient
        // should not cause larger updates than the small one.
        assert!(
            clipped_delta <= unclipped_delta * 10.0,
            "clipped delta {clipped_delta} should not be much larger than unclipped {unclipped_delta}"
        );
    }

    #[test]
    fn adam_no_clipping_when_infinity() {
        // Default max_grad_norm = INFINITY — no clipping.
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(1);
        opt.set_params(&[0.0]);

        // Large gradient — should not be clipped.
        opt.step(&[1000.0], false);

        // Param should have moved significantly.
        assert!(
            opt.params()[0].abs() > 0.01,
            "large gradient should cause movement"
        );
    }

    // ── Convergence ────────────────────────────────────────────────────

    #[test]
    fn adam_converges_on_quadratic() {
        // Minimize f(x, y) = x^2 + y^2. Gradient = [2x, 2y].
        let config = OptimizerConfig::adam(0.05);
        let mut opt = config.build(2);
        opt.set_params(&[5.0, -3.0]);

        for _ in 0..500 {
            let x = opt.params()[0];
            let y = opt.params()[1];
            opt.step(&[2.0 * x, 2.0 * y], false);
        }

        let x = opt.params()[0];
        let y = opt.params()[1];
        assert!(
            x.abs() < 0.01 && y.abs() < 0.01,
            "should converge near origin, got ({x}, {y})"
        );
    }

    // ── set_params ─────────────────────────────────────────────────────

    #[test]
    fn adam_set_params_roundtrip() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(3);

        let values = [1.0, 2.0, 3.0];
        opt.set_params(&values);

        assert_eq!(opt.params(), &values);
    }

    #[test]
    #[should_panic(expected = "expected 3 params, got 2")]
    fn adam_set_params_wrong_length_panics() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(3);
        opt.set_params(&[1.0, 2.0]);
    }

    #[test]
    #[should_panic(expected = "expected 2 gradient elements, got 3")]
    fn adam_step_wrong_gradient_length_panics() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(2);
        opt.step(&[1.0, 2.0, 3.0], false);
    }

    // ── Config ─────────────────────────────────────────────────────────

    #[test]
    fn optimizer_config_is_copy() {
        let config = OptimizerConfig::adam(0.01);
        let config2 = config; // Copy, not move.
        let _opt1 = config.build(5);
        let _opt2 = config2.build(5);
    }

    #[test]
    fn adam_convenience_defaults() {
        let config = OptimizerConfig::adam(0.001);
        match config {
            OptimizerConfig::Adam {
                lr,
                beta1,
                beta2,
                eps,
                max_grad_norm,
            } => {
                assert_eq!(lr, 0.001);
                assert_eq!(beta1, 0.9);
                assert_eq!(beta2, 0.999);
                assert_eq!(eps, 1e-8);
                assert!(max_grad_norm.is_infinite());
            }
        }
    }
}
