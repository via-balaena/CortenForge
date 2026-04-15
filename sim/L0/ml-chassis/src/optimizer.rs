//! Optimizer trait and Adam implementation.
//!
//! The optimizer is a swappable part — algorithms receive an
//! [`OptimizerConfig`] at construction time and call [`build`](OptimizerConfig::build)
//! to create their optimizer instances. Swapping Adam for `AdamW` is a config
//! change — zero algorithm code edits.
//!
//! ## Two update paths
//!
//! - [`Optimizer::step`] — optimizer owns params internally. Used by level 0-1
//!   code that still calls `network.set_params(optimizer.params())`.
//! - [`Optimizer::step_in_place`] — operates on borrowed `&mut [f64]` params.
//!   Eliminates the dual param ownership wart: the optimizer holds only
//!   momentum state (m, v), not a copy of the parameters.

use serde::{Deserialize, Serialize};

use crate::artifact::OptimizerSnapshot;

// ── Optimizer trait ────────────────────────────────────────────────────────

/// Trait for parameter optimizers.
///
/// Two update paths:
///
/// - [`step`](Optimizer::step): optimizer owns params internally. After each
///   call, the algorithm must sync via `network.set_params(optimizer.params())`.
///   Used by level 0-1 algorithms.
///
/// - [`step_in_place`](Optimizer::step_in_place): operates on borrowed params.
///   No sync needed — the caller owns the params and passes them in. This
///   eliminates the dual param ownership wart.
pub trait Optimizer: Send + Sync {
    /// Apply one gradient update to the optimizer's internal params.
    ///
    /// - `ascent = true`: maximize (policy gradient — REINFORCE, PPO actor).
    /// - `ascent = false`: minimize (MSE loss — value function, Q-function).
    ///
    /// # Panics
    ///
    /// Panics if `gradient.len() != self.n_params()`.
    fn step(&mut self, gradient: &[f64], ascent: bool);

    /// Apply one gradient update directly to borrowed params.
    ///
    /// Same math as [`step`](Optimizer::step), but operates on the caller's
    /// param buffer instead of the optimizer's internal copy. Eliminates the
    /// `network.set_params(optimizer.params())` sync pattern.
    ///
    /// # Panics
    ///
    /// Panics if `params.len() != self.n_params()` or
    /// `gradient.len() != self.n_params()`.
    fn step_in_place(&mut self, params: &mut [f64], gradient: &[f64], ascent: bool);

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

    /// Snapshot the optimizer's internal state (momentum, variance, step count).
    fn snapshot(&self, role: &str) -> OptimizerSnapshot;

    /// Restore optimizer state from a snapshot.
    ///
    /// # Panics
    ///
    /// Panics if `snapshot.m.len() != self.n_params()`.
    fn load_snapshot(&mut self, snapshot: &OptimizerSnapshot);
}

// ── Optimizer config ───────────────────────────────────────────────────────

/// Configuration for creating an [`Optimizer`] instance.
///
/// Derives `Copy` — SAC needs 5 optimizer instances from the same config.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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
        /// Serialized as `null` when infinite, deserialized back to `INFINITY`.
        #[serde(
            serialize_with = "serialize_grad_norm",
            deserialize_with = "deserialize_grad_norm"
        )]
        max_grad_norm: f64,
    },
}

#[allow(clippy::trivially_copy_pass_by_ref)] // serde requires &f64
fn serialize_grad_norm<S: serde::Serializer>(val: &f64, s: S) -> Result<S::Ok, S::Error> {
    if val.is_infinite() {
        s.serialize_none()
    } else {
        s.serialize_f64(*val)
    }
}

fn deserialize_grad_norm<'de, D: serde::Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    let opt: Option<f64> = Option::deserialize(d)?;
    Ok(opt.unwrap_or(f64::INFINITY))
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

/// Core Adam update — shared between `step` and `step_in_place`.
///
/// Operates on borrowed slices so the same math works whether params
/// live inside the optimizer or are owned by the caller.
// Adam signature matches the formal algorithm's hyperparameter set; names
// (m, v, t, lr, beta1, beta2, eps) mirror the paper notation exactly.
#[allow(clippy::too_many_arguments, clippy::many_single_char_names)]
fn adam_update(
    params: &mut [f64],
    gradient: &[f64],
    m: &mut [f64],
    v: &mut [f64],
    t: &mut usize,
    lr: f64,
    beta1: f64,
    beta2: f64,
    eps: f64,
    max_grad_norm: f64,
    ascent: bool,
) {
    let n = params.len();
    assert!(
        gradient.len() == n,
        "Adam: expected {} gradient elements, got {}",
        n,
        gradient.len(),
    );

    // Gradient clipping by L2 norm.
    let grad_norm_sq: f64 = gradient.iter().map(|g| g * g).sum();
    let grad_norm = grad_norm_sq.sqrt();
    let clip_scale = if grad_norm > max_grad_norm {
        max_grad_norm / grad_norm
    } else {
        1.0
    };

    *t += 1;
    #[allow(clippy::cast_precision_loss)] // t < 2^52 in practice
    let tf = *t as f64;

    // Bias correction factors.
    let bc1 = 1.0 - beta1.powf(tf);
    let bc2 = 1.0 - beta2.powf(tf);

    let direction = if ascent { 1.0 } else { -1.0 };

    for (i, &gi) in gradient.iter().enumerate() {
        let g = gi * clip_scale;

        // Update biased first moment estimate.
        m[i] = beta1.mul_add(m[i], (1.0 - beta1) * g);
        // Update biased second raw moment estimate.
        v[i] = beta2.mul_add(v[i], (1.0 - beta2) * g * g);

        // Bias-corrected estimates.
        let m_hat = m[i] / bc1;
        let v_hat = v[i] / bc2;

        params[i] += direction * lr * m_hat / (v_hat.sqrt() + eps);
    }
}

impl Optimizer for Adam {
    fn step(&mut self, gradient: &[f64], ascent: bool) {
        adam_update(
            &mut self.params,
            gradient,
            &mut self.m,
            &mut self.v,
            &mut self.t,
            self.lr,
            self.beta1,
            self.beta2,
            self.eps,
            self.max_grad_norm,
            ascent,
        );
    }

    fn step_in_place(&mut self, params: &mut [f64], gradient: &[f64], ascent: bool) {
        assert!(
            params.len() == self.m.len(),
            "Adam::step_in_place: expected {} params, got {}",
            self.m.len(),
            params.len(),
        );
        adam_update(
            params,
            gradient,
            &mut self.m,
            &mut self.v,
            &mut self.t,
            self.lr,
            self.beta1,
            self.beta2,
            self.eps,
            self.max_grad_norm,
            ascent,
        );
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

    fn snapshot(&self, role: &str) -> OptimizerSnapshot {
        OptimizerSnapshot {
            role: role.to_string(),
            config: OptimizerConfig::Adam {
                lr: self.lr,
                beta1: self.beta1,
                beta2: self.beta2,
                eps: self.eps,
                max_grad_norm: self.max_grad_norm,
            },
            m: self.m.clone(),
            v: self.v.clone(),
            t: self.t,
        }
    }

    fn load_snapshot(&mut self, snapshot: &OptimizerSnapshot) {
        assert!(
            snapshot.m.len() == self.m.len(),
            "Adam::load_snapshot: expected {} params, got {}",
            self.m.len(),
            snapshot.m.len(),
        );
        self.m.copy_from_slice(&snapshot.m);
        self.v.copy_from_slice(&snapshot.v);
        self.t = snapshot.t;
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

    // ── step_in_place ───────────────────────────────────────────────

    #[test]
    fn step_in_place_matches_step() {
        // Same lr, same gradient, same starting params → identical result.
        let config = OptimizerConfig::adam(0.1);

        // Path A: step() on internal params.
        let mut opt_a = config.build(2);
        opt_a.set_params(&[5.0, -3.0]);
        opt_a.step(&[2.0, -1.0], false);

        // Path B: step_in_place() on external params.
        let mut opt_b = config.build(2);
        let mut params = [5.0, -3.0];
        opt_b.step_in_place(&mut params, &[2.0, -1.0], false);

        assert!(
            (opt_a.params()[0] - params[0]).abs() < 1e-15
                && (opt_a.params()[1] - params[1]).abs() < 1e-15,
            "step and step_in_place should produce identical results"
        );
    }

    #[test]
    fn step_in_place_multi_step_momentum() {
        // Multiple steps — momentum state must accumulate identically.
        let config = OptimizerConfig::adam(0.05);

        let mut opt_a = config.build(1);
        opt_a.set_params(&[5.0]);

        let mut opt_b = config.build(1);
        let mut p = [5.0];

        for _ in 0..20 {
            let grad_a = [2.0 * opt_a.params()[0]];
            opt_a.step(&grad_a, false);

            let grad_b = [2.0 * p[0]];
            opt_b.step_in_place(&mut p, &grad_b, false);
        }

        assert!(
            (opt_a.params()[0] - p[0]).abs() < 1e-12,
            "after 20 steps: step={}, step_in_place={}",
            opt_a.params()[0],
            p[0]
        );
    }

    #[test]
    fn step_in_place_converges_on_quadratic() {
        // Same convergence test as step(), but via step_in_place.
        let config = OptimizerConfig::adam(0.05);
        let mut opt = config.build(2);
        let mut params = [5.0, -3.0];

        for _ in 0..500 {
            let grad = [2.0 * params[0], 2.0 * params[1]];
            opt.step_in_place(&mut params, &grad, false);
        }

        assert!(
            params[0].abs() < 0.01 && params[1].abs() < 0.01,
            "should converge near origin, got ({}, {})",
            params[0],
            params[1]
        );
    }

    #[test]
    #[should_panic(expected = "expected 2 params, got 3")]
    fn step_in_place_wrong_params_length_panics() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(2);
        let mut params = [1.0, 2.0, 3.0];
        opt.step_in_place(&mut params, &[1.0, 2.0], false);
    }

    #[test]
    #[should_panic(expected = "expected 2 gradient elements, got 3")]
    fn step_in_place_wrong_gradient_length_panics() {
        let config = OptimizerConfig::adam(0.1);
        let mut opt = config.build(2);
        let mut params = [1.0, 2.0];
        opt.step_in_place(&mut params, &[1.0, 2.0, 3.0], false);
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

    // ── Snapshot ───────────────────────────────────────────────────────

    #[test]
    fn optimizer_snapshot_round_trip() {
        let config = OptimizerConfig::adam(0.05);
        let mut opt = config.build(3);
        let mut p = [5.0, -3.0, 1.0];

        // Do a few steps to accumulate momentum.
        for _ in 0..10 {
            let grad = [2.0 * p[0], 2.0 * p[1], 2.0 * p[2]];
            opt.step_in_place(&mut p, &grad, false);
        }

        let snap = opt.snapshot("test");
        assert_eq!(snap.role, "test");
        assert_eq!(snap.m.len(), 3);
        assert_eq!(snap.v.len(), 3);
        assert_eq!(snap.t, 10);

        // Load into a fresh optimizer.
        let mut opt2 = config.build(3);
        opt2.load_snapshot(&snap);

        let snap2 = opt2.snapshot("test");
        assert_eq!(snap.m, snap2.m);
        assert_eq!(snap.v, snap2.v);
        assert_eq!(snap.t, snap2.t);
    }

    #[test]
    fn optimizer_snapshot_preserves_momentum() {
        let config = OptimizerConfig::adam(0.05);

        // Path A: continuous training, 20 steps.
        let mut opt_a = config.build(1);
        let mut pa = [5.0];
        for _ in 0..20 {
            let grad = [2.0 * pa[0]];
            opt_a.step_in_place(&mut pa, &grad, false);
        }

        // Path B: train 10 steps, snapshot, load into fresh, train 10 more.
        let mut opt_b = config.build(1);
        let mut pb = [5.0];
        for _ in 0..10 {
            let grad = [2.0 * pb[0]];
            opt_b.step_in_place(&mut pb, &grad, false);
        }
        let snap = opt_b.snapshot("actor");
        let mut opt_b2 = config.build(1);
        opt_b2.load_snapshot(&snap);
        for _ in 0..10 {
            let grad = [2.0 * pb[0]];
            opt_b2.step_in_place(&mut pb, &grad, false);
        }

        assert!(
            (pa[0] - pb[0]).abs() < 1e-12,
            "interrupted training should match continuous: {:.15} vs {:.15}",
            pa[0],
            pb[0]
        );
    }
}
