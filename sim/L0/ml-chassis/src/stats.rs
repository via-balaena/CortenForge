//! Statistics helpers used across policies and algorithms.
//!
//! This module exists to host pure math helpers that belong to neither a
//! specific algorithm nor the autograd tape. They are called by
//! hand-coded code paths (e.g., `sim-rl`'s SAC implementation) and by
//! chassis-level consumers that build RL loops from primitives (e.g.,
//! the `vec-env/sac` visual example). The autograd-tape version of
//! `gaussian_log_prob` lives in [`crate::autograd_layers`] alongside the
//! other `Var`-based ops.

/// Log probability of `action` under a diagonal Gaussian with mean `mu`
/// and per-dimension `log_std`.
///
/// Computes `log N(action | mu, diag(exp(log_std)^2))`.
///
/// # Panics
///
/// Does not panic on its own, but indexes `mu[i]` and `log_std[i]` for
/// `i ∈ [0, action.len())`, so the three slices must be the same length
/// — caller responsibility.
#[must_use]
pub fn gaussian_log_prob(action: &[f64], mu: &[f64], log_std: &[f64]) -> f64 {
    let mut lp = 0.0;
    for i in 0..action.len() {
        let std = log_std[i].exp();
        let var = std * std;
        let diff = action[i] - mu[i];
        lp += 0.5f64.mul_add(
            -(2.0 * std::f64::consts::PI).ln(),
            -0.5 * diff * diff / var - log_std[i],
        );
    }
    lp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_difference_matches_closed_form() {
        // At action == mu with log_std = 0 (i.e., std=1), log N(0|0,1) per
        // dim is -0.5 * ln(2π).
        let lp = gaussian_log_prob(&[0.0], &[0.0], &[0.0]);
        let expected = -0.5 * (2.0 * std::f64::consts::PI).ln();
        assert!((lp - expected).abs() < 1e-12);
    }

    #[test]
    fn diagonal_independence_sums_per_dim() {
        // Three independent dims should sum.
        let lp = gaussian_log_prob(&[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0]);
        let per_dim = -0.5 * (2.0 * std::f64::consts::PI).ln();
        assert!(3.0f64.mul_add(-per_dim, lp).abs() < 1e-12);
    }
}
