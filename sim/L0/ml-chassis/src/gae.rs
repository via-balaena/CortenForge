//! Generalized Advantage Estimation (GAE).
//!
//! Standalone function — decoupled from any trajectory struct or algorithm.
//! Used by PPO (and A2C, PPG, any GAE-based on-policy method).
//!
//! Reference: Schulman et al., "High-Dimensional Continuous Control Using
//! Generalized Advantage Estimation", 2016.

/// Compute GAE advantages and value targets from raw slices.
///
/// # Arguments
///
/// - `rewards` — reward at each step, length `n`.
/// - `values` — `V(s_t)` at each step, length `n`.
/// - `next_value` — bootstrap value. 0 if the episode truly ended (done),
///   V(s') if truncated (timeout). This is the value at the state *after*
///   the last step.
/// - `gamma` — discount factor (typically 0.99).
/// - `lambda` — GAE lambda (typically 0.95). λ=0 gives TD residuals,
///   λ=1 gives discounted returns minus baseline.
///
/// # Returns
///
/// `(advantages, value_targets)` where:
/// - `advantages[t] = Σ_{l=0}^{n-t-1} (γλ)^l δ_{t+l}`
/// - `value_targets[t] = advantages[t] + values[t]`
///
/// # Panics
///
/// Panics if `rewards.len() != values.len()`.
#[must_use]
pub fn compute_gae(
    rewards: &[f64],
    values: &[f64],
    next_value: f64,
    gamma: f64,
    lambda: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = rewards.len();
    assert!(
        values.len() == n,
        "compute_gae: rewards.len() ({n}) != values.len() ({})",
        values.len(),
    );

    if n == 0 {
        return (Vec::new(), Vec::new());
    }

    let mut advantages = vec![0.0; n];
    let mut gae = 0.0;

    // Backward pass: A_t = δ_t + γλ A_{t+1}
    for t in (0..n).rev() {
        let v_next = if t + 1 < n { values[t + 1] } else { next_value };
        let delta = gamma.mul_add(v_next, rewards[t]) - values[t];
        gae = (gamma * lambda).mul_add(gae, delta);
        advantages[t] = gae;
    }

    // Value targets: target_t = A_t + V(s_t)
    let value_targets: Vec<f64> = advantages
        .iter()
        .zip(values)
        .map(|(&a, &v)| a + v)
        .collect();

    (advantages, value_targets)
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::let_underscore_must_use)]
mod tests {
    use super::*;

    const GAMMA: f64 = 0.99;
    const LAMBDA: f64 = 0.95;

    #[test]
    fn gae_single_step() {
        // 1-step trajectory: δ = r + γV' - V
        let rewards = [5.0];
        let values = [2.0];
        let next_value = 3.0; // truncated — bootstrap

        let (adv, targets) = compute_gae(&rewards, &values, next_value, GAMMA, LAMBDA);

        let expected_delta = GAMMA.mul_add(3.0, 5.0) - 2.0; // 5.97
        assert!(
            (adv[0] - expected_delta).abs() < 1e-12,
            "adv={}, expected={expected_delta}",
            adv[0],
        );
        assert!(
            (targets[0] - (expected_delta + 2.0)).abs() < 1e-12,
            "target={}, expected={}",
            targets[0],
            expected_delta + 2.0,
        );
    }

    #[test]
    fn gae_lambda_one_matches_discounted_returns() {
        // At λ=1, advantages should equal discounted returns minus V(s).
        let rewards = [1.0, 2.0, 3.0];
        let values = [-5.0, -5.0, -5.0];
        let next_value = 0.0; // done — no bootstrap

        let (adv, _) = compute_gae(&rewards, &values, next_value, GAMMA, 1.0);

        // Hand-compute discounted returns (done → no bootstrap):
        let r2 = 3.0;
        let r1 = GAMMA.mul_add(r2, 2.0);
        let r0 = GAMMA.mul_add(r1, 1.0);

        assert!(
            (adv[2] - (r2 - (-5.0))).abs() < 1e-8,
            "A[2]={}, expected={}",
            adv[2],
            r2 + 5.0,
        );
        assert!(
            (adv[1] - (r1 - (-5.0))).abs() < 1e-8,
            "A[1]={}, expected={}",
            adv[1],
            r1 + 5.0,
        );
        assert!(
            (adv[0] - (r0 - (-5.0))).abs() < 1e-8,
            "A[0]={}, expected={}",
            adv[0],
            r0 + 5.0,
        );
    }

    #[test]
    fn gae_lambda_zero_is_td_residual() {
        // At λ=0, A_t = δ_t = r_t + γV(s_{t+1}) - V(s_t).
        let rewards = [1.0, 2.0, 3.0];
        let values = [10.0, 20.0, 30.0];
        let next_value = 0.0;

        let (adv, _) = compute_gae(&rewards, &values, next_value, GAMMA, 0.0);

        let delta_0 = GAMMA.mul_add(20.0, 1.0) - 10.0;
        let delta_1 = GAMMA.mul_add(30.0, 2.0) - 20.0;
        let delta_2 = GAMMA.mul_add(0.0, 3.0) - 30.0;

        assert!(
            (adv[0] - delta_0).abs() < 1e-12,
            "A[0]={}, expected={delta_0}",
            adv[0],
        );
        assert!(
            (adv[1] - delta_1).abs() < 1e-12,
            "A[1]={}, expected={delta_1}",
            adv[1],
        );
        assert!(
            (adv[2] - delta_2).abs() < 1e-12,
            "A[2]={}, expected={delta_2}",
            adv[2],
        );
    }

    #[test]
    fn gae_done_zeroes_bootstrap() {
        // next_value = 0 → no bootstrap at terminal.
        let rewards = [1.0];
        let values = [0.0];

        let (adv, _) = compute_gae(&rewards, &values, 0.0, GAMMA, LAMBDA);

        // δ = 1.0 + 0.99 * 0.0 - 0.0 = 1.0
        assert!((adv[0] - 1.0).abs() < 1e-12, "adv={}, expected=1.0", adv[0]);
    }

    #[test]
    fn gae_truncated_bootstraps() {
        // next_value = V(s') → bootstraps the future return.
        let rewards = [1.0];
        let values = [0.0];
        let bootstrap = 100.0;

        let (adv, _) = compute_gae(&rewards, &values, bootstrap, GAMMA, LAMBDA);

        // δ = 1.0 + 0.99 * 100.0 - 0.0 = 100.0
        let expected = GAMMA.mul_add(bootstrap, 1.0);
        assert!(
            (adv[0] - expected).abs() < 1e-12,
            "adv={}, expected={expected}",
            adv[0],
        );
    }

    #[test]
    fn gae_empty_trajectory() {
        let (adv, targets) = compute_gae(&[], &[], 0.0, GAMMA, LAMBDA);
        assert!(adv.is_empty());
        assert!(targets.is_empty());
    }

    #[test]
    fn gae_value_targets_are_advantages_plus_values() {
        let rewards = [1.0, 2.0, 3.0, 4.0, 5.0];
        let values = [0.5, 1.5, 2.5, 3.5, 4.5];
        let next_value = 2.0;

        let (adv, targets) = compute_gae(&rewards, &values, next_value, GAMMA, LAMBDA);

        for t in 0..rewards.len() {
            let expected = adv[t] + values[t];
            assert!(
                (targets[t] - expected).abs() < 1e-12,
                "targets[{t}]={}, expected={expected}",
                targets[t],
            );
        }
    }

    #[test]
    #[should_panic(expected = "rewards.len() (3) != values.len() (2)")]
    fn gae_mismatched_lengths_panics() {
        let _ = compute_gae(&[1.0, 2.0, 3.0], &[1.0, 2.0], 0.0, GAMMA, LAMBDA);
    }
}
