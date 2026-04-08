//! RL-specific layer functions built on [`Tape`].
//!
//! These are convenience functions that construct subgraphs on an existing
//! tape. They are NOT types — just functions that take `&mut Tape` and
//! return `Var`s. Gradients are automatically correct because every
//! function composes from the primitives in [`autograd`](crate::autograd).
//!
//! # Layers
//!
//! - [`linear_tanh`] — `tanh(W · x + b)`, used for policy hidden + output layers.
//! - [`linear_relu`] — `relu(W · x + b)`, alternative for hidden layers in deep nets.
//! - [`linear_raw`] — `W · x + b` (no activation), used for value/Q output layers.
//!
//! # Activation selection
//!
//! [`Activation`] enum (`Tanh` / `Relu`) — stored in autograd network types.
//! Hidden layers dispatch on it; output layers stay fixed (tanh for policy,
//! raw for value/Q).
//!
//! # Loss functions
//!
//! - [`mse_loss`] — `(pred − target)²`, single sample.
//! - [`mse_loss_batch`] — `mean((pred_i − target_i)²)`, batch.
//!
//! # Probability
//!
//! - [`gaussian_log_prob`] — log π(a|s) under diagonal Gaussian, with per-dim
//!   `log_std`. Differentiable w.r.t. `mu` and `log_std`.

use crate::autograd::{Tape, Var};

// ── Linear layers ─────────────────────────────────────────────────────────

/// `z = tanh(W · x + b)`.
///
/// `w` is row-major `[out_dim × in_dim]`, `b` is `[out_dim]`, `x` is
/// `[in_dim]`. Returns `[out_dim]` output variables.
///
/// Used for policy hidden layers and output layers (bounded to `[-1, 1]`).
///
/// # Panics
///
/// Panics if slice lengths don't match dimensions.
#[must_use]
pub fn linear_tanh(
    tape: &mut Tape,
    w: &[Var],
    b: &[Var],
    x: &[Var],
    out_dim: usize,
    in_dim: usize,
) -> Vec<Var> {
    let z = tape.affine(w, x, b, out_dim, in_dim);
    z.into_iter().map(|zi| tape.tanh(zi)).collect()
}

/// `z = relu(W · x + b)`.
///
/// Same as [`linear_tanh`] but with `ReLU` activation. Preferred for hidden
/// layers in deeper networks (2+ layers) — avoids tanh saturation.
///
/// # Panics
///
/// Panics if slice lengths don't match dimensions.
#[must_use]
pub fn linear_relu(
    tape: &mut Tape,
    w: &[Var],
    b: &[Var],
    x: &[Var],
    out_dim: usize,
    in_dim: usize,
) -> Vec<Var> {
    let z = tape.affine(w, x, b, out_dim, in_dim);
    z.into_iter().map(|zi| tape.relu(zi)).collect()
}

/// `z = W · x + b` (no activation).
///
/// Same as [`linear_tanh`] but without the `tanh` — output is unbounded.
/// Used for value function and Q-function output layers.
///
/// # Panics
///
/// Panics if slice lengths don't match dimensions.
#[must_use]
pub fn linear_raw(
    tape: &mut Tape,
    w: &[Var],
    b: &[Var],
    x: &[Var],
    out_dim: usize,
    in_dim: usize,
) -> Vec<Var> {
    tape.affine(w, x, b, out_dim, in_dim)
}

// ── Activation selection ─────────────────────────────────────────────────

/// Hidden-layer activation function.
///
/// Stored in autograd network types. Output layers are unaffected:
/// policy output uses tanh (bounds to [-1, 1]), value/Q output uses
/// raw (unbounded).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Activation {
    /// `tanh` — bounded to [-1, 1]. Default for compatibility with
    /// hand-coded MLP oracle.
    #[default]
    Tanh,
    /// `relu` — `max(0, x)`. Preferred for deeper networks (2+ layers).
    Relu,
}

/// Apply a linear layer with the given hidden activation.
///
/// Dispatches to [`linear_tanh`] or [`linear_relu`] based on `activation`.
/// For output layers, use [`linear_tanh`] (policy) or [`linear_raw`]
/// (value/Q) directly.
#[must_use]
pub fn linear_hidden(
    tape: &mut Tape,
    w: &[Var],
    b: &[Var],
    x: &[Var],
    out_dim: usize,
    in_dim: usize,
    activation: Activation,
) -> Vec<Var> {
    match activation {
        Activation::Tanh => linear_tanh(tape, w, b, x, out_dim, in_dim),
        Activation::Relu => linear_relu(tape, w, b, x, out_dim, in_dim),
    }
}

// ── Loss functions ────────────────────────────────────────────────────────

/// MSE loss: `(pred − target)²`.
///
/// Returns a single scalar `Var`.
#[must_use]
pub fn mse_loss(tape: &mut Tape, pred: Var, target: Var) -> Var {
    let diff = tape.sub(pred, target);
    tape.square(diff)
}

/// Batched MSE loss: `mean_i((pred_i − target_i)²)`.
///
/// `preds` and `targets` must have the same length.
///
/// # Panics
///
/// Panics if `preds.len() != targets.len()` or if either is empty.
#[must_use]
pub fn mse_loss_batch(tape: &mut Tape, preds: &[Var], targets: &[Var]) -> Var {
    assert!(
        preds.len() == targets.len(),
        "mse_loss_batch: preds.len() ({}) != targets.len() ({})",
        preds.len(),
        targets.len(),
    );
    assert!(!preds.is_empty(), "mse_loss_batch: empty input");
    let losses: Vec<Var> = preds
        .iter()
        .zip(targets)
        .map(|(&pred, &target)| mse_loss(tape, pred, target))
        .collect();
    tape.mean(&losses)
}

// ── Gaussian log-probability ──────────────────────────────────────────────

/// Log-probability under a diagonal Gaussian.
///
/// ```text
/// log π(a|s) = Σ_i [ -0.5 * (a_i − μ_i)² / σ_i² − log(σ_i) − 0.5 * log(2π) ]
/// ```
///
/// `mu`, `log_std`, and `action` must all have the same length.
///
/// - `mu`: policy mean, typically from a forward pass (differentiable).
/// - `log_std`: log standard deviation. For [`StochasticPolicy`](crate::StochasticPolicy),
///   these are learnable parameters. For fixed-sigma policies, create them
///   with [`Tape::constant`].
/// - `action`: observed actions. Typically constants (not differentiated).
///
/// Returns a single scalar `Var` — the total log-probability.
///
/// # Panics
///
/// Panics if the three slices have different lengths or are empty.
#[must_use]
pub fn gaussian_log_prob(tape: &mut Tape, mu: &[Var], log_std: &[Var], action: &[Var]) -> Var {
    let dim = mu.len();
    assert!(dim > 0, "gaussian_log_prob: empty input");
    assert!(
        log_std.len() == dim,
        "gaussian_log_prob: log_std.len() ({}) != mu.len() ({dim})",
        log_std.len(),
    );
    assert!(
        action.len() == dim,
        "gaussian_log_prob: action.len() ({}) != mu.len() ({dim})",
        action.len(),
    );

    let log_2pi = tape.constant(std::f64::consts::TAU.ln()); // ln(2π)
    let neg_half = tape.constant(-0.5);

    let mut terms = Vec::with_capacity(dim);
    for i in 0..dim {
        // diff = action_i - mu_i
        let diff = tape.sub(action[i], mu[i]);
        // diff² = (action_i - mu_i)²
        let diff_sq = tape.square(diff);
        // 1/var_i = exp(-2 * log_std_i) — avoids computing var then inverting
        let neg_two = tape.constant(-2.0);
        let neg2_log_std = tape.mul(neg_two, log_std[i]);
        let inv_var = tape.exp(neg2_log_std);
        // diff² / var_i
        let scaled = tape.mul(diff_sq, inv_var);
        // -0.5 * diff² / var_i
        let term_a = tape.mul(neg_half, scaled);
        // -log_std_i
        let term_b = tape.neg(log_std[i]);
        // -0.5 * log(2π)
        let term_c = tape.mul(neg_half, log_2pi);
        // sum the three terms for this dimension
        let ab = tape.add(term_a, term_b);
        terms.push(tape.add(ab, term_c));
    }

    tape.sum(&terms)
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;

    const FD_EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    // ── Multi-param FD helper (same pattern as autograd.rs) ───────

    fn assert_grads_fd(params: &[f64], build_graph: impl Fn(&mut Tape, &[Var]) -> Var) {
        let mut tape = Tape::new();
        let pvars: Vec<Var> = params.iter().map(|&val| tape.param(val)).collect();
        let out = build_graph(&mut tape, &pvars);
        tape.backward(out);
        let analytical: Vec<f64> = pvars.iter().map(|&var| tape.grad(var)).collect();

        for (i, &analytic_i) in analytical.iter().enumerate() {
            let eval_at = |delta: f64| {
                let mut fd_tape = Tape::new();
                let vs: Vec<Var> = params
                    .iter()
                    .enumerate()
                    .map(|(j, &val)| fd_tape.param(if j == i { val + delta } else { val }))
                    .collect();
                let fd_out = build_graph(&mut fd_tape, &vs);
                fd_tape.value(fd_out)
            };
            let numerical = (eval_at(FD_EPS) - eval_at(-FD_EPS)) / (2.0 * FD_EPS);
            let err = (analytic_i - numerical).abs();
            assert!(
                err < FD_TOL,
                "FD mismatch param[{i}]: analytical={analytic_i}, numerical={numerical}, err={err}",
            );
        }
    }

    // ── linear_tanh tests ─────────────────────────────────────────

    #[test]
    fn linear_tanh_gradient_matches_fd() {
        // 2×3 linear_tanh: W[2×3] + b[2] + x[3] = 11 params
        let params = vec![
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, // W: 2×3
            0.1, -0.05, // b: 2
            1.0, -0.5, 2.0, // x: 3
        ];
        assert_grads_fd(&params, |tape, p| {
            let w = &p[0..6];
            let b = &p[6..8];
            let x = &p[8..11];
            let out = linear_tanh(tape, w, b, x, 2, 3);
            tape.sum(&out)
        });
    }

    #[test]
    fn linear_tanh_output_in_range() {
        let mut tape = Tape::new();
        let w: Vec<Var> = [5.0, -5.0, 5.0, -5.0]
            .iter()
            .map(|&val| tape.param(val))
            .collect();
        let b: Vec<Var> = [0.0, 0.0].iter().map(|&val| tape.param(val)).collect();
        let x: Vec<Var> = [1.0, 1.0].iter().map(|&val| tape.constant(val)).collect();
        let out = linear_tanh(&mut tape, &w, &b, &x, 2, 2);
        for &var in &out {
            let val = tape.value(var);
            assert!(
                (-1.0..=1.0).contains(&val),
                "linear_tanh output {val} outside [-1, 1]"
            );
        }
    }

    // ── linear_relu tests ─────────────────────────────────────────

    #[test]
    fn linear_relu_gradient_matches_fd() {
        let params = vec![
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, // W: 2×3
            0.1, -0.05, // b: 2
            1.0, -0.5, 2.0, // x: 3
        ];
        assert_grads_fd(&params, |tape, p| {
            let w = &p[0..6];
            let b = &p[6..8];
            let x = &p[8..11];
            let out = linear_relu(tape, w, b, x, 2, 3);
            tape.sum(&out)
        });
    }

    // ── linear_raw tests ──────────────────────────────────────────

    #[test]
    fn linear_raw_gradient_matches_fd() {
        let params = vec![
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, // W: 2×3
            0.1, -0.05, // b: 2
            1.0, -0.5, 2.0, // x: 3
        ];
        assert_grads_fd(&params, |tape, p| {
            let w = &p[0..6];
            let b = &p[6..8];
            let x = &p[8..11];
            let out = linear_raw(tape, w, b, x, 2, 3);
            tape.sum(&out)
        });
    }

    #[test]
    fn linear_raw_output_unbounded() {
        let mut tape = Tape::new();
        let w: Vec<Var> = [10.0, 10.0].iter().map(|&val| tape.param(val)).collect();
        let b: Vec<Var> = vec![tape.param(100.0)];
        let x: Vec<Var> = [5.0, 5.0].iter().map(|&val| tape.constant(val)).collect();
        let out = linear_raw(&mut tape, &w, &b, &x, 1, 2);
        // W·x + b = 10*5 + 10*5 + 100 = 200 — no tanh clamping
        assert!((tape.value(out[0]) - 200.0).abs() < 1e-12);
    }

    // ── mse_loss tests ────────────────────────────────────────────

    #[test]
    fn mse_loss_gradient_matches_fd() {
        assert_grads_fd(&[2.5, 1.0], |tape, p| mse_loss(tape, p[0], p[1]));
    }

    #[test]
    fn mse_loss_value_correct() {
        let mut tape = Tape::new();
        let pred = tape.param(3.0);
        let target = tape.constant(1.0);
        let loss = mse_loss(&mut tape, pred, target);
        assert!((tape.value(loss) - 4.0).abs() < 1e-12); // (3-1)² = 4
    }

    #[test]
    fn mse_loss_batch_gradient_matches_fd() {
        // 3 pred-target pairs
        let params = vec![2.0, 3.0, 1.0, 1.5, 2.5, 0.5];
        assert_grads_fd(&params, |tape, p| {
            let preds = &p[0..3];
            let targets = &p[3..6];
            mse_loss_batch(tape, preds, targets)
        });
    }

    // ── gaussian_log_prob tests ───────────────────────────────────

    #[test]
    fn gaussian_log_prob_gradient_matches_fd() {
        // Params: mu[2] + log_std[2] + action[2] = 6
        let params = vec![0.5, -0.3, -1.0, -0.5, 0.7, -0.1];
        assert_grads_fd(&params, |tape, p| {
            let mu = &p[0..2];
            let log_std = &p[2..4];
            let action = &p[4..6];
            gaussian_log_prob(tape, mu, log_std, action)
        });
    }

    #[test]
    fn gaussian_log_prob_matches_hand_coded() {
        // Compare against the existing hand-coded gaussian_log_prob in sac.rs.
        let mu: [f64; 2] = [0.5, -0.3];
        let log_std: [f64; 2] = [-1.0, -0.5];
        let action: [f64; 2] = [0.7, -0.1];

        // Hand-coded (reference from sac.rs)
        let mut expected = 0.0_f64;
        for i in 0..2 {
            let std_i = log_std[i].exp();
            let var = std_i * std_i;
            let diff = action[i] - mu[i];
            expected += 0.5_f64.mul_add(
                -(2.0 * std::f64::consts::PI).ln(),
                -0.5 * diff * diff / var - log_std[i],
            );
        }

        // Autograd
        let mut tape = Tape::new();
        let mu_vars: Vec<Var> = mu.iter().map(|&val| tape.param(val)).collect();
        let log_std_vars: Vec<Var> = log_std.iter().map(|&val| tape.param(val)).collect();
        let action_vars: Vec<Var> = action.iter().map(|&val| tape.constant(val)).collect();
        let lp = gaussian_log_prob(&mut tape, &mu_vars, &log_std_vars, &action_vars);

        let actual = tape.value(lp);
        let err = (actual - expected).abs();
        assert!(
            err < 1e-12,
            "gaussian_log_prob value mismatch: expected={expected}, actual={actual}, err={err}",
        );
    }

    // ── Forward parity with MlpPolicy ─────────────────────────────

    #[test]
    fn linear_tanh_matches_mlp_forward() {
        // Verify that linear_tanh produces the same output as MlpPolicy
        // for a single hidden layer forward pass.
        use crate::mlp::MlpPolicy;
        use crate::policy::Policy;

        let obs_dim = 4;
        let hidden_dim = 3;
        let act_dim = 2;
        let obs_scale = [0.5, 1.0, 0.1, 2.0];
        let obs = [1.0_f32, -0.5, 3.0, 0.2];

        let mut mlp = MlpPolicy::new(obs_dim, hidden_dim, act_dim, &obs_scale);
        // Set non-trivial params: W1[3×4]=12 + b1[3] + W2[2×3]=6 + b2[2] = 23
        let params = vec![
            0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, // W1
            0.05, -0.1, 0.15, // b1
            0.4, -0.6, 0.2, -0.3, 0.5, -0.1, // W2
            0.08, -0.05, // b2
        ];
        mlp.set_params(&params);
        let mlp_out = mlp.forward(&obs);

        // Reproduce with autograd layers: two linear layers.
        let mut tape = Tape::new();

        // Load params as Vars.
        let param_vars: Vec<Var> = params.iter().map(|&val| tape.param(val)).collect();
        // Scaled observations as constants.
        let x: Vec<Var> = obs
            .iter()
            .zip(&obs_scale)
            .map(|(&ob, &sc)| tape.constant(f64::from(ob) * sc))
            .collect();

        // Layer 1: hidden = tanh(W1 · x + b1)
        let w1 = &param_vars[0..12];
        let b1 = &param_vars[12..15];
        let hidden = linear_tanh(&mut tape, w1, b1, &x, hidden_dim, obs_dim);

        // Layer 2: output = tanh(W2 · hidden + b2)
        let w2 = &param_vars[15..21];
        let b2 = &param_vars[21..23];
        let output = linear_tanh(&mut tape, w2, b2, &hidden, act_dim, hidden_dim);

        for (i, &var) in output.iter().enumerate() {
            let ag_val = tape.value(var);
            let err = (ag_val - mlp_out[i]).abs();
            assert!(
                err < 1e-12,
                "forward parity mismatch output[{i}]: mlp={}, autograd={ag_val}, err={err}",
                mlp_out[i],
            );
        }
    }
}
