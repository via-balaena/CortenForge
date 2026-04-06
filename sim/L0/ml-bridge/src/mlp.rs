//! MLP policy, value, and Q-function implementations.
//!
//! Single hidden layer with tanh activation — the "bolt-on turbo" upgrade
//! from [`LinearPolicy`](crate::LinearPolicy). Same traits, enough capacity
//! for 6-DOF tasks (614 actor params). Hand-coded backprop; the algorithm
//! never knows the difference from an autograd backend.
//!
//! # Architecture
//!
//! ```text
//! z1 = W1 · s_scaled + b1     // [hidden_dim]
//! h  = tanh(z1)                // [hidden_dim]
//! output = W2 · h + b2         // policy: tanh(output), value/Q: linear
//! ```
//!
//! # Parameter layouts
//!
//! All parameters are stored as a contiguous `Vec<f64>`:
//!
//! - [`MlpPolicy`]: `W1[H×O]` + `b1[H]` + `W2[A×H]` + `b2[A]`.
//! - [`MlpValue`]: `W1[H×O]` + `b1[H]` + `w2[H]` + `b2` (scalar).
//! - [`MlpQ`]: `W1[H×(O+A)]` + `b1[H]` + `w2[H]` + `b2` (scalar).

use crate::policy::{DifferentiablePolicy, Policy};
use crate::value::{QFunction, ValueFn};

// ── Shared helpers ────────────────────────────────────────────────────────

/// Compute scaled observation.
fn scaled_obs(obs: &[f32], obs_scale: &[f64]) -> Vec<f64> {
    obs.iter()
        .zip(obs_scale)
        .map(|(&o, &s)| f64::from(o) * s)
        .collect()
}

/// Hidden layer forward: h = tanh(W1 · input + b1).
///
/// `w1` is row-major `[hidden_dim × input_dim]`, `b1` is `[hidden_dim]`.
/// Returns `(h, z1)` — both needed for backprop.
fn hidden_forward(
    w1: &[f64],
    b1: &[f64],
    input: &[f64],
    hidden_dim: usize,
) -> (Vec<f64>, Vec<f64>) {
    let input_dim = input.len();
    let mut z1 = Vec::with_capacity(hidden_dim);
    let mut h = Vec::with_capacity(hidden_dim);
    for hi in 0..hidden_dim {
        let mut z = b1[hi];
        for (j, &inp) in input.iter().enumerate() {
            z = w1[hi * input_dim + j].mul_add(inp, z);
        }
        z1.push(z);
        h.push(z.tanh());
    }
    (h, z1)
}

// ── MlpPolicy ─────────────────────────────────────────────────────────────

/// MLP policy: `μ(s) = tanh(W2 · tanh(W1 · s_scaled + b1) + b2)`.
///
/// Implements [`Policy`] + [`DifferentiablePolicy`]. Single hidden layer
/// with tanh activation. Enough capacity for 6-DOF reaching (614 params
/// with `hidden_dim=32`).
pub struct MlpPolicy {
    obs_dim: usize,
    hidden_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    // Offsets into the flat param vector.
    w1_end: usize, // W1: [0, w1_end)
    b1_end: usize, // b1: [w1_end, b1_end)
    w2_end: usize, // W2: [b1_end, w2_end)
                   // b2: [w2_end, n_params)
}

impl MlpPolicy {
    /// Create a new MLP policy with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dim: usize, act_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "MlpPolicy::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        let w1_end = hidden_dim * obs_dim;
        let b1_end = w1_end + hidden_dim;
        let w2_end = b1_end + act_dim * hidden_dim;
        let n_params = w2_end + act_dim;
        Self {
            obs_dim,
            hidden_dim,
            act_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            w1_end,
            b1_end,
            w2_end,
        }
    }

    /// Run forward pass, returning `(mu, h)` — both needed for gradient methods.
    fn forward_with_hidden(&self, obs: &[f32]) -> (Vec<f64>, Vec<f64>) {
        let s = scaled_obs(obs, &self.obs_scale);
        let w1 = &self.params[..self.w1_end];
        let b1 = &self.params[self.w1_end..self.b1_end];
        let w2 = &self.params[self.b1_end..self.w2_end];
        let b2 = &self.params[self.w2_end..];

        let (h, _z1) = hidden_forward(w1, b1, &s, self.hidden_dim);

        let mut mu = Vec::with_capacity(self.act_dim);
        for a in 0..self.act_dim {
            let mut z = b2[a];
            for (hi, &hv) in h.iter().enumerate() {
                z = w2[a * self.hidden_dim + hi].mul_add(hv, z);
            }
            mu.push(z.tanh());
        }
        (mu, h)
    }

    /// Backprop from output-layer derivatives `d_out[A]` through the network.
    ///
    /// `d_out[a]` is the derivative of the loss w.r.t. the output-layer
    /// pre-tanh activation (for `log_prob_gradient`, this is `score`;
    /// for `forward_vjp`, this is `v[a] * (1 - mu[a]²)`).
    fn backprop(&self, obs: &[f32], h: &[f64], d_out: &[f64]) -> Vec<f64> {
        let s = scaled_obs(obs, &self.obs_scale);
        let w2 = &self.params[self.b1_end..self.w2_end];

        let mut grad = vec![0.0; self.params.len()];

        // ── Layer 2 gradients ─────────────────────────────────────────
        for a in 0..self.act_dim {
            for (hi, &hv) in h.iter().enumerate() {
                grad[self.b1_end + a * self.hidden_dim + hi] = d_out[a] * hv; // dW2
            }
            grad[self.w2_end + a] = d_out[a]; // db2
        }

        // ── Backprop through hidden layer ─────────────────────────────
        for hi in 0..self.hidden_dim {
            // d_h[hi] = Σ_a d_out[a] * W2[a, hi]
            let mut d_h = 0.0;
            for a in 0..self.act_dim {
                d_h += d_out[a] * w2[a * self.hidden_dim + hi];
            }
            // d_z1[hi] = d_h * (1 - h[hi]²)
            let d_z1 = d_h * h[hi].mul_add(-h[hi], 1.0);

            // dW1[hi, o] = d_z1 * s_scaled[o]
            for (o, &so) in s.iter().enumerate() {
                grad[hi * self.obs_dim + o] = d_z1 * so;
            }
            // db1[hi] = d_z1
            grad[self.w1_end + hi] = d_z1;
        }

        grad
    }
}

impl Policy for MlpPolicy {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "MlpPolicy::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32]) -> Vec<f64> {
        self.forward_with_hidden(obs).0
    }
}

impl DifferentiablePolicy for MlpPolicy {
    fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64> {
        let (mu, h) = self.forward_with_hidden(obs);
        let sigma2 = sigma * sigma;

        // score[a] = (action[a] - mu[a]) / σ² * (1 - mu[a]²)
        let d_out: Vec<f64> = (0..self.act_dim)
            .map(|a| (action[a] - mu[a]) / sigma2 * mu[a].mul_add(-mu[a], 1.0))
            .collect();

        self.backprop(obs, &h, &d_out)
    }

    fn forward_vjp(&self, obs: &[f32], v: &[f64]) -> Vec<f64> {
        let (mu, h) = self.forward_with_hidden(obs);

        // d_mu[a] = v[a] * (1 - mu[a]²)
        let d_out: Vec<f64> = (0..self.act_dim)
            .map(|a| v[a] * mu[a].mul_add(-mu[a], 1.0))
            .collect();

        self.backprop(obs, &h, &d_out)
    }
}

// ── MlpValue ──────────────────────────────────────────────────────────────

/// MLP value function: `V(s) = w2 · tanh(W1 · s_scaled + b1) + b2`.
///
/// Implements [`ValueFn`]. Linear output layer (no tanh) — value predictions
/// are unbounded.
pub struct MlpValue {
    obs_dim: usize,
    hidden_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    w1_end: usize,
    b1_end: usize,
    w2_end: usize,
    // b2: params[w2_end] (scalar)
}

impl MlpValue {
    /// Create a new MLP value function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "MlpValue::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        let w1_end = hidden_dim * obs_dim;
        let b1_end = w1_end + hidden_dim;
        let w2_end = b1_end + hidden_dim;
        let n_params = w2_end + 1; // +1 for scalar b2
        Self {
            obs_dim,
            hidden_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            w1_end,
            b1_end,
            w2_end,
        }
    }

    /// Forward pass returning `(V, h)`.
    fn forward_with_hidden(&self, obs: &[f32]) -> (f64, Vec<f64>) {
        let s = scaled_obs(obs, &self.obs_scale);
        let w1 = &self.params[..self.w1_end];
        let b1 = &self.params[self.w1_end..self.b1_end];
        let w2 = &self.params[self.b1_end..self.w2_end];
        let b2 = self.params[self.w2_end];

        let (h, _z1) = hidden_forward(w1, b1, &s, self.hidden_dim);

        let mut v = b2;
        for (hi, &hv) in h.iter().enumerate() {
            v = w2[hi].mul_add(hv, v);
        }
        (v, h)
    }
}

impl ValueFn for MlpValue {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "MlpValue::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32]) -> f64 {
        self.forward_with_hidden(obs).0
    }

    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64> {
        let s = scaled_obs(obs, &self.obs_scale);
        let (v, h) = self.forward_with_hidden(obs);
        let dl_dv = 2.0 * (v - target);
        let w2 = &self.params[self.b1_end..self.w2_end];

        let mut grad = vec![0.0; self.params.len()];

        // ── Layer 2 gradients ─────────────────────────────────────────
        for (hi, &hv) in h.iter().enumerate() {
            grad[self.b1_end + hi] = dl_dv * hv; // dw2[h]
        }
        grad[self.w2_end] = dl_dv; // db2

        // ── Backprop through hidden layer ─────────────────────────────
        for hi in 0..self.hidden_dim {
            let d_z1 = dl_dv * w2[hi] * h[hi].mul_add(-h[hi], 1.0);
            for (o, &so) in s.iter().enumerate() {
                grad[hi * self.obs_dim + o] = d_z1 * so; // dW1[h, o]
            }
            grad[self.w1_end + hi] = d_z1; // db1[h]
        }

        grad
    }
}

// ── MlpQ ──────────────────────────────────────────────────────────────────

/// MLP Q-function: `Q(s, a) = w2 · tanh(W1 · [s_scaled; a] + b1) + b2`.
///
/// Implements [`QFunction`]. Linear output layer. Input is the concatenation
/// of scaled observations and raw actions.
pub struct MlpQ {
    obs_dim: usize,
    act_dim: usize,
    hidden_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    w1_end: usize,
    b1_end: usize,
    w2_end: usize,
}

impl MlpQ {
    /// Create a new MLP Q-function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim`.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dim: usize, act_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "MlpQ::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        let input_dim = obs_dim + act_dim;
        let w1_end = hidden_dim * input_dim;
        let b1_end = w1_end + hidden_dim;
        let w2_end = b1_end + hidden_dim;
        let n_params = w2_end + 1;
        Self {
            obs_dim,
            act_dim,
            hidden_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            w1_end,
            b1_end,
            w2_end,
        }
    }

    /// Build the concatenated input vector `[s_scaled; action]`.
    fn input_vec(&self, obs: &[f32], action: &[f64]) -> Vec<f64> {
        let mut input = scaled_obs(obs, &self.obs_scale);
        input.extend_from_slice(action);
        input
    }

    /// Forward pass returning `(Q, h)`.
    fn forward_with_hidden(&self, obs: &[f32], action: &[f64]) -> (f64, Vec<f64>) {
        let input = self.input_vec(obs, action);
        let input_dim = self.obs_dim + self.act_dim;
        let w1 = &self.params[..self.w1_end];
        let b1 = &self.params[self.w1_end..self.b1_end];
        let w2 = &self.params[self.b1_end..self.w2_end];
        let b2 = self.params[self.w2_end];

        let (h, _z1) = hidden_forward(w1, b1, &input, self.hidden_dim);
        // Suppress unused variable warning — input_dim is used only to
        // document the W1 shape for the reader.
        let _ = input_dim;

        let mut q = b2;
        for (hi, &hv) in h.iter().enumerate() {
            q = w2[hi].mul_add(hv, q);
        }
        (q, h)
    }
}

impl QFunction for MlpQ {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "MlpQ::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32], action: &[f64]) -> f64 {
        self.forward_with_hidden(obs, action).0
    }

    fn mse_gradient(&self, obs: &[f32], action: &[f64], target: f64) -> Vec<f64> {
        let input = self.input_vec(obs, action);
        let input_dim = self.obs_dim + self.act_dim;
        let (q, h) = self.forward_with_hidden(obs, action);
        let dl_dq = 2.0 * (q - target);
        let w2 = &self.params[self.b1_end..self.w2_end];

        let mut grad = vec![0.0; self.params.len()];

        // ── Layer 2 gradients ─────────────────────────────────────────
        for (hi, &hv) in h.iter().enumerate() {
            grad[self.b1_end + hi] = dl_dq * hv; // dw2[h]
        }
        grad[self.w2_end] = dl_dq; // db2

        // ── Backprop through hidden layer ─────────────────────────────
        for hi in 0..self.hidden_dim {
            let d_z1 = dl_dq * w2[hi] * h[hi].mul_add(-h[hi], 1.0);
            for (j, &inp) in input.iter().enumerate() {
                grad[hi * input_dim + j] = d_z1 * inp; // dW1[h, j]
            }
            grad[self.w1_end + hi] = d_z1; // db1[h]
        }

        grad
    }

    fn action_gradient(&self, obs: &[f32], action: &[f64]) -> Vec<f64> {
        let (_, h) = self.forward_with_hidden(obs, action);
        let input_dim = self.obs_dim + self.act_dim;
        let w1 = &self.params[..self.w1_end];
        let w2 = &self.params[self.b1_end..self.w2_end];

        // dQ/da[j] = Σ_h w2[h] * (1 - h[h]²) * W1[h, obs_dim + j]
        let mut dq_da = vec![0.0; self.act_dim];
        for hi in 0..self.hidden_dim {
            let coeff = w2[hi] * h[hi].mul_add(-h[hi], 1.0);
            for j in 0..self.act_dim {
                dq_da[j] += coeff * w1[hi * input_dim + self.obs_dim + j];
            }
        }
        dq_da
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    const OBS_DIM: usize = 4;
    const ACT_DIM: usize = 2;
    const HIDDEN: usize = 3;
    const OBS_SCALE: [f64; OBS_DIM] = [0.5, 1.0, 0.1, 2.0];
    const FD_EPS: f64 = 1e-6;
    const FD_TOL: f64 = 1e-5;

    fn test_obs() -> Vec<f32> {
        vec![1.0_f32, -0.5, 3.0, 0.2]
    }

    fn test_action() -> Vec<f64> {
        vec![0.3, -0.7]
    }

    /// Non-trivial params for `MlpPolicy`: W1[3×4] + b1[3] + W2[2×3] + b2[2] = 23.
    fn test_params_policy() -> Vec<f64> {
        vec![
            // W1[3×4] = 12
            0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, // b1[3] = 3
            0.05, -0.1, 0.15, // W2[2×3] = 6
            0.4, -0.6, 0.2, -0.3, 0.5, -0.1, // b2[2] = 2
            0.08, -0.05,
        ]
    }

    /// Non-trivial params for `MlpValue`: W1[3×4] + b1[3] + w2[3] + b2 = 19.
    fn test_params_value() -> Vec<f64> {
        vec![
            // W1[3×4] = 12
            0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, // b1[3] = 3
            0.05, -0.1, 0.15, // w2[3] = 3
            0.4, -0.6, 0.2, // b2 = 1
            0.08,
        ]
    }

    /// Non-trivial params for `MlpQ`: W1[3×6] + b1[3] + w2[3] + b2 = 25.
    fn test_params_q() -> Vec<f64> {
        vec![
            // W1[3×6] = 18
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, 0.1, 0.6, -0.4, 0.7, -0.3, 0.5, -0.3, 0.2, 0.5, -0.1,
            0.6, -0.4, // b1[3] = 3
            0.05, -0.1, 0.15, // w2[3] = 3
            0.4, -0.6, 0.2, // b2 = 1
            0.08,
        ]
    }

    // ── MlpPolicy smoke tests ─────────────────────────────────────────

    #[test]
    fn mlp_policy_n_params() {
        let p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        // W1[3×4] + b1[3] + W2[2×3] + b2[2] = 12 + 3 + 6 + 2 = 23
        assert_eq!(p.n_params(), 23);
    }

    #[test]
    fn mlp_policy_params_roundtrip() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let vals = test_params_policy();
        p.set_params(&vals);
        assert_eq!(p.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 23 params, got 3")]
    fn mlp_policy_set_params_wrong_length() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        p.set_params(&[1.0, 2.0, 3.0]);
    }

    #[test]
    fn mlp_policy_forward_in_tanh_range() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());
        let mu = p.forward(&test_obs());
        assert_eq!(mu.len(), ACT_DIM);
        for &m in &mu {
            assert!((-1.0..=1.0).contains(&m), "tanh output {m} out of [-1,1]");
        }
    }

    // ── MlpPolicy FD: log_prob_gradient ───────────────────────────────

    fn log_prob(policy: &MlpPolicy, obs: &[f32], action: &[f64], sigma: f64) -> f64 {
        let mu = policy.forward(obs);
        let sigma2 = sigma * sigma;
        let mut lp = 0.0;
        for (a, m) in action.iter().zip(&mu) {
            lp -= (a - m) * (a - m) / (2.0 * sigma2);
        }
        lp
    }

    #[test]
    fn mlp_policy_log_prob_gradient_matches_fd() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
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
        p.set_params(&base_params);
    }

    // ── MlpPolicy FD: forward_vjp ────────────────────────────────────

    #[test]
    fn mlp_policy_forward_vjp_matches_fd() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());
        let obs = test_obs();
        let v = vec![0.7, -0.3];

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

    // ── MlpValue smoke tests ──────────────────────────────────────────

    #[test]
    fn mlp_value_n_params() {
        let v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        // W1[3×4] + b1[3] + w2[3] + b2 = 12 + 3 + 3 + 1 = 19
        assert_eq!(v.n_params(), 19);
    }

    #[test]
    fn mlp_value_params_roundtrip() {
        let mut v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        let vals = test_params_value();
        v.set_params(&vals);
        assert_eq!(v.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 19 params, got 2")]
    fn mlp_value_set_params_wrong_length() {
        let mut v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        v.set_params(&[1.0, 2.0]);
    }

    // ── MlpValue FD: mse_gradient ─────────────────────────────────────

    #[test]
    fn mlp_value_mse_gradient_matches_fd() {
        let mut vf = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        vf.set_params(&test_params_value());
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

    // ── MlpQ smoke tests ──────────────────────────────────────────────

    #[test]
    fn mlp_q_n_params() {
        let q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        // W1[3×6] + b1[3] + w2[3] + b2 = 18 + 3 + 3 + 1 = 25
        assert_eq!(q.n_params(), 25);
    }

    #[test]
    fn mlp_q_params_roundtrip() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let vals = test_params_q();
        q.set_params(&vals);
        assert_eq!(q.params(), &vals[..]);
    }

    #[test]
    #[should_panic(expected = "expected 25 params, got 2")]
    fn mlp_q_set_params_wrong_length() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&[1.0, 2.0]);
    }

    // ── MlpQ FD: mse_gradient ─────────────────────────────────────────

    #[test]
    fn mlp_q_mse_gradient_matches_fd() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&test_params_q());
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

    // ── MlpQ FD: action_gradient ──────────────────────────────────────

    #[test]
    fn mlp_q_action_gradient_matches_fd() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&test_params_q());
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
    fn mlp_policy_forward_batch_matches_loop() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
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
    fn mlp_policy_forward_vjp_batch_matches_loop() {
        let mut p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        p.set_params(&test_params_policy());

        let obs_batch = batch_obs();
        let v_batch = vec![0.5, -0.3, 0.2, 0.8];

        let batch_result = p.forward_vjp_batch(&obs_batch, &v_batch, OBS_DIM);

        let vjp0 = p.forward_vjp(&test_obs(), &v_batch[..ACT_DIM]);
        let vjp1 = p.forward_vjp(&test_obs2(), &v_batch[ACT_DIM..]);

        assert_eq!(batch_result.len(), p.n_params());
        for i in 0..p.n_params() {
            let expected = f64::midpoint(vjp0[i], vjp1[i]);
            assert!(
                (batch_result[i] - expected).abs() < 1e-12,
                "mlp_vjp_batch[{i}]: batch={}, manual={}",
                batch_result[i],
                expected,
            );
        }
    }

    #[test]
    fn mlp_value_forward_batch_matches_loop() {
        let mut v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        v.set_params(&test_params_value());

        let obs_batch = batch_obs();
        let batch_result = v.forward_batch(&obs_batch, OBS_DIM);

        let single0 = v.forward(&test_obs());
        let single1 = v.forward(&test_obs2());

        assert_eq!(batch_result.len(), 2);
        assert!((batch_result[0] - single0).abs() < 1e-12);
        assert!((batch_result[1] - single1).abs() < 1e-12);
    }

    #[test]
    fn mlp_value_mse_gradient_batch_matches_loop() {
        let mut v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        v.set_params(&test_params_value());

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
                "mlp_v_mse_batch[{i}]: batch={}, manual={}",
                batch_grad[i],
                expected,
            );
        }
    }

    #[test]
    fn mlp_q_forward_batch_matches_loop() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&test_params_q());

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
    fn mlp_q_mse_gradient_batch_matches_loop() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&test_params_q());

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
                "mlp_q_mse_batch[{i}]: batch={}, manual={}",
                batch_grad[i],
                expected,
            );
        }
    }

    #[test]
    fn mlp_q_action_gradient_batch_matches_loop() {
        let mut q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        q.set_params(&test_params_q());

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
    fn mlp_batch_empty_returns_empty_or_zeros() {
        let p = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        assert!(p.forward_batch(&[], OBS_DIM).is_empty());
        assert_eq!(
            p.forward_vjp_batch(&[], &[], OBS_DIM),
            vec![0.0; p.n_params()]
        );

        let v = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        assert!(v.forward_batch(&[], OBS_DIM).is_empty());
        assert_eq!(
            v.mse_gradient_batch(&[], &[], OBS_DIM),
            vec![0.0; v.n_params()]
        );

        let q = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
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
