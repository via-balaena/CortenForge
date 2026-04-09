//! Autograd-backed policy implementation.
//!
//! [`AutogradPolicy`] implements [`Policy`] + [`DifferentiablePolicy`] for
//! arbitrary-depth networks. Gradients are computed automatically via the
//! tape — no hand-coded backprop per architecture.
//!
//! # Parameter layout
//!
//! For a network with `hidden_dims = [H1, H2]`, `obs_dim = O`, `act_dim = A`:
//!
//! ```text
//! W1[H1 × O]  b1[H1]  W2[H2 × H1]  b2[H2]  W3[A × H2]  b3[A]
//! ```
//!
//! Each layer's W is row-major. Parameters are contiguous in a single
//! `Vec<f64>` — same convention as [`MlpPolicy`](crate::MlpPolicy).

use crate::artifact::{NetworkKind, PolicyDescriptor};
use crate::autograd::{Tape, Var};
use crate::autograd_layers::{Activation, gaussian_log_prob, linear_hidden, linear_tanh};
use crate::policy::{DifferentiablePolicy, Policy, StochasticPolicy};

// ── Layer offset bookkeeping ──────────────────────────────────────────────

/// Start and end indices for one layer's W and b in the flat param vector.
#[derive(Clone, Debug)]
struct LayerOffsets {
    w_start: usize,
    w_end: usize,
    b_start: usize,
    b_end: usize,
    in_dim: usize,
    out_dim: usize,
}

/// Compute layer offsets for a sequence of layer sizes.
fn compute_offsets(layer_sizes: &[usize]) -> Vec<LayerOffsets> {
    let mut offsets = Vec::with_capacity(layer_sizes.len() - 1);
    let mut cursor = 0;
    for window in layer_sizes.windows(2) {
        let in_dim = window[0];
        let out_dim = window[1];
        let w_start = cursor;
        let w_end = w_start + out_dim * in_dim;
        let b_start = w_end;
        let b_end = b_start + out_dim;
        offsets.push(LayerOffsets {
            w_start,
            w_end,
            b_start,
            b_end,
            in_dim,
            out_dim,
        });
        cursor = b_end;
    }
    offsets
}

// ── Xavier / He initialization ────────────────────────────────────────────

/// Box-Muller normal sample.
fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Initialize weights using Xavier (Glorot) or He initialization.
///
/// - `Activation::Tanh` → Glorot uniform: `W ~ U(-limit, limit)`,
///   `limit = √(6 / (fan_in + fan_out))`.
/// - `Activation::Relu` → He normal: `W ~ N(0, √(2 / fan_in))`.
/// - Biases: always zero.
#[allow(clippy::cast_precision_loss)]
fn xavier_init(
    params: &mut [f64],
    layer_offsets: &[LayerOffsets],
    activation: Activation,
    rng: &mut impl rand::Rng,
) {
    for layer in layer_offsets {
        match activation {
            Activation::Tanh => {
                let limit = (6.0 / (layer.in_dim + layer.out_dim) as f64).sqrt();
                for p in &mut params[layer.w_start..layer.w_end] {
                    // Uniform on [-limit, limit]
                    *p = (2.0_f64).mul_add(rng.random::<f64>(), -1.0) * limit;
                }
            }
            Activation::Relu => {
                let std = (2.0 / layer.in_dim as f64).sqrt();
                for p in &mut params[layer.w_start..layer.w_end] {
                    *p = randn(rng) * std;
                }
            }
        }
        // Biases stay zero (already initialized).
    }
}

// ── AutogradPolicy ────────────────────────────────────────────────────────

/// Autograd-backed policy with arbitrary hidden layer depth.
///
/// Implements [`Policy`] + [`DifferentiablePolicy`]. For a single hidden
/// layer with the same dimensions, produces identical output to
/// [`MlpPolicy`](crate::MlpPolicy).
///
/// All gradient methods build a fresh [`Tape`], run the forward pass,
/// compute the loss/objective, call `backward()`, and read parameter
/// gradients. No hand-coded backprop.
pub struct AutogradPolicy {
    act_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<LayerOffsets>,
    activation: Activation,
}

impl AutogradPolicy {
    /// Create a new policy with zero-initialized parameters.
    ///
    /// `hidden_dims` specifies the size of each hidden layer. For example,
    /// `vec![32]` gives one hidden layer of 32 units (matching `MlpPolicy`),
    /// `vec![64, 64]` gives two hidden layers.
    ///
    /// Uses [`Activation::Tanh`] for hidden layers (matching the hand-coded
    /// oracle). Use [`new_with`](Self::new_with) to select a different
    /// activation.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dims: &[usize], act_dim: usize, obs_scale: &[f64]) -> Self {
        Self::new_with(obs_dim, hidden_dims, act_dim, obs_scale, Activation::Tanh)
    }

    /// Create a new policy with zero-initialized parameters and explicit
    /// activation choice for hidden layers.
    ///
    /// Output layer always uses tanh (bounds actions to [-1, 1]).
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new_with(
        obs_dim: usize,
        hidden_dims: &[usize],
        act_dim: usize,
        obs_scale: &[f64],
        activation: Activation,
    ) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "AutogradPolicy::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        assert!(
            !hidden_dims.is_empty(),
            "AutogradPolicy::new: hidden_dims must not be empty",
        );

        // Build layer size sequence: [obs_dim, H1, H2, ..., act_dim]
        let mut sizes = Vec::with_capacity(hidden_dims.len() + 2);
        sizes.push(obs_dim);
        sizes.extend_from_slice(hidden_dims);
        sizes.push(act_dim);

        let layer_offsets = compute_offsets(&sizes);
        let n_params = layer_offsets.last().map_or(0, |l| l.b_end);

        Self {
            act_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            layer_offsets,
            activation,
        }
    }

    /// Create a policy with Xavier/He-initialized weights.
    ///
    /// - [`Activation::Tanh`] → Glorot uniform: `W ~ U(-limit, limit)`
    ///   where `limit = √(6 / (fan_in + fan_out))`.
    /// - [`Activation::Relu`] → He normal: `W ~ N(0, √(2 / fan_in))`.
    /// - Biases: zero.
    #[must_use]
    pub fn new_xavier(
        obs_dim: usize,
        hidden_dims: &[usize],
        act_dim: usize,
        obs_scale: &[f64],
        activation: Activation,
        rng: &mut impl rand::Rng,
    ) -> Self {
        let mut policy = Self::new_with(obs_dim, hidden_dims, act_dim, obs_scale, activation);
        xavier_init(&mut policy.params, &policy.layer_offsets, activation, rng);
        policy
    }

    /// Run the forward pass on a tape, returning output `Var`s.
    ///
    /// `param_vars` are the policy parameters loaded onto the tape.
    /// `x` are the scaled observation `Var`s.
    fn forward_on_tape(
        &self,
        tape: &mut Tape,
        param_vars: &[crate::autograd::Var],
        x: &[crate::autograd::Var],
    ) -> Vec<crate::autograd::Var> {
        let n_layers = self.layer_offsets.len();
        let mut current = x.to_vec();

        for (i, layer) in self.layer_offsets.iter().enumerate() {
            let w = &param_vars[layer.w_start..layer.w_end];
            let b = &param_vars[layer.b_start..layer.b_end];
            let is_last = i == n_layers - 1;

            current = if is_last {
                // Output layer always uses tanh to bound actions to [-1, 1].
                linear_tanh(tape, w, b, &current, layer.out_dim, layer.in_dim)
            } else {
                // Hidden layers use the configured activation.
                linear_hidden(
                    tape,
                    w,
                    b,
                    &current,
                    layer.out_dim,
                    layer.in_dim,
                    self.activation,
                )
            };
        }
        current
    }

    /// Load params onto tape and build scaled obs constants.
    fn setup_tape(
        &self,
        obs: &[f32],
    ) -> (Tape, Vec<crate::autograd::Var>, Vec<crate::autograd::Var>) {
        let mut tape = Tape::new();
        let param_vars: Vec<crate::autograd::Var> =
            self.params.iter().map(|&val| tape.param(val)).collect();
        let x: Vec<crate::autograd::Var> = obs
            .iter()
            .zip(&self.obs_scale)
            .map(|(&ob, &sc)| tape.constant(f64::from(ob) * sc))
            .collect();
        (tape, param_vars, x)
    }
}

impl Policy for AutogradPolicy {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "AutogradPolicy::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn descriptor(&self) -> PolicyDescriptor {
        let hidden_dims: Vec<usize> = self
            .layer_offsets
            .iter()
            .take(self.layer_offsets.len().saturating_sub(1))
            .map(|l| l.out_dim)
            .collect();
        PolicyDescriptor {
            kind: NetworkKind::Autograd,
            obs_dim: self.obs_scale.len(),
            act_dim: self.act_dim,
            hidden_dims,
            activation: self.activation,
            obs_scale: self.obs_scale.clone(),
            stochastic: false,
        }
    }

    fn forward(&self, obs: &[f32]) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let output = self.forward_on_tape(&mut tape, &param_vars, &x);
        output.iter().map(|&var| tape.value(var)).collect()
    }
}

impl DifferentiablePolicy for AutogradPolicy {
    fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        // Build log_prob on the tape.
        let log_sigma = sigma.ln();
        let log_std_vars: Vec<crate::autograd::Var> = (0..self.act_dim)
            .map(|_| tape.constant(log_sigma))
            .collect();
        let action_vars: Vec<crate::autograd::Var> =
            action.iter().map(|&a| tape.constant(a)).collect();
        let log_prob = gaussian_log_prob(&mut tape, &mu, &log_std_vars, &action_vars);

        tape.backward(log_prob);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }

    fn forward_vjp(&self, obs: &[f32], v: &[f64]) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        // Compute v^T · μ — backward from this gives dμ/dθ · v.
        let v_vars: Vec<crate::autograd::Var> = v.iter().map(|&vi| tape.constant(vi)).collect();
        let products: Vec<crate::autograd::Var> = mu
            .iter()
            .zip(&v_vars)
            .map(|(&mi, &vi)| tape.mul(mi, vi))
            .collect();
        let objective = tape.sum(&products);

        tape.backward(objective);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }
}

// ── AutogradStochasticPolicy ──────────────────────────────────────────────

/// Autograd-backed stochastic policy with learned `log_std`.
///
/// Same MLP architecture as [`AutogradPolicy`] for the mean, plus
/// `act_dim` learnable `log_std` parameters appended after all network
/// params. Required by SAC.
///
/// # Parameter layout
///
/// `[network W1, b1, W2, b2, ..., Wn, bn, log_std[act_dim]]`
///
/// The first `n_network_params` entries are identical to [`AutogradPolicy`].
/// The last `act_dim` entries are `log_std`.
pub struct AutogradStochasticPolicy {
    act_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<LayerOffsets>,
    /// Index where `log_std` params begin in the flat param vector.
    log_std_offset: usize,
    activation: Activation,
}

impl AutogradStochasticPolicy {
    /// Create a new stochastic policy with zero-initialized network params
    /// and `log_std` initialized to `init_log_std`.
    ///
    /// Uses [`Activation::Tanh`] for hidden layers.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new(
        obs_dim: usize,
        hidden_dims: &[usize],
        act_dim: usize,
        obs_scale: &[f64],
        init_log_std: f64,
    ) -> Self {
        Self::new_with(
            obs_dim,
            hidden_dims,
            act_dim,
            obs_scale,
            init_log_std,
            Activation::Tanh,
        )
    }

    /// Create a new stochastic policy with explicit activation choice.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new_with(
        obs_dim: usize,
        hidden_dims: &[usize],
        act_dim: usize,
        obs_scale: &[f64],
        init_log_std: f64,
        activation: Activation,
    ) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "AutogradStochasticPolicy::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        assert!(
            !hidden_dims.is_empty(),
            "AutogradStochasticPolicy::new: hidden_dims must not be empty",
        );

        let mut sizes = Vec::with_capacity(hidden_dims.len() + 2);
        sizes.push(obs_dim);
        sizes.extend_from_slice(hidden_dims);
        sizes.push(act_dim);

        let layer_offsets = compute_offsets(&sizes);
        let log_std_offset = layer_offsets.last().map_or(0, |l| l.b_end);
        let n_params = log_std_offset + act_dim;

        let mut params = vec![0.0; n_params];
        for i in 0..act_dim {
            params[log_std_offset + i] = init_log_std;
        }

        Self {
            act_dim,
            obs_scale: obs_scale.to_vec(),
            params,
            layer_offsets,
            log_std_offset,
            activation,
        }
    }

    /// Create a stochastic policy with Xavier/He-initialized weights.
    ///
    /// Network weights are initialized per activation strategy.
    /// `log_std` params are initialized to `init_log_std`.
    #[must_use]
    pub fn new_xavier(
        obs_dim: usize,
        hidden_dims: &[usize],
        act_dim: usize,
        obs_scale: &[f64],
        init_log_std: f64,
        activation: Activation,
        rng: &mut impl rand::Rng,
    ) -> Self {
        let mut policy = Self::new_with(
            obs_dim,
            hidden_dims,
            act_dim,
            obs_scale,
            init_log_std,
            activation,
        );
        xavier_init(&mut policy.params, &policy.layer_offsets, activation, rng);
        policy
    }

    /// Forward pass on a tape, returning output mean `Var`s.
    fn forward_on_tape(&self, tape: &mut Tape, param_vars: &[Var], x: &[Var]) -> Vec<Var> {
        let n_layers = self.layer_offsets.len();
        let mut current = x.to_vec();
        for (i, layer) in self.layer_offsets.iter().enumerate() {
            let w = &param_vars[layer.w_start..layer.w_end];
            let b = &param_vars[layer.b_start..layer.b_end];
            let is_last = i == n_layers - 1;
            current = if is_last {
                linear_tanh(tape, w, b, &current, layer.out_dim, layer.in_dim)
            } else {
                linear_hidden(
                    tape,
                    w,
                    b,
                    &current,
                    layer.out_dim,
                    layer.in_dim,
                    self.activation,
                )
            };
        }
        current
    }

    fn setup_tape(&self, obs: &[f32]) -> (Tape, Vec<Var>, Vec<Var>) {
        let mut tape = Tape::new();
        let param_vars: Vec<Var> = self.params.iter().map(|&val| tape.param(val)).collect();
        let x: Vec<Var> = obs
            .iter()
            .zip(&self.obs_scale)
            .map(|(&ob, &sc)| tape.constant(f64::from(ob) * sc))
            .collect();
        (tape, param_vars, x)
    }

    /// Get `log_std` `Var`s from the param vars on the tape.
    fn log_std_vars<'a>(&self, param_vars: &'a [Var]) -> &'a [Var] {
        &param_vars[self.log_std_offset..self.log_std_offset + self.act_dim]
    }
}

impl Policy for AutogradStochasticPolicy {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "AutogradStochasticPolicy::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn descriptor(&self) -> PolicyDescriptor {
        let hidden_dims: Vec<usize> = self
            .layer_offsets
            .iter()
            .take(self.layer_offsets.len().saturating_sub(1))
            .map(|l| l.out_dim)
            .collect();
        PolicyDescriptor {
            kind: NetworkKind::Autograd,
            obs_dim: self.obs_scale.len(),
            act_dim: self.act_dim,
            hidden_dims,
            activation: self.activation,
            obs_scale: self.obs_scale.clone(),
            stochastic: true,
        }
    }

    fn forward(&self, obs: &[f32]) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);
        mu.iter().map(|&var| tape.value(var)).collect()
    }
}

impl DifferentiablePolicy for AutogradStochasticPolicy {
    fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        let log_sigma = sigma.ln();
        let log_std_vars: Vec<Var> = (0..self.act_dim)
            .map(|_| tape.constant(log_sigma))
            .collect();
        let action_vars: Vec<Var> = action.iter().map(|&a| tape.constant(a)).collect();
        let log_prob = gaussian_log_prob(&mut tape, &mu, &log_std_vars, &action_vars);

        tape.backward(log_prob);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }

    fn forward_vjp(&self, obs: &[f32], v: &[f64]) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        let v_vars: Vec<Var> = v.iter().map(|&vi| tape.constant(vi)).collect();
        let products: Vec<Var> = mu
            .iter()
            .zip(&v_vars)
            .map(|(&mi, &vi)| tape.mul(mi, vi))
            .collect();
        let objective = tape.sum(&products);

        tape.backward(objective);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }
}

impl StochasticPolicy for AutogradStochasticPolicy {
    fn forward_stochastic(&self, obs: &[f32]) -> (Vec<f64>, Vec<f64>) {
        let mu = self.forward(obs);
        let log_std = self.params[self.log_std_offset..self.log_std_offset + self.act_dim].to_vec();
        (mu, log_std)
    }

    fn log_prob_gradient_stochastic(&self, obs: &[f32], action: &[f64]) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        // Use the learned log_std params (not external sigma).
        let ls_vars = self.log_std_vars(&param_vars).to_vec();
        let action_vars: Vec<Var> = action.iter().map(|&a| tape.constant(a)).collect();
        let log_prob = gaussian_log_prob(&mut tape, &mu, &ls_vars, &action_vars);

        tape.backward(log_prob);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }

    fn entropy(&self, _obs: &[f32]) -> f64 {
        // H[N(μ, σ²)] = Σ_a [0.5 * ln(2πe) + log_std_a]
        let half_ln_2pie = 0.5 * (2.0 * std::f64::consts::PI * std::f64::consts::E).ln();
        let mut h = 0.0;
        for a in 0..self.act_dim {
            h += half_ln_2pie + self.params[self.log_std_offset + a];
        }
        h
    }

    fn reparameterized_vjp(&self, obs: &[f32], eps: &[f64], v: &[f64]) -> Vec<f64> {
        // a = μ(obs) + exp(log_std) · ε
        // We want v^T · d(a)/d(params).
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let mu = self.forward_on_tape(&mut tape, &param_vars, &x);

        // Build reparameterized action on the tape:
        // a_i = mu_i + exp(log_std_i) * eps_i
        let ls_vars = self.log_std_vars(&param_vars).to_vec();
        let mut action_vars = Vec::with_capacity(self.act_dim);
        for a in 0..self.act_dim {
            let std_a = tape.exp(ls_vars[a]);
            let eps_var = tape.constant(eps[a]);
            let noise = tape.mul(std_a, eps_var);
            action_vars.push(tape.add(mu[a], noise));
        }

        // Objective = v^T · action
        let v_vars: Vec<Var> = v.iter().map(|&vi| tape.constant(vi)).collect();
        let products: Vec<Var> = action_vars
            .iter()
            .zip(&v_vars)
            .map(|(&ai, &vi)| tape.mul(ai, vi))
            .collect();
        let objective = tape.sum(&products);

        tape.backward(objective);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use crate::mlp::MlpPolicy;
    use crate::policy::{DifferentiablePolicy, Policy};

    const OBS_DIM: usize = 4;
    const HIDDEN: usize = 3;
    const ACT_DIM: usize = 2;
    const OBS_SCALE: [f64; OBS_DIM] = [0.5, 1.0, 0.1, 2.0];
    const PARITY_TOL: f64 = 1e-10;

    fn test_obs() -> Vec<f32> {
        vec![1.0_f32, -0.5, 3.0, 0.2]
    }

    fn test_action() -> Vec<f64> {
        vec![0.3, -0.7]
    }

    /// Same params used in mlp.rs tests: W1[3×4] + b1[3] + W2[2×3] + b2[2] = 23
    fn test_params() -> Vec<f64> {
        vec![
            0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, // W1
            0.05, -0.1, 0.15, // b1
            0.4, -0.6, 0.2, -0.3, 0.5, -0.1, // W2
            0.08, -0.05, // b2
        ]
    }

    fn make_pair() -> (MlpPolicy, AutogradPolicy) {
        let mut mlp = MlpPolicy::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let mut ag = AutogradPolicy::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_params();
        mlp.set_params(&params);
        ag.set_params(&params);
        (mlp, ag)
    }

    // ── Forward parity ────────────────────────────────────────────

    #[test]
    fn forward_matches_mlp() {
        let (mlp, ag) = make_pair();
        let obs = test_obs();
        let mlp_out = mlp.forward(&obs);
        let ag_out = ag.forward(&obs);
        for (i, (&m, &a)) in mlp_out.iter().zip(&ag_out).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "forward[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    #[test]
    fn n_params_matches_mlp() {
        let (mlp, ag) = make_pair();
        assert_eq!(mlp.n_params(), ag.n_params());
    }

    // ── log_prob_gradient parity ──────────────────────────────────

    #[test]
    fn log_prob_gradient_matches_mlp() {
        let (mlp, ag) = make_pair();
        let obs = test_obs();
        let action = test_action();
        let sigma = 0.3;

        let mlp_grad = mlp.log_prob_gradient(&obs, &action, sigma);
        let ag_grad = ag.log_prob_gradient(&obs, &action, sigma);

        assert_eq!(mlp_grad.len(), ag_grad.len());
        for (i, (&m, &a)) in mlp_grad.iter().zip(&ag_grad).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "log_prob_gradient[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    // ── forward_vjp parity ────────────────────────────────────────

    #[test]
    fn forward_vjp_matches_mlp() {
        let (mlp, ag) = make_pair();
        let obs = test_obs();
        let v = vec![0.5, -0.3];

        let mlp_vjp = mlp.forward_vjp(&obs, &v);
        let ag_vjp = ag.forward_vjp(&obs, &v);

        assert_eq!(mlp_vjp.len(), ag_vjp.len());
        for (i, (&m, &a)) in mlp_vjp.iter().zip(&ag_vjp).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "forward_vjp[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    // ── Params roundtrip ──────────────────────────────────────────

    #[test]
    fn params_roundtrip() {
        let mut ag = AutogradPolicy::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_params();
        ag.set_params(&params);
        assert_eq!(ag.params(), &params[..]);
    }

    // ── Two hidden layers (no oracle — just FD) ───────────────────

    #[test]
    fn two_layer_log_prob_gradient_matches_fd() {
        let obs_dim = 3;
        let act_dim = 2;
        let scale = [1.0, 1.0, 1.0];
        let mut ag = AutogradPolicy::new(obs_dim, &[4, 4], act_dim, &scale);
        // Random-ish params
        let n = ag.n_params();
        let params: Vec<f64> = (0..n).map(|i| ((i as f64) * 0.13).sin() * 0.5).collect();
        ag.set_params(&params);

        let obs = [0.5_f32, -0.3, 0.8];
        let action = [0.2, -0.4];
        let sigma = 0.3;

        let analytical = ag.log_prob_gradient(&obs, &action, sigma);

        let eps = 1e-6;
        let tol = 1e-4;
        let mut perturbed = params.clone();
        for i in 0..n {
            let orig = perturbed[i];

            perturbed[i] = orig + eps;
            ag.set_params(&perturbed);
            let lp_plus = eval_log_prob(&ag, &obs, &action, sigma);

            perturbed[i] = orig - eps;
            ag.set_params(&perturbed);
            let lp_minus = eval_log_prob(&ag, &obs, &action, sigma);

            perturbed[i] = orig;

            let fd = (lp_plus - lp_minus) / (2.0 * eps);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < tol,
                "2-layer FD param[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }
    }

    /// Helper: compute `log_prob` as a scalar for FD testing.
    fn eval_log_prob(policy: &AutogradPolicy, obs: &[f32], action: &[f64], sigma: f64) -> f64 {
        let mu = policy.forward(obs);
        let sigma2 = sigma * sigma;
        mu.iter()
            .zip(action)
            .map(|(&m, &a)| -0.5 * (a - m).powi(2) / sigma2)
            .sum::<f64>()
    }

    // ── AutogradStochasticPolicy FD tests ─────────────────────────

    fn make_stochastic() -> AutogradStochasticPolicy {
        let obs_dim = 3;
        let act_dim = 2;
        let scale = [1.0, 1.0, 1.0];
        let mut p = AutogradStochasticPolicy::new(obs_dim, &[4], act_dim, &scale, -0.5);
        let n = p.n_params();
        let params: Vec<f64> = (0..n).map(|i| ((i as f64) * 0.17).sin() * 0.5).collect();
        p.set_params(&params);
        p
    }

    fn stochastic_obs() -> Vec<f32> {
        vec![0.5_f32, -0.3, 0.8]
    }

    fn stochastic_action() -> Vec<f64> {
        vec![0.2, -0.4]
    }

    #[test]
    fn stochastic_forward_stochastic_returns_log_std() {
        let p = make_stochastic();
        let (mu, log_std) = p.forward_stochastic(&stochastic_obs());
        assert_eq!(mu.len(), 2);
        assert_eq!(log_std.len(), 2);
        // log_std should match the params at the offset
        for (i, &ls) in log_std.iter().enumerate() {
            assert!((ls - p.params()[p.log_std_offset + i]).abs() < 1e-12);
        }
    }

    #[test]
    fn stochastic_log_prob_gradient_matches_fd() {
        let mut p = make_stochastic();
        let obs = stochastic_obs();
        let action = stochastic_action();
        let analytical = p.log_prob_gradient_stochastic(&obs, &action);

        let params = p.params().to_vec();
        let eps = 1e-6;
        let tol = 1e-4;

        for i in 0..params.len() {
            let mut perturbed = params.clone();

            perturbed[i] = params[i] + eps;
            p.set_params(&perturbed);
            let lp_plus = eval_stochastic_log_prob(&p, &obs, &action);

            perturbed[i] = params[i] - eps;
            p.set_params(&perturbed);
            let lp_minus = eval_stochastic_log_prob(&p, &obs, &action);

            let fd = (lp_plus - lp_minus) / (2.0 * eps);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < tol,
                "stochastic log_prob_grad FD param[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }
    }

    #[test]
    fn stochastic_reparameterized_vjp_matches_fd() {
        let mut p = make_stochastic();
        let obs = stochastic_obs();
        let eps_noise = vec![0.7, -0.3];
        let v = vec![0.5, -0.2];
        let analytical = p.reparameterized_vjp(&obs, &eps_noise, &v);

        let params = p.params().to_vec();
        let fd_eps = 1e-6;
        let tol = 1e-4;

        for i in 0..params.len() {
            let mut perturbed = params.clone();

            perturbed[i] = params[i] + fd_eps;
            p.set_params(&perturbed);
            let a_plus = eval_reparam_action(&p, &obs, &eps_noise);
            let obj_plus: f64 = a_plus.iter().zip(&v).map(|(&ai, &vi)| ai * vi).sum();

            perturbed[i] = params[i] - fd_eps;
            p.set_params(&perturbed);
            let a_minus = eval_reparam_action(&p, &obs, &eps_noise);
            let obj_minus: f64 = a_minus.iter().zip(&v).map(|(&ai, &vi)| ai * vi).sum();

            let fd = (obj_plus - obj_minus) / (2.0 * fd_eps);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < tol,
                "reparam VJP FD param[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }
    }

    #[test]
    fn stochastic_entropy_positive() {
        let p = make_stochastic();
        let h = p.entropy(&stochastic_obs());
        // Gaussian entropy is always positive for finite σ.
        assert!(h > 0.0, "entropy should be positive, got {h}");
    }

    #[test]
    fn stochastic_params_roundtrip() {
        let p = make_stochastic();
        let params = p.params().to_vec();
        let mut p2 = AutogradStochasticPolicy::new(3, &[4], 2, &[1.0, 1.0, 1.0], -0.5);
        p2.set_params(&params);
        assert_eq!(p2.params(), &params[..]);
    }

    /// Evaluate stochastic `log_prob` numerically (no autograd).
    fn eval_stochastic_log_prob(
        policy: &AutogradStochasticPolicy,
        obs: &[f32],
        action: &[f64],
    ) -> f64 {
        let (mu, log_std) = policy.forward_stochastic(obs);
        let mut lp = 0.0;
        for i in 0..mu.len() {
            let std_i = log_std[i].exp();
            let var = std_i * std_i;
            let diff = action[i] - mu[i];
            lp += 0.5_f64.mul_add(
                -(2.0 * std::f64::consts::PI).ln(),
                -0.5 * diff * diff / var - log_std[i],
            );
        }
        lp
    }

    /// Evaluate reparameterized action numerically.
    fn eval_reparam_action(
        policy: &AutogradStochasticPolicy,
        obs: &[f32],
        eps: &[f64],
    ) -> Vec<f64> {
        let (mu, log_std) = policy.forward_stochastic(obs);
        mu.iter()
            .zip(&log_std)
            .zip(eps)
            .map(|((&m, &ls), &e)| ls.exp().mul_add(e, m))
            .collect()
    }

    // ── ReLU activation tests ────────────────────────────────────

    #[test]
    fn relu_two_layer_log_prob_gradient_matches_fd() {
        let obs_dim = 3;
        let act_dim = 2;
        let scale = [1.0, 1.0, 1.0];
        let mut ag =
            AutogradPolicy::new_with(obs_dim, &[4, 4], act_dim, &scale, crate::Activation::Relu);
        let n = ag.n_params();
        let params: Vec<f64> = (0..n).map(|i| ((i as f64) * 0.13).sin() * 0.5).collect();
        ag.set_params(&params);

        let obs = [0.5_f32, -0.3, 0.8];
        let action = [0.2, -0.4];
        let sigma = 0.3;

        let analytical = ag.log_prob_gradient(&obs, &action, sigma);

        let eps = 1e-6;
        let tol = 1e-4;
        let mut perturbed = params.clone();
        for i in 0..n {
            let orig = perturbed[i];

            perturbed[i] = orig + eps;
            ag.set_params(&perturbed);
            let lp_plus = eval_log_prob(&ag, &obs, &action, sigma);

            perturbed[i] = orig - eps;
            ag.set_params(&perturbed);
            let lp_minus = eval_log_prob(&ag, &obs, &action, sigma);

            perturbed[i] = orig;

            let fd = (lp_plus - lp_minus) / (2.0 * eps);
            let err = (analytical[i] - fd).abs();
            assert!(
                err < tol,
                "ReLU 2-layer FD param[{i}]: analytical={}, fd={fd}, err={err}",
                analytical[i],
            );
        }
    }

    // ── Xavier init ──────────────────────────────────────────────

    #[test]
    fn xavier_init_nonzero_weights() {
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);

        let p =
            AutogradPolicy::new_xavier(4, &[8, 8], 2, &[1.0; 4], crate::Activation::Relu, &mut rng);

        // Weights should be non-zero.
        let any_nonzero = p.params().iter().any(|&v| v.abs() > 1e-12);
        assert!(any_nonzero, "Xavier init should produce non-zero weights");
    }

    // ── Convergence with 2-layer ReLU + Xavier ───────────────────

    #[test]
    fn relu_xavier_convergence_2dof() {
        use crate::algorithm::{Algorithm, TrainingBudget};
        use crate::optimizer::OptimizerConfig;
        use crate::reaching_2dof;
        use crate::reinforce::{Reinforce, ReinforceHyperparams};
        use rand::SeedableRng;

        let task = reaching_2dof();
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);

        let policy = Box::new(AutogradPolicy::new_xavier(
            task.obs_dim(),
            &[32, 32],
            task.act_dim(),
            task.obs_scale(),
            crate::Activation::Relu,
            &mut rng,
        ));

        let mut algo = Reinforce::new(
            policy,
            OptimizerConfig::adam(0.05),
            ReinforceHyperparams {
                gamma: 0.99,
                sigma_init: 0.5,
                sigma_decay: 0.95,
                sigma_min: 0.05,
                max_episode_steps: 300,
            },
        );

        let mut env = task.build_vec_env(20).unwrap();
        let metrics = algo.train(&mut env, TrainingBudget::Epochs(20), 42, &|_| {});

        let first = metrics[0].mean_reward;
        let last = metrics[19].mean_reward;
        assert!(
            last > first,
            "2-layer ReLU+Xavier should improve: first={first:.1}, last={last:.1}",
        );
    }
}
