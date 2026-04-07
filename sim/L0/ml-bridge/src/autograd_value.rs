//! Autograd-backed value function and Q-function implementations.
//!
//! - [`AutogradValue`] implements [`ValueFn`] — state value V(s).
//! - [`AutogradQ`] implements [`QFunction`] — state-action value Q(s, a).
//!
//! Both support arbitrary hidden layer depth. For single-hidden-layer
//! configs with matching params, they produce identical output to
//! [`MlpValue`](crate::MlpValue) and [`MlpQ`](crate::MlpQ).
//!
//! # Parameter layout
//!
//! Same convention as [`AutogradPolicy`](crate::AutogradPolicy): contiguous
//! `Vec<f64>` with `W1 b1 W2 b2 ...` in row-major order. The output layer
//! is linear (no tanh) — value predictions are unbounded.

use crate::autograd::{Tape, Var};
use crate::autograd_layers::{linear_raw, linear_tanh, mse_loss};
use crate::value::{QFunction, ValueFn};

// ── Layer offset bookkeeping (shared with autograd_policy.rs) ─────────────

#[derive(Clone, Debug)]
struct LayerOffsets {
    w_start: usize,
    w_end: usize,
    b_start: usize,
    b_end: usize,
    in_dim: usize,
    out_dim: usize,
}

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

// ── AutogradValue ─────────────────────────────────────────────────────────

/// Autograd-backed state value function V(s).
///
/// Linear output layer (no tanh) — value predictions are unbounded.
/// Output dimension is always 1 (scalar value).
pub struct AutogradValue {
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<LayerOffsets>,
}

impl AutogradValue {
    /// Create a new value function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dims: &[usize], obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "AutogradValue::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        assert!(
            !hidden_dims.is_empty(),
            "AutogradValue::new: hidden_dims must not be empty",
        );

        let mut sizes = Vec::with_capacity(hidden_dims.len() + 2);
        sizes.push(obs_dim);
        sizes.extend_from_slice(hidden_dims);
        sizes.push(1); // scalar output
        let layer_offsets = compute_offsets(&sizes);
        let n_params = layer_offsets.last().map_or(0, |l| l.b_end);

        Self {
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            layer_offsets,
        }
    }

    /// Forward pass on a tape, returning the scalar output `Var`.
    fn forward_on_tape(&self, tape: &mut Tape, param_vars: &[Var], x: &[Var]) -> Var {
        let n_layers = self.layer_offsets.len();
        let mut current = x.to_vec();

        for (i, layer) in self.layer_offsets.iter().enumerate() {
            let w = &param_vars[layer.w_start..layer.w_end];
            let b = &param_vars[layer.b_start..layer.b_end];
            let is_last = i == n_layers - 1;

            current = if is_last {
                linear_raw(tape, w, b, &current, layer.out_dim, layer.in_dim)
            } else {
                linear_tanh(tape, w, b, &current, layer.out_dim, layer.in_dim)
            };
        }
        current[0]
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
}

impl ValueFn for AutogradValue {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "AutogradValue::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32]) -> f64 {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let out = self.forward_on_tape(&mut tape, &param_vars, &x);
        tape.value(out)
    }

    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64> {
        let (mut tape, param_vars, x) = self.setup_tape(obs);
        let pred = self.forward_on_tape(&mut tape, &param_vars, &x);
        let target_var = tape.constant(target);
        let loss = mse_loss(&mut tape, pred, target_var);
        tape.backward(loss);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }
}

// ── AutogradQ ─────────────────────────────────────────────────────────────

/// Autograd-backed state-action value function Q(s, a).
///
/// Input is the concatenation of scaled observations and raw actions:
/// `[s_scaled; a]`. Linear output layer (scalar Q-value).
pub struct AutogradQ {
    obs_dim: usize,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<LayerOffsets>,
}

impl AutogradQ {
    /// Create a new Q-function with zero-initialized parameters.
    ///
    /// # Panics
    ///
    /// Panics if `obs_scale.len() != obs_dim` or `hidden_dims` is empty.
    #[must_use]
    pub fn new(obs_dim: usize, hidden_dims: &[usize], act_dim: usize, obs_scale: &[f64]) -> Self {
        assert!(
            obs_scale.len() == obs_dim,
            "AutogradQ::new: obs_scale.len() ({}) != obs_dim ({obs_dim})",
            obs_scale.len(),
        );
        assert!(
            !hidden_dims.is_empty(),
            "AutogradQ::new: hidden_dims must not be empty",
        );

        let input_dim = obs_dim + act_dim;
        let mut sizes = Vec::with_capacity(hidden_dims.len() + 2);
        sizes.push(input_dim);
        sizes.extend_from_slice(hidden_dims);
        sizes.push(1); // scalar output
        let layer_offsets = compute_offsets(&sizes);
        let n_params = layer_offsets.last().map_or(0, |l| l.b_end);

        Self {
            obs_dim,
            obs_scale: obs_scale.to_vec(),
            params: vec![0.0; n_params],
            layer_offsets,
        }
    }

    /// Build concatenated input `[s_scaled; action]` on the tape.
    fn build_input(
        &self,
        tape: &mut Tape,
        obs: &[f32],
        action: &[f64],
        action_as_params: bool,
    ) -> Vec<Var> {
        let mut input: Vec<Var> = obs
            .iter()
            .zip(&self.obs_scale)
            .map(|(&ob, &sc)| tape.constant(f64::from(ob) * sc))
            .collect();
        if action_as_params {
            input.extend(action.iter().map(|&a| tape.param(a)));
        } else {
            input.extend(action.iter().map(|&a| tape.constant(a)));
        }
        input
    }

    fn forward_on_tape(&self, tape: &mut Tape, param_vars: &[Var], input: &[Var]) -> Var {
        let n_layers = self.layer_offsets.len();
        let mut current = input.to_vec();

        for (i, layer) in self.layer_offsets.iter().enumerate() {
            let w = &param_vars[layer.w_start..layer.w_end];
            let b = &param_vars[layer.b_start..layer.b_end];
            let is_last = i == n_layers - 1;

            current = if is_last {
                linear_raw(tape, w, b, &current, layer.out_dim, layer.in_dim)
            } else {
                linear_tanh(tape, w, b, &current, layer.out_dim, layer.in_dim)
            };
        }
        current[0]
    }
}

impl QFunction for AutogradQ {
    fn n_params(&self) -> usize {
        self.params.len()
    }

    fn params(&self) -> &[f64] {
        &self.params
    }

    fn set_params(&mut self, params: &[f64]) {
        assert!(
            params.len() == self.params.len(),
            "AutogradQ::set_params: expected {} params, got {}",
            self.params.len(),
            params.len(),
        );
        self.params.copy_from_slice(params);
    }

    fn forward(&self, obs: &[f32], action: &[f64]) -> f64 {
        let mut tape = Tape::new();
        let param_vars: Vec<Var> = self.params.iter().map(|&val| tape.param(val)).collect();
        let input = self.build_input(&mut tape, obs, action, false);
        let out = self.forward_on_tape(&mut tape, &param_vars, &input);
        tape.value(out)
    }

    fn mse_gradient(&self, obs: &[f32], action: &[f64], target: f64) -> Vec<f64> {
        let mut tape = Tape::new();
        let param_vars: Vec<Var> = self.params.iter().map(|&val| tape.param(val)).collect();
        let input = self.build_input(&mut tape, obs, action, false);
        let pred = self.forward_on_tape(&mut tape, &param_vars, &input);
        let target_var = tape.constant(target);
        let loss = mse_loss(&mut tape, pred, target_var);
        tape.backward(loss);
        param_vars.iter().map(|&var| tape.grad(var)).collect()
    }

    fn action_gradient(&self, obs: &[f32], action: &[f64]) -> Vec<f64> {
        // Actions are params on the tape so backward computes dQ/da.
        let mut tape = Tape::new();
        let param_vars: Vec<Var> = self.params.iter().map(|&val| tape.param(val)).collect();
        let input = self.build_input(&mut tape, obs, action, true);
        // The action Vars are the last `act_dim` entries of `input`.
        let action_vars = &input[self.obs_dim..];
        let q_out = self.forward_on_tape(&mut tape, &param_vars, &input);
        tape.backward(q_out);
        action_vars.iter().map(|&var| tape.grad(var)).collect()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use crate::mlp::{MlpQ, MlpValue};
    use crate::value::{QFunction, ValueFn};

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

    // MlpValue: W1[3×4]=12 + b1[3] + w2[3] + b2 = 19
    fn test_value_params() -> Vec<f64> {
        vec![
            0.3, -0.5, 0.8, -0.2, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, // W1
            0.05, -0.1, 0.15, // b1
            0.4, -0.6, 0.2,  // w2
            0.08, // b2
        ]
    }

    // MlpQ: W1[3×6]=18 + b1[3] + w2[3] + b2 = 25 (input_dim = obs+act = 6)
    fn test_q_params() -> Vec<f64> {
        vec![
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, 0.1, 0.6, -0.4, 0.7, -0.3, 0.2, 0.5, -0.1, 0.2, -0.3,
            0.6, -0.4, // W1[3×6]
            0.05, -0.1, 0.15, // b1
            0.4, -0.6, 0.2,  // w2
            0.08, // b2
        ]
    }

    // ── AutogradValue parity ──────────────────────────────────────

    #[test]
    fn value_forward_matches_mlp() {
        let mut mlp = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        let mut ag = AutogradValue::new(OBS_DIM, &[HIDDEN], &OBS_SCALE);
        let params = test_value_params();
        mlp.set_params(&params);
        ag.set_params(&params);

        let obs = test_obs();
        let err = (mlp.forward(&obs) - ag.forward(&obs)).abs();
        assert!(err < PARITY_TOL, "value forward: err={err}");
    }

    #[test]
    fn value_n_params_matches_mlp() {
        let mlp = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        let ag = AutogradValue::new(OBS_DIM, &[HIDDEN], &OBS_SCALE);
        assert_eq!(mlp.n_params(), ag.n_params());
    }

    #[test]
    fn value_mse_gradient_matches_mlp() {
        let mut mlp = MlpValue::new(OBS_DIM, HIDDEN, &OBS_SCALE);
        let mut ag = AutogradValue::new(OBS_DIM, &[HIDDEN], &OBS_SCALE);
        let params = test_value_params();
        mlp.set_params(&params);
        ag.set_params(&params);

        let obs = test_obs();
        let target = 2.0;
        let mlp_grad = mlp.mse_gradient(&obs, target);
        let ag_grad = ag.mse_gradient(&obs, target);

        assert_eq!(mlp_grad.len(), ag_grad.len());
        for (i, (&m, &a)) in mlp_grad.iter().zip(&ag_grad).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "value mse_gradient[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    // ── AutogradQ parity ──────────────────────────────────────────

    #[test]
    fn q_forward_matches_mlp() {
        let mut mlp = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let mut ag = AutogradQ::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_q_params();
        mlp.set_params(&params);
        ag.set_params(&params);

        let obs = test_obs();
        let action = test_action();
        let err = (mlp.forward(&obs, &action) - ag.forward(&obs, &action)).abs();
        assert!(err < PARITY_TOL, "Q forward: err={err}");
    }

    #[test]
    fn q_n_params_matches_mlp() {
        let mlp = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let ag = AutogradQ::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        assert_eq!(mlp.n_params(), ag.n_params());
    }

    #[test]
    fn q_mse_gradient_matches_mlp() {
        let mut mlp = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let mut ag = AutogradQ::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_q_params();
        mlp.set_params(&params);
        ag.set_params(&params);

        let obs = test_obs();
        let action = test_action();
        let target = 2.0;
        let mlp_grad = mlp.mse_gradient(&obs, &action, target);
        let ag_grad = ag.mse_gradient(&obs, &action, target);

        assert_eq!(mlp_grad.len(), ag_grad.len());
        for (i, (&m, &a)) in mlp_grad.iter().zip(&ag_grad).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "Q mse_gradient[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    #[test]
    fn q_action_gradient_matches_mlp() {
        let mut mlp = MlpQ::new(OBS_DIM, HIDDEN, ACT_DIM, &OBS_SCALE);
        let mut ag = AutogradQ::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_q_params();
        mlp.set_params(&params);
        ag.set_params(&params);

        let obs = test_obs();
        let action = test_action();
        let mlp_dq = mlp.action_gradient(&obs, &action);
        let ag_dq = ag.action_gradient(&obs, &action);

        assert_eq!(mlp_dq.len(), ag_dq.len());
        for (i, (&m, &a)) in mlp_dq.iter().zip(&ag_dq).enumerate() {
            let err = (m - a).abs();
            assert!(
                err < PARITY_TOL,
                "Q action_gradient[{i}]: mlp={m}, autograd={a}, err={err}",
            );
        }
    }

    // ── Params roundtrip ──────────────────────────────────────────

    #[test]
    fn value_params_roundtrip() {
        let mut ag = AutogradValue::new(OBS_DIM, &[HIDDEN], &OBS_SCALE);
        let params = test_value_params();
        ag.set_params(&params);
        assert_eq!(ag.params(), &params[..]);
    }

    #[test]
    fn q_params_roundtrip() {
        let mut ag = AutogradQ::new(OBS_DIM, &[HIDDEN], ACT_DIM, &OBS_SCALE);
        let params = test_q_params();
        ag.set_params(&params);
        assert_eq!(ag.params(), &params[..]);
    }
}
