//! Tape-based reverse-mode automatic differentiation and the RL building
//! blocks layered on it.
//!
//! - `tape`: the core reverse-mode engine — [`Var`], [`Tape`], and the
//!   [`VjpOp`] vector-Jacobian-product trait custom ops implement.
//! - `layers`: linear layers, activations, and loss functions expressed as
//!   `Tape` operations.
//! - `policy`: [`AutogradPolicy`] / [`AutogradStochasticPolicy`] — the
//!   differentiable policy networks used by the gradient-based RL algorithms.
//! - `value`: [`AutogradValue`] / [`AutogradQ`] — value and Q-function
//!   networks built on the same primitives.
//!
//! The public surface is re-exported here so callers reach it through
//! `crate::autograd::…` (and, via `lib.rs`, `sim_ml_chassis::…`) regardless of
//! which submodule an item lives in.

mod layers;
mod policy;
mod tape;
mod value;

pub use layers::{
    Activation, gaussian_log_prob, linear_hidden, linear_raw, linear_relu, linear_tanh, mse_loss,
    mse_loss_batch,
};
pub use policy::{AutogradPolicy, AutogradStochasticPolicy};
pub use tape::{Tape, Var, VjpOp};
pub use value::{AutogradQ, AutogradValue};
