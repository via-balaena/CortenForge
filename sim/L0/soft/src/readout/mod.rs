//! Readout types + `ForwardMap` trait.
//!
//! γ-locked API surface per
//! [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md):
//! `RewardBreakdown`, `EditResult`, `GradientEstimate`, `RewardWeights`,
//! and the `ForwardMap` trait itself. Skeleton defines the types;
//! Phase B wires `SkeletonForwardMap` (concrete impl) to compose the
//! solver + differentiable + observable stack.

use sim_ml_chassis::{Tape, Tensor};

pub mod edit_result;
pub mod gradient_estimate;
pub mod reward_breakdown;
pub mod reward_weights;
pub mod scene;
pub mod skeleton_forward_map;

pub use edit_result::EditResult;
pub use gradient_estimate::GradientEstimate;
pub use reward_breakdown::{ResidualCorrections, RewardBreakdown};
pub use reward_weights::RewardWeights;
pub use scene::{BoundaryConditions, LoadAxis, SceneInitial, SoftScene};
pub use skeleton_forward_map::SkeletonForwardMap;

/// γ-locked forward-map trait.
///
/// One `evaluate` call produces the per-term `RewardBreakdown` plus
/// the `EditResult` classification; one `gradient` call reads the
/// cotangent of θ from the already-populated tape.
///
/// **Determinism-in-θ contract:** repeated calls at the same θ produce
/// the same `RewardBreakdown` modulo cache state. No stochastic sim
/// parameters update between calls. This invariant is what keeps the
/// `BayesOpt` GP's cached training data from Part 10 Ch 02 valid across
/// residual-GP updates in Ch 05 §01.
pub trait ForwardMap {
    /// Run one forward pass at θ, recording adjoint hooks on `tape`.
    fn evaluate(&mut self, theta: &Tensor<f64>, tape: &mut Tape) -> (RewardBreakdown, EditResult);

    /// Read the cotangent of θ from the tape populated by `evaluate`.
    /// Returns the gradient + its estimate class (`Exact` in skeleton,
    /// `Noisy { variance }` at Phase G).
    fn gradient(&mut self, theta: &Tensor<f64>, tape: &Tape) -> (Tensor<f64>, GradientEstimate);
}
