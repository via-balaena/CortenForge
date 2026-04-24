//! `GradientEstimate` — γ-locked exact-or-noisy gradient classifier.
//!
//! Skeleton always returns `Exact` (IFT-adjoint path is closed-form).
//! `Noisy { variance }` activates at Phase G when stochastic-adjoint
//! methods enter the `Differentiable::fd_wrapper` path.

/// Gradient estimate classification. Distinguishes the IFT-adjoint
/// closed-form path from finite-difference / stochastic-adjoint paths
/// that return sampled gradients with variance.
#[derive(Clone, Copy, Debug)]
pub enum GradientEstimate {
    /// Closed-form analytical gradient (IFT-adjoint).
    Exact,
    /// Sampled gradient with per-component variance.
    Noisy {
        /// Per-component noise variance (isotropic for scalar variance).
        variance: f64,
    },
}
