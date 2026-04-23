//! `Observable` trait — readout surface for forward-pass telemetry.
//!
//! Four items: stress, pressure, temperature fields, and the γ-locked
//! `reward_breakdown`. Associated `Step` type threads the active
//! solver's `NewtonStep<Tape>` into readout code. Skeleton ships
//! `BasicObservable` (impl `type Step = NewtonStep<CpuTape>`).

use sim_ml_chassis::Tensor;

use crate::readout::RewardBreakdown;

pub mod basic;

pub use basic::BasicObservable;

/// Per-Gauss-point Cauchy stress samples. Unit stub; Phase B carries
/// a `9 x n_gauss_points` sample grid.
#[derive(Clone, Debug, Default)]
pub struct StressField;

/// Per-Gauss-point hydrostatic pressure (negative one-third trace of
/// Cauchy stress). Unit stub; Phase B carries a scalar sample grid.
#[derive(Clone, Debug, Default)]
pub struct PressureField;

/// Per-vertex temperature. Empty for skeleton; thermal coupling is
/// Phase H per spec §8.
#[derive(Clone, Debug, Default)]
pub struct TemperatureField;

/// Readout surface. Associated `Step` binds the concrete
/// `NewtonStep<Tape>` so CPU and GPU backends each thread their own
/// tape type through readout code.
pub trait Observable {
    /// Per-step record consumed by readout methods.
    type Step;

    /// Per-Gauss-point Cauchy stress samples.
    fn stress_field(&self, step: &Self::Step) -> StressField;

    /// Per-Gauss-point hydrostatic pressure.
    fn pressure_field(&self, step: &Self::Step) -> PressureField;

    /// Per-vertex temperature. Empty unless thermal coupling is active.
    fn temperature_field(&self, step: &Self::Step) -> TemperatureField;

    /// γ-locked reward breakdown per [Part 10 Ch 00](../../100-optimization/00-forward.md).
    /// Four per-term fields; skeleton 1-tet gap is encoded as
    /// `f64::NAN` sentinels on `pressure_uniformity` + `coverage` per
    /// spec §2.
    fn reward_breakdown(&self, step: &Self::Step, theta: &Tensor<f64>) -> RewardBreakdown;
}
