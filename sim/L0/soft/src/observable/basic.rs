//! `BasicObservable` — CPU-backend readout impl.
//!
//! Associates `type Step = NewtonStep<CpuTape>` and closed-forms the
//! four readout methods. Phase B step 6 filled in `reward_breakdown`
//! with the 1-tet NaN-sentinel gap per spec §2; `stress_field`,
//! `pressure_field`, and `temperature_field` remain Phase-C/+
//! deferrals.

use sim_ml_chassis::Tensor;

use super::{Observable, PressureField, StressField, TemperatureField};
use crate::readout::RewardBreakdown;
use crate::solver::{CpuTape, NewtonStep};

/// Free-DOF index of `v_3.z` — the only DOF that moves on the skeleton
/// scene, and the one the reward bounds are anchored on. Duplicated
/// from `solver::backward_euler` / `differentiable::newton_vjp` to keep
/// readout self-contained.
const FREE_DOF_Z: usize = 11;

/// CPU-backend `Observable` impl.
#[derive(Clone, Copy, Debug, Default)]
pub struct BasicObservable;

impl Observable for BasicObservable {
    type Step = NewtonStep<CpuTape>;

    fn stress_field(&self, _step: &Self::Step) -> StressField {
        unimplemented!("skeleton phase 2")
    }

    fn pressure_field(&self, _step: &Self::Step) -> PressureField {
        unimplemented!("skeleton phase 2")
    }

    fn temperature_field(&self, _step: &Self::Step) -> TemperatureField {
        unimplemented!("skeleton phase 2")
    }

    /// Skeleton 1-tet `RewardBreakdown` per scope §2.
    ///
    /// - `pressure_uniformity` and `coverage`: `f64::NAN` sentinel —
    ///   structurally undefined on a single tet with no contact pair.
    ///   `RewardBreakdown::score_with` silently drops `NaN` fields.
    /// - `peak_bound`: free-DOF peak displacement `x_final[11]`
    ///   (SQ1 option α). Matches the step-5 gradcheck scalar directly,
    ///   so the step-6 composition adds division-chain coverage on top
    ///   without changing the leading term.
    /// - `stiffness_bound`: `θ[0] / x_final[11]` — effective spring
    ///   constant at the operating point. Exposes the `DivOp` chain to
    ///   backward; sign follows `x_final[11]` (positive under tensile
    ///   +ẑ traction at θ=10 N).
    #[allow(clippy::panic)]
    fn reward_breakdown(&self, step: &Self::Step, theta: &Tensor<f64>) -> RewardBreakdown {
        assert!(
            step.x_final.len() > FREE_DOF_Z,
            "BasicObservable: step.x_final has {} DOFs; skeleton scene must have at least {}",
            step.x_final.len(),
            FREE_DOF_Z + 1,
        );
        assert!(
            theta.shape() == [1],
            "BasicObservable: Stage-1 θ must have shape [1], got {:?}",
            theta.shape(),
        );

        let x_peak = step.x_final[FREE_DOF_Z];
        let theta_val = theta.as_slice()[0];

        RewardBreakdown {
            pressure_uniformity: f64::NAN,
            coverage: f64::NAN,
            peak_bound: x_peak,
            stiffness_bound: theta_val / x_peak,
        }
    }
}
