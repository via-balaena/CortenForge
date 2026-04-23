//! `BasicObservable` — CPU-backend readout impl.
//!
//! Associates `type Step = NewtonStep<CpuTape>` and closed-forms the
//! four readout methods. Phase B fills in `reward_breakdown` with the
//! 1-tet NaN-sentinel gap per spec §2.

use sim_ml_chassis::Tensor;

use super::{Observable, PressureField, StressField, TemperatureField};
use crate::readout::RewardBreakdown;
use crate::solver::{CpuTape, NewtonStep};

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

    fn reward_breakdown(&self, _step: &Self::Step, _theta: &Tensor<f64>) -> RewardBreakdown {
        unimplemented!("skeleton phase 2")
    }
}
