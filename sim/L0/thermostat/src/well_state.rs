//! Threshold classification of a bistable element's position.

use crate::params::{Domain, or_panic};

/// Which region of a double well a position is in: the left well, the right
/// well, or the barrier between them, split at `±x_thresh`.
///
/// The classification is stateless: [`Self::from_position`] looks at one
/// position. A caller that keeps an element's last well while it is in the
/// barrier gets a dead band of width `2·x_thresh`, so a trajectory that
/// recrosses the barrier top without reaching the other well does not count
/// as a switch.
///
/// [`IsingLearner`](crate::IsingLearner) and [`SpinLatch`](crate::SpinLatch)
/// read spins this way, and the crate's tests use `x_thresh = x₀/2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WellState {
    /// Position is in the left well: `x < −x_thresh`.
    Left,
    /// Position is in the right well: `x > +x_thresh`.
    Right,
    /// Position is in the barrier region: `−x_thresh ≤ x ≤ +x_thresh`.
    Barrier,
}

impl WellState {
    /// Classify position `x` with threshold `x_thresh`. A `NaN` position
    /// reads as the barrier.
    ///
    /// # Panics
    /// Unless `x_thresh` is finite and non-negative: a negative threshold would
    /// leave no barrier and classify `0.0` as a well.
    #[must_use]
    #[track_caller]
    pub fn from_position(x: f64, x_thresh: f64) -> Self {
        or_panic(Domain::NonNegative.check("WellState", "x_thresh", x_thresh));
        if x > x_thresh {
            Self::Right
        } else if x < -x_thresh {
            Self::Left
        } else {
            Self::Barrier
        }
    }

    /// Convert to a spin value: `+1.0` for Right, `−1.0` for Left.
    ///
    /// # Panics
    /// Panics if called on `Barrier` — callers must check
    /// [`is_in_well`](Self::is_in_well) first.
    #[must_use]
    // Panic on Barrier is a deliberate contract — callers must check is_in_well() first.
    #[allow(clippy::panic)]
    pub fn spin(self) -> f64 {
        match self {
            Self::Right => 1.0,
            Self::Left => -1.0,
            Self::Barrier => panic!("spin() called on Barrier state"), // deliberate contract violation
        }
    }

    /// The spin, or `None` in the barrier.
    #[must_use]
    pub const fn checked_spin(self) -> Option<f64> {
        match self {
            Self::Right => Some(1.0),
            Self::Left => Some(-1.0),
            Self::Barrier => None,
        }
    }

    /// Returns `true` if the element is in a well (not in the barrier).
    #[must_use]
    pub fn is_in_well(self) -> bool {
        self != Self::Barrier
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    #[should_panic(expected = "WellState: x_thresh must be finite and non-negative, got -0.1")]
    fn from_position_refuses_a_negative_threshold() {
        let _state = WellState::from_position(0.0, -0.1);
    }

    #[test]
    fn checked_spin_is_none_in_the_barrier() {
        assert_eq!(WellState::Right.checked_spin(), Some(1.0));
        assert_eq!(WellState::Left.checked_spin(), Some(-1.0));
        assert_eq!(WellState::Barrier.checked_spin(), None);
        assert_eq!(WellState::from_position(f64::NAN, 0.5), WellState::Barrier);
    }

    #[test]
    fn well_state_from_position_classifies_correctly() {
        let thresh = 0.5;
        assert_eq!(WellState::from_position(1.0, thresh), WellState::Right);
        assert_eq!(WellState::from_position(-1.0, thresh), WellState::Left);
        assert_eq!(WellState::from_position(0.0, thresh), WellState::Barrier);
        assert_eq!(WellState::from_position(0.5, thresh), WellState::Barrier);
        assert_eq!(WellState::from_position(-0.5, thresh), WellState::Barrier);
        assert_eq!(
            WellState::from_position(0.500_001, thresh),
            WellState::Right
        );
        assert_eq!(
            WellState::from_position(-0.500_001, thresh),
            WellState::Left
        );
    }

    #[test]
    fn well_state_spin_values() {
        assert_eq!(WellState::Right.spin(), 1.0);
        assert_eq!(WellState::Left.spin(), -1.0);
    }

    #[test]
    #[should_panic(expected = "spin() called on Barrier")]
    fn well_state_spin_panics_on_barrier() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = WellState::Barrier.spin();
    }

    #[test]
    fn well_state_is_in_well() {
        assert!(WellState::Right.is_in_well());
        assert!(WellState::Left.is_in_well());
        assert!(!WellState::Barrier.is_in_well());
    }
}
