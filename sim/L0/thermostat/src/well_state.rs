//! Hysteresis-based well-state classification for bistable elements.

/// Hysteresis-based well-state classification for bistable elements.
///
/// Classifies a 1-DOF position into one of three regions: left well,
/// right well, or barrier. The threshold `x_thresh` defines the
/// boundary between well and barrier regions.
///
/// Used by [`IsingLearner`](crate::IsingLearner)'s trajectory scoring to
/// map a continuous bistable position onto a discrete spin, and by the
/// Phase 3+ integration tests. At the Phase 3 central parameters
/// (`κ = λ_r/ω_b = 0.313`), ~69% of zero-crossings are recrossings;
/// hysteresis at `x_thresh = x₀/2` filters these out and recovers the
/// genuine committed-transition rate.
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
    /// Classify a position into a well state given the hysteresis threshold.
    #[must_use]
    pub fn from_position(x: f64, x_thresh: f64) -> Self {
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
