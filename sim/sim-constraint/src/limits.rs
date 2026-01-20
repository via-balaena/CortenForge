//! Joint limits and limit enforcement.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Joint position limits.
///
/// Limits can be hard (infinite stiffness) or soft (spring-like enforcement).
/// Soft limits are more stable but allow some penetration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointLimits {
    /// Lower bound (minimum position).
    lower: f64,

    /// Upper bound (maximum position).
    upper: f64,

    /// Stiffness parameters for limit enforcement.
    stiffness: LimitStiffness,

    /// Restitution coefficient when hitting limits (0 = inelastic, 1 = elastic).
    restitution: f64,
}

impl JointLimits {
    /// Create new position limits.
    ///
    /// # Arguments
    ///
    /// * `lower` - Minimum allowed position
    /// * `upper` - Maximum allowed position
    #[must_use]
    pub fn new(lower: f64, upper: f64) -> Self {
        let (lower, upper) = if lower <= upper {
            (lower, upper)
        } else {
            (upper, lower)
        };

        Self {
            lower,
            upper,
            stiffness: LimitStiffness::default(),
            restitution: 0.0,
        }
    }

    /// Create symmetric limits around zero.
    #[must_use]
    pub fn symmetric(bound: f64) -> Self {
        Self::new(-bound.abs(), bound.abs())
    }

    /// Create unlimited joint (no limits).
    #[must_use]
    pub fn unlimited() -> Self {
        Self::new(f64::NEG_INFINITY, f64::INFINITY)
    }

    /// Set limit stiffness.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: LimitStiffness) -> Self {
        self.stiffness = stiffness;
        self
    }

    /// Set soft limits with given stiffness and damping.
    #[must_use]
    pub fn soft(mut self, stiffness: f64, damping: f64) -> Self {
        self.stiffness = LimitStiffness::Soft { stiffness, damping };
        self
    }

    /// Set restitution coefficient.
    #[must_use]
    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Get the lower limit.
    #[must_use]
    pub fn lower(&self) -> f64 {
        self.lower
    }

    /// Get the upper limit.
    #[must_use]
    pub fn upper(&self) -> f64 {
        self.upper
    }

    /// Get the range of motion.
    #[must_use]
    pub fn range(&self) -> f64 {
        self.upper - self.lower
    }

    /// Get the stiffness parameters.
    #[must_use]
    pub fn stiffness(&self) -> &LimitStiffness {
        &self.stiffness
    }

    /// Get restitution coefficient.
    #[must_use]
    pub fn restitution(&self) -> f64 {
        self.restitution
    }

    /// Check if a position is within limits.
    #[must_use]
    pub fn contains(&self, position: f64) -> bool {
        position >= self.lower && position <= self.upper
    }

    /// Check if limits are effectively unlimited.
    #[must_use]
    pub fn is_unlimited(&self) -> bool {
        self.lower == f64::NEG_INFINITY && self.upper == f64::INFINITY
    }

    /// Clamp a position to the limits.
    #[must_use]
    pub fn clamp(&self, position: f64) -> f64 {
        position.clamp(self.lower, self.upper)
    }

    /// Get the current limit state for a position.
    #[must_use]
    pub fn state(&self, position: f64) -> LimitState {
        if position < self.lower {
            LimitState::AtLower(self.lower - position)
        } else if position > self.upper {
            LimitState::AtUpper(position - self.upper)
        } else {
            LimitState::Free
        }
    }

    /// Compute the limit force for a given position and velocity.
    ///
    /// Returns force that pushes the joint back into limits.
    #[must_use]
    pub fn compute_force(&self, position: f64, velocity: f64) -> f64 {
        match self.stiffness {
            LimitStiffness::Hard => {
                // Hard limits are enforced by the constraint solver, not forces
                0.0
            }
            LimitStiffness::Soft { stiffness, damping } => {
                let state = self.state(position);
                match state {
                    LimitState::Free => 0.0,
                    LimitState::AtLower(penetration) => {
                        // Force pushing toward positive (back into range)
                        let spring = stiffness * penetration;
                        let damp = if velocity < 0.0 {
                            -damping * velocity
                        } else {
                            0.0
                        };
                        spring + damp
                    }
                    LimitState::AtUpper(penetration) => {
                        // Force pushing toward negative (back into range)
                        let spring = -stiffness * penetration;
                        let damp = if velocity > 0.0 {
                            -damping * velocity
                        } else {
                            0.0
                        };
                        spring + damp
                    }
                }
            }
        }
    }

    /// Compute the rebound velocity when hitting a limit.
    #[must_use]
    pub fn rebound_velocity(&self, velocity: f64, state: LimitState) -> f64 {
        match state {
            LimitState::AtLower(_) if velocity < 0.0 => -velocity * self.restitution,
            LimitState::AtUpper(_) if velocity > 0.0 => -velocity * self.restitution,
            LimitState::Free | LimitState::AtLower(_) | LimitState::AtUpper(_) => velocity,
        }
    }
}

impl Default for JointLimits {
    fn default() -> Self {
        Self::unlimited()
    }
}

/// Stiffness model for limit enforcement.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LimitStiffness {
    /// Hard limits (infinite stiffness, enforced by constraints).
    #[default]
    Hard,

    /// Soft limits (spring-damper model).
    Soft {
        /// Spring stiffness (N/m or Nm/rad).
        stiffness: f64,
        /// Damping coefficient (Ns/m or Nms/rad).
        damping: f64,
    },
}

impl LimitStiffness {
    /// Create soft limits with critical damping.
    #[must_use]
    pub fn critically_damped(stiffness: f64, effective_mass: f64) -> Self {
        let damping = 2.0 * (stiffness * effective_mass).sqrt();
        Self::Soft { stiffness, damping }
    }

    /// Check if these are hard limits.
    #[must_use]
    pub fn is_hard(&self) -> bool {
        matches!(self, Self::Hard)
    }
}

/// Current state of a joint with respect to its limits.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LimitState {
    /// Joint is within limits.
    Free,

    /// Joint is at or past lower limit.
    /// Contains the penetration depth (positive value).
    AtLower(f64),

    /// Joint is at or past upper limit.
    /// Contains the penetration depth (positive value).
    AtUpper(f64),
}

impl LimitState {
    /// Check if the joint is at any limit.
    #[must_use]
    pub fn is_at_limit(&self) -> bool {
        !matches!(self, Self::Free)
    }

    /// Get the penetration depth (0 if free).
    #[must_use]
    pub fn penetration(&self) -> f64 {
        match self {
            Self::Free => 0.0,
            Self::AtLower(d) | Self::AtUpper(d) => *d,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_limits_creation() {
        let limits = JointLimits::new(-1.0, 1.0);
        assert_relative_eq!(limits.lower(), -1.0, epsilon = 1e-10);
        assert_relative_eq!(limits.upper(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(limits.range(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_limits_swapped_order() {
        // Should auto-correct swapped limits
        let limits = JointLimits::new(1.0, -1.0);
        assert_relative_eq!(limits.lower(), -1.0, epsilon = 1e-10);
        assert_relative_eq!(limits.upper(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_symmetric_limits() {
        let limits = JointLimits::symmetric(2.0);
        assert_relative_eq!(limits.lower(), -2.0, epsilon = 1e-10);
        assert_relative_eq!(limits.upper(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_unlimited() {
        let limits = JointLimits::unlimited();
        assert!(limits.is_unlimited());
        assert!(limits.contains(1e100));
        assert!(limits.contains(-1e100));
    }

    #[test]
    fn test_contains() {
        let limits = JointLimits::new(-1.0, 1.0);
        assert!(limits.contains(0.0));
        assert!(limits.contains(-1.0));
        assert!(limits.contains(1.0));
        assert!(!limits.contains(-1.5));
        assert!(!limits.contains(1.5));
    }

    #[test]
    fn test_clamp() {
        let limits = JointLimits::new(-1.0, 1.0);
        assert_relative_eq!(limits.clamp(0.5), 0.5, epsilon = 1e-10);
        assert_relative_eq!(limits.clamp(-2.0), -1.0, epsilon = 1e-10);
        assert_relative_eq!(limits.clamp(2.0), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_limit_state() {
        let limits = JointLimits::new(-1.0, 1.0);

        assert!(matches!(limits.state(0.0), LimitState::Free));
        assert!(matches!(limits.state(-1.5), LimitState::AtLower(_)));
        assert!(matches!(limits.state(1.5), LimitState::AtUpper(_)));

        if let LimitState::AtLower(pen) = limits.state(-1.5) {
            assert_relative_eq!(pen, 0.5, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_soft_limit_force() {
        let limits = JointLimits::new(-1.0, 1.0).soft(1000.0, 100.0);

        // Within limits - no force
        assert_relative_eq!(limits.compute_force(0.0, 0.0), 0.0, epsilon = 1e-10);

        // At lower limit - positive force (pushes back)
        let force = limits.compute_force(-1.1, 0.0);
        assert!(force > 0.0);

        // At upper limit - negative force (pushes back)
        let force = limits.compute_force(1.1, 0.0);
        assert!(force < 0.0);
    }

    #[test]
    fn test_restitution() {
        let limits = JointLimits::new(-1.0, 1.0).with_restitution(0.5);

        // Hitting lower limit with negative velocity
        let new_vel = limits.rebound_velocity(-2.0, LimitState::AtLower(0.1));
        assert_relative_eq!(new_vel, 1.0, epsilon = 1e-10); // 0.5 * 2.0

        // Hitting upper limit with positive velocity
        let new_vel = limits.rebound_velocity(2.0, LimitState::AtUpper(0.1));
        assert_relative_eq!(new_vel, -1.0, epsilon = 1e-10);

        // Moving away from limit - no rebound
        let new_vel = limits.rebound_velocity(2.0, LimitState::AtLower(0.1));
        assert_relative_eq!(new_vel, 2.0, epsilon = 1e-10);
    }
}
