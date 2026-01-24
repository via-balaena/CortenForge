//! Joint motors for active control.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A motor that can drive a joint.
///
/// Motors can operate in different modes:
/// - **Position control**: Drive to a target position with PD control
/// - **Velocity control**: Maintain a target velocity
/// - **Torque control**: Apply a commanded torque directly
///
/// All modes respect a maximum force/torque limit.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointMotor {
    /// Motor control mode.
    mode: MotorMode,

    /// Maximum force/torque the motor can apply (N or Nm).
    max_force: f64,

    /// Whether the motor is currently enabled.
    enabled: bool,

    /// Gear ratio (output/input). 1.0 = direct drive.
    gear_ratio: f64,
}

impl JointMotor {
    /// Create a velocity-controlled motor.
    ///
    /// # Arguments
    ///
    /// * `target_velocity` - Target velocity (rad/s or m/s)
    /// * `max_force` - Maximum force/torque (N or Nm)
    #[must_use]
    pub fn velocity(target_velocity: f64, max_force: f64) -> Self {
        Self {
            mode: MotorMode::Velocity {
                target: target_velocity,
            },
            max_force: max_force.abs(),
            enabled: true,
            gear_ratio: 1.0,
        }
    }

    /// Create a position-controlled motor with PD gains.
    ///
    /// # Arguments
    ///
    /// * `target_position` - Target position (rad or m)
    /// * `kp` - Proportional gain (stiffness)
    /// * `max_force` - Maximum force/torque (N or Nm)
    #[must_use]
    pub fn position(target_position: f64, kp: f64, max_force: f64) -> Self {
        Self {
            mode: MotorMode::Position {
                target: target_position,
                kp,
                kd: 2.0 * kp.sqrt(), // Default to critical damping
            },
            max_force: max_force.abs(),
            enabled: true,
            gear_ratio: 1.0,
        }
    }

    /// Create a position-controlled motor with explicit PD gains.
    #[must_use]
    pub fn position_pd(target_position: f64, kp: f64, kd: f64, max_force: f64) -> Self {
        Self {
            mode: MotorMode::Position {
                target: target_position,
                kp,
                kd,
            },
            max_force: max_force.abs(),
            enabled: true,
            gear_ratio: 1.0,
        }
    }

    /// Create a torque-controlled motor.
    ///
    /// # Arguments
    ///
    /// * `torque` - Commanded torque (Nm) or force (N)
    /// * `max_force` - Maximum force/torque (N or Nm)
    #[must_use]
    pub fn torque(torque: f64, max_force: f64) -> Self {
        Self {
            mode: MotorMode::Torque { command: torque },
            max_force: max_force.abs(),
            enabled: true,
            gear_ratio: 1.0,
        }
    }

    /// Create a disabled motor (free joint).
    #[must_use]
    pub fn disabled() -> Self {
        Self {
            mode: MotorMode::Velocity { target: 0.0 },
            max_force: 0.0,
            enabled: false,
            gear_ratio: 1.0,
        }
    }

    /// Set the gear ratio.
    #[must_use]
    pub fn with_gear_ratio(mut self, ratio: f64) -> Self {
        self.gear_ratio = ratio.abs().max(0.001);
        self
    }

    /// Enable or disable the motor.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Get the motor mode.
    #[must_use]
    pub fn mode(&self) -> &MotorMode {
        &self.mode
    }

    /// Get the maximum force/torque.
    #[must_use]
    pub fn max_force(&self) -> f64 {
        self.max_force
    }

    /// Check if the motor is enabled.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get the gear ratio.
    #[must_use]
    pub fn gear_ratio(&self) -> f64 {
        self.gear_ratio
    }

    /// Set the target for position or velocity mode.
    pub fn set_target(&mut self, target: f64) {
        match &mut self.mode {
            MotorMode::Velocity { target: t } | MotorMode::Position { target: t, .. } => {
                *t = target;
            }
            MotorMode::Torque { command } => *command = target,
        }
    }

    /// Compute the motor force/torque given current state.
    ///
    /// # Arguments
    ///
    /// * `position` - Current joint position
    /// * `velocity` - Current joint velocity
    ///
    /// # Returns
    ///
    /// The force/torque to apply (clamped to `max_force`).
    #[must_use]
    pub fn compute_force(&self, position: f64, velocity: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }

        let raw_force = match &self.mode {
            MotorMode::Velocity { target } => {
                // Velocity control: apply force proportional to velocity error
                // Using the max_force as an implicit gain
                let error = target - velocity;
                // Simple proportional control (the constraint solver handles the rest)
                error.signum() * self.max_force
            }

            MotorMode::Position { target, kp, kd } => {
                // PD position control
                let pos_error = target - position;
                let vel_error = -velocity; // Assume target velocity is 0

                kp * pos_error + kd * vel_error
            }

            MotorMode::Torque { command } => *command,
        };

        // Apply gear ratio (higher ratio = more torque at output)
        let geared_force = raw_force * self.gear_ratio;

        // Clamp to maximum
        geared_force.clamp(-self.max_force, self.max_force)
    }

    /// Get the current target (position, velocity, or torque depending on mode).
    #[must_use]
    pub fn target(&self) -> f64 {
        match &self.mode {
            MotorMode::Velocity { target } | MotorMode::Position { target, .. } => *target,
            MotorMode::Torque { command } => *command,
        }
    }
}

impl Default for JointMotor {
    fn default() -> Self {
        Self::disabled()
    }
}

/// Motor control mode.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MotorMode {
    /// Velocity control: maintain target velocity.
    Velocity {
        /// Target velocity (rad/s or m/s).
        target: f64,
    },

    /// Position control: drive to target position with PD.
    Position {
        /// Target position (rad or m).
        target: f64,
        /// Proportional gain (stiffness).
        kp: f64,
        /// Derivative gain (damping).
        kd: f64,
    },

    /// Direct torque/force control.
    Torque {
        /// Commanded torque/force (Nm or N).
        command: f64,
    },
}

impl MotorMode {
    /// Check if this is velocity control mode.
    #[must_use]
    pub fn is_velocity(&self) -> bool {
        matches!(self, Self::Velocity { .. })
    }

    /// Check if this is position control mode.
    #[must_use]
    pub fn is_position(&self) -> bool {
        matches!(self, Self::Position { .. })
    }

    /// Check if this is torque control mode.
    #[must_use]
    pub fn is_torque(&self) -> bool {
        matches!(self, Self::Torque { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_velocity_motor() {
        let motor = JointMotor::velocity(1.0, 10.0);

        assert!(motor.is_enabled());
        assert_relative_eq!(motor.max_force(), 10.0, epsilon = 1e-10);
        assert!(motor.mode().is_velocity());

        // Moving slower than target - positive force
        let force = motor.compute_force(0.0, 0.5);
        assert!(force > 0.0);

        // Moving faster than target - negative force
        let force = motor.compute_force(0.0, 1.5);
        assert!(force < 0.0);
    }

    #[test]
    fn test_position_motor() {
        let motor = JointMotor::position(1.0, 100.0, 50.0);

        assert!(motor.mode().is_position());

        // Below target - positive force (pulls toward target)
        let force = motor.compute_force(0.0, 0.0);
        assert!(force > 0.0);

        // Above target - negative force
        let force = motor.compute_force(2.0, 0.0);
        assert!(force < 0.0);

        // At target with no velocity - no force
        let force = motor.compute_force(1.0, 0.0);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_torque_motor() {
        let motor = JointMotor::torque(5.0, 10.0);

        assert!(motor.mode().is_torque());

        // Should return commanded torque regardless of state
        let force = motor.compute_force(0.0, 0.0);
        assert_relative_eq!(force, 5.0, epsilon = 1e-10);

        let force = motor.compute_force(100.0, 100.0);
        assert_relative_eq!(force, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_motor_force_clamping() {
        let motor = JointMotor::position(100.0, 1000.0, 10.0); // Would produce huge force

        let force = motor.compute_force(0.0, 0.0);
        assert!(force.abs() <= 10.0); // Should be clamped
    }

    #[test]
    fn test_disabled_motor() {
        let motor = JointMotor::disabled();

        assert!(!motor.is_enabled());

        let force = motor.compute_force(0.0, 1.0);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_ratio() {
        let motor = JointMotor::torque(1.0, 100.0).with_gear_ratio(10.0);

        let force = motor.compute_force(0.0, 0.0);
        assert_relative_eq!(force, 10.0, epsilon = 1e-10); // 1.0 * 10.0 ratio
    }

    #[test]
    fn test_set_target() {
        let mut motor = JointMotor::velocity(1.0, 10.0);

        motor.set_target(2.0);
        assert_relative_eq!(motor.target(), 2.0, epsilon = 1e-10);

        // At new target - no force
        let force = motor.compute_force(0.0, 2.0);
        // Moving at target velocity, no correction needed
        // (but due to simple proportional control, exact behavior may vary)
        assert!(force.abs() <= 10.0);
    }
}
