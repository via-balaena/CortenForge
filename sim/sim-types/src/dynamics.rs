//! Dynamics types: actions, forces, and commands.
//!
//! This module provides types for representing control inputs to the simulation:
//! forces, torques, joint commands, and higher-level actions.

use crate::{BodyId, JointId};
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An action to be applied to the simulation.
///
/// Actions are the control interface between policies/controllers and the physics.
/// They can range from low-level (direct force application) to high-level (target positions).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Action {
    /// The type of action being taken.
    pub action_type: ActionType,
    /// Timestamp at which this action should be applied (simulation time).
    pub timestamp: f64,
}

impl Action {
    /// Create a new action.
    #[must_use]
    pub fn new(action_type: ActionType, timestamp: f64) -> Self {
        Self {
            action_type,
            timestamp,
        }
    }

    /// Create an action at time 0 (apply immediately).
    #[must_use]
    pub fn immediate(action_type: ActionType) -> Self {
        Self {
            action_type,
            timestamp: 0.0,
        }
    }
}

/// Types of actions that can be applied to the simulation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ActionType {
    /// Apply external forces to bodies.
    ExternalForces(Vec<ExternalForce>),

    /// Send commands to joints.
    JointCommands(Vec<JointCommand>),

    /// Combined action with both forces and joint commands.
    Combined {
        /// External forces to apply.
        forces: Vec<ExternalForce>,
        /// Joint commands to apply.
        joints: Vec<JointCommand>,
    },

    /// No-op action (do nothing).
    NoOp,
}

impl ActionType {
    /// Create an action with a single external force.
    #[must_use]
    pub fn force(force: ExternalForce) -> Self {
        Self::ExternalForces(vec![force])
    }

    /// Create an action with a single joint command.
    #[must_use]
    pub fn joint(command: JointCommand) -> Self {
        Self::JointCommands(vec![command])
    }

    /// Check if this is a no-op action.
    #[must_use]
    pub fn is_noop(&self) -> bool {
        matches!(self, Self::NoOp)
    }
}

/// An external force applied to a rigid body.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ExternalForce {
    /// The body to apply the force to.
    pub body: BodyId,
    /// Force vector in world coordinates (Newtons).
    pub force: Vector3<f64>,
    /// Torque vector in world coordinates (Newton-meters).
    pub torque: Vector3<f64>,
    /// Point of application in world coordinates (for force-induced torque).
    /// If None, force is applied at center of mass.
    pub point: Option<Point3<f64>>,
}

impl ExternalForce {
    /// Create a force applied at center of mass.
    #[must_use]
    pub fn at_com(body: BodyId, force: Vector3<f64>) -> Self {
        Self {
            body,
            force,
            torque: Vector3::zeros(),
            point: None,
        }
    }

    /// Create a torque (no linear force).
    #[must_use]
    pub fn torque_only(body: BodyId, torque: Vector3<f64>) -> Self {
        Self {
            body,
            force: Vector3::zeros(),
            torque,
            point: None,
        }
    }

    /// Create a force applied at a specific point.
    #[must_use]
    pub fn at_point(body: BodyId, force: Vector3<f64>, point: Point3<f64>) -> Self {
        Self {
            body,
            force,
            torque: Vector3::zeros(),
            point: Some(point),
        }
    }

    /// Create both force and torque at center of mass.
    #[must_use]
    pub fn full(body: BodyId, force: Vector3<f64>, torque: Vector3<f64>) -> Self {
        Self {
            body,
            force,
            torque,
            point: None,
        }
    }

    /// Create a zero force (no effect).
    #[must_use]
    pub fn zero(body: BodyId) -> Self {
        Self::at_com(body, Vector3::zeros())
    }

    /// Add another external force (accumulate).
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        // This only makes sense if both are for the same body and same point type
        Self {
            body: self.body,
            force: self.force + other.force,
            torque: self.torque + other.torque,
            point: self.point.or(other.point),
        }
    }

    /// Scale the force and torque by a factor.
    #[must_use]
    pub fn scale(&self, factor: f64) -> Self {
        Self {
            body: self.body,
            force: self.force * factor,
            torque: self.torque * factor,
            point: self.point,
        }
    }

    /// Check if this force is effectively zero.
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.force.norm_squared() < 1e-20 && self.torque.norm_squared() < 1e-20
    }
}

/// A command to a joint actuator.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointCommand {
    /// The joint to command.
    pub joint: JointId,
    /// The command type and value.
    pub command: JointCommandType,
}

impl JointCommand {
    /// Create a position command.
    #[must_use]
    pub fn position(joint: JointId, target: f64) -> Self {
        Self {
            joint,
            command: JointCommandType::Position(target),
        }
    }

    /// Create a velocity command.
    #[must_use]
    pub fn velocity(joint: JointId, target: f64) -> Self {
        Self {
            joint,
            command: JointCommandType::Velocity(target),
        }
    }

    /// Create a torque/force command.
    #[must_use]
    pub fn effort(joint: JointId, value: f64) -> Self {
        Self {
            joint,
            command: JointCommandType::Effort(value),
        }
    }

    /// Create a position-velocity command.
    #[must_use]
    pub fn position_velocity(joint: JointId, position: f64, velocity: f64) -> Self {
        Self {
            joint,
            command: JointCommandType::PositionVelocity { position, velocity },
        }
    }

    /// Create a full PD command with feedforward torque.
    #[must_use]
    pub fn pd_control(
        joint: JointId,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        feedforward: f64,
    ) -> Self {
        Self {
            joint,
            command: JointCommandType::PdControl {
                position,
                velocity,
                kp,
                kd,
                feedforward,
            },
        }
    }
}

/// Types of commands that can be sent to joints.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum JointCommandType {
    /// Target position (servo mode).
    Position(f64),

    /// Target velocity.
    Velocity(f64),

    /// Direct effort (torque for revolute, force for prismatic).
    Effort(f64),

    /// Position and velocity target combined.
    PositionVelocity {
        /// Target position.
        position: f64,
        /// Target velocity.
        velocity: f64,
    },

    /// Full PD control with gains and feedforward.
    PdControl {
        /// Target position.
        position: f64,
        /// Target velocity.
        velocity: f64,
        /// Position gain (stiffness).
        kp: f64,
        /// Velocity gain (damping).
        kd: f64,
        /// Feedforward torque/force.
        feedforward: f64,
    },

    /// Disable actuator (passive joint).
    Disable,
}

impl JointCommandType {
    /// Compute the effort for a PD controller given current state.
    ///
    /// `effort = kp * (target_pos - current_pos) + kd * (target_vel - current_vel) + feedforward`
    #[must_use]
    pub fn compute_pd_effort(&self, current_position: f64, current_velocity: f64) -> f64 {
        match self {
            Self::PdControl {
                position,
                velocity,
                kp,
                kd,
                feedforward,
            } => {
                let pos_error = position - current_position;
                let vel_error = velocity - current_velocity;
                kp * pos_error + kd * vel_error + feedforward
            }
            Self::Position(target) => {
                // Use default gains - this is typically overridden by the simulator
                let kp = 100.0;
                let kd = 10.0;
                let pos_error = target - current_position;
                let vel_error = -current_velocity; // target velocity is 0
                kp * pos_error + kd * vel_error
            }
            Self::Velocity(target) => {
                // Velocity control with default gain
                let kd = 10.0;
                kd * (target - current_velocity)
            }
            Self::Effort(e) => *e,
            Self::PositionVelocity { position, velocity } => {
                let kp = 100.0;
                let kd = 10.0;
                kp * (position - current_position) + kd * (velocity - current_velocity)
            }
            Self::Disable => 0.0,
        }
    }

    /// Check if this is a position-based command.
    #[must_use]
    pub fn is_position_control(&self) -> bool {
        matches!(
            self,
            Self::Position(_) | Self::PositionVelocity { .. } | Self::PdControl { .. }
        )
    }

    /// Check if this is a velocity-based command.
    #[must_use]
    pub fn is_velocity_control(&self) -> bool {
        matches!(self, Self::Velocity(_))
    }

    /// Check if this is a direct effort command.
    #[must_use]
    pub fn is_effort_control(&self) -> bool {
        matches!(self, Self::Effort(_))
    }
}

/// Gravity configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Gravity {
    /// Acceleration due to gravity (m/s²).
    pub acceleration: Vector3<f64>,
}

impl Default for Gravity {
    fn default() -> Self {
        Self::earth()
    }
}

impl Gravity {
    /// Standard Earth gravity (9.81 m/s² in -Z direction).
    #[must_use]
    pub fn earth() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    /// Moon gravity (1.62 m/s² in -Z direction).
    #[must_use]
    pub fn moon() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -1.62),
        }
    }

    /// Mars gravity (3.71 m/s² in -Z direction).
    #[must_use]
    pub fn mars() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -3.71),
        }
    }

    /// Zero gravity (microgravity).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            acceleration: Vector3::zeros(),
        }
    }

    /// Custom gravity vector.
    #[must_use]
    pub fn custom(acceleration: Vector3<f64>) -> Self {
        Self { acceleration }
    }

    /// Compute the gravitational force on a body.
    #[must_use]
    pub fn force_on_mass(&self, mass: f64) -> Vector3<f64> {
        self.acceleration * mass
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_external_force_at_com() {
        let body = BodyId::new(1);
        let force = ExternalForce::at_com(body, Vector3::new(10.0, 0.0, 0.0));

        assert_eq!(force.body, body);
        assert_eq!(force.force.x, 10.0);
        assert!(force.point.is_none());
    }

    #[test]
    fn test_external_force_scale() {
        let body = BodyId::new(1);
        let force = ExternalForce::at_com(body, Vector3::new(10.0, 0.0, 0.0));
        let scaled = force.scale(0.5);

        assert_relative_eq!(scaled.force.x, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_command_pd() {
        let cmd = JointCommand::pd_control(JointId::new(1), 1.0, 0.0, 100.0, 10.0, 5.0);

        if let JointCommandType::PdControl {
            position,
            velocity,
            kp,
            kd,
            feedforward,
        } = cmd.command
        {
            // Current state: position=0, velocity=0
            // Error: pos=1, vel=0
            // Effort = 100*1 + 10*0 + 5 = 105
            let effort = cmd.command.compute_pd_effort(0.0, 0.0);
            assert_relative_eq!(effort, 105.0, epsilon = 1e-10);

            assert_eq!(position, 1.0);
            assert_eq!(velocity, 0.0);
            assert_eq!(kp, 100.0);
            assert_eq!(kd, 10.0);
            assert_eq!(feedforward, 5.0);
        } else {
            panic!("Expected PdControl");
        }
    }

    #[test]
    fn test_gravity() {
        let g = Gravity::earth();
        assert_relative_eq!(g.acceleration.z, -9.81, epsilon = 1e-10);

        let force = g.force_on_mass(2.0);
        assert_relative_eq!(force.z, -19.62, epsilon = 1e-10);
    }

    #[test]
    fn test_action_type() {
        let action = ActionType::force(ExternalForce::at_com(
            BodyId::new(1),
            Vector3::new(1.0, 0.0, 0.0),
        ));

        assert!(!action.is_noop());
        assert!(ActionType::NoOp.is_noop());
    }

    #[test]
    fn test_joint_command_types() {
        let pos_cmd = JointCommandType::Position(1.0);
        assert!(pos_cmd.is_position_control());
        assert!(!pos_cmd.is_velocity_control());
        assert!(!pos_cmd.is_effort_control());

        let vel_cmd = JointCommandType::Velocity(1.0);
        assert!(!vel_cmd.is_position_control());
        assert!(vel_cmd.is_velocity_control());

        let eff_cmd = JointCommandType::Effort(10.0);
        assert!(eff_cmd.is_effort_control());
    }
}
