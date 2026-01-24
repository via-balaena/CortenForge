//! Muscle-tendon kinematics and moment arm models.
//!
//! This module provides tools for computing:
//! - Muscle-tendon unit length from joint angles
//! - Moment arms (lever arms) at joints
//! - Path length through multiple joints (for biarticular muscles)
//!
//! # Moment Arm Models
//!
//! The moment arm determines how muscle force converts to joint torque:
//!
//! ```text
//! τ = F * r(θ)
//! ```
//!
//! Where `r(θ)` is the moment arm, which may vary with joint angle.
//!
//! ## Available Models
//!
//! - [`ConstantMomentArm`]: Fixed moment arm (simplest)
//! - [`PolynomialMomentArm`]: Polynomial function of joint angle
//! - [`SplineMomentArm`]: Spline interpolation from measured data

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Trait for computing moment arm at a given joint angle.
pub trait MomentArmModel {
    /// Compute the moment arm at the given joint angle.
    ///
    /// # Arguments
    ///
    /// * `joint_angle` - Joint angle in radians
    ///
    /// # Returns
    ///
    /// Moment arm in meters.
    fn moment_arm(&self, joint_angle: f64) -> f64;

    /// Compute the derivative of moment arm with respect to joint angle.
    ///
    /// Useful for computing muscle velocity from joint velocity:
    /// v_muscle = -(dL/dθ) * θ_dot = r(θ) * θ_dot
    fn moment_arm_derivative(&self, joint_angle: f64) -> f64;

    /// Compute muscle-tendon length change from reference.
    ///
    /// # Arguments
    ///
    /// * `joint_angle` - Current joint angle
    /// * `reference_angle` - Reference angle (usually 0)
    ///
    /// # Returns
    ///
    /// Change in muscle-tendon length (negative = shorter).
    fn length_change(&self, joint_angle: f64, reference_angle: f64) -> f64;
}

/// Constant moment arm model.
///
/// The simplest model where moment arm doesn't change with joint angle.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConstantMomentArm {
    /// Moment arm value (m).
    pub value: f64,
}

impl ConstantMomentArm {
    /// Create a new constant moment arm.
    #[must_use]
    pub fn new(value: f64) -> Self {
        Self { value }
    }
}

impl MomentArmModel for ConstantMomentArm {
    fn moment_arm(&self, _joint_angle: f64) -> f64 {
        self.value
    }

    fn moment_arm_derivative(&self, _joint_angle: f64) -> f64 {
        0.0 // Constant
    }

    fn length_change(&self, joint_angle: f64, reference_angle: f64) -> f64 {
        -self.value * (joint_angle - reference_angle)
    }
}

/// Polynomial moment arm model.
///
/// Models moment arm as a polynomial function of joint angle:
///
/// ```text
/// r(θ) = c₀ + c₁θ + c₂θ² + c₃θ³ + ...
/// ```
///
/// # Example
///
/// ```
/// use sim_muscle::kinematics::{PolynomialMomentArm, MomentArmModel};
///
/// // Quadratic moment arm: r = 0.05 - 0.01*θ + 0.005*θ²
/// let model = PolynomialMomentArm::new(vec![0.05, -0.01, 0.005]);
/// let r = model.moment_arm(0.5); // Evaluate at θ = 0.5 rad
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PolynomialMomentArm {
    /// Polynomial coefficients [c₀, c₁, c₂, ...].
    coefficients: Vec<f64>,
}

impl PolynomialMomentArm {
    /// Create a new polynomial moment arm model.
    #[must_use]
    pub fn new(coefficients: Vec<f64>) -> Self {
        Self { coefficients }
    }

    /// Create a linear moment arm model: r = r₀ + r₁*θ.
    #[must_use]
    pub fn linear(r0: f64, r1: f64) -> Self {
        Self::new(vec![r0, r1])
    }

    /// Create a quadratic moment arm model: r = r₀ + r₁*θ + r₂*θ².
    #[must_use]
    pub fn quadratic(r0: f64, r1: f64, r2: f64) -> Self {
        Self::new(vec![r0, r1, r2])
    }
}

impl MomentArmModel for PolynomialMomentArm {
    fn moment_arm(&self, joint_angle: f64) -> f64 {
        let mut result = 0.0;
        let mut power = 1.0;

        for coeff in &self.coefficients {
            result += coeff * power;
            power *= joint_angle;
        }

        result
    }

    fn moment_arm_derivative(&self, joint_angle: f64) -> f64 {
        if self.coefficients.len() < 2 {
            return 0.0;
        }

        let mut result = 0.0;
        let mut power = 1.0;

        for (i, coeff) in self.coefficients.iter().enumerate().skip(1) {
            result += coeff * (i as f64) * power;
            power *= joint_angle;
        }

        result
    }

    fn length_change(&self, joint_angle: f64, reference_angle: f64) -> f64 {
        // Integrate r(θ) from reference to current angle
        // For polynomial: ∫(c₀ + c₁θ + c₂θ² + ...) dθ
        // = c₀θ + (c₁/2)θ² + (c₂/3)θ³ + ...
        let integrate = |theta: f64| -> f64 {
            let mut result = 0.0;
            let mut power = theta;

            for (i, coeff) in self.coefficients.iter().enumerate() {
                result += coeff * power / ((i + 1) as f64);
                power *= theta;
            }

            result
        };

        -(integrate(joint_angle) - integrate(reference_angle))
    }
}

/// Piecewise linear (spline) moment arm model.
///
/// Interpolates moment arm values from measured data points.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SplineMomentArm {
    /// Joint angles at data points (sorted, ascending).
    angles: Vec<f64>,

    /// Moment arm values at data points.
    values: Vec<f64>,
}

impl SplineMomentArm {
    /// Create a new spline moment arm model from data points.
    ///
    /// # Panics
    ///
    /// Panics if angles and values have different lengths or if empty.
    #[must_use]
    pub fn new(angles: Vec<f64>, values: Vec<f64>) -> Self {
        assert_eq!(
            angles.len(),
            values.len(),
            "angles and values must have same length"
        );
        assert!(!angles.is_empty(), "must have at least one data point");

        Self { angles, values }
    }

    /// Create from a slice of (angle, value) pairs.
    #[must_use]
    pub fn from_pairs(data: &[(f64, f64)]) -> Self {
        let (angles, values): (Vec<_>, Vec<_>) = data.iter().copied().unzip();
        Self::new(angles, values)
    }

    fn find_segment(&self, joint_angle: f64) -> (usize, f64) {
        // Find segment containing joint_angle
        for i in 0..self.angles.len() - 1 {
            if joint_angle >= self.angles[i] && joint_angle <= self.angles[i + 1] {
                let t = (joint_angle - self.angles[i]) / (self.angles[i + 1] - self.angles[i]);
                return (i, t);
            }
        }

        // Extrapolate
        if joint_angle < self.angles[0] {
            (
                0,
                (joint_angle - self.angles[0]) / (self.angles[1] - self.angles[0]),
            )
        } else {
            let n = self.angles.len();
            (
                n - 2,
                (joint_angle - self.angles[n - 2]) / (self.angles[n - 1] - self.angles[n - 2]),
            )
        }
    }
}

impl MomentArmModel for SplineMomentArm {
    fn moment_arm(&self, joint_angle: f64) -> f64 {
        if self.angles.len() == 1 {
            return self.values[0];
        }

        let (i, t) = self.find_segment(joint_angle);

        // Linear interpolation
        self.values[i] * (1.0 - t) + self.values[i + 1] * t
    }

    fn moment_arm_derivative(&self, joint_angle: f64) -> f64 {
        if self.angles.len() == 1 {
            return 0.0;
        }

        let (i, _) = self.find_segment(joint_angle);

        // Slope of segment
        (self.values[i + 1] - self.values[i]) / (self.angles[i + 1] - self.angles[i])
    }

    fn length_change(&self, joint_angle: f64, reference_angle: f64) -> f64 {
        // Numerical integration using trapezoidal rule
        let n_steps = 20;
        let step = (joint_angle - reference_angle) / (n_steps as f64);

        let mut sum = 0.0;
        let mut prev_r = self.moment_arm(reference_angle);

        for i in 1..=n_steps {
            let theta = reference_angle + (i as f64) * step;
            let r = self.moment_arm(theta);
            sum += (prev_r + r) * 0.5 * step;
            prev_r = r;
        }

        -sum
    }
}

/// Muscle path geometry for computing muscle length.
///
/// Handles wrapping around bones and through via points.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MusclePath {
    /// Origin point (insertion on proximal bone).
    pub origin: Vector3<f64>,

    /// Insertion point (on distal bone).
    pub insertion: Vector3<f64>,

    /// Via points (intermediate points the muscle passes through).
    pub via_points: Vec<ViaPoint>,
}

/// A via point along the muscle path.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ViaPoint {
    /// Position in local body frame.
    pub position: Vector3<f64>,

    /// Body index this via point is attached to.
    pub body_index: usize,
}

impl MusclePath {
    /// Create a simple straight-line muscle path.
    #[must_use]
    pub fn straight(origin: Vector3<f64>, insertion: Vector3<f64>) -> Self {
        Self {
            origin,
            insertion,
            via_points: Vec::new(),
        }
    }

    /// Add a via point to the path.
    #[must_use]
    pub fn with_via_point(mut self, position: Vector3<f64>, body_index: usize) -> Self {
        self.via_points.push(ViaPoint {
            position,
            body_index,
        });
        self
    }

    /// Compute the total path length (sum of segment lengths).
    ///
    /// # Arguments
    ///
    /// * `transforms` - Transform for each body (world position of body frames)
    ///
    /// # Returns
    ///
    /// Total muscle-tendon path length.
    #[must_use]
    pub fn compute_length(&self, _transforms: &[nalgebra::Isometry3<f64>]) -> f64 {
        // Simplified: just compute straight-line distance for now
        // Full implementation would transform via points through body transforms
        (self.insertion - self.origin).norm()
    }
}

/// Configuration for a biarticular (two-joint) muscle.
///
/// Biarticular muscles span two joints and contribute to coordination
/// between joints (e.g., rectus femoris crosses hip and knee).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BiarticularlMuscleConfig {
    /// Moment arm at the proximal joint.
    pub proximal_moment_arm: f64,

    /// Moment arm at the distal joint.
    pub distal_moment_arm: f64,

    /// Reference muscle-tendon length at zero joint angles.
    pub reference_length: f64,
}

impl BiarticularlMuscleConfig {
    /// Compute muscle-tendon length from two joint angles.
    ///
    /// # Arguments
    ///
    /// * `proximal_angle` - Angle of proximal joint (radians)
    /// * `distal_angle` - Angle of distal joint (radians)
    ///
    /// # Returns
    ///
    /// Muscle-tendon unit length (m).
    #[must_use]
    pub fn length(&self, proximal_angle: f64, distal_angle: f64) -> f64 {
        self.reference_length
            - self.proximal_moment_arm * proximal_angle
            - self.distal_moment_arm * distal_angle
    }

    /// Compute muscle velocity from joint velocities.
    ///
    /// # Arguments
    ///
    /// * `proximal_velocity` - Angular velocity of proximal joint (rad/s)
    /// * `distal_velocity` - Angular velocity of distal joint (rad/s)
    ///
    /// # Returns
    ///
    /// Muscle-tendon velocity (m/s), negative = shortening.
    #[must_use]
    pub fn velocity(&self, proximal_velocity: f64, distal_velocity: f64) -> f64 {
        -self.proximal_moment_arm * proximal_velocity - self.distal_moment_arm * distal_velocity
    }

    /// Compute joint torques from muscle force.
    ///
    /// # Arguments
    ///
    /// * `muscle_force` - Force produced by the muscle (N)
    ///
    /// # Returns
    ///
    /// (proximal_torque, distal_torque) in Nm.
    #[must_use]
    pub fn torques(&self, muscle_force: f64) -> (f64, f64) {
        (
            muscle_force * self.proximal_moment_arm,
            muscle_force * self.distal_moment_arm,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_constant_moment_arm() {
        let model = ConstantMomentArm::new(0.05);

        assert_relative_eq!(model.moment_arm(0.0), 0.05, epsilon = 1e-10);
        assert_relative_eq!(model.moment_arm(1.0), 0.05, epsilon = 1e-10);
        assert_relative_eq!(model.moment_arm_derivative(0.5), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_constant_length_change() {
        let model = ConstantMomentArm::new(0.05);

        // Positive angle = muscle shortens
        let dl = model.length_change(0.5, 0.0);
        assert_relative_eq!(dl, -0.025, epsilon = 1e-10);
    }

    #[test]
    fn test_polynomial_moment_arm() {
        // r(θ) = 0.05 + 0.01*θ
        let model = PolynomialMomentArm::linear(0.05, 0.01);

        assert_relative_eq!(model.moment_arm(0.0), 0.05, epsilon = 1e-10);
        assert_relative_eq!(model.moment_arm(1.0), 0.06, epsilon = 1e-10);
        assert_relative_eq!(model.moment_arm_derivative(0.0), 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_quadratic_moment_arm() {
        // r(θ) = 0.05 + 0.01*θ - 0.005*θ²
        let model = PolynomialMomentArm::quadratic(0.05, 0.01, -0.005);

        let r_at_1 = 0.05 + 0.01 - 0.005;
        assert_relative_eq!(model.moment_arm(1.0), r_at_1, epsilon = 1e-10);

        // Derivative: r'(θ) = 0.01 - 0.01*θ
        let dr_at_1 = 0.01 - 0.01;
        assert_relative_eq!(model.moment_arm_derivative(1.0), dr_at_1, epsilon = 1e-10);
    }

    #[test]
    fn test_spline_moment_arm() {
        let model = SplineMomentArm::from_pairs(&[(-1.0, 0.04), (0.0, 0.05), (1.0, 0.04)]);

        // At data points
        assert_relative_eq!(model.moment_arm(0.0), 0.05, epsilon = 1e-10);
        assert_relative_eq!(model.moment_arm(1.0), 0.04, epsilon = 1e-10);

        // Interpolated
        assert_relative_eq!(model.moment_arm(0.5), 0.045, epsilon = 1e-10);
    }

    #[test]
    fn test_biarticular_muscle() {
        let config = BiarticularlMuscleConfig {
            proximal_moment_arm: 0.04,
            distal_moment_arm: 0.03,
            reference_length: 0.35,
        };

        // At zero angles
        assert_relative_eq!(config.length(0.0, 0.0), 0.35, epsilon = 1e-10);

        // Both joints flexed shortens the muscle
        let len = config.length(0.5, 0.5);
        assert!(len < 0.35);

        // Torques
        let (t1, t2) = config.torques(100.0);
        assert_relative_eq!(t1, 4.0, epsilon = 1e-10);
        assert_relative_eq!(t2, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_muscle_path_straight() {
        let path = MusclePath::straight(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.1, 0.0, 0.0));

        assert!(path.via_points.is_empty());
        assert_relative_eq!((path.insertion - path.origin).norm(), 0.1, epsilon = 1e-10);
    }
}
