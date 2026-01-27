//! Intermediate representation types for URDF data.
//!
//! These types represent the parsed URDF structure before conversion to sim types.
//! They closely mirror the URDF XML schema but use Rust-native types.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Safe axis normalization with Z fallback for zero-length vectors.
#[inline]
fn safe_normalize_axis(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { Vector3::z() }
}

// ============================================================================
// Origin (Pose)
// ============================================================================

/// Origin/pose specification in URDF.
///
/// Represents the `<origin>` element with xyz position and rpy rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfOrigin {
    /// Position (xyz) in meters.
    pub xyz: Vector3<f64>,
    /// Rotation as roll-pitch-yaw (rpy) in radians.
    pub rpy: Vector3<f64>,
}

impl Default for UrdfOrigin {
    fn default() -> Self {
        Self {
            xyz: Vector3::zeros(),
            rpy: Vector3::zeros(),
        }
    }
}

impl UrdfOrigin {
    /// Create a new origin at position with identity rotation.
    #[must_use]
    pub fn from_xyz(x: f64, y: f64, z: f64) -> Self {
        Self {
            xyz: Vector3::new(x, y, z),
            rpy: Vector3::zeros(),
        }
    }

    /// Create from position and rpy.
    #[must_use]
    pub fn new(xyz: Vector3<f64>, rpy: Vector3<f64>) -> Self {
        Self { xyz, rpy }
    }

    /// Get the position as a point.
    #[must_use]
    pub fn position(&self) -> Point3<f64> {
        Point3::from(self.xyz)
    }

    /// Convert rpy to quaternion.
    ///
    /// URDF uses fixed-axis XYZ (roll about X, then pitch about Y, then yaw about Z).
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(self.rpy.x, self.rpy.y, self.rpy.z)
    }
}

// ============================================================================
// Inertial Properties
// ============================================================================

/// Inertial properties from `<inertial>` element.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfInertial {
    /// Origin of the inertial frame relative to link frame.
    pub origin: UrdfOrigin,
    /// Mass in kg.
    pub mass: f64,
    /// Inertia tensor elements (symmetric 3x3 matrix).
    /// Stored as [ixx, ixy, ixz, iyy, iyz, izz].
    pub inertia: UrdfInertia,
}

impl Default for UrdfInertial {
    fn default() -> Self {
        Self {
            origin: UrdfOrigin::default(),
            mass: 1.0,
            inertia: UrdfInertia::default(),
        }
    }
}

impl UrdfInertial {
    /// Create inertial properties with given mass and default inertia.
    #[must_use]
    pub fn with_mass(mass: f64) -> Self {
        Self {
            mass,
            ..Default::default()
        }
    }
}

/// Inertia tensor from URDF.
///
/// URDF specifies the upper-triangular elements of the symmetric inertia tensor.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfInertia {
    /// Moment of inertia about X axis.
    pub ixx: f64,
    /// Product of inertia XY.
    pub ixy: f64,
    /// Product of inertia XZ.
    pub ixz: f64,
    /// Moment of inertia about Y axis.
    pub iyy: f64,
    /// Product of inertia YZ.
    pub iyz: f64,
    /// Moment of inertia about Z axis.
    pub izz: f64,
}

impl Default for UrdfInertia {
    fn default() -> Self {
        // Default to unit sphere inertia: I = 2/5 * m * r^2 with m=1, r=1
        let i = 0.4;
        Self {
            ixx: i,
            ixy: 0.0,
            ixz: 0.0,
            iyy: i,
            iyz: 0.0,
            izz: i,
        }
    }
}

impl UrdfInertia {
    /// Create a diagonal inertia tensor.
    #[must_use]
    pub fn diagonal(ixx: f64, iyy: f64, izz: f64) -> Self {
        Self {
            ixx,
            ixy: 0.0,
            ixz: 0.0,
            iyy,
            iyz: 0.0,
            izz,
        }
    }

    /// Convert to a 3x3 matrix.
    #[must_use]
    pub fn to_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.ixx, self.ixy, self.ixz, self.ixy, self.iyy, self.iyz, self.ixz, self.iyz,
            self.izz,
        )
    }
}

// ============================================================================
// Geometry
// ============================================================================

/// Geometry shape from `<geometry>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum UrdfGeometry {
    /// Box with size (x, y, z) in meters.
    Box {
        /// Size in x, y, z dimensions.
        size: Vector3<f64>,
    },
    /// Cylinder with radius and length in meters.
    Cylinder {
        /// Cylinder radius in meters.
        radius: f64,
        /// Cylinder length in meters.
        length: f64,
    },
    /// Sphere with radius in meters.
    Sphere {
        /// Sphere radius in meters.
        radius: f64,
    },
    /// Mesh file reference.
    Mesh {
        /// Path to mesh file.
        filename: String,
        /// Optional scale factor.
        scale: Option<Vector3<f64>>,
    },
}

impl UrdfGeometry {
    /// Create a box geometry.
    #[must_use]
    pub fn box_shape(x: f64, y: f64, z: f64) -> Self {
        Self::Box {
            size: Vector3::new(x, y, z),
        }
    }

    /// Create a cylinder geometry.
    #[must_use]
    pub fn cylinder(radius: f64, length: f64) -> Self {
        Self::Cylinder { radius, length }
    }

    /// Create a sphere geometry.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self::Sphere { radius }
    }
}

// ============================================================================
// Visual and Collision
// ============================================================================

/// Visual properties from `<visual>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfVisual {
    /// Optional name.
    pub name: Option<String>,
    /// Origin relative to link frame.
    pub origin: UrdfOrigin,
    /// Geometry shape.
    pub geometry: UrdfGeometry,
    /// Material name reference.
    pub material: Option<String>,
}

/// Collision properties from `<collision>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfCollision {
    /// Optional name.
    pub name: Option<String>,
    /// Origin relative to link frame.
    pub origin: UrdfOrigin,
    /// Geometry shape.
    pub geometry: UrdfGeometry,
}

// ============================================================================
// Link
// ============================================================================

/// A link (rigid body) from `<link>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfLink {
    /// Link name (required, must be unique).
    pub name: String,
    /// Inertial properties (optional for massless/fixed links).
    pub inertial: Option<UrdfInertial>,
    /// Visual geometries (can have multiple).
    pub visuals: Vec<UrdfVisual>,
    /// Collision geometries (can have multiple).
    pub collisions: Vec<UrdfCollision>,
}

impl UrdfLink {
    /// Create a new link with just a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            inertial: None,
            visuals: Vec::new(),
            collisions: Vec::new(),
        }
    }

    /// Set inertial properties.
    #[must_use]
    pub fn with_inertial(mut self, inertial: UrdfInertial) -> Self {
        self.inertial = Some(inertial);
        self
    }

    /// Add a collision geometry.
    #[must_use]
    pub fn with_collision(mut self, collision: UrdfCollision) -> Self {
        self.collisions.push(collision);
        self
    }

    /// Check if this is a massless/fixed link.
    #[must_use]
    pub fn is_massless(&self) -> bool {
        self.inertial.is_none()
    }
}

// ============================================================================
// Joint
// ============================================================================

/// Joint type from URDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum UrdfJointType {
    /// Revolute joint with limits.
    Revolute,
    /// Continuous (unlimited revolute) joint.
    Continuous,
    /// Prismatic (sliding) joint.
    Prismatic,
    /// Fixed (welded) joint.
    Fixed,
    /// Floating (6-DOF) joint.
    Floating,
    /// Planar (2D translation + rotation) joint.
    Planar,
}

impl UrdfJointType {
    /// Parse joint type from string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "revolute" => Some(Self::Revolute),
            "continuous" => Some(Self::Continuous),
            "prismatic" => Some(Self::Prismatic),
            "fixed" => Some(Self::Fixed),
            "floating" => Some(Self::Floating),
            "planar" => Some(Self::Planar),
            _ => None,
        }
    }

    /// Get degrees of freedom for this joint type.
    #[must_use]
    pub fn dof(&self) -> usize {
        match self {
            Self::Fixed => 0,
            Self::Revolute | Self::Continuous | Self::Prismatic => 1,
            Self::Planar => 3,
            Self::Floating => 6,
        }
    }
}

/// Joint limits from `<limit>` element.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfJointLimit {
    /// Lower position limit (rad or m).
    pub lower: f64,
    /// Upper position limit (rad or m).
    pub upper: f64,
    /// Maximum effort (N or Nm).
    pub effort: f64,
    /// Maximum velocity (rad/s or m/s).
    pub velocity: f64,
}

impl Default for UrdfJointLimit {
    fn default() -> Self {
        Self {
            lower: 0.0,
            upper: 0.0,
            effort: 0.0,
            velocity: 0.0,
        }
    }
}

impl UrdfJointLimit {
    /// Create symmetric limits.
    #[must_use]
    pub fn symmetric(limit: f64, effort: f64, velocity: f64) -> Self {
        Self {
            lower: -limit,
            upper: limit,
            effort,
            velocity,
        }
    }
}

/// Joint dynamics from `<dynamics>` element.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfJointDynamics {
    /// Viscous damping coefficient.
    pub damping: f64,
    /// Static friction (Coulomb).
    pub friction: f64,
}

impl Default for UrdfJointDynamics {
    fn default() -> Self {
        Self {
            damping: 0.0,
            friction: 0.0,
        }
    }
}

/// A joint connecting two links.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfJoint {
    /// Joint name (required, must be unique).
    pub name: String,
    /// Joint type.
    pub joint_type: UrdfJointType,
    /// Parent link name.
    pub parent: String,
    /// Child link name.
    pub child: String,
    /// Origin of joint frame relative to parent link frame.
    pub origin: UrdfOrigin,
    /// Joint axis in joint frame (default: z-axis).
    pub axis: Vector3<f64>,
    /// Joint limits (required for revolute and prismatic).
    pub limit: Option<UrdfJointLimit>,
    /// Joint dynamics (optional).
    pub dynamics: Option<UrdfJointDynamics>,
}

impl UrdfJoint {
    /// Create a new joint.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        joint_type: UrdfJointType,
        parent: impl Into<String>,
        child: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            joint_type,
            parent: parent.into(),
            child: child.into(),
            origin: UrdfOrigin::default(),
            axis: Vector3::z(), // URDF default axis
            limit: None,
            dynamics: None,
        }
    }

    /// Set the joint origin.
    #[must_use]
    pub fn with_origin(mut self, origin: UrdfOrigin) -> Self {
        self.origin = origin;
        self
    }

    /// Set the joint axis.
    #[must_use]
    pub fn with_axis(mut self, axis: Vector3<f64>) -> Self {
        self.axis = safe_normalize_axis(axis);
        self
    }

    /// Set the joint limits.
    #[must_use]
    pub fn with_limit(mut self, limit: UrdfJointLimit) -> Self {
        self.limit = Some(limit);
        self
    }

    /// Set the joint dynamics.
    #[must_use]
    pub fn with_dynamics(mut self, dynamics: UrdfJointDynamics) -> Self {
        self.dynamics = Some(dynamics);
        self
    }
}

// ============================================================================
// Robot
// ============================================================================

/// A complete URDF robot model.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UrdfRobot {
    /// Robot name.
    pub name: String,
    /// All links in the robot.
    pub links: Vec<UrdfLink>,
    /// All joints in the robot.
    pub joints: Vec<UrdfJoint>,
}

impl UrdfRobot {
    /// Create a new robot with just a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            links: Vec::new(),
            joints: Vec::new(),
        }
    }

    /// Add a link.
    #[must_use]
    pub fn with_link(mut self, link: UrdfLink) -> Self {
        self.links.push(link);
        self
    }

    /// Add a joint.
    #[must_use]
    pub fn with_joint(mut self, joint: UrdfJoint) -> Self {
        self.joints.push(joint);
        self
    }

    /// Get a link by name.
    #[must_use]
    pub fn link(&self, name: &str) -> Option<&UrdfLink> {
        self.links.iter().find(|l| l.name == name)
    }

    /// Get a joint by name.
    #[must_use]
    pub fn joint(&self, name: &str) -> Option<&UrdfJoint> {
        self.joints.iter().find(|j| j.name == name)
    }

    /// Get all link names.
    pub fn link_names(&self) -> impl Iterator<Item = &str> {
        self.links.iter().map(|l| l.name.as_str())
    }

    /// Get all joint names.
    pub fn joint_names(&self) -> impl Iterator<Item = &str> {
        self.joints.iter().map(|j| j.name.as_str())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_origin_default() {
        let origin = UrdfOrigin::default();
        assert_eq!(origin.xyz, Vector3::zeros());
        assert_eq!(origin.rpy, Vector3::zeros());
    }

    #[test]
    fn test_origin_rotation() {
        let origin = UrdfOrigin::new(
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        let rot = origin.rotation();
        let rotated = rot * Vector3::x();
        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_inertia_to_matrix() {
        let inertia = UrdfInertia {
            ixx: 1.0,
            ixy: 0.1,
            ixz: 0.2,
            iyy: 2.0,
            iyz: 0.3,
            izz: 3.0,
        };
        let m = inertia.to_matrix();
        assert_relative_eq!(m[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(m[(0, 1)], 0.1, epsilon = 1e-10);
        assert_relative_eq!(m[(1, 0)], 0.1, epsilon = 1e-10); // Symmetric
        assert_relative_eq!(m[(2, 2)], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_type_dof() {
        assert_eq!(UrdfJointType::Fixed.dof(), 0);
        assert_eq!(UrdfJointType::Revolute.dof(), 1);
        assert_eq!(UrdfJointType::Continuous.dof(), 1);
        assert_eq!(UrdfJointType::Prismatic.dof(), 1);
        assert_eq!(UrdfJointType::Planar.dof(), 3);
        assert_eq!(UrdfJointType::Floating.dof(), 6);
    }

    #[test]
    fn test_joint_type_from_str() {
        assert_eq!(
            UrdfJointType::from_str("revolute"),
            Some(UrdfJointType::Revolute)
        );
        assert_eq!(UrdfJointType::from_str("fixed"), Some(UrdfJointType::Fixed));
        assert_eq!(UrdfJointType::from_str("invalid"), None);
    }

    #[test]
    fn test_robot_builder() {
        let robot = UrdfRobot::new("test_robot")
            .with_link(UrdfLink::new("base_link"))
            .with_link(UrdfLink::new("link1"))
            .with_joint(UrdfJoint::new(
                "joint1",
                UrdfJointType::Revolute,
                "base_link",
                "link1",
            ));

        assert_eq!(robot.name, "test_robot");
        assert_eq!(robot.links.len(), 2);
        assert_eq!(robot.joints.len(), 1);
        assert!(robot.link("base_link").is_some());
        assert!(robot.joint("joint1").is_some());
    }
}
