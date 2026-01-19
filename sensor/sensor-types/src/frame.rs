//! Coordinate frames and transforms for sensor data.
//!
//! All sensor readings reference a coordinate frame, enabling proper
//! spatial alignment in `sensor-fusion`.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Reference frame for sensor data.
///
/// Sensor readings must specify their coordinate frame to enable
/// proper transformation and fusion.
///
/// # Example
///
/// ```
/// use sensor_types::CoordinateFrame;
///
/// let body = CoordinateFrame::Body;
/// let camera = CoordinateFrame::sensor("front_camera");
///
/// assert!(matches!(camera, CoordinateFrame::Sensor(_)));
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CoordinateFrame {
    /// Global world frame (inertial reference).
    World,
    /// Robot/vehicle body frame (moves with the platform).
    #[default]
    Body,
    /// Named sensor frame (e.g., `front_camera`, `left_lidar`).
    Sensor(String),
}

impl CoordinateFrame {
    /// Creates a sensor-specific coordinate frame.
    ///
    /// # Example
    ///
    /// ```
    /// use sensor_types::CoordinateFrame;
    ///
    /// let frame = CoordinateFrame::sensor("imu_link");
    /// assert!(matches!(frame, CoordinateFrame::Sensor(ref name) if name == "imu_link"));
    /// ```
    #[must_use]
    pub fn sensor(name: impl Into<String>) -> Self {
        Self::Sensor(name.into())
    }

    /// Returns the name of the frame for display purposes.
    #[must_use]
    pub fn name(&self) -> &str {
        match self {
            Self::World => "world",
            Self::Body => "body",
            Self::Sensor(name) => name,
        }
    }
}

/// A 3D pose (position + orientation).
///
/// Represents a rigid body transformation in 3D space.
/// Position is in meters, orientation is a unit quaternion (w, x, y, z).
///
/// # Quaternion Convention
///
/// The quaternion is stored as `[w, x, y, z]` where `w` is the scalar part.
/// The quaternion should be normalized (unit length).
///
/// # Example
///
/// ```
/// use sensor_types::Pose3d;
///
/// // Identity pose (no rotation, at origin)
/// let pose = Pose3d::identity();
/// assert_eq!(pose.position, [0.0, 0.0, 0.0]);
/// assert_eq!(pose.orientation, [1.0, 0.0, 0.0, 0.0]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Pose3d {
    /// Position in meters: `[x, y, z]`.
    pub position: [f64; 3],
    /// Orientation as unit quaternion: `[w, x, y, z]`.
    pub orientation: [f64; 4],
}

impl Pose3d {
    /// Creates a new pose from position and orientation.
    ///
    /// # Arguments
    ///
    /// * `position` - Position in meters `[x, y, z]`
    /// * `orientation` - Unit quaternion `[w, x, y, z]`
    ///
    /// # Example
    ///
    /// ```
    /// use sensor_types::Pose3d;
    ///
    /// let pose = Pose3d::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
    /// assert_eq!(pose.position[0], 1.0);
    /// ```
    #[must_use]
    pub const fn new(position: [f64; 3], orientation: [f64; 4]) -> Self {
        Self {
            position,
            orientation,
        }
    }

    /// Creates the identity pose (at origin, no rotation).
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Creates a pose with only translation (no rotation).
    ///
    /// # Example
    ///
    /// ```
    /// use sensor_types::Pose3d;
    ///
    /// let pose = Pose3d::from_translation([1.0, 2.0, 3.0]);
    /// assert_eq!(pose.position, [1.0, 2.0, 3.0]);
    /// assert_eq!(pose.orientation, [1.0, 0.0, 0.0, 0.0]);
    /// ```
    #[must_use]
    pub const fn from_translation(position: [f64; 3]) -> Self {
        Self {
            position,
            orientation: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Returns the quaternion norm (should be ~1.0 for valid poses).
    #[must_use]
    pub fn quaternion_norm(&self) -> f64 {
        let [w, x, y, z] = self.orientation;
        w.mul_add(w, x.mul_add(x, y.mul_add(y, z * z))).sqrt()
    }

    /// Normalizes the quaternion to unit length.
    ///
    /// Returns `None` if the quaternion has zero length.
    #[must_use]
    pub fn normalized(&self) -> Option<Self> {
        let norm = self.quaternion_norm();
        if norm < 1e-10 {
            return None;
        }
        let [w, x, y, z] = self.orientation;
        Some(Self {
            position: self.position,
            orientation: [w / norm, x / norm, y / norm, z / norm],
        })
    }

    /// Checks if the quaternion is approximately normalized.
    #[must_use]
    pub fn is_normalized(&self, tolerance: f64) -> bool {
        (self.quaternion_norm() - 1.0).abs() < tolerance
    }
}

impl Default for Pose3d {
    fn default() -> Self {
        Self::identity()
    }
}

/// A 3D rigid transform (rotation + translation + optional scale).
///
/// Used for coordinate frame transformations in `sensor-fusion`.
/// Transforms are applied as: `p' = scale * R * p + t`
///
/// # Example
///
/// ```
/// use sensor_types::Transform3d;
///
/// let tf = Transform3d::identity();
/// assert_eq!(tf.scale, 1.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Transform3d {
    /// Rotation as unit quaternion: `[w, x, y, z]`.
    pub rotation: [f64; 4],
    /// Translation in meters: `[x, y, z]`.
    pub translation: [f64; 3],
    /// Uniform scale factor.
    pub scale: f64,
}

impl Transform3d {
    /// Creates a new transform.
    #[must_use]
    pub const fn new(rotation: [f64; 4], translation: [f64; 3], scale: f64) -> Self {
        Self {
            rotation,
            translation,
            scale,
        }
    }

    /// Creates the identity transform.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            rotation: [1.0, 0.0, 0.0, 0.0],
            translation: [0.0, 0.0, 0.0],
            scale: 1.0,
        }
    }

    /// Creates a pure translation transform.
    #[must_use]
    pub const fn from_translation(translation: [f64; 3]) -> Self {
        Self {
            rotation: [1.0, 0.0, 0.0, 0.0],
            translation,
            scale: 1.0,
        }
    }

    /// Creates a transform from a pose (no scaling).
    #[must_use]
    pub const fn from_pose(pose: Pose3d) -> Self {
        Self {
            rotation: pose.orientation,
            translation: pose.position,
            scale: 1.0,
        }
    }

    /// Converts this transform to a pose (ignores scale).
    #[must_use]
    pub const fn to_pose(self) -> Pose3d {
        Pose3d {
            position: self.translation,
            orientation: self.rotation,
        }
    }

    /// Applies this transform to a 3D point.
    ///
    /// The transform is applied as: `p' = scale * R * p + t`
    ///
    /// # Example
    ///
    /// ```
    /// use sensor_types::Transform3d;
    ///
    /// let tf = Transform3d::from_translation([1.0, 0.0, 0.0]);
    /// let point = [0.0, 0.0, 0.0];
    /// let result = tf.apply_point(point);
    /// assert!((result[0] - 1.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn apply_point(&self, point: [f64; 3]) -> [f64; 3] {
        // Rotate the point using quaternion rotation
        let rotated = self.rotate_vector(point);

        // Scale and translate
        [
            self.scale.mul_add(rotated[0], self.translation[0]),
            self.scale.mul_add(rotated[1], self.translation[1]),
            self.scale.mul_add(rotated[2], self.translation[2]),
        ]
    }

    /// Rotates a vector by the transform's rotation (no translation/scale).
    ///
    /// Uses the quaternion rotation formula: `v' = q * v * q^-1`
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    pub fn rotate_vector(&self, v: [f64; 3]) -> [f64; 3] {
        let [w, x, y, z] = self.rotation;
        let [vx, vy, vz] = v;

        // Quaternion rotation using the standard formula:
        // v' = v + 2*w*(cross(q_xyz, v)) + 2*cross(q_xyz, cross(q_xyz, v))
        // This is equivalent to v' = q * v * q^-1 but more efficient

        // cross(q_xyz, v)
        let cx = y.mul_add(vz, -(z * vy));
        let cy = z.mul_add(vx, -(x * vz));
        let cz = x.mul_add(vy, -(y * vx));

        // 2 * w * cross(q_xyz, v)
        let t1x = 2.0 * w * cx;
        let t1y = 2.0 * w * cy;
        let t1z = 2.0 * w * cz;

        // 2 * cross(q_xyz, cross(q_xyz, v))
        let t2x = 2.0 * y.mul_add(cz, -(z * cy));
        let t2y = 2.0 * z.mul_add(cx, -(x * cz));
        let t2z = 2.0 * x.mul_add(cy, -(y * cx));

        [vx + t1x + t2x, vy + t1y + t2y, vz + t1z + t2z]
    }
}

impl Default for Transform3d {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordinate_frame_sensor() {
        let frame = CoordinateFrame::sensor("front_camera");
        assert_eq!(frame.name(), "front_camera");
    }

    #[test]
    fn coordinate_frame_names() {
        assert_eq!(CoordinateFrame::World.name(), "world");
        assert_eq!(CoordinateFrame::Body.name(), "body");
    }

    #[test]
    #[allow(clippy::float_cmp)] // Exact constant values from identity()
    fn pose_identity() {
        let pose = Pose3d::identity();
        assert_eq!(pose.position, [0.0, 0.0, 0.0]);
        assert_eq!(pose.orientation, [1.0, 0.0, 0.0, 0.0]);
        assert!(pose.is_normalized(1e-10));
    }

    #[test]
    fn pose_normalize() {
        let pose = Pose3d::new([0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0]);
        let normalized = pose.normalized();
        assert!(normalized.is_some());
        let n = normalized.unwrap_or_default();
        assert!(n.is_normalized(1e-10));
    }

    #[test]
    fn pose_zero_quaternion() {
        let pose = Pose3d::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]);
        assert!(pose.normalized().is_none());
    }

    #[test]
    fn transform_identity() {
        let tf = Transform3d::identity();
        let point = [1.0, 2.0, 3.0];
        let result = tf.apply_point(point);
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!((result[1] - 2.0).abs() < 1e-10);
        assert!((result[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn transform_translation() {
        let tf = Transform3d::from_translation([10.0, 20.0, 30.0]);
        let point = [1.0, 2.0, 3.0];
        let result = tf.apply_point(point);
        assert!((result[0] - 11.0).abs() < 1e-10);
        assert!((result[1] - 22.0).abs() < 1e-10);
        assert!((result[2] - 33.0).abs() < 1e-10);
    }

    #[test]
    #[allow(clippy::float_cmp)] // Exact constant values, direct copy
    fn transform_from_pose() {
        let pose = Pose3d::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
        let tf = Transform3d::from_pose(pose);
        assert_eq!(tf.translation, [1.0, 2.0, 3.0]);
        assert_eq!(tf.scale, 1.0);
    }

    #[test]
    #[allow(clippy::float_cmp)] // Exact constant values, direct copy
    fn transform_to_pose() {
        let tf = Transform3d::new([1.0, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0], 2.0);
        let pose = tf.to_pose();
        assert_eq!(pose.position, [1.0, 2.0, 3.0]);
        assert_eq!(pose.orientation, [1.0, 0.0, 0.0, 0.0]);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn coordinate_frame_serialization() {
        let frame = CoordinateFrame::sensor("test");
        let json = serde_json::to_string(&frame).ok();
        assert!(json.is_some());
    }
}
