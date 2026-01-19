//! Moving frame computation for curve sweeping.
//!
//! Provides parallel transport frames along curves for consistent orientation.

use nalgebra::{Point3, UnitVector3, Vector3};

/// A reference frame at a point on a curve.
///
/// Consists of three orthonormal vectors: tangent, normal, and binormal.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    /// Tangent direction (forward along curve).
    pub tangent: Vector3<f64>,
    /// Normal direction (perpendicular to tangent).
    pub normal: Vector3<f64>,
    /// Binormal direction (perpendicular to both tangent and normal).
    pub binormal: Vector3<f64>,
}

impl Frame {
    /// Create a new frame from tangent, normal, and binormal vectors.
    ///
    /// Vectors are assumed to be orthonormal.
    #[must_use]
    pub fn new(tangent: Vector3<f64>, normal: Vector3<f64>, binormal: Vector3<f64>) -> Self {
        Self {
            tangent,
            normal,
            binormal,
        }
    }

    /// Create an initial frame from a tangent vector.
    ///
    /// Computes a perpendicular normal and binormal.
    #[must_use]
    pub fn from_tangent(tangent: Vector3<f64>) -> Self {
        let tangent = tangent.try_normalize(f64::EPSILON).unwrap_or(Vector3::z());

        // Find a perpendicular vector
        let normal = find_perpendicular(tangent);
        let binormal = tangent.cross(&normal);

        Self {
            tangent,
            normal,
            binormal,
        }
    }

    /// Rotate the frame's normal and binormal by an angle around the tangent.
    #[must_use]
    pub fn rotate_around_tangent(&self, angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let new_normal = self.normal * cos_a + self.binormal * sin_a;
        let new_binormal = -self.normal * sin_a + self.binormal * cos_a;

        Self {
            tangent: self.tangent,
            normal: new_normal,
            binormal: new_binormal,
        }
    }
}

/// Find a vector perpendicular to the given vector.
fn find_perpendicular(v: Vector3<f64>) -> Vector3<f64> {
    // Choose the axis most perpendicular to v
    let abs_x = v.x.abs();
    let abs_y = v.y.abs();
    let abs_z = v.z.abs();

    let perp = if abs_x <= abs_y && abs_x <= abs_z {
        Vector3::x()
    } else if abs_y <= abs_z {
        Vector3::y()
    } else {
        Vector3::z()
    };

    v.cross(&perp)
        .try_normalize(f64::EPSILON)
        .unwrap_or(Vector3::y())
}

/// Compute parallel transport frames along a curve.
///
/// Uses rotation minimizing frames to avoid twisting.
///
/// # Arguments
///
/// * `points` - Points along the curve
///
/// # Returns
///
/// A frame at each point on the curve.
///
/// # Example
///
/// ```
/// use mesh_from_curves::parallel_transport_frames;
/// use nalgebra::Point3;
///
/// let points = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ];
///
/// let frames = parallel_transport_frames(&points);
/// assert_eq!(frames.len(), 3);
/// ```
#[must_use]
pub fn parallel_transport_frames(points: &[Point3<f64>]) -> Vec<Frame> {
    if points.len() < 2 {
        return Vec::new();
    }

    let mut frames = Vec::with_capacity(points.len());

    // Compute tangent at first point
    let first_tangent = (points[1] - points[0])
        .try_normalize(f64::EPSILON)
        .unwrap_or(Vector3::z());
    let first_frame = Frame::from_tangent(first_tangent);
    frames.push(first_frame);

    // Propagate frames using parallel transport
    for i in 1..points.len() {
        let prev_frame = frames[i - 1];

        // Compute tangent at this point
        let tangent = if i < points.len() - 1 {
            // Average of incoming and outgoing directions
            let prev_dir = points[i] - points[i - 1];
            let next_dir = points[i + 1] - points[i];
            (prev_dir + next_dir)
                .try_normalize(f64::EPSILON)
                .unwrap_or(prev_frame.tangent)
        } else {
            // Last point: use direction from previous
            (points[i] - points[i - 1])
                .try_normalize(f64::EPSILON)
                .unwrap_or(prev_frame.tangent)
        };

        // Parallel transport the frame
        let frame = parallel_transport_frame(&prev_frame, tangent);
        frames.push(frame);
    }

    frames
}

/// Transport a frame from one tangent to another.
///
/// Uses the rotation that maps the old tangent to the new one
/// to transform the normal and binormal.
fn parallel_transport_frame(prev_frame: &Frame, new_tangent: Vector3<f64>) -> Frame {
    let new_tangent = new_tangent
        .try_normalize(f64::EPSILON)
        .unwrap_or(prev_frame.tangent);

    // Compute rotation axis
    let axis = prev_frame.tangent.cross(&new_tangent);
    let axis_len = axis.norm();

    if axis_len < f64::EPSILON {
        // Tangents are parallel
        let dot = prev_frame.tangent.dot(&new_tangent);
        if dot > 0.0 {
            // Same direction
            Frame {
                tangent: new_tangent,
                normal: prev_frame.normal,
                binormal: prev_frame.binormal,
            }
        } else {
            // Opposite direction
            Frame {
                tangent: new_tangent,
                normal: -prev_frame.normal,
                binormal: -prev_frame.binormal,
            }
        }
    } else {
        // Rotate around axis
        let axis = UnitVector3::new_normalize(axis);
        let dot = prev_frame.tangent.dot(&new_tangent).clamp(-1.0, 1.0);
        let angle = dot.acos();

        // Rodrigues rotation formula
        let rotate = |v: Vector3<f64>| {
            let k = axis.into_inner();
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            v * cos_a + k.cross(&v) * sin_a + k * (k.dot(&v)) * (1.0 - cos_a)
        };

        Frame {
            tangent: new_tangent,
            normal: rotate(prev_frame.normal),
            binormal: rotate(prev_frame.binormal),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn frame_from_tangent_x() {
        let frame = Frame::from_tangent(Vector3::x());
        assert_relative_eq!(frame.tangent.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(frame.normal.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(frame.binormal.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(frame.tangent.dot(&frame.normal), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn frame_from_tangent_z() {
        let frame = Frame::from_tangent(Vector3::z());
        assert_relative_eq!(frame.tangent.dot(&frame.normal), 0.0, epsilon = 1e-10);
        assert_relative_eq!(frame.tangent.dot(&frame.binormal), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn frame_rotate() {
        let frame = Frame::from_tangent(Vector3::x());
        let rotated = frame.rotate_around_tangent(std::f64::consts::FRAC_PI_2);

        // Normal and binormal should swap (with sign)
        assert_relative_eq!(rotated.tangent.dot(&rotated.normal), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn parallel_transport_straight_line() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let frames = parallel_transport_frames(&points);

        assert_eq!(frames.len(), 3);

        // All tangents should point in +X
        for frame in &frames {
            assert_relative_eq!(frame.tangent.x, 1.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn parallel_transport_quarter_turn() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];

        let frames = parallel_transport_frames(&points);

        assert_eq!(frames.len(), 3);

        // First tangent should be roughly +X
        assert!(frames[0].tangent.x > 0.5);

        // Last tangent should be roughly +Y
        assert!(frames[2].tangent.y > 0.5);
    }

    #[test]
    fn parallel_transport_empty() {
        let points: Vec<Point3<f64>> = vec![];
        let frames = parallel_transport_frames(&points);
        assert!(frames.is_empty());
    }

    #[test]
    fn parallel_transport_single_point() {
        let points = vec![Point3::origin()];
        let frames = parallel_transport_frames(&points);
        assert!(frames.is_empty());
    }

    #[test]
    fn find_perpendicular_x() {
        let perp = find_perpendicular(Vector3::x());
        assert_relative_eq!(Vector3::x().dot(&perp), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn find_perpendicular_y() {
        let perp = find_perpendicular(Vector3::y());
        assert_relative_eq!(Vector3::y().dot(&perp), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn find_perpendicular_z() {
        let perp = find_perpendicular(Vector3::z());
        assert_relative_eq!(Vector3::z().dot(&perp), 0.0, epsilon = 1e-10);
    }
}
