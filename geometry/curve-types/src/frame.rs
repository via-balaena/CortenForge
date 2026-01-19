//! Curve framing and parallel transport.
//!
//! This module provides utilities for computing coordinate frames along curves,
//! using rotation-minimizing frames (parallel transport) to avoid the gimbal
//! lock and twisting issues of Frenet-Serret frames.

use crate::Curve;
use nalgebra::{Point3, UnitQuaternion, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A coordinate frame at a point on a curve.
///
/// The frame consists of three mutually orthonormal vectors:
/// - `tangent`: Points along the curve in the direction of increasing `t`
/// - `normal`: Perpendicular to tangent, typically toward center of curvature
/// - `binormal`: Completes the right-handed coordinate system (`tangent × normal`)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Frame {
    /// Position on the curve.
    pub position: Point3<f64>,
    /// Unit tangent vector (forward direction).
    pub tangent: Vector3<f64>,
    /// Unit normal vector (perpendicular to tangent).
    pub normal: Vector3<f64>,
    /// Unit binormal vector (`tangent × normal`).
    pub binormal: Vector3<f64>,
    /// Parameter value `t` where this frame was computed.
    pub t: f64,
}

impl Frame {
    /// Create a new frame with the given components.
    ///
    /// # Note
    ///
    /// The vectors are assumed to be orthonormal. Use [`Self::from_tangent_and_up`]
    /// for automatic orthonormalization.
    #[must_use]
    pub fn new(
        position: Point3<f64>,
        tangent: Vector3<f64>,
        normal: Vector3<f64>,
        binormal: Vector3<f64>,
        t: f64,
    ) -> Self {
        Self {
            position,
            tangent,
            normal,
            binormal,
            t,
        }
    }

    /// Create a frame from a tangent vector and an "up" hint.
    ///
    /// The normal and binormal are computed to form an orthonormal basis,
    /// with the normal as close to the up vector as possible.
    #[must_use]
    pub fn from_tangent_and_up(
        position: Point3<f64>,
        tangent: Vector3<f64>,
        up: Vector3<f64>,
        t: f64,
    ) -> Self {
        let tangent = tangent.normalize();

        // Binormal is perpendicular to both tangent and up
        let binormal = tangent.cross(&up);
        let binormal_norm = binormal.norm();

        let (normal, binormal) = if binormal_norm > 1e-10 {
            let binormal = binormal / binormal_norm;
            let normal = binormal.cross(&tangent);
            (normal, binormal)
        } else {
            // Tangent is parallel to up, choose arbitrary perpendicular
            let perp = if tangent.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let binormal = tangent.cross(&perp).normalize();
            let normal = binormal.cross(&tangent);
            (normal, binormal)
        };

        Self {
            position,
            tangent,
            normal,
            binormal,
            t,
        }
    }

    /// Convert the frame to a rotation quaternion.
    ///
    /// The quaternion represents the rotation from the world frame
    /// (where X=tangent, Y=normal, Z=binormal) to this local frame.
    #[must_use]
    pub fn to_quaternion(&self) -> UnitQuaternion<f64> {
        let rotation_matrix =
            nalgebra::Matrix3::from_columns(&[self.tangent, self.normal, self.binormal]);
        UnitQuaternion::from_matrix(&rotation_matrix)
    }

    /// Create a frame from a rotation quaternion.
    #[must_use]
    pub fn from_quaternion(position: Point3<f64>, q: UnitQuaternion<f64>, t: f64) -> Self {
        let m = q.to_rotation_matrix();
        Self {
            position,
            tangent: m.matrix().column(0).into(),
            normal: m.matrix().column(1).into(),
            binormal: m.matrix().column(2).into(),
            t,
        }
    }

    /// Transform a local point to world coordinates.
    ///
    /// The local coordinate system has:
    /// - X axis along the tangent
    /// - Y axis along the normal
    /// - Z axis along the binormal
    #[must_use]
    pub fn local_to_world(&self, local: Point3<f64>) -> Point3<f64> {
        self.position + self.tangent * local.x + self.normal * local.y + self.binormal * local.z
    }

    /// Transform a local direction to world coordinates.
    #[must_use]
    pub fn local_direction_to_world(&self, local: Vector3<f64>) -> Vector3<f64> {
        self.tangent * local.x + self.normal * local.y + self.binormal * local.z
    }

    /// Transform a world point to local coordinates.
    #[must_use]
    pub fn world_to_local(&self, world: Point3<f64>) -> Point3<f64> {
        let v = world - self.position;
        Point3::new(
            v.dot(&self.tangent),
            v.dot(&self.normal),
            v.dot(&self.binormal),
        )
    }

    /// Check if the frame is orthonormal within tolerance.
    #[must_use]
    pub fn is_orthonormal(&self, tolerance: f64) -> bool {
        let t_len = (self.tangent.norm() - 1.0).abs();
        let n_len = (self.normal.norm() - 1.0).abs();
        let b_len = (self.binormal.norm() - 1.0).abs();
        let tn_dot = self.tangent.dot(&self.normal).abs();
        let tb_dot = self.tangent.dot(&self.binormal).abs();
        let nb_dot = self.normal.dot(&self.binormal).abs();

        t_len < tolerance
            && n_len < tolerance
            && b_len < tolerance
            && tn_dot < tolerance
            && tb_dot < tolerance
            && nb_dot < tolerance
    }

    /// Orthonormalize the frame using Gram-Schmidt.
    #[must_use]
    pub fn orthonormalized(&self) -> Self {
        let tangent = self.tangent.normalize();
        let normal = (self.normal - tangent * tangent.dot(&self.normal)).normalize();
        let binormal = tangent.cross(&normal);

        Self {
            position: self.position,
            tangent,
            normal,
            binormal,
            t: self.t,
        }
    }

    /// Interpolate between two frames.
    ///
    /// Uses spherical linear interpolation (SLERP) for the rotation
    /// and linear interpolation for the position.
    #[must_use]
    pub fn interpolate(&self, other: &Frame, alpha: f64) -> Self {
        let position = self.position + (other.position - self.position) * alpha;
        let t = self.t + (other.t - self.t) * alpha;

        let q1 = self.to_quaternion();
        let q2 = other.to_quaternion();
        let q = q1.slerp(&q2, alpha);

        Self::from_quaternion(position, q, t)
    }
}

/// Compute rotation-minimizing frames along a curve using parallel transport.
///
/// Parallel transport produces frames that rotate as little as possible
/// around the tangent vector, avoiding the sudden flips that can occur
/// with Frenet-Serret frames at inflection points.
///
/// # Algorithm
///
/// Uses the double reflection method by Wang et al. (2008) for robust
/// parallel transport without singularities.
///
/// # Parameters
///
/// - `curve`: The curve to compute frames for
/// - `n`: Number of frames to compute (must be >= 2)
///
/// # Returns
///
/// Vector of `n` frames evenly spaced in parameter space along the curve.
///
/// # Example
///
/// ```
/// use curve_types::{Polyline, parallel_transport_frames};
/// use nalgebra::Point3;
///
/// let curve = Polyline::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 1.0, 0.0),
/// ]);
///
/// let frames = parallel_transport_frames(&curve, 10);
/// assert_eq!(frames.len(), 10);
///
/// // All frames are orthonormal
/// for frame in &frames {
///     assert!(frame.is_orthonormal(1e-10));
/// }
/// ```
#[must_use]
pub fn parallel_transport_frames<C: Curve + ?Sized>(curve: &C, n: usize) -> Vec<Frame> {
    let n = n.max(2);
    let mut frames = Vec::with_capacity(n);

    // Compute the first frame
    let t0 = 0.0;
    let p0 = curve.point_at(t0);
    let tangent0 = curve.tangent_at(t0);

    // Choose initial normal perpendicular to tangent
    let up = if tangent0.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::y()
    };
    let initial_frame = Frame::from_tangent_and_up(p0, tangent0, up, t0);
    frames.push(initial_frame);

    // Propagate frame using parallel transport
    for i in 1..n {
        let t = i as f64 / (n - 1) as f64;
        let p = curve.point_at(t);
        let tangent = curve.tangent_at(t);

        let prev = &frames[i - 1];
        let new_frame = transport_frame(prev, p, tangent, t);
        frames.push(new_frame);
    }

    frames
}

/// Compute parallel transport frames at specified parameter values.
///
/// # Parameters
///
/// - `curve`: The curve to compute frames for
/// - `params`: Parameter values where frames should be computed
///
/// # Returns
///
/// Vector of frames at the specified parameters.
#[must_use]
pub fn parallel_transport_frames_at<C: Curve + ?Sized>(curve: &C, params: &[f64]) -> Vec<Frame> {
    if params.is_empty() {
        return Vec::new();
    }

    let mut frames = Vec::with_capacity(params.len());

    // Compute the first frame
    let t0 = params[0];
    let p0 = curve.point_at(t0);
    let tangent0 = curve.tangent_at(t0);

    let up = if tangent0.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::y()
    };
    let initial_frame = Frame::from_tangent_and_up(p0, tangent0, up, t0);
    frames.push(initial_frame);

    // Propagate
    for &t in &params[1..] {
        let p = curve.point_at(t);
        let tangent = curve.tangent_at(t);

        let prev = frames.last().map_or(&initial_frame, |f| f);
        let new_frame = transport_frame(prev, p, tangent, t);
        frames.push(new_frame);
    }

    frames
}

/// Transport a frame to a new position and tangent using double reflection.
fn transport_frame(prev: &Frame, position: Point3<f64>, tangent: Vector3<f64>, t: f64) -> Frame {
    let tangent = tangent.normalize();

    // Double reflection method (Wang et al., 2008)
    let v1 = position - prev.position;
    let c1 = v1.dot(&v1);

    if c1 < 1e-20 {
        // Points are coincident, just update tangent
        let binormal = tangent.cross(&prev.normal);
        let binormal_norm = binormal.norm();

        let (normal, binormal) = if binormal_norm > 1e-10 {
            let binormal = binormal / binormal_norm;
            let normal = binormal.cross(&tangent);
            (normal, binormal)
        } else {
            (prev.normal, prev.binormal)
        };

        return Frame::new(position, tangent, normal, binormal, t);
    }

    // First reflection: reflect prev normal across plane perpendicular to v1
    let r_l = prev.normal - v1 * (2.0 / c1) * v1.dot(&prev.normal);

    // Compute t_l (intermediate tangent direction)
    let t_l = prev.tangent - v1 * (2.0 / c1) * v1.dot(&prev.tangent);

    // Second reflection
    let v2 = tangent - t_l;
    let c2 = v2.dot(&v2);

    let normal = if c2 < 1e-20 {
        r_l.normalize()
    } else {
        (r_l - v2 * (2.0 / c2) * v2.dot(&r_l)).normalize()
    };

    let binormal = tangent.cross(&normal);

    Frame::new(position, tangent, normal, binormal, t)
}

/// Compute Frenet-Serret frames along a curve.
///
/// The Frenet-Serret frame uses the curve's intrinsic geometry (curvature)
/// to define the normal direction. This can cause sudden flips at inflection
/// points where curvature changes sign.
///
/// For most applications, [`parallel_transport_frames`] is preferred.
///
/// # Parameters
///
/// - `curve`: The curve to compute frames for
/// - `n`: Number of frames to compute
///
/// # Returns
///
/// Vector of Frenet-Serret frames.
#[must_use]
pub fn frenet_serret_frames<C: Curve + ?Sized>(curve: &C, n: usize) -> Vec<Frame> {
    let n = n.max(2);

    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            let position = curve.point_at(t);
            let tangent = curve.tangent_at(t);
            let normal = curve.normal_at(t);
            let binormal = curve.binormal_at(t);

            Frame::new(position, tangent, normal, binormal, t)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    struct StraightLine;

    impl Curve for StraightLine {
        fn point_at(&self, t: f64) -> Point3<f64> {
            Point3::new(t * 10.0, 0.0, 0.0)
        }

        fn tangent_at(&self, _t: f64) -> Vector3<f64> {
            Vector3::x()
        }

        fn derivative_at(&self, _t: f64) -> Vector3<f64> {
            Vector3::new(10.0, 0.0, 0.0)
        }

        fn second_derivative_at(&self, _t: f64) -> Vector3<f64> {
            Vector3::zeros()
        }
    }

    #[test]
    fn test_frame_creation() {
        let frame = Frame::from_tangent_and_up(Point3::origin(), Vector3::x(), Vector3::z(), 0.0);

        assert!(frame.is_orthonormal(1e-10));
        assert_relative_eq!(frame.tangent, Vector3::x(), epsilon = 1e-10);
    }

    #[test]
    fn test_frame_local_to_world() {
        let frame =
            Frame::from_tangent_and_up(Point3::new(1.0, 2.0, 3.0), Vector3::x(), Vector3::z(), 0.0);

        let local = Point3::new(1.0, 0.0, 0.0);
        let world = frame.local_to_world(local);
        assert_relative_eq!(world.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(world.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(world.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_frame_interpolation() {
        let f1 = Frame::from_tangent_and_up(Point3::origin(), Vector3::x(), Vector3::z(), 0.0);
        let f2 = Frame::from_tangent_and_up(
            Point3::new(10.0, 0.0, 0.0),
            Vector3::x(),
            Vector3::z(),
            1.0,
        );

        let mid = f1.interpolate(&f2, 0.5);
        assert_relative_eq!(mid.position.x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(mid.t, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_parallel_transport_straight_line() {
        let line = StraightLine;
        let frames = parallel_transport_frames(&line, 5);

        assert_eq!(frames.len(), 5);

        // All frames should be orthonormal
        for frame in &frames {
            assert!(frame.is_orthonormal(1e-10));
        }

        // Normals should be consistent (no twisting on straight line)
        for i in 1..frames.len() {
            let dot = frames[i].normal.dot(&frames[i - 1].normal);
            assert_relative_eq!(dot, 1.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_quaternion_roundtrip() {
        let frame = Frame::from_tangent_and_up(
            Point3::new(1.0, 2.0, 3.0),
            Vector3::new(1.0, 1.0, 0.0).normalize(),
            Vector3::z(),
            0.5,
        );

        let q = frame.to_quaternion();
        let reconstructed = Frame::from_quaternion(frame.position, q, frame.t);

        assert_relative_eq!(frame.tangent, reconstructed.tangent, epsilon = 1e-10);
        assert_relative_eq!(frame.normal, reconstructed.normal, epsilon = 1e-10);
        assert_relative_eq!(frame.binormal, reconstructed.binormal, epsilon = 1e-10);
    }
}
