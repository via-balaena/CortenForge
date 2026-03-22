//! PhysicsShape trait — shape-specific contact metadata for SDF-backed geoms.
//!
//! Each SDF geom in the Model carries a `PhysicsShape` implementation that
//! provides shape-specific effective radius computation. The SDF grid handles
//! broadphase detection and surface tracing; the PhysicsShape provides the
//! analytical depth/normal/contact-point computation that the solver needs.
//!
//! Two free-function dispatchers solve double dispatch:
//!
//! - [`compute_shape_contact`] — SDF-SDF contacts (analytical or grid fallback)
//! - [`compute_shape_plane_contact`] — SDF-Plane contacts (analytical or grid fallback)

use nalgebra::Vector3;
use sim_types::Pose;

use super::operations::{sdf_sdf_contact_raw, separation_direction};
use super::primitives::sdf_plane_contact;
use super::{SdfContact, SdfGrid};

/// Physics metadata for an SDF-backed collision shape.
///
/// Each SDF geom in the Model carries a PhysicsShape implementation
/// that provides shape-specific contact computation. The SDF grid
/// handles broadphase detection; the PhysicsShape handles the
/// analytical depth/normal/contact-point computation that the solver
/// needs.
pub trait PhysicsShape: Send + Sync + std::fmt::Debug {
    /// Distance from the shape center to the surface along a
    /// local-frame direction.
    ///
    /// Returns `Some(radius)` for convex shapes where the concept of
    /// "center to surface" is well-defined. The collision pipeline
    /// uses this for analytical depth: `r_a + r_b - center_dist`.
    ///
    /// Returns `None` for non-convex shapes (socket, torus, hollow
    /// body) where the center may be outside the surface. The
    /// pipeline falls back to grid-based multi-contact surface
    /// tracing.
    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64>;

    /// The underlying SDF grid for broadphase detection and surface
    /// tracing fallback.
    fn sdf_grid(&self) -> &SdfGrid;
}

/// Compute contacts between two SDF-backed shapes.
///
/// If both shapes provide an analytical effective radius (`effective_radius`
/// returns `Some`), uses the analytical single-contact path:
/// depth = r_a + r_b - center_dist. Otherwise falls back to grid-based
/// multi-contact surface tracing via [`sdf_sdf_contact_raw`].
pub fn compute_shape_contact(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
) -> Vec<SdfContact> {
    let dir = separation_direction(pose_a, pose_b).unwrap_or_else(Vector3::z);

    // Transform to each shape's local frame
    let local_dir_a = pose_a.rotation.inverse() * dir;
    let local_dir_b = pose_b.rotation.inverse() * (-dir);

    // Analytical path: both convex
    if let (Some(r_a), Some(r_b)) = (
        a.effective_radius(&local_dir_a),
        b.effective_radius(&local_dir_b),
    ) {
        let center_dist = (pose_b.position - pose_a.position).norm();
        let depth = (r_a + r_b - center_dist).max(0.0);
        if depth > 0.0 || center_dist < r_a + r_b + margin {
            let contact_point = pose_a.position + dir * (r_a - depth * 0.5);
            return vec![SdfContact {
                point: contact_point,
                normal: dir,
                penetration: depth,
            }];
        }
        return vec![];
    }

    // Fallback: grid-based multi-contact surface tracing
    sdf_sdf_contact_raw(a.sdf_grid(), pose_a, b.sdf_grid(), pose_b, margin)
}

/// Compute contacts between an SDF-backed shape and an infinite plane.
///
/// If the shape provides an analytical effective radius, uses the analytical
/// single-contact path: depth = radius - distance_to_plane. Otherwise falls
/// back to grid-based multi-contact plane tracing via [`sdf_plane_contact`].
pub fn compute_shape_plane_contact(
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    plane_pos: &Vector3<f64>,
    plane_normal: &Vector3<f64>,
    margin: f64,
) -> Vec<SdfContact> {
    let local_dir = shape_pose.rotation.inverse() * (-*plane_normal);

    if let Some(radius) = shape.effective_radius(&local_dir) {
        let dist_to_plane = (shape_pose.position.coords - plane_pos).dot(plane_normal);
        let depth = radius - dist_to_plane;
        if depth > -margin {
            let contact_point = shape_pose.position - plane_normal * (radius - depth * 0.5);
            return vec![SdfContact {
                point: contact_point,
                normal: *plane_normal,
                penetration: depth.max(0.0),
            }];
        }
        return vec![];
    }

    // Fallback: grid-based multi-contact plane tracing
    let plane_offset = plane_normal.dot(plane_pos);
    sdf_plane_contact(shape.sdf_grid(), shape_pose, plane_normal, plane_offset)
}
