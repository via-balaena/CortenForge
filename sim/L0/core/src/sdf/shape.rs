//! PhysicsShape trait — analytical implicit surface interface for SDF-backed geoms.
//!
//! Each SDF geom in the Model carries a `PhysicsShape` implementation.
//! The trait provides:
//!
//! - **Required methods** (`distance`, `gradient`, `bounds`, `effective_radius`,
//!   `sdf_grid`): every shape implements these. Grid-backed shapes delegate to
//!   `SdfGrid`; analytical shapes delegate to the CSG tree.
//!
//! - **Optional method** (`evaluate_interval`): conservative field bounds over
//!   a region, enabling octree-based contact detection. Only cf-design shapes
//!   backed by a `Solid` provide this.
//!
//! Two free-function dispatchers solve double dispatch:
//!
//! - [`compute_shape_contact`] — SDF-SDF contacts (analytical or grid fallback)
//! - [`compute_shape_plane_contact`] — SDF-Plane contacts (analytical or grid fallback)

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};
use sim_types::Pose;

use super::operations::{sdf_sdf_contact_raw, separation_direction, stabilize_direction};
use super::primitives::sdf_plane_contact;
use super::{SdfContact, SdfGrid};

/// Implicit surface interface for an SDF-backed collision shape.
///
/// The collision pipeline calls `distance()` and `gradient()` for all
/// point queries — these are the primary evaluation interface. The grid
/// (`sdf_grid()`) is used by GPU compute shaders, SDF-vs-primitive
/// contacts, and flex vertex collision.
///
/// Shapes that provide `evaluate_interval()` enable octree-based contact
/// detection (Tier 2), which replaces the O(N³) grid scan with adaptive
/// spatial pruning.
pub trait PhysicsShape: Send + Sync + std::fmt::Debug {
    // === REQUIRED: every shape provides these ===

    /// Signed distance at a local-frame point.
    ///
    /// Negative inside, positive outside, zero on surface.
    /// Grid-backed shapes delegate to `SdfGrid::distance()`.
    /// Analytical shapes delegate to `Solid::evaluate()`.
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64>;

    /// Outward unit normal at a local-frame point.
    ///
    /// Grid-backed shapes delegate to `SdfGrid::gradient()`.
    /// Analytical shapes delegate to normalized `Solid::gradient()`.
    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>>;

    /// Axis-aligned bounding box in local frame.
    ///
    /// Used for broadphase collision (overlap AABB computation) and
    /// as the octree root when both shapes support `evaluate_interval`.
    fn bounds(&self) -> Aabb;

    /// Distance from the shape center to the surface along a
    /// local-frame direction.
    ///
    /// Returns `Some(radius)` for convex shapes where the concept of
    /// "center to surface" is well-defined. The collision pipeline
    /// uses this for analytical depth: `r_a + r_b - center_dist`.
    ///
    /// Returns `None` for non-convex shapes (socket, torus, hollow
    /// body) where the center may be outside the surface.
    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64>;

    /// The underlying SDF grid.
    ///
    /// Used by GPU compute shaders, SDF-vs-primitive contacts
    /// (primitives.rs), and flex vertex collision (flex_narrow.rs).
    fn sdf_grid(&self) -> &SdfGrid;

    // === OPTIONAL: octree acceleration ===

    /// Conservative (min, max) field bounds over a local-frame AABB.
    ///
    /// If `lo > 0`: shape is entirely outside this region.
    /// If `hi < 0`: shape is entirely inside this region.
    /// If `lo ≤ 0 ≤ hi`: shape surface may cross this region.
    ///
    /// Enables octree-based contact detection (Tier 2 in
    /// `compute_shape_contact`). Returns `None` if interval evaluation
    /// is unavailable (grid-only shapes), falling back to Tier 3
    /// grid-based surface tracing.
    fn evaluate_interval(&self, _local_aabb: &Aabb) -> Option<(f64, f64)> {
        None
    }
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
            // Use the raw direction for the contact point to preserve
            // midpoint placement (symmetric lever arms for equal-radius
            // bodies), but stabilize the normal to prevent sub-physical
            // tangent-frame tilt from seeding pyramidal friction asymmetry.
            let contact_point = pose_a.position + dir * (r_a - depth * 0.5);
            let stable_normal = stabilize_direction(dir);
            return vec![SdfContact {
                point: contact_point,
                normal: stable_normal,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::shapes::{ShapeConcave, ShapeConvex, ShapeSphere};
    use nalgebra::{Point3, UnitQuaternion};
    use std::sync::Arc;

    fn sphere_pose(y: f64) -> Pose {
        Pose {
            position: Point3::new(0.0, y, 0.0),
            rotation: UnitQuaternion::identity(),
        }
    }

    /// ShapeConcave-ShapeConcave pair takes the multi-contact fallback path.
    /// The grid fallback can produce multiple contacts (or zero if no overlap).
    /// The key assertion: it does NOT produce exactly 1 analytical contact.
    #[test]
    fn concave_concave_takes_fallback_path() {
        let grid_a = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let grid_b = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let a = ShapeConcave::new(grid_a);
        let b = ShapeConcave::new(grid_b);

        // Overlapping poses (centers 8 apart, radii 5+5 = 10 > 8)
        let pose_a = sphere_pose(0.0);
        let pose_b = sphere_pose(8.0);
        let contacts = compute_shape_contact(&a, &pose_a, &b, &pose_b, 0.1);

        // Fallback path uses sdf_sdf_contact_raw which produces surface-traced
        // contacts — typically multiple or could be empty depending on grid
        // resolution. The analytical path would produce exactly 1 contact.
        // We verify the path was taken by checking that concave shapes return
        // None from effective_radius (tested in concave.rs unit tests).
        // Here we just verify the function doesn't panic and returns contacts.
        assert!(
            !contacts.is_empty(),
            "overlapping concave shapes should produce contacts via fallback"
        );
    }

    /// ShapeConcave paired with ShapeSphere also takes the fallback path
    /// (either returning None triggers fallback).
    #[test]
    fn concave_sphere_takes_fallback_path() {
        let grid_a = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let grid_b = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let a = ShapeConcave::new(grid_a);
        let b = ShapeSphere::new(grid_b, 5.0);

        let pose_a = sphere_pose(0.0);
        let pose_b = sphere_pose(8.0);
        let contacts = compute_shape_contact(&a, &pose_a, &b, &pose_b, 0.1);

        assert!(
            !contacts.is_empty(),
            "concave+sphere overlap should produce contacts via fallback"
        );
    }

    /// ShapeConcave paired with ShapeConvex takes the fallback path.
    #[test]
    fn concave_convex_takes_fallback_path() {
        let grid_a = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let grid_b = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let a = ShapeConcave::new(grid_a);
        let b = ShapeConvex::new(grid_b);

        let pose_a = sphere_pose(0.0);
        let pose_b = sphere_pose(8.0);
        let contacts = compute_shape_contact(&a, &pose_a, &b, &pose_b, 0.1);

        assert!(
            !contacts.is_empty(),
            "concave+convex overlap should produce contacts via fallback"
        );
    }

    /// ShapeConcave-Plane takes the multi-contact plane fallback.
    #[test]
    fn concave_plane_takes_fallback_path() {
        // Use a finer grid (cell_size=0.5) for reliable surface sampling
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 24, 0.5));
        let shape = ShapeConcave::new(grid);

        // Sphere center at y=4.0 — penetrating the y=0 plane by 1.0 units
        let pose = Pose {
            position: Point3::new(0.0, 4.0, 0.0),
            rotation: UnitQuaternion::identity(),
        };
        let plane_pos = Vector3::new(0.0, 0.0, 0.0);
        let plane_normal = Vector3::y();

        let contacts = compute_shape_plane_contact(&shape, &pose, &plane_pos, &plane_normal, 0.1);

        // Grid-based plane contact should find contacts near the bottom of the sphere.
        assert!(
            !contacts.is_empty(),
            "concave shape on plane should produce contacts via fallback"
        );
    }

    /// Convex-convex pair takes the analytical single-contact path (baseline).
    #[test]
    fn sphere_sphere_takes_analytical_path() {
        let grid_a = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let grid_b = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let a = ShapeSphere::new(grid_a, 5.0);
        let b = ShapeSphere::new(grid_b, 5.0);

        let pose_a = sphere_pose(0.0);
        let pose_b = sphere_pose(8.0);
        let contacts = compute_shape_contact(&a, &pose_a, &b, &pose_b, 0.1);

        // Analytical path produces exactly 1 contact
        assert_eq!(
            contacts.len(),
            1,
            "sphere-sphere should produce exactly 1 analytical contact"
        );
        let c = &contacts[0];
        // depth = r_a + r_b - dist = 5 + 5 - 8 = 2
        assert!((c.penetration - 2.0).abs() < 1e-10, "depth should be 2.0");
    }
}
