//! Ray casting for physics queries.
//!
//! This module provides ray casting functionality for distance queries against
//! collision shapes. It's used for rangefinder sensors, visibility checks, and
//! other distance-based queries.
//!
//! Shape-level intersection is delegated to [`cf_geometry::ray_cast`], which
//! supports all 10 shape types in local space. This module adds world-space
//! pose transforms and the scene-level [`raycast_scene`] query.

use std::sync::Arc;

use nalgebra::{Point3, Rotation3, UnitQuaternion, UnitVector3, Vector3};
use sim_types::Pose;

use cf_geometry::{Ray, Shape, ray_cast, ray_triangle};

use crate::collision::narrow::geom_to_shape;
use crate::types::{Data, GeomType, Model};

/// Result of a ray cast against a shape.
#[derive(Debug, Clone, Copy)]
pub struct RaycastHit {
    /// Distance from ray origin to hit point.
    pub distance: f64,
    /// Hit point in world coordinates.
    pub point: Point3<f64>,
    /// Surface normal at hit point (pointing away from surface).
    pub normal: Vector3<f64>,
}

impl RaycastHit {
    /// Create a new ray hit.
    #[must_use]
    pub fn new(distance: f64, point: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            distance,
            point,
            normal,
        }
    }
}

/// Safe vector normalization with fallback.
/// Returns the normalized vector if norm > threshold, otherwise returns the fallback.
#[inline]
fn safe_normalize(v: &Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { fallback }
}

/// Cast a ray against a collision shape.
///
/// Transforms the ray to the shape's local frame, delegates to
/// [`cf_geometry::ray_cast`] for the intersection test, and transforms
/// the result back to world coordinates.
///
/// # Arguments
///
/// * `shape` - The collision shape to test
/// * `shape_pose` - World pose of the shape
/// * `ray_origin` - Ray origin in world coordinates
/// * `ray_direction` - Ray direction (unit vector) in world coordinates
/// * `max_distance` - Maximum ray distance
///
/// # Returns
///
/// The closest ray hit, or `None` if the ray doesn't intersect within `max_distance`.
#[must_use]
pub fn raycast_shape(
    shape: &Shape,
    shape_pose: &Pose,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to shape's local space
    let local_origin = shape_pose.inverse_transform_point(&ray_origin);
    let local_dir = shape_pose.inverse_transform_vector(ray_direction.as_ref());
    let local_ray = Ray::new(local_origin, local_dir);

    // Delegate to cf-geometry (all 10 shapes, local-space)
    let hit = ray_cast(&local_ray, shape, max_distance)?;

    // Transform hit back to world space
    let world_point = shape_pose.transform_point(&hit.point);
    let world_normal = safe_normalize(&shape_pose.transform_vector(&hit.normal), hit.normal);
    Some(RaycastHit::new(hit.distance, world_point, world_normal))
}

/// Ray-triangle mesh intersection using BVH with early termination.
///
/// Uses `query_ray_closest` for optimal pruning: as closer hits are found,
/// distant BVH nodes are automatically culled. This is O(log n) average case.
#[must_use]
pub fn raycast_triangle_mesh_data(
    pose: &Pose,
    data: &crate::mesh::TriangleMeshData,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Validate max_distance
    if !max_distance.is_finite() || max_distance <= 0.0 {
        return None;
    }

    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Check for degenerate ray (zero direction after transform)
    let dir_norm = local_dir.norm();
    if dir_norm < 1e-10 {
        return None;
    }

    let local_ray = Ray::new(local_origin, local_dir);

    // Use BVH with early termination if available
    if let Some(bvh) = data.bvh() {
        // Precompute inverse direction for slab test
        // IEEE 754: 1.0/0.0 = ±∞, which the slab test handles correctly for axis-aligned rays
        let ray_dir_inv = Vector3::new(1.0 / local_dir.x, 1.0 / local_dir.y, 1.0 / local_dir.z);

        // Track the closest hit for both result and early termination
        let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;
        let mut cutoff = max_distance;

        // Use query_ray_closest with a callback that tests triangles and updates cutoff
        let result = bvh.query_ray_closest(&local_origin, &ray_dir_inv, max_distance, |tri_idx| {
            let (v0, v1, v2) = data.triangle_vertices(tri_idx)?;

            if let Some(hit) = ray_triangle(&local_ray, v0, v1, v2, cutoff) {
                if hit.distance < cutoff {
                    cutoff = hit.distance;
                    closest_hit = Some((hit.distance, hit.point, hit.normal));
                    return Some(hit.distance);
                }
            }
            None
        });

        // Use either the callback result or our tracked closest_hit
        let hit = closest_hit.or_else(|| {
            result.and_then(|(tri_idx, _)| {
                let (v0, v1, v2) = data.triangle_vertices(tri_idx)?;
                let h = ray_triangle(&local_ray, v0, v1, v2, max_distance)?;
                Some((h.distance, h.point, h.normal))
            })
        });

        hit.map(|(t, local_pt, local_normal)| {
            let world_pt = pose.transform_point(&local_pt);
            let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);
            RaycastHit::new(t, world_pt, world_normal)
        })
    } else {
        // Fallback: brute force test all triangles
        let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;

        for (face_idx, face) in data.triangles().iter().enumerate() {
            let v0 = data.vertices()[face[0] as usize];
            let v1 = data.vertices()[face[1] as usize];
            let v2 = data.vertices()[face[2] as usize];
            let _ = face_idx; // Index available if needed
            let cutoff = closest_hit.as_ref().map_or(max_distance, |h| h.0);

            if let Some(hit) = ray_triangle(&local_ray, v0, v1, v2, cutoff) {
                if closest_hit.is_none()
                    || hit.distance < closest_hit.as_ref().map_or(f64::MAX, |h| h.0)
                {
                    closest_hit = Some((hit.distance, hit.point, hit.normal));
                }
            }
        }

        closest_hit.map(|(t, local_pt, local_normal)| {
            let world_pt = pose.transform_point(&local_pt);
            let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);
            RaycastHit::new(t, world_pt, world_normal)
        })
    }
}

/// Result of a scene-level ray cast, identifying both the hit geom and
/// the intersection details.
#[derive(Debug, Clone, Copy)]
pub struct SceneRayHit {
    /// Index of the geom that was hit.
    pub geom_id: usize,
    /// Intersection details in world coordinates.
    pub hit: RaycastHit,
}

/// Build a world-space [`Pose`] from a geom's position and rotation matrix.
fn geom_pose(data: &Data, geom_id: usize) -> Pose {
    let pos = data.geom_xpos[geom_id];
    let mat = data.geom_xmat[geom_id];
    let quat = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(mat));
    Pose::from_position_rotation(Point3::from(pos), quat)
}

/// Track a closer hit, tightening the search cutoff.
#[inline]
fn track_closest(
    hit: RaycastHit,
    geom_id: usize,
    cutoff: &mut f64,
    closest: &mut Option<SceneRayHit>,
) {
    if hit.distance < *cutoff {
        *cutoff = hit.distance;
        *closest = Some(SceneRayHit { geom_id, hit });
    }
}

/// Cast a ray against all geoms in the scene, returning the closest hit.
///
/// Matches MuJoCo's `mj_ray` API. Iterates over all geoms, applying optional
/// filters, and returns the nearest intersection.
///
/// # Filters
///
/// - `bodyexclude` — skip all geoms attached to this body (e.g., to avoid
///   self-intersection). To exclude static/worldbody geoms, pass `Some(0)`.
/// - `geomgroup` — 6-element boolean array. A geom is skipped if
///   `!geomgroup[geom.group]`. Pass `None` to include all groups.
///
/// # Returns
///
/// The closest [`SceneRayHit`], or `None` if no geom is hit within `max_distance`.
#[must_use]
pub fn raycast_scene(
    model: &Model,
    data: &Data,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
    bodyexclude: Option<usize>,
    geomgroup: Option<&[bool; 6]>,
) -> Option<SceneRayHit> {
    let mut closest: Option<SceneRayHit> = None;
    let mut cutoff = max_distance;

    for geom_id in 0..model.ngeom {
        // Body exclusion filter
        if let Some(body_id) = bodyexclude {
            if model.geom_body[geom_id] == body_id {
                continue;
            }
        }

        // Geom group filter
        if let Some(groups) = geomgroup {
            if let Some(&g) = model.geom_group.get(geom_id) {
                let g = usize::try_from(g).unwrap_or(0);
                if g < 6 && !groups[g] {
                    continue;
                }
            }
        }

        let pose = geom_pose(data, geom_id);
        let geom_type = model.geom_type[geom_id];

        // Primitive shapes via geom_to_shape (Sphere, Box, Capsule, Cylinder, Ellipsoid)
        if let Some(shape) = geom_to_shape(geom_type, model.geom_size[geom_id]) {
            if let Some(hit) = raycast_shape(&shape, &pose, ray_origin, ray_direction, cutoff) {
                track_closest(hit, geom_id, &mut cutoff, &mut closest);
            }
        }

        // Handle types not covered by geom_to_shape
        match geom_type {
            GeomType::Plane => {
                // MuJoCo convention: plane normal = local +Z, passes through geom origin
                let shape = Shape::plane(Vector3::z(), 0.0);
                if let Some(hit) = raycast_shape(&shape, &pose, ray_origin, ray_direction, cutoff) {
                    track_closest(hit, geom_id, &mut cutoff, &mut closest);
                }
            }
            GeomType::Mesh => {
                if let Some(mesh_id) = model.geom_mesh[geom_id] {
                    if let Some(hit) = raycast_triangle_mesh_data(
                        &pose,
                        model.mesh_data[mesh_id].as_ref(),
                        ray_origin,
                        ray_direction,
                        cutoff,
                    ) {
                        track_closest(hit, geom_id, &mut cutoff, &mut closest);
                    }
                }
            }
            GeomType::Hfield => {
                if let Some(hfield_id) = model.geom_hfield[geom_id] {
                    let shape = Shape::height_field(Arc::clone(&model.hfield_data[hfield_id]));
                    if let Some(hit) =
                        raycast_shape(&shape, &pose, ray_origin, ray_direction, cutoff)
                    {
                        track_closest(hit, geom_id, &mut cutoff, &mut closest);
                    }
                }
            }
            GeomType::Sdf => {
                if let Some(shape_id) = model.geom_shape[geom_id] {
                    let grid = model.shape_data[shape_id].sdf_grid_arc();
                    let shape = Shape::sdf(grid);
                    if let Some(hit) =
                        raycast_shape(&shape, &pose, ray_origin, ray_direction, cutoff)
                    {
                        track_closest(hit, geom_id, &mut cutoff, &mut closest);
                    }
                }
            }
            _ => {} // Already handled by geom_to_shape above
        }
    }

    closest
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_raycast_sphere_hit() {
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.point.z, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_sphere_miss() {
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_sphere_max_distance() {
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 10.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        // Hit is at t=9, which is beyond max_distance=5
        let hit = raycast_shape(&shape, &pose, origin, direction, 5.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_plane_hit() {
        let shape = Shape::plane(Vector3::z(), 0.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 5.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_plane_parallel() {
        let shape = Shape::plane(Vector3::z(), 0.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::x()); // Parallel to plane

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_box_hit() {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_box_miss() {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let pose = Pose::from_position(Point3::new(5.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_capsule_hit_body() {
        let shape = Shape::capsule(2.0, 0.5);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        // Should hit the cap sphere at z = 5 - 2 - 0.5 = 2.5
        assert_relative_eq!(hit.distance, 2.5, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_cylinder_hit() {
        let shape = Shape::cylinder(1.0, 0.5);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        // Should hit the bottom cap at z = 5 - 1 = 4
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_ellipsoid_hit() {
        let shape = Shape::ellipsoid(Vector3::new(1.0, 1.0, 2.0));
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0).unwrap();

        // Should hit at z = 5 - 2 = 3
        assert_relative_eq!(hit.distance, 3.0, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_shape_dispatch() {
        let sphere = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&sphere, &pose, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn test_ray_triangle_intersection() {
        let ray = Ray::new(Point3::new(0.0, 0.0, 1.0), -Vector3::z());
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let hit = ray_triangle(&ray, v0, v1, v2, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 1.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, 1.0, epsilon = 1e-6);
    }

    // =========================================================================
    // Edge case tests
    // =========================================================================

    #[test]
    fn test_raycast_sphere_inside() {
        // Ray origin inside sphere — should return exit point
        let shape = Shape::sphere(1.0);
        let pose = Pose::identity();
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());
        let hit = raycast_shape(&shape, &pose, origin, direction, 10.0);
        assert!(hit.is_some());
    }

    #[test]
    #[should_panic(expected = "all radii must be positive")]
    fn test_raycast_ellipsoid_zero_radius() {
        // Ellipsoid with zero radius panics at construction (cf-geometry invariant)
        let _shape = Shape::ellipsoid(Vector3::new(0.0, 1.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "normal must be non-zero")]
    fn test_raycast_plane_zero_normal() {
        // Plane with zero normal panics at construction (cf-geometry invariant)
        let _shape = Shape::plane(Vector3::zeros(), 0.0);
    }

    #[test]
    fn test_raycast_max_distance_zero() {
        // Zero max_distance should return None
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 0.5));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, 0.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_negative_max_distance() {
        // Negative max_distance should return None
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&shape, &pose, origin, direction, -10.0);
        assert!(hit.is_none());
    }

    // =========================================================================
    // Scene-level raycast tests
    // =========================================================================

    /// Push a sphere geom attached to `body_id` at the given local position.
    fn push_sphere_geom(model: &mut Model, body_id: usize, pos: Vector3<f64>, radius: f64) {
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(body_id);
        model.geom_pos.push(pos);
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(radius, radius, radius));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(radius);
        model.geom_mesh.push(None);
        model.geom_hfield.push(None);
        model.geom_shape.push(None);
        model.geom_group.push(0);
        model.geom_rgba.push([0.5, 0.5, 0.5, 1.0]);
        model.ngeom += 1;
    }

    /// Build a minimal model+data with two sphere geoms along +Z.
    /// Geom 0: sphere r=1 at z=3. Geom 1: sphere r=1 at z=8.
    fn make_scene_model() -> (Model, Data) {
        let mut model = Model::empty();
        model.nbody = 1; // just world body

        // Two sphere geoms — body IDs 1 and 2 (for exclusion tests)
        push_sphere_geom(&mut model, 1, Vector3::zeros(), 1.0);
        push_sphere_geom(&mut model, 2, Vector3::zeros(), 1.0);

        let mut data = model.make_data();

        // Manually place geom world positions (skip forward kinematics)
        data.geom_xpos = vec![
            Vector3::new(0.0, 0.0, 3.0), // geom 0 at z=3
            Vector3::new(0.0, 0.0, 8.0), // geom 1 at z=8
        ];
        data.geom_xmat = vec![nalgebra::Matrix3::identity(); 2];

        (model, data)
    }

    #[test]
    fn test_raycast_scene_hits_closest() {
        let (model, data) = make_scene_model();
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let result = raycast_scene(&model, &data, origin, direction, 100.0, None, None);
        let hit = result.unwrap();

        // Closer sphere at z=3 (r=1) → hit at z=2
        assert_eq!(hit.geom_id, 0);
        assert_relative_eq!(hit.hit.distance, 2.0, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_scene_bodyexclude() {
        let (model, data) = make_scene_model();
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        // Exclude body 1 (has geom 0) — should hit the farther sphere on body 2
        let result = raycast_scene(&model, &data, origin, direction, 100.0, Some(1), None);
        let hit = result.unwrap();

        assert_eq!(hit.geom_id, 1);
        assert_relative_eq!(hit.hit.distance, 7.0, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_scene_geomgroup_filter() {
        let (mut model, mut data) = make_scene_model();

        // Put geom 0 in group 1, geom 1 in group 0
        model.geom_group[0] = 1;
        model.geom_group[1] = 0;

        // Re-set positions (make_scene_model already did this, but model changed)
        data.geom_xpos = vec![Vector3::new(0.0, 0.0, 3.0), Vector3::new(0.0, 0.0, 8.0)];
        data.geom_xmat = vec![nalgebra::Matrix3::identity(); 2];

        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        // Only include group 0 → geom 0 (group 1) is filtered out
        let groups = [true, false, false, false, false, false];
        let result = raycast_scene(&model, &data, origin, direction, 100.0, None, Some(&groups));
        let hit = result.unwrap();

        assert_eq!(hit.geom_id, 1); // The farther sphere in group 0
    }

    #[test]
    fn test_raycast_scene_no_hit() {
        let (model, data) = make_scene_model();
        let origin = Point3::origin();
        // Fire ray in -Z — away from both spheres
        let direction = UnitVector3::new_normalize(-Vector3::z());

        let result = raycast_scene(&model, &data, origin, direction, 100.0, None, None);
        assert!(result.is_none());
    }

    #[test]
    fn test_raycast_convex_mesh_not_bounding_sphere() {
        // A long thin tetrahedron along Z. Its bounding sphere is much larger
        // than the hull in the X/Y directions. A ray aimed at (0.8, 0, 5) in +Z
        // would hit the bounding sphere but must miss the actual hull.
        let vertices = vec![
            Point3::new(0.0, 0.0, -2.0),
            Point3::new(0.1, 0.0, 2.0),
            Point3::new(-0.05, 0.087, 2.0),
            Point3::new(-0.05, -0.087, 2.0),
        ];
        let hull = cf_geometry::convex_hull(&vertices, None).unwrap();
        let shape = Shape::convex_mesh(hull);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));

        // Ray at x=0.8 — well outside the thin hull but inside its bounding sphere
        let origin = Point3::new(0.8, 0.0, 0.0);
        let direction = UnitVector3::new_normalize(Vector3::z());
        let hit = raycast_shape(&shape, &pose, origin, direction, 20.0);
        assert!(hit.is_none(), "Ray should miss the thin hull");

        // Ray at x=0 — should hit the hull
        let origin_center = Point3::new(0.0, 0.0, 0.0);
        let hit_center = raycast_shape(&shape, &pose, origin_center, direction, 20.0);
        assert!(hit_center.is_some(), "Ray along center should hit the hull");
    }

    #[test]
    fn test_raycast_scene_plane_geom() {
        let mut model = Model::empty();
        model.nbody = 1;

        // Add a plane geom (ground at z=0)
        model.geom_type.push(GeomType::Plane);
        model.geom_body.push(0);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(10.0, 10.0, 0.01)); // MuJoCo plane size
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.0);
        model.geom_mesh.push(None);
        model.geom_hfield.push(None);
        model.geom_shape.push(None);
        model.geom_group.push(0);
        model.geom_rgba.push([0.5, 0.5, 0.5, 1.0]);
        model.ngeom = 1;

        let mut data = model.make_data();
        // Plane at z=2 with identity orientation (normal = +Z)
        data.geom_xpos = vec![Vector3::new(0.0, 0.0, 2.0)];
        data.geom_xmat = vec![nalgebra::Matrix3::identity()];

        let origin = Point3::new(0.0, 0.0, 0.0);
        let direction = UnitVector3::new_normalize(Vector3::z());

        let result = raycast_scene(&model, &data, origin, direction, 100.0, None, None);
        let hit = result.unwrap();
        assert_eq!(hit.geom_id, 0);
        assert_relative_eq!(hit.hit.distance, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_scene_mesh_geom() {
        let mut model = Model::empty();
        model.nbody = 1;

        // Build a simple triangle mesh (a quad made of 2 triangles at z=3)
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2, 0, 2, 3];
        let mesh_data = crate::mesh::TriangleMeshData::new(vertices, indices);

        model.mesh_data.push(std::sync::Arc::new(mesh_data));
        model.nmesh = 1;

        // Add a mesh geom
        model.geom_type.push(GeomType::Mesh);
        model.geom_body.push(0);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(1.0, 1.0, 1.0));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(1.5);
        model.geom_mesh.push(Some(0)); // references mesh_data[0]
        model.geom_hfield.push(None);
        model.geom_shape.push(None);
        model.geom_group.push(0);
        model.geom_rgba.push([0.5, 0.5, 0.5, 1.0]);
        model.ngeom = 1;

        let mut data = model.make_data();
        // Mesh at z=3
        data.geom_xpos = vec![Vector3::new(0.0, 0.0, 3.0)];
        data.geom_xmat = vec![nalgebra::Matrix3::identity()];

        let origin = Point3::new(0.0, 0.0, 0.0);
        let direction = UnitVector3::new_normalize(Vector3::z());

        let result = raycast_scene(&model, &data, origin, direction, 100.0, None, None);
        let hit = result.unwrap();
        assert_eq!(hit.geom_id, 0);
        assert_relative_eq!(hit.hit.distance, 3.0, epsilon = 1e-6);
    }
}
