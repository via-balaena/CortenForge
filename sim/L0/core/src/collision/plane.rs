//! Collision detection against infinite planes (sphere, box, capsule, cylinder, ellipsoid).

use super::narrow::{
    AXIS_HORIZONTAL_THRESHOLD, AXIS_VERTICAL_THRESHOLD, GEOM_EPSILON, make_contact_from_geoms,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Collision between a plane and another geometry.
///
/// Dispatches to specialized implementations based on the other geometry's type.
/// The plane is always treated as an infinite half-space with normal along its
/// local Z-axis.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
#[inline]
pub fn collide_with_plane(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    type2: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Option<Contact> {
    // Determine which is the plane
    let (
        plane_geom,
        other_geom,
        plane_pos,
        plane_mat,
        other_pos,
        other_mat,
        other_type,
        other_size,
    ) = if type1 == GeomType::Plane {
        (geom1, geom2, pos1, mat1, pos2, mat2, type2, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, type1, size1)
    };

    // Plane normal is the Z-axis of the plane's frame
    let plane_normal = plane_mat.column(2).into_owned();
    // Plane passes through its position
    let plane_distance = plane_normal.dot(&plane_pos);

    match other_type {
        GeomType::Sphere => {
            let radius = other_size.x;
            let center_dist = plane_normal.dot(&other_pos) - plane_distance;
            let penetration = radius - center_dist;

            if penetration > -margin {
                // Contact position is at the sphere surface toward the plane
                let contact_pos = other_pos - plane_normal * center_dist;
                // Contact normal points from other_geom toward plane_geom (from ball into plane = -plane_normal)
                // But for force calculation, we want the force to push the ball OUT of the plane,
                // so we use +plane_normal for the contact force direction.
                // Store the "push direction" as the normal to simplify force calculation.
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal, // Points UP, away from plane (push direction)
                    penetration,
                    plane_geom,
                    other_geom,
                    margin,
                ))
            } else {
                None
            }
        }
        GeomType::Box => {
            // Optimized box-plane: compute lowest corner directly instead of testing all 8
            //
            // The support point (lowest corner toward plane) uses the formula:
            //   corner = center + sum_i( -sign(n·r_i) * h_i * r_i )
            //
            // where r_i are box local axes in world space and h_i are half-extents.
            // The sign logic: if n·r_i > 0, the axis points "up" relative to the plane,
            // so we want the negative end (-h_i * r_i) to get the lowest point.
            //
            // Implementation: dx = -sign(n·rx), so corner = center + dx*hx*rx + ...
            // This is equivalent to the formula but avoids explicit negation.
            //
            // This is O(1) instead of O(8) per box.
            let half = other_size;
            let rx = other_mat.column(0).into_owned();
            let ry = other_mat.column(1).into_owned();
            let rz = other_mat.column(2).into_owned();

            // Compute -sign(n·axis) for each axis. When dot > 0, axis points toward
            // plane normal, so we pick the negative end (d = -1). When dot <= 0, we pick
            // the positive end (d = +1). This gives the corner furthest in -n direction.
            let dx = if plane_normal.dot(&rx) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dy = if plane_normal.dot(&ry) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dz = if plane_normal.dot(&rz) > 0.0 {
                -1.0
            } else {
                1.0
            };

            // Lowest corner is the one most toward the plane (-normal direction)
            let lowest_corner =
                other_pos + rx * (dx * half.x) + ry * (dy * half.y) + rz * (dz * half.z);

            let dist = plane_normal.dot(&lowest_corner) - plane_distance;
            let depth = -dist;

            if depth > -margin {
                // Contact position on plane surface (project corner onto plane)
                // This is consistent with sphere-plane which places contact at surface
                let contact_pos = lowest_corner - plane_normal * dist;
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal,
                    depth,
                    plane_geom,
                    other_geom,
                    margin,
                ))
            } else {
                None
            }
        }
        GeomType::Capsule => {
            // Check both end spheres of the capsule
            let radius = other_size.x;
            let half_length = other_size.y;
            let axis = other_mat.column(2).into_owned(); // Z is capsule axis

            let end1 = other_pos + axis * half_length;
            let end2 = other_pos - axis * half_length;

            let dist1 = plane_normal.dot(&end1) - plane_distance;
            let dist2 = plane_normal.dot(&end2) - plane_distance;

            let (closest_end, min_dist) = if dist1 < dist2 {
                (end1, dist1)
            } else {
                (end2, dist2)
            };

            let penetration = radius - min_dist;

            if penetration > -margin {
                let contact_pos = closest_end - plane_normal * min_dist;
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal,
                    penetration,
                    plane_geom,
                    other_geom,
                    margin,
                ))
            } else {
                None
            }
        }
        GeomType::Cylinder => {
            // Cylinder-plane collision: find deepest point on cylinder
            collide_cylinder_plane_impl(
                model,
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
                margin,
            )
        }
        GeomType::Ellipsoid => {
            // Ellipsoid-plane collision: find support point in plane normal direction
            collide_ellipsoid_plane_impl(
                model,
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
                margin,
            )
        }
        // INVARIANT: collide_geoms() dispatches mesh collision before plane collision.
        // If either geom is a mesh, collide_with_mesh() handles it—including mesh-plane.
        // This branch exists only for match exhaustiveness; reaching it indicates a bug.
        GeomType::Mesh => unreachable!(
            "mesh collision must be dispatched before plane collision in collide_geoms"
        ),
        // Plane-plane: two infinite half-spaces. Intersection is either empty, a plane,
        // or a half-space—none of which produce a meaningful contact point.
        GeomType::Plane => None,
        // Hfield is dispatched before plane in collide_geoms()
        GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
        // SDF is dispatched before plane in collide_geoms()
        GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
    }
}

/// Cylinder-plane collision detection (internal helper).
///
/// Finds the deepest penetrating point on the cylinder. For tilted/upright
/// cylinders, this is on the rim edge. For horizontal cylinders (axis parallel
/// to plane), the curved surface is checked.
///
/// # Algorithm
/// 1. Compute cylinder axis in world frame
/// 2. Determine if cylinder is "horizontal" (axis nearly parallel to plane)
/// 3. For non-horizontal: check rim points on both caps in the deepest direction
/// 4. For horizontal: check the curved surface point closest to plane
/// 5. Return the deepest penetrating point as contact
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane (points away from solid half-space)
/// - `plane_d`: Signed distance from origin to plane along normal
/// - `cyl_size`: [radius, half_height, unused]
///
/// # Returns
/// `Some(Contact)` if cylinder penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_plane_impl(
    model: &Model,
    plane_geom: usize,
    cyl_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    cyl_pos: Vector3<f64>,
    cyl_mat: Matrix3<f64>,
    cyl_size: Vector3<f64>,
    margin: f64,
) -> Option<Contact> {
    let radius = cyl_size.x;
    let half_height = cyl_size.y;

    // Cylinder axis is local Z transformed to world
    let cyl_axis = cyl_mat.column(2).into_owned();

    // How aligned is cylinder axis with plane normal?
    // axis_dot_signed: positive = axis points away from plane, negative = toward plane
    // axis_dot (abs): 1 = vertical, 0 = horizontal
    let axis_dot_signed = plane_normal.dot(&cyl_axis);
    let axis_dot = axis_dot_signed.abs();

    // Pre-compute radial direction (projection of plane normal onto radial plane)
    // Used in Case 1 (horizontal) and Case 3 (tilted)
    let radial = plane_normal - cyl_axis * axis_dot_signed;
    let radial_len = radial.norm();

    // Compute the deepest point on the cylinder
    let deepest_point = if axis_dot < AXIS_HORIZONTAL_THRESHOLD {
        // Case 1: Cylinder is horizontal (axis parallel to plane)
        // The deepest point is on the curved surface, directly below the axis
        if radial_len > GEOM_EPSILON {
            // Point on curved surface in the direction opposite to plane normal
            cyl_pos - (radial / radial_len) * radius
        } else {
            // Degenerate: plane normal exactly along axis (contradicts horizontal check)
            cyl_pos - plane_normal * radius
        }
    } else if axis_dot > AXIS_VERTICAL_THRESHOLD {
        // Case 2: Cylinder is vertical (axis perpendicular to plane)
        // The deepest point is the entire bottom rim (pick center for stability)
        // Determine which cap faces the plane
        let cap_dir = if axis_dot_signed > 0.0 {
            -cyl_axis // Bottom cap faces plane
        } else {
            cyl_axis // Top cap faces plane
        };
        cyl_pos + cap_dir * half_height
    } else {
        // Case 3: Cylinder is tilted
        // The deepest point is on one of the rim edges
        // Find the rim direction that points most into the plane
        // Note: radial_len = sqrt(1 - axis_dot²) > 0 since axis_dot < AXIS_VERTICAL_THRESHOLD
        let rim_dir = -radial / radial_len;

        // Determine which cap is lower (faces the plane more)
        let top_center = cyl_pos + cyl_axis * half_height;
        let bottom_center = cyl_pos - cyl_axis * half_height;

        let top_rim_point = top_center + rim_dir * radius;
        let bottom_rim_point = bottom_center + rim_dir * radius;

        let top_depth = -(plane_normal.dot(&top_rim_point) - plane_d);
        let bottom_depth = -(plane_normal.dot(&bottom_rim_point) - plane_d);

        if bottom_depth > top_depth {
            bottom_rim_point
        } else {
            top_rim_point
        }
    };

    // Compute penetration depth
    let signed_dist = plane_normal.dot(&deepest_point) - plane_d;
    let depth = -signed_dist;

    if depth <= -margin {
        return None;
    }

    // Contact point is on the plane surface
    let contact_pos = deepest_point + plane_normal * depth;

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        cyl_geom,
        margin,
    ))
}

/// Ellipsoid-plane collision detection (internal helper).
///
/// Finds the support point on the ellipsoid surface in the direction toward the plane.
/// Ellipsoid is defined by radii (a, b, c) = (rx, ry, rz) along local axes.
///
/// # Algorithm
///
/// For an ellipsoid `x²/a² + y²/b² + z²/c² = 1`, the support point in direction `-n`
/// (i.e., the point on the surface with outward normal parallel to `-n`) is:
///
/// ```text
/// p = -(r ⊙ r ⊙ n) / ||r ⊙ n||
///   = -(a²·nₓ, b²·nᵧ, c²·nᵤ) / ||(a·nₓ, b·nᵧ, c·nᵤ)||
/// ```
///
/// where `⊙` denotes element-wise (Hadamard) product.
///
/// **Derivation**: The outward normal at point `(x,y,z)` on the ellipsoid is
/// `∇f = (2x/a², 2y/b², 2z/c²)`. Setting this parallel to `n` and solving with
/// the ellipsoid constraint yields the support point formula.
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane
/// - `plane_d`: Signed distance from origin to plane
/// - `ell_radii`: Ellipsoid radii [rx, ry, rz]
///
/// # Returns
/// `Some(Contact)` if ellipsoid penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_ellipsoid_plane_impl(
    model: &Model,
    plane_geom: usize,
    ell_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    ell_pos: Vector3<f64>,
    ell_mat: Matrix3<f64>,
    ell_radii: Vector3<f64>,
    margin: f64,
) -> Option<Contact> {
    // Transform plane normal to ellipsoid local frame
    let local_normal = ell_mat.transpose() * plane_normal;

    // Compute support point in local frame
    // For ellipsoid, support in direction -n is: -(r² ⊙ n) / ||r ⊙ n||
    // where ⊙ is element-wise multiply and r = radii
    let scaled = Vector3::new(
        ell_radii.x * local_normal.x,
        ell_radii.y * local_normal.y,
        ell_radii.z * local_normal.z,
    );
    let scale_norm = scaled.norm();

    if scale_norm < GEOM_EPSILON {
        // Degenerate case: normal perpendicular to all radii (shouldn't happen)
        return None;
    }

    // Support point on ellipsoid surface in direction toward plane (negative normal)
    let local_support = -Vector3::new(
        ell_radii.x * scaled.x / scale_norm,
        ell_radii.y * scaled.y / scale_norm,
        ell_radii.z * scaled.z / scale_norm,
    );

    // Transform to world frame
    let world_support = ell_pos + ell_mat * local_support;

    // Check penetration depth
    let signed_dist = plane_normal.dot(&world_support) - plane_d;
    let depth = -signed_dist; // Positive = penetrating

    if depth <= -margin {
        return None;
    }

    // Contact point is on plane surface
    let contact_pos = world_support + plane_normal * depth;

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        ell_geom,
        margin,
    ))
}

// ============================================================================
// Tests for Primitive Collision Detection
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod primitive_collision_tests {
    use super::collide_with_plane;
    use crate::types::{GeomType, Model};
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    /// Helper to create a minimal Model for collision testing.
    fn make_collision_test_model(ngeom: usize) -> Model {
        let mut model = Model::empty();
        model.ngeom = ngeom;
        model.geom_type = vec![GeomType::Sphere; ngeom];
        model.geom_body = vec![0; ngeom];
        model.geom_pos = vec![Vector3::zeros(); ngeom];
        model.geom_quat = vec![UnitQuaternion::identity(); ngeom];
        model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); ngeom];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); ngeom];
        model.geom_condim = vec![3; ngeom]; // Default condim = 3 (sliding friction)
        model.geom_contype = vec![1; ngeom];
        model.geom_conaffinity = vec![1; ngeom];
        model.geom_margin = vec![0.0; ngeom];
        model.geom_gap = vec![0.0; ngeom];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; ngeom];
        model.geom_solref = vec![[0.02, 1.0]; ngeom];
        model.geom_name = vec![None; ngeom];
        model.geom_rbound = vec![1.0; ngeom];
        model.geom_mesh = vec![None; ngeom]; // No mesh geoms in test helper
        model.geom_priority = vec![0; ngeom];
        model.geom_solmix = vec![1.0; ngeom];
        model
    }

    // ========================================================================
    // Cylinder-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_cylinder_plane_upright_penetrating() {
        // Cylinder standing upright on plane, bottom rim penetrating
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;
        model.geom_size[0] = Vector3::new(10.0, 10.0, 0.1);

        // Geom 1: Cylinder (radius=0.3, half_height=0.5)
        // Center at z=0.4, so bottom cap center is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // [radius, half_height, unused]

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let cyl_mat = Matrix3::identity(); // Axis along +Z (upright)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_upright_no_contact() {
        // Cylinder above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 1.0); // Center at z=1.0, bottom at z=0.5
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(
            contact.is_none(),
            "Cylinder should not contact plane when above"
        );
    }

    #[test]
    fn test_cylinder_plane_tilted() {
        // Cylinder tilted 45° around X axis, verifying rim point depth calculation
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Cylinder with radius=0.3, half_height=0.5, tilted 45° around X
        // After rotation, the cylinder axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Cylinder;
        let radius = 0.3;
        let half_height = 0.5;
        model.geom_size[1] = Vector3::new(radius, half_height, 0.0);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Tilt cylinder 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let cyl_mat = rot.to_rotation_matrix().into_inner();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected deepest point calculation (Case 3: tilted cylinder):
        // Cylinder axis in world frame: (0, sin(45°), cos(45°)) = (0, 0.707, 0.707)
        // Plane normal: (0, 0, 1)
        // axis_dot_signed = plane_normal · cyl_axis = cos(45°) ≈ 0.707
        // radial = plane_normal - cyl_axis * axis_dot_signed
        //        = (0, 0, 1) - (0, 0.707, 0.707) * 0.707
        //        = (0, -0.5, 0.5)
        // rim_dir = -radial / ||radial|| = (0, 0.707, -0.707)
        //
        // Bottom cap center: cyl_pos - cyl_axis * half_height
        //                  = (0, 0, 0.5) - (0, 0.354, 0.354) = (0, -0.354, 0.146)
        // Bottom rim point: bottom_center + rim_dir * radius
        //                 = (0, -0.354, 0.146) + (0, 0.212, -0.212) = (0, -0.142, -0.066)
        //
        // Top cap center: cyl_pos + cyl_axis * half_height
        //               = (0, 0, 0.5) + (0, 0.354, 0.354) = (0, 0.354, 0.854)
        // Top rim point: top_center + rim_dir * radius
        //              = (0, 0.354, 0.854) + (0, 0.212, -0.212) = (0, 0.566, 0.642)
        //
        // Bottom rim z = -0.066 (below plane) → depth = 0.066
        // Top rim z = 0.642 (above plane) → no penetration
        // Deepest point is bottom rim with depth ≈ 0.066
        let cyl_axis_y = sin_a;
        let cyl_axis_z = cos_a;
        let bottom_center_z = cyl_pos.z - cyl_axis_z * half_height;
        // radial = (0, -axis_dot_signed * cyl_axis_y, 1 - axis_dot_signed * cyl_axis_z)
        let axis_dot_signed = cos_a;
        let radial_y = -axis_dot_signed * cyl_axis_y;
        let radial_z = 1.0 - axis_dot_signed * cyl_axis_z;
        let radial_len = radial_y.hypot(radial_z);
        let rim_dir_z = -radial_z / radial_len;
        let bottom_rim_z = bottom_center_z + rim_dir_z * radius;
        let expected_depth = -bottom_rim_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Tilted cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_horizontal() {
        // Cylinder lying flat (axis parallel to plane)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // radius=0.3, half_height=0.5

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();

        // Rotate 90 degrees around X axis (cylinder now horizontal, axis along Y)
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let cyl_mat = rot.to_rotation_matrix().into_inner();

        // Position center at z = radius - epsilon for penetration
        let cyl_pos = Vector3::new(0.0, 0.0, 0.2); // Below radius, should penetrate

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(
            contact.is_some(),
            "Horizontal cylinder should contact plane"
        );
        let c = contact.unwrap();
        // Penetration should be radius - z_center = 0.3 - 0.2 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    // ========================================================================
    // Ellipsoid-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_ellipsoid_plane_sphere_case() {
        // Ellipsoid with equal radii (sphere) for validation against known sphere formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Ellipsoid with rx=ry=rz=0.5 (degenerates to sphere with radius 0.5)
        // Center at z=0.4, so bottom (support point) is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let ell_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Ellipsoid (sphere) should contact plane");
        let c = contact.unwrap();
        // Penetration = radius - z = 0.5 - 0.4 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_z() {
        // Ellipsoid stretched along Z axis (tall and thin)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8); // Tall ellipsoid

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.7); // Bottom at z = -0.1
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Tall ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.8 - 0.7 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_x() {
        // Ellipsoid stretched along X axis (wide and flat)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.8, 0.3, 0.2); // Wide ellipsoid (short in Z)

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.15); // Bottom at z = -0.05 (penetrating)
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Wide ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.2 - 0.15 = 0.05
        assert_relative_eq!(c.depth, 0.05, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_rotated() {
        // Ellipsoid rotated 45° around X axis, verifying support point formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Tall ellipsoid with radii (0.2, 0.2, 0.8), rotated 45° around X
        // After rotation, the long axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Rotate 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let ell_mat = rot.to_rotation_matrix().into_inner();
        let ell_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected support point calculation:
        // Local plane normal: n_local = R^T * (0,0,1) = (0, sin(45°), cos(45°))
        //   (R rotates around X, so R^T maps world Z to local (0, sin, cos))
        // scaled = r ⊙ n_local = (0, 0.2*sin, 0.8*cos) = (0, 0.1414, 0.5657)
        // ||scaled|| = sqrt(0.2² * sin² + 0.8² * cos²) ≈ 0.583
        // local_support = -(r² ⊙ n) / ||scaled||
        //               = -(0, 0.04*sin, 0.64*cos) / 0.583
        //               = (0, -0.0485, -0.776)
        // world_support = center + R * local_support
        // The z-component: 0.5 + (local_y * sin + local_z * cos)
        //                = 0.5 + (-0.0485 * 0.707 + (-0.776) * 0.707)
        //                = 0.5 - 0.583 = -0.083
        // Expected depth = -(-0.083) = 0.083 (positive, penetrating)
        let ry = 0.2;
        let rz = 0.8;
        let scaled_y = ry * sin_a;
        let scaled_z = rz * cos_a;
        let scale_norm = scaled_y.hypot(scaled_z);
        let local_support_y = -(ry * ry * sin_a) / scale_norm;
        let local_support_z = -(rz * rz * cos_a) / scale_norm;
        let world_support_z = ell_pos.z + local_support_y * sin_a + local_support_z * cos_a;
        let expected_depth = -world_support_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(contact.is_some(), "Rotated ellipsoid should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_no_contact() {
        // Ellipsoid above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.3, 0.3, 0.3);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 1.0); // Far above plane
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        assert!(
            contact.is_none(),
            "Ellipsoid should not contact plane when above"
        );
    }

    // ========================================================================
    // Edge Cases and Numerical Stability
    // ========================================================================

    #[test]
    fn test_cylinder_plane_axis_parallel_to_normal() {
        // Cylinder axis exactly parallel to plane normal (degenerate case)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Bottom at z = -0.1
        let cyl_mat = Matrix3::identity(); // Axis along Z (parallel to plane normal)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        );

        // Should still detect contact at the bottom rim
        assert!(
            contact.is_some(),
            "Should detect contact even when axis parallel to normal"
        );
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_contact_frame_is_valid() {
        // Verify contact frame is orthonormal
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4);
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
            0.0, // margin
        )
        .expect("should have contact");

        // Normal should be unit length
        assert_relative_eq!(contact.normal.norm(), 1.0, epsilon = 1e-10);

        // Tangent vectors should be unit length
        assert_relative_eq!(contact.frame[0].norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(contact.frame[1].norm(), 1.0, epsilon = 1e-10);

        // All three should be mutually orthogonal
        assert_relative_eq!(contact.normal.dot(&contact.frame[0]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(contact.normal.dot(&contact.frame[1]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(
            contact.frame[0].dot(&contact.frame[1]),
            0.0,
            epsilon = 1e-10
        );
    }
}
