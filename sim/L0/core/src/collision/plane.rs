//! Collision detection against infinite planes (sphere, box, capsule, cylinder, ellipsoid).

use super::narrow::{GEOM_EPSILON, make_contact_from_geoms};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Collision between a plane and another geometry.
///
/// Dispatches to specialized implementations based on the other geometry's type.
/// The plane is always treated as an infinite half-space with normal along its
/// local Z-axis.
// Plane-mesh dispatcher takes the full per-geom context; inlined as a single function so the SAT loop and contact emission read top-to-bottom.
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
) -> Vec<Contact> {
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
                // Contact position: midpoint between sphere surface and plane surface.
                // sphere_surface = center - normal * radius
                // plane_surface  = center - normal * center_dist
                // midpoint = center - normal * midpoint(radius, center_dist)
                let contact_pos = other_pos - plane_normal * f64::midpoint(radius, center_dist);
                // Contact normal points from other_geom toward plane_geom (from ball into plane = -plane_normal)
                // But for force calculation, we want the force to push the ball OUT of the plane,
                // so we use +plane_normal for the contact force direction.
                // Store the "push direction" as the normal to simplify force calculation.
                vec![make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal, // Points UP, away from plane (push direction)
                    penetration,
                    plane_geom,
                    other_geom,
                    margin,
                )]
            } else {
                vec![]
            }
        }
        GeomType::Box => {
            // Multi-contact box-plane: return up to 4 contacts (all bottom-face
            // corners), matching MuJoCo's mjc_PlaneBox algorithm.
            //
            // Algorithm (from engine_collision_primitive.c:~178):
            //   For each of the 8 box corners (bitmask i = 0..7):
            //     corner_offset = ±half.x * rx ± half.y * ry ± half.z * rz
            //     ldist = dot(normal, corner_offset)
            //   Filter: ldist <= 0 selects the bottom face (max 4 of 8).
            //   Inclusion: if the DEEPEST bottom corner is within margin, emit
            //   ALL bottom-face corners with their individual depths.
            //
            // Why all-or-nothing: on a tilted plane, per-corner depth filtering
            // produces oscillating contact counts (1–3 per step) that destabilize
            // the noslip postprocessor. Emitting all 4 bottom-face corners
            // whenever ANY is in contact ensures stable support. Corners with
            // negative depth (slightly above plane) produce near-zero constraint
            // forces — the solver handles this naturally.
            // See MULTI_CONTACT_ANALYSIS.md Appendix A for verification.
            let half = other_size;
            let rx = other_mat.column(0).into_owned();
            let ry = other_mat.column(1).into_owned();
            let rz = other_mat.column(2).into_owned();

            // Signed distance from box center to plane (positive = above plane)
            let dist = plane_normal.dot(&other_pos) - plane_distance;

            // Pass 1: collect bottom-face corners (ldist <= 0) and find deepest
            let mut bottom_corners: [(Vector3<f64>, f64); 4] = Default::default();
            let mut n_bottom = 0usize;
            let mut min_ldist = f64::MAX; // most negative = deepest

            for i in 0u8..8 {
                let sx = if i & 1 != 0 { 1.0 } else { -1.0 };
                let sy = if i & 2 != 0 { 1.0 } else { -1.0 };
                let sz = if i & 4 != 0 { 1.0 } else { -1.0 };

                let corner_offset = rx * (sx * half.x) + ry * (sy * half.y) + rz * (sz * half.z);
                let ldist = plane_normal.dot(&corner_offset);

                if ldist <= 0.0 && n_bottom < 4 {
                    bottom_corners[n_bottom] = (corner_offset, ldist);
                    n_bottom += 1;
                    if ldist < min_ldist {
                        min_ldist = ldist;
                    }
                }
            }

            // Pass 2: if the deepest bottom corner is within margin, emit all
            let deepest_corner_dist = dist + min_ldist; // signed dist of deepest
            if n_bottom > 0 && deepest_corner_dist <= margin {
                let mut contacts = Vec::with_capacity(n_bottom);
                for &(corner_offset, ldist) in &bottom_corners[..n_bottom] {
                    let corner_depth = -(dist + ldist);
                    let corner_world = other_pos + corner_offset;
                    let contact_pos = corner_world + plane_normal * (corner_depth / 2.0);

                    contacts.push(make_contact_from_geoms(
                        model,
                        contact_pos,
                        plane_normal,
                        corner_depth,
                        plane_geom,
                        other_geom,
                        margin,
                    ));
                }
                contacts
            } else {
                vec![]
            }
        }
        GeomType::Capsule => {
            // Multi-contact capsule-plane: test both endpoint spheres
            // independently, matching MuJoCo's mjc_PlaneCapsule which
            // delegates each endpoint to mjraw_PlaneSphere.
            let radius = other_size.x;
            let half_length = other_size.y;
            let axis = other_mat.column(2).into_owned(); // Z is capsule axis

            let endpoints = [
                other_pos + axis * half_length,
                other_pos - axis * half_length,
            ];

            let mut contacts = Vec::with_capacity(2);
            for end in &endpoints {
                let dist = plane_normal.dot(end) - plane_distance;
                let penetration = radius - dist;

                if penetration > -margin {
                    let contact_pos = end - plane_normal * f64::midpoint(radius, dist);
                    contacts.push(make_contact_from_geoms(
                        model,
                        contact_pos,
                        plane_normal,
                        penetration,
                        plane_geom,
                        other_geom,
                        margin,
                    ));
                }
            }
            contacts
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
        GeomType::Plane => vec![],
        // Hfield pairs intercepted at broadphase level (collide_hfield_multi)
        GeomType::Hfield => unreachable!("hfield pairs routed before plane dispatch"),
        // SDF is dispatched before plane in collide_geoms()
        GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
    }
}

/// Cylinder-plane collision detection (internal helper).
///
/// Returns up to 4 contacts using MuJoCo's `mjc_PlaneCylinder` algorithm:
/// 2 rim points (near/far cap) + 2 triangle points on the near-cap disk
/// forming equilateral support.
///
/// # Algorithm (from `engine_collision_primitive.c`)
/// 1. Orient axis toward plane (flip if needed)
/// 2. Compute radial direction `vec` = projection of `-normal` onto radial plane
/// 3. Contact 1: near-cap rim point (deepest). Early exit if not within margin.
/// 4. Contact 2: far-cap rim point (same radial direction)
/// 5. Contacts 3–4: triangle points on near-cap disk at `±cross(vec, axis)`
///    offset by `sqrt(3)/2 * radius`, pulled back to `-0.5 * vec`
///
/// Uses all-or-nothing inclusion: if contact 1 (deepest) is within margin,
/// emit all candidate contacts that individually pass the margin test.
/// See `MULTI_CONTACT_ANALYSIS.md` Appendix A for rationale.
///
/// # Returns
/// `Vec` with 0–4 contacts.
#[inline]
// Plane-cylinder pair dispatcher takes the full per-geom context (poses, sizes, friction, margin).
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
) -> Vec<Contact> {
    let radius = cyl_size.x;
    let half_height = cyl_size.y;

    // Cylinder axis is local Z transformed to world
    let mut axis = cyl_mat.column(2).into_owned();

    // Signed distance from cylinder center to plane
    let dist0 = plane_normal.dot(&cyl_pos) - plane_d;

    // Projection of axis onto plane normal — flip axis so it points toward
    // the plane (prjaxis >= 0 means the near cap is at +axis * half_height).
    let mut prjaxis = plane_normal.dot(&axis);
    if prjaxis > 0.0 {
        axis = -axis;
        prjaxis = -prjaxis;
    }
    // Now: near cap = cyl_pos + axis * half_height, far cap = cyl_pos - axis * half_height
    // prjaxis <= 0 (axis points toward plane)

    // Radial direction: projection of -normal onto the plane perpendicular to axis,
    // scaled to radius. This gives the rim offset that pushes deepest into the plane.
    let raw_vec = -plane_normal - axis * (-prjaxis); // -normal + axis * prjaxis
    let vec_len = raw_vec.norm();

    let vec = if vec_len > GEOM_EPSILON {
        raw_vec * (radius / vec_len)
    } else {
        // Axis is parallel to normal (disk is parallel to plane).
        // Use cylinder's local X-axis as fallback radial direction.
        cyl_mat.column(0).into_owned() * radius
    };

    let vec_dot_normal = plane_normal.dot(&vec);

    // --- Contact 1: near-cap rim point (deepest) ---
    let pos1 = cyl_pos + axis * half_height + vec;
    let depth1 = -(dist0 + prjaxis * half_height + vec_dot_normal);

    // Early exit: if the deepest point isn't within margin, no contacts.
    if depth1 <= -margin {
        return vec![];
    }

    let mut contacts = Vec::with_capacity(4);
    let contact_pos1 = pos1 + plane_normal * (depth1 / 2.0);
    contacts.push(make_contact_from_geoms(
        model,
        contact_pos1,
        plane_normal,
        depth1,
        plane_geom,
        cyl_geom,
        margin,
    ));

    // --- Contact 2: far-cap rim point ---
    let pos2 = cyl_pos - axis * half_height + vec;
    let depth2 = -(dist0 - prjaxis * half_height + vec_dot_normal);

    if depth2 > -margin {
        let contact_pos2 = pos2 + plane_normal * (depth2 / 2.0);
        contacts.push(make_contact_from_geoms(
            model,
            contact_pos2,
            plane_normal,
            depth2,
            plane_geom,
            cyl_geom,
            margin,
        ));
    }

    // --- Contacts 3–4: triangle points on the near-cap disk ---
    // Two points offset sideways from the rim point by ±cross(vec, axis),
    // normalized and scaled by radius * sqrt(3)/2, pulled halfway back
    // from the rim (at -0.5 * vec). These form an equilateral triangle
    // inscribed in the near-cap disk with contact 1.
    let cross = vec.cross(&axis);
    let cross_len = cross.norm();

    if cross_len > GEOM_EPSILON {
        let side = cross * (radius * SQRT_3_OVER_2 / cross_len);

        // Triangle depth: near-cap depth with half the radial contribution
        // (these points are at -0.5*vec instead of +vec, so the radial
        // component contribution is -0.5 * vec_dot_normal instead of +vec_dot_normal)
        let depth_tri = -(dist0 + prjaxis * half_height - vec_dot_normal * 0.5);

        if depth_tri > -margin {
            let base = cyl_pos + axis * half_height - vec * 0.5;

            let pos3 = base + side;
            let contact_pos3 = pos3 + plane_normal * (depth_tri / 2.0);
            contacts.push(make_contact_from_geoms(
                model,
                contact_pos3,
                plane_normal,
                depth_tri,
                plane_geom,
                cyl_geom,
                margin,
            ));

            let pos4 = base - side;
            let contact_pos4 = pos4 + plane_normal * (depth_tri / 2.0);
            contacts.push(make_contact_from_geoms(
                model,
                contact_pos4,
                plane_normal,
                depth_tri,
                plane_geom,
                cyl_geom,
                margin,
            ));
        }
    }

    contacts
}

/// sqrt(3) / 2 — used for equilateral triangle inscribed in the cap disk.
const SQRT_3_OVER_2: f64 = 0.866_025_403_784_438_6;

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
/// `Vec` with one contact if ellipsoid penetrates plane, empty otherwise.
#[inline]
// Plane-capsule pair dispatcher takes the full per-geom context (poses, sizes, friction, margin).
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
) -> Vec<Contact> {
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
        return vec![];
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
        return vec![];
    }

    // Contact position: midpoint between ellipsoid surface and plane surface.
    // ellipsoid_surface = world_support (already on ellipsoid surface)
    // plane_surface = world_support + normal * depth
    // midpoint = world_support + normal * depth / 2
    let contact_pos = world_support + plane_normal * (depth / 2.0);

    vec![make_contact_from_geoms(
        model,
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        ell_geom,
        margin,
    )]
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

        let contacts = collide_with_plane(
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

        assert!(!contacts.is_empty(), "Cylinder should contact plane");
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
            contacts.is_empty(),
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

        let contacts = collide_with_plane(
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

        assert!(!contacts.is_empty(), "Tilted cylinder should contact plane");
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
            !contacts.is_empty(),
            "Horizontal cylinder should contact plane"
        );
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
            !contacts.is_empty(),
            "Ellipsoid (sphere) should contact plane"
        );
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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

        assert!(!contacts.is_empty(), "Tall ellipsoid should contact plane");
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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

        assert!(!contacts.is_empty(), "Wide ellipsoid should contact plane");
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
            !contacts.is_empty(),
            "Rotated ellipsoid should contact plane"
        );
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
            contacts.is_empty(),
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

        let contacts = collide_with_plane(
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
            !contacts.is_empty(),
            "Should detect contact even when axis parallel to normal"
        );
        let c = &contacts[0];
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

        let contacts = collide_with_plane(
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
        assert!(!contacts.is_empty(), "should have contact");
        let contact = &contacts[0];

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
