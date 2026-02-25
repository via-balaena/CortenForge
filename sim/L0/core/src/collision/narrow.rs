//! Narrow-phase collision dispatch — selects analytical or GJK/EPA path for a geom pair.
//!
//! Contains the top-level `collide_geoms` dispatcher, the `geom_to_collision_shape` converter,
//! `make_contact_from_geoms` contact constructor, `apply_pair_overrides` for mechanism-2
//! contacts, and shared collision constants.

use super::hfield::collide_with_hfield;
use super::mesh_collide::collide_with_mesh;
use super::pair_convex::{
    collide_capsule_capsule, collide_sphere_box, collide_sphere_capsule, collide_sphere_sphere,
};
use super::pair_cylinder::{
    collide_box_box, collide_capsule_box, collide_cylinder_capsule, collide_cylinder_sphere,
};
use super::plane::collide_with_plane;
use super::sdf_collide::collide_with_sdf;

use super::{assign_friction, assign_imp, assign_ref, contact_param};
use crate::collision_shape::CollisionShape;
use crate::gjk_epa::gjk_epa_contact;
use crate::types::{
    Contact, ContactPair, ENABLE_OVERRIDE, GeomType, Model, compute_tangent_frame, enabled,
};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

// ============================================================================
// Collision constants
// ============================================================================

/// Minimum norm threshold for geometric operations.
///
/// Used to prevent division by zero and detect degenerate cases in collision
/// detection. This value is chosen to be:
/// - Small enough to not reject valid geometric configurations
/// - Large enough to avoid numerical instability near machine epsilon
///
/// For reference: f64::EPSILON ~= 2.2e-16, so 1e-10 provides ~6 orders of
/// magnitude of safety margin while still detecting near-degenerate cases.
pub const GEOM_EPSILON: f64 = 1e-10;

/// Threshold for cylinder axis being nearly vertical (perpendicular to plane).
/// When |cos(theta)| > 0.999 (theta < 2.6 deg), treat cylinder as vertical.
pub const AXIS_VERTICAL_THRESHOLD: f64 = 0.999;

/// Threshold for cylinder axis being nearly horizontal (parallel to plane).
/// When |cos(theta)| < 0.001 (theta > 89.9 deg), treat cylinder as horizontal.
pub const AXIS_HORIZONTAL_THRESHOLD: f64 = 0.001;

/// Threshold for detecting cylinder cap collision in cylinder-capsule.
/// When normal is within ~45 deg of cylinder axis (cos > 0.7), treat as cap collision.
pub const CAP_COLLISION_THRESHOLD: f64 = 0.7;

// ============================================================================
// Narrow-phase dispatch
// ============================================================================

/// Narrow-phase collision between two geometries.
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
#[allow(clippy::items_after_statements)] // use statement placed after special cases for readability
#[allow(clippy::too_many_arguments)]
pub fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Fast path: handle all analytical collision cases first
    // These avoid the expensive quaternion conversion and GJK/EPA

    // Special case: SDF collision (before mesh/hfield/plane — SDF has its own
    // contact functions for all shapes including Mesh, Hfield, and Plane)
    if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
        return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: mesh collision (has its own BVH-accelerated path)
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: height field collision
    if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
        return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-sphere collision (analytical, more robust than GJK/EPA)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2, margin);
    }

    // Special case: capsule-capsule collision (analytical, much faster than GJK/EPA)
    if type1 == GeomType::Capsule && type2 == GeomType::Capsule {
        return collide_capsule_capsule(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-capsule collision
    if (type1 == GeomType::Sphere && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Sphere)
    {
        return collide_sphere_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-box collision (analytical)
    if (type1 == GeomType::Sphere && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Sphere)
    {
        return collide_sphere_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: capsule-box collision (analytical)
    if (type1 == GeomType::Capsule && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Capsule)
    {
        return collide_capsule_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: box-box collision (SAT)
    if type1 == GeomType::Box && type2 == GeomType::Box {
        return collide_box_box(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: cylinder-sphere collision (analytical)
    if (type1 == GeomType::Cylinder && type2 == GeomType::Sphere)
        || (type1 == GeomType::Sphere && type2 == GeomType::Cylinder)
    {
        return collide_cylinder_sphere(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: cylinder-capsule collision (analytical with GJK/EPA fallback)
    // Analytical solution handles common cases; degenerate cases fall through to GJK/EPA
    if (type1 == GeomType::Cylinder && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Cylinder)
    {
        if let Some(contact) = collide_cylinder_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        ) {
            return Some(contact);
        }
        // Fall through to GJK/EPA for degenerate cases (intersecting/parallel axes, cap collisions)
    }

    // Slow path: Build shapes and poses for GJK/EPA (cylinder-cylinder, cylinder-box, ellipsoid-*, and fallback cases)
    let shape1 = geom_to_collision_shape(type1, size1);
    let shape2 = geom_to_collision_shape(type2, size2);

    // Build poses for GJK/EPA - expensive quaternion conversion
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    // Use GJK/EPA for general convex collision
    let collision_shape1 = shape1?;
    let collision_shape2 = shape2?;

    if let Some(result) = gjk_epa_contact(&collision_shape1, &pose1, &collision_shape2, &pose2) {
        if result.penetration > -margin {
            return Some(make_contact_from_geoms(
                model,
                Vector3::new(result.point.x, result.point.y, result.point.z),
                result.normal,
                result.penetration,
                geom1,
                geom2,
                margin,
            ));
        }
    }

    None
}

// ============================================================================
// Shape conversion
// ============================================================================

/// Convert `MuJoCo` `GeomType` to `CollisionShape`.
#[allow(clippy::match_same_arms)] // Plane and Mesh both return None but for different reasons
pub fn geom_to_collision_shape(geom_type: GeomType, size: Vector3<f64>) -> Option<CollisionShape> {
    match geom_type {
        GeomType::Sphere => Some(CollisionShape::Sphere { radius: size.x }),
        GeomType::Box => Some(CollisionShape::Box { half_extents: size }),
        GeomType::Capsule => Some(CollisionShape::Capsule {
            half_length: size.y, // MuJoCo: size[0]=radius, size[1]=half_length
            radius: size.x,
        }),
        GeomType::Cylinder => Some(CollisionShape::Cylinder {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Ellipsoid => Some(CollisionShape::Ellipsoid { radii: size }),
        GeomType::Plane => None,  // Handled via collide_with_plane()
        GeomType::Mesh => None,   // Handled via collide_with_mesh()
        GeomType::Hfield => None, // Handled via collide_with_hfield()
        GeomType::Sdf => None,    // Handled via collide_with_sdf()
    }
}

// ============================================================================
// Contact construction
// ============================================================================

/// Apply global contact-parameter override to a contact (S10: `ENABLE_OVERRIDE`).
///
/// Called **after** `apply_pair_overrides` so the global override wins over
/// pair-level values. No-op when `ENABLE_OVERRIDE` is not set.
#[inline]
pub fn apply_global_override(model: &Model, contact: &mut Contact) {
    if !enabled(model, ENABLE_OVERRIDE) {
        return;
    }
    contact.solref = model.o_solref;
    contact.solreffriction = model.o_solref;
    contact.solimp = model.o_solimp;
    let mu = assign_friction(model, &contact.mu);
    contact.mu = mu;
    contact.friction = mu[0];
}

/// Apply explicit `<pair>` overrides to a contact produced by `collide_geoms`.
///
/// `collide_geoms` combines friction/condim/solref/solimp from the two geoms.
/// For mechanism-2 contacts, those geom-combined values are overwritten here
/// with the fully-resolved pair parameters.
#[inline]
pub fn apply_pair_overrides(contact: &mut Contact, pair: &ContactPair) {
    // condim -> dim mapping (same logic as Contact::with_condim)
    #[allow(clippy::match_same_arms, clippy::cast_sign_loss)]
    let dim = match pair.condim {
        1 => 1,
        3 => 3,
        4 => 4,
        6 => 6,
        0 | 2 => 3,
        5 => 6,
        _ => 6,
    } as usize;
    contact.dim = dim;
    // 5D friction: directly from pair (already fully resolved)
    contact.mu = pair.friction;
    contact.friction = pair.friction[0]; // legacy scalar = tan1
    // Solver params
    contact.solref = pair.solref;
    contact.solimp = pair.solimp;
    // Propagate solreffriction from pair -> runtime contact.
    // [0.0, 0.0] sentinel means "use solref" (auto-generated contacts keep this default).
    contact.solreffriction = pair.solreffriction;
    // Pair margin/gap override geom-derived includemargin
    contact.includemargin = pair.margin - pair.gap;
}

/// Create a contact with solver parameters derived from the colliding geoms.
///
/// Uses the unified `contact_param()` function (MuJoCo `mj_contactParam()`
/// equivalent) for parameter combination: priority gating, solmix-weighted
/// solver params, element-wise max friction, and additive gap.
#[inline]
pub fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
    margin: f64,
) -> Contact {
    let (condim, gap, solref, solimp, mu) = contact_param(model, geom1, geom2);
    // S10: apply global override to solver params when ENABLE_OVERRIDE is active.
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let includemargin = margin - gap;

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        6 => 6,
        _ => 3,
    };

    let (t1, t2) = compute_tangent_frame(&normal);

    Contact {
        pos,
        normal,
        depth,
        geom1,
        geom2,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: [0.0, 0.0],
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: None,
    }
}
