//! Narrow-phase collision dispatch — selects analytical or GJK/EPA path for a geom pair.
//!
//! Contains the top-level `collide_geoms` dispatcher, the `geom_to_shape` converter,
//! `make_contact_from_geoms` contact constructor, `apply_pair_overrides` for mechanism-2
//! contacts, and shared collision constants.

use super::mesh_collide::collide_with_mesh;
use super::pair_convex::{
    collide_capsule_capsule, collide_sphere_box, collide_sphere_capsule, collide_sphere_sphere,
};
use super::pair_cylinder::{
    collide_box_box, collide_capsule_box, collide_cylinder_capsule, collide_cylinder_sphere,
};
use super::plane::collide_with_plane;
use super::sdf_collide::collide_with_sdf;

use super::hfield::MAX_CONTACTS_PER_PAIR;
use super::{assign_friction, assign_imp, assign_ref, contact_param};
use crate::gjk_epa::{GjkContact, gjk_distance, gjk_epa_contact, support_face_points};
use crate::types::{
    Contact, ContactPair, ENABLE_MULTICCD, ENABLE_OVERRIDE, GeomType, Model, compute_tangent_frame,
    enabled,
};
use cf_geometry::Shape;
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
) -> Vec<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Fast path: handle all analytical collision cases first
    // These avoid the expensive quaternion conversion and GJK/EPA

    // Special case: SDF collision (before mesh/hfield/plane — SDF has its own
    // contact functions for all shapes including Mesh, Hfield, and Plane)
    if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
        return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
            .into_iter()
            .collect();
    }

    // Height field collision handled via collide_hfield_multi at the
    // broadphase loop level (multi-contact). Return empty here as a safety
    // fallback — hfield pairs should never reach collide_geoms in normal flow.
    if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
        return vec![];
    }

    // Special case: mesh collision (has its own BVH-accelerated path)
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: sphere-sphere collision (analytical, more robust than GJK/EPA)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2, margin)
            .into_iter()
            .collect();
    }

    // Special case: capsule-capsule collision (analytical, much faster than GJK/EPA)
    if type1 == GeomType::Capsule && type2 == GeomType::Capsule {
        return collide_capsule_capsule(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: sphere-capsule collision
    if (type1 == GeomType::Sphere && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Sphere)
    {
        return collide_sphere_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: sphere-box collision (analytical)
    if (type1 == GeomType::Sphere && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Sphere)
    {
        return collide_sphere_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: capsule-box collision (analytical)
    if (type1 == GeomType::Capsule && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Capsule)
    {
        return collide_capsule_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: box-box collision (SAT)
    if type1 == GeomType::Box && type2 == GeomType::Box {
        return collide_box_box(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: cylinder-sphere collision (analytical)
    if (type1 == GeomType::Cylinder && type2 == GeomType::Sphere)
        || (type1 == GeomType::Sphere && type2 == GeomType::Cylinder)
    {
        return collide_cylinder_sphere(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        )
        .into_iter()
        .collect();
    }

    // Special case: cylinder-capsule collision (analytical with GJK/EPA fallback)
    // Analytical solution handles common cases; degenerate cases fall through to GJK/EPA
    if (type1 == GeomType::Cylinder && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Cylinder)
    {
        if let Some(contact) = collide_cylinder_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        ) {
            return vec![contact];
        }
        // Fall through to GJK/EPA for degenerate cases (intersecting/parallel axes, cap collisions)
    }

    // Slow path: Build shapes and poses for GJK/EPA (cylinder-cylinder, cylinder-box, ellipsoid-*, and fallback cases)
    let shape1 = geom_to_shape(type1, size1);
    let shape2 = geom_to_shape(type2, size2);

    // Build poses for GJK/EPA - expensive quaternion conversion
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    // Use GJK/EPA for general convex collision
    let Some(shape1) = shape1 else {
        return vec![];
    };
    let Some(shape2) = shape2 else {
        return vec![];
    };

    if let Some(result) = gjk_epa_contact(
        &shape1,
        &pose1,
        &shape2,
        &pose2,
        model.ccd_iterations,
        model.ccd_tolerance,
    ) {
        if result.penetration > -margin {
            // MULTICCD: generate multiple contacts for flat contact surfaces
            if enabled(model, ENABLE_MULTICCD) {
                let multi = multiccd_contacts(
                    &shape1,
                    &pose1,
                    &shape2,
                    &pose2,
                    &result,
                    model.ccd_iterations,
                    model.ccd_tolerance,
                );
                return multi
                    .into_iter()
                    .map(|gjk| {
                        make_contact_from_geoms(
                            model,
                            Vector3::new(gjk.point.x, gjk.point.y, gjk.point.z),
                            gjk.normal,
                            gjk.penetration,
                            geom1,
                            geom2,
                            margin,
                        )
                    })
                    .collect();
            }

            return vec![make_contact_from_geoms(
                model,
                Vector3::new(result.point.x, result.point.y, result.point.z),
                result.normal,
                result.penetration,
                geom1,
                geom2,
                margin,
            )];
        }
    } else if margin > 0.0 {
        // Shapes don't overlap — check if within margin distance (margin-zone contact).
        // MuJoCo's convex solver returns separation distance for non-overlapping shapes;
        // CortenForge uses gjk_distance() as the equivalent.
        if let Some(dist_result) = gjk_distance(
            &shape1,
            &pose1,
            &shape2,
            &pose2,
            model.ccd_iterations,
            model.ccd_tolerance,
        ) {
            if dist_result.distance < margin {
                // Margin-zone contact: depth is negative (separated)
                let depth = -dist_result.distance;
                let diff = dist_result.witness_b - dist_result.witness_a;
                let diff_norm = diff.norm();
                let normal = if diff_norm > GEOM_EPSILON {
                    diff / diff_norm
                } else {
                    Vector3::z()
                };
                let midpoint = nalgebra::center(&dist_result.witness_a, &dist_result.witness_b);
                return vec![make_contact_from_geoms(
                    model,
                    midpoint.coords,
                    normal,
                    depth,
                    geom1,
                    geom2,
                    margin,
                )];
            }
        }
    }

    vec![]
}

// ============================================================================
// MULTICCD helper
// ============================================================================

/// Duplicate contact distance threshold for MULTICCD filtering.
const MULTICCD_DEDUP_DIST: f64 = 1e-4;

/// Generate multiple contacts for a convex-convex pair using MULTICCD.
///
/// Enumerates face vertices on shape A's contact face to find additional
/// contact points on flat surfaces. Returns 1-4 contacts.
/// Only called when `ENABLE_MULTICCD` is set.
pub fn multiccd_contacts(
    shape_a: &Shape,
    pose_a: &Pose,
    _shape_b: &Shape,
    _pose_b: &Pose,
    primary: &GjkContact,
    _max_iterations: usize,
    _tolerance: f64,
) -> Vec<GjkContact> {
    // MULTICCD strategy: enumerate all vertices on shape A's contact face
    // (the set of vertices sharing the maximum support value in the -normal
    // direction). For flat faces (box, convex mesh), this gives the face
    // corners. For curved shapes, this gives a single point.
    let face_points = support_face_points(shape_a, pose_a, &(-primary.normal));

    if face_points.len() <= 1 {
        // Curved surface or single vertex — no additional contacts
        return vec![primary.clone()];
    }

    let mut contacts: Vec<GjkContact> =
        Vec::with_capacity(face_points.len().min(MAX_CONTACTS_PER_PAIR));

    for pt in &face_points {
        if contacts.len() >= MAX_CONTACTS_PER_PAIR {
            break;
        }
        // Check for duplicates
        let dominated = contacts
            .iter()
            .any(|existing| (existing.point - pt).norm() < MULTICCD_DEDUP_DIST);
        if !dominated {
            contacts.push(GjkContact {
                point: *pt,
                normal: primary.normal,
                penetration: primary.penetration,
            });
        }
    }

    contacts
}

// ============================================================================
// Shape conversion
// ============================================================================

/// Convert `MuJoCo` `GeomType` to `cf_geometry::Shape`.
#[allow(clippy::match_same_arms)] // Plane and Mesh both return None but for different reasons
pub fn geom_to_shape(geom_type: GeomType, size: Vector3<f64>) -> Option<Shape> {
    match geom_type {
        GeomType::Sphere => Some(Shape::Sphere { radius: size.x }),
        GeomType::Box => Some(Shape::Box { half_extents: size }),
        GeomType::Capsule => Some(Shape::Capsule {
            half_length: size.y, // MuJoCo: size[0]=radius, size[1]=half_length
            radius: size.x,
        }),
        GeomType::Cylinder => Some(Shape::Cylinder {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Ellipsoid => Some(Shape::Ellipsoid { radii: size }),
        GeomType::Plane => None,  // Handled via collide_with_plane()
        GeomType::Mesh => None,   // Handled via collide_with_mesh()
        GeomType::Hfield => None, // Handled via collide_hfield_multi at broadphase level
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
///
/// `gap` is the additive gap from the pair (mechanism-2) so that
/// `includemargin = o_margin - gap` preserves the gap while overriding margin.
#[inline]
pub fn apply_global_override(model: &Model, contact: &mut Contact, gap: f64) {
    if !enabled(model, ENABLE_OVERRIDE) {
        return;
    }
    contact.solref = model.o_solref;
    contact.solreffriction = model.o_solref;
    contact.solimp = model.o_solimp;
    contact.includemargin = model.o_margin - gap;
    let mu = assign_friction(model, &model.o_friction);
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
///
/// Note: For mechanism-2 contacts, `apply_pair_overrides` + `apply_global_override`
/// overwrite `solref`/`solimp`/`friction` immediately after this call, so the
/// `assign_ref`/`assign_imp`/`assign_friction` work here is redundant for that
/// path. The cost is ~3 predictable branches per contact — accepted overhead to
/// keep a single construction path for both mechanisms.
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
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: None,
        flex_vertex2: None,
    }
}
