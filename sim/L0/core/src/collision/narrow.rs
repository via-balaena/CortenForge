//! Narrow-phase collision dispatch — selects a geom pair's collision function.
//!
//! Contains the top-level `collide_geoms` dispatcher, the `geom_to_shape` converter,
//! `make_contact_from_geoms` contact constructor, `apply_pair_overrides` for mechanism-2
//! contacts, and shared collision constants.

use super::mesh_collide::collide_with_mesh;
use super::mj_ccd::mjc_convex;
use super::order::{lower_body_first, type_ordered};
use super::pair_convex::{
    collide_capsule_capsule, collide_sphere_box, collide_sphere_capsule, collide_sphere_sphere,
};
use super::pair_cylinder::{collide_box_box, collide_capsule_box, collide_cylinder_sphere};
use super::plane::collide_with_plane;
use super::sdf_collide::collide_with_sdf;

use super::{assign_friction, assign_imp, assign_ref, contact_param};
use crate::types::{Contact, ContactPair, ENABLE_OVERRIDE, GeomType, Model, enabled, make_frame};
use cf_geometry::Shape;
use nalgebra::{Matrix3, Vector3};

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

// ============================================================================
// Narrow-phase dispatch
// ============================================================================

/// Whether MuJoCo collides two geom types through `mjc_Convex`
/// (`engine_collision_driver.c:41-52`).
pub fn convex_pair(type1: GeomType, type2: GeomType) -> bool {
    let (a, b) = if type1.mjtgeom() <= type2.mjtgeom() {
        (type1, type2)
    } else {
        (type2, type1)
    };
    matches!(
        (a, b),
        (GeomType::Sphere, GeomType::Ellipsoid | GeomType::Mesh)
            | (
                GeomType::Capsule,
                GeomType::Ellipsoid | GeomType::Cylinder | GeomType::Mesh
            )
            | (
                GeomType::Ellipsoid,
                GeomType::Ellipsoid | GeomType::Cylinder | GeomType::Box | GeomType::Mesh
            )
            | (
                GeomType::Cylinder,
                GeomType::Cylinder | GeomType::Box | GeomType::Mesh
            )
            | (GeomType::Box | GeomType::Mesh, GeomType::Mesh)
    )
}

/// A convex pair through `mjc_Convex` ([`mjc_convex`]), its geoms taken as
/// MuJoCo takes them: the geom of the lower body first, then the lower
/// `mjtGeom` first.
// The full per-geom pair context, so dispatch passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
fn collide_convex(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: &Vector3<f64>,
    mat1: &Matrix3<f64>,
    pos2: &Vector3<f64>,
    mat2: &Matrix3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let (g1, g2) = type_ordered(model, lower_body_first(model, (geom1, geom2)));
    let ((p1, m1), (p2, m2)) = if g1 == geom1 {
        ((pos1, mat1), (pos2, mat2))
    } else {
        ((pos2, mat2), (pos1, mat1))
    };
    mjc_convex(model, g1, p1, m1, g2, p2, m2, margin)
        .iter()
        .map(|c| c.contact(model, g1, g2, margin))
        .collect()
}

/// Narrow-phase collision between two geometries.
// The full per-geom pair context, so dispatch passes one uniform arg list.
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

    // SDF collision first: SDF has its own contact functions for every
    // shape, including meshes, height fields and planes.
    if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
        return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Height field collision handled via collide_hfield_multi at the
    // broadphase loop level (multi-contact). Return empty here as a safety
    // fallback — hfield pairs should never reach collide_geoms in normal flow.
    if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
        return vec![];
    }

    // MuJoCo's native convex collision for every pair its table gives
    // `mjc_Convex`.
    if convex_pair(type1, type2) {
        return collide_convex(model, geom1, geom2, &pos1, &mat1, &pos2, &mat2, margin);
    }

    // A mesh with a plane.
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-sphere collision (MuJoCo's `mjc_SphereSphere`)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: capsule-capsule collision (MuJoCo's `mjraw_CapsuleCapsule`)
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

    // Special case: box-box collision (MuJoCo's `mjc_BoxBox`)
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

    // Every other pair is above: MuJoCo's table has no further function.
    vec![]
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
        GeomType::Mesh => None, // Handled via mjc_convex(), or collide_with_mesh() against a plane
        GeomType::Hfield => None, // Handled via collide_hfield_multi at broadphase level
        GeomType::Sdf => None,  // Handled via collide_with_sdf()
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

    let (normal, frame) = make_frame(&normal, &Vector3::zeros());

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
        frame,
        flex_vertex: None,
        regularized_mu: 0.0,
        flex_vertex2: None,
    }
}

#[cfg(test)]
mod tests {
    use super::collide_geoms;
    use crate::test_fixtures::collision::collision_geoms;
    use crate::types::GeomType;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, Vector3};

    /// A cylinder (geom 0) and a capsule (geom 1) side by side, 0.1 into
    /// each other: through `mjc_Convex`, listed as MuJoCo lists them, the
    /// capsule (the lower geom type) first, its normal from the capsule to
    /// the cylinder, the same contact to the bit whichever geom is given
    /// first; apart, none.
    #[test]
    fn convex_dispatch_returns_mujoco_order() {
        let mut m = collision_geoms(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.4, 0.0);
        let mat = Matrix3::identity();
        let (cylinder, capsule) = (Vector3::zeros(), Vector3::new(0.4, 0.0, 0.0));

        let given = collide_geoms(&m, 0, 1, cylinder, mat, capsule, mat, 0.0);
        let swapped = collide_geoms(&m, 1, 0, capsule, mat, cylinder, mat, 0.0);
        assert_eq!(given.len(), 1);
        let c = &given[0];
        assert_eq!((c.geom1, c.geom2), (1, 0), "the capsule first");
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c.normal, -Vector3::x(), epsilon = 1e-9);
        assert_eq!(swapped.len(), 1);
        let bits = |v: &Vector3<f64>| v.map(f64::to_bits);
        assert_eq!(swapped[0].depth.to_bits(), c.depth.to_bits());
        assert_eq!(bits(&swapped[0].pos), bits(&c.pos));
        assert_eq!(bits(&swapped[0].normal), bits(&c.normal));

        let apart = Vector3::new(1.2, 0.0, 0.0);
        assert!(collide_geoms(&m, 0, 1, cylinder, mat, apart, mat, 0.0).is_empty());
    }
}
