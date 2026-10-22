//! Collision detection pipeline — broad-phase dispatch, affinity filtering, and
//! contact parameter mixing.
//!
//! Gated by `DISABLE_CONTACT` and `DISABLE_CONSTRAINT` (§41 S4.1) — both flags
//! skip collision entirely. `ENABLE_OVERRIDE` (§41 S10) replaces per-geom solver
//! parameters with global `o_margin`/`o_solref`/`o_solimp`/`o_friction`.
//!
//! Corresponds to MuJoCo's `engine_collision_driver.c` (broad-phase dispatch)
//! and parameter combination from `engine_collision_primitive.c`.

mod broadphase;
pub(crate) mod flex_collide;
pub(crate) mod flex_narrow;
mod flex_rigid;
pub(crate) mod flex_self;
pub(crate) mod hfield;
mod midphase;
mod mj_ccd;
mod mj_primitive;
mod mj_triangle;
pub(crate) mod narrow;
mod order;
pub(crate) mod pair_convex;
pub(crate) mod pair_cylinder;
mod pairs;
pub(crate) mod plane;
pub(crate) mod sdf_collide;

pub(crate) use mj_ccd::geom_distance_ccd;

/// A geom's rotation as nalgebra's `UnitQuaternion::from_matrix`, or `None`
/// where the matrix holds a NaN or an infinity, on which that conversion
/// never returns.
pub(crate) fn finite_rotation(
    mat: &nalgebra::Matrix3<f64>,
) -> Option<nalgebra::UnitQuaternion<f64>> {
    mat.iter()
        .all(|x| x.is_finite())
        .then(|| nalgebra::UnitQuaternion::from_matrix(mat))
}

// Note: collide_geoms and geom_to_shape are accessed internally via
// self::narrow::, self::flex_collide::, self::flex_narrow::, and
// self::flex_self:: within this module. No pub(crate) re-export needed.

use crate::types::{
    Contact, DISABLE_CONSTRAINT, DISABLE_CONTACT, DISABLE_FILTERPARENT, DISABLE_MIDPHASE, Data,
    ENABLE_OVERRIDE, ENABLE_SLEEP, GeomType, Model, SleepState, disabled, enabled,
};

use self::broadphase::{BodyGeoms, bodyflex_pairs, filter_sphere};
use self::flex_rigid::collide_body_flex;
use self::hfield::collide_hfield_multi;
use self::midphase::collide_tree;
use self::narrow::{apply_global_override, apply_pair_overrides, collide_geoms};
use self::order::{ListPlace, Lister, flex_place, pair_place, type_ordered};
use self::pairs::{mj_collide_flex_flex, mj_collision_flex_self};

// ============================================================================
// Collision affinity filtering
// ============================================================================

/// Whether two geoms' bodies may collide, the filters MuJoCo applies to the
/// pair of bodies (`engine_collision_driver.c:160-186`, `:332-343`):
/// 1. an excluded body pair (`<contact><exclude>`) - no collision
/// 2. an explicit `<pair>` of these geoms - collided with the explicit pairs
/// 3. Same body - no collision
/// 4. Same weld group, or one the other's parent group - no collision
///    Exception: the world's group is exempt from the parent rule
///
/// The world body exception ensures ground planes can collide with objects
/// even when those objects are direct children of the world body. The
/// geoms' own bitmask is [`bitmask_allows`], which the user's contact
/// filter replaces.
#[allow(clippy::inline_always)] // Hot path - profiling shows inlining improves debug performance
#[inline(always)]
pub(crate) fn bodies_allow(model: &Model, geom1: usize, geom2: usize) -> bool {
    let body1 = model.geom_body[geom1];
    let body2 = model.geom_body[geom2];

    // Check body-pair exclude list (from <contact><exclude>)
    let exclude_key = (body1.min(body2), body1.max(body2));
    if model.contact_excludes.contains(&exclude_key) {
        return false;
    }

    // Skip if this geom pair has an explicit <pair> entry —
    // mechanism 2 will handle it with its overridden parameters.
    let pair_key = (geom1.min(geom2), geom1.max(geom2));
    if model.contact_pair_set.contains(&pair_key) {
        return false;
    }

    // Same body - no collision
    if body1 == body2 {
        return false;
    }

    // ── Weld-group filtering, matching MuJoCo ───────────────────────────
    //
    // ⛔ Compare WELD GROUPS, not bodies. Bodies joined by zero-dof joints are
    // one rigid body; a contact force between them is work against a
    // constraint that already holds. A `body_parent` test cannot see this —
    // two SIBLINGS may share a weld group's interior (a welded plate and a
    // hinged arm on the same chassis), and it read them as unrelated.
    //
    // Measured against MuJoCo 3.4.0 rather than recalled, on a model built for
    // it (`weld_model`), and each rule below is exercised there by a pair that
    // genuinely OVERLAPS, so its absence is evidence:
    //
    //   same weld group        -> never collide, even with FILTERPARENT off
    //   one group is the other's parent group -> filtered while FILTERPARENT is on
    //   either group is world  -> exempt from the parent rule
    //
    // ⚠ The world exemption is not a special case bolted on: a jointless body
    // joins weld group 0 along with the ground, so static bodies never collide
    // with each other, while a free body still collides with the ground — and
    // an articulated child of a STATIC body collides with it, which the old
    // parent-only test wrongly suppressed.
    let weld1 = model.body_weldid[body1];
    let weld2 = model.body_weldid[body2];

    // Same rigid body. Unconditional: FILTERPARENT relaxes the parent rule,
    // never this one.
    if weld1 == weld2 {
        return false;
    }

    // S4.12: Skip the parent exclusion when DISABLE_FILTERPARENT is set,
    // allowing parent-child geom pairs to collide.
    if !disabled(model, DISABLE_FILTERPARENT) && weld1 != 0 && weld2 != 0 {
        let parent_weld1 = model.body_weldid[model.body_parent[weld1]];
        let parent_weld2 = model.body_weldid[model.body_parent[weld2]];
        if weld1 == parent_weld2 || weld2 == parent_weld1 {
            return false;
        }
    }
    true
}

/// Whether two geoms' contype/conaffinity bitmasks let them collide,
/// `(c1 & a2) != 0 || (c2 & a1) != 0` (MuJoCo's `filterBitmask`,
/// `engine_collision_driver.c:99-102`, negated).
fn bitmask_allows(model: &Model, geom1: usize, geom2: usize) -> bool {
    let c1 = model.geom_contype[geom1];
    let a1 = model.geom_conaffinity[geom1];
    let c2 = model.geom_contype[geom2];
    let a2 = model.geom_conaffinity[geom2];

    (c1 & a2) != 0 || (c2 & a1) != 0
}

// ============================================================================
// Contact parameter combination
// ============================================================================

/// Contact parameter combination — MuJoCo `mj_contactParam()` equivalent.
///
/// Computes combined contact parameters from two geoms.
/// Handles priority (#25), solmix (#26), friction max (#24), and gap (#27).
///
/// Returns: (condim, gap, solref, solimp, friction\[5\])
pub(crate) fn contact_param(
    model: &Model,
    geom1: usize,
    geom2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    // 1. Load parameters
    let priority1 = model.geom_priority[geom1];
    let priority2 = model.geom_priority[geom2];
    let gap = model.geom_gap[geom1] + model.geom_gap[geom2];

    // 2. Priority check — higher priority geom's params win entirely
    if priority1 != priority2 {
        let winner = if priority1 > priority2 { geom1 } else { geom2 };
        let fri = model.geom_friction[winner];
        return (
            model.geom_condim[winner],
            gap,
            model.geom_solref[winner],
            model.geom_solimp[winner],
            [fri.x, fri.x, fri.y, fri.z, fri.z], // 3→5 unpack
        );
    }

    // 3. Equal priority — combine
    let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]);

    // 3a. Solmix weight
    let s1 = model.geom_solmix[geom1];
    let s2 = model.geom_solmix[geom2];
    let mix = solmix_weight(s1, s2);

    // 3b. Solref combination
    let solref1 = model.geom_solref[geom1];
    let solref2 = model.geom_solref[geom2];
    let solref = combine_solref(solref1, solref2, mix);

    // 3c. Solimp: weighted average
    let solimp1 = model.geom_solimp[geom1];
    let solimp2 = model.geom_solimp[geom2];
    let solimp = combine_solimp(solimp1, solimp2, mix);

    // 3d. Friction: element-wise max (NOT affected by solmix)
    let f1 = model.geom_friction[geom1];
    let f2 = model.geom_friction[geom2];
    let fri = [
        f1.x.max(f2.x),
        f1.x.max(f2.x), // sliding1, sliding2
        f1.y.max(f2.y), // torsional
        f1.z.max(f2.z),
        f1.z.max(f2.z), // rolling1, rolling2
    ];

    (condim, gap, solref, solimp, fri)
}

/// `mj_contactParam(g, -1, -1, f)` (`engine_collision_driver.c:1305-1398`):
/// geom `geom`'s and flex `flex`'s contact parameters combined, the geom's
/// first as MuJoCo's driver passes them: the higher priority's alone, else
/// the larger condim, solref and solimp mixed by their solmix (solref's
/// direct form the smaller), the larger friction of each kind; their gaps
/// added. Returns (condim, gap, solref, solimp, friction\[5\]).
pub(crate) fn contact_param_geom_flex(
    model: &Model,
    geom: usize,
    flex: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let priority_geom = model.geom_priority[geom];
    let priority_flex = model.flex_priority[flex];
    let gap = model.geom_gap[geom] + model.flex_gap[flex];

    if priority_geom > priority_flex {
        let f = model.geom_friction[geom];
        return (
            model.geom_condim[geom],
            gap,
            model.geom_solref[geom],
            model.geom_solimp[geom],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }
    if priority_flex > priority_geom {
        let f = model.flex_friction[flex];
        return (
            model.flex_condim[flex],
            gap,
            model.flex_solref[flex],
            model.flex_solimp[flex],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }

    let condim = model.geom_condim[geom].max(model.flex_condim[flex]);
    let mix = solmix_weight(model.geom_solmix[geom], model.flex_solmix[flex]);
    let solref = combine_solref(model.geom_solref[geom], model.flex_solref[flex], mix);
    let solimp = combine_solimp(model.geom_solimp[geom], model.flex_solimp[flex], mix);
    let (gf, ff) = (model.geom_friction[geom], model.flex_friction[flex]);
    let fri = [
        gf.x.max(ff.x),
        gf.x.max(ff.x),
        gf.y.max(ff.y),
        gf.z.max(ff.z),
        gf.z.max(ff.z),
    ];
    (condim, gap, solref, solimp, fri)
}

/// Compute contact parameters for flex self-collision.
///
/// Both sides are the same flex — parameters are trivially the flex's own
/// values. Gap and margin are additive (both sides contribute), so they
/// are doubled. Follows MuJoCo's `mj_contactParam()` convention for
/// same-entity pairs.
pub(crate) fn contact_param_flex_self(
    model: &Model,
    flex_id: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let f = model.flex_friction[flex_id];
    (
        model.flex_condim[flex_id],
        2.0 * model.flex_gap[flex_id], // additive: flex_gap + flex_gap
        model.flex_solref[flex_id],
        model.flex_solimp[flex_id],
        [f.x, f.x, f.y, f.z, f.z], // tangential, tangential, torsional, rolling, rolling
    )
}

/// Contact parameter combination for flex-flex cross-object collision pairs.
///
/// Both sides read `flex_*` arrays. Follows MuJoCo's `mj_contactParam()`
/// with the same priority + solmix protocol as geom-geom and flex-rigid.
/// Gap is additive: `flex_gap[f1] + flex_gap[f2]`.
pub(crate) fn contact_param_flex_flex(
    model: &Model,
    flex_id1: usize,
    flex_id2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let priority1 = model.flex_priority[flex_id1];
    let priority2 = model.flex_priority[flex_id2];
    let gap = model.flex_gap[flex_id1] + model.flex_gap[flex_id2];

    if priority1 > priority2 {
        let f = model.flex_friction[flex_id1];
        return (
            model.flex_condim[flex_id1],
            gap,
            model.flex_solref[flex_id1],
            model.flex_solimp[flex_id1],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }
    if priority2 > priority1 {
        let f = model.flex_friction[flex_id2];
        return (
            model.flex_condim[flex_id2],
            gap,
            model.flex_solref[flex_id2],
            model.flex_solimp[flex_id2],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }

    // Equal priority — combine
    let condim = model.flex_condim[flex_id1].max(model.flex_condim[flex_id2]);

    let s1 = model.flex_solmix[flex_id1];
    let s2 = model.flex_solmix[flex_id2];
    let mix = solmix_weight(s1, s2);

    let solref = combine_solref(
        model.flex_solref[flex_id1],
        model.flex_solref[flex_id2],
        mix,
    );
    let solimp = combine_solimp(
        model.flex_solimp[flex_id1],
        model.flex_solimp[flex_id2],
        mix,
    );

    let f1 = model.flex_friction[flex_id1];
    let f2 = model.flex_friction[flex_id2];
    let fri = [
        f1.x.max(f2.x),
        f1.x.max(f2.x), // tangential1, tangential2
        f1.y.max(f2.y), // torsional
        f1.z.max(f2.z),
        f1.z.max(f2.z), // rolling1, rolling2
    ];

    (condim, gap, solref, solimp, fri)
}

/// Compute solmix weight, matching MuJoCo's edge-case handling.
/// Returns weight for entity 1 (entity 2 weight = 1 - mix).
#[must_use]
pub fn solmix_weight(s1: f64, s2: f64) -> f64 {
    const MJ_MINVAL: f64 = 1e-15;
    if s1 >= MJ_MINVAL && s2 >= MJ_MINVAL {
        s1 / (s1 + s2)
    } else if s1 < MJ_MINVAL && s2 < MJ_MINVAL {
        0.5
    } else if s1 < MJ_MINVAL {
        0.0 // entity 2 dominates
    } else {
        1.0 // entity 1 dominates
    }
}

/// Combine solref using solmix weight.
/// Standard reference (solref\[0\] > 0): weighted average.
/// Direct reference (solref\[0\] <= 0): element-wise minimum.
#[must_use]
pub fn combine_solref(solref1: [f64; 2], solref2: [f64; 2], mix: f64) -> [f64; 2] {
    if solref1[0] > 0.0 && solref2[0] > 0.0 {
        [
            mix * solref1[0] + (1.0 - mix) * solref2[0],
            mix * solref1[1] + (1.0 - mix) * solref2[1],
        ]
    } else {
        [solref1[0].min(solref2[0]), solref1[1].min(solref2[1])]
    }
}

/// Combine solimp using solmix weight (always weighted average).
#[must_use]
pub fn combine_solimp(solimp1: [f64; 5], solimp2: [f64; 5], mix: f64) -> [f64; 5] {
    std::array::from_fn(|i| mix * solimp1[i] + (1.0 - mix) * solimp2[i])
}

// ============================================================================
// ENABLE_OVERRIDE assignment helpers (S10c)
// ============================================================================

/// Minimum friction coefficient. Prevents zero-friction contacts from creating
/// singular constraint Jacobians. Matches MuJoCo's `mjMINMU`.
pub(crate) const MIN_MU: f64 = 1e-5;

/// Returns the global override margin when `ENABLE_OVERRIDE` is active,
/// otherwise returns `source` unchanged.
#[inline]
pub(crate) fn assign_margin(model: &Model, source: f64) -> f64 {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_margin
    } else {
        source
    }
}

/// Returns the global override `solref` when `ENABLE_OVERRIDE` is active,
/// otherwise returns `source` unchanged.
#[inline]
pub(crate) fn assign_ref(model: &Model, source: &[f64; 2]) -> [f64; 2] {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_solref
    } else {
        *source
    }
}

/// Returns the global override `solimp` when `ENABLE_OVERRIDE` is active,
/// otherwise returns `source` unchanged.
#[inline]
pub(crate) fn assign_imp(model: &Model, source: &[f64; 5]) -> [f64; 5] {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_solimp
    } else {
        *source
    }
}

/// Returns the global override friction when `ENABLE_OVERRIDE` is active,
/// otherwise returns `source` unchanged. All elements are clamped to `MIN_MU`
/// regardless of override status.
#[inline]
pub(crate) fn assign_friction(model: &Model, source: &[f64; 5]) -> [f64; 5] {
    let f = if enabled(model, ENABLE_OVERRIDE) {
        model.o_friction
    } else {
        *source
    };
    [
        f[0].max(MIN_MU),
        f[1].max(MIN_MU),
        f[2].max(MIN_MU),
        f[3].max(MIN_MU),
        f[4].max(MIN_MU),
    ]
}

// ============================================================================
// Broad-phase collision dispatch
// ============================================================================

/// Collision detection: populate contacts from geometry pairs.
///
/// Following `MuJoCo`'s `mj_collision` (`engine_collision_driver.c:264-482`):
/// 1. Broadphase: the pairs of bodies MuJoCo's `mj_broadphase` passes
///    (`broadphase`); within a pair, the geom pairs MuJoCo's midphase
///    passes (`midphase`), or each pair's geoms in turn
/// 2. The geom pair's filters: exclusions, explicit pairs, weld groups,
///    then the user's filter where one is set and the contype/conaffinity
///    bitmask where none is, then the bounding spheres
/// 3. Narrow-phase: analytical or GJK/EPA collision detection
/// 4. The explicit pairs, the flexes, and the list in MuJoCo's order (`order`)
///
/// This function is called after forward kinematics (`mj_fwd_position`) so
/// `geom_xpos` and `geom_xmat` are up-to-date.
pub(crate) fn mj_collision(model: &Model, data: &mut Data) {
    // Unconditional reset (S4.1 — matches MuJoCo's mj_collision).
    data.contacts.clear();
    data.ncon = 0;
    // Each rigid contact's place in MuJoCo's list, as it is made.
    let mut places: Vec<ListPlace> = Vec::new();

    // S4.1: Skip collision detection when contacts or constraints are disabled.
    let nbodyflex = model.nbody + model.nflex;
    if disabled(model, DISABLE_CONTACT) || disabled(model, DISABLE_CONSTRAINT) || nbodyflex < 2 {
        return;
    }

    let sleep_enabled = enabled(model, ENABLE_SLEEP);
    let lister = Lister::new(model);

    // MuJoCo's broadphase (`broadphase`): pairs of bodies and flexes in
    // signature order, a repeated pair once
    // (`engine_collision_driver.c:307-312`). A sleeping body is dropped
    // there, with the body-level filters. Within a pair of bodies
    // (`:354-387`), two single-geom bodies collide their one pair; other
    // bodies the geom pairs the midphase (`midphase`) finds overlapping;
    // without the midphase, or a body without a hierarchy, the first body's
    // geoms in turn against the second's. A body and a flex collide their
    // geoms and elements (`flex_rigid`); two flexes, the flexes' own pass
    // below.
    let body_geoms = BodyGeoms::new(model);
    let midphase = !disabled(model, DISABLE_MIDPHASE);
    let mut previous = None;
    let mut geom_pairs: Vec<(usize, usize)> = Vec::new();
    let mut tree_stack: Vec<(usize, usize)> = Vec::new();
    for (body1, body2) in bodyflex_pairs(model, data, &body_geoms) {
        if previous == Some((body1, body2)) {
            continue;
        }
        previous = Some((body1, body2));
        if body2 >= model.nbody {
            if body1 < model.nbody {
                let flex = body2 - model.nbody;
                let scratch = (&mut geom_pairs, &mut tree_stack);
                collide_body_flex(
                    model,
                    data,
                    &body_geoms,
                    &mut places,
                    (body1, flex),
                    scratch,
                );
            }
            continue;
        }
        let (geoms1, geoms2) = (body_geoms.of(body1), body_geoms.of(body2));
        geom_pairs.clear();
        let single = geoms1.len() == 1 && geoms2.len() == 1;
        if single
            || !midphase
            || !collide_tree(model, data, body1, body2, &mut geom_pairs, &mut tree_stack)
        {
            for &geom1 in geoms1 {
                geom_pairs.extend(geoms2.iter().map(|&geom2| (geom1, geom2)));
            }
        }
        for &(geom1, geom2) in &geom_pairs {
            collide_geom_pair(model, data, &lister, &mut places, geom1, geom2);
        }
    }

    // Mechanism 2: explicit contact pairs (bypass kinematic + bitmask filters)
    for (pair_id, pair) in model.contact_pairs.iter().enumerate() {
        let geom1 = pair.geom1;
        let geom2 = pair.geom2;

        // §16.5b: an explicit pair collides only when one of its bodies
        // is awake (engine_collision_driver.c:1465-1472).
        if sleep_enabled
            && data.body_sleep_state[model.geom_body[geom1]] != SleepState::Awake
            && data.body_sleep_state[model.geom_body[geom2]] != SleepState::Awake
        {
            continue;
        }

        // Pair margin overrides geom margins.
        // S10: assign_margin returns o_margin when global override active.
        let margin = assign_margin(model, pair.margin);

        // MuJoCo's bounding-sphere filter (`mj_filterSphere`).
        if filter_sphere(model, data, geom1, geom2, margin) {
            continue;
        }

        // Narrow-phase collision detection
        let pos1 = data.geom_xpos[geom1];
        let mat1 = data.geom_xmat[geom1];
        let pos2 = data.geom_xpos[geom2];
        let mat2 = data.geom_xmat[geom2];

        let is_hfield_pair = (model.geom_type[geom1] == GeomType::Hfield
            || model.geom_type[geom2] == GeomType::Hfield)
            && model.geom_type[geom1] != GeomType::Sdf
            && model.geom_type[geom2] != GeomType::Sdf;

        if is_hfield_pair {
            let hf_contacts =
                collide_hfield_multi(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
            for mut contact in hf_contacts {
                apply_pair_overrides(&mut contact, pair);
                apply_global_override(model, &mut contact, pair.gap);
                if !enabled(model, ENABLE_OVERRIDE) {
                    contact.mu = assign_friction(model, &contact.mu);
                    contact.friction = contact.mu[0];
                }
                places.push(pair_place(model, &mut contact, pair_id));
                data.contacts.push(contact);
                data.ncon += 1;
            }
        } else {
            let contacts = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
            for mut contact in contacts {
                apply_pair_overrides(&mut contact, pair);
                apply_global_override(model, &mut contact, pair.gap);
                // MIN_MU clamp for the non-override pair path (apply_global_override
                // already handles the override case; this covers the else branch).
                if !enabled(model, ENABLE_OVERRIDE) {
                    contact.mu = assign_friction(model, &contact.mu);
                    contact.friction = contact.mu[0];
                }
                places.push(pair_place(model, &mut contact, pair_id));
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    }

    // Mechanism 4: flex self-collision (internal + non-adjacent)
    mj_collision_flex_self(model, data);

    // Mechanism 5: flex-flex cross-object collision
    mj_collide_flex_flex(model, data);

    // List the contacts as MuJoCo does (`order`); the sort is stable, so a
    // pair's contacts keep the order its collision function made them in.
    places.extend(
        data.contacts[places.len()..]
            .iter()
            .map(|c| flex_place(model, c)),
    );
    let mut ordered: Vec<(ListPlace, Contact)> =
        places.into_iter().zip(data.contacts.drain(..)).collect();
    ordered.sort_by_key(|(place, _)| *place);
    data.contacts
        .extend(ordered.into_iter().map(|(_, contact)| contact));
}

/// Collide one geom pair of a pair of bodies and list its contacts as MuJoCo
/// does (`mj_collideGeomPair`, `mj_collideGeoms`,
/// `engine_collision_driver.c:496-519`, `:1453-1529`): the bodies' filters,
/// the user's filter or else the bitmask, the bounding spheres, then the
/// narrow phase.
fn collide_geom_pair(
    model: &Model,
    data: &mut Data,
    lister: &Lister,
    places: &mut Vec<ListPlace>,
    geom1: usize,
    geom2: usize,
) {
    // The body pair's filters: exclusions, explicit pairs, weld groups.
    if !bodies_allow(model, geom1, geom2) {
        return;
    }

    // MuJoCo takes the pair in type order, the lower `mjtGeom` first, and
    // collides nothing where it has no collision function: a plane or a
    // height field with a plane or a height field (`:1475-1490`, `:41-53`).
    let (typed1, typed2) = type_ordered(model, (geom1, geom2));
    let flat = |g: usize| matches!(model.geom_type[g], GeomType::Plane | GeomType::Hfield);
    if flat(typed1) && flat(typed2) {
        return;
    }

    // DT-79: the user's contact filter, given the pair in type order, in
    // place of the bitmask, as MuJoCo's (`:1493-1507`). Return false to
    // reject the pair.
    let allowed = match model.cb_contactfilter {
        Some(ref cb) => (cb.0)(model, data, typed1, typed2),
        None => bitmask_allows(model, geom1, geom2),
    };
    if !allowed {
        return;
    }

    // Get world-space poses
    let pos1 = data.geom_xpos[geom1];
    let mat1 = data.geom_xmat[geom1];
    let pos2 = data.geom_xpos[geom2];
    let mat2 = data.geom_xmat[geom2];

    // Compute effective margin for this pair.
    // S10: assign_margin returns o_margin directly when override active.
    let margin = assign_margin(model, model.geom_margin[geom1] + model.geom_margin[geom2]);

    // MuJoCo's bounding-sphere filter (`mj_filterSphere`).
    if filter_sphere(model, data, geom1, geom2, margin) {
        return;
    }

    // Check if either geom is a heightfield — use multi-contact path.
    // EXCLUDE SDF-Hfield pairs: MuJoCo uses mjc_HFieldSDF (not
    // mjc_ConvexHField) for that pair, and CortenForge's collide_with_sdf
    // already handles it.
    let is_hfield_pair = (model.geom_type[geom1] == GeomType::Hfield
        || model.geom_type[geom2] == GeomType::Hfield)
        && model.geom_type[geom1] != GeomType::Sdf
        && model.geom_type[geom2] != GeomType::Sdf;

    if is_hfield_pair {
        let hf_contacts = collide_hfield_multi(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
        for mut contact in hf_contacts {
            places.push(lister.broadphase(model, &mut contact, (geom1, geom2)));
            data.contacts.push(contact);
            data.ncon += 1;
        }
    } else {
        // The narrow phase does not read DISABLE_MIDPHASE.
        let contacts = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
        for mut contact in contacts {
            places.push(lister.broadphase(model, &mut contact, (geom1, geom2)));
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}

// =============================================================================
// Unit tests -- contact_param (#24-#27) Batch 1 contact parameter combination
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod contact_param_tests {
    use super::narrow::make_contact_from_geoms;
    use super::*;
    use crate::types::GeomType;
    use approx::assert_relative_eq;
    use nalgebra::{UnitQuaternion, Vector3};

    /// Helper to create a Model with two geoms and configurable contact fields.
    fn make_two_geom_model() -> Model {
        let mut model = Model::empty();
        model.ngeom = 2;
        model.geom_type = vec![GeomType::Sphere; 2];
        model.geom_body = vec![0; 2];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); 2];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![1.0; 2];
        model.geom_mesh = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];

        // Defaults matching MuJoCo
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model
    }

    // ========================================================================
    // #24 -- Friction combination: element-wise max (NOT geometric mean)
    // ========================================================================

    #[test]
    fn friction_uses_element_wise_max() {
        // AC1: Asymmetric friction -> max wins, not geometric mean
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.8, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.2, 0.05, 0.003);

        let (_condim, _gap, _solref, _solimp, mu) = contact_param(&model, 0, 1);

        // Element-wise max: slide=max(0.8,0.2)=0.8, spin=max(0.01,0.05)=0.05, roll=max(0.001,0.003)=0.003
        assert_relative_eq!(mu[0], 0.8, epsilon = 1e-15); // sliding1
        assert_relative_eq!(mu[1], 0.8, epsilon = 1e-15); // sliding2
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15); // torsional
        assert_relative_eq!(mu[3], 0.003, epsilon = 1e-15); // rolling1
        assert_relative_eq!(mu[4], 0.003, epsilon = 1e-15); // rolling2

        // Verify NOT geometric mean: sqrt(0.8*0.2) ~ 0.4, which != 0.8
        assert!((mu[0] - (0.8_f64 * 0.2).sqrt()).abs() > 0.01);
    }

    #[test]
    fn friction_symmetric_equal() {
        // AC2: Equal friction -> max = same value
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.5, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.5, 0.01, 0.001);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], 0.5, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.01, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.001, epsilon = 1e-15);
    }

    #[test]
    fn friction_zero_one_geom() {
        // AC3: One geom zero friction -> max picks the other
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.0, 0.0, 0.0);
        model.geom_friction[1] = Vector3::new(0.6, 0.02, 0.002);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], 0.6, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.02, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.002, epsilon = 1e-15);
    }

    #[test]
    fn friction_3_to_5_unpack() {
        // Verify 3->5 unpack: [slide, slide, spin, roll, roll]
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.7, 0.03, 0.004);
        model.geom_friction[1] = Vector3::new(0.1, 0.01, 0.001);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], mu[1], epsilon = 1e-15); // sliding1 == sliding2
        assert_relative_eq!(mu[3], mu[4], epsilon = 1e-15); // rolling1 == rolling2
        assert!((mu[0] - mu[2]).abs() > 1e-10); // sliding != torsional
        assert!((mu[2] - mu[3]).abs() > 1e-10); // torsional != rolling
    }

    #[test]
    fn friction_not_affected_by_solmix() {
        // AC4: Solmix weight does NOT affect friction (only affects solref/solimp)
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_solmix[0] = 100.0; // Extreme weight toward geom 0
        model.geom_solmix[1] = 0.001;

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        // Friction should still be element-wise max, regardless of solmix
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.005, epsilon = 1e-15);
    }

    // ========================================================================
    // #25 -- Priority gating: higher priority wins verbatim
    // ========================================================================

    #[test]
    fn priority_higher_wins_verbatim() {
        // AC1: Higher priority geom's params copied verbatim, no combination
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 5;
        model.geom_priority[1] = 0;
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 6;
        model.geom_solref[0] = [0.01, 0.5];
        model.geom_solref[1] = [0.05, 2.0];
        model.geom_solimp[0] = [0.8, 0.9, 0.002, 0.4, 1.5];
        model.geom_solimp[1] = [0.95, 0.99, 0.01, 0.6, 3.0];

        let (condim, _gap, solref, solimp, mu) = contact_param(&model, 0, 1);

        // Geom 0 wins (priority 5 > 0) -- all params from geom 0
        assert_eq!(condim, 1);
        for (i, &expected) in [0.01, 0.5].iter().enumerate() {
            assert_relative_eq!(solref[i], expected, epsilon = 1e-15);
        }
        for (i, &expected) in [0.8, 0.9, 0.002, 0.4, 1.5].iter().enumerate() {
            assert_relative_eq!(solimp[i], expected, epsilon = 1e-15);
        }
        assert_relative_eq!(mu[0], 0.3, epsilon = 1e-15); // geom 0's friction
    }

    #[test]
    fn priority_lower_index_can_lose() {
        // AC2: Priority on geom2 > geom1 -> geom2 wins
        let mut model = make_two_geom_model();
        model.geom_priority[0] = -1;
        model.geom_priority[1] = 3;
        model.geom_friction[0] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_friction[1] = Vector3::new(0.2, 0.01, 0.001);
        model.geom_condim[0] = 6;
        model.geom_condim[1] = 1;

        let (condim, _, _, _, mu) = contact_param(&model, 0, 1);

        // Geom 1 wins (priority 3 > -1)
        assert_eq!(condim, 1);
        assert_relative_eq!(mu[0], 0.2, epsilon = 1e-15);
    }

    #[test]
    fn priority_equal_combines() {
        // AC3: Equal priority -> combination rules (not verbatim copy)
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 2;
        model.geom_priority[1] = 2;
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 4;

        let (condim, _, _, _, mu) = contact_param(&model, 0, 1);

        // Equal priority -> condim = max(1, 4) = 4
        assert_eq!(condim, 4);
        // Friction = max -> 0.9 (not 0.3 verbatim)
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
    }

    #[test]
    fn priority_negative_values() {
        // AC4: Negative priorities work correctly
        let mut model = make_two_geom_model();
        model.geom_priority[0] = -5;
        model.geom_priority[1] = -2;
        model.geom_condim[0] = 6;
        model.geom_condim[1] = 1;

        let (condim, _, _, _, _) = contact_param(&model, 0, 1);

        // Geom 1 wins (priority -2 > -5) -> condim from geom 1
        assert_eq!(condim, 1);
    }

    #[test]
    fn priority_gap_still_additive() {
        // AC5: Gap is additive even when priority wins
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 10;
        model.geom_priority[1] = 0;
        model.geom_gap[0] = 0.01;
        model.geom_gap[1] = 0.02;

        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        // Gap is always additive regardless of priority
        assert_relative_eq!(gap, 0.03, epsilon = 1e-15);
    }

    // ========================================================================
    // #26 -- Solmix: solver parameter mixing weight
    // ========================================================================

    #[test]
    fn solmix_weight_equal_default() {
        // AC1: Default solmix=1.0 -> weight = 0.5 -> equal average
        let mix = solmix_weight(1.0, 1.0);
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_asymmetric() {
        // AC2: solmix 3:1 -> weight = 3/4 = 0.75 for entity 1
        let mix = solmix_weight(3.0, 1.0);
        assert_relative_eq!(mix, 0.75, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_both_below_minval() {
        // AC3: Both below threshold -> equal weight (0.5)
        let mix = solmix_weight(1e-20, 1e-20);
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_one_below_minval() {
        // AC4: s1 below threshold -> entity 2 dominates (mix=0)
        let mix = solmix_weight(0.0, 1.0);
        assert_relative_eq!(mix, 0.0, epsilon = 1e-15);

        // AC5: s2 below threshold -> entity 1 dominates (mix=1)
        let mix2 = solmix_weight(1.0, 0.0);
        assert_relative_eq!(mix2, 1.0, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_exactly_at_minval() {
        // Edge case: exactly at MJ_MINVAL boundary
        let mix = solmix_weight(1e-15, 1e-15);
        // Both at 1e-15 = MJ_MINVAL -> both "valid" -> s1/(s1+s2) = 0.5
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solref_combination_standard_mode() {
        // AC6: Standard solref (both > 0) -> weighted average
        let sr1 = [0.01, 0.5];
        let sr2 = [0.05, 2.0];
        let mix = 0.75; // entity 1 dominates

        let sr = combine_solref(sr1, sr2, mix);

        // 0.75*0.01 + 0.25*0.05 = 0.0075 + 0.0125 = 0.02
        assert_relative_eq!(sr[0], 0.02, epsilon = 1e-15);
        // 0.75*0.5 + 0.25*2.0 = 0.375 + 0.5 = 0.875
        assert_relative_eq!(sr[1], 0.875, epsilon = 1e-15);
    }

    #[test]
    fn solref_combination_direct_mode() {
        // AC7: Direct solref (at least one <= 0) -> element-wise minimum
        let sr1 = [-100.0, -5.0];
        let sr2 = [-200.0, -3.0];
        let mix = 0.5; // Should be ignored for direct mode

        let sr = combine_solref(sr1, sr2, mix);

        assert_relative_eq!(sr[0], -200.0, epsilon = 1e-15); // min(-100, -200)
        assert_relative_eq!(sr[1], -5.0, epsilon = 1e-15); // min(-5, -3)
    }

    #[test]
    fn solref_combination_mixed_mode_falls_to_direct() {
        // AC8: One standard, one direct -> direct mode (element-wise min)
        let sr1 = [0.02, 1.0]; // standard (positive)
        let sr2 = [-100.0, -5.0]; // direct (negative)

        let sr = combine_solref(sr1, sr2, 0.5);

        // At least one <= 0 -> element-wise min
        assert_relative_eq!(sr[0], -100.0, epsilon = 1e-15);
        assert_relative_eq!(sr[1], -5.0, epsilon = 1e-15);
    }

    #[test]
    fn solimp_combination_weighted_average() {
        // AC9: Solimp always uses weighted average
        let si1 = [0.8, 0.9, 0.002, 0.4, 1.5];
        let si2 = [0.95, 0.99, 0.01, 0.6, 3.0];
        let mix = 0.75;

        let si = combine_solimp(si1, si2, mix);

        for i in 0..5 {
            let expected = 0.75 * si1[i] + 0.25 * si2[i];
            assert_relative_eq!(si[i], expected, epsilon = 1e-15);
        }
    }

    #[test]
    fn solmix_affects_solref_solimp_not_friction() {
        // AC10: Full integration -- solmix weights affect solref/solimp but NOT friction
        let mut model = make_two_geom_model();
        model.geom_solmix[0] = 9.0;
        model.geom_solmix[1] = 1.0;
        // mix = 9/(9+1) = 0.9 -> entity 0 has 90% weight

        model.geom_solref[0] = [0.01, 0.5];
        model.geom_solref[1] = [0.05, 2.0];
        model.geom_solimp[0] = [0.8, 0.9, 0.002, 0.4, 1.5];
        model.geom_solimp[1] = [0.95, 0.99, 0.01, 0.6, 3.0];
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);

        let (_, _, solref, solimp, mu) = contact_param(&model, 0, 1);

        // Solref: 0.9*0.01 + 0.1*0.05 = 0.009 + 0.005 = 0.014
        assert_relative_eq!(solref[0], 0.014, epsilon = 1e-15);
        // Solref[1]: 0.9*0.5 + 0.1*2.0 = 0.45 + 0.2 = 0.65
        assert_relative_eq!(solref[1], 0.65, epsilon = 1e-15);

        // Solimp: weighted average with mix=0.9
        for (i, &actual) in solimp.iter().enumerate() {
            let expected = 0.9 * model.geom_solimp[0][i] + 0.1 * model.geom_solimp[1][i];
            assert_relative_eq!(actual, expected, epsilon = 1e-15);
        }

        // Friction: still element-wise max, NOT weighted
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.005, epsilon = 1e-15);
    }

    // ========================================================================
    // #27 -- Contact margin/gap
    // ========================================================================

    #[test]
    fn gap_additive() {
        // AC1: Gap is additive from both geoms
        let mut model = make_two_geom_model();
        model.geom_gap[0] = 0.01;
        model.geom_gap[1] = 0.02;

        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        assert_relative_eq!(gap, 0.03, epsilon = 1e-15);
    }

    #[test]
    fn gap_default_zero() {
        // AC2: Default gap=0.0 -> no change to existing behavior
        let model = make_two_geom_model();
        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        assert_relative_eq!(gap, 0.0, epsilon = 1e-15);
    }

    #[test]
    fn includemargin_equals_margin_minus_gap() {
        // AC3: includemargin = margin - gap (tested via make_contact_from_geoms)
        let mut model = make_two_geom_model();
        model.geom_gap[0] = 0.005;
        model.geom_gap[1] = 0.003;
        // gap = 0.008

        let margin = 0.02; // effective margin from broadphase
        let contact =
            make_contact_from_geoms(&model, Vector3::zeros(), Vector3::z(), 0.01, 0, 1, margin);

        // includemargin = 0.02 - 0.008 = 0.012
        assert_relative_eq!(contact.includemargin, 0.012, epsilon = 1e-15);
    }

    #[test]
    fn condim_max_combination() {
        // Condim uses max when equal priority
        let mut model = make_two_geom_model();
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 4;

        let (condim, _, _, _, _) = contact_param(&model, 0, 1);

        assert_eq!(condim, 4);
    }

    // ========================================================================
    // DT-90 -- Flex-rigid friction: Vector3 mapping
    // ========================================================================

    /// Helper to add a single flex entity to a model for flex-rigid contact tests.
    fn add_flex_to_model(model: &mut Model) {
        model.nflex = 1;
        model.flex_friction = vec![Vector3::new(1.0, 0.005, 0.0001)];
        model.flex_condim = vec![3];
        model.flex_solref = vec![[0.02, 1.0]];
        model.flex_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.flex_priority = vec![0];
        model.flex_solmix = vec![1.0];
        model.flex_margin = vec![0.0];
        model.flex_gap = vec![0.0];
    }

    #[test]
    fn dt90_flex_priority_friction_mapping() {
        // DT-90: flex priority > geom → 5-element array = [tan, tan, tor, roll, roll]
        let mut model = make_two_geom_model();
        add_flex_to_model(&mut model);
        model.flex_priority[0] = 5;
        model.flex_friction[0] = Vector3::new(0.8, 0.03, 0.004);

        let (_condim, _gap, _solref, _solimp, mu) = contact_param_geom_flex(&model, 0, 0);

        assert_relative_eq!(mu[0], 0.8, epsilon = 1e-15); // tangential1
        assert_relative_eq!(mu[1], 0.8, epsilon = 1e-15); // tangential2
        assert_relative_eq!(mu[2], 0.03, epsilon = 1e-15); // torsional
        assert_relative_eq!(mu[3], 0.004, epsilon = 1e-15); // rolling1
        assert_relative_eq!(mu[4], 0.004, epsilon = 1e-15); // rolling2
    }

    #[test]
    fn dt90_flex_equal_priority_per_component_max() {
        // DT-90: equal priority → element-wise max uses per-component comparison
        let mut model = make_two_geom_model();
        add_flex_to_model(&mut model);
        model.flex_priority[0] = 0;
        model.geom_priority[0] = 0;
        model.flex_friction[0] = Vector3::new(0.3, 0.04, 0.001);
        model.geom_friction[0] = Vector3::new(0.9, 0.01, 0.005);

        let (_condim, _gap, _solref, _solimp, mu) = contact_param_geom_flex(&model, 0, 0);

        // Per-component max:
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15); // max(0.3, 0.9) tangential1
        assert_relative_eq!(mu[1], 0.9, epsilon = 1e-15); // tangential2
        assert_relative_eq!(mu[2], 0.04, epsilon = 1e-15); // max(0.04, 0.01) torsional
        assert_relative_eq!(mu[3], 0.005, epsilon = 1e-15); // max(0.001, 0.005) rolling1
        assert_relative_eq!(mu[4], 0.005, epsilon = 1e-15); // rolling2
    }

    #[test]
    fn defaults_produce_standard_behavior() {
        // AC4: All defaults -> standard MuJoCo behavior
        let model = make_two_geom_model();

        let (condim, gap, solref, solimp, mu) = contact_param(&model, 0, 1);

        assert_eq!(condim, 3);
        assert_relative_eq!(gap, 0.0, epsilon = 1e-15);
        for (i, &expected) in [0.02, 1.0].iter().enumerate() {
            assert_relative_eq!(solref[i], expected, epsilon = 1e-15);
        }
        for (i, &expected) in [0.9, 0.95, 0.001, 0.5, 2.0].iter().enumerate() {
            assert_relative_eq!(solimp[i], expected, epsilon = 1e-15);
        }
        assert_relative_eq!(mu[0], 1.0, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.005, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.0001, epsilon = 1e-15);
    }
}

// ==========================================================================
// Spec D — Convex collision solver completeness tests
// ==========================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod spec_d_tests {
    use super::hfield::collide_hfield_multi;
    use super::narrow::collide_geoms;
    use super::sdf_collide::collide_with_sdf;
    use crate::heightfield::HeightFieldData;
    use crate::mesh::TriangleMeshData;
    use crate::sdf::{PhysicsShape, SdfGrid, ShapeConvex};
    use crate::types::{ENABLE_MULTICCD, GeomType, Model};
    use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    /// Build a minimal Model with two geoms of given types and sizes.
    fn make_model(
        type1: GeomType,
        size1: Vector3<f64>,
        type2: GeomType,
        size2: Vector3<f64>,
    ) -> Model {
        let mut model = Model::empty();
        model.ngeom = 2;
        model.geom_type = vec![type1, type2];
        model.geom_body = vec![0; 2];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![size1, size2];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![1.0; 2];
        model.geom_mesh = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model.ccd_iterations = 35;
        model.ccd_tolerance = 1e-6;
        model
    }

    /// T3: AC3 — margin-zone contact for ellipsoid pair within margin
    #[test]
    fn test_margin_zone_ellipsoid_pair() {
        let model = make_model(
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
        );
        // Separate by 1.05 along X — gap = 1.05 - 0.5 - 0.5 = 0.05
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(1.05, 0.0, 0.0);
        let mat = Matrix3::identity();
        let margin = 0.2; // Combined margin > gap → should produce contact

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, margin);
        assert_eq!(contacts.len(), 1, "should produce one margin-zone contact");
        assert!(
            contacts[0].depth < 0.0,
            "depth should be negative (separated), got {}",
            contacts[0].depth
        );
    }

    /// T4: AC4 — beyond margin produces no contact
    #[test]
    fn test_beyond_margin_no_contact() {
        let model = make_model(
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
        );
        // Separate by 1.05 along X — gap = 0.05, margin = 0.02 (too small)
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(1.05, 0.0, 0.0);
        let mat = Matrix3::identity();
        let margin = 0.02; // Combined margin < gap → no contact

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, margin);
        assert!(
            contacts.is_empty(),
            "should produce no contact when beyond margin"
        );
    }

    /// T13: Edge case — zero margin, through `mjc_convex`
    #[test]
    fn test_zero_margin_gjk_epa_path() {
        let model = make_model(
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.5, 0.5),
        );
        // Separated, zero margin → no margin-zone contact
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(1.5, 0.0, 0.0);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, 0.0);
        assert!(
            contacts.is_empty(),
            "zero margin should produce no contact for separated shapes"
        );
    }

    /// T9: AC9 — MULTICCD disabled (default), single contact
    #[test]
    fn test_multiccd_disabled_single_contact() {
        let model = make_model(
            GeomType::Cylinder,
            Vector3::new(0.3, 0.5, 0.0),
            GeomType::Cylinder,
            Vector3::new(0.3, 0.5, 0.0),
        );
        // Overlapping cylinders (through `mjc_convex`)
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(0.3, 0.0, 0.0);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, 0.0);
        assert_eq!(
            contacts.len(),
            1,
            "without MULTICCD, single contact expected"
        );
    }

    /// Create a box-shaped mesh with given half-sizes, with convex hull computed.
    fn box_mesh(hx: f64, hy: f64, hz: f64) -> Arc<TriangleMeshData> {
        let vertices = vec![
            Point3::new(-hx, -hy, -hz),
            Point3::new(hx, -hy, -hz),
            Point3::new(hx, hy, -hz),
            Point3::new(-hx, hy, -hz),
            Point3::new(-hx, -hy, hz),
            Point3::new(hx, -hy, hz),
            Point3::new(hx, hy, hz),
            Point3::new(-hx, hy, hz),
        ];
        // 12 triangles (2 per face)
        #[rustfmt::skip]
        let indices = vec![
            0,1,2, 0,2,3, // -Z face
            4,6,5, 4,7,6, // +Z face
            0,4,5, 0,5,1, // -Y face
            2,6,7, 2,7,3, // +Y face
            0,3,7, 0,7,4, // -X face
            1,5,6, 1,6,2, // +X face
        ];
        let mut mesh = TriangleMeshData::new(vertices, indices);
        mesh.compute_convex_hull(None);
        Arc::new(mesh)
    }

    /// T8: AC8 — MULTICCD with mesh-mesh hull pair produces multiple contacts
    /// at corners of the flat contact surface.
    ///
    /// MuJoCo 3.5.0 empirical (EGT-7): two box-shaped meshes with flat face
    /// contact produce 4 contacts at the corners of the upper mesh's bottom face.
    #[test]
    fn test_multiccd_enabled_multiple_contacts() {
        // Upper box: half-size 0.3 × 0.3 × 0.3
        let upper_mesh = box_mesh(0.3, 0.3, 0.3);
        // Lower box: half-size 0.5 × 0.5 × 0.5
        let lower_mesh = box_mesh(0.5, 0.5, 0.5);

        let mut model = make_model(
            GeomType::Mesh,
            Vector3::new(0.3, 0.3, 0.3),
            GeomType::Mesh,
            Vector3::new(0.5, 0.5, 0.5),
        );
        model.enableflags |= ENABLE_MULTICCD;
        model.geom_mesh = vec![Some(0), Some(1)];
        model.mesh_data = vec![upper_mesh, lower_mesh];

        // Upper mesh at z=0.75 resting on lower mesh at origin.
        // Bottom face of upper mesh at z=0.45, top face of lower at z=0.5.
        // Overlap = 0.05 along Z.
        let pos1 = Vector3::new(0.0, 0.0, 0.75);
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, 0.0);

        // MULTICCD should produce multiple contacts (up to 4 for flat face)
        assert!(
            contacts.len() > 1,
            "MULTICCD mesh-mesh hull should produce multiple contacts, got {}",
            contacts.len()
        );

        // All contacts should have approximately equal depth (flat face contact)
        let depths: Vec<f64> = contacts.iter().map(|c| c.depth).collect();
        let max_depth = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_depth = depths.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            (max_depth - min_depth).abs() < 0.01,
            "all contacts should have similar depth on flat face, range: {min_depth:.4}..{max_depth:.4}",
        );
    }

    /// T15: Edge case — MULTICCD on curved contact (single contact expected)
    #[test]
    fn test_multiccd_curved_surface_single_contact() {
        let mut model = make_model(
            GeomType::Ellipsoid,
            Vector3::new(1.0, 1.0, 1.0),
            GeomType::Ellipsoid,
            Vector3::new(1.0, 1.0, 1.0),
        );
        // Enable MULTICCD
        model.enableflags |= ENABLE_MULTICCD;

        // Overlapping ellipsoids
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(1.5, 0.0, 0.0);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, 0.0);
        // `mjc_convex` makes no extra MULTICCD pass for an ellipsoid
        assert!(!contacts.is_empty(), "should produce at least one contact");
    }

    // ========================================================================
    // Mesh–Cylinder dispatch tests (through `mjc_convex`)
    // ========================================================================

    /// Ground slab half-thickness. The slab mesh is centered at origin with
    /// top face at `z = SLAB_HZ`. Tests place the mesh at `z = -SLAB_HZ`
    /// so the top surface aligns with `z = 0`.
    const SLAB_HZ: f64 = 0.01;

    /// Create a thin ground-slab mesh (2×2×0.02) with convex hull.
    fn ground_slab_mesh() -> Arc<TriangleMeshData> {
        box_mesh(1.0, 1.0, SLAB_HZ)
    }

    /// Helper: set up a (Mesh, Cylinder) model with given mesh and cylinder size.
    /// Returns the model with geom0 = mesh, geom1 = cylinder.
    fn mesh_cylinder_model(
        mesh: Arc<TriangleMeshData>,
        cyl_half_length: f64,
        cyl_radius: f64,
    ) -> Model {
        let mut model = make_model(
            GeomType::Mesh,
            Vector3::zeros(), // mesh size unused (hull provides geometry)
            GeomType::Cylinder,
            Vector3::new(cyl_radius, cyl_half_length, 0.0),
        );
        model.geom_mesh = vec![Some(0), None];
        model.mesh_data = vec![mesh];
        model
    }

    /// T1: Cylinder flat-end on mesh slab — upright cylinder penetrating slightly.
    /// Slab top at z=0, cylinder center at z=0.18 → bottom cap at z=-0.02 → 0.02 penetration.
    #[test]
    fn test_mesh_cylinder_flat_end_on_mesh() {
        let mesh = ground_slab_mesh();
        let model = mesh_cylinder_model(mesh, 0.2, 0.1);

        // Slab centered at z=-SLAB_HZ so top face is at z=0
        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Cylinder center at z=0.18 → bottom cap at z=-0.02 → overlaps slab top by 0.02
        let pos_cyl = Vector3::new(0.0, 0.0, 0.18);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat, pos_cyl, mat, 0.0);
        assert!(!contacts.is_empty(), "should produce contact");

        let c = &contacts[0];
        // MuJoCo lists the cylinder first (the lower geom type), its normal
        // toward the mesh below: -Z.
        assert_eq!(c.geom, [Some(1), Some(0)], "MuJoCo's geom order");
        assert!(
            c.normal.z < -0.9,
            "normal should point -Z, got {:?}",
            c.normal
        );
        // Penetration ~0.02
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive (~0.02), got {}",
            c.depth
        );
    }

    /// T2: Cylinder curved-side on mesh — lying on its side.
    /// Rotated 90 deg about Y so axis points along X. Center at z=0.08
    /// → curved surface bottom at z=-0.02 → overlaps slab top by 0.02.
    #[test]
    fn test_mesh_cylinder_curved_side_on_mesh() {
        let mesh = ground_slab_mesh();
        let model = mesh_cylinder_model(mesh, 0.2, 0.1);

        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Center at z=0.08 so curved surface (radius=0.1) bottom at z=-0.02
        let pos_cyl = Vector3::new(0.0, 0.0, 0.08);
        let mat_mesh = Matrix3::identity();
        // Rotate cylinder 90 deg about Y axis (axis now points along X)
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let mat_cyl = *rot.to_rotation_matrix().matrix();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat_mesh, pos_cyl, mat_cyl, 0.0);
        assert!(!contacts.is_empty(), "should produce contact");

        let c = &contacts[0];
        // From the cylinder, listed first, toward the mesh: -Z.
        assert_eq!(c.geom, [Some(1), Some(0)], "MuJoCo's geom order");
        assert!(
            c.normal.z < -0.9,
            "normal should point -Z, got {:?}",
            c.normal
        );
    }

    /// T5: Cylinder vs mesh — capsule regression. The cylinder's flat cap
    /// 0.01 into the slab's top: a cylinder is 0.01 deep, a capsule of the
    /// same half-length would be 0.11 (its hemisphere reaches 0.1 further).
    #[test]
    fn test_mesh_cylinder_not_capsule_regression() {
        let mesh = ground_slab_mesh();
        let model = mesh_cylinder_model(mesh, 0.2, 0.1);

        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Cylinder center at z=0.19: bottom flat cap at z=-0.01
        let pos_cyl = Vector3::new(0.0, 0.0, 0.19);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat, pos_cyl, mat, 0.0);
        assert!(
            !contacts.is_empty(),
            "cylinder at z=0.19 should contact slab"
        );

        let c = &contacts[0];
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small (~0.01) not large (~0.11 from capsule approx), got {}",
            c.depth
        );
    }

    /// T7: Mesh–Cylinder swapped geom order (Cylinder, Mesh).
    /// Should produce identical physics — same contact with correct normal convention.
    #[test]
    fn test_mesh_cylinder_swapped_geom_order() {
        let mesh = ground_slab_mesh();

        // Model with geom0 = cylinder, geom1 = mesh (swapped)
        let mut model = make_model(
            GeomType::Cylinder,
            Vector3::new(0.1, 0.2, 0.0), // radius=0.1, half_length=0.2
            GeomType::Mesh,
            Vector3::zeros(),
        );
        model.geom_mesh = vec![None, Some(0)];
        model.mesh_data = vec![mesh];

        // Cylinder at z=0.18, mesh slab at z=-SLAB_HZ
        let pos_cyl = Vector3::new(0.0, 0.0, 0.18);
        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_cyl, mat, pos_mesh, mat, 0.0);
        assert!(!contacts.is_empty(), "swapped order should produce contact");

        let c = &contacts[0];
        // The normal points from geom1 (the cylinder) toward geom2 (the mesh
        // below it): −Z.
        assert!(
            c.normal.z < -0.9,
            "normal should point down, got {:?}",
            c.normal
        );
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive, got {}",
            c.depth
        );
    }

    /// T9: No convex hull — MuJoCo's support without a graph searches every
    /// vertex (`mjc_meshSupport`), so the slab of eight vertices still
    /// collides: one contact, from the cylinder toward the mesh.
    #[test]
    fn test_mesh_cylinder_no_hull_searches_the_vertices() {
        // Create mesh WITHOUT computing convex hull
        let vertices = vec![
            Point3::new(-1.0, -1.0, -0.01),
            Point3::new(1.0, -1.0, -0.01),
            Point3::new(1.0, 1.0, -0.01),
            Point3::new(-1.0, 1.0, -0.01),
            Point3::new(-1.0, -1.0, 0.01),
            Point3::new(1.0, -1.0, 0.01),
            Point3::new(1.0, 1.0, 0.01),
            Point3::new(-1.0, 1.0, 0.01),
        ];
        #[rustfmt::skip]
        let indices = vec![
            0,1,2, 0,2,3,
            4,6,5, 4,7,6,
            0,4,5, 0,5,1,
            2,6,7, 2,7,3,
            0,3,7, 0,7,4,
            1,5,6, 1,6,2,
        ];
        let mesh = TriangleMeshData::new(vertices, indices);
        // Deliberately NOT calling mesh.compute_convex_hull()
        assert!(mesh.convex_hull().is_none(), "precondition: no hull");

        let mut model = make_model(
            GeomType::Mesh,
            Vector3::zeros(),
            GeomType::Cylinder,
            Vector3::new(0.1, 0.2, 0.0),
        );
        model.geom_mesh = vec![Some(0), None];
        model.mesh_data = vec![Arc::new(mesh)];

        let pos1 = Vector3::new(0.0, 0.0, -SLAB_HZ);
        let pos2 = Vector3::new(0.0, 0.0, 0.18);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos1, mat, pos2, mat, 0.0);
        assert_eq!(contacts.len(), 1, "one contact from the vertex set");
        let c = &contacts[0];
        assert_eq!(c.geom, [Some(1), Some(0)], "MuJoCo's geom order");
        assert!(
            c.normal.z < -0.9,
            "normal should point -Z, got {:?}",
            c.normal
        );
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive (~0.02), got {}",
            c.depth
        );
    }

    // ========================================================================
    // Mesh–Ellipsoid dispatch tests (through `mjc_convex`)
    // ========================================================================

    /// Helper: set up a (Mesh, Ellipsoid) model.
    /// Returns the model with geom0 = mesh, geom1 = ellipsoid.
    fn mesh_ellipsoid_model(mesh: Arc<TriangleMeshData>, radii: Vector3<f64>) -> Model {
        let mut model = make_model(GeomType::Mesh, Vector3::zeros(), GeomType::Ellipsoid, radii);
        model.geom_mesh = vec![Some(0), None];
        model.mesh_data = vec![mesh];
        model
    }

    /// T3: Oblate ellipsoid flat-axis on mesh slab.
    /// Radii [0.3, 0.3, 0.05] — thin disc. Center at z = 0.05 - 0.02 = 0.03
    /// so bottom at z = -0.02, penetrating slab top (z=0) by 0.02.
    /// Old sphere approx would use max_r = 0.3, making the object float.
    #[test]
    fn test_mesh_ellipsoid_oblate_on_mesh() {
        let mesh = ground_slab_mesh();
        let radii = Vector3::new(0.3, 0.3, 0.05);
        let model = mesh_ellipsoid_model(mesh, radii);

        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Center at z=0.03 → bottom at z=0.03-0.05=-0.02 → overlaps slab top by 0.02
        let pos_ell = Vector3::new(0.0, 0.0, 0.03);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat, pos_ell, mat, 0.0);
        assert!(!contacts.is_empty(), "should produce contact");

        let c = &contacts[0];
        // From the ellipsoid, listed first, toward the mesh: -Z.
        assert_eq!(c.geom, [Some(1), Some(0)], "MuJoCo's geom order");
        assert!(
            c.normal.z < -0.9,
            "normal should point -Z, got {:?}",
            c.normal
        );
        // Penetration ~0.02
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive (~0.02), got {}",
            c.depth
        );
    }

    /// T4: Prolate ellipsoid long-axis on mesh slab.
    /// Radii [0.05, 0.05, 0.3] — tall cigar shape. Center at z = 0.28
    /// → bottom at z = -0.02, penetrating slab top by 0.02.
    #[test]
    fn test_mesh_ellipsoid_prolate_on_mesh() {
        let mesh = ground_slab_mesh();
        let radii = Vector3::new(0.05, 0.05, 0.3);
        let model = mesh_ellipsoid_model(mesh, radii);

        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Center at z=0.28 → bottom at z=0.28-0.3=-0.02 → overlaps slab top
        let pos_ell = Vector3::new(0.0, 0.0, 0.28);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat, pos_ell, mat, 0.0);
        assert!(!contacts.is_empty(), "should produce contact");

        let c = &contacts[0];
        assert_eq!(c.geom, [Some(1), Some(0)], "MuJoCo's geom order");
        assert!(
            c.normal.z < -0.9,
            "normal should point -Z, got {:?}",
            c.normal
        );
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive, got {}",
            c.depth
        );
    }

    /// T6: Ellipsoid vs mesh — sphere regression. An oblate ellipsoid (radii
    /// [0.3, 0.3, 0.05]) with its bottom 0.01 into the slab's top: 0.01
    /// deep, where a sphere of the largest radius would be 0.26.
    #[test]
    fn test_mesh_ellipsoid_not_sphere_regression() {
        let mesh = ground_slab_mesh();
        let radii = Vector3::new(0.3, 0.3, 0.05);
        let model = mesh_ellipsoid_model(mesh, radii);

        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        // Center at z=0.04: bottom at z=-0.01
        let pos_ell = Vector3::new(0.0, 0.0, 0.04);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_mesh, mat, pos_ell, mat, 0.0);
        assert!(
            !contacts.is_empty(),
            "ellipsoid at z=0.04 should contact slab"
        );

        let c = &contacts[0];
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small (~0.01) not large (~0.26 from sphere approx), got {}",
            c.depth
        );
    }

    /// MuJoCo's convex collision lists a contact only where the geoms
    /// penetrate (`dist < 0`, `engine_collision_convex.c:804`): a cylinder's
    /// cap and an ellipsoid's bottom exactly on the slab's top give none.
    #[test]
    fn convex_contact_at_exact_touch_is_absent() {
        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        let mat = Matrix3::identity();
        let cylinder = mesh_cylinder_model(ground_slab_mesh(), 0.2, 0.1);
        let touching = Vector3::new(0.0, 0.0, 0.2);
        assert!(
            collide_geoms(&cylinder, 0, 1, pos_mesh, mat, touching, mat, 0.0).is_empty(),
            "a cylinder's cap on the slab"
        );
        let ellipsoid = mesh_ellipsoid_model(ground_slab_mesh(), Vector3::new(0.3, 0.3, 0.05));
        let touching = Vector3::new(0.0, 0.0, 0.05);
        assert!(
            collide_geoms(&ellipsoid, 0, 1, pos_mesh, mat, touching, mat, 0.0).is_empty(),
            "an ellipsoid's bottom on the slab"
        );
    }

    /// T8: Mesh–Ellipsoid swapped geom order (Ellipsoid, Mesh).
    #[test]
    fn test_mesh_ellipsoid_swapped_geom_order() {
        let mesh = ground_slab_mesh();
        let radii = Vector3::new(0.3, 0.3, 0.05);

        // geom0 = ellipsoid, geom1 = mesh (swapped)
        let mut model = make_model(GeomType::Ellipsoid, radii, GeomType::Mesh, Vector3::zeros());
        model.geom_mesh = vec![None, Some(0)];
        model.mesh_data = vec![mesh];

        // Ellipsoid at z=0.03 (bottom at -0.02), slab at z=-SLAB_HZ
        let pos_ell = Vector3::new(0.0, 0.0, 0.03);
        let pos_mesh = Vector3::new(0.0, 0.0, -SLAB_HZ);
        let mat = Matrix3::identity();

        let contacts = collide_geoms(&model, 0, 1, pos_ell, mat, pos_mesh, mat, 0.0);
        assert!(!contacts.is_empty(), "swapped order should produce contact");

        let c = &contacts[0];
        // The normal points from geom1 (the ellipsoid) toward geom2 (the mesh
        // below it): −Z.
        assert!(
            c.normal.z < -0.9,
            "normal should point down, got {:?}",
            c.normal
        );
        assert!(
            c.depth > 0.0 && c.depth < 0.05,
            "depth should be small positive, got {}",
            c.depth
        );
    }

    /// A box, and a mesh with polygons, resting on a mesh whose polygons were
    /// never merged (no `compute_convex_hull`) under MULTICCD, the unmerged
    /// mesh the second object of the pair and then the first: the
    /// multi-contact has no faces to clip, so each pair takes one contact per
    /// pass: the first, and those MULTICCD's four turned passes add (all four
    /// for the box, three for the mesh).
    #[test]
    fn a_mesh_without_polygons_under_multiccd() {
        let slab = box_mesh(1.0, 1.0, SLAB_HZ);
        let unmerged = Arc::new(TriangleMeshData::from_faces(
            slab.vertices().to_vec(),
            slab.triangles().to_vec(),
        ));
        assert!(
            unmerged.polygons().paths.is_empty(),
            "precondition: no polygons"
        );
        let pos_slab = Vector3::new(0.0, 0.0, -SLAB_HZ);
        let pos_upper = Vector3::new(0.0, 0.0, 0.099);
        let mat = Matrix3::identity();
        for (upper, upper_mesh, count) in [
            (GeomType::Box, None, 5),
            (GeomType::Mesh, Some(box_mesh(0.1, 0.1, 0.1)), 4),
        ] {
            let mut model = make_model(
                GeomType::Mesh,
                Vector3::zeros(),
                upper,
                Vector3::new(0.1, 0.1, 0.1),
            );
            model.enableflags |= ENABLE_MULTICCD;
            model.geom_mesh = vec![Some(0), upper_mesh.as_ref().map(|_| 1)];
            model.mesh_data = std::iter::once(Arc::clone(&unmerged))
                .chain(upper_mesh)
                .collect();
            let contacts = collide_geoms(&model, 0, 1, pos_slab, mat, pos_upper, mat, 0.0);
            assert_eq!(contacts.len(), count, "{upper:?}: one contact per pass");
            for c in &contacts {
                assert!(
                    (c.depth - 0.001).abs() < 1e-9,
                    "{upper:?}: depth {}",
                    c.depth
                );
            }
        }
    }

    /// A geom turned by a matrix holding a NaN or an infinity collides as
    /// nothing at every site that turned it into a quaternion (nalgebra's
    /// `from_matrix`, which iterates without a cap): a sphere on a signed
    /// distance field, each geom's matrix bad in turn. MuJoCo's
    /// functions, which take no quaternion, give MuJoCo's results (the ccd
    /// golden's `plane_convex` and `hfield_convex` `nan` cases): a mesh so
    /// turned on a plane, one contact at a NaN distance; a height field so
    /// turned, none; a sphere so turned on a height field, its contacts, a
    /// sphere's support not reading its turn. Each call returns.
    #[test]
    fn a_non_finite_turn_returns() {
        let n = 5;
        let hfield = Arc::new(HeightFieldData::new(vec![0.5; n * n], n, n, 1.0));
        let sdf: Arc<dyn PhysicsShape> = Arc::new(ShapeConvex::new(Arc::new(SdfGrid::sphere(
            Point3::origin(),
            1.0,
            16,
            1.0,
        ))));
        let ok = Matrix3::identity();
        for bad in [f64::NAN, f64::INFINITY] {
            let mut turned = Matrix3::identity();
            turned[(0, 1)] = bad;

            let mut plane_mesh = make_model(
                GeomType::Plane,
                Vector3::new(5.0, 5.0, 0.1),
                GeomType::Mesh,
                Vector3::zeros(),
            );
            plane_mesh.geom_mesh = vec![None, Some(0)];
            plane_mesh.mesh_data = vec![box_mesh(0.1, 0.1, 0.1)];
            let mut hfield_sphere = make_model(
                GeomType::Hfield,
                Vector3::zeros(),
                GeomType::Sphere,
                Vector3::new(0.3, 0.0, 0.0),
            );
            hfield_sphere.hfield_data = vec![Arc::clone(&hfield)];
            hfield_sphere.hfield_size = vec![[2.0, 2.0, 1.0, 1.0]];
            hfield_sphere.geom_hfield = vec![Some(0), None];
            let mut sdf_sphere = make_model(
                GeomType::Sdf,
                Vector3::zeros(),
                GeomType::Sphere,
                Vector3::new(0.3, 0.0, 0.0),
            );
            sdf_sphere.shape_data = vec![Arc::clone(&sdf)];
            sdf_sphere.geom_shape = vec![Some(0), None];

            let near = Vector3::new(0.0, 0.0, 0.7);
            // The plane's normal is its matrix's third column, which `turned`
            // leaves finite: the mesh is above the plane.
            assert!(
                collide_geoms(&plane_mesh, 0, 1, Vector3::zeros(), turned, near, ok, 0.0)
                    .is_empty(),
                "plane turned, {bad}"
            );
            let nan = collide_geoms(&plane_mesh, 0, 1, Vector3::zeros(), ok, near, turned, 0.0);
            assert!(
                nan.len() == 1 && nan[0].depth.is_nan(),
                "mesh turned on a plane, {bad}: {nan:?}"
            );
            assert!(
                collide_hfield_multi(
                    &hfield_sphere,
                    0,
                    1,
                    Vector3::zeros(),
                    turned,
                    near,
                    ok,
                    0.0
                )
                .is_empty(),
                "height field turned, {bad}"
            );
            let sphere = collide_hfield_multi(
                &hfield_sphere,
                0,
                1,
                Vector3::zeros(),
                ok,
                near,
                turned,
                0.0,
            );
            assert!(
                !sphere.is_empty() && sphere.iter().all(|c| c.depth.is_finite()),
                "sphere turned on a height field, {bad}: {sphere:?}"
            );
            for (m1, m2) in [(turned, ok), (ok, turned)] {
                assert!(
                    collide_with_sdf(&sdf_sphere, 0, 1, Vector3::zeros(), m1, near, m2, 0.0)
                        .is_empty(),
                    "signed distance field and sphere, {bad}"
                );
            }
        }
    }

    /// A box mesh with its base 1/64 into a plane, either geom first: each
    /// contact is listed plane first, its normal the plane's, midway
    /// between its corner and the plane (`mjc_PlaneConvex`'s position,
    /// `engine_collision_convex.c:1062-1063`, `:1030-1031`), at the corner's
    /// depth; three of the base's four corners (`maxplanemesh`). Before, each
    /// was at its corner, in the argument order.
    #[test]
    fn mesh_plane_contacts_at_midpoint() {
        for plane in [0, 1] {
            let mesh = 1 - plane;
            let mut model = make_model(
                GeomType::Plane,
                Vector3::new(5.0, 5.0, 0.1),
                GeomType::Mesh,
                Vector3::zeros(),
            );
            model.geom_type.swap(0, plane);
            model.geom_mesh[mesh] = Some(0);
            model.mesh_data = vec![box_mesh(0.125, 0.125, 0.0625)];
            let mut pos = [Vector3::zeros(), Vector3::zeros()];
            pos[mesh] = Vector3::new(0.25, -0.5, 0.0625 - 0.015_625);
            let eye = Matrix3::identity();
            let contacts = collide_geoms(&model, 0, 1, pos[0], eye, pos[1], eye, 0.0);
            assert_eq!(contacts.len(), 3, "plane geom {plane}: contacts");
            for c in &contacts {
                assert_eq!(
                    c.geom,
                    [Some(plane), Some(mesh)],
                    "plane geom {plane}: order"
                );
                assert_eq!(c.normal, Vector3::z(), "plane geom {plane}: normal");
                assert_eq!(
                    c.depth.to_bits(),
                    0.015_625_f64.to_bits(),
                    "plane geom {plane}: depth"
                );
                assert_eq!(
                    c.pos.z.to_bits(),
                    (-0.007_812_5_f64).to_bits(),
                    "plane geom {plane}: midway"
                );
                assert!(
                    [0.125, 0.375].contains(&c.pos.x),
                    "plane geom {plane}: a corner's x, {}",
                    c.pos.x
                );
                assert!(
                    [-0.625, -0.375].contains(&c.pos.y),
                    "plane geom {plane}: a corner's y, {}",
                    c.pos.y
                );
            }
        }
    }

    /// The ccd golden (`sim/L0/tests/scripts/gen_ccd_reference.py`).
    fn ccd_golden() -> serde_json::Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/ccd/ccd.json"
        );
        let text = std::fs::read_to_string(path).expect("the ccd golden reads");
        serde_json::from_str(&text).expect("the ccd golden parses")
    }

    fn golden_vector(v: &serde_json::Value) -> Vector3<f64> {
        let a: Vec<f64> = v
            .as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect();
        Vector3::new(a[0], a[1], a[2])
    }

    fn golden_matrix(v: &serde_json::Value) -> Matrix3<f64> {
        let a: Vec<f64> = v
            .as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect();
        Matrix3::from_row_slice(&a)
    }

    /// Ellipsoids of random sizes and turns apart from planes of random
    /// poses, each at a margin equal to MuJoCo's own distance (the ccd
    /// golden's `plane_convex` `ellipsoid_at_margin`, 95 cases): each is
    /// listed, as MuJoCo lists it (`dist <= margin`), at MuJoCo's distance
    /// and position bit for bit, from its support point taken as
    /// `mjccd_support` takes it (ledger L187). Before, the support and the
    /// distance were computed in another order, which lists fewer.
    #[test]
    fn plane_ellipsoid_listed_at_mujocos_distance() {
        let doc = ccd_golden();
        let cases = doc["plane_convex"]["ellipsoid_at_margin"]
            .as_object()
            .expect("cases");
        assert_eq!(cases.len(), 95, "cases");
        for (name, case) in cases {
            let model = make_model(
                GeomType::Plane,
                Vector3::new(5.0, 5.0, 0.1),
                GeomType::Ellipsoid,
                golden_vector(&case["size"]),
            );
            let want = &case["contacts"][0];
            let contacts = collide_geoms(
                &model,
                0,
                1,
                golden_vector(&case["plane"]["pos"]),
                golden_matrix(&case["plane"]["mat"]),
                golden_vector(&case["pos"]),
                golden_matrix(&case["mat"]),
                case["margin"].as_f64().expect("margin"),
            );
            assert_eq!(contacts.len(), 1, "{name}: listed");
            let dist = want["dist"].as_f64().expect("dist");
            assert_eq!(
                (-contacts[0].depth).to_bits(),
                dist.to_bits(),
                "{name}: dist"
            );
            assert_eq!(
                contacts[0].pos,
                golden_vector(&want["pos"]),
                "{name}: position"
            );
        }
    }
}
