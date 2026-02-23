//! Collision detection pipeline — broad-phase dispatch, affinity filtering, and
//! contact parameter mixing.
//!
//! Corresponds to MuJoCo's `engine_collision_driver.c` (broad-phase dispatch)
//! and parameter combination from `engine_collision_primitive.c`.

pub(crate) mod flex_collide;
pub(crate) mod hfield;
pub(crate) mod mesh_collide;
pub(crate) mod narrow;
pub(crate) mod pair_convex;
pub(crate) mod pair_cylinder;
pub(crate) mod plane;
pub(crate) mod sdf_collide;

// Re-exports: uncomment in Phase 12 when monolith shim is deleted and callers
// import via crate::collision:: instead of crate::mujoco_pipeline::.
// pub(crate) use narrow::{collide_geoms, geom_to_collision_shape};
// pub(crate) use flex_collide::{make_contact_flex_rigid, narrowphase_sphere_geom};

use crate::collision_shape::Aabb;
use crate::mujoco_pipeline::SweepAndPrune; // monolith: removed in Phase 8a
use crate::mujoco_pipeline::aabb_from_geom; // monolith: removed in Phase 8a
use crate::types::{Data, ENABLE_SLEEP, Model, SleepState};
use nalgebra::{Point3, Vector3};

use self::flex_collide::{make_contact_flex_rigid, narrowphase_sphere_geom};
use self::narrow::{apply_pair_overrides, collide_geoms};

// ============================================================================
// Collision affinity filtering
// ============================================================================

/// Check if two geometries can collide based on contype/conaffinity bitmasks.
///
/// Following MuJoCo's collision filtering:
/// 1. Same body - no collision
/// 2. Parent-child (adjacent bodies in kinematic tree) - no collision
///    Exception: World body (0) collides with all non-child bodies
/// 3. contype/conaffinity check: (c1 & a2) != 0 || (c2 & a1) != 0
///
/// The world body exception ensures ground planes can collide with objects
/// even when those objects are direct children of the world body.
#[allow(clippy::inline_always)] // Hot path - profiling shows inlining improves debug performance
#[inline(always)]
pub(crate) fn check_collision_affinity(model: &Model, geom1: usize, geom2: usize) -> bool {
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

    // Parent-child filtering: bodies connected by a joint shouldn't collide
    // Exception: World body (0) geometries (ground planes) should collide with
    // any body, including direct children. The world body has no joints connecting
    // it to children - children are simply anchored in world space.
    //
    // This is important because:
    // - A ball with a free joint has body_parent[ball_body] = 0 (world)
    // - But the ball should still collide with ground planes on body 0
    // - The "parent-child" filter is for bodies connected by articulated joints
    //   (hinge, slide, ball) where collision would be geometrically impossible
    if body1 != 0
        && body2 != 0
        && (model.body_parent[body1] == body2 || model.body_parent[body2] == body1)
    {
        return false;
    }

    // contype/conaffinity bitmask check
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

/// Contact parameter combination for flex-rigid collision pairs.
///
/// Mirrors `contact_param()` for geom-geom pairs, but reads `flex_*` fields
/// for the flex entity and `geom_*` fields for the rigid entity. Follows
/// MuJoCo's `mj_contactParam()` with f1=flex_id, f2=-1 (geom).
pub(crate) fn contact_param_flex_rigid(
    model: &Model,
    flex_id: usize,
    geom_idx: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let priority_flex = model.flex_priority[flex_id];
    let priority_geom = model.geom_priority[geom_idx];
    let gap = model.flex_gap[flex_id] + model.geom_gap[geom_idx];

    if priority_flex > priority_geom {
        let f = model.flex_friction[flex_id]; // scalar until Vec<Vector3> upgrade
        return (
            model.flex_condim[flex_id],
            gap,
            model.flex_solref[flex_id],
            model.flex_solimp[flex_id],
            [f, f, f, f, f], // scalar → uniform 5-element unpack
        );
    }
    if priority_geom > priority_flex {
        let f = model.geom_friction[geom_idx];
        return (
            model.geom_condim[geom_idx],
            gap,
            model.geom_solref[geom_idx],
            model.geom_solimp[geom_idx],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }

    // Equal priority — combine
    let condim = model.flex_condim[flex_id].max(model.geom_condim[geom_idx]);

    let s1 = model.flex_solmix[flex_id];
    let s2 = model.geom_solmix[geom_idx];
    let mix = solmix_weight(s1, s2);

    let solref = combine_solref(model.flex_solref[flex_id], model.geom_solref[geom_idx], mix);
    let solimp = combine_solimp(model.flex_solimp[flex_id], model.geom_solimp[geom_idx], mix);

    // Friction: element-wise max (flex scalar applied to all components)
    let ff = model.flex_friction[flex_id];
    let gf = model.geom_friction[geom_idx];
    let fri = [
        ff.max(gf.x),
        ff.max(gf.x), // sliding1, sliding2
        ff.max(gf.y), // torsional
        ff.max(gf.z),
        ff.max(gf.z), // rolling1, rolling2
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
// Broad-phase collision dispatch
// ============================================================================

/// Collision detection: populate contacts from geometry pairs.
///
/// Following `MuJoCo`'s collision detection order:
/// 1. Broad-phase: sweep-and-prune to find candidate pairs (O(n log n))
/// 2. Check contact affinity (contype/conaffinity bitmasks)
/// 3. Narrow-phase: analytical or GJK/EPA collision detection
/// 4. Populate data.contacts with contact information
///
/// This function is called after forward kinematics (`mj_fwd_position`) so
/// `geom_xpos` and `geom_xmat` are up-to-date.
///
/// # Algorithm: Sweep-and-Prune
///
/// Unlike spatial hashing which degrades to O(n²) when objects cluster,
/// sweep-and-prune maintains O(n log n + k) complexity where k is the
/// number of overlapping pairs. This is robust against clustering scenarios
/// like a pile of boxes or a humanoid with many self-collision checks.
///
/// # Performance
///
/// | Scene Size | Complexity | Notes |
/// |------------|------------|-------|
/// | Any n | O(n log n + k) | k = overlapping pairs |
/// | Coherent | O(n + k) | Nearly-sorted input |
pub(crate) fn mj_collision(model: &Model, data: &mut Data) {
    // Clear existing contacts
    data.contacts.clear();
    data.ncon = 0;

    // Rigid-rigid collision requires at least 2 geoms for a pair.
    // Even with 0 or 1 geoms, flex-vertex-vs-rigid contacts are still possible.
    if model.ngeom >= 2 {
        // Build AABBs for all geoms
        // This is O(n) and cache-friendly (linear memory access)
        let aabbs: Vec<Aabb> = (0..model.ngeom)
            .map(|geom_id| {
                let mut aabb = aabb_from_geom(
                    model.geom_type[geom_id],
                    model.geom_size[geom_id],
                    data.geom_xpos[geom_id],
                    data.geom_xmat[geom_id],
                );
                // Expand AABB by geom margin so SAP doesn't reject pairs
                // that are within margin distance but not overlapping.
                let m = model.geom_margin[geom_id];
                if m > 0.0 {
                    let expand = Vector3::new(m, m, m);
                    aabb = Aabb::new(
                        Point3::from(Vector3::new(aabb.min.x, aabb.min.y, aabb.min.z) - expand),
                        Point3::from(Vector3::new(aabb.max.x, aabb.max.y, aabb.max.z) + expand),
                    );
                }
                aabb
            })
            .collect();

        // Sweep-and-prune broad-phase: O(n log n) worst case, O(n + k) for coherent scenes
        let sap = SweepAndPrune::new(aabbs);
        let candidates = sap.query_pairs();

        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // Process candidate pairs
        // The SAP already filtered to only AABB-overlapping pairs
        for (geom1, geom2) in candidates {
            // Affinity check: same body, parent-child, contype/conaffinity
            if !check_collision_affinity(model, geom1, geom2) {
                continue;
            }

            // §16.5b: Skip narrow-phase if BOTH geoms belong to sleeping bodies.
            // Keep sleeping-vs-awake pairs — they may trigger wake detection (§16.4).
            if sleep_enabled {
                let b1 = model.geom_body[geom1];
                let b2 = model.geom_body[geom2];
                if data.body_sleep_state[b1] == SleepState::Asleep
                    && data.body_sleep_state[b2] == SleepState::Asleep
                {
                    continue;
                }
            }

            // Get world-space poses
            let pos1 = data.geom_xpos[geom1];
            let mat1 = data.geom_xmat[geom1];
            let pos2 = data.geom_xpos[geom2];
            let mat2 = data.geom_xmat[geom2];

            // Compute effective margin for this pair
            let margin = model.geom_margin[geom1] + model.geom_margin[geom2];

            // Narrow-phase collision detection
            if let Some(contact) =
                collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
            {
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }

        // Mechanism 2: explicit contact pairs (bypass kinematic + bitmask filters)
        for pair in &model.contact_pairs {
            let geom1 = pair.geom1;
            let geom2 = pair.geom2;

            // §16.5b: Skip narrow-phase if BOTH geoms belong to sleeping bodies.
            if sleep_enabled {
                let b1 = model.geom_body[geom1];
                let b2 = model.geom_body[geom2];
                if data.body_sleep_state[b1] == SleepState::Asleep
                    && data.body_sleep_state[b2] == SleepState::Asleep
                {
                    continue;
                }
            }

            // Pair margin overrides geom margins
            let margin = pair.margin;

            // Distance cull using bounding radii (replaces SAP broad-phase for pairs).
            // geom_rbound is the bounding sphere radius, pre-computed per geom.
            // For planes, rbound = INFINITY so this check always passes.
            // Margin is added to match MuJoCo's mj_filterSphere.
            let dist = (data.geom_xpos[geom1] - data.geom_xpos[geom2]).norm();
            if dist > model.geom_rbound[geom1] + model.geom_rbound[geom2] + margin {
                continue;
            }

            // Narrow-phase collision detection
            let pos1 = data.geom_xpos[geom1];
            let mat1 = data.geom_xmat[geom1];
            let pos2 = data.geom_xpos[geom2];
            let mat2 = data.geom_xmat[geom2];

            if let Some(mut contact) =
                collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
            {
                apply_pair_overrides(&mut contact, pair);
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    } // end if model.ngeom >= 2

    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);
}

/// Detect contacts between flex vertices and rigid geoms.
///
/// Each flex vertex is treated as a sphere of radius `flexvert_radius[vi]`.
/// Uses brute-force broadphase (O(V*G)) for simplicity — SAP integration
/// deferred to when nflexvert is large enough to warrant it.
/// No-op when nflexvert == 0.
fn mj_collision_flex(model: &Model, data: &mut Data) {
    if model.nflexvert == 0 {
        return;
    }

    for vi in 0..model.nflexvert {
        let vpos = data.flexvert_xpos[vi];
        let flex_id = model.flexvert_flexid[vi];
        let radius = model.flexvert_radius[vi];
        let margin = model.flex_margin[flex_id];

        // Skip pinned vertices (infinite mass = immovable)
        if model.flexvert_invmass[vi] <= 0.0 {
            continue;
        }

        // Per-flex bitmask values (invariant across the inner geom loop)
        let fcontype = model.flex_contype[flex_id];
        let fconaffinity = model.flex_conaffinity[flex_id];

        for gi in 0..model.ngeom {
            // Proper contype/conaffinity bitmask filtering (MuJoCo filterBitmask protocol).
            // Collision proceeds iff: (flex_contype & geom_conaffinity) != 0
            //                      || (geom_contype & flex_conaffinity) != 0
            let gcontype = model.geom_contype[gi];
            let gconaffinity = model.geom_conaffinity[gi];
            if (fcontype & gconaffinity) == 0 && (gcontype & fconaffinity) == 0 {
                continue;
            }

            // Skip geoms belonging to the same body as this vertex
            let vertex_body = model.flexvert_bodyid[vi];
            let geom_body = model.geom_body[gi];
            if geom_body == vertex_body {
                continue;
            }
            // Skip geoms on the vertex's parent body (node body for node-flex)
            if geom_body < model.nbody && model.body_parent[vertex_body] == geom_body {
                continue;
            }

            // Narrowphase: vertex sphere vs rigid geom
            let geom_pos = data.geom_xpos[gi];
            let geom_mat = data.geom_xmat[gi];

            if let Some((depth, normal, contact_pos)) =
                narrowphase_sphere_geom(vpos, radius + margin, gi, model, geom_pos, geom_mat)
            {
                let contact = make_contact_flex_rigid(model, vi, gi, contact_pos, normal, depth);
                data.contacts.push(contact);
                data.ncon += 1;
            }
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
    use nalgebra::UnitQuaternion;

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
