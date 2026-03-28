//! Forward kinematics and position-stage computations.
//!
//! Computes body poses, site poses, geom poses, subtree centers of mass,
//! and AABBs from joint positions. Corresponds to the position-stage of
//! MuJoCo's `engine_core_smooth.c`.

use crate::collision::narrow::GEOM_EPSILON;
use crate::dynamics::compute_body_spatial_inertia;
use crate::tendon::mj_fwd_tendon;
use crate::types::{Data, ENABLE_SLEEP, MjJointType, Model, SleepState};
use cf_geometry::Aabb;
use nalgebra::{Matrix3, Matrix6, Point3, UnitQuaternion, Vector3};

/// Forward kinematics: compute body poses from qpos.
///
/// This traverses the kinematic tree from root to leaves, computing
/// the world-frame position and orientation of each body.
pub fn mj_fwd_position(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Body 0 (world) is always at origin
    data.xpos[0] = Vector3::zeros();
    data.xquat[0] = UnitQuaternion::identity();
    data.xmat[0] = Matrix3::identity();

    // Process bodies in order (assumes topological sort: parent before child).
    // Phase B (§16.15): compute FK for ALL bodies (including sleeping) to detect
    // external qpos modifications via xpos/xquat comparison. The Phase A skip
    // is removed; performance comes from per-island constraint solve (§16.16).
    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Determine body pose: mocap bodies use mocap arrays, regular bodies
        // use parent frame + body offset + joints.
        let (pos, quat) = if let Some(&Some(mocap_idx)) = model.body_mocapid.get(body_id) {
            // Mocap body: replace body_pos/body_quat with mocap arrays.
            // Parent is always world (origin), so xpos = mocap_pos, xquat = mocap_quat.
            //
            // Renormalize quaternion (matching MuJoCo's mju_normalize4 in
            // mj_kinematics1). Users set mocap_quat between steps; floating-point
            // arithmetic can cause drift from unit norm.
            let mquat = UnitQuaternion::new_normalize(data.mocap_quat[mocap_idx].into_inner());
            (data.mocap_pos[mocap_idx], mquat)
        } else {
            // Regular body: parent frame + body offset + joints.
            let mut pos = data.xpos[parent_id];
            let mut quat = data.xquat[parent_id];

            // Apply body offset in parent frame
            pos += quat * model.body_pos[body_id];
            quat *= model.body_quat[body_id];

            // Apply each joint for this body
            let jnt_start = model.body_jnt_adr[body_id];
            let jnt_end = jnt_start + model.body_jnt_num[body_id];

            for jnt_id in jnt_start..jnt_end {
                let qpos_adr = model.jnt_qpos_adr[jnt_id];

                match model.jnt_type[jnt_id] {
                    MjJointType::Hinge => {
                        let angle = data.qpos[qpos_adr];
                        let axis = model.jnt_axis[jnt_id];
                        let anchor = model.jnt_pos[jnt_id];

                        // Transform anchor to current frame
                        let world_anchor = pos + quat * anchor;

                        // Rotate around axis
                        let world_axis = quat * axis;
                        // Safety: use try_new_normalize to handle degenerate cases
                        let rot =
                            if let Some(unit_axis) = nalgebra::Unit::try_new(world_axis, 1e-10) {
                                UnitQuaternion::from_axis_angle(&unit_axis, angle)
                            } else {
                                // Degenerate axis - no rotation (should not happen with valid model)
                                UnitQuaternion::identity()
                            };
                        quat = rot * quat;

                        // Adjust position for rotation around anchor
                        pos = world_anchor + rot * (pos - world_anchor);
                    }
                    MjJointType::Slide => {
                        let displacement = data.qpos[qpos_adr];
                        let axis = model.jnt_axis[jnt_id];
                        pos += quat * (axis * displacement);
                    }
                    MjJointType::Ball => {
                        // qpos stores quaternion [w, x, y, z]
                        // Must normalize — integration can drift from unit norm.
                        let q = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                            data.qpos[qpos_adr],
                            data.qpos[qpos_adr + 1],
                            data.qpos[qpos_adr + 2],
                            data.qpos[qpos_adr + 3],
                        ));
                        quat *= q;
                    }
                    MjJointType::Free => {
                        // qpos stores [x, y, z, qw, qx, qy, qz]
                        pos = Vector3::new(
                            data.qpos[qpos_adr],
                            data.qpos[qpos_adr + 1],
                            data.qpos[qpos_adr + 2],
                        );
                        // Must normalize — integration can drift from unit norm.
                        quat = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                            data.qpos[qpos_adr + 3],
                            data.qpos[qpos_adr + 4],
                            data.qpos[qpos_adr + 5],
                            data.qpos[qpos_adr + 6],
                        ));
                    }
                }
            }

            (pos, quat)
        };

        // §16.15: qpos change detection — if this body was sleeping and its
        // computed pose differs from the stored pose (exact bitwise match),
        // mark the tree for waking. This detects external qpos modifications.
        if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
            let tree = model.body_treeid[body_id];
            if tree < data.tree_qpos_dirty.len() {
                // Compare new vs stored (exact floating-point equality)
                let pos_changed = pos != data.xpos[body_id];
                let quat_changed =
                    quat.into_inner().coords != data.xquat[body_id].into_inner().coords;
                if pos_changed || quat_changed {
                    data.tree_qpos_dirty[tree] = true;
                }
            }
        }

        // Store computed pose (shared path for both mocap and regular bodies)
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();

        // Compute inertial frame position (shared)
        data.xipos[body_id] = pos + quat * model.body_ipos[body_id];
        data.ximat[body_id] = (quat * model.body_iquat[body_id])
            .to_rotation_matrix()
            .into_inner();

        // Compute body spatial inertia in world frame (cinert, shared).
        // MuJoCo does NOT zero mass for mocap bodies, and neither do we.
        let h = data.xipos[body_id] - pos; // COM offset from body origin in world frame
        data.cinert[body_id] = compute_body_spatial_inertia(
            model.body_mass[body_id],
            model.body_inertia[body_id],
            &data.ximat[body_id],
            h,
        );
    }

    // World body has zero inertia
    data.cinert[0] = Matrix6::zeros();

    // Update geom poses
    for geom_id in 0..model.ngeom {
        let body_id = model.geom_body[geom_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.geom_xpos[geom_id] = body_pos + body_quat * model.geom_pos[geom_id];
        data.geom_xmat[geom_id] = (body_quat * model.geom_quat[geom_id])
            .to_rotation_matrix()
            .into_inner();
    }

    // Update site poses
    for site_id in 0..model.nsite {
        let body_id = model.site_body[site_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.site_xpos[site_id] = body_pos + body_quat * model.site_pos[site_id];
        let site_world_quat = body_quat * model.site_quat[site_id];
        data.site_xmat[site_id] = site_world_quat.to_rotation_matrix().into_inner();
        data.site_xquat[site_id] = site_world_quat;
    }

    // Tendon kinematics (after site poses, before subtree COM)
    mj_fwd_tendon(model, data);

    // ========== Compute subtree mass and COM (for O(n) RNE gravity) ==========
    // Initialize with each body's own mass and COM
    for body_id in 0..model.nbody {
        data.subtree_mass[body_id] = model.body_mass[body_id];
        data.subtree_com[body_id] = model.body_mass[body_id] * data.xipos[body_id];
    }

    // Backward pass: accumulate children's contributions to parents
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        // Copy child data first to satisfy borrow checker
        let child_mass = data.subtree_mass[body_id];
        let child_weighted_com = data.subtree_com[body_id];
        data.subtree_mass[parent_id] += child_mass;
        data.subtree_com[parent_id] += child_weighted_com;
    }

    // Convert weighted sum to actual COM
    for body_id in 0..model.nbody {
        if data.subtree_mass[body_id] > 1e-10 {
            data.subtree_com[body_id] /= data.subtree_mass[body_id];
        } else {
            data.subtree_com[body_id] = data.xipos[body_id];
        }
    }
}

// ============================================================================
// Broad-Phase Collision Detection (Spatial Hashing)
// ============================================================================

/// Transform a pre-computed local-frame AABB to world space.
///
/// `geom_aabb` is `[cx, cy, cz, hx, hy, hz]` — center offset and half-extents
/// in geom-local frame (from `model.geom_aabb`, pre-computed at build time).
///
/// The world-space AABB is computed by rotating the local OBB (oriented bounding
/// box) into world frame and taking its axis-aligned envelope. This is the
/// standard rotated-box AABB formula:
///
///   world_center = pos + mat * local_center
///   world_half[i] = Σ_j |mat[i,j]| * local_half[j]
///
/// Works for all geom types uniformly — no type dispatch needed at runtime.
#[inline]
pub fn aabb_from_geom_aabb(geom_aabb: [f64; 6], pos: Vector3<f64>, mat: Matrix3<f64>) -> Aabb {
    let local_center = Vector3::new(geom_aabb[0], geom_aabb[1], geom_aabb[2]);
    let local_half = Vector3::new(geom_aabb[3], geom_aabb[4], geom_aabb[5]);

    // Transform center to world space
    let world_center = pos + mat * local_center;

    // Compute world-space half-extents from the rotated OBB.
    // For each world axis i, the half-extent is the sum of |mat[i,j]| * local_half[j].
    let abs_mat = mat.abs();
    let world_half = abs_mat * local_half;

    Aabb::new(
        Point3::from(world_center - world_half),
        Point3::from(world_center + world_half),
    )
}

/// Sweep-and-prune broad-phase collision detection.
///
/// This is the algorithm used by MuJoCo and most physics engines:
/// 1. Project all AABBs onto the X-axis
/// 2. Sort by min-X coordinate
/// 3. Sweep through sorted intervals to find overlaps
/// 4. Check Y and Z overlap only for X-overlapping pairs
///
/// For coherent simulations (objects move incrementally), the sort is nearly
/// O(n) due to temporal coherence — insertion sort on nearly-sorted data.
///
/// # Performance Characteristics
///
/// | Operation | Complexity | Notes |
/// |-----------|------------|-------|
/// | Build | O(n log n) | Initial sort (Rust's pdqsort) |
/// | Query (typical) | O(n + k) | k = output pairs, assumes bounded X-overlap |
/// | Query (worst) | O(n² + k) | All AABBs overlap on X-axis (degenerate) |
/// | Incremental | O(n) | Nearly sorted → insertion sort behavior |
///
/// The query worst case occurs when all objects have overlapping X-intervals
/// (e.g., objects stacked vertically). In practice, this is rare and still
/// faster than spatial hashing's worst case (clustering in one cell).
///
/// # Why Not Spatial Hash?
///
/// Spatial hashing degrades to O(n²) when objects cluster in a single cell
/// (e.g., a pile of boxes). SAP's worst case is the same O(n²), but it occurs
/// less frequently in practice (requires all X-intervals to overlap, not just
/// spatial proximity).
pub struct SweepAndPrune {
    /// AABBs indexed by geom ID
    aabbs: Vec<Aabb>,
    /// Geom IDs sorted by AABB min-X coordinate
    sorted_x: Vec<usize>,
}

impl SweepAndPrune {
    /// Create a new sweep-and-prune structure from AABBs.
    ///
    /// # Arguments
    ///
    /// * `aabbs` - Vector of AABBs indexed by geom ID
    ///
    /// # Panics (debug only)
    ///
    /// Debug builds assert that all AABBs have finite coordinates.
    /// NaN in AABBs would cause non-deterministic sort behavior.
    #[must_use]
    pub fn new(aabbs: Vec<Aabb>) -> Self {
        // Validate AABBs in debug builds to catch NaN/Inf early.
        // NaN comparisons break sort transitivity, causing non-deterministic results.
        debug_assert!(
            aabbs.iter().all(|a| {
                a.min.x.is_finite()
                    && a.min.y.is_finite()
                    && a.min.z.is_finite()
                    && a.max.x.is_finite()
                    && a.max.y.is_finite()
                    && a.max.z.is_finite()
            }),
            "All AABBs must have finite coordinates for deterministic sweep-and-prune"
        );

        let n = aabbs.len();
        let mut sorted_x: Vec<usize> = (0..n).collect();

        // Sort by min-X coordinate using total ordering.
        // f64::total_cmp provides IEEE 754 total ordering: -NaN < -Inf < ... < Inf < NaN
        // This guarantees deterministic sort even if validation is skipped in release.
        sorted_x.sort_by(|&a, &b| aabbs[a].min.x.total_cmp(&aabbs[b].min.x));

        Self { aabbs, sorted_x }
    }

    /// Query all potentially overlapping pairs.
    ///
    /// Returns pairs `(geom_i, geom_j)` where `i < j` and AABBs overlap.
    /// The pairs are returned in arbitrary order.
    ///
    /// See struct-level documentation for complexity analysis.
    #[must_use]
    pub fn query_pairs(&self) -> Vec<(usize, usize)> {
        // Pre-allocate with heuristic: ~2 overlaps per geom on average
        let mut pairs = Vec::with_capacity(self.aabbs.len() * 2);
        let n = self.sorted_x.len();

        // Sweep through sorted list
        for i in 0..n {
            let geom_i = self.sorted_x[i];
            let aabb_i = &self.aabbs[geom_i];
            let max_x_i = aabb_i.max.x;

            // Check subsequent geoms until their min-X exceeds our max-X
            for j in (i + 1)..n {
                let geom_j = self.sorted_x[j];
                let aabb_j = &self.aabbs[geom_j];

                // If min-X of j exceeds max-X of i, no more overlaps possible
                // (since sorted_x is sorted by min-X)
                if aabb_j.min.x > max_x_i {
                    break;
                }

                // X overlaps — check Y and Z
                if aabb_i.max.y >= aabb_j.min.y
                    && aabb_j.max.y >= aabb_i.min.y
                    && aabb_i.max.z >= aabb_j.min.z
                    && aabb_j.max.z >= aabb_i.min.z
                {
                    // Full 3D overlap — add pair (normalized: smaller ID first)
                    let (g1, g2) = if geom_i < geom_j {
                        (geom_i, geom_j)
                    } else {
                        (geom_j, geom_i)
                    };
                    pairs.push((g1, g2));
                }
            }
        }

        pairs
    }
}

/// Find closest point on line segment AB to point P.
#[inline]
pub fn closest_point_segment(a: Vector3<f64>, b: Vector3<f64>, p: Vector3<f64>) -> Vector3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < GEOM_EPSILON {
        return a; // Degenerate segment
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Find closest points between two line segments.
///
/// Returns (`point_on_seg1`, `point_on_seg2`).
///
/// Uses standard segment-segment distance algorithm with single-letter variables
/// matching the mathematical notation from computational geometry literature.
#[allow(clippy::many_single_char_names)]
pub fn closest_points_segments(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let d1 = q1 - p1; // Direction of segment 1
    let d2 = q2 - p2; // Direction of segment 2
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Check for degenerate segments
    if a < GEOM_EPSILON && e < GEOM_EPSILON {
        return (p1, p2);
    }
    if a < GEOM_EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }
    if e < GEOM_EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    // Determinant of the 2x2 system: denom = a*e - b² (intentionally b*b, not a*b)
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    // Compute initial s and t parameters for the closest points
    let (mut s, mut t) = if denom.abs() < GEOM_EPSILON {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Clamp to [0,1] and recompute
    if s < 0.0 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else if s > 1.0 {
        s = 1.0;
        t = ((b + f) / e).clamp(0.0, 1.0);
    }

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (p1 + d1 * s, p2 + d2 * t)
}

/// Like [`closest_points_segments`] but returns parametric values and clamping state
/// instead of points. Used by capsule-box collision to classify contact features.
///
/// Returns `(s, t, clamp_s, clamp_t)` where:
/// - `s`, `t`: clamped parameter values in `[0, 1]`
/// - `clamp_s`, `clamp_t`: `0` = clamped to start, `1` = interior, `2` = clamped to end
#[allow(clippy::many_single_char_names)]
pub fn closest_points_segments_parametric(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> (f64, f64, u8, u8) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Degenerate segments
    if a < GEOM_EPSILON && e < GEOM_EPSILON {
        return (0.0, 0.0, 0, 0);
    }
    if a < GEOM_EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        let clamp_t = if t <= 0.0 {
            0
        } else if t >= 1.0 {
            2
        } else {
            1
        };
        return (0.0, t, 0, clamp_t);
    }
    if e < GEOM_EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        let clamp_s = if s <= 0.0 {
            0
        } else if s >= 1.0 {
            2
        } else {
            1
        };
        return (s, 0.0, clamp_s, 0);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    let (mut s, mut t) = if denom.abs() < GEOM_EPSILON {
        // Parallel segments — convention: s=0, clamp_s=0
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Track clamping state through the clamp-and-recompute cascade
    let mut clamp_s: u8 = u8::from(denom.abs() >= GEOM_EPSILON);
    let mut clamp_t: u8 = 1;

    if s < 0.0 {
        s = 0.0;
        clamp_s = 0;
        t = (f / e).clamp(0.0, 1.0);
        clamp_t = if t <= 0.0 {
            0
        } else if t >= 1.0 {
            2
        } else {
            1
        };
    } else if s > 1.0 {
        s = 1.0;
        clamp_s = 2;
        t = ((b + f) / e).clamp(0.0, 1.0);
        clamp_t = if t <= 0.0 {
            0
        } else if t >= 1.0 {
            2
        } else {
            1
        };
    }

    if t < 0.0 {
        t = 0.0;
        clamp_t = 0;
        s = (-c / a).clamp(0.0, 1.0);
        clamp_s = if s <= 0.0 {
            0
        } else if s >= 1.0 {
            2
        } else {
            1
        };
    } else if t > 1.0 {
        t = 1.0;
        clamp_t = 2;
        s = ((b - c) / a).clamp(0.0, 1.0);
        clamp_s = if s <= 0.0 {
            0
        } else if s >= 1.0 {
            2
        } else {
            1
        };
    }

    (s, t, clamp_s, clamp_t)
}
