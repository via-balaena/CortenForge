//! Forward kinematics and position-stage computations.
//!
//! Computes body poses, site poses, geom poses, subtree centers of mass,
//! and AABBs from joint positions. Corresponds to the position-stage of
//! MuJoCo's `engine_core_smooth.c`.

use crate::collision::narrow::GEOM_EPSILON;
use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::compute_body_spatial_inertia;
use crate::quat;
use crate::quat::mul_mat_vec3;
use crate::tendon::mj_fwd_tendon;
use crate::types::{Data, ENABLE_SLEEP, MjJointType, Model, SameFrame, SleepState};
use cf_geometry::Aabb;
use nalgebra::{DVector, Matrix3, Matrix6, Point3, Quaternion, UnitQuaternion, Vector3};

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

    // Process bodies in order (assumes topological sort: parent before child),
    // sleeping ones included.
    for body_id in 1..model.nbody {
        let (pos, xquat) = body_pose(model, data, body_id);
        let pos = Vector3::from(pos);
        let quat =
            UnitQuaternion::new_unchecked(Quaternion::new(xquat[0], xquat[1], xquat[2], xquat[3]));

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
        data.xmat[body_id] = mat3(&quat::quat_to_mat(&xquat));

        // Compute inertial frame position (shared)
        let (xipos, _, ximat) = local_to_global(
            data,
            body_id,
            &model.body_ipos[body_id],
            &model.body_iquat[body_id],
            sameframe(&model.body_sameframe, body_id),
        );
        data.xipos[body_id] = xipos;
        data.ximat[body_id] = ximat;

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
        let (xpos, _, xmat) = local_to_global(
            data,
            model.geom_body[geom_id],
            &model.geom_pos[geom_id],
            &model.geom_quat[geom_id],
            sameframe(&model.geom_sameframe, geom_id),
        );
        data.geom_xpos[geom_id] = xpos;
        data.geom_xmat[geom_id] = xmat;
    }

    // Update site poses
    for site_id in 0..model.nsite {
        let (xpos, xquat, xmat) = local_to_global(
            data,
            model.site_body[site_id],
            &model.site_pos[site_id],
            &model.site_quat[site_id],
            sameframe(&model.site_sameframe, site_id),
        );
        data.site_xpos[site_id] = xpos;
        data.site_xmat[site_id] = xmat;
        data.site_xquat[site_id] =
            UnitQuaternion::new_unchecked(Quaternion::new(xquat[0], xquat[1], xquat[2], xquat[3]));
    }

    // ========== Subtree mass and COM, before the tendons (MuJoCo's `mj_comPos`
    // precedes `mj_tendon`, `engine_forward.c:140-145`; a spatial tendon's
    // Jacobian reads `subtree_com`) ==========
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

    // Convert weighted sum to actual COM as `mj_comPos` normalizes it
    // (`engine_core_smooth.c:274-284`): a subtree with less than mjMINVAL of
    // mass takes the body's own centre of mass; another is scaled by the
    // reciprocal of its mass.
    for body_id in 0..model.nbody {
        if data.subtree_mass[body_id] < MJ_MINVAL {
            data.subtree_com[body_id] = data.xipos[body_id];
        } else {
            data.subtree_com[body_id] *= 1.0 / data.subtree_mass[body_id];
        }
    }

    // Tendon kinematics (after site poses and the subtree COM)
    mj_fwd_tendon(model, data);
}

/// The world pose of body `body_id` (`[x, y, z]`, `[w, x, y, z]`) from its
/// parent's, its offset (or its mocap pose) and its joints, in MuJoCo 3.5.0's
/// arithmetic (`mj_kinematics1`, `engine_core_smooth.c:40-187`). Writes its
/// joints' `xanchor` and `xaxis`: each joint's anchor and axis in the frame
/// before it turns, as MuJoCo's.
///
/// A hinge or a slide moves by `qpos − qpos0`; a hinge or a ball turns about
/// its anchor, so a `jnt_pos` off the body origin moves the origin. A free
/// joint, alone on its body, sets the pose from `qpos`.
fn body_pose(model: &Model, data: &mut Data, body_id: usize) -> ([f64; 3], [f64; 4]) {
    let jnt_adr = model.body_jnt_adr[body_id];
    let jnt_num = model.body_jnt_num[body_id];
    let (xpos, mut xquat) = if jnt_num == 1 && model.jnt_type[jnt_adr] == MjJointType::Free {
        let pose = free_pose(&data.qpos, model.jnt_qpos_adr[jnt_adr]);
        data.xanchor[jnt_adr] = Vector3::from(pose.0);
        data.xaxis[jnt_adr] = model.jnt_axis[jnt_adr];
        pose
    } else {
        let (body_pos, body_quat) = match model.body_mocapid.get(body_id) {
            Some(&Some(mocap)) => {
                let mut quat = quat_array(&data.mocap_quat[mocap]);
                quat::normalize4(&mut quat);
                (data.mocap_pos[mocap].into(), quat)
            }
            _ => (
                model.body_pos[body_id].into(),
                quat_array(&model.body_quat[body_id]),
            ),
        };
        let parent = model.body_parent[body_id];
        let (mut xpos, mut xquat) = if parent == 0 {
            (body_pos, body_quat)
        } else {
            (
                add3(
                    &mul_mat_vec3(&data.xmat[parent], &body_pos),
                    &data.xpos[parent].into(),
                ),
                quat::mul_quat(&quat_array(&data.xquat[parent]), &body_quat),
            )
        };
        for jnt_id in jnt_adr..jnt_adr + jnt_num {
            let qadr = model.jnt_qpos_adr[jnt_id];
            let jnt_axis: [f64; 3] = model.jnt_axis[jnt_id].into();
            let jnt_pos: [f64; 3] = model.jnt_pos[jnt_id].into();
            let xaxis = quat::rot_vec_quat(&jnt_axis, &xquat);
            let xanchor = add3(&quat::rot_vec_quat(&jnt_pos, &xquat), &xpos);
            match model.jnt_type[jnt_id] {
                MjJointType::Slide => {
                    let displacement = data.qpos[qadr] - model.qpos0[qadr];
                    for i in 0..3 {
                        xpos[i] += xaxis[i] * displacement;
                    }
                }
                jnt_type @ (MjJointType::Hinge | MjJointType::Ball) => {
                    let local = if jnt_type == MjJointType::Ball {
                        quat::qpos_quat(&data.qpos, qadr)
                    } else {
                        quat::axis_angle_to_quat(&jnt_axis, data.qpos[qadr] - model.qpos0[qadr])
                    };
                    xquat = quat::mul_quat(&xquat, &local);
                    let offset = quat::rot_vec_quat(&jnt_pos, &xquat);
                    xpos = [
                        xanchor[0] - offset[0],
                        xanchor[1] - offset[1],
                        xanchor[2] - offset[2],
                    ];
                }
                // MuJoCo refuses a free joint that shares its body; this
                // one sets the pose from `qpos` as a lone one does.
                MjJointType::Free => (xpos, xquat) = free_pose(&data.qpos, qadr),
            }
            data.xanchor[jnt_id] = Vector3::from(xanchor);
            data.xaxis[jnt_id] = Vector3::from(xaxis);
        }
        (xpos, xquat)
    };
    quat::normalize4(&mut xquat);
    (xpos, xquat)
}

/// A free joint's pose from `qpos[qadr..qadr + 7]`, its quaternion normalized.
fn free_pose(qpos: &DVector<f64>, qadr: usize) -> ([f64; 3], [f64; 4]) {
    (
        [qpos[qadr], qpos[qadr + 1], qpos[qadr + 2]],
        quat::qpos_quat(qpos, qadr + 3),
    )
}

/// The world position, orientation and rotation matrix of a frame at `pos`,
/// `quat` in body `body_id`'s frame (MuJoCo `mj_local2Global`,
/// `engine_core_util.c:844-885`): the position and the matrix are copied
/// from the body's frame or its inertial frame where `same` says the frame
/// is that one, as MuJoCo does. The quaternion, which MuJoCo does not store,
/// is computed whatever `same` says, as MuJoCo's `framequat` sensor computes
/// a site's.
fn local_to_global(
    data: &Data,
    body_id: usize,
    pos: &Vector3<f64>,
    quat: &UnitQuaternion<f64>,
    same: SameFrame,
) -> (Vector3<f64>, [f64; 4], Matrix3<f64>) {
    let xpos = match same {
        SameFrame::Body => data.xpos[body_id],
        SameFrame::Inertia => data.xipos[body_id],
        SameFrame::None | SameFrame::BodyRot | SameFrame::InertiaRot => Vector3::from(add3(
            &mul_mat_vec3(&data.xmat[body_id], &(*pos).into()),
            &data.xpos[body_id].into(),
        )),
    };
    let xquat = quat::mul_quat(&quat_array(&data.xquat[body_id]), &quat_array(quat));
    let xmat = match same {
        SameFrame::Body | SameFrame::BodyRot => data.xmat[body_id],
        SameFrame::Inertia | SameFrame::InertiaRot => data.ximat[body_id],
        SameFrame::None => mat3(&quat::quat_to_mat(&xquat)),
    };
    (xpos, xquat, xmat)
}

/// Entry `i` of a `*_sameframe` array; [`SameFrame::None`], which computes
/// the frame, for a model built without [`Model::compute_simple_bodies`].
fn sameframe(flags: &[SameFrame], i: usize) -> SameFrame {
    flags.get(i).copied().unwrap_or_default()
}

/// A row-major 3×3 matrix.
fn mat3(m: &[f64; 9]) -> Matrix3<f64> {
    Matrix3::new(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8])
}

/// `[w, x, y, z]` of a unit quaternion.
fn quat_array(q: &UnitQuaternion<f64>) -> [f64; 4] {
    [q.w, q.i, q.j, q.k]
}

fn add3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
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
///   `world_half[i] = Σ_j |mat[i,j]| * local_half[j]`
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
// Joint-frame composition uses single-letter names (q, v, R, p) following the standard rigid-body kinematics notation.
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
// Joint-frame composition uses single-letter names following the standard rigid-body kinematics notation.
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
    // Cross-axis arrangements in the rotation composition match the published quaternion-product expansion.
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
