//! Forward kinematics and position-stage computations.
//!
//! Computes body poses, site poses, geom poses, subtree centers of mass,
//! and AABBs from joint positions. Corresponds to the position-stage of
//! MuJoCo's `engine_core_smooth.c`.

use crate::collision::narrow::GEOM_EPSILON;
use crate::collision_shape::Aabb;
use crate::dynamics::compute_body_spatial_inertia;
use crate::tendon::mj_fwd_tendon;
use crate::types::{Data, ENABLE_SLEEP, GeomType, MjJointType, Model, SleepState};
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
                        let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
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
                        quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
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

/// Compute AABB for a geometry given its world-space pose and type/size.
///
/// This function creates an axis-aligned bounding box for MuJoCo-style geometry
/// specifications, using the canonical `Aabb` type from `collision_shape`.
///
/// # Arguments
/// * `geom_type` - The geometry type (sphere, box, capsule, etc.)
/// * `size` - MuJoCo-style size parameters (interpretation depends on geom_type)
/// * `pos` - World-space position of the geometry
/// * `mat` - World-space rotation matrix (3x3)
#[allow(dead_code)] // consumers still import from monolith; live after Phase 8b
#[inline]
pub fn aabb_from_geom(
    geom_type: GeomType,
    size: Vector3<f64>,
    pos: Vector3<f64>,
    mat: Matrix3<f64>,
) -> Aabb {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            Aabb::new(
                Point3::new(pos.x - r, pos.y - r, pos.z - r),
                Point3::new(pos.x + r, pos.y + r, pos.z + r),
            )
        }
        GeomType::Box => {
            // For a rotated box, compute the world-space extents
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
        GeomType::Capsule => {
            // Capsule: radius + half_length along Z axis
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Cylinder => {
            // Cylinder AABB: endpoints ± radius in all directions.
            // Same formula as capsule — the flat caps still extend radius `r`
            // perpendicular to the axis, so the bounding box is identical.
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Ellipsoid => {
            // Conservative AABB: use sphere with largest semi-axis radius.
            // A tight AABB would require transforming each axis by the rotation
            // matrix, but this simple approach is correct and fast for broad-phase.
            let max_r = size.x.max(size.y).max(size.z);
            Aabb::new(
                Point3::new(pos.x - max_r, pos.y - max_r, pos.z - max_r),
                Point3::new(pos.x + max_r, pos.y + max_r, pos.z + max_r),
            )
        }
        GeomType::Plane => {
            // Planes are infinite — use large bounds in perpendicular directions
            // and thin bounds along the normal to allow proper AABB overlap tests.
            const PLANE_EXTENT: f64 = 1e6; // Effectively infinite for simulation scale
            const PLANE_THICKNESS: f64 = 0.001; // Thin slab for AABB overlap detection

            // Plane normal is Z axis of the rotation matrix
            let normal = mat.column(2).into_owned();

            // Create thin slab AABB based on dominant normal direction
            if normal.z.abs() > 0.9 {
                // Near-horizontal plane (normal ≈ ±Z)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, -PLANE_EXTENT, pos.z - PLANE_THICKNESS),
                    Point3::new(PLANE_EXTENT, PLANE_EXTENT, pos.z + PLANE_THICKNESS),
                )
            } else if normal.y.abs() > 0.9 {
                // Near-vertical plane (normal ≈ ±Y)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, pos.y - PLANE_THICKNESS, -PLANE_EXTENT),
                    Point3::new(PLANE_EXTENT, pos.y + PLANE_THICKNESS, PLANE_EXTENT),
                )
            } else {
                // Near-vertical plane (normal ≈ ±X)
                Aabb::new(
                    Point3::new(pos.x - PLANE_THICKNESS, -PLANE_EXTENT, -PLANE_EXTENT),
                    Point3::new(pos.x + PLANE_THICKNESS, PLANE_EXTENT, PLANE_EXTENT),
                )
            }
        }
        GeomType::Mesh => {
            // For mesh, use a conservative large bounding box.
            // In a full implementation, mesh AABBs would be pre-computed from vertices.
            const MESH_DEFAULT_EXTENT: f64 = 10.0; // Conservative fallback for unprocessed meshes
            Aabb::new(
                Point3::new(
                    pos.x - MESH_DEFAULT_EXTENT,
                    pos.y - MESH_DEFAULT_EXTENT,
                    pos.z - MESH_DEFAULT_EXTENT,
                ),
                Point3::new(
                    pos.x + MESH_DEFAULT_EXTENT,
                    pos.y + MESH_DEFAULT_EXTENT,
                    pos.z + MESH_DEFAULT_EXTENT,
                ),
            )
        }
        GeomType::Hfield => {
            // Conservative: treat as a rotated box with half-extents from geom_size
            // [x_half_extent, y_half_extent, z_top]. Same formula as GeomType::Box.
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
        GeomType::Sdf => {
            // Conservative: rotated box with half-extents from geom_size.
            // For programmatic SDF geoms, geom_size should store meaningful
            // half-extents. For MJCF placeholders, defaults to 0.1 —
            // small AABB is acceptable since the geom has no sdf_data.
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
    }
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
#[allow(dead_code)] // consumers still import from monolith; live after Phase 8b
pub struct SweepAndPrune {
    /// AABBs indexed by geom ID
    aabbs: Vec<Aabb>,
    /// Geom IDs sorted by AABB min-X coordinate
    sorted_x: Vec<usize>,
}

#[allow(dead_code)] // consumers still import from monolith; live after Phase 8b
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
#[allow(dead_code)] // consumers still import from monolith; live after Phase 8b
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
#[allow(dead_code)] // consumers still import from monolith; live after Phase 8b
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
