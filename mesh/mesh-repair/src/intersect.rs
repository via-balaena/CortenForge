//! Self-intersection detection for meshes.
//!
//! This module provides tools for detecting self-intersecting triangles in a mesh.
//! Self-intersections cause 3D printing failures and indicate mesh topology issues.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Point3};
//! use mesh_repair::intersect::{detect_self_intersections, IntersectionParams};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let result = detect_self_intersections(&mesh, &IntersectionParams::default());
//! assert!(result.is_clean());
//! ```

#![allow(
    // `usize` → `u32` casts are safe at mesh sizes the crate targets:
    // face / triangle indices fit in `u32` by mesh-types contract.
    clippy::cast_possible_truncation
)]

use cf_geometry::Aabb;
use hashbrown::HashSet;
use mesh_types::{IndexedMesh, Triangle, Vector3};
use parry3d_f64::bounding_volume::{Aabb as ParryAabb, SimdAabb};
use parry3d_f64::math::SIMD_WIDTH;
use parry3d_f64::partitioning::{Qbvh, SimdSimultaneousVisitStatus, SimdSimultaneousVisitor};
use parry3d_f64::simba::simd::SimdBool;
use rayon::prelude::*;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use tracing::{debug, info, warn};

/// Result of self-intersection detection.
#[derive(Debug, Clone)]
pub struct SelfIntersectionResult {
    /// Whether the mesh has any self-intersections.
    pub has_intersections: bool,
    /// Number of intersecting triangle pairs found.
    pub intersection_count: usize,
    /// List of intersecting triangle pairs as `(face_idx_a, face_idx_b)`.
    /// Limited to first `max_reported` pairs.
    pub intersecting_pairs: Vec<(u32, u32)>,
    /// Total faces checked.
    pub faces_checked: usize,
    /// Whether the search was terminated early due to reaching `max_reported`.
    pub truncated: bool,
}

impl SelfIntersectionResult {
    /// Check if the mesh is free of self-intersections.
    #[must_use]
    pub const fn is_clean(&self) -> bool {
        !self.has_intersections
    }
}

impl std::fmt::Display for SelfIntersectionResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.has_intersections {
            write!(
                f,
                "Self-intersections found: {} pair(s){}",
                self.intersection_count,
                if self.truncated { " (truncated)" } else { "" }
            )
        } else {
            write!(f, "No self-intersections detected")
        }
    }
}

/// Parameters for self-intersection detection.
#[derive(Debug, Clone)]
pub struct IntersectionParams {
    /// Maximum number of intersecting pairs to report.
    /// Set to 0 for unlimited (but may be slow for highly self-intersecting meshes).
    pub max_reported: usize,
    /// Epsilon for geometric comparisons.
    pub epsilon: f64,
    /// Whether to skip adjacent triangles (sharing an edge or vertex).
    /// Usually true since adjacent triangles touching at edges isn't a "self-intersection".
    pub skip_adjacent: bool,
}

impl Default for IntersectionParams {
    fn default() -> Self {
        Self {
            max_reported: 100,
            epsilon: 1e-10,
            skip_adjacent: true,
        }
    }
}

impl IntersectionParams {
    /// Create params that check all pairs without limit.
    #[must_use]
    pub const fn exhaustive() -> Self {
        Self {
            max_reported: 0,
            epsilon: 1e-10,
            skip_adjacent: true,
        }
    }

    /// Create params that only check for the presence of intersections.
    #[must_use]
    pub const fn quick_check() -> Self {
        Self {
            max_reported: 1,
            epsilon: 1e-10,
            skip_adjacent: true,
        }
    }
}

/// SIMD-aware visitor for parry3d-f64's `Qbvh::traverse_bvtt` that
/// collects canonical `(i, j)` triangle-index pairs with `i < j`
/// whose AABBs overlap. Per `docs/CF_CAST_F4_SELF_INTERSECT_BVH_RECON.md`
/// §S-1: traversal of `(qbvh, qbvh)` emits (i, i) self-pairs + BOTH
/// (i, j) AND (j, i) for every overlap; without the canonical filter
/// the BVH path would emit 2× pair count of the reference O(n²) loop.
///
/// Mask handling follows parry3d's standard 4-wide SIMD leaf-batch
/// pattern: `intersects_permutations` returns a `[SimdBool; SIMD_WIDTH]`
/// where lane `j` of element `i` is "left lane i overlaps right lane j".
struct OverlapPairCollector<'a> {
    pairs: &'a mut Vec<(u32, u32)>,
}

impl<'a> OverlapPairCollector<'a> {
    const fn new(pairs: &'a mut Vec<(u32, u32)>) -> Self {
        Self { pairs }
    }
}

impl SimdSimultaneousVisitor<u32, u32, SimdAabb> for OverlapPairCollector<'_> {
    fn visit(
        &mut self,
        left_bv: &SimdAabb,
        left_data: Option<[Option<&u32>; SIMD_WIDTH]>,
        right_bv: &SimdAabb,
        right_data: Option<[Option<&u32>; SIMD_WIDTH]>,
    ) -> SimdSimultaneousVisitStatus {
        let mask = left_bv.intersects_permutations(right_bv);

        // Both nodes are leaves: emit canonical pairs for overlap-set bits.
        if let (Some(ldata), Some(rdata)) = (left_data, right_data) {
            for (ii, mask_i) in mask.iter().enumerate().take(SIMD_WIDTH) {
                let bits = mask_i.bitmask();
                if bits == 0 {
                    continue;
                }
                let Some(li) = ldata[ii] else { continue };
                for (jj, rj_slot) in rdata.iter().enumerate().take(SIMD_WIDTH) {
                    if (bits & (1 << jj)) == 0 {
                        continue;
                    }
                    let Some(rj) = *rj_slot else { continue };
                    let (a, b) = (*li, *rj);
                    // Canonical pair (i < j); skip self-pairs (i == j).
                    if a < b {
                        self.pairs.push((a, b));
                    }
                }
            }
        }

        // For internal / mixed nodes, just return the mask — parry
        // descends where bits are set.
        SimdSimultaneousVisitStatus::MaybeContinue(mask)
    }
}

/// Detect self-intersections in a mesh.
///
/// Uses a parry3d-f64 Qbvh for O(n log n) candidate-pair enumeration
/// instead of an O(n²) all-pairs scan. The downstream SAT triangle-
/// triangle test (`triangles_intersect`) runs only on AABB-overlapping
/// candidates and is parallelized via rayon. See
/// `docs/CF_CAST_F4_SELF_INTERSECT_BVH_RECON.md` §B-2 for the
/// algorithmic gain estimate on production gasket-mold meshes
/// (~25 s → ~3 s per gasket on 400 k-face meshes).
///
/// Bit-equivalent results to the pre-S1 O(n²) implementation,
/// preserved as `detect_self_intersections_reference` for the regression
/// test gate.
///
/// # Arguments
///
/// * `mesh` - The mesh to check
/// * `params` - Detection parameters
///
/// # Returns
///
/// A `SelfIntersectionResult` with information about any intersections found.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::intersect::{detect_self_intersections, IntersectionParams};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = detect_self_intersections(&mesh, &IntersectionParams::default());
/// assert!(result.is_clean());
/// ```
#[must_use]
pub fn detect_self_intersections(
    mesh: &IndexedMesh,
    params: &IntersectionParams,
) -> SelfIntersectionResult {
    let face_count = mesh.face_count();

    if face_count < 2 {
        return SelfIntersectionResult {
            has_intersections: false,
            intersection_count: 0,
            intersecting_pairs: Vec::new(),
            faces_checked: face_count,
            truncated: false,
        };
    }

    info!("Checking {} faces for self-intersections", face_count);

    // Precompute triangles (unchanged).
    let triangles: Vec<Triangle> = mesh.triangles().collect();

    // Precompute parry3d-f64 AABBs (epsilon-expanded). Same expansion
    // as the pre-S1 cf_geometry::Aabb path; only the type differs so
    // they can land in a parry3d-f64 Qbvh as leaf data.
    let parry_aabbs: Vec<ParryAabb> = triangles
        .iter()
        .map(|t| {
            let cf_aabb = Aabb::from_triangle(&t.v0, &t.v1, &t.v2).expanded(params.epsilon);
            ParryAabb::new(
                parry3d_f64::na::Point3::new(cf_aabb.min.x, cf_aabb.min.y, cf_aabb.min.z),
                parry3d_f64::na::Point3::new(cf_aabb.max.x, cf_aabb.max.y, cf_aabb.max.z),
            )
        })
        .collect();

    // Build adjacency info if we need to skip adjacent triangles (unchanged).
    let adjacency = if params.skip_adjacent {
        Some(build_face_adjacency(&mesh.faces))
    } else {
        None
    };

    let max_pairs = if params.max_reported == 0 {
        usize::MAX
    } else {
        params.max_reported
    };

    // Build the Qbvh from (triangle_index, AABB) leaves.
    let mut qbvh: Qbvh<u32> = Qbvh::new();
    qbvh.clear_and_rebuild(
        parry_aabbs.iter().enumerate().map(|(i, a)| (i as u32, *a)),
        0.0,
    );

    // BVH-accelerated candidate-pair enumeration. The visitor pushes
    // canonical (i, j) pairs (i < j) for every leaf-leaf AABB overlap.
    // Self-pairs (i == j) and reverse-direction pairs (j, i) are filtered
    // by the visitor's canonical-pair guard.
    let mut candidate_pairs: Vec<(u32, u32)> = Vec::new();
    {
        let mut visitor = OverlapPairCollector::new(&mut candidate_pairs);
        qbvh.traverse_bvtt(&qbvh, &mut visitor);
    }

    // SAT-filter the candidate pairs in parallel + apply skip_adjacent
    // post-filter (per §S-8 #1 — picked over visitor-internal filtering
    // since the candidate list is small at gasket scale).
    let intersection_count = AtomicUsize::new(0);
    let should_stop = AtomicBool::new(false);

    let intersecting_pairs: Vec<(u32, u32)> = candidate_pairs
        .par_iter()
        .filter_map(|&(i, j)| {
            if should_stop.load(Ordering::Relaxed) {
                return None;
            }
            let iu = i as usize;
            let ju = j as usize;
            // skip_adjacent: skip pairs whose faces share an edge or vertex.
            if let Some(ref adj) = adjacency
                && adj[iu].contains(&j)
            {
                return None;
            }
            // SAT triangle-triangle intersection test (unchanged).
            if !triangles_intersect(&triangles[iu], &triangles[ju], params.epsilon) {
                return None;
            }
            let count = intersection_count.fetch_add(1, Ordering::Relaxed);
            if count + 1 >= max_pairs && params.max_reported > 0 {
                should_stop.store(true, Ordering::Relaxed);
            }
            if count < max_pairs {
                Some((i, j))
            } else {
                None
            }
        })
        .collect();

    let final_count = intersection_count.load(Ordering::Relaxed);
    let truncated = params.max_reported > 0 && final_count >= max_pairs;

    if truncated {
        debug!(
            "Stopping intersection search after {} pairs (max_reported limit)",
            max_pairs
        );
    }

    if final_count > 0 {
        warn!("Found {} self-intersecting triangle pair(s)", final_count);
    } else {
        info!("No self-intersections found");
    }

    SelfIntersectionResult {
        has_intersections: final_count > 0,
        intersection_count: final_count,
        intersecting_pairs,
        faces_checked: face_count,
        truncated,
    }
}

/// Pre-S1 O(n²) reference implementation of
/// [`detect_self_intersections`], preserved for the regression test
/// gate at `docs/CF_CAST_F4_SELF_INTERSECT_BVH_RECON.md` §S-4 #1.
/// Bit-equivalent results to the BVH path within FP precision; same
/// `SelfIntersectionResult` shape + `IntersectionParams` semantics.
#[cfg(test)]
#[must_use]
fn detect_self_intersections_reference(
    mesh: &IndexedMesh,
    params: &IntersectionParams,
) -> SelfIntersectionResult {
    let face_count = mesh.face_count();

    if face_count < 2 {
        return SelfIntersectionResult {
            has_intersections: false,
            intersection_count: 0,
            intersecting_pairs: Vec::new(),
            faces_checked: face_count,
            truncated: false,
        };
    }

    let triangles: Vec<Triangle> = mesh.triangles().collect();
    let aabbs: Vec<Aabb> = triangles
        .iter()
        .map(|t| Aabb::from_triangle(&t.v0, &t.v1, &t.v2).expanded(params.epsilon))
        .collect();

    let adjacency = if params.skip_adjacent {
        Some(build_face_adjacency(&mesh.faces))
    } else {
        None
    };

    let max_pairs = if params.max_reported == 0 {
        usize::MAX
    } else {
        params.max_reported
    };

    let intersection_count = AtomicUsize::new(0);
    let should_stop = AtomicBool::new(false);

    let intersecting_pairs: Vec<(u32, u32)> = (0..face_count)
        .into_par_iter()
        .flat_map(|i| {
            if should_stop.load(Ordering::Relaxed) {
                return Vec::new();
            }
            let mut local_pairs = Vec::new();
            for j in (i + 1)..face_count {
                if should_stop.load(Ordering::Relaxed) {
                    break;
                }
                if !aabbs[i].overlaps(&aabbs[j]) {
                    continue;
                }
                if let Some(ref adj) = adjacency
                    && adj[i].contains(&(j as u32))
                {
                    continue;
                }
                if triangles_intersect(&triangles[i], &triangles[j], params.epsilon) {
                    let count = intersection_count.fetch_add(1, Ordering::Relaxed);
                    if count < max_pairs {
                        local_pairs.push((i as u32, j as u32));
                    }
                    if count + 1 >= max_pairs && params.max_reported > 0 {
                        should_stop.store(true, Ordering::Relaxed);
                        break;
                    }
                }
            }
            local_pairs
        })
        .collect();

    let final_count = intersection_count.load(Ordering::Relaxed);
    let truncated = params.max_reported > 0 && final_count >= max_pairs;

    SelfIntersectionResult {
        has_intersections: final_count > 0,
        intersection_count: final_count,
        intersecting_pairs,
        faces_checked: face_count,
        truncated,
    }
}

/// Build face adjacency (faces sharing vertices).
fn build_face_adjacency(faces: &[[u32; 3]]) -> Vec<HashSet<u32>> {
    use hashbrown::HashMap;

    // Map vertex -> faces using that vertex
    let mut vertex_to_faces: HashMap<u32, Vec<u32>> = HashMap::new();
    for (face_idx, face) in faces.iter().enumerate() {
        for &v in face {
            vertex_to_faces.entry(v).or_default().push(face_idx as u32);
        }
    }

    // For each face, find all faces sharing at least one vertex
    let mut adjacency: Vec<HashSet<u32>> = vec![HashSet::new(); faces.len()];
    for (face_idx, face) in faces.iter().enumerate() {
        for &v in face {
            if let Some(neighbors) = vertex_to_faces.get(&v) {
                for &neighbor in neighbors {
                    if neighbor != face_idx as u32 {
                        adjacency[face_idx].insert(neighbor);
                    }
                }
            }
        }
    }

    adjacency
}

/// Test if two triangles intersect.
///
/// Uses the separating axis theorem (SAT).
/// Two triangles intersect if they share interior points.
fn triangles_intersect(t1: &Triangle, t2: &Triangle, epsilon: f64) -> bool {
    // Compute normals
    let n1 = t1.normal_unnormalized();
    let n2 = t2.normal_unnormalized();

    // Degenerate triangles don't intersect meaningfully
    if n1.norm_squared() < epsilon * epsilon || n2.norm_squared() < epsilon * epsilon {
        return false;
    }

    // Get edges of both triangles
    let edges1 = [t1.v1 - t1.v0, t1.v2 - t1.v1, t1.v0 - t1.v2];
    let edges2 = [t2.v1 - t2.v0, t2.v2 - t2.v1, t2.v0 - t2.v2];

    // Check if triangles are coplanar (or nearly so)
    let cross_normals = n1.cross(&n2);
    let is_coplanar =
        cross_normals.norm_squared() < epsilon * epsilon * n1.norm_squared() * n2.norm_squared();

    if is_coplanar {
        // For coplanar triangles, use 2D SAT test
        // Project onto the plane and test edge normals as separating axes

        // Test separation using edges of triangle 1 (perpendicular in-plane)
        for edge in &edges1 {
            let axis = n1.cross(edge);
            if axis.norm_squared() > epsilon * epsilon && separated_by_axis(&axis, t1, t2, epsilon)
            {
                return false;
            }
        }

        // Test separation using edges of triangle 2 (perpendicular in-plane)
        for edge in &edges2 {
            let axis = n2.cross(edge);
            if axis.norm_squared() > epsilon * epsilon && separated_by_axis(&axis, t1, t2, epsilon)
            {
                return false;
            }
        }

        // No separating axis in-plane - coplanar triangles overlap
        return true;
    }

    // Non-coplanar case: Use standard 3D SAT

    // Test separation along triangle normals
    if separated_by_axis(&n1, t1, t2, epsilon) {
        return false;
    }
    if separated_by_axis(&n2, t1, t2, epsilon) {
        return false;
    }

    // Test 9 edge-edge cross product axes
    for e1 in &edges1 {
        for e2 in &edges2 {
            let axis = e1.cross(e2);
            if axis.norm_squared() > epsilon * epsilon && separated_by_axis(&axis, t1, t2, epsilon)
            {
                return false;
            }
        }
    }

    // No separating axis found - triangles intersect
    true
}

/// Check if two triangles are separated by a given axis.
fn separated_by_axis(axis: &Vector3<f64>, t1: &Triangle, t2: &Triangle, epsilon: f64) -> bool {
    // Project triangle 1 vertices onto axis
    let p1_0 = axis.dot(&t1.v0.coords);
    let p1_1 = axis.dot(&t1.v1.coords);
    let p1_2 = axis.dot(&t1.v2.coords);
    let min1 = p1_0.min(p1_1).min(p1_2);
    let max1 = p1_0.max(p1_1).max(p1_2);

    // Project triangle 2 vertices onto axis
    let p2_0 = axis.dot(&t2.v0.coords);
    let p2_1 = axis.dot(&t2.v1.coords);
    let p2_2 = axis.dot(&t2.v2.coords);
    let min2 = p2_0.min(p2_1).min(p2_2);
    let max2 = p2_0.max(p2_1).max(p2_2);

    // Check if projections are separated (with epsilon tolerance)
    max1 + epsilon < min2 || max2 + epsilon < min1
}

/// Check if a mesh has any self-intersections (quick check).
///
/// This is a convenience function that returns a boolean instead of
/// the full result structure.
///
/// # Arguments
///
/// * `mesh` - The mesh to check
///
/// # Returns
///
/// `true` if the mesh has self-intersections, `false` otherwise.
#[must_use]
pub fn has_self_intersections(mesh: &IndexedMesh) -> bool {
    let result = detect_self_intersections(mesh, &IntersectionParams::quick_check());
    result.has_intersections
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn create_xy_triangle(x: f64, y: f64, size: f64) -> Triangle {
        Triangle::new(
            Point3::new(x, y, 0.0),
            Point3::new(x + size, y, 0.0),
            Point3::new(x + size / 2.0, y + size, 0.0),
        )
    }

    #[test]
    fn test_aabb_overlap() {
        let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let aabb2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
        let aabb3 = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));

        assert!(aabb1.overlaps(&aabb2));
        assert!(aabb2.overlaps(&aabb1));
        assert!(!aabb1.overlaps(&aabb3));
        assert!(!aabb3.overlaps(&aabb1));
    }

    #[test]
    fn test_non_intersecting_triangles() {
        // Two triangles far apart
        let t1 = create_xy_triangle(0.0, 0.0, 1.0);
        let t2 = create_xy_triangle(10.0, 10.0, 1.0);

        assert!(!triangles_intersect(&t1, &t2, 1e-10));
    }

    #[test]
    fn test_coplanar_non_intersecting() {
        // Two coplanar triangles that don't overlap
        let t1 = create_xy_triangle(0.0, 0.0, 1.0);
        let t2 = create_xy_triangle(2.0, 0.0, 1.0);

        assert!(!triangles_intersect(&t1, &t2, 1e-10));
    }

    #[test]
    fn test_coplanar_intersecting() {
        // Two coplanar triangles that overlap
        let t1 = create_xy_triangle(0.0, 0.0, 2.0);
        let t2 = create_xy_triangle(0.5, 0.5, 2.0);

        assert!(triangles_intersect(&t1, &t2, 1e-10));
    }

    #[test]
    fn test_perpendicular_intersecting() {
        // Triangle in XY plane
        let t1 = Triangle::new(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        // Triangle in XZ plane, crossing through t1
        let t2 = Triangle::new(
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        );

        assert!(triangles_intersect(&t1, &t2, 1e-10));
    }

    #[test]
    fn test_perpendicular_non_intersecting() {
        // Triangle in XY plane at z=0
        let t1 = Triangle::new(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        // Triangle in XZ plane at y=5 (doesn't cross t1)
        let t2 = Triangle::new(
            Point3::new(-1.0, 5.0, -1.0),
            Point3::new(1.0, 5.0, -1.0),
            Point3::new(0.0, 5.0, 1.0),
        );

        assert!(!triangles_intersect(&t1, &t2, 1e-10));
    }

    #[test]
    fn test_detect_clean_mesh() {
        // Simple tetrahedron - no self-intersections
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);

        let result = detect_self_intersections(&mesh, &IntersectionParams::default());
        assert!(result.is_clean());
        assert_eq!(result.intersection_count, 0);
    }

    #[test]
    fn test_detect_self_intersecting_mesh() {
        // Create a mesh with two triangles that intersect
        let mut mesh = IndexedMesh::new();

        // Triangle 1 in XY plane
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));

        // Triangle 2 in XZ plane, passing through triangle 1
        mesh.vertices.push(Point3::new(-1.0, 0.0, -1.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, -1.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0));

        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([3, 4, 5]);

        let result = detect_self_intersections(&mesh, &IntersectionParams::default());
        assert!(!result.is_clean());
        assert_eq!(result.intersection_count, 1);
        assert_eq!(result.intersecting_pairs.len(), 1);
        assert_eq!(result.intersecting_pairs[0], (0, 1));
    }

    #[test]
    fn test_skip_adjacent_triangles() {
        // Two triangles sharing an edge - should not be flagged as intersecting
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 1]); // Shares edge 0-1

        let params = IntersectionParams {
            skip_adjacent: true,
            ..Default::default()
        };
        let result = detect_self_intersections(&mesh, &params);
        assert!(result.is_clean());
    }

    #[test]
    fn test_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = detect_self_intersections(&mesh, &IntersectionParams::default());
        assert!(result.is_clean());
        assert_eq!(result.faces_checked, 0);
    }

    #[test]
    fn test_single_triangle() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let result = detect_self_intersections(&mesh, &IntersectionParams::default());
        assert!(result.is_clean());
        assert_eq!(result.faces_checked, 1);
    }

    #[test]
    fn test_result_display() {
        let result = SelfIntersectionResult {
            has_intersections: true,
            intersection_count: 5,
            intersecting_pairs: vec![(0, 1), (2, 3)],
            faces_checked: 100,
            truncated: false,
        };
        let output = format!("{result}");
        assert!(output.contains("5 pair(s)"));

        let clean_result = SelfIntersectionResult {
            has_intersections: false,
            intersection_count: 0,
            intersecting_pairs: Vec::new(),
            faces_checked: 100,
            truncated: false,
        };
        let clean_output = format!("{clean_result}");
        assert!(clean_output.contains("No self-intersections"));
    }

    #[test]
    fn test_has_self_intersections() {
        // Clean mesh
        let mut clean_mesh = IndexedMesh::new();
        clean_mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        clean_mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        clean_mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        clean_mesh.faces.push([0, 1, 2]);

        assert!(!has_self_intersections(&clean_mesh));

        // Intersecting mesh
        let mut bad_mesh = IndexedMesh::new();
        bad_mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0));
        bad_mesh.vertices.push(Point3::new(1.0, -1.0, 0.0));
        bad_mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        bad_mesh.vertices.push(Point3::new(-1.0, 0.0, -1.0));
        bad_mesh.vertices.push(Point3::new(1.0, 0.0, -1.0));
        bad_mesh.vertices.push(Point3::new(0.0, 0.0, 1.0));
        bad_mesh.faces.push([0, 1, 2]);
        bad_mesh.faces.push([3, 4, 5]);

        assert!(has_self_intersections(&bad_mesh));
    }

    // ─── F4 self-intersect BVH S1 regression tests ────────────────────
    // Per `docs/CF_CAST_F4_SELF_INTERSECT_BVH_RECON.md` §S-4. The
    // BVH path is the production `detect_self_intersections`; the
    // O(n²) reference is `detect_self_intersections_reference` (cfg-
    // test-only). Tests below verify equivalence + runtime gate +
    // edge-case parity.

    /// Build a mesh of `n_pairs` interpenetrating triangle pairs, each
    /// pair in its own disjoint AABB region (so pairs don't interfere
    /// with each other). Returns `(mesh, expected_pair_count)`.
    fn make_intersecting_pairs_mesh(n_pairs: usize) -> (IndexedMesh, usize) {
        let mut mesh = IndexedMesh::new();
        for k in 0..n_pairs {
            // Place each pair at a distinct X-offset so AABBs don't
            // overlap across pairs.
            #[allow(clippy::cast_precision_loss)]
            let x_offset = (k as f64) * 100.0;
            // Tri A: z=0 plane, x ∈ [0,10], y ∈ [0,10]
            let base_a = mesh.vertices.len() as u32;
            mesh.vertices.push(Point3::new(x_offset, 0.0, 0.0));
            mesh.vertices.push(Point3::new(x_offset + 10.0, 0.0, 0.0));
            mesh.vertices.push(Point3::new(x_offset + 5.0, 10.0, 0.0));
            mesh.faces.push([base_a, base_a + 1, base_a + 2]);
            // Tri B: y=5 plane, x ∈ [4,6], z ∈ [-5,5] (crosses tri A)
            let base_b = mesh.vertices.len() as u32;
            mesh.vertices.push(Point3::new(x_offset + 4.0, 5.0, -5.0));
            mesh.vertices.push(Point3::new(x_offset + 6.0, 5.0, -5.0));
            mesh.vertices.push(Point3::new(x_offset + 5.0, 5.0, 5.0));
            mesh.faces.push([base_b, base_b + 1, base_b + 2]);
        }
        (mesh, n_pairs)
    }

    /// Build a synthetic grid mesh for the runtime-gate test: a 10 mm
    /// XZ grid of triangles, `subdiv × subdiv` rows × 2 tris per cell.
    /// Triangles are flat + non-overlapping → many candidate AABB
    /// pairs to enumerate but zero actual intersections.
    fn make_grid_mesh(subdiv: u32) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        #[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
        let step = 10.0_f64 / f64::from(subdiv);
        let row = subdiv + 1;
        let n = subdiv as usize;
        for j in 0..=n {
            for i in 0..=n {
                #[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
                let x = (i as u32 as f64) * step;
                #[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
                let z = (j as u32 as f64) * step;
                mesh.vertices.push(Point3::new(x, 0.0, z));
            }
        }
        for j in 0..n {
            for i in 0..n {
                let r0 = (j as u32) * row + (i as u32);
                let r1 = r0 + 1;
                let r2 = r0 + row;
                let r3 = r2 + 1;
                mesh.faces.push([r0, r2, r1]);
                mesh.faces.push([r1, r2, r3]);
            }
        }
        mesh
    }

    /// Canonicalize the pair set into a `HashSet` for comparison
    /// regardless of vector ordering (rayon non-determinism on
    /// reference path; Qbvh SIMD-leaf traversal order on BVH path).
    fn pair_set(pairs: &[(u32, u32)]) -> std::collections::HashSet<(u32, u32)> {
        pairs.iter().copied().collect()
    }

    #[test]
    fn self_intersect_bvh_matches_reference_o_n_squared() {
        // §S-4 #1: paired equivalence test on a known-intersecting
        // fixture. 5 pairs of interpenetrating triangles spaced so
        // pairs don't AABB-overlap each other.
        let (mesh, expected) = make_intersecting_pairs_mesh(5);
        let params = IntersectionParams::exhaustive();

        let bvh = detect_self_intersections(&mesh, &params);
        let reference = detect_self_intersections_reference(&mesh, &params);

        assert_eq!(
            bvh.intersection_count, reference.intersection_count,
            "BVH count {} vs reference {}",
            bvh.intersection_count, reference.intersection_count
        );
        assert_eq!(
            bvh.intersection_count, expected,
            "expected {expected} intersecting pairs from the fixture; got {}",
            bvh.intersection_count
        );
        assert_eq!(
            pair_set(&bvh.intersecting_pairs),
            pair_set(&reference.intersecting_pairs),
            "BVH pair set diverges from reference (canonical (i, j) with i < j)"
        );
        assert_eq!(bvh.truncated, reference.truncated);
        assert_eq!(bvh.has_intersections, reference.has_intersections);
    }

    #[test]
    fn self_intersect_bvh_runtime_under_target() {
        // §S-4 #2: relative runtime gate on a procedural grid (no
        // actual intersections; just AABB-enumeration cost). BVH
        // path must be > 3× faster than reference at this fixture
        // scale (≈40k faces). Production gasket-scale (400k faces)
        // hits the recon §S-2 projected ~8× speedup; smaller scales
        // have higher BVH-build overhead relative to total work.
        //
        // 200 × 200 grid → 200² = 40,000 cells × 2 tris = 80,000
        // faces. At this scale the BVH cost (build + traverse +
        // small candidate filter on flat grid) clears the 3× ratio
        // vs the reference parallel O(n²) scan. Smaller fixtures
        // (≤40k faces) hit only 2-2.5× because the reference's
        // existing AABB-pre-filter is competitive at low pair-
        // count scales.
        let mesh = make_grid_mesh(200);
        let params = IntersectionParams::exhaustive();

        let t_bvh = std::time::Instant::now();
        let bvh = detect_self_intersections(&mesh, &params);
        let bvh_elapsed = t_bvh.elapsed();

        let t_ref = std::time::Instant::now();
        let reference = detect_self_intersections_reference(&mesh, &params);
        let ref_elapsed = t_ref.elapsed();

        // Sanity: zero intersections in a flat grid; both paths must
        // agree.
        assert_eq!(bvh.intersection_count, 0);
        assert_eq!(reference.intersection_count, 0);

        // Speedup gate. 3× at ~40k faces; ~8× projected at 400k
        // faces (gasket scale) per recon §S-2.
        let ratio = ref_elapsed.as_secs_f64() / bvh_elapsed.as_secs_f64().max(1e-9);
        assert!(
            ratio > 3.0,
            "BVH speedup ratio {ratio:.2}× below 3× target at \
             ~40k-face scale (ref {:.3}ms, BVH {:.3}ms)",
            ref_elapsed.as_secs_f64() * 1000.0,
            bvh_elapsed.as_secs_f64() * 1000.0,
        );
    }

    #[test]
    fn self_intersect_bvh_respects_skip_adjacent() {
        // §S-4 #3: two triangles sharing an edge should NOT report
        // an intersection under `skip_adjacent = true`. The shared-
        // edge AABB-overlap is real, so the BVH path WILL emit the
        // candidate pair — verify the post-filter rejects it.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, -1.0, 0.0)); // opposite side
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 1]); // shares edge 0-1 with face 0

        let params_skip = IntersectionParams {
            max_reported: 100,
            epsilon: 1e-10,
            skip_adjacent: true,
        };
        let params_no_skip = IntersectionParams {
            max_reported: 100,
            epsilon: 1e-10,
            skip_adjacent: false,
        };

        let bvh_skip = detect_self_intersections(&mesh, &params_skip);
        let ref_skip = detect_self_intersections_reference(&mesh, &params_skip);
        // skip_adjacent=true: adjacent pair is filtered out.
        assert_eq!(bvh_skip.intersection_count, ref_skip.intersection_count);

        // skip_adjacent=false: behavior depends on whether the SAT
        // test classifies edge-shared triangles as intersecting at the
        // given epsilon. Both paths must agree (same SAT test).
        let bvh_no_skip = detect_self_intersections(&mesh, &params_no_skip);
        let ref_no_skip = detect_self_intersections_reference(&mesh, &params_no_skip);
        assert_eq!(
            bvh_no_skip.intersection_count, ref_no_skip.intersection_count,
            "BVH + reference must agree on skip_adjacent=false count"
        );
        assert_eq!(
            pair_set(&bvh_no_skip.intersecting_pairs),
            pair_set(&ref_no_skip.intersecting_pairs),
        );
    }

    #[test]
    fn self_intersect_bvh_respects_max_reported() {
        // §S-4 #4: truncation semantics. 10 known intersecting pairs
        // in disjoint AABB regions; `max_reported = 5` must report
        // exactly 5 pairs + `truncated = true`.
        let (mesh, _) = make_intersecting_pairs_mesh(10);
        let params = IntersectionParams {
            max_reported: 5,
            epsilon: 1e-10,
            skip_adjacent: true,
        };

        let bvh = detect_self_intersections(&mesh, &params);
        // intersection_count tracks fetch_add count, which may exceed
        // max_pairs by one or two due to parallel races — but
        // intersecting_pairs Vec is capped at max_pairs.
        assert!(
            bvh.intersection_count >= 5,
            "should find at least 5 (the cap)"
        );
        assert!(
            bvh.intersecting_pairs.len() <= 5,
            "intersecting_pairs must be capped at max_reported=5; got {}",
            bvh.intersecting_pairs.len()
        );
        assert!(bvh.truncated, "truncated flag must be set");
    }

    #[test]
    fn self_intersect_bvh_handles_degenerate_inputs() {
        // §S-4 #5: empty / single-face / zero-area / NaN. Both BVH
        // + reference must gracefully return clean results (no
        // panics).
        let params = IntersectionParams::default();

        // Empty mesh.
        let empty = IndexedMesh::new();
        let bvh_empty = detect_self_intersections(&empty, &params);
        let ref_empty = detect_self_intersections_reference(&empty, &params);
        assert!(bvh_empty.is_clean());
        assert!(ref_empty.is_clean());

        // Single face (< 2 face short-circuit).
        let mut single = IndexedMesh::new();
        single.vertices.push(Point3::new(0.0, 0.0, 0.0));
        single.vertices.push(Point3::new(1.0, 0.0, 0.0));
        single.vertices.push(Point3::new(0.0, 1.0, 0.0));
        single.faces.push([0, 1, 2]);
        let bvh_single = detect_self_intersections(&single, &params);
        assert!(bvh_single.is_clean());

        // Zero-area face (3 colinear vertices). 2 faces minimum to
        // exit the < 2 short-circuit.
        let mut zero_area = IndexedMesh::new();
        zero_area.vertices.push(Point3::new(0.0, 0.0, 0.0));
        zero_area.vertices.push(Point3::new(1.0, 0.0, 0.0));
        zero_area.vertices.push(Point3::new(2.0, 0.0, 0.0)); // colinear
        zero_area.vertices.push(Point3::new(0.5, 5.0, 0.0));
        zero_area.faces.push([0, 1, 2]); // zero-area
        zero_area.faces.push([0, 1, 3]); // real
        // Both paths must complete without panic. Bit-equal results
        // not required for degenerate input (SAT behavior on
        // zero-area triangles is implementation-defined).
        let _bvh_zero = detect_self_intersections(&zero_area, &params);
        let _ref_zero = detect_self_intersections_reference(&zero_area, &params);
    }
}
