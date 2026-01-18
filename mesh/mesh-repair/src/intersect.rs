//! Self-intersection detection for meshes.
//!
//! This module provides tools for detecting self-intersecting triangles in a mesh.
//! Self-intersections cause 3D printing failures and indicate mesh topology issues.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_repair::intersect::{detect_self_intersections, IntersectionParams};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let result = detect_self_intersections(&mesh, &IntersectionParams::default());
//! assert!(result.is_clean());
//! ```

use hashbrown::HashSet;
use mesh_types::{IndexedMesh, MeshTopology, Point3, Triangle, Vector3};
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
    /// List of intersecting triangle pairs as (face_idx_a, face_idx_b).
    /// Limited to first `max_reported` pairs.
    pub intersecting_pairs: Vec<(u32, u32)>,
    /// Total faces checked.
    pub faces_checked: usize,
    /// Whether the search was terminated early due to reaching max_reported.
    pub truncated: bool,
}

impl SelfIntersectionResult {
    /// Check if the mesh is free of self-intersections.
    #[must_use]
    pub fn is_clean(&self) -> bool {
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
    pub fn exhaustive() -> Self {
        Self {
            max_reported: 0,
            epsilon: 1e-10,
            skip_adjacent: true,
        }
    }

    /// Create params that only check for the presence of intersections.
    #[must_use]
    pub fn quick_check() -> Self {
        Self {
            max_reported: 1,
            epsilon: 1e-10,
            skip_adjacent: true,
        }
    }
}

/// Axis-aligned bounding box for spatial acceleration.
#[derive(Debug, Clone, Copy)]
struct Aabb {
    min: Point3<f64>,
    max: Point3<f64>,
}

impl Aabb {
    /// Create AABB from a triangle.
    fn from_triangle(tri: &Triangle) -> Self {
        let min = Point3::new(
            tri.v0.x.min(tri.v1.x).min(tri.v2.x),
            tri.v0.y.min(tri.v1.y).min(tri.v2.y),
            tri.v0.z.min(tri.v1.z).min(tri.v2.z),
        );
        let max = Point3::new(
            tri.v0.x.max(tri.v1.x).max(tri.v2.x),
            tri.v0.y.max(tri.v1.y).max(tri.v2.y),
            tri.v0.z.max(tri.v1.z).max(tri.v2.z),
        );
        Self { min, max }
    }

    /// Expand AABB by epsilon for numerical robustness.
    fn expand(&self, epsilon: f64) -> Self {
        Self {
            min: Point3::new(
                self.min.x - epsilon,
                self.min.y - epsilon,
                self.min.z - epsilon,
            ),
            max: Point3::new(
                self.max.x + epsilon,
                self.max.y + epsilon,
                self.max.z + epsilon,
            ),
        }
    }

    /// Check if two AABBs overlap.
    fn overlaps(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

/// Detect self-intersections in a mesh.
///
/// Uses bounding box culling to avoid O(n²) triangle-triangle tests where possible.
/// Uses parallel processing via rayon for performance on large meshes.
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
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::intersect::{detect_self_intersections, IntersectionParams};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
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

    // Precompute triangles and AABBs
    let triangles: Vec<Triangle> = mesh.triangles().collect();
    let aabbs: Vec<Aabb> = triangles
        .iter()
        .map(|t| Aabb::from_triangle(t).expand(params.epsilon))
        .collect();

    // Build adjacency info if we need to skip adjacent triangles
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

    // Shared atomics for counting and early termination
    let intersection_count = AtomicUsize::new(0);
    let should_stop = AtomicBool::new(false);

    // Process triangle pairs in parallel
    // We parallelize the outer loop and collect intersecting pairs from each chunk
    let intersecting_pairs: Vec<(u32, u32)> = (0..face_count)
        .into_par_iter()
        .flat_map(|i| {
            // Early termination check
            if should_stop.load(Ordering::Relaxed) {
                return Vec::new();
            }

            let mut local_pairs = Vec::new();

            for j in (i + 1)..face_count {
                // Early termination check inside inner loop
                if should_stop.load(Ordering::Relaxed) {
                    break;
                }

                // Skip if AABBs don't overlap
                if !aabbs[i].overlaps(&aabbs[j]) {
                    continue;
                }

                // Skip adjacent triangles if requested
                if let Some(ref adj) = adjacency
                    && adj[i].contains(&(j as u32))
                {
                    continue;
                }

                // Perform actual triangle-triangle intersection test
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
    use mesh_types::Vertex;

    fn create_xy_triangle(x: f64, y: f64, size: f64) -> Triangle {
        Triangle::new(
            Point3::new(x, y, 0.0),
            Point3::new(x + size, y, 0.0),
            Point3::new(x + size / 2.0, y + size, 0.0),
        )
    }

    #[test]
    fn test_aabb_overlap() {
        let aabb1 = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let aabb2 = Aabb {
            min: Point3::new(0.5, 0.5, 0.5),
            max: Point3::new(1.5, 1.5, 1.5),
        };
        let aabb3 = Aabb {
            min: Point3::new(2.0, 2.0, 2.0),
            max: Point3::new(3.0, 3.0, 3.0),
        };

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
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));
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
        mesh.vertices.push(Vertex::from_coords(-1.0, -1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, -1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));

        // Triangle 2 in XZ plane, passing through triangle 1
        mesh.vertices.push(Vertex::from_coords(-1.0, 0.0, -1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, -1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));

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
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));
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
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
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
        clean_mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        clean_mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        clean_mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        clean_mesh.faces.push([0, 1, 2]);

        assert!(!has_self_intersections(&clean_mesh));

        // Intersecting mesh
        let mut bad_mesh = IndexedMesh::new();
        bad_mesh.vertices.push(Vertex::from_coords(-1.0, -1.0, 0.0));
        bad_mesh.vertices.push(Vertex::from_coords(1.0, -1.0, 0.0));
        bad_mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        bad_mesh.vertices.push(Vertex::from_coords(-1.0, 0.0, -1.0));
        bad_mesh.vertices.push(Vertex::from_coords(1.0, 0.0, -1.0));
        bad_mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        bad_mesh.faces.push([0, 1, 2]);
        bad_mesh.faces.push([3, 4, 5]);

        assert!(has_self_intersections(&bad_mesh));
    }
}
