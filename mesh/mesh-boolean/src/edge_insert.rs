//! Edge insertion for clean boolean operation boundaries.
//!
//! This module handles splitting triangles along intersection curves,
//! creating clean boundary edges at the intersection of two meshes.
//!
//! Without edge insertion, boolean operations simply classify and keep/discard
//! whole triangles, leading to jagged stair-step boundaries. With edge insertion,
//! triangles that cross the intersection are split along the exact intersection
//! curve, producing clean edge loops.

#![allow(clippy::too_many_arguments)]

use crate::bvh::{Aabb, Bvh};
use crate::intersect::compute_triangle_intersection;
use hashbrown::HashMap;
use mesh_types::{IndexedMesh, Point3, Vertex};
use smallvec::SmallVec;

/// Information about where a triangle is split.
#[derive(Debug, Clone)]
pub struct TriangleSplit {
    /// Original triangle index.
    pub original_face: u32,
    /// New triangles created from the split.
    pub new_faces: SmallVec<[[u32; 3]; 4]>,
    /// Intersection points on this triangle.
    pub intersection_points: SmallVec<[Point3<f64>; 4]>,
}

/// Result of edge insertion for a mesh.
#[derive(Debug)]
pub struct EdgeInsertionResult {
    /// The mesh with inserted edges (triangles split along intersections).
    pub mesh: IndexedMesh,
    /// Mapping from original face indices to new face indices.
    pub face_mapping: HashMap<u32, SmallVec<[u32; 4]>>,
    /// Number of triangles that were split.
    pub split_count: usize,
    /// Number of new vertices created.
    pub new_vertex_count: usize,
}

/// Find all intersection points between mesh A triangles and mesh B.
///
/// For each triangle in mesh A that intersects mesh B, collects
/// all the intersection points that lie on the triangle.
#[must_use]
pub fn find_intersection_points(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    bvh_b: &Bvh,
    epsilon: f64,
) -> HashMap<u32, SmallVec<[Point3<f64>; 8]>> {
    let mut intersections: HashMap<u32, SmallVec<[Point3<f64>; 8]>> = HashMap::new();

    for (ai, face_a) in mesh_a.faces.iter().enumerate() {
        let a0 = mesh_a.vertices[face_a[0] as usize].position;
        let a1 = mesh_a.vertices[face_a[1] as usize].position;
        let a2 = mesh_a.vertices[face_a[2] as usize].position;

        // Query BVH for potential intersections
        let bbox_a = Aabb::from_triangle(&a0, &a1, &a2);
        let candidates = bvh_b.query(&bbox_a, epsilon);

        let mut points: SmallVec<[Point3<f64>; 8]> = SmallVec::new();

        for bi in candidates {
            let face_b = &mesh_b.faces[bi as usize];
            let b0 = mesh_b.vertices[face_b[0] as usize].position;
            let b1 = mesh_b.vertices[face_b[1] as usize].position;
            let b2 = mesh_b.vertices[face_b[2] as usize].position;

            // Get intersection segment
            if let Some(intersection) =
                compute_triangle_intersection(&a0, &a1, &a2, &b0, &b1, &b2, epsilon)
            {
                // Add both endpoints if they're not duplicates
                add_unique_point(&mut points, intersection.start, epsilon);
                add_unique_point(&mut points, intersection.end, epsilon);
            }
        }

        if !points.is_empty() {
            intersections.insert(ai as u32, points);
        }
    }

    intersections
}

/// Add a point to the list if it's not already present (within epsilon).
fn add_unique_point(points: &mut SmallVec<[Point3<f64>; 8]>, point: Point3<f64>, epsilon: f64) {
    let epsilon_sq = epsilon * epsilon;
    let is_duplicate = points
        .iter()
        .any(|p| (point - p).norm_squared() < epsilon_sq);

    if !is_duplicate {
        points.push(point);
    }
}

/// Split a triangle by inserting new vertices at intersection points.
///
/// This is a simplified splitting approach that works for the common case
/// of 2 intersection points (a segment crossing the triangle).
///
/// # Arguments
///
/// * `v0`, `v1`, `v2` - Original triangle vertices (indices into vertex array)
/// * `vertices` - Vertex array to add new vertices to
/// * `intersection_points` - Points where the triangle is intersected
/// * `epsilon` - Tolerance for point comparisons
///
/// # Returns
///
/// Vector of new triangle face indices.
#[must_use]
pub fn split_triangle(
    v0: u32,
    v1: u32,
    v2: u32,
    vertices: &mut Vec<Vertex>,
    intersection_points: &[Point3<f64>],
    epsilon: f64,
) -> SmallVec<[[u32; 3]; 4]> {
    if intersection_points.is_empty() {
        // No intersection - return original triangle
        return smallvec::smallvec![[v0, v1, v2]];
    }

    if intersection_points.len() == 1 {
        // Single point - triangle touches at a point, no real split needed
        return smallvec::smallvec![[v0, v1, v2]];
    }

    // Get original vertex positions
    let p0 = vertices[v0 as usize].position;
    let p1 = vertices[v1 as usize].position;
    let p2 = vertices[v2 as usize].position;

    // For 2 intersection points (the common case), we split the triangle
    // into 3-4 triangles depending on which edges the points lie on
    if intersection_points.len() == 2 {
        let int0 = intersection_points[0];
        let int1 = intersection_points[1];

        // Find which edges each intersection point lies on
        let edge0_param0 = point_on_edge_param(&int0, &p0, &p1, epsilon);
        let edge1_param0 = point_on_edge_param(&int0, &p1, &p2, epsilon);
        let edge2_param0 = point_on_edge_param(&int0, &p2, &p0, epsilon);

        let edge0_param1 = point_on_edge_param(&int1, &p0, &p1, epsilon);
        let edge1_param1 = point_on_edge_param(&int1, &p1, &p2, epsilon);
        let edge2_param1 = point_on_edge_param(&int1, &p2, &p0, epsilon);

        // Determine edge locations
        let int0_edge = find_edge(edge0_param0, edge1_param0, edge2_param0);
        let int1_edge = find_edge(edge0_param1, edge1_param1, edge2_param1);

        if let (Some(e0), Some(e1)) = (int0_edge, int1_edge) {
            if e0 != e1 {
                // Points on different edges - standard case
                return split_triangle_two_points(
                    v0, v1, v2, vertices, &int0, &int1, e0, e1, epsilon,
                );
            }
        }
    }

    // Fallback: for complex cases, use fan triangulation from centroid
    split_triangle_complex(v0, v1, v2, vertices, intersection_points, epsilon)
}

/// Find which edge a point lies on (0, 1, or 2).
fn find_edge(param0: Option<f64>, param1: Option<f64>, param2: Option<f64>) -> Option<u8> {
    if param0.is_some() {
        Some(0)
    } else if param1.is_some() {
        Some(1)
    } else if param2.is_some() {
        Some(2)
    } else {
        None
    }
}

/// Compute parameter t if point lies on edge (e0 to e1).
/// Returns Some(t) where t in [0,1] if point is on edge, None otherwise.
fn point_on_edge_param(
    point: &Point3<f64>,
    e0: &Point3<f64>,
    e1: &Point3<f64>,
    epsilon: f64,
) -> Option<f64> {
    let edge = e1 - e0;
    let edge_len_sq = edge.norm_squared();

    if edge_len_sq < epsilon * epsilon {
        return None; // Degenerate edge
    }

    let t = (point - e0).dot(&edge) / edge_len_sq;

    if t < -epsilon || t > 1.0 + epsilon {
        return None; // Point not on edge segment
    }

    // Check if point is actually close to the edge
    let projected = Point3::from(e0.coords + edge * t);
    let dist_sq = (point - projected).norm_squared();

    if dist_sq < epsilon * epsilon {
        Some(t.clamp(0.0, 1.0))
    } else {
        None
    }
}

/// Split a triangle when two intersection points are on different edges.
fn split_triangle_two_points(
    v0: u32,
    v1: u32,
    v2: u32,
    vertices: &mut Vec<Vertex>,
    int0: &Point3<f64>,
    int1: &Point3<f64>,
    edge0: u8,
    edge1: u8,
    _epsilon: f64,
) -> SmallVec<[[u32; 3]; 4]> {
    // Add new vertices for intersection points
    let i0 = vertices.len() as u32;
    vertices.push(Vertex::new(*int0));
    let i1 = vertices.len() as u32;
    vertices.push(Vertex::new(*int1));

    // Create triangles based on which edges are cut
    // Edge 0: v0-v1, Edge 1: v1-v2, Edge 2: v2-v0
    let mut result: SmallVec<[[u32; 3]; 4]> = SmallVec::new();

    match (edge0, edge1) {
        (0, 1) => {
            // Cut edges v0-v1 and v1-v2
            result.push([v0, i0, i1]);
            result.push([v0, i1, v2]);
            result.push([i0, v1, i1]);
        }
        (1, 0) => {
            // Cut edges v1-v2 and v0-v1
            result.push([v0, i1, i0]);
            result.push([v0, i0, v2]);
            result.push([i1, v1, i0]);
        }
        (0, 2) => {
            // Cut edges v0-v1 and v2-v0
            result.push([v0, i0, i1]);
            result.push([i0, v1, v2]);
            result.push([i0, v2, i1]);
        }
        (2, 0) => {
            // Cut edges v2-v0 and v0-v1
            result.push([v0, i1, i0]);
            result.push([i1, v1, v2]);
            result.push([i1, v2, i0]);
        }
        (1, 2) => {
            // Cut edges v1-v2 and v2-v0
            result.push([v0, v1, i0]);
            result.push([v0, i0, i1]);
            result.push([i0, v2, i1]);
        }
        (2, 1) => {
            // Cut edges v2-v0 and v1-v2
            result.push([v0, v1, i1]);
            result.push([v0, i1, i0]);
            result.push([i1, v2, i0]);
        }
        _ => {
            // Same edge or unexpected case - return original
            result.push([v0, v1, v2]);
        }
    }

    result
}

/// Split a triangle with complex intersection pattern using fan triangulation.
fn split_triangle_complex(
    v0: u32,
    v1: u32,
    v2: u32,
    vertices: &mut Vec<Vertex>,
    intersection_points: &[Point3<f64>],
    epsilon: f64,
) -> SmallVec<[[u32; 3]; 4]> {
    // Get original positions
    let p0 = vertices[v0 as usize].position;
    let p1 = vertices[v1 as usize].position;
    let p2 = vertices[v2 as usize].position;

    // Compute centroid of all points (original vertices + intersections)
    let mut centroid = p0.coords + p1.coords + p2.coords;
    for ip in intersection_points {
        centroid += ip.coords;
    }
    let total_points = 3 + intersection_points.len();
    centroid /= total_points as f64;
    let centroid = Point3::from(centroid);

    // Add centroid as new vertex
    let center_idx = vertices.len() as u32;
    vertices.push(Vertex::new(centroid));

    // Collect all boundary points (original + intersection) and sort by angle
    let mut boundary: Vec<(u32, f64)> = Vec::with_capacity(total_points);

    // Add original vertices
    boundary.push((v0, angle_around_center(&p0, &centroid)));
    boundary.push((v1, angle_around_center(&p1, &centroid)));
    boundary.push((v2, angle_around_center(&p2, &centroid)));

    // Add intersection points
    for ip in intersection_points {
        let idx = vertices.len() as u32;
        vertices.push(Vertex::new(*ip));
        boundary.push((idx, angle_around_center(ip, &centroid)));
    }

    // Sort by angle
    boundary.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    // Remove duplicate points (within epsilon angle)
    let mut deduped: Vec<u32> = Vec::with_capacity(boundary.len());
    for (idx, _angle) in &boundary {
        if deduped.is_empty() {
            deduped.push(*idx);
        } else {
            let last_idx = *deduped.last().unwrap_or(&0);
            let last_pos = vertices[last_idx as usize].position;
            let this_pos = vertices[*idx as usize].position;
            if (this_pos - last_pos).norm_squared() > epsilon * epsilon {
                deduped.push(*idx);
            }
        }
    }

    // Create fan triangles from centroid
    let mut result: SmallVec<[[u32; 3]; 4]> = SmallVec::new();
    let n = deduped.len();
    for i in 0..n {
        let next = (i + 1) % n;
        result.push([center_idx, deduped[i], deduped[next]]);
    }

    result
}

/// Compute angle of point around center (for sorting boundary points).
fn angle_around_center(point: &Point3<f64>, center: &Point3<f64>) -> f64 {
    let d = point - center;
    // Project to dominant plane (XY for simplicity)
    d.y.atan2(d.x)
}

/// Perform edge insertion on a mesh at its intersections with another mesh.
///
/// # Arguments
///
/// * `mesh` - The mesh to process
/// * `other` - The mesh to intersect against
/// * `other_bvh` - Pre-built BVH for the other mesh
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `EdgeInsertionResult` containing the mesh with split triangles.
#[must_use]
pub fn insert_edges(
    mesh: &IndexedMesh,
    other: &IndexedMesh,
    other_bvh: &Bvh,
    epsilon: f64,
) -> EdgeInsertionResult {
    // Find all intersection points per triangle
    let intersections = find_intersection_points(mesh, other, other_bvh, epsilon);

    if intersections.is_empty() {
        // No intersections - return copy of original
        return EdgeInsertionResult {
            mesh: mesh.clone(),
            face_mapping: HashMap::new(),
            split_count: 0,
            new_vertex_count: 0,
        };
    }

    // Create result mesh with copy of original vertices
    let mut result = IndexedMesh::new();
    result.vertices.clone_from(&mesh.vertices);
    let original_vertex_count = result.vertices.len();

    let mut face_mapping: HashMap<u32, SmallVec<[u32; 4]>> = HashMap::new();
    let mut split_count = 0;

    // Process each face
    for (fi, face) in mesh.faces.iter().enumerate() {
        let fi_u32 = fi as u32;

        if let Some(int_points) = intersections.get(&fi_u32) {
            // This face needs to be split
            let new_faces = split_triangle(
                face[0],
                face[1],
                face[2],
                &mut result.vertices,
                int_points,
                epsilon,
            );

            let mut indices: SmallVec<[u32; 4]> = SmallVec::new();

            for new_face in new_faces {
                indices.push(result.faces.len() as u32);
                result.faces.push(new_face);
            }

            if indices.len() > 1 {
                split_count += 1;
            }

            face_mapping.insert(fi_u32, indices);
        } else {
            // No intersection - keep original face
            let idx = result.faces.len() as u32;
            result.faces.push(*face);
            face_mapping.insert(fi_u32, smallvec::smallvec![idx]);
        }
    }

    let new_vertex_count = result.vertices.len() - original_vertex_count;

    EdgeInsertionResult {
        mesh: result,
        face_mapping,
        split_count,
        new_vertex_count,
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;

    fn create_triangle_mesh(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(v0));
        mesh.vertices.push(Vertex::new(v1));
        mesh.vertices.push(Vertex::new(v2));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn test_point_on_edge_param() {
        let e0 = Point3::new(0.0, 0.0, 0.0);
        let e1 = Point3::new(1.0, 0.0, 0.0);

        // Point at start
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let t0 = point_on_edge_param(&p0, &e0, &e1, 1e-10);
        assert!(t0.is_some());
        assert!((t0.unwrap() - 0.0).abs() < 1e-6);

        // Point in middle
        let p1 = Point3::new(0.5, 0.0, 0.0);
        let t1 = point_on_edge_param(&p1, &e0, &e1, 1e-10);
        assert!(t1.is_some());
        assert!((t1.unwrap() - 0.5).abs() < 1e-6);

        // Point at end
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let t2 = point_on_edge_param(&p2, &e0, &e1, 1e-10);
        assert!(t2.is_some());
        assert!((t2.unwrap() - 1.0).abs() < 1e-6);

        // Point not on edge
        let p3 = Point3::new(0.5, 1.0, 0.0);
        let t3 = point_on_edge_param(&p3, &e0, &e1, 1e-10);
        assert!(t3.is_none());
    }

    #[test]
    fn test_split_triangle_no_intersection() {
        let mut vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0)),
        ];

        let result = split_triangle(0, 1, 2, &mut vertices, &[], 1e-10);

        assert_eq!(result.len(), 1);
        assert_eq!(result[0], [0, 1, 2]);
    }

    #[test]
    fn test_split_triangle_single_point() {
        let mut vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0)),
        ];

        let int_points = [Point3::new(0.5, 0.5, 0.0)];
        let result = split_triangle(0, 1, 2, &mut vertices, &int_points, 1e-10);

        // Single point doesn't split
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_split_triangle_two_points() {
        let mut vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0)),
        ];

        // Two points on different edges
        let int_points = [
            Point3::new(0.5, 0.0, 0.0),  // On edge v0-v1
            Point3::new(0.75, 0.5, 0.0), // On edge v1-v2
        ];

        let result = split_triangle(0, 1, 2, &mut vertices, &int_points, 1e-10);

        // Should create 3 triangles
        assert_eq!(result.len(), 3);

        // Should have added 2 new vertices
        assert_eq!(vertices.len(), 5);
    }

    #[test]
    fn test_add_unique_point() {
        let mut points: SmallVec<[Point3<f64>; 8]> = SmallVec::new();

        add_unique_point(&mut points, Point3::new(1.0, 0.0, 0.0), 1e-10);
        assert_eq!(points.len(), 1);

        // Same point - should not add
        add_unique_point(&mut points, Point3::new(1.0, 0.0, 0.0), 1e-10);
        assert_eq!(points.len(), 1);

        // Different point - should add
        add_unique_point(&mut points, Point3::new(2.0, 0.0, 0.0), 1e-10);
        assert_eq!(points.len(), 2);
    }

    #[test]
    fn test_insert_edges_no_intersection() {
        // Two non-intersecting triangles
        let mesh_a = create_triangle_mesh(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        );

        let mesh_b = create_triangle_mesh(
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(1.0, 0.0, 5.0),
            Point3::new(0.5, 1.0, 5.0),
        );

        let bvh_b = Bvh::build(&mesh_b, 4);
        let result = insert_edges(&mesh_a, &mesh_b, &bvh_b, 1e-10);

        assert_eq!(result.split_count, 0);
        assert_eq!(result.new_vertex_count, 0);
        assert_eq!(result.mesh.faces.len(), 1);
    }

    #[test]
    fn test_angle_around_center() {
        let center = Point3::new(0.0, 0.0, 0.0);

        let p0 = Point3::new(1.0, 0.0, 0.0);
        let p90 = Point3::new(0.0, 1.0, 0.0);
        let p180 = Point3::new(-1.0, 0.0, 0.0);

        let a0 = angle_around_center(&p0, &center);
        let a90 = angle_around_center(&p90, &center);
        let a180 = angle_around_center(&p180, &center);

        assert!(a0.abs() < 0.01); // ~0 radians
        assert!((a90 - std::f64::consts::FRAC_PI_2).abs() < 0.01); // ~π/2
        assert!((a180.abs() - std::f64::consts::PI).abs() < 0.01); // ~π or -π
    }
}
