//! Face classification for boolean operations.
//!
//! This module determines whether faces of one mesh are inside or outside
//! another mesh, using ray casting for robust point-in-mesh tests.

#![allow(clippy::option_if_let_else)]
#![allow(clippy::manual_let_else)]

use crate::bvh::{Aabb, Bvh};
use crate::intersect::ray_triangle_intersect;
use mesh_types::{IndexedMesh, Point3, Vector3};
use rayon::prelude::*;

/// Classification of a face relative to another mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceLocation {
    /// Face is inside the other mesh.
    Inside,
    /// Face is outside the other mesh.
    Outside,
    /// Face is on the boundary (coplanar with other mesh faces).
    OnBoundary,
}

/// Result of point-in-mesh test with additional information.
#[derive(Debug, Clone)]
pub struct PointInMeshResult {
    /// Whether the point is inside the mesh.
    pub inside: bool,
    /// Number of ray intersections counted.
    pub intersection_count: usize,
}

/// Test if a point is inside a mesh using ray casting.
///
/// Casts a ray in the +X direction and counts intersections.
/// An odd count means inside, even means outside.
///
/// # Arguments
///
/// * `point` - The point to test
/// * `mesh` - The mesh to test against
/// * `bvh` - Pre-built BVH for the mesh (optional, for acceleration)
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `PointInMeshResult` containing whether the point is inside and intersection count.
#[must_use]
pub fn point_in_mesh(
    point: &Point3<f64>,
    mesh: &IndexedMesh,
    bvh: Option<&Bvh>,
    epsilon: f64,
) -> PointInMeshResult {
    // Use +X direction for ray casting (could be randomized for robustness)
    let ray_dir = Vector3::new(1.0, 0.0, 0.0);

    let intersection_count = if let Some(bvh) = bvh {
        count_ray_intersections_bvh(point, &ray_dir, mesh, bvh, epsilon)
    } else {
        count_ray_intersections_naive(point, &ray_dir, mesh, epsilon)
    };

    PointInMeshResult {
        inside: intersection_count % 2 == 1,
        intersection_count,
    }
}

/// Test if a point is inside a mesh using multiple ray directions for robustness.
///
/// Uses three ray directions (+X, +Y, +Z) and takes the majority vote
/// to handle edge cases where a single ray might graze an edge.
///
/// # Arguments
///
/// * `point` - The point to test
/// * `mesh` - The mesh to test against
/// * `bvh` - Pre-built BVH for the mesh (optional)
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `true` if point is inside the mesh (by majority vote).
#[must_use]
pub fn point_in_mesh_robust(
    point: &Point3<f64>,
    mesh: &IndexedMesh,
    bvh: Option<&Bvh>,
    epsilon: f64,
) -> bool {
    let directions = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];

    let mut inside_count = 0;

    for dir in &directions {
        let count = if let Some(bvh) = bvh {
            count_ray_intersections_bvh(point, dir, mesh, bvh, epsilon)
        } else {
            count_ray_intersections_naive(point, dir, mesh, epsilon)
        };

        if count % 2 == 1 {
            inside_count += 1;
        }
    }

    // Majority vote: need 2 out of 3 to be "inside"
    inside_count >= 2
}

/// Count ray intersections without BVH (naive O(n) approach).
fn count_ray_intersections_naive(
    origin: &Point3<f64>,
    direction: &Vector3<f64>,
    mesh: &IndexedMesh,
    epsilon: f64,
) -> usize {
    let mut count = 0;

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0] as usize].position;
        let v1 = mesh.vertices[face[1] as usize].position;
        let v2 = mesh.vertices[face[2] as usize].position;

        if ray_triangle_intersect(origin, direction, &v0, &v1, &v2, epsilon).is_some() {
            count += 1;
        }
    }

    count
}

/// Count ray intersections using BVH acceleration.
fn count_ray_intersections_bvh(
    origin: &Point3<f64>,
    direction: &Vector3<f64>,
    mesh: &IndexedMesh,
    bvh: &Bvh,
    epsilon: f64,
) -> usize {
    // Create a ray bounding box extending to infinity in the ray direction
    // We approximate this with a very large extent
    let ray_extent = 1e10;

    let ray_end = Point3::from(origin.coords + direction * ray_extent);

    let ray_bbox = Aabb::from_min_max(
        Point3::new(
            origin.x.min(ray_end.x),
            origin.y.min(ray_end.y),
            origin.z.min(ray_end.z),
        ),
        Point3::new(
            origin.x.max(ray_end.x),
            origin.y.max(ray_end.y),
            origin.z.max(ray_end.z),
        ),
    );

    // Query BVH for candidate triangles
    let candidates = bvh.query(&ray_bbox, epsilon);

    let mut count = 0;

    for tri_idx in candidates {
        let face = &mesh.faces[tri_idx as usize];
        let v0 = mesh.vertices[face[0] as usize].position;
        let v1 = mesh.vertices[face[1] as usize].position;
        let v2 = mesh.vertices[face[2] as usize].position;

        if ray_triangle_intersect(origin, direction, &v0, &v1, &v2, epsilon).is_some() {
            count += 1;
        }
    }

    count
}

/// Classify all faces of a mesh relative to another mesh.
///
/// For each face, determines whether its centroid is inside, outside,
/// or on the boundary of the other mesh.
///
/// # Arguments
///
/// * `mesh` - The mesh whose faces to classify
/// * `other` - The mesh to classify against
/// * `other_bvh` - Pre-built BVH for the other mesh
/// * `epsilon` - Tolerance for classification
///
/// # Returns
///
/// Vector of `FaceLocation` values, one per face in `mesh`.
#[must_use]
pub fn classify_faces(
    mesh: &IndexedMesh,
    other: &IndexedMesh,
    other_bvh: &Bvh,
    epsilon: f64,
) -> Vec<FaceLocation> {
    mesh.faces
        .iter()
        .map(|face| {
            let v0 = mesh.vertices[face[0] as usize].position;
            let v1 = mesh.vertices[face[1] as usize].position;
            let v2 = mesh.vertices[face[2] as usize].position;

            // Use face centroid for classification
            let centroid = Point3::from((v0.coords + v1.coords + v2.coords) / 3.0);

            if point_in_mesh_robust(&centroid, other, Some(other_bvh), epsilon) {
                FaceLocation::Inside
            } else {
                FaceLocation::Outside
            }
        })
        .collect()
}

/// Classify all faces of a mesh relative to another mesh, in parallel.
///
/// Uses rayon for parallel processing on large meshes.
///
/// # Arguments
///
/// * `mesh` - The mesh whose faces to classify
/// * `other` - The mesh to classify against
/// * `other_bvh` - Pre-built BVH for the other mesh
/// * `epsilon` - Tolerance for classification
///
/// # Returns
///
/// Vector of `FaceLocation` values, one per face in `mesh`.
#[must_use]
pub fn classify_faces_parallel(
    mesh: &IndexedMesh,
    other: &IndexedMesh,
    other_bvh: &Bvh,
    epsilon: f64,
) -> Vec<FaceLocation> {
    mesh.faces
        .par_iter()
        .map(|face| {
            let v0 = mesh.vertices[face[0] as usize].position;
            let v1 = mesh.vertices[face[1] as usize].position;
            let v2 = mesh.vertices[face[2] as usize].position;

            // Use face centroid for classification
            let centroid = Point3::from((v0.coords + v1.coords + v2.coords) / 3.0);

            if point_in_mesh_robust(&centroid, other, Some(other_bvh), epsilon) {
                FaceLocation::Inside
            } else {
                FaceLocation::Outside
            }
        })
        .collect()
}

/// Classify faces with an option to use parallel processing.
///
/// Automatically chooses parallel processing for meshes with more
/// than `parallel_threshold` faces.
///
/// # Arguments
///
/// * `mesh` - The mesh whose faces to classify
/// * `other` - The mesh to classify against
/// * `other_bvh` - Pre-built BVH for the other mesh
/// * `epsilon` - Tolerance for classification
/// * `parallel` - Whether to use parallel processing
///
/// # Returns
///
/// Vector of `FaceLocation` values, one per face in `mesh`.
#[must_use]
pub fn classify_faces_auto(
    mesh: &IndexedMesh,
    other: &IndexedMesh,
    other_bvh: &Bvh,
    epsilon: f64,
    parallel: bool,
) -> Vec<FaceLocation> {
    if parallel && mesh.faces.len() > 100 {
        classify_faces_parallel(mesh, other, other_bvh, epsilon)
    } else {
        classify_faces(mesh, other, other_bvh, epsilon)
    }
}

/// Compute the bounding box of a mesh.
///
/// # Arguments
///
/// * `mesh` - The mesh to compute bounds for
///
/// # Returns
///
/// `Some((min, max))` if mesh has vertices, `None` if empty.
#[must_use]
pub fn compute_mesh_bounds(mesh: &IndexedMesh) -> Option<(Point3<f64>, Point3<f64>)> {
    if mesh.vertices.is_empty() {
        return None;
    }

    let first = mesh.vertices[0].position;
    let mut min = first;
    let mut max = first;

    for v in &mesh.vertices {
        min.x = min.x.min(v.position.x);
        min.y = min.y.min(v.position.y);
        min.z = min.z.min(v.position.z);
        max.x = max.x.max(v.position.x);
        max.y = max.y.max(v.position.y);
        max.z = max.z.max(v.position.z);
    }

    Some((min, max))
}

/// Check if two mesh bounding boxes overlap.
///
/// # Arguments
///
/// * `mesh_a` - First mesh
/// * `mesh_b` - Second mesh
///
/// # Returns
///
/// `true` if the bounding boxes overlap.
#[must_use]
pub fn meshes_overlap(mesh_a: &IndexedMesh, mesh_b: &IndexedMesh) -> bool {
    let bounds_a = match compute_mesh_bounds(mesh_a) {
        Some(b) => b,
        None => return false,
    };

    let bounds_b = match compute_mesh_bounds(mesh_b) {
        Some(b) => b,
        None => return false,
    };

    let (a_min, a_max) = bounds_a;
    let (b_min, b_max) = bounds_b;

    !(a_max.x < b_min.x
        || b_max.x < a_min.x
        || a_max.y < b_min.y
        || b_max.y < a_min.y
        || a_max.z < b_min.z
        || b_max.z < a_min.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of unit cube
        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        // 12 triangles (2 per face)
        // Bottom (z=0)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Top (z=1)
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        // Front (y=0)
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // Back (y=1)
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // Left (x=0)
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        // Right (x=1)
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_point_in_mesh_inside() {
        let cube = create_unit_cube();
        // Use a point slightly off-center to avoid edge cases with shared edges
        let point = Point3::new(0.51, 0.52, 0.53);

        let result = point_in_mesh(&point, &cube, None, 1e-10);
        assert!(result.inside);
    }

    #[test]
    fn test_point_in_mesh_outside() {
        let cube = create_unit_cube();
        let point = Point3::new(2.0, 2.0, 2.0);

        let result = point_in_mesh(&point, &cube, None, 1e-10);
        assert!(!result.inside);
    }

    #[test]
    fn test_point_in_mesh_with_bvh() {
        let cube = create_unit_cube();
        let bvh = Bvh::build(&cube, 4);

        // Use points slightly off-center to avoid edge cases
        let inside = Point3::new(0.51, 0.52, 0.53);
        let outside = Point3::new(2.0, 2.0, 2.0);

        let result_inside = point_in_mesh(&inside, &cube, Some(&bvh), 1e-10);
        let result_outside = point_in_mesh(&outside, &cube, Some(&bvh), 1e-10);

        assert!(result_inside.inside);
        assert!(!result_outside.inside);
    }

    #[test]
    fn test_point_in_mesh_robust() {
        let cube = create_unit_cube();
        let bvh = Bvh::build(&cube, 4);

        // Use points slightly off-center to avoid degenerate cases where
        // rays hit shared edges exactly
        let inside = Point3::new(0.51, 0.52, 0.53);
        let outside = Point3::new(2.0, 2.0, 2.0);

        assert!(point_in_mesh_robust(&inside, &cube, Some(&bvh), 1e-10));
        assert!(!point_in_mesh_robust(&outside, &cube, Some(&bvh), 1e-10));
    }

    #[test]
    fn test_classify_faces() {
        // Create two cubes, one inside the other
        let outer_cube = create_unit_cube();
        let mut inner_cube = IndexedMesh::new();

        // Smaller cube (0.25 to 0.75)
        let vertices = [
            Point3::new(0.25, 0.25, 0.25),
            Point3::new(0.75, 0.25, 0.25),
            Point3::new(0.75, 0.75, 0.25),
            Point3::new(0.25, 0.75, 0.25),
            Point3::new(0.25, 0.25, 0.75),
            Point3::new(0.75, 0.25, 0.75),
            Point3::new(0.75, 0.75, 0.75),
            Point3::new(0.25, 0.75, 0.75),
        ];

        for v in &vertices {
            inner_cube.vertices.push(Vertex::new(*v));
        }

        // Same face structure as outer cube
        inner_cube.faces.push([0, 2, 1]);
        inner_cube.faces.push([0, 3, 2]);
        inner_cube.faces.push([4, 5, 6]);
        inner_cube.faces.push([4, 6, 7]);
        inner_cube.faces.push([0, 1, 5]);
        inner_cube.faces.push([0, 5, 4]);
        inner_cube.faces.push([2, 3, 7]);
        inner_cube.faces.push([2, 7, 6]);
        inner_cube.faces.push([0, 4, 7]);
        inner_cube.faces.push([0, 7, 3]);
        inner_cube.faces.push([1, 2, 6]);
        inner_cube.faces.push([1, 6, 5]);

        let bvh = Bvh::build(&outer_cube, 4);
        let classifications = classify_faces(&inner_cube, &outer_cube, &bvh, 1e-10);

        // All faces of inner cube should be inside outer cube
        for loc in &classifications {
            assert_eq!(*loc, FaceLocation::Inside);
        }
    }

    #[test]
    fn test_compute_mesh_bounds() {
        let cube = create_unit_cube();
        let bounds = compute_mesh_bounds(&cube);

        assert!(bounds.is_some());
        let (min, max) = bounds.unwrap();

        assert!((min.x - 0.0).abs() < 1e-10);
        assert!((min.y - 0.0).abs() < 1e-10);
        assert!((min.z - 0.0).abs() < 1e-10);
        assert!((max.x - 1.0).abs() < 1e-10);
        assert!((max.y - 1.0).abs() < 1e-10);
        assert!((max.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_compute_mesh_bounds_empty() {
        let mesh = IndexedMesh::new();
        let bounds = compute_mesh_bounds(&mesh);
        assert!(bounds.is_none());
    }

    #[test]
    fn test_meshes_overlap_yes() {
        let cube1 = create_unit_cube();

        let mut cube2 = IndexedMesh::new();
        // Cube from (0.5, 0.5, 0.5) to (1.5, 1.5, 1.5) - overlaps with cube1
        let vertices = [
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(1.5, 0.5, 0.5),
            Point3::new(1.5, 1.5, 0.5),
            Point3::new(0.5, 1.5, 0.5),
            Point3::new(0.5, 0.5, 1.5),
            Point3::new(1.5, 0.5, 1.5),
            Point3::new(1.5, 1.5, 1.5),
            Point3::new(0.5, 1.5, 1.5),
        ];

        for v in &vertices {
            cube2.vertices.push(Vertex::new(*v));
        }

        cube2.faces.push([0, 1, 2]);

        assert!(meshes_overlap(&cube1, &cube2));
    }

    #[test]
    fn test_meshes_overlap_no() {
        let cube1 = create_unit_cube();

        let mut cube2 = IndexedMesh::new();
        // Cube from (5, 5, 5) to (6, 6, 6) - doesn't overlap with cube1
        let vertices = [
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 5.0),
            Point3::new(5.0, 6.0, 5.0),
            Point3::new(5.0, 5.0, 6.0),
            Point3::new(6.0, 5.0, 6.0),
            Point3::new(6.0, 6.0, 6.0),
            Point3::new(5.0, 6.0, 6.0),
        ];

        for v in &vertices {
            cube2.vertices.push(Vertex::new(*v));
        }

        cube2.faces.push([0, 1, 2]);

        assert!(!meshes_overlap(&cube1, &cube2));
    }

    #[test]
    fn test_classify_faces_parallel() {
        let outer_cube = create_unit_cube();
        let bvh = Bvh::build(&outer_cube, 4);

        // Create a simple inner mesh
        let mut inner = IndexedMesh::new();
        inner.vertices.push(Vertex::new(Point3::new(0.4, 0.4, 0.4)));
        inner.vertices.push(Vertex::new(Point3::new(0.6, 0.4, 0.4)));
        inner.vertices.push(Vertex::new(Point3::new(0.5, 0.6, 0.4)));
        inner.faces.push([0, 1, 2]);

        let classifications = classify_faces_parallel(&inner, &outer_cube, &bvh, 1e-10);

        assert_eq!(classifications.len(), 1);
        assert_eq!(classifications[0], FaceLocation::Inside);
    }
}
