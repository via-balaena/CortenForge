//! Coplanar face handling for boolean operations.
//!
//! When two triangles from different meshes lie in the same plane,
//! special handling is required to determine which parts to keep.

#![allow(clippy::manual_let_else)]

use crate::config::CoplanarStrategy;
use crate::intersect::{triangle_normal, triangle_unit_normal};
use hashbrown::HashSet;
use mesh_types::{IndexedMesh, Point3, Vector3};

/// Result of coplanarity test between two triangles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoplanarityResult {
    /// Triangles are not coplanar.
    NotCoplanar,
    /// Triangles are coplanar with same orientation (normals point same direction).
    SameOrientation,
    /// Triangles are coplanar with opposite orientation (normals point opposite).
    OppositeOrientation,
}

/// Information about coplanar triangle pairs.
#[derive(Debug, Clone)]
pub struct CoplanarPair {
    /// Index of triangle in mesh A.
    pub tri_a: u32,
    /// Index of triangle in mesh B.
    pub tri_b: u32,
    /// Coplanarity result.
    pub result: CoplanarityResult,
}

/// Check if two triangles are coplanar.
///
/// # Arguments
///
/// * `a0`, `a1`, `a2` - First triangle vertices
/// * `b0`, `b1`, `b2` - Second triangle vertices
/// * `tolerance` - Distance tolerance for coplanarity detection
///
/// # Returns
///
/// `CoplanarityResult` indicating whether and how the triangles are coplanar.
#[must_use]
pub fn check_coplanarity(
    a0: &Point3<f64>,
    a1: &Point3<f64>,
    a2: &Point3<f64>,
    b0: &Point3<f64>,
    b1: &Point3<f64>,
    b2: &Point3<f64>,
    tolerance: f64,
) -> CoplanarityResult {
    // Get normal of triangle A
    let normal_a = match triangle_unit_normal(a0, a1, a2, tolerance) {
        Some(n) => n,
        None => return CoplanarityResult::NotCoplanar, // Degenerate triangle
    };

    // Check if all vertices of B are on the plane of A
    let d_a = normal_a.dot(&a0.coords);
    let dist_b0 = (normal_a.dot(&b0.coords) - d_a).abs();
    let dist_b1 = (normal_a.dot(&b1.coords) - d_a).abs();
    let dist_b2 = (normal_a.dot(&b2.coords) - d_a).abs();

    if dist_b0 > tolerance || dist_b1 > tolerance || dist_b2 > tolerance {
        return CoplanarityResult::NotCoplanar;
    }

    // Triangles are coplanar - check orientation
    let normal_b = match triangle_unit_normal(b0, b1, b2, tolerance) {
        Some(n) => n,
        None => return CoplanarityResult::NotCoplanar, // Degenerate triangle
    };

    let dot = normal_a.dot(&normal_b);

    if dot > 0.0 {
        CoplanarityResult::SameOrientation
    } else {
        CoplanarityResult::OppositeOrientation
    }
}

/// Check if two 2D triangles overlap using the separating axis theorem.
///
/// # Arguments
///
/// * `a0`, `a1`, `a2` - First triangle vertices in 2D
/// * `b0`, `b1`, `b2` - Second triangle vertices in 2D
///
/// # Returns
///
/// `true` if the triangles overlap.
#[must_use]
pub fn triangles_overlap_2d(
    a0: &[f64; 2],
    a1: &[f64; 2],
    a2: &[f64; 2],
    b0: &[f64; 2],
    b1: &[f64; 2],
    b2: &[f64; 2],
) -> bool {
    // Use separating axis theorem with 6 potential separating axes
    // (perpendicular to each edge of both triangles)
    let edges = [
        [a1[0] - a0[0], a1[1] - a0[1]],
        [a2[0] - a1[0], a2[1] - a1[1]],
        [a0[0] - a2[0], a0[1] - a2[1]],
        [b1[0] - b0[0], b1[1] - b0[1]],
        [b2[0] - b1[0], b2[1] - b1[1]],
        [b0[0] - b2[0], b0[1] - b2[1]],
    ];

    for edge in &edges {
        // Normal to edge (perpendicular)
        let axis = [-edge[1], edge[0]];

        // Skip zero-length edges
        if axis[0].abs() < 1e-10 && axis[1].abs() < 1e-10 {
            continue;
        }

        // Project all vertices onto axis
        let project = |p: &[f64; 2]| axis[0] * p[0] + axis[1] * p[1];

        let a_proj = [project(a0), project(a1), project(a2)];
        let b_proj = [project(b0), project(b1), project(b2)];

        let a_min = a_proj.iter().copied().fold(f64::MAX, f64::min);
        let a_max = a_proj.iter().copied().fold(f64::MIN, f64::max);
        let b_min = b_proj.iter().copied().fold(f64::MAX, f64::min);
        let b_max = b_proj.iter().copied().fold(f64::MIN, f64::max);

        // If projections don't overlap, triangles don't overlap
        if a_max < b_min || b_max < a_min {
            return false;
        }
    }

    // No separating axis found, triangles overlap
    true
}

/// Project a 3D point onto a 2D plane based on the dominant normal axis.
///
/// # Arguments
///
/// * `point` - The 3D point to project
/// * `normal` - The plane normal
///
/// # Returns
///
/// 2D coordinates in the projection plane.
#[must_use]
pub fn project_to_2d(point: &Point3<f64>, normal: &Vector3<f64>) -> [f64; 2] {
    let abs_normal = [normal.x.abs(), normal.y.abs(), normal.z.abs()];

    if abs_normal[0] >= abs_normal[1] && abs_normal[0] >= abs_normal[2] {
        // X is dominant, project to YZ plane
        [point.y, point.z]
    } else if abs_normal[1] >= abs_normal[2] {
        // Y is dominant, project to XZ plane
        [point.x, point.z]
    } else {
        // Z is dominant, project to XY plane
        [point.x, point.y]
    }
}

/// Check if two coplanar triangles overlap in 3D.
///
/// Projects both triangles to 2D and checks for overlap.
///
/// # Arguments
///
/// * `a0`, `a1`, `a2` - First triangle vertices
/// * `b0`, `b1`, `b2` - Second triangle vertices
///
/// # Returns
///
/// `true` if the coplanar triangles overlap.
#[must_use]
pub fn coplanar_triangles_overlap(
    a0: &Point3<f64>,
    a1: &Point3<f64>,
    a2: &Point3<f64>,
    b0: &Point3<f64>,
    b1: &Point3<f64>,
    b2: &Point3<f64>,
) -> bool {
    // Get normal for projection
    let normal = triangle_normal(a0, a1, a2);

    // Project all vertices to 2D
    let a0_2d = project_to_2d(a0, &normal);
    let a1_2d = project_to_2d(a1, &normal);
    let a2_2d = project_to_2d(a2, &normal);
    let b0_2d = project_to_2d(b0, &normal);
    let b1_2d = project_to_2d(b1, &normal);
    let b2_2d = project_to_2d(b2, &normal);

    triangles_overlap_2d(&a0_2d, &a1_2d, &a2_2d, &b0_2d, &b1_2d, &b2_2d)
}

/// Find all coplanar face pairs between two meshes.
///
/// # Arguments
///
/// * `mesh_a` - First mesh
/// * `mesh_b` - Second mesh
/// * `intersecting_pairs` - Pairs of triangles known to potentially intersect
/// * `tolerance` - Coplanarity tolerance
///
/// # Returns
///
/// Vector of `CoplanarPair` describing each coplanar pair found.
#[must_use]
pub fn find_coplanar_pairs(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    intersecting_pairs: &[(u32, u32)],
    tolerance: f64,
) -> Vec<CoplanarPair> {
    let mut pairs = Vec::new();

    for &(tri_a, tri_b) in intersecting_pairs {
        let face_a = &mesh_a.faces[tri_a as usize];
        let a0 = mesh_a.vertices[face_a[0] as usize].position;
        let a1 = mesh_a.vertices[face_a[1] as usize].position;
        let a2 = mesh_a.vertices[face_a[2] as usize].position;

        let face_b = &mesh_b.faces[tri_b as usize];
        let b0 = mesh_b.vertices[face_b[0] as usize].position;
        let b1 = mesh_b.vertices[face_b[1] as usize].position;
        let b2 = mesh_b.vertices[face_b[2] as usize].position;

        let result = check_coplanarity(&a0, &a1, &a2, &b0, &b1, &b2, tolerance);

        if result != CoplanarityResult::NotCoplanar {
            // Also check if they actually overlap in the plane
            if coplanar_triangles_overlap(&a0, &a1, &a2, &b0, &b1, &b2) {
                pairs.push(CoplanarPair {
                    tri_a,
                    tri_b,
                    result,
                });
            }
        }
    }

    pairs
}

/// Get set of coplanar face indices from mesh A.
#[must_use]
pub fn coplanar_faces_a(pairs: &[CoplanarPair]) -> HashSet<u32> {
    pairs.iter().map(|p| p.tri_a).collect()
}

/// Get set of coplanar face indices from mesh B.
#[must_use]
pub fn coplanar_faces_b(pairs: &[CoplanarPair]) -> HashSet<u32> {
    pairs.iter().map(|p| p.tri_b).collect()
}

/// Determine if a coplanar face should be included in the result.
///
/// # Arguments
///
/// * `is_from_first_mesh` - Whether the face is from mesh A (true) or B (false)
/// * `strategy` - The coplanar handling strategy
///
/// # Returns
///
/// `true` if the face should be included.
#[must_use]
pub fn should_include_coplanar_face(is_from_first_mesh: bool, strategy: CoplanarStrategy) -> bool {
    match strategy {
        CoplanarStrategy::Include => is_from_first_mesh,
        CoplanarStrategy::Exclude => false,
        CoplanarStrategy::KeepBoth => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_check_coplanarity_same_plane_same_orientation() {
        // Two triangles in z=0 plane, same winding
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let a2 = Point3::new(0.5, 1.0, 0.0);

        let b0 = Point3::new(0.5, 0.0, 0.0);
        let b1 = Point3::new(1.5, 0.0, 0.0);
        let b2 = Point3::new(1.0, 1.0, 0.0);

        let result = check_coplanarity(&a0, &a1, &a2, &b0, &b1, &b2, 1e-10);
        assert_eq!(result, CoplanarityResult::SameOrientation);
    }

    #[test]
    fn test_check_coplanarity_same_plane_opposite_orientation() {
        // Two triangles in z=0 plane, opposite winding
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let a2 = Point3::new(0.5, 1.0, 0.0);

        // Reversed winding
        let b0 = Point3::new(0.5, 0.0, 0.0);
        let b1 = Point3::new(1.0, 1.0, 0.0);
        let b2 = Point3::new(1.5, 0.0, 0.0);

        let result = check_coplanarity(&a0, &a1, &a2, &b0, &b1, &b2, 1e-10);
        assert_eq!(result, CoplanarityResult::OppositeOrientation);
    }

    #[test]
    fn test_check_coplanarity_not_coplanar() {
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let a2 = Point3::new(0.5, 1.0, 0.0);

        // Triangle in different plane
        let b0 = Point3::new(0.0, 0.0, 1.0);
        let b1 = Point3::new(1.0, 0.0, 1.0);
        let b2 = Point3::new(0.5, 1.0, 1.0);

        let result = check_coplanarity(&a0, &a1, &a2, &b0, &b1, &b2, 1e-10);
        assert_eq!(result, CoplanarityResult::NotCoplanar);
    }

    #[test]
    fn test_triangles_overlap_2d_yes() {
        let a0 = [0.0, 0.0];
        let a1 = [1.0, 0.0];
        let a2 = [0.5, 1.0];

        let b0 = [0.25, 0.25];
        let b1 = [0.75, 0.25];
        let b2 = [0.5, 0.75];

        assert!(triangles_overlap_2d(&a0, &a1, &a2, &b0, &b1, &b2));
    }

    #[test]
    fn test_triangles_overlap_2d_no() {
        let a0 = [0.0, 0.0];
        let a1 = [1.0, 0.0];
        let a2 = [0.5, 1.0];

        let b0 = [2.0, 0.0];
        let b1 = [3.0, 0.0];
        let b2 = [2.5, 1.0];

        assert!(!triangles_overlap_2d(&a0, &a1, &a2, &b0, &b1, &b2));
    }

    #[test]
    fn test_project_to_2d_z_dominant() {
        let point = Point3::new(1.0, 2.0, 3.0);
        let normal = Vector3::new(0.0, 0.0, 1.0);

        let proj = project_to_2d(&point, &normal);
        assert!((proj[0] - 1.0).abs() < 1e-10); // x
        assert!((proj[1] - 2.0).abs() < 1e-10); // y
    }

    #[test]
    fn test_project_to_2d_x_dominant() {
        let point = Point3::new(1.0, 2.0, 3.0);
        let normal = Vector3::new(1.0, 0.0, 0.0);

        let proj = project_to_2d(&point, &normal);
        assert!((proj[0] - 2.0).abs() < 1e-10); // y
        assert!((proj[1] - 3.0).abs() < 1e-10); // z
    }

    #[test]
    fn test_project_to_2d_y_dominant() {
        let point = Point3::new(1.0, 2.0, 3.0);
        let normal = Vector3::new(0.0, 1.0, 0.0);

        let proj = project_to_2d(&point, &normal);
        assert!((proj[0] - 1.0).abs() < 1e-10); // x
        assert!((proj[1] - 3.0).abs() < 1e-10); // z
    }

    #[test]
    fn test_coplanar_triangles_overlap() {
        // Two overlapping triangles in z=0 plane
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let a2 = Point3::new(0.5, 1.0, 0.0);

        let b0 = Point3::new(0.25, 0.25, 0.0);
        let b1 = Point3::new(0.75, 0.25, 0.0);
        let b2 = Point3::new(0.5, 0.75, 0.0);

        assert!(coplanar_triangles_overlap(&a0, &a1, &a2, &b0, &b1, &b2));
    }

    #[test]
    fn test_should_include_coplanar_face() {
        // Include strategy
        assert!(should_include_coplanar_face(true, CoplanarStrategy::Include));
        assert!(!should_include_coplanar_face(
            false,
            CoplanarStrategy::Include
        ));

        // Exclude strategy
        assert!(!should_include_coplanar_face(
            true,
            CoplanarStrategy::Exclude
        ));
        assert!(!should_include_coplanar_face(
            false,
            CoplanarStrategy::Exclude
        ));

        // KeepBoth strategy
        assert!(should_include_coplanar_face(
            true,
            CoplanarStrategy::KeepBoth
        ));
        assert!(should_include_coplanar_face(
            false,
            CoplanarStrategy::KeepBoth
        ));
    }

    #[test]
    fn test_coplanar_faces_sets() {
        let pairs = vec![
            CoplanarPair {
                tri_a: 0,
                tri_b: 5,
                result: CoplanarityResult::SameOrientation,
            },
            CoplanarPair {
                tri_a: 2,
                tri_b: 7,
                result: CoplanarityResult::OppositeOrientation,
            },
            CoplanarPair {
                tri_a: 0,
                tri_b: 8,
                result: CoplanarityResult::SameOrientation,
            },
        ];

        let faces_a = coplanar_faces_a(&pairs);
        let faces_b = coplanar_faces_b(&pairs);

        assert_eq!(faces_a.len(), 2); // 0 and 2
        assert!(faces_a.contains(&0));
        assert!(faces_a.contains(&2));

        assert_eq!(faces_b.len(), 3); // 5, 7, and 8
        assert!(faces_b.contains(&5));
        assert!(faces_b.contains(&7));
        assert!(faces_b.contains(&8));
    }
}
