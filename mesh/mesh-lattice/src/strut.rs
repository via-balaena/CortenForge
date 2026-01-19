//! Cylindrical strut mesh generation.
//!
//! This module provides functions for creating triangulated cylindrical
//! struts, which are used to build strut-based lattice structures.

// Allow numeric casts inherent to geometry (vertex indices, segment counts)
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_precision_loss)]

use mesh_types::{IndexedMesh, Vertex};
use nalgebra::{Point3, Vector3};

/// Number of segments around the circumference of a strut.
const STRUT_SEGMENTS: usize = 6;

/// Generates a cylindrical strut mesh between two points.
///
/// Creates a hexagonal prism (6 segments) with end caps.
///
/// # Arguments
///
/// * `start` - Start point of the strut
/// * `end` - End point of the strut
/// * `radius` - Radius of the cylinder
///
/// # Returns
///
/// An `IndexedMesh` representing the strut, or `None` if the strut
/// is degenerate (zero length).
///
/// # Examples
///
/// ```
/// use mesh_lattice::generate_strut;
/// use mesh_types::MeshTopology;
/// use nalgebra::Point3;
///
/// let strut = generate_strut(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(0.0, 0.0, 10.0),
///     0.5,
/// );
/// assert!(strut.is_some());
/// let mesh = strut.unwrap();
/// assert!(mesh.vertex_count() > 0);
/// ```
#[must_use]
pub fn generate_strut(start: Point3<f64>, end: Point3<f64>, radius: f64) -> Option<IndexedMesh> {
    generate_strut_tapered(start, end, radius, radius)
}

/// Generates a tapered cylindrical strut mesh between two points.
///
/// # Arguments
///
/// * `start` - Start point of the strut
/// * `end` - End point of the strut
/// * `start_radius` - Radius at the start point
/// * `end_radius` - Radius at the end point
///
/// # Returns
///
/// An `IndexedMesh` representing the strut, or `None` if the strut
/// is degenerate (zero length).
#[must_use]
pub fn generate_strut_tapered(
    start: Point3<f64>,
    end: Point3<f64>,
    start_radius: f64,
    end_radius: f64,
) -> Option<IndexedMesh> {
    let axis = end - start;
    let length = axis.norm();

    if length < f64::EPSILON {
        return None;
    }

    let axis_normalized = axis / length;

    // Find perpendicular vectors to the axis
    let (perp1, perp2) = compute_perpendicular_vectors(axis_normalized);

    // Generate vertices around the circumference at each end
    let mut vertices = Vec::with_capacity(STRUT_SEGMENTS * 2 + 2);
    let mut faces = Vec::new();

    // Add cap center vertices
    let start_center_idx = 0u32;
    let end_center_idx = 1u32;
    vertices.push(Vertex::new(start));
    vertices.push(Vertex::new(end));

    // Generate ring vertices at start and end
    for i in 0..STRUT_SEGMENTS {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (STRUT_SEGMENTS as f64);
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        // Start ring
        let offset_start = (perp1 * cos_a + perp2 * sin_a) * start_radius;
        let start_pos = start + offset_start;
        vertices.push(Vertex::new(start_pos));

        // End ring
        let offset_end = (perp1 * cos_a + perp2 * sin_a) * end_radius;
        let end_pos = end + offset_end;
        vertices.push(Vertex::new(end_pos));
    }

    // Generate triangles for the side surface
    for i in 0..STRUT_SEGMENTS {
        let next = (i + 1) % STRUT_SEGMENTS;

        // Indices in the vertex array (start ring begins at index 2)
        let s0 = (2 + i * 2) as u32;
        let s1 = (2 + next * 2) as u32;
        let e0 = (2 + i * 2 + 1) as u32;
        let e1 = (2 + next * 2 + 1) as u32;

        // Two triangles per quad
        faces.push([s0, e0, s1]);
        faces.push([s1, e0, e1]);
    }

    // Generate triangles for the start cap (pointing inward along axis)
    for i in 0..STRUT_SEGMENTS {
        let next = (i + 1) % STRUT_SEGMENTS;
        let s0 = (2 + i * 2) as u32;
        let s1 = (2 + next * 2) as u32;
        // Wind counter-clockwise when viewed from start
        faces.push([start_center_idx, s1, s0]);
    }

    // Generate triangles for the end cap (pointing outward along axis)
    for i in 0..STRUT_SEGMENTS {
        let next = (i + 1) % STRUT_SEGMENTS;
        let e0 = (2 + i * 2 + 1) as u32;
        let e1 = (2 + next * 2 + 1) as u32;
        // Wind counter-clockwise when viewed from end
        faces.push([end_center_idx, e0, e1]);
    }

    Some(IndexedMesh::from_parts(vertices, faces))
}

/// Computes two perpendicular vectors to the given axis.
///
/// Returns vectors that form an orthonormal basis with the axis.
fn compute_perpendicular_vectors(axis: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Choose a vector not parallel to axis
    let up = if axis.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };

    let perp1 = axis.cross(&up).normalize();
    let perp2 = axis.cross(&perp1);

    (perp1, perp2)
}

/// Combines multiple strut meshes into a single mesh.
///
/// This is more efficient than keeping separate meshes for each strut.
///
/// # Arguments
///
/// * `struts` - Iterator of individual strut meshes
///
/// # Returns
///
/// A single combined mesh.
///
/// # Examples
///
/// ```
/// use mesh_lattice::{generate_strut, combine_struts};
/// use mesh_types::MeshTopology;
/// use nalgebra::Point3;
///
/// let struts = vec![
///     generate_strut(Point3::origin(), Point3::new(1.0, 0.0, 0.0), 0.1),
///     generate_strut(Point3::origin(), Point3::new(0.0, 1.0, 0.0), 0.1),
/// ];
/// let combined = combine_struts(struts.into_iter().flatten());
/// assert!(combined.vertex_count() > 0);
/// ```
#[must_use]
pub fn combine_struts(struts: impl Iterator<Item = IndexedMesh>) -> IndexedMesh {
    let mut all_vertices = Vec::new();
    let mut all_faces = Vec::new();

    for strut in struts {
        let base_index = all_vertices.len() as u32;

        // Add vertices
        all_vertices.extend(strut.vertices.iter().cloned());

        // Add faces with offset
        for face in &strut.faces {
            all_faces.push([
                face[0] + base_index,
                face[1] + base_index,
                face[2] + base_index,
            ]);
        }
    }

    IndexedMesh::from_parts(all_vertices, all_faces)
}

/// Estimates the volume of a single strut.
///
/// For uniform radius, this is the cylinder volume: π * r² * h.
/// For tapered struts, uses truncated cone formula.
#[must_use]
pub fn estimate_strut_volume(length: f64, start_radius: f64, end_radius: f64) -> f64 {
    use std::f64::consts::PI;

    if (start_radius - end_radius).abs() < f64::EPSILON {
        // Cylinder
        PI * start_radius.powi(2) * length
    } else {
        // Truncated cone: (π/3) * h * (r1² + r1*r2 + r2²)
        (PI / 3.0)
            * length
            * end_radius.mul_add(
                end_radius,
                start_radius.mul_add(start_radius, start_radius * end_radius),
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::MeshTopology;

    #[test]
    fn test_generate_strut_basic() {
        let strut = generate_strut(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 10.0), 0.5);
        assert!(strut.is_some());

        let mesh = strut.unwrap();
        // 2 cap centers + 6 * 2 ring vertices = 14
        assert_eq!(mesh.vertex_count(), 14);
        // 6 quads * 2 triangles + 6 start cap + 6 end cap = 24 triangles
        assert_eq!(mesh.face_count(), 24);
    }

    #[test]
    fn test_generate_strut_zero_length() {
        let strut = generate_strut(Point3::new(1.0, 2.0, 3.0), Point3::new(1.0, 2.0, 3.0), 0.5);
        assert!(strut.is_none());
    }

    #[test]
    fn test_generate_strut_tapered() {
        let strut = generate_strut_tapered(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            0.5,
            0.25,
        );
        assert!(strut.is_some());
    }

    #[test]
    fn test_generate_strut_along_x() {
        let strut = generate_strut(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0), 0.5);
        assert!(strut.is_some());
    }

    #[test]
    fn test_generate_strut_diagonal() {
        let strut = generate_strut(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 10.0),
            0.5,
        );
        assert!(strut.is_some());
    }

    #[test]
    fn test_combine_struts() {
        let struts = vec![
            generate_strut(Point3::origin(), Point3::new(1.0, 0.0, 0.0), 0.1),
            generate_strut(Point3::origin(), Point3::new(0.0, 1.0, 0.0), 0.1),
            generate_strut(Point3::origin(), Point3::new(0.0, 0.0, 1.0), 0.1),
        ];

        let combined = combine_struts(struts.into_iter().flatten());
        assert_eq!(combined.vertex_count(), 14 * 3);
        assert_eq!(combined.face_count(), 24 * 3);
    }

    #[test]
    fn test_combine_empty() {
        let combined = combine_struts(std::iter::empty());
        assert!(combined.is_empty());
    }

    #[test]
    fn test_estimate_strut_volume_cylinder() {
        use std::f64::consts::PI;

        let volume = estimate_strut_volume(10.0, 1.0, 1.0);
        let expected = PI * 1.0 * 10.0;
        assert_relative_eq!(volume, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_estimate_strut_volume_cone() {
        use std::f64::consts::PI;

        // Full cone (end radius = 0)
        let volume = estimate_strut_volume(10.0, 1.0, 0.0);
        // Cone volume = (1/3) * π * r² * h
        let expected = (PI / 3.0) * 1.0 * 10.0;
        assert_relative_eq!(volume, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_perpendicular_vectors() {
        let axis = Vector3::new(0.0, 0.0, 1.0);
        let (perp1, perp2) = compute_perpendicular_vectors(axis);

        // Check orthogonality
        assert_relative_eq!(axis.dot(&perp1), 0.0, epsilon = 1e-10);
        assert_relative_eq!(axis.dot(&perp2), 0.0, epsilon = 1e-10);
        assert_relative_eq!(perp1.dot(&perp2), 0.0, epsilon = 1e-10);

        // Check normalization
        assert_relative_eq!(perp1.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(perp2.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_perpendicular_vectors_x_axis() {
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let (perp1, perp2) = compute_perpendicular_vectors(axis);

        assert_relative_eq!(axis.dot(&perp1), 0.0, epsilon = 1e-10);
        assert_relative_eq!(axis.dot(&perp2), 0.0, epsilon = 1e-10);
    }
}
