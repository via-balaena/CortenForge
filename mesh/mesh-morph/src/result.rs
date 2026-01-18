//! Morphing result and quality metrics.
//!
//! This module provides the [`MorphResult`] struct which contains the deformed
//! mesh along with quality metrics for analyzing the deformation.

use mesh_types::IndexedMesh;

/// Result of a mesh morphing operation.
///
/// Contains the deformed mesh along with metrics describing the deformation.
///
/// # Examples
///
/// ```
/// use mesh_morph::{morph_mesh, MorphParams, Constraint, MorphOutput};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Point3;
///
/// // Create a simple mesh
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let params = MorphParams::rbf()
///     .with_constraint(Constraint::point(
///         Point3::new(0.0, 0.0, 0.0),
///         Point3::new(0.0, 0.0, 0.0),
///     ))
///     .with_constraint(Constraint::point(
///         Point3::new(1.0, 0.0, 0.0),
///         Point3::new(1.0, 0.0, 0.0),
///     ))
///     .with_constraint(Constraint::point(
///         Point3::new(0.0, 1.0, 0.0),
///         Point3::new(0.0, 1.0, 0.0),
///     ));
///
/// let result = morph_mesh(&mesh, &params).unwrap();
///
/// println!("Vertices modified: {}", result.vertices_modified);
/// println!("Max displacement: {:.6}", result.max_displacement);
/// ```
#[derive(Debug, Clone)]
pub struct MorphOutput {
    /// The deformed mesh.
    pub mesh: IndexedMesh,
    /// Number of vertices that were modified.
    pub vertices_modified: usize,
    /// Maximum vertex displacement distance.
    pub max_displacement: f64,
    /// Average vertex displacement distance.
    pub average_displacement: f64,
    /// Maximum edge stretch ratio (>1 means elongation).
    pub max_stretch: f64,
    /// Maximum edge compression ratio (<1 means compression).
    pub max_compression: f64,
    /// Volume ratio: `new_volume` / `original_volume`.
    pub volume_ratio: f64,
}

impl MorphOutput {
    /// Creates a new morph result.
    #[must_use]
    pub const fn new(mesh: IndexedMesh) -> Self {
        Self {
            mesh,
            vertices_modified: 0,
            max_displacement: 0.0,
            average_displacement: 0.0,
            max_stretch: 1.0,
            max_compression: 1.0,
            volume_ratio: 1.0,
        }
    }

    /// Returns whether the deformation has significant distortion.
    ///
    /// Distortion is measured as the maximum deviation from uniform scaling.
    ///
    /// # Arguments
    ///
    /// * `threshold` - Maximum allowed stretch/compression ratio deviation from 1.0
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphOutput;
    /// use mesh_types::IndexedMesh;
    ///
    /// let mut result = MorphOutput::new(IndexedMesh::new());
    /// result.max_stretch = 1.2;
    /// result.max_compression = 0.8;
    ///
    /// // 20% stretch exceeds 10% threshold
    /// assert!(result.has_significant_distortion(0.1));
    ///
    /// // 20% stretch is within 25% threshold
    /// assert!(!result.has_significant_distortion(0.25));
    /// ```
    #[must_use]
    pub fn has_significant_distortion(&self, threshold: f64) -> bool {
        (self.max_stretch - 1.0).abs() > threshold || (self.max_compression - 1.0).abs() > threshold
    }

    /// Returns whether the deformation has significant volume change.
    ///
    /// # Arguments
    ///
    /// * `threshold` - Maximum allowed volume ratio deviation from 1.0
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphOutput;
    /// use mesh_types::IndexedMesh;
    ///
    /// let mut result = MorphOutput::new(IndexedMesh::new());
    /// result.volume_ratio = 1.15;
    ///
    /// // 15% volume increase exceeds 10% threshold
    /// assert!(result.has_significant_volume_change(0.1));
    ///
    /// // 15% volume increase is within 20% threshold
    /// assert!(!result.has_significant_volume_change(0.2));
    /// ```
    #[must_use]
    pub fn has_significant_volume_change(&self, threshold: f64) -> bool {
        (self.volume_ratio - 1.0).abs() > threshold
    }

    /// Returns a summary of the deformation quality.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphOutput;
    /// use mesh_types::IndexedMesh;
    ///
    /// let result = MorphOutput::new(IndexedMesh::new());
    /// println!("{}", result.summary());
    /// ```
    #[must_use]
    pub fn summary(&self) -> String {
        format!(
            "MorphResult: {} vertices modified, max displacement: {:.6}, \
             avg displacement: {:.6}, max stretch: {:.4}, max compression: {:.4}, \
             volume ratio: {:.4}",
            self.vertices_modified,
            self.max_displacement,
            self.average_displacement,
            self.max_stretch,
            self.max_compression,
            self.volume_ratio
        )
    }
}

/// Computes the signed volume of a mesh using the divergence theorem.
///
/// Returns a positive value for a consistently-wound closed mesh,
/// negative if the winding is inverted, or an unreliable value
/// if the mesh is not closed.
pub fn compute_signed_volume(mesh: &IndexedMesh) -> f64 {
    let mut volume = 0.0;

    for tri in &mesh.faces {
        let v0 = &mesh.vertices[tri[0] as usize].position;
        let v1 = &mesh.vertices[tri[1] as usize].position;
        let v2 = &mesh.vertices[tri[2] as usize].position;

        // Signed volume of tetrahedron with origin
        volume += v0.coords.dot(&v1.coords.cross(&v2.coords));
    }

    volume / 6.0
}

/// Computes edge length statistics for a mesh.
///
/// Returns (`min_length`, `max_length`, `total_length`, `edge_count`).
pub fn compute_edge_stats(mesh: &IndexedMesh) -> (f64, f64, f64, usize) {
    use std::collections::HashSet;

    let mut edges: HashSet<(usize, usize)> = HashSet::new();

    for tri in &mesh.faces {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;

        edges.insert((i0.min(i1), i0.max(i1)));
        edges.insert((i1.min(i2), i1.max(i2)));
        edges.insert((i2.min(i0), i2.max(i0)));
    }

    if edges.is_empty() {
        return (0.0, 0.0, 0.0, 0);
    }

    let mut min_len: f64 = f64::MAX;
    let mut max_len: f64 = 0.0;
    let mut total_len: f64 = 0.0;

    for (i, j) in &edges {
        let len = (mesh.vertices[*i].position - mesh.vertices[*j].position).norm();
        min_len = min_len.min(len);
        max_len = max_len.max(len);
        total_len += len;
    }

    (min_len, max_len, total_len, edges.len())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Regular tetrahedron
        mesh.vertices
            .push(Vertex::from_coords(1.0, 1.0, 1.0));
        mesh.vertices
            .push(Vertex::from_coords(1.0, -1.0, -1.0));
        mesh.vertices
            .push(Vertex::from_coords(-1.0, 1.0, -1.0));
        mesh.vertices
            .push(Vertex::from_coords(-1.0, -1.0, 1.0));

        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh.faces.push([0, 3, 1]);
        mesh.faces.push([1, 3, 2]);

        mesh
    }

    #[test]
    fn test_morph_result_new() {
        let result = MorphOutput::new(IndexedMesh::new());
        assert_eq!(result.vertices_modified, 0);
        assert!((result.max_displacement - 0.0).abs() < f64::EPSILON);
        assert!((result.max_stretch - 1.0).abs() < f64::EPSILON);
        assert!((result.volume_ratio - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_has_significant_distortion() {
        let mut result = MorphOutput::new(IndexedMesh::new());

        // No distortion
        assert!(!result.has_significant_distortion(0.1));

        // Stretch distortion
        result.max_stretch = 1.15;
        assert!(result.has_significant_distortion(0.1));
        assert!(!result.has_significant_distortion(0.2));

        // Compression distortion
        result.max_stretch = 1.0;
        result.max_compression = 0.85;
        assert!(result.has_significant_distortion(0.1));
    }

    #[test]
    fn test_has_significant_volume_change() {
        let mut result = MorphOutput::new(IndexedMesh::new());

        assert!(!result.has_significant_volume_change(0.1));

        result.volume_ratio = 1.2;
        assert!(result.has_significant_volume_change(0.1));
        assert!(!result.has_significant_volume_change(0.25));

        result.volume_ratio = 0.8;
        assert!(result.has_significant_volume_change(0.1));
    }

    #[test]
    fn test_summary() {
        let result = MorphOutput::new(IndexedMesh::new());
        let summary = result.summary();
        assert!(summary.contains("vertices modified"));
        assert!(summary.contains("displacement"));
    }

    #[test]
    fn test_compute_signed_volume() {
        let mesh = make_tetrahedron();
        let volume = compute_signed_volume(&mesh);
        // Volume of a tetrahedron with edge length 2√2 is 8/3 ≈ 2.667
        // Our tetrahedron has different orientation, so sign may vary
        assert!(volume.abs() > 0.1);
    }

    #[test]
    fn test_compute_edge_stats() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let (min_len, max_len, total_len, edge_count) = compute_edge_stats(&mesh);

        assert_eq!(edge_count, 3);
        assert_relative_eq!(min_len, 1.0, epsilon = 1e-10);
        assert_relative_eq!(max_len, 2.0_f64.sqrt(), epsilon = 1e-10);
        assert!(total_len > 0.0);
    }

    #[test]
    fn test_compute_edge_stats_empty() {
        let mesh = IndexedMesh::new();
        let (_min_len, _max_len, total_len, edge_count) = compute_edge_stats(&mesh);

        assert_eq!(edge_count, 0);
        assert!((total_len - 0.0).abs() < f64::EPSILON);
    }
}
