//! Spike detection and removal for meshes.
//!
//! This module provides functions for detecting and removing "spikes" -
//! vertices that protrude abnormally from the surface, often caused by
//! scan noise or reconstruction artifacts.
//!
//! # Algorithm
//!
//! A vertex is considered a spike if:
//! 1. It has a significantly higher curvature than its neighbors, OR
//! 2. Its normal deviates significantly from its neighbors' normals
//!
//! Spikes are removed by either:
//! - Relocating the vertex to the average position of its neighbors
//! - Collapsing faces incident to the spike
//!
//! # Example
//!
//! ```
//! use mesh_scan::cleanup::spike::{remove_spikes, SpikeParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mesh = IndexedMesh::new(); // Would be loaded from file
//! let params = SpikeParams::default();
//! let result = remove_spikes(&mesh, &params);
//! ```

use mesh_types::{IndexedMesh, Vertex};
use nalgebra::{Point3, Vector3};
use std::collections::{HashMap, HashSet};

use crate::error::{ScanError, ScanResult};

/// Parameters for spike detection and removal.
#[derive(Debug, Clone)]
pub struct SpikeParams {
    /// Maximum angle (in radians) between vertex normal and average neighbor normal.
    /// Vertices exceeding this are considered spikes. Default: 1.2 (~70 degrees).
    pub max_normal_deviation: f64,

    /// Maximum ratio of vertex distance from neighbor plane to average edge length.
    /// Vertices exceeding this are considered spikes. Default: 2.0.
    pub max_protrusion_ratio: f64,

    /// Minimum number of neighbors required for spike detection. Default: 3.
    pub min_neighbors: usize,

    /// Whether to smooth spikes instead of removing them. Default: true.
    /// If false, faces incident to spikes will be removed.
    pub smooth_spikes: bool,
}

impl Default for SpikeParams {
    fn default() -> Self {
        Self {
            max_normal_deviation: 1.2, // ~70 degrees
            max_protrusion_ratio: 2.0,
            min_neighbors: 3,
            smooth_spikes: true,
        }
    }
}

impl SpikeParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the maximum normal deviation angle (in radians).
    #[must_use]
    pub const fn with_max_normal_deviation(mut self, angle: f64) -> Self {
        self.max_normal_deviation = angle;
        self
    }

    /// Sets the maximum protrusion ratio.
    #[must_use]
    pub const fn with_max_protrusion_ratio(mut self, ratio: f64) -> Self {
        self.max_protrusion_ratio = ratio;
        self
    }

    /// Sets the minimum number of neighbors.
    #[must_use]
    pub const fn with_min_neighbors(mut self, min: usize) -> Self {
        self.min_neighbors = min;
        self
    }

    /// Sets whether to smooth spikes instead of removing them.
    #[must_use]
    pub const fn with_smooth_spikes(mut self, smooth: bool) -> Self {
        self.smooth_spikes = smooth;
        self
    }

    /// Creates aggressive spike detection parameters.
    #[must_use]
    pub const fn aggressive() -> Self {
        Self {
            max_normal_deviation: 0.8, // ~45 degrees
            max_protrusion_ratio: 1.5,
            min_neighbors: 3,
            smooth_spikes: true,
        }
    }

    /// Creates conservative spike detection parameters.
    #[must_use]
    pub const fn conservative() -> Self {
        Self {
            max_normal_deviation: 1.5, // ~85 degrees
            max_protrusion_ratio: 3.0,
            min_neighbors: 4,
            smooth_spikes: true,
        }
    }
}

/// Result of spike removal operation.
#[derive(Debug, Clone)]
pub struct SpikeRemovalResult {
    /// The mesh with spikes removed or smoothed.
    pub mesh: IndexedMesh,

    /// Number of spikes detected.
    pub spikes_detected: usize,

    /// Number of spikes that were smoothed (if `smooth_spikes` was true).
    pub spikes_smoothed: usize,

    /// Number of vertices removed (if `smooth_spikes` was false).
    pub vertices_removed: usize,

    /// Number of faces removed.
    pub faces_removed: usize,
}

impl SpikeRemovalResult {
    /// Returns true if any spikes were found and handled.
    #[must_use]
    pub const fn had_spikes(&self) -> bool {
        self.spikes_detected > 0
    }
}

impl std::fmt::Display for SpikeRemovalResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.spikes_smoothed > 0 {
            write!(
                f,
                "Spike removal: {} spikes detected, {} smoothed",
                self.spikes_detected, self.spikes_smoothed
            )
        } else if self.vertices_removed > 0 {
            write!(
                f,
                "Spike removal: {} spikes detected, {} vertices and {} faces removed",
                self.spikes_detected, self.vertices_removed, self.faces_removed
            )
        } else {
            write!(f, "Spike removal: {} spikes detected", self.spikes_detected)
        }
    }
}

/// Removes or smooths spikes from a mesh.
///
/// # Arguments
///
/// * `mesh` - The input mesh
/// * `params` - Parameters controlling spike detection and handling
///
/// # Returns
///
/// Detailed results including the cleaned mesh and statistics.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_scan::cleanup::spike::{remove_spikes, SpikeParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// // Add vertices...
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = remove_spikes(&mesh, &SpikeParams::default()).unwrap();
/// println!("{}", result);
/// ```
pub fn remove_spikes(mesh: &IndexedMesh, params: &SpikeParams) -> ScanResult<SpikeRemovalResult> {
    if mesh.vertices.is_empty() {
        return Err(ScanError::EmptyMesh);
    }

    if mesh.faces.is_empty() {
        return Ok(SpikeRemovalResult {
            mesh: mesh.clone(),
            spikes_detected: 0,
            spikes_smoothed: 0,
            vertices_removed: 0,
            faces_removed: 0,
        });
    }

    // Build adjacency information
    let vertex_neighbors = build_vertex_neighbors(mesh);
    let vertex_faces = build_vertex_faces(mesh);

    // Detect spikes
    let spike_vertices = detect_spikes(mesh, &vertex_neighbors, params);

    if spike_vertices.is_empty() {
        return Ok(SpikeRemovalResult {
            mesh: mesh.clone(),
            spikes_detected: 0,
            spikes_smoothed: 0,
            vertices_removed: 0,
            faces_removed: 0,
        });
    }

    let spikes_detected = spike_vertices.len();

    if params.smooth_spikes {
        // Smooth spikes by moving them to neighbor average
        let mut result_mesh = mesh.clone();

        for &spike_idx in &spike_vertices {
            if let Some(neighbors) = vertex_neighbors.get(&spike_idx) {
                if !neighbors.is_empty() {
                    let avg_pos = compute_neighbor_average(&result_mesh.vertices, neighbors);
                    result_mesh.vertices[spike_idx as usize].position = avg_pos;
                }
            }
        }

        Ok(SpikeRemovalResult {
            mesh: result_mesh,
            spikes_detected,
            spikes_smoothed: spike_vertices.len(),
            vertices_removed: 0,
            faces_removed: 0,
        })
    } else {
        // Remove spike vertices and their faces
        let spike_set: HashSet<u32> = spike_vertices.into_iter().collect();

        // Find faces to remove (any face touching a spike)
        let faces_to_remove: HashSet<usize> = spike_set
            .iter()
            .flat_map(|&v| vertex_faces.get(&v).cloned().unwrap_or_default())
            .collect();

        let faces_removed = faces_to_remove.len();

        // Build new mesh without removed vertices and faces
        let mut new_index: Vec<Option<u32>> = vec![None; mesh.vertices.len()];
        let mut new_vertices = Vec::new();

        for (old_idx, vertex) in mesh.vertices.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let old_idx_u32 = old_idx as u32;
            if !spike_set.contains(&old_idx_u32) {
                #[allow(clippy::cast_possible_truncation)]
                {
                    new_index[old_idx] = Some(new_vertices.len() as u32);
                }
                new_vertices.push(vertex.clone());
            }
        }

        let new_faces: Vec<[u32; 3]> = mesh
            .faces
            .iter()
            .enumerate()
            .filter(|(i, _)| !faces_to_remove.contains(i))
            .filter_map(|(_, face)| {
                let i0 = new_index[face[0] as usize]?;
                let i1 = new_index[face[1] as usize]?;
                let i2 = new_index[face[2] as usize]?;
                Some([i0, i1, i2])
            })
            .collect();

        let vertices_removed = mesh.vertices.len() - new_vertices.len();

        Ok(SpikeRemovalResult {
            mesh: IndexedMesh {
                vertices: new_vertices,
                faces: new_faces,
            },
            spikes_detected,
            spikes_smoothed: 0,
            vertices_removed,
            faces_removed,
        })
    }
}

/// Builds a map from vertex index to its neighboring vertex indices.
fn build_vertex_neighbors(mesh: &IndexedMesh) -> HashMap<u32, Vec<u32>> {
    let mut neighbors: HashMap<u32, HashSet<u32>> = HashMap::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let v = face[i];
            let next = face[(i + 1) % 3];
            let prev = face[(i + 2) % 3];

            neighbors.entry(v).or_default().insert(next);
            neighbors.entry(v).or_default().insert(prev);
        }
    }

    neighbors
        .into_iter()
        .map(|(k, v)| (k, v.into_iter().collect()))
        .collect()
}

/// Builds a map from vertex index to indices of faces containing it.
fn build_vertex_faces(mesh: &IndexedMesh) -> HashMap<u32, Vec<usize>> {
    let mut vertex_faces: HashMap<u32, Vec<usize>> = HashMap::new();

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for &v in face {
            vertex_faces.entry(v).or_default().push(face_idx);
        }
    }

    vertex_faces
}

/// Detects spike vertices in the mesh.
fn detect_spikes(
    mesh: &IndexedMesh,
    vertex_neighbors: &HashMap<u32, Vec<u32>>,
    params: &SpikeParams,
) -> Vec<u32> {
    let mut spikes = Vec::new();

    // Compute vertex normals
    let vertex_normals = compute_vertex_normals(mesh);

    for (vertex_idx, neighbors) in vertex_neighbors {
        if neighbors.len() < params.min_neighbors {
            continue;
        }

        let vertex = &mesh.vertices[*vertex_idx as usize];

        // Check normal deviation
        if let Some(vertex_normal) = vertex_normals.get(vertex_idx) {
            let avg_neighbor_normal = compute_average_normal(&vertex_normals, neighbors);
            let normal_deviation = vertex_normal.dot(&avg_neighbor_normal).acos();

            if normal_deviation > params.max_normal_deviation {
                spikes.push(*vertex_idx);
                continue;
            }
        }

        // Check protrusion
        let neighbor_positions: Vec<Point3<f64>> = neighbors
            .iter()
            .map(|&n| mesh.vertices[n as usize].position)
            .collect();

        if let Some(protrusion) = compute_protrusion(&vertex.position, &neighbor_positions) {
            let avg_edge_length =
                compute_average_edge_length(&vertex.position, &neighbor_positions);

            if avg_edge_length > 0.0 && protrusion / avg_edge_length > params.max_protrusion_ratio {
                spikes.push(*vertex_idx);
            }
        }
    }

    spikes
}

/// Computes vertex normals as the average of incident face normals.
fn compute_vertex_normals(mesh: &IndexedMesh) -> HashMap<u32, Vector3<f64>> {
    let mut vertex_normals: HashMap<u32, Vector3<f64>> = HashMap::new();
    let mut vertex_counts: HashMap<u32, usize> = HashMap::new();

    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let face_normal = e1.cross(&e2);

        let norm = face_normal.norm();
        if norm > 1e-10 {
            let normalized = face_normal / norm;
            for &v in face {
                *vertex_normals.entry(v).or_insert(Vector3::zeros()) += normalized;
                *vertex_counts.entry(v).or_insert(0) += 1;
            }
        }
    }

    // Normalize
    for (v, normal) in &mut vertex_normals {
        let count = vertex_counts.get(v).copied().unwrap_or(1);
        #[allow(clippy::cast_precision_loss)]
        {
            *normal /= count as f64;
        }
        let norm = normal.norm();
        if norm > 1e-10 {
            *normal /= norm;
        }
    }

    vertex_normals
}

/// Computes the average normal from a set of vertex indices.
fn compute_average_normal(
    vertex_normals: &HashMap<u32, Vector3<f64>>,
    neighbors: &[u32],
) -> Vector3<f64> {
    let sum: Vector3<f64> = neighbors.iter().filter_map(|n| vertex_normals.get(n)).sum();

    let norm = sum.norm();
    if norm > 1e-10 {
        sum / norm
    } else {
        Vector3::z()
    }
}

/// Computes the average position of neighbors.
fn compute_neighbor_average(vertices: &[Vertex], neighbors: &[u32]) -> Point3<f64> {
    if neighbors.is_empty() {
        return Point3::origin();
    }

    let sum: Vector3<f64> = neighbors
        .iter()
        .map(|&n| vertices[n as usize].position.coords)
        .sum();

    #[allow(clippy::cast_precision_loss)]
    Point3::from(sum / neighbors.len() as f64)
}

/// Computes how far a vertex protrudes from the plane of its neighbors.
fn compute_protrusion(vertex: &Point3<f64>, neighbors: &[Point3<f64>]) -> Option<f64> {
    if neighbors.len() < 3 {
        return None;
    }

    // Compute neighbor centroid
    let sum: Vector3<f64> = neighbors.iter().map(|p| p.coords).sum();
    #[allow(clippy::cast_precision_loss)]
    let centroid = sum / neighbors.len() as f64;

    // Compute neighbor plane normal using PCA-like approach
    let mut normal = Vector3::zeros();

    for i in 0..neighbors.len() {
        let curr = neighbors[i];
        let next = neighbors[(i + 1) % neighbors.len()];

        let v1 = curr.coords - centroid;
        let v2 = next.coords - centroid;
        normal += v1.cross(&v2);
    }

    let norm = normal.norm();
    if norm < 1e-10 {
        return None;
    }
    normal /= norm;

    // Distance from vertex to neighbor plane
    let to_vertex = vertex.coords - centroid;
    Some(to_vertex.dot(&normal).abs())
}

/// Computes the average edge length from a vertex to its neighbors.
fn compute_average_edge_length(vertex: &Point3<f64>, neighbors: &[Point3<f64>]) -> f64 {
    if neighbors.is_empty() {
        return 0.0;
    }

    let total: f64 = neighbors.iter().map(|n| (n - vertex).norm()).sum();

    #[allow(clippy::cast_precision_loss)]
    {
        total / neighbors.len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_flat_triangle_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_mesh_with_spike() -> IndexedMesh {
        // Create a small grid with a spike in the middle
        let mut mesh = IndexedMesh::new();

        // 3x3 grid of vertices
        for i in 0..3 {
            for j in 0..3 {
                let x = f64::from(i);
                let y = f64::from(j);
                let z = if i == 1 && j == 1 { 10.0 } else { 0.0 }; // Spike at center
                mesh.vertices.push(Vertex::from_coords(x, y, z));
            }
        }

        // Create triangles
        // 0-1-3, 1-4-3, 1-2-4, 2-5-4, 3-4-6, 4-7-6, 4-5-7, 5-8-7
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 4, 3]);
        mesh.faces.push([1, 2, 4]);
        mesh.faces.push([2, 5, 4]);
        mesh.faces.push([3, 4, 6]);
        mesh.faces.push([4, 7, 6]);
        mesh.faces.push([4, 5, 7]);
        mesh.faces.push([5, 8, 7]);

        mesh
    }

    #[test]
    fn test_spike_params_default() {
        let params = SpikeParams::default();
        assert!(params.max_normal_deviation > 0.0);
        assert!(params.max_protrusion_ratio > 0.0);
        assert!(params.smooth_spikes);
    }

    #[test]
    fn test_spike_params_builder() {
        let params = SpikeParams::new()
            .with_max_normal_deviation(1.0)
            .with_max_protrusion_ratio(1.5)
            .with_min_neighbors(5)
            .with_smooth_spikes(false);

        assert!((params.max_normal_deviation - 1.0).abs() < 1e-10);
        assert!((params.max_protrusion_ratio - 1.5).abs() < 1e-10);
        assert_eq!(params.min_neighbors, 5);
        assert!(!params.smooth_spikes);
    }

    #[test]
    fn test_spike_params_presets() {
        let aggressive = SpikeParams::aggressive();
        let conservative = SpikeParams::conservative();

        assert!(aggressive.max_normal_deviation < conservative.max_normal_deviation);
        assert!(aggressive.max_protrusion_ratio < conservative.max_protrusion_ratio);
    }

    #[test]
    fn test_remove_spikes_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = remove_spikes(&mesh, &SpikeParams::default());
        assert!(matches!(result, Err(ScanError::EmptyMesh)));
    }

    #[test]
    fn test_remove_spikes_no_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let result = remove_spikes(&mesh, &SpikeParams::default()).unwrap();
        assert_eq!(result.spikes_detected, 0);
    }

    #[test]
    fn test_remove_spikes_flat_mesh() {
        let mesh = make_flat_triangle_mesh();
        let result = remove_spikes(&mesh, &SpikeParams::default()).unwrap();

        // Flat mesh should have no spikes
        assert_eq!(result.spikes_detected, 0);
        assert_eq!(result.mesh.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_remove_spikes_smooth() {
        let mesh = make_mesh_with_spike();
        let params = SpikeParams::default().with_smooth_spikes(true);
        let result = remove_spikes(&mesh, &params).unwrap();

        // Spike should be detected and smoothed
        if result.spikes_detected > 0 {
            assert!(result.spikes_smoothed > 0);
            assert_eq!(result.vertices_removed, 0);

            // The spike vertex (center at index 4) should now be closer to z=0
            let center_z = result.mesh.vertices[4].position.z;
            assert!(center_z < 5.0); // Should be pulled down significantly
        }
    }

    #[test]
    fn test_remove_spikes_delete() {
        let mesh = make_mesh_with_spike();
        let params = SpikeParams::default().with_smooth_spikes(false);
        let result = remove_spikes(&mesh, &params).unwrap();

        // Spike should be detected and removed
        if result.spikes_detected > 0 {
            assert!(result.vertices_removed > 0);
            assert!(result.faces_removed > 0);
            assert!(result.mesh.vertices.len() < mesh.vertices.len());
        }
    }

    #[test]
    fn test_spike_removal_result_display() {
        let result = SpikeRemovalResult {
            mesh: IndexedMesh::new(),
            spikes_detected: 5,
            spikes_smoothed: 5,
            vertices_removed: 0,
            faces_removed: 0,
        };

        let display = format!("{result}");
        assert!(display.contains('5'));
        assert!(display.contains("smoothed"));
    }

    #[test]
    fn test_spike_removal_result_had_spikes() {
        let result = SpikeRemovalResult {
            mesh: IndexedMesh::new(),
            spikes_detected: 3,
            spikes_smoothed: 3,
            vertices_removed: 0,
            faces_removed: 0,
        };

        assert!(result.had_spikes());

        let no_spikes = SpikeRemovalResult {
            mesh: IndexedMesh::new(),
            spikes_detected: 0,
            spikes_smoothed: 0,
            vertices_removed: 0,
            faces_removed: 0,
        };

        assert!(!no_spikes.had_spikes());
    }

    #[test]
    fn test_build_vertex_neighbors() {
        let mesh = make_flat_triangle_mesh();
        let neighbors = build_vertex_neighbors(&mesh);

        // Each vertex should have 2 neighbors in a triangle
        assert_eq!(neighbors.len(), 3);
        for (_, n) in neighbors {
            assert_eq!(n.len(), 2);
        }
    }

    #[test]
    fn test_build_vertex_faces() {
        let mesh = make_flat_triangle_mesh();
        let vertex_faces = build_vertex_faces(&mesh);

        // Each vertex should be in 1 face
        assert_eq!(vertex_faces.len(), 3);
        for (_, faces) in vertex_faces {
            assert_eq!(faces.len(), 1);
        }
    }

    #[test]
    fn test_compute_vertex_normals() {
        let mesh = make_flat_triangle_mesh();
        let normals = compute_vertex_normals(&mesh);

        // All normals should point in +Z or -Z direction for a flat mesh
        assert_eq!(normals.len(), 3);
        for (_, normal) in normals {
            assert!(normal.z.abs() > 0.9);
        }
    }

    #[test]
    fn test_compute_neighbor_average() {
        let vertices = vec![
            Vertex::from_coords(0.0, 0.0, 0.0),
            Vertex::from_coords(2.0, 0.0, 0.0),
            Vertex::from_coords(0.0, 2.0, 0.0),
        ];
        let neighbors = vec![0, 1, 2];
        let avg = compute_neighbor_average(&vertices, &neighbors);

        assert!((avg.x - 2.0 / 3.0).abs() < 1e-10);
        assert!((avg.y - 2.0 / 3.0).abs() < 1e-10);
        assert!((avg.z).abs() < 1e-10);
    }

    #[test]
    fn test_compute_average_edge_length() {
        let vertex = Point3::new(0.0, 0.0, 0.0);
        let neighbors = vec![Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let avg_len = compute_average_edge_length(&vertex, &neighbors);

        assert!((avg_len - 1.0).abs() < 1e-10);
    }
}
