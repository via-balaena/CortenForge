//! Bilateral mesh filtering for feature-preserving denoising.
//!
//! Bilateral filtering smooths the mesh while preserving sharp edges and
//! features by considering both spatial distance and normal similarity.
//!
//! # Algorithm
//!
//! For each vertex v, the displacement is computed as:
//! ```text
//! displacement = sum_over_neighbors( w_s * w_n * (neighbor - v) ) / sum_of_weights
//! ```
//!
//! where:
//! - `w_s` = exp(-distance^2 / (2 * `sigma_s^2`)) - spatial weight
//! - `w_n` = exp(-normal_diff^2 / (2 * `sigma_n^2`)) - normal weight
//!
//! The normal weight ensures that vertices near sharp edges are less
//! influenced by neighbors on the other side of the edge.
//!
//! # Reference
//!
//! Fleishman, S., Drori, I., & Cohen-Or, D. (2003).
//! "Bilateral mesh denoising"
//! ACM Transactions on Graphics (TOG), 22(3), 950-953.

use mesh_types::IndexedMesh;
use nalgebra::Vector3;
use std::collections::{HashMap, HashSet};

/// Parameters for bilateral filtering.
#[derive(Debug, Clone, Copy)]
pub struct BilateralParams {
    /// Spatial sigma - controls influence based on distance. Default: 1.0.
    /// Larger values allow more distant neighbors to influence the result.
    pub sigma_spatial: f64,

    /// Normal sigma - controls influence based on normal similarity. Default: 0.5.
    /// Smaller values better preserve sharp edges.
    pub sigma_normal: f64,

    /// Number of iterations. Default: 1.
    pub iterations: u32,
}

impl Default for BilateralParams {
    fn default() -> Self {
        Self {
            sigma_spatial: 1.0,
            sigma_normal: 0.5,
            iterations: 1,
        }
    }
}

impl BilateralParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the spatial sigma.
    #[must_use]
    pub const fn with_sigma_spatial(mut self, sigma: f64) -> Self {
        self.sigma_spatial = sigma;
        self
    }

    /// Sets the normal sigma.
    #[must_use]
    pub const fn with_sigma_normal(mut self, sigma: f64) -> Self {
        self.sigma_normal = sigma;
        self
    }

    /// Sets the number of iterations.
    #[must_use]
    pub const fn with_iterations(mut self, iterations: u32) -> Self {
        self.iterations = iterations;
        self
    }

    /// Creates parameters for strong edge preservation.
    #[must_use]
    pub const fn edge_preserving() -> Self {
        Self {
            sigma_spatial: 1.0,
            sigma_normal: 0.2, // Small sigma_normal preserves edges better
            iterations: 2,
        }
    }

    /// Creates parameters for aggressive smoothing.
    #[must_use]
    pub const fn aggressive() -> Self {
        Self {
            sigma_spatial: 2.0,
            sigma_normal: 1.0, // Large sigma_normal allows more smoothing
            iterations: 3,
        }
    }
}

/// Result of bilateral filtering.
#[derive(Debug, Clone)]
pub struct BilateralResult {
    /// The filtered mesh.
    pub mesh: IndexedMesh,

    /// Number of iterations performed.
    pub iterations_performed: u32,

    /// Maximum vertex displacement across all iterations.
    pub max_displacement: f64,

    /// Average vertex displacement.
    pub avg_displacement: f64,
}

/// Applies bilateral mesh filtering.
///
/// # Arguments
///
/// * `mesh` - The mesh to filter
/// * `params` - Filtering parameters
/// * `preserve_boundaries` - If true, boundary vertices are not moved.
///
/// # Returns
///
/// The filtered mesh and statistics.
///
/// # Example
///
/// ```
/// use mesh_scan::denoise::bilateral::{filter_bilateral, BilateralParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// // ... create or load mesh ...
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = filter_bilateral(&mesh, &BilateralParams::default(), false);
/// ```
#[must_use]
pub fn filter_bilateral(
    mesh: &IndexedMesh,
    params: &BilateralParams,
    preserve_boundaries: bool,
) -> BilateralResult {
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return BilateralResult {
            mesh: mesh.clone(),
            iterations_performed: 0,
            max_displacement: 0.0,
            avg_displacement: 0.0,
        };
    }

    // Build adjacency
    let neighbors = build_vertex_neighbors(mesh);
    let boundary_vertices = if preserve_boundaries {
        find_boundary_vertices(mesh)
    } else {
        HashSet::new()
    };

    let mut current = mesh.clone();
    let mut total_max_disp = 0.0_f64;
    let mut total_avg_disp = 0.0;

    for _ in 0..params.iterations {
        // Compute vertex normals
        let vertex_normals = compute_vertex_normals(&current);

        // Compute average edge length for sigma_spatial scaling
        let avg_edge_length = compute_average_edge_length(&current);
        let sigma_s_scaled = params.sigma_spatial * avg_edge_length;

        // Compute displacements in parallel
        let displacements: Vec<(Vector3<f64>, bool)> = current
            .vertices
            .iter()
            .enumerate()
            .map(|(i, vertex)| {
                #[allow(clippy::cast_possible_truncation)]
                let idx = i as u32;

                // Skip boundary vertices
                if boundary_vertices.contains(&idx) {
                    return (Vector3::zeros(), false);
                }

                let vertex_normal = vertex_normals.get(&idx).copied().unwrap_or(Vector3::z());

                if let Some(neighbor_indices) = neighbors.get(&idx) {
                    if neighbor_indices.is_empty() {
                        return (Vector3::zeros(), false);
                    }

                    let mut weighted_sum = Vector3::zeros();
                    let mut weight_sum = 0.0;

                    for &n in neighbor_indices {
                        let neighbor = &current.vertices[n as usize];
                        let neighbor_normal =
                            vertex_normals.get(&n).copied().unwrap_or(Vector3::z());

                        // Spatial weight
                        let dist = (neighbor.position - vertex.position).norm();
                        let w_s = (-dist * dist / (2.0 * sigma_s_scaled * sigma_s_scaled)).exp();

                        // Normal weight
                        let normal_diff = 1.0 - vertex_normal.dot(&neighbor_normal).abs();
                        let w_n = (-normal_diff * normal_diff
                            / (2.0 * params.sigma_normal * params.sigma_normal))
                            .exp();

                        let weight = w_s * w_n;
                        weighted_sum +=
                            (neighbor.position.coords - vertex.position.coords) * weight;
                        weight_sum += weight;
                    }

                    if weight_sum > 1e-10 {
                        let displacement = weighted_sum / weight_sum;
                        (displacement, true)
                    } else {
                        (Vector3::zeros(), false)
                    }
                } else {
                    (Vector3::zeros(), false)
                }
            })
            .collect();

        // Apply displacements and track statistics
        let mut max_disp = 0.0_f64;
        let mut total_disp = 0.0;
        let mut moved_count = 0_usize;

        for (vertex, (displacement, applied)) in
            current.vertices.iter_mut().zip(displacements.iter())
        {
            if *applied {
                let dist = displacement.norm();
                max_disp = max_disp.max(dist);
                total_disp += dist;
                moved_count += 1;
                vertex.position.coords += displacement;
            }
        }

        total_max_disp = total_max_disp.max(max_disp);
        if moved_count > 0 {
            #[allow(clippy::cast_precision_loss)]
            {
                total_avg_disp += total_disp / moved_count as f64;
            }
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let avg_displacement = if params.iterations > 0 {
        total_avg_disp / f64::from(params.iterations)
    } else {
        0.0
    };

    BilateralResult {
        mesh: current,
        iterations_performed: params.iterations,
        max_displacement: total_max_disp,
        avg_displacement,
    }
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

/// Computes the average edge length of the mesh.
fn compute_average_edge_length(mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return 1.0;
    }

    let mut total_length = 0.0;
    let mut edge_count = 0_usize;

    for face in &mesh.faces {
        for i in 0..3 {
            let v0 = &mesh.vertices[face[i] as usize].position;
            let v1 = &mesh.vertices[face[(i + 1) % 3] as usize].position;
            total_length += (v1 - v0).norm();
            edge_count += 1;
        }
    }

    if edge_count > 0 {
        #[allow(clippy::cast_precision_loss)]
        {
            total_length / edge_count as f64
        }
    } else {
        1.0
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

/// Finds all vertices on the mesh boundary.
fn find_boundary_vertices(mesh: &IndexedMesh) -> HashSet<u32> {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    let mut boundary_vertices = HashSet::new();
    for ((v0, v1), count) in edge_counts {
        if count == 1 {
            boundary_vertices.insert(v0);
            boundary_vertices.insert(v1);
        }
    }

    boundary_vertices
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_plane_mesh(n: usize) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        for i in 0..n {
            for j in 0..n {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i as u32),
                    f64::from(j as u32),
                    0.0,
                ));
            }
        }

        for i in 0..(n - 1) {
            for j in 0..(n - 1) {
                #[allow(clippy::cast_possible_truncation)]
                let idx = (i * n + j) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let n_u32 = n as u32;
                mesh.faces.push([idx, idx + 1, idx + n_u32]);
                mesh.faces.push([idx + 1, idx + n_u32 + 1, idx + n_u32]);
            }
        }

        mesh
    }

    fn make_noisy_plane_mesh(n: usize, noise: f64) -> IndexedMesh {
        use rand::Rng;

        let mut mesh = make_plane_mesh(n);
        let mut rng = rand::thread_rng();

        for vertex in &mut mesh.vertices {
            vertex.position.z += rng.gen_range(-noise..noise);
        }

        mesh
    }

    #[test]
    fn test_bilateral_params_default() {
        let params = BilateralParams::default();
        assert_relative_eq!(params.sigma_spatial, 1.0);
        assert_relative_eq!(params.sigma_normal, 0.5);
        assert_eq!(params.iterations, 1);
    }

    #[test]
    fn test_bilateral_params_builder() {
        let params = BilateralParams::new()
            .with_sigma_spatial(2.0)
            .with_sigma_normal(0.3)
            .with_iterations(5);

        assert_relative_eq!(params.sigma_spatial, 2.0);
        assert_relative_eq!(params.sigma_normal, 0.3);
        assert_eq!(params.iterations, 5);
    }

    #[test]
    fn test_bilateral_params_presets() {
        let edge = BilateralParams::edge_preserving();
        let aggressive = BilateralParams::aggressive();

        // Edge preserving should have smaller normal sigma
        assert!(edge.sigma_normal < aggressive.sigma_normal);
    }

    #[test]
    fn test_filter_bilateral_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = filter_bilateral(&mesh, &BilateralParams::default(), false);
        assert!(result.mesh.vertices.is_empty());
        assert_eq!(result.iterations_performed, 0);
    }

    #[test]
    fn test_filter_bilateral_flat_plane() {
        let mesh = make_plane_mesh(5);
        let result = filter_bilateral(&mesh, &BilateralParams::default(), false);

        assert_eq!(result.mesh.vertices.len(), mesh.vertices.len());
        assert_eq!(result.iterations_performed, 1);
    }

    #[test]
    fn test_filter_bilateral_noisy_plane() {
        let mesh = make_noisy_plane_mesh(10, 0.3);

        // Compute initial z variance
        let initial_z_variance: f64 = mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / mesh.vertices.len() as f64;

        let params = BilateralParams::new().with_iterations(3);
        let result = filter_bilateral(&mesh, &params, false);

        // Compute final z variance
        let final_z_variance: f64 = result
            .mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / result.mesh.vertices.len() as f64;

        // Bilateral filtering should reduce variance
        assert!(final_z_variance < initial_z_variance);
    }

    #[test]
    fn test_filter_bilateral_preserve_boundaries() {
        let mesh = make_plane_mesh(5);
        let boundary = find_boundary_vertices(&mesh);

        let result = filter_bilateral(&mesh, &BilateralParams::default(), true);

        for &v in &boundary {
            let original = &mesh.vertices[v as usize].position;
            let filtered = &result.mesh.vertices[v as usize].position;
            assert_relative_eq!(original.x, filtered.x, epsilon = 1e-10);
            assert_relative_eq!(original.y, filtered.y, epsilon = 1e-10);
            assert_relative_eq!(original.z, filtered.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_filter_bilateral_result_stats() {
        let mesh = make_noisy_plane_mesh(10, 0.3);
        let params = BilateralParams::new().with_iterations(2);

        let result = filter_bilateral(&mesh, &params, false);

        assert_eq!(result.iterations_performed, 2);
        assert!(result.max_displacement > 0.0);
        assert!(result.avg_displacement > 0.0);
    }

    #[test]
    fn test_compute_average_edge_length() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let avg_len = compute_average_edge_length(&mesh);
        assert!(avg_len > 0.0);
    }

    #[test]
    fn test_compute_vertex_normals() {
        let mesh = make_plane_mesh(3);
        let normals = compute_vertex_normals(&mesh);

        // All normals should point in +Z or -Z for a flat mesh
        assert!(!normals.is_empty());
        for (_, normal) in normals {
            assert!(normal.z.abs() > 0.9);
        }
    }
}
