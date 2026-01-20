//! Taubin mesh smoothing (shrink-free).
//!
//! Taubin smoothing alternates between smoothing and inflating steps to
//! prevent the mesh shrinkage that occurs with pure Laplacian smoothing.
//!
//! # Algorithm
//!
//! Each iteration consists of two passes:
//! 1. Smoothing pass with positive lambda (shrinks mesh)
//! 2. Inflation pass with negative mu (expands mesh)
//!
//! The parameters are chosen so that shrinkage cancels out:
//! - lambda = 0.5 (smoothing)
//! - mu = -0.53 (inflation, slightly larger magnitude)
//!
//! # Reference
//!
//! Taubin, G. (1995). "A signal processing approach to fair surface design"
//! Proceedings of SIGGRAPH 1995.

use mesh_types::IndexedMesh;
use nalgebra::Vector3;
use std::collections::{HashMap, HashSet};

/// Applies one iteration of Taubin smoothing.
///
/// # Arguments
///
/// * `mesh` - The mesh to smooth
/// * `lambda` - Smoothing factor (positive, typically 0.5)
/// * `mu` - Inflation factor (negative, typically -0.53)
/// * `preserve_boundaries` - If true, boundary vertices are not moved.
///
/// # Returns
///
/// The smoothed mesh and the maximum vertex displacement.
#[must_use]
pub fn smooth_taubin(
    mesh: &IndexedMesh,
    lambda: f64,
    mu: f64,
    preserve_boundaries: bool,
) -> (IndexedMesh, f64) {
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return (mesh.clone(), 0.0);
    }

    // Build adjacency
    let neighbors = build_vertex_neighbors(mesh);
    let boundary_vertices = if preserve_boundaries {
        find_boundary_vertices(mesh)
    } else {
        HashSet::new()
    };

    // Pass 1: Smoothing (lambda > 0)
    let (smoothed, disp1) = apply_laplacian_step(mesh, &neighbors, &boundary_vertices, lambda);

    // Pass 2: Inflation (mu < 0)
    let (result, disp2) = apply_laplacian_step(&smoothed, &neighbors, &boundary_vertices, mu);

    (result, disp1.max(disp2))
}

/// Applies multiple iterations of Taubin smoothing.
///
/// # Arguments
///
/// * `mesh` - The mesh to smooth
/// * `iterations` - Number of smoothing iterations
/// * `lambda` - Smoothing factor (positive, typically 0.5)
/// * `mu` - Inflation factor (negative, typically -0.53)
/// * `preserve_boundaries` - If true, boundary vertices are not moved.
///
/// # Returns
///
/// The smoothed mesh and statistics about the operation.
#[must_use]
pub fn smooth_taubin_iterations(
    mesh: &IndexedMesh,
    iterations: u32,
    lambda: f64,
    mu: f64,
    preserve_boundaries: bool,
) -> TaubinResult {
    let mut current = mesh.clone();
    let mut total_displacement = 0.0;
    let mut max_displacement = 0.0_f64;

    for _ in 0..iterations {
        let (smoothed, iter_max) = smooth_taubin(&current, lambda, mu, preserve_boundaries);
        total_displacement += iter_max;
        max_displacement = max_displacement.max(iter_max);
        current = smoothed;
    }

    TaubinResult {
        mesh: current,
        iterations_performed: iterations,
        total_displacement,
        max_displacement,
    }
}

/// Result of Taubin smoothing.
#[derive(Debug, Clone)]
pub struct TaubinResult {
    /// The smoothed mesh.
    pub mesh: IndexedMesh,

    /// Number of iterations performed.
    pub iterations_performed: u32,

    /// Sum of maximum displacements across all iterations.
    pub total_displacement: f64,

    /// Maximum single vertex displacement across all iterations.
    pub max_displacement: f64,
}

/// Default Taubin parameters.
#[derive(Debug, Clone, Copy)]
pub struct TaubinParams {
    /// Smoothing factor (positive). Default: 0.5.
    pub lambda: f64,

    /// Inflation factor (negative). Default: -0.53.
    pub mu: f64,
}

impl Default for TaubinParams {
    fn default() -> Self {
        Self {
            lambda: 0.5,
            mu: -0.53,
        }
    }
}

impl TaubinParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the smoothing factor (lambda).
    #[must_use]
    pub const fn with_lambda(mut self, lambda: f64) -> Self {
        self.lambda = lambda;
        self
    }

    /// Sets the inflation factor (mu).
    #[must_use]
    pub const fn with_mu(mut self, mu: f64) -> Self {
        self.mu = mu;
        self
    }

    /// Creates parameters for stronger smoothing.
    #[must_use]
    pub const fn strong() -> Self {
        Self {
            lambda: 0.6,
            mu: -0.64,
        }
    }

    /// Creates parameters for gentle smoothing.
    #[must_use]
    pub const fn gentle() -> Self {
        Self {
            lambda: 0.3,
            mu: -0.31,
        }
    }
}

/// Applies a single Laplacian step (used internally by Taubin).
fn apply_laplacian_step(
    mesh: &IndexedMesh,
    neighbors: &HashMap<u32, Vec<u32>>,
    boundary_vertices: &HashSet<u32>,
    factor: f64,
) -> (IndexedMesh, f64) {
    // Compute new positions in parallel
    let displacements: Vec<(Vector3<f64>, bool)> = mesh
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

            if let Some(neighbor_indices) = neighbors.get(&idx) {
                if neighbor_indices.is_empty() {
                    return (Vector3::zeros(), false);
                }

                // Compute centroid of neighbors
                let sum: Vector3<f64> = neighbor_indices
                    .iter()
                    .map(|&n| mesh.vertices[n as usize].position.coords)
                    .sum();

                #[allow(clippy::cast_precision_loss)]
                let centroid = sum / neighbor_indices.len() as f64;

                // Displacement toward/away from centroid
                let displacement = (centroid - vertex.position.coords) * factor;
                (displacement, true)
            } else {
                (Vector3::zeros(), false)
            }
        })
        .collect();

    // Apply displacements and track maximum
    let mut result = mesh.clone();
    let mut max_displacement = 0.0_f64;

    for (vertex, (displacement, applied)) in result.vertices.iter_mut().zip(displacements.iter()) {
        if *applied {
            let dist = displacement.norm();
            max_displacement = max_displacement.max(dist);
            vertex.position.coords += displacement;
        }
    }

    (result, max_displacement)
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

    fn compute_mesh_volume_approx(mesh: &IndexedMesh) -> f64 {
        // Compute bounding box volume as a proxy for mesh volume
        if mesh.vertices.is_empty() {
            return 0.0;
        }

        let mut min = mesh.vertices[0].position;
        let mut max = mesh.vertices[0].position;

        for v in &mesh.vertices[1..] {
            min.x = min.x.min(v.position.x);
            min.y = min.y.min(v.position.y);
            min.z = min.z.min(v.position.z);
            max.x = max.x.max(v.position.x);
            max.y = max.y.max(v.position.y);
            max.z = max.z.max(v.position.z);
        }

        (max.x - min.x) * (max.y - min.y) * (max.z - min.z).max(0.001)
    }

    #[test]
    fn test_smooth_taubin_empty_mesh() {
        let mesh = IndexedMesh::new();
        let params = TaubinParams::default();
        let (result, max_disp) = smooth_taubin(&mesh, params.lambda, params.mu, false);
        assert!(result.vertices.is_empty());
        assert_relative_eq!(max_disp, 0.0);
    }

    #[test]
    fn test_smooth_taubin_flat_plane() {
        let mesh = make_plane_mesh(5);
        let params = TaubinParams::default();
        let (result, _) = smooth_taubin(&mesh, params.lambda, params.mu, false);

        assert_eq!(result.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_smooth_taubin_noisy_plane() {
        let mesh = make_noisy_plane_mesh(10, 0.5);
        let params = TaubinParams::default();

        // Compute initial z variance
        let initial_z_variance: f64 = mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / mesh.vertices.len() as f64;

        let result = smooth_taubin_iterations(&mesh, 10, params.lambda, params.mu, false);

        // Compute final z variance
        let final_z_variance: f64 = result
            .mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / result.mesh.vertices.len() as f64;

        // Taubin smoothing should reduce variance
        assert!(final_z_variance < initial_z_variance);
    }

    #[test]
    fn test_smooth_taubin_preserves_volume() {
        let mesh = make_plane_mesh(10);
        let params = TaubinParams::default();

        let initial_volume = compute_mesh_volume_approx(&mesh);
        let result = smooth_taubin_iterations(&mesh, 5, params.lambda, params.mu, false);
        let final_volume = compute_mesh_volume_approx(&result.mesh);

        // Taubin should approximately preserve volume (within 20%)
        let ratio = final_volume / initial_volume;
        assert!(ratio > 0.8 && ratio < 1.2, "Volume ratio: {ratio}");
    }

    #[test]
    fn test_smooth_taubin_preserve_boundaries() {
        let mesh = make_plane_mesh(5);
        let params = TaubinParams::default();
        let boundary = find_boundary_vertices(&mesh);

        let (result, _) = smooth_taubin(&mesh, params.lambda, params.mu, true);

        for &v in &boundary {
            let original = &mesh.vertices[v as usize].position;
            let smoothed = &result.vertices[v as usize].position;
            assert_relative_eq!(original.x, smoothed.x, epsilon = 1e-10);
            assert_relative_eq!(original.y, smoothed.y, epsilon = 1e-10);
            assert_relative_eq!(original.z, smoothed.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_smooth_taubin_iterations() {
        let mesh = make_noisy_plane_mesh(10, 0.3);
        let params = TaubinParams::default();

        let result = smooth_taubin_iterations(&mesh, 5, params.lambda, params.mu, false);

        assert_eq!(result.iterations_performed, 5);
        assert!(result.total_displacement > 0.0);
    }

    #[test]
    fn test_taubin_params_default() {
        let params = TaubinParams::default();
        assert_relative_eq!(params.lambda, 0.5);
        assert_relative_eq!(params.mu, -0.53);
    }

    #[test]
    fn test_taubin_params_builder() {
        let params = TaubinParams::new().with_lambda(0.4).with_mu(-0.45);

        assert_relative_eq!(params.lambda, 0.4);
        assert_relative_eq!(params.mu, -0.45);
    }

    #[test]
    fn test_taubin_params_presets() {
        let strong = TaubinParams::strong();
        let gentle = TaubinParams::gentle();

        assert!(strong.lambda > gentle.lambda);
        assert!(strong.mu.abs() > gentle.mu.abs());
    }
}
