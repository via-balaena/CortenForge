//! Laplacian mesh smoothing.
//!
//! Laplacian smoothing moves each vertex toward the centroid of its neighbors.
//! It's simple and fast but can cause mesh shrinkage with many iterations.
//!
//! # Algorithm
//!
//! For each vertex v with neighbors N(v):
//! ```text
//! v_new = v + lambda * (centroid(N(v)) - v)
//! ```
//!
//! where lambda controls the smoothing strength (typically 0.5).

use std::collections::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use nalgebra::Vector3;


/// Applies one iteration of Laplacian smoothing.
///
/// # Arguments
///
/// * `mesh` - The mesh to smooth
/// * `lambda` - Smoothing factor (0.0-1.0). Higher = more smoothing.
/// * `preserve_boundaries` - If true, boundary vertices are not moved.
///
/// # Returns
///
/// The smoothed mesh and the maximum vertex displacement.
#[must_use] 
pub fn smooth_laplacian(
    mesh: &IndexedMesh,
    lambda: f64,
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

    // Compute new positions in parallel
    let displacements: Vec<(Vector3<f64>, bool)> = mesh
        .vertices
        .iter()
        .enumerate()
        .map(|(i, vertex)| {
            #[allow(clippy::cast_possible_truncation)]
            let idx = i as u32;

            // Skip boundary vertices if preserving
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

                // Displacement toward centroid
                let displacement = (centroid - vertex.position.coords) * lambda;
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

/// Applies multiple iterations of Laplacian smoothing.
///
/// # Arguments
///
/// * `mesh` - The mesh to smooth
/// * `iterations` - Number of smoothing iterations
/// * `lambda` - Smoothing factor per iteration (0.0-1.0)
/// * `preserve_boundaries` - If true, boundary vertices are not moved.
///
/// # Returns
///
/// The smoothed mesh and statistics about the operation.
#[must_use] 
pub fn smooth_laplacian_iterations(
    mesh: &IndexedMesh,
    iterations: u32,
    lambda: f64,
    preserve_boundaries: bool,
) -> LaplacianResult {
    let mut current = mesh.clone();
    let mut total_displacement = 0.0;
    let mut max_displacement = 0.0_f64;

    for _ in 0..iterations {
        let (smoothed, iter_max) = smooth_laplacian(&current, lambda, preserve_boundaries);
        total_displacement += iter_max;
        max_displacement = max_displacement.max(iter_max);
        current = smoothed;
    }

    LaplacianResult {
        mesh: current,
        iterations_performed: iterations,
        total_displacement,
        max_displacement,
    }
}

/// Result of Laplacian smoothing.
#[derive(Debug, Clone)]
pub struct LaplacianResult {
    /// The smoothed mesh.
    pub mesh: IndexedMesh,

    /// Number of iterations performed.
    pub iterations_performed: u32,

    /// Sum of maximum displacements across all iterations.
    pub total_displacement: f64,

    /// Maximum single vertex displacement across all iterations.
    pub max_displacement: f64,
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
    // Count how many times each edge appears
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    // Boundary edges appear exactly once
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
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_plane_mesh(n: usize) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Create n x n grid of vertices
        for i in 0..n {
            for j in 0..n {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i as u32),
                    f64::from(j as u32),
                    0.0,
                ));
            }
        }

        // Create triangles
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
    fn test_smooth_laplacian_empty_mesh() {
        let mesh = IndexedMesh::new();
        let (result, max_disp) = smooth_laplacian(&mesh, 0.5, false);
        assert!(result.vertices.is_empty());
        assert_relative_eq!(max_disp, 0.0);
    }

    #[test]
    fn test_smooth_laplacian_single_triangle() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let (result, _) = smooth_laplacian(&mesh, 0.5, false);

        // Vertices should move toward triangle centroid
        assert_eq!(result.vertices.len(), 3);
    }

    #[test]
    fn test_smooth_laplacian_flat_plane() {
        let mesh = make_plane_mesh(5);
        let (result, max_disp) = smooth_laplacian(&mesh, 0.5, false);

        // Flat plane should not change much
        assert!(max_disp < 1.0);
        assert_eq!(result.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_smooth_laplacian_noisy_plane() {
        let mesh = make_noisy_plane_mesh(10, 0.5);

        // Compute initial z variance
        let initial_z_variance: f64 = mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / mesh.vertices.len() as f64;

        let result = smooth_laplacian_iterations(&mesh, 10, 0.5, false);

        // Compute final z variance
        let final_z_variance: f64 = result
            .mesh
            .vertices
            .iter()
            .map(|v| v.position.z.powi(2))
            .sum::<f64>()
            / result.mesh.vertices.len() as f64;

        // Smoothing should reduce variance
        assert!(final_z_variance < initial_z_variance);
    }

    #[test]
    fn test_smooth_laplacian_preserve_boundaries() {
        let mesh = make_plane_mesh(5);

        // Find boundary vertices before
        let boundary = find_boundary_vertices(&mesh);

        let (result, _) = smooth_laplacian(&mesh, 0.5, true);

        // Boundary vertices should not have moved
        for &v in &boundary {
            let original = &mesh.vertices[v as usize].position;
            let smoothed = &result.vertices[v as usize].position;
            assert_relative_eq!(original.x, smoothed.x, epsilon = 1e-10);
            assert_relative_eq!(original.y, smoothed.y, epsilon = 1e-10);
            assert_relative_eq!(original.z, smoothed.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_smooth_laplacian_iterations() {
        let mesh = make_noisy_plane_mesh(10, 0.3);

        let result = smooth_laplacian_iterations(&mesh, 5, 0.5, false);

        assert_eq!(result.iterations_performed, 5);
        assert!(result.total_displacement > 0.0);
        assert!(result.max_displacement > 0.0);
    }

    #[test]
    fn test_smooth_laplacian_zero_lambda() {
        let mesh = make_noisy_plane_mesh(5, 0.5);
        let (result, max_disp) = smooth_laplacian(&mesh, 0.0, false);

        // With lambda=0, mesh should not change
        assert_relative_eq!(max_disp, 0.0);
        for (orig, smoothed) in mesh.vertices.iter().zip(result.vertices.iter()) {
            assert_relative_eq!(orig.position.x, smoothed.position.x, epsilon = 1e-10);
            assert_relative_eq!(orig.position.y, smoothed.position.y, epsilon = 1e-10);
            assert_relative_eq!(orig.position.z, smoothed.position.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_find_boundary_vertices() {
        let mesh = make_plane_mesh(3);
        let boundary = find_boundary_vertices(&mesh);

        // 3x3 grid has 8 boundary vertices (all edges minus 1 interior)
        assert!(!boundary.is_empty());
        // Interior vertex (at index 4) should not be on boundary
        assert!(!boundary.contains(&4));
    }

    #[test]
    fn test_build_vertex_neighbors() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let neighbors = build_vertex_neighbors(&mesh);

        // Each vertex should have 2 neighbors
        assert_eq!(neighbors.len(), 3);
        for (_, n) in neighbors {
            assert_eq!(n.len(), 2);
        }
    }
}
