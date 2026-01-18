//! Dijkstra's algorithm for geodesic distance computation.
//!
//! This module implements shortest-path computation on mesh surfaces
//! using Dijkstra's algorithm on the edge graph.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use mesh_types::IndexedMesh;

use crate::adjacency::AdjacencyList;
use crate::distance::DistanceField;

/// Geodesic distance solver using Dijkstra's algorithm.
///
/// Computes shortest-path distances along mesh edges from one or more
/// source vertices to all other vertices.
///
/// # Performance
///
/// Time complexity: O((V + E) log V) where V is vertices and E is edges.
/// Space complexity: O(V + E) for the adjacency list.
///
/// # Example
///
/// ```
/// use mesh_geodesic::GeodesicSolver;
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let solver = GeodesicSolver::new(&mesh);
/// let distances = solver.compute_from_vertex(0);
/// ```
#[derive(Debug, Clone)]
pub struct GeodesicSolver {
    /// Adjacency list for the mesh.
    adjacency: AdjacencyList,
}

/// State for the priority queue in Dijkstra's algorithm.
#[derive(Debug, Clone, Copy)]
struct State {
    /// Current vertex.
    vertex: u32,
    /// Distance from source(s) to this vertex.
    distance: f64,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.vertex == other.vertex && (self.distance - other.distance).abs() < f64::EPSILON
    }
}

impl Eq for State {}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (BinaryHeap is max-heap by default)
        other
            .distance
            .partial_cmp(&self.distance)
            .unwrap_or(Ordering::Equal)
    }
}

impl GeodesicSolver {
    /// Create a new geodesic solver from a mesh.
    ///
    /// Builds the internal adjacency list for efficient distance computation.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh to compute distances on
    #[must_use]
    pub fn new(mesh: &IndexedMesh) -> Self {
        Self {
            adjacency: AdjacencyList::from_mesh(mesh),
        }
    }

    /// Create a solver from a pre-built adjacency list.
    ///
    /// Use this if you already have an adjacency list to avoid rebuilding.
    #[must_use]
    pub const fn from_adjacency(adjacency: AdjacencyList) -> Self {
        Self { adjacency }
    }

    /// Get a reference to the internal adjacency list.
    #[must_use]
    pub const fn adjacency(&self) -> &AdjacencyList {
        &self.adjacency
    }

    /// Get the number of vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.adjacency.vertex_count()
    }

    /// Compute distances from a single source vertex.
    ///
    /// # Arguments
    ///
    /// * `source` - The source vertex index
    ///
    /// # Returns
    ///
    /// A distance field with distances from the source to all vertices.
    /// Unreachable vertices have distance `f64::INFINITY`.
    #[must_use]
    pub fn compute_from_vertex(&self, source: usize) -> DistanceField {
        self.compute_from_vertices(&[source])
    }

    /// Compute distances from multiple source vertices.
    ///
    /// This is useful for computing distance to a set of seed points,
    /// where the distance to a vertex is the minimum distance to any source.
    ///
    /// # Arguments
    ///
    /// * `sources` - Slice of source vertex indices
    ///
    /// # Returns
    ///
    /// A distance field with minimum distances to any source vertex.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: vertex indices are u32, meshes with >4B vertices unsupported
    pub fn compute_from_vertices(&self, sources: &[usize]) -> DistanceField {
        let vertex_count = self.adjacency.vertex_count();
        let mut distances = DistanceField::new(vertex_count);

        if vertex_count == 0 || sources.is_empty() {
            return distances;
        }

        // Priority queue for Dijkstra
        let mut heap = BinaryHeap::with_capacity(vertex_count);

        // Initialize sources
        for &source in sources {
            if source < vertex_count {
                distances.set_distance(source, 0.0);
                heap.push(State {
                    vertex: source as u32,
                    distance: 0.0,
                });
            }
        }

        // Process vertices in order of increasing distance
        while let Some(State { vertex, distance }) = heap.pop() {
            let vertex_idx = vertex as usize;

            // Skip if we've already found a shorter path
            if distance > distances.distance(vertex_idx) {
                continue;
            }

            // Relax edges
            for &(neighbor, edge_length) in self.adjacency.neighbors(vertex_idx) {
                let neighbor_idx = neighbor as usize;
                let new_distance = distance + edge_length;

                // Check if this path is shorter
                if new_distance < distances.distance(neighbor_idx) {
                    distances.set_distance(neighbor_idx, new_distance);
                    heap.push(State {
                        vertex: neighbor,
                        distance: new_distance,
                    });
                }
            }
        }

        distances
    }

    /// Compute distances with a maximum distance cutoff.
    ///
    /// Vertices beyond the cutoff distance are left at infinity,
    /// which can improve performance for localized queries.
    ///
    /// # Arguments
    ///
    /// * `sources` - Source vertex indices
    /// * `max_distance` - Maximum distance to compute
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: vertex indices are u32, meshes with >4B vertices unsupported
    pub fn compute_with_cutoff(&self, sources: &[usize], max_distance: f64) -> DistanceField {
        let vertex_count = self.adjacency.vertex_count();
        let mut distances = DistanceField::new(vertex_count);

        if vertex_count == 0 || sources.is_empty() {
            return distances;
        }

        let mut heap = BinaryHeap::with_capacity(vertex_count);

        // Initialize sources
        for &source in sources {
            if source < vertex_count {
                distances.set_distance(source, 0.0);
                heap.push(State {
                    vertex: source as u32,
                    distance: 0.0,
                });
            }
        }

        while let Some(State { vertex, distance }) = heap.pop() {
            // Stop if beyond cutoff
            if distance > max_distance {
                continue;
            }

            let vertex_idx = vertex as usize;

            // Skip if we've already found a shorter path
            if distance > distances.distance(vertex_idx) {
                continue;
            }

            // Relax edges
            for &(neighbor, edge_length) in self.adjacency.neighbors(vertex_idx) {
                let neighbor_idx = neighbor as usize;
                let new_distance = distance + edge_length;

                // Check if within cutoff and shorter
                if new_distance <= max_distance && new_distance < distances.distance(neighbor_idx) {
                    distances.set_distance(neighbor_idx, new_distance);
                    heap.push(State {
                        vertex: neighbor,
                        distance: new_distance,
                    });
                }
            }
        }

        distances
    }

    /// Find the k nearest vertices to a source.
    ///
    /// # Arguments
    ///
    /// * `source` - Source vertex index
    /// * `k` - Number of nearest vertices to find
    ///
    /// # Returns
    ///
    /// Vector of (vertex index, distance) pairs, sorted by distance.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: vertex indices are u32, meshes with >4B vertices unsupported
    pub fn k_nearest(&self, source: usize, k: usize) -> Vec<(usize, f64)> {
        let vertex_count = self.adjacency.vertex_count();

        if vertex_count == 0 || source >= vertex_count {
            return Vec::new();
        }

        let mut distances = vec![f64::INFINITY; vertex_count];
        let mut heap = BinaryHeap::with_capacity(vertex_count);
        let mut result = Vec::with_capacity(k.min(vertex_count));

        distances[source] = 0.0;
        heap.push(State {
            vertex: source as u32,
            distance: 0.0,
        });

        while let Some(State { vertex, distance }) = heap.pop() {
            let vertex_idx = vertex as usize;

            // Skip if we've already found a shorter path
            if distance > distances[vertex_idx] {
                continue;
            }

            // Add to result
            result.push((vertex_idx, distance));
            if result.len() >= k {
                break;
            }

            // Relax edges
            for &(neighbor, edge_length) in self.adjacency.neighbors(vertex_idx) {
                let neighbor_idx = neighbor as usize;
                let new_distance = distance + edge_length;

                if new_distance < distances[neighbor_idx] {
                    distances[neighbor_idx] = new_distance;
                    heap.push(State {
                        vertex: neighbor,
                        distance: new_distance,
                    });
                }
            }
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_square_mesh() -> IndexedMesh {
        // Square with vertices at corners, two triangles
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0)); // 0
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0)); // 1
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0)); // 3
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh
    }

    #[test]
    fn dijkstra_single_source() {
        let mesh = create_square_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertex(0);

        assert_eq!(distances.reachable_count(), 4);
        assert!(distances.distance(0) < 1e-10);
        assert!((distances.distance(1) - 1.0).abs() < 1e-10);
        // Distance to vertex 2: either via 1 (1 + sqrt(1)) or via 3 (1 + sqrt(1))
        // Direct diagonal is sqrt(2), but that's not an edge
        // Via 1: 1.0 + 1.0 = 2.0, Via 0-2 direct: sqrt(2) ≈ 1.414
    }

    #[test]
    fn dijkstra_multi_source() {
        let mesh = create_square_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertices(&[0, 2]);

        // Both corners are sources
        assert!(distances.distance(0) < 1e-10);
        assert!(distances.distance(2) < 1e-10);
    }

    #[test]
    fn dijkstra_with_cutoff() {
        let mesh = create_square_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_with_cutoff(&[0], 0.5);

        // Only vertex 0 should be reachable (all edges are length 1)
        assert_eq!(distances.reachable_count(), 1);
        assert!(distances.distance(0) < 1e-10);
    }

    #[test]
    fn k_nearest() {
        let mesh = create_square_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let nearest = solver.k_nearest(0, 2);

        assert_eq!(nearest.len(), 2);
        // First should be source itself
        assert_eq!(nearest[0].0, 0);
        assert!(nearest[0].1 < 1e-10);
        // Second should be an adjacent vertex (1 or 3)
        assert!(nearest[1].0 == 1 || nearest[1].0 == 3);
        assert!((nearest[1].1 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn empty_mesh() {
        let mesh = IndexedMesh::new();
        let solver = GeodesicSolver::new(&mesh);

        assert_eq!(solver.vertex_count(), 0);
        let distances = solver.compute_from_vertex(0);
        assert_eq!(distances.len(), 0);
    }

    #[test]
    fn invalid_source() {
        let mesh = create_square_mesh();
        let solver = GeodesicSolver::new(&mesh);

        // Source index out of bounds - should be handled gracefully
        let distances = solver.compute_from_vertices(&[100]);
        assert_eq!(distances.reachable_count(), 0);
    }
}
