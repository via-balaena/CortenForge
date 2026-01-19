//! Mesh adjacency data structure.
//!
//! Provides efficient neighbor lookup for mesh vertices.

use mesh_types::IndexedMesh;

/// Adjacency list for mesh vertices.
///
/// Stores neighbors for each vertex, enabling efficient graph traversal.
/// Each neighbor entry includes the neighbor index and edge length.
#[derive(Debug, Clone)]
pub struct AdjacencyList {
    /// For each vertex, list of (neighbor index, edge length) pairs.
    neighbors: Vec<Vec<(u32, f64)>>,
}

impl AdjacencyList {
    /// Build an adjacency list from a mesh.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh to build adjacency from
    ///
    /// # Returns
    ///
    /// An adjacency list where each vertex has a list of its neighbors
    /// with corresponding edge lengths.
    #[must_use]
    pub fn from_mesh(mesh: &IndexedMesh) -> Self {
        let vertex_count = mesh.vertices.len();
        let mut neighbors: Vec<Vec<(u32, f64)>> = vec![Vec::new(); vertex_count];

        // Process each face
        for &[i0, i1, i2] in &mesh.faces {
            let v0 = &mesh.vertices[i0 as usize].position;
            let v1 = &mesh.vertices[i1 as usize].position;
            let v2 = &mesh.vertices[i2 as usize].position;

            // Compute edge lengths
            let len01 = (v1 - v0).norm();
            let len12 = (v2 - v1).norm();
            let len20 = (v0 - v2).norm();

            // Add edges (bidirectional)
            Self::add_edge(&mut neighbors, i0, i1, len01);
            Self::add_edge(&mut neighbors, i1, i2, len12);
            Self::add_edge(&mut neighbors, i2, i0, len20);
        }

        Self { neighbors }
    }

    /// Add an edge between two vertices (if not already present).
    fn add_edge(neighbors: &mut [Vec<(u32, f64)>], v0: u32, v1: u32, length: f64) {
        // Add v1 to v0's neighbors (if not already there)
        if !neighbors[v0 as usize].iter().any(|&(n, _)| n == v1) {
            neighbors[v0 as usize].push((v1, length));
        }

        // Add v0 to v1's neighbors (if not already there)
        if !neighbors[v1 as usize].iter().any(|&(n, _)| n == v0) {
            neighbors[v1 as usize].push((v0, length));
        }
    }

    /// Get the number of vertices.
    #[inline]
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.neighbors.len()
    }

    /// Get the neighbors of a vertex.
    ///
    /// Returns a slice of (neighbor index, edge length) pairs.
    #[inline]
    #[must_use]
    pub fn neighbors(&self, vertex: usize) -> &[(u32, f64)] {
        self.neighbors.get(vertex).map_or(&[], |v| v.as_slice())
    }

    /// Check if the adjacency list is empty.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.neighbors.is_empty()
    }

    /// Get the total number of edges.
    #[must_use]
    pub fn edge_count(&self) -> usize {
        // Each edge is stored twice (once for each direction)
        self.neighbors.iter().map(Vec::len).sum::<usize>() / 2
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::map_unwrap_or
)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    #[test]
    fn adjacency_from_triangle() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let adj = AdjacencyList::from_mesh(&mesh);

        assert_eq!(adj.vertex_count(), 3);
        assert_eq!(adj.edge_count(), 3);

        // Each vertex should have 2 neighbors
        assert_eq!(adj.neighbors(0).len(), 2);
        assert_eq!(adj.neighbors(1).len(), 2);
        assert_eq!(adj.neighbors(2).len(), 2);
    }

    #[test]
    fn adjacency_edge_lengths() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(3.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 4.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let adj = AdjacencyList::from_mesh(&mesh);

        // Check edge lengths
        let neighbors_0 = adj.neighbors(0);

        // Find edge to vertex 1 (should be length 3)
        let edge_01 = neighbors_0.iter().find(|&&(n, _)| n == 1);
        assert!(edge_01.is_some());
        assert!((edge_01.map(|e| e.1).unwrap_or(0.0) - 3.0).abs() < 1e-10);

        // Find edge to vertex 2 (should be length 4)
        let edge_02 = neighbors_0.iter().find(|&&(n, _)| n == 2);
        assert!(edge_02.is_some());
        assert!((edge_02.map(|e| e.1).unwrap_or(0.0) - 4.0).abs() < 1e-10);
    }

    #[test]
    fn adjacency_empty_mesh() {
        let mesh = IndexedMesh::new();
        let adj = AdjacencyList::from_mesh(&mesh);

        assert!(adj.is_empty());
        assert_eq!(adj.vertex_count(), 0);
        assert_eq!(adj.edge_count(), 0);
    }

    #[test]
    fn adjacency_shared_edge() {
        // Two triangles sharing an edge
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 1]);

        let adj = AdjacencyList::from_mesh(&mesh);

        assert_eq!(adj.vertex_count(), 4);
        // Edges: 0-1, 1-2, 2-0, 0-3, 3-1 = 5 edges
        assert_eq!(adj.edge_count(), 5);

        // Vertex 0 should have 3 neighbors (1, 2, 3)
        assert_eq!(adj.neighbors(0).len(), 3);
    }
}
