//! Mesh adjacency data structures.
//!
//! Provides efficient lookups for edge-to-face and vertex-to-face relationships.

use hashbrown::HashMap;

/// Adjacency information for a mesh.
///
/// Provides efficient lookups for:
/// - Faces adjacent to an edge
/// - Faces adjacent to a vertex
/// - Boundary edges (edges with only one adjacent face)
/// - Non-manifold edges (edges with more than two adjacent faces)
#[derive(Debug, Clone)]
pub struct MeshAdjacency {
    /// Maps edge (v0, v1) to list of face indices. v0 < v1.
    edge_to_faces: HashMap<(u32, u32), Vec<usize>>,
    /// Maps vertex index to list of face indices.
    vertex_to_faces: HashMap<u32, Vec<usize>>,
}

impl MeshAdjacency {
    /// Build adjacency information from a list of faces.
    ///
    /// # Arguments
    ///
    /// * `faces` - Triangle faces as vertex index triplets
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_repair::MeshAdjacency;
    ///
    /// let faces = vec![[0, 1, 2], [1, 3, 2]];
    /// let adj = MeshAdjacency::build(&faces);
    ///
    /// assert_eq!(adj.boundary_edge_count(), 4); // 4 boundary edges
    /// ```
    #[must_use]
    pub fn build(faces: &[[u32; 3]]) -> Self {
        let mut edge_to_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        let mut vertex_to_faces: HashMap<u32, Vec<usize>> = HashMap::new();

        for (face_idx, face) in faces.iter().enumerate() {
            // Add vertex-to-face mappings
            for &v in face {
                vertex_to_faces.entry(v).or_default().push(face_idx);
            }

            // Add edge-to-face mappings (normalize edge direction)
            let edges = [
                normalize_edge(face[0], face[1]),
                normalize_edge(face[1], face[2]),
                normalize_edge(face[2], face[0]),
            ];

            for edge in edges {
                edge_to_faces.entry(edge).or_default().push(face_idx);
            }
        }

        Self {
            edge_to_faces,
            vertex_to_faces,
        }
    }

    /// Get faces adjacent to an edge.
    ///
    /// Returns `None` if the edge doesn't exist in the mesh.
    #[must_use]
    pub fn faces_for_edge(&self, v0: u32, v1: u32) -> Option<&[usize]> {
        let edge = normalize_edge(v0, v1);
        self.edge_to_faces.get(&edge).map(Vec::as_slice)
    }

    /// Get faces adjacent to a vertex.
    ///
    /// Returns an empty slice if the vertex has no adjacent faces.
    #[must_use]
    pub fn faces_for_vertex(&self, v: u32) -> &[usize] {
        self.vertex_to_faces
            .get(&v)
            .map_or(&[], Vec::as_slice)
    }

    /// Iterate over all boundary edges (edges with exactly one adjacent face).
    ///
    /// Boundary edges indicate holes in the mesh surface.
    pub fn boundary_edges(&self) -> impl Iterator<Item = (u32, u32)> + '_ {
        self.edge_to_faces
            .iter()
            .filter(|(_, faces)| faces.len() == 1)
            .map(|(&edge, _)| edge)
    }

    /// Count the number of boundary edges.
    #[must_use]
    pub fn boundary_edge_count(&self) -> usize {
        self.edge_to_faces
            .values()
            .filter(|faces| faces.len() == 1)
            .count()
    }

    /// Iterate over all non-manifold edges (edges with more than two adjacent faces).
    pub fn non_manifold_edges(&self) -> impl Iterator<Item = (u32, u32)> + '_ {
        self.edge_to_faces
            .iter()
            .filter(|(_, faces)| faces.len() > 2)
            .map(|(&edge, _)| edge)
    }

    /// Count the number of non-manifold edges.
    #[must_use]
    pub fn non_manifold_edge_count(&self) -> usize {
        self.edge_to_faces
            .values()
            .filter(|faces| faces.len() > 2)
            .count()
    }

    /// Check if the mesh is manifold (all edges have at most 2 adjacent faces).
    #[must_use]
    pub fn is_manifold(&self) -> bool {
        self.edge_to_faces.values().all(|faces| faces.len() <= 2)
    }

    /// Check if the mesh is watertight (no boundary edges).
    #[must_use]
    pub fn is_watertight(&self) -> bool {
        self.edge_to_faces.values().all(|faces| faces.len() >= 2)
    }

    /// Get the total number of edges.
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.edge_to_faces.len()
    }

    /// Get the number of unique vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertex_to_faces.len()
    }
}

/// Normalize edge direction so v0 < v1.
#[inline]
fn normalize_edge(v0: u32, v1: u32) -> (u32, u32) {
    if v0 < v1 { (v0, v1) } else { (v1, v0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn single_triangle() -> Vec<[u32; 3]> {
        vec![[0, 1, 2]]
    }

    fn two_triangles_sharing_edge() -> Vec<[u32; 3]> {
        vec![[0, 1, 2], [1, 3, 2]]
    }

    fn non_manifold_edge() -> Vec<[u32; 3]> {
        // Three triangles sharing the same edge (0, 1)
        vec![[0, 1, 2], [0, 1, 3], [0, 1, 4]]
    }

    #[test]
    fn build_single_triangle() {
        let faces = single_triangle();
        let adj = MeshAdjacency::build(&faces);

        assert_eq!(adj.edge_count(), 3);
        assert_eq!(adj.vertex_count(), 3);
    }

    #[test]
    fn faces_for_edge() {
        let faces = two_triangles_sharing_edge();
        let adj = MeshAdjacency::build(&faces);

        // Shared edge (1, 2) should have 2 faces
        let shared = adj.faces_for_edge(1, 2);
        assert!(shared.is_some());
        assert_eq!(shared.unwrap().len(), 2);

        // Boundary edge (0, 1) should have 1 face
        let boundary = adj.faces_for_edge(0, 1);
        assert!(boundary.is_some());
        assert_eq!(boundary.unwrap().len(), 1);
    }

    #[test]
    fn faces_for_vertex() {
        let faces = two_triangles_sharing_edge();
        let adj = MeshAdjacency::build(&faces);

        // Vertex 2 is shared by both triangles
        let v2_faces = adj.faces_for_vertex(2);
        assert_eq!(v2_faces.len(), 2);

        // Vertex 0 is only in first triangle
        let v0_faces = adj.faces_for_vertex(0);
        assert_eq!(v0_faces.len(), 1);
    }

    #[test]
    fn boundary_edges_single_triangle() {
        let faces = single_triangle();
        let adj = MeshAdjacency::build(&faces);

        // Single triangle has 3 boundary edges
        assert_eq!(adj.boundary_edge_count(), 3);
        assert!(!adj.is_watertight());
    }

    #[test]
    fn boundary_edges_two_triangles() {
        let faces = two_triangles_sharing_edge();
        let adj = MeshAdjacency::build(&faces);

        // Two triangles share one edge, so 4 boundary edges remain
        assert_eq!(adj.boundary_edge_count(), 4);
    }

    #[test]
    fn non_manifold_detection() {
        let faces = non_manifold_edge();
        let adj = MeshAdjacency::build(&faces);

        // Edge (0, 1) is shared by 3 faces - non-manifold
        assert_eq!(adj.non_manifold_edge_count(), 1);
        assert!(!adj.is_manifold());
    }

    #[test]
    fn manifold_mesh() {
        let faces = two_triangles_sharing_edge();
        let adj = MeshAdjacency::build(&faces);

        assert!(adj.is_manifold());
    }

    #[test]
    fn edge_direction_normalization() {
        let faces = single_triangle();
        let adj = MeshAdjacency::build(&faces);

        // Both directions should find the same edge
        let fwd = adj.faces_for_edge(0, 1);
        let rev = adj.faces_for_edge(1, 0);
        assert_eq!(fwd, rev);
    }

    #[test]
    fn nonexistent_edge() {
        let faces = single_triangle();
        let adj = MeshAdjacency::build(&faces);

        assert!(adj.faces_for_edge(0, 5).is_none());
    }

    #[test]
    fn nonexistent_vertex() {
        let faces = single_triangle();
        let adj = MeshAdjacency::build(&faces);

        assert_eq!(adj.faces_for_vertex(99).len(), 0);
    }
}
