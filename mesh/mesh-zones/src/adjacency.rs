//! Face adjacency computation for zone operations.
//!
//! Provides efficient face-to-face adjacency lookup.

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;

/// Face adjacency information.
///
/// Stores which faces share edges, enabling efficient traversal
/// for region growing and zone assignment.
#[derive(Debug, Clone)]
pub struct FaceAdjacency {
    /// For each face, the list of adjacent face indices.
    adjacent: Vec<Vec<usize>>,
}

impl FaceAdjacency {
    /// Build face adjacency from a mesh.
    ///
    /// Two faces are adjacent if they share an edge (two vertices).
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use mesh_zones::FaceAdjacency;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    /// mesh.faces.push([1, 3, 2]);
    ///
    /// let adj = FaceAdjacency::from_mesh(&mesh);
    /// assert_eq!(adj.neighbors(0).len(), 1);
    /// assert!(adj.neighbors(0).contains(&1));
    /// ```
    #[must_use]
    pub fn from_mesh(mesh: &IndexedMesh) -> Self {
        let mut edge_to_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

        // Map each edge to the faces that contain it
        for (face_idx, face) in mesh.faces.iter().enumerate() {
            for i in 0..3 {
                let v0 = face[i];
                let v1 = face[(i + 1) % 3];
                // Normalize edge direction for consistent lookup
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_faces.entry(edge).or_default().push(face_idx);
            }
        }

        // Build adjacency lists
        let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); mesh.faces.len()];

        for faces in edge_to_faces.values() {
            if faces.len() == 2 {
                let f0 = faces[0];
                let f1 = faces[1];
                adjacent[f0].push(f1);
                adjacent[f1].push(f0);
            }
            // Non-manifold edges (more than 2 faces) are ignored for now
        }

        // Remove duplicates (can happen with degenerate meshes)
        for adj_list in &mut adjacent {
            adj_list.sort_unstable();
            adj_list.dedup();
        }

        Self { adjacent }
    }

    /// Get the neighbors of a face.
    ///
    /// Returns an empty slice if the face index is out of bounds.
    #[must_use]
    pub fn neighbors(&self, face_idx: usize) -> &[usize] {
        self.adjacent.get(face_idx).map_or(&[], Vec::as_slice)
    }

    /// Get the number of faces.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.adjacent.len()
    }

    /// Check if two faces are adjacent.
    #[must_use]
    pub fn are_adjacent(&self, face_a: usize, face_b: usize) -> bool {
        if let Some(neighbors) = self.adjacent.get(face_a) {
            neighbors.contains(&face_b)
        } else {
            false
        }
    }

    /// Get all faces reachable from a starting face via adjacency.
    ///
    /// This is a connected component search.
    #[must_use]
    pub fn connected_component(&self, start_face: usize) -> HashSet<usize> {
        let mut visited = HashSet::new();
        let mut stack = vec![start_face];

        while let Some(face) = stack.pop() {
            if visited.contains(&face) {
                continue;
            }
            if face >= self.adjacent.len() {
                continue;
            }
            visited.insert(face);

            for &neighbor in &self.adjacent[face] {
                if !visited.contains(&neighbor) {
                    stack.push(neighbor);
                }
            }
        }

        visited
    }

    /// Find all connected components in the mesh.
    ///
    /// Returns a vector of sets, each containing face indices of one component.
    #[must_use]
    pub fn connected_components(&self) -> Vec<HashSet<usize>> {
        let mut visited: HashSet<usize> = HashSet::new();
        let mut components = Vec::new();

        for face_idx in 0..self.adjacent.len() {
            if visited.contains(&face_idx) {
                continue;
            }

            let component = self.connected_component(face_idx);
            for &idx in &component {
                visited.insert(idx);
            }
            components.push(component);
        }

        components
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn two_triangles() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]);
        mesh
    }

    fn disconnected_triangles() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // Triangle 1
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        // Triangle 2 (not connected)
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([3, 4, 5]);
        mesh
    }

    #[test]
    fn adjacency_basic() {
        let mesh = two_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        assert_eq!(adj.face_count(), 2);
        assert_eq!(adj.neighbors(0).len(), 1);
        assert!(adj.neighbors(0).contains(&1));
        assert!(adj.neighbors(1).contains(&0));
    }

    #[test]
    fn adjacency_out_of_bounds() {
        let mesh = two_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        assert!(adj.neighbors(100).is_empty());
    }

    #[test]
    fn are_adjacent() {
        let mesh = two_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        assert!(adj.are_adjacent(0, 1));
        assert!(adj.are_adjacent(1, 0));
    }

    #[test]
    fn not_adjacent() {
        let mesh = disconnected_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        assert!(!adj.are_adjacent(0, 1));
    }

    #[test]
    fn connected_component_single() {
        let mesh = two_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        let component = adj.connected_component(0);
        assert_eq!(component.len(), 2);
        assert!(component.contains(&0));
        assert!(component.contains(&1));
    }

    #[test]
    fn connected_component_disconnected() {
        let mesh = disconnected_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        let comp0 = adj.connected_component(0);
        let comp1 = adj.connected_component(1);

        assert_eq!(comp0.len(), 1);
        assert_eq!(comp1.len(), 1);
        assert!(comp0.contains(&0));
        assert!(comp1.contains(&1));
    }

    #[test]
    fn connected_components_multiple() {
        let mesh = disconnected_triangles();
        let adj = FaceAdjacency::from_mesh(&mesh);

        let components = adj.connected_components();
        assert_eq!(components.len(), 2);
    }

    #[test]
    fn empty_mesh() {
        let mesh = IndexedMesh::new();
        let adj = FaceAdjacency::from_mesh(&mesh);

        assert_eq!(adj.face_count(), 0);
        assert!(adj.connected_components().is_empty());
    }
}
