//! Connected component analysis for meshes.
//!
//! This module provides tools for detecting and handling disconnected mesh components.
//! A connected component is a set of faces that are connected to each other through
//! shared edges.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_repair::components::{find_connected_components, split_into_components};
//!
//! let mut mesh = IndexedMesh::new();
//! // Add two disconnected triangles
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//! mesh.faces.push([3, 4, 5]);
//!
//! let analysis = find_connected_components(&mesh);
//! assert_eq!(analysis.component_count, 2);
//! ```

use std::cmp::Reverse;

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use tracing::{debug, info};

use crate::adjacency::MeshAdjacency;

/// Result of connected component analysis.
#[derive(Debug, Clone)]
pub struct ComponentAnalysis {
    /// Number of connected components found.
    pub component_count: usize,
    /// Face indices for each component, sorted by component size (largest first).
    pub components: Vec<Vec<u32>>,
    /// Size of the largest component (number of faces).
    pub largest_component_size: usize,
    /// Size of the smallest component (number of faces).
    pub smallest_component_size: usize,
}

impl ComponentAnalysis {
    /// Check if the mesh is fully connected (single component).
    #[must_use]
    pub fn is_connected(&self) -> bool {
        self.component_count == 1
    }

    /// Get the face indices of the largest component.
    #[must_use]
    pub fn largest_component(&self) -> &[u32] {
        self.components.first().map(Vec::as_slice).unwrap_or(&[])
    }
}

impl std::fmt::Display for ComponentAnalysis {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Component Analysis:")?;
        writeln!(f, "  Connected components: {}", self.component_count)?;
        if self.component_count > 0 {
            writeln!(
                f,
                "  Largest component: {} faces",
                self.largest_component_size
            )?;
            writeln!(
                f,
                "  Smallest component: {} faces",
                self.smallest_component_size
            )?;
            if self.component_count > 1 {
                writeln!(f, "  Component sizes:")?;
                for (i, comp) in self.components.iter().enumerate() {
                    writeln!(f, "    Component {}: {} faces", i + 1, comp.len())?;
                }
            }
        }
        Ok(())
    }
}

/// Find all connected components in a mesh.
///
/// Uses a flood-fill algorithm starting from each unvisited face.
/// Two faces are connected if they share an edge.
///
/// # Arguments
///
/// * `mesh` - The mesh to analyze
///
/// # Returns
///
/// A `ComponentAnalysis` containing the component information.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
/// use mesh_repair::components::find_connected_components;
///
/// let mut mesh = IndexedMesh::new();
/// // Add two disconnected triangles
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([3, 4, 5]);
///
/// let analysis = find_connected_components(&mesh);
/// assert_eq!(analysis.component_count, 2);
/// ```
#[must_use]
pub fn find_connected_components(mesh: &IndexedMesh) -> ComponentAnalysis {
    if mesh.faces.is_empty() {
        return ComponentAnalysis {
            component_count: 0,
            components: Vec::new(),
            largest_component_size: 0,
            smallest_component_size: 0,
        };
    }

    let adjacency = MeshAdjacency::build(&mesh.faces);
    let face_count = mesh.faces.len();

    // Build face-to-face adjacency via shared edges
    let mut face_neighbors: Vec<Vec<u32>> = vec![Vec::new(); face_count];
    for (face_idx, face) in mesh.faces.iter().enumerate() {
        // Check all three edges of this face
        for edge_idx in 0..3 {
            let v0 = face[edge_idx];
            let v1 = face[(edge_idx + 1) % 3];

            if let Some(neighbors) = adjacency.faces_for_edge(v0, v1) {
                for &neighbor in neighbors {
                    if neighbor != face_idx {
                        face_neighbors[face_idx].push(neighbor as u32);
                    }
                }
            }
        }
    }

    // Flood-fill to find connected components
    let mut visited = vec![false; face_count];
    let mut components: Vec<Vec<u32>> = Vec::new();

    for start_face in 0..face_count {
        if visited[start_face] {
            continue;
        }

        // BFS flood fill from this face
        let mut component = Vec::new();
        let mut queue = vec![start_face as u32];
        visited[start_face] = true;

        while let Some(face_idx) = queue.pop() {
            component.push(face_idx);

            for &neighbor in &face_neighbors[face_idx as usize] {
                if !visited[neighbor as usize] {
                    visited[neighbor as usize] = true;
                    queue.push(neighbor);
                }
            }
        }

        components.push(component);
    }

    // Sort components by size (largest first)
    components.sort_by_key(|c| Reverse(c.len()));

    let component_count = components.len();
    let largest_component_size = components.first().map(Vec::len).unwrap_or(0);
    let smallest_component_size = components.last().map(Vec::len).unwrap_or(0);

    info!(
        "Found {} connected component(s) in mesh with {} faces",
        component_count, face_count
    );

    if component_count > 1 {
        debug!(
            "Component sizes: {:?}",
            components.iter().map(Vec::len).collect::<Vec<_>>()
        );
    }

    ComponentAnalysis {
        component_count,
        components,
        largest_component_size,
        smallest_component_size,
    }
}

/// Split a mesh into separate meshes, one per connected component.
///
/// Each resulting mesh is independent and has its own vertex and face arrays.
///
/// # Arguments
///
/// * `mesh` - The mesh to split
///
/// # Returns
///
/// A vector of meshes, one per component, sorted by size (largest first).
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
/// use mesh_repair::components::split_into_components;
///
/// let mut mesh = IndexedMesh::new();
/// // Add two disconnected triangles
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([3, 4, 5]);
///
/// let components = split_into_components(&mesh);
/// assert_eq!(components.len(), 2);
/// assert_eq!(components[0].face_count(), 1);
/// assert_eq!(components[1].face_count(), 1);
/// ```
#[must_use]
pub fn split_into_components(mesh: &IndexedMesh) -> Vec<IndexedMesh> {
    let analysis = find_connected_components(mesh);

    if analysis.component_count <= 1 {
        // Return clone of original if single component or empty
        return vec![mesh.clone()];
    }

    info!(
        "Splitting mesh into {} components",
        analysis.component_count
    );

    let mut result = Vec::with_capacity(analysis.component_count);

    for (comp_idx, face_indices) in analysis.components.iter().enumerate() {
        // Collect all vertices used by this component
        let mut used_vertices: HashSet<u32> = HashSet::new();
        for &face_idx in face_indices {
            let face = &mesh.faces[face_idx as usize];
            used_vertices.insert(face[0]);
            used_vertices.insert(face[1]);
            used_vertices.insert(face[2]);
        }

        // Create mapping from old vertex index to new vertex index
        let mut old_to_new: HashMap<u32, u32> = HashMap::new();
        let mut new_vertices = Vec::with_capacity(used_vertices.len());

        for old_idx in used_vertices {
            let new_idx = new_vertices.len() as u32;
            old_to_new.insert(old_idx, new_idx);
            new_vertices.push(mesh.vertices[old_idx as usize].clone());
        }

        // Remap face indices
        let new_faces: Vec<[u32; 3]> = face_indices
            .iter()
            .map(|&face_idx| {
                let face = &mesh.faces[face_idx as usize];
                [
                    *old_to_new.get(&face[0]).unwrap_or(&0),
                    *old_to_new.get(&face[1]).unwrap_or(&0),
                    *old_to_new.get(&face[2]).unwrap_or(&0),
                ]
            })
            .collect();

        debug!(
            "Component {}: {} vertices, {} faces",
            comp_idx + 1,
            new_vertices.len(),
            new_faces.len()
        );

        result.push(IndexedMesh::from_parts(new_vertices, new_faces));
    }

    result
}

/// Keep only the largest connected component in a mesh.
///
/// Modifies the mesh in-place, removing all faces and vertices that don't belong
/// to the largest connected component.
///
/// # Arguments
///
/// * `mesh` - The mesh to modify
///
/// # Returns
///
/// The number of components that were removed.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
/// use mesh_repair::components::keep_largest_component;
///
/// let mut mesh = IndexedMesh::new();
/// // Add two disconnected triangles
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([3, 4, 5]);
///
/// let removed = keep_largest_component(&mut mesh);
/// assert_eq!(removed, 1);
/// assert_eq!(mesh.face_count(), 1);
/// ```
pub fn keep_largest_component(mesh: &mut IndexedMesh) -> usize {
    let analysis = find_connected_components(mesh);

    if analysis.component_count <= 1 {
        return 0;
    }

    let components_removed = analysis.component_count - 1;

    info!(
        "Keeping largest component ({} faces), removing {} smaller component(s)",
        analysis.largest_component_size, components_removed
    );

    // Get the largest component (first one, since sorted by size)
    let largest = split_into_components(mesh).into_iter().next();

    if let Some(largest_mesh) = largest {
        // Replace mesh contents with largest component
        mesh.vertices = largest_mesh.vertices;
        mesh.faces = largest_mesh.faces;
    }

    components_removed
}

/// Keep only components that meet a minimum face count threshold.
///
/// Useful for removing small noise components from scanned meshes.
///
/// # Arguments
///
/// * `mesh` - The mesh to modify
/// * `min_faces` - Minimum number of faces a component must have to be kept
///
/// # Returns
///
/// The number of components that were removed.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
/// use mesh_repair::components::remove_small_components;
///
/// let mut mesh = IndexedMesh::new();
/// // Create mesh with components of different sizes...
/// // (In a real scenario, you'd have multiple triangles per component)
///
/// // Uncomment to test:
/// // let removed = remove_small_components(&mut mesh, 100);
/// ```
pub fn remove_small_components(mesh: &mut IndexedMesh, min_faces: usize) -> usize {
    let analysis = find_connected_components(mesh);

    if analysis.component_count <= 1 {
        if analysis.largest_component_size < min_faces {
            // Single component but too small - clear the mesh
            mesh.vertices.clear();
            mesh.faces.clear();
            return 1;
        }
        return 0;
    }

    // Find components to keep
    let components_to_keep: Vec<&Vec<u32>> = analysis
        .components
        .iter()
        .filter(|c| c.len() >= min_faces)
        .collect();

    if components_to_keep.is_empty() {
        mesh.vertices.clear();
        mesh.faces.clear();
        return analysis.component_count;
    }

    let components_removed = analysis.component_count - components_to_keep.len();

    if components_removed == 0 {
        return 0;
    }

    info!(
        "Removing {} component(s) with fewer than {} faces",
        components_removed, min_faces
    );

    // Collect face indices to keep
    let face_indices_to_keep: HashSet<u32> = components_to_keep
        .iter()
        .flat_map(|c| c.iter().copied())
        .collect();

    // Collect vertices used by kept faces
    let mut used_vertices: HashSet<u32> = HashSet::new();
    for &face_idx in &face_indices_to_keep {
        let face = &mesh.faces[face_idx as usize];
        used_vertices.insert(face[0]);
        used_vertices.insert(face[1]);
        used_vertices.insert(face[2]);
    }

    // Create new vertex array and mapping
    let mut old_to_new: HashMap<u32, u32> = HashMap::new();
    let mut new_vertices = Vec::with_capacity(used_vertices.len());

    for old_idx in 0..mesh.vertices.len() as u32 {
        if used_vertices.contains(&old_idx) {
            let new_idx = new_vertices.len() as u32;
            old_to_new.insert(old_idx, new_idx);
            new_vertices.push(mesh.vertices[old_idx as usize].clone());
        }
    }

    // Create new face array with remapped indices
    let new_faces: Vec<[u32; 3]> = (0..mesh.faces.len())
        .filter(|&i| face_indices_to_keep.contains(&(i as u32)))
        .map(|i| {
            let face = &mesh.faces[i];
            [
                *old_to_new.get(&face[0]).unwrap_or(&0),
                *old_to_new.get(&face[1]).unwrap_or(&0),
                *old_to_new.get(&face[2]).unwrap_or(&0),
            ]
        })
        .collect();

    mesh.vertices = new_vertices;
    mesh.faces = new_faces;

    components_removed
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{MeshTopology, Vertex};

    fn create_single_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn create_two_disconnected_triangles() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // First triangle
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        // Second triangle (disconnected)
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([3, 4, 5]);
        mesh
    }

    fn create_two_connected_triangles() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        // Two triangles sharing edge (1, 2)
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]);
        mesh
    }

    fn create_three_components() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Component 1: 2 triangles
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]);

        // Component 2: 1 triangle
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
        mesh.faces.push([4, 5, 6]);

        // Component 3: 1 triangle
        mesh.vertices.push(Vertex::from_coords(20.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(21.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(20.0, 1.0, 0.0));
        mesh.faces.push([7, 8, 9]);

        mesh
    }

    #[test]
    fn test_empty_mesh() {
        let mesh = IndexedMesh::new();
        let analysis = find_connected_components(&mesh);
        assert_eq!(analysis.component_count, 0);
        assert!(!analysis.is_connected());
    }

    #[test]
    fn test_single_component() {
        let mesh = create_single_triangle();
        let analysis = find_connected_components(&mesh);
        assert_eq!(analysis.component_count, 1);
        assert!(analysis.is_connected());
        assert_eq!(analysis.largest_component_size, 1);
    }

    #[test]
    fn test_two_disconnected_components() {
        let mesh = create_two_disconnected_triangles();
        let analysis = find_connected_components(&mesh);
        assert_eq!(analysis.component_count, 2);
        assert!(!analysis.is_connected());
        assert_eq!(analysis.largest_component_size, 1);
        assert_eq!(analysis.smallest_component_size, 1);
    }

    #[test]
    fn test_connected_triangles() {
        let mesh = create_two_connected_triangles();
        let analysis = find_connected_components(&mesh);
        assert_eq!(analysis.component_count, 1);
        assert!(analysis.is_connected());
        assert_eq!(analysis.largest_component_size, 2);
    }

    #[test]
    fn test_three_components_sorted_by_size() {
        let mesh = create_three_components();
        let analysis = find_connected_components(&mesh);
        assert_eq!(analysis.component_count, 3);
        assert_eq!(analysis.largest_component_size, 2);
        assert_eq!(analysis.smallest_component_size, 1);
        // Largest component should be first
        assert_eq!(analysis.components[0].len(), 2);
    }

    #[test]
    fn test_split_single_component() {
        let mesh = create_single_triangle();
        let components = split_into_components(&mesh);
        assert_eq!(components.len(), 1);
        assert_eq!(components[0].face_count(), 1);
        assert_eq!(components[0].vertex_count(), 3);
    }

    #[test]
    fn test_split_two_components() {
        let mesh = create_two_disconnected_triangles();
        let components = split_into_components(&mesh);
        assert_eq!(components.len(), 2);
        // Each component should have its own vertices (not shared)
        assert_eq!(components[0].vertex_count(), 3);
        assert_eq!(components[1].vertex_count(), 3);
        assert_eq!(components[0].face_count(), 1);
        assert_eq!(components[1].face_count(), 1);
    }

    #[test]
    fn test_split_preserves_geometry() {
        let mesh = create_two_disconnected_triangles();
        let components = split_into_components(&mesh);

        // Check that face indices are valid in each component
        for comp in &components {
            for face in &comp.faces {
                assert!(face[0] < comp.vertices.len() as u32);
                assert!(face[1] < comp.vertices.len() as u32);
                assert!(face[2] < comp.vertices.len() as u32);
            }
        }
    }

    #[test]
    fn test_keep_largest_single_component() {
        let mut mesh = create_single_triangle();
        let removed = keep_largest_component(&mut mesh);
        assert_eq!(removed, 0);
        assert_eq!(mesh.face_count(), 1);
    }

    #[test]
    fn test_keep_largest_multiple_components() {
        let mut mesh = create_three_components();
        let removed = keep_largest_component(&mut mesh);
        assert_eq!(removed, 2);
        assert_eq!(mesh.face_count(), 2); // Kept the component with 2 faces
    }

    #[test]
    fn test_remove_small_components_none_removed() {
        let mut mesh = create_three_components();
        let removed = remove_small_components(&mut mesh, 1);
        assert_eq!(removed, 0);
        assert_eq!(mesh.face_count(), 4); // All components kept
    }

    #[test]
    fn test_remove_small_components_some_removed() {
        let mut mesh = create_three_components();
        let removed = remove_small_components(&mut mesh, 2);
        assert_eq!(removed, 2); // Removed the two 1-face components
        assert_eq!(mesh.face_count(), 2); // Only 2-face component kept
    }

    #[test]
    fn test_remove_small_components_all_removed() {
        let mut mesh = create_three_components();
        let removed = remove_small_components(&mut mesh, 10);
        assert_eq!(removed, 3);
        assert_eq!(mesh.face_count(), 0);
        assert_eq!(mesh.vertex_count(), 0);
    }

    #[test]
    fn test_component_analysis_display() {
        let mesh = create_three_components();
        let analysis = find_connected_components(&mesh);
        let output = format!("{analysis}");
        assert!(output.contains("Connected components: 3"));
        assert!(output.contains("Largest component: 2 faces"));
    }
}
