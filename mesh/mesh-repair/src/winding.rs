//! Normal consistency and winding order correction.
//!
//! This module provides tools for fixing inconsistent face winding in meshes.
//! Consistent winding is required for correct normal computation and rendering.
//!
//! # Winding Convention
//!
//! CortenForge uses **counter-clockwise (CCW) winding** when viewed from the front.
//! Normals point outward by the right-hand rule.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_repair::winding::fix_winding_order;
//!
//! let mut mesh = IndexedMesh::new();
//! // ... add vertices and faces ...
//!
//! // Fix inconsistent winding
//! fix_winding_order(&mut mesh).unwrap();
//! ```

use hashbrown::HashSet;
use mesh_types::IndexedMesh;
use std::collections::VecDeque;
use tracing::{debug, info};

use crate::adjacency::MeshAdjacency;
use crate::error::RepairResult;

/// Fix winding order so all faces have consistent orientation.
///
/// Uses BFS flood fill from an arbitrary start face in each connected component.
/// For each face, ensures that shared edges are traversed in opposite directions.
///
/// This function handles disconnected meshes by processing each component separately.
///
/// # Arguments
///
/// * `mesh` - The mesh to repair
///
/// # Returns
///
/// Ok(()) if successful.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::winding::fix_winding_order;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));
///
/// // Two triangles with inconsistent winding
/// mesh.faces.push([0, 1, 2]); // CCW
/// mesh.faces.push([0, 1, 3]); // Wrong: same direction as first on shared edge
///
/// fix_winding_order(&mut mesh).unwrap();
/// // Now both triangles have consistent winding
/// ```
pub fn fix_winding_order(mesh: &mut IndexedMesh) -> RepairResult<()> {
    if mesh.faces.is_empty() {
        return Ok(());
    }

    let adjacency = MeshAdjacency::build(&mesh.faces);
    let face_count = mesh.faces.len();

    // Track which faces have been visited globally
    let mut global_visited: HashSet<u32> = HashSet::new();
    let mut to_flip: HashSet<u32> = HashSet::new();
    let mut component_count = 0;
    let mut total_flipped = 0;

    // Process all faces, starting new components as needed
    for start_face in 0..face_count {
        let start_face = start_face as u32;

        // Skip already visited faces
        if global_visited.contains(&start_face) {
            continue;
        }

        // Start a new component
        component_count += 1;
        let mut component_flips: HashSet<u32> = HashSet::new();
        let mut queue: VecDeque<u32> = VecDeque::new();

        queue.push_back(start_face);
        global_visited.insert(start_face);

        while let Some(face_idx) = queue.pop_front() {
            let face = mesh.faces[face_idx as usize];

            // Check all three edges of this face
            for edge_idx in 0..3 {
                let v0 = face[edge_idx];
                let v1 = face[(edge_idx + 1) % 3];

                // Find neighbor faces sharing this edge
                if let Some(neighbors) = adjacency.faces_for_edge(v0, v1) {
                    for &neighbor_idx in neighbors {
                        let neighbor_idx = neighbor_idx as u32;
                        if neighbor_idx == face_idx {
                            continue;
                        }

                        if global_visited.contains(&neighbor_idx) {
                            continue;
                        }

                        global_visited.insert(neighbor_idx);

                        // Check edge direction in neighbor
                        let neighbor_face = mesh.faces[neighbor_idx as usize];
                        let neighbor_dir = edge_direction_in_face(&neighbor_face, v0, v1);

                        // Current face traverses edge as v0 -> v1
                        // For consistent winding, neighbor should traverse as v1 -> v0
                        // (opposite direction on the shared edge)
                        // If neighbor has same direction, one of them needs flipping
                        // Since current face is "correct", flip the neighbor
                        // (Edge not found shouldn't happen, defaults to no flip)
                        let should_flip = neighbor_dir.unwrap_or(false);

                        let actual_flip = if component_flips.contains(&face_idx) {
                            // Current face was itself flipped, so invert the decision
                            !should_flip
                        } else {
                            should_flip
                        };

                        if actual_flip {
                            component_flips.insert(neighbor_idx);
                        }

                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        // Add this component's flips to the global set
        total_flipped += component_flips.len();
        to_flip.extend(component_flips);
    }

    // Apply flips (swap indices 1 and 2)
    for &face_idx in &to_flip {
        let face = &mut mesh.faces[face_idx as usize];
        face.swap(1, 2);
    }

    if total_flipped > 0 {
        info!(
            "Fixed winding order: flipped {} faces across {} component(s)",
            total_flipped, component_count
        );
    } else {
        debug!(
            "Winding order already consistent across {} component(s)",
            component_count
        );
    }

    Ok(())
}

/// Check if edge (a, b) appears in face in the same direction (a -> b).
/// Returns `Some(true)` if same direction, `Some(false)` if opposite, `None` if edge not found.
fn edge_direction_in_face(face: &[u32; 3], a: u32, b: u32) -> Option<bool> {
    for i in 0..3 {
        let v0 = face[i];
        let v1 = face[(i + 1) % 3];

        if v0 == a && v1 == b {
            return Some(true); // Same direction
        }
        if v0 == b && v1 == a {
            return Some(false); // Opposite direction
        }
    }
    None
}

/// Count the number of faces that would need to be flipped.
///
/// This is a non-mutating version that just reports how many faces
/// have inconsistent winding.
///
/// # Arguments
///
/// * `mesh` - The mesh to analyze
///
/// # Returns
///
/// The number of faces that would be flipped by `fix_winding_order`.
#[must_use]
pub fn count_inconsistent_faces(mesh: &IndexedMesh) -> usize {
    if mesh.faces.is_empty() {
        return 0;
    }

    let adjacency = MeshAdjacency::build(&mesh.faces);
    let face_count = mesh.faces.len();

    let mut global_visited: HashSet<u32> = HashSet::new();
    let mut total_flipped = 0;

    for start_face in 0..face_count {
        let start_face = start_face as u32;

        if global_visited.contains(&start_face) {
            continue;
        }

        let mut component_flips: HashSet<u32> = HashSet::new();
        let mut queue: VecDeque<u32> = VecDeque::new();

        queue.push_back(start_face);
        global_visited.insert(start_face);

        while let Some(face_idx) = queue.pop_front() {
            let face = mesh.faces[face_idx as usize];

            for edge_idx in 0..3 {
                let v0 = face[edge_idx];
                let v1 = face[(edge_idx + 1) % 3];

                if let Some(neighbors) = adjacency.faces_for_edge(v0, v1) {
                    for &neighbor_idx in neighbors {
                        let neighbor_idx = neighbor_idx as u32;
                        if neighbor_idx == face_idx || global_visited.contains(&neighbor_idx) {
                            continue;
                        }

                        global_visited.insert(neighbor_idx);

                        let neighbor_face = mesh.faces[neighbor_idx as usize];
                        let neighbor_dir = edge_direction_in_face(&neighbor_face, v0, v1);
                        let should_flip = neighbor_dir.unwrap_or(false);

                        let actual_flip = if component_flips.contains(&face_idx) {
                            !should_flip
                        } else {
                            should_flip
                        };

                        if actual_flip {
                            component_flips.insert(neighbor_idx);
                        }

                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        total_flipped += component_flips.len();
    }

    total_flipped
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    #[test]
    fn test_already_consistent() {
        // Tetrahedron with consistent winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));

        // All faces with outward normals (CCW when viewed from outside)
        mesh.faces.push([0, 1, 2]); // Bottom
        mesh.faces.push([0, 3, 1]); // Front
        mesh.faces.push([1, 3, 2]); // Right
        mesh.faces.push([2, 3, 0]); // Left

        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
        // May or may not flip depending on starting face, but should be consistent
    }

    #[test]
    fn test_fix_inconsistent() {
        // Two triangles sharing an edge, one with wrong winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));

        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong: should be [1, 0, 3] for consistent winding

        fix_winding_order(&mut mesh).unwrap();

        // Check that edge (0,1) is now traversed in opposite directions
        let f0 = mesh.faces[0];
        let f1 = mesh.faces[1];

        let dir0 = edge_direction_in_face(&f0, 0, 1);
        let dir1 = edge_direction_in_face(&f1, 0, 1);

        // They should be opposite
        match (dir0, dir1) {
            (Some(d0), Some(d1)) => assert_ne!(d0, d1),
            _ => panic!("Edge should exist in both faces"),
        }
    }

    #[test]
    fn test_fix_disconnected_components() {
        // Two disconnected components, each with inconsistent winding
        let mut mesh = IndexedMesh::new();

        // Component 1: Two triangles sharing edge (0,1)
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));
        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong winding

        // Component 2: Two triangles sharing edge (4,5), disconnected from component 1
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.5, -1.0, 0.0));
        mesh.faces.push([4, 5, 6]); // CCW
        mesh.faces.push([4, 5, 7]); // Wrong winding

        fix_winding_order(&mut mesh).unwrap();

        // Check both components have consistent winding
        // Component 1: edge (0,1) should be opposite in faces 0 and 1
        let f0 = mesh.faces[0];
        let f1 = mesh.faces[1];
        let dir0 = edge_direction_in_face(&f0, 0, 1);
        let dir1 = edge_direction_in_face(&f1, 0, 1);
        match (dir0, dir1) {
            (Some(d0), Some(d1)) => assert_ne!(d0, d1, "Component 1 winding inconsistent"),
            _ => panic!("Edge should exist in both faces of component 1"),
        }

        // Component 2: edge (4,5) should be opposite in faces 2 and 3
        let f2 = mesh.faces[2];
        let f3 = mesh.faces[3];
        let dir2 = edge_direction_in_face(&f2, 4, 5);
        let dir3 = edge_direction_in_face(&f3, 4, 5);
        match (dir2, dir3) {
            (Some(d2), Some(d3)) => assert_ne!(d2, d3, "Component 2 winding inconsistent"),
            _ => panic!("Edge should exist in both faces of component 2"),
        }
    }

    #[test]
    fn test_empty_mesh() {
        let mut mesh = IndexedMesh::new();
        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn test_single_face() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
        // Single face should remain unchanged
    }

    #[test]
    fn test_count_inconsistent() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, -1.0, 0.0));

        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong winding

        let count = count_inconsistent_faces(&mesh);
        assert_eq!(count, 1);
    }
}
