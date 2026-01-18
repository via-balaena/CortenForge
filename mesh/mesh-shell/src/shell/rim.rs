//! Rim generation for connecting inner and outer shell surfaces.
//!
//! The rim is a strip of triangles that connects the boundary edges of the
//! inner and outer surfaces, creating a watertight shell.

// Mesh processing uses u32 indices; truncation would only occur for meshes with >4B vertices
// which exceeds practical limits.
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_sign_loss)]

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use tracing::debug;

/// Result of rim generation.
#[derive(Debug, Clone)]
pub struct RimResult {
    /// Faces generated for the rim.
    pub faces: Vec<[u32; 3]>,
    /// Number of boundary edges that were connected.
    pub boundary_edge_count: usize,
    /// Number of boundary loops found.
    pub loop_count: usize,
}

/// Generate rim faces to connect inner and outer surfaces.
///
/// For a shell with normal-based offset where inner and outer vertices have
/// a 1:1 correspondence, we generate triangles connecting boundary edges.
///
/// # Arguments
///
/// * `mesh` - The inner mesh (used to find boundary edges)
/// * `inner_vertex_count` - Number of inner vertices (outer vertices are offset by this)
///
/// # Returns
///
/// A tuple of (rim faces, boundary edge count).
pub fn generate_rim(mesh: &IndexedMesh, inner_vertex_count: usize) -> (Vec<[u32; 3]>, usize) {
    let boundary_edges = find_boundary_edges(&mesh.faces);

    if boundary_edges.is_empty() {
        debug!("No boundary edges found - mesh is watertight");
        return (Vec::new(), 0);
    }

    let boundary_count = boundary_edges.len();
    debug!("Found {} boundary edges for rim generation", boundary_count);

    let mut rim_faces = Vec::with_capacity(boundary_count * 2);
    let offset = inner_vertex_count as u32;

    // For each boundary edge in the inner surface, create a quad (2 triangles)
    // connecting it to the corresponding edge in the outer surface
    for (v0, v1) in &boundary_edges {
        // Inner edge: v0 -> v1
        // Outer edge: (v0 + offset) -> (v1 + offset)
        // Create two triangles to form a quad:
        // Triangle 1: inner v0, outer v0, outer v1
        // Triangle 2: inner v0, outer v1, inner v1
        rim_faces.push([*v0, v0 + offset, v1 + offset]);
        rim_faces.push([*v0, v1 + offset, *v1]);
    }

    debug!("Generated {} rim faces", rim_faces.len());
    (rim_faces, boundary_count)
}

/// Generate rim faces for SDF-based shell where inner and outer surfaces
/// may have different vertex counts.
///
/// This uses nearest-neighbor matching to connect boundaries.
///
/// # Arguments
///
/// * `inner_mesh` - The inner surface mesh
/// * `outer_mesh` - The outer surface mesh (different vertex count)
/// * `inner_vertex_offset` - Offset to add to inner vertex indices in the combined mesh
///
/// # Returns
///
/// A tuple of (rim faces, boundary edge count).
#[allow(dead_code)]
pub fn generate_rim_for_sdf_shell(
    inner_mesh: &IndexedMesh,
    outer_mesh: &IndexedMesh,
    inner_vertex_offset: usize,
) -> (Vec<[u32; 3]>, usize) {
    let inner_boundary = find_boundary_loops(&inner_mesh.faces, &inner_mesh.vertices);
    let outer_boundary = find_boundary_loops(&outer_mesh.faces, &outer_mesh.vertices);

    if inner_boundary.is_empty() || outer_boundary.is_empty() {
        debug!("No boundary loops found for rim generation");
        return (Vec::new(), 0);
    }

    let offset = inner_vertex_offset as u32;
    let mut rim_faces = Vec::new();
    let mut boundary_count = 0;

    // For each inner boundary loop, find the matching outer loop and stitch them
    for inner_loop in &inner_boundary {
        // Find the closest outer loop by comparing centroid positions
        let inner_centroid = compute_loop_centroid(inner_loop, &inner_mesh.vertices);

        let mut best_outer_loop = None;
        let mut best_dist = f64::MAX;

        for outer_loop in &outer_boundary {
            let outer_centroid = compute_loop_centroid(outer_loop, &outer_mesh.vertices);
            let dist = (outer_centroid - inner_centroid).norm();
            if dist < best_dist {
                best_dist = dist;
                best_outer_loop = Some(outer_loop);
            }
        }

        if let Some(outer_loop) = best_outer_loop {
            // Stitch the two loops together
            let faces = stitch_boundary_loops(inner_loop, outer_loop, offset);
            boundary_count += inner_loop.len();
            rim_faces.extend(faces);
        }
    }

    debug!(
        "Generated {} rim faces from {} boundary edges",
        rim_faces.len(),
        boundary_count
    );
    (rim_faces, boundary_count)
}

/// Find all boundary edges in the mesh.
///
/// A boundary edge appears in exactly one face.
fn find_boundary_edges(faces: &[[u32; 3]]) -> Vec<(u32, u32)> {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();

    for face in faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            // Normalize edge direction for counting
            let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    // Boundary edges appear exactly once
    let boundary_edges: Vec<(u32, u32)> = edge_counts
        .into_iter()
        .filter(|&(_, count)| count == 1)
        .map(|(edge, _)| edge)
        .collect();

    // Reorder edges to maintain winding order from the original face
    let mut ordered_edges = Vec::with_capacity(boundary_edges.len());
    let boundary_set: HashSet<(u32, u32)> = boundary_edges.iter().copied().collect();

    for face in faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let normalized = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            if boundary_set.contains(&normalized) {
                // Keep original winding direction
                ordered_edges.push((v0, v1));
            }
        }
    }

    ordered_edges
}

/// Find boundary loops (ordered sequences of boundary vertices).
fn find_boundary_loops(
    faces: &[[u32; 3]],
    vertices: &[mesh_types::Vertex],
) -> Vec<Vec<u32>> {
    let boundary_edges = find_boundary_edges(faces);

    if boundary_edges.is_empty() {
        return Vec::new();
    }

    // Build adjacency for boundary vertices
    let mut vertex_neighbors: HashMap<u32, Vec<u32>> = HashMap::new();
    for (v0, v1) in &boundary_edges {
        vertex_neighbors.entry(*v0).or_default().push(*v1);
        vertex_neighbors.entry(*v1).or_default().push(*v0);
    }

    // Trace loops
    let mut visited: HashSet<u32> = HashSet::new();
    let mut loops = Vec::new();

    for &start_vertex in vertex_neighbors.keys() {
        if visited.contains(&start_vertex) {
            continue;
        }

        let mut current_loop = Vec::new();
        let mut current = start_vertex;
        let mut prev = u32::MAX;

        loop {
            if visited.contains(&current) && current != start_vertex {
                break;
            }

            visited.insert(current);
            current_loop.push(current);

            // Find next unvisited neighbor
            let neighbors = vertex_neighbors.get(&current).map_or(&[][..], Vec::as_slice);
            let mut next = None;

            for &n in neighbors {
                if n != prev && (n == start_vertex || !visited.contains(&n)) {
                    next = Some(n);
                    break;
                }
            }

            match next {
                Some(n) if n == start_vertex => break, // Closed loop
                Some(n) => {
                    prev = current;
                    current = n;
                }
                None => break, // Dead end
            }
        }

        if current_loop.len() >= 3 {
            loops.push(current_loop);
        }
    }

    // Sort loops by size (largest first) for better matching
    loops.sort_by_key(|b| std::cmp::Reverse(b.len()));

    let _ = vertices; // Used for computing centroids in caller
    loops
}

/// Compute the centroid of a boundary loop.
fn compute_loop_centroid(
    loop_vertices: &[u32],
    vertices: &[mesh_types::Vertex],
) -> nalgebra::Point3<f64> {
    let mut sum = nalgebra::Point3::new(0.0, 0.0, 0.0);

    for &v_idx in loop_vertices {
        let pos = &vertices[v_idx as usize].position;
        sum.x += pos.x;
        sum.y += pos.y;
        sum.z += pos.z;
    }

    let n = loop_vertices.len() as f64;
    nalgebra::Point3::new(sum.x / n, sum.y / n, sum.z / n)
}

/// Stitch two boundary loops together with triangles.
///
/// Uses a simple approach: for each vertex in inner loop, connect to
/// corresponding vertex in outer loop (by position along the loop).
fn stitch_boundary_loops(
    inner_loop: &[u32],
    outer_loop: &[u32],
    outer_offset: u32,
) -> Vec<[u32; 3]> {
    let inner_len = inner_loop.len();
    let outer_len = outer_loop.len();

    if inner_len == 0 || outer_len == 0 {
        return Vec::new();
    }

    let mut faces = Vec::new();

    if inner_len == outer_len {
        // Simple case: equal lengths, create quads
        for i in 0..inner_len {
            let i_next = (i + 1) % inner_len;

            let inner_v0 = inner_loop[i];
            let inner_v1 = inner_loop[i_next];
            let outer_v0 = outer_loop[i] + outer_offset;
            let outer_v1 = outer_loop[i_next] + outer_offset;

            // Create two triangles for the quad
            faces.push([inner_v0, outer_v0, outer_v1]);
            faces.push([inner_v0, outer_v1, inner_v1]);
        }
    } else {
        // Different lengths: use linear interpolation
        let ratio = outer_len as f64 / inner_len as f64;

        for i in 0..inner_len {
            let i_next = (i + 1) % inner_len;
            let j = ((i as f64) * ratio) as usize % outer_len;
            let j_next = ((i_next as f64) * ratio) as usize % outer_len;

            let inner_v0 = inner_loop[i];
            let inner_v1 = inner_loop[i_next];
            let outer_v0 = outer_loop[j] + outer_offset;
            let outer_v1 = outer_loop[j_next] + outer_offset;

            faces.push([inner_v0, outer_v0, outer_v1]);
            faces.push([inner_v0, outer_v1, inner_v1]);
        }
    }

    faces
}

/// Analyze boundary edges and loops in a mesh.
#[derive(Debug, Clone)]
pub struct BoundaryAnalysis {
    /// Total number of boundary edges.
    pub edge_count: usize,
    /// Number of boundary loops found.
    pub loop_count: usize,
    /// Sizes of each boundary loop (number of vertices).
    pub loop_sizes: Vec<usize>,
}

/// Analyze the boundary structure of a mesh.
#[must_use]
pub fn analyze_boundary(mesh: &IndexedMesh) -> BoundaryAnalysis {
    let boundary_edges = find_boundary_edges(&mesh.faces);
    let loops = find_boundary_loops(&mesh.faces, &mesh.vertices);

    BoundaryAnalysis {
        edge_count: boundary_edges.len(),
        loop_count: loops.len(),
        loop_sizes: loops.iter().map(Vec::len).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_open_box() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a cube
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 10.0));

        // 5 faces (open top)
        // Bottom
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Front
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // Back
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // Left
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        // Right
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    fn create_closed_box() -> IndexedMesh {
        let mut mesh = create_open_box();

        // Add top face
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);

        mesh
    }

    #[test]
    fn test_find_boundary_edges_open_box() {
        let mesh = create_open_box();
        let edges = find_boundary_edges(&mesh.faces);

        // Open box has 4 boundary edges at the top
        assert_eq!(edges.len(), 4);
    }

    #[test]
    fn test_find_boundary_edges_closed_box() {
        let mesh = create_closed_box();
        let edges = find_boundary_edges(&mesh.faces);

        // Closed box has no boundary edges
        assert!(edges.is_empty());
    }

    #[test]
    fn test_generate_rim() {
        let mesh = create_open_box();
        let inner_vertex_count = mesh.vertices.len();

        let (rim_faces, boundary_count) = generate_rim(&mesh, inner_vertex_count);

        // Should have 2 triangles per boundary edge
        assert_eq!(rim_faces.len(), boundary_count * 2);
        assert_eq!(boundary_count, 4); // 4 boundary edges
    }

    #[test]
    fn test_generate_rim_closed_mesh() {
        let mesh = create_closed_box();
        let inner_vertex_count = mesh.vertices.len();

        let (rim_faces, boundary_count) = generate_rim(&mesh, inner_vertex_count);

        // Closed mesh has no boundary, so no rim
        assert!(rim_faces.is_empty());
        assert_eq!(boundary_count, 0);
    }

    #[test]
    fn test_analyze_boundary() {
        let mesh = create_open_box();
        let analysis = analyze_boundary(&mesh);

        assert_eq!(analysis.edge_count, 4);
        assert_eq!(analysis.loop_count, 1);
        assert_eq!(analysis.loop_sizes[0], 4);
    }

    #[test]
    fn test_analyze_boundary_closed() {
        let mesh = create_closed_box();
        let analysis = analyze_boundary(&mesh);

        assert_eq!(analysis.edge_count, 0);
        assert_eq!(analysis.loop_count, 0);
    }

    #[test]
    fn test_rim_faces_valid_indices() {
        let mesh = create_open_box();
        let inner_count = mesh.vertices.len();

        let (rim_faces, _) = generate_rim(&mesh, inner_count);

        // All indices should be valid for combined inner + outer mesh
        let total_vertices = inner_count * 2;
        for face in &rim_faces {
            for &idx in face {
                assert!(
                    (idx as usize) < total_vertices,
                    "Invalid vertex index {} >= {}",
                    idx,
                    total_vertices
                );
            }
        }
    }
}
