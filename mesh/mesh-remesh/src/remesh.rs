//! Core isotropic remeshing algorithm.

// Algorithm uses many indexing operations and similar variable names
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::similar_names)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::ptr_arg)]
#![allow(clippy::tuple_array_conversions)]

use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Vertex};
use tracing::debug;

use crate::error::{RemeshError, RemeshResult};
use crate::params::RemeshParams;
use crate::result::{EdgeStatistics, RemeshResult as RemeshOutput};

/// Remesh a mesh to achieve uniform edge lengths.
///
/// This performs isotropic remeshing using iterative edge operations:
/// - Split edges longer than the target length
/// - Collapse edges shorter than the target length
/// - Flip edges to improve triangle quality
/// - Smooth vertices tangentially
///
/// # Errors
///
/// Returns an error if:
/// - The mesh is empty (no vertices or faces)
/// - The target edge length is invalid
/// - The iteration count is 0
///
/// # Examples
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_remesh::{remesh, RemeshParams};
///
/// // Create a simple triangle mesh
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// // Remesh with target edge length
/// let params = RemeshParams::with_edge_length(0.5)
///     .with_iterations(3);
/// let result = remesh(&mesh, &params)?;
///
/// // Mesh has been remeshed
/// assert!(result.final_faces >= result.original_faces);
/// # Ok::<(), mesh_remesh::RemeshError>(())
/// ```
pub fn remesh(mesh: &IndexedMesh, params: &RemeshParams) -> RemeshResult<RemeshOutput> {
    // Validate input
    if mesh.vertices.is_empty() {
        return Err(RemeshError::EmptyMesh);
    }
    if mesh.faces.is_empty() {
        return Err(RemeshError::NoFaces);
    }
    if params.target_edge_length <= 0.0 {
        return Err(RemeshError::InvalidEdgeLength(params.target_edge_length));
    }
    if params.iterations == 0 {
        return Err(RemeshError::InvalidIterations(0));
    }

    let original_faces = mesh.faces.len();
    let original_vertices = mesh.vertices.len();
    let original_edge_stats = compute_edge_statistics(mesh);

    debug!(
        "Remeshing mesh: {} faces, {} vertices, target length: {:.3}, {} iterations",
        original_faces, original_vertices, params.target_edge_length, params.iterations
    );

    // Convert to working representation
    let mut vertices: Vec<[f64; 3]> = mesh.vertices
        .iter()
        .map(|v| [v.position.x, v.position.y, v.position.z])
        .collect();
    let mut faces: Vec<[u32; 3]> = mesh.faces.clone();

    let mut total_splits = 0;
    let mut total_collapses = 0;
    let mut total_flips = 0;

    // Find boundary edges if preserving boundaries
    let boundary_edges = if params.preserve_boundaries {
        find_boundary_edges(&faces)
    } else {
        HashSet::new()
    };

    // Perform remeshing iterations
    for iter in 0..params.iterations {
        let mut iter_splits = 0;
        let mut iter_collapses = 0;
        let mut iter_flips = 0;

        // 1. Split long edges
        if params.enable_split {
            let splits = split_long_edges(
                &mut vertices,
                &mut faces,
                params.max_edge_length(),
                &boundary_edges,
            );
            iter_splits = splits;
            total_splits += splits;
        }

        // 2. Collapse short edges
        if params.enable_collapse {
            let collapses = collapse_short_edges(
                &mut vertices,
                &mut faces,
                params.min_edge_length(),
                &boundary_edges,
            );
            iter_collapses = collapses;
            total_collapses += collapses;
        }

        // 3. Flip edges to improve quality
        if params.enable_flip {
            let flips = flip_edges_for_quality(&mut faces, &vertices);
            iter_flips = flips;
            total_flips += flips;
        }

        // 4. Tangential smoothing
        if params.enable_smooth {
            tangential_smooth(&mut vertices, &faces, &boundary_edges);
        }

        debug!(
            "Iteration {}: {} splits, {} collapses, {} flips",
            iter + 1, iter_splits, iter_collapses, iter_flips
        );

        // Early exit if no operations performed
        if iter_splits == 0 && iter_collapses == 0 && iter_flips == 0 {
            debug!("Converged at iteration {}", iter + 1);
            break;
        }
    }

    // Convert back to IndexedMesh
    let result_mesh = IndexedMesh {
        vertices: vertices
            .into_iter()
            .map(|v| Vertex::from_coords(v[0], v[1], v[2]))
            .collect(),
        faces,
    };

    let final_edge_stats = compute_edge_statistics(&result_mesh);

    Ok(RemeshOutput {
        mesh: result_mesh.clone(),
        original_faces,
        final_faces: result_mesh.faces.len(),
        original_vertices,
        final_vertices: result_mesh.vertices.len(),
        iterations: params.iterations,
        splits_performed: total_splits,
        collapses_performed: total_collapses,
        flips_performed: total_flips,
        original_edge_stats,
        final_edge_stats,
    })
}

/// Compute edge length statistics for a mesh.
fn compute_edge_statistics(mesh: &IndexedMesh) -> EdgeStatistics {
    let mut edge_lengths: Vec<f64> = Vec::new();
    let mut seen_edges: HashSet<(u32, u32)> = HashSet::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 <= v1 { (v0, v1) } else { (v1, v0) };

            if seen_edges.insert(edge) {
                let p0 = &mesh.vertices[v0 as usize].position;
                let p1 = &mesh.vertices[v1 as usize].position;
                let length = ((p1.x - p0.x).powi(2) + (p1.y - p0.y).powi(2) + (p1.z - p0.z).powi(2)).sqrt();
                edge_lengths.push(length);
            }
        }
    }

    if edge_lengths.is_empty() {
        return EdgeStatistics::default();
    }

    let min_length = edge_lengths.iter().copied().fold(f64::INFINITY, f64::min);
    let max_length = edge_lengths.iter().copied().fold(0.0, f64::max);
    let avg_length = edge_lengths.iter().sum::<f64>() / edge_lengths.len() as f64;

    let variance = edge_lengths.iter()
        .map(|&l| (l - avg_length).powi(2))
        .sum::<f64>() / edge_lengths.len() as f64;
    let std_dev = variance.sqrt();

    EdgeStatistics {
        min_length,
        max_length,
        avg_length,
        std_dev,
        edge_count: edge_lengths.len(),
    }
}

/// Find boundary edges (edges with only one adjacent face).
fn find_boundary_edges(faces: &[[u32; 3]]) -> HashSet<(u32, u32)> {
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();

    for face in faces {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 <= v1 { (v0, v1) } else { (v1, v0) };
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }

    edge_count
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(edge, _)| edge)
        .collect()
}

/// Split edges longer than the threshold.
fn split_long_edges(
    vertices: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[u32; 3]>,
    max_length: f64,
    boundary_edges: &HashSet<(u32, u32)>,
) -> usize {
    let max_length_sq = max_length * max_length;
    let mut splits = 0;

    // Collect edges to split
    let mut edges_to_split: Vec<(u32, u32, f64)> = Vec::new();

    for face in faces.iter() {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 <= v1 { (v0, v1) } else { (v1, v0) };

            let p0 = vertices[v0 as usize];
            let p1 = vertices[v1 as usize];
            let length_sq = (p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2);

            if length_sq > max_length_sq && !boundary_edges.contains(&edge) {
                edges_to_split.push((v0, v1, length_sq));
            }
        }
    }

    // Sort by length (longest first) and deduplicate
    edges_to_split.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    edges_to_split.retain(|(v0, v1, _)| {
        let edge = if v0 <= v1 { (*v0, *v1) } else { (*v1, *v0) };
        seen.insert(edge)
    });

    // Split each edge (limit to prevent infinite growth)
    let max_splits = faces.len();
    for (v0, v1, _) in edges_to_split.into_iter().take(max_splits) {
        if split_edge(vertices, faces, v0, v1) {
            splits += 1;
        }
    }

    splits
}

/// Split a single edge by inserting a midpoint vertex.
fn split_edge(
    vertices: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[u32; 3]>,
    v0: u32,
    v1: u32,
) -> bool {
    // Find faces containing this edge
    let mut adjacent_faces: Vec<(usize, usize)> = Vec::new();
    for (fi, face) in faces.iter().enumerate() {
        for i in 0..3 {
            if (face[i] == v0 && face[(i + 1) % 3] == v1) || (face[i] == v1 && face[(i + 1) % 3] == v0) {
                adjacent_faces.push((fi, i));
                break;
            }
        }
    }

    if adjacent_faces.is_empty() {
        return false;
    }

    // Create midpoint vertex
    let p0 = vertices[v0 as usize];
    let p1 = vertices[v1 as usize];
    let midpoint = [
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    ];
    let mid_idx = vertices.len() as u32;
    vertices.push(midpoint);

    // Split each adjacent face into two
    let mut new_faces: Vec<[u32; 3]> = Vec::new();
    let mut faces_to_remove: Vec<usize> = Vec::new();

    for (fi, edge_start) in &adjacent_faces {
        let face = faces[*fi];
        let va = face[*edge_start];
        let vb = face[(*edge_start + 1) % 3];
        let vc = face[(*edge_start + 2) % 3];

        // Replace original face with two new triangles
        faces_to_remove.push(*fi);
        new_faces.push([va, mid_idx, vc]);
        new_faces.push([mid_idx, vb, vc]);
    }

    // Remove old faces (in reverse order to maintain indices)
    faces_to_remove.sort_by(|a, b| b.cmp(a));
    for fi in faces_to_remove {
        faces.swap_remove(fi);
    }

    // Add new faces
    faces.extend(new_faces);

    true
}

/// Collapse edges shorter than the threshold.
fn collapse_short_edges(
    vertices: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[u32; 3]>,
    min_length: f64,
    boundary_edges: &HashSet<(u32, u32)>,
) -> usize {
    let min_length_sq = min_length * min_length;
    let mut collapses = 0;

    // Collect edges to collapse
    let mut edges_to_collapse: Vec<(u32, u32, f64)> = Vec::new();

    for face in faces.iter() {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 <= v1 { (v0, v1) } else { (v1, v0) };

            let p0 = vertices[v0 as usize];
            let p1 = vertices[v1 as usize];
            let length_sq = (p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2);

            if length_sq < min_length_sq && !boundary_edges.contains(&edge) {
                edges_to_collapse.push((v0, v1, length_sq));
            }
        }
    }

    // Sort by length (shortest first) and deduplicate
    edges_to_collapse.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    edges_to_collapse.retain(|(v0, v1, _)| {
        let edge = if v0 <= v1 { (*v0, *v1) } else { (*v1, *v0) };
        seen.insert(edge)
    });

    // Collapse each edge (limit to prevent removing too many)
    let max_collapses = faces.len() / 4;
    for (v0, v1, _) in edges_to_collapse.into_iter().take(max_collapses) {
        if collapse_edge(vertices, faces, v0, v1) {
            collapses += 1;
        }
    }

    collapses
}

/// Collapse a single edge by merging two vertices.
fn collapse_edge(
    vertices: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[u32; 3]>,
    v0: u32,
    v1: u32,
) -> bool {
    // Move v0 to midpoint
    let p0 = vertices[v0 as usize];
    let p1 = vertices[v1 as usize];
    vertices[v0 as usize] = [
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    ];

    // Replace v1 with v0 in all faces
    for face in faces.iter_mut() {
        for v in face.iter_mut() {
            if *v == v1 {
                *v = v0;
            }
        }
    }

    // Remove degenerate faces (where two or more vertices are the same)
    faces.retain(|face| face[0] != face[1] && face[1] != face[2] && face[2] != face[0]);

    true
}

/// Flip edges to improve triangle quality (Delaunay-like criterion).
fn flip_edges_for_quality(faces: &mut Vec<[u32; 3]>, vertices: &[[f64; 3]]) -> usize {
    let mut flips = 0;

    // Build edge to face mapping
    let mut edge_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            let edge = if v0 <= v1 { (v0, v1) } else { (v1, v0) };
            edge_faces.entry(edge).or_default().push(fi);
        }
    }

    // Find edges to flip
    let mut edges_to_flip: Vec<(u32, u32)> = Vec::new();

    for (edge, adj_faces) in &edge_faces {
        if adj_faces.len() == 2
            && should_flip_edge(*edge, adj_faces[0], adj_faces[1], faces, vertices)
        {
            edges_to_flip.push(*edge);
        }
    }

    // Perform flips
    for edge in edges_to_flip {
        if let Some(adj) = edge_faces.get(&edge) {
            if adj.len() == 2 && flip_edge(faces, adj[0], adj[1], edge) {
                flips += 1;
            }
        }
    }

    flips
}

/// Check if flipping an edge would improve triangle quality.
fn should_flip_edge(
    edge: (u32, u32),
    fi0: usize,
    fi1: usize,
    faces: &[[u32; 3]],
    vertices: &[[f64; 3]],
) -> bool {
    let face0 = faces[fi0];
    let face1 = faces[fi1];

    // Find the opposite vertices
    let v_opp0 = face0.iter().find(|&&v| v != edge.0 && v != edge.1).copied();
    let v_opp1 = face1.iter().find(|&&v| v != edge.0 && v != edge.1).copied();

    let (Some(vo0), Some(vo1)) = (v_opp0, v_opp1) else {
        return false;
    };

    // Check Delaunay criterion: sum of opposite angles < 180 degrees
    let a = vertices[edge.0 as usize];
    let b = vertices[edge.1 as usize];
    let c = vertices[vo0 as usize];
    let d = vertices[vo1 as usize];

    let angle_c = compute_angle(&a, &c, &b);
    let angle_d = compute_angle(&a, &d, &b);

    // If sum of angles > pi, flipping improves quality
    angle_c + angle_d > std::f64::consts::PI
}

/// Compute angle at vertex b in triangle abc.
fn compute_angle(a: &[f64; 3], b: &[f64; 3], c: &[f64; 3]) -> f64 {
    let ba = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let bc = [c[0] - b[0], c[1] - b[1], c[2] - b[2]];

    let dot = ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2];
    let len_ba = (ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2]).sqrt();
    let len_bc = (bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2]).sqrt();

    if len_ba < 1e-10 || len_bc < 1e-10 {
        return 0.0;
    }

    let cos_angle = (dot / (len_ba * len_bc)).clamp(-1.0, 1.0);
    cos_angle.acos()
}

/// Flip an edge between two triangles.
fn flip_edge(faces: &mut [[u32; 3]], fi0: usize, fi1: usize, edge: (u32, u32)) -> bool {
    let face0 = faces[fi0];
    let face1 = faces[fi1];

    // Find the opposite vertices
    let v_opp0 = face0.iter().find(|&&v| v != edge.0 && v != edge.1).copied();
    let v_opp1 = face1.iter().find(|&&v| v != edge.0 && v != edge.1).copied();

    let (Some(vo0), Some(vo1)) = (v_opp0, v_opp1) else {
        return false;
    };

    // Create new faces with flipped edge
    faces[fi0] = [edge.0, vo1, vo0];
    faces[fi1] = [edge.1, vo0, vo1];

    true
}

/// Apply tangential smoothing to vertices.
fn tangential_smooth(
    vertices: &mut [[f64; 3]],
    faces: &[[u32; 3]],
    boundary_edges: &HashSet<(u32, u32)>,
) {
    // Build vertex neighbors
    let mut neighbors: Vec<HashSet<u32>> = vec![HashSet::new(); vertices.len()];
    for face in faces {
        for i in 0..3 {
            let v = face[i] as usize;
            neighbors[v].insert(face[(i + 1) % 3]);
            neighbors[v].insert(face[(i + 2) % 3]);
        }
    }

    // Find boundary vertices
    let boundary_vertices: HashSet<u32> = boundary_edges
        .iter()
        .flat_map(|&(a, b)| [a, b])
        .collect();

    // Compute new positions
    let mut new_positions: Vec<[f64; 3]> = vertices.to_vec();

    for (vi, vertex) in vertices.iter().enumerate() {
        // Don't smooth boundary vertices
        if boundary_vertices.contains(&(vi as u32)) {
            continue;
        }

        let neighbor_list: Vec<_> = neighbors[vi].iter().collect();
        if neighbor_list.is_empty() {
            continue;
        }

        // Compute centroid of neighbors
        let mut centroid = [0.0, 0.0, 0.0];
        for &ni in &neighbor_list {
            let np = vertices[*ni as usize];
            centroid[0] += np[0];
            centroid[1] += np[1];
            centroid[2] += np[2];
        }
        let n = neighbor_list.len() as f64;
        centroid[0] /= n;
        centroid[1] /= n;
        centroid[2] /= n;

        // Move towards centroid (with damping)
        let lambda: f64 = 0.5;
        new_positions[vi] = [
            lambda.mul_add(centroid[0] - vertex[0], vertex[0]),
            lambda.mul_add(centroid[1] - vertex[1], vertex[1]),
            lambda.mul_add(centroid[2] - vertex[2], vertex[2]),
        ];
    }

    vertices.copy_from_slice(&new_positions);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_quad() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh
    }

    #[test]
    fn test_remesh_empty_mesh() {
        let mesh = IndexedMesh::new();
        let params = RemeshParams::default();
        let result = remesh(&mesh, &params);
        assert!(matches!(result, Err(RemeshError::EmptyMesh)));
    }

    #[test]
    fn test_remesh_no_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        let params = RemeshParams::default();
        let result = remesh(&mesh, &params);
        assert!(matches!(result, Err(RemeshError::NoFaces)));
    }

    #[test]
    fn test_remesh_invalid_edge_length() {
        let mesh = make_triangle();
        let params = RemeshParams::with_edge_length(0.0);
        let result = remesh(&mesh, &params);
        assert!(matches!(result, Err(RemeshError::InvalidEdgeLength(_))));
    }

    #[test]
    fn test_remesh_invalid_iterations() {
        let mesh = make_triangle();
        let params = RemeshParams::default().with_iterations(0);
        let result = remesh(&mesh, &params);
        assert!(matches!(result, Err(RemeshError::InvalidIterations(0))));
    }

    #[test]
    fn test_remesh_basic() {
        let mesh = make_triangle();
        let params = RemeshParams::with_edge_length(0.3)
            .with_iterations(3);
        let result = remesh(&mesh, &params).expect("remesh failed");

        // Should have more faces after splitting
        assert!(result.final_faces >= result.original_faces);
    }

    #[test]
    fn test_remesh_quad() {
        let mesh = make_quad();
        let params = RemeshParams::with_edge_length(0.5)
            .with_iterations(2);
        let result = remesh(&mesh, &params).expect("remesh failed");

        // Should remesh successfully
        assert!(result.final_faces >= 2);
    }

    #[test]
    fn test_compute_edge_statistics() {
        let mesh = make_triangle();
        let stats = compute_edge_statistics(&mesh);

        assert!(stats.edge_count == 3);
        assert!(stats.min_length > 0.0);
        assert!(stats.max_length >= stats.min_length);
        assert!(stats.avg_length > 0.0);
    }

    #[test]
    fn test_find_boundary_edges() {
        let mesh = make_triangle();
        let boundary = find_boundary_edges(&mesh.faces);
        // Single triangle has 3 boundary edges
        assert_eq!(boundary.len(), 3);
    }

    #[test]
    fn test_find_boundary_edges_quad() {
        let mesh = make_quad();
        let boundary = find_boundary_edges(&mesh.faces);
        // Quad has 4 boundary edges (diagonal is shared)
        assert_eq!(boundary.len(), 4);
    }
}
