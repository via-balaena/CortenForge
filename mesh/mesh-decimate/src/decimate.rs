//! Core mesh decimation algorithm.
//!
//! Implements edge collapse with quadric error metrics (QEM).

// Mesh indices and counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
// Algorithm uses standard mathematical variable names
#![allow(clippy::many_single_char_names)]

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use mesh_types::{IndexedMesh, Point3, Vertex};
use tracing::{debug, info};

use crate::params::DecimateParams;
use crate::quadric::Quadric;
use crate::result::DecimationResult;

/// An edge collapse candidate in the priority queue.
#[derive(Debug, Clone)]
struct EdgeCollapse {
    /// The two vertex indices forming the edge.
    v1: u32,
    v2: u32,
    /// The error cost of this collapse.
    cost: f64,
    /// The optimal position for the merged vertex.
    optimal_pos: [f64; 3],
}

impl PartialEq for EdgeCollapse {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for EdgeCollapse {}

impl PartialOrd for EdgeCollapse {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EdgeCollapse {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior (smaller cost = higher priority)
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

/// Decimate a mesh using edge collapse with quadric error metrics.
///
/// # Arguments
///
/// * `mesh` - The input mesh to decimate
/// * `params` - Decimation parameters
///
/// # Returns
///
/// A [`DecimationResult`] containing the decimated mesh and statistics.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_decimate::{decimate_mesh, DecimateParams};
///
/// let cube = unit_cube();
/// let result = decimate_mesh(&cube, &DecimateParams::with_target_ratio(0.5));
/// println!("{}", result);
/// ```
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn decimate_mesh(mesh: &IndexedMesh, params: &DecimateParams) -> DecimationResult {
    let original_triangles = mesh.faces.len();

    // Handle edge cases
    if original_triangles == 0 {
        return DecimationResult {
            mesh: mesh.clone(),
            original_triangles: 0,
            final_triangles: 0,
            collapses_performed: 0,
            collapses_rejected: 0,
        };
    }

    // Calculate target triangle count
    let target = params
        .target_triangles
        .unwrap_or_else(|| ((original_triangles as f64) * params.target_ratio).ceil() as usize);

    // Don't decimate if already at or below target
    if original_triangles <= target {
        return DecimationResult {
            mesh: mesh.clone(),
            original_triangles,
            final_triangles: original_triangles,
            collapses_performed: 0,
            collapses_rejected: 0,
        };
    }

    info!(
        original = original_triangles,
        target = target,
        "Starting mesh decimation"
    );

    // Create working copy of mesh data
    let mut vertices: Vec<Option<Vertex>> = mesh.vertices.iter().cloned().map(Some).collect();
    let mut faces: Vec<Option<[u32; 3]>> = mesh.faces.iter().copied().map(Some).collect();
    let mut active_faces = original_triangles;

    // Build edge to face adjacency
    let edge_to_faces = build_edge_to_faces(&mesh.faces);

    // Compute initial quadrics for each vertex
    let mut quadrics = compute_vertex_quadrics(mesh);

    // Identify boundary edges
    let boundary_edges = find_boundary_edges(&edge_to_faces);

    // Identify sharp feature edges if needed
    let sharp_edges = if params.preserve_sharp_features {
        find_sharp_edges(mesh, &edge_to_faces, params.sharp_angle_threshold)
    } else {
        HashSet::new()
    };

    // Build initial edge collapse queue
    let mut heap =
        build_collapse_queue(mesh, &quadrics, &boundary_edges, &sharp_edges, params);

    // Track which vertices have been merged (maps old index -> new index)
    let mut vertex_remap: HashMap<u32, u32> = HashMap::new();

    let mut collapses_performed = 0;
    let mut collapses_rejected = 0;

    // Main decimation loop
    while active_faces > target {
        let Some(collapse) = heap.pop() else {
            break;
        };

        // Get actual vertex indices (following remap chain)
        let v1 = get_actual_vertex(collapse.v1, &vertex_remap);
        let v2 = get_actual_vertex(collapse.v2, &vertex_remap);

        // Skip if vertices have been merged or are the same
        if v1 == v2 || vertices[v1 as usize].is_none() || vertices[v2 as usize].is_none() {
            continue;
        }

        // Skip if this is a boundary edge and we're preserving boundaries
        if params.preserve_boundary {
            let edge = normalize_edge(v1, v2);
            if boundary_edges.contains(&edge) {
                collapses_rejected += 1;
                continue;
            }
        }

        // Check if collapse would create non-manifold geometry
        if !is_collapse_valid(&vertices, &faces, v1, v2) {
            collapses_rejected += 1;
            continue;
        }

        // Check max error threshold
        if let Some(max_error) = params.max_error {
            let error = quadrics[v1 as usize].evaluate(
                collapse.optimal_pos[0],
                collapse.optimal_pos[1],
                collapse.optimal_pos[2],
            ) + quadrics[v2 as usize].evaluate(
                collapse.optimal_pos[0],
                collapse.optimal_pos[1],
                collapse.optimal_pos[2],
            );
            if error > max_error {
                collapses_rejected += 1;
                continue;
            }
        }

        // Perform the collapse: merge v2 into v1
        if let Some(ref mut v) = vertices[v1 as usize] {
            v.position = Point3::new(
                collapse.optimal_pos[0],
                collapse.optimal_pos[1],
                collapse.optimal_pos[2],
            );
        }

        // Combine quadrics
        let q2 = quadrics[v2 as usize];
        quadrics[v1 as usize].add(&q2);

        // Remove v2
        vertices[v2 as usize] = None;
        vertex_remap.insert(v2, v1);

        // Update faces: replace v2 with v1, remove degenerate faces
        for face_opt in &mut faces {
            if let Some(face) = face_opt {
                for idx in face.iter_mut() {
                    let actual = get_actual_vertex(*idx, &vertex_remap);
                    *idx = actual;
                    if actual == v2 {
                        *idx = v1;
                    }
                }

                // Check if face is degenerate (has duplicate vertices)
                if face[0] == face[1] || face[1] == face[2] || face[0] == face[2] {
                    *face_opt = None;
                    active_faces -= 1;
                }
            }
        }

        collapses_performed += 1;

        // Re-queue edges involving v1 with updated costs
        requeue_vertex_edges(
            v1,
            &vertices,
            &faces,
            &quadrics,
            &boundary_edges,
            &sharp_edges,
            params,
            &mut heap,
        );
    }

    // Build the final mesh
    let final_mesh = build_final_mesh(&vertices, &faces);

    info!(
        final_triangles = active_faces,
        collapses = collapses_performed,
        "Decimation complete"
    );

    DecimationResult {
        mesh: final_mesh,
        original_triangles,
        final_triangles: active_faces,
        collapses_performed,
        collapses_rejected,
    }
}

// ============================================================================
// Internal helper functions
// ============================================================================

const fn normalize_edge(v1: u32, v2: u32) -> (u32, u32) {
    if v1 < v2 {
        (v1, v2)
    } else {
        (v2, v1)
    }
}

fn get_actual_vertex(mut v: u32, remap: &HashMap<u32, u32>) -> u32 {
    while let Some(&new_v) = remap.get(&v) {
        v = new_v;
    }
    v
}

fn build_edge_to_faces(faces: &[[u32; 3]]) -> HashMap<(u32, u32), Vec<usize>> {
    let mut edge_to_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

    for (face_idx, face) in faces.iter().enumerate() {
        for i in 0..3 {
            let edge = normalize_edge(face[i], face[(i + 1) % 3]);
            edge_to_faces.entry(edge).or_default().push(face_idx);
        }
    }

    edge_to_faces
}

fn find_boundary_edges(edge_to_faces: &HashMap<(u32, u32), Vec<usize>>) -> HashSet<(u32, u32)> {
    edge_to_faces
        .iter()
        .filter(|(_, faces)| faces.len() == 1)
        .map(|(edge, _)| *edge)
        .collect()
}

fn find_sharp_edges(
    mesh: &IndexedMesh,
    edge_to_faces: &HashMap<(u32, u32), Vec<usize>>,
    threshold: f64,
) -> HashSet<(u32, u32)> {
    let mut sharp_edges = HashSet::new();

    for (&edge, face_indices) in edge_to_faces {
        if face_indices.len() != 2 {
            continue;
        }

        let f1 = &mesh.faces[face_indices[0]];
        let f2 = &mesh.faces[face_indices[1]];

        let n1 = compute_face_normal(mesh, f1);
        let n2 = compute_face_normal(mesh, f2);

        if let (Some(n1), Some(n2)) = (n1, n2) {
            let dot = n1[0].mul_add(n2[0], n1[1].mul_add(n2[1], n1[2] * n2[2]));
            let angle = dot.clamp(-1.0, 1.0).acos();
            if angle > threshold {
                sharp_edges.insert(edge);
            }
        }
    }

    sharp_edges
}

fn compute_face_normal(mesh: &IndexedMesh, face: &[u32; 3]) -> Option<[f64; 3]> {
    let v0 = &mesh.vertices[face[0] as usize].position;
    let v1 = &mesh.vertices[face[1] as usize].position;
    let v2 = &mesh.vertices[face[2] as usize].position;

    let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
    let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];

    let normal = [
        e1[1].mul_add(e2[2], -e1[2] * e2[1]),
        e1[2].mul_add(e2[0], -e1[0] * e2[2]),
        e1[0].mul_add(e2[1], -e1[1] * e2[0]),
    ];

    let len = normal[0]
        .mul_add(normal[0], normal[1].mul_add(normal[1], normal[2] * normal[2]))
        .sqrt();
    if len < 1e-10 {
        return None;
    }

    Some([normal[0] / len, normal[1] / len, normal[2] / len])
}

fn compute_vertex_quadrics(mesh: &IndexedMesh) -> Vec<Quadric> {
    let mut quadrics = vec![Quadric::default(); mesh.vertices.len()];

    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
        let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];

        let normal = [
            e1[1].mul_add(e2[2], -e1[2] * e2[1]),
            e1[2].mul_add(e2[0], -e1[0] * e2[2]),
            e1[0].mul_add(e2[1], -e1[1] * e2[0]),
        ];

        let len = normal[0]
            .mul_add(normal[0], normal[1].mul_add(normal[1], normal[2] * normal[2]))
            .sqrt();
        if len < 1e-10 {
            continue;
        }

        let a = normal[0] / len;
        let b = normal[1] / len;
        let c = normal[2] / len;
        let d = -a.mul_add(v0.x, b.mul_add(v0.y, c * v0.z));

        let q = Quadric::from_plane(a, b, c, d);

        for &vi in face {
            quadrics[vi as usize].add(&q);
        }
    }

    quadrics
}

fn build_collapse_queue(
    mesh: &IndexedMesh,
    quadrics: &[Quadric],
    boundary_edges: &HashSet<(u32, u32)>,
    sharp_edges: &HashSet<(u32, u32)>,
    params: &DecimateParams,
) -> BinaryHeap<EdgeCollapse> {
    let mut heap = BinaryHeap::new();
    let mut seen_edges = HashSet::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let v1 = face[i];
            let v2 = face[(i + 1) % 3];
            let edge = normalize_edge(v1, v2);

            if seen_edges.contains(&edge) {
                continue;
            }
            seen_edges.insert(edge);

            if let Some(collapse) =
                compute_edge_collapse(v1, v2, mesh, quadrics, boundary_edges, sharp_edges, params)
            {
                heap.push(collapse);
            }
        }
    }

    heap
}

fn compute_edge_collapse(
    v1: u32,
    v2: u32,
    mesh: &IndexedMesh,
    quadrics: &[Quadric],
    boundary_edges: &HashSet<(u32, u32)>,
    sharp_edges: &HashSet<(u32, u32)>,
    params: &DecimateParams,
) -> Option<EdgeCollapse> {
    let edge = normalize_edge(v1, v2);

    // Skip if preserving boundary and this is boundary
    if params.preserve_boundary && boundary_edges.contains(&edge) {
        return None;
    }

    // Skip if preserving sharp features and this is sharp
    if params.preserve_sharp_features && sharp_edges.contains(&edge) {
        return None;
    }

    let q1 = &quadrics[v1 as usize];
    let q2 = &quadrics[v2 as usize];

    // Combined quadric
    let mut combined = *q1;
    combined.add(q2);

    // Find optimal position
    let pos1 = &mesh.vertices[v1 as usize].position;
    let pos2 = &mesh.vertices[v2 as usize].position;
    let midpoint = [
        f64::midpoint(pos1.x, pos2.x),
        f64::midpoint(pos1.y, pos2.y),
        f64::midpoint(pos1.z, pos2.z),
    ];

    let optimal_pos = combined.optimal_point().unwrap_or(midpoint);

    // Compute error at optimal position
    let mut cost = combined.evaluate(optimal_pos[0], optimal_pos[1], optimal_pos[2]);

    // Apply boundary penalty
    if boundary_edges.contains(&edge) {
        cost *= params.boundary_penalty;
    }

    Some(EdgeCollapse {
        v1,
        v2,
        cost,
        optimal_pos,
    })
}

fn is_collapse_valid(
    vertices: &[Option<Vertex>],
    faces: &[Option<[u32; 3]>],
    v1: u32,
    v2: u32,
) -> bool {
    // Find faces adjacent to v1 and v2
    let mut v1_neighbors: HashSet<u32> = HashSet::new();
    let mut v2_neighbors: HashSet<u32> = HashSet::new();

    for face in faces.iter().flatten() {
        let has_v1 = face.contains(&v1);
        let has_v2 = face.contains(&v2);

        if has_v1 {
            for &vi in face {
                if vi != v1 && vi != v2 && vertices[vi as usize].is_some() {
                    v1_neighbors.insert(vi);
                }
            }
        }

        if has_v2 {
            for &vi in face {
                if vi != v1 && vi != v2 && vertices[vi as usize].is_some() {
                    v2_neighbors.insert(vi);
                }
            }
        }
    }

    // The collapse is valid if there are at most 2 shared neighbors
    // (the vertices of the edge's adjacent triangles)
    let shared: HashSet<_> = v1_neighbors.intersection(&v2_neighbors).collect();
    shared.len() <= 2
}

#[allow(clippy::too_many_arguments)]
fn requeue_vertex_edges(
    v1: u32,
    vertices: &[Option<Vertex>],
    faces: &[Option<[u32; 3]>],
    quadrics: &[Quadric],
    boundary_edges: &HashSet<(u32, u32)>,
    sharp_edges: &HashSet<(u32, u32)>,
    params: &DecimateParams,
    heap: &mut BinaryHeap<EdgeCollapse>,
) {
    let Some(v1_vertex) = &vertices[v1 as usize] else {
        return;
    };

    // Find all neighbors of v1
    let mut neighbors: HashSet<u32> = HashSet::new();
    for face in faces.iter().flatten() {
        if face.contains(&v1) {
            for &vi in face {
                if vi != v1 && vertices[vi as usize].is_some() {
                    neighbors.insert(vi);
                }
            }
        }
    }

    // Create temporary mesh-like data for collapse computation
    for &v2 in &neighbors {
        let Some(v2_vertex) = &vertices[v2 as usize] else {
            continue;
        };

        let edge = normalize_edge(v1, v2);

        if params.preserve_boundary && boundary_edges.contains(&edge) {
            continue;
        }

        if params.preserve_sharp_features && sharp_edges.contains(&edge) {
            continue;
        }

        let q1 = &quadrics[v1 as usize];
        let q2 = &quadrics[v2 as usize];

        let mut combined = *q1;
        combined.add(q2);

        let midpoint = [
            f64::midpoint(v1_vertex.position.x, v2_vertex.position.x),
            f64::midpoint(v1_vertex.position.y, v2_vertex.position.y),
            f64::midpoint(v1_vertex.position.z, v2_vertex.position.z),
        ];

        let optimal_pos = combined.optimal_point().unwrap_or(midpoint);
        let mut cost = combined.evaluate(optimal_pos[0], optimal_pos[1], optimal_pos[2]);

        if boundary_edges.contains(&edge) {
            cost *= params.boundary_penalty;
        }

        heap.push(EdgeCollapse {
            v1,
            v2,
            cost,
            optimal_pos,
        });
    }
}

fn build_final_mesh(vertices: &[Option<Vertex>], faces: &[Option<[u32; 3]>]) -> IndexedMesh {
    // Create vertex remap from old indices to new indices
    let mut vertex_remap: HashMap<u32, u32> = HashMap::new();
    let mut new_vertices = Vec::new();

    for (old_idx, vertex_opt) in vertices.iter().enumerate() {
        if let Some(vertex) = vertex_opt {
            vertex_remap.insert(old_idx as u32, new_vertices.len() as u32);
            new_vertices.push(vertex.clone());
        }
    }

    // Remap faces
    let mut new_faces = Vec::new();
    for face in faces.iter().flatten() {
        if let (Some(&i0), Some(&i1), Some(&i2)) = (
            vertex_remap.get(&face[0]),
            vertex_remap.get(&face[1]),
            vertex_remap.get(&face[2]),
        ) {
            new_faces.push([i0, i1, i2]);
        }
    }

    debug!(
        vertices = new_vertices.len(),
        faces = new_faces.len(),
        "Built final decimated mesh"
    );

    IndexedMesh {
        vertices: new_vertices,
        faces: new_faces,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::unit_cube;

    #[test]
    fn test_decimate_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = decimate_mesh(&mesh, &DecimateParams::default());

        assert_eq!(result.original_triangles, 0);
        assert_eq!(result.final_triangles, 0);
        assert_eq!(result.collapses_performed, 0);
    }

    #[test]
    fn test_decimate_unit_cube() {
        let cube = unit_cube();
        let original_faces = cube.faces.len();

        let result = decimate_mesh(&cube, &DecimateParams::with_target_ratio(0.5));

        // Should reduce face count
        assert!(result.final_triangles <= original_faces);
        assert_eq!(result.original_triangles, original_faces);
    }

    #[test]
    fn test_decimate_preserve_boundary() {
        let cube = unit_cube();

        let result = decimate_mesh(
            &cube,
            &DecimateParams::default().with_preserve_boundary(true),
        );

        // Should still produce valid mesh
        assert!(result.final_triangles > 0);
    }

    #[test]
    fn test_decimate_aggressive() {
        let cube = unit_cube();

        let result = decimate_mesh(&cube, &DecimateParams::aggressive());

        // Aggressive should reduce more
        assert!(result.final_triangles <= cube.faces.len() / 2);
    }

    #[test]
    fn test_decimate_target_triangles() {
        let cube = unit_cube();
        let original = cube.faces.len();

        // Target exactly half
        let result = decimate_mesh(&cube, &DecimateParams::with_target_triangles(original / 2));

        assert!(result.final_triangles <= original / 2 + 2); // Allow small margin
    }

    #[test]
    fn test_normalize_edge() {
        assert_eq!(normalize_edge(5, 3), (3, 5));
        assert_eq!(normalize_edge(3, 5), (3, 5));
        assert_eq!(normalize_edge(1, 1), (1, 1));
    }
}
