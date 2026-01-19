//! Core subdivision algorithms.

// Algorithm uses many indexing operations
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_possible_truncation)]

use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Vertex};
use tracing::debug;

use crate::error::{SubdivideError, SubdivideResult};
use crate::params::{SubdivideParams, SubdivisionMethod};
use crate::result::SubdivisionResult;

/// Subdivide a mesh using the specified parameters.
///
/// # Errors
///
/// Returns an error if:
/// - The mesh is empty (no vertices or faces)
/// - The iteration count is 0
/// - The resulting mesh would exceed `max_faces`
///
/// # Examples
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_subdivide::{subdivide_mesh, SubdivideParams, SubdivisionMethod};
///
/// // Create a simple triangle mesh
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// // Subdivide with Loop subdivision
/// let params = SubdivideParams::loop_subdivision();
/// let result = subdivide_mesh(&mesh, &params)?;
///
/// // Each triangle becomes 4 triangles
/// assert_eq!(result.final_faces, 4);
/// # Ok::<(), mesh_subdivide::SubdivideError>(())
/// ```
pub fn subdivide_mesh(
    mesh: &IndexedMesh,
    params: &SubdivideParams,
) -> SubdivideResult<SubdivisionResult> {
    // Validate input
    if mesh.vertices.is_empty() {
        return Err(SubdivideError::EmptyMesh);
    }
    if mesh.faces.is_empty() {
        return Err(SubdivideError::NoFaces);
    }
    if params.iterations == 0 {
        return Err(SubdivideError::InvalidIterations(0));
    }

    // Check projected size
    let projected = params.expected_faces(mesh.faces.len());
    if projected > params.max_faces {
        return Err(SubdivideError::MeshTooLarge {
            current: mesh.faces.len(),
            projected,
            max: params.max_faces,
        });
    }

    let original_faces = mesh.faces.len();
    let original_vertices = mesh.vertices.len();

    debug!(
        "Subdividing mesh: {} faces, {} vertices, {} iterations using {:?}",
        original_faces, original_vertices, params.iterations, params.method
    );

    // Perform subdivision iterations
    let mut current_mesh = mesh.clone();
    for i in 0..params.iterations {
        current_mesh = subdivide_once(&current_mesh, params);
        debug!(
            "Iteration {}: {} faces, {} vertices",
            i + 1,
            current_mesh.faces.len(),
            current_mesh.vertices.len()
        );
    }

    Ok(SubdivisionResult {
        mesh: current_mesh.clone(),
        original_faces,
        final_faces: current_mesh.faces.len(),
        original_vertices,
        final_vertices: current_mesh.vertices.len(),
        iterations: params.iterations,
        method: params.method,
    })
}

/// Perform a single subdivision iteration.
fn subdivide_once(mesh: &IndexedMesh, params: &SubdivideParams) -> IndexedMesh {
    match params.method {
        SubdivisionMethod::Midpoint | SubdivisionMethod::Flat => subdivide_midpoint(mesh),
        SubdivisionMethod::Loop => subdivide_loop(mesh, params),
    }
}

/// Midpoint subdivision - split each triangle into 4 by adding edge midpoints.
fn subdivide_midpoint(mesh: &IndexedMesh) -> IndexedMesh {
    let mut new_vertices = mesh.vertices.clone();
    let mut new_faces = Vec::with_capacity(mesh.faces.len() * 4);

    // Map from edge (sorted vertex indices) to new midpoint vertex index
    let mut edge_midpoints: HashMap<(u32, u32), u32> = HashMap::new();

    for face in &mesh.faces {
        let v0 = face[0];
        let v1 = face[1];
        let v2 = face[2];

        // Get or create midpoint vertices for each edge
        let m01 = get_or_create_midpoint(
            v0,
            v1,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
        );
        let m12 = get_or_create_midpoint(
            v1,
            v2,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
        );
        let m20 = get_or_create_midpoint(
            v2,
            v0,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
        );

        // Create 4 new triangles
        // Corner triangles
        new_faces.push([v0, m01, m20]);
        new_faces.push([v1, m12, m01]);
        new_faces.push([v2, m20, m12]);
        // Center triangle
        new_faces.push([m01, m12, m20]);
    }

    IndexedMesh {
        vertices: new_vertices,
        faces: new_faces,
    }
}

/// Get or create a midpoint vertex for an edge.
fn get_or_create_midpoint(
    v0: u32,
    v1: u32,
    original_vertices: &[Vertex],
    new_vertices: &mut Vec<Vertex>,
    edge_midpoints: &mut HashMap<(u32, u32), u32>,
) -> u32 {
    let edge = normalize_edge(v0, v1);

    if let Some(&midpoint_idx) = edge_midpoints.get(&edge) {
        return midpoint_idx;
    }

    // Calculate midpoint
    let p0 = &original_vertices[v0 as usize].position;
    let p1 = &original_vertices[v1 as usize].position;
    let midpoint = Vertex::from_coords(
        (p0.x + p1.x) * 0.5,
        (p0.y + p1.y) * 0.5,
        (p0.z + p1.z) * 0.5,
    );

    let new_idx = new_vertices.len() as u32;
    new_vertices.push(midpoint);
    edge_midpoints.insert(edge, new_idx);

    new_idx
}

/// Normalize edge so smaller vertex index comes first.
const fn normalize_edge(v0: u32, v1: u32) -> (u32, u32) {
    if v0 <= v1 { (v0, v1) } else { (v1, v0) }
}

/// Loop subdivision - smoothing subdivision for triangle meshes.
fn subdivide_loop(mesh: &IndexedMesh, params: &SubdivideParams) -> IndexedMesh {
    // Build adjacency information
    let boundary_edges = find_boundary_edges(mesh);
    let vertex_neighbors = build_vertex_neighbors(mesh);

    // Calculate new positions for original vertices (odd vertices)
    let mut new_vertices = Vec::with_capacity(mesh.vertices.len() * 4);

    for (vi, vertex) in mesh.vertices.iter().enumerate() {
        let neighbors = &vertex_neighbors[vi];
        let is_boundary = is_boundary_vertex(vi as u32, &boundary_edges);

        let new_pos = if is_boundary && params.preserve_boundaries {
            // Keep boundary vertices in place or use boundary rules
            boundary_vertex_position(vi as u32, &mesh.vertices, &boundary_edges)
        } else if neighbors.is_empty() {
            vertex.position
        } else {
            // Interior vertex - apply Loop's smoothing rule
            interior_vertex_position(&vertex.position, neighbors, &mesh.vertices)
        };

        new_vertices.push(Vertex::from_coords(new_pos.x, new_pos.y, new_pos.z));
    }

    // Create edge midpoints (even vertices) with Loop's edge rule
    let mut edge_midpoints: HashMap<(u32, u32), u32> = HashMap::new();
    let mut new_faces = Vec::with_capacity(mesh.faces.len() * 4);

    // Build edge-to-face mapping for Loop's edge vertex rule
    let edge_faces = build_edge_faces(mesh);

    for face in &mesh.faces {
        let v0 = face[0];
        let v1 = face[1];
        let v2 = face[2];

        // Get or create edge vertices
        let m01 = get_or_create_loop_edge_vertex(
            v0,
            v1,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
            &boundary_edges,
            &edge_faces,
            params.preserve_boundaries,
        );
        let m12 = get_or_create_loop_edge_vertex(
            v1,
            v2,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
            &boundary_edges,
            &edge_faces,
            params.preserve_boundaries,
        );
        let m20 = get_or_create_loop_edge_vertex(
            v2,
            v0,
            &mesh.vertices,
            &mut new_vertices,
            &mut edge_midpoints,
            &boundary_edges,
            &edge_faces,
            params.preserve_boundaries,
        );

        // Create 4 new triangles
        new_faces.push([v0, m01, m20]);
        new_faces.push([v1, m12, m01]);
        new_faces.push([v2, m20, m12]);
        new_faces.push([m01, m12, m20]);
    }

    IndexedMesh {
        vertices: new_vertices,
        faces: new_faces,
    }
}

/// Find boundary edges (edges with only one adjacent face).
fn find_boundary_edges(mesh: &IndexedMesh) -> HashSet<(u32, u32)> {
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();

    for face in &mesh.faces {
        for i in 0..3 {
            let edge = normalize_edge(face[i], face[(i + 1) % 3]);
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }

    edge_count
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(edge, _)| edge)
        .collect()
}

/// Build vertex neighbor lists.
fn build_vertex_neighbors(mesh: &IndexedMesh) -> Vec<Vec<u32>> {
    let mut neighbors: Vec<HashSet<u32>> = vec![HashSet::new(); mesh.vertices.len()];

    for face in &mesh.faces {
        for (i, &vi) in face.iter().enumerate() {
            let v = vi as usize;
            for (j, &vj) in face.iter().enumerate() {
                if i != j {
                    neighbors[v].insert(vj);
                }
            }
        }
    }

    neighbors
        .into_iter()
        .map(|s| s.into_iter().collect())
        .collect()
}

/// Build edge to adjacent faces mapping.
fn build_edge_faces(mesh: &IndexedMesh) -> HashMap<(u32, u32), Vec<usize>> {
    let mut edge_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

    for (fi, face) in mesh.faces.iter().enumerate() {
        for i in 0..3 {
            let edge = normalize_edge(face[i], face[(i + 1) % 3]);
            edge_faces.entry(edge).or_default().push(fi);
        }
    }

    edge_faces
}

/// Check if a vertex is on the boundary.
fn is_boundary_vertex(v: u32, boundary_edges: &HashSet<(u32, u32)>) -> bool {
    boundary_edges.iter().any(|&(a, b)| a == v || b == v)
}

/// Position type for internal calculations.
type Pos = nalgebra::Point3<f64>;

/// Calculate new position for boundary vertex using Loop's boundary rule.
fn boundary_vertex_position(
    v: u32,
    vertices: &[Vertex],
    boundary_edges: &HashSet<(u32, u32)>,
) -> Pos {
    // Find the two boundary neighbors
    let mut neighbors = Vec::new();
    for &(a, b) in boundary_edges {
        if a == v {
            neighbors.push(b);
        } else if b == v {
            neighbors.push(a);
        }
    }

    let p = &vertices[v as usize].position;

    if neighbors.len() == 2 {
        // Apply boundary rule: 3/4 * v + 1/8 * (n1 + n2)
        let n1 = &vertices[neighbors[0] as usize].position;
        let n2 = &vertices[neighbors[1] as usize].position;

        Pos::new(
            0.75_f64.mul_add(p.x, 0.125 * (n1.x + n2.x)),
            0.75_f64.mul_add(p.y, 0.125 * (n1.y + n2.y)),
            0.75_f64.mul_add(p.z, 0.125 * (n1.z + n2.z)),
        )
    } else {
        // If we can't find exactly 2 neighbors, keep original position
        *p
    }
}

/// Calculate new position for interior vertex using Loop's interior rule.
fn interior_vertex_position(vertex: &Pos, neighbors: &[u32], vertices: &[Vertex]) -> Pos {
    let n = neighbors.len();
    if n == 0 {
        return *vertex;
    }

    // Loop's beta coefficient
    let beta = if n == 3 {
        3.0 / 16.0
    } else {
        3.0 / (8.0 * n as f64)
    };

    let alpha = (n as f64).mul_add(-beta, 1.0);

    // Sum neighbor positions
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_z = 0.0;
    for &ni in neighbors {
        let np = &vertices[ni as usize].position;
        sum_x += np.x;
        sum_y += np.y;
        sum_z += np.z;
    }

    Pos::new(
        alpha.mul_add(vertex.x, beta * sum_x),
        alpha.mul_add(vertex.y, beta * sum_y),
        alpha.mul_add(vertex.z, beta * sum_z),
    )
}

/// Create or get edge vertex for Loop subdivision.
///
/// Note: Currently uses simple midpoint for all edges. Full Loop subdivision
/// would use 3/8 * (v0 + v1) + 1/8 * (v2 + v3) for interior edges where v2 and v3
/// are the opposite vertices of adjacent triangles.
#[allow(clippy::too_many_arguments)]
fn get_or_create_loop_edge_vertex(
    v0: u32,
    v1: u32,
    vertices: &[Vertex],
    new_vertices: &mut Vec<Vertex>,
    edge_midpoints: &mut HashMap<(u32, u32), u32>,
    _boundary_edges: &HashSet<(u32, u32)>,
    _edge_faces: &HashMap<(u32, u32), Vec<usize>>,
    _preserve_boundaries: bool,
) -> u32 {
    let edge = normalize_edge(v0, v1);

    if let Some(&idx) = edge_midpoints.get(&edge) {
        return idx;
    }

    let p0 = &vertices[v0 as usize].position;
    let p1 = &vertices[v1 as usize].position;

    // Use simple midpoint for all edges
    let new_pos = Vertex::from_coords(
        (p0.x + p1.x) * 0.5,
        (p0.y + p1.y) * 0.5,
        (p0.z + p1.z) * 0.5,
    );

    let new_idx = new_vertices.len() as u32;
    new_vertices.push(new_pos);
    edge_midpoints.insert(edge, new_idx);

    new_idx
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;

    fn make_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_two_triangles() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]);
        mesh
    }

    #[test]
    fn test_subdivide_empty_mesh() {
        let mesh = IndexedMesh::new();
        let params = SubdivideParams::default();
        let result = subdivide_mesh(&mesh, &params);
        assert!(matches!(result, Err(SubdivideError::EmptyMesh)));
    }

    #[test]
    fn test_subdivide_no_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        let params = SubdivideParams::default();
        let result = subdivide_mesh(&mesh, &params);
        assert!(matches!(result, Err(SubdivideError::NoFaces)));
    }

    #[test]
    fn test_subdivide_zero_iterations() {
        let mesh = make_triangle();
        let params = SubdivideParams::new().with_iterations(0);
        let result = subdivide_mesh(&mesh, &params);
        assert!(matches!(result, Err(SubdivideError::InvalidIterations(0))));
    }

    #[test]
    fn test_subdivide_too_large() {
        let mesh = make_triangle();
        let params = SubdivideParams::new().with_iterations(2).with_max_faces(10); // 1 * 4^2 = 16 > 10
        let result = subdivide_mesh(&mesh, &params);
        assert!(matches!(result, Err(SubdivideError::MeshTooLarge { .. })));
    }

    #[test]
    fn test_subdivide_midpoint_single_triangle() {
        let mesh = make_triangle();
        let params = SubdivideParams::midpoint();
        let result = subdivide_mesh(&mesh, &params).expect("subdivision failed");

        assert_eq!(result.original_faces, 1);
        assert_eq!(result.final_faces, 4);
        assert_eq!(result.original_vertices, 3);
        // 3 original + 3 edge midpoints = 6
        assert_eq!(result.final_vertices, 6);
        assert!(result.was_subdivided());
    }

    #[test]
    fn test_subdivide_loop_single_triangle() {
        let mesh = make_triangle();
        let params = SubdivideParams::loop_subdivision();
        let result = subdivide_mesh(&mesh, &params).expect("subdivision failed");

        assert_eq!(result.original_faces, 1);
        assert_eq!(result.final_faces, 4);
        assert_eq!(result.method, SubdivisionMethod::Loop);
    }

    #[test]
    fn test_subdivide_two_iterations() {
        let mesh = make_triangle();
        let params = SubdivideParams::midpoint().with_iterations(2);
        let result = subdivide_mesh(&mesh, &params).expect("subdivision failed");

        assert_eq!(result.original_faces, 1);
        assert_eq!(result.final_faces, 16); // 1 * 4^2
        assert_eq!(result.iterations, 2);
    }

    #[test]
    fn test_subdivide_shared_edge() {
        let mesh = make_two_triangles();
        let params = SubdivideParams::midpoint();
        let result = subdivide_mesh(&mesh, &params).expect("subdivision failed");

        assert_eq!(result.original_faces, 2);
        assert_eq!(result.final_faces, 8);
        // 4 original + 5 edge midpoints (one shared edge)
        assert_eq!(result.final_vertices, 9);
    }

    #[test]
    fn test_normalize_edge() {
        assert_eq!(normalize_edge(0, 1), (0, 1));
        assert_eq!(normalize_edge(1, 0), (0, 1));
        assert_eq!(normalize_edge(5, 3), (3, 5));
    }

    #[test]
    fn test_find_boundary_edges() {
        let mesh = make_triangle();
        let boundary = find_boundary_edges(&mesh);
        // Single triangle has 3 boundary edges
        assert_eq!(boundary.len(), 3);
    }

    #[test]
    fn test_shared_edge_not_boundary() {
        let mesh = make_two_triangles();
        let boundary = find_boundary_edges(&mesh);
        // Two triangles share edge (1,2), so 4 boundary edges total
        assert_eq!(boundary.len(), 4);
        // Shared edge should not be in boundary
        assert!(!boundary.contains(&(1, 2)));
    }

    #[test]
    fn test_flat_subdivision() {
        let mesh = make_triangle();
        let params = SubdivideParams::flat();
        let result = subdivide_mesh(&mesh, &params).expect("subdivision failed");

        // Flat should produce same topology as midpoint
        assert_eq!(result.final_faces, 4);
        assert_eq!(result.final_vertices, 6);
    }
}
