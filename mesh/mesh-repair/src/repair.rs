//! Core mesh repair operations.
//!
//! Provides functions for fixing common mesh issues like degenerate triangles,
//! duplicate vertices, and unreferenced vertices.

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use nalgebra::Point3;

/// Configuration parameters for mesh repair operations.
///
/// All thresholds are in the same units as the mesh coordinates (typically millimeters).
///
/// # Example
///
/// ```
/// use mesh_repair::RepairParams;
///
/// // Use defaults (good for mm-scale meshes)
/// let params = RepairParams::default();
///
/// // Or customize for your use case
/// let params = RepairParams {
///     weld_epsilon: 0.01,  // More aggressive welding for noisy scans
///     degenerate_area_threshold: 0.001,
///     ..Default::default()
/// };
/// ```
#[derive(Debug, Clone)]
pub struct RepairParams {
    /// Distance threshold for vertex welding.
    ///
    /// Vertices closer than this distance will be merged into one.
    /// Default: `1e-6`
    pub weld_epsilon: f64,

    /// Minimum triangle area threshold.
    ///
    /// Triangles with area below this threshold are removed.
    /// Default: `1e-9`
    pub degenerate_area_threshold: f64,

    /// Maximum triangle aspect ratio threshold.
    ///
    /// Triangles with aspect ratio above this are considered degenerate.
    /// Set to `f64::INFINITY` to disable.
    /// Default: `1000.0`
    pub degenerate_aspect_ratio: f64,

    /// Minimum edge length threshold.
    ///
    /// Triangles with any edge shorter than this are removed.
    /// Set to `0.0` to disable.
    /// Default: `1e-9`
    pub degenerate_min_edge_length: f64,

    /// Whether to remove unreferenced vertices after repair.
    ///
    /// Default: `true`
    pub remove_unreferenced: bool,
}

impl Default for RepairParams {
    fn default() -> Self {
        Self {
            weld_epsilon: 1e-6,
            degenerate_area_threshold: 1e-9,
            degenerate_aspect_ratio: 1000.0,
            degenerate_min_edge_length: 1e-9,
            remove_unreferenced: true,
        }
    }
}

impl RepairParams {
    /// Create params optimized for 3D scan data.
    ///
    /// Uses more aggressive welding and degenerate removal.
    #[must_use]
    pub fn for_scans() -> Self {
        Self {
            weld_epsilon: 0.01,
            degenerate_area_threshold: 0.0001,
            degenerate_aspect_ratio: 100.0,
            degenerate_min_edge_length: 0.001,
            ..Default::default()
        }
    }

    /// Create params optimized for CAD models.
    ///
    /// Uses conservative settings to preserve intentional geometry.
    #[must_use]
    pub fn for_cad() -> Self {
        Self {
            weld_epsilon: 1e-9,
            degenerate_area_threshold: 1e-12,
            degenerate_aspect_ratio: f64::INFINITY,
            degenerate_min_edge_length: 0.0,
            ..Default::default()
        }
    }

    /// Create params optimized for 3D printing preparation.
    #[must_use]
    pub fn for_printing() -> Self {
        Self {
            weld_epsilon: 0.001,
            degenerate_area_threshold: 0.00001,
            degenerate_aspect_ratio: 500.0,
            degenerate_min_edge_length: 0.0001,
            ..Default::default()
        }
    }

    /// Set the vertex welding distance threshold.
    ///
    /// Vertices closer than this distance will be merged.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_repair::RepairParams;
    ///
    /// let params = RepairParams::default()
    ///     .with_weld_epsilon(0.01);
    /// ```
    #[must_use]
    pub fn with_weld_epsilon(mut self, epsilon: f64) -> Self {
        self.weld_epsilon = epsilon;
        self
    }

    /// Set the minimum triangle area threshold.
    ///
    /// Triangles with area below this are removed as degenerate.
    #[must_use]
    pub fn with_degenerate_area_threshold(mut self, threshold: f64) -> Self {
        self.degenerate_area_threshold = threshold;
        self
    }

    /// Set the maximum triangle aspect ratio threshold.
    ///
    /// Triangles with aspect ratio above this are considered degenerate.
    /// Use `f64::INFINITY` to disable this check.
    #[must_use]
    pub fn with_degenerate_aspect_ratio(mut self, ratio: f64) -> Self {
        self.degenerate_aspect_ratio = ratio;
        self
    }

    /// Set the minimum edge length threshold.
    ///
    /// Triangles with any edge shorter than this are removed.
    /// Use `0.0` to disable this check.
    #[must_use]
    pub fn with_degenerate_min_edge_length(mut self, length: f64) -> Self {
        self.degenerate_min_edge_length = length;
        self
    }

    /// Set whether to remove unreferenced vertices after repair.
    #[must_use]
    pub fn with_remove_unreferenced(mut self, remove: bool) -> Self {
        self.remove_unreferenced = remove;
        self
    }
}

/// Remove triangles with area below threshold.
///
/// Returns the number of triangles removed.
///
/// # Arguments
///
/// * `mesh` - The mesh to repair
/// * `area_threshold` - Minimum area for a triangle to be kept
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::remove_degenerate_triangles;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 0.0, 0.0)); // Collinear - degenerate
/// mesh.faces.push([0, 1, 2]);
///
/// let removed = remove_degenerate_triangles(&mut mesh, 0.001);
/// assert_eq!(removed, 1);
/// ```
pub fn remove_degenerate_triangles(mesh: &mut IndexedMesh, area_threshold: f64) -> usize {
    let original_count = mesh.faces.len();

    mesh.faces.retain(|face| {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e1 = *v1 - *v0;
        let e2 = *v2 - *v0;
        let area = e1.cross(&e2).norm() * 0.5;

        area >= area_threshold
    });

    original_count - mesh.faces.len()
}

/// Remove degenerate triangles using multiple criteria.
///
/// A triangle is considered degenerate if:
/// - Area is below `area_threshold`
/// - Aspect ratio exceeds `max_aspect_ratio`
/// - Any edge is shorter than `min_edge_length`
///
/// Returns the number of triangles removed.
pub fn remove_degenerate_triangles_enhanced(
    mesh: &mut IndexedMesh,
    area_threshold: f64,
    max_aspect_ratio: f64,
    min_edge_length: f64,
) -> usize {
    let original_count = mesh.faces.len();

    mesh.faces.retain(|face| {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e0 = *v1 - *v0;
        let e1 = *v2 - *v1;
        let e2 = *v0 - *v2;

        // Check area
        let cross = e0.cross(&(*v2 - *v0));
        let area = cross.norm() * 0.5;
        if area < area_threshold {
            return false;
        }

        // Check edge lengths
        if min_edge_length > 0.0 {
            let len0 = e0.norm();
            let len1 = e1.norm();
            let len2 = e2.norm();
            if len0 < min_edge_length || len1 < min_edge_length || len2 < min_edge_length {
                return false;
            }
        }

        // Check aspect ratio
        if max_aspect_ratio.is_finite() && area > 0.0 {
            let len0 = e0.norm();
            let len1 = e1.norm();
            let len2 = e2.norm();
            let longest = len0.max(len1).max(len2);
            let aspect = (longest * longest) / (2.0 * area);
            if aspect > max_aspect_ratio {
                return false;
            }
        }

        true
    });

    original_count - mesh.faces.len()
}

/// Weld vertices that are within epsilon distance of each other.
///
/// Uses spatial hashing for efficiency. Returns the number of vertices merged.
///
/// # Arguments
///
/// * `mesh` - The mesh to repair
/// * `epsilon` - Maximum distance for vertices to be merged
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::weld_vertices;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0001, 0.0, 0.0)); // Near-duplicate of vertex 1
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([0, 3, 2]);
///
/// let merged = weld_vertices(&mut mesh, 0.001);
/// assert_eq!(merged, 1);
/// ```
pub fn weld_vertices(mesh: &mut IndexedMesh, epsilon: f64) -> usize {
    let original_count = mesh.vertices.len();
    if original_count == 0 {
        return 0;
    }

    let cell_size = epsilon * 2.0;

    // Build spatial hash
    let mut spatial_hash: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();

    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let cell = pos_to_cell(&vertex.position, cell_size);
        spatial_hash.entry(cell).or_default().push(idx as u32);
    }

    // Find canonical representatives
    let mut vertex_remap: Vec<u32> = (0..mesh.vertices.len() as u32).collect();
    let mut merged_count = 0;

    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let idx = idx as u32;
        if vertex_remap[idx as usize] != idx {
            continue;
        }

        let cell = pos_to_cell(&vertex.position, cell_size);

        // Check 3x3x3 neighborhood
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbor_cell = (cell.0 + dx, cell.1 + dy, cell.2 + dz);

                    if let Some(candidates) = spatial_hash.get(&neighbor_cell) {
                        for &other_idx in candidates {
                            if other_idx <= idx {
                                continue;
                            }
                            if vertex_remap[other_idx as usize] != other_idx {
                                continue;
                            }

                            let other_pos = &mesh.vertices[other_idx as usize].position;
                            let dist = (vertex.position - other_pos).norm();

                            if dist < epsilon {
                                vertex_remap[other_idx as usize] = idx;
                                merged_count += 1;
                            }
                        }
                    }
                }
            }
        }
    }

    if merged_count == 0 {
        return 0;
    }

    // Resolve transitive merges
    for i in 0..vertex_remap.len() {
        let mut target = vertex_remap[i];
        while vertex_remap[target as usize] != target {
            target = vertex_remap[target as usize];
        }
        vertex_remap[i] = target;
    }

    // Remap face indices
    for face in &mut mesh.faces {
        face[0] = vertex_remap[face[0] as usize];
        face[1] = vertex_remap[face[1] as usize];
        face[2] = vertex_remap[face[2] as usize];
    }

    // Remove degenerate faces created by welding
    mesh.faces
        .retain(|&[i0, i1, i2]| i0 != i1 && i1 != i2 && i0 != i2);

    merged_count
}

/// Convert position to spatial hash cell.
fn pos_to_cell(pos: &Point3<f64>, cell_size: f64) -> (i64, i64, i64) {
    (
        (pos.x / cell_size).floor() as i64,
        (pos.y / cell_size).floor() as i64,
        (pos.z / cell_size).floor() as i64,
    )
}

/// Remove unreferenced vertices and compact the vertex array.
///
/// Returns the number of vertices removed.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::remove_unreferenced_vertices;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(100.0, 100.0, 100.0)); // Unreferenced
/// mesh.faces.push([0, 1, 2]);
///
/// let removed = remove_unreferenced_vertices(&mut mesh);
/// assert_eq!(removed, 1);
/// assert_eq!(mesh.vertices.len(), 3);
/// ```
pub fn remove_unreferenced_vertices(mesh: &mut IndexedMesh) -> usize {
    let original_count = mesh.vertices.len();

    // Find all referenced vertices
    let mut referenced: HashSet<u32> = HashSet::new();
    for face in &mesh.faces {
        referenced.insert(face[0]);
        referenced.insert(face[1]);
        referenced.insert(face[2]);
    }

    if referenced.len() == original_count {
        return 0;
    }

    // Build compacted vertex list and remap
    let mut new_vertices = Vec::with_capacity(referenced.len());
    let mut remap: HashMap<u32, u32> = HashMap::new();

    for (old_idx, vertex) in mesh.vertices.iter().enumerate() {
        if referenced.contains(&(old_idx as u32)) {
            let new_idx = new_vertices.len() as u32;
            remap.insert(old_idx as u32, new_idx);
            new_vertices.push(vertex.clone());
        }
    }

    // Remap face indices
    for face in &mut mesh.faces {
        face[0] = remap[&face[0]];
        face[1] = remap[&face[1]];
        face[2] = remap[&face[2]];
    }

    let removed = original_count - new_vertices.len();
    mesh.vertices = new_vertices;

    removed
}

/// Remove duplicate faces from the mesh.
///
/// Faces are considered duplicate if they have the same vertices
/// (regardless of winding order or starting vertex).
///
/// Returns the number of duplicate faces removed.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::remove_duplicate_faces;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([0, 1, 2]); // Duplicate
///
/// let removed = remove_duplicate_faces(&mut mesh);
/// assert_eq!(removed, 1);
/// ```
pub fn remove_duplicate_faces(mesh: &mut IndexedMesh) -> usize {
    let original_count = mesh.faces.len();

    let mut seen: HashSet<[u32; 3]> = HashSet::new();
    let mut duplicate_indices: HashSet<usize> = HashSet::new();

    for (i, face) in mesh.faces.iter().enumerate() {
        let fwd = normalize_face(*face);
        let rev = normalize_face([face[0], face[2], face[1]]);

        if seen.contains(&fwd) || seen.contains(&rev) {
            duplicate_indices.insert(i);
        } else {
            seen.insert(fwd);
        }
    }

    if duplicate_indices.is_empty() {
        return 0;
    }

    let mut idx = 0;
    mesh.faces.retain(|_| {
        let keep = !duplicate_indices.contains(&idx);
        idx += 1;
        keep
    });

    original_count - mesh.faces.len()
}

/// Normalize a face so the smallest vertex index comes first.
fn normalize_face(face: [u32; 3]) -> [u32; 3] {
    let min_idx = if face[0] <= face[1] && face[0] <= face[2] {
        0
    } else if face[1] <= face[2] {
        1
    } else {
        2
    };

    [
        face[min_idx],
        face[(min_idx + 1) % 3],
        face[(min_idx + 2) % 3],
    ]
}

/// Run the basic repair pipeline on a mesh.
///
/// This performs:
/// 1. Remove degenerate triangles
/// 2. Weld nearby vertices
/// 3. Remove duplicate faces
/// 4. Remove unreferenced vertices
///
/// # Arguments
///
/// * `mesh` - The mesh to repair
/// * `params` - Repair parameters
///
/// # Returns
///
/// Summary of repair operations performed.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::{repair_mesh, RepairParams};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = repair_mesh(&mut mesh, &RepairParams::default());
/// ```
#[must_use]
pub fn repair_mesh(mesh: &mut IndexedMesh, params: &RepairParams) -> RepairSummary {
    let initial_vertices = mesh.vertices.len();
    let initial_faces = mesh.faces.len();

    // 1. Remove degenerate triangles
    let degenerates_removed = remove_degenerate_triangles_enhanced(
        mesh,
        params.degenerate_area_threshold,
        params.degenerate_aspect_ratio,
        params.degenerate_min_edge_length,
    );

    // 2. Weld vertices
    let vertices_welded = weld_vertices(mesh, params.weld_epsilon);

    // 3. Remove duplicate faces
    let duplicates_removed = remove_duplicate_faces(mesh);

    // 4. Remove unreferenced vertices
    let unreferenced_removed = if params.remove_unreferenced {
        remove_unreferenced_vertices(mesh)
    } else {
        0
    };

    RepairSummary {
        initial_vertices,
        initial_faces,
        final_vertices: mesh.vertices.len(),
        final_faces: mesh.faces.len(),
        vertices_welded,
        degenerates_removed,
        duplicates_removed,
        unreferenced_removed,
    }
}

/// Result of a repair operation.
#[derive(Debug, Clone, Default)]
pub struct RepairSummary {
    /// Number of vertices before repair.
    pub initial_vertices: usize,
    /// Number of faces before repair.
    pub initial_faces: usize,
    /// Number of vertices after repair.
    pub final_vertices: usize,
    /// Number of faces after repair.
    pub final_faces: usize,
    /// Number of vertices merged by welding.
    pub vertices_welded: usize,
    /// Number of degenerate triangles removed.
    pub degenerates_removed: usize,
    /// Number of duplicate faces removed.
    pub duplicates_removed: usize,
    /// Number of unreferenced vertices removed.
    pub unreferenced_removed: usize,
}

impl RepairSummary {
    /// Check if any repairs were performed.
    #[must_use]
    pub fn had_changes(&self) -> bool {
        self.vertices_welded > 0
            || self.degenerates_removed > 0
            || self.duplicates_removed > 0
            || self.unreferenced_removed > 0
    }
}

impl std::fmt::Display for RepairSummary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Repair: {} verts ({} welded, {} unreferenced), {} faces ({} degenerate, {} duplicate)",
            self.final_vertices,
            self.vertices_welded,
            self.unreferenced_removed,
            self.final_faces,
            self.degenerates_removed,
            self.duplicates_removed
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn simple_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn remove_degenerate_collinear() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.faces.push([0, 1, 2]); // Collinear = zero area

        let removed = remove_degenerate_triangles(&mut mesh, 0.001);
        assert_eq!(removed, 1);
        assert_eq!(mesh.faces.len(), 0);
    }

    #[test]
    fn remove_degenerate_keeps_valid() {
        let mut mesh = simple_mesh();

        let removed = remove_degenerate_triangles(&mut mesh, 0.001);
        assert_eq!(removed, 0);
        assert_eq!(mesh.faces.len(), 1);
    }

    #[test]
    fn weld_near_vertices() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.001, 0.0, 0.0)); // Near vertex 1
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 2]);

        let merged = weld_vertices(&mut mesh, 0.01);
        assert_eq!(merged, 1);

        // Face should now reference vertex 1 instead of 3
        assert_eq!(mesh.faces[1][1], 1);
    }

    #[test]
    fn weld_empty_mesh() {
        let mut mesh = IndexedMesh::new();
        let merged = weld_vertices(&mut mesh, 0.01);
        assert_eq!(merged, 0);
    }

    #[test]
    fn remove_unreferenced() {
        let mut mesh = simple_mesh();
        mesh.vertices.push(Vertex::from_coords(100.0, 100.0, 100.0)); // Unreferenced

        let removed = remove_unreferenced_vertices(&mut mesh);
        assert_eq!(removed, 1);
        assert_eq!(mesh.vertices.len(), 3);
    }

    #[test]
    fn remove_unreferenced_none() {
        let mut mesh = simple_mesh();

        let removed = remove_unreferenced_vertices(&mut mesh);
        assert_eq!(removed, 0);
    }

    #[test]
    fn remove_duplicate_exact() {
        let mut mesh = simple_mesh();
        mesh.faces.push([0, 1, 2]); // Exact duplicate

        let removed = remove_duplicate_faces(&mut mesh);
        assert_eq!(removed, 1);
        assert_eq!(mesh.faces.len(), 1);
    }

    #[test]
    fn remove_duplicate_reversed() {
        let mut mesh = simple_mesh();
        mesh.faces.push([0, 2, 1]); // Reversed winding = duplicate

        let removed = remove_duplicate_faces(&mut mesh);
        assert_eq!(removed, 1);
    }

    #[test]
    fn remove_duplicate_rotated() {
        let mut mesh = simple_mesh();
        mesh.faces.push([1, 2, 0]); // Rotated = duplicate

        let removed = remove_duplicate_faces(&mut mesh);
        assert_eq!(removed, 1);
    }

    #[test]
    fn repair_params_default() {
        let params = RepairParams::default();
        assert!((params.weld_epsilon - 1e-6).abs() < 1e-12);
        assert!(params.remove_unreferenced);
    }

    #[test]
    fn repair_params_for_scans() {
        let params = RepairParams::for_scans();
        assert!(params.weld_epsilon > RepairParams::default().weld_epsilon);
    }

    #[test]
    fn repair_params_for_cad() {
        let params = RepairParams::for_cad();
        assert!(params.weld_epsilon < RepairParams::default().weld_epsilon);
        assert!(params.degenerate_aspect_ratio.is_infinite());
    }

    #[test]
    fn repair_full_pipeline() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0001, 0.0, 0.0)); // Near-duplicate
        mesh.vertices.push(Vertex::from_coords(999.0, 999.0, 999.0)); // Unreferenced
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 2]); // Uses near-duplicate

        let params = RepairParams {
            weld_epsilon: 0.001,
            ..Default::default()
        };

        let result = repair_mesh(&mut mesh, &params);

        assert_eq!(result.vertices_welded, 1);
        assert!(result.had_changes());

        // All face indices should be valid
        for face in &mesh.faces {
            assert!((face[0] as usize) < mesh.vertices.len());
            assert!((face[1] as usize) < mesh.vertices.len());
            assert!((face[2] as usize) < mesh.vertices.len());
        }
    }

    #[test]
    fn repair_result_display() {
        let result = RepairSummary {
            initial_vertices: 100,
            initial_faces: 50,
            final_vertices: 95,
            final_faces: 48,
            vertices_welded: 3,
            degenerates_removed: 2,
            duplicates_removed: 0,
            unreferenced_removed: 2,
        };

        let display = format!("{}", result);
        assert!(display.contains("95 verts"));
        assert!(display.contains("3 welded"));
    }

    #[test]
    fn repair_result_no_changes() {
        let result = RepairSummary::default();
        assert!(!result.had_changes());
    }

    #[test]
    fn repair_params_for_printing() {
        let params = RepairParams::for_printing();
        assert!((params.weld_epsilon - 0.001).abs() < 1e-9);
        assert!((params.degenerate_area_threshold - 0.00001).abs() < 1e-12);
        assert!((params.degenerate_aspect_ratio - 500.0).abs() < 1e-9);
        assert!((params.degenerate_min_edge_length - 0.0001).abs() < 1e-12);
    }

    #[test]
    fn repair_params_builder_methods() {
        let params = RepairParams::default()
            .with_weld_epsilon(0.05)
            .with_degenerate_area_threshold(0.002)
            .with_degenerate_aspect_ratio(200.0)
            .with_degenerate_min_edge_length(0.003)
            .with_remove_unreferenced(false);

        assert!((params.weld_epsilon - 0.05).abs() < 1e-12);
        assert!((params.degenerate_area_threshold - 0.002).abs() < 1e-12);
        assert!((params.degenerate_aspect_ratio - 200.0).abs() < 1e-12);
        assert!((params.degenerate_min_edge_length - 0.003).abs() < 1e-12);
        assert!(!params.remove_unreferenced);
    }

    #[test]
    fn remove_degenerate_enhanced_by_aspect_ratio() {
        let mut mesh = IndexedMesh::new();
        // Create a very thin, elongated triangle (high aspect ratio)
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(100.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(50.0, 0.01, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Remove with strict aspect ratio threshold
        let removed = remove_degenerate_triangles_enhanced(&mut mesh, 1e-12, 10.0, 0.0);
        assert_eq!(removed, 1);
    }

    #[test]
    fn remove_degenerate_enhanced_by_min_edge() {
        let mut mesh = IndexedMesh::new();
        // Create triangle with one very short edge
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.001, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Remove triangles with edge shorter than 0.01
        let removed = remove_degenerate_triangles_enhanced(&mut mesh, 1e-12, f64::INFINITY, 0.01);
        // This triangle's edges are all longer than 0.01, so none removed
        assert_eq!(removed, 0);

        // Create triangle with truly short edge
        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh2.vertices.push(Vertex::from_coords(0.001, 0.0, 0.0)); // Very short edge
        mesh2.vertices.push(Vertex::from_coords(0.5, 10.0, 0.0));
        mesh2.faces.push([0, 1, 2]);

        let removed2 = remove_degenerate_triangles_enhanced(&mut mesh2, 1e-12, f64::INFINITY, 0.01);
        assert_eq!(removed2, 1);
    }
}
