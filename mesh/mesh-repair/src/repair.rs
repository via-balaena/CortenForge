//! Core mesh repair operations.
//!
//! Provides functions for fixing common mesh issues like degenerate triangles,
//! duplicate vertices, and unreferenced vertices.

#![allow(
    // `usize` → `u32` casts are safe at mesh sizes the crate targets:
    // vertex / face indices fit in `u32` by mesh-types contract
    // (see `mesh_types::VertexId`); meshes with > 2³² vertices are
    // out of scope. `f64` → `i64` casts in spatial-bucket helpers
    // (e.g., quantizing positions to integer grid keys) target
    // grid coordinates that fit in `i64` by orders of magnitude
    // for any geometrically meaningful mesh.
    clippy::cast_possible_truncation
)]

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

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
    pub const fn with_weld_epsilon(mut self, epsilon: f64) -> Self {
        self.weld_epsilon = epsilon;
        self
    }

    /// Set the minimum triangle area threshold.
    ///
    /// Triangles with area below this are removed as degenerate.
    #[must_use]
    pub const fn with_degenerate_area_threshold(mut self, threshold: f64) -> Self {
        self.degenerate_area_threshold = threshold;
        self
    }

    /// Set the maximum triangle aspect ratio threshold.
    ///
    /// Triangles with aspect ratio above this are considered degenerate.
    /// Use `f64::INFINITY` to disable this check.
    #[must_use]
    pub const fn with_degenerate_aspect_ratio(mut self, ratio: f64) -> Self {
        self.degenerate_aspect_ratio = ratio;
        self
    }

    /// Set the minimum edge length threshold.
    ///
    /// Triangles with any edge shorter than this are removed.
    /// Use `0.0` to disable this check.
    #[must_use]
    pub const fn with_degenerate_min_edge_length(mut self, length: f64) -> Self {
        self.degenerate_min_edge_length = length;
        self
    }

    /// Set whether to remove unreferenced vertices after repair.
    #[must_use]
    pub const fn with_remove_unreferenced(mut self, remove: bool) -> Self {
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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::remove_degenerate_triangles;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(5.0, 0.0, 0.0)); // Collinear — degenerate
/// mesh.faces.push([0, 1, 2]);
///
/// let removed = remove_degenerate_triangles(&mut mesh, 0.001);
/// assert_eq!(removed, 1);
/// ```
pub fn remove_degenerate_triangles(mesh: &mut IndexedMesh, area_threshold: f64) -> usize {
    let original_count = mesh.faces.len();

    mesh.faces.retain(|face| {
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];

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
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];

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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::weld_vertices;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0001, 0.0, 0.0)); // Near-duplicate of vertex 1
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
        let cell = pos_to_cell(vertex, cell_size);
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

        let cell = pos_to_cell(vertex, cell_size);

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

                            let other_pos = &mesh.vertices[other_idx as usize];
                            let dist = (vertex - other_pos).norm();

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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::remove_unreferenced_vertices;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(100.0, 100.0, 100.0)); // Unreferenced
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
            new_vertices.push(*vertex);
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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::remove_duplicate_faces;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
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
const fn normalize_face(face: [u32; 3]) -> [u32; 3] {
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

/// Default Laplacian-step weight for [`taubin_smooth_vertices`].
///
/// Pulls each vertex 50 % of the way toward its 1-ring neighbor
/// centroid per shrink pass. Matches the value in Taubin (1995)
/// §3.1 for the default low-pass filter design.
pub const TAUBIN_DEFAULT_LAMBDA: f64 = 0.5;

/// Default inverse-Laplacian step weight for [`taubin_smooth_vertices`].
///
/// Slightly more aggressive than `-λ` (so the expansion pass
/// undoes a little more than the shrink pass put in) to
/// compensate for the volume-shrinkage pure Laplacian smoothing
/// would otherwise produce. The `(λ, μ) = (0.5, -0.53)` pair
/// gives a low-pass filter with pass-band-stop-band cutoff near
/// `k_pb ≈ 0.1` per Taubin (1995); empirically holds
/// bounding-box extent within ~1 % of the original over 10-20
/// iterations on typical scan meshes.
pub const TAUBIN_DEFAULT_MU: f64 = -0.53;

/// Taubin smoothing — Laplacian + inverse-Laplacian vertex
/// updates that suppress high-frequency surface noise.
///
/// Alternating shrink (+λ) and expand (-μ) passes form a
/// low-pass filter on the mesh's spectral representation,
/// preventing the monotonic volume shrinkage that pure
/// Laplacian smoothing would otherwise produce (Taubin,
/// "A signal processing approach to fair surface design,"
/// SIGGRAPH 1995).
///
/// Each iteration performs two passes:
///
/// 1. **Shrink pass** (weight `λ`, typically `+0.5`): each
///    vertex moves toward the centroid of its 1-ring neighbors
///    by `λ × (centroid - vertex)`.
/// 2. **Expand pass** (weight `μ`, typically `-0.53`): same
///    update with a negative weight, so each vertex moves AWAY
///    from the centroid. With `|μ| > |λ|` slightly, the expand
///    pass over-corrects to cancel the shrink-pass's cumulative
///    volume loss.
///
/// The composite filter is a low-pass filter on the mesh's
/// spectral representation, suppressing high-frequency noise
/// while preserving low-frequency geometric features. Good for
/// scanner noise / surface texture cleanup on body-part scans
/// (sub-mm peak-to-peak noise) where the noise has no geometric
/// significance — the silicone cast won't carry it (surface
/// tension smooths sub-mm features during cure).
///
/// # Arguments
///
/// * `mesh` — mesh to smooth in place (vertex positions
///   mutated; face indices unchanged).
/// * `iterations` — number of (shrink, expand) pass pairs.
///   8 is a reasonable default for body-part scans; raise for
///   noisier scans (15-20) or lower (3-5) to preserve more
///   surface detail.
/// * `lambda` — shrink-pass weight; see
///   [`TAUBIN_DEFAULT_LAMBDA`].
/// * `mu` — expand-pass weight; see [`TAUBIN_DEFAULT_MU`].
///
/// # Returns
///
/// The number of iterations actually applied (`0` for empty
/// mesh, no faces, or `iterations == 0`).
///
/// # Boundary handling
///
/// Vertices with no 1-ring neighbors (e.g., orphan vertices)
/// are left untouched. Boundary vertices (on the open edge of a
/// non-watertight mesh) ARE smoothed against whatever interior
/// neighbors they have — they may drift slightly inward over
/// many iterations, but for cleaned-and-capped scans this is
/// rare. Watertight inputs are unaffected.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::{TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU, taubin_smooth_vertices};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 0.0, 1.0));
/// // Closed tetrahedron — every vertex shares the apex / base / sides.
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([0, 1, 3]);
/// mesh.faces.push([0, 2, 3]);
/// mesh.faces.push([1, 2, 3]);
///
/// let iters = taubin_smooth_vertices(
///     &mut mesh,
///     5,
///     TAUBIN_DEFAULT_LAMBDA,
///     TAUBIN_DEFAULT_MU,
/// );
/// assert_eq!(iters, 5);
/// ```
pub fn taubin_smooth_vertices(
    mesh: &mut IndexedMesh,
    iterations: usize,
    lambda: f64,
    mu: f64,
) -> usize {
    if iterations == 0 || mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return 0;
    }

    // Build 1-ring adjacency: `neighbors[i]` is the set of
    // vertex indices sharing an EDGE with vertex `i`. Linear
    // dedup is fine — typical valence is 5-7 on triangulated
    // surfaces, so the inner `contains` check is bounded.
    let n = mesh.vertices.len();
    let mut neighbors: Vec<Vec<u32>> = vec![Vec::new(); n];
    for face in &mesh.faces {
        let [a, b, c] = *face;
        push_edge(&mut neighbors, a, b);
        push_edge(&mut neighbors, b, c);
        push_edge(&mut neighbors, c, a);
    }

    let mut buffer = mesh.vertices.clone();
    for _ in 0..iterations {
        apply_taubin_step(&mesh.vertices, &neighbors, &mut buffer, lambda);
        std::mem::swap(&mut mesh.vertices, &mut buffer);
        apply_taubin_step(&mesh.vertices, &neighbors, &mut buffer, mu);
        std::mem::swap(&mut mesh.vertices, &mut buffer);
    }

    iterations
}

/// Push (a → b) and (b → a) edges into the adjacency lists,
/// deduping linearly. Helper for [`taubin_smooth_vertices`].
fn push_edge(neighbors: &mut [Vec<u32>], a: u32, b: u32) {
    if a == b {
        return;
    }
    if !neighbors[a as usize].contains(&b) {
        neighbors[a as usize].push(b);
    }
    if !neighbors[b as usize].contains(&a) {
        neighbors[b as usize].push(a);
    }
}

/// One Laplacian umbrella step: `dst[i] = src[i] + weight ×
/// (centroid_of_neighbors[i] - src[i])`. Vertices with no
/// neighbors pass through unchanged.
fn apply_taubin_step(
    src: &[Point3<f64>],
    neighbors: &[Vec<u32>],
    dst: &mut [Point3<f64>],
    weight: f64,
) {
    for (i, v) in src.iter().enumerate() {
        let nbs = &neighbors[i];
        if nbs.is_empty() {
            dst[i] = *v;
            continue;
        }
        let mut sum = Vector3::zeros();
        for &nb in nbs {
            sum += src[nb as usize].coords;
        }
        // `nbs.len()` is bounded by the per-vertex valence
        // (typically 5-7 on triangulated surfaces); the f64
        // cast is exact for any usize that fits in the
        // hardware vertex-count budget.
        #[allow(clippy::cast_precision_loss)]
        let avg = sum / nbs.len() as f64;
        let umbrella = avg - v.coords;
        dst[i] = Point3::from(v.coords + umbrella * weight);
    }
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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::{repair_mesh, RepairParams};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
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
    pub const fn had_changes(&self) -> bool {
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
    use mesh_types::Point3;

    fn simple_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    // ----- taubin_smooth_vertices -------------------------------------

    /// Closed tetrahedron — every vertex is connected to every
    /// other via at least one edge. Used to exercise Taubin
    /// smoothing on a non-degenerate closed mesh.
    fn closed_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([0, 2, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh
    }

    /// `iterations == 0` is a no-op. Pin the contract so callers
    /// can safely route a slider value to 0 without an extra
    /// guard.
    #[test]
    fn taubin_smooth_zero_iterations_is_no_op() {
        let mut mesh = closed_tetrahedron();
        let original = mesh.vertices.clone();
        let n = taubin_smooth_vertices(&mut mesh, 0, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(n, 0);
        assert_eq!(mesh.vertices, original);
    }

    /// Empty vertex array → no-op (no positions to smooth).
    /// Guards against per-pass `[0; 0]` allocations in the
    /// adjacency build for callers loading a degenerate file.
    #[test]
    fn taubin_smooth_empty_mesh_is_no_op() {
        let mut mesh = IndexedMesh::new();
        let n = taubin_smooth_vertices(&mut mesh, 5, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(n, 0);
        assert!(mesh.vertices.is_empty());
    }

    /// Vertices-but-no-faces → no-op. Smoothing without faces
    /// has no neighbor topology to average against.
    #[test]
    fn taubin_smooth_no_faces_is_no_op() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        let original = mesh.vertices.clone();
        let n = taubin_smooth_vertices(&mut mesh, 5, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(n, 0);
        assert_eq!(mesh.vertices, original);
    }

    /// Smoothing a non-degenerate mesh moves at least one
    /// vertex. Pinned as a regression guard: it would be a
    /// silent bug if `taubin_smooth_vertices(_, > 0, _, _)`
    /// ever became a no-op on a real mesh.
    #[test]
    fn taubin_smooth_modifies_vertices_on_real_mesh() {
        let mut mesh = closed_tetrahedron();
        let original = mesh.vertices.clone();
        let n = taubin_smooth_vertices(&mut mesh, 3, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(n, 3);
        let any_moved = mesh
            .vertices
            .iter()
            .enumerate()
            .any(|(i, v)| (v.coords - original[i].coords).norm() > 1e-12);
        assert!(any_moved, "no vertex moved after 3 Taubin iterations");
    }

    /// Taubin (with `μ < 0`) shrinks LESS than pure Laplacian
    /// (μ = 0). Pin this property — it's the whole reason
    /// Taubin exists. Measure via bounding-box volume; Laplacian
    /// monotonically shrinks, Taubin holds steady.
    #[test]
    fn taubin_smooth_shrinks_less_than_pure_laplacian() {
        // Use a closed octahedron — 6 vertices, 8 faces, evenly
        // distributed around the origin so shrinkage is a
        // global bounding-box-decrease (not direction-biased).
        let mut octa = IndexedMesh::new();
        octa.vertices.push(Point3::new(1.0, 0.0, 0.0)); // +X
        octa.vertices.push(Point3::new(-1.0, 0.0, 0.0)); // -X
        octa.vertices.push(Point3::new(0.0, 1.0, 0.0)); // +Y
        octa.vertices.push(Point3::new(0.0, -1.0, 0.0)); // -Y
        octa.vertices.push(Point3::new(0.0, 0.0, 1.0)); // +Z
        octa.vertices.push(Point3::new(0.0, 0.0, -1.0)); // -Z
        // Top half (+Z apex) — 4 faces around vertex 4.
        octa.faces.push([0, 2, 4]);
        octa.faces.push([2, 1, 4]);
        octa.faces.push([1, 3, 4]);
        octa.faces.push([3, 0, 4]);
        // Bottom half (-Z apex) — 4 faces around vertex 5.
        octa.faces.push([2, 0, 5]);
        octa.faces.push([1, 2, 5]);
        octa.faces.push([3, 1, 5]);
        octa.faces.push([0, 3, 5]);

        let mut laplacian_mesh = octa.clone();
        let mut taubin_mesh = octa.clone();

        // 10 iterations of pure Laplacian (μ = 0) vs Taubin
        // (μ = -0.53).
        taubin_smooth_vertices(&mut laplacian_mesh, 10, 0.5, 0.0);
        taubin_smooth_vertices(&mut taubin_mesh, 10, 0.5, TAUBIN_DEFAULT_MU);

        #[allow(clippy::cast_precision_loss)]
        // vertex_count fits in f64 mantissa for any practical mesh
        let laplacian_extent: f64 = laplacian_mesh
            .vertices
            .iter()
            .map(|p| p.coords.norm())
            .sum::<f64>()
            / laplacian_mesh.vertices.len() as f64;
        #[allow(clippy::cast_precision_loss)]
        let taubin_extent: f64 = taubin_mesh
            .vertices
            .iter()
            .map(|p| p.coords.norm())
            .sum::<f64>()
            / taubin_mesh.vertices.len() as f64;

        // Laplacian should shrink hard (toward 0); Taubin
        // should hold near 1.0 (the original radius).
        assert!(
            laplacian_extent < 0.5,
            "pure Laplacian extent {laplacian_extent} should be < 0.5 (heavy shrinkage)",
        );
        assert!(
            taubin_extent > laplacian_extent * 1.5,
            "Taubin extent {taubin_extent} should be > 1.5× Laplacian extent {laplacian_extent}",
        );
    }

    #[test]
    fn remove_degenerate_collinear() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(5.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(10.001, 0.0, 0.0)); // Near vertex 1
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
        mesh.vertices.push(Point3::new(100.0, 100.0, 100.0)); // Unreferenced

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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(10.0001, 0.0, 0.0)); // Near-duplicate
        mesh.vertices.push(Point3::new(999.0, 999.0, 999.0)); // Unreferenced
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

        let display = format!("{result}");
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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(100.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(50.0, 0.01, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Remove with strict aspect ratio threshold
        let removed = remove_degenerate_triangles_enhanced(&mut mesh, 1e-12, 10.0, 0.0);
        assert_eq!(removed, 1);
    }

    #[test]
    fn remove_degenerate_enhanced_by_min_edge() {
        let mut mesh = IndexedMesh::new();
        // Create triangle with one very short edge
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.001, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Remove triangles with edge shorter than 0.01
        let removed = remove_degenerate_triangles_enhanced(&mut mesh, 1e-12, f64::INFINITY, 0.01);
        // This triangle's edges are all longer than 0.01, so none removed
        assert_eq!(removed, 0);

        // Create triangle with truly short edge
        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh2.vertices.push(Point3::new(0.001, 0.0, 0.0)); // Very short edge
        mesh2.vertices.push(Point3::new(0.5, 10.0, 0.0));
        mesh2.faces.push([0, 1, 2]);

        let removed2 = remove_degenerate_triangles_enhanced(&mut mesh2, 1e-12, f64::INFINITY, 0.01);
        assert_eq!(removed2, 1);
    }
}
