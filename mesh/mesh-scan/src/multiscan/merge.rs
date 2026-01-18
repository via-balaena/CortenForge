//! Scan merging with overlap handling.
//!
//! This module provides algorithms for merging multiple aligned scans
//! into a single unified mesh, handling overlapping regions and filling gaps.

use std::collections::HashSet;
use kiddo::{KdTree, SquaredEuclidean};
use mesh_registration::RigidTransform;
use mesh_types::{IndexedMesh, Vertex};
use nalgebra::Point3;

use crate::error::{ScanError, ScanResult};

/// Strategy for handling overlapping regions between scans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OverlapHandling {
    /// Keep all vertices, may result in duplicate geometry.
    KeepAll,

    /// Weld nearby vertices within threshold distance.
    #[default]
    Weld,

    /// Average positions of nearby vertices.
    Average,

    /// Keep only the first scan's vertices in overlap regions.
    FirstWins,

    /// Keep only the last scan's vertices in overlap regions.
    LastWins,
}

/// Parameters for scan merging.
#[derive(Debug, Clone)]
pub struct MergeParams {
    /// How to handle overlapping regions.
    pub overlap_handling: OverlapHandling,

    /// Distance threshold for welding vertices (default: 0.001).
    pub weld_threshold: f64,

    /// Whether to recompute vertex normals after merging (default: true).
    pub recompute_normals: bool,

    /// Whether to remove duplicate faces after merging (default: true).
    pub remove_duplicate_faces: bool,
}

impl Default for MergeParams {
    fn default() -> Self {
        Self {
            overlap_handling: OverlapHandling::Weld,
            weld_threshold: 0.001,
            recompute_normals: true,
            remove_duplicate_faces: true,
        }
    }
}

impl MergeParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the overlap handling strategy.
    #[must_use]
    pub const fn with_overlap_handling(mut self, handling: OverlapHandling) -> Self {
        self.overlap_handling = handling;
        self
    }

    /// Sets the weld threshold distance.
    #[must_use]
    pub const fn with_weld_threshold(mut self, threshold: f64) -> Self {
        self.weld_threshold = threshold;
        self
    }

    /// Sets whether to recompute normals.
    #[must_use]
    pub const fn with_recompute_normals(mut self, recompute: bool) -> Self {
        self.recompute_normals = recompute;
        self
    }

    /// Sets whether to remove duplicate faces.
    #[must_use]
    pub const fn with_remove_duplicate_faces(mut self, remove: bool) -> Self {
        self.remove_duplicate_faces = remove;
        self
    }

    /// Creates parameters for fast merging (keep all, no post-processing).
    #[must_use]
    pub const fn fast() -> Self {
        Self {
            overlap_handling: OverlapHandling::KeepAll,
            weld_threshold: 0.001,
            recompute_normals: false,
            remove_duplicate_faces: false,
        }
    }

    /// Creates parameters for high-quality merging.
    #[must_use]
    pub const fn high_quality() -> Self {
        Self {
            overlap_handling: OverlapHandling::Average,
            weld_threshold: 0.0005,
            recompute_normals: true,
            remove_duplicate_faces: true,
        }
    }
}

/// Result of scan merging.
#[derive(Debug, Clone)]
pub struct MergeResult {
    /// The merged mesh.
    pub mesh: IndexedMesh,

    /// Number of vertices before merging.
    pub original_vertex_count: usize,

    /// Number of vertices after merging.
    pub merged_vertex_count: usize,

    /// Number of vertices welded/removed.
    pub vertices_merged: usize,

    /// Number of duplicate faces removed.
    pub duplicate_faces_removed: usize,
}

impl std::fmt::Display for MergeResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Merge: {} → {} vertices ({} merged), {} duplicate faces removed",
            self.original_vertex_count,
            self.merged_vertex_count,
            self.vertices_merged,
            self.duplicate_faces_removed
        )
    }
}

/// Merges multiple scans into a single mesh.
///
/// # Arguments
///
/// * `scans` - Slice of meshes to merge
/// * `transforms` - Transform for each scan (should be same length as scans)
/// * `params` - Merge parameters
///
/// # Returns
///
/// The merged mesh with statistics.
///
/// # Errors
///
/// Returns an error if:
/// - Scans and transforms have different lengths
/// - No scans provided
///
/// # Example
///
/// ```
/// use mesh_scan::multiscan::{merge_scans, MergeParams};
/// use mesh_registration::RigidTransform;
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut scan1 = IndexedMesh::new();
/// scan1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// scan1.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// scan1.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// scan1.faces.push([0, 1, 2]);
///
/// let mut scan2 = IndexedMesh::new();
/// scan2.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// scan2.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
/// scan2.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
/// scan2.faces.push([0, 1, 2]);
///
/// let scans = vec![scan1, scan2];
/// let transforms = vec![RigidTransform::identity(), RigidTransform::identity()];
///
/// let result = merge_scans(&scans, &transforms, &MergeParams::default()).unwrap();
/// println!("{}", result);
/// ```
pub fn merge_scans(
    scans: &[IndexedMesh],
    transforms: &[RigidTransform],
    params: &MergeParams,
) -> ScanResult<MergeResult> {
    if scans.is_empty() {
        return Err(ScanError::EmptyMesh);
    }

    if scans.len() != transforms.len() {
        return Err(ScanError::InvalidParameter {
            reason: format!(
                "scans ({}) and transforms ({}) must have same length",
                scans.len(),
                transforms.len()
            ),
        });
    }

    // Count total vertices
    let original_vertex_count: usize = scans.iter().map(|s| s.vertices.len()).sum();

    // Transform all scans and collect vertices/faces
    let transformed_scans: Vec<IndexedMesh> = scans
        .iter()
        .zip(transforms.iter())
        .map(|(scan, transform)| transform_mesh(scan, transform))
        .collect();

    // Merge based on overlap handling
    let mut merged = match params.overlap_handling {
        OverlapHandling::KeepAll => merge_keep_all(&transformed_scans),
        OverlapHandling::Weld => merge_weld(&transformed_scans, params.weld_threshold),
        OverlapHandling::Average => merge_average(&transformed_scans, params.weld_threshold),
        OverlapHandling::FirstWins => merge_first_wins(&transformed_scans, params.weld_threshold),
        OverlapHandling::LastWins => merge_last_wins(&transformed_scans, params.weld_threshold),
    };

    let merged_vertex_count = merged.vertices.len();
    let vertices_merged = original_vertex_count.saturating_sub(merged_vertex_count);

    // Remove duplicate faces if requested
    let duplicate_faces_removed = if params.remove_duplicate_faces {
        remove_duplicate_faces(&mut merged)
    } else {
        0
    };

    // Recompute normals if requested and mesh has faces
    if params.recompute_normals && !merged.faces.is_empty() {
        compute_vertex_normals(&mut merged);
    }

    Ok(MergeResult {
        mesh: merged,
        original_vertex_count,
        merged_vertex_count,
        vertices_merged,
        duplicate_faces_removed,
    })
}

/// Transforms a mesh by a rigid transform.
fn transform_mesh(mesh: &IndexedMesh, transform: &RigidTransform) -> IndexedMesh {
    let mut result = mesh.clone();
    for v in &mut result.vertices {
        v.position = transform.transform_point(&v.position);
        if let Some(normal) = v.attributes.normal {
            v.attributes.normal = Some(transform.rotation * normal);
        }
    }
    result
}

/// Merges scans keeping all vertices.
fn merge_keep_all(scans: &[IndexedMesh]) -> IndexedMesh {
    let mut merged = IndexedMesh::new();
    let mut vertex_offset = 0u32;

    for scan in scans {
        // Add vertices
        merged.vertices.extend(scan.vertices.iter().cloned());

        // Add faces with offset indices
        for face in &scan.faces {
            merged.faces.push([
                face[0] + vertex_offset,
                face[1] + vertex_offset,
                face[2] + vertex_offset,
            ]);
        }

        #[allow(clippy::cast_possible_truncation)]
        {
            vertex_offset += scan.vertices.len() as u32;
        }
    }

    merged
}

/// Merges scans welding nearby vertices.
fn merge_weld(scans: &[IndexedMesh], threshold: f64) -> IndexedMesh {
    let threshold_sq = threshold * threshold;

    // Collect all vertices with their scan/index info
    let mut all_vertices: Vec<(Point3<f64>, usize, usize)> = Vec::new();
    for (scan_idx, scan) in scans.iter().enumerate() {
        for (v_idx, v) in scan.vertices.iter().enumerate() {
            all_vertices.push((v.position, scan_idx, v_idx));
        }
    }

    if all_vertices.is_empty() {
        return IndexedMesh::new();
    }

    // Build KD-tree
    let mut tree: KdTree<f64, 3> = KdTree::new();
    for (i, (p, _, _)) in all_vertices.iter().enumerate() {
        tree.add(&[p.x, p.y, p.z], i as u64);
    }

    // Find vertex mapping (old index -> new index)
    let mut vertex_map: Vec<Option<u32>> = vec![None; all_vertices.len()];
    let mut merged = IndexedMesh::new();

    for (i, (pos, scan_idx, v_idx)) in all_vertices.iter().enumerate() {
        if vertex_map[i].is_some() {
            continue; // Already mapped
        }

        // Find all nearby vertices
        let nearby = tree.within::<SquaredEuclidean>(&[pos.x, pos.y, pos.z], threshold_sq);

        // Get the original vertex attributes
        let original_vertex = &scans[*scan_idx].vertices[*v_idx];

        // Add this vertex to merged mesh
        #[allow(clippy::cast_possible_truncation)]
        let new_idx = merged.vertices.len() as u32;
        merged.vertices.push(original_vertex.clone());

        // Map all nearby vertices to this one
        for neighbor in nearby {
            #[allow(clippy::cast_possible_truncation)]
            let neighbor_idx = neighbor.item as usize;
            if vertex_map[neighbor_idx].is_none() {
                vertex_map[neighbor_idx] = Some(new_idx);
            }
        }
    }

    // Build cumulative vertex offsets for face remapping
    let mut scan_offsets: Vec<usize> = vec![0];
    for scan in scans.iter().take(scans.len().saturating_sub(1)) {
        scan_offsets.push(scan_offsets.last().copied().unwrap_or(0) + scan.vertices.len());
    }

    // Add faces with remapped indices
    for (scan_idx, scan) in scans.iter().enumerate() {
        let offset = scan_offsets.get(scan_idx).copied().unwrap_or(0);

        for face in &scan.faces {
            let new_face = [
                vertex_map[offset + face[0] as usize].unwrap_or(0),
                vertex_map[offset + face[1] as usize].unwrap_or(0),
                vertex_map[offset + face[2] as usize].unwrap_or(0),
            ];

            // Skip degenerate faces
            if new_face[0] != new_face[1]
                && new_face[1] != new_face[2]
                && new_face[0] != new_face[2]
            {
                merged.faces.push(new_face);
            }
        }
    }

    merged
}

/// Merges scans averaging nearby vertices.
fn merge_average(scans: &[IndexedMesh], threshold: f64) -> IndexedMesh {
    let threshold_sq = threshold * threshold;

    // Collect all vertices
    let mut all_vertices: Vec<Vertex> = Vec::new();
    let mut scan_offsets: Vec<usize> = vec![0];

    for scan in scans {
        all_vertices.extend(scan.vertices.iter().cloned());
        scan_offsets.push(all_vertices.len());
    }

    if all_vertices.is_empty() {
        return IndexedMesh::new();
    }

    // Build KD-tree
    let mut tree: KdTree<f64, 3> = KdTree::new();
    for (i, v) in all_vertices.iter().enumerate() {
        tree.add(&[v.position.x, v.position.y, v.position.z], i as u64);
    }

    // Find vertex clusters and compute averages
    let mut visited = vec![false; all_vertices.len()];
    let mut vertex_map: Vec<Option<u32>> = vec![None; all_vertices.len()];
    let mut merged = IndexedMesh::new();

    for i in 0..all_vertices.len() {
        if visited[i] {
            continue;
        }

        let pos = all_vertices[i].position;
        let nearby = tree.within::<SquaredEuclidean>(&[pos.x, pos.y, pos.z], threshold_sq);

        // Compute average position
        let mut sum = nalgebra::Vector3::zeros();
        let mut count = 0usize;

        for neighbor in &nearby {
            #[allow(clippy::cast_possible_truncation)]
            let idx = neighbor.item as usize;
            sum += all_vertices[idx].position.coords;
            count += 1;
            visited[idx] = true;
        }

        #[allow(clippy::cast_precision_loss)]
        let avg_pos = Point3::from(sum / count as f64);

        // Create averaged vertex (keep first vertex's attributes)
        let mut new_vertex = all_vertices[i].clone();
        new_vertex.position = avg_pos;

        #[allow(clippy::cast_possible_truncation)]
        let new_idx = merged.vertices.len() as u32;
        merged.vertices.push(new_vertex);

        // Map all nearby vertices
        for neighbor in nearby {
            #[allow(clippy::cast_possible_truncation)]
            let idx = neighbor.item as usize;
            vertex_map[idx] = Some(new_idx);
        }
    }

    // Add faces with remapped indices
    for (scan_idx, scan) in scans.iter().enumerate() {
        let offset = scan_offsets.get(scan_idx).copied().unwrap_or(0);

        for face in &scan.faces {
            let new_face = [
                vertex_map[offset + face[0] as usize].unwrap_or(0),
                vertex_map[offset + face[1] as usize].unwrap_or(0),
                vertex_map[offset + face[2] as usize].unwrap_or(0),
            ];

            if new_face[0] != new_face[1]
                && new_face[1] != new_face[2]
                && new_face[0] != new_face[2]
            {
                merged.faces.push(new_face);
            }
        }
    }

    merged
}

/// Merges scans keeping first scan's vertices in overlaps.
fn merge_first_wins(scans: &[IndexedMesh], threshold: f64) -> IndexedMesh {
    merge_with_priority(scans, threshold, true)
}

/// Merges scans keeping last scan's vertices in overlaps.
fn merge_last_wins(scans: &[IndexedMesh], threshold: f64) -> IndexedMesh {
    merge_with_priority(scans, threshold, false)
}

/// Merges with priority to first or last scan.
fn merge_with_priority(scans: &[IndexedMesh], threshold: f64, first_wins: bool) -> IndexedMesh {
    let threshold_sq = threshold * threshold;

    let ordered_scans: Vec<&IndexedMesh> = if first_wins {
        scans.iter().collect()
    } else {
        scans.iter().rev().collect()
    };

    let mut merged = IndexedMesh::new();
    let mut merged_tree: KdTree<f64, 3> = KdTree::new();
    let mut scan_vertex_maps: Vec<Vec<u32>> = Vec::new();

    for scan in &ordered_scans {
        let mut vertex_map = Vec::with_capacity(scan.vertices.len());

        for v in &scan.vertices {
            // Check if there's already a nearby vertex
            let nearest =
                merged_tree.nearest_one::<SquaredEuclidean>(&[v.position.x, v.position.y, v.position.z]);

            #[allow(clippy::cast_possible_truncation)]
            if nearest.distance <= threshold_sq && !merged.vertices.is_empty() {
                // Map to existing vertex
                vertex_map.push(nearest.item as u32);
            } else {
                // Add new vertex
                let new_idx = merged.vertices.len() as u32;
                merged_tree.add(&[v.position.x, v.position.y, v.position.z], u64::from(new_idx));
                merged.vertices.push(v.clone());
                vertex_map.push(new_idx);
            }
        }

        scan_vertex_maps.push(vertex_map);
    }

    // Add faces
    for (scan_idx, scan) in ordered_scans.iter().enumerate() {
        let vertex_map = &scan_vertex_maps[scan_idx];

        for face in &scan.faces {
            let new_face = [
                vertex_map[face[0] as usize],
                vertex_map[face[1] as usize],
                vertex_map[face[2] as usize],
            ];

            if new_face[0] != new_face[1]
                && new_face[1] != new_face[2]
                && new_face[0] != new_face[2]
            {
                merged.faces.push(new_face);
            }
        }
    }

    merged
}

/// Removes duplicate faces from a mesh.
fn remove_duplicate_faces(mesh: &mut IndexedMesh) -> usize {
    let original_count = mesh.faces.len();

    let mut seen: HashSet<[u32; 3]> = HashSet::new();
    mesh.faces.retain(|face| {
        // Normalize face by sorting indices
        let mut sorted = *face;
        sorted.sort_unstable();
        seen.insert(sorted)
    });

    original_count - mesh.faces.len()
}

/// Computes vertex normals from face normals.
fn compute_vertex_normals(mesh: &mut IndexedMesh) {
    // Initialize normals to zero
    let mut normals = vec![nalgebra::Vector3::zeros(); mesh.vertices.len()];

    // Accumulate face normals
    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let face_normal = e1.cross(&e2);

        for &idx in face {
            normals[idx as usize] += face_normal;
        }
    }

    // Normalize and assign
    for (i, normal) in normals.iter().enumerate() {
        let len = normal.norm();
        if len > 1e-10 {
            mesh.vertices[i].attributes.normal = Some(normal / len);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn make_simple_mesh(offset_x: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0 + offset_x, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0 + offset_x, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5 + offset_x, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn test_merge_params_default() {
        let params = MergeParams::default();
        assert_eq!(params.overlap_handling, OverlapHandling::Weld);
        assert!(params.recompute_normals);
    }

    #[test]
    fn test_merge_params_builder() {
        let params = MergeParams::new()
            .with_overlap_handling(OverlapHandling::Average)
            .with_weld_threshold(0.01)
            .with_recompute_normals(false);

        assert_eq!(params.overlap_handling, OverlapHandling::Average);
        assert_relative_eq!(params.weld_threshold, 0.01);
        assert!(!params.recompute_normals);
    }

    #[test]
    fn test_merge_params_presets() {
        let fast = MergeParams::fast();
        assert_eq!(fast.overlap_handling, OverlapHandling::KeepAll);

        let hq = MergeParams::high_quality();
        assert_eq!(hq.overlap_handling, OverlapHandling::Average);
    }

    #[test]
    fn test_merge_empty_scans() {
        let scans: Vec<IndexedMesh> = Vec::new();
        let transforms: Vec<RigidTransform> = Vec::new();
        let result = merge_scans(&scans, &transforms, &MergeParams::default());
        assert!(matches!(result, Err(ScanError::EmptyMesh)));
    }

    #[test]
    fn test_merge_mismatched_counts() {
        let scans = vec![make_simple_mesh(0.0)];
        let transforms = vec![RigidTransform::identity(), RigidTransform::identity()];
        let result = merge_scans(&scans, &transforms, &MergeParams::default());
        assert!(matches!(result, Err(ScanError::InvalidParameter { .. })));
    }

    #[test]
    fn test_merge_keep_all() {
        let scans = vec![make_simple_mesh(0.0), make_simple_mesh(0.5)];
        let transforms = vec![RigidTransform::identity(), RigidTransform::identity()];
        let params = MergeParams::new()
            .with_overlap_handling(OverlapHandling::KeepAll)
            .with_recompute_normals(false);

        let result = merge_scans(&scans, &transforms, &params).unwrap();

        assert_eq!(result.original_vertex_count, 6);
        assert_eq!(result.merged_vertex_count, 6);
        assert_eq!(result.mesh.faces.len(), 2);
    }

    #[test]
    fn test_merge_weld_overlapping() {
        // Two meshes that share a vertex
        let mut mesh1 = IndexedMesh::new();
        mesh1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh1.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh1.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh1.faces.push([0, 1, 2]);

        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0)); // Same as mesh1 vertex 1
        mesh2.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
        mesh2.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        mesh2.faces.push([0, 1, 2]);

        let scans = vec![mesh1, mesh2];
        let transforms = vec![RigidTransform::identity(), RigidTransform::identity()];
        let params = MergeParams::new()
            .with_overlap_handling(OverlapHandling::Weld)
            .with_weld_threshold(0.01)
            .with_recompute_normals(false);

        let result = merge_scans(&scans, &transforms, &params).unwrap();

        // Should have 5 vertices (6 - 1 welded)
        assert_eq!(result.merged_vertex_count, 5);
        assert_eq!(result.vertices_merged, 1);
    }

    #[test]
    fn test_merge_average() {
        let mut mesh1 = IndexedMesh::new();
        mesh1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Vertex::from_coords(0.002, 0.0, 0.0));

        let scans = vec![mesh1, mesh2];
        let transforms = vec![RigidTransform::identity(), RigidTransform::identity()];
        let params = MergeParams::new()
            .with_overlap_handling(OverlapHandling::Average)
            .with_weld_threshold(0.01)
            .with_recompute_normals(false);

        let result = merge_scans(&scans, &transforms, &params).unwrap();

        // Should have 1 vertex at averaged position
        assert_eq!(result.merged_vertex_count, 1);
        assert_relative_eq!(result.mesh.vertices[0].position.x, 0.001, epsilon = 1e-10);
    }

    #[test]
    fn test_merge_with_transform() {
        let mesh = make_simple_mesh(0.0);
        let scans = vec![mesh.clone(), mesh];

        let transforms = vec![
            RigidTransform::identity(),
            RigidTransform::from_translation(nalgebra::Vector3::new(10.0, 0.0, 0.0)),
        ];

        let params = MergeParams::new()
            .with_overlap_handling(OverlapHandling::KeepAll)
            .with_recompute_normals(false);

        let result = merge_scans(&scans, &transforms, &params).unwrap();

        // Should have all 6 vertices since they don't overlap
        assert_eq!(result.merged_vertex_count, 6);

        // Check second mesh is transformed
        let last_vertex = &result.mesh.vertices[5];
        assert_relative_eq!(last_vertex.position.x, 10.5, epsilon = 1e-10);
    }

    #[test]
    fn test_remove_duplicate_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 2]); // Duplicate
        mesh.faces.push([2, 1, 0]); // Same face, different order (will be normalized)

        let removed = remove_duplicate_faces(&mut mesh);

        assert_eq!(removed, 2);
        assert_eq!(mesh.faces.len(), 1);
    }

    #[test]
    fn test_merge_result_display() {
        let result = MergeResult {
            mesh: IndexedMesh::new(),
            original_vertex_count: 100,
            merged_vertex_count: 80,
            vertices_merged: 20,
            duplicate_faces_removed: 5,
        };

        let display = format!("{result}");
        assert!(display.contains("100"));
        assert!(display.contains("80"));
        assert!(display.contains("20 merged"));
    }
}
