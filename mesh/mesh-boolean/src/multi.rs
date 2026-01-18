//! Multi-mesh boolean operations using tree-based parallel processing.
//!
//! When combining many meshes, sequential boolean operations are O(n²) in
//! worst case because each operation processes the growing result mesh.
//!
//! This module provides tree-based algorithms that:
//! 1. Process pairs of meshes in parallel using rayon
//! 2. Merge results pairwise up the tree
//! 3. Achieve O(n log n) total complexity for n meshes
//!
//! # Example
//!
//! ```ignore
//! use mesh_boolean::{multi_union, BooleanConfig};
//!
//! let meshes = vec![mesh1, mesh2, mesh3, mesh4, mesh5];
//! let result = multi_union(&meshes, &BooleanConfig::default())?;
//! ```

#![allow(clippy::doc_markdown)]

use crate::config::{BooleanConfig, BooleanOp};
use crate::error::{BooleanError, BooleanResult};
use crate::operation::boolean_operation;
use mesh_types::IndexedMesh;
use rayon::prelude::*;

/// Result of a multi-mesh boolean operation.
#[derive(Debug)]
pub struct MultiMeshResult {
    /// The resulting mesh.
    pub mesh: IndexedMesh,
    /// Aggregated statistics.
    pub stats: MultiMeshStats,
}

/// Statistics from a multi-mesh boolean operation.
#[derive(Debug, Clone, Default)]
pub struct MultiMeshStats {
    /// Number of input meshes.
    pub input_count: usize,
    /// Number of pairwise operations performed.
    pub operations_performed: usize,
    /// Total faces in all input meshes.
    pub total_input_faces: usize,
    /// Faces in final result.
    pub result_faces: usize,
    /// Maximum tree depth.
    pub tree_depth: usize,
}

/// Perform union of multiple meshes using parallel tree reduction.
///
/// This is more efficient than sequential unions for large numbers of meshes.
/// Uses a tree-based approach that processes pairs in parallel, then merges
/// results up the tree.
///
/// # Arguments
///
/// * `meshes` - Slice of meshes to union
/// * `config` - Configuration for boolean operations
///
/// # Returns
///
/// `MultiMeshResult` containing the combined mesh and statistics.
///
/// # Errors
///
/// Returns `BooleanError` if:
/// - No meshes are provided
/// - Any boolean operation fails
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::{multi_union, BooleanConfig};
///
/// let meshes = vec![sphere1, sphere2, sphere3, sphere4];
/// let result = multi_union(&meshes, &BooleanConfig::default())?;
/// ```
pub fn multi_union(meshes: &[IndexedMesh], config: &BooleanConfig) -> BooleanResult<MultiMeshResult> {
    multi_boolean(meshes, BooleanOp::Union, config)
}

/// Perform intersection of multiple meshes using parallel tree reduction.
///
/// # Arguments
///
/// * `meshes` - Slice of meshes to intersect
/// * `config` - Configuration for boolean operations
///
/// # Returns
///
/// `MultiMeshResult` containing the intersected mesh and statistics.
///
/// # Errors
///
/// Returns `BooleanError` if no meshes are provided or any operation fails.
pub fn multi_intersection(
    meshes: &[IndexedMesh],
    config: &BooleanConfig,
) -> BooleanResult<MultiMeshResult> {
    multi_boolean(meshes, BooleanOp::Intersection, config)
}

/// Perform a boolean operation on multiple meshes using parallel tree reduction.
///
/// Note: Difference is not associative, so multi_difference is not provided.
/// For multiple differences, use sequential operations:
/// `A - B - C = (A - B) - C`
fn multi_boolean(
    meshes: &[IndexedMesh],
    operation: BooleanOp,
    config: &BooleanConfig,
) -> BooleanResult<MultiMeshResult> {
    if meshes.is_empty() {
        return Err(BooleanError::EmptyMesh {
            details: "No meshes provided for multi-mesh operation".to_string(),
        });
    }

    if meshes.len() == 1 {
        return Ok(MultiMeshResult {
            mesh: meshes[0].clone(),
            stats: MultiMeshStats {
                input_count: 1,
                operations_performed: 0,
                total_input_faces: meshes[0].faces.len(),
                result_faces: meshes[0].faces.len(),
                tree_depth: 0,
            },
        });
    }

    let total_input_faces: usize = meshes.iter().map(|m| m.faces.len()).sum();
    let input_count = meshes.len();

    // Convert to owned meshes for tree processing
    let owned: Vec<IndexedMesh> = meshes.to_vec();

    // Process using tree reduction
    let (result, operations, depth) = if config.parallel {
        tree_reduce_parallel(owned, operation, config)?
    } else {
        tree_reduce_sequential(owned, operation, config)?
    };

    Ok(MultiMeshResult {
        mesh: result.clone(),
        stats: MultiMeshStats {
            input_count,
            operations_performed: operations,
            total_input_faces,
            result_faces: result.faces.len(),
            tree_depth: depth,
        },
    })
}

/// Sequential tree reduction (for testing or when parallelism is disabled).
fn tree_reduce_sequential(
    mut meshes: Vec<IndexedMesh>,
    operation: BooleanOp,
    config: &BooleanConfig,
) -> BooleanResult<(IndexedMesh, usize, usize)> {
    let mut operations = 0;
    let mut depth = 0;

    while meshes.len() > 1 {
        let mut next_level = Vec::with_capacity(meshes.len().div_ceil(2));

        let mut i = 0;
        while i < meshes.len() {
            if i + 1 < meshes.len() {
                // Combine pair
                let result = boolean_operation(&meshes[i], &meshes[i + 1], operation, config)?;
                next_level.push(result.mesh);
                operations += 1;
                i += 2;
            } else {
                // Odd one out - pass through
                next_level.push(meshes[i].clone());
                i += 1;
            }
        }

        meshes = next_level;
        depth += 1;
    }

    Ok((meshes.into_iter().next().unwrap_or_default(), operations, depth))
}

/// Parallel tree reduction using rayon.
fn tree_reduce_parallel(
    mut meshes: Vec<IndexedMesh>,
    operation: BooleanOp,
    config: &BooleanConfig,
) -> BooleanResult<(IndexedMesh, usize, usize)> {
    let mut total_operations = 0;
    let mut depth = 0;

    while meshes.len() > 1 {
        // Create pairs for parallel processing
        let pairs: Vec<(usize, Option<usize>)> = (0..meshes.len())
            .step_by(2)
            .map(|i| {
                if i + 1 < meshes.len() {
                    (i, Some(i + 1))
                } else {
                    (i, None)
                }
            })
            .collect();

        // Process pairs in parallel
        let results: Vec<BooleanResult<IndexedMesh>> = pairs
            .par_iter()
            .map(|&(i, j)| {
                if let Some(j) = j {
                    let result = boolean_operation(&meshes[i], &meshes[j], operation, config)?;
                    Ok(result.mesh)
                } else {
                    Ok(meshes[i].clone())
                }
            })
            .collect();

        // Collect results, propagating any errors
        let mut next_level = Vec::with_capacity(results.len());

        for result in results {
            next_level.push(result?);
        }

        // Count operations (pairs that were actually combined)
        let level_operations = pairs.iter().filter(|(_, j)| j.is_some()).count();
        total_operations += level_operations;

        meshes = next_level;
        depth += 1;
    }

    Ok((
        meshes.into_iter().next().unwrap_or_default(),
        total_operations,
        depth,
    ))
}

/// Perform sequential differences: A - B - C - D = ((A - B) - C) - D
///
/// Unlike union/intersection, difference is not associative, so tree-based
/// parallel processing doesn't apply. This function provides a convenient
/// sequential implementation.
///
/// # Arguments
///
/// * `base` - The base mesh to subtract from
/// * `subtrahends` - Meshes to subtract in order
/// * `config` - Configuration for boolean operations
///
/// # Returns
///
/// `MultiMeshResult` containing the resulting mesh and statistics.
///
/// # Errors
///
/// Returns `BooleanError` if any difference operation fails.
pub fn sequential_difference(
    base: &IndexedMesh,
    subtrahends: &[IndexedMesh],
    config: &BooleanConfig,
) -> BooleanResult<MultiMeshResult> {
    if subtrahends.is_empty() {
        return Ok(MultiMeshResult {
            mesh: base.clone(),
            stats: MultiMeshStats {
                input_count: 1,
                operations_performed: 0,
                total_input_faces: base.faces.len(),
                result_faces: base.faces.len(),
                tree_depth: 0,
            },
        });
    }

    let total_input_faces: usize =
        base.faces.len() + subtrahends.iter().map(|m| m.faces.len()).sum::<usize>();
    let input_count = 1 + subtrahends.len();

    let mut result = base.clone();
    let mut operations = 0;

    for subtrahend in subtrahends {
        let op_result = boolean_operation(&result, subtrahend, BooleanOp::Difference, config)?;
        result = op_result.mesh;
        operations += 1;
    }

    Ok(MultiMeshResult {
        mesh: result.clone(),
        stats: MultiMeshStats {
            input_count,
            operations_performed: operations,
            total_input_faces,
            result_faces: result.faces.len(),
            tree_depth: operations, // Linear depth for sequential
        },
    })
}

/// Simple mesh concatenation without boolean operations.
///
/// Useful when you know meshes don't overlap and just want to combine
/// them into a single mesh data structure.
///
/// # Arguments
///
/// * `meshes` - Meshes to concatenate
///
/// # Returns
///
/// A single mesh containing all vertices and faces from input meshes.
#[must_use]
pub fn concatenate_meshes(meshes: &[IndexedMesh]) -> IndexedMesh {
    if meshes.is_empty() {
        return IndexedMesh::new();
    }

    if meshes.len() == 1 {
        return meshes[0].clone();
    }

    // Calculate total sizes
    let total_vertices: usize = meshes.iter().map(|m| m.vertices.len()).sum();
    let total_faces: usize = meshes.iter().map(|m| m.faces.len()).sum();

    let mut result = IndexedMesh::new();
    result.vertices.reserve(total_vertices);
    result.faces.reserve(total_faces);

    let mut vertex_offset = 0u32;

    for mesh in meshes {
        // Add vertices
        result.vertices.extend(mesh.vertices.iter().cloned());

        // Add faces with offset
        for face in &mesh.faces {
            result.faces.push([
                face[0] + vertex_offset,
                face[1] + vertex_offset,
                face[2] + vertex_offset,
            ]);
        }

        vertex_offset += mesh.vertices.len() as u32;
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{Point3, Vertex};

    fn create_small_cube(offset_x: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        let size = 0.5;
        let vertices = [
            Point3::new(offset_x, 0.0, 0.0),
            Point3::new(offset_x + size, 0.0, 0.0),
            Point3::new(offset_x + size, size, 0.0),
            Point3::new(offset_x, size, 0.0),
            Point3::new(offset_x, 0.0, size),
            Point3::new(offset_x + size, 0.0, size),
            Point3::new(offset_x + size, size, size),
            Point3::new(offset_x, size, size),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_multi_union_single() {
        let mesh = create_small_cube(0.0);
        let result = multi_union(&[mesh.clone()], &BooleanConfig::default()).unwrap();

        assert_eq!(result.mesh.faces.len(), 12);
        assert_eq!(result.stats.operations_performed, 0);
    }

    #[test]
    fn test_multi_union_two() {
        let mesh1 = create_small_cube(0.0);
        let mesh2 = create_small_cube(2.0); // Non-overlapping

        let result = multi_union(&[mesh1, mesh2], &BooleanConfig::default()).unwrap();

        assert_eq!(result.mesh.faces.len(), 24);
        assert_eq!(result.stats.operations_performed, 1);
    }

    #[test]
    fn test_multi_union_four_non_overlapping() {
        let meshes: Vec<IndexedMesh> = (0..4).map(|i| create_small_cube(i as f64 * 2.0)).collect();

        let result = multi_union(&meshes, &BooleanConfig::default()).unwrap();

        // 4 cubes × 12 faces each
        assert_eq!(result.mesh.faces.len(), 48);
        assert_eq!(result.stats.input_count, 4);
        // Tree depth should be 2 (4 -> 2 -> 1)
        assert_eq!(result.stats.tree_depth, 2);
    }

    #[test]
    fn test_multi_union_empty() {
        let result = multi_union(&[], &BooleanConfig::default());
        assert!(result.is_err());
    }

    #[test]
    fn test_multi_intersection_two_non_overlapping() {
        let mesh1 = create_small_cube(0.0);
        let mesh2 = create_small_cube(5.0); // Far apart

        let result = multi_intersection(&[mesh1, mesh2], &BooleanConfig::default()).unwrap();

        // No intersection - empty result
        assert_eq!(result.mesh.faces.len(), 0);
    }

    #[test]
    fn test_sequential_difference() {
        let base = create_small_cube(0.0);
        let sub1 = create_small_cube(5.0); // Non-overlapping
        let sub2 = create_small_cube(10.0); // Non-overlapping

        let result =
            sequential_difference(&base, &[sub1, sub2], &BooleanConfig::default()).unwrap();

        // Subtrahends don't overlap, so result should equal base
        assert_eq!(result.mesh.faces.len(), 12);
        assert_eq!(result.stats.operations_performed, 2);
    }

    #[test]
    fn test_sequential_difference_empty_subtrahends() {
        let base = create_small_cube(0.0);

        let result = sequential_difference(&base, &[], &BooleanConfig::default()).unwrap();

        assert_eq!(result.mesh.faces.len(), 12);
        assert_eq!(result.stats.operations_performed, 0);
    }

    #[test]
    fn test_concatenate_meshes() {
        let mesh1 = create_small_cube(0.0);
        let mesh2 = create_small_cube(2.0);

        let result = concatenate_meshes(&[mesh1, mesh2]);

        assert_eq!(result.vertices.len(), 16);
        assert_eq!(result.faces.len(), 24);

        // Check that face indices are properly offset
        // Second mesh's faces should reference vertices 8-15
        for face in result.faces.iter().skip(12) {
            assert!(face[0] >= 8);
            assert!(face[1] >= 8);
            assert!(face[2] >= 8);
        }
    }

    #[test]
    fn test_concatenate_empty() {
        let result = concatenate_meshes(&[]);
        assert!(result.vertices.is_empty());
        assert!(result.faces.is_empty());
    }

    #[test]
    fn test_concatenate_single() {
        let mesh = create_small_cube(0.0);
        let result = concatenate_meshes(&[mesh.clone()]);

        assert_eq!(result.vertices.len(), mesh.vertices.len());
        assert_eq!(result.faces.len(), mesh.faces.len());
    }

    #[test]
    fn test_sequential_vs_parallel() {
        let meshes: Vec<IndexedMesh> = (0..4).map(|i| create_small_cube(i as f64 * 2.0)).collect();

        let config_seq = BooleanConfig::default().with_parallel(false);
        let config_par = BooleanConfig::default().with_parallel(true);

        let result_seq = multi_union(&meshes, &config_seq).unwrap();
        let result_par = multi_union(&meshes, &config_par).unwrap();

        // Results should be equivalent
        assert_eq!(result_seq.mesh.faces.len(), result_par.mesh.faces.len());
        assert_eq!(
            result_seq.stats.operations_performed,
            result_par.stats.operations_performed
        );
    }
}
