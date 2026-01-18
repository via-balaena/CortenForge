//! Core boolean operations: union, intersection, and difference.
//!
//! This module provides the main boolean operation functions that combine
//! two meshes using CSG (Constructive Solid Geometry) operations.

// Match arms are clearer separate even if they have identical bodies
#![allow(clippy::match_same_arms)]

use crate::bvh::Bvh;
use crate::classify::{classify_faces_auto, meshes_overlap, FaceLocation};
use crate::config::{BooleanConfig, BooleanOp, CleanupLevel, CoplanarStrategy};
use crate::coplanar::{coplanar_faces_a, coplanar_faces_b, find_coplanar_pairs, should_include_coplanar_face};
use crate::edge_insert::insert_edges;
use crate::error::{BooleanError, BooleanResult};
use crate::intersect::triangles_intersect;
use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Vertex};

/// Statistics from a boolean operation.
#[derive(Debug, Clone, Default)]
pub struct BooleanStats {
    /// Faces from mesh A in result.
    pub faces_from_a: usize,
    /// Faces from mesh B in result.
    pub faces_from_b: usize,
    /// Number of face pairs that were coplanar.
    pub coplanar_pairs: usize,
    /// Number of faces split by edge insertion.
    pub faces_split: usize,
    /// Number of new vertices created.
    pub new_vertices: usize,
    /// Whether meshes actually intersected.
    pub meshes_intersected: bool,
}

/// Result of a boolean operation.
#[derive(Debug)]
pub struct BooleanOperationResult {
    /// The resulting mesh.
    pub mesh: IndexedMesh,
    /// Statistics about the operation.
    pub stats: BooleanStats,
}

/// Perform a boolean operation on two meshes.
///
/// # Arguments
///
/// * `mesh_a` - First mesh (the "base" for difference operations)
/// * `mesh_b` - Second mesh (subtracted in difference operations)
/// * `operation` - The boolean operation to perform
/// * `config` - Configuration controlling tolerances and cleanup
///
/// # Returns
///
/// `BooleanOperationResult` containing the result mesh and statistics.
///
/// # Errors
///
/// Returns `BooleanError` if:
/// - Either mesh is empty
/// - Numerical errors occur during intersection detection
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::{boolean_operation, BooleanOp, BooleanConfig};
///
/// let result = boolean_operation(&mesh_a, &mesh_b, BooleanOp::Union, &BooleanConfig::default())?;
/// println!("Result has {} faces", result.mesh.faces.len());
/// ```
pub fn boolean_operation(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    operation: BooleanOp,
    config: &BooleanConfig,
) -> BooleanResult<BooleanOperationResult> {
    // Validate inputs
    if mesh_a.vertices.is_empty() || mesh_a.faces.is_empty() {
        return Err(BooleanError::EmptyMesh {
            details: "Mesh A is empty".to_string(),
        });
    }
    if mesh_b.vertices.is_empty() || mesh_b.faces.is_empty() {
        return Err(BooleanError::EmptyMesh {
            details: "Mesh B is empty".to_string(),
        });
    }

    // Check if bounding boxes overlap
    if !meshes_overlap(mesh_a, mesh_b) {
        return Ok(handle_non_overlapping(mesh_a, mesh_b, operation));
    }

    // Build BVHs for both meshes
    let bvh_a = if config.parallel {
        Bvh::build_parallel(mesh_a, config.bvh_leaf_size, 1000)
    } else {
        Bvh::build(mesh_a, config.bvh_leaf_size)
    };

    let bvh_b = if config.parallel {
        Bvh::build_parallel(mesh_b, config.bvh_leaf_size, 1000)
    } else {
        Bvh::build(mesh_b, config.bvh_leaf_size)
    };

    // Find intersecting triangle pairs
    let intersecting_pairs = find_intersecting_pairs(mesh_a, mesh_b, &bvh_b, config.edge_tolerance);

    if intersecting_pairs.is_empty() {
        // Meshes overlap in bounding box but don't actually intersect
        return Ok(handle_non_intersecting(mesh_a, mesh_b, &bvh_b, operation, config));
    }

    // Find coplanar pairs
    let coplanar_pairs = find_coplanar_pairs(mesh_a, mesh_b, &intersecting_pairs, config.coplanar_tolerance);
    let coplanar_a = coplanar_faces_a(&coplanar_pairs);
    let coplanar_b = coplanar_faces_b(&coplanar_pairs);

    // Insert edges for clean boundaries
    let mesh_a_split = insert_edges(mesh_a, mesh_b, &bvh_b, config.edge_tolerance);
    let mesh_b_split = insert_edges(mesh_b, mesh_a, &bvh_a, config.edge_tolerance);

    // Rebuild BVHs for split meshes
    let bvh_a_split = Bvh::build(&mesh_a_split.mesh, config.bvh_leaf_size);
    let bvh_b_split = Bvh::build(&mesh_b_split.mesh, config.bvh_leaf_size);

    // Classify faces
    let a_classifications = classify_faces_auto(
        &mesh_a_split.mesh,
        &mesh_b_split.mesh,
        &bvh_b_split,
        config.classification_tolerance,
        config.parallel,
    );
    let b_classifications = classify_faces_auto(
        &mesh_b_split.mesh,
        &mesh_a_split.mesh,
        &bvh_a_split,
        config.classification_tolerance,
        config.parallel,
    );

    // Map coplanar faces from original to split mesh
    let coplanar_a_split = map_coplanar_to_split(&coplanar_a, &mesh_a_split.face_mapping);
    let coplanar_b_split = map_coplanar_to_split(&coplanar_b, &mesh_b_split.face_mapping);

    // Build result mesh
    let mut result = IndexedMesh::new();
    let mut stats = BooleanStats {
        coplanar_pairs: coplanar_pairs.len(),
        faces_split: mesh_a_split.split_count + mesh_b_split.split_count,
        new_vertices: mesh_a_split.new_vertex_count + mesh_b_split.new_vertex_count,
        meshes_intersected: true,
        ..Default::default()
    };

    match operation {
        BooleanOp::Union => {
            // Keep faces of A that are outside B
            add_faces_with_classification(
                &mut result,
                &mesh_a_split.mesh,
                &a_classifications,
                FaceLocation::Outside,
                &coplanar_a_split,
                config.coplanar_strategy,
                true,
            );
            stats.faces_from_a = result.faces.len();

            // Keep faces of B that are outside A
            add_faces_with_classification(
                &mut result,
                &mesh_b_split.mesh,
                &b_classifications,
                FaceLocation::Outside,
                &coplanar_b_split,
                config.coplanar_strategy,
                false,
            );
            stats.faces_from_b = result.faces.len() - stats.faces_from_a;
        }

        BooleanOp::Difference => {
            // Keep faces of A that are outside B
            add_faces_with_classification(
                &mut result,
                &mesh_a_split.mesh,
                &a_classifications,
                FaceLocation::Outside,
                &coplanar_a_split,
                config.coplanar_strategy,
                true,
            );
            stats.faces_from_a = result.faces.len();

            // Keep faces of B that are inside A (inverted)
            add_faces_inverted(
                &mut result,
                &mesh_b_split.mesh,
                &b_classifications,
                FaceLocation::Inside,
                &coplanar_b_split,
                config.coplanar_strategy,
                false,
            );
            stats.faces_from_b = result.faces.len() - stats.faces_from_a;
        }

        BooleanOp::Intersection => {
            // Keep faces of A that are inside B
            add_faces_with_classification(
                &mut result,
                &mesh_a_split.mesh,
                &a_classifications,
                FaceLocation::Inside,
                &coplanar_a_split,
                config.coplanar_strategy,
                true,
            );
            stats.faces_from_a = result.faces.len();

            // Keep faces of B that are inside A
            add_faces_with_classification(
                &mut result,
                &mesh_b_split.mesh,
                &b_classifications,
                FaceLocation::Inside,
                &coplanar_b_split,
                config.coplanar_strategy,
                false,
            );
            stats.faces_from_b = result.faces.len() - stats.faces_from_a;
        }
    }

    // Apply cleanup
    apply_cleanup(&mut result, config);

    Ok(BooleanOperationResult { mesh: result, stats })
}

/// Find all pairs of triangles that potentially intersect.
fn find_intersecting_pairs(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    bvh_b: &Bvh,
    epsilon: f64,
) -> Vec<(u32, u32)> {
    let mut pairs = Vec::new();

    for (ai, face_a) in mesh_a.faces.iter().enumerate() {
        let a0 = mesh_a.vertices[face_a[0] as usize].position;
        let a1 = mesh_a.vertices[face_a[1] as usize].position;
        let a2 = mesh_a.vertices[face_a[2] as usize].position;

        let bbox_a = crate::bvh::Aabb::from_triangle(&a0, &a1, &a2);
        let candidates = bvh_b.query(&bbox_a, epsilon);

        for bi in candidates {
            let face_b = &mesh_b.faces[bi as usize];
            let b0 = mesh_b.vertices[face_b[0] as usize].position;
            let b1 = mesh_b.vertices[face_b[1] as usize].position;
            let b2 = mesh_b.vertices[face_b[2] as usize].position;

            if triangles_intersect(&a0, &a1, &a2, &b0, &b1, &b2, epsilon) {
                pairs.push((ai as u32, bi));
            }
        }
    }

    pairs
}

/// Map coplanar face indices from original mesh to split mesh.
fn map_coplanar_to_split(
    coplanar: &HashSet<u32>,
    face_mapping: &HashMap<u32, smallvec::SmallVec<[u32; 4]>>,
) -> HashSet<u32> {
    let mut result = HashSet::new();

    for &original in coplanar {
        if let Some(new_faces) = face_mapping.get(&original) {
            for &new_face in new_faces {
                result.insert(new_face);
            }
        } else {
            // If not in mapping, the face wasn't split - use original index
            result.insert(original);
        }
    }

    result
}

/// Handle case where meshes don't overlap at all.
fn handle_non_overlapping(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    operation: BooleanOp,
) -> BooleanOperationResult {
    let (mesh, faces_from_a, faces_from_b) = match operation {
        BooleanOp::Union => {
            // Combine both meshes
            let mut result = mesh_a.clone();
            let offset = result.vertices.len() as u32;
            result.vertices.extend(mesh_b.vertices.iter().cloned());
            for face in &mesh_b.faces {
                result.faces.push([face[0] + offset, face[1] + offset, face[2] + offset]);
            }
            (result, mesh_a.faces.len(), mesh_b.faces.len())
        }
        BooleanOp::Difference => {
            // Just mesh A (B doesn't affect it)
            (mesh_a.clone(), mesh_a.faces.len(), 0)
        }
        BooleanOp::Intersection => {
            // Empty result (no overlap)
            (IndexedMesh::new(), 0, 0)
        }
    };

    BooleanOperationResult {
        mesh,
        stats: BooleanStats {
            faces_from_a,
            faces_from_b,
            meshes_intersected: false,
            ..Default::default()
        },
    }
}

/// Handle case where meshes overlap in bounding box but don't intersect.
fn handle_non_intersecting(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    bvh_b: &Bvh,
    operation: BooleanOp,
    config: &BooleanConfig,
) -> BooleanOperationResult {
    use crate::classify::point_in_mesh_robust;

    // Check if A is inside B or vice versa
    let a_sample = mesh_a.vertices.first().map(|v| v.position);
    let b_sample = mesh_b.vertices.first().map(|v| v.position);

    let a_inside_b = a_sample
        .is_some_and(|p| point_in_mesh_robust(&p, mesh_b, Some(bvh_b), config.classification_tolerance));

    let bvh_a = Bvh::build(mesh_a, config.bvh_leaf_size);
    let b_inside_a = b_sample
        .is_some_and(|p| point_in_mesh_robust(&p, mesh_a, Some(&bvh_a), config.classification_tolerance));

    let (mesh, faces_from_a, faces_from_b) = match (operation, a_inside_b, b_inside_a) {
        // Union
        (BooleanOp::Union, true, _) => (mesh_b.clone(), 0, mesh_b.faces.len()),
        (BooleanOp::Union, _, true) => (mesh_a.clone(), mesh_a.faces.len(), 0),
        (BooleanOp::Union, false, false) => {
            let mut result = mesh_a.clone();
            let offset = result.vertices.len() as u32;
            result.vertices.extend(mesh_b.vertices.iter().cloned());
            for face in &mesh_b.faces {
                result.faces.push([face[0] + offset, face[1] + offset, face[2] + offset]);
            }
            (result, mesh_a.faces.len(), mesh_b.faces.len())
        }

        // Difference
        (BooleanOp::Difference, true, _) => (IndexedMesh::new(), 0, 0),
        (BooleanOp::Difference, _, _) => (mesh_a.clone(), mesh_a.faces.len(), 0),

        // Intersection
        (BooleanOp::Intersection, true, _) => (mesh_a.clone(), mesh_a.faces.len(), 0),
        (BooleanOp::Intersection, _, true) => (mesh_b.clone(), 0, mesh_b.faces.len()),
        (BooleanOp::Intersection, false, false) => (IndexedMesh::new(), 0, 0),
    };

    BooleanOperationResult {
        mesh,
        stats: BooleanStats {
            faces_from_a,
            faces_from_b,
            meshes_intersected: false,
            ..Default::default()
        },
    }
}

/// Add faces from source mesh with classification filtering.
fn add_faces_with_classification(
    result: &mut IndexedMesh,
    source: &IndexedMesh,
    classifications: &[FaceLocation],
    keep_location: FaceLocation,
    coplanar_faces: &HashSet<u32>,
    coplanar_strategy: CoplanarStrategy,
    is_first_mesh: bool,
) {
    let mut vertex_map: HashMap<u32, u32> = HashMap::new();

    for (fi, face) in source.faces.iter().enumerate() {
        let fi_u32 = fi as u32;
        let is_coplanar = coplanar_faces.contains(&fi_u32);

        let should_include = if is_coplanar {
            should_include_coplanar_face(is_first_mesh, coplanar_strategy)
        } else {
            classifications.get(fi).copied() == Some(keep_location)
        };

        if should_include {
            add_face_to_result(result, source, face, &mut vertex_map);
        }
    }
}

/// Add faces with inverted winding order (for difference operation).
fn add_faces_inverted(
    result: &mut IndexedMesh,
    source: &IndexedMesh,
    classifications: &[FaceLocation],
    keep_location: FaceLocation,
    coplanar_faces: &HashSet<u32>,
    coplanar_strategy: CoplanarStrategy,
    is_first_mesh: bool,
) {
    let mut vertex_map: HashMap<u32, u32> = HashMap::new();

    for (fi, face) in source.faces.iter().enumerate() {
        let fi_u32 = fi as u32;
        let is_coplanar = coplanar_faces.contains(&fi_u32);

        let should_include = if is_coplanar {
            should_include_coplanar_face(is_first_mesh, coplanar_strategy)
        } else {
            classifications.get(fi).copied() == Some(keep_location)
        };

        if should_include {
            // Swap indices 1 and 2 to invert winding
            let inverted_face = [face[0], face[2], face[1]];
            add_face_to_result(result, source, &inverted_face, &mut vertex_map);
        }
    }
}

/// Add a single face to the result mesh, mapping vertices.
fn add_face_to_result(
    result: &mut IndexedMesh,
    source: &IndexedMesh,
    face: &[u32; 3],
    vertex_map: &mut HashMap<u32, u32>,
) {
    let new_face: [u32; 3] = [
        *vertex_map.entry(face[0]).or_insert_with(|| {
            let idx = result.vertices.len() as u32;
            result.vertices.push(source.vertices[face[0] as usize].clone());
            idx
        }),
        *vertex_map.entry(face[1]).or_insert_with(|| {
            let idx = result.vertices.len() as u32;
            result.vertices.push(source.vertices[face[1] as usize].clone());
            idx
        }),
        *vertex_map.entry(face[2]).or_insert_with(|| {
            let idx = result.vertices.len() as u32;
            result.vertices.push(source.vertices[face[2] as usize].clone());
            idx
        }),
    ];

    result.faces.push(new_face);
}

/// Apply cleanup based on configuration.
fn apply_cleanup(mesh: &mut IndexedMesh, config: &BooleanConfig) {
    match config.cleanup {
        CleanupLevel::None => {}
        CleanupLevel::Fast => {
            // Weld vertices and remove degenerates
            weld_vertices(mesh, config.vertex_weld_tolerance);
            remove_degenerate_faces(mesh);
        }
        CleanupLevel::Full => {
            // Full cleanup including winding repair
            weld_vertices(mesh, config.vertex_weld_tolerance);
            remove_degenerate_faces(mesh);
            remove_unreferenced_vertices(mesh);
            // Note: winding repair would use mesh-repair crate
        }
    }
}

/// Weld duplicate vertices within tolerance.
fn weld_vertices(mesh: &mut IndexedMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }

    let tol_sq = tolerance * tolerance;
    let mut vertex_map: Vec<u32> = (0..mesh.vertices.len() as u32).collect();
    let mut kept_vertices: Vec<Vertex> = Vec::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let mut found = None;
        for (j, kv) in kept_vertices.iter().enumerate() {
            let dist_sq = (v.position - kv.position).norm_squared();
            if dist_sq < tol_sq {
                found = Some(j);
                break;
            }
        }

        if let Some(j) = found {
            vertex_map[i] = j as u32;
        } else {
            vertex_map[i] = kept_vertices.len() as u32;
            kept_vertices.push(v.clone());
        }
    }

    // Update faces
    for face in &mut mesh.faces {
        face[0] = vertex_map[face[0] as usize];
        face[1] = vertex_map[face[1] as usize];
        face[2] = vertex_map[face[2] as usize];
    }

    mesh.vertices = kept_vertices;
}

/// Remove faces where any two vertices are the same.
fn remove_degenerate_faces(mesh: &mut IndexedMesh) {
    mesh.faces.retain(|f| f[0] != f[1] && f[1] != f[2] && f[0] != f[2]);
}

/// Remove vertices that are not referenced by any face.
fn remove_unreferenced_vertices(mesh: &mut IndexedMesh) {
    if mesh.faces.is_empty() {
        mesh.vertices.clear();
        return;
    }

    // Find all referenced vertices
    let mut referenced = vec![false; mesh.vertices.len()];
    for face in &mesh.faces {
        referenced[face[0] as usize] = true;
        referenced[face[1] as usize] = true;
        referenced[face[2] as usize] = true;
    }

    // Build mapping from old to new indices
    let mut new_indices = vec![0u32; mesh.vertices.len()];
    let mut new_vertices = Vec::with_capacity(mesh.vertices.len());

    for (old_idx, (is_ref, vertex)) in referenced.iter().zip(mesh.vertices.iter()).enumerate() {
        if *is_ref {
            new_indices[old_idx] = new_vertices.len() as u32;
            new_vertices.push(vertex.clone());
        }
    }

    // Update faces
    for face in &mut mesh.faces {
        face[0] = new_indices[face[0] as usize];
        face[1] = new_indices[face[1] as usize];
        face[2] = new_indices[face[2] as usize];
    }

    mesh.vertices = new_vertices;
}

/// Convenience function for union operation.
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::union;
///
/// let result = union(&mesh_a, &mesh_b)?;
/// ```
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn union(mesh_a: &IndexedMesh, mesh_b: &IndexedMesh) -> BooleanResult<IndexedMesh> {
    let result = boolean_operation(mesh_a, mesh_b, BooleanOp::Union, &BooleanConfig::default())?;
    Ok(result.mesh)
}

/// Convenience function for union with custom config.
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn union_with_config(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    config: &BooleanConfig,
) -> BooleanResult<BooleanOperationResult> {
    boolean_operation(mesh_a, mesh_b, BooleanOp::Union, config)
}

/// Convenience function for difference operation.
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::difference;
///
/// let result = difference(&mesh_a, &mesh_b)?;
/// ```
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn difference(mesh_a: &IndexedMesh, mesh_b: &IndexedMesh) -> BooleanResult<IndexedMesh> {
    let result = boolean_operation(mesh_a, mesh_b, BooleanOp::Difference, &BooleanConfig::default())?;
    Ok(result.mesh)
}

/// Convenience function for difference with custom config.
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn difference_with_config(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    config: &BooleanConfig,
) -> BooleanResult<BooleanOperationResult> {
    boolean_operation(mesh_a, mesh_b, BooleanOp::Difference, config)
}

/// Convenience function for intersection operation.
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::intersection;
///
/// let result = intersection(&mesh_a, &mesh_b)?;
/// ```
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn intersection(mesh_a: &IndexedMesh, mesh_b: &IndexedMesh) -> BooleanResult<IndexedMesh> {
    let result = boolean_operation(mesh_a, mesh_b, BooleanOp::Intersection, &BooleanConfig::default())?;
    Ok(result.mesh)
}

/// Convenience function for intersection with custom config.
///
/// # Errors
///
/// Returns `BooleanError` if either mesh is empty or the operation fails.
pub fn intersection_with_config(
    mesh_a: &IndexedMesh,
    mesh_b: &IndexedMesh,
    config: &BooleanConfig,
) -> BooleanResult<BooleanOperationResult> {
    boolean_operation(mesh_a, mesh_b, BooleanOp::Intersection, config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn create_unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        // 12 triangles
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

    fn create_translated_cube(offset: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        let vertices = [
            Point3::new(offset, offset, offset),
            Point3::new(1.0 + offset, offset, offset),
            Point3::new(1.0 + offset, 1.0 + offset, offset),
            Point3::new(offset, 1.0 + offset, offset),
            Point3::new(offset, offset, 1.0 + offset),
            Point3::new(1.0 + offset, offset, 1.0 + offset),
            Point3::new(1.0 + offset, 1.0 + offset, 1.0 + offset),
            Point3::new(offset, 1.0 + offset, 1.0 + offset),
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
    fn test_union_non_overlapping() {
        let cube_a = create_unit_cube();
        let cube_b = create_translated_cube(5.0); // Far away

        let result = union(&cube_a, &cube_b).unwrap();

        // Should have all faces from both cubes
        assert_eq!(result.faces.len(), 24);
    }

    #[test]
    fn test_difference_non_overlapping() {
        let cube_a = create_unit_cube();
        let cube_b = create_translated_cube(5.0); // Far away

        let result = difference(&cube_a, &cube_b).unwrap();

        // B doesn't affect A - should have all of A's faces
        assert_eq!(result.faces.len(), 12);
    }

    #[test]
    fn test_intersection_non_overlapping() {
        let cube_a = create_unit_cube();
        let cube_b = create_translated_cube(5.0); // Far away

        let result = intersection(&cube_a, &cube_b).unwrap();

        // No overlap - empty result
        assert_eq!(result.faces.len(), 0);
    }

    #[test]
    fn test_empty_mesh_error() {
        let cube = create_unit_cube();
        let empty = IndexedMesh::new();

        let result = union(&cube, &empty);
        assert!(result.is_err());

        let result = union(&empty, &cube);
        assert!(result.is_err());
    }

    #[test]
    fn test_weld_vertices() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(1e-8, 0.0, 0.0))); // Very close
        mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.faces.push([0, 1, 2]);

        weld_vertices(&mut mesh, 1e-6);

        // First two vertices should be merged
        assert_eq!(mesh.vertices.len(), 2);
    }

    #[test]
    fn test_remove_degenerate_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
        mesh.faces.push([0, 1, 2]); // Valid
        mesh.faces.push([0, 0, 1]); // Degenerate
        mesh.faces.push([1, 2, 2]); // Degenerate

        remove_degenerate_faces(&mut mesh);

        assert_eq!(mesh.faces.len(), 1);
    }

    #[test]
    fn test_remove_unreferenced_vertices() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(5.0, 5.0, 5.0))); // Unreferenced
        mesh.faces.push([0, 1, 2]);

        remove_unreferenced_vertices(&mut mesh);

        assert_eq!(mesh.vertices.len(), 3);
    }

    #[test]
    fn test_boolean_stats() {
        let cube_a = create_unit_cube();
        let cube_b = create_translated_cube(5.0);

        let result = union_with_config(&cube_a, &cube_b, &BooleanConfig::default()).unwrap();

        assert_eq!(result.stats.faces_from_a, 12);
        assert_eq!(result.stats.faces_from_b, 12);
        assert!(!result.stats.meshes_intersected);
    }
}
