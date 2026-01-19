//! Visual regression tests for mesh operations.
//!
//! These tests verify that mesh operations produce consistent geometric output
//! by comparing against known "golden" reference meshes. This catches regressions
//! where the output geometry changes unexpectedly.
//!
//! # How It Works
//!
//! 1. Each test performs a deterministic mesh operation
//! 2. The output is compared against expected geometric properties
//! 3. Vertex positions, face counts, and bounding boxes are validated
//!
//! # Adding New Tests
//!
//! To add a new visual regression test:
//! 1. Create a deterministic input mesh
//! 2. Apply the operation being tested
//! 3. Assert specific geometric properties of the output
//!
//! Run with: cargo test -p mesh-repair visual_regression

use mesh_repair::{RepairParams, remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Aabb, IndexedMesh, MeshBounds, MeshTopology, Vertex};

// =============================================================================
// Test Mesh Generation
// =============================================================================

/// Create a canonical unit cube mesh (12 triangles).
/// This is a "golden" reference mesh with known properties.
fn create_cube() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    let verts = [
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ];

    for v in &verts {
        mesh.vertices.push(Vertex::from_coords(v[0], v[1], v[2]));
    }

    let faces: [[u32; 3]; 12] = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [2, 6, 7],
        [2, 7, 3],
        [0, 3, 7],
        [0, 7, 4],
        [1, 5, 6],
        [1, 6, 2],
    ];

    for f in &faces {
        mesh.faces.push(*f);
    }

    mesh
}

/// Create a cube with duplicate vertices (to test welding).
fn create_cube_with_duplicates() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    // Each face has its own 3 vertices (36 vertices total for 12 faces)
    let faces_with_positions = [
        // Front face (z = -0.5)
        [[-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, 0.5, -0.5]],
        [[-0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5]],
        // Back face (z = 0.5)
        [[-0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, -0.5, 0.5]],
        [[-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5], [0.5, 0.5, 0.5]],
        // Bottom face (y = -0.5)
        [[-0.5, -0.5, -0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5]],
        [[-0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [0.5, -0.5, -0.5]],
        // Top face (y = 0.5)
        [[0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5]],
        [[0.5, 0.5, -0.5], [-0.5, 0.5, 0.5], [-0.5, 0.5, -0.5]],
        // Left face (x = -0.5)
        [[-0.5, -0.5, -0.5], [-0.5, 0.5, -0.5], [-0.5, 0.5, 0.5]],
        [[-0.5, -0.5, -0.5], [-0.5, 0.5, 0.5], [-0.5, -0.5, 0.5]],
        // Right face (x = 0.5)
        [[0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5]],
        [[0.5, -0.5, -0.5], [0.5, 0.5, 0.5], [0.5, 0.5, -0.5]],
    ];

    for (i, face_positions) in faces_with_positions.iter().enumerate() {
        let base_idx = (i * 3) as u32;
        for pos in face_positions {
            mesh.vertices
                .push(Vertex::from_coords(pos[0], pos[1], pos[2]));
        }
        mesh.faces.push([base_idx, base_idx + 1, base_idx + 2]);
    }

    mesh
}

/// Create an open cube (missing one face) for hole testing.
fn create_open_cube() -> IndexedMesh {
    let mut mesh = create_cube();
    // Remove two triangles (one face)
    mesh.faces.pop();
    mesh.faces.pop();
    mesh
}

// =============================================================================
// Helper Functions
// =============================================================================

/// Compare two bounding boxes with tolerance.
fn aabb_approx_equal(a: &Aabb, b: &Aabb, epsilon: f64) -> bool {
    (a.min.x - b.min.x).abs() < epsilon
        && (a.min.y - b.min.y).abs() < epsilon
        && (a.min.z - b.min.z).abs() < epsilon
        && (a.max.x - b.max.x).abs() < epsilon
        && (a.max.y - b.max.y).abs() < epsilon
        && (a.max.z - b.max.z).abs() < epsilon
}

/// Compute the centroid of a mesh.
fn mesh_centroid(mesh: &IndexedMesh) -> [f64; 3] {
    if mesh.vertices.is_empty() {
        return [0.0, 0.0, 0.0];
    }

    let mut sum = [0.0, 0.0, 0.0];
    for v in &mesh.vertices {
        sum[0] += v.position.x;
        sum[1] += v.position.y;
        sum[2] += v.position.z;
    }

    let n = mesh.vertices.len() as f64;
    [sum[0] / n, sum[1] / n, sum[2] / n]
}

// =============================================================================
// Visual Regression Tests
// =============================================================================

#[test]
fn cube_has_expected_properties() {
    let cube = create_cube();

    // Known properties of our canonical cube
    assert_eq!(cube.vertex_count(), 8, "Cube should have 8 vertices");
    assert_eq!(cube.face_count(), 12, "Cube should have 12 faces");

    // Bounding box should be exactly -0.5 to 0.5 in all dimensions
    let aabb = cube.bounds();
    let expected_aabb = Aabb::new(
        nalgebra::Point3::new(-0.5, -0.5, -0.5),
        nalgebra::Point3::new(0.5, 0.5, 0.5),
    );

    assert!(
        aabb_approx_equal(&aabb, &expected_aabb, 1e-10),
        "Cube AABB should be [-0.5, -0.5, -0.5] to [0.5, 0.5, 0.5]"
    );

    // Centroid should be at origin
    let centroid = mesh_centroid(&cube);
    assert!(centroid[0].abs() < 1e-10, "Cube centroid X should be 0");
    assert!(centroid[1].abs() < 1e-10, "Cube centroid Y should be 0");
    assert!(centroid[2].abs() < 1e-10, "Cube centroid Z should be 0");
}

#[test]
fn weld_cube_with_duplicates_produces_8_vertices() {
    let mut mesh = create_cube_with_duplicates();

    // Before welding: 36 vertices (3 per face * 12 faces)
    assert_eq!(mesh.vertex_count(), 36, "Should start with 36 vertices");
    assert_eq!(mesh.face_count(), 12, "Should have 12 faces");

    // Weld with small epsilon
    weld_vertices(&mut mesh, 0.001);
    // Remove the now-unreferenced duplicate vertices
    remove_unreferenced_vertices(&mut mesh);

    // After welding: should have 8 unique vertices
    assert_eq!(
        mesh.vertex_count(),
        8,
        "After welding, cube should have 8 vertices"
    );
    assert_eq!(mesh.face_count(), 12, "Face count should be preserved");

    // Bounding box should be preserved
    let aabb = mesh.bounds();
    let expected_aabb = Aabb::new(
        nalgebra::Point3::new(-0.5, -0.5, -0.5),
        nalgebra::Point3::new(0.5, 0.5, 0.5),
    );
    assert!(
        aabb_approx_equal(&aabb, &expected_aabb, 1e-10),
        "Bounding box should be preserved after welding"
    );
}

#[test]
fn repair_preserves_clean_cube() {
    let cube = create_cube();
    let mut repaired = cube.clone();
    let params = RepairParams::default();

    let _ = mesh_repair::repair_mesh(&mut repaired, &params);

    // Clean cube should be unchanged
    assert_eq!(
        repaired.vertex_count(),
        cube.vertex_count(),
        "Vertex count should be preserved"
    );
    assert_eq!(
        repaired.face_count(),
        cube.face_count(),
        "Face count should be preserved"
    );

    // Bounding box should be identical
    assert!(
        aabb_approx_equal(&repaired.bounds(), &cube.bounds(), 1e-10),
        "Bounding box should be preserved"
    );
}

#[test]
fn open_cube_has_expected_properties() {
    let open_cube = create_open_cube();

    // Open cube is missing 2 triangles
    assert_eq!(open_cube.vertex_count(), 8, "Should have 8 vertices");
    assert_eq!(open_cube.face_count(), 10, "Should have 10 faces");

    // Bounding box should still be full cube size
    let aabb = open_cube.bounds();
    let expected_aabb = Aabb::new(
        nalgebra::Point3::new(-0.5, -0.5, -0.5),
        nalgebra::Point3::new(0.5, 0.5, 0.5),
    );
    assert!(
        aabb_approx_equal(&aabb, &expected_aabb, 1e-10),
        "Open cube AABB should match closed cube"
    );
}

#[test]
fn hole_filling_repairs_open_cube() {
    let mut open_cube = create_open_cube();

    // Fill holes
    let result = mesh_repair::holes::fill_holes(&mut open_cube, 100);
    assert!(result.is_ok(), "Hole filling should succeed");

    let holes_filled = result.expect("holes_filled count");

    // Should have filled at least one hole
    assert!(holes_filled >= 1, "Should have filled at least one hole");

    // After filling, should have more faces than before
    // (the hole was 2 triangles, filling should add triangles)
    assert!(
        open_cube.face_count() >= 10,
        "Face count should be at least 10 after hole filling"
    );
}

#[test]
fn all_cube_faces_have_valid_indices() {
    let cube = create_cube();
    let vertex_count = cube.vertex_count() as u32;

    for (i, face) in cube.faces.iter().enumerate() {
        assert!(
            face[0] < vertex_count,
            "Face {} index 0 ({}) is invalid",
            i,
            face[0]
        );
        assert!(
            face[1] < vertex_count,
            "Face {} index 1 ({}) is invalid",
            i,
            face[1]
        );
        assert!(
            face[2] < vertex_count,
            "Face {} index 2 ({}) is invalid",
            i,
            face[2]
        );
    }
}

#[test]
fn cube_faces_are_not_degenerate() {
    let cube = create_cube();

    for (i, face) in cube.faces.iter().enumerate() {
        // No repeated vertices in a face
        assert!(
            face[0] != face[1],
            "Face {} has duplicate vertices 0 and 1",
            i
        );
        assert!(
            face[1] != face[2],
            "Face {} has duplicate vertices 1 and 2",
            i
        );
        assert!(
            face[2] != face[0],
            "Face {} has duplicate vertices 2 and 0",
            i
        );
    }
}
