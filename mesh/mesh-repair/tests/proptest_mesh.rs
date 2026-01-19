//! Property-based tests for mesh operations.
//!
//! These tests use proptest to generate random meshes and verify invariants.
//!
//! Run with: cargo test -p mesh-repair -- proptest

use mesh_repair::{RepairParams, validate_mesh, weld_vertices};
use mesh_types::{IndexedMesh, Vertex};
use proptest::prelude::*;

// =============================================================================
// Strategies for generating random meshes
// =============================================================================

/// Generate a random vertex position in a bounded range.
fn arb_position() -> impl Strategy<Value = [f64; 3]> {
    prop::array::uniform3(-100.0..100.0f64)
}

/// Generate a random vertex with position only.
fn arb_vertex() -> impl Strategy<Value = Vertex> {
    arb_position().prop_map(|[x, y, z]| Vertex::from_coords(x, y, z))
}

/// Generate a valid mesh with the specified number of vertices and faces.
/// Ensures all face indices are valid.
fn arb_mesh(
    min_vertices: usize,
    max_vertices: usize,
    min_faces: usize,
    max_faces: usize,
) -> impl Strategy<Value = IndexedMesh> {
    (min_vertices..=max_vertices).prop_flat_map(move |num_vertices| {
        let vertices = prop::collection::vec(arb_vertex(), num_vertices);

        vertices.prop_flat_map(move |verts| {
            let n = verts.len() as u32;
            if n < 3 {
                // Need at least 3 vertices for a face
                return Just(IndexedMesh {
                    vertices: verts,
                    faces: Vec::new(),
                })
                .boxed();
            }

            let face = prop::array::uniform3(0..n);
            let faces = prop::collection::vec(face, min_faces..=max_faces);

            faces
                .prop_map(move |f| IndexedMesh {
                    vertices: verts.clone(),
                    faces: f,
                })
                .boxed()
        })
    })
}

/// Generate a simple triangulated cube mesh.
fn cube_mesh() -> IndexedMesh {
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

    let faces = [
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
        mesh.faces.push([f[0] as u32, f[1] as u32, f[2] as u32]);
    }

    mesh
}

// =============================================================================
// Property Tests: Validation
// =============================================================================

proptest! {
    /// Validation should never panic on any mesh.
    #[test]
    fn validation_never_panics(mesh in arb_mesh(3, 50, 0, 100)) {
        let _ = validate_mesh(&mesh);
    }

    /// Validation is idempotent - running it twice produces same result.
    #[test]
    fn validation_is_idempotent(mesh in arb_mesh(3, 30, 1, 50)) {
        let report1 = validate_mesh(&mesh);
        let report2 = validate_mesh(&mesh);

        prop_assert_eq!(report1.vertex_count, report2.vertex_count);
        prop_assert_eq!(report1.face_count, report2.face_count);
        prop_assert_eq!(report1.is_manifold, report2.is_manifold);
        prop_assert_eq!(report1.is_watertight, report2.is_watertight);
    }
}

// =============================================================================
// Property Tests: Vertex Welding
// =============================================================================

proptest! {
    /// Welding with tiny tolerance should not increase vertex count.
    #[test]
    fn weld_tiny_tolerance_doesnt_increase_vertices(mesh in arb_mesh(3, 30, 1, 50)) {
        let original_vertex_count = mesh.vertices.len();
        let mut welded = mesh.clone();

        // Use tiny epsilon - should only merge exact duplicates
        weld_vertices(&mut welded, 1e-15);

        // Vertex count should not increase (might decrease if there are exact duplicates)
        prop_assert!(welded.vertices.len() <= original_vertex_count);
    }

    /// Welding should never increase vertex count.
    #[test]
    fn weld_never_increases_vertices(mesh in arb_mesh(3, 30, 1, 50)) {
        let original_vertex_count = mesh.vertices.len();
        let mut welded = mesh.clone();

        weld_vertices(&mut welded, 0.001);

        prop_assert!(welded.vertices.len() <= original_vertex_count);
    }

    /// Welding should preserve face count (faces still reference valid vertices).
    #[test]
    fn weld_preserves_face_count(mesh in arb_mesh(3, 30, 1, 50)) {
        let original_face_count = mesh.faces.len();
        let mut welded = mesh.clone();

        weld_vertices(&mut welded, 0.001);

        prop_assert_eq!(welded.faces.len(), original_face_count);
    }

    /// All face indices should be valid after welding.
    #[test]
    fn weld_produces_valid_indices(mesh in arb_mesh(3, 30, 1, 50)) {
        let mut welded = mesh.clone();
        weld_vertices(&mut welded, 0.01);

        let vertex_count = welded.vertices.len() as u32;
        for face in &welded.faces {
            prop_assert!(face[0] < vertex_count, "Face index {} >= vertex count {}", face[0], vertex_count);
            prop_assert!(face[1] < vertex_count, "Face index {} >= vertex count {}", face[1], vertex_count);
            prop_assert!(face[2] < vertex_count, "Face index {} >= vertex count {}", face[2], vertex_count);
        }
    }
}

// =============================================================================
// Property Tests: Full Repair
// =============================================================================

proptest! {
    /// Full repair should never panic.
    #[test]
    fn repair_never_panics(mesh in arb_mesh(3, 30, 1, 50)) {
        let mut repaired = mesh.clone();
        let params = RepairParams::default();
        let _ = mesh_repair::repair_mesh(&mut repaired, &params);
    }

    /// Repair should not increase face count unexpectedly.
    #[test]
    fn repair_doesnt_explode_faces(mesh in arb_mesh(3, 30, 1, 50)) {
        let original_face_count = mesh.faces.len();
        let mut repaired = mesh.clone();
        let params = RepairParams::default();

        let _ = mesh_repair::repair_mesh(&mut repaired, &params);

        // Repair might add faces for hole filling, but shouldn't go crazy
        // Allow up to 2x original for reasonable hole filling
        prop_assert!(repaired.faces.len() <= original_face_count * 2 + 100);
    }
}

// =============================================================================
// Property Tests: Cube mesh invariants
// =============================================================================

#[test]
fn cube_is_valid() {
    let cube = cube_mesh();
    let report = validate_mesh(&cube);

    assert_eq!(report.vertex_count, 8);
    assert_eq!(report.face_count, 12);
}

#[test]
fn cube_weld_is_noop() {
    let cube = cube_mesh();
    let mut welded = cube.clone();

    weld_vertices(&mut welded, 0.001);

    // Cube has no duplicate vertices, so welding should be a no-op
    assert_eq!(welded.vertices.len(), cube.vertices.len());
    assert_eq!(welded.faces.len(), cube.faces.len());
}

#[test]
fn cube_repair_is_stable() {
    let cube = cube_mesh();
    let mut repaired = cube.clone();
    let params = RepairParams::default();

    let _ = mesh_repair::repair_mesh(&mut repaired, &params);

    // Cube is already clean, so repair should be mostly a no-op
    assert_eq!(repaired.vertices.len(), cube.vertices.len());
    assert_eq!(repaired.faces.len(), cube.faces.len());
}
