#![allow(missing_docs, clippy::unwrap_used, clippy::expect_used)]

use cf_geometry::{Bounded, IndexedMesh};
use nalgebra::{Point3, Vector3};

// ---------------------------------------------------------------------------
// Helper: unit cube (0,0,0) to (1,1,1), CCW winding from outside
// ---------------------------------------------------------------------------

fn unit_cube() -> IndexedMesh {
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0), // 0
        Point3::new(1.0, 0.0, 0.0), // 1
        Point3::new(1.0, 1.0, 0.0), // 2
        Point3::new(0.0, 1.0, 0.0), // 3
        Point3::new(0.0, 0.0, 1.0), // 4
        Point3::new(1.0, 0.0, 1.0), // 5
        Point3::new(1.0, 1.0, 1.0), // 6
        Point3::new(0.0, 1.0, 1.0), // 7
    ];

    #[rustfmt::skip]
    let faces = vec![
        // Bottom (z=0)
        [0, 2, 1], [0, 3, 2],
        // Top (z=1)
        [4, 5, 6], [4, 6, 7],
        // Front (y=0)
        [0, 1, 5], [0, 5, 4],
        // Back (y=1)
        [3, 7, 6], [3, 6, 2],
        // Left (x=0)
        [0, 4, 7], [0, 7, 3],
        // Right (x=1)
        [1, 2, 6], [1, 6, 5],
    ];

    IndexedMesh::from_parts(vertices, faces)
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn new_is_empty() {
    let mesh = IndexedMesh::new();
    assert!(mesh.is_empty());
    assert_eq!(mesh.vertex_count(), 0);
    assert_eq!(mesh.face_count(), 0);
}

#[test]
fn with_capacity_is_empty() {
    let mesh = IndexedMesh::with_capacity(100, 200);
    assert!(mesh.is_empty());
}

#[test]
fn from_parts() {
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let faces = vec![[0, 1, 2]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(mesh.face_count(), 1);
    assert!(!mesh.is_empty());
}

#[test]
fn from_raw() {
    let positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let indices = [0, 1, 2];
    let mesh = IndexedMesh::from_raw(&positions, &indices);
    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(mesh.face_count(), 1);
}

#[test]
fn from_raw_invalid_lengths() {
    // Not divisible by 3
    let mesh = IndexedMesh::from_raw(&[1.0, 2.0], &[0, 1, 2]);
    assert!(mesh.is_empty());

    let mesh = IndexedMesh::from_raw(&[0.0, 0.0, 0.0], &[0, 1]);
    assert!(mesh.is_empty());
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

#[test]
fn is_empty_vertices_only() {
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    // Has vertices but no faces — still empty
    assert!(mesh.is_empty());
}

#[test]
fn face_access() {
    let mesh = IndexedMesh::from_raw(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0], &[0, 1, 2]);
    assert_eq!(mesh.face(0), Some([0, 1, 2]));
    assert_eq!(mesh.face(1), None);
}

#[test]
fn triangle_access() {
    let mesh = IndexedMesh::from_raw(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0], &[0, 1, 2]);
    let tri = mesh.triangle(0).expect("valid face");
    assert_eq!(tri.v0, Point3::new(0.0, 0.0, 0.0));
    assert_eq!(tri.v1, Point3::new(1.0, 0.0, 0.0));
    assert_eq!(tri.v2, Point3::new(0.0, 1.0, 0.0));
    assert!(mesh.triangle(1).is_none());
}

#[test]
fn triangles_iterator() {
    let cube = unit_cube();
    assert_eq!(cube.triangles().count(), 12);
}

#[test]
fn positions_slice() {
    let mesh = IndexedMesh::from_raw(
        &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        &[], // no faces, but positions still accessible
    );
    let pos = mesh.positions();
    assert_eq!(pos.len(), 2);
    assert_eq!(pos[0], Point3::new(1.0, 2.0, 3.0));
    assert_eq!(pos[1], Point3::new(4.0, 5.0, 6.0));
}

// ---------------------------------------------------------------------------
// Geometric queries
// ---------------------------------------------------------------------------

#[test]
fn unit_cube_signed_volume() {
    let cube = unit_cube();
    let vol = cube.signed_volume();
    assert!(
        (vol - 1.0).abs() < 1e-10,
        "unit cube volume should be 1.0, got {vol}"
    );
}

#[test]
fn unit_cube_volume() {
    let cube = unit_cube();
    assert!((cube.volume() - 1.0).abs() < 1e-10);
}

#[test]
fn unit_cube_not_inside_out() {
    let cube = unit_cube();
    assert!(!cube.is_inside_out());
}

#[test]
fn flipped_cube_inside_out() {
    let mut cube = unit_cube();
    // Reverse winding on all faces
    for face in &mut cube.faces {
        face.swap(1, 2);
    }
    assert!(cube.is_inside_out());
}

#[test]
fn unit_cube_surface_area() {
    let cube = unit_cube();
    let area = cube.surface_area();
    assert!(
        (area - 6.0).abs() < 1e-10,
        "unit cube surface area should be 6.0, got {area}"
    );
}

#[test]
fn scaled_cube_volume() {
    let mut cube = unit_cube();
    cube.scale(2.0);
    assert!((cube.volume() - 8.0).abs() < 1e-10);
}

#[test]
fn compute_face_normals_count() {
    let cube = unit_cube();
    let normals = cube.compute_face_normals();
    assert_eq!(normals.len(), 12);

    // All normals should be unit-length
    for n in &normals {
        assert!((n.norm() - 1.0).abs() < 1e-10);
    }
}

#[test]
fn compute_face_normals_directions() {
    // Single triangle in XY plane — normal should be +Z
    let mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );
    let normals = mesh.compute_face_normals();
    assert_eq!(normals.len(), 1);
    assert!((normals[0].z - 1.0).abs() < 1e-10);
}

#[test]
fn compute_face_normals_degenerate() {
    let mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );
    let normals = mesh.compute_face_normals();
    assert!(
        (normals[0].norm()).abs() < 1e-10,
        "degenerate face → zero normal"
    );
}

#[test]
fn compute_vertex_normals() {
    // Single triangle — all three vertices get the face normal
    let mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );
    let normals = mesh.compute_vertex_normals();
    assert_eq!(normals.len(), 3);
    for n in &normals {
        assert!((n.z - 1.0).abs() < 1e-10, "should point +Z");
    }
}

#[test]
fn compute_vertex_normals_cube() {
    let cube = unit_cube();
    let normals = cube.compute_vertex_normals();
    assert_eq!(normals.len(), 8);

    // Each corner vertex of a cube should have a unit-length normal
    for n in &normals {
        assert!(
            (n.norm() - 1.0).abs() < 1e-10,
            "vertex normal should be unit-length, got {:.6}",
            n.norm()
        );
    }

    // Corner (0,0,0) — average of 3 face normals: -X, -Y, -Z
    // Should point in (-1,-1,-1) direction, normalized
    let expected = Vector3::new(-1.0, -1.0, -1.0).normalize();
    let n0 = normals[0];
    assert!(
        (n0 - expected).norm() < 1e-10,
        "corner normal should be (-1,-1,-1)/√3, got ({:.4},{:.4},{:.4})",
        n0.x,
        n0.y,
        n0.z
    );
}

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

#[test]
fn translate() {
    let mut mesh = IndexedMesh::from_parts(vec![Point3::new(0.0, 0.0, 0.0)], vec![]);
    mesh.translate(Vector3::new(1.0, 2.0, 3.0));
    assert_eq!(mesh.vertices[0], Point3::new(1.0, 2.0, 3.0));
}

#[test]
fn scale() {
    let mut mesh = IndexedMesh::from_parts(vec![Point3::new(1.0, 2.0, 3.0)], vec![]);
    mesh.scale(2.0);
    assert_eq!(mesh.vertices[0], Point3::new(2.0, 4.0, 6.0));
}

#[test]
fn scale_centered() {
    let mut cube = unit_cube();
    cube.scale_centered(2.0);
    let aabb = cube.aabb();
    // Center should still be (0.5, 0.5, 0.5), extents doubled
    assert!((aabb.center().x - 0.5).abs() < 1e-10);
    assert!((aabb.center().y - 0.5).abs() < 1e-10);
    assert!((aabb.center().z - 0.5).abs() < 1e-10);
    assert!((cube.volume() - 8.0).abs() < 1e-10);
}

// ---------------------------------------------------------------------------
// Combinators
// ---------------------------------------------------------------------------

#[test]
fn merge() {
    let mut mesh1 = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );

    let mesh2 = IndexedMesh::from_parts(
        vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );

    mesh1.merge(&mesh2);
    assert_eq!(mesh1.vertex_count(), 6);
    assert_eq!(mesh1.face_count(), 2);
    assert_eq!(mesh1.faces[1], [3, 4, 5]);
}

#[test]
fn reserve() {
    let mut mesh = IndexedMesh::new();
    mesh.reserve(100, 200);
    // Should not change counts, just capacity
    assert!(mesh.is_empty());
}

// ---------------------------------------------------------------------------
// Bounded
// ---------------------------------------------------------------------------

#[test]
fn bounded_cube() {
    let cube = unit_cube();
    let aabb = cube.aabb();
    assert!((aabb.min.x).abs() < 1e-10);
    assert!((aabb.min.y).abs() < 1e-10);
    assert!((aabb.min.z).abs() < 1e-10);
    assert!((aabb.max.x - 1.0).abs() < 1e-10);
    assert!((aabb.max.y - 1.0).abs() < 1e-10);
    assert!((aabb.max.z - 1.0).abs() < 1e-10);
}

#[test]
fn bounded_empty_mesh() {
    let mesh = IndexedMesh::new();
    assert!(mesh.aabb().is_empty());
}

#[test]
fn bounded_single_point() {
    let mesh = IndexedMesh::from_parts(vec![Point3::new(5.0, 3.0, 1.0)], vec![]);
    let aabb = mesh.aabb();
    assert_eq!(aabb.min, Point3::new(5.0, 3.0, 1.0));
    assert_eq!(aabb.max, Point3::new(5.0, 3.0, 1.0));
}

// ---------------------------------------------------------------------------
// Default, Clone, Send + Sync
// ---------------------------------------------------------------------------

#[test]
fn default_is_empty() {
    let mesh = IndexedMesh::default();
    assert!(mesh.is_empty());
}

#[test]
fn clone_is_independent() {
    let mesh = unit_cube();
    let mut cloned = mesh.clone();
    cloned.scale(2.0);
    // Original unchanged
    assert!((mesh.volume() - 1.0).abs() < 1e-10);
    assert!((cloned.volume() - 8.0).abs() < 1e-10);
}

#[test]
fn send_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<IndexedMesh>();
}
