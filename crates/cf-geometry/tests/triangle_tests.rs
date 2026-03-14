#![allow(missing_docs, clippy::unwrap_used, clippy::expect_used)]

use cf_geometry::{Bounded, Triangle};
use nalgebra::Point3;

#[test]
fn new_and_fields() {
    let tri = Triangle::new(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 5.0, 6.0),
        Point3::new(7.0, 8.0, 9.0),
    );
    assert_eq!(tri.v0, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(tri.v1, Point3::new(4.0, 5.0, 6.0));
    assert_eq!(tri.v2, Point3::new(7.0, 8.0, 9.0));
}

#[test]
fn from_arrays() {
    let tri = Triangle::from_arrays([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    assert_eq!(tri.v0, Point3::origin());
    assert_eq!(tri.v1, Point3::new(1.0, 0.0, 0.0));
    assert_eq!(tri.v2, Point3::new(0.0, 1.0, 0.0));
}

#[test]
fn normal_xy_plane() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let n = tri.normal().expect("valid triangle");
    assert!((n.x).abs() < 1e-10);
    assert!((n.y).abs() < 1e-10);
    assert!((n.z - 1.0).abs() < 1e-10);
}

#[test]
fn normal_unnormalized_magnitude() {
    // Triangle with area 2, so unnormalized normal has magnitude 4
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    );
    let n = tri.normal_unnormalized();
    assert!((n.norm() - 4.0).abs() < 1e-10);
}

#[test]
fn degenerate_normal_is_none() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );
    assert!(tri.normal().is_none());
}

#[test]
fn area_right_triangle() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    );
    assert!((tri.area() - 6.0).abs() < 1e-10);
}

#[test]
fn area_unit_right_triangle() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    assert!((tri.area() - 0.5).abs() < 1e-10);
}

#[test]
fn centroid() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    );
    let c = tri.centroid();
    assert!((c.x - 1.0).abs() < 1e-10);
    assert!((c.y - 1.0).abs() < 1e-10);
    assert!(c.z.abs() < 1e-10);
}

#[test]
fn edges() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let edges = tri.edges();
    assert_eq!(edges[0], (tri.v0, tri.v1));
    assert_eq!(edges[1], (tri.v1, tri.v2));
    assert_eq!(edges[2], (tri.v2, tri.v0));
}

#[test]
fn edge_lengths_345() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    );
    let lengths = tri.edge_lengths();
    assert!((lengths[0] - 3.0).abs() < 1e-10);
    assert!((lengths[1] - 5.0).abs() < 1e-10);
    assert!((lengths[2] - 4.0).abs() < 1e-10);
}

#[test]
fn min_max_edge_length() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    );
    assert!((tri.min_edge_length() - 3.0).abs() < 1e-10);
    assert!((tri.max_edge_length() - 5.0).abs() < 1e-10);
}

#[test]
fn aspect_ratio_equilateral() {
    let sqrt3 = 3.0_f64.sqrt();
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(1.0, sqrt3, 0.0),
    );
    let ar = tri.aspect_ratio();
    assert!(ar > 1.1 && ar < 1.2, "equilateral aspect ratio: {ar}");
}

#[test]
fn aspect_ratio_degenerate() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );
    assert!(tri.aspect_ratio().is_infinite());
}

#[test]
fn is_nearly_collinear() {
    let degen = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );
    assert!(degen.is_nearly_collinear(0.01));

    let good = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
    );
    assert!(!good.is_nearly_collinear(0.01));
}

#[test]
fn is_degenerate() {
    let degen = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );
    assert!(degen.is_degenerate(1e-9));

    let good = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    assert!(!good.is_degenerate(1e-9));
}

#[test]
fn is_degenerate_enhanced() {
    // Needle triangle: very thin
    let needle = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(100.0, 0.0, 0.0),
        Point3::new(50.0, 0.001, 0.0),
    );
    // High aspect ratio should trigger
    assert!(needle.is_degenerate_enhanced(1e-12, 10.0, 1e-12));
    // But not if max_aspect_ratio is very high
    assert!(!needle.is_degenerate_enhanced(1e-12, 1e12, 1e-12));
}

#[test]
fn vertices_array() {
    let tri = Triangle::new(
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    );
    let verts = tri.vertices();
    assert_eq!(verts[0], tri.v0);
    assert_eq!(verts[1], tri.v1);
    assert_eq!(verts[2], tri.v2);
}

#[test]
fn reversed_flips_normal() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let rev = tri.reversed();
    let n1 = tri.normal().expect("valid");
    let n2 = rev.normal().expect("valid");
    assert!((n1.z + n2.z).abs() < 1e-10, "normals should be opposite");
    // Vertices preserved
    assert_eq!(rev.v0, tri.v0);
    assert_eq!(rev.v1, tri.v2);
    assert_eq!(rev.v2, tri.v1);
}

#[test]
fn bounded_impl() {
    let tri = Triangle::new(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 0.0, 5.0),
        Point3::new(-1.0, 3.0, 1.0),
    );
    let aabb = tri.aabb();
    assert!((aabb.min.x - (-1.0)).abs() < 1e-10);
    assert!((aabb.min.y - 0.0).abs() < 1e-10);
    assert!((aabb.min.z - 1.0).abs() < 1e-10);
    assert!((aabb.max.x - 4.0).abs() < 1e-10);
    assert!((aabb.max.y - 3.0).abs() < 1e-10);
    assert!((aabb.max.z - 5.0).abs() < 1e-10);
}

#[test]
fn copy_semantics() {
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let tri2 = tri; // Copy
    assert_eq!(tri, tri2);
}

#[test]
fn area_3d_triangle() {
    // Triangle not in XY plane
    let tri = Triangle::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    );
    assert!((tri.area() - 0.5).abs() < 1e-10);
    let n = tri.normal().expect("valid");
    // Normal should be in -Y direction
    assert!((n.y - (-1.0)).abs() < 1e-10);
}

#[test]
fn send_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Triangle>();
}
