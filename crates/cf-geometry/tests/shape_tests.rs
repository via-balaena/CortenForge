//! Tests for the [`Shape`] enum.

use std::sync::Arc;

use cf_geometry::{
    Bounded, Bvh, ConvexHull, HeightFieldData, IndexedMesh, SdfGrid, Shape, bvh_from_mesh,
    convex_hull,
};
use nalgebra::{Point3, Vector3};

// ===========================================================================
// Construction
// ===========================================================================

#[test]
fn sphere_construction() {
    let s = Shape::sphere(2.0);
    assert!(matches!(s, Shape::Sphere { radius } if (radius - 2.0).abs() < 1e-15));
    assert_eq!(s.radius(), Some(2.0));
}

#[test]
fn plane_construction() {
    let p = Shape::plane(Vector3::z(), 5.0);
    assert!(matches!(p, Shape::Plane { normal, distance }
        if (normal - Vector3::z()).norm() < 1e-15 && (distance - 5.0).abs() < 1e-15));
}

#[test]
fn ground_plane_construction() {
    let g = Shape::ground_plane(0.0);
    assert!(matches!(g, Shape::Plane { normal, distance }
        if (normal - Vector3::z()).norm() < 1e-15 && distance.abs() < 1e-15));
}

#[test]
fn box_construction() {
    let he = Vector3::new(1.0, 2.0, 3.0);
    let b = Shape::box_shape(he);
    assert_eq!(b.half_extents(), Some(he));
}

#[test]
fn capsule_construction() {
    let c = Shape::capsule(1.5, 0.5);
    assert_eq!(c.radius(), Some(0.5));
    assert_eq!(c.half_length(), Some(1.5));
}

#[test]
fn cylinder_construction() {
    let c = Shape::cylinder(2.0, 1.0);
    assert_eq!(c.radius(), Some(1.0));
    assert_eq!(c.half_length(), Some(2.0));
}

#[test]
fn ellipsoid_construction() {
    let e = Shape::ellipsoid(Vector3::new(1.0, 2.0, 3.0));
    assert!(matches!(e, Shape::Ellipsoid { radii } if radii == Vector3::new(1.0, 2.0, 3.0)));
}

#[test]
fn ellipsoid_xyz_construction() {
    let e = Shape::ellipsoid_xyz(1.0, 2.0, 3.0);
    assert!(matches!(e, Shape::Ellipsoid { radii } if radii == Vector3::new(1.0, 2.0, 3.0)));
}

#[test]
fn convex_mesh_construction() {
    let hull = make_tetrahedron_hull();
    let s = Shape::convex_mesh(hull);
    assert!(matches!(s, Shape::ConvexMesh { .. }));
}

#[test]
fn triangle_mesh_construction() {
    let (mesh, bvh) = make_triangle_mesh();
    let s = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));
    assert!(matches!(s, Shape::TriangleMesh { .. }));
}

#[test]
fn height_field_construction() {
    let hf = make_height_field();
    let s = Shape::height_field(Arc::new(hf));
    assert!(matches!(s, Shape::HeightField { .. }));
}

#[test]
fn sdf_construction() {
    let sdf = make_sdf();
    let s = Shape::sdf(Arc::new(sdf));
    assert!(matches!(s, Shape::Sdf { .. }));
}

// ===========================================================================
// Type predicates
// ===========================================================================

#[test]
fn is_convex() {
    assert!(Shape::sphere(1.0).is_convex());
    assert!(Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_convex());
    assert!(Shape::capsule(1.0, 0.5).is_convex());
    assert!(Shape::cylinder(1.0, 0.5).is_convex());
    assert!(Shape::ellipsoid(Vector3::new(1.0, 2.0, 3.0)).is_convex());
    assert!(Shape::convex_mesh(make_tetrahedron_hull()).is_convex());

    // Non-convex shapes
    assert!(!Shape::plane(Vector3::z(), 0.0).is_convex());
    let (mesh, bvh) = make_triangle_mesh();
    assert!(!Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh)).is_convex());
    assert!(!Shape::height_field(Arc::new(make_height_field())).is_convex());
    assert!(!Shape::sdf(Arc::new(make_sdf())).is_convex());
}

#[test]
fn is_primitive() {
    assert!(Shape::sphere(1.0).is_primitive());
    assert!(Shape::plane(Vector3::z(), 0.0).is_primitive());
    assert!(Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_primitive());
    assert!(Shape::capsule(1.0, 0.5).is_primitive());
    assert!(Shape::cylinder(1.0, 0.5).is_primitive());
    assert!(Shape::ellipsoid(Vector3::new(1.0, 2.0, 3.0)).is_primitive());

    // Non-primitives
    assert!(!Shape::convex_mesh(make_tetrahedron_hull()).is_primitive());
    let (mesh, bvh) = make_triangle_mesh();
    assert!(!Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh)).is_primitive());
}

#[test]
fn is_infinite() {
    assert!(Shape::plane(Vector3::z(), 0.0).is_infinite());
    assert!(!Shape::sphere(1.0).is_infinite());
    assert!(!Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_infinite());
}

#[test]
fn is_mesh_based() {
    assert!(Shape::convex_mesh(make_tetrahedron_hull()).is_mesh_based());
    let (mesh, bvh) = make_triangle_mesh();
    assert!(Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh)).is_mesh_based());
    assert!(Shape::sdf(Arc::new(make_sdf())).is_mesh_based());

    // Not mesh-based
    assert!(!Shape::sphere(1.0).is_mesh_based());
    assert!(!Shape::height_field(Arc::new(make_height_field())).is_mesh_based());
}

#[test]
fn individual_type_checks() {
    assert!(Shape::sphere(1.0).is_sphere());
    assert!(Shape::plane(Vector3::z(), 0.0).is_plane());
    assert!(Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_box());
    assert!(Shape::capsule(1.0, 0.5).is_capsule());
    assert!(Shape::cylinder(1.0, 0.5).is_cylinder());

    // Negative: sphere is not a box, etc.
    assert!(!Shape::sphere(1.0).is_box());
    assert!(!Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_sphere());
}

// ===========================================================================
// Accessors
// ===========================================================================

#[test]
fn radius_accessor() {
    assert_eq!(Shape::sphere(3.0).radius(), Some(3.0));
    assert_eq!(Shape::capsule(1.0, 0.5).radius(), Some(0.5));
    assert_eq!(Shape::cylinder(1.0, 0.75).radius(), Some(0.75));
    assert_eq!(Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)).radius(), None);
    assert_eq!(Shape::plane(Vector3::z(), 0.0).radius(), None);
}

#[test]
fn half_length_accessor() {
    assert_eq!(Shape::capsule(1.5, 0.5).half_length(), Some(1.5));
    assert_eq!(Shape::cylinder(2.0, 1.0).half_length(), Some(2.0));
    assert_eq!(Shape::sphere(1.0).half_length(), None);
}

#[test]
fn half_extents_accessor() {
    let he = Vector3::new(1.0, 2.0, 3.0);
    assert_eq!(Shape::box_shape(he).half_extents(), Some(he));
    assert_eq!(Shape::sphere(1.0).half_extents(), None);
}

// ===========================================================================
// Bounding radius
// ===========================================================================

#[test]
fn bounding_radius_sphere() {
    assert!((Shape::sphere(2.5).bounding_radius() - 2.5).abs() < 1e-15);
}

#[test]
fn bounding_radius_plane_is_infinite() {
    assert!(
        Shape::plane(Vector3::z(), 0.0)
            .bounding_radius()
            .is_infinite()
    );
}

#[test]
fn bounding_radius_box() {
    let he = Vector3::new(1.0, 2.0, 3.0);
    let expected = he.norm(); // diagonal
    assert!((Shape::box_shape(he).bounding_radius() - expected).abs() < 1e-12);
}

#[test]
fn bounding_radius_capsule() {
    // half_length + radius
    assert!((Shape::capsule(1.5, 0.5).bounding_radius() - 2.0).abs() < 1e-15);
}

#[test]
fn bounding_radius_cylinder() {
    // hypot(half_length, radius) = sqrt(h^2 + r^2)
    let expected = 2.0_f64.hypot(1.0);
    assert!((Shape::cylinder(2.0, 1.0).bounding_radius() - expected).abs() < 1e-12);
}

#[test]
fn bounding_radius_ellipsoid() {
    // max(rx, ry, rz)
    assert!((Shape::ellipsoid(Vector3::new(1.0, 3.0, 2.0)).bounding_radius() - 3.0).abs() < 1e-15);
}

#[test]
fn bounding_radius_convex_mesh() {
    let hull = make_tetrahedron_hull();
    let expected = hull
        .vertices
        .iter()
        .map(|v| v.coords.norm())
        .fold(0.0_f64, f64::max);
    let s = Shape::convex_mesh(hull);
    assert!((s.bounding_radius() - expected).abs() < 1e-12);
}

#[test]
fn bounding_radius_triangle_mesh() {
    let (mesh, bvh) = make_triangle_mesh();
    let expected = mesh
        .vertices
        .iter()
        .map(|v| v.coords.norm())
        .fold(0.0_f64, f64::max);
    let s = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));
    assert!((s.bounding_radius() - expected).abs() < 1e-12);
}

// ===========================================================================
// Bounded (aabb)
// ===========================================================================

#[test]
fn aabb_sphere() {
    let s = Shape::sphere(3.0);
    let aabb = s.aabb();
    assert_approx(aabb.min, Point3::new(-3.0, -3.0, -3.0));
    assert_approx(aabb.max, Point3::new(3.0, 3.0, 3.0));
}

#[test]
fn aabb_plane_is_infinite() {
    let aabb = Shape::plane(Vector3::z(), 0.0).aabb();
    assert!(aabb.min.x.is_infinite() && aabb.min.x < 0.0);
    assert!(aabb.max.x.is_infinite() && aabb.max.x > 0.0);
}

#[test]
fn aabb_box() {
    let he = Vector3::new(1.0, 2.0, 3.0);
    let aabb = Shape::box_shape(he).aabb();
    assert_approx(aabb.min, Point3::new(-1.0, -2.0, -3.0));
    assert_approx(aabb.max, Point3::new(1.0, 2.0, 3.0));
}

#[test]
fn aabb_capsule() {
    let aabb = Shape::capsule(1.5, 0.5).aabb();
    assert_approx(aabb.min, Point3::new(-0.5, -0.5, -2.0));
    assert_approx(aabb.max, Point3::new(0.5, 0.5, 2.0));
}

#[test]
fn aabb_cylinder() {
    let aabb = Shape::cylinder(2.0, 1.0).aabb();
    assert_approx(aabb.min, Point3::new(-1.0, -1.0, -2.0));
    assert_approx(aabb.max, Point3::new(1.0, 1.0, 2.0));
}

#[test]
fn aabb_ellipsoid() {
    let aabb = Shape::ellipsoid(Vector3::new(1.0, 2.0, 3.0)).aabb();
    assert_approx(aabb.min, Point3::new(-1.0, -2.0, -3.0));
    assert_approx(aabb.max, Point3::new(1.0, 2.0, 3.0));
}

#[test]
fn aabb_convex_mesh_delegates_to_hull() {
    let hull = make_tetrahedron_hull();
    let expected = hull.aabb();
    let s = Shape::convex_mesh(hull);
    let aabb = s.aabb();
    assert_approx(aabb.min, expected.min);
    assert_approx(aabb.max, expected.max);
}

#[test]
fn aabb_triangle_mesh_delegates_to_mesh() {
    let (mesh, bvh) = make_triangle_mesh();
    let expected = mesh.aabb();
    let s = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));
    let aabb = s.aabb();
    assert_approx(aabb.min, expected.min);
    assert_approx(aabb.max, expected.max);
}

#[test]
fn aabb_height_field_delegates() {
    let hf = make_height_field();
    let expected = hf.aabb();
    let s = Shape::height_field(Arc::new(hf));
    let aabb = s.aabb();
    assert_approx(aabb.min, expected.min);
    assert_approx(aabb.max, expected.max);
}

#[test]
fn aabb_sdf_delegates() {
    let sdf = make_sdf();
    let expected = sdf.aabb();
    let s = Shape::sdf(Arc::new(sdf));
    let aabb = s.aabb();
    assert_approx(aabb.min, expected.min);
    assert_approx(aabb.max, expected.max);
}

// ===========================================================================
// Clone + Send + Sync
// ===========================================================================

#[test]
fn shape_is_clone() {
    let s = Shape::sphere(1.0);
    #[allow(clippy::redundant_clone)]
    let s2 = s.clone();
    assert!((s2.bounding_radius() - 1.0).abs() < 1e-15);
}

#[test]
fn shape_is_send_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Shape>();
}

#[test]
fn arc_shapes_share_data() {
    let (mesh, bvh) = make_triangle_mesh();
    let mesh = Arc::new(mesh);
    let bvh = Arc::new(bvh);
    let s1 = Shape::triangle_mesh(Arc::clone(&mesh), Arc::clone(&bvh));
    let s2 = Shape::triangle_mesh(Arc::clone(&mesh), Arc::clone(&bvh));
    // Both shapes reference the same underlying data.
    assert_eq!(Arc::strong_count(&mesh), 3); // mesh + s1 + s2
    assert_eq!(Arc::strong_count(&bvh), 3);
    // Verify both are valid
    assert!((s1.bounding_radius() - s2.bounding_radius()).abs() < 1e-15);
}

// ===========================================================================
// Helpers
// ===========================================================================

fn assert_approx(a: Point3<f64>, b: Point3<f64>) {
    let d = (a - b).norm();
    assert!(d < 1e-12, "expected {a:?} ≈ {b:?}, delta = {d}");
}

/// Makes a small tetrahedron convex hull.
fn make_tetrahedron_hull() -> ConvexHull {
    let points = [
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(-1.0, -1.0, 1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(1.0, -1.0, -1.0),
    ];
    #[allow(clippy::expect_used)]
    convex_hull(&points, None).expect("tetrahedron should produce a hull")
}

/// Makes a simple two-triangle mesh (unit square in XY plane).
fn make_triangle_mesh() -> (IndexedMesh, Bvh) {
    let mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    );
    let bvh = bvh_from_mesh(&mesh);
    (mesh, bvh)
}

/// Makes a small 3×3 flat height field.
fn make_height_field() -> HeightFieldData {
    HeightFieldData::flat(3, 3, 1.0, 0.0)
}

/// Makes a small 3×3×3 SDF grid.
fn make_sdf() -> SdfGrid {
    let center = Point3::new(1.5, 1.5, 1.5);
    SdfGrid::from_fn(3, 3, 3, 1.0, Point3::origin(), |p| {
        (p - center).norm() - 1.0
    })
}
