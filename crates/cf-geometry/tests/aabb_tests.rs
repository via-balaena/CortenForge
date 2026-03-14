#![allow(missing_docs)]

use cf_geometry::{Aabb, Axis, Bounded};
use nalgebra::{Point3, Vector3};

const EPS: f64 = 1e-12;

// ---- Construction ----

#[test]
fn new_preserves_min_max() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
    assert_eq!(a.min, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(a.max, Point3::new(4.0, 5.0, 6.0));
}

#[test]
fn from_corners_normalizes() {
    let a = Aabb::from_corners(Point3::new(5.0, 1.0, 3.0), Point3::new(2.0, 4.0, 0.0));
    assert_eq!(a.min, Point3::new(2.0, 1.0, 0.0));
    assert_eq!(a.max, Point3::new(5.0, 4.0, 3.0));
}

#[test]
fn from_center_creates_correct_box() {
    let a = Aabb::from_center(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.5, 1.0, 1.5));
    assert!((a.min.x - 0.5).abs() < EPS);
    assert!((a.min.y - 1.0).abs() < EPS);
    assert!((a.min.z - 1.5).abs() < EPS);
    assert!((a.max.x - 1.5).abs() < EPS);
    assert!((a.max.y - 3.0).abs() < EPS);
    assert!((a.max.z - 4.5).abs() < EPS);
}

#[test]
fn from_center_abs_half_extents() {
    let a = Aabb::from_center(Point3::origin(), Vector3::new(-1.0, -2.0, -3.0));
    let b = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 2.0, 3.0));
    assert_eq!(a, b);
}

#[test]
fn from_point_zero_volume() {
    let p = Point3::new(3.0, 4.0, 5.0);
    let a = Aabb::from_point(p);
    assert_eq!(a.min, p);
    assert_eq!(a.max, p);
    assert!((a.volume()).abs() < EPS);
}

#[test]
fn from_points_encloses_all() {
    let pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, -2.0, 3.0),
        Point3::new(-1.0, 5.0, 0.5),
    ];
    let a = Aabb::from_points(pts.iter());
    assert_eq!(a.min, Point3::new(-1.0, -2.0, 0.0));
    assert_eq!(a.max, Point3::new(1.0, 5.0, 3.0));
}

#[test]
fn from_points_empty_iterator() {
    let pts: [Point3<f64>; 0] = [];
    let a = Aabb::from_points(pts.iter());
    assert!(a.is_empty());
}

#[test]
fn from_triangle_encloses_vertices() {
    let v0 = Point3::new(1.0, 0.0, 0.0);
    let v1 = Point3::new(0.0, 2.0, 0.0);
    let v2 = Point3::new(0.0, 0.0, 3.0);
    let a = Aabb::from_triangle(&v0, &v1, &v2);
    assert_eq!(a.min, Point3::new(0.0, 0.0, 0.0));
    assert_eq!(a.max, Point3::new(1.0, 2.0, 3.0));
}

// ---- Empty / Valid ----

#[test]
fn empty_is_empty() {
    assert!(Aabb::empty().is_empty());
}

#[test]
fn empty_is_not_valid() {
    assert!(!Aabb::empty().is_valid());
}

#[test]
fn default_is_empty() {
    assert!(Aabb::default().is_empty());
}

#[test]
fn unit_box_is_valid() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
    assert!(a.is_valid());
    assert!(!a.is_empty());
}

#[test]
fn nan_is_not_valid() {
    let a = Aabb::new(Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    assert!(!a.is_valid());
}

// ---- Metrics ----

#[test]
fn center_of_unit_box() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 4.0, 6.0));
    let c = a.center();
    assert!((c.x - 1.0).abs() < EPS);
    assert!((c.y - 2.0).abs() < EPS);
    assert!((c.z - 3.0).abs() < EPS);
}

#[test]
fn half_extents() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 4.0, 6.0));
    let he = a.half_extents();
    assert!((he.x - 1.0).abs() < EPS);
    assert!((he.y - 2.0).abs() < EPS);
    assert!((he.z - 3.0).abs() < EPS);
}

#[test]
fn size() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 6.0, 9.0));
    let s = a.size();
    assert!((s.x - 3.0).abs() < EPS);
    assert!((s.y - 4.0).abs() < EPS);
    assert!((s.z - 6.0).abs() < EPS);
}

#[test]
fn volume() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 3.0, 4.0));
    assert!((a.volume() - 24.0).abs() < EPS);
}

#[test]
fn surface_area() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 3.0, 4.0));
    // 2*(2*3 + 3*4 + 4*2) = 2*(6+12+8) = 52
    assert!((a.surface_area() - 52.0).abs() < EPS);
}

#[test]
fn diagonal() {
    let a = Aabb::new(Point3::origin(), Point3::new(3.0, 4.0, 0.0));
    assert!((a.diagonal() - 5.0).abs() < EPS);
}

#[test]
fn max_min_extent() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 3.0, 2.0));
    assert!((a.max_extent() - 3.0).abs() < EPS);
    assert!((a.min_extent() - 1.0).abs() < EPS);
}

// ---- Axis queries ----

#[test]
fn extent_per_axis() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 3.0, 2.0));
    assert!((a.extent(Axis::X) - 1.0).abs() < EPS);
    assert!((a.extent(Axis::Y) - 3.0).abs() < EPS);
    assert!((a.extent(Axis::Z) - 2.0).abs() < EPS);
}

#[test]
fn longest_axis() {
    assert_eq!(
        Aabb::new(Point3::origin(), Point3::new(1.0, 3.0, 2.0)).longest_axis(),
        Axis::Y
    );
    assert_eq!(
        Aabb::new(Point3::origin(), Point3::new(5.0, 3.0, 2.0)).longest_axis(),
        Axis::X
    );
    assert_eq!(
        Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 9.0)).longest_axis(),
        Axis::Z
    );
}

// ---- Spatial queries ----

#[test]
fn contains_interior_point() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
    assert!(a.contains(&Point3::new(1.0, 1.0, 1.0)));
}

#[test]
fn contains_boundary_point() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
    assert!(a.contains(&Point3::new(0.0, 0.0, 0.0)));
    assert!(a.contains(&Point3::new(2.0, 2.0, 2.0)));
}

#[test]
fn does_not_contain_exterior_point() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
    assert!(!a.contains(&Point3::new(3.0, 1.0, 1.0)));
    assert!(!a.contains(&Point3::new(-0.001, 1.0, 1.0)));
}

#[test]
fn overlaps_touching() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
    let b = Aabb::new(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0));
    assert!(a.overlaps(&b));
    assert!(b.overlaps(&a));
}

#[test]
fn overlaps_partial() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
    let b = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
    assert!(a.overlaps(&b));
}

#[test]
fn no_overlap_separated() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
    let b = Aabb::new(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 1.0, 1.0));
    assert!(!a.overlaps(&b));
}

// ---- Set operations ----

#[test]
fn intersection_overlap() {
    let a = Aabb::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
    let b = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
    let i = a.intersection(&b);
    assert_eq!(i.min, Point3::new(1.0, 1.0, 1.0));
    assert_eq!(i.max, Point3::new(2.0, 2.0, 2.0));
}

#[test]
fn intersection_no_overlap_is_empty() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
    let b = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
    let i = a.intersection(&b);
    assert!(i.is_empty());
}

#[test]
fn union_encloses_both() {
    let a = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
    let b = Aabb::new(Point3::new(2.0, 3.0, 4.0), Point3::new(5.0, 6.0, 7.0));
    let u = a.union(&b);
    assert_eq!(u.min, Point3::new(0.0, 0.0, 0.0));
    assert_eq!(u.max, Point3::new(5.0, 6.0, 7.0));
}

#[test]
fn union_with_empty_is_identity() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
    let u = a.union(&Aabb::empty());
    assert_eq!(u, a);
}

// ---- Mutation ----

#[test]
fn expand_to_include_grows() {
    let mut a = Aabb::from_point(Point3::origin());
    a.expand_to_include(&Point3::new(1.0, -1.0, 2.0));
    assert_eq!(a.min, Point3::new(0.0, -1.0, 0.0));
    assert_eq!(a.max, Point3::new(1.0, 0.0, 2.0));
}

#[test]
fn expand_to_include_from_empty() {
    let mut a = Aabb::empty();
    a.expand_to_include(&Point3::new(5.0, 5.0, 5.0));
    assert_eq!(a, Aabb::from_point(Point3::new(5.0, 5.0, 5.0)));
}

#[test]
fn expanded_margin() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
    let e = a.expanded(0.5);
    assert_eq!(e.min, Point3::new(0.5, 1.5, 2.5));
    assert_eq!(e.max, Point3::new(4.5, 5.5, 6.5));
}

// ---- Corners ----

#[test]
fn corners_count_and_extremes() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
    let c = a.corners();
    assert_eq!(c.len(), 8);
    // All corners must be contained
    for corner in &c {
        assert!(a.contains(corner));
    }
    // First is min, last is max
    assert_eq!(c[0], a.min);
    assert_eq!(c[7], a.max);
}

// ---- Axis ----

#[test]
fn axis_index_roundtrip() {
    for i in 0..3 {
        let axis = Axis::from_index(i);
        assert!(axis.is_some());
        assert_eq!(axis.map(Axis::index), Some(i));
    }
    assert!(Axis::from_index(3).is_none());
}

// ---- Bounded trait ----

#[test]
fn bounded_aabb_is_identity() {
    let a = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
    assert_eq!(a.aabb(), a);
}
