#![allow(missing_docs)]

use cf_geometry::{Bounded, Sphere};
use nalgebra::Point3;

const EPS: f64 = 1e-12;

// ---- Construction ----

#[test]
fn new_clamps_negative_radius() {
    let s = Sphere::new(Point3::origin(), -5.0);
    assert!((s.radius).abs() < EPS);
}

#[test]
fn default_is_unit_sphere() {
    let s = Sphere::default();
    assert_eq!(s.center, Point3::origin());
    assert!((s.radius - 1.0).abs() < EPS);
}

// ---- Containment ----

#[test]
fn contains_center() {
    let s = Sphere::new(Point3::new(1.0, 2.0, 3.0), 5.0);
    assert!(s.contains(&Point3::new(1.0, 2.0, 3.0)));
}

#[test]
fn contains_surface_point() {
    let s = Sphere::new(Point3::origin(), 1.0);
    assert!(s.contains(&Point3::new(1.0, 0.0, 0.0)));
}

#[test]
fn does_not_contain_exterior() {
    let s = Sphere::new(Point3::origin(), 1.0);
    assert!(!s.contains(&Point3::new(1.1, 0.0, 0.0)));
}

// ---- Signed distance ----

#[test]
fn signed_distance_inside() {
    let s = Sphere::new(Point3::origin(), 2.0);
    let d = s.signed_distance(&Point3::new(1.0, 0.0, 0.0));
    assert!((d - (-1.0)).abs() < EPS);
}

#[test]
fn signed_distance_surface() {
    let s = Sphere::new(Point3::origin(), 2.0);
    let d = s.signed_distance(&Point3::new(2.0, 0.0, 0.0));
    assert!(d.abs() < EPS);
}

#[test]
fn signed_distance_outside() {
    let s = Sphere::new(Point3::origin(), 2.0);
    let d = s.signed_distance(&Point3::new(5.0, 0.0, 0.0));
    assert!((d - 3.0).abs() < EPS);
}

// ---- Metrics ----

#[test]
fn diameter() {
    let s = Sphere::new(Point3::origin(), 3.0);
    assert!((s.diameter() - 6.0).abs() < EPS);
}

#[test]
fn volume_unit_sphere() {
    let s = Sphere::new(Point3::origin(), 1.0);
    let expected = (4.0 / 3.0) * std::f64::consts::PI;
    assert!((s.volume() - expected).abs() < 1e-10);
}

#[test]
fn surface_area_unit_sphere() {
    let s = Sphere::new(Point3::origin(), 1.0);
    let expected = 4.0 * std::f64::consts::PI;
    assert!((s.surface_area() - expected).abs() < 1e-10);
}

// ---- Bounded ----

#[test]
fn bounding_box_of_unit_sphere() {
    let s = Sphere::new(Point3::new(1.0, 2.0, 3.0), 0.5);
    let bb = s.aabb();
    assert!((bb.min.x - 0.5).abs() < EPS);
    assert!((bb.min.y - 1.5).abs() < EPS);
    assert!((bb.min.z - 2.5).abs() < EPS);
    assert!((bb.max.x - 1.5).abs() < EPS);
    assert!((bb.max.y - 2.5).abs() < EPS);
    assert!((bb.max.z - 3.5).abs() < EPS);
}
