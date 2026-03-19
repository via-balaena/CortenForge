#![allow(missing_docs)]

use cf_geometry::{Ray, RayHit};
use nalgebra::{Point3, Vector3};

const EPS: f64 = 1e-12;

// ---- Ray construction ----

#[test]
fn new_accepts_unit_direction() {
    let r = Ray::new(Point3::origin(), Vector3::x());
    assert_eq!(r.origin, Point3::origin());
    assert_eq!(r.direction, Vector3::x());
}

#[test]
fn new_normalize_normalizes() {
    let r = Ray::new_normalize(Point3::origin(), Vector3::new(3.0, 0.0, 0.0));
    assert!(r.is_some());
    let r = r.map(|r| r.direction.norm());
    assert!((r.unwrap_or(0.0) - 1.0).abs() < EPS);
}

#[test]
fn new_normalize_zero_returns_none() {
    let r = Ray::new_normalize(Point3::origin(), Vector3::zeros());
    assert!(r.is_none());
}

#[test]
fn new_normalize_tiny_returns_none() {
    let r = Ray::new_normalize(Point3::origin(), Vector3::new(1e-400, 0.0, 0.0));
    assert!(r.is_none());
}

// ---- point_at ----

#[test]
fn point_at_zero_is_origin() {
    let r = Ray::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    assert_eq!(r.point_at(0.0), Point3::new(1.0, 2.0, 3.0));
}

#[test]
fn point_at_positive() {
    let r = Ray::new(Point3::origin(), Vector3::z());
    let p = r.point_at(5.0);
    assert!((p.x).abs() < EPS);
    assert!((p.y).abs() < EPS);
    assert!((p.z - 5.0).abs() < EPS);
}

#[test]
fn point_at_negative() {
    let r = Ray::new(Point3::origin(), Vector3::x());
    let p = r.point_at(-2.0);
    assert!((p.x - (-2.0)).abs() < EPS);
}

// ---- RayHit ----

#[test]
fn ray_hit_stores_fields() {
    let h = RayHit::new(3.5, Point3::new(1.0, 2.0, 3.0), Vector3::y());
    assert!((h.distance - 3.5).abs() < EPS);
    assert_eq!(h.point, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(h.normal, Vector3::y());
}
