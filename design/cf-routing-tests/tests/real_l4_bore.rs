//! Real-anatomy productionization of the routing spike: bore a tube through a
//! real L4 vertebra and assert the three spike measurements (watertight bore,
//! removes bone, differentiable w.r.t. the path).
//!
//! Licensed mesh, never committed — set `CF_L4_STL` to a `BodyParts3D` `FMA13075`
//! STL. Env-gated + `#[ignore]`, so it never runs in CI.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use approx::assert_abs_diff_eq;
use cf_design::Solid;
use cf_geometry::{Aabb, Sdf};
use cf_routing::{Path, bore};
use mesh_repair::validate_mesh;
use nalgebra::Vector3;

#[test]
#[ignore = "requires CF_L4_STL (licensed mesh, not committed)"]
fn bore_through_real_l4() {
    let mesh = cf_fsu_geometry::load_from_env("CF_L4_STL").expect("load L4 mesh");
    let oracle = cf_fsu_geometry::oracle(&mesh).expect("build L4 SDF oracle");
    let (center, _) = cf_fsu_geometry::body_center(&mesh, &oracle);

    // A gentle curved trajectory through the vertebral body centre.
    let cps = vec![
        center + Vector3::new(-30.0, -6.0, 0.0),
        center + Vector3::new(-8.0, 3.0, 2.0),
        center + Vector3::new(8.0, -3.0, -2.0),
        center + Vector3::new(30.0, 6.0, 0.0),
    ];
    let path = Path::new(cps).unwrap();
    let radius = 3.0_f64;

    // Bounds covering the whole vertebra for meshing.
    let bounds = Aabb::from_points(mesh.vertices.iter());
    let target = Solid::from_sdf(oracle, bounds.expanded(5.0));

    // (a) watertight bore through real bone.
    let bored = bore(target, &path, radius);
    let report = validate_mesh(&bored.mesh(0.5).geometry);
    assert!(
        report.is_watertight && report.is_manifold,
        "bore not watertight/manifold: {} boundary / {} non-manifold edges",
        report.boundary_edge_count,
        report.non_manifold_edge_count
    );

    // (b) removes real bone: an interior span midpoint is inside the L4.
    // (`from_sdf` consumed the first oracle, so build a second for probing.)
    let probe_oracle = cf_fsu_geometry::oracle(&mesh).expect("build L4 SDF oracle");
    let mid = path.sample(0.5);
    assert!(
        probe_oracle.eval(mid) < 0.0,
        "path midpoint not inside bone"
    );

    // (c) differentiable w.r.t. an interior control point — bore the field by
    // reference (a `from_sdf` Solid is not Clone) as max(phi_bone, -phi_pipe).
    let cps2 = path.control_points().to_vec();
    let probe = mid + Vector3::new(0.0, 1.0, 0.0) * (radius - 0.5); // near the bore wall
    let field_at = |dy: f64| -> f64 {
        let mut p = cps2.clone();
        p[1].y += dy;
        let pipe = Solid::pipe_spline(p, radius);
        probe_oracle.eval(probe).max(-pipe.evaluate(&probe))
    };
    let fd = |eps: f64| (field_at(eps) - field_at(-eps)) / (2.0 * eps);
    let d3 = fd(1e-3);
    let d4 = fd(1e-4);
    assert!(d4.abs() > 1e-4, "real-L4 route gradient vanishes: {d4}");
    assert_abs_diff_eq!(d3, d4, epsilon = 5e-2 * d4.abs().max(1e-6));
}
