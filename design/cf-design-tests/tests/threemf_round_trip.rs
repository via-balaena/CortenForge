//! 3MF round-trip tests for cf-design's mesh output.
//!
//! Held in `cf-design-tests` (L0-integration tier) so the
//! `mesh-io[threemf]` chain (zip + zstd + image) stays out of cf-design's
//! L0 dev-graph.

#![allow(clippy::expect_used)]

use cf_design::Solid;
use nalgebra::Vector3;

#[test]
fn threemf_round_trip() {
    let sphere = Solid::sphere(5.0);
    let mesh = sphere.mesh_adaptive_par(0.5);

    let dir = tempfile::tempdir().expect("temp dir");
    let path = dir.path().join("sphere.3mf");

    mesh_io::save_3mf(&mesh, &path).expect("save 3mf");
    let loaded = mesh_io::load_3mf(&path).expect("load 3mf");

    assert_eq!(
        loaded.vertex_count(),
        mesh.vertex_count(),
        "Vertex count mismatch after 3MF round-trip"
    );
    assert_eq!(
        loaded.face_count(),
        mesh.face_count(),
        "Face count mismatch after 3MF round-trip"
    );

    // Volume should be very close (only f64→string→f64 precision loss)
    let vol_error = (loaded.volume() - mesh.volume()).abs() / mesh.volume();
    assert!(
        vol_error < 0.001,
        "Volume error {:.4}% after 3MF round-trip (expected < 0.1%)",
        vol_error * 100.0
    );
}

#[test]
fn threemf_round_trip_composed_shape() {
    let body = Solid::cuboid(Vector3::new(3.0, 3.0, 3.0));
    let hole = Solid::cylinder(1.0, 4.0);
    let shape = body.subtract(hole);
    let mesh = shape.mesh_adaptive_par(0.4);

    let dir = tempfile::tempdir().expect("temp dir");
    let path = dir.path().join("composed.3mf");

    mesh_io::save_3mf(&mesh, &path).expect("save 3mf");
    let loaded = mesh_io::load_3mf(&path).expect("load 3mf");

    assert_eq!(loaded.vertex_count(), mesh.vertex_count());
    assert_eq!(loaded.face_count(), mesh.face_count());
}
