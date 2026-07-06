//! MC-vs-DC mesher fidelity measurement — rung 1 of the geometry-fidelity
//! ladder.
//!
//! Points the mesh-sdf surface-fidelity harness (rung 0) at cf-design's two
//! production meshers — marching cubes ([`Solid::mesh`]) and dual contouring
//! ([`Solid::mesh_dc`]) — across fixtures spanning smooth → sharp → organic
//! geometry, at matched resolution (same `tolerance` → same cell size).
//!
//! # What the numbers say (measured, radius/half-extent ~1 m, cell 0.05 m)
//!
//! | fixture                     | MC max dev | DC max dev |
//! |-----------------------------|-----------:|-----------:|
//! | sphere (smooth)             |   0.0009 m |   0.0055 m |
//! | cuboid (sharp convex)       |   0.0333 m |   0.0000 m |
//! | sphere−sphere (concave)     |   0.0313 m |   0.0353 m |
//! | blob + thin process         |   0.0031 m |   0.0162 m |
//!
//! **DC's win is confined to sharp *convex* edges** — there its QEF vertex
//! snaps exactly onto the crease while MC bevels it by ~⅔ cell. On smooth and
//! organic (feature-free) surfaces the opposite holds: DC's per-cell QEF
//! vertex wanders off the true surface more than MC's on-edge zero-crossing,
//! so **MC has the lower point-distance error there**. On concave creases the
//! two are comparable. Consequence for cf-cast (which meshes smooth scanned
//! anatomy, no sharp convex edges): switching MC→DC would not improve — and
//! could slightly worsen — point-distance fidelity. The tests below lock each
//! of these relationships.
//!
//! Caveat: this harness measures *point-distance* fidelity. DC also improves
//! normal/triangle-shape quality, which point-distance under-captures; for
//! silicone casts (fit + wall thickness = point-distance) that is the metric
//! that matters, but a normal-deviation metric would be the tool for a
//! normal-sensitive downstream.

#![allow(clippy::unwrap_used, clippy::print_stdout)]

use nalgebra::Vector3;

use crate::Solid;
use mesh_sdf::{DeviationReport, SampleOptions, hausdorff_distance, surface_deviation_to_sdf};

/// Mesh `solid` with both meshers at matched resolution and measure each
/// against the analytic field; returns `(mc_dev, dc_dev, mc_vs_dc_hausdorff)`
/// and prints one table row (visible under `--nocapture`).
fn measure(
    name: &str,
    solid: &Solid,
    tol: f64,
) -> (DeviationReport, DeviationReport, DeviationReport) {
    let opts = SampleOptions::default();
    let mc = solid.mesh(tol);
    let dc = solid.mesh_dc(tol);
    let mc_dev = surface_deviation_to_sdf(&mc.geometry, solid, opts).unwrap();
    let dc_dev = surface_deviation_to_sdf(&dc.geometry, solid, opts).unwrap();
    let hd = hausdorff_distance(&mc.geometry, &dc.geometry, opts).unwrap();
    println!(
        "{name:<30} MC[tris={} max={:.5} mean={:.5}]  DC[tris={} max={:.5} mean={:.5}]  MC-DC_hd={:.5}",
        mc.geometry.faces.len(),
        mc_dev.max_abs,
        mc_dev.mean_abs,
        dc.geometry.faces.len(),
        dc_dev.max_abs,
        dc_dev.mean_abs,
        hd.max_abs,
    );
    (mc_dev, dc_dev, hd)
}

/// On sharp convex edges DC's QEF snaps vertices onto the crease → near-exact,
/// while MC cannot represent the crease and bevels it, missing the true corner
/// by ~⅔ of a cell. This is the one place DC decisively wins.
#[test]
fn dc_is_near_exact_on_sharp_convex_edges_where_mc_bevels() {
    let (mc, dc, _) = measure(
        "cuboid (sharp convex)",
        &Solid::cuboid(Vector3::new(1.0, 1.0, 1.0)),
        0.05,
    );
    assert!(
        dc.max_abs < 1e-4,
        "DC should reproduce sharp convex edges near-exactly, got {}",
        dc.max_abs
    );
    assert!(
        mc.max_abs > 0.02,
        "MC should bevel sharp edges by ~cell size, got {}",
        mc.max_abs
    );
}

/// On feature-free (smooth / organic) surfaces DC's per-cell QEF vertex wanders
/// off the true surface more than MC's on-edge zero-crossing, so MC is the
/// better *point-distance* mesher there — the mission-relevant regime, since
/// scanned anatomy has no sharp convex edges.
#[test]
fn mc_holds_tighter_than_dc_on_smooth_and_organic_surfaces() {
    let smooth = measure("sphere (smooth)", &Solid::sphere(1.0), 0.05);
    assert!(
        smooth.0.max_abs < smooth.1.max_abs,
        "sphere: MC max {} should beat DC max {}",
        smooth.0.max_abs,
        smooth.1.max_abs
    );

    let organic = measure(
        "blob + thin process",
        &Solid::sphere(1.0).smooth_union(
            Solid::capsule(0.15, 0.8).translate(Vector3::new(1.0, 0.0, 0.0)),
            0.1,
        ),
        0.03,
    );
    assert!(
        organic.0.max_abs < organic.1.max_abs,
        "organic: MC max {} should beat DC max {}",
        organic.0.max_abs,
        organic.1.max_abs
    );
}

/// On a concave crease neither mesher dominates on point distance — both sit
/// within about a cell of the true surface. So DC's advantage is specific to
/// sharp *convex* features, not concavities (which scanned anatomy does have).
#[test]
fn mc_and_dc_are_comparable_on_concave_creases() {
    let concave =
        Solid::sphere(1.0).subtract(Solid::sphere(0.7).translate(Vector3::new(0.6, 0.0, 0.0)));
    let (mc, dc, _) = measure("sphere-minus-sphere (concave)", &concave, 0.05);
    assert!(
        (mc.max_abs - dc.max_abs).abs() < 0.02,
        "concave: MC {} and DC {} should be comparable (within ~cell)",
        mc.max_abs,
        dc.max_abs
    );
    assert!(
        mc.max_abs < 0.06 && dc.max_abs < 0.06,
        "both bounded by ~cell size"
    );
}
