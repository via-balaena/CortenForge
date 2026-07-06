//! MC-vs-DC mesher fidelity measurement — rung 1 of the geometry-fidelity
//! ladder.
//!
//! Points the mesh-sdf surface-fidelity harness (rung 0) at cf-design's two
//! production meshers — marching cubes ([`Solid::mesh`]) and dual contouring
//! ([`Solid::mesh_dc`]) — at matched resolution (same `tolerance` → same cell
//! size), turning "which mesher is more faithful?" into numbers.
//!
//! # Fixtures are EXACT signed-distance fields only
//!
//! `surface_deviation_to_sdf` reads `reference.eval(p)` as the point-distance
//! deviation, which is a true Euclidean distance only when the field is metric
//! (`|∇| = 1`). cf-design's `smooth_union` / CSG fields are deliberately
//! non-metric (the crate carries a `lipschitz_factor` for exactly that), so
//! measuring against them would report field values, not distances — precisely
//! in the blend region where MC and DC diverge. Every fixture here is therefore
//! an exact-SDF primitive (sphere, torus, capsule = exact), plus a *rotated*
//! cuboid for sharp convex edges — tilted off every grid plane so DC's feature
//! snapping is genuine, not a grid-alignment coincidence.
//!
//! # What the numbers say (measured; extent ~1 m, cell 0.05 m)
//!
//! | fixture                  | MC max  | DC max  | MC mean | DC mean |
//! |--------------------------|--------:|--------:|--------:|--------:|
//! | sphere (smooth)          | 0.00093 | 0.00553 | 0.00053 | 0.00033 |
//! | torus (smooth)           | 0.00266 | 0.00814 | 0.00089 | 0.00062 |
//! | capsule (smooth)         | 0.00228 | 0.00360 | 0.00091 | 0.00047 |
//! | rotated cuboid (sharp)   | 0.02479 | 0.01478 | 0.00051 | 0.00019 |
//!
//! Two consistent facts: **DC has the lower *mean* (average) deviation on every
//! fixture**, while **MC has the lower *max* (worst-case) deviation on the
//! smooth ones**. At a genuinely off-grid sharp convex edge DC beats MC on both
//! metrics, but only ~1.7× on max — it is *not* exact (an earlier grid-aligned
//! cuboid made DC look exact; that was an artifact).
//!
//! Consequence for cf-cast (smooth scanned anatomy, no sharp convex edges): the
//! two meshers differ by sub-millimetre at this cell size, and it is a genuine
//! trade-off (DC better average, MC better worst-case) rather than a win for
//! either — so there is no compelling fidelity reason to swap the tested MC
//! production path. DC's clear advantage shows up only on sharp CAD-like
//! features, which casts do not have.
//!
//! Caveat: this harness measures *point-distance* fidelity. DC also improves
//! normal/triangle-shape quality, which point-distance under-captures; that is
//! the metric that matters for casts (fit + wall thickness), but a
//! normal-deviation metric would be the tool for a normal-sensitive downstream.

#![allow(clippy::unwrap_used, clippy::print_stdout)]

use nalgebra::{Unit, UnitQuaternion, Vector3};

use crate::Solid;
use mesh_sdf::{DeviationReport, SampleOptions, surface_deviation_to_sdf};

/// Mesh `solid` with both meshers at matched resolution, measure each against
/// the analytic field, and print one row (visible under `--nocapture`).
fn measure(name: &str, solid: &Solid, tol: f64) -> (DeviationReport, DeviationReport) {
    let opts = SampleOptions::default();
    let mc = solid.mesh(tol);
    let dc = solid.mesh_dc(tol);
    let mc_dev = surface_deviation_to_sdf(&mc.geometry, solid, opts).unwrap();
    let dc_dev = surface_deviation_to_sdf(&dc.geometry, solid, opts).unwrap();
    println!(
        "{name:<24} MC[max={:.5} mean={:.5}]  DC[max={:.5} mean={:.5}]",
        mc_dev.max_abs, mc_dev.mean_abs, dc_dev.max_abs, dc_dev.mean_abs
    );
    (mc_dev, dc_dev)
}

/// A cuboid tilted off every grid plane, so DC's sharp-edge snapping is
/// measured on its own merit rather than coinciding with grid-aligned faces.
fn rotated_cuboid() -> Solid {
    let axis = Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0));
    Solid::cuboid(Vector3::new(0.8, 0.8, 0.8)).rotate(UnitQuaternion::from_axis_angle(&axis, 0.5))
}

/// On smooth surfaces the two meshers trade places: MC's on-edge zero-crossing
/// gives the tighter worst-case (max), while DC's per-cell QEF vertex gives the
/// tighter average (mean). Holds consistently across three exact-SDF shapes, so
/// neither mesher is uniformly more faithful on feature-free geometry.
#[test]
fn mc_wins_worst_case_dc_wins_average_on_smooth_surfaces() {
    for (name, solid) in [
        ("sphere", Solid::sphere(1.0)),
        ("torus", Solid::torus(1.0, 0.3)),
        ("capsule", Solid::capsule(0.4, 0.8)),
    ] {
        let (mc, dc) = measure(name, &solid, 0.05);
        assert!(
            mc.max_abs < dc.max_abs,
            "{name}: MC worst-case {} should beat DC {}",
            mc.max_abs,
            dc.max_abs
        );
        assert!(
            dc.mean_abs < mc.mean_abs,
            "{name}: DC average {} should beat MC {}",
            dc.mean_abs,
            mc.mean_abs
        );
    }
}

/// At a genuinely off-grid sharp convex edge DC beats MC on BOTH max and mean —
/// its QEF snaps toward the crease that MC can only bevel. But it is a modest
/// win (~1.7× on max), not the exactness a grid-aligned box would fake: DC's max
/// error is still a good fraction of a cell.
#[test]
fn dc_beats_mc_at_off_grid_sharp_convex_edges_but_not_exactly() {
    let (mc, dc) = measure("rotated cuboid", &rotated_cuboid(), 0.05);
    assert!(
        dc.max_abs < mc.max_abs && dc.mean_abs < mc.mean_abs,
        "DC should beat MC on both metrics at a sharp edge: MC(max={} mean={}) DC(max={} mean={})",
        mc.max_abs,
        mc.mean_abs,
        dc.max_abs,
        dc.mean_abs
    );
    assert!(
        dc.max_abs > 0.005,
        "off-grid DC is not exact — its max should be a real fraction of a cell, got {}",
        dc.max_abs
    );
}
