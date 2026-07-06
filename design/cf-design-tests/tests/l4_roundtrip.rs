//! Rung 2 of the geometry-fidelity ladder — round-trip a REAL anatomical part.
//!
//! Extends rung 1 (which round-tripped exact-SDF *primitives*) to real organic
//! anatomy: takes an imported lumbar vertebra mesh, builds an exact metric
//! distance oracle from it, re-meshes that oracle with both production meshers
//! (marching cubes `Solid::mesh`, dual contouring `Solid::mesh_dc`) at several
//! resolutions, and measures how faithfully the re-meshed surface matches the
//! original — the SDF→mesh half the import spike did not exercise.
//!
//! # Asset & licensing
//!
//! The reference mesh (`BodyParts3D` L4 = `FMA13075`) is CC BY-SA — copyleft —
//! so it is **not** committed to this Apache-licensed tree. The test is marked
//! `#[ignore]` and gated on `$CF_L4_STL` (mirrors the `CF_CAST_ITER1_DIR`
//! precedent): it skips by default / in CI and runs against a locally-provided
//! mesh. Run with:
//!
//! ```text
//! CF_L4_STL=/path/to/L4.stl cargo test -p cf-design-tests \
//!   --test l4_roundtrip -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::sync::Arc;

use cf_design::Solid;
use cf_geometry::{Aabb, IndexedMesh};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh, validate_mesh};
use mesh_sdf::{
    PseudoNormalSign, SampleOptions, Signed, TriMeshDistance, hausdorff_distance,
    surface_deviation_to_sdf,
};
use nalgebra::{Point3, Vector3};

fn aabb_of(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut lo = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for v in &mesh.vertices {
        lo = Point3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
        hi = Point3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
    }
    (lo, hi)
}

#[test]
#[ignore = "needs a local vertebra mesh via $CF_L4_STL (CC BY-SA asset, not committed)"]
fn l4_roundtrips_feature_faithfully() {
    let path = std::env::var("CF_L4_STL").expect("set $CF_L4_STL to a lumbar vertebra STL");

    // 1. Import + repair into a clean watertight reference mesh.
    let mut mesh = load_stl(&path).expect("load vertebra STL");
    let repair = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!("[repair] welded {} verts", repair.vertices_welded);
    let report = validate_mesh(&mesh);
    assert!(
        report.is_watertight && report.is_manifold,
        "reference must be watertight+manifold after repair (got {report:?})"
    );
    let (lo, hi) = aabb_of(&mesh);
    let ext = hi - lo;
    println!(
        "\n[L4] {} verts {} faces  extent ({:.1},{:.1},{:.1}) mm",
        mesh.vertices.len(),
        mesh.faces.len(),
        ext.x,
        ext.y,
        ext.z
    );

    // 2. Exact, grid-free metric oracle from the watertight mesh
    //    (TriMeshDistance = exact BVH distance; PseudoNormalSign = exact sign on
    //    a watertight mesh). Arc-shared so it feeds both `from_sdf` and the
    //    measurement without a deep clone.
    let distance = TriMeshDistance::new(mesh.clone()).expect("bvh");
    let sign = PseudoNormalSign::from_distance(&distance);
    let oracle = Arc::new(Signed { distance, sign });
    let opts = SampleOptions::default();

    // Sanity: the reference mesh must lie on its own oracle's zero-set, else
    // the oracle is not a valid metric reference (bad sign, etc.).
    let self_dev = surface_deviation_to_sdf(&mesh, &oracle, opts).expect("self dev");
    println!(
        "[oracle] self-consistency max_abs = {:.6} mm",
        self_dev.max_abs
    );
    assert!(
        self_dev.max_abs < 1e-3,
        "oracle self-consistency should be ~0, got {}",
        self_dev.max_abs
    );

    // 3+4. Re-mesh the oracle with MC and DC at several resolutions and measure
    //      fidelity to the original (symmetric Hausdorff → catches dropped thin
    //      processes) and deviation from the true surface (vs the oracle).
    let margin = 4.0;
    let bounds = Aabb::new(lo - Vector3::repeat(margin), hi + Vector3::repeat(margin));
    println!(
        "{:<6} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "tol_mm", "MC_tris", "DC_tris", "MC_hd", "DC_hd", "MC_dev", "DC_dev"
    );
    let tols = [2.0_f64, 1.0, 0.5];
    let mut mc_hds = Vec::new();
    for &tol in &tols {
        let mc = Solid::from_sdf(Arc::clone(&oracle), bounds).mesh(tol);
        let dc = Solid::from_sdf(Arc::clone(&oracle), bounds).mesh_dc(tol);
        let mc_hd = hausdorff_distance(&mesh, &mc.geometry, opts)
            .expect("mc hd")
            .max_abs;
        let dc_hd = hausdorff_distance(&mesh, &dc.geometry, opts)
            .expect("dc hd")
            .max_abs;
        let mc_dev = surface_deviation_to_sdf(&mc.geometry, &oracle, opts)
            .expect("mc dev")
            .max_abs;
        let dc_dev = surface_deviation_to_sdf(&dc.geometry, &oracle, opts)
            .expect("dc dev")
            .max_abs;
        println!(
            "{:<6.2} {:>10} {:>10} {:>10.4} {:>10.4} {:>10.4} {:>10.4}",
            tol,
            mc.geometry.faces.len(),
            dc.geometry.faces.len(),
            mc_hd,
            dc_hd,
            mc_dev,
            dc_dev
        );
        // The pipeline round-trips real anatomy: on this smooth-dominated organic
        // part MC is the tighter mesher at every resolution (extends rung 1;
        // `from_sdf` also hands DC finite-difference gradients, blunting its QEF).
        assert!(
            mc_hd < dc_hd,
            "tol {tol}: MC Hausdorff {mc_hd} should beat DC {dc_hd} on organic anatomy"
        );
        mc_hds.push(mc_hd);
    }

    // Feature-faithful: MC Hausdorff-to-original converges monotonically and, at
    // 0.5 mm cell, matches the original vertebra to well under a millimetre
    // (~0.5% of the part's 43 mm height) — the thin processes / foramen / facets
    // survive the round-trip.
    assert!(
        mc_hds[0] > mc_hds[1] && mc_hds[1] > mc_hds[2],
        "MC round-trip error must converge with resolution: {mc_hds:?}"
    );
    assert!(
        *mc_hds.last().unwrap() < 0.3,
        "at 0.5 mm cell the L4 round-trip should be sub-0.3 mm, got {}",
        mc_hds.last().unwrap()
    );
    println!();
}
