//! Rung 2 of the geometry-fidelity ladder — round-trip a REAL anatomical part.
//!
//! Extends rung 1 (which round-tripped exact-SDF *primitives*) to real organic
//! anatomy: takes an imported lumbar vertebra mesh, builds an exact metric
//! distance oracle from it, re-meshes that oracle with both production meshers
//! (marching cubes `Solid::mesh`, dual contouring `Solid::mesh_dc`) at several
//! resolutions, and measures how faithfully the re-meshed surface matches the
//! original — the SDF→mesh half the import spike did not exercise.
//!
//! # What it establishes (and the important scope limit)
//!
//! - The pipeline round-trips real vertebra anatomy: MC Hausdorff-to-original
//!   converges to a small fraction of the part size as the cell shrinks.
//! - **MC beats DC here because the reference is a MESH-DERIVED SDF, not because
//!   the shape is organic.** `Solid::from_sdf` wraps the oracle as a
//!   `FieldNode::UserFn` whose gradients are finite-difference, and DC's QEF
//!   sharp-feature advantage *requires* accurate gradients. A scan/mesh-derived
//!   SDF can only ever hand DC finite-difference gradients, so this is the
//!   permanent reality of meshing scanned anatomy — MC is the right mesher for
//!   that path. (Contrast rung 1: on an *analytic* field with exact gradients,
//!   smooth geometry was a trade-off.)
//!
//! `hausdorff_distance.max_abs` is a *sampled lower bound* on the true Hausdorff
//! (see mesh-sdf docs), so the fidelity check below is a strong necessary
//! indicator of feature preservation, not a proof.
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
//!   --release --test l4_roundtrip -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::sync::Arc;

use cf_design::Solid;
use cf_geometry::Aabb;
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh, validate_mesh};
use mesh_sdf::{
    PseudoNormalSign, SampleOptions, Signed, TriMeshDistance, hausdorff_distance,
    surface_deviation_to_sdf,
};

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
    let bbox = Aabb::from_points(mesh.vertices.iter());
    // Scale everything to the mesh's own size, so the test is unit-agnostic
    // (STL is unitless and `$CF_L4_STL` may point at any specimen/scale).
    let diag = bbox.diagonal();
    println!(
        "[L4] {} verts {} faces  bbox diagonal {:.2}",
        mesh.vertices.len(),
        mesh.faces.len(),
        diag
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
        "[oracle] self-consistency max_abs = {:.6}",
        self_dev.max_abs
    );
    assert!(
        self_dev.max_abs < diag * 1e-4,
        "oracle self-consistency should be ~0, got {}",
        self_dev.max_abs
    );

    // 3+4. Re-mesh the oracle with MC and DC at several (scale-relative) cell
    //      sizes and measure fidelity to the original (symmetric Hausdorff →
    //      catches dropped thin processes) and deviation from the true surface.
    let bounds = bbox.expanded(diag * 0.03);
    println!(
        "{:<8} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "cell", "MC_tris", "DC_tris", "MC_hd", "DC_hd", "MC_dev", "DC_dev"
    );
    let cells = [diag / 64.0, diag / 128.0, diag / 256.0];
    let mut mc_hds = Vec::new();
    for &cell in &cells {
        let mc = Solid::from_sdf(Arc::clone(&oracle), bounds).mesh(cell);
        let dc = Solid::from_sdf(Arc::clone(&oracle), bounds).mesh_dc(cell);
        // NOTE: each call rebuilds a parry BVH over the original mesh (the
        // `hausdorff_distance` mesh-in-mesh signature can't reuse the oracle's
        // existing BVH — the standing rung-0 borrowing-ctor follow-up).
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
            "{:<8.3} {:>10} {:>10} {:>10.4} {:>10.4} {:>10.4} {:>10.4}",
            cell,
            mc.geometry.faces.len(),
            dc.geometry.faces.len(),
            mc_hd,
            dc_hd,
            mc_dev,
            dc_dev
        );
        // MC beats DC at every resolution — but the load-bearing reason is that
        // DC is fed finite-difference gradients here (mesh-derived SDF via
        // `from_sdf`), which is exactly the scan/anatomy regime. This is a
        // claim about the FIELD being mesh-derived, not about organic shape.
        assert!(
            mc_hd < dc_hd,
            "cell {cell}: MC Hausdorff {mc_hd} should beat FD-gradient DC {dc_hd}"
        );
        mc_hds.push(mc_hd);
    }

    // Overall convergence (not strict step-wise — grid aliasing on thin
    // processes can make a finer cell align slightly worse): the finest cell
    // more than halves the coarsest cell's round-trip error, and lands within a
    // small fraction of the part size. Since the Hausdorff is a sampled lower
    // bound, this is a strong necessary indicator that the thin processes /
    // foramen / facets are preserved — not a proof.
    let (coarse, fine) = (mc_hds[0], *mc_hds.last().unwrap());
    assert!(
        fine < coarse * 0.5,
        "MC round-trip error should converge with resolution: {mc_hds:?}"
    );
    assert!(
        fine < diag * 5e-3,
        "finest-cell round-trip error should be a small fraction of the part, got {fine}"
    );
    println!();
}
