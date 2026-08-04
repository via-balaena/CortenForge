//! What does the flagship geometry path actually load?
//!
//! `cf_fsu_geometry::load` welds a raw STL and hands the result to
//! `cf_fsu_geometry::oracle`, which composes `PseudoNormalSign` — a sign
//! computed from face winding. Until the `SurfaceReport` this test prints,
//! nothing between those two points measured the surface in between.
//!
//! This is that measurement, on the three meshes the FSU ladder loads. It is a
//! **diagnosis, not a threshold**: it prints the full reading and asserts only
//! that the report describes the mesh it was returned with.
//!
//! # Why it asserts so little
//!
//! Almost nothing here is anatomy-dependent. That a raw STL has no interior
//! edges is a property of the *format* — `load_stl` emits three fresh vertices
//! and a consecutive index triple per facet, so no STL whatsoever can falsify
//! it — and it is already pinned licence-free by
//! `the_surface_report_is_measured_after_the_weld_not_before` in this crate's
//! lib tests. Re-asserting it here would spend a licensed mesh fetch on a
//! mesh-independent fact and read as anatomical coverage it is not.
//!
//! What this test buys is the **reading itself**, on real scan geometry, from
//! the production entry point rather than from a re-implementation of it.
//!
//! # Running it
//!
//! Licence-gated — the `BodyParts3D` meshes are CC BY-SA 2.1 JP and are never
//! committed. Fetch them per `BODYPARTS3D.md` at this crate's root, then:
//!
//! ```sh
//! cargo test -p cf-fsu-geometry --test winding_census_on_the_real_fsu_meshes \
//!     -- --ignored --nocapture
//! ```

// Licence-gated diagnostic: `expect` on a missing env var or an unreadable
// mesh is the intended failure mode — the operator set the wrong path and
// needs to be told which one.
#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use cf_fsu_geometry::load_from_env_with_report;
use mesh_io::load_stl;
use mesh_repair::winding_census;

/// The three meshes the FSU ladder loads, read through the production entry
/// point, with the local winding instrument that did not exist when the ladder
/// was built.
///
/// The one assertion is that every field of the report describes the surface
/// actually returned. That is what would catch the wiring regression this arc
/// is about — a report built from the wrong mesh, or built before the weld —
/// on the real path rather than on a synthetic box.
#[test]
#[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA 2.1 JP, not committed)"]
fn winding_census_of_the_meshes_the_fsu_ladder_loads() {
    for (label, var) in [
        ("L4 vertebra", "CF_L4_STL"),
        ("L5 vertebra", "CF_L5_STL"),
        ("L4-L5 disc", "CF_DISC_STL"),
    ] {
        // `load_from_env_with_report` names the missing variable itself, so a
        // half-configured environment reports which one is absent.
        let (mesh, report) = load_from_env_with_report(var)
            .unwrap_or_else(|e| panic!("{label}: {e:#} (see BODYPARTS3D.md)"));

        // Context only: what the census would have said before the weld. Not
        // asserted — see the module docs.
        let path = std::env::var(var).expect("the load above already read it");
        let raw = load_stl(std::path::Path::new(&path)).expect("re-read for the raw reading");
        let raw_census = winding_census(&raw);

        println!("\n════ {label}  (${var}) ════");
        println!(
            "  RAW (as read)      {} verts {} faces | {raw_census}",
            raw.vertices.len(),
            raw.faces.len(),
        );
        println!("  REPAIRED (as used) {report}");

        assert_eq!(
            (report.topology.vertex_count, report.topology.face_count),
            (mesh.vertices.len(), mesh.faces.len()),
            "{label}: the report must describe the surface it came back with",
        );
        assert_eq!(
            report.repair.final_faces,
            mesh.faces.len(),
            "{label}: and so must the repair summary",
        );
    }
}
