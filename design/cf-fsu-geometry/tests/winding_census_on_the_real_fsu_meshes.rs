//! What does the flagship geometry path actually load?
//!
//! [`cf_fsu_geometry::load`] reads an STL, runs `repair_mesh`, and **discards
//! the summary** — its own comment says the statistics are "diagnostic only
//! and intentionally not surfaced by this (silent) library API". Downstream,
//! [`cf_fsu_geometry::oracle`] composes `PseudoNormalSign`, which is undefined
//! on non-manifold input by its own contract. Nothing between those two points
//! has ever measured what came in.
//!
//! This is that measurement. It is a **diagnosis**, not a threshold: it prints
//! the full topology reading for each mesh the FSU ladder loads and asserts
//! only the structural claims that hold by construction — see below.
//!
//! # Why the census must run after the weld
//!
//! An STL is triangle soup: the format stores three vertex *positions* per
//! facet with no shared indices, so no two triangles share an edge by index
//! until something welds them. A winding census on raw STL therefore has no
//! interior edges to judge and reports zero inconsistencies no matter how
//! badly wound the mesh is — the vacuity that
//! `mesh_repair::WindingCensus::has_judgeable_edges` exists to expose. This
//! test pins that ordering: raw is inconclusive, repaired is conclusive.
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

// Licence-gated diagnostic: `expect`/`panic` on a missing env var or an
// unreadable mesh is the intended failure mode — the operator set the wrong
// path and needs to be told which one.
#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use mesh_io::load_stl;
use mesh_repair::{validate_mesh, winding_census};

/// Print the two topology instruments side by side for one mesh, at both
/// stages of the load path, and return the post-repair census.
fn report(label: &str, var: &str) -> mesh_repair::WindingCensus {
    let path = std::env::var(var)
        .unwrap_or_else(|_| panic!("${var} must point at the {label} STL (see BODYPARTS3D.md)"));

    let raw = load_stl(std::path::Path::new(&path)).expect("read STL");
    let repaired = cf_fsu_geometry::load(std::path::Path::new(&path)).expect("load + repair");

    println!("\n════ {label}  (${var}) ════");
    for (stage, mesh) in [("RAW (as read)", &raw), ("REPAIRED (as used)", &repaired)] {
        let v = validate_mesh(mesh);
        let w = winding_census(mesh);
        println!(
            "  {stage}\n    {} verts {} faces | watertight {} manifold {} \
             INSIDE-OUT {}\n    {w}\n    judgeable {} | interior/boundary {}/{}",
            mesh.vertices.len(),
            mesh.faces.len(),
            v.is_watertight,
            v.is_manifold,
            v.is_inside_out,
            w.has_judgeable_edges(),
            w.interior_edges,
            w.boundary_edges,
        );
    }

    winding_census(&repaired)
}

/// The three meshes the FSU ladder loads, measured with the local winding
/// instrument that did not exist when the ladder was built.
///
/// # What is asserted, and why only this
///
/// The two structural claims are asserted because they are properties of the
/// *pipeline*, true independently of which anatomy is on the other end of the
/// env var:
///
/// 1. **Raw STL is inconclusive.** No shared indices ⇒ no interior edges ⇒
///    nothing to judge. Any winding check placed before the weld is vacuous.
/// 2. **The repaired mesh is conclusive.** `repair_mesh` welds, which is what
///    creates the shared edges the census reads. If this ever fails, the load
///    path stopped welding and every downstream topology claim is void.
///
/// The census *counts* are printed and not asserted. A threshold on them
/// would be a number invented before the measurement it is meant to describe,
/// and this arc has paid for that mistake more than once. Once the readings
/// below are known and stable, an anti-rot range belongs here — not before.
#[test]
#[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA 2.1 JP, not committed)"]
fn winding_census_of_the_meshes_the_fsu_ladder_loads() {
    for (label, var) in [
        ("L4 vertebra", "CF_L4_STL"),
        ("L5 vertebra", "CF_L5_STL"),
        ("L4-L5 disc", "CF_DISC_STL"),
    ] {
        let raw_census = {
            let path = std::env::var(var).expect("env var set");
            winding_census(&load_stl(std::path::Path::new(&path)).expect("read STL"))
        };
        assert!(
            !raw_census.has_judgeable_edges(),
            "{label}: raw STL should be triangle soup with no interior edges, \
             but the census found {} — if STL loading started welding, the \
             claim that a pre-weld winding check is vacuous needs revisiting",
            raw_census.interior_edges,
        );

        let census = report(label, var);
        assert!(
            census.has_judgeable_edges(),
            "{label}: after repair_mesh the surface must have interior edges \
             to judge — zero means the load path stopped welding, and every \
             downstream topology claim about this mesh is void",
        );
    }
}
