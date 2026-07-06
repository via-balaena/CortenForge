//! Rung 4 of the geometry-fidelity ladder — NON-CONVEX organic rigid contact.
//!
//! Facet joints (and any bone-on-bone contact) between two vertebrae need the
//! engine to resolve contact at the true CONCAVE surfaces. The default
//! `<geom type="mesh">` path cannot: sim-mjcf always convexifies a mesh geom,
//! and a vertebra's convex hull fills the foramen + inter-process gaps (rung 3
//! measured the L4 hull at ~2.6× the true volume — 238 g vs 91 g). The only
//! faithful route is the SDF concave collider (`ShapeConcave`), whose Tier-3
//! grid path does concave-vs-concave multi-contact surface tracing.
//!
//! This proves that path works on REAL anatomy AND that it genuinely traces the
//! concave surface rather than a near-hull — by DIRECT measurement. It builds
//! two colliders over the same grid resolution and pose: one from the true L4
//! mesh, one from the L4's convex hull. At an overlap where both engage, the
//! hull collider reports much deeper penetration than the concave one — because
//! the hull fills the concavities and collides where the real surface does not.
//! (Anatomically-precise L4–L5 facet articulation is a follow-on; here two real
//! vertebra surfaces in genuine concave contact prove the capability.)
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_L4_STL`.
//! Run with:
//!
//! ```text
//! CF_L4_STL=/path/to/L4.stl cargo test -p cf-design-tests \
//!   --release --test rung4_concave_contact -- --ignored --nocapture
//! ```

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::sync::Arc;

use cf_geometry::{Aabb, IndexedMesh};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfContact, SdfGrid, ShapeConcave, convex_hull};

const CELL: f64 = 2.0; // mm

/// Sample a mesh's exact metric SDF into a grid over the given bounds.
fn grid_from_mesh(
    mesh: &IndexedMesh,
    lo: Point3<f64>,
    dims: (usize, usize, usize),
    cell: f64,
) -> Arc<SdfGrid> {
    let dist = TriMeshDistance::new(mesh.clone()).unwrap();
    let sign = PseudoNormalSign::from_distance(&dist);
    let oracle = Signed {
        distance: dist,
        sign,
    };
    Arc::new(SdfGrid::from_fn(dims.0, dims.1, dims.2, cell, lo, |p| {
        oracle.evaluate(p)
    }))
}

fn max_pen(contacts: &[SdfContact]) -> f64 {
    contacts
        .iter()
        .map(|c| c.penetration)
        .fold(0.0_f64, f64::max)
}

#[test]
#[ignore = "needs a local vertebra mesh via $CF_L4_STL (CC BY-SA asset, not committed)"]
fn concave_collider_contacts_the_true_surface_not_the_hull() {
    let path = std::env::var("CF_L4_STL").expect("set $CF_L4_STL to a lumbar vertebra STL");

    // Load + repair + recenter (min-corner → origin).
    let mut mesh = load_stl(&path).expect("load L4");
    let repair = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!("[repair] welded {} verts", repair.vertices_welded);
    let pre = Aabb::from_points(mesh.vertices.iter());
    let shift = -pre.min.coords;
    for v in &mut mesh.vertices {
        *v += shift;
    }

    // Grid bounds shared by both colliders (the hull encloses the mesh, so their
    // AABBs coincide) → same resolution, isolating geometry as the only variable.
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let ext = bbox.max - bbox.min;
    let pad = 4.0 * CELL;
    let lo = bbox.min - Vector3::repeat(pad);
    let span = (bbox.max + Vector3::repeat(pad)) - lo;
    let d = |a: f64| (a / CELL).ceil() as usize + 1;
    let dims = (d(span.x), d(span.y), d(span.z));

    // Concave collider (true L4 surface) and hull collider (its convex hull),
    // both as ShapeConcave over an equal grid so the contact ALGORITHM is
    // identical and only the geometry differs.
    let concave_grid = grid_from_mesh(&mesh, lo, dims, CELL);
    let hull = convex_hull(&mesh.vertices, None).expect("hull");
    let mut hull_mesh = IndexedMesh::new();
    hull_mesh.vertices = hull.vertices;
    hull_mesh.faces = hull.faces;
    let hull_grid = grid_from_mesh(&hull_mesh, lo, dims, CELL);
    println!(
        "[grid] {}x{}x{}  concave_faces={} hull_faces={}",
        dims.0,
        dims.1,
        dims.2,
        mesh.faces.len(),
        hull_mesh.faces.len()
    );

    let (cc_a, cc_b) = (
        ShapeConcave::new(Arc::clone(&concave_grid)),
        ShapeConcave::new(Arc::clone(&concave_grid)),
    );
    let (h_a, h_b) = (
        ShapeConcave::new(Arc::clone(&hull_grid)),
        ShapeConcave::new(Arc::clone(&hull_grid)),
    );
    let pose_a = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };
    let contact = |a: &ShapeConcave, b: &ShapeConcave, dz: f64| {
        let pose_b = Pose {
            position: Point3::new(0.0, 0.0, dz),
            rotation: UnitQuaternion::identity(),
        };
        compute_shape_contact(a, &pose_a, b, &pose_b, CELL, 64)
    };

    // Sweep: heavy overlap → moderate overlap → far apart (clearance ≫ the 2 mm
    // detection margin, so the vanish check is robust).
    println!(
        "{:<8} {:>16} {:>16} {:>10}",
        "sep_mm", "concave(n,pen)", "hull(n,pen)", "localized"
    );
    let mut rows = Vec::new();
    for &frac in &[0.35_f64, 0.65, 1.5] {
        let dz = ext.z * frac;
        let cc = contact(&cc_a, &cc_b, dz);
        let hh = contact(&h_a, &h_b, dz);
        let (cn, cp) = (cc.len(), max_pen(&cc));
        let (hn, hp) = (hh.len(), max_pen(&hh));
        let localized = cc.iter().all(|c| {
            c.point.x >= -pad
                && c.point.x <= ext.x + pad
                && c.point.y >= -pad
                && c.point.y <= ext.y + pad
                && c.point.z >= -pad
                && c.point.z <= ext.z + dz + pad
        });
        println!("{dz:<8.1} {cn:>10}/{cp:<5.2} {hn:>10}/{hp:<5.2} {localized:>10}");
        rows.push((dz, cn, cp, hn, hp, localized));
    }

    let (heavy, moderate, far) = (&rows[0], &rows[1], &rows[2]);

    // (1) Concave contact fires on real vertebra geometry at genuine overlap.
    assert!(
        heavy.1 > 0,
        "overlapping vertebra surfaces must produce contacts"
    );

    // (2) It vanishes once the surfaces fully part (clearance ≫ detection margin).
    assert_eq!(
        far.1, 0,
        "no concave contact once the surfaces are well separated"
    );

    // (3) ★ MEASURED discriminator: at a moderate overlap where BOTH colliders
    //     engage, the hull penetrates far deeper than the concave one — it
    //     collides where the real surface does not (fills the concavities). This
    //     is measured against an actual hull collider, not a self-derived proxy.
    assert!(
        moderate.1 > 0 && moderate.3 > 0,
        "both colliders must engage at moderate overlap to compare them"
    );
    assert!(
        moderate.4 > 2.0 * moderate.2,
        "hull penetration ({:.2}) should be far greater than concave ({:.2}) — \
         the concave collider traces the true surface, the hull fills the gaps",
        moderate.4,
        moderate.2
    );

    // (4) Concave contacts sit on the geometry (checked on the rows that have any).
    assert!(
        heavy.5 && moderate.5,
        "concave contacts must be localized on the vertebra surfaces"
    );
}
