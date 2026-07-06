//! Rung 4 of the geometry-fidelity ladder — NON-CONVEX organic rigid contact.
//!
//! Facet joints (and any bone-on-bone contact) between two vertebrae need the
//! engine to resolve contact at the true CONCAVE surfaces. The default
//! `<geom type="mesh">` path cannot: sim-mjcf always convexifies a mesh geom,
//! and a vertebra's convex hull fills the foramen + inter-process gaps (rung 3
//! measured the L4 hull at ~2.6× the true volume — 238 g vs 91 g), so two hulls
//! collide at their bounding envelopes, never at the real surfaces. The only
//! faithful route is the SDF concave collider (`ShapeConcave`), whose Tier-3
//! grid path does concave-vs-concave multi-contact surface tracing.
//!
//! This proves that path works on REAL anatomy: build an `SdfGrid` from the L4
//! mesh-sdf oracle, make two `ShapeConcave` colliders, and sweep their
//! separation. The contact must track the true geometry — falling off
//! monotonically and vanishing once the surfaces part — and its penetration
//! must stay far below the bounding-volume overlap (i.e. it traces the concave
//! surface, not the hull). (Anatomically-precise L4–L5 facet articulation is a
//! follow-on; here two real vertebra surfaces in genuine concave contact are
//! enough to prove the capability.)
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

use cf_geometry::Aabb;
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfGrid, ShapeConcave};

const CELL: f64 = 2.0; // mm — resolves the L4's gross features; finer would sharpen thin gaps.

#[test]
#[ignore = "needs a local vertebra mesh via $CF_L4_STL (CC BY-SA asset, not committed)"]
fn two_real_vertebra_surfaces_contact_at_the_concave_surface() {
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

    // Metric oracle → SdfGrid → two shared ShapeConcave colliders.
    let dist = TriMeshDistance::new(mesh.clone()).unwrap();
    let sign = PseudoNormalSign::from_distance(&dist);
    let oracle = Signed {
        distance: dist,
        sign,
    };
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let ext = bbox.max - bbox.min;
    let pad = 4.0 * CELL;
    let lo = bbox.min - Vector3::repeat(pad);
    let span = (bbox.max + Vector3::repeat(pad)) - lo;
    let dims = |a: f64| (a / CELL).ceil() as usize + 1;
    let grid = Arc::new(SdfGrid::from_fn(
        dims(span.x),
        dims(span.y),
        dims(span.z),
        CELL,
        lo,
        |p| oracle.evaluate(p),
    ));
    let a = ShapeConcave::new(Arc::clone(&grid));
    let b = ShapeConcave::new(Arc::clone(&grid));
    let pose_a = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };

    // Sweep vertical separation: B is the same vertebra shifted up by a fraction
    // of its own height, so overlap shrinks to nothing as the fraction → 1.
    println!("[rung4] ext.z={:.1}mm  cell={CELL}mm", ext.z);
    let fracs = [0.25_f64, 0.45, 0.65, 0.85, 1.05];
    let mut rows = Vec::new();
    for &frac in &fracs {
        let dz = ext.z * frac;
        let pose_b = Pose {
            position: Point3::new(0.0, 0.0, dz),
            rotation: UnitQuaternion::identity(),
        };
        let contacts = compute_shape_contact(&a, &pose_a, &b, &pose_b, CELL, 64);
        let max_pen = contacts
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f64, f64::max);
        // Contacts must sit on the geometry (within the two bodies' AABB union).
        let localized = contacts.iter().all(|c| {
            c.point.x >= -pad
                && c.point.x <= ext.x + pad
                && c.point.y >= -pad
                && c.point.y <= ext.y + pad
                && c.point.z >= -pad
                && c.point.z <= ext.z + dz + pad
        });
        println!(
            "  sep={:.1}mm ({:.0}%)  n={}  max_pen={:.2}mm  localized={}",
            dz,
            frac * 100.0,
            contacts.len(),
            max_pen,
            localized
        );
        rows.push((dz, contacts.len(), max_pen, localized));
    }

    let n: Vec<usize> = rows.iter().map(|r| r.1).collect();

    // (1) Concave contact fires on real vertebra geometry at genuine overlap.
    assert!(
        n[0] > 0,
        "overlapping vertebra surfaces must produce contacts"
    );

    // (2) It TRACKS the real geometry: contact falls off monotonically with
    //     separation and vanishes once the surfaces part (a convex hull, which
    //     fills the concavities, could not fall to zero here).
    for i in 1..n.len() {
        assert!(
            n[i] <= n[i - 1],
            "contact must not grow as bodies separate: {n:?}"
        );
    }
    assert!(
        n[0] > *n.last().unwrap(),
        "contact must fall off with separation: {n:?}"
    );
    assert_eq!(
        *n.last().unwrap(),
        0,
        "no contact once surfaces fully part: {n:?}"
    );

    // (3) It traces the CONCAVE surface, not the envelope: at a partial overlap
    //     the penetration is far below the bounding-volume overlap (ext.z − sep)
    //     — a hull/convex collider would report ~the full bounding overlap.
    let (dz_mid, _, pen_mid, _) = rows[2]; // 65%
    let bounding_overlap = ext.z - dz_mid;
    assert!(
        pen_mid < 0.5 * bounding_overlap,
        "concave penetration {pen_mid:.2} should be far below the bounding overlap \
         {bounding_overlap:.2} (else it is tracing the hull, not the surface)"
    );

    // (4) All contacts sit on the geometry.
    assert!(
        rows.iter().all(|r| r.3),
        "all contacts must be localized on the vertebra surfaces"
    );
}
