//! Rung 4b of the geometry-fidelity ladder — anatomical **L4–L5 facet contact**.
//!
//! Rung 4 proved the concave SDF collider (`ShapeConcave`) resolves non-convex
//! bone-on-bone contact, but z-stacked two *copies* of L4 synthetically. This
//! rung uses two **distinct** real vertebrae — L4 and L5 — at their **true
//! anatomical pose** (native `BodyParts3D` coordinates, identity poses), and
//! proves the contact localizes to the actual **zygapophyseal (facet) joints**,
//! not the vertebral bodies. This is the two-vertebra FSU substrate that
//! ligaments (rung 5), the bonded disc (rung 6), and FSU validation (rung 7)
//! all build on.
//!
//! **Everything is measured, nothing asserted via a proxy** — the ladder's
//! recurring lesson is that adversarial review caught an over-claim on every
//! prior rung whenever a comparison was *asserted* instead of *built and
//! measured*. In particular this test takes no anatomical-axis convention on
//! faith: it *locates* each vertebral body as the **deepest interior point of
//! the signed field** (the thickest solid mass), then states every localization
//! claim as a distance to those measured body centers. It also uses a **signed**
//! surface query, so "the facets nearly touch" means a measured positive
//! clearance, not an unsigned magnitude that cannot tell a 0.3 mm gap from a
//! 0.3 mm overlap.
//!
//! What it establishes:
//!
//! 1. **The facets nearly touch, and it is the facets.** The closest surface
//!    approach between the two vertebrae is a small *positive* (signed)
//!    clearance, at a point tens of mm from either body center — an articular
//!    process, not a body.
//! 2. **The bodies are disc-separated, so contact there is impossible.** At the
//!    vertebral bodies the surfaces are an order of magnitude farther apart (the
//!    disc space) than at the facets — far beyond the contact margin. This is
//!    what makes "contact localizes to the facets" a measured fact rather than a
//!    tautology about where two AABBs happen to overlap.
//! 3. **Concave contact fires at the facets, bilaterally, and nowhere near the
//!    bodies** — every concave contact stays tens of mm from both body centers.
//! 4. **Hull contrast (built + measured).** A convex-hull collider of each
//!    vertebra, over the same grid resolution and pose, fills the disc space:
//!    it jams an order of magnitude deeper *and* reaches markedly closer to the
//!    body centers — colliding where the real joint has clearance. Only the
//!    geometry differs, so the difference is the concavity.
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_L4_STL` +
//! `$CF_L5_STL` (`BodyParts3D` vertebrae are CC BY-SA, not committed). Run with:
//!
//! ```text
//! CF_L4_STL=/path/to/L4.stl CF_L5_STL=/path/to/L5.stl \
//!   cargo test -p cf-design-tests --release \
//!   --test rung4b_facet_contact -- --ignored --nocapture
//! ```

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::sync::Arc;

use cf_geometry::{Aabb, IndexedMesh};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfContact, SdfGrid, ShapeConcave, convex_hull};

/// Grid cell size (mm). The facet clearance is sub-mm and the joint band is a
/// few mm; 1 mm resolves it and localizes the contact tightly. Coordinates are
/// in millimetres (`BodyParts3D` scale).
const CELL: f64 = 1.0;

/// Radius (mm) around a measured body center counted as "at the vertebral body"
/// when probing the surface separation there. ~ a lumbar body's half-extent, so
/// the probe samples the body's endplate region, not the pedicles/facets.
const BODY_PROBE_R: f64 = 12.0;

/// The exact mesh-derived metric oracle (signed distance to a vertebra surface).
type MeshOracle = Signed<TriMeshDistance, PseudoNormalSign>;

/// Load, weld-repair, and KEEP native coordinates (do not recenter — the two
/// vertebrae must stay in their shared anatomical frame so they stack).
fn load_native(path_var: &str) -> IndexedMesh {
    let path =
        std::env::var(path_var).unwrap_or_else(|_| panic!("set ${path_var} to a vertebra STL"));
    let mut mesh = load_stl(&path).unwrap_or_else(|e| panic!("load {path_var}: {e}"));
    let rep = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!(
        "[{path_var}] welded {} verts -> {} verts / {} faces",
        rep.vertices_welded,
        mesh.vertices.len(),
        mesh.faces.len()
    );
    mesh
}

/// Build the exact signed distance oracle for a mesh (parry BVH + pseudo-normal
/// sign — metric by construction).
fn oracle(mesh: &IndexedMesh) -> MeshOracle {
    let dist = TriMeshDistance::new(mesh.clone()).unwrap();
    let sign = PseudoNormalSign::from_distance(&dist);
    Signed {
        distance: dist,
        sign,
    }
}

/// Locate the vertebral body WITHOUT any axis assumption: the deepest interior
/// point of the signed field (most negative distance) is the thickest solid
/// mass, which for a vertebra is the body. Coarse AABB scan. Returns the point
/// and its depth (negative = inside).
fn body_center(mesh: &IndexedMesh, sdf: &MeshOracle) -> (Point3<f64>, f64) {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let step = 2.0;
    let span = bbox.max - bbox.min;
    let count = |len: f64| (len / step).ceil() as usize + 1;
    let (nx, ny, nz) = (count(span.x), count(span.y), count(span.z));
    let (mut depth, mut center) = (f64::MAX, Point3::origin());
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let offset = Vector3::new(ix as f64, iy as f64, iz as f64) * step;
                let pt = bbox.min + offset;
                let val = sdf.evaluate(pt);
                if val < depth {
                    depth = val;
                    center = pt;
                }
            }
        }
    }
    (center, depth)
}

/// Sample an oracle's field into a grid over its mesh's padded AABB. The grid
/// stores absolute native coordinates, so an identity pose places the shape at
/// its anatomical location. Borrows the oracle (no rebuilt BVH).
fn grid(mesh: &IndexedMesh, sdf: &MeshOracle) -> Arc<SdfGrid> {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let pad = 4.0 * CELL;
    let lo = bbox.min - Vector3::repeat(pad);
    let span = (bbox.max + Vector3::repeat(pad)) - lo;
    let cells = |len: f64| (len / CELL).ceil() as usize + 1;
    Arc::new(SdfGrid::from_fn(
        cells(span.x),
        cells(span.y),
        cells(span.z),
        CELL,
        lo,
        |pt| sdf.evaluate(pt),
    ))
}

/// Convex hull of a mesh as an `IndexedMesh` — the comparison object.
fn hull_of(mesh: &IndexedMesh) -> IndexedMesh {
    let hull = convex_hull(&mesh.vertices, None).expect("convex hull");
    let mut hm = IndexedMesh::new();
    hm.vertices = hull.vertices;
    hm.faces = hull.faces;
    hm
}

/// Native-frame identity pose (grids already carry absolute coordinates).
fn identity() -> Pose {
    Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    }
}

fn contact(a: &Arc<SdfGrid>, b: &Arc<SdfGrid>) -> Vec<SdfContact> {
    compute_shape_contact(
        &ShapeConcave::new(Arc::clone(a)),
        &identity(),
        &ShapeConcave::new(Arc::clone(b)),
        &identity(),
        CELL,
        256,
    )
}

fn max_pen(cs: &[SdfContact]) -> f64 {
    cs.iter().map(|c| c.penetration).fold(0.0_f64, f64::max)
}

/// Closest any contact in the set gets to *either* body center — the localization
/// discriminator (large = the contact set stays away from both bodies).
fn nearest_to_a_body(cs: &[SdfContact], b0: Point3<f64>, b1: Point3<f64>) -> f64 {
    cs.iter()
        .map(|c| (c.point - b0).norm().min((c.point - b1).norm()))
        .fold(f64::MAX, f64::min)
}

#[test]
#[ignore = "needs local L4+L5 vertebra meshes via $CF_L4_STL/$CF_L5_STL (CC BY-SA, not committed)"]
fn l4_l5_facet_contact_localizes_to_the_joints_not_the_bodies() {
    let l4 = load_native("CF_L4_STL");
    let l5 = load_native("CF_L5_STL");
    let o4 = oracle(&l4);
    let o5 = oracle(&l5);

    // Native pose stacks the vertebrae so the posterior elements overlap in z
    // (context — NOT the localization proof; two AABBs overlapping in z is not
    // by itself evidence of facet contact).
    let (a4, a5) = (
        Aabb::from_points(l4.vertices.iter()),
        Aabb::from_points(l5.vertices.iter()),
    );
    let (z_lo, z_hi) = (a4.min.z.max(a5.min.z), a4.max.z.min(a5.max.z));
    println!(
        "z-overlap band = [{z_lo:.1}, {z_hi:.1}]  ({:.1} mm)",
        z_hi - z_lo
    );
    assert!(
        z_hi > z_lo,
        "L4 and L5 must overlap in z at their native pose"
    );

    // Locate each vertebral body as the deepest interior point (no axis
    // assumption). A genuine body is a thick solid mass — several mm deep.
    let (b4, depth4) = body_center(&l4, &o4);
    let (b5, depth5) = body_center(&l5, &o5);
    println!(
        "body centers: L4 ({:.1},{:.1},{:.1}) depth {depth4:.1} | L5 ({:.1},{:.1},{:.1}) depth {depth5:.1}",
        b4.x, b4.y, b4.z, b5.x, b5.y, b5.z
    );
    assert!(
        depth4 < -5.0 && depth5 < -5.0,
        "body centers must be thick solid mass (depths {depth4:.1}, {depth5:.1} mm)"
    );

    // (1) Facet clearance — SIGNED, so a positive value means the surfaces are
    //     apart (clearance), not overlapping. The closest approach is far from
    //     either body center → it is an articular process, not a body.
    let mut facet_gap = f64::MAX;
    let mut at = Point3::origin();
    for v in &l4.vertices {
        let d = o5.evaluate(*v);
        if d < facet_gap {
            facet_gap = d;
            at = *v;
        }
    }
    let approach_to_body = (at - b4).norm().min((at - b5).norm());
    println!(
        "closest L4<->L5 signed approach = {facet_gap:.3} mm at ({:.1},{:.1},{:.1}); \
         {approach_to_body:.1} mm from nearest body center",
        at.x, at.y, at.z
    );
    assert!(
        facet_gap > 0.0 && facet_gap < 1.0,
        "facets should nearly touch with a positive clearance (signed gap {facet_gap:.3} mm; \
         >0 proves no interpenetration)"
    );
    assert!(
        approach_to_body > 20.0,
        "closest approach must be a facet, far from either body center ({approach_to_body:.1} mm)"
    );

    // (2) The bodies are disc-separated: the surface separation AT a body center
    //     is an order of magnitude larger than the facet clearance and far
    //     beyond the contact margin — so contact there is geometrically
    //     impossible, and any contact must be at the facets. (Non-tautological:
    //     this measures the disc gap, it does not assume it.)
    let mut body_sep = f64::MAX;
    for v in &l4.vertices {
        if (v - b4).norm() <= BODY_PROBE_R {
            body_sep = body_sep.min(o5.evaluate(*v));
        }
    }
    println!("surface separation at the vertebral body = {body_sep:.2} mm (the disc space)");
    assert!(
        body_sep > 5.0 && body_sep > 8.0 * facet_gap,
        "bodies must be disc-separated (sep {body_sep:.2} mm) far beyond the {CELL} mm margin \
         and the {facet_gap:.3} mm facet clearance"
    );

    // Build both colliders over identical resolution + pose; only geometry
    // differs (concave true surface vs its convex hull).
    let (cg4, cg5) = (grid(&l4, &o4), grid(&l5, &o5));
    let (hull4, hull5) = (hull_of(&l4), hull_of(&l5));
    let (ho4, ho5) = (oracle(&hull4), oracle(&hull5));
    let (hg4, hg5) = (grid(&hull4, &ho4), grid(&hull5, &ho5));
    let cc = contact(&cg4, &cg5);
    let hh = contact(&hg4, &hg5);
    let (cc_pen, hh_pen) = (max_pen(&cc), max_pen(&hh));
    let (cc_near, hh_near) = (
        nearest_to_a_body(&cc, b4, b5),
        nearest_to_a_body(&hh, b4, b5),
    );
    let cc_x = cc.iter().map(|c| c.point.x);
    let (cc_xmin, cc_xmax) = (
        cc_x.clone().fold(f64::MAX, f64::min),
        cc_x.fold(f64::MIN, f64::max),
    );
    println!(
        "[concave] n={} maxpen={cc_pen:.2} nearest-body={cc_near:.1} x[{cc_xmin:.1},{cc_xmax:.1}]",
        cc.len()
    );
    println!(
        "[hull   ] n={} maxpen={hh_pen:.2} nearest-body={hh_near:.1}",
        hh.len()
    );

    // (3) Concave contact fires at the facets, bilaterally, and nowhere near the
    //     bodies. Bilaterality is stated relative to the MEASURED mid-sagittal
    //     line (mean of the two body-center x's), not an assumed axis label.
    assert!(
        !cc.is_empty(),
        "concave collider must contact at the facet joints"
    );
    assert!(
        cc_near > 20.0,
        "every concave contact must stay far from both body centers (closest {cc_near:.1} mm)"
    );
    let mid_x = 0.5 * (b4.x + b5.x);
    assert!(
        cc_xmin < mid_x - 10.0 && cc_xmax > mid_x + 10.0,
        "concave contacts must straddle the mid-sagittal line at x={mid_x:.1} \
         (both left + right facet joints engage): x[{cc_xmin:.1},{cc_xmax:.1}]"
    );

    // (4) Hull contrast — built + measured, not a proxy. The hull fills the disc
    //     space: it jams far deeper AND reaches markedly closer to the body
    //     centers than the concave collider, which stays out at the facets.
    assert!(
        !hh.is_empty(),
        "hull collider must engage to compare against it"
    );
    assert!(
        hh_pen > 10.0 && hh_pen > 5.0 * cc_pen.max(0.1),
        "hull jams the envelope far deeper than the light concave facet contact \
         (hull {hh_pen:.2} mm vs concave {cc_pen:.2} mm)"
    );
    assert!(
        hh_near < cc_near - 8.0,
        "hull contact reaches toward the disc-separated bodies while concave stays at the facets \
         (hull nearest-body {hh_near:.1} mm vs concave {cc_near:.1} mm)"
    );
}
