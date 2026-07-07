//! Headless FSU scene builder (Bevy-free).
//!
//! Reproduces the geometry recipes proven in the rung 4b / 5 / 7 integration
//! tests to assemble the static L4–L5 Functional Spinal Unit as plain data
//! for rendering — everything in native BodyParts3D millimetres (a static
//! view runs no solver, so no recenter/scale is needed; the three meshes
//! already stack in their shared anatomical frame):
//!
//!   * the two vertebra surfaces (welded), plus the disc STL shell;
//!   * the field-derived ligament attachment sites (ALL + interspinous),
//!     located from the signed field — no axis convention on faith;
//!   * the facet near-contact points, from the same `ShapeConcave` SDF
//!     collision the rung 4b/7 tests use, at the neutral pose.
//!
//! Kept LOCAL to the viewer for this rung. The shared field-recipe helpers
//! (`load` / `oracle` / `body_center` / `segment_frame` / `extreme_vertex` /
//! `facet_grid`) are still duplicated across the rung7 + rung6c tests; once
//! this viewer proves the reusable shape they lift into one graded home
//! (the next rung). The values here mirror those tests exactly.

use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use mesh_types::{Aabb, IndexedMesh};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfContact, SdfGrid, ShapeConcave};

/// The exact mesh-derived metric oracle (parry BVH distance + pseudo-normal
/// inside/outside sign) — metric by construction. Mirrors the ladder tests.
type MeshOracle = Signed<TriMeshDistance, PseudoNormalSign>;

// ── Facet / ligament constants (native mm frame) — matched to rung 4b/7. ──
const FACET_CELL: f64 = 1.0; // facet SDF grid cell (mm)
const MIDLINE_TOL: f64 = 8.0; // mm off mid-sagittal for a near-midline site
const BODY_RADIUS: f64 = 30.0; // mm: keep the ALL site on the vertebral body
const FACET_MAX_CONTACTS: usize = 256; // per-pair contact cap (rung 4b/7; 184 at neutral)

/// A ligament rendered as a straight line between two field-derived sites.
pub struct Ligament {
    pub name: &'static str,
    pub inferior: Point3<f64>, // attachment on L5
    pub superior: Point3<f64>, // attachment on L4
}

/// The assembled static scene, as plain geometry in native millimetres.
pub struct FsuScene {
    pub l4: IndexedMesh,
    pub l5: IndexedMesh,
    pub disc: IndexedMesh,
    pub ligaments: Vec<Ligament>,
    /// Facet near-contact points (within the 1 mm detection margin at the
    /// neutral pose) — the articular-process contact region.
    pub facet_contacts: Vec<Point3<f64>>,
    /// Combined bounding box across all three meshes, for camera framing.
    pub aabb: Aabb,
    /// Co-registration / degeneracy warnings surfaced during assembly (empty
    /// for a well-formed, co-registered trio). Shown in the panel + on stderr
    /// so a misaligned or mismatched input is never presented as a valid FSU.
    pub warnings: Vec<String>,
}

/// The shared segmental frame, derived from the two vertebrae (native mm).
struct SegmentFrame {
    b4: Point3<f64>,         // L4 body centre
    b5: Point3<f64>,         // L5 body centre
    superior: Vector3<f64>,  // L5 → L4 body centres (SI axis)
    posterior: Vector3<f64>, // body → arch, ⟂ superior
    ml: Vector3<f64>,        // medio-lateral, superior × posterior (vertebral-frame ML)
}

/// Load + weld-repair a vertebra/disc STL, KEEPING native coordinates (the
/// three meshes must stay in their shared anatomical frame so they stack).
fn load(path: &Path) -> Result<IndexedMesh> {
    let mut mesh = load_stl(path).with_context(|| format!("loading STL {}", path.display()))?;
    let rep = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!(
        "[{}] welded {} verts -> {} verts / {} faces",
        path.display(),
        rep.vertices_welded,
        mesh.vertices.len(),
        mesh.faces.len()
    );
    Ok(mesh)
}

/// Build the exact signed-distance oracle for a mesh (metric by construction).
fn oracle(mesh: &IndexedMesh) -> Result<MeshOracle> {
    let distance = TriMeshDistance::new(mesh.clone())
        .map_err(|e| anyhow::anyhow!("build distance oracle: {e:?}"))?;
    let sign = PseudoNormalSign::from_distance(&distance);
    Ok(Signed { distance, sign })
}

/// Vertebral body centre = deepest interior point of the signed field (no axis
/// assumption; the thickest solid mass is the body). Coarse 2 mm AABB scan.
/// Returns (point, depth < 0 inside).
fn body_center(mesh: &IndexedMesh, sdf: &MeshOracle) -> (Point3<f64>, f64) {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let step = 2.0;
    let span = bbox.max - bbox.min;
    let n = |l: f64| (l / step).ceil() as usize + 1;
    let (mut depth, mut center) = (f64::MAX, Point3::origin());
    for iz in 0..n(span.z) {
        for iy in 0..n(span.y) {
            for ix in 0..n(span.x) {
                let p = bbox.min + Vector3::new(ix as f64, iy as f64, iz as f64) * step;
                let v = sdf.evaluate(p);
                if v < depth {
                    depth = v;
                    center = p;
                }
            }
        }
    }
    (center, depth)
}

fn centroid(mesh: &IndexedMesh) -> Point3<f64> {
    let s: Vector3<f64> = mesh.vertices.iter().map(|v| v.coords).sum();
    Point3::from(s / mesh.vertices.len() as f64)
}

/// Derive the anatomical frame from the field: superior = L5→L4 body centres,
/// posterior = body→mesh-centroid (arch pulls it back, orthogonalised against
/// superior), ML = superior × posterior.
fn segment_frame(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    o4: &MeshOracle,
    o5: &MeshOracle,
) -> Result<SegmentFrame> {
    let (b4, d4) = body_center(l4, o4);
    let (b5, d5) = body_center(l5, o5);
    if d4 >= -5.0 || d5 >= -5.0 {
        bail!(
            "body centres must be thick solid mass (depths {d4:.1}, {d5:.1}) — are the L4/L5 STLs swapped or non-manifold?"
        );
    }
    // Guard the normalize below: coincident body centres (duplicated or
    // overlapping input meshes) would divide by zero → an all-NaN frame.
    let sep = (b4 - b5).norm();
    if sep < 1.0 {
        bail!(
            "L4/L5 body centres are {sep:.2} mm apart — duplicate or overlapping meshes? (expected the two vertebrae stacked ~20 mm apart)"
        );
    }
    let superior = (b4 - b5).normalize();
    // Posterior = the body→arch direction, orthogonalised against superior.
    // Guard THIS normalize too: if the arch offset is (near-)parallel to the SI
    // axis (an arch-stripped or synthetic mesh), the orthogonal part is ~0 and
    // `.normalize()` would yield a silent NaN frame — misattributed downstream
    // as a "degenerate mesh" ligament failure rather than a lost posterior.
    let raw_post = (centroid(l4) - b4) + (centroid(l5) - b5);
    let orthogonal = raw_post - raw_post.dot(&superior) * superior;
    if orthogonal.norm() < 1e-6 {
        bail!(
            "cannot derive a posterior direction — each vertebra's centroid sits directly along the SI axis over its body centre (arch-stripped or synthetic mesh?)"
        );
    }
    let posterior = orthogonal.normalize();
    let ml = superior.cross(&posterior).normalize();
    Ok(SegmentFrame {
        b4,
        b5,
        superior,
        posterior,
        ml,
    })
}

/// The disc's principal medio-lateral axis = its widest AABB extent, snapped to
/// the coordinate axis (rung-7's `ml_scaled` derivation). This is the CANONICAL
/// flexion/extension axis rung-7 adopted over the vertebral-frame ML: the latter
/// is tilted ~19° by the lordotic wedge + coarse body-centre localisation, so
/// the ligament midline filter uses THIS axis to match the validated FSU.
fn disc_principal_ml(disc: &IndexedMesh) -> Vector3<f64> {
    let bbox = Aabb::from_points(disc.vertices.iter());
    let span = bbox.max - bbox.min;
    let extents = [span.x, span.y, span.z];
    let widest = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap_or(0);
    let mut ml = Vector3::zeros();
    ml[widest] = 1.0;
    ml
}

/// Farthest near-midline surface vertex from `origin` along `dir`, within
/// `max_radius` (∞ for the far spinous tip; a body radius to force the ALL
/// site onto the vertebral-body rim). `None` when no vertex passes the
/// midline/radius filter (a degenerate mesh) — callers must not silently
/// fall back to `origin` (the interior body centre), which would draw a
/// ligament terminating inside the bone.
fn extreme_vertex(
    mesh: &IndexedMesh,
    origin: Point3<f64>,
    dir: Vector3<f64>,
    ml: Vector3<f64>,
    max_radius: f64,
) -> Option<Point3<f64>> {
    let mut best = None;
    let mut best_proj = f64::MIN;
    for v in &mesh.vertices {
        let d = v - origin;
        if d.dot(&ml).abs() > MIDLINE_TOL || d.norm() > max_radius {
            continue;
        }
        let proj = d.dot(&dir);
        if proj > best_proj {
            best_proj = proj;
            best = Some(*v);
        }
    }
    best
}

/// The two modelled ligaments: anterior longitudinal (ALL, on the body rim,
/// `-posterior`) and interspinous (ISP, at the spinous tips, `+posterior`) —
/// endpoints field-derived. `ml` is the disc principal axis (rung-7's canonical
/// flexion axis), used as the mid-sagittal-plane normal for the near-midline
/// filter so the sites match the validated FSU. A ligament whose sites can't be
/// located on a degenerate mesh is skipped with a warning rather than drawn wrong.
fn build_ligaments(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    frame: &SegmentFrame,
    ml: Vector3<f64>,
    warnings: &mut Vec<String>,
) -> Vec<Ligament> {
    let (b4, b5, post) = (frame.b4, frame.b5, frame.posterior);
    let mut ligaments = Vec::new();

    match (
        extreme_vertex(l5, b5, -post, ml, BODY_RADIUS),
        extreme_vertex(l4, b4, -post, ml, BODY_RADIUS),
    ) {
        (Some(inferior), Some(superior)) => {
            ligaments.push(Ligament {
                name: "ALL",
                inferior,
                superior,
            });
        }
        _ => warnings.push(
            "ALL ligament: no near-midline body-rim vertex found (degenerate mesh?)".to_string(),
        ),
    }

    match (
        extreme_vertex(l5, b5, post, ml, f64::INFINITY),
        extreme_vertex(l4, b4, post, ml, f64::INFINITY),
    ) {
        (Some(inferior), Some(superior)) => {
            ligaments.push(Ligament {
                name: "ISP",
                inferior,
                superior,
            });
        }
        _ => warnings.push(
            "ISP ligament: no near-midline spinous vertex found (degenerate mesh?)".to_string(),
        ),
    }

    ligaments
}

/// Co-registration sanity checks (rung-7's validity cross-checks, cheaply
/// reproduced from AABBs — no SDF needed): the disc's principal ML must agree
/// with the vertebral frame ML, and the disc must be seated between the two body
/// centres along the superior axis. A grossly mismatched or misaligned trio
/// (wrong specimen, wrong disc level, rotated frame) trips these — so it is never
/// shown as a literature-validated FSU.
///
/// The ML agreement mirrors rung-7's own obliquity assert (`disc_ml · frame.ml >
/// 0.9`): `disc_ml` is a coordinate axis, `frame.ml` continuous, so the 0.9 band
/// assumes the anatomy sits roughly axis-aligned — true for the BodyParts3D
/// frame this tool targets (rung 7 measured ~0.95), not for arbitrary rotations.
fn coregistration_warnings(
    disc: &IndexedMesh,
    frame: &SegmentFrame,
    disc_ml: Vector3<f64>,
) -> Vec<String> {
    let mut warnings = Vec::new();

    let agreement = disc_ml.dot(&frame.ml).abs();
    if agreement < 0.9 {
        warnings.push(format!(
            "disc ML vs vertebral ML agreement {agreement:.2} < 0.90 — meshes may be misaligned or from different specimens"
        ));
    }

    // Disc centre must sit between the L4/L5 body centres along the SI axis.
    let bbox = Aabb::from_points(disc.vertices.iter());
    let disc_center = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
    let along = (disc_center - frame.b5).dot(&frame.superior);
    let separation = (frame.b4 - frame.b5).dot(&frame.superior);
    if along < 0.0 || along > separation {
        warnings.push(format!(
            "disc centre is not between the L4/L5 bodies ({along:.1} mm of {separation:.1} mm along SI) — wrong disc level?"
        ));
    }

    warnings
}

/// Absolute-coordinate SDF grid over a vertebra (identity pose places it
/// anatomically). Same recipe as rung 4b/7's `facet_grid`.
fn facet_grid(mesh: &IndexedMesh, sdf: &MeshOracle) -> Arc<SdfGrid> {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let pad = 4.0 * FACET_CELL;
    let lo = bbox.min - Vector3::repeat(pad);
    let span = (bbox.max + Vector3::repeat(pad)) - lo;
    let cells = |l: f64| (l / FACET_CELL).ceil() as usize + 1;
    Arc::new(SdfGrid::from_fn(
        cells(span.x),
        cells(span.y),
        cells(span.z),
        FACET_CELL,
        lo,
        |p| sdf.evaluate(p),
    ))
}

/// Facet near-contact points at the NEUTRAL pose (both vertebrae at identity).
/// `compute_shape_contact` returns every pair within the detection margin
/// (`FACET_CELL` = 1 mm); at neutral the articular surfaces nearly touch
/// (rung 4b measured a +0.3 mm closest approach) so these are the near-contact
/// facet region, not interpenetrations — we render them all rather than
/// filtering `penetration > 0` (which is the force-relevant subset, empty at
/// neutral).
fn facet_contact_points(g4: &Arc<SdfGrid>, g5: &Arc<SdfGrid>) -> Vec<Point3<f64>> {
    let identity = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };
    let contacts: Vec<SdfContact> = compute_shape_contact(
        &ShapeConcave::new(Arc::clone(g4)),
        &identity,
        &ShapeConcave::new(Arc::clone(g5)),
        &identity,
        FACET_CELL,
        FACET_MAX_CONTACTS,
    );
    contacts.iter().map(|c| c.point).collect()
}

fn combined_aabb(meshes: &[&IndexedMesh]) -> Aabb {
    Aabb::from_points(meshes.iter().flat_map(|m| m.vertices.iter()))
}

/// Assemble the static FSU scene from the three STL paths (native mm).
pub fn build(l4_path: &Path, l5_path: &Path, disc_path: &Path) -> Result<FsuScene> {
    let l4 = load(l4_path)?;
    let l5 = load(l5_path)?;
    let disc = load(disc_path)?;

    let o4 = oracle(&l4)?;
    let o5 = oracle(&l5)?;
    let frame = segment_frame(&l4, &l5, &o4, &o5)?;

    // The disc's own principal axis is the canonical ML (rung-7): use it for
    // both the ligament midline filter and the co-registration cross-check.
    let disc_ml = disc_principal_ml(&disc);
    let mut warnings = coregistration_warnings(&disc, &frame, disc_ml);
    let ligaments = build_ligaments(&l4, &l5, &frame, disc_ml, &mut warnings);

    let g4 = facet_grid(&l4, &o4);
    let g5 = facet_grid(&l5, &o5);
    let facet_contacts = facet_contact_points(&g4, &g5);
    for lig in &ligaments {
        println!(
            "ligament {}: L5 site {:.1?} → L4 site {:.1?}  ({:.1} mm)",
            lig.name,
            lig.inferior.coords.as_slice(),
            lig.superior.coords.as_slice(),
            (lig.superior - lig.inferior).norm()
        );
    }
    println!(
        "assembled FSU: {} facet near-contact points",
        facet_contacts.len()
    );
    for w in &warnings {
        eprintln!("WARNING: {w}");
    }

    let aabb = combined_aabb(&[&l4, &l5, &disc]);
    Ok(FsuScene {
        l4,
        l5,
        disc,
        ligaments,
        facet_contacts,
        aabb,
        warnings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A vertices-only mesh (the coordinate-free helpers read only `vertices`;
    /// a dummy face keeps the type well-formed).
    fn mesh(verts: &[[f64; 3]]) -> IndexedMesh {
        IndexedMesh {
            vertices: verts
                .iter()
                .map(|&[x, y, z]| Point3::new(x, y, z))
                .collect(),
            faces: vec![[0, 1, 2]],
        }
    }

    #[test]
    fn extreme_vertex_picks_farthest_along_dir() {
        // Three near-midline points along +x; the farthest projects highest.
        let m = mesh(&[[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]);
        let got = extreme_vertex(
            &m,
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        assert_eq!(got, Some(Point3::new(10.0, 0.0, 0.0)));
    }

    #[test]
    fn extreme_vertex_rejects_off_midline_and_beyond_radius() {
        // Farthest-along-x (x=20) is off-midline (z=20 > MIDLINE_TOL=8) → rejected;
        // x=10 is on-midline but beyond max_radius=8 → rejected; so x=5 wins.
        let m = mesh(&[[5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 20.0]]);
        let got = extreme_vertex(&m, Point3::origin(), Vector3::x(), Vector3::z(), 8.0);
        assert_eq!(got, Some(Point3::new(5.0, 0.0, 0.0)));
    }

    #[test]
    fn extreme_vertex_opposite_directions_pick_opposite_ends() {
        // The ALL/ISP split is exactly ±posterior: opposite `dir` → opposite tips.
        let m = mesh(&[[-7.0, 0.0, 0.0], [0.0, 0.0, 0.0], [7.0, 0.0, 0.0]]);
        let anterior = extreme_vertex(
            &m,
            Point3::origin(),
            -Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        let posterior = extreme_vertex(
            &m,
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        assert_eq!(anterior, Some(Point3::new(-7.0, 0.0, 0.0)));
        assert_eq!(posterior, Some(Point3::new(7.0, 0.0, 0.0)));
    }

    #[test]
    fn extreme_vertex_none_when_nothing_qualifies() {
        // Every vertex is off-midline (z=20 > MIDLINE_TOL=8) → no fallback to origin.
        let m = mesh(&[[5.0, 0.0, 20.0], [10.0, 0.0, 20.0]]);
        let got = extreme_vertex(
            &m,
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        assert_eq!(got, None);
    }

    #[test]
    fn combined_aabb_spans_every_mesh() {
        let a = mesh(&[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [0.5, 0.5, 0.5]]);
        let b = mesh(&[[-2.0, 0.0, 0.0], [3.0, 4.0, 5.0], [1.0, 1.0, 1.0]]);
        let bb = combined_aabb(&[&a, &b]);
        assert_eq!(bb.min, Point3::new(-2.0, 0.0, 0.0));
        assert_eq!(bb.max, Point3::new(3.0, 4.0, 5.0));
    }

    #[test]
    fn centroid_is_the_mean_vertex() {
        let m = mesh(&[[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [0.0, 3.0, 3.0]]);
        assert_eq!(centroid(&m), Point3::new(1.0, 1.0, 1.0));
    }
}
