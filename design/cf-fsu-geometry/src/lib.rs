//! Shared geometry recipes for the lumbar **Functional Spinal Unit** (FSU).
//!
//! These helpers turn a raw vertebra/disc surface mesh into the anatomical
//! quantities an FSU model needs, deriving everything **from the signed
//! field — no coordinate convention on faith**:
//!
//!   * [`load`] — read + weld-repair an STL, keeping native coordinates so
//!     sibling vertebrae stay in their shared frame;
//!   * [`oracle`] — build the exact mesh-derived signed-distance [`MeshOracle`];
//!   * [`segment_frame`] — derive the segmental frame (superior / posterior / ML)
//!     from two vertebrae, locating each vertebral body as the deepest interior
//!     point of the field;
//!   * [`extreme_vertex`] — pick a near-midline attachment vertex along a
//!     direction (ligament sites);
//!   * [`facet_grid`] — sample the field into an [`SdfGrid`] for facet contact.
//!
//! The recipes were proven in the geometry-fidelity ladder (rung 4b/5/7) and are
//! consumed by the `cf-spine-viewer` tool; those integration tests still carry
//! their own copies and migrate to this crate in a follow-up. The crate is
//! deliberately **sim-core-free**: [`facet_grid`] builds a `cf_geometry::SdfGrid`
//! directly, and the `ShapeConcave` collision that consumes the grid stays in
//! each caller.

use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use cf_geometry::{Aabb, IndexedMesh, SdfGrid};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, Vector3};

/// Facet SDF grid cell size, in millimetres (rung 4b/7).
pub const FACET_CELL: f64 = 1.0;
/// Maximum distance off the mid-sagittal plane for a near-midline site (mm).
/// Internal to [`extreme_vertex`] — not part of the public recipe-parameter set.
const MIDLINE_TOL: f64 = 8.0;
/// Radius that keeps an anterior-longitudinal site on the vertebral body (mm).
pub const BODY_RADIUS: f64 = 30.0;
/// Per-pair facet contact cap (rung 4b/7); the neutral pose stays well under it.
pub const FACET_MAX_CONTACTS: usize = 256;

/// Coarse scan step for [`body_center`], in millimetres.
const BODY_CENTER_STEP: f64 = 2.0;
/// A body centre shallower than this (mm, negative = interior) is not thick
/// solid mass — the mesh is likely swapped, non-manifold, or not a vertebra.
const MIN_BODY_DEPTH: f64 = -5.0;
/// Below this body-centre separation (mm) the two vertebrae are treated as
/// coincident (duplicate/overlapping meshes) — guards a divide-by-zero.
const MIN_BODY_SEPARATION: f64 = 1.0;
/// Below this the orthogonalised posterior vector is treated as degenerate.
const MIN_POSTERIOR_NORM: f64 = 1e-6;

/// The exact mesh-derived metric oracle: a parry BVH distance composed with a
/// pseudo-normal inside/outside sign. Metric by construction (satisfies the
/// `|∇| ≈ 1` precondition the fidelity harness needs).
pub type MeshOracle = Signed<TriMeshDistance, PseudoNormalSign>;

/// Load + weld-repair a vertebra/disc STL at `path`, **keeping native
/// coordinates** (do not recenter — sibling vertebrae must stay in their shared
/// anatomical frame so they stack).
///
/// # Errors
/// Returns an error if the STL cannot be read, or if the mesh has no vertices
/// or no faces after repair (empty, unreadable, or fully-degenerate) — either
/// would otherwise yield a meaningless field or a `NaN` bounding-box centre
/// downstream.
pub fn load(path: &Path) -> Result<IndexedMesh> {
    let mut mesh = load_stl(path).with_context(|| format!("loading STL {}", path.display()))?;
    // Repair mutates `mesh` in place; the returned weld statistics are diagnostic
    // only and intentionally not surfaced by this (silent) library API.
    let _stats = repair_mesh(&mut mesh, &RepairParams::for_scans());
    // A usable oracle needs both vertices AND faces: a vertices-but-no-faces
    // mesh (all triangles degenerate) would build a `TriMeshDistance` over zero
    // triangles and yield a meaningless field, so reject it here with a clear
    // message rather than let it surface as a misleading downstream bail.
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        bail!(
            "{} has no usable geometry after repair ({} vertices, {} faces) — empty, unreadable, or fully-degenerate STL",
            path.display(),
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }
    Ok(mesh)
}

/// Build the exact signed-distance [`MeshOracle`] for a mesh.
///
/// # Errors
/// Returns an error if the distance structure cannot be built from the mesh
/// (e.g. an empty mesh).
pub fn oracle(mesh: &IndexedMesh) -> Result<MeshOracle> {
    let distance = TriMeshDistance::new(mesh.clone())
        .map_err(|e| anyhow::anyhow!("build distance oracle: {e:?}"))?;
    let sign = PseudoNormalSign::from_distance(&distance);
    Ok(Signed { distance, sign })
}

/// Vertebral body centre = the deepest interior point of the signed field (the
/// thickest solid mass, no axis assumption). Coarse `BODY_CENTER_STEP`-mm AABB
/// scan; returns `(point, depth)` where `depth < 0` is interior. Used internally
/// by [`segment_frame`].
// Grid indices/counts are small non-negative integers over an mm-scale AABB;
// the f64↔usize casts cannot truncate, lose sign, or lose precision in practice.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn body_center(mesh: &IndexedMesh, sdf: &MeshOracle) -> (Point3<f64>, f64) {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let span = bbox.max - bbox.min;
    let steps = |len: f64| (len / BODY_CENTER_STEP).ceil().max(0.0) as usize + 1;
    let (mut depth, mut center) = (f64::MAX, Point3::origin());
    for iz in 0..steps(span.z) {
        for iy in 0..steps(span.y) {
            for ix in 0..steps(span.x) {
                let offset = Vector3::new(ix as f64, iy as f64, iz as f64) * BODY_CENTER_STEP;
                let p = bbox.min + offset;
                let value = sdf.evaluate(p);
                if value < depth {
                    depth = value;
                    center = p;
                }
            }
        }
    }
    (center, depth)
}

/// The mean of a mesh's vertices (used internally by [`segment_frame`]).
// A vertex count fits f64 exactly for any realistic mesh; the precision-loss
// cast is benign.
#[allow(clippy::cast_precision_loss)]
fn centroid(mesh: &IndexedMesh) -> Point3<f64> {
    let sum: Vector3<f64> = mesh.vertices.iter().map(|v| v.coords).sum();
    Point3::from(sum / mesh.vertices.len() as f64)
}

/// The shared segmental frame, derived from the two vertebrae (native mm).
///
/// All directions are unit vectors; the fields are public so callers can project
/// sites/poses onto the frame.
#[derive(Debug, Clone, Copy)]
pub struct SegmentFrame {
    /// L4 (superior vertebra) body centre.
    pub b4: Point3<f64>,
    /// L5 (inferior vertebra) body centre.
    pub b5: Point3<f64>,
    /// Superior direction: L5 → L4 body centres (the SI axis).
    pub superior_axis: Vector3<f64>,
    /// Posterior direction: body → arch, orthogonalised against `superior_axis`.
    pub posterior: Vector3<f64>,
    /// Medio-lateral direction: `superior_axis × posterior` (vertebral-frame ML).
    pub ml: Vector3<f64>,
}

/// Derive the anatomical frame from the field: `superior` = L5→L4 body centres,
/// `posterior` = body→mesh-centroid (the arch pulls it back, orthogonalised
/// against superior), `ml` = superior × posterior.
///
/// Each body centre is the deepest interior point of its field. This assumes the
/// **vertebral body is the thickest solid mass** — true for the lumbar vertebrae
/// this crate targets (validated on L4/L5 in rung 4b/7); a mesh whose posterior
/// elements enclose a deeper interior region would yield a posterior "body"
/// centre, so reuse on other anatomy should confirm the centre lands in the
/// centrum.
///
/// # Errors
/// Returns an error when the inputs are degenerate: a body centre that is not
/// thick solid mass (`MIN_BODY_DEPTH`), coincident body centres
/// (`MIN_BODY_SEPARATION` — a divide-by-zero guard), or an arch offset parallel
/// to the SI axis so no posterior direction exists (`MIN_POSTERIOR_NORM`). Each
/// guard prevents a silent all-`NaN` frame.
pub fn segment_frame(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    o4: &MeshOracle,
    o5: &MeshOracle,
) -> Result<SegmentFrame> {
    let (b4, d4) = body_center(l4, o4);
    let (b5, d5) = body_center(l5, o5);
    if d4 >= MIN_BODY_DEPTH || d5 >= MIN_BODY_DEPTH {
        bail!(
            "body centres must be thick solid mass (depths {d4:.1}, {d5:.1}) — are the L4/L5 STLs swapped or non-manifold?"
        );
    }
    let separation = (b4 - b5).norm();
    if separation < MIN_BODY_SEPARATION {
        bail!(
            "L4/L5 body centres are {separation:.2} mm apart — duplicate or overlapping meshes? (expected the two vertebrae stacked ~20 mm apart)"
        );
    }
    let superior_axis = (b4 - b5).normalize();
    let raw_posterior = (centroid(l4) - b4) + (centroid(l5) - b5);
    let orthogonal = raw_posterior - raw_posterior.dot(&superior_axis) * superior_axis;
    if orthogonal.norm() < MIN_POSTERIOR_NORM {
        bail!(
            "cannot derive a posterior direction — each vertebra's centroid sits directly along the SI axis over its body centre (arch-stripped or synthetic mesh?)"
        );
    }
    let posterior = orthogonal.normalize();
    let ml = superior_axis.cross(&posterior).normalize();
    Ok(SegmentFrame {
        b4,
        b5,
        superior_axis,
        posterior,
        ml,
    })
}

/// Farthest near-midline surface vertex from `origin` along `dir`.
///
/// `max_radius` bounds the search (`f64::INFINITY` for the far spinous tip; a
/// body radius forces an anterior site onto the vertebral-body rim). `ml` is the
/// mid-sagittal-plane normal for the near-midline filter (keep vertices with
/// `|d·ml|` within `MIDLINE_TOL`).
///
/// Returns `None` when no vertex passes the midline/radius filter (a degenerate
/// mesh) — callers must **not** silently fall back to `origin` (the interior
/// body centre), which would place an attachment inside the bone.
#[must_use]
pub fn extreme_vertex(
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

/// Sample a vertebra's signed field into an absolute-coordinate [`SdfGrid`].
///
/// [`FACET_CELL`]-mm resolution (identity pose places it anatomically), padded by
/// `4·FACET_CELL`. The grid feeds a `ShapeConcave` facet-contact query in the
/// caller.
// Cell counts are small non-negative integers over an mm-scale padded AABB; the
// f64→usize cast cannot truncate or lose sign in practice.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[must_use]
pub fn facet_grid(mesh: &IndexedMesh, sdf: &MeshOracle) -> Arc<SdfGrid> {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let pad = 4.0 * FACET_CELL;
    let lo = bbox.min - Vector3::repeat(pad);
    let span = (bbox.max + Vector3::repeat(pad)) - lo;
    let cells = |len: f64| (len / FACET_CELL).ceil().max(0.0) as usize + 1;
    Arc::new(SdfGrid::from_fn(
        cells(span.x),
        cells(span.y),
        cells(span.z),
        FACET_CELL,
        lo,
        |p| sdf.evaluate(p),
    ))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)] // tests may unwrap.

    use super::*;

    /// A vertices-only mesh (the coordinate-free helpers read only `vertices`).
    /// A dummy face is added only at ≥3 vertices, so the helper never hands back
    /// an out-of-bounds face index.
    fn vertex_mesh(verts: &[[f64; 3]]) -> IndexedMesh {
        let faces = if verts.len() >= 3 {
            vec![[0, 1, 2]]
        } else {
            vec![]
        };
        IndexedMesh {
            vertices: verts
                .iter()
                .map(|&[x, y, z]| Point3::new(x, y, z))
                .collect(),
            faces,
        }
    }

    /// A watertight axis-aligned box (8 verts, 12 outward-wound triangles) — a
    /// closed surface the signed oracle can sign correctly.
    fn box_mesh(center: Point3<f64>, half: f64) -> IndexedMesh {
        let c = center;
        let vertices = vec![
            Point3::new(c.x - half, c.y - half, c.z - half), // 0
            Point3::new(c.x + half, c.y - half, c.z - half), // 1
            Point3::new(c.x + half, c.y + half, c.z - half), // 2
            Point3::new(c.x - half, c.y + half, c.z - half), // 3
            Point3::new(c.x - half, c.y - half, c.z + half), // 4
            Point3::new(c.x + half, c.y - half, c.z + half), // 5
            Point3::new(c.x + half, c.y + half, c.z + half), // 6
            Point3::new(c.x - half, c.y + half, c.z + half), // 7
        ];
        // Outward-wound (CCW seen from outside) triangles.
        let faces = vec![
            [0, 3, 2],
            [0, 2, 1], // z- (bottom)
            [4, 5, 6],
            [4, 6, 7], // z+ (top)
            [0, 1, 5],
            [0, 5, 4], // y- (front)
            [2, 3, 7],
            [2, 7, 6], // y+ (back)
            [0, 4, 7],
            [0, 7, 3], // x- (left)
            [1, 2, 6],
            [1, 6, 5], // x+ (right)
        ];
        IndexedMesh { vertices, faces }
    }

    #[test]
    fn centroid_is_the_mean_vertex() {
        let m = vertex_mesh(&[[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [0.0, 3.0, 3.0]]);
        assert_eq!(centroid(&m), Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn extreme_vertex_picks_farthest_along_dir() {
        let m = vertex_mesh(&[[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]);
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
        // x=10 rejected by radius (norm 10 > 8), x=20 rejected off-midline
        // (z=20 > MIDLINE_TOL); x=5 wins.
        let m = vertex_mesh(&[[5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 20.0]]);
        let got = extreme_vertex(&m, Point3::origin(), Vector3::x(), Vector3::z(), 8.0);
        assert_eq!(got, Some(Point3::new(5.0, 0.0, 0.0)));
    }

    #[test]
    fn extreme_vertex_opposite_directions_pick_opposite_ends() {
        let m = vertex_mesh(&[[-7.0, 0.0, 0.0], [0.0, 0.0, 0.0], [7.0, 0.0, 0.0]]);
        let ant = extreme_vertex(
            &m,
            Point3::origin(),
            -Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        let post = extreme_vertex(
            &m,
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            f64::INFINITY,
        );
        assert_eq!(ant, Some(Point3::new(-7.0, 0.0, 0.0)));
        assert_eq!(post, Some(Point3::new(7.0, 0.0, 0.0)));
    }

    #[test]
    fn extreme_vertex_none_when_nothing_qualifies() {
        let m = vertex_mesh(&[[5.0, 0.0, 20.0], [10.0, 0.0, 20.0]]);
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
    fn extreme_vertex_midline_filter_is_symmetric() {
        // Negative-ML-side vertex (z=-20) rejected too — guards the `.abs()`.
        let m = vertex_mesh(&[[5.0, 0.0, -20.0], [10.0, 0.0, -20.0]]);
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
    fn oracle_signs_inside_negative_outside_positive() {
        let m = box_mesh(Point3::origin(), 10.0);
        let o = oracle(&m).unwrap();
        assert!(
            o.evaluate(Point3::origin()) < 0.0,
            "centre must be interior"
        );
        assert!(
            o.evaluate(Point3::new(100.0, 0.0, 0.0)) > 0.0,
            "far point must be exterior"
        );
    }

    #[test]
    fn body_center_finds_deep_interior() {
        let m = box_mesh(Point3::origin(), 10.0);
        let o = oracle(&m).unwrap();
        let (c, depth) = body_center(&m, &o);
        assert!(
            depth < MIN_BODY_DEPTH,
            "a 20mm box is thick (depth {depth})"
        );
        assert!(
            c.coords.norm() < 3.0,
            "centre near the box middle, got {c:?}"
        );
    }

    #[test]
    fn facet_grid_covers_the_padded_mesh() {
        let m = box_mesh(Point3::origin(), 10.0);
        let o = oracle(&m).unwrap();
        let grid = facet_grid(&m, &o);
        // 20mm box + 4mm pad each side ≈ 28mm / 1mm cell → ~29 cells per axis.
        assert!(grid.width() > 20 && grid.width() < 40, "w={}", grid.width());
    }

    #[test]
    fn segment_frame_derives_orthonormal_axes() {
        // Two stacked boxes, each with an extra (unreferenced) posterior marker
        // vertex so the vertex-centroid is pulled to -y off the body centre —
        // decoupling centroid from body_center exercises the posterior/ML math.
        let frame = |z: f64| {
            let mut m = box_mesh(Point3::new(0.0, 0.0, z), 10.0);
            m.vertices.push(Point3::new(0.0, -40.0, z)); // posterior marker
            m
        };
        let (l4, l5) = (frame(20.0), frame(0.0));
        let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
        let f = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        assert!(
            (f.superior_axis - Vector3::z()).norm() < 1e-6,
            "superior=+z"
        );
        assert!(f.posterior.dot(&Vector3::y()) < -0.5, "posterior points -y");
        assert!(f.ml.dot(&f.superior_axis).abs() < 1e-9, "ml ⟂ superior");
        assert!(f.ml.dot(&f.posterior).abs() < 1e-9, "ml ⟂ posterior");
    }

    #[test]
    fn segment_frame_bails_on_coincident_bodies() {
        let m = box_mesh(Point3::origin(), 10.0);
        let o = oracle(&m).unwrap();
        let err = segment_frame(&m, &m, &o, &o).unwrap_err();
        assert!(format!("{err}").contains("apart"), "got {err}");
    }

    #[test]
    fn segment_frame_bails_on_thin_body() {
        // A 4mm-half box: interior depth ~ -4 mm, shallower than MIN_BODY_DEPTH.
        let thin = box_mesh(Point3::origin(), 2.0);
        let o = oracle(&thin).unwrap();
        let err = segment_frame(&thin, &thin, &o, &o).unwrap_err();
        assert!(format!("{err}").contains("thick solid mass"), "got {err}");
    }

    #[test]
    fn segment_frame_bails_on_symmetric_posterior() {
        // Symmetric boxes: vertex-centroid == body-centre → no posterior.
        let (l4, l5) = (
            box_mesh(Point3::new(0.0, 0.0, 20.0), 10.0),
            box_mesh(Point3::origin(), 10.0),
        );
        let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
        let err = segment_frame(&l4, &l5, &o4, &o5).unwrap_err();
        assert!(format!("{err}").contains("posterior"), "got {err}");
    }

    #[test]
    fn load_round_trips_and_rejects_empty() {
        let dir = tempfile::tempdir().unwrap();
        let good = dir.path().join("box.stl");
        mesh_io::save_stl(&box_mesh(Point3::origin(), 10.0), &good, false).unwrap();
        let loaded = load(&good).unwrap();
        assert!(
            !loaded.vertices.is_empty(),
            "round-tripped box has vertices"
        );

        let empty = dir.path().join("empty.stl");
        std::fs::write(&empty, "solid empty\nendsolid empty\n").unwrap();
        assert!(load(&empty).is_err(), "empty STL must error");
    }
}
