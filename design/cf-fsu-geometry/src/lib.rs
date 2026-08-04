//! Shared geometry recipes for the lumbar **Functional Spinal Unit** (FSU).
//!
//! These helpers turn a raw vertebra/disc surface mesh into the anatomical
//! quantities an FSU model needs, deriving everything **from the signed
//! field — no coordinate convention on faith**:
//!
//!   * [`load`] / [`load_from_env`] — read + weld-repair an STL (by path, or by
//!     the environment variable holding the path), keeping native coordinates so
//!     sibling vertebrae stay in their shared frame;
//!   * [`load_with_report`] / [`load_from_env_with_report`] — the same, plus a
//!     [`SurfaceReport`] saying what came in: what the weld did, whether the
//!     surface is watertight and manifold, and whether any faces are locally
//!     flipped. Prefer these when the mesh feeds an [`oracle`];
//!   * [`oracle`] — build the exact mesh-derived signed-distance [`MeshOracle`];
//!   * [`body_center`] — locate one vertebral body as the deepest interior point
//!     of the field (no axis on faith);
//!   * [`segment_frame`] — derive the segmental frame (superior / posterior / ML)
//!     from two vertebrae, locating each vertebral body with [`body_center`];
//!   * [`extreme_vertex`] — pick a near-midline attachment vertex along a
//!     direction (ligament sites);
//!   * [`facet_grid`] — sample the field into an [`SdfGrid`] for facet contact.
//!
//! The recipes were proven in the geometry-fidelity ladder and are the single
//! home consumed by the `cf-spine-studio` tool and the rung 4b / 5 / 6c / 7
//! integration tests. The crate is deliberately **sim-core-free**: [`facet_grid`]
//! builds a `cf_geometry::SdfGrid` directly, and the `ShapeConcave` collision
//! that consumes the grid stays in each caller.

use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use cf_design::Solid;
use cf_geometry::{Aabb, IndexedMesh, Sdf, SdfGrid};
use mesh_io::load_stl;
use mesh_repair::{
    RepairParams, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU, find_connected_components,
    keep_largest_component, repair_mesh, taubin_smooth_vertices, validate_mesh, winding_census,
};

// The three instrument types that make up a [`SurfaceReport`]. Re-exported so a
// consumer can NAME them — write a signature, a typed binding, or a struct
// field — without taking a direct `mesh-repair` dependency of its own.
pub use mesh_repair::{MeshReport, RepairSummary, WindingCensus};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Matrix3, Point3, Vector3};

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

/// What [`load_with_report`] found on the way in — the hand-off from a file to
/// a surface, measured instead of assumed.
///
/// [`load_with_report`] welds a raw STL and hands the result to [`oracle`],
/// which composes `PseudoNormalSign`. That sign is computed from face winding,
/// so whether it can be trusted is a property of the surface that came out of
/// the weld. This report carries the readings that bear on it, and **judges
/// none of them** — see "Reading this" below.
///
/// | field | answers |
/// |---|---|
/// | `repair` | how much the weld changed (was this file already a mesh, or triangle soup?) |
/// | `topology` | watertight? manifold? and `is_inside_out`, a global signed-volume test |
/// | `winding` | are any faces **locally** flipped — the question signed volume cannot ask |
/// | `components` / `inward_facing_components` | are there disjoint shells, and do they agree on which way is out? |
///
/// # Reading this
///
/// The winding instruments are orthogonal and none subsumes another.
///
/// * `winding.inconsistent_edges` is per-edge and **coordinate-free**. It is
///   the only one of the three that answers "are two neighbouring faces
///   oriented against each other".
/// * `topology.is_inside_out` sums origin-apex tetrahedra. ⚠ **That sum is
///   translation-invariant only while the surface is consistently oriented.**
///   Once any face is flipped the flag depends on the mesh's distance from the
///   world origin — and [`load`] deliberately keeps native anatomical
///   coordinates, which are ~1e3 mm out. So on this crate's own inputs a `true`
///   here may mean "globally reversed" **or** "locally flipped and far from the
///   origin". Do not read it as the former. (Producer: `mesh-repair`'s
///   `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.)
/// * `inward_facing_components` compares each connected shell against its own
///   volume, so it catches the case both of the above miss: two disjoint closed
///   shells wound oppositely. Neither a per-edge census (the shells share no
///   edge) nor a global volume (the larger shell dominates) can see that.
///
/// # What this still does not answer
///
/// Every reading here is combinatorial or volumetric. None of them sees
/// self-intersection, or near-degenerate slivers whose pseudo-normals vanish —
/// for that consequence use `mesh_sdf::TriMeshDistance::health()` on the oracle
/// you actually build. A clean `SurfaceReport` is **necessary, not sufficient**,
/// for a surface to be safe to sign.
///
/// ⚠ **Order matters, and this report is taken after the weld.** An STL stores
/// three vertex positions per facet with no shared indices, so a raw STL is
/// triangle soup and no two triangles share an edge by index. A winding census
/// on unwelded input has nothing to judge and reports zero inconsistencies for
/// any input whatsoever; `winding.has_judgeable_edges()` is what distinguishes
/// that from a clean bill.
#[derive(Debug, Clone)]
pub struct SurfaceReport {
    /// What the weld itself did — vertices merged, degenerates and duplicates
    /// dropped, before/after counts.
    pub repair: RepairSummary,
    /// Watertightness, manifoldness, and the global signed-volume orientation
    /// flag. ⚠ See "Reading this" before trusting `is_inside_out`.
    pub topology: MeshReport,
    /// Per-edge local orientation consistency.
    pub winding: WindingCensus,
    /// Number of connected components (disjoint shells).
    pub components: usize,
    /// How many components enclose a **negative** signed volume — i.e. are
    /// wound inward while their neighbours are not.
    ///
    /// `None` when the surface is not watertight: per-component signed volume
    /// is only translation-invariant, and therefore only meaningful, for a
    /// closed component. Reporting a number there would be inventing one.
    pub inward_facing_components: Option<usize>,
}

impl std::fmt::Display for SurfaceReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} verts {} faces (welded {}, dropped {} degenerate / {} duplicate) | \
             watertight {} manifold {} inside-out {} | components {} inward-facing {} | {}",
            self.topology.vertex_count,
            self.topology.face_count,
            self.repair.vertices_welded,
            self.repair.degenerates_removed,
            self.repair.duplicates_removed,
            self.topology.is_watertight,
            self.topology.is_manifold,
            self.topology.is_inside_out,
            self.components,
            self.inward_facing_components
                .map_or_else(|| "n/a (open surface)".to_string(), |n| n.to_string()),
            self.winding,
        )
    }
}

/// Load + weld-repair a vertebra/disc STL at `path`, **keeping native
/// coordinates** (do not recenter — sibling vertebrae must stay in their shared
/// anatomical frame so they stack).
///
/// Equivalent to [`load_with_report`] with the report dropped. Use that one
/// when you want to know what came in — this crate's oracle is only as
/// trustworthy as the surface it is built on.
///
/// Note that the report is computed either way: this function pays for a
/// `validate_mesh` and a `winding_census` over the welded surface and then
/// discards them. On a ~15k-face vertebra that is a couple of edge-map builds,
/// small beside the parry BVH every caller constructs next, but it is not
/// nothing — and it buys this caller nothing.
///
/// # Errors
/// Returns an error if the STL cannot be read, or if the mesh has no vertices
/// or no faces after repair (empty, unreadable, or fully-degenerate) — either
/// would otherwise yield a meaningless field or a `NaN` bounding-box centre
/// downstream.
pub fn load(path: &Path) -> Result<IndexedMesh> {
    load_with_report(path).map(|(mesh, _)| mesh)
}

/// [`load`], plus the [`SurfaceReport`] describing what came in.
///
/// The repair statistics used to be discarded here — `let _stats = …`, on the
/// stated grounds that they were diagnostic only. They are diagnostic, and the
/// diagnosis is the point: without it, a caller has no way to learn that the
/// surface it is about to build a signed-distance oracle over is open,
/// non-manifold, or **locally flipped** — a defect distinct from the
/// whole-mesh `is_inside_out` flag, and one that flag cannot reliably see.
///
/// **Report-only.** This function returns exactly the mesh [`load`] returns and
/// fails in exactly the same cases; a non-clean report is never itself an
/// error. Deciding what an unhealthy surface means is the caller's call, and
/// making it fatal here without a census of what real inputs actually look
/// like would be a guess dressed as a guarantee.
///
/// # Errors
/// Identical to [`load`].
pub fn load_with_report(path: &Path) -> Result<(IndexedMesh, SurfaceReport)> {
    let mut mesh = load_stl(path).with_context(|| format!("loading STL {}", path.display()))?;
    let repair = repair_mesh(&mut mesh, &RepairParams::for_scans());
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
    // Measured on the WELDED surface, which is both the surface `oracle` will
    // see and the only stage at which a winding census means anything.
    let topology = validate_mesh(&mesh);
    let parts = find_connected_components(&mesh);
    let report = SurfaceReport {
        repair,
        // Per-component signed volume is translation-invariant only for a
        // CLOSED component, so it is computed only when the whole surface is
        // watertight — which is exactly when every component is closed.
        inward_facing_components: topology.is_watertight.then(|| {
            component_signed_volumes(&mesh, &parts)
                .filter(|v| *v < 0.0)
                .count()
        }),
        components: parts.component_count,
        topology,
        winding: winding_census(&mesh),
    };
    Ok((mesh, report))
}

/// Signed volume of each connected component, via the divergence theorem.
///
/// Only meaningful per-component when that component is closed; the sole
/// caller gates on watertightness for that reason.
fn component_signed_volumes<'a>(
    mesh: &'a IndexedMesh,
    parts: &'a mesh_repair::ComponentAnalysis,
) -> impl Iterator<Item = f64> + 'a {
    parts.components.iter().map(move |faces| {
        faces
            .iter()
            .map(|&f| {
                let [i0, i1, i2] = mesh.faces[f as usize];
                let (a, b, c) = (
                    mesh.vertices[i0 as usize].coords,
                    mesh.vertices[i1 as usize].coords,
                    mesh.vertices[i2 as usize].coords,
                );
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum::<f64>()
    })
}

/// [`load_from_env`], plus the [`SurfaceReport`] describing what came in.
///
/// This is the reporting entry point for the licence-gated FSU consumers, which
/// key their mesh paths off environment variables rather than literal paths —
/// without it they would have to re-implement the lookup to reach a report.
///
/// # Errors
/// Returns an error if `var` is unset, or if [`load_with_report`] rejects the
/// file it names.
pub fn load_from_env_with_report(var: &str) -> Result<(IndexedMesh, SurfaceReport)> {
    let path = std::env::var(var)
        .with_context(|| format!("environment variable ${var} is not set (path to an STL)"))?;
    load_with_report(Path::new(&path))
}

/// Load + weld-repair the STL whose path is held in environment variable `var`.
///
/// The license-gated ladder tests key the `BodyParts3D` mesh paths off
/// `$CF_L4_STL` / `$CF_L5_STL` / `$CF_DISC_STL` (the meshes are CC BY-SA and not
/// committed); this is the thin env-lookup convenience over [`load`] they share.
///
/// **Where to get the meshes:** `BODYPARTS3D.md` at this crate's root is the canonical
/// provenance for every workspace consumer — the source repo pinned to a commit, each
/// mesh's FMA id / env var / SHA-256, and a fetch-and-verify recipe.
///
/// # Errors
/// Returns an error if `var` is unset, or if [`load`] rejects the file it names.
pub fn load_from_env(var: &str) -> Result<IndexedMesh> {
    let path = std::env::var(var)
        .with_context(|| format!("environment variable ${var} is not set (path to an STL)"))?;
    load(Path::new(&path))
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

/// Vertebral body centre = the deepest interior point of the signed field.
///
/// The thickest solid mass, no axis assumption. Coarse `BODY_CENTER_STEP`-mm AABB
/// scan; returns `(point, depth)` where `depth < 0` is interior. Used by
/// [`segment_frame`], and directly by callers that locate a single vertebral body
/// without building the full segmental frame (rung 4b facet contact, rung 5
/// ligament tension).
// Grid indices/counts are small non-negative integers over an mm-scale AABB;
// the f64↔usize casts cannot truncate, lose sign, or lose precision in practice.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
#[must_use]
pub fn body_center(mesh: &IndexedMesh, sdf: &MeshOracle) -> (Point3<f64>, f64) {
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

/// Maximum distance a vertex may be projected onto a bone (mm).
///
/// Acts as the thickness of the near-endplate layer that conforms: a disc vertex within this
/// of L4 or L5 seats on that endplate, everything deeper (the disc interior / annulus middle)
/// is left free. Comfortably spans the observed disc↔vertebra registration gap plus the full
/// endplate-face relief (≤ ~4 mm on the `BodyParts3D` L4/L5 disc), yet stays well inside the
/// ~9 mm disc half-thickness so the two endplate layers never meet.
pub const SI_CONFORM_CAP: f64 = 4.0;
/// Minimum |move·SI| / |move| to accept a projection (else the vertex is left in place). A
/// projection onto an endplate FACE moves the vertex roughly along the SI axis (endplate
/// normal ≈ SI), so its alignment is near 1; a projection onto the vertebra's LATERAL WALL
/// moves nearly perpendicular to SI (alignment near 0). This `cos 70°` threshold seats the
/// faces (which the disc must render flush on) while declining an annulus vertex that would
/// wrap sideways onto the body wall. Only applied to moves larger than
/// [`SI_CONFORM_ON_SURFACE`] — a vertex already on the bone needs no direction.
const SI_CONFORM_MIN_ALIGN: f64 = 0.34;
/// A vertex already within this distance of the bone (mm) is left as-is (already seated; the
/// direction guard does not apply). Sub-10-µm — the disc's median native endplate gap.
const SI_CONFORM_ON_SURFACE: f64 = 1.0e-3;
/// Move cap (mm) for the **bonded-band node** conform ([`bonded_conform_target`]).
///
/// Looser than the render-surface [`SI_CONFORM_CAP`]: the bonded FEM nodes float a measured
/// p90 ≈ 5.8 mm off the bone (a longer registration gap than the drawn face, which is caught at
/// its own extremes), so a hard 3–4 mm cap would wrongly decline genuine endplate-facing nodes
/// 3–5 mm out. The
/// SI-alignment guard — not this distance — is the primary discriminator between a face node
/// (seat it) and an annular-rim node reaching sideways for the body wall (leave it straight); the
/// cap is only a loose backstop against a runaway projection. Still well inside the ~9 mm disc
/// half-thickness, so the two endplate bands never meet.
pub const SI_CONFORM_CAP_BONDED: f64 = 6.0;

/// Conform the disc's endplate-facing vertices onto the **real** L4/L5 endplate surfaces
/// (native mm), so the rendered disc *is* the contact geometry — no proxy gap.
///
/// The intervertebral disc and its neighbouring vertebrae are independent `BodyParts3D`
/// segmentations that do not perfectly abut: the disc's top/bottom faces float a fraction of
/// a millimetre off — or slightly into — the real bone, which shows as a disc-lift seam. For
/// every disc vertex this projects onto the **nearer** of the two endplates
/// ([`MeshOracle::closest_point`], which follows the local surface normal so it seats a face
/// even where it tilts or curves away from the global SI axis), subject to two guards keyed
/// off the SI axis ([`SegmentFrame::superior_axis`]):
///
/// - a [`SI_CONFORM_CAP`] cap on the move — this both leaves the disc interior/annulus middle
///   (far from either bone) untouched and defines the *thickness* of the seated endplate
///   layer, so the WHOLE top and bottom face seats, not a fixed slab;
/// - a `SI_CONFORM_MIN_ALIGN` direction guard — a vertex whose nearest surface point is
///   reached by a near-lateral move (the vertebra's side WALL, not its endplate) is left in
///   place rather than wrapped sideways onto the body.
///
/// Picking the nearer endplate per vertex needs no SI-orientation convention; the guards
/// alone decide what is a face (seat it) versus interior/annulus (leave it). Purely geometric:
/// the returned mesh keeps the same topology (faces); only endplate-facing vertex positions
/// move.
///
/// `restrict_band` chooses the caller's two uses, which deliberately differ:
/// - `None` — conform the **whole** top and bottom face. This is the RENDER surface: the
///   drawn disc must seat flush along its entire endplate (rendered === contacts).
/// - `Some(band_frac)` — conform only the top/bottom `band_frac` of the disc's native-z
///   extent. This is the BONDED disc: seating the full FEM face over-constrains and stiffens
///   the disc (its measured `k_disc` shifts ~6×), so the bond seats just the endplate band it
///   actually ties to (a subset the whole-face render then extends). Pass the disc `band_frac`
///   `cf_fsu_model::build_bonded_disc` bonds, so the two agree on the band.
#[must_use]
pub fn conform_disc_to_endplates(
    disc: &IndexedMesh,
    o4: &MeshOracle,
    o5: &MeshOracle,
    frame: &SegmentFrame,
    restrict_band: Option<f64>,
) -> IndexedMesh {
    let si = frame.superior_axis; // unit SI axis — the endplate-normal direction for the guard
    // Optional native-z band cut: (low_cut, high_cut) — only z ≤ low_cut or z ≥ high_cut conform.
    let band = restrict_band.map(|bf| {
        let bbox = Aabb::from_points(disc.vertices.iter());
        let (lo, hi) = (bbox.min.z, bbox.max.z);
        let b = bf * (hi - lo);
        (lo + b, hi - b)
    });
    let mut out = disc.clone();
    for (i, &v) in disc.vertices.iter().enumerate() {
        if let Some((low_cut, high_cut)) = band {
            if v.z > low_cut && v.z < high_cut {
                continue; // interior (not in the endplate band) — bond use skips it
            }
        }
        if let Some(p) = project_to_nearest_endplate(v, si, o4, o5) {
            out.vertices[i] = p;
        }
    }
    out
}

/// Project `v` onto its nearest point on whichever endplate (`o4` = L4 or `o5` = L5) is
/// closer, following the LOCAL surface normal (`closest_point`), subject to the
/// [`SI_CONFORM_CAP`] move cap and the [`SI_CONFORM_MIN_ALIGN`] direction guard (`si` = the
/// SI axis). Returns the surface point, or `None` — leaving the vertex in place — when the
/// nearest endplate is past the cap (the disc interior / annulus middle) or is reached by a
/// near-lateral move onto a side wall rather than an endplate face.
fn project_to_nearest_endplate(
    v: Point3<f64>,
    si: Vector3<f64>,
    o4: &MeshOracle,
    o5: &MeshOracle,
) -> Option<Point3<f64>> {
    conform_target(v, si, o4, o5, SI_CONFORM_CAP, SI_CONFORM_MIN_ALIGN)
}

/// The conform target for a single **bonded-band node**: the point on the nearer real endplate
/// this node should seat on, or `None` to leave it straight.
///
/// Same discriminator as the render-surface [`conform_disc_to_endplates`] (nearest of the two
/// endplates, SI-alignment direction guard) but with the looser [`SI_CONFORM_CAP_BONDED`]
/// backstop — the FEM nodes sit a larger registration gap off the bone than the drawn face does.
///
/// This is the node-level primitive `cf_fsu_model::build_bonded_disc` uses to bond the disc to the
/// exact vertebral endplate (Strategy B: mesh the raw disc, then project its bonded-band nodes),
/// so the physics runs on the real bone rather than a floating raw disc. `v` and the returned
/// point are in the caller's frame; `si` is the (unit) SI axis ([`SegmentFrame::superior_axis`]),
/// invariant under the disc's translate + uniform-scale solver bridge so the alignment guard holds
/// in either frame. The caller is responsible for keeping the resulting move mesh-valid (a
/// quality-floor back-off); leaving the overhanging annular rim straight is intentional — the
/// outer annulus attaches to the ring apophysis, not the endplate face.
#[must_use]
pub fn bonded_conform_target(
    v: Point3<f64>,
    si: Vector3<f64>,
    o4: &MeshOracle,
    o5: &MeshOracle,
) -> Option<Point3<f64>> {
    conform_target(v, si, o4, o5, SI_CONFORM_CAP_BONDED, SI_CONFORM_MIN_ALIGN)
}

/// Shared core of [`project_to_nearest_endplate`] (render surface, [`SI_CONFORM_CAP`]) and
/// [`bonded_conform_target`] (bonded band, [`SI_CONFORM_CAP_BONDED`]): project `v` onto its
/// nearest point on whichever endplate (`o4` = L4 or `o5` = L5) is closer, following the LOCAL
/// surface normal ([`MeshOracle::closest_point`]), subject to a `cap` on the move distance and a
/// `min_align` SI-direction guard (`si` = the SI axis). Returns the surface point, or `None` —
/// leaving the vertex in place — when the nearest endplate is past the cap (the disc interior /
/// annulus middle) or is reached by a near-lateral move onto a side wall rather than an endplate
/// face.
fn conform_target(
    v: Point3<f64>,
    si: Vector3<f64>,
    o4: &MeshOracle,
    o5: &MeshOracle,
    cap: f64,
    min_align: f64,
) -> Option<Point3<f64>> {
    // Nearer of the two endplates — no band / SI-orientation convention needed.
    let (p4, p5) = (o4.closest_point(v), o5.closest_point(v));
    let p = if (p4 - v).norm_squared() <= (p5 - v).norm_squared() {
        p4
    } else {
        p5
    };
    let d = p - v;
    let dist = d.norm();
    if dist < SI_CONFORM_ON_SURFACE {
        return Some(p); // already on the surface — no meaningful direction to guard
    }
    if dist > cap {
        return None; // nearest endplate is too far — disc interior / annulus middle
    }
    // Endplate FACE ⇒ move ≈ along SI (alignment ~1); side WALL ⇒ move ≈ ⟂ SI (alignment ~0).
    if (d.dot(&si) / dist).abs() < min_align {
        return None; // near-lateral move — would wrap onto the body wall, decline
    }
    Some(p)
}

/// The disc's transverse cross-section at `z_mid`, extruded infinitely along the SI (native z)
/// axis and optionally eroded inward by `erode` mm — an `Sdf` giving the disc's own footprint as
/// a prism. Used as the lateral bound in [`model_disc_between_endplates`]; the endplate carve
/// trims its top and bottom, so only its footprint (not its SI extent) is meaningful.
struct DiscFootprintPrism {
    disc: MeshOracle,
    z_mid: f64,
    erode: f64,
}

impl Sdf for DiscFootprintPrism {
    fn eval(&self, p: Point3<f64>) -> f64 {
        // Ignore the SI coordinate → an infinite prism of the mid-plane section; `+erode` shrinks
        // the footprint inward (a positive shift of the signed distance).
        self.disc.evaluate(Point3::new(p.x, p.y, self.z_mid)) + self.erode
    }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let g = self.disc.grad(Point3::new(p.x, p.y, self.z_mid));
        Vector3::new(g.x, g.y, 0.0) // a prism has no SI gradient
    }
    fn hessian(&self, _p: Point3<f64>) -> Matrix3<f64> {
        Matrix3::zeros()
    }
}

/// Parameters for [`model_disc_between_endplates`]. [`Default`] is tuned for the `BodyParts3D`
/// L4–L5 disc (native mm).
#[derive(Debug, Clone, Copy)]
pub struct DiscModelParams {
    /// Surface-mesh tolerance (native mm) — the lattice cell the carved solid is dual-contoured
    /// at; smaller resolves the endplate curvature more finely (at more triangles + compute).
    pub cell: f64,
    /// How far (mm) the SI bound pokes past the disc's own SI extent into each vertebra, so the
    /// endplate carve reliably reaches L4/L5 even where the scanned disc under-fills the gap
    /// (measured: the disc's superior face floats ~2.5 mm below L4 — the margin must exceed that).
    pub si_margin: f64,
    /// Erode the disc footprint inward (mm) so its rim stays within the vertebral body, where the
    /// endplates carve it cleanly; `0.0` keeps the scanned silhouette. A small value tidies the
    /// annulus rim (the carve leaves a thin sliver where the footprint overhangs the body).
    pub footprint_erode: f64,
    /// Taubin (shrink-free) smoothing passes over the carved surface — removes the high-frequency
    /// dual-contouring crazing the vertebra oracle's finite-difference gradient induces on the
    /// endplate faces, without shrinking the disc off the endplates. `0` leaves the raw carve.
    pub smooth_iters: usize,
    /// Trim the disc `Some(mm)` behind the vertebral-body midpoint along the segment frame's
    /// posterior axis — the scanned footprint reaches back into the spinal canal, where only one
    /// endplate is present and the carve leaves a thin posterior flap; this cuts the disc to the
    /// body region. `None` skips the trim (and the segment-frame computation) — used where the
    /// posterior axis is undefined (e.g. symmetric test geometry).
    pub posterior_trim: Option<f64>,
}

impl Default for DiscModelParams {
    fn default() -> Self {
        Self {
            cell: 0.8,
            si_margin: 4.0,
            footprint_erode: 1.0,
            smooth_iters: 8,
            posterior_trim: Some(14.0),
        }
    }
}

/// Model the intervertebral disc as the solid that **fills the gap between the two vertebral
/// endplates**.
///
/// It is carved from the disc's own footprint by subtracting the real vertebrae, so its top face
/// **is** L4's inferior endplate and its bottom face **is** L5's superior endplate, by construction.
///
/// This replaces using an independent disc scan as the disc geometry. The `BodyParts3D` disc,
/// vertebrae, and their interfaces are separate segmentations that do not abut: the scanned disc's
/// faces float off the endplates (its superior band sits ~2.5 mm below L4), so bonding it to the
/// real bone over-stretches it. The **modeled** disc fits both endplates exactly — zero seam,
/// rendered === physics — and, being one connected solid carved from a clean prism, does not
/// fragment the way the thin scanned rim does.
///
/// Construction: `disc = (disc footprint prism ∩ SI slab) − L4 − L5`. The prism is the disc's
/// mid-plane cross-section extruded along SI (`DiscFootprintPrism`); the slab clamps its height
/// to reach `si_margin` into each vertebra; subtracting the two vertebra oracles carves the top
/// and bottom into the real curved endplates. The result is polygonised with `cf-design`'s
/// dual-contouring mesher (which keeps the sharp annulus edge and avoids marching-cubes aliasing)
/// and reduced to its largest connected component.
///
/// `l4` is the SUPERIOR vertebra (above the disc, +z), `l5` the INFERIOR; `disc` is the scanned
/// disc, used ONLY for its lateral footprint + SI extent (its ill-fitting faces are discarded by
/// the carve). All meshes are native millimetres; SI must be native z (the disc's thinnest extent
/// — the shared FSU convention, matching `build_bonded_disc`).
///
/// # Errors
/// Returns an error if the disc's thinnest extent is not native z (a mis-oriented mesh), if a
/// vertebra/disc oracle fails to build, or if the carve yields an empty solid (L4/L5 swapped, or
/// `si_margin` too small to reach the endplates).
pub fn model_disc_between_endplates(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    disc: &IndexedMesh,
    params: &DiscModelParams,
) -> Result<IndexedMesh> {
    let bbox = Aabb::from_points(disc.vertices.iter());
    let center = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
    let ext = bbox.max - bbox.min;
    if !(ext.z < ext.x && ext.z < ext.y) {
        bail!(
            "disc SI extent (thinnest) must be native z; got extents ({:.2}, {:.2}, {:.2}) mm — mis-oriented mesh?",
            ext.x,
            ext.y,
            ext.z
        );
    }

    // A domain generously covering the disc plus the endplate patches of both vertebrae (the
    // `from_sdf` evaluation bound; the carved solid's own bounds — set by the slab below — keep
    // the mesher tight regardless).
    let pad = Vector3::new(
        ext.x * 0.5 + 25.0,
        ext.y * 0.5 + 25.0,
        ext.z * 0.5 + params.si_margin + 25.0,
    );
    let bounds = Aabb::from_points([center - pad, center + pad].iter());

    let o4 = oracle(l4).context("L4 oracle")?;
    let o5 = oracle(l5).context("L5 oracle")?;

    // Lateral bound: the disc's own footprint (its mid-plane section extruded along SI, eroded),
    // clamped to a slab that reaches `si_margin` into each vertebra so the carve connects.
    let prism = Solid::from_sdf(
        DiscFootprintPrism {
            disc: oracle(disc).context("disc oracle")?,
            z_mid: center.z,
            erode: params.footprint_erode,
        },
        bounds,
    );
    let half_z = ext.z * 0.5 + params.si_margin;
    let slab = Solid::cuboid(Vector3::new(ext.x, ext.y, half_z)).translate(center.coords);
    let mut lateral = prism.intersect(slab);

    // Optionally cut the posterior canal flap: keep only the half-space anterior of the body
    // midpoint + `trim` mm along the frame's posterior axis.
    if let Some(trim) = params.posterior_trim {
        let frame = segment_frame(l4, l5, &o4, &o5).context("segment frame for posterior trim")?;
        let body_mid = (frame.b4.coords + frame.b5.coords) * 0.5;
        let threshold = frame.posterior.dot(&body_mid) + trim;
        lateral = lateral.intersect(Solid::plane(frame.posterior, threshold));
    }

    // Carve the two vertebrae out of the footprint column → the disc between the endplates.
    let disc_solid = lateral
        .subtract(Solid::from_sdf(o4, bounds))
        .subtract(Solid::from_sdf(o5, bounds));

    let mut mesh = disc_solid.mesh_dc(params.cell).geometry;
    if mesh.is_empty() {
        bail!(
            "modeled disc is empty — check L4/L5 orientation (l4 must be superior/+z) or increase si_margin"
        );
    }
    // Drop any isolated fragments the carve leaves (small pockets sealed off between the endplates)
    // so the disc is a single connected solid, then Taubin-smooth the endplate crazing.
    keep_largest_component(&mut mesh);
    taubin_smooth_vertices(
        &mut mesh,
        params.smooth_iters,
        TAUBIN_DEFAULT_LAMBDA,
        TAUBIN_DEFAULT_MU,
    );
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    // tests may unwrap; the small-count → f64 casts (vertex tallies) are exact.
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

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

    /// ★ The wiring gate for hand-off 1: a locally flipped face survives the
    /// whole load path and is **visible in the report**, where the global
    /// signed-volume flag the crate previously had no reason to consult reads
    /// the surface as fine.
    ///
    /// This runs licence-free and therefore in CI, unlike the real-anatomy
    /// census in `tests/winding_census_on_the_real_fsu_meshes.rs`. It is what
    /// actually protects the wiring: `load` discarding its report again, or
    /// the report being taken before the weld, both surface here.
    ///
    /// The flip is injected into the mesh *before* the STL round-trip so the
    /// defect travels the production path — save, read back as soup, weld —
    /// rather than being handed to the census directly.
    #[test]
    fn load_with_report_sees_a_local_flip_that_signed_volume_misses() {
        let dir = tempfile::tempdir().unwrap();

        // Control: the same box, untouched.
        let clean_path = dir.path().join("clean.stl");
        let clean = box_mesh(Point3::origin(), 10.0);
        mesh_io::save_stl(&clean, &clean_path, false).unwrap();
        let (_, clean_report) = load_with_report(&clean_path).unwrap();
        assert_eq!(
            clean_report.winding.inconsistent_edges, 0,
            "the control box must be locally consistent, or the flipped arm \
             below proves nothing",
        );
        assert!(
            clean_report.winding.has_judgeable_edges(),
            "welded, so judgeable"
        );

        // Inject one flipped face, and assert the injection landed.
        let mut flipped = clean;
        let before = flipped.faces[0];
        flipped.faces[0].swap(1, 2);
        assert_ne!(flipped.faces[0], before, "the flip must change face 0");

        let flipped_path = dir.path().join("flipped.stl");
        mesh_io::save_stl(&flipped, &flipped_path, false).unwrap();
        let (_, report) = load_with_report(&flipped_path).unwrap();

        assert_eq!(
            report.winding.inconsistent_edges, 3,
            "one flipped triangle disagrees with its neighbours on its own \
             three edges, and that must survive save -> load -> weld",
        );
        assert!(report.winding.has_inconsistent_winding());

        // THE POINT: the instrument this crate could already have consulted
        // reads both surfaces identically. Wiring up `is_inside_out` alone
        // would have closed the hand-off in name only.
        assert_eq!(
            report.topology.is_inside_out, clean_report.topology.is_inside_out,
            "signed volume is global and must not distinguish these two \
             surfaces — if it starts to, this gate is no longer evidence that \
             the local census was needed",
        );
    }

    /// ★★ The blind spot an adversarial review found, and the reading added to
    /// close it: two disjoint closed shells wound oppositely.
    ///
    /// Every instrument that existed before — watertight, manifold,
    /// `is_inside_out`, the per-edge census, `count_inconsistent_faces` — gives
    /// this mesh a clean bill of health, while `PseudoNormalSign` reports the
    /// space outside the inverted shell as solid.
    #[test]
    fn two_shells_one_inverted_are_caught_only_by_the_component_reading() {
        // A big outward shell plus a small shell wound entirely inward — the
        // canonical scan artifact (a stray fragment emitted with the opposite
        // convention). `load` does not call `keep_largest_component`, so a
        // fragment like this reaches `oracle` intact.
        let mut mesh = box_mesh(Point3::origin(), 10.0);
        let small = box_mesh(Point3::new(100.0, 0.0, 0.0), 1.0);
        let base = u32::try_from(mesh.vertices.len()).unwrap();
        mesh.vertices.extend(small.vertices.iter().copied());
        for f in &small.faces {
            mesh.faces.push([f[0] + base, f[2] + base, f[1] + base]); // reversed
        }

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("two_shells.stl");
        mesh_io::save_stl(&mesh, &path, false).unwrap();
        let (_, report) = load_with_report(&path).unwrap();

        // EVERY other instrument reads clean. This is the point of the gate:
        // an adversarial review produced this mesh precisely because the
        // per-edge census cannot see it — the two shells share no edge — and
        // the global volume cannot either, since the big shell dominates.
        assert_eq!(
            report.winding.inconsistent_edges, 0,
            "the per-edge census cannot compare faces that share no edge",
        );
        assert!(report.winding.has_judgeable_edges());
        assert!(report.topology.is_watertight && report.topology.is_manifold);
        assert!(
            !report.topology.is_inside_out,
            "the larger shell dominates the signed volume, so the global flag \
             reads clean",
        );

        // Only the component reading catches it.
        assert_eq!(report.components, 2);
        assert_eq!(
            report.inward_facing_components,
            Some(1),
            "exactly one shell encloses a negative volume: {report}",
        );
    }

    /// The component reading must report `None`, not a number, when the
    /// surface is open — per-component signed volume is only translation-
    /// invariant for a closed component, so any value there would be invented.
    #[test]
    fn the_component_reading_declines_to_answer_on_an_open_surface() {
        let mut mesh = box_mesh(Point3::origin(), 10.0);
        mesh.faces.truncate(mesh.faces.len() - 2); // remove one face pair -> hole

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("open.stl");
        mesh_io::save_stl(&mesh, &path, false).unwrap();
        let (_, report) = load_with_report(&path).unwrap();

        assert!(!report.topology.is_watertight, "fixture must be open");
        assert_eq!(
            report.inward_facing_components, None,
            "an undefined quantity must be absent, not zero: {report}",
        );
        // And the Display renders that absence on a passing run, which is the
        // only place this impl gets exercised.
        assert!(
            format!("{report}").contains("inward-facing n/a (open surface)"),
            "{report}",
        );
    }

    /// A raw STL is triangle soup, so the census must be taken **after** the
    /// weld or it judges nothing. Pins the ordering inside `load_with_report`
    /// on the same synthetic path as the gate above.
    #[test]
    fn the_surface_report_is_measured_after_the_weld_not_before() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("box.stl");
        mesh_io::save_stl(&box_mesh(Point3::origin(), 10.0), &path, false).unwrap();

        // What the census would have said if taken at the raw stage.
        let raw = mesh_io::load_stl(&path).unwrap();
        let raw_census = winding_census(&raw);
        assert!(
            !raw_census.has_judgeable_edges(),
            "raw STL is soup: {} interior edges",
            raw_census.interior_edges,
        );

        let (mesh, report) = load_with_report(&path).unwrap();
        assert!(
            report.winding.has_judgeable_edges(),
            "the report must be taken after the weld, where edges exist to judge",
        );
        assert!(
            report.repair.vertices_welded > 0,
            "and the weld must be the thing that created them",
        );

        // Every field must describe THE RETURNED SURFACE. Without this the
        // report could be measured on some other mesh entirely and the gates
        // above would still pass — a must-fail pass caught exactly that, by
        // swapping in an empty mesh and breaking nothing.
        assert_eq!(
            (report.topology.vertex_count, report.topology.face_count),
            (mesh.vertices.len(), mesh.faces.len()),
            "the topology report must be of the mesh that was returned",
        );
        assert_eq!(
            report.repair.final_faces,
            mesh.faces.len(),
            "and so must the repair summary",
        );
        assert!(
            report.topology.is_watertight && report.topology.is_manifold,
            "a welded box is closed and manifold — {report}",
        );
        assert_eq!(
            report.winding.interior_edges,
            mesh.faces.len() * 3 / 2,
            "a closed triangle mesh has 3F/2 edges, all interior",
        );
    }

    #[test]
    fn load_from_env_errors_when_var_unset() {
        // A variable name guaranteed unset in the test environment: the env
        // lookup fails before any file I/O. (The happy path delegates to `load`,
        // exercised by `load_round_trips_and_rejects_empty`; setting a process
        // env var here would be `unsafe` on edition 2024 and racy under the
        // parallel test runner.)
        let err = load_from_env("CF_FSU_GEOMETRY_DEFINITELY_UNSET_VAR").unwrap_err();
        assert!(format!("{err}").contains("is not set"), "got {err}");
    }

    /// A two-box synthetic FSU + a gapped disc for the conform recipe: L5 box spans
    /// z∈[−10,10] (top surface z=+10), L4 box spans z∈[20,40] (bottom surface z=+20),
    /// and the disc's endplate bands float 2 mm short of each — top verts at z=18, bottom
    /// at z=12 — with a mid row at z=15 that must stay put. SI = +z. Returns
    /// `(disc, o4, o5, frame)`.
    fn conform_scene() -> (IndexedMesh, MeshOracle, MeshOracle, SegmentFrame) {
        let o4 = oracle(&box_mesh(Point3::new(0.0, 0.0, 30.0), 10.0)).unwrap();
        let o5 = oracle(&box_mesh(Point3::new(0.0, 0.0, 0.0), 10.0)).unwrap();
        let frame = SegmentFrame {
            b4: Point3::new(0.0, 0.0, 30.0),
            b5: Point3::new(0.0, 0.0, 0.0),
            superior_axis: Vector3::z(),
            posterior: Vector3::y(),
            ml: Vector3::x(),
        };
        // Disc vertices: 4 top (z=18, gap to L4@20), 4 mid (z=15, interior), 4 bottom
        // (z=12, gap to L5@10). Faces are irrelevant to the conform (vertex-only), so none.
        let mut vertices = Vec::new();
        for &z in &[18.0_f64, 15.0, 12.0] {
            for &(x, y) in &[(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)] {
                vertices.push(Point3::new(x, y, z));
            }
        }
        let disc = IndexedMesh {
            vertices,
            faces: vec![],
        };
        (disc, o4, o5, frame)
    }

    #[test]
    fn conform_snaps_endplate_faces_onto_the_real_bone() {
        let (disc, o4, o5, frame) = conform_scene();
        let conformed = conform_disc_to_endplates(&disc, &o4, &o5, &frame, None);
        assert_eq!(conformed.vertices.len(), disc.vertices.len());

        for (i, (v0, v)) in disc.vertices.iter().zip(&conformed.vertices).enumerate() {
            if v0.z > 16.0 {
                // Top face (nearer L4) → L4 bottom face at z=20 (moved +2 mm along +z).
                assert!(
                    o4.evaluate(*v).abs() < 1e-3,
                    "top vertex {i} must land on L4"
                );
                assert!(
                    (v.z - 20.0).abs() < 1e-3,
                    "top vertex {i} at z≈20, got {}",
                    v.z
                );
            } else if v0.z < 14.0 {
                // Bottom face (nearer L5) → L5 top face at z=10 (moved −2 mm along −z).
                assert!(
                    o5.evaluate(*v).abs() < 1e-3,
                    "bottom vertex {i} must land on L5"
                );
                assert!(
                    (v.z - 10.0).abs() < 1e-3,
                    "bottom vertex {i} at z≈10, got {}",
                    v.z
                );
            } else {
                // Mid row (z=15): 5 mm from each endplate, beyond the 4 mm cap → untouched.
                assert_eq!(*v, *v0, "interior vertex {i} must not move");
            }
        }
    }

    #[test]
    fn conform_leaves_overhang_vertices_in_place() {
        // A vertex far outside the L4/L5 footprint (x=50, half-width 10): its nearest bone
        // point is ~40 mm away, beyond SI_CONFORM_CAP → left exactly in place.
        let (mut disc, o4, o5, frame) = conform_scene();
        let overhang = Point3::new(50.0, 0.0, 18.0);
        disc.vertices.push(overhang);
        let conformed = conform_disc_to_endplates(&disc, &o4, &o5, &frame, None);
        let got = *conformed.vertices.last().unwrap();
        assert_eq!(
            got, overhang,
            "an overhang vertex must not be projected, got {got:?}"
        );
    }

    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn conform_seats_the_real_disc_endplate_faces_on_the_bone() {
        // The exact-geometry contract on the REAL anatomy: EVERY vertex the conform moves
        // sits on the nearer vertebra surface (|sdf| ≈ 0) — it never leaves a vertex floating
        // off the bone. A meaningful share of the disc (its top + bottom faces) must seat;
        // the untouched remainder is the disc interior / annulus (far from either endplate).
        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc = load_from_env("CF_DISC_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();

        let conformed = conform_disc_to_endplates(&disc, &o4, &o5, &frame, None);
        assert_eq!(conformed.vertices.len(), disc.vertices.len());
        assert_eq!(
            conformed.faces, disc.faces,
            "conform is vertex-only: topology unchanged"
        );

        // Radius from the SI axis through the disc centre — to show the untouched vertices are
        // the disc interior / annulus, not scattered face vertices the conform should have hit.
        let bbox = Aabb::from_points(disc.vertices.iter());
        let center = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
        let si = frame.superior_axis;
        let radius = |p: Point3<f64>| {
            let d = p - center;
            (d - d.dot(&si) * si).norm()
        };
        let (mut seated, mut untouched, mut worst) = (0_usize, 0_usize, 0.0_f64);
        let (mut r_seated, mut r_untouched) = (0.0_f64, 0.0_f64);
        for (v0, v) in disc.vertices.iter().zip(&conformed.vertices) {
            if v == v0 {
                untouched += 1;
                r_untouched += radius(*v0);
            } else {
                // A moved vertex MUST sit on the nearer vertebra surface — the contract.
                let d = o4.evaluate(*v).abs().min(o5.evaluate(*v).abs());
                assert!(
                    d < 1e-3,
                    "a moved vertex must land on a bone, got |sdf|={d:.4}"
                );
                seated += 1;
                worst = worst.max(d);
                r_seated += radius(*v0);
            }
        }
        let total = seated + untouched;
        println!(
            "conform: {seated}/{total} vertices seated on an endplate (worst |sdf| {worst:.2e} mm), {untouched} untouched (interior/annulus)"
        );
        println!(
            "mean radius from SI axis: seated {:.1} mm, untouched {:.1} mm",
            r_seated / seated.max(1) as f64,
            r_untouched / untouched.max(1) as f64,
        );
        // The disc's two endplate faces are a large share of its surface, so a healthy count
        // must seat — a regression that broke the projection would collapse this.
        assert!(
            seated > total / 4,
            "at least a quarter of the disc must seat ({seated}/{total})"
        );
    }

    /// A watertight axis-aligned box (8 verts, 12 outward-CCW tris) centred at `c` with the given
    /// half-extents — a closed surface the signed oracle can sign; a flat-endplate stand-in for a
    /// vertebra (and, thin, for the disc footprint).
    fn axis_box(c: Point3<f64>, half: Vector3<f64>) -> IndexedMesh {
        const S: [[f64; 3]; 8] = [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ];
        const F: [[u32; 3]; 12] = [
            [0, 3, 2],
            [0, 2, 1],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let vertices = S
            .iter()
            .map(|s| {
                Point3::new(
                    c.x + s[0] * half.x,
                    c.y + s[1] * half.y,
                    c.z + s[2] * half.z,
                )
            })
            .collect();
        IndexedMesh::from_parts(vertices, F.to_vec())
    }

    #[test]
    fn models_a_disc_that_fills_the_gap_between_two_box_endplates() {
        // Two box "vertebrae" with a 10 mm gap (upper z∈[5,15], lower z∈[-15,-5]); the modeled disc
        // must FILL the gap — top face on the upper box (z≈5), bottom on the lower (z≈-5), thickness
        // ≈ the 10 mm gap — regardless of the (deliberately too-thin, 8 mm) scanned-disc footprint.
        let upper = axis_box(Point3::new(0.0, 0.0, 10.0), Vector3::new(20.0, 20.0, 5.0));
        let lower = axis_box(Point3::new(0.0, 0.0, -10.0), Vector3::new(20.0, 20.0, 5.0));
        let disc = axis_box(Point3::new(0.0, 0.0, 0.0), Vector3::new(15.0, 15.0, 4.0));
        let params = DiscModelParams {
            cell: 1.0,
            si_margin: 4.0,
            footprint_erode: 0.0,
            smooth_iters: 0, // flat box endplates → no crazing to smooth; keep the exact thickness
            posterior_trim: None, // symmetric boxes have no defined posterior axis
        };
        let modeled = model_disc_between_endplates(&upper, &lower, &disc, &params).unwrap();
        assert!(!modeled.is_empty(), "carve must not be empty");

        let bb = Aabb::from_points(modeled.vertices.iter());
        let thickness = bb.max.z - bb.min.z;
        assert!(
            (thickness - 10.0).abs() < 1.5,
            "disc thickness {thickness:.2} mm should ≈ the 10 mm gap (fills it despite the thin footprint)"
        );
        assert!(
            (bb.max.z - 5.0).abs() < 1.0,
            "top face carved onto the upper box (z≈5), got {:.2}",
            bb.max.z
        );
        assert!(
            (bb.min.z + 5.0).abs() < 1.0,
            "bottom face carved onto the lower box (z≈-5), got {:.2}",
            bb.min.z
        );
    }

    #[test]
    fn model_rejects_a_misoriented_disc() {
        // SI must be native z (thinnest). A disc whose thinnest axis is x must be rejected loudly.
        let upper = axis_box(Point3::new(0.0, 0.0, 10.0), Vector3::new(20.0, 20.0, 5.0));
        let lower = axis_box(Point3::new(0.0, 0.0, -10.0), Vector3::new(20.0, 20.0, 5.0));
        let bad = axis_box(Point3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 15.0, 15.0)); // thinnest = x
        let err = model_disc_between_endplates(&upper, &lower, &bad, &DiscModelParams::default())
            .unwrap_err();
        assert!(
            err.to_string().contains("thinnest"),
            "expected a mis-orientation error, got: {err}"
        );
    }

    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn models_the_real_disc_seated_on_both_endplates() {
        // The pivot on real anatomy: the modeled disc is a single non-empty solid whose surface
        // seats a meaningful share on the REAL endplates (the fit the scanned disc lacked — its
        // superior face floated ~2.5 mm off L4).
        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc = load_from_env("CF_DISC_STL").unwrap();

        let modeled =
            model_disc_between_endplates(&l4, &l5, &disc, &DiscModelParams::default()).unwrap();
        assert!(!modeled.is_empty(), "modeled disc must not be empty");

        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let on_bone = modeled
            .vertices
            .iter()
            .filter(|v| o4.evaluate(**v).abs() < 0.5 || o5.evaluate(**v).abs() < 0.5)
            .count();
        let frac = on_bone as f64 / modeled.vertices.len() as f64;
        assert!(
            frac > 0.3,
            "at least 30% of the disc surface should seat on an endplate (top+bottom faces), got {:.0}%",
            frac * 100.0
        );
        println!(
            "modeled disc: {} surface verts, {:.0}% seated on L4/L5",
            modeled.vertices.len(),
            frac * 100.0
        );
    }
}
