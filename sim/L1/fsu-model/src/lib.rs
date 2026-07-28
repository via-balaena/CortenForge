//! The assembled **Functional Spinal Unit** (FSU) flexion model.
//!
//! `cf-fsu-geometry` turns a raw vertebra/disc mesh into static anatomical geometry;
//! this crate turns it into a *live, simulatable* FSU. Two layers:
//!
//! - the **bonded soft disc** ([`build_bonded_disc`] / [`BondedDisc`]): tet-meshes the
//!   real intervertebral disc from its own signed field and bonds it between two rigid
//!   vertebra-endplate boxes ([`BondedSandwich`]), then drives its quasi-static
//!   flexion/extension response. It comes in two element arms over one shared geometry
//!   pipeline — the linear [`Tet4`] disc and the quadratic [`build_bonded_disc_tet10`]
//!   one, which is ~1/3 softer in bending because the linear element bending-locks. With
//!   endplates supplied, the quadratic arm is genuinely **curved**: its bonded-face
//!   boundary midsides are projected onto the real endplate too, so the bonded face
//!   follows the bone between its nodes instead of chording across it
//!   (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` rung 3);
//! - the **coupled FSU** ([`CoupledFsu`]): assembles the disc (as a
//!   linearised bushing), the ligaments (tendons), and the facets (oriented SDF contact)
//!   into ONE model and solves for the equilibrium pose under an applied moment — the
//!   force-driven, ROM-limited segment, vs rung 7's analytic superposition of the parts.
//!   Since rung 4 it assembles on the **curved quadratic** disc: the segment's physics runs
//!   on the real endplate, not on a floating raw mesh, and the linear arm remains available
//!   as [`CoupledFsuTet4`] so the effect of that flip stays measurable.
//!
//! ## Two frames, one bridge
//!
//! The disc solves in its own recentred **SI-metre** frame — the soft solver's
//! convergence tolerance is an absolute residual floor, so the native ~950 mm
//! coordinates must be recentred to the origin and scaled mm→m first (rung-6c
//! discipline). The vertebrae / ligaments / facets, by contrast, live in native
//! millimetres. The map is a pure translate + uniform scale, so
//! [`BondedDisc::center_native`] (the pivot) and the scale bridge the two: a solved
//! node at `p_si` sits at `center_native + p_si / scale` in native mm, and the
//! flexion axis ([`BondedDisc::ml_axis`]) is the same coordinate direction in both.
//!
//! ## Scope / honesty
//!
//! The limit is on the size of a **single step**, not on the angle reached. A lone jump
//! from rest past ~1° drives the boundary tets out of their SPD region and the soft solve
//! diverges (and panics), so [`BondedDisc::flexion_moment`] is a small-angle probe and
//! rung 7's larger-angle range of motion is a linear extrapolation of it. But a *chain* of
//! small warm-started steps goes much further: `CoupledFsu::capture_ramp` walks the linear
//! disc to the full ±ROM in 0.1° sub-steps, and rung 2 of
//! `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` measured the same chain reaching **±6.0° on
//! the endplate-conformed disc for BOTH elements** — every step converging, conserving and
//! strictly restoring (`conformed_disc_survives_a_large_angle_sweep`). So the quadratic
//! arm's large-angle envelope is no longer unmeasured, and it is not narrower than the
//! linear arm's.
//!
//! The single-step limit is *tighter* on a conformed disc, which is measured rather than
//! assumed: the raw linear disc survives a single +0.5° → −0.5° jump (rung 1's FOM probes
//! it exactly that way) while the conformed one does not — the same 1° jump trips the
//! solver's fail-closed validity bound. Drive a conformed disc in steps.
//!
//! ⚠ That is why [`CoupledFsu`] tracks the angle its disc is sitting at and routes **every**
//! drive — the `k_disc` probe, the return to rest, each leg of a capture — through one
//! stepping driver. Once the segment assembles on a conformed disc, "jump straight to θ"
//! stops being safe anywhere, including in code that used to do it: the 0.86° `k_disc` probe
//! is itself past the conformed disc's single-jump envelope. Rung 4 measured that walking
//! there does not move the number (the linear arm still reads −0.2819 to four decimals).
//!
//! Note the deliberate API asymmetry: [`build_bonded_disc`] returns [`Result`], but the
//! drive methods **panic** on non-convergence rather than returning one — they inherit
//! the fail-close contract of [`BondedSandwich::probe`](sim_coupling::BondedSandwich),
//! which aborts loudly on a diverged solve. Consumers keep `|theta|` inside the
//! validated sub-degree range (a fixed safe sweep never diverges). Threading a
//! recoverable `Result` through the probe is a sim-coupling change deferred until a
//! consumer must drive to genuinely unknown angles.

use std::collections::BTreeSet;

use anyhow::{Context, Result, bail};
use cf_fsu_geometry::{MeshOracle, bonded_conform_target, oracle};
use cf_geometry::{Aabb, IndexedMesh};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_core::{Data, Model};
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::{
    Aabb3, Element, MaterialField, Mesh, MeshingHints, SdfMeshedTetMesh, Tet4, Tet10Mesh, TetId,
    Vec3, element::TET10_EDGE_NODES, element::Tet10, pick_vertices_by_predicate,
    referenced_vertices,
};
// Re-exported: `FlexionTrajectory::boundary_faces` is `Vec<[VertexId; 3]>`, so consumers
// (e.g. a viewer building a mesh from it) need to name the vertex-index type.
pub use sim_soft::VertexId;

mod coupled;
pub use coupled::{
    CoupledFrame, CoupledFsu, CoupledFsuTet4, CoupledParams, CoupledTrajectory, PHYSIOLOGIC_MOMENT,
    RAMP_FRAMES, is_engaged, moment_ramp, posed_facet_contacts,
};

/// Body index of the inferior (lower) vertebra box in the disc scene (world = 0).
const LOWER: usize = 1;
/// Body index of the superior (upper) vertebra box.
const UPPER: usize = 2;

/// Quality floor for the endplate conform's back-off: a projected bonded-band node is
/// backed off until every incident tet's rest Jacobian is at least this fraction of its
/// original value. A bare `> 0` bisects onto the `detJ → 0⁺` degeneracy boundary and
/// manufactures slivers, so the floor holds `detJ` above a fraction of its healthy rest
/// value instead. See `sim_soft::SdfMeshedTetMesh::with_projected_nodes`.
///
/// **Raised 0.05 → 0.25 at rung 4, because 0.05 was measured on ONE disc and does not
/// transfer.** At 0.05 the conform produces a mesh that is valid by the projector's own rule
/// and **cannot be driven at all** on a *lofted* (painted) disc — `CoupledFsu::build`'s 0.86°
/// `k_disc` probe dies on a fail-closed validity violation (principal stretch 2.055). The
/// scanned `BodyParts3D` disc happens to tolerate 0.05; a second geometry does not. Since
/// `build_bonded_disc` accepts any disc mesh, a floor that only works on one of them is a
/// defect in the default, not a property of the input.
///
/// **Measured on BOTH geometries.** Drivability is the largest single from-rest jump the
/// conformed disc survives, probed to 0.86° in 0.1° warm-started sub-steps (the production
/// path). Fidelity is the authorised bonded-face residual to the real bone on the scanned
/// disc — the arc's payoff metric, which a higher floor costs:
///
/// | floor | lofted Tet4 | lofted Tet10 | scanned corner RMS (mm) | scanned midside RMS |
/// |---|---|---|---|---|
/// | 0.05 | **stalls** | **stalls** | 0.656 | 0.694 |
/// | 0.08 | drives | **stalls** | — | — |
/// | 0.10 | drives | drives | — | — |
/// | 0.15 | drives | drives | 0.700 | 0.729 |
/// | 0.20 | drives | drives | 0.724 | 0.748 |
/// | **0.25** | **drives** | **drives** | **0.750** | **0.767** |
/// | 0.40 | drives | drives | 0.836 | — |
///
/// So the cliff sits at **0.10** (the quadratic element is the stricter constraint — the same
/// direction rung 3 found for midsides), and 0.25 buys **2.5× margin** for 0.094 mm of RMS
/// residual. The conform still nearly halves the seated population's distance to the bone
/// (raw 1.332 → 0.750 mm). `k_disc` varies by **&lt;0.1 %** across the whole 0.05–0.60 range on
/// the lofted disc (−24.83 … −24.86 Tet4, −22.28 … −22.35 Tet10): like
/// `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`, this floor moves the *conditioning*, not the physics.
///
/// Chosen with margin above the cliff rather than at its edge, for the reason rung 3 gives: a
/// cliff's location is a property of the meshes it was measured on, and only two are known
/// here. It generalizes rung 3's rule from "do not inherit a tuning constant across element
/// orders" to "**or across input geometries**".
const DISC_CONFORM_QUALITY_FLOOR: f64 = 0.25;

/// Quality floor for the **midside** projection of a curved quadratic disc (rung 3) — the
/// same rule as [`DISC_CONFORM_QUALITY_FLOOR`], but held higher, and the difference is
/// measured rather than inherited.
///
/// A Tet4 corner move perturbs an affine element's single Jacobian; a Tet10 midside move bends
/// the element's Jacobian at four interior Gauss points, so the *same* fractional floor is not
/// the same amount of distortion. Reusing 0.05 here produces a mesh that is valid by the
/// projector's own rule and yet no longer drivable: the largest single jump from rest it
/// survives is **0.10°** — 0.15° already stalls Newton with a `NaN` residual — against 0.5° for
/// the straight-conformed disc rung 2 measured. That is a five-fold loss of step envelope, and
/// `CoupledFsu::capture_ramp` (rung 4) drives 0.1° sub-steps with a per-frame equilibrium
/// bisection on top.
///
/// **Measured on the real disc (`BodyParts3D`, `DiscParams::default`)** — authorised-midside RMS
/// residual to the bone, projection coverage, and whether the disc survives a single 0.5°
/// jump from rest:
///
/// | floor | RMS residual (mm) | delivered | 0.5° jump |
/// |---|---|---|---|
/// | 0.05 | 0.658 | 72.4 % | **stalls** |
/// | 0.20 | 0.673 | 70.5 % | **stalls** |
/// | 0.25 | 0.678 | 70.0 % | solves |
/// | **0.40** | **0.694** | **67.9 %** | **solves** |
/// | 0.60 | 0.720 | 65.5 % | solves |
/// | 0.80 | 0.752 | 59.0 % | solves |
///
/// So the whole cost of a well-conditioned mesh is **0.036 mm of RMS residual** (0.658 → 0.694,
/// on a population whose straight arm sits at 0.796), and the restoring moment at 0.5° is
/// identical to six digits across every row — the floor moves the conditioning, not the physics.
/// 0.40 is chosen with margin rather than at the measured cliff (0.20 → 0.25): the cliff's
/// location is a property of this mesh, and tuning to its edge would make the next mesh's
/// failure a surprise.
///
/// ⚠ **The whole table above was measured at the rung-3 corner floor of 0.05**, which rung 4
/// then raised to 0.25 (see [`DISC_CONFORM_QUALITY_FLOOR`]). Since the midsides are projected
/// from *conformed corners*, every row's absolute numbers shift — the shipped row now reads
/// 0.767 mm RMS at 67.4 % delivered. Read the rows as the *comparison between floors* they were
/// built to be. The **choice** of 0.40 is re-verified rather than assumed: it drives on both the
/// scanned and the lofted disc at the new corner floor. Re-deriving the midside cliff there is
/// not done, and would only be able to move it in the permissive direction — a better-conditioned
/// corner mesh cannot make a midside floor *more* necessary — so 0.40 keeps at least the margin
/// it was chosen with.
const DISC_MIDSIDE_CONFORM_QUALITY_FLOOR: f64 = 0.4;

/// Tunable parameters for [`build_bonded_disc`]. [`Default`] reproduces the
/// rung-6c/7 disc recipe exactly.
#[derive(Debug, Clone, Copy)]
pub struct DiscParams {
    /// Neo-Hookean shear modulus μ (Pa); the first Lamé parameter is `λ = 4μ`.
    pub mu: f64,
    /// Similarity scale applied at load (native mm → solver metres). The soft
    /// solver's tolerance is an absolute residual floor, so the disc is recentred +
    /// scaled into SI metres before meshing (rung-6c discipline).
    pub scale: f64,
    /// Tet lattice spacing in the solver frame (m).
    pub cell: f64,
    /// Lattice padding beyond the disc AABB (m).
    pub pad: f64,
    /// Endplate band thickness as a fraction of the disc's SI extent.
    pub band_frac: f64,
    /// Rigid vertebra-box half-thickness (m).
    pub h_box: f64,
    /// Quasi-static timestep (large, so the inertial term `M/dt²` is negligible).
    pub static_dt: f64,
}

impl Default for DiscParams {
    fn default() -> Self {
        Self {
            mu: 1.0e5,
            scale: 1.0e-3,
            cell: 0.003,
            pad: 0.0015,
            band_frac: 0.18,
            h_box: 0.006,
            static_dt: 1.0e3,
        }
    }
}

/// The vertebral endplate oracles + SI axis that seat a bonded disc onto the **real** bone.
///
/// Strategy B: mesh the raw disc, then conform its bonded-band boundary nodes onto the endplate.
/// Pass `Some(...)` to [`build_bonded_disc`] to bond the disc to the exact endplate surface
/// (exact geometry === exact physics); pass `None` to bond the raw floating disc (the
/// pre-conform baseline). The oracles are native-mm [`cf_fsu_geometry::oracle`]s of the two
/// vertebrae; nearest-of-the-two is chosen per node, so which is `o4`/`o5` only labels them.
#[derive(Clone, Copy)]
pub struct EndplateConform<'a> {
    /// Native-mm signed-distance oracle of the superior (L4) vertebra.
    pub o4: &'a MeshOracle,
    /// Native-mm signed-distance oracle of the inferior (L5) vertebra.
    pub o5: &'a MeshOracle,
    /// The unit SI (superior) axis — `cf_fsu_geometry::SegmentFrame::superior_axis`. Used by
    /// the alignment guard; invariant under the disc's translate + uniform-scale solver bridge.
    pub superior_axis: Vector3<f64>,
}

/// A live bonded intervertebral disc.
///
/// The real disc geometry tet-meshed and bonded between two rigid vertebra-endplate
/// boxes, plus the frame data a flexion probe needs. Build it with [`build_bonded_disc`].
///
/// Carries the same `<Msh, E, N, G>` parameters as the underlying
/// [`BondedSandwich`], defaulting to the linear [`Tet4`] disc on an
/// [`SdfMeshedTetMesh`] — every method here is node-based, so the quadratic arm of the
/// Tet10 migration is a type substitution rather than a fork
/// (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` rung 1, which needs both element arms
/// live at once to measure the `k_disc` ratio).
///
/// **One method is not element-neutral in its *semantics*:** [`Self::boundary_faces`]
/// returns corner-only triangles, so on a quadratic disc the rendered surface reflects
/// the linear field. It compiles and the indices stay valid — see that method's warning.
pub struct BondedDisc<Msh = SdfMeshedTetMesh, E = Tet4, const N: usize = 4, const G: usize = 1>
where
    Msh: Mesh,
    E: Element<N, G>,
{
    sandwich: BondedSandwich<Msh, E, N, G>,
    /// The medio-lateral (flexion/extension) axis: a coordinate unit vector.
    ml_axis: Vector3<f64>,
    /// The superior box's body-origin position at rest, in the solver SI frame.
    rest_upper: Vec3,
    /// The disc AABB centre in native mm — the shared flexion pivot.
    center_native: Point3<f64>,
    /// The native-mm → solver-metre similarity scale (`params.scale`), retained so
    /// [`Self::deformed_nodes_native`] can invert the solve frame back to native mm
    /// (`center_native + p_si / scale`). Rendering is its first consumer.
    scale: f64,
}

/// The two-box disc scene: free-joint inferior / superior vertebra boxes whose COMs
/// sit `h_box` beyond each endplate centroid along SI, so each box face meets a disc
/// surface. Gravity off — the scene is probe-driven.
fn disc_mjcf(c_inf: Vec3, c_sup: Vec3, h_box: f64) -> String {
    let lo = c_inf - Vec3::z() * h_box;
    let hi = c_sup + Vec3::z() * h_box;
    format!(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/><worldbody>
    <body name="inf" pos="{lx} {ly} {lz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    <body name="sup" pos="{ux} {uy} {uz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    </worldbody></mujoco>"#,
        lx = lo.x,
        ly = lo.y,
        lz = lo.z,
        ux = hi.x,
        uy = hi.y,
        uz = hi.z,
        h = h_box,
    )
}

/// The disc's medio-lateral (flexion/extension) axis: the coordinate axis of the point
/// cloud's widest **AABB extent**.
///
/// Deliberately the axis-aligned AABB extent — NOT a PCA principal direction (an oblique
/// disc's longest point-to-point direction can differ from its widest axis-aligned extent),
/// and NOT [`Aabb::longest_axis`] (which resolves an exact extent tie to the FIRST maximum;
/// rung 7's original `max_by` takes the LAST, and a byte-identical extraction must match it).
/// A real anatomical disc has a unique widest axis, so the choices agree on every real input.
fn ml_axis_from_points(vertices: &[Point3<f64>]) -> Vector3<f64> {
    let size = Aabb::from_points(vertices.iter()).size();
    let extents = [size.x, size.y, size.z];
    let widest = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap_or(0);
    let mut ml = Vector3::zeros();
    ml[widest] = 1.0;
    ml
}

/// The endplate-conform moves for a bonded disc's band nodes: each endplate-facing SURFACE node
/// in the inferior/superior bands, projected onto the nearer real endplate via
/// [`bonded_conform_target`] (SI-alignment-gated), as `(node, target)` in the solver SI frame.
///
/// Interior band nodes are omitted (restricted to the mesh boundary — they are not endplate-facing
/// and the alignment guard would decline them anyway) and rim nodes the guard declines are left
/// straight, so the overhanging annular rim (which attaches to the ring apophysis, not the
/// endplate face) stays put. The frame bridge (rung-6c discipline) lifts each node from the disc's
/// SI-metre solver frame to the native-mm oracle and brings the target back; the SI axis is a
/// coordinate direction, identical in both frames (translate + uniform scale).
fn endplate_conform_moves(
    tet: &SdfMeshedTetMesh,
    inferior: &[VertexId],
    superior: &[VertexId],
    ep: EndplateConform,
    center_native: Point3<f64>,
    scale: f64,
) -> Vec<(VertexId, Vec3)> {
    let surface: std::collections::HashSet<VertexId> =
        tet.boundary_faces().iter().flatten().copied().collect();
    let mut moves: Vec<(VertexId, Vec3)> = Vec::new();
    for &node in inferior.iter().chain(superior) {
        if !surface.contains(&node) {
            continue;
        }
        let p_si = tet.positions()[node as usize];
        let n_native = center_native + p_si / scale;
        if let Some(target_native) = bonded_conform_target(n_native, ep.superior_axis, ep.o4, ep.o5)
        {
            let target_si = (target_native - center_native) * scale;
            moves.push((node, target_si));
        }
    }
    moves
}

/// The **bonded-face boundary midsides** of an enriched disc: the trailing three slots of
/// every six-node boundary face whose three corners all lie in the bonded corner band.
///
/// This is the quadratic analogue of the surface restriction `endplate_conform_moves` applies
/// to the corner band, and it is the population rung 3 projects onto the real endplate. Two
/// exclusions are deliberate:
///
/// - **Interior band midsides stay at their edge midpoints.** The band is a volumetric slab
///   (`full_face_band`), so most of its midsides are inside the disc with no endplate to sit
///   on; projecting them is the #699 false-degradation failure mode.
/// - **Only fully-in-band faces count.** A boundary face straddling the band edge has a
///   midside whose parents are not both bonded; curving it would bow an element that is only
///   half tied.
///
/// Returned as a [`BTreeSet`] so the population is deduplicated (a midside is shared by every
/// face that carries it) and deterministically ordered.
fn bonded_face_boundary_midsides(
    tet10: &Tet10Mesh,
    inferior: &[VertexId],
    superior: &[VertexId],
) -> BTreeSet<VertexId> {
    let band: std::collections::HashSet<VertexId> =
        inferior.iter().chain(superior).copied().collect();
    let Some(faces6) = tet10.boundary_faces6() else {
        // Unreachable for a `Tet10Mesh` (it always carries six-node faces); skip rather than
        // unwrap so this stays a total function over the `Mesh` trait.
        return BTreeSet::new();
    };
    faces6
        .iter()
        .filter(|f| f[..3].iter().all(|v| band.contains(v)))
        .flat_map(|f| f[3..].iter().copied())
        .collect()
}

/// The **midside** endplate-conform moves for an enriched disc: each bonded-face boundary
/// midside (see [`bonded_face_boundary_midsides`]) projected onto the nearer real endplate via
/// [`bonded_conform_target`], as `(node, target)` in the solver SI frame.
///
/// The exact discriminator the corner conform uses (`endplate_conform_moves`) — nearest of the
/// two endplates, SI-alignment primary, loose distance backstop — applied one level up, so an
/// overhanging annular-rim midside is declined for the same reason its parent corners were.
/// The frame bridge is the same too: lift into native mm, project, bring the target back.
///
/// A straight midside chords *across* the curved endplate even when both of its parent corners
/// are seated exactly on the bone, so this is not implied by the corner conform: it is the gap
/// that makes the bonded face genuinely curved rather than a fan of flat chords.
fn endplate_midside_conform_moves(
    tet10: &Tet10Mesh,
    inferior: &[VertexId],
    superior: &[VertexId],
    ep: EndplateConform,
    center_native: Point3<f64>,
    scale: f64,
) -> Vec<(VertexId, Vec3)> {
    let positions = tet10.positions();
    bonded_face_boundary_midsides(tet10, inferior, superior)
        .into_iter()
        .filter_map(|node| {
            let p_si = positions[node as usize];
            let n_native = center_native + p_si / scale;
            let target_native = bonded_conform_target(n_native, ep.superior_axis, ep.o4, ep.o5)?;
            Some((node, (target_native - center_native) * scale))
        })
        .collect()
}

/// Everything the linear and quadratic disc arms share: the meshed (and optionally
/// endplate-conformed) **linear** tet mesh, its two *corner* endplate bands, the posed
/// two-box rigid scene, and the frame data a flexion probe needs.
///
/// The element choice enters strictly *after* this point — [`build_bonded_disc`] bonds
/// `tet` directly, [`build_bonded_disc_tet10`] enriches it and widens the bands — so the
/// two arms are guaranteed to share one geometry, one band rule, and one rigid scene. That
/// is what makes their `k_disc` ratio a measurement of the **element order alone**
/// (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §0.1 attribution bound 1).
struct PreparedDisc {
    /// The linear disc mesh, conformed onto the endplates if `endplates` was `Some`.
    tet: SdfMeshedTetMesh,
    /// The inferior endplate band — orphan-filtered **corner** vertex ids.
    inferior: Vec<VertexId>,
    /// The superior endplate band — orphan-filtered **corner** vertex ids.
    superior: Vec<VertexId>,
    /// The two-box rigid scene, already `forward`-ed so both endplate poses are current.
    model: Model,
    /// The scene's forward-ed state (the pose snapshot each bond's rest offsets read).
    data: Data,
    /// The superior box's body-origin position at rest, in the solver SI frame.
    rest_upper: Vec3,
    /// The medio-lateral (flexion/extension) axis.
    ml_axis: Vector3<f64>,
    /// The disc AABB centre in native mm — the shared flexion pivot.
    center_native: Point3<f64>,
}

/// Tet-mesh the real intervertebral disc `mesh` (native mm), optionally seat its bonded
/// band on the real endplate, and pose the two rigid endplate boxes around it — the
/// element-agnostic half of [`build_bonded_disc`] / [`build_bonded_disc_tet10`].
///
/// # Errors
/// See [`build_bonded_disc`] — every failure mode listed there originates here.
// `vs.len()` (an endplate vertex count) is tiny; the usize→f64 cast for its centroid
// is exact for any real mesh.
#[allow(clippy::cast_precision_loss)]
fn prepare_disc(
    mut mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
) -> Result<PreparedDisc> {
    let bbox0 = Aabb::from_points(mesh.vertices.iter());
    let center_native = Point3::from(bbox0.min.coords + (bbox0.max - bbox0.min) * 0.5);
    for v in &mut mesh.vertices {
        *v = Point3::from((v.coords - center_native.coords) * params.scale);
    }
    let bbox = Aabb::from_points(mesh.vertices.iter());

    // Principal axes from the AABB (rung-6c discipline): SI = thinnest (guarded to be
    // native z), ML = widest (the flexion/extension axis) — no axis taken on faith.
    let size = bbox.size();
    if !(size.z < size.x && size.z < size.y) {
        bail!(
            "disc SI extent (thinnest) must be native z; got extents ({:.4}, {:.4}, {:.4}) m — mis-oriented mesh?",
            size.x,
            size.y,
            size.z
        );
    }
    let ml_axis = ml_axis_from_points(&mesh.vertices);

    // Pad the lattice beyond the disc so the tet mesh fully contains the surface.
    let padded = bbox.expanded(params.pad);
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(padded.min.x, padded.min.y, padded.min.z),
            Vec3::new(padded.max.x, padded.max.y, padded.max.z),
        ),
        cell_size: params.cell,
        material_field: Some(MaterialField::uniform(params.mu, 4.0 * params.mu)),
    };
    // Build the exact SDF only once the cheap bbox guards have passed (a mis-oriented
    // mesh bails above without paying for the oracle).
    let sdf = oracle(&mesh).context("disc oracle")?;
    let tet = SdfMeshedTetMesh::from_sdf(&sdf, &hints)
        .map_err(|e| anyhow::anyhow!("tet-mesh disc: {e:?}"))?;
    // A physical disc is one connected solid, but the BCC isosurface-stuffing mesher
    // fragments the disc's sub-cell-thin tapering rim into disconnected islands — which
    // both scatter the rendered surface and poison the Newton tangent's conditioning
    // (a floating tet component carries unconstrained rigid modes). Keep the main body.
    let mut tet = tet.largest_component();

    // Endplate faces = bands at the SI surface extremes (field-derived, not z=const).
    let (lo_z, hi_z) = (bbox.min.z, bbox.max.z);
    let band = params.band_frac * (hi_z - lo_z);
    // `largest_component` (and the mesher's own lattice) retain unreferenced "orphan"
    // vertices; a spatial predicate over ALL positions can pick them, and bonding a
    // zero-stiffness orphan (or averaging it into the endplate centroid) would silently
    // skew the disc. Drop orphans first — the established `referenced_vertices` pattern.
    let referenced: std::collections::HashSet<VertexId> =
        referenced_vertices(&tet).into_iter().collect();
    let inferior: Vec<VertexId> = pick_vertices_by_predicate(&tet, |p| p.z < lo_z + band)
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
    let superior: Vec<VertexId> = pick_vertices_by_predicate(&tet, |p| p.z > hi_z - band)
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
    if inferior.is_empty() || superior.is_empty() {
        bail!(
            "endplate band ({:.4} m) captured no vertices (inferior {}, superior {}) — increase band_frac or refine cell",
            band,
            inferior.len(),
            superior.len()
        );
    }
    // The two bands must be DISJOINT: a vertex in both would bond one disc node to both
    // vertebrae, which `BondedSandwich::from_tet_mesh` rejects with a panic. A
    // `band_frac ≥ 0.5` makes the bands meet/overlap at the mid-plane. Catch it here as
    // a recoverable error (the `# Errors` contract) rather than a downstream panic.
    let inferior_set: std::collections::HashSet<VertexId> = inferior.iter().copied().collect();
    if superior.iter().any(|v| inferior_set.contains(v)) {
        bail!(
            "inferior/superior endplate bands overlap (band_frac {:.2} too large) — they share a vertex, which cannot bond to both endplates",
            params.band_frac
        );
    }

    // Strategy B: with the raw disc meshed (well-conditioned), seat the bonded-band boundary
    // nodes onto the REAL vertebral endplate so the bond ties to the exact bone rather than a
    // floating raw disc (exact geometry === exact physics). The quality-floor back-off keeps
    // every incident tet well-shaped; the rest offset the bond snapshots
    // (`BondedSandwich::from_tet_mesh`, below) is read from these conformed positions, so the
    // bond ties to the seated surface with NO change to the bond math.
    if let Some(ep) = endplates {
        let moves =
            endplate_conform_moves(&tet, &inferior, &superior, ep, center_native, params.scale);
        tet = tet.with_projected_nodes(&moves, DISC_CONFORM_QUALITY_FLOOR);
    }

    // Endplate-box poses derive from the (now possibly conformed) band centroids, so the
    // rigid carriers meet the seated disc faces.
    let cen = |vs: &[VertexId]| -> Vec3 {
        let s: Vec3 = vs.iter().map(|&v| tet.positions()[v as usize]).sum();
        s / vs.len() as f64
    };
    let (c_inf, c_sup) = (cen(&inferior), cen(&superior));

    let model = load_model(&disc_mjcf(c_inf, c_sup, params.h_box)).context("disc scene MJCF")?;
    let mut data = model.make_data();
    data.forward(&model).context("disc scene forward")?;
    let rest_upper = data.xpos[UPPER];
    Ok(PreparedDisc {
        tet,
        inferior,
        superior,
        model,
        data,
        rest_upper,
        ml_axis,
        center_native,
    })
}

/// Tet-mesh the real intervertebral disc `mesh` (native mm) and bond it between two
/// field-posed rigid endplate boxes, returning the live [`BondedDisc`].
///
/// This is the **linear** ([`Tet4`]) arm; [`build_bonded_disc_tet10`] is the quadratic one.
///
/// The disc is recentred to its AABB centre and scaled by `params.scale` into the
/// solver's SI-metre frame; its principal axes are derived from the AABB (SI =
/// thinnest, ML/flexion = widest — no axis taken on faith).
///
/// With `endplates = Some(...)`, the bonded-band boundary nodes are seated onto the
/// **real** vertebral endplate before bonding (Strategy B): the raw disc is meshed first
/// (well-conditioned), then its endplate-facing surface nodes are projected onto the nearer
/// endplate oracle — SI-alignment-gated so the overhanging annular rim stays straight — and
/// backed off to keep every incident tet well-shaped, so the physics runs on the exact bone
/// rather than a floating raw disc. With `endplates = None` the raw floating disc is bonded
/// unchanged (the pre-conform baseline).
///
/// # Errors
/// Returns an error if the disc's thinnest extent is not its native `z` (a
/// mis-oriented mesh), if an endplate band captures no vertices (band/cell too coarse
/// for the mesh), if the two endplate bands overlap (`band_frac` too large — they
/// would share a vertex), or if the signed-distance oracle, tet-mesher, or MJCF scene
/// build fails.
pub fn build_bonded_disc(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
) -> Result<BondedDisc> {
    let p = prepare_disc(mesh, params, endplates)?;
    let sandwich = BondedSandwich::from_tet_mesh(
        p.model,
        p.data,
        LOWER,
        UPPER,
        p.tet,
        p.inferior,
        p.superior,
        params.static_dt,
    );
    Ok(BondedDisc {
        sandwich,
        ml_axis: p.ml_axis,
        rest_upper: p.rest_upper,
        center_native: p.center_native,
        scale: params.scale,
    })
}

/// The **quadratic** ([`Tet10`]) sibling of [`build_bonded_disc`].
///
/// The identical disc geometry, band rule and rigid scene, enriched to a curved-capable
/// quadratic element and bonded with a **full-face tie**.
///
/// Everything before the element choice is literally the same code (one shared private
/// `prepare_disc`), so a `k_disc` comparison between the two arms isolates the element
/// order (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` rung 1). Two things differ:
///
/// - the mesh is enriched with [`Tet10Mesh::from_tet4`], which preserves every corner id
///   and position bit-for-bit and appends the shared edge midsides after them;
/// - each bonded band is widened to a **full-face tie** (`full_face_band`) — the band
///   corners *plus* every midside interior to the band. Pinning corners alone would
///   corner-spot-weld a quadratic band and soften `k_disc` on top of the element effect
///   (the retracted 43 % artifact, plan §0.1); this is the modeling choice that keeps the
///   measurement about the element.
///
/// With `endplates = Some(...)` the disc is **curved**, not merely conformed (plan rung 3):
/// after the corner band is seated (in `prepare_disc`) and the mesh enriched, the bonded-face
/// boundary *midsides* are projected onto the real endplate too
/// (`endplate_midside_conform_moves` + `Tet10Mesh::with_projected_midsides`), so the bonded
/// face is genuinely curved between its nodes instead of a fan of flat chords through seated
/// corners. Interior-band midsides stay at their edge midpoints and the overhanging annular
/// rim stays straight — the same two exclusions the corner conform makes, for the same
/// reasons. With `endplates = None` every midside stays at its edge midpoint: that arm is the
/// *straight* Tet10 disc rung 1 measured, and it is untouched by rung 3.
///
/// **Ordering is load-bearing:** conform corners → enrich → project midsides → *bond last*.
/// `BondedSandwich::from_tet_mesh` snapshots the rest positions its body-frame offsets are
/// frozen from, so a node moved after that point would silently tie the bond to un-projected
/// geometry.
///
/// # Errors
/// Same as [`build_bonded_disc`].
pub fn build_bonded_disc_tet10(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
) -> Result<BondedDisc<Tet10Mesh, Tet10, 10, 4>> {
    let p = prepare_disc(mesh, params, endplates)?;
    let mut tet10 = Tet10Mesh::from_tet4(&p.tet);
    if let Some(ep) = endplates {
        let moves = endplate_midside_conform_moves(
            &tet10,
            &p.inferior,
            &p.superior,
            ep,
            p.center_native,
            params.scale,
        );
        tet10 = tet10.with_projected_midsides(&moves, DISC_MIDSIDE_CONFORM_QUALITY_FLOOR);
    }
    // Topological, so the band is the same set before and after the projection — but it is
    // computed here, on the mesh that is about to be bonded, so the two can never disagree.
    let inferior = full_face_band(&tet10, &p.inferior);
    let superior = full_face_band(&tet10, &p.superior);
    let sandwich = BondedSandwich::from_tet_mesh(
        p.model,
        p.data,
        LOWER,
        UPPER,
        tet10,
        inferior,
        superior,
        params.static_dt,
    );
    Ok(BondedDisc {
        sandwich,
        ml_axis: p.ml_axis,
        rest_upper: p.rest_upper,
        center_native: p.center_native,
        scale: params.scale,
    })
}

/// Widen a linear (corner-only) endplate band into the **full-face tie** a quadratic band
/// needs: `corner_band` plus every midside node whose *two parent corners are both in the
/// band* (the topological closure of the band over element edges).
///
/// ## Why the closure, and why not the geometric predicate
///
/// `DiscParams::band_frac` makes each bonded band a volumetric **slab**, not a surface, so
/// most of the midsides added here are *interior* to the slab rather than on the bonded
/// face. The honest statement of the rule is therefore: *a quadratic slab tie must pin the
/// midsides interior to the slab as well as those on its face — otherwise the slab is
/// corner-spot-welded and softens for a bond reason rather than an element reason.*
///
/// The linear band rule is **geometric** (a z-slab predicate over positions); this one is
/// **topological**. They are deliberately different sets: a midside between an in-band and
/// an out-of-band corner can sit inside the slab yet stay free here. The closure is chosen
/// because it guarantees *element fully inside the band ⇒ element fully pinned* and never
/// leaves a half-pinned edge — so the measured band sizes and `k_disc` ratio are specific
/// to **this** rule (plan §2.2).
///
/// ## Two structural traps this avoids
///
/// - **Never filter the result through [`referenced_vertices`]** — it walks four-corner
///   connectivity only (`sim_soft::referenced_vertices`), so it would silently delete every
///   midside and reproduce the corner-spot-weld by omission. It is unnecessary here anyway:
///   midsides are created only inside the tet walk, so an *orphan* midside cannot exist, and
///   `corner_band` is already orphan-filtered.
/// - **[`BTreeSet`], not a `HashSet`** — it dedups (a midside is shared by every incident
///   tet, and `BondedSandwich::from_tet_mesh` rejects a repeated id because it would
///   double-count that node's reaction) *and* it fixes iteration order, which the bond's
///   wrench sum walks (`face_wrench` accumulates `w[3..] += f` and the moment term in
///   `verts.iter()` order, `sim/L1/coupling/src/bonded.rs`) — so a `HashSet`, whose order is
///   randomly seeded per process, would make `k_disc` non-reproducible run to run.
///
/// The local edge→midside slot mapping is [`TET10_EDGE_NODES`], cited rather than re-typed: a
/// permuted table pins the *wrong* midsides, and the instrument for that is the rung-1 gate's
/// set-equality cross-check, not a stiffness band. (Measured, so the limits are known rather
/// than assumed: rotating the slot index by one on the synthetic disc changed the band from
/// 413 to 647 ids, which an exact size assert would also have caught — the reason equality is
/// still the right instrument is that a size-preserving permutation is possible and a size
/// assert would then see nothing.)
fn full_face_band(tet10: &Tet10Mesh, corner_band: &[VertexId]) -> Vec<VertexId> {
    let corners: std::collections::HashSet<VertexId> = corner_band.iter().copied().collect();
    let mut band: BTreeSet<VertexId> = corner_band.iter().copied().collect();
    // `as TetId` is the Mesh-trait API tax (`n_tets()` is usize, `tet_vertices` takes u32);
    // tet counts stay far below u32::MAX.
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = tet10.n_tets() as TetId;
    for t in 0..n_tets {
        let c = tet10.tet_vertices(t);
        // A `Tet10Mesh` always surfaces midsides; skip rather than unwrap so this stays a
        // total function over the `Mesh` trait (grade Safety: no `unwrap` in library code).
        let Some(m) = tet10.tet_midside_nodes(t) else {
            continue;
        };
        for (slot, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            if corners.contains(&c[a]) && corners.contains(&c[b]) {
                band.insert(m[slot]);
            }
        }
    }
    band.into_iter().collect()
}

/// One captured flexion pose: the imposed angle, the disc's deformed surface (native
/// mm), and the small-angle restoring response + conservation residual at that pose.
pub struct FlexionFrame {
    /// The imposed flexion angle about the ML axis (rad).
    pub theta: f64,
    /// The disc's deformed tet-vertex positions in **native millimetres**, ready to
    /// pair with [`FlexionTrajectory::boundary_faces`] for the deformed surface.
    pub deformed_nodes_native: Vec<Point3<f64>>,
    /// The superior-endplate restoring moment about the disc centre, projected on the
    /// ML axis (N·m) — negative for a restoring response (see [`BondedDisc::flexion_moment`]).
    pub moment: f64,
    /// The bond's conservation residual `‖ΣF‖ + ‖ΣM‖` over both endplates (≈ 0).
    pub conservation_resid: f64,
}

/// A replayable capture of a [`BondedDisc`] flexion sweep.
///
/// Holds the shared pivot + axis, the disc's rest surface, its (deformation-invariant)
/// boundary triangulation, and one [`FlexionFrame`] per swept angle — everything a
/// viewer needs to replay the disc deforming while the superior vertebra rotates about
/// `(pivot, axis, theta)`.
///
/// All positions are **native millimetres** (the vertebra/ligament/facet frame), so a
/// renderer never touches the solver's SI frame. Produced by [`BondedDisc::capture_flexion`].
pub struct FlexionTrajectory {
    /// The flexion pivot — the disc AABB centre in native mm ([`BondedDisc::center_native`]).
    pub pivot: Point3<f64>,
    /// The flexion axis — the disc ML unit vector ([`BondedDisc::ml_axis`]).
    pub axis: Vector3<f64>,
    /// The disc's rest (θ = 0 equilibrium) tet-vertex positions in native mm — the
    /// reference a viewer exaggerates deformation against.
    pub rest_nodes_native: Vec<Point3<f64>>,
    /// The disc surface triangulation, indexing into every frame's
    /// `deformed_nodes_native` (constant across the sweep; see [`BondedDisc::boundary_faces`]).
    pub boundary_faces: Vec<[VertexId; 3]>,
    /// The captured poses, one per swept angle.
    pub frames: Vec<FlexionFrame>,
}

impl<Msh: Mesh, E: Element<N, G> + Default, const N: usize, const G: usize>
    BondedDisc<Msh, E, N, G>
{
    /// The medio-lateral (flexion/extension) axis — a coordinate unit vector,
    /// identical in the solver SI frame and native mm (the map is translate + scale).
    #[must_use]
    pub const fn ml_axis(&self) -> Vector3<f64> {
        self.ml_axis
    }

    /// The shared flexion pivot: the disc's AABB centre in **native millimetres**.
    #[must_use]
    pub const fn center_native(&self) -> Point3<f64> {
        self.center_native
    }

    /// The disc's current deformed tet-vertex positions mapped back to **native
    /// millimetres** (`center_native + p_si / scale`), inverting the recentre + scale
    /// applied at build. Reads the last solved configuration, so call it after
    /// [`Self::set_flexion`] / [`Self::flexion_moment`] to read that pose's surface.
    ///
    /// Pairs with [`Self::boundary_faces`] to build the deformed disc surface in the
    /// same native-mm frame as the vertebrae, ligaments, and facets.
    #[must_use]
    pub fn deformed_nodes_native(&self) -> Vec<Point3<f64>> {
        let x = self.sandwich.soft_positions();
        (0..x.len() / 3)
            .map(|i| {
                let p_si = Vector3::new(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
                self.center_native + p_si / self.scale
            })
            .collect()
    }

    /// The disc surface triangulation (outward-oriented boundary faces), indexing into
    /// [`Self::deformed_nodes_native`]. Constant across a flexion sweep — only the vertex
    /// positions deform — so a viewer snapshots it once and rebuilds each frame's mesh.
    ///
    /// ⚠ **Corner-only for higher-order elements.** These are 3-vertex triangles over
    /// *corner* nodes. On a quadratic disc, [`Self::deformed_nodes_native`] returns every
    /// node (corners *and* midsides) while this triangulation still references corners
    /// alone — the indices stay valid (enrichment preserves the corner id-space and keeps
    /// it first), but the rendered surface silently reflects the **linear** field and
    /// omits the quadratic enrichment. Nothing breaks; the picture is just less curved
    /// than the physics. Skinning against the 6-node boundary faces is the named viz
    /// follow-on in `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §6 — until it lands, do
    /// not read a Tet10 disc's rendered surface as `rendered === contacts`.
    ///
    /// ⚠ Rung 3 sharpened this from cosmetic to real: on a curved disc the bonded-face
    /// midsides carry the seating onto the bone, so a corner-only surface now omits
    /// geometry that exists rather than only a smoother interpolation of geometry it has.
    #[must_use]
    pub fn boundary_faces(&self) -> &[[VertexId; 3]] {
        self.sandwich.soft_boundary_faces()
    }

    /// Sweep a sequence of flexion `angles` (rad) and record a replayable
    /// [`FlexionTrajectory`]: the shared pivot/axis, the rest surface, the boundary
    /// triangulation, and one [`FlexionFrame`] per angle (deformed surface + restoring
    /// moment + conservation residual). On return the disc is left at the last swept
    /// angle; the recorded rest surface is independent of that (see below).
    ///
    /// The rest surface is the θ = 0 equilibrium, solved first so it is independent of any
    /// prior drive state. Each frame is a single quasi-static solve (via
    /// [`Self::flexion_moment`]), then the deformed surface is read back in native mm — so
    /// a capture costs exactly `N + 1` solves (rest + one per angle).
    ///
    /// # Panics
    /// Panics if any angle drives the soft solve past its SPD region — see
    /// [`Self::set_flexion`]. Keep every `|angle|` inside the validated sub-degree range.
    #[must_use]
    pub fn capture_flexion(&mut self, angles: &[f64]) -> FlexionTrajectory {
        // Rest = the θ = 0 equilibrium, solved up front so it does not depend on whatever
        // pose a prior caller left the disc in.
        self.set_flexion(0.0);
        let rest_nodes_native = self.deformed_nodes_native();
        let boundary_faces = self.boundary_faces().to_vec();

        let frames = angles
            .iter()
            .map(|&theta| {
                let (moment, conservation_resid) = self.flexion_moment(theta);
                FlexionFrame {
                    theta,
                    deformed_nodes_native: self.deformed_nodes_native(),
                    moment,
                    conservation_resid,
                }
            })
            .collect();

        FlexionTrajectory {
            pivot: self.center_native,
            axis: self.ml_axis,
            rest_nodes_native,
            boundary_faces,
            frames,
        }
    }

    /// Element validity at the disc's **current deformed** configuration: the worst
    /// `detJ / detJ_rest` over every element and every Gauss point.
    ///
    /// `> 0` means no element has folded at the pose the last
    /// [`Self::set_flexion`] solved; `≤ 0` means one has. This is a **different invariant**
    /// from the quality floors `DISC_CONFORM_QUALITY_FLOOR` /
    /// `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR` enforce, which bound the *rest* mesh the
    /// projectors produce and say nothing about what a drive does to it. Nothing in this
    /// arc checked the deformed configuration before rung 4.
    #[must_use]
    pub fn min_jacobian_ratio(&self) -> f64 {
        self.sandwich.min_gauss_det_ratio()
    }

    /// Impose flexion angle `theta` (rad) about the ML axis through the disc centre
    /// and re-solve the quasi-static bond: the superior endplate box rotates about the
    /// SI-frame origin (the bonded face follows rigidly), the inferior stays at rest.
    ///
    /// Keep `|theta|` sub-degree — the bond converges only while the boundary tets stay
    /// in their SPD region.
    ///
    /// # Panics
    /// Panics if the quasi-static soft solve fails to converge — a `|theta|` large
    /// enough to drive the boundary tets out of their SPD region (beyond ~1°) will
    /// exceed the Newton iteration cap and abort.
    pub fn set_flexion(&mut self, theta: f64) {
        let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(self.ml_axis), theta);
        self.sandwich
            .set_body_pose(UPPER, rot * self.rest_upper, rot);
        self.sandwich.probe();
    }

    /// Impose flexion `theta` (rad) and measure the disc's restoring response: the
    /// reaction moment on the superior endplate about the disc centre projected on the
    /// ML axis (N·m), plus a conservation residual `‖ΣF‖ + ‖ΣM‖` over both bonded faces
    /// (≈ 0 for a self-equilibrated field — the rung-6 oracle).
    ///
    /// # Panics
    /// Panics if the soft solve diverges — see [`Self::set_flexion`]; keep `|theta|`
    /// sub-degree.
    pub fn flexion_moment(&mut self, theta: f64) -> (f64, f64) {
        self.set_flexion(theta);
        let react = self.sandwich.last_reaction();
        let targets = self.sandwich.last_targets();
        let at = |i: usize, s: &[f64]| Vec3::new(s[3 * i], s[3 * i + 1], s[3 * i + 2]);
        // Moment on the superior vertebra about the origin (disc centre).
        let mut m_up = Vec3::zeros();
        for &v in self.sandwich.upper_face() {
            let i = v as usize;
            m_up += at(i, targets).cross(&at(i, react));
        }
        // Conservation over BOTH faces (should vanish).
        let (mut f_tot, mut m_tot) = (Vec3::zeros(), Vec3::zeros());
        for &v in self
            .sandwich
            .lower_face()
            .iter()
            .chain(self.sandwich.upper_face())
        {
            let i = v as usize;
            let f = at(i, react);
            f_tot += f;
            m_tot += at(i, targets).cross(&f);
        }
        (m_up.dot(&self.ml_axis), f_tot.norm() + m_tot.norm())
    }
}

/// License-free geometry fixtures shared by this crate's test modules (`lib.rs` and
/// `coupled.rs`), so the box triangulation lives in exactly one place.
#[cfg(test)]
pub(crate) mod test_support {
    use nalgebra::{Point3, Vector3};

    use crate::IndexedMesh;

    /// A watertight axis-aligned box (8 verts, 12 outward-wound triangles) with the given
    /// half-extents, centred at `center` — a closed surface the signed oracle can sign,
    /// standing in for the disc (thinnest extent = a disc-like endplate gap).
    #[must_use]
    pub fn box_mesh(center: Point3<f64>, half: Vector3<f64>) -> IndexedMesh {
        let c = center;
        let (hx, hy, hz) = (half.x, half.y, half.z);
        let vertices = vec![
            Point3::new(c.x - hx, c.y - hy, c.z - hz),
            Point3::new(c.x + hx, c.y - hy, c.z - hz),
            Point3::new(c.x + hx, c.y + hy, c.z - hz),
            Point3::new(c.x - hx, c.y + hy, c.z - hz),
            Point3::new(c.x - hx, c.y - hy, c.z + hz),
            Point3::new(c.x + hx, c.y - hy, c.z + hz),
            Point3::new(c.x + hx, c.y + hy, c.z + hz),
            Point3::new(c.x - hx, c.y + hy, c.z + hz),
        ];
        let faces = vec![
            [0, 3, 2],
            [0, 2, 1], // z-
            [4, 5, 6],
            [4, 6, 7], // z+
            [0, 1, 5],
            [0, 5, 4], // y-
            [2, 3, 7],
            [2, 7, 6], // y+
            [0, 4, 7],
            [0, 7, 3], // x-
            [1, 2, 6],
            [1, 6, 5], // x+
        ];
        IndexedMesh { vertices, faces }
    }

    /// A watertight lat-lon sphere (outward-wound), standing in for a **curved** endplate.
    ///
    /// The box fixture above is planar, and a planar "endplate" cannot exercise the rung-3
    /// midside projection at all: a straight midside between two seated corners lies exactly
    /// on a plane through them, so its residual is already zero and there is nothing to
    /// project. A sphere placed tangent to a disc face gives the one property the box cannot
    /// — a chord gap between seated corners — license-free, so CI can gate it.
    ///
    /// `n_lat` stacks × `n_lon` sectors; poles are fans, the rest quads split into two
    /// triangles. Winding is outward (the oracle's pseudo-normal sign convention).
    #[must_use]
    pub fn sphere_mesh(
        center: Point3<f64>,
        radius: f64,
        n_lat: usize,
        n_lon: usize,
    ) -> IndexedMesh {
        assert!(n_lat >= 2 && n_lon >= 3, "degenerate sphere tessellation");
        let mut vertices = vec![Point3::new(center.x, center.y, center.z + radius)]; // north pole
        for i in 1..n_lat {
            // Interior stacks, from the north pole down.
            #[allow(clippy::cast_precision_loss)] // small loop counters
            let theta = std::f64::consts::PI * (i as f64) / (n_lat as f64);
            let (st, ct) = theta.sin_cos();
            for j in 0..n_lon {
                #[allow(clippy::cast_precision_loss)]
                let phi = 2.0 * std::f64::consts::PI * (j as f64) / (n_lon as f64);
                let (sp, cp) = phi.sin_cos();
                vertices.push(Point3::new(
                    center.x + radius * st * cp,
                    center.y + radius * st * sp,
                    center.z + radius * ct,
                ));
            }
        }
        vertices.push(Point3::new(center.x, center.y, center.z - radius)); // south pole
        // Vertex ids are `(n_lat - 1) * n_lon + 2` at most — far below `u32::MAX` for any
        // tessellation a test would ask for (`IndexedMesh` indexes faces with `u32`).
        #[allow(clippy::cast_possible_truncation)]
        let south = (vertices.len() - 1) as u32;
        // Ring `i` (1-based interior stack) starts at vertex `1 + (i - 1) * n_lon`.
        #[allow(clippy::cast_possible_truncation)]
        let ring = |i: usize, j: usize| (1 + (i - 1) * n_lon + (j % n_lon)) as u32;

        let mut faces = Vec::new();
        for j in 0..n_lon {
            faces.push([0, ring(1, j), ring(1, j + 1)]);
        }
        for i in 1..(n_lat - 1) {
            for j in 0..n_lon {
                let (a, b) = (ring(i, j), ring(i, j + 1));
                let (c, d) = (ring(i + 1, j), ring(i + 1, j + 1));
                faces.push([a, c, d]);
                faces.push([a, d, b]);
            }
        }
        for j in 0..n_lon {
            faces.push([south, ring(n_lat - 1, j + 1), ring(n_lat - 1, j)]);
        }
        IndexedMesh { vertices, faces }
    }

    /// A disc-like synthetic slab: widest in x (ML), thinnest in z (SI), placed at a
    /// native-mm-scale offset so recentring + scaling are exercised.
    #[must_use]
    pub fn synthetic_disc() -> IndexedMesh {
        box_mesh(
            Point3::new(100.0, 100.0, 950.0),
            Vector3::new(12.0, 10.0, 3.0), // 24 × 20 × 6 mm — x widest, z thinnest
        )
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

    use cf_geometry::Sdf;

    use nalgebra::SMatrix;

    use super::test_support::{box_mesh, sphere_mesh, synthetic_disc};
    use super::*;

    #[test]
    fn builds_and_derives_the_ml_axis_and_pivot() {
        let disc = build_bonded_disc(synthetic_disc(), &DiscParams::default(), None).unwrap();
        // ML = widest extent = native x; pivot = the AABB centre in native mm.
        assert_eq!(
            disc.ml_axis(),
            Vector3::x(),
            "widest extent (x) is the ML axis"
        );
        assert!(
            (disc.center_native() - Point3::new(100.0, 100.0, 950.0)).norm() < 1e-9,
            "pivot is the native-mm AABB centre, got {:?}",
            disc.center_native()
        );
    }

    /// Rough endplate face selection: the down-facing lower region (`sign = -1`,
    /// L4 inferior) or up-facing upper region (`sign = +1`, L5 superior).
    fn select_endplate(mesh: &IndexedMesh, sign: f64) -> Vec<usize> {
        let bbox = Aabb::from_points(mesh.vertices.iter());
        let zmid = 0.5 * (bbox.min.z + bbox.max.z);
        let mut ids = Vec::new();
        for i in 0..mesh.faces.len() {
            let Some(tri) = mesh.triangle(i) else {
                continue;
            };
            let centroid = Point3::from((tri.v0.coords + tri.v1.coords + tri.v2.coords) / 3.0);
            let normal = (tri.v1 - tri.v0).cross(&(tri.v2 - tri.v0));
            let norm = normal.norm();
            if norm < f64::EPSILON {
                continue;
            }
            let region = if sign < 0.0 {
                centroid.z < zmid
            } else {
                centroid.z > zmid
            };
            if region && (normal.z / norm) * sign > 0.7 {
                ids.push(i);
            }
        }
        ids
    }

    /// Loft a disc from rough auto-selected L4/L5 endplate patches (the painting
    /// GUI does this cleaner).
    fn lofted_disc(l4: &IndexedMesh, l5: &IndexedMesh) -> IndexedMesh {
        use mesh_loft::{
            WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch,
        };
        let l4_faces = select_endplate(l4, -1.0);
        let l5_faces = select_endplate(l5, 1.0);
        assert!(
            !l4_faces.is_empty() && !l5_faces.is_empty(),
            "empty endplate selection"
        );
        // Prepare each patch (largest component, interior holes sealed → one outer
        // rim) so the loft meets assemble_bushing's single-boundary precondition.
        let top = finalize_patch(&extract_patch(l4, &l4_faces));
        let bottom = finalize_patch(&extract_patch(l5, &l5_faces));
        let top = flip_patch(&top);
        // `finalize_patch` already guarantees one connected component with its
        // interior holes sealed; the assembled disc may still carry a few open
        // wall-seam edges (the arc-length correspondence on these dissimilar
        // auto-selected rims), which the SDF tet-mesher resamples away — so we do
        // NOT require strict watertightness here (measured: 50 open edges / 0
        // non-manifold on the real L4/L5, and it tet-meshes, bonds, and sweeps).
        assemble_bushing(&top, &bottom, 1, WallCorrespondence::ArcLength).mesh
    }

    /// B6 end-to-end: a human-lofted disc **tet-meshes, bonds to a restoring
    /// stiffness, seats on the exact bone for free, and sweeps** — the payoff of
    /// building the disc *from* the endplates. The scanned disc could do none of
    /// these (it over-stretched 2.15× and its conform shifted `k_disc` ~6×). The
    /// bond + sweep succeeding is itself proof the tet mesh is clean — inverted or
    /// fragmented tets diverge (spike-measured: 99% kept, 11.8° min dihedral, 0
    /// inverted).
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn b6_lofted_disc_bonds_seats_and_sweeps() {
        use cf_fsu_geometry::{conform_disc_to_endplates, load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let disc = lofted_disc(&l4, &l5);
        let params = DiscParams::default();

        // (1) The caps ARE the endplate surfaces: conforming them onto the exact
        // bone is a sub-mm move (the scanned disc needed multi-mm, stiffening moves).
        let conformed = conform_disc_to_endplates(&disc, &o4, &o5, &frame, Some(params.band_frac));
        let max_move = disc
            .vertices
            .iter()
            .zip(&conformed.vertices)
            .map(|(a, b)| (a - b).norm())
            .fold(0.0, f64::max);
        println!("conform: max cap-band move {max_move:.2} mm");
        assert!(
            max_move < 4.0,
            "cap band not seated on the bone ({max_move:.2} mm)"
        );

        // (2) Bonds + probes to a restoring, symmetric, self-equilibrated k_disc,
        // and the exact-bone conform leaves it intact (no ~6× shift).
        let theta = 0.5_f64.to_radians();
        let mut raw_bond = build_bonded_disc(disc, &params, None).expect("lofted disc bonds");
        let (m_flex, resid_flex) = raw_bond.flexion_moment(theta);
        let (m_ext, resid_ext) = raw_bond.flexion_moment(-theta);
        let (k_flex, k_ext) = (m_flex / theta, m_ext / -theta);
        let k_conformed = build_bonded_disc(conformed, &params, None)
            .expect("conformed disc bonds")
            .flexion_moment(theta)
            .0
            / theta;
        println!(
            "k_disc: flex {k_flex:.1}, ext {k_ext:.1}, conformed {k_conformed:.1} N·m/rad; resid {resid_flex:.1e} / {resid_ext:.1e}"
        );
        assert!(k_flex.is_finite() && k_ext.is_finite(), "non-finite k_disc");
        assert!(
            k_flex * k_ext > 0.0 && k_flex.abs() > 1.0,
            "not a consistent linear spring ({k_flex:.1} vs {k_ext:.1})"
        );
        assert!(
            resid_flex < 1e-2 && resid_ext < 1e-2,
            "bond not self-equilibrated ({resid_flex:.2e} / {resid_ext:.2e})"
        );
        assert!(
            (k_conformed - k_flex).abs() < 0.5 * k_flex.abs(),
            "exact-bone conform shifted k_disc ({k_flex:.1} -> {k_conformed:.1})"
        );

        // (3) A sub-degree sweep (incremental, warm-started) genuinely deforms and
        // restores across angles — conservation + strictly-restoring moment per
        // frame + a real imposed deformation (assert_restoring_sweep, shared with
        // the synthetic and scanned-disc capture tests).
        let angles: Vec<f64> = [-0.5_f64, -0.25, 0.0, 0.25, 0.5]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = raw_bond.capture_flexion(&angles);
        let max_disp = assert_restoring_sweep(&traj, 2e-2);
        println!(
            "sweep: {} frames, max node displacement {max_disp:.3} mm",
            traj.frames.len()
        );

        // ── (4) The **Strategy-B node conform** on a lofted disc, DRIVEN, on both elements. ──
        //
        // ★ This arm exists because its absence cost rung 4 a shipped panic. Everything above
        // exercises the lofted disc either RAW (`None`) or against a Strategy-A *surface*
        // conform — so the node conform, which is what `CoupledFsu` actually uses, had no
        // lofted coverage at all. At `DISC_CONFORM_QUALITY_FLOOR = 0.05` it produced a mesh
        // that is valid by the projector's rule and undrivable: the coupled build's 0.86°
        // probe died on a fail-closed validity violation, on BOTH elements. The element was
        // innocent; the floor was wrong for this discretization (see that constant).
        //
        // So the gate DRIVES rather than inspects, and it drives past `K_DISC_PROBE` (0.86°)
        // because that is the first thing production asks of a conformed disc. A geometric
        // check here would have gone green on the mesh that panicked.
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };
        let disc = lofted_disc(&l4, &l5); // rebuilt: the arms above consumed theirs
        let probe = 0.9_f64.to_radians();
        let steps = 9;
        let walk = |k: f64| {
            assert!(
                k.is_finite() && k < 0.0,
                "the conformed lofted disc must reach {:.2}° and stay restoring, got k = {k}",
                probe.to_degrees()
            );
            k
        };
        let mut lin = build_bonded_disc(disc.clone(), &params, Some(ep))
            .expect("lofted disc takes the node conform");
        let mut quad = build_bonded_disc_tet10(disc, &params, Some(ep))
            .expect("lofted disc takes the curved node conform");
        let (mut k_lin, mut k_quad) = (0.0, 0.0);
        for s in 1..=steps {
            let t = probe * f64::from(s) / f64::from(steps);
            k_lin = lin.flexion_moment(t).0 / t;
            k_quad = quad.flexion_moment(t).0 / t;
        }
        println!(
            "lofted + node conform, walked to {:.2}°: Tet4 k {:.1}, curved Tet10 k {:.1} N·m/rad",
            probe.to_degrees(),
            walk(k_lin),
            walk(k_quad)
        );
    }

    /// B6.4 — the capstone: a painted (lofted) disc drops into the FULL coupled
    /// FSU (disc bushing + ligaments + facet contact) and produces a physically
    /// sound, monotone, restoring, facet-stopped segmental response — the same
    /// assembly rung 7 validates against literature, now driven by a human-painted
    /// disc.
    ///
    /// This validates the **integration** (painted disc → full FSU → physiologic
    /// response), not the tight literature ROM band: the rough *auto-lofted* disc
    /// is softer in flexion than the scanned disc (a different centre/geometry
    /// shifts the ligament lever arms), so its flexion exits the ROM bracket
    /// before the full 7.5 N·m. We sweep the moment range where equilibria exist;
    /// a carefully painted disc matching the real geometry is what would hit the
    /// literature band.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn b6_4_coupled_fsu_from_a_lofted_disc() {
        use cf_fsu_geometry::load_from_env;

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc = lofted_disc(&l4, &l5);
        let mut fsu = CoupledFsu::build(&l4, &l5, &disc, &CoupledParams::default())
            .expect("coupled FSU builds with a lofted disc");
        println!("coupled FSU: k_disc {:.3} N·m/rad", fsu.k_disc());

        // Force-driven equilibrium sweep over the in-bracket moment range.
        let ramp: Vec<f64> = (0..=18).map(|i| -6.0 + f64::from(i) * 0.5).collect();
        let traj = fsu
            .capture_ramp(&ramp)
            .expect("coupled equilibria exist within the ROM bracket");

        // Monotone extension → flexion, a physiologic few-degrees ROM each way.
        for w in traj.frames.windows(2) {
            assert!(
                w[1].theta >= w[0].theta - 1e-9,
                "equilibrium angle must rise monotonically with the applied moment"
            );
        }
        let ext_deg = traj.frames.first().unwrap().theta.to_degrees();
        let flex_deg = traj.frames.last().unwrap().theta.to_degrees();
        println!("ROM: extension {ext_deg:.1}° … flexion {flex_deg:.1}° (in-bracket sweep)");
        assert!(ext_deg < 0.0 && flex_deg > 0.0, "wrong flexion sense");

        // The facets stop extension and open in flexion — the ROM is contact-limited,
        // not just spring-limited.
        assert!(
            !traj.frames.first().unwrap().facet_points.is_empty(),
            "facets must engage at the extension peak"
        );
        assert!(
            traj.frames.last().unwrap().facet_points.is_empty(),
            "facets must open at the flexion peak"
        );
    }

    #[test]
    fn ml_axis_is_widest_aabb_extent_not_pca() {
        // The flexion axis must be the widest AXIS-ALIGNED extent, NOT a PCA principal
        // direction. This cloud's longest point-to-point direction is the x-y diagonal, but
        // its widest axis-aligned extent is x (span 10 vs y-span 3) → +x. A regression of
        // `ml_axis_from_points` to a PCA axis would silently rotate the whole segment about
        // the wrong axis on a real oblique disc, with no other test catching it.
        let pts = [Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 3.0, 1.0)];
        assert_eq!(ml_axis_from_points(&pts), Vector3::x());
    }

    #[test]
    fn ml_axis_breaks_extent_ties_by_last_max_not_longest_axis() {
        // The deliberate `max_by` (LAST maximum) tie-break vs `Aabb::longest_axis` (FIRST
        // maximum): x and y extents TIED at the widest must resolve to the last (y). A
        // regression to `longest_axis()` would flip a square disc (a defect the PR-A gating
        // review caught once).
        let pts = [
            Point3::new(-10.0, -10.0, -3.0),
            Point3::new(10.0, 10.0, 3.0),
        ];
        assert_eq!(ml_axis_from_points(&pts), Vector3::y());
    }

    #[test]
    fn flexion_is_restoring_conserving_and_antisymmetric() {
        let mut disc = build_bonded_disc(synthetic_disc(), &DiscParams::default(), None).unwrap();
        let theta = 0.3_f64.to_radians(); // sub-degree: stay in the SPD region

        let (m_pos, resid_pos) = disc.flexion_moment(theta);
        let (m_neg, resid_neg) = disc.flexion_moment(-theta);

        // Conservation: the bond's reaction is a self-equilibrated wrench.
        assert!(
            resid_pos < 1e-9 && resid_neg < 1e-9,
            "bond must conserve (residuals {resid_pos:.2e}, {resid_neg:.2e})"
        );
        // Restoring: a +θ tilt is opposed by a −ML moment on the superior plate.
        assert!(
            m_pos < 0.0,
            "flexion must be restoring (M·ML < 0), got {m_pos:+.3e}"
        );
        // Small-strain linearity ⇒ the response is antisymmetric in θ.
        let asym = (m_pos + m_neg).abs() / m_pos.abs().max(1e-12);
        assert!(
            asym < 0.1,
            "small-strain response should be antisymmetric (asymmetry {asym:.3})"
        );
    }

    #[test]
    fn rejects_a_misoriented_disc() {
        // Thinnest extent along x (not z) — the SI-axis guard must reject it.
        let bad = box_mesh(Point3::origin(), Vector3::new(3.0, 10.0, 12.0));
        let Err(err) = build_bonded_disc(bad, &DiscParams::default(), None) else {
            panic!("expected the SI-orientation guard to reject a mis-oriented disc");
        };
        assert!(
            format!("{err}").contains("mis-oriented"),
            "expected the SI-orientation guard to fire, got: {err}"
        );
    }

    /// Assert a captured sweep is physically sound and return its max node displacement
    /// (native mm). Shared by the synthetic and real-disc capture tests so the flexion
    /// invariant lives in ONE place: a valid deformation-invariant boundary surface,
    /// per-frame conservation, a strictly restoring moment off neutral / a vanishing one
    /// at neutral, and a real imposed deformation of at least `min_disp` mm. `min_disp` is
    /// passed per-test, tied to that disc's geometry (`extent·sin θ`) and set comfortably
    /// above the ~0.02 mm interior-node solve-noise floor so it tests imposed deformation,
    /// not noise.
    fn assert_restoring_sweep(traj: &FlexionTrajectory, min_disp: f64) -> f64 {
        let n = traj.rest_nodes_native.len();
        assert!(!traj.boundary_faces.is_empty(), "disc must have a surface");
        assert!(
            traj.boundary_faces
                .iter()
                .flatten()
                .all(|&v| (v as usize) < n),
            "every boundary-face vertex must index into the {n}-vertex node buffer"
        );
        // Scale for the neutral-frame check: the largest loaded restoring moment.
        let moment_scale = traj
            .frames
            .iter()
            .map(|f| f.moment.abs())
            .fold(0.0_f64, f64::max);
        let mut max_disp = 0.0_f64;
        for f in &traj.frames {
            assert_eq!(f.deformed_nodes_native.len(), n, "node count constant");
            assert!(
                f.conservation_resid < 1e-8,
                "θ={:.3}° bond must conserve (resid {:.2e})",
                f.theta.to_degrees(),
                f.conservation_resid
            );
            if f.theta.abs() > 1e-9 {
                // Loaded: the ML moment strictly opposes the tilt (θ·M < 0). A `≤ ε` bound
                // would pass a spurious M = 0, so require a strictly restoring sign.
                assert!(
                    f.theta * f.moment < 0.0,
                    "θ={:.3}° must be restoring (θ·M = {:.2e})",
                    f.theta.to_degrees(),
                    f.theta * f.moment
                );
            } else {
                // Neutral: the response must vanish (checked explicitly — `0·M ≤ ε` is
                // vacuously true and would never test the θ=0 moment).
                assert!(
                    f.moment.abs() < 0.1 * moment_scale,
                    "neutral moment must be ~0, got {:.2e} (scale {moment_scale:.2e})",
                    f.moment
                );
            }
            for (p, r) in f.deformed_nodes_native.iter().zip(&traj.rest_nodes_native) {
                max_disp = max_disp.max((p - r).norm());
            }
        }
        assert!(
            max_disp > min_disp,
            "the sweep must impose a real deformation (≥ {min_disp:.2e} mm, above solve noise); got {max_disp:.2e} mm"
        );
        max_disp
    }

    #[test]
    fn capture_flexion_records_a_deforming_restoring_sweep() {
        // License-free coverage of the capture seam on the synthetic disc: exercises
        // capture_flexion + deformed_nodes_native + boundary_faces (the real-anatomy
        // build+measure gate is the #[ignore]d test below).
        let mut disc = build_bonded_disc(synthetic_disc(), &DiscParams::default(), None).unwrap();
        let angles: Vec<f64> = [-0.3_f64, 0.0, 0.3]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = disc.capture_flexion(&angles);

        // Shared pivot/axis + the direct accessors agree with the trajectory snapshot.
        assert_eq!(traj.pivot, disc.center_native());
        assert_eq!(traj.axis, disc.ml_axis());
        assert_eq!(disc.boundary_faces(), traj.boundary_faces.as_slice());
        assert_eq!(
            disc.deformed_nodes_native().len(),
            traj.rest_nodes_native.len()
        );

        // The 24×20×6 mm slab tilted 0.3° about its ML(x) axis moves its farthest node
        // (~10 mm off-axis) by ~10·sin(0.3°) ≈ 0.05 mm; require ≥ 0.02 mm (above noise).
        assert_restoring_sweep(&traj, 2e-2);
    }

    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn captures_a_replayable_flexion_sweep_on_the_real_disc() {
        // Build+measure on the real intervertebral disc: capture a validated sub-degree
        // sweep and assert the trajectory a viewer will replay is physically sound and
        // deterministic — no proxy. Gated + license-clean like the rung tests.
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let mut disc =
            build_bonded_disc(disc_mesh, &DiscParams::default(), None).expect("build disc");

        // Symmetric, all within rung-7's validated SPD range (|θ| ≤ 0.86°).
        let angles: Vec<f64> = [-0.86_f64, -0.5, 0.0, 0.5, 0.86]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = disc.capture_flexion(&angles);

        // Shared pivot/axis are the disc's own.
        assert_eq!(traj.pivot, disc.center_native());
        assert_eq!(traj.axis, disc.ml_axis());

        assert_eq!(traj.frames.len(), angles.len());

        // Physics (shared with the synthetic test): valid surface, conservation, restoring
        // moment, real deformation. The ±0.86° sweep on the real disc deforms it ~0.4 mm;
        // require ≥ 0.1 mm — well above the ~0.02 mm interior-node solve-noise floor.
        let max_disp = assert_restoring_sweep(&traj, 1e-1);
        // The disc ACTUALLY deforms (sub-mm at sub-degree — the viewer exaggerates it).
        println!(
            "captured {} frames; max node displacement {max_disp:.4} mm",
            traj.frames.len()
        );

        // Reproducibility — asserted on the PHYSICAL OBSERVABLE (the restoring moment),
        // not per-node bit-identity. `build_bonded_disc` now drops the mesher's
        // disconnected rim islands (`SdfMeshedTetMesh::largest_component`), which were the
        // dominant source of the near-singular Newton tangent (floating tet components =
        // unconstrained rigid modes): the faer LU fallback count fell from ~17 to ~4 on
        // the real disc. A small residual remains (near-sliver tets within the main body),
        // so the multi-threaded indefinite LU can still land on slightly different interior
        // configurations run-to-run. The bonded (boundary) nodes are Dirichlet-pinned, so
        // the reaction — hence the moment — is well-determined regardless. We assert the
        // observable a regression would actually track. (Fully eliminating the residual
        // needs a thin-feature-capable mesher — a separate sim-soft rung.)
        //
        // Drift is the absolute moment change normalised by the sweep's peak moment, NOT by
        // each frame's own moment: the near-zero θ=0 frame would make a per-frame ratio blow up.
        let moment_scale = traj
            .frames
            .iter()
            .map(|f| f.moment.abs())
            .fold(0.0_f64, f64::max);
        let again = disc.capture_flexion(&angles);
        let mut max_moment_drift = 0.0_f64;
        for (a, b) in traj.frames.iter().zip(&again.frames) {
            max_moment_drift = max_moment_drift.max((a.moment - b.moment).abs());
        }
        let moment_rel = max_moment_drift / moment_scale.max(1e-12);
        println!("re-capture moment drift: {moment_rel:.2e} rel (scale {moment_scale:.2e})");
        assert!(
            moment_rel < 1e-3,
            "the restoring moment must reproduce across captures (drift {moment_rel:.2e} rel)"
        );
    }

    /// License-free coverage of the endplate-conform wiring (`EndplateConform` →
    /// `endplate_conform_moves` → `SdfMeshedTetMesh::with_projected_nodes`): two box "endplates"
    /// straddling the synthetic disc pull its bonded-band faces onto their inner surfaces, and the
    /// conformed disc still bonds and sweeps soundly while its rest surface differs from the raw
    /// build (the conform engaged). The real-anatomy moment-rotation FOM is the `#[ignore]`d gate
    /// below.
    #[test]
    fn endplate_conform_seats_the_band_and_still_sweeps() {
        // Disc: centre (100,100,950), half (12,10,3) → z ∈ [947, 953], SI = z. The two box
        // "endplates" sit at DELIBERATELY ASYMMETRIC gaps from the disc faces (superior 0.5 mm,
        // inferior 2 mm) so the frame-bridge gate below has teeth: a bridge that collapses every
        // node toward the disc centre would seat them all to the NEARER (superior) face, which the
        // asymmetry exposes (the far inferior face is left empty).
        let o_sup = oracle(&box_mesh(
            Point3::new(100.0, 100.0, 955.0),
            Vector3::new(20.0, 20.0, 1.5), // bottom face z = 953.5, 0.5 mm above the disc top (953)
        ))
        .unwrap();
        let o_inf = oracle(&box_mesh(
            Point3::new(100.0, 100.0, 943.0),
            Vector3::new(20.0, 20.0, 2.0), // top face z = 945.0, 2 mm below the disc bottom (947)
        ))
        .unwrap();
        let params = DiscParams::default();
        let endplates = EndplateConform {
            o4: &o_sup,
            o5: &o_inf,
            superior_axis: Vector3::z(),
        };

        let mut conf = build_bonded_disc(synthetic_disc(), &params, Some(endplates))
            .expect("conformed synthetic disc bonds");
        let angles: Vec<f64> = [-0.3_f64, 0.0, 0.3]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = conf.capture_flexion(&angles);
        assert_restoring_sweep(&traj, 2e-2);

        // The conform engaged: the θ=0 rest surface differs from the raw (un-conformed) build,
        // node-for-node (same deterministic topology), by a real (> noise) endplate-seat move.
        let raw_rest = build_bonded_disc(synthetic_disc(), &params, None)
            .expect("raw synthetic disc bonds")
            .capture_flexion(&[0.0])
            .rest_nodes_native;
        assert_eq!(
            raw_rest.len(),
            traj.rest_nodes_native.len(),
            "same topology"
        );
        // Strongly-seated (pinned band) nodes and the native-mm z-band they land in. Aligned node
        // arrays: the raw and conformed builds share topology/ordering (deterministic mesher, same
        // input). A band node moves ≥ 0.5 mm onto its box face; the 0.4 mm cut excludes the padded-
        // lattice orphan nodes (which the conform never touches) and interior solve-relaxation.
        let (mut hi, mut lo, mut seated) = (f64::MIN, f64::MAX, 0usize);
        for (raw, conf) in raw_rest.iter().zip(&traj.rest_nodes_native) {
            if (raw - conf).norm() > 0.4 {
                hi = hi.max(conf.z);
                lo = lo.min(conf.z);
                seated += 1;
            }
        }
        assert!(
            seated > 0,
            "the endplate conform must seat at least one band node"
        );

        // FRAME-BRIDGE gate: BOTH bands must seat on their OWN box face in native mm — the superior
        // band on the superior box's bottom face (z = 953.5) and the inferior band on the inferior
        // box's top face (z = 945.0). This is the check `max_seat` cannot make: a broken SI↔native
        // bridge collapses every node toward the disc centre and (given the asymmetric gaps) seats
        // them all to the nearer superior face, so the far inferior face is left empty and `lo`
        // never reaches 945.
        assert!(
            (hi - 953.5).abs() < 0.2,
            "superior band must seat on the superior box face z ≈ 953.5, got {hi:.3}"
        );
        assert!(
            (lo - 945.0).abs() < 0.2,
            "inferior band must seat on the inferior box face z ≈ 945.0, got {lo:.3}"
        );

        // EXACT-GEOMETRY RESIDUAL, license-free (rung 2's §4.3 gate on the real anatomy is
        // `#[ignore]`d, so this is the only arm of it CI runs). Both discs are read at rest,
        // before any solve — `from_tet_mesh` initialises the solve configuration to the mesh
        // positions it snapshots the bond offsets from. Against flat box "endplates" a seated
        // node lands exactly on the face, so the conformed residual collapses to the quality
        // floor's back-off while the raw one still carries the 0.5 / 2 mm build gaps.
        let raw_disc = build_bonded_disc(synthetic_disc(), &params, None).unwrap();
        let conf_disc = build_bonded_disc(synthetic_disc(), &params, Some(endplates)).unwrap();
        let (q_raw, q_conf) = (
            raw_disc.deformed_nodes_native(),
            conf_disc.deformed_nodes_native(),
        );
        let cand = bonded_face_boundary_nodes(&conf_disc);
        let stats = |nodes: &[VertexId], p: &[Point3<f64>]| {
            residual_stats(
                &nodes
                    .iter()
                    .map(|&v| endplate_residual(p[v as usize], &o_sup, &o_inf))
                    .collect::<Vec<_>>(),
            )
        };
        let (max_raw, rms_raw) = stats(&cand, &q_raw);
        let (max_conf, rms_conf) = stats(&cand, &q_conf);
        assert!(!cand.is_empty(), "the bonded band must reach the surface");
        // The raw gaps are real (≥ the 0.5 mm superior build gap), and the conform closes them
        // to a fraction of that — asserted on both the extreme and the population statistic, so
        // a conform that seated only a few nodes would fail the RMS half.
        //
        // ★ TEETH MEASURED, not asserted, and specifically teeth NOTHING ELSE HERE HAS: a mutant
        // that seats only every other band node survives every #701 assert above — `seated > 0`
        // holds, both frame-bridge z-extremes still land on their box faces (the nodes that do
        // seat set them), and the sweep still solves — and is caught here alone, at
        // max 2.0000 -> 2.0000 (an unseated inferior node still sits its full 2 mm off) with rms
        // only halved, 1.4577 -> 1.0349. That is the partial-seat failure mode a "did anything
        // move" gate structurally cannot see.
        assert!(
            max_raw > 0.4 && rms_raw > 0.4,
            "the raw bonded face must sit off the boxes (max {max_raw:.4}, rms {rms_raw:.4} mm)"
        );
        assert!(
            max_conf < 0.1 * max_raw && rms_conf < 0.1 * rms_raw,
            "the conform must seat the bonded face on the boxes \
             (max {max_raw:.4} -> {max_conf:.4}, rms {rms_raw:.4} -> {rms_conf:.4} mm)"
        );

        // The QUADRATIC arm of the same conform. `build_bonded_disc_tet10` accepts
        // `endplates` because rung 2 measures the conform on both elements, and shipping an
        // accepted-but-unexercised argument is how a silent break gets in: everything before
        // the element choice is the same `prepare_disc` code, but enrichment reads the
        // *conformed* corner positions, so the midsides land on moved edges. Assert the
        // conformed quadratic disc bonds, still ties full-face (the band closure is a
        // topological property, unchanged by the conform), and still sweeps soundly. The
        // exact-geometry residual FOM this feeds is rung 2's gate, not this test's.
        let mut conf10 = build_bonded_disc_tet10(synthetic_disc(), &params, Some(endplates))
            .expect("conformed synthetic Tet10 disc bonds");
        let prepared = prepare_disc(synthetic_disc(), &params, Some(endplates))
            .expect("prepare conformed synthetic disc");
        let mesh10 = Tet10Mesh::from_tet4(&prepared.tet);
        assert_full_face_band(&mesh10, &prepared.inferior, conf10.sandwich.lower_face());
        assert_full_face_band(&mesh10, &prepared.superior, conf10.sandwich.upper_face());
        assert_restoring_sweep(&conf10.capture_flexion(&angles), 2e-2);
    }

    /// The worst `detJ / detJ_rest` over **every element and every Gauss point** of a curved
    /// mesh, measured against its straight twin.
    ///
    /// This is rung 3's element-validity oracle (plan §4.4), and it is deliberately *not*
    /// [`Mesh::quality`]: `Tet10Mesh` never recomputes `QualityMetrics` after a midside moves,
    /// and those metrics are four-corner quantities anyway, so `quality()` is structurally
    /// blind to midside-induced degeneracy. [`Element::rest_jacobian_dets`] is not — and it
    /// varies per Gauss point precisely when an element is curved
    /// (`sim_soft::element::tet10`'s `rest_jacobian_dets_vary_per_gauss_point_when_curved`).
    ///
    /// ⚠ **What makes this stricter than the projector's own guarantee** — the point of the
    /// gate, since "the back-off never inverts an element" is a construction guarantee and
    /// therefore untestable here. `with_projected_midsides` enforces the floor over the
    /// elements it believes are *incident* to a moved node, from an incidence map it builds by
    /// walking each tet's midside slots. This sweeps **every** element of the mesh, so an
    /// incidence map that missed an element (a wrong slot range, a midside reached through a
    /// tet the map never visited) shows up here as a sub-floor ratio, while every element the
    /// projector did check contributes exactly its guarantee.
    ///
    /// ★ Measured, not argued: narrowing that walk to `t[4..7]` drives the worst ratio to
    /// **−9.7870** on the real disc — genuinely inverted elements — while the §4.3 residual gate
    /// still *improves* (authorised RMS 0.796 → 0.711) and passes. The mutation record is in
    /// [`curved_tet10_midsides_seat_on_the_endplate_fom`].
    fn worst_gauss_det_ratio(curved: &Tet10Mesh, straight: &Tet10Mesh) -> f64 {
        let element = Tet10;
        let dets = |mesh: &Tet10Mesh, t: TetId| -> [f64; 4] {
            let corners = mesh.tet_vertices(t);
            let mids = mesh
                .tet_midside_nodes(t)
                .expect("a Tet10Mesh surfaces midsides");
            let mut nodes = [Vec3::zeros(); 10];
            for (a, &c) in corners.iter().enumerate() {
                nodes[a] = mesh.positions()[c as usize];
            }
            for (i, &m) in mids.iter().enumerate() {
                nodes[4 + i] = mesh.positions()[m as usize];
            }
            element.rest_jacobian_dets(&SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k]))
        };
        assert_eq!(
            curved.n_tets(),
            straight.n_tets(),
            "the projection preserves topology"
        );
        #[allow(clippy::cast_possible_truncation)]
        let n_tets = curved.n_tets() as TetId;
        (0..n_tets)
            .flat_map(|t| {
                let (c, s) = (dets(curved, t), dets(straight, t));
                (0..4).map(move |q| c[q] / s[q]).collect::<Vec<_>>()
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// `(delivered fraction, max move, mean move)` over a midside projection — the plan's
    /// §4.4 **coverage** statistic, in native mm.
    ///
    /// "Delivered" means the node reached its *full* projected target (`|moved − target| <
    /// 1e-9` in the solver frame), not merely that it moved. Both projection helpers back off
    /// **silently**, so a run in which most midsides quietly stayed straight would satisfy any
    /// non-vacuity-plus-max-move gate while delivering little of the geometry.
    ///
    /// ★★ **The three members catch different things, and which catches which was measured —
    /// the tidy story that "the fraction is the one with teeth" is FALSE.** The fraction sees
    /// the back-off engaging *more* (a mutant that breaks the projector's incidence map takes it
    /// 67.9 % → 49.5 %). It is blind to a mutation that changes what the projector is *asked*
    /// for, because a smaller request is easier to satisfy: a silent 0.2 mm cap on every move
    /// takes the fraction the WRONG WAY, 67.9 % → 68.6 %, and is caught by `max_move`
    /// (1.388 → 0.867) and `mean_move` (0.127 → 0.096) instead — and, first, by the §4.3
    /// residual pin. Report and gate all three; do not quote the fraction as the instrument.
    ///
    /// ⚠ Every number in this doc comment is a **mutation delta measured at the rung-3 corner
    /// floor (0.05)**, before rung 4 raised it to 0.25. Read them as the before/after PAIRS they
    /// are — what each mutant does to each statistic — not against the values committed today
    /// (67.4 %, 1.533, 0.141). Re-running the mutants at the new floor would move both sides of
    /// every arrow and change none of the conclusions.
    // Node counts are in the thousands — exact in f64.
    #[allow(clippy::cast_precision_loss)]
    fn projection_coverage(
        straight: &Tet10Mesh,
        curved: &Tet10Mesh,
        moves: &[(VertexId, Vec3)],
        scale: f64,
    ) -> (f64, f64, f64) {
        assert!(!moves.is_empty(), "an empty projection has no coverage");
        let mut delivered = 0usize;
        let (mut max_move, mut sum_move) = (0.0_f64, 0.0_f64);
        for &(v, target) in moves {
            let got = curved.positions()[v as usize];
            if (got - target).norm() < 1e-9 {
                delivered += 1;
            }
            let d = (got - straight.positions()[v as usize]).norm() / scale;
            max_move = max_move.max(d);
            sum_move += d;
        }
        (
            delivered as f64 / moves.len() as f64,
            max_move,
            sum_move / moves.len() as f64,
        )
    }

    /// Assert a midside projection touched **only** the nodes it was handed: every named node is
    /// a midside, and every other node — corners included — is bit-identical.
    ///
    /// The corner half is also guarded inside `with_projected_midsides` (it panics on a corner
    /// id); this catches the same class one level earlier, in the *selection*, where a silent
    /// mistake would be a corner never named rather than a corner wrongly named.
    fn assert_only_named_midsides_moved(
        straight: &Tet10Mesh,
        curved: &Tet10Mesh,
        moves: &[(VertexId, Vec3)],
    ) {
        let named: std::collections::HashSet<usize> =
            moves.iter().map(|&(v, _)| v as usize).collect();
        for &v in &named {
            assert!(
                v >= straight.n_corners(),
                "move names corner {v} — the corner conform runs pre-enrichment",
            );
        }
        for (v, p) in straight.positions().iter().enumerate() {
            if !named.contains(&v) {
                assert_eq!(
                    curved.positions()[v],
                    *p,
                    "unnamed node {v} must not move (corners and interior midsides stay put)",
                );
            }
        }
    }

    /// **Rung 3's license-free arm** (plan §4.4 + §4.7): the curved-midside projection gated
    /// on a *curved* synthetic endplate, so CI runs it — every real-anatomy gate in this crate
    /// is `#[ignore]`d and env-gated.
    ///
    /// The box "endplates" #701 and rung 2 use cannot exercise this rung **at all**: a straight
    /// midside between two corners seated on a plane already lies on that plane, so its
    /// residual is zero before the projection and the whole rung is a no-op against them. A
    /// sphere tangent to the disc face is the smallest fixture that produces the thing rung 3
    /// exists to close — a chord gap between seated corners — and it is license-free.
    #[test]
    fn curved_midsides_seat_on_a_curved_synthetic_endplate() {
        // Two 30 mm spheres, each tangent to a disc face (disc z ∈ [947, 953]) at the ML
        // centre, so the endplate curves away from the disc by ~2.5 mm over its 24 mm width.
        let o_sup = oracle(&sphere_mesh(Point3::new(100.0, 100.0, 983.0), 30.0, 24, 48)).unwrap();
        let o_inf = oracle(&sphere_mesh(Point3::new(100.0, 100.0, 917.0), 30.0, 24, 48)).unwrap();
        let params = DiscParams::default();
        let ep = EndplateConform {
            o4: &o_sup,
            o5: &o_inf,
            superior_axis: Vector3::z(),
        };

        let CurvedArms {
            prepared,
            straight,
            curved,
            moves,
        } = curved_disc_arms(synthetic_disc(), &params, ep);
        let native = |p: Vec3| prepared.center_native + p / params.scale;

        // (1) The projection is aimed at midsides only, and it moved nothing else. A corner id
        // in the move list would be rejected by `with_projected_midsides` (it panics), so this
        // checks the *selection*, one level before that guard.
        assert!(
            !moves.is_empty(),
            "the curved endplate must authorise moves"
        );
        assert_only_named_midsides_moved(&straight, &curved, &moves);
        let named: BTreeSet<VertexId> = moves.iter().map(|&(v, _)| v).collect();

        // (2) THE PAYOFF, like for like: the authorised midsides end up closer to the bone.
        let res = |mesh: &Tet10Mesh| -> (f64, f64) {
            let rs: Vec<f64> = named
                .iter()
                .map(|&v| endplate_residual(native(mesh.positions()[v as usize]), &o_sup, &o_inf))
                .collect();
            residual_stats(&rs)
        };
        let (max_s, rms_s) = res(&straight);
        let (max_c, rms_c) = res(&curved);
        // (3) VALIDITY + COVERAGE (plan §4.4).
        let worst = worst_gauss_det_ratio(&curved, &straight);
        let (delivered, max_move, mean_move) =
            projection_coverage(&straight, &curved, &moves, params.scale);
        println!(
            "synthetic curved endplate: {} midsides projected — residual max {max_s:.4} -> \
             {max_c:.4}, rms {rms_s:.4} -> {rms_c:.4} mm; delivered {:.1} % \
             (max move {max_move:.4}, mean {mean_move:.4} mm); worst detJ/detJ_rest {worst:.4}",
            moves.len(),
            100.0 * delivered,
        );
        // MEASURED on this fixture (deterministic mesher, fixed spheres): 604 midsides, straight
        // residual max 0.0896 / rms 0.0291 mm, closing to max 0.0001 / rms 0.0000; 100 % of the
        // moves delivered in full (max move 0.0896, mean 0.0217 mm); worst detJ/detJ_rest 0.9303,
        // i.e. the quality floor never has to engage at this curvature. The chord gap is small in
        // absolute terms because a 3 mm cell against a 30 mm sphere is finely resolved — the
        // sagitta of a half-cell chord is ~(1.5 mm)²/(2·30 mm) ≈ 0.04 mm, which is what these
        // numbers are. The population pin below is what makes a silent change in the mesh, the
        // band rule or the selection fail here rather than dilute the statistics.
        assert_eq!(moves.len(), 604, "the projected midside population changed");
        assert!(
            max_s > 0.05 && rms_s > 0.02,
            "the straight midsides must chord across the curved endplate \
             (max {max_s:.4}, rms {rms_s:.4} mm) — otherwise this fixture gates nothing",
        );
        assert!(
            max_c < 0.25 * max_s && rms_c < 0.25 * rms_s,
            "the projection must seat the midsides on the curved endplate \
             (max {max_s:.4} -> {max_c:.4}, rms {rms_s:.4} -> {rms_c:.4} mm)",
        );
        assert!(
            worst >= DISC_MIDSIDE_CONFORM_QUALITY_FLOOR,
            "an element fell below the quality floor at some Gauss point \
             (worst detJ/detJ_rest {worst:.4})",
        );
        assert!(
            delivered > 0.9,
            "only {:.1} % of the authorised midsides reached their full projection — the \
             back-off is silent, so a mostly-straight run must fail here",
            100.0 * delivered,
        );

        // (4) The shipped path builds exactly this mesh, and the curved disc still bonds and
        // sweeps soundly — a curved element is what the solver's isoparametric path handles,
        // and this is the only place CI drives it through the bonded solve.
        let mut built = build_bonded_disc_tet10(synthetic_disc(), &params, Some(ep))
            .expect("curved synthetic Tet10 disc bonds");
        assert_builder_bonds_this_mesh(&built, &curved, prepared.center_native, params.scale);
        // ⚠ `assert_full_face_band` is fed the PRE-projection mesh: its independence comes
        // from midpoint coincidence, which the projection destroys by design. The topology it
        // checks is unchanged by the projection, so the check is still the right one.
        assert_full_face_band(&straight, &prepared.inferior, built.sandwich.lower_face());
        assert_full_face_band(&straight, &prepared.superior, built.sandwich.upper_face());
        let angles: Vec<f64> = [-0.3_f64, 0.0, 0.3]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        assert_restoring_sweep(&built.capture_flexion(&angles), 2e-2);
    }

    /// The exact-geometry FOM gate: bonding the disc's endplate band onto the **real**
    /// vertebral endplate (Strategy B) still produces a physically sound flexion/extension
    /// moment-rotation response, and does not regress vs the raw floating-disc baseline. The
    /// win — the bond ties to the exact bone, not a proxy gap — is MEASURED here (both
    /// moment-rotation curves committed), not asserted.
    ///
    /// This is the rung's headline: the settled modeling call gates the exact-geometry claim
    /// on this FOM, not on the conform distances. The conform moves are sub-mm to a few mm, so
    /// a large moment shift is not expected; the gate demands soundness + no regression and
    /// records the comparison.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn disc_endplate_conform_moment_rotation_fom() {
        use cf_fsu_geometry::{load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc_mesh = load_from_env("CF_DISC_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let params = DiscParams::default();

        // Symmetric sub-degree sweep at ±0.5°. This is the small-strain LINEARISATION range the
        // k_disc figure of merit is defined on, not a limit on the conformed disc — rung 2's
        // `conformed_disc_survives_a_large_angle_sweep` drives the same conformed disc to ±6.0°
        // in 0.1° steps.
        let angles: Vec<f64> = [-0.5_f64, -0.25, 0.0, 0.25, 0.5]
            .iter()
            .map(|d| d.to_radians())
            .collect();

        // BASELINE arm: raw floating disc (the pre-conform physics).
        let mut raw = build_bonded_disc(disc_mesh.clone(), &params, None).expect("raw disc bonds");
        let traj_raw = raw.capture_flexion(&angles);

        // CONFORMED arm: bonded band seated on the real endplate.
        let endplates = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };
        let mut conf =
            build_bonded_disc(disc_mesh, &params, Some(endplates)).expect("conformed disc bonds");
        let traj_conf = conf.capture_flexion(&angles);

        // (1) NON-VACUITY: the conform actually seated nodes — the two rest surfaces differ
        // (same topology/ordering: deterministic mesher, same input). A move above the ~0.02 mm
        // solve-noise floor and inside the 6 mm bonded cap proves the endplate band moved onto
        // the bone rather than the test comparing two identical meshes.
        assert_eq!(
            traj_raw.rest_nodes_native.len(),
            traj_conf.rest_nodes_native.len(),
            "raw and conformed discs must share topology for a node-wise comparison"
        );
        let max_seat = traj_raw
            .rest_nodes_native
            .iter()
            .zip(&traj_conf.rest_nodes_native)
            .map(|(a, b)| (a - b).norm())
            .fold(0.0_f64, f64::max);
        println!("max endplate-seat move: {max_seat:.3} mm");
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): the deepest seated node
        // moves 4.156 mm onto the bone — a genuine central endplate node the old hard 3–4 mm
        // cap would have clipped, seated by the loose 6 mm backstop + SI-alignment guard.
        // (4.457 mm before rung 4 raised `DISC_CONFORM_QUALITY_FLOOR` 0.05 -> 0.25; the deepest
        // node is exactly where a stricter back-off bites hardest, and it still clears 4 mm.)
        assert!(
            (3.95..=4.37).contains(&max_seat),
            "the conform must seat nodes onto the bone (committed 4.156 mm), got {max_seat:.3} mm"
        );

        // (2) BOTH arms are physically sound (valid surface, conservation, strictly restoring
        // off neutral, real imposed deformation). This is the spike's central worry — that the
        // conform breaks the bonded solve — retired as a hard gate.
        let disp_raw = assert_restoring_sweep(&traj_raw, 5e-2);
        let disp_conf = assert_restoring_sweep(&traj_conf, 5e-2);

        // (3) MOMENT-ROTATION FOM: small-strain stiffness each way, both arms. Report the full
        // curves; gate on soundness + no regression (same restoring sense, finite, same order).
        let k = |traj: &FlexionTrajectory, theta: f64| -> f64 {
            let f = traj
                .frames
                .iter()
                .find(|f| (f.theta - theta).abs() < 1e-12)
                .expect("swept angle present");
            f.moment / theta
        };
        let (flex, ext) = (0.5_f64.to_radians(), -0.5_f64.to_radians());
        let (k_flex_raw, k_ext_raw) = (k(&traj_raw, flex), k(&traj_raw, ext));
        let (k_flex_conf, k_ext_conf) = (k(&traj_conf, flex), k(&traj_conf, ext));
        println!(
            "k_disc (N·m/rad) — raw: flex {k_flex_raw:.2} / ext {k_ext_raw:.2} (disp {disp_raw:.3} mm); \
             conformed: flex {k_flex_conf:.2} / ext {k_ext_conf:.2} (disp {disp_conf:.3} mm)"
        );

        // Restoring (k > 0 for a restoring spring: M = k·θ opposes θ ⇒ M/θ < 0? here moment is
        // the reaction, negative-restoring, so k < 0 — assert the CONFORMED matches the raw
        // sign and stays a consistent spring). Both arms already passed the strict per-frame
        // restoring check in assert_restoring_sweep; here we compare the linearised stiffnesses.
        // COMMITTED: both arms read k_disc ≈ −0.28 N·m/rad each way (restoring); seating the
        // band onto the real endplate shifts it ~1.8 % (−0.2811 → −0.2760 flexion, −0.2788 →
        // −0.2738 extension — the four-decimal figures come from rung 2's
        // `conform_delta_by_element_fom`; this test's own two-decimal print of the same arms,
        // −0.28 → −0.27, was originally read as "~4 %", which was a rounding artifact). The FOM
        // is essentially unchanged — the exact-geometry win is that the bond ties to the REAL
        // bone, not a moment shift, which is why the gate is soundness + no-regression, not
        // "improves"; the direct geometric gate is `conform_seats_the_bonded_face_on_the_bone_fom`.
        for (kc, kr, name) in [
            (k_flex_conf, k_flex_raw, "flexion"),
            (k_ext_conf, k_ext_raw, "extension"),
        ] {
            assert!(kc.is_finite(), "conformed {name} k_disc must be finite");
            assert!(
                kc.signum() == kr.signum(),
                "conformed {name} k_disc must keep the raw restoring sense ({kc:.3} vs {kr:.3})"
            );
            assert!(
                (-0.34..=-0.22).contains(&kc) && (-0.34..=-0.22).contains(&kr),
                "k_disc {name} outside the committed ≈ −0.28 N·m/rad band (conf {kc:.3}, raw {kr:.3})"
            );
            // No-regression: the seated band shifts the linearised stiffness by < ~10 % (observed
            // ~1.8 %). Loose enough to record a legitimate shift, tight enough to catch a collapse.
            let ratio = (kc / kr).abs();
            assert!(
                (0.9..=1.1).contains(&ratio),
                "conformed {name} k_disc must not regress vs raw (ratio {ratio:.3}: {kc:.3} vs {kr:.3})"
            );
        }
    }

    /// Recompute a full-face bonded band a **second, independent way** and assert it equals
    /// the set that was actually bonded.
    ///
    /// The teeth are in the independence: [`full_face_band`] maps an edge to its midside
    /// through the canonical [`TET10_EDGE_NODES`] slot table, while this identifies each
    /// midside's parent corners **geometrically** — a straight Tet10 midside is placed at
    /// exactly `(p[a] + p[b]) * 0.5`, so the corner pair whose midpoint coincides with it
    /// names its parents without consulting any table. A permuted slot table pins the *wrong*
    /// midsides, which the ratio band cannot see; this can, whether or not the permutation
    /// happens to change the band's size (the one measured did — 413 → 647 — so a size assert
    /// would have caught that particular rotation, but a size-preserving one would slip past
    /// it). So does the other trap: a band accidentally filtered through
    /// `referenced_vertices` (corner-only) loses every midside, and the recomputed set still
    /// has them.
    ///
    /// Also checks the boundary-face route as a **subset**, not an equality: the midsides of
    /// a fully-in-band `boundary_faces6` face must all be bonded, but the band is a
    /// volumetric slab, so its interior midsides have no boundary face to appear on and
    /// equality would be false by construction.
    ///
    /// ⚠ **Straight-Tet10 only.** The midpoint-coincidence step is what makes the
    /// recomputation table-free, and it holds precisely because this rung leaves every
    /// midside at its edge midpoint. Rung 3 projects the bonded-face midsides onto the real
    /// endplate, at which point this helper must be given the band from the *pre-projection*
    /// mesh (the topology it checks is unchanged by the projection) rather than silently
    /// reused on the curved one.
    fn assert_full_face_band(tet10: &Tet10Mesh, corner_band: &[VertexId], bonded: &[VertexId]) {
        let corners: std::collections::HashSet<VertexId> = corner_band.iter().copied().collect();
        let pos = tet10.positions();
        let bonded_set: std::collections::HashSet<VertexId> = bonded.iter().copied().collect();
        assert_eq!(
            bonded_set.len(),
            bonded.len(),
            "bonded band has a duplicate id"
        );

        // Geometric route: every midside whose two (geometrically identified) parent corners
        // are both in the corner band.
        let mut expected: BTreeSet<VertexId> = corner_band.iter().copied().collect();
        let n_corners = tet10.n_corners();
        #[allow(clippy::cast_possible_truncation)]
        let n_tets = tet10.n_tets() as TetId;
        let mut midsides_seen = 0usize;
        for t in 0..n_tets {
            let c = tet10.tet_vertices(t);
            let m = tet10
                .tet_midside_nodes(t)
                .expect("a Tet10Mesh surfaces midside nodes");
            for &mid in &m {
                assert!(
                    mid as usize >= n_corners,
                    "midside id {mid} must live above the {n_corners} corner ids"
                );
                midsides_seen += 1;
                // Identify the parent edge by midpoint coincidence — table-free.
                let mut parents = None;
                for a in 0..4 {
                    for b in (a + 1)..4 {
                        let midpoint = (pos[c[a] as usize] + pos[c[b] as usize]) * 0.5;
                        if (midpoint - pos[mid as usize]).norm() < 1e-12 {
                            assert!(
                                parents.is_none(),
                                "midside {mid} has two candidate parent edges (degenerate tet {t})"
                            );
                            parents = Some((c[a], c[b]));
                        }
                    }
                }
                let (pa, pb) =
                    parents.unwrap_or_else(|| panic!("no parent edge found for midside {mid}"));
                if corners.contains(&pa) && corners.contains(&pb) {
                    expected.insert(mid);
                }
            }
        }
        assert!(midsides_seen > 0, "the enriched mesh has no midside nodes");

        let expected: Vec<VertexId> = expected.into_iter().collect();
        assert_eq!(
            expected,
            bonded,
            "the bonded band must match its independent (geometric) recomputation — \
             {} vs {} ids",
            expected.len(),
            bonded.len()
        );
        // NON-VACUITY: the tie is genuinely full-face, i.e. midsides were added at all. A
        // corner-only band would pass every soundness and stiffness check while measuring a
        // spot-welded bond.
        assert!(
            bonded.len() > corner_band.len(),
            "a full-face tie must add midsides to the {} corner band, got {}",
            corner_band.len(),
            bonded.len()
        );

        // Boundary route (subset — see the doc comment): the three midsides of any
        // boundary face whose corners are all in the band must be bonded.
        let faces6 = tet10
            .boundary_faces6()
            .expect("a Tet10Mesh carries six-node boundary faces");
        for f in faces6 {
            if f[..3].iter().all(|v| corners.contains(v)) {
                for &mid in &f[3..] {
                    assert!(
                        bonded_set.contains(&mid),
                        "midside {mid} of a fully-in-band boundary face is not bonded"
                    );
                }
            }
        }
    }

    /// License-free Tet10 coverage of the rung-1 seam — the **only** arm of this rung CI can
    /// run (every real-anatomy gate is `#[ignore]`d + env-gated).
    ///
    /// Asserts what does not need the real disc: the enrichment preserves the corner
    /// id-space and adds midsides, the full-face band matches its independent recomputation
    /// (`assert_full_face_band` — the assertion with teeth), and the quadratic bonded solve
    /// is sound (converges, conserves, strictly restoring, really deforms). The `k_disc`
    /// ratio bracket is measured on the real disc in the `#[ignore]`d gate below; the
    /// synthetic slab's ratio is *reported* here, not asserted, because the pre-registered
    /// 0.665 is a real-anatomy number and a 24×20×6 mm box at `cell = 3 mm` is a different
    /// bending problem.
    #[test]
    fn tet10_full_face_bond_is_sound_on_the_synthetic_disc() {
        let params = DiscParams::default();
        let theta = 0.3_f64.to_radians(); // sub-degree: stay in the SPD region

        let mut tet4 = build_bonded_disc(synthetic_disc(), &params, None).unwrap();
        let mut tet10 = build_bonded_disc_tet10(synthetic_disc(), &params, None).unwrap();

        // Enrichment preserved the corner id-space and appended midsides.
        let prepared = prepare_disc(synthetic_disc(), &params, None).unwrap();
        let n_corners = prepared.tet.n_vertices();
        let mesh10 = Tet10Mesh::from_tet4(&prepared.tet);
        assert_eq!(mesh10.n_corners(), n_corners, "corner count preserved");
        assert!(
            mesh10.n_vertices() > n_corners,
            "enrichment must add midside nodes ({} vs {n_corners})",
            mesh10.n_vertices()
        );

        // The full-face tie, cross-checked against its table-free recomputation.
        assert_full_face_band(&mesh10, &prepared.inferior, tet10.sandwich.lower_face());
        assert_full_face_band(&mesh10, &prepared.superior, tet10.sandwich.upper_face());

        // The quadratic bonded solve is sound: converges (no panic), conserves, strictly
        // restoring off neutral, and imposes a real deformation.
        let angles: Vec<f64> = [-0.3_f64, 0.0, 0.3]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = tet10.capture_flexion(&angles);
        assert_restoring_sweep(&traj, 2e-2);

        // Reported, not asserted (see the doc comment).
        let k4 = tet4.flexion_moment(theta).0 / theta;
        let k10 = tet10.flexion_moment(theta).0 / theta;
        println!(
            "synthetic k_disc: Tet4 {k4:.4}, Tet10 {k10:.4} N·m/rad (ratio {:.3})",
            k10 / k4
        );
    }

    /// **The rung-1 payoff gate** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §4.2): the
    /// element-order effect on the disc's bending stiffness, re-earned as committed,
    /// re-runnable code.
    ///
    /// Two arms on the **raw** (un-conformed) real disc, sharing one mesh, one band rule and
    /// one rigid scene ([`prepare_disc`]) and differing *only* in the element + its full-face
    /// tie — so the ratio attributes to element order at fixed geometry.
    ///
    /// A reverted spike predicted 0.665 (≈ 33 % softer), and the gate was written with that
    /// **pre-registered** as a 0.60..=0.73 bracket — validate-the-harness-against-a-known-
    /// value, not a guess. The first run landed at 0.666 / 0.663 with the band sizes matching
    /// the spike id-for-id, so the assert was then tightened to ±5 % of the measured values
    /// (the numbers are committed at the assert). Note what the ratio is *not*: the spike's
    /// −0.1861 is not proven to be the converged truth — a refined Tet4 would soften too —
    /// so the settled claim is that the quadratic element relaxes this bending mode by ~1/3
    /// at fixed geometry. Earning the accuracy claim is rung 5's h-refinement.
    ///
    /// **The band cross-check, not the ratio, is the assertion with teeth.** An under-tied
    /// band drifts the ratio from 0.665 *toward* the corner-only 0.570, so a *partial*
    /// under-tie — some midsides missed, or a mis-indexed slot table — lands between the two
    /// and sits comfortably inside any band drawn around 0.665. (To be exact about what the
    /// stiffness band does cover: a *fully* corner-only tie at 0.570 would fall outside both
    /// the pre-registered bracket and the ±5 % pin. It is the partial case, and a permuted
    /// table, that the stiffness cannot see.) `assert_full_face_band` sees all of them, and
    /// is mutation-verified against both traps.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn tet10_full_face_bond_element_order_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let params = DiscParams::default();
        // ±0.5°: the spike's configuration, and inside the validated SPD range.
        let (flex, ext) = (0.5_f64.to_radians(), -0.5_f64.to_radians());

        let mut tet4 =
            build_bonded_disc(disc_mesh.clone(), &params, None).expect("Tet4 raw disc bonds");
        let mut tet10 = build_bonded_disc_tet10(disc_mesh.clone(), &params, None)
            .expect("Tet10 raw disc bonds");

        // (1) BAND CROSS-CHECK — the non-vacuity gate. Re-derive the shared linear mesh and
        // check the bonded sets against a table-free recomputation, then commit their exact
        // sizes. Note the re-derivation does not *assume* the mesher is deterministic: it
        // builds a second mesh from the same input and asserts id-for-id set equality against
        // the band the first one bonded, so non-determinism in the mesher would fail this
        // gate rather than hide inside it.
        let prepared = prepare_disc(disc_mesh, &params, None).expect("prepare raw disc");
        let mesh10 = Tet10Mesh::from_tet4(&prepared.tet);
        assert_eq!(
            mesh10.n_corners(),
            prepared.tet.n_vertices(),
            "enrichment preserves the corner id-space"
        );
        let (inf4, sup4) = (prepared.inferior.len(), prepared.superior.len());
        let (inf10, sup10) = (
            tet10.sandwich.lower_face().len(),
            tet10.sandwich.upper_face().len(),
        );
        println!(
            "bands: inferior {inf4} -> {inf10}, superior {sup4} -> {sup10}; \
             nodes {} -> {}",
            prepared.tet.n_vertices(),
            mesh10.n_vertices()
        );
        assert_full_face_band(&mesh10, &prepared.inferior, tet10.sandwich.lower_face());
        assert_full_face_band(&mesh10, &prepared.superior, tet10.sandwich.upper_face());
        // COMMITTED (BodyParts3D FMA16036, DiscParams::default): the corner bands are 228 /
        // 367 and the full-face tie widens them to 1005 / 1598 — the same sizes the reverted
        // spike reported, which is itself the confirmation that this harness reproduces it.
        assert_eq!(
            (inf4, sup4, inf10, sup10),
            (228, 367, 1005, 1598),
            "committed band sizes changed — the mesh or the band rule moved, and the ratio \
             below is no longer comparable to the spike"
        );
        assert_eq!(
            (prepared.tet.n_vertices(), mesh10.n_vertices()),
            (7849, 19449),
            "committed node counts changed (7849 corners -> 19449 with midsides)"
        );

        // (2) SOUNDNESS + (3) the stiffnesses, both arms, both directions. The probe
        // converging at all is itself a hard check — a diverged solve panics.
        let (m_flex4, r_flex4) = tet4.flexion_moment(flex);
        let (m_ext4, r_ext4) = tet4.flexion_moment(ext);
        let (m_flex10, r_flex10) = tet10.flexion_moment(flex);
        let (m_ext10, r_ext10) = tet10.flexion_moment(ext);
        for (resid, name) in [
            (r_flex4, "Tet4 flexion"),
            (r_ext4, "Tet4 extension"),
            (r_flex10, "Tet10 flexion"),
            (r_ext10, "Tet10 extension"),
        ] {
            assert!(
                resid < 1e-8,
                "{name} bond must conserve (‖ΣF‖+‖ΣM‖ = {resid:.2e})"
            );
        }
        let (k_flex4, k_ext4) = (m_flex4 / flex, m_ext4 / ext);
        let (k_flex10, k_ext10) = (m_flex10 / flex, m_ext10 / ext);
        println!(
            "k_disc (N·m/rad) — Tet4: flex {k_flex4:.4} / ext {k_ext4:.4}; \
             Tet10 full-face: flex {k_flex10:.4} / ext {k_ext10:.4}; \
             ratio flex {:.3} / ext {:.3}; \
             conservation resid Tet4 {r_flex4:.2e}/{r_ext4:.2e}, Tet10 {r_flex10:.2e}/{r_ext10:.2e}",
            k_flex10 / k_flex4,
            k_ext10 / k_ext4
        );

        // (4) DIRECTION + MAGNITUDE, asserted PER DIRECTION (the spike quoted a mean and the
        // two directions differ ~0.5 %, so a mean would hide a one-sided failure).
        //
        // COMMITTED (BodyParts3D FMA16036, DiscParams::default, raw un-conformed mesh, ±0.5°):
        //   Tet4            flex −0.2811  ext −0.2788  N·m/rad
        //   Tet10 full-face flex −0.1873  ext −0.1849  N·m/rad
        //   ratio           flex  0.666   ext  0.663
        // The pre-registered bracket was 0.60..=0.73 around the reverted spike's 0.665; both
        // directions landed inside it, so the harness reproduces the spike and the bands
        // (228→1005 / 367→1598) match it id-for-id. The live assert below is the *tightened*
        // ±5 % no-regression band around these measured values — a change big enough to leave
        // it is a real shift in the element effect, not noise (the bonded moment reproduces to
        // < 1e-3 relative across captures).
        for (k10, k4, expect, name) in [
            (k_flex10, k_flex4, 0.666, "flexion"),
            (k_ext10, k_ext4, 0.663, "extension"),
        ] {
            assert!(
                k10 < 0.0 && k4 < 0.0,
                "{name}: both arms must be restoring ({k10:.4} / {k4:.4})"
            );
            assert!(
                k10.abs() < k4.abs(),
                "{name}: the quadratic element must relax the bending mode ({k10:.4} vs {k4:.4})"
            );
            let ratio = k10 / k4;
            assert!(
                ((0.95 * expect)..=(1.05 * expect)).contains(&ratio),
                "{name}: k_disc ratio {ratio:.3} is outside ±5 % of the committed {expect:.3} \
                 — debug the band and the enrichment; do NOT widen the band to admit it"
            );
        }
    }

    /// Distance from a native-mm point to the **real bone surface**: the nearer of the two
    /// vertebra oracles' `|eval|`.
    ///
    /// This is the exact-geometry residual — how far the geometry the bond actually ties to
    /// still sits off the endplate. Deliberately read through [`Sdf::eval`] and *not*
    /// `MeshOracle::closest_point`, which is the primitive `bonded_conform_target` projects
    /// with: a residual computed from the projector could not falsify the projector.
    fn endplate_residual(p: Point3<f64>, o4: &MeshOracle, o5: &MeshOracle) -> f64 {
        o4.eval(p).abs().min(o5.eval(p).abs())
    }

    /// `(max, RMS)` of a residual sample, both reported and both gated: the max catches a
    /// single node left off the bone, the RMS catches a population that only *mostly* seated
    /// (a max-only gate passes a run where 95 % of nodes never moved).
    // Node counts are in the thousands — exact in f64.
    #[allow(clippy::cast_precision_loss)]
    fn residual_stats(rs: &[f64]) -> (f64, f64) {
        let max = rs.iter().copied().fold(0.0_f64, f64::max);
        let rms = (rs.iter().map(|r| r * r).sum::<f64>() / rs.len() as f64).sqrt();
        (max, rms)
    }

    /// The bonded-face **boundary** nodes of a built disc: the bonded band restricted to the
    /// mesh surface. Read off the sandwich's own vertex sets and boundary triangulation, so it
    /// is the set the shipped bond ties, not a re-derivation — and it is exactly the candidate
    /// set `endplate_conform_moves` walks (interior band nodes are inside the disc and have no
    /// endplate to sit on).
    fn bonded_face_boundary_nodes<Msh, E, const N: usize, const G: usize>(
        disc: &BondedDisc<Msh, E, N, G>,
    ) -> Vec<VertexId>
    where
        Msh: Mesh,
        E: Element<N, G> + Default,
    {
        let surface: std::collections::HashSet<VertexId> =
            disc.boundary_faces().iter().flatten().copied().collect();
        disc.sandwich
            .lower_face()
            .iter()
            .chain(disc.sandwich.upper_face())
            .copied()
            .filter(|v| surface.contains(v))
            .collect()
    }

    /// The bonded-face boundary nodes split by what the conform *intended* versus what it
    /// *delivered* — three sets, not two, because the difference between them is where the
    /// silent failures live.
    ///
    /// - `authorised` — what the SI-alignment discriminator said should seat, read from
    ///   `bonded_conform_target` on the RAW positions (INTENT).
    /// - `moved` — what the pipeline actually delivered, read from the two node arrays (OUTCOME).
    /// - `backed_off` — `authorised` minus `moved`: nodes the quality-floor back-off gave up on
    ///   entirely. No "max move" statistic can show these, because they did not move.
    /// - `guard_declined` — candidates the discriminator refused, i.e. the overhanging annular
    ///   rim that #701's settled call leaves straight on purpose.
    struct ConformSplit {
        candidates: Vec<VertexId>,
        authorised: Vec<VertexId>,
        guard_declined: Vec<VertexId>,
        moved: Vec<VertexId>,
        backed_off: usize,
    }

    /// Compute the [`ConformSplit`] for a built conformed disc against its raw twin's node
    /// positions (both native mm, same topology).
    fn conform_split(
        conf: &BondedDisc,
        p_raw: &[Point3<f64>],
        p_conf: &[Point3<f64>],
        ep: EndplateConform,
    ) -> ConformSplit {
        let candidates = bonded_face_boundary_nodes(conf);
        assert!(
            !candidates.is_empty(),
            "the bonded band must reach the disc surface"
        );
        let authorised: Vec<VertexId> = candidates
            .iter()
            .copied()
            .filter(|&v| {
                bonded_conform_target(p_raw[v as usize], ep.superior_axis, ep.o4, ep.o5).is_some()
            })
            .collect();
        let authorised_set: std::collections::HashSet<VertexId> =
            authorised.iter().copied().collect();
        let guard_declined: Vec<VertexId> = candidates
            .iter()
            .copied()
            .filter(|v| !authorised_set.contains(v))
            .collect();
        let moved: Vec<VertexId> = candidates
            .iter()
            .copied()
            .filter(|&v| (p_raw[v as usize] - p_conf[v as usize]).norm() > 1e-12)
            .collect();
        // `with_projected_nodes` only touches the nodes it is handed, so a moved node must have
        // been authorised — asserted rather than assumed, since it is what makes `backed_off`
        // readable as "the back-off gave up", not "something else moved a node".
        assert!(
            moved.iter().all(|v| authorised_set.contains(v)),
            "a node moved without the SI-alignment guard authorising it"
        );
        let backed_off = authorised.len() - moved.len();
        ConformSplit {
            candidates,
            authorised,
            guard_declined,
            moved,
            backed_off,
        }
    }

    /// **The rung-2 exact-geometry residual gate**
    /// (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §4.3): how close the bonded-face boundary
    /// nodes *ended up* to the real bone.
    ///
    /// This is the arc's own stated payoff, and until this rung nothing measured it. #701's FOM
    /// commits `max_seat` — how far nodes **moved** — which is a different quantity: a node can
    /// move 4 mm and still stop 2 mm short, and a conform that seated nothing would report a
    /// `max_seat` of 0 rather than a residual.
    ///
    /// Both arms are read **at rest, before any solve**: `BondedSandwich::from_tet_mesh`
    /// initialises the solve configuration to the same mesh positions it snapshots the bond's
    /// body-frame offsets from, so `deformed_nodes_native()` on a freshly built disc is
    /// literally the geometry the bond ties to.
    ///
    /// ## Which population the gate is on, and why it is not selected on the outcome
    ///
    /// The SI-alignment guard declines nodes silently, and by the #701 settled modeling call it
    /// is *supposed* to: the overhanging annular rim attaches to the ring apophysis, not the
    /// endplate face, so its residual is a modeling choice rather than a defect. Averaging that
    /// rim into the payoff metric would let a population that never moves mask (or manufacture) a
    /// change in the seated one. So the strict-decrease assert is on the **authorised** nodes —
    /// the ones the discriminator said should seat — with the guard-declined population reported
    /// alongside, and the whole candidate set *also* gated on RMS strictly down and max
    /// non-increasing (which is what "no node ended up further from the bone" means).
    ///
    /// Authorised, deliberately, and **not** "nodes that moved": the moved set is selected on the
    /// outcome. The two are read independently — `bonded_conform_target` on the raw positions
    /// says what the discriminator *intended*, the two node arrays say what the pipeline
    /// *delivered* — and their difference (2 nodes here) is a failure no "max move" number shows.
    ///
    /// **What that buys was measured on two mutants rather than argued** — and it is narrower
    /// than the tidy version of the argument:
    ///
    /// - *Nodes dropped from the move list* (mutant: skip most authorised nodes in
    ///   `endplate_conform_moves`, so 214 of 233 never seat). The moved-set statistic still
    ///   improves — RMS 2.000 → 1.103, max 5.037 → 3.255 — so a **strict-decrease gate on the
    ///   moved set PASSES** while 92 % of the intended seating silently vanished. The authorised
    ///   set carries the dropped nodes at their raw residual, so its max goes 5.724 → 5.724 and
    ///   the same gate **FAILS**. This is the case the definition exists for.
    /// - *Back-off degrades everything proportionally* (mutant: `DISC_CONFORM_QUALITY_FLOOR`
    ///   0.05 → 0.50, measured when the shipped floor was still 0.05 — a before/after pair, not
    ///   today's committed values). Here the two sets move together — moved RMS 0.659 → 0.908
    ///   against authorised 0.656 → 0.901 — so the choice of population buys **nothing**, because the
    ///   same mechanism degrades the survivors too. What catches this one is the committed
    ///   population split and the ±5 % pins, not the authorised-vs-moved distinction.
    ///
    /// Both mutants trip the exact population-split assert in (1), which is the broadest-teeth
    /// instrument here; the authorised-set choice is what additionally makes the *residual
    /// statistic itself* falsifiable in the drop case.
    ///
    /// ⚠ This is a **require-improvement** gate, deliberately unlike #701's `k_disc` gate
    /// (sound + measured + no-regression). The two must not be merged: `k_disc` is a physics
    /// consequence whose more-correct value may legitimately move either way, so requiring it to
    /// improve would false-fail better geometry. The residual is the *geometric* claim itself —
    /// if seating the band on the bone does not reduce the distance to the bone, the conform did
    /// not happen.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn conform_seats_the_bonded_face_on_the_bone_fom() {
        use cf_fsu_geometry::{load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc_mesh = load_from_env("CF_DISC_STL").unwrap();
        let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let params = DiscParams::default();
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };

        let raw = build_bonded_disc(disc_mesh.clone(), &params, None).expect("raw disc bonds");
        let conf = build_bonded_disc(disc_mesh, &params, Some(ep)).expect("conformed disc bonds");

        let (p_raw, p_conf) = (raw.deformed_nodes_native(), conf.deformed_nodes_native());
        assert_eq!(p_raw.len(), p_conf.len(), "the conform preserves topology");
        // The band is computed BEFORE the conform (`prepare_disc`), so both arms bond the same
        // ids — which is what makes the node-wise comparison below well-posed.
        assert_eq!(
            raw.sandwich.lower_face(),
            conf.sandwich.lower_face(),
            "both arms must bond the same inferior band"
        );
        assert_eq!(
            raw.sandwich.upper_face(),
            conf.sandwich.upper_face(),
            "both arms must bond the same superior band"
        );

        let ConformSplit {
            candidates,
            authorised,
            guard_declined,
            moved,
            backed_off,
        } = conform_split(&conf, &p_raw, &p_conf, ep);

        let res = |nodes: &[VertexId], p: &[Point3<f64>]| -> (f64, f64) {
            let rs: Vec<f64> = nodes
                .iter()
                .map(|&v| endplate_residual(p[v as usize], &o4, &o5))
                .collect();
            residual_stats(&rs)
        };
        let (all_max_raw, all_rms_raw) = res(&candidates, &p_raw);
        let (all_max_conf, all_rms_conf) = res(&candidates, &p_conf);
        let (au_max_raw, au_rms_raw) = res(&authorised, &p_raw);
        let (au_max_conf, au_rms_conf) = res(&authorised, &p_conf);
        let (dec_max, dec_rms) = res(&guard_declined, &p_conf);

        println!(
            "bonded-face boundary nodes: {} candidates = {} authorised ({} moved, {backed_off} \
             fully backed off by the quality floor) + {} guard-declined",
            candidates.len(),
            authorised.len(),
            moved.len(),
            guard_declined.len(),
        );
        println!(
            "residual |eval| to the nearer vertebra (mm) — \
             AUTHORISED: raw max {au_max_raw:.3} rms {au_rms_raw:.3} -> conformed max {au_max_conf:.3} rms {au_rms_conf:.3}; \
             ALL: raw max {all_max_raw:.3} rms {all_rms_raw:.3} -> conformed max {all_max_conf:.3} rms {all_rms_conf:.3}; \
             GUARD-DECLINED (left straight by design): max {dec_max:.3} rms {dec_rms:.3}"
        );

        // (1) NON-VACUITY, committed exactly. A conform that declined everything would satisfy a
        // "no node got worse" gate trivially, and these counts pin the whole population split —
        // so a silent change in the mesh, the band rule, or the discriminator fails here rather
        // than diluting the statistics below.
        //
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): of 583 bonded-face boundary
        // nodes the SI-alignment guard authorises 233; 230 of those are delivered and **3 are
        // backed off entirely** by the quality floor. That 3 is the number this split exists to
        // surface — it is invisible to a max-move statistic, and it is the failure mode that
        // grows if a future mesh is worse conditioned.
        //
        // ⚠ It grew from 2 to 3 at rung 4 when `DISC_CONFORM_QUALITY_FLOOR` went 0.05 -> 0.25,
        // which is the split doing its job: a stricter floor refuses more moves, and that is
        // precisely the "worse conditioned" direction this assert was written to make visible.
        assert_eq!(
            (
                candidates.len(),
                authorised.len(),
                moved.len(),
                backed_off,
                guard_declined.len()
            ),
            (583, 233, 230, 3, 350),
            "the bonded-face population split changed"
        );

        // (2) THE PAYOFF: the population the discriminator INTENDED to seat ends up strictly
        // closer to the bone, on both the extreme and the population statistic. Gating on the
        // authorised set rather than the moved set is deliberate, and what that buys is measured
        // on two mutants in this test's doc comment — decisive for nodes dropped from the move
        // list (where a moved-set gate still improves and PASSES) and worth nothing against a
        // back-off that degrades every node alike. Read the doc comment before widening this.
        assert!(
            au_max_conf < au_max_raw && au_rms_conf < au_rms_raw,
            "the authorised nodes must end up closer to the bone \
             (max {au_max_raw:.3} -> {au_max_conf:.3}, rms {au_rms_raw:.3} -> {au_rms_conf:.3} mm)"
        );

        // (3) The whole candidate population improves and no node is pushed further off the
        // bone. The max is only required not to INCREASE: it is set by a guard-declined rim
        // node, whose residual is identical in both arms by construction.
        assert!(
            all_rms_conf < all_rms_raw,
            "the bonded face as a whole must end up closer to the bone \
             (rms {all_rms_raw:.3} -> {all_rms_conf:.3} mm)"
        );
        assert!(
            all_max_conf <= all_max_raw,
            "no bonded-face node may end up further from the bone \
             (max {all_max_raw:.3} -> {all_max_conf:.3} mm)"
        );

        // (4) COMMITTED VALUES, two-sided at ±5 % (the #701 / rung-1 shape), so a regression AND
        // a silent geometry change both fail rather than only the first.
        //
        // COMMITTED (as above), residual in mm:
        //   AUTHORISED       raw max 5.724  rms 1.332  ->  conformed max 3.833  rms 0.750
        //   ALL candidates   raw max 12.577 rms 3.494  ->  conformed max 12.577 rms 3.424
        //
        // ⚠ RE-ANCHORED at rung 4, once, when `DISC_CONFORM_QUALITY_FLOOR` went 0.05 -> 0.25
        // (0.05 produced an undrivable mesh on a lofted disc — see that constant). The conform
        // is 0.094 mm of RMS less tight than it was and still nearly halves the residual; the
        // trade is stated in full in the constant's measured table.
        //   GUARD-DECLINED   (unchanged by construction) max 12.577 rms 4.377
        //
        // Read these honestly. The conform nearly halves the seated population's RMS distance to
        // the bone (1.332 -> 0.750) and that is the arc's payoff measured directly for the first
        // time. It does NOT drive the residual to zero: the worst authorised node still ends up
        // 3.833 mm out, because the quality floor backs a move off rather than invert a tet. And
        // the ALL-candidate figures barely move because 350 of 583 nodes are the overhanging
        // annular rim, left straight ON PURPOSE (#701's settled call — Sharpey's fibres attach
        // to the ring apophysis, not the endplate face); their 12.577 mm max is a closest-point
        // artifact of reaching sideways for the vertebral body wall, not an unmet target.
        for (v, expect, name) in [
            (au_max_raw, 5.724, "authorised raw max"),
            (au_rms_raw, 1.332, "authorised raw rms"),
            (au_max_conf, 3.833, "authorised conformed max"),
            (au_rms_conf, 0.750, "authorised conformed rms"),
            (all_rms_raw, 3.494, "all raw rms"),
            (all_rms_conf, 3.424, "all conformed rms"),
        ] {
            assert!(
                ((0.95 * expect)..=(1.05 * expect)).contains(&v),
                "{name} {v:.3} mm is outside ±5 % of the committed {expect:.3} mm"
            );
        }
    }
    /// Assert each `(measured, committed, name)` row sits within ±5 % of its committed value —
    /// the two-sided pin shape #701, rung 1 and rung 2 all use, in one place.
    fn assert_within_5_percent(rows: &[(f64, f64, &str)]) {
        for &(v, expect, name) in rows {
            assert!(
                ((0.95 * expect)..=(1.05 * expect)).contains(&v),
                "{name} {v:.4} is outside ±5 % of the committed {expect:.4}"
            );
        }
    }

    /// The rung-3 straight/curved pair, built the way `build_bonded_disc_tet10` builds them:
    /// prepare (corner-conform) → enrich → select the bonded-face boundary midsides → project.
    ///
    /// Both gates need *both* arms from ONE prepared mesh, so that the only difference between
    /// them is the midside projection — the rung-1 discipline (`prepare_disc` shared by the two
    /// element arms) applied one rung up.
    struct CurvedArms {
        prepared: PreparedDisc,
        straight: Tet10Mesh,
        curved: Tet10Mesh,
        moves: Vec<(VertexId, Vec3)>,
    }

    fn curved_disc_arms(mesh: IndexedMesh, params: &DiscParams, ep: EndplateConform) -> CurvedArms {
        let prepared = prepare_disc(mesh, params, Some(ep)).expect("prepare the conformed disc");
        let straight = Tet10Mesh::from_tet4(&prepared.tet);
        let moves = endplate_midside_conform_moves(
            &straight,
            &prepared.inferior,
            &prepared.superior,
            ep,
            prepared.center_native,
            params.scale,
        );
        let curved = straight
            .clone()
            .with_projected_midsides(&moves, DISC_MIDSIDE_CONFORM_QUALITY_FLOOR);
        CurvedArms {
            prepared,
            straight,
            curved,
            moves,
        }
    }

    /// Assert a built disc bonds exactly `mesh` — every node, midsides included, mapped back to
    /// native mm. Without it a gate could measure a geometry no production path produces.
    fn assert_builder_bonds_this_mesh(
        built: &BondedDisc<Tet10Mesh, Tet10, 10, 4>,
        mesh: &Tet10Mesh,
        center_native: Point3<f64>,
        scale: f64,
    ) {
        let nodes = built.deformed_nodes_native();
        assert_eq!(nodes.len(), mesh.n_vertices(), "same node count");
        assert!(
            nodes
                .iter()
                .zip(mesh.positions())
                .all(|(&q, &p)| (center_native + p / scale - q).norm() < 1e-12),
            "the builder must bond the mesh this gate measures",
        );
    }

    /// The bonded-face boundary **midsides** split by what the anatomy discriminator intends —
    /// the quadratic sibling of [`ConformSplit`], and computed the same way: from
    /// `bonded_conform_target` on the *straight* positions (INTENT), never from what moved
    /// (OUTCOME).
    ///
    /// `candidates` is recomputed here from the six-node boundary faces rather than read back
    /// from the production selector, so the two can disagree — which is what makes the equality
    /// assert in the gate worth writing.
    struct MidsideSplit {
        candidates: BTreeSet<VertexId>,
        authorised: BTreeSet<VertexId>,
        declined: BTreeSet<VertexId>,
    }

    fn midside_split(
        straight: &Tet10Mesh,
        band: &std::collections::HashSet<VertexId>,
        ep: EndplateConform,
        center_native: Point3<f64>,
        scale: f64,
    ) -> MidsideSplit {
        let candidates: BTreeSet<VertexId> = straight
            .boundary_faces6()
            .expect("a Tet10Mesh carries six-node boundary faces")
            .iter()
            .filter(|f| f[..3].iter().all(|v| band.contains(v)))
            .flat_map(|f| f[3..].iter().copied())
            .collect();
        let authorised: BTreeSet<VertexId> = candidates
            .iter()
            .copied()
            .filter(|&v| {
                let native = center_native + straight.positions()[v as usize] / scale;
                bonded_conform_target(native, ep.superior_axis, ep.o4, ep.o5).is_some()
            })
            .collect();
        let declined: BTreeSet<VertexId> = candidates
            .iter()
            .copied()
            .filter(|v| !authorised.contains(v))
            .collect();
        MidsideSplit {
            candidates,
            authorised,
            declined,
        }
    }

    /// `(max, RMS)` residual to the bone for each of the three midside populations, in both
    /// arms — and the print that puts them in the log next to each other.
    struct MidsideResiduals {
        /// The authorised population, straight then curved: the gate's payoff pair.
        authorised: ((f64, f64), (f64, f64)),
        /// Every candidate, straight then curved (rim included — reported, not the payoff).
        all: ((f64, f64), (f64, f64)),
        /// The guard-declined rim, curved arm (identical in both arms by construction).
        declined: (f64, f64),
    }

    fn midside_residuals(
        split: &MidsideSplit,
        straight: &Tet10Mesh,
        curved: &Tet10Mesh,
        (o4, o5): (&MeshOracle, &MeshOracle),
        center_native: Point3<f64>,
        scale: f64,
    ) -> MidsideResiduals {
        let res = |nodes: &BTreeSet<VertexId>, mesh: &Tet10Mesh| -> (f64, f64) {
            let rs: Vec<f64> = nodes
                .iter()
                .map(|&v| {
                    endplate_residual(center_native + mesh.positions()[v as usize] / scale, o4, o5)
                })
                .collect();
            residual_stats(&rs)
        };
        let out = MidsideResiduals {
            authorised: (
                res(&split.authorised, straight),
                res(&split.authorised, curved),
            ),
            all: (
                res(&split.candidates, straight),
                res(&split.candidates, curved),
            ),
            declined: res(&split.declined, curved),
        };
        let (
            ((au_max_s, au_rms_s), (au_max_c, au_rms_c)),
            ((all_max_s, all_rms_s), (all_max_c, all_rms_c)),
        ) = (out.authorised, out.all);
        let (dec_max, dec_rms) = out.declined;
        println!(
            "bonded-face boundary midsides: {} candidates = {} authorised + {} guard-declined",
            split.candidates.len(),
            split.authorised.len(),
            split.declined.len(),
        );
        println!(
            "residual |eval| to the nearer vertebra (mm) — \
             AUTHORISED: straight max {au_max_s:.3} rms {au_rms_s:.3} -> curved max {au_max_c:.3} rms {au_rms_c:.3}; \
             ALL: straight max {all_max_s:.3} rms {all_rms_s:.3} -> curved max {all_max_c:.3} rms {all_rms_c:.3}; \
             GUARD-DECLINED (left straight by design): max {dec_max:.3} rms {dec_rms:.3}"
        );
        out
    }

    /// **The rung-3 payoff gate** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §4.3 + §4.4):
    /// projecting the bonded-face boundary **midsides** onto the real endplate seats the
    /// quadratic face on the bone — measured straight-vs-curved, like for like, in one run.
    ///
    /// A straight quadratic midside sits at the chord midpoint of its edge, so it chords across
    /// the curved endplate *even when both of its parent corners are seated exactly on the
    /// bone*. Closing that gap is the whole of rung 3, and it is the last geometric step of
    /// "exact geometry IS the exact physics" for this disc: after it, the bonded face is
    /// genuinely curved between its nodes rather than a fan of flat chords.
    ///
    /// ## Like for like, and why the aggregate would have been self-deception
    ///
    /// Rung 2 committed the "before" arm as a single aggregate over **all 1562** bonded-face
    /// boundary midsides (max 12.464 / RMS 3.375 mm) and flagged the trap on the way past: that
    /// number is dominated by the overhanging annular rim, which stays straight **by design**
    /// (#701's settled call — Sharpey's fibres attach to the ring apophysis, not the endplate
    /// face). Quoting a drop in it would be measuring how much rim happens to be in the average.
    /// So the payoff assert is on the **authorised** midsides — the ones the anatomy
    /// discriminator (`bonded_conform_target`, the same primitive the corner conform uses) says
    /// should seat — exactly as `conform_seats_the_bonded_face_on_the_bone_fom` does for
    /// corners. The declined population is reported, never averaged in, and the all-candidate
    /// aggregate is kept as the continuity check against rung 2's committed number.
    ///
    /// ## The validity gate is written so it cannot be tautological (§4.4)
    ///
    /// v1's "0 inverted / 0 sliver" could not fail: the back-off makes the straight position
    /// always feasible, so non-inversion is a *construction guarantee*. Its two replacements are
    /// both falsifiable — **coverage** (`projection_coverage`: what fraction reached its FULL
    /// projection, since both projection helpers back off silently) and **element validity over
    /// every element** (`worst_gauss_det_ratio`, whose doc comment states precisely what it
    /// catches that the projector's own guarantee does not).
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    // Five helpers are already extracted above (`curved_disc_arms`, `midside_split`,
    // `midside_residuals`, `projection_coverage`, `worst_gauss_det_ratio`); what is left is one
    // gate's linear narrative — three populations, two arms, four committed statistics — and
    // splitting it further would scatter numbers away from the prose that justifies them.
    #[allow(clippy::too_many_lines)]
    fn curved_tet10_midsides_seat_on_the_endplate_fom() {
        use cf_fsu_geometry::{load_from_env, segment_frame};

        let (l4, l5) = (
            load_from_env("CF_L4_STL").unwrap(),
            load_from_env("CF_L5_STL").unwrap(),
        );
        let disc_mesh = load_from_env("CF_DISC_STL").unwrap();
        let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
        let params = DiscParams::default();
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: segment_frame(&l4, &l5, &o4, &o5).unwrap().superior_axis,
        };

        let CurvedArms {
            prepared,
            straight,
            curved,
            moves,
        } = curved_disc_arms(disc_mesh.clone(), &params, ep);

        // (0) FAITHFULNESS: the shipped builder bonds *this* mesh. Without it the gate could
        // measure a geometry no production path ever produces (and it also re-checks the
        // re-derivation against the built disc node-for-node, midsides included).
        //
        // ★★ **This gate has teeth nothing else in the crate has, and that is measured, not
        // asserted.** Disabling the midside projection in `build_bonded_disc_tet10` outright —
        // the whole rung reverted, silently — leaves **six of the seven** licence-gated anatomy
        // gates GREEN: rung 2's corner residual gate, rung 1's element-order FOM, #701's
        // conform moment-rotation FOM, the replayable sweep, and both lofted-disc gates all
        // pass, because none of them observes a midside. This test fails on the assert below,
        // and the license-free `curved_midsides_seat_on_a_curved_synthetic_endplate` fails too,
        // so CI catches it as well. (Per rung 2's lesson, a mutation only proves something if it
        // SURVIVES the pre-existing asserts — this one survives all of them.)
        assert_builder_bonds_this_mesh(
            &build_bonded_disc_tet10(disc_mesh, &params, Some(ep))
                .expect("curved Tet10 disc bonds"),
            &curved,
            prepared.center_native,
            params.scale,
        );

        // (1) POPULATION, recomputed test-side from the six-node boundary faces rather than
        // read back from the production selector, then split by INTENT (the discriminator) —
        // the corner gate's `ConformSplit` shape, one level up.
        let band: std::collections::HashSet<VertexId> = prepared
            .inferior
            .iter()
            .chain(&prepared.superior)
            .copied()
            .collect();
        let split = midside_split(&straight, &band, ep, prepared.center_native, params.scale);
        let move_ids: BTreeSet<VertexId> = moves.iter().map(|&(v, _)| v).collect();
        assert_eq!(
            move_ids, split.authorised,
            "the production selection must be exactly the authorised candidates",
        );
        assert_only_named_midsides_moved(&straight, &curved, &moves);

        // (2) RESIDUAL, straight vs curved, on three populations.
        let residuals = midside_residuals(
            &split,
            &straight,
            &curved,
            (&o4, &o5),
            prepared.center_native,
            params.scale,
        );
        let ((au_max_s, au_rms_s), (au_max_c, au_rms_c)) = residuals.authorised;
        let ((all_max_s, all_rms_s), (all_max_c, all_rms_c)) = residuals.all;
        let (_, dec_rms) = residuals.declined;

        // (3) COVERAGE + ELEMENT VALIDITY (§4.4).
        let (delivered, max_move, mean_move) =
            projection_coverage(&straight, &curved, &moves, params.scale);
        let worst_det = worst_gauss_det_ratio(&curved, &straight);
        println!(
            "projection: {:.1} % delivered in full, max move {max_move:.3} mm, mean {mean_move:.3} mm; \
             worst detJ/detJ_rest over every element and Gauss point {worst_det:.4}",
            100.0 * delivered,
        );

        // (4) NON-VACUITY, committed exactly — the population split, so a silent change in the
        // mesh, the band rule, the boundary-face selection or the discriminator fails here
        // rather than diluting the statistics below.
        //
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): of 1562 bonded-face boundary
        // midsides the SI-alignment guard authorises 580; the other 982 are the overhanging
        // annular rim, left straight on purpose. (37 % authorised, close to but not the same as
        // the 40 % the guard authorises among the corners at rung 2 — 233 of 583 — which is what
        // a rim occupying a slightly larger share of the midside population looks like.)
        assert_eq!(
            (
                split.candidates.len(),
                split.authorised.len(),
                split.declined.len()
            ),
            (1562, 580, 982),
            "the bonded-face boundary midside population split changed",
        );

        // (5) THE PAYOFF: the midsides the discriminator intended to seat end up strictly closer
        // to the bone, on both the extreme and the population statistic.
        assert!(
            au_max_c < au_max_s && au_rms_c < au_rms_s,
            "the authorised midsides must end up closer to the bone \
             (max {au_max_s:.3} -> {au_max_c:.3}, rms {au_rms_s:.3} -> {au_rms_c:.3} mm)",
        );
        // The whole candidate population improves, and no midside is pushed further off the
        // bone. The max is only required not to INCREASE: it is set by a guard-declined rim
        // midside, whose residual is identical in both arms by construction.
        assert!(
            all_rms_c < all_rms_s,
            "the bonded face as a whole must end up closer to the bone \
             (rms {all_rms_s:.3} -> {all_rms_c:.3} mm)",
        );
        assert!(
            all_max_c <= all_max_s,
            "no bonded-face midside may end up further from the bone \
             (max {all_max_s:.3} -> {all_max_c:.3} mm)",
        );

        // (6) COMMITTED VALUES, two-sided at ±5 % (the #701 / rung-1 / rung-2 shape).
        //
        // COMMITTED (as above), residual in mm:
        //   AUTHORISED       straight max 3.973  rms 0.881  ->  curved max 3.971  rms 0.767
        //   ALL candidates   straight max 12.464 rms 3.375  ->  curved max 12.464 rms 3.364
        //   GUARD-DECLINED   (unchanged by construction) max 12.464 rms 4.202
        //   COVERAGE         67.4 % delivered in full; max move 1.533, mean 0.141 mm
        //
        // ⚠ RE-ANCHORED at rung 4 with the corner floor (0.05 -> 0.25): the midsides start from
        // conformed CORNERS, so a corner-floor change moves this whole table. The structural
        // result is unchanged — see below.
        //   VALIDITY         worst detJ/detJ_rest 0.4000 over every element and Gauss point
        //
        // Read these honestly, and read the RMS as the payoff — the max is not the story here.
        //
        // **The payoff.** The seated population's RMS distance to the bone falls 0.881 → 0.767
        // mm, and that endpoint is the point: rung 2 left the *corners* of the authorised region
        // at 0.750 mm RMS, so after this rung the midsides sit at essentially the same distance
        // from the bone as the corners they span (0.767 vs 0.750). The bonded face is now uniformly
        // seated instead of seated at its corners and chording between them. That is a smaller
        // relative move than rung 2's (which nearly halved 1.332 → 0.750) for a structural reason,
        // not a disappointing one: the corners were already conformed before enrichment, so a
        // straight midside starts much closer to the bone (0.881) than a raw corner did (1.332).
        //
        // **The max barely moves (3.973 → 3.971), and that is the same fact rung 2 recorded one
        // level down.** The worst authorised midside is one the quality floor refuses to seat —
        // the same reason rung 2's worst authorised corner stopped at 3.833 mm. The assert is a
        // strict decrease, not a target: a projection that stopped seating this node entirely
        // would fail it, but no gate here claims the extreme is closed.
        //
        // **The ALL-candidate straight figures are rung 2's committed baseline reproduced**
        // (1562 midsides, max 12.464 / RMS 3.375 — `straight_tet10_midsides_chord_across_the_
        // endplate_fom`, which this test replaces by measuring both arms in one run). That is
        // what makes this a like-for-like comparison rather than a new measurement of a new
        // population. They barely move because 982 of 1562 midsides are the rim, whose 12.464 mm
        // max is a closest-point artifact of reaching sideways for the vertebral body wall.
        assert_within_5_percent(&[
            (au_max_s, 3.973, "authorised straight max (mm)"),
            (au_rms_s, 0.881, "authorised straight rms (mm)"),
            (au_max_c, 3.971, "authorised curved max (mm)"),
            (au_rms_c, 0.767, "authorised curved rms (mm)"),
            (all_rms_s, 3.375, "all straight rms (mm)"),
            (all_rms_c, 3.357, "all curved rms (mm)"),
            (all_max_s, 12.464, "all straight max (mm)"),
            (dec_rms, 4.200, "guard-declined rms (mm)"),
        ]);

        // (7) §4.4 ELEMENT VALIDITY, asserted FIRST because it is the more severe fact: an
        // element folded over makes every statistic below meaningless. The assert is the
        // INEQUALITY, not the value.
        //
        // ⚠ The value is pinned to the floor by construction whenever any node backs off at all:
        // the bisection converges onto the constraint boundary, so *some* element ends at
        // exactly `quality_floor`. Committing 0.4000 with a two-sided band would therefore be a
        // gate that reads as a measurement and is really a tautology — the very shape §4.4
        // exists to replace. What is falsifiable is the inequality holding over **every**
        // element rather than only over the projector's own incidence map.
        //
        // ★★ **MEASURED, on a mutant that survives every other assert in this test.** Narrowing
        // `with_projected_midsides`' incidence walk from `t[4..10]` to `t[4..7]` — an incidence
        // map that misses elements, exactly the bookkeeping error this sweep exists for — drives
        // the worst ratio to **−9.7870**, i.e. genuinely inverted elements, while the §4.3
        // residual gate above still *improves* (authorised RMS 0.796 → 0.711) and **passes**.
        // A geometry gate cannot see an inverted element; this one can. (On the synthetic arm
        // the same mutant leaves the worst ratio at 0.9307 and shows up only as lost coverage —
        // the back-off never engages at that curvature, so the real disc is where this bites.)
        assert!(
            worst_det >= DISC_MIDSIDE_CONFORM_QUALITY_FLOOR,
            "an element fell below the quality floor at some Gauss point \
             (worst detJ/detJ_rest {worst_det:.4}) — fix the projection, not this gate",
        );
        assert!(
            worst_det < 1.0,
            "no element's rest Jacobian shrank at all (worst ratio {worst_det:.4}) — the \
             projection cannot have engaged, so nothing above is measuring it",
        );

        // (8) §4.4 COVERAGE — the falsifiable half of the validity gate, two-sided.
        //
        // **27.6 % of the authorised midsides do NOT reach their full projection**, and that is
        // the number this statistic exists to make impossible to hide: both projection helpers
        // back off silently, so a "did anything move / how far did the furthest node move" gate
        // would have reported this run as a clean success. The non-delivery is real geometry,
        // not a bug — a midside asked to move 1.388 mm across a 3 mm cell is asked to fold its
        // element, and the quality floor refuses. Compare the corner conform, where 231 of 233
        // authorised nodes were delivered: corner moves are sparser and a Tet4 element's
        // Jacobian is affine, so the floor almost never binds there.
        //
        // ⚠ **A selection variant was measured and REFUTED here, not argued.** The natural
        // hypothesis for the low delivery is that it is concentrated in midsides whose parent
        // corners were themselves guard-declined (a midside pulled onto the bone while its two
        // rim parents stay put has to bow hard). Gating the selection on both parents being
        // authorised gives 484 of 580 midsides at **79.5 %** delivered — barely better — while
        // dropping 96 midsides that were *improving* (their RMS falls 1.477 → 1.198). So the
        // simpler rule ships: project every authorised bonded-face boundary midside.
        // ⚠ Both halves of that comparison were measured at the *corner* floor 0.05, before this
        // rung split the constant (72.4 % ungated vs 79.5 % parent-gated), so read it as the
        // like-for-like pair it is and not against the 67.9 % committed above.
        //
        // ⚠⚠ **And a second mutant refuted this gate's own advertised justification.** (Measured
        // at the rung-3 corner floor 0.05, like the parent-gating pair above — before/after
        // pairs, not today's committed values.) The
        // fraction is *not* the member with the teeth in every direction: a silent 0.2 mm cap
        // inside `with_projected_midsides` (the archetypal "backs off without telling you" bug)
        // moves it the WRONG way, 67.9 % → 68.6 %, because a smaller request is easier to
        // satisfy. What catches that mutant is the pair `max_move` 1.388 → 0.867 and `mean_move`
        // 0.127 → 0.096 — and, firing first, the §4.3 residual pin (authorised curved RMS
        // 0.694 → 0.733 at the then-current corner floor, which still *improves* on the straight
        // arm and so passes the
        // strict-decrease assert; only the two-sided pin sees it). Gate all three, and read the
        // fraction as covering the back-off-engages-more direction only.
        assert_within_5_percent(&[
            (delivered, 0.674, "delivered fraction"),
            (max_move, 1.533, "max midside move (mm)"),
            (mean_move, 0.141, "mean midside move (mm)"),
        ]);
    }

    #[test]
    fn rejects_overlapping_endplate_bands() {
        // band_frac ≥ 0.5 makes the two bands meet at the mid-plane and share vertices;
        // build_bonded_disc must return an error, NOT let the shared-vertex panic escape
        // from BondedSandwich::from_tet_mesh.
        let params = DiscParams {
            band_frac: 0.6,
            ..DiscParams::default()
        };
        let Err(err) = build_bonded_disc(synthetic_disc(), &params, None) else {
            panic!("expected overlapping endplate bands to be rejected");
        };
        assert!(format!("{err}").contains("overlap"), "got: {err}");
    }
}
