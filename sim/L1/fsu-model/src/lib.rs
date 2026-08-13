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
//!   one, which is ~17 % softer in bending because the linear element bending-locks (ratio
//!   0.827; it read ~1/3 / 0.666 before α.1 removed the phantom material that was inflating
//!   the gap — see `src/committed_anchors.rs`). With
//!   endplates supplied, the quadratic arm is genuinely **curved**: its bonded-face
//!   boundary midsides are projected onto the real endplate too, so the bonded face
//!   follows the bone between its nodes instead of chording across it
//!   (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` rung 3);
//! - the **coupled FSU** ([`CoupledFsu`]): assembles the disc (as a
//!   linearised bushing), the ligaments (tendons), and the facets (oriented SDF contact)
//!   into ONE model and solves for the equilibrium pose under an applied moment — the
//!   force-driven, ROM-limited segment, vs rung 7's analytic superposition of the parts.
//!   Since rung 4 it assembles on the **quadratic** element, with the linear arm still
//!   available as [`CoupledFsuTet4`] so the effect of that flip stays measurable. ⚠ Its bonded
//!   disc is **straight**: seating it on the real endplate is rung 4b, deferred on a measured
//!   blocker (the conform inverts a lofted disc at production angles). The *render* surface is
//!   conformed either way — it is the solved disc that is waiting.
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
//! there does not move the number. ⚠ Rung 4 evidenced that as "the linear arm still reads
//! −0.2819 to four decimals"; post-α.1 it reads −0.1175, so quote the *ratio* (0.827, which
//! survived the geometry change) rather than that absolute, which did not.
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
/// **Measured on BOTH geometries.** Drivability is how far the conformed disc can be WALKED
/// from rest — never jumped — in 0.1° warm-started sub-steps, the production step, out to
/// `DRIVABILITY_PROBE_DEG` (0.9°). Fidelity is the authorised bonded-face residual to the real
/// bone on the scanned disc — the arc's payoff metric, which a higher floor costs:
///
/// | floor | lofted Tet4 | lofted Tet10 | scanned corner RMS (mm) | scanned midside RMS |
/// |---|---|---|---|---|
/// | 0.05 | **stalls** | **stalls** | 0.126 | 0.108 |
/// | 0.08 | drives | **stalls** | 0.131 | 0.110 |
/// | 0.10 | drives | **stalls** | 0.136 | 0.111 |
/// | 0.15 | drives | drives | 0.146 | 0.114 |
/// | 0.20 | drives | drives | 0.156 | 0.116 |
/// | **0.25** | **drives** | **drives** | **0.170** | **0.119** |
/// | 0.40 | drives | drives | 0.242 | 0.151 |
///
/// ⚠⚠ **RE-ANCHORED at rung β.** The fidelity columns fell ~4.4× (shipped row 0.750 → 0.170 and
/// 0.767 → 0.111) because α.1 (#714) stopped the mesher emitting material that is not disc, so the
/// conform starts from a much better mesh. **The drivability columns did NOT move** — same cliff
/// position, same verdicts in all seven rows — which is the useful part: the floor's *job* is
/// unchanged by the geometry fix, so the constant does not need re-deriving, only its cost
/// re-quoting. Had the drivability column moved, this constant would be a different decision.
///
/// ⚠⚠ **RE-ANCHORED AGAIN for the reference-corner back-off**, and the shape of that move is the
/// evidence it is the right one. `with_projected_midsides` now measures feasibility at the four
/// reference corners as well as the four Gauss points, so it refuses more often. **Only the
/// midside column moved** — every row up by ~0.010 mm — while the **corner column is identical
/// in all seven rows** (0.126 … 0.242, unchanged to three decimals). That is the localisation:
/// the corner conform is a *Tet4* operation on an affine element, which the midside projector
/// cannot reach, and it did not budge. **The drivability verdicts and the cliff position did not
/// move either**, so once again the floor's job is unchanged and 0.25 needs its cost re-quoted
/// rather than re-derived.
///
/// ★★ **The lofted columns were BIT-IDENTICAL across α.1** — every drivability verdict, and
/// `assert_floor_is_conditioning_only`'s spreads to four decimals (−24.8449 … −24.8269 and
/// −22.3225 … −22.2903). That was the two-geometry design paying off: the lofted disc is
/// *fabricated*, so it never went through the scanned-SDF path α.1 fixed, and it was an
/// untouched control for **that** change.
///
/// ⚠ **It is NOT a control for this one, and saying so would be false.** The lofted disc is
/// *conformed*, so its midsides go through the very projector this rung changed. The Tet4 arm
/// is unmoved (−24.8449 … −24.8269, still bit-identical — Tet4 has no midsides), but the
/// **Tet10 arm moved to −22.2964 … −22.2732**, about 0.12 %. That is expected and it is small:
/// `assert_floor_is_conditioning_only` bounds the spread at 1 % and both arms read 0.07 % /
/// 0.10 %. Read the lofted Tet4 arm as the surviving control — it is the one this change
/// genuinely cannot reach.
///
/// Regenerated by `conform_quality_floor_selection_fom`, whose swept rows ARE this table's
/// rows. ⚠ That gate *prints* every cell but only **asserts** four things: the shipped row
/// drives both arms, 0.05 does not drive both, fidelity is monotone across rows, and the
/// shipped row's two RMS values sit within ±5 %. The other cells are reported, deliberately —
/// a drivability verdict next to the cliff is a converge/refuse boundary and pinning it would
/// flake. So re-run the gate to regenerate this table; do not read it as fully asserted.
///
/// The quadratic element is the stricter constraint (the same direction rung 3 found for
/// midsides), and its cliff is **bracketed in (0.10, 0.15]** — bracketed, not located, because
/// drivability is a converge/refuse boundary and the sweep only samples it. So 0.25 sits 2.5×
/// above the last floor known to stall and **1.67× above the first known to drive**; the
/// conservative reading is the second, and it is still above the 1.6× rung 3 chose. The margin
/// costs **0.044 mm** of RMS residual (0.126 at floor 0.05 → 0.170 at the shipped 0.25), and the
/// conform still cuts the seated population's distance to the bone by 3.1× (raw 0.530 → 0.170 mm).
/// ⚠ Both figures re-anchored at rung β; they read 0.094 mm and 1.332 → 0.750 mm before.
///
/// **The floor moves the *conditioning*, not the physics** — measured, and asserted by
/// `assert_floor_is_conditioning_only`: across every floor the lofted disc can actually be
/// driven at, `k_disc` spans **0.07 % (Tet4, −24.8449 … −24.8269)** and **0.10 % (Tet10,
/// −22.2964 … −22.2732)**. That is the claim which makes a quality floor a conditioning knob
/// rather than a modelling parameter, and every `k_disc` anchor downstream would silently
/// inherit a violation of it.
///
/// ⚠ This read "&lt;0.1 %" until rung 4's numeric audit gave it a producer — a figure that was
/// **both wrong and self-contradicting**: the range quoted beside it (−24.83 … −24.86,
/// −22.28 … −22.35) works out to 0.12 % and 0.31 %, so the parenthetical refuted the headline
/// and nothing recomputed either. The measured spread is *tighter* than that old range for a
/// substantive reason: the old one averaged in floors where an arm never reached the probe, so
/// its `k_disc` came from a partial walk. Only drivable floors have a stiffness to compare,
/// which is why the sweep reports `Option`.
///
/// ⚠ **The 0.10 row read "drives" until the gate landed**, because the throwaway sweep that
/// first produced this table walked to 0.86° in nine ~0.0956° steps while production walks in
/// 0.1° steps to 0.9°. A slightly further, slightly coarser walk gives a different answer at
/// the margin — which is precisely why `max_drivable_angle` exists and why every drivability
/// claim in this crate now routes through it.
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
/// ⚠⚠ **The whole table above is a PRE-α.1 measurement at the rung-3 corner floor of 0.05**, and
/// it has now drifted twice: rung 4 raised the corner floor to 0.25 (see
/// [`DISC_CONFORM_QUALITY_FLOOR`]), and rung β's geometry fix moved the residuals again. Since the
/// midsides are projected from *conformed corners*, every row's absolute numbers shift — the
/// shipped row now reads **0.119 mm RMS at 83.1 % delivered** (0.111 mm at 91.0 % before the
/// reference-corner back-off, 0.767 mm at 67.4 % before rung β, and 0.694 mm at 67.9 % when this
/// table was built). Read the rows as the *comparison between floors* they were built to be, not
/// as current absolutes.
///
/// ▶ **NOT re-swept at rung β**, deliberately: re-deriving the midside cliff needs its own sweep
/// of this floor, which is a measurement rung rather than a re-anchor. What rung β did check is
/// that the shipped 0.40 still binds and still drives — `worst detJ/detJ_rest` comes back at
/// exactly 0.4000 on the real disc, so the floor is still the active constraint.
///
/// ▶ **Still not re-swept after the reference-corner back-off, and the case for sweeping it is
/// now stronger.** This floor was chosen against a *Gauss-only* acceptance test; the eight-point
/// test is strictly stricter at the same nominal value, so 0.40 is now conservative in the safe
/// direction — it conditions the geometry at least as well as the table's rows claim, which is
/// why it is sound to ship unchanged. But that also means a **lower** floor might now buy back
/// the fidelity the stricter test costs (the shipped midside RMS rose 0.111 → 0.119 mm). That is
/// a measurement rung, not a re-anchor, and it is deliberately not attempted here.
///
/// The **choice** of 0.40 is re-verified rather than assumed: it drives on both the
/// scanned and the lofted disc at the new corner floor. Re-deriving the midside cliff there is
/// not done, and would only be able to move it in the permissive direction — a better-conditioned
/// corner mesh cannot make a midside floor *more* necessary — so 0.40 keeps at least the margin
/// it was chosen with.
const DISC_MIDSIDE_CONFORM_QUALITY_FLOOR: f64 = 0.4;

/// The two quality floors a conform's back-off runs at.
///
/// Production always uses [`ConformFloors::SHIPPED`], whose fields *are*
/// [`DISC_CONFORM_QUALITY_FLOOR`] and [`DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`] — so threading
/// them through a parameter changes no shipped behaviour. The only other caller is the
/// floor-**selection** gate, which sweeps them.
///
/// ★ **Why this seam exists at all.** Both floors are justified by measured tables in their
/// doc comments, and until rung 4 *neither table could be regenerated by anything in the
/// tree* — they were produced by throwaway scaffolding and then deleted, which is precisely
/// the "committed re-runnable measurement or it is unverified" standard this arc applies to
/// every other number it commits. A production constant chosen on a table nothing re-runs is
/// a constant nobody can check, and silently reverting it would break no test.
#[derive(Clone, Copy, Debug)]
struct ConformFloors {
    /// Floor for the Tet4 corner projection ([`DISC_CONFORM_QUALITY_FLOOR`]).
    corner: f64,
    /// Floor for the Tet10 midside projection ([`DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`]).
    midside: f64,
}

impl ConformFloors {
    /// What every production path uses.
    const SHIPPED: Self = Self {
        corner: DISC_CONFORM_QUALITY_FLOOR,
        midside: DISC_MIDSIDE_CONFORM_QUALITY_FLOOR,
    };
}

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

impl PreparedDisc {
    /// A second bondable copy of this prepared disc.
    ///
    /// **Cloning the tet mesh is the point.** Re-running `prepare_disc` to obtain a second arm
    /// rebuilds the exact SDF, re-stuffs the BCC lattice and re-runs `largest_component` — the
    /// dominant cost of every two-arm gate in this crate, paid to reproduce a mesh that is
    /// deterministic anyway.
    ///
    /// It is also **stricter, not merely cheaper**: rung 1's attribution argument is that the
    /// linear and quadratic arms differ in the element and in nothing else, which today holds
    /// because both call the same constructor and a reader checks that they match. After this
    /// the two arms share one mesh *object*, so the property is structural.
    ///
    /// [`Data`] is not [`Clone`] and does not need to be — it is a pure function of `model` at
    /// rest (`make_data` then `forward`), so re-deriving it is exact rather than approximate,
    /// and the two-body scene makes it free. `rest_upper` is read back from the fresh `Data`
    /// exactly as `prepare_disc_at` does rather than copied across, so the arms cannot end up
    /// disagreeing about the rest pose; the `debug_assert` states that as a checked invariant.
    ///
    /// `#[cfg(test)]`: the production builders each prepare and bond once, so the only callers
    /// that need a second arm from one mesh are the gates that *compare* arms. Shipping it on
    /// the production path with no production consumer would be surface nobody asked for.
    ///
    /// # Errors
    /// Propagates a failure of the two-box scene's forward solve — the same call
    /// `prepare_disc_at` makes, and unreachable for this fixed, valid scene.
    #[cfg(test)]
    fn duplicate(&self) -> Result<Self> {
        let model = self.model.clone();
        let mut data = model.make_data();
        data.forward(&model).context("disc scene forward")?;
        let rest_upper = data.xpos[UPPER];
        debug_assert!(
            (rest_upper - self.rest_upper).norm() < 1e-12,
            "a re-derived rest pose must reproduce the original"
        );
        Ok(Self {
            tet: self.tet.clone(),
            inferior: self.inferior.clone(),
            superior: self.superior.clone(),
            model,
            data,
            rest_upper,
            ml_axis: self.ml_axis,
            center_native: self.center_native,
        })
    }
}

/// [`prepare_disc_at`] at the shipped quality floors.
///
/// `#[cfg(test)]` because the production builders thread their floors explicitly; the gates
/// are the only callers that want "whatever ships".
///
/// # Errors
/// See [`build_bonded_disc`].
#[cfg(test)]
fn prepare_disc(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
) -> Result<PreparedDisc> {
    prepare_disc_at(mesh, params, endplates, ConformFloors::SHIPPED)
}

/// The disc as the BCC stuffer emitted it — **before**
/// [`SdfMeshedTetMesh::largest_component`] filters it down to the main body — plus the
/// frame the rest of the pipeline derives from it.
///
/// Split out of [`prepare_disc_at`] as a **pure move** (no float operation reordered) so
/// the meshing-stability census can see the mesher's whole output, not only the part that
/// survives. Everything downstream of the split still runs exactly where it did.
struct MeshedDisc {
    /// Straight from the stuffer: the main body **plus** every disconnected island.
    raw: SdfMeshedTetMesh,
    /// AABB of the recentred, scaled *surface* — the frame the endplate bands are cut in.
    bbox: Aabb,
    /// The medio-lateral (flexion/extension) axis.
    ml_axis: Vector3<f64>,
    /// The disc AABB centre in native mm — the shared flexion pivot.
    center_native: Point3<f64>,
    /// The exact oracle the mesher consumed, in the same scaled frame as `raw`'s positions.
    ///
    /// Test-only: production drops it the moment the mesh exists. The meshing-stability
    /// census needs it to ask *how deep inside the surface* a discarded island sat, which
    /// is the discriminator between "a sub-cell-thin tapering rim" and "an island anywhere".
    #[cfg(test)]
    sdf: MeshOracle,
}

/// Recentre, scale, and tet-mesh the disc surface — the head of [`prepare_disc_at`], up to
/// but **not including** the `largest_component` filter.
///
/// # Errors
/// Returns an error if the disc's thinnest extent is not its native `z` (a mis-oriented
/// mesh), or if the signed-distance oracle or the tet-mesher fails.
fn mesh_disc_raw(mut mesh: IndexedMesh, params: &DiscParams) -> Result<MeshedDisc> {
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
    Ok(MeshedDisc {
        raw: tet,
        bbox,
        ml_axis,
        center_native,
        #[cfg(test)]
        sdf,
    })
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
fn prepare_disc_at(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
    floors: ConformFloors,
) -> Result<PreparedDisc> {
    let MeshedDisc {
        raw,
        bbox,
        ml_axis,
        center_native,
        ..
    } = mesh_disc_raw(mesh, params)?;
    // A physical disc is one connected solid. Keep the main body: a floating tet component would
    // both scatter the rendered surface and poison the Newton tangent's conditioning, since it
    // carries unconstrained rigid modes.
    //
    // ✅ **THE ISLANDS ARE GONE, and this filter is now a no-op on the shipped disc.** Producer:
    // `mesh_stability_step0_discarded_component_census_fom`, which reads **1 component / 0
    // discarded / 0 islands / 100.00 % of tets kept at every cell** it sweeps (0.00150, 0.00200,
    // 0.00310 m). Kept volume is 96.83–98.97 % of the surface's own volume, rising with
    // refinement — a slight *under*-fill from discretisation, which is the direction a healthy
    // stuffer errs in.
    //
    // ⚠ **History, because the fix is easy to un-learn.** This comment used to record 60–135
    // islands and ~6 % of even the KEPT mesh's volume sitting outside the disc's own AABB, and
    // before that it wrongly called the islands the disc's "sub-cell-thin tapering rim". The real
    // cause was the frame: `oracle` was built on the mesh AFTER the rescale to SI metres, where
    // ~30 % of this disc's triangles fall under the absolute area floor parry uses to decide
    // whether to compute a pseudo-normal. A skipped triangle leaves a zero pseudo-normal, parry's
    // `dot(..) <= 0.0` inside-test reads zero as "inside", and the stuffer fills space that is not
    // disc. **α.1 (#714) fixed exactly that** — `mesh-sdf` now normalises internally, so the frame
    // the caller happens to use no longer decides the sign. That is why the census reads zero.
    //
    // ⚠ **`largest_component` STAYS anyway.** It is no longer load-bearing on this input, but
    // `build_bonded_disc` accepts any disc mesh, and a component filter is the cheap defence
    // against a future one that does fragment. Keeping it is not evidence that fragmenting still
    // happens — the census above is the thing to read for that.
    let mut tet = raw.largest_component();

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
        tet = tet.with_projected_nodes(&moves, floors.corner);
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
    build_bonded_disc_at(mesh, params, endplates, ConformFloors::SHIPPED)
}

/// [`build_bonded_disc`] at explicit quality floors — the seam the floor-selection gate
/// sweeps. `ConformFloors::SHIPPED` reproduces the public builder exactly.
///
/// # Errors
/// As [`build_bonded_disc`].
fn build_bonded_disc_at(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
    floors: ConformFloors,
) -> Result<BondedDisc> {
    Ok(bond_prepared_tet4(
        prepare_disc_at(mesh, params, endplates, floors)?,
        params,
    ))
}

/// Bond an already-[`prepare_disc`]d mesh with the **linear** element.
///
/// Split from [`build_bonded_disc_at`] so a caller that needs both element arms can prepare
/// **once** ([`PreparedDisc::duplicate`]) instead of re-meshing per arm. That is a large cost
/// saving on every two-arm gate, and — more importantly — it makes rung 1's attribution
/// argument structural: the arms then share one mesh *object*, rather than each re-deriving
/// the same geometry from the same constructor and the reader having to check they match.
fn bond_prepared_tet4(p: PreparedDisc, params: &DiscParams) -> BondedDisc {
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
    BondedDisc {
        sandwich,
        ml_axis: p.ml_axis,
        rest_upper: p.rest_upper,
        center_native: p.center_native,
        scale: params.scale,
    }
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
    build_bonded_disc_tet10_at(mesh, params, endplates, ConformFloors::SHIPPED)
}

/// [`build_bonded_disc_tet10`] at explicit quality floors — the seam the floor-selection gate
/// sweeps. `ConformFloors::SHIPPED` reproduces the public builder exactly.
///
/// # Errors
/// As [`build_bonded_disc`].
fn build_bonded_disc_tet10_at(
    mesh: IndexedMesh,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
    floors: ConformFloors,
) -> Result<BondedDisc<Tet10Mesh, Tet10, 10, 4>> {
    Ok(bond_prepared_tet10(
        prepare_disc_at(mesh, params, endplates, floors)?,
        params,
        endplates,
        floors,
    ))
}

/// The **quadratic disc mesh itself**: enrich the prepared linear mesh, then project the
/// bonded-face boundary midsides onto the real endplate if `endplates` is given.
///
/// Split out of [`bond_prepared_tet10`] so that a consumer wanting to *inspect* the shipped
/// Tet10 geometry — rather than bond it — measures the same mesh object the production
/// builder bonds, instead of a replica of these few lines. Its first consumer is the
/// reference-corner rest census
/// (`conformed_disc_reference_corner_rest_census_fom`), which asks whether this mesh
/// contains elements that are folded in their own *reference* configuration; a replica
/// there could drift from the builder and quietly measure a mesh nobody ships.
fn prepared_tet10_mesh(
    p: &PreparedDisc,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
    floors: ConformFloors,
) -> Tet10Mesh {
    let tet10 = Tet10Mesh::from_tet4(&p.tet);
    let Some(ep) = endplates else {
        return tet10;
    };
    let moves = endplate_midside_conform_moves(
        &tet10,
        &p.inferior,
        &p.superior,
        ep,
        p.center_native,
        params.scale,
    );
    tet10.with_projected_midsides(&moves, floors.midside)
}

/// Bond an already-[`prepare_disc`]d mesh with the **quadratic** element: enrich, project the
/// bonded-face boundary midsides if `endplates` is given, widen the band to a full-face tie.
///
/// The quadratic half of the prepare-once/bond-twice split — see [`bond_prepared_tet4`].
fn bond_prepared_tet10(
    p: PreparedDisc,
    params: &DiscParams,
    endplates: Option<EndplateConform>,
    floors: ConformFloors,
) -> BondedDisc<Tet10Mesh, Tet10, 10, 4> {
    let tet10 = prepared_tet10_mesh(&p, params, endplates, floors);
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
    BondedDisc {
        sandwich,
        ml_axis: p.ml_axis,
        rest_upper: p.rest_upper,
        center_native: p.center_native,
        scale: params.scale,
    }
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

/// Committed measurement anchors shared with this crate's integration tests.
///
/// See the module docs: gated on `any(test, feature = "test-fixtures")` so `tests/` can see it
/// without the constants becoming public API of a shipped crate.
#[cfg(any(test, feature = "test-fixtures"))]
pub mod committed_anchors;

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

    use cf_fsu_geometry::ConformDecision;
    use cf_geometry::Sdf;

    use nalgebra::SMatrix;

    use super::test_support::{box_mesh, sphere_mesh, synthetic_disc};
    use super::*;

    // The committed disc-stiffness table, shared with `tests/conform_delta_by_element.rs`.
    // `rung5_step1_mesh_realization_noise_floor_fom` below cross-checks its sweep's shipped
    // row against the raw column, so the two gates must pin the same numbers by construction.
    use crate::committed_anchors::{COMMITTED_TET4_RAW, COMMITTED_TET10_RAW};

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

    /// `(largest flexion angle survived, restoring moment there)` for a disc **walked** in
    /// production sub-steps until the fail-closed solver refuses — capped at `limit`.
    ///
    /// The moment is what makes "this knob is *only* a conditioning knob" checkable: if the
    /// physics moved with a quality floor, `moment / angle` would move with it too. That claim
    /// sits in [`DISC_CONFORM_QUALITY_FLOOR`]'s doc and had no producer until this returned it.
    ///
    /// ★ **One definition of "drives", because until rung 4 there were several.** The
    /// drivability columns of `DISC_CONFORM_QUALITY_FLOOR`'s and
    /// `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`'s tables, and rung 4b's entry criterion, were each
    /// produced by a bespoke walker with its own probe angle and step rule (0.86° vs 0.9°,
    /// `ceil()` steps vs a fixed count). They reported the same word for measurably different
    /// experiments, so the tables could not be compared to one another. They can now.
    ///
    /// Walks in [`crate::coupled::CAPTURE_SUBSTEP`] increments — the step
    /// `CoupledFsu::capture_ramp` actually uses — so "drivable" means drivable by production,
    /// not by a test-only regime.
    ///
    /// Takes the disc **by value**: the soft solver is fail-closed, so a refusal unwinds out of
    /// `set_flexion` and leaves the disc indeterminate. Owning it makes reuse impossible rather
    /// than merely discouraged.
    // Sub-step counts are small; the usize→f64 casts are exact.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn drivable_envelope<Msh, E, const N: usize, const G: usize>(
        mut disc: BondedDisc<Msh, E, N, G>,
        limit: f64,
    ) -> (f64, f64)
    where
        Msh: Mesh,
        E: Element<N, G> + Default,
    {
        let steps = (limit.abs() / crate::coupled::CAPTURE_SUBSTEP)
            .ceil()
            .max(1.0) as usize;
        let (mut reached, mut moment) = (0.0_f64, 0.0_f64);
        // The refusal IS the measurement: an `Err` means the solver fail-closed one
        // sub-step past `reached`, which is precisely what this reports. Discarding the
        // `Result` is deliberate.
        drop(std::panic::catch_unwind(std::panic::AssertUnwindSafe(
            || {
                for s in 1..=steps {
                    let theta = limit * (s as f64 / steps as f64);
                    moment = disc.flexion_moment(theta).0;
                    reached = theta;
                }
            },
        )));
        (reached, moment)
    }

    /// [`drivable_envelope`]'s angle alone, for callers that only ask "how far".
    fn max_drivable_angle<Msh, E, const N: usize, const G: usize>(
        disc: BondedDisc<Msh, E, N, G>,
        limit: f64,
    ) -> f64
    where
        Msh: Mesh,
        E: Element<N, G> + Default,
    {
        drivable_envelope(disc, limit).0
    }

    /// One swept row: `(floor, Tet4 k_disc if it drove, Tet10 k_disc if it drove, scanned
    /// corner RMS, scanned midside RMS)`. `None` means that arm refused the probe.
    type FloorRow = (f64, Option<f64>, Option<f64>, f64, f64);

    /// The floor rows both quality-floor tables are built from.
    ///
    /// The gate's list and the doc tables' rows are the **same list**, so a row cannot appear
    /// in a constant's justification without this test regenerating it.
    const SWEPT_CORNER_FLOORS: [f64; 7] = [0.05, 0.08, 0.10, 0.15, 0.20, 0.25, 0.40];
    /// The probe angle the drivability column is measured at — `K_DISC_PROBE` rounded up, since
    /// surviving the production `k_disc` probe is the property that matters.
    const DRIVABILITY_PROBE_DEG: f64 = 0.9;

    /// `(Tet4 drives, Tet10 drives)` for a **conformed lofted** disc at `floors`, walked to
    /// `probe` in production sub-steps.
    ///
    /// ONE prepare feeds both arms ([`PreparedDisc::duplicate`]), so the elements are compared
    /// over one mesh *object* rather than two identically-built ones — and the expensive SDF +
    /// BCC stuffing is paid once per floor instead of twice.
    fn lofted_drivability(
        lofted: &IndexedMesh,
        params: &DiscParams,
        ep: EndplateConform,
        floors: ConformFloors,
        probe: f64,
    ) -> (Option<f64>, Option<f64>) {
        let p = prepare_disc_at(lofted.clone(), params, Some(ep), floors).unwrap();
        let lin = bond_prepared_tet4(p.duplicate().unwrap(), params);
        let quad = bond_prepared_tet10(p, params, Some(ep), floors);
        // `Some(k_disc)` when the arm reached the probe, `None` when it refused — so no caller
        // can read a stiffness off a disc that never got there.
        let k = |(reached, moment): (f64, f64)| -> Option<f64> {
            (reached >= probe - 1e-12).then(|| moment / reached)
        };
        (
            k(drivable_envelope(lin, probe)),
            k(drivable_envelope(quad, probe)),
        )
    }

    /// `(authorised corner RMS, authorised midside RMS)` in mm for the **scanned** disc at
    /// `floors` — what a stricter floor costs in exact-geometry fidelity.
    ///
    /// Both columns come from one conformed prepare, so the corner and midside populations are
    /// measured over the same mesh. `moves` IS the authorised midside set: a midside is in it
    /// exactly when the discriminator gave it a target.
    fn scanned_fidelity(
        scanned: &IndexedMesh,
        params: &DiscParams,
        ep: EndplateConform,
        floors: ConformFloors,
        o4: &MeshOracle,
        o5: &MeshOracle,
    ) -> (f64, f64) {
        let raw = build_bonded_disc_at(scanned.clone(), params, None, floors).unwrap();
        let cp = prepare_disc_at(scanned.clone(), params, Some(ep), floors).unwrap();
        let conf = bond_prepared_tet4(cp.duplicate().unwrap(), params);
        let (p_raw, p_conf) = (raw.deformed_nodes_native(), conf.deformed_nodes_native());
        let split = conform_split(&conf, &p_raw, &p_conf, ep);
        let corner_rms = residual_stats(
            &split
                .authorised
                .iter()
                .map(|&v| endplate_residual(p_conf[v as usize], o4, o5))
                .collect::<Vec<_>>(),
        )
        .1;

        let straight = Tet10Mesh::from_tet4(&cp.tet);
        let moves = endplate_midside_conform_moves(
            &straight,
            &cp.inferior,
            &cp.superior,
            ep,
            cp.center_native,
            params.scale,
        );
        let curved = straight.with_projected_midsides(&moves, floors.midside);
        let pos = curved.positions();
        let midside_rms = residual_stats(
            &moves
                .iter()
                .map(|&(v, _)| {
                    endplate_residual(cp.center_native + pos[v as usize] / params.scale, o4, o5)
                })
                .collect::<Vec<_>>(),
        )
        .1;
        (corner_rms, midside_rms)
    }

    /// A quality floor must move the **conditioning**, not the physics: `k_disc` has to hold
    /// still across every floor the disc can actually be driven at.
    ///
    /// This is `DISC_CONFORM_QUALITY_FLOOR`'s central claim, and until rung 4's audit it had no
    /// producer at all — the "<0.1 %" in its doc came from a throwaway diagnostic that was
    /// deleted. If a quality floor were quietly changing the disc's stiffness, every `k_disc`
    /// anchor downstream would silently inherit it, so the claim is worth checking rather than
    /// repeating.
    fn assert_floor_is_conditioning_only(rows: &[FloorRow]) {
        for (element, ks) in [
            ("Tet4", rows.iter().filter_map(|r| r.1).collect::<Vec<_>>()),
            ("Tet10", rows.iter().filter_map(|r| r.2).collect::<Vec<_>>()),
        ] {
            assert!(
                ks.len() >= 2,
                "{element}: need two drivable floors to compare"
            );
            let (lo, hi) = ks
                .iter()
                .fold((f64::MAX, f64::MIN), |(l, h), &k| (l.min(k), h.max(k)));
            let spread = (hi - lo).abs() / lo.abs();
            println!(
                "[{element}] k_disc across drivable floors: {lo:+.4} … {hi:+.4} ({:.2} %)",
                100.0 * spread
            );
            assert!(
                spread < 0.01,
                "{element}: k_disc varies {:.2} % across the drivable floors ({lo:+.4} … \
                 {hi:+.4}). A quality floor is supposed to move the CONDITIONING, not the \
                 physics; if it moves the stiffness this much it is a modelling parameter and \
                 must be justified as one.",
                100.0 * spread
            );
        }
    }

    /// **Regenerates the measured tables in `DISC_CONFORM_QUALITY_FLOOR` and
    /// `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`** — the gate those two constants were chosen
    /// without.
    ///
    /// ★ **Why this exists.** Both are production constants justified by measured tables in
    /// their doc comments, and until rung 4 *nothing in the tree could regenerate either*: rung
    /// 3's seven rows and rung 4's two-geometry sweep were both produced by throwaway
    /// scaffolding that was then deleted. That fails this arc's own standard — **a committed
    /// re-runnable measurement, or it is unverified** — and the cost is concrete: either floor
    /// could be reverted to a value that ships an undrivable mesh, and no test would notice.
    /// Rung 4 found exactly that, the hard way, from a panic in an unrelated gate.
    ///
    /// **Two geometries, because that is the finding.** The shipped 0.05 was measured on the
    /// **scanned** disc alone and does not transfer to a **lofted** (painted) one — the geometry
    /// `cf-spine-studio` actually builds. So drivability is measured on the lofted disc (the
    /// strict one) and fidelity on the scanned disc (where the conform has real work to do).
    ///
    /// ⚠ The asserts deliberately land on rows **clear of the cliff** (0.05 and the shipped
    /// value, against a cliff at 0.10). The cliff-adjacent row is reported, never asserted: a
    /// drivability cell is a converge/refuse boundary, so a row sitting on it could flip on a
    /// numerics change without anything being wrong.
    #[test]
    #[ignore = "needs $CF_L4_STL $CF_L5_STL $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn conform_quality_floor_selection_fom() {
        use cf_fsu_geometry::{load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let scanned = load_from_env("CF_DISC_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };
        let lofted = lofted_disc(&l4, &l5);
        let params = DiscParams::default();
        let probe = DRIVABILITY_PROBE_DEG.to_radians();

        println!(
            "\n{:>6} {:>13} {:>13} {:>15} {:>15}",
            "floor", "Tet4 k_disc", "Tet10 k_disc", "corner RMS mm", "midside RMS mm"
        );
        let rows: Vec<FloorRow> = SWEPT_CORNER_FLOORS
            .iter()
            .map(|&corner| {
                let floors = ConformFloors {
                    corner,
                    midside: DISC_MIDSIDE_CONFORM_QUALITY_FLOOR,
                };
                let (t4, t10) = lofted_drivability(&lofted, &params, ep, floors, probe);
                let (corner_rms, midside_rms) =
                    scanned_fidelity(&scanned, &params, ep, floors, &o4, &o5);
                let cell =
                    |k: Option<f64>| k.map_or_else(|| "STALLS".to_string(), |v| format!("{v:+.3}"));
                println!(
                    "{corner:>6.2} {:>13} {:>13} {corner_rms:>15.3} {midside_rms:>15.3}",
                    cell(t4),
                    cell(t10),
                );
                (corner, t4, t10, corner_rms, midside_rms)
            })
            .collect();

        let row = |f: f64| -> &FloorRow {
            rows.iter()
                .find(|r| (r.0 - f).abs() < 1e-12)
                .expect("swept row")
        };

        // ── (1) The shipped floor drives BOTH elements on the strict geometry. ──
        let shipped = row(DISC_CONFORM_QUALITY_FLOOR);
        assert!(
            shipped.1.is_some() && shipped.2.is_some(),
            "the SHIPPED corner floor {DISC_CONFORM_QUALITY_FLOOR} must drive a conformed lofted \
             disc on both elements — the property it was raised to guarantee"
        );

        // ── (2) The raise EARNS ITS KEEP: the value it replaced does not drive. ──
        //
        // Without this, the constant could be reverted to 0.05 and every other gate in the
        // crate would stay green — which is exactly the state that let an undrivable mesh reach
        // a shipped default in the first place. This assert IS the floor-regression mutant,
        // committed, rather than a mutation someone has to remember to re-run by hand.
        assert!(
            !(row(0.05).1.is_some() && row(0.05).2.is_some()),
            "0.05 must NOT drive both elements on a lofted disc. If it now does, the reason this \
             floor was raised has gone away and the constant should be re-derived — not kept out \
             of habit, and not widened because a table says so."
        );

        // ── (3) Fidelity is monotone in the floor: a stricter back-off cannot seat BETTER. ──
        for w in rows.windows(2) {
            assert!(
                w[1].3 >= w[0].3 - 1e-9 && w[1].4 >= w[0].4 - 1e-9,
                "a stricter floor cannot reduce the residual: corner {:.3} -> {:.3}, midside \
                 {:.3} -> {:.3} across floors {:.2} -> {:.2}",
                w[0].3,
                w[1].3,
                w[0].4,
                w[1].4,
                w[0].0,
                w[1].0
            );
        }

        assert_floor_is_conditioning_only(&rows);

        // ── (4) The shipped row reproduces what rungs 2 and 3 committed. ──
        //
        // ★★ **These two numbers are INHERITED, not measured here.** They are rung 2's
        // `au_rms_conf` (`conform_seats_the_bonded_face_on_the_bone_fom`) and rung 3's `au_rms_c`
        // (`curved_tet10_midsides_seat_on_the_endplate_fom`) — this sweep reaches them by a
        // different route (`scanned_fidelity` over the whole floor sweep rather than a single
        // dedicated build), so agreement is a genuine cross-harness check on both.
        //
        // ⚠ **Which means this assert must never be updated from this test's own output.** Copying
        // the live value here would turn a cross-check into a tautology — the failure mode this
        // repo names "a cross-check must not route through the artifact under test". Re-anchor the
        // two dedicated gates first, then transcribe *their* committed values into this list.
        //
        // ⚠⚠ RE-ANCHORED at rung β from 0.750 / 0.767. Both dedicated gates were re-anchored first
        // and independently, and the sweep then reproduced them **to three decimals** — corner
        // 0.170 and midside 0.111 — which is the evidence that the new values are mutually
        // consistent rather than merely new.
        //
        // ⚠⚠ MIDSIDE RE-ANCHORED AGAIN, 0.111 -> 0.119, for the reference-corner back-off — and
        // the rule above was followed rather than shortcut. The value is transcribed from
        // `curved_tet10_midsides_seat_on_the_endplate_fom`'s committed 0.119, which was measured
        // on its own dedicated arm; this sweep then independently produced **0.1194** through a
        // different driver. That agreement is the cross-harness check. Had 0.1194 been copied
        // from this test's own output the assert would have become a tautology.
        //
        // ★ The corner value is UNCHANGED at 0.170, and that is the localisation rather than an
        // oversight: the corner conform is a Tet4 operation on an affine element, which the
        // midside projector cannot reach.
        assert_within_5_percent(&[
            (shipped.3, 0.170, "shipped-floor authorised corner RMS (mm)"),
            (
                shipped.4,
                0.119,
                "shipped-floor authorised midside RMS (mm)",
            ),
        ]);
    }

    /// **Rung 4b's entry criterion, made checkable** — how far a *conformed* lofted disc can be
    /// driven before an element inverts.
    ///
    /// Rung 4 ships `CoupledFsu` on a STRAIGHT quadratic disc because seating it breaks this
    /// geometry at production angles. That deferral is only honest if the blocker is a measured
    /// number rather than a sentence, and only useful if fixing it forces the docs to change —
    /// so this pins the angles reached. When 4b improves the conform, these pins fire and the
    /// plan, `CoupledFsu::build`'s table and this test must all be re-anchored together.
    ///
    /// The RAW arms are the control: they reach the full ±6° **in both senses** on the same
    /// mesh, which is what makes this the conform's fault and not the element's or the
    /// geometry's.
    ///
    /// **Measured** (lofted disc, flexion / extension):
    ///
    /// | arm | flexion | extension |
    /// |---|---|---|
    /// | raw Tet4 | +6.00° | −6.00° |
    /// | raw Tet10 | +6.00° | −6.00° |
    /// | conformed Tet4 | **+3.70°** | −6.00° |
    /// | conformed Tet10 | **+2.40°** | **−2.80°** |
    ///
    /// ⚠ Driving both senses is not decoration. Tet4's conform fails in flexion *only* — it
    /// takes the full extension travel — and Tet10's worst direction is **flexion at +2.40°**,
    /// not the −2.80° extension figure this gate reported while it drove each arm in one sense.
    /// The earlier single-sense version also compared a +6° control against a −6° subject,
    /// differing in two variables at once.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn lofted_conformed_disc_angle_envelope_fom() {
        use cf_fsu_geometry::{load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };
        let disc = lofted_disc(&l4, &l5);
        let params = DiscParams::default();
        let full = 6.0_f64.to_radians();

        // ⚠ EVERY arm is driven in the SAME direction. An earlier version drove the raw arms
        // and the conformed Tet4 to +6° but the conformed Tet10 to −6°, so the control and the
        // subject differed in *two* variables (conform AND sense) — the precise attribution
        // error this rung was split to avoid. Extension is the direction the quadratic arm
        // fails soonest, so it is the one that must carry a matched control.
        let arms = |endplates: Option<EndplateConform>, limit: f64| -> (f64, f64) {
            let p =
                prepare_disc_at(disc.clone(), &params, endplates, ConformFloors::SHIPPED).unwrap();
            let t4 = max_drivable_angle(bond_prepared_tet4(p.duplicate().unwrap(), &params), limit);
            let t10 = max_drivable_angle(
                bond_prepared_tet10(p, &params, endplates, ConformFloors::SHIPPED),
                limit,
            );
            (t4, t10)
        };
        // Control: both RAW arms reach the full envelope, in both senses.
        let (raw4_flex, raw10_flex) = arms(None, full);
        let (raw4_ext, raw10_ext) = arms(None, -full);
        // The blocker: the CONFORMED arms stop short, measured in the same two senses.
        let (conf4, conf10_flex) = arms(Some(ep), full);
        let (conf4_ext, conf10) = arms(Some(ep), -full);
        let raw4 = raw4_flex.abs().min(raw4_ext.abs());
        let raw10 = raw10_flex.abs().min(raw10_ext.abs());

        println!(
            "lofted envelope (deg), flexion / extension:\n  raw      Tet4 {:+.2} / {:+.2}   \
             Tet10 {:+.2} / {:+.2}\n  conformed Tet4 {:+.2} / {:+.2}   Tet10 {:+.2} / {:+.2}",
            raw4_flex.to_degrees(),
            raw4_ext.to_degrees(),
            raw10_flex.to_degrees(),
            raw10_ext.to_degrees(),
            conf4.to_degrees(),
            conf4_ext.to_degrees(),
            conf10_flex.to_degrees(),
            conf10.to_degrees()
        );

        assert!(
            raw4 >= full - 1e-12 && raw10 >= full - 1e-12,
            "the RAW arms must reach the full ±6° — they are the control that makes the \
             conformed arms' failure attributable to the conform"
        );
        // All FOUR conformed measurements, because the criterion is "±6° on both elements" and
        // three of the four fall short in different ways: Tet4 fails in flexion but reaches the
        // full extension travel, while Tet10 fails in both senses and its WORSE direction is
        // flexion (+2.40°), not the −2.80° extension figure this gate reported before it drove
        // both senses. Asserting only two of the four would have kept that hidden.
        assert!(
            conf4 < full && conf4_ext.abs() < full || conf10_flex < full && conf10.abs() < full,
            "rung 4b's entry criterion is that a CONFORMED lofted disc completes ±6° on BOTH \
             elements in BOTH senses. Measured: Tet4 {:+.2}°/{:+.2}°, Tet10 {:+.2}°/{:+.2}°. If \
             all four now reach ±6°, 4b's blocker is gone: seat `CoupledFsu`'s disc, re-anchor \
             RUNG7_K_DISC, and DELETE this assert rather than relaxing it.",
            conf4.to_degrees(),
            conf4_ext.to_degrees(),
            conf10_flex.to_degrees(),
            conf10.to_degrees()
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

    /// The worst `detJ / detJ_rest` over **every element and all eight sample points** of a
    /// curved mesh — the four Gauss points and the four reference corners — measured against
    /// its straight twin.
    ///
    /// This is rung 3's element-validity oracle (plan §4.4), and it is deliberately *not*
    /// [`Mesh::quality`]: `Tet10Mesh` never recomputes `QualityMetrics` after a midside moves,
    /// and those metrics are four-corner quantities anyway, so `quality()` is structurally
    /// blind to midside-induced degeneracy. [`Element::rest_jacobian_dets`] is not — and it
    /// varies per Gauss point precisely when an element is curved
    /// (`sim_soft::element::tet10`'s `rest_jacobian_dets_vary_per_gauss_point_when_curved`).
    ///
    /// ⚠⚠ **The reference corners are here because the Gauss points alone were measured
    /// insufficient, on this very mesh.** While this oracle swept the four Stroud points only,
    /// it reported a healthy `0.4000` on a conformed disc that
    /// `conformed_disc_reference_corner_rest_census_fom` found contained **18 elements folded
    /// at a reference corner** — because every Stroud point is strictly interior and none of
    /// them bounds the corner region. An oracle blind to the defect its own gate exists to
    /// exclude is worse than no oracle: it reads green and licenses the conclusion. It now
    /// sweeps what the projector's acceptance test sweeps, so the two cannot disagree about
    /// what "valid" means.
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
    fn worst_rest_det_ratio(curved: &Tet10Mesh, straight: &Tet10Mesh) -> f64 {
        let element = Tet10;
        let corner_grads: [SMatrix<f64, 10, 3>; 4] =
            REFERENCE_CORNERS.map(|xi| element.shape_gradients(xi));
        let dets = |mesh: &Tet10Mesh, t: TetId| -> [f64; 8] {
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
            let x_ref = SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k]);
            let gauss = element.rest_jacobian_dets(&x_ref);
            let x_t = x_ref.transpose();
            std::array::from_fn(|i| {
                if i < 4 {
                    gauss[i]
                } else {
                    (x_t * corner_grads[i - 4]).determinant()
                }
            })
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
                (0..8).map(move |q| c[q] / s[q]).collect::<Vec<_>>()
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
    /// (83.1 %, 0.740, 0.076 since the reference-corner back-off; 91.0 %, 0.818, 0.086 between
    /// rung β and it; 67.4 %, 1.533, 0.141 before rung β). Re-running
    /// the mutants at the current floor and geometry would move both sides of every arrow and
    /// change none of the conclusions.
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
        let worst = worst_rest_det_ratio(&curved, &straight);
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
            "an element fell below the quality floor at some sample point — a Gauss point \
             or a reference corner (worst detJ/detJ_rest {worst:.4})",
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
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): the deepest seated node moves
        // **1.734 mm** onto the bone.
        //
        // ⚠⚠ **RE-ANCHORED at rung β from 4.156 mm (4.457 before rung 4 raised
        // `DISC_CONFORM_QUALITY_FLOOR` 0.05 -> 0.25). The 2.4× drop is the mesh, not the conform:**
        // α.1 (#714) stopped the SDF filling space that is not disc, so the raw disc this starts
        // from already sits 3× closer to the bone (rung 2's authorised raw max 5.724 -> 1.921 mm).
        // There is simply less to correct.
        //
        // ★★ **This retires the sentence that used to justify `SI_CONFORM_CAP_BONDED`.** The old
        // text read "a genuine central endplate node the old hard 3–4 mm cap would have clipped,
        // seated by the loose 6 mm backstop" — i.e. this gate was the *evidence* that the loose cap
        // earned its keep. At 1.734 mm **nothing here exercises a cap above ~2 mm**, so that
        // evidence is gone and no test in the tree now demonstrates the 6 mm value is needed.
        // Deliberately NOT retuned: a backstop is sized against inputs not yet seen, and one
        // anatomy is no basis for tightening it. Made loud instead — see
        // `conform_seats_the_bonded_face_on_the_bone_fom`'s `beyond_cap == 0` assert.
        //
        // The band is ±5 % of the measured value, matching the shape used elsewhere in this crate,
        // rather than the old hand-set 3.95..=4.37 window (±5.1 % / +5.2 %, asymmetric for no
        // stated reason).
        assert!(
            (1.647..=1.821).contains(&max_seat),
            "the conform must seat nodes onto the bone (committed 1.734 mm), got {max_seat:.3} mm"
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
        // COMMITTED: both arms read k_disc ≈ **−0.117 N·m/rad** each way (restoring); seating the
        // band onto the real endplate shifts it ~2 % (−0.117 → −0.115 flexion). The FOM is
        // essentially unchanged by the conform — the exact-geometry win is that the bond ties to
        // the REAL bone, not a moment shift, which is why the gate is soundness + no-regression,
        // not "improves"; the direct geometric gate is
        // `conform_seats_the_bonded_face_on_the_bone_fom`.
        //
        // ⚠⚠ **RE-ANCHORED at rung β from ≈ −0.28** (−0.2811 flexion / −0.2788 extension). The
        // absolute halved because α.1 changed the retained tet domain, and this crate has already
        // measured that absolute `k_disc` is not a stable quantity:
        // `rung5_step1_mesh_realization_noise_floor_fom` records the absolutes moving **~100 %
        // peak-to-peak** across a ±3.4 % cell window pre-α.1, i.e. every published absolute is one
        // draw from a wide distribution.
        //
        // ★★ **So the band is DERIVED from that measurement, not set at ±5 % of one draw.** The
        // post-α.1 sweep (same gate, cells 0.00290…0.00310) spans Tet4 flexion **−0.1131 …
        // −0.1397**, a 21 % envelope. `-0.150 ..= -0.095` covers it with margin on both sides, so a
        // legitimate re-mesh cannot false-fail this gate while a collapse or sign change still
        // will. The old `-0.34 ..= -0.22` was a hand-set window around a single draw with no stated
        // derivation — narrower in relative terms (±21 %) than the noise it was supposed to tolerate.
        //
        // ✅ **RESOLVED at β.4**, which anchored absolute `k_disc` across the arc. The band is
        // KEPT unchanged: β.4 measured the conformed Tet4 arm at −0.1146, comfortably inside
        // −0.150…−0.095, so the envelope derivation above still holds on measured values. The
        // absolutes now have one producer (`src/committed_anchors.rs`); the hand-copy
        // this note warned about is gone, and it was two files, not three. This band remains a
        // sanity envelope on a conform gate, not the arc's `k_disc` anchor — do not treat it as
        // the latter.
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
                (-0.150..=-0.095).contains(&kc) && (-0.150..=-0.095).contains(&kr),
                "k_disc {name} outside the realization envelope −0.150…−0.095 N·m/rad measured by \
                 rung 5 step 1 (conf {kc:.3}, raw {kr:.3})"
            );
            // No-regression: the seated band shifts the linearised stiffness by < ~10 % (observed
            // ~2.1 % at β.4; ~1.8 % pre-α.1). Loose enough to record a legitimate shift, tight
            // enough to catch a collapse.
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
    /// synthetic slab's ratio is *reported* here, not asserted, because the real-anatomy ratio
    /// (pre-registered as 0.665 at rung 1; **measured** at 0.827 post-α.1) and a 24×20×6 mm box
    /// at `cell = 3 mm` are different bending problems.
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
    /// so the settled claim is that the quadratic element relaxes this bending mode at fixed
    /// geometry — **by ~17 % (ratio 0.827) as of β.4**. ⚠ The 0.665 / ~1/3 figures in this
    /// paragraph are rung 1's, measured before α.1 removed phantom material; the pre-registered
    /// bracket and the first run are kept as the record of how the harness was validated, not as
    /// current values. Phantom slivers inflated Tet4's bend-locking, so the *element* effect was
    /// overstated by the geometry. Earning the accuracy claim is rung 5's h-refinement.
    ///
    /// **The band cross-check, not the ratio, is the assertion with teeth.** An under-tied
    /// band drifts the ratio *toward* the corner-only tie, so a *partial*
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
        // ★ The committed band/node sizes are asserted at the END of this gate, not here.
        // `assert_full_face_band` above is a GUARD — if the band rule is incoherent nothing
        // downstream means anything, so it fails fast. The committed *sizes*, by contrast, are
        // a comparison, and asserting them here aborted the gate in ~1 s before the stiffnesses
        // were ever computed, so a re-anchor could only learn one number per run.

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

        // (4) COMMITTED VALUES — every one of them, after everything above has printed.
        //
        // COMMITTED (BodyParts3D FMA16036, DiscParams::default): the corner bands are 73 / 116
        // and the full-face tie widens them to 276 / 402.
        assert_eq!(
            (inf4, sup4, inf10, sup10),
            (73, 116, 276, 402),
            "committed band sizes changed — the mesh or the band rule moved, and the ratio \
             below is no longer comparable across runs"
        );
        assert_eq!(
            (prepared.tet.n_vertices(), mesh10.n_vertices()),
            (5551, 14296),
            "committed node counts changed (5551 corners -> 14296 with midsides)"
        );

        // DIRECTION + MAGNITUDE, asserted PER DIRECTION (the spike quoted a mean and the
        // two directions differ ~0.5 %, so a mean would hide a one-sided failure).
        //
        // COMMITTED (BodyParts3D FMA16036, DiscParams::default, raw un-conformed mesh, ±0.5°):
        //   Tet4            flex −0.1170  ext −0.1156  N·m/rad
        //   Tet10 full-face flex −0.0968  ext −0.0958  N·m/rad
        //   ratio           flex  0.827   ext  0.829
        //
        // ⚠ RE-ANCHORED at β.4 from bands (228,367,1005,1598), nodes 7849→19449, and ratio
        // 0.666/0.663. α.1 stopped the mesher retaining phantom material: the domain shrank,
        // which is why the band and node counts fell together with the stiffnesses. The
        // absolutes are the shared table in `src/committed_anchors.rs`.
        // ⚠ The bracket below is RETIRED as a validation of the *current* values: post-α.1 the
        // ratio (0.827/0.829) falls outside 0.60..=0.73 and the bands no longer match the spike
        // id-for-id. The harness's live known-value check is now `rung5_step1`'s shipped row,
        // which reproduces this gate's raw column to four decimals through a different probe.
        // The pre-registered bracket was 0.60..=0.73 around the reverted spike's 0.665; both
        // directions landed inside it, so the harness reproduces the spike and the bands
        // (228→1005 / 367→1598) matched it id-for-id AT RUNG 1. The live assert below is the
        // ±5 % no-regression band around these measured values — a change big enough to leave
        // it is a real shift in the element effect, not noise (the bonded moment reproduces to
        // < 1e-3 relative across captures).
        for (k10, k4, expect, name) in [
            (k_flex10, k_flex4, 0.827, "flexion"),
            (k_ext10, k_ext4, 0.829, "extension"),
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

    /// One mesh realization of the disc: what both element arms report off a single prepared
    /// mesh. Rung 5.0 step 1's unit of measurement.
    struct Realization {
        cell: f64,
        corners: usize,
        bands: (usize, usize), // (inferior, superior) pinned corner counts
        linear: (f64, f64),    // (flexion, extension)
        quadratic: (f64, f64), // (flexion, extension)
    }

    impl Realization {
        fn ratio(&self) -> (f64, f64) {
            (
                self.quadratic.0 / self.linear.0,
                self.quadratic.1 / self.linear.1,
            )
        }
    }

    /// `(min, max, mean, peak-to-peak as % of mean)` — what makes rung 5.0 step 1 a
    /// distribution rather than a single delta.
    ///
    /// The `usize -> f64` here is `xs.len()`, the sample count (single digits), so the widening
    /// is exact — nowhere near `f64`'s 2^53 mantissa limit. (Band populations are widened at the
    /// *call sites*, which carry their own justification.)
    #[allow(clippy::cast_precision_loss)]
    fn spread(xs: &[f64]) -> (f64, f64, f64, f64) {
        let (lo, hi) = xs
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &x| {
                (l.min(x), h.max(x))
            });
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        (lo, hi, mean, (hi - lo).abs() / mean.abs() * 100.0)
    }

    /// **Rung 5.0 step 1** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §3): the **mesh-realization
    /// noise floor** — how much `k_disc` moves when the mesh is re-realized at essentially the
    /// same resolution.
    ///
    /// **Nothing in rung 5's convergence table is interpretable without this number.** The BCC
    /// lattice is anchored to the world origin, so the *only* public knob that re-phases it
    /// against fixed geometry is `cell` itself. Sweeping `cell` across a narrow **±3.4 %** window
    /// therefore re-realizes the mesh at nearly-fixed resolution: whatever `k_disc` does across it
    /// is (a smooth h-effect over that window) **+** (re-meshing jitter), and the spread is an
    /// **upper bound on the jitter**. Read it against the ladder's intended signals — 33 % and
    /// 50 % cell steps (those two are step SIZES, unrelated to the element effect) — and against
    /// the ~17 % element effect the rung exists to bound (~33 % as rung 1 measured it, pre-α.1).
    ///
    /// **Five points, not two, and that is load-bearing.** The first version of this measurement
    /// compared a single pair (0.003 vs 0.00305) and reported ~9 %. That is one sample of the
    /// jitter, not a distribution, and it **understated the real spread by ~6×** (post-α.1: that
    /// pair gives 3.4 % against the sweep's 21.13 %; pre-α.1 the same comparison read >10×) — the
    /// sweep measures **21.13 %** peak-to-peak post-α.1 (~100 % pre-α.1, which is the figure
    /// this paragraph carried until β.4). A single perturbation cannot distinguish a typical
    /// lattice phase from a pathological one, and quoting it would have been exactly the
    /// false-precision point this repo's UQ position refuses.
    ///
    /// ⚠ **This is an upper bound, not the jitter itself**, and it cannot separate the two terms:
    /// a short ladder has no way to tell a small smooth slope from realization noise. That is
    /// exactly why rung 5 delivers a bracket rather than an extrapolated `k*` — an order fitted
    /// through differences this size would be fitting noise.
    ///
    /// Both arms at each cell come from **one** prepared mesh (`PreparedDisc::duplicate`), so the
    /// per-cell ratio `k10/k4` is attributable to element order by construction — and the ratio's
    /// spread (**1.81 %**) being far below the absolutes' (**21.13 %**) is the measured
    /// confirmation that
    /// the §3 confounds are largely common-mode in it.
    ///
    /// **Asserts** that the sweep genuinely re-realized the mesh (else it measures nothing), that
    /// every arm conserved, and — the free known-value check — that the `cell = 0.003` row
    /// reproduces rung 1's committed raw-disc numbers, since 0.003 *is* the shipped configuration.
    /// The spread itself is a measurement to read, not a threshold asserted here.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn rung5_step1_mesh_realization_noise_floor_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let base = DiscParams::default();
        let (flex, ext) = (0.5_f64.to_radians(), -0.5_f64.to_radians());

        let mut rows: Vec<Realization> = Vec::new();
        for cell in [0.002_90, 0.002_95, 0.003_00, 0.003_05, 0.003_10] {
            let params = DiscParams { cell, ..base };
            let p = prepare_disc(disc_mesh.clone(), &params, None)
                .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
            let corners = referenced_vertices(&p.tet).len();
            let p_bands = (p.inferior.len(), p.superior.len());

            // ONE mesh, both arms — rung 1's attribution argument, made structural.
            let mut tet4 = bond_prepared_tet4(p.duplicate().expect("duplicate"), &params);
            let mut tet10 = bond_prepared_tet10(p, &params, None, ConformFloors::SHIPPED);

            // `(moment, conservation residual)` per arm per direction.
            let lin_flex = tet4.flexion_moment(flex);
            let lin_ext = tet4.flexion_moment(ext);
            let quad_flex = tet10.flexion_moment(flex);
            let quad_ext = tet10.flexion_moment(ext);
            for (probe, name) in [
                (lin_flex, "Tet4 flexion"),
                (lin_ext, "Tet4 extension"),
                (quad_flex, "Tet10 flexion"),
                (quad_ext, "Tet10 extension"),
            ] {
                assert!(
                    probe.1 < 1e-8,
                    "cell {cell}: {name} bond must conserve (‖ΣF‖+‖ΣM‖ = {:.2e})",
                    probe.1
                );
            }
            rows.push(Realization {
                cell,
                corners,
                bands: (p_bands.0, p_bands.1),
                linear: (lin_flex.0 / flex, lin_ext.0 / ext),
                quadratic: (quad_flex.0 / flex, quad_ext.0 / ext),
            });
        }

        let distinct: std::collections::BTreeSet<usize> = rows.iter().map(|r| r.corners).collect();
        assert!(
            distinct.len() > 1,
            "every cell produced the same referenced-corner count — the mesh was never \
             re-realized, so this measures nothing"
        );

        for r in &rows {
            let (rf, re) = r.ratio();
            println!(
                "cell {:.5} | corners {:5} | bands {:4}/{:4} | Tet4 {:.4}/{:.4} | \
                 Tet10 {:.4}/{:.4} | ratio {rf:.4}/{re:.4}",
                r.cell,
                r.corners,
                r.bands.0,
                r.bands.1,
                r.linear.0,
                r.linear.1,
                r.quadratic.0,
                r.quadratic.1,
            );
        }

        // ★ VALIDATE-AGAINST-A-KNOWN-VALUE, free: cell 0.003 IS the shipped configuration, so
        // this sweep's middle row must reproduce rung 1's committed raw-disc numbers. If it does
        // not, the sweep is a different probe and the spread below describes nothing.
        let shipped = rows
            .iter()
            .find(|r| (r.cell - 0.003).abs() < 1e-12)
            .expect("the sweep must contain the shipped cell");
        for (got, want, name) in [
            (shipped.linear.0, COMMITTED_TET4_RAW.0, "Tet4 flexion"),
            (shipped.linear.1, COMMITTED_TET4_RAW.1, "Tet4 extension"),
            (shipped.quadratic.0, COMMITTED_TET10_RAW.0, "Tet10 flexion"),
            (
                shipped.quadratic.1,
                COMMITTED_TET10_RAW.1,
                "Tet10 extension",
            ),
        ] {
            assert!(
                (got - want).abs() < 5e-4,
                "cell 0.003 {name} = {got:.4}, but rung 1 committed {want:.4} — this sweep is \
                 not probing the shipped configuration"
            );
        }

        let col = |f: fn(&Realization) -> f64| rows.iter().map(f).collect::<Vec<_>>();
        for (label, xs) in [
            ("k4    flex", col(|r| r.linear.0)),
            ("k4    ext ", col(|r| r.linear.1)),
            ("k10   flex", col(|r| r.quadratic.0)),
            ("k10   ext ", col(|r| r.quadratic.1)),
            ("RATIO flex", col(|r| r.ratio().0)),
            ("RATIO ext ", col(|r| r.ratio().1)),
            // Band populations are in the thousands — the widening is exact below 2^53.
            #[allow(clippy::cast_precision_loss)]
            ("sup band  ", col(|r| r.bands.1 as f64)),
            #[allow(clippy::cast_precision_loss)]
            ("inf band  ", col(|r| r.bands.0 as f64)),
        ] {
            let (lo, hi, mean, pp) = spread(&xs);
            println!("{label}: min {lo:.4}  max {hi:.4}  mean {mean:.4}  peak-to-peak {pp:.2} %");
        }
    }

    /// The four **reference corners** of the parametric simplex — `ξ` at the tet's vertices.
    ///
    /// The Stroud quadrature points the mesher's own guard evaluates are all interior
    /// (barycentric weight 0.5854 on the nearest vertex), so a fold confined to a corner
    /// region is invisible to them. These are the points that can see it.
    ///
    /// ⚠ `sim_soft`'s `with_projected_midsides` holds its back-off at these same four points
    /// and defines them privately. **Restated here rather than exported**, deliberately, for
    /// the reason this crate's other oracles give: a gate that imports the constant its
    /// subject is built from stops being an independent check. These are the vertices of the
    /// unit simplex — a definition, not a tunable — so the two cannot drift apart in any way a
    /// reader cannot see at a glance.
    const REFERENCE_CORNERS: [Vec3; 4] = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];

    /// `det J_rest` at each of the four reference corners, **normalised by the element's own
    /// affine determinant** (`det [X₁−X₀, X₂−X₀, X₃−X₀]` = `6·V`).
    ///
    /// The normalisation is what makes the number readable and comparable across a mesh whose
    /// elements differ in size by orders of magnitude: it is exactly **1.0 at every corner of a
    /// straight element**, so a reading is "how much this element's reference map has been bent
    /// at this corner, as a fraction of its affine volume". `≤ 0` is a **fold in the reference
    /// configuration** — the element is inside-out before any load is applied, which no
    /// deformation can excuse.
    ///
    /// ⚠ Deliberately NOT `det F`. At rest `F = I` wherever `J_rest` is invertible, so a
    /// rest-state census that measured `det F` would read 1.0 everywhere and see nothing. The
    /// reference map's own orientation is the rest-state question.
    fn rest_corner_det_ratios(x_ref: &SMatrix<f64, 10, 3>, element: Tet10) -> [f64; 4] {
        let scale = affine_det(x_ref);
        REFERENCE_CORNERS.map(|xi| {
            let j_rest = x_ref.transpose() * element.shape_gradients(xi);
            j_rest.determinant() / scale
        })
    }

    /// `det [X₁−X₀, X₂−X₀, X₃−X₀]` — the element's affine (corner-block) determinant, `6·V`.
    /// The normaliser every ratio in this census is expressed against.
    fn affine_det(x_ref: &SMatrix<f64, 10, 3>) -> f64 {
        SMatrix::<f64, 3, 3>::from_fn(|r, c| x_ref[(c + 1, r)] - x_ref[(0, r)]).determinant()
    }

    /// The ten ordered Tet10 node ids of one element, or `None` on a linear mesh.
    fn tet10_nodes(tet10: &Tet10Mesh, tid: TetId) -> Option<[VertexId; 10]> {
        let c = tet10.tet_vertices(tid);
        let m = tet10.tet_midside_nodes(tid)?;
        Some([c[0], c[1], c[2], c[3], m[0], m[1], m[2], m[3], m[4], m[5]])
    }

    /// One arm's reference-corner rest census.
    struct RestCensus {
        n_tets: usize,
        /// Elements with `det J_rest ≤ 0` at one or more reference corner — folded at rest.
        folded: Vec<(usize, usize, f64)>,
        /// The per-element minimum normalised corner determinant, over the whole mesh.
        worst: f64,
        /// Elements whose minimum ratio is positive but below this fraction of affine.
        near_degenerate: usize,
    }

    fn census_rest_corners(tet10: &Tet10Mesh) -> RestCensus {
        let element = Tet10;
        let positions = tet10.positions();
        let mut folded = Vec::new();
        let mut worst = f64::INFINITY;
        let mut near_degenerate = 0;
        #[allow(clippy::cast_possible_truncation)] // Mesh-trait API tax, as elsewhere here.
        let n_tets = tet10.n_tets();
        for t in 0..n_tets {
            #[allow(clippy::cast_possible_truncation)]
            let tid = t as TetId;
            let Some(nodes) = tet10_nodes(tet10, tid) else {
                continue;
            };
            let x_ref = SMatrix::<f64, 10, 3>::from_fn(|a, k| positions[nodes[a] as usize][k]);
            let ratios = rest_corner_det_ratios(&x_ref, element);
            let mut tet_min = f64::INFINITY;
            for (c, &r) in ratios.iter().enumerate() {
                if !r.is_finite() || r <= 0.0 {
                    folded.push((t, c, r));
                }
                tet_min = tet_min.min(r);
            }
            if tet_min > 0.0 && tet_min < 1.0e-6 {
                near_degenerate += 1;
            }
            worst = worst.min(tet_min);
        }
        RestCensus {
            n_tets,
            folded,
            worst,
            near_degenerate,
        }
    }

    /// Whether the folded elements a census found reach the physics, whether the shipped
    /// guard can see them — and the regression gate that keeps the answer at "none of them".
    ///
    /// Two questions decide what a fold costs, and both are per-element.
    ///
    /// (a) **Free DOFs.** These elements sit in the endplate region, and the `full_face_band`
    ///     tie pins every node it covers. An element whose ten nodes are ALL in the band is
    ///     folded but Dirichlet-clamped: it cannot move, so it contributes nothing to the
    ///     free-DOF system `k_disc` is read from. One with a free node is integrated live, and
    ///     the committed anchors are then measured over an inside-out element. Of the 18 found
    ///     before the fix, 12 were fully pinned and **6 carried free DOFs**.
    ///
    /// (b) **Guard blindness on REAL geometry.** The projector used to accept a placement on
    ///     `Element::rest_jacobian_dets` alone — the four Stroud points, all interior. The
    ///     60 % → 6 % miss-rate table behind that blindness claim was measured on straight-edged
    ///     synthetic tets; on the shipped anatomy **all 18 of 18** were missed, so the real-
    ///     geometry miss rate for this class was 100 %.
    fn report_and_gate_folded(tet10: &Tet10Mesh, p: &PreparedDisc, c: &RestCensus) {
        // ## Do the folded elements reach the physics, and can the shipped guard see them?
        //
        // Two questions decide what this finding costs, and both are per-element.
        //
        // (a) **Free DOFs.** These elements sit in the endplate region, and the
        //     `full_face_band` tie pins every node it covers. An element whose ten nodes
        //     are ALL in the band is folded but Dirichlet-clamped: it cannot move, so it
        //     contributes nothing to the free-DOF system `k_disc` is read from. One with a
        //     free node is integrated live, and the committed anchors are then measured
        //     over an inside-out element.
        //
        // (b) **Guard blindness on REAL geometry.** `Tet10Mesh::with_projected_midsides`
        //     accepts a projection by `Element::rest_jacobian_dets` — the four Stroud
        //     points, all interior. The 60 % → 6 % miss-rate table behind that claim was
        //     measured on straight-edged synthetic tets, so this is the first reading of it
        //     on the shipped anatomy: if every Gauss point of a folded element is positive,
        //     the guard is confirmed blind to exactly what it is supposed to prevent.
        let element = Tet10;
        let positions = tet10.positions();
        let band: BTreeSet<VertexId> = full_face_band(tet10, &p.inferior)
            .into_iter()
            .chain(full_face_band(tet10, &p.superior))
            .collect();
        let folded_tets: BTreeSet<usize> = c.folded.iter().map(|&(t, _, _)| t).collect();
        let (mut fully_pinned, mut with_free_dofs, mut gauss_blind) = (0, 0, 0);
        for &t in &folded_tets {
            #[allow(clippy::cast_possible_truncation)]
            let tid = t as TetId;
            let Some(nodes) = tet10_nodes(tet10, tid) else {
                continue;
            };
            let pinned = nodes.iter().filter(|n| band.contains(n)).count();
            let x_ref = SMatrix::<f64, 10, 3>::from_fn(|a, k| positions[nodes[a] as usize][k]);
            let scale = affine_det(&x_ref);
            let gauss = element.rest_jacobian_dets(&x_ref);
            let min_gauss = gauss.iter().fold(f64::INFINITY, |m, &g| m.min(g / scale));
            if pinned == 10 {
                fully_pinned += 1;
            } else {
                with_free_dofs += 1;
            }
            if min_gauss > 0.0 {
                gauss_blind += 1;
            }
            println!(
                "    tet {t:6}: pinned {pinned:2}/10, min Gauss det ratio {min_gauss:+.6}, \
                 min corner det ratio {:+.6}",
                c.folded
                    .iter()
                    .filter(|&&(ft, _, _)| ft == t)
                    .fold(f64::INFINITY, |m, &(_, _, r)| m.min(r)),
            );
        }
        println!(
            "  of {} folded elements: {fully_pinned} fully pinned by the bond, \
             {with_free_dofs} carry FREE DOFs; {gauss_blind} are invisible to the \
             shipped Gauss-point guard",
            folded_tets.len(),
        );

        // ★ THE REGRESSION GATE. The shipped conformed disc must contain no element folded
        // in its own rest configuration. This was 18 before the projector's back-off was
        // held at the reference corners; it is the reason that change exists, and it is the
        // instrument that fails if the blind metric ever comes back.
        assert!(
            c.folded.is_empty(),
            "{} conformed-disc elements are folded at a reference corner AT REST (worst \
             normalised determinant {:.6}); {with_free_dofs} of them carry free DOFs and \
             {gauss_blind} are invisible to the Gauss-point rule. The midside projection's \
             acceptance test has stopped sampling the corners.",
            folded_tets.len(),
            c.worst,
        );
        // ...and the corner constraint is what binds, which is what makes the assert above
        // non-vacuous: a floor nothing reaches would pass it for the wrong reason.
        assert!(
            (c.worst - DISC_MIDSIDE_CONFORM_QUALITY_FLOOR).abs() < 1.0e-9,
            "the worst normalised corner determinant is {:.6}, not the quality floor {:.6} \
             — if the corners are slack this census no longer witnesses the constraint that \
             keeps them unfolded",
            c.worst,
            DISC_MIDSIDE_CONFORM_QUALITY_FLOOR,
        );
    }

    /// **The reference-corner REST census** of the shipped raw and conformed Tet10 discs —
    /// the instrument that found the fold, and now the regression gate that keeps it gone.
    ///
    /// # What it found, and what fixed it
    ///
    /// A reference-corner validity **stage** was written on
    /// `fix/tet10-inversion-gate-at-gauss-points` and split back out at `cad72838`, because
    /// adding it turned three previously-green licensed gates red — all with the same
    /// signature, `det F` negative at reference corner 2 while every Gauss point and the
    /// affine corner block read positive. That stage fails closed on the **first** violator,
    /// which is the right shape for a gate and the wrong shape for the question it left open:
    /// *are these elements acceptable, and if not, how much of the mesh is affected?* That
    /// needs a distribution, so this counts every element instead of stopping at one.
    ///
    /// It measured **18 of 6256 conformed-disc elements folded at a reference corner**, worst
    /// normalised determinant −0.856, of which **6 carried free DOFs** (the other 12 were
    /// fully pinned by the `full_face_band` tie) and **all 18 were invisible** to the
    /// Gauss-point rule that accepted them. `Tet10Mesh::with_projected_midsides` now holds its
    /// back-off at the reference corners as well, and this reads **0 folded, worst ratio
    /// exactly at the floor** — the constraint binding where it was designed to.
    ///
    /// ⚠ **`k_disc` did not move.** The conformed absolutes reproduce
    /// [`committed_anchors`] to four decimals across the fix, so the anchors measured over
    /// those six live folded elements were not measurably wrong. That is a *measured*
    /// resolution of the provenance question, not a reason the fold was acceptable.
    ///
    /// It is deliberately a **rest-state** measurement. The three red gates observed folds at
    /// deformed states reached through a flexion sweep, which confounds two questions — is the
    /// mesh bad, or does the load path drive good elements bad? The reference map's own
    /// orientation is a property of the mesh alone: an element with `det J_rest ≤ 0` at a
    /// corner is folded before anything is applied to it. What this finds is attributable to
    /// the mesher and to nothing else.
    ///
    /// ⚠ **Scope, and what it does NOT cover.** A straight Tet10 is affine whatever its
    /// corners do, so this sees only **midside-driven** folds; corner-conform damage stays
    /// affine and positive and is invisible here. Nor does it cover folds that a *deformation*
    /// induces in a mesh that is sound at rest — a straight-edged element's `F` is not constant
    /// once it deforms, so that class is real and is the solver-stage question (`cad72838`),
    /// not this one.
    ///
    /// # The negative control is free, and it is the raw arm
    ///
    /// With `endplates = None` every midside stays at its edge midpoint, so the raw disc is
    /// exactly affine and its normalised corner determinant must be **1.0 at all four corners
    /// of all ~20 000 elements**. That is a known value this census must reproduce, and it
    /// discriminates the instrument from the subject: if the raw arm reads anything else,
    /// this census is broken and its conformed-arm numbers mean nothing — which is precisely
    /// the failure mode that a "measure only the suspect thing" census could not detect.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn conformed_disc_reference_corner_rest_census_fom() {
        let l4 = cf_fsu_geometry::load_from_env("CF_L4_STL").expect("load L4");
        let l5 = cf_fsu_geometry::load_from_env("CF_L5_STL").expect("load L5");
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc");
        let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
        let frame = cf_fsu_geometry::segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let params = DiscParams::default();
        let ep = EndplateConform {
            o4: &o4,
            o5: &o5,
            superior_axis: frame.superior_axis,
        };

        for (label, endplates) in [("raw", None), ("conformed", Some(ep))] {
            let p = prepare_disc(disc_mesh.clone(), &params, endplates)
                .unwrap_or_else(|e| panic!("prepare {label} disc: {e:?}"));
            // The SHIPPED mesh object, via the same helper `bond_prepared_tet10` calls.
            let tet10 = prepared_tet10_mesh(&p, &params, endplates, ConformFloors::SHIPPED);
            let c = census_rest_corners(&tet10);

            println!(
                "\n{label} Tet10 disc — {} tets\n  \
                 folded corners (det J_rest <= 0): {}\n  \
                 distinct folded elements:         {}\n  \
                 near-degenerate elements:         {}\n  \
                 worst normalised corner det:      {:.6}",
                c.n_tets,
                c.folded.len(),
                c.folded
                    .iter()
                    .map(|&(t, _, _)| t)
                    .collect::<std::collections::BTreeSet<_>>()
                    .len(),
                c.near_degenerate,
                c.worst,
            );
            let mut worst_first = c.folded.clone();
            worst_first.sort_by(|a, b| a.2.total_cmp(&b.2));
            for &(t, corner, ratio) in worst_first.iter().take(10) {
                println!("    tet {t:6} reference corner {corner} : {ratio:+.6}");
            }

            // ★ VALIDATE-AGAINST-A-KNOWN-VALUE: the raw arm is affine by construction, so
            // every corner of every element must read exactly 1.0. This is the instrument
            // check — it fails if the census is wrong, not if the mesh is bad.
            if label == "raw" {
                assert!(
                    (c.worst - 1.0).abs() < 1.0e-9,
                    "the raw disc is straight by construction, so every normalised corner \
                     determinant must be 1.0 — worst is {:.12}. This census is measuring \
                     something other than what it claims.",
                    c.worst
                );
                continue;
            }

            report_and_gate_folded(&tet10, &p, &c);
        }
    }

    /// **Rung 5.0 REPLICATION** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §3): the
    /// mesh-realization spread `σ(h)` at the two **refined** ladder levels.
    ///
    /// This is what §3 declared rung 5 blocked on. Every ladder-level quantity is `n = 1`, and
    /// the jitter at a *fixed* level is comparable to the difference claimed *between* levels.
    /// Step 1 caught this once at the level of the pair — a single perturbation reported ~9 %
    /// where a five-point sweep measured 21.13 % — and **the ladder has the identical defect one
    /// level up**. Re-running it post-α.1 without this would reproduce the same mistake with
    /// better-looking numbers.
    ///
    /// ⇒ **Replication per level, not more levels.** `σ` is known only at `cell = 0.003`
    /// (21.13 % p2p). There is no reason to assume it is level-independent — more nodes *should*
    /// mean less relative quantization, but that is a hypothesis, and this measures it.
    ///
    /// Each level sweeps the same **±3.3 %** relative window step 1 used at `0.003`, so the three
    /// levels' spreads are comparable by construction. Both arms at each cell come from **one**
    /// prepared mesh (`PreparedDisc::duplicate`), so the per-cell `k10/k4` ratio is attributable
    /// to element order — rung 1's attribution argument, made structural.
    ///
    /// ▶ **The decision rule this feeds is committed in §3, before any of it ran:** with
    /// `Δ = |k10(coarse)| − |k10(fine)|` and `σ(h)` the measured p2p spread, **`Δ > max(σ)` ⇒ the
    /// bracket is earned** and `Δ` is a committed lower bound on the error in the shipped
    /// `RUNG7_K_DISC`. **Otherwise the accuracy claim stays unearned, and that is a result, not a
    /// failure.** This test prints `σ`; it does not decide, and deliberately asserts no threshold
    /// on the spread — that would be reading the outcome back into the instrument.
    ///
    /// ⚠ **Runs `0.002` only.** The fine level `0.0015` does not converge under the single
    /// ±0.5° jump — see the note on `CENTRES` — so `σ` is measured at two levels (0.003 from
    /// step 1, 0.002 here), not three.
    ///
    /// ⚠ Cost: ~6 min. `rung5_step2` costs ONE solve per level; this does FOUR per realization
    /// (Tet4 flex/ext + Tet10 flex/ext), which is why the probe's figure understates it 2×.
    /// Peak RSS with both arms live reached 9.39 GB on the two-level run — above the 8 GB budget
    /// committed for the single-arm probe, which that budget never covered. Run under
    /// `/usr/bin/time -l`.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed); ~27 min"]
    fn rung5_replication_realization_spread_at_refined_levels_fom() {
        // The same ±3.3 % relative window step 1 swept at 0.003 (0.00290 … 0.00310), so the
        // levels' spreads are comparable rather than merely both being called "the spread".
        const OFFSETS: [f64; 5] = [-0.033_33, -0.016_67, 0.0, 0.016_67, 0.033_33];

        // ⚠ `0.0015` IS EXCLUDED, and that exclusion is a measurement, not a convenience.
        //
        // The first run of this sweep reached the fine level and died there: `free residual
        // norm NaN` at Newton iter 0, "Armijo line-search stalled" (`newton.rs:273`). Not OOM —
        // the documented OOM signature is `SymbolicLu::try_new`. The `faer` LU fallback fires
        // throughout every level and is separately documented as benign; a NaN residual is not.
        //
        // §5.5 step 4 asked precisely this — "does a refined arm survive the single ±0.5°
        // jump?" — and recorded that rung 2 had only ever answered it at `cell = 0.003`. At
        // `0.0015` the answer is NO for at least one realization, so `σ(0.0015)` cannot be
        // measured as specified: a spread needs every sample, and this level cannot supply them.
        //
        // §5.5 names the remedy (step the angle rather than jumping it; rung 2 records that
        // stepping does not move the measurement) but notes it turns every solve into 5–10.
        // That is a deliberate, separate decision — not something to slip into this sweep.
        const CENTRES: [f64; 1] = [0.002];

        use std::io::Write as _;

        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let base = DiscParams::default();
        let (flex, ext) = (0.5_f64.to_radians(), -0.5_f64.to_radians());

        for centre in CENTRES {
            let mut rows: Vec<Realization> = Vec::new();

            for off in OFFSETS {
                let cell = centre * (1.0 + off);
                let params = DiscParams { cell, ..base };
                let p = prepare_disc(disc_mesh.clone(), &params, None)
                    .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
                let corners = referenced_vertices(&p.tet).len();
                let p_bands = (p.inferior.len(), p.superior.len());

                // ONE mesh, both arms — attribution by construction, as in step 1.
                let mut tet4 = bond_prepared_tet4(p.duplicate().expect("duplicate"), &params);
                let mut tet10 = bond_prepared_tet10(p, &params, None, ConformFloors::SHIPPED);

                let lin_flex = tet4.flexion_moment(flex);
                let lin_ext = tet4.flexion_moment(ext);
                let quad_flex = tet10.flexion_moment(flex);
                let quad_ext = tet10.flexion_moment(ext);
                for (probe, name) in [
                    (lin_flex, "Tet4 flexion"),
                    (lin_ext, "Tet4 extension"),
                    (quad_flex, "Tet10 flexion"),
                    (quad_ext, "Tet10 extension"),
                ] {
                    assert!(
                        probe.1 < 1e-8,
                        "cell {cell}: {name} bond must conserve (‖ΣF‖+‖ΣM‖ = {:.2e})",
                        probe.1
                    );
                }

                let row = Realization {
                    cell,
                    corners,
                    bands: p_bands,
                    linear: (lin_flex.0 / flex, lin_ext.0 / ext),
                    quadratic: (quad_flex.0 / flex, quad_ext.0 / ext),
                };

                // ⚠ Print AND FLUSH per realization, not per level. The first run of this test
                // died on realization 2 of the second level after 20 minutes and lost every
                // completed row: `println!` to a redirected stdout is BLOCK-buffered, and the
                // panic discarded the buffer. A sweep this expensive must not be all-or-nothing.
                let (rf, re) = row.ratio();
                println!(
                    "cell {:.5} | corners {:5} | bands {:3}/{:3} | Tet4 {:.4}/{:.4} | \
                     Tet10 {:.4}/{:.4} | ratio {rf:.4}/{re:.4}",
                    row.cell,
                    row.corners,
                    row.bands.0,
                    row.bands.1,
                    row.linear.0,
                    row.linear.1,
                    row.quadratic.0,
                    row.quadratic.1,
                );
                std::io::stdout().flush().ok();

                rows.push(row);
            }

            // Without this the sweep measures nothing: if every cell produced the same mesh,
            // the "spread" is solver noise, not realization jitter.
            let distinct: std::collections::BTreeSet<usize> =
                rows.iter().map(|r| r.corners).collect();
            assert!(
                distinct.len() > 1,
                "centre {centre}: every cell produced the same referenced-corner count — the \
                 mesh was never re-realized, so this measures nothing"
            );

            println!("--- centre cell {centre:.4} ---");
            for (label, xs) in [
                (
                    "Tet10 flex",
                    rows.iter().map(|r| r.quadratic.0).collect::<Vec<_>>(),
                ),
                (
                    "Tet10 ext ",
                    rows.iter().map(|r| r.quadratic.1).collect::<Vec<_>>(),
                ),
                (
                    "Tet4  flex",
                    rows.iter().map(|r| r.linear.0).collect::<Vec<_>>(),
                ),
                (
                    "ratio flex",
                    rows.iter().map(|r| r.ratio().0).collect::<Vec<_>>(),
                ),
            ] {
                let (lo, hi, mean, pp) = spread(&xs);
                println!(
                    "  sigma {label}: min {lo:.4}  max {hi:.4}  mean {mean:.4}  p2p {pp:.2} %"
                );
            }
        }
    }

    /// **Rung 5.0 step 3** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §5.5): does the **Tet4**
    /// arm survive refinement? One build, one ±0.5° solve, at `cell = 0.002` and `0.0015`.
    ///
    /// §5.5 listed this step ("Tet4 at `cell = 0.0015`: one build, one solve … nearly free, and
    /// the first draft omitted it") and it was never run. The replication sweep then failed in a
    /// way that points straight at it, so it is run here as an isolation.
    ///
    /// ▶ **WHAT THIS DISCRIMINATES.** The sweep panicked with `free residual norm NaN` at Newton
    /// iter 0 on its third realization — whose *first* solve is the Tet4 arm at `cell = 0.002`.
    /// Two facts make that surprising:
    ///
    /// - `rung5_step2` solved **Tet10** at that same cell cleanly (residual 1.23e-11), and used
    ///   **zero** LU fallbacks — `Llt` succeeded on every iteration at both refined cells.
    /// - The sweep fires the fallback continuously, and those messages precede the first printed
    ///   row, i.e. they come from the Tet4 solves.
    ///
    /// So the non-SPD tangent belongs to the **Tet4** arm, and somewhere between `0.003` (where
    /// the fallback is documented benign) and `0.002` it stops being benign. That is backwards
    /// from the expectation that Tet4 is the cheap, robust arm — which is exactly why it needs
    /// measuring rather than assuming.
    ///
    /// Builds Tet4 **alone**, with no `duplicate()` and no Tet10 alongside, so a failure here is
    /// the arm itself rather than an interaction. If this passes at `0.002`, the sweep's failure
    /// is in the pairing and not in the element.
    ///
    /// Nearly free — Tet4 is ~1/18 of a Tet10 solve.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn rung5_step3_tet4_refined_arms_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let flex = 0.5_f64.to_radians();

        for cell in [0.002_f64, 0.001_5] {
            let params = DiscParams {
                cell,
                ..DiscParams::default()
            };
            let p = prepare_disc(disc_mesh.clone(), &params, None)
                .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
            let corners = referenced_vertices(&p.tet).len();

            // ⚠ BOTH construction paths. The sweep feeds Tet4 a `duplicate()` (so the original
            // can go to Tet10); this test originally fed it `p` directly and passed. That is the
            // one structural difference between the passing isolation and the failing sweep, so
            // it is the discriminator: if `duplicate` alone reproduces the NaN, the defect is in
            // `duplicate`; if it does not, the defect is in the two arms' co-residency.
            let dup = p.duplicate().expect("duplicate");

            // A closure rather than an array of `(label, PreparedDisc)`: `PreparedDisc` is large
            // enough that `large_stack_arrays` objects to the array and `useless_vec` objects to
            // the `vec!` fix, so the two lints have no shared solution. Passing each one in
            // avoids the container entirely.
            let solve_one = |label: &str, prepared: PreparedDisc| {
                let t = std::time::Instant::now();
                let mut tet4 = bond_prepared_tet4(prepared, &params);
                let (moment, residual) = tet4.flexion_moment(flex);
                let wall = t.elapsed();

                println!(
                    "rung5 step3 | cell {cell:.4} | corners {corners:5} | Tet4 {label} | \
                     {:5.1} s | moment {moment:.6} | resid {residual:.2e}",
                    wall.as_secs_f64(),
                );
                std::io::Write::flush(&mut std::io::stdout()).ok();

                assert!(
                    residual < 1e-8,
                    "cell {cell} ({label}): Tet4 arm must conserve (‖ΣF‖+‖ΣM‖ = {residual:.2e})"
                );
            };

            solve_one("direct   ", p);
            solve_one("duplicate", dup);
        }
    }

    /// **Rung 5.0 — minimal co-residency reproducer** for the refined-level `NaN`.
    ///
    /// Isolation has cleared everything else: Tet4 alone converges at `0.002` and `0.0015` with
    /// zero LU fallbacks, Tet10 alone likewise, and `duplicate()` is bit-identical to direct
    /// construction. What remains is the two arms being live at once — which step 1 does at
    /// `cell = 0.003` in a shipped, passing test, so the interaction is refinement-dependent.
    ///
    /// This is ONE realization at `cell = 0.002`, mirroring the sweep's structure exactly: both
    /// arms built **before** either is solved (the Tet10 build lands between the Tet4 build and
    /// the Tet4 solve), then the same four solves in the same order.
    ///
    /// ▶ **WHAT EACH OUTCOME MEANS — stated before the run, so neither can be talked into being
    /// the interesting one:**
    ///
    /// - **Panics with `r_norm NaN`** ⇒ minimal reproducer. The defect needs nothing but two
    ///   refined arms co-resident, and it is a `sim-soft` bug rather than a rung-5 artefact.
    /// - **Passes** ⇒ co-residency alone is NOT sufficient, and the sweep's failure required
    ///   accumulation across realizations (two meshes at `0.00193` and `0.00197` were built and
    ///   solved, and dropped, before the failing one). That points at state surviving a
    ///   `PreparedDisc`'s lifetime, and the next probe is two realizations rather than one.
    ///
    /// Asserts conservation only. Whether it panics IS the measurement.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn rung5_coresidency_minimal_reproducer_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let params = DiscParams {
            cell: 0.002,
            ..DiscParams::default()
        };
        let (flex, ext) = (0.5_f64.to_radians(), -0.5_f64.to_radians());

        let p = prepare_disc(disc_mesh, &params, None).expect("prepare raw disc at cell 0.002");
        let corners = referenced_vertices(&p.tet).len();

        // Same order as the sweep: BOTH built before EITHER is solved.
        let mut tet4 = bond_prepared_tet4(p.duplicate().expect("duplicate"), &params);
        let mut tet10 = bond_prepared_tet10(p, &params, None, ConformFloors::SHIPPED);

        let t = std::time::Instant::now();
        let lin_flex = tet4.flexion_moment(flex);
        let lin_ext = tet4.flexion_moment(ext);
        let quad_flex = tet10.flexion_moment(flex);
        let quad_ext = tet10.flexion_moment(ext);
        let wall = t.elapsed();

        println!(
            "rung5 coresidency | cell 0.002 | corners {corners} | {:.1} s | \
             Tet4 {:.4}/{:.4} | Tet10 {:.4}/{:.4}",
            wall.as_secs_f64(),
            lin_flex.0 / flex,
            lin_ext.0 / ext,
            quad_flex.0 / flex,
            quad_ext.0 / ext,
        );
        std::io::Write::flush(&mut std::io::stdout()).ok();

        for (probe, name) in [
            (lin_flex, "Tet4 flexion"),
            (lin_ext, "Tet4 extension"),
            (quad_flex, "Tet10 flexion"),
            (quad_ext, "Tet10 extension"),
        ] {
            assert!(
                probe.1 < 1e-8,
                "{name} must conserve (‖ΣF‖+‖ΣM‖ = {:.2e})",
                probe.1
            );
        }
    }

    /// **Rung 5.0 — does DROPPING the Tet10 arm save the Tet4 solve?**
    ///
    /// `rung5_coresidency_minimal_reproducer_fom` is byte-for-byte this test plus one thing: it
    /// keeps the Tet10 arm alive across the Tet4 solve. Here it is dropped first. That single
    /// difference is the whole experiment.
    ///
    /// ▶ **Committed before the run:**
    ///
    /// - **Still `NaN`** ⇒ the damage is done at CONSTRUCTION and outlives the object. That
    ///   points at process- or thread-global state — a shared workspace, a pool, a cached
    ///   symbolic pattern — not at the two arms competing for anything.
    /// - **Converges** ⇒ construction is harmless and CO-EXISTENCE during the solve is what
    ///   matters. That points instead at aliasing between two live instances.
    ///
    /// Either way the bug report gains the one fact that decides where to look. ~2 s.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn rung5_coresidency_discriminator_dropped_tet10_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let params = DiscParams {
            cell: 0.002,
            ..DiscParams::default()
        };
        let flex = 0.5_f64.to_radians();

        let p = prepare_disc(disc_mesh, &params, None).expect("prepare raw disc at cell 0.002");
        let corners = referenced_vertices(&p.tet).len();

        let mut tet4 = bond_prepared_tet4(p.duplicate().expect("duplicate"), &params);

        // Built exactly as the reproducer builds it — then dropped, which is the one difference.
        let tet10 = bond_prepared_tet10(p, &params, None, ConformFloors::SHIPPED);
        drop(tet10);

        let t = std::time::Instant::now();
        let (moment, residual) = tet4.flexion_moment(flex);
        let wall = t.elapsed();

        println!(
            "rung5 discriminator | cell 0.002 | corners {corners} | Tet10 built then DROPPED | \
             {:.1} s | moment {moment:.6} | resid {residual:.2e}",
            wall.as_secs_f64(),
        );
        std::io::Write::flush(&mut std::io::stdout()).ok();

        assert!(
            residual < 1e-8,
            "Tet4 must conserve after a dropped Tet10 arm (‖ΣF‖+‖ΣM‖ = {residual:.2e})"
        );
    }

    /// **Rung 5.0 step 2** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §5.5): do the refined
    /// Tet10 arms fit? One build, one ±0.5° solve, at `cell = 0.002` **and `0.0015`**.
    ///
    /// §5.5 recorded "Tet10 at `cell = 0.002` exceeds the RSS budget" as a rollback constraint,
    /// on a pre-α.1 estimate of ~140k DOF and 3–8 GB (the solver holds **two** symbolic
    /// factorizations for its lifetime — a Cholesky factor on the lower triangle and a
    /// `SymbolicLu` on the full reflected pattern — and rebuilds the numeric LU every
    /// iteration). α.1 then shrank
    /// the ladder ~45 % (fine level 18 485 → 10 048 referenced corners), so §3 flags that
    /// constraint as needing re-measurement **before it is planned around — it may simply be
    /// gone**. This is that measurement.
    ///
    /// ▶ **BUDGET, pre-registered here before the first run** (§5.5 asks for a budget *with a
    /// threshold action*, so the outcome cannot be re-read afterwards):
    ///
    /// > **8 GB peak RSS.** If the single-arm probe exceeds it, the Tet10 fine arm drops to
    /// > `cell = 0.0025` rather than reshaping mid-FOM.
    ///
    /// The 8 GB is not arbitrary: this arm is the *cheap* case. Step 1's replication sweep holds
    /// a Tet4 arm live beside the Tet10 one at each of five realizations per level, and §5.5
    /// warns that unless each level's arms are dropped before the next is built, the ladder's
    /// peak is the **sum** rather than the max. On a 24 GB machine a single arm claiming more
    /// than a third of physical memory makes that sweep unsafe.
    ///
    /// ⚠ **This test does not assert the budget, and deliberately so.** Peak RSS is not
    /// observable from inside the process without `unsafe` libc (`getrusage`), which would spend
    /// the crate's Safety criterion on a diagnostic. Run it under an external sampler and read
    /// the number off that:
    ///
    /// ```sh
    /// /usr/bin/time -l cargo test -p cf-fsu-model --release \
    ///   rung5_step2_tet10_fine_cost_ceiling_fom -- --ignored --nocapture
    /// ```
    ///
    /// ⚠ **OOM here is not graceful.** `SymbolicLu::try_new(..).expect("symbolic LU
    /// factorization of free-block pattern failed")` (`construct.rs:188-189`) panics with a
    /// message that reads as a *pattern* bug. If this test dies with that string, read it as
    /// "the budget was exceeded", not as a solver defect.
    ///
    /// What it prints — referenced corners, meshing wall, solve wall, the moment and its
    /// conservation residual — is the cost model §5.5 asks for. It asserts only conservation,
    /// which must hold for the solve to mean anything at all.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn rung5_step2_tet10_fine_cost_ceiling_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");

        // ⚠ BOTH refined levels, not just §5.5's `0.002`. Step 0 puts the ladder at
        // 1 580 / 4 632 / 10 048 referenced corners for `cell` = 0.003 / 0.002 / 0.0015, so
        // `0.002` is the MIDDLE rung and the fine arm is `0.0015`. §4.8's bracket
        // `|k*| ≤ |k10(fine)| ≤ |k10(coarse)| ≤ |k4|` needs `k10` at the fine level, but §5.5's
        // cost spike costs Tet10 at `0.002` (step 2) and *Tet4* at `0.0015` (step 3) — the
        // Tet10 fine arm the bracket depends on was never costed. Both are measured here.
        //
        // ⚠ Each level's arm is dropped before the next is built (§5.5: otherwise the ladder's
        // peak RSS is the SUM rather than the max) — hence the inner scope.
        for cell in [0.002_f64, 0.001_5] {
            let params = DiscParams {
                cell,
                ..DiscParams::default()
            };

            let t_mesh = std::time::Instant::now();
            let p = prepare_disc(disc_mesh.clone(), &params, None)
                .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
            let corners = referenced_vertices(&p.tet).len();
            let bands = (p.inferior.len(), p.superior.len());
            let mesh_wall = t_mesh.elapsed();

            let t_bond = std::time::Instant::now();
            let mut tet10 = bond_prepared_tet10(p, &params, None, ConformFloors::SHIPPED);
            let bond_wall = t_bond.elapsed();

            let t_solve = std::time::Instant::now();
            let (moment, residual) = tet10.flexion_moment(0.5_f64.to_radians());
            let solve_wall = t_solve.elapsed();

            println!(
                "rung5 step2 | cell {cell:.4} | corners {corners:5} | bands {:3}/{:3} | \
                 mesh {:5.1} s | bond {:5.1} s | solve {:6.1} s | moment {moment:.6} | \
                 resid {residual:.2e}",
                bands.0,
                bands.1,
                mesh_wall.as_secs_f64(),
                bond_wall.as_secs_f64(),
                solve_wall.as_secs_f64(),
            );

            assert!(
                residual < 1e-8,
                "cell {cell}: Tet10 arm must conserve (‖ΣF‖+‖ΣM‖ = {residual:.2e}) — a \
                 non-conserving solve makes the cost measurement meaningless"
            );
        }
    }

    /// **Rung 5.0 step 0** (`docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §3): the disc's SI
    /// extent and the **realized** bonded band, at every candidate `cell` of the rung-5 ladder.
    ///
    /// **This gates the ladder itself, and it is the cheapest measurement in the rung** — one
    /// mesh per level, no solve. The rung-5 spec's clamp-quantization confound says the pinned
    /// Dirichlet band is a fixed *physical* slab (`band_frac · SI extent`) while the BCC lattice
    /// is anchored to the **world origin** (`sdf_bridge/lattice.rs`), so the *realized* clamp
    /// planes are quantized in steps of `cell/2` and their depth need not vary monotonically —
    /// or even vary at all — with `cell`. A ladder whose levels alias onto the same layer count
    /// measures nothing; a ladder whose free height swings is measuring the boundary condition
    /// rather than the element.
    ///
    /// ⚠ **The spec's worked figures (a ~1.8 mm band, 1.5/1.0/1.5 mm clamp depths, a ±13 %
    /// swing) are an ILLUSTRATION at an assumed ~10 mm SI extent — the disc's real extent is
    /// committed nowhere, because `band_frac` is a fraction precisely so no absolute ever had
    /// to be.** This test is the producer. Do not quote the illustration.
    ///
    /// Prints the table and asserts what must hold for the ladder to be *measurable at all*:
    /// non-empty bands and a positive free height at **every** cell, plus strictly-increasing
    /// refinement across **the ladder proper only** — the probe cells are exempt, because whether
    /// retention is monotone *there* is precisely the open question and asserting it would beg it.
    ///
    /// ★ **Plus assert 2b (added 2026-08-07): the pinned SHARE of the domain must not GROW**
    /// across the ladder. That is the boundary condition converging or not, which is a
    /// precondition for reading anything as element convergence — see the block at the assert.
    /// It is directional and carries no tuned constant.
    ///
    /// Everything beyond those is a head-engineer call on the printed numbers, not an assert
    /// here: this test does not decide whether the ladder is worth running.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    // Band and corner counts are in the hundreds-to-thousands — the usize→f64 casts for the
    // pinned share are exact.
    #[allow(clippy::cast_precision_loss)]
    fn rung5_step0_realized_band_across_the_ladder_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let base = DiscParams::default();

        // The SI extent of the *scaled* surface AABB, derived from the native mesh rather than
        // by re-running production's recentre: recentring cannot change an extent and the scale
        // is uniform, so `extent_scaled == extent_native · scale` exactly. That keeps this
        // measurement off a reimplementation of `prepare_disc_at`'s framing.
        let (nz_lo, nz_hi) = disc_mesh
            .vertices
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v.z), hi.max(v.z))
            });
        let si_extent = (nz_hi - nz_lo) * base.scale;
        let band = base.band_frac * si_extent;
        println!(
            "disc SI extent: {:.4} mm native -> {si_extent:.6} m scaled; \
             band_frac {:.2} => band {:.6} m ({:.4} mm)",
            nz_hi - nz_lo,
            base.band_frac,
            band,
            band * 1e3,
        );

        // Two groups. The LADDER proper is {0.003, 0.002, 0.0015} and must strictly refine. The
        // PROBE cells are step 1's re-realization window: they are deliberately NOT asserted
        // monotone, because whether retention is monotone there is exactly what is in question.
        let ladder = [0.003, 0.002, 0.0015];
        let mut prev_corners = 0usize;
        // The COARSEST ladder level's `(cell, pinned share)` — the baseline every finer level
        // is held against below. Comparing against the coarsest rather than step-wise costs
        // nothing and is the more jitter-tolerant of the two.
        let mut coarsest_share: Option<(f64, f64)> = None;
        for cell in [
            0.002_90, 0.002_95, 0.003_05, 0.003_10, // step-1 probe window
            0.003, 0.002, 0.0015, // the ladder
        ] {
            let params = DiscParams { cell, ..base };
            let p = prepare_disc(disc_mesh.clone(), &params, None)
                .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
            let pos = p.tet.positions();
            let z = |v: &VertexId| pos[*v as usize].z;

            // The REALIZED clamp planes: the deepest pinned layer on each face. These, not the
            // nominal `band`, are the boundary condition the solve actually sees.
            let inf_max = p.inferior.iter().map(z).fold(f64::NEG_INFINITY, f64::max);
            let sup_min = p.superior.iter().map(z).fold(f64::INFINITY, f64::min);
            let free = sup_min - inf_max;
            let referenced = referenced_vertices(&p.tet).len();
            let tets = p.tet.n_tets();

            assert!(
                !p.inferior.is_empty() && !p.superior.is_empty(),
                "cell {cell}: an endplate band captured no vertices"
            );
            assert!(
                free > 0.0,
                "cell {cell}: bands meet or cross (free height {free:.6} m)"
            );
            if ladder.contains(&cell) {
                assert!(
                    referenced > prev_corners,
                    "cell {cell}: refinement must strictly increase referenced corners \
                     ({referenced} vs {prev_corners}) — otherwise the ladder has an inert level"
                );
                prev_corners = referenced;

                // ── The PINNED SHARE of the domain must not GROW under refinement ──
                //
                // Step 1's mechanism was never the clamp plane, it was the Dirichlet
                // population: a lattice-phase shift swept a whole layer of nodes out of the
                // superior band and halved the constraints on the top face, while the free
                // height moved 0.065 %. The step-0 gate as first written could not see that —
                // it asserted the clamp *plane* — and the plan's own conclusion was "gate the
                // POPULATION, not just the plane, and NORMALISE it, since the raw count must
                // grow under refinement". This is that gate; the normalisation is the
                // load-bearing half.
                //
                // ★ Why the SHARE, and not a growth rate against an ideal `r³`. The band is a
                // fixed physical slab (`band_frac · SI extent`), so it is a sub-VOLUME of a
                // fixed domain: it and the whole mesh fill at the same realized node density,
                // making their ratio an h-independent geometric property. A raw growth rate
                // would instead measure the MESHER — the domain's own corner count grows
                // 2.93× across the first ladder step against an ideal 3.375×, so a band
                // tracking its domain perfectly would still read "slow". Dividing by the
                // domain removes that confound completely.
                //
                // ★★ No tuned constant appears here. The assertion is DIRECTIONAL: a boundary
                // condition claiming an ever-larger share of the domain as you refine is by
                // definition not converging, and no h-convergence reading taken across those
                // levels can be separated from it. That is a property of the sequence, not a
                // threshold someone had to choose.
                //
                // ★ It discriminates on the data that exists — pinned share across the ladder:
                //   pre-α.1   0.2636 → 0.3084 → 0.3270   GROWS +24 %  ← the pathology
                //   post-α.1  0.1196 → 0.1038 → 0.0984   falls −18 %
                //
                // ⚠ Margin, stated: the share's own jitter over a ±3.3 % `cell` window is
                // ±12 % (0.1138…0.1457 at cell ≈ 0.003) against the 18 % margin here — thin,
                // even though normalising already damps the raw superior band's ±36 % by ~3×.
                // A re-mesh CAN false-fire this; it is gated anyway because the meshes are
                // SHA-pinned and these cells are fixed constants. ⚠⚠ Read a failure as "the
                // Dirichlet set is not converging", NEVER as "the elements converged
                // differently". Note the asymmetry in what the data supports: the pre-α.1
                // violation is trustworthy (+24 % against ±12 %), the post-α.1 conformance is
                // suggestive only — which is precisely why rung 5 still needs replication per
                // level before any bracket claim.
                let pinned_share = (p.inferior.len() + p.superior.len()) as f64 / referenced as f64;
                match coarsest_share {
                    None => coarsest_share = Some((cell, pinned_share)),
                    Some((coarse_cell, coarse_share)) => assert!(
                        pinned_share <= coarse_share,
                        "cell {coarse_cell} -> {cell}: the pinned share of the domain GREW, \
                         {coarse_share:.4} -> {pinned_share:.4} ({} of {referenced} corners \
                         pinned). A Dirichlet set that claims an increasing fraction of the \
                         mesh as it refines is not converging, so no element-convergence \
                         reading across these levels is separable from it.",
                        p.inferior.len() + p.superior.len()
                    ),
                }
            }

            println!(
                "cell {cell:.5} m | corners {referenced:5} tets {tets:6} (verts {:5}) | \
                 band {:.3} cell/2 units | inf {:4} nodes, clamp z {inf_max:+.6} | \
                 sup {:4} nodes, clamp z {sup_min:+.6} | free {:.6} m = {:.3} cell/2 = {:.4} mm",
                p.tet.n_vertices(),
                band / (cell / 2.0),
                p.inferior.len(),
                p.superior.len(),
                free,
                free / (cell / 2.0),
                free * 1e3,
            );
        }
    }

    /// Face-connected components of a raw tet mesh, largest first.
    ///
    /// A **reimplementation** of the union-find inside
    /// [`SdfMeshedTetMesh::largest_component`], and deliberately so: that method returns
    /// only the survivors, so the discarded set is unobservable through it. Production
    /// answers "which component is biggest"; the census needs "what are all the components",
    /// which is strictly more information than the production API exposes.
    ///
    /// Being a reimplementation, it is **validated against production** at every cell of the
    /// census — the largest component here must equal, tet for tet, the mesh
    /// `prepare_disc` actually keeps. Without that check this function's output would
    /// describe some other mesh's connectivity.
    fn face_components(mesh: &SdfMeshedTetMesh) -> Vec<Vec<TetId>> {
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]]; // path halving
                x = parent[x];
            }
            x
        }

        let n = mesh.n_tets();
        let mut parent: Vec<usize> = (0..n).collect();
        // Two tets are connected when they share a triangular face — production's rule.
        let mut owner: std::collections::HashMap<[VertexId; 3], usize> =
            std::collections::HashMap::new();
        for ti in 0..n {
            let t = mesh.tet_vertices(u32::try_from(ti).expect("tet id fits u32"));
            for mut f in [
                [t[1], t[2], t[3]],
                [t[0], t[2], t[3]],
                [t[0], t[1], t[3]],
                [t[0], t[1], t[2]],
            ] {
                f.sort_unstable();
                if let Some(&other) = owner.get(&f) {
                    let (ra, rb) = (find(&mut parent, ti), find(&mut parent, other));
                    parent[rb] = ra;
                } else {
                    owner.insert(f, ti);
                }
            }
        }

        let mut by_root: std::collections::BTreeMap<usize, Vec<TetId>> =
            std::collections::BTreeMap::new();
        for t in 0..n {
            let r = find(&mut parent, t);
            by_root
                .entry(r)
                .or_default()
                .push(u32::try_from(t).expect("tet id fits u32"));
        }
        let mut comps: Vec<Vec<TetId>> = by_root.into_values().collect();
        // Size-descending, then by first member, so the order is total and reproducible
        // (equal-sized islands must not be ordered by hash iteration).
        comps.sort_by(|a, b| b.len().cmp(&a.len()).then_with(|| a[0].cmp(&b[0])));
        comps
    }

    /// Enclosed volume of a closed triangle surface, by the divergence theorem.
    ///
    /// Computed here rather than read from any pipeline stage: it is the census's only
    /// **independent** yardstick, the one number that does not come from the mesher whose
    /// output is under examination.
    fn surface_volume(mesh: &IndexedMesh) -> f64 {
        mesh.faces
            .iter()
            .map(|f| {
                let a = mesh.vertices[f[0] as usize].coords;
                let b = mesh.vertices[f[1] as usize].coords;
                let c = mesh.vertices[f[2] as usize].coords;
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum::<f64>()
            .abs()
    }

    /// Volume-weighted summary of one set of tets: where it sits and how thin it is.
    ///
    /// ⚠ Every field but `volume` was designed to test the tapering-rim hypothesis, which
    /// the branch went on to **refute** — and the two oracle-derived ones are measured
    /// through the very oracle whose sign is wrong in this frame. They are kept as
    /// diagnostics for the remedy to move, **not** as facts about the disc. Do not quote
    /// them.
    struct Region {
        volume: f64,
        /// Volume-weighted mean radius from the SI axis, as a fraction of the surface's own
        /// maximum radius. Values **above 1** mean material outside the disc's own radial
        /// extent — which is the phantom material, not a rim.
        mean_r_norm: f64,
        max_r_norm: f64,
        /// Volume-weighted `-φ(centroid)` in mm. ⚠ **NOT a depth.** It is signed by the
        /// broken oracle, so exterior phantom material reports as deeply *interior* — the
        /// reading that first exposed the bug (15.13 mm "inside" a disc whose deepest point
        /// is 5.74 mm down).
        mean_depth_mm: f64,
        max_depth_mm: f64,
        /// Fraction of this region's volume inside the physical endplate band slab.
        band_frac_of_volume: f64,
    }

    /// Summarise `tets` of `mesh` in the scaled frame (origin = the disc AABB centre).
    #[allow(clippy::cast_precision_loss)]
    fn summarise(
        mesh: &SdfMeshedTetMesh,
        tets: &[TetId],
        sdf: &MeshOracle,
        r_surface_max: f64,
        band_z: (f64, f64),
    ) -> Region {
        let pos = mesh.positions();
        let vols = &mesh.quality().signed_volume;
        let (mut volume, mut wr, mut wd, mut band) = (0.0, 0.0, 0.0, 0.0);
        let (mut max_r, mut max_d) = (0.0_f64, f64::NEG_INFINITY);
        for &t in tets {
            let v = vols[t as usize].abs();
            let c = mesh.tet_vertices(t);
            let centroid: Vec3 = c.iter().map(|&i| pos[i as usize]).sum::<Vec3>() / 4.0;
            let r = centroid.x.hypot(centroid.y) / r_surface_max;
            let depth_mm = -sdf.eval(Point3::from(centroid)) * 1e3;
            volume += v;
            wr += v * r;
            wd += v * depth_mm;
            if centroid.z < band_z.0 || centroid.z > band_z.1 {
                band += v;
            }
            max_r = max_r.max(r);
            max_d = max_d.max(depth_mm);
        }
        Region {
            volume,
            mean_r_norm: if volume > 0.0 { wr / volume } else { 0.0 },
            max_r_norm: max_r,
            mean_depth_mm: if volume > 0.0 { wd / volume } else { 0.0 },
            max_depth_mm: if max_d.is_finite() { max_d } else { 0.0 },
            band_frac_of_volume: if volume > 0.0 { band / volume } else { 0.0 },
        }
    }

    /// **Meshing-stability rung, step 0 — RECON: a census of what `largest_component`
    /// throws away.**
    ///
    /// Rung 5 established that `k_disc` is not a stable function of `cell`: over a ±3.4 %
    /// window the absolutes move **21.13 %** peak-to-peak post-α.1 (~100 % pre-α.1, when phantom
    /// material amplified the swing), and the mover is the *retained
    /// domain* — the tet count swings ~49.5 % non-monotonically while the clamp depth holds
    /// to 0.20 %, so it is not the boundary condition. That makes every absolute `k_disc`
    /// the arc has published, `RUNG7_K_DISC` included, one draw from a wide distribution.
    ///
    /// This test does **not** fix that. It measures the discard, because the fix that is
    /// correct depends on a fact nobody has measured yet. Two places in the tree —
    /// `SdfMeshedTetMesh::largest_component`'s doc comment and `prepare_disc_at`'s inline
    /// comment — already assert *as fact* that the stuffer "fragments the disc's sub-cell-thin
    /// tapering rim into disconnected islands". That claim has never been run. It is the
    /// hinge of the whole remedy choice, so the census either promotes it to a measurement
    /// or kills it.
    ///
    /// **★ IT KILLED IT.** The discarded material is not anatomy: it is **phantom** material
    /// the mesher emitted outside the disc, because `prepare_disc_at` builds its oracle on a
    /// mesh already rescaled to SI metres, where 30 % of the disc's triangles fall under the
    /// area floor parry silently skips (`mesh_stability_instrument_check_fom` has the chain,
    /// and `mesh-sdf`'s `pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use`
    /// pins the regime). So meshing stability and exact geometry are **NOT one fix**, and the
    /// remedy is none of the three this census set out to discriminate — not **policy**, not
    /// **mesher**, not **input conditioning**, but the frame the oracle is built in.
    ///
    /// The census stays because it is the producer for the island and volume numbers, and
    /// because it is the measurement the remedy has to move: when the oracle is built in a
    /// frame it works in, the discarded fraction here should collapse.
    ///
    /// ⚠ **Read the volume and component columns; distrust the `depth` and `r/r_max` ones.**
    /// Volumes come from the mesh's own quality cache scored against an independent
    /// divergence-theorem yardstick, and the components are pure topology — neither touches
    /// the oracle. The depth and radius columns are computed **through the very oracle whose
    /// sign is wrong here**, which is why they read as "discarded material is deeper than
    /// kept material" — an artefact, not a fact about the disc.
    ///
    /// ⚠ **Asserts nothing about stability, on purpose.** The only assertions here are
    /// harness-validity ones: that the components partition the mesh, and that this file's
    /// reimplemented connectivity reproduces production's kept mesh *tet for tet*. A
    /// threshold asserted before the mechanism is known would just freeze today's draw.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    #[allow(clippy::cast_precision_loss)]
    fn mesh_stability_step0_discarded_component_census_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let base = DiscParams::default();

        // The surface volume in the SOLVER frame. Recentring cannot change a volume and the
        // scale is uniform, so `V_scaled == V_native · scale³` exactly — no need to re-run
        // production's recentre to get it.
        let v_surface = surface_volume(&disc_mesh) * base.scale.powi(3);
        println!(
            "disc surface: {} faces, enclosed volume {:.4} mm³ native -> {v_surface:.6e} m³ scaled",
            disc_mesh.faces.len(),
            surface_volume(&disc_mesh),
        );

        for cell in [
            0.002_90, 0.002_95, 0.003_00, 0.003_05, 0.003_10, // rung 5's probe window
            0.002, 0.001_5, // the refinement ladder
        ] {
            census_one_cell(&disc_mesh, &DiscParams { cell, ..base }, v_surface);
        }
    }

    // The oracle's scale regime is now pinned where it belongs — `mesh-sdf`'s
    // `pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use`, which gates the
    // primitive for every consumer instead of only this one. What stays here is the
    // disc-specific question that gate cannot answer: where THIS anatomy's triangles sit
    // relative to parry's area floor, in each of the two frames (`report_area_floor_margin`).

    /// The remedy, expressed as an **experiment**: a scaled-frame view of an oracle that was
    /// BUILT in native millimetres.
    ///
    /// `prepare_disc_at` rescales the disc and then builds its oracle on the rescaled mesh,
    /// which puts 30 % of this disc's triangles under the area floor parry silently skips.
    /// This adapter changes exactly one thing — the frame the oracle is *built* in — while
    /// keeping the frame it is *queried* in identical, so the resulting tet mesh is directly
    /// comparable to production's, tet count for tet count.
    ///
    /// Under a uniform scale `s` about `center_native`, `φ_scaled(p) = s · φ_native(p/s + c)`,
    /// so the gradient passes through **unchanged**: `∇φ_scaled = s · ∇φ_native · (1/s)`.
    struct NativeFrameOracle<'a> {
        inner: &'a MeshOracle,
        center_native: Point3<f64>,
        scale: f64,
    }

    impl NativeFrameOracle<'_> {
        fn to_native(&self, p: Point3<f64>) -> Point3<f64> {
            Point3::from(p.coords / self.scale + self.center_native.coords)
        }
    }

    impl Sdf for NativeFrameOracle<'_> {
        fn eval(&self, p: Point3<f64>) -> f64 {
            self.inner.eval(self.to_native(p)) * self.scale
        }
        fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
            self.inner.grad(self.to_native(p))
        }
    }

    /// **Instrument check for the census** — run this before believing any of its numbers.
    ///
    /// The census reports quantities from three sources that could each be wrong
    /// independently: an enclosed volume from the surface, an interior depth from the SDF
    /// oracle, and a mesh volume from the stuffer's quality cache. Each is checked here
    /// against something that does not come from the same place.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    #[allow(clippy::cast_precision_loss)]
    fn mesh_stability_instrument_check_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let native_sdf = report_surface_health(&disc_mesh);

        // Now the mesher, in production's own frame, at the shipped cell.
        let params = DiscParams::default();
        let meshed = mesh_disc_raw(disc_mesh.clone(), &params).expect("mesh raw disc");
        report_scaled_frame_disagreement(&meshed, &native_sdf, &params);

        // ★ THE SDF-FREE CHECK. Every φ above depends on the oracle's sign, and the depth
        // reading already contradicts `body_center`, so the oracle cannot be the witness to
        // its own reliability. A point outside the surface's AABB is outside the solid — no
        // oracle, no winding number, no argument. This is a hard LOWER bound on how much of
        // the mesh sits off the disc.
        let v_surface = surface_volume(&disc_mesh) * params.scale.powi(3);
        let kept = meshed.raw.largest_component();
        report_phantom_material("raw ", &meshed.raw, &meshed.bbox, v_surface);
        report_phantom_material("kept", &kept, &meshed.bbox, v_surface);

        // ★ THE CAUSAL TEST for the open boundary the health report found. Consistency is not
        // causation: close the hole and re-run the identical measurement. If the leak was the
        // cause the phantom material goes away; if it was not, the numbers do not move.
        // Nothing here is a production change — it is the experiment that tells the next rung
        // which remedy to build.
        println!("\n--- same measurement, with the boundary closed ---");
        let mut filled = disc_mesh.clone();
        let n_filled = mesh_repair::fill_holes(&mut filled, 64).expect("fill holes");
        let after = mesh_repair::validate_mesh(&filled);
        println!(
            "filled {n_filled} hole(s) -> watertight {} manifold {} | boundary edges {} | \
             enclosed volume {:.2} mm³ (was {:.2})",
            after.is_watertight,
            after.is_manifold,
            after.boundary_edge_count,
            surface_volume(&filled),
            surface_volume(&disc_mesh),
        );
        let v_filled = surface_volume(&filled) * params.scale.powi(3);
        let refilled = mesh_disc_raw(filled, &params).expect("mesh repaired disc");
        let kept_f = refilled.raw.largest_component();
        report_phantom_material("raw ", &refilled.raw, &refilled.bbox, v_filled);
        report_phantom_material("kept", &kept_f, &refilled.bbox, v_filled);
        println!(
            "components: {} (was {}) | tets raw {} kept {} (was {} / {})",
            face_components(&refilled.raw).len(),
            face_components(&meshed.raw).len(),
            refilled.raw.n_tets(),
            kept_f.n_tets(),
            meshed.raw.n_tets(),
            kept.n_tets(),
        );

        // ★★ THE CAUSAL TEST FOR THE FRAME, and the one this whole diagnosis rests on.
        //
        // Everything above is *consistent* with "the metre-frame oracle's wrong sign is what
        // fills space with phantom tets", and consistency is exactly what the STL hole also
        // had before filling it turned out to change nothing. So run the same experiment on
        // the frame: build the oracle in native mm, query it in the scaled frame, change
        // NOTHING else, and re-measure. If the frame is the cause the phantom material
        // collapses; if it is not, these numbers barely move and the diagnosis is incomplete.
        //
        // This is a measurement, not the remedy — production still builds its own oracle.
        println!("\n--- same measurement, oracle BUILT in native mm (the causal test) ---");
        let adapter = NativeFrameOracle {
            inner: &native_sdf,
            center_native: meshed.center_native,
            scale: params.scale,
        };
        let padded = meshed.bbox.expanded(params.pad);
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(padded.min.x, padded.min.y, padded.min.z),
                Vec3::new(padded.max.x, padded.max.y, padded.max.z),
            ),
            cell_size: params.cell,
            material_field: Some(MaterialField::uniform(params.mu, 4.0 * params.mu)),
        };
        let fixed = SdfMeshedTetMesh::from_sdf(&adapter, &hints).expect("mesh with native oracle");
        let fixed_kept = fixed.largest_component();
        report_phantom_material("raw ", &fixed, &meshed.bbox, v_surface);
        report_phantom_material("kept", &fixed_kept, &meshed.bbox, v_surface);
        println!(
            "components: {} (was {}) | tets raw {} kept {} (was {} / {}) | retained {:.2} % \
             (was {:.2} %)",
            face_components(&fixed).len(),
            face_components(&meshed.raw).len(),
            fixed.n_tets(),
            fixed_kept.n_tets(),
            meshed.raw.n_tets(),
            kept.n_tets(),
            100.0 * fixed_kept.n_tets() as f64 / fixed.n_tets() as f64,
            100.0 * kept.n_tets() as f64 / meshed.raw.n_tets() as f64,
        );
    }

    /// **Step 0 of the frame fix: does the LOFTED disc suffer the frame bug at all?**
    ///
    /// The recon measured the *scanned* disc (`FMA16036`) and found 30 % of its triangles under
    /// parry's area floor once rescaled to metres. It says nothing about the **lofted** disc —
    /// the surface [`lofted_disc`] assembles from L4/L5 endplate patches, which is a completely
    /// different construction path and is what `cf-spine-studio` actually builds.
    ///
    /// **That distinction decides real scope**, which is why it is measured before anything is
    /// re-anchored: every drivability cell in [`DISC_CONFORM_QUALITY_FLOOR`]'s and
    /// [`DISC_MIDSIDE_CONFORM_QUALITY_FLOOR`]'s tables, and all four of rung 4b's entry-criterion
    /// angles, are measured on the lofted disc. If it is clear of the floor, those tables'
    /// *drivability* columns do not move and only their scanned-disc fidelity columns do.
    ///
    /// **Validated against a known value, for free:** the scanned arm must reproduce the recon's
    /// `0 of 14489` native / `4342 of 14489` scaled. That is what says this is the same
    /// instrument the diagnosis was built on, rather than a second one that happens to print
    /// similar-looking numbers.
    ///
    /// The lofted arm is **reported, never asserted**. Its answer selects a branch of the fix
    /// rung's plan, and a threshold pinned before the mechanism is known would only freeze
    /// today's draw — the mistake rung 5's step 0 made when it gated the clamp *depth* instead
    /// of the pinned *population*.
    ///
    /// ## ⚠ The triangle census is a PROXY; the second half measures the consequence
    ///
    /// The fraction of triangles under the floor is not the quantity that matters, and the
    /// relationship between the two is not linear: a **single** zeroed pseudo-normal reports
    /// "inside" at *any* distance, so a mesh with 0.3 % of its triangles under the floor can
    /// still stuff phantom material far from the surface. So the census is followed by the
    /// SDF-free measurement it is a proxy for — tets whose centroids fall outside the surface's
    /// own AABB — with the disc meshed in **both** frames so the difference is attributable.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn frame_fix_step0_area_floor_margin_by_geometry_fom() {
        use cf_fsu_geometry::load_from_env;

        let (l4, l5) = (
            load_from_env("CF_L4_STL").unwrap(),
            load_from_env("CF_L5_STL").unwrap(),
        );
        let scanned = load_from_env("CF_DISC_STL").unwrap();
        let lofted = lofted_disc(&l4, &l5);

        // The KNOWN-VALUE arm first: if this does not reproduce, nothing below is comparable
        // to the diagnosis.
        println!("\n--- SCANNED disc (FMA16036) — the recon's geometry ---");
        let (nat_s, scaled_s, total_s) = report_area_floor_margin(&scanned);
        assert_eq!(
            (nat_s, scaled_s, total_s),
            (0, 4342, 14489),
            "the scanned disc must reproduce the recon's area-floor census (0 / 4342 of 14489) \
             — if it does not, this instrument is not the one the diagnosis was built on"
        );

        println!(
            "\n--- LOFTED disc (assembled from L4/L5 endplate patches) — the open question ---"
        );
        let (nat_l, scaled_l, total_l) = report_area_floor_margin(&lofted);
        #[allow(clippy::cast_precision_loss)]
        let pct = |n: usize, d: usize| 100.0 * n as f64 / d as f64;
        println!(
            "\nVERDICT | scanned: native {nat_s} ({:.2} %) scaled {scaled_s} ({:.2} %) of {total_s} \
             | lofted: native {nat_l} ({:.2} %) scaled {scaled_l} ({:.2} %) of {total_l}",
            pct(nat_s, total_s),
            pct(scaled_s, total_s),
            pct(nat_l, total_l),
            pct(scaled_l, total_l),
        );
        println!(
            "{}",
            if scaled_l == 0 {
                "⇒ the LOFTED disc is CLEAR of the floor in both frames: the floor tables' \
                 drivability columns and rung 4b's entry criterion do NOT move."
            } else {
                "⇒ the LOFTED disc is AFFECTED: the floor tables' drivability columns and rung \
                 4b's entry criterion are both in the blast radius, and both must be re-measured."
            }
        );

        // ── The consequence the census is a proxy for, measured on both geometries. ──
        println!("\n════ PHANTOM MATERIAL — the SDF-free consequence, both frames ════");
        for (label, mesh) in [("SCANNED", &scanned), ("LOFTED ", &lofted)] {
            phantom_by_frame(label, mesh);
        }
    }

    /// **α.2's known-value check: does `SurfaceHealth` reproduce the census the frame-bug
    /// diagnosis was written in?**
    ///
    /// `TriMeshDistance::health` is a brand-new instrument, and a new instrument with no
    /// anchor is what this arc keeps being punished by. [`report_area_floor_margin`] above
    /// already produces the number the whole diagnosis rests on — **0 of 14489 triangles
    /// under parry's area floor in native mm, 4342 once rescaled to solver metres** — and
    /// it is asserted, not merely printed, by
    /// `frame_fix_step0_area_floor_margin_by_geometry_fom`.
    ///
    /// So the new census gets checked against it on the same geometry, as an **exact
    /// integer match** rather than a resemblance. If the two disagree, one of them is
    /// wrong and neither should be believed until that is settled.
    ///
    /// ## ★ Why the caller-frame column is the one that anchors
    ///
    /// Since α.1 the oracle normalises internally, so the *internal* census on the metre
    /// disc reads a healthy zero — that is the repair working. The quantity comparable to
    /// the diagnosis is `faces_under_floor_caller`: how many triangles parry **would** have
    /// dropped had the oracle been built in the units the caller handed over, which is
    /// exactly what happened before α.1.
    ///
    /// ## ⚠ This is a VALIDATION, not a regression gate
    ///
    /// It is `#[ignore]`d and licence-gated, so CI never runs it and a change to
    /// `mesh-sdf` will not trip it. It was run by hand against the pinned, checksummed
    /// `BodyParts3D` triad and its result recorded in the commit that added it. Read it as
    /// "the instrument was checked against a known value once, here is how to redo it" —
    /// **not** as ongoing protection. The instrument's live cover is `mesh-sdf`'s own
    /// licence-free gates in `health.rs`.
    ///
    /// ## ⚠ What this does NOT close
    ///
    /// R3 asked whether real decimated scans carry sub-floor slivers. α.2's downstream
    /// sweep answered the *mechanism* half — decimating a uniform body to
    /// `SDF_SOURCE_TARGET_FACES` does not manufacture them; the frame does — but that sweep
    /// was an uncommitted spike, so treat it as a recorded observation rather than a
    /// standing result. What *is* reproducible is the gap measured below: a uniform body
    /// has far less spread between its smallest and median triangle than this scan does.
    /// This test measures one real scan. It does not survey the workspace's scan consumers,
    /// and **R3 stays open.**
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn alpha2_surface_health_reproduces_the_area_floor_census_on_the_real_disc() {
        use cf_fsu_geometry::load_from_env;

        let scanned = load_from_env("CF_DISC_STL").unwrap();
        let scale = DiscParams::default().scale;

        // The known value, from the instrument the diagnosis was built on.
        let (native_known, scaled_known, total_known) = report_area_floor_margin(&scanned);
        assert_eq!(
            (native_known, scaled_known, total_known),
            (0, 4342, 14489),
            "the reference instrument no longer reproduces the recon's census, so it cannot \
             anchor anything — settle that before reading the comparison below"
        );

        // The metre frame, built by the pipeline's own transform: recentre to the AABB
        // centre, then scale. Areas are translation-invariant, so the recentring cannot
        // move the census — but it is reproduced rather than skipped, because the claim is
        // about the mesh `prepare_disc_at` actually hands the oracle.
        let bbox = Aabb::from_points(scanned.vertices.iter());
        let center = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
        let mut metre = scanned.clone();
        for v in &mut metre.vertices {
            *v = Point3::from((v.coords - center.coords) * scale);
        }

        let native_health = cf_fsu_geometry::oracle(&scanned).unwrap().distance.health();
        let metre_health = cf_fsu_geometry::oracle(&metre).unwrap().distance.health();
        println!("\nnative mm : {native_health}");
        println!("scaled  m : {metre_health}");

        // ── The anchor: exact integers, both frames. ──
        assert_eq!(
            native_health.faces, total_known,
            "the census counted a different number of faces than the reference instrument"
        );
        assert_eq!(
            native_health.faces_under_floor_caller, native_known,
            "native mm: SurfaceHealth says {} triangles under the floor, the reference \
             instrument says {native_known}",
            native_health.faces_under_floor_caller,
        );
        assert_eq!(
            metre_health.faces_under_floor_caller, scaled_known,
            "solver metres: SurfaceHealth says {} triangles under the floor, the reference \
             instrument says {scaled_known}. These compute the same quantity two ways — f64 \
             areas from the caller's vertices here, native areas times scale^2 there — so a \
             mismatch is a real difference in where the rescale lands faces relative to the \
             floor, not a tolerance to widen",
            metre_health.faces_under_floor_caller,
        );

        // ── R3's reading on the real disc: is the metre-frame oracle actually signable? ──
        println!(
            "\nR3 on the real disc | native: {} zeroed vertices, {} zeroed edges | metre: {} \
             zeroed vertices, {} zeroed edges",
            native_health.zero_pseudo_normal_vertices,
            native_health.zero_pseudo_normal_edges,
            metre_health.zero_pseudo_normal_vertices,
            metre_health.zero_pseudo_normal_edges,
        );
        for (label, h) in [
            ("native mm", &native_health),
            ("solver metres", &metre_health),
        ] {
            assert!(
                !h.has_zeroed_features(),
                "the {label} disc has unsignable features after internal normalisation, \
                 which is the defect remedy D was supposed to remove: {h}"
            );
        }

        // ★ Reported, not asserted: the NATIVE-mm disc is the clamped one, because it sits
        //   near z = 970 mm in the whole-atlas frame and the coordinate cap bounds absolute
        //   coordinates, not size. Recentring to the origin is what buys the metre frame a
        //   larger margin than the millimetre frame gets. Left as a print because it is a
        //   fact about where BodyParts3D happens to place this vertebra, not a property of
        //   the pipeline worth freezing — but it is exactly the silent clamp α.1 shipped
        //   with nothing able to report it.
        println!(
            "clamped? native {} (margin {:.1}) | metre {} (margin {:.1})",
            native_health.clamped,
            native_health.area_margin_binades_achieved,
            metre_health.clamped,
            metre_health.area_margin_binades_achieved,
        );

        // ── How far a synthetic body's answer can be carried to a real scan. ──
        //
        // ⚠ Measured here, on a licence-free fixture, rather than quoted. α.2's downstream
        //   sweep found the same contrast on decimated icospheres, but that sweep was a
        //   throwaway spike and left no producer in the tree — so the comparison is
        //   recomputed against `synthetic_disc`, which every other test in this module
        //   already uses and which anyone can run.
        let synthetic = cf_fsu_geometry::oracle(&synthetic_disc())
            .unwrap()
            .distance
            .health();
        // Named by value rather than by type: `cf-fsu-model` has no direct `mesh-sdf`
        // dependency, and a diagnostic print is not a reason to add one.
        let scan_spread = metre_health.median_area_internal / metre_health.min_area_internal;
        let synthetic_spread = synthetic.median_area_internal / synthetic.min_area_internal;
        println!(
            "area spread (median / min, internal): real scan {scan_spread:.1}x vs synthetic \
             {synthetic_spread:.1}x"
        );
        assert!(
            scan_spread > synthetic_spread * 20.0,
            "the real scan's area spread ({scan_spread:.1}x) is not decisively wider than a \
             synthetic body's ({synthetic_spread:.1}x). The claim that a uniform fixture \
             cannot stand in for a scan when reasoning about sub-floor slivers rests on \
             this gap being large"
        );
    }

    /// **Step 0b — why is the LOFTED disc ~40 % phantom in BOTH frames?**
    ///
    /// Step 0 measured that the frame fix moves the lofted disc's phantom material by exactly
    /// zero, while it takes the scanned disc's to zero. So the lofted disc has a **second,
    /// independent** defect, and this is the discriminator for it.
    ///
    /// Two hypotheses, and they predict different things:
    ///
    /// 1. **Open surface.** The loft's arc-length wall correspondence leaves open seam edges,
    ///    and `mesh-sdf`'s pseudo-normal sign is **undefined by contract** on a non-watertight
    ///    surface. Predicts: closing the boundary collapses the phantom material.
    /// 2. **Inverted / inconsistent winding.** `assemble_bushing` joins two auto-selected rims,
    ///    one of them flipped (`flip_patch`); a wall wound inside-out flips the sign over a
    ///    whole coherent region. Predicts: closing holes changes little, and `is_inside_out`
    ///    or a large contiguous wrong-sign region shows up instead.
    ///
    /// The measured shape favours (2) — 40 % phantom that is **contiguous** with the body (2
    /// components, 99.90 % retained) looks like a coherent sign flip, whereas a leak through a
    /// seam would be local. But that is a reading, and this arc has already spent a hypothesis
    /// on a surface defect that was real, present, and **irrelevant**: the scanned disc's
    /// 3-edge hole, where filling it changed literally nothing. So the same causal test that
    /// killed it runs here, before anything is built on either story.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    #[allow(clippy::cast_precision_loss)]
    fn frame_fix_step0b_lofted_disc_phantom_diagnosis_fom() {
        use cf_fsu_geometry::load_from_env;

        let (l4, l5) = (
            load_from_env("CF_L4_STL").unwrap(),
            load_from_env("CF_L5_STL").unwrap(),
        );
        let lofted = lofted_disc(&l4, &l5);
        let scanned = load_from_env("CF_DISC_STL").unwrap();

        // (1) HEALTH of both surfaces, side by side — the scanned disc is the control, since
        //     its defect is known and its phantom material is known to be frame-caused.
        for (label, m) in [("SCANNED", &scanned), ("LOFTED ", &lofted)] {
            let r = mesh_repair::validate_mesh(m);
            println!(
                "{label} health | watertight {} manifold {} INSIDE-OUT {} | boundary edges {} \
                 non-manifold edges {} degenerate {} duplicate {} | {} verts {} faces | \
                 enclosed volume {:.2} mm³",
                r.is_watertight,
                r.is_manifold,
                r.is_inside_out,
                r.boundary_edge_count,
                r.non_manifold_edge_count,
                r.degenerate_face_count,
                r.duplicate_face_count,
                m.vertices.len(),
                m.faces.len(),
                surface_volume(m),
            );
        }

        // (2) EXTENT, so "8.6x the tets" cannot be explained away as "it is a bigger disc".
        for (label, m) in [("SCANNED", &scanned), ("LOFTED ", &lofted)] {
            let s = Aabb::from_points(m.vertices.iter()).size();
            println!(
                "{label} AABB (native mm) | x {:.3} y {:.3} z {:.3} | AABB volume {:.2} mm³",
                s.x,
                s.y,
                s.z,
                s.x * s.y * s.z
            );
        }

        // (3) THE CAUSAL TEST. Close the boundary and re-measure the identical quantity. If
        //     hypothesis (1) is right the phantom material collapses; if it is wrong these
        //     numbers barely move, exactly as they did not move for the scanned disc's hole.
        println!("\n--- LOFTED, boundary CLOSED (the causal test) ---");
        let mut filled = lofted.clone();
        let n_filled = mesh_repair::fill_holes(&mut filled, 512).expect("fill holes");
        let after = mesh_repair::validate_mesh(&filled);
        println!(
            "filled {n_filled} hole(s) -> watertight {} manifold {} inside-out {} | boundary \
             edges {} | enclosed volume {:.2} mm³ (was {:.2})",
            after.is_watertight,
            after.is_manifold,
            after.is_inside_out,
            after.boundary_edge_count,
            surface_volume(&filled),
            surface_volume(&lofted),
        );
        phantom_by_frame("LOFTED-FILLED", &filled);
    }

    /// A **power-of-two normalised** view of an oracle that was built on the mesh rescaled by
    /// `s = 2^k`. This is the shape a `mesh-sdf`-side fix would take, expressed as a test-only
    /// adapter so it can be measured before any primitive changes.
    ///
    /// Distinct from [`NativeFrameOracle`] in the one way that matters for bookkeeping: that
    /// adapter builds on the **native-mm** surface (scale 1), whereas this builds on the
    /// **already-metre-scaled** surface times a power of two. `native × 1e-3` is *not* a power
    /// of two, so the two do not have to agree bit for bit, and whether they agree
    /// *mesh for mesh* is exactly what R1 asks.
    struct NormalisedOracle<'a> {
        inner: &'a MeshOracle,
        s: f64,
    }

    impl Sdf for NormalisedOracle<'_> {
        fn eval(&self, p: Point3<f64>) -> f64 {
            self.inner
                .eval(Point3::new(p.x * self.s, p.y * self.s, p.z * self.s))
                / self.s
        }
        fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
            // ⚠ The blanket `Sdf::grad` central-differences with an ABSOLUTE 1e-6 step, so
            // querying the inner oracle in the scaled frame makes the effective step
            // `1e-6 / s` in this frame. Normalisation therefore changes `grad`'s step size —
            // measured in `mesh-sdf`'s companion gate, not assumed here.
            self.inner
                .grad(Point3::new(p.x * self.s, p.y * self.s, p.z * self.s))
        }
    }

    /// The two reference arms R1 compares against, plus the `MeshingHints` every arm must
    /// share — production's own mesh (already in `meshed`) and the recon's native-frame
    /// adapter. Returning the hints is what keeps the sweep on **one** lattice: a normalised
    /// arm meshed over a different padded box would differ for a reason that has nothing to
    /// do with the oracle.
    fn native_frame_reference(
        disc_mesh: &IndexedMesh,
        meshed: &MeshedDisc,
        params: &DiscParams,
        v_surface: f64,
    ) -> (SdfMeshedTetMesh, MeshingHints) {
        println!("\n--- reference arms ---");
        report_phantom_material(
            "today  (metre oracle)  kept",
            &meshed.raw.largest_component(),
            &meshed.bbox,
            v_surface,
        );
        let native_sdf = oracle(disc_mesh).expect("native oracle");
        let adapter = NativeFrameOracle {
            inner: &native_sdf,
            center_native: meshed.center_native,
            scale: params.scale,
        };
        let padded = meshed.bbox.expanded(params.pad);
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(padded.min.x, padded.min.y, padded.min.z),
                Vec3::new(padded.max.x, padded.max.y, padded.max.z),
            ),
            cell_size: params.cell,
            material_field: Some(MaterialField::uniform(params.mu, 4.0 * params.mu)),
        };
        let native_mesh = SdfMeshedTetMesh::from_sdf(&adapter, &hints).expect("native-frame mesh");
        report_phantom_material(
            "native (recon adapter) kept",
            &native_mesh.largest_component(),
            &meshed.bbox,
            v_surface,
        );
        (native_mesh, hints)
    }

    /// Scale every vertex of `m` by exactly `s`.
    fn scaled_mesh(m: &IndexedMesh, s: f64) -> IndexedMesh {
        let mut out = m.clone();
        for v in &mut out.vertices {
            *v = Point3::new(v.x * s, v.y * s, v.z * s);
        }
        out
    }

    /// **R1 + R2 on the real disc — does a power-of-two normalisation reproduce the
    /// native-frame mesh, and at which target extent?**
    ///
    /// The re-anchor targets this whole arc is about to be rebuilt on — 6256 tets, 97.00 % of
    /// true volume, one component — were measured with [`NativeFrameOracle`], a *test* adapter
    /// that builds at scale 1 on the native-mm surface. A fix inside `mesh-sdf` cannot do that:
    /// it only ever sees the surface its caller handed it, which here is the metre-scaled one.
    /// It would normalise *that* by a power of two.
    ///
    /// **`native × 1e-3` is not a power of two**, so the two paths' `f64` values — and hence
    /// parry's `f64 -> f32` narrowing — differ. The meshes therefore need not be identical.
    /// Physically that is irrelevant; for bookkeeping it is decisive, because **every Tier-A
    /// re-anchor number must come from the path that actually ships**, not from the recon's
    /// adapter. This measures whether those are the same path.
    ///
    /// Sweeps the normalisation target so R2 (what extent to normalise to) is answered on real
    /// anatomy at the same time. Reports the floor margin per target.
    ///
    /// ## What it asserts now
    ///
    /// Production emits `(6256 raw, 6256 kept)`, and the native-frame adapter emits the same
    /// pair. **Tet counts are all that is compared** — equal counts do not imply equal
    /// vertices. The sweep's six targets must each reproduce the native counts.
    ///
    /// It previously asserted `(12517, 7759)` for production, which is what the recon
    /// measured before `TriMeshDistance` began normalising internally. That assert was red
    /// when this gate was next run.
    ///
    /// ⚠ **CI cannot run this test** — it is `#[ignore]`d and needs `$CF_DISC_STL`. A
    /// regression here surfaces only when someone runs the licence-gated suite by hand.
    ///
    /// ⚠ Every swept row is now asserted to match, so widening the sweep to probe where
    /// the oracle stops working would fail those assertions by design.
    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    #[allow(clippy::cast_precision_loss)]
    fn frame_fix_r1_power_of_two_normalisation_vs_native_frame_fom() {
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let params = DiscParams::default();
        let v_surface = surface_volume(&disc_mesh) * params.scale.powi(3);
        let floor = f64::from(f32::EPSILON) / 2.0;

        // Production's own head: the metre-scaled surface and the oracle it builds on it.
        let meshed = mesh_disc_raw(disc_mesh.clone(), &params).expect("mesh raw disc");
        let scaled_surface = meshed.sdf.mesh().clone();
        let extent = Aabb::from_points(scaled_surface.vertices.iter()).size();
        let max_extent = extent.x.max(extent.y).max(extent.z);
        let min_area = |m: &IndexedMesh| {
            m.faces
                .iter()
                .map(|f| {
                    let (a, b, c) = (
                        m.vertices[f[0] as usize],
                        m.vertices[f[1] as usize],
                        m.vertices[f[2] as usize],
                    );
                    0.5 * (b - a).cross(&(c - a)).norm()
                })
                .fold(f64::INFINITY, f64::min)
        };
        println!(
            "metre-scaled surface: extent {:.6} x {:.6} x {:.6} m (max {max_extent:.6}) | \
             min triangle area {:.4e} vs floor {floor:.4e} ({:.3}x)",
            extent.x,
            extent.y,
            extent.z,
            min_area(&scaled_surface),
            min_area(&scaled_surface) / floor,
        );

        // ── Reference arms: production today, and the native-frame adapter the recon used. ──
        let kept_now = meshed.raw.largest_component();
        let (native_mesh, hints) = native_frame_reference(&disc_mesh, &meshed, &params, v_surface);
        let native_kept = native_mesh.largest_component();
        println!(
            "  today  raw {} kept {} components {}\n  native raw {} kept {} components {}",
            meshed.raw.n_tets(),
            kept_now.n_tets(),
            face_components(&meshed.raw).len(),
            native_mesh.n_tets(),
            native_kept.n_tets(),
            face_components(&native_mesh).len(),
        );

        // ── R2 sweep: normalise the METRE surface by 2^k for a range of target extents. ──
        let (mut sweep_rows, mut sweep_matching) = (0_usize, 0_usize);
        println!("\n--- power-of-two normalisation of the metre surface (R2 sweep) ---");
        for target in [0.25_f64, 1.0, 4.0, 16.0, 64.0, 256.0] {
            #[allow(clippy::cast_possible_truncation)]
            let k = (target / max_extent).log2().round() as i32;
            let s = 2.0_f64.powi(k);
            let surf = scaled_mesh(&scaled_surface, s);
            let a_min = min_area(&surf);
            let norm_inner = oracle(&surf).expect("normalised oracle");
            let adapter = NormalisedOracle {
                inner: &norm_inner,
                s,
            };
            let tet = SdfMeshedTetMesh::from_sdf(&adapter, &hints).expect("normalised mesh");
            let kept = tet.largest_component();
            let matches_native =
                tet.n_tets() == native_mesh.n_tets() && kept.n_tets() == native_kept.n_tets();
            sweep_rows += 1;
            sweep_matching += usize::from(matches_native);
            println!(
                "target {target:>7.2} | k {k:>3} (s = 2^{k}) | realised extent {:.4} | \
                 min area {a_min:.4e} ({:.2}x floor) | raw {:>6} kept {:>6} components {:>4} | \
                 vs native: {}",
                max_extent * s,
                a_min / floor,
                tet.n_tets(),
                kept.n_tets(),
                face_components(&tet).len(),
                if matches_native {
                    "IDENTICAL counts"
                } else {
                    "DIFFERS"
                },
            );
        }

        assert_r1_production_matches_native_frame(
            (meshed.raw.n_tets(), kept_now.n_tets()),
            (native_mesh.n_tets(), native_kept.n_tets()),
            (sweep_matching, sweep_rows),
        );
    }

    /// The R1 answer, pinned: production and the native-frame adapter agree on both tet
    /// counts, and `kept == raw` means the largest-component filter discards nothing.
    ///
    /// Extracted from the test body so the assertions are nameable and the test stays under
    /// the line limit.
    fn assert_r1_production_matches_native_frame(
        production: (usize, usize),
        native: (usize, usize),
        sweep: (usize, usize),
    ) {
        let (matching, rows) = sweep;
        assert_eq!(
            production,
            (6256, 6256),
            "production (raw, kept) must be the shipped pair. It read 12517/7759 before \
             `TriMeshDistance` began normalising internally. Things that move it include \
             that normalisation, `DiscParams::default()`'s `cell`/`pad`/`scale` (they set \
             the lattice and the disc's size in it), the stuffer itself, `mesh-sdf`'s \
             coordinate cap, a parry point-query change under the caret range, or a \
             different disc mesh"
        );
        // Compared against `production`, not against the literal — so this reports an
        // arm that has diverged rather than restating the pin above. (Production is
        // already pinned, so in practice this fires for the native arm.)
        assert_eq!(
            native, production,
            "the native-frame adapter must agree with production on both counts. These \
             are the two paths the R1 question is about; if they part company, the sweep \
             rows already computed above were compared against the wrong reference"
        );
        // Scale-insensitivity: EVERY normalisation target reproduces the native frame, not
        // just the one production happens to pick. The row count is asserted too, so a
        // future edit that empties the sweep cannot pass vacuously.
        assert_eq!(rows, 6, "the R2 sweep must actually have run its rows");
        assert_eq!(
            matching, rows,
            "every normalisation target must reproduce the native frame — the fix is \
             insensitive to the target extent, which is why production does not have to \
             pick a special one"
        );
    }

    /// Per-triangle soup: every face gets its own three vertices, destroying all shared
    /// topology. `cf-spine-studio`'s `loft_painted_disc` does exactly this to the raw loft
    /// before welding it back, because — its own comment — "the raw loft tet-meshes into a
    /// shattered surface". Reproduced here so the fixture can be compared to production on
    /// the one variable that differs.
    fn explode_to_soup(m: &IndexedMesh) -> IndexedMesh {
        let mut vertices = Vec::with_capacity(m.faces.len() * 3);
        let mut faces = Vec::with_capacity(m.faces.len());
        for f in &m.faces {
            let base = u32::try_from(vertices.len()).expect("soup vertex count fits u32");
            for &v in f {
                vertices.push(m.vertices[v as usize]);
            }
            faces.push([base, base + 1, base + 2]);
        }
        IndexedMesh { vertices, faces }
    }

    /// **Step 0c — is the lofted disc's phantom material a FIXTURE defect?**
    ///
    /// Steps 0 and 0b established that the lofted disc is ~40 % phantom, that the frame fix
    /// moves it by zero, that closing its boundary moves it by 0.2 %, and that its winding is
    /// consistent. So neither of the SDF-side hypotheses survived — which turns the question
    /// around: is the *input* a valid disc at all?
    ///
    /// Reading `cf-spine-studio`'s `loft_painted_disc` says probably not, in **two** ways:
    ///
    /// 1. **Face selection.** The Studio lofts from *painted* faces. [`lofted_disc`] lofts from
    ///    [`select_endplate`]'s automatic `normal.z · sign > 0.7` predicate, which on a real
    ///    vertebra can also take up-facing facets on the transverse and articular processes.
    /// 2. **Topology.** The Studio explodes the raw loft to per-triangle soup and re-welds it
    ///    (`repair_mesh`), because the raw loft "tet-meshes into a shattered surface".
    ///    [`lofted_disc`] returns `assemble_bushing(..).mesh` **raw** and does neither.
    ///
    /// (2) is the one with a mechanism: unwelded seam vertices mean a position that should
    /// carry one pseudo-normal instead carries several, each accumulated from only *some* of
    /// its incident triangles — which is frame-independent, hole-independent and
    /// winding-consistent, exactly the signature measured.
    ///
    /// This measures both. (1) by extent — a patch that reaches the process tips is wider than
    /// the vertebral body. (2) by the causal test: run production's own repair over the
    /// fixture's loft, change nothing else, and re-measure the identical quantity.
    ///
    /// Asserts nothing. It is a diagnosis, and the arc has twice been wrong about this disc by
    /// reasoning instead of running.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    #[allow(clippy::cast_precision_loss)]
    fn frame_fix_step0c_lofted_fixture_vs_production_pipeline_fom() {
        use cf_fsu_geometry::load_from_env;

        let (l4, l5) = (
            load_from_env("CF_L4_STL").unwrap(),
            load_from_env("CF_L5_STL").unwrap(),
        );

        // ── (1) Does `select_endplate` over-select? Compare the patch to the whole bone. ──
        println!("════ PATCH EXTENT — is the auto-selected 'endplate' an endplate? ════");
        for (label, mesh, sign) in [("L4 (inferior)", &l4, -1.0), ("L5 (superior)", &l5, 1.0)] {
            let faces = select_endplate(mesh, sign);
            let pts: Vec<Point3<f64>> = faces
                .iter()
                .flat_map(|&i| mesh.faces[i])
                .map(|v| mesh.vertices[v as usize])
                .collect();
            let whole = Aabb::from_points(mesh.vertices.iter()).size();
            let patch = Aabb::from_points(pts.iter()).size();
            println!(
                "{label} | whole bone x {:.2} y {:.2} z {:.2} | PATCH x {:.2} y {:.2} z {:.2} \
                 ({:.0} % of bone width) | {} of {} faces selected",
                whole.x,
                whole.y,
                whole.z,
                patch.x,
                patch.y,
                patch.z,
                100.0 * patch.x / whole.x,
                faces.len(),
                mesh.faces.len(),
            );
        }

        // ── (2) THE CAUSAL TEST: production's repair, one variable, same measurement. ──
        let raw_loft = lofted_disc(&l4, &l5);
        let mut welded = explode_to_soup(&raw_loft);
        let summary =
            mesh_repair::repair_mesh(&mut welded, &mesh_repair::RepairParams::for_scans());
        println!("\n════ TOPOLOGY — the fixture's raw loft vs production's welded one ════");
        println!(
            "repair: {} verts {} faces -> {} verts {} faces | {summary:?}",
            raw_loft.vertices.len(),
            raw_loft.faces.len(),
            welded.vertices.len(),
            welded.faces.len(),
        );
        for (label, m) in [
            ("FIXTURE (raw loft)", &raw_loft),
            ("PRODUCTION (welded)", &welded),
        ] {
            let r = mesh_repair::validate_mesh(m);
            println!(
                "{label} | watertight {} manifold {} inside-out {} | boundary edges {} \
                 non-manifold {} degenerate {} duplicate {} | enclosed volume {:.2} mm³",
                r.is_watertight,
                r.is_manifold,
                r.is_inside_out,
                r.boundary_edge_count,
                r.non_manifold_edge_count,
                r.degenerate_face_count,
                r.duplicate_face_count,
                surface_volume(m),
            );
        }
        for (label, m) in [("FIXTURE-RAW", &raw_loft), ("PRODUCTION-WELDED", &welded)] {
            phantom_by_frame(label, m);
        }
    }

    /// Mesh `disc_mesh` twice — once in production's rescaled frame, once with the oracle built
    /// in native mm — and report how much provably-off-disc material each one stuffs.
    ///
    /// Everything reported is either SDF-free (centroids outside the surface's own AABB;
    /// outside the box is outside the solid) or connectivity (`face_components`), so none of it
    /// routes through the oracle whose sign is under test.
    ///
    /// ⚠ The `% of the surface's volume` column divides by a divergence-theorem volume, which
    /// assumes a closed surface. The lofted disc carries a few open wall-seam edges, so read
    /// **its** percentage as indicative and the AABB/component columns as the hard ones.
    #[allow(clippy::cast_precision_loss)]
    fn phantom_by_frame(label: &str, disc_mesh: &IndexedMesh) {
        let params = DiscParams::default();
        let v_surface = surface_volume(disc_mesh) * params.scale.powi(3);

        // Arm 1: exactly what production does today.
        let meshed = mesh_disc_raw(disc_mesh.clone(), &params).expect("mesh raw disc");
        let kept = meshed.raw.largest_component();

        // Arm 2: one variable changed — the frame the oracle is BUILT in.
        let native_sdf = oracle(disc_mesh).expect("native oracle");
        let adapter = NativeFrameOracle {
            inner: &native_sdf,
            center_native: meshed.center_native,
            scale: params.scale,
        };
        let padded = meshed.bbox.expanded(params.pad);
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(padded.min.x, padded.min.y, padded.min.z),
                Vec3::new(padded.max.x, padded.max.y, padded.max.z),
            ),
            cell_size: params.cell,
            material_field: Some(MaterialField::uniform(params.mu, 4.0 * params.mu)),
        };
        let fixed = SdfMeshedTetMesh::from_sdf(&adapter, &hints).expect("mesh with native oracle");
        let fixed_kept = fixed.largest_component();

        println!("\n--- {label} ---");
        let (out_now, vol_now, reach_now) = report_phantom_material(
            "  today (rescaled frame) kept",
            &kept,
            &meshed.bbox,
            v_surface,
        );
        let (out_fix, vol_fix, reach_fix) = report_phantom_material(
            "  fixed (native frame)   kept",
            &fixed_kept,
            &meshed.bbox,
            v_surface,
        );
        println!(
            "  components {} -> {} | raw tets {} -> {} | kept tets {} -> {} | retained {:.2} % -> {:.2} %",
            face_components(&meshed.raw).len(),
            face_components(&fixed).len(),
            meshed.raw.n_tets(),
            fixed.n_tets(),
            kept.n_tets(),
            fixed_kept.n_tets(),
            100.0 * kept.n_tets() as f64 / meshed.raw.n_tets() as f64,
            100.0 * fixed_kept.n_tets() as f64 / fixed.n_tets() as f64,
        );
        println!(
            "  ⇒ {label}: phantom tets {out_now} -> {out_fix} | volume {vol_now:.2} % -> \
             {vol_fix:.2} % of true | overhang {reach_now:.4} -> {reach_fix:.4} mm"
        );
    }

    /// Size, enclosed volume, repair health, and exterior sign probes of the input surface,
    /// in its native millimetre frame. Returns the native-frame oracle for reuse.
    fn report_surface_health(disc_mesh: &IndexedMesh) -> MeshOracle {
        let bbox = Aabb::from_points(disc_mesh.vertices.iter());
        let size = bbox.size();
        println!(
            "surface bbox (native mm): x {:.4} y {:.4} z {:.4} | {} verts {} faces",
            size.x,
            size.y,
            size.z,
            disc_mesh.vertices.len(),
            disc_mesh.faces.len(),
        );
        println!(
            "enclosed volume {:.2} mm³; AABB volume {:.2} mm³ (fill {:.1} %)",
            surface_volume(disc_mesh),
            size.x * size.y * size.z,
            100.0 * surface_volume(disc_mesh) / (size.x * size.y * size.z),
        );

        // Mesh health of the input AS THE PIPELINE RECEIVES IT (post-`load`, which runs a
        // repair pass and discards its statistics). The pseudo-normal sign is only defined
        // for a closed, consistently-oriented, manifold surface, so if this says the disc is
        // none of those, nothing downstream can be assumed to have a sign at all.
        let report = mesh_repair::validate_mesh(disc_mesh);
        println!(
            "input health | watertight {} manifold {} inside-out {} | boundary edges {} \
             non-manifold edges {} degenerate faces {} duplicate faces {}",
            report.is_watertight,
            report.is_manifold,
            report.is_inside_out,
            report.boundary_edge_count,
            report.non_manifold_edge_count,
            report.degenerate_face_count,
            report.duplicate_face_count,
        );

        // INDEPENDENT max-depth oracle: the deepest interior point of the field, found by
        // scan rather than by asking about any particular tet. No interior point of a solid
        // of SI extent `t` can be deeper than `t/2` — the bound the census's reading must
        // clear to be believable.
        report_area_floor_margin(disc_mesh);

        let native_sdf = oracle(disc_mesh).expect("native oracle");
        let (deepest, depth) = cf_fsu_geometry::body_center(disc_mesh, &native_sdf);
        println!(
            "deepest interior point {deepest:?} at depth {:.4} mm | half SI extent {:.4} mm",
            -depth,
            size.z / 2.0,
        );

        // Probe points that are exterior BY CONSTRUCTION — each displaced from the surface
        // AABB by a margin no solid inside that box can reach. Every φ must be positive.
        // `body_center` scans only the AABB, so these cover the space it never looks at.
        let ctr = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
        let half = size / 2.0;
        for (name, probe) in [
            (
                "+x  10 mm out",
                Point3::new(ctr.x + half.x + 10.0, ctr.y, ctr.z),
            ),
            (
                "-x  10 mm out",
                Point3::new(ctr.x - half.x - 10.0, ctr.y, ctr.z),
            ),
            (
                "+y  10 mm out",
                Point3::new(ctr.x, ctr.y + half.y + 10.0, ctr.z),
            ),
            (
                "+z  10 mm out",
                Point3::new(ctr.x, ctr.y, ctr.z + half.z + 10.0),
            ),
            (
                "corner  4 mm",
                Point3::new(
                    ctr.x + half.x + 4.0,
                    ctr.y + half.y + 4.0,
                    ctr.z + half.z + 4.0,
                ),
            ),
            (
                "far  100 mm  ",
                Point3::new(ctr.x + 100.0, ctr.y + 100.0, ctr.z + 100.0),
            ),
        ] {
            let phi = native_sdf.eval(probe);
            println!(
                "  probe {name}: φ = {phi:+.4} mm  {}",
                if phi > 0.0 {
                    "ok (outside)"
                } else {
                    "★ WRONG SIGN — reported interior"
                }
            );
        }
        native_sdf
    }

    /// Where the input's triangles sit relative to the area floor parry silently drops
    /// triangles at, in both the native-mm frame and the SI-metre frame the disc pipeline
    /// rescales into.
    ///
    /// `Triangle::normal` is `Unit::try_new(ab × ac, parry3d::math::DEFAULT_EPSILON)`, and
    /// `parry3d` is the f32 build, so that epsilon is `f32::EPSILON`. ‖ab × ac‖ is twice the
    /// area, so a triangle whose area is at or under `f32::EPSILON / 2` never contributes to
    /// `compute_pseudo_normals` — leaving a ZERO pseudo-normal on its vertices and edges,
    /// which parry's `dpt.dot(&pseudo_normal) <= 0.0` inside test reads as "inside" at any
    /// distance. The floor is ABSOLUTE and area goes as length², so the mm → m rescale moves
    /// every triangle 1e6 times closer to it.
    ///
    /// Returns `(under the floor in native mm, under the floor once scaled, total faces)` so a
    /// caller can validate this instrument against a known value instead of reading its print.
    fn report_area_floor_margin(disc_mesh: &IndexedMesh) -> (usize, usize, usize) {
        let eps_area = f64::from(f32::EPSILON) / 2.0;
        let mut areas: Vec<f64> = disc_mesh
            .faces
            .iter()
            .map(|f| {
                let (a, b, c) = (
                    disc_mesh.vertices[f[0] as usize],
                    disc_mesh.vertices[f[1] as usize],
                    disc_mesh.vertices[f[2] as usize],
                );
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .collect();
        areas.sort_by(f64::total_cmp);
        let scale = DiscParams::default().scale;
        println!("parry area floor = f32::EPSILON/2 = {eps_area:.4e} (absolute, any unit)");
        let mut under_by_frame = [0usize; 2];
        for (i, (frame, factor)) in [("native mm", 1.0), ("scaled  m", scale * scale)]
            .into_iter()
            .enumerate()
        {
            let under = areas.iter().filter(|a| *a * factor <= eps_area).count();
            under_by_frame[i] = under;
            println!(
                "  {frame} | min {:.4e} median {:.4e} | margin of the SMALLEST triangle over \
                 the floor: {:.1}x | under the floor: {under} of {}",
                areas[0] * factor,
                areas[areas.len() / 2] * factor,
                areas[0] * factor / eps_area,
                areas.len(),
            );
        }
        (under_by_frame[0], under_by_frame[1], areas.len())
    }

    /// Ask the **scaled** oracle (the one the mesher consumed) and the **native-mm** oracle
    /// (the one every other consumer in the workspace builds) about the same physical point
    /// on the same anatomy.
    ///
    /// Geometry is scale-free, so a disagreement here is not a fact about the disc — it is a
    /// fact about the frame the oracle was built in. The inverse transform is self-checking:
    /// if the point were not the same one, the two magnitudes would not agree.
    ///
    /// ⚠ **The probed point is a SELECTED WORST CASE, not a sample** — it is the argmin of
    /// the scaled oracle. It demonstrates that the frames disagree; it says nothing about
    /// *how often*. For the rate, read `report_area_floor_margin`.
    ///
    /// ⚠⚠ **Only HALF of the pair this used to cite survives, and the halves must not be
    /// conflated.** It read: *"`report_area_floor_margin` (30 % of triangles under the floor) and
    /// `report_phantom_material` (32.94 % of tets outside the AABB)."*
    ///
    /// - **The 30 % still holds.** It is a property of the raw triangle areas in the metre frame,
    ///   which α.1 did not change — α.2 measured 4342 of 14489 (30.0 %) under floor there. What α.1
    ///   removed is the *consequence*, not the count: the oracle now normalises internally, so
    ///   those small triangles no longer produce zeroed pseudo-normals.
    /// - **The 32.94 % is DEAD.** `mesh_stability_step0_discarded_component_census_fom` now reads
    ///   0 discarded tets and 0 islands at every cell, with kept volume *below* the surface's own
    ///   (96.83–98.97 %). There is no phantom material left to be outside the AABB.
    ///
    /// ⇒ a proxy figure surviving while its consequence disappears is exactly the distinction α.2's
    /// `SurfaceHealth` split into proxy / decision / consequence layers. Read the 30 % as a proxy
    /// with no live consequence, and do not re-pair it with a phantom-material rate.
    #[allow(clippy::cast_precision_loss)]
    fn report_scaled_frame_disagreement(
        meshed: &MeshedDisc,
        native_sdf: &MeshOracle,
        params: &DiscParams,
    ) {
        let pos = meshed.raw.positions();
        let (mut outside_n, mut outside_v, mut total_v) = (0usize, 0.0, 0.0);
        let (mut phi_lo, mut phi_hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut deepest_centroid = Vec3::zeros();
        for tid in 0..meshed.raw.n_tets() {
            let tid = u32::try_from(tid).expect("tet id fits u32");
            let corners = meshed.raw.tet_vertices(tid);
            let centroid: Vec3 = corners.iter().map(|&i| pos[i as usize]).sum::<Vec3>() / 4.0;
            let phi_mm = meshed.sdf.eval(Point3::from(centroid)) * 1e3;
            if phi_mm < phi_lo {
                phi_lo = phi_mm;
                deepest_centroid = centroid;
            }
            phi_hi = phi_hi.max(phi_mm);
            let vol = meshed.raw.quality().signed_volume[tid as usize].abs();
            total_v += vol;
            if phi_mm > 0.0 {
                outside_n += 1;
                outside_v += vol;
            }
        }
        println!(
            "surface bbox (scaled m): min ({:.6}, {:.6}, {:.6}) max ({:.6}, {:.6}, {:.6})",
            meshed.bbox.min.x,
            meshed.bbox.min.y,
            meshed.bbox.min.z,
            meshed.bbox.max.x,
            meshed.bbox.max.y,
            meshed.bbox.max.z,
        );
        println!(
            "tet centroid φ range: {phi_lo:.4} .. {phi_hi:.4} mm (negative = inside)\n\
             centroids the BROKEN oracle still admits are outside: {outside_n} of {} \
             ({:.2} %), carrying {:.2} % of the raw mesh volume — a LOWER bound, since the \
             misclassified exterior material is exactly what this count misses; \
             `report_phantom_material`'s AABB test is the oracle-free number",
            meshed.raw.n_tets(),
            100.0 * outside_n as f64 / meshed.raw.n_tets() as f64,
            100.0 * outside_v / total_v,
        );

        // WHERE does the impossible depth sit? Inside the surface AABB it would contradict
        // `body_center`'s scan of that same box; outside it, `body_center` never looked.
        let over = Vec3::new(
            deepest_centroid.x.abs() - meshed.bbox.max.x,
            deepest_centroid.y.abs() - meshed.bbox.max.y,
            deepest_centroid.z.abs() - meshed.bbox.max.z,
        );
        println!(
            "  deepest centroid at {deepest_centroid:?} m | outside surface AABB: {} | \
             per-axis overhang ({:.4}, {:.4}, {:.4}) mm",
            over.x > 0.0 || over.y > 0.0 || over.z > 0.0,
            over.x * 1e3,
            over.y * 1e3,
            over.z * 1e3,
        );
        let native_pt = Point3::from(deepest_centroid / params.scale + meshed.center_native.coords);
        let native_phi = native_sdf.eval(native_pt);
        println!(
            "  same point, native-mm oracle: φ = {native_phi:+.4} mm (scaled oracle said \
             {phi_lo:+.4} mm) — {}",
            if native_phi.signum() == phi_lo.signum() {
                "frames AGREE"
            } else {
                "★ FRAMES DISAGREE ON THE SIGN"
            },
        );
    }

    /// How much of `mesh` provably sits off the disc: tets whose centroid falls outside the
    /// surface's own AABB. Outside the box is outside the solid, so this needs no oracle and
    /// is a strict lower bound (the AABB is far larger than the lens it contains).
    ///
    /// Returns `(tets beyond the AABB, volume as % of the surface's, furthest overhang in mm)`
    /// so a caller can compare two geometries on the numbers rather than on the prints.
    #[allow(clippy::cast_precision_loss)]
    fn report_phantom_material(
        label: &str,
        mesh: &SdfMeshedTetMesh,
        bbox: &Aabb,
        v_surface: f64,
    ) -> (usize, f64, f64) {
        let pos = mesh.positions();
        let (mut out_v, mut all_v, mut out_n) = (0.0, 0.0, 0usize);
        let mut reach = 0.0_f64;
        for tid in 0..mesh.n_tets() {
            let tid = u32::try_from(tid).expect("tet id fits u32");
            let corners = mesh.tet_vertices(tid);
            let centroid: Vec3 = corners.iter().map(|&i| pos[i as usize]).sum::<Vec3>() / 4.0;
            all_v += mesh.quality().signed_volume[tid as usize].abs();
            if centroid.x < bbox.min.x
                || centroid.x > bbox.max.x
                || centroid.y < bbox.min.y
                || centroid.y > bbox.max.y
                || centroid.z < bbox.min.z
                || centroid.z > bbox.max.z
            {
                out_n += 1;
                out_v += mesh.quality().signed_volume[tid as usize].abs();
            }
            for &i in &corners {
                let pt = pos[i as usize];
                reach = reach.max(
                    (pt.x.abs() - bbox.max.x)
                        .max(pt.y.abs() - bbox.max.y)
                        .max(pt.z.abs() - bbox.max.z),
                );
            }
        }
        println!(
            "{label} | volume {all_v:.4e} m³ = {:.2} % of the surface's | centroids beyond the \
             surface AABB: {out_n} ({:.2} % of tets, {:.2} % of volume) | furthest corner \
             overhang {:.4} mm",
            100.0 * all_v / v_surface,
            100.0 * out_n as f64 / mesh.n_tets() as f64,
            100.0 * out_v / all_v,
            reach * 1e3,
        );
        (out_n, 100.0 * all_v / v_surface, reach * 1e3)
    }

    /// One `cell` of [`mesh_stability_step0_discarded_component_census_fom`]: mesh, decompose,
    /// validate against production, report.
    #[allow(clippy::cast_precision_loss)]
    fn census_one_cell(disc_mesh: &IndexedMesh, params: &DiscParams, v_surface: f64) {
        let cell = params.cell;
        let m = mesh_disc_raw(disc_mesh.clone(), params)
            .unwrap_or_else(|e| panic!("mesh raw disc at cell {cell}: {e:?}"));

        // Radial normaliser from the SURFACE, not the tets: a tet-derived maximum would
        // shrink with whatever the mesher dropped, hiding the very effect under study.
        let r_surface_max = disc_mesh
            .vertices
            .iter()
            .map(|v| {
                ((v.x - m.center_native.x) * params.scale)
                    .hypot((v.y - m.center_native.y) * params.scale)
            })
            .fold(0.0_f64, f64::max);
        let band = params.band_frac * (m.bbox.max.z - m.bbox.min.z);
        let band_z = (m.bbox.min.z + band, m.bbox.max.z - band);

        let comps = face_components(&m.raw);
        let total: usize = comps.iter().map(Vec::len).sum();
        assert_eq!(
            total,
            m.raw.n_tets(),
            "cell {cell}: components must partition the mesh ({total} vs {})",
            m.raw.n_tets()
        );

        // ★ HARNESS VALIDATION, and the load-bearing assert of this test: production's kept
        // mesh must BE this file's largest component, tet for tet. If it is not, every
        // "discarded" number below describes a set production never discarded.
        let kept_ids = &comps[0];
        let production = prepare_disc(disc_mesh.clone(), params, None)
            .unwrap_or_else(|e| panic!("prepare raw disc at cell {cell}: {e:?}"));
        let mut mine: Vec<[VertexId; 4]> =
            kept_ids.iter().map(|&t| m.raw.tet_vertices(t)).collect();
        let mut theirs: Vec<[VertexId; 4]> = (0..production.tet.n_tets())
            .map(|t| {
                production
                    .tet
                    .tet_vertices(u32::try_from(t).expect("tet id fits u32"))
            })
            .collect();
        mine.sort_unstable();
        theirs.sort_unstable();
        assert_eq!(
            mine,
            theirs,
            "cell {cell}: this file's largest component is not the mesh production keeps \
             ({} vs {} tets) — the census would be describing the wrong set",
            mine.len(),
            theirs.len()
        );

        let discarded: Vec<TetId> = comps[1..].iter().flatten().copied().collect();
        let kept = summarise(&m.raw, kept_ids, &m.sdf, r_surface_max, band_z);
        let lost = summarise(&m.raw, &discarded, &m.sdf, r_surface_max, band_z);
        let v_raw = kept.volume + lost.volume;

        println!(
            "\ncell {cell:.5} m | components {:5} | tets raw {:6} kept {:6} ({:.2} %) \
             discarded {:5}",
            comps.len(),
            m.raw.n_tets(),
            kept_ids.len(),
            100.0 * kept_ids.len() as f64 / m.raw.n_tets() as f64,
            discarded.len(),
        );
        println!(
            "  volume | raw {v_raw:.4e} ({:.2} % of surface) | kept {:.4e} ({:.2} % of surface, \
             {:.2} % of raw) | DISCARDED {:.4e} ({:.3} % of raw)",
            100.0 * v_raw / v_surface,
            kept.volume,
            100.0 * kept.volume / v_surface,
            100.0 * kept.volume / v_raw,
            lost.volume,
            100.0 * lost.volume / v_raw,
        );
        for (name, reg) in [("kept     ", &kept), ("DISCARDED", &lost)] {
            println!(
                "  {name} | r/r_max mean {:.3} max {:.3} | depth mean {:.4} mm max {:.4} mm \
                 | in endplate band {:.1} % of its volume",
                reg.mean_r_norm,
                reg.max_r_norm,
                reg.mean_depth_mm,
                reg.max_depth_mm,
                100.0 * reg.band_frac_of_volume,
            );
        }
        let sizes: Vec<usize> = comps[1..].iter().map(Vec::len).collect();
        let singletons = sizes.iter().filter(|&&s| s == 1).count();
        println!(
            "  islands | {} of them, largest {:?}, singletons {singletons} ({:.1} %)",
            sizes.len(),
            sizes.iter().take(8).collect::<Vec<_>>(),
            100.0 * singletons as f64 / sizes.len().max(1) as f64,
        );
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
    ///
    /// ⚠ **Undefined on an EMPTY population, and it fails silently**: the RMS divides by the
    /// count, so an empty slice returns `(0.0, NaN)` — which prints as a plausible-looking row and
    /// makes any `±5 %` comparison against it false rather than loud.
    ///
    /// This is live, not hypothetical. The guard-declined populations are the ones that can empty
    /// out, and on the shipped post-α.1 geometry the declined **midsides number zero**, so
    /// `curved_tet10_midsides_seat_on_the_endplate_fom`'s committed `guard-declined rms 4.200 mm`
    /// pin now compares against `NaN`. ▶ That assert cannot be re-anchored — the RMS of an empty
    /// set is not a quantity — it has to be retired in favour of a pin on the population *count*.
    /// Deliberately left standing here: it belongs with the re-anchor rung that has the run to
    /// justify what replaces it, not with the rung that added the decline instrument.
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
    /// - `guard_declined` — candidates the discriminator refused, split by **which guard
    ///   refused them** into `beyond_cap` and `lateral`.
    ///
    /// ⚠ **The decline split is not decoration, and this comment used to get it wrong.** It read
    /// "candidates the discriminator refused, i.e. the overhanging annular rim that #701's settled
    /// call leaves straight on purpose" — an attribution to the SI-alignment guard. But
    /// `cf_fsu_geometry`'s conform tests the distance cap **first**, so a node past the cap never
    /// has its direction computed at all (`ConformDecision::BeyondCap`, and the order gate
    /// `a_node_past_the_cap_is_attributed_to_the_cap_not_to_direction` pins it). Only `lateral`
    /// is the rim #701 reasoned about; `beyond_cap` is a node too far from any bone, which is a
    /// registration gap — or material that should not be in the mesh at all.
    struct ConformSplit {
        candidates: Vec<VertexId>,
        authorised: Vec<VertexId>,
        guard_declined: Vec<VertexId>,
        /// Refused by the distance cap. Direction UNMEASURED — not evidence of a rim.
        beyond_cap: Vec<VertexId>,
        /// Refused by the SI-alignment guard: in-cap, near-lateral. This is #701's rim.
        lateral: Vec<VertexId>,
        moved: Vec<VertexId>,
        backed_off: usize,
    }

    /// The corner population split, printed with the **decline attributed to its guard** — the
    /// line `midside_residuals` prints for midsides, for corners.
    ///
    /// `beyond_cap` and `lateral` partition the declined set, so their sum is the guard-declined
    /// total and no separate count is passed.
    fn report_conform_split(
        candidates: usize,
        authorised: usize,
        moved: usize,
        backed_off: usize,
        (beyond_cap, lateral): (usize, usize),
    ) {
        println!(
            "bonded-face boundary nodes: {candidates} candidates = {authorised} authorised \
             ({moved} moved, {backed_off} fully backed off by the quality floor) + {} \
             guard-declined ({beyond_cap} beyond the {} mm cap — direction UNMEASURED; \
             {lateral} near-lateral, i.e. #701's rim)",
            beyond_cap + lateral,
            cf_fsu_geometry::SI_CONFORM_CAP_BONDED,
        );
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
        // ONE decision per candidate, and the reason kept — rather than calling the discriminator
        // twice and re-deriving why it said no, which would measure a model of the production
        // decision instead of the decision.
        //
        // ★ The classification is an EXHAUSTIVE MATCH on purpose. The obvious alternative — three
        // predicate passes plus `assert_eq!(beyond_cap + lateral, guard_declined)` — is a gate
        // that CANNOT FAIL: `ConformDecision::declined()` is defined as exactly those two
        // variants, so the equality is a theorem about the enum rather than a check on the data.
        // A match makes a future variant a COMPILE error, which is the check that assert was
        // pretending to be.
        let (mut authorised, mut guard_declined) = (Vec::new(), Vec::new());
        let (mut beyond_cap, mut lateral) = (Vec::new(), Vec::new());
        for &v in &candidates {
            match cf_fsu_geometry::bonded_conform_decision(
                p_raw[v as usize],
                ep.superior_axis,
                ep.o4,
                ep.o5,
            ) {
                ConformDecision::AlreadySeated(_) | ConformDecision::Seated(_) => {
                    authorised.push(v);
                }
                ConformDecision::BeyondCap { .. } => {
                    beyond_cap.push(v);
                    guard_declined.push(v);
                }
                ConformDecision::Lateral { .. } => {
                    lateral.push(v);
                    guard_declined.push(v);
                }
            }
        }
        let authorised_set: std::collections::HashSet<VertexId> =
            authorised.iter().copied().collect();
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
            beyond_cap,
            lateral,
            moved,
            backed_off,
        }
    }

    /// `(max, RMS)` residual to the bone for each of the three CORNER populations, in both arms —
    /// the direct counterpart of [`MidsideResiduals`], one level down.
    ///
    /// The two existed asymmetrically until rung β: the midside gate had a struct and a helper
    /// while the corner gate computed its four pairs inline. Same shape now, including `declined`
    /// as an `Option` for the same reason (an empty population has no statistic, and
    /// `residual_stats` would hand back `NaN` for one).
    struct CornerResiduals {
        /// The authorised population, raw then conformed: the gate's payoff pair.
        authorised: ((f64, f64), (f64, f64)),
        /// Every candidate, raw then conformed.
        all: ((f64, f64), (f64, f64)),
        /// The guard-declined population, conformed arm — `None` when nothing was declined.
        declined: Option<(f64, f64)>,
    }

    fn corner_residuals(
        (candidates, authorised, guard_declined): (&[VertexId], &[VertexId], &[VertexId]),
        (p_raw, p_conf): (&[Point3<f64>], &[Point3<f64>]),
        (o4, o5): (&MeshOracle, &MeshOracle),
    ) -> CornerResiduals {
        let res = |nodes: &[VertexId], p: &[Point3<f64>]| -> (f64, f64) {
            let rs: Vec<f64> = nodes
                .iter()
                .map(|&v| endplate_residual(p[v as usize], o4, o5))
                .collect();
            residual_stats(&rs)
        };
        let out = CornerResiduals {
            authorised: (res(authorised, p_raw), res(authorised, p_conf)),
            all: (res(candidates, p_raw), res(candidates, p_conf)),
            declined: (!guard_declined.is_empty()).then(|| res(guard_declined, p_conf)),
        };
        let (
            ((au_max_raw, au_rms_raw), (au_max_conf, au_rms_conf)),
            ((all_max_raw, all_rms_raw), (all_max_conf, all_rms_conf)),
        ) = (out.authorised, out.all);
        let declined_cell = out.declined.map_or_else(
            || "none (no node was declined)".to_string(),
            |(m, r)| format!("max {m:.3} rms {r:.3}"),
        );
        println!(
            "residual |eval| to the nearer vertebra (mm) — \
             AUTHORISED: raw max {au_max_raw:.3} rms {au_rms_raw:.3} -> conformed max {au_max_conf:.3} rms {au_rms_conf:.3}; \
             ALL: raw max {all_max_raw:.3} rms {all_rms_raw:.3} -> conformed max {all_max_conf:.3} rms {all_rms_conf:.3}; \
             GUARD-DECLINED (left straight by design): {declined_cell}"
        );
        out
    }

    /// The corner population split, asserted: the exact tuple, the cap tripwire, and the
    /// rim-is-a-minority bound.
    ///
    /// Extracted so `conform_seats_the_bonded_face_on_the_bone_fom` stays readable — these three
    /// asserts carry ~50 lines of justification between them and none of it belongs inline with the
    /// residual statistics.
    fn assert_corner_population_split(
        candidates: &[VertexId],
        authorised: &[VertexId],
        moved: &[VertexId],
        backed_off: usize,
        guard_declined: &[VertexId],
        beyond_cap: &[VertexId],
        lateral: &[VertexId],
    ) {
        // (1) NON-VACUITY, committed exactly. A conform that declined everything would satisfy a
        // "no node got worse" gate trivially, and these counts pin the whole population split —
        // so a silent change in the mesh, the band rule, or the discriminator fails here rather
        // than diluting the statistics below.
        //
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): of 189 bonded-face boundary
        // nodes the discriminator authorises 188; 182 are delivered and **6 are backed off
        // entirely** by the quality floor. That 6 is the number this split exists to surface — it
        // is invisible to a max-move statistic, and it is the failure mode that grows if a future
        // mesh is worse conditioned.
        //
        // ⚠⚠ **RE-ANCHORED at rung β from `(583, 233, 230, 3, 350)`, and the change is a FINDING,
        // not a drift.** α.1's internal normalisation (#714) stopped the SDF filling space that is
        // not disc, and the declined population went **350 -> 1** while authorised went only
        // 233 -> 188: **99.7 % of declined vanished against 19 % of authorised.** The midside
        // gate shows the same signature independently (982 -> 0 of 1562). A population that is
        // annihilated by removing phantom material was not anatomy.
        //
        // ⇒ the pre-α.1 "overhanging annular rim" was overwhelmingly **phantom material**, and
        // #701's ring-apophysis modelling call — which is real — covers **one node**. See the
        // `beyond_cap` assert below for the instrument that settles it.
        //
        // ⚠ `backed_off` grew 3 -> 6. Read honestly: the *absolute* count rose while the
        // population shrank 3×, so the fraction backed off went 1.3 % -> 3.2 %. This is the
        // quality floor refusing proportionally more moves on a mesh whose remaining nodes sit
        // much closer to the bone (raw max 5.724 -> 1.921 mm), where a given detJ back-off costs
        // relatively more of a shorter move. Reported, not explained away — it is the one number
        // here that moved in the *unfavourable* direction.
        assert_eq!(
            (
                candidates.len(),
                authorised.len(),
                moved.len(),
                backed_off,
                guard_declined.len()
            ),
            (189, 188, 182, 6, 1),
            "the bonded-face population split changed"
        );

        // (1b) ★★ THE CAP IS A TRIPWIRE, NOT A SILENT ABSORBER — the assert this rung adds.
        //
        // `beyond_cap` counts nodes the conform refused because the nearer endplate is past
        // `SI_CONFORM_CAP_BONDED` (6 mm) — nodes whose direction was never even evaluated, since
        // the cap is tested first (`ConformDecision::BeyondCap`). On the shipped geometry it is
        // **ZERO**: every remaining decline is a genuine near-lateral one.
        //
        // ★ Pre-α.1 this would have read ~350, with a committed maximum residual of 12.577 mm —
        // **twice the cap** — and nothing reported it. A backstop that quietly swallows garbage is
        // worse than no backstop, because it manufactures a plausible-looking population.
        //
        // ⚠ **Be precise about what this adds, because assert (1) above fires FIRST.** Any change
        // that pushes nodes past the cap also moves `authorised`, so (1)'s exact tuple catches it
        // and (1b) never runs — verified by mutation (cap 6.0 -> 1.0 gives 166 authorised, so (1)
        // fires). What (1b) uniquely catches is a node moving between `lateral` and `beyond_cap`
        // with the tuple unchanged: same `guard_declined` count, different guard. So (1b) is the
        // DIAGNOSIS — it names the cap as the absorber — while (1) is the detector. Both earn their
        // place; neither is the other.
        //
        // ★ **It can fail for the reason it claims** (mutation-verified): with (1) neutralised and
        // the cap at 1.0, this assert fires reading `23 beyond the 1 mm cap`.
        assert_eq!(
            beyond_cap.len(),
            0,
            "{} bonded-face node(s) are further from the bone than the {} mm bonded cap. Their \
         direction is UNMEASURED (the cap is tested first), so they are NOT the annular rim — \
         either the disc/vertebra registration has drifted, or the mesher is again emitting \
         material that is not disc. Measure before re-anchoring this to a non-zero value.",
            beyond_cap.len(),
            cf_fsu_geometry::SI_CONFORM_CAP_BONDED,
        );

        // (1c) The rim is a MINORITY, stated as a proportion rather than pinned to 1.
        //
        // The exact count above is a knife-edge — one node flips on any nudge to
        // `SI_CONFORM_MIN_ALIGN` — so the robust claim is the one that actually distinguishes
        // clean geometry from phantom-material geometry: the lateral population is a rounding
        // error, not the ~60 % it used to be (350 of 583 corners, 982 of 1562 midsides).
        // → the range-not-anchor discipline: this survives a re-mesh, the exact count may not.
        assert!(
            lateral.len() * 20 < candidates.len(),
            "the near-lateral (rim) population is {} of {} candidates — over 5 %. Pre-α.1 it was \
         ~60 %, and that population turned out to be phantom material rather than the ring \
         apophysis. A rim reappearing at this scale means the mesher is filling space that is \
         not disc again.",
            lateral.len(),
            candidates.len(),
        );
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
    /// The discriminator declines nodes silently. By the #701 settled modeling call it is
    /// *supposed* to for one of its two arms: a node reached by a near-lateral move attaches to the
    /// ring apophysis, not the endplate face, so its residual is a modeling choice rather than a
    /// defect. So the strict-decrease assert is on the **authorised** nodes — the ones the
    /// discriminator said should seat — with the declined population reported alongside, and the
    /// whole candidate set *also* gated on RMS strictly down and max non-increasing (which is what
    /// "no node ended up further from the bone" means).
    ///
    /// ⚠⚠ **RUNG β CHANGED WHY THIS SPLIT EXISTS, and the original reason is retired.** It used to
    /// read: *"Averaging that rim into the payoff metric would let a population that never moves
    /// mask (or manufacture) a change in the seated one"* — a real risk when **350 of 583**
    /// candidates were declined. It is now **1 of 189**, so the aggregate can no longer be
    /// dominated by the rim and that argument no longer justifies anything.
    ///
    /// **The split stays anyway, for a different and better reason: the near-coincidence of the
    /// two columns is now itself the instrument.** `all` and `authorised` differ only through that
    /// one node, so a declined population reappearing separates them again — which is exactly the
    /// signature of phantom material returning (asserts 1b/1c make it loud rather than leaving it
    /// to be noticed). Deleting the split would buy nothing and would blind the gate to the
    /// regression α.1 fixed. **Weaker in one way, stronger in another, and worth saying which:** it
    /// no longer gates a rim-vs-seated distinction, because there is no rim; it now gates the
    /// *absence* of one.
    ///
    /// Authorised, deliberately, and **not** "nodes that moved": the moved set is selected on the
    /// outcome. The two are read independently — the discriminator on the raw positions says what
    /// it *intended*, the two node arrays say what the pipeline *delivered* — and their difference
    /// (6 nodes now, 2 when this was written) is a failure no "max move" number shows.
    ///
    /// **What that buys was measured on two mutants rather than argued** — and it is narrower
    /// than the tidy version of the argument. ⚠ **Both mutant runs below are PRE-α.1 figures**,
    /// measured on the old 583-candidate geometry; they are kept as the record of the experiment
    /// that chose this definition, not as claims about the current mesh. The *reasoning* is
    /// geometry-independent (it is about which set carries a dropped node's residual), so it
    /// survives the re-anchor; the numbers were not re-run, and are not asserted anywhere.
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
            beyond_cap,
            lateral,
            moved,
            backed_off,
        } = conform_split(&conf, &p_raw, &p_conf, ep);

        let CornerResiduals {
            authorised: ((au_max_raw, au_rms_raw), (au_max_conf, au_rms_conf)),
            all: ((all_max_raw, all_rms_raw), (all_max_conf, all_rms_conf)),
            declined: dec,
        } = corner_residuals(
            (&candidates, &authorised, &guard_declined),
            (&p_raw, &p_conf),
            (&o4, &o5),
        );
        report_conform_split(
            candidates.len(),
            authorised.len(),
            moved.len(),
            backed_off,
            (beyond_cap.len(), lateral.len()),
        );
        // Pinned at exactly one node by the split assert below, so the statistic exists — but read
        // through the same `Option` the midside side uses, rather than trusting that.
        let (dec_max, _dec_rms) = dec.expect("the guard-declined corner population is non-empty");

        assert_corner_population_split(
            &candidates,
            &authorised,
            &moved,
            backed_off,
            &guard_declined,
            &beyond_cap,
            &lateral,
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

        // (3b) ★ The claim in (3) — "the max is set by a guard-declined node" — MADE CHECKABLE.
        //
        // It was prose for two rungs and nothing tested it. It matters, because it is the entire
        // reason (3) asks only that the max not INCREASE rather than that it improve: a node the
        // discriminator never authorised cannot get closer, so requiring the all-candidate max to
        // fall would be requiring the conform to move a node it deliberately left alone.
        //
        // Now that the declined population is a single node, this is also what separates the ALL
        // column from the AUTHORISED one: 0.207 vs 0.170 mm RMS is that one node's 1.625 mm.
        assert!(
            (all_max_conf - dec_max).abs() < 1e-9,
            "the all-candidate max ({all_max_conf:.3} mm) must be the guard-declined node's own \
             residual ({dec_max:.3} mm) — that identity is why assert (3) only forbids the max \
             from rising. If an AUTHORISED node is now the worst, the conform left something it \
             intended to seat further out than the population it deliberately declined, and (3) \
             has stopped testing what its comment says."
        );

        // (4) COMMITTED VALUES, two-sided at ±5 % (the #701 / rung-1 shape), so a regression AND
        // a silent geometry change both fail rather than only the first.
        //
        // COMMITTED (as above), residual in mm:
        //   AUTHORISED       raw max 1.921  rms 0.530  ->  conformed max 1.054  rms 0.170
        //   ALL candidates   raw max 1.921  rms 0.541  ->  conformed max 1.625  rms 0.207
        //   GUARD-DECLINED   (unchanged by construction) max 1.625 — a SINGLE node, so its "rms"
        //                    is that same number and carries no distributional meaning.
        //
        // ⚠⚠ **RE-ANCHORED at rung β** from `AUTHORISED 5.724 / 1.332 -> 3.833 / 0.750` and
        // `ALL 12.577 / 3.494 -> 12.577 / 3.424`. Previously re-anchored once at rung 4 when
        // `DISC_CONFORM_QUALITY_FLOOR` went 0.05 -> 0.25.
        //
        // **The conform now cuts the seated population's RMS distance to the bone by 3.1×
        // (0.530 -> 0.170 mm) against 1.8× before, and the worst authorised node ends up 1.054 mm
        // out where it used to be 3.833.** That is a better result on every axis, and the reason is
        // the *input*, not the conform: the raw disc it starts from is already 3× closer to the
        // bone (raw max 5.724 -> 1.921 mm) because α.1 stopped the mesher emitting material that
        // is not disc. The conform is doing less work because there is less damage to correct.
        //
        // ★★ **The ALL and AUTHORISED columns have collapsed onto each other** — 188 of 189
        // candidates are authorised, so `all_rms_conf` (0.207) differs from `au_rms_conf` (0.170)
        // only through that one declined node at 1.625 mm. Pre-α.1 they were 3.424 vs 0.750,
        // because 350 of 583 nodes were the "rim". **That near-coincidence is now itself the
        // measurement** — assert (3b) below pins the one node that separates them.
        //
        // ⚠ **`SI_CONFORM_CAP_BONDED = 6.0` is now 3× above anything it can bind on** (whole
        // candidate max 1.921 mm), and its doc's "measured p90 ≈ 5.8 mm" is not merely
        // unproducible but falsified by this row. Deliberately NOT retuned here: a backstop is
        // sized against inputs you have not seen yet, and n=1 anatomy is no basis for tightening
        // it. What this rung does instead is make it LOUD — assert (1b).
        for (v, expect, name) in [
            (au_max_raw, 1.921, "authorised raw max"),
            (au_rms_raw, 0.530, "authorised raw rms"),
            (au_max_conf, 1.054, "authorised conformed max"),
            (au_rms_conf, 0.170, "authorised conformed rms"),
            (all_rms_raw, 0.541, "all raw rms"),
            (all_rms_conf, 0.207, "all conformed rms"),
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
        curved_disc_arms_at(mesh, params, ep, ConformFloors::SHIPPED)
    }

    /// [`curved_disc_arms`] at explicit quality floors — the seam the floor-selection gate
    /// sweeps to regenerate the midside column of both floor tables.
    fn curved_disc_arms_at(
        mesh: IndexedMesh,
        params: &DiscParams,
        ep: EndplateConform,
        floors: ConformFloors,
    ) -> CurvedArms {
        let prepared =
            prepare_disc_at(mesh, params, Some(ep), floors).expect("prepare the conformed disc");
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
            .with_projected_midsides(&moves, floors.midside);
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
        /// Refused by the distance cap. Direction UNMEASURED — not evidence of a rim. See
        /// [`ConformSplit`] for why the two declines are not interchangeable.
        beyond_cap: BTreeSet<VertexId>,
        /// Refused by the SI-alignment guard: in-cap, near-lateral. This is #701's rim.
        lateral: BTreeSet<VertexId>,
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
        // One decision per candidate, reason retained, classified by an exhaustive match — the
        // corner split's shape, one level up. See `conform_split` for why the match rather than a
        // count assert.
        let (mut authorised, mut declined) = (BTreeSet::new(), BTreeSet::new());
        let (mut beyond_cap, mut lateral) = (BTreeSet::new(), BTreeSet::new());
        for &v in &candidates {
            let native = center_native + straight.positions()[v as usize] / scale;
            match cf_fsu_geometry::bonded_conform_decision(native, ep.superior_axis, ep.o4, ep.o5) {
                ConformDecision::AlreadySeated(_) | ConformDecision::Seated(_) => {
                    authorised.insert(v);
                }
                ConformDecision::BeyondCap { .. } => {
                    beyond_cap.insert(v);
                    declined.insert(v);
                }
                ConformDecision::Lateral { .. } => {
                    lateral.insert(v);
                    declined.insert(v);
                }
            }
        }
        MidsideSplit {
            candidates,
            authorised,
            declined,
            beyond_cap,
            lateral,
        }
    }

    /// `(max, RMS)` residual to the bone for each of the three midside populations, in both
    /// arms — and the print that puts them in the log next to each other.
    struct MidsideResiduals {
        /// The authorised population, straight then curved: the gate's payoff pair.
        authorised: ((f64, f64), (f64, f64)),
        /// Every candidate, straight then curved (rim included — reported, not the payoff).
        all: ((f64, f64), (f64, f64)),
        /// The guard-declined population, curved arm (identical in both arms by construction) —
        /// `None` when nothing was declined, which is the case on the shipped geometry.
        ///
        /// ★ `Option`, not `(0.0, NaN)`. `residual_stats` divides by the count, so an empty
        /// population used to yield a max of 0.0 and an RMS of `NaN` — a row that prints as
        /// plausible and makes any `±5 %` comparison against it silently *false* instead of loud.
        /// The type now says "there is no such population" rather than inventing a statistic for
        /// one that does not exist.
        declined: Option<(f64, f64)>,
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
            declined: (!split.declined.is_empty()).then(|| res(&split.declined, curved)),
        };
        let (
            ((au_max_s, au_rms_s), (au_max_c, au_rms_c)),
            ((all_max_s, all_rms_s), (all_max_c, all_rms_c)),
        ) = (out.authorised, out.all);
        let declined_cell = out.declined.map_or_else(
            || "none (no midside was declined)".to_string(),
            |(m, r)| format!("max {m:.3} rms {r:.3}"),
        );
        println!(
            "bonded-face boundary midsides: {} candidates = {} authorised + {} guard-declined \
             ({} beyond the {} mm cap — direction UNMEASURED; {} near-lateral, i.e. #701's rim)",
            split.candidates.len(),
            split.authorised.len(),
            split.declined.len(),
            split.beyond_cap.len(),
            cf_fsu_geometry::SI_CONFORM_CAP_BONDED,
            split.lateral.len(),
        );
        println!(
            "residual |eval| to the nearer vertebra (mm) — \
             AUTHORISED: straight max {au_max_s:.3} rms {au_rms_s:.3} -> curved max {au_max_c:.3} rms {au_rms_c:.3}; \
             ALL: straight max {all_max_s:.3} rms {all_rms_s:.3} -> curved max {all_max_c:.3} rms {all_rms_c:.3}; \
             GUARD-DECLINED (left straight by design): {declined_cell}"
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
    /// number was dominated by a population that stays straight, so quoting a drop in it would be
    /// measuring how much of that population happens to be in the average. So the payoff assert is
    /// on the **authorised** midsides — the ones the discriminator says should seat — exactly as
    /// `conform_seats_the_bonded_face_on_the_bone_fom` does for corners.
    ///
    /// ⚠⚠ **RUNG β RETIRED THE PREMISE, exactly as it did on the corner gate.** This paragraph used
    /// to attribute the dominating population to "the overhanging annular rim, which stays straight
    /// by design (#701's settled call — Sharpey's fibres attach to the ring apophysis)". Measured:
    /// that population was **phantom material**, and there are now **445 candidates of which 445
    /// are authorised and 0 declined**. So the aggregate cannot be dominated by anything, and the
    /// self-deception this section warns about is no longer available.
    ///
    /// **The like-for-like structure stays, for the reason the corner gate gives:** with the two
    /// populations identical, their coincidence is the instrument — a declined midside reappearing
    /// separates them again and is the signature of phantom material returning. Assert (4b) pins it
    /// as a set equality. The declined population is reported, never averaged in, and the all-candidate
    /// aggregate is kept as the continuity check against rung 2's committed number.
    ///
    /// ## The validity gate is written so it cannot be tautological (§4.4)
    ///
    /// v1's "0 inverted / 0 sliver" could not fail: the back-off makes the straight position
    /// always feasible, so non-inversion is a *construction guarantee*. Its two replacements are
    /// both falsifiable — **coverage** (`projection_coverage`: what fraction reached its FULL
    /// projection, since both projection helpers back off silently) and **element validity over
    /// every element** (`worst_rest_det_ratio`, whose doc comment states precisely what it
    /// catches that the projector's own guarantee does not).
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
    // Five helpers are already extracted above (`curved_disc_arms`, `midside_split`,
    // `midside_residuals`, `projection_coverage`, `worst_rest_det_ratio`); what is left is one
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
        // The declined population is empty on the shipped geometry (assert (4b)), so there is no
        // declined statistic to read — its committed `rms 4.200 mm` pin is retired, not re-anchored.
        assert!(
            residuals.declined.is_none(),
            "a declined midside statistic exists ({:?}), so the population is no longer empty — \
             see assert (4b)",
            residuals.declined
        );

        // (3) COVERAGE + ELEMENT VALIDITY (§4.4).
        let (delivered, max_move, mean_move) =
            projection_coverage(&straight, &curved, &moves, params.scale);
        let worst_det = worst_rest_det_ratio(&curved, &straight);
        println!(
            "projection: {:.1} % delivered in full, max move {max_move:.3} mm, mean {mean_move:.3} mm; \
             worst detJ/detJ_rest over every element, Gauss point and reference corner \
             {worst_det:.4}",
            100.0 * delivered,
        );

        // (4) NON-VACUITY, committed exactly — the population split, so a silent change in the
        // mesh, the band rule, the boundary-face selection or the discriminator fails here
        // rather than diluting the statistics below.
        //
        // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default): all 445 bonded-face boundary
        // midsides are authorised and **none is declined**.
        //
        // ⚠⚠ **RE-ANCHORED at rung β from `(1562, 580, 982)`.** The declined population went
        // **982 -> 0** while authorised went 580 -> 445: the declines were annihilated (100 %)
        // while the seated population fell only 23 %. Rung 2's corner gate shows the same
        // signature independently (350 -> 1 declined against 233 -> 188 authorised). ⇒ the
        // "overhanging annular rim" this table used to attribute 982 midsides to was
        // **phantom material** that α.1 (#714) stopped the mesher emitting, not the ring
        // apophysis. The corner gate's asserts (1b)/(1c) carry the instrument and the argument.
        assert_eq!(
            (
                split.candidates.len(),
                split.authorised.len(),
                split.declined.len()
            ),
            (445, 445, 0),
            "the bonded-face boundary midside population split changed",
        );

        // (4b) ★★ THE COINCIDENCE IS THE INSTRUMENT. With nothing declined, the ALL and AUTHORISED
        // populations are the *same set* — so this gate's three-population design has collapsed to
        // two, and the collapse is the measurement rather than a reason to delete the structure.
        //
        // Asserted as a SET EQUALITY, not by comparing their statistics: two numbers that agree
        // would also agree if the populations differed and the extra members happened not to move
        // the aggregate. This fires the moment a declined midside reappears, which is the
        // signature of the mesher filling space that is not disc again.
        assert_eq!(
            split.candidates, split.authorised,
            "every bonded-face boundary midside must be authorised on the shipped geometry — the \
             ALL and AUTHORISED statistics below are the same population, and the committed values \
             assume it. A declined midside appearing here means either the registration drifted or \
             phantom material is back; re-measure rather than re-anchoring."
        );

        // (5) THE PAYOFF: the midsides the discriminator intended to seat end up strictly closer
        // to the bone, on both the extreme and the population statistic.
        assert!(
            au_max_c < au_max_s && au_rms_c < au_rms_s,
            "the authorised midsides must end up closer to the bone \
             (max {au_max_s:.3} -> {au_max_c:.3}, rms {au_rms_s:.3} -> {au_rms_c:.3} mm)",
        );
        // The whole candidate population improves, and no midside is pushed further off the bone.
        //
        // ⚠ **These two asserts are currently IDENTICAL to the authorised pair above**, because
        // assert (4b) establishes the populations are the same set. Kept deliberately, not deleted:
        // they cost nothing, and the moment a declined midside reappears they become the distinct
        // check they were written to be. The max is only required not to INCREASE rather than to
        // improve — the original reason was that it was set by a guard-declined midside the conform
        // never intended to move, which no longer applies here but is still the correct contract for
        // a population that may contain declined nodes again.
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
        // COMMITTED (as above), residual in mm — ALL == AUTHORISED, per assert (4b):
        //   AUTHORISED       straight max 1.375  rms 0.197  ->  curved max 1.045  rms 0.119
        //   ALL candidates   identical to AUTHORISED (the populations are the same set)
        //   GUARD-DECLINED   EMPTY — see the retired pin below
        //   COVERAGE         83.1 % delivered in full; max move 0.740, mean 0.076 mm
        //   VALIDITY         worst detJ/detJ_rest 0.4000 over every element and all EIGHT
        //                    sample points (four Gauss points + four reference corners)
        //
        // ⚠⚠ **RE-ANCHORED for the reference-corner back-off.** `with_projected_midsides` now
        // measures feasibility at the four reference corners as well as the four Gauss points,
        // because the conformed disc it accepted under the Gauss-only rule contained 18
        // elements folded at a corner —
        // `conformed_disc_reference_corner_rest_census_fom` carries that measurement and is now
        // the regression gate for it. Four numbers here move, all in the direction a *stricter*
        // acceptance test must move them, which is the check that this re-anchor is the change
        // and not a drift:
        //
        //   curved max   0.923 -> 1.045 mm      curved rms  0.111 -> 0.119 mm
        //   coverage     91.0  -> 83.1  %       max move    0.818 -> 0.740 mm
        //
        // ★ **The straight column did not move** (max 1.375, rms 0.197), which localises the
        // change to the projection rather than to the mesh, the band rule or the selection —
        // the same structure rung β's re-anchor used, and the reason those two are still pinned
        // here.
        //
        // ★★ **The payoff survives the stricter rule, and that was the kill/confirm.** A
        // corner constraint tight enough to unfold the mesh but tight enough to also stop the
        // midsides moving would have traded a real fold for a useless projection; the seated
        // RMS lands at 0.119 mm against the straight 0.197 mm, still a 40 % improvement and
        // still below rung 2's corners at 0.170 mm. So the bonded face is *still* seated more
        // closely than the corners it spans, which is the claim rung 3 exists to make.
        //
        // ⚠ `k_disc` is NOT re-anchored, and that is measured rather than assumed:
        // `conform_delta_by_element_fom` reproduces all eight committed absolutes to four
        // decimals across this change. The 18 folded elements did not measurably move the
        // physics — 12 were fully pinned by the bond and the 6 that carried free DOFs sit at
        // the endplate periphery.
        //
        // ⚠⚠ **RE-ANCHORED at rung β** from `AUTHORISED 3.973 / 0.881 -> 3.971 / 0.767`,
        // `ALL 12.464 / 3.375 -> 12.464 / 3.357`, coverage 67.4 %. Previously re-anchored at
        // rung 4 with the corner floor (0.05 -> 0.25), since midsides start from conformed corners.
        //
        // ★★ **A RETIRED PIN, not a re-anchored one: `guard-declined rms 4.200 mm` is GONE.**
        // The declined population is empty, and the RMS of an empty set is not a quantity — it
        // evaluated to `NaN`, which made the ±5 % comparison silently *false* rather than loud
        // (see `residual_stats`' hazard note). It could not honestly be given a new value, so its
        // job passes to the population asserts (4)/(4b), which pin the *count* and the set identity
        // instead of a statistic over nothing.
        //
        // **The payoff, and it is much stronger than the figures it replaces.** The seated
        // population's RMS distance to the bone falls **0.197 -> 0.119 mm** and the worst midside
        // ends up **1.045 mm** out, where this table used to record 0.881 -> 0.767 with a 3.971 mm
        // extreme. Rung 2's corners now sit at 0.170 mm RMS, so the midsides land *closer to the
        // bone than the corners they span* (0.119 vs 0.170) — the bonded face is uniformly seated,
        // which is what this rung exists to deliver.
        //
        // ★ **The max finally moves.** It used to barely budge (3.973 -> 3.971) because the worst
        // authorised midside was one the quality floor refused; now it drops 24 % (1.375 -> 1.045)
        // and coverage rose 67.4 % -> **83.1 %**. Both have the same cause: with the phantom
        // material gone, a midside is asked to move 0.740 mm rather than 1.533 mm across a 3 mm
        // cell, so the floor refuses far less often. **The projection did not improve — its input
        // did.**
        //
        // ⚠ The ALL-candidate straight figures no longer reproduce rung 2's committed baseline
        // (1562 midsides, max 12.464 / RMS 3.375 from `straight_tet10_midsides_chord_across_the_
        // endplate_fom`), because that baseline was measured over a population 3.5× larger that was
        // mostly phantom. The like-for-like continuity check this table used to carry is therefore
        // NOT available across α.1, and is not faked here.
        assert_within_5_percent(&[
            (au_max_s, 1.375, "authorised straight max (mm)"),
            (au_rms_s, 0.197, "authorised straight rms (mm)"),
            (au_max_c, 1.045, "authorised curved max (mm)"),
            (au_rms_c, 0.119, "authorised curved rms (mm)"),
            (all_rms_s, 0.197, "all straight rms (mm)"),
            (all_rms_c, 0.119, "all curved rms (mm)"),
            (all_max_s, 1.375, "all straight max (mm)"),
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
            "an element fell below the quality floor at some sample point — a Gauss point or a \
             reference corner (worst detJ/detJ_rest {worst_det:.4}) — fix the projection, not \
             this gate",
        );
        assert!(
            worst_det < 1.0,
            "no element's rest Jacobian shrank at all (worst ratio {worst_det:.4}) — the \
             projection cannot have engaged, so nothing above is measuring it",
        );

        // (8) §4.4 COVERAGE — the falsifiable half of the validity gate, two-sided.
        //
        // **16.9 % of the authorised midsides do NOT reach their full projection**, and that is
        // the number this statistic exists to make impossible to hide: both projection helpers
        // back off silently, so a "did anything move / how far did the furthest node move" gate
        // would have reported this run as a clean success. The non-delivery is real geometry,
        // not a bug — a midside asked to move 0.740 mm across a 3 mm cell is asked to fold its
        // element, and the quality floor refuses. Compare the corner conform, where 182 of 188
        // authorised nodes were delivered (96.8 %): corner moves are sparser and a Tet4 element's
        // Jacobian is affine, so the floor almost never binds there.
        //
        // ⚠⚠ **RE-ANCHORED at rung β from 67.4 % / max move 1.533 / mean 0.141**, then again for
        // the reference-corner back-off, from **91.0 % / 0.818 / 0.086** to today's
        // **83.1 % / 0.740 / 0.076**. The two moves have opposite causes and it is worth keeping
        // them apart. The rise to 91.0 % was α.1 (#714) stopping the mesher emitting material
        // that is not disc, so the straight midsides start 7× closer to the bone
        // (RMS 3.375 -> 0.197 mm) and a much smaller move is requested — **the projector did not
        // improve, its input did**. The fall to 83.1 % is this rung's own doing: the acceptance
        // test now also samples the reference corners, so the floor refuses more often, on
        // exactly the moves that were folding a corner while every Gauss point looked healthy.
        // A coverage number falling because the guard got honest is not a regression.
        //
        // ★ Note the direction: coverage RISING is the benign direction for this statistic (see the
        // mutant below, where a silent cap also raises it). What makes the rise trustworthy here is
        // that `max_move` and `mean_move` FELL together with the residual pin tightening — a silent
        // cap would have raised coverage while the residuals got worse.
        //
        // ⚠ **A selection variant was measured and REFUTED here, not argued.** The natural
        // hypothesis for the low delivery is that it is concentrated in midsides whose parent
        // corners were themselves guard-declined (a midside pulled onto the bone while its two
        // rim parents stay put has to bow hard). Gating the selection on both parents being
        // authorised gives 484 of 580 midsides at **79.5 %** delivered — barely better — while
        // dropping 96 midsides that were *improving* (their RMS falls 1.477 → 1.198). So the
        // simpler rule ships: project every authorised bonded-face boundary midside.
        // ⚠ Both halves of that comparison were measured at the *corner* floor 0.05 and pre-α.1,
        // before this rung split the constant (72.4 % ungated vs 79.5 % parent-gated), so read it
        // as the like-for-like pair it is and not against the 91.0 % committed above. ⚠⚠ Its
        // premise is also gone: the hypothesis under test was that low delivery concentrates in
        // midsides whose parents were guard-declined, and there are now **no declined parents** —
        // so the parent-gated variant would today select the identical population. The comparison
        // is kept as the record of why the simpler rule was chosen, not as a live alternative.
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
            (delivered, 0.831, "delivered fraction"),
            (max_move, 0.740, "max midside move (mm)"),
            (mean_move, 0.076, "mean midside move (mm)"),
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
