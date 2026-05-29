//! Plug-floor lock — truncated-pyramid press-fit retention.
//!
//! Step 9 of `docs/CURVE_FOLLOWING_DESIGN.md` originally shipped an
//! axial cylindrical plug-shaft + perpendicular T-bar + cup-side
//! T-slot captive-insertion mechanism (pre-S4 of the FDM-friendly
//! geometry arc). Workshop iter-2 printed cleanly on Bambu Lab
//! calibrated FDM but the consumer-FDM target floor (Bambu A1 +
//! default + Jayo per the FDM-friendly geometry recon §G-3)
//! struggles with the cylindrical-shaft cup-wall penetration: the
//! shaft-through-cup-wall through-hole relies on tight Ø-vs-Ø fit
//! for the silicone seal, and on consumer FDM the tolerance gap
//! becomes a leak path (silicone wicks past the shaft and out the
//! cap-plane outer face).
//!
//! Recon-1 §G-1 (2026-05-24) replaced the entire mechanism with a
//! single **truncated-pyramid plug-floor lock**: the plug's
//! cap-plane face carries a pyramid protruding AWAY from the body
//! (along `cap_normal` = downward in pour orientation), and the
//! cup-piece cap-plane interior surface carries a matching socket
//! recessed INTO the cup material (cavity, NOT a through-hole — no
//! cup-wall penetration, no leak path). Workshop motion = mold-
//! assembly press-fit: lower the plug into the open cup half (seam
//! plane up) so the pyramid seats into the cup-piece floor socket;
//! close the second cup half over it. Press-stop tactile feedback
//! = the seam halves bottom out flush against each other when the
//! pyramid is fully seated.
//!
//! S4 of the FDM-friendly geometry implementation arc (2026-05-24)
//! migrates this module from the post-MC mesh-CSG cylinder ops
//! (`MatingTransform::UnionCylinder` / `SubtractCylinder` consuming
//! [`crate::mesh_csg::CylinderParent`] for the shaft + T-bar) to
//! SDF-side [`crate::PrismaticPin`][`crate::prismatic_pin`] composition
//! pre-MC per the §G-12 #2 architectural correction (mirrors the S3
//! cup-pin migration; same paradigm-boundary placement). The
//! mesh-CSG `MatingTransform` variants stay for the cup pour-gate
//! carve (S7 of the prior mating-features arc); plug-side emission
//! no longer produces any mesh-CSG ops.
//!
//! # Paradigm-boundary placement
//!
//! The S1 probe-spike (`design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs`,
//! commit `a218ddfb`) characterised mesh-CSG `Manifold::union` of a
//! `Manifold::hull_pts` truncated-pyramid primitive against an
//! SDF→MC curved-shell host. Outcome: 2 disconnected components +
//! `SelfIntersecting` F4 Critical at the boolean junction (§G-7
//! BRANCH B + C BOTH fire). The §G-12 #2 SDF-side bail-out (pin
//! composed pre-MC into the host SDF via [`Solid::union`]) PASSES
//! BRANCH-A (1 component, no new F4 Critical types) on the same
//! fixture. The S3 cup-pin migration cleared the §G-11 #1 §R1
//! inspector at 1 component per cup-piece STL at production scale;
//! S4 inherits that paradigm-boundary placement (bulk-welded
//! geometry → SDF/MC; see
//! [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).
//!
//! # 3-piece shared-primitive invariant (analog of S6)
//!
//! Per §G-5 the plug-floor lock socket is **seam-plane-symmetric**:
//! the pyramid axis is along `cap_normal` (in-seam-plane direction,
//! perpendicular to the ribbon binormal), so the seam plane bisects
//! the pyramid laterally through its center. Each cup half carves
//! one half of the socket; the plug-side pyramid is one solid
//! spanning both halves. The same [`PrismaticPinSpec`] +
//! [`PrismaticPinPose`] triple flows through three call sites:
//! plug self-emission (`build_plug_lock_sdf`) unions the full
//! pyramid into the plug body; cup self-emission
//! (`build_plug_lock_socket_sdf`) returns one socket Solid that
//! BOTH cup halves' [`crate::piece::compose_piece_solid`]
//! invocations subtract from their half-shell pre-MC (side-agnostic
//! single solid — the per-side halfspace intersect bisects it by
//! construction, mirroring the recon-4 (P) seam architecture). This
//! is the SDF analog of the pre-S4 mesh-CSG 3-piece bit-equal
//! determinism contract; here it lives at the SDF input layer
//! rather than the mesh-vertex layer (same shape, different stage).
//!
//! # Pose convention (symmetric across cap-plane)
//!
//! - `center_m` = cap-plane centroid (passed via
//!   [`crate::ribbon::Ribbon::pour_end_hint`]
//!   `(centroid, cap_normal)`; cf-cast-cli's derive path threads
//!   the cap-plane data from `.prep.toml [caps]`).
//! - `axis_unit` = `+cap_normal` (= downward in pour orientation;
//!   = away from plug body's cap-plane face). The pyramid extends
//!   `±half_length_m` along this axis symmetrically across the
//!   cap-plane (mirrors S3 cup-pin's symmetric-across-seam pose):
//!   the `-axis_unit` half lives INSIDE the plug body / no-op
//!   inside the body cavity on the cup side; the `+axis_unit` half
//!   protrudes outward from the plug / carves cup-wall material on
//!   the cup side. Because both pyramid and socket share the same
//!   pose, the cross-section at the cap-plane (axis = 0, the main
//!   taper's widest exposed section) matches by construction — the
//!   workshop fit is bit-precise modulo the symmetric `/2` extent
//!   inflation on the socket side.
//! - `lateral_unit` = `+split_normal`. Orthogonal to `cap_normal`
//!   by ribbon construction (cap-plane lies on the centerline; the
//!   split-normal is perpendicular to the tangent at the cap-plane;
//!   any reasonable cap-plane sits with normal along the centerline
//!   tangent at the cap). [`PrismaticPinPose`] requires a
//!   deterministic lateral axis even for square-base pins.
//!
//! Per §G-4 the plug print orientation is **deferred to S6
//! procedure.rs** with the constraint that cap-plane-face-down is
//! invalid (the pyramid would protrude into the bed → unprintable);
//! preferred orientation is dome-end-on-bed (cap-plane face up,
//! pyramid pointing up). Under that preferred orientation the
//! pyramid base + chamfer band sit at the TOP of the print (no
//! first-layer elephant foot concern) — the chamfer remains in
//! [`PrismaticPinSpec::plug_lock_default`] as a lead-in self-
//! centering aid, not an FDM-driven chamfer.
//!
//! # Cap-plane anchor (post-S4 unchanged from pre-S4)
//!
//! The pin/lock anchor stays at the **cap-plane centroid**, NOT at
//! the trimmed centerline tip. cf-scan-prep's `trim_floor_mm`
//! (typically 40 mm) leaves `centerline.last()` 40 mm INSIDE the
//! plug body's bulk; the cap plane is where the plug body's cap-
//! plane face sits, so a lock anchored there straddles the plug
//! surface and the cup-piece cap-plane interior correctly. Callers
//! supply the cap-plane centroid + normal via
//! [`crate::ribbon::Ribbon::with_pour_end_hint`]; callers without
//! cap-plane data fall back to `(points.last(), last_segment.tangent)`
//! per cf-scan-prep's tip→base convention (workshop iter-1 fixtures
//! always pass an explicit hint).
//!
//! # Workshop-iter changelog
//!
//! - **Pre-S4** (post-S6 mating-features arc + recon-4 (P) seam +
//!   plug-shaft overlap-bias): plug emits cylindrical shaft + T-bar
//!   via two `MatingTransform::UnionCylinder` ops; cup carves shaft
//!   socket + T-slot via two `MatingTransform::SubtractCylinder`
//!   ops; `platform.stl` provides a blind pocket for the T-bar
//!   protrusion. `PlugPinSpec` carries 10 fields (pin + T-bar +
//!   shaft + dome variants).
//! - **Post-S4**: plug emits one [`PrismaticPin`][`crate::prismatic_pin`]
//!   pyramid SDF unioned into the plug body pre-MC; cup carves one
//!   matching socket SDF subtracted from each cup-piece half-shell
//!   pre-MC; `platform.stl` retires to a flat support slab (no
//!   pocket — no cup-wall penetration → no T-bar protrusion).
//!   [`PlugPinSpec`] wraps a single [`PrismaticPinSpec`].
//!
//! # Default off
//!
//! [`Ribbon::new`] sets `plug_pins = PlugPinKind::None` for
//! backward compatibility — Steps 5-10 of the v2 arc pre-date this
//! mechanism. Workshop iter-1 callers opt into
//! [`PlugPinKind::Axial`] via [`Ribbon::with_plug_pins`].
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new
//! [`Ribbon::with_plug_pins`]: crate::ribbon::Ribbon::with_plug_pins
//! [`CastSpec`]: crate::CastSpec
//! [`Solid::union`]: cf_design::Solid::union

use cf_design::Solid;
use nalgebra::{Point3, UnitVector3, Vector3};

use crate::mesh_csg::MatingTransform;
use crate::prismatic_pin::{PrismaticPinPose, PrismaticPinSpec};
use crate::ribbon::Ribbon;

/// Inward offset bias (meters) applied to
/// [`build_plug_cap_trim_transform`]'s emitted plane to keep the
/// resulting trim face from coinciding exactly with the plug
/// body's SDF cap-plane surface.
///
/// **Paradigm-boundary rationale** (recon §Q-4 + the
/// `project-cf-cast-sdf-meshcsg-paradigm-boundary` framework): a
/// face-coincident `MatingTransform::SeamTrim` at the body's SDF
/// cap-plane triggers the recon-4 (P) §F-2 WELDED-TO-BULK failure
/// mode (the 2026-05-24 disabling of this builder in
/// [`add_plug_pins`] was caused by precisely that mode — F4
/// flagged a Critical issue on the trimmed plug). Shifting the
/// plane 1 µm INWARD (into the kept plug-body half-space) makes
/// the trim face CONTAINED inside the body's pre-trim material →
/// no exact-coincidence → paradigm-safe per the
/// [recon-4 §F-2 framework](crate::mesh_csg).
///
/// Magnitude rationale (1 µm = `1e-6` m):
/// - Below FDM print resolution (Bambu A1 ≈ 200 µm) by 200× —
///   workshop-invisible at the slicer-quantization layer.
/// - Above mesh-CSG numerical noise (manifold3d f64 internal
///   precision ≈ 1 nm) by 6+ orders of magnitude — empirically
///   safe from manifold3d ambiguity.
/// - Same magnitude as the §G-7 plug-shaft `extend_near_end`
///   inward overlap-bias (same paradigm-boundary pattern).
pub const PLUG_CAP_TRIM_BIAS_M: f64 = 1.0e-6;

/// Plug-floor-lock geometry spec — wraps the SDF-side
/// [`PrismaticPinSpec`] primitive with no extra fields.
///
/// Post-S4 of the FDM-friendly geometry arc, the lock is a single
/// truncated-pyramid press-fit feature at the plug body's
/// cap-plane face (§G-1). The wrapper previously paralleled
/// `crate::PinSpec`'s shape (`PinSpec` retired in §M-S4); it is
/// retained as a thin wrapper around [`PrismaticPinSpec`] plus
/// per-feature placement so future iters can extend it without
/// breaking the [`crate::PlugPinKind::Axial`] surface. The plug
/// lock has a single fixed placement (cap-plane centroid, see
/// module docstring §"Pose convention"), so the wrapper carries no
/// additional fields today. Future iters may add a
/// `dome_lock: Option<PrismaticPinSpec>` for the high-curvature
/// dome-end retention case (§G-4 alternative orientation), kept
/// out of scope for iter-3 default path.
#[derive(Debug, Clone, PartialEq)]
pub struct PlugPinSpec {
    /// Per-lock SDF-side primitive spec. Default
    /// [`PrismaticPinSpec::plug_lock_default`] per recon-1 §G-6 /
    /// §G-8: 4 mm half-extent square base, 2.8 mm half-extent
    /// square tip (taper ratio 0.7), 4 mm half-length, 0.8 mm
    /// chamfer (optional under preferred dome-down plug print
    /// orientation; kept as a lead-in self-centering aid), 0.35 mm
    /// diametral clearance, 0.50 mm axial clearance.
    pub lock_spec: PrismaticPinSpec,
}

impl PlugPinSpec {
    /// v2 iter-3 defaults (post-S4): plug-floor-lock geometry per
    /// [`PrismaticPinSpec::plug_lock_default`]. Numeric values
    /// (clearances + chamfer) pinned at S7 workshop-physical
    /// calibration on Bambu A1 + default + Jayo per §G-8.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            lock_spec: PrismaticPinSpec::plug_lock_default(),
        }
    }
}

impl Default for PlugPinSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// What plug-floor-lock geometry, if any, the [`Ribbon`] uses.
///
/// `None` is the v2 (pre-v2.1) default — no plug retention; the
/// workshop user hand-positions the plug during pour + cure.
/// `Axial(PlugPinSpec)` enables the §G-1 truncated-pyramid press-fit
/// lock at the cap-plane end. The variant name `Axial` is preserved
/// from the pre-S4 cylindrical-shaft mechanism for cf-cast-cli
/// surface compatibility; the payload shape changed entirely.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum PlugPinKind {
    /// No plug-floor lock.
    #[default]
    None,
    /// Truncated-pyramid plug-floor lock at the cap-plane end, per
    /// [`PlugPinSpec`].
    Axial(PlugPinSpec),
}

/// Resolve the plug-floor-lock [`PrismaticPinPose`] for this ribbon,
/// or `None` when the ribbon has no plug pins / no resolvable cap-
/// plane anchor.
///
/// Returned pose: `center_m` at the cap-plane centroid;
/// `axis_unit = -cap_normal` (= INTO plug body, upward in pour
/// orientation); `lateral_unit = +split_normal` projected
/// orthogonal to `axis_unit` (see module docstring §"Pose
/// convention").
///
/// # Axis direction (base-down captive convention, 2026-05-24 polish)
///
/// `axis_unit = -cap_normal` orients the pyramid **BASE DOWN** in
/// pour orientation: the `-axis_unit` half of the symmetric-across-
/// cap-plane pose lives along `+cap_normal` (= DOWN, workshop-
/// visible) and carries the pyramid's BASE end (full base half-
/// extents + chamfer band), while the `+axis_unit` half lives along
/// `-cap_normal` (= UP, inside plug body) and carries the TIP end
/// (smaller tip half-extents). The cup-piece socket inherits the
/// same flip — its inflated cavity opens at a NARROW mouth at the
/// cap-plane and widens DOWN into the cup-wall material. Workshop
/// assembly: insert plug downward into the open cup half; the
/// pyramid's narrow top threads through the socket mouth and the
/// wider base settles into the deeper socket cavity. Close the
/// second cup half; the cup halves trap the pyramid's wider base
/// against vertical pull-out, making the plug captive in 6 `DoF`
/// during pour + cure. Pre-2026-05-24-polish convention was
/// `axis_unit = +cap_normal` (BASE UP, TIP DOWN) — workshop user
/// flagged that the plug pulled straight back up out of the socket
/// (no vertical lock), so the convention flipped.
///
/// Returns `None` for [`PlugPinKind::None`], an empty ribbon, or
/// any of the degenerate fallbacks per [`pour_anchor`].
fn build_plug_lock_pose(ribbon: &Ribbon) -> Option<(&PlugPinSpec, PrismaticPinPose)> {
    let spec = match &ribbon.plug_pins {
        PlugPinKind::None => return None,
        PlugPinKind::Axial(spec) => spec,
    };
    let (center, outward) = pour_anchor(ribbon)?;
    // `outward` arrives as a tangent / cap-normal vector;
    // renormalize defensively. Ribbon construction guarantees unit
    // magnitude for typical inputs; tiny FP drift is absorbed.
    // **Flip to -outward** so the pyramid orients BASE-DOWN (per
    // the axis direction note above) — gives the captive vertical
    // lock the workshop user requested 2026-05-24.
    let axis_unit = UnitVector3::new_normalize(-outward);
    // Gram-Schmidt `split_normal` against `axis_unit` to enforce the
    // [`PrismaticPinPose::new`] orthogonality contract. The cf-scan-
    // prep cap-plane normal is fitted to point cloud data (PCA on
    // the cap-plane vertices) while the ribbon's `split_normal` is
    // a user-axis projection, so production casts often see
    // ~1-3° between them. The pose's rotation about `axis_unit`
    // is geometrically immaterial for square-base pins (the
    // [`PrismaticPinSpec::plug_lock_default`] is square — base.x ==
    // base.y, tip.x == tip.y), so projecting `split_normal` onto
    // the plane perpendicular to `axis_unit` and renormalizing
    // preserves the workshop-meaningful pose while satisfying the
    // strict orthogonality assertion. If the input vectors are
    // exactly parallel (|dot| ≈ 1), the projection collapses to
    // zero — `new_normalize` would emit a degenerate unit-Z
    // fallback; gate that as `None` so the caller drops the lock
    // rather than emitting a skewed pose.
    // Orient the square lock's lateral axis to the SEAM normal so one pair of
    // its faces is parallel to the seam plane → the seam bisects the pin
    // symmetrically into two halves ("rotationally square to the two halves").
    // With a FITTED (apex-anchored) seam the seam normal is diagonal, NOT
    // `split_normal`, so deriving the lateral axis from `split_normal` (as the
    // legacy path does) leaves the square skewed ~33° to the seam (workshop
    // 2026-05-29). Use the fitted seam normal when present; else keep
    // `split_normal` so binormal/curve-following casts stay byte-identical.
    let ref_vec = match ribbon.seam_plane_basis() {
        Some(_) => ribbon.seam_plane_reference().1.into_inner(),
        None => ribbon.split_normal.as_vector(),
    };
    let axis_v = axis_unit.into_inner();
    let projected = ref_vec - axis_v * ref_vec.dot(&axis_v);
    if projected.norm_squared() < 1.0e-9 {
        return None;
    }
    let lateral_unit = UnitVector3::new_normalize(projected);
    Some((spec, PrismaticPinPose::new(center, axis_unit, lateral_unit)))
}

/// Resolve the pour-end anchor `(center, outward)` for this
/// ribbon's plug-floor lock.
///
/// - `pour_end_hint = Some((centroid, outward))` → returns the hint
///   verbatim. This is the production path: cf-cast-cli threads the
///   cap-plane centroid + normal from `.prep.toml [caps]` to
///   [`crate::ribbon::Ribbon::with_pour_end_hint`]. cf-scan-prep's
///   `trim_floor_mm` (typically 40 mm) leaves `centerline.last()`
///   40 mm INSIDE the plug body's bulk, so anchoring on the trimmed
///   tip would bury the lock; the cap-plane centroid is where the
///   plug's cap-plane face sits and the lock must anchor there.
/// - `pour_end_hint = None` → fall back to
///   `(last.end, last.tangent)` per cf-scan-prep's tip→base
///   centerline-orientation convention. Workshop iter-1 fixtures
///   always pass an explicit hint; the fallback is defensive for
///   hand-built ribbons in tests.
fn pour_anchor(ribbon: &Ribbon) -> Option<(Point3<f64>, Vector3<f64>)> {
    if let Some((centroid, outward)) = ribbon.pour_end_hint {
        return Some((centroid, outward));
    }
    let last = ribbon.segments.last()?;
    Some((last.end, last.tangent))
}

/// Build the plug-side plug-floor-lock mesh-CSG transform for this
/// ribbon, or `None` when [`PlugPinKind::None`] or the cap-plane
/// anchor is unresolvable.
///
/// The returned [`MatingTransform::UnionTruncatedPyramid`] applies
/// POST-MC via [`crate::apply_mating_transforms`] to union the
/// truncated-pyramid geometry into the plug mesh: the `-axis_unit`
/// half lives inside the plug body (CONTAINED in the mesh-CSG
/// boolean sense — manifold3d absorbs the inside half cleanly), and
/// the `+axis_unit` half PROTRUDES past the plug's cap-plane face
/// as the workshop-visible pyramid lock that seats into the
/// cup-piece floor socket. See [`add_plug_pins`] for the production
/// caller.
///
/// # Architecture note (2026-05-24 FDM-friendly arc salvage)
///
/// Pre-salvage this function was `build_plug_lock_sdf` returning
/// `Option<Solid>` for pre-MC SDF-side composition. The post-mortem
/// found pre-MC SDF composition silently destroyed workshop-visible
/// feature fidelity at production 3 mm MC cell size — pyramid
/// features 4 mm half-extent are a few MC cells across and got
/// mangled by quantization. Post-salvage this restores the prior
/// mating-features arc S6 architecture (POST-MC mesh-CSG primitives
/// carry their own native hull resolution). See
/// [[feedback-read-prior-arc-memory-before-architectural-decisions]].
#[must_use]
pub fn build_plug_lock_transform(ribbon: &Ribbon) -> Option<MatingTransform> {
    let (spec, pose) = build_plug_lock_pose(ribbon)?;
    Some(MatingTransform::UnionTruncatedPyramid {
        params: spec.lock_spec.pin_params(pose),
    })
}

/// Build the cup-side plug-floor-lock socket mesh-CSG transform
/// for this ribbon, or `None` when [`PlugPinKind::None`] or the
/// cap-plane anchor is unresolvable.
///
/// **Side-agnostic** transform: both Negative and Positive cup
/// pieces apply the *same* [`MatingTransform::SubtractTruncatedPyramid`]
/// to their per-piece half-shell mesh POST-MC. The cup-piece SDF
/// halfspace intersect (recon-4 (P), see
/// [`crate::piece::compose_piece_solid`]) bisects the cup-piece
/// material laterally through the seam plane in the SDF/MC stage;
/// the POST-MC mesh-CSG subtract then carves the socket cavity
/// from whichever half of cup-wall material exists on each piece —
/// the same physical socket geometry shows up correctly on both
/// halves (S6 three-piece shared-primitive invariant analog; see
/// module docstring §"3-piece shared-primitive invariant").
///
/// The socket inflates the pyramid extents per the symmetric `/2`
/// clearance convention from [`PrismaticPinSpec::socket_params`]:
/// each lateral extent grows by `diametral_clearance_m / 2` per
/// side; `half_length_m` grows by `axial_clearance_m / 2` on each
/// axial face. The cross-section at the cap-plane (axis = 0 from
/// the symmetric pose, the main-taper widest exposed section)
/// matches the plug-side pyramid cross-section by construction —
/// the bit-precise workshop fit invariant from [`PrismaticPinSpec`]
/// flows through.
#[must_use]
pub fn build_plug_lock_socket_transform(ribbon: &Ribbon) -> Option<MatingTransform> {
    let (spec, pose) = build_plug_lock_pose(ribbon)?;
    Some(MatingTransform::SubtractTruncatedPyramid {
        params: spec.lock_spec.socket_params(pose),
    })
}

/// Build a [`MatingTransform::SeamTrim`] enforcing a flat plug-
/// side cap-plane face.
///
/// Trims any MC-quantization facets the plug body's scan-derived
/// SDF produces at the cap-plane (cf-cast's mesh → SDF → MC
/// pipeline samples the flat scan cap-plane face at 3 mm grid
/// points and re-meshes, producing visible facets even when the
/// SDF is mathematically flat).
///
/// `normal = -cap_normal` (points into the plug body — the kept
/// half-space); base plane equation `dot(-cap_normal, cap_centroid)`
/// is shifted INWARD by [`PLUG_CAP_TRIM_BIAS_M`] (added to
/// `offset_m`) so the resulting trim face does NOT coincide with
/// the plug body's SDF cap-plane surface (see
/// [`PLUG_CAP_TRIM_BIAS_M`] for the paradigm-boundary rationale).
/// Trims away everything on the `+cap_normal` side of the biased
/// plane (= outside the plug body + a thin 1 µm slab inside the
/// body at the cap-plane face) post-MC. Must be applied BEFORE
/// [`build_plug_lock_transform`]'s `UnionTruncatedPyramid` —
/// the post-flip pyramid extends past the cap-plane into the
/// `+cap_normal` half-space (workshop-visible base-down protrusion);
/// trimming first flattens the plug body face, then the pyramid
/// union adds the protruding lock on top.
///
/// Returns `None` for an empty / unresolvable ribbon (same
/// fallback as `build_plug_lock_pose`).
#[must_use]
pub fn build_plug_cap_trim_transform(ribbon: &Ribbon) -> Option<MatingTransform> {
    // Gate on an EXPLICIT `pour_end_hint` — the centerline-tip
    // fallback from `pour_anchor` (used for hand-built test
    // ribbons without `with_pour_end_hint`) isn't a real cap-plane
    // anchor and the trim there could cut unintended material.
    let (cap_centroid, cap_normal_vec) = ribbon.pour_end_hint?;
    let cap_normal = UnitVector3::new_normalize(cap_normal_vec);
    let kept_normal = UnitVector3::new_normalize(-cap_normal.into_inner());
    // Base plane = cap-plane through `cap_centroid` with `kept_normal`
    // pointing into the plug body. Adding `PLUG_CAP_TRIM_BIAS_M` to
    // `offset_m` shifts the plane INTO the kept half-space (= 1 µm
    // INWARD from the body cap-plane face), producing a CONTAINED
    // trim face inside body pre-trim material — paradigm-safe per
    // [`PLUG_CAP_TRIM_BIAS_M`].
    let base_offset_m = kept_normal.into_inner().dot(&cap_centroid.coords);
    let offset_m = base_offset_m + PLUG_CAP_TRIM_BIAS_M;
    Some(MatingTransform::SeamTrim {
        normal: kept_normal,
        offset_m,
    })
}

/// Build a [`MatingTransform::SeamTrim`] enforcing a flat cup-
/// side cap-plane face.
///
/// Targets the interior boundary of the cup-piece at the cap-
/// plane plane (= the cup's floor face from the body-cavity side,
/// where the plug seats onto the cup-floor when assembled).
/// Sibling of [`build_plug_cap_trim_transform`]; uses the SAME
/// cap-plane reference but trims the OPPOSITE side (cup-floor
/// material is on `+cap_normal` side of the cap-plane, opposite
/// of the plug body).
///
/// `normal = +cap_normal` (points into cup-floor material — the
/// kept half-space); `offset_m = dot(cap_normal, cap_centroid)`.
/// Trims away everything on the `-cap_normal` side of the cap-
/// plane (= cup body cavity air + any post-CSG artifacts above
/// the cap-plane). Safe to apply AFTER the cup-side plug-lock
/// socket subtract: the socket cavity's `+axis_unit` half (post-
/// flip = `-cap_normal` direction = above cap-plane in cup-floor
/// reference) is a no-op carve in body cavity air; the trim
/// removes any boolean-junction artifacts there.
///
/// Returns `None` for an empty / unresolvable ribbon.
#[must_use]
pub fn build_cup_cap_trim_transform(ribbon: &Ribbon) -> Option<MatingTransform> {
    // Same `pour_end_hint` gating as [`build_plug_cap_trim_transform`]
    // — without explicit cap-plane data the trim could remove
    // unintended material in synthetic test fixtures.
    let (cap_centroid, cap_normal_vec) = ribbon.pour_end_hint?;
    let kept_normal = UnitVector3::new_normalize(cap_normal_vec);
    // Mirror the plug-side fix (§Q-4 S1): bias the trim plane
    // [`PLUG_CAP_TRIM_BIAS_M`] INTO the kept cup-floor half-space so the
    // trim face is CONTAINED in pre-trim material, not coincident with
    // the cup-floor MC surface — paradigm-safe per [`PLUG_CAP_TRIM_BIAS_M`].
    // This is what re-enables the cup trim disabled 2026-05-24 (the
    // unbiased plane was face-coincident → recon-4 (P) §F-2 failure).
    let offset_m = kept_normal.into_inner().dot(&cap_centroid.coords) + PLUG_CAP_TRIM_BIAS_M;
    Some(MatingTransform::SeamTrim {
        normal: kept_normal,
        offset_m,
    })
}

/// Pair a user-supplied plug [`Solid`] with the plug-floor-lock.
///
/// Returns `(plug_unchanged, transforms)` for downstream
/// marching-cubes + F4 gating — the pyramid transform is appended
/// to the post-MC mesh-CSG transforms Vec.
///
/// Post-salvage (2026-05-24): the plug [`Solid`] passes through
/// unchanged (no pre-MC SDF union); the lock geometry is added
/// POST-MC via [`MatingTransform::UnionTruncatedPyramid`] applied
/// by [`crate::apply_mating_transforms`]. For [`PlugPinKind::None`]
/// the transforms Vec is empty.
///
/// [`CastSpec::export_molds_v2`] calls this internally for each
/// per-layer plug (derived from `spec.plug` for layer 0 and
/// `layers[N-1].body` for `N > 0`), so callers passing
/// [`PlugPinKind::Axial`] via [`Ribbon::with_plug_pins`] get the
/// pyramid geometry baked into every plug STL automatically. The
/// function stays public for callers building a custom plug
/// [`Solid`] outside the [`crate::CastSpec`] per-layer-plug derivation.
///
/// [`Ribbon::with_plug_pins`]: crate::ribbon::Ribbon::with_plug_pins
/// [`CastSpec::export_molds_v2`]: crate::CastSpec::export_molds_v2
#[must_use]
pub fn add_plug_pins(plug: Solid, ribbon: &Ribbon) -> (Solid, Vec<MatingTransform>) {
    // **Cap-plane trim RE-ENABLED 2026-05-26 (§Q-4 S1 probe).** The
    // 2026-05-24 disabling here was caused by a face-coincident
    // trim plane (recon-4 (P) §F-2 WELDED-TO-BULK paradigm-boundary
    // failure — F4 flagged Critical on the trimmed plug). §Q-4 S1
    // bakes a 1 µm inward offset bias into `build_plug_cap_trim_transform`
    // via `PLUG_CAP_TRIM_BIAS_M`, shifting the trim plane OFF the
    // body SDF cap-plane → resulting trim face is CONTAINED inside
    // body pre-trim material → paradigm-safe. The trim flattens
    // the MC-quantization facets on the plug bottom (workshop-
    // visible PR-blocker at 2 mm cells per the recon's §Q-4-1
    // worked example), then the pyramid union adds the protruding
    // lock on top (order: trim before union per
    // `build_plug_cap_trim_transform`'s ordering note).
    //
    // The sibling cup-side builder `build_cup_cap_trim_transform`
    // stays retained-but-unused: cup-piece cap-plane MC flatness
    // needs the §Q-4 S2 `MatingTransform::UnionCuboid` slab
    // (handles MC under- AND overshoot symmetrically), not a
    // halfspace trim (which would only handle one direction).
    let mut transforms: Vec<MatingTransform> = Vec::new();
    transforms.extend(build_plug_cap_trim_transform(ribbon));
    transforms.extend(build_plug_lock_transform(ribbon));
    (plug, transforms)
}

#[cfg(test)]
// Tests deliberately compare f64 fields by exact value for emission
// determinism (any bit-difference would indicate a real regression,
// not measurement noise). `assert_abs_diff_eq!` machine-epsilon
// variants live alongside for decimal-mm input values that are not
// f64-exact.
#[allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::expect_used,
    clippy::float_cmp
)]
mod tests {
    use super::*;
    use crate::prismatic_pin::build_prismatic_pin_sdf;
    use crate::ribbon::{Ribbon, SplitNormal};
    use approx::assert_abs_diff_eq;
    use nalgebra::Point3;

    /// Test-only SDF-side shim — re-derives the plug-lock pin SDF
    /// from the post-salvage [`build_plug_lock_transform`] output
    /// for assertions that probe SDF values at world coords.
    /// Production no longer composes the lock pyramid into the plug
    /// `Solid` pre-MC (post-2026-05-24 salvage); this shim keeps
    /// the existing pose-math assertions working pending a
    /// follow-up session that migrates them to transform-parameter
    /// inspection.
    fn build_plug_lock_sdf(ribbon: &Ribbon) -> Option<Solid> {
        match build_plug_lock_transform(ribbon)? {
            MatingTransform::UnionTruncatedPyramid { params } => {
                Some(build_prismatic_pin_sdf(&params))
            }
            other => panic!("build_plug_lock_transform emitted unexpected variant: {other:?}"),
        }
    }

    /// Sibling SDF shim for the cup-side plug-lock socket — same
    /// rationale as [`build_plug_lock_sdf`].
    fn build_plug_lock_socket_sdf(ribbon: &Ribbon) -> Option<Solid> {
        match build_plug_lock_socket_transform(ribbon)? {
            MatingTransform::SubtractTruncatedPyramid { params } => {
                Some(build_prismatic_pin_sdf(&params))
            }
            other => {
                panic!("build_plug_lock_socket_transform emitted unexpected variant: {other:?}")
            }
        }
    }

    /// Standard test fixture mirroring the iter-1 cap-plane layout
    /// (centerline along ±Z with cf-scan-prep `trim_floor_mm` shape;
    /// split-normal +X; `cap_centroid` at the centerline's `-Z` end +
    /// `cap_normal` = -Z). Returns a ribbon with
    /// [`PlugPinKind::Axial(PlugPinSpec::iter1())`] enabled.
    ///
    /// Pose derivation under this fixture:
    /// - `axis_unit` = `cap_normal` = `-Z`
    /// - `lateral_unit` = `split_normal` = `+X`
    /// - Third basis = `lateral × axis = +X × -Z = +Y` — the
    ///   seam-normal direction (binormal = `tangent × split` = `-Z × +X`
    ///   = `-Y`, so `-Y` is the seam plane's outward normal; `+Y` per
    ///   the third-basis convention).
    ///
    /// Delegates to [`iter1_like_ribbon_with_spec`] with the iter-1
    /// default plug-lock spec.
    fn iter1_like_ribbon() -> Ribbon {
        iter1_like_ribbon_with_spec(PlugPinSpec::iter1())
    }

    #[test]
    fn plug_pin_spec_iter1_wraps_plug_lock_default() {
        let s = PlugPinSpec::iter1();
        assert_eq!(s.lock_spec, PrismaticPinSpec::plug_lock_default());
    }

    #[test]
    fn plug_pin_kind_default_is_none() {
        assert_eq!(PlugPinKind::default(), PlugPinKind::None);
    }

    #[test]
    fn build_plug_lock_sdf_none_returns_none() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        // Plug-pin-less ribbon → both SDF builders return None and
        // the empty-Vec pass-through in `apply_mating_transforms`
        // short-circuits the (empty) cup-side path.
        let ribbon = Ribbon::new(centerline, split).unwrap();
        assert!(build_plug_lock_sdf(&ribbon).is_none());
        assert!(build_plug_lock_socket_sdf(&ribbon).is_none());
    }

    /// With the standard iter-1 fixture both builders return `Some`
    /// and the returned Solids report interior at the cap-plane
    /// centroid (the centre point of the symmetric-across-cap-plane
    /// pose).
    #[test]
    fn build_plug_lock_sdf_axial_returns_solid_interior_at_cap_centroid() {
        let ribbon = iter1_like_ribbon();
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind must build a lock SDF");
        let socket =
            build_plug_lock_socket_sdf(&ribbon).expect("Axial kind must build a socket SDF");

        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        assert!(
            lock.evaluate(&cap_centroid) < 0.0,
            "plug-lock SDF at cap_centroid must be interior (centre of symmetric pose); SDF = {}",
            lock.evaluate(&cap_centroid),
        );
        assert!(
            socket.evaluate(&cap_centroid) < 0.0,
            "plug-lock socket SDF at cap_centroid must be interior; SDF = {}",
            socket.evaluate(&cap_centroid),
        );
    }

    /// Cap-plane anchor regression preserved post-S4: cf-scan-prep
    /// trims the centerline `trim_floor_mm` above the cap plane; the
    /// lock's pose centre lands at the cap-plane centroid, NOT at
    /// the trimmed centerline tip. Pre-S4 this lived in the shaft
    /// transform's parent-`center_m`; post-S4 it lives in the lock
    /// pose's `center_m`.
    #[test]
    fn plug_lock_pose_anchors_at_cap_plane_centroid_past_trimmed_centerline_tip() {
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));

        let (_spec, pose) = build_plug_lock_pose(&ribbon).expect("Axial kind");
        assert!(
            (pose.center_m.z - cap_centroid.z).abs() < 1e-12,
            "lock pose centre must anchor at cap_centroid (z = {}), \
             NOT at centerline.last() (z = -0.013); got centre z = {}",
            cap_centroid.z,
            pose.center_m.z,
        );
        assert!(
            (pose.axis_unit.z - 1.0).abs() < 1e-12,
            "lock pose axis must align with -cap_normal = +Z post-2026-05-24 base-down \
             captive-lock flip; got axis.z = {}",
            pose.axis_unit.z,
        );
    }

    /// Default (no hint) fallback: pour-end anchor uses
    /// `centerline.last()` extending along
    /// `+last_segment.tangent`, per cf-scan-prep's tip→base
    /// convention. Workshop iter-1 fixtures always pass an explicit
    /// hint; this test exercises the defensive hand-built-ribbon
    /// path.
    #[test]
    fn plug_lock_pose_default_fallback_uses_centerline_last() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.054)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let (_spec, pose) = build_plug_lock_pose(&ribbon).expect("Axial kind");
        // last segment runs +Z→-Z so tangent = -Z (cap_normal); anchor
        // centre = last.end = (0, 0, -0.054). Post-2026-05-24 base-down
        // flip: pose axis_unit = -cap_normal = +Z.
        assert!((pose.center_m.z - -0.054).abs() < 1e-12);
        assert!((pose.axis_unit.z - 1.0).abs() < 1e-12);
    }

    /// `add_plug_pins` for [`PlugPinKind::None`] passes the plug
    /// [`Solid`] through unchanged and returns an empty transforms
    /// Vec (no mesh-CSG ops emitted post-S4).
    #[test]
    fn add_plug_pins_passes_plug_through_unchanged_when_none() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.054)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let plug = Solid::capsule(0.005, 0.020);
        let (returned_plug, transforms) = add_plug_pins(plug.clone(), &ribbon);
        for q in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.025),
        ] {
            let bare = plug.evaluate(&q);
            let returned = returned_plug.evaluate(&q);
            assert!(
                (bare - returned).abs() < 1e-12,
                "PlugPinKind::None must pass plug through unchanged at {q:?}; \
                 bare={bare}, returned={returned}",
            );
        }
        assert!(
            transforms.is_empty(),
            "S4: plug self-emission emits no post-MC mesh-CSG ops; got {} transforms",
            transforms.len(),
        );
    }

    /// `build_plug_cap_trim_transform` emits a `SeamTrim` whose
    /// `normal` direction points INTO the plug body (= `-cap_normal`)
    /// and whose `offset_m` is the plane-equation value at
    /// `cap_centroid` PLUS the 1 µm inward bias per
    /// [`PLUG_CAP_TRIM_BIAS_M`]. Verifies the trim keeps the plug
    /// body side (where the workshop-visible plug material lives)
    /// and trims away anything on the `+cap_normal` side (above the
    /// cap-plane in pour orientation — outside the plug body, where
    /// MC quantization artifacts would otherwise sit).
    #[test]
    fn build_plug_cap_trim_transform_keeps_plug_body_side() {
        let ribbon = iter1_like_ribbon();
        let trim = build_plug_cap_trim_transform(&ribbon).expect("pour_end_hint set");
        match trim {
            MatingTransform::SeamTrim { normal, offset_m } => {
                let cap_centroid = Point3::new(0.0, 0.0, -0.054);
                let cap_normal = Vector3::new(0.0, 0.0, -1.0);
                let kept_normal = -cap_normal;
                let expected_offset = kept_normal.dot(&cap_centroid.coords) + PLUG_CAP_TRIM_BIAS_M;
                assert_abs_diff_eq!(normal.into_inner().x, kept_normal.x, epsilon = 1.0e-12);
                assert_abs_diff_eq!(normal.into_inner().y, kept_normal.y, epsilon = 1.0e-12);
                assert_abs_diff_eq!(normal.into_inner().z, kept_normal.z, epsilon = 1.0e-12);
                assert_abs_diff_eq!(offset_m, expected_offset, epsilon = 1.0e-12);
            }
            other => panic!("expected SeamTrim, got {other:?}"),
        }
    }

    /// §Q-4 S1 paradigm-boundary regression gate. The emitted
    /// `SeamTrim.offset_m` MUST be greater than the unbiased
    /// cap-plane offset by EXACTLY [`PLUG_CAP_TRIM_BIAS_M`] — i.e.,
    /// the trim plane is shifted INWARD (into the kept plug-body
    /// half-space) by the documented bias magnitude. This is the
    /// load-bearing invariant that makes the trim face CONTAINED
    /// inside the body's pre-trim material rather than coincident
    /// with the body SDF cap-plane (recon-4 (P) §F-2). A future
    /// rewrite that drops the bias or flips its sign would silently
    /// reintroduce the WELDED-TO-BULK failure that disabled this
    /// transform 2026-05-24.
    #[test]
    fn build_plug_cap_trim_transform_shifts_plane_one_micron_inward() {
        let ribbon = iter1_like_ribbon();
        let trim =
            build_plug_cap_trim_transform(&ribbon).expect("iter1_like_ribbon sets pour_end_hint");
        match trim {
            MatingTransform::SeamTrim { normal, offset_m } => {
                let cap_centroid = Point3::new(0.0, 0.0, -0.054);
                let unbiased_offset = normal.into_inner().dot(&cap_centroid.coords);
                let delta = offset_m - unbiased_offset;
                assert_abs_diff_eq!(delta, PLUG_CAP_TRIM_BIAS_M, epsilon = 1.0e-15);
                assert!(
                    delta > 0.0,
                    "bias MUST be POSITIVE (shift plane INTO kept half-space) — \
                     a non-positive delta would put the trim plane on / outside \
                     the body SDF cap-plane and re-open the recon-4 (P) §F-2 \
                     WELDED-TO-BULK paradigm-boundary failure; got delta = {delta}",
                );
            }
            other => panic!("expected SeamTrim, got {other:?}"),
        }
    }

    /// `build_plug_cap_trim_transform` returns `None` when the
    /// ribbon has no `pour_end_hint` — the `centerline-tip` fallback
    /// from `pour_anchor` isn't a real cap-plane anchor.
    #[test]
    fn build_plug_cap_trim_transform_none_without_pour_end_hint() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.054)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        assert!(build_plug_cap_trim_transform(&ribbon).is_none());
        assert!(build_cup_cap_trim_transform(&ribbon).is_none());
    }

    /// `build_cup_cap_trim_transform` is the SIBLING of the
    /// plug-side trim — same cap-plane reference, OPPOSITE kept
    /// half-space (cup-floor material lives on the `+cap_normal`
    /// side, opposite of the plug body). Verifies the trim keeps
    /// the cup-floor side and trims away material on the
    /// `-cap_normal` side (body cavity air + any post-CSG
    /// artifacts there).
    #[test]
    fn build_cup_cap_trim_transform_keeps_cup_floor_side() {
        let ribbon = iter1_like_ribbon();
        let trim = build_cup_cap_trim_transform(&ribbon).expect("pour_end_hint set");
        match trim {
            MatingTransform::SeamTrim { normal, offset_m } => {
                let cap_centroid = Point3::new(0.0, 0.0, -0.054);
                let cap_normal = Vector3::new(0.0, 0.0, -1.0);
                let expected_offset = cap_normal.dot(&cap_centroid.coords) + PLUG_CAP_TRIM_BIAS_M;
                assert_abs_diff_eq!(normal.into_inner().x, cap_normal.x, epsilon = 1.0e-12);
                assert_abs_diff_eq!(normal.into_inner().y, cap_normal.y, epsilon = 1.0e-12);
                assert_abs_diff_eq!(normal.into_inner().z, cap_normal.z, epsilon = 1.0e-12);
                assert_abs_diff_eq!(offset_m, expected_offset, epsilon = 1.0e-12);
            }
            other => panic!("expected SeamTrim, got {other:?}"),
        }
    }

    /// `add_plug_pins` for [`PlugPinKind::Axial`] returns the bare
    /// plug [`Solid`] unchanged AND a transforms Vec containing
    /// (post-§Q-4-S1) TWO transforms in declared order:
    /// (0) [`MatingTransform::SeamTrim`] flattening the plug bottom
    /// face at the biased cap-plane (paradigm-safe per
    /// [`PLUG_CAP_TRIM_BIAS_M`]); (1)
    /// [`MatingTransform::UnionTruncatedPyramid`] adding the
    /// protruding plug-lock geometry on top. Order is load-bearing
    /// per `build_plug_cap_trim_transform`'s ordering note — trim
    /// FIRST then union, so the pyramid's base-down protrusion is
    /// preserved against the otherwise-applied trim.
    #[test]
    fn add_plug_pins_axial_unions_pyramid_into_plug() {
        let ribbon = iter1_like_ribbon();
        let plug = Solid::capsule(0.005, 0.020);
        let (returned_plug, transforms) = add_plug_pins(plug.clone(), &ribbon);

        // Returned plug Solid is the BARE plug — lock geometry no
        // longer composed into it pre-MC.
        for q in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.025),
        ] {
            let bare = plug.evaluate(&q);
            let returned = returned_plug.evaluate(&q);
            assert!(
                (bare - returned).abs() < 1e-12,
                "plug Solid passes through unchanged at {q:?}; bare={bare}, returned={returned}",
            );
        }

        // Transforms Vec carries the cap-plane trim followed by the
        // plug-lock pyramid union (re-enabled 2026-05-26 in §Q-4 S1
        // with the 1 µm paradigm-safety bias; see
        // `add_plug_pins` body comment + `PLUG_CAP_TRIM_BIAS_M`).
        assert_eq!(
            transforms.len(),
            2,
            "Axial kind emits cap-plane SeamTrim then plug-lock UnionTruncatedPyramid; \
             got {transforms:#?}",
        );
        match &transforms[0] {
            MatingTransform::SeamTrim { .. } => {}
            other => panic!("expected SeamTrim at index 0 (trim FIRST), got {other:?}"),
        }
        match &transforms[1] {
            MatingTransform::UnionTruncatedPyramid { params } => {
                let spec = PlugPinSpec::iter1();
                assert_eq!(
                    params.half_length_m, spec.lock_spec.pin_half_length_m,
                    "transform params carry pin (not socket) half-length",
                );
                assert_eq!(
                    params.base_chamfer_m, spec.lock_spec.base_chamfer_m,
                    "transform params carry the lock spec's chamfer",
                );
            }
            other => panic!("expected UnionTruncatedPyramid at index 1, got {other:?}"),
        }

        // Shimmed SDF probe at the same probe point as the
        // pre-salvage test — confirms the transform's params encode
        // the correct pyramid geometry (re-deriving the SDF
        // matches what `apply_mating_transforms` will union into
        // the plug mesh post-MC).
        let lock_sdf = build_plug_lock_sdf(&ribbon).expect("Axial kind builds a lock");
        let probe_in_lock = Point3::new(0.0, 0.0, -0.056);
        assert!(
            lock_sdf.evaluate(&probe_in_lock) < 0.0,
            "lock pyramid SDF must report interior at probe inside the `+axis_unit` half; \
             SDF = {}",
            lock_sdf.evaluate(&probe_in_lock),
        );
        // Bare-plug spot-check: same probe is OUTSIDE the bare
        // capsule (capsule half-length 20 mm, cap-plane face at z =
        // -0.020; probe at z = -0.056 is well past).
        assert!(
            plug.evaluate(&probe_in_lock) > 0.0,
            "bare plug must report exterior at probe {probe_in_lock:?}; SDF = {}",
            plug.evaluate(&probe_in_lock),
        );
        // Far-field probe: well outside both bare plug + lock.
        let probe_outside = Point3::new(0.1, 0.0, 0.0);
        assert!(returned_plug.evaluate(&probe_outside) > 0.0);
    }

    /// §G-10 #1 bit-precise fit invariant at the spec layer: lock
    /// pin extents differ from socket extents by exactly
    /// `clearance / 2` per axis within machine epsilon. SDF-side
    /// analog of S5's mesh-CSG `pin_and_socket_fit_invariant`. The
    /// contract lives at the SDF input layer (per
    /// [`crate::prismatic_pin`] module docstring) — the SDF kernel
    /// composes the primitive lazily, so a determinism contract on
    /// the SDF input bytes implies bit-equal SDF evaluation across
    /// plug-side / cup-side modulo clearance inflation.
    ///
    /// Extends [`crate::prismatic_pin`]'s
    /// `prismatic_pin_pair_extents_match_spec_clearance_within_machine_epsilon`
    /// to the plug-lock call path (verifies this module's spec
    /// consumption — not just the primitive in isolation).
    #[test]
    fn plug_lock_and_socket_fit_invariant() {
        let ribbon = iter1_like_ribbon();
        let (spec, pose) = build_plug_lock_pose(&ribbon).expect("Axial kind");
        let pin_params = spec.lock_spec.pin_params(pose.clone());
        let socket_params = spec.lock_spec.socket_params(pose);

        let half_diametral = spec.lock_spec.diametral_clearance_m / 2.0;
        let half_axial = spec.lock_spec.axial_clearance_m / 2.0;

        assert_abs_diff_eq!(
            socket_params.base_half_extents_m.x - pin_params.base_half_extents_m.x,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.base_half_extents_m.y - pin_params.base_half_extents_m.y,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.tip_half_extents_m.x - pin_params.tip_half_extents_m.x,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.tip_half_extents_m.y - pin_params.tip_half_extents_m.y,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.half_length_m - pin_params.half_length_m,
            half_axial,
            epsilon = 1.0e-15,
        );
        assert_eq!(socket_params.base_chamfer_m, pin_params.base_chamfer_m);
    }

    /// §G-10 #4 (S4-specific): the plug-side pyramid and cup-side
    /// socket are pose-symmetric across the cap-plane, so at the
    /// cap-plane (axis = 0 in pose-local coords) the SDF offset
    /// between socket and pin equals zero IN THE AXIAL DIRECTION
    /// (both span axis ∈ `[-half_length_pin, +half_length_pin]` for
    /// the pin and `[-half_length_socket, +half_length_socket]` for
    /// the socket — at axis = 0 both cross-sections share the same
    /// nominal axial-cap distance). The lateral SDF gap at the
    /// cap-plane equals the lateral clearance per the symmetric `/2`
    /// inflation — i.e., the workshop fit at the cap-plane interior
    /// is bit-precise modulo the clearance budget by construction.
    ///
    /// This is the synthetic analog of the workshop-physical
    /// "lock seats flush against floor" gate: pyramid and socket
    /// share the same pose; their cross-sections at the cap-plane
    /// are concentric with the socket inflated by `diametral/2` per
    /// lateral side.
    #[test]
    fn plug_lock_pin_and_socket_share_cap_plane_cross_section() {
        let ribbon = iter1_like_ribbon();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind");
        let socket = build_plug_lock_socket_sdf(&ribbon).expect("Axial kind");
        // Probe at cap_centroid (axis = 0 in pose-local) directly:
        // the pin reads interior at its centre; the socket reads
        // interior at the same point (inflated extents). Difference
        // is the SDF gap to the pin's surface (negative, deeper
        // interior on the socket side because the socket is larger).
        let pin_sdf = lock.evaluate(&cap_centroid);
        let socket_sdf = socket.evaluate(&cap_centroid);
        assert!(pin_sdf < 0.0);
        assert!(socket_sdf < 0.0);
        assert!(
            socket_sdf < pin_sdf,
            "socket interior at cap_centroid must be DEEPER than pin interior \
             (socket is the inflated pair); pin SDF = {pin_sdf}, socket SDF = {socket_sdf}",
        );
        // Probe at a point lateral to the cap-plane centroid by
        // (base_chamfer + 0.5 mm) — past the chamfer band, within
        // the main taper cross-section at axis = 0. At axis = 0 the
        // main taper's lateral extent is the midpoint between base
        // and tip = (4.0 + 2.8) / 2 = 3.4 mm half-extent. A probe at
        // (3.2 mm, 0, cap_z) sits 0.2 mm inside the pin's +X face;
        // the socket extends 0.175 mm (= diametral/2) further along
        // each lateral axis, so the socket is interior by ~0.375 mm
        // there.
        let lateral_probe = cap_centroid + Vector3::new(0.0032, 0.0, 0.0);
        assert!(
            lock.evaluate(&lateral_probe) < 0.0,
            "pin must be interior at (3.2 mm, 0, cap_z); SDF = {}",
            lock.evaluate(&lateral_probe),
        );
        assert!(
            socket.evaluate(&lateral_probe) < 0.0,
            "socket must be interior at (3.2 mm, 0, cap_z) too (inflated extents); SDF = {}",
            socket.evaluate(&lateral_probe),
        );
        // Just-outside the pin's lateral extent at the cap-plane:
        // 3.6 mm (= 0.2 mm past the 3.4 mm half-extent midpoint).
        // Pin exterior; socket still interior (3.6 mm < 3.4 + 0.175
        // = 3.575 mm? — no, 3.6 > 3.575, so socket is also exterior
        // at this probe). Sample a finer point: 3.5 mm (between pin
        // surface and socket surface).
        let between_probe = cap_centroid + Vector3::new(0.0035, 0.0, 0.0);
        assert!(
            lock.evaluate(&between_probe) > 0.0,
            "pin must be exterior at (3.5 mm, 0, cap_z); SDF = {}",
            lock.evaluate(&between_probe),
        );
        assert!(
            socket.evaluate(&between_probe) < 0.0,
            "socket must still be interior at (3.5 mm, 0, cap_z) — the inflated lateral \
             clearance places the socket wall past the pin wall by diametral/2 = 0.175 mm; \
             SDF = {}",
            socket.evaluate(&between_probe),
        );
    }

    /// Gram-Schmidt regression: production cf-scan-prep cap-normal
    /// is PCA-fitted on the cap-plane vertex cloud while the ribbon's
    /// `split_normal` is a user-axis projection, so production casts
    /// see ~1-3° between them. Pre-Gram-Schmidt the
    /// [`PrismaticPinPose::new`] orthogonality assert panicked at
    /// `|dot| > 1e-9` on the first iter-1 regen attempt with
    /// `|dot| ≈ 0.041` (≈ 2.4°). This test rebuilds that fixture
    /// (non-orthogonal `cap_normal` vs `split_normal`) and verifies
    /// (a) the pose builder doesn't panic, (b) the returned pose's
    /// `lateral_unit ⊥ axis_unit` to f64 precision (the assertion
    /// contract), and (c) the lock SDF still reports interior at
    /// `cap_centroid` (the pose's geometric centre).
    #[test]
    fn build_plug_lock_pose_gram_schmidt_handles_non_orthogonal_cap_normal() {
        // cap_normal tilted 2-3° from -Z: dot(cap_normal, +X
        // split_normal) ≈ 0.04. Matches the iter-1 production
        // misalignment that surfaced this regression.
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.04, 0.0, -1.0).normalize();
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));

        let (_spec, pose) = build_plug_lock_pose(&ribbon)
            .expect("Gram-Schmidt must rescue non-orthogonal cap-normal × split-normal");

        let dot = pose
            .axis_unit
            .into_inner()
            .dot(&pose.lateral_unit.into_inner());
        assert!(
            dot.abs() < 1.0e-12,
            "pose lateral_unit must be orthogonal to axis_unit to f64 precision after \
             Gram-Schmidt projection; got |dot| = {} (PrismaticPinPose::new's \
             LATERAL_ORTHOGONALITY_TOLERANCE = 1e-9 — Gram-Schmidt should hit ~ulp-level \
             orthogonality, well past the tolerance)",
            dot.abs(),
        );

        // Geometric sanity: the lock SDF still reports interior at
        // the pose centre (the cap-plane centroid).
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind must build");
        assert!(
            lock.evaluate(&cap_centroid) < 0.0,
            "lock SDF at cap_centroid must be interior after Gram-Schmidt; got SDF = {}",
            lock.evaluate(&cap_centroid),
        );
    }

    /// Bisect for the SDF zero-crossing along `lateral_unit` from
    /// `base_world` (`base_world = center + axial * axis_unit` at
    /// lateral = 0). World-coord analog of
    /// `crate::prismatic_pin::tests::find_x_zero_crossing`; same
    /// half-open interior predicate (`sdf <= +1e-12`) for sub-ulp
    /// positive noise on the chamfer-band cap planes.
    ///
    /// MIRROR: an identical copy lives in `crate::registration::tests`;
    /// any change to one MUST mirror the other (or the helper should
    /// be promoted to a shared test-util module).
    fn find_lateral_zero_crossing(
        sdf: &Solid,
        base_world: Point3<f64>,
        lateral_unit: Vector3<f64>,
        lo: f64,
        hi: f64,
    ) -> f64 {
        let mut a = lo;
        let mut b = hi;
        for _ in 0..64 {
            let m = f64::midpoint(a, b);
            if sdf.evaluate(&(base_world + m * lateral_unit)) <= 1.0e-12 {
                a = m;
            } else {
                b = m;
            }
        }
        f64::midpoint(a, b)
    }

    /// Build an iter-1-like ribbon with a custom `PlugPinSpec`. Lets
    /// the chamfer-band tests build a chamfer-disabled bare-baseline
    /// ribbon for fixture-breakage detection per
    /// [[feedback-load-bearing-test-fixtures]].
    fn iter1_like_ribbon_with_spec(spec: PlugPinSpec) -> Ribbon {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(spec))
    }

    /// §G-10 S5 chamfer band production-emission gate: the chamfer
    /// band that S2's [`crate::build_prismatic_pin_sdf`] composes at
    /// the primitive layer must hold geometrically when emitted
    /// through [`build_plug_lock_sdf`]'s pose-rotation +
    /// world-translation on the production plug-lock call path
    /// (symmetric-across-cap-plane pose, not the synthetic identity
    /// pose used in
    /// `crate::prismatic_pin::tests::chamfer_band_lateral_extents_match_spec_across_g6_range`).
    ///
    /// Iter-1 fixture pose: `center = cap_centroid` = (0, 0, -0.054),
    /// `axis_unit` = +Z (`-cap_normal` post-2026-05-24 base-down
    /// captive-lock flip), `lateral_unit` = +X (`split_normal`).
    /// Pin-local -Y (the chamfer-band base; this is the
    /// `-axis_unit`-end of the symmetric-across-cap-plane pose —
    /// post-flip it now lives along `+cap_normal` = DOWN =
    /// **workshop-visible bottom face of the protruding pyramid**,
    /// acting as an insertion lead-in into the cup-piece floor
    /// socket) maps to world -Z; lateral pin-local +X maps to
    /// world +X. Chamfer-band-base axial coord = `-half_length`,
    /// world `z = cap_z - half_length`. Chamfer-top axial coord =
    /// `-half_length + chamfer`, world `z = cap_z - half_length + chamfer`.
    ///
    /// Verifies (a) the chamfer-band-base lateral extent equals
    /// `(base - chamfer)` to ~1 µm; (b) the chamfer-top extent
    /// equals `base` to ~1 µm. Paired with a chamfer-disabled bare-
    /// baseline per [[feedback-load-bearing-test-fixtures]]: a
    /// `base_chamfer_m = 0.0` override emits a single-frustum
    /// pyramid whose chamfer-band-base extent equals `base` exactly;
    /// the delta between bare-baseline and with-chamfer base extents
    /// must equal `base_chamfer_m`.
    #[test]
    fn plug_lock_chamfer_band_holds_through_sdf_emission() {
        let lock_spec = PrismaticPinSpec::plug_lock_default();
        let ribbon = iter1_like_ribbon();
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind must build a lock SDF");

        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        // axis_unit = +Z post-2026-05-24 base-down flip → pin-local
        // +Y maps to world +Z; the chamfer-band-base axial coord
        // (pin-local `-half_length`) maps to `cap_centroid +
        // (-half_length) * +Z = cap_centroid - half_length * +Z`.
        let axis_unit = Vector3::new(0.0, 0.0, 1.0);
        let lateral_unit = Vector3::new(1.0, 0.0, 0.0);

        let bed_axial = -lock_spec.pin_half_length_m;
        let bed_base = cap_centroid + bed_axial * axis_unit;
        let bed_expected = lock_spec.pin_base_half_extents_m.x - lock_spec.base_chamfer_m;
        let bed_extent = find_lateral_zero_crossing(&lock, bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(bed_extent, bed_expected, epsilon = 1.0e-6);

        let top_axial = -lock_spec.pin_half_length_m + lock_spec.base_chamfer_m;
        let top_base = cap_centroid + top_axial * axis_unit;
        let top_expected = lock_spec.pin_base_half_extents_m.x;
        let top_extent = find_lateral_zero_crossing(&lock, top_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(top_extent, top_expected, epsilon = 1.0e-6);

        // Bare-baseline pairing: chamfer-disabled spec emits a
        // single-frustum pyramid; bed-face extent equals `base`.
        let bare_ribbon = iter1_like_ribbon_with_spec(PlugPinSpec {
            lock_spec: PrismaticPinSpec {
                base_chamfer_m: 0.0,
                ..lock_spec
            },
        });
        let bare_lock = build_plug_lock_sdf(&bare_ribbon).expect("bare-baseline lock must build");
        let bare_bed_extent =
            find_lateral_zero_crossing(&bare_lock, bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(
            bare_bed_extent,
            lock_spec.pin_base_half_extents_m.x,
            epsilon = 1.0e-6,
        );
        assert_abs_diff_eq!(
            bare_bed_extent - bed_extent,
            lock_spec.base_chamfer_m,
            epsilon = 1.0e-6,
        );
    }

    /// §G-10 S5 cup-side-socket chamfer gate: the cup-side socket
    /// emitted by [`build_plug_lock_socket_sdf`] inflates the
    /// plug-side lock's chamfer-band cross-section **at the socket's
    /// own bed face** by exactly `diametral_clearance_m / 2` per
    /// lateral side. Same invariant at the chamfer-top boundary
    /// (widest section).
    ///
    /// Why each side's **own** bed face (not a shared world-coord):
    /// [`PrismaticPinSpec::socket_params`] inflates `half_length_m`
    /// by `axial_clearance_m / 2`, so the socket's bed face sits
    /// `axial_clearance_m / 2` axially deeper than the lock's
    /// (mirror of the cup-pin pattern; same rationale in
    /// `crate::registration::tests::cup_pin_socket_chamfer_matches_pin`).
    /// Probing at a shared world-axial coord would land midway
    /// through the socket's chamfer-band interpolation, conflating
    /// chamfer-band emission with the axial-clearance offset.
    ///
    /// Paired with a chamfer-disabled bare baseline per
    /// [[feedback-load-bearing-test-fixtures]]: with chamfer disabled
    /// the socket-vs-pin bed-face delta still equals
    /// `diametral_clearance_m / 2` — confirms the socket extents
    /// emission path is chamfer-independent.
    #[test]
    fn plug_lock_socket_chamfer_matches_lock_pin() {
        let lock_spec = PrismaticPinSpec::plug_lock_default();
        let half_diametral = lock_spec.diametral_clearance_m / 2.0;
        let half_axial = lock_spec.axial_clearance_m / 2.0;
        let ribbon = iter1_like_ribbon();
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind");
        let socket = build_plug_lock_socket_sdf(&ribbon).expect("Axial kind");

        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        // Post-2026-05-24 base-down flip: pose axis_unit = +Z
        // (= -cap_normal), not -Z.
        let axis_unit = Vector3::new(0.0, 0.0, 1.0);
        let lateral_unit = Vector3::new(1.0, 0.0, 0.0);

        // Pin bed face at axial = -pin_half_length; socket bed face at
        // axial = -(pin_half_length + half_axial) (socket inflates
        // half_length by axial_clearance/2).
        let pin_bed_base = cap_centroid + (-lock_spec.pin_half_length_m) * axis_unit;
        let socket_bed_base =
            cap_centroid + (-(lock_spec.pin_half_length_m + half_axial)) * axis_unit;
        let pin_bed = find_lateral_zero_crossing(&lock, pin_bed_base, lateral_unit, 0.0, 0.01);
        let socket_bed =
            find_lateral_zero_crossing(&socket, socket_bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(socket_bed - pin_bed, half_diametral, epsilon = 1.0e-6);

        // Each side's own chamfer-top boundary.
        let pin_top_base =
            cap_centroid + (-lock_spec.pin_half_length_m + lock_spec.base_chamfer_m) * axis_unit;
        let socket_top_base = cap_centroid
            + (-(lock_spec.pin_half_length_m + half_axial) + lock_spec.base_chamfer_m) * axis_unit;
        let pin_top = find_lateral_zero_crossing(&lock, pin_top_base, lateral_unit, 0.0, 0.01);
        let socket_top =
            find_lateral_zero_crossing(&socket, socket_top_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(socket_top - pin_top, half_diametral, epsilon = 1.0e-6);

        // Bare-baseline (chamfer disabled): socket-vs-pin delta at
        // each side's bed face equals `diametral_clearance_m / 2`.
        // Chamfer-independence sanity check.
        let bare_ribbon = iter1_like_ribbon_with_spec(PlugPinSpec {
            lock_spec: PrismaticPinSpec {
                base_chamfer_m: 0.0,
                ..lock_spec
            },
        });
        let bare_lock = build_plug_lock_sdf(&bare_ribbon).expect("bare-baseline lock must build");
        let bare_socket =
            build_plug_lock_socket_sdf(&bare_ribbon).expect("bare-baseline socket must build");
        let bare_pin_bed =
            find_lateral_zero_crossing(&bare_lock, pin_bed_base, lateral_unit, 0.0, 0.01);
        let bare_socket_bed =
            find_lateral_zero_crossing(&bare_socket, socket_bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(
            bare_socket_bed - bare_pin_bed,
            half_diametral,
            epsilon = 1.0e-6,
        );
    }

    /// §G-10 lock-extends-across-cap-plane check: the lock pyramid
    /// extends `±half_length_m` along `axis_unit` symmetrically
    /// across the cap-plane. The `-axis_unit` half is buried inside
    /// the plug body (SDF-union absorbs it); the `+axis_unit` half
    /// protrudes outward. Confirms the symmetric-across-cap-plane
    /// pose convention from the module docstring.
    #[test]
    fn plug_lock_extends_symmetrically_across_cap_plane() {
        let ribbon = iter1_like_ribbon();
        let lock = build_plug_lock_sdf(&ribbon).expect("Axial kind");
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let half_length = PrismaticPinSpec::plug_lock_default().pin_half_length_m;
        // axis_unit = -Z, so -axis_unit half = +Z direction (toward
        // plug body), +axis_unit half = -Z direction (protruding).
        // Probe 1 mm INSIDE each tip (axially).
        let inside_neg_axis = cap_centroid + Vector3::new(0.0, 0.0, half_length - 1.0e-3);
        let inside_pos_axis = cap_centroid + Vector3::new(0.0, 0.0, -(half_length - 1.0e-3));
        // Probe 0.1 mm PAST the +axis_unit tip (exterior).
        let exterior_pos_axis = cap_centroid + Vector3::new(0.0, 0.0, -(half_length + 1.0e-4));

        assert!(
            lock.evaluate(&inside_neg_axis) < 0.0,
            "lock interior at -axis_unit half (z = cap_z + half_length - 1 mm) must be inside; \
             SDF = {}",
            lock.evaluate(&inside_neg_axis),
        );
        assert!(
            lock.evaluate(&inside_pos_axis) < 0.0,
            "lock interior at +axis_unit half (z = cap_z - half_length + 1 mm) must be inside; \
             confirms symmetric extent across cap-plane; SDF = {}",
            lock.evaluate(&inside_pos_axis),
        );
        assert!(
            lock.evaluate(&exterior_pos_axis) > 0.0,
            "0.1 mm past +axis_unit tip must be exterior; SDF = {}",
            lock.evaluate(&exterior_pos_axis),
        );
    }
}
