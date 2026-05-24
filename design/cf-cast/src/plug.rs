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
//! plug self-emission ([`build_plug_lock_sdf`]) unions the full
//! pyramid into the plug body; cup self-emission
//! ([`build_plug_lock_socket_sdf`]) returns one socket Solid that
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
use crate::prismatic_pin::{PrismaticPinPose, PrismaticPinSpec, build_prismatic_pin_sdf};
use crate::ribbon::Ribbon;

/// Plug-floor-lock geometry spec — wraps the SDF-side
/// [`PrismaticPinSpec`] primitive with no extra fields.
///
/// Post-S4 of the FDM-friendly geometry arc, the lock is a single
/// truncated-pyramid press-fit feature at the plug body's
/// cap-plane face (§G-1). The wrapper exists for parallel structure
/// with [`crate::PinSpec`] (cup-pin registration) — both wrap a
/// [`PrismaticPinSpec`] and add per-feature placement; the plug
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
/// `axis_unit = +cap_normal` (= away from plug body, downward in
/// pour orientation); `lateral_unit = +split_normal` (orthogonal to
/// `cap_normal` by ribbon construction — see module docstring
/// §"Pose convention").
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
    let axis_unit = UnitVector3::new_normalize(outward);
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
    let split_vec = ribbon.split_normal.as_vector();
    let axis_v = axis_unit.into_inner();
    let projected = split_vec - axis_v * split_vec.dot(&axis_v);
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

/// Build the plug-side plug-floor-lock SDF for this ribbon, or
/// `None` when [`PlugPinKind::None`] or the cap-plane anchor is
/// unresolvable.
///
/// The returned [`Solid`] is the truncated-pyramid in world coords:
/// base flush at the plug body's cap-plane face (`-axis_unit` half
/// buried inside the plug body, absorbed by SDF union), tip
/// protruding outward along `+axis_unit` for `half_length_m` (the
/// workshop-visible portion that seats into the cup-piece floor
/// socket). Callers UNION this into the per-layer plug [`Solid`]
/// pre-MC; see [`add_plug_pins`] for the production caller.
#[must_use]
pub fn build_plug_lock_sdf(ribbon: &Ribbon) -> Option<Solid> {
    let (spec, pose) = build_plug_lock_pose(ribbon)?;
    Some(build_prismatic_pin_sdf(&spec.lock_spec.pin_params(pose)))
}

/// Build the cup-side plug-floor-lock socket SDF for this ribbon,
/// or `None` when [`PlugPinKind::None`] or the cap-plane anchor is
/// unresolvable.
///
/// **Side-agnostic** Solid: both Negative and Positive cup pieces
/// SUBTRACT the *same* socket Solid from their per-piece half-shell
/// pre-MC. The cup-piece SDF halfspace intersect (recon-4 (P), see
/// [`crate::piece::compose_piece_solid`]) bisects the socket
/// laterally through the seam plane by construction — the pyramid
/// extends `±base_half_extents.y` along the seam-normal direction,
/// so each cup half receives one half of the socket cross-section
/// (S6 three-piece shared-primitive invariant analog; see module
/// docstring §"3-piece shared-primitive invariant").
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
pub fn build_plug_lock_socket_sdf(ribbon: &Ribbon) -> Option<Solid> {
    let (spec, pose) = build_plug_lock_pose(ribbon)?;
    Some(build_prismatic_pin_sdf(&spec.lock_spec.socket_params(pose)))
}

/// Pair a user-supplied plug [`Solid`] with the plug-floor-lock
/// pyramid SDF unioned into it, returning `(plug_with_lock,
/// transforms)` for downstream marching-cubes + F4 gating.
///
/// `transforms` is always empty post-S4 — the plug-floor lock lives
/// entirely SDF-side per §G-12 #2, so plug self-emission produces
/// no post-MC mesh-CSG ops. The tuple shape matches pre-S4 callers
/// in [`crate::CastSpec::export_molds_v2`] that thread
/// `(Solid, Vec<MatingTransform>)` into the per-layer plug pipeline
/// uniformly with other artifact paths.
///
/// For [`PlugPinKind::None`] the plug Solid is returned unchanged
/// and `transforms` is empty.
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
    let with_lock = match build_plug_lock_sdf(ribbon) {
        Some(lock) => plug.union(lock),
        None => plug,
    };
    (with_lock, Vec::new())
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
    use crate::ribbon::{Ribbon, SplitNormal};
    use approx::assert_abs_diff_eq;
    use nalgebra::Point3;

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
    fn iter1_like_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()))
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
            (pose.axis_unit.z - -1.0).abs() < 1e-12,
            "lock pose axis must align with cap_normal = -Z; got axis.z = {}",
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
        // last segment runs +Z→-Z so tangent = -Z; anchor centre =
        // last.end = (0, 0, -0.054).
        assert!((pose.center_m.z - -0.054).abs() < 1e-12);
        assert!((pose.axis_unit.z - -1.0).abs() < 1e-12);
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

    /// `add_plug_pins` for [`PlugPinKind::Axial`] returns a plug
    /// Solid that is the SDF-union of the bare plug body with the
    /// pyramid SDF. The returned plug's SDF at a point JUST OUTSIDE
    /// the bare plug body but INSIDE the pyramid's `+axis_unit`
    /// (protruding) half reports interior — confirms the union
    /// composed the pyramid into the plug.
    #[test]
    fn add_plug_pins_axial_unions_pyramid_into_plug() {
        let ribbon = iter1_like_ribbon();
        // Use a small capsule so a probe point just past its
        // cap-plane face is clearly outside the bare plug body.
        let plug = Solid::capsule(0.005, 0.020);
        let (returned_plug, transforms) = add_plug_pins(plug.clone(), &ribbon);

        assert!(
            transforms.is_empty(),
            "S4: plug self-emission emits no mesh-CSG ops",
        );

        // Probe at cap_centroid + axis_unit × pin_half_length / 2 =
        // cap_centroid + (0, 0, -1) × 0.002 = (0, 0, -0.056). This
        // lies on the `+axis_unit` (protruding) half of the
        // symmetric-across-cap-plane pyramid (axis = +0.002 < +0.004
        // = half_length, so inside the pyramid axially) and inside
        // the pyramid's main-taper cross-section in lateral terms
        // (Ø > 0 mm at axis = 0.002, which is past the chamfer band
        // ending at axis = -0.004 + 0.0008 = -0.0032 in pin-local
        // coords; main-taper interpolates linearly from 4 mm to
        // 2.8 mm half-extent over axis ∈ [-0.0032, +0.004]).
        let probe_in_lock = Point3::new(0.0, 0.0, -0.056);
        assert!(
            returned_plug.evaluate(&probe_in_lock) < 0.0,
            "returned plug must report interior at a point inside the lock pyramid's \
             `+axis_unit` half (outside bare plug body); SDF = {}",
            returned_plug.evaluate(&probe_in_lock),
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
    /// Iter-1 fixture pose: `center = cap_centroid = (0, 0, -0.054)`,
    /// `axis_unit = -Z` (`cap_normal`), `lateral_unit = +X`
    /// (`split_normal`). Pin-local `-Y` (the chamfer-band base; this
    /// is the `-axis_unit`-end of the symmetric-across-cap-plane
    /// pose — buried inside the plug body under SDF union, and
    /// **not the workshop bed face** under the §G-4 preferred
    /// dome-end-on-bed plug orientation; the chamfer remains a
    /// lead-in self-centering aid here, with the bed-adjacency
    /// reconciliation deferred to S6 procedure.rs) maps to world
    /// `+Z`; lateral pin-local `+X` maps to world `+X`. Chamfer-
    /// band-base axial coord = `-half_length`, world `z = cap_z +
    /// half_length`. Chamfer-top axial coord = `-half_length +
    /// chamfer`, world `z = cap_z + half_length - chamfer`.
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
        // axis_unit = -Z → pin-local +Y maps to world -Z; the
        // chamfer-band-base axial coord (pin-local `-half_length`)
        // maps to `cap_centroid + (-half_length) * (-Z) =
        // cap_centroid + half_length * +Z`.
        let axis_unit = Vector3::new(0.0, 0.0, -1.0);
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
        let axis_unit = Vector3::new(0.0, 0.0, -1.0);
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
