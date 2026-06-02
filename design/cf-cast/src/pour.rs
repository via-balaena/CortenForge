//! Pour-gate + air-vent geometry for v2 multi-piece molds.
//!
//! v2 Step 10 + v2.1 sub-leaves 2/3 placed two cylindrical channels
//! off the v2 mold pieces. Workshop iter-1 visual-pass review
//! (2026-05-21) surfaced two problems with that layout for sock-
//! shaped scans:
//!
//! - The side-mounted pour gate exited on the cup wall's curved
//!   binormal face — no flat surface for a funnel to rest on
//!   during pour.
//! - The apex vent at the centerline's argmax-z vertex landed on
//!   roughly the same dome region as the workshop-natural pour
//!   end, but the gate was somewhere else entirely.
//!
//! v2.1 sub-leaf 4 (this module, 2026-05-21) reshapes the channels
//! into a **V at the dome end** of the centerline — opposite the
//! plug-anchor pin / cap plane. Both channels share the V apex
//! (the centerline endpoint farthest from
//! [`crate::ribbon::Ribbon::pour_end_hint`]'s centroid) and splay
//! apart in the plane containing the local outward-axis + the
//! local segment's `binormal`, with a 30° half-angle. The pour leg
//! lands on the [`crate::ribbon::PieceSide::Positive`] side
//! (`+binormal` half-space); the vent leg lands on the
//! [`crate::ribbon::PieceSide::Negative`] side (`-binormal`
//! half-space). Workshop user orients the assembled mold `+Z` up
//! and sees two distinct holes at the top:
//!
//! - **Pour leg** (`gate_radius_m`, default 5 mm = 10 mm Ø) —
//!   sized for honey-thick workshop silicones (Dragon Skin 10A /
//!   20A at ~20-23k cps) so per-layer pour completes in seconds
//!   without trapping air. Mates with the printable
//!   [`crate::funnel::build_funnel_solid`] funnel.
//! - **Vent leg** (`vent_radius_m`, default 3 mm = 6 mm Ø) — wide
//!   enough to mesh cleanly at cf-cast's default 3 mm cells, narrow
//!   enough to be workshop-visually distinct from the pour leg.
//!
//! ## Composition
//!
//! [`build_pour_gate_transforms`] returns one
//! [`MatingTransform::SubtractCylinder`] per channel — pour leg
//! always, plus the vent leg when `spec.include_vent`. Empty Vec
//! when [`PourGateKind::None`]. Post-S7 the carve lives as post-MC
//! mesh-CSG primitives (bit-precise to f64) rather than as an SDF
//! cylinder unioned into the cup-piece Solid before MC.
//!
//! [`crate::piece::compose_piece_solid`] appends these to BOTH
//! [`crate::ribbon::PieceSide`]s' transform Vec — since the legs
//! lie on opposite sides of the seam (one on `+binormal`, one on
//! `-binormal`), the cup-piece Solid's SDF halfspace intersect
//! (recon-4 (P), see
//! `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2) bounds each
//! piece's mesh to its kept half-shell; the post-MC mesh-CSG
//! subtract of the full cylinder is a no-op for the portion
//! outside the half-shell. Each piece keeps only its own side's
//! half-cylinder carve at the exact mating plane (post-§M-S1
//! flush mating; pre-§M 2026-05-27 had a 0.5 mm seam-overlap slice
//! from the cup-wall's `RIBBON_PIECE_OVERLAP_M`).
//!
//! ## Why splay along binormal, not split-normal
//!
//! The ribbon's seam zero-set is the plane perpendicular to the
//! local `binormal`. Splaying the V along `±binormal` puts the
//! pour leg on `+binormal` (Positive piece) and the vent leg on
//! `-binormal` (Negative piece) — each leg lives entirely on one
//! piece. Splaying along `±split_normal` would keep both legs in
//! the SEAM plane (since `split_normal` is perpendicular to
//! `binormal`), so both legs would straddle the seam and each
//! piece would carve a half-cylinder of each — workshop user can't
//! tell pour from vent at the seam. Binormal splay gives the
//! cleaner per-piece geometry.
//!
//! ## Why opposite the cap plane
//!
//! The plug-anchor pin lives at the cap-plane centroid (per
//! [`crate::ribbon::Ribbon::pour_end_hint`]). The cap plane is the
//! body's natural open end and sits at the BOTTOM of the assembled
//! mold under `+Z` up orientation. Putting the pour channels at the
//! cap-plane end would put the pour openings on the floor of the
//! mold — silicone would pour DOWN through the cup wall and out
//! the bottom rather than INTO the cavity.
//!
//! Anchoring at the OPPOSITE centerline endpoint (the body's closed
//! dome, which sits on top when the mold is `+Z` up) puts the pour
//! openings at the top of the assembly where gravity feeds silicone
//! into the cavity.
//!
//! ## Default off
//!
//! [`Ribbon::new`] sets `pour_gate = PourGateKind::None`. Callers
//! opt into the V-at-dome channels via
//! [`crate::ribbon::Ribbon::with_pour_gate`].
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new

use cf_design::Solid;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};

use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::ribbon::Ribbon;

/// Polygonal facet count around the circumference for the
/// pour-gate cylinder primitives.
///
/// 32 segments at the workshop default 5 mm pour-leg radius gives
/// chord error `r(1 - cos(π/32))` ≈ 24 µm — well below FDM bead
/// width. Pre-S3 / pre-S4 of the FDM-friendly geometry arc this
/// matched parallel `PIN_SEGMENTS` / `PLUG_CYLINDER_SEGMENTS`
/// consts elsewhere (both now retired with their respective SDF-
/// side migrations); the pour-gate carve is the lone surviving
/// mesh-CSG mating-feature surface post-S4 and the only consumer
/// of this const. Part of the determinism contract — same parent
/// + same `segments` → bit-equal output.
pub const POUR_GATE_SEGMENTS: u32 = 32;

/// V half-angle in radians — 30° (= π/6). Each leg of the V splays
/// from the body's outward axis at the dome end by this angle
/// toward `±binormal`; total V opening is 60°.
///
/// Chosen so the two outer-surface punctures are visibly separate
/// on the bounding-region outer surface while keeping each leg
/// close enough to axial that workshop pour ergonomics aren't
/// awkward. Not user-tunable; iter-1 visual gate accepted the
/// default.
pub const V_HALF_ANGLE_RAD: f64 = std::f64::consts::FRAC_PI_6;

/// Pour-channel layout — how the pour (and any modeled vent) are
/// placed on the body's dome end.
///
/// [`Self::VAtDome`] is the iter-1 default (V-shape: pour + vent
/// splayed `±V_HALF_ANGLE_RAD` from a shared apex). [`Self::ApexAxial`]
/// is the organic-parts opt-in (a single AXIAL pour bore at the dome
/// apex, lying IN the seam plane, with NO splay and NO modeled vent).
///
/// **Why `ApexAxial` exists (organic-parts arc, 2026-05-29):** the
/// workshop reasoned through the fill physics for a dome-up mold and
/// landed on pouring at the single HIGHEST point of the cavity (the
/// dome apex). That makes "is it full?" trivial — the cavity is full
/// the instant silicone reaches the pour hole, because there is no
/// point above it — and lets trapped air migrate up to that same
/// point. Placing the bore IN the seam plane (axial → zero binormal
/// component) means it splits the flange: separating the two cup
/// halves bisects the channel lengthwise into open half-troughs, so
/// the cured sprue lifts out instead of having to pull through a blind
/// hole. Air handling moves to tiny hand-drilled carbide vents at the
/// high spots (air's ~1000× lower viscosity escapes a sub-mm hole the
/// viscous silicone won't weep through), so no vent is modeled here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PourGateLayout {
    /// V-shape: pour leg `+binormal` (Positive piece) + optional vent
    /// leg `-binormal` (Negative piece), both splayed
    /// `V_HALF_ANGLE_RAD` from the shared dome apex. iter-1 default.
    #[default]
    VAtDome,
    /// Single axial pour bore at the dome apex, lying in the seam
    /// plane (splits the flange). No splay, no modeled vent
    /// (`include_vent` is ignored). Pairs with the straight-nipple
    /// funnel ([`crate::funnel::build_funnel_solid`]); the seam-placement
    /// solver brackets the bore by construction (it excludes the pour as a
    /// swept channel — see [`crate::bolt_pattern::plan_smart_bolt_placements`]).
    ApexAxial,
}

/// Cylindrical pour-gate + air-vent geometry spec. All dimensions
/// in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PourGateSpec {
    /// Pour-leg channel radius (m). Default `0.005` = 5 mm =
    /// 10 mm diameter.
    ///
    /// Sized for honey-thick workshop silicones (Dragon Skin 10A /
    /// 20A / 30A at ~20-23k cps): 10 mm Ø roughly doubles the
    /// pour-area vs the pre-iter-1 6 mm Ø, dropping per-layer pour
    /// time from minutes to seconds. The matching pour funnel
    /// ([`crate::build_funnel_solid`]) sizes its nipple OD to
    /// `2 × gate_radius_m − NIPPLE_DIAMETRAL_CLEARANCE_M` (asymmetric
    /// `/2` convention — cup pour-gate hole stays at exactly
    /// `2 × gate_radius_m`; funnel side bears all the diametral
    /// clearance).
    pub gate_radius_m: f64,
    /// Vent-leg channel radius (m). Default `0.003` = 3 mm =
    /// 6 mm diameter — smaller than the 10 mm pour leg, but wider
    /// than the pre-V-shape 2 mm vent that workshop iter-1 visually
    /// falsified.
    ///
    /// Sized so the vent meshes cleanly at cf-cast's default 3 mm
    /// mesh-cell size: 6 mm Ø = 1 cell radial, the minimum that MC
    /// resolves without fragmenting a through-hole. The pre-V 2 mm
    /// Ø vent captured as only 0.67 cells radial and MC produced
    /// partial through-holes — workshop iter-1 visual gate falsified
    /// that variant. The pour leg's 10 mm Ø is larger by design (see
    /// [`Self::gate_radius_m`] — honey-thick silicone flow); workshop
    /// user tells pour from vent by binormal side
    /// (`procedure.md` surfaces the +/-binormal mapping), not hole
    /// diameter.
    ///
    /// Trade-off: silicone surface tension can't hold against the
    /// pour-pressure differential on a 6 mm Ø vent as reliably as
    /// the prior 2 mm Ø. Workshop iter-1 accepts that — the pour
    /// leg is the main filler; the vent's role is post-fill air
    /// release as the pour decelerates.
    pub vent_radius_m: f64,
    /// Half-length of the **pour leg** cylinder (m). Total channel
    /// length is `2 * gate_half_length_m`. Default `0.045` = 45 mm
    /// half-length → 90 mm total channel.
    ///
    /// The pour leg anchors at the V apex (centerline endpoint
    /// farthest from [`crate::ribbon::Ribbon::pour_end_hint`])
    /// and extends along the pour-leg axis (outward at apex,
    /// tilted by the module-private `V_HALF_ANGLE_RAD` (30°)
    /// toward `+binormal`, where
    /// `binormal` is the local segment's cached binormal at the
    /// apex endpoint).
    /// For a typical workshop bounding region (cuboid with ≤80 mm
    /// half-extent along the body axis past the dome), the default
    /// 45 mm half-length puts the outer tip past the bounding-
    /// region outer surface — the channel reliably punches all the
    /// way through the cup wall to the outside.
    ///
    /// Workshop users with larger bounding regions tune this up.
    pub gate_half_length_m: f64,
    /// Half-length of the **vent leg** cylinder (m). Total channel
    /// length is `2 * vent_half_length_m`. Default `0.040` = 40 mm
    /// half-length → 80 mm total channel.
    ///
    /// The vent leg anchors at the same V apex as the pour leg and
    /// extends along the vent-leg axis (outward at apex, tilted by
    /// the module-private `V_HALF_ANGLE_RAD` (30°) toward
    /// `-binormal`).
    pub vent_half_length_m: f64,
    /// Whether to include the vent leg. Default `true`. Disable for
    /// short straight molds where air escape back through the pour
    /// leg is sufficient (silicone fills bottom-up; air bubbles
    /// rise out the same channel).
    ///
    /// Ignored when `layout == PourGateLayout::ApexAxial` (that layout
    /// models no vent — the workshop hand-drills tiny carbide vents at
    /// the high spots).
    pub include_vent: bool,
    /// How the pour (and any modeled vent) are placed. Default
    /// [`PourGateLayout::VAtDome`] keeps the iter-1 V-shape (existing
    /// casts byte-identical); [`PourGateLayout::ApexAxial`] is the
    /// organic-parts single-bore-at-the-apex opt-in.
    pub layout: PourGateLayout,
}

impl PourGateSpec {
    /// v2.1 iter-1 defaults: 10 mm Ø pour leg (90 mm total), 6 mm Ø
    /// vent leg (80 mm total), vent enabled. Both legs anchored at
    /// the V apex on the body's dome end (opposite the cap plane),
    /// splayed ±30° in the (outward + binormal) plane.
    ///
    /// Pour leg is bigger than the vent: honey-thick workshop
    /// silicones (Dragon Skin 10A / 20A at ~20-23k cps) pour
    /// tediously through smaller channels and trap air bubbles.
    /// 10 mm Ø roughly doubles the pour-area vs the original
    /// 6 mm Ø, dropping per-layer pour time from minutes to
    /// seconds. Vent stays at 6 mm Ø — it only needs to release
    /// air (no flow-rate constraint) and is workshop-visually
    /// distinct from the larger pour hole. Both Øs mesh cleanly at
    /// cf-cast's default 3 mm cells (10 mm = 1.67 cells radial,
    /// 6 mm = 1 cell radial).
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            gate_radius_m: 0.005,
            vent_radius_m: 0.003,
            gate_half_length_m: 0.045,
            vent_half_length_m: 0.040,
            include_vent: true,
            layout: PourGateLayout::VAtDome,
        }
    }
}

impl Default for PourGateSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// What pour-gate + vent geometry, if any, the [`Ribbon`] uses.
///
/// `None` is the v1/v2-pre-Step-10 default — no integral pour
/// gate or vent; the workshop user pours through the seam where
/// pieces meet and drills vent holes post-print as needed.
/// `Default(PourGateSpec)` enables the V-at-dome channels.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum PourGateKind {
    /// No pour-gate or vent channels.
    #[default]
    None,
    /// V-at-dome pour-leg + (optional) vent-leg per the supplied
    /// [`PourGateSpec`]. Both legs share an apex at the centerline
    /// endpoint opposite the cap plane (per
    /// [`crate::ribbon::Ribbon::pour_end_hint`]) and splay ±30°
    /// in the (outward + `binormal`) plane.
    Default(PourGateSpec),
}

/// Build the cup-side pour-gate mesh-CSG transforms for the
/// ribbon's pour-gate kind.
///
/// Returns an empty Vec when [`PourGateKind::None`] or when the
/// ribbon has no segments (impossible via the public
/// [`Ribbon::new`] constructor).
///
/// Each leg of the V emits one [`MatingTransform::SubtractCylinder`]:
/// the pour leg always, plus the vent leg when `spec.include_vent`.
/// `compose_piece_solid` appends these to both [`crate::ribbon::PieceSide`]
/// transforms; the cup-piece Solid's SDF halfspace intersect
/// (recon-4 (P)) bounds each piece's mesh to its kept half-shell,
/// so the `SubtractCylinder` is effectively bisected at the seam
/// plane by construction — each piece keeps only its own side's
/// half-cylinder carve (pour leg → Positive, vent leg → Negative)
/// per the splay-along-binormal layout.
///
/// V geometry (unchanged from the pre-S7 SDF encoding):
/// - **Apex** at `v_apex_anchor(ribbon)` — the centerline endpoint
///   FARTHER from `ribbon.pour_end_hint`'s centroid, or
///   `centerline[0]` when the hint is unset (cf-scan-prep's
///   tip→base convention puts the dome at `centerline[0]`).
/// - **Outward axis at apex** — the local-segment tangent pointing
///   AWAY from body interior at that endpoint (`-first.tangent` if
///   apex = `centerline[0]`, `+last.tangent` if apex =
///   `centerline.last()`).
/// - **Pour leg axis** — `cos(θ) * outward + sin(θ) * binormal`,
///   normalized. Lands on the [`crate::ribbon::PieceSide::Positive`]
///   side (the `+binormal` side of the ribbon seam).
/// - **Vent leg axis** — `cos(θ) * outward - sin(θ) * binormal`,
///   normalized. Lands on the [`crate::ribbon::PieceSide::Negative`]
///   side.
///
/// Per recon §9 M5 + the asymmetric `/2` clearance convention, the
/// cup pour-gate hole stays at exactly `spec.gate_radius_m` (no
/// inflation) — the funnel side bears all the diametral slack via
/// [`crate::funnel::NIPPLE_DIAMETRAL_CLEARANCE_M`]. The vent leg
/// has no funnel-side counterpart and uses `spec.vent_radius_m`
/// verbatim.
///
/// [`Ribbon::new`]: crate::ribbon::Ribbon::new
#[must_use]
pub fn build_pour_gate_transforms(ribbon: &Ribbon) -> Vec<MatingTransform> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return Vec::new(),
        PourGateKind::Default(spec) => spec,
    };

    // Apex-axial: a single bore leg on the seam. This footprint feeds the
    // seam-placement solver's pour exclusion (the bolts bracket it — see
    // `crate::seam_placement::build_layer_loops`). The CUP CARVE for this
    // layout is NOT this post-MC cylinder, though — it's the integral split
    // funnel + bore fused into the cup SDF
    // ([`build_integral_pour_channel`]); `crate::piece::compose_piece_shared`
    // drops this leg from the post-MC carve set for apex-axial so the bore
    // isn't carved twice.
    if spec.layout == PourGateLayout::ApexAxial {
        // The exclusion footprint is the funnel BASE radius (+ comfort gap),
        // NOT the bore — so the apex-bracketing fasteners clear the funnel
        // cone, not just the narrower bore. (The SDF carve in
        // `build_integral_pour_channel` still uses the true bore radius.)
        let exclusion_r = integral_funnel_exclusion_radius(spec.gate_radius_m);
        return apex_axial_pose(ribbon)
            .map(|(apex, axis)| {
                vec![leg_transform(
                    apex,
                    axis,
                    exclusion_r,
                    spec.gate_half_length_m,
                )]
            })
            .unwrap_or_default();
    }

    let Some((apex, outward, binormal)) = v_apex_anchor(ribbon) else {
        return Vec::new();
    };

    let cos = V_HALF_ANGLE_RAD.cos();
    let sin = V_HALF_ANGLE_RAD.sin();
    let pour_leg_axis = (cos * outward + sin * binormal).normalize();

    let mut transforms = Vec::with_capacity(if spec.include_vent { 2 } else { 1 });
    transforms.push(leg_transform(
        apex,
        pour_leg_axis,
        spec.gate_radius_m,
        spec.gate_half_length_m,
    ));

    if spec.include_vent {
        let vent_leg_axis = (cos * outward - sin * binormal).normalize();
        transforms.push(leg_transform(
            apex,
            vent_leg_axis,
            spec.vent_radius_m,
            spec.vent_half_length_m,
        ));
    }
    transforms
}

/// Funnel-wall thickness for the integral apex split funnel (m).
///
/// 2.5 mm = 5 cells at the 0.5 mm production cup cell — a robust
/// printable half-shell wall.
pub(crate) const INTEGRAL_FUNNEL_WALL_M: f64 = 0.0025;

/// Integral-funnel mouth INNER radius as a multiple of the bore radius.
///
/// 2.2 → mouth Ø = 2.2 × bore Ø (recon §7.8, balanced catch vs.
/// cone-stiffness). The pour throat stays the bore Ø (the funnel only
/// widens above it). `pub(crate)` so `crate::procedure` can cite the
/// mouth Ø in the workshop prose.
pub(crate) const INTEGRAL_FUNNEL_MOUTH_FACTOR: f64 = 2.2;

/// Integral-funnel protrusion height above the cup outer surface (m).
///
/// 18 mm (recon §7.8) — short + cone-stiff so the bore-bracketing bolt
/// clamp at the base dominates and the cantilevered mouth can't splay.
/// `pub(crate)` so `crate::procedure` can cite the height.
pub(crate) const INTEGRAL_FUNNEL_HEIGHT_M: f64 = 0.018;

/// Comfort gap between a bracketing fastener's footprint and the funnel
/// base wall (m). The seam-placement pour-exclusion is the funnel BASE
/// radius (bore + wall) plus this margin, so the apex-bracketing dowel +
/// bolt clear the funnel cone — not just the bore. Without it they sit
/// against the funnel (the bore is 2.5 mm narrower than the funnel base).
const INTEGRAL_FUNNEL_FASTENER_CLEARANCE_M: f64 = 0.002;

/// Seam-placement pour-exclusion radius for the integral apex funnel: the
/// funnel BASE outer radius (`bore + wall`) plus the fastener comfort gap.
/// Fasteners bracket THIS (not the bore Ø) so they clear the funnel cone.
/// The SDF carve still uses the true `bore_radius_m`.
pub(crate) fn integral_funnel_exclusion_radius(bore_radius_m: f64) -> f64 {
    bore_radius_m + INTEGRAL_FUNNEL_WALL_M + INTEGRAL_FUNNEL_FASTENER_CLEARANCE_M
}

/// Resolve the apex-axial pour pose: `(apex_on_seam, axis)`.
///
/// - the apex point is projected onto the ribbon's seam plane (the
///   planar-seam plane when set, else the arc-midpoint reference), so
///   the bore's centerline sits exactly on the seam, and
/// - the outward axis has its seam-normal component removed, so the
///   bore lies entirely IN the seam plane.
///
/// A bore on the seam straddles it → separating the two cup halves
/// bisects it lengthwise into open half-troughs (easy sprue removal)
/// and the two half-bores re-register into one round hole on assembly
/// (the planar seam guarantees coplanar faces). Returns `None` when the
/// ribbon has no apex anchor (no segments).
fn apex_axial_pose(ribbon: &Ribbon) -> Option<(Point3<f64>, Vector3<f64>)> {
    let (apex, outward, _binormal) = v_apex_anchor(ribbon)?;
    let (seam_mid, seam_normal) = ribbon.seam_plane_reference();
    let n = seam_normal.into_inner();
    let apex_on_seam = apex - (apex - seam_mid).dot(&n) * n;
    let axis_in_seam = outward - outward.dot(&n) * n;
    let axis = if axis_in_seam.norm() > 1.0e-9 {
        axis_in_seam.normalize()
    } else {
        // Degenerate: outward parallel to the seam normal (the body
        // axis lies in the split direction). Fall back to the raw
        // outward axis — the bore won't split evenly, but this is a
        // pathological split-normal choice the 2-piece gate rejects.
        outward.normalize()
    };
    Some((apex_on_seam, axis))
}

/// The integral split funnel + pour bore for the [`PourGateLayout::ApexAxial`]
/// layout, as SDF [`Solid`]s the cup composer fuses into each half.
///
/// The funnel is part of the cup print (recon §7.8): a cone rising from
/// the apex along the in-seam bore axis, split by the seam halfspace
/// intersect (its axis lies IN the seam plane), with its lumen
/// continuous into the bore — no separate inserted nipple, so the throat
/// is the bore Ø itself (no nipple-wall constriction).
pub(crate) struct IntegralPourChannel {
    /// Funnel outer cone (positive material) to UNION into the cup half.
    /// The caller intersects it with the side halfspace first so each
    /// piece keeps its own half-cone. `None` when the surface ray-march
    /// failed (apex not interior / no exit) — a graceful degrade to a
    /// bore-only channel (still pourable, just no integral catch).
    pub funnel_cone: Option<Solid>,
    /// Full pour lumen (straight bore ∪ funnel taper) to SUBTRACT from the
    /// cup half. Carves the bore through the cup wall into the cavity AND
    /// the widened funnel lumen. Subtracting the full (un-halved) lumen
    /// from a half-piece is side-safe (only removes material that exists).
    pub lumen: Solid,
}

/// Build the integral split funnel + bore for the apex-axial layout, sized
/// to `bore_radius_m`.
///
/// Returns `None` when the ribbon's pour gate is not an apex-axial
/// `Default` spec, or when the apex anchor is unresolved. The funnel is
/// anchored at the cup outer surface via a ray-march along the bore axis
/// (`crate::piece` supplies `layer_body`); if that march can't find the
/// surface the channel degrades to bore-only (`funnel_cone = None`).
#[must_use]
pub(crate) fn build_integral_pour_channel(
    ribbon: &Ribbon,
    layer_body: &Solid,
    bore_radius_m: f64,
) -> Option<IntegralPourChannel> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };
    if spec.layout != PourGateLayout::ApexAxial {
        return None;
    }
    let (apex, axis) = apex_axial_pose(ribbon)?;

    // Straight bore: same geometry as the retired post-MC apex leg — a
    // cylinder from the apex (cavity-side inner end) out along the axis by
    // 2 × gate_half_length, so it breaches the cavity and pierces the wall
    // to outside. Carves the bore portion of the lumen.
    let straight_bore = axis_cylinder(apex, axis, bore_radius_m, spec.gate_half_length_m);

    // Ray-march the body surface along the axis to anchor the funnel at the
    // cup outer surface. Failure (apex not interior / no exit within the body
    // diagonal) → bore-only degrade.
    let Some(s_surf) = march_surface_along_axis(layer_body, apex, axis) else {
        return Some(IntegralPourChannel {
            funnel_cone: None,
            lumen: straight_bore,
        });
    };

    let wall = INTEGRAL_FUNNEL_WALL_M;
    let height = INTEGRAL_FUNNEL_HEIGHT_M;
    let mouth_inner = bore_radius_m * INTEGRAL_FUNNEL_MOUTH_FACTOR;
    let base_outer = bore_radius_m + wall;
    let mouth_outer = mouth_inner + wall;
    // Tip overshoot so MC cuts the mouth rim cleanly (mirrors the funnel
    // module's `INNER_CAVITY_OVERSHOOT_M`).
    let overshoot = 0.0005_f64;

    // Build the cone + taper in a local +Z frame where local z = arc-length
    // `s` from `apex` along `axis`, then rotate +Z → axis and translate to
    // `apex`. (`s = 0` at the apex, increasing outward.)
    let to_axis = align_z_to(axis);
    let place = |s: Solid| s.rotate(to_axis).translate(apex.coords);

    // Funnel outer cone: narrow base (bore_r + wall) at the surface, widening
    // to the mouth. Its lower band [s_surf, s_surf + wall] overlaps the cup
    // shell → welds; the rest protrudes.
    let funnel_cone = place(crate::funnel::truncated_cone(
        base_outer,
        mouth_outer,
        s_surf,
        s_surf + height,
    ));
    // Funnel taper lumen: bore_r at the surface widening to mouth_inner, with
    // tip overshoot. Unioned with the straight bore → the lumen is bore_r
    // through the wall/cavity and widens through the funnel.
    let funnel_taper = place(crate::funnel::truncated_cone(
        bore_radius_m,
        mouth_inner,
        s_surf,
        s_surf + height + overshoot,
    ));
    let lumen = straight_bore.union(funnel_taper);

    Some(IntegralPourChannel {
        funnel_cone: Some(funnel_cone),
        lumen,
    })
}

/// A cylinder of `radius_m`, `2 × half_length_m` long, with its inner face
/// at `apex` extending along unit `axis` (matches the retired
/// `leg_transform` bore geometry).
fn axis_cylinder(
    apex: Point3<f64>,
    axis: Vector3<f64>,
    radius_m: f64,
    half_length_m: f64,
) -> Solid {
    let center = apex + axis * half_length_m;
    Solid::cylinder(radius_m, half_length_m)
        .rotate(align_z_to(axis))
        .translate(center.coords)
}

/// Unit quaternion rotating local `+Z` onto unit `axis`. Falls back to a
/// 180° flip about `+X` for the antiparallel (`axis = −Z`) degenerate case
/// where `rotation_between` is undefined.
fn align_z_to(axis: Vector3<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::rotation_between(&Vector3::z(), &axis).unwrap_or_else(|| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    })
}

/// March from `start` along unit `axis` until the body surface exits
/// (`layer_body.evaluate` crosses 0⁺), returning the arc-length to the
/// surface. `None` when `start` is not interior, or no exit is found within
/// the body's AABB diagonal (a runaway / tangent ray). Mirrors the
/// `crate::piece` cavity-floor cross-section march.
// The 0.5 mm march casts a small bounded step count (`r_max ≤ 0.15 m` /
// 0.5 mm ≈ 300) and the loop index to f64 — all exact / in-range, so the
// `ceil() as usize` truncation, `as f64` precision, and sign casts carry no
// meaningful loss (`r_max` and the index are non-negative by construction).
// Same justification as the cavity-floor march in `crate::piece`.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn march_surface_along_axis(
    layer_body: &Solid,
    start: Point3<f64>,
    axis: Vector3<f64>,
) -> Option<f64> {
    const STEP_M: f64 = 0.0005;
    if layer_body.evaluate(&start) >= 0.0 {
        return None;
    }
    let r_max = layer_body
        .bounds()
        .map_or(0.150_f64, |b| (b.max - b.min).norm())
        .min(0.150_f64);
    let max_steps = (r_max / STEP_M).ceil() as usize;
    for s in 1..=max_steps {
        let dist = STEP_M * (s as f64);
        if layer_body.evaluate(&(start + axis * dist)) > 0.0 {
            return Some(dist);
        }
    }
    None
}

/// Resolve the V apex `(point, outward_axis, binormal)` for this
/// ribbon — the centerline endpoint opposite the
/// [`crate::ribbon::Ribbon::pour_end_hint`]'s centroid, with the
/// local-segment tangent pointing outward (away from body interior)
/// and the local-segment cached binormal threaded through so the
/// V's leg splay aligns with the ribbon seam at the apex.
///
/// Hint resolution mirrors `crate::plug::build_plug_pin_solid`'s
/// pour/dome split: when a hint is set, the V apex is the centerline
/// endpoint FARTHER from the hint centroid (the hint marks the cap
/// plane / plug-anchor side; the V apex is on the opposite end).
/// When no hint is set, the apex falls back to `centerline[0]`
/// (per cf-scan-prep's tip→base convention, `centerline[0]` is the
/// closed dome).
///
/// The returned `binormal` is the segment's cached binormal at the
/// apex endpoint: `first.binormal` if apex = `first.start`,
/// `last.binormal` if apex = `last.end`. Both are unit-length per
/// `Ribbon::new`'s invariant.
fn v_apex_anchor(ribbon: &Ribbon) -> Option<(Point3<f64>, Vector3<f64>, Vector3<f64>)> {
    let first = ribbon.segments.first()?;
    let last = ribbon.segments.last()?;
    let start_anchor = (first.start, -first.tangent, first.binormal);
    let end_anchor = (last.end, last.tangent, last.binormal);
    match ribbon.pour_end_hint {
        Some((centroid, _)) => {
            let d_start = (centroid - first.start).norm_squared();
            let d_end = (centroid - last.end).norm_squared();
            if d_start > d_end {
                Some(start_anchor)
            } else {
                Some(end_anchor)
            }
        }
        // No hint: fall back to `centerline[0]` per cf-scan-prep's
        // tip→base convention (closed dome lives at `centerline[0]`).
        None => Some(start_anchor),
    }
}

/// Build a single leg's [`MatingTransform::SubtractCylinder`] —
/// inner tip at `apex`, extending outward along `axis` by
/// `2 × half_length`. The unit `axis` must already be normalized.
fn leg_transform(
    apex: Point3<f64>,
    axis: Vector3<f64>,
    radius_m: f64,
    half_length_m: f64,
) -> MatingTransform {
    let center_m = apex + axis * half_length_m;
    let parent = CylinderParent {
        center_m,
        axis: Unit::new_normalize(axis),
        half_length_m,
    };
    MatingTransform::SubtractCylinder {
        params: CylinderParams {
            parent,
            radius_m,
            segments: POUR_GATE_SEGMENTS,
        },
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]

    use super::*;
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn straight_x_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split).unwrap()
    }

    #[test]
    fn pour_gate_spec_iter1_has_workshop_defaults() {
        let s = PourGateSpec::iter1();
        assert!(
            (s.gate_radius_m - 0.005).abs() < f64::EPSILON,
            "gate_radius_m should be 5 mm (10 mm Ø) for honey-thick \
             silicone flow",
        );
        assert!(
            (s.vent_radius_m - 0.003).abs() < f64::EPSILON,
            "vent_radius_m should be 3 mm (6 mm Ø) — smaller than \
             pour gate; visually distinct + meshes cleanly at 3 mm cells",
        );
        assert!((s.gate_half_length_m - 0.045).abs() < f64::EPSILON);
        assert!((s.vent_half_length_m - 0.040).abs() < f64::EPSILON);
        assert!(s.include_vent);
    }

    #[test]
    fn pour_gate_kind_default_is_none() {
        assert_eq!(PourGateKind::default(), PourGateKind::None);
    }

    /// Extract the inner-tip apex point of a `SubtractCylinder`
    /// leg (= `center − axis × half_length`, i.e., the cylinder
    /// face nearest the V apex).
    fn leg_apex(t: &MatingTransform) -> Point3<f64> {
        match t {
            MatingTransform::SubtractCylinder { params } => {
                let p = &params.parent;
                p.center_m - p.axis.into_inner() * p.half_length_m
            }
            other => panic!("expected SubtractCylinder; got {other:?}"),
        }
    }

    /// Extract the outward tip point of a `SubtractCylinder` leg
    /// (= `center + axis × half_length`, i.e., the cylinder face
    /// farthest from the V apex — where the carve emerges from the
    /// cup outer surface).
    fn leg_outward_tip(t: &MatingTransform) -> Point3<f64> {
        match t {
            MatingTransform::SubtractCylinder { params } => {
                let p = &params.parent;
                p.center_m + p.axis.into_inner() * p.half_length_m
            }
            other => panic!("expected SubtractCylinder; got {other:?}"),
        }
    }

    #[test]
    fn build_pour_gate_transforms_none_returns_empty() {
        let ribbon = straight_x_ribbon(); // default pour_gate = None
        assert!(build_pour_gate_transforms(&ribbon).is_empty());
    }

    /// Straight +X centerline with +Y split-normal → binormal +Z.
    /// With no hint set, V apex falls back to
    /// `centerline[0] = (-0.050, 0, 0)` with outward axis =
    /// `-first.tangent = -X`. Pour leg tilted +30° toward
    /// `+binormal = +Z` from `-X` (so pour leg axis
    /// ≈ `(-cos30°, 0, +sin30°)` = `(-0.866, 0, +0.5)`); vent leg
    /// tilted -30° toward `-binormal = -Z` (so vent axis
    /// ≈ `(-0.866, 0, -0.5)`). Each leg's inner tip is at the
    /// shared apex; outward tip extends `2 * half_length` from apex.
    #[test]
    fn build_pour_gate_transforms_default_emits_pour_and_vent() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let transforms = build_pour_gate_transforms(&ribbon);
        assert_eq!(transforms.len(), 2, "expect [pour, vent] SubtractCylinders");

        // Pour leg radius = nominal `gate_radius_m` (cup side stays
        // nominal per the asymmetric `/2` convention; funnel bears
        // all M5 diametral slack).
        let spec = PourGateSpec::iter1();
        if let MatingTransform::SubtractCylinder { params } = &transforms[0] {
            assert!((params.radius_m - spec.gate_radius_m).abs() < f64::EPSILON);
            assert!((params.parent.half_length_m - spec.gate_half_length_m).abs() < f64::EPSILON);
            assert_eq!(params.segments, POUR_GATE_SEGMENTS);
        } else {
            panic!(
                "transforms[0] expected SubtractCylinder; got {:?}",
                transforms[0]
            );
        }
        // Vent leg radius = `vent_radius_m`; no funnel-side counterpart.
        if let MatingTransform::SubtractCylinder { params } = &transforms[1] {
            assert!((params.radius_m - spec.vent_radius_m).abs() < f64::EPSILON);
            assert!((params.parent.half_length_m - spec.vent_half_length_m).abs() < f64::EPSILON);
        } else {
            panic!(
                "transforms[1] expected SubtractCylinder; got {:?}",
                transforms[1]
            );
        }

        // Both legs share the V apex point at (-0.050, 0, 0).
        let apex = Point3::new(-0.050, 0.0, 0.0);
        for t in &transforms {
            let inner = leg_apex(t);
            assert!(
                (inner - apex).norm() < 1.0e-9,
                "leg inner tip should sit at apex; got {inner:?}"
            );
        }

        // Pour leg's outward tip lands at apex + 90 mm × (-cos30°, 0, +sin30°)
        // ≈ (-0.128, 0, +0.045). Vent leg's outward tip lands at
        // apex + 80 mm × (-cos30°, 0, -sin30°) ≈ (-0.119, 0, -0.040).
        let pour_tip = leg_outward_tip(&transforms[0]);
        assert!(
            pour_tip.x < -0.120,
            "pour leg should reach x ≤ -120 mm; got {}",
            pour_tip.x
        );
        assert!(
            pour_tip.z > 0.040,
            "pour leg should reach z ≥ +40 mm; got {}",
            pour_tip.z
        );
        let vent_tip = leg_outward_tip(&transforms[1]);
        assert!(
            vent_tip.z < -0.035,
            "vent leg should reach z ≤ -35 mm; got {}",
            vent_tip.z
        );
    }

    /// With vent disabled, only the pour leg remains.
    #[test]
    fn build_pour_gate_transforms_no_vent_omits_vent_leg() {
        let mut spec_no_vent = PourGateSpec::iter1();
        spec_no_vent.include_vent = false;
        let ribbon = straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec_no_vent));
        let transforms = build_pour_gate_transforms(&ribbon);
        assert_eq!(transforms.len(), 1, "no-vent should emit only the pour leg");
        // No -binormal extent (no vent leg).
        let tip = leg_outward_tip(&transforms[0]);
        assert!(
            tip.z > 0.040,
            "remaining leg should be the +binormal pour leg"
        );
    }

    /// V apex sits at the centerline endpoint opposite the
    /// `pour_end_hint`. Set the hint near `centerline.last()` to
    /// confirm the apex flips to `centerline[0]`; set near
    /// `centerline[0]` to confirm the apex flips to
    /// `centerline.last()`.
    #[test]
    fn v_apex_flips_to_endpoint_opposite_pour_end_hint() {
        // Hint near `centerline.last()` (= (+0.050, 0, 0)) → V apex
        // at `centerline[0]` (= (-0.050, 0, 0)).
        let ribbon = straight_x_ribbon()
            .with_pour_end_hint(Point3::new(0.060, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let transforms = build_pour_gate_transforms(&ribbon);
        let apex = leg_apex(&transforms[0]);
        assert!(
            (apex.x - -0.050).abs() < 1.0e-9,
            "apex should be at centerline[0]; got x={}",
            apex.x
        );

        // Hint near `centerline[0]` → V apex at `centerline.last()`.
        let ribbon = straight_x_ribbon()
            .with_pour_end_hint(Point3::new(-0.060, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0))
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let transforms = build_pour_gate_transforms(&ribbon);
        let apex = leg_apex(&transforms[0]);
        assert!(
            (apex.x - 0.050).abs() < 1.0e-9,
            "apex should be at centerline.last(); got x={}",
            apex.x
        );
    }

    /// `ApexAxial` layout: one leg only (no vent), inner tip at the
    /// dome apex, axis lying IN the seam plane (zero binormal
    /// component). Straight +X centerline, +Y split → binormal +Z;
    /// the apex bore must have no +Z (binormal) component so it splits
    /// the flange symmetrically.
    #[test]
    fn build_pour_gate_transforms_apex_axial_emits_single_seam_aligned_leg() {
        let mut spec = PourGateSpec::iter1();
        spec.layout = PourGateLayout::ApexAxial;
        let ribbon = straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec.clone()));
        let transforms = build_pour_gate_transforms(&ribbon);
        assert_eq!(
            transforms.len(),
            1,
            "ApexAxial emits a single pour leg (no vent); got {}",
            transforms.len()
        );

        // Apex at centerline[0] = (-0.050, 0, 0); on the seam plane
        // (y = 0). Inner tip must sit there.
        let inner = leg_apex(&transforms[0]);
        assert!(
            (inner - Point3::new(-0.050, 0.0, 0.0)).norm() < 1.0e-9,
            "apex-axial inner tip should sit at the dome apex on the seam; got {inner:?}"
        );

        // Axis must lie in the seam plane: for +Y split the seam
        // normal is the binormal +Z, so the axis must have ~zero Z.
        let (_, seam_normal) = ribbon.seam_plane_reference();
        if let MatingTransform::SubtractCylinder { params } = &transforms[0] {
            let axis = params.parent.axis.into_inner();
            let n = seam_normal.into_inner();
            assert!(
                axis.dot(&n).abs() < 1.0e-9,
                "apex-axial bore must lie in the seam plane (zero seam-normal \
                 component); got axis·n = {}",
                axis.dot(&n)
            );
            // Axis points outward (−X for the centerline[0] dome end).
            assert!(
                axis.x < -0.99,
                "apex-axial bore should point outward along −X; got {axis:?}"
            );
            // The exclusion footprint is the funnel BASE radius (+ comfort
            // gap), not the bore radius — so apex-bracketing fasteners clear
            // the funnel cone. (The SDF carve uses the true bore radius.)
            let expect_r = integral_funnel_exclusion_radius(spec.gate_radius_m);
            assert!(
                (params.radius_m - expect_r).abs() < f64::EPSILON,
                "apex-axial exclusion radius should be the funnel base ({expect_r} m); \
                 got {}",
                params.radius_m
            );
            assert!(
                params.radius_m > spec.gate_radius_m,
                "exclusion must be wider than the bore so fasteners clear the funnel"
            );
        } else {
            panic!("expected SubtractCylinder; got {:?}", transforms[0]);
        }
    }

    /// The integral split funnel (apex-axial) builds a funnel cone that
    /// PROTRUDES past the cup outer surface. Body = cuboid half-0.06 centred
    /// at origin; apex at (−0.05, 0, 0), axis −X → the surface is at x=−0.06
    /// (`s_surf` ≈ 0.01), so the 18 mm funnel reaches ≈ x = −0.078.
    #[test]
    fn integral_pour_channel_funnel_protrudes_past_surface() {
        let mut spec = PourGateSpec::iter1();
        spec.layout = PourGateLayout::ApexAxial;
        let ribbon = straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec));
        let body = Solid::cuboid(Vector3::new(0.06, 0.06, 0.06));

        let channel =
            build_integral_pour_channel(&ribbon, &body, 0.005).expect("apex-axial → Some");
        let cone = channel
            .funnel_cone
            .expect("surface ray-march succeeds → funnel cone present");
        let b = cone.bounds().expect("funnel cone is bounded");
        assert!(
            b.min.x < -0.070,
            "funnel should protrude past the body surface (x=−0.06) along −X; \
             got min.x={}",
            b.min.x
        );
        // Lumen exists (bore ∪ taper) and is bounded.
        assert!(channel.lumen.bounds().is_some());
    }

    /// V-at-dome and no-pour-gate ribbons get no integral channel (the
    /// builder is apex-axial-only).
    #[test]
    fn integral_pour_channel_none_for_non_apex_axial() {
        let body = Solid::cuboid(Vector3::new(0.06, 0.06, 0.06));
        let v_ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        assert!(build_integral_pour_channel(&v_ribbon, &body, 0.005).is_none());
        let none_ribbon = straight_x_ribbon();
        assert!(build_integral_pour_channel(&none_ribbon, &body, 0.005).is_none());
    }

    /// When the apex is not interior to the body (ray-march can't anchor the
    /// surface), the channel degrades to bore-only: lumen present, no funnel
    /// cone. Body = cuboid half-0.04 → apex (−0.05) is OUTSIDE (|x|>0.04).
    #[test]
    fn integral_pour_channel_degrades_to_bore_only_when_apex_outside() {
        let mut spec = PourGateSpec::iter1();
        spec.layout = PourGateLayout::ApexAxial;
        let ribbon = straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec));
        let body = Solid::cuboid(Vector3::new(0.04, 0.04, 0.04));

        let channel =
            build_integral_pour_channel(&ribbon, &body, 0.005).expect("apex-axial → Some");
        assert!(
            channel.funnel_cone.is_none(),
            "apex outside body → no surface anchor → bore-only degrade"
        );
        assert!(channel.lumen.bounds().is_some(), "bore lumen still present");
    }

    /// Iter-1-shaped fixture: trimmed centerline (tip at z=+0.073,
    /// trimmed end at z=-0.013), cap centroid 40 mm past trimmed
    /// end at z=-0.053. V apex must land at `centerline[0]` (the
    /// closed dome at z=+0.073), NOT at `centerline.last()` (the
    /// trimmed body interior).
    #[test]
    fn v_apex_at_dome_for_iter1_trimmed_centerline_with_cap_hint() {
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073), // closed dome
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.053);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let transforms = build_pour_gate_transforms(&ribbon);
        assert_eq!(transforms.len(), 2);
        let apex = leg_apex(&transforms[0]);
        assert!(
            (apex.z - 0.073).abs() < 1.0e-9,
            "V apex must land at the closed dome z=+0.073; got z={}",
            apex.z,
        );
        // Outward axis = -first.tangent ≈ +Z (first.tangent goes
        // from z=+0.073 to z=+0.020 = -Z). Pour-leg + vent-leg both
        // extend roughly +Z from apex with ±split offsets.
        let pour_tip = leg_outward_tip(&transforms[0]);
        let vent_tip = leg_outward_tip(&transforms[1]);
        assert!(pour_tip.z > 0.10, "pour leg should reach high +Z");
        assert!(vent_tip.z > 0.10, "vent leg should reach high +Z");
    }
}
