//! Axial plug-anchor pins for v2.1 multi-piece molds.
//!
//! v2 iter-1 visual-pass surfaced a workshop pain point: the printed
//! plug has no mechanical attachment to the assembled mold pieces.
//! Workshop user would have to hand-hold the plug at the correct
//! position during pour + cure. v2.1 closes the "plug floats" gap by
//! adding an **axial pin** at the pour end of the centerline,
//! matched by a **cylindrical socket** subtracted from each mold
//! piece. The pin + socket co-locate with the pour-gate channel
//! (same end of the centerline), so no NEW through-hole is punched
//! through the cup wall — the pour-gate exit IS the socket entry.
//!
//! The dome end (the non-pour `centerline.last()` end) ships
//! **without a pin** by default: the plug's hemispherical cap seats
//! naturally into the matching dome of the body cavity (same
//! [`cf_design::Solid`] SDF on both sides), giving geometric
//! centering with no through-hole at the dome and no leak path.
//! Workshop users with high-curvature centerlines that need
//! dome-end rotation constraint opt in via
//! [`PlugPinSpec::include_dome_pin`].
//!
//! See [`crate::registration`] for the parallel inter-piece pin
//! mechanism (Step 9 of the v2 arc) that locks the two mold pieces
//! together along the ribbon seam — that mechanism is orthogonal and
//! co-exists with the plug-anchor pins added here.
//!
//! # Geometry
//!
//! - The pour-end pin shaft is centered at the **cap-plane centroid**
//!   passed via [`crate::ribbon::Ribbon::pour_end_hint`]
//!   `(centroid, outward_axis)` and extends along `outward_axis`
//!   (away from the body interior). cf-cast-cli's derive path
//!   threads the cap-plane data from `.prep.toml [caps]` here;
//!   callers without cap-plane data either pass explicit anchor +
//!   outward direction or leave unset and rely on the fallback to
//!   `(points.last(), last_segment.tangent)` per cf-scan-prep's
//!   tip→base convention.
//! - **Why the cap plane and not the centerline tip**: cf-scan-prep
//!   trims the centerline `trim_floor_mm` (typically 40 mm) above
//!   the cap plane to keep the polyline inside the body interior.
//!   `centerline.last()` therefore lives 40 mm INSIDE the plug
//!   body, where a pin of typical length is entirely buried (no
//!   visible protrusion + no socket carve in the cup wall). The
//!   cap plane is where the plug body's pinned floor sits, so a
//!   pin anchored there straddles the plug surface and the cup
//!   wall correctly.
//! - The optional dome-end pin (enabled via
//!   [`PlugPinSpec::include_dome_pin`]) sits at the centerline
//!   endpoint **farthest** from the pour anchor, extending
//!   outward along that endpoint's local segment tangent.
//! - Cup-piece sockets are the **same cylinder geometry** as the
//!   plug's pin/T-bar, inflated diametrically by
//!   [`PlugPinSpec::shaft_diametral_clearance_m`] /
//!   [`PlugPinSpec::t_bar_diametral_clearance_m`] (symmetric `/2`
//!   convention — half the budget on each radial side) and axially
//!   by [`PlugPinSpec::shaft_axial_clearance_m`] /
//!   [`PlugPinSpec::t_bar_axial_clearance_m`] (symmetric `/2`
//!   convention on each axial end). The plug-side and cup-side
//!   cylinder primitives share the same [`CylinderParent`] triple
//!   (`center`, `axis`, `half_length`) modulo the `+ axial/2`
//!   inflation on the cup side, so the workshop fit is exact by
//!   construction (S6 of [[mating-features arc]]).
//!
//! # Mesh-CSG migration (S6)
//!
//! S6 of `docs/CF_CAST_MATING_FEATURES_PLAN.md` migrated this path
//! from SDF [`Solid::union`] / [`Solid::subtract`] (MC-resolved at
//! marching-cubes grid resolution) to post-MC mesh-CSG primitives.
//! - [`build_plug_self_transforms`] emits one
//!   [`MatingTransform::UnionCylinder`] per pin-shaft + T-bar
//!   feature for the **plug** STL (consumed by
//!   [`add_plug_pins`]).
//! - [`build_plug_socket_transforms`] emits one
//!   [`MatingTransform::SubtractCylinder`] per pin-socket + T-slot
//!   feature for the **cup pieces** (consumed by
//!   [`crate::piece::compose_piece_solid`]; side-agnostic — both
//!   cup halves emit the same Vec, and each piece's SDF halfspace
//!   intersect bounds the subtract to its kept half-shell).
//!
//! The cup-piece T-slot lands as a half-disk on each cup half's
//! mating face: the T-bar's axis is parallel to the seam-plane
//! normal (= the ribbon binormal), the cup-piece Solid is the
//! halfspace-intersected half-shell (recon-4 (P) — see
//! `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2), and the
//! post-MC mesh-CSG subtract punches the cylinder cross-section
//! through the half-shell's seam face. The workshop user closes
//! the second cup half around the plug's T-bar (captive
//! insertion).
//!
//! # Multi-layer trade-off
//!
//! For an N-layer cast the plug stays the same; each layer's mold
//! pieces all get the same socket cylinder subtraction. A 20 mm pin
//! engages a layer's cup-wall socket only where the cup wall is
//! within `pin_length` of the centerline endpoint — outer layers
//! with thicker walls get shorter effective engagement. Workshop
//! users who need full engagement on every layer of a thick-wall
//! cast tune [`PlugPinSpec::pin_length_m`] up; expect a longer
//! axial pin-channel through the cured silicone shells at the cap
//! end as the trade-off. The [`crate::write_procedure_v2`]-generated
//! Markdown surfaces this for the workshop user.
//!
//! [`crate::write_procedure_v2`]: crate::CastSpec::write_procedure_v2
//!
//! # Default off
//!
//! [`Ribbon::new`] sets `plug_pins = PlugPinKind::None` for
//! backward compatibility with the v2 Steps 5-10 arc. Workshop
//! iter-1 callers opt into [`PlugPinKind::Axial`] via
//! [`Ribbon::with_plug_pins`] and let
//! [`crate::CastSpec::export_molds_v2`] handle the rest —
//! `export_molds_v2` derives each layer's plug solid from
//! [`crate::CastSpec::plug`] (layer 0) or `layers[N-1].body`
//! (layer N>0) and internally calls [`add_plug_pins`] to attach
//! the pin geometry as `MatingTransform`s; the cup-piece
//! [`crate::piece::compose_piece_solid`] consults the same
//! [`crate::ribbon::Ribbon::plug_pins`] field to emit the matching
//! socket transforms. Single source of truth — sockets-without-pins
//! (jams the plug) and pins-without-sockets (prevents mold close)
//! can't drift apart.
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new
//! [`Ribbon::with_plug_pins`]: crate::ribbon::Ribbon::with_plug_pins
//! [`CastSpec`]: crate::CastSpec
//! [`CylinderParent`]: crate::mesh_csg::CylinderParent

use cf_design::Solid;
use nalgebra::{Point3, UnitVector3, Vector3};

use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::ribbon::Ribbon;

/// Polygonal facet count around the plug shaft + T-bar cylinder
/// circumference.
///
/// 32-segment circles match [`crate::registration::PIN_SEGMENTS`]
/// at 3 mm Ø: chord error `r(1 - cos(π/32))` ≈ 7 µm at radius
/// 1.5 mm, well below FDM bead width. The plug's 6 mm Ø
/// shaft + T-bar (`PlugPinSpec::iter1` default) inherit the same
/// facet count: chord error scales linearly with radius
/// (`r(1 - cos(π/32))` ≈ 14 µm at 3 mm radius), still inside FDM
/// tolerance. Part of the determinism contract — same
/// [`CylinderParent`] + same radius + same `PLUG_CYLINDER_SEGMENTS`
/// → bit-equal output across plug, Negative cup, and Positive cup
/// (3-piece shared primitive per recon §2).
pub(crate) const PLUG_CYLINDER_SEGMENTS: u32 = 32;

/// Axial plug-anchor pin geometry spec. All dimensions in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PlugPinSpec {
    /// Pin radius (m). Default `0.003` = 3 mm = 6 mm diameter.
    ///
    /// 6 mm Ø ships **wider** than [`crate::registration::PinSpec`]'s
    /// 3 mm Ø inter-piece pin radius. Pre-S6 this was a marching-
    /// cubes resolution constraint (a 3 mm Ø pin captured as only
    /// 1 cell radially at the 3 mm default cell size, fragmenting
    /// on MC); post-S6 the plug pin lands as an exact post-MC
    /// cylinder primitive via [`build_plug_self_transforms`], so
    /// MC resolution no longer constrains the minimum diameter.
    /// The 6 mm baseline is retained for workshop ergonomics
    /// (FDM-printable without supports, comfortable hand-insertion)
    /// and consistency with the T-bar's diameter. Cup-side socket
    /// inflates by [`Self::shaft_diametral_clearance_m`] / 2 on
    /// each radial side for slide-fit.
    pub pin_radius_m: f64,
    /// Total axial pin length (m). Default `0.020` = 20 mm. The
    /// pin starts at the centerline endpoint and extends outward
    /// along the local segment tangent (away from the body).
    ///
    /// For a multi-layer cast the pin must reach into the cup-wall
    /// material of every layer that benefits from registration;
    /// outer layers with thicker shells need a longer pin. 20 mm
    /// suits single-layer + thin-shell multi-layer casts;
    /// thick-shell layouts override to ~25-30 mm.
    pub pin_length_m: f64,
    /// Diametral clearance between the plug pin shaft and the
    /// cup-wall socket (m). The cup-side
    /// [`MatingTransform::SubtractCylinder`] uses
    /// `pin_radius_m + shaft_diametral_clearance_m / 2` so the
    /// pin-vs-socket *diameter* difference equals
    /// `shaft_diametral_clearance_m` (symmetric `/2` convention —
    /// half the budget appears as radial gap on each side). v2
    /// iter-1 default `0.00030` = 0.30 mm matches recon §9 M4
    /// (plug-shaft positional fit) baseline. **Escalate-on-bind**:
    /// if the workshop M2 pin (`PinSpec::diametral_clearance_m =
    /// 0.20 mm`) trial binds for a given printer, lift this to
    /// `0.00050` (0.50 mm slide-fit) to retain workshop hand-
    /// insertion ergonomics.
    pub shaft_diametral_clearance_m: f64,
    /// Axial socket-bottom relief for the plug pin shaft (m). The
    /// cup-side socket cylinder's [`CylinderParent::half_length_m`]
    /// extends past the plug-side pin's half-length by
    /// `shaft_axial_clearance_m / 2` on each axial face (symmetric
    /// `/2` convention matching the diametral budget).
    ///
    /// Unlike the registration pin's
    /// [`crate::registration::PinSpec::axial_clearance_m`] — where
    /// the per-piece `SeamTrim` clips the near-seam half of the
    /// cylinder so workshop engagement reduces to
    /// [`crate::piece::RIBBON_PIECE_OVERLAP_M`] — the plug-side
    /// pin shaft is **not** clipped by a `SeamTrim` (the plug STL
    /// is a single unsplit piece). Workshop engagement on the plug
    /// side is the full [`Self::pin_length_m`].
    ///
    /// The shaft's axis is parallel to the centerline tangent at
    /// the cap plane (= along `pour_outward`) which is typically
    /// perpendicular to the seam-plane normal — i.e., the shaft
    /// axis lies WITHIN the seam plane. The cup-side socket
    /// cylinder is therefore bisected LENGTHWISE by the per-piece
    /// `SeamTrim` (each cup half receives one half-cylinder of the
    /// socket); the `axial / 2` extension SURVIVES the trim on both
    /// axial faces, but only the DEEP-end extension is workshop-
    /// meaningful (the near-end extension lies inside the body
    /// cavity, where there is no cup-wall material to subtract from).
    ///
    /// v2 iter-1 default `0.00100` = 1.00 mm matches recon §9
    /// M4 baseline.
    pub shaft_axial_clearance_m: f64,
    /// Whether to add a matching pin at the dome end
    /// (`centerline.last()`). Default `false` — the plug's
    /// hemispherical cap seats naturally into the body cavity's
    /// matching dome for centerline alignment at the un-pinned
    /// end, so a second pin would only punch an unneeded leak path
    /// through the cup wall at the dome. High-curvature centerlines
    /// (max tangent rotation approaching 120°) may need dome-end
    /// rotation constraint and opt in here.
    pub include_dome_pin: bool,
    /// Whether to add a T-bar at the pour-end pin's tip, locking
    /// the plug against axial pull-out + rotation around the pin
    /// axis once the cup pieces close around it.
    ///
    /// The T-bar is a cylinder whose axis is computed as
    /// `pour_outward × split_normal` (normalized). For the typical
    /// iter-1 pour-end pin (`pour_outward` aligned with the
    /// centerline's local tangent at the cap plane), this axis is
    /// **parallel to the ribbon binormal** — i.e., parallel to the
    /// seam-plane normal — so the seam plane bisects the T-bar
    /// through its center along a flat circular cross-section.
    /// Each cup half therefore receives one half-cylinder T-slot
    /// (with a co-planar half-disk cross-section on its mating
    /// face), and the workshop user closes the second cup half
    /// around the plug's T-bar (captive insertion). The plug
    /// cannot pull up because the T-bar would have to push through
    /// cup-wall material between the T-slot and the shaft socket;
    /// it cannot rotate around the pin axis because the T-bar
    /// would have to rotate out of the seam-normal orientation the
    /// T-slot accepts.
    ///
    /// Default `true` for v2.1 iter-1 +. Disable when the cured
    /// silicone alone provides adequate plug-to-mold hold and the
    /// workshop doesn't want the T-bar's protrusion through the cup
    /// wall outer face (at typical 5 mm wall thicknesses, the T-bar
    /// at iter-1's default dimensions protrudes ~2 mm below the cup
    /// outer face — cf-cast emits a complementary `platform.stl`
    /// with a matching pocket so the assembled mold sits flat on
    /// the platform during pour + cure).
    pub include_t_bar: bool,
    /// T-bar cylinder radius (m). Default `0.003` = 3 mm = 6 mm
    /// diameter, matching [`Self::pin_radius_m`].
    pub t_bar_radius_m: f64,
    /// T-bar cylinder half-length along its axis (m). Default
    /// `0.012` = 12 mm half-length → 24 mm total = 4× pin Ø.
    /// Long enough that each cup piece's half-T-slot has
    /// comfortable mechanical purchase against the T-bar's upward
    /// motion; short enough that the T-bar fits inside typical
    /// workshop bounding regions along the binormal direction.
    pub t_bar_half_length_m: f64,
    /// Diametral clearance between the T-bar and the cup-wall
    /// T-slot (m). Same `/2` convention as
    /// [`Self::shaft_diametral_clearance_m`] — cup-side T-slot
    /// radius is `t_bar_radius_m + t_bar_diametral_clearance_m /
    /// 2`.
    ///
    /// v2 iter-1 default `0.00030` = 0.30 mm matches recon §9 M3
    /// (T-bar positional fit) baseline. The T-bar is captive-
    /// inserted (workshop user closes the second cup half around
    /// it), not axial-slid, so the tolerance only needs to absorb
    /// FDM print tolerance + the workshop's ability to close the
    /// cup halves flush. Tightens further (e.g., `0.00010` = 0.10
    /// mm) when a future iter's printer surfaces a binding-tight
    /// fit; loosens when iter-1 print surfaces excess wobble.
    pub t_bar_diametral_clearance_m: f64,
    /// Axial pocket-bottom relief for the T-slot (m). The cup-side
    /// T-slot's [`CylinderParent::half_length_m`] extends past the
    /// plug-side T-bar's by `t_bar_axial_clearance_m / 2` on each
    /// axial face (lateral ends of the bar, in seam-plane-normal
    /// terms: each cup half's T-slot is one half of the bisected
    /// T-bar cylinder, with `axial/2` of pocket relief at the
    /// AWAY-from-seam tip).
    ///
    /// v2 iter-1 default `0.00100` = 1.00 mm matches recon §9
    /// M3 baseline.
    pub t_bar_axial_clearance_m: f64,
}

impl PlugPinSpec {
    /// v2.1 iter-1 defaults: single pour-end pin (6 mm Ø × 20 mm
    /// long, 0.30 mm × 1.00 mm shaft clearance), T-bar lock
    /// enabled at pin tip (6 mm Ø × 24 mm long, 0.30 mm × 1.00 mm
    /// T-slot clearance), dome-end pin disabled.
    ///
    /// Pin Ø matches [`crate::registration::PinSpec`]'s 3 mm × 2
    /// = 6 mm for workshop hand-insertion ergonomics + consistency
    /// with the T-bar's diameter (pre-S6 it was an MC-resolution
    /// constraint; S6 retired that constraint via mesh-CSG). Both
    /// clearance pairs ship at the recon §9 M3/M4 positional-fit
    /// baseline (0.30 mm diametral, 1.00 mm axial); escalate-on-
    /// bind to 0.50 mm diametral only if a future iter's printer
    /// surfaces binding.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            pin_radius_m: 0.003,
            pin_length_m: 0.020,
            shaft_diametral_clearance_m: 0.00030,
            shaft_axial_clearance_m: 0.00100,
            include_dome_pin: false,
            include_t_bar: true,
            t_bar_radius_m: 0.003,
            t_bar_half_length_m: 0.012,
            t_bar_diametral_clearance_m: 0.00030,
            t_bar_axial_clearance_m: 0.00100,
        }
    }
}

impl Default for PlugPinSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// What plug-anchor pin geometry, if any, the [`Ribbon`] uses.
///
/// `None` is the v2 (pre-v2.1) default — no axial plug pins; the
/// workshop user hand-positions the plug during pour + cure (the
/// gap that v2.1 closes). `Axial(PlugPinSpec)` enables v2.1's
/// axial pins + matching sockets.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum PlugPinKind {
    /// No plug-anchor pins.
    #[default]
    None,
    /// Axial pins at each centerline endpoint, per [`PlugPinSpec`].
    Axial(PlugPinSpec),
}

/// Per-feature [`CylinderParent`] triples for one ribbon's plug-pin
/// geometry.
///
/// All parents are at PIN (= plug-side) dimensions —
/// `half_length = pin_length_m / 2` for the shaft,
/// `half_length = t_bar_half_length_m` for the T-bar. Cup-side
/// inflation (`+ axial_clearance / 2`) happens at transform-emit
/// time in [`build_plug_socket_transforms`]; plug-side consumers
/// in [`build_plug_self_transforms`] use the parents verbatim.
/// Same triple consumed by plug + Negative cup + Positive cup is
/// the load-bearing 3-piece shared-primitive invariant (recon §2).
#[derive(Debug)]
struct PlugMatingParents {
    /// Pour-end shaft cylinder parent (always present when
    /// `plug_pins = Axial(_)`).
    pour_shaft: CylinderParent,
    /// Pour-end T-bar cylinder parent (when `include_t_bar`).
    pour_t_bar: Option<CylinderParent>,
    /// Dome-end shaft cylinder parent (when `include_dome_pin`).
    /// No T-bar at the dome end (the dome end doesn't lock against
    /// axial pull — it relies on the plug's hemispherical cap
    /// seating in the body-cavity dome).
    dome_shaft: Option<CylinderParent>,
}

/// Resolve the per-feature [`CylinderParent`] triples for the
/// ribbon's plug-pin kind, or `None` when the ribbon has no plug
/// pins.
///
/// Returns `Some((spec, parents))` where `parents` carries one
/// triple per enabled feature. The `pour_shaft` triple is always
/// present; `pour_t_bar` follows [`PlugPinSpec::include_t_bar`];
/// `dome_shaft` follows [`PlugPinSpec::include_dome_pin`].
///
/// Returns `None` for [`PlugPinKind::None`], a degenerate pour
/// anchor (empty ribbon), or a degenerate T-bar axis when
/// `include_t_bar = true` (pour outward parallel to split-normal —
/// caught upstream by [`Ribbon::new`] but defended-against here
/// for robustness). The degenerate-axis case suppresses ALL
/// transforms rather than just the T-bar so the caller doesn't
/// produce a misshapen plug with a shaft but no anti-rotation lock.
fn build_plug_mating_parents(ribbon: &Ribbon) -> Option<(&PlugPinSpec, PlugMatingParents)> {
    let spec = match &ribbon.plug_pins {
        PlugPinKind::None => return None,
        PlugPinKind::Axial(spec) => spec,
    };
    let (pour, dome) = pour_and_dome_anchors(ribbon)?;
    let split_vec = ribbon.split_normal.as_vector();
    let pour_shaft = shaft_parent(pour.0, pour.1, spec.pin_length_m);
    let pour_t_bar = if spec.include_t_bar {
        // `?` propagates degenerate-axis None up — see fn-level doc
        // on why we suppress all transforms in that case.
        Some(t_bar_parent_at(pour.0, pour.1, &split_vec, spec)?)
    } else {
        None
    };
    let dome_shaft = spec
        .include_dome_pin
        .then(|| shaft_parent(dome.0, dome.1, spec.pin_length_m));
    Some((
        spec,
        PlugMatingParents {
            pour_shaft,
            pour_t_bar,
            dome_shaft,
        },
    ))
}

/// Build a shaft [`CylinderParent`] for a pin anchored at `anchor`
/// extending along `outward` for `length_m` total.
///
/// The cylinder is centered at `anchor + outward * (length_m / 2)`
/// with axis along `outward` and `half_length = length_m / 2`.
fn shaft_parent(anchor: Point3<f64>, outward: Vector3<f64>, length_m: f64) -> CylinderParent {
    let half_length = length_m / 2.0;
    let center = anchor + outward * half_length;
    // `UnitVector3::new_normalize` is defensive against tiny
    // numerical drift; ribbon construction guarantees `outward` is
    // already unit-magnitude for the typical (centerline tangent
    // or cap-plane normal) inputs.
    let axis = UnitVector3::new_normalize(outward);
    CylinderParent {
        center_m: center,
        axis,
        half_length_m: half_length,
    }
}

/// Build the T-bar [`CylinderParent`] for the pour-end pin, or
/// `None` when the T-bar axis (`pour_outward × split_normal_vec`)
/// is degenerate (pour direction parallel to split-normal).
///
/// The T-bar sits at the pin tip (`pour_anchor + pour_outward *
/// pin_length_m`) with axis = `pour_outward × split_normal_vec`
/// normalized. For the typical iter-1 pour-end pin where
/// `pour_outward` aligns with the centerline tangent at the cap
/// plane, this axis equals the ribbon binormal — i.e., is parallel
/// to the seam-plane normal — so the per-piece `SeamTrim` bisects
/// the cylinder through its center along a co-planar circular
/// cross-section (recon §5 approach (a)).
fn t_bar_parent_at(
    pour_anchor: Point3<f64>,
    pour_outward: Vector3<f64>,
    split_normal_vec: &Vector3<f64>,
    spec: &PlugPinSpec,
) -> Option<CylinderParent> {
    let bar_axis_raw = pour_outward.cross(split_normal_vec);
    let bar_axis_norm = bar_axis_raw.norm();
    if bar_axis_norm < 1e-9 {
        return None;
    }
    let bar_axis = UnitVector3::new_unchecked(bar_axis_raw / bar_axis_norm);
    let bar_center = pour_anchor + pour_outward * spec.pin_length_m;
    Some(CylinderParent {
        center_m: bar_center,
        axis: bar_axis,
        half_length_m: spec.t_bar_half_length_m,
    })
}

/// Build the plug-side mesh-CSG transforms for the ribbon's
/// plug-pin geometry.
///
/// Returns one [`MatingTransform::UnionCylinder`] per enabled
/// feature (shaft, T-bar, optional dome shaft), each consuming the
/// shared [`CylinderParent`] at the **pin** dimensions verbatim.
/// Used by [`add_plug_pins`] to attach the post-MC shaft + T-bar
/// geometry to the bare plug Solid.
///
/// Cup-side counterparts come from [`build_plug_socket_transforms`]
/// and share each parent triple modulo the symmetric `/2`
/// clearance inflation (recon §2 + §9). The 3-piece shared-
/// primitive invariant (plug ↔ Negative cup ↔ Positive cup all
/// consume the same T-bar parent) is the strongest determinism
/// gate in the mating-features arc — see `mesh_csg::tests`
/// `t_bar_cylinder_mesh_is_bit_equal_across_three_pieces`.
///
/// Returns an empty `Vec` when:
/// - `ribbon.plug_pins` is [`PlugPinKind::None`];
/// - the ribbon has no segments (degenerate pour anchor);
/// - the T-bar axis would be degenerate when `include_t_bar = true`
///   (caught by [`Ribbon::new`] upstream).
#[must_use]
pub fn build_plug_self_transforms(ribbon: &Ribbon) -> Vec<MatingTransform> {
    let Some((spec, parents)) = build_plug_mating_parents(ribbon) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    out.push(MatingTransform::UnionCylinder {
        params: CylinderParams {
            parent: parents.pour_shaft,
            radius_m: spec.pin_radius_m,
            segments: PLUG_CYLINDER_SEGMENTS,
        },
    });
    if let Some(t_bar) = parents.pour_t_bar {
        out.push(MatingTransform::UnionCylinder {
            params: CylinderParams {
                parent: t_bar,
                radius_m: spec.t_bar_radius_m,
                segments: PLUG_CYLINDER_SEGMENTS,
            },
        });
    }
    if let Some(dome) = parents.dome_shaft {
        out.push(MatingTransform::UnionCylinder {
            params: CylinderParams {
                parent: dome,
                radius_m: spec.pin_radius_m,
                segments: PLUG_CYLINDER_SEGMENTS,
            },
        });
    }
    out
}

/// Build the cup-side mesh-CSG transforms for the ribbon's plug-
/// pin sockets and (when enabled) T-slot.
///
/// **Side-agnostic** per S4: both Negative and Positive cup pieces
/// emit the *same* Vec, and the per-piece
/// [`MatingTransform::SeamTrim`] downstream bisects each cylinder
/// at the seam plane. The T-bar's axis is parallel to the seam
/// normal, so the trim cuts each T-slot through its center along
/// a co-planar half-disk on the cup-piece mating face (recon §5
/// approach (a)).
///
/// Returns one [`MatingTransform::SubtractCylinder`] per enabled
/// feature, with the cup-side parameters inflated relative to the
/// plug-side pin/T-bar by the symmetric `/2` clearance convention:
/// - Shaft socket: `half_length += shaft_axial_clearance_m / 2`,
///   `radius_m = pin_radius_m + shaft_diametral_clearance_m / 2`.
/// - T-slot: `half_length += t_bar_axial_clearance_m / 2`,
///   `radius_m = t_bar_radius_m + t_bar_diametral_clearance_m / 2`.
/// - Dome-end shaft socket (when [`PlugPinSpec::include_dome_pin`]
///   is `true`): same inflation as the pour-end shaft socket.
///
/// Consumed by [`crate::piece::compose_piece_solid`]. Returns an
/// empty `Vec` when the ribbon has no plug pins; see
/// [`build_plug_self_transforms`] for the empty-Vec preconditions.
#[must_use]
pub fn build_plug_socket_transforms(ribbon: &Ribbon) -> Vec<MatingTransform> {
    let Some((spec, parents)) = build_plug_mating_parents(ribbon) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    out.push(MatingTransform::SubtractCylinder {
        params: CylinderParams {
            parent: inflate_axial(&parents.pour_shaft, spec.shaft_axial_clearance_m),
            radius_m: spec.pin_radius_m + spec.shaft_diametral_clearance_m / 2.0,
            segments: PLUG_CYLINDER_SEGMENTS,
        },
    });
    if let Some(t_bar) = parents.pour_t_bar {
        out.push(MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: inflate_axial(&t_bar, spec.t_bar_axial_clearance_m),
                radius_m: spec.t_bar_radius_m + spec.t_bar_diametral_clearance_m / 2.0,
                segments: PLUG_CYLINDER_SEGMENTS,
            },
        });
    }
    if let Some(dome) = parents.dome_shaft {
        out.push(MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: inflate_axial(&dome, spec.shaft_axial_clearance_m),
                radius_m: spec.pin_radius_m + spec.shaft_diametral_clearance_m / 2.0,
                segments: PLUG_CYLINDER_SEGMENTS,
            },
        });
    }
    out
}

/// Extend a [`CylinderParent`]'s half-length by `axial_clearance_m
/// / 2`, preserving `center_m` + `axis`.
///
/// The cup-side cylinder primitive then extends `axial / 2` past
/// the plug-side cylinder on EACH axial face (symmetric `/2`
/// convention matching the diametral budget). Per-piece `SeamTrim`
/// downstream interacts with the extended cylinder differently
/// depending on which feature it carries:
///
/// - **T-slot** (T-bar axis parallel to seam normal): seam plane
///   bisects the cylinder through its center, perpendicular to its
///   long axis. Each cup half receives one half-cylinder; the
///   `axial / 2` extension at each lateral tip becomes a pocket-
///   bottom relief at the AWAY-from-seam end of that half's T-slot.
/// - **Shaft socket** (shaft axis perpendicular to seam normal):
///   shaft axis lies WITHIN the seam plane, so the trim bisects
///   the cylinder LENGTHWISE; each cup half receives one half-
///   cylinder split along its long axis. The `axial / 2`
///   extension survives the trim on both axial faces, but only the
///   DEEP-end extension is workshop-meaningful (the near-end
///   extension lies inside the body cavity, where there is no
///   cup-wall material to subtract from).
fn inflate_axial(parent: &CylinderParent, axial_clearance_m: f64) -> CylinderParent {
    CylinderParent {
        center_m: parent.center_m,
        axis: parent.axis,
        half_length_m: parent.half_length_m + axial_clearance_m / 2.0,
    }
}

/// Resolve the pour-end T-bar location + axis for this ribbon's
/// plug-pin kind, or `None` if disabled / inapplicable.
///
/// Returns `None` for `PlugPinKind::None`, `include_t_bar = false`,
/// or a degenerate pour anchor. Exposes the geometry the
/// [`crate::platform`] module needs to size the matching pocket in
/// `platform.stl` without duplicating the anchor-resolution logic.
///
/// Returns `(t_bar_center, t_bar_axis, t_bar_radius_m,
/// t_bar_half_length_m)` — bare plug-side dimensions in world-
/// frame meters / unit directions. The platform consumer adds its
/// own [`crate::platform::PLATFORM_HOLE_LATERAL_SLACK_M`] to size
/// the pocket; the post-S6 cup-side T-slot clearance fields
/// ([`PlugPinSpec::t_bar_diametral_clearance_m`] /
/// [`PlugPinSpec::t_bar_axial_clearance_m`]) are NOT applied here
/// — the platform pocket sizes against the bare T-bar, not the
/// inflated T-slot.
#[must_use]
pub fn pour_end_t_bar_geometry(ribbon: &Ribbon) -> Option<(Point3<f64>, Vector3<f64>, f64, f64)> {
    let (spec, parents) = build_plug_mating_parents(ribbon)?;
    let t_bar = parents.pour_t_bar?;
    Some((
        t_bar.center_m,
        t_bar.axis.into_inner(),
        spec.t_bar_radius_m,
        spec.t_bar_half_length_m,
    ))
}

/// Resolve `((pour_point, pour_outward), (dome_point, dome_outward))`
/// for this ribbon — i.e., (point, outward axis) tuples for the
/// plug-anchor pin's pour-end and (optional) dome-end positions.
///
/// **Pour-end**:
/// - `pour_end_hint = Some((centroid, outward))` → anchor at
///   `centroid` extending along `outward`. This sits where the
///   caller says it sits, **NOT** at any centerline endpoint —
///   cf-scan-prep's `trim_floor_mm` typically leaves
///   `centerline.last()` 40 mm short of the cap plane, and the
///   pin must sit on the plug body's pinned floor (= cap plane),
///   not at the trimmed centerline tip (which lives 40 mm inside
///   the plug body).
/// - `pour_end_hint = None` → fall back to `(last.end, last.tangent)`
///   per cf-scan-prep's tip→base centerline-orientation convention.
///
/// **Dome-end** (only consulted when [`PlugPinSpec::include_dome_pin`]
/// is `true`): the centerline endpoint **farthest** from the pour
/// anchor. When the hint is set, that's whichever of
/// `first.start` / `last.end` lies farther from the hint centroid;
/// when unset, defaults to `first.start` (the opposite end from
/// the fallback pour). Outward axis is the local-segment tangent
/// pointing away from the body interior at that endpoint.
fn pour_and_dome_anchors(
    ribbon: &Ribbon,
) -> Option<((Point3<f64>, Vector3<f64>), (Point3<f64>, Vector3<f64>))> {
    let first = ribbon.segments.first()?;
    let last = ribbon.segments.last()?;
    let start_anchor = (first.start, -first.tangent);
    let end_anchor = (last.end, last.tangent);
    let (pour_anchor, dome_anchor) = match ribbon.pour_end_hint {
        Some((centroid, outward)) => {
            // Pour-end is the hinted location verbatim. Dome-end is
            // the centerline endpoint farther from the hint.
            let d_start = (centroid - first.start).norm_squared();
            let d_end = (centroid - last.end).norm_squared();
            let dome = if d_start > d_end {
                start_anchor
            } else {
                end_anchor
            };
            ((centroid, outward), dome)
        }
        None => (end_anchor, start_anchor),
    };
    Some((pour_anchor, dome_anchor))
}

/// Pair a user-supplied plug [`Solid`] with the post-MC mesh-CSG
/// transforms that materialize its axial pin shaft + T-bar
/// geometry per the ribbon's [`PlugPinKind`].
///
/// Returns `(plug, transforms)` where `plug` is the **bare** plug
/// Solid (passed through unchanged) and `transforms` is the
/// [`MatingTransform`] Vec from [`build_plug_self_transforms`].
/// Empty `transforms` for [`PlugPinKind::None`]; otherwise one
/// [`MatingTransform::UnionCylinder`] per enabled feature (shaft,
/// T-bar, optional dome shaft) that the downstream
/// [`crate::mesh_csg::apply_mating_transforms`] applies between MC
/// output and the F4 printability gate.
///
/// Pre-S6 this function returned `plug.union(pins_SDF)` — pin +
/// T-bar geometry baked into the SDF expression tree before MC. S6
/// moved the pin + T-bar to post-MC mesh-CSG (recon §1 + §5) so
/// the plug's shaft + T-bar match the cup-piece sockets + T-slot
/// by shared-parent construction rather than by MC tolerance.
///
/// [`CastSpec::export_molds_v2`] calls this internally for each
/// per-layer plug (derived from `spec.plug` for layer 0 and
/// `layers[N-1].body` for `N > 0`), so callers passing
/// [`PlugPinKind::Axial`] via [`Ribbon::with_plug_pins`] get the
/// pin geometry baked into every plug STL automatically — no need
/// to call `add_plug_pins` from the example crate. The function
/// stays public for callers building a custom plug `Solid` outside
/// the [`CastSpec`] per-layer-plug derivation.
///
/// [`Ribbon::plug_pins`]: crate::ribbon::Ribbon::plug_pins
/// [`CastSpec::export_molds_v2`]: crate::CastSpec::export_molds_v2
/// [`CastSpec`]: crate::CastSpec
#[must_use]
pub fn add_plug_pins(plug: Solid, ribbon: &Ribbon) -> (Solid, Vec<MatingTransform>) {
    (plug, build_plug_self_transforms(ribbon))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]

    use super::*;
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    /// Standard test fixture — a 100 mm centerline along +X with +Y
    /// split-normal. `first_segment.tangent = +X` so the start
    /// endpoint's outward axis is `-X`; `last_segment.tangent = +X`
    /// so the end endpoint's outward axis is `+X`. With no
    /// `pour_end_hint` set, the pour-end falls back to
    /// `points.last() = (+0.050, 0, 0)` and the pour-end pin extends
    /// along `+X` from there.
    fn straight_x_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split).unwrap()
    }

    /// Extract `CylinderParams` from a [`MatingTransform`] for test
    /// inspection. Both Union and Subtract cylinder variants carry
    /// the same payload type. Plug self/socket transforms never
    /// emit `SeamTrim`, so a `SeamTrim` variant is a structural
    /// bug in the producer.
    fn params_of(t: &MatingTransform) -> &CylinderParams {
        match t {
            MatingTransform::UnionCylinder { params }
            | MatingTransform::SubtractCylinder { params } => params,
            MatingTransform::SeamTrim { .. } => {
                panic!("plug transforms should never include SeamTrim")
            }
        }
    }

    #[test]
    fn plug_pin_spec_iter1_has_workshop_defaults() {
        let s = PlugPinSpec::iter1();
        assert!((s.pin_radius_m - 0.003).abs() < f64::EPSILON);
        assert!((s.pin_length_m - 0.020).abs() < f64::EPSILON);
        // S6 clearance defaults — recon §9 M3+M4 baseline
        // (positional fit). 0.30 mm diametral × 1.00 mm axial for
        // BOTH the shaft and T-bar.
        assert!((s.shaft_diametral_clearance_m - 0.00030).abs() < f64::EPSILON);
        assert!((s.shaft_axial_clearance_m - 0.00100).abs() < f64::EPSILON);
        assert!((s.t_bar_diametral_clearance_m - 0.00030).abs() < f64::EPSILON);
        assert!((s.t_bar_axial_clearance_m - 0.00100).abs() < f64::EPSILON);
        assert!(s.include_t_bar, "iter-1 default ships the T-bar lock");
        assert!(
            !s.include_dome_pin,
            "iter-1 default ships pour-end pin only (no dome-end pin); \
             dome-end is geometrically closed by the cap-into-cavity seating"
        );
    }

    #[test]
    fn plug_pin_kind_default_is_none() {
        assert_eq!(PlugPinKind::default(), PlugPinKind::None);
    }

    /// `PlugPinKind::None` short-circuits both transform builders to
    /// empty Vec — the empty-Vec pass-through path
    /// `apply_mating_transforms` short-circuits on (in
    /// `mesh_csg.rs`).
    #[test]
    fn build_plug_self_transforms_none_returns_empty() {
        let ribbon = straight_x_ribbon();
        assert!(build_plug_self_transforms(&ribbon).is_empty());
        assert!(build_plug_socket_transforms(&ribbon).is_empty());
    }

    /// `add_plug_pins` passes the plug Solid through unchanged
    /// (S6: Solid path is bare; pin/T-bar geometry lands post-MC).
    /// With `PlugPinKind::None` the transforms Vec is empty too.
    #[test]
    fn add_plug_pins_passes_plug_through_unchanged() {
        let ribbon = straight_x_ribbon();
        let plug = Solid::capsule(0.005, 0.020);
        let (returned_plug, transforms) = add_plug_pins(plug.clone(), &ribbon);
        // Bare plug Solid is returned verbatim — same SDF response
        // at a handful of sample points.
        for q in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.025),
        ] {
            let bare = plug.evaluate(&q);
            let returned = returned_plug.evaluate(&q);
            assert!(
                (bare - returned).abs() < 1e-12,
                "add_plug_pins must pass through plug unchanged (S6: pin geometry \
                 lands post-MC); differed at {q:?}: bare={bare}, returned={returned}",
            );
        }
        // PlugPinKind::None → empty transforms Vec.
        assert!(
            transforms.is_empty(),
            "PlugPinKind::None should produce empty transforms Vec; got {} ops",
            transforms.len(),
        );
    }

    /// `add_plug_pins` with `PlugPinKind::Axial` returns the plug
    /// unchanged AND a Vec containing one `UnionCylinder` per
    /// enabled feature. Replaces the pre-S6
    /// `add_plug_pins_extends_plug_into_pin_region` test — post-S6
    /// the pin no longer extends the plug `Solid`; it appears in
    /// the returned transforms Vec as a separate primitive.
    #[test]
    fn add_plug_pins_axial_emits_shaft_and_t_bar_transforms() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let plug = Solid::capsule(0.005, 0.020);
        let (returned_plug, transforms) = add_plug_pins(plug.clone(), &ribbon);

        // (1) Plug Solid is the bare input; the previous "pin
        // extends plug into pin region" invariant moves to the
        // mesh-CSG output, no longer testable at the Solid SDF
        // level.
        let bare = plug.evaluate(&Point3::origin());
        let returned = returned_plug.evaluate(&Point3::origin());
        assert!((bare - returned).abs() < 1e-12);

        // (2) Transforms Vec carries shaft + T-bar (iter-1 default
        // ships both, no dome-pin).
        assert_eq!(
            transforms.len(),
            2,
            "iter-1 default emits shaft + T-bar = 2 transforms; got {}",
            transforms.len(),
        );
        // Both are UnionCylinder (plug-side adds material).
        for t in &transforms {
            assert!(
                matches!(t, MatingTransform::UnionCylinder { .. }),
                "plug-side transforms should all be UnionCylinder; got {t:?}",
            );
        }
    }

    /// With no `pour_end_hint`, the pour-end shaft parent anchors
    /// at `centerline.last()` extending along `+last_segment.tangent`
    /// per cf-scan-prep's tip→base convention. Pre-S6 this was an
    /// AABB-bounds test; post-S6 the equivalent invariant lives in
    /// the shaft transform's parent geometry (center + axis +
    /// `half_length`).
    #[test]
    fn build_plug_self_transforms_default_anchors_shaft_at_centerline_last() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let transforms = build_plug_self_transforms(&ribbon);
        // First transform = shaft (per Vec ordering in
        // build_plug_self_transforms).
        let shaft = params_of(&transforms[0]);
        // pour anchor = centerline.last() = (+0.050, 0, 0); outward
        // = +X; pin_length = 20 mm; half_length = 10 mm.
        // Shaft center = (0.050 + 0.010, 0, 0) = (+0.060, 0, 0).
        let c = shaft.parent.center_m;
        assert!((c.x - 0.060).abs() < 1e-9, "shaft center.x = {}", c.x);
        assert!(c.y.abs() < 1e-9);
        assert!(c.z.abs() < 1e-9);
        // Axis = +X (last_segment.tangent).
        assert!((shaft.parent.axis.x - 1.0).abs() < 1e-9);
        assert!(shaft.parent.axis.y.abs() < 1e-9);
        assert!(shaft.parent.axis.z.abs() < 1e-9);
        // half_length = pin_length / 2 = 10 mm.
        assert!(
            (shaft.parent.half_length_m - 0.010).abs() < 1e-9,
            "shaft half_length = {} (expected 0.010)",
            shaft.parent.half_length_m,
        );
        // Bare pin radius (no clearance inflation on plug side).
        assert!((shaft.radius_m - 0.003).abs() < f64::EPSILON);
    }

    /// `with_pour_end_hint(centroid, outward)` anchors the pour-
    /// shaft at the hint `centroid` extending along `outward`,
    /// **NOT** at any centerline endpoint. Pre-E.2 fixture regression
    /// preserved post-S6 as transform-parameter inspection.
    #[test]
    fn build_plug_self_transforms_with_pour_end_hint_anchors_at_centroid() {
        let hint_centroid = Point3::new(-0.060, 0.0, 0.0);
        let outward = Vector3::new(-1.0, 0.0, 0.0);
        let ribbon = straight_x_ribbon()
            .with_pour_end_hint(hint_centroid, outward)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let transforms = build_plug_self_transforms(&ribbon);
        let shaft = params_of(&transforms[0]);
        // Shaft center = hint_centroid + outward * pin_length / 2
        //              = (-0.060, 0, 0) + (-1, 0, 0) * 0.010
        //              = (-0.070, 0, 0).
        let c = shaft.parent.center_m;
        assert!(
            (c.x - -0.070).abs() < 1e-9,
            "shaft center anchored past hint centroid; got {c:?}",
        );
        // Axis = -X (outward).
        assert!((shaft.parent.axis.x - -1.0).abs() < 1e-9);
        // Crucially: center.x < -0.050 (past centerline[0]) — the
        // anchor really is the hint centroid, not a centerline
        // endpoint.
        assert!(
            c.x < -0.050,
            "hint-anchored shaft must NOT regress to centerline-endpoint anchoring; \
             got center.x = {}",
            c.x,
        );
    }

    /// `include_dome_pin = true` emits THREE transforms — pour
    /// shaft + T-bar + dome shaft. Default ships only two (shaft +
    /// T-bar), so this test guards the dome-pin migration (S6:
    /// migrated uniformly per user instruction).
    #[test]
    fn build_plug_self_transforms_with_dome_pin_emits_three() {
        let spec = PlugPinSpec {
            include_dome_pin: true,
            ..PlugPinSpec::iter1()
        };
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(spec));
        let transforms = build_plug_self_transforms(&ribbon);
        assert_eq!(
            transforms.len(),
            3,
            "include_dome_pin enables a 3rd UnionCylinder transform; got {}",
            transforms.len(),
        );
        // Locate the dome shaft by its negative-X center (pour end
        // is +X for straight_x_ribbon's tangent fallback).
        let dome = transforms
            .iter()
            .map(params_of)
            .find(|p| p.parent.center_m.x < 0.0)
            .expect("dome shaft transform should appear at -X center");
        // Dome anchor = centerline[0] = (-0.050, 0, 0); outward =
        // -first_segment.tangent = -X. Dome shaft center =
        // (-0.050, 0, 0) + (-1, 0, 0) * 0.010 = (-0.060, 0, 0).
        assert!((dome.parent.center_m.x - -0.060).abs() < 1e-9);
    }

    /// `include_t_bar = false` drops the T-bar transform — only
    /// the shaft remains.
    #[test]
    fn build_plug_self_transforms_without_t_bar_emits_only_shaft() {
        let spec = PlugPinSpec {
            include_t_bar: false,
            ..PlugPinSpec::iter1()
        };
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(spec));
        let transforms = build_plug_self_transforms(&ribbon);
        assert_eq!(transforms.len(), 1);
    }

    /// Three-piece bit-equal determinism contract (recon §2): the
    /// T-bar [`CylinderParent`] is consumed by THREE pieces — the
    /// plug (`UnionCylinder`) plus both cup halves
    /// (`SubtractCylinder` with shared `center` + `axis` + pin
    /// `half_length`). All three calls to
    /// [`crate::mesh_csg::build_cylinder_along_axis`] with that
    /// shared parent + the same pin radius + same segments must
    /// produce bit-equal `to_mesh_f64` output.
    ///
    /// The S5 two-piece variant ships in `mesh_csg::tests`
    /// (`pin_cylinder_mesh_is_bit_equal_across_pieces`); this is
    /// the load-bearing 3-piece extension for S6's most-coupled
    /// feature.
    #[test]
    fn t_bar_cylinder_mesh_is_bit_equal_across_three_pieces() {
        use crate::mesh_csg::build_cylinder_along_axis;

        // Synthetic ribbon mirroring the iter-1 fixture (centerline
        // along +Z, split-normal +X, cap hint at -Z) so the T-bar
        // axis exercises the typical pour_outward × split_normal
        // path.
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));

        // Resolve the T-bar parent triple directly. Same payload is
        // consumed by plug (UnionCylinder), Negative cup
        // (SubtractCylinder), Positive cup (SubtractCylinder).
        let (_spec, parents) = build_plug_mating_parents(&ribbon).expect("Axial kind");
        let t_bar_parent = parents.pour_t_bar.expect("include_t_bar = true");
        let r_pin = PlugPinSpec::iter1().t_bar_radius_m;
        let segments = PLUG_CYLINDER_SEGMENTS;

        // Build the SAME radius + parent + segments THREE times.
        let m_plug = build_cylinder_along_axis(&t_bar_parent, r_pin, segments);
        let m_neg = build_cylinder_along_axis(&t_bar_parent, r_pin, segments);
        let m_pos = build_cylinder_along_axis(&t_bar_parent, r_pin, segments);
        let (v_plug, p_plug, t_plug) = m_plug.to_mesh_f64();
        let (v_neg, p_neg, t_neg) = m_neg.to_mesh_f64();
        let (v_pos, p_pos, t_pos) = m_pos.to_mesh_f64();

        // All three properties counts agree.
        assert_eq!(p_plug, p_neg);
        assert_eq!(p_neg, p_pos);
        // Bit-equal vertices across all three consumers.
        assert_eq!(
            v_plug, v_neg,
            "plug vs Negative cup: T-bar parent must produce bit-equal mesh"
        );
        assert_eq!(
            v_neg, v_pos,
            "Negative vs Positive cup: T-bar parent must produce bit-equal mesh"
        );
        // …and bit-equal triangle index arrays.
        assert_eq!(t_plug, t_neg);
        assert_eq!(t_neg, t_pos);
        assert!(!v_plug.is_empty());
        assert!(!t_plug.is_empty());
    }

    /// Positive-side T-slot and shaft socket params carry the
    /// symmetric `/2` diametral + axial inflation; the cup-side
    /// transforms share `center_m` + `axis` with the plug-side
    /// transforms per shared-parent invariant (recon §2). S5 polish
    /// pattern carried forward — the `mesh_csg` fit invariant
    /// constructs parents inline, so without this test a stray
    /// `* 2.0` or dropped `/ 2.0` on the socket arm could land
    /// without any test surfacing it.
    #[test]
    fn plug_socket_transforms_inflate_per_spec() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let plug_side = build_plug_self_transforms(&ribbon);
        let cup_side = build_plug_socket_transforms(&ribbon);
        assert_eq!(
            plug_side.len(),
            cup_side.len(),
            "plug-side and cup-side emit one transform per feature"
        );

        let spec = PlugPinSpec::iter1();
        for (plug_t, cup_t) in plug_side.iter().zip(&cup_side) {
            // plug-side = UnionCylinder; cup-side = SubtractCylinder.
            assert!(matches!(plug_t, MatingTransform::UnionCylinder { .. }));
            assert!(matches!(cup_t, MatingTransform::SubtractCylinder { .. }));
            let p = params_of(plug_t);
            let c = params_of(cup_t);
            // Shared center + axis.
            assert_eq!(p.parent.center_m, c.parent.center_m);
            assert_eq!(p.parent.axis, c.parent.axis);
            assert_eq!(p.segments, c.segments);
            // Cup-side half_length inflates by `axial / 2`. Which
            // axial budget depends on whether this is the T-bar
            // (different radius) or a shaft.
            let is_t_bar = (p.radius_m - spec.t_bar_radius_m).abs() < f64::EPSILON
                && (p.parent.half_length_m - spec.t_bar_half_length_m).abs() < f64::EPSILON;
            let (axial, diametral) = if is_t_bar {
                (
                    spec.t_bar_axial_clearance_m,
                    spec.t_bar_diametral_clearance_m,
                )
            } else {
                (
                    spec.shaft_axial_clearance_m,
                    spec.shaft_diametral_clearance_m,
                )
            };
            let expected_half_length = p.parent.half_length_m + axial / 2.0;
            let expected_radius = p.radius_m + diametral / 2.0;
            assert!(
                (c.parent.half_length_m - expected_half_length).abs() < f64::EPSILON,
                "cup-side half_length should be plug + axial/2 = {expected_half_length}; got {}",
                c.parent.half_length_m,
            );
            assert!(
                (c.radius_m - expected_radius).abs() < f64::EPSILON,
                "cup-side radius should be plug + diametral/2 = {expected_radius}; got {}",
                c.radius_m,
            );
        }
    }

    /// Math-verified plug-shaft / socket fit invariant — same shape
    /// as registration's `pin_and_socket_fit_invariant` but at the
    /// plug-shaft scale (6 mm Ø × 20 mm long) and using the M4
    /// clearance budget. Locks the workshop fit at engine
    /// precision rather than via eyeball cf-view per
    /// `feedback_math_verify_geometric_contracts`.
    #[test]
    fn plug_shaft_and_socket_fit_invariant() {
        use crate::mesh_csg::build_cylinder_along_axis;

        let spec = PlugPinSpec::iter1();
        let parent_pin = CylinderParent {
            center_m: Point3::origin(),
            axis: nalgebra::Vector3::z_axis(),
            half_length_m: spec.pin_length_m / 2.0,
        };
        let parent_socket = CylinderParent {
            center_m: parent_pin.center_m,
            axis: parent_pin.axis,
            half_length_m: parent_pin.half_length_m + spec.shaft_axial_clearance_m / 2.0,
        };
        let r_socket = spec.pin_radius_m + spec.shaft_diametral_clearance_m / 2.0;

        let pin = build_cylinder_along_axis(&parent_pin, spec.pin_radius_m, PLUG_CYLINDER_SEGMENTS);
        let socket = build_cylinder_along_axis(&parent_socket, r_socket, PLUG_CYLINDER_SEGMENTS);

        let (pin_min, pin_max) = pin.bounding_box_nalgebra().unwrap();
        let (sock_min, sock_max) = socket.bounding_box_nalgebra().unwrap();

        let half_diametral_mm =
            (spec.shaft_diametral_clearance_m / 2.0) * crate::mesher::METERS_TO_MM;
        let half_axial_mm = (spec.shaft_axial_clearance_m / 2.0) * crate::mesher::METERS_TO_MM;
        let tol_mm = 1.0e-3;

        for (name, gap) in [
            ("+X", sock_max.x - pin_max.x),
            ("-X", pin_min.x - sock_min.x),
            ("+Y", sock_max.y - pin_max.y),
            ("-Y", pin_min.y - sock_min.y),
        ] {
            assert!(
                (gap - half_diametral_mm).abs() < tol_mm,
                "plug shaft socket radial gap on {name} = {gap:.6} mm \
                 (expected {half_diametral_mm:.6} mm)",
            );
        }
        let axial_gap_z_max = sock_max.z - pin_max.z;
        let axial_gap_z_min = pin_min.z - sock_min.z;
        assert!(
            (axial_gap_z_max - half_axial_mm).abs() < tol_mm,
            "plug shaft socket +Z axial gap = {axial_gap_z_max:.6} mm \
             (expected {half_axial_mm:.6} mm)",
        );
        assert!(
            (axial_gap_z_min - half_axial_mm).abs() < tol_mm,
            "plug shaft socket -Z axial gap = {axial_gap_z_min:.6} mm \
             (expected {half_axial_mm:.6} mm)",
        );
    }

    /// Math-verified T-bar / T-slot fit invariant — sibling to
    /// `plug_shaft_and_socket_fit_invariant` using the M3
    /// clearance budget.
    #[test]
    fn t_bar_and_t_slot_fit_invariant() {
        use crate::mesh_csg::build_cylinder_along_axis;

        let spec = PlugPinSpec::iter1();
        let parent_bar = CylinderParent {
            center_m: Point3::origin(),
            axis: nalgebra::Vector3::z_axis(),
            half_length_m: spec.t_bar_half_length_m,
        };
        let parent_slot = CylinderParent {
            center_m: parent_bar.center_m,
            axis: parent_bar.axis,
            half_length_m: parent_bar.half_length_m + spec.t_bar_axial_clearance_m / 2.0,
        };
        let r_slot = spec.t_bar_radius_m + spec.t_bar_diametral_clearance_m / 2.0;

        let bar =
            build_cylinder_along_axis(&parent_bar, spec.t_bar_radius_m, PLUG_CYLINDER_SEGMENTS);
        let slot = build_cylinder_along_axis(&parent_slot, r_slot, PLUG_CYLINDER_SEGMENTS);

        let (bar_min, bar_max) = bar.bounding_box_nalgebra().unwrap();
        let (slot_min, slot_max) = slot.bounding_box_nalgebra().unwrap();

        let half_diametral_mm =
            (spec.t_bar_diametral_clearance_m / 2.0) * crate::mesher::METERS_TO_MM;
        let half_axial_mm = (spec.t_bar_axial_clearance_m / 2.0) * crate::mesher::METERS_TO_MM;
        let tol_mm = 1.0e-3;

        for (name, gap) in [
            ("+X", slot_max.x - bar_max.x),
            ("-X", bar_min.x - slot_min.x),
            ("+Y", slot_max.y - bar_max.y),
            ("-Y", bar_min.y - slot_min.y),
        ] {
            assert!(
                (gap - half_diametral_mm).abs() < tol_mm,
                "T-slot radial gap on {name} = {gap:.6} mm \
                 (expected {half_diametral_mm:.6} mm)",
            );
        }
        let axial_gap_z_max = slot_max.z - bar_max.z;
        let axial_gap_z_min = bar_min.z - slot_min.z;
        assert!(
            (axial_gap_z_max - half_axial_mm).abs() < tol_mm,
            "T-slot +Z axial gap = {axial_gap_z_max:.6} mm \
             (expected {half_axial_mm:.6} mm)",
        );
        assert!(
            (axial_gap_z_min - half_axial_mm).abs() < tol_mm,
            "T-slot -Z axial gap = {axial_gap_z_min:.6} mm \
             (expected {half_axial_mm:.6} mm)",
        );
    }

    /// Cap-plane anchor regression preserved post-S6: cf-scan-prep
    /// trims the centerline `trim_floor_mm` above the cap plane;
    /// the pour-shaft must anchor at the cap-plane centroid, NOT
    /// at the trimmed centerline tip. Pre-S6 the invariant was
    /// asserted via the pin Solid's AABB; post-S6 it lives in the
    /// shaft transform's parent geometry.
    #[test]
    fn plug_pin_anchors_at_cap_plane_centroid_past_trimmed_centerline_tip() {
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.053);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));

        let transforms = build_plug_self_transforms(&ribbon);
        let shaft = params_of(&transforms[0]);
        let c = shaft.parent.center_m;
        // Shaft center = cap_centroid + cap_normal * pin_length / 2
        //              = (0, 0, -0.053) + (0, 0, -1) * 0.010
        //              = (0, 0, -0.063).
        assert!(
            (c.z - -0.063).abs() < 1e-9,
            "shaft center anchored at cap_centroid + outward*half_length; got {c:?}",
        );
        assert!(
            c.z < -0.013,
            "shaft anchor must NOT regress to centerline.last() (z = -0.013); got {}",
            c.z,
        );
    }

    /// Default (no hint) fallback: pour-end maps to
    /// `centerline.last()` per cf-scan-prep's tip→base convention.
    /// Sanity-checks the fallback rule at the transform-parameter
    /// level (pre-S6 was Solid-AABB).
    #[test]
    fn plug_pin_default_fallback_uses_centerline_last() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.054)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let transforms = build_plug_self_transforms(&ribbon);
        let shaft = params_of(&transforms[0]);
        // pour anchor = centerline.last() = (0, 0, -0.054); outward
        // = last_segment.tangent = -Z (segment +0.073 → -0.054).
        // Shaft center = (0, 0, -0.054) + (0, 0, -1) * 0.010 =
        // (0, 0, -0.064).
        assert!((shaft.parent.center_m.z - -0.064).abs() < 1e-9);
    }

    /// `pour_end_t_bar_geometry` continues to return the bare plug-
    /// side T-bar dimensions (consumed by `platform::build_platform_solid`
    /// to size the workshop pocket). Post-S6 this fn routes through
    /// `build_plug_mating_parents`; the output contract is unchanged.
    #[test]
    fn pour_end_t_bar_geometry_returns_bare_plug_side_dimensions() {
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let (center, axis, r, h) = pour_end_t_bar_geometry(&ribbon).unwrap();
        let spec = PlugPinSpec::iter1();
        // Bare T-bar dimensions (no clearance inflation).
        assert!((r - spec.t_bar_radius_m).abs() < f64::EPSILON);
        assert!((h - spec.t_bar_half_length_m).abs() < f64::EPSILON);
        // T-bar center = pour_anchor + pour_outward * pin_length =
        // (0,0,-0.054) + (0,0,-1) * 0.020 = (0,0,-0.074).
        assert!((center.z - -0.074).abs() < 1e-9);
        // T-bar axis = pour_outward × split_normal = (-Z) × (+X) =
        // -Y. (Parallel to seam-plane normal — bisection per recon
        // §5.)
        assert!(axis.x.abs() < 1e-9);
        assert!((axis.y - -1.0).abs() < 1e-9);
        assert!(axis.z.abs() < 1e-9);
    }

    /// `pour_end_t_bar_geometry` returns `None` when the T-bar is
    /// disabled — platform consumer skips emitting `platform.stl`
    /// in that case.
    #[test]
    fn pour_end_t_bar_geometry_returns_none_when_include_t_bar_false() {
        let spec = PlugPinSpec {
            include_t_bar: false,
            ..PlugPinSpec::iter1()
        };
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(spec));
        assert!(pour_end_t_bar_geometry(&ribbon).is_none());
    }
}
