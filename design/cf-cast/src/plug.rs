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
//! Per `project_cf_cast_workshop_conventions.md` §"v2.1
//! architectural recommendations":
//!
//! - The pour-end pin is centered at `centerline[0]` and extends
//!   **outward along `-first_segment.tangent`** (away from the body
//!   interior). The plug's natural cap region partially overlaps
//!   the pin near `base`; the union is well-defined (boolean OR of
//!   the two regions).
//! - The optional dome-end pin (enabled via
//!   [`PlugPinSpec::include_dome_pin`]) mirrors the pour-end pin at
//!   `centerline.last()` extending along `+last_segment.tangent`.
//! - Sockets are the **same cylinder geometry** with a larger
//!   radius ([`PlugPinSpec::socket_radial_slack_m`] adds to the pin
//!   radius). Where the socket cylinder overlaps the layer body, the
//!   subtraction is a no-op (the piece's cup wall already excludes
//!   that region); where it overlaps cup-wall material it carves a
//!   slide-fit channel that receives the pin.
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
//! (layer N>0) and internally calls [`add_plug_pins`] to extend
//! it with the pin geometry, then [`crate::piece::compose_piece_solid`]
//! subtracts the matching sockets. The `plug_pins` field on the
//! ribbon is the single source of truth that both call paths
//! consult, so sockets-without-pins (jams the plug) and
//! pins-without-sockets (prevents mold close) can't drift apart.
//!
//! [`add_plug_pins`] remains public for callers building a custom
//! plug `Solid` outside [`CastSpec`]'s per-layer-plug derivation
//! (e.g., a one-off cast with a hand-tuned plug shape).
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new
//! [`Ribbon::with_plug_pins`]: crate::ribbon::Ribbon::with_plug_pins
//! [`CastSpec`]: crate::CastSpec

use cf_design::Solid;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::ribbon::Ribbon;

/// Axial plug-anchor pin geometry spec. All dimensions in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PlugPinSpec {
    /// Pin radius (m). Default `0.003` = 3 mm = 6 mm diameter.
    ///
    /// 6 mm Ø ships **wider** than [`crate::registration::PinSpec`]'s
    /// 3 mm Ø inter-piece pin radius. The reason is mesh resolution:
    /// inter-piece pins are CSG'd into a much-larger mold piece (the
    /// surrounding cup-wall surface gives marching cubes plenty of
    /// neighboring samples to resolve the pin as a protrusion), but
    /// the plug-anchor pin sticks out into otherwise-empty space
    /// past the plug cap and must be at least 2 cells across in
    /// every axis to resolve cleanly. At cf-cast's default 3 mm
    /// mesh-cell size (`MESH_CELL_SIZE_M` in the iter-1 example), a
    /// 3 mm Ø pin captures as only 1 cell radially and MC produces
    /// disconnected tip fragments; 6 mm Ø clears the threshold.
    /// FDM hole accuracy is ±0.2 mm; the socket adds
    /// [`Self::socket_radial_slack_m`] on top for slide-fit.
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
    /// Radial slack between the pin and the socket cylinder (m).
    /// Default `0.0005` = 0.5 mm. The socket radius is
    /// `pin_radius_m + socket_radial_slack_m`, so the diametric gap
    /// is `2 × socket_radial_slack_m` = 1 mm at the default.
    ///
    /// FDM holes typically print smaller than nominal by ~0.1-0.2 mm;
    /// 0.5 mm radial slack provides slide-fit clearance for typical
    /// FDM accuracy. Tighter fits jam; looser fits wobble. Workshop
    /// iter-1 reams to taste.
    pub socket_radial_slack_m: f64,
    /// Whether to add a matching pin at the dome end
    /// (`centerline.last()`). Default `false` — the plug's
    /// hemispherical cap seats naturally into the body cavity's
    /// matching dome for centerline alignment at the un-pinned
    /// end, so a second pin would only punch an unneeded leak path
    /// through the cup wall at the dome. High-curvature centerlines
    /// (max tangent rotation approaching 120°) may need dome-end
    /// rotation constraint and opt in here.
    pub include_dome_pin: bool,
}

impl PlugPinSpec {
    /// v2.1 iter-1 defaults: single pour-end pin, 6 mm Ø × 20 mm
    /// long, 0.5 mm radial socket slack (7 mm Ø socket). Pin Ø >
    /// [`crate::registration::PinSpec`]'s 3 mm to clear the
    /// marching-cubes resolution threshold at the 3 mm default
    /// cell size; see [`Self::pin_radius_m`] docstring. Dome-end
    /// pin disabled per [`Self::include_dome_pin`] default.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            pin_radius_m: 0.003,
            pin_length_m: 0.020,
            socket_radial_slack_m: 0.0005,
            include_dome_pin: false,
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

/// Build the combined plug-pin Solid for the ribbon's plug-pin
/// kind, or `None` if [`PlugPinKind::None`].
///
/// Returns `Some(solid)` whose SDF is the union of two cylinders:
/// one centered on `centerline[0]` extending along
/// `-first_segment.tangent`, and one centered on `centerline.last()`
/// extending along `+last_segment.tangent`. The plug's natural cap
/// region overlaps the inner half of each pin; the union with the
/// plug `Solid` resolves cleanly via boolean OR.
///
/// Used by [`add_plug_pins`] to extend a user-supplied plug `Solid`
/// with the matched-to-socket pin geometry.
#[must_use]
pub fn build_plug_pin_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.plug_pins {
        PlugPinKind::None => return None,
        PlugPinKind::Axial(spec) => spec,
    };
    let first = ribbon.segments.first()?;
    let pin_at_start = anchor_cylinder(
        first.start,
        -first.tangent,
        spec.pin_radius_m,
        spec.pin_length_m,
    );
    if !spec.include_dome_pin {
        return Some(pin_at_start);
    }
    let last = ribbon.segments.last()?;
    let pin_at_end = anchor_cylinder(last.end, last.tangent, spec.pin_radius_m, spec.pin_length_m);
    Some(pin_at_start.union(pin_at_end))
}

/// Build the combined plug-socket Solid for the ribbon's plug-pin
/// kind, or `None` if [`PlugPinKind::None`].
///
/// Returns `Some(solid)` whose SDF is the union of two cylinders
/// positioned identically to [`build_plug_pin_solid`]'s output but
/// with the socket radius (`pin_radius_m + socket_radial_slack_m`).
/// Used by [`crate::piece::compose_piece_solid`] to carve the socket
/// channels into each mold piece via CSG subtraction. The same
/// socket cylinder is subtracted from BOTH [`crate::ribbon::PieceSide`]s
/// — sockets straddle the ribbon seam since centerline endpoints
/// lie on or near it for typical cf-scan-prep outputs.
#[must_use]
pub fn build_plug_socket_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.plug_pins {
        PlugPinKind::None => return None,
        PlugPinKind::Axial(spec) => spec,
    };
    let first = ribbon.segments.first()?;
    let socket_radius = spec.pin_radius_m + spec.socket_radial_slack_m;
    let socket_at_start = anchor_cylinder(
        first.start,
        -first.tangent,
        socket_radius,
        spec.pin_length_m,
    );
    if !spec.include_dome_pin {
        return Some(socket_at_start);
    }
    let last = ribbon.segments.last()?;
    let socket_at_end = anchor_cylinder(last.end, last.tangent, socket_radius, spec.pin_length_m);
    Some(socket_at_start.union(socket_at_end))
}

/// Extend a user-supplied plug [`Solid`] with axial pin cylinders
/// per the ribbon's [`PlugPinKind`].
///
/// Returns the plug unchanged when [`Ribbon::plug_pins`] is
/// [`PlugPinKind::None`]; otherwise returns `plug.union(pins)` where
/// `pins` is [`build_plug_pin_solid`]'s output.
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
pub fn add_plug_pins(plug: Solid, ribbon: &Ribbon) -> Solid {
    match build_plug_pin_solid(ribbon) {
        Some(pins) => plug.union(pins),
        None => plug,
    }
}

/// Build a cylinder of `radius` × `length` centered along its axis
/// such that one end is exactly at `base` and the cylinder extends
/// `length` outward along `outward_axis`.
///
/// Used by [`build_plug_pin_solid`] (with the pin radius) and
/// [`build_plug_socket_solid`] (with the slack-augmented socket
/// radius) so the two solids share the same axis/length geometry and
/// only differ in radius.
fn anchor_cylinder(
    base: Point3<f64>,
    outward_axis: Vector3<f64>,
    radius: f64,
    length: f64,
) -> Solid {
    let half_length = length / 2.0;
    let center = base + outward_axis * half_length;
    let rotation = UnitQuaternion::rotation_between(&Vector3::z_axis().into_inner(), &outward_axis)
        .unwrap_or_else(UnitQuaternion::identity);
    Solid::cylinder(radius, half_length)
        .rotate(rotation)
        .translate(center.coords)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]

    use super::*;
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    /// Standard test fixture — a 100 mm centerline along +X with +Y
    /// split-normal. `first_segment.tangent = +X` so the start pin
    /// extends along `-X`; `last_segment.tangent = +X` so the end pin
    /// extends along `+X`.
    fn straight_x_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split).unwrap()
    }

    #[test]
    fn plug_pin_spec_iter1_has_workshop_defaults() {
        let s = PlugPinSpec::iter1();
        assert!((s.pin_radius_m - 0.003).abs() < f64::EPSILON);
        assert!((s.pin_length_m - 0.020).abs() < f64::EPSILON);
        assert!((s.socket_radial_slack_m - 0.0005).abs() < f64::EPSILON);
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

    #[test]
    fn build_plug_pin_solid_none_returns_none() {
        let ribbon = straight_x_ribbon();
        assert!(build_plug_pin_solid(&ribbon).is_none());
    }

    #[test]
    fn build_plug_socket_solid_none_returns_none() {
        let ribbon = straight_x_ribbon();
        assert!(build_plug_socket_solid(&ribbon).is_none());
    }

    #[test]
    fn add_plug_pins_passes_through_when_kind_is_none() {
        let ribbon = straight_x_ribbon();
        let plug = Solid::capsule(0.005, 0.020);
        // Sample a few points on the bare plug + the extended-plug
        // SDFs; they should agree exactly when plug_pins is None.
        let extended = add_plug_pins(plug.clone(), &ribbon);
        for q in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.025),
        ] {
            let bare = plug.evaluate(&q);
            let ext = extended.evaluate(&q);
            assert!(
                (bare - ext).abs() < 1e-12,
                "add_plug_pins must pass through plug unchanged when ribbon has no pins; \
                 differed at {q:?}: bare={bare}, extended={ext}",
            );
        }
    }

    #[test]
    fn build_plug_pin_solid_axial_default_ships_only_pour_end_pin() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let pins = build_plug_pin_solid(&ribbon).expect("Axial kind should yield a solid");
        let aabb = pins
            .bounds()
            .expect("single pin cylinder should have finite bounds");
        // Pour-end pin centered at (-50 - 10) = -60 mm along -X,
        // half-length 10 mm → spans x ∈ [-70, -50] mm.
        assert!(
            aabb.min.x <= -0.069,
            "pour-end pin should reach x ≤ -69 mm; got min.x = {}",
            aabb.min.x
        );
        assert!(
            aabb.max.x <= -0.049,
            "no dome-end pin by default: AABB should stop at the start-pin's base (x ≈ -50 mm), \
             not extend to +70 mm; got max.x = {}",
            aabb.max.x
        );
        // y, z extents are bounded by pin radius (3 mm).
        assert!(aabb.min.y >= -0.0035);
        assert!(aabb.max.y <= 0.0035);
    }

    #[test]
    fn build_plug_pin_solid_axial_include_dome_pin_spans_both_endpoints() {
        let spec = PlugPinSpec {
            include_dome_pin: true,
            ..PlugPinSpec::iter1()
        };
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(spec));
        let pins = build_plug_pin_solid(&ribbon).expect("Axial kind should yield a solid");
        let aabb = pins
            .bounds()
            .expect("two finite cylinders should have finite bounds");
        // With include_dome_pin = true, pin AABB spans both endpoints:
        // pour-end x ∈ [-70, -50] mm + dome-end x ∈ [+50, +70] mm.
        assert!(aabb.min.x <= -0.069);
        assert!(aabb.max.x >= 0.069);
    }

    /// On-axis interior point at the pour-end pin should be inside
    /// the pin Solid; the dome-end position (no pin by default) and
    /// far-away points should be outside.
    #[test]
    fn plug_pin_solid_is_inside_only_at_pour_end_pin_by_default() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let pins = build_plug_pin_solid(&ribbon).unwrap();

        // Pour-end pin spans x ∈ [-70, -50] mm. A query at x = -60 mm
        // on the axis is dead-center → inside the pin.
        let on_pour_axis = Point3::new(-0.060, 0.0, 0.0);
        assert!(
            pins.evaluate(&on_pour_axis) < 0.0,
            "pin SDF on pour-end pin axis should be negative; got {}",
            pins.evaluate(&on_pour_axis),
        );

        // Default config has NO dome-end pin → x = +60 mm is outside.
        let on_dome_axis = Point3::new(0.060, 0.0, 0.0);
        assert!(
            pins.evaluate(&on_dome_axis) > 0.0,
            "pin SDF at dome-end position should be positive (no dome pin \
             by default); got {}",
            pins.evaluate(&on_dome_axis),
        );

        // Far from any pin — clearly outside.
        let far = Point3::new(0.100, 0.100, 0.100);
        assert!(
            pins.evaluate(&far) > 0.0,
            "pin SDF far from pins should be positive; got {}",
            pins.evaluate(&far),
        );
    }

    /// Socket has the same axis/length as the pin but larger radius.
    /// A point inside the socket but outside the pin (in the slack
    /// annulus) should be INSIDE the socket Solid and OUTSIDE the pin
    /// Solid.
    #[test]
    fn socket_solid_is_larger_radius_than_pin_solid() {
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let pins = build_plug_pin_solid(&ribbon).unwrap();
        let sockets = build_plug_socket_solid(&ribbon).unwrap();

        // Pin radius 3.0 mm, socket radius 3.5 mm. Query at the
        // start-pin axis midpoint, offset radially 3.2 mm: inside
        // socket, outside pin.
        let in_slack_annulus = Point3::new(-0.060, 0.0032, 0.0);
        assert!(
            pins.evaluate(&in_slack_annulus) > 0.0,
            "pin SDF at slack-annulus point should be positive; got {}",
            pins.evaluate(&in_slack_annulus),
        );
        assert!(
            sockets.evaluate(&in_slack_annulus) < 0.0,
            "socket SDF at slack-annulus point should be negative; got {}",
            sockets.evaluate(&in_slack_annulus),
        );
    }

    /// `add_plug_pins` returns a Solid whose SDF is negative at the
    /// pin region (the pin extends past the bare plug into territory
    /// that was outside the bare plug).
    #[test]
    fn add_plug_pins_extends_plug_into_pin_region() {
        // Bare plug: capsule at origin spanning roughly [-25, +25] mm in z.
        // Centerline along x means pin extends along x — so pin
        // territory (x = -60 mm) is OUTSIDE the bare plug.
        let plug = Solid::capsule(0.005, 0.020);
        let ribbon = straight_x_ribbon().with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
        let extended = add_plug_pins(plug.clone(), &ribbon);

        let in_start_pin = Point3::new(-0.060, 0.0, 0.0);
        // Bare plug at this point is outside (SDF > 0); extended plug
        // at this point is inside the pin (SDF < 0).
        assert!(plug.evaluate(&in_start_pin) > 0.0);
        assert!(
            extended.evaluate(&in_start_pin) < 0.0,
            "extended plug should contain pin region; got {}",
            extended.evaluate(&in_start_pin),
        );
    }
}
