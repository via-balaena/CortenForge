//! Per-layer gasket mold composition for v2 cup-half seam sealing.
//!
//! S1 of `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md` — the gasket mold
//! is a flat tray with a closed-loop channel that workshop user
//! pours silicone into. The cured gasket is a thin closed-loop strip
//! shaped like the cup-piece body-cavity perimeter at the seam plane.
//! Workshop installs the gasket on the Negative cup-half seam face
//! before closing the Positive half and pouring the layer's silicone
//! — the compressible silicone-on-silicone interface absorbs FDM
//! print tolerance (~200 µm) + STL-level MC cell-quantization
//! mismatch (~3 mm one-cell width per
//! `~/scans/cast_iter1_bare/` cross-half match probe from
//! 2026-05-25 head-architect session).
//!
//! See `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md` §G-1 — §G-6 for the
//! locked design decisions and §G-13 S1 for this module's scope.
//!
//! # Geometry overview
//!
//! - **Gasket** (the silicone strip user gets after pouring + curing
//!   into this mold): a thin closed-loop tube with rectangular
//!   cross-section ([`GasketSpec::cross_section_width_m`] wide ×
//!   [`GasketSpec::cross_section_thickness_m`] tall) sweeping along
//!   the body-cavity perimeter on the seam plane.
//! - **Gasket mold** (the printed PLA artifact this module composes):
//!   a flat rectangular tray with a closed-loop channel carved into
//!   its top face. Channel cross-section matches the gasket
//!   cross-section. User pours silicone in from above, screeds
//!   level, cures, peels gasket out.
//!
//! # SDF composition
//!
//! ```text
//! mold = tray_cuboid ∖ channel_sdf
//!
//! channel_sdf(P) = max(
//!     |body.evaluate(P_projected_to_seam)| - cross_section_width/2,
//!     |P.y - channel_y_center|             - channel_depth/2
//! )
//! ```
//!
//! `body.evaluate(P_projected_to_seam)` reads the layer body's SDF
//! at the seam-plane projection of `P` — captures the body cavity
//! perimeter shape at the seam plane regardless of `P.y` (so the
//! channel traces the perimeter even when probed away from the
//! seam plane). `|body_sdf| < width/2` means `P` is laterally
//! within `width/2` of the perimeter curve; combined with the
//! vertical box at `channel_y_center ± depth/2`, the result is the
//! 3D channel sweep.
//!
//! # Coordinate frame
//!
//! Mold sits in the SAME world frame as the cup pieces (no
//! translation). The tray's Y axis aligns with cup-world Y (the
//! seam-normal direction). Per `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`
//! §G-3, the tray top face hosts the channel opening; bottom face
//! sits on the printer bed. The seam-plane projection Y for body
//! evaluation = the ribbon's arc-midpoint Y per
//! [`Ribbon::seam_plane_reference`].
//!
//! # Scope (S1)
//!
//! S1 ships the SDF composition + tests. Out of scope per §G-13:
//! - cf-cast-cli integration (`[gasket]` config block, STL
//!   emission) — S3 territory.
//! - `procedure.rs` workshop protocol prose — S4 territory.
//! - Channel cross-section + material calibration — S2 territory.
//! - Pin-routing (§G-5 (a)/(b)/(c) variants) — S3 territory.
//! - Dovetail retention (§G-3 fallback) — S5 territory.

use cf_design::{Aabb, Sdf, Solid};
use nalgebra::{Point3, Vector3};

use crate::error::{CastError, CastTarget};
use crate::mesh_csg::MatingTransform;
use crate::ribbon::Ribbon;

/// Default cross-section width for the iter-1 gasket (~1.5 mm).
///
/// Picked at recon §G-1 typed-range midpoint. S2 narrows via
/// compression calibration against Ecoflex 00-30 shore hardness
/// vs workshop clamping pressure. FDM-printable: well above the
/// Bambu A1 default 0.4 mm extrusion-width minimum + leaves room
/// for the channel's two side walls (>= 0.6 mm each).
const DEFAULT_CROSS_SECTION_WIDTH_M: f64 = 0.0015;

/// Default cross-section thickness (gasket height perpendicular to
/// the seam plane; ~0.8 mm).
///
/// Picked at recon §G-1 typed-range midpoint. Below 0.5 mm and the
/// gasket may not bridge FDM-tolerance gaps (~200 µm); above 1 mm
/// and the gasket may not compress enough to let halves close. S2
/// calibrates against the §G-3 target FDM floor.
const DEFAULT_CROSS_SECTION_THICKNESS_M: f64 = 0.0008;

/// Default tray total thickness (3 mm). Sized so the channel
/// (depth = cross-section thickness) carves a notch into the top
/// without weakening the tray base below the FDM minimum-wall floor.
const DEFAULT_TRAY_THICKNESS_M: f64 = 0.003;

/// Default lateral margin around the bounding-region XZ projection
/// when sizing the tray (5 mm each side). Gives workshop user a
/// rim to grab + handle the tray during pour + demold.
const DEFAULT_TRAY_MARGIN_M: f64 = 0.005;

/// Gasket mold parameter envelope per recon §G-1 + §G-3.
///
/// S2 narrows numeric values via compression calibration vs Ecoflex
/// 00-30 / Dragon Skin 10A shore hardness; S6 workshop physical
/// iter-3 pins the final defaults. See
/// `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`.
#[derive(Debug, Clone, Copy)]
pub struct GasketSpec {
    /// Gasket cross-section width (perpendicular to perimeter curve,
    /// in the seam plane). Default [`DEFAULT_CROSS_SECTION_WIDTH_M`]
    /// (1.5 mm).
    pub cross_section_width_m: f64,
    /// Gasket cross-section thickness (perpendicular to seam plane,
    /// along the cup-world Y axis). Default
    /// [`DEFAULT_CROSS_SECTION_THICKNESS_M`] (0.8 mm).
    pub cross_section_thickness_m: f64,
    /// Tray total Y extent (printer-bed-vertical). Default
    /// [`DEFAULT_TRAY_THICKNESS_M`] (3 mm).
    pub tray_thickness_m: f64,
    /// Lateral margin around the bounding region's XZ projection when
    /// sizing the tray. Default [`DEFAULT_TRAY_MARGIN_M`] (5 mm).
    pub tray_margin_m: f64,
    /// Channel depth into tray top (`<= cross_section_thickness_m` to
    /// keep the gasket fully contained; `> tray_thickness_m` would
    /// pierce the tray base). Default = cross-section thickness.
    pub channel_depth_m: f64,
}

impl GasketSpec {
    /// Workshop iter-1 starting defaults pinned at recon §G-1 typed-range
    /// midpoints. S2 calibration narrows.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            cross_section_width_m: DEFAULT_CROSS_SECTION_WIDTH_M,
            cross_section_thickness_m: DEFAULT_CROSS_SECTION_THICKNESS_M,
            tray_thickness_m: DEFAULT_TRAY_THICKNESS_M,
            tray_margin_m: DEFAULT_TRAY_MARGIN_M,
            channel_depth_m: DEFAULT_CROSS_SECTION_THICKNESS_M,
        }
    }
}

impl Default for GasketSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// SDF adapter for the gasket channel.
///
/// Evaluates the layer body's SDF at the seam-plane projection of
/// the query point, then composes lateral (width) + vertical (depth)
/// box constraints to define the channel as a 3D swept tube.
#[derive(Debug, Clone)]
struct GasketChannelSdf {
    body: Solid,
    seam_plane_y: f64,
    channel_y_min: f64,
    channel_y_max: f64,
    width_m: f64,
}

impl Sdf for GasketChannelSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let projected = Point3::new(p.x, self.seam_plane_y, p.z);
        let body_dist = self.body.evaluate(&projected);
        let lateral = 0.5_f64.mul_add(-self.width_m, body_dist.abs());
        let y_center = 0.5 * (self.channel_y_min + self.channel_y_max);
        let y_extent = 0.5 * (self.channel_y_max - self.channel_y_min);
        let vertical = (p.y - y_center).abs() - y_extent;
        lateral.max(vertical)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // The max() composition's analytical gradient is multivalued
        // at facet boundaries (lateral-vs-vertical switch). Per the
        // ribbon halfspace pattern: `Solid::from_sdf` bridges this via
        // `FieldNode::UserFn` which always uses finite differences, so
        // this analytical grad is unused. Provided for `Sdf` contract
        // completeness. Picks the +Y direction arbitrarily (matches
        // the channel's "open at top" orientation).
        Vector3::new(0.0, 1.0, 0.0)
    }
}

/// Compose the per-layer gasket mold geometry as a flat tray with a
/// closed-loop channel matching the cup-piece body-cavity perimeter
/// at the seam plane.
///
/// Returns `(Solid, Vec<MatingTransform>)` — the second element is
/// empty in S1 (per §G-13 scope). S3 may append mating transforms
/// for the §G-5 (a)/(b) pin-routing variants (notch-outs at cup-pin
/// footprints); S1 default `(c)` continuous channel needs no
/// transforms.
///
/// # Geometry
///
/// - **Tray** centered at `(0, tray_thickness / 2, 0)` (cup-world
///   coords). Bottom face at `Y = 0` (printer-bed side); top face
///   at `Y = tray_thickness_m`. XZ extents = bounding-region XZ
///   bounds plus `tray_margin_m` on each side.
/// - **Channel** at the tray top: lateral extent traces the body
///   perimeter at the seam plane (where
///   `|body.evaluate(P_at_seam_plane)| < cross_section_width_m / 2`);
///   vertical extent `Y ∈ [tray_thickness - channel_depth,
///   tray_thickness]` (carves into tray from the top).
///
/// # Errors
///
/// - [`CastError::InfiniteBounds`] with [`CastTarget::BoundingRegion`]
///   if `bounding_region.bounds()` returns `None` — the tray
///   dimensions derive from the bounding region's XZ extent.
pub fn compose_gasket_mold_solid(
    ribbon: &Ribbon,
    layer_body: &Solid,
    bounding_region: &Solid,
    spec: &GasketSpec,
) -> Result<(Solid, Vec<MatingTransform>), CastError> {
    let region_bounds = bounding_region
        .bounds()
        .ok_or(CastError::InfiniteBounds(CastTarget::BoundingRegion))?;

    let half_x = 0.5_f64.mul_add(region_bounds.size().x, spec.tray_margin_m);
    let half_z = 0.5_f64.mul_add(region_bounds.size().z, spec.tray_margin_m);
    let half_thickness = 0.5 * spec.tray_thickness_m;

    // Tray cuboid centered at (0, tray_thickness / 2, 0). Bottom face
    // sits on the printer bed at Y = 0; top face at Y = tray_thickness.
    let tray_solid = Solid::cuboid(Vector3::new(half_x, half_thickness, half_z))
        .translate(Vector3::new(0.0, half_thickness, 0.0));

    // Seam plane Y from the ribbon arc-midpoint reference.
    let (seam_midpoint, _seam_normal) = ribbon.seam_plane_reference();
    let seam_plane_y = seam_midpoint.y;

    let channel_y_max = spec.tray_thickness_m;
    let channel_y_min = spec.tray_thickness_m - spec.channel_depth_m;

    // Channel SDF bounds: at minimum cover the tray top region plus a
    // small Y margin so MC sees the channel's full vertical extent.
    // X/Z bounds match the tray (the channel sweep is contained within
    // the tray's XZ projection).
    let channel_bounds = Aabb::new(
        Point3::new(-half_x, channel_y_min - spec.channel_depth_m, -half_z),
        Point3::new(half_x, channel_y_max + spec.channel_depth_m, half_z),
    );

    let channel_solid = Solid::from_sdf(
        GasketChannelSdf {
            body: layer_body.clone(),
            seam_plane_y,
            channel_y_min,
            channel_y_max,
            width_m: spec.cross_section_width_m,
        },
        channel_bounds,
    );

    let mold = tray_solid.subtract(channel_solid);

    // S3 territory: append mating transforms for §G-5 pin-routing
    // variants. S1 default `(c)` continuous channel needs no
    // transforms.
    Ok((mold, Vec::new()))
}

#[cfg(test)]
mod tests {
    // Workspace lint policy allows unwrap/panic in tests for clean
    // assertion-failure prose; matches `spec.rs` + `piece.rs`.
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::ribbon::SplitNormal;
    use approx::assert_relative_eq;

    /// Synthetic fixture mirroring production seam-plane geometry:
    /// straight-along-X centerline at Y=0; cylindrical body cavity
    /// (sock proxy) centered on the centerline; large bounding cuboid.
    /// The body's XZ cross-section at the seam plane is a circle of
    /// radius 10 mm — `compose_gasket_mold_solid` should produce a
    /// channel tracing this circular perimeter.
    fn synthetic_fixture() -> (Ribbon, Solid, Solid) {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        // Body cavity: 10 mm radius cylinder along X (the centerline).
        // Cross-section at seam plane (XZ projection): a circle of
        // radius 10 mm centered at (any X, 0, 0).
        let layer_body =
            Solid::cylinder(0.010, 0.060).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        // Bounding cuboid: 200 × 30 × 30 mm centered at origin.
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.015, 0.015));
        (ribbon, layer_body, bounding_region)
    }

    #[test]
    fn gasket_spec_iter1_defaults_within_recon_envelope() {
        // §G-1 typed-range envelope: cross-section width 1.0-2.0 mm,
        // thickness 0.5-1.0 mm. iter1 midpoints must fall inside.
        let s = GasketSpec::iter1();
        assert!(
            (0.001..=0.002).contains(&s.cross_section_width_m),
            "iter1 cross_section_width_m {} not in [1, 2] mm",
            s.cross_section_width_m
        );
        assert!(
            (0.0005..=0.001).contains(&s.cross_section_thickness_m),
            "iter1 cross_section_thickness_m {} not in [0.5, 1] mm",
            s.cross_section_thickness_m
        );
        assert!(
            s.tray_thickness_m > s.channel_depth_m,
            "tray must be thicker than channel depth (otherwise channel pierces tray base): \
             tray {} vs channel {}",
            s.tray_thickness_m,
            s.channel_depth_m
        );
        assert_eq!(
            s.channel_depth_m, s.cross_section_thickness_m,
            "iter1 channel depth must match gasket thickness so cured gasket fills the channel"
        );
    }

    #[test]
    fn gasket_spec_default_matches_iter1() {
        let d = GasketSpec::default();
        let i = GasketSpec::iter1();
        assert_eq!(d.cross_section_width_m, i.cross_section_width_m);
        assert_eq!(d.cross_section_thickness_m, i.cross_section_thickness_m);
        assert_eq!(d.tray_thickness_m, i.tray_thickness_m);
        assert_eq!(d.tray_margin_m, i.tray_margin_m);
        assert_eq!(d.channel_depth_m, i.channel_depth_m);
    }

    #[test]
    fn compose_gasket_mold_rejects_infinite_bounds() {
        let (ribbon, body, _) = synthetic_fixture();
        // A bare plane has infinite bounds → bounding_region.bounds()
        // returns None → InfiniteBounds(BoundingRegion).
        let infinite = Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0);
        let err =
            compose_gasket_mold_solid(&ribbon, &body, &infinite, &GasketSpec::iter1()).unwrap_err();
        assert!(
            matches!(err, CastError::InfiniteBounds(CastTarget::BoundingRegion)),
            "expected InfiniteBounds(BoundingRegion); got {err:?}"
        );
    }

    #[test]
    fn compose_gasket_mold_returns_empty_transforms_at_s1() {
        // S1 ships SDF composition only; mating transforms are S3
        // territory (§G-5 pin-routing variants).
        let (ribbon, body, bounds) = synthetic_fixture();
        let (_mold, transforms) =
            compose_gasket_mold_solid(&ribbon, &body, &bounds, &GasketSpec::iter1()).unwrap();
        assert_eq!(
            transforms.len(),
            0,
            "S1 must return empty MatingTransform vec; got {} transforms",
            transforms.len()
        );
    }

    #[test]
    fn compose_gasket_mold_tray_bounds_cover_bounding_region_with_margin() {
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let mold_bounds = mold.bounds().expect("finite tray must have bounds");
        let region_bounds = bounds.bounds().unwrap();
        // Tray X extent = region_x + 2 * margin.
        assert_relative_eq!(
            mold_bounds.size().x,
            2.0_f64.mul_add(spec.tray_margin_m, region_bounds.size().x),
            epsilon = 1e-9,
        );
        assert_relative_eq!(
            mold_bounds.size().z,
            2.0_f64.mul_add(spec.tray_margin_m, region_bounds.size().z),
            epsilon = 1e-9,
        );
        // Tray Y extent = tray_thickness (translated so base sits at Y=0).
        assert_relative_eq!(mold_bounds.min.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(mold_bounds.max.y, spec.tray_thickness_m, epsilon = 1e-9);
    }

    #[test]
    fn gasket_channel_carves_void_at_body_perimeter_seam_plane_height() {
        // With-channel assertion paired with bare-baseline per
        // [[feedback-load-bearing-test-fixtures]]: probe at a point
        // ON the body perimeter (body.evaluate = 0) at the channel
        // Y height. The mold SDF should be POSITIVE (exterior of mold
        // = inside channel void). Bare baseline: a tray with no
        // channel is solid at the same probe (SDF NEGATIVE).
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        // Body cavity is 10 mm radius cylinder along X centered on
        // (X, 0, 0). At the channel Y (top of tray), probe on the
        // perimeter at (X=0, Y=tray_top, Z=+10 mm).
        let perimeter = Point3::new(
            0.0,
            0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m),
            0.010,
        );
        let mold_sdf = mold.evaluate(&perimeter);
        assert!(
            mold_sdf > 0.0,
            "mold SDF at body perimeter (mid-channel) must be > 0 (inside channel void); \
             got {mold_sdf}"
        );
        // Bare baseline: a tray with NO channel is solid everywhere.
        let half_thickness = 0.5 * spec.tray_thickness_m;
        let region_size = bounds.bounds().unwrap().size();
        let bare_tray = Solid::cuboid(Vector3::new(
            0.5_f64.mul_add(region_size.x, spec.tray_margin_m),
            half_thickness,
            0.5_f64.mul_add(region_size.z, spec.tray_margin_m),
        ))
        .translate(Vector3::new(0.0, half_thickness, 0.0));
        let bare_sdf = bare_tray.evaluate(&perimeter);
        assert!(
            bare_sdf < 0.0,
            "bare tray (no channel) must be solid (SDF < 0) at the same probe; got {bare_sdf}. \
             If this fails, the fixture probe is outside the tray bounds — the with-channel \
             assertion above is uninformative."
        );
    }

    #[test]
    fn gasket_channel_excludes_body_cavity_interior_at_channel_height() {
        // At a probe WELL INSIDE the body cavity (body.evaluate << -w/2)
        // at the channel Y height: the channel SDF should be POSITIVE
        // (outside channel laterally); the tray subtract preserves tray
        // material → mold SDF NEGATIVE (solid tray material).
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        // Cavity interior: 5 mm from center, well inside 10 mm radius
        // cavity → body.evaluate ≈ -5 mm, |body.evaluate| > 5 mm >>
        // width/2 = 0.75 mm → channel SDF positive → mold solid.
        let cavity_interior = Point3::new(
            0.0,
            0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m),
            0.005,
        );
        let mold_sdf = mold.evaluate(&cavity_interior);
        assert!(
            mold_sdf < 0.0,
            "mold SDF inside body cavity (where channel does NOT trace) must be < 0 \
             (solid tray material); got {mold_sdf}"
        );
    }

    #[test]
    fn gasket_channel_excludes_bounding_exterior_at_channel_height() {
        // At a probe FAR OUTSIDE the body cavity (body.evaluate >> +w/2)
        // but inside the tray bounds: mold SDF NEGATIVE (solid tray).
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        // Outside cavity by ~3 mm: body.evaluate ≈ +3 mm, |body.evaluate|
        // = 3 mm >> width/2 = 0.75 mm → channel SDF positive → mold solid.
        let outside_cavity = Point3::new(
            0.0,
            0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m),
            0.013,
        );
        let mold_sdf = mold.evaluate(&outside_cavity);
        assert!(
            mold_sdf < 0.0,
            "mold SDF outside body cavity (where channel does NOT trace) must be < 0 \
             (solid tray material); got {mold_sdf}"
        );
    }

    #[test]
    fn gasket_channel_width_matches_spec_at_perimeter_offset() {
        // Bit-precise fit invariant per [[project-cf-cast-mating-features-s5-registration-pins]]
        // §"math-verified clearance": probe at (body.evaluate = +w/2)
        // and (body.evaluate = -w/2) — both should land at the channel
        // boundary (channel SDF ≈ 0). The cavity radius is 10 mm; the
        // channel width is 1.5 mm so the channel boundary lies at
        // r = 10 ± 0.75 mm.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let y_mid = 0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m);
        let half_w = 0.5 * spec.cross_section_width_m;
        // Outer boundary of channel: r = 10 mm + 0.75 mm = 10.75 mm.
        let outer_boundary = Point3::new(0.0, y_mid, 0.010 + half_w);
        let outer_sdf = mold.evaluate(&outer_boundary);
        // Channel SDF ≈ 0 at the boundary → mold SDF (tray subtract
        // channel) ≈ 0 too. Tolerate small numerical error from
        // composition.
        assert!(
            outer_sdf.abs() < 1e-6,
            "mold SDF at outer channel boundary (r = 10.75 mm) must be ≈ 0; got {outer_sdf}"
        );
        // Inner boundary: r = 10 mm - 0.75 mm = 9.25 mm.
        let inner_boundary = Point3::new(0.0, y_mid, 0.010 - half_w);
        let inner_sdf = mold.evaluate(&inner_boundary);
        assert!(
            inner_sdf.abs() < 1e-6,
            "mold SDF at inner channel boundary (r = 9.25 mm) must be ≈ 0; got {inner_sdf}"
        );
    }

    #[test]
    fn gasket_channel_depth_matches_spec_at_tray_top() {
        // Vertical extent invariant: probe at the channel-Y boundaries
        // along the perimeter line. Channel exists from
        // `tray_thickness - channel_depth` to `tray_thickness`. At the
        // channel's bottom face (Y = tray_thickness - channel_depth)
        // ON the perimeter, the mold SDF transitions from solid (just
        // below) to void (just above). Probe just above and just below
        // and assert the sign flip.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let perimeter_x = 0.0;
        let perimeter_z = 0.010; // body-cavity radius
        let just_above_channel_floor = Point3::new(
            perimeter_x,
            spec.tray_thickness_m - spec.channel_depth_m + 1e-6,
            perimeter_z,
        );
        let just_below_channel_floor = Point3::new(
            perimeter_x,
            spec.tray_thickness_m - spec.channel_depth_m - 1e-6,
            perimeter_z,
        );
        let above_sdf = mold.evaluate(&just_above_channel_floor);
        let below_sdf = mold.evaluate(&just_below_channel_floor);
        assert!(
            above_sdf > 0.0,
            "just ABOVE channel floor must be in void (SDF > 0); got {above_sdf}"
        );
        assert!(
            below_sdf < 0.0,
            "just BELOW channel floor must be in solid tray (SDF < 0); got {below_sdf}. \
             If this fails, channel_depth is consuming the full tray thickness — fixture bug \
             or spec misconfiguration (channel_depth >= tray_thickness)."
        );
    }

    #[test]
    fn gasket_channel_open_at_tray_top() {
        // The channel must open at the top face — probe just above the
        // top face on the perimeter line. The point is outside the
        // tray's Y extent, so mold SDF > 0 trivially (above tray top).
        // This test gates against a future bug that makes the channel
        // a SEALED tube (closed at top) — which would make the gasket
        // unrecoverable from the mold.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let above_top = Point3::new(0.0, spec.tray_thickness_m + 1e-6, 0.010);
        let above_sdf = mold.evaluate(&above_top);
        assert!(
            above_sdf > 0.0,
            "above tray top must be exterior (SDF > 0); got {above_sdf}"
        );
        // And the channel TOP (Y = tray_thickness) on the perimeter
        // should ALSO be at the boundary or void — the channel opens
        // there. Probe just below the top face on the perimeter and
        // assert void.
        let just_below_top = Point3::new(0.0, spec.tray_thickness_m - 1e-6, 0.010);
        let below_top_sdf = mold.evaluate(&just_below_top);
        assert!(
            below_top_sdf > 0.0,
            "just below tray top ON perimeter must be in channel void (SDF > 0); \
             got {below_top_sdf}"
        );
    }
}
