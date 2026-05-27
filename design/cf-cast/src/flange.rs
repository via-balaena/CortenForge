//! Seam-flange composition for the cup-piece clamp-grip surface.
//!
//! S1 of `docs/CF_CAST_SEAM_FLANGE_RECON.md` — the flange is a flat
//! rectangular plate at the seam plane extending OUTWARD from the
//! cup-piece body cavity perimeter. Each cup half gets its
//! mirror-image flange in its own Y half-space (Negative half: -Y;
//! Positive half: +Y). Workshop user grips the flat flange face with
//! C-clamps to apply the design-target clamp pressure
//! ([`crate::GasketSpec::workshop_clamp_pressure_pa`] = 20 kPa at
//! iter-1) needed for the gasket arc's predicted compression to
//! absorb FDM tolerance.
//!
//! See `docs/CF_CAST_SEAM_FLANGE_RECON.md` §F-1 — §F-5 for the
//! locked design decisions and §F-13 S1 for this module's scope.
//!
//! # Geometry overview
//!
//! - **Flange** (the printed PLA plate): flat rectangular ring at
//!   the seam plane (Y = 0 in production cast.toml's split-normal
//!   frame). Inner edge follows the body cavity perimeter at
//!   `body_dist = flange_inner_offset_m` (default 2 mm — keeps the
//!   flange material LATERALLY DISJOINT from the gasket channel
//!   per §F-4); outer edge at `body_dist = flange_width_m` (default
//!   20 mm in iter-1 — sized to land the M5 through-bolt clamp
//!   pattern with safe walls on both sides of the bolt hole). Thickness
//!   `flange_thickness_m` (default 4 mm PER HALF) extends
//!   perpendicular to the seam plane into the cup half's Y region.
//!
//! - **Per-half symmetry:** both cup halves get a flange in their
//!   respective Y half-space; the two flanges meet at the seam
//!   plane when the cup closes; gasket strip is sandwiched between
//!   them along the body perimeter. C-clamp force applied
//!   perpendicular to the seam plane (±Y) translates to compressive
//!   force on the gasket strip via the cup body.
//!
//! # SDF composition (§F-3 option (a), updated per §F-1)
//!
//! ```text
//! flange_sdf(P) = max(
//!     silhouette.signed_distance_to(P.x, P.z) - flange_width_m,
//!     flange_inner_offset_m - silhouette.signed_distance_to(P.x, P.z),
//!     |P.y - seam_plane_y| - flange_thickness_m
//! )
//! ```
//!
//! The lateral term uses [`crate::silhouette_2d::Silhouette2d`]'s
//! 2D point-to-silhouette distance rather than the body's 3D SDF
//! projected to the seam plane. The 3D-projected form (pre-fix) had
//! a workshop-visible bug: at concave perimeter regions the 3D
//! distance to the nearest body surface can collapse to a 3D
//! wrap-around feature (overhang, lid, sock thickness in Y direction)
//! at a small Y offset rather than the in-plane distance to the
//! silhouette curve, falsely tripping the inner check and suppressing
//! flange material. See `silhouette_2d` module docs and
//! `tests::s0_flange_continuity_probe` for the diagnostic data.
//!
//! The flange exists where ALL THREE inequalities are ≤ 0:
//! - `body_dist ≤ flange_width_m` (within outward reach of body perimeter)
//! - `body_dist ≥ flange_inner_offset_m` (clears gasket channel laterally)
//! - `|P.y - seam_plane_y| ≤ flange_thickness_m` (±thickness around seam plane)
//!
//! After the per-half halfspace cut in [`crate::compose_piece_solid`],
//! each side gets `flange_thickness_m` of flange material in its own
//! Y half-space (total closed flange-zone thickness ≈ 2 ×
//! `flange_thickness_m` + gasket compression).
//!
//! Mirrors the `GasketChannelSdf` pattern from
//! [[project-cf-cast-seam-gasket-mold-s1]] — projection-to-seam-plane
//! + `max()` composition.
//!
//! # Coordinate frame (S1 simplification)
//!
//! Seam plane is hardcoded to Y = `ribbon.seam_plane_reference().y`,
//! same as the gasket-mold S1. Production `cast.toml`'s
//! `split_normal = [1, 0, 0]` makes binormal ≈ `-Y` → seam plane ≈
//! XZ-plane → assumption holds. Other `split_normals` would need
//! seam-normal-aware flange orientation; deferred per recon §F-1.
//!
//! # Scope (S1)
//!
//! S1 ships the [`FlangeSpec`] + [`FlangeKind`] + SDF builder.
//! Public-API integration into [`crate::compose_piece_solid`] happens in
//! the same commit (the flange is unioned into the cup-piece SDF
//! when `ribbon.flange` is [`FlangeKind::Plate`]).
//!
//! Out of scope per §F-13:
//! - cf-cast-cli `[flange]` config block — S2 territory.
//! - procedure.rs workshop clamp protocol — S3 territory.
//! - Cross-field validation `inner_offset_m > half_gasket_channel_width`
//!   — S2 at the cf-cast-cli derive layer (cross-spec gate lands
//!   there, not in cf-cast).

use cf_design::{Aabb, Sdf, Solid};
use nalgebra::{Point3, Vector3};

use crate::ribbon::{PieceSide, Ribbon};
use crate::silhouette_2d::Silhouette2d;

/// Default flange lateral width (20 mm).
///
/// Picked at recon §F-2: ~10 mm for a standard C-clamp jaw reach plus
/// ~5 mm perimeter clearance (= 15 mm pre-§B). §B-S1 bumped to 16 mm
/// to give the M5 through-bolt clamp pattern symmetric 4.25 mm walls.
/// Workshop cf-view smoke 2026-05-27 surfaced a separate constraint:
/// the bolt-HEAD/WASHER footprint must clear the cup-wall outer step.
/// Bolt offset bumped 9 → 13 mm so the M5 washer (10 mm OD) clears
/// the 5 mm-thick cup-wall with 3 mm safety margin. To maintain the
/// 0.75× outboard wall rule with the new 13 mm offset, flange width
/// bumped to 20 mm (`13 + 2.75 + 4.25 = 20`). Workshop trades 4 mm
/// extra flange perimeter per piece for genuinely safe washer
/// seating. Dowels (radius 1.6 mm, offset 10 mm after the 8 → 10 mm
/// vice-jaw-clearance bump) have outboard margin 8.4 mm at the
/// 20 mm flange (`20 - 10 - 1.6 = 8.4`), well above the FDM floor.
const DEFAULT_FLANGE_WIDTH_M: f64 = 0.020;

/// Default flange thickness PER HALF (4 mm).
///
/// Total closed flange-zone thickness is roughly `2 ×
/// DEFAULT_FLANGE_THICKNESS_M` plus the gasket's compressed
/// thickness — about 8 mm of PLA plus ~0.2 mm of gasket at iter-1
/// design pressure. Above the FDM minimum-wall floor (1.0 mm).
/// Per recon §F-2 bending check: a 1 mm-wide flange strip
/// cantilevered along its 20 mm width direction, 4 mm thick, PLA
/// flexural modulus ≈ 3.5 `GPa`, distributed load ~1 `N/mm` of
/// perimeter yields δ ≈ 30 µm, well below the gasket's ~200 µm
/// compression budget.
const DEFAULT_FLANGE_THICKNESS_M: f64 = 0.004;

/// Default inner-edge gap between body perimeter and flange (2 mm).
///
/// = ~2.7 × the gasket-channel half-width at iter1 (0.75 mm). Keeps
/// the flange material LATERALLY DISJOINT from the gasket channel
/// at the seam plane so the silicone seal isn't pinched by PLA.
/// MC quantization at the §G-13 0.5 mm gasket cell size also can't
/// accidentally touch the channel at this comfortable margin.
const DEFAULT_FLANGE_INNER_OFFSET_M: f64 = 0.002;

/// Flange geometry parameter envelope per recon §F-2.
///
/// S6 workshop physical iter-3 pins the final empirical defaults;
/// iter-1 starts at the typed-range picks below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlangeSpec {
    /// Flange lateral extent perpendicular to the body perimeter,
    /// measured in the seam plane. Default `DEFAULT_FLANGE_WIDTH_M`
    /// (20 mm in iter-1).
    pub flange_width_m: f64,
    /// Flange thickness PER HALF perpendicular to the seam plane
    /// (along the cup-world Y axis in production frame). Default
    /// `DEFAULT_FLANGE_THICKNESS_M` (4 mm per cup half → ~8 mm
    /// total closed flange-zone thickness).
    pub flange_thickness_m: f64,
    /// Gap between body perimeter and flange's INNER edge — keeps
    /// the flange laterally disjoint from the gasket channel. Default
    /// `DEFAULT_FLANGE_INNER_OFFSET_M` (2 mm).
    pub flange_inner_offset_m: f64,
}

impl FlangeSpec {
    /// Workshop iter-1 starting defaults pinned at recon §F-2.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            flange_width_m: DEFAULT_FLANGE_WIDTH_M,
            flange_thickness_m: DEFAULT_FLANGE_THICKNESS_M,
            flange_inner_offset_m: DEFAULT_FLANGE_INNER_OFFSET_M,
        }
    }
}

impl Default for FlangeSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Seam-flange kind on the [`Ribbon`].
///
/// Matches the existing `RegistrationKind` / `PourGateKind` /
/// `PlugPinKind` / `GasketKind` patterns — bridges the cf-cast-cli
/// `[flange]` toml block (S2 territory) to the per-cup-half flange
/// emission in [`crate::compose_piece_solid`].
///
/// Default [`Self::None`] (no flange emission — cup pieces have
/// contour-following outer surface only). Set
/// `FlangeKind::Plate(FlangeSpec::iter1())` via
/// [`Ribbon::with_flange`] to enable the clamp-grip flange at the
/// seam plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlangeKind {
    /// No flange emission. Cup-piece outer surface stays
    /// contour-following per the archived mold-wall recon's Option
    /// A. Workshop relies on hand-clamping the curved cup outer
    /// (less even pressure on the gasket).
    None,
    /// Per-cup-half flange emission, parameterized by the inner
    /// [`FlangeSpec`]. The flange is unioned into the cup-piece SDF
    /// at the seam plane with per-half halfspace cut applied.
    Plate(FlangeSpec),
}

impl FlangeKind {
    /// Returns the inner [`FlangeSpec`] for [`FlangeKind::Plate`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&FlangeSpec> {
        match self {
            Self::None => None,
            Self::Plate(spec) => Some(spec),
        }
    }
}

/// SDF adapter for the seam-flange (§F-3 option (a) per-perimeter-
/// offset, §F-1 silhouette-based lateral distance, §M-S1.5 binormal-
/// aligned vertical term). Evaluates the body's seam-plane 2D
/// silhouette signed distance at the query point's (X, Z), then
/// composes lateral (width + inner-offset) + vertical (thickness)
/// box constraints to define the flange as a flat slab at the seam
/// plane extending outward from the body cavity perimeter.
///
/// Pattern mirrors [`crate::gasket_mold::compose_gasket_mold_solid`]'s
/// `GasketChannelSdf` — `max()` composition; the lateral distance
/// metric is the only difference.
///
/// §M-S1.5 (2026-05-27): vertical term uses
/// `dot(p - seam_midpoint, binormal)` so the flange slab is
/// perpendicular to the ribbon's binormal — coplanar with the
/// halfspace cut in [`crate::compose_piece_solid`]. Pre-§M-S1.5 the
/// vertical term used `(p.y - seam_plane_y).abs()` which assumed the
/// seam plane was Y=constant. For non-Y-aligned centerlines (e.g.,
/// the iter-1 sock scan whose centerline tilts ~3.4° from cast Z),
/// the cast-Y-aligned flange slab and the binormal-aligned halfspace
/// cut were at different orientations → their intersection was a
/// wedge that tapered the flange thickness from one Z extreme to the
/// other. Per [[project-cf-cast-unified-mating-plane-recon]] §M-S1.5
/// + workshop cf-view smoke 2026-05-27.
#[derive(Debug, Clone)]
struct FlangeSdf {
    silhouette: Silhouette2d,
    /// Seam-plane midpoint (cf-cast world frame). Used as the origin
    /// for the binormal-aligned vertical term.
    seam_midpoint: Point3<f64>,
    /// Ribbon binormal at the seam-plane reference sample. Unit
    /// vector pointing from the Positive side toward the Negative
    /// side. Defines the flange slab's thickness direction.
    binormal: Vector3<f64>,
    flange_width_m: f64,
    flange_thickness_m: f64,
    flange_inner_offset_m: f64,
}

impl Sdf for FlangeSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let body_dist = self.silhouette.signed_distance_to(p.x, p.z);
        let outer = body_dist - self.flange_width_m;
        let inner = self.flange_inner_offset_m - body_dist;
        // §M-S1.5: signed distance from p to the seam plane measured
        // along the binormal direction. The flange slab is the band
        // |this distance| < flange_thickness_m, perpendicular to
        // binormal (coplanar with the halfspace cut).
        let signed_dist_from_seam = (p - self.seam_midpoint).dot(&self.binormal);
        let vertical = signed_dist_from_seam.abs() - self.flange_thickness_m;
        outer.max(inner).max(vertical)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // The max() composition's analytical gradient is multivalued
        // at facet boundaries. Same rationale as `GasketChannelSdf`:
        // `Solid::from_sdf` bridges this via `FieldNode::UserFn`
        // which always uses finite differences, so this analytical
        // grad is unused. Picks binormal arbitrarily.
        self.binormal
    }
}

/// Build the symmetric (pre-halfspace-cut) flange [`Solid`] for the
/// given layer body + ribbon's seam plane.
///
/// The returned solid spans `|P.y - seam_plane_y| ≤ flange_thickness_m`
/// (both Y half-spaces). The caller must intersect with the per-side
/// [`Ribbon::halfspace_solid`] to keep only the side's half — see
/// [`crate::compose_piece_solid`]'s flange-union block.
///
/// `bounds` constrains the flange's SDF evaluation region; pass the
/// cup-piece's `bounding_region.bounds()` extended by the flange's
/// outward reach so MC sees the full flange extent.
pub(crate) fn build_flange_solid(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &FlangeSpec,
    bounds: Aabb,
) -> Solid {
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    // §M-S1.5: binormal == seam_normal == perpendicular to seam plane.
    // Both the flange's vertical slab AND the cup-wall's halfspace
    // cut must use this direction so they're coplanar (no tilted-cut
    // wedge artifact for non-Y-aligned centerlines).
    let binormal = seam_normal.into_inner();
    // Silhouette sampling Y is the seam midpoint's Y coordinate.
    // For nearly-Y-aligned binormals (production iter-1 ~3.4° tilt),
    // sampling on a constant-Y plane is approximately the body's
    // seam-plane cross-section to within sub-mm. Fully-correct would
    // sample on the tilted seam plane itself, but the SDF lateral
    // metric stays based on the 2D (X, Z) projection regardless, so
    // the constant-Y approximation is workshop-imperceptible.
    let seam_plane_y = seam_midpoint.y;

    // Expand the bounds outward by flange_width + a small margin so
    // MC samples the flange's outer edge correctly. The flange's
    // outward reach is `flange_width_m` from the body perimeter;
    // bounds passed in already include the body, so we just need to
    // expand them by `flange_width_m` on each side.
    let pad = spec.flange_width_m;
    let expanded_bounds = Aabb::new(
        Point3::new(bounds.min.x - pad, bounds.min.y - pad, bounds.min.z - pad),
        Point3::new(bounds.max.x + pad, bounds.max.y + pad, bounds.max.z + pad),
    );

    // Pre-compute the 2D silhouette at the seam plane over the
    // expanded XZ bounds. This is the §F-1 fix — the lateral distance
    // metric is the silhouette's 2D point-to-segment distance, NOT
    // the body's 3D SDF projected to the seam plane.
    let silhouette = Silhouette2d::from_body_at_y(
        layer_body,
        seam_plane_y,
        expanded_bounds.min.x,
        expanded_bounds.max.x,
        expanded_bounds.min.z,
        expanded_bounds.max.z,
    );

    Solid::from_sdf(
        FlangeSdf {
            silhouette,
            seam_midpoint,
            binormal,
            flange_width_m: spec.flange_width_m,
            flange_thickness_m: spec.flange_thickness_m,
            flange_inner_offset_m: spec.flange_inner_offset_m,
        },
        expanded_bounds,
    )
}

/// Build the per-side flange [`Solid`] (with halfspace cut already
/// applied) for unioning into a cup-piece's SDF.
///
/// Wraps [`build_flange_solid`] + intersects with
/// [`Ribbon::halfspace_solid`] for the given side, using zero
/// overlap-bias (the flange's seam edge meets the cup-wall's seam
/// edge exactly at the seam plane; both depend on the same MC
/// vertex-placement determinism from the halfspace SDF intersect
/// per recon-4 (P) §F-4).
pub(crate) fn build_flange_solid_for_side(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &FlangeSpec,
    bounds: Aabb,
    side: PieceSide,
) -> Solid {
    let flange = build_flange_solid(layer_body, ribbon, spec, bounds);
    flange.intersect(ribbon.halfspace_solid(side, bounds, 0.0))
}

#[cfg(test)]
mod tests {
    // Workspace lint policy allows unwrap/panic in tests for clean
    // assertion-failure prose; matches `gasket_mold.rs` + `piece.rs`.
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::ribbon::SplitNormal;

    /// Synthetic fixture mirroring the `gasket_mold` S1 fixture +
    /// production frame: straight-along-X centerline at Y=0;
    /// cylindrical body (sock proxy along X-axis, 10 mm radius in YZ,
    /// 60 mm length along X — cylinder caps at |X| = 30 mm); bounding
    /// cuboid 200 × 30 × 30 mm. Body XZ cross-section at seam plane
    /// Y=0 is the rectangle |X| ≤ 30 AND |Z| ≤ 10.
    fn synthetic_fixture() -> (Ribbon, Solid, Aabb) {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let layer_body =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.015, 0.015));
        let bounds = bounding_region.bounds().unwrap();
        (ribbon, layer_body, bounds)
    }

    /// Half-width of the iter-1 gasket channel — used as a cross-
    /// field invariant guard in the flange test below. Lives at
    /// test-module scope to satisfy
    /// `clippy::items_after_statements`.
    const GASKET_HALF_CHANNEL_WIDTH_M: f64 = 0.00075;

    #[test]
    fn flange_spec_iter1_defaults_within_recon_envelope() {
        // §F-2 typed-range envelope: width 10-20 mm, thickness
        // 3-5 mm per half, inner_offset 1-5 mm.
        let s = FlangeSpec::iter1();
        assert!(
            (0.010..=0.020).contains(&s.flange_width_m),
            "iter1 flange_width_m {} not in [10, 20] mm",
            s.flange_width_m
        );
        assert!(
            (0.003..=0.005).contains(&s.flange_thickness_m),
            "iter1 flange_thickness_m {} not in [3, 5] mm per half",
            s.flange_thickness_m
        );
        assert!(
            (0.001..=0.005).contains(&s.flange_inner_offset_m),
            "iter1 flange_inner_offset_m {} not in [1, 5] mm",
            s.flange_inner_offset_m
        );
        // Cross-field invariant: inner_offset MUST exceed half the
        // iter1 gasket channel width (0.75 mm) to keep the flange
        // material laterally disjoint from the gasket channel per
        // §F-4 "Critical invariant". cf-cast-cli S2 will gate this
        // cross-spec at TOML parse time; this test pins the default
        // iter1 spec satisfies it.
        assert!(
            s.flange_inner_offset_m > GASKET_HALF_CHANNEL_WIDTH_M,
            "iter1 flange_inner_offset_m {} must exceed half gasket \
             channel width {GASKET_HALF_CHANNEL_WIDTH_M}",
            s.flange_inner_offset_m
        );
    }

    #[test]
    fn flange_spec_default_matches_iter1() {
        let d = FlangeSpec::default();
        let i = FlangeSpec::iter1();
        assert_eq!(d.flange_width_m, i.flange_width_m);
        assert_eq!(d.flange_thickness_m, i.flange_thickness_m);
        assert_eq!(d.flange_inner_offset_m, i.flange_inner_offset_m);
    }

    #[test]
    fn flange_kind_spec_accessor() {
        assert!(FlangeKind::None.spec().is_none());
        let s = FlangeSpec::iter1();
        let kind = FlangeKind::Plate(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn flange_sdf_inside_at_body_perimeter_plus_offset() {
        // Probe at (body_perimeter + 0.5 × (width + inner_offset),
        // seam plane Y, on a body-perimeter ray) — squarely INSIDE
        // the flange material. flange_sdf should be < 0.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = FlangeSpec::iter1();
        let flange = build_flange_solid(&body, &ribbon, &spec, bounds);

        // At long-side perimeter (X=0, Z=10) the body_dist = 0; step
        // outward in +Z by midpoint of inner_offset + width:
        let step = 0.5 * (spec.flange_inner_offset_m + spec.flange_width_m);
        let probe = Point3::new(0.0, 0.0, 0.010 + step);
        let sdf = flange.evaluate(&probe);
        assert!(
            sdf < 0.0,
            "flange SDF at body-perimeter + {step} m on seam plane \
             must be < 0 (inside flange material); got {sdf}"
        );
    }

    #[test]
    fn flange_excludes_gasket_channel_region() {
        // Critical invariant per §F-4: at body perimeter (body_dist
        // = 0), the flange must NOT exist (so the gasket strip can
        // sit there without PLA pinching it). flange_sdf > 0 at
        // body_dist = 0.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = FlangeSpec::iter1();
        let flange = build_flange_solid(&body, &ribbon, &spec, bounds);

        // Probe at body perimeter (X=0, Z=10) on seam plane:
        let on_perimeter = Point3::new(0.0, 0.0, 0.010);
        let sdf = flange.evaluate(&on_perimeter);
        assert!(
            sdf > 0.0,
            "flange SDF at body perimeter (body_dist = 0) must be > 0 \
             (gasket region — no flange material); got {sdf}"
        );

        // Probe slightly outside body perimeter, still inside gasket
        // half-channel-width (~0.75 mm) — also outside flange region:
        let in_gasket_zone = Point3::new(0.0, 0.0, 0.010 + 0.0005);
        let sdf_gasket = flange.evaluate(&in_gasket_zone);
        assert!(
            sdf_gasket > 0.0,
            "flange SDF in gasket-channel half-width region must be \
             > 0 (laterally disjoint per §F-4 invariant); got {sdf_gasket}"
        );
    }

    #[test]
    fn flange_excludes_beyond_outer_edge() {
        // At body_dist > flange_width_m: outside flange material.
        // flange_sdf > 0.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = FlangeSpec::iter1();
        let flange = build_flange_solid(&body, &ribbon, &spec, bounds);

        // Probe 1mm past the flange's outer reach (X=0, Z=10 + width + 1mm):
        let beyond = Point3::new(0.0, 0.0, 0.010 + spec.flange_width_m + 0.001);
        let sdf = flange.evaluate(&beyond);
        assert!(
            sdf > 0.0,
            "flange SDF beyond outer edge (body_dist > width) must \
             be > 0 (no flange material); got {sdf}"
        );
    }

    #[test]
    fn flange_thickness_extent_on_each_side_of_seam_plane() {
        // The symmetric flange spans Y ∈ [-thickness, +thickness]
        // around seam plane. Probe just inside and just outside the
        // Y extent on a known-inside-flange XZ point.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = FlangeSpec::iter1();
        let flange = build_flange_solid(&body, &ribbon, &spec, bounds);

        // XZ point inside flange laterally (body_dist halfway between
        // inner_offset and width):
        let step = 0.5 * (spec.flange_inner_offset_m + spec.flange_width_m);
        let xz_in = (0.0, 0.010 + step);

        // Just inside +Y thickness boundary:
        let inside_plus_y = Point3::new(xz_in.0, spec.flange_thickness_m - 1e-7, xz_in.1);
        assert!(
            flange.evaluate(&inside_plus_y) < 0.0,
            "just inside +thickness Y boundary must be inside flange (SDF < 0)"
        );

        // Just outside +Y thickness boundary:
        let outside_plus_y = Point3::new(xz_in.0, spec.flange_thickness_m + 1e-7, xz_in.1);
        assert!(
            flange.evaluate(&outside_plus_y) > 0.0,
            "just outside +thickness Y boundary must be outside flange (SDF > 0)"
        );

        // Symmetry: same on -Y side.
        let inside_minus_y = Point3::new(xz_in.0, -spec.flange_thickness_m + 1e-7, xz_in.1);
        assert!(
            flange.evaluate(&inside_minus_y) < 0.0,
            "just inside -thickness Y boundary must be inside flange (SDF < 0)"
        );
        let outside_minus_y = Point3::new(xz_in.0, -spec.flange_thickness_m - 1e-7, xz_in.1);
        assert!(
            flange.evaluate(&outside_minus_y) > 0.0,
            "just outside -thickness Y boundary must be outside flange (SDF > 0)"
        );
    }

    #[test]
    fn flange_per_side_halfspace_cut_keeps_only_one_y_half() {
        // After the halfspace cut, each side gets exactly one Y
        // half. The Negative/Positive ↔ ±Y mapping follows the
        // binormal sign: for fixture split_normal=(0,0,1) the
        // tangent is +X and binormal = tangent × split = -Y. Per
        // `PieceSide::sign`, ribbon.sdf < 0 ↔ +Y region (opposite-
        // binormal); Negative side has multiplier +1 so Negative
        // halfspace contains +Y region. Positive contains -Y.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = FlangeSpec::iter1();
        let neg_flange =
            build_flange_solid_for_side(&body, &ribbon, &spec, bounds, PieceSide::Negative);
        let pos_flange =
            build_flange_solid_for_side(&body, &ribbon, &spec, bounds, PieceSide::Positive);

        // XZ point inside flange laterally:
        let step = 0.5 * (spec.flange_inner_offset_m + spec.flange_width_m);
        let probe_plus_y = Point3::new(0.0, 0.5 * spec.flange_thickness_m, 0.010 + step);
        let probe_minus_y = Point3::new(0.0, -0.5 * spec.flange_thickness_m, 0.010 + step);

        // Negative side: +Y probe inside, -Y probe outside.
        let neg_plus = neg_flange.evaluate(&probe_plus_y);
        let neg_minus = neg_flange.evaluate(&probe_minus_y);
        assert!(
            neg_plus < 0.0,
            "Negative flange must contain +Y probe (binormal=-Y \
             fixture); got {neg_plus}"
        );
        assert!(
            neg_minus > 0.0,
            "Negative flange must exclude -Y probe; got {neg_minus}"
        );

        // Positive side: mirror.
        let pos_minus = pos_flange.evaluate(&probe_minus_y);
        let pos_plus = pos_flange.evaluate(&probe_plus_y);
        assert!(
            pos_minus < 0.0,
            "Positive flange must contain -Y probe; got {pos_minus}"
        );
        assert!(
            pos_plus > 0.0,
            "Positive flange must exclude +Y probe; got {pos_plus}"
        );
    }

    /// §M-S1.5 falsification: with a tilted centerline (binormal not
    /// aligned with cast Y), the flange's vertical thickness must
    /// stay uniform across the body's Z extent. Pre-§M-S1.5 the
    /// `FlangeSdf` used `(p.y - seam_plane_y).abs()` which is
    /// perpendicular to cast Y; the cup-wall halfspace used the
    /// ribbon's binormal-perpendicular plane. For tilted centerlines
    /// the two planes were at the tilt angle to each other → the
    /// intersection (the flange material that survived the cut)
    /// tapered from one Z extreme to the other.
    ///
    /// Fixture: ribbon with centerline tilted ~5° from cast Z axis
    /// (centerline start (0, -5, -50), end (0, +5, +50)). Tangent
    /// has +Y component → binormal tilts ~5° from cast Y. Cylinder
    /// body along the tilted centerline.
    ///
    /// Probe: at multiple Z positions in the flange band (mid-band
    /// laterally), evaluate the per-side flange SDF. The set of
    /// `p` such that `flange_sdf(p)` < 0 (interior) has a uniform
    /// "thickness" in the binormal direction at every Z value —
    /// specifically, the flange's max binormal-distance from the
    /// seam plane is `flange_thickness_m` (4 mm) at every Z probed.
    ///
    /// Pre-§M-S1.5: the flange tapers; thickness varies with Z by
    /// up to ~`tan(tilt) × Z_extent` mm (~8-9 mm for 5° tilt over
    /// 100 mm).
    /// Post-§M-S1.5: thickness uniform ±FP-precision.
    #[test]
    fn flange_thickness_uniform_across_x_under_tilted_centerline() {
        // Centerline tilted in the X-Y plane: tangent direction
        // (0.995, 0.0995, 0) before normalization (≈ +X with small
        // +Y component, ~5.7° tilt). split_normal = +Z. Then
        // binormal = tangent × split ≈ (0.0995, -0.995, 0) — mostly
        // -Y but tilted ~5.7° toward +X. The binormal-aligned plane
        // is no longer parallel to the cast XZ plane, exposing the
        // pre-§M-S1.5 cast-Y-aligned vertical term as a wedge bug.
        let centerline = vec![
            Point3::new(-0.050, -0.005, 0.0),
            Point3::new(0.050, 0.005, 0.0),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        // Body: cylinder along X with radius 10 mm in YZ. Silhouette
        // at Y=0 is the rectangle X ∈ [-60, 60], Z ∈ [-10, 10] — a
        // long stadium along X.
        let layer_body =
            Solid::cylinder(0.010, 0.060).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let spec = FlangeSpec::iter1();
        let neg_flange =
            build_flange_solid_for_side(&layer_body, &ribbon, &spec, bounds, PieceSide::Negative);

        let (_seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();

        // Probe at 5 X positions across the body's length. At each
        // X, pick a probe in the lateral flange band (Z=+18 mm =
        // silhouette_dist 8 mm from the +Z silhouette edge at Z=+10).
        // Walk in BOTH +binormal and -binormal directions from this
        // probe; record the binormal-distance range where flange SDF
        // is negative (inside flange material). That range's width is
        // the flange's thickness at this X.
        let probe_xs = [-0.050, -0.025, 0.0, 0.025, 0.050];
        let mut measured_thicknesses = Vec::new();
        for x in probe_xs {
            // Base probe: walk from binormal-dist = -8 mm to +8 mm
            // (covers both halfspaces around the seam plane).
            let base_xz = Point3::new(x, 0.0, 0.018);
            let mut min_dist_inside: f64 = f64::MAX;
            let mut max_dist_inside: f64 = f64::MIN;
            for i in -100i32..=100 {
                let dist = f64::from(i) * 0.0001; // -10 mm to +10 mm in 0.1 mm steps
                let probe = base_xz + binormal * dist;
                let sdf = neg_flange.evaluate(&probe);
                if sdf < 0.0 {
                    min_dist_inside = min_dist_inside.min(dist);
                    max_dist_inside = max_dist_inside.max(dist);
                }
            }
            let thickness = if min_dist_inside < max_dist_inside {
                max_dist_inside - min_dist_inside
            } else {
                0.0
            };
            measured_thicknesses.push((x, thickness, min_dist_inside, max_dist_inside));
        }

        let mut min_t = f64::MAX;
        let mut max_t = f64::MIN;
        for (x, t, lo, hi) in &measured_thicknesses {
            eprintln!(
                "  X={:+.3} m: flange binormal-dist range [{:+.4}, {:+.4}] mm; \
                 thickness {:.3} mm",
                x,
                lo * 1000.0,
                hi * 1000.0,
                t * 1000.0
            );
            if *t > 0.0 {
                min_t = min_t.min(*t);
                max_t = max_t.max(*t);
            }
        }
        assert!(
            min_t < max_t || (min_t - max_t).abs() < 1e-9,
            "at least one probe must have found flange material"
        );
        let spread_mm = (max_t - min_t) * 1000.0;
        // Post-§M-S1.5: thicknesses uniform within 0.5 mm (probe step
        // granularity = 0.1 mm; allow some slack for binormal-aligned
        // walk vs lateral flange band intersections). Pre-§M-S1.5 the
        // spread was several mm because the cast-Y-aligned slab × the
        // binormal-tilted halfspace cut produced a wedge.
        assert!(
            spread_mm < 0.5,
            "flange thickness must be UNIFORM across X extent for tilted \
             centerline (binormal-aligned slab matches halfspace cut); got \
             spread {spread_mm:.3} mm across X probes"
        );
        // Post-§M-S1.5: thickness ≈ flange_thickness_m = 4 mm (Negative
        // side keeps half the slab, so thickness = 4 mm).
        let expected_mm = spec.flange_thickness_m * 1000.0;
        assert!(
            max_t.mul_add(1000.0, -expected_mm).abs() < 0.3,
            "Negative-side flange thickness should equal flange_thickness_m \
             = {expected_mm:.1} mm; got {:.3} mm",
            max_t * 1000.0
        );
    }

    /// §F-S1 falsification fixture per
    /// [[feedback-load-bearing-test-fixtures]] + [[project-cf-cast-flange-perimeter-continuity-bookmark]].
    ///
    /// Builds a C-shape body (cuboid with a thin-Y notch on +Z face)
    /// where the silhouette at Y=0 has a concavity AND the body wraps
    /// in 3D via thin "lids" at Y=±1.5 mm (within `flange_inner_offset`
    /// = 2 mm of the seam plane). The §F-S0 probe
    /// ([`s0_flange_continuity_probe`]) characterized the bug mechanism
    /// as the INNER check (`flange_inner_offset - body_dist > 0`)
    /// tripping when 3D `body.evaluate(projected)` returns the distance
    /// to a 3D wrap-around feature (here: the lids) rather than the
    /// in-plane distance to the silhouette curve.
    ///
    /// The original synthetic-cylinder fixture
    /// ([`flange_sdf_inside_at_body_perimeter_plus_offset`]) missed
    /// this because a cylinder cross-section is fully convex and the
    /// body is 2D-extruded (no 3D wrap), so 3D `body_dist` matches the
    /// 2D silhouette distance everywhere.
    ///
    /// Expected behavior:
    /// - At a probe IN the C's mouth, inside the flange band per 2D
    ///   silhouette distance: flange must be PRESENT.
    /// - Pre-fix (3D-projected `body.evaluate`): asserts FAIL —
    ///   `body_dist ≈ 1.5 mm` (lid surface) trips the inner check.
    /// - Post-fix (2D `silhouette.signed_distance_to`): asserts PASS —
    ///   `silhouette_dist ≈ 13 mm` (notch floor) satisfies all three
    ///   checks; flange material emitted.
    ///
    /// Paired with a cylinder-control assertion at the same 2D probe
    /// distance to disambiguate "fixture is broken" from "feature
    /// regressed" per the feedback memo.
    #[test]
    fn flange_continues_around_silhouette_concavity() {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        // C-shape body: outer cuboid 80×30×40 mm with thin-Y notch on
        // +Z face. Notch X∈[-20,20], Y∈[-1.5,1.5], Z∈[-3,23] mm
        // (intersected with body Z∈[-20,20] gives Z∈[-3,20]). Lids at
        // Y∈[-15,-1.5]∪[1.5,15] mm. Lids sit inside flange thickness
        // (Y∈[-4,4]) so they shadow the flange in their Y range, but
        // the flange should still EXIST in the lid's Y gap (Y∈[-1.5,1.5])
        // — that's the workshop-visible PR-blocker.
        let outer = Solid::cuboid(Vector3::new(0.040, 0.015, 0.020));
        let notch = Solid::cuboid(Vector3::new(0.020, 0.0015, 0.013))
            .translate(Vector3::new(0.0, 0.0, 0.010));
        let c_body = outer.subtract(notch);
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let spec = FlangeSpec::iter1();
        let c_flange = build_flange_solid(&c_body, &ribbon, &spec, bounds);

        // Probe P_concave: inside the C's mouth, mid-flange-band per
        // 2D silhouette distance.
        // - 2D silhouette dist from (0,0,+10) to notch floor (0,0,-3)
        //   = 13 mm; inside flange band [inner_offset=2, width=20].
        // - 3D body_dist (pre-fix) ≈ 1.5 mm (Y-distance to lid surface
        //   at Y=±1.5); trips inner check, flange ABSENT (BUG).
        // - 2D silhouette dist (post-fix) = 13 mm; all checks pass,
        //   flange PRESENT.
        let p_concave = Point3::new(0.0, 0.0, 0.010);
        let sdf_concave = c_flange.evaluate(&p_concave);
        assert!(
            sdf_concave < 0.0,
            "FALSIFICATION: flange must be PRESENT inside C's mouth \
             concavity at (X=0, Y=0, Z=+10 mm) — 2D silhouette dist to \
             notch floor (0,0,-3) is 13 mm which is inside the flange \
             band [2, 20] mm. Got SDF = {sdf_mm:.3} mm (positive = \
             flange absent). Pre-fix 3D `body.evaluate` returns ≈1.5 mm \
             (lid surface at Y=±1.5) tripping the inner check; post-fix \
             2D silhouette returns 13 mm and emits flange material. \
             See §F-S0 probe + \
             [[project-cf-cast-flange-perimeter-continuity-bookmark]].",
            sdf_mm = sdf_concave * 1000.0,
        );

        // Probe P_wall_outboard: 5 mm outboard of the notch's -X wall,
        // inside the C's mouth. 2D silhouette dist = 5 mm (to notch
        // wall at -20). Pre-fix body_dist = 1.5 mm (lid). Same bug.
        let p_wall_outboard = Point3::new(-0.015, 0.0, 0.010);
        let sdf_wall = c_flange.evaluate(&p_wall_outboard);
        assert!(
            sdf_wall < 0.0,
            "FALSIFICATION: flange must be PRESENT 5 mm outboard of \
             notch -X wall, inside C's mouth at (X=-15, Y=0, Z=+10 mm). \
             2D silhouette dist = 5 mm, inside flange band. Got SDF \
             = {:.3} mm.",
            sdf_wall * 1000.0,
        );

        // Cylinder-control: parallel fixture with same probe 2D distance
        // from the silhouette. Cylinder has no concavity + no 3D wrap,
        // so pre-fix and post-fix agree. If THIS assertion fails, the
        // fixture is broken (not the silhouette logic) per
        // [[feedback-load-bearing-test-fixtures]].
        let cyl_body =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let cyl_flange = build_flange_solid(&cyl_body, &ribbon, &spec, bounds);
        // 13 mm outboard of cylinder's +Z silhouette edge (at Z=10 mm)
        // — same lateral distance as P_concave from its nearest
        // silhouette point.
        let p_cyl = Point3::new(0.0, 0.0, 0.023);
        let sdf_cyl = cyl_flange.evaluate(&p_cyl);
        assert!(
            sdf_cyl < 0.0,
            "FIXTURE PRECONDITION FAILED: cylinder-control flange must \
             be PRESENT 13 mm outboard of +Z perimeter at (X=0, Y=0, \
             Z=+23 mm); both pre-fix and post-fix should agree here. \
             Got SDF = {:.3} mm — indicates build_flange_solid wiring \
             or FlangeSpec defaults regressed, not the silhouette \
             change.",
            sdf_cyl * 1000.0,
        );

        // Sanity: well past the C-shape's flange outer reach in -Z
        // direction (convex region) — flange must be ABSENT. Both
        // pre-fix and post-fix agree here (convex region; no bug).
        let p_beyond = Point3::new(0.0, 0.0, -0.045);
        let sdf_beyond = c_flange.evaluate(&p_beyond);
        assert!(
            sdf_beyond > 0.0,
            "flange must be ABSENT 25 mm past body's -Z silhouette \
             (flange_width = 20 mm); got SDF = {:.3} mm.",
            sdf_beyond * 1000.0,
        );
    }

    /// §F-S0 empirical probe — disambiguates which `FlangeSdf` check
    /// (outer/inner/vertical) trips the workshop-visible "flange isn't
    /// all the way around" symptom on real 3D-concave body geometry.
    ///
    /// Background: [[project-cf-cast-flange-perimeter-continuity-bookmark]]
    /// claims `body.evaluate(P_projected_to_seam)` returns a 3D distance
    /// LARGER than the 2D in-plane distance to the body's seam-plane
    /// silhouette curve in concavity regions, tripping the OUTER check
    /// (`body_dist - flange_width > 0`). Cold-read analysis of the math
    /// suggests the inequality goes the OTHER way: the silhouette is a
    /// subset of the body surface, so 3D `body.evaluate(Q)` ≤ 2D
    /// distance from Q to silhouette for any Q in the seam plane. If
    /// that analysis is correct, the actual bug mechanism is the INNER
    /// check (`flange_inner_offset - body_dist > 0`): when the body has
    /// 3D structure (overhang/shelf/lid) near the projected seam-plane
    /// point, `body_dist` is unexpectedly small, falsely tripping the
    /// "this point is inside the gasket-channel margin" check.
    ///
    /// This probe runs three synthetic fixtures and dumps the four SDF
    /// components at each probe to see which check actually trips. The
    /// data informs the §F-S1 C-shape fixture geometry — a falsification
    /// fixture has to ACTUALLY reproduce the bug pre-fix, and depending
    /// on which mechanism is real, the fixture geometry is different.
    ///
    /// `#[ignore]`-gated; run manually with:
    /// `cargo test --release -p cf-cast --lib \
    ///     flange::tests::s0_flange_continuity_probe -- --ignored --nocapture`
    #[test]
    #[ignore = "S0 diagnostic probe — manual invocation only"]
    fn s0_flange_continuity_probe() {
        let spec = FlangeSpec::iter1();
        eprintln!(
            "\n§F-S0 probe — FlangeSpec: width={} mm, thickness={} mm/half, \
             inner_offset={} mm\n",
            spec.flange_width_m * 1000.0,
            spec.flange_thickness_m * 1000.0,
            spec.flange_inner_offset_m * 1000.0,
        );

        // Fixture A (control — pure cylinder along X, convex silhouette).
        // 2D silhouette at Y=0 is the rectangle |Z|≤10 mm, |X|≤30 mm.
        // 3D body is 2D-extruded ⇒ 3D body_dist == 2D silhouette dist
        // at any seam-plane query. Expect no divergence between body
        // and silhouette signals; flange present in the lateral
        // [inner_offset, width] band.
        eprintln!("=== Fixture A (control): cylinder along X, R=10, len=60 mm ===");
        let body_a =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        probe_flange_sdf(&body_a, &spec);

        // Fixture B (overhang — cylinder + thin wing at Y=2 mm).
        // The wing is a thin slab at Y∈[1.5, 2.5] mm extending in +Z
        // beyond the cylinder's perimeter. At Y=0, the silhouette is
        // unchanged (just the cylinder rectangle) — the wing has no
        // presence at Y=0. But for query points at Y=0 beyond the
        // cylinder's +Z silhouette edge, the 3D nearest body surface
        // is the wing (distance ≈ 1.5 mm), NOT the cylinder edge
        // (distance > 1.5 mm in 3D). So 3D body_dist < 2D silhouette
        // dist. If body_dist drops below flange_inner_offset (2 mm),
        // the INNER check trips → flange absent. Predicted bug
        // manifestation.
        eprintln!("\n=== Fixture B (overhang): cylinder + thin wing at Y=2 mm ===");
        let cyl_b =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let wing_b = Solid::cuboid(Vector3::new(0.025, 0.0005, 0.015))
            .translate(Vector3::new(0.0, 0.002, 0.020));
        let body_b = cyl_b.union(wing_b);
        probe_flange_sdf(&body_b, &spec);

        // Fixture C (extruded C-shape — notch through all Y).
        // Outer cuboid 80×30×40 mm minus inner notch 40×40×26 mm
        // translated +Z=10 — creates a 2D concavity on +Z face that
        // extends through ALL Y of the body. 3D body is 2D-extruded ⇒
        // body_dist == silhouette dist. Predicted: no bug
        // manifestation (concavity alone is not sufficient).
        eprintln!("\n=== Fixture C: extruded C-shape (notch through Y) ===");
        let outer_c = Solid::cuboid(Vector3::new(0.040, 0.015, 0.020));
        let notch_c = Solid::cuboid(Vector3::new(0.020, 0.020, 0.013))
            .translate(Vector3::new(0.0, 0.0, 0.010));
        let body_c = outer_c.subtract(notch_c);
        probe_flange_sdf(&body_c, &spec);

        eprintln!(
            "\nInterpretation: if Fixture B's +Z probes at Y=0 show INNER>0 and \
             total>0 (flange ABSENT) while Fixture A's same-Z probes show flange \
             PRESENT, the INNER-check hypothesis is confirmed and the §F-S1 C-shape \
             fixture should pair an in-plane concavity with a 3D wrap-around \
             feature (lid/wing at small Y offset) that won't shadow the flange but \
             will trip body_dist below inner_offset. If neither check trips on \
             Fixture B, the bug mechanism is something else and the hypothesis \
             needs revision."
        );
    }

    /// Probe helper for `s0_flange_continuity_probe`. Builds a
    /// `FlangeSdf` directly (skipping `Solid::from_sdf` so we can
    /// inspect the components in isolation) and dumps both the
    /// pre-fix 3D body distance AND the post-fix 2D silhouette
    /// distance at each probe, plus the four post-fix SDF terms.
    /// The 3D-vs-2D column is the empirical signal that drove the
    /// §F-1 architectural pivot — kept post-fix as a regression
    /// diagnostic for future flange-continuity work.
    fn probe_flange_sdf(body: &Solid, spec: &FlangeSpec) {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        let seam_plane_y = seam_midpoint.y;

        // Build silhouette over a generous XZ region covering the
        // fixture's body + flange reach.
        let silhouette =
            Silhouette2d::from_body_at_y(body, seam_plane_y, -0.060, 0.060, -0.040, 0.040);
        let sdf = FlangeSdf {
            silhouette,
            seam_midpoint,
            binormal,
            flange_width_m: spec.flange_width_m,
            flange_thickness_m: spec.flange_thickness_m,
            flange_inner_offset_m: spec.flange_inner_offset_m,
        };

        // All probes at X=0, Y=0 (seam plane), varying Z. Cylinder
        // silhouette +Z edge is at Z=10 mm; flange band is
        // Z ∈ [12, 30] mm (inner_offset=2, width=20).
        let probes: &[(&str, Point3<f64>)] = &[
            (
                "on +Z perimeter           @ Z=10  mm",
                Point3::new(0.0, 0.0, 0.010),
            ),
            (
                "gasket-channel zone       @ Z=10.5mm",
                Point3::new(0.0, 0.0, 0.0105),
            ),
            (
                "just inside flange inner  @ Z=12.5mm",
                Point3::new(0.0, 0.0, 0.0125),
            ),
            (
                "mid-flange band           @ Z=20  mm",
                Point3::new(0.0, 0.0, 0.020),
            ),
            (
                "deeper into flange band   @ Z=25  mm",
                Point3::new(0.0, 0.0, 0.025),
            ),
            (
                "near outer flange edge    @ Z=29  mm",
                Point3::new(0.0, 0.0, 0.029),
            ),
            (
                "past outer flange edge    @ Z=31  mm",
                Point3::new(0.0, 0.0, 0.031),
            ),
        ];

        eprintln!(
            "  {:42}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}  flange?",
            "probe (all at X=0, Y=0)", "body3D", "silh2D", "inner", "vert", "total"
        );
        for (plabel, p) in probes {
            let projected = Point3::new(p.x, seam_plane_y, p.z);
            let body_dist_3d = body.evaluate(&projected);
            let body_dist_2d = sdf.silhouette.signed_distance_to(p.x, p.z);
            // The post-fix inner check uses the 2D silhouette distance.
            let inner = spec.flange_inner_offset_m - body_dist_2d;
            let vertical = (p.y - seam_plane_y).abs() - spec.flange_thickness_m;
            let total = sdf.eval(*p);
            let present = if total < 0.0 { "PRESENT" } else { "absent " };
            eprintln!(
                "  {:42}  {:>7.3}  {:>7.3}  {:>7.3}  {:>7.3}  {:>7.3}  {}",
                plabel,
                body_dist_3d * 1000.0,
                body_dist_2d * 1000.0,
                inner * 1000.0,
                vertical * 1000.0,
                total * 1000.0,
                present,
            );
        }
    }
}
