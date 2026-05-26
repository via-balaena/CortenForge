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
//!   15 mm — C-clamp jaw reach + 5 mm grip clearance). Thickness
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
//! # SDF composition (§F-3 option (a))
//!
//! ```text
//! flange_sdf(P) = max(
//!     body.evaluate(P_projected_to_seam) - flange_width_m,
//!     flange_inner_offset_m - body.evaluate(P_projected_to_seam),
//!     |P.y - seam_plane_y| - flange_thickness_m
//! )
//! ```
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
//! Public-API integration into [`compose_piece_solid`] happens in
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

/// Default flange lateral width (15 mm).
///
/// Picked at recon §F-2: ~10 mm for a standard C-clamp jaw reach
/// + ~5 mm perimeter clearance.
const DEFAULT_FLANGE_WIDTH_M: f64 = 0.015;

/// Default flange thickness PER HALF (4 mm).
///
/// Total closed flange-zone thickness is roughly `2 ×
/// DEFAULT_FLANGE_THICKNESS_M` plus the gasket's compressed
/// thickness — about 8 mm of PLA plus ~0.2 mm of gasket at iter-1
/// design pressure. Above the FDM minimum-wall floor (1.0 mm).
/// Per recon §F-2 bending check: a 1 mm-wide flange strip
/// cantilevered along its 15 mm width direction, 4 mm thick, PLA
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
    /// measured in the seam plane. Default [`DEFAULT_FLANGE_WIDTH_M`]
    /// (15 mm).
    pub flange_width_m: f64,
    /// Flange thickness PER HALF perpendicular to the seam plane
    /// (along the cup-world Y axis in production frame). Default
    /// [`DEFAULT_FLANGE_THICKNESS_M`] (4 mm per cup half → ~8 mm
    /// total closed flange-zone thickness).
    pub flange_thickness_m: f64,
    /// Gap between body perimeter and flange's INNER edge — keeps
    /// the flange laterally disjoint from the gasket channel. Default
    /// [`DEFAULT_FLANGE_INNER_OFFSET_M`] (2 mm).
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
/// offset). Evaluates the layer body's SDF at the seam-plane
/// projection of the query point, then composes lateral (width +
/// inner-offset) + vertical (thickness) box constraints to define
/// the flange as a flat slab at the seam plane extending outward
/// from the body cavity perimeter.
///
/// Pattern mirrors [`crate::gasket_mold::compose_gasket_mold_solid`]'s
/// `GasketChannelSdf` — projection-to-seam-plane + `max()`
/// composition. Y-clamp on the vertical term not needed here (the
/// vertical `max()` component handles outside-thickness-range fully
/// via the standard `|p.y - seam_y| - thickness` SDF box).
#[derive(Debug, Clone)]
struct FlangeSdf {
    body: Solid,
    seam_plane_y: f64,
    flange_width_m: f64,
    flange_thickness_m: f64,
    flange_inner_offset_m: f64,
}

impl Sdf for FlangeSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let projected = Point3::new(p.x, self.seam_plane_y, p.z);
        let body_dist = self.body.evaluate(&projected);
        let outer = body_dist - self.flange_width_m;
        let inner = self.flange_inner_offset_m - body_dist;
        let vertical = (p.y - self.seam_plane_y).abs() - self.flange_thickness_m;
        outer.max(inner).max(vertical)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // The max() composition's analytical gradient is multivalued
        // at facet boundaries. Same rationale as `GasketChannelSdf`:
        // `Solid::from_sdf` bridges this via `FieldNode::UserFn`
        // which always uses finite differences, so this analytical
        // grad is unused. Picks +Y arbitrarily.
        Vector3::new(0.0, 1.0, 0.0)
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
    let (seam_midpoint, _seam_normal) = ribbon.seam_plane_reference();
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

    Solid::from_sdf(
        FlangeSdf {
            body: layer_body.clone(),
            seam_plane_y,
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
}
