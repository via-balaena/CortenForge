//! Per-layer gasket mold composition for v2 cup-half seam sealing.
//!
//! S1 + S2 of `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md` — the gasket
//! mold is a flat tray with a closed-loop channel that workshop user
//! pours silicone into. The cured gasket is a thin closed-loop strip
//! shaped like the cup-piece body-cavity perimeter at the seam plane.
//! Workshop installs the gasket on the Negative cup-half seam face
//! before closing the Positive half and pouring the layer's silicone
//! — the compressible silicone-on-silicone interface absorbs FDM
//! print tolerance (~200 µm per §G-0 typed-range table) + STL-level
//! MC cell-quantization mismatch (~3 mm one-cell width per
//! `~/scans/cast_iter1_bare/` cross-half match probe from
//! 2026-05-25 head-architect session).
//!
//! See `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md` §G-1 — §G-6 for the
//! locked design decisions and §G-13 S1 + S2 for this module's scope.
//!
//! # Geometry overview
//!
//! - **Gasket** (the silicone strip user gets after pouring + curing
//!   into this mold): a thin closed-loop tube with trapezoidal
//!   cross-section ([`GasketSpec::cross_section_width_m`] midline-wide
//!   × [`GasketSpec::cross_section_thickness_m`] tall, with
//!   [`GasketSpec::draft_angle_deg`] wall draft) sweeping along the
//!   body-cavity perimeter on the seam plane.
//! - **Gasket mold** (the printed PLA artifact this module composes):
//!   a flat rectangular tray with a closed-loop channel carved into
//!   its top face. Channel matches the gasket cross-section, wider at
//!   the tray top (the demold-draw face) + narrower at the channel
//!   floor for clean peel-out. User pours silicone in from above,
//!   screeds level, cures, peels gasket out.
//!
//! # SDF composition
//!
//! ```text
//! mold = tray_cuboid ∖ channel_sdf
//!
//! channel_sdf(P) = max(
//!     |body.evaluate(P_projected_to_seam)| - half_width(P.y),
//!     |P.y - channel_y_center|             - channel_depth/2
//! )
//!
//! half_width(P.y) = 0.5 * cross_section_width
//!                 + tan(draft_angle) * clamp(P.y - channel_y_center,
//!                                             ±channel_depth/2)
//! ```
//!
//! `body.evaluate(P_projected_to_seam)` reads the layer body's SDF
//! at the seam-plane projection of `P` — captures the body cavity
//! perimeter shape at the seam plane regardless of `P.y` (so the
//! channel traces the perimeter even when probed away from the
//! seam plane). `|body_sdf| < half_width(P.y)` means `P` is
//! laterally within the (Y-dependent) channel envelope of the
//! perimeter curve; combined with the vertical box at
//! `channel_y_center ± depth/2`, the result is the 3D channel
//! sweep. For `draft_angle = 0`, `half_width(P.y)` collapses to
//! `cross_section_width / 2` and the channel recovers a rectangular
//! cross-section bit-precisely.
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
//! # Scope (S1 + S2 shipped)
//!
//! - **S1**: SDF composition + characterisation tests.
//! - **S2**: Trapezoidal cross-section ([`GasketSpec::draft_angle_deg`])
//!   per recon §G-3 demold; gasket-material enum ([`GasketMaterial`])
//!   with Ecoflex 00-30 + Dragon Skin 10A picks per §G-1;
//!   Hookean compression prediction ([`GasketSpec::predicted_compression_m`])
//!   anchoring the FDM-tolerance gate (§G-0 typed-range; ~200 µm)
//!   at workshop clamp pressure; short-side + corner perimeter
//!   probes carried from S1 cold-read pass-1.
//!
//! Out of scope per §G-13:
//! - cf-cast-cli integration (`[gasket]` config block, STL
//!   emission) — S3 territory.
//! - `procedure.rs` workshop protocol prose — S4 territory.
//! - Pin-routing (§G-5 (a)/(b)/(c) variants) — S3 territory.
//! - Dovetail retention (§G-3 fallback) — S5 territory.
//!
//! # Open design questions carried to S3-S6
//!
//! - **Gasket centerline position.** The SDF places gasket centerline
//!   at `body.evaluate = 0` — i.e., the gasket sits HALF-INSIDE the
//!   silicone cast volume + half-outside in the cup-wall region.
//!   Workshop consequence: silicone bonds to the gasket's inner half
//!   during pour + cure; after demold the cast carries a thin
//!   silicone bead at the seam (the gasket itself). Workshop user
//!   trims. **Alternative** for S3-S5 consideration: shift centerline
//!   to `body.evaluate = +width/2 + offset` so the gasket sits
//!   outside the silicone (gasket stays in cup-wall on demold; no
//!   cast contamination). Adds 1 term to the channel SDF formula;
//!   defer until S6 iter-3 reveals cast contamination.
//! - **Single-plane body cross-section approximation.** The channel
//!   SDF evaluates body at the seam-plane midpoint Y from
//!   [`Ribbon::seam_plane_reference`], not at the per-segment
//!   binormal plane. For low-curvature centerlines (production:
//!   `max tangent rotation: 0.1°`) the per-Z variation is small and
//!   absorbed by silicone gasket flexibility on install. S3 should
//!   validate on a curved-centerline production fixture during
//!   cf-cast-cli iter-1 regen.
//! - **Per-STL positioning.** Mold is composed at the SAME world
//!   coordinates as the cup pieces (tray Y ∈ [0, `tray_thickness_m`]
//!   overlaps the cup-piece bounding region). cf-view assembly mode
//!   will show interpenetration. S3 cf-cast-cli integration is
//!   responsible for per-STL world-coordinate offsets (e.g., shift
//!   gasket mold outside the cup piece bounding region) so cf-view
//!   smoke isn't confusing.
//! - **`channel_depth_m` vs `cross_section_thickness_m`.** Default
//!   has them equal: silicone fills the channel to exactly the
//!   gasket thickness. If `channel_depth_m < cross_section_thickness_m`,
//!   silicone overflows the channel during pour → forms a flat sheet
//!   on top of the tray → workshop user trims the strip out of the
//!   sheet. If `channel_depth_m > cross_section_thickness_m`, silicone
//!   doesn't fill the channel fully — usually undesirable. S2 keeps
//!   them equal at iter1; S6 iter-3 pour may recalibrate.
//! - **Tray orientation.** Tray hardcoded to XZ-plane with Y as
//!   vertical. Production `~/scans/cast.toml` has
//!   `split_normal = [1, 0, 0]` → binormal ≈ `-Y` → seam plane ≈
//!   XZ-plane → tray orientation correct. For other `split_normal`
//!   pickings the tray would need rotating to match the binormal
//!   direction. S3 cf-cast-cli should add a tray-orientation
//!   invariant test if a non-Y binormal becomes a production target.
//! - **MC quantization at body-perimeter corners.** S2 SDF probes
//!   verify the channel rounds the body-rectangle corners with
//!   half-width radius. MC meshing at S3 (3 mm default cell) will
//!   smooth that further; if iter-1 cf-view smoke shows lost corners
//!   on production fixtures the (4')-pattern from the cap-plane
//!   recon `docs/CF_CAST_CAP_PLANE_FLATNESS_BOOKMARK.md` applies
//!   ("accept + document" below-print-resolution MC byproducts —
//!   workshop trims if needed).

use cf_design::{Aabb, Sdf, Solid};
use nalgebra::{Point3, Vector3};

use crate::error::{CastError, CastTarget};
use crate::mesh_csg::MatingTransform;
use crate::ribbon::Ribbon;

/// Default cross-section width for the iter-1 gasket (~1.5 mm).
///
/// Picked at recon §G-1 typed-range midpoint. S6 workshop iter-3
/// pour may narrow empirically. FDM-printable: well above the Bambu
/// A1 default 0.4 mm extrusion-width minimum + leaves room for the
/// channel's two side walls (>= 0.6 mm each).
const DEFAULT_CROSS_SECTION_WIDTH_M: f64 = 0.0015;

/// Default cross-section thickness (gasket height perpendicular to
/// the seam plane; ~0.8 mm).
///
/// Picked at recon §G-1 typed-range midpoint. Below 0.5 mm and the
/// gasket may not bridge FDM-tolerance gaps (~200 µm); above 1 mm
/// and the gasket may not compress enough to let halves close. S2
/// `predicted_compression_m` cross-checks this against the FDM-
/// tolerance target (§G-0 typed-range; ~200 µm) at iter1 spec; S6
/// iter-3 empirically validates.
const DEFAULT_CROSS_SECTION_THICKNESS_M: f64 = 0.0008;

/// Default tray total thickness (3 mm). Sized so the channel
/// (depth = cross-section thickness) carves a notch into the top
/// without weakening the tray base below the FDM minimum-wall floor.
const DEFAULT_TRAY_THICKNESS_M: f64 = 0.003;

/// Default lateral margin around the bounding-region XZ projection
/// when sizing the tray (5 mm each side). Gives workshop user a
/// rim to grab + handle the tray during pour + demold.
const DEFAULT_TRAY_MARGIN_M: f64 = 0.005;

/// Default channel-wall draft angle (5°) for demold per recon §G-3
/// ("channel walls: vertical or slightly tapered for demold"). The
/// channel is WIDER at the tray top + NARROWER at the channel floor
/// so the cured gasket peels out upward through the wide opening.
/// `cross_section_width_m` is the MIDLINE width; top half-width is
/// `0.5 * width + 0.5 * depth * tan(draft)`, bottom is
/// `0.5 * width - 0.5 * depth * tan(draft)`. Setting `draft = 0`
/// recovers a rectangular cross-section bit-precisely.
const DEFAULT_DRAFT_ANGLE_DEG: f64 = 5.0;

/// Cap on the MC sampling cell size used for gasket-mold meshing.
///
/// At iter1 the channel cross-section is 1.5 mm × 0.8 mm — sub-cell
/// at the cast's default 3 mm cells, which MC would either drop
/// entirely or fragment unrecognisably. 0.5 mm cells give ~3 cells
/// across channel width + ~1.6 cells across depth (channel surface
/// resolves cleanly with the trapezoidal draft preserved). The
/// tray's bounding cuboid is small (Y ≈ 3 mm, X ~ scan-extent-x,
/// Z ~ scan-extent-z) so the cell-count cost is bounded — ~3-5 s
/// per gasket mold at production scan sizes. Same pattern as
/// `crate::spec::FUNNEL_MAX_CELL_SIZE_M` for the funnel nipple.
pub const GASKET_MAX_CELL_SIZE_M: f64 = 0.0005;

/// Default workshop-clamp pressure target for the compression
/// calculation (20 kPa). Roughly: workshop user closes two cup
/// halves over the gasket with a hand-applied C-clamp or rubber
/// band; the seam-face contact area gets ~5-30 kPa contact
/// pressure. Picked mid-range; S6 workshop iter-3 measures actual.
/// Used by [`GasketSpec::predicted_compression_m`] to predict the
/// gasket's clamped-state compression vs the FDM-tolerance floor
/// (§G-0 typed-range; ~200 µm).
const DEFAULT_WORKSHOP_CLAMP_PRESSURE_PA: f64 = 20_000.0;

/// Platinum-cure silicone material pick for the gasket pour.
///
/// Per recon §G-1 the gasket is poured into the printed mold then
/// cured + peeled out; the cured strip's compression behaviour under
/// workshop clamp pressure determines whether it absorbs FDM print
/// tolerance (~200 µm) at the cup-half seam. Two candidates ship at
/// S2 — picked workshop-empirically at S6 iter-3 pour.
///
/// Young's-modulus values are published Smooth-On TDS data
/// (100% modulus interpolated to the low-strain Hookean regime).
/// They are predictions, not S6-validated; S7 may recalibrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GasketMaterial {
    /// Ecoflex 00-30 (Shore 00-30). Very soft + tacky; same workshop
    /// silicone pool as the cast itself per recon §G-1. iter1 default
    /// per recon §G-1.
    Ecoflex0030,
    /// Dragon Skin 10A (Shore 10A). Stiffer + tear-resistant
    /// alternative. Recon §G-1 fallback if Ecoflex compresses too
    /// freely on the workshop iter-3 clamp.
    DragonSkin10A,
}

impl GasketMaterial {
    /// Young's-modulus prediction (Pa) at low-strain Hookean regime.
    /// Source: Smooth-On TDS 100% modulus, interpolated.
    #[must_use]
    pub const fn youngs_modulus_pa(self) -> f64 {
        match self {
            // ~10 psi 100% modulus per TDS → linearized E ≈ 69 kPa.
            Self::Ecoflex0030 => 69_000.0,
            // ~32 psi 100% modulus per TDS → linearized E ≈ 220 kPa.
            Self::DragonSkin10A => 220_000.0,
        }
    }

    /// Short workshop label (e.g. for procedure.rs prose + log lines).
    #[must_use]
    pub const fn shore_label(self) -> &'static str {
        match self {
            Self::Ecoflex0030 => "Ecoflex 00-30",
            Self::DragonSkin10A => "Dragon Skin 10A",
        }
    }
}

/// Gasket mold parameter envelope per recon §G-1 + §G-3.
///
/// S2 introduced the trapezoidal-draft-angle cross-section profile,
/// the gasket-material enum (Ecoflex 00-30 / Dragon Skin 10A), and
/// the workshop-clamp-pressure target fields. S6 workshop physical
/// iter-3 pins the final empirical defaults. See
/// `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GasketSpec {
    /// Gasket cross-section midline width (perpendicular to perimeter
    /// curve, in the seam plane). Default
    /// `DEFAULT_CROSS_SECTION_WIDTH_M` (1.5 mm). For non-zero
    /// [`Self::draft_angle_deg`] the channel is wider at the tray top +
    /// narrower at the channel floor; this field is the MID-Y width.
    pub cross_section_width_m: f64,
    /// Gasket cross-section thickness (perpendicular to seam plane,
    /// along the cup-world Y axis). Default
    /// `DEFAULT_CROSS_SECTION_THICKNESS_M` (0.8 mm).
    pub cross_section_thickness_m: f64,
    /// Tray total Y extent (printer-bed-vertical). Default
    /// `DEFAULT_TRAY_THICKNESS_M` (3 mm).
    pub tray_thickness_m: f64,
    /// Lateral margin around the bounding region's XZ projection when
    /// sizing the tray. Default `DEFAULT_TRAY_MARGIN_M` (5 mm).
    pub tray_margin_m: f64,
    /// Channel depth into tray top (`<= cross_section_thickness_m` to
    /// keep the gasket fully contained; `> tray_thickness_m` would
    /// pierce the tray base). Default = cross-section thickness.
    pub channel_depth_m: f64,
    /// Channel-wall draft angle for demold (degrees). Default
    /// `DEFAULT_DRAFT_ANGLE_DEG` (5°). Channel is wider at tray top
    /// (wide-face opening) + narrower at the channel floor; cured
    /// gasket peels out upward. `0.0` recovers a rectangular cross-
    /// section bit-precisely.
    pub draft_angle_deg: f64,
    /// Gasket material — informs the compression prediction in
    /// [`GasketSpec::predicted_compression_m`]. Default
    /// [`GasketMaterial::Ecoflex0030`] per recon §G-1.
    pub material: GasketMaterial,
    /// Target workshop clamp pressure (Pa) for the predicted
    /// compression at install. Default
    /// `DEFAULT_WORKSHOP_CLAMP_PRESSURE_PA` (20 kPa). S6 iter-3
    /// empirically measures actual clamp pressure.
    pub workshop_clamp_pressure_pa: f64,
}

impl GasketSpec {
    /// Workshop iter-1 starting defaults pinned at recon §G-1 typed-range
    /// midpoints + S2 trapezoidal draft + Ecoflex 00-30 default. S6
    /// physical iter-3 calibration may narrow.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            cross_section_width_m: DEFAULT_CROSS_SECTION_WIDTH_M,
            cross_section_thickness_m: DEFAULT_CROSS_SECTION_THICKNESS_M,
            tray_thickness_m: DEFAULT_TRAY_THICKNESS_M,
            tray_margin_m: DEFAULT_TRAY_MARGIN_M,
            channel_depth_m: DEFAULT_CROSS_SECTION_THICKNESS_M,
            draft_angle_deg: DEFAULT_DRAFT_ANGLE_DEG,
            material: GasketMaterial::Ecoflex0030,
            workshop_clamp_pressure_pa: DEFAULT_WORKSHOP_CLAMP_PRESSURE_PA,
        }
    }

    /// Predicted gasket compression (m) at the spec's workshop clamp
    /// pressure, using the linear Hookean approximation
    /// `compression = (P / E) * thickness`. Valid for strains < ~30%
    /// (the linear-low-strain regime); at higher strain silicone is
    /// non-linear + this prediction overestimates by ~10-20%.
    ///
    /// S6 workshop iter-3 empirically validates; if measured
    /// compression < FDM-tolerance target (§G-0 typed-range;
    /// ~200 µm) the seal fails + workshop user picks a softer
    /// material or higher clamp.
    #[must_use]
    pub fn predicted_compression_m(&self) -> f64 {
        let strain = self.workshop_clamp_pressure_pa / self.material.youngs_modulus_pa();
        strain * self.cross_section_thickness_m
    }
}

impl Default for GasketSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Gasket-mold kind on the [`crate::Ribbon`].
///
/// Matches the existing `RegistrationKind` / `PourGateKind` /
/// `PlugPinKind` patterns — bridges the cf-cast-cli `[gasket]` toml
/// block to the per-layer gasket-mold emission in `export_molds_v2`.
///
/// Default [`Self::None`] (no gasket mold emission — cup halves
/// hand-clamped at iter-1, no silicone seal). Set
/// `GasketKind::Mold(GasketSpec::iter1())` via
/// [`Ribbon::with_gasket`](crate::Ribbon::with_gasket) to enable
/// per-layer gasket mold STL emission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GasketKind {
    /// No gasket-mold emission. Workshop user hand-clamps cup halves
    /// without a silicone seal — relies on FDM print tolerance + cup
    /// curved-centerline match alone (cross-half delta ≤ 200 µm per
    /// `~/scans/cast_iter1_bare/` baseline, but ≥ FDM-tolerance
    /// envelope for soft silicones).
    None,
    /// Per-layer gasket mold emission, parameterized by the inner
    /// [`GasketSpec`] (`iter1()` for workshop iter-3 defaults).
    /// `export_molds_v2` writes `gasket_mold_layer_N.stl` for each
    /// layer with the channel traced to that layer's body-cavity
    /// perimeter at the seam plane.
    Mold(GasketSpec),
}

impl GasketKind {
    /// Returns the inner [`GasketSpec`] for [`GasketKind::Mold`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&GasketSpec> {
        match self {
            Self::None => None,
            Self::Mold(spec) => Some(spec),
        }
    }
}

/// SDF adapter for the gasket channel.
///
/// Evaluates the layer body's SDF at the seam-plane projection of
/// the query point, then composes lateral (width) + vertical (depth)
/// box constraints to define the channel as a 3D swept tube.
///
/// For non-zero `draft_tan` the lateral half-width varies linearly
/// with Y across the channel depth: top half-width is
/// `0.5 * width + 0.5 * depth * draft_tan`, bottom is
/// `0.5 * width - 0.5 * depth * draft_tan`. Setting `draft_tan = 0`
/// recovers a rectangular cross-section bit-precisely. Y-clamp on
/// the draft term prevents extrapolation outside the channel Y
/// range (the vertical term dominates there anyway).
#[derive(Debug, Clone)]
struct GasketChannelSdf {
    body: Solid,
    seam_plane_y: f64,
    channel_y_min: f64,
    channel_y_max: f64,
    width_m: f64,
    draft_tan: f64,
}

impl Sdf for GasketChannelSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let projected = Point3::new(p.x, self.seam_plane_y, p.z);
        let body_dist = self.body.evaluate(&projected);
        let y_center = 0.5 * (self.channel_y_min + self.channel_y_max);
        let y_extent = 0.5 * (self.channel_y_max - self.channel_y_min);
        let y_offset = (p.y - y_center).clamp(-y_extent, y_extent);
        let half_w = self.draft_tan.mul_add(y_offset, 0.5 * self.width_m);
        let lateral = body_dist.abs() - half_w;
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
/// empty pre-S3 (per §G-13 scope). S3 ships the §G-5 (a)/(b) pin-
/// routing variants (notch-outs at cup-pin footprints); S1 + S2
/// ship the default `(c)` continuous channel which needs no
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
///   `|body.evaluate(P_at_seam_plane)| < half_width(P.y)`, with
///   `half_width(P.y) = 0.5 * cross_section_width_m + tan(draft) *
///   clamp(P.y - channel_y_center, ±channel_depth/2)` for the
///   trapezoidal profile, or `cross_section_width_m / 2` for
///   `draft_angle_deg = 0`); vertical extent
///   `Y ∈ [tray_thickness - channel_depth, tray_thickness]`
///   (carves into tray from the top).
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
            draft_tan: spec.draft_angle_deg.to_radians().tan(),
        },
        channel_bounds,
    );

    let mold = tray_solid.subtract(channel_solid);

    // S3 territory: append mating transforms for §G-5 pin-routing
    // variants. S1 + S2 ship the default `(c)` continuous channel
    // which needs no transforms.
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
    /// (sock proxy along X-axis, 10 mm radius in YZ, 60 mm length
    /// along X) centered on the centerline; large bounding cuboid
    /// (200 × 30 × 30 mm). The body's XZ cross-section at the seam
    /// plane (Y=0) is a **rectangle 60 × 20 mm** (|X| ≤ 30 AND
    /// |Z| ≤ 10), bounded by the cylinder's axial caps along X and
    /// its lateral surface at |Z| = 10. `compose_gasket_mold_solid`
    /// produces a channel tracing this rectangular perimeter — a
    /// closed loop with 4 straight sides + 4 corners at (±30, ±10).
    /// S1 tests below probe the long-side perimeter (|Z| = 10 within
    /// |X| ≤ 30); S2 adds short-side + corner probes
    /// ([`gasket_channel_traces_short_side_perimeter`],
    /// [`gasket_channel_rounds_body_corner_with_half_width_radius`]).
    fn synthetic_fixture() -> (Ribbon, Solid, Solid) {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        // Body cavity: 10 mm radius cylinder along X, half_length =
        // 30 mm (so total length = 60 mm; caps at |X| = 30 mm). S1
        // cold-read pass-1 docstring claimed 60 mm but earlier code
        // used `Solid::cylinder(_, 0.060)` (= 120 mm total) — S2
        // realigns code to that docstring intent. Cross-section at the
        // seam plane is the rectangle below (|X| ≤ 30 AND |Z| ≤ 10).
        let layer_body =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
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
        // S2 envelope: draft angle in (0, 10]° — recon §G-3 calls for
        // "vertical or slightly tapered for demold"; 10° is a soft
        // upper bound (above this the cross-section asymmetry starts
        // to noticeably distort the gasket-to-cup contact face).
        assert!(
            s.draft_angle_deg > 0.0 && s.draft_angle_deg <= 10.0,
            "iter1 draft_angle_deg {} not in (0, 10]°",
            s.draft_angle_deg
        );
        // S2 envelope: workshop clamp pressure in [5, 100] kPa
        // (hand-clamp regime). Above 100 kPa requires a real C-clamp
        // and crushes soft silicone; below 5 kPa is too loose to seal.
        assert!(
            (5_000.0..=100_000.0).contains(&s.workshop_clamp_pressure_pa),
            "iter1 workshop_clamp_pressure_pa {} not in [5, 100] kPa",
            s.workshop_clamp_pressure_pa
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
        assert_eq!(d.draft_angle_deg, i.draft_angle_deg);
        assert_eq!(d.material, i.material);
        assert_eq!(d.workshop_clamp_pressure_pa, i.workshop_clamp_pressure_pa);
    }

    #[test]
    fn gasket_spec_iter1_material_is_ecoflex_per_recon_g1() {
        // §G-1: "Ecoflex 00-30 first pick (matches cast material; same
        // workshop silicone pool)". Dragon Skin 10A is the §G-1
        // fallback if Ecoflex compresses too freely at S6 iter-3.
        let s = GasketSpec::iter1();
        assert_eq!(s.material, GasketMaterial::Ecoflex0030);
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
        // Body cross-section at seam plane Y=0 is the rectangle
        // |X| ≤ 30 AND |Z| ≤ 10. Probe the long-side perimeter at
        // (X=0, Y=channel_mid, Z=+10) — on the body boundary.
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
        // Cavity interior: probe at (X=0, _, Z=5) → 5 mm from the
        // body's lateral surface (|Z| = 10) along the short axis →
        // body.evaluate ≈ -5 mm, |body.evaluate| = 5 mm >> width/2 =
        // 0.75 mm → channel SDF positive → mold solid.
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
        // Outside cavity by ~3 mm: probe at (X=0, _, Z=13) →
        // body.evaluate = max(|Z|-10, |X|-30) = max(3, -30) = +3 mm →
        // |body.evaluate| = 3 mm >> width/2 = 0.75 mm → channel SDF
        // positive → mold solid.
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
        // boundary (channel SDF ≈ 0). At the long-side perimeter
        // (|Z| = 10 within |X| ≤ 30), the channel boundary lies at
        // Z = 10 ± width/2 = 10 ± 0.75 mm.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let y_mid = 0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m);
        let half_w = 0.5 * spec.cross_section_width_m;
        // Outer boundary of channel (Z away from cavity): Z = 10 mm + 0.75 mm = 10.75 mm.
        let outer_boundary = Point3::new(0.0, y_mid, 0.010 + half_w);
        let outer_sdf = mold.evaluate(&outer_boundary);
        // Channel SDF ≈ 0 at the boundary → mold SDF (tray subtract
        // channel) ≈ 0 too. Tolerate small numerical error from
        // composition.
        assert!(
            outer_sdf.abs() < 1e-6,
            "mold SDF at outer channel boundary (Z = 10.75 mm) must be ≈ 0; got {outer_sdf}"
        );
        // Inner boundary of channel (Z toward cavity): Z = 10 mm - 0.75 mm = 9.25 mm.
        let inner_boundary = Point3::new(0.0, y_mid, 0.010 - half_w);
        let inner_sdf = mold.evaluate(&inner_boundary);
        assert!(
            inner_sdf.abs() < 1e-6,
            "mold SDF at inner channel boundary (Z = 9.25 mm) must be ≈ 0; got {inner_sdf}"
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

    /// Target FDM-tolerance floor (§G-0 typed-range; ~200 µm) the
    /// gasket must absorb at workshop clamp pressure. Shared by the
    /// S2 compression tests.
    const FDM_TOLERANCE_TARGET_M: f64 = 200e-6;

    #[test]
    fn ecoflex_iter1_predicted_compression_absorbs_fdm_tolerance() {
        // S2 compression calibration: at the iter1 spec (Ecoflex 00-30
        // + 20 kPa workshop clamp + 0.8 mm gasket thickness) the
        // Hookean compression prediction MUST exceed the FDM-tolerance
        // target (§G-0 typed-range; ~200 µm). If this fails, either
        // the material is wrong (too stiff) or the clamp pressure is
        // too low for the chosen gasket geometry.
        //
        // Hand math: strain = 20 kPa / 69 kPa ≈ 0.29; compression =
        // 0.29 × 0.8 mm = 0.232 mm = 232 µm > 200 µm target. ✓
        let s = GasketSpec::iter1();
        let compression = s.predicted_compression_m();
        assert!(
            compression >= FDM_TOLERANCE_TARGET_M,
            "iter1 Ecoflex 00-30 compression at 20 kPa = {compression} m must absorb FDM \
             tolerance ({FDM_TOLERANCE_TARGET_M} m). If this regresses, the {} \
             gasket needs a softer material, higher clamp pressure, or thicker cross-section.",
            s.material.shore_label()
        );
    }

    #[test]
    fn dragon_skin_predicted_compression_below_fdm_tolerance_at_iter1_pressure() {
        // Anchor for §G-1 "Dragon Skin 10A FALLBACK if Ecoflex compresses
        // too freely" pick — at iter1 workshop clamp pressure, Dragon
        // Skin's 220 kPa modulus means compression = 20/220 × 0.8 mm
        // ≈ 73 µm < 200 µm. This is WHY Ecoflex is iter1 default. If
        // this test starts passing (Dragon Skin DOES absorb FDM
        // tolerance), the Young's modulus estimate has changed +
        // workshop-empirical recheck is warranted.
        let s = GasketSpec {
            material: GasketMaterial::DragonSkin10A,
            ..GasketSpec::iter1()
        };
        let compression = s.predicted_compression_m();
        assert!(
            compression < FDM_TOLERANCE_TARGET_M,
            "Dragon Skin 10A compression at iter1 clamp pressure = {compression} m. Expected \
             BELOW FDM-tolerance target {FDM_TOLERANCE_TARGET_M} m to anchor iter1 Ecoflex \
             pick. If this passes, re-validate Young's modulus prediction or pick Dragon Skin."
        );
    }

    #[test]
    fn predicted_compression_scales_linearly_with_clamp_pressure() {
        // Hookean invariant: doubling pressure doubles compression at
        // fixed material + thickness. Validates the predicted-
        // compression formula isn't inadvertently quadratic / inverted.
        let base = GasketSpec::iter1();
        let doubled = GasketSpec {
            workshop_clamp_pressure_pa: 2.0 * base.workshop_clamp_pressure_pa,
            ..base
        };
        assert_relative_eq!(
            doubled.predicted_compression_m(),
            2.0 * base.predicted_compression_m(),
            epsilon = 1e-12,
        );
    }

    #[test]
    fn trapezoidal_channel_wider_at_top_than_bottom_on_perimeter() {
        // S2 trapezoidal cross-section invariant: at the body perimeter
        // (where body.evaluate = 0), the channel outer boundary lies
        // at lateral offset = half-width(Y). For non-zero draft, the
        // top-Y half-width > bottom-Y half-width by depth × tan(draft).
        // Probe both Y planes at the body's long-side perimeter
        // (Z = 10 mm, X = 0) and verify the boundary positions.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let draft_tan = spec.draft_angle_deg.to_radians().tan();
        let half_depth = 0.5 * spec.channel_depth_m;
        // Top channel Y = tray_thickness; bottom = tray_thickness - depth.
        let y_top = spec.tray_thickness_m;
        let y_bottom = spec.tray_thickness_m - spec.channel_depth_m;
        // Trapezoidal half-width: top wider by + half_depth × tan(draft),
        // bottom narrower by - half_depth × tan(draft).
        let half_w_top = draft_tan.mul_add(half_depth, 0.5 * spec.cross_section_width_m);
        let half_w_bottom = draft_tan.mul_add(-half_depth, 0.5 * spec.cross_section_width_m);
        assert!(
            half_w_top > half_w_bottom,
            "trapezoidal top half-width {half_w_top} must exceed bottom half-width \
             {half_w_bottom} (positive draft → wider at demold opening)"
        );
        // Probe just BELOW tray top (Y = y_top - ε) at the predicted
        // outer boundary (Z = 10 + half_w_top): channel SDF ≈ 0 →
        // mold SDF ≈ 0.
        let probe_top = Point3::new(0.0, y_top - 1e-6, 0.010 + half_w_top);
        let sdf_top = mold.evaluate(&probe_top);
        assert!(
            sdf_top.abs() < 5e-6,
            "mold SDF at trapezoidal TOP outer boundary (Z = {} m) must be ≈ 0; got {sdf_top}",
            0.010 + half_w_top
        );
        // Probe just ABOVE channel floor (Y = y_bottom + ε) at the
        // predicted (narrower) outer boundary (Z = 10 + half_w_bottom).
        let probe_bottom = Point3::new(0.0, y_bottom + 1e-6, 0.010 + half_w_bottom);
        let sdf_bottom = mold.evaluate(&probe_bottom);
        assert!(
            sdf_bottom.abs() < 5e-6,
            "mold SDF at trapezoidal BOTTOM outer boundary (Z = {} m) must be ≈ 0; \
             got {sdf_bottom}",
            0.010 + half_w_bottom
        );
        // Cross-check: at the TOP Y plane, probing at the BOTTOM-Y half-
        // width (Z = 10 + half_w_bottom) lands INSIDE the channel void
        // (channel wider at top → bottom-width position is still inside).
        let cross_check = Point3::new(0.0, y_top - 1e-6, 0.010 + half_w_bottom);
        let sdf_cross = mold.evaluate(&cross_check);
        assert!(
            sdf_cross > 0.0,
            "at top-Y, bottom-half-width Z position must be INSIDE channel void (SDF > 0); \
             got {sdf_cross}. If this fails, the trapezoid is inverted (narrower at top) \
             — demold direction is reversed."
        );
    }

    #[test]
    fn rectangular_cross_section_zero_draft_constant_width() {
        // Recon §G-3 lists "vertical or slightly tapered" walls as
        // co-equal demold options; vertical = `draft_angle_deg = 0`
        // reduces the trapezoidal channel SDF to the S1 rectangular
        // case bit-precisely. Verify top + bottom half-widths match
        // within numerical noise.
        let spec = GasketSpec {
            draft_angle_deg: 0.0,
            ..GasketSpec::iter1()
        };
        let (ribbon, body, bounds) = synthetic_fixture();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let half_w = 0.5 * spec.cross_section_width_m;
        let y_top = spec.tray_thickness_m;
        let y_bottom = spec.tray_thickness_m - spec.channel_depth_m;
        // Same Z boundary at both Y planes → both SDFs ≈ 0.
        let probe_top = Point3::new(0.0, y_top - 1e-6, 0.010 + half_w);
        let probe_bottom = Point3::new(0.0, y_bottom + 1e-6, 0.010 + half_w);
        let sdf_top = mold.evaluate(&probe_top);
        let sdf_bottom = mold.evaluate(&probe_bottom);
        assert!(
            sdf_top.abs() < 5e-6 && sdf_bottom.abs() < 5e-6,
            "rectangular (draft=0) channel must have constant width across Y: \
             top SDF {sdf_top}, bottom SDF {sdf_bottom}, both should be ≈ 0"
        );
    }

    #[test]
    fn gasket_channel_traces_short_side_perimeter() {
        // Cold-read pass-1 S2 carryover (commit 6c8bc7f8): the bit-
        // precise width invariant test only probed the long-side
        // rectangle perimeter (Z = ±10). The short-side perimeter
        // lies at X = ±30 (the cylinder's axial caps). Verify the
        // channel SDF also traces it: probe at (X = 30 ± half_w,
        // Y = mid, Z = 0) → channel boundary (mid-Y half-width is
        // bit-precisely `0.5 * cross_section_width_m` regardless of
        // draft angle).
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let y_mid = 0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m);
        let half_w = 0.5 * spec.cross_section_width_m;
        // Short-side outer boundary: X = 30 + half_w, Y = mid, Z = 0.
        let outer = Point3::new(0.030 + half_w, y_mid, 0.0);
        let outer_sdf = mold.evaluate(&outer);
        assert!(
            outer_sdf.abs() < 1e-6,
            "mold SDF at short-side outer boundary (X = {} m) must be ≈ 0; got {outer_sdf}",
            0.030 + half_w
        );
        // Short-side inner boundary: X = 30 - half_w, Y = mid, Z = 0.
        let inner = Point3::new(0.030 - half_w, y_mid, 0.0);
        let inner_sdf = mold.evaluate(&inner);
        assert!(
            inner_sdf.abs() < 1e-6,
            "mold SDF at short-side inner boundary (X = {} m) must be ≈ 0; got {inner_sdf}",
            0.030 - half_w
        );
    }

    #[test]
    fn gasket_channel_rounds_body_corner_with_half_width_radius() {
        // Cold-read pass-1 S2 carryover (commit 6c8bc7f8): the 90°
        // corner of the body rectangle (X = ±30, Z = ±10) is rounded
        // by the body SDF — finite-cylinder SDF at the cap × lateral
        // junction returns Euclidean distance to the corner. At
        // mid-Y, the channel outer boundary therefore traces a
        // quarter-arc of radius half-width around the corner.
        //
        // Probe at the corner-bisector at distance half_w from the
        // corner point: (X = 30 + half_w*cos(45°), Y = mid,
        // Z = 10 + half_w*sin(45°)) → body_dist = half_w → channel
        // SDF ≈ 0.
        let (ribbon, body, bounds) = synthetic_fixture();
        let spec = GasketSpec::iter1();
        let (mold, _) = compose_gasket_mold_solid(&ribbon, &body, &bounds, &spec).unwrap();
        let y_mid = 0.5_f64.mul_add(-spec.channel_depth_m, spec.tray_thickness_m);
        let half_w = 0.5 * spec.cross_section_width_m;
        let cos45 = std::f64::consts::FRAC_1_SQRT_2;
        let corner_arc = Point3::new(0.030 + half_w * cos45, y_mid, 0.010 + half_w * cos45);
        let corner_sdf = mold.evaluate(&corner_arc);
        assert!(
            corner_sdf.abs() < 1e-6,
            "mold SDF at corner-arc midpoint (half_w from body corner along bisector) \
             must be ≈ 0; got {corner_sdf}. If this fails, the body SDF doesn't \
             round the corner — channel won't trace continuously around it."
        );
        // Probe AT the body corner itself (X=30, Y=mid, Z=10) →
        // body_dist = 0 → channel SDF = -half_w (well inside channel
        // void). Mold SDF > 0.
        let corner_point = Point3::new(0.030, y_mid, 0.010);
        let corner_point_sdf = mold.evaluate(&corner_point);
        assert!(
            corner_point_sdf > 0.0,
            "mold SDF AT body corner must be in channel void (SDF > 0); got {corner_point_sdf}"
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
