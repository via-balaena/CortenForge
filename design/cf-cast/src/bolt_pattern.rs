//! M5 through-bolt clamp pattern (§B of
//! `docs/CF_CAST_FLANGE_CONTINUITY_BOLT_PATTERN_RECON.md`).
//!
//! Both cup-halves get IDENTICAL bolt-clearance holes carved through
//! their mating faces, arc-length-equal-spaced around the body
//! silhouette in the flange band. Workshop user inserts an M5 hex
//! bolt + flat washer through one back face, threads a flat washer +
//! hex nut on the other side, and hand-tightens to compress the
//! gasket. Mirrors the engine-valve-cover clamp pattern; replaces the
//! pre-§B hand-clamped-with-rubber-bands assembly.
//!
//! Composes on top of the §M unified-mating-plane arc: the symmetric
//! [`crate::dowel_hole`] pattern registers the two halves laterally,
//! while the bolt pattern clamps them axially across the gasket.
//!
//! # Geometry
//!
//! Each bolt hole is a [`MatingTransform::SubtractCylinder`] post-MC
//! mesh-CSG primitive with:
//! - **axis** = ribbon binormal (perpendicular to mating plane —
//!   parallel to print Z when the cup piece is mated-face-down on the
//!   build plate, so the hole prints as a vertical cylinder requiring
//!   no support).
//! - **center** = silhouette point at arc fraction
//!   `(k + 0.5) / count` (k = 0..count) offset OUTWARD from the
//!   silhouette by `silhouette_outboard_offset_m` in the seam plane,
//!   then projected onto the binormal-aligned seam plane through the
//!   ribbon midpoint (same projection §M-S2 dowel-hole uses).
//! - **`half_length`** = `flange_thickness_m + BOLT_AXIAL_SLACK_M`.
//!   The full cylinder spans `2 × half_length` straddling the seam
//!   plane, carving a through-hole through BOTH cup-halves' flange
//!   material. The +slack ensures the cylindrical caps are well
//!   OUTSIDE the flange's back face on each side (no mesh-CSG
//!   operating on coincident endcap planes).
//! - **radius** = `clearance_diameter_m / 2`. Workshop user supplies
//!   the M5 bolt + washer; the washer compensates for FDM dimensional
//!   error on the hole, so no extra radial clearance slack is added.
//!
//! # Bolt-dowel arc-length stagger (automatic at default counts)
//!
//! Both this module and [`crate::dowel_hole`] use the same
//! `(k + 0.5) / count` arc-fraction formula. With the iter-1 defaults
//! (`DowelHoleSpec::iter1().count = 4` + `BoltPatternSpec::iter1().count
//! = 8`), bolt positions naturally interleave between dowel positions
//! — dowels at arc fractions {1/8, 3/8, 5/8, 7/8}, bolts at
//! {1/16, 3/16, 5/16, …, 15/16}, minimum arc-length separation =
//! 1/16 of perimeter. For typical sock geometry (200 mm perimeter)
//! that's 12.5 mm separation, far above the radial-collision
//! threshold of `dowel_radius + bolt_radius + clearance`. Workshop
//! configs that violate this (e.g., equal counts) are gated at parse
//! time in `cf-cast-cli::config::validate_after_layer_source`.
//!
//! # Pour-gate collision skip
//!
//! The V-shape pour gate at the dome apex emits
//! [`MatingTransform::SubtractCylinder`] legs that may overlap with
//! arc-length-equal-spaced bolt positions near the dome. When
//! `spec.skip_pour_gate_collision` is true (the default), each bolt
//! position is checked against the ribbon's pour-gate cylinder
//! centerlines; bolts whose centers fall within
//! `bolt_radius + pour_gate_radius + spec.pour_gate_clearance_m` of
//! any pour-gate cylinder axis are dropped silently (no rebalance —
//! the workshop accepts a slight asymmetry near the pour gate; full
//! rebalance is bookmarked for a future arc).
//!
//! When `spec.bracket_pour_gate` is true (the organic-parts apex-axial
//! pour, 2026-05-29) the pour is bracketed PROACTIVELY: before the
//! arc-spaced loop, `find_pour_arc_fraction` locates where the bore
//! pierces the flange and `place_pour_flanking_bolts` places a bolt
//! hugging it on EACH side. The collision-skip above is reactive (it
//! only fires when an arc-bolt lands inside the clearance zone), which
//! a bore sitting in a gap between bolt slots never triggers — so the
//! proactive pair guarantees the split flange is clamped at the pour
//! regardless of arc spacing. Arc-bolts that then collide with the bore
//! (or double onto a flanking bolt) are dropped.
//!
//! # Side-agnostic emission
//!
//! Identical transform list applied to BOTH Negative + Positive cup
//! pieces (mesh-CSG subtract is invariant to which half it's applied
//! to; cylinder portion outside a given half is a no-op). Both
//! halves get matching bolt-hole positions mirrored across the seam
//! plane, so a single M5 bolt passes through the corresponding holes
//! on the two halves to clamp them together.

use cf_design::Solid;
use nalgebra::{Point3, Unit, Vector3};

use crate::flange::FlangeSpec;
use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::pour::build_pour_gate_transforms;
use crate::ribbon::Ribbon;
use crate::silhouette_2d::{SILHOUETTE_GRID_STEP_M, Silhouette2d};

/// Default bolt clearance hole diameter (5.5 mm) — standard M5
/// clearance per ISO 273 medium-fit.
const DEFAULT_CLEARANCE_DIAMETER_M: f64 = 0.0055;

/// Default bolt count around the silhouette (8 — even pressure
/// distribution on the iter-1 sock geometry, ~25 mm spacing on a
/// typical 200 mm perimeter; engine-valve-cover-equivalent for a
/// small mold). Workshop user picks per §B recon OQ1.
const DEFAULT_COUNT: u32 = 8;

/// Default radial offset from the body silhouette curve to the bolt
/// centerline (13 mm). Workshop user feedback 2026-05-27 cf-view
/// smoke: the bolt-HEAD/WASHER footprint must clear the cup-wall
/// outer step (cup-wall extends to `body_dist = wall_thickness_m =
/// 5 mm`, then steps DOWN past the flange's back face on the cup
/// piece's outer shell). M5 washer OD = 10 mm = 5 mm radius. Bolt
/// offset must satisfy `bolt_offset - washer_radius >=
/// wall_thickness + safety_margin`. With 3 mm safety margin
/// (absorbs FDM ±0.3 mm tolerance + body-curvature variation up to
/// 2 mm in the Y direction below the seam plane): 13 mm offset
/// gives washer inboard edge at `body_dist 8 mm`, cup-wall outer at
/// 5 mm → **3 mm clearance**. Pre-fix 9 mm offset gave -1 mm
/// clearance (washer landed ON cup-wall step), would have caused
/// non-flat washer seating + uneven bolt tension.
const DEFAULT_OUTBOARD_OFFSET_M: f64 = 0.013;

/// Default minimum clearance between bolt cylinder + pour-gate
/// cylinder centerlines, beyond the sum of the two radii. 1 mm of
/// PLA between the two cylindrical voids — well above the FDM-PLA
/// minimum-wall floor.
const DEFAULT_POUR_GATE_CLEARANCE_M: f64 = 0.001;

/// Polygonal facets around each bolt cylinder. 32 segments give
/// ~0.4 mm chord error at 5.5 mm Ø, well below FDM print resolution
/// and matching the segment count of the dowel-HOLE
/// [`SubtractCylinder`] primitives so both surfaces have uniform
/// chord error.
const DEFAULT_SEGMENTS: u32 = 32;

/// Extra axial slack on each side of the hole's nominal half-length
/// (= `flange_thickness_m`).
///
/// Ensures the cylinder's CYLINDRICAL flat caps are well OUTSIDE the
/// flange's back face on each side, avoiding mesh-CSG operating on
/// coincident endcap planes at the flange surface. 0.5 mm is well
/// below FDM resolution + safely past any back-face MC ambiguity.
/// Public so `crate::procedure` can quote the bolt-traverse length
/// accurately in workshop-facing markdown.
pub const BOLT_AXIAL_SLACK_M: f64 = 0.0005;

/// M5 bolt-pattern geometry parameter envelope per §B-S1 recon.
///
/// Workshop user picks via cf-cast-cli `[bolt_pattern]` TOML block;
/// iter-1 starts at the defaults below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoltPatternSpec {
    /// Bolt clearance hole diameter (meters). Default 5.5 mm (M5
    /// ISO 273 medium fit).
    pub clearance_diameter_m: f64,
    /// Number of bolts arc-length-equal-spaced around the silhouette.
    /// Default 8.
    pub count: u32,
    /// Radial offset from the body silhouette curve to the bolt
    /// centerline (meters). Default 13 mm (bumped 9 → 13 mm in iter-1
    /// so the M5 washer footprint clears the cup-wall outer step;
    /// see `DEFAULT_OUTBOARD_OFFSET_M` rationale).
    pub silhouette_outboard_offset_m: f64,
    /// Whether to silently drop bolts that collide with the pour-gate
    /// channels. Default `true`. When `false`, every bolt is emitted
    /// regardless — workshop may need to drill out the collision-zone
    /// pour-gate leg by hand or accept that those bolts can't seat.
    pub skip_pour_gate_collision: bool,
    /// Minimum clearance distance between bolt cylinder + pour-gate
    /// cylinder centerlines, ABOVE the sum of the two radii (meters).
    /// Default 1 mm. Only consulted when
    /// `skip_pour_gate_collision = true`.
    pub pour_gate_clearance_m: f64,
    /// When `true`, place two bolts PROACTIVELY flanking the pour bore
    /// (one each side, as close as the outboard offset allows), so the
    /// flange the bore splits stays clamped — regardless of where the
    /// arc-spaced bolts fall. Default `false` (existing casts
    /// byte-identical).
    ///
    /// Pairs with [`crate::pour::PourGateLayout::ApexAxial`] (a single
    /// axial pour bore on the seam splits the flange at the dome apex).
    /// Supersedes the reactive collision-bracket, which never fired
    /// when the bore sat in a gap between bolt slots. Organic-parts
    /// arc, 2026-05-29.
    pub bracket_pour_gate: bool,
}

impl BoltPatternSpec {
    /// Workshop iter-1 starting defaults pinned at §B-S1 recon.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            clearance_diameter_m: DEFAULT_CLEARANCE_DIAMETER_M,
            count: DEFAULT_COUNT,
            silhouette_outboard_offset_m: DEFAULT_OUTBOARD_OFFSET_M,
            skip_pour_gate_collision: true,
            pour_gate_clearance_m: DEFAULT_POUR_GATE_CLEARANCE_M,
            bracket_pour_gate: false,
        }
    }
}

impl Default for BoltPatternSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Bolt-pattern kind on the [`Ribbon`].
///
/// Matches the existing `DowelHoleKind` / `FlangeKind` / `GasketKind`
/// / `PourGateKind` / `PlugPinKind` patterns — bridges the
/// cf-cast-cli `[bolt_pattern]` TOML block to the per-piece bolt-hole
/// emission in [`crate::compose_piece_solid`].
///
/// Default [`Self::None`] (no bolt holes — cup pieces clamped by hand
/// or by external method). Set
/// `BoltPatternKind::Auto(BoltPatternSpec::iter1())` via
/// [`Ribbon::with_bolt_pattern`] to enable the M5 through-bolt
/// clamp pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoltPatternKind {
    /// No bolt-hole emission.
    None,
    /// Arc-length-equal-spaced M5 bolt holes around the silhouette,
    /// parameterized by the inner [`BoltPatternSpec`].
    Auto(BoltPatternSpec),
}

impl BoltPatternKind {
    /// Returns the inner [`BoltPatternSpec`] for [`BoltPatternKind::Auto`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&BoltPatternSpec> {
        match self {
            Self::None => None,
            Self::Auto(spec) => Some(spec),
        }
    }
}

/// Distance from point `p` to the FINITE cylinder axis segment
/// centered at `c` with unit direction `axis` and half-length
/// `half_length`. The cylinder's axis spans `c ± half_length * axis`;
/// the returned distance is the perpendicular distance to that
/// segment, clamping the axis-projection of `p` to the endcaps so
/// queries past the cylinder's caps measure to the cap center rather
/// than to the infinite axis line.
///
/// Used by the pour-gate collision check — pour-gate legs are short
/// (≈40-45 mm half-length) and a bolt whose axis-projection lands
/// PAST the leg's endcap is not actually colliding with the leg even
/// if its perpendicular-line distance is small. Cold-read finding
/// 2026-05-27 (I1).
///
/// `axis` must already be unit-length (the caller passes
/// `CylinderParent.axis: Unit<Vector3<f64>>`).
fn point_to_cylinder_axis_distance(
    p: Point3<f64>,
    c: Point3<f64>,
    axis: Vector3<f64>,
    half_length: f64,
) -> f64 {
    let to_p = p - c;
    // Project `to_p` onto axis; clamp to ±half_length to stay within
    // the finite cylinder. Closest axis point is then
    // `c + clamped_t * axis`.
    let t = to_p.dot(&axis).clamp(-half_length, half_length);
    let closest = c + axis * t;
    (p - closest).norm()
}

/// Returns `true` if a bolt at `bolt_center` with radius `bolt_radius`
/// collides with any pour-gate cylinder in `pour_xforms`, within
/// `extra_clearance` of the cylinder surfaces.
fn collides_with_pour_gate(
    bolt_center: Point3<f64>,
    bolt_radius: f64,
    pour_xforms: &[MatingTransform],
    extra_clearance: f64,
) -> bool {
    for t in pour_xforms {
        if let MatingTransform::SubtractCylinder { params } = t {
            let dist = point_to_cylinder_axis_distance(
                bolt_center,
                params.parent.center_m,
                params.parent.axis.into_inner(),
                params.parent.half_length_m,
            );
            if dist < bolt_radius + params.radius_m + extra_clearance {
                return true;
            }
        }
    }
    false
}

/// Build the side-agnostic bolt-pattern [`MatingTransform`] vec for
/// the given `ribbon.bolt_pattern` spec + layer body silhouette.
///
/// Returns an empty Vec if the silhouette has no polylines (e.g.,
/// body extends past silhouette region) — caller treats this as
/// "no bolt holes for this piece".
///
/// `flange_spec` carries the `flange_thickness_m` that sizes the
/// bolt cylinder half-length (the bolt must carve through both
/// halves' flange material). `bounds` is the cup-piece's MC bounds
/// (used to size the silhouette sampling region — same as the
/// flange's silhouette-construction bounds for consistency).
#[must_use]
pub fn build_bolt_pattern_transforms(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &BoltPatternSpec,
    flange_spec: &FlangeSpec,
    bounds: cf_design::Aabb,
) -> Vec<MatingTransform> {
    if spec.count == 0 {
        return Vec::new();
    }
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();

    let radius_m = spec.clearance_diameter_m / 2.0;
    // Silhouette sampling padding: bolt center + radius + grid step.
    let pad = spec.silhouette_outboard_offset_m + radius_m + SILHOUETTE_GRID_STEP_M;
    // Fitted seam (item A) → sample IN the seam plane; else the legacy X-Z
    // constant-Y silhouette (bit-identical to the pre-generalization path).
    let fitted = ribbon.seam_plane_basis();
    let silhouette = fitted.map_or_else(
        || {
            Silhouette2d::from_body_at_y(
                layer_body,
                seam_midpoint.y,
                bounds.min.x - pad,
                bounds.max.x + pad,
                bounds.min.z - pad,
                bounds.max.z + pad,
            )
        },
        |basis| {
            let expanded = cf_design::Aabb::new(
                Point3::new(bounds.min.x - pad, bounds.min.y - pad, bounds.min.z - pad),
                Point3::new(bounds.max.x + pad, bounds.max.y + pad, bounds.max.z + pad),
            );
            let (u_min, u_max, v_min, v_max) = basis.inplane_bounds(expanded);
            Silhouette2d::from_body_in_plane(layer_body, basis, u_min, u_max, v_min, v_max)
        },
    );

    // Through-hole half-length = flange thickness + slack. Cylinder
    // spans ±(flange + slack) in binormal direction, carving through
    // BOTH halves' flange material (which together span ±flange in
    // binormal). Slack ensures the cylindrical caps sit OUTSIDE the
    // flange's back face on each side.
    let half_length_m = flange_spec.flange_thickness_m + BOLT_AXIAL_SLACK_M;
    let axis = Unit::new_normalize(binormal);

    let pour_xforms = build_pour_gate_transforms(ribbon);
    let offset = spec.silhouette_outboard_offset_m;
    let fitted = fitted.is_some();

    let mut transforms = Vec::with_capacity(spec.count as usize + 2);

    // PROACTIVELY flank the pour bore (apex-axial pour). The collision-skip
    // below is REACTIVE — it only fires when an arc-spaced bolt happens to land
    // inside the pour's clearance zone, which a bore sitting in a gap between
    // bolt slots never triggers (workshop 2026-05-29: the nearest bolt was
    // 16 mm out, so the old bracket never fired). Find where the bore pierces
    // the flange and place a bolt hugging it on EACH side, so the split flange
    // is clamped at the pour regardless of the arc spacing.
    if spec.bracket_pour_gate && !pour_xforms.is_empty() {
        if let Some(t_pour) = find_pour_arc_fraction(
            &silhouette,
            fitted,
            offset,
            seam_midpoint,
            binormal,
            &pour_xforms,
        ) {
            place_pour_flanking_bolts(
                &mut transforms,
                t_pour,
                &silhouette,
                fitted,
                offset,
                seam_midpoint,
                binormal,
                radius_m,
                axis,
                half_length_m,
                &pour_xforms,
            );
        }
    }

    for k in 0..spec.count {
        // Same (k + 0.5) / count formula as dowel_hole — produces
        // natural arc-length stagger between dowel + bolt patterns
        // when their counts differ (the iter-1 defaults give
        // bolts {1/16, 3/16, …} interleaved with dowels {2/16, 6/16,
        // 10/16, 14/16}). The +0.5 also avoids placing a bolt at the
        // silhouette's polyline-assembly join point.
        let t = (f64::from(k) + 0.5) / f64::from(spec.count);
        if silhouette.point_at_arc_fraction(t).is_none() {
            return Vec::new();
        }
        // Offset outward from the silhouette, then project onto the seam plane
        // (in-plane for a fitted seam, legacy X-Z otherwise) — see
        // `bolt_center_at`.
        let Some(center_m) =
            bolt_center_at(&silhouette, fitted, offset, seam_midpoint, binormal, t)
        else {
            continue;
        };

        if spec.skip_pour_gate_collision
            && collides_with_pour_gate(center_m, radius_m, &pour_xforms, spec.pour_gate_clearance_m)
        {
            // The proactive flanking pair (above) clamps the pour; drop this
            // colliding arc-bolt rather than leave it half-carved by the bore.
            continue;
        }
        // Don't double an arc-bolt onto a flanking bolt placed above.
        if spec.bracket_pour_gate && too_close_to_existing(center_m, &transforms, 2.0 * radius_m) {
            continue;
        }

        transforms.push(MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m,
                    axis,
                    half_length_m,
                },
                radius_m,
                segments: DEFAULT_SEGMENTS,
            },
        });
    }
    transforms
}

/// The bolt center for silhouette arc fraction `t`: the silhouette point offset
/// outward by `offset`, mapped to world (in-plane for a fitted seam, legacy X-Z
/// otherwise) and projected onto the seam plane through `seam_midpoint`. `None`
/// when the silhouette has no polyline at `t`.
///
/// The legacy (`fitted = false`) branch keeps the exact `mul_add` form so the
/// Y-normal-seam casts stay bit-identical.
fn bolt_center_at(
    silhouette: &Silhouette2d,
    fitted: bool,
    offset: f64,
    seam_midpoint: Point3<f64>,
    binormal: Vector3<f64>,
    t: f64,
) -> Option<Point3<f64>> {
    let p = silhouette.point_at_arc_fraction(t)?;
    let n = silhouette.outward_normal_at_arc_fraction(t)?;
    let candidate = if fitted {
        silhouette.to_world(p) + silhouette.dir_to_world(n) * offset
    } else {
        Point3::new(
            offset.mul_add(n.x, p.x),
            seam_midpoint.y,
            offset.mul_add(n.z, p.z),
        )
    };
    let signed = (candidate - seam_midpoint).dot(&binormal);
    Some(candidate - signed * binormal)
}

/// Distance from `p` to the nearest pour-gate cylinder AXIS (finite-segment
/// clamped), or `f64::MAX` if there are no pour cylinders.
fn pour_axis_distance(p: Point3<f64>, pour_xforms: &[MatingTransform]) -> f64 {
    pour_xforms
        .iter()
        .filter_map(|t| match t {
            MatingTransform::SubtractCylinder { params } => Some(point_to_cylinder_axis_distance(
                p,
                params.parent.center_m,
                params.parent.axis.into_inner(),
                params.parent.half_length_m,
            )),
            _ => None,
        })
        .fold(f64::MAX, f64::min)
}

/// `true` if `center` is within `min_sep` of any already-emitted bolt — guards
/// against doubling an arc-bolt onto a proactively-placed flanking bolt.
fn too_close_to_existing(
    center: Point3<f64>,
    transforms: &[MatingTransform],
    min_sep: f64,
) -> bool {
    transforms.iter().any(|xf| match xf {
        MatingTransform::SubtractCylinder { params } => {
            (params.parent.center_m - center).norm() < min_sep
        }
        _ => false,
    })
}

/// Largest pour-gate cylinder radius in `pour_xforms` (0 if none).
fn max_pour_radius(pour_xforms: &[MatingTransform]) -> f64 {
    pour_xforms
        .iter()
        .filter_map(|t| match t {
            MatingTransform::SubtractCylinder { params } => Some(params.radius_m),
            _ => None,
        })
        .fold(0.0_f64, f64::max)
}

/// Arc fraction where the pour bore pierces the flange — the silhouette
/// fraction whose bolt center is closest to the pour-gate axis. Dense scan
/// (arc length is monotone in `t`). `None` if no fraction has a polyline.
fn find_pour_arc_fraction(
    silhouette: &Silhouette2d,
    fitted: bool,
    offset: f64,
    seam_midpoint: Point3<f64>,
    binormal: Vector3<f64>,
    pour_xforms: &[MatingTransform],
) -> Option<f64> {
    /// Arc-fraction scan resolution.
    const SCAN: u32 = 512;
    let mut best: Option<(f64, f64)> = None;
    for i in 0..SCAN {
        let t = f64::from(i) / f64::from(SCAN);
        let Some(center) = bolt_center_at(silhouette, fitted, offset, seam_midpoint, binormal, t)
        else {
            continue;
        };
        let dist = pour_axis_distance(center, pour_xforms);
        if best.is_none_or(|(best_dist, _)| dist < best_dist) {
            best = Some((dist, t));
        }
    }
    best.map(|(_, t)| t)
}

/// Place a bolt hugging the pour bore on EACH side: from `t_pour` (where the
/// bore pierces the flange) step outward along the silhouette in each direction
/// until the bolt's PERPENDICULAR distance to the pour axis clears the bore
/// (`pour_r + bolt_r + clearance`), then emit one bolt there. Emits up to 2.
///
/// Stepping along the silhouette moves the bolt LATERALLY around the bore (the
/// in-plane direction ⟂ the axis), unlike the outward-normal offset — which
/// near the apex points ALONG the axis and would leave the bolt on the bore.
/// A side that can't clear within `MAX_STEPS`, or whose clear position doubles
/// onto an existing bolt, is skipped.
// Flanking needs the full placement context (silhouette + seam frame + cylinder
// params); bundling into a struct would just shuffle the same fields.
#[allow(clippy::too_many_arguments)]
fn place_pour_flanking_bolts(
    transforms: &mut Vec<MatingTransform>,
    t_pour: f64,
    silhouette: &Silhouette2d,
    fitted: bool,
    offset: f64,
    seam_midpoint: Point3<f64>,
    binormal: Vector3<f64>,
    radius_m: f64,
    axis: Unit<Vector3<f64>>,
    half_length_m: f64,
    pour_xforms: &[MatingTransform],
) {
    /// Arc-fraction step when searching outward from the pour.
    const STEP: f64 = 1.0 / 512.0;
    /// Search cap per side (~1/5 of the perimeter at STEP).
    const MAX_STEPS: u32 = 96;
    // Clear the bore by `clearance` on each side (= the collision threshold).
    let clear = max_pour_radius(pour_xforms) + radius_m + DEFAULT_POUR_GATE_CLEARANCE_M;
    let min_sep = 2.0 * radius_m;

    for dir in [-1.0_f64, 1.0_f64] {
        for step in 1..=MAX_STEPS {
            let t = (dir * STEP)
                .mul_add(f64::from(step), t_pour)
                .rem_euclid(1.0);
            let Some(center_m) =
                bolt_center_at(silhouette, fitted, offset, seam_midpoint, binormal, t)
            else {
                continue;
            };
            // Step until LATERALLY clear of the bore (perpendicular distance).
            if pour_axis_distance(center_m, pour_xforms) < clear {
                continue;
            }
            // Already clamped by a neighbor on this side? Don't double up.
            if too_close_to_existing(center_m, transforms, min_sep) {
                break;
            }
            transforms.push(MatingTransform::SubtractCylinder {
                params: CylinderParams {
                    parent: CylinderParent {
                        center_m,
                        axis,
                        half_length_m,
                    },
                    radius_m,
                    segments: DEFAULT_SEGMENTS,
                },
            });
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::pour::{PourGateKind, PourGateLayout, PourGateSpec};
    use crate::ribbon::SplitNormal;
    use nalgebra::Vector3;

    fn cylinder_fixture() -> (Solid, cf_design::Aabb, Ribbon) {
        // Cylinder along X, R=10 mm, length 60 mm.
        let body = Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        (body, bounds, ribbon)
    }

    #[test]
    fn bolt_pattern_spec_iter1_defaults() {
        let s = BoltPatternSpec::iter1();
        assert_eq!(s.clearance_diameter_m, 0.0055);
        assert_eq!(s.count, 8);
        assert_eq!(s.silhouette_outboard_offset_m, 0.013);
        assert!(s.skip_pour_gate_collision);
        assert_eq!(s.pour_gate_clearance_m, 0.001);
        assert!(
            !s.bracket_pour_gate,
            "iter-1 default keeps the lossy skip (byte-identical); bracketing \
             is the organic-parts apex-axial opt-in"
        );
    }

    /// Bracketing the pour gate (apex-axial organic-parts opt-in):
    /// instead of DROPPING bolts that collide with the apex pour bore,
    /// `bracket_pour_gate = true` adds bolts just outside the
    /// clearance on each side. Invariants: (a) no emitted bolt
    /// collides with the pour gate in EITHER mode, (b) the fixture
    /// actually exercises a collision (skip drops at least one), and
    /// (c) bracket mode never emits FEWER bolts than skip mode.
    #[test]
    fn bracket_pour_gate_brackets_instead_of_dropping() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(pour));

        let spec_skip = BoltPatternSpec::iter1(); // bracket_pour_gate = false
        let spec_bracket = BoltPatternSpec {
            bracket_pour_gate: true,
            ..BoltPatternSpec::iter1()
        };
        let flange = FlangeSpec::iter1();
        let radius = spec_skip.clearance_diameter_m / 2.0;
        let pour_xforms = build_pour_gate_transforms(&ribbon);

        let skipped = build_bolt_pattern_transforms(&body, &ribbon, &spec_skip, &flange, bounds);
        let bracketed =
            build_bolt_pattern_transforms(&body, &ribbon, &spec_bracket, &flange, bounds);

        // (b) the fixture's apex pour bore collides with at least one
        // arc-spaced bolt (else this test proves nothing).
        assert!(
            skipped.len() < spec_skip.count as usize,
            "fixture must exercise a pour-gate collision; skip emitted all \
             {} bolts (no collision)",
            spec_skip.count
        );
        // (c) bracketing adds back what skip dropped.
        assert!(
            bracketed.len() >= skipped.len(),
            "bracket mode must not emit fewer bolts than skip; skip {} vs \
             bracket {}",
            skipped.len(),
            bracketed.len()
        );
        // (a) no emitted bolt collides with the pour gate, either mode.
        for xf in bracketed.iter().chain(skipped.iter()) {
            let MatingTransform::SubtractCylinder { params } = xf else {
                continue;
            };
            assert!(
                !collides_with_pour_gate(
                    params.parent.center_m,
                    radius,
                    &pour_xforms,
                    spec_skip.pour_gate_clearance_m,
                ),
                "no emitted bolt may collide with the pour gate; \
                 center {:?} does",
                params.parent.center_m
            );
        }
    }

    /// The proactive fix: `bracket_pour_gate` must land at least TWO bolts
    /// hugging the pour (one each side), where the reactive skip leaves a gap.
    /// Counts bolts within `offset + bolt_r + 4 mm` of the pour axis — the
    /// flanking pair sits at ~the outboard offset, far closer than the
    /// arc-spaced bolts that miss the pour.
    #[test]
    fn bracket_pour_gate_lands_two_bolts_close_to_the_pour() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(pour));
        let flange = FlangeSpec::iter1();

        let spec_skip = BoltPatternSpec::iter1();
        let spec_bracket = BoltPatternSpec {
            bracket_pour_gate: true,
            ..BoltPatternSpec::iter1()
        };
        let radius = spec_skip.clearance_diameter_m / 2.0;
        let near = spec_skip.silhouette_outboard_offset_m + radius + 0.004;

        let pour_xforms = build_pour_gate_transforms(&ribbon);
        let count_near = |xs: &[MatingTransform]| {
            xs.iter()
                .filter(|xf| match xf {
                    MatingTransform::SubtractCylinder { params } => {
                        pour_axis_distance(params.parent.center_m, &pour_xforms) < near
                    }
                    _ => false,
                })
                .count()
        };

        let skipped = build_bolt_pattern_transforms(&body, &ribbon, &spec_skip, &flange, bounds);
        let bracketed =
            build_bolt_pattern_transforms(&body, &ribbon, &spec_bracket, &flange, bounds);

        assert!(
            count_near(&bracketed) >= 2,
            "proactive bracketing must land ≥2 bolts within {near:.3} m of the \
             pour axis (one each side); got {}",
            count_near(&bracketed),
        );
        assert!(
            count_near(&bracketed) > count_near(&skipped),
            "bracket mode must add close bolts the reactive skip doesn't: \
             skip {} vs bracket {} near the pour",
            count_near(&skipped),
            count_near(&bracketed),
        );
    }

    #[test]
    fn bolt_pattern_kind_spec_accessor() {
        assert!(BoltPatternKind::None.spec().is_none());
        let s = BoltPatternSpec::iter1();
        let kind = BoltPatternKind::Auto(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn build_transforms_zero_count_emits_empty_vec() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec {
            count: 0,
            ..BoltPatternSpec::iter1()
        };
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert!(transforms.is_empty());
    }

    #[test]
    fn build_transforms_no_pour_gate_emits_count_subtract_cylinders() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "must emit one SubtractCylinder per bolt when no pour-gate collision; \
             got {}",
            transforms.len()
        );
        for t in &transforms {
            assert!(
                matches!(t, MatingTransform::SubtractCylinder { .. }),
                "all transforms must be SubtractCylinder; got {t:?}"
            );
        }
    }

    #[test]
    fn bolt_radius_matches_clearance_half_no_slop() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let expected_radius = spec.clearance_diameter_m / 2.0;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.radius_m - expected_radius).abs() < 1e-12,
                "bolt radius = clearance_diameter / 2 = {expected_radius:.5} m; \
                 got {:.5} m",
                params.radius_m
            );
        }
    }

    #[test]
    fn bolts_axis_aligned_with_ribbon_binormal() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let (_, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            let dot = params.parent.axis.into_inner().dot(&binormal).abs();
            assert!(
                (dot - 1.0).abs() < 1e-9,
                "bolt axis must align with ribbon binormal; got dot = {dot}"
            );
        }
    }

    #[test]
    fn bolts_centered_outboard_of_silhouette() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("expected SubtractCylinder, got {t:?}");
            };
            let center = params.parent.center_m;
            let body_dist = body.evaluate(&center);
            assert!(
                body_dist > 0.0,
                "bolt #{i} center must be OUTSIDE body in 3D; got body_dist = \
                 {body_dist:.5} m"
            );
            let target = spec.silhouette_outboard_offset_m;
            let tol = 2.0 * SILHOUETTE_GRID_STEP_M;
            assert!(
                (body_dist - target).abs() < tol,
                "bolt #{i} center distance from body must be ≈ {target:.4} m \
                 (silhouette_outboard_offset_m); got {body_dist:.5} m \
                 (tolerance {tol:.4} m)"
            );
        }
    }

    #[test]
    fn bolt_half_length_spans_flange_plus_slack() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let expected_half_length = flange.flange_thickness_m + BOLT_AXIAL_SLACK_M;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.parent.half_length_m - expected_half_length).abs() < 1e-12,
                "bolt half-length = flange_thickness + slack = \
                 {expected_half_length:.5} m; got {:.5} m",
                params.parent.half_length_m
            );
        }
    }

    /// Same regression discipline §M-S2 dowel-hole got post cold-read:
    /// bolt cylinder centers must land ON the binormal-aligned seam
    /// plane regardless of centerline tilt. Pre-fix the math put
    /// `center_m.y = seam_midpoint.y` (constant-Y plane), which for
    /// tilted centerlines placed the center off the actual seam plane
    /// along the binormal direction — making the bolt carve
    /// asymmetrically between the two cup-halves.
    #[test]
    fn bolt_centers_lie_on_binormal_aligned_seam_plane_under_tilted_centerline() {
        let centerline = vec![
            Point3::new(-0.050, -0.005, 0.0),
            Point3::new(0.050, 0.005, 0.0),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let layer_body =
            Solid::cylinder(0.010, 0.060).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms =
            build_bolt_pattern_transforms(&layer_body, &ribbon, &spec, &flange, bounds);
        assert_eq!(transforms.len(), spec.count as usize);

        let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            let dist_from_seam = (params.parent.center_m - seam_midpoint).dot(&binormal);
            assert!(
                dist_from_seam.abs() < 1e-9,
                "bolt #{i} center must lie ON the binormal-aligned seam plane; \
                 got binormal-distance = {dist_from_seam:.6} m (pre-fix this \
                 could be several mm for tilted centerlines, producing \
                 asymmetric hole carve across the two cup-halves)"
            );
        }
    }

    /// Direct unit test of `collides_with_pour_gate` with a synthetic
    /// pour-gate cylinder at a known position. Bolt sits 2 mm from
    /// the pour-gate cylinder's axis midpoint (well inside the
    /// `bolt_radius + pour_radius + clearance` threshold = 8.5 mm) —
    /// must register as colliding.
    #[test]
    fn collides_with_pour_gate_detects_close_bolt() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.045, // pour-gate iter-1 default
                },
                radius_m: 0.005, // pour-gate iter-1 default
                segments: 32,
            },
        };
        // Bolt 2 mm off the pour-leg axis, inside the cap extent.
        let bolt_center = Point3::new(0.010, 0.002, 0.0);
        let bolt_radius = 0.00275; // M5 / 2
        let clearance = 0.001;
        assert!(
            collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt 2 mm off pour-leg axis must register as colliding \
             (bolt_radius {bolt_radius} + pour_radius 0.005 + clearance \
             {clearance} = 0.00875 > 0.002)"
        );
    }

    /// Direct unit test of `collides_with_pour_gate`: bolt safely
    /// distant from the pour-leg must NOT register as colliding.
    /// Bolt center 20 mm off-axis is far above the 8.75 mm threshold.
    #[test]
    fn collides_with_pour_gate_ignores_distant_bolt() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.045,
                },
                radius_m: 0.005,
                segments: 32,
            },
        };
        let bolt_center = Point3::new(0.010, 0.020, 0.0);
        let bolt_radius = 0.00275;
        let clearance = 0.001;
        assert!(
            !collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt 20 mm off pour-leg axis must NOT register as colliding"
        );
    }

    /// Cold-read I1 regression: a bolt PAST the pour-gate cylinder's
    /// endcap, even when close to the infinite-axis line, must NOT
    /// trigger collision (the cylinder is FINITE). Pre-fix
    /// `point_to_line_distance` returned the perpendicular ~0.002
    /// (false collision); post-fix `point_to_cylinder_axis_distance`
    /// clamps to the endcap and returns sqrt(0.030² + 0.002²) ≈
    /// 0.030 m, well past the 0.00875 m threshold.
    #[test]
    fn collides_with_pour_gate_does_not_count_bolt_past_endcap() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.020, // short pour leg for the test
                },
                radius_m: 0.005,
                segments: 32,
            },
        };
        // Bolt at X = 0.050 → axis-projection = 0.050, well past the
        // +X cap at X = 0.020. Off-axis only 0.002 m — pre-fix
        // (infinite line) would have flagged this as colliding.
        let bolt_center = Point3::new(0.050, 0.002, 0.0);
        let bolt_radius = 0.00275;
        let clearance = 0.001;
        assert!(
            !collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt past pour-leg endcap must NOT register as colliding \
             (pre-I1-fix this would falsely fire because infinite-line \
             distance ignored the cap)"
        );
    }

    /// End-to-end skip-disabled test: with `skip_pour_gate_collision
    /// = false`, all bolts emitted regardless of pour-gate presence.
    /// Pairs with the direct unit tests above to lock the wiring.
    #[test]
    fn pour_gate_collision_skip_disabled_emits_all_bolts() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let spec = BoltPatternSpec {
            skip_pour_gate_collision: false,
            ..BoltPatternSpec::iter1()
        };
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "skip_pour_gate_collision=false must emit all {} bolts; got {}",
            spec.count,
            transforms.len()
        );
    }

    /// Bolt-dowel arc-length stagger at default counts: bolts at
    /// `(k + 0.5) / 8` and dowels at `(k + 0.5) / 4` interleave with
    /// minimum arc-fraction separation = 1/16. On a 100 mm centerline
    /// (200 mm typical perimeter for the test body, ~62.8 mm for
    /// R=10 mm cylinder) this still gives well above the radial-
    /// collision threshold. Locks the documented "auto-stagger at
    /// defaults" invariant — workshop default configs never collide.
    #[test]
    fn bolt_arc_fractions_interleave_with_dowel_arc_fractions_at_default_counts() {
        // Dowels emitted at arc fractions {1/8, 3/8, 5/8, 7/8}.
        // Bolts emitted at arc fractions {1/16, 3/16, 5/16, …, 15/16}.
        // Min separation = 1/16 = 0.0625 of perimeter.
        let dowel_count = crate::dowel_hole::DowelHoleSpec::iter1().count;
        let bolt_count = BoltPatternSpec::iter1().count;
        let dowel_fractions: Vec<f64> = (0..dowel_count)
            .map(|k| (f64::from(k) + 0.5) / f64::from(dowel_count))
            .collect();
        let bolt_fractions: Vec<f64> = (0..bolt_count)
            .map(|k| (f64::from(k) + 0.5) / f64::from(bolt_count))
            .collect();
        let mut min_sep = f64::INFINITY;
        for db in &dowel_fractions {
            for bb in &bolt_fractions {
                let sep = (db - bb).abs();
                if sep < min_sep {
                    min_sep = sep;
                }
            }
        }
        // 1/16 = 0.0625; allow FP slack.
        assert!(
            (min_sep - 0.0625).abs() < 1e-9,
            "bolt + dowel arc-fraction interleave must give 1/16 separation \
             at default counts; got {min_sep:.6}"
        );
    }

    #[test]
    fn point_to_cylinder_distance_zero_on_axis_inside_caps() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point ON the axis, INSIDE the cap span (|t| = 0.010 ≤ 0.020).
        let p = Point3::new(0.010, 0.0, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        assert!(
            d.abs() < 1e-12,
            "on-axis interior distance must be 0; got {d}"
        );
    }

    #[test]
    fn point_to_cylinder_distance_perpendicular_inside_caps() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point 3 mm off-axis, axis-projection inside caps.
        let p = Point3::new(0.005, 0.003, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        assert!(
            (d - 0.003).abs() < 1e-12,
            "perpendicular offset 3 mm inside caps; got {d}"
        );
    }

    /// Cold-read finding I1: a point BEYOND the cylinder's endcap
    /// must measure to the cap center, not to the infinite axis line.
    /// Pre-fix (infinite-line distance) returned 0.003 for this query;
    /// post-fix returns sqrt(0.010² + 0.003²) ≈ 0.0104.
    #[test]
    fn point_to_cylinder_distance_past_endcap_measures_to_cap_center() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point past the +X cap (axis-projection = 0.030 > 0.020),
        // 3 mm off-axis. Closest axis point is the +X cap center
        // (0.020, 0, 0). Distance = sqrt(0.010² + 0.003²) ≈ 0.0104 m.
        let p = Point3::new(0.030, 0.003, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        let expected = 0.010_f64.hypot(0.003_f64);
        assert!(
            (d - expected).abs() < 1e-12,
            "past-endcap distance must measure to cap center ({expected:.5}); \
             got {d:.5} (pre-fix infinite-line returned the perpendicular \
             ~0.003 — would falsely trigger pour-gate collision skips for \
             bolts well past the pour-gate leg's endpoint)",
        );
    }
}
