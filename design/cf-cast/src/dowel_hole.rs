//! Symmetric dowel-hole registration primitive (§M-S2 of
//! `docs/CF_CAST_UNIFIED_MATING_PLANE_RECON.md`).
//!
//! Both cup-halves get IDENTICAL dowel holes through their mating
//! faces, arc-length-equal-spaced around the body silhouette in the
//! flange band. Workshop user inserts printed-separately PLA dowels
//! into the holes at assembly time; the dowels register the two
//! halves laterally (preventing slip in the seam plane) while the
//! bolts (§B arc) clamp them axially.
//!
//! Replaces the pre-§M asymmetric pin/socket pair pattern (post-
//! 2026-05-24 salvage mesh-CSG `UnionTruncatedPyramid` +
//! `SubtractTruncatedPyramid` paired with side-specific pose). The
//! symmetric `SubtractCylinder`-only emission is cleaner per
//! [[project-cf-cast-unified-mating-plane-recon]]'s §M-10 lesson
//! about symmetric registration being preferable to asymmetric
//! pin/socket primitives.
//!
//! # Geometry
//!
//! Each dowel hole is a `MatingTransform::SubtractCylinder` post-MC
//! mesh-CSG primitive with:
//! - **axis** = ribbon binormal (perpendicular to mating plane —
//!   parallel to print Z when the cup piece is mated-face-down on the
//!   build plate, so the hole prints as a vertical cylinder requiring
//!   no support).
//! - **center** = silhouette point at arc fraction
//!   `(k + 0.5) / count` (k = 0..count) offset OUTWARD from the
//!   silhouette by `silhouette_outboard_offset_m` in the seam plane.
//!   The +0.5 offset avoids placing dowels at the silhouette's
//!   polyline-assembly join point (which may be on an MC corner
//!   ambiguity).
//! - **`half_length`** = `depth_m + clearance_slack_m`. The full
//!   cylinder spans `2 × half_length` straddling the seam plane,
//!   carving `depth_m` into EACH cup half.
//! - **radius** = `dowel_diameter_m / 2 + clearance_m`. PLA-on-PLA
//!   slide-fit clearance.
//!
//! # Side-agnostic emission
//!
//! The same transform list is applied to BOTH Negative + Positive
//! cup pieces (mesh-CSG subtract is invariant to which half it's
//! applied to; cylinder portion outside a given half is a no-op).
//! Both halves get matching hole positions mirrored across the seam
//! plane, so a single printed dowel inserts through the corresponding
//! holes on the two halves to register them.

// The smart-placement planner does 2-D seam-plane geometry (PCA / covariance):
// f64↔index casts (station counts) are bounded + non-negative, and short
// coordinate names (x, z, a, n) are idiomatic for the point math here — same
// rationale + allow set as `seam_profile` / `seam_solver`. (The legacy emission
// below uses `f64::from` and needs none of these; they cover the planner.)
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    clippy::similar_names
)]

use cf_design::{Aabb, Solid};
use nalgebra::{Point3, Unit};

use crate::flange::FlangeSpec;
use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::pour::build_pour_gate_transforms;
use crate::ribbon::Ribbon;
use crate::seam_placement::{
    cross_layer_snap, pour_exclusions, seam_silhouette, smart_center_to_world,
};
use crate::seam_profile::SeamProfile;
use crate::seam_solver::{FastenerClass, Feasibility, Seed, SeedKind, place_fasteners};
use crate::silhouette_2d::{Point2, SILHOUETTE_GRID_STEP_M, Silhouette2d};

/// Default dowel diameter (3 mm) — matches standard 3 mm wood-dowel
/// stock + small PLA-printed dowel sizes (per §M-S2 recon).
const DEFAULT_DIAMETER_M: f64 = 0.003;

/// Default radial clearance between dowel and hole wall (0.1 mm) —
/// PLA-on-PLA slide fit with thermal-tolerance margin.
const DEFAULT_CLEARANCE_M: f64 = 0.0001;

/// Default hole depth PER HALF (5 mm). Printed dowel length should
/// be `2 × DEFAULT_DEPTH_M - 2 × axial_slack` ≈ 9 mm (0.5 mm slack
/// each side for assembly play).
const DEFAULT_DEPTH_M: f64 = 0.005;

/// Default hole count around the silhouette (4 — one in each
/// quadrant for a typical sock-shaped body). Workshop user picks per
/// §M recon OQ3.
const DEFAULT_COUNT: u32 = 4;

/// Default radial offset from the body silhouette curve to the dowel
/// centerline (10 mm). Workshop user feedback 2026-05-27 cf-view
/// smoke: the dowel hole at the pre-bump 8 mm offset left only 3 mm
/// of FLAT flange between the cup-wall outer step (at
/// `body_dist = wall_thickness_m = 5 mm`) and the dowel hole — too
/// narrow for a 5 mm-wide vice jaw to grip during press-fit dowel
/// insertion. At 10 mm offset the inboard flat is 5 mm (matches
/// vice jaw width) and dowel outboard wall is `20 - 10 - 1.6 = 8.4
/// mm` (still 5.25× hole-Ø safe margin in the post-§B-washer-
/// clearance 20 mm flange band). Dowel-bolt radial separation
/// drops 5 → 3 mm but the arc-length stagger (1/16 perimeter ≈
/// 12.5 mm at iter-1 sock) keeps the 3D distance between dowel +
/// bolt cylinder centers well above the 4.35 mm cylinder-collision
/// threshold.
const DEFAULT_OUTBOARD_OFFSET_M: f64 = 0.010;

/// Polygonal facets around the dowel cylinder. 32 segments give a
/// ~0.2 mm chord error at 3 mm diameter — workshop-imperceptible at
/// FDM print resolution. Determinism contract per
/// [`CylinderParams::segments`].
const DEFAULT_SEGMENTS: u32 = 32;

/// Extra axial slack on each side of the hole's nominal depth.
///
/// Ensures the cylinder's CYLINDRICAL flat caps are well INSIDE the
/// cup-piece material (avoids mesh-CSG operating on coincident
/// endcap planes). 0.5 mm is well below FDM resolution + safely past
/// any mating-face MC ambiguity. Public so consumers (e.g.
/// `crate::dowel`, `crate::procedure`) can compute total cavity
/// depth + dowel tip-slack accurately.
pub const HOLE_AXIAL_SLACK_M: f64 = 0.0005;

/// PLA wall a dowel hole keeps to its surroundings in the seam-placement solver
/// (§3.6). The dowel *footprint* radius is the hole radius plus this, so (a) the
/// hole sits inside the flange band with PLA around it and (b) a bolt washer
/// excluded from the dowel keeps this much wall to the hole. 2 mm.
pub(crate) const SMART_DOWEL_WALL_M: f64 = 0.002;

/// FDM-tolerance margin the dowel footprint keeps beyond the cup-wall outer step
/// (1 mm), mirroring the bolt washer's `WASHER_CUP_WALL_MARGIN_M`: the inboard
/// radial floor is `wall_thickness + footprint + this`.
const DOWEL_CUP_WALL_MARGIN_M: f64 = 0.001;

/// The dowel *footprint* radius (hole + wall) for the seam-placement solver
/// (§3.3): the dowel hole radius (`diameter/2 + clearance`) plus
/// [`SMART_DOWEL_WALL_M`]. Used both as the dowel run's own feasibility footprint
/// AND (in [`crate::bolt_pattern`]) as the radius of the dowel keep-out disk the
/// bolt washer must clear, so the two patterns agree on the wall between them.
#[must_use]
pub(crate) fn smart_dowel_footprint(spec: &DowelHoleSpec) -> f64 {
    spec.diameter_m / 2.0 + spec.clearance_m + SMART_DOWEL_WALL_M
}

/// Dowel-hole geometry parameter envelope per §M-S2 recon.
///
/// Workshop user picks via cf-cast-cli `[dowel_hole]` TOML block
/// (§M-S3 territory); iter-1 starts at the defaults below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DowelHoleSpec {
    /// Dowel diameter (meters). Default 3 mm.
    pub diameter_m: f64,
    /// Radial clearance between dowel and hole wall (meters).
    /// Default 0.1 mm.
    pub clearance_m: f64,
    /// Hole depth PER HALF (meters). The cylinder spans
    /// `2 × depth_m` straddling the seam plane. Default 5 mm.
    pub depth_m: f64,
    /// Number of dowels arc-length-equal-spaced around the
    /// silhouette. Default 4.
    pub count: u32,
    /// Radial offset from the body silhouette curve to the dowel
    /// centerline (meters). Default 10 mm (bumped 8 → 10 mm in iter-1
    /// for vice-jaw flange clearance). Must satisfy the §M-5-b
    /// cross-field invariants in the recon (inboard + outboard wall
    /// thicknesses ≥ FDM floor).
    pub silhouette_outboard_offset_m: f64,
}

impl DowelHoleSpec {
    /// Workshop iter-1 starting defaults pinned at §M-S2 recon.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            diameter_m: DEFAULT_DIAMETER_M,
            clearance_m: DEFAULT_CLEARANCE_M,
            depth_m: DEFAULT_DEPTH_M,
            count: DEFAULT_COUNT,
            silhouette_outboard_offset_m: DEFAULT_OUTBOARD_OFFSET_M,
        }
    }
}

impl Default for DowelHoleSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Dowel-hole kind on the [`Ribbon`].
///
/// Matches the existing `RegistrationKind` / `PourGateKind` /
/// `FlangeKind` / `PlugPinKind` / `GasketKind` patterns — bridges
/// the cf-cast-cli `[dowel_hole]` TOML block (§M-S3 territory) to
/// the per-piece dowel-hole emission in
/// [`crate::compose_piece_solid`].
///
/// Default [`Self::None`] (no dowel holes — cup pieces register via
/// whatever other mechanism the workshop uses). Set
/// `DowelHoleKind::Auto(DowelHoleSpec::iter1())` via
/// [`Ribbon::with_dowel_hole`] to enable the symmetric dowel-hole
/// pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DowelHoleKind {
    /// No dowel-hole emission.
    None,
    /// Arc-length-equal-spaced dowel holes around the silhouette,
    /// parameterized by the inner [`DowelHoleSpec`].
    Auto(DowelHoleSpec),
}

impl DowelHoleKind {
    /// Returns the inner [`DowelHoleSpec`] for [`DowelHoleKind::Auto`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&DowelHoleSpec> {
        match self {
            Self::None => None,
            Self::Auto(spec) => Some(spec),
        }
    }
}

/// Build the side-agnostic dowel-hole [`MatingTransform`] vec for
/// the given `ribbon.dowel_hole` spec + layer body silhouette.
///
/// Returns an empty Vec if the silhouette has no polylines (e.g.,
/// body extends past silhouette region) — caller treats this as
/// "no dowel holes for this piece".
///
/// `bounds` is the cup-piece's MC bounds (used to size the silhouette
/// sampling region — same as the flange's silhouette-construction
/// bounds for consistency).
///
/// When `smart_dowels` is `Some`, the geometry-blind uniform loop is bypassed
/// entirely: the slice carries the per-layer seam-plane centers the
/// seam-placement solver already resolved + cross-layer-validated (see
/// [`plan_smart_dowel_placements`]), and each maps straight to a
/// [`MatingTransform::SubtractCylinder`] (emission unchanged — only the
/// *positions* come from the solver). `None` keeps the legacy path bit-identical
/// (S4 of `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`).
#[must_use]
pub fn build_dowel_hole_transforms(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &DowelHoleSpec,
    bounds: cf_design::Aabb,
    smart_dowels: Option<&[Point2]>,
) -> Vec<MatingTransform> {
    if let Some(centers) = smart_dowels {
        return emit_smart_dowels(centers, ribbon, spec);
    }
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();
    let seam_plane_y = seam_midpoint.y;

    // Silhouette sampling region: same expansion as the flange uses
    // so the silhouette curve is fully captured for arc-length walks.
    // Pad by silhouette_outboard_offset + dowel radius + clearance
    // so the dowel centerline + hole radius stay inside the sampled
    // grid (otherwise a dowel near the silhouette extreme might fall
    // off the grid).
    let pad = spec.silhouette_outboard_offset_m
        + spec.diameter_m / 2.0
        + spec.clearance_m
        + SILHOUETTE_GRID_STEP_M;
    // Fitted seam (item A) → sample the silhouette IN the seam plane; else the
    // legacy X-Z constant-Y silhouette (bit-identical to the pre-generalization
    // path; `to_world`/`dir_to_world` below reduce to the old (X, Z) maps).
    let fitted = ribbon.seam_plane_basis();
    let silhouette = fitted.map_or_else(
        || {
            Silhouette2d::from_body_at_y(
                layer_body,
                seam_plane_y,
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

    let half_length_m = spec.depth_m + HOLE_AXIAL_SLACK_M;
    let radius_m = spec.diameter_m / 2.0 + spec.clearance_m;

    let mut transforms = Vec::with_capacity(spec.count as usize);
    if spec.count == 0 {
        return transforms;
    }
    let axis = Unit::new_normalize(binormal);
    for k in 0..spec.count {
        // (k + 0.5) / count: midpoint of the k-th arc segment.
        // Avoids placing a dowel at the silhouette's polyline-
        // assembly join point (which may be on an MC corner with
        // ambiguous tangent).
        let t = (f64::from(k) + 0.5) / f64::from(spec.count);
        let Some(p_silh) = silhouette.point_at_arc_fraction(t) else {
            // No polylines (silhouette empty) → no dowels.
            return Vec::new();
        };
        let Some(n_out) = silhouette.outward_normal_at_arc_fraction(t) else {
            continue;
        };
        // Offset outward from the silhouette in (X, Z) by
        // silhouette_outboard_offset_m, then project the result onto
        // the binormal-aligned seam plane through seam_midpoint so
        // the cylinder center sits EXACTLY on the mating plane.
        //
        // Cold-read finding 2026-05-27: pre-projection version put
        // `center_m.y = seam_midpoint.y` (constant-Y plane), which
        // for the iter-1 ~3.4° tilted centerline placed the cylinder
        // center up to a few mm off the binormal-aligned seam plane —
        // making the dowel carve asymmetrically (one half ~9 mm
        // deep, the other ~2 mm). Now: project onto the binormal
        // plane so both halves get equal carve depth. Per
        // [[project-cf-cast-unified-mating-plane-recon]] §M-S1.5
        // (binormal-aligned vertical) extended to dowel positioning.
        let candidate = if fitted.is_some() {
            // In-plane: map the silhouette point + outward offset through the
            // fitted basis. The point is already on the seam plane.
            silhouette.to_world(p_silh)
                + silhouette.dir_to_world(n_out) * spec.silhouette_outboard_offset_m
        } else {
            Point3::new(
                spec.silhouette_outboard_offset_m.mul_add(n_out.x, p_silh.x),
                seam_midpoint.y,
                spec.silhouette_outboard_offset_m.mul_add(n_out.z, p_silh.z),
            )
        };
        let signed_dist_from_seam = (candidate - seam_midpoint).dot(&binormal);
        let center_m = candidate - signed_dist_from_seam * binormal;
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

// === Smart placement (S4, `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`) ===

/// The flange-band feasibility regime for a dowel footprint (§3.3): the
/// `footprint` disk (hole + wall) must lie in the flange band, the radial offset
/// floored by the inboard footprint-vs-cup-wall-step clearance (§3.2) — the
/// computed analogue of the legacy hand-set 10 mm dowel offset.
fn dowel_feasibility(
    flange_spec: &FlangeSpec,
    wall_thickness_m: f64,
    footprint: f64,
) -> Feasibility {
    let d_floor = wall_thickness_m + footprint + DOWEL_CUP_WALL_MARGIN_M;
    Feasibility::band(
        flange_spec.flange_inner_offset_m,
        flange_spec.flange_width_m,
        d_floor,
        flange_spec.flange_width_m,
    )
}

/// Two registration-extreme seeds at maximum moment arm (§3.4): the loop's
/// principal axis (PCA over the uniform stations) and the two arc lengths whose
/// projection onto it is min / max — the long-axis ends, the farthest-apart pair
/// of dowels for clamshell registration. Deterministic: the principal axis falls
/// back to a coordinate axis when the loop is near-isotropic (PCA degeneracy,
/// Risk #2), and ties on the extreme projection break to the lowest arc length.
/// Returns an empty `Vec` if the loop has too few stations to seed from.
fn long_axis_extreme_seeds(profile: &SeamProfile) -> Vec<Seed> {
    let n = profile.station_count();
    if n < 3 {
        return Vec::new();
    }
    let perim = profile.perimeter();
    let arc_of = |k: usize| perim * k as f64 / n as f64;
    let pts: Vec<Point2> = (0..n).map(|k| profile.point_at(arc_of(k))).collect();

    // Centroid + 2×2 covariance of the boundary stations.
    let (mut cx, mut cz) = (0.0, 0.0);
    for p in &pts {
        cx += p.x;
        cz += p.z;
    }
    cx /= n as f64;
    cz /= n as f64;
    let (mut sxx, mut sxz, mut szz) = (0.0, 0.0, 0.0);
    for p in &pts {
        let dx = p.x - cx;
        let dz = p.z - cz;
        sxx += dx * dx;
        sxz += dx * dz;
        szz += dz * dz;
    }

    // Principal eigenvector of [[sxx, sxz], [sxz, szz]] (largest eigenvalue).
    // λ_max = tr/2 + √((tr/2)² − det); eigenvector ∝ (λ_max − szz, sxz), which is
    // non-degenerate whenever sxz ≠ 0 or sxx > szz. The remaining near-isotropic
    // case (sxz ≈ 0, sxx ≤ szz) falls back to a coordinate axis — deterministic.
    let tr = sxx + szz;
    let sxz2 = sxz * sxz;
    let det = sxx * szz - sxz2;
    let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
    let lambda = tr / 2.0 + disc;
    let (mut ax, mut az) = if sxz.abs() > 1e-12 {
        (lambda - szz, sxz)
    } else if sxx >= szz {
        (1.0, 0.0)
    } else {
        (0.0, 1.0)
    };
    let an = ax.hypot(az);
    if an > 0.0 {
        ax /= an;
        az /= an;
    } else {
        ax = 1.0;
        az = 0.0;
    }

    // Arc lengths of the min / max projection onto the principal axis. Strict
    // comparisons keep the first (lowest-arc) station on a tie → deterministic.
    let (mut min_k, mut max_k) = (0usize, 0usize);
    let (mut min_proj, mut max_proj) = (f64::INFINITY, f64::NEG_INFINITY);
    for (k, p) in pts.iter().enumerate() {
        let proj = (p.x - cx) * ax + (p.z - cz) * az;
        if proj < min_proj {
            min_proj = proj;
            min_k = k;
        }
        if proj > max_proj {
            max_proj = proj;
            max_k = k;
        }
    }
    vec![
        Seed {
            arc: arc_of(min_k),
            kind: SeedKind::Anchor,
        },
        Seed {
            arc: arc_of(max_k),
            kind: SeedKind::Anchor,
        },
    ]
}

/// Emit one dowel-hole [`MatingTransform::SubtractCylinder`] per solver-placed
/// center (S4). The positions come from the solver; the cylinder (axis,
/// half-length, radius, segments) is identical to the legacy emission.
fn emit_smart_dowels(
    centers: &[Point2],
    ribbon: &Ribbon,
    spec: &DowelHoleSpec,
) -> Vec<MatingTransform> {
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();
    let axis = Unit::new_normalize(binormal);
    let half_length_m = spec.depth_m + HOLE_AXIAL_SLACK_M;
    let radius_m = spec.diameter_m / 2.0 + spec.clearance_m;
    centers
        .iter()
        .map(|&c| {
            let center_m = smart_center_to_world(ribbon, seam_midpoint, binormal, c);
            MatingTransform::SubtractCylinder {
                params: CylinderParams {
                    parent: CylinderParent {
                        center_m,
                        axis,
                        half_length_m,
                    },
                    radius_m,
                    segments: DEFAULT_SEGMENTS,
                },
            }
        })
        .collect()
}

/// Per-layer seam-plane dowel centers from the seam-placement solver (§3.4/§3.8).
///
/// Dowels run **first** (registration is primary; the bolt run then excludes
/// these footprints — §3.6). The solver runs **once** on the outermost layer's
/// loop with `fill = None` (seeds only) and the two long-axis registration
/// extremes ([`long_axis_extreme_seeds`]) as `Anchor` seeds, the pour bore
/// excluded as a swept channel so a dowel seeded at the apex steps clear of it.
/// Each resolved center is then snapped onto **every** layer's loop
/// ([`crate::seam_placement::cross_layer_snap`], §3.8) and kept only if feasible
/// on all, so the stack shares one dowel pattern. Returns a `Vec` parallel to
/// `layer_bodies` (innermost-first) of the per-layer centers.
///
/// `layer_bounds` are the per-layer MC bounds (parallel to `layer_bodies`) — the
/// same bounds [`build_dowel_hole_transforms`] receives, so the silhouettes
/// match.
#[must_use]
pub fn plan_smart_dowel_placements(
    layer_bodies: &[&Solid],
    layer_bounds: &[Aabb],
    ribbon: &Ribbon,
    dowel_spec: &DowelHoleSpec,
    flange_spec: &FlangeSpec,
    wall_thickness_m: f64,
) -> Vec<Vec<Point2>> {
    let n_layers = layer_bodies.len();
    let result = vec![Vec::new(); n_layers];
    if n_layers == 0 || layer_bounds.len() != n_layers {
        return result;
    }

    let footprint = smart_dowel_footprint(dowel_spec);
    let feas = dowel_feasibility(flange_spec, wall_thickness_m, footprint);
    // Generous window: the body loop sits well inside the MC bounds, padded by
    // the flange reach so the whole perimeter is captured at any seam tilt
    // (matches `plan_smart_bolt_placements`).
    let pad = flange_spec.flange_width_m + SILHOUETTE_GRID_STEP_M;
    let pour_xforms = build_pour_gate_transforms(ribbon);

    // Per-layer clean loop + seam-plane exclusions (`None` = silhouette empty).
    // Dowels run first, so only the pour is excluded (no prior fasteners).
    let layers: Vec<crate::seam_placement::LayerLoop> = layer_bodies
        .iter()
        .zip(layer_bounds)
        .map(|(body, &bounds)| {
            let silhouette = seam_silhouette(body, ribbon, bounds, pad);
            let profile = SeamProfile::from_silhouette(&silhouette)?;
            let excluded = pour_exclusions(&pour_xforms, silhouette.basis());
            Some((profile, excluded))
        })
        .collect();

    let Some(Some((outer_profile, outer_excluded))) = layers.last() else {
        eprintln!(
            "[cf-cast] WARNING: smart dowel placement — outermost layer silhouette is empty; \
             no dowels placed."
        );
        return result;
    };

    let seeds = long_axis_extreme_seeds(outer_profile);
    if seeds.len() < 2 {
        eprintln!(
            "[cf-cast] WARNING: smart dowel placement — could not derive the long-axis \
             registration extremes (degenerate loop); no dowels placed."
        );
        return result;
    }
    let class = FastenerClass {
        footprint_radius: footprint,
        fill: None,
        seeds,
    };
    let master = place_fasteners(outer_profile, &feas, outer_excluded, &class);

    // Cross-layer snap + validate: keep a position only if feasible on EVERY
    // layer, so the stack shares one dowel pattern (§3.8).
    cross_layer_snap(&master, &layers, &feas, footprint, "dowel")
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
    fn dowel_hole_spec_iter1_defaults() {
        let s = DowelHoleSpec::iter1();
        assert_eq!(s.diameter_m, 0.003);
        assert_eq!(s.clearance_m, 0.0001);
        assert_eq!(s.depth_m, 0.005);
        assert_eq!(s.count, 4);
        assert_eq!(s.silhouette_outboard_offset_m, 0.010);
    }

    #[test]
    fn dowel_hole_kind_spec_accessor() {
        assert!(DowelHoleKind::None.spec().is_none());
        let s = DowelHoleSpec::iter1();
        let kind = DowelHoleKind::Auto(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn build_transforms_emits_count_subtract_cylinders() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds, None);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "must emit one SubtractCylinder per dowel; got {}",
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
    fn build_transforms_zero_count_emits_empty_vec() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec {
            count: 0,
            ..DowelHoleSpec::iter1()
        };
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds, None);
        assert!(transforms.is_empty());
    }

    #[test]
    fn dowel_holes_centered_outboard_of_silhouette() {
        // Each dowel center must be OUTSIDE the body silhouette by
        // approximately silhouette_outboard_offset_m. Probe each
        // cylinder's center in the body SDF — should be at distance
        // ≈ offset (within the silhouette grid step).
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds, None);
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("expected SubtractCylinder, got {t:?}");
            };
            let center = params.parent.center_m;
            // 3D body distance at the cylinder center.
            let body_dist = body.evaluate(&center);
            assert!(
                body_dist > 0.0,
                "dowel #{i} center {center:?} must be OUTSIDE body in 3D; \
                 got body_dist = {body_dist:.5} m"
            );
            // Tolerance: silhouette_outboard_offset ± grid_step. (3D
            // body distance can be ≤ 2D silhouette distance for body
            // with Y extent; for our flat cylinder cross-section at
            // Y=0 the two metrics agree.)
            let target = spec.silhouette_outboard_offset_m;
            let tol = 2.0 * SILHOUETTE_GRID_STEP_M;
            assert!(
                (body_dist - target).abs() < tol,
                "dowel #{i} center distance from body must be ≈ {target:.4} m \
                 (silhouette_outboard_offset_m); got {body_dist:.5} m \
                 (tolerance {tol:.4} m)"
            );
        }
    }

    #[test]
    fn dowel_holes_axis_aligned_with_ribbon_binormal() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds, None);
        let (_, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            // Dot product of dowel axis with binormal should be ±1.
            let dot = params.parent.axis.into_inner().dot(&binormal).abs();
            assert!(
                (dot - 1.0).abs() < 1e-9,
                "dowel axis must align with ribbon binormal; got dot = {dot}"
            );
        }
    }

    /// Cold-read regression 2026-05-27: dowel cylinder centers must
    /// land ON the binormal-aligned seam plane regardless of
    /// centerline tilt. Pre-fix the code put `center_m.y =
    /// seam_midpoint.y` (constant-Y plane), which for tilted
    /// centerlines placed the center off the actual seam plane along
    /// the binormal direction — making the dowel carve asymmetrically
    /// between the two cup-halves.
    ///
    /// Fixture: tilted centerline (~5.7° XY-plane tilt, similar to
    /// the §M-S1.5 flange-thickness test). Verify each dowel center
    /// satisfies `(center - seam_midpoint).dot(&binormal) ≈ 0`.
    #[test]
    fn dowel_hole_centers_lie_on_binormal_aligned_seam_plane_under_tilted_centerline() {
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
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&layer_body, &ribbon, &spec, bounds, None);
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
                "dowel #{i} center must lie ON the binormal-aligned seam \
                 plane; got binormal-distance = {dist_from_seam:.6} m \
                 (pre-fix this could be several mm for tilted centerlines, \
                 producing asymmetric hole carve across the two cup-halves)"
            );
        }
    }

    #[test]
    fn dowel_hole_radius_includes_clearance() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds, None);
        let expected_radius = spec.diameter_m / 2.0 + spec.clearance_m;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.radius_m - expected_radius).abs() < 1e-12,
                "dowel hole radius = diameter/2 + clearance = {expected_radius:.5} m; \
                 got {:.5} m",
                params.radius_m
            );
        }
    }

    // === Smart placement (S4) ===

    use crate::flange::{FlangeKind, FlangeSpec};
    use crate::silhouette_2d::{Point2, SeamPlaneBasis};

    /// PCA registration extremes (§3.4): on an ellipse (long axis +x) the two
    /// dowel seeds must land at the long-axis ENDS — `x ≈ ±a`, `z ≈ 0` — on
    /// opposite sides of the centroid (the maximum moment arm).
    #[test]
    fn long_axis_extreme_seeds_land_at_the_ellipse_long_axis_ends() {
        let (a, b) = (0.045, 0.025);
        let n: usize = 400;
        let raw: Vec<Point2> = (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * i as f64 / n as f64;
                Point2::new(a * th.cos(), b * th.sin())
            })
            .collect();
        let prof =
            SeamProfile::from_polyline(&raw, SeamPlaneBasis::y_normal(0.0), 0.001, 0.002).unwrap();
        let seeds = long_axis_extreme_seeds(&prof);
        assert_eq!(seeds.len(), 2, "two registration extremes");
        for s in &seeds {
            let p = prof.point_at(s.arc);
            assert!(
                p.x.abs() > 0.9 * a,
                "seed not at the long-axis end: x={} (a={a})",
                p.x
            );
            assert!(p.z.abs() < 0.15 * b, "seed off the long axis: z={}", p.z);
        }
        let (x0, x1) = (prof.point_at(seeds[0].arc).x, prof.point_at(seeds[1].arc).x);
        assert!(
            x0 * x1 < 0.0,
            "the two extremes must straddle the centroid: x0={x0}, x1={x1}"
        );
    }

    /// PCA degeneracy (Risk #2): a circle is isotropic, so the principal axis is
    /// the deterministic coordinate-axis fallback — the seeds must still be two
    /// distinct, antipodal points (no `NaN`, no collapse to one).
    #[test]
    fn long_axis_seeds_are_deterministic_on_an_isotropic_loop() {
        let raw: Vec<Point2> = (0..256usize)
            .map(|i| {
                let th = std::f64::consts::TAU * i as f64 / 256.0;
                Point2::new(0.030 * th.cos(), 0.030 * th.sin())
            })
            .collect();
        let prof =
            SeamProfile::from_polyline(&raw, SeamPlaneBasis::y_normal(0.0), 0.001, 0.002).unwrap();
        let seeds = long_axis_extreme_seeds(&prof);
        assert_eq!(seeds.len(), 2);
        let gap = (seeds[1].arc - seeds[0].arc)
            .rem_euclid(prof.perimeter())
            .min((seeds[0].arc - seeds[1].arc).rem_euclid(prof.perimeter()));
        assert!(
            gap > 0.4 * prof.perimeter(),
            "isotropic-loop seeds must be ~antipodal; gap={gap}"
        );
    }

    /// Per-layer (§3.8): solving dowels on the outer layer and snapping inward
    /// yields the SAME count on every layer — the shared registration pattern —
    /// and `fill = None` keeps it to the two seeded extremes.
    #[test]
    fn smart_dowels_share_one_pattern_across_layers() {
        let make = |r: f64| {
            Solid::cylinder(r, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ))
        };
        let inner = make(0.010);
        let outer = make(0.012);
        let (_b, bounds, base) = cylinder_fixture();
        let ribbon = base
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_dowel_hole(DowelHoleKind::Auto(DowelHoleSpec::iter1()))
            .with_smart_placement(true);
        let dowel = DowelHoleSpec::iter1();
        let flange = FlangeSpec::iter1();

        let plan = plan_smart_dowel_placements(
            &[&inner, &outer],
            &[bounds, bounds],
            &ribbon,
            &dowel,
            &flange,
            0.005,
        );
        assert_eq!(plan.len(), 2);
        assert_eq!(
            plan[0].len(),
            plan[1].len(),
            "shared pattern: every layer carries the same dowel count"
        );
        assert_eq!(
            plan[0].len(),
            2,
            "fill=None → the two registration extremes"
        );
    }

    /// `smart_dowels = Some` bypasses the uniform loop and emits one
    /// `SubtractCylinder` per planned center, with the legacy hole radius +
    /// half-length (only the positions come from the solver).
    #[test]
    fn smart_dowel_emission_matches_legacy_cylinder_geometry() {
        let (body, bounds, base) = cylinder_fixture();
        let ribbon = base
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_dowel_hole(DowelHoleKind::Auto(DowelHoleSpec::iter1()))
            .with_smart_placement(true);
        let dowel = DowelHoleSpec::iter1();
        let flange = FlangeSpec::iter1();
        let plan =
            plan_smart_dowel_placements(&[&body], &[bounds], &ribbon, &dowel, &flange, 0.005);
        let xforms = build_dowel_hole_transforms(&body, &ribbon, &dowel, bounds, Some(&plan[0]));
        assert_eq!(xforms.len(), plan[0].len());
        let expected_radius = dowel.diameter_m / 2.0 + dowel.clearance_m;
        let expected_half = dowel.depth_m + HOLE_AXIAL_SLACK_M;
        for t in &xforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("smart dowel emission must be SubtractCylinder");
            };
            assert!((params.radius_m - expected_radius).abs() < 1e-12);
            assert!((params.parent.half_length_m - expected_half).abs() < 1e-12);
            // each dowel sits OUTSIDE the body in 3D (in the flange band).
            assert!(body.evaluate(&params.parent.center_m) > 0.0);
        }
    }
}
