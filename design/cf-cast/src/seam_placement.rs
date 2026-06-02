//! Shared seam-placement plumbing (§3.5–§3.8 of
//! `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`).
//!
//! The pure solver lives in [`crate::seam_solver`] (no [`Solid`]/[`Ribbon`]
//! dependency — it works on a [`SeamProfile`]). This module is the bridge from
//! the build geometry ([`Solid`] + [`Ribbon`] + [`Silhouette2d`]) into that
//! solver, and is shared by BOTH fastener planners:
//! [`crate::bolt_pattern::plan_smart_bolt_placements`] (S3) and
//! [`crate::dowel_hole::plan_smart_dowel_placements`] (S4). Keeping it neutral
//! (here, rather than in either feature module) avoids a bolt↔dowel module
//! dependency and keeps the two planners exact analogues.
//!
//! - `seam_silhouette` — the fitted-seam in-plane (or legacy Y-normal) body
//!   cross-section the [`SeamProfile`] loop is built from.
//! - `pour_exclusions` — the pour bores projected onto the seam plane as swept
//!   [`Exclusion::Channel`] keep-outs (§3.4).
//! - `smart_center_to_world` — a solver-resolved seam-plane centre lifted back
//!   to the world mating plane (shared by both `emit_smart_*`).
//! - `cross_layer_snap` — the §3.8 per-layer snap + all-layer-feasible filter
//!   (solve once on the outermost loop, share one angular pattern across the
//!   stack).

use cf_design::{Aabb, Solid};
use nalgebra::{Point3, Vector3};

use crate::flange::{FlangeKind, Tadpole};
use crate::mesh_csg::MatingTransform;
use crate::pour::build_pour_gate_transforms;
use crate::ribbon::Ribbon;
use crate::seam_profile::SeamProfile;
use crate::seam_solver::{Exclusion, Feasibility, Placement, snap_placement};
use crate::silhouette_2d::{Point2, SILHOUETTE_GRID_STEP_M, SeamPlaneBasis, Silhouette2d};

/// One layer's clean seam loop + its seam-plane keep-outs, or `None` when the
/// layer's silhouette is empty (no polyline → no placeable loop). Built once per
/// layer and consumed by [`cross_layer_snap`].
pub(crate) type LayerLoop = Option<(SeamProfile, Vec<Exclusion>)>;

/// Build the seam-plane silhouette for a layer body — the fitted-seam in-plane
/// path (item A) or the legacy Y-normal X-Z path, bit-identical to the
/// pre-extraction inline form. Shared by the legacy bolt builder
/// ([`crate::bolt_pattern::build_bolt_pattern_transforms`]) and both
/// seam-placement planners so every consumer samples the same cross-section.
#[must_use]
pub(crate) fn seam_silhouette(
    layer_body: &Solid,
    ribbon: &Ribbon,
    bounds: Aabb,
    pad: f64,
) -> Silhouette2d {
    let (seam_midpoint, _) = ribbon.seam_plane_reference();
    ribbon.seam_plane_basis().map_or_else(
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
            let expanded = Aabb::new(
                Point3::new(bounds.min.x - pad, bounds.min.y - pad, bounds.min.z - pad),
                Point3::new(bounds.max.x + pad, bounds.max.y + pad, bounds.max.z + pad),
            );
            let (u_min, u_max, v_min, v_max) = basis.inplane_bounds(expanded);
            Silhouette2d::from_body_in_plane(layer_body, basis, u_min, u_max, v_min, v_max)
        },
    )
}

/// Project the pour-gate bores onto the seam plane as swept [`Exclusion::Channel`]
/// keep-outs (§3.4 — the pour is modelled once, correctly: a capsule, not a point
/// disk). Each pour cylinder's axis segment `center ± half_length·axis` projects
/// to the in-plane segment; the capsule radius is the bore radius.
#[must_use]
pub(crate) fn pour_exclusions(
    pour_xforms: &[MatingTransform],
    basis: SeamPlaneBasis,
) -> Vec<Exclusion> {
    pour_xforms
        .iter()
        .filter_map(|t| match t {
            MatingTransform::SubtractCylinder { params } => {
                let c = params.parent.center_m;
                let ax = params.parent.axis.into_inner();
                let h = params.parent.half_length_m;
                Some(Exclusion::Channel {
                    a: basis.project(c - ax * h),
                    b: basis.project(c + ax * h),
                    half_width: params.radius_m,
                })
            }
            _ => None,
        })
        .collect()
}

/// Build each layer's clean seam loop + its **pour-only** seam-plane exclusions,
/// ONCE, to be shared by both fastener planners (S5d-(A),
/// `docs/CF_CAST_SEAM_PLACEMENT_RECON.md` §7.5).
///
/// Both `plan_smart_dowel_placements` and `plan_smart_bolt_placements` otherwise
/// rebuild the bit-identical per-layer silhouette (same body, same `bounds`, same
/// `pad = flange_width + SILHOUETTE_GRID_STEP_M`); the silhouette flood-fill is the
/// dominant solve cost (§MA-7), so building it twice doubles that cost. The pour
/// channels are common to both runs and included here; the **dowel-disk**
/// exclusions are layered on per-planner (the bolt run appends them — the
/// dowels-first contract, §3.6 — while the dowel run uses the pour-only base).
///
/// Returns a `Vec` parallel to `layer_bodies` (innermost-first); a layer whose
/// silhouette is empty becomes `None` (no placeable loop), exactly as the inline
/// per-planner build did.
#[must_use]
pub(crate) fn build_layer_loops(
    layer_bodies: &[&Solid],
    layer_bounds: &[Aabb],
    ribbon: &Ribbon,
    flange: &FlangeKind,
) -> Vec<LayerLoop> {
    // Generous window: the body loop sits well inside the MC bounds, padded by the
    // flange reach so the whole perimeter is captured at any seam tilt. Reach is
    // flange-kind-agnostic (`lateral_reach_m`); 0 when no flange (caller gates on a
    // flange being present, so this is the defensive fallback).
    let pad = flange.lateral_reach_m().unwrap_or(0.0) + SILHOUETTE_GRID_STEP_M;
    let pour_xforms = build_pour_gate_transforms(ribbon);
    layer_bodies
        .iter()
        .zip(layer_bounds)
        .map(|(body, &bounds)| {
            let silhouette = seam_silhouette(body, ribbon, bounds, pad);
            let profile = SeamProfile::from_silhouette(&silhouette)?;
            let excluded = pour_exclusions(&pour_xforms, silhouette.basis());
            Some((profile, excluded))
        })
        .collect()
}

/// Build the per-fastener tadpoles (§4.1) for the demand flange. Each fastener
/// `(center, boss_r)` maps to a [`Tadpole`] whose seal-ring `anchor` is the ring's
/// outer edge directly under the fastener — the loop point at the fastener's arc
/// (`nearest_arc`), offset outward along the loop normal by `land_outer_m`
/// (`= land_inner + land_width`). The spoke runs anchor → center; under the
/// pin-at-floor radial strategy (E1) the boss usually subsumes the anchor
/// (degenerate spoke, recon §4.1 N4) and the capsule SDF handles it.
#[must_use]
pub(crate) fn build_tadpoles(
    profile: &SeamProfile,
    fasteners: &[(Point2, f64)],
    land_outer_m: f64,
) -> Vec<Tadpole> {
    fasteners
        .iter()
        .map(|&(center, boss_r)| {
            let s = profile.nearest_arc(center);
            let p = profile.point_at(s);
            let n = profile.outward_normal_at(s);
            let anchor = Point2::new(p.x + n.x * land_outer_m, p.z + n.z * land_outer_m);
            Tadpole {
                anchor,
                center,
                boss_r,
            }
        })
        .collect()
}

/// Map a solver-resolved seam-plane centre to the world fastener centre: lift it
/// off the seam plane (fitted basis, or the legacy `(x, seam_y, z)` map) and
/// project onto the binormal-aligned seam plane through `seam_midpoint` — exactly
/// the projection the legacy `*_center_at` builders apply, so smart + legacy
/// fasteners share a plane. Shared by both `emit_smart_*`.
#[must_use]
pub(crate) fn smart_center_to_world(
    ribbon: &Ribbon,
    seam_midpoint: Point3<f64>,
    binormal: Vector3<f64>,
    c: Point2,
) -> Point3<f64> {
    let candidate = ribbon
        .seam_plane_basis()
        .map_or_else(|| Point3::new(c.x, seam_midpoint.y, c.z), |b| b.to_world(c));
    let signed = (candidate - seam_midpoint).dot(&binormal);
    candidate - signed * binormal
}

/// Cross-layer snap + validate (§3.8): map each `master` placement (solved on the
/// outermost loop) onto **every** layer's loop and keep it only if it is feasible
/// on all of them — so the stack shares one angular pattern (the workshop aligns
/// one jig).
///
/// Returns a `Vec` parallel to `layers` (innermost-first) of the per-layer
/// seam-plane centres; a master position infeasible on some layer is dropped from
/// the **whole** shared set, with a `warn` summarising the count. `label`
/// distinguishes the bolt vs dowel run in that message.
#[must_use]
pub(crate) fn cross_layer_snap(
    master: &[Placement],
    layers: &[LayerLoop],
    feas: &Feasibility,
    footprint_r: f64,
    label: &str,
) -> Vec<Vec<Point2>> {
    let n_layers = layers.len();
    let mut result = vec![Vec::new(); n_layers];
    let mut dropped = 0usize;
    for m in master {
        let mut snapped: Vec<Point2> = Vec::with_capacity(n_layers);
        let mut feasible_on_all = true;
        for slot in layers {
            let Some((profile, excluded)) = slot else {
                feasible_on_all = false;
                break;
            };
            let Some(resolved) = snap_placement(profile, feas, excluded, footprint_r, m.center)
            else {
                feasible_on_all = false;
                break;
            };
            snapped.push(resolved.center);
        }
        if feasible_on_all {
            for (out, c) in result.iter_mut().zip(snapped) {
                out.push(c);
            }
        } else {
            dropped += 1;
        }
    }
    if dropped > 0 {
        eprintln!(
            "[cf-cast] smart {label} placement: dropped {dropped} of {} candidate position(s) \
             that were infeasible on at least one layer (kept the shared, all-layer-feasible \
             set).",
            master.len(),
        );
    }
    result
}

#[cfg(test)]
mod tests {
    // Workspace lint policy allows unwrap/panic in tests; matches the sibling
    // cf-cast test modules.
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::seam_solver::{FastenerClass, place_fasteners};

    /// A clean circular [`SeamProfile`] of radius `r` in the Y-normal seam plane.
    fn circle_profile(r: f64) -> SeamProfile {
        let n: u32 = 360;
        let raw: Vec<Point2> = (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * f64::from(i) / f64::from(n);
                Point2::new(r * th.cos(), r * th.sin())
            })
            .collect();
        SeamProfile::from_polyline(&raw, SeamPlaneBasis::y_normal(0.0), 0.001, 0.002).unwrap()
    }

    /// The incremental band the cross-layer tests place against (washer footprint
    /// 5 mm, d floored at 11 mm — the bolt regime).
    fn band() -> Feasibility {
        Feasibility::band(0.002, 0.020, 0.011, 0.020)
    }

    /// A realistic master set: an even ring solved on a 10 mm-radius loop.
    fn master_ring() -> (SeamProfile, Vec<Placement>) {
        let outer = circle_profile(0.010);
        let class = FastenerClass {
            footprint_radius: 0.005,
            fill: Some(0.030),
            seeds: Vec::new(),
        };
        let master = place_fasteners(&outer, &band(), &[], &class);
        assert!(master.len() >= 2, "fixture must place a few fasteners");
        (outer, master)
    }

    /// §3.8 positive case: when every layer can host the pattern, all master
    /// positions are kept and every layer carries the SAME count.
    #[test]
    fn cross_layer_snap_keeps_positions_feasible_on_all_layers() {
        let (outer, master) = master_ring();
        let inner = circle_profile(0.010);
        let layers: Vec<LayerLoop> = vec![Some((inner, Vec::new())), Some((outer, Vec::new()))];
        let result = cross_layer_snap(&master, &layers, &band(), 0.005, "test");
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].len(), master.len(), "inner keeps every position");
        assert_eq!(result[1].len(), master.len(), "outer keeps every position");
    }

    /// §3.8 drop-from-WHOLE-set: a layer that cannot host the pattern (here an empty
    /// silhouette → `None` slot) drops EVERY master position from the WHOLE stack —
    /// including the outer layer where each position is feasible. This pins the
    /// load-bearing "kept only if feasible on every layer" branch (the C2 case): the
    /// equal-count tests above pass even if this branch were deleted, but this one
    /// would not (the outer result would be non-empty).
    #[test]
    fn cross_layer_snap_drops_whole_set_when_a_layer_cannot_host_it() {
        let (outer, master) = master_ring();
        // Inner layer unplaceable (empty silhouette → None), outer fully feasible.
        let layers: Vec<LayerLoop> = vec![None, Some((outer, Vec::new()))];
        let result = cross_layer_snap(&master, &layers, &band(), 0.005, "test");
        assert_eq!(result.len(), 2);
        assert!(
            result.iter().all(Vec::is_empty),
            "an unplaceable layer must drop the whole shared set from EVERY layer \
             (incl. the outer, where the positions are feasible); got {:?}",
            result.iter().map(Vec::len).collect::<Vec<_>>(),
        );
    }

    /// §4.1 tadpole anchors: each fastener's ring anchor is the seal-ring outer edge
    /// (`land_outer` signed-distance outboard) at the fastener's own azimuth, and
    /// `boss_r` passes through. Pins the production anchor computation the
    /// `DemandFlangeSdf` tests sidestep by hand-building `Tadpole`s.
    #[test]
    fn build_tadpoles_anchors_on_the_ring_outer_edge_at_each_azimuth() {
        let profile = circle_profile(0.010);
        let land_outer = 0.0065; // land_inner 0.5 + land_width 6 mm
        // Two fasteners at azimuth 0° and 90°, each at radius 21 mm (d ≈ 11 mm).
        let fasteners = vec![
            (Point2::new(0.021, 0.0), 0.007),
            (Point2::new(0.0, 0.021), 0.0056),
        ];
        let tads = build_tadpoles(&profile, &fasteners, land_outer);
        assert_eq!(tads.len(), 2);

        // Azimuth 0° → anchor at (10 + 6.5, 0) mm on +x.
        assert!(
            (tads[0].anchor.x - 0.0165).abs() < 1e-3,
            "anchor x: {}",
            tads[0].anchor.x
        );
        assert!(
            tads[0].anchor.z.abs() < 1e-3,
            "anchor z: {}",
            tads[0].anchor.z
        );
        assert!(
            (tads[0].boss_r - 0.007).abs() < 1e-12,
            "boss_r passes through"
        );
        assert_eq!(tads[0].center, Point2::new(0.021, 0.0));

        // Azimuth 90° → anchor at (0, 16.5) mm on +z.
        assert!(
            tads[1].anchor.x.abs() < 1e-3,
            "anchor x: {}",
            tads[1].anchor.x
        );
        assert!(
            (tads[1].anchor.z - 0.0165).abs() < 1e-3,
            "anchor z: {}",
            tads[1].anchor.z
        );
        assert!((tads[1].boss_r - 0.0056).abs() < 1e-12);
    }
}
