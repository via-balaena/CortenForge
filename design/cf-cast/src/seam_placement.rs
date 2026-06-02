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

use crate::flange::FlangeSpec;
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
    flange_spec: &FlangeSpec,
) -> Vec<LayerLoop> {
    // Generous window: the body loop sits well inside the MC bounds, padded by the
    // flange reach so the whole perimeter is captured at any seam tilt.
    let pad = flange_spec.flange_width_m + SILHOUETTE_GRID_STEP_M;
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
