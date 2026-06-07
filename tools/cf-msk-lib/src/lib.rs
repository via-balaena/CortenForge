//! cf-msk-lib — the library/parameter spine of the musculoskeletal builder.
//!
//! The parametric-builder-first reframe (`docs/msk_builder/01_vision/vision.md`)
//! turns the program inside out: a body is *generated* from a validated template
//! and a small **parameter** vector, and a scan — when present — is just one
//! source of those parameters, never the entry point. This crate is that spine:
//!
//! ```text
//!   ParamSource ──► BodyParams ──► realize(template, params) ──► emit ──► MJCF
//!    (pluggable)                      (pure morph)            (cf-osim today)
//! ```
//!
//! * [`BodyParams`] — the parameters that morph a template into a body. v1 carries
//!   per-segment **scale** factors (the slice the canonical/uniform path needs and
//!   that the morph spike validated); scan-facing length/girth/center params and
//!   the `Measurable` gate arrive with `ScanSource` (recon S3).
//! * [`realize`] — a pure function that applies the morph to a template, returning
//!   a new template. It scales geometry only and leaves the joint-coupling / patella
//!   spline *relationships* intact ("coupling stays symbolic", recon D11).
//! * [`ParamSource`] + [`CanonicalSource`] — the pluggable parameter interface.
//!   `CanonicalSource` is **builder-first**: it emits the canonical body from
//!   library defaults with **no scan**.
//!
//! **What v1 reuses vs. defers.** The "template" is `cf_osim::osim::Subgraph` (the
//! validated gait2392 knee), re-exported here as [`Template`]; emission is
//! `cf_osim::emit::emit_coupled_knee`. The source-agnostic IR extraction and the
//! `cf-mjcf-emit` crate split (recon S1) are deliberately deferred until a second
//! joint actually needs them — generalizing one joint is premature. This crate is
//! the smallest real increment that makes the builder-first claim concrete and
//! oracle-validated.
//!
//! **Validation.** [`realize`] is ported verbatim from the throwaway morph spike
//! (`cf-osim/tests/spike_param_morph.rs`), which proved identity params are exact,
//! a uniform scale is an exact dilation, and anisotropic scaling preserves moment-
//! arm shape. `tests/builder_first.rs` re-establishes those as regression tests and
//! grades the canonical (no-scan) body against the OpenSim oracle.

use cf_osim::osim::{Kind, Subgraph};

pub use cf_osim::emit::Emitted;
/// The body template this crate morphs. v1 = the validated OpenSim knee subgraph;
/// the source-agnostic IR (recon S1) is a deferred generalization.
pub use cf_osim::osim::Subgraph as Template;

/// Parameters that morph a [`Template`] into a specific body.
///
/// v1 carries per-segment uniform **scale** factors — the slice the builder-first
/// (canonical / uniform) and anisotropic paths need, validated by the morph spike.
/// Scan-facing parameters (segment **lengths**, girths, joint **centers** — the
/// scanner-`Measurable` quantities a `ScanSource` derives) and the length→scale
/// derivation arrive with `ScanSource` (recon S3). Kept a plain, extensible struct
/// on purpose.
///
/// Scale convention matches the morph rule in [`realize`]: a point on a body is
/// scaled about that body's proximal-joint frame origin (the OpenSim Scale-tool
/// convention). The knee coupling splines scale with the femur; the patella moving
/// point scales with the tibia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyParams {
    /// Scale for points anchored to the pelvis / ground (proximal anchors).
    pub pelvis_scale: f64,
    /// Scale for the femur segment and the knee coupling (condyle) geometry.
    pub femur_scale: f64,
    /// Scale for the tibia segment and the patella moving point.
    pub tibia_scale: f64,
}

impl BodyParams {
    /// The identity morph — `realize` with these reproduces the template exactly.
    /// This is what [`CanonicalSource`] returns (builder-first defaults).
    pub const IDENTITY: BodyParams = BodyParams {
        pelvis_scale: 1.0,
        femur_scale: 1.0,
        tibia_scale: 1.0,
    };

    /// A single scale applied to every segment — an exact uniform dilation of the
    /// whole body (all moment arms scale by `s`, shape unchanged).
    pub fn uniform(s: f64) -> Self {
        BodyParams {
            pelvis_scale: s,
            femur_scale: s,
            tibia_scale: s,
        }
    }
}

impl Default for BodyParams {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Apply `params` to `template`, returning a new morphed template.
///
/// A pure function. It scales each path point's body-frame location by its
/// segment's factor, and the coupling / patella spline **outputs** by the owning
/// segment — leaving the spline knots and θ-dependence untouched (D11: scaling
/// changes geometry, not the rolling-glide *relationship*). Body-frame origins sit
/// at the proximal joint, so scaling a location is "scale the segment about its
/// proximal joint" — the OpenSim Scale-tool convention. Moment arms are
/// translation-invariant, so `hip_in_pelvis` (pure global placement) is scaled
/// with the femur for internal consistency only; it does not affect moment arms.
pub fn realize(template: &Template, params: &BodyParams) -> Subgraph {
    let scale_for = |body: &str| match body {
        "femur_r" => params.femur_scale,
        "tibia_r" => params.tibia_scale,
        _ => params.pelvis_scale, // pelvis / ground (proximal anchors)
    };

    let mut out = template.clone();
    out.hip_in_pelvis *= params.femur_scale;

    // Knee coupling splines: tibial translation in the femur frame → femur scale.
    out.knee.tx.scale *= params.femur_scale;
    out.knee.ty.scale *= params.femur_scale;
    out.knee.tz.scale *= params.femur_scale;

    for muscle in &mut out.muscles {
        for point in &mut muscle.path {
            let k = scale_for(&point.body);
            point.location *= k;
            if let Kind::Moving(splines) = &mut point.kind {
                // Patella moving point: its location splines ride the tibia frame.
                splines.x.scale *= k;
                splines.y.scale *= k;
                splines.z.scale *= k;
            }
        }
    }
    out
}

/// A pluggable source of [`BodyParams`] — the parameter interface. Where the
/// parameters come from (library defaults, a scan, a randomizer) is decoupled from
/// the morph + emit path.
pub trait ParamSource {
    /// The parameters this source produces for `template`.
    fn params(&self, template: &Template) -> BodyParams;
}

/// Builder-first: the canonical body from library defaults — **no scan**.
pub struct CanonicalSource;

impl ParamSource for CanonicalSource {
    fn params(&self, _template: &Template) -> BodyParams {
        BodyParams::IDENTITY
    }
}

/// Morph `template` with the parameters from `source` and emit the coupled-knee
/// MJCF — the full `ParamSource → realize → emit` path.
pub fn build(template: &Template, source: &dyn ParamSource) -> Emitted {
    cf_osim::emit::emit_coupled_knee(&realize(template, &source.params(template)))
}

/// The canonical (no-scan) body: [`build`] with [`CanonicalSource`]. The headline
/// builder-first artifact — a simulatable knee that needs no scan to exist.
pub fn build_canonical(template: &Template) -> Emitted {
    build(template, &CanonicalSource)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_is_default_and_uniform_one() {
        assert_eq!(BodyParams::default(), BodyParams::IDENTITY);
        assert_eq!(BodyParams::uniform(1.0), BodyParams::IDENTITY);
    }

    #[test]
    fn canonical_source_yields_identity() {
        // A tiny hand-built template is enough to check the source contract
        // without parsing an asset (that lives in tests/builder_first.rs).
        let params = CanonicalSource.params(&minimal_template());
        assert_eq!(params, BodyParams::IDENTITY);
    }

    #[test]
    fn realize_identity_clones_template() {
        let t = minimal_template();
        let out = realize(&t, &BodyParams::IDENTITY);
        assert_eq!(out.hip_in_pelvis, t.hip_in_pelvis);
        assert_eq!(out.muscles.len(), t.muscles.len());
        // Scale fields multiplied by 1.0 are unchanged.
        assert_eq!(out.knee.tx.scale, t.knee.tx.scale);
    }

    /// A degenerate one-muscle template — exercises the morph plumbing without the
    /// vendored `.osim` (the real oracle grading is in `tests/builder_first.rs`).
    fn minimal_template() -> Template {
        use cf_osim::osim::{KneeJoint, Muscle, PathPoint, Spline};
        use nalgebra::Vector3;
        Template {
            hip_in_pelvis: Vector3::new(0.0, -0.4, 0.0),
            knee: KneeJoint {
                flexion_axis: Vector3::z(),
                tx: Spline::constant(0.0),
                ty: Spline::constant(0.0),
                tz: Spline::constant(0.0),
            },
            muscles: vec![Muscle {
                name: "probe".to_string(),
                path: vec![
                    PathPoint {
                        name: "o".to_string(),
                        body: "femur_r".to_string(),
                        location: Vector3::new(0.0, 0.1, 0.0),
                        kind: Kind::Fixed,
                    },
                    PathPoint {
                        name: "i".to_string(),
                        body: "tibia_r".to_string(),
                        location: Vector3::new(0.0, -0.1, 0.0),
                        kind: Kind::Fixed,
                    },
                ],
            }],
        }
    }
}
