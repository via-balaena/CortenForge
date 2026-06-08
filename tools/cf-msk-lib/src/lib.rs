//! cf-msk-lib — the library/parameter spine of the musculoskeletal builder.
//!
//! The parametric-builder-first reframe (`docs/msk_builder/01_vision/vision.md`)
//! turns the program inside out: a body is *generated* from a validated template
//! and a small **parameter** vector, and a scan — when present — is just one
//! source of those parameters, never the entry point. This crate is that spine:
//!
//! ```text
//!   ParamSource ──► BodyParams ──► realize(template, params) ──► emit ──► MJCF
//!    (pluggable)                      (pure morph)           (cf-mjcf-emit)
//! ```
//!
//! As of the leg-region cutover (recon `03_phases/leg_region`, A1 PR-2) this crate
//! owns the **source-agnostic kinematic-tree IR** ([`ir`]) and the model types it
//! is built from ([`Spline`], [`Muscle`] et al.), and the dependency points
//! cf-osim → cf-msk-lib (the `.osim` reader produces a [`Model`]). Emission lives
//! in the sibling `cf-mjcf-emit` crate, so this crate is `MJCF`-free and pure.
//!
//! * [`Model`] (in [`ir`]) — the template: a `Body` tree of `CustomJoint` transform
//!   axes plus straight-segment muscle paths. Produced by `cf_osim::parse_leg_chain`.
//! * [`BodyParams`] — the parameters that morph a template into a body. v1 carries
//!   per-segment **scale** factors (the canonical/uniform slice the morph spike
//!   validated); scan-facing length/girth/center params arrive with `ScanSource`.
//! * [`realize`] — a pure function that applies the morph to a [`Model`], returning a
//!   new `Model`. It scales geometry only and leaves the joint-coupling spline
//!   *relationships* intact ("coupling stays symbolic", recon D11).
//! * [`ParamSource`] + [`CanonicalSource`] — the pluggable parameter interface.
//!   `CanonicalSource` is **builder-first**: it emits the canonical body from
//!   library defaults with **no scan**.

pub mod ir;
pub mod muscle;
pub mod spline;

pub use ir::{Body, Coordinate, Joint, Model, TransformAxis, TransformFn};
pub use muscle::{Kind, MovingSplines, Muscle, PathPoint};
pub use spline::Spline;

/// The body template this crate morphs: the source-agnostic kinematic-tree
/// [`Model`] produced by the `.osim` reader (`cf_osim::parse_leg_chain`).
pub type Template = Model;

/// Parameters that morph a [`Template`] into a specific body.
///
/// v1 carries per-segment uniform **scale** factors — the slice the builder-first
/// (canonical / uniform) and anisotropic paths need, validated by the morph spike.
/// Scan-facing parameters (segment **lengths**, girths, joint **centers** — the
/// scanner-`Measurable` quantities a `ScanSource` derives) and the length→scale
/// derivation live with `ScanSource` (cf-msk-fit). Kept a plain, extensible struct
/// on purpose.
///
/// Scale convention matches the morph rule in [`realize`]: a point on a body is
/// scaled about that body's proximal-joint frame origin (the OpenSim Scale-tool
/// convention). A joint's coupled translation splines scale with the parent
/// segment (they are expressed in the parent frame); the patella moving point
/// scales with the tibia.
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

/// Apply `params` to `template`, returning a new morphed [`Model`].
///
/// A pure function. Each body's placement (`location_in_parent`) scales with that
/// body's segment, its joint's coupled translation outputs scale with the **parent**
/// segment (they live in the parent frame — e.g. the knee coupling scales with the
/// femur), and each muscle path point / patella spline scales with its owning body.
/// Rotation axes and the spline *knots* (the θ-dependence) are untouched — scaling
/// changes geometry, not the rolling-glide *relationship* (D11). Body-frame origins
/// sit at the proximal joint, so scaling a location is "scale the segment about its
/// proximal joint" — the OpenSim Scale-tool convention. Moment arms are
/// translation-invariant, so a body's `location_in_parent` (pure placement) scales
/// for internal consistency only; it does not affect moment arms.
pub fn realize(template: &Template, params: &BodyParams) -> Model {
    let scale_for = |body: &str| match body {
        "femur_r" => params.femur_scale,
        "tibia_r" => params.tibia_scale,
        _ => params.pelvis_scale, // pelvis / ground (proximal anchors)
    };

    let mut out = template.clone();
    let names: Vec<String> = out.bodies.iter().map(|b| b.name.clone()).collect();

    for b in &mut out.bodies {
        b.location_in_parent *= scale_for(&b.name);
        // Coupled translations are expressed in the parent frame, so they scale
        // with the parent segment (the knee coupling scales with the femur).
        let parent_scale = b.parent.map_or(1.0, |p| scale_for(&names[p]));
        for ax in &mut b.joint {
            if !ax.rotation {
                match &mut ax.function {
                    TransformFn::Spline(s) => s.scale *= parent_scale,
                    TransformFn::Constant(v) => *v *= parent_scale,
                    TransformFn::Linear { coeff } => *coeff *= parent_scale,
                }
            }
        }
    }

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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn identity_is_default_and_uniform_one() {
        assert_eq!(BodyParams::default(), BodyParams::IDENTITY);
        assert_eq!(BodyParams::uniform(1.0), BodyParams::IDENTITY);
    }

    #[test]
    fn canonical_source_yields_identity() {
        let params = CanonicalSource.params(&minimal_template());
        assert_eq!(params, BodyParams::IDENTITY);
    }

    #[test]
    fn realize_identity_clones_template() {
        let t = minimal_template();
        let out = realize(&t, &BodyParams::IDENTITY);
        assert_eq!(out.bodies.len(), t.bodies.len());
        assert_eq!(out.muscles.len(), t.muscles.len());
        assert_eq!(
            out.bodies[1].location_in_parent,
            t.bodies[1].location_in_parent
        );
    }

    #[test]
    fn realize_scales_placement_and_muscle_points() {
        let t = minimal_template();
        let p = BodyParams {
            pelvis_scale: 1.0,
            femur_scale: 2.0,
            tibia_scale: 0.5,
        };
        let r = realize(&t, &p);
        // femur placement scales with the femur.
        assert_eq!(
            r.bodies[1].location_in_parent,
            t.bodies[1].location_in_parent * 2.0
        );
        // muscle points scale with their owning body.
        for (mo, mr) in t.muscles.iter().zip(&r.muscles) {
            for (po, pr) in mo.path.iter().zip(&mr.path) {
                let k = match po.body.as_str() {
                    "femur_r" => 2.0,
                    "tibia_r" => 0.5,
                    _ => 1.0,
                };
                assert!((pr.location - po.location * k).norm() < 1e-12);
            }
        }
    }

    /// The morph's per-frame scaling rule: a joint's coupled translation outputs
    /// scale with the **parent** segment (they live in the parent frame — the knee
    /// coupling scales with the femur), a body's placement scales with itself, and
    /// rotation axes are untouched. This pins the rule numerically (the emitted-MJCF
    /// path only asserts loadability).
    #[test]
    fn realize_scales_coupled_translation_with_parent_segment() {
        use crate::ir::{TransformAxis, TransformFn};
        use crate::spline::Spline;
        let model = Model {
            bodies: vec![
                Body {
                    name: "pelvis".into(),
                    parent: None,
                    location_in_parent: Vector3::zeros(),
                    joint: vec![],
                },
                Body {
                    name: "femur_r".into(),
                    parent: Some(0),
                    location_in_parent: Vector3::new(0.0, -0.4, 0.0),
                    joint: vec![],
                },
                Body {
                    name: "tibia_r".into(),
                    parent: Some(1), // parent = femur
                    location_in_parent: Vector3::new(0.0, -0.1, 0.0),
                    joint: vec![
                        TransformAxis {
                            rotation: false,
                            axis: Vector3::x(),
                            coordinate: "k".into(),
                            function: TransformFn::Constant(0.1),
                        },
                        TransformAxis {
                            rotation: false,
                            axis: Vector3::y(),
                            coordinate: "k".into(),
                            function: TransformFn::Spline(Spline::new(
                                vec![0.0, 1.0],
                                vec![0.0, 0.2],
                                1.0,
                            )),
                        },
                        TransformAxis {
                            rotation: true,
                            axis: Vector3::z(),
                            coordinate: "k".into(),
                            function: TransformFn::Linear { coeff: 1.0 },
                        },
                    ],
                },
            ],
            coordinates: vec![],
            muscles: vec![],
        };
        let r = realize(
            &model,
            &BodyParams {
                pelvis_scale: 1.0,
                femur_scale: 2.0,
                tibia_scale: 0.5,
            },
        );
        let tibia = &r.bodies[2].joint;
        // Coupled translations scale with the PARENT (femur = 2.0), NOT tibia (0.5).
        match &tibia[0].function {
            TransformFn::Constant(v) => assert!((*v - 0.2).abs() < 1e-12, "constant tx {v} != 0.2"),
            other => panic!("expected Constant, got {other:?}"),
        }
        match &tibia[1].function {
            TransformFn::Spline(s) => {
                assert!(
                    (s.scale - 2.0).abs() < 1e-12,
                    "spline scale {} != 2.0",
                    s.scale
                )
            }
            other => panic!("expected Spline, got {other:?}"),
        }
        // Rotation axes are untouched — angles don't scale.
        match &tibia[2].function {
            TransformFn::Linear { coeff } => assert!((*coeff - 1.0).abs() < 1e-12),
            other => panic!("expected Linear, got {other:?}"),
        }
        // A body's placement scales with itself: tibia × tibia_scale, femur × femur_scale.
        assert!((r.bodies[2].location_in_parent - Vector3::new(0.0, -0.05, 0.0)).norm() < 1e-12);
        assert!((r.bodies[1].location_in_parent - Vector3::new(0.0, -0.8, 0.0)).norm() < 1e-12);
    }

    /// A degenerate one-muscle template — exercises the morph plumbing without the
    /// vendored `.osim` (real oracle grading lives in cf-mjcf-emit's tests).
    fn minimal_template() -> Template {
        use crate::muscle::{Kind, Muscle, PathPoint};
        Model {
            bodies: vec![
                Body {
                    name: "pelvis".into(),
                    parent: None,
                    location_in_parent: Vector3::zeros(),
                    joint: vec![],
                },
                Body {
                    name: "femur_r".into(),
                    parent: Some(0),
                    location_in_parent: Vector3::new(0.0, -0.4, 0.0),
                    joint: vec![],
                },
            ],
            coordinates: vec![],
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
