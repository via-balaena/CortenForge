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
//! * [`BodyParams`] — the parameters that morph a template into a body: a per-axis
//!   [`SegmentScale`] per segment (length → axial, girth → transverse). Real
//!   lengths are dialed via [`BodyParams::from_lengths`]; girth (transverse) is the
//!   machinery [`BodyParams::with_girth_scales`] sets, with the real
//!   girth→scale *derivation* (it needs an anthropometric reference) arriving with
//!   the generator (A3-PR3). The scale is the internal morph currency, matching
//!   OpenSim's ScaleTool per-axis.
//! * [`realize`] — a pure function that applies the morph to a [`Model`], returning a
//!   new `Model`. It scales geometry only and leaves the joint-coupling spline
//!   *relationships* intact ("coupling stays symbolic", recon D11).
//! * [`ParamSource`] + [`CanonicalSource`] — the pluggable parameter interface.
//!   `CanonicalSource` is **builder-first**: it emits the canonical body from
//!   library defaults with **no scan**.
//! * [`AnthroSource`] — the **dial-able** generator: a sex/percentile family of
//!   bodies (no scan), scaling the template proportionally from published
//!   stature/girth distributions. Validates the *machinery*, not personhood (see
//!   the [`anthro`] module).
//! * [`RandomizerSource`] — samples a **population** over the [`AnthroSource`] family
//!   (a seeded, dependency-free PRNG): coupled-by-default with a bounded decoupled
//!   tail (free training data; see the [`randomizer`] module). Validates *coverage +
//!   the machinery*, not personhood.

pub mod anthro;
pub mod ir;
pub mod muscle;
pub mod randomizer;
pub mod spline;

use nalgebra::Vector3;

pub use anthro::{AnthroSource, Sex};
pub use ir::{Body, Coordinate, Joint, Model, TransformAxis, TransformFn};
pub use muscle::{Kind, MovingSplines, Muscle, MuscleForce, PathPoint};
pub use randomizer::{RandomizerConfig, RandomizerSource, Rng};
pub use spline::Spline;

/// The body template this crate morphs: the source-agnostic kinematic-tree
/// [`Model`] produced by the `.osim` reader (`cf_osim::parse_leg_chain`).
pub type Template = Model;

/// Per-axis scale of one segment, in its body frame. For the gait2392 limb the
/// long axis is body-frame **y**, so [`axial`](Self::axial) (segment *length*)
/// drives *y* and [`transverse`](Self::transverse) (segment *girth*) drives *x*
/// and *z* together (a limb girth is one axisymmetric circumference, so *x* = *z*).
/// A uniform value (`axial == transverse`) is an isotropic scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentScale {
    /// Scale along the long (axial, *y*) axis — driven by segment **length**.
    pub axial: f64,
    /// Scale across the transverse (*x*, *z*) axes — driven by segment **girth**.
    pub transverse: f64,
}

impl SegmentScale {
    /// No scaling.
    pub const IDENTITY: SegmentScale = SegmentScale {
        axial: 1.0,
        transverse: 1.0,
    };

    /// An isotropic scale (axial == transverse).
    pub fn uniform(s: f64) -> Self {
        SegmentScale {
            axial: s,
            transverse: s,
        }
    }

    /// The per-axis scale vector in the body frame: `(transverse, axial, transverse)`
    /// — *x* and *z* transverse, *y* axial. This is what [`realize`] multiplies
    /// component-wise into frame-resident geometry.
    pub fn vector(self) -> Vector3<f64> {
        Vector3::new(self.transverse, self.axial, self.transverse)
    }
}

/// Parameters that morph a [`Template`] into a specific body.
///
/// A3 generalizes the morph from a scalar per-segment scale to a **per-axis**
/// [`SegmentScale`] (length → axial, girth → transverse) — a one-to-one match for
/// OpenSim's ScaleTool, which is what lets real OpenSim grade the morph (the
/// differential oracle). The user-facing dial is real **lengths/girths**;
/// [`BodyParams::from_lengths`] derives the scale currency from them against a
/// template. Scan-facing parameters (joint **centers**, the `Measurable`
/// quantities a `ScanSource` derives) live with `ScanSource` (cf-msk-fit).
///
/// Scale convention (pinned by the length round-trip + the differential oracle):
/// geometry scales with the segment whose **body frame expresses it** —
/// `location_in_parent` and a joint's translation axes live in the **parent**
/// frame (so they scale with the parent segment: the knee coupling and the femur
/// length scale with the femur; the ankle offset / tibia length scales with the
/// tibia), while a muscle path point and the patella moving point live in their
/// **own** body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyParams {
    /// Scale for points anchored to the pelvis / ground (the root anchor). v1 has
    /// no length/girth *dial* for the pelvis (it is the fixed root, and the leg
    /// twin's measurable params are the femur/tibia); set it directly if needed.
    pub pelvis: SegmentScale,
    /// Scale for the femur segment and the knee coupling (condyle) geometry.
    pub femur: SegmentScale,
    /// Scale for the tibia segment and the patella moving point.
    pub tibia: SegmentScale,
}

impl BodyParams {
    /// The identity morph — `realize` with these reproduces the template exactly.
    /// This is what [`CanonicalSource`] returns (builder-first defaults).
    pub const IDENTITY: BodyParams = BodyParams {
        pelvis: SegmentScale::IDENTITY,
        femur: SegmentScale::IDENTITY,
        tibia: SegmentScale::IDENTITY,
    };

    /// A single isotropic scale applied to every segment — an exact uniform
    /// dilation of the whole body (all moment arms scale by `s`, shape unchanged).
    pub fn uniform(s: f64) -> Self {
        BodyParams {
            pelvis: SegmentScale::uniform(s),
            femur: SegmentScale::uniform(s),
            tibia: SegmentScale::uniform(s),
        }
    }

    /// Derive the morph that gives the femur and tibia these **axial lengths**
    /// (meters), leaving girth (transverse) at the template default. The reference
    /// lengths are measured from `template` ([`Model::segment_axial_length`]), so
    /// realizing the result and re-measuring reproduces the targets exactly — the
    /// length round-trip (see the crate tests). Needs the ankle (`talus_r`) for the
    /// tibia length; on a knee-only template the tibia length is undefined.
    pub fn from_lengths(template: &Template, femur_len_m: f64, tibia_len_m: f64) -> Self {
        let f = template.segment_axial_length("femur_r", "tibia_r");
        let t = template.segment_axial_length("tibia_r", "talus_r");
        // Structural precondition, enforced in ALL builds (not debug-only): a
        // template missing the ankle (`talus_r`) has no tibia length, so the
        // division below would silently produce an Inf/NaN scale. Fail loudly at
        // the API boundary instead.
        assert!(
            f > 1e-6 && t > 1e-6,
            "degenerate template axial lengths (femur {f}, tibia {t}) — \
             is the ankle (talus_r) present?"
        );
        BodyParams {
            pelvis: SegmentScale::IDENTITY,
            femur: SegmentScale {
                axial: femur_len_m / f,
                transverse: 1.0,
            },
            tibia: SegmentScale {
                axial: tibia_len_m / t,
                transverse: 1.0,
            },
        }
    }

    /// Set the femur/tibia **transverse (girth)** scale factors, leaving axial
    /// (length) untouched. The factors are scales relative to the template; the
    /// real girth→scale derivation (a measured circumference / an anthropometric
    /// reference) lands with the generator/scan that owns the reference.
    pub fn with_girth_scales(mut self, femur_transverse: f64, tibia_transverse: f64) -> Self {
        self.femur.transverse = femur_transverse;
        self.tibia.transverse = tibia_transverse;
        self
    }
}

impl Default for BodyParams {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Apply `params` to `template`, returning a new morphed [`Model`].
///
/// A pure function implementing OpenSim's ScaleTool convention per-axis: geometry
/// scales **component-wise in the frame that expresses it**.
/// - `location_in_parent` and a joint's translation axes live in the **parent**
///   frame, so they scale with the parent segment's [`SegmentScale`] (the knee
///   coupling — which carries the femur length — scales with the femur; the ankle
///   offset — the tibia length — scales with the tibia).
/// - A muscle path point and the patella moving point live in their **own** body
///   frame, so they scale with that body.
///
/// Rotation axes and the spline *knots* (the θ-dependence) are untouched — scaling
/// changes geometry, not the rolling-glide *relationship* (D11). A uniform
/// [`BodyParams::uniform`] therefore reproduces an exact dilation (every moment arm
/// ×s); an anisotropic morph changes proportions while keeping the coupling.
pub fn realize(template: &Template, params: &BodyParams) -> Model {
    // Maps a body name to its segment scale. The pelvis (root anchor) is the
    // fallback; the talus also falls here, which is moot in v1 — it is the inert
    // length-grounding ankle endpoint with NO muscle points or dialable geometry
    // of its own (its `location_in_parent`, the tibia length, scales with its
    // PARENT tibia below, not via this map). Adding a distal segment with its own
    // geometry (e.g. a real foot) would need its own `BodyParams` entry here.
    let scale_for = |body: &str| -> Vector3<f64> {
        match body {
            "femur_r" => params.femur,
            "tibia_r" => params.tibia,
            _ => params.pelvis,
        }
        .vector()
    };

    let mut out = template.clone();
    let names: Vec<String> = out.bodies.iter().map(|b| b.name.clone()).collect();

    for b in &mut out.bodies {
        // `location_in_parent` and the translation axes live in the PARENT frame.
        let parent_scale = b
            .parent
            .map_or_else(|| Vector3::repeat(1.0), |p| scale_for(&names[p]));
        b.location_in_parent = b.location_in_parent.component_mul(&parent_scale);
        for ax in &mut b.joint {
            if !ax.rotation {
                // The scale of this axis's (canonical) direction: project the
                // per-axis scale onto the unit axis. For e_i this picks
                // `parent_scale[i]`; gait2392's translation axes are canonical.
                let f = ax.axis.component_mul(&parent_scale).dot(&ax.axis);
                match &mut ax.function {
                    TransformFn::Spline(s) => s.scale *= f,
                    TransformFn::Constant(v) => *v *= f,
                    TransformFn::Linear { coeff } => *coeff *= f,
                }
            }
        }
    }

    for muscle in &mut out.muscles {
        for point in &mut muscle.path {
            let k = scale_for(&point.body);
            point.location = point.location.component_mul(&k);
            if let Kind::Moving(splines) = &mut point.kind {
                // Patella moving point: its location splines ride its body frame.
                splines.x.scale *= k.x;
                splines.y.scale *= k.y;
                splines.z.scale *= k.z;
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
            pelvis: SegmentScale::IDENTITY,
            femur: SegmentScale {
                axial: 2.0,
                transverse: 3.0,
            },
            tibia: SegmentScale {
                axial: 0.5,
                transverse: 0.7,
            },
        };
        let r = realize(&t, &p);
        // The femur's `location_in_parent` lives in the PELVIS frame, so it scales
        // with the pelvis (identity here) — NOT the femur. The femur *length* lives
        // in the child joint's translation and scales with the femur (see the
        // coupled-translation test below).
        assert_eq!(
            r.bodies[1].location_in_parent,
            t.bodies[1].location_in_parent
        );
        // muscle points scale component-wise (x,z transverse; y axial) with body.
        for (mo, mr) in t.muscles.iter().zip(&r.muscles) {
            for (po, pr) in mo.path.iter().zip(&mr.path) {
                let s = match po.body.as_str() {
                    "femur_r" => p.femur,
                    "tibia_r" => p.tibia,
                    _ => SegmentScale::IDENTITY,
                };
                let expected = po.location.component_mul(&s.vector());
                assert!((pr.location - expected).norm() < 1e-12);
            }
        }
    }

    /// The morph's per-frame scaling rule, **per axis**: a joint's translation
    /// outputs live in the parent frame and scale with the parent segment along the
    /// axis's direction (the knee `tx` with the femur's transverse, `ty` with the
    /// femur's axial); a body's `location_in_parent` also lives in the parent frame
    /// (so it scales with the parent); rotation axes are untouched. Pins the rule
    /// numerically (the emitted-MJCF path only asserts loadability).
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
                pelvis: SegmentScale::IDENTITY,
                femur: SegmentScale {
                    axial: 2.0,
                    transverse: 3.0,
                },
                tibia: SegmentScale {
                    axial: 0.5,
                    transverse: 0.7,
                },
            },
        );
        let tibia = &r.bodies[2].joint;
        // Translations scale with the PARENT (femur) along the axis's direction:
        // `tx` (x-axis) with femur transverse (3.0) → 0.1·3 = 0.3.
        match &tibia[0].function {
            TransformFn::Constant(v) => assert!((*v - 0.3).abs() < 1e-12, "constant tx {v} != 0.3"),
            other => panic!("expected Constant, got {other:?}"),
        }
        // `ty` (y-axis) with femur axial (2.0) → spline scale 1·2 = 2.0.
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
        // `location_in_parent` lives in the parent frame: the tibia's (femur frame)
        // scales with the femur component-wise → (0,−0.1,0)·(3,2,3) = (0,−0.2,0);
        // the femur's (pelvis frame) scales with the pelvis (identity) → unchanged.
        assert!((r.bodies[2].location_in_parent - Vector3::new(0.0, -0.2, 0.0)).norm() < 1e-12);
        assert!((r.bodies[1].location_in_parent - Vector3::new(0.0, -0.4, 0.0)).norm() < 1e-12);
    }

    /// **The length round-trip — Tier-2 internal consistency, the convention pin.**
    /// Dial real axial lengths, realize, re-measure: the morph reproduces the
    /// targets *exactly*. This is what would FAIL under the pre-A3 convention
    /// (`location_in_parent` scaling with the own body), so it pins the corrected
    /// "offset scales with the frame that expresses it" rule. It deliberately
    /// exercises BOTH length encodings present in gait2392: the femur length lives
    /// in the knee *joint translation* (scales with the parent femur); the tibia
    /// length lives in the talus `location_in_parent` (scales with the parent tibia).
    #[test]
    fn length_round_trip_reproduces_dialed_axial_lengths() {
        let t = chain_template();
        // template axial lengths: femur 0.45 (knee joint ty), tibia 0.40 (talus offset).
        assert!((t.segment_axial_length("femur_r", "tibia_r") - 0.45).abs() < 1e-12);
        assert!((t.segment_axial_length("tibia_r", "talus_r") - 0.40).abs() < 1e-12);

        for &(fl, tl) in &[(0.6, 0.5), (0.30, 0.55), (0.45, 0.40)] {
            let r = realize(&t, &BodyParams::from_lengths(&t, fl, tl));
            assert!(
                (r.segment_axial_length("femur_r", "tibia_r") - fl).abs() < 1e-12,
                "femur length round-trip: dialed {fl}, got {}",
                r.segment_axial_length("femur_r", "tibia_r")
            );
            assert!(
                (r.segment_axial_length("tibia_r", "talus_r") - tl).abs() < 1e-12,
                "tibia length round-trip: dialed {tl}, got {}",
                r.segment_axial_length("tibia_r", "talus_r")
            );
        }
    }

    /// The girth (transverse) round-trip: dialing a segment's transverse scale
    /// scales that segment's muscle-point *x,z* components exactly while leaving the
    /// axial (*y*) component — and the other segment — untouched. The full-girth
    /// morph machinery (length and girth are independent per-axis dials).
    #[test]
    fn girth_round_trip_scales_transverse_only() {
        let t = chain_template();
        let p = BodyParams::from_lengths(&t, 0.45, 0.40).with_girth_scales(1.5, 0.8);
        let r = realize(&t, &p);
        // femur point (0.10, 0.20, 0.05): x,z ×1.5, y unchanged (axial dialed to 1).
        let fp = r.muscles[0].path[0].location;
        assert!(
            (fp - Vector3::new(0.15, 0.20, 0.075)).norm() < 1e-12,
            "femur pt {fp:?}"
        );
        // tibia point (0.02, -0.10, -0.03): x,z ×0.8, y unchanged.
        let tp = r.muscles[0].path[1].location;
        assert!(
            (tp - Vector3::new(0.016, -0.10, -0.024)).norm() < 1e-12,
            "tibia pt {tp:?}"
        );
        // Girth must NOT change the segment lengths — assert axial lengths directly
        // (not just implicitly via the unchanged muscle-point y-components above).
        assert!((r.segment_axial_length("femur_r", "tibia_r") - 0.45).abs() < 1e-12);
        assert!((r.segment_axial_length("tibia_r", "talus_r") - 0.40).abs() < 1e-12);
    }

    /// `from_lengths` fails loudly (in release too) on a template with no ankle —
    /// the tibia length is undefined, so a silent Inf/NaN scale must not slip out.
    #[test]
    #[should_panic(expected = "is the ankle (talus_r) present?")]
    fn from_lengths_panics_without_ankle() {
        // `minimal_template` is pelvis+femur only — no tibia, no talus.
        let _ = BodyParams::from_lengths(&minimal_template(), 0.4, 0.4);
    }

    /// Pelvis anisotropy IS applied (the pelvis code path isn't dead): a muscle
    /// point anchored to the pelvis scales component-wise with the pelvis scale.
    #[test]
    fn pelvis_anisotropy_scales_pelvis_points() {
        use crate::muscle::{Kind, Muscle, PathPoint};
        let mut t = chain_template();
        t.muscles.push(Muscle {
            name: "p".to_string(),
            force: None,
            path: vec![PathPoint {
                name: "o".to_string(),
                body: "pelvis".to_string(),
                location: Vector3::new(0.10, 0.20, 0.04),
                kind: Kind::Fixed,
            }],
        });
        let p = BodyParams {
            pelvis: SegmentScale {
                axial: 0.9,
                transverse: 1.1,
            },
            femur: SegmentScale::IDENTITY,
            tibia: SegmentScale::IDENTITY,
        };
        let r = realize(&t, &p);
        let pt = r.muscles[1].path[0].location;
        // x,z ×1.1 (transverse), y ×0.9 (axial).
        assert!(
            (pt - Vector3::new(0.11, 0.18, 0.044)).norm() < 1e-12,
            "pelvis pt {pt:?}"
        );
    }

    /// The per-axis projection factor is correct for an OBLIQUE translation axis
    /// (gait2392 has none, but the formula must hold if a future model does): a
    /// translation along a non-canonical direction scales by the anisotropic scale
    /// projected onto that direction.
    #[test]
    fn oblique_translation_axis_scales_by_projection() {
        use crate::ir::{TransformAxis, TransformFn};
        let axis = Vector3::new(1.0, 1.0, 0.0).normalize(); // 45° in the x–y plane
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
                    location_in_parent: Vector3::zeros(),
                    joint: vec![],
                },
                Body {
                    name: "tibia_r".into(),
                    parent: Some(1), // parent = femur
                    location_in_parent: Vector3::zeros(),
                    joint: vec![TransformAxis {
                        rotation: false,
                        axis,
                        coordinate: String::new(),
                        function: TransformFn::Constant(1.0),
                    }],
                },
            ],
            coordinates: vec![],
            muscles: vec![],
        };
        // femur transverse(x)=2, axial(y)=4 → vector (2,4,2); project onto the unit
        // 45° axis: (2,4,0)·(1,1,0)/√2·… → f = (axis∘scale)·axis = 2·0.5 + 4·0.5 = 3.
        let p = BodyParams {
            pelvis: SegmentScale::IDENTITY,
            femur: SegmentScale {
                axial: 4.0,
                transverse: 2.0,
            },
            tibia: SegmentScale::IDENTITY,
        };
        let r = realize(&model, &p);
        match &r.bodies[2].joint[0].function {
            TransformFn::Constant(v) => assert!((*v - 3.0).abs() < 1e-12, "got {v}, want 3.0"),
            other => panic!("expected Constant, got {other:?}"),
        }
    }

    /// A four-body chain (pelvis → femur → tibia → talus) with known axial lengths,
    /// mirroring gait2392's two length encodings (knee joint translation for the
    /// femur, talus offset for the tibia). Pure — no vendored `.osim`.
    fn chain_template() -> Template {
        use crate::ir::{TransformAxis, TransformFn};
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
                    location_in_parent: Vector3::new(0.0, -0.05, 0.0), // hip in pelvis
                    joint: vec![],
                },
                Body {
                    name: "tibia_r".into(),
                    parent: Some(1),
                    location_in_parent: Vector3::zeros(), // knee at femur origin (gait2392)
                    joint: vec![
                        // femur length: a constant -0.45 translation along y (femur frame).
                        TransformAxis {
                            rotation: false,
                            axis: Vector3::y(),
                            coordinate: String::new(),
                            function: TransformFn::Constant(-0.45),
                        },
                        // knee flexion hinge about z (identity at default q=0).
                        TransformAxis {
                            rotation: true,
                            axis: Vector3::z(),
                            coordinate: "knee".into(),
                            function: TransformFn::Linear { coeff: 1.0 },
                        },
                    ],
                },
                Body {
                    name: "talus_r".into(),
                    parent: Some(2),
                    location_in_parent: Vector3::new(0.0, -0.40, 0.0), // tibia length
                    joint: vec![],
                },
            ],
            coordinates: vec![],
            muscles: vec![Muscle {
                name: "m".to_string(),
                force: None,
                path: vec![
                    PathPoint {
                        name: "o".to_string(),
                        body: "femur_r".to_string(),
                        location: Vector3::new(0.10, 0.20, 0.05),
                        kind: Kind::Fixed,
                    },
                    PathPoint {
                        name: "i".to_string(),
                        body: "tibia_r".to_string(),
                        location: Vector3::new(0.02, -0.10, -0.03),
                        kind: Kind::Fixed,
                    },
                ],
            }],
        }
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
                force: None,
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
