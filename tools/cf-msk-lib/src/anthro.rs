//! `AnthroSource` — a sex/percentile **anthropometric generator**: the dial-able
//! body the A3 reframe is about. It turns `CanonicalSource` (one fixed canonical
//! body) into a *family* of bodies parameterized by sex and percentile, with no
//! scan.
//!
//! ## What it does (and the honesty boundary)
//!
//! It scales the validated template **proportionally**, using published adult
//! stature/girth distributions, taking the template as a **reference percentile**
//! (default 50th-percentile male). So:
//! - at the reference it reproduces the template **exactly** (every ratio = 1);
//! - elsewhere, segment lengths scale by the **stature ratio** (segment length ∝
//!   stature, Winter 2009) and girths by the **girth ratio** vs the reference.
//!
//! This is deliberately a *relative* family anchored on the validated template, not
//! an absolutely-calibrated one. Two reasons it is the honest choice: (1) the
//! template's absolute size is gait2392's (subject01), which we *declare* the
//! reference — we do not re-label its bones with absolute table values; (2) it
//! avoids the definitional mismatch between a table's "thigh length"
//! (greater-trochanter→knee) and our femur **axial** length (hip-joint→knee).
//! Per the vision's two-claims rule, this validates the **machinery** (the morph
//! tracks literature proportions), **not personhood** — a generated body is "a
//! clone of someone with these proportions," never a specific person.
//!
//! Joint default poses are out of scope here: a [`BodyParams`] carries **size**
//! (per-axis scale), not pose. The generated body is built at the template's
//! neutral pose; articulating it is the fitter/animation concern, not the morph.
//!
//! ## The table
//!
//! Representative adult anthropometry — stature from ANSUR II (US Army, 2012);
//! segment-length-∝-stature from Winter, *Biomechanics and Motor Control of Human
//! Movement* (2009); thigh/calf circumference are representative ANSUR-II-scale
//! values. They are the *literature anchor* for the proportions (we do not invent
//! them); the exact figures are coarse and meant to be refined, which is why the
//! validation is internal-consistency + plausibility + shape-correlation, not a
//! claim of per-person accuracy.

use crate::{BodyParams, ParamSource, Template};

/// Biological sex — selects the anthropometric distribution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sex {
    Male,
    Female,
}

/// A normal distribution (meters) for one measurement.
#[derive(Debug, Clone, Copy)]
struct Norm {
    mean: f64,
    sd: f64,
}

impl Norm {
    /// The value at percentile `p ∈ (0, 1)` — `mean + sd · Φ⁻¹(p)`.
    fn at(&self, p: f64) -> f64 {
        self.mean + self.sd * probit(p)
    }
}

/// One sex's distributions: stature and the two limb girths we dial.
struct SexAnthro {
    stature: Norm,
    thigh_girth: Norm,
    calf_girth: Norm,
}

/// Representative adult anthropometry (see the module docs for sources). `sd`
/// values give realistic percentile spread; figures are coarse by design.
fn table(sex: Sex) -> SexAnthro {
    match sex {
        Sex::Male => SexAnthro {
            stature: Norm {
                mean: 1.756,
                sd: 0.0686,
            },
            thigh_girth: Norm {
                mean: 0.620,
                sd: 0.054,
            },
            calf_girth: Norm {
                mean: 0.377,
                sd: 0.028,
            },
        },
        Sex::Female => SexAnthro {
            stature: Norm {
                mean: 1.629,
                sd: 0.0640,
            },
            thigh_girth: Norm {
                mean: 0.600,
                sd: 0.056,
            },
            calf_girth: Norm {
                mean: 0.359,
                sd: 0.029,
            },
        },
    }
}

/// The reference the template is taken to represent: 50th-percentile male. At this
/// `(sex, percentile)` the generator reproduces the template exactly.
const REF_SEX: Sex = Sex::Male;
const REF_PCT: f64 = 0.5;

/// A sex/percentile anthropometric body generator (a [`ParamSource`]).
///
/// Scales the template **relative to a fixed reference** — 50th-percentile male —
/// at which it reproduces the template exactly; elsewhere lengths scale by the
/// stature ratio and girths by the girth ratio vs that reference. Percentiles are
/// in the open interval `(0, 1)` (the 0th/100th are ±∞ z-scores); they are
/// validated when [`params`](ParamSource::params) is called, not at construction.
///
/// ```
/// # use cf_msk_lib::anthro::{AnthroSource, Sex};
/// // A 90th-percentile female; girth tracks stature unless set separately.
/// let src = AnthroSource::new(Sex::Female, 0.90);
/// let _taller_girthier = src.with_girth_percentile(0.75);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct AnthroSource {
    pub sex: Sex,
    /// Stature percentile in `(0, 1)` — drives segment lengths.
    pub stature_percentile: f64,
    /// Girth percentile in `(0, 1)` — drives segment transverse (girth) scale.
    pub girth_percentile: f64,
}

impl AnthroSource {
    /// A body at the given sex and stature percentile; girth tracks stature.
    pub fn new(sex: Sex, stature_percentile: f64) -> Self {
        AnthroSource {
            sex,
            stature_percentile,
            girth_percentile: stature_percentile,
        }
    }

    /// Set the girth percentile independently of stature (e.g. a tall lean or a
    /// short stocky build).
    ///
    /// Note: an **extreme** stature/girth decoupling (the two percentiles far
    /// apart) produces large transverse-only attachment offsets that distort
    /// moment-arm curve *shape* (the most girth-sensitive hamstring drops to ~0.87
    /// correlation vs canonical). The **default** coupled family (girth tracks
    /// stature, [`AnthroSource::new`]) is the validated regime (≥0.95); see
    /// `cf-osim`'s `anthro_validation` tests.
    pub fn with_girth_percentile(mut self, p: f64) -> Self {
        self.girth_percentile = p;
        self
    }
}

impl ParamSource for AnthroSource {
    fn params(&self, template: &Template) -> BodyParams {
        // Open interval (both ends excluded): the 0th/100th percentiles are ±∞
        // z-scores (`probit`'s domain), so reject them here at the API boundary with
        // a clear message rather than panicking deeper in `probit`. (A half-open
        // `0.0..1.0` would wrongly admit 0.0, so compare explicitly.)
        let in_open = |p: f64| p > 0.0 && p < 1.0;
        assert!(
            in_open(self.stature_percentile) && in_open(self.girth_percentile),
            "percentiles must be in (0,1) (stature {}, girth {})",
            self.stature_percentile,
            self.girth_percentile
        );
        let t = table(self.sex);
        let r = table(REF_SEX);

        // Lengths scale by the stature ratio vs the reference (segment ∝ stature).
        let stature_ratio = t.stature.at(self.stature_percentile) / r.stature.at(REF_PCT);
        let femur_axial = template.segment_axial_length("femur_r", "tibia_r");
        let tibia_axial = template.segment_axial_length("tibia_r", "talus_r");

        // Girths scale by the girth ratio vs the reference (the transverse dial).
        let femur_girth = t.thigh_girth.at(self.girth_percentile) / r.thigh_girth.at(REF_PCT);
        let tibia_girth = t.calf_girth.at(self.girth_percentile) / r.calf_girth.at(REF_PCT);

        BodyParams::from_lengths(
            template,
            femur_axial * stature_ratio,
            tibia_axial * stature_ratio,
        )
        .with_girth_scales(femur_girth, tibia_girth)
    }
}

/// Inverse standard-normal CDF (Φ⁻¹), Acklam's rational approximation — accurate to
/// ~1.15e-9 over `p ∈ (0, 1)`. Maps a percentile to a z-score. No external dep
/// (this crate stays pure); `probit(0.5) = 0`.
fn probit(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969683028665376e+01,
        2.209460984245205e+02,
        -2.759285104469687e+02,
        1.38357751867269e+02,
        -3.066479806614716e+01,
        2.506628277459239e+00,
    ];
    const B: [f64; 5] = [
        -5.447609879822406e+01,
        1.615858368580409e+02,
        -1.556989798598866e+02,
        6.680131188771972e+01,
        -1.328068155288572e+01,
    ];
    const C: [f64; 6] = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e+00,
        -2.549732539343734e+00,
        4.374664141464968e+00,
        2.938163982698783e+00,
    ];
    const D: [f64; 4] = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e+00,
        3.754408661907416e+00,
    ];
    const PLOW: f64 = 0.02425;
    const PHIGH: f64 = 1.0 - PLOW;
    assert!(p > 0.0 && p < 1.0, "probit domain is (0,1), got {p}");
    if p < PLOW {
        let q = (-2.0 * p.ln()).sqrt();
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    } else if p <= PHIGH {
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SegmentScale, realize};
    use nalgebra::Vector3;

    #[test]
    fn probit_known_quantiles() {
        assert!(probit(0.5).abs() < 1e-9);
        assert!((probit(0.975) - 1.959_963_98).abs() < 1e-6);
        assert!((probit(0.025) + 1.959_963_98).abs() < 1e-6);
        // Monotonic.
        assert!(probit(0.1) < probit(0.2) && probit(0.2) < probit(0.8));
    }

    /// At the reference (50th-percentile male) the generator reproduces the
    /// template exactly — every scale ratio is 1.
    #[test]
    fn reference_percentile_is_identity() {
        let t = chain_template();
        let p = AnthroSource::new(Sex::Male, 0.5).params(&t);
        let i = BodyParams::IDENTITY;
        assert!((p.femur.axial - i.femur.axial).abs() < 1e-12);
        assert!((p.femur.transverse - 1.0).abs() < 1e-12);
        assert!((p.tibia.axial - 1.0).abs() < 1e-12);
        assert!((p.tibia.transverse - 1.0).abs() < 1e-12);
        assert_eq!(p.pelvis, SegmentScale::IDENTITY);
    }

    /// Taller percentile → longer realized segments; girth percentile drives the
    /// transverse scale; female stature < male at the same percentile.
    #[test]
    fn percentile_monotonic_and_plausible() {
        let t = chain_template();
        let femur_len = |src: AnthroSource| {
            realize(&t, &src.params(&t)).segment_axial_length("femur_r", "tibia_r")
        };
        let m05 = femur_len(AnthroSource::new(Sex::Male, 0.5));
        let m95 = femur_len(AnthroSource::new(Sex::Male, 0.95));
        let f05 = femur_len(AnthroSource::new(Sex::Female, 0.5));
        assert!(
            m95 > m05,
            "taller percentile must be longer ({m05} -> {m95})"
        );
        assert!(f05 < m05, "female 50th < male 50th ({f05} vs {m05})");

        // Girth percentile drives transverse, not axial.
        let lean = AnthroSource::new(Sex::Male, 0.5).with_girth_percentile(0.05);
        let stocky = AnthroSource::new(Sex::Male, 0.5).with_girth_percentile(0.95);
        assert!(lean.params(&t).femur.transverse < stocky.params(&t).femur.transverse);
        assert!((lean.params(&t).femur.axial - stocky.params(&t).femur.axial).abs() < 1e-12);

        // Plausibility: realized femur axial stays physiological across the range
        // (template axial 0.45 here → scaled by the stature ratio).
        for &pct in &[0.01, 0.5, 0.99] {
            for sex in [Sex::Male, Sex::Female] {
                let l = femur_len(AnthroSource::new(sex, pct));
                assert!(
                    (0.30..0.65).contains(&l),
                    "femur {l} implausible at {sex:?}/{pct}"
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "percentiles must be in")]
    fn rejects_out_of_range_percentile() {
        let _ = AnthroSource::new(Sex::Male, 1.5).params(&chain_template());
    }

    /// The open-interval boundary (0.0 / 1.0) is rejected at the API boundary with
    /// the clear `params` message — not deeper inside `probit` (its domain is the
    /// open interval, the 0th/100th percentiles being ±∞ z-scores).
    #[test]
    #[should_panic(expected = "percentiles must be in (0,1)")]
    fn rejects_boundary_percentile_zero() {
        let _ = AnthroSource::new(Sex::Male, 0.0).params(&chain_template());
    }

    #[test]
    #[should_panic(expected = "percentiles must be in (0,1)")]
    fn rejects_boundary_girth_one() {
        let _ = AnthroSource::new(Sex::Male, 0.5)
            .with_girth_percentile(1.0)
            .params(&chain_template());
    }

    /// A minimal four-body chain with a defined femur (0.45) and tibia (0.40) axial
    /// length, so `from_lengths`/`segment_axial_length` resolve (mirrors the morph
    /// tests' fixture). Pure — no vendored `.osim`.
    fn chain_template() -> Template {
        use crate::ir::{TransformAxis, TransformFn};
        use crate::{Body, Model};
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
                    location_in_parent: Vector3::new(0.0, -0.05, 0.0),
                    joint: vec![],
                },
                Body {
                    name: "tibia_r".into(),
                    parent: Some(1),
                    location_in_parent: Vector3::zeros(),
                    joint: vec![TransformAxis {
                        rotation: false,
                        axis: Vector3::y(),
                        coordinate: String::new(),
                        function: TransformFn::Constant(-0.45),
                    }],
                },
                Body {
                    name: "talus_r".into(),
                    parent: Some(2),
                    location_in_parent: Vector3::new(0.0, -0.40, 0.0),
                    joint: vec![],
                },
            ],
            coordinates: vec![],
            muscles: vec![],
        }
    }
}
