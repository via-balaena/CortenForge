//! The A3 **body scorecard** — "is this dialed/generated body valid?" in one place.
//!
//! A3-PR1–PR3 proved a scan-free, dial-able thigh–knee–shank–ankle twin, but the
//! validation that proves it lives scattered across three test files. This module
//! consolidates it into ONE grader over a *generated* body (a `template` + the
//! [`BodyParams`] that morph it), the way `cf-msk-fit`'s `scorecard` did for the
//! *placed/scanned* G1 knee. It needs only `cf_msk_lib`'s morph and this crate's
//! oracle — no scan, no `Fitter` — so it lives here, the one crate that owns both
//! the oracle and the real-OpenSim references.
//!
//! ## The tier-applicability matrix (the keystone — read first)
//!
//! The recon's validation pyramid has three tiers, but they are **not** symmetric
//! in cost or per-body applicability. The honest per-body grade is **Tier 2
//! (exact) + Tier 3 (per-body oracle) + Tier 1 as *coverage***:
//!
//! | Tier | Grades | Per body? | Can claim | Cannot claim |
//! |------|--------|-----------|-----------|--------------|
//! | **T1 differential oracle** | the morph *operator* (`realize` vs OpenSim ScaleTool) | **No** — one-time over a scale grid | "per-axis morphing reproduces ScaleTool to ~0.3 mm *within the validated factor envelope*" | a body's parameter *choice*; factors outside the grid |
//! | **T1-coverage** (here, per body) | is this body's scale config inside the T1 envelope? | **Yes** (free) | "in-envelope (interpolated, OpenSim-graded region)" / "extrapolated" | that an extrapolated body is *wrong* — only that T1 didn't cover it |
//! | **T2 internal consistency** | the `realize()` result for *this* body | **Yes, exact** | "lengths/girths realize exactly; deterministic" | plausibility or OpenSim agreement |
//! | **T3 plausibility + shape-corr** | this body's realized proportions + curve shape vs canonical | **Yes** (oracle FK) | "proportions physiological; curve shape correlated (≥0.95 coupled / reported tail)" | **personhood** |
//!
//! So T1 enters a per-body card not as a recomputed RMSE (we do **not** re-run
//! OpenSim per body) but as (a) the grid result **cited** as a standing machinery
//! guarantee and (b) a cheap [`T1Coverage`] envelope check. The honesty rule
//! carried from the recon: **T1+T2 prove the machinery is correct and matches
//! OpenSim's scaling; T3 proves plausibility, not personhood.** A dialed body is
//! "a clone of someone with these proportions," never a validated individual.
//!
//! ## The extreme-decoupled tail: report, don't hard-gate
//!
//! A genuinely tall-lean person *has* a different hamstring moment-arm curve shape,
//! so the ~0.87 shape-corr of an extreme stature/girth-decoupled build is the morph
//! faithfully expressing a real anisotropy — not a bug. Hard-gating every body at
//! ≥0.95 would dishonestly reject valid extreme bodies. So the scorecard **gates
//! ≥0.95 only for the coupled regime** (girth tracks stature) and **reports** the
//! decoupled tail against a loose ≥0.80 sanity floor. See [`Regime`].

use crate::oracle::{Kinematics, Pose};
use cf_msk_lib::{BodyParams, Model, SegmentScale, realize};
use std::collections::HashMap;
use std::f64::consts::PI;

/// The standing Tier-1 guarantee this scorecard cites (never recomputes per body):
/// `morph_matches_real_opensim_scaletool` grades `realize` against OpenSim's own
/// ScaleTool over the differential-oracle grid and holds to a measured worst
/// ~0.37 mm under an 0.8 mm gate. A per-body card only checks whether the body's
/// factors fall *inside* that grid's envelope ([`T1Coverage`]).
pub const T1_CITATION: &str = "differential oracle (morph_matches_real_opensim_scaletool): realize == OpenSim ScaleTool \
     to worst ~0.37 mm over the grid (0.8 mm gate)";

const EPS: f64 = 0.5 * PI / 180.0;

// --- Tier 1: differential-oracle coverage ------------------------------------

/// An inclusive `[min, max]` factor range.
#[derive(Debug, Clone, Copy)]
struct Range {
    min: f64,
    max: f64,
}

impl Range {
    fn point(v: f64) -> Self {
        Range { min: v, max: v }
    }
    fn extend(&mut self, v: f64) {
        self.min = self.min.min(v);
        self.max = self.max.max(v);
    }
    /// Inside with a tiny epsilon for float-equality at the bounds.
    fn contains(&self, v: f64) -> bool {
        v >= self.min - 1e-9 && v <= self.max + 1e-9
    }
}

/// The per-axis factor envelope the differential-oracle grid actually exercised,
/// built from the vendored `scaled_moment_arms_opensim.json`. A body whose per-axis
/// factors all fall inside is *interpolated* within the OpenSim-graded region;
/// outside, the Tier-1 guarantee is *extrapolated* (honestly flagged, not failed).
///
/// **Caveat (kept modest):** the grid is sparse — single-axis perturbations + a
/// realistic mix + the generator's coupled extremes — so "in-envelope" means each
/// factor is within the per-axis range tested, with the multi-axis `realistic_mix`
/// / `gen_*` configs the directly-graded combined points. It is a coverage claim,
/// not a dense hypercube proof.
#[derive(Debug, Clone)]
pub struct DiffOracleEnvelope {
    /// body name → (axial=y range, transverse=x=z range).
    ranges: HashMap<String, (Range, Range)>,
}

impl DiffOracleEnvelope {
    /// Build from the parsed `scaled_moment_arms_opensim.json` (path-free — the
    /// caller reads the vendored asset and hands over the value). Reads every
    /// config's `factors` (per-body `[x, y, z]`, y axial / x=z transverse) and
    /// unions them per body. Identity (1.0) is always included so an unscaled
    /// segment is trivially in-envelope.
    pub fn from_grid_json(grid: &serde_json::Value) -> Self {
        let mut ranges: HashMap<String, (Range, Range)> = HashMap::new();
        let configs = grid["configs"]
            .as_object()
            .expect("scaled_moment_arms_opensim.json: missing `configs` object");
        for cfg in configs.values() {
            let factors = cfg["factors"]
                .as_object()
                .expect("config: missing `factors` object");
            for (body, v) in factors {
                let a = v.as_array().expect("factor must be a [x,y,z] array");
                let (x, y) = (
                    a[0].as_f64().expect("factor x"),
                    a[1].as_f64().expect("factor y"),
                );
                let e = ranges
                    .entry(body.clone())
                    .or_insert((Range::point(1.0), Range::point(1.0)));
                e.0.extend(y); // axial = y
                e.1.extend(x); // transverse = x (= z)
            }
        }
        DiffOracleEnvelope { ranges }
    }

    /// The range for a body, or identity-only if the grid never scaled it.
    fn range(&self, body: &str) -> (Range, Range) {
        self.ranges
            .get(body)
            .copied()
            .unwrap_or((Range::point(1.0), Range::point(1.0)))
    }

    /// Check a body's factors against the envelope, returning per-factor flags for
    /// anything that falls outside (empty ⇒ fully in-envelope).
    fn check(&self, params: &BodyParams) -> Vec<String> {
        let mut out = Vec::new();
        let mut one = |body: &str, scale: SegmentScale| {
            let (ax, tr) = self.range(body);
            if !ax.contains(scale.axial) {
                out.push(format!(
                    "{body} axial {:.3} outside grid [{:.3},{:.3}]",
                    scale.axial, ax.min, ax.max
                ));
            }
            if !tr.contains(scale.transverse) {
                out.push(format!(
                    "{body} transverse {:.3} outside grid [{:.3},{:.3}]",
                    scale.transverse, tr.min, tr.max
                ));
            }
        };
        one("pelvis", params.pelvis);
        one("femur_r", params.femur);
        one("tibia_r", params.tibia);
        out
    }
}

/// Tier-1 coverage for one body: does its morph fall inside the differential-oracle
/// envelope? Carries the standing-guarantee citation either way.
#[derive(Debug, Clone)]
pub struct T1Coverage {
    pub in_envelope: bool,
    /// Per-factor flags for anything extrapolated beyond the grid (empty ⇒ inside).
    pub extrapolated: Vec<String>,
    pub cited: &'static str,
}

// --- Tier 2: internal consistency (exact) ------------------------------------

/// Tier-2 internal consistency for one body — exact, computed directly (no oracle,
/// no OpenSim). The morph guarantee: a segment's realized axial length equals the
/// template's times the dialed axial factor, exactly; and `realize` is deterministic.
#[derive(Debug, Clone, Copy)]
pub struct T2Consistency {
    /// `|realized_femur_axial − template_femur_axial · femur.axial|` (m).
    pub femur_axial_residual_m: f64,
    /// `|realized_tibia_axial − template_tibia_axial · tibia.axial|` (m).
    pub tibia_axial_residual_m: f64,
    /// Re-running `realize` reproduces the same measurable geometry.
    pub deterministic: bool,
}

impl T2Consistency {
    /// Exact to within floating-point round-off.
    pub fn exact(&self) -> bool {
        self.femur_axial_residual_m < 1e-9
            && self.tibia_axial_residual_m < 1e-9
            && self.deterministic
    }
}

// --- Tier 3: plausibility + shape-corr (per-body oracle) ----------------------

/// How the body's length and girth dials relate — selects the shape-corr policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Regime {
    /// Girth tracks stature (the `AnthroSource::new` default): the validated regime,
    /// gated at shape-corr ≥ 0.95.
    Coupled,
    /// Stature and girth dialed apart (tall+lean / short+stocky): the documented
    /// boundary, *reported* against a loose ≥ 0.80 sanity floor, not gated at 0.95.
    Decoupled,
    /// Provenance unknown — treated conservatively as [`Regime::Decoupled`] (the
    /// loose floor) so an unlabelled body is never silently held to the tight gate.
    Unknown,
}

impl Regime {
    /// The shape-corr floor this regime is held to.
    pub fn corr_floor(self) -> f64 {
        match self {
            Regime::Coupled => 0.95,
            Regime::Decoupled | Regime::Unknown => 0.80,
        }
    }

    /// Infer the regime from the morph alone (when there is no generator
    /// provenance). **Decoupled** when, on either limb segment, length and girth
    /// pull in *opposite* directions by more than a small tolerance (the tall+lean /
    /// short+stocky signature: axial > 1 with transverse < 1, or vice versa). The
    /// coupled family (girth tracks stature) always moves both the same way.
    pub fn infer(params: &BodyParams) -> Regime {
        const TOL: f64 = 0.03;
        let opposed = |s: SegmentScale| {
            let (da, dt) = (s.axial - 1.0, s.transverse - 1.0);
            da.abs() > TOL && dt.abs() > TOL && da.signum() != dt.signum()
        };
        if opposed(params.femur) || opposed(params.tibia) {
            Regime::Decoupled
        } else {
            Regime::Coupled
        }
    }
}

/// Shape correlation of one muscle's knee moment-arm curve vs the canonical body.
#[derive(Debug, Clone)]
pub struct MuscleCorr {
    pub muscle: String,
    pub corr: f64,
}

/// Tier-3 plausibility + shape-correlation for one body (uses the oracle FK).
#[derive(Debug, Clone)]
pub struct T3Plausibility {
    pub femur_axial_m: f64,
    pub tibia_axial_m: f64,
    /// Realized femur/tibia axial lengths within physiological bounds.
    pub lengths_plausible: bool,
    pub shape_corr: Vec<MuscleCorr>,
    /// The worst (minimum) per-muscle shape correlation vs canonical.
    pub worst_corr: f64,
    pub regime: Regime,
}

impl T3Plausibility {
    /// Plausible lengths AND shape-corr above the regime's floor (≥0.95 coupled,
    /// ≥0.80 reported tail).
    pub fn passes(&self) -> bool {
        self.lengths_plausible && self.worst_corr >= self.regime.corr_floor()
    }
}

/// Physiological bounds for the realized segment axial lengths (m), matching the
/// recon's plausibility checks on the real gait2392 template across percentiles.
const FEMUR_BOUNDS_M: (f64, f64) = (0.34, 0.55);
const TIBIA_BOUNDS_M: (f64, f64) = (0.32, 0.52);

// --- The body scorecard + grader ---------------------------------------------

/// The consolidated three-tier card for one generated body.
#[derive(Debug, Clone)]
pub struct BodyScorecard {
    pub label: String,
    pub t1: T1Coverage,
    pub t2: T2Consistency,
    pub t3: T3Plausibility,
}

impl BodyScorecard {
    /// The honest overall verdict: T2 exact AND T3 passes (plausible + regime
    /// shape-corr floor). **T1 coverage is reported, not gated** — an extrapolated
    /// body is flagged, not failed (the morph machinery is still cited; T1 simply
    /// did not directly grade that config). T2+T3 are the per-body correctness gate.
    pub fn passes(&self) -> bool {
        self.t2.exact() && self.t3.passes()
    }
}

/// A reusable grader: holds the template, the differential-oracle envelope, and the
/// canonical (identity-morph) moment-arm curves once, then grades any number of
/// bodies against them.
pub struct Scorecard<'a> {
    template: &'a Model,
    env: DiffOracleEnvelope,
    angles: Vec<f64>,
    /// Canonical curves (muscle name, curve over `angles`), the identity morph.
    canonical: Vec<(String, Vec<f64>)>,
}

impl<'a> Scorecard<'a> {
    /// Build the grader. `env` is the differential-oracle envelope
    /// ([`DiffOracleEnvelope::from_grid_json`]); the canonical curves are the
    /// template under the identity morph (the reference every body is compared to).
    pub fn new(template: &'a Model, env: DiffOracleEnvelope) -> Self {
        // Knee 0 → −100° in 5° steps — the same ROM the differential oracle and the
        // anthro validation use.
        let angles: Vec<f64> = (0..=20)
            .map(|i| -(f64::from(i)) * 5.0 * PI / 180.0)
            .collect();
        let canonical_model = realize(template, &BodyParams::IDENTITY);
        let canonical = Self::curves(&canonical_model, &angles);
        Scorecard {
            template,
            env,
            angles,
            canonical,
        }
    }

    /// Knee moment-arm curve (mm) per muscle for a model, over `angles`.
    fn curves(model: &Model, angles: &[f64]) -> Vec<(String, Vec<f64>)> {
        let kin = Kinematics::new(model);
        model
            .muscles
            .iter()
            .map(|m| {
                let c = angles
                    .iter()
                    .map(|&th| {
                        let q = Pose::from([("knee_angle_r".to_string(), th)]);
                        kin.moment_arm(m, &q, "knee_angle_r", EPS) * 1000.0
                    })
                    .collect();
                (m.name.clone(), c)
            })
            .collect()
    }

    /// Grade one body: `params` morph the template; `regime` selects the shape-corr
    /// policy (pass [`Regime::infer`] if there is no generator provenance).
    pub fn grade(&self, label: &str, params: &BodyParams, regime: Regime) -> BodyScorecard {
        let model = realize(self.template, params);

        // T1 — coverage.
        let extrapolated = self.env.check(params);
        let t1 = T1Coverage {
            in_envelope: extrapolated.is_empty(),
            extrapolated,
            cited: T1_CITATION,
        };

        // T2 — exact axial-length scaling + determinism.
        let femur_axial = model.segment_axial_length("femur_r", "tibia_r");
        let tibia_axial = model.segment_axial_length("tibia_r", "talus_r");
        let femur_expected =
            self.template.segment_axial_length("femur_r", "tibia_r") * params.femur.axial;
        let tibia_expected =
            self.template.segment_axial_length("tibia_r", "talus_r") * params.tibia.axial;
        let again = realize(self.template, params);
        let deterministic = (again.segment_axial_length("femur_r", "tibia_r") - femur_axial).abs()
            < 1e-15
            && (again.segment_axial_length("tibia_r", "talus_r") - tibia_axial).abs() < 1e-15;
        let t2 = T2Consistency {
            femur_axial_residual_m: (femur_axial - femur_expected).abs(),
            tibia_axial_residual_m: (tibia_axial - tibia_expected).abs(),
            deterministic,
        };

        // T3 — plausibility + shape-corr vs canonical.
        let lengths_plausible = (FEMUR_BOUNDS_M.0..=FEMUR_BOUNDS_M.1).contains(&femur_axial)
            && (TIBIA_BOUNDS_M.0..=TIBIA_BOUNDS_M.1).contains(&tibia_axial);
        let body_curves = Self::curves(&model, &self.angles);
        let mut shape_corr = Vec::new();
        let mut worst_corr = 1.0_f64;
        for ((name, base), (_, d)) in self.canonical.iter().zip(&body_curves) {
            let corr = pearson(base, d);
            worst_corr = worst_corr.min(corr);
            shape_corr.push(MuscleCorr {
                muscle: name.clone(),
                corr,
            });
        }
        let t3 = T3Plausibility {
            femur_axial_m: femur_axial,
            tibia_axial_m: tibia_axial,
            lengths_plausible,
            shape_corr,
            worst_corr,
            regime,
        };

        BodyScorecard {
            label: label.to_string(),
            t1,
            t2,
            t3,
        }
    }

    /// Grade a whole population (the API A4's `RandomizerSource` feeds). Each entry
    /// is `(label, params, regime)`; returns the per-body cards plus the aggregate.
    pub fn grade_population(&self, bodies: &[(String, BodyParams, Regime)]) -> PopulationScorecard {
        let cards: Vec<BodyScorecard> = bodies
            .iter()
            .map(|(label, params, regime)| self.grade(label, params, *regime))
            .collect();
        PopulationScorecard::from_cards(cards)
    }
}

/// Pearson correlation of two equal-length curves (shape agreement).
fn pearson(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len() as f64;
    let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
    let (mut sab, mut saa, mut sbb) = (0.0, 0.0, 0.0);
    for (&x, &y) in a.iter().zip(b) {
        sab += (x - ma) * (y - mb);
        saa += (x - ma).powi(2);
        sbb += (y - mb).powi(2);
    }
    if saa > 1e-18 && sbb > 1e-18 {
        sab / (saa.sqrt() * sbb.sqrt())
    } else {
        // A flat canonical/body curve has no shape to correlate; treat matching-flat
        // as perfect, else uncorrelated.
        if (saa - sbb).abs() < 1e-18 { 1.0 } else { 0.0 }
    }
}

// --- Population aggregate ------------------------------------------------------

/// The aggregate grade over a population of generated bodies — coverage of the
/// parameter space and the machinery, **not** personhood.
#[derive(Debug, Clone)]
pub struct PopulationScorecard {
    pub n: usize,
    pub n_in_envelope: usize,
    pub n_lengths_plausible: usize,
    pub n_coupled: usize,
    pub n_decoupled: usize,
    /// Worst shape-corr among the *coupled* bodies (the gated regime); 1.0 if none.
    pub worst_corr_coupled: f64,
    /// Worst shape-corr among the *decoupled/unknown* bodies (reported); 1.0 if none.
    pub worst_corr_decoupled: f64,
    /// Median per-body worst-muscle shape-corr across the whole population.
    pub median_worst_corr: f64,
    /// Every body's Tier-2 morph is exact.
    pub all_t2_exact: bool,
    pub cards: Vec<BodyScorecard>,
}

impl PopulationScorecard {
    fn from_cards(cards: Vec<BodyScorecard>) -> Self {
        let n = cards.len();
        let mut worst_corr_coupled = 1.0_f64;
        let mut worst_corr_decoupled = 1.0_f64;
        let (mut n_in_envelope, mut n_lengths_plausible, mut n_coupled, mut n_decoupled) =
            (0, 0, 0, 0);
        let mut all_t2_exact = true;
        let mut worsts: Vec<f64> = Vec::with_capacity(n);
        for c in &cards {
            if c.t1.in_envelope {
                n_in_envelope += 1;
            }
            if c.t3.lengths_plausible {
                n_lengths_plausible += 1;
            }
            if !c.t2.exact() {
                all_t2_exact = false;
            }
            worsts.push(c.t3.worst_corr);
            match c.t3.regime {
                Regime::Coupled => {
                    n_coupled += 1;
                    worst_corr_coupled = worst_corr_coupled.min(c.t3.worst_corr);
                }
                Regime::Decoupled | Regime::Unknown => {
                    n_decoupled += 1;
                    worst_corr_decoupled = worst_corr_decoupled.min(c.t3.worst_corr);
                }
            }
        }
        worsts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median_worst_corr = if worsts.is_empty() {
            1.0
        } else {
            worsts[worsts.len() / 2]
        };
        PopulationScorecard {
            n,
            n_in_envelope,
            n_lengths_plausible,
            n_coupled,
            n_decoupled,
            worst_corr_coupled,
            worst_corr_decoupled,
            median_worst_corr,
            all_t2_exact,
            cards,
        }
    }

    /// The population gate: every Tier-2 morph exact, every body plausible, and the
    /// **coupled** bodies clear the ≥0.95 shape-corr bar. The decoupled tail is
    /// reported (`worst_corr_decoupled`), not gated here.
    pub fn passes(&self) -> bool {
        self.all_t2_exact && self.n_lengths_plausible == self.n && self.worst_corr_coupled >= 0.95
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cf_msk_lib::ParamSource;
    use cf_msk_lib::anthro::{AnthroSource, Sex};

    fn template() -> Model {
        let path = format!(
            "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
            env!("CARGO_MANIFEST_DIR")
        );
        crate::parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"))
    }

    fn envelope() -> DiffOracleEnvelope {
        let path = format!(
            "{}/../../sim/L0/tests/assets/opensim_gait2392/scaled_moment_arms_opensim.json",
            env!("CARGO_MANIFEST_DIR")
        );
        let v: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        DiffOracleEnvelope::from_grid_json(&v)
    }

    #[test]
    fn regime_infer_separates_coupled_from_decoupled() {
        // Coupled: girth tracks stature (both > 1).
        let coupled = AnthroSource::new(Sex::Male, 0.9).params(&template());
        assert_eq!(Regime::infer(&coupled), Regime::Coupled);
        // Decoupled: tall + lean (axial > 1, transverse < 1).
        let lean = AnthroSource::new(Sex::Male, 0.9)
            .with_girth_percentile(0.05)
            .params(&template());
        assert_eq!(Regime::infer(&lean), Regime::Decoupled);
    }

    #[test]
    fn canonical_body_is_exact_in_envelope_and_perfectly_correlated() {
        let t = template();
        let card = Scorecard::new(&t, envelope()).grade(
            "canonical",
            &BodyParams::IDENTITY,
            Regime::Coupled,
        );
        assert!(card.t1.in_envelope, "identity must be in-envelope");
        assert!(card.t2.exact());
        assert!(
            (card.t3.worst_corr - 1.0).abs() < 1e-9,
            "identity vs itself = 1"
        );
        assert!(card.passes());
    }

    #[test]
    fn coupled_family_passes_decoupled_extreme_reported() {
        let t = template();
        let sc = Scorecard::new(&t, envelope());
        // A coupled body clears the tight gate.
        let coupled = AnthroSource::new(Sex::Female, 0.95);
        let cc = sc.grade("f95", &coupled.params(&t), Regime::Coupled);
        assert!(
            cc.t3.worst_corr >= 0.95,
            "coupled corr {}",
            cc.t3.worst_corr
        );
        assert!(cc.passes());
        // An extreme decoupled body dips below 0.95 but clears the loose floor and
        // is NOT failed by the (reported) decoupled policy.
        let extreme = AnthroSource::new(Sex::Male, 0.90).with_girth_percentile(0.05);
        let ec = sc.grade("decoupled", &extreme.params(&t), Regime::Decoupled);
        assert!(ec.t3.worst_corr < 0.95, "expected tail < 0.95");
        assert!(
            ec.t3.worst_corr >= 0.80,
            "tail below floor: {}",
            ec.t3.worst_corr
        );
        assert!(ec.passes(), "decoupled body should pass its loose floor");
    }

    #[test]
    fn out_of_envelope_factor_is_flagged_not_failed() {
        let t = template();
        // A wildly long femur — far outside the grid's axial range.
        let params = BodyParams {
            femur: SegmentScale {
                axial: 5.0,
                transverse: 1.0,
            },
            ..BodyParams::IDENTITY
        };
        let card = Scorecard::new(&t, envelope()).grade("giant", &params, Regime::Coupled);
        assert!(!card.t1.in_envelope);
        assert!(!card.t1.extrapolated.is_empty());
        // T2 is still exact (the morph machinery does not care about the envelope).
        assert!(card.t2.exact());
    }

    #[test]
    fn population_aggregate_counts_and_gate() {
        let t = template();
        let sc = Scorecard::new(&t, envelope());
        let bodies = vec![
            (
                "m50".to_string(),
                AnthroSource::new(Sex::Male, 0.5).params(&t),
                Regime::Coupled,
            ),
            (
                "f50".to_string(),
                AnthroSource::new(Sex::Female, 0.5).params(&t),
                Regime::Coupled,
            ),
            (
                "tall_lean".to_string(),
                AnthroSource::new(Sex::Male, 0.9)
                    .with_girth_percentile(0.1)
                    .params(&t),
                Regime::Decoupled,
            ),
        ];
        let pop = sc.grade_population(&bodies);
        assert_eq!(pop.n, 3);
        assert_eq!(pop.n_coupled, 2);
        assert_eq!(pop.n_decoupled, 1);
        assert!(pop.all_t2_exact);
        assert!(
            pop.passes(),
            "coupled members + plausibility must gate green"
        );
    }
}
