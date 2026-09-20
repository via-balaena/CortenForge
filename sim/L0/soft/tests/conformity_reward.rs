#![allow(clippy::expect_used)]

//! The four conformity-reward terms, gated per
//! `docs/INSERTION_REWARD_WIRING_SPEC.md` §7.
//!
//! Each gate names the single property it isolates. Where a property is
//! asymptotic rather than exact, the gate says so and pins BOTH faces —
//! otherwise it cannot distinguish the weighted formulation from the
//! unweighted one, which is the thing it exists to detect.

use sim_soft::contact::{ContactPair, ContactPairReadout};
use sim_soft::readout::{ConformityParams, RewardWeights, conformity_breakdown};
use sim_soft::{Vec3, VertexId};

/// Ecoflex 00-30 tensile strength: 200 psi on the Smooth-On bulletin,
/// = 1 379 000 Pa. Taken from the data sheet, not from the study appendix,
/// which disagrees with the sheet on Dragon Skin 30A (spec §8.1).
const ECOFLEX_00_30_TENSILE_PA: f64 = 200.0 * 6894.757;

fn readout(pressure: f64, area: f64) -> ContactPairReadout {
    ContactPairReadout {
        pair: ContactPair::Vertex {
            vertex_id: 0 as VertexId,
            primitive_id: 0,
        },
        position: Vec3::zeros(),
        sd: -1e-4,
        normal: Vec3::new(0.0, 0.0, 1.0),
        force_on_soft: Vec3::new(0.0, 0.0, pressure * area),
        tributary_area: area,
        pressure,
    }
}

fn field(pressures: &[f64], area: f64) -> Vec<ContactPairReadout> {
    pressures.iter().map(|&p| readout(p, area)).collect()
}

/// Γ exactly covered by the given readouts — the fully-contacted case.
fn params_for(rs: &[ContactPairReadout]) -> ConformityParams {
    let a: f64 = rs.iter().map(|r| r.tributary_area).sum();
    ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, a)
}

// ---------------------------------------------------------------- uniformity

#[test]
fn uniform_pressure_scores_zero_uniformity_cost() {
    let p = ECOFLEX_00_30_TENSILE_PA * 0.2;
    let rs = field(&[p, p, p, p], 1e-4);
    let out = conformity_breakdown(&rs, &params_for(&rs));
    // R_unif = -J_unif, and J_unif = 0 at perfect uniformity.
    assert!(
        out.breakdown.pressure_uniformity.abs() < 1e-12,
        "uniform field must cost nothing, got {}",
        out.breakdown.pressure_uniformity,
    );
}

#[test]
fn uniformity_cost_grows_monotonically_with_spread() {
    let mid = ECOFLEX_00_30_TENSILE_PA * 0.2;
    let mut previous = f64::INFINITY;
    for spread in [0.0_f64, 0.1, 0.2, 0.4] {
        let d = mid * spread;
        let rs = field(&[mid - d, mid, mid, mid + d], 1e-4);
        let r = conformity_breakdown(&rs, &params_for(&rs))
            .breakdown
            .pressure_uniformity;
        assert!(
            r <= previous,
            "reward must not increase as spread grows: spread {spread} gave {r}, previous {previous}",
        );
        previous = r;
    }
    assert!(
        previous < -1e-6,
        "the widest spread must cost something real"
    );
}

#[test]
fn scale_invariance_is_asymptotic_not_exact() {
    // ⛔ The spec originally asserted EXACT invariance. That gate would reject
    // a correct implementation: `w(p) = logistic(beta_w (p - p_th))` reads
    // ABSOLUTE pressure, so doubling moves elements deeper into saturation.
    let p_th = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, 1.0).p_th;
    let shape = [1.0_f64, 2.0, 3.0, 4.0];

    // Saturated regime: every pressure far above the threshold -> exact.
    let base: Vec<f64> = shape.iter().map(|s| s * 10.0 * p_th).collect();
    let dbl: Vec<f64> = base.iter().map(|p| 2.0 * p).collect();
    let rs_b = field(&base, 1e-4);
    let rs_d = field(&dbl, 1e-4);
    let jb = conformity_breakdown(&rs_b, &params_for(&rs_b))
        .breakdown
        .pressure_uniformity;
    let jd = conformity_breakdown(&rs_d, &params_for(&rs_d))
        .breakdown
        .pressure_uniformity;
    assert!(
        (jb - jd).abs() < 1e-9,
        "saturated regime must be scale-invariant: {jb} vs {jd}",
    );

    // Near-threshold regime: the weights bite -> NOT invariant. Pinning this
    // face is what separates a weighted CoV from an unweighted one.
    let near: Vec<f64> = shape.iter().map(|s| s * 0.25 * p_th).collect();
    let near2: Vec<f64> = near.iter().map(|p| 2.0 * p).collect();
    let rs_n = field(&near, 1e-4);
    let rs_n2 = field(&near2, 1e-4);
    let jn = conformity_breakdown(&rs_n, &params_for(&rs_n))
        .breakdown
        .pressure_uniformity;
    let jn2 = conformity_breakdown(&rs_n2, &params_for(&rs_n2))
        .breakdown
        .pressure_uniformity;
    assert!(
        (jn - jn2).abs() > 1e-9,
        "near the threshold the weights must bite — an unweighted CoV would be \
         invariant here and this gate would not catch it: {jn} vs {jn2}",
    );
}

// ------------------------------------------------------------------ coverage

#[test]
fn coverage_spans_zero_to_one_and_is_strict_between() {
    let p_th = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, 1.0).p_th;

    let loaded = field(&[10.0 * p_th; 4], 1e-4);
    let all = conformity_breakdown(&loaded, &params_for(&loaded))
        .breakdown
        .coverage;
    assert!(
        (all - 1.0).abs() < 1e-6,
        "fully loaded Γ must score 1, got {all}"
    );

    let unloaded = field(&[0.0; 4], 1e-4);
    let none = conformity_breakdown(&unloaded, &params_for(&unloaded))
        .breakdown
        .coverage;
    assert!(none < 1e-6, "unloaded Γ must score ~0, got {none}");

    let mixed = field(&[10.0 * p_th, 10.0 * p_th, 0.0, 0.0], 1e-4);
    let half = conformity_breakdown(&mixed, &params_for(&mixed))
        .breakdown
        .coverage;
    assert!(
        half > 0.4 && half < 0.6,
        "half-loaded Γ must land near 0.5, got {half}",
    );
}

#[test]
fn coverage_falls_when_gamma_grows_with_the_loaded_set_fixed() {
    // The denominator is |Γ|, not the active area. Growing Γ while holding the
    // loaded elements fixed must LOWER coverage — this is the gate that
    // catches a denominator computed from the numerator, which the book warns
    // "would trivially score 1 regardless of extent".
    let p_th = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, 1.0).p_th;
    let loaded = field(&[10.0 * p_th; 4], 1e-4);
    let loaded_area: f64 = loaded.iter().map(|r| r.tributary_area).sum();

    let tight = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, loaded_area);
    let wide = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, 4.0 * loaded_area);

    let c_tight = conformity_breakdown(&loaded, &tight).breakdown.coverage;
    let c_wide = conformity_breakdown(&loaded, &wide).breakdown.coverage;
    assert!((c_tight - 1.0).abs() < 1e-6, "tight Γ: {c_tight}");
    assert!(
        (c_wide - 0.25).abs() < 1e-6,
        "Γ four times larger must quarter coverage, got {c_wide}",
    );
}

// ---------------------------------------------------------------------- peak

#[test]
fn smoothed_peak_never_exceeds_the_true_peak_and_reports_its_error() {
    // A narrow band: one loaded element in twenty. The spec measured 13-25%
    // underestimate at q=16 for this regime, and requires the ratio be
    // REPORTED rather than assumed away.
    let mut p = vec![ECOFLEX_00_30_TENSILE_PA * 0.01; 20];
    p[0] = ECOFLEX_00_30_TENSILE_PA * 0.30;
    let rs = field(&p, 1e-4);
    let out = conformity_breakdown(&rs, &params_for(&rs));

    assert!(
        out.p_peak_smoothed <= out.p_peak_true + 1e-9,
        "L^q max must not exceed the true max: {} vs {}",
        out.p_peak_smoothed,
        out.p_peak_true,
    );
    assert!(
        out.lq_ratio > 0.0 && out.lq_ratio <= 1.0,
        "lq_ratio must be reported in (0, 1], got {}",
        out.lq_ratio,
    );
    // The underestimate is real and must be visible, not silently absorbed.
    assert!(
        out.lq_ratio < 0.99,
        "a narrow band must show a measurable L^q underestimate, got ratio {}",
        out.lq_ratio,
    );
}

#[test]
fn peak_barrier_is_slack_far_from_the_ceiling_and_bites_near_it() {
    let params =
        |area: f64| ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, area);

    // Far below the ceiling: margin >= m_hat -> exactly zero penalty.
    let low = field(&[ECOFLEX_00_30_TENSILE_PA * 0.1; 4], 1e-4);
    let a: f64 = low.iter().map(|r| r.tributary_area).sum();
    let slack = conformity_breakdown(&low, &params(a)).breakdown.peak_bound;
    assert!(
        (slack - 0.0).abs() < 1e-12,
        "comfortably below the ceiling must cost exactly 0, got {slack}",
    );

    // Inside the barrier band: strictly negative, and more negative as the
    // peak climbs toward the ceiling.
    let mut previous = 0.0_f64;
    for frac in [0.90_f64, 0.95, 0.99] {
        let f = field(&[ECOFLEX_00_30_TENSILE_PA * frac; 4], 1e-4);
        let a: f64 = f.iter().map(|r| r.tributary_area).sum();
        let b = conformity_breakdown(&f, &params(a)).breakdown.peak_bound;
        assert!(
            b < 0.0,
            "inside the band the barrier must penalise, got {b} at {frac}"
        );
        assert!(
            b < previous,
            "penalty must deepen toward the ceiling: {b} at {frac}, previous {previous}",
        );
        previous = b;
    }
}

// ---------------------------------------------- stiffness, degenerate, compose

#[test]
fn stiffness_bound_is_nan_and_score_with_drops_it() {
    let rs = field(&[ECOFLEX_00_30_TENSILE_PA * 0.2; 4], 1e-4);
    let out = conformity_breakdown(&rs, &params_for(&rs));
    assert!(
        out.breakdown.stiffness_bound.is_nan(),
        "k_min has no source; the term must be NaN, not an invented floor",
    );

    let w = RewardWeights {
        pressure_uniformity: 0.25,
        coverage: 0.25,
        peak_bound: 0.25,
        stiffness_bound: 0.25,
    };
    let score = out.breakdown.score_with(&w);
    assert!(
        score.is_finite(),
        "a NaN term must be dropped, not poison the sum"
    );

    // Hand-computed from the three live terms.
    let expect = 0.25 * out.breakdown.pressure_uniformity
        + 0.25 * out.breakdown.coverage
        + 0.25 * out.breakdown.peak_bound;
    assert!(
        (score - expect).abs() < 1e-12,
        "composition must equal the hand-summed weighted value: {score} vs {expect}",
    );
}

#[test]
fn degenerate_contact_reports_nan_rather_than_a_flattering_zero() {
    let empty: Vec<ContactPairReadout> = Vec::new();
    let p = ConformityParams::from_tensile_strength(ECOFLEX_00_30_TENSILE_PA, 1e-3);
    let out = conformity_breakdown(&empty, &p);
    assert!(
        out.breakdown.pressure_uniformity.is_nan(),
        "no contact: uniformity undefined"
    );
    assert!(
        out.breakdown.coverage.abs() < 1e-12,
        "no contact: coverage is a real 0"
    );

    let zeroed = field(&[0.0; 4], 1e-4);
    let out0 = conformity_breakdown(&zeroed, &params_for(&zeroed));
    assert!(
        out0.breakdown.pressure_uniformity.is_nan(),
        "all-zero pressure: the weighted mean vanishes, uniformity undefined",
    );
}

#[test]
fn non_finite_pressures_are_counted_as_defects_not_silently_zeroed() {
    // Before rung 8d a Tet10 midside node had tributary_area = 0 and so NaN
    // pressure. Treating that as unloaded would hide a mis-reported force
    // distribution.
    let mut rs = field(&[ECOFLEX_00_30_TENSILE_PA * 0.2; 3], 1e-4);
    rs.push(readout(f64::NAN, 1e-4));
    rs.push(readout(1.0, f64::NAN));
    let out = conformity_breakdown(&rs, &params_for(&rs));
    assert_eq!(
        out.non_finite_pressures, 2,
        "both malformed readouts must be counted",
    );
}
