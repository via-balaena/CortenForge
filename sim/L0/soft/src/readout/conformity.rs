//! Conformity reward — the four terms of Part 1 Ch 01, computed from contact.
//!
//! Spec: `docs/INSERTION_REWARD_WIRING_SPEC.md`. This module implements §5 of
//! that document; the constants and their provenance are §6, and the
//! deviations from the book are §8.5.
//!
//! # What this computes
//!
//! Given per-element contact pressure and tributary area restricted to the
//! **intended contact surface** Γ, it produces a [`RewardBreakdown`]:
//!
//! | term | quantity | sense |
//! |---|---|---|
//! | `pressure_uniformity` | `−J_unif`, weighted squared `CoV` | higher better, 0 is perfect |
//! | `coverage` | `+J_cov` ∈ [0, 1] | higher better |
//! | `peak_bound` | `−B_peak / m̂²`, normalised inverted IPC barrier | higher better, 0 is slack |
//! | `stiffness_bound` | `NaN` — see below | dropped by `score_with` |
//!
//! # ⛔ `stiffness_bound` is `NaN` on purpose
//!
//! The stiffness barrier needs `k_min`, which the book calls "material- and
//! application-dependent" and which **no source in this repository provides**.
//! Unlike `p_max` it has no material basis — it is an application requirement
//! ("the sleeve must transmit at least X N per mm of engagement") that does not
//! exist yet. Scoring against an invented floor would bury an unfalsifiable
//! number inside the function that defines "good", so the term reports `NaN`
//! and [`RewardBreakdown::score_with`] drops it. Three terms, honestly.
//!
//! # ⚠ Γ is the caller's responsibility
//!
//! This module takes readouts **already restricted to Γ** plus `|Γ|`. It cannot
//! derive Γ itself: the intended contact surface is a property of the scene's
//! geometry (for an insertion cavity, the scan isosurface at the cavity
//! offset), not of the contact readouts. Passing the full boundary would cap
//! coverage structurally below 1, because faces that can never contact would
//! sit in the denominator.

use crate::contact::ContactPairReadout;
use crate::readout::RewardBreakdown;

/// ⛔ **FREE PARAMETER — chosen, not measured.**
///
/// The book places the pressure threshold "at a small fraction of the
/// material's tensile strength" and gives no number. 5 % is this
/// implementation's choice.
///
/// **What would replace it:** the pressure below which contact is not
/// mechanically meaningful for the application — measurable on a cast part as
/// the pressure at which the sleeve stops transmitting perceptible load.
pub const P_TH_FRACTION_OF_TENSILE: f64 = 0.05;

/// ⛔ **FREE PARAMETER — chosen, not measured.**
///
/// The book says only that `β_c` is "typically somewhat larger (steeper)" than
/// `β_w`, because coverage cares about the transition's location while
/// uniformity's weight acts as a mean-filter mask. 2× is this implementation's
/// reading of "somewhat".
pub const BETA_C_OVER_BETA_W: f64 = 2.0;

/// Barrier engagement margin as a fraction of the ceiling.
///
/// **Sourced**, not free: the book sets `m̂` at "a fraction of the ceiling
/// (typical: 10–25 % of `p_max`)" so the barrier's active region stays
/// scale-consistent across materials. 15 % is the mid-range.
pub const M_HAT_FRACTION_OF_P_MAX: f64 = 0.15;

/// Order of the `L^q` smoothed maximum.
///
/// **Sourced** as a range — the book gives "typical `q ∈ [8, 16]`". The top of
/// the range is taken because the approximation error grows sharply as the
/// peak's area fraction shrinks, and an insertion contact is a narrow band.
///
/// ⚠ Even at `q = 16` the smoothed max reads **13–25 % below** the true max for
/// a 1–10 % peak area fraction, and that error is *under-protective*. It is
/// reported as [`ConformityReadout::lq_ratio`] rather than assumed away.
pub const LQ_ORDER: i32 = 16;

/// Sharpness derived from the threshold, per the book's saturation condition.
///
/// "`β_w` chosen so `w` saturates within ±`p_th`/2 of the threshold". Reading
/// "saturates" as *within 1 % of the asymptote* gives `logistic(±4.6) ≈
/// 0.99/0.01`, hence `β_w · p_th/2 = 4.6`.
///
/// ⚠ The 1 % reading is this implementation's, not the book's — the free
/// choice inside a derived constant.
#[must_use]
pub fn beta_w_from_threshold(p_th: f64) -> f64 {
    const SATURATION_LOGIT: f64 = 4.6;
    2.0 * SATURATION_LOGIT / p_th
}

/// Numerically stable logistic. Guards the `exp` rather than relying on `f64`
/// saturation: `β_w` scales as `1/p_th`, so a small threshold makes the
/// argument large.
fn logistic(u: f64) -> f64 {
    if u < -700.0 {
        0.0
    } else if u > 700.0 {
        1.0
    } else {
        1.0 / (1.0 + (-u).exp())
    }
}

/// Everything the conformity reward needs that the contact readouts do not
/// carry.
///
/// Construct via [`ConformityParams::from_tensile_strength`] to get the
/// documented derivations rather than assembling the fields by hand.
#[derive(Clone, Copy, Debug)]
pub struct ConformityParams {
    /// Pressure threshold (Pa) — shared by uniformity's weight and coverage's
    /// indicator **by construction**, so the two terms agree on what counts as
    /// actively contacted.
    pub p_th: f64,
    /// Uniformity weight sharpness (1/Pa).
    pub beta_w: f64,
    /// Coverage indicator sharpness (1/Pa).
    pub beta_c: f64,
    /// Peak-pressure ceiling (Pa), read from the material's tensile strength.
    pub p_max: f64,
    /// Barrier engagement margin (Pa).
    pub m_hat: f64,
    /// `L^q` order for the smoothed maximum.
    pub q: i32,
    /// Total area of the intended contact surface Γ (m²) — the denominator
    /// coverage is measured against.
    pub gamma_area: f64,
}

impl ConformityParams {
    /// Build from a material's tensile strength and the intended-surface area,
    /// applying every derivation documented on the constants above.
    #[must_use]
    pub fn from_tensile_strength(tensile_pa: f64, gamma_area: f64) -> Self {
        let p_th = P_TH_FRACTION_OF_TENSILE * tensile_pa;
        let beta_w = beta_w_from_threshold(p_th);
        Self {
            p_th,
            beta_w,
            beta_c: BETA_C_OVER_BETA_W * beta_w,
            p_max: tensile_pa,
            m_hat: M_HAT_FRACTION_OF_P_MAX * tensile_pa,
            q: LQ_ORDER,
            gamma_area,
        }
    }
}

/// The breakdown plus the diagnostics needed to judge whether it can be
/// trusted.
#[derive(Clone, Debug)]
pub struct ConformityReadout {
    /// The four reward terms, ready for [`RewardBreakdown::score_with`].
    pub breakdown: RewardBreakdown,
    /// `L^q` smoothed maximum pressure (Pa).
    pub p_peak_smoothed: f64,
    /// True maximum pressure over Γ (Pa).
    pub p_peak_true: f64,
    /// `p_peak_smoothed / p_peak_true` ∈ (0, 1].
    ///
    /// ⛔ **Read this before trusting `peak_bound`.** The `L^q` max
    /// underestimates by an amount that depends on the peak's *area fraction*,
    /// not just on `q` — 13–25 % at `q = 16` for a narrow contact band. An
    /// underestimated peak inflates the margin to the ceiling, so the barrier
    /// reports slack that does not exist.
    pub lq_ratio: f64,
    /// Contact pairs in Γ that carried a non-finite pressure.
    ///
    /// ⚠ A defect, never a zero. On a Tet10 mesh before rung 8d the loaded
    /// midside nodes had `tributary_area = 0` and so `NaN` pressure; any
    /// non-zero count here means the readout path is mis-reporting the force
    /// distribution, not that those elements are unloaded.
    pub non_finite_pressures: usize,
}

/// Inverted IPC barrier, **normalised by `m̂²`** so the term is dimensionless.
///
/// ⚠ **Deviation from the book, deliberate** (spec §8.5). The unnormalised
/// barrier carries units of the margin squared, making it O(1e5–1e8) at
/// realistic tolerances while uniformity and coverage are O(1) — under the
/// canonical equal weights it would swamp them. Dividing by `m̂²` preserves the
/// `C²` shape, the zero at `m = m̂` and the divergence at `m → 0⁺`, and puts the
/// term on the same footing so the equal weights mean what they say.
fn normalised_barrier(margin: f64, m_hat: f64) -> f64 {
    if margin <= 0.0 {
        return f64::INFINITY;
    }
    if margin >= m_hat {
        return 0.0;
    }
    let d = margin - m_hat;
    -(d * d) * (margin / m_hat).ln() / (m_hat * m_hat)
}

/// `L^q` smoothed maximum, computed in **normalised form**.
///
/// `max(p) · (Σ (pᵢ/max(p))^q aᵢ / A)^(1/q)` rather than `(Σ pᵢ^q aᵢ / A)^(1/q)`
/// — bit-equal across the realistic pressure range and free insurance against
/// overflow at `q = 16`.
fn lq_smoothed_max(pressures: &[f64], areas: &[f64], gamma_area: f64, q: i32) -> f64 {
    let peak = pressures.iter().copied().fold(0.0_f64, f64::max);
    if peak <= 0.0 || gamma_area <= 0.0 {
        return 0.0;
    }
    let acc: f64 = pressures
        .iter()
        .zip(areas)
        .map(|(&p, &a)| (p / peak).powi(q) * a)
        .sum();
    peak * (acc / gamma_area).powf(1.0 / f64::from(q))
}

/// Compute the conformity reward from contact readouts **already restricted to
/// Γ**.
///
/// Returns `NaN` for `pressure_uniformity` and `coverage` when the weighted
/// denominator vanishes — no contact, or all pressures at zero.
/// [`RewardBreakdown::score_with`] drops `NaN` terms, so a degenerate step
/// scores on what it does have rather than poisoning the sum.
#[must_use]
pub fn conformity_breakdown(
    gamma_readouts: &[ContactPairReadout],
    params: &ConformityParams,
) -> ConformityReadout {
    let mut pressures: Vec<f64> = Vec::with_capacity(gamma_readouts.len());
    let mut areas: Vec<f64> = Vec::with_capacity(gamma_readouts.len());
    let mut non_finite_pressures = 0_usize;
    for r in gamma_readouts {
        if r.pressure.is_finite() && r.tributary_area.is_finite() {
            pressures.push(r.pressure);
            areas.push(r.tributary_area);
        } else {
            non_finite_pressures += 1;
        }
    }

    // --- §5.1 uniformity: weighted squared coefficient of variation ---
    let w: Vec<f64> = pressures
        .iter()
        .map(|&p| logistic(params.beta_w * (p - params.p_th)))
        .collect();
    let w_area: f64 = w.iter().zip(&areas).map(|(&wi, &a)| wi * a).sum();
    let pressure_uniformity = if w_area > 0.0 {
        let bar: f64 = w
            .iter()
            .zip(&pressures)
            .zip(&areas)
            .map(|((&wi, &p), &a)| wi * p * a)
            .sum::<f64>()
            / w_area;
        if bar > 0.0 {
            let var: f64 = w
                .iter()
                .zip(&pressures)
                .zip(&areas)
                .map(|((&wi, &p), &a)| wi * (p - bar) * (p - bar) * a)
                .sum::<f64>()
                / w_area;
            -(var / (bar * bar))
        } else {
            f64::NAN
        }
    } else {
        f64::NAN
    };

    // --- §5.2 coverage: area-weighted mean of a smooth indicator over |Γ| ---
    let coverage = if params.gamma_area > 0.0 {
        pressures
            .iter()
            .zip(&areas)
            .map(|(&p, &a)| logistic(params.beta_c * (p - params.p_th)) * a)
            .sum::<f64>()
            / params.gamma_area
    } else {
        f64::NAN
    };

    // --- §5.3 peak-pressure barrier ---
    let p_peak_true = pressures.iter().copied().fold(0.0_f64, f64::max);
    let p_peak_smoothed = lq_smoothed_max(&pressures, &areas, params.gamma_area, params.q);
    let peak_bound = -normalised_barrier(params.p_max - p_peak_smoothed, params.m_hat);
    let lq_ratio = if p_peak_true > 0.0 {
        p_peak_smoothed / p_peak_true
    } else {
        f64::NAN
    };

    ConformityReadout {
        breakdown: RewardBreakdown {
            pressure_uniformity,
            coverage,
            peak_bound,
            // §6.3: k_min has no source. NaN, dropped by `score_with`.
            stiffness_bound: f64::NAN,
        },
        p_peak_smoothed,
        p_peak_true,
        lq_ratio,
        non_finite_pressures,
    }
}
