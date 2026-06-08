//! G1 scorecard helpers — grade the placed, articulated knee against the
//! OpenSim oracle.
//!
//! The moment-arm helpers finite-difference the muscle path lengths from the
//! placed, articulated model (the same `Fitter::pose` the flexion sweep renders)
//! over the knee angle. **Important honesty note:** placement is a SIMILARITY
//! (`xform(q) = kp + scale·R·(q − knee_osim)`), so every length is exactly
//! `scale × model_length` and de-scaling recovers the cf-osim *model* moment arm
//! independent of R/scale/kp. So a match here does NOT independently validate the
//! placement — it equals the S1 cf-osim↔OpenSim agreement (~0.3 mm), re-derived
//! through the `pose()` muscle-assembly path. Its value is a REGRESSION check:
//! it catches a bug where `pose()` diverges from the oracle (wrong body lookup,
//! dropped conditional point, pose-map drift). Placement itself is graded by the
//! joint-center anchor + envelope containment, not by this.

use crate::Fitter;

/// Placed muscle path length at `theta` (rad), de-scaled to model meters.
///
/// Correct ONLY because v1 uses a single uniform `scale` (so length =
/// `scale × model_length` exactly). If anisotropic per-segment scaling lands
/// (the S3 follow-up), `length ≠ scale × model_length` and this de-scaling must
/// be reworked.
pub fn placed_length_m(fit: &Fitter, muscle: &str, theta: f64) -> f64 {
    let placed: f64 = fit
        .pose(theta)
        .muscles
        .iter()
        .find(|m| m.name == muscle)
        .map(|m| m.polyline.windows(2).map(|w| (w[1] - w[0]).norm()).sum())
        .unwrap_or(0.0);
    placed / fit.scale()
}

/// Moment arm `-d(length)/d(theta)` of a placed muscle at `theta` (rad),
/// de-scaled to model meters — directly comparable to the OpenSim reference.
/// Central finite difference with step `eps` (rad).
pub fn placed_moment_arm_m(fit: &Fitter, muscle: &str, theta: f64, eps: f64) -> f64 {
    let plus = placed_length_m(fit, muscle, theta + eps);
    let minus = placed_length_m(fit, muscle, theta - eps);
    -(plus - minus) / (2.0 * eps)
}

/// Agreement between two series (reported in millimeters).
#[derive(Debug, Clone, Copy)]
pub struct Agreement {
    pub rmse_mm: f64,
    pub max_abs_mm: f64,
    /// Pearson correlation of the two series (shape agreement).
    pub corr: f64,
}

/// Compare our series to a reference series (both in meters → reported in mm).
/// Returns `None` on length mismatch or an empty/degenerate series.
pub fn agreement(ours_m: &[f64], reference_m: &[f64]) -> Option<Agreement> {
    if ours_m.len() != reference_m.len() || ours_m.is_empty() {
        return None;
    }
    let n = ours_m.len() as f64;
    let mut sse = 0.0;
    let mut max_abs = 0.0_f64;
    for (&o, &r) in ours_m.iter().zip(reference_m) {
        let d = (o - r) * 1000.0;
        sse += d * d;
        max_abs = max_abs.max(d.abs());
    }
    let rmse_mm = (sse / n).sqrt();

    // Pearson correlation.
    let mean = |s: &[f64]| s.iter().sum::<f64>() / n;
    let (mo, mr) = (mean(ours_m), mean(reference_m));
    let mut cov = 0.0;
    let mut vo = 0.0;
    let mut vr = 0.0;
    for (&o, &r) in ours_m.iter().zip(reference_m) {
        cov += (o - mo) * (r - mr);
        vo += (o - mo).powi(2);
        vr += (r - mr).powi(2);
    }
    let corr = if vo > 1e-18 && vr > 1e-18 {
        cov / (vo.sqrt() * vr.sqrt())
    } else {
        // A flat reference (no variation) → correlation undefined; treat a
        // matching-flat series as perfect, else uncorrelated.
        if rmse_mm < 1e-6 { 1.0 } else { 0.0 }
    };

    Some(Agreement {
        rmse_mm,
        max_abs_mm: max_abs,
        corr,
    })
}
