//! D4 physical-pbit analysis/calibration (S3).
//!
//! Host-side analysis of the Teensy's CSV stream (the
//! `pbit-fw-core::LogRecord` format: `t_us,drive,h0,…,ax,ay,az`). For one p-bit
//! channel it recovers the quantities the sim-to-real comparison needs:
//!
//! - **switching rate** and **dwell-time distribution** (with its coefficient of
//!   variation — ≈ 1 means exponential / Poisson escape),
//! - **in-well jitter** `⟨δx²⟩`, which calibrates the effective temperature via
//!   equipartition `kT_eff = M·ω_a²·⟨δx²⟩` ([`kt_eff`]),
//! - the **bath level** (accelerometer RMS),
//!
//! and compares the measured switching rate to the analytic Kramers prediction
//! ([`kramers_rate`], mirroring `sim_thermostat::DoubleWellPotential`).
//!
//! The well centres are auto-detected from the signal's two modes, so the
//! analysis needs no hand-set thresholds. See
//! `docs/thermo_computing/03_phases/d4_physical_pbit/recon.md`.

#![allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]

use std::error::Error;
use std::fmt;

/// One parsed log sample (SI-ish units: seconds, and raw counts for the rest).
#[derive(Debug, Clone)]
pub struct Sample {
    /// Time in seconds (converted from the `t_us` microsecond column).
    pub t_s: f64,
    /// Commanded drive (raw `i16` scale).
    pub drive: f64,
    /// Hall readings (raw ADC counts), one per p-bit.
    pub hall: Vec<f64>,
    /// Base accelerometer X/Y/Z (raw ADC counts).
    pub accel: [f64; 3],
}

/// CSV parse failure, with the offending 1-based line number.
#[derive(Debug)]
pub struct ParseError {
    /// 1-based line number where parsing failed.
    pub line: usize,
    /// Human-readable reason.
    pub msg: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "CSV parse error on line {}: {}", self.line, self.msg)
    }
}

impl Error for ParseError {}

/// Parse the firmware CSV stream into samples.
///
/// Accepts an optional header line (a leading line whose first field is not
/// numeric is skipped). Each data row is `t_us,drive,h0,…,h{k-1},ax,ay,az`, so a
/// row has `5 + k` fields with `k ≥ 1` Hall channels. `t_us` is converted from
/// microseconds to seconds.
///
/// # Errors
/// Returns [`ParseError`] on a row with too few fields or a non-numeric value.
pub fn parse_csv(text: &str) -> Result<Vec<Sample>, ParseError> {
    let mut out = Vec::new();
    let mut expected_cols: Option<usize> = None;
    for (idx, raw) in text.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        let mut fields = line.split(',');
        let first = fields.next().unwrap_or("");
        // Skip a header line (non-numeric first field).
        let t_us: f64 = match first.trim().parse() {
            Ok(v) => v,
            Err(_) if out.is_empty() => continue, // header
            Err(e) => {
                return Err(ParseError {
                    line: line_no,
                    msg: format!("t_us: {e}"),
                });
            }
        };
        let cols: Vec<&str> = line.split(',').collect();
        if cols.len() < 6 {
            return Err(ParseError {
                line: line_no,
                msg: format!(
                    "expected ≥6 fields (t_us,drive,h0,ax,ay,az), got {}",
                    cols.len()
                ),
            });
        }
        // Enforce rectangular data: a ragged row (e.g. a truncated serial line)
        // would otherwise yield Samples with mismatched hall lengths and panic
        // a later channel index. Reject it cleanly with the offending line.
        match expected_cols {
            None => expected_cols = Some(cols.len()),
            Some(n) if cols.len() != n => {
                return Err(ParseError {
                    line: line_no,
                    msg: format!("ragged row: {} fields, expected {n}", cols.len()),
                });
            }
            Some(_) => {}
        }
        let parse_at = |i: usize| -> Result<f64, ParseError> {
            cols[i].trim().parse().map_err(|e| ParseError {
                line: line_no,
                msg: format!("field {i}: {e}"),
            })
        };
        let drive = parse_at(1)?;
        let n_hall = cols.len() - 5; // drop t_us, drive, ax, ay, az
        let mut hall = Vec::with_capacity(n_hall);
        for i in 0..n_hall {
            hall.push(parse_at(2 + i)?);
        }
        let accel = [
            parse_at(cols.len() - 3)?,
            parse_at(cols.len() - 2)?,
            parse_at(cols.len() - 1)?,
        ];
        out.push(Sample {
            t_s: t_us * 1e-6,
            drive,
            hall,
            accel,
        });
    }
    Ok(out)
}

/// Switching + in-well statistics for one p-bit channel (input units).
#[derive(Debug, Clone)]
pub struct WellAnalysis {
    /// Number of samples used.
    pub n_samples: usize,
    /// Total record duration (s).
    pub duration_s: f64,
    /// Whether the record is bimodal (a switching two-well signal). `false`
    /// means a single occupied well — the cold calibration regime — where
    /// `well_lo == well_hi` (the global mean) and `n_switches == 0`.
    pub bimodal: bool,
    /// Lower well centre (input units); equal to `well_hi` in single-well mode.
    pub well_lo: f64,
    /// Upper well centre (input units); equal to `well_lo` in single-well mode.
    pub well_hi: f64,
    /// Number of detected well-to-well transitions.
    pub n_switches: usize,
    /// Switching rate (Hz) = transitions / duration.
    pub switch_rate_hz: f64,
    /// Mean dwell time in a well (s).
    pub dwell_mean_s: f64,
    /// Coefficient of variation of dwell times (≈ 1 ⇒ exponential).
    pub dwell_cv: f64,
    /// In-well variance `⟨δx²⟩` (input units²). In single-well mode, taken about
    /// the global mean over all samples (the unbiased calibration value); in
    /// bimodal mode, about the assigned well centre over deep-in-well samples.
    pub in_well_var: f64,
    /// Accelerometer RMS about its mean (bath-level proxy, raw counts).
    pub accel_rms: f64,
}

/// Well separation must exceed this multiple of the within-cluster spread for a
/// record to count as bimodal (a real two-well switching signal). Below it, the
/// signal is treated as a single occupied well. Empirically two-means on a single
/// Gaussian gives a separation/spread ratio ≈ 2.7, a true two-well signal ≫ 10.
const BIMODAL_SEPARATION_FACTOR: f64 = 4.0;

/// Analyse one Hall channel for switching and in-well statistics.
///
/// Auto-detects the two wells, counts hysteretic transitions, and measures the
/// dwell-time distribution and in-well variance. Returns `None` if the channel
/// index is invalid or there are fewer than two samples.
#[must_use]
pub fn analyze_channel(samples: &[Sample], channel: usize) -> Option<WellAnalysis> {
    // Guard against too-few samples and against any (e.g. hand-built or ragged)
    // sample lacking the channel — not just `samples[0]`.
    if samples.len() < 2 || samples.iter().any(|s| channel >= s.hall.len()) {
        return None;
    }
    let pos: Vec<f64> = samples.iter().map(|s| s.hall[channel]).collect();
    let duration_s = samples[samples.len() - 1].t_s - samples[0].t_s;

    // Auto-detect the two well centres by two-means (Lloyd), initialised at the
    // signal extremes. Robust to unequal dwell fractions — a median/mean split can
    // land *inside* a well when the system spends more time in one state.
    let mut lo = pos.iter().copied().fold(f64::INFINITY, f64::min);
    let mut hi = pos.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    for _ in 0..20 {
        let (mut lo_sum, mut lo_n, mut hi_sum, mut hi_n) = (0.0, 0u64, 0.0, 0u64);
        for &p in &pos {
            if (p - lo).abs() <= (p - hi).abs() {
                lo_sum += p;
                lo_n += 1;
            } else {
                hi_sum += p;
                hi_n += 1;
            }
        }
        if lo_n > 0 {
            lo = lo_sum / lo_n as f64;
        }
        if hi_n > 0 {
            hi = hi_sum / hi_n as f64;
        }
    }
    let (cl_lo, cl_hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };

    // Pooled within-cluster spread, to distinguish a true two-well (switching)
    // record from a single occupied well (the cold calibration regime). On a
    // unimodal signal two-means splits the lone Gaussian into two spurious
    // half-centres; gating on separation-vs-spread avoids reading a fake ⟨δx²⟩
    // (biased ~3× low) and fabricating switches on a non-switching record.
    let within_ss: f64 = pos
        .iter()
        .map(|&p| {
            let c = if (p - cl_lo).abs() <= (p - cl_hi).abs() {
                cl_lo
            } else {
                cl_hi
            };
            (p - c) * (p - c)
        })
        .sum();
    let within_std = (within_ss / pos.len() as f64).sqrt();
    let separation = cl_hi - cl_lo;
    let bimodal = separation > BIMODAL_SEPARATION_FACTOR * within_std;

    let (well_lo, well_hi, dwell, in_well_var) = if bimodal {
        // Switching two-well signal: hysteretic state machine + dwell + in-well
        // variance about the assigned well centre over deep-in-well samples.
        // NB: the `|p - mid| > band` gate truncates the barrier-side tail, so
        // this hot-regime variance is biased low — calibrate kT_eff cold instead.
        let mid = f64::midpoint(cl_lo, cl_hi);
        let band = separation * 0.25; // hysteresis half-width
        let mut state_hi = pos[0] >= mid;
        let mut last_switch_t = samples[0].t_s;
        let mut dwell: Vec<f64> = Vec::new();
        let (mut var_acc, mut var_n) = (0.0_f64, 0u64);
        for (s, &p) in samples.iter().zip(&pos) {
            if state_hi && p < mid - band {
                state_hi = false;
                dwell.push(s.t_s - last_switch_t);
                last_switch_t = s.t_s;
            } else if !state_hi && p > mid + band {
                state_hi = true;
                dwell.push(s.t_s - last_switch_t);
                last_switch_t = s.t_s;
            }
            if (p - mid).abs() > band {
                let centre = if state_hi { cl_hi } else { cl_lo };
                var_acc += (p - centre) * (p - centre);
                var_n += 1;
            }
        }
        let v = if var_n > 0 {
            var_acc / var_n as f64
        } else {
            f64::NAN
        };
        (cl_lo, cl_hi, dwell, v)
    } else {
        // Single occupied well: ⟨δx²⟩ about the global mean over ALL samples —
        // no clustering, no band gate — the unbiased calibration measurement.
        let m = mean(&pos);
        let var = pos.iter().map(|&p| (p - m) * (p - m)).sum::<f64>() / pos.len() as f64;
        (m, m, Vec::new(), var)
    };

    let n_switches = dwell.len();
    let switch_rate_hz = if duration_s > 0.0 {
        n_switches as f64 / duration_s
    } else {
        0.0
    };
    let dwell_mean_s = mean(&dwell);
    let dwell_cv = coefficient_of_variation(&dwell);
    let accel_rms = accel_rms(samples);

    Some(WellAnalysis {
        n_samples: samples.len(),
        duration_s,
        bimodal,
        well_lo,
        well_hi,
        n_switches,
        switch_rate_hz,
        dwell_mean_s,
        dwell_cv,
        in_well_var,
        accel_rms,
    })
}

/// Effective temperature `kT_eff` from in-well jitter via equipartition.
///
/// `kT_eff = M · ω_a² · ⟨δx²⟩`, where `in_well_var` is in raw counts² and
/// `pos_per_count` converts counts → physical position. **Calibrate cold**
/// (`kT_eff ≪ ΔV`, a single occupied well): pass the `in_well_var` from a
/// single-well capture (`WellAnalysis::bimodal == false`). At switching
/// temperatures the band-gate truncation biases `⟨δx²⟩` low (D4 S2 finding).
#[must_use]
pub fn kt_eff(in_well_var: f64, mass: f64, omega_a: f64, pos_per_count: f64) -> f64 {
    let var_phys = in_well_var * pos_per_count * pos_per_count;
    mass * omega_a * omega_a * var_phys
}

/// Analytic Kramers escape rate (Kramers–Grote–Hynes, spatial-diffusion regime).
///
/// Mirrors `sim_thermostat::DoubleWellPotential::kramers_rate` so the analysis
/// tool can overlay a prediction without depending on the sim crate. The
/// drift-guard test pins it to the validated value (0.01214 at the Phase-3 point).
#[must_use]
pub fn kramers_rate(delta_v: f64, x_0: f64, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
    let omega_a = (8.0 * delta_v / (mass * x_0 * x_0)).sqrt();
    let omega_b = (4.0 * delta_v / (mass * x_0 * x_0)).sqrt();
    let gamma_tilde = gamma / mass;
    let discriminant = (gamma_tilde * gamma_tilde + 4.0 * omega_b * omega_b).sqrt();
    let lambda_r = f64::midpoint(-gamma_tilde, discriminant);
    (omega_a / (2.0 * std::f64::consts::PI)) * (lambda_r / omega_b) * (-delta_v / k_b_t).exp()
}

fn mean(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        return f64::NAN;
    }
    xs.iter().sum::<f64>() / xs.len() as f64
}

fn coefficient_of_variation(xs: &[f64]) -> f64 {
    if xs.len() < 2 {
        return f64::NAN;
    }
    let m = mean(xs);
    let var = xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / xs.len() as f64;
    var.sqrt() / m
}

fn accel_rms(samples: &[Sample]) -> f64 {
    let n = samples.len() as f64;
    let mut mean_axis = [0.0_f64; 3];
    for s in samples {
        for (m, &a) in mean_axis.iter_mut().zip(s.accel.iter()) {
            *m += a / n;
        }
    }
    let mut acc = 0.0;
    for s in samples {
        for (&m, &a) in mean_axis.iter().zip(s.accel.iter()) {
            let d = a - m;
            acc += d * d;
        }
    }
    (acc / n).sqrt()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn next_u32(rng: &mut u64) -> u32 {
        let mut x = *rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        *rng = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }

    fn uniform(rng: &mut u64) -> f64 {
        f64::from(next_u32(rng) >> 8) / f64::from(1u32 << 24)
    }

    fn gauss(rng: &mut u64) -> f64 {
        let mut acc = 0.0;
        for _ in 0..12 {
            acc += uniform(rng);
        }
        acc - 6.0
    }

    /// Deterministic two-state telegraph with in-well Gaussian jitter, written as
    /// `Sample`s — exact ground truth for the analysis math.
    fn synth_telegraph(
        rate_hz: f64,
        dt: f64,
        sigma: f64,
        centre: f64,
        n: usize,
        seed: u64,
    ) -> Vec<Sample> {
        let mut rng = seed | 1;
        let p_switch = rate_hz * dt;
        let mut state = 1.0_f64;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            if uniform(&mut rng) < p_switch {
                state = -state;
            }
            let pos = state * centre + sigma * gauss(&mut rng);
            out.push(Sample {
                t_s: i as f64 * dt,
                drive: 0.0,
                hall: vec![pos],
                accel: [0.0, 0.0, 0.0],
            });
        }
        out
    }

    #[test]
    fn recovers_known_telegraph_statistics() {
        let (rate, dt, sigma, centre) = (5.0, 0.001, 0.15, 1.0);
        let samples = synth_telegraph(rate, dt, sigma, centre, 200_000, 12345);
        let a = analyze_channel(&samples, 0).expect("analysis");

        assert!(
            a.bimodal,
            "a switching two-well signal must be flagged bimodal"
        );
        // Well centres ≈ ±centre.
        assert!((a.well_hi - centre).abs() < 0.05, "well_hi {}", a.well_hi);
        assert!((a.well_lo + centre).abs() < 0.05, "well_lo {}", a.well_lo);
        // Switching rate ≈ known rate.
        let rate_ratio = a.switch_rate_hz / rate;
        assert!(
            (0.9..1.1).contains(&rate_ratio),
            "rate {} ({rate_ratio:.3}x)",
            a.switch_rate_hz
        );
        // Dwell times exponential (CV ≈ 1).
        assert!(
            (0.85..1.20).contains(&a.dwell_cv),
            "dwell CV {}",
            a.dwell_cv
        );
        // In-well variance ≈ σ².
        let var_ratio = a.in_well_var / (sigma * sigma);
        assert!(
            (0.85..1.15).contains(&var_ratio),
            "var {} ({var_ratio:.3}x)",
            a.in_well_var
        );
    }

    #[test]
    fn kramers_rate_matches_validated_value() {
        // Drift guard against sim_thermostat::DoubleWellPotential::kramers_rate.
        let k = kramers_rate(3.0, 1.0, 10.0, 1.0, 1.0);
        assert!((k - 0.01214).abs() < 0.0001, "kramers {k}");
    }

    #[test]
    fn kt_eff_inverts_equipartition() {
        // ⟨δx²⟩ = kT/(M ω_a²) ⇒ kt_eff should return kT. ω_a²=24, kT=0.3.
        let omega_a = (24.0_f64).sqrt();
        let var = 0.3 / 24.0;
        let recovered = kt_eff(var, 1.0, omega_a, 1.0);
        assert!((recovered - 0.3).abs() < 1e-9, "kt_eff {recovered}");
    }

    #[test]
    fn cold_single_well_recovers_unbiased_variance() {
        // rate 0 → never switches → one occupied well (the kT_eff calibration
        // regime). Two-means must NOT split the lone Gaussian into fake wells.
        let (sigma, centre) = (0.15, 1.0);
        let samples = synth_telegraph(0.0, 0.001, sigma, centre, 100_000, 777);
        let a = analyze_channel(&samples, 0).expect("analysis");

        assert!(
            !a.bimodal,
            "a cold single-mode signal must not be flagged bimodal"
        );
        assert_eq!(a.n_switches, 0, "single well has no switches");
        assert_eq!(a.well_lo, a.well_hi, "single-well centres collapse to one");
        assert!((a.well_hi - centre).abs() < 0.02, "centre {}", a.well_hi);
        // ⟨δx²⟩ ≈ σ² — unbiased (no two-means split, no band truncation).
        let var_ratio = a.in_well_var / (sigma * sigma);
        assert!(
            (0.9..1.1).contains(&var_ratio),
            "var {} ({var_ratio:.3}x)",
            a.in_well_var
        );
    }

    #[test]
    fn parse_csv_rejects_ragged_rows() {
        // A truncated/extended serial line must error cleanly, not slip through
        // to panic analyze_channel later.
        let csv = "t_us,drive,h0,ax,ay,az\n0,0,10,0,0,0\n1000,0,10,20,0,0,0\n";
        assert!(
            parse_csv(csv).is_err(),
            "ragged row (7 fields vs 6) must be rejected"
        );
    }
}
