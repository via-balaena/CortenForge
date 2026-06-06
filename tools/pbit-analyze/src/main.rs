//! `pbit-analyze` — analyse a Teensy p-bit CSV capture.
//!
//! ```text
//! pbit-analyze <capture.csv> [channel]
//! ```
//!
//! Prints switching rate, dwell-time statistics, in-well jitter, and bath level
//! for one Hall channel. See `pbit_analyze` for the library API and the
//! physical-units helpers (`kt_eff`, `kramers_rate`).

use anyhow::{Context, Result};

use pbit_analyze::{analyze_channel, parse_csv};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .context("usage: pbit-analyze <capture.csv> [channel]")?;
    let channel: usize = args
        .next()
        .map_or(Ok(0), |s| s.parse())
        .context("channel must be a non-negative integer")?;

    let text = std::fs::read_to_string(&path).with_context(|| format!("read {path}"))?;
    let samples = parse_csv(&text)?;
    let a = analyze_channel(&samples, channel)
        .context("analysis failed: channel out of range or fewer than 2 samples")?;

    let dwell_verdict = if (0.7..1.3).contains(&a.dwell_cv) {
        "~exponential (Poisson escape)"
    } else {
        "NON-exponential"
    };

    println!("pbit-analyze — {path}  (channel {channel})");
    println!("  samples       {}", a.n_samples);
    println!("  duration      {:.3} s", a.duration_s);
    println!(
        "  wells         lo={:.4}  hi={:.4}  sep={:.4}",
        a.well_lo,
        a.well_hi,
        a.well_hi - a.well_lo
    );
    println!("  switches      {}", a.n_switches);
    println!("  switch rate   {:.4} Hz", a.switch_rate_hz);
    println!(
        "  dwell         mean={:.4} s   CV={:.3}  [{dwell_verdict}]",
        a.dwell_mean_s, a.dwell_cv
    );
    println!(
        "  in-well var   {:.6} (counts²)   → feed to kt_eff() with the rig calibration",
        a.in_well_var
    );
    println!(
        "  accel RMS     {:.4} (counts, bath-level proxy)",
        a.accel_rms
    );
    Ok(())
}
