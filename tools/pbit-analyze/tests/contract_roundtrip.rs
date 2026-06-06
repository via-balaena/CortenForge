//! Firmware → analysis data-contract round-trip.
//!
//! Generates a known two-state telegraph, frames every sample through the *real*
//! on-wire format (`pbit_fw_core::LogRecord::write_csv` / `csv_header`), then
//! parses it back with `pbit_analyze::parse_csv` and checks the analysis recovers
//! the ground truth. This validates both the analysis math and that the firmware
//! and the analyser agree on the CSV format — before any hardware exists.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::missing_const_for_fn
)]

use pbit_analyze::{analyze_channel, parse_csv};
use pbit_fw_core::LogRecord;

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

fn build_csv(rate_hz: f64, dt: f64, sigma: f64, centre: f64, n: usize, seed: u64) -> String {
    let mut rng = seed | 1;
    let mut csv = String::new();
    let mut buf = [0u8; 128];
    let h = LogRecord::csv_header(1, &mut buf).unwrap();
    csv.push_str(core::str::from_utf8(&buf[..h]).unwrap());

    let p_switch = rate_hz * dt;
    let mut state = 1.0_f64;
    for i in 0..n {
        if uniform(&mut rng) < p_switch {
            state = -state;
        }
        let pos = state * centre + sigma * gauss(&mut rng);
        let rec = LogRecord {
            t_us: (i as f64 * dt * 1e6) as u64,
            drive: 0,
            hall: [pos as i16, 0, 0, 0, 0, 0, 0, 0],
            n_hall: 1,
            accel: [0, 0, 0],
        };
        let m = rec.write_csv(&mut buf).unwrap();
        csv.push_str(core::str::from_utf8(&buf[..m]).unwrap());
    }
    csv
}

#[test]
fn firmware_csv_round_trips_through_analysis() {
    let (rate, dt, sigma, centre) = (5.0, 0.001, 150.0, 1000.0);
    let csv = build_csv(rate, dt, sigma, centre, 200_000, 0xABCD);

    let samples = parse_csv(&csv).expect("parse");
    assert_eq!(samples.len(), 200_000);
    assert_eq!(samples[0].hall.len(), 1);

    let a = analyze_channel(&samples, 0).expect("analysis");

    // Well centres ≈ ±1000 counts.
    assert!((a.well_hi - centre).abs() < 30.0, "well_hi {}", a.well_hi);
    assert!((a.well_lo + centre).abs() < 30.0, "well_lo {}", a.well_lo);
    // Switching rate ≈ 5 Hz.
    let rate_ratio = a.switch_rate_hz / rate;
    assert!(
        (0.9..1.1).contains(&rate_ratio),
        "rate {} ({rate_ratio:.3}x)",
        a.switch_rate_hz
    );
    // Dwell ~exponential.
    assert!(
        (0.85..1.20).contains(&a.dwell_cv),
        "dwell CV {}",
        a.dwell_cv
    );
    // In-well variance ≈ σ² = 150² = 22500 counts².
    let var_ratio = a.in_well_var / (sigma * sigma);
    assert!(
        (0.85..1.15).contains(&var_ratio),
        "var {} ({var_ratio:.3}x)",
        a.in_well_var
    );
}
