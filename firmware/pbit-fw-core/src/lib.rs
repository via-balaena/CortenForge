//! Host-testable core of the D4 physical-pbit firmware.
//!
//! This `no_std` crate holds the parts of the Teensy 4.1 firmware that have **no
//! hardware dependency** and therefore can be unit-tested on the host:
//!
//! - [`DriveGen`] — generates the shaker drive signal: band-limited Gaussian
//!   noise (the effective *temperature* — the knob we sweep) plus an optional
//!   sinusoidal probe (for the stochastic-resonance step), as a normalized value
//!   the embedded layer maps onto a PWM duty cycle.
//! - [`LogRecord`] — one logged sample, plus the **CSV wire format** that the
//!   S3 analysis/calibration tool parses. Defining it here makes the
//!   firmware → analysis data contract a single source of truth.
//!
//! The embedded binary (`firmware/pbit-fw`, a separate project for the
//! `thumbv7em-none-eabihf` target) owns the peripheral glue — ADC sampling of
//! the Hall sensors + accelerometer, PWM output, USB-serial streaming — and
//! calls into this crate for the math and framing.
//!
//! Rationale: the accelerometer measures the *actual* injected vibration, so the
//! drive generator does not need codec-grade fidelity — PWM is sufficient and we
//! calibrate the bath from what the shaker really did. See
//! `docs/thermo_computing/03_phases/d4_physical_pbit/recon.md`.

#![no_std]

mod drive;
mod record;

pub use drive::DriveGen;
pub use record::LogRecord;

/// Maximum number of Hall channels in a log record — one per p-bit in a chain
/// of up to eight coupled elements (Step 2 of the arc).
pub const MAX_HALL: usize = 8;
