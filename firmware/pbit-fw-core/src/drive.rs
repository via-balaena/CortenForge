//! Shaker drive-signal generator: band-limited Gaussian noise (the temperature
//! knob) plus an optional sinusoidal probe.

// Numeric DSP code: the index/dimension and unit-range casts are intentional and
// bounded. Matches the house allow-block used across the thermo experiments.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::must_use_candidate,
    // const-ness of these tiny helpers isn't worth the constructor cascade.
    clippy::missing_const_for_fn
)]

use core::f32::consts::TAU;

use libm::sinf;

/// Generates the shaker drive signal as a normalized value in `[-1, 1]`.
///
/// The signal is `noise_gain · lowpass(white_gaussian) + sine_amp · sin(φ)`:
/// band-limited Gaussian noise sets the effective temperature (sweep
/// [`set_noise_gain`](Self::set_noise_gain)), and an optional sinusoid
/// (configured via [`set_sine`](Self::set_sine)) is the sub-threshold probe for
/// the stochastic-resonance step. The embedded layer maps the output onto a PWM
/// duty cycle with [`to_pwm_duty`](Self::to_pwm_duty).
///
/// Deterministic from `seed`; uses no heap and no transcendental noise (Gaussian
/// samples come from an Irwin–Hall sum), so it runs identically on host and MCU.
pub struct DriveGen {
    rng: u64,
    sample_rate_hz: f32,
    noise_gain: f32,
    lp_alpha: f32,
    lp_state: f32,
    sine_amp: f32,
    sine_phase: f32,
    sine_dphase: f32,
}

impl DriveGen {
    /// Create a generator at `sample_rate_hz`, low-passing the noise at
    /// `noise_cutoff_hz` (set it a few × the cantilever resonance so the bath is
    /// broadband relative to the well but not full white). Noise gain starts at
    /// zero and no sine is configured.
    pub fn new(sample_rate_hz: f32, noise_cutoff_hz: f32, seed: u64) -> Self {
        Self {
            rng: seed | 1,
            sample_rate_hz,
            noise_gain: 0.0,
            lp_alpha: one_pole_alpha(noise_cutoff_hz, sample_rate_hz),
            lp_state: 0.0,
            sine_amp: 0.0,
            sine_phase: 0.0,
            sine_dphase: 0.0,
        }
    }

    /// Set the noise amplitude — the effective *temperature* knob we sweep to
    /// trace the Kramers/Arrhenius curve. Negative values are clamped to zero.
    pub fn set_noise_gain(&mut self, gain: f32) {
        self.noise_gain = gain.max(0.0);
    }

    /// Configure the sinusoidal probe (amplitude in drive units, frequency Hz).
    /// Set `amp` to zero to disable. Used only in the stochastic-resonance step.
    pub fn set_sine(&mut self, amp: f32, freq_hz: f32) {
        self.sine_amp = amp.max(0.0);
        self.sine_dphase = TAU * freq_hz / self.sample_rate_hz;
    }

    /// The exact instantaneous sine phase (radians, in `[0, 2π)`). Logged
    /// alongside samples to give the phase reference the synchrony metric needs.
    pub fn sine_phase(&self) -> f32 {
        self.sine_phase
    }

    /// Advance one sample; returns the drive value clamped to `[-1, 1]`.
    pub fn next_sample(&mut self) -> f32 {
        // One-pole low-pass of white Gaussian noise → band-limited bath.
        let white = self.gaussian();
        self.lp_state += self.lp_alpha * (white - self.lp_state);
        let noise = self.noise_gain * self.lp_state;

        // Sinusoidal probe with an exactly-tracked phase accumulator.
        let probe = self.sine_amp * sinf(self.sine_phase);
        self.sine_phase += self.sine_dphase;
        if self.sine_phase >= TAU {
            self.sine_phase -= TAU;
        }

        clamp_unit(noise + probe)
    }

    /// Map a `[-1, 1]` drive value onto a PWM duty in `[0, max_duty]`, with zero
    /// at mid-rail (the amp + RC filter recovers the bipolar analog signal).
    pub fn to_pwm_duty(value: f32, max_duty: u16) -> u16 {
        let frac = 0.5 * (clamp_unit(value) + 1.0);
        (frac * f32::from(max_duty)) as u16
    }

    /// xorshift64* — small, fast, deterministic PRNG step.
    fn next_u32(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }

    /// Uniform float in `[0, 1)` from 24 random bits.
    fn uniform(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Approximately-N(0,1) sample via the Irwin–Hall sum of 12 uniforms (mean 6,
    /// variance 1), avoiding any transcendental in the noise path.
    fn gaussian(&mut self) -> f32 {
        let mut acc = 0.0;
        for _ in 0..12 {
            acc += self.uniform();
        }
        acc - 6.0
    }
}

/// One-pole low-pass coefficient `α = dt / (RC + dt)` for a cutoff in Hz. A
/// non-positive cutoff yields `α = 1` (pass-through, full white noise).
fn one_pole_alpha(cutoff_hz: f32, sample_rate_hz: f32) -> f32 {
    if cutoff_hz <= 0.0 {
        return 1.0;
    }
    let dt = 1.0 / sample_rate_hz;
    let rc = 1.0 / (TAU * cutoff_hz);
    dt / (rc + dt)
}

/// Clamp to the unit interval `[-1, 1]`.
fn clamp_unit(v: f32) -> f32 {
    v.clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 20_000.0;

    #[test]
    fn gaussian_is_unit_normal() {
        let mut g = DriveGen::new(SR, 5000.0, 42);
        let n = 200_000;
        let mut sum = 0.0_f64;
        let mut sumsq = 0.0_f64;
        for _ in 0..n {
            let x = f64::from(g.gaussian());
            sum += x;
            sumsq += x * x;
        }
        let mean = sum / f64::from(n);
        let var = sumsq / f64::from(n) - mean * mean;
        assert!(mean.abs() < 0.02, "mean {mean} not ~0");
        assert!((var - 1.0).abs() < 0.05, "var {var} not ~1");
    }

    #[test]
    fn noise_gain_scales_output() {
        // Higher gain → larger output variance (the temperature knob works).
        let var_at = |gain: f32| {
            let mut g = DriveGen::new(SR, 8000.0, 7);
            g.set_noise_gain(gain);
            let mut sumsq = 0.0_f64;
            let n = 100_000;
            for _ in 0..n {
                let v = f64::from(g.next_sample());
                sumsq += v * v;
            }
            sumsq / f64::from(n)
        };
        assert!(var_at(0.3) > var_at(0.1), "variance should grow with gain");
    }

    #[test]
    fn lower_cutoff_reduces_variance() {
        // A one-pole low-pass passes less white-noise power as the cutoff drops.
        let var_at_cutoff = |cutoff: f32| {
            let mut g = DriveGen::new(SR, cutoff, 99);
            g.set_noise_gain(1.0);
            let mut sumsq = 0.0_f64;
            let n = 100_000;
            for _ in 0..n {
                let v = f64::from(g.next_sample());
                sumsq += v * v;
            }
            sumsq / f64::from(n)
        };
        assert!(var_at_cutoff(500.0) < var_at_cutoff(8000.0));
    }

    #[test]
    fn sine_has_requested_frequency() {
        let freq = 50.0_f32;
        let mut g = DriveGen::new(SR, 1.0, 1); // cutoff tiny so noise≈0
        g.set_noise_gain(0.0);
        g.set_sine(1.0, freq);
        let n = (SR as usize) * 2; // 2 seconds
        let mut crossings = 0;
        let mut prev = g.next_sample();
        for _ in 1..n {
            let cur = g.next_sample();
            if prev <= 0.0 && cur > 0.0 {
                crossings += 1;
            }
            prev = cur;
        }
        // ~freq upward zero-crossings per second.
        let est = crossings as f32 / 2.0;
        assert!((est - freq).abs() < 1.0, "estimated {est} Hz vs {freq} Hz");
    }

    #[test]
    fn pwm_duty_maps_endpoints_and_midpoint() {
        assert_eq!(DriveGen::to_pwm_duty(-1.0, 1000), 0);
        assert_eq!(DriveGen::to_pwm_duty(1.0, 1000), 1000);
        assert_eq!(DriveGen::to_pwm_duty(0.0, 1000), 500);
        // Out-of-range clamps rather than wraps.
        assert_eq!(DriveGen::to_pwm_duty(-5.0, 1000), 0);
        assert_eq!(DriveGen::to_pwm_duty(5.0, 1000), 1000);
    }
}
