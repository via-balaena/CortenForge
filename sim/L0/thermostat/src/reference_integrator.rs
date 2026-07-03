//! Shared building blocks for the standalone 1-DOF reference integrators
//! ([`Baoab1D`](crate::Baoab1D), [`ColoredDriveSim`](crate::ColoredDriveSim)).
//!
//! These two integrators live outside the `sim-core` `cb_passive` pipeline —
//! they are self-contained rate engines for the underdamped regime — and they
//! shared two verbatim pieces: the quartic-well conservative force and a
//! Box–Muller normal sampler with a cached spare. This module holds the single
//! copy of each.
//!
//! ## Numerics note (load-bearing)
//!
//! [`quartic_well_force`] evaluates `x² − x₀²` as a plain multiply-then-subtract
//! (two roundings), **not** the fused `mul_add` that
//! [`DoubleWellPotential::apply`](crate::DoubleWellPotential) uses on the
//! production `sim-core` path. That divergence is deliberate: it reproduces the
//! integrators' prior inline form bit-for-bit, so their seeded trajectories (and
//! the stochastic validators that pin them) are unchanged by the extraction.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// Quartic double-well conservative force `F(x) = −V′(x) = −4ax(x² − x₀²)`,
/// with `a = ΔV / x₀⁴`.
///
/// Plain (non-FMA) evaluation shared by the standalone reference integrators;
/// see the module-level numerics note for why it does not mirror
/// [`DoubleWellPotential::apply`](crate::DoubleWellPotential)'s fused form.
pub fn quartic_well_force(a: f64, x_0: f64, x: f64) -> f64 {
    -4.0 * a * x * (x * x - x_0 * x_0)
}

/// A `ChaCha8`-backed standard-normal sampler that caches the second of each
/// Box–Muller pair.
///
/// This is the one-sample-at-a-time counterpart to [`crate::prf`]'s block-based
/// `box_muller_from_block`: the reference integrators consume one Gaussian per
/// call, so a cached spare halves the transcendental cost.
pub struct NormalSampler {
    rng: ChaCha8Rng,
    /// Cached second Box–Muller normal sample.
    spare: Option<f64>,
}

impl NormalSampler {
    /// Seed a fresh sampler (empty spare cache).
    pub fn seed_from_u64(seed: u64) -> Self {
        Self {
            rng: ChaCha8Rng::seed_from_u64(seed),
            spare: None,
        }
    }

    /// One standard-normal sample via Box–Muller (caches the pair's second).
    pub fn sample(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let u1: f64 = self.rng.random::<f64>().max(1e-300);
        let u2: f64 = self.rng.random::<f64>();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }
}
