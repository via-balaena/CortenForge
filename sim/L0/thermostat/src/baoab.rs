//! BAOAB Langevin integrator (1-DOF) — D4 Layer-2 R6.
//!
//! The reference integrator that is **correct in the underdamped limit**, where
//! the production Euler–Maruyama path is not (R1 showed EM tracks the
//! spatial-diffusion rate even at γ=0.1, missing the energy-diffusion turnover).
//!
//! BAOAB (Leimkuhler & Matthews) splits one Langevin step into
//! `B A O A B`:
//! - **B** velocity kick: `v += ½·dt·F(x)/m`
//! - **A** drift: `x += ½·dt·v`
//! - **O** Ornstein–Uhlenbeck thermostat, solved **exactly**:
//!   `v ← c·v + √(kT/m·(1−c²))·ξ`, `c = exp(−(γ/m)·dt)`, `ξ ~ N(0,1)`
//!
//! The exact O step is the whole point: it samples the velocity's stationary
//! distribution without the Euler-Maruyama discretisation bias, so the kinetic
//! temperature `⟨v²⟩ = kT/m` holds essentially independent of `dt`, and the
//! underdamped energy-diffusion bottleneck is reproduced.
//!
//! Standalone (no sim-core coupling) — this is the trustworthy rate engine for
//! the high-Q regime the real p-bit lives in.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

use crate::double_well::DoubleWellPotential;

/// A 1-DOF BAOAB Langevin integrator in a quartic double well.
pub struct Baoab1D {
    /// Quartic coefficient `a = ΔV / x₀⁴`.
    a: f64,
    /// Well half-separation `x₀`.
    x_0: f64,
    /// Particle mass.
    mass: f64,
    /// Friction coefficient `γ` (force = −γ·v + noise).
    gamma: f64,
    /// Bath temperature `kT`.
    k_b_t: f64,
    /// Integration timestep.
    dt: f64,
    /// Position.
    x: f64,
    /// Velocity.
    v: f64,
    /// Deterministic noise source.
    rng: ChaCha8Rng,
    /// Cached second Box–Muller normal sample.
    spare: Option<f64>,
}

impl Baoab1D {
    /// Create an integrator for `well` at mass `mass`, friction `gamma`,
    /// temperature `k_b_t`, timestep `dt`, seeded by `seed`, starting at `x_init`
    /// (zero velocity).
    #[must_use]
    pub fn new(
        well: &DoubleWellPotential,
        mass: f64,
        gamma: f64,
        k_b_t: f64,
        dt: f64,
        seed: u64,
        x_init: f64,
    ) -> Self {
        let x_0 = well.well_separation();
        let a = well.barrier_height() / x_0.powi(4);
        Self {
            a,
            x_0,
            mass,
            gamma,
            k_b_t,
            dt,
            x: x_init,
            v: 0.0,
            rng: ChaCha8Rng::seed_from_u64(seed),
            spare: None,
        }
    }

    /// Conservative force `F(x) = −V′(x) = −4ax(x² − x₀²)`.
    fn force(&self, x: f64) -> f64 {
        -4.0 * self.a * x * (x * x - self.x_0 * self.x_0)
    }

    /// One standard-normal sample via Box–Muller (caches the pair's second).
    fn gauss(&mut self) -> f64 {
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

    /// Advance one BAOAB step.
    pub fn step(&mut self) {
        let m = self.mass;
        let half_dt = 0.5 * self.dt;
        let c = (-(self.gamma / m) * self.dt).exp();
        let sigma = (self.k_b_t / m * (1.0 - c * c)).sqrt();

        // B
        self.v += half_dt * self.force(self.x) / m;
        // A
        self.x += half_dt * self.v;
        // O (exact Ornstein–Uhlenbeck)
        self.v = c * self.v + sigma * self.gauss();
        // A
        self.x += half_dt * self.v;
        // B
        self.v += half_dt * self.force(self.x) / m;
    }

    /// Current position.
    #[must_use]
    pub const fn position(&self) -> f64 {
        self.x
    }

    /// Current velocity.
    #[must_use]
    pub const fn velocity(&self) -> f64 {
        self.v
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

    use super::*;

    /// BAOAB's hallmark: the kinetic temperature `⟨v²⟩ = kT/m` holds to within a
    /// few percent essentially independent of `dt` — even at a coarse step where
    /// Euler–Maruyama would be badly biased.
    #[test]
    fn equipartition_holds_across_timesteps() {
        let well = DoubleWellPotential::new(3.0, 1.0, 0);
        // Higher friction → faster velocity decorrelation → tight estimate in
        // fewer steps. ⟨v²⟩=kT/m is a velocity-marginal property (γ-independent).
        let (mass, gamma, kt) = (1.0, 5.0, 1.0);
        for &dt in &[0.001, 0.005, 0.02] {
            let mut sim = Baoab1D::new(&well, mass, gamma, kt, dt, 20_260_606, 1.0);
            for _ in 0..50_000 {
                sim.step();
            }
            let n = 2_000_000usize;
            let mut sum_v2 = 0.0;
            for _ in 0..n {
                sim.step();
                sum_v2 += sim.velocity() * sim.velocity();
            }
            let mean_v2 = sum_v2 / n as f64;
            assert!(
                (mean_v2 - kt / mass).abs() < 0.04,
                "dt={dt}: ⟨v²⟩={mean_v2:.4} should be ≈ kT/m = 1 (dt-independent for BAOAB)"
            );
        }
    }

    #[test]
    fn position_stays_bounded_in_the_wells() {
        let well = DoubleWellPotential::new(3.0, 1.0, 0);
        let mut sim = Baoab1D::new(&well, 1.0, 1.0, 1.0, 0.001, 7, 1.0);
        for _ in 0..200_000 {
            sim.step();
            // Quartic walls keep a kT≈1 particle well inside ±3·x₀.
            assert!(
                sim.position().abs() < 3.0,
                "position {} escaped",
                sim.position()
            );
        }
    }
}
