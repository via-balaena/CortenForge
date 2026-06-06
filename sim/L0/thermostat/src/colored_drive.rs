//! Colored-noise-driven 1-DOF integrator — D4 Layer-2 R2.
//!
//! Models the macroscopic shaker-driven beam: intrinsic damping `γ` plus an
//! **external** Ornstein–Uhlenbeck colored force `η(t)` that is **not**
//! fluctuation-dissipation-paired with the damping (the shaker noise), with
//! negligible room-temperature thermal noise. The question R2 answers: does this
//! reach a Boltzmann-like (thermal) stationary state, and under what noise
//! bandwidth?
//!
//! ```text
//! m·ẍ = F(x) − γ·ẋ + η(t),   η̇ = −η/τ + √(σ²)·white,   σ² = γ·kT_eff/τ
//! ```
//!
//! `η` has correlation time `τ` (inverse bandwidth); its white-noise limit
//! (`τ→0`) is intensity `2γ·kT_eff`, i.e. an ordinary thermal bath at `kT_eff`.
//! The test of "thermal-ness" is whether the **kinetic** temperature `m⟨v²⟩` and
//! the **configurational** temperature `mω_a²⟨δx²⟩` agree (equipartition) — they
//! do for short `τ` (wide bandwidth) and diverge for long `τ` (narrow band),
//! which sets the hard rig rule: drive the shaker with broadband noise.
//!
//! Integrated BAOAB-style (damping-only O step, since the colored force is the
//! energy source) for underdamped fidelity.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

use crate::double_well::DoubleWellPotential;

/// A 1-DOF quartic-double-well oscillator driven by external OU colored noise.
pub struct ColoredDriveSim {
    a: f64,
    x_0: f64,
    mass: f64,
    gamma: f64,
    dt: f64,
    /// OU retention factor `exp(−dt/τ)`.
    ou_retain: f64,
    /// OU per-step innovation std `√(σ²(1−retain²))`.
    ou_innov: f64,
    /// Current colored force.
    eta: f64,
    x: f64,
    v: f64,
    rng: ChaCha8Rng,
    spare: Option<f64>,
}

impl ColoredDriveSim {
    /// Drive `well` (mass `mass`, damping `gamma`) with OU colored noise of
    /// correlation time `tau` whose white-noise limit is a thermal bath at
    /// `kt_eff`. Timestep `dt`, seed `seed`, starts at `x_init` (zero velocity).
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        well: &DoubleWellPotential,
        mass: f64,
        gamma: f64,
        kt_eff: f64,
        tau: f64,
        dt: f64,
        seed: u64,
        x_init: f64,
    ) -> Self {
        let x_0 = well.well_separation();
        let a = well.barrier_height() / x_0.powi(4);
        let sigma2 = gamma * kt_eff / tau; // ⟨η²⟩
        let ou_retain = (-dt / tau).exp();
        let ou_innov = (sigma2 * (1.0 - ou_retain * ou_retain)).sqrt();
        Self {
            a,
            x_0,
            mass,
            gamma,
            dt,
            ou_retain,
            ou_innov,
            eta: 0.0,
            x: x_init,
            v: 0.0,
            rng: ChaCha8Rng::seed_from_u64(seed),
            spare: None,
        }
    }

    /// Conservative force `F(x) = −4ax(x² − x₀²)`.
    fn force(&self, x: f64) -> f64 {
        -4.0 * self.a * x * (x * x - self.x_0 * self.x_0)
    }

    /// Potential second derivative `V''(x) = 4a(3x² − x₀²)` (for config temp).
    fn curvature(&self, x: f64) -> f64 {
        4.0 * self.a * (3.0 * x * x - self.x_0 * self.x_0)
    }

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

    /// Advance one step.
    pub fn step(&mut self) {
        // Exact OU update of the colored force.
        self.eta = self.ou_retain * self.eta + self.ou_innov * self.gauss();

        let m = self.mass;
        let half_dt = 0.5 * self.dt;
        let c = (-(self.gamma / m) * self.dt).exp();
        let total = self.force(self.x) + self.eta;

        self.v += half_dt * total / m; // B
        self.x += half_dt * self.v; // A
        self.v *= c; // O (damping only)
        self.x += half_dt * self.v; // A
        let total2 = self.force(self.x) + self.eta;
        self.v += half_dt * total2 / m; // B
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

    /// `(F′² , V″)` contributions for the configurational temperature estimator
    /// `kT_conf = ⟨V′²⟩ / ⟨V″⟩` (equals `kT` for any Boltzmann state).
    #[must_use]
    pub fn config_temp_terms(&self) -> (f64, f64) {
        let fprime = -self.force(self.x); // V′ = −F
        (fprime * fprime, self.curvature(self.x))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

    use super::*;

    /// Wide bandwidth (short τ): the injected colored noise behaves as a thermal
    /// bath — kinetic and configurational temperatures both ≈ `kT_eff`.
    #[test]
    fn short_correlation_time_is_thermal() {
        let well = DoubleWellPotential::new(10.0, 1.0, 0); // deep well: no escape
        let (mass, gamma, kt_eff, dt) = (1.0, 1.0, 1.0, 0.001);
        let tau = 0.005; // ≪ 1/ω_a (ω_a = √80 ≈ 8.9 → 1/ω_a ≈ 0.11)
        let mut sim = ColoredDriveSim::new(&well, mass, gamma, kt_eff, tau, dt, 20_260_607, 1.0);
        for _ in 0..100_000 {
            sim.step();
        }
        let n = 3_000_000usize;
        let (mut sum_v2, mut sum_fp2, mut sum_curv) = (0.0, 0.0, 0.0);
        for _ in 0..n {
            sim.step();
            sum_v2 += sim.velocity() * sim.velocity();
            let (fp2, curv) = sim.config_temp_terms();
            sum_fp2 += fp2;
            sum_curv += curv;
        }
        let kt_kin = mass * sum_v2 / n as f64;
        let kt_conf = sum_fp2 / sum_curv;
        assert!(
            (kt_kin - kt_eff).abs() < 0.1,
            "kinetic kT={kt_kin:.3} not ≈ {kt_eff}"
        );
        assert!(
            (kt_conf - kt_eff).abs() < 0.15,
            "config kT={kt_conf:.3} not ≈ {kt_eff}"
        );
        // The two agree → thermal/Boltzmann-like.
        assert!(
            (kt_kin / kt_conf - 1.0).abs() < 0.2,
            "kin/conf = {:.3} not ≈1",
            kt_kin / kt_conf
        );
    }
}
