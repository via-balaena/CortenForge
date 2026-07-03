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
//! The diagnostic is Boltzmann **shape**: whether the **kinetic** temperature
//! `m⟨v²⟩` and the **configurational** temperature `⟨V′²⟩/⟨V″⟩` agree
//! (equipartition). They do for short `τ` (wide bandwidth) and diverge for long
//! `τ`, setting the rig rule: drive the shaker with broadband noise. **Note:**
//! the ratio measures *shape*, not absolute temperature — the OU rolloff also
//! suppresses the absolute `kT` (~10% at `τ·ω_a ≈ 0.3`), so the operating point
//! must be calibrated against the measured in-well variance.
//!
//! Integrated BAOAB-style (damping-only O step, since the colored force is the
//! energy source) for underdamped fidelity.

use crate::double_well::DoubleWellPotential;
use crate::reference_integrator::{NormalSampler, quartic_well_force};

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
    /// Deterministic noise source (Box–Muller, cached spare).
    noise: NormalSampler,
}

impl ColoredDriveSim {
    /// Drive `well` (mass `mass`, damping `gamma`) with OU colored noise of
    /// correlation time `tau` whose white-noise limit is a thermal bath at
    /// `kt_eff`. Timestep `dt`, seed `seed`, starts at `x_init` (zero velocity).
    #[must_use]
    // integrator config: 8 physical parameters; a config struct would add
    // ceremony without clarity for a numerical constructor.
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
            noise: NormalSampler::seed_from_u64(seed),
        }
    }

    /// Conservative force `F(x) = −4ax(x² − x₀²)`.
    fn force(&self, x: f64) -> f64 {
        quartic_well_force(self.a, self.x_0, x)
    }

    /// Potential second derivative `V''(x) = 4a(3x² − x₀²)` (for config temp).
    fn curvature(&self, x: f64) -> f64 {
        4.0 * self.a * (3.0 * x * x - self.x_0 * self.x_0)
    }

    /// Advance one step.
    pub fn step(&mut self) {
        // Exact OU update of the colored force.
        self.eta = self.ou_retain * self.eta + self.ou_innov * self.noise.sample();

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
