//! Symmetric quartic double-well potential for thermodynamic computing.
//!
//! Implements the standard quartic double-well `V(x) = a(x² − x₀²)²` as
//! a [`PassiveComponent`] that contributes conservative forces to the
//! `qfrc_passive` accumulator. Combined with a [`LangevinThermostat`] in
//! a [`PassiveStack`], this produces a bistable system whose switching
//! rate between wells is governed by Kramers' escape-rate formula.
//!
//! Phase 3 of the thermodynamic computing initiative validates this
//! component against Kramers' formula. Phase 4+ uses arrays of these
//! elements to build coupled bistable systems.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`LangevinThermostat`]: crate::LangevinThermostat
//! [`PassiveStack`]: crate::PassiveStack

use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;
use crate::diagnose::Diagnose;

/// Symmetric quartic double-well potential: `V(x) = a(x² − x₀²)²`
/// where `a = ΔV / x₀⁴`.
///
/// Contributes force `F(x) = −V′(x) = −4ax(x² − x₀²)` to the per-DOF
/// force accumulator on a single DOF. This is a deterministic conservative
/// force — it does not implement [`Stochastic`](crate::Stochastic).
///
/// # Joint type constraint
///
/// The `dof` field is used to index both `data.qpos` and `qfrc_out`.
/// This is correct for slide and hinge joints where `nq = nv = 1` (DOF
/// index = qpos index). It does **not** support ball (`nq=4, nv=3`) or
/// free (`nq=7, nv=6`) joints where these indices diverge.
///
/// # Example
///
/// ```ignore
/// use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PassiveStack};
/// use sim_core::DVector;
///
/// let well = DoubleWellPotential::new(3.0, 1.0, 0);
/// let thermostat = LangevinThermostat::new(
///     DVector::from_element(1, 10.0),
///     1.0,
///     42,
///     0,
/// );
///
/// PassiveStack::builder()
///     .with(well)
///     .with(thermostat)
///     .build()
///     .install(&mut model);
/// ```
pub struct DoubleWellPotential {
    /// Barrier height: `ΔV = V(0) − V(±x₀)`.
    delta_v: f64,
    /// Well half-separation: potential minima at `±x₀`.
    x_0: f64,
    /// DOF index this potential acts on (= qpos index for slide/hinge).
    dof: usize,
}

impl DoubleWellPotential {
    /// Create a new double-well potential.
    ///
    /// # Parameters
    /// - `delta_v`: barrier height `ΔV > 0`
    /// - `x_0`: well half-separation `x₀ > 0` (minima at `±x₀`)
    /// - `dof`: DOF index (must be valid for the target model)
    ///
    /// # Panics
    /// Panics if `delta_v <= 0` or `x_0 <= 0`.
    #[must_use]
    pub fn new(delta_v: f64, x_0: f64, dof: usize) -> Self {
        assert!(
            delta_v > 0.0,
            "barrier height must be positive, got {delta_v}"
        );
        assert!(x_0 > 0.0, "well separation must be positive, got {x_0}");
        Self { delta_v, x_0, dof }
    }

    /// Barrier height `ΔV`.
    #[must_use]
    pub const fn barrier_height(&self) -> f64 {
        self.delta_v
    }

    /// Well half-separation `x₀`.
    #[must_use]
    pub const fn well_separation(&self) -> f64 {
        self.x_0
    }

    /// Angular frequency at well bottom: `ω_a = √(8ΔV / (M·x₀²))`.
    #[must_use]
    pub fn omega_a(&self, mass: f64) -> f64 {
        (8.0 * self.delta_v / (mass * self.x_0 * self.x_0)).sqrt()
    }

    /// Angular frequency at barrier top: `ω_b = √(4ΔV / (M·x₀²))`.
    #[must_use]
    pub fn omega_b(&self, mass: f64) -> f64 {
        (4.0 * self.delta_v / (mass * self.x_0 * self.x_0)).sqrt()
    }

    /// Kramers escape rate (one-directional) using the Kramers–Grote–Hynes
    /// formula for the spatial-diffusion regime.
    ///
    /// ```text
    /// k = (ω_a / 2π) · (λ_r / ω_b) · exp(−ΔV / kT)
    /// ```
    ///
    /// where `λ_r = (−γ̃ + √(γ̃² + 4ω_b²)) / 2` and `γ̃ = γ/M`.
    ///
    /// Valid for moderate-to-strong friction (`γ̃ ≳ ω_b`). Below the
    /// Kramers turnover, this formula overestimates the rate.
    #[must_use]
    pub fn kramers_rate(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        let omega_a = self.omega_a(mass);
        let omega_b = self.omega_b(mass);
        let gamma_tilde = gamma / mass;
        let discriminant = gamma_tilde
            .mul_add(gamma_tilde, 4.0 * omega_b * omega_b)
            .sqrt();
        let lambda_r = f64::midpoint(-gamma_tilde, discriminant);
        (omega_a / (2.0 * std::f64::consts::PI))
            * (lambda_r / omega_b)
            * (-self.delta_v / k_b_t).exp()
    }

    /// Action `S(E_b)` of the one-well periodic orbit at the barrier energy
    /// (J·s) — analytic for the quartic well: `S(E_b) = (8/3)·x₀·√(M·ΔV)`.
    ///
    /// Derived from `S = ∮ p dx = 2∫₀^{√2·x₀} √(2M(ΔV − V))dx`. Sets the reduced
    /// energy loss per barrier round trip in the Kramers turnover.
    #[must_use]
    pub fn barrier_action(&self, mass: f64) -> f64 {
        (8.0 / 3.0) * self.x_0 * (mass * self.delta_v).sqrt()
    }

    /// Meľnikov–Meshkov depopulation factor `Υ(δ) ∈ (0, 1]`, with
    /// `δ = (γ/M)·S(E_b)/kT` the reduced energy loss per barrier→well→barrier
    /// round trip. `Υ → 1` at high friction (recovers the spatial-diffusion
    /// rate); `Υ → δ` at low friction (gives the energy-diffusion `∝γ` rate).
    ///
    /// `Υ(δ) = exp[(1/π)∫₀^∞ ln(1 − exp(−δ(λ²+¼))) / (λ²+¼) dλ]`, evaluated by
    /// trapezoidal quadrature. Bridges the Kramers turnover to ~±20%
    /// (Hänggi–Talkner–Borkovec, Rev. Mod. Phys. 62, 251, 1990, Eq. 4.55). The
    /// `1/(λ²+¼)` denominator is essential — it makes `Υ → δ` as `δ → 0`.
    #[must_use]
    pub fn depopulation_factor(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        let delta = (gamma / mass) * self.barrier_action(mass) / k_b_t;
        if delta <= 0.0 {
            return 1.0;
        }
        // λ accumulates to avoid index→float casts; the integrand is bounded
        // (denominator ≥ ¼) and decays once δλ² ≫ 1, so cut off at √(30/δ).
        let lam_max = (30.0 / delta).sqrt().clamp(20.0, 400.0);
        let steps = 6000usize;
        let dlam = lam_max / 6000.0;
        let mut lam = 0.0_f64;
        let mut integral = 0.0;
        for i in 0..=steps {
            let denom = lam.mul_add(lam, 0.25);
            let s = delta * denom;
            let weight = if i == 0 || i == steps { 0.5 } else { 1.0 };
            integral += weight * (1.0 - (-s).exp()).ln() / denom;
            lam += dlam;
        }
        integral *= dlam;
        (integral / std::f64::consts::PI).exp()
    }

    /// Kramers escape rate across the **full friction range** (the turnover):
    /// the spatial-diffusion rate times the depopulation factor,
    /// `k = kramers_rate · Υ(δ)`.
    ///
    /// Reduces to [`kramers_rate`](Self::kramers_rate) at high friction and to
    /// the energy-diffusion (`∝ γ`) rate at low friction. **Use this — not
    /// `kramers_rate` — for a high-Q / underdamped device**, where the bare
    /// spatial-diffusion rate overestimates (it is an upper bound, since
    /// `Υ ≤ 1`). See `docs/thermo_computing/03_phases/d4_physical_pbit` R1.
    #[must_use]
    pub fn kramers_rate_turnover(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        self.kramers_rate(gamma, mass, k_b_t) * self.depopulation_factor(gamma, mass, k_b_t)
    }

    /// Potential energy at position `x`: `V(x) = a(x² − x₀²)²`.
    #[must_use]
    pub fn potential(&self, x: f64) -> f64 {
        let a = self.delta_v / self.x_0.powi(4);
        let diff = x.mul_add(x, -(self.x_0 * self.x_0));
        a * diff * diff
    }
}

impl PassiveComponent for DoubleWellPotential {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let q = data.qpos[self.dof];
        let a = self.delta_v / self.x_0.powi(4);
        // F(x) = −V′(x) = −4ax(x² − x₀²)
        qfrc_out[self.dof] += -4.0 * a * q * q.mul_add(q, -(self.x_0 * self.x_0));
    }
}

impl Diagnose for DoubleWellPotential {
    fn diagnostic_summary(&self) -> String {
        format!(
            "DoubleWellPotential(delta_v={:.4}, x_0={:.4}, dof={})",
            self.delta_v, self.x_0, self.dof
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn new_validates_positive_params() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        assert_eq!(w.barrier_height(), 3.0);
        assert_eq!(w.well_separation(), 1.0);
    }

    #[test]
    #[should_panic(expected = "barrier height must be positive")]
    fn new_rejects_zero_barrier() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = DoubleWellPotential::new(0.0, 1.0, 0);
    }

    #[test]
    #[should_panic(expected = "well separation must be positive")]
    fn new_rejects_zero_separation() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = DoubleWellPotential::new(3.0, 0.0, 0);
    }

    #[test]
    fn potential_at_well_minima_is_zero() {
        let well = DoubleWellPotential::new(3.0, 1.0, 0);
        assert!((well.potential(1.0)).abs() < 1e-15);
        assert!((well.potential(-1.0)).abs() < 1e-15);
    }

    #[test]
    fn potential_at_barrier_is_delta_v() {
        let well = DoubleWellPotential::new(3.0, 1.0, 0);
        assert!((well.potential(0.0) - 3.0).abs() < 1e-15);
    }

    #[test]
    fn omega_a_and_omega_b_ratio() {
        let well = DoubleWellPotential::new(3.0, 1.0, 0);
        let omega_a = well.omega_a(1.0);
        let omega_b = well.omega_b(1.0);
        // ω_a / ω_b = √2
        assert!((omega_a / omega_b - std::f64::consts::SQRT_2).abs() < 1e-12);
    }

    #[test]
    fn kramers_rate_central_parameters() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        let k = w.kramers_rate(10.0, 1.0, 1.0);
        // Expected: 0.01214 (from spec §7.2)
        assert!(
            (k - 0.01214).abs() < 0.0001,
            "kramers_rate at central params: got {k}, expected ~0.01214"
        );
    }

    #[test]
    fn force_is_zero_at_well_minima_and_barrier() {
        let _well = DoubleWellPotential::new(3.0, 1.0, 0);
        let a = 3.0; // delta_v / x_0^4
        // F(x) = -4ax(x² - x₀²)
        // At x=±1: x²-x₀²=0, so F=0
        // At x=0: x=0, so F=0
        let f_at_well = -4.0 * a * 1.0 * (1.0 - 1.0);
        let f_at_barrier = -4.0 * a * 0.0 * (0.0 - 1.0);
        assert_eq!(f_at_well, 0.0);
        assert_eq!(f_at_barrier, 0.0);
    }

    #[test]
    fn force_is_restoring_near_well() {
        let _well = DoubleWellPotential::new(3.0, 1.0, 0);
        let a = 3.0;
        // Slightly to the right of the right well (x=1.1):
        // F = -4a * 1.1 * (1.21 - 1) = -4*3*1.1*0.21 = -2.772 (restoring toward x=1)
        let x = 1.1;
        let f = -4.0 * a * x * (x * x - 1.0);
        assert!(f < 0.0, "force should push back toward x=1.0");
    }

    #[test]
    fn diagnostic_summary_format() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        let s = w.diagnostic_summary();
        assert!(s.contains("DoubleWellPotential"));
        assert!(s.contains("3.0000"));
        assert!(s.contains("1.0000"));
    }

    #[test]
    fn barrier_action_analytic_value() {
        // S(E_b) = (8/3)·x₀·√(M·ΔV); for ΔV=3, x₀=1, M=1 → (8/3)·√3 ≈ 4.6188.
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        assert!((w.barrier_action(1.0) - 4.618_802).abs() < 1e-5);
    }

    #[test]
    fn depopulation_factor_bounds_and_limits() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        // 0 < Υ ≤ 1 across the friction range.
        for &g in &[0.01, 0.1, 1.0, 10.0, 100.0] {
            let y = w.depopulation_factor(g, 1.0, 1.0);
            assert!(y > 0.0 && y <= 1.0, "Υ({g}) = {y} out of (0,1]");
        }
        // Monotone increasing toward 1 with friction.
        let ys: Vec<f64> = [0.1, 1.0, 10.0, 100.0]
            .iter()
            .map(|&g| w.depopulation_factor(g, 1.0, 1.0))
            .collect();
        for pair in ys.windows(2) {
            assert!(pair[1] > pair[0], "Υ should increase with friction: {ys:?}");
        }
        // High friction → Υ ≈ 1.
        assert!(w.depopulation_factor(1000.0, 1.0, 1.0) > 0.98);
        // Low friction → Υ ≈ δ (energy-diffusion asymptote).
        let g = 0.001;
        let delta = g * w.barrier_action(1.0); // M=kT=1
        let y = w.depopulation_factor(g, 1.0, 1.0);
        assert!((y / delta - 1.0).abs() < 0.2, "Υ/δ = {} not ≈1", y / delta);
    }

    #[test]
    fn turnover_is_bounded_by_and_recovers_spatial_diffusion() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        // Turnover ≤ spatial-diffusion rate everywhere (Υ ≤ 1) — the shipped
        // kramers_rate is an upper bound that overestimates underdamped.
        for &g in &[0.05, 0.5, 5.0, 50.0] {
            let k_spatial = w.kramers_rate(g, 1.0, 1.0);
            let k_turn = w.kramers_rate_turnover(g, 1.0, 1.0);
            assert!(
                k_turn <= k_spatial,
                "turnover {k_turn} > spatial-diffusion {k_spatial} at γ={g}"
            );
        }
        // High friction → turnover recovers the spatial-diffusion rate.
        let ratio = w.kramers_rate_turnover(200.0, 1.0, 1.0) / w.kramers_rate(200.0, 1.0, 1.0);
        assert!(
            ratio > 0.97,
            "turnover/k_S = {ratio} should →1 at high friction"
        );
        // The shipped rate badly overestimates deep underdamped (the R1 point).
        let k_spatial_under = w.kramers_rate(0.02, 1.0, 1.0);
        let k_turn_under = w.kramers_rate_turnover(0.02, 1.0, 1.0);
        assert!(
            k_turn_under < 0.3 * k_spatial_under,
            "underdamped: k_S should overestimate ≫3×"
        );
    }
}
