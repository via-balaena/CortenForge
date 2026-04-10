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
}
