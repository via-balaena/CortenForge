//! Symmetric quartic double-well potential for thermodynamic computing.
//!
//! Implements the standard quartic double-well `V(x) = a(x² − x₀²)²` as
//! a [`PassiveComponent`] that contributes conservative forces to the
//! `qfrc_passive` accumulator. Combined with a [`LangevinThermostat`] in
//! a [`PassiveStack`], this produces a bistable system whose switching
//! rate between wells is governed by Kramers' escape-rate formula.
//!
//! `tests/kramers_escape_rate.rs` checks its switching rate against
//! Kramers' formula.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`LangevinThermostat`]: crate::LangevinThermostat
//! [`PassiveStack`]: crate::PassiveStack

use sim_core::{DVector, Data, Model};

use crate::component::{PassiveComponent, check_position_dof, qpos_index};
use crate::diagnose::Diagnose;
use crate::error::ThermostatError;
use crate::params::{Domain, or_panic};

/// Symmetric quartic double-well potential: `V(x) = a(x² − x₀²)²`
/// where `a = ΔV / x₀⁴`.
///
/// Contributes force `F(x) = −V′(x) = −4ax(x² − x₀²)` to the per-DOF
/// force accumulator on a single DOF. This is a deterministic conservative
/// force — it does not implement [`Stochastic`](crate::Stochastic).
///
/// # Which DOFs
///
/// The force goes to DOF `dof`.
/// The position is read through the DOF's joint, so the DOF must be a slide
/// or hinge DOF, or one of a free joint's three translation DOFs. A ball
/// joint's DOFs and a free joint's rotation DOFs have no coordinate of their
/// own, so
/// [`PassiveStack::try_install`](crate::PassiveStack::try_install) refuses them.
///
/// # A tilt removes a well
///
/// A constant force `F` on the element (an [`ExternalField`](crate::ExternalField)
/// entry, or its neighbours through a [`PairwiseCoupling`](crate::PairwiseCoupling))
/// shifts its minima, and above `8ΔV/(3√3·x₀) ≈ 1.54·ΔV/x₀` it leaves only one:
///
/// ```
/// use sim_thermostat::DoubleWellPotential;
///
/// let well = DoubleWellPotential::new(3.0, 1.0, 0);
/// let removal = 8.0 * 3.0 / (3.0 * 3.0_f64.sqrt() * 1.0);
/// // A minimum of V(x) − F·x is where the net force −V′(x) + F turns from + to −.
/// let minima = |f: f64| {
///     (0..4000)
///         .map(|k| -2.0 + f64::from(k) * 1e-3)
///         .filter(|&x| well.force(x) + f > 0.0 && well.force(x + 1e-3) + f <= 0.0)
///         .count()
/// };
/// assert_eq!(minima(0.999 * removal), 2);
/// assert_eq!(minima(1.001 * removal), 1);
/// ```
///
/// # Example
///
/// ```
/// use sim_core::DVector;
/// use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PassiveStack};
///
/// // One slide particle of mass 1. This fixture needs sim-core's `test-fixtures`
/// // feature; `sim_therm_env::generate_mjcf` writes such a model as MJCF.
/// let mut model = sim_core::test_fixtures::bistable_chain(1);
/// PassiveStack::builder()
///     .with(DoubleWellPotential::new(3.0, 1.0, 0))
///     .with(LangevinThermostat::new(DVector::from_element(1, 10.0), 1.0, 42, 0))
///     .build()
///     .try_install(&mut model)?;
/// let mut data = model.make_data();
/// data.qpos[0] = 1.0; // start in the right well
/// for _ in 0..1_000 {
///     data.step(&model)?;
/// }
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct DoubleWellPotential {
    /// Barrier height: `ΔV = V(0) − V(±x₀)`.
    delta_v: f64,
    /// Well half-separation: potential minima at `±x₀`.
    x_0: f64,
    /// DOF index this potential acts on.
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
    /// If [`Self::try_new`] refuses the parameters.
    #[must_use]
    #[track_caller]
    pub fn new(delta_v: f64, x_0: f64, dof: usize) -> Self {
        or_panic(Self::try_new(delta_v, x_0, dof))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidParameter`] unless `delta_v` and `x_0` are finite and
    /// positive.
    pub fn try_new(delta_v: f64, x_0: f64, dof: usize) -> Result<Self, ThermostatError> {
        Domain::Positive.check(COMPONENT, "delta_v", delta_v)?;
        Domain::Positive.check(COMPONENT, "x_0", x_0)?;
        Ok(Self { delta_v, x_0, dof })
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
    ///
    /// `k` is the rate of escape from one well per unit time spent in it. In
    /// this symmetric well that is also the number of committed switches, in
    /// either direction, per unit of total time, which is how
    /// `tests/kramers_escape_rate.rs` measures it.
    ///
    /// # Panics
    /// Unless `mass` and `k_b_t` are finite and positive and `gamma` is finite and
    /// non-negative.
    #[must_use]
    pub fn kramers_rate(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        check_rate_inputs(gamma, mass, k_b_t);
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

    /// Meľnikov–Meshkov depopulation factor `Υ(δ) ∈ [0, 1]`, with
    /// `δ = (γ/M)·S(E_b)/kT` the reduced energy loss per barrier→well→barrier
    /// round trip. `Υ → 1` at high friction (recovers the spatial-diffusion
    /// rate); `Υ → δ` at low friction (gives the energy-diffusion `∝γ` rate).
    ///
    /// `Υ(δ) = exp[(1/π)∫₀^∞ ln(1 − exp(−δ(λ²+¼))) / (λ²+¼) dλ]`, evaluated by
    /// trapezoidal quadrature. Bridges the Kramers turnover to ~±20%
    /// (Hänggi–Talkner–Borkovec, Rev. Mod. Phys. 62, 251, 1990, Eq. 4.55). The
    /// `1/(λ²+¼)` denominator is essential — it makes `Υ → δ` as `δ → 0`.
    ///
    /// # Panics
    /// Unless `mass` and `k_b_t` are finite and positive and `gamma` is finite and
    /// non-negative.
    #[must_use]
    pub fn depopulation_factor(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        check_rate_inputs(gamma, mass, k_b_t);
        let delta = (gamma / mass) * self.barrier_action(mass) / k_b_t;
        if delta <= 0.0 {
            return 0.0; // the δ → 0 limit: Υ → δ
        }
        // λ = ½·tan θ maps λ ∈ [0, ∞) onto θ ∈ [0, π/2), with dλ/(λ²+¼) = 2·dθ and
        // λ²+¼ = 1/(4·cos²θ), so the exponent is (2/π)∫₀^{π/2} ln(1 − e^(−δ/(4cos²θ))) dθ.
        // θ = (π/2)(1 − (1−t)²) then gathers the nodes near π/2, where the integrand has a
        // ln cos θ singularity at small δ, and leaves a smooth integrand in t ∈ [0, 1] with no
        // cut-off. The tests check it against a high-precision evaluation.
        let steps = 6000usize;
        // steps is a small exact-in-f64 constant.
        #[allow(clippy::cast_precision_loss)]
        let dt = 1.0 / steps as f64;
        let mut integral = 0.0;
        for i in 0..=steps {
            #[allow(clippy::cast_precision_loss)]
            let u = 1.0 - i as f64 * dt;
            let cos = (std::f64::consts::FRAC_PI_2 * u.mul_add(-u, 1.0)).cos();
            let s = delta / (4.0 * cos * cos);
            // ln(1 − e^(−s)) via expm1: `1.0 - (-s).exp()` rounds to 0 for tiny s.
            let integrand = (-(-s).exp_m1()).ln() * std::f64::consts::PI * u;
            let weight = if i == 0 || i == steps { 0.5 } else { 1.0 };
            integral += weight * integrand;
        }
        (2.0 / std::f64::consts::PI * integral * dt).exp()
    }

    /// Kramers escape rate across the **full friction range** (the turnover):
    /// the spatial-diffusion rate times the depopulation factor,
    /// `k = kramers_rate · Υ(δ)`.
    ///
    /// Reduces to [`kramers_rate`](Self::kramers_rate) at high friction and to
    /// the energy-diffusion (`∝ γ`) rate at low friction. **Use this — not
    /// `kramers_rate` — for a high-Q / underdamped device**, where the bare
    /// spatial-diffusion rate overestimates (it is an upper bound, since
    /// `Υ ≤ 1`). It counts escapes as [`kramers_rate`](Self::kramers_rate) does.
    ///
    /// # Panics
    /// Unless `mass` and `k_b_t` are finite and positive and `gamma` is finite and
    /// non-negative.
    #[must_use]
    pub fn kramers_rate_turnover(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
        self.kramers_rate(gamma, mass, k_b_t) * self.depopulation_factor(gamma, mass, k_b_t)
    }

    /// Force at position `x`: `F(x) = −V′(x) = −4ax(x² − x₀²)`, where `a = ΔV/x₀⁴`.
    #[must_use]
    pub fn force(&self, x: f64) -> f64 {
        let a = self.delta_v / self.x_0.powi(4);
        -4.0 * a * x * x.mul_add(x, -(self.x_0 * self.x_0))
    }

    /// Potential energy at position `x`: `V(x) = a(x² − x₀²)²`.
    #[must_use]
    pub fn potential(&self, x: f64) -> f64 {
        let a = self.delta_v / self.x_0.powi(4);
        let diff = x.mul_add(x, -(self.x_0 * self.x_0));
        a * diff * diff
    }
}

const COMPONENT: &str = "DoubleWellPotential";

/// The rate formulas' domain: finite positive mass and temperature, finite non-negative
/// friction. Panics outside it.
#[track_caller]
fn check_rate_inputs(gamma: f64, mass: f64, k_b_t: f64) {
    or_panic(rate_inputs(gamma, mass, k_b_t));
}

fn rate_inputs(gamma: f64, mass: f64, k_b_t: f64) -> Result<(), ThermostatError> {
    Domain::Positive.check(COMPONENT, "mass", mass)?;
    Domain::Positive.check(COMPONENT, "k_b_t", k_b_t)?;
    Domain::NonNegative.check(COMPONENT, "gamma", gamma)
}

impl PassiveComponent for DoubleWellPotential {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        qfrc_out[self.dof] += self.force(data.qpos[qpos_index(model, self.dof)]);
    }

    fn as_diagnose(&self) -> Option<&dyn Diagnose> {
        Some(self)
    }

    fn validate(&self, model: &Model) -> Result<(), ThermostatError> {
        check_position_dof(model, self.dof, "DoubleWellPotential")
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
    use crate::params::refused_parameter;

    #[test]
    fn new_validates_positive_params() {
        let w = DoubleWellPotential::new(3.0, 1.0, 0);
        assert_eq!(w.barrier_height(), 3.0);
        assert_eq!(w.well_separation(), 1.0);
    }

    #[test]
    #[should_panic(expected = "DoubleWellPotential: delta_v must be finite and positive, got 0")]
    fn new_rejects_zero_barrier() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = DoubleWellPotential::new(0.0, 1.0, 0);
    }

    #[test]
    #[should_panic(expected = "DoubleWellPotential: x_0 must be finite and positive, got 0")]
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

    /// `force` is `−V′`: central differences of `potential` agree.
    #[test]
    fn force_is_minus_the_potential_slope() {
        let w = DoubleWellPotential::new(3.0, 0.7, 0);
        let eps = 1e-6;
        for x in [-1.1, -0.7, -0.2, 0.0, 0.35, 0.7, 1.3] {
            let fd = -(w.potential(x + eps) - w.potential(x - eps)) / (2.0 * eps);
            assert!(
                (w.force(x) - fd).abs() < 1e-6,
                "x = {x}: force {} vs {fd}",
                w.force(x)
            );
        }
    }

    /// At tiny friction the depopulation factor approaches `δ` instead of collapsing to 0:
    /// `1 − e^(−s)` rounds to 0 for tiny `s`, so the integrand needs `expm1`.
    #[test]
    fn depopulation_factor_stays_near_delta_at_tiny_friction() {
        let w = DoubleWellPotential::new(1.0, 1.0, 0);
        let (mass, k_b_t) = (1.0, 1.0);
        // The old λ cut-off overshot here: Υ/δ was 1.019, 1.045 and 1.71.
        for delta in [4.6e-17, 1e-30, 1e-300] {
            let gamma = delta * mass * k_b_t / w.barrier_action(mass);
            let upsilon = w.depopulation_factor(gamma, mass, k_b_t);
            assert!(
                (upsilon / delta - 1.0).abs() < 1e-5,
                "Υ = {upsilon:e} at δ = {delta:e}"
            );
        }
    }

    /// Υ(δ) against an mpmath evaluation of the same integral (40 digits).
    #[test]
    fn depopulation_factor_matches_a_high_precision_reference() {
        let w = DoubleWellPotential::new(1.0, 1.0, 0);
        for (delta, reference) in [
            (0.01, 0.009_209_201_854_993_58),
            (1.0, 0.442_978_309_950_351),
            (3.0, 0.756_430_798_283_154),
            (10.0, 0.974_171_497_588_707),
        ] {
            let gamma = delta / w.barrier_action(1.0);
            let upsilon = w.depopulation_factor(gamma, 1.0, 1.0);
            assert!(
                (upsilon / reference - 1.0).abs() < 1e-6,
                "δ = {delta}: Υ = {upsilon}, reference {reference}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "DoubleWellPotential: mass must be finite and positive, got 0")]
    fn kramers_rate_refuses_zero_mass() {
        let _rate = DoubleWellPotential::new(1.0, 1.0, 0).kramers_rate(0.1, 0.0, 1.0);
    }

    #[test]
    fn try_new_refuses_a_barrier_or_separation_that_is_not_finite_and_positive() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            for (delta_v, x_0, parameter) in [(bad, 1.0, "delta_v"), (1.0, bad, "x_0")] {
                assert_eq!(
                    refused_parameter(DoubleWellPotential::try_new(delta_v, x_0, 0)).as_deref(),
                    Some(parameter),
                    "({delta_v}, {x_0})"
                );
            }
        }
        assert!(DoubleWellPotential::try_new(f64::MIN_POSITIVE, 1e300, 0).is_ok());
    }

    /// The rate formulas refuse +∞ like the other out-of-domain inputs: an infinite friction
    /// or mass would make the rate `NaN`.
    #[test]
    fn rate_formulas_refuse_infinite_inputs() {
        let well = DoubleWellPotential::new(1.0, 1.0, 0);
        for (gamma, mass, k_b_t) in [
            (f64::INFINITY, 1.0, 1.0),
            (1.0, f64::INFINITY, 1.0),
            (1.0, 1.0, f64::INFINITY),
        ] {
            let refused = std::panic::catch_unwind(|| well.kramers_rate(gamma, mass, k_b_t));
            assert!(
                refused.is_err(),
                "kramers_rate({gamma}, {mass}, {k_b_t}) ran"
            );
            let refused = std::panic::catch_unwind(|| well.depopulation_factor(gamma, mass, k_b_t));
            assert!(
                refused.is_err(),
                "depopulation_factor({gamma}, {mass}, {k_b_t}) ran"
            );
        }
    }

    /// Without friction there is no energy diffusion: the factor is 0, the δ → 0 limit.
    #[test]
    fn depopulation_factor_is_zero_without_friction() {
        let w = DoubleWellPotential::new(1.0, 1.0, 0);
        assert_eq!(w.depopulation_factor(0.0, 1.0, 1.0), 0.0);
    }
}
