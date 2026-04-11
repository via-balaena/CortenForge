//! Two-harmonic ratchet potential for Brownian motor experiments.
//!
//! Implements the asymmetric periodic potential
//! `V(x) = α · [−V₁ sin(2πx/L) − V₂ sin(4πx/L + φ)]` as a
//! [`PassiveComponent`] that contributes conservative forces to the
//! `qfrc_passive` accumulator. The amplitude multiplier `α ∈ [0, 1]` is
//! read from `data.ctrl[ctrl_idx]` at each step, enabling RL control of
//! the potential via the ctrl-channel bridge pattern (spec §3).
//!
//! With `V₂ > 0` and `φ ≠ 0, nπ`, the potential is spatially asymmetric
//! within each period `L`. A flashing protocol (alternating α between 0
//! and 1) breaks detailed balance and produces directed transport — a
//! Brownian ratchet motor. With constant α (no flashing), detailed balance
//! holds and the steady-state current is exactly zero regardless of the
//! asymmetry.
//!
//! D1 of the thermodynamic computing initiative validates this component
//! in combination with a [`LangevinThermostat`] and an RL agent (CEM)
//! that discovers the flashing strategy.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`LangevinThermostat`]: crate::LangevinThermostat

use std::f64::consts::PI;

use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;
use crate::diagnose::Diagnose;

/// Two-harmonic ratchet potential with ctrl-dependent amplitude modulation.
///
/// ```text
/// V(x) = α · [ −V₁ sin(k₁x) − V₂ sin(k₂x + φ) ]
/// F(x) = α · [ V₁ k₁ cos(k₁x) + V₂ k₂ cos(k₂x + φ) ]
/// ```
///
/// where `k₁ = 2π/L`, `k₂ = 4π/L`, and `α = data.ctrl[ctrl_idx]`
/// clamped to `[0, 1]`.
///
/// # Joint type constraint
///
/// Same as [`DoubleWellPotential`](crate::DoubleWellPotential): the `dof`
/// field indexes both `data.qpos` and `qfrc_out`. This is correct for
/// slide and hinge joints where `nq = nv = 1`. It does **not** support
/// ball or free joints.
///
/// # Ctrl-channel pattern
///
/// The amplitude `α` is read from `data.ctrl[ctrl_idx]` at each physics
/// step. The MJCF model must include a zero-gain actuator at `ctrl_idx`
/// so the RL agent can write to `data.ctrl` via `ActionSpace::apply`
/// without the actuator producing any force of its own. See spec §3 and
/// §5 for the MJCF model and ctrl flow.
pub struct RatchetPotential {
    /// First harmonic amplitude V₁.
    v1: f64,
    /// Second harmonic amplitude V₂ (breaks symmetry when > 0).
    v2: f64,
    /// Phase offset φ (radians). Controls asymmetry direction.
    phi: f64,
    /// Spatial period L.
    period: f64,
    /// DOF index (= qpos index for slide/hinge joints).
    dof: usize,
    /// Index into `data.ctrl` for amplitude modulation α.
    ctrl_idx: usize,
}

impl RatchetPotential {
    /// Create a new ratchet potential.
    ///
    /// # Parameters
    /// - `v1`: first harmonic amplitude `V₁ > 0`
    /// - `v2`: second harmonic amplitude `V₂ ≥ 0` (0 = symmetric)
    /// - `phi`: phase offset in radians
    /// - `period`: spatial period `L > 0`
    /// - `dof`: DOF index (must be valid for the target model)
    /// - `ctrl_idx`: index into `data.ctrl` for amplitude modulation
    ///
    /// # Panics
    /// Panics if `v1 <= 0` or `period <= 0`.
    #[must_use]
    pub fn new(v1: f64, v2: f64, phi: f64, period: f64, dof: usize, ctrl_idx: usize) -> Self {
        assert!(v1 > 0.0, "V₁ must be positive, got {v1}");
        assert!(period > 0.0, "period must be positive, got {period}");
        Self {
            v1,
            v2,
            phi,
            period,
            dof,
            ctrl_idx,
        }
    }

    /// First harmonic amplitude V₁.
    #[must_use]
    pub const fn v1(&self) -> f64 {
        self.v1
    }

    /// Second harmonic amplitude V₂.
    #[must_use]
    pub const fn v2(&self) -> f64 {
        self.v2
    }

    /// Phase offset φ (radians).
    #[must_use]
    pub const fn phi(&self) -> f64 {
        self.phi
    }

    /// Spatial period L.
    #[must_use]
    pub const fn period(&self) -> f64 {
        self.period
    }

    /// Potential energy at position `x` with amplitude `alpha`.
    ///
    /// `V(x) = α · [−V₁ sin(k₁x) − V₂ sin(k₂x + φ)]`
    #[must_use]
    pub fn potential(&self, x: f64, alpha: f64) -> f64 {
        let k1 = 2.0 * PI / self.period;
        let k2 = 4.0 * PI / self.period;
        alpha * (-self.v1).mul_add((k1 * x).sin(), -(self.v2 * (k2 * x + self.phi).sin()))
    }

    /// Force at position `x` with amplitude `alpha`.
    ///
    /// `F(x) = −dV/dx = α · [V₁ k₁ cos(k₁x) + V₂ k₂ cos(k₂x + φ)]`
    #[must_use]
    pub fn force(&self, x: f64, alpha: f64) -> f64 {
        let k1 = 2.0 * PI / self.period;
        let k2 = 4.0 * PI / self.period;
        alpha * (self.v1 * k1).mul_add((k1 * x).cos(), self.v2 * k2 * (k2 * x + self.phi).cos())
    }
}

impl PassiveComponent for RatchetPotential {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let x = data.qpos[self.dof];
        let alpha = data.ctrl[self.ctrl_idx].clamp(0.0, 1.0);

        qfrc_out[self.dof] += self.force(x, alpha);
    }
}

impl Diagnose for RatchetPotential {
    fn diagnostic_summary(&self) -> String {
        format!(
            "RatchetPotential(V1={:.4}, V2={:.4}, phi={:.4}, L={:.4}, dof={}, ctrl={})",
            self.v1, self.v2, self.phi, self.period, self.dof, self.ctrl_idx,
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_lossless)]
mod tests {
    use super::*;

    // ── construction ────────────────────────────────────────────────────

    #[test]
    fn new_validates_positive_v1() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        assert_eq!(r.v1(), 1.0);
        assert_eq!(r.v2(), 0.25);
    }

    #[test]
    #[should_panic(expected = "V₁ must be positive")]
    fn new_rejects_zero_v1() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = RatchetPotential::new(0.0, 0.25, PI / 4.0, 1.0, 0, 0);
    }

    #[test]
    #[should_panic(expected = "period must be positive")]
    fn new_rejects_zero_period() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = RatchetPotential::new(1.0, 0.25, PI / 4.0, 0.0, 0, 0);
    }

    #[test]
    fn v2_zero_is_allowed() {
        let r = RatchetPotential::new(1.0, 0.0, 0.0, 1.0, 0, 0);
        assert_eq!(r.v2(), 0.0);
    }

    // ── force = −dV/dx (finite-difference check) ────────────────────────

    #[test]
    fn force_matches_negative_potential_derivative() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        let eps = 1e-7;

        for alpha in [0.0, 0.5, 1.0] {
            for &x in &[0.0, 0.1, 0.25, 0.5, 0.73, 0.99] {
                let fd = -(r.potential(x + eps, alpha) - r.potential(x - eps, alpha)) / (2.0 * eps);
                let analytic = r.force(x, alpha);
                assert!(
                    (fd - analytic).abs() < 1e-5,
                    "FD mismatch at x={x}, α={alpha}: fd={fd}, analytic={analytic}",
                );
            }
        }
    }

    // ── zero force when α = 0 ───────────────────────────────────────────

    #[test]
    fn zero_force_when_alpha_zero() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        for &x in &[0.0, 0.1, 0.5, 0.9, 1.5, -0.3] {
            assert_eq!(
                r.force(x, 0.0),
                0.0,
                "force should be exactly zero when α=0, got {} at x={x}",
                r.force(x, 0.0),
            );
        }
    }

    #[test]
    fn zero_potential_when_alpha_zero() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        for &x in &[0.0, 0.5, 1.0] {
            assert_eq!(r.potential(x, 0.0), 0.0);
        }
    }

    // ── symmetry breaking ───────────────────────────────────────────────

    #[test]
    fn asymmetric_potential_has_unequal_well_depths() {
        // With V₂ > 0 and φ ≠ 0, the two extrema within one period
        // have different depths.
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        let n = 1000;
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        let mut min_x = 0.0;
        for i in 0..n {
            let x = (i as f64) / (n as f64);
            let v = r.potential(x, 1.0);
            if v < min_v {
                min_v = v;
                min_x = x;
            }
            if v > max_v {
                max_v = v;
            }
        }
        // The potential has both a minimum and a maximum within [0, L)
        assert!(max_v > min_v, "potential should have variation");
        // The minimum is not at the midpoint (asymmetric)
        assert!(
            (min_x - 0.5).abs() > 0.01,
            "minimum at x={min_x} is too close to the symmetric midpoint 0.5"
        );
        // Force averages to zero over one period (conservative)
        let force_sum: f64 = (0..n)
            .map(|i| {
                let x = (i as f64) / (n as f64);
                r.force(x, 1.0)
            })
            .sum();
        let force_avg = force_sum / (n as f64);
        assert!(
            force_avg.abs() < 1e-10,
            "force should average to zero over one period, got {force_avg}"
        );
    }

    // ── symmetric case (V₂ = 0) ─────────────────────────────────────────

    #[test]
    fn symmetric_potential_when_v2_zero() {
        let r = RatchetPotential::new(1.0, 0.0, 0.0, 1.0, 0, 0);
        // V(x) = -sin(2πx). Minimum at x = L/4 = 0.25, maximum at x = 3L/4 = 0.75.
        let v_quarter = r.potential(0.25, 1.0);
        let v_three_quarter = r.potential(0.75, 1.0);
        // Minimum: V(L/4) = -sin(π/2) = -1.0
        assert!(
            (v_quarter - (-1.0)).abs() < 1e-12,
            "V(L/4) should be -1.0, got {v_quarter}"
        );
        // Maximum: V(3L/4) = -sin(3π/2) = 1.0
        assert!(
            (v_three_quarter - 1.0).abs() < 1e-12,
            "V(3L/4) should be 1.0, got {v_three_quarter}"
        );
        // Potential is symmetric: V(x) = V(L/2 - x) around the minimum
        // Check V(0.15) == V(0.35) (equidistant from 0.25)
        let v_a = r.potential(0.15, 1.0);
        let v_b = r.potential(0.35, 1.0);
        assert!(
            (v_a - v_b).abs() < 1e-12,
            "symmetric potential should have V(0.15)=V(0.35), got {v_a} vs {v_b}"
        );
    }

    // ── periodicity ─────────────────────────────────────────────────────

    #[test]
    fn potential_is_periodic() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        for &x in &[0.0, 0.1, 0.37, 0.99] {
            let v0 = r.potential(x, 1.0);
            let v1 = r.potential(x + 1.0, 1.0);
            let v2 = r.potential(x + 5.0, 1.0);
            assert!(
                (v0 - v1).abs() < 1e-12,
                "V({x}) = {v0} ≠ V({}) = {v1}",
                x + 1.0,
            );
            assert!(
                (v0 - v2).abs() < 1e-12,
                "V({x}) = {v0} ≠ V({}) = {v2}",
                x + 5.0,
            );
        }
    }

    // ── diagnostic summary ──────────────────────────────────────────────

    #[test]
    fn diagnostic_summary_format() {
        let r = RatchetPotential::new(1.0, 0.25, PI / 4.0, 1.0, 0, 0);
        let s = r.diagnostic_summary();
        assert!(s.contains("RatchetPotential"));
        assert!(s.contains("V1=1.0000"));
        assert!(s.contains("V2=0.2500"));
        assert!(s.contains("L=1.0000"));
        assert!(s.contains("dof=0"));
        assert!(s.contains("ctrl=0"));
    }
}
