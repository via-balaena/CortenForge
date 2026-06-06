//! Magnetoelastic forward model (D4 Layer-2 R3).
//!
//! Predicts the **real** double-well potential of the rig — a spring-steel
//! cantilever whose tip magnet sits between two fixed magnets (Moon & Holmes
//! 1979) — so the rig is spec'd from physics instead of guessed. The tip
//! deflects laterally by `x`; the potential is
//!
//! ```text
//! U(x) = ½·k_beam·x²  −  k_mag/r₊(x)³  −  k_mag/r₋(x)³
//! ```
//!
//! elastic restoring (centring) competing with dipole attraction to each fixed
//! magnet (off-centring). Above a critical magnet strength/proximity the centre
//! goes unstable and two wells appear — the bistable p-bit. From `U(x)` we read
//! `ΔV`, the well separation `x₀`, and the curvatures `ω_a`, `ω_b` that feed the
//! Kramers rate, and we can ask the design question: what magnet/beam/gap, driven
//! to what RMS tip amplitude, lands `ΔV/kT_eff` in a measurable window.
//!
//! **Model fidelity / caveats** (this is a *design* model, good to a factor, not
//! a metrology model):
//! - Point-dipole magnets with a fixed `orientation_factor` (≈2 for head-to-tail
//!   attraction) — the dominant uncertainty; real magnets are extended, so treat
//!   `ΔV` as ±~2×.
//! - Small-deflection linear beam (`k_beam = 3EI/L³`); the magnetic nonlinearity
//!   dominates the well structure anyway.
//! - The friction regime (underdamped → Kramers turnover) is **R1**; this module
//!   reports the regime-independent quantities + the Arrhenius/TST bound.
//! - The `kT_eff ↔ shaker drive` link assumes a thermal effective temperature,
//!   which **R2** validates.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::must_use_candidate,
    clippy::doc_markdown
)]

use std::f64::consts::PI;

/// Vacuum permeability μ₀ (T·m/A).
const MU0: f64 = 4.0e-7 * PI;

/// A cylindrical (disc) permanent magnet.
#[derive(Debug, Clone, Copy)]
pub struct Magnet {
    /// Remanence `B_r` (T) — ~1.32 for N52.
    pub remanence_t: f64,
    /// Disc radius (m).
    pub radius_m: f64,
    /// Disc thickness (m).
    pub thickness_m: f64,
    /// Material density (kg/m³) — ~7500 for sintered NdFeB.
    pub density_kgm3: f64,
}

impl Magnet {
    /// Magnet volume (m³).
    pub fn volume_m3(&self) -> f64 {
        PI * self.radius_m * self.radius_m * self.thickness_m
    }

    /// Magnetic dipole moment `m = B_r·V/μ₀` (A·m²).
    pub fn moment_am2(&self) -> f64 {
        self.remanence_t * self.volume_m3() / MU0
    }

    /// Magnet mass (kg).
    pub fn mass_kg(&self) -> f64 {
        self.density_kgm3 * self.volume_m3()
    }
}

/// A spring-steel cantilever beam (rectangular cross-section, thickness in the
/// bending/snap direction).
#[derive(Debug, Clone, Copy)]
pub struct Beam {
    /// Free length (m).
    pub length_m: f64,
    /// Width, transverse to bending (m).
    pub width_m: f64,
    /// Thickness, along the bending/snap direction (m).
    pub thickness_m: f64,
    /// Young's modulus (Pa) — ~200e9 for spring steel.
    pub youngs_pa: f64,
    /// Material density (kg/m³) — ~7850 for steel.
    pub density_kgm3: f64,
}

impl Beam {
    /// Second moment of area `I = w·t³/12` (m⁴).
    pub fn second_moment_m4(&self) -> f64 {
        self.width_m * self.thickness_m.powi(3) / 12.0
    }

    /// Tip lateral stiffness `k = 3EI/L³` (N/m).
    pub fn tip_stiffness_npm(&self) -> f64 {
        3.0 * self.youngs_pa * self.second_moment_m4() / self.length_m.powi(3)
    }

    /// Beam mass (kg).
    pub fn mass_kg(&self) -> f64 {
        self.density_kgm3 * self.width_m * self.thickness_m * self.length_m
    }
}

/// The full magnetoelastic configuration.
#[derive(Debug, Clone, Copy)]
pub struct Rig {
    /// The cantilever.
    pub beam: Beam,
    /// Magnet carried on the beam tip.
    pub tip: Magnet,
    /// Each of the two fixed magnets.
    pub fixed: Magnet,
    /// Axial gap from the (undeflected) tip to the fixed-magnet plane (m).
    pub gap_axial_m: f64,
    /// Lateral half-spread of the two fixed magnets, at `±spread` (m).
    pub spread_lateral_m: f64,
    /// Dipole orientation factor (≈2 head-to-tail attractive). Dominant model
    /// uncertainty — treat resulting `ΔV` as ±~2×.
    pub orientation_factor: f64,
}

impl Rig {
    /// Dipole interaction prefactor `k_mag = (μ₀/4π)·C·m_tip·m_fixed` (J·m³), so
    /// the attraction energy to one magnet is `−k_mag/r³`.
    fn k_mag(&self) -> f64 {
        (MU0 / (4.0 * PI))
            * self.orientation_factor
            * self.tip.moment_am2()
            * self.fixed.moment_am2()
    }

    /// First-bending-mode effective tip mass `≈ 0.2427·m_beam + m_tip` (kg).
    pub fn modal_mass_kg(&self) -> f64 {
        0.2427 * self.beam.mass_kg() + self.tip.mass_kg()
    }

    /// Potential energy `U(x)` (J) at lateral tip displacement `x` (m).
    /// Symmetric: `U(x) = U(−x)`.
    pub fn potential_j(&self, x: f64) -> f64 {
        let k = self.beam.tip_stiffness_npm();
        let a = self.spread_lateral_m;
        let g = self.gap_axial_m;
        let km = self.k_mag();
        let rp = (a - x).hypot(g);
        let rm = (a + x).hypot(g);
        0.5 * k * x * x - km / rp.powi(3) - km / rm.powi(3)
    }

    /// Locate the wells and barrier and report the derived quantities.
    pub fn analyze(&self) -> WellModel {
        let m_eff = self.modal_mass_kg();
        let k_beam = self.beam.tip_stiffness_npm();
        // Curvature at the centre (symmetric → use ±dx).
        let dx = (self.spread_lateral_m + self.gap_axial_m) * 1e-4;
        let u0 = self.potential_j(0.0);
        let curv_centre = 2.0 * (self.potential_j(dx) - u0) / (dx * dx);

        // Bistable when the centre is a maximum (negative curvature).
        if curv_centre >= 0.0 {
            let omega_a = (curv_centre / m_eff).sqrt();
            return WellModel {
                bistable: false,
                x0_m: 0.0,
                delta_v_j: 0.0,
                omega_a,
                omega_b: 0.0,
                attempt_freq_hz: omega_a / (2.0 * PI),
                k_beam_npm: k_beam,
                modal_mass_kg: m_eff,
            };
        }

        // Scan x>0 for the well minimum.
        let x_hi = self.spread_lateral_m * 1.3 + self.gap_axial_m;
        let n = 4000usize;
        let mut x0 = 0.0;
        let mut u_min = u0;
        for i in 1..=n {
            let x = x_hi * i as f64 / n as f64;
            let u = self.potential_j(x);
            if u < u_min {
                u_min = u;
                x0 = x;
            }
        }

        let curv_well = {
            let h = x_hi / n as f64;
            (self.potential_j(x0 + h) + self.potential_j(x0 - h) - 2.0 * self.potential_j(x0))
                / (h * h)
        };
        let omega_a = (curv_well.max(0.0) / m_eff).sqrt();
        let omega_b = (-curv_centre / m_eff).sqrt();

        WellModel {
            bistable: true,
            x0_m: x0,
            delta_v_j: u0 - u_min,
            omega_a,
            omega_b,
            attempt_freq_hz: omega_a / (2.0 * PI),
            k_beam_npm: k_beam,
            modal_mass_kg: m_eff,
        }
    }
}

/// Derived double-well quantities for a [`Rig`].
#[derive(Debug, Clone, Copy)]
pub struct WellModel {
    /// Two stable wells exist (centre is unstable).
    pub bistable: bool,
    /// Well minimum location `x₀` (m); 0 if not bistable.
    pub x0_m: f64,
    /// Barrier height `ΔV` (J); 0 if not bistable.
    pub delta_v_j: f64,
    /// Angular frequency at a well bottom (rad/s).
    pub omega_a: f64,
    /// Angular frequency at the barrier (rad/s); 0 if not bistable.
    pub omega_b: f64,
    /// Attempt frequency `ω_a/2π` (Hz) — the Kramers prefactor scale.
    pub attempt_freq_hz: f64,
    /// Beam tip stiffness (N/m).
    pub k_beam_npm: f64,
    /// Effective modal mass (kg).
    pub modal_mass_kg: f64,
}

impl WellModel {
    /// Effective temperature (J) giving a target `ΔV/kT_eff` ratio.
    pub fn kt_for_ratio(&self, ratio: f64) -> f64 {
        self.delta_v_j / ratio
    }

    /// RMS in-well tip amplitude (m) at effective temperature `kt_j`, from
    /// equipartition `½·m·ω_a²·⟨δx²⟩ = ½·kT_eff`.
    pub fn rms_amplitude_m(&self, kt_j: f64) -> f64 {
        if self.omega_a <= 0.0 {
            return f64::NAN;
        }
        (kt_j / (self.modal_mass_kg * self.omega_a * self.omega_a)).sqrt()
    }

    /// Arrhenius / TST attempt rate `(ω_a/2π)·exp(−ΔV/kT_eff)` (Hz). This is an
    /// **upper bound**; the true rate is this times a transmission factor ≤ 1
    /// set by the friction regime (D4 Layer-2 R1, the Kramers turnover).
    pub fn arrhenius_rate_hz(&self, kt_j: f64) -> f64 {
        self.attempt_freq_hz * (-self.delta_v_j / kt_j).exp()
    }

    /// Damping coefficient (kg/s) implied by a quality factor `Q`:
    /// `γ = m·ω_a/Q`. Spring steel is high-Q (underdamped) → small γ → R1 regime.
    pub fn gamma_for_q(&self, q: f64) -> f64 {
        self.modal_mass_kg * self.omega_a / q
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    fn n52(radius_m: f64, thickness_m: f64) -> Magnet {
        Magnet {
            remanence_t: 1.32,
            radius_m,
            thickness_m,
            density_kgm3: 7500.0,
        }
    }

    fn spring_steel_beam(length_m: f64, thickness_m: f64) -> Beam {
        Beam {
            length_m,
            width_m: 6e-3,
            thickness_m,
            youngs_pa: 200e9,
            density_kgm3: 7850.0,
        }
    }

    #[test]
    fn magnet_moment_sane_for_6x3_n52() {
        // A 6 mm Ø × 3 mm N52 disc has a moment of order 0.08–0.10 A·m².
        let m = n52(3e-3, 3e-3).moment_am2();
        assert!((0.05..0.15).contains(&m), "moment {m}");
    }

    #[test]
    fn thin_beam_is_compliant() {
        // 40 mm × 6 mm × 0.1 mm spring steel → a few N/m.
        let k = spring_steel_beam(40e-3, 0.1e-3).tip_stiffness_npm();
        assert!((1.0..20.0).contains(&k), "k_beam {k}");
    }

    #[test]
    fn potential_is_symmetric() {
        let rig = Rig {
            beam: spring_steel_beam(40e-3, 0.1e-3),
            tip: n52(2e-3, 2e-3),
            fixed: n52(3e-3, 3e-3),
            gap_axial_m: 6e-3,
            spread_lateral_m: 4e-3,
            orientation_factor: 2.0,
        };
        for &x in &[1e-3, 2.5e-3, 5e-3] {
            assert!((rig.potential_j(x) - rig.potential_j(-x)).abs() < 1e-18);
        }
    }

    #[test]
    fn strong_close_magnets_are_bistable_weak_far_are_not() {
        let base = |gap: f64, spread: f64| Rig {
            beam: spring_steel_beam(45e-3, 0.08e-3),
            tip: n52(2.5e-3, 2e-3),
            fixed: n52(4e-3, 3e-3),
            gap_axial_m: gap,
            spread_lateral_m: spread,
            orientation_factor: 2.0,
        };
        // Close, well-spread magnets destabilise the centre → bistable.
        let close = base(3e-3, 5e-3).analyze();
        assert!(close.bistable, "close config should be bistable");
        assert!(close.x0_m > 0.0 && close.delta_v_j > 0.0);
        assert!(close.omega_a > 0.0 && close.omega_b > 0.0);

        // Far magnets: elastic stiffness wins → single central well.
        let far = base(40e-3, 5e-3).analyze();
        assert!(!far.bistable, "far config should be monostable");
        assert_eq!(far.delta_v_j, 0.0);
    }

    #[test]
    fn barrier_grows_as_magnets_approach() {
        let at = |gap: f64| {
            Rig {
                beam: spring_steel_beam(45e-3, 0.08e-3),
                tip: n52(2.5e-3, 2e-3),
                fixed: n52(4e-3, 3e-3),
                gap_axial_m: gap,
                spread_lateral_m: 5e-3,
                orientation_factor: 2.0,
            }
            .analyze()
            .delta_v_j
        };
        // Bringing the magnets closer deepens the wells.
        assert!(at(2.5e-3) > at(3.5e-3), "ΔV should grow as the gap shrinks");
    }

    #[test]
    fn equipartition_amplitude_and_ratio_round_trip() {
        let wm = WellModel {
            bistable: true,
            x0_m: 1e-3,
            delta_v_j: 3e-7,
            omega_a: 300.0,
            omega_b: 200.0,
            attempt_freq_hz: 300.0 / (2.0 * PI),
            k_beam_npm: 5.0,
            modal_mass_kg: 2e-4,
        };
        let kt = wm.kt_for_ratio(3.0);
        assert!((kt - 1e-7).abs() < 1e-12, "kt {kt}");
        // RMS amplitude is real and sub-mm-ish for these scales.
        let rms = wm.rms_amplitude_m(kt);
        assert!(rms > 0.0 && rms < 1e-2, "rms {rms}");
        // equipartition: m·ω_a²·rms² == kt.
        assert!((wm.modal_mass_kg * wm.omega_a * wm.omega_a * rms * rms - kt).abs() < 1e-12);
    }
}
