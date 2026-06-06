//! Magnetostatic sensing model (D4 Layer-2 R4).
//!
//! The Hall sensor sees the **moving tip magnet** plus the **two static
//! well-defining magnets**. Because the well magnets are fixed, they contribute
//! only a constant DC offset to the reading — trivially subtractable. The design
//! problem is therefore the *x-dependence*: place the tip magnet and sensor so
//! the readout is **monotonic, sensitive, and high-resolution** over the tip's
//! operating range.
//!
//! Key result (see the tests): magnetise the tip magnet **along the motion axis**
//! and offset the Hall sensor **perpendicular** by `d`; then `B_z ∝ x/(x²+d²)^{5/2}`
//! is odd and **monotonic only for `|x| < d/2`** (it folds at the inflection
//! `x = d/2`). So `d ≳ 2·x_max`, where `x_max` is the peak excursion that must read
//! cleanly: the wells `±x₀` for which-well counting + in-well variance, larger if
//! continuous position through the transit overshoot is required. Sensitivity
//! `∝ 1/d⁴` — **~20 mT/mm at the shallow operating point** (`x₀≈2 mm`, `d≈5 mm`),
//! ~5 mT/mm at the deep end (`x₀≈3.5 mm`, `d≈7 mm`) — so `d` is chosen against the
//! magnet gap and the micron-scale resolution coarsens accordingly. The well
//! magnets add only a DC offset (tens of mT) that sets the DRV5055 range variant.

#![allow(clippy::cast_precision_loss, clippy::doc_markdown, clippy::float_cmp)]

/// `μ₀/4π` (T·m/A).
const MU0_4PI: f64 = 1e-7;

type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// Magnetic field (T) of a point dipole `moment` (A·m²) located at `source`,
/// evaluated at `at`: `B = (μ₀/4π)·(3(m·r̂)r̂ − m)/r³`.
#[must_use]
pub fn dipole_field(moment: V3, source: V3, at: V3) -> V3 {
    let r = sub(at, source);
    let rn = norm(r);
    let rhat = scale(r, 1.0 / rn);
    let m_dot_rhat = dot(moment, rhat);
    let term = sub(scale(rhat, 3.0 * m_dot_rhat), moment);
    scale(term, MU0_4PI / (rn * rn * rn))
}

/// A fixed (well-defining) magnet: position (m) and moment vector (A·m²).
#[derive(Debug, Clone, Copy)]
pub struct FixedMagnet {
    /// Position (m).
    pub pos: V3,
    /// Dipole moment (A·m²).
    pub moment: V3,
}

/// The sensing geometry. Tip moves along **x**; the tip magnet sits at axial
/// position `tip_axial_m` (the y coordinate) and lateral position = the swept x.
#[derive(Debug, Clone)]
pub struct SensingRig {
    /// Tip-magnet dipole moment (A·m²).
    pub tip_moment: V3,
    /// Tip rest axial (y) coordinate (m).
    pub tip_axial_m: f64,
    /// The two fixed well magnets.
    pub fixed: Vec<FixedMagnet>,
    /// Hall sensor position (m).
    pub sensor_pos: V3,
    /// Hall sensor sensitive axis (unit vector).
    pub sensor_axis: V3,
}

impl SensingRig {
    /// Field along the sensor axis (T) from the tip magnet alone, with the tip at
    /// lateral position `x`.
    #[must_use]
    pub fn tip_field_t(&self, x: f64) -> f64 {
        let tip_pos = [x, self.tip_axial_m, 0.0];
        dot(
            dipole_field(self.tip_moment, tip_pos, self.sensor_pos),
            self.sensor_axis,
        )
    }

    /// Constant field along the sensor axis (T) from the fixed well magnets — the
    /// x-independent DC offset.
    #[must_use]
    pub fn fixed_offset_t(&self) -> f64 {
        let mut b = [0.0, 0.0, 0.0];
        for fm in &self.fixed {
            b = add(b, dipole_field(fm.moment, fm.pos, self.sensor_pos));
        }
        dot(b, self.sensor_axis)
    }

    /// Total Hall field along the sensor axis (T) at tip position `x`.
    #[must_use]
    pub fn hall_field_t(&self, x: f64) -> f64 {
        self.tip_field_t(x) + self.fixed_offset_t()
    }

    /// Sensitivity `dB/dx` (T/m = mT/mm) at tip position `x` (central difference).
    #[must_use]
    pub fn sensitivity_t_per_m(&self, x: f64, h: f64) -> f64 {
        (self.tip_field_t(x + h) - self.tip_field_t(x - h)) / (2.0 * h)
    }

    /// Is the readout monotonic across `x ∈ [−range, range]` (so position is
    /// recoverable)? Sampled at `n` points.
    #[must_use]
    pub fn is_monotonic(&self, range: f64, n: usize) -> bool {
        let mut prev = self.tip_field_t(-range);
        let mut sign = 0.0_f64;
        for i in 1..=n {
            let x = -range + 2.0 * range * i as f64 / n as f64;
            let cur = self.tip_field_t(x);
            let d = cur - prev;
            if d != 0.0 {
                let s = d.signum();
                if sign == 0.0 {
                    sign = s;
                } else if s != sign {
                    return false;
                }
            }
            prev = cur;
        }
        true
    }
}

/// Position resolution: metres of tip motion per ADC count.
///
/// For a ratiometric Hall sensor of sensitivity `hall_v_per_t` (V/T) read by an
/// ADC with `adc_lsb_v` volts/count, given a field sensitivity `sens_t_per_m`
/// (T/m): `Δx = adc_lsb / (sens·hall)`.
#[must_use]
pub fn position_resolution_m(sens_t_per_m: f64, hall_v_per_t: f64, adc_lsb_v: f64) -> f64 {
    adc_lsb_v / (sens_t_per_m.abs() * hall_v_per_t)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp)]

    use super::*;
    use crate::magnetoelastic::Magnet;

    /// The recommended geometry — tip magnet along the motion axis, sensor offset
    /// perpendicular by d ≳ 2·x_max — gives a readout monotonic through the wells,
    /// sensitive, and micron-resolving, with the fixed magnets only a DC offset.
    /// Also demonstrates the d/2 fold that fixes the d ≳ 2·x_max rule.
    #[test]
    fn perpendicular_offset_readout_is_monotonic_sensitive_and_fine() {
        let tip_m = Magnet {
            remanence_t: 1.32,
            radius_m: 2.5e-3,
            thickness_m: 2e-3,
            density_kgm3: 7500.0,
        }
        .moment_am2();
        let fixed_m = Magnet {
            remanence_t: 1.32,
            radius_m: 4e-3,
            thickness_m: 3e-3,
            density_kgm3: 7500.0,
        }
        .moment_am2();
        let x_0 = 2e-3;
        let d = 5e-3; // sensor perpendicular offset ≳ 2·x₀

        let rig = SensingRig {
            tip_moment: [tip_m, 0.0, 0.0], // along motion axis x
            tip_axial_m: 0.0,
            fixed: vec![
                FixedMagnet {
                    pos: [5e-3, 9e-3, 0.0],
                    moment: [0.0, fixed_m, 0.0],
                },
                FixedMagnet {
                    pos: [-5e-3, 9e-3, 0.0],
                    moment: [0.0, fixed_m, 0.0],
                },
            ],
            sensor_pos: [0.0, 0.0, d],
            sensor_axis: [0.0, 0.0, 1.0], // read B_z
        };

        // Monotonic through the wells ±x₀ (which-well counting + in-well variance).
        assert!(
            rig.is_monotonic(x_0, 400),
            "readout must be monotonic over ±x₀"
        );

        // ...but it folds at the inflection x = d/2 = 1.25·x₀ here, so it is NOT
        // monotonic over the larger transit excursion. This makes the design rule
        // d ≳ 2·x_max explicit: with d=5 mm a ±1.5·x₀ swing already exceeds d/2.
        // (Fine for hysteretic which-well counting; continuous position is only
        // valid for |x| < d/2 — push d larger, trading sensitivity ∝ 1/d⁴.)
        assert!(
            !rig.is_monotonic(1.5 * x_0, 400),
            "with d=5mm/x₀=2mm the readout SHOULD fold past d/2 — demonstrating d≳2·x_max"
        );

        // Odd response: B(+x) = −B(−x) (so + and − wells are distinguishable).
        assert!((rig.tip_field_t(x_0) + rig.tip_field_t(-x_0)).abs() < 1e-12);

        // Sensitivity of order 10 mT/mm.
        let sens = rig.sensitivity_t_per_m(0.0, 1e-5); // T/m
        assert!(
            sens.abs() > 5.0,
            "sensitivity {:.2} mT/mm too low",
            sens.abs()
        );

        // The well magnets are only a DC offset; total field sets the DRV5055
        // range variant. Report it's within the widest variant (±169 mT).
        let total = rig.hall_field_t(0.0).abs();
        assert!(
            total < 0.169,
            "total DC field {:.1} mT exceeds DRV5055 range",
            total * 1e3
        );

        // Position resolution with a DRV5055A4 (12.5 mV/mT, the wide-range variant
        // needed for the DC field) and the Teensy 12-bit ADC (3.3V/4096).
        let hall_v_per_t = 12.5e-3 / 1e-3; // 12.5 mV/mT → V/T
        let adc_lsb = 3.3 / 4096.0;
        let res = position_resolution_m(sens, hall_v_per_t, adc_lsb);
        // Far finer than the in-well RMS jitter (sub-mm) we must measure.
        assert!(
            res < 20e-6,
            "position resolution {:.1} µm too coarse",
            res * 1e6
        );
    }
}
