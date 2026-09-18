//! Elastomer bushings — a joint rate you derive, not a number you type.
//!
//! [`JointDef::with_stiffness`](super::JointDef::with_stiffness) takes a
//! number. Nothing said where that number comes from, so a suspension bush
//! was a guess wearing units. A bonded cylindrical bush is a pure-shear
//! problem with a closed form, so it does not have to be.
//!
//! ⚠ **Two things here are estimates and are marked as such**: the Shore-A
//! correlation ([`shear_modulus_from_shore_a`]), and the assumption that the
//! elastomer is bonded to both sleeves and loaded in shear alone. Radial
//! rate is deliberately absent — it is not a shear problem, and a plausible
//! formula for it would be worse than none.

use std::f64::consts::PI;

/// Torque unit of the mechanism's own coordinates, in newton-metres.
///
/// cf-design geometry is millimetres and mass is kilograms, so the model's
/// torque unit is kg·mm²/s² — a **microjoule**, a millionth of a newton-metre
/// (`model_builder.rs` scales gravity to 9810 mm/s² for the same reason).
/// A rate handed to `with_stiffness` in N·m/rad is therefore a million times
/// too soft, and nothing would report it: the machine would simply sag.
use super::units;

/// A bonded cylindrical elastomer bushing.
///
/// Inner sleeve of radius `a`, outer of radius `b`, elastomer between them
/// over a length `L`, bonded to both. Loading it in torsion or along its axis
/// puts the elastomer in **pure shear**, which is why those two rates have
/// exact closed forms and the radial one does not appear here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bushing {
    inner_radius_mm: f64,
    outer_radius_mm: f64,
    length_mm: f64,
    shear_modulus_pa: f64,
}

impl Bushing {
    /// Define a bushing from its geometry and the elastomer's shear modulus.
    ///
    /// # Panics
    ///
    /// Panics unless every dimension is positive and finite and the outer
    /// radius exceeds the inner — a bushing with no elastomer in it is not a
    /// stiff bushing, it is a nonsense one.
    #[must_use]
    pub fn new(
        inner_radius_mm: f64,
        outer_radius_mm: f64,
        length_mm: f64,
        shear_modulus_pa: f64,
    ) -> Self {
        for (label, v) in [
            ("inner radius", inner_radius_mm),
            ("outer radius", outer_radius_mm),
            ("length", length_mm),
            ("shear modulus", shear_modulus_pa),
        ] {
            assert!(
                v > 0.0 && v.is_finite(),
                "bushing {label} must be positive and finite, got {v}"
            );
        }
        assert!(
            outer_radius_mm > inner_radius_mm,
            "bushing outer radius {outer_radius_mm} must exceed inner {inner_radius_mm}; \
             there is no elastomer otherwise"
        );
        Self {
            inner_radius_mm,
            outer_radius_mm,
            length_mm,
            shear_modulus_pa,
        }
    }

    /// The same, taking the elastomer's hardness instead of its modulus.
    ///
    /// # Panics
    ///
    /// Panics on the same geometry conditions as [`Bushing::new`], and if
    /// `shore_a` is outside the range [`shear_modulus_from_shore_a`] accepts.
    #[must_use]
    pub fn from_shore_a(
        inner_radius_mm: f64,
        outer_radius_mm: f64,
        length_mm: f64,
        shore_a: f64,
    ) -> Self {
        Self::new(
            inner_radius_mm,
            outer_radius_mm,
            length_mm,
            shear_modulus_from_shore_a(shore_a),
        )
    }

    /// Rate in twist about the sleeve axis, newton-metres per radian.
    ///
    /// Shear stress at radius `r` under torque `T` is `T / (2πr²L)`, so
    /// integrating `γ/r` from `a` to `b` gives
    /// `θ = T(1/a² − 1/b²) / (4πGL)`. This is the reciprocal of that.
    ///
    /// ★ **Wall thickness dominates, not size.** As the bore approaches the
    /// outer radius the denominator runs to zero and the rate runs away: a
    /// thin sliver of elastomer is very stiff in shear. Softening a bush
    /// means giving it *more* wall, not a bigger one.
    #[must_use]
    pub fn torsional_rate_n_m_per_rad(&self) -> f64 {
        let a = self.inner_radius_mm / 1000.0;
        let b = self.outer_radius_mm / 1000.0;
        let l = self.length_mm / 1000.0;
        4.0 * PI * self.shear_modulus_pa * l / (1.0 / (a * a) - 1.0 / (b * b))
    }

    /// Rate for sliding the inner sleeve along the axis, newtons per metre.
    ///
    /// Also pure shear: `K = 2πGL / ln(b/a)`.
    #[must_use]
    pub fn axial_rate_n_per_m(&self) -> f64 {
        let l = self.length_mm / 1000.0;
        2.0 * PI * self.shear_modulus_pa * l / (self.outer_radius_mm / self.inner_radius_mm).ln()
    }

    /// Torsional rate in the mechanism's own units, for
    /// [`JointDef::with_stiffness`](super::JointDef::with_stiffness).
    ///
    /// ⚠ **Use this, not [`Bushing::torsional_rate_n_m_per_rad`], when
    /// building a joint.** The physical rate is the reportable quantity; this
    /// is the one the model wants, and the two differ by a factor of a
    /// million: the model's torque unit is kg·mm²/s², a microjoule.
    #[must_use]
    pub fn joint_stiffness(&self) -> f64 {
        units::model_stiffness_from_n_m_per_rad(self.torsional_rate_n_m_per_rad())
    }
}

/// Shear modulus of an elastomer from its Shore A hardness, in pascals.
///
/// Gent's correlation for Young's modulus, with `G = E / 3` for a material
/// taken as incompressible (`ν → 0.5`):
///
/// ```text
/// E (MPa) = 0.0981 (56 + 7.66 S) / (0.137_505 (254 − 2.54 S))
/// ```
///
/// ⚠⚠ **This is a correlation, not a measurement, and it stiffens sharply at
/// the top of the scale** — `90A` gives 21 `MPa` and `95A` gives 44 `MPa`, a
/// doubling
/// across five points, because the denominator heads for zero at `100A`.
/// Treat anything above about 90A as an order of magnitude, design for a
/// range, and measure the real material before trusting a single figure.
///
/// # Panics
///
/// Panics unless `shore_a` is finite and in `(0, 100)`; the correlation has a
/// pole at 100 and is meaningless outside its scale.
#[must_use]
pub fn shear_modulus_from_shore_a(shore_a: f64) -> f64 {
    assert!(
        shore_a.is_finite() && shore_a > 0.0 && shore_a < 100.0,
        "Shore A hardness must be in (0, 100), got {shore_a}"
    );
    let e_mpa = 0.0981 * (56.0 + 7.66 * shore_a) / (0.137_505 * (254.0 - 2.54 * shore_a));
    e_mpa * 1e6 / 3.0
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::{Bushing, shear_modulus_from_shore_a};
    use crate::mechanism::{JointDef, JointKind, Material, Mechanism, Part};
    use crate::solid::Solid;

    /// A steel arm hinged at the origin, sagging under its own weight against
    /// a bushing of the given joint stiffness. Returns the sag in radians.
    fn sag_of_loaded_arm(joint_stiffness: f64) -> (f64, f64, f64) {
        let steel = Material::new("mild steel", 7850.0);
        let mut m = Mechanism::builder("bushed arm")
            .part(Part::new(
                "post",
                Solid::cuboid(Vector3::new(10.0, 10.0, 10.0)),
                steel.clone(),
            ))
            // ⚠ The bar is offset so its body origin is the hinge END, not
            // its centre, and `with_joint_origin` keeps `to_model` from
            // bbox-aligning it back. Centred on the hinge it has no lever,
            // gravity makes no torque, and the whole fixture measures
            // nothing — which is how it was first written, and passed.
            .part(
                Part::new(
                    "arm",
                    Solid::cuboid(Vector3::new(100.0, 5.0, 5.0))
                        .translate(Vector3::new(100.0, 0.0, 0.0)),
                    steel,
                )
                .with_joint_origin(Vector3::zeros()),
            )
            .joint(JointDef::new(
                "anchor",
                "world",
                "post",
                JointKind::Fixed,
                Point3::origin(),
                Vector3::z(),
            ))
            .joint(
                JointDef::new(
                    "bush",
                    "post",
                    "arm",
                    JointKind::Revolute,
                    Point3::origin(),
                    Vector3::y(),
                )
                .with_stiffness(joint_stiffness)
                .with_damping(joint_stiffness / 50.0),
            )
            .build()
            .to_model(4.0, 4.0)
            .unwrap();
        // ⛔ This gate measures TORSIONAL SAG, not contact. The arm's solid
        // necessarily passes through its own post at the pivot — measured, 8
        // contacts at 7.5 mm and 2.5 mm depth — which is what a hinge lug
        // inside a bracket looks like, and a real model excludes the pair.
        //
        // ⚠ Those contacts used to be absent for the WRONG reason: the
        // collision filter suppressed any parent-child pair, and `post` is the
        // arm's parent. `post` is welded to the world, so it belongs to weld
        // group 0, and MuJoCo exempts the world group from the parent rule —
        // it collides them. Once the filter matched MuJoCo the arm started
        // resting on its own post and settled at 0.00086 rad against an
        // arithmetic 0.00858, i.e. this gate began measuring contact stiffness
        // instead of the bushing. Disabled explicitly rather than left to a
        // filter bug.
        m.disableflags |= sim_core::DISABLE_CONTACT;

        let arm = m
            .body_name
            .iter()
            .position(|b| b.as_deref() == Some("arm"))
            .unwrap();
        let mass = m.body_mass[arm];

        let mut data = m.make_data();
        data.forward(&m).unwrap();
        // Lever arm: horizontal distance from the hinge to the arm's centre
        // of mass, read off the model rather than assumed from the cuboid.
        let lever = (data.xipos[arm] - data.xpos[arm]).x.abs();
        for _ in 0..4000 {
            data.step(&m).unwrap();
        }
        (data.qpos[data.qpos.len() - 1].abs(), mass, lever)
    }

    /// A bushing's rate means the same to the model as it does on paper.
    ///
    /// ⚠⚠ **This is the gate for the factor of a million.** The mechanism's
    /// torque unit is kg·mm²/s², so a rate handed over in N·m/rad is a
    /// millionth of what was meant — and nothing reports it, the machine just
    /// sags. Both halves are needed: that the converted rate holds the arm
    /// near where the arithmetic says, and that the raw one does not hold it
    /// at all.
    ///
    /// ⚠ **It cannot see the formula.** The prediction is built from
    /// `torsional_rate_n_m_per_rad`, so scaling that function scales both
    /// sides and this still passes — halving it survives here and is caught
    /// by `the_rates_match_hand_computed_values`, which asserts a literal.
    /// The two gates cover different axes on purpose.
    #[test]
    fn the_model_feels_the_rate_the_arithmetic_predicts() {
        let bush = Bushing::from_shore_a(5.0, 25.0, 30.0, 70.0);
        let rate = bush.torsional_rate_n_m_per_rad();

        let (sag, mass, lever) = sag_of_loaded_arm(bush.joint_stiffness());
        // θ = m g r / k, in SI, with the mass and lever the model reports.
        let predicted = mass * 9.81 * (lever / 1000.0) / rate;
        assert!(
            (sag - predicted).abs() < 0.1 * predicted,
            "the arm settled at {sag:.5} rad, arithmetic says {predicted:.5}              ({mass:.4} kg on a {lever:.1} mm lever against {rate:.3} N·m/rad)"
        );

        let (raw_sag, _, _) = sag_of_loaded_arm(rate);
        assert!(
            raw_sag > 20.0 * sag,
            "passing the rate in N·m/rad should collapse the arm, but it              settled at {raw_sag:.5} rad against {sag:.5} — if these are              close, the unit conversion is not doing anything"
        );
    }

    /// The closed form, against numbers computed outside this file.
    ///
    /// ⚠ The expected values are literals worked out independently, not the
    /// formula restated — an oracle that recomputes the expression agrees
    /// with a transcription error in it.
    #[test]
    fn the_rates_match_hand_computed_values() {
        // 10 mm bore, 20 mm outer radius, 40 mm long, G = 10 MPa exactly.
        let b = Bushing::new(10.0, 20.0, 40.0, 10.0e6);

        let torsion = b.torsional_rate_n_m_per_rad();
        assert!(
            (torsion - 670.206_433).abs() < 1e-5,
            "torsional rate {torsion}, expected 670.206433 N·m/rad"
        );

        let axial = b.axial_rate_n_per_m();
        assert!(
            (axial - 3_625_888.113_5).abs() < 1e-2,
            "axial rate {axial}, expected 3625888.1135 N/m"
        );
    }

    /// Shore A 95 lands where the correlation says, and 90 is half of it.
    ///
    /// The second assertion is the point: this doubles across five points of
    /// hardness, which is why the doc says to design for a range.
    #[test]
    fn the_hardness_correlation_matches_and_stiffens_sharply() {
        let g95 = shear_modulus_from_shore_a(95.0);
        assert!(
            (g95 / 1e6 - 14.675).abs() < 1e-3,
            "95A gives {} MPa, expected 14.675",
            g95 / 1e6
        );
        let g90 = shear_modulus_from_shore_a(90.0);
        assert!(
            (g90 / 1e6 - 6.979).abs() < 1e-3,
            "90A gives {} MPa, expected 6.979",
            g90 / 1e6
        );
        assert!(
            g95 / g90 > 2.0,
            "five points of hardness should more than double it, got {}",
            g95 / g90
        );
    }

    /// A thinner wall of elastomer is dramatically stiffer.
    ///
    /// ⚠ This test was first written — and first passed — under the name
    /// `a_bigger_bore_is_dramatically_softer`, which is backwards. The
    /// assertion was right and the claim above it was not, which is the only
    /// reason it surfaced. Same outer radius, so the fat pin is the thin wall.
    #[test]
    fn a_thinner_wall_is_dramatically_stiffer() {
        let thick_wall = Bushing::new(5.0, 25.0, 30.0, 10.0e6);
        let thin_wall = Bushing::new(12.5, 25.0, 30.0, 10.0e6);
        let ratio =
            thin_wall.torsional_rate_n_m_per_rad() / thick_wall.torsional_rate_n_m_per_rad();
        assert!(
            ratio > 4.0,
            "thinning the wall should stiffen it several-fold, got {ratio}×"
        );
    }

    /// Rejects geometry that has no elastomer in it.
    #[test]
    #[should_panic(expected = "must exceed inner")]
    fn a_bushing_with_no_wall_is_rejected() {
        let _b = Bushing::new(10.0, 10.0, 30.0, 10.0e6);
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn a_bushing_with_no_length_is_rejected() {
        let _b = Bushing::new(10.0, 20.0, 0.0, 10.0e6);
    }

    #[test]
    #[should_panic(expected = "Shore A hardness")]
    fn the_correlations_pole_is_out_of_bounds() {
        let _g = shear_modulus_from_shore_a(100.0);
    }
}
