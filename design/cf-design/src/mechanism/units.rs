//! The model's unit system, and the bridges a physical quantity crosses to
//! reach it.
//!
//! A [`Mechanism`](super::Mechanism) is built in **mm-kg-s**: lengths in
//! millimetres, masses in kilograms, time in seconds. That is a perfectly
//! consistent system, and it is not SI — which is the whole difficulty, because
//! quantities derived from material properties arrive in SI or in the
//! engineer's mixed units and look entirely at home when they are a thousand
//! or a million times wrong.
//!
//! ## ⛔⛔ The torque unit is the trap
//!
//! Torque is `mass × length² / time²`, so the model's is **kg·mm²/s² — a
//! MICROJOULE**. Against the two units a rotational rate usually arrives in:
//!
//! ```text
//! 1 N·m   = 1 000 000 model torque units
//! 1 N·mm  =     1 000 model torque units
//! ```
//!
//! Neither ratio is 1, neither is obviously wrong at a glance, and a stiffness
//! that is a thousandfold soft still simulates — it just simulates a different
//! machine. This has been got wrong twice in this repo, in both directions,
//! which is why the conversions live here under names that say what they take
//! rather than as bare multiplications at the call site.
//!
//! ## What crosses which bridge
//!
//! - [`Bushing::joint_stiffness`](super::Bushing::joint_stiffness) derives a
//!   rate in **N·m/rad** from the elastomer's shear modulus.
//! - [`split_part`](super::split_part) derives one in **N·mm/rad** from
//!   `E·I/L`, because `E` is in N/mm² once converted from pascals and the
//!   section is in mm⁴.
//!
//! Both end at [`JointDef::with_stiffness`](super::JointDef::with_stiffness),
//! which takes **model units** and cannot tell what it was handed.

/// One model torque unit, `kg·mm²/s²`, expressed in newton-metres.
///
/// A microjoule. This is the only place the ratio is written down; everything
/// else here is derived from it.
pub const N_M_PER_MODEL_TORQUE_UNIT: f64 = 1e-6;

/// Newton-metres in one newton-millimetre.
const N_M_PER_N_MM: f64 = 1e-3;

/// A rotational stiffness in **N·m/rad**, in the model's torque units.
///
/// ```
/// use cf_design::mechanism::units::model_stiffness_from_n_m_per_rad;
/// // A newton-metre per radian is a million microjoules per radian.
/// assert!((model_stiffness_from_n_m_per_rad(1.0) - 1.0e6).abs() < 1e-3);
/// ```
#[must_use]
pub fn model_stiffness_from_n_m_per_rad(k_n_m_per_rad: f64) -> f64 {
    k_n_m_per_rad / N_M_PER_MODEL_TORQUE_UNIT
}

/// A rotational stiffness in **N·mm/rad**, in the model's torque units.
///
/// ⚠ Derived through [`model_stiffness_from_n_m_per_rad`] rather than written
/// as its own factor, so the two cannot drift apart.
///
/// ```
/// use cf_design::mechanism::units::model_stiffness_from_n_mm_per_rad;
/// // A newton-millimetre per radian is a thousand microjoules per radian.
/// assert!((model_stiffness_from_n_mm_per_rad(1.0) - 1.0e3).abs() < 1e-6);
/// ```
#[must_use]
pub fn model_stiffness_from_n_mm_per_rad(k_n_mm_per_rad: f64) -> f64 {
    model_stiffness_from_n_m_per_rad(k_n_mm_per_rad * N_M_PER_N_MM)
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// The two bridges agree where they meet: a thousand newton-millimetres
    /// per radian and one newton-metre per radian are the same stiffness.
    ///
    /// ⛔ This is the assertion that would have caught the defect. Each
    /// conversion on its own looks right; what was wrong was that one of them
    /// was not being applied at all.
    #[test]
    fn a_thousand_newton_millimetres_is_one_newton_metre() {
        let by_n_m = model_stiffness_from_n_m_per_rad(1.0);
        let by_n_mm = model_stiffness_from_n_mm_per_rad(1000.0);
        assert!(
            (by_n_m - by_n_mm).abs() < 1e-6,
            "{by_n_m} against {by_n_mm}"
        );
    }

    /// ★ Anchored to an independently-stated figure, not to the constant the
    /// functions share — otherwise both could be scaled together and agree.
    #[test]
    fn the_ratios_are_a_million_and_a_thousand() {
        assert!((model_stiffness_from_n_m_per_rad(1.0) - 1.0e6).abs() < 1e-3);
        assert!((model_stiffness_from_n_mm_per_rad(1.0) - 1.0e3).abs() < 1e-6);
    }

    /// Zero and negative rates pass through unchanged in sign — a free joint
    /// carries no spring, and a caller that computed a negative rate should
    /// see it rather than have it swallowed.
    #[test]
    fn the_bridges_do_not_clamp() {
        assert!((model_stiffness_from_n_m_per_rad(0.0)).abs() < f64::EPSILON);
        assert!(model_stiffness_from_n_mm_per_rad(-1.0) < 0.0);
    }
}
