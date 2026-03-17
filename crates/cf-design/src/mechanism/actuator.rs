//! Actuator definition types.
//!
//! An [`ActuatorDef`] drives a tendon with a specific actuation model defined
//! by [`ActuatorKind`]. These map to MJCF `<actuator>` elements during MJCF
//! generation (Session 10).
//!
//! Downstream mapping to sim-types:
//! - `Motor` → `<motor>` / `<position>` / `<velocity>` (depending on ctrl mode)
//! - `Muscle` → `<muscle>` (Hill-type muscle model)

/// Kind of actuator driving a tendon.
///
/// Uses biomechanics-informed names. Maps to `MuJoCo` actuator types for
/// simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ActuatorKind {
    /// Position/velocity servo motor.
    Motor,
    /// Hill-type muscle model (force-length-velocity curves).
    Muscle,
}

/// Actuator driving a tendon in a mechanism.
///
/// The actuator references a [`super::TendonDef`] by name and specifies
/// the force range and optional control range for simulation.
///
/// # Example
///
/// ```
/// use cf_design::{ActuatorDef, ActuatorKind};
///
/// let motor = ActuatorDef::new("wrist_motor", "flexor_1", ActuatorKind::Motor, (-50.0, 50.0))
///     .with_ctrl_range(-1.0, 1.0);
/// ```
#[derive(Debug, Clone)]
pub struct ActuatorDef {
    name: String,
    tendon: String,
    kind: ActuatorKind,
    force_range: (f64, f64),
    ctrl_range: Option<(f64, f64)>,
}

impl ActuatorDef {
    /// Create an actuator definition.
    ///
    /// - `tendon` — name of the [`super::TendonDef`] this actuator drives.
    /// - `force_range` — `(min, max)` force/torque the actuator can exert.
    ///
    /// # Panics
    ///
    /// Panics if any name is empty, or `force_range` has non-finite bounds
    /// or `min >= max`.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        tendon: impl Into<String>,
        kind: ActuatorKind,
        force_range: (f64, f64),
    ) -> Self {
        let name = name.into();
        let tendon = tendon.into();

        assert!(!name.is_empty(), "actuator name must not be empty");
        assert!(!tendon.is_empty(), "actuator tendon must not be empty");
        assert!(
            force_range.0.is_finite() && force_range.1.is_finite(),
            "actuator force_range bounds must be finite"
        );
        assert!(
            force_range.0 < force_range.1,
            "actuator force_range min must be less than max, got [{}, {}]",
            force_range.0,
            force_range.1
        );

        Self {
            name,
            tendon,
            kind,
            force_range,
            ctrl_range: None,
        }
    }

    /// Set the control input range for simulation.
    ///
    /// # Panics
    ///
    /// Panics if `min >= max` or either bound is non-finite.
    #[must_use]
    pub fn with_ctrl_range(mut self, min: f64, max: f64) -> Self {
        assert!(
            min.is_finite() && max.is_finite(),
            "actuator ctrl_range bounds must be finite"
        );
        assert!(
            min < max,
            "actuator ctrl_range min must be less than max, got [{min}, {max}]"
        );
        self.ctrl_range = Some((min, max));
        self
    }

    /// Actuator name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Name of the tendon this actuator drives.
    #[must_use]
    pub fn tendon(&self) -> &str {
        &self.tendon
    }

    /// Actuator kind (motor or muscle).
    #[must_use]
    pub const fn kind(&self) -> ActuatorKind {
        self.kind
    }

    /// Force/torque range `(min, max)`.
    #[must_use]
    pub const fn force_range(&self) -> (f64, f64) {
        self.force_range
    }

    /// Control input range, if set.
    #[must_use]
    pub const fn ctrl_range(&self) -> Option<(f64, f64)> {
        self.ctrl_range
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_actuator() -> ActuatorDef {
        ActuatorDef::new("motor_1", "flexor_1", ActuatorKind::Motor, (-50.0, 50.0))
    }

    // ── ActuatorKind ─────────────────────────────────────────────────

    #[test]
    fn actuator_kind_variants() {
        let kinds = [ActuatorKind::Motor, ActuatorKind::Muscle];
        for (i, a) in kinds.iter().enumerate() {
            for (j, b) in kinds.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
    }

    #[test]
    fn actuator_kind_is_copy() {
        let k = ActuatorKind::Motor;
        let k2 = k; // Copy
        assert_eq!(k, k2);
    }

    // ── ActuatorDef construction ─────────────────────────────────────

    #[test]
    fn actuator_new_valid() {
        let a = make_actuator();
        assert_eq!(a.name(), "motor_1");
        assert_eq!(a.tendon(), "flexor_1");
        assert_eq!(a.kind(), ActuatorKind::Motor);
        assert_eq!(a.force_range(), (-50.0, 50.0));
        assert!(a.ctrl_range().is_none());
    }

    #[test]
    fn actuator_muscle_variant() {
        let a = ActuatorDef::new("bicep", "flexor", ActuatorKind::Muscle, (0.0, 200.0));
        assert_eq!(a.kind(), ActuatorKind::Muscle);
    }

    #[test]
    #[should_panic(expected = "actuator name must not be empty")]
    fn actuator_rejects_empty_name() {
        let _a = ActuatorDef::new("", "tendon", ActuatorKind::Motor, (-1.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "actuator tendon must not be empty")]
    fn actuator_rejects_empty_tendon() {
        let _a = ActuatorDef::new("act", "", ActuatorKind::Motor, (-1.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "force_range min must be less than max")]
    fn actuator_rejects_invalid_force_range() {
        let _a = ActuatorDef::new("act", "t", ActuatorKind::Motor, (10.0, -10.0));
    }

    #[test]
    #[should_panic(expected = "force_range min must be less than max")]
    fn actuator_rejects_equal_force_range() {
        let _a = ActuatorDef::new("act", "t", ActuatorKind::Motor, (5.0, 5.0));
    }

    #[test]
    #[should_panic(expected = "force_range bounds must be finite")]
    fn actuator_rejects_nan_force_range() {
        let _a = ActuatorDef::new("act", "t", ActuatorKind::Motor, (f64::NAN, 1.0));
    }

    // ── ctrl_range builder ───────────────────────────────────────────

    #[test]
    fn actuator_with_ctrl_range() {
        let a = make_actuator().with_ctrl_range(-1.0, 1.0);
        assert_eq!(a.ctrl_range(), Some((-1.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "ctrl_range min must be less than max")]
    fn actuator_rejects_invalid_ctrl_range() {
        let _a = make_actuator().with_ctrl_range(1.0, -1.0);
    }

    #[test]
    #[should_panic(expected = "ctrl_range bounds must be finite")]
    fn actuator_rejects_nan_ctrl_range() {
        let _a = make_actuator().with_ctrl_range(f64::NAN, 1.0);
    }
}
