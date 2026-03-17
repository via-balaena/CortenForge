//! Joint definition types.
//!
//! A [`JointDef`] connects two [`super::Part`]s with a specific degree of
//! freedom defined by [`JointKind`]. These map 1:1 to MJCF `<joint>` elements
//! during MJCF generation (Session 10).
//!
//! Downstream mapping to sim-types:
//! - `Revolute` → `MjJointType::Hinge`
//! - `Prismatic` → `MjJointType::Slide`
//! - `Ball` → `MjJointType::Ball`
//! - `Free` → `MjJointType::Free`

use nalgebra::{Point3, Vector3};

/// Kind of joint connecting two parts.
///
/// Uses robotics-standard names. Maps to `MuJoCo` joint types for simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JointKind {
    /// 1 DOF rotation about the joint axis.
    Revolute,
    /// 1 DOF translation along the joint axis.
    Prismatic,
    /// 3 DOF free rotation (spherical joint).
    Ball,
    /// 6 DOF floating body (unconstrained).
    Free,
}

/// Joint connecting two parts in a mechanism.
///
/// The joint is defined at an `anchor` point with a primary `axis` direction.
/// An optional `range` constrains the joint's travel.
///
/// # Example
///
/// ```
/// use cf_design::{JointDef, JointKind};
/// use nalgebra::{Point3, Vector3};
///
/// let hinge = JointDef::new(
///     "elbow",
///     "upper_arm",
///     "forearm",
///     JointKind::Revolute,
///     Point3::new(0.0, 0.0, 5.0),
///     Vector3::x(),
/// ).with_range(-2.0, 0.5);
/// ```
#[derive(Debug, Clone)]
pub struct JointDef {
    name: String,
    parent: String,
    child: String,
    kind: JointKind,
    anchor: Point3<f64>,
    axis: Vector3<f64>,
    range: Option<(f64, f64)>,
    stiffness: Option<f64>,
    damping: Option<f64>,
}

impl JointDef {
    /// Create a joint definition.
    ///
    /// The `axis` is normalized internally. For [`JointKind::Ball`] and
    /// [`JointKind::Free`], the axis serves as a reference direction.
    ///
    /// # Panics
    ///
    /// Panics if any name is empty, `anchor` has non-finite coordinates,
    /// or `axis` is zero or non-finite.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        parent: impl Into<String>,
        child: impl Into<String>,
        kind: JointKind,
        anchor: Point3<f64>,
        axis: Vector3<f64>,
    ) -> Self {
        let name = name.into();
        let parent = parent.into();
        let child = child.into();

        assert!(!name.is_empty(), "joint name must not be empty");
        assert!(!parent.is_empty(), "joint parent must not be empty");
        assert!(!child.is_empty(), "joint child must not be empty");
        assert!(
            anchor.iter().all(|c| c.is_finite()),
            "joint anchor must have finite coordinates"
        );
        assert!(
            axis.iter().all(|c| c.is_finite()) && axis.norm() > 1e-10,
            "joint axis must be non-zero and finite"
        );

        Self {
            name,
            parent,
            child,
            kind,
            anchor,
            axis: axis.normalize(),
            range: None,
            stiffness: None,
            damping: None,
        }
    }

    /// Set joint spring stiffness (N·mm/rad for revolute/ball, N/mm for prismatic).
    ///
    /// Used to model compliant/flexure joints derived from `FlexZone` splitting.
    ///
    /// # Panics
    ///
    /// Panics if `k` is negative or non-finite.
    #[must_use]
    pub fn with_stiffness(mut self, k: f64) -> Self {
        assert!(
            k >= 0.0 && k.is_finite(),
            "joint stiffness must be non-negative and finite, got {k}"
        );
        self.stiffness = Some(k);
        self
    }

    /// Set joint damping coefficient.
    ///
    /// # Panics
    ///
    /// Panics if `c` is negative or non-finite.
    #[must_use]
    pub fn with_damping(mut self, c: f64) -> Self {
        assert!(
            c >= 0.0 && c.is_finite(),
            "joint damping must be non-negative and finite, got {c}"
        );
        self.damping = Some(c);
        self
    }

    /// Constrain the joint's range of motion.
    ///
    /// For [`JointKind::Revolute`], range is in radians.
    /// For [`JointKind::Prismatic`], range is in meters.
    ///
    /// # Panics
    ///
    /// Panics if `min >= max` or either bound is non-finite.
    #[must_use]
    pub fn with_range(mut self, min: f64, max: f64) -> Self {
        assert!(
            min.is_finite() && max.is_finite(),
            "joint range bounds must be finite"
        );
        assert!(
            min < max,
            "joint range min must be less than max, got [{min}, {max}]"
        );
        self.range = Some((min, max));
        self
    }

    /// Joint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Parent part name.
    #[must_use]
    pub fn parent(&self) -> &str {
        &self.parent
    }

    /// Child part name.
    #[must_use]
    pub fn child(&self) -> &str {
        &self.child
    }

    /// Joint kind (degree of freedom type).
    #[must_use]
    pub const fn kind(&self) -> JointKind {
        self.kind
    }

    /// Anchor point (joint position in world frame).
    #[must_use]
    pub const fn anchor(&self) -> &Point3<f64> {
        &self.anchor
    }

    /// Primary axis (unit vector, normalized at construction).
    #[must_use]
    pub const fn axis(&self) -> &Vector3<f64> {
        &self.axis
    }

    /// Range of motion, if constrained.
    #[must_use]
    pub const fn range(&self) -> Option<(f64, f64)> {
        self.range
    }

    /// Spring stiffness, if set.
    #[must_use]
    pub const fn stiffness(&self) -> Option<f64> {
        self.stiffness
    }

    /// Damping coefficient, if set.
    #[must_use]
    pub const fn damping(&self) -> Option<f64> {
        self.damping
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_joint() -> JointDef {
        JointDef::new(
            "hinge",
            "parent",
            "child",
            JointKind::Revolute,
            Point3::origin(),
            Vector3::z(),
        )
    }

    #[test]
    fn joint_def_valid() {
        let j = make_joint();
        assert_eq!(j.name(), "hinge");
        assert_eq!(j.parent(), "parent");
        assert_eq!(j.child(), "child");
        assert_eq!(j.kind(), JointKind::Revolute);
        assert!(j.range().is_none());
    }

    #[test]
    fn joint_def_with_range() {
        let j = make_joint().with_range(-1.5, 1.5);
        assert_eq!(j.range(), Some((-1.5, 1.5)));
    }

    #[test]
    #[should_panic(expected = "min must be less than max")]
    fn joint_def_range_min_ge_max() {
        let _j = make_joint().with_range(1.0, -1.0);
    }

    #[test]
    #[should_panic(expected = "min must be less than max")]
    fn joint_def_range_min_eq_max() {
        let _j = make_joint().with_range(0.0, 0.0);
    }

    #[test]
    #[should_panic(expected = "range bounds must be finite")]
    fn joint_def_range_rejects_nan() {
        let _j = make_joint().with_range(f64::NAN, 1.0);
    }

    #[test]
    #[should_panic(expected = "axis must be non-zero")]
    fn joint_def_rejects_zero_axis() {
        let _j = JointDef::new(
            "bad",
            "a",
            "b",
            JointKind::Revolute,
            Point3::origin(),
            Vector3::zeros(),
        );
    }

    #[test]
    fn joint_def_normalizes_axis() {
        let j = JointDef::new(
            "j",
            "a",
            "b",
            JointKind::Prismatic,
            Point3::origin(),
            Vector3::new(0.0, 0.0, 10.0),
        );
        let norm = j.axis().norm();
        assert!(
            (norm - 1.0).abs() < 1e-10,
            "axis should be unit length, got {norm}"
        );
    }

    #[test]
    #[should_panic(expected = "joint name must not be empty")]
    fn joint_def_rejects_empty_name() {
        let _j = JointDef::new(
            "",
            "a",
            "b",
            JointKind::Revolute,
            Point3::origin(),
            Vector3::z(),
        );
    }

    #[test]
    #[should_panic(expected = "joint parent must not be empty")]
    fn joint_def_rejects_empty_parent() {
        let _j = JointDef::new(
            "j",
            "",
            "b",
            JointKind::Revolute,
            Point3::origin(),
            Vector3::z(),
        );
    }

    #[test]
    #[should_panic(expected = "joint child must not be empty")]
    fn joint_def_rejects_empty_child() {
        let _j = JointDef::new(
            "j",
            "a",
            "",
            JointKind::Revolute,
            Point3::origin(),
            Vector3::z(),
        );
    }

    #[test]
    fn joint_kind_all_variants() {
        let kinds = [
            JointKind::Revolute,
            JointKind::Prismatic,
            JointKind::Ball,
            JointKind::Free,
        ];
        // Verify all 4 variants are distinct.
        for (i, a) in kinds.iter().enumerate() {
            for (j, b) in kinds.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
    }

    #[test]
    fn joint_def_stiffness_damping() {
        let j = make_joint().with_stiffness(1000.0).with_damping(10.0);
        assert_eq!(j.stiffness(), Some(1000.0));
        assert_eq!(j.damping(), Some(10.0));
    }

    #[test]
    fn joint_def_stiffness_damping_default_none() {
        let j = make_joint();
        assert!(j.stiffness().is_none());
        assert!(j.damping().is_none());
    }

    #[test]
    fn joint_def_zero_stiffness_allowed() {
        let j = make_joint().with_stiffness(0.0);
        assert_eq!(j.stiffness(), Some(0.0));
    }

    #[test]
    #[should_panic(expected = "non-negative and finite")]
    fn joint_def_rejects_negative_stiffness() {
        let _j = make_joint().with_stiffness(-1.0);
    }

    #[test]
    #[should_panic(expected = "non-negative and finite")]
    fn joint_def_rejects_negative_damping() {
        let _j = make_joint().with_damping(-1.0);
    }

    #[test]
    #[should_panic(expected = "non-negative and finite")]
    fn joint_def_rejects_nan_stiffness() {
        let _j = make_joint().with_stiffness(f64::NAN);
    }

    #[test]
    fn joint_kind_is_copy() {
        let k = JointKind::Revolute;
        let k2 = k; // Copy
        assert_eq!(k, k2);
    }

    #[test]
    fn joint_def_ball_and_free() {
        // Ball and Free joints are valid with any non-zero axis.
        let _ball = JointDef::new(
            "ball",
            "a",
            "b",
            JointKind::Ball,
            Point3::new(1.0, 2.0, 3.0),
            Vector3::y(),
        );
        let _free = JointDef::new(
            "free",
            "a",
            "b",
            JointKind::Free,
            Point3::origin(),
            Vector3::x(),
        );
    }
}
