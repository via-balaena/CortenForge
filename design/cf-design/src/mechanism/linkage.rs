//! Loop-closing constraints.
//!
//! A [`super::Mechanism`] is a kinematic **tree**: every part has one parent,
//! and a joint gives it freedom relative to that parent. Real machines are not
//! always trees. A steering tie rod ties both arms together; a double-wishbone
//! upright is held by two A-arms at once; a shock runs between a frame and a
//! swingarm that are already joined through the pivot. Each of those is a
//! **loop**, and a tree cannot say it.
//!
//! A [`LinkageDef`] closes one. It is not a joint — it adds no parent, and the
//! tree is unchanged — it is a constraint applied on top, which is how physics
//! engines spell a loop.
//!
//! Downstream this becomes a MuJoCo equality constraint: `Ball` is
//! `EqualityType::Connect`, three rows holding two points together.

use nalgebra::Point3;

/// What a linkage holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LinkageKind {
    /// Two points coincide, and the parts may still rotate about them freely
    /// — a rod end. Removes 3 degrees of freedom.
    Ball,
}

impl LinkageKind {
    /// Degrees of freedom this linkage removes.
    #[must_use]
    pub const fn constrained_dof(self) -> usize {
        match self {
            Self::Ball => 3,
        }
    }
}

/// A constraint tying two parts together outside the joint tree.
///
/// The anchor is a point on `a`, in **`a`'s own frame**. Where it falls on `b`
/// is not given: it is whatever point of `b` coincides with it at the
/// reference configuration, computed when the physics model is built. That
/// mirrors MJCF's `<connect>`, and it means a linkage cannot be declared
/// inconsistently — there is only one point to get right.
///
/// # Example
///
/// ```
/// use cf_design::{LinkageDef, LinkageKind};
/// use nalgebra::Point3;
///
/// // A tie rod end, 120 mm aft of the left steering arm's own origin.
/// let rod = LinkageDef::new(
///     "tie_rod_left",
///     "steer_arm_l",
///     "tie_rod",
///     LinkageKind::Ball,
///     Point3::new(120.0, 0.0, 0.0),
/// );
/// assert_eq!(rod.kind().constrained_dof(), 3);
/// ```
#[derive(Debug, Clone)]
pub struct LinkageDef {
    name: String,
    a: String,
    b: String,
    kind: LinkageKind,
    anchor: Point3<f64>,
}

impl LinkageDef {
    /// Define a linkage between two parts.
    ///
    /// # Panics
    ///
    /// Panics if any name is empty, if `a` and `b` are the same part — a part
    /// cannot be tied to itself — or if `anchor` has non-finite coordinates.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        a: impl Into<String>,
        b: impl Into<String>,
        kind: LinkageKind,
        anchor: Point3<f64>,
    ) -> Self {
        let (name, a, b) = (name.into(), a.into(), b.into());
        assert!(!name.is_empty(), "linkage name must not be empty");
        assert!(
            !a.is_empty() && !b.is_empty(),
            "linkage part names must not be empty"
        );
        assert!(
            a != b,
            "linkage \"{name}\" ties \"{a}\" to itself, which constrains nothing"
        );
        assert!(
            anchor.iter().all(|c| c.is_finite()),
            "linkage anchor must have finite coordinates"
        );
        Self {
            name,
            a,
            b,
            kind,
            anchor,
        }
    }

    /// Linkage name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The part the anchor is expressed in.
    #[must_use]
    pub fn a(&self) -> &str {
        &self.a
    }

    /// The part held to it.
    #[must_use]
    pub fn b(&self) -> &str {
        &self.b
    }

    /// What the linkage holds.
    #[must_use]
    pub const fn kind(&self) -> LinkageKind {
        self.kind
    }

    /// The held point, in `a`'s frame.
    #[must_use]
    pub const fn anchor(&self) -> &Point3<f64> {
        &self.anchor
    }
}
