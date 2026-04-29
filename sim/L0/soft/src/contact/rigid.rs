//! Hand-rolled kinematic rigid primitives — the rigid side of one-way
//! soft↔rigid penalty contact.
//!
//! Phase 5 ships [`RigidPlane`] only; `RigidSphere` is deferred to Phase
//! H per `phase_5_penalty_contact_scope.md` Decision B (no V-* test
//! exercises it). Primitives are kinematic — they don't move during a
//! step (Decision C: one-way coupling). sim-mjcf rigid-body integration
//! is its own future phase between Phase 5 and Phase H.

use crate::Vec3;

/// Closed analytic surface that a soft vertex can contact.
///
/// `signed_distance` follows the SDF convention used elsewhere in
/// `sim-soft` (see [`crate::sdf_bridge::SphereSdf`]): positive outside
/// the rigid solid (in the direction of [`RigidPrimitive::outward_normal`]),
/// zero on the surface, negative inside.
///
/// `pub(crate)` because external users construct concrete primitives
/// (e.g. [`RigidPlane`]) and pass them to the Phase 5 penalty contact
/// model; the trait is the internal `dyn`-erasure surface for primitive
/// heterogeneity inside a single contact model.
pub(crate) trait RigidPrimitive: Send + Sync {
    fn signed_distance(&self, p: Vec3) -> f64;
    fn outward_normal(&self, p: Vec3) -> Vec3;
}

/// Infinite half-space `{ p : p · normal ≥ offset }` with a kinematic
/// pose.
///
/// Constructor takes any nonzero `normal`, normalizes it, and stores the
/// unit form so [`RigidPlane::signed_distance`] is `p · n - offset`.
/// Constructor panics on a zero or non-finite `normal` — a degenerate
/// plane has no well-defined outward direction and would silently break
/// sign conventions downstream.
#[derive(Clone, Copy, Debug)]
pub struct RigidPlane {
    normal: Vec3,
    offset: f64,
}

impl RigidPlane {
    /// Construct a plane from a (possibly non-unit) `normal` and `offset`.
    ///
    /// # Panics
    /// If `normal.norm_squared()` is not strictly positive and finite
    /// (`NaN`, infinity, or exactly zero).
    #[must_use]
    pub fn new(normal: Vec3, offset: f64) -> Self {
        let n_squared = normal.norm_squared();
        assert!(
            n_squared.is_finite() && n_squared > 0.0,
            "RigidPlane::new: normal must have strictly positive finite length, got {normal:?}"
        );
        let normal = normal / n_squared.sqrt();
        Self { normal, offset }
    }

    /// Unit outward normal (constructor-normalized).
    #[must_use]
    pub const fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Plane offset along the unit normal: surface satisfies
    /// `p · normal == offset`.
    #[must_use]
    pub const fn offset(&self) -> f64 {
        self.offset
    }

    /// Signed distance `p · normal - offset` — positive above the plane
    /// (in the direction of the unit normal), negative below.
    #[must_use]
    pub fn signed_distance(&self, p: Vec3) -> f64 {
        p.dot(&self.normal) - self.offset
    }

    /// Constant outward normal — independent of `_p` because the plane
    /// is flat. The argument is part of the [`RigidPrimitive`] trait
    /// contract for primitives whose normal varies with surface position.
    #[must_use]
    pub const fn outward_normal(&self, _p: Vec3) -> Vec3 {
        self.normal
    }
}

impl RigidPrimitive for RigidPlane {
    fn signed_distance(&self, p: Vec3) -> f64 {
        Self::signed_distance(self, p)
    }

    fn outward_normal(&self, p: Vec3) -> Vec3 {
        Self::outward_normal(self, p)
    }
}
