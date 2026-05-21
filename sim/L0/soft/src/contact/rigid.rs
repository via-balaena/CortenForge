//! Hand-rolled kinematic rigid plane primitive — the rigid side of
//! one-way soft↔rigid penalty contact.
//!
//! Penalty contact is routed through the crate-public [`Sdf`] trait,
//! so any `impl Sdf` (`SphereSdf`, a scan-derived
//! `mesh_sdf::Signed<TriMeshDistance, _>`, cf-design `Solid`, ...) is a valid
//! rigid primitive without per-shape boilerplate; this file holds only
//! `RigidPlane` and its [`Sdf`] impl. Primitives stay kinematic — they
//! don't move during a step
//! (one-way coupling: rigid kinematic, soft side feels the force).
//! sim-mjcf rigid-body integration is its own future phase before
//! Phase H IPC.

use nalgebra::Point3;

use crate::{Vec3, sdf_bridge::Sdf};

/// Infinite half-space `{ p : p · normal ≥ offset }` with a kinematic
/// pose.
///
/// Constructor takes any nonzero `normal`, normalizes it, and stores the
/// unit form so [`RigidPlane::signed_distance`] is `p · n - offset`.
/// Constructor panics on a zero or non-finite `normal` — a degenerate
/// plane has no well-defined outward direction and would silently break
/// sign conventions downstream.
///
/// `RigidPlane` impls [`Sdf`] so it can be passed directly to
/// [`PenaltyRigidContact::new`](super::PenaltyRigidContact::new) and
/// composed with other `Sdf` primitives in a mixed-primitive contact
/// list. `eval` is the signed distance and `grad` is the (constant)
/// outward unit normal.
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
    /// (in the direction of the unit normal), negative below. Inherent
    /// alias for [`Sdf::eval`]; the inherent name reads better at
    /// plane-specific call sites where the SDF abstraction is incidental.
    #[must_use]
    pub fn signed_distance(&self, p: Vec3) -> f64 {
        p.dot(&self.normal) - self.offset
    }

    /// Constant outward normal — independent of `_p` because the plane
    /// is flat. Inherent alias for [`Sdf::grad`]; kept for symmetry with
    /// [`RigidPlane::signed_distance`].
    #[must_use]
    pub const fn outward_normal(&self, _p: Vec3) -> Vec3 {
        self.normal
    }
}

impl Sdf for RigidPlane {
    fn eval(&self, p: Point3<f64>) -> f64 {
        Self::signed_distance(self, p.coords)
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        Self::outward_normal(self, p.coords)
    }
}
