//! The **oracle** — OpenSim's `GeometryPath` muscle geometry, computed from our IR.
//!
//! Because the vendored gait2392 has no wrap objects, a muscle's path length is
//! exactly the sum of straight segments between its *active* path points. This
//! module places those points in the world with the general [`Model`] forward
//! kinematics (`cf_msk_lib`) and sums the segments — i.e. it reproduces OpenSim's
//! `GeometryPath` length, the ground truth the converted MJCF is graded against.
//! Moment arm = `-dL/d(coordinate)`, the total derivative through any coupled
//! `CustomJoint`.
//!
//! A pose is a **coordinate → value map**, so the same oracle serves the knee
//! study *and* (leg-region A2) the articulated hip — any subset of coordinates can
//! be set; absent coordinates read as 0 (neutral). The FK it uses is the one the
//! emitter and checkpoints share, anchored against real OpenSim 4.6 in
//! `tests/opensim_cross_check.rs` (the independent truth; an earlier machine-zero
//! self-check vs a bespoke knee implementation is retired now that the general FK
//! *is* the oracle).

use cf_msk_lib::muscle::Kind;
use cf_msk_lib::{Model, Muscle};
use nalgebra::{Isometry3, Point3};
use std::collections::HashMap;

/// A pose: generalized-coordinate values keyed by name. Absent coordinates are 0.
pub type Pose = HashMap<String, f64>;

/// The coordinate that gates a `ConditionalPathPoint`'s membership.
///
/// A `MovingPathPoint`'s location is evaluated at its *own* retained coordinate
/// (`MovingSplines.coordinate`, the same one the emitter drives the patella with),
/// so moving points generalize beyond one DOF. A `ConditionalPathPoint`, however,
/// carries no coordinate in the IR (`Kind::Conditional` is just `{lo, hi}`), so
/// membership is gated on this one — the knee angle. gait2392's conditionals are
/// all knee-driven, so this is exact today; retaining the conditional's coordinate
/// (to drop this constant) is a future generalization.
const MUSCLE_PARAM_COORD: &str = "knee_angle_r";

/// Forward kinematics + muscle geometry over the leg-chain [`Model`].
pub struct Kinematics<'a> {
    model: &'a Model,
}

impl<'a> Kinematics<'a> {
    pub fn new(model: &'a Model) -> Self {
        Self { model }
    }

    /// World pose of `body` at the multi-coordinate pose `q` (absent coord = 0).
    /// A name not in the tree (e.g. `ground`/`pelvis`) is the world frame.
    pub fn body_pose(&self, body: &str, q: &Pose) -> Isometry3<f64> {
        self.model.body_pose(body, q)
    }

    /// Muscle path length at pose `q` (m): the sum of straight segments between the
    /// muscle's active path points, each placed by the general FK. A moving point's
    /// location is evaluated at its own retained coordinate's value in `q` (matching
    /// the emitter); conditional membership uses `MUSCLE_PARAM_COORD` (conditionals
    /// carry no coordinate in the IR).
    pub fn path_length(&self, muscle: &Muscle, q: &Pose) -> f64 {
        let val = |name: &str| q.get(name).copied().unwrap_or(0.0);
        let theta = val(MUSCLE_PARAM_COORD);
        let pts: Vec<Point3<f64>> = muscle
            .path
            .iter()
            .filter(|p| p.active(theta))
            .map(|p| {
                // `body_pose` maps an unknown name to the world frame for ground
                // convenience; a muscle point must ride a real body, else it would
                // be silently placed at the origin. Fail loudly (the emitter does too).
                assert!(
                    self.model.index_of(&p.body).is_some(),
                    "muscle point '{}' rides unknown body '{}'",
                    p.name,
                    p.body
                );
                // A moving point follows its own driving coordinate (as the emitter
                // drives the patella); fixed/conditional points are θ-independent.
                let pt = match &p.kind {
                    Kind::Moving(s) => val(&s.coordinate),
                    _ => theta,
                };
                self.model.body_pose(&p.body, q) * Point3::from(p.location_at(pt))
            })
            .collect();
        pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
    }

    /// Moment arm of `muscle` about coordinate `coord` at pose `q` (m):
    /// `-dL/d(q[coord])` by central difference — the total derivative along the
    /// coupled manifold (any coupling driven by `coord` moves with it via the FK).
    pub fn moment_arm(&self, muscle: &Muscle, q: &Pose, coord: &str, eps: f64) -> f64 {
        let perturbed = |d: f64| {
            let mut p = q.clone();
            *p.entry(coord.to_string()).or_insert(0.0) += d;
            self.path_length(muscle, &p)
        };
        -(perturbed(eps) - perturbed(-eps)) / (2.0 * eps)
    }
}
