//! Muscle path geometry — a muscle as an ordered list of [`PathPoint`]s.
//!
//! gait2392 is wrap-free, so a muscle's path is a straight-segment polyline
//! between its *active* path points. A point is placed on a named body and may be
//! fixed, conditional (active only over a coordinate range), or moving (its body-
//! frame location is a per-axis [`Spline`] of the coordinate — the patella
//! emulation).
//!
//! (Moved verbatim from `cf-osim::osim` when the source-agnostic IR became the
//! home of the model types.)

use crate::spline::Spline;
use nalgebra::Vector3;

/// The per-axis location splines of a `MovingPathPoint` (boxed in [`Kind`] so the
/// enum stays small), plus the coordinate that drives them.
///
/// `coordinate` is the generalized coordinate the location splines are functions
/// of (gait2392's patella points are driven by the knee angle). Retaining it — vs
/// assuming a single muscle-parameter coordinate — is what lets a moving point be
/// driven correctly once the chain has more than one free DOF (the A2 hip): the
/// emitter wires the patella's coupled slides to *this* coordinate, not a guess.
#[derive(Debug, Clone)]
pub struct MovingSplines {
    pub coordinate: String,
    pub x: Spline,
    pub y: Spline,
    pub z: Spline,
}

/// How a path point is placed.
#[derive(Debug, Clone)]
pub enum Kind {
    /// Always active, fixed location on its body.
    Fixed,
    /// Active only while the coordinate ∈ [lo, hi]; fixed location when active.
    Conditional { lo: f64, hi: f64 },
    /// Always active; location is a per-axis spline of the coordinate (patella).
    Moving(Box<MovingSplines>),
}

/// One muscle path point on a named body.
#[derive(Debug, Clone)]
pub struct PathPoint {
    pub name: String,
    pub body: String,
    /// Location for `Fixed`/`Conditional`; unused for `Moving` (use the splines).
    pub location: Vector3<f64>,
    pub kind: Kind,
}

impl PathPoint {
    /// Is this point part of the path at coordinate value `theta`?
    pub fn active(&self, theta: f64) -> bool {
        match self.kind {
            Kind::Conditional { lo, hi } => theta >= lo && theta <= hi,
            _ => true,
        }
    }

    /// Activity used by a given variant. When `freeze_conditional`, a
    /// `ConditionalPathPoint` is included iff it is active at θ=0 (matching the
    /// fixed-site emit, which can't toggle path membership with angle).
    pub fn active_under(&self, theta: f64, freeze_conditional: bool) -> bool {
        self.active(if freeze_conditional { 0.0 } else { theta })
    }

    /// Location in its body frame at coordinate value `theta`. When `freeze_moving`,
    /// a `MovingPathPoint` is pinned to its θ=0 location (the G1 patella
    /// approximation, O2); plain/conditional points are unaffected.
    pub fn location_at(&self, theta: f64, freeze_moving: bool) -> Vector3<f64> {
        match &self.kind {
            Kind::Moving(s) => {
                let t = if freeze_moving { 0.0 } else { theta };
                Vector3::new(s.x.eval(t), s.y.eval(t), s.z.eval(t))
            }
            _ => self.location,
        }
    }
}

/// A muscle: an ordered path of points.
#[derive(Debug, Clone)]
pub struct Muscle {
    pub name: String,
    pub path: Vec<PathPoint>,
}
