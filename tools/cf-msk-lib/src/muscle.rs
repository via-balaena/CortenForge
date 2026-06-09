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

    /// Location in its body frame at coordinate value `theta`. A `MovingPathPoint`
    /// evaluates its per-axis splines at `theta` (its driving coordinate's value);
    /// plain/conditional points are θ-independent.
    pub fn location_at(&self, theta: f64) -> Vector3<f64> {
        match &self.kind {
            Kind::Moving(s) => Vector3::new(s.x.eval(theta), s.y.eval(theta), s.z.eval(theta)),
            _ => self.location,
        }
    }
}

/// A muscle's Millard2012 force-generating parameters (the engine's
/// `sim_core::MillardMuscleParams`, kept here dependency-free). `None` on a muscle
/// means only its path geometry was parsed (the kinematic-only path); the muscle
/// actuator emit needs these.
#[derive(Debug, Clone, Copy)]
pub struct MuscleForce {
    /// Max isometric force, newtons.
    pub f0: f64,
    /// Optimal fiber length, meters.
    pub l0: f64,
    /// Tendon slack length, meters.
    pub lts: f64,
    /// Max contraction velocity, optimal fiber lengths per second.
    pub vmax: f64,
    /// Pennation angle at optimal fiber length, radians.
    pub penn0: f64,
}

/// A muscle: an ordered path of points, plus optional force-generating parameters.
#[derive(Debug, Clone)]
pub struct Muscle {
    pub name: String,
    pub path: Vec<PathPoint>,
    /// Millard force parameters, if parsed (needed to emit a driven actuator).
    pub force: Option<MuscleForce>,
}
