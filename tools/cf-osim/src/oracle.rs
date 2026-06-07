//! The S0 **oracle** — OpenSim's own muscle-path geometry, computed directly.
//!
//! Because the vendored gait2392 has no wrap objects, a muscle's path length is
//! exactly the sum of straight segments between its *active* path points. This
//! module places those points in the world using the **true** knee kinematics
//! (coupled tibial translation + spline-driven moving points) and sums the
//! segments — i.e. it reproduces OpenSim's `GeometryPath` length, which is the
//! ground truth the converted MJCF is graded against. Moment arm = `-dL/dθ`.
//!
//! Knee-only study: the hip (and everything proximal) is held at neutral, so
//! `pelvis` is the world frame and `femur_r` is fixed at the hip location.

use crate::osim::{Muscle, Subgraph};
use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion};

/// Which knee approximations are active — lets us attribute the moment-arm
/// error between the two G1 modelling choices and cross-check the engine.
#[derive(Clone, Copy, Debug)]
pub struct Variant {
    /// Freeze the coupled tibial translation at θ=0 (the fixed-hinge approx, R3).
    pub freeze_coupling: bool,
    /// Freeze `MovingPathPoint`s at θ=0 (the fixed-patella approx, O2).
    pub freeze_moving: bool,
    /// Freeze `ConditionalPathPoint` membership at θ=0 (fixed sites can't toggle, C).
    pub freeze_conditional: bool,
}

impl Variant {
    /// OpenSim ground truth — nothing frozen.
    pub const TRUTH: Variant = Variant {
        freeze_coupling: false,
        freeze_moving: false,
        freeze_conditional: false,
    };
    /// What the frozen-hinge MJCF does — all frozen. The engine should match this.
    pub const ENGINE: Variant = Variant {
        freeze_coupling: true,
        freeze_moving: true,
        freeze_conditional: true,
    };
    /// Truth except the moving point is frozen — isolates the patella (O2) error.
    pub const FREEZE_MOVING_ONLY: Variant = Variant {
        freeze_coupling: false,
        freeze_moving: true,
        freeze_conditional: false,
    };
    /// Truth except the coupling is frozen — isolates the hinge (R3) error.
    pub const FREEZE_COUPLING_ONLY: Variant = Variant {
        freeze_coupling: true,
        freeze_moving: false,
        freeze_conditional: false,
    };
    /// Truth except conditional membership is frozen — isolates the (C) error.
    pub const FREEZE_CONDITIONAL_ONLY: Variant = Variant {
        freeze_coupling: false,
        freeze_moving: false,
        freeze_conditional: true,
    };
}

/// Forward kinematics of the knee subgraph at a given knee angle.
pub struct Kinematics<'a> {
    sub: &'a Subgraph,
}

impl<'a> Kinematics<'a> {
    pub fn new(sub: &'a Subgraph) -> Self {
        Self { sub }
    }

    /// Femur frame in the pelvis (world) frame — fixed (hip at neutral).
    fn femur(&self) -> Isometry3<f64> {
        Isometry3::translation(
            self.sub.hip_in_pelvis.x,
            self.sub.hip_in_pelvis.y,
            self.sub.hip_in_pelvis.z,
        )
    }

    /// Tibia frame in the pelvis frame at knee angle `theta`. The rotation
    /// always uses `theta`; the coupled translation uses θ=0 when frozen (R3),
    /// else the true splines: `x_femur = p(θ_t) + R_z(θ)·x_tibia`.
    fn tibia(&self, theta: f64, freeze_coupling: bool) -> Isometry3<f64> {
        let k = &self.sub.knee;
        let tt = if freeze_coupling { 0.0 } else { theta };
        let t = Translation3::new(k.tx.eval(tt), k.ty.eval(tt), k.tz.eval(tt));
        let r = UnitQuaternion::from_axis_angle(&Unit::new_normalize(k.flexion_axis), theta);
        self.femur() * Isometry3::from_parts(t, r)
    }

    /// Pose of a named body at knee angle `theta` under `variant`.
    pub fn body_pose(&self, body: &str, theta: f64, variant: Variant) -> Isometry3<f64> {
        match body {
            "pelvis" | "ground" => Isometry3::identity(),
            "femur_r" => self.femur(),
            "tibia_r" => self.tibia(theta, variant.freeze_coupling),
            other => panic!("S0 knee-only model has no body '{other}'"),
        }
    }

    /// Muscle path length at knee angle `theta` under `variant` (m): sum of
    /// straight segments between active path points. `Variant::TRUTH` is
    /// OpenSim's `GeometryPath` length; `Variant::ENGINE` mirrors the frozen
    /// MJCF (and should match the engine's `ten_length`).
    pub fn path_length(&self, muscle: &Muscle, theta: f64, variant: Variant) -> f64 {
        let pts: Vec<Point3<f64>> = muscle
            .path
            .iter()
            .filter(|p| p.active_under(theta, variant.freeze_conditional))
            .map(|p| {
                self.body_pose(&p.body, theta, variant)
                    * Point3::from(p.location_at(theta, variant.freeze_moving))
            })
            .collect();
        pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
    }

    /// Moment arm of `muscle` about the knee at `theta` under `variant` (m):
    /// `-dL/dθ` via central difference.
    pub fn moment_arm(&self, muscle: &Muscle, theta: f64, eps: f64, variant: Variant) -> f64 {
        -(self.path_length(muscle, theta + eps, variant)
            - self.path_length(muscle, theta - eps, variant))
            / (2.0 * eps)
    }
}
