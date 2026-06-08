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
//!
//! **Source of geometry** (leg-region cutover): the oracle now reads its knee
//! parameters (`hip_in_pelvis`, flexion axis, the three coupled translation
//! splines) from the source-agnostic [`Model`] produced by [`crate::parse_leg_chain`]
//! — *not* from a bespoke `Subgraph`. The math below is unchanged from the
//! version validated against real OpenSim 4.6 (`tests/opensim_cross_check.rs`);
//! only the extraction is new. The general [`Model`] FK is proven equal to this
//! math to machine zero by `cf-msk-lib`'s `general_ir_fk` checkpoint, so the
//! oracle deliberately keeps its own minimal, validated knee math as the anchor.

use cf_msk_lib::ir::TransformFn;
use cf_msk_lib::{Model, Muscle, Spline};
use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3};

/// Which knee **modelling approximations** are active — an analysis vocabulary for
/// attributing moment-arm error to each G1 choice (coupled-translation freeze,
/// patella freeze, conditional-membership freeze). The production path uses only
/// [`Variant::TRUTH`] (the OpenSim ground truth); the frozen variants are kept for
/// per-approximation attribution (and A2's per-joint validation).
///
/// Note: these describe *oracle* approximations, not what the emitter produces.
/// The current `cf-mjcf-emit` emitter **drops** conditional points (a static
/// tendon can't toggle membership) rather than freezing their θ=0 membership, so
/// `freeze_conditional` is a different conditional-handling choice, not a
/// description of the emitted MJCF.
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
    /// All approximations frozen — the fully-frozen modelling extreme (what the
    /// retired S0 frozen-hinge emitter represented). Kept for error attribution;
    /// not produced by the current emitter.
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

/// The knee parameters the oracle math needs, extracted from a [`Model`].
struct KneeParams {
    /// Femur origin in the pelvis (world) frame — the femur's `location_in_parent`
    /// (hip welded at neutral).
    hip_in_pelvis: Vector3<f64>,
    /// The flexion rotation axis (the tibia joint's driven rotation axis).
    flexion_axis: Vector3<f64>,
    /// Coupled tibial translation splines along the femur x/y/z axes.
    tx: Spline,
    ty: Spline,
    tz: Spline,
}

/// Extract the knee parameters from the leg-chain [`Model`]: the femur placement,
/// the tibia joint's driven (non-constant) rotation axis, and its three coupled
/// translation functions matched by femur basis axis.
fn knee_params(model: &Model) -> KneeParams {
    let body = |name: &str| {
        &model.bodies[model
            .index_of(name)
            // the leg chain always carries these bodies; absence = a parse bug.
            .unwrap_or_else(|| panic!("model has no body '{name}'"))]
    };
    let tibia = body("tibia_r");

    // The driven flexion rotation. The oracle rotates by `theta` directly, so it
    // assumes the rotation function is the coordinate itself (unit-gain linear) —
    // the invariant the old `assert_unit_linear` guarded. Assert it so a non-unit
    // gain can't silently make the oracle disagree with the IR FK (which honors
    // the coefficient).
    let flexion = tibia
        .joint
        .iter()
        .find(|a| a.rotation && matches!(a.function, TransformFn::Linear { .. }))
        // the knee CustomJoint always has a driven (LinearFunction) rotation axis.
        .unwrap_or_else(|| panic!("knee has no driven flexion rotation axis"));
    if let TransformFn::Linear { coeff } = flexion.function {
        assert!(
            (coeff - 1.0).abs() < 1e-9,
            "knee flexion rotation must be unit-gain (coeff=1), got {coeff}"
        );
    }

    // Coupled tibial translation, matched by femur basis axis. Guard that every
    // translation axis is basis-aligned (the old `assert_axis` guarantee): a
    // non-canonical axis would otherwise be silently dropped to constant-0 below.
    let is_basis = |v: Vector3<f64>| {
        [Vector3::x(), Vector3::y(), Vector3::z()]
            .iter()
            .any(|b| (v - b).norm() < 1e-9)
    };
    for a in tibia.joint.iter().filter(|a| !a.rotation) {
        // gait2392's knee translations are along the canonical femur x/y/z axes.
        assert!(
            is_basis(a.axis),
            "non-canonical knee translation axis {:?} unsupported",
            a.axis
        );
    }
    let translation = |dir: Vector3<f64>| {
        tibia
            .joint
            .iter()
            .find(|a| !a.rotation && a.axis == dir)
            .map_or_else(|| Spline::constant(0.0), |a| as_spline(&a.function))
    };
    KneeParams {
        flexion_axis: flexion.axis,
        hip_in_pelvis: body("femur_r").location_in_parent,
        tx: translation(Vector3::x()),
        ty: translation(Vector3::y()),
        tz: translation(Vector3::z()),
    }
}

/// A coupled translation function as a [`Spline`] for the oracle math: a spline as
/// itself, a constant as a single-knot spline. The knee's translation axes are
/// only ever SimmSpline or Constant (a `Linear` translation would be a free slide
/// DOF, which the gait2392 knee does not use).
fn as_spline(f: &TransformFn) -> Spline {
    match f {
        TransformFn::Spline(s) => s.clone(),
        TransformFn::Constant(v) => Spline::constant(*v),
        TransformFn::Linear { .. } => {
            // a Linear translation would be a free slide DOF; the knee has none.
            panic!(
                "knee translation axis is a LinearFunction (a free slide) — unsupported by the oracle"
            )
        }
    }
}

/// Forward kinematics of the knee subgraph at a given knee angle.
pub struct Kinematics {
    knee: KneeParams,
}

impl Kinematics {
    pub fn new(model: &Model) -> Self {
        Self {
            knee: knee_params(model),
        }
    }

    /// Femur frame in the pelvis (world) frame — fixed (hip at neutral).
    fn femur(&self) -> Isometry3<f64> {
        Isometry3::translation(
            self.knee.hip_in_pelvis.x,
            self.knee.hip_in_pelvis.y,
            self.knee.hip_in_pelvis.z,
        )
    }

    /// Tibia frame in the pelvis frame at knee angle `theta`. The rotation
    /// always uses `theta`; the coupled translation uses θ=0 when frozen (R3),
    /// else the true splines: `x_femur = p(θ_t) + R_z(θ)·x_tibia`.
    fn tibia(&self, theta: f64, freeze_coupling: bool) -> Isometry3<f64> {
        let k = &self.knee;
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
            // S0 knee-only model; any other body name is a caller bug.
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
