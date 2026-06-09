//! The general kinematic-tree IR — a source-agnostic kinematic tree that
//! replaces the bespoke knee `Subgraph` as the thing this crate morphs and emits.
//!
//! This is the productionized form of the A1 spike (`cf-osim`'s throwaway
//! `spike_general_fk.rs`), which proved that composing OpenSim `CustomJoint`
//! transform axes and walking the parent chain reproduces the validated knee
//! oracle moment arms to **machine zero**. Rests on the gait2392 cold-read: every
//! joint is a `CustomJoint` whose axes use Linear (a DOF) / Constant / SimmSpline
//! (coupled) functions, and the model is wrap-free — so one general [`Joint`]
//! (a list of transform axes) plus straight-segment muscle paths cover the whole
//! leg with no wrap math.
//!
//! The IR mirrors OpenSim's own structure: a [`TransformAxis`] carries its
//! `coordinate`, `axis`, and `function` as siblings (the `<coordinates>`/`<axis>`/
//! `<function>` elements of a `CustomJoint` `SpatialTransform`). This is
//! intentionally flatter than the leg-region recon's `TransformFn { coordinate,
//! kind }` sketch — keeping the coordinate on the axis matches the source model
//! one-to-one and is what the validated spike used.
//!
//! This crate owns the model types ([`Spline`], [`Muscle`]/[`crate::PathPoint`]); the
//! `.osim` reader produces a [`Model`] directly (`cf_osim::parse_leg_chain`), so
//! the dependency points cf-osim → cf-msk-lib. This FK *is* `cf-osim`'s oracle,
//! anchored against real OpenSim 4.6 in `cf-osim`'s `opensim_cross_check`.
//!
//! **FK convention** (the thing the spike validated): the joint transform at
//! coordinates `q` composes the rotation axes (product) and translation axes
//! (sum) into `x → R·x + t`, and a body's world pose is
//! `world(parent) · T(location_in_parent) · joint_xform(q)`. This is exactly the
//! OpenSim CustomJoint convention (`Isometry3::from_parts(translation, rotation)`)
//! the knee oracle was validated against. Rotation order only matters away from
//! neutral; A1 holds the multi-DOF hip at neutral.

use crate::muscle::Muscle;
use crate::spline::Spline;
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use std::collections::HashMap;

/// A `CustomJoint` transform-axis function of a single coordinate.
///
/// gait2392's transform functions are all single-coordinate: a `LinearFunction`
/// (a DOF — `coeff·q`, where `coeff = 1` makes the transform equal the
/// coordinate), a `Constant`, or a `SimmSpline` (a coupled DOF). [`Spline`]
/// already carries a constant as a single knot, but a dedicated [`TransformFn::Constant`]
/// keeps constant axes explicit (e.g. a joint's fixed offsets).
#[derive(Debug, Clone)]
pub enum TransformFn {
    /// `coeff · q` — a linear function of the coordinate (a DOF when `coeff = 1`).
    Linear { coeff: f64 },
    /// A constant, independent of the coordinate.
    Constant(f64),
    /// A natural-cubic `SimmSpline` of the coordinate (a coupled DOF).
    Spline(Spline),
}

impl TransformFn {
    /// Evaluate the function at coordinate value `q`.
    pub fn eval(&self, q: f64) -> f64 {
        match self {
            TransformFn::Linear { coeff } => coeff * q,
            TransformFn::Constant(v) => *v,
            TransformFn::Spline(s) => s.eval(q),
        }
    }
}

/// One transform axis of a `CustomJoint`: a rotation about / translation along
/// `axis`, by `function` of the coordinate named `coordinate`.
///
/// `coordinate` may be empty for a constant axis whose function ignores it
/// (gait2392 leaves `<coordinates>` blank for such axes); the value then evaluates
/// with `q = 0`, which a [`TransformFn::Constant`] ignores anyway.
#[derive(Debug, Clone)]
pub struct TransformAxis {
    /// `true` = rotation about `axis`; `false` = translation along `axis`.
    pub rotation: bool,
    /// The axis (body frame) the transform acts about/along.
    pub axis: Vector3<f64>,
    /// The coordinate that drives `function` (empty for a coordinate-independent axis).
    pub coordinate: String,
    /// The transform amount as a function of the coordinate.
    pub function: TransformFn,
}

/// A joint = the ordered list of its transform axes (OpenSim `SpatialTransform`:
/// rotations first, then translations). Rotation axes compose as a product,
/// translation axes as a sum.
pub type Joint = Vec<TransformAxis>;

/// A generalized coordinate (DOF) of the model.
///
/// `range` is `None` when the source can't supply it. FK does not use
/// `default`/`range`; they exist for the emitter (joint ranges) and for posing the
/// model at its defaults.
#[derive(Debug, Clone)]
pub struct Coordinate {
    pub name: String,
    pub default: f64,
    pub range: Option<(f64, f64)>,
}

/// A body's mass distribution: total mass (kg), center of mass in the body frame
/// (m), and the symmetric inertia tensor about that CoM in the body frame, stored
/// as the six independent components `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` (kg·m², the
/// MuJoCo `fullinertia` order). The general emitter lowers this to
/// `<inertial pos mass fullinertia>`; the loader eigendecomposes to principal axes,
/// so no eigendecomposition is needed here. gait2392's femur/tibia are diagonal
/// (products 0); the lumped foot (the `talus_r` composite) carries products.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia {
    pub mass: f64,
    pub com: Vector3<f64>,
    /// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` about [`com`](Self::com), body frame.
    pub tensor: [f64; 6],
}

/// A body in the kinematic tree: its parent (index into [`Model::bodies`], `None`
/// for the root), the fixed offset to the joint frame (`location_in_parent`), the
/// joint that connects it to its parent, and its mass distribution.
///
/// `inertia` is `None` when the source has no mass data (a synthetic/kinematic-only
/// body), mirroring [`crate::Muscle::force`]. The emitter requires it on any body
/// that carries a free DOF (a moving body needs a mass matrix) and fails loud
/// otherwise; a welded root (the pelvis) needs none.
#[derive(Debug, Clone)]
pub struct Body {
    pub name: String,
    pub parent: Option<usize>,
    pub location_in_parent: Vector3<f64>,
    pub joint: Joint,
    pub inertia: Option<Inertia>,
}

impl Body {
    /// This body's joint transform at coordinates `q`: compose the rotation axes
    /// (product) and translation axes (sum), then `x → R·x + t`
    /// (`Isometry3::from_parts(translation, rotation)` — the validated CustomJoint
    /// convention). A coordinate absent from `q` reads as 0.
    pub fn joint_xform(&self, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let mut rot = UnitQuaternion::identity();
        let mut trans = Vector3::zeros();
        for ax in &self.joint {
            let val = ax
                .function
                .eval(q.get(&ax.coordinate).copied().unwrap_or(0.0));
            if ax.rotation {
                rot *= UnitQuaternion::from_axis_angle(&Unit::new_normalize(ax.axis), val);
            } else {
                trans += ax.axis * val;
            }
        }
        Isometry3::from_parts(Translation3::from(trans), rot)
    }
}

/// A source-agnostic musculoskeletal kinematic tree: a `Body` tree, its
/// coordinates, and its (straight-segment) muscle paths.
#[derive(Debug, Clone)]
pub struct Model {
    pub bodies: Vec<Body>,
    pub coordinates: Vec<Coordinate>,
    pub muscles: Vec<Muscle>,
}

impl Model {
    /// Index of the body named `name`, if present.
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    /// World pose of body `idx` at coordinates `q` — walk the parent chain.
    pub fn world_pose(&self, idx: usize, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let b = &self.bodies[idx];
        let parent = match b.parent {
            Some(p) => self.world_pose(p, q),
            None => Isometry3::identity(),
        };
        parent * Translation3::from(b.location_in_parent) * b.joint_xform(q)
    }

    /// World pose of the body named `name` at coordinates `q`. A name not in the
    /// tree (e.g. `"ground"`) maps to the identity (the world frame), matching the
    /// oracle's convention where `ground`/`pelvis` is the fixed world frame.
    pub fn body_pose(&self, name: &str, q: &HashMap<String, f64>) -> Isometry3<f64> {
        match self.index_of(name) {
            Some(idx) => self.world_pose(idx, q),
            None => Isometry3::identity(),
        }
    }

    /// The default pose: every coordinate at its `default` value. The canonical
    /// (neutral standing) configuration the body is measured/morphed at.
    pub fn default_pose(&self) -> HashMap<String, f64> {
        self.coordinates
            .iter()
            .map(|c| (c.name.clone(), c.default))
            .collect()
    }

    /// The **axial length** of the segment between two bodies, at the default pose:
    /// the distal body's frame origin expressed in the proximal body's frame,
    /// projected onto the proximal frame's long axis (body-frame *y* — the gait2392
    /// limb long axis), as a magnitude.
    ///
    /// This is the quantity a per-axis morph scales *exactly*: scaling the
    /// proximal segment's axial (*y*) factor by `k` scales this by `k`, independent
    /// of any small transverse (condylar) offset that makes the Euclidean
    /// hip→knee distance scale only approximately. It is the length A3 dials.
    /// `femur` axial = `segment_axial_length("femur_r", "tibia_r")` (hip→knee);
    /// `tibia` axial = `segment_axial_length("tibia_r", "talus_r")` (knee→ankle).
    pub fn segment_axial_length(&self, proximal: &str, distal: &str) -> f64 {
        let q = self.default_pose();
        let distal_o = self.body_pose(distal, &q) * nalgebra::Point3::origin();
        let in_proximal = self.body_pose(proximal, &q).inverse() * distal_o;
        in_proximal.y.abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_fn_eval() {
        assert_eq!(TransformFn::Linear { coeff: 2.0 }.eval(3.0), 6.0);
        assert_eq!(TransformFn::Constant(0.5).eval(99.0), 0.5);
        assert_eq!(TransformFn::Spline(Spline::constant(0.7)).eval(1.2), 0.7);
    }

    /// Translation axes sum, rotation axes compose; the result maps `x → R·x + t`.
    #[test]
    fn joint_xform_sums_translations_and_composes_rotations() {
        let body = Body {
            name: "b".into(),
            parent: None,
            location_in_parent: Vector3::zeros(),
            joint: vec![
                TransformAxis {
                    rotation: false,
                    axis: Vector3::x(),
                    coordinate: "q".into(),
                    function: TransformFn::Constant(0.3),
                },
                TransformAxis {
                    rotation: false,
                    axis: Vector3::y(),
                    coordinate: "q".into(),
                    function: TransformFn::Constant(0.4),
                },
                TransformAxis {
                    rotation: true,
                    axis: Vector3::z(),
                    coordinate: "q".into(),
                    function: TransformFn::Linear { coeff: 1.0 },
                },
            ],
            inertia: None,
        };
        let q = HashMap::from([("q".to_string(), std::f64::consts::FRAC_PI_2)]);
        let x = body.joint_xform(&q);
        // Translation = (0.3, 0.4, 0); rotation = +90° about z maps x̂ → ŷ.
        assert!((x.translation.vector - Vector3::new(0.3, 0.4, 0.0)).norm() < 1e-12);
        let mapped = x * nalgebra::Point3::new(1.0, 0.0, 0.0);
        assert!((mapped - nalgebra::Point3::new(0.3, 1.4, 0.0)).norm() < 1e-12);
    }

    /// World pose walks the parent chain: a child's offset stacks on its parent's.
    #[test]
    fn world_pose_walks_parent_chain() {
        let model = Model {
            bodies: vec![
                Body {
                    name: "root".into(),
                    parent: None,
                    location_in_parent: Vector3::new(1.0, 0.0, 0.0),
                    joint: vec![],
                    inertia: None,
                },
                Body {
                    name: "child".into(),
                    parent: Some(0),
                    location_in_parent: Vector3::new(0.0, 2.0, 0.0),
                    joint: vec![],
                    inertia: None,
                },
            ],
            coordinates: vec![],
            muscles: vec![],
        };
        let q = HashMap::new();
        let p = model.body_pose("child", &q);
        assert!((p.translation.vector - Vector3::new(1.0, 2.0, 0.0)).norm() < 1e-12);
        // An unknown body name is the world frame (identity).
        assert_eq!(model.body_pose("ground", &q), Isometry3::identity());
    }
}
