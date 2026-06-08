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
//! v1 reuses `cf-osim`'s [`Spline`] and muscle path types (`Muscle`/`PathPoint`)
//! verbatim; the direct `.osim → Model` parser and the dependency inversion that
//! moves those types here are the next increment (recon leg-region A1, PR-2). For
//! now [`Model::from_subgraph`] bridges the existing knee `Subgraph` into the IR
//! so the general FK can be exercised and oracle-checked additively, with nothing
//! removed from the validated knee path.
//!
//! **FK convention** (the thing the spike validated): the joint transform at
//! coordinates `q` composes the rotation axes (product) and translation axes
//! (sum) into `x → R·x + t`, and a body's world pose is
//! `world(parent) · T(location_in_parent) · joint_xform(q)`. This is exactly the
//! OpenSim CustomJoint convention (`Isometry3::from_parts(translation, rotation)`)
//! the knee oracle was validated against. Rotation order only matters away from
//! neutral; A1 holds the multi-DOF hip at neutral.

use cf_osim::osim::{Muscle, Spline, Subgraph};
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
/// `axis`, by `func` of the coordinate named `coord`.
///
/// `coord` may be empty for a constant axis whose function ignores the
/// coordinate (gait2392 leaves `<coordinates>` blank for such axes); the value
/// then evaluates with `q = 0`, which a [`TransformFn::Constant`] ignores anyway.
#[derive(Debug, Clone)]
pub struct TransformAxis {
    /// `true` = rotation about `axis`; `false` = translation along `axis`.
    pub rotation: bool,
    /// The axis (body frame) the transform acts about/along.
    pub axis: Vector3<f64>,
    /// The coordinate that drives `func` (empty for a coordinate-independent axis).
    pub coord: String,
    /// The transform amount as a function of the coordinate.
    pub func: TransformFn,
}

/// A joint = the ordered list of its transform axes (OpenSim `SpatialTransform`:
/// rotations first, then translations). Rotation axes compose as a product,
/// translation axes as a sum.
pub type Joint = Vec<TransformAxis>;

/// A generalized coordinate (DOF) of the model.
///
/// `range` is `None` when the source can't supply it — the [`Model::from_subgraph`]
/// bridge has no range information, whereas the direct `.osim` parser (PR-2) fills
/// it. FK does not use `default`/`range`; they exist for the emitter (joint ranges)
/// and for posing the model at its defaults.
#[derive(Debug, Clone)]
pub struct Coordinate {
    pub name: String,
    pub default: f64,
    pub range: Option<(f64, f64)>,
}

/// A body in the kinematic tree: its parent (index into [`Model::bodies`], `None`
/// for the root), the fixed offset to the joint frame (`location_in_parent`), and
/// the joint that connects it to its parent.
#[derive(Debug, Clone)]
pub struct Body {
    pub name: String,
    pub parent: Option<usize>,
    pub loc_in_parent: Vector3<f64>,
    pub joint: Joint,
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

    /// The joint transform for `body` at coordinates `q`: compose the rotation
    /// axes (product) and translation axes (sum), then `x → R·x + t`
    /// (`Isometry3::from_parts(translation, rotation)` — the validated CustomJoint
    /// convention). A coordinate absent from `q` reads as 0.
    pub fn joint_xform(&self, body: &Body, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let mut rot = UnitQuaternion::identity();
        let mut trans = Vector3::zeros();
        for ax in &body.joint {
            let val = ax.func.eval(q.get(&ax.coord).copied().unwrap_or(0.0));
            if ax.rotation {
                rot *= UnitQuaternion::from_axis_angle(&Unit::new_normalize(ax.axis), val);
            } else {
                trans += ax.axis * val;
            }
        }
        Isometry3::from_parts(Translation3::from(trans), rot)
    }

    /// World pose of body `idx` at coordinates `q` — walk the parent chain.
    pub fn world_pose(&self, idx: usize, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let b = &self.bodies[idx];
        let parent = match b.parent {
            Some(p) => self.world_pose(p, q),
            None => Isometry3::identity(),
        };
        parent * Translation3::from(b.loc_in_parent) * self.joint_xform(b, q)
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

    /// Build the general tree that represents the knee chain from the parsed knee
    /// [`Subgraph`] — the productionized form of the A1 spike's `knee_chain_model`:
    /// `pelvis` (root) → `femur_r` (hip held at neutral: just the `hip_in_pelvis`
    /// offset, no DOFs) → `tibia_r` (the coupled knee: a unit-gain flexion rotation
    /// about the flexion axis plus the three coordinate-coupled translation splines
    /// along the femur x/y/z axes).
    ///
    /// This is the additive PR-1 bridge: it lets the general FK be exercised and
    /// oracle-checked without inverting the cf-osim ↔ cf-msk-lib dependency. The
    /// direct `.osim → Model` parser (which also reads the real hip joint for A2)
    /// replaces it in PR-2.
    pub fn from_subgraph(sub: &Subgraph) -> Model {
        let k = &sub.knee;
        let bodies = vec![
            Body {
                name: "pelvis".into(),
                parent: None,
                loc_in_parent: Vector3::zeros(),
                joint: vec![],
            },
            Body {
                name: "femur_r".into(),
                parent: Some(0),
                loc_in_parent: sub.hip_in_pelvis,
                joint: vec![], // hip welded at neutral for the knee study
            },
            Body {
                name: "tibia_r".into(),
                parent: Some(1),
                loc_in_parent: Vector3::zeros(),
                joint: vec![
                    TransformAxis {
                        rotation: true,
                        axis: k.flexion_axis,
                        coord: "knee_angle_r".into(),
                        func: TransformFn::Linear { coeff: 1.0 },
                    },
                    TransformAxis {
                        rotation: false,
                        axis: Vector3::x(),
                        coord: "knee_angle_r".into(),
                        func: TransformFn::Spline(k.tx.clone()),
                    },
                    TransformAxis {
                        rotation: false,
                        axis: Vector3::y(),
                        coord: "knee_angle_r".into(),
                        func: TransformFn::Spline(k.ty.clone()),
                    },
                    TransformAxis {
                        rotation: false,
                        axis: Vector3::z(),
                        coord: "knee_angle_r".into(),
                        func: TransformFn::Spline(k.tz.clone()),
                    },
                ],
            },
        ];
        Model {
            bodies,
            coordinates: vec![Coordinate {
                name: "knee_angle_r".into(),
                default: 0.0,
                range: None,
            }],
            muscles: sub.muscles.clone(),
        }
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
            loc_in_parent: Vector3::zeros(),
            joint: vec![
                TransformAxis {
                    rotation: false,
                    axis: Vector3::x(),
                    coord: "q".into(),
                    func: TransformFn::Constant(0.3),
                },
                TransformAxis {
                    rotation: false,
                    axis: Vector3::y(),
                    coord: "q".into(),
                    func: TransformFn::Constant(0.4),
                },
                TransformAxis {
                    rotation: true,
                    axis: Vector3::z(),
                    coord: "q".into(),
                    func: TransformFn::Linear { coeff: 1.0 },
                },
            ],
        };
        let model = Model {
            bodies: vec![],
            coordinates: vec![],
            muscles: vec![],
        };
        let q = HashMap::from([("q".to_string(), std::f64::consts::FRAC_PI_2)]);
        let x = model.joint_xform(&body, &q);
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
                    loc_in_parent: Vector3::new(1.0, 0.0, 0.0),
                    joint: vec![],
                },
                Body {
                    name: "child".into(),
                    parent: Some(0),
                    loc_in_parent: Vector3::new(0.0, 2.0, 0.0),
                    joint: vec![],
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
