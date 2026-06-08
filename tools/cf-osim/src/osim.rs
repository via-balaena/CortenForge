//! Reader for the gait2392 **leg chain** (pelvis → femur → tibia) into the
//! source-agnostic [`Model`] IR (`cf_msk_lib`).
//!
//! `.osim` version: OpenSim 3.x (`Version="30000"`), where each `Body` owns the
//! `Joint` that connects it to its parent. Every joint is a `CustomJoint` whose
//! `SpatialTransform` lists rotation axes first, then translation axes, each
//! driven by a `LinearFunction` (a DOF), a `Constant`, or a `SimmSpline` (a
//! coupled DOF) of a named coordinate. gait2392 is wrap-free, so muscle paths are
//! straight-segment polylines between active path points.
//!
//! A1 reads the chain with the **hip welded at neutral**: the femur is placed by
//! the hip joint's `location_in_parent` with no DOFs, and only the knee's
//! coordinate is a free DOF. Unwelding the hip (parsing its `SpatialTransform`
//! into the femur joint + adding its coordinates) is the A2 step.

use crate::xml::{self, Node};
use cf_msk_lib::ir::{Body, Coordinate, Model, TransformAxis, TransformFn};
use cf_msk_lib::{Kind, MovingSplines, Muscle, PathPoint, Spline};
use nalgebra::Vector3;

/// The four muscles the builder converts + validates: two extensors whose
/// effective insertion rides a `MovingPathPoint` (the patella emulation), and
/// two flexors built from plain points.
pub const TARGET_MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];

/// Parse the gait2392 leg chain (pelvis → femur → tibia, hip welded at neutral)
/// plus the target muscles into the [`Model`] IR.
pub fn parse_leg_chain(osim_xml: &str) -> Model {
    let root = xml::parse(osim_xml);
    let hip = find_custom_joint(&root, "hip_r");
    let knee = find_custom_joint(&root, "knee_r");

    let pelvis = Body {
        name: "pelvis".into(),
        parent: None,
        location_in_parent: Vector3::zeros(),
        joint: vec![],
    };
    // Femur: hip welded at neutral → placed by the hip joint's location_in_parent,
    // no DOFs. (A2 unwelds it: parse the hip SpatialTransform into this joint.)
    let femur = Body {
        name: "femur_r".into(),
        parent: Some(0),
        location_in_parent: joint_location(hip),
        joint: vec![],
    };
    // Tibia: the knee CustomJoint, read generically.
    let tibia = Body {
        name: "tibia_r".into(),
        parent: Some(1),
        location_in_parent: joint_location(knee),
        joint: parse_spatial_transform(knee),
    };

    Model {
        bodies: vec![pelvis, femur, tibia],
        // Only the knee's coordinate is free for A1 (the hip is welded).
        coordinates: parse_coordinates(knee),
        muscles: TARGET_MUSCLES
            .iter()
            .map(|m| parse_muscle(&root, m))
            .collect(),
    }
}

fn find_custom_joint<'a>(root: &'a Node, name: &str) -> &'a Node {
    root.find_with_attr("CustomJoint", "name", name)
        // the vendored gait2392 always defines hip_r / knee_r.
        .unwrap_or_else(|| panic!("<CustomJoint name=\"{name}\"> not found"))
}

fn joint_location(joint: &Node) -> Vector3<f64> {
    joint
        .child("location_in_parent")
        .map(vec3)
        // a CustomJoint always carries location_in_parent (default 0).
        .unwrap_or_else(Vector3::zeros)
}

/// Read a `CustomJoint`'s `CoordinateSet` into the IR coordinates (name, default,
/// range).
fn parse_coordinates(joint: &Node) -> Vec<Coordinate> {
    joint
        .find("CoordinateSet")
        .and_then(|s| s.child("objects"))
        .map(|objs| {
            objs.children
                .iter()
                .filter(|c| c.name == "Coordinate")
                .map(|c| {
                    let r = c.child("range").map(|n| n.floats()).unwrap_or_default();
                    Coordinate {
                        name: c.attr("name").unwrap_or("").to_string(),
                        default: c
                            .child("default_value")
                            .and_then(|n| n.floats().first().copied())
                            .unwrap_or(0.0),
                        range: (r.len() >= 2).then(|| (r[0], r[1])),
                    }
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Read a `CustomJoint`'s `SpatialTransform` into the IR joint: each
/// `TransformAxis` (rotation1..3, translation1..3) becomes a [`TransformAxis`]
/// carrying its rotation flag, axis, driving coordinate, and function.
fn parse_spatial_transform(joint: &Node) -> Vec<TransformAxis> {
    let spatial = joint
        .child("SpatialTransform")
        // every gait2392 CustomJoint defines a SpatialTransform.
        .unwrap_or_else(|| panic!("CustomJoint has no SpatialTransform"));
    spatial
        .children
        .iter()
        .filter(|c| c.name == "TransformAxis")
        .map(|ax| {
            let name = ax.attr("name").unwrap_or("");
            TransformAxis {
                // OpenSim lists rotation1..3 then translation1..3.
                rotation: name.starts_with("rotation"),
                axis: ax
                    .child("axis")
                    .map(vec3)
                    // every TransformAxis carries an <axis>.
                    .unwrap_or_else(|| panic!("TransformAxis '{name}' has no <axis>")),
                coordinate: ax
                    .child("coordinates")
                    .map(|n| n.text_trim().to_string())
                    .unwrap_or_default(),
                function: parse_transform_fn(ax),
            }
        })
        .collect()
}

/// Parse a `TransformAxis`'s `<function>` into a [`TransformFn`]: a
/// `LinearFunction` (a DOF), a `Constant`, or a `SimmSpline` (a coupled DOF),
/// with any `MultiplierFunction` scale folded in. Navigates explicitly and panics
/// on any other shape so a structurally different model can't be silently
/// mis-converted.
///
/// The `MultiplierFunction` wrapper is resolved FIRST (its `scale` + inner
/// `function`), then the inner shape is classified — so a multiplier-wrapped
/// `LinearFunction` keeps its scale instead of being matched as a bare linear by a
/// descendant search.
fn parse_transform_fn(axis_or_fn: &Node) -> TransformFn {
    let mf = axis_or_fn.find("MultiplierFunction");
    let scale = mf
        .and_then(|m| m.child("scale"))
        .and_then(|s| s.floats().first().copied())
        .unwrap_or(1.0);
    let inner = mf.and_then(|m| m.child("function")).unwrap_or(axis_or_fn);
    if let Some(lin) = inner.find("LinearFunction") {
        let c = lin
            .child("coefficients")
            .map(|n| n.floats())
            .unwrap_or_default();
        // The IR's Linear is `coeff·q` (no intercept). gait2392's DOFs are [1 0];
        // a nonzero intercept would need an affine term we don't model. The
        // multiplier scales the slope (scale·coeff·q).
        assert!(
            c.len() >= 2 && c[1].abs() < 1e-9,
            "LinearFunction must be [coeff 0] (no intercept), got {c:?}"
        );
        return TransformFn::Linear {
            coeff: c[0] * scale,
        };
    }
    if let Some(ss) = inner.find("SimmSpline") {
        let x = ss.child("x").map(|n| n.floats()).unwrap_or_default();
        let y = ss.child("y").map(|n| n.floats()).unwrap_or_default();
        return TransformFn::Spline(Spline::new(x, y, scale));
    }
    if let Some(c) = inner.find("Constant") {
        let v = c
            .child("value")
            .and_then(|n| n.floats().first().copied())
            .unwrap_or(0.0);
        return TransformFn::Constant(v * scale);
    }
    // gait2392 uses only LinearFunction / SimmSpline / Constant function shapes.
    panic!(
        "<{}> has an unsupported function shape (not Linear/SimmSpline/Constant)",
        axis_or_fn.name
    );
}

/// A path-point location function as a [`Spline`] (a `MovingPathPoint` axis or a
/// constant). Reuses [`parse_transform_fn`]; a `LinearFunction` location would be
/// unexpected and is rejected.
fn parse_spline(node: &Node) -> Spline {
    match parse_transform_fn(node) {
        TransformFn::Spline(s) => s,
        TransformFn::Constant(v) => Spline::constant(v),
        TransformFn::Linear { .. } => {
            // path-point locations are SimmSpline/Constant, never a LinearFunction.
            panic!("path-point location must be a spline/constant, not a LinearFunction")
        }
    }
}

fn parse_muscle(root: &Node, name: &str) -> Muscle {
    let node = ["Millard2012EquilibriumMuscle", "Thelen2003Muscle"]
        .iter()
        .find_map(|tag| root.find_with_attr(tag, "name", name))
        // the four target muscles are always present in gait2392.
        .unwrap_or_else(|| panic!("muscle '{name}' not found"));

    let objects = node
        .find("PathPointSet")
        .and_then(|s| s.child("objects"))
        // every gait2392 muscle carries a PathPointSet/objects block.
        .unwrap_or_else(|| panic!("muscle '{name}' has no PathPointSet/objects"));

    Muscle {
        name: name.to_string(),
        path: objects
            .children
            .iter()
            .filter_map(parse_path_point)
            .collect(),
    }
}

fn parse_path_point(c: &Node) -> Option<PathPoint> {
    let name = c.attr("name").unwrap_or("").to_string();
    let body = c.child("body")?.text_trim().to_string();
    match c.name.as_str() {
        "PathPoint" => Some(PathPoint {
            name,
            body,
            location: vec3(c.child("location")?),
            kind: Kind::Fixed,
        }),
        "ConditionalPathPoint" => {
            let r = c.child("range")?.floats();
            assert!(
                r.len() >= 2,
                "ConditionalPathPoint '{name}' <range> needs 2 values, got {r:?}"
            );
            Some(PathPoint {
                name,
                body,
                location: vec3(c.child("location")?),
                kind: Kind::Conditional { lo: r[0], hi: r[1] },
            })
        }
        "MovingPathPoint" => Some(PathPoint {
            name,
            body,
            location: Vector3::zeros(),
            kind: Kind::Moving(Box::new(MovingSplines {
                x: parse_spline(c.child("x_location")?),
                y: parse_spline(c.child("y_location")?),
                z: parse_spline(c.child("z_location")?),
            })),
        }),
        _ => None,
    }
}

fn vec3(node: &Node) -> Vector3<f64> {
    let f = node.floats();
    assert!(
        f.len() >= 3,
        "expected 3 floats in <{}>, got {:?}",
        node.name,
        f
    );
    Vector3::new(f[0], f[1], f[2])
}
