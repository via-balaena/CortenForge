//! SPIKE (throwaway, `#[ignore]`) — A1: a GENERAL kinematic-tree FK reproduces the
//! bespoke knee oracle.
//! Run: cargo test -p cf-osim --test spike_general_fk -- --ignored --nocapture
//!
//! De-risks the deferred IR generalization: turning cf-osim's hardcoded knee
//! `Subgraph` (+ bespoke `Kinematics::femur()/tibia()`) into a source-agnostic
//! kinematic tree — a `Body` tree where each `Joint` is a list of CustomJoint
//! transform axes (rotation/translation about an axis, driven by a function of a
//! coordinate). The risk is the FK *convention*: does composing transform axes and
//! walking the parent chain reproduce the validated knee moment arms exactly?
//!
//! Rests on the gait2392 cold-read: every joint is a `CustomJoint` whose axes use
//! Linear (a DOF) / Constant / SimmSpline (coupled) functions, and the model is
//! WRAP-FREE (0 wrap geoms) — so one general `Joint` + straight-segment paths
//! cover the whole leg. Here the general tree is built to represent the *knee*
//! chain (pelvis → femur[hip welded at neutral] → tibia[knee coupled]); its moment
//! arms must equal `oracle::Kinematics` (Variant::TRUTH) to ~machine precision.

use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::{Muscle, Spline, Subgraph, parse_knee_subgraph};
use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3};
use std::collections::HashMap;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

// --- prototype of the general IR (throwaway) -------------------------------

/// A CustomJoint transform-axis function of a single coordinate.
enum Func {
    Linear {
        coeff: f64,
    },
    #[allow(dead_code)]
    Constant(f64),
    Spline(Spline),
}
impl Func {
    fn eval(&self, q: f64) -> f64 {
        match self {
            Func::Linear { coeff } => coeff * q,
            Func::Constant(v) => *v,
            Func::Spline(s) => s.eval(q),
        }
    }
}

/// One transform axis: a rotation about / translation along `axis`, by `func` of
/// the coordinate named `coord`.
struct Axis {
    rotation: bool,
    axis: Vector3<f64>,
    coord: String,
    func: Func,
}

/// A body in the kinematic tree: its parent, the fixed offset to the joint frame
/// (`location_in_parent`), and the joint's transform axes.
struct Body {
    name: String,
    parent: Option<usize>,
    loc_in_parent: Vector3<f64>,
    axes: Vec<Axis>,
}

struct Model {
    bodies: Vec<Body>,
}

impl Model {
    fn index_of(&self, name: &str) -> usize {
        self.bodies.iter().position(|b| b.name == name).unwrap()
    }

    /// The joint transform for `body` at coordinates `q`: compose the rotation
    /// axes (product) and the translation axes (sum), then `x -> R·x + t`. This is
    /// the OpenSim CustomJoint convention the knee oracle was validated against
    /// (`from_parts(translation, rotation)`). Rotation-order only matters away from
    /// neutral; the knee chain holds the multi-DOF hip at neutral.
    fn joint_xform(&self, body: &Body, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let mut rot = UnitQuaternion::identity();
        let mut trans = Vector3::zeros();
        for ax in &body.axes {
            let val = ax.func.eval(q.get(&ax.coord).copied().unwrap_or(0.0));
            if ax.rotation {
                rot *= UnitQuaternion::from_axis_angle(&Unit::new_normalize(ax.axis), val);
            } else {
                trans += ax.axis * val;
            }
        }
        Isometry3::from_parts(Translation3::from(trans), rot)
    }

    /// World pose of `body_idx` at coordinates `q` (walk the parent chain).
    fn world_pose(&self, body_idx: usize, q: &HashMap<String, f64>) -> Isometry3<f64> {
        let b = &self.bodies[body_idx];
        let parent = match b.parent {
            Some(p) => self.world_pose(p, q),
            None => Isometry3::identity(),
        };
        parent * Translation3::from(b.loc_in_parent) * self.joint_xform(b, q)
    }

    fn body_pose(&self, name: &str, q: &HashMap<String, f64>) -> Isometry3<f64> {
        match name {
            "pelvis" | "ground" => Isometry3::identity(),
            other => self.world_pose(self.index_of(other), q),
        }
    }

    /// Muscle path length at knee angle `theta` (truth kinematics): sum of straight
    /// segments between active path points placed by the general FK.
    fn path_length(&self, m: &Muscle, theta: f64) -> f64 {
        let q = HashMap::from([("knee_angle_r".to_string(), theta)]);
        let pts: Vec<Point3<f64>> = m
            .path
            .iter()
            .filter(|p| p.active(theta))
            .map(|p| self.body_pose(&p.body, &q) * Point3::from(p.location_at(theta, false)))
            .collect();
        pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
    }

    fn moment_arm(&self, m: &Muscle, theta: f64, eps: f64) -> f64 {
        -(self.path_length(m, theta + eps) - self.path_length(m, theta - eps)) / (2.0 * eps)
    }
}

/// Build the general tree that REPRESENTS the knee chain, from the parsed subgraph:
/// pelvis (root) → femur_r (hip held at neutral, just the `hip_in_pelvis` offset)
/// → tibia_r (the coupled knee: a flexion rotation + the three spline translations).
fn knee_chain_model(sub: &Subgraph) -> Model {
    let k = &sub.knee;
    Model {
        bodies: vec![
            Body {
                name: "pelvis".into(),
                parent: None,
                loc_in_parent: Vector3::zeros(),
                axes: vec![],
            },
            Body {
                name: "femur_r".into(),
                parent: Some(0),
                loc_in_parent: sub.hip_in_pelvis,
                axes: vec![], // hip welded at neutral for the knee study
            },
            Body {
                name: "tibia_r".into(),
                parent: Some(1),
                loc_in_parent: Vector3::zeros(),
                axes: vec![
                    Axis {
                        rotation: true,
                        axis: k.flexion_axis,
                        coord: "knee_angle_r".into(),
                        func: Func::Linear { coeff: 1.0 },
                    },
                    Axis {
                        rotation: false,
                        axis: Vector3::x(),
                        coord: "knee_angle_r".into(),
                        func: Func::Spline(k.tx.clone()),
                    },
                    Axis {
                        rotation: false,
                        axis: Vector3::y(),
                        coord: "knee_angle_r".into(),
                        func: Func::Spline(k.ty.clone()),
                    },
                    Axis {
                        rotation: false,
                        axis: Vector3::z(),
                        coord: "knee_angle_r".into(),
                        func: Func::Spline(k.tz.clone()),
                    },
                ],
            },
        ],
    }
}

fn osim_path() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    )
}

#[test]
#[ignore = "spike — run with --ignored --nocapture"]
fn general_fk_reproduces_knee_oracle() {
    let sub = parse_knee_subgraph(&std::fs::read_to_string(osim_path()).expect("read osim"));
    let oracle = Kinematics::new(&sub);
    let model = knee_chain_model(&sub);

    let angles: Vec<f64> = (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect();
    let eps = 0.5 * DEG;

    println!("\n===== SPIKE A1 — GENERAL TREE-FK vs KNEE ORACLE =====");
    println!(
        "{:<11} {:>16} {:>16}",
        "muscle", "max |Δ MA| (mm)", "max |Δ L| (mm)"
    );
    println!("{:-<46}", "");

    let mut worst = 0.0_f64;
    for m in &sub.muscles {
        let mut max_dma = 0.0_f64;
        let mut max_dl = 0.0_f64;
        for &th in &angles {
            let dma = (model.moment_arm(m, th, eps)
                - oracle.moment_arm(m, th, eps, Variant::TRUTH))
            .abs()
                * 1000.0;
            let dl = (model.path_length(m, th) - oracle.path_length(m, th, Variant::TRUTH)).abs()
                * 1000.0;
            max_dma = max_dma.max(dma);
            max_dl = max_dl.max(dl);
        }
        println!("{:<11} {max_dma:>16.2e} {max_dl:>16.2e}", m.name);
        worst = worst.max(max_dma);

        // The general FK is the SAME math as the oracle, routed through a tree
        // walker + transform-axis composition — so they must agree to ~machine eps.
        assert!(
            max_dma < 1e-9,
            "{}: general FK moment arm diverges from the oracle by {max_dma:.2e} mm",
            m.name
        );
    }

    println!("{:-<46}", "");
    println!("worst |Δ moment arm| across all muscles/angles: {worst:.2e} mm");
    println!("→ a general kinematic-tree FK (Body tree + CustomJoint transform axes)");
    println!("  reproduces the validated knee oracle exactly. The IR generalization");
    println!("  is sound; remaining A1 work is the parser + emit, then unweld the hip.");
    println!("=====================================================\n");
}
