//! Targeted reader for the gait2392 **knee subgraph** — just what the S0 spike
//! needs: the `knee_r` joint kinematics, the femur (hip) placement, and the
//! four crossing muscles' path geometry. Not a general OpenSim importer.
//!
//! `.osim` version: OpenSim 3.x (`Version="30000"`), where each `Body` owns the
//! `Joint` that connects it to its parent.

use crate::xml::{self, Node};
use nalgebra::Vector3;

/// The four muscles the S0 spike converts + validates: two extensors whose
/// effective insertion rides a `MovingPathPoint` (the patella emulation), and
/// two flexors built from plain points.
pub const TARGET_MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];

/// A 1-D function of the knee coordinate: an OpenSim `SimmSpline` (or a constant
/// as a single knot), with its `MultiplierFunction` scale folded in.
///
/// OpenSim's `SimmSpline` is a **natural cubic** spline; we reproduce that
/// (tridiagonal solve, second-derivative = 0 at the ends) because the moment arm
/// is `dL/dθ`, so the spline's *derivative* enters the result directly — linear
/// interpolation would inject a discontinuous, biased slope (~mm/rad on the
/// dominant knee couplings). Within-knot interpolation only; our flexion sweep
/// stays inside every spline's knot range, so end extrapolation is not exercised
/// (we clamp flat there). NOTE: this is a faithful re-derivation of OpenSim's
/// spline math, NOT bit-identical to OpenSim's own evaluator — the real-OpenSim
/// cross-check (S1-remaining) is still the independent anchor.
#[derive(Debug, Clone)]
pub struct Spline {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub scale: f64,
    /// Precomputed second derivatives at the knots (natural cubic).
    m: Vec<f64>,
}

impl Spline {
    /// Build a natural-cubic spline through `(x, y)` with multiplier `scale`.
    pub fn new(x: Vec<f64>, y: Vec<f64>, scale: f64) -> Self {
        assert_eq!(x.len(), y.len(), "spline x/y length mismatch");
        let m = natural_cubic_second_derivs(&x, &y);
        Spline { x, y, scale, m }
    }

    pub fn constant(v: f64) -> Self {
        Spline {
            x: vec![0.0],
            y: vec![v],
            scale: 1.0,
            m: vec![0.0],
        }
    }

    /// Evaluate at `t` (natural-cubic within range, flat clamp outside), scaled.
    pub fn eval(&self, t: f64) -> f64 {
        let (xs, ys, m) = (&self.x, &self.y, &self.m);
        let v = if xs.len() == 1 || t <= xs[0] {
            ys[0]
        } else if t >= xs[xs.len() - 1] {
            ys[ys.len() - 1]
        } else {
            let i = xs.partition_point(|&xi| xi <= t) - 1;
            let (klo, khi) = (i, i + 1);
            let h = xs[khi] - xs[klo];
            let a = (xs[khi] - t) / h;
            let b = (t - xs[klo]) / h;
            // Numerical-Recipes "splint": cubic through the bracketing knots
            // using the precomputed second derivatives.
            a * ys[klo]
                + b * ys[khi]
                + ((a * a * a - a) * m[klo] + (b * b * b - b) * m[khi]) * (h * h) / 6.0
        };
        v * self.scale
    }
}

/// Second derivatives at the knots for a natural cubic spline (y''=0 at the
/// ends), via the standard tridiagonal solve. Degenerate inputs (<3 knots)
/// return zeros, so `eval` falls back to linear/constant behavior there.
fn natural_cubic_second_derivs(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 3 {
        return vec![0.0; n];
    }
    let mut y2 = vec![0.0; n];
    let mut u = vec![0.0; n];
    for i in 1..n - 1 {
        let sig = (x[i] - x[i - 1]) / (x[i + 1] - x[i - 1]);
        let p = sig * y2[i - 1] + 2.0;
        y2[i] = (sig - 1.0) / p;
        let d = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
        u[i] = (6.0 * d / (x[i + 1] - x[i - 1]) - sig * u[i - 1]) / p;
    }
    for k in (0..n - 1).rev() {
        y2[k] = y2[k] * y2[k + 1] + u[k];
    }
    y2
}

/// The `knee_r` CustomJoint: a flexion rotation about `flexion_axis` plus the
/// coordinate-coupled translations of the child (tibia) origin in the parent
/// (femur) frame. `tx`/`ty`/`tz` are along the femur x/y/z axes respectively.
#[derive(Debug, Clone)]
pub struct KneeJoint {
    pub flexion_axis: Vector3<f64>,
    pub tx: Spline,
    pub ty: Spline,
    pub tz: Spline,
}

/// The per-axis location splines of a `MovingPathPoint` (boxed in `Kind` so the
/// enum stays small).
#[derive(Debug, Clone)]
pub struct MovingSplines {
    pub x: Spline,
    pub y: Spline,
    pub z: Spline,
}

/// How a path point is placed.
#[derive(Debug, Clone)]
pub enum Kind {
    /// Always active, fixed location on its body.
    Fixed,
    /// Active only while `knee_angle_r ∈ [lo, hi]`; fixed location when active.
    Conditional { lo: f64, hi: f64 },
    /// Always active; location is a per-axis spline of `knee_angle_r` (patella).
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
    /// Is this point part of the path at knee angle `theta`?
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

    /// Location in its body frame at knee angle `theta`. When `freeze_moving`,
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

/// The extracted knee subgraph.
#[derive(Debug, Clone)]
pub struct Subgraph {
    /// Femur origin in the pelvis frame (hip joint `location_in_parent`).
    pub hip_in_pelvis: Vector3<f64>,
    pub knee: KneeJoint,
    pub muscles: Vec<Muscle>,
}

/// Parse the knee subgraph + target muscles from a gait2392 `.osim` string.
pub fn parse_knee_subgraph(osim_xml: &str) -> Subgraph {
    let root = xml::parse(osim_xml);

    let hip = find_named(&root, "CustomJoint", "hip_r")
        .child("location_in_parent")
        .map(vec3)
        .expect("hip_r location_in_parent");

    let knee_node = find_named(&root, "CustomJoint", "knee_r");
    let spatial = knee_node
        .child("SpatialTransform")
        .expect("SpatialTransform");
    let flexion_axis = vec3(
        transform_axis(spatial, "rotation1")
            .child("axis")
            .expect("rotation1 axis"),
    );
    // The downstream math (oracle frames + emitted slide axes) assumes the knee
    // SpatialTransform is: rotation1 = θ about the flexion axis with unit gain;
    // rotation2/3 absent; translations along the canonical femur x/y/z axes.
    // gait2392 satisfies this — assert it so a structurally different CustomJoint
    // can't be silently mis-converted (it must be handled explicitly instead).
    assert_unit_linear(spatial, "rotation1");
    for rot in ["rotation2", "rotation3"] {
        assert_zero_function(spatial, rot);
    }
    assert_axis(spatial, "translation1", [1.0, 0.0, 0.0]);
    assert_axis(spatial, "translation2", [0.0, 1.0, 0.0]);
    assert_axis(spatial, "translation3", [0.0, 0.0, 1.0]);

    let knee = KneeJoint {
        flexion_axis,
        tx: spline_from(transform_axis(spatial, "translation1")),
        ty: spline_from(transform_axis(spatial, "translation2")),
        tz: spline_from(transform_axis(spatial, "translation3")),
    };

    let muscles = TARGET_MUSCLES
        .iter()
        .map(|m| parse_muscle(&root, m))
        .collect();

    Subgraph {
        hip_in_pelvis: hip,
        knee,
        muscles,
    }
}

fn parse_muscle(root: &Node, name: &str) -> Muscle {
    let node = ["Millard2012EquilibriumMuscle", "Thelen2003Muscle"]
        .iter()
        .find_map(|tag| root.find_with_attr(tag, "name", name))
        // the four G1 muscles are always present in gait2392.
        .unwrap_or_else(|| panic!("muscle '{name}' not found"));

    let objects = node
        .find("PathPointSet")
        .and_then(|s| s.child("objects"))
        // every gait2392 muscle carries a PathPointSet/objects block.
        .unwrap_or_else(|| panic!("muscle '{name}' has no PathPointSet/objects"));

    let path = objects
        .children
        .iter()
        .filter_map(parse_path_point)
        .collect();

    Muscle {
        name: name.to_string(),
        path,
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
                x: spline_from(c.child("x_location")?),
                y: spline_from(c.child("y_location")?),
                z: spline_from(c.child("z_location")?),
            })),
        }),
        _ => None,
    }
}

/// Build a `Spline` from a node wrapping a `MultiplierFunction` over either a
/// `SimmSpline` (the spline knots) or a `Constant`, folding the multiplier
/// `scale` in. Navigates the `MultiplierFunction → function → {SimmSpline|
/// Constant}` structure explicitly (rather than a broad descendant search, which
/// could grab a `scale` from the wrong nested function), and panics on any other
/// shape so a future model can't be silently mis-converted to constant-0.
fn spline_from(node: &Node) -> Spline {
    let mf = node.find("MultiplierFunction");
    let scale = mf
        .and_then(|m| m.child("scale"))
        .and_then(|s| s.floats().first().copied())
        .unwrap_or(1.0);
    // The function wrapped by the MultiplierFunction (or the node itself if there
    // is no multiplier wrapper).
    let inner = mf.and_then(|m| m.child("function")).unwrap_or(node);
    if let Some(ss) = inner.find("SimmSpline") {
        let x = ss.child("x").map(|n| n.floats()).unwrap_or_default();
        let y = ss.child("y").map(|n| n.floats()).unwrap_or_default();
        return Spline::new(x, y, scale);
    }
    if let Some(c) = inner.find("Constant") {
        let v = c
            .child("value")
            .and_then(|n| n.floats().first().copied())
            .unwrap_or(0.0);
        return Spline::constant(v * scale);
    }
    // gait2392's knee uses only SimmSpline + Constant function shapes.
    panic!(
        "spline_from: <{}> has an unsupported function shape (not SimmSpline/Constant)",
        node.name
    );
}

fn transform_axis<'a>(spatial: &'a Node, axis_name: &str) -> &'a Node {
    spatial
        // the knee CustomJoint always defines this TransformAxis.
        .find_with_attr("TransformAxis", "name", axis_name)
        .unwrap_or_else(|| panic!("TransformAxis '{axis_name}' not found"))
}

/// Assert a TransformAxis's `<axis>` equals an expected basis vector (the code
/// hardcodes which spline drives which femur axis).
fn assert_axis(spatial: &Node, name: &str, expected: [f64; 3]) {
    let a = transform_axis(spatial, name)
        .child("axis")
        .map(|n| n.floats())
        .unwrap_or_default();
    assert!(
        a.len() == 3 && (0..3).all(|i| (a[i] - expected[i]).abs() < 1e-9),
        "knee {name} axis {a:?} != expected {expected:?} — non-canonical CustomJoint not supported"
    );
}

/// Assert a TransformAxis is a `LinearFunction` with coefficients `[1, 0]`, i.e.
/// the transform value equals the coordinate (here: rotation = knee angle).
fn assert_unit_linear(spatial: &Node, name: &str) {
    let coeffs = transform_axis(spatial, name)
        .find("LinearFunction")
        .and_then(|f| f.child("coefficients"))
        .map(|n| n.floats())
        .unwrap_or_default();
    assert!(
        coeffs.len() >= 2 && (coeffs[0] - 1.0).abs() < 1e-9 && coeffs[1].abs() < 1e-9,
        "knee {name} must be LinearFunction [1 0] (transform = coordinate), got {coeffs:?}"
    );
}

/// Assert a TransformAxis's function is identically zero across the ROM.
fn assert_zero_function(spatial: &Node, name: &str) {
    let s = spline_from(transform_axis(spatial, name));
    for &th in &[0.17_f64, 0.0, -1.0, -2.094] {
        assert!(
            s.eval(th).abs() < 1e-9,
            "knee {name} must be the zero function, got {} at {th}",
            s.eval(th)
        );
    }
}

fn find_named<'a>(root: &'a Node, tag: &str, name: &str) -> &'a Node {
    root.find_with_attr(tag, "name", name)
        // callers pass element names known to exist in the vendored model.
        .unwrap_or_else(|| panic!("<{tag} name=\"{name}\"> not found"))
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

#[cfg(test)]
mod spline_tests {
    use super::Spline;

    #[test]
    fn interpolates_knots_exactly() {
        let s = Spline::new(vec![0.0, 1.0, 2.0, 3.0], vec![0.0, 1.0, 4.0, 9.0], 2.0);
        for (x, y) in [(0.0, 0.0), (1.0, 1.0), (2.0, 4.0), (3.0, 9.0)] {
            assert!(
                (s.eval(x) - y * 2.0).abs() < 1e-12,
                "knot ({x},{y}) not hit"
            );
        }
    }

    #[test]
    fn reduces_to_linear_for_collinear_data() {
        // Natural cubic through collinear points is the line itself (y''=0).
        let s = Spline::new(vec![-2.0, 0.0, 1.0, 4.0], vec![-4.0, 0.0, 2.0, 8.0], 1.0);
        for t in [-1.3, 0.25, 2.7, 3.9] {
            assert!((s.eval(t) - 2.0 * t).abs() < 1e-9, "not linear at {t}");
        }
    }

    #[test]
    fn second_derivative_is_continuous() {
        // C2: a finite-difference second derivative is smooth across an interior
        // knot (no jump like linear interpolation would show).
        let s = Spline::new(vec![0.0, 1.0, 2.0, 3.0], vec![0.0, 1.0, 0.0, 1.0], 1.0);
        let d2 = |t: f64| {
            let h = 1e-3;
            (s.eval(t + h) - 2.0 * s.eval(t) + s.eval(t - h)) / (h * h)
        };
        assert!(
            (d2(1.0 - 1e-2) - d2(1.0 + 1e-2)).abs() < 1e-1,
            "y'' jumps across knot"
        );
    }

    #[test]
    fn clamps_flat_outside_range() {
        let s = Spline::new(vec![0.0, 1.0, 2.0], vec![3.0, 5.0, 7.0], 1.0);
        assert!((s.eval(-10.0) - 3.0).abs() < 1e-12);
        assert!((s.eval(10.0) - 7.0).abs() < 1e-12);
    }

    #[test]
    fn two_knot_and_constant_degenerate() {
        // <3 knots → falls back to linear; equal endpoints → constant.
        let s = Spline::new(vec![-2.0, 0.17], vec![0.0014, 0.0014], 0.98);
        assert!((s.eval(-1.0) - 0.0014 * 0.98).abs() < 1e-12);
        let c = Spline::constant(0.42);
        assert!((c.eval(-5.0) - 0.42).abs() < 1e-12);
    }
}
