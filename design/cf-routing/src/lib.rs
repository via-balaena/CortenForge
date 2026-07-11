//! Differentiable centerline routing.
//!
//! A route is one **centerline** — a differentiable curve through R³ — that can
//! be *realized* against matter in several ways: subtracted from a solid (a
//! bore or conduit), raised as a rib, or traced as a toolpath. This crate owns
//! the shared centerline ([`Path`]) and the realizations that consume it.
//!
//! The dependency is strictly one-way: **realizations consume a [`Path`]; a
//! [`Path`] never knows its realizations.** That keeps "one centerline, many
//! realizations" a structural fact — the geometry has a single source (the
//! [`CatmullRomCurve`] a `Path` wraps), so a route
//! and any realization of it can never describe *different* curves.
//!
//! # Scope
//!
//! Consumer-gated: the surface is exactly what today's callers need — a `Path`
//! (`sample`/`tangent`/`control_points`) and the subtractive [`bore`]. Length,
//! moment-arm gradients, parallel-transport frames, and route optimization are
//! added when a realization pulls them in, not before.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_design::Solid;
use cf_geometry::CatmullRomCurve;
use nalgebra::{Point3, Vector3};

/// A differentiable route: a centerline in R³ that realizations are cut, raised,
/// or traced along.
///
/// A `Path` is the routing-domain object built on a
/// [`CatmullRomCurve`]; it delegates all curve
/// geometry to that single source. Construct it from at least two control
/// points.
///
/// ```
/// use cf_routing::Path;
/// use nalgebra::Point3;
///
/// let path = Path::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ])
/// .expect("two or more control points");
///
/// // The route passes through its control points at the span boundaries.
/// assert!((path.sample(0.5) - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-12);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    curve: CatmullRomCurve,
}

impl Path {
    /// Creates a route through `control_points`.
    ///
    /// Returns [`None`] when fewer than two control points are supplied, or when
    /// any coordinate is non-finite (see [`CatmullRomCurve::new`]). A `Path`
    /// therefore always has a valid, finite centerline.
    #[must_use]
    pub fn new(control_points: Vec<Point3<f64>>) -> Option<Self> {
        CatmullRomCurve::new(control_points).map(|curve| Self { curve })
    }

    /// The control points the route interpolates.
    #[must_use]
    pub fn control_points(&self) -> &[Point3<f64>] {
        self.curve.control_points()
    }

    /// Position on the route at global parameter `t ∈ [0, 1]`.
    ///
    /// `t` is clamped to `[0, 1]`; `t = 0` is the start, `t = 1` the end.
    #[must_use]
    pub fn sample(&self, t: f64) -> Point3<f64> {
        self.curve.sample(t)
    }

    /// Tangent `dC/dt` of the route at global parameter `t ∈ [0, 1]`.
    ///
    /// Taken with respect to the global parameter (consistent with a finite
    /// difference of [`sample`](Self::sample)); not normalized.
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vector3<f64> {
        self.curve.tangent(t)
    }
}

/// Bores a constant-radius tube along `path` out of `target`, returning the
/// carved solid.
///
/// This is the *subtractive* realization of a route: a radially symmetric tube
/// swept along the centerline and removed from `target` via CSG subtraction.
/// The result's signed-distance field is differentiable with respect to the
/// path's control points, so a route and its bore can be optimized together.
///
/// # Panics
///
/// Panics if `radius` is not positive and finite (the tube is built by
/// [`Solid::pipe_spline`], which enforces this).
#[must_use]
pub fn bore(target: Solid, path: &Path, radius: f64) -> Solid {
    let tube = Solid::pipe_spline(path.control_points().to_vec(), radius);
    target.subtract(tube)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use cf_geometry::CatmullRomCurve;
    use mesh_repair::{MeshAdjacency, validate_mesh};
    use nalgebra::Vector3;

    /// A gentle arc entering and leaving a box along +x, so a bore through it is
    /// a clean through-hole (endpoints outside the box on both x-faces).
    fn through_arc() -> Vec<Point3<f64>> {
        vec![
            Point3::new(-7.0, 0.0, 0.0),
            Point3::new(-2.0, 1.0, 0.0),
            Point3::new(2.0, -1.0, 0.0),
            Point3::new(7.0, 0.0, 0.0),
        ]
    }

    fn box_target() -> Solid {
        Solid::cuboid(Vector3::new(5.0, 5.0, 5.0))
    }

    #[test]
    fn path_rejects_invalid_control_points() {
        assert!(Path::new(vec![]).is_none());
        assert!(Path::new(vec![Point3::origin()]).is_none());
        // Non-finite coordinates are rejected too (inherited from CatmullRomCurve).
        assert!(Path::new(vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::origin()]).is_none());
        assert!(Path::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)]).is_some());
    }

    #[test]
    fn path_delegates_to_its_catmull_rom_curve() {
        // A `Path` is a pure routing-domain wrapper; its job is to wire through
        // to the one underlying curve. cf-geometry already proves the curve is
        // correct, so here we assert only the delegation is exact.
        let cps = through_arc();
        let path = Path::new(cps.clone()).unwrap();
        let curve = CatmullRomCurve::new(cps).unwrap();
        assert_eq!(path.control_points(), curve.control_points());
        for &t in &[0.0, 0.13, 0.37, 0.5, 0.71, 1.0] {
            assert_eq!(path.sample(t), curve.sample(t));
            assert_eq!(path.tangent(t), curve.tangent(t));
        }
    }

    #[test]
    fn bore_through_box_is_a_watertight_through_hole() {
        let path = Path::new(through_arc()).unwrap();
        let mesh = bore(box_target(), &path, 1.0).mesh(0.25);
        let report = validate_mesh(&mesh.geometry);
        assert!(
            report.is_watertight,
            "bored solid not watertight: {} boundary edges",
            report.boundary_edge_count
        );
        assert!(
            report.is_manifold,
            "bored solid not manifold: {} non-manifold edges",
            report.non_manifold_edge_count
        );
        // A closed manifold box has Euler characteristic V-E+F = 2 (genus 0);
        // a single through-hole makes it a solid torus (genus 1, χ = 0). This
        // distinguishes a real through-bore from a blind hole or an unpunched
        // solid, which marching cubes would also mesh watertight. χ = 0 is
        // exactly V + F = E (kept unsigned; genus 0 gives V+F = E+2, genus 2
        // gives V+F = E-2).
        let v = mesh.geometry.vertices.len();
        let f = mesh.geometry.faces.len();
        let e = MeshAdjacency::build(&mesh.geometry.faces).edge_count();
        assert_eq!(v + f, e, "expected a through-hole (χ=0): V={v} E={e} F={f}");
    }

    #[test]
    fn bore_removes_material_along_the_path() {
        // A point on the centerline, deep inside the box, must be carved out
        // (outside the remaining solid → positive field) after boring.
        let path = Path::new(through_arc()).unwrap();
        let on_axis = path.sample(0.5);
        assert!(
            box_target().evaluate(&on_axis) < 0.0,
            "probe not inside box"
        );
        let bored = bore(box_target(), &path, 1.0);
        assert!(
            bored.evaluate(&on_axis) > 0.0,
            "centerline point still inside the bored solid (field {})",
            bored.evaluate(&on_axis)
        );
    }

    #[test]
    fn bore_field_is_differentiable_wrt_control_point() {
        // Finite-difference the bored field at a fixed probe near the bore wall
        // w.r.t. an interior control point's y. A well-conditioned, genuinely
        // differentiable route gives a flat plateau across shrinking eps.
        let cps = through_arc();
        // Probe inside the box and inside the tube near mid-span (pipe-dominant,
        // clear of the CSG max() crease).
        let probe = Point3::new(0.0, 0.7, 0.0);

        let field_at = |dy: f64| -> f64 {
            let mut perturbed = cps.clone();
            perturbed[1].y += dy; // interior control point
            let path = Path::new(perturbed).unwrap();
            bore(box_target(), &path, 1.0).evaluate(&probe)
        };
        let fd = |eps: f64| -> f64 { (field_at(eps) - field_at(-eps)) / (2.0 * eps) };

        let d3 = fd(1e-3);
        let d4 = fd(1e-4);
        let d5 = fd(1e-5);
        // Non-trivial gradient (the route really moves the field here)...
        assert!(d4.abs() > 1e-3, "gradient vanishes: {d4}");
        // ...and a flat plateau across eps (well-conditioned, no crease blowup).
        assert_abs_diff_eq!(d3, d4, epsilon = 1e-2 * d4.abs().max(1e-6));
        assert_abs_diff_eq!(d5, d4, epsilon = 1e-2 * d4.abs().max(1e-6));
    }
}
