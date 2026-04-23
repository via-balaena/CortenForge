//! Parameter gradient computation for field nodes.
//!
//! Computes `∂f/∂θ` — the derivative of the scalar field with respect to a
//! named design parameter — via chain rule through the expression tree.
//!
//! Extends Session 19's spatial gradient (`∇f` w.r.t. x,y,z) to
//! parameter-space gradients (`∂f/∂param_i`). Used by Session 26's
//! optimization loop for gradient-based design through simulation.

use nalgebra::Point3;

use crate::field_node::FieldNode;

impl FieldNode {
    /// Compute `∂f/∂θ` at a point, where θ is the parameter with `target_id`.
    ///
    /// Returns 0.0 if this subtree does not reference the target parameter.
    ///
    /// Chain rule: at each node, `∂f/∂θ = ∂f/∂v · ∂v/∂θ + Σ ∂f/∂child_j · ∂child_j/∂θ`,
    /// where `v` is the node's local `Val` (if any).
    #[must_use]
    // Procedural glue code; natural breakpoints are few.
    #[allow(clippy::too_many_lines, clippy::many_single_char_names)]
    pub(crate) fn param_gradient(&self, p: &Point3<f64>, target_id: usize) -> f64 {
        match self {
            // ── Leaf primitives with Val ─────────────────────────────────

            // f = |p| - radius  →  ∂f/∂radius = -1
            Self::Sphere { radius } => -radius.param_deriv(target_id),

            // ── Leaf primitives without Val ──────────────────────────────
            Self::Cuboid { .. }
            | Self::Cylinder { .. }
            | Self::Capsule { .. }
            | Self::Ellipsoid { .. }
            | Self::Torus { .. }
            | Self::Cone { .. }
            | Self::Plane { .. }
            | Self::Superellipsoid { .. }
            | Self::LogSpiral { .. }
            | Self::Gyroid { .. }
            | Self::SchwarzP { .. }
            | Self::Helix { .. }
            | Self::Pipe { .. }
            | Self::PipeSpline { .. }
            | Self::Loft { .. }
            | Self::UserFn { .. } => 0.0,

            // ── Hard booleans ───────────────────────────────────────────

            // f = min(a, b)  →  gradient follows the active child
            Self::Union(a, b) => {
                if a.evaluate(p) <= b.evaluate(p) {
                    a.param_gradient(p, target_id)
                } else {
                    b.param_gradient(p, target_id)
                }
            }

            // f = max(a, -b)
            Self::Subtract(a, b) => {
                if a.evaluate(p) >= -b.evaluate(p) {
                    a.param_gradient(p, target_id)
                } else {
                    -b.param_gradient(p, target_id)
                }
            }

            // f = max(a, b)
            Self::Intersect(a, b) => {
                if a.evaluate(p) >= b.evaluate(p) {
                    a.param_gradient(p, target_id)
                } else {
                    b.param_gradient(p, target_id)
                }
            }

            // ── Smooth booleans ─────────────────────────────────────────

            // f = va·h + vb·(1-h) - k·h·(1-h)
            // ∂f/∂θ = h·∂a/∂θ + (1-h)·∂b/∂θ - h(1-h)·∂k/∂θ
            Self::SmoothUnion(a, b, k) => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (vb - va) / kv).clamp(0.0, 1.0);
                let da = a.param_gradient(p, target_id);
                let db = b.param_gradient(p, target_id);
                let dk = k.param_deriv(target_id);
                let h1h = h * (1.0 - h);
                h1h.mul_add(-dk, h.mul_add(da, (1.0 - h) * db))
            }

            // f = -smooth_union(-a, b, k)
            // ∂f/∂θ = h·∂a/∂θ - (1-h)·∂b/∂θ + h(1-h)·∂k/∂θ
            // where h = clamp(0.5 + 0.5*(vb + va)/k, 0, 1)
            Self::SmoothSubtract(a, b, k) => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (vb + va) / kv).clamp(0.0, 1.0);
                let da = a.param_gradient(p, target_id);
                let db = b.param_gradient(p, target_id);
                let dk = k.param_deriv(target_id);
                let h1h = h * (1.0 - h);
                h1h.mul_add(dk, h.mul_add(da, -(1.0 - h) * db))
            }

            // f = -smooth_union(-a, -b, k)
            // ∂f/∂θ = h·∂a/∂θ + (1-h)·∂b/∂θ + h(1-h)·∂k/∂θ
            // where h = clamp(0.5 + 0.5*(va - vb)/k, 0, 1)
            Self::SmoothIntersect(a, b, k) => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (va - vb) / kv).clamp(0.0, 1.0);
                let da = a.param_gradient(p, target_id);
                let db = b.param_gradient(p, target_id);
                let dk = k.param_deriv(target_id);
                let h1h = h * (1.0 - h);
                h1h.mul_add(dk, h.mul_add(da, (1.0 - h) * db))
            }

            // f = m - k·ln(Σ exp(-(x_i - m)/k))
            // ∂f/∂θ = Σ w_i·∂child_i/∂θ + (∂f/∂k)·∂k/∂θ
            Self::SmoothUnionAll(children, k) => {
                if children.is_empty() {
                    return 0.0;
                }
                if children.len() == 1 {
                    return children[0].param_gradient(p, target_id);
                }

                let kv = k.eval();
                let values: Vec<f64> = children.iter().map(|c| c.evaluate(p)).collect();
                let m = values.iter().copied().fold(f64::INFINITY, f64::min);

                // Softmax weights
                let exps: Vec<f64> = values.iter().map(|&v| (-(v - m) / kv).exp()).collect();
                let sum: f64 = exps.iter().sum();
                let weights: Vec<f64> = exps.iter().map(|&e| e / sum).collect();

                // Child contributions: Σ w_i · ∂child_i/∂θ
                let mut result = 0.0;
                for (i, child) in children.iter().enumerate() {
                    let dci = child.param_gradient(p, target_id);
                    if dci != 0.0 {
                        result += weights[i] * dci;
                    }
                }

                // k-derivative: ∂f/∂k · ∂k/∂θ
                let dk = k.param_deriv(target_id);
                if dk != 0.0 {
                    // ∂f/∂k = -ln(S') + (1/k)·Σ(x_i - m)·w_i
                    let ln_sum = sum.ln();
                    let weighted_shifted: f64 = values
                        .iter()
                        .zip(weights.iter())
                        .map(|(&v, &w)| (v - m) * w)
                        .sum();
                    let dfdk = -ln_sum - weighted_shifted / kv;
                    result += dfdk * dk;
                }

                result
            }

            // Variable k from closure — no Val for k, just child passthrough
            Self::SmoothUnionVariable {
                a, b, radius_fn, ..
            } => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = (radius_fn.0)(*p).max(1e-15);
                let h = (0.5 + 0.5 * (vb - va) / kv).clamp(0.0, 1.0);
                let da = a.param_gradient(p, target_id);
                let db = b.param_gradient(p, target_id);
                h * da + (1.0 - h) * db
            }

            // ── Transforms ──────────────────────────────────────────────
            Self::Translate(child, offset) => {
                let q = Point3::new(p.x - offset.x, p.y - offset.y, p.z - offset.z);
                child.param_gradient(&q, target_id)
            }

            Self::Rotate(child, rot) => {
                let q = rot.inverse_transform_point(p);
                child.param_gradient(&q, target_id)
            }

            // f = s · child(p/s)  →  ∂f/∂θ = s · ∂child/∂θ(p/s)
            Self::ScaleUniform(child, s) => {
                let inv_s = 1.0 / *s;
                let q = Point3::new(p.x * inv_s, p.y * inv_s, p.z * inv_s);
                *s * child.param_gradient(&q, target_id)
            }

            Self::Mirror(child, normal) => {
                let d = p.coords.dot(normal).min(0.0);
                let two_d = 2.0 * d;
                let q = Point3::new(
                    two_d.mul_add(-normal.x, p.x),
                    two_d.mul_add(-normal.y, p.y),
                    two_d.mul_add(-normal.z, p.z),
                );
                child.param_gradient(&q, target_id)
            }

            // ── Domain operations ───────────────────────────────────────

            // f = |child(p)| - thickness  →  ∂f/∂θ = sign(c)·∂child/∂θ - ∂thickness/∂θ
            Self::Shell(child, thickness) => {
                let c = child.evaluate(p);
                let sign = if c >= 0.0 { 1.0 } else { -1.0 };
                sign * child.param_gradient(p, target_id) - thickness.param_deriv(target_id)
            }

            // f = child(p) - radius  →  ∂f/∂θ = ∂child/∂θ - ∂radius/∂θ
            Self::Round(child, radius) => {
                child.param_gradient(p, target_id) - radius.param_deriv(target_id)
            }

            // f = child(p) - distance  →  ∂f/∂θ = ∂child/∂θ - ∂distance/∂θ
            Self::Offset(child, distance) => {
                child.param_gradient(p, target_id) - distance.param_deriv(target_id)
            }

            Self::Elongate(child, half) => {
                let q = Point3::new(
                    p.x - p.x.clamp(-half.x, half.x),
                    p.y - p.y.clamp(-half.y, half.y),
                    p.z - p.z.clamp(-half.z, half.z),
                );
                child.param_gradient(&q, target_id)
            }

            Self::Twist(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.y)), s.mul_add(p.x, c * p.y), p.z);
                child.param_gradient(&q, target_id)
            }

            Self::Bend(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.z)), p.y, s.mul_add(p.x, c * p.z));
                child.param_gradient(&q, target_id)
            }

            Self::Repeat(child, spacing) => {
                let q = Point3::new(
                    fold_repeat(p.x, spacing.x),
                    fold_repeat(p.y, spacing.y),
                    fold_repeat(p.z, spacing.z),
                );
                child.param_gradient(&q, target_id)
            }

            Self::RepeatBounded {
                child,
                spacing,
                count,
            } => {
                let q = Point3::new(
                    fold_repeat_bounded(p.x, spacing.x, count[0]),
                    fold_repeat_bounded(p.y, spacing.y, count[1]),
                    fold_repeat_bounded(p.z, spacing.z, count[2]),
                );
                child.param_gradient(&q, target_id)
            }
        }
    }
}

// ── Repeat fold helpers (duplicated from evaluate.rs — pub(crate) would
//    require refactoring; these are trivial one-liners) ──────────────────

fn fold_repeat(coord: f64, spacing: f64) -> f64 {
    spacing.mul_add(-(coord / spacing).round(), coord)
}

fn fold_repeat_bounded(coord: f64, spacing: f64, count: u32) -> f64 {
    if count <= 1 {
        return coord;
    }
    let n = f64::from(count);
    let half = (n - 1.0) * spacing * 0.5;
    let id = ((coord + half) / spacing).round().clamp(0.0, n - 1.0);
    coord - id.mul_add(spacing, -half)
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::{ParamStore, Solid};

    /// Central finite-difference oracle for parameter gradients.
    fn finite_diff(solid: &Solid, p: &Point3<f64>, name: &str, eps: f64) -> f64 {
        let orig = solid.get_param(name).unwrap_or(0.0);
        solid.set_param(name, orig + eps);
        let f_plus = solid.evaluate(p);
        solid.set_param(name, orig - eps);
        let f_minus = solid.evaluate(p);
        solid.set_param(name, orig); // restore
        (f_plus - f_minus) / (2.0 * eps)
    }

    /// Assert analytic gradient matches finite difference.
    fn assert_grad(solid: &Solid, p: &Point3<f64>, name: &str, tol: f64) {
        let analytic = solid.param_gradient(p, name);
        let numeric = finite_diff(solid, p, name, 1e-6);
        let err = (analytic - numeric).abs();
        assert!(
            err < tol,
            "param_gradient mismatch for '{name}' at {p:?}: analytic={analytic:.10}, \
             numeric={numeric:.10}, err={err:.2e}"
        );
    }

    // ── 1. Sphere radius ────────────────────────────────────────────────

    #[test]
    fn param_grad_sphere_radius() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let s = Solid::sphere_p(r);

        // ∂f/∂radius = -1 everywhere (f = |p| - r)
        for p in &[
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(3.0, 2.0, 1.0),
            Point3::new(0.0, 0.0, 7.0),
        ] {
            assert_grad(&s, p, "radius", 1e-6);
        }
    }

    // ── 2. Shell thickness ──────────────────────────────────────────────

    #[test]
    fn param_grad_shell_thickness() {
        let store = ParamStore::new();
        let t = store.add("thickness", 0.5);
        let s = Solid::sphere(5.0).shell_p(t);

        // Points outside shell, inside shell, in the wall
        for p in &[
            Point3::new(5.3, 0.0, 0.0),
            Point3::new(4.6, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ] {
            assert_grad(&s, p, "thickness", 1e-6);
        }
    }

    // ── 3. Round radius ─────────────────────────────────────────────────

    #[test]
    fn param_grad_round_radius() {
        let store = ParamStore::new();
        let r = store.add("round_r", 0.3);
        let s = Solid::cuboid(Vector3::new(2.0, 2.0, 2.0)).round_p(r);

        for p in &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.5, 1.5, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ] {
            assert_grad(&s, p, "round_r", 1e-6);
        }
    }

    // ── 4. Offset distance (two params) ─────────────────────────────────

    #[test]
    fn param_grad_offset_two_params() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let d = store.add("dist", 0.5);
        let s = Solid::sphere_p(r).offset_p(d);

        let p = Point3::new(6.0, 0.0, 0.0);
        assert_grad(&s, &p, "radius", 1e-6);
        assert_grad(&s, &p, "dist", 1e-6);
    }

    // ── 5. SmoothUnion k ────────────────────────────────────────────────

    #[test]
    fn param_grad_smooth_union_k() {
        let store = ParamStore::new();
        let k = store.add("blend_k", 2.0);

        let a = Solid::sphere(3.0).translate(Vector3::new(-2.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(2.0, 0.0, 0.0));
        let s = a.smooth_union_p(b, k);

        // Blend region center + off-center points
        for p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 0.5, 0.5),
        ] {
            assert_grad(&s, p, "blend_k", 1e-6);
        }
    }

    // ── 6. SmoothUnion child radius ─────────────────────────────────────

    #[test]
    fn param_grad_smooth_union_child_radius() {
        let store = ParamStore::new();
        let r = store.add("radius", 3.0);

        let a = Solid::sphere_p(r).translate(Vector3::new(-2.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(2.0, 0.0, 0.0));
        let s = a.smooth_union(b, 2.0);

        for p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-3.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ] {
            assert_grad(&s, p, "radius", 1e-6);
        }
    }

    // ── 7. SmoothSubtract k ─────────────────────────────────────────────

    #[test]
    fn param_grad_smooth_subtract_k() {
        let store = ParamStore::new();
        let k = store.add("blend_k", 1.0);

        let a = Solid::sphere(5.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = a.smooth_subtract_p(b, k);

        for p in &[
            Point3::new(3.5, 0.0, 0.0),
            Point3::new(4.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ] {
            assert_grad(&s, p, "blend_k", 1e-6);
        }
    }

    // ── 8. SmoothIntersect k ────────────────────────────────────────────

    #[test]
    fn param_grad_smooth_intersect_k() {
        let store = ParamStore::new();
        let k = store.add("blend_k", 1.0);

        let a = Solid::sphere(5.0);
        let b = Solid::sphere(5.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = a.smooth_intersect_p(b, k);

        for p in &[
            Point3::new(1.5, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ] {
            assert_grad(&s, p, "blend_k", 1e-6);
        }
    }

    // ── 9. SmoothUnionAll k ─────────────────────────────────────────────

    #[test]
    fn param_grad_smooth_union_all_k() {
        let store = ParamStore::new();
        let k = store.add("blend_k", 2.0);

        let spheres = vec![
            Solid::sphere(2.0).translate(Vector3::new(-3.0, 0.0, 0.0)),
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0)),
        ];
        let s = Solid::smooth_union_all_p(spheres, k);

        for p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.5, 0.0, 0.0),
            Point3::new(-1.5, 1.0, 0.0),
        ] {
            assert_grad(&s, p, "blend_k", 1e-6);
        }
    }

    // ── 10. SmoothUnionAll child radius ─────────────────────────────────

    #[test]
    fn param_grad_smooth_union_all_child() {
        let store = ParamStore::new();
        let r = store.add("radius", 2.0);

        let spheres = vec![
            Solid::sphere_p(r).translate(Vector3::new(-3.0, 0.0, 0.0)),
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0)),
        ];
        let s = Solid::smooth_union_all(spheres, 2.0);

        for p in &[
            Point3::new(-3.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-4.0, 0.0, 0.0),
        ] {
            assert_grad(&s, p, "radius", 1e-6);
        }
    }

    // ── 11. Translated sphere ───────────────────────────────────────────

    #[test]
    fn param_grad_translated() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let s = Solid::sphere_p(r).translate(Vector3::new(10.0, 0.0, 0.0));

        assert_grad(&s, &Point3::new(15.0, 0.0, 0.0), "radius", 1e-6);
        assert_grad(&s, &Point3::new(12.0, 1.0, 0.0), "radius", 1e-6);
    }

    // ── 12. Rotated sphere ──────────────────────────────────────────────

    #[test]
    fn param_grad_rotated() {
        use nalgebra::UnitQuaternion;

        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let rot = UnitQuaternion::from_euler_angles(0.3, 0.5, 0.7);
        let s = Solid::sphere_p(r).rotate(rot);

        assert_grad(&s, &Point3::new(3.0, 2.0, 1.0), "radius", 1e-6);
        assert_grad(&s, &Point3::new(0.0, 5.0, 0.0), "radius", 1e-6);
    }

    // ── 13. Scaled sphere ───────────────────────────────────────────────

    #[test]
    fn param_grad_scaled() {
        let store = ParamStore::new();
        let r = store.add("radius", 3.0);
        let s = Solid::sphere_p(r).scale_uniform(2.0);

        // f = 2 * (|p/2| - r), so ∂f/∂r = -2
        assert_grad(&s, &Point3::new(6.0, 0.0, 0.0), "radius", 1e-6);
        assert_grad(&s, &Point3::new(4.0, 2.0, 0.0), "radius", 1e-6);
    }

    // ── 14. Composed tree (3 params) ────────────────────────────────────

    #[test]
    fn param_grad_composed() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let k = store.add("blend_k", 1.0);
        let t = store.add("thickness", 0.3);

        let body = Solid::sphere_p(r);
        let hole = Solid::cuboid(Vector3::new(2.0, 2.0, 2.0));
        let blended = body.smooth_union_p(hole, k);
        let s = blended.shell_p(t);

        // Avoid points where blended.evaluate == 0 (Shell's |·| kink)
        let p = Point3::new(5.5, 0.0, 0.0); // outside shell (blended > 0)
        assert_grad(&s, &p, "radius", 1e-6);
        assert_grad(&s, &p, "blend_k", 1e-6);
        assert_grad(&s, &p, "thickness", 1e-6);

        let p2 = Point3::new(4.5, 0.0, 0.0); // inside shell (blended < 0)
        assert_grad(&s, &p2, "radius", 1e-6);
        assert_grad(&s, &p2, "blend_k", 1e-6);
        assert_grad(&s, &p2, "thickness", 1e-6);
    }

    // ── 15. Literal sphere → zero gradient ──────────────────────────────

    #[test]
    fn param_grad_literal_zero() {
        let s = Solid::sphere(5.0);
        let grads = s.param_gradients(&Point3::new(3.0, 0.0, 0.0));
        assert!(
            grads.is_empty(),
            "literal solid should have no param gradients"
        );
    }

    // ── 16. param_gradients returns all ─────────────────────────────────

    #[test]
    fn param_gradients_all() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let k = store.add("blend_k", 2.0);

        let body = Solid::sphere_p(r);
        let other = Solid::sphere(3.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = body.smooth_union_p(other, k);

        let p = Point3::new(1.0, 0.0, 0.0);
        let grads = s.param_gradients(&p);
        assert_eq!(grads.len(), 2);

        // Verify each matches the per-param version
        for (name, val) in &grads {
            let single = s.param_gradient(&p, name);
            assert!(
                (val - single).abs() < 1e-12,
                "param_gradients vs param_gradient mismatch for '{name}': {val} vs {single}"
            );
        }

        // Also verify against finite differences
        for (name, _) in &grads {
            assert_grad(&s, &p, name, 1e-6);
        }
    }

    // ── 17. Union selects active child ──────────────────────────────────

    #[test]
    fn param_grad_union_select() {
        let store = ParamStore::new();
        let r = store.add("radius", 3.0);

        // Param sphere at origin, literal sphere at x=10 (no overlap)
        let a = Solid::sphere_p(r);
        let b = Solid::sphere(3.0).translate(Vector3::new(10.0, 0.0, 0.0));
        let s = a.union(b);

        // Near sphere a — a is active, gradient = -1
        let p = Point3::new(3.0, 0.0, 0.0);
        assert_grad(&s, &p, "radius", 1e-6);

        // Near sphere b — b is active, gradient = 0 (no param)
        let p2 = Point3::new(10.0, 0.0, 0.0);
        let g = s.param_gradient(&p2, "radius");
        assert!(
            g.abs() < 1e-12,
            "gradient near literal child should be 0, got {g}"
        );
    }

    // ── 18. Twist passthrough ─────────────────────────────────────────

    #[test]
    fn param_grad_twist_passthrough() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let s = Solid::sphere_p(r).twist(0.1);

        // Point away from twist axis where coordinate transform matters.
        for p in &[
            Point3::new(3.0, 2.0, 5.0),
            Point3::new(0.0, 4.0, 3.0),
            Point3::new(-2.0, 1.0, -4.0),
        ] {
            assert_grad(&s, p, "radius", 1e-5);
        }
    }

    // ── 19. Elongate passthrough ──────────────────────────────────────

    #[test]
    fn param_grad_elongate_passthrough() {
        let store = ParamStore::new();
        let r = store.add("radius", 3.0);
        let s = Solid::sphere_p(r).elongate(Vector3::new(2.0, 1.0, 0.0));

        for p in &[
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(2.0, 1.0, 2.0),
        ] {
            assert_grad(&s, p, "radius", 1e-5);
        }
    }

    // ── 20. Bend passthrough ──────────────────────────────────────────

    #[test]
    fn param_grad_bend_passthrough() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        let s = Solid::sphere_p(r).bend(0.05);

        for p in &[
            Point3::new(3.0, 0.0, 4.0),
            Point3::new(0.0, 3.0, -2.0),
            Point3::new(-2.0, 1.0, 5.0),
        ] {
            assert_grad(&s, p, "radius", 1e-5);
        }
    }

    // ── 21. Repeat passthrough ────────────────────────────────────────

    #[test]
    fn param_grad_repeat_passthrough() {
        let store = ParamStore::new();
        let r = store.add("radius", 2.0);
        let s = Solid::sphere_p(r).repeat(Vector3::new(8.0, 8.0, 8.0));

        // Evaluate near the central copy (within one period).
        for p in &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(-1.5, 0.5, 0.0),
        ] {
            assert_grad(&s, p, "radius", 1e-5);
        }
    }

    // ── 22. Subtract ────────────────────────────────────────────────────

    #[test]
    fn param_grad_subtract() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);

        let body = Solid::sphere_p(r);
        let hole = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = body.subtract(hole);

        // Avoid points where a.evaluate == -b.evaluate (non-differentiable boundary)
        for p in &[
            Point3::new(0.0, 5.0, 0.0),  // on body surface, far from hole
            Point3::new(-5.0, 0.0, 0.0), // on body surface, far from hole
            Point3::new(0.0, 3.0, 0.0),  // inside body, far from hole
        ] {
            assert_grad(&s, p, "radius", 1e-6);
        }
    }
}
