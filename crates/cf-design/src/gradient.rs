//! Analytic gradient computation for field nodes.
//!
//! Computes `∇f(p)` for every [`FieldNode`] variant. The gradient is the
//! direction of steepest ascent of the scalar field. For exact SDFs, the
//! gradient is a unit vector (the outward normal on the surface).
//!
//! Used by dual contouring (Session 20) to place vertices at the precise
//! zero-crossing with correct normals.

use nalgebra::{Point3, Vector3};

use crate::field_node::FieldNode;

impl FieldNode {
    /// Compute the analytic gradient of the scalar field at a point.
    ///
    /// For exact SDFs, returns a unit vector (the outward surface normal).
    /// For approximate SDFs (ellipsoid, superellipsoid, loft taper), the
    /// magnitude may differ from 1 but the direction is correct.
    ///
    /// Returns the zero vector at singular points (e.g., the center of a
    /// sphere) where the gradient is undefined.
    #[must_use]
    #[allow(clippy::too_many_lines, clippy::many_single_char_names)]
    pub(crate) fn gradient(&self, p: &Point3<f64>) -> Vector3<f64> {
        match self {
            // ── Primitives ─────────────────────────────────────────────
            Self::Sphere { .. } => grad_sphere(p),
            Self::Cuboid { half_extents } => grad_cuboid(half_extents, p),
            Self::Cylinder {
                radius,
                half_height,
            } => grad_cylinder(*radius, *half_height, p),
            Self::Capsule { half_height, .. } => grad_capsule(*half_height, p),
            Self::Ellipsoid { radii } => grad_ellipsoid(radii, p),
            Self::Torus { major, .. } => grad_torus(*major, p),
            Self::Cone { radius, height } => grad_cone(*radius, *height, p),
            Self::Plane { normal, .. } => *normal,

            // ── Bio-inspired primitives ────────────────────────────────
            Self::Superellipsoid { radii, n1, n2 } => grad_superellipsoid(radii, *n1, *n2, p),
            Self::LogSpiral {
                a,
                b,
                turns,
                thickness: _,
            } => grad_log_spiral(*a, *b, *turns, p),
            Self::Gyroid { scale, .. } => grad_gyroid(*scale, p),
            Self::SchwarzP { scale, .. } => grad_schwarz_p(*scale, p),
            Self::Helix {
                radius,
                pitch,
                turns,
                thickness: _,
            } => grad_helix(*radius, *pitch, *turns, p),

            // ── Path-based primitives ──────────────────────────────────
            Self::Pipe { vertices, .. } => grad_pipe(vertices, p),
            Self::PipeSpline { control_points, .. } => grad_pipe_spline(control_points, p),
            Self::Loft { stations } => grad_loft(stations, p),

            // ── Booleans ───────────────────────────────────────────────
            Self::Union(a, b) => {
                if a.evaluate(p) <= b.evaluate(p) {
                    a.gradient(p)
                } else {
                    b.gradient(p)
                }
            }
            Self::Subtract(a, b) => {
                let va = a.evaluate(p);
                let neg_vb = -b.evaluate(p);
                if va >= neg_vb {
                    a.gradient(p)
                } else {
                    -b.gradient(p)
                }
            }
            Self::Intersect(a, b) => {
                if a.evaluate(p) >= b.evaluate(p) {
                    a.gradient(p)
                } else {
                    b.gradient(p)
                }
            }
            Self::SmoothUnion(a, b, k) => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (vb - va) / kv).clamp(0.0, 1.0);
                a.gradient(p) * h + b.gradient(p) * (1.0 - h)
            }
            Self::SmoothSubtract(a, b, k) => {
                // f = -smooth_union(-a, b, k)
                // h for smooth_union(-a, b): clamp(0.5 + 0.5*(b+a)/k, 0, 1)
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (vb + va) / kv).clamp(0.0, 1.0);
                a.gradient(p) * h - b.gradient(p) * (1.0 - h)
            }
            Self::SmoothIntersect(a, b, k) => {
                // f = -smooth_union(-a, -b, k)
                // h for smooth_union(-a, -b): clamp(0.5 + 0.5*(a-b)/k, 0, 1)
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let kv = k.eval();
                let h = (0.5 + 0.5 * (va - vb) / kv).clamp(0.0, 1.0);
                a.gradient(p) * h + b.gradient(p) * (1.0 - h)
            }
            Self::SmoothUnionAll(children, k) => {
                if children.is_empty() {
                    return Vector3::zeros();
                }
                if children.len() == 1 {
                    return children[0].gradient(p);
                }
                // Log-sum-exp: weights are softmax(-x_i / k)
                let kv = k.eval();
                let values: Vec<f64> = children.iter().map(|c| c.evaluate(p)).collect();
                let m = values.iter().copied().fold(f64::INFINITY, f64::min);
                let weights: Vec<f64> = values.iter().map(|&v| (-(v - m) / kv).exp()).collect();
                let sum: f64 = weights.iter().sum();
                let mut grad = Vector3::zeros();
                for (child, &w) in children.iter().zip(weights.iter()) {
                    grad += child.gradient(p) * (w / sum);
                }
                grad
            }
            Self::SmoothUnionVariable {
                a, b, radius_fn, ..
            } => {
                let va = a.evaluate(p);
                let vb = b.evaluate(p);
                let k = (radius_fn.0)(*p).max(1e-15);
                let h = (0.5 + 0.5 * (vb - va) / k).clamp(0.0, 1.0);
                a.gradient(p) * h + b.gradient(p) * (1.0 - h)
            }

            // ── Transforms ─────────────────────────────────────────────
            Self::Translate(child, offset) => {
                let q = Point3::new(p.x - offset.x, p.y - offset.y, p.z - offset.z);
                child.gradient(&q)
            }
            Self::Rotate(child, rot) => {
                let q = rot.inverse_transform_point(p);
                let gc = child.gradient(&q);
                rot.transform_vector(&gc)
            }
            Self::ScaleUniform(child, s) => {
                // f(p) = s · child(p/s), so ∇f = ∇child(p/s)
                let inv_s = 1.0 / *s;
                let q = Point3::new(p.x * inv_s, p.y * inv_s, p.z * inv_s);
                child.gradient(&q)
            }
            Self::Mirror(child, normal) => {
                let d = p.coords.dot(normal);
                if d >= 0.0 {
                    child.gradient(p)
                } else {
                    let two_d = 2.0 * d;
                    let q = Point3::new(
                        two_d.mul_add(-normal.x, p.x),
                        two_d.mul_add(-normal.y, p.y),
                        two_d.mul_add(-normal.z, p.z),
                    );
                    let gc = child.gradient(&q);
                    // Householder reflection: gc - 2·(gc·n)·n
                    let dot = gc.dot(normal);
                    gc - *normal * (2.0 * dot)
                }
            }

            // ── Domain operations ──────────────────────────────────────
            Self::Shell(child, _) => {
                let f = child.evaluate(p);
                let gc = child.gradient(p);
                if f >= 0.0 { gc } else { -gc }
            }
            Self::Round(child, _) | Self::Offset(child, _) => child.gradient(p),
            Self::Elongate(child, half) => {
                let q = Point3::new(
                    p.x - p.x.clamp(-half.x, half.x),
                    p.y - p.y.clamp(-half.y, half.y),
                    p.z - p.z.clamp(-half.z, half.z),
                );
                let gc = child.gradient(&q);
                Vector3::new(
                    if p.x.abs() > half.x { gc.x } else { 0.0 },
                    if p.y.abs() > half.y { gc.y } else { 0.0 },
                    if p.z.abs() > half.z { gc.z } else { 0.0 },
                )
            }
            Self::Twist(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.y)), s.mul_add(p.x, c * p.y), p.z);
                let gc = child.gradient(&q);
                // ∇f = Jᵀ · gc where J = ∂q/∂p
                Vector3::new(
                    c.mul_add(gc.x, s * gc.y),
                    (-s).mul_add(gc.x, c * gc.y),
                    rate.mul_add(q.x.mul_add(gc.y, -(q.y * gc.x)), gc.z),
                )
            }
            Self::Bend(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.z)), p.y, s.mul_add(p.x, c * p.z));
                let gc = child.gradient(&q);
                // Jᵀ · gc
                Vector3::new(
                    c.mul_add(gc.x, s * gc.z),
                    gc.y,
                    (-(rate * q.z + s)).mul_add(gc.x, (rate * q.x + c) * gc.z),
                )
            }
            Self::Repeat(child, spacing) => {
                let q = Point3::new(
                    fold_repeat(p.x, spacing.x),
                    fold_repeat(p.y, spacing.y),
                    fold_repeat(p.z, spacing.z),
                );
                child.gradient(&q)
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
                child.gradient(&q)
            }

            // ── User escape hatch ──────────────────────────────────────
            Self::UserFn { eval, .. } => {
                // Finite differences fallback (opaque closure).
                let eps = 1e-6;
                let inv_2eps = 0.5 / eps;
                Vector3::new(
                    ((eval.0)(Point3::new(p.x + eps, p.y, p.z))
                        - (eval.0)(Point3::new(p.x - eps, p.y, p.z)))
                        * inv_2eps,
                    ((eval.0)(Point3::new(p.x, p.y + eps, p.z))
                        - (eval.0)(Point3::new(p.x, p.y - eps, p.z)))
                        * inv_2eps,
                    ((eval.0)(Point3::new(p.x, p.y, p.z + eps))
                        - (eval.0)(Point3::new(p.x, p.y, p.z - eps)))
                        * inv_2eps,
                )
            }
        }
    }
}

// ── Primitive gradient helpers ──────────────────────────────────────────

/// Sphere: `∇(|p| - r) = p / |p|`.
fn grad_sphere(p: &Point3<f64>) -> Vector3<f64> {
    let norm = p.coords.norm();
    if norm < 1e-15 {
        Vector3::zeros()
    } else {
        p.coords / norm
    }
}

/// Cuboid: region-based gradient (face / edge / corner).
fn grad_cuboid(half: &Vector3<f64>, p: &Point3<f64>) -> Vector3<f64> {
    let q = Vector3::new(p.x.abs() - half.x, p.y.abs() - half.y, p.z.abs() - half.z);
    let sx = p.x.signum();
    let sy = p.y.signum();
    let sz = p.z.signum();

    if q.x > 0.0 || q.y > 0.0 || q.z > 0.0 {
        // Outside: gradient of |max(q, 0)|
        let w = Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0));
        let w_norm = w.norm();
        if w_norm < 1e-15 {
            return Vector3::zeros();
        }
        Vector3::new(
            if q.x > 0.0 { w.x / w_norm * sx } else { 0.0 },
            if q.y > 0.0 { w.y / w_norm * sy } else { 0.0 },
            if q.z > 0.0 { w.z / w_norm * sz } else { 0.0 },
        )
    } else {
        // Inside: gradient of max(q_i) → normal of closest face
        if q.x >= q.y && q.x >= q.z {
            Vector3::new(sx, 0.0, 0.0)
        } else if q.y >= q.x && q.y >= q.z {
            Vector3::new(0.0, sy, 0.0)
        } else {
            Vector3::new(0.0, 0.0, sz)
        }
    }
}

/// Cylinder: barrel / cap / corner regions.
fn grad_cylinder(radius: f64, half_height: f64, p: &Point3<f64>) -> Vector3<f64> {
    let r_xy = p.x.hypot(p.y);
    let d_r = r_xy - radius;
    let d_a = p.z.abs() - half_height;

    let radial = if r_xy > 1e-15 {
        Vector3::new(p.x / r_xy, p.y / r_xy, 0.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let axial = Vector3::new(0.0, 0.0, p.z.signum());

    if d_r > 0.0 && d_a > 0.0 {
        let d = d_r.hypot(d_a);
        radial * (d_r / d) + axial * (d_a / d)
    } else if d_r > d_a {
        radial
    } else {
        axial
    }
}

/// Capsule: direction from nearest axis point to query.
fn grad_capsule(half_height: f64, p: &Point3<f64>) -> Vector3<f64> {
    let cz = p.z.clamp(-half_height, half_height);
    let diff = Vector3::new(p.x, p.y, p.z - cz);
    let d = diff.norm();
    if d < 1e-15 {
        Vector3::zeros()
    } else {
        diff / d
    }
}

/// Ellipsoid: analytic gradient of the Quilez approximate SDF.
///
/// `f = S·(S-1)/G` where `S = |s|`, `G = |g|`, `s_i = p_i/r_i`,
/// `g_i = p_i/r_i²`.
fn grad_ellipsoid(radii: &Vector3<f64>, p: &Point3<f64>) -> Vector3<f64> {
    let s = Vector3::new(p.x / radii.x, p.y / radii.y, p.z / radii.z);
    let s_norm = s.norm();
    if s_norm < 1e-12 {
        return Vector3::zeros();
    }

    let g = Vector3::new(
        p.x / (radii.x * radii.x),
        p.y / (radii.y * radii.y),
        p.z / (radii.z * radii.z),
    );
    let g_norm = g.norm();
    if g_norm < 1e-12 {
        return Vector3::zeros();
    }

    // f = S*(S-1)/G
    // df/dp_i = g_i · [(2S-1)/(S·G) - S·(S-1)/(G³·r_i²)]
    let c1 = 2.0f64.mul_add(s_norm, -1.0) / (s_norm * g_norm);
    let c2 = s_norm * (s_norm - 1.0) / (g_norm * g_norm * g_norm);

    Vector3::new(
        g.x * (c1 - c2 / (radii.x * radii.x)),
        g.y * (c1 - c2 / (radii.y * radii.y)),
        g.z * (c1 - c2 / (radii.z * radii.z)),
    )
}

/// Torus: direction from nearest point on major circle.
fn grad_torus(major: f64, p: &Point3<f64>) -> Vector3<f64> {
    let r_xy = p.x.hypot(p.y);
    let q_xy = r_xy - major;
    let d = q_xy.hypot(p.z);

    if d < 1e-15 {
        return Vector3::zeros();
    }
    if r_xy < 1e-15 {
        // On Z axis — xy gradient averages to zero by symmetry
        return Vector3::new(0.0, 0.0, p.z / d);
    }

    Vector3::new(q_xy * p.x / (d * r_xy), q_xy * p.y / (d * r_xy), p.z / d)
}

/// Cone: region-based gradient (above apex / below base / surface / inside).
#[allow(clippy::many_single_char_names)]
fn grad_cone(radius: f64, height: f64, p: &Point3<f64>) -> Vector3<f64> {
    let r = p.x.hypot(p.y);
    let cone_len = radius.hypot(height);
    let nr = height / cone_len; // outward normal radial component
    let nz = radius / cone_len; // outward normal z component

    let (rx, ry) = if r > 1e-15 {
        (p.x / r, p.y / r)
    } else {
        (1.0, 0.0)
    };

    if p.z > 0.0 {
        // Above apex: f = |p|
        let d = r.hypot(p.z);
        if d < 1e-15 {
            return Vector3::zeros();
        }
        Vector3::new(p.x / d, p.y / d, p.z / d)
    } else if p.z < -height {
        // Below base
        let d_radial = (r - radius).max(0.0);
        let d_base = -(p.z + height);

        if d_radial > 0.0 {
            let d = d_radial.hypot(d_base);
            if d < 1e-15 {
                return Vector3::zeros();
            }
            Vector3::new(d_radial / d * rx, d_radial / d * ry, -d_base / d)
        } else {
            Vector3::new(0.0, 0.0, -1.0)
        }
    } else {
        // Between apex and base
        let t = -p.z / height;
        let cone_r_at_z = radius * t;

        if r <= cone_r_at_z {
            // Inside cone
            let dot_surface = r.mul_add(nr, p.z * nz);
            let d_surf = -dot_surface;
            let d_base_inner = p.z + height;

            if d_surf <= d_base_inner {
                // Nearest to cone surface: f = dot_surface
                Vector3::new(nr * rx, nr * ry, nz)
            } else {
                // Nearest to base: f = -(z + height)
                Vector3::new(0.0, 0.0, -1.0)
            }
        } else {
            // Outside cone surface: f = dot_surface
            Vector3::new(nr * rx, nr * ry, nz)
        }
    }
}

// ── Bio-inspired gradient helpers ───────────────────────────────────────

/// Superellipsoid: partial derivatives of the implicit function.
///
/// `f = F^(n1/2) - 1` where `F = G^(n2/n1) + |z/rz|^e1`,
/// `G = |x/rx|^e2 + |y/ry|^e2`, `e1 = 2/n1`, `e2 = 2/n2`.
fn grad_superellipsoid(radii: &Vector3<f64>, n1: f64, n2: f64, p: &Point3<f64>) -> Vector3<f64> {
    let e2 = 2.0 / n2;
    let e1 = 2.0 / n1;
    let eps = 1e-15;

    let ax = (p.x / radii.x).abs().max(eps);
    let ay = (p.y / radii.y).abs().max(eps);
    let az = (p.z / radii.z).abs().max(eps);

    let g = ax.powf(e2) + ay.powf(e2);
    let f_inner = g.powf(n2 / n1) + az.powf(e1);

    if g < eps || f_inner < eps {
        return Vector3::zeros();
    }

    let f_pow = f_inner.powf(n1 / 2.0 - 1.0);
    let g_pow = g.powf(n2 / n1 - 1.0);

    let common_xy = (n1 / 2.0) * f_pow * (n2 / n1) * g_pow * e2;

    Vector3::new(
        common_xy * ax.powf(e2 - 1.0) * p.x.signum() / radii.x,
        common_xy * ay.powf(e2 - 1.0) * p.y.signum() / radii.y,
        (n1 / 2.0) * f_pow * e1 * az.powf(e1 - 1.0) * p.z.signum() / radii.z,
    )
}

/// Log spiral: direction from nearest point on spiral curve to query,
/// with correction for the discrete candidate set's dependence on `p`.
///
/// The eval function picks the nearest among candidates at `theta + 2πk`.
/// Since `theta = atan2(y, x)` depends on `p`, the candidate angles shift
/// as `p` moves, adding a correction term to the naive direction gradient.
#[allow(
    clippy::many_single_char_names,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]
fn grad_log_spiral(init_radius: f64, growth: f64, turns: f64, p: &Point3<f64>) -> Vector3<f64> {
    use std::f64::consts::TAU;

    let r_xy = p.x.hypot(p.y);
    let theta = p.y.atan2(p.x);
    let theta_max = turns * TAU;
    let z_sq = p.z * p.z;

    let k_continuous = if r_xy > 1e-15 && growth.abs() > 1e-15 {
        ((r_xy / init_radius).ln() / growth - theta) / TAU
    } else {
        0.0
    };
    let k_base = k_continuous.round() as i64;

    let mut min_dist_sq = f64::INFINITY;
    let mut best_phi = 0.0_f64;
    let mut best_clamped = false; // was the winning candidate clamped/endpoint?

    for dk in -2..=2_i64 {
        let phi_raw = TAU.mul_add((k_base + dk) as f64, theta);
        let phi = phi_raw.clamp(0.0, theta_max);
        let clamped = (phi_raw - phi).abs() > 1e-12;
        let r_s = init_radius * (growth * phi).exp();
        let dx = r_s.mul_add(phi.cos(), -p.x);
        let dy = r_s.mul_add(phi.sin(), -p.y);
        let d_sq = dy.mul_add(dy, dx.mul_add(dx, z_sq));
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
            best_phi = phi;
            best_clamped = clamped;
        }
    }
    // Endpoints (phi doesn't depend on p → no correction needed)
    for &phi in &[0.0, theta_max] {
        let r_s = init_radius * (growth * phi).exp();
        let dx = r_s.mul_add(phi.cos(), -p.x);
        let dy = r_s.mul_add(phi.sin(), -p.y);
        let d_sq = dy.mul_add(dy, dx.mul_add(dx, z_sq));
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
            best_phi = phi;
            best_clamped = true; // endpoint → no correction
        }
    }

    let dist = min_dist_sq.sqrt();
    if dist < 1e-15 {
        return Vector3::zeros();
    }

    let r_s = init_radius * (growth * best_phi).exp();
    let sx = r_s * best_phi.cos();
    let sy = r_s * best_phi.sin();

    // Base gradient: direction from nearest spiral point to query
    let mut gx = (p.x - sx) / dist;
    let mut gy = (p.y - sy) / dist;
    let gz = p.z / dist;

    // Correction for unclamped candidates: phi depends on theta(p)
    // dphi/dp_x = dtheta/dp_x = -p_y / r_xy², dphi/dp_y = p_x / r_xy²
    // Correction_i = -(diff · S'(phi)) / dist · dphi/dp_i
    if !best_clamped && r_xy > 1e-15 {
        // S'(phi) = a·exp(b·phi) · (b·cos(phi) - sin(phi), b·sin(phi) + cos(phi))
        let s_prime_x = r_s * growth.mul_add(best_phi.cos(), -best_phi.sin());
        let s_prime_y = r_s * growth.mul_add(best_phi.sin(), best_phi.cos());

        let diff_x = p.x - sx;
        let diff_y = p.y - sy;
        let dot_diff_sprime = diff_x.mul_add(s_prime_x, diff_y * s_prime_y);
        let correction_scale = -dot_diff_sprime / dist;

        let r_xy_sq = r_xy * r_xy;
        gx += correction_scale * (-p.y / r_xy_sq);
        gy += correction_scale * (p.x / r_xy_sq);
    }

    Vector3::new(gx, gy, gz)
}

/// Gyroid: `sign(g) · ∇g` where `g = sin·cos + sin·cos + sin·cos`.
fn grad_gyroid(scale: f64, p: &Point3<f64>) -> Vector3<f64> {
    let sx = (scale * p.x).sin();
    let sy = (scale * p.y).sin();
    let sz = (scale * p.z).sin();
    let cx = (scale * p.x).cos();
    let cy = (scale * p.y).cos();
    let cz = (scale * p.z).cos();

    let g = sz.mul_add(cx, sx.mul_add(cy, sy * cz));
    let sign = if g >= 0.0 { 1.0 } else { -1.0 };

    let s_sign = sign * scale;
    Vector3::new(
        s_sign * cx.mul_add(cy, -(sz * sx)),
        s_sign * cy.mul_add(cz, -(sx * sy)),
        s_sign * cz.mul_add(cx, -(sy * sz)),
    )
}

/// Schwarz P: `sign(g) · ∇g` where `g = cos(sx) + cos(sy) + cos(sz)`.
fn grad_schwarz_p(scale: f64, p: &Point3<f64>) -> Vector3<f64> {
    let g = (scale * p.x).cos() + (scale * p.y).cos() + (scale * p.z).cos();
    let sign = if g >= 0.0 { 1.0 } else { -1.0 };

    Vector3::new(
        sign * (-scale * (scale * p.x).sin()),
        sign * (-scale * (scale * p.y).sin()),
        sign * (-scale * (scale * p.z).sin()),
    )
}

/// Helix: direction from nearest point on helix curve to query.
#[allow(
    clippy::many_single_char_names,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]
fn grad_helix(radius: f64, pitch: f64, turns: f64, p: &Point3<f64>) -> Vector3<f64> {
    use std::f64::consts::TAU;

    let theta = p.y.atan2(p.x);
    let tau_sq = TAU * TAU;

    let k_continuous = if pitch.abs() > 1e-15 {
        (p.z / pitch) - theta / TAU
    } else {
        0.0
    };
    let k_base = k_continuous.round() as i64;

    let mut min_d_sq = f64::INFINITY;
    let mut best_t = 0.0_f64;

    for dk in -1..=1_i64 {
        let t_initial = (theta / TAU + (k_base + dk) as f64).clamp(0.0, turns);

        let mut t = t_initial;
        for _ in 0..4 {
            let alpha = TAU * t;
            let (sa, ca) = alpha.sin_cos();

            let ex = radius.mul_add(ca, -p.x);
            let ey = radius.mul_add(sa, -p.y);
            let ez = pitch.mul_add(t, -p.z);

            let vx = -radius * TAU * sa;
            let vy = radius * TAU * ca;
            let vz = pitch;

            let fp = 2.0 * ez.mul_add(vz, ex.mul_add(vx, ey * vy));
            let vel_sq = vz.mul_add(vz, vx.mul_add(vx, vy * vy));
            let acc_x = -radius * tau_sq * ca;
            let acc_y = -radius * tau_sq * sa;
            let diff_dot_acc = ex.mul_add(acc_x, ey * acc_y);
            let fpp = 2.0 * (vel_sq + diff_dot_acc);

            if fpp.abs() < 1e-20 {
                break;
            }
            t -= fp / fpp;
            t = t.clamp(0.0, turns);
        }

        let alpha = TAU * t;
        let ex = radius.mul_add(-alpha.cos(), p.x);
        let ey = radius.mul_add(-alpha.sin(), p.y);
        let ez = pitch.mul_add(-t, p.z);
        let d_sq = ez.mul_add(ez, ex.mul_add(ex, ey * ey));
        if d_sq < min_d_sq {
            min_d_sq = d_sq;
            best_t = t;
        }
    }

    let dist = min_d_sq.sqrt();
    if dist < 1e-15 {
        return Vector3::zeros();
    }

    let alpha = TAU * best_t;
    Vector3::new(
        radius.mul_add(-alpha.cos(), p.x) / dist,
        radius.mul_add(-alpha.sin(), p.y) / dist,
        pitch.mul_add(-best_t, p.z) / dist,
    )
}

// ── Path-based gradient helpers ─────────────────────────────────────────

/// Pipe: direction from nearest point on polyline to query.
fn grad_pipe(vertices: &[Point3<f64>], p: &Point3<f64>) -> Vector3<f64> {
    let mut min_dist_sq = f64::INFINITY;
    let mut nearest = Point3::origin();

    for seg in vertices.windows(2) {
        let closest = cf_geometry::closest_point_segment(seg[0], seg[1], *p);
        let d_sq = nalgebra::distance_squared(p, &closest);
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
            nearest = closest;
        }
    }

    let dist = min_dist_sq.sqrt();
    if dist < 1e-15 {
        return Vector3::zeros();
    }
    (p - nearest) / dist
}

/// Pipe spline: direction from nearest point on Catmull-Rom spline to query.
fn grad_pipe_spline(control_points: &[Point3<f64>], p: &Point3<f64>) -> Vector3<f64> {
    let n = control_points.len();

    if n == 2 {
        let closest = cf_geometry::closest_point_segment(control_points[0], control_points[1], *p);
        let d = nalgebra::distance(p, &closest);
        if d < 1e-15 {
            return Vector3::zeros();
        }
        return (p - closest) / d;
    }

    let mut min_dist_sq = f64::INFINITY;
    let mut nearest = Point3::origin();

    let num_spans = n - 1;
    for span in 0..num_spans {
        let p0 = control_points[if span == 0 { 0 } else { span - 1 }];
        let p1 = control_points[span];
        let p2 = control_points[span + 1];
        let p3 = control_points[if span + 2 < n { span + 2 } else { n - 1 }];

        // Coarse search
        let subdivs: u32 = 16;
        let mut best_t = 0.0_f64;
        let mut best_d_sq = f64::INFINITY;

        for i in 0..=subdivs {
            let t = f64::from(i) / f64::from(subdivs);
            let q = cr_point(p0, p1, p2, p3, t);
            let d_sq = nalgebra::distance_squared(p, &q);
            if d_sq < best_d_sq {
                best_d_sq = d_sq;
                best_t = t;
            }
        }

        // Newton refinement
        let mut t = best_t;
        for _ in 0..4 {
            let c = cr_point(p0, p1, p2, p3, t);
            let c_d = cr_deriv(p0, p1, p2, p3, t);
            let c_dd = cr_deriv2(p0, p1, p2, p3, t);
            let diff = c - p;
            let fp = 2.0 * diff.dot(&c_d);
            let fpp = 2.0 * (c_d.dot(&c_d) + diff.dot(&c_dd));
            if fpp.abs() < 1e-20 {
                break;
            }
            t -= fp / fpp;
            t = t.clamp(0.0, 1.0);
        }

        let c = cr_point(p0, p1, p2, p3, t);
        let d_sq = nalgebra::distance_squared(p, &c);
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
            nearest = c;
        }
    }

    let dist = min_dist_sq.sqrt();
    if dist < 1e-15 {
        return Vector3::zeros();
    }
    (p - nearest) / dist
}

// ── Loft gradient helper ────────────────────────────────────────────────

/// Loft: cylinder-like gradient with z-varying radius.
fn grad_loft(stations: &[[f64; 2]], p: &Point3<f64>) -> Vector3<f64> {
    let z_min = stations[0][0];
    let z_max = stations[stations.len() - 1][0];
    let r_xy = p.x.hypot(p.y);

    let z_clamped = p.z.clamp(z_min, z_max);
    let r_at_z = loft_radius_at(stations, z_clamped);

    let d_radial = r_xy - r_at_z;
    let d_axial = (z_min - p.z).max(p.z - z_max);

    let (rx, ry) = if r_xy > 1e-15 {
        (p.x / r_xy, p.y / r_xy)
    } else {
        (1.0, 0.0)
    };

    // R'(z) — only meaningful when z is strictly inside the station range
    let dr_dz = if p.z > z_min && p.z < z_max {
        loft_radius_deriv(stations, p.z)
    } else {
        0.0
    };

    if d_radial > 0.0 && d_axial > 0.0 {
        // Corner: outside both barrel and cap
        let d = d_radial.hypot(d_axial);
        if d < 1e-15 {
            return Vector3::zeros();
        }
        let z_sign = if p.z < z_min { -1.0 } else { 1.0 };
        Vector3::new(d_radial / d * rx, d_radial / d * ry, d_axial / d * z_sign)
    } else if d_radial > d_axial {
        // Barrel dominant
        Vector3::new(rx, ry, -dr_dz)
    } else {
        // Cap dominant
        if p.z <= z_min {
            Vector3::new(0.0, 0.0, -1.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        }
    }
}

// ── Duplicated helpers (private in evaluate.rs) ─────────────────────────

/// Fold coordinate into fundamental domain for infinite repetition.
fn fold_repeat(coord: f64, spacing: f64) -> f64 {
    spacing.mul_add(-(coord / spacing).round(), coord)
}

/// Fold coordinate for bounded repetition.
fn fold_repeat_bounded(coord: f64, spacing: f64, count: u32) -> f64 {
    if count <= 1 {
        return coord;
    }
    let n = f64::from(count);
    let half = (n - 1.0) * spacing * 0.5;
    let id = ((coord + half) / spacing).round().clamp(0.0, n - 1.0);
    coord - id.mul_add(spacing, -half)
}

/// Catmull-Rom spline evaluation at parameter t ∈ [0, 1].
fn cr_point(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Point3<f64> {
    let t2 = t * t;
    let t3 = t2 * t;
    let c = (p1.coords * 2.0
        + (-p0.coords + p2.coords) * t
        + (p0.coords * 2.0 - p1.coords * 5.0 + p2.coords * 4.0 - p3.coords) * t2
        + (-p0.coords + p1.coords * 3.0 - p2.coords * 3.0 + p3.coords) * t3)
        * 0.5;
    Point3::from(c)
}

/// First derivative of Catmull-Rom spline.
fn cr_deriv(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Vector3<f64> {
    let t2 = t * t;
    ((-p0.coords + p2.coords)
        + (p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0) * t
        + (-p0.coords * 3.0 + p1.coords * 9.0 - p2.coords * 9.0 + p3.coords * 3.0) * t2)
        * 0.5
}

/// Second derivative of Catmull-Rom spline.
fn cr_deriv2(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Vector3<f64> {
    ((p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0)
        + (-p0.coords * 6.0 + p1.coords * 18.0 - p2.coords * 18.0 + p3.coords * 6.0) * t)
        * 0.5
}

/// 1D Catmull-Rom interpolation (same basis as 3D version).
#[allow(clippy::many_single_char_names)]
fn cr_1d(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;
    let linear = 2.0f64.mul_add(p1, (-p0 + p2) * t);
    let quad = 2.0f64.mul_add(p0, 4.0f64.mul_add(p2, -(5.0 * p1) - p3)) * t2;
    let cubic = 3.0f64.mul_add(p1, 3.0f64.mul_add(-p2, -p0 + p3)) * t3;
    0.5 * (linear + quad + cubic)
}

/// 1D Catmull-Rom first derivative.
#[allow(clippy::many_single_char_names)]
fn cr_1d_deriv(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let linear = -p0 + p2;
    let quad = 2.0 * 2.0f64.mul_add(p0, 4.0f64.mul_add(p2, -(5.0 * p1) - p3)) * t;
    let cubic = 3.0 * 3.0f64.mul_add(p1, 3.0f64.mul_add(-p2, -p0 + p3)) * t2;
    0.5 * (linear + quad + cubic)
}

/// Catmull-Rom interpolation of loft radius at a given z.
fn loft_radius_at(stations: &[[f64; 2]], z: f64) -> f64 {
    let n = stations.len();
    let z_clamped = z.clamp(stations[0][0], stations[n - 1][0]);

    let mut span = 0;
    for (i, station) in stations[..n - 1].iter().enumerate() {
        if z_clamped >= station[0] {
            span = i;
        }
    }
    span = span.min(n - 2);

    let z0 = stations[span][0];
    let z1 = stations[span + 1][0];
    let dz = z1 - z0;
    if dz < 1e-15 {
        return stations[span][1];
    }
    let t = ((z_clamped - z0) / dz).clamp(0.0, 1.0);

    let r_m1 = stations[span.saturating_sub(1)][1];
    let r0 = stations[span][1];
    let r1 = stations[span + 1][1];
    let r2 = stations[(span + 2).min(n - 1)][1];

    cr_1d(r_m1, r0, r1, r2, t)
}

/// Derivative dR/dz of loft radius interpolation at a given z.
fn loft_radius_deriv(stations: &[[f64; 2]], z: f64) -> f64 {
    let n = stations.len();
    let z_clamped = z.clamp(stations[0][0], stations[n - 1][0]);

    let mut span = 0;
    for (i, station) in stations[..n - 1].iter().enumerate() {
        if z_clamped >= station[0] {
            span = i;
        }
    }
    span = span.min(n - 2);

    let z0 = stations[span][0];
    let z1 = stations[span + 1][0];
    let dz = z1 - z0;
    if dz < 1e-15 {
        return 0.0;
    }
    let t = ((z_clamped - z0) / dz).clamp(0.0, 1.0);

    let r_m1 = stations[span.saturating_sub(1)][1];
    let r0 = stations[span][1];
    let r1 = stations[span + 1][1];
    let r2 = stations[(span + 2).min(n - 1)][1];

    cr_1d_deriv(r_m1, r0, r1, r2, t) / dz
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field_node::{UserEvalFn, UserIntervalFn, Val};
    use std::sync::Arc;

    const EPS: f64 = 1e-7;

    /// Central finite difference gradient.
    fn fd_gradient(node: &FieldNode, p: &Point3<f64>, h: f64) -> Vector3<f64> {
        let inv = 0.5 / h;
        Vector3::new(
            (node.evaluate(&Point3::new(p.x + h, p.y, p.z))
                - node.evaluate(&Point3::new(p.x - h, p.y, p.z)))
                * inv,
            (node.evaluate(&Point3::new(p.x, p.y + h, p.z))
                - node.evaluate(&Point3::new(p.x, p.y - h, p.z)))
                * inv,
            (node.evaluate(&Point3::new(p.x, p.y, p.z + h))
                - node.evaluate(&Point3::new(p.x, p.y, p.z - h)))
                * inv,
        )
    }

    /// Check analytic vs FD gradient component-wise.
    fn check(node: &FieldNode, p: Point3<f64>, tol: f64) {
        let a = node.gradient(&p);
        let fd = fd_gradient(node, &p, EPS);
        let dx = (a.x - fd.x).abs();
        let dy = (a.y - fd.y).abs();
        let dz = (a.z - fd.z).abs();
        assert!(
            dx < tol && dy < tol && dz < tol,
            "gradient mismatch at {p:?}\n  analytic = {a:?}\n  fd       = {fd:?}\n  diff     = ({dx:.2e}, {dy:.2e}, {dz:.2e})\n  tol      = {tol:.0e}"
        );
    }

    // ── Geometric primitives ────────────────────────────────────────

    #[test]
    fn gradient_sphere() {
        let n = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        check(&n, Point3::new(1.0, 2.0, 3.0), 1e-6);
        check(&n, Point3::new(0.5, -0.3, 0.1), 1e-6);
        check(&n, Point3::new(5.0, 0.0, 0.0), 1e-6);
    }

    #[test]
    fn gradient_cuboid() {
        let n = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        // Inside
        check(&n, Point3::new(0.3, 0.4, 0.5), 1e-6);
        // Outside face
        check(&n, Point3::new(2.0, 0.5, 0.5), 1e-6);
        // Outside edge
        check(&n, Point3::new(2.0, 3.0, 0.5), 1e-6);
        // Outside corner
        check(&n, Point3::new(2.0, 3.0, 4.0), 1e-6);
    }

    #[test]
    fn gradient_cylinder() {
        let n = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        check(&n, Point3::new(0.5, 0.3, 0.5), 1e-6);
        check(&n, Point3::new(3.0, 0.5, 0.5), 1e-6);
        check(&n, Point3::new(0.5, 0.3, 4.0), 1e-6);
        check(&n, Point3::new(3.0, 0.5, 4.0), 1e-6);
    }

    #[test]
    fn gradient_capsule() {
        let n = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        check(&n, Point3::new(0.3, 0.4, 0.5), 1e-6);
        check(&n, Point3::new(0.3, 0.4, 3.5), 1e-6);
        check(&n, Point3::new(2.0, 0.5, 0.0), 1e-6);
    }

    #[test]
    fn gradient_ellipsoid() {
        let n = FieldNode::Ellipsoid {
            radii: Vector3::new(2.0, 3.0, 4.0),
        };
        check(&n, Point3::new(1.0, 1.0, 1.0), 1e-4);
        check(&n, Point3::new(3.0, 1.0, 1.0), 1e-4);
        check(&n, Point3::new(0.5, 2.0, 3.0), 1e-4);
    }

    #[test]
    fn gradient_torus() {
        let n = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        check(&n, Point3::new(4.0, 0.5, 0.3), 1e-6);
        check(&n, Point3::new(6.5, 0.0, 0.3), 1e-6);
        check(&n, Point3::new(3.0, 3.0, 0.5), 1e-6);
    }

    #[test]
    fn gradient_cone() {
        let n = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        // Outside surface
        check(&n, Point3::new(2.0, 0.3, -1.0), 1e-5);
        // Inside
        check(&n, Point3::new(0.3, 0.2, -2.0), 1e-5);
        // Above apex
        check(&n, Point3::new(0.5, 0.3, 1.0), 1e-5);
        // Below base
        check(&n, Point3::new(0.5, 0.3, -5.0), 1e-5);
        check(&n, Point3::new(3.0, 0.3, -5.0), 1e-5);
    }

    #[test]
    fn gradient_plane() {
        let n = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 2.0,
        };
        check(&n, Point3::new(1.0, 2.0, 5.0), 1e-6);
    }

    // ── Bio-inspired primitives ─────────────────────────────────────

    #[test]
    fn gradient_superellipsoid() {
        let n = FieldNode::Superellipsoid {
            radii: Vector3::new(2.0, 3.0, 4.0),
            n1: 1.5,
            n2: 2.5,
        };
        check(&n, Point3::new(1.0, 1.0, 1.0), 1e-4);
        check(&n, Point3::new(2.5, 1.0, 3.0), 1e-4);
    }

    #[test]
    fn gradient_log_spiral() {
        let n = FieldNode::LogSpiral {
            a: 1.0,
            b: 0.2,
            thickness: 0.3,
            turns: 2.0,
        };
        check(&n, Point3::new(2.0, 1.0, 0.5), 1e-4);
        check(&n, Point3::new(0.5, 1.5, 0.0), 1e-4);
    }

    #[test]
    fn gradient_gyroid() {
        let n = FieldNode::Gyroid {
            scale: 2.0,
            thickness: 0.3,
        };
        // Avoid points where g ≈ 0 (sign discontinuity)
        check(&n, Point3::new(0.5, 0.3, 0.7), 1e-4);
        check(&n, Point3::new(1.0, 0.5, 0.2), 1e-4);
    }

    #[test]
    fn gradient_schwarz_p() {
        let n = FieldNode::SchwarzP {
            scale: 2.0,
            thickness: 0.3,
        };
        check(&n, Point3::new(0.5, 0.3, 0.7), 1e-4);
        check(&n, Point3::new(1.0, 0.5, 0.2), 1e-4);
    }

    #[test]
    fn gradient_helix() {
        let n = FieldNode::Helix {
            radius: 2.0,
            pitch: 3.0,
            thickness: 0.5,
            turns: 3.0,
        };
        check(&n, Point3::new(2.5, 0.5, 1.5), 1e-4);
        check(&n, Point3::new(1.0, 2.0, 4.0), 1e-4);
    }

    // ── Path-based primitives ───────────────────────────────────────

    #[test]
    fn gradient_pipe() {
        let n = FieldNode::Pipe {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            radius: 0.5,
        };
        check(&n, Point3::new(2.5, 1.0, 0.5), 1e-5);
        check(&n, Point3::new(5.5, 2.5, 0.5), 1e-5);
    }

    #[test]
    fn gradient_pipe_spline() {
        let n = FieldNode::PipeSpline {
            control_points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(4.0, 2.0, 0.0),
                Point3::new(6.0, 2.0, 0.0),
            ],
            radius: 0.3,
        };
        check(&n, Point3::new(3.0, 0.5, 0.5), 1e-4);
        check(&n, Point3::new(1.0, 0.5, 0.3), 1e-4);
    }

    #[test]
    fn gradient_loft() {
        // Constant radius → cylinder
        let n = FieldNode::Loft {
            stations: vec![[0.0, 2.0], [5.0, 2.0]],
        };
        check(&n, Point3::new(1.0, 0.5, 2.5), 1e-5);
        check(&n, Point3::new(3.0, 0.5, -1.0), 1e-5);
        // Varying radius
        let n2 = FieldNode::Loft {
            stations: vec![[0.0, 1.0], [3.0, 2.0], [6.0, 1.5]],
        };
        check(&n2, Point3::new(1.0, 0.5, 1.5), 1e-4);
        check(&n2, Point3::new(0.5, 0.3, 4.5), 1e-4);
    }

    // ── Booleans ────────────────────────────────────────────────────

    #[test]
    fn gradient_union() {
        let n = FieldNode::Union(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere {
                    radius: Val::from(2.0),
                }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
        );
        check(&n, Point3::new(-1.0, 0.5, 0.3), 1e-5);
        check(&n, Point3::new(4.0, 0.5, 0.3), 1e-5);
    }

    #[test]
    fn gradient_subtract() {
        let n = FieldNode::Subtract(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
        );
        check(&n, Point3::new(2.0, 0.3, 0.2), 1e-5);
        check(&n, Point3::new(4.0, 0.3, 0.2), 1e-5);
    }

    #[test]
    fn gradient_intersect() {
        let n = FieldNode::Intersect(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
        );
        check(&n, Point3::new(0.5, 0.3, 0.2), 1e-5);
    }

    #[test]
    fn gradient_smooth_union() {
        let n = FieldNode::SmoothUnion(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere {
                    radius: Val::from(2.0),
                }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            Val::from(1.0),
        );
        check(&n, Point3::new(1.5, 0.5, 0.3), 1e-4);
        check(&n, Point3::new(-1.0, 0.5, 0.3), 1e-4);
    }

    #[test]
    fn gradient_smooth_subtract() {
        let n = FieldNode::SmoothSubtract(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.5),
            }),
            Val::from(0.5),
        );
        check(&n, Point3::new(2.0, 0.3, 0.2), 1e-4);
    }

    #[test]
    fn gradient_smooth_intersect() {
        let n = FieldNode::SmoothIntersect(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
            Val::from(0.5),
        );
        check(&n, Point3::new(1.5, 1.0, 0.5), 1e-4);
    }

    #[test]
    fn gradient_smooth_union_all() {
        let n = FieldNode::SmoothUnionAll(
            vec![
                FieldNode::Sphere {
                    radius: Val::from(2.0),
                },
                FieldNode::Translate(
                    Box::new(FieldNode::Sphere {
                        radius: Val::from(2.0),
                    }),
                    Vector3::new(3.0, 0.0, 0.0),
                ),
                FieldNode::Translate(
                    Box::new(FieldNode::Sphere {
                        radius: Val::from(2.0),
                    }),
                    Vector3::new(0.0, 3.0, 0.0),
                ),
            ],
            Val::from(1.0),
        );
        check(&n, Point3::new(1.0, 1.0, 0.5), 1e-4);
    }

    #[test]
    fn gradient_smooth_union_variable() {
        // Constant radius_fn → same as SmoothUnion
        let n = FieldNode::SmoothUnionVariable {
            a: Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            b: Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere {
                    radius: Val::from(2.0),
                }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            radius_fn: UserEvalFn(Arc::new(|_| 1.0)),
            max_k: 1.0,
        };
        check(&n, Point3::new(1.5, 0.5, 0.3), 1e-4);
    }

    // ── Transforms ──────────────────────────────────────────────────

    #[test]
    fn gradient_translate() {
        let n = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(1.0, 2.0, 3.0),
        );
        check(&n, Point3::new(2.0, 3.0, 4.0), 1e-6);
    }

    #[test]
    fn gradient_rotate() {
        use nalgebra::UnitQuaternion;
        let n = FieldNode::Rotate(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 3.0),
            }),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_4),
        );
        check(&n, Point3::new(1.5, 0.5, 0.3), 1e-5);
    }

    #[test]
    fn gradient_scale_uniform() {
        let n = FieldNode::ScaleUniform(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            3.0,
        );
        check(&n, Point3::new(2.0, 1.0, 0.5), 1e-6);
    }

    #[test]
    fn gradient_mirror() {
        let n = FieldNode::Mirror(
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere {
                    radius: Val::from(1.0),
                }),
                Vector3::new(2.0, 0.0, 0.0),
            )),
            Vector3::new(1.0, 0.0, 0.0),
        );
        // Positive side
        check(&n, Point3::new(2.5, 0.3, 0.2), 1e-5);
        // Negative side (mirrored)
        check(&n, Point3::new(-2.5, 0.3, 0.2), 1e-5);
    }

    // ── Domain operations ───────────────────────────────────────────

    #[test]
    fn gradient_shell() {
        let n = FieldNode::Shell(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Val::from(0.2),
        );
        // Outside shell
        check(&n, Point3::new(4.0, 0.3, 0.2), 1e-5);
        // Inside shell (between inner and outer surface)
        check(&n, Point3::new(2.0, 0.3, 0.2), 1e-5);
    }

    #[test]
    fn gradient_round() {
        let n = FieldNode::Round(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
            Val::from(0.3),
        );
        check(&n, Point3::new(1.5, 0.5, 0.3), 1e-5);
    }

    #[test]
    fn gradient_offset() {
        let n = FieldNode::Offset(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Val::from(0.5),
        );
        check(&n, Point3::new(3.0, 0.5, 0.3), 1e-5);
    }

    #[test]
    fn gradient_elongate() {
        let n = FieldNode::Elongate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        // Outside elongation region
        check(&n, Point3::new(3.5, 0.3, 0.4), 1e-5);
    }

    #[test]
    fn gradient_twist() {
        let n = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 5.0),
            }),
            0.5,
        );
        check(&n, Point3::new(0.5, 0.3, 2.0), 1e-4);
        check(&n, Point3::new(1.5, 0.3, 1.0), 1e-4);
    }

    #[test]
    fn gradient_bend() {
        let n = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 5.0),
            }),
            0.3,
        );
        check(&n, Point3::new(0.5, 0.3, 2.0), 1e-4);
        check(&n, Point3::new(1.5, 0.3, 1.0), 1e-4);
    }

    #[test]
    fn gradient_repeat() {
        let n = FieldNode::Repeat(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(4.0, 4.0, 4.0),
        );
        // Well inside a cell
        check(&n, Point3::new(0.5, 0.3, 0.2), 1e-5);
    }

    #[test]
    fn gradient_repeat_bounded() {
        let n = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere {
                radius: Val::from(0.5),
            }),
            spacing: Vector3::new(3.0, 3.0, 3.0),
            count: [3, 1, 1],
        };
        // Near a copy center
        check(&n, Point3::new(0.3, 0.2, 0.1), 1e-5);
        check(&n, Point3::new(3.3, 0.2, 0.1), 1e-5);
    }

    #[test]
    fn gradient_user_fn() {
        // Custom sphere as UserFn
        let n = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p| {
                p.z.mul_add(p.z, p.x.mul_add(p.x, p.y * p.y)).sqrt() - 2.0
            })),
            interval: Some(UserIntervalFn(Arc::new(|_| (-2.0, 10.0)))),
            bounds: cf_geometry::Aabb::new(
                Point3::new(-3.0, -3.0, -3.0),
                Point3::new(3.0, 3.0, 3.0),
            ),
        };
        check(&n, Point3::new(1.0, 0.5, 0.3), 1e-4);
    }
}
