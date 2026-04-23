//! Point evaluation of field nodes.
//!
//! Each primitive computes its signed distance (or approximate signed distance)
//! at a query point. Convention: negative inside, positive outside, zero on
//! surface.

use nalgebra::Point3;

use crate::field_node::FieldNode;

impl FieldNode {
    /// Evaluate the scalar field at a point.
    ///
    /// Returns the signed distance (exact or approximate depending on the
    /// primitive). Negative = inside, positive = outside, zero = on surface.
    // Procedural glue code; natural breakpoints are few.
    #[allow(clippy::too_many_lines)]
    pub(crate) fn evaluate(&self, p: &Point3<f64>) -> f64 {
        match self {
            // Primitives
            Self::Sphere { radius } => eval_sphere(radius.eval(), p),
            Self::Cuboid { half_extents } => eval_cuboid(half_extents, p),
            Self::Cylinder {
                radius,
                half_height,
            } => eval_cylinder(*radius, *half_height, p),
            Self::Capsule {
                radius,
                half_height,
            } => eval_capsule(*radius, *half_height, p),
            Self::Ellipsoid { radii } => eval_ellipsoid(radii, p),
            Self::Torus { major, minor } => eval_torus(*major, *minor, p),
            Self::Cone { radius, height } => eval_cone(*radius, *height, p),
            Self::Plane { normal, offset } => eval_plane(normal, *offset, p),
            Self::Superellipsoid { radii, n1, n2 } => eval_superellipsoid(radii, *n1, *n2, p),
            Self::LogSpiral {
                a,
                b,
                thickness,
                turns,
            } => eval_log_spiral(*a, *b, *thickness, *turns, p),
            Self::Gyroid { scale, thickness } => eval_gyroid(*scale, *thickness, p),
            Self::SchwarzP { scale, thickness } => eval_schwarz_p(*scale, *thickness, p),
            Self::Helix {
                radius,
                pitch,
                thickness,
                turns,
            } => eval_helix(*radius, *pitch, *thickness, *turns, p),
            Self::Pipe { vertices, radius } => eval_pipe(vertices, *radius, p),
            Self::PipeSpline {
                control_points,
                radius,
            } => eval_pipe_spline(control_points, *radius, p),
            Self::Loft { stations } => eval_loft(stations, p),

            // Booleans
            Self::Union(a, b) => a.evaluate(p).min(b.evaluate(p)),
            Self::Subtract(a, b) => a.evaluate(p).max(-b.evaluate(p)),
            Self::Intersect(a, b) => a.evaluate(p).max(b.evaluate(p)),
            Self::SmoothUnion(a, b, k) => eval_smooth_union(a.evaluate(p), b.evaluate(p), k.eval()),
            Self::SmoothSubtract(a, b, k) => {
                // smooth_subtract(a, b, k) = -smooth_union(-a, b, k)
                -eval_smooth_union(-a.evaluate(p), b.evaluate(p), k.eval())
            }
            Self::SmoothIntersect(a, b, k) => {
                // smooth_intersect(a, b, k) = -smooth_union(-a, -b, k)
                -eval_smooth_union(-a.evaluate(p), -b.evaluate(p), k.eval())
            }
            Self::SmoothUnionAll(children, k) => {
                let values: Vec<f64> = children.iter().map(|c| c.evaluate(p)).collect();
                eval_smooth_union_all(&values, k.eval())
            }
            Self::SmoothUnionVariable {
                a, b, radius_fn, ..
            } => {
                let k = (radius_fn.0)(*p).max(1e-15);
                eval_smooth_union(a.evaluate(p), b.evaluate(p), k)
            }

            // Transforms
            Self::Translate(child, offset) => {
                let p_local = Point3::new(p.x - offset.x, p.y - offset.y, p.z - offset.z);
                child.evaluate(&p_local)
            }
            Self::Rotate(child, q) => {
                let p_local = q.inverse_transform_point(p);
                child.evaluate(&p_local)
            }
            Self::ScaleUniform(child, s) => {
                let inv_s = 1.0 / *s;
                let p_local = Point3::new(p.x * inv_s, p.y * inv_s, p.z * inv_s);
                child.evaluate(&p_local) * *s
            }
            Self::Mirror(child, normal) => {
                let d = p.coords.dot(normal).min(0.0);
                let two_d = 2.0 * d;
                let p_mirrored = Point3::new(
                    two_d.mul_add(-normal.x, p.x),
                    two_d.mul_add(-normal.y, p.y),
                    two_d.mul_add(-normal.z, p.z),
                );
                child.evaluate(&p_mirrored)
            }

            // Domain operations
            Self::Shell(child, thickness) => child.evaluate(p).abs() - thickness.eval(),
            Self::Round(child, radius) => child.evaluate(p) - radius.eval(),
            Self::Offset(child, distance) => child.evaluate(p) - distance.eval(),
            Self::Elongate(child, half) => {
                let q = Point3::new(
                    p.x - p.x.clamp(-half.x, half.x),
                    p.y - p.y.clamp(-half.y, half.y),
                    p.z - p.z.clamp(-half.z, half.z),
                );
                child.evaluate(&q)
            }
            Self::Twist(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.y)), s.mul_add(p.x, c * p.y), p.z);
                child.evaluate(&q)
            }
            Self::Bend(child, rate) => {
                let angle = rate * p.z;
                let (s, c) = angle.sin_cos();
                let q = Point3::new(c.mul_add(p.x, -(s * p.z)), p.y, s.mul_add(p.x, c * p.z));
                child.evaluate(&q)
            }
            Self::Repeat(child, spacing) => {
                let q = Point3::new(
                    fold_repeat(p.x, spacing.x),
                    fold_repeat(p.y, spacing.y),
                    fold_repeat(p.z, spacing.z),
                );
                child.evaluate(&q)
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
                child.evaluate(&q)
            }

            // User function
            Self::UserFn { eval, .. } => (eval.0)(*p),
        }
    }
}

// ── Smooth boolean helpers ──────────────────────────────────────────────

/// Polynomial smooth minimum (IQ). Blend radius `k`.
///
/// `h = clamp(0.5 + 0.5*(b-a)/k, 0, 1)`
/// `result = mix(b, a, h) - k*h*(1-h)`
///
/// Always ≤ min(a, b). Maximum correction is k/4 at h=0.5.
fn eval_smooth_union(a: f64, b: f64, k: f64) -> f64 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    let one_minus_h = 1.0 - h;
    let mix = a.mul_add(h, b * one_minus_h);
    (k * h).mul_add(-one_minus_h, mix)
}

/// Symmetric n-ary smooth union via log-sum-exp.
///
/// `f = m - k * ln(Σ exp(-(xᵢ - m) / k))`
/// where `m = min(xᵢ)` for numerical stability.
///
/// Order-independent. Approaches `min()` as k→0.
fn eval_smooth_union_all(values: &[f64], k: f64) -> f64 {
    if values.is_empty() {
        return f64::INFINITY;
    }
    if values.len() == 1 {
        return values[0];
    }
    // Log-sum-exp with shift for numerical stability
    let m = values.iter().copied().fold(f64::INFINITY, f64::min);
    let sum: f64 = values.iter().map(|&v| (-(v - m) / k).exp()).sum();
    k.mul_add(-sum.ln(), m)
}

// ── Repeat fold helpers ─────────────────────────────────────────────────

/// Fold coordinate into fundamental domain for infinite repetition.
/// Maps coord into `[-spacing/2, spacing/2]`.
fn fold_repeat(coord: f64, spacing: f64) -> f64 {
    spacing.mul_add(-(coord / spacing).round(), coord)
}

/// Fold coordinate for bounded repetition with N copies centered at origin.
/// Copy positions: `(i - (N-1)/2) · spacing` for `i` in `0..N`.
/// Returns the coordinate relative to the nearest copy center.
fn fold_repeat_bounded(coord: f64, spacing: f64, count: u32) -> f64 {
    if count <= 1 {
        return coord;
    }
    let n = f64::from(count);
    let half = (n - 1.0) * spacing * 0.5;
    let id = ((coord + half) / spacing).round().clamp(0.0, n - 1.0);
    coord - id.mul_add(spacing, -half)
}

// ── Primitive SDF implementations ────────────────────────────────────────

/// Sphere: `|p| - r`. Exact SDF.
fn eval_sphere(radius: f64, p: &Point3<f64>) -> f64 {
    p.coords.norm() - radius
}

/// Axis-aligned box: exact signed distance to box with given half-extents.
fn eval_cuboid(half: &nalgebra::Vector3<f64>, p: &Point3<f64>) -> f64 {
    let q = nalgebra::Vector3::new(p.x.abs() - half.x, p.y.abs() - half.y, p.z.abs() - half.z);
    let outside = nalgebra::Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
    let inside = q.x.max(q.y.max(q.z)).min(0.0);
    outside + inside
}

/// Z-aligned cylinder: exact signed distance.
fn eval_cylinder(radius: f64, half_height: f64, p: &Point3<f64>) -> f64 {
    let d_radial = p.x.hypot(p.y) - radius;
    let d_axial = p.z.abs() - half_height;
    let outside = d_radial.max(0.0).hypot(d_axial.max(0.0));
    let inside = d_radial.max(d_axial).min(0.0);
    outside + inside
}

/// Z-aligned capsule: exact signed distance.
/// The capsule is a cylinder of half-height `half_height` capped with
/// hemispheres of radius `radius`.
fn eval_capsule(radius: f64, half_height: f64, p: &Point3<f64>) -> f64 {
    let clamped_z = p.z.clamp(-half_height, half_height);
    let closest_axis = Point3::new(0.0, 0.0, clamped_z);
    nalgebra::distance(p, &closest_axis) - radius
}

/// Ellipsoid: approximate signed distance via scaled-sphere method.
///
/// This is NOT an exact SDF — the magnitude is only approximately correct,
/// but the sign and zero-isosurface are correct. The approximation uses
/// Inigo Quilez's method: evaluate on the unit sphere in scaled space,
/// then correct by the local scaling factor.
fn eval_ellipsoid(radii: &nalgebra::Vector3<f64>, p: &Point3<f64>) -> f64 {
    // Scale point into unit-sphere space
    let scaled = nalgebra::Vector3::new(p.x / radii.x, p.y / radii.y, p.z / radii.z);
    let scaled_norm = scaled.norm();

    if scaled_norm < 1.0e-12 {
        // At the center — distance is the smallest radius
        return -radii.x.min(radii.y.min(radii.z));
    }

    // Gradient of the scaling for distance correction
    let grad = nalgebra::Vector3::new(
        p.x / (radii.x * radii.x),
        p.y / (radii.y * radii.y),
        p.z / (radii.z * radii.z),
    );
    let grad_norm = grad.norm();

    if grad_norm < 1.0e-12 {
        return scaled_norm - 1.0;
    }

    // Approximate signed distance
    scaled_norm * (scaled_norm - 1.0) / grad_norm
}

/// Torus in the XY plane: exact SDF.
/// `major` = distance from center to tube center.
/// `minor` = tube radius.
fn eval_torus(major: f64, minor: f64, p: &Point3<f64>) -> f64 {
    let q_xy = p.x.hypot(p.y) - major;
    q_xy.hypot(p.z) - minor
}

/// Cone with apex at origin, expanding along -Z with base at `z = -height`.
/// Exact SDF.
fn eval_cone(radius: f64, height: f64, p: &Point3<f64>) -> f64 {
    // Work in the 2D (r, z) plane where r = distance from Z axis.
    let r = p.x.hypot(p.y);

    // Cone axis: apex at z=0, base at z=-height with radius `radius`.
    // The cone surface in (r, z) is the line from (0, 0) to (radius, -height).
    let cone_len = radius.hypot(height);
    // Outward normal of the cone surface in (r, z) space: (height, radius) / cone_len
    let nr = height / cone_len;
    let nz = radius / cone_len;

    // Project onto the cone surface
    let dot_surface = r.mul_add(nr, p.z * nz);

    if p.z > 0.0 {
        // Above apex — distance to apex point
        r.hypot(p.z)
    } else if p.z < -height {
        // Below base — distance to base disk
        let d_radial = (r - radius).max(0.0);
        let d_base = -(p.z + height);
        d_radial.hypot(d_base)
    } else {
        // Between apex and base height
        let t = -p.z / height;
        let cone_r_at_z = radius * t;

        if r <= cone_r_at_z {
            // Inside the cone — negative distance
            let d_surf = -dot_surface;
            let d_base_inner = p.z + height;
            -(d_surf.min(d_base_inner))
        } else {
            // Outside the cone surface between apex and base
            dot_surface
        }
    }
}

/// Half-space: `dot(normal, p) - offset`. Exact SDF when normal is unit length.
fn eval_plane(normal: &nalgebra::Vector3<f64>, offset: f64, p: &Point3<f64>) -> f64 {
    normal.dot(&p.coords) - offset
}

// ── Bio-inspired primitive implementations ──────────────────────────────

/// Superellipsoid: tunable between box, cylinder, sphere, diamond.
/// Raw implicit function — zero-isosurface is correct but field magnitude is
/// NOT a signed distance. Safe for meshing (sign correctness + bounded gradient).
///
/// Gradient (for Session 19):
///   Let `G = |x/rx|^e2 + |y/ry|^e2`, `F = G^(n2/n1) + |z/rz|^e1`.
///   `f = F^(n1/2)`. Then:
///   `df/dx = (n1/2)·F^(n1/2-1)·(n2/n1)·G^(n2/n1-1)·e2·|x/rx|^(e2-1)·sign(x)/rx`
///   `df/dy = (n1/2)·F^(n1/2-1)·(n2/n1)·G^(n2/n1-1)·e2·|y/ry|^(e2-1)·sign(y)/ry`
///   `df/dz = (n1/2)·F^(n1/2-1)·e1·|z/rz|^(e1-1)·sign(z)/rz`
fn eval_superellipsoid(radii: &nalgebra::Vector3<f64>, n1: f64, n2: f64, p: &Point3<f64>) -> f64 {
    let e2 = 2.0 / n2;
    let e1 = 2.0 / n1;
    let xy = (p.x / radii.x).abs().powf(e2) + (p.y / radii.y).abs().powf(e2);
    let f = (xy.powf(n2 / n1) + (p.z / radii.z).abs().powf(e1)).powf(n1 / 2.0);
    f - 1.0
}

/// Logarithmic spiral tube: distance to spiral curve `r(θ) = a·exp(b·θ)` minus
/// thickness. Multi-turn search finds the nearest spiral arm.
///
/// Gradient (for Session 19):
///   The gradient is the unit direction from the nearest point on the spiral
///   curve to the query point (Lipschitz = 1 for the distance part).
// Short names mirror textbook / paper notation.
#[allow(clippy::many_single_char_names, clippy::cast_possible_truncation)]
fn eval_log_spiral(
    init_radius: f64,
    growth: f64,
    thickness: f64,
    turns: f64,
    p: &Point3<f64>,
) -> f64 {
    use std::f64::consts::TAU;

    let r_xy = p.x.hypot(p.y);
    let theta = p.y.atan2(p.x); // [-π, π]
    let z_sq = p.z * p.z;

    let theta_max = turns * TAU;

    // Find the turn whose spiral radius is closest to r_xy at this angle.
    // The spiral at angle φ has radius init_radius·exp(growth·φ).
    // At our query angle θ, the spiral passes at φ = θ + 2πk.
    // Solve for k: init_radius·exp(growth·(θ+2πk)) = r_xy
    //   → k = (ln(r_xy/init_radius)/growth - θ) / 2π
    let k_continuous = if r_xy > 1e-15 && growth.abs() > 1e-15 {
        ((r_xy / init_radius).ln() / growth - theta) / TAU
    } else {
        0.0
    };
    let k_base = k_continuous.round() as i64;

    let mut min_dist_sq = f64::INFINITY;

    let check_angle = |phi: f64, best: &mut f64| {
        let r_spiral = init_radius * (growth * phi).exp();
        let dx = r_spiral.mul_add(phi.cos(), -p.x);
        let dy = r_spiral.mul_add(phi.sin(), -p.y);
        let d_sq = dy.mul_add(dy, dx.mul_add(dx, z_sq));
        *best = best.min(d_sq);
    };

    // Check 5 nearest turns for robustness
    #[allow(clippy::cast_precision_loss)]
    for dk in -2..=2_i64 {
        let ki = k_base + dk;
        let phi = TAU.mul_add(ki as f64, theta).clamp(0.0, theta_max);
        check_angle(phi, &mut min_dist_sq);
    }

    // Also check endpoints explicitly
    check_angle(0.0, &mut min_dist_sq);
    check_angle(theta_max, &mut min_dist_sq);

    min_dist_sq.sqrt() - thickness
}

/// Gyroid triply-periodic minimal surface.
/// Field: `|sin(sx)·cos(sy) + sin(sy)·cos(sz) + sin(sz)·cos(sx)| - thickness`
///
/// Gradient (for Session 19):
///   Let `g = sin(sx)cos(sy) + sin(sy)cos(sz) + sin(sz)cos(sx)`.
///   `dg/dx = s·cos(sx)·cos(sy) - s·sin(sz)·sin(sx)`
///   `dg/dy = -s·sin(sx)·sin(sy) + s·cos(sy)·cos(sz)`
///   `dg/dz = -s·sin(sy)·sin(sz) + s·cos(sz)·cos(sx)`
///   Gradient of field = `sign(g) · ∇g`
fn eval_gyroid(scale: f64, thickness: f64, p: &Point3<f64>) -> f64 {
    let sx = (scale * p.x).sin();
    let sy = (scale * p.y).sin();
    let sz = (scale * p.z).sin();
    let cx = (scale * p.x).cos();
    let cy = (scale * p.y).cos();
    let cz = (scale * p.z).cos();
    let g = sz.mul_add(cx, sx.mul_add(cy, sy * cz));
    g.abs() - thickness
}

/// Schwarz P triply-periodic minimal surface.
/// Field: `|cos(sx) + cos(sy) + cos(sz)| - thickness`
///
/// Gradient (for Session 19):
///   Let `g = cos(sx) + cos(sy) + cos(sz)`.
///   `dg/dx = -s·sin(sx)`, `dg/dy = -s·sin(sy)`, `dg/dz = -s·sin(sz)`.
///   Gradient of field = `sign(g) · ∇g`
fn eval_schwarz_p(scale: f64, thickness: f64, p: &Point3<f64>) -> f64 {
    let g = (scale * p.x).cos() + (scale * p.y).cos() + (scale * p.z).cos();
    g.abs() - thickness
}

/// Helix tube: distance to helix curve `H(t) = (R·cos(2πt), R·sin(2πt), P·t)`
/// minus thickness. Uses cylindrical nearest-turn lookup + Newton refinement.
///
/// Gradient (for Session 19):
///   `∇field = (p - H(t*)) / |p - H(t*)|` where `t*` is the closest parameter.
///   This is the unit direction from nearest helix point to the query point.
// Index/count conversion bounded by domain (size well below 2^32).
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn eval_helix(radius: f64, pitch: f64, thickness: f64, turns: f64, p: &Point3<f64>) -> f64 {
    use std::f64::consts::TAU;

    let theta = p.y.atan2(p.x); // [-π, π]

    // The helix at angle (θ + 2πk) has z = pitch·(θ/(2π) + k).
    // Find k such that the helix z is closest to p.z.
    let k_continuous = if pitch.abs() > 1e-15 {
        (p.z / pitch) - theta / TAU
    } else {
        0.0
    };
    let k_base = k_continuous.round() as i64;

    let tau_sq = TAU * TAU;
    let mut min_d_sq = f64::INFINITY;

    // Check 3 nearest turns, Newton-refine each
    for dk in -1..=1_i64 {
        let ki = k_base + dk;
        let t_initial = (theta / TAU + ki as f64).clamp(0.0, turns);

        // Newton refinement: minimize |H(t) - p|²
        // H(t) = (R·cos(τt), R·sin(τt), P·t)
        // H'(t) = (-Rτ·sin(τt), Rτ·cos(τt), P)
        // H''(t) = (-Rτ²·cos(τt), -Rτ²·sin(τt), 0)
        let mut t = t_initial;
        for _ in 0..4 {
            let alpha = TAU * t;
            let ca = alpha.cos();
            let sa = alpha.sin();

            let hx = radius * ca;
            let hy = radius * sa;
            let hz = pitch * t;

            // First derivative of helix
            let vel_x = -radius * TAU * sa;
            let vel_y = radius * TAU * ca;
            let vel_z = pitch;

            // Second derivative of helix
            let acc_x = -radius * tau_sq * ca;
            let acc_y = -radius * tau_sq * sa;
            // acc_z = 0

            let ex = hx - p.x;
            let ey = hy - p.y;
            let ez = hz - p.z;

            let fp = 2.0 * ez.mul_add(vel_z, ex.mul_add(vel_x, ey * vel_y));
            // f''(t) = 2·(|H'|² + (H-p)·H'')
            let vel_sq = vel_z.mul_add(vel_z, vel_x.mul_add(vel_x, vel_y * vel_y));
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
        min_d_sq = min_d_sq.min(d_sq);
    }

    min_d_sq.sqrt() - thickness
}

/// Pipe along a polyline: min distance to any segment minus radius. Exact SDF.
fn eval_pipe(vertices: &[Point3<f64>], radius: f64, p: &Point3<f64>) -> f64 {
    let mut min_dist_sq = f64::INFINITY;
    for seg in vertices.windows(2) {
        let closest = cf_geometry::closest_point_segment(seg[0], seg[1], *p);
        let d_sq = nalgebra::distance_squared(p, &closest);
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
        }
    }
    min_dist_sq.sqrt() - radius
}

/// Pipe along a Catmull-Rom spline: distance to closest point on spline minus
/// radius. Near-exact SDF via subdivision + Newton refinement.
fn eval_pipe_spline(control_points: &[Point3<f64>], radius: f64, p: &Point3<f64>) -> f64 {
    let n = control_points.len();
    let mut min_dist_sq = f64::INFINITY;

    // For 2 control points: degenerate to single line segment
    if n == 2 {
        let closest = cf_geometry::closest_point_segment(control_points[0], control_points[1], *p);
        return nalgebra::distance_squared(p, &closest).sqrt() - radius;
    }

    // Catmull-Rom: each span uses 4 control points (p0, p1, p2, p3).
    // We iterate spans from index 0..n-3 (span i uses points i, i+1, i+2, i+3).
    // For open curves: first span repeats the first point, last span repeats the last.
    let num_spans = n - 1;
    for span in 0..num_spans {
        let p0 = control_points[if span == 0 { 0 } else { span - 1 }];
        let p1 = control_points[span];
        let p2 = control_points[span + 1];
        let p3 = control_points[if span + 2 < n { span + 2 } else { n - 1 }];

        // Coarse search: subdivide span into 16 linear segments
        let subdivs: u32 = 16;
        let mut best_t = 0.0_f64;
        let mut best_d_sq = f64::INFINITY;

        for i in 0..=subdivs {
            let t = f64::from(i) / f64::from(subdivs);
            let q = catmull_rom_point(p0, p1, p2, p3, t);
            let d_sq = nalgebra::distance_squared(p, &q);
            if d_sq < best_d_sq {
                best_d_sq = d_sq;
                best_t = t;
            }
        }

        // Newton refinement on distance²(t): minimize f(t) = |C(t) - p|²
        // f'(t) = 2 * dot(C(t) - p, C'(t))
        // f''(t) = 2 * (dot(C'(t), C'(t)) + dot(C(t) - p, C''(t)))
        // Newton step: t -= f'(t) / f''(t)
        let mut t = best_t;
        for _ in 0..4 {
            let c = catmull_rom_point(p0, p1, p2, p3, t);
            let c_d = catmull_rom_deriv(p0, p1, p2, p3, t);
            let c_dd = catmull_rom_deriv2(p0, p1, p2, p3, t);
            let diff = c - p;
            let fp = 2.0 * diff.dot(&c_d);
            let fpp = 2.0 * (c_d.dot(&c_d) + diff.dot(&c_dd));
            if fpp.abs() < 1e-20 {
                break;
            }
            t -= fp / fpp;
            t = t.clamp(0.0, 1.0);
        }

        let c = catmull_rom_point(p0, p1, p2, p3, t);
        let d_sq = nalgebra::distance_squared(p, &c);
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
        }
    }

    min_dist_sq.sqrt() - radius
}

// ── Loft helpers ──────────────────────────────────────────────────────────

/// Loft along Z: cylinder-like SDF with cubic-interpolated radius.
fn eval_loft(stations: &[[f64; 2]], p: &Point3<f64>) -> f64 {
    let z_min = stations[0][0];
    let z_max = stations[stations.len() - 1][0];
    let r_xy = p.x.hypot(p.y);

    let z_clamped = p.z.clamp(z_min, z_max);
    let r_at_z = loft_radius_at(stations, z_clamped);

    let d_radial = r_xy - r_at_z;
    let d_axial = (z_min - p.z).max(p.z - z_max);

    // Cylinder-like exact SDF formula
    let outside = d_radial.max(0.0).hypot(d_axial.max(0.0));
    let inside = d_radial.max(d_axial).min(0.0);
    outside + inside
}

/// Catmull-Rom cubic interpolation of radius along a loft profile.
///
/// `stations` is `[[z, radius], ...]` sorted by z, with at least 2 entries.
/// Returns the interpolated radius at the given z (clamped to station range).
fn loft_radius_at(stations: &[[f64; 2]], z: f64) -> f64 {
    let n = stations.len();
    debug_assert!(n >= 2);

    let z_clamped = z.clamp(stations[0][0], stations[n - 1][0]);

    // Find span: last station where z_clamped >= station z
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

    // Catmull-Rom 4-point stencil (endpoint repetition for boundaries)
    let r_m1 = stations[span.saturating_sub(1)][1];
    let r0 = stations[span][1];
    let r1 = stations[span + 1][1];
    let r2 = stations[(span + 2).min(n - 1)][1];

    catmull_rom_1d(r_m1, r0, r1, r2, t)
}

/// 1D Catmull-Rom interpolation. Same basis as `catmull_rom_point`.
// Short names mirror textbook / paper notation.
#[allow(clippy::many_single_char_names)]
fn catmull_rom_1d(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;
    // C(t) = 0.5 * (2P1 + (-P0+P2)t + (2P0-5P1+4P2-P3)t² + (-P0+3P1-3P2+P3)t³)
    let linear = 2.0f64.mul_add(p1, (-p0 + p2) * t);
    let quad = 2.0f64.mul_add(p0, 4.0f64.mul_add(p2, -(5.0 * p1) - p3)) * t2;
    let cubic = 3.0f64.mul_add(p1, 3.0f64.mul_add(-p2, -p0 + p3)) * t3;
    0.5 * (linear + quad + cubic)
}

// ── Catmull-Rom helpers ──────────────────────────────────────────────────

/// Catmull-Rom spline evaluation at parameter t ∈ [0, 1].
///
/// Uses the standard matrix form with α = 0.5 (centripetal is typical, but
/// uniform α=0.5 is the standard Catmull-Rom used in graphics).
fn catmull_rom_point(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Point3<f64> {
    let t2 = t * t;
    let t3 = t2 * t;
    // C(t) = 0.5 * ((2*P1) + (-P0 + P2)*t + (2*P0 - 5*P1 + 4*P2 - P3)*t² + (-P0 + 3*P1 - 3*P2 + P3)*t³)
    let c = (p1.coords * 2.0
        + (-p0.coords + p2.coords) * t
        + (p0.coords * 2.0 - p1.coords * 5.0 + p2.coords * 4.0 - p3.coords) * t2
        + (-p0.coords + p1.coords * 3.0 - p2.coords * 3.0 + p3.coords) * t3)
        * 0.5;
    Point3::from(c)
}

/// First derivative of Catmull-Rom spline at parameter t.
fn catmull_rom_deriv(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> nalgebra::Vector3<f64> {
    let t2 = t * t;
    // C'(t) = 0.5 * ((-P0 + P2) + (4*P0 - 10*P1 + 8*P2 - 2*P3)*t + (-3*P0 + 9*P1 - 9*P2 + 3*P3)*t²)
    ((-p0.coords + p2.coords)
        + (p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0) * t
        + (-p0.coords * 3.0 + p1.coords * 9.0 - p2.coords * 9.0 + p3.coords * 3.0) * t2)
        * 0.5
}

/// Second derivative of Catmull-Rom spline at parameter t.
fn catmull_rom_deriv2(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> nalgebra::Vector3<f64> {
    // C''(t) = 0.5 * ((4*P0 - 10*P1 + 8*P2 - 2*P3) + (-6*P0 + 18*P1 - 18*P2 + 6*P3)*t)
    ((p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0)
        + (-p0.coords * 6.0 + p1.coords * 18.0 - p2.coords * 18.0 + p3.coords * 6.0) * t)
        * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field_node::Val;
    use approx::assert_abs_diff_eq;
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    const EPS: f64 = 1e-10;

    // ── Sphere ───────────────────────────────────────────────────────

    #[test]
    fn sphere_origin_is_negative_radius() {
        let s = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
    }

    #[test]
    fn sphere_on_surface_is_zero() {
        let s = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 2.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 0.0, 2.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn sphere_outside_is_positive() {
        let s = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    // ── Cuboid ───────────────────────────────────────────────────────

    #[test]
    fn cuboid_origin_is_negative() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 2.0, 3.0),
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -1.0, epsilon = EPS);
    }

    #[test]
    fn cuboid_on_face_is_zero() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 2.0, 3.0),
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 2.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn cuboid_corner_distance() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 1.0, 1.0),
        };
        // Point at (2, 2, 2) — distance to corner (1,1,1) = sqrt(3)
        let d = c.evaluate(&Point3::new(2.0, 2.0, 2.0));
        assert_abs_diff_eq!(d, 3.0_f64.sqrt(), epsilon = EPS);
    }

    // ── Cylinder ─────────────────────────────────────────────────────

    #[test]
    fn cylinder_origin_is_negative() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
    }

    #[test]
    fn cylinder_on_barrel_is_zero() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn cylinder_on_cap_is_zero() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
    }

    // ── Capsule ──────────────────────────────────────────────────────

    #[test]
    fn capsule_origin_is_negative_radius() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -1.0, epsilon = EPS);
    }

    #[test]
    fn capsule_on_barrel_is_zero() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn capsule_on_cap_is_zero() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        // Top of capsule: z = half_height + radius = 3.0
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
    }

    // ── Ellipsoid ────────────────────────────────────────────────────

    #[test]
    fn ellipsoid_origin_is_negative() {
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(2.0, 3.0, 4.0),
        };
        assert!(e.evaluate(&Point3::origin()) < 0.0);
    }

    #[test]
    fn ellipsoid_on_surface_is_near_zero() {
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(2.0, 3.0, 4.0),
        };
        // On-axis points are on the surface
        assert_abs_diff_eq!(e.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(e.evaluate(&Point3::new(0.0, 3.0, 0.0)), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(e.evaluate(&Point3::new(0.0, 0.0, 4.0)), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn ellipsoid_sphere_equivalence() {
        // When all radii are equal, should match sphere SDF
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(3.0, 3.0, 3.0),
        };
        let s = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let test_points = [
            Point3::origin(),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(e.evaluate(p), s.evaluate(p), epsilon = 1e-6);
        }
    }

    // ── Torus ────────────────────────────────────────────────────────

    #[test]
    fn torus_center_of_tube_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        // Point on the ring at (5, 0, 0) should be on tube center -> d = -minor
        assert_abs_diff_eq!(t.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);
    }

    #[test]
    fn torus_on_outer_surface_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        assert_abs_diff_eq!(t.evaluate(&Point3::new(6.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn torus_on_inner_surface_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        assert_abs_diff_eq!(t.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn torus_origin_is_positive() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        // Origin: q_xy = sqrt(0) - 5 = -5, dist = sqrt(25 + 0) - 1 = 4
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 4.0, epsilon = EPS);
    }

    // ── Cone ─────────────────────────────────────────────────────────

    #[test]
    fn cone_apex_is_zero() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), 0.0, epsilon = EPS);
    }

    #[test]
    fn cone_base_center_is_negative() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        // Base center at (0, 0, -4) — should be inside (negative)
        assert!(c.evaluate(&Point3::new(0.0, 0.0, -4.0)) <= 0.0);
    }

    #[test]
    fn cone_outside_is_positive() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert!(c.evaluate(&Point3::new(5.0, 0.0, -2.0)) > 0.0);
    }

    #[test]
    fn cone_above_apex_is_positive() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert!(c.evaluate(&Point3::new(0.0, 0.0, 1.0)) > 0.0);
    }

    // ── Plane ────────────────────────────────────────────────────────

    #[test]
    fn plane_on_surface_is_zero() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(5.0, 3.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn plane_above_is_positive() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert!(pl.evaluate(&Point3::new(0.0, 0.0, 1.0)) > 0.0);
    }

    #[test]
    fn plane_below_is_negative() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert!(pl.evaluate(&Point3::new(0.0, 0.0, -1.0)) < 0.0);
    }

    #[test]
    fn plane_with_offset() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 3.0,
        };
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(0.0, 0.0, 5.0)), 2.0, epsilon = EPS);
    }

    // ── Union ───────────────────────────────────────────────────────

    #[test]
    fn union_picks_smaller_value() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        // At origin: min(-2, -3) = -3
        assert_abs_diff_eq!(u.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
    }

    #[test]
    fn union_is_commutative() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let u1 = FieldNode::Union(Box::new(a.clone()), Box::new(b.clone()));
        let u2 = FieldNode::Union(Box::new(b), Box::new(a));
        let p = Point3::new(2.5, 0.0, 0.0);
        assert_abs_diff_eq!(u1.evaluate(&p), u2.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn union_inside_either_is_inside() {
        // Two non-overlapping spheres
        let a = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(-3.0, 0.0, 0.0),
        );
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        assert!(u.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0); // inside a
        assert!(u.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0); // inside b
        assert!(u.evaluate(&Point3::new(0.0, 0.0, 0.0)) > 0.0); // outside both
    }

    // ── Subtract ────────────────────────────────────────────────────

    #[test]
    fn subtract_removes_second() {
        // Big sphere minus small sphere at origin
        let big = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let small = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let sub = FieldNode::Subtract(Box::new(big), Box::new(small));
        // Origin: inside big (-5) but inside small (-2), so max(-5, 2) = 2 > 0 (outside)
        assert!(sub.evaluate(&Point3::origin()) > 0.0);
        // On surface of small sphere (just outside): inside big, outside small
        assert!(sub.evaluate(&Point3::new(2.5, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn subtract_is_not_commutative() {
        let a = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let sub_ab = FieldNode::Subtract(Box::new(a.clone()), Box::new(b.clone()));
        let sub_ba = FieldNode::Subtract(Box::new(b), Box::new(a));
        let p = Point3::new(3.0, 0.0, 0.0);
        // a-b at (3,0,0): max(-2, 2) = 2... wait let me recalculate.
        // a = sphere(5): eval at (3,0,0) = 3-5 = -2
        // b = sphere(2): eval at (3,0,0) = 3-2 = 1
        // a-b: max(-2, -1) = -1 (inside)
        // b-a: max(1, 2) = 2 (outside)
        assert!(sub_ab.evaluate(&p) < 0.0);
        assert!(sub_ba.evaluate(&p) > 0.0);
    }

    // ── Intersect ───────────────────────────────────────────────────

    #[test]
    fn intersect_requires_both_inside() {
        let a = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let inter = FieldNode::Intersect(Box::new(a), Box::new(b));
        // At origin: max(-5, -2) = -2 (inside both)
        assert_abs_diff_eq!(inter.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        // At (3,0,0): max(-2, 1) = 1 (outside small sphere)
        assert!(inter.evaluate(&Point3::new(3.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn intersect_is_commutative() {
        let a = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let i1 = FieldNode::Intersect(Box::new(a.clone()), Box::new(b.clone()));
        let i2 = FieldNode::Intersect(Box::new(b), Box::new(a));
        let p = Point3::new(1.0, 1.0, 0.0);
        assert_abs_diff_eq!(i1.evaluate(&p), i2.evaluate(&p), epsilon = EPS);
    }

    // ── Smooth Union ────────────────────────────────────────────────

    #[test]
    fn smooth_union_adds_material_in_blend_region() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let k = 1.0;
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), Val::from(k));
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        // In the blend region between the two spheres, smooth union should
        // produce a smaller (more negative or less positive) value than sharp
        let p = Point3::new(1.5, 0.0, 0.0);
        assert!(su.evaluate(&p) <= sharp.evaluate(&p) + EPS);
    }

    #[test]
    fn smooth_union_far_from_blend_matches_sharp() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(10.0, 0.0, 0.0),
        );
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), Val::from(0.5));
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        // Far from blend region: should be nearly identical
        let p = Point3::origin();
        assert_abs_diff_eq!(su.evaluate(&p), sharp.evaluate(&p), epsilon = 1e-6);
    }

    #[test]
    fn smooth_union_max_correction_is_k_over_4() {
        // When a ≈ b, correction is at most k/4
        let a = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(3.0),
        }; // identical
        let k = 2.0;
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), Val::from(k));
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        let p = Point3::origin();
        // sharp = -3, smooth should be -3 - k/4 = -3.5
        let diff = sharp.evaluate(&p) - su.evaluate(&p);
        assert!(diff >= -EPS, "smooth union should not exceed sharp union");
        assert!(diff <= k / 4.0 + EPS, "correction should be at most k/4");
    }

    // ── Smooth Subtract ─────────────────────────────────────────────

    #[test]
    fn smooth_subtract_removes_with_blend() {
        let big = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let small = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let ss = FieldNode::SmoothSubtract(Box::new(big), Box::new(small), Val::from(1.0));
        // Origin: big is deep inside (-5), small is deep inside (-2).
        // Sharp subtract: max(-5, 2) = 2. Smooth should be close.
        assert!(ss.evaluate(&Point3::origin()) > 0.0);
        // Far from subtraction zone: should still be inside
        assert!(ss.evaluate(&Point3::new(4.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn smooth_subtract_degenerates_to_sharp_when_far() {
        let big = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let small = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(0.0, 0.0, 0.0),
        );
        let k = 0.1;
        let ss =
            FieldNode::SmoothSubtract(Box::new(big.clone()), Box::new(small.clone()), Val::from(k));
        let sharp = FieldNode::Subtract(Box::new(big), Box::new(small));
        // Far from blend region
        let p = Point3::new(4.0, 0.0, 0.0);
        assert_abs_diff_eq!(ss.evaluate(&p), sharp.evaluate(&p), epsilon = 1e-3);
    }

    // ── Smooth Intersect ────────────────────────────────────────────

    #[test]
    fn smooth_intersect_removes_material_in_blend() {
        let a = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let b = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let k = 2.0;
        let si = FieldNode::SmoothIntersect(Box::new(a.clone()), Box::new(b.clone()), Val::from(k));
        let sharp = FieldNode::Intersect(Box::new(a), Box::new(b));
        // Smooth intersect removes material: field value should be ≥ sharp
        let p = Point3::origin();
        assert!(si.evaluate(&p) >= sharp.evaluate(&p) - EPS);
    }

    #[test]
    fn smooth_intersect_matches_sharp_when_far() {
        let a = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let b = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 2.0, 2.0),
        };
        let k = 0.1;
        let si = FieldNode::SmoothIntersect(Box::new(a.clone()), Box::new(b.clone()), Val::from(k));
        let sharp = FieldNode::Intersect(Box::new(a), Box::new(b));
        // Deep inside intersection
        let p = Point3::origin();
        assert_abs_diff_eq!(si.evaluate(&p), sharp.evaluate(&p), epsilon = 0.1);
    }

    // ── Smooth Union All ────────────────────────────────────────────

    #[test]
    fn smooth_union_all_single_element() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let sua = FieldNode::SmoothUnionAll(vec![a.clone()], Val::from(1.0));
        let p = Point3::origin();
        assert_abs_diff_eq!(sua.evaluate(&p), a.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn smooth_union_all_is_order_independent() {
        let sa = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let sb = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let sc = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(0.0, 3.0, 0.0),
        );
        let blend = 1.0;
        let abc =
            FieldNode::SmoothUnionAll(vec![sa.clone(), sb.clone(), sc.clone()], Val::from(blend));
        let bca =
            FieldNode::SmoothUnionAll(vec![sb.clone(), sc.clone(), sa.clone()], Val::from(blend));
        let cab = FieldNode::SmoothUnionAll(vec![sc, sa, sb], Val::from(blend));
        let pt = Point3::new(1.0, 1.0, 0.0);
        assert_abs_diff_eq!(abc.evaluate(&pt), bca.evaluate(&pt), epsilon = EPS);
        assert_abs_diff_eq!(abc.evaluate(&pt), cab.evaluate(&pt), epsilon = EPS);
    }

    #[test]
    fn smooth_union_all_adds_material() {
        let a = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let k = 1.0;
        let sua = FieldNode::SmoothUnionAll(vec![a.clone(), b.clone()], Val::from(k));
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        let p = Point3::new(1.5, 0.0, 0.0);
        assert!(sua.evaluate(&p) <= sharp.evaluate(&p) + EPS);
    }

    // ── Translate ───────────────────────────────────────────────────

    #[test]
    fn translate_moves_shape() {
        let s = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let t = FieldNode::Translate(Box::new(s), Vector3::new(5.0, 0.0, 0.0));
        // Center of translated sphere is at (5, 0, 0)
        assert_abs_diff_eq!(t.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);
        // Origin should be 4 units from surface
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 4.0, epsilon = EPS);
    }

    #[test]
    fn translate_preserves_sdf() {
        let s = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let t = FieldNode::Translate(Box::new(s), Vector3::new(3.0, 4.0, 0.0));
        // Distance from origin to center (3,4,0) is 5. SDF = 5 - 2 = 3.
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 3.0, epsilon = EPS);
    }

    // ── Rotate ──────────────────────────────────────────────────────

    #[test]
    fn rotate_90_degrees_around_z() {
        // Cuboid at origin, 1x2x1 half-extents. Rotate 90° around Z.
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 1.0),
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let r = FieldNode::Rotate(Box::new(c), rot);
        // After 90° rotation: half_extents become (2, 1, 1) in world space.
        // Point (1.5, 0, 0): was outside x-extent (1.0), now inside rotated x-extent (2.0)
        assert!(r.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        // Point (0, 1.5, 0): was inside y-extent (2.0), now outside rotated y-extent (1.0)
        assert!(r.evaluate(&Point3::new(0.0, 1.5, 0.0)) > 0.0);
    }

    #[test]
    fn rotate_preserves_sphere() {
        // Sphere is rotationally symmetric
        let s = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 1.234);
        let r = FieldNode::Rotate(Box::new(s.clone()), rot);
        let test_points = [
            Point3::origin(),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(5.0, 0.0, 0.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(r.evaluate(p), s.evaluate(p), epsilon = EPS);
        }
    }

    // ── Scale Uniform ───────────────────────────────────────────────

    #[test]
    fn scale_doubles_size() {
        let s = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let scaled = FieldNode::ScaleUniform(Box::new(s), 2.0);
        // Effective radius is 2.0
        assert_abs_diff_eq!(scaled.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(2.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(3.0, 0.0, 0.0)),
            1.0,
            epsilon = EPS
        );
    }

    #[test]
    fn scale_preserves_sdf_property() {
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let scaled = FieldNode::ScaleUniform(Box::new(c), 3.0);
        // Effective half-extents are (3, 3, 3)
        assert_abs_diff_eq!(scaled.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(3.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
    }

    // ── Mirror ──────────────────────────────────────────────────────

    #[test]
    fn mirror_x_reflects_across_yz_plane() {
        // Sphere at (3, 0, 0), mirrored across YZ plane
        let s = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let m = FieldNode::Mirror(Box::new(s), Vector3::new(1.0, 0.0, 0.0));
        // Point at (3, 0, 0): positive side, should be inside sphere center
        assert_abs_diff_eq!(m.evaluate(&Point3::new(3.0, 0.0, 0.0)), -1.0, epsilon = EPS);
        // Point at (-3, 0, 0): reflected to (3, 0, 0), should also be inside
        assert_abs_diff_eq!(
            m.evaluate(&Point3::new(-3.0, 0.0, 0.0)),
            -1.0,
            epsilon = EPS
        );
        // Origin: reflected to origin, which is 2 units from sphere center
        assert_abs_diff_eq!(m.evaluate(&Point3::origin()), 2.0, epsilon = EPS);
    }

    #[test]
    fn mirror_positive_side_unchanged() {
        let s = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let m = FieldNode::Mirror(Box::new(s.clone()), Vector3::new(1.0, 0.0, 0.0));
        // Points on the positive x side should evaluate identically to the original
        let positive_points = [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ];
        for p in &positive_points {
            assert_abs_diff_eq!(m.evaluate(p), s.evaluate(p), epsilon = EPS);
        }
    }

    // ── Composed trees ──────────────────────────────────────────────

    #[test]
    fn union_of_translated_spheres() {
        let a = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(-2.0, 0.0, 0.0),
        );
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        assert!(u.evaluate(&Point3::new(-2.0, 0.0, 0.0)) < 0.0);
        assert!(u.evaluate(&Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(u.evaluate(&Point3::origin()) > 0.0);
    }

    #[test]
    fn subtract_then_translate() {
        let big = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let hole = FieldNode::Sphere {
            radius: Val::from(2.0),
        };
        let hollowed = FieldNode::Subtract(Box::new(big), Box::new(hole));
        let moved = FieldNode::Translate(Box::new(hollowed), Vector3::new(10.0, 0.0, 0.0));
        // Center of moved hollow sphere is at (10, 0, 0) — hole is there
        assert!(moved.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0);
        // Shell region
        assert!(moved.evaluate(&Point3::new(13.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn rotate_then_scale() {
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 1.0),
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let rotated = FieldNode::Rotate(Box::new(c), rot);
        let scaled = FieldNode::ScaleUniform(Box::new(rotated), 2.0);
        // Original: 1x2x1. After 90° Z rotation: 2x1x1. After 2x scale: 4x2x2.
        assert!(scaled.evaluate(&Point3::new(3.5, 0.0, 0.0)) < 0.0);
        assert!(scaled.evaluate(&Point3::new(4.5, 0.0, 0.0)) > 0.0);
        assert!(scaled.evaluate(&Point3::new(0.0, 1.5, 0.0)) < 0.0);
        assert!(scaled.evaluate(&Point3::new(0.0, 2.5, 0.0)) > 0.0);
    }

    // ── Shell ──────────────────────────────────────────────────────────

    #[test]
    fn shell_sphere_wall_thickness() {
        // Shell a sphere of radius 5 with thickness 1.
        // Field = |dist| - thickness = ||p|-5| - 1.
        // At origin: ||0|-5|-1 = |5|-1 = 4 (inside the wall? No, 4 > 0 = outside wall)
        // Wait: origin is deep inside the sphere: dist = -5.
        // |dist| = 5, shell = 5 - 1 = 4. Positive = outside the shell wall. Correct.
        let s = FieldNode::Shell(
            Box::new(FieldNode::Sphere {
                radius: Val::from(5.0),
            }),
            Val::from(1.0),
        );
        assert!(
            s.evaluate(&Point3::origin()) > 0.0,
            "origin should be outside shell"
        );

        // On the original surface (r=5): |0| - 1 = -1 (inside wall)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);

        // At r=4 (inner surface): |(-1)| - 1 = 0
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At r=6 (outer surface): |1| - 1 = 0
        assert_abs_diff_eq!(s.evaluate(&Point3::new(6.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At r=4.5 (inside wall, inner side): |(-0.5)| - 1 = -0.5
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.5, 0.0, 0.0)), -0.5, epsilon = EPS);

        // At r=5.5 (inside wall, outer side): |0.5| - 1 = -0.5
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.5, 0.0, 0.0)), -0.5, epsilon = EPS);
    }

    // ── Round ──────────────────────────────────────────────────────────

    #[test]
    fn round_cuboid_shifts_surface() {
        // Round a unit cube by 0.2 — surface moves inward by 0.2.
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let r = FieldNode::Round(Box::new(c), Val::from(0.2));

        // Origin: cuboid SDF = -1.0, rounded = -1.0 - 0.2 = -1.2
        assert_abs_diff_eq!(r.evaluate(&Point3::origin()), -1.2, epsilon = EPS);

        // On the original face (1,0,0): cuboid SDF = 0, rounded = -0.2 (now inside)
        assert_abs_diff_eq!(r.evaluate(&Point3::new(1.0, 0.0, 0.0)), -0.2, epsilon = EPS);

        // At (1.2, 0, 0): cuboid SDF = 0.2, rounded = 0 (new surface)
        assert_abs_diff_eq!(r.evaluate(&Point3::new(1.2, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── Offset ─────────────────────────────────────────────────────────

    #[test]
    fn offset_grow_sphere() {
        // Offset sphere(3) by +1 → effective radius 4.
        let s = FieldNode::Offset(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Val::from(1.0),
        );
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -4.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn offset_shrink_sphere() {
        // Offset sphere(3) by -1 → effective radius 2.
        let s = FieldNode::Offset(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Val::from(-1.0),
        );
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── Elongate ──────────────────────────────────────────────────────

    #[test]
    fn elongate_sphere_becomes_capsule_like() {
        // Elongate a sphere(1) by (2, 0, 0) → stretches along X.
        // At (0,0,0): q = (0,0,0) - clamp(0, -2, 2) = 0, sphere(0) = -1
        let s = FieldNode::Elongate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -1.0, epsilon = EPS);

        // At (2,0,0): q = (2-2,0,0) = (0,0,0), sphere(0) = -1
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), -1.0, epsilon = EPS);

        // At (3,0,0): q = (3-2,0,0) = (1,0,0), sphere = |1|-1 = 0 (surface)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(3.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At (-3,0,0): q = (-3-(-2),0,0) = (-1,0,0), sphere = 0 (surface)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(-3.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // Y direction not elongated: at (0,1,0) → surface
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 1.0, 0.0)), 0.0, epsilon = EPS);

        // At (0,2,0): q = (0,2,0) → sphere = 2-1 = 1 (outside)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 2.0, 0.0)), 1.0, epsilon = EPS);
    }

    #[test]
    fn elongate_preserves_sdf() {
        // Elongated sphere: surface at x=±(h+r), y/z=±r
        let s = FieldNode::Elongate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(2.0),
            }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        // Surface at x = ±5 (elongation 3 + radius 2)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(-5.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── UserFn ─────────────────────────────────────────────────────────

    #[test]
    fn user_fn_evaluates_closure() {
        use crate::field_node::UserEvalFn;
        use cf_geometry::Aabb;
        use std::sync::Arc;

        // Custom sphere of radius 4 via UserFn
        let node = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 4.0)),
            interval: None,
            bounds: Aabb::new(Point3::new(-5.0, -5.0, -5.0), Point3::new(5.0, 5.0, 5.0)),
        };

        assert_abs_diff_eq!(node.evaluate(&Point3::origin()), -4.0, epsilon = EPS);
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(4.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
        assert!(node.evaluate(&Point3::new(6.0, 0.0, 0.0)) > 0.0);
    }

    // ── Pipe ──────────────────────────────────────────────────────────

    #[test]
    fn pipe_straight_matches_capsule() {
        // Straight pipe from (0,0,-2) to (0,0,2) with radius 1 should match
        // a capsule with half_height=2, radius=1.
        let pipe = FieldNode::Pipe {
            vertices: vec![Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, 2.0)],
            radius: 1.0,
        };
        let capsule = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        let test_points = [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.0, 0.0, -3.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 1.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(pipe.evaluate(p), capsule.evaluate(p), epsilon = 1e-10);
        }
    }

    #[test]
    fn pipe_l_shaped_inside_outside() {
        // L-shaped pipe: (0,0,0) → (5,0,0) → (5,5,0), radius 1
        let pipe = FieldNode::Pipe {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            radius: 1.0,
        };
        // Center of first segment
        assert!(pipe.evaluate(&Point3::new(2.5, 0.0, 0.0)) < 0.0);
        // Center of second segment
        assert!(pipe.evaluate(&Point3::new(5.0, 2.5, 0.0)) < 0.0);
        // Corner vertex — exactly at radius distance from corner
        assert_abs_diff_eq!(
            pipe.evaluate(&Point3::new(5.0, 0.0, 0.0)),
            -1.0,
            epsilon = 1e-10
        );
        // Far outside
        assert!(pipe.evaluate(&Point3::new(2.5, 5.0, 0.0)) > 0.0);
        // On surface of first segment
        assert_abs_diff_eq!(
            pipe.evaluate(&Point3::new(2.5, 1.0, 0.0)),
            0.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn pipe_multi_segment_continuity() {
        // 4-vertex pipe: zigzag pattern
        let pipe = FieldNode::Pipe {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
                Point3::new(3.0, 3.0, 0.0),
                Point3::new(6.0, 3.0, 0.0),
            ],
            radius: 0.5,
        };
        // Each segment center should be inside
        assert!(pipe.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        assert!(pipe.evaluate(&Point3::new(3.0, 1.5, 0.0)) < 0.0);
        assert!(pipe.evaluate(&Point3::new(4.5, 3.0, 0.0)) < 0.0);
        // At vertices (joints) — should be inside (negative radius)
        assert_abs_diff_eq!(
            pipe.evaluate(&Point3::new(3.0, 0.0, 0.0)),
            -0.5,
            epsilon = 1e-10
        );
        assert_abs_diff_eq!(
            pipe.evaluate(&Point3::new(3.0, 3.0, 0.0)),
            -0.5,
            epsilon = 1e-10
        );
    }

    // ── PipeSpline ────────────────────────────────────────────────────

    #[test]
    fn pipe_spline_two_points_matches_capsule() {
        // 2-point spline degenerates to a line segment (capsule)
        let spline = FieldNode::PipeSpline {
            control_points: vec![Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, 2.0)],
            radius: 1.0,
        };
        let capsule = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        let test_points = [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(spline.evaluate(p), capsule.evaluate(p), epsilon = 1e-10);
        }
    }

    #[test]
    fn pipe_spline_smooth_curvature() {
        // Curved spline: quarter-circle-ish path
        let spline = FieldNode::PipeSpline {
            control_points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            radius: 0.5,
        };
        // Start point should be inside
        assert!(spline.evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0);
        // End point should be inside
        assert!(spline.evaluate(&Point3::new(5.0, 5.0, 0.0)) < 0.0);
        // Midpoint of spline — somewhere near (5, 0) to (5, 5) curve
        // The spline passes through control points, so point near the
        // curve should be inside
        assert!(spline.evaluate(&Point3::new(5.0, 2.5, 0.0)) < 0.0);
        // Far away should be outside
        assert!(spline.evaluate(&Point3::new(0.0, 5.0, 0.0)) > 0.0);
    }

    #[test]
    fn pipe_spline_inner_outer_radii() {
        // Straight-ish spline, check surface at radius distance
        let spline = FieldNode::PipeSpline {
            control_points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            radius: 1.0,
        };
        // On the axis at midpoint: should be -radius
        assert_abs_diff_eq!(
            spline.evaluate(&Point3::new(5.0, 0.0, 0.0)),
            -1.0,
            epsilon = 1e-6
        );
        // At radius distance from axis: should be ~0
        assert_abs_diff_eq!(
            spline.evaluate(&Point3::new(5.0, 1.0, 0.0)),
            0.0,
            epsilon = 1e-6
        );
        // Outside
        assert!(spline.evaluate(&Point3::new(5.0, 2.0, 0.0)) > 0.0);
    }

    // ── Loft ────────────────────────────────────────────────────────

    #[test]
    fn loft_constant_radius_matches_cylinder() {
        // Constant radius loft should behave like a cylinder
        let loft = FieldNode::Loft {
            stations: vec![[-5.0, 2.0], [5.0, 2.0]],
        };
        let cyl = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 5.0,
        };
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 7.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(loft.evaluate(p), cyl.evaluate(p), epsilon = 1e-6);
        }
    }

    #[test]
    fn loft_tapered_sign_correctness() {
        // Taper from radius 3 at z=-2 to radius 1 at z=2
        let loft = FieldNode::Loft {
            stations: vec![[-2.0, 3.0], [2.0, 1.0]],
        };
        // Origin: z=0, R(0) = catmull_rom(3,3,1,1, 0.5) = 2.0
        // r_xy=0, d_radial = -2.0, d_axial = -2.0, field = -2.0
        assert!(loft.evaluate(&Point3::origin()) < 0.0, "center inside");

        // On barrel at z=-2 (bottom): R=3, point at (3, 0, -2) → on surface
        assert_abs_diff_eq!(
            loft.evaluate(&Point3::new(3.0, 0.0, -2.0)),
            0.0,
            epsilon = 1e-6
        );

        // On barrel at z=2 (top): R=1, point at (1, 0, 2) → on surface
        assert_abs_diff_eq!(
            loft.evaluate(&Point3::new(1.0, 0.0, 2.0)),
            0.0,
            epsilon = 1e-6
        );

        // Outside far away
        assert!(loft.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn loft_midpoint_radius_interpolation() {
        // 3 stations: [-3, r=1], [0, r=2], [3, r=1]
        let loft = FieldNode::Loft {
            stations: vec![[-3.0, 1.0], [0.0, 2.0], [3.0, 1.0]],
        };
        // At z=0, radius = 2.0 (exact station)
        assert_abs_diff_eq!(
            loft.evaluate(&Point3::new(2.0, 0.0, 0.0)),
            0.0,
            epsilon = 1e-6,
        );
        // At z=-3, radius = 1.0
        assert_abs_diff_eq!(
            loft.evaluate(&Point3::new(1.0, 0.0, -3.0)),
            0.0,
            epsilon = 1e-6,
        );
    }

    #[test]
    fn loft_cap_distance() {
        // Constant-radius loft: point above the top cap
        let loft = FieldNode::Loft {
            stations: vec![[-5.0, 2.0], [5.0, 2.0]],
        };
        // Point on Z axis above top cap: d = distance beyond cap
        let val = loft.evaluate(&Point3::new(0.0, 0.0, 7.0));
        assert_abs_diff_eq!(val, 2.0, epsilon = 1e-6);
    }

    // ── Twist ───────────────────────────────────────────────────────

    #[test]
    fn twist_zero_rate_is_identity() {
        let child = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        let twisted = FieldNode::Twist(Box::new(child.clone()), 0.0);
        let test_points = [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 2.0),
            Point3::new(3.0, 3.0, 3.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(twisted.evaluate(p), child.evaluate(p), epsilon = EPS);
        }
    }

    #[test]
    fn twist_cylinder_is_unchanged() {
        // A Z-aligned cylinder is rotationally symmetric about Z, so twisting
        // doesn't change the field.
        let cyl = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 5.0,
        };
        let twisted = FieldNode::Twist(Box::new(cyl.clone()), 1.0);
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 3.0),
            Point3::new(3.0, 0.0, -4.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(twisted.evaluate(p), cyl.evaluate(p), epsilon = 1e-6);
        }
    }

    #[test]
    fn twist_cuboid_rotates_cross_section() {
        // Twist a cuboid (hx=2, hy=1, hz=5) with rate=PI/2 per unit Z.
        // At z=1, the cross-section should be rotated by PI/2.
        let child = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 1.0, 5.0),
        };
        let rate = std::f64::consts::FRAC_PI_2;
        let twisted = FieldNode::Twist(Box::new(child), rate);

        // At z=0, no rotation: point (2, 0, 0) is on surface → field ≈ 0
        assert_abs_diff_eq!(
            twisted.evaluate(&Point3::new(2.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );

        // At z=1, rotated by PI/2: the x-axis of child maps from y-axis.
        // Point (0, 2, 1) in world → query child at (c*0-s*2, s*0+c*2, 1)
        //   angle = PI/2*1 = PI/2, c=0, s=1 → child at (-2, 0, 1)
        //   cuboid at (-2, 0, 1): |x|-2=0, |y|-1=-1, |z|-5=-4 → on surface
        assert_abs_diff_eq!(
            twisted.evaluate(&Point3::new(0.0, 2.0, 1.0)),
            0.0,
            epsilon = 1e-6,
        );
    }

    // ── Bend ────────────────────────────────────────────────────────

    #[test]
    fn bend_zero_rate_is_identity() {
        let child = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        let bent = FieldNode::Bend(Box::new(child.clone()), 0.0);
        let test_points = [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 2.0),
            Point3::new(3.0, 3.0, 3.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(bent.evaluate(p), child.evaluate(p), epsilon = EPS);
        }
    }

    #[test]
    fn bend_at_origin_is_unchanged() {
        // At z=0, bend angle=0, so field should match child regardless of rate
        let child = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 2.0, 5.0),
        };
        let bent = FieldNode::Bend(Box::new(child.clone()), 0.5);
        let p = Point3::new(1.0, 1.0, 0.0);
        assert_abs_diff_eq!(bent.evaluate(&p), child.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn bend_curves_z_into_x() {
        // Bend a cylinder along Z. At z=0, unchanged. At z>0, the shape
        // should curve in the XZ plane.
        let child = FieldNode::Cylinder {
            radius: 1.0,
            half_height: 5.0,
        };
        let bent = FieldNode::Bend(Box::new(child.clone()), 0.1);

        // At origin: unchanged (z=0 → angle=0)
        assert_abs_diff_eq!(
            bent.evaluate(&Point3::origin()),
            child.evaluate(&Point3::origin()),
            epsilon = EPS,
        );

        // At z=3 on axis: the deformation maps to a different child location.
        // The center should still be inside the cylinder for moderate bend.
        let val = bent.evaluate(&Point3::new(0.0, 0.0, 3.0));
        assert!(
            val < 0.0,
            "center of bent cylinder at z=3 should be inside, got {val}"
        );
    }

    // ── 1D Catmull-Rom ──────────────────────────────────────────────

    #[test]
    fn catmull_rom_1d_endpoints() {
        // At t=0, result should be p1; at t=1, result should be p2
        let r = catmull_rom_1d(0.0, 1.0, 3.0, 4.0, 0.0);
        assert_abs_diff_eq!(r, 1.0, epsilon = EPS);
        let r = catmull_rom_1d(0.0, 1.0, 3.0, 4.0, 1.0);
        assert_abs_diff_eq!(r, 3.0, epsilon = EPS);
    }

    #[test]
    fn catmull_rom_1d_linear_for_uniform_spacing() {
        // For equally spaced values (0, 1, 2, 3), midpoint should be 1.5
        let r = catmull_rom_1d(0.0, 1.0, 2.0, 3.0, 0.5);
        assert_abs_diff_eq!(r, 1.5, epsilon = EPS);
    }

    // ── Repeat ───────────────────────────────────────────────────────

    #[test]
    fn repeat_sphere_at_origin_matches_child() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::Repeat(Box::new(child.clone()), Vector3::new(5.0, 5.0, 5.0));
        // At origin, fold is identity → should match child
        assert_abs_diff_eq!(
            node.evaluate(&Point3::origin()),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
    }

    #[test]
    fn repeat_sphere_at_offset_copy() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::Repeat(Box::new(child.clone()), Vector3::new(5.0, 5.0, 5.0));
        // At (5, 0, 0), should fold to (0, 0, 0) → same as child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(5.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
        // At (10, 0, 0), should fold to (0, 0, 0)
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(10.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
    }

    #[test]
    fn repeat_sphere_midpoint_between_copies() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::Repeat(Box::new(child), Vector3::new(5.0, 5.0, 5.0));
        // At (2.5, 0, 0) — midpoint between two X copies.
        // Folds to (2.5, 0, 0) relative to nearest copy. Child = sphere → distance = 2.5 - 1.0 = 1.5
        let val = node.evaluate(&Point3::new(2.5, 0.0, 0.0));
        assert_abs_diff_eq!(val, 1.5, epsilon = EPS);
    }

    #[test]
    fn repeat_bounded_count_1_is_identity() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::RepeatBounded {
            child: Box::new(child.clone()),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [1, 1, 1],
        };
        // Count 1 on all axes → no repetition, should equal child
        let p = Point3::new(2.0, 0.0, 0.0);
        assert_abs_diff_eq!(node.evaluate(&p), child.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn repeat_bounded_3_copies_along_x() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::RepeatBounded {
            child: Box::new(child.clone()),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [3, 1, 1],
        };
        // 3 copies at x = -5, 0, 5
        // At origin → near center copy → should match child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::origin()),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
        // At x=5 → near right copy → should match child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(5.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
        // At x=-5 → near left copy → should match child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(-5.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
    }

    #[test]
    fn repeat_bounded_beyond_array_uses_nearest_copy() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::RepeatBounded {
            child: Box::new(child.clone()),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [3, 1, 1],
        };
        // At x=12 → beyond rightmost copy at x=5.
        // Should evaluate child at (12 - 5, 0, 0) = (7, 0, 0)
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(12.0, 0.0, 0.0)),
            child.evaluate(&Point3::new(7.0, 0.0, 0.0)),
            epsilon = EPS
        );
    }

    #[test]
    fn repeat_bounded_2_copies_straddle_origin() {
        let child = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let node = FieldNode::RepeatBounded {
            child: Box::new(child.clone()),
            spacing: Vector3::new(4.0, 4.0, 4.0),
            count: [2, 1, 1],
        };
        // 2 copies at x = -2 and x = 2
        // At x=-2: should match child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(-2.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
        // At x=2: should match child at origin
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(2.0, 0.0, 0.0)),
            child.evaluate(&Point3::origin()),
            epsilon = EPS
        );
    }

    #[test]
    fn fold_repeat_maps_to_fundamental_domain() {
        assert_abs_diff_eq!(fold_repeat(0.0, 5.0), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(fold_repeat(5.0, 5.0), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(fold_repeat(2.0, 5.0), 2.0, epsilon = EPS);
        assert_abs_diff_eq!(fold_repeat(7.0, 5.0), 2.0, epsilon = EPS);
        assert_abs_diff_eq!(fold_repeat(-3.0, 5.0), 2.0, epsilon = EPS);
    }

    #[test]
    fn fold_repeat_bounded_count_1_is_identity() {
        assert_abs_diff_eq!(fold_repeat_bounded(7.0, 5.0, 1), 7.0, epsilon = EPS);
        assert_abs_diff_eq!(fold_repeat_bounded(-3.0, 5.0, 1), -3.0, epsilon = EPS);
    }

    #[test]
    fn fold_repeat_bounded_clamps_at_edges() {
        // count=3, spacing=5 → copies at -5, 0, 5
        // At x=12: nearest copy is at 5, so fold = 12 - 5 = 7
        assert_abs_diff_eq!(fold_repeat_bounded(12.0, 5.0, 3), 7.0, epsilon = EPS);
        // At x=-12: nearest copy is at -5, so fold = -12 - (-5) = -7
        assert_abs_diff_eq!(fold_repeat_bounded(-12.0, 5.0, 3), -7.0, epsilon = EPS);
    }
}
