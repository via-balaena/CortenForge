//! Axis-aligned bounding box computation for field nodes.
//!
//! Computes conservative AABBs for the geometry defined by each expression
//! tree node. Used by the mesher to determine the evaluation domain.
//! Returns `None` for infinite geometry (e.g., a bare `Plane`).

use cf_geometry::Aabb;
use nalgebra::Point3;

use crate::field_node::FieldNode;

impl FieldNode {
    /// Compute the axis-aligned bounding box of the geometry.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    /// Finite operations on infinite geometry (e.g., `Intersect(Sphere, Plane)`)
    /// inherit the finite child's bounds.
    // Precision loss acceptable for approximate / visualization values.
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
    pub(crate) fn bounds(&self) -> Option<Aabb> {
        match self {
            // ── Primitives ───────────────────────────────────────────
            Self::Sphere { radius } => {
                let r = radius.eval();
                Some(Aabb::new(Point3::new(-r, -r, -r), Point3::new(r, r, r)))
            }
            Self::Cuboid { half_extents: h } => Some(Aabb::new(
                Point3::new(-h.x, -h.y, -h.z),
                Point3::new(h.x, h.y, h.z),
            )),
            Self::Cylinder {
                radius,
                half_height,
            } => {
                let r = *radius;
                let h = *half_height;
                Some(Aabb::new(Point3::new(-r, -r, -h), Point3::new(r, r, h)))
            }
            Self::Capsule {
                radius,
                half_height,
            } => {
                let r = *radius;
                let h = *half_height;
                Some(Aabb::new(
                    Point3::new(-r, -r, -(h + r)),
                    Point3::new(r, r, h + r),
                ))
            }
            Self::Ellipsoid { radii } | Self::Superellipsoid { radii, .. } => Some(Aabb::new(
                Point3::new(-radii.x, -radii.y, -radii.z),
                Point3::new(radii.x, radii.y, radii.z),
            )),
            Self::Torus { major, minor } => {
                let outer = major + minor;
                let m = *minor;
                Some(Aabb::new(
                    Point3::new(-outer, -outer, -m),
                    Point3::new(outer, outer, m),
                ))
            }
            Self::Cone { radius, height } => {
                let r = *radius;
                let h = *height;
                Some(Aabb::new(Point3::new(-r, -r, -h), Point3::new(r, r, 0.0)))
            }
            Self::LogSpiral {
                a,
                b,
                thickness,
                turns,
            } => {
                // The spiral extends from θ=0 to θ=turns·2π.
                // Max radius = max(a, a·exp(b·turns·2π)) + thickness.
                let r_start = *a;
                let r_end = a * (b * turns * std::f64::consts::TAU).exp();
                let r_max = r_start.max(r_end) + thickness;
                Some(Aabb::new(
                    Point3::new(-r_max, -r_max, -thickness),
                    Point3::new(r_max, r_max, *thickness),
                ))
            }
            Self::Gyroid { .. }
            | Self::SchwarzP { .. }
            | Self::Plane { .. }
            | Self::Repeat(_, _) => None,
            Self::Helix {
                radius,
                pitch,
                thickness,
                turns,
            } => {
                let r = radius + thickness;
                let z_max = pitch * turns + thickness;
                Some(Aabb::new(
                    Point3::new(-r, -r, -thickness),
                    Point3::new(r, r, z_max),
                ))
            }
            Self::Loft { stations } => {
                if stations.len() < 2 {
                    return Some(Aabb::from_point(Point3::origin()));
                }
                let z_min = stations[0][0];
                let z_max = stations[stations.len() - 1][0];
                // Conservative max radius: max station radius + overshoot bound.
                // Catmull-Rom can overshoot by up to max(|Δr|)/2 (conservative).
                let r_max = stations.iter().map(|s| s[1]).fold(0.0_f64, f64::max);
                let max_diff = stations
                    .windows(2)
                    .map(|w| (w[1][1] - w[0][1]).abs())
                    .fold(0.0_f64, f64::max);
                let r_safe = r_max + max_diff * 0.5;
                Some(Aabb::new(
                    Point3::new(-r_safe, -r_safe, z_min),
                    Point3::new(r_safe, r_safe, z_max),
                ))
            }
            Self::Pipe { vertices, radius } => {
                if vertices.is_empty() {
                    return Some(Aabb::from_point(Point3::origin()));
                }
                Some(Aabb::from_points(vertices.iter()).expanded(*radius))
            }
            Self::PipeSpline {
                control_points,
                radius,
            } => {
                if control_points.is_empty() {
                    return Some(Aabb::from_point(Point3::origin()));
                }
                // Catmull-Rom splines can overshoot the control point convex
                // hull. The maximum overshoot is bounded by max_segment_length/4
                // (from the second-difference bound on the basis functions).
                let mut max_seg = 0.0_f64;
                for pair in control_points.windows(2) {
                    let d = (pair[1] - pair[0]).norm();
                    if d > max_seg {
                        max_seg = d;
                    }
                }
                Some(Aabb::from_points(control_points.iter()).expanded(radius + max_seg / 4.0))
            }

            // ── Booleans ─────────────────────────────────────────────
            Self::Union(a, b) => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.union(&b_bb)),
                _ => None,
            },
            Self::Subtract(a, _b) => a.bounds(),
            Self::Intersect(a, b) => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.intersection(&b_bb)),
                (Some(bb), None) | (None, Some(bb)) => Some(bb),
                (None, None) => None,
            },
            Self::SmoothUnion(a, b, k) => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.union(&b_bb).expanded(k.eval() / 4.0)),
                _ => None,
            },
            Self::SmoothSubtract(a, _b, k) => a.bounds().map(|bb| bb.expanded(k.eval() / 4.0)),
            Self::SmoothIntersect(a, b, k) => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.intersection(&b_bb).expanded(k.eval() / 4.0)),
                (Some(bb), None) | (None, Some(bb)) => Some(bb.expanded(k.eval() / 4.0)),
                (None, None) => None,
            },
            Self::SmoothUnionVariable { a, b, max_k, .. } => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.union(&b_bb).expanded(max_k / 4.0)),
                _ => None,
            },
            Self::SmoothUnionAll(children, k) => {
                let mut result: Option<Aabb> = None;
                for child in children {
                    match (result, child.bounds()) {
                        (Some(acc), Some(bb)) => result = Some(acc.union(&bb)),
                        (None, Some(bb)) => result = Some(bb),
                        (_, None) => return None,
                    }
                }
                let n = children.len() as f64;
                let expansion = k.eval() * n.ln().max(0.0);
                result.map(|bb| bb.expanded(expansion))
            }

            // ── Transforms ───────────────────────────────────────────
            Self::Translate(child, offset) => child.bounds().map(|bb| {
                Aabb::new(
                    Point3::from(bb.min.coords + offset),
                    Point3::from(bb.max.coords + offset),
                )
            }),
            Self::Rotate(child, q) => child.bounds().map(|bb| {
                let corners = bb.corners();
                let rotated: Vec<Point3<f64>> =
                    corners.iter().map(|c| q.transform_point(c)).collect();
                Aabb::from_points(rotated.iter())
            }),
            Self::ScaleUniform(child, s) => child.bounds().map(|bb| {
                Aabb::new(
                    Point3::new(bb.min.x * s, bb.min.y * s, bb.min.z * s),
                    Point3::new(bb.max.x * s, bb.max.y * s, bb.max.z * s),
                )
            }),
            Self::Mirror(child, normal) => child.bounds().map(|bb| {
                let corners = bb.corners();
                let mut reflected: Vec<Point3<f64>> = Vec::with_capacity(8);
                for c in &corners {
                    let d = c.coords.dot(normal);
                    reflected.push(Point3::from(c.coords - normal * (2.0 * d)));
                }
                let reflected_bb = Aabb::from_points(reflected.iter());
                bb.union(&reflected_bb)
            }),

            // ── Domain operations ────────────────────────────────────
            Self::Shell(child, thickness) => child.bounds().map(|bb| bb.expanded(thickness.eval())),
            Self::Round(child, radius) => child.bounds().map(|bb| bb.expanded(radius.eval())),
            Self::Offset(child, distance) => {
                child.bounds().map(|bb| bb.expanded(distance.eval().abs()))
            }
            Self::Elongate(child, half) => child.bounds().map(|bb| {
                Aabb::new(
                    Point3::new(bb.min.x - half.x, bb.min.y - half.y, bb.min.z - half.z),
                    Point3::new(bb.max.x + half.x, bb.max.y + half.y, bb.max.z + half.z),
                )
            }),
            Self::Twist(child, _) => child.bounds().map(|bb| {
                // Twist rotates XY — bounds expand to circumscribed circle in XY.
                let x_ext = bb.min.x.abs().max(bb.max.x.abs());
                let y_ext = bb.min.y.abs().max(bb.max.y.abs());
                let r = x_ext.hypot(y_ext);
                Aabb::new(Point3::new(-r, -r, bb.min.z), Point3::new(r, r, bb.max.z))
            }),
            Self::Bend(child, _) => child.bounds().map(|bb| {
                // Bend rotates XZ — bounds expand to circumscribed circle in XZ.
                let x_ext = bb.min.x.abs().max(bb.max.x.abs());
                let z_ext = bb.min.z.abs().max(bb.max.z.abs());
                let r = x_ext.hypot(z_ext);
                Aabb::new(Point3::new(-r, bb.min.y, -r), Point3::new(r, bb.max.y, r))
            }),
            Self::RepeatBounded {
                child,
                spacing,
                count,
            } => child.bounds().map(|bb| {
                // Total array extent: (count - 1) * spacing, centered at origin.
                let ext_x = (f64::from(count[0]) - 1.0) * spacing.x * 0.5;
                let ext_y = (f64::from(count[1]) - 1.0) * spacing.y * 0.5;
                let ext_z = (f64::from(count[2]) - 1.0) * spacing.z * 0.5;
                Aabb::new(
                    Point3::new(bb.min.x - ext_x, bb.min.y - ext_y, bb.min.z - ext_z),
                    Point3::new(bb.max.x + ext_x, bb.max.y + ext_y, bb.max.z + ext_z),
                )
            }),

            // ── User function ────────────────────────────────────────
            Self::UserFn { bounds, .. } => Some(*bounds),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field_node::Val;
    use nalgebra::Vector3;

    #[test]
    fn sphere_bounds() {
        let node = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)))
        );
    }

    #[test]
    fn cuboid_bounds() {
        let node = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-1.0, -2.0, -3.0), Point3::new(1.0, 2.0, 3.0)))
        );
    }

    #[test]
    fn plane_bounds_is_none() {
        let node = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 0.0,
        };
        assert!(node.bounds().is_none());
    }

    #[test]
    fn translate_shifts_bounds() {
        let node = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(5.0, 0.0, 0.0),
        );
        let bb = node.bounds();
        assert!(bb.is_some());
        let bb = bb.map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(4.0, -1.0, -1.0), Point3::new(6.0, 1.0, 1.0)))
        );
    }

    #[test]
    fn union_merges_bounds() {
        let a = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(5.0, 0.0, 0.0),
        );
        let node = FieldNode::Union(Box::new(a), Box::new(b));
        let bb = node.bounds();
        assert!(bb.is_some());
        let bb = bb.map(|b| b.size());
        // Should span from -1 to 6 in x, -1 to 1 in y/z
        assert_eq!(bb, Some(Vector3::new(7.0, 2.0, 2.0)));
    }

    #[test]
    fn intersect_clips_bounds() {
        let a = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let b = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 2.0, 2.0),
        };
        let node = FieldNode::Intersect(Box::new(a), Box::new(b));
        let bb = node.bounds();
        assert!(bb.is_some());
        let bb = bb.map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0)))
        );
    }

    #[test]
    fn intersect_with_plane_uses_finite_child() {
        let sphere = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let plane = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 0.0,
        };
        let node = FieldNode::Intersect(Box::new(sphere), Box::new(plane));
        let bb = node.bounds();
        assert!(bb.is_some());
        let bb = bb.map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)))
        );
    }

    #[test]
    fn subtract_uses_first_child_bounds() {
        let big = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let small = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(100.0),
            }),
            Vector3::new(200.0, 0.0, 0.0),
        );
        let node = FieldNode::Subtract(Box::new(big), Box::new(small));
        let bb = node.bounds();
        assert!(bb.is_some());
        let bb = bb.map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-5.0, -5.0, -5.0), Point3::new(5.0, 5.0, 5.0)))
        );
    }

    #[test]
    fn union_with_plane_is_none() {
        let sphere = FieldNode::Sphere {
            radius: Val::from(1.0),
        };
        let plane = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 0.0,
        };
        let node = FieldNode::Union(Box::new(sphere), Box::new(plane));
        assert!(node.bounds().is_none());
    }

    #[test]
    fn torus_bounds() {
        let node = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-6.0, -6.0, -1.0), Point3::new(6.0, 6.0, 1.0)))
        );
    }

    #[test]
    fn pipe_bounds() {
        let node = FieldNode::Pipe {
            vertices: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            radius: 1.0,
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-1.0, -1.0, -1.0), Point3::new(11.0, 1.0, 1.0)))
        );
    }

    #[test]
    fn scale_uniform_bounds() {
        let node = FieldNode::ScaleUniform(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            3.0,
        );
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)))
        );
    }

    #[test]
    fn shell_expands_bounds() {
        let node = FieldNode::Shell(
            Box::new(FieldNode::Sphere {
                radius: Val::from(5.0),
            }),
            Val::from(1.0),
        );
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-6.0, -6.0, -6.0), Point3::new(6.0, 6.0, 6.0)))
        );
    }

    #[test]
    fn elongate_expands_bounds() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.0, -1.0, -1.0), Point3::new(3.0, 1.0, 1.0)))
        );
    }

    #[test]
    fn user_fn_uses_explicit_bounds() {
        use crate::field_node::UserEvalFn;
        use std::sync::Arc;

        let aabb = Aabb::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );
        let node = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 5.0)),
            interval: None,
            bounds: aabb,
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((
                Point3::new(-10.0, -10.0, -10.0),
                Point3::new(10.0, 10.0, 10.0)
            ))
        );
    }

    // ── Bio-inspired primitive bounds tests ──────────────────────────

    #[test]
    fn superellipsoid_bounds() {
        let node = FieldNode::Superellipsoid {
            radii: Vector3::new(2.0, 3.0, 4.0),
            n1: 2.0,
            n2: 2.0,
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-2.0, -3.0, -4.0), Point3::new(2.0, 3.0, 4.0)))
        );
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn log_spiral_bounds_contains_endpoints() {
        let node = FieldNode::LogSpiral {
            a: 1.0,
            b: 0.1,
            thickness: 0.3,
            turns: 2.0,
        };
        let bb = node.bounds().expect("finite bounds");
        // Start: r=1 at θ=0. End: r=1*exp(0.1*2*2π) ≈ 3.51 at θ=4π.
        // Max radius should be ≥ 3.51 + 0.3 ≈ 3.81
        assert!(
            bb.max.x > 3.5,
            "bounds should contain spiral endpoint, got x_max={}",
            bb.max.x
        );
    }

    #[test]
    fn gyroid_bounds_is_none() {
        let node = FieldNode::Gyroid {
            scale: 1.0,
            thickness: 0.5,
        };
        assert!(node.bounds().is_none());
    }

    #[test]
    fn schwarz_p_bounds_is_none() {
        let node = FieldNode::SchwarzP {
            scale: 1.0,
            thickness: 0.5,
        };
        assert!(node.bounds().is_none());
    }

    #[test]
    fn helix_bounds() {
        let node = FieldNode::Helix {
            radius: 3.0,
            pitch: 2.0,
            thickness: 0.5,
            turns: 2.0,
        };
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.5, -3.5, -0.5), Point3::new(3.5, 3.5, 4.5)))
        );
    }

    // ── Loft bounds tests ───────────────────────────────────────────

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn loft_constant_radius_bounds() {
        let node = FieldNode::Loft {
            stations: vec![[-5.0, 2.0], [5.0, 2.0]],
        };
        let bb = node.bounds().expect("finite bounds");
        assert!((bb.min.x - (-2.0)).abs() < 1e-10);
        assert!((bb.max.x - 2.0).abs() < 1e-10);
        assert!((bb.min.z - (-5.0)).abs() < 1e-10);
        assert!((bb.max.z - 5.0).abs() < 1e-10);
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn loft_tapered_bounds_contains_max_radius() {
        let node = FieldNode::Loft {
            stations: vec![[-3.0, 3.0], [3.0, 1.0]],
        };
        let bb = node.bounds().expect("finite bounds");
        // Max radius is 3.0 (at z=-3), with overshoot bound
        assert!(
            bb.max.x >= 3.0,
            "bounds should contain max radius 3.0, got x_max={}",
            bb.max.x
        );
        assert!((bb.min.z - (-3.0)).abs() < 1e-10);
        assert!((bb.max.z - 3.0).abs() < 1e-10);
    }

    // ── Twist bounds tests ──────────────────────────────────────────

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn twist_bounds_expands_xy() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 5.0),
            }),
            1.0,
        );
        let bb = node.bounds().expect("finite bounds");
        // XY should expand to circumscribed circle: r = sqrt(1² + 2²) ≈ 2.236
        let r = 1.0_f64.hypot(2.0);
        assert!((bb.min.x - (-r)).abs() < 1e-10);
        assert!((bb.max.x - r).abs() < 1e-10);
        assert!((bb.min.y - (-r)).abs() < 1e-10);
        assert!((bb.max.y - r).abs() < 1e-10);
        // Z should be unchanged
        assert!((bb.min.z - (-5.0)).abs() < 1e-10);
        assert!((bb.max.z - 5.0).abs() < 1e-10);
    }

    // ── Bend bounds tests ───────────────────────────────────────────

    // ── Repeat bounds tests ───────────────────────────────────────────

    #[test]
    fn repeat_bounds_is_none() {
        let node = FieldNode::Repeat(
            Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            Vector3::new(5.0, 5.0, 5.0),
        );
        assert!(node.bounds().is_none());
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn repeat_bounded_bounds_single_copy() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [1, 1, 1],
        };
        let bb = node.bounds().expect("finite bounds");
        // Single copy → same as child bounds
        assert!((bb.min.x - (-1.0)).abs() < 1e-10);
        assert!((bb.max.x - 1.0).abs() < 1e-10);
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn repeat_bounded_bounds_3x1x1() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere {
                radius: Val::from(1.0),
            }),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [3, 1, 1],
        };
        let bb = node.bounds().expect("finite bounds");
        // 3 copies along X: extent = (3-1)*5/2 = 5. Child bounds ±1.
        // Total: [-1-5, 1+5] = [-6, 6] along X.
        assert!((bb.min.x - (-6.0)).abs() < 1e-10);
        assert!((bb.max.x - 6.0).abs() < 1e-10);
        // Y and Z: single copy → child bounds
        assert!((bb.min.y - (-1.0)).abs() < 1e-10);
        assert!((bb.max.y - 1.0).abs() < 1e-10);
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn repeat_bounded_bounds_2x2x1() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere {
                radius: Val::from(0.5),
            }),
            spacing: Vector3::new(3.0, 3.0, 3.0),
            count: [2, 2, 1],
        };
        let bb = node.bounds().expect("finite bounds");
        // X: (2-1)*3/2 = 1.5. Total: [-0.5-1.5, 0.5+1.5] = [-2, 2]
        assert!((bb.min.x - (-2.0)).abs() < 1e-10);
        assert!((bb.max.x - 2.0).abs() < 1e-10);
        // Y: same as X
        assert!((bb.min.y - (-2.0)).abs() < 1e-10);
        assert!((bb.max.y - 2.0).abs() < 1e-10);
        // Z: single copy
        assert!((bb.min.z - (-0.5)).abs() < 1e-10);
        assert!((bb.max.z - 0.5).abs() < 1e-10);
    }

    #[test]
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn bend_bounds_expands_xz() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 5.0),
            }),
            0.5,
        );
        let bb = node.bounds().expect("finite bounds");
        // XZ should expand to circumscribed circle: r = sqrt(1² + 5²) ≈ 5.1
        let r = 1.0_f64.hypot(5.0);
        assert!((bb.min.x - (-r)).abs() < 1e-10);
        assert!((bb.max.x - r).abs() < 1e-10);
        // Y should be unchanged
        assert!((bb.min.y - (-2.0)).abs() < 1e-10);
        assert!((bb.max.y - 2.0).abs() < 1e-10);
        // Z should expand
        assert!((bb.min.z - (-r)).abs() < 1e-10);
        assert!((bb.max.z - r).abs() < 1e-10);
    }
}
