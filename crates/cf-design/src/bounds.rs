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
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
    pub(crate) fn bounds(&self) -> Option<Aabb> {
        match self {
            // ── Primitives ───────────────────────────────────────────
            Self::Sphere { radius } => {
                let r = *radius;
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
            Self::Ellipsoid { radii } => Some(Aabb::new(
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
            Self::Plane { .. } => None,
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
                (Some(a_bb), Some(b_bb)) => Some(a_bb.union(&b_bb).expanded(k / 4.0)),
                _ => None,
            },
            Self::SmoothSubtract(a, _b, k) => a.bounds().map(|bb| bb.expanded(k / 4.0)),
            Self::SmoothIntersect(a, b, k) => match (a.bounds(), b.bounds()) {
                (Some(a_bb), Some(b_bb)) => Some(a_bb.intersection(&b_bb).expanded(k / 4.0)),
                (Some(bb), None) | (None, Some(bb)) => Some(bb.expanded(k / 4.0)),
                (None, None) => None,
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
                let expansion = k * n.ln().max(0.0);
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
            Self::Shell(child, thickness) => child.bounds().map(|bb| bb.expanded(*thickness)),
            Self::Round(child, radius) => child.bounds().map(|bb| bb.expanded(*radius)),
            Self::Offset(child, distance) => child.bounds().map(|bb| bb.expanded(distance.abs())),
            Self::Elongate(child, half) => child.bounds().map(|bb| {
                Aabb::new(
                    Point3::new(bb.min.x - half.x, bb.min.y - half.y, bb.min.z - half.z),
                    Point3::new(bb.max.x + half.x, bb.max.y + half.y, bb.max.z + half.z),
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
    use nalgebra::Vector3;

    #[test]
    fn sphere_bounds() {
        let node = FieldNode::Sphere { radius: 3.0 };
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
            Box::new(FieldNode::Sphere { radius: 1.0 }),
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
        let a = FieldNode::Sphere { radius: 1.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
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
        let a = FieldNode::Sphere { radius: 5.0 };
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
        let sphere = FieldNode::Sphere { radius: 3.0 };
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
        let big = FieldNode::Sphere { radius: 5.0 };
        let small = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 100.0 }),
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
        let sphere = FieldNode::Sphere { radius: 1.0 };
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
        let node = FieldNode::ScaleUniform(Box::new(FieldNode::Sphere { radius: 1.0 }), 3.0);
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)))
        );
    }

    #[test]
    fn shell_expands_bounds() {
        let node = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 5.0 }), 1.0);
        let bb = node.bounds().map(|b| (b.min, b.max));
        assert_eq!(
            bb,
            Some((Point3::new(-6.0, -6.0, -6.0), Point3::new(6.0, 6.0, 6.0)))
        );
    }

    #[test]
    fn elongate_expands_bounds() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
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
}
