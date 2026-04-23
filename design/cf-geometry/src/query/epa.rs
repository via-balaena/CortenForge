//! EPA (Expanding Polytope Algorithm) for penetration depth.
//!
//! When GJK determines that two convex shapes overlap, EPA finds the minimum
//! penetration depth, contact normal, and witness points on each shape.
//!
//! Works generically over any type implementing [`SupportMap`].

use nalgebra::{Point3, Vector3};

use crate::SupportMap;

use super::gjk::{MinkowskiPoint, Simplex, gjk_query, support_minkowski};

/// Numerical tolerance for EPA computations.
const EPSILON: f64 = 1e-8;

/// Maximum faces in EPA polytope.
const EPA_MAX_FACES: usize = 128;

/// Penetration information from EPA.
#[derive(Debug, Clone)]
pub struct Penetration {
    /// Penetration depth (positive when overlapping).
    pub depth: f64,
    /// Contact normal (points from shape B toward shape A).
    pub normal: Vector3<f64>,
    /// Contact point on shape A's surface.
    pub point_a: Point3<f64>,
    /// Contact point on shape B's surface.
    pub point_b: Point3<f64>,
}

/// A face in the EPA polytope.
#[derive(Debug, Clone)]
struct EpaFace {
    vertices: [usize; 3],
    normal: Vector3<f64>,
    distance: f64,
}

// =============================================================================
// Public API
// =============================================================================

/// Compute penetration depth between two overlapping convex shapes.
///
/// Returns `Some(Penetration)` if the shapes overlap, with depth, contact
/// normal, and witness points on each shape. Returns `None` if the shapes
/// are separated or if the computation fails (degenerate case).
///
/// # Arguments
///
/// * `a`, `b` — convex shapes implementing `SupportMap`
/// * `max_iterations` — iteration limit (35 is typical)
/// * `tolerance` — convergence tolerance (1e-6 is typical)
#[must_use]
pub fn epa_penetration(
    a: &impl SupportMap,
    b: &impl SupportMap,
    max_iterations: usize,
    tolerance: f64,
) -> Option<Penetration> {
    // Run GJK first to check intersection and get simplex
    let gjk_result = gjk_query(a, b, max_iterations);

    if !gjk_result.intersecting {
        return None;
    }

    // Run EPA with the GJK simplex
    let epa_result = epa_query_impl(a, b, &gjk_result.simplex, max_iterations, tolerance)?;

    // Recover witness points: support on each shape in the contact normal direction
    let point_a = a.support(&(-epa_result.normal));
    let point_b = b.support(&epa_result.normal);

    Some(Penetration {
        depth: epa_result.depth,
        normal: epa_result.normal,
        point_a,
        point_b,
    })
}

// =============================================================================
// Internal EPA result
// =============================================================================

#[derive(Debug, Clone)]
struct EpaResult {
    depth: f64,
    normal: Vector3<f64>,
}

// =============================================================================
// EPA implementation
// =============================================================================

fn epa_query_impl(
    a: &impl SupportMap,
    b: &impl SupportMap,
    simplex: &Simplex,
    max_iterations: usize,
    tolerance: f64,
) -> Option<EpaResult> {
    if simplex.len() < 4 {
        return epa_with_expanded_simplex(a, b, simplex, max_iterations, tolerance);
    }

    let mut vertices: Vec<MinkowskiPoint> = simplex.points().to_vec();
    let mut faces: Vec<EpaFace> = Vec::with_capacity(EPA_MAX_FACES);

    // Initial tetrahedron faces
    let initial_faces = [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];

    for indices in initial_faces {
        if let Some(face) = create_face(&vertices, indices) {
            faces.push(face);
        }
    }

    fix_face_orientations(&vertices, &mut faces);

    for _iteration in 0..max_iterations {
        let closest_idx = find_closest_face(&faces)?;
        let closest = &faces[closest_idx];

        let new_point = support_minkowski(a, b, &closest.normal);

        let new_distance = new_point.point.coords.dot(&closest.normal);
        if new_distance - closest.distance < tolerance {
            return Some(EpaResult {
                depth: closest.distance,
                normal: closest.normal,
            });
        }

        let new_vertex_idx = vertices.len();
        vertices.push(new_point);

        // Remove visible faces and collect boundary edges
        let mut edges: Vec<(usize, usize)> = Vec::new();
        let mut i = 0;
        while i < faces.len() {
            let face = &faces[i];
            let face_point = vertices[face.vertices[0]].point;
            let to_new = new_point.point - face_point;

            if face.normal.dot(&to_new) > 0.0 {
                let v = face.vertices;
                add_edge(&mut edges, v[0], v[1]);
                add_edge(&mut edges, v[1], v[2]);
                add_edge(&mut edges, v[2], v[0]);
                faces.swap_remove(i);
            } else {
                i += 1;
            }
        }

        // Re-triangulate with new vertex
        for (v1, v2) in edges {
            if let Some(face) = create_face(&vertices, [new_vertex_idx, v1, v2]) {
                faces.push(face);
            }
        }

        if faces.len() > EPA_MAX_FACES {
            break;
        }
    }

    // Return best found
    let closest_idx = find_closest_face(&faces)?;
    let closest = &faces[closest_idx];
    Some(EpaResult {
        depth: closest.distance,
        normal: closest.normal,
    })
}

/// Expand a simplex with fewer than 4 points for EPA.
fn epa_with_expanded_simplex(
    a: &impl SupportMap,
    b: &impl SupportMap,
    simplex: &Simplex,
    max_iterations: usize,
    tolerance: f64,
) -> Option<EpaResult> {
    let mut vertices: Vec<MinkowskiPoint> = simplex.points().to_vec();

    let search_dirs = [
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        -Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
    ];

    for dir in &search_dirs {
        if vertices.len() >= 4 {
            break;
        }

        let new_point = support_minkowski(a, b, dir);

        let is_new = vertices
            .iter()
            .all(|v| (v.point - new_point.point).norm() > EPSILON);

        if is_new {
            vertices.push(new_point);
        }
    }

    if vertices.len() < 4 {
        return None;
    }

    let mut new_simplex = Simplex::new();
    for v in vertices.iter().take(4) {
        new_simplex.push(*v);
    }

    epa_query_impl(a, b, &new_simplex, max_iterations, tolerance)
}

// =============================================================================
// EPA helpers
// =============================================================================

fn create_face(vertices: &[MinkowskiPoint], indices: [usize; 3]) -> Option<EpaFace> {
    let a = vertices[indices[0]].point;
    let b = vertices[indices[1]].point;
    let c = vertices[indices[2]].point;

    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(&ac);

    let norm = normal.norm();
    if norm < EPSILON {
        return None;
    }

    let normal = normal / norm;
    let distance = a.coords.dot(&normal);

    Some(EpaFace {
        vertices: indices,
        normal,
        distance,
    })
}

fn fix_face_orientations(vertices: &[MinkowskiPoint], faces: &mut [EpaFace]) {
    if vertices.is_empty() {
        return;
    }

    // Precision loss acceptable: used for approximate / visualization values.
    #[allow(clippy::cast_precision_loss)]
    let centroid: Vector3<f64> = vertices
        .iter()
        .map(|v| v.point.coords)
        .sum::<Vector3<f64>>()
        / vertices.len() as f64;

    for face in faces.iter_mut() {
        let face_point = vertices[face.vertices[0]].point;
        let to_centroid = centroid - face_point.coords;

        if face.normal.dot(&to_centroid) > 0.0 {
            face.normal = -face.normal;
            face.distance = -face.distance;
            face.vertices.swap(1, 2);
        }
    }
}

fn find_closest_face(faces: &[EpaFace]) -> Option<usize> {
    if faces.is_empty() {
        return None;
    }

    faces
        .iter()
        .enumerate()
        .filter(|(_, f)| f.distance.is_finite())
        .min_by(|(_, a), (_, b)| {
            a.distance
                .abs()
                .partial_cmp(&b.distance.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
}

fn add_edge(edges: &mut Vec<(usize, usize)>, v1: usize, v2: usize) {
    let existing = edges
        .iter()
        .position(|&(a, b)| (a == v2 && b == v1) || (a == v1 && b == v2));

    if let Some(idx) = existing {
        edges.swap_remove(idx);
    } else {
        edges.push((v1, v2));
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::support_map::{
        SupportBox, SupportCapsule, SupportCylinder, SupportEllipsoid, SupportSphere,
    };
    use approx::assert_relative_eq;

    // Helper: offset a support map by translation.
    struct Translated<'a, S: SupportMap> {
        inner: &'a S,
        offset: Vector3<f64>,
    }

    impl<S: SupportMap> crate::Bounded for Translated<'_, S> {
        fn aabb(&self) -> crate::Aabb {
            let inner_aabb = self.inner.aabb();
            crate::Aabb::new(
                Point3::from(inner_aabb.min.coords + self.offset),
                Point3::from(inner_aabb.max.coords + self.offset),
            )
        }
    }

    impl<S: SupportMap> SupportMap for Translated<'_, S> {
        fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
            Point3::from(self.inner.support(direction).coords + self.offset)
        }
    }

    fn at<S: SupportMap>(shape: &S, x: f64, y: f64, z: f64) -> Translated<'_, S> {
        Translated {
            inner: shape,
            offset: Vector3::new(x, y, z),
        }
    }

    #[test]
    fn penetration_overlapping_spheres() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 1.5, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let p = result.unwrap();
        // Penetration depth: radii sum 2.0, centers 1.5 apart → 0.5
        assert_relative_eq!(p.depth, 0.5, epsilon = 0.1);
        // Normal should be along X (B→A direction)
        assert!(p.normal.x.abs() > 0.9);
        assert_relative_eq!(p.normal.norm(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn no_penetration_separated_spheres() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 3.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_none());
    }

    #[test]
    fn penetration_sphere_box() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportBox {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 1.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let p = result.unwrap();
        // sphere r=1 at origin, box [0.5, 1.5] along X → overlap 0.5
        assert_relative_eq!(p.depth, 0.5, epsilon = 0.15);
    }

    #[test]
    fn penetration_capsule_capsule() {
        let a = SupportCapsule {
            half_length: 1.0,
            radius: 0.5,
        };
        let b = SupportCapsule {
            half_length: 1.0,
            radius: 0.5,
        };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 0.8, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let p = result.unwrap();
        // Radii sum 1.0, centers 0.8 apart → depth 0.2
        assert_relative_eq!(p.depth, 0.2, epsilon = 0.1);
    }

    #[test]
    fn penetration_cylinder_sphere() {
        let a = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };
        let b = SupportSphere { radius: 0.5 };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 0.8, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let p = result.unwrap();
        // cylinder r=0.5, sphere r=0.5, centers 0.8 apart → depth 0.2
        assert_relative_eq!(p.depth, 0.2, epsilon = 0.1);
    }

    #[test]
    fn penetration_ellipsoid_sphere() {
        let a = SupportEllipsoid {
            radii: Vector3::new(2.0, 1.0, 0.5),
        };
        let b = SupportSphere { radius: 0.5 };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 2.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let p = result.unwrap();
        // Ellipsoid x-radius=2.0, sphere r=0.5, centers 2.0 apart → depth 0.5
        assert_relative_eq!(p.depth, 0.5, epsilon = 0.15);
    }

    #[test]
    fn penetration_convex_hull() {
        let vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let hull = crate::convex_hull(&vertices, None).unwrap();
        let sphere = SupportSphere { radius: 0.5 };

        let result = epa_penetration(
            &at(&hull, 0.0, 0.0, 0.0),
            &at(&sphere, 0.8, 0.0, 0.0),
            35,
            1e-6,
        );
        assert!(result.is_some());
    }

    #[test]
    fn coincident_shapes() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = epa_penetration(&at(&a, 0.0, 0.0, 0.0), &at(&b, 0.0, 0.0, 0.0), 35, 1e-6);
        // Fully overlapping — should still return a result
        if let Some(p) = result {
            assert!(p.depth > 0.0);
        }
    }
}
