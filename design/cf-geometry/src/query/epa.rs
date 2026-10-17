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
///
/// The witness points come from the closest face of the EPA polytope, as
/// MuJoCo's `epaWitness`: `point_a − point_b` is `depth · normal`, up to the
/// polytope's rounding.
#[derive(Debug, Clone)]
pub struct Penetration {
    /// Penetration depth (positive when overlapping).
    pub depth: f64,
    /// Contact normal (points from shape A toward shape B).
    pub normal: Vector3<f64>,
    /// Witness point of shape A.
    pub point_a: Point3<f64>,
    /// Witness point of shape B.
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

    Some(Penetration {
        depth: epa_result.depth,
        normal: epa_result.normal,
        point_a: epa_result.witness_a,
        point_b: epa_result.witness_b,
    })
}

// =============================================================================
// Internal EPA result
// =============================================================================

#[derive(Debug, Clone)]
struct EpaResult {
    depth: f64,
    normal: Vector3<f64>,
    witness_a: Point3<f64>,
    witness_b: Point3<f64>,
}

impl EpaResult {
    /// The result at the polytope's closest face, its witness points as
    /// MuJoCo's `epaWitness` recovers them (`engine_collision_gjk.c:1273-1287`):
    /// the affine coordinates of the origin's projection on the face, applied
    /// to the face vertices' support points on each shape.
    fn from_face(vertices: &[MinkowskiPoint], face: &EpaFace) -> Self {
        let [v1, v2, v3] = face.vertices.map(|i| &vertices[i]);
        let lambda = tri_affine_coord(
            &v1.point.coords,
            &v2.point.coords,
            &v3.point.coords,
            &(face.normal * face.distance),
        );
        let combine = |p1: &Point3<f64>, p2: &Point3<f64>, p3: &Point3<f64>| {
            Point3::from(p1.coords * lambda[0] + p2.coords * lambda[1] + p3.coords * lambda[2])
        };
        Self {
            depth: face.distance,
            normal: face.normal,
            witness_a: combine(&v1.support_a, &v2.support_a, &v3.support_a),
            witness_b: combine(&v1.support_b, &v2.support_b, &v3.support_b),
        }
    }
}

/// Affine coordinates of `p` in the triangle `v1 v2 v3`, as MuJoCo's
/// `triAffineCoord` (`engine_collision_gjk.c:979-1020`): the signed areas of
/// the projection onto the coordinate plane where the triangle's is largest.
#[allow(clippy::many_single_char_names)] // MuJoCo's names
fn tri_affine_coord(
    v1: &Vector3<f64>,
    v2: &Vector3<f64>,
    v3: &Vector3<f64>,
    p: &Vector3<f64>,
) -> [f64; 3] {
    let m_14 = v2[1] * v3[2] - v2[2] * v3[1] - v1[1] * v3[2] + v1[2] * v3[1] + v1[1] * v2[2]
        - v1[2] * v2[1];
    let m_24 = v2[0] * v3[2] - v2[2] * v3[0] - v1[0] * v3[2] + v1[2] * v3[0] + v1[0] * v2[2]
        - v1[2] * v2[0];
    let m_34 = v2[0] * v3[1] - v2[1] * v3[0] - v1[0] * v3[1] + v1[1] * v3[0] + v1[0] * v2[1]
        - v1[1] * v2[0];

    // Project onto the coordinate plane where the triangle's area is largest.
    let (mu1, mu2, mu3) = (m_14.abs(), m_24.abs(), m_34.abs());
    let (m_max, x, y) = if mu1 >= mu2 && mu1 >= mu3 {
        (m_14, 1, 2)
    } else if mu2 >= mu3 {
        (m_24, 0, 2)
    } else {
        (m_34, 0, 1)
    };

    // Signed areas of (p, v2, v3), (p, v1, v3) and (p, v1, v2).
    let c31 =
        p[x] * v2[y] + p[y] * v3[x] + v2[x] * v3[y] - p[x] * v3[y] - p[y] * v2[x] - v3[x] * v2[y];
    let c32 =
        p[x] * v3[y] + p[y] * v1[x] + v3[x] * v1[y] - p[x] * v1[y] - p[y] * v3[x] - v1[x] * v3[y];
    let c33 =
        p[x] * v1[y] + p[y] * v2[x] + v1[x] * v2[y] - p[x] * v2[y] - p[y] * v1[x] - v2[x] * v1[y];

    [c31 / m_max, c32 / m_max, c33 / m_max]
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
            return Some(EpaResult::from_face(&vertices, closest));
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
    Some(EpaResult::from_face(&vertices, &faces[closest_idx]))
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
        // The normal points from A toward B: +x
        assert!(p.normal.x > 0.9);
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

    // Helper: a support map rotated, then translated.
    struct Posed<'a> {
        inner: &'a dyn SupportMap,
        rotation: nalgebra::UnitQuaternion<f64>,
        offset: Vector3<f64>,
    }

    impl crate::Bounded for Posed<'_> {
        fn aabb(&self) -> crate::Aabb {
            let mut min = Vector3::repeat(f64::INFINITY);
            let mut max = Vector3::repeat(f64::NEG_INFINITY);
            for axis in 0..3 {
                let mut d = Vector3::zeros();
                d[axis] = 1.0;
                max[axis] = self.support(&d)[axis];
                min[axis] = self.support(&-d)[axis];
            }
            crate::Aabb::new(Point3::from(min), Point3::from(max))
        }
    }

    impl SupportMap for Posed<'_> {
        fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
            let local = self
                .inner
                .support(&self.rotation.inverse_transform_vector(direction));
            Point3::from(self.rotation * local.coords + self.offset)
        }
    }

    /// The witness points are the closest face's (MuJoCo's `epaWitness`), so
    /// their difference is the penetration vector. Each shape's support along
    /// the normal, which the witnesses were before, misses it by up to the
    /// shapes' size.
    #[test]
    fn epa_witness_difference_is_depth_times_normal() {
        // A fixed linear congruential sequence (Knuth's MMIX constants).
        let mut state: u64 = 0x2545_f491_4f6c_dd1d;
        let mut uniform = |lo: f64, hi: f64| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            #[allow(clippy::cast_precision_loss)]
            let u = (state >> 11) as f64 / (1u64 << 53) as f64;
            lo + (hi - lo) * u
        };
        let mut overlaps = 0;
        let mut worst = 0.0_f64;
        for _ in 0..2000 {
            let shape = |u: &mut dyn FnMut(f64, f64) -> f64| -> Box<dyn SupportMap> {
                let (a, b, c) = (u(0.05, 0.5), u(0.05, 0.5), u(0.05, 0.5));
                let kind = u(0.0, 4.0);
                if kind < 1.0 {
                    Box::new(SupportBox {
                        half_extents: Vector3::new(a, b, c),
                    })
                } else if kind < 2.0 {
                    Box::new(SupportEllipsoid {
                        radii: Vector3::new(a, b, c),
                    })
                } else if kind < 3.0 {
                    Box::new(SupportCylinder {
                        half_length: a,
                        radius: b,
                    })
                } else {
                    Box::new(SupportCapsule {
                        half_length: a,
                        radius: b,
                    })
                }
            };
            let (shape_a, shape_b) = (shape(&mut uniform), shape(&mut uniform));
            let mut pose = || {
                let axis = Vector3::new(uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0));
                let rotation = nalgebra::UnitQuaternion::from_scaled_axis(
                    axis.normalize() * uniform(0.0, std::f64::consts::PI),
                );
                let offset =
                    Vector3::new(uniform(-0.4, 0.4), uniform(-0.4, 0.4), uniform(-0.4, 0.4));
                (rotation, offset)
            };
            let ((rotation_a, offset_a), (rotation_b, offset_b)) = (pose(), pose());
            let a = Posed {
                inner: shape_a.as_ref(),
                rotation: rotation_a,
                offset: offset_a,
            };
            let b = Posed {
                inner: shape_b.as_ref(),
                rotation: rotation_b,
                offset: offset_b,
            };
            if let Some(p) = epa_penetration(&a, &b, 35, 1e-6) {
                overlaps += 1;
                let off = (p.point_a - p.point_b - p.normal * p.depth).norm();
                worst = worst.max(off);
            }
        }
        assert!(overlaps >= 1000, "{overlaps} overlaps of 2000 poses");
        assert!(
            worst <= 1e-10,
            "point_a − point_b misses depth · normal by {worst}"
        );
    }
}
