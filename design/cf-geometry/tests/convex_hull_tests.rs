//! Tests for `ConvexHull` and `convex_hull` (quickhull algorithm).

#![allow(
    missing_docs,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::float_cmp,
    clippy::manual_midpoint,
    clippy::suboptimal_flops,
    clippy::uninlined_format_args
)]

use cf_geometry::{Bounded, convex_hull};
use nalgebra::{Point3, Vector3};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn cube_vertices() -> Vec<Point3<f64>> {
    vec![
        Point3::new(-0.5, -0.5, -0.5),
        Point3::new(0.5, -0.5, -0.5),
        Point3::new(0.5, 0.5, -0.5),
        Point3::new(-0.5, 0.5, -0.5),
        Point3::new(-0.5, -0.5, 0.5),
        Point3::new(0.5, -0.5, 0.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(-0.5, 0.5, 0.5),
    ]
}

fn tetrahedron_vertices() -> Vec<Point3<f64>> {
    vec![
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(-1.0, -1.0, 1.0),
    ]
}

fn icosphere_vertices(subdivisions: u32) -> Vec<Point3<f64>> {
    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let mut verts = vec![
        Point3::new(-1.0, phi, 0.0),
        Point3::new(1.0, phi, 0.0),
        Point3::new(-1.0, -phi, 0.0),
        Point3::new(1.0, -phi, 0.0),
        Point3::new(0.0, -1.0, phi),
        Point3::new(0.0, 1.0, phi),
        Point3::new(0.0, -1.0, -phi),
        Point3::new(0.0, 1.0, -phi),
        Point3::new(phi, 0.0, -1.0),
        Point3::new(phi, 0.0, 1.0),
        Point3::new(-phi, 0.0, -1.0),
        Point3::new(-phi, 0.0, 1.0),
    ];
    for v in &mut verts {
        let len = v.coords.norm();
        *v = Point3::from(v.coords / len);
    }

    let mut faces: Vec<[usize; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    let mut cache = std::collections::HashMap::new();
    for _ in 0..subdivisions {
        let mut new_faces = Vec::new();
        for &[a, b, c] in &faces {
            let ab = get_midpoint(&mut verts, &mut cache, a, b);
            let bc = get_midpoint(&mut verts, &mut cache, b, c);
            let ca = get_midpoint(&mut verts, &mut cache, c, a);
            new_faces.push([a, ab, ca]);
            new_faces.push([b, bc, ab]);
            new_faces.push([c, ca, bc]);
            new_faces.push([ab, bc, ca]);
        }
        faces = new_faces;
    }
    verts
}

fn get_midpoint(
    verts: &mut Vec<Point3<f64>>,
    cache: &mut std::collections::HashMap<(usize, usize), usize>,
    a: usize,
    b: usize,
) -> usize {
    let key = if a < b { (a, b) } else { (b, a) };
    if let Some(&idx) = cache.get(&key) {
        return idx;
    }
    let mid = Point3::from((verts[a].coords + verts[b].coords) / 2.0);
    let len = mid.coords.norm();
    let normalized = Point3::from(mid.coords / len);
    let idx = verts.len();
    verts.push(normalized);
    cache.insert(key, idx);
    idx
}

/// Scale-dependent epsilon (matches algorithm's internal computation).
fn compute_epsilon(points: &[Point3<f64>]) -> f64 {
    let mut min = points[0];
    let mut max = points[0];
    for p in points {
        for i in 0..3 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    let diagonal = (max - min).norm();
    let eps = diagonal * 1e-10;
    eps.max(1e-14)
}

// ---------------------------------------------------------------------------
// Basic hull correctness
// ---------------------------------------------------------------------------

#[test]
fn test_cube_hull_correctness() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, None).expect("cube should produce a hull");
    assert_eq!(hull.vertices.len(), 8, "cube hull should have 8 vertices");
    assert_eq!(
        hull.faces.len(),
        12,
        "cube hull should have 12 triangular faces"
    );
}

#[test]
fn test_tetrahedron_hull() {
    let pts = tetrahedron_vertices();
    let hull = convex_hull(&pts, None).expect("tetrahedron should produce a hull");
    assert_eq!(hull.vertices.len(), 4);
    assert_eq!(hull.faces.len(), 4);
}

#[test]
fn test_interior_point_excluded() {
    let mut pts = cube_vertices();
    pts.push(Point3::new(0.0, 0.0, 0.0)); // interior
    let hull = convex_hull(&pts, None).expect("should produce a hull");
    assert_eq!(hull.vertices.len(), 8, "interior point should be excluded");
}

// ---------------------------------------------------------------------------
// maxhullvert
// ---------------------------------------------------------------------------

#[test]
fn test_maxhullvert_limits() {
    let pts = icosphere_vertices(1); // 42 vertices
    assert!(pts.len() >= 42);
    let hull = convex_hull(&pts, Some(10)).expect("should produce a hull");
    assert!(
        hull.vertices.len() <= 10,
        "hull should have at most 10 vertices, got {}",
        hull.vertices.len()
    );

    // Validate the truncated hull is a valid convex polytope:
    // all hull vertices are on or below all face planes.
    let eps = compute_epsilon(&pts);
    for hv in &hull.vertices {
        for (fi, &[a, _b, _c]) in hull.faces.iter().enumerate() {
            let dist = (hv - hull.vertices[a as usize]).dot(&hull.normals[fi]);
            assert!(
                dist <= eps * 100.0,
                "hull vertex {:?} is above face {} by {}",
                hv,
                fi,
                dist
            );
        }
    }

    // Verify hull vertices are a subset of the original input points.
    for hv in &hull.vertices {
        assert!(
            pts.iter().any(|p| (p - hv).norm() < 1e-14),
            "hull vertex {:?} is not from the input point set",
            hv
        );
    }
}

#[test]
fn test_maxhullvert_4_boundary() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, Some(4)).expect("should produce a hull");
    assert_eq!(
        hull.vertices.len(),
        4,
        "maxhullvert=4 should yield 4 vertices (initial simplex only)"
    );
}

#[test]
fn test_maxhullvert_exceeds_vertex_count() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, Some(100)).expect("should produce a hull");
    assert_eq!(
        hull.vertices.len(),
        8,
        "limit never reached, full hull expected"
    );
    assert_eq!(hull.faces.len(), 12);
}

// ---------------------------------------------------------------------------
// Degenerate inputs
// ---------------------------------------------------------------------------

#[test]
fn test_fewer_than_4_vertices() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    assert!(convex_hull(&pts, None).is_none());
}

#[test]
fn test_empty_input() {
    let pts: Vec<Point3<f64>> = vec![];
    assert!(convex_hull(&pts, None).is_none());
}

#[test]
fn test_single_point() {
    let pts = vec![Point3::new(1.0, 2.0, 3.0)];
    assert!(convex_hull(&pts, None).is_none());
}

#[test]
fn test_two_points() {
    let pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
    assert!(convex_hull(&pts, None).is_none());
}

#[test]
fn test_coplanar_points() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    assert!(
        convex_hull(&pts, None).is_none(),
        "coplanar points should return None"
    );
}

#[test]
fn test_collinear_points() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    assert!(
        convex_hull(&pts, None).is_none(),
        "collinear points should return None"
    );
}

#[test]
fn test_all_identical_vertices() {
    let pts: Vec<Point3<f64>> = (0..10).map(|_| Point3::new(1.0, 2.0, 3.0)).collect();
    assert!(convex_hull(&pts, None).is_none());
}

#[test]
fn test_duplicate_vertices() {
    let pts = cube_vertices();
    let mut doubled: Vec<Point3<f64>> = pts.iter().chain(pts.iter()).copied().collect();
    doubled.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap()
            .then(a[1].partial_cmp(&b[1]).unwrap())
            .then(a[2].partial_cmp(&b[2]).unwrap())
    });
    let hull = convex_hull(&doubled, None).expect("should produce a hull");
    assert_eq!(
        hull.vertices.len(),
        8,
        "duplicates should not add hull vertices"
    );
    assert_eq!(hull.faces.len(), 12);
}

#[test]
fn test_thin_sliver() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.5, 0.5, 1e-8),
    ];
    let hull = convex_hull(&pts, None);
    assert!(
        hull.is_some(),
        "thin sliver should produce a hull (1e-8 > epsilon)"
    );
    let hull = hull.unwrap();
    assert_eq!(hull.vertices.len(), 4);
    assert_eq!(hull.faces.len(), 4);
}

#[test]
fn test_near_zero_volume_at_large_coordinates() {
    // Tetrahedron at large offset — tests numerical stability of epsilon.
    let offset = 1e6;
    let pts = vec![
        Point3::new(offset, offset, offset),
        Point3::new(offset + 1.0, offset, offset),
        Point3::new(offset, offset + 1.0, offset),
        Point3::new(offset, offset, offset + 1.0),
    ];
    let hull = convex_hull(&pts, None).expect("should produce a hull even at large coordinates");
    assert_eq!(hull.vertices.len(), 4);
    assert_eq!(hull.faces.len(), 4);
}

#[test]
fn test_jittered_cube() {
    // Slightly perturb a cube — hull should still have 8 vertices.
    let mut pts = cube_vertices();
    let jitter = 1e-6;
    for (i, p) in pts.iter_mut().enumerate() {
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        p.x += sign * jitter;
        p.y -= sign * jitter;
        p.z += sign * jitter * 0.5;
    }
    let hull = convex_hull(&pts, None).expect("jittered cube should produce a hull");
    assert_eq!(hull.vertices.len(), 8);
    assert_eq!(hull.faces.len(), 12);
}

// ---------------------------------------------------------------------------
// Face normals
// ---------------------------------------------------------------------------

#[test]
fn test_face_normals_outward() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, None).expect("cube should produce a hull");
    let centroid = Point3::from(
        hull.vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>() / hull.vertices.len() as f64,
    );
    for (fi, &[a, b, c]) in hull.faces.iter().enumerate() {
        let face_center = Point3::from(
            (hull.vertices[a as usize].coords
                + hull.vertices[b as usize].coords
                + hull.vertices[c as usize].coords)
                / 3.0,
        );
        let outward = face_center - centroid;
        assert!(
            outward.dot(&hull.normals[fi]) > 0.0,
            "face {} normal does not point outward",
            fi
        );
    }
}

#[test]
fn test_face_normals_unit_length() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    for (fi, normal) in hull.normals.iter().enumerate() {
        let len = normal.norm();
        assert!(
            (len - 1.0).abs() < 1e-10,
            "face {} normal has length {} (expected 1.0)",
            fi,
            len
        );
    }
}

// ---------------------------------------------------------------------------
// Adjacency graph structural validity
// ---------------------------------------------------------------------------

#[test]
fn test_graph_adjacency_structural() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, None).expect("cube should produce a hull");

    // One adjacency list per vertex
    assert_eq!(hull.adjacency.len(), hull.vertices.len());

    // Every vertex has ≥3 neighbors (minimum degree on a convex polyhedron)
    for (i, adj) in hull.adjacency.iter().enumerate() {
        assert!(
            adj.len() >= 3,
            "vertex {} has only {} neighbors (need ≥3)",
            i,
            adj.len()
        );
    }

    // Adjacency is symmetric
    for (a, adj) in hull.adjacency.iter().enumerate() {
        for &b in adj {
            assert!(
                hull.adjacency[b as usize].contains(&(a as u32)),
                "adjacency not symmetric: {} -> {} but not {} -> {}",
                a,
                b,
                b,
                a
            );
        }
    }

    // Every face edge appears in adjacency
    for &[a, b, c] in &hull.faces {
        assert!(
            hull.adjacency[a as usize].contains(&b),
            "edge ({},{}) missing",
            a,
            b
        );
        assert!(
            hull.adjacency[b as usize].contains(&a),
            "edge ({},{}) missing",
            b,
            a
        );
        assert!(
            hull.adjacency[b as usize].contains(&c),
            "edge ({},{}) missing",
            b,
            c
        );
        assert!(
            hull.adjacency[c as usize].contains(&b),
            "edge ({},{}) missing",
            c,
            b
        );
        assert!(
            hull.adjacency[a as usize].contains(&c),
            "edge ({},{}) missing",
            a,
            c
        );
        assert!(
            hull.adjacency[c as usize].contains(&a),
            "edge ({},{}) missing",
            c,
            a
        );
    }

    // No self-loops
    for (i, adj) in hull.adjacency.iter().enumerate() {
        assert!(!adj.contains(&(i as u32)), "self-loop at vertex {}", i);
    }
}

// ---------------------------------------------------------------------------
// Support / hill-climbing
// ---------------------------------------------------------------------------

#[test]
fn test_hill_climb_equiv_exhaustive_42v() {
    let pts = icosphere_vertices(1); // 42 vertices
    let hull = convex_hull(&pts, None).expect("icosphere hull");

    // 200 random directions (seeded for reproducibility)
    let mut seed: u64 = 12345;
    for _ in 0..200 {
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let x = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let y = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let z = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        let dir = Vector3::new(x, y, z);
        if dir.norm() < 1e-10 {
            continue;
        }
        let dir = dir.normalize();

        let idx_hill = hull.hill_climb(&dir, 0);
        let dot_hill = hull.vertices[idx_hill].coords.dot(&dir);

        // Exhaustive
        let mut max_dot = f64::NEG_INFINITY;
        for v in &hull.vertices {
            let dot = v.coords.dot(&dir);
            if dot > max_dot {
                max_dot = dot;
            }
        }
        assert_eq!(
            dot_hill, max_dot,
            "hill-climbing ≠ exhaustive on 42-vertex icosphere"
        );
    }
}

#[test]
fn test_hill_climb_equiv_exhaustive_162v() {
    let pts = icosphere_vertices(2); // 162 vertices
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    assert!(hull.vertices.len() <= pts.len());

    let mut seed: u64 = 99999;
    for _ in 0..100 {
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let x = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let y = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let z = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
        let dir = Vector3::new(x, y, z);
        if dir.norm() < 1e-10 {
            continue;
        }
        let dir = dir.normalize();

        let idx_hill = hull.hill_climb(&dir, 0);
        let dot_hill = hull.vertices[idx_hill].coords.dot(&dir);

        let mut max_dot = f64::NEG_INFINITY;
        for v in &hull.vertices {
            let dot = v.coords.dot(&dir);
            if dot > max_dot {
                max_dot = dot;
            }
        }
        assert_eq!(
            dot_hill, max_dot,
            "hill-climbing ≠ exhaustive on 162-vertex icosphere"
        );
    }
}

#[test]
fn test_support_returns_correct_point() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, None).expect("cube hull");

    // Support in +X direction should be at x = 0.5
    let sp = hull.support(&Vector3::new(1.0, 0.0, 0.0));
    assert!(
        (sp.x - 0.5).abs() < 1e-10,
        "support in +X should have x=0.5, got {}",
        sp.x
    );

    // Support in -Z direction should be at z = -0.5
    let sp = hull.support(&Vector3::new(0.0, 0.0, -1.0));
    assert!(
        (sp.z - (-0.5)).abs() < 1e-10,
        "support in -Z should have z=-0.5, got {}",
        sp.z
    );
}

#[test]
fn test_support_on_small_hull_uses_exhaustive() {
    // Tetrahedron (4 vertices < HILL_CLIMB_MIN=10) → exhaustive path
    let pts = tetrahedron_vertices();
    let hull = convex_hull(&pts, None).expect("tet hull");
    assert!(hull.vertices.len() < 10);

    let sp = hull.support(&Vector3::new(1.0, 1.0, 1.0).normalize());
    // Support in (1,1,1) direction should be vertex (1,1,1)
    assert!(
        (sp - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
        "support should be (1,1,1), got {:?}",
        sp
    );
}

#[test]
fn test_support_hill_climb_from_various_starts() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    let dir = Vector3::new(1.0, 0.0, 0.0);

    // Hill-climbing from different start vertices should all converge to same result
    let expected = hull.hill_climb(&dir, 0);
    for start in 0..hull.vertices.len() {
        let idx = hull.hill_climb(&dir, start);
        assert_eq!(
            hull.vertices[idx].coords.dot(&dir),
            hull.vertices[expected].coords.dot(&dir),
            "hill-climb from start={} disagrees with start=0",
            start
        );
    }
}

// ---------------------------------------------------------------------------
// Bounded trait
// ---------------------------------------------------------------------------

#[test]
fn test_bounded_impl() {
    let pts = cube_vertices();
    let hull = convex_hull(&pts, None).expect("cube hull");
    let aabb = hull.aabb();
    assert!((aabb.min.x - (-0.5)).abs() < 1e-10);
    assert!((aabb.max.x - 0.5).abs() < 1e-10);
    assert!((aabb.min.y - (-0.5)).abs() < 1e-10);
    assert!((aabb.max.y - 0.5).abs() < 1e-10);
    assert!((aabb.min.z - (-0.5)).abs() < 1e-10);
    assert!((aabb.max.z - 0.5).abs() < 1e-10);
}

// ---------------------------------------------------------------------------
// Structural invariants
// ---------------------------------------------------------------------------

#[test]
fn test_normals_count_equals_faces_count() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    assert_eq!(hull.normals.len(), hull.faces.len());
}

#[test]
fn test_adjacency_count_equals_vertices_count() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    assert_eq!(hull.adjacency.len(), hull.vertices.len());
}

#[test]
fn test_face_indices_in_range() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    let n = hull.vertices.len() as u32;
    for (fi, &[a, b, c]) in hull.faces.iter().enumerate() {
        assert!(a < n, "face {} index a={} out of range", fi, a);
        assert!(b < n, "face {} index b={} out of range", fi, b);
        assert!(c < n, "face {} index c={} out of range", fi, c);
    }
}

#[test]
fn test_adjacency_indices_in_range() {
    let pts = icosphere_vertices(1);
    let hull = convex_hull(&pts, None).expect("icosphere hull");
    let n = hull.vertices.len() as u32;
    for (vi, adj) in hull.adjacency.iter().enumerate() {
        for &neighbor in adj {
            assert!(
                neighbor < n,
                "vertex {} has neighbor {} out of range",
                vi,
                neighbor
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Euler characteristic: V - E + F = 2 for convex polyhedra
// ---------------------------------------------------------------------------

#[test]
fn test_euler_characteristic() {
    for pts in [
        cube_vertices(),
        tetrahedron_vertices(),
        icosphere_vertices(1),
    ] {
        let hull = convex_hull(&pts, None).expect("should produce a hull");
        let v = hull.vertices.len() as i64;
        let f = hull.faces.len() as i64;
        // Count edges from adjacency (each edge counted twice)
        let e: i64 = hull
            .adjacency
            .iter()
            .map(|adj| adj.len() as i64)
            .sum::<i64>()
            / 2;
        assert_eq!(
            v - e + f,
            2,
            "Euler characteristic V-E+F = {} (expected 2), V={}, E={}, F={}",
            v - e + f,
            v,
            e,
            f
        );
    }
}
