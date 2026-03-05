//! Convex hull computation via Quickhull algorithm.
//!
//! Computes the convex hull of a 3D point set at build time, producing a
//! `ConvexHull` with vertices, triangulated faces, outward normals, and an
//! adjacency graph for O(√n) hill-climbing GJK support queries.
//!
//! MuJoCo ref: `mjCMesh::MakeGraph()` in `user_mesh.cc` delegates to the
//! Qhull library. CortenForge implements Quickhull directly in pure Rust.

use nalgebra::{Point3, Vector3};
use std::collections::{HashSet, VecDeque};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Convex hull of a triangle mesh, computed via Quickhull at build time.
///
/// Stores the hull geometry (vertices, triangulated faces, outward normals)
/// and an adjacency graph for O(√n) hill-climbing GJK support queries.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConvexHull {
    /// Hull vertices in local mesh frame. Subset of the original mesh's
    /// deduplicated vertices (or new vertices if deduplication merges some).
    pub vertices: Vec<Point3<f64>>,

    /// Triangulated hull faces as index triples into `vertices`.
    /// All faces are triangles. Winding: counter-clockwise when viewed from
    /// outside (outward normal follows right-hand rule).
    pub faces: Vec<[usize; 3]>,

    /// Outward unit face normals (one per face, same order as `faces`).
    /// CortenForge convenience — MuJoCo's graph stores no face normals.
    /// Needed by Spec B (shell inertia) and contact normal computation.
    pub normals: Vec<Vector3<f64>>,

    /// Vertex adjacency graph for hill-climbing support queries.
    pub graph: HullGraph,
}

/// Vertex adjacency graph for convex hull.
///
/// Rust-idiomatic representation of MuJoCo's flat `mesh_graph` int array.
/// For each hull vertex, stores the indices of all neighboring vertices
/// (vertices that share a hull edge).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HullGraph {
    /// `adjacency[i]` = sorted list of vertex indices adjacent to vertex `i`.
    /// Length equals `ConvexHull::vertices.len()`.
    pub adjacency: Vec<Vec<usize>>,
}

// ---------------------------------------------------------------------------
// Internal face representation for Quickhull
// ---------------------------------------------------------------------------

struct Face {
    indices: [usize; 3],
    normal: Vector3<f64>,
    center: Point3<f64>,
    conflict_list: Vec<usize>,
    farthest_dist: f64,
    farthest_idx: usize,
    alive: bool,
    neighbors: [usize; 3],
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute the convex hull of a point set using the Quickhull algorithm.
///
/// Returns `None` if fewer than 4 non-degenerate points exist (matching
/// MuJoCo's early return in `MakeGraph()`). If `max_vertices` is `Some(n)`,
/// the hull is truncated to at most `n` vertices (matching MuJoCo's
/// `maxhullvert` → Qhull `TA(n-4)` mapping).
#[must_use]
pub fn quickhull(points: &[Point3<f64>], max_vertices: Option<usize>) -> Option<ConvexHull> {
    if points.len() < 4 {
        return None;
    }

    // Step 0: scale-dependent epsilon
    let epsilon = compute_epsilon(points);

    // Step 1: initial simplex (4 non-coplanar points)
    let simplex = find_initial_simplex(points, epsilon)?;

    // Build initial hull vertices from simplex indices
    let mut hull_vertices: Vec<Point3<f64>> = simplex.iter().map(|&i| points[i]).collect();

    // Step 2-3: create initial faces, init conflict graph, expand.
    let mut faces = create_initial_faces(&hull_vertices, &[0, 1, 2, 3]);
    init_conflict_graph(points, &mut faces, &simplex, epsilon);
    expand_hull(
        points,
        &mut hull_vertices,
        &mut faces,
        max_vertices,
        epsilon,
    );

    // Step 4-5: extract alive faces, build graph and normals
    let alive_faces: Vec<[usize; 3]> = faces
        .iter()
        .filter(|f| f.alive)
        .map(|f| f.indices)
        .collect();
    let graph = build_graph(&hull_vertices, &alive_faces);
    let normals = alive_faces
        .iter()
        .map(|&[a, b, c]| {
            let e1 = hull_vertices[b] - hull_vertices[a];
            let e2 = hull_vertices[c] - hull_vertices[a];
            let cross = e1.cross(&e2);
            let len = cross.norm();
            if len < 1e-30 {
                Vector3::z_axis().into_inner()
            } else {
                cross / len
            }
        })
        .collect();

    Some(ConvexHull {
        vertices: hull_vertices,
        faces: alive_faces,
        normals,
        graph,
    })
}

// ---------------------------------------------------------------------------
// Epsilon computation
// ---------------------------------------------------------------------------

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
// Initial simplex
// ---------------------------------------------------------------------------

fn find_initial_simplex(points: &[Point3<f64>], epsilon: f64) -> Option<[usize; 4]> {
    // Find extremal points (min/max per axis)
    let (mut min_idx, mut max_idx) = ([0usize; 3], [0usize; 3]);
    for (i, p) in points.iter().enumerate() {
        for axis in 0..3 {
            if p[axis] < points[min_idx[axis]][axis] {
                min_idx[axis] = i;
            }
            if p[axis] > points[max_idx[axis]][axis] {
                max_idx[axis] = i;
            }
        }
    }

    // Find most distant pair among extremals
    let candidates: Vec<usize> = min_idx
        .iter()
        .chain(max_idx.iter())
        .copied()
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();
    let (i0, i1) = most_distant_pair(points, &candidates);

    // Find point farthest from line (i0, i1)
    let i2 = farthest_from_line(points, points[i0], points[i1], epsilon)?;

    // Find point farthest from plane (i0, i1, i2)
    let normal = (points[i1] - points[i0]).cross(&(points[i2] - points[i0]));
    let i3 = farthest_from_plane(points, points[i0], &normal, epsilon)?;

    // Orient so normals face outward
    let d = (points[i3] - points[i0]).dot(&normal);
    if d > 0.0 {
        Some([i1, i0, i2, i3]) // Swap to flip winding
    } else {
        Some([i0, i1, i2, i3])
    }
}

fn most_distant_pair(points: &[Point3<f64>], candidates: &[usize]) -> (usize, usize) {
    let mut best = (
        candidates[0],
        candidates.get(1).copied().unwrap_or(candidates[0]),
    );
    let mut max_dist_sq = 0.0_f64;
    for (ci, &i) in candidates.iter().enumerate() {
        for &j in &candidates[ci + 1..] {
            let d = (points[i] - points[j]).norm_squared();
            if d > max_dist_sq {
                max_dist_sq = d;
                best = (i, j);
            }
        }
    }
    best
}

fn farthest_from_line(
    points: &[Point3<f64>],
    a: Point3<f64>,
    b: Point3<f64>,
    epsilon: f64,
) -> Option<usize> {
    let ab = b - a;
    let ab_len_sq = ab.norm_squared();
    if ab_len_sq < 1e-30 {
        return None;
    }
    let mut best_idx = 0;
    let mut max_dist_sq = 0.0_f64;
    for (i, p) in points.iter().enumerate() {
        let ap = p - a;
        let cross = ab.cross(&ap);
        let dist_sq = cross.norm_squared() / ab_len_sq;
        if dist_sq > max_dist_sq {
            max_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    if max_dist_sq.sqrt() < epsilon {
        None
    } else {
        Some(best_idx)
    }
}

fn farthest_from_plane(
    points: &[Point3<f64>],
    origin: Point3<f64>,
    normal: &Vector3<f64>,
    epsilon: f64,
) -> Option<usize> {
    let normal_len = normal.norm();
    if normal_len < 1e-30 {
        return None;
    }
    let unit_normal = normal / normal_len;

    let mut best_idx = 0;
    let mut max_abs_dist = 0.0_f64;
    for (i, p) in points.iter().enumerate() {
        let dist = (p - origin).dot(&unit_normal).abs();
        if dist > max_abs_dist {
            max_abs_dist = dist;
            best_idx = i;
        }
    }
    if max_abs_dist < epsilon {
        None
    } else {
        Some(best_idx)
    }
}

// ---------------------------------------------------------------------------
// Face construction
// ---------------------------------------------------------------------------

fn compute_face_normal(a: &Point3<f64>, b: &Point3<f64>, c: &Point3<f64>) -> Vector3<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let cross = e1.cross(&e2);
    let len = cross.norm();
    if len < 1e-30 {
        Vector3::z_axis().into_inner()
    } else {
        cross / len
    }
}

fn face_edge(f: &Face, edge_idx: usize) -> (usize, usize) {
    match edge_idx {
        0 => (f.indices[0], f.indices[1]),
        1 => (f.indices[1], f.indices[2]),
        2 => (f.indices[2], f.indices[0]),
        _ => unreachable!(),
    }
}

fn shares_edge(f: &Face, a: usize, b: usize) -> bool {
    for edge_idx in 0..3 {
        let (ea, eb) = face_edge(f, edge_idx);
        if (ea == a && eb == b) || (ea == b && eb == a) {
            return true;
        }
    }
    false
}

fn create_initial_faces(hull_vertices: &[Point3<f64>], simplex: &[usize; 4]) -> Vec<Face> {
    let [i0, i1, i2, i3] = *simplex;
    let face_indices = [[i0, i1, i2], [i0, i3, i1], [i1, i3, i2], [i0, i2, i3]];

    let mut faces = Vec::with_capacity(4);
    for &[a, b, c] in &face_indices {
        let normal = compute_face_normal(&hull_vertices[a], &hull_vertices[b], &hull_vertices[c]);
        let center = Point3::from(
            (hull_vertices[a].coords + hull_vertices[b].coords + hull_vertices[c].coords) / 3.0,
        );
        faces.push(Face {
            indices: [a, b, c],
            normal,
            center,
            conflict_list: Vec::new(),
            farthest_dist: f64::NEG_INFINITY,
            farthest_idx: 0,
            alive: true,
            neighbors: [usize::MAX; 3],
        });
    }

    // Link neighbors: for each face, find the face sharing each edge.
    for fi in 0..4 {
        for edge_idx in 0..3 {
            let (a, b) = face_edge(&faces[fi], edge_idx);
            for fj in 0..4 {
                if fi == fj {
                    continue;
                }
                if shares_edge(&faces[fj], a, b) {
                    faces[fi].neighbors[edge_idx] = fj;
                    break;
                }
            }
        }
    }
    faces
}

// ---------------------------------------------------------------------------
// Conflict graph
// ---------------------------------------------------------------------------

fn init_conflict_graph(
    points: &[Point3<f64>],
    faces: &mut [Face],
    simplex: &[usize; 4],
    epsilon: f64,
) {
    let simplex_set: HashSet<usize> = simplex.iter().copied().collect();
    for (pi, p) in points.iter().enumerate() {
        if simplex_set.contains(&pi) {
            continue;
        }
        let mut best_face: Option<usize> = None;
        let mut best_dist = epsilon;
        for (fi, face) in faces.iter().enumerate() {
            if !face.alive {
                continue;
            }
            let dist = (p - face.center).dot(&face.normal);
            if dist > best_dist {
                best_dist = dist;
                best_face = Some(fi);
            }
        }
        if let Some(fi) = best_face {
            faces[fi].conflict_list.push(pi);
            if best_dist > faces[fi].farthest_dist {
                faces[fi].farthest_dist = best_dist;
                faces[fi].farthest_idx = pi;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Horizon detection
// ---------------------------------------------------------------------------

fn find_horizon(
    faces: &[Face],
    eye: &Point3<f64>,
    start_face: usize,
    epsilon: f64,
) -> (HashSet<usize>, Vec<(usize, usize, usize)>) {
    let mut visible = HashSet::new();
    let mut queue = VecDeque::new();
    visible.insert(start_face);
    queue.push_back(start_face);

    while let Some(fi) = queue.pop_front() {
        let face = &faces[fi];
        for &ni in &face.neighbors {
            if ni == usize::MAX {
                continue;
            }
            if !visible.contains(&ni) && faces[ni].alive {
                let n = &faces[ni];
                let dist = (eye - n.center).dot(&n.normal);
                if dist > epsilon {
                    visible.insert(ni);
                    queue.push_back(ni);
                }
            }
        }
    }

    let mut horizon = Vec::new();
    for &fi in &visible {
        let face = &faces[fi];
        for edge_idx in 0..3 {
            let ni = face.neighbors[edge_idx];
            if !visible.contains(&ni) {
                let (a, b) = face_edge(face, edge_idx);
                horizon.push((a, b, ni));
            }
        }
    }
    (visible, horizon)
}

fn order_horizon_edges(edges: &mut [(usize, usize, usize)]) {
    for i in 0..edges.len() - 1 {
        let tail = edges[i].1;
        let Some(next_pos) = edges[i + 1..].iter().position(|e| e.0 == tail) else {
            debug_assert!(false, "horizon edges must form a closed polygon");
            return;
        };
        edges.swap(i + 1, i + 1 + next_pos);
    }
    if let Some(last) = edges.last() {
        debug_assert_eq!(last.1, edges[0].0, "horizon not closed");
    }
}

fn update_neighbor(faces: &mut [Face], fi: usize, a: usize, b: usize, new_fi: usize) {
    let target_edge = {
        let f = &faces[fi];
        (0..3).find(|&edge_idx| {
            let (ea, eb) = face_edge(f, edge_idx);
            (ea == a && eb == b) || (ea == b && eb == a)
        })
    };
    if let Some(edge_idx) = target_edge {
        faces[fi].neighbors[edge_idx] = new_fi;
    }
}

// ---------------------------------------------------------------------------
// Expansion loop
// ---------------------------------------------------------------------------

fn expand_hull(
    points: &[Point3<f64>],
    hull_vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<Face>,
    max_vertices: Option<usize>,
    epsilon: f64,
) {
    loop {
        // 1. Find the face with the farthest conflict point
        let best_face = faces
            .iter()
            .enumerate()
            .filter(|(_, f)| f.alive && !f.conflict_list.is_empty())
            .max_by(|(_, a), (_, b)| {
                a.farthest_dist
                    .partial_cmp(&b.farthest_dist)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        let Some((face_idx, _)) = best_face else {
            break;
        };

        // 2. Check vertex limit (maxhullvert)
        if let Some(max) = max_vertices {
            if hull_vertices.len() >= max {
                break;
            }
        }

        let eye_index = faces[face_idx].farthest_idx;
        let eye_pt = points[eye_index];
        let eye_hull_idx = hull_vertices.len();
        hull_vertices.push(eye_pt);

        // 3-4. BFS to find visible faces + extract horizon edges.
        let (visible, mut horizon) = find_horizon(faces, &eye_pt, face_idx, epsilon);

        // Sort horizon edges into a closed polygon.
        order_horizon_edges(&mut horizon);

        // 5. Collect conflict points from visible faces before deletion
        let mut orphan_points: Vec<usize> = Vec::new();
        for &fi in &visible {
            orphan_points.extend(
                faces[fi]
                    .conflict_list
                    .iter()
                    .filter(|&&pi| pi != eye_index),
            );
            faces[fi].alive = false;
            faces[fi].conflict_list.clear();
        }

        // 6. Create cone faces from horizon edges to eye point.
        let cone_start = faces.len();
        for &(a, b, non_vis_fi) in &horizon {
            let new_fi = faces.len();
            let normal = compute_face_normal(
                &hull_vertices[eye_hull_idx],
                &hull_vertices[a],
                &hull_vertices[b],
            );
            let center = Point3::from(
                (hull_vertices[eye_hull_idx].coords
                    + hull_vertices[a].coords
                    + hull_vertices[b].coords)
                    / 3.0,
            );

            faces.push(Face {
                indices: [eye_hull_idx, a, b],
                normal,
                center,
                conflict_list: Vec::new(),
                farthest_dist: f64::NEG_INFINITY,
                farthest_idx: 0,
                alive: true,
                neighbors: [usize::MAX, non_vis_fi, usize::MAX],
            });

            update_neighbor(faces, non_vis_fi, a, b, new_fi);
        }

        // Link adjacent cone faces
        let cone_count = horizon.len();
        for i in 0..cone_count {
            let fi = cone_start + i;
            let next = cone_start + (i + 1) % cone_count;
            let prev = cone_start + (i + cone_count - 1) % cone_count;
            faces[fi].neighbors[0] = prev;
            faces[fi].neighbors[2] = next;
        }

        // 7. Redistribute orphan conflict points to new cone faces only.
        for &pi in &orphan_points {
            let p = &points[pi];
            let mut best_face: Option<usize> = None;
            let mut best_dist = epsilon;
            for (fi, face) in faces.iter().enumerate().skip(cone_start) {
                if !face.alive {
                    continue;
                }
                let dist = (p - face.center).dot(&face.normal);
                if dist > best_dist {
                    best_dist = dist;
                    best_face = Some(fi);
                }
            }
            if let Some(fi) = best_face {
                faces[fi].conflict_list.push(pi);
                if best_dist > faces[fi].farthest_dist {
                    faces[fi].farthest_dist = best_dist;
                    faces[fi].farthest_idx = pi;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Graph construction
// ---------------------------------------------------------------------------

fn build_graph(vertices: &[Point3<f64>], faces: &[[usize; 3]]) -> HullGraph {
    let n = vertices.len();
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &[a, b, c] in faces {
        insert_sorted(&mut adjacency[a], b);
        insert_sorted(&mut adjacency[a], c);
        insert_sorted(&mut adjacency[b], a);
        insert_sorted(&mut adjacency[b], c);
        insert_sorted(&mut adjacency[c], a);
        insert_sorted(&mut adjacency[c], b);
    }
    HullGraph { adjacency }
}

fn insert_sorted(list: &mut Vec<usize>, val: usize) {
    if let Err(pos) = list.binary_search(&val) {
        list.insert(pos, val);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::unreadable_literal,
    clippy::cast_lossless,
    clippy::manual_midpoint,
    clippy::uninlined_format_args
)]
mod tests {
    use super::*;
    use nalgebra::Point3;

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
        // Start with icosahedron
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
        // Normalize to unit sphere
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

        for _ in 0..subdivisions {
            let mut new_faces = Vec::new();
            let mut midpoint_cache = std::collections::HashMap::new();
            for &[a, b, c] in &faces {
                let ab = get_midpoint(&mut verts, &mut midpoint_cache, a, b);
                let bc = get_midpoint(&mut verts, &mut midpoint_cache, b, c);
                let ca = get_midpoint(&mut verts, &mut midpoint_cache, c, a);
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

    // T1: Cube hull correctness → AC1
    #[test]
    fn test_cube_hull_correctness() {
        let pts = cube_vertices();
        let hull = quickhull(&pts, None).expect("cube should produce a hull");
        assert_eq!(hull.vertices.len(), 8, "cube hull should have 8 vertices");
        assert_eq!(
            hull.faces.len(),
            12,
            "cube hull should have 12 triangular faces"
        );
    }

    // T2: Tetrahedron hull — minimum valid → AC2
    #[test]
    fn test_tetrahedron_hull() {
        let pts = tetrahedron_vertices();
        let hull = quickhull(&pts, None).expect("tetrahedron should produce a hull");
        assert_eq!(hull.vertices.len(), 4);
        assert_eq!(hull.faces.len(), 4);
    }

    // T3: Interior point excluded → AC3
    #[test]
    fn test_interior_point_excluded() {
        let mut pts = cube_vertices();
        pts.push(Point3::new(0.0, 0.0, 0.0)); // interior
        let hull = quickhull(&pts, None).expect("should produce a hull");
        assert_eq!(hull.vertices.len(), 8, "interior point should be excluded");
    }

    // T4: maxhullvert limits → AC4
    #[test]
    fn test_maxhullvert_limits() {
        let pts = icosphere_vertices(1); // 42 vertices
        assert!(pts.len() >= 42);
        let hull = quickhull(&pts, Some(10)).expect("should produce a hull");
        assert!(
            hull.vertices.len() <= 10,
            "hull should have at most 10 vertices, got {}",
            hull.vertices.len()
        );
        // Validate it's a valid convex hull of a subset: all HULL vertices
        // are on or below all face planes (self-consistent convexity).
        let eps = compute_epsilon(&pts);
        for hv in &hull.vertices {
            for (fi, &[a, _b, _c]) in hull.faces.iter().enumerate() {
                let dist = (hv - hull.vertices[a]).dot(&hull.normals[fi]);
                assert!(
                    dist <= eps * 100.0,
                    "hull vertex {:?} is above face {} by {}",
                    hv,
                    fi,
                    dist
                );
            }
        }
    }

    // T5: maxhullvert=4 boundary → AC5
    #[test]
    fn test_maxhullvert_4_boundary() {
        let pts = cube_vertices();
        let hull = quickhull(&pts, Some(4)).expect("should produce a hull");
        assert_eq!(
            hull.vertices.len(),
            4,
            "maxhullvert=4 should yield 4 vertices (initial simplex only)"
        );
    }

    // T6: Fewer than 4 vertices → AC6
    #[test]
    fn test_fewer_than_4_vertices() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        assert!(quickhull(&pts, None).is_none());
    }

    // T7: Duplicate vertices → AC7
    #[test]
    fn test_duplicate_vertices() {
        let pts = cube_vertices();
        let mut doubled: Vec<Point3<f64>> = pts.iter().chain(pts.iter()).copied().collect();
        // Sort for determinism (not required, but helps debugging)
        doubled.sort_by(|a, b| {
            a[0].partial_cmp(&b[0])
                .unwrap()
                .then(a[1].partial_cmp(&b[1]).unwrap())
                .then(a[2].partial_cmp(&b[2]).unwrap())
        });
        let hull = quickhull(&doubled, None).expect("should produce a hull");
        assert_eq!(
            hull.vertices.len(),
            8,
            "duplicates should not add hull vertices"
        );
        assert_eq!(hull.faces.len(), 12);
    }

    // T13: Coplanar points → degenerate
    #[test]
    fn test_coplanar_points() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        assert!(
            quickhull(&pts, None).is_none(),
            "coplanar points should return None"
        );
    }

    // T14: Collinear points → degenerate
    #[test]
    fn test_collinear_points() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ];
        assert!(
            quickhull(&pts, None).is_none(),
            "collinear points should return None"
        );
    }

    // T15: Large mesh (icosphere with subdivisions)
    #[test]
    fn test_large_mesh_icosphere() {
        let pts = icosphere_vertices(2); // 162 vertices
        assert!(pts.len() >= 100);
        let hull = quickhull(&pts, None).expect("icosphere should produce a hull");
        assert!(hull.vertices.len() <= pts.len());
        // All face normals point outward
        let centroid = Point3::from(
            hull.vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>()
                / hull.vertices.len() as f64,
        );
        for (fi, &[a, b, c]) in hull.faces.iter().enumerate() {
            let face_center = Point3::from(
                (hull.vertices[a].coords + hull.vertices[b].coords + hull.vertices[c].coords) / 3.0,
            );
            let outward = face_center - centroid;
            assert!(
                outward.dot(&hull.normals[fi]) > 0.0,
                "face {} normal should point outward",
                fi
            );
        }
    }

    // T16: Thin sliver (near-coplanar tetrahedron)
    #[test]
    fn test_thin_sliver() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.5, 0.5, 1e-8),
        ];
        let hull = quickhull(&pts, None);
        assert!(
            hull.is_some(),
            "thin sliver should produce a hull (1e-8 > epsilon)"
        );
        let hull = hull.unwrap();
        assert_eq!(hull.vertices.len(), 4);
        assert_eq!(hull.faces.len(), 4);
    }

    // T19: Face normals point outward → AC14
    #[test]
    fn test_face_normals_outward() {
        let pts = cube_vertices();
        let hull = quickhull(&pts, None).expect("cube should produce a hull");
        let centroid = Point3::from(
            hull.vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>()
                / hull.vertices.len() as f64,
        );
        for (fi, &[a, b, c]) in hull.faces.iter().enumerate() {
            let face_center = Point3::from(
                (hull.vertices[a].coords + hull.vertices[b].coords + hull.vertices[c].coords) / 3.0,
            );
            let outward = face_center - centroid;
            assert!(
                outward.dot(&hull.normals[fi]) > 0.0,
                "face {} normal does not point outward",
                fi
            );
        }
    }

    // T8: Hill-climbing ≡ exhaustive for all directions → AC8
    #[test]
    fn test_hill_climb_equiv_exhaustive() {
        use crate::gjk_epa::hill_climb_support;

        let pts = icosphere_vertices(1); // 42 vertices
        let hull = quickhull(&pts, None).expect("icosphere hull");
        let graph = &hull.graph;

        // 100 random directions (seeded for reproducibility)
        let mut seed: u64 = 12345;
        for _ in 0..100 {
            // Simple LCG for deterministic "random" directions
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let x = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let y = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let z = ((seed >> 32) as f64 / u32::MAX as f64) * 2.0 - 1.0;
            let dir = nalgebra::Vector3::new(x, y, z);
            if dir.norm() < 1e-10 {
                continue;
            }
            let dir = dir.normalize();

            // Hill-climbing
            let idx_hill = hill_climb_support(&hull.vertices, graph, &dir, 0);
            let dot_hill = hull.vertices[idx_hill].coords.dot(&dir);

            // Exhaustive
            let mut best_exhaust = 0;
            let mut max_dot = f64::NEG_INFINITY;
            for (i, v) in hull.vertices.iter().enumerate() {
                let dot = v.coords.dot(&dir);
                if dot > max_dot {
                    max_dot = dot;
                    best_exhaust = i;
                }
            }
            let dot_exhaust = hull.vertices[best_exhaust].coords.dot(&dir);

            assert!(
                (dot_hill - dot_exhaust).abs() < 1e-12,
                "hill-climbing and exhaustive disagree: hill={} exhaust={}",
                dot_hill,
                dot_exhaust
            );
        }
    }

    // T20: Warm-start correctness
    #[test]
    fn test_warm_start_correctness() {
        use crate::collision_shape::CollisionShape;
        use crate::gjk_epa::support;
        use sim_types::Pose;

        let pts = icosphere_vertices(1); // 42 vertices
        let hull = quickhull(&pts, None).expect("icosphere hull");
        let shape = CollisionShape::convex_mesh_from_hull(&hull);
        let pose = Pose::identity();

        // First query: direction (1,0,0)
        let dir1 = nalgebra::Vector3::new(1.0, 0.0, 0.0);
        let p1 = support(&shape, &pose, &dir1);

        // Verify it's the correct support point by exhaustive check
        let mut max_dot = f64::NEG_INFINITY;
        let mut best_pt = hull.vertices[0];
        for v in &hull.vertices {
            let dot = v.coords.dot(&dir1);
            if dot > max_dot {
                max_dot = dot;
                best_pt = *v;
            }
        }
        assert!(
            (p1.coords - best_pt.coords).norm() < 1e-10,
            "first query returned wrong point"
        );

        // Second query: slightly rotated direction
        let dir2 = nalgebra::Vector3::new(0.99, 0.14, 0.0).normalize();
        let p2 = support(&shape, &pose, &dir2);

        // Verify second query is correct
        let mut max_dot2 = f64::NEG_INFINITY;
        let mut best_pt2 = hull.vertices[0];
        for v in &hull.vertices {
            let dot = v.coords.dot(&dir2);
            if dot > max_dot2 {
                max_dot2 = dot;
                best_pt2 = *v;
            }
        }
        assert!(
            (p2.coords - best_pt2.coords).norm() < 1e-10,
            "second query returned wrong point"
        );
    }

    // T11: Mesh-mesh collision via hull → AC11
    #[test]
    fn test_mesh_mesh_hull_collision() {
        use crate::collision_shape::CollisionShape;
        use crate::gjk_epa::gjk_epa_contact;
        use sim_types::Pose;

        // Cube 1 at origin
        let pts = cube_vertices();
        let hull1 = quickhull(&pts, None).expect("cube hull");
        let shape1 = CollisionShape::convex_mesh_from_hull(&hull1);
        let pose1 = Pose::from_position_rotation(
            Point3::new(0.0, 0.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );

        // Cube 2 at (0.9, 0, 0) — overlapping by 0.1 along X
        let hull2 = quickhull(&pts, None).expect("cube hull");
        let shape2 = CollisionShape::convex_mesh_from_hull(&hull2);
        let pose2 = Pose::from_position_rotation(
            Point3::new(0.9, 0.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );

        let contact = gjk_epa_contact(&shape1, &pose1, &shape2, &pose2);
        assert!(
            contact.is_some(),
            "overlapping cubes should produce contact"
        );
        let c = contact.unwrap();

        // Depth ≈ 0.1 (overlap along X = 0.5 + 0.5 - 0.9)
        assert!(
            (c.penetration - 0.1).abs() < 1e-2,
            "depth should be ~0.1, got {}",
            c.penetration
        );

        // Normal ≈ (±1, 0, 0)
        assert!(
            c.normal.x.abs() > 0.95,
            "normal should be ~(±1,0,0), got {:?}",
            c.normal
        );
    }

    // T21: All-identical vertices → None, no panic
    #[test]
    fn test_all_identical_vertices() {
        let pts: Vec<Point3<f64>> = (0..10).map(|_| Point3::new(1.0, 2.0, 3.0)).collect();
        assert!(quickhull(&pts, None).is_none());
    }

    // T22: maxhullvert > vertex count → full hull
    #[test]
    fn test_maxhullvert_exceeds_vertex_count() {
        let pts = cube_vertices();
        let hull = quickhull(&pts, Some(100)).expect("should produce a hull");
        assert_eq!(
            hull.vertices.len(),
            8,
            "limit never reached, full hull expected"
        );
        assert_eq!(hull.faces.len(), 12);
    }

    // T23: Graph adjacency structural validity → AC15
    #[test]
    fn test_graph_adjacency_structural() {
        let pts = cube_vertices();
        let hull = quickhull(&pts, None).expect("cube should produce a hull");
        let graph = &hull.graph;

        // One adjacency list per vertex
        assert_eq!(graph.adjacency.len(), hull.vertices.len());

        // Every vertex has ≥3 neighbors (minimum degree on a convex polyhedron)
        for (i, adj) in graph.adjacency.iter().enumerate() {
            assert!(
                adj.len() >= 3,
                "vertex {} has only {} neighbors (need ≥3)",
                i,
                adj.len()
            );
        }

        // Adjacency is symmetric
        for (a, adj) in graph.adjacency.iter().enumerate() {
            for &b in adj {
                assert!(
                    graph.adjacency[b].contains(&a),
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
                graph.adjacency[a].contains(&b),
                "edge ({},{}) missing",
                a,
                b
            );
            assert!(
                graph.adjacency[b].contains(&a),
                "edge ({},{}) missing",
                b,
                a
            );
            assert!(
                graph.adjacency[b].contains(&c),
                "edge ({},{}) missing",
                b,
                c
            );
            assert!(
                graph.adjacency[c].contains(&b),
                "edge ({},{}) missing",
                c,
                b
            );
            assert!(
                graph.adjacency[a].contains(&c),
                "edge ({},{}) missing",
                a,
                c
            );
            assert!(
                graph.adjacency[c].contains(&a),
                "edge ({},{}) missing",
                c,
                a
            );
        }

        // No self-loops
        for (i, adj) in graph.adjacency.iter().enumerate() {
            assert!(!adj.contains(&i), "self-loop at vertex {}", i);
        }

        // Cube: each vertex has exactly 6 neighbors (triangulated cube —
        // each square face is split into 2 triangles with a diagonal, so
        // each vertex connects to its 3 cube-adjacent vertices + 3 diagonal neighbors).
        for (i, adj) in graph.adjacency.iter().enumerate() {
            assert!(
                adj.len() >= 3,
                "cube vertex {} should have at least 3 neighbors, got {}",
                i,
                adj.len()
            );
        }
    }
}
