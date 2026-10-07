//! Convex hull computation via Quickhull algorithm.
//!
//! Computes the convex hull of a 3D point set, producing a [`ConvexHull`] with
//! vertices, triangulated faces, outward normals, and a vertex adjacency graph
//! for O(√n) hill-climbing support queries.
//!
//! `MuJoCo` ref: `mjCMesh::MakeGraph()` in `user_mesh.cc` delegates to the Qhull
//! library. CortenForge implements Quickhull directly in pure Rust.

use nalgebra::{Point3, Vector3};

use crate::{Aabb, Bounded};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Convex hull with adjacency graph for GJK hill-climbing.
///
/// Stores the hull geometry (vertices, triangulated faces, outward normals)
/// and a vertex adjacency graph for O(√n) hill-climbing support queries.
///
/// Replaces: `sim_core::ConvexHull` + `sim_core::HullGraph`
///
/// # Invariants
///
/// - All face indices are valid indices into `vertices`, and every vertex is
///   used by at least one face.
/// - All adjacency indices are valid indices into `vertices`.
/// - Faces are CCW-wound (outward normal follows right-hand rule).
/// - `normals.len() == faces.len()`.
/// - `adjacency.len() == vertices.len()`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConvexHull {
    /// Hull vertices in local frame.
    pub vertices: Vec<Point3<f64>>,

    /// Triangulated hull faces as index triples into `vertices`.
    /// CCW winding when viewed from outside.
    pub faces: Vec<[u32; 3]>,

    /// Outward unit normals, one per face (same order as `faces`).
    pub normals: Vec<Vector3<f64>>,

    /// Vertex adjacency graph for hill-climbing support queries.
    /// `adjacency[i]` = sorted list of vertex indices adjacent to vertex `i`.
    pub adjacency: Vec<Vec<u32>>,
}

impl ConvexHull {
    /// Returns the number of hull vertices.
    #[must_use]
    pub const fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the number of hull faces.
    #[must_use]
    pub const fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Returns the support point in the given direction via hill-climbing.
    ///
    /// Uses the adjacency graph for O(√n) queries on convex polyhedra.
    /// Falls back to exhaustive search if the adjacency graph is empty or
    /// has fewer than 10 vertices (matching `MuJoCo`'s `mjMESH_HILLCLIMB_MIN`).
    ///
    /// Returns the origin if the hull has no vertices.
    #[must_use]
    pub fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        if self.vertices.is_empty() {
            return Point3::origin();
        }
        let idx = self.support_index(direction);
        self.vertices[idx]
    }

    /// Returns the index of the support point in the given direction.
    ///
    /// Uses hill-climbing if the hull has ≥ 10 vertices and a valid adjacency
    /// graph; exhaustive search otherwise.
    #[must_use]
    pub fn support_index(&self, direction: &Vector3<f64>) -> usize {
        const HILL_CLIMB_MIN: usize = 10;

        if self.vertices.is_empty() {
            return 0;
        }

        if self.vertices.len() >= HILL_CLIMB_MIN && self.adjacency.len() == self.vertices.len() {
            self.hill_climb(direction, 0)
        } else {
            self.exhaustive_support(direction)
        }
    }

    /// Hill-climbing support query starting from `start`.
    ///
    /// Steepest-ascent: at each vertex, scan all neighbors and move to the one
    /// with the highest dot product. Converges when no neighbor improves.
    ///
    /// # Panics
    ///
    /// Debug-asserts that `start < vertices.len()`.
    #[must_use]
    pub fn hill_climb(&self, direction: &Vector3<f64>, start: usize) -> usize {
        debug_assert!(start < self.vertices.len());

        let mut current = start.min(self.vertices.len().saturating_sub(1));
        let mut current_dot = self.vertices[current].coords.dot(direction);

        loop {
            let mut improved = false;
            if let Some(neighbors) = self.adjacency.get(current) {
                for &neighbor in neighbors {
                    let ni = neighbor as usize;
                    if ni >= self.vertices.len() {
                        continue;
                    }
                    let dot = self.vertices[ni].coords.dot(direction);
                    if dot > current_dot {
                        current = ni;
                        current_dot = dot;
                        improved = true;
                    }
                }
            }
            if !improved {
                break;
            }
        }
        current
    }

    /// Exhaustive O(n) support query.
    fn exhaustive_support(&self, direction: &Vector3<f64>) -> usize {
        let mut best = 0;
        let mut max_dot = f64::NEG_INFINITY;
        for (i, v) in self.vertices.iter().enumerate() {
            let dot = v.coords.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                best = i;
            }
        }
        best
    }
}

impl Bounded for ConvexHull {
    fn aabb(&self) -> Aabb {
        Aabb::from_points(self.vertices.iter())
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute the convex hull of a point set using the Quickhull algorithm.
///
/// Returns `None` if fewer than 4 non-degenerate (non-coplanar) points exist.
/// If `max_vertices` is `Some(n)`, the hull is truncated to at most `n`
/// vertices (matching `MuJoCo`'s `maxhullvert`).
#[must_use]
pub fn convex_hull(points: &[Point3<f64>], max_vertices: Option<usize>) -> Option<ConvexHull> {
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
    init_conflict_graph(points, &hull_vertices, &mut faces, &simplex);
    expand_hull(points, &mut hull_vertices, &mut faces, max_vertices);

    // Step 4-5: extract alive faces, build graph and normals.
    let alive_faces: Vec<[usize; 3]> = faces
        .iter()
        .filter(|f| f.alive)
        .map(|f| f.indices)
        .collect();
    let (hull_vertices, alive_faces) = drop_unused_vertices(&hull_vertices, &alive_faces);

    let adjacency = build_adjacency(&hull_vertices, &alive_faces);

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

    // Convert usize faces to u32 at output boundary.
    // Convex hulls have at most thousands of vertices — u32 overflow is impossible.
    #[allow(clippy::cast_possible_truncation)]
    let u32_faces: Vec<[u32; 3]> = alive_faces
        .iter()
        .map(|&[a, b, c]| [a as u32, b as u32, c as u32])
        .collect();

    Some(ConvexHull {
        vertices: hull_vertices,
        faces: u32_faces,
        normals,
        adjacency,
    })
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
    // Ascending and deduplicated: `most_distant_pair` keeps the first of equally
    // distant pairs, so the candidates' order must not come from a hash set.
    let mut candidates: Vec<usize> = min_idx.iter().chain(max_idx.iter()).copied().collect();
    candidates.sort_unstable();
    candidates.dedup();
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

const fn face_edge(f: &Face, edge_idx: usize) -> (usize, usize) {
    match edge_idx {
        0 => (f.indices[0], f.indices[1]),
        1 => (f.indices[1], f.indices[2]),
        // edge_idx is always 0, 1, or 2 — called in `for edge_idx in 0..3`
        _ => (f.indices[2], f.indices[0]),
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

/// Whether `p` lies strictly outside `face`, decided exactly (Shewchuk's
/// `orient3d`). A float distance misjudges a point within rounding of a
/// near-coplanar face: the visible region then splits, a face the new point
/// can see survives, and the hull is no longer convex.
fn is_outside(hull_vertices: &[Point3<f64>], face: &Face, p: &Point3<f64>) -> bool {
    let coord = |q: &Point3<f64>| robust::Coord3D {
        x: q.x,
        y: q.y,
        z: q.z,
    };
    let [a, b, c] = face.indices.map(|i| coord(&hull_vertices[i]));
    // Negative: `p` is on the side the face's counter-clockwise winding faces.
    robust::orient3d(a, b, c, coord(p)) < 0.0
}

/// Give point `pi` to the alive face (from `first_face` on) that it lies
/// outside of and farthest from; a point outside none of them is inside the
/// hull and is dropped. The float distance only ranks the faces it is outside.
fn assign_conflict(
    points: &[Point3<f64>],
    hull_vertices: &[Point3<f64>],
    faces: &mut [Face],
    first_face: usize,
    pi: usize,
) {
    let p = &points[pi];
    let mut best: Option<(usize, f64)> = None;
    for (fi, face) in faces.iter().enumerate().skip(first_face) {
        if !face.alive || !is_outside(hull_vertices, face, p) {
            continue;
        }
        let dist = (p - face.center).dot(&face.normal);
        if best.is_none_or(|(_, farthest)| dist > farthest) {
            best = Some((fi, dist));
        }
    }
    if let Some((fi, dist)) = best {
        let face = &mut faces[fi];
        face.conflict_list.push(pi);
        if dist > face.farthest_dist {
            face.farthest_dist = dist;
            face.farthest_idx = pi;
        }
    }
}

fn init_conflict_graph(
    points: &[Point3<f64>],
    hull_vertices: &[Point3<f64>],
    faces: &mut [Face],
    simplex: &[usize; 4],
) {
    for pi in (0..points.len()).filter(|pi| !simplex.contains(pi)) {
        assign_conflict(points, hull_vertices, faces, 0, pi);
    }
}

// ---------------------------------------------------------------------------
// Horizon detection
// ---------------------------------------------------------------------------

fn find_horizon(
    hull_vertices: &[Point3<f64>],
    faces: &[Face],
    eye: &Point3<f64>,
    start_face: usize,
) -> (Vec<usize>, Vec<(usize, usize, usize)>) {
    // `visible` in BFS order: its order decides the horizon's first edge, hence
    // the cone faces' order and the orphan points' order — never a hash order.
    let mut is_visible = vec![false; faces.len()];
    let mut visible = vec![start_face];
    is_visible[start_face] = true;
    let mut head = 0;
    while head < visible.len() {
        let face = &faces[visible[head]];
        head += 1;
        for &ni in &face.neighbors {
            if ni == usize::MAX {
                continue;
            }
            if !is_visible[ni] && faces[ni].alive && is_outside(hull_vertices, &faces[ni], eye) {
                is_visible[ni] = true;
                visible.push(ni);
            }
        }
    }

    let mut horizon = Vec::new();
    for &fi in &visible {
        let face = &faces[fi];
        for edge_idx in 0..3 {
            let ni = face.neighbors[edge_idx];
            if ni == usize::MAX || !is_visible[ni] {
                let (a, b) = face_edge(face, edge_idx);
                horizon.push((a, b, ni));
            }
        }
    }
    (visible, horizon)
}

fn order_horizon_edges(edges: &mut [(usize, usize, usize)]) {
    for i in 0..edges.len().saturating_sub(1) {
        let tail = edges[i].1;
        let next_pos = edges[i + 1..].iter().position(|e| e.0 == tail);
        let Some(pos) = next_pos else {
            debug_assert!(false, "horizon edges must form a closed polygon");
            return;
        };
        edges.swap(i + 1, i + 1 + pos);
    }
    if let (Some(last), Some(first)) = (edges.last(), edges.first()) {
        debug_assert_eq!(last.1, first.0, "horizon not closed");
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
        if let Some(max) = max_vertices
            && hull_vertices.len() >= max
        {
            break;
        }

        let eye_index = faces[face_idx].farthest_idx;
        let eye_pt = points[eye_index];
        let eye_hull_idx = hull_vertices.len();
        hull_vertices.push(eye_pt);

        // 3-4. BFS to find visible faces + extract horizon edges.
        let (visible, mut horizon) = find_horizon(hull_vertices, faces, &eye_pt, face_idx);

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
            assign_conflict(points, hull_vertices, faces, cone_start, pi);
        }
    }
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

/// The vertices some face uses, in the order they were added, and the faces
/// renumbered to them. A point added as an eye can end up inside a later
/// hull with no face of its own.
fn drop_unused_vertices(
    hull_vertices: &[Point3<f64>],
    faces: &[[usize; 3]],
) -> (Vec<Point3<f64>>, Vec<[usize; 3]>) {
    let mut used = vec![false; hull_vertices.len()];
    for &i in faces.iter().flatten() {
        used[i] = true;
    }
    let mut new_index = vec![usize::MAX; hull_vertices.len()];
    let mut vertices = Vec::with_capacity(hull_vertices.len());
    for (i, v) in hull_vertices.iter().enumerate().filter(|&(i, _)| used[i]) {
        new_index[i] = vertices.len();
        vertices.push(*v);
    }
    let faces = faces.iter().map(|f| f.map(|i| new_index[i])).collect();
    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Adjacency graph construction
// ---------------------------------------------------------------------------

// Convex hulls have at most thousands of vertices — u32 overflow is impossible.
#[allow(clippy::cast_possible_truncation)]
fn build_adjacency(vertices: &[Point3<f64>], faces: &[[usize; 3]]) -> Vec<Vec<u32>> {
    let n = vertices.len();
    let mut adjacency: Vec<Vec<u32>> = vec![Vec::new(); n];
    for &[a, b, c] in faces {
        insert_sorted(&mut adjacency[a], b as u32);
        insert_sorted(&mut adjacency[a], c as u32);
        insert_sorted(&mut adjacency[b], a as u32);
        insert_sorted(&mut adjacency[b], c as u32);
        insert_sorted(&mut adjacency[c], a as u32);
        insert_sorted(&mut adjacency[c], b as u32);
    }
    adjacency
}

fn insert_sorted(list: &mut Vec<u32>, val: u32) {
    if let Err(pos) = list.binary_search(&val) {
        list.insert(pos, val);
    }
}
