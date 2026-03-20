//! Mesh simplification via Garland-Heckbert quadric error metric edge collapse.
//!
//! Two entry points:
//! - [`simplify_mesh`] — reduce face count to a target.
//! - [`simplify_mesh_tolerance`] — collapse edges until the next collapse would
//!   exceed `max_deviation` from the true implicit surface.

use std::collections::{BinaryHeap, HashMap, HashSet};

use cf_geometry::IndexedMesh;
use nalgebra::{Matrix4, Point3, Vector4};

use crate::field_node::FieldNode;

// ── Quadric ──────────────────────────────────────────────────────────────

/// Symmetric 4×4 quadric matrix.
#[derive(Debug, Clone, Copy)]
struct Quadric {
    mat: Matrix4<f64>,
}

impl Quadric {
    const ZERO: Self = Self {
        mat: Matrix4::new(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ),
    };

    /// Build a quadric from a plane equation `ax + by + cz + d = 0`.
    #[allow(clippy::many_single_char_names)]
    fn from_plane(plane_a: f64, plane_b: f64, plane_c: f64, plane_d: f64) -> Self {
        let vec = Vector4::new(plane_a, plane_b, plane_c, plane_d);
        Self {
            mat: vec * vec.transpose(),
        }
    }

    fn add(&self, other: &Self) -> Self {
        Self {
            mat: self.mat + other.mat,
        }
    }

    /// Evaluate the quadric error for a point `[x, y, z, 1]`.
    fn error(&self, point: &Vector4<f64>) -> f64 {
        point.dot(&(self.mat * point))
    }

    /// Find the optimal point minimizing the quadric error.
    /// Returns `None` if the matrix is singular — caller should use midpoint.
    fn optimal_point(&self) -> Option<Point3<f64>> {
        // Replace the last row with [0, 0, 0, 1] to solve for the minimum.
        let mut solve_mat = self.mat;
        solve_mat[(3, 0)] = 0.0;
        solve_mat[(3, 1)] = 0.0;
        solve_mat[(3, 2)] = 0.0;
        solve_mat[(3, 3)] = 1.0;

        let det = solve_mat.determinant();
        if det.abs() < 1e-15 {
            return None;
        }

        let inv = solve_mat.try_inverse()?;
        let result = inv * Vector4::new(0.0, 0.0, 0.0, 1.0);
        Some(Point3::new(result.x, result.y, result.z))
    }
}

// ── Edge key ──────────────────────────────────────────────────────────────

/// Canonical edge key with `lo < hi`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Edge(u32, u32);

impl Edge {
    const fn new(a: u32, b: u32) -> Self {
        if a < b { Self(a, b) } else { Self(b, a) }
    }
}

// ── Priority queue entry ──────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct CollapseCandidate {
    cost: f64,
    edge: Edge,
    /// Generation counter — entries with stale generation are skipped.
    generation: u64,
}

impl PartialEq for CollapseCandidate {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl Eq for CollapseCandidate {}

impl PartialOrd for CollapseCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for CollapseCandidate {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse: min-heap by cost
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

// ── Working mesh representation ───────────────────────────────────────────

/// Working mesh for simplification with adjacency.
struct SimplMesh {
    positions: Vec<Point3<f64>>,
    /// Live face indices (3 vertex indices per face).
    faces: Vec<[u32; 3]>,
    /// Which faces each vertex is part of.
    vertex_faces: Vec<HashSet<usize>>,
    /// Quadric per vertex.
    quadrics: Vec<Quadric>,
    /// Set of live edges.
    edges: HashSet<Edge>,
    /// Edge generation counter for lazy deletion in priority queue.
    edge_gen: HashMap<Edge, u64>,
    /// Vertex mapping: vertex → vertex it was collapsed into (union-find).
    parent: Vec<u32>,
    /// Live face count.
    live_faces: usize,
}

impl SimplMesh {
    #[allow(clippy::cast_possible_truncation)]
    fn from_indexed(mesh: &IndexedMesh) -> Self {
        let n_verts = mesh.vertices.len();
        let n_faces = mesh.faces.len();

        let positions = mesh.vertices.clone();
        let faces = mesh.faces.clone();

        // Build vertex → face adjacency
        let mut vertex_faces = vec![HashSet::new(); n_verts];
        for (fi, face) in faces.iter().enumerate() {
            for &vi in face {
                vertex_faces[vi as usize].insert(fi);
            }
        }

        // Compute per-vertex quadrics from incident face planes
        let mut quadrics = vec![Quadric::ZERO; n_verts];
        for face in &faces {
            let v0 = &positions[face[0] as usize];
            let v1 = &positions[face[1] as usize];
            let v2 = &positions[face[2] as usize];

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let normal = edge1.cross(&edge2);
            let len = normal.norm();
            if len < f64::EPSILON {
                continue;
            }
            let unit_normal = normal / len;
            let dist = -unit_normal.dot(&v0.coords);
            let quad = Quadric::from_plane(unit_normal.x, unit_normal.y, unit_normal.z, dist);

            for &vi in face {
                quadrics[vi as usize] = quadrics[vi as usize].add(&quad);
            }
        }

        // Collect all edges
        let mut edges = HashSet::new();
        for face in &faces {
            edges.insert(Edge::new(face[0], face[1]));
            edges.insert(Edge::new(face[1], face[2]));
            edges.insert(Edge::new(face[2], face[0]));
        }

        let edge_gen: HashMap<Edge, u64> = edges.iter().map(|&edge| (edge, 0_u64)).collect();
        let parent: Vec<u32> = (0..n_verts as u32).collect();

        Self {
            positions,
            faces,
            vertex_faces,
            quadrics,
            edges,
            edge_gen,
            parent,
            live_faces: n_faces,
        }
    }

    /// Find root with path compression.
    fn find(&mut self, mut vertex: u32) -> u32 {
        while self.parent[vertex as usize] != vertex {
            let par = self.parent[vertex as usize];
            self.parent[vertex as usize] = self.parent[par as usize];
            vertex = self.parent[vertex as usize];
        }
        vertex
    }

    /// Check if an edge is on the mesh boundary (has only one incident face).
    fn is_boundary_edge(&self, edge: Edge) -> bool {
        let v0_faces = &self.vertex_faces[edge.0 as usize];
        let v1_faces = &self.vertex_faces[edge.1 as usize];
        let shared = v0_faces.intersection(v1_faces).count();
        shared < 2
    }

    /// Check if collapsing an edge would create a non-manifold situation.
    /// Uses the link condition: safe iff the common neighbors of both endpoints
    /// equals the number of shared faces.
    fn would_create_non_manifold(&self, edge: Edge) -> bool {
        let v0 = edge.0 as usize;
        let v1 = edge.1 as usize;

        let neighbors_v0 = self.vertex_neighbors(v0);
        let neighbors_v1 = self.vertex_neighbors(v1);

        let common: HashSet<u32> = neighbors_v0.intersection(&neighbors_v1).copied().collect();

        let shared_face_count = self.vertex_faces[v0]
            .intersection(&self.vertex_faces[v1])
            .count();

        if shared_face_count == 2 {
            common.len() != 2
        } else if shared_face_count == 1 {
            common.len() != 1
        } else {
            true
        }
    }

    fn vertex_neighbors(&self, vertex: usize) -> HashSet<u32> {
        let mut neighbors = HashSet::new();
        for &fi in &self.vertex_faces[vertex] {
            let face = &self.faces[fi];
            for &vi in face {
                let root = self.parent[vi as usize]; // peek without mutation
                if root as usize != vertex {
                    neighbors.insert(root);
                }
            }
        }
        neighbors
    }

    /// Compute collapse cost and target point for an edge.
    fn collapse_info(&self, edge: Edge) -> (f64, Point3<f64>) {
        let q_sum = self.quadrics[edge.0 as usize].add(&self.quadrics[edge.1 as usize]);

        let target = q_sum.optimal_point().unwrap_or_else(|| {
            let p0 = &self.positions[edge.0 as usize];
            let p1 = &self.positions[edge.1 as usize];
            Point3::from((p0.coords + p1.coords) * 0.5)
        });

        let point_vec = Vector4::new(target.x, target.y, target.z, 1.0);
        let cost = q_sum.error(&point_vec).max(0.0);

        (cost, target)
    }

    /// Build the initial priority queue.
    fn build_queue(&self) -> BinaryHeap<CollapseCandidate> {
        let mut heap = BinaryHeap::new();
        for &edge in &self.edges {
            if self.is_boundary_edge(edge) {
                continue;
            }
            let (cost, _) = self.collapse_info(edge);
            let generation = self.edge_gen.get(&edge).copied().unwrap_or(0);
            heap.push(CollapseCandidate {
                cost,
                edge,
                generation,
            });
        }
        heap
    }

    /// Collapse an edge: merge v1 into v0, update adjacency.
    #[allow(clippy::cast_possible_truncation)]
    fn collapse_edge(&mut self, edge: Edge, target: Point3<f64>) {
        let keep = edge.0;
        let remove = edge.1;

        self.positions[keep as usize] = target;
        self.quadrics[keep as usize] =
            self.quadrics[keep as usize].add(&self.quadrics[remove as usize]);
        self.parent[remove as usize] = keep;

        let remove_faces: Vec<usize> = self.vertex_faces[remove as usize].iter().copied().collect();

        for fi in &remove_faces {
            let face = &mut self.faces[*fi];

            for vi in face.iter_mut() {
                if *vi == remove {
                    *vi = keep;
                }
            }

            if face[0] == face[1] || face[1] == face[2] || face[0] == face[2] {
                self.vertex_faces[keep as usize].remove(fi);
                self.vertex_faces[remove as usize].remove(fi);
                for &vi in &self.faces[*fi] {
                    if vi != keep && vi != remove {
                        self.vertex_faces[vi as usize].remove(fi);
                    }
                }
                self.live_faces -= 1;
            } else {
                self.vertex_faces[keep as usize].insert(*fi);
            }
        }

        self.vertex_faces[remove as usize].clear();

        let keep_face_indices: Vec<usize> =
            self.vertex_faces[keep as usize].iter().copied().collect();

        self.edges
            .retain(|edge| edge.0 != remove && edge.1 != remove);

        for fi in &keep_face_indices {
            let face = self.faces[*fi];
            for edge in [
                Edge::new(face[0], face[1]),
                Edge::new(face[1], face[2]),
                Edge::new(face[2], face[0]),
            ] {
                if self.edges.insert(edge) {
                    self.edge_gen.insert(edge, 0);
                }
            }
        }

        self.edges.remove(&Edge::new(keep, remove));

        for fi in &keep_face_indices {
            let face = self.faces[*fi];
            for pair in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])] {
                let edge = Edge::new(pair.0, pair.1);
                if let Some(gen_count) = self.edge_gen.get_mut(&edge) {
                    *gen_count += 1;
                }
            }
        }
    }

    /// Re-queue updated edges around a vertex after a collapse.
    fn requeue_around(&self, vertex: u32, heap: &mut BinaryHeap<CollapseCandidate>) {
        let face_indices: Vec<usize> = self.vertex_faces[vertex as usize].iter().copied().collect();

        let mut seen = HashSet::new();
        for fi in &face_indices {
            let face = self.faces[*fi];
            for pair in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])] {
                let edge = Edge::new(pair.0, pair.1);
                if !seen.insert(edge) {
                    continue;
                }
                if !self.edges.contains(&edge) || self.is_boundary_edge(edge) {
                    continue;
                }
                let (cost, _) = self.collapse_info(edge);
                let generation = self.edge_gen.get(&edge).copied().unwrap_or(0);
                heap.push(CollapseCandidate {
                    cost,
                    edge,
                    generation,
                });
            }
        }
    }

    /// Convert back to `IndexedMesh`, compacting unused vertices.
    #[allow(clippy::cast_possible_truncation)]
    fn to_indexed_mesh(&self) -> IndexedMesh {
        let live_face_set: HashSet<usize> = (0..self.faces.len())
            .filter(|fi| {
                let face = &self.faces[*fi];
                face[0] != face[1]
                    && face[1] != face[2]
                    && face[0] != face[2]
                    && self.vertex_faces.iter().any(|vf| vf.contains(fi))
            })
            .collect();

        let mut used_verts: HashSet<u32> = HashSet::new();
        for &fi in &live_face_set {
            let face = &self.faces[fi];
            used_verts.insert(face[0]);
            used_verts.insert(face[1]);
            used_verts.insert(face[2]);
        }

        let mut sorted_verts: Vec<u32> = used_verts.into_iter().collect();
        sorted_verts.sort_unstable();

        let mut remap: HashMap<u32, u32> = HashMap::new();
        let mut new_verts = Vec::new();
        for (new_idx, &old_idx) in sorted_verts.iter().enumerate() {
            remap.insert(old_idx, new_idx as u32);
            new_verts.push(self.positions[old_idx as usize]);
        }

        let mut new_faces = Vec::new();
        let mut sorted_faces: Vec<usize> = live_face_set.into_iter().collect();
        sorted_faces.sort_unstable();

        for fi in sorted_faces {
            let face = &self.faces[fi];
            if let (Some(&idx_a), Some(&idx_b), Some(&idx_c)) = (
                remap.get(&face[0]),
                remap.get(&face[1]),
                remap.get(&face[2]),
            ) {
                new_faces.push([idx_a, idx_b, idx_c]);
            }
        }

        IndexedMesh::from_parts(new_verts, new_faces)
    }
}

// ── Public API ────────────────────────────────────────────────────────────

/// Simplify a mesh to the target face count using QEM edge collapse.
///
/// Returns a new mesh with approximately `target_faces` faces (within ±10%
/// for well-behaved inputs). Preserves manifold topology and boundary edges.
///
/// # Panics
///
/// Panics if `target_faces` is zero.
#[must_use]
pub fn simplify_mesh(mesh: &IndexedMesh, target_faces: usize) -> IndexedMesh {
    assert!(target_faces > 0, "target_faces must be > 0");

    if mesh.face_count() <= target_faces {
        return mesh.clone();
    }

    let mut sm = SimplMesh::from_indexed(mesh);
    let mut heap = sm.build_queue();

    while sm.live_faces > target_faces {
        let Some(candidate) = heap.pop() else {
            break;
        };

        let current_gen = sm.edge_gen.get(&candidate.edge).copied();
        if current_gen != Some(candidate.generation) {
            continue;
        }

        let v0 = sm.find(candidate.edge.0);
        let v1 = sm.find(candidate.edge.1);
        if v0 == v1 {
            continue;
        }

        let edge = Edge::new(v0, v1);
        if !sm.edges.contains(&edge) {
            continue;
        }

        if sm.is_boundary_edge(edge) {
            continue;
        }

        if sm.would_create_non_manifold(edge) {
            continue;
        }

        let (_, target) = sm.collapse_info(edge);
        sm.collapse_edge(edge, target);
        sm.requeue_around(edge.0, &mut heap);
    }

    sm.to_indexed_mesh()
}

/// Simplify a mesh until the next edge collapse would exceed `max_deviation`
/// from the true implicit surface.
///
/// Uses the field node to verify that each collapsed vertex stays within
/// the tolerance: `|field(v)| ≤ max_deviation`.
///
/// # Panics
///
/// Panics if `max_deviation` is not positive and finite.
#[must_use]
pub fn simplify_mesh_tolerance(
    mesh: &IndexedMesh,
    node: &FieldNode,
    max_deviation: f64,
) -> IndexedMesh {
    assert!(
        max_deviation > 0.0 && max_deviation.is_finite(),
        "max_deviation must be positive and finite, got {max_deviation}"
    );

    let mut sm = SimplMesh::from_indexed(mesh);
    let mut heap = sm.build_queue();

    loop {
        let Some(candidate) = heap.pop() else {
            break;
        };

        let current_gen = sm.edge_gen.get(&candidate.edge).copied();
        if current_gen != Some(candidate.generation) {
            continue;
        }

        let v0 = sm.find(candidate.edge.0);
        let v1 = sm.find(candidate.edge.1);
        if v0 == v1 {
            continue;
        }

        let edge = Edge::new(v0, v1);
        if !sm.edges.contains(&edge) {
            continue;
        }

        if sm.is_boundary_edge(edge) {
            continue;
        }

        if sm.would_create_non_manifold(edge) {
            continue;
        }

        let (_, target) = sm.collapse_info(edge);

        let field_val = node.evaluate(&target).abs();
        if field_val > max_deviation {
            continue;
        }

        sm.collapse_edge(edge, target);
        sm.requeue_around(edge.0, &mut heap);
    }

    sm.to_indexed_mesh()
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use std::collections::HashMap as StdHashMap;

    use crate::Solid;

    /// Check mesh topology: returns (watertight, manifold).
    fn check_topology(mesh: &IndexedMesh) -> (bool, bool) {
        let mut directed: StdHashMap<(u32, u32), usize> = StdHashMap::new();
        for face in &mesh.faces {
            for idx in 0..3 {
                *directed
                    .entry((face[idx], face[(idx + 1) % 3]))
                    .or_insert(0) += 1;
            }
        }

        let mut boundary = 0_usize;
        let mut non_manifold = 0_usize;
        for (&(va, vb), &count) in &directed {
            if count > 1 {
                non_manifold += 1;
            }
            if directed.get(&(vb, va)).copied().unwrap_or(0) == 0 {
                boundary += 1;
            }
        }

        (boundary == 0, non_manifold == 0)
    }

    #[test]
    fn simplify_sphere_reduces_face_count() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.5);
        let original_faces = mesh.face_count();
        let target = original_faces / 2;

        let simplified = simplify_mesh(&mesh, target);

        let lo = target * 8 / 10;
        let hi = target * 12 / 10;
        assert!(
            simplified.face_count() >= lo && simplified.face_count() <= hi,
            "Expected face count near {target} (±20%), got {}",
            simplified.face_count()
        );
    }

    #[test]
    fn simplify_sphere_preserves_topology() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.5);
        let target = mesh.face_count() / 2;

        let simplified = simplify_mesh(&mesh, target);

        let (watertight, manifold) = check_topology(&simplified);
        assert!(watertight, "Simplified mesh should be watertight");
        assert!(manifold, "Simplified mesh should be manifold");
    }

    #[test]
    fn simplify_sphere_preserves_volume() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.5);
        let original_vol = mesh.volume();
        let target = mesh.face_count() / 2;

        let simplified = simplify_mesh(&mesh, target);
        let simplified_vol = simplified.volume();

        let error = (simplified_vol - original_vol).abs() / original_vol;
        assert!(
            error < 0.10,
            "Volume error {:.1}% exceeds 10% after 50% face reduction \
             (original={original_vol:.2}, simplified={simplified_vol:.2})",
            error * 100.0
        );
    }

    #[test]
    fn simplify_cuboid_preserves_sharp_corners() {
        let cube = Solid::cuboid(nalgebra::Vector3::new(2.0, 2.0, 2.0));
        let mesh = cube.mesh_adaptive_par(0.3);
        let target = mesh.face_count() * 2 / 3;

        let simplified = simplify_mesh(&mesh, target);

        let true_corners: Vec<Point3<f64>> = [
            (-2.0, -2.0, -2.0),
            (-2.0, -2.0, 2.0),
            (-2.0, 2.0, -2.0),
            (-2.0, 2.0, 2.0),
            (2.0, -2.0, -2.0),
            (2.0, -2.0, 2.0),
            (2.0, 2.0, -2.0),
            (2.0, 2.0, 2.0),
        ]
        .iter()
        .map(|&(x, y, z)| Point3::new(x, y, z))
        .collect();

        for tc in &true_corners {
            let min_dist = simplified
                .vertices
                .iter()
                .map(|v| (v - tc).norm())
                .fold(f64::MAX, f64::min);
            assert!(
                min_dist < 0.5,
                "True corner {tc:?} is {min_dist:.3} from nearest simplified vertex (expected < 0.5)"
            );
        }
    }

    #[test]
    fn simplify_tolerance_stays_within_deviation() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.3);
        let max_dev = 0.2;

        let simplified = simplify_mesh_tolerance(&mesh, &sphere.node, max_dev);

        for v in &simplified.vertices {
            let field_val = sphere.node.evaluate(v).abs();
            assert!(
                field_val <= max_dev + 1e-10,
                "Vertex {v:?} has field deviation {field_val:.6} > max_deviation {max_dev}"
            );
        }

        assert!(
            simplified.face_count() < mesh.face_count(),
            "Tolerance simplification should reduce face count: {} vs {}",
            simplified.face_count(),
            mesh.face_count()
        );
    }

    #[test]
    fn simplify_tolerance_preserves_topology() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.3);
        let max_dev = 0.2;

        let simplified = simplify_mesh_tolerance(&mesh, &sphere.node, max_dev);

        let (watertight, manifold) = check_topology(&simplified);
        assert!(watertight, "Tolerance-simplified mesh should be watertight");
        assert!(manifold, "Tolerance-simplified mesh should be manifold");
    }

    #[test]
    fn simplify_already_small_is_noop() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(1.0);
        let original = mesh.face_count();

        let simplified = simplify_mesh(&mesh, original + 100);
        assert_eq!(simplified.face_count(), original);
    }

    #[test]
    fn simplify_aggressive_90_percent() {
        let sphere = Solid::sphere(5.0);
        let mesh = sphere.mesh_adaptive_par(0.5);
        let target = mesh.face_count() / 10; // 90% reduction

        let simplified = simplify_mesh(&mesh, target);

        let (watertight, manifold) = check_topology(&simplified);
        assert!(
            watertight,
            "Aggressively simplified mesh should be watertight"
        );
        assert!(manifold, "Aggressively simplified mesh should be manifold");
        assert!(
            simplified.face_count() > 4,
            "Even at 90% reduction, should keep more than a tetrahedron"
        );
    }

    #[test]
    fn simplify_dc_cuboid_preserves_corners() {
        // Use DC output (not MC) — DC should have sharper corners.
        use crate::dual_contouring::mesh_field_dc;
        use crate::field_node::FieldNode;

        let node = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(2.0, 2.0, 2.0),
        };
        let bounds = node.bounds().map(|b| b.expanded(0.5));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(cf_geometry::Aabb::empty()), 0.5);
        let target = mesh.face_count() * 2 / 3;

        let simplified = simplify_mesh(&mesh, target);

        let true_corners: Vec<Point3<f64>> = [
            (-2.0, -2.0, -2.0),
            (-2.0, -2.0, 2.0),
            (-2.0, 2.0, -2.0),
            (-2.0, 2.0, 2.0),
            (2.0, -2.0, -2.0),
            (2.0, -2.0, 2.0),
            (2.0, 2.0, -2.0),
            (2.0, 2.0, 2.0),
        ]
        .iter()
        .map(|&(x, y, z)| Point3::new(x, y, z))
        .collect();

        for tc in &true_corners {
            let min_dist = simplified
                .vertices
                .iter()
                .map(|v| (v - tc).norm())
                .fold(f64::MAX, f64::min);
            assert!(
                min_dist < 0.5,
                "DC corner {tc:?} is {min_dist:.3} from nearest simplified vertex (expected < 0.5)"
            );
        }
    }

    #[test]
    fn simplify_composed_shape() {
        let body = Solid::cuboid(nalgebra::Vector3::new(3.0, 3.0, 3.0));
        let hole = Solid::cylinder(1.0, 4.0);
        let shape = body.subtract(hole);
        let mesh = shape.mesh_adaptive_par(0.4);
        let target = mesh.face_count() * 2 / 3;

        let simplified = simplify_mesh(&mesh, target);

        let (watertight, manifold) = check_topology(&simplified);
        assert!(watertight, "Simplified composed shape should be watertight");
        assert!(manifold, "Simplified composed shape should be manifold");
    }
}
