//! A mesh's octree as MuJoCo 3.5.0's compiler makes it for a signed distance
//! field geom.
//!
//! It is `mjCOctree` (`user_objects.cc:549-1000`), its coefficients made as
//! `mjCMesh::TryCompile` makes them (`user_mesh.cc:774-830`), held as
//! MuJoCo's model holds it (`oct_aabb`, `oct_child`, `oct_depth`,
//! `oct_coeff`; `mjCModel::CopyObjects`, `user_model.cc:3281-3335`).
//!
//! The octree is made over the mesh's faces in the frame MuJoCo keeps the
//! mesh in, from its vertices before they are rounded to single precision.
//! Its root is the mesh's box grown by a tenth about its centre; a node is
//! split in eight where a face touches its box (`boxTriangle`, Schwarz and
//! Seidel's test), to six levels; then leaves are split until no leaf meets a
//! leaf more than one level finer (`BalanceOctree`), and the vertices of a
//! finer leaf that lie on a coarser one's face or edge are marked hanging
//! (`MarkHangingNodes`). Each vertex's coefficient is its signed distance to
//! the mesh (TriangleMeshDistance's) or, hanging, the mean of the coarser
//! vertices it hangs from; each node holds its eight corners'.
//! MuJoCo reads a distance inside the octree by trilinear interpolation of a
//! leaf's coefficients (`oct_distance`, `engine_collision_sdf.c:136-160`).

mod cxx_sort;
mod distance;

use std::collections::{HashMap, VecDeque};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// The levels a node is split to by the faces it meets (`MakeOctree`'s
/// `task.lev >= 6`).
const MAX_LEVEL: i32 = 6;

/// A mesh's octree as MuJoCo 3.5.0's model holds it, one entry per node in
/// the order MuJoCo makes them: the root, then each level's splits in turn,
/// then the balancing's.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MeshOctree {
    /// Each node's box as its centre and half sizes (`oct_aabb`), in the
    /// mesh's frame.
    pub aabb: Vec<[f64; 6]>,
    /// Each node's eight children by index, `-1` for none (`oct_child`): the
    /// child in slot `i` is the octant with `x` high where `i & 1`, `y` where
    /// `i & 2`, `z` where `i & 4`. A leaf has none.
    pub child: Vec<[i32; 8]>,
    /// Each node's level, the root's 0 (`oct_depth`).
    pub depth: Vec<i32>,
    /// Each node's eight corners' coefficients (`oct_coeff`), in the order of
    /// `child`'s slots: the corner's signed distance to the mesh, negative
    /// inside it, or, where the corner hangs on a coarser node's face or
    /// edge, the mean of the corners it hangs from.
    pub coeff: Vec<[f64; 8]>,
}

impl MeshOctree {
    /// Its number of nodes (MuJoCo's `mesh_octnum`).
    #[must_use]
    pub fn len(&self) -> usize {
        self.depth.len()
    }

    /// Whether it has no node; an octree MuJoCo makes always has its root.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.depth.is_empty()
    }
}

/// Why no octree was made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OctreeError {
    /// The mesh has no faces: TriangleMeshDistance prints "Empty triangle
    /// list" and exits the process there (`TriangleMeshDistance.h:344-347`).
    NoFaces,
    /// A hanging vertex's coefficient was asked before a vertex it hangs
    /// from had one: MuJoCo's "sdf coefficient computation failed"
    /// (`user_mesh.cc:807-809`).
    Coefficients,
}

impl std::fmt::Display for OctreeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NoFaces => "a mesh with no faces has no signed distance field",
            Self::Coefficients => "sdf coefficient computation failed",
        })
    }
}

impl std::error::Error for OctreeError {}

/// One node as it is made (`OctNode`).
#[derive(Debug, Clone, Copy)]
struct Node {
    level: i32,
    parent: Option<usize>,
    slot: usize,
    child: [i32; 8],
    vertid: [usize; 8],
    /// Its box as (min, max) (`aamm`).
    aamm: [f64; 6],
}

/// A node waiting in `MakeOctree`'s queue (`OctreeTask`): the faces its
/// parent met.
struct Task {
    faces: Vec<usize>,
    level: i32,
    node: usize,
}

type Triangle = [[f64; 3]; 3];

/// The octree as it is made (`mjCOctree`).
struct Builder {
    nodes: Vec<Node>,
    vertices: Vec<[f64; 3]>,
    /// Each vertex's index by its coordinates' bits, a zero's sign dropped
    /// ([`point_key`]). MuJoCo's `vert_map` hashes the bits and holds two
    /// points of one bucket equal within 1e-9 on each axis
    /// (`Point::operator==`, `user_objects.h:224-229`).
    vertex_ids: HashMap<[u64; 3], usize>,
    /// Each vertex's coarser vertices where it hangs (`hang_`), else empty.
    hang: Vec<Vec<usize>>,
}

/// The key of a point in [`Builder::vertex_ids`]: its coordinates' bits, a
/// negative zero taken as zero (libc++'s `hash<double>` gives both one hash,
/// and MuJoCo's equality holds them equal).
fn point_key(p: [f64; 3]) -> [u64; 3] {
    p.map(|x| (x + 0.0).to_bits())
}

/// The octree MuJoCo 3.5.0 makes over a mesh for a signed distance field
/// geom naming it.
///
/// It is made from the mesh's vertices and faces in the frame MuJoCo keeps
/// the mesh in, before they are rounded (`vert_`, `face_`), and the box over
/// those vertices (`aamm_`, as (min, max)): `mjCOctree::CreateOctree`, then
/// the coefficients `mjCMesh::TryCompile` gives its vertices.
///
/// # Errors
///
/// [`OctreeError::NoFaces`] where `faces` is empty; MuJoCo's
/// [`OctreeError::Coefficients`] (not seen made).
///
/// # Panics
///
/// If a face names a vertex past `vertices`.
#[allow(clippy::manual_midpoint)] // MuJoCo's `(a + b) / 2`, not `f64::midpoint`
pub fn make_octree(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    aamm: [f64; 6],
) -> Result<MeshOctree, OctreeError> {
    if faces.is_empty() {
        return Err(OctreeError::NoFaces);
    }
    let faces: Vec<[usize; 3]> = faces.iter().map(|f| f.map(|v| v as usize)).collect();
    // `Make` moves the faces to the octree's frame, which for a mesh is its
    // own (`ipos_` zero and `iquat_` the identity, never set):
    // `mjuu_rotVecQuat` copies each vertex, but gives a zero vector positive
    // zeros. That sign is not kept here: these faces are read only by
    // `box_triangle`, which compares and never divides, and a comparison
    // holds -0 equal to +0.
    let triangles: Vec<Triangle> = faces.iter().map(|f| f.map(|v| vertices[v])).collect();
    let center = [0, 1, 2].map(|i| (aamm[i] + aamm[i + 3]) / 2.0);
    let half = [0, 1, 2].map(|i| (aamm[i + 3] - aamm[i]) / 2.0);
    let root = [
        center[0] - 1.1 * half[0],
        center[1] - 1.1 * half[1],
        center[2] - 1.1 * half[2],
        center[0] + 1.1 * half[0],
        center[1] + 1.1 * half[1],
        center[2] + 1.1 * half[2],
    ];
    let mut builder = Builder {
        nodes: Vec::new(),
        vertices: Vec::new(),
        vertex_ids: HashMap::new(),
        hang: Vec::new(),
    };
    builder.make(&triangles, root);
    builder.mark_hanging();
    let coeffs = builder.coefficients(vertices, &faces)?;
    Ok(MeshOctree {
        aabb: builder
            .nodes
            .iter()
            .map(|n| {
                let a = n.aamm;
                [
                    (a[0] + a[3]) / 2.0,
                    (a[1] + a[4]) / 2.0,
                    (a[2] + a[5]) / 2.0,
                    (a[3] - a[0]) / 2.0,
                    (a[4] - a[1]) / 2.0,
                    (a[5] - a[2]) / 2.0,
                ]
            })
            .collect(),
        child: builder.nodes.iter().map(|n| n.child).collect(),
        depth: builder.nodes.iter().map(|n| n.level).collect(),
        coeff: builder
            .nodes
            .iter()
            .map(|n| n.vertid.map(|v| coeffs[v]))
            .collect(),
    })
}

impl Builder {
    /// `MakeOctree`: the root over `root`, then each node of the queue split
    /// where a face of its parent's touches its box, below the last level;
    /// then `BalanceOctree`.
    fn make(&mut self, triangles: &[Triangle], root: [f64; 6]) {
        let mut queue = VecDeque::new();
        self.nodes.push(Node {
            level: 0,
            parent: None,
            slot: 0,
            child: [-1; 8],
            vertid: [0; 8],
            aamm: root,
        });
        self.finish_node(0);
        queue.push_back(Task {
            faces: (0..triangles.len()).collect(),
            level: 0,
            node: 0,
        });
        while let Some(task) = queue.pop_front() {
            let aamm = self.nodes[task.node].aamm;
            let colliding: Vec<usize> = task
                .faces
                .iter()
                .copied()
                .filter(|&t| box_triangle(&triangles[t], &aamm))
                .collect();
            if colliding.is_empty() || task.level >= MAX_LEVEL {
                continue;
            }
            for node in self.subdivide(task.node, task.level) {
                queue.push_back(Task {
                    faces: colliding.clone(),
                    level: task.level + 1,
                    node,
                });
            }
        }
        self.balance();
    }

    /// `Subdivide`: eight children of `parent`, at `level + 1`, in slot
    /// order; their indices.
    #[allow(clippy::manual_midpoint)] // MuJoCo's `(a + b) / 2`, not `f64::midpoint`
    fn subdivide(&mut self, parent: usize, level: i32) -> [usize; 8] {
        std::array::from_fn(|slot| {
            let p = self.nodes[parent].aamm;
            let mid = |lo: usize| (p[lo + 3] + p[lo]) / 2.0;
            let mid_low = |lo: usize| (p[lo] + p[lo + 3]) / 2.0;
            let aamm = [
                if slot & 1 != 0 { mid(0) } else { p[0] },
                if slot & 2 != 0 { mid(1) } else { p[1] },
                if slot & 4 != 0 { mid(2) } else { p[2] },
                if slot & 1 != 0 { p[3] } else { mid_low(0) },
                if slot & 2 != 0 { p[4] } else { mid_low(1) },
                if slot & 4 != 0 { p[5] } else { mid_low(2) },
            ];
            let index = self.nodes.len();
            self.nodes.push(Node {
                level: level + 1,
                parent: Some(parent),
                slot,
                child: [-1; 8],
                vertid: [0; 8],
                aamm,
            });
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)] // MuJoCo's int
            {
                self.nodes[parent].child[slot] = index as i32;
            }
            self.finish_node(index);
            index
        })
    }

    /// `TaskToNode`'s corners: each of the node's eight, by `vert_map`, a
    /// new vertex where none is there yet.
    fn finish_node(&mut self, index: usize) {
        let a = self.nodes[index].aamm;
        for i in 0..8 {
            let v = [
                if i & 1 != 0 { a[3] } else { a[0] },
                if i & 2 != 0 { a[4] } else { a[1] },
                if i & 4 != 0 { a[5] } else { a[2] },
            ];
            let next = self.vertices.len();
            let id = *self.vertex_ids.entry(point_key(v)).or_insert(next);
            if id == next {
                self.vertices.push(v);
            }
            self.nodes[index].vertid[i] = id;
        }
    }

    fn is_leaf(&self, node: usize) -> bool {
        self.nodes[node].child[0] == -1
    }

    fn leaves(&self) -> Vec<usize> {
        (0..self.nodes.len()).filter(|&i| self.is_leaf(i)).collect()
    }

    /// `FindCoarseNeighbor`: the node beside `node` across its face `dir`
    /// (axis `dir / 2`, the high side where `dir % 2`) at its level or
    /// coarser, climbing to the ancestor whose sibling it is.
    fn find_coarse_neighbor(&self, mut node: usize, dir: usize) -> Option<usize> {
        loop {
            let parent = self.nodes[node].parent?;
            let slot = self.nodes[node].slot;
            let bit = 1 << (dir / 2);
            let side = dir % 2 == 1;
            if side != (slot & bit != 0) {
                let sibling = self.nodes[parent].child[slot ^ bit];
                return usize::try_from(sibling).ok();
            }
            node = parent;
        }
    }

    /// `FindNeighbor`: the coarse neighbour, then down to the leaf beside
    /// `node`'s centre.
    #[allow(clippy::manual_midpoint)] // MuJoCo's `(a + b) / 2`, not `f64::midpoint`
    fn find_neighbor(&self, node: usize, dir: usize) -> Option<usize> {
        let mut result = self.find_coarse_neighbor(node, dir)?;
        let center = |n: usize| {
            let a = self.nodes[n].aamm;
            [
                (a[0] + a[3]) / 2.0,
                (a[1] + a[4]) / 2.0,
                (a[2] + a[5]) / 2.0,
            ]
        };
        let node_center = center(node);
        while !self.is_leaf(result) {
            let result_center = center(result);
            let mut slot = 0;
            for axis in 0..3 {
                if node_center[axis] > result_center[axis] {
                    slot |= 1 << axis;
                }
            }
            let bit = 1 << (dir / 2);
            let op_side = usize::from(dir % 2 != 1);
            slot = (slot & !bit) | (op_side * bit);
            // A node that is split has all eight children (`Subdivide`).
            match usize::try_from(self.nodes[result].child[slot]) {
                Ok(child) => result = child,
                Err(_) => break,
            }
        }
        Some(result)
    }

    /// `BalanceOctree`: until none is found, each leaf more than one level
    /// coarser than a leaf beside it split (as `Subdivide` with no faces),
    /// the leaves taken in index order and split in the order found.
    fn balance(&mut self) {
        loop {
            let mut split = Vec::new();
            for leaf in self.leaves() {
                for dir in 0..6 {
                    let Some(neighbor) = self.find_neighbor(leaf, dir) else {
                        continue;
                    };
                    let (lv, nv) = (self.nodes[leaf].level, self.nodes[neighbor].level);
                    if nv > lv + 1 {
                        split.push(leaf);
                    }
                    if lv > nv + 1 {
                        split.push(neighbor);
                    }
                }
            }
            if split.is_empty() {
                return;
            }
            for node in split {
                if self.is_leaf(node) {
                    self.subdivide(node, self.nodes[node].level);
                }
            }
        }
    }

    /// `MarkHangingNodes`: for each leaf beside a coarser leaf across a
    /// face, each of the four corners of that face not yet marked: on two of
    /// the coarser leaf's box's planes (an edge) it hangs from that edge's
    /// two corners, on one (a face) from that face's four.
    fn mark_hanging(&mut self) {
        self.hang = vec![Vec::new(); self.vertices.len()];
        for leaf in self.leaves() {
            for dir in 0..6 {
                let Some(neighbor) = self.find_neighbor(leaf, dir) else {
                    continue;
                };
                if self.nodes[neighbor].level >= self.nodes[leaf].level {
                    continue;
                }
                let dim = dir / 2;
                let side = dir % 2;
                for i in 0..4 {
                    let d1 = (dim + 1) % 3;
                    let d2 = (dim + 2) % 3;
                    let v_idx = (side << dim) | ((i & 1) << d1) | (((i >> 1) & 1) << d2);
                    let hv = self.nodes[leaf].vertid[v_idx];
                    if !self.hang[hv].is_empty() {
                        continue;
                    }
                    let pos = self.vertices[hv];
                    let box_ = self.nodes[neighbor].aamm;
                    let is_min = [0, 1, 2].map(|d| (pos[d] - box_[d]).abs() < 1e-9);
                    let is_max = [0, 1, 2].map(|d| (pos[d] - box_[d + 3]).abs() < 1e-9);
                    let planes = (0..3).filter(|&d| is_min[d] || is_max[d]).count();
                    let corner = |bits: [usize; 3]| {
                        self.nodes[neighbor].vertid[(bits[2] << 2) | (bits[1] << 1) | bits[0]]
                    };
                    if planes == 2 {
                        let Some(d_mid) = (0..3).find(|&d| !is_min[d] && !is_max[d]) else {
                            continue;
                        };
                        let mut bits = [0; 3];
                        bits[(d_mid + 1) % 3] = usize::from(is_max[(d_mid + 1) % 3]);
                        bits[(d_mid + 2) % 3] = usize::from(is_max[(d_mid + 2) % 3]);
                        let first = corner(bits);
                        bits[d_mid] = 1;
                        let second = corner(bits);
                        self.hang[hv].extend([first, second]);
                    } else if planes == 1 {
                        let Some(d_face) = (0..3).find(|&d| is_min[d] || is_max[d]) else {
                            continue;
                        };
                        let mut bits = [0; 3];
                        bits[d_face] = usize::from(is_max[d_face]);
                        for j in 0..4 {
                            bits[(d_face + 1) % 3] = j & 1;
                            bits[(d_face + 2) % 3] = (j >> 1) & 1;
                            let c = corner(bits);
                            self.hang[hv].push(c);
                        }
                    }
                }
            }
        }
    }

    /// `mjCMesh::TryCompile`'s coefficients: the nodes visited breadth
    /// first from the root, each corner on its first visit given its signed
    /// distance to the mesh, or, hanging, the mean of those it hangs from.
    fn coefficients(
        &self,
        vertices: &[[f64; 3]],
        faces: &[[usize; 3]],
    ) -> Result<Vec<f64>, OctreeError> {
        let field = distance::MeshDistance::new(vertices, faces);
        let mut coeffs = vec![0.0; self.vertices.len()];
        let mut processed = vec![false; self.vertices.len()];
        let mut queue = VecDeque::from([0]);
        while let Some(node) = queue.pop_front() {
            for &v in &self.nodes[node].vertid {
                if processed[v] {
                    continue;
                }
                if self.hang[v].is_empty() {
                    coeffs[v] = field.signed_distance(self.vertices[v]);
                } else {
                    let mut sum = 0.0;
                    for &dep in &self.hang[v] {
                        sum += coeffs[dep];
                        if !processed[dep] {
                            return Err(OctreeError::Coefficients);
                        }
                    }
                    #[allow(clippy::cast_precision_loss)] // two or four
                    let count = self.hang[v].len() as f64;
                    coeffs[v] = sum / count;
                }
                processed[v] = true;
            }
            queue.extend(
                self.nodes[node]
                    .child
                    .iter()
                    .filter_map(|&c| usize::try_from(c).ok()),
            );
        }
        Ok(coeffs)
    }
}

/// `mju_max(a, b)`: `a` where `a >= b`.
fn mju_max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

/// `boxTriangle` (`user_objects.cc:629-678`, from Schwarz and Seidel, "Fast
/// Parallel Surface and Solid Voxelization on GPUs"): whether the triangle
/// `v` touches the box `aamm` (min, max).
#[allow(clippy::many_single_char_names)] // MuJoCo's names
fn box_triangle(v: &Triangle, aamm: &[f64; 6]) -> bool {
    for i in 0..3 {
        if v[0][i] < aamm[i] && v[1][i] < aamm[i] && v[2][i] < aamm[i] {
            return false;
        }
        let j = i + 3;
        if v[0][i] > aamm[j] && v[1][i] > aamm[j] && v[2][i] > aamm[j] {
            return false;
        }
    }
    let e: [[f64; 3]; 3] =
        std::array::from_fn(|i| std::array::from_fn(|j| v[(i + 1) % 3][j] - v[i][j]));
    // `mjuu_crossvec(n, e[0], e[1])`.
    let n = [
        e[0][1] * e[1][2] - e[0][2] * e[1][1],
        e[0][2] * e[1][0] - e[0][0] * e[1][2],
        e[0][0] * e[1][1] - e[0][1] * e[1][0],
    ];
    let dot3 = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let size = [aamm[3] - aamm[0], aamm[4] - aamm[1], aamm[5] - aamm[2]];
    let c = [0, 1, 2].map(|k| if n[k] > 0.0 { size[k] } else { 0.0 });
    let c1 = [0, 1, 2].map(|k| c[k] - v[0][k]);
    let c2 = [0, 1, 2].map(|k| size[k] - c[k] - v[0][k]);
    let lo = [aamm[0], aamm[1], aamm[2]];
    if (dot3(n, lo) + dot3(n, c1)) * (dot3(n, lo) + dot3(n, c2)) > 0.0 {
        return false;
    }
    let dot2 = |a: [f64; 2], b: [f64; 2]| a[0] * b[0] + a[1] * b[1];
    for (a, &na) in n.iter().enumerate() {
        let b = (a + 1) % 3;
        let cc = (a + 2) % 3;
        for (i, ei) in e.iter().enumerate() {
            let sign = if na >= 0.0 { 1.0 } else { -1.0 };
            let ne = [-ei[cc] * sign, ei[b] * sign];
            let vi = [v[i][b], v[i][cc]];
            let d = -dot2(ne, vi) + mju_max(0.0, size[b] * ne[0]) + mju_max(0.0, size[cc] * ne[1]);
            let p = [aamm[b], aamm[cc]];
            if dot2(ne, p) + d < 0.0 {
                return false;
            }
        }
    }
    true
}
