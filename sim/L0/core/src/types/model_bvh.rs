//! Each body's bounding-volume hierarchy over its colliding geoms, each
//! mesh's over its faces, and each flex's over its active elements, as
//! MuJoCo's compiler builds them (`mjCBody::ComputeBVH`,
//! `user_objects.cc:2323-2335`; `mjCMesh::SetBoundingVolume`,
//! `user_mesh.cc:847-870`; `mjCFlex::CreateBVH`, `user_mesh.cc:4423-4468`;
//! `mjCBoundingVolumeHierarchy`, `user_objects.cc:351-543`). The midphase
//! descends two bodies' hierarchies together (`collision::midphase`); a
//! mesh's ray descends the mesh's (`sensor::ray_mesh`).

use super::model::Model;
use super::model_derived::aabb_of;
use crate::forward::mj_fwd_position;
use crate::quat::rot_vec_quat;
use nalgebra::Vector3;

/// MuJoCo's compiler `mjEPS` (`user_util.h:29`): positions along the split
/// axis nearer than it tie, and a box flat along an axis is widened by it.
const EPS: f64 = 1e-14;

/// A leaf of a hierarchy, its id (a body's geom, a mesh's face, a flex's
/// element numbered within the flex), its position and the eight corners of
/// its box: a geom's in its body's inertial frame (`body_ipos`,
/// `body_iquat`), a face's in its mesh's frame, a flex element's in the
/// world's axes.
struct Element {
    id: usize,
    pos: [f64; 3],
    corners: [[f64; 3]; 8],
}

/// The nodes of the hierarchies built, a body's, a mesh's or each flex's in
/// turn, in the order made: a node, its first child's subtree, then its
/// second's.
#[derive(Default)]
struct Tree {
    aabb: Vec<[f64; 6]>,
    child: Vec<[Option<usize>; 2]>,
    nodeid: Vec<Option<usize>>,
    depth: Vec<usize>,
}

impl Model {
    /// Build each body's bounding-volume hierarchy over its geoms that
    /// collide (a nonzero `geom_contype` or `geom_conaffinity`), and each
    /// body's margin, the largest of its geoms' (MuJoCo's compiler,
    /// `user_objects.cc:2479-2485`, `:2323-2335`; `user_model.cc:2700-2713`).
    ///
    /// A hierarchy's boxes are in its body's inertial frame. A node bounds
    /// its geoms' boxes; with more than one geom it splits them in half by
    /// their positions along its box's longest axis (an axis must be longer
    /// by 1e-14, MuJoCo's `mjEPS`, to be taken), the lower half its first child. A body none
    /// of whose geoms collides has no hierarchy (`body_bvhadr` `None`). Each
    /// mesh's hierarchy over its faces follows the bodies'
    /// (`compute_mesh_bvhs`).
    ///
    /// Writes `body_margin`, `body_bvhadr`, `body_bvhnum`, `mesh_bvhadr`,
    /// `mesh_bvhnum`, `bvh_aabb`, `bvh_child`, `bvh_nodeid` and
    /// `bvh_depth`. Reads `geom_body`, `geom_pos`, `geom_quat`, `geom_aabb`
    /// (so it runs after [`Self::compute_geom_bounding_radii`]),
    /// `geom_contype`, `geom_conaffinity`, `geom_margin`, `body_ipos`,
    /// `body_iquat` and `mesh_data`: a model edited in any of them afterwards
    /// keeps the hierarchies made before (as MuJoCo's compiled ones), and the
    /// midphase finds the pairs those hold, and a mesh's ray the faces, until
    /// [`Self::recompute_derived`] runs again. The flexes' hierarchies, which
    /// follow the meshes' nodes, are dropped: [`Self::compute_flex_bvhs`]
    /// appends them again.
    pub fn compute_body_bvhs(&mut self) {
        let mut geoms: Vec<Vec<usize>> = vec![Vec::new(); self.nbody];
        for (geom, &body) in self.geom_body.iter().enumerate() {
            geoms[body].push(geom);
        }
        self.body_margin = geoms
            .iter()
            .map(|g| g.iter().map(|&g| self.geom_margin[g]).fold(0.0, f64::max))
            .collect();
        self.body_bvhadr = vec![None; self.nbody];
        self.body_bvhnum = vec![0; self.nbody];
        self.bvh_aabb.clear();
        self.bvh_child.clear();
        self.bvh_nodeid.clear();
        self.bvh_depth.clear();
        for (body, geoms) in geoms.iter().enumerate() {
            let elements: Vec<Element> = geoms
                .iter()
                .filter(|&&g| self.geom_contype[g] != 0 || self.geom_conaffinity[g] != 0)
                .map(|&g| self.element(body, g))
                .collect();
            let mut order: Vec<usize> = (0..elements.len()).collect();
            let mut tree = Tree::default();
            make_bvh(&elements, &mut order, 0, &mut tree);
            if tree.aabb.is_empty() {
                continue;
            }
            self.body_bvhadr[body] = Some(self.bvh_aabb.len());
            self.body_bvhnum[body] = tree.aabb.len();
            self.bvh_aabb.extend(tree.aabb);
            self.bvh_child.extend(tree.child);
            self.bvh_nodeid.extend(tree.nodeid);
            self.bvh_depth.extend(tree.depth);
        }
        self.compute_mesh_bvhs();
    }

    /// Build each mesh's bounding-volume hierarchy over its faces, after
    /// every body's, as MuJoCo's compiler (`mjCMesh::SetBoundingVolume` and
    /// its caller, `user_mesh.cc:847-870`, `:1699-1707`; laid out after the
    /// bodies', `user_model.cc:2134-2145`, `:3280`): a leaf is a face, its
    /// box over its three vertices, placed at its circumcentre, unrotated;
    /// split as a body's. The boxes are in the mesh's frame. A mesh with no
    /// face has no hierarchy (`mesh_bvhadr` `None`). A mesh's ray descends it
    /// (`sensor::ray_mesh`).
    ///
    /// A face of no area (its edges' cross product exactly 0, as where a
    /// vertex is repeated) has no circumcentre (it is NaN, in MuJoCo's too), and MuJoCo's
    /// split, comparing such a face's place with another's, is no longer an
    /// order: its tree is then its C++ library's `nth_element`'s, which
    /// [`merge_sort`] is not (registry `D-MESH-SPLIT`). Its boxes are over
    /// the vertices sim-core holds, MuJoCo's over its double-precision ones,
    /// before it rounds them to the single precision it keeps (ledger L281).
    fn compute_mesh_bvhs(&mut self) {
        let trees: Vec<Tree> = self
            .mesh_data
            .iter()
            .map(|mesh| {
                let vertices = mesh.vertices();
                let elements: Vec<Element> = mesh
                    .triangles()
                    .iter()
                    .enumerate()
                    .map(|(f, face)| {
                        face_element(f, face.map(|v| vertices[v as usize].coords.into()))
                    })
                    .collect();
                let mut order: Vec<usize> = (0..elements.len()).collect();
                let mut tree = Tree::default();
                make_bvh(&elements, &mut order, 0, &mut tree);
                tree
            })
            .collect();
        self.mesh_bvhadr = vec![None; trees.len()];
        self.mesh_bvhnum = vec![0; trees.len()];
        for (mesh, tree) in trees.into_iter().enumerate() {
            if tree.aabb.is_empty() {
                continue;
            }
            self.mesh_bvhadr[mesh] = Some(self.bvh_aabb.len());
            self.mesh_bvhnum[mesh] = tree.aabb.len();
            self.bvh_aabb.extend(tree.aabb);
            self.bvh_child.extend(tree.child);
            self.bvh_nodeid.extend(tree.nodeid);
            self.bvh_depth.extend(tree.depth);
        }
    }

    /// Build each flex's bounding-volume hierarchy over its active elements,
    /// after every body's and mesh's, as MuJoCo's compiler
    /// (`mjCFlex::CreateBVH`, `user_mesh.cc:4423-4468`): a leaf is an element,
    /// numbered within its flex, its box over its vertices at `qpos0` grown by
    /// the flex's radius, unrotated; a dim-3 element is active while its layer
    /// is below the flex's `flex_activelayers`. The hierarchy is split as a
    /// body's. Only its shape is kept (`bvh_child`, `bvh_nodeid`,
    /// `bvh_depth`): its boxes move with the vertices, `Data::bvh_aabb_dyn`
    /// (MuJoCo copies no `bvh_aabb` for a flex, `user_model.cc:3447-3454`). A
    /// flex with no active element has no hierarchy (`flex_bvhadr` `None`;
    /// MuJoCo's compiler does not take such a flex, registry
    /// `D-FLEX-NO-ACTIVE`).
    ///
    /// Writes `flex_bvhadr`, `flex_bvhnum`, `nbvhstatic` and `nbvhdynamic`
    /// and appends to `bvh_child`, `bvh_nodeid` and `bvh_depth`, after the
    /// bodies' and meshes' nodes (so it runs after [`Self::compute_body_bvhs`]);
    /// reads `flex_elemlayer` ([`Self::compute_flex_topology`]), `flex_radius`
    /// and the vertices' bodies at `qpos0`, which forward kinematics on a
    /// derivation `Data` places: the model's joint layout and ranges must
    /// pass, as for the derivations that step it.
    pub fn compute_flex_bvhs(&mut self) {
        self.nbvhstatic = self.bvh_aabb.len();
        self.bvh_child.truncate(self.nbvhstatic);
        self.bvh_nodeid.truncate(self.nbvhstatic);
        self.bvh_depth.truncate(self.nbvhstatic);
        self.flex_bvhadr = vec![None; self.nflex];
        self.flex_bvhnum = vec![0; self.nflex];
        self.nbvhdynamic = 0;
        if self.nflexelem == 0 {
            return;
        }
        let mut data = self.make_data_for_derivation();
        mj_fwd_position(self, &mut data);
        let position = |v: usize| data.xpos[self.flexvert_bodyid[v]];
        let mut nodes = Tree::default();
        for f in 0..self.nflex {
            let elements: Vec<Element> = (0..self.flex_elemnum[f])
                .filter(|&e| self.flex_elem_active(f, e))
                .map(|e| {
                    let global = self.flex_elemadr[f] + e;
                    let adr = self.flexelem_dataadr[global];
                    let vertices = &self.flexelem_data[adr..adr + self.flexelem_datanum[global]];
                    flex_element(
                        e,
                        vertices.iter().map(|&v| position(v)),
                        self.flex_radius[f],
                    )
                })
                .collect();
            let mut order: Vec<usize> = (0..elements.len()).collect();
            let mut tree = Tree::default();
            make_bvh(&elements, &mut order, 0, &mut tree);
            if tree.child.is_empty() {
                continue;
            }
            self.flex_bvhadr[f] = Some(self.nbvhstatic + nodes.child.len());
            self.flex_bvhnum[f] = tree.child.len();
            nodes.child.extend(tree.child);
            nodes.nodeid.extend(tree.nodeid);
            nodes.depth.extend(tree.depth);
        }
        self.nbvhdynamic = nodes.child.len();
        self.bvh_child.extend(nodes.child);
        self.bvh_nodeid.extend(nodes.nodeid);
        self.bvh_depth.extend(nodes.depth);
    }

    /// Geom `geom` of body `body`, placed in the body's inertial frame
    /// (`mjCBoundingVolumeHierarchy::Make` and `MakeBVH`,
    /// `user_objects.cc:403-419`, `:436-468`).
    fn element(&self, body: usize, geom: usize) -> Element {
        let (ipos, iq) = (self.body_ipos[body], self.body_iquat[body]);
        let qinv = [iq.w, -iq.i, -iq.j, -iq.k];
        let gq = self.geom_quat[geom];
        let quat = [gq.w, gq.i, gq.j, gq.k];
        let p = self.geom_pos[geom];
        let offset = [p.x - ipos.x, p.y - ipos.y, p.z - ipos.z];
        let a = self.geom_aabb[geom];
        let (lo, hi) = (
            [a[0] - a[3], a[1] - a[4], a[2] - a[5]],
            [a[0] + a[3], a[1] + a[4], a[2] + a[5]],
        );
        let corners = std::array::from_fn(|v| {
            let vert = std::array::from_fn(|i| if v >> i & 1 == 1 { hi[i] } else { lo[i] });
            let mut placed = rot_vec_quat(&vert, &quat);
            for i in 0..3 {
                placed[i] += offset[i];
            }
            rot_vec_quat(&placed, &qinv)
        });
        Element {
            id: geom,
            pos: rot_vec_quat(&offset, &qinv),
            corners,
        }
    }
}

/// Flex element `e` (numbered within its flex) as a leaf: its box over its
/// vertices, grown by the flex's `radius`, centre `0.5 (max + min)` and
/// half-size `0.5 (max - min) + radius` (`user_mesh.cc:4438-4456`); placed
/// at the box's centre, with no rotation (`user_objects.cc:409-416`,
/// `:447-468`).
fn flex_element(
    e: usize,
    mut vertices: impl Iterator<Item = Vector3<f64>>,
    radius: f64,
) -> Element {
    let first = vertices.next().unwrap_or_else(Vector3::zeros);
    let (mut min, mut max) = (first, first);
    for v in vertices {
        for j in 0..3 {
            // `std::min` and `std::max`.
            min[j] = if v[j] < min[j] { v[j] } else { min[j] };
            max[j] = if max[j] < v[j] { v[j] } else { max[j] };
        }
    }
    let centre = [
        0.5 * (max[0] + min[0]),
        0.5 * (max[1] + min[1]),
        0.5 * (max[2] + min[2]),
    ];
    let half = [
        0.5 * (max[0] - min[0]) + radius,
        0.5 * (max[1] - min[1]) + radius,
        0.5 * (max[2] - min[2]) + radius,
    ];
    let corners = std::array::from_fn(|v| {
        std::array::from_fn(|i| {
            if v >> i & 1 == 1 {
                centre[i] + half[i]
            } else {
                centre[i] - half[i]
            }
        })
    });
    Element {
        id: e,
        pos: centre,
        corners,
    }
}

/// Face `f` of a mesh, its corners `v`, as a leaf: its box over them,
/// centre `0.5 (min + max)` and half-size `0.5 (max - min)`
/// (`mjCMesh::SetBoundingVolume`, `user_mesh.cc:847-870`), placed at its
/// circumcentre, with no rotation (`user_objects.cc:409-416`, `:447-468`).
fn face_element(f: usize, v: [[f64; 3]; 3]) -> Element {
    let mut min = [f64::MAX; 3];
    let mut max = [-f64::MAX; 3];
    for p in &v {
        for j in 0..3 {
            // `std::min` and `std::max`.
            min[j] = if p[j] < min[j] { p[j] } else { min[j] };
            max[j] = if max[j] < p[j] { p[j] } else { max[j] };
        }
    }
    let centre: [f64; 3] = std::array::from_fn(|j| 0.5 * (min[j] + max[j]));
    let half: [f64; 3] = std::array::from_fn(|j| 0.5 * (max[j] - min[j]));
    let corners = std::array::from_fn(|c| {
        std::array::from_fn(|i| {
            if c >> i & 1 == 1 {
                centre[i] + half[i]
            } else {
                centre[i] - half[i]
            }
        })
    });
    Element {
        id: f,
        pos: circumcentre(&v),
        corners,
    }
}

/// The circumcentre of the triangle `v` (`mjCMesh::MakeCenter`,
/// `user_mesh.cc:2677-2715`): with `a` and `b` its first two corners less
/// its third and `n = a × b`, the third plus `(|a|² b − |b|² a) × n / (2
/// |n|²)`.
fn circumcentre(v: &[[f64; 3]; 3]) -> [f64; 3] {
    // `mjuu_crossvec` and `mjuu_dot3`.
    let cross = |p: &[f64; 3], q: &[f64; 3]| {
        [
            p[1] * q[2] - p[2] * q[1],
            p[2] * q[0] - p[0] * q[2],
            p[0] * q[1] - p[1] * q[0],
        ]
    };
    let dot = |p: &[f64; 3], q: &[f64; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    let a: [f64; 3] = std::array::from_fn(|j| v[0][j] - v[2][j]);
    let b: [f64; 3] = std::array::from_fn(|j| v[1][j] - v[2][j]);
    let nrm = cross(&a, &b);
    let (a_sqr, b_sqr) = (dot(&a, &a), dot(&b, &b));
    let area = dot(&nrm, &nrm).sqrt();
    let vec: [f64; 3] = std::array::from_fn(|j| a_sqr * b[j] - b_sqr * a[j]);
    let res = cross(&vec, &nrm);
    std::array::from_fn(|j| res[j] / (2.0 * area * area) + v[2][j])
}

/// Make the node bounding the elements `order` names, then its subtrees;
/// its index, `None` for no elements (`MakeBVH`, `user_objects.cc:423-543`).
fn make_bvh(
    elements: &[Element],
    order: &mut [usize],
    depth: usize,
    tree: &mut Tree,
) -> Option<usize> {
    if order.is_empty() {
        return None;
    }
    let mut min = [f64::MAX; 3];
    let mut max = [-f64::MAX; 3];
    for corner in order.iter().flat_map(|&e| &elements[e].corners) {
        for i in 0..3 {
            min[i] = min[i].min(corner[i]);
            max[i] = max[i].max(corner[i]);
        }
    }
    for i in 0..3 {
        if (min[i] - max[i]).abs() < EPS {
            min[i] -= EPS;
            max[i] += EPS;
        }
    }
    let index = tree.aabb.len();
    tree.aabb.push(aabb_of(min, max));
    tree.child.push([None, None]);
    tree.nodeid.push(None);
    tree.depth.push(depth);
    if let [element] = order {
        tree.nodeid[index] = Some(elements[*element].id);
        return Some(index);
    }

    let edges = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    let first = usize::from(edges[1] >= edges[0] + EPS);
    let axis = if edges[2] >= edges[first] + EPS {
        2
    } else {
        first
    };
    let (a, b) = (axis, &elements);
    merge_sort(order, &|i, j| {
        let (pi, pj) = (b[i].pos[a], b[j].pos[a]);
        if (pi - pj).abs() > EPS {
            pi < pj
        } else {
            b[i].id < b[j].id
        }
    });
    let (lower, upper) = order.split_at_mut(order.len() / 2);
    tree.child[index] = [
        make_bvh(elements, lower, depth + 1, tree),
        make_bvh(elements, upper, depth + 1, tree),
    ];
    Some(index)
}

/// Sort `order` by `before`, keeping equal elements in place: MuJoCo's split
/// comparison, positions farther apart than [`EPS`] by position and nearer
/// by the leaves' order, their ids' (`user_objects.cc:511-517`), need not be transitive, which
/// `slice::sort_by` may panic on; this never does. Where it is transitive,
/// the lower half is the half MuJoCo's `nth_element` takes; where it is not
/// (a mesh's face with no circumcentre), it need not be (registry
/// `D-MESH-SPLIT`).
fn merge_sort(order: &mut [usize], before: &impl Fn(usize, usize) -> bool) {
    if order.len() < 2 {
        return;
    }
    let mid = order.len() / 2;
    merge_sort(&mut order[..mid], before);
    merge_sort(&mut order[mid..], before);
    let mut merged = Vec::with_capacity(order.len());
    let (mut i, mut j) = (0, mid);
    while i < mid && j < order.len() {
        if before(order[j], order[i]) {
            merged.push(order[j]);
            j += 1;
        } else {
            merged.push(order[i]);
            i += 1;
        }
    }
    merged.extend_from_slice(&order[i..mid]);
    merged.extend_from_slice(&order[j..]);
    order.copy_from_slice(&merged);
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use nalgebra::Vector3;
    use serde_json::Value;

    use crate::types::Model;

    /// Three spheres along x at 0, 0.8e-14 and 1.6e-14, numbered in reverse:
    /// the first two tie, and the last two, but the first and last do not,
    /// so MuJoCo's comparison is not transitive here. The hierarchy is built
    /// all the same, its three leaves the three geoms.
    #[test]
    fn a_comparison_that_is_not_transitive_still_builds() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        for x in [1.6e-14, 0.8e-14, 0.0] {
            model.add_ground_plane();
            let g = model.ngeom - 1;
            model.geom_type[g] = crate::types::GeomType::Sphere;
            model.geom_size[g] = Vector3::new(0.1, 0.0, 0.0);
            model.geom_pos[g] = Vector3::new(x, 0.0, 0.0);
        }
        model.compute_geom_bounding_radii();
        model.compute_body_bvhs();
        let mut leaves: Vec<usize> = model.bvh_nodeid.iter().flatten().copied().collect();
        leaves.sort_unstable();
        assert_eq!(leaves, vec![0, 1, 2]);
        assert_eq!(model.body_bvhnum[0], 5);
    }

    /// A box flat along z: its node is widened by `EPS` each way along it,
    /// as MuJoCo's (`user_objects.cc:471-477`).
    #[test]
    fn a_flat_box_is_widened_by_eps() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.geom_type[0] = crate::types::GeomType::Box;
        model.geom_size[0] = Vector3::new(0.1, 0.2, 0.0);
        model.compute_geom_bounding_radii();
        model.compute_body_bvhs();
        let node = model.bvh_aabb[0];
        assert_eq!(node[2].to_bits(), 0.0_f64.to_bits(), "centre z {}", node[2]);
        assert_eq!(
            node[5].to_bits(),
            super::EPS.to_bits(),
            "half-height {}",
            node[5]
        );
        assert_eq!(
            node[3].to_bits(),
            0.1_f64.to_bits(),
            "half-width {}",
            node[3]
        );
    }

    /// A body none of whose geoms collides has no hierarchy, and its
    /// margin is still its geoms' largest.
    #[test]
    fn a_body_of_visual_geoms_has_no_hierarchy() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.geom_contype[0] = 0;
        model.geom_conaffinity[0] = 0;
        model.geom_margin[0] = 0.25;
        model.compute_body_bvhs();
        assert_eq!(model.body_bvhadr, vec![None, None]);
        assert_eq!(model.body_bvhnum, vec![0, 0]);
        assert!(model.bvh_aabb.is_empty());
        assert_eq!(model.body_margin[0].to_bits(), 0.25_f64.to_bits());
    }

    /// Deriving the model again rebuilds the hierarchies from its geoms: a
    /// plane made visual leaves the world without one.
    #[test]
    fn recompute_derived_rebuilds_the_hierarchies() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        assert_eq!(model.body_bvhadr[0], Some(0));
        model.geom_contype[0] = 0;
        model.geom_conaffinity[0] = 0;
        model.recompute_derived().expect("derive");
        assert_eq!(model.body_bvhadr[0], None);
        assert!(model.bvh_aabb.is_empty());
    }

    /// `add_ground_plane` appends to a derived model; the world's hierarchy
    /// then holds every plane it appended.
    #[test]
    fn a_ground_plane_joins_the_worlds_hierarchy() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.add_ground_plane();
        assert_eq!(model.body_bvhadr[0], Some(0));
        let mut leaves: Vec<usize> = model.bvh_nodeid.iter().flatten().copied().collect();
        leaves.sort_unstable();
        assert_eq!(leaves, vec![0, 1]);
    }

    fn numbers(v: &Value) -> Vec<f64> {
        v.as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect()
    }

    /// MuJoCo's index, -1 for none, as an `Option`.
    fn index(v: &Value) -> Option<usize> {
        usize::try_from(v.as_i64().expect("an index")).ok()
    }

    /// The ray golden's meshes (`gen_ray_reference.py` `mesh_models`: a box,
    /// a box whose faces are 4 by 4 squares and an octahedron, which MuJoCo
    /// keeps in their written frame; a slab and a convex hull, which it
    /// turns; a star that is not convex; and the 4 by 4 box with 27 faces of
    /// no area added), each built from MuJoCo's vertices and faces as it
    /// compiles them: the mesh's hierarchy follows the world's node, and its
    /// children and depths are MuJoCo's. Its leaves' faces are MuJoCo's, and,
    /// for the meshes in their written frame, its boxes are MuJoCo's bit for
    /// bit, but for the mesh with faces of no area: 202 of its 219 leaves name
    /// another face, and 396 of its 437 boxes differ (registry D-MESH-SPLIT:
    /// MuJoCo's split compares their NaN circumcentres). (MuJoCo builds a mesh's boxes over its vertices in
    /// double precision, before it rounds them to the single precision it
    /// keeps, which is what this model is given: ledger L281.)
    #[test]
    fn each_mesh_hierarchy_is_mujocos() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/ray/ray.json"
        );
        let text = std::fs::read_to_string(path).expect("the ray golden reads");
        let golden: Value = serde_json::from_str(&text).expect("the ray golden parses");
        let meshes = golden["mesh_models"].as_object().expect("mesh_models");
        assert_eq!(meshes.len(), 8, "every mesh");
        let (mut leaves, mut leaves_differ, mut boxes_differ) = (0, 0, 0);
        for (name, mesh) in meshes {
            let face: Vec<usize> = mesh["face"]
                .as_array()
                .expect("faces")
                .iter()
                .map(|i| index(i).expect("a vertex"))
                .collect();
            let model = Model::one_mesh(&numbers(&mesh["vert"]), &face);
            let adr = index(&mesh["mesh_bvhadr"][0]).expect("a hierarchy");
            let num = index(&mesh["mesh_bvhnum"][0]).expect("its nodes");
            assert_eq!(model.mesh_bvhadr, vec![Some(adr)], "{name}: mesh_bvhadr");
            assert_eq!(model.mesh_bvhnum, vec![num], "{name}: mesh_bvhnum");
            assert_eq!(
                model.nbvhstatic,
                index(&mesh["nbvhstatic"]).expect("a count"),
                "{name}: nbvhstatic"
            );
            let zero_area = mesh["zero_area_faces"].as_u64().expect("a count") > 0;
            for k in adr..adr + num {
                let child = &mesh["bvh_child"][k];
                assert_eq!(
                    model.bvh_child[k],
                    [index(&child[0]), index(&child[1])],
                    "{name} node {k}: children"
                );
                assert_eq!(
                    Some(model.bvh_depth[k]),
                    index(&mesh["bvh_depth"][k]),
                    "{name} node {k}: depth"
                );
                if zero_area {
                    let theirs = numbers(&mesh["bvh_aabb"][k]);
                    leaves += usize::from(model.bvh_nodeid[k].is_some());
                    leaves_differ +=
                        usize::from(model.bvh_nodeid[k] != index(&mesh["bvh_nodeid"][k]));
                    boxes_differ += usize::from(
                        (0..6).any(|i| model.bvh_aabb[k][i].to_bits() != theirs[i].to_bits()),
                    );
                    continue;
                }
                assert_eq!(
                    model.bvh_nodeid[k],
                    index(&mesh["bvh_nodeid"][k]),
                    "{name} node {k}: face"
                );
                if mesh["authored_frame"].as_bool().expect("a flag") {
                    let theirs = numbers(&mesh["bvh_aabb"][k]);
                    let ours = model.bvh_aabb[k];
                    assert!(
                        ours.iter()
                            .zip(&theirs)
                            .all(|(a, b)| a.to_bits() == b.to_bits()),
                        "{name} node {k}: box {ours:?}, MuJoCo {theirs:?}"
                    );
                }
            }
        }
        assert_eq!(
            (leaves, leaves_differ, boxes_differ),
            (219, 202, 396),
            "the mesh with faces of no area: its leaves, those naming another face than MuJoCo's, \
             its boxes not MuJoCo's (registry D-MESH-SPLIT states these)"
        );
    }
}
