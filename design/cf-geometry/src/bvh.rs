//! Bounding Volume Hierarchy for efficient spatial queries.
//!
//! The BVH organizes primitives in a binary AABB tree. Construction uses a
//! top-down median-split strategy: find the longest axis, sort primitives by
//! centroid, split at the median, recurse.
//!
//! # Query types
//!
//! - [`Bvh::query`] — find all primitives overlapping an AABB
//! - [`Bvh::query_callback`] — callback variant (no allocation)
//! - [`Bvh::query_pairs`] — dual-BVH traversal for mesh-mesh overlap
//! - [`Bvh::query_ray`] — find primitives along a ray path
//! - [`Bvh::query_ray_closest`] — closest ray hit with progressive pruning
//!
//! # Free functions
//!
//! - [`bvh_from_mesh`] — build BVH from an [`IndexedMesh`]
//! - [`bvh_from_triangle_mesh`] — build BVH from raw vertex/index arrays
//! - [`query_bvh_pair`] — dual-BVH query with isometry transforms

use nalgebra::{Isometry3, Point3, Vector3};

use crate::IndexedMesh;
use crate::aabb::Aabb;

// ---------------------------------------------------------------------------
// BvhPrimitive
// ---------------------------------------------------------------------------

/// A primitive stored in the BVH.
///
/// Wraps an AABB with an index back into the original data and optional
/// metadata. Primitives can represent triangles, convex parts, or any
/// geometry that has an associated AABB.
#[derive(Debug, Clone)]
pub struct BvhPrimitive {
    /// Axis-aligned bounding box of this primitive.
    pub aabb: Aabb,
    /// Index into the original primitive array (e.g. triangle index).
    pub index: usize,
    /// Optional additional data (e.g. material ID).
    pub data: u32,
}

impl BvhPrimitive {
    /// Creates a new primitive with `data = 0`.
    #[must_use]
    pub const fn new(aabb: Aabb, index: usize) -> Self {
        Self {
            aabb,
            index,
            data: 0,
        }
    }

    /// Creates a new primitive with additional data.
    #[must_use]
    pub const fn with_data(aabb: Aabb, index: usize, data: u32) -> Self {
        Self { aabb, index, data }
    }

    /// Creates a primitive from three triangle vertices.
    #[must_use]
    pub fn from_triangle(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>, index: usize) -> Self {
        Self::new(Aabb::from_triangle(&v0, &v1, &v2), index)
    }

    /// Returns the centroid of this primitive's AABB.
    #[must_use]
    pub fn centroid(&self) -> Point3<f64> {
        self.aabb.center()
    }
}

// ---------------------------------------------------------------------------
// BvhNode (internal)
// ---------------------------------------------------------------------------

/// A node in the BVH tree.
#[derive(Debug, Clone)]
enum BvhNode {
    /// Internal node with two children.
    Internal {
        aabb: Aabb,
        left: usize,
        right: usize,
    },
    /// Leaf node containing a contiguous range of primitives.
    Leaf {
        aabb: Aabb,
        first_primitive: usize,
        primitive_count: usize,
    },
}

impl BvhNode {
    const fn aabb(&self) -> &Aabb {
        match self {
            Self::Internal { aabb, .. } | Self::Leaf { aabb, .. } => aabb,
        }
    }
}

// ---------------------------------------------------------------------------
// Bvh
// ---------------------------------------------------------------------------

/// Bounding Volume Hierarchy for efficient spatial queries.
///
/// The BVH organizes primitives in a binary tree where each node contains an
/// AABB bounding all its descendants. Queries traverse the tree, descending
/// only when AABBs overlap.
///
/// # Construction
///
/// ```ignore
/// use cf_geometry::{Bvh, BvhPrimitive, Aabb};
/// use nalgebra::Point3;
///
/// let primitives: Vec<BvhPrimitive> = /* ... */;
/// let bvh = Bvh::build(primitives);
/// ```
#[derive(Debug, Clone)]
pub struct Bvh {
    nodes: Vec<BvhNode>,
    primitives: Vec<BvhPrimitive>,
    max_primitives_per_leaf: usize,
}

impl Default for Bvh {
    fn default() -> Self {
        Self::new()
    }
}

impl Bvh {
    /// Creates an empty BVH.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            nodes: Vec::new(),
            primitives: Vec::new(),
            max_primitives_per_leaf: 4,
        }
    }

    /// Sets the maximum number of primitives per leaf node.
    ///
    /// Lower values → deeper trees, faster narrow-phase. Higher values →
    /// shallower trees, more tests per leaf. Default is 4. Minimum is 1.
    #[must_use]
    pub fn with_max_primitives_per_leaf(mut self, max: usize) -> Self {
        self.max_primitives_per_leaf = max.max(1);
        self
    }

    /// Builds a BVH from primitives (convenience for `Bvh::new().build_from()`).
    #[must_use]
    pub fn build(primitives: Vec<BvhPrimitive>) -> Self {
        Self::new().build_from(primitives)
    }

    /// Builds the BVH from primitives, consuming the builder configuration.
    #[must_use]
    pub fn build_from(mut self, mut primitives: Vec<BvhPrimitive>) -> Self {
        if primitives.is_empty() {
            return self;
        }

        self.nodes.clear();
        self.nodes.reserve(primitives.len() * 2);

        let len = primitives.len();
        self.build_recursive(&mut primitives, 0, len);

        self.primitives = primitives;
        self
    }

    // -- Construction internals ---------------------------------------------

    /// Recursively builds the tree. Returns the index of the created node.
    fn build_recursive(
        &mut self,
        primitives: &mut [BvhPrimitive],
        start: usize,
        end: usize,
    ) -> usize {
        let count = end - start;
        let slice = &primitives[start..end];

        let aabb = compute_bounds(slice);

        // Leaf
        if count <= self.max_primitives_per_leaf {
            let node_idx = self.nodes.len();
            self.nodes.push(BvhNode::Leaf {
                aabb,
                first_primitive: start,
                primitive_count: count,
            });
            return node_idx;
        }

        // Find longest axis
        let axis = aabb.longest_axis();

        // Sort by centroid along that axis
        let primitives_slice = &mut primitives[start..end];
        primitives_slice.sort_by(|a, b| {
            let va = a.centroid()[axis.index()];
            let vb = b.centroid()[axis.index()];
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Median split
        let mid = start + count / 2;

        // Reserve a slot for this internal node (children filled after recursion)
        let node_idx = self.nodes.len();
        self.nodes.push(BvhNode::Internal {
            aabb,
            left: 0,
            right: 0,
        });

        let left_idx = self.build_recursive(primitives, start, mid);
        let right_idx = self.build_recursive(primitives, mid, end);

        if let BvhNode::Internal { left, right, .. } = &mut self.nodes[node_idx] {
            *left = left_idx;
            *right = right_idx;
        }

        node_idx
    }

    // -- AABB queries -------------------------------------------------------

    /// Finds all primitives whose AABB overlaps `query_aabb`.
    ///
    /// Returns the original primitive indices (not internal array positions).
    /// The caller should perform precise intersection tests on these.
    #[must_use]
    pub fn query(&self, query_aabb: &Aabb) -> Vec<usize> {
        let mut results = Vec::new();
        if !self.nodes.is_empty() {
            self.query_recursive(0, query_aabb, &mut results);
        }
        results
    }

    fn query_recursive(&self, node_idx: usize, query_aabb: &Aabb, results: &mut Vec<usize>) {
        let node = &self.nodes[node_idx];

        if !node.aabb().overlaps(query_aabb) {
            return;
        }

        match node {
            BvhNode::Internal { left, right, .. } => {
                self.query_recursive(*left, query_aabb, results);
                self.query_recursive(*right, query_aabb, results);
            }
            BvhNode::Leaf {
                first_primitive,
                primitive_count,
                ..
            } => {
                for i in *first_primitive..(*first_primitive + *primitive_count) {
                    if self.primitives[i].aabb.overlaps(query_aabb) {
                        results.push(self.primitives[i].index);
                    }
                }
            }
        }
    }

    /// Queries with a callback for each overlapping primitive (no allocation).
    pub fn query_callback<F>(&self, query_aabb: &Aabb, mut callback: F)
    where
        F: FnMut(&BvhPrimitive),
    {
        if !self.nodes.is_empty() {
            self.query_callback_recursive(0, query_aabb, &mut callback);
        }
    }

    fn query_callback_recursive<F>(&self, node_idx: usize, query_aabb: &Aabb, callback: &mut F)
    where
        F: FnMut(&BvhPrimitive),
    {
        let node = &self.nodes[node_idx];

        if !node.aabb().overlaps(query_aabb) {
            return;
        }

        match node {
            BvhNode::Internal { left, right, .. } => {
                self.query_callback_recursive(*left, query_aabb, callback);
                self.query_callback_recursive(*right, query_aabb, callback);
            }
            BvhNode::Leaf {
                first_primitive,
                primitive_count,
                ..
            } => {
                for i in *first_primitive..(*first_primitive + *primitive_count) {
                    if self.primitives[i].aabb.overlaps(query_aabb) {
                        callback(&self.primitives[i]);
                    }
                }
            }
        }
    }

    // -- Dual-BVH queries ---------------------------------------------------

    /// Finds overlapping primitive pairs between two BVHs.
    ///
    /// `transform_b_aabb` maps AABBs from B's space into A's space.
    /// Returns `(index_a, index_b)` pairs that may intersect.
    #[must_use]
    pub fn query_pairs<F>(&self, other: &Self, transform_b_aabb: F) -> Vec<(usize, usize)>
    where
        F: Fn(&Aabb) -> Aabb,
    {
        let mut results = Vec::new();
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return results;
        }
        self.query_pairs_recursive(0, other, 0, &transform_b_aabb, &mut results);
        results
    }

    #[allow(clippy::similar_names)]
    fn query_pairs_recursive<F>(
        &self,
        idx_a: usize,
        other: &Self,
        idx_b: usize,
        transform_b_aabb: &F,
        results: &mut Vec<(usize, usize)>,
    ) where
        F: Fn(&Aabb) -> Aabb,
    {
        let node_a = &self.nodes[idx_a];
        let node_b = &other.nodes[idx_b];

        let aabb_b_in_a = transform_b_aabb(node_b.aabb());

        if !node_a.aabb().overlaps(&aabb_b_in_a) {
            return;
        }

        match (node_a, node_b) {
            // Both leaves — check primitive pairs
            (
                BvhNode::Leaf {
                    first_primitive: first_a,
                    primitive_count: count_a,
                    ..
                },
                BvhNode::Leaf {
                    first_primitive: first_b,
                    primitive_count: count_b,
                    ..
                },
            ) => {
                for i in *first_a..(*first_a + *count_a) {
                    let prim_a = &self.primitives[i];
                    for j in *first_b..(*first_b + *count_b) {
                        let prim_b = &other.primitives[j];
                        let aabb_b_prim = transform_b_aabb(&prim_b.aabb);
                        if prim_a.aabb.overlaps(&aabb_b_prim) {
                            results.push((prim_a.index, prim_b.index));
                        }
                    }
                }
            }
            // A internal, B leaf — descend into A
            (BvhNode::Internal { left, right, .. }, BvhNode::Leaf { .. }) => {
                self.query_pairs_recursive(*left, other, idx_b, transform_b_aabb, results);
                self.query_pairs_recursive(*right, other, idx_b, transform_b_aabb, results);
            }
            // A leaf, B internal — descend into B
            (BvhNode::Leaf { .. }, BvhNode::Internal { left, right, .. }) => {
                self.query_pairs_recursive(idx_a, other, *left, transform_b_aabb, results);
                self.query_pairs_recursive(idx_a, other, *right, transform_b_aabb, results);
            }
            // Both internal — descend into the larger one (volume heuristic)
            (
                BvhNode::Internal {
                    left: left_a,
                    right: right_a,
                    aabb: aabb_a,
                },
                BvhNode::Internal {
                    left: left_b,
                    right: right_b,
                    ..
                },
            ) => {
                let vol_a = aabb_a.volume();
                let vol_b = aabb_b_in_a.volume();

                if vol_a >= vol_b {
                    self.query_pairs_recursive(*left_a, other, idx_b, transform_b_aabb, results);
                    self.query_pairs_recursive(*right_a, other, idx_b, transform_b_aabb, results);
                } else {
                    self.query_pairs_recursive(idx_a, other, *left_b, transform_b_aabb, results);
                    self.query_pairs_recursive(idx_a, other, *right_b, transform_b_aabb, results);
                }
            }
        }
    }

    // -- Ray queries --------------------------------------------------------

    /// Finds primitives whose AABB intersects a ray.
    ///
    /// `ray_dir_inv` is the **inverse** of the ray direction (`1/dir` per
    /// component). For axis-aligned rays where a component is zero, pass
    /// `f64::INFINITY` or `f64::NEG_INFINITY` — the slab test handles IEEE
    /// 754 infinity arithmetic correctly.
    ///
    /// Returns candidate primitive indices; the caller must do precise tests.
    #[must_use]
    pub fn query_ray(
        &self,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &Vector3<f64>,
        max_distance: f64,
    ) -> Vec<usize> {
        let mut results = Vec::new();
        if !self.nodes.is_empty() {
            self.query_ray_recursive(0, ray_origin, ray_dir_inv, max_distance, &mut results);
        }
        results
    }

    fn query_ray_recursive(
        &self,
        node_idx: usize,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &Vector3<f64>,
        max_distance: f64,
        results: &mut Vec<usize>,
    ) {
        let node = &self.nodes[node_idx];

        if ray_aabb_intersect_dist(ray_origin, ray_dir_inv, node.aabb(), max_distance).is_none() {
            return;
        }

        match node {
            BvhNode::Internal { left, right, .. } => {
                self.query_ray_recursive(*left, ray_origin, ray_dir_inv, max_distance, results);
                self.query_ray_recursive(*right, ray_origin, ray_dir_inv, max_distance, results);
            }
            BvhNode::Leaf {
                first_primitive,
                primitive_count,
                ..
            } => {
                for i in *first_primitive..(*first_primitive + *primitive_count) {
                    results.push(self.primitives[i].index);
                }
            }
        }
    }

    /// Finds the closest ray hit with progressive pruning.
    ///
    /// `test_primitive` receives a primitive index and returns `Some(t)` if
    /// the ray hits at distance `t`, or `None`. The BVH uses each hit to
    /// tighten the cutoff, pruning branches that can't contain closer hits.
    ///
    /// Children are visited in front-to-back order for optimal pruning.
    pub fn query_ray_closest<F>(
        &self,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &Vector3<f64>,
        max_distance: f64,
        mut test_primitive: F,
    ) -> Option<(usize, f64)>
    where
        F: FnMut(usize) -> Option<f64>,
    {
        if self.nodes.is_empty() {
            return None;
        }

        let mut closest: Option<(usize, f64)> = None;
        let mut cutoff = max_distance;

        // Explicit stack avoids recursion overhead
        let mut stack = Vec::with_capacity(64);
        stack.push(0usize);

        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];

            let Some(tmin) = ray_aabb_intersect_dist(ray_origin, ray_dir_inv, node.aabb(), cutoff)
            else {
                continue;
            };

            if tmin > cutoff {
                continue;
            }

            match node {
                BvhNode::Internal { left, right, .. } => {
                    // Front-to-back ordering: push farther child first
                    let left_t = ray_aabb_intersect_dist(
                        ray_origin,
                        ray_dir_inv,
                        self.nodes[*left].aabb(),
                        cutoff,
                    );
                    let right_t = ray_aabb_intersect_dist(
                        ray_origin,
                        ray_dir_inv,
                        self.nodes[*right].aabb(),
                        cutoff,
                    );

                    match (left_t, right_t) {
                        (Some(lt), Some(rt)) => {
                            if lt < rt {
                                stack.push(*right);
                                stack.push(*left);
                            } else {
                                stack.push(*left);
                                stack.push(*right);
                            }
                        }
                        (Some(_), None) => stack.push(*left),
                        (None, Some(_)) => stack.push(*right),
                        (None, None) => {}
                    }
                }
                BvhNode::Leaf {
                    first_primitive,
                    primitive_count,
                    ..
                } => {
                    for i in *first_primitive..(*first_primitive + *primitive_count) {
                        let prim_idx = self.primitives[i].index;
                        if let Some(hit_t) = test_primitive(prim_idx) {
                            if hit_t < cutoff {
                                cutoff = hit_t;
                                closest = Some((prim_idx, hit_t));
                            }
                        }
                    }
                }
            }
        }

        closest
    }

    // -- Inspection ---------------------------------------------------------

    /// Returns the root AABB, or `None` if the BVH is empty.
    #[must_use]
    pub fn root_aabb(&self) -> Option<&Aabb> {
        self.nodes.first().map(BvhNode::aabb)
    }

    /// Returns the number of internal + leaf nodes.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the number of primitives stored.
    #[must_use]
    pub const fn primitive_count(&self) -> usize {
        self.primitives.len()
    }

    /// Returns `true` if the BVH contains no primitives.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.primitives.is_empty()
    }

    /// Returns the primitives (reordered during construction).
    #[must_use]
    pub fn primitives(&self) -> &[BvhPrimitive] {
        &self.primitives
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Builds a BVH from an [`IndexedMesh`].
///
/// Each triangle becomes one [`BvhPrimitive`] whose AABB tightly encloses
/// the triangle's three vertices.
#[must_use]
pub fn bvh_from_mesh(mesh: &IndexedMesh) -> Bvh {
    let mut primitives = Vec::with_capacity(mesh.face_count());

    for (tri_idx, &[i0, i1, i2]) in mesh.faces.iter().enumerate() {
        let v0 = mesh.vertices[i0 as usize];
        let v1 = mesh.vertices[i1 as usize];
        let v2 = mesh.vertices[i2 as usize];
        primitives.push(BvhPrimitive::from_triangle(v0, v1, v2, tri_idx));
    }

    Bvh::build(primitives)
}

/// Builds a BVH from raw vertex/index arrays (triangle-soup format).
///
/// `indices` must have a length that is a multiple of 3. Each consecutive
/// triple of indices defines one triangle.
///
/// # Panics
///
/// Panics if `indices.len()` is not a multiple of 3.
#[must_use]
pub fn bvh_from_triangle_mesh(vertices: &[Point3<f64>], indices: &[usize]) -> Bvh {
    assert!(
        indices.len().is_multiple_of(3),
        "indices.len() must be a multiple of 3"
    );

    let mut primitives = Vec::with_capacity(indices.len() / 3);

    for (tri_idx, tri) in indices.chunks_exact(3).enumerate() {
        let v0 = vertices[tri[0]];
        let v1 = vertices[tri[1]];
        let v2 = vertices[tri[2]];
        primitives.push(BvhPrimitive::from_triangle(v0, v1, v2, tri_idx));
    }

    Bvh::build(primitives)
}

/// Queries two BVHs for overlapping primitive pairs under isometry transforms.
///
/// Both BVHs are in their respective body's local space. The function
/// computes the relative transform `A⁻¹ · B` and performs a dual-tree
/// traversal, returning candidate `(index_a, index_b)` pairs.
#[must_use]
pub fn query_bvh_pair(
    bvh_a: &Bvh,
    bvh_b: &Bvh,
    transform_a: &Isometry3<f64>,
    transform_b: &Isometry3<f64>,
) -> Vec<(usize, usize)> {
    let transform_b_to_a = transform_a.inverse() * transform_b;
    bvh_a.query_pairs(bvh_b, |aabb_b| transform_aabb(aabb_b, &transform_b_to_a))
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Computes the bounding box enclosing all primitives.
fn compute_bounds(primitives: &[BvhPrimitive]) -> Aabb {
    let mut result = Aabb::empty();
    for prim in primitives {
        result = result.union(&prim.aabb);
    }
    result
}

/// Transforms an AABB by an isometry (conservative — the result bounds the
/// rotated box).
fn transform_aabb(aabb: &Aabb, transform: &Isometry3<f64>) -> Aabb {
    let corners = aabb.corners();

    let first = transform * corners[0];
    let mut min = first;
    let mut max = first;

    for corner in &corners[1..] {
        let t = transform * corner;
        min.x = min.x.min(t.x);
        min.y = min.y.min(t.y);
        min.z = min.z.min(t.z);
        max.x = max.x.max(t.x);
        max.y = max.y.max(t.y);
        max.z = max.z.max(t.z);
    }

    Aabb::new(min, max)
}

/// Ray-AABB slab test returning the entry distance.
///
/// `ray_dir_inv` is the precomputed `1/direction`. IEEE 754 infinity
/// arithmetic handles axis-aligned rays correctly: `(boundary - origin) * ∞`
/// produces the correct `±∞`, and the min/max logic propagates correctly.
///
/// Returns `Some(tmin)` if the ray hits, `None` if it misses or is beyond
/// `max_distance`.
fn ray_aabb_intersect_dist(
    ray_origin: &Point3<f64>,
    ray_dir_inv: &Vector3<f64>,
    aabb: &Aabb,
    max_distance: f64,
) -> Option<f64> {
    let t1x = (aabb.min.x - ray_origin.x) * ray_dir_inv.x;
    let t2x = (aabb.max.x - ray_origin.x) * ray_dir_inv.x;
    let t1y = (aabb.min.y - ray_origin.y) * ray_dir_inv.y;
    let t2y = (aabb.max.y - ray_origin.y) * ray_dir_inv.y;
    let t1z = (aabb.min.z - ray_origin.z) * ray_dir_inv.z;
    let t2z = (aabb.max.z - ray_origin.z) * ray_dir_inv.z;

    let tmin_x = t1x.min(t2x);
    let tmax_x = t1x.max(t2x);
    let tmin_y = t1y.min(t2y);
    let tmax_y = t1y.max(t2y);
    let tmin_z = t1z.min(t2z);
    let tmax_z = t1z.max(t2z);

    let tmin = tmin_x.max(tmin_y).max(tmin_z);
    let tmax = tmax_x.min(tmax_y).min(tmax_z);

    // NaN from 0 * ∞ on slab boundary — reject
    if tmin.is_nan() || tmax.is_nan() {
        return None;
    }

    if tmax >= tmin && tmin < max_distance && tmax >= 0.0 {
        Some(tmin.max(0.0))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;

    // -- helpers ------------------------------------------------------------

    fn test_primitives() -> Vec<BvhPrimitive> {
        vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(3.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                1,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(6.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                2,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(9.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                3,
            ),
        ]
    }

    /// Makes a simple tetrahedron mesh for `bvh_from_mesh` tests.
    fn tetrahedron_mesh() -> IndexedMesh {
        IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.5, 1.0, 0.0),
                Point3::new(0.5, 0.5, 1.0),
            ],
            vec![[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]],
        )
    }

    // -- construction -------------------------------------------------------

    #[test]
    fn build_basic() {
        let bvh = Bvh::build(test_primitives());
        assert_eq!(bvh.primitive_count(), 4);
        assert!(!bvh.is_empty());
        assert!(bvh.node_count() > 0);
    }

    #[test]
    fn build_empty() {
        let bvh = Bvh::build(vec![]);
        assert!(bvh.is_empty());
        assert_eq!(bvh.node_count(), 0);
        assert!(bvh.root_aabb().is_none());
        assert!(
            bvh.query(&Aabb::from_center(
                Point3::origin(),
                Vector3::new(1.0, 1.0, 1.0)
            ))
            .is_empty()
        );
    }

    #[test]
    fn build_single_primitive() {
        let bvh = Bvh::build(vec![BvhPrimitive::new(
            Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0)),
            42,
        )]);

        assert_eq!(bvh.primitive_count(), 1);
        assert_eq!(bvh.node_count(), 1); // single leaf

        let results = bvh.query(&Aabb::from_center(
            Point3::origin(),
            Vector3::new(0.5, 0.5, 0.5),
        ));
        assert_eq!(results, vec![42]);
    }

    #[test]
    fn build_with_custom_leaf_size() {
        let bvh = Bvh::new()
            .with_max_primitives_per_leaf(2)
            .build_from(test_primitives());

        assert_eq!(bvh.primitive_count(), 4);
        // With max 2 per leaf and 4 primitives, we need at least 3 nodes
        assert!(bvh.node_count() >= 3);
    }

    #[test]
    fn root_aabb_bounds_all_primitives() {
        let bvh = Bvh::build(test_primitives());
        let root = bvh.root_aabb().unwrap();
        // Primitives span x: [-1, 10], y: [-1, 1], z: [-1, 1]
        assert!(root.min.x <= -1.0);
        assert!(root.max.x >= 10.0);
    }

    // -- AABB queries -------------------------------------------------------

    #[test]
    fn query_hit() {
        let bvh = Bvh::build(test_primitives());
        let query = Aabb::from_center(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let results = bvh.query(&query);
        assert!(results.contains(&0));
    }

    #[test]
    fn query_miss() {
        let bvh = Bvh::build(test_primitives());
        let query = Aabb::from_center(Point3::new(-5.0, 0.0, 0.0), Vector3::new(0.5, 0.5, 0.5));
        let results = bvh.query(&query);
        assert!(results.is_empty());
    }

    #[test]
    fn query_multiple() {
        let prims = vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                1,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(2.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                2,
            ),
        ];
        let bvh = Bvh::build(prims);

        let query = Aabb::from_center(Point3::new(1.0, 0.0, 0.0), Vector3::new(3.0, 1.0, 1.0));
        let results = bvh.query(&query);
        assert!(results.len() >= 2);
    }

    #[test]
    fn query_callback_counts() {
        let bvh = Bvh::build(test_primitives());
        let query = Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0));
        let mut count = 0;
        bvh.query_callback(&query, |_| count += 1);
        assert!(count > 0);
    }

    // -- dual-BVH queries ---------------------------------------------------

    #[test]
    fn query_pairs_overlap() {
        let prims_a = vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(3.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                1,
            ),
        ];
        let prims_b = vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(10.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                1,
            ),
        ];

        let bvh_a = Bvh::build(prims_a);
        let bvh_b = Bvh::build(prims_b);

        let pairs = bvh_a.query_pairs(&bvh_b, |aabb| *aabb);
        assert!(pairs.iter().any(|&(a, b)| a == 0 && b == 0));
        assert!(!pairs.iter().any(|&(a, b)| a == 1 && b == 1));
    }

    #[test]
    fn query_bvh_pair_identity() {
        let prims_a = vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
                1,
            ),
        ];
        let prims_b = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];

        let bvh_a = Bvh::build(prims_a);
        let bvh_b = Bvh::build(prims_b);

        let pairs = query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::identity(),
        );
        assert!(pairs.iter().any(|&(a, b)| a == 0 && b == 0));
        assert!(!pairs.iter().any(|&(a, _)| a == 1));
    }

    #[test]
    fn query_bvh_pair_translation() {
        let prims = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];
        let bvh_a = Bvh::build(prims.clone());
        let bvh_b = Bvh::build(prims);

        // Far apart — no overlap
        let pairs_far = query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::translation(10.0, 0.0, 0.0),
        );
        assert!(pairs_far.is_empty());

        // Close — overlap
        let pairs_close = query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::translation(0.5, 0.0, 0.0),
        );
        assert!(!pairs_close.is_empty());
    }

    #[test]
    fn query_bvh_pair_rotation() {
        let prims_a = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];
        let prims_b = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::origin(), Vector3::new(3.0, 0.2, 0.2)),
            0,
        )];

        let bvh_a = Bvh::build(prims_a);
        let bvh_b = Bvh::build(prims_b);

        // Identity — overlap
        let pairs = query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::identity(),
        );
        assert!(!pairs.is_empty());

        // B translated far on X — no overlap
        let pairs_far = query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::translation(5.0, 0.0, 0.0),
        );
        assert!(pairs_far.is_empty());
    }

    // -- BvhPrimitive -------------------------------------------------------

    #[test]
    fn primitive_from_triangle_aabb() {
        let prim = BvhPrimitive::from_triangle(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            7,
        );

        assert_eq!(prim.index, 7);
        assert!(prim.aabb.min.x <= 0.0);
        assert!(prim.aabb.max.x >= 1.0);
        assert!(prim.aabb.min.y <= 0.0);
        assert!(prim.aabb.max.y >= 1.0);
        assert!((prim.aabb.min.z - 0.0).abs() < f64::EPSILON);
        assert!((prim.aabb.max.z - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn primitive_with_data() {
        let prim = BvhPrimitive::with_data(
            Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0)),
            5,
            99,
        );
        assert_eq!(prim.index, 5);
        assert_eq!(prim.data, 99);
    }

    #[test]
    fn primitive_centroid() {
        let prim = BvhPrimitive::new(
            Aabb::new(Point3::new(2.0, 4.0, 6.0), Point3::new(4.0, 8.0, 10.0)),
            0,
        );
        let c = prim.centroid();
        assert!((c.x - 3.0).abs() < f64::EPSILON);
        assert!((c.y - 6.0).abs() < f64::EPSILON);
        assert!((c.z - 8.0).abs() < f64::EPSILON);
    }

    // -- bvh_from_mesh ------------------------------------------------------

    #[test]
    fn bvh_from_mesh_tetrahedron() {
        let mesh = tetrahedron_mesh();
        let bvh = bvh_from_mesh(&mesh);

        assert_eq!(bvh.primitive_count(), 4);
        assert!(!bvh.is_empty());

        // Query the whole mesh's AABB — should return all faces
        let root = bvh.root_aabb().unwrap();
        let results = bvh.query(root);
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn bvh_from_mesh_empty() {
        let mesh = IndexedMesh::new();
        let bvh = bvh_from_mesh(&mesh);
        assert!(bvh.is_empty());
    }

    // -- bvh_from_triangle_mesh ---------------------------------------------

    #[test]
    fn bvh_from_triangle_mesh_basic() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let indices = vec![0, 1, 2, 0, 1, 3];
        let bvh = bvh_from_triangle_mesh(&vertices, &indices);
        assert_eq!(bvh.primitive_count(), 2);
    }

    #[test]
    #[should_panic(expected = "multiple of 3")]
    fn bvh_from_triangle_mesh_bad_indices() {
        let vertices = vec![Point3::origin()];
        drop(bvh_from_triangle_mesh(&vertices, &[0, 1]));
    }

    // -- ray queries --------------------------------------------------------

    #[test]
    fn query_ray_hit() {
        // A row of 4 boxes along X
        let bvh = Bvh::build(test_primitives());

        // Ray along +X from far left
        let origin = Point3::new(-5.0, 0.0, 0.0);
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let dir_inv = Vector3::new(1.0 / dir.x, f64::INFINITY, f64::INFINITY);

        let results = bvh.query_ray(&origin, &dir_inv, 100.0);
        // Should hit all 4 primitives
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn query_ray_miss() {
        let bvh = Bvh::build(test_primitives());

        // Ray along +Y that misses everything
        let origin = Point3::new(-5.0, 5.0, 0.0);
        let dir_inv = Vector3::new(f64::INFINITY, 1.0, f64::INFINITY);

        let results = bvh.query_ray(&origin, &dir_inv, 100.0);
        assert!(results.is_empty());
    }

    #[test]
    fn query_ray_max_distance() {
        // Force a deeper tree so max_distance actually prunes branches
        let bvh = Bvh::new()
            .with_max_primitives_per_leaf(1)
            .build_from(test_primitives());

        // Ray along +X but with short max distance (only reaches first box)
        // Origin at -5, first box at x=[-1,1] (entry t=4), second at [2,4] (entry t=7)
        let origin = Point3::new(-5.0, 0.0, 0.0);
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY);

        let results = bvh.query_ray(&origin, &dir_inv, 6.0);
        // With max_distance=6, only the first primitive (entry at t=4) should be reached
        assert!(results.contains(&0));
        assert!(!results.contains(&3)); // primitive at x=9 is way beyond
    }

    #[test]
    fn query_ray_closest_finds_nearest() {
        // Two boxes: one at x=2, one at x=10
        let prims = vec![
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.5, 0.5, 0.5)),
                0,
            ),
            BvhPrimitive::new(
                Aabb::from_center(Point3::new(10.0, 0.0, 0.0), Vector3::new(0.5, 0.5, 0.5)),
                1,
            ),
        ];
        let bvh = Bvh::build(prims);

        let origin = Point3::new(0.0, 0.0, 0.0);
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY);

        // test_primitive returns the centroid distance as a fake hit
        let result = bvh.query_ray_closest(&origin, &dir_inv, f64::INFINITY, |idx| {
            match idx {
                0 => Some(2.0),  // closer
                1 => Some(10.0), // farther
                _ => None,
            }
        });

        assert_eq!(result, Some((0, 2.0)));
    }

    #[test]
    fn query_ray_closest_empty_bvh() {
        let bvh = Bvh::build(vec![]);
        let result = bvh.query_ray_closest(
            &Point3::origin(),
            &Vector3::new(1.0, f64::INFINITY, f64::INFINITY),
            100.0,
            |_| Some(1.0),
        );
        assert!(result.is_none());
    }

    // -- transform_aabb (internal) ------------------------------------------

    #[test]
    fn transform_aabb_identity() {
        let aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let result = transform_aabb(&aabb, &Isometry3::identity());
        assert!((result.min.x - aabb.min.x).abs() < 1e-10);
        assert!((result.max.x - aabb.max.x).abs() < 1e-10);
    }

    #[test]
    fn transform_aabb_translation() {
        let aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let result = transform_aabb(&aabb, &Isometry3::translation(5.0, 0.0, 0.0));
        assert!((result.min.x - (-1.0 + 5.0)).abs() < 1e-10);
        assert!((result.max.x - (1.0 + 5.0)).abs() < 1e-10);
    }

    // -- ray_aabb_intersect_dist (internal) ---------------------------------

    #[test]
    fn ray_aabb_hit() {
        let aabb = Aabb::from_center(Point3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let origin = Point3::origin();
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY);

        let t = ray_aabb_intersect_dist(&origin, &dir_inv, &aabb, f64::INFINITY);
        assert!(t.is_some());
        let t = t.unwrap();
        assert!((t - 4.0).abs() < 1e-10); // entry at x=4
    }

    #[test]
    fn ray_aabb_miss() {
        let aabb = Aabb::from_center(Point3::new(5.0, 5.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let origin = Point3::origin();
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY); // along +X

        let t = ray_aabb_intersect_dist(&origin, &dir_inv, &aabb, f64::INFINITY);
        assert!(t.is_none());
    }

    #[test]
    fn ray_aabb_origin_inside() {
        let aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let origin = Point3::origin();
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY);

        let t = ray_aabb_intersect_dist(&origin, &dir_inv, &aabb, f64::INFINITY);
        assert!(t.is_some());
        assert!((t.unwrap() - 0.0).abs() < 1e-10); // clamped to 0
    }

    #[test]
    fn ray_aabb_max_distance_cutoff() {
        let aabb = Aabb::from_center(Point3::new(100.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let origin = Point3::origin();
        let dir_inv = Vector3::new(1.0, f64::INFINITY, f64::INFINITY);

        let t = ray_aabb_intersect_dist(&origin, &dir_inv, &aabb, 10.0);
        assert!(t.is_none()); // beyond max_distance
    }
}
