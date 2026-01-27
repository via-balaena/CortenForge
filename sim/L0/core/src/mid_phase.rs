//! Mid-phase collision detection using Bounding Volume Hierarchies (BVH).
//!
//! The mid-phase sits between broad-phase (which identifies potentially colliding body pairs)
//! and narrow-phase (which computes exact contact information). For complex meshes with many
//! primitives, the mid-phase uses an AABB tree to quickly cull non-colliding primitive pairs.
//!
//! # Algorithm
//!
//! The BVH is built using a top-down approach:
//! 1. Compute AABB for all primitives
//! 2. Find the axis with maximum extent
//! 3. Sort primitives by centroid along that axis
//! 4. Split at median and recurse
//!
//! Queries traverse the tree, only descending when AABBs overlap.
//!
//! # Usage
//!
//! ```ignore
//! use sim_core::mid_phase::{Bvh, BvhPrimitive, Aabb};
//! use nalgebra::Point3;
//!
//! // Create primitives (e.g., triangles from a mesh)
//! let primitives: Vec<BvhPrimitive> = /* ... */;
//!
//! // Build the BVH
//! let bvh = Bvh::build(primitives);
//!
//! // Query for potentially colliding primitives
//! let query_aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
//! for &primitive_idx in bvh.query(&query_aabb) {
//!     // Perform narrow-phase test on primitive
//! }
//! ```

use crate::collision_shape::{Aabb, Axis};
use nalgebra::{Isometry3, Point3};

/// A primitive that can be stored in the BVH.
///
/// Primitives can represent triangles, convex parts, or any geometry
/// that has an associated AABB.
#[derive(Debug, Clone)]
pub struct BvhPrimitive {
    /// Axis-aligned bounding box of this primitive.
    pub aabb: Aabb,
    /// Index into the original primitive array (e.g., triangle index).
    pub index: usize,
    /// Optional additional data (e.g., material ID).
    pub data: u32,
}

impl BvhPrimitive {
    /// Create a new primitive.
    #[must_use]
    pub fn new(aabb: Aabb, index: usize) -> Self {
        Self {
            aabb,
            index,
            data: 0,
        }
    }

    /// Create a new primitive with additional data.
    #[must_use]
    pub fn with_data(aabb: Aabb, index: usize, data: u32) -> Self {
        Self { aabb, index, data }
    }

    /// Create a primitive from a triangle.
    #[must_use]
    pub fn from_triangle(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>, index: usize) -> Self {
        let min = Point3::new(
            v0.x.min(v1.x).min(v2.x),
            v0.y.min(v1.y).min(v2.y),
            v0.z.min(v1.z).min(v2.z),
        );
        let max = Point3::new(
            v0.x.max(v1.x).max(v2.x),
            v0.y.max(v1.y).max(v2.y),
            v0.z.max(v1.z).max(v2.z),
        );
        Self::new(Aabb::new(min, max), index)
    }

    /// Get the centroid of this primitive's AABB.
    #[must_use]
    pub fn centroid(&self) -> Point3<f64> {
        Point3::from((self.aabb.min.coords + self.aabb.max.coords) * 0.5)
    }
}

/// A node in the BVH tree.
#[derive(Debug, Clone)]
enum BvhNode {
    /// Internal node with two children.
    Internal {
        /// Bounding box containing all descendants.
        aabb: Aabb,
        /// Index of left child in the nodes array.
        left: usize,
        /// Index of right child in the nodes array.
        right: usize,
    },
    /// Leaf node containing primitives.
    Leaf {
        /// Bounding box of this leaf.
        aabb: Aabb,
        /// Start index in the primitives array.
        first_primitive: usize,
        /// Number of primitives in this leaf.
        primitive_count: usize,
    },
}

impl BvhNode {
    /// Get the AABB for this node.
    #[must_use]
    fn aabb(&self) -> &Aabb {
        match self {
            Self::Internal { aabb, .. } | Self::Leaf { aabb, .. } => aabb,
        }
    }
}

/// Bounding Volume Hierarchy for efficient spatial queries.
///
/// The BVH organizes primitives in a binary tree structure where each node
/// contains an AABB that bounds all of its descendants. This allows quick
/// culling of non-overlapping regions during collision queries.
#[derive(Debug, Clone)]
pub struct Bvh {
    /// The tree nodes (index 0 is the root).
    nodes: Vec<BvhNode>,
    /// The primitives, reordered during construction.
    primitives: Vec<BvhPrimitive>,
    /// Maximum primitives per leaf node.
    max_primitives_per_leaf: usize,
}

impl Default for Bvh {
    fn default() -> Self {
        Self::new()
    }
}

impl Bvh {
    /// Create an empty BVH.
    #[must_use]
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            primitives: Vec::new(),
            max_primitives_per_leaf: 4,
        }
    }

    /// Set the maximum number of primitives per leaf node.
    ///
    /// Lower values result in deeper trees but faster narrow-phase queries.
    /// Higher values result in shallower trees but more narrow-phase tests per query.
    /// Default is 4.
    #[must_use]
    pub fn with_max_primitives_per_leaf(mut self, max: usize) -> Self {
        self.max_primitives_per_leaf = max.max(1);
        self
    }

    /// Build a BVH from a list of primitives.
    #[must_use]
    pub fn build(primitives: Vec<BvhPrimitive>) -> Self {
        Self::new().build_from(primitives)
    }

    /// Build the BVH from primitives, consuming the builder.
    #[must_use]
    pub fn build_from(mut self, mut primitives: Vec<BvhPrimitive>) -> Self {
        if primitives.is_empty() {
            return self;
        }

        self.nodes.clear();
        self.nodes.reserve(primitives.len() * 2); // Upper bound for binary tree

        // Build recursively
        let len = primitives.len();
        self.build_recursive(&mut primitives, 0, len);

        self.primitives = primitives;
        self
    }

    /// Recursively build the BVH.
    ///
    /// Returns the index of the created node.
    fn build_recursive(
        &mut self,
        primitives: &mut [BvhPrimitive],
        start: usize,
        end: usize,
    ) -> usize {
        let count = end - start;
        let slice = &primitives[start..end];

        // Compute bounding box for this node
        let aabb = Self::compute_bounds(slice);

        // Create leaf if few enough primitives
        if count <= self.max_primitives_per_leaf {
            let node_idx = self.nodes.len();
            self.nodes.push(BvhNode::Leaf {
                aabb,
                first_primitive: start,
                primitive_count: count,
            });
            return node_idx;
        }

        // Find the best axis to split on (largest extent)
        let extent = aabb.max - aabb.min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            Axis::X
        } else if extent.y >= extent.z {
            Axis::Y
        } else {
            Axis::Z
        };

        // Sort primitives by centroid along the chosen axis
        let primitives_slice = &mut primitives[start..end];
        primitives_slice.sort_by(|a, b| {
            let ca = a.centroid();
            let cb = b.centroid();
            let va = match axis {
                Axis::X => ca.x,
                Axis::Y => ca.y,
                Axis::Z => ca.z,
            };
            let vb = match axis {
                Axis::X => cb.x,
                Axis::Y => cb.y,
                Axis::Z => cb.z,
            };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Split at median
        let mid = start + count / 2;

        // Reserve space for this internal node (we'll fill in children later)
        let node_idx = self.nodes.len();
        self.nodes.push(BvhNode::Internal {
            aabb,
            left: 0,  // Placeholder
            right: 0, // Placeholder
        });

        // Build children
        let left_idx = self.build_recursive(primitives, start, mid);
        let right_idx = self.build_recursive(primitives, mid, end);

        // Update this node with actual child indices
        if let BvhNode::Internal {
            left,
            right,
            aabb: _,
        } = &mut self.nodes[node_idx]
        {
            *left = left_idx;
            *right = right_idx;
        }

        node_idx
    }

    /// Compute the bounding box for a set of primitives.
    fn compute_bounds(primitives: &[BvhPrimitive]) -> Aabb {
        if primitives.is_empty() {
            return Aabb::default();
        }

        let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for prim in primitives {
            min.x = min.x.min(prim.aabb.min.x);
            min.y = min.y.min(prim.aabb.min.y);
            min.z = min.z.min(prim.aabb.min.z);
            max.x = max.x.max(prim.aabb.max.x);
            max.y = max.y.max(prim.aabb.max.y);
            max.z = max.z.max(prim.aabb.max.z);
        }

        Aabb::new(min, max)
    }

    /// Query the BVH for primitives that may overlap with the given AABB.
    ///
    /// Returns indices of potentially overlapping primitives.
    /// The caller should perform precise intersection tests on these primitives.
    #[must_use]
    pub fn query(&self, query_aabb: &Aabb) -> Vec<usize> {
        let mut results = Vec::new();
        if !self.nodes.is_empty() {
            self.query_recursive(0, query_aabb, &mut results);
        }
        results
    }

    /// Recursively query the BVH.
    fn query_recursive(&self, node_idx: usize, query_aabb: &Aabb, results: &mut Vec<usize>) {
        let node = &self.nodes[node_idx];

        // Early out if no overlap with this node's bounds
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
                // Add all primitives in this leaf that overlap
                for i in *first_primitive..(*first_primitive + *primitive_count) {
                    if self.primitives[i].aabb.overlaps(query_aabb) {
                        results.push(self.primitives[i].index);
                    }
                }
            }
        }
    }

    /// Query the BVH with a callback for each potentially overlapping primitive.
    ///
    /// More efficient than [`Self::query`] when you don't need to collect all results.
    pub fn query_callback<F>(&self, query_aabb: &Aabb, mut callback: F)
    where
        F: FnMut(&BvhPrimitive),
    {
        if !self.nodes.is_empty() {
            self.query_callback_recursive(0, query_aabb, &mut callback);
        }
    }

    /// Recursively query with callback.
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

    /// Query for potentially overlapping pairs between two BVHs.
    ///
    /// Both BVHs should be in their respective body's local space.
    /// The `transform_b` function transforms points from B's space to A's space.
    ///
    /// Returns pairs of (`primitive_idx_a`, `primitive_idx_b`) that may intersect.
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

    /// Recursively find overlapping pairs.
    #[allow(clippy::similar_names)]
    fn query_pairs_recursive<F>(
        &self,
        node_a_idx: usize,
        other: &Self,
        idx_b: usize,
        transform_b_aabb: &F,
        results: &mut Vec<(usize, usize)>,
    ) where
        F: Fn(&Aabb) -> Aabb,
    {
        let node_a = &self.nodes[node_a_idx];
        let node_b = &other.nodes[idx_b];

        // Transform B's AABB to A's space
        let aabb_b_in_a = transform_b_aabb(node_b.aabb());

        // Early out if no overlap
        if !node_a.aabb().overlaps(&aabb_b_in_a) {
            return;
        }

        match (node_a, node_b) {
            // Both are leaves - check primitive pairs
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
            // A is internal, B is leaf - descend into A
            (BvhNode::Internal { left, right, .. }, BvhNode::Leaf { .. }) => {
                self.query_pairs_recursive(*left, other, idx_b, transform_b_aabb, results);
                self.query_pairs_recursive(*right, other, idx_b, transform_b_aabb, results);
            }
            // A is leaf, B is internal - descend into B
            (BvhNode::Leaf { .. }, BvhNode::Internal { left, right, .. }) => {
                self.query_pairs_recursive(node_a_idx, other, *left, transform_b_aabb, results);
                self.query_pairs_recursive(node_a_idx, other, *right, transform_b_aabb, results);
            }
            // Both internal - descend into the larger one first (heuristic)
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
                let extent_a = aabb_a.max - aabb_a.min;
                let extent_b = aabb_b_in_a.max - aabb_b_in_a.min;
                let vol_a = extent_a.x * extent_a.y * extent_a.z;
                let vol_b = extent_b.x * extent_b.y * extent_b.z;

                if vol_a >= vol_b {
                    // Descend into A
                    self.query_pairs_recursive(*left_a, other, idx_b, transform_b_aabb, results);
                    self.query_pairs_recursive(*right_a, other, idx_b, transform_b_aabb, results);
                } else {
                    // Descend into B
                    self.query_pairs_recursive(
                        node_a_idx,
                        other,
                        *left_b,
                        transform_b_aabb,
                        results,
                    );
                    self.query_pairs_recursive(
                        node_a_idx,
                        other,
                        *right_b,
                        transform_b_aabb,
                        results,
                    );
                }
            }
        }
    }

    /// Get the root AABB of the BVH.
    #[must_use]
    pub fn root_aabb(&self) -> Option<&Aabb> {
        self.nodes.first().map(BvhNode::aabb)
    }

    /// Get the number of nodes in the BVH.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of primitives.
    #[must_use]
    pub fn primitive_count(&self) -> usize {
        self.primitives.len()
    }

    /// Check if the BVH is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.primitives.is_empty()
    }

    /// Get the primitives stored in this BVH.
    #[must_use]
    pub fn primitives(&self) -> &[BvhPrimitive] {
        &self.primitives
    }

    /// Query the BVH for primitives that a ray might intersect.
    ///
    /// Uses ray-AABB intersection to cull branches that can't contain hits.
    /// Returns indices of potentially intersecting primitives (caller must do precise test).
    ///
    /// # Arguments
    ///
    /// * `ray_origin` - Origin of the ray
    /// * `ray_dir_inv` - **Inverse** of ray direction (1/dir for each component).
    ///   For axis-aligned rays where a component is 0, pass `±∞` (the slab test
    ///   handles IEEE 754 infinity arithmetic correctly).
    /// * `max_distance` - Maximum ray distance (updated during traversal for early termination)
    ///
    /// # Performance
    ///
    /// - O(log n) average case for localized queries
    /// - Visits children in front-to-back order for better culling
    /// - Caller should update max_distance after each hit for optimal pruning
    #[must_use]
    pub fn query_ray(
        &self,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &nalgebra::Vector3<f64>,
        max_distance: f64,
    ) -> Vec<usize> {
        let mut results = Vec::new();
        if !self.nodes.is_empty() {
            self.query_ray_recursive(0, ray_origin, ray_dir_inv, max_distance, &mut results);
        }
        results
    }

    /// Query with early termination using a mutable max_distance.
    ///
    /// The callback receives (primitive_index, tmin) where tmin is the minimum
    /// distance to the primitive's AABB. Return the actual hit distance to update
    /// the cutoff, or `None` if no hit. This enables progressive pruning.
    pub fn query_ray_closest<F>(
        &self,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &nalgebra::Vector3<f64>,
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

        // Use explicit stack to avoid recursion overhead
        let mut stack = Vec::with_capacity(64);
        stack.push(0usize);

        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];

            // Test ray-AABB intersection with current cutoff
            let Some(tmin) = ray_aabb_intersect_dist(ray_origin, ray_dir_inv, node.aabb(), cutoff)
            else {
                continue;
            };

            // Skip if this node's nearest point is beyond our best hit
            if tmin > cutoff {
                continue;
            }

            match node {
                BvhNode::Internal { left, right, .. } => {
                    // Get distances to children for front-to-back ordering
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

                    // Push farther child first (so closer is processed first)
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

    /// Recursively query for ray intersections (simple version without early termination).
    fn query_ray_recursive(
        &self,
        node_idx: usize,
        ray_origin: &Point3<f64>,
        ray_dir_inv: &nalgebra::Vector3<f64>,
        max_distance: f64,
        results: &mut Vec<usize>,
    ) {
        let node = &self.nodes[node_idx];

        // Early out if ray doesn't hit this node's AABB
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
}

/// Ray-AABB intersection test (slab method) returning entry distance.
///
/// Uses the optimized slab test with precomputed 1/direction.
/// Returns `Some(tmin)` if ray intersects, where tmin is the entry distance.
/// Returns `None` if ray misses or intersection is beyond max_distance.
///
/// # IEEE 754 Correctness
///
/// When `ray_dir_inv` contains `±∞` (from 1/0 for axis-aligned rays):
/// - `(aabb.min.x - ray_origin.x) * ∞` = `±∞` depending on sign
/// - `∞.min(-∞)` = `-∞`, `∞.max(-∞)` = `∞`
/// - The slab logic correctly handles rays parallel to slab faces
fn ray_aabb_intersect_dist(
    ray_origin: &Point3<f64>,
    ray_dir_inv: &nalgebra::Vector3<f64>,
    aabb: &Aabb,
    max_distance: f64,
) -> Option<f64> {
    // Compute intersection distances for each slab
    let t1x = (aabb.min.x - ray_origin.x) * ray_dir_inv.x;
    let t2x = (aabb.max.x - ray_origin.x) * ray_dir_inv.x;
    let t1y = (aabb.min.y - ray_origin.y) * ray_dir_inv.y;
    let t2y = (aabb.max.y - ray_origin.y) * ray_dir_inv.y;
    let t1z = (aabb.min.z - ray_origin.z) * ray_dir_inv.z;
    let t2z = (aabb.max.z - ray_origin.z) * ray_dir_inv.z;

    // Find the min/max for each axis (handles negative directions)
    let tmin_x = t1x.min(t2x);
    let tmax_x = t1x.max(t2x);
    let tmin_y = t1y.min(t2y);
    let tmax_y = t1y.max(t2y);
    let tmin_z = t1z.min(t2z);
    let tmax_z = t1z.max(t2z);

    // Find the largest tmin and smallest tmax
    let tmin = tmin_x.max(tmin_y).max(tmin_z);
    let tmax = tmax_x.min(tmax_y).min(tmax_z);

    // Handle NaN: if any t-value is NaN (from 0*∞ when ray origin is on slab boundary
    // and direction is zero in that axis), reject the intersection
    if tmin.is_nan() || tmax.is_nan() {
        return None;
    }

    // Ray intersects if tmax >= tmin and tmin < max_distance and tmax >= 0
    if tmax >= tmin && tmin < max_distance && tmax >= 0.0 {
        Some(tmin.max(0.0)) // Clamp to 0 if origin is inside AABB
    } else {
        None
    }
}

/// Create a BVH from a triangle mesh.
///
/// Vertices are stored as a flat array of Point3, and indices reference
/// them in groups of 3 (triangle soup format).
///
/// # Panics
///
/// Panics if `indices.len()` is not a multiple of 3.
#[must_use]
pub fn bvh_from_triangle_mesh(vertices: &[Point3<f64>], indices: &[usize]) -> Bvh {
    assert!(indices.len() % 3 == 0, "Indices must be a multiple of 3");

    let mut primitives = Vec::with_capacity(indices.len() / 3);

    for (tri_idx, tri) in indices.chunks(3).enumerate() {
        let v0 = vertices[tri[0]];
        let v1 = vertices[tri[1]];
        let v2 = vertices[tri[2]];
        primitives.push(BvhPrimitive::from_triangle(v0, v1, v2, tri_idx));
    }

    Bvh::build(primitives)
}

/// Query two BVHs for potentially colliding triangle pairs.
///
/// Performs dual-tree traversal of two BVHs, pruning branches where AABBs don't overlap.
/// Returns candidate triangle pairs that may intersect, which should be tested with
/// narrow-phase collision detection (e.g., `triangle_triangle_intersection`).
///
/// # Arguments
///
/// * `bvh_a` - BVH for mesh A (in mesh A's local space)
/// * `bvh_b` - BVH for mesh B (in mesh B's local space)
/// * `transform_a` - Transform from mesh A's local space to world space
/// * `transform_b` - Transform from mesh B's local space to world space
///
/// # Returns
///
/// Vector of `(tri_index_a, tri_index_b)` pairs representing potentially colliding triangles.
///
/// # Example
///
/// ```ignore
/// use sim_core::mid_phase::{query_bvh_pair, Bvh};
/// use nalgebra::Isometry3;
///
/// let bvh_a = /* BVH for mesh A */;
/// let bvh_b = /* BVH for mesh B */;
/// let transform_a = Isometry3::identity();
/// let transform_b = Isometry3::translation(1.0, 0.0, 0.0);
///
/// let pairs = query_bvh_pair(&bvh_a, &bvh_b, &transform_a, &transform_b);
/// for (tri_a, tri_b) in pairs {
///     // Test triangles for intersection
/// }
/// ```
#[must_use]
pub fn query_bvh_pair(
    bvh_a: &Bvh,
    bvh_b: &Bvh,
    transform_a: &Isometry3<f64>,
    transform_b: &Isometry3<f64>,
) -> Vec<(usize, usize)> {
    // Compute the relative transform: B's local space to A's local space
    // If point_b is in B's local space, then:
    //   world = transform_b * point_b
    //   local_a = transform_a^-1 * world = transform_a^-1 * transform_b * point_b
    let transform_b_to_a = transform_a.inverse() * transform_b;

    // Use the existing query_pairs method with a transform function
    bvh_a.query_pairs(bvh_b, |aabb_b| transform_aabb(aabb_b, &transform_b_to_a))
}

/// Transform an AABB by an isometry.
///
/// Since AABBs are axis-aligned, transforming by a rotation will generally
/// produce a larger AABB that bounds the rotated box.
fn transform_aabb(aabb: &Aabb, transform: &Isometry3<f64>) -> Aabb {
    // Get the 8 corners of the AABB
    let corners = [
        Point3::new(aabb.min.x, aabb.min.y, aabb.min.z),
        Point3::new(aabb.max.x, aabb.min.y, aabb.min.z),
        Point3::new(aabb.min.x, aabb.max.y, aabb.min.z),
        Point3::new(aabb.max.x, aabb.max.y, aabb.min.z),
        Point3::new(aabb.min.x, aabb.min.y, aabb.max.z),
        Point3::new(aabb.max.x, aabb.min.y, aabb.max.z),
        Point3::new(aabb.min.x, aabb.max.y, aabb.max.z),
        Point3::new(aabb.max.x, aabb.max.y, aabb.max.z),
    ];

    // Transform all corners and compute new AABB
    let first = transform * corners[0];
    let mut min = first;
    let mut max = first;

    for corner in &corners[1..] {
        let transformed = transform * corner;
        min.x = min.x.min(transformed.x);
        min.y = min.y.min(transformed.y);
        min.z = min.z.min(transformed.z);
        max.x = max.x.max(transformed.x);
        max.y = max.y.max(transformed.y);
        max.z = max.z.max(transformed.z);
    }

    Aabb::new(min, max)
}

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
    use nalgebra::Vector3;

    fn create_test_primitives() -> Vec<BvhPrimitive> {
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

    #[test]
    fn test_bvh_build() {
        let primitives = create_test_primitives();
        let bvh = Bvh::build(primitives);

        assert_eq!(bvh.primitive_count(), 4);
        assert!(!bvh.is_empty());
        assert!(bvh.node_count() > 0);
    }

    #[test]
    fn test_bvh_query_hit() {
        let primitives = create_test_primitives();
        let bvh = Bvh::build(primitives);

        // Query that overlaps first primitive
        let query = Aabb::from_center(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let results = bvh.query(&query);

        assert!(results.contains(&0));
    }

    #[test]
    fn test_bvh_query_miss() {
        let primitives = create_test_primitives();
        let bvh = Bvh::build(primitives);

        // Query that doesn't overlap any primitive
        let query = Aabb::from_center(Point3::new(-5.0, 0.0, 0.0), Vector3::new(0.5, 0.5, 0.5));
        let results = bvh.query(&query);

        assert!(results.is_empty());
    }

    #[test]
    fn test_bvh_query_multiple() {
        let primitives = vec![
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
        let bvh = Bvh::build(primitives);

        // Query that overlaps all primitives
        let query = Aabb::from_center(Point3::new(1.0, 0.0, 0.0), Vector3::new(3.0, 1.0, 1.0));
        let results = bvh.query(&query);

        assert!(results.len() >= 2); // Should hit at least 2 primitives
    }

    #[test]
    fn test_bvh_callback() {
        let primitives = create_test_primitives();
        let bvh = Bvh::build(primitives);

        let query = Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0));
        let mut count = 0;
        bvh.query_callback(&query, |_prim| {
            count += 1;
        });

        assert!(count > 0);
    }

    #[test]
    fn test_bvh_from_triangles() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let indices = vec![0, 1, 2, 0, 1, 3]; // Two triangles

        let bvh = bvh_from_triangle_mesh(&vertices, &indices);

        assert_eq!(bvh.primitive_count(), 2);
    }

    #[test]
    fn test_bvh_root_aabb() {
        let primitives = create_test_primitives();
        let bvh = Bvh::build(primitives);

        let root = bvh.root_aabb().unwrap();
        // Root should encompass all primitives (0 to 10 on X)
        assert!(root.min.x <= -1.0);
        assert!(root.max.x >= 10.0);
    }

    #[test]
    fn test_bvh_empty() {
        let bvh = Bvh::build(vec![]);

        assert!(bvh.is_empty());
        assert_eq!(bvh.node_count(), 0);
        assert!(bvh.root_aabb().is_none());

        let results = bvh.query(&Aabb::from_center(
            Point3::origin(),
            Vector3::new(1.0, 1.0, 1.0),
        ));
        assert!(results.is_empty());
    }

    #[test]
    fn test_bvh_single_primitive() {
        let primitives = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0)),
            42,
        )];
        let bvh = Bvh::build(primitives);

        assert_eq!(bvh.primitive_count(), 1);
        assert_eq!(bvh.node_count(), 1); // Single leaf

        let results = bvh.query(&Aabb::from_center(
            Point3::origin(),
            Vector3::new(0.5, 0.5, 0.5),
        ));
        assert_eq!(results, vec![42]);
    }

    #[test]
    fn test_primitive_from_triangle() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let prim = BvhPrimitive::from_triangle(v0, v1, v2, 7);

        assert_eq!(prim.index, 7);
        assert!(prim.aabb.min.x <= 0.0);
        assert!(prim.aabb.max.x >= 1.0);
        assert!(prim.aabb.min.y <= 0.0);
        assert!(prim.aabb.max.y >= 1.0);
    }

    #[test]
    fn test_bvh_query_pairs() {
        // Create two BVHs
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

        // Identity transform
        let pairs = bvh_a.query_pairs(&bvh_b, |aabb| *aabb);

        // First primitive of A should overlap with first primitive of B
        assert!(pairs.iter().any(|&(a, b)| a == 0 && b == 0));
        // Second primitives should not overlap
        assert!(!pairs.iter().any(|&(a, b)| a == 1 && b == 1));
    }

    #[test]
    fn test_query_bvh_pair_identity_transform() {
        // Create two BVHs with overlapping primitives
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

        // With identity transforms, the BVHs overlap at first primitive
        let pairs = super::query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::identity(),
        );

        assert!(pairs.iter().any(|&(a, b)| a == 0 && b == 0));
        assert!(!pairs.iter().any(|&(a, _)| a == 1));
    }

    #[test]
    fn test_query_bvh_pair_with_translation() {
        // Create two BVHs
        let prims_a = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];
        let prims_b = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];

        let bvh_a = Bvh::build(prims_a);
        let bvh_b = Bvh::build(prims_b);

        // With identity transform for A and translation for B, they should not overlap
        let transform_a = Isometry3::identity();
        let transform_b = Isometry3::translation(10.0, 0.0, 0.0);

        let pairs = super::query_bvh_pair(&bvh_a, &bvh_b, &transform_a, &transform_b);
        assert!(pairs.is_empty(), "Separated BVHs should not produce pairs");

        // With B translated close, they should overlap
        let transform_b_close = Isometry3::translation(0.5, 0.0, 0.0);
        let pairs_close = super::query_bvh_pair(&bvh_a, &bvh_b, &transform_a, &transform_b_close);
        assert!(
            !pairs_close.is_empty(),
            "Overlapping BVHs should produce pairs"
        );
    }

    #[test]
    fn test_query_bvh_pair_with_rotation() {
        // Create a long thin BVH for B
        let prims_a = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
            0,
        )];
        // B is a long thin box along X
        let prims_b = vec![BvhPrimitive::new(
            Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(3.0, 0.2, 0.2)),
            0,
        )];

        let bvh_a = Bvh::build(prims_a);
        let bvh_b = Bvh::build(prims_b);

        // With identity transforms, they overlap
        let pairs = super::query_bvh_pair(
            &bvh_a,
            &bvh_b,
            &Isometry3::identity(),
            &Isometry3::identity(),
        );
        assert!(!pairs.is_empty());

        // With B translated far on X, they don't overlap
        let transform_b = Isometry3::translation(5.0, 0.0, 0.0);
        let pairs_far = super::query_bvh_pair(&bvh_a, &bvh_b, &Isometry3::identity(), &transform_b);
        assert!(pairs_far.is_empty());
    }

    #[test]
    fn test_transform_aabb() {
        let aabb = Aabb::from_center(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

        // Identity transform should preserve the AABB
        let identity = Isometry3::identity();
        let result = super::transform_aabb(&aabb, &identity);
        assert!((result.min.x - aabb.min.x).abs() < 1e-10);
        assert!((result.max.x - aabb.max.x).abs() < 1e-10);

        // Translation should shift the AABB
        let translation = Isometry3::translation(5.0, 0.0, 0.0);
        let result = super::transform_aabb(&aabb, &translation);
        assert!((result.min.x - (-1.0 + 5.0)).abs() < 1e-10);
        assert!((result.max.x - (1.0 + 5.0)).abs() < 1e-10);
    }
}
