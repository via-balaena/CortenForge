//! Bounding Volume Hierarchy for accelerated intersection queries.
//!
//! This module provides a BVH implementation optimized for triangle meshes,
//! enabling O(n log n + k) intersection queries instead of O(n*m) naive comparisons.

use mesh_types::{IndexedMesh, Point3};
use rayon::prelude::*;
use smallvec::SmallVec;

/// Axis-aligned bounding box for BVH nodes.
#[derive(Debug, Clone)]
pub struct Aabb {
    /// Minimum corner of the bounding box.
    pub min: Point3<f64>,
    /// Maximum corner of the bounding box.
    pub max: Point3<f64>,
}

impl Default for Aabb {
    fn default() -> Self {
        Self::empty()
    }
}

impl Aabb {
    /// Create an empty (inverted) bounding box.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::MAX, f64::MAX, f64::MAX),
            max: Point3::new(f64::MIN, f64::MIN, f64::MIN),
        }
    }

    /// Create a bounding box from a triangle.
    #[must_use]
    pub fn from_triangle(v0: &Point3<f64>, v1: &Point3<f64>, v2: &Point3<f64>) -> Self {
        Self {
            min: Point3::new(
                v0.x.min(v1.x).min(v2.x),
                v0.y.min(v1.y).min(v2.y),
                v0.z.min(v1.z).min(v2.z),
            ),
            max: Point3::new(
                v0.x.max(v1.x).max(v2.x),
                v0.y.max(v1.y).max(v2.y),
                v0.z.max(v1.z).max(v2.z),
            ),
        }
    }

    /// Create a bounding box from min and max points.
    #[must_use]
    pub fn from_min_max(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self { min, max }
    }

    /// Expand this bounding box to include another.
    pub fn expand(&mut self, other: &Self) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.min.z = self.min.z.min(other.min.z);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
        self.max.z = self.max.z.max(other.max.z);
    }

    /// Expand this bounding box to include a point.
    pub fn expand_point(&mut self, point: &Point3<f64>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Check if this bounding box intersects another, with tolerance.
    #[must_use]
    pub fn intersects(&self, other: &Self, tolerance: f64) -> bool {
        !(self.max.x + tolerance < other.min.x
            || other.max.x + tolerance < self.min.x
            || self.max.y + tolerance < other.min.y
            || other.max.y + tolerance < self.min.y
            || self.max.z + tolerance < other.min.z
            || other.max.z + tolerance < self.min.z)
    }

    /// Get the center of this bounding box.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the index of the longest axis (0=X, 1=Y, 2=Z).
    #[must_use]
    pub fn longest_axis(&self) -> usize {
        let dx = self.max.x - self.min.x;
        let dy = self.max.y - self.min.y;
        let dz = self.max.z - self.min.z;

        if dx >= dy && dx >= dz {
            0
        } else if dy >= dz {
            1
        } else {
            2
        }
    }

    /// Get the extent (size) along each axis.
    #[must_use]
    pub fn extent(&self) -> [f64; 3] {
        [
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        ]
    }

    /// Get the surface area of this bounding box.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let [dx, dy, dz] = self.extent();
        2.0 * (dx * dy + dy * dz + dz * dx)
    }

    /// Check if this bounding box is valid (non-empty).
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    /// Pad this bounding box by a given amount in all directions.
    #[must_use]
    pub fn padded(&self, padding: f64) -> Self {
        Self {
            min: Point3::new(
                self.min.x - padding,
                self.min.y - padding,
                self.min.z - padding,
            ),
            max: Point3::new(
                self.max.x + padding,
                self.max.y + padding,
                self.max.z + padding,
            ),
        }
    }
}

/// BVH node containing either leaf triangles or child nodes.
#[derive(Debug)]
pub enum BvhNode {
    /// Leaf node containing triangle indices.
    Leaf {
        /// Bounding box of all triangles in this leaf.
        bbox: Aabb,
        /// Triangle indices stored in this leaf.
        triangles: SmallVec<[u32; 8]>,
    },
    /// Internal node with two children.
    Internal {
        /// Bounding box of all triangles in this subtree.
        bbox: Aabb,
        /// Left child node.
        left: Box<Self>,
        /// Right child node.
        right: Box<Self>,
    },
}

impl BvhNode {
    /// Get the bounding box of this node.
    #[must_use]
    pub fn bbox(&self) -> &Aabb {
        match self {
            Self::Leaf { bbox, .. } | Self::Internal { bbox, .. } => bbox,
        }
    }
}

/// Bounding Volume Hierarchy for triangle meshes.
///
/// Provides O(log n) query time for finding triangles that potentially
/// intersect a given bounding box.
#[derive(Debug)]
pub struct Bvh {
    /// Root node of the BVH (None for empty meshes).
    root: Option<BvhNode>,
    /// Total number of triangles in the BVH.
    triangle_count: usize,
}

impl Bvh {
    /// Build a BVH from a mesh.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh to build the BVH for
    /// * `max_leaf_size` - Maximum triangles per leaf node
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex, Point3};
    /// use mesh_boolean::bvh::Bvh;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let bvh = Bvh::build(&mesh, 8);
    /// assert_eq!(bvh.triangle_count(), 1);
    /// ```
    #[must_use]
    pub fn build(mesh: &IndexedMesh, max_leaf_size: usize) -> Self {
        if mesh.faces.is_empty() {
            return Self {
                root: None,
                triangle_count: 0,
            };
        }

        // Build list of triangle bounding boxes
        let triangles: Vec<(u32, Aabb)> = mesh
            .faces
            .iter()
            .enumerate()
            .map(|(i, face)| {
                let v0 = mesh.vertices[face[0] as usize].position;
                let v1 = mesh.vertices[face[1] as usize].position;
                let v2 = mesh.vertices[face[2] as usize].position;
                (i as u32, Aabb::from_triangle(&v0, &v1, &v2))
            })
            .collect();

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let max_leaf = max_leaf_size.max(1);
        let root = Self::build_recursive(&triangles, indices, max_leaf);

        Self {
            root: Some(root),
            triangle_count: mesh.faces.len(),
        }
    }

    /// Build a BVH using parallel construction for large meshes.
    ///
    /// Uses rayon for parallel BVH construction when the mesh has
    /// more than `parallel_threshold` triangles.
    #[must_use]
    pub fn build_parallel(
        mesh: &IndexedMesh,
        max_leaf_size: usize,
        parallel_threshold: usize,
    ) -> Self {
        if mesh.faces.is_empty() {
            return Self {
                root: None,
                triangle_count: 0,
            };
        }

        // Build list of triangle bounding boxes
        let triangles: Vec<(u32, Aabb)> = mesh
            .faces
            .par_iter()
            .enumerate()
            .map(|(i, face)| {
                let v0 = mesh.vertices[face[0] as usize].position;
                let v1 = mesh.vertices[face[1] as usize].position;
                let v2 = mesh.vertices[face[2] as usize].position;
                (i as u32, Aabb::from_triangle(&v0, &v1, &v2))
            })
            .collect();

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let max_leaf = max_leaf_size.max(1);

        let root = if triangles.len() >= parallel_threshold {
            Self::build_recursive_parallel(&triangles, indices, max_leaf, parallel_threshold)
        } else {
            Self::build_recursive(&triangles, indices, max_leaf)
        };

        Self {
            root: Some(root),
            triangle_count: mesh.faces.len(),
        }
    }

    fn build_recursive(
        triangles: &[(u32, Aabb)],
        indices: Vec<usize>,
        max_leaf_size: usize,
    ) -> BvhNode {
        // Compute bounding box of all triangles
        let mut bbox = Aabb::empty();
        for &i in &indices {
            bbox.expand(&triangles[i].1);
        }

        // If few enough triangles, make a leaf
        if indices.len() <= max_leaf_size {
            let triangle_indices: SmallVec<[u32; 8]> =
                indices.iter().map(|&i| triangles[i].0).collect();
            return BvhNode::Leaf {
                bbox,
                triangles: triangle_indices,
            };
        }

        // Split along longest axis using midpoint
        let axis = bbox.longest_axis();
        let mut sorted_indices = indices;
        sorted_indices.sort_by(|&a, &b| {
            let ca = triangles[a].1.center();
            let cb = triangles[b].1.center();
            let va = match axis {
                0 => ca.x,
                1 => ca.y,
                _ => ca.z,
            };
            let vb = match axis {
                0 => cb.x,
                1 => cb.y,
                _ => cb.z,
            };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = sorted_indices.len() / 2;
        let left_indices: Vec<usize> = sorted_indices[..mid].to_vec();
        let right_indices: Vec<usize> = sorted_indices[mid..].to_vec();

        let left = Self::build_recursive(triangles, left_indices, max_leaf_size);
        let right = Self::build_recursive(triangles, right_indices, max_leaf_size);

        BvhNode::Internal {
            bbox,
            left: Box::new(left),
            right: Box::new(right),
        }
    }

    fn build_recursive_parallel(
        triangles: &[(u32, Aabb)],
        indices: Vec<usize>,
        max_leaf_size: usize,
        parallel_threshold: usize,
    ) -> BvhNode {
        // Compute bounding box of all triangles
        let mut bbox = Aabb::empty();
        for &i in &indices {
            bbox.expand(&triangles[i].1);
        }

        // If few enough triangles, make a leaf
        if indices.len() <= max_leaf_size {
            let triangle_indices: SmallVec<[u32; 8]> =
                indices.iter().map(|&i| triangles[i].0).collect();
            return BvhNode::Leaf {
                bbox,
                triangles: triangle_indices,
            };
        }

        // Split along longest axis using midpoint
        let axis = bbox.longest_axis();
        let mut sorted_indices = indices;
        sorted_indices.sort_by(|&a, &b| {
            let ca = triangles[a].1.center();
            let cb = triangles[b].1.center();
            let va = match axis {
                0 => ca.x,
                1 => ca.y,
                _ => ca.z,
            };
            let vb = match axis {
                0 => cb.x,
                1 => cb.y,
                _ => cb.z,
            };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = sorted_indices.len() / 2;
        let left_indices: Vec<usize> = sorted_indices[..mid].to_vec();
        let right_indices: Vec<usize> = sorted_indices[mid..].to_vec();

        // Use parallel construction for large subtrees
        let (left, right) = if left_indices.len() >= parallel_threshold
            || right_indices.len() >= parallel_threshold
        {
            rayon::join(
                || {
                    Self::build_recursive_parallel(
                        triangles,
                        left_indices,
                        max_leaf_size,
                        parallel_threshold,
                    )
                },
                || {
                    Self::build_recursive_parallel(
                        triangles,
                        right_indices,
                        max_leaf_size,
                        parallel_threshold,
                    )
                },
            )
        } else {
            (
                Self::build_recursive(triangles, left_indices, max_leaf_size),
                Self::build_recursive(triangles, right_indices, max_leaf_size),
            )
        };

        BvhNode::Internal {
            bbox,
            left: Box::new(left),
            right: Box::new(right),
        }
    }

    /// Query the BVH for triangles that may intersect a bounding box.
    ///
    /// Returns indices of all triangles whose bounding boxes intersect
    /// the query box (within tolerance).
    ///
    /// # Arguments
    ///
    /// * `query_bbox` - The bounding box to query
    /// * `tolerance` - Distance tolerance for intersection test
    ///
    /// # Returns
    ///
    /// Vector of triangle indices that may intersect the query box.
    #[must_use]
    pub fn query(&self, query_bbox: &Aabb, tolerance: f64) -> Vec<u32> {
        let mut result = Vec::new();
        if let Some(ref root) = self.root {
            Self::query_recursive(root, query_bbox, tolerance, &mut result);
        }
        result
    }

    fn query_recursive(node: &BvhNode, query_bbox: &Aabb, tolerance: f64, result: &mut Vec<u32>) {
        match node {
            BvhNode::Leaf { bbox, triangles } => {
                if bbox.intersects(query_bbox, tolerance) {
                    result.extend(triangles.iter().copied());
                }
            }
            BvhNode::Internal { bbox, left, right } => {
                if bbox.intersects(query_bbox, tolerance) {
                    Self::query_recursive(left, query_bbox, tolerance, result);
                    Self::query_recursive(right, query_bbox, tolerance, result);
                }
            }
        }
    }

    /// Query the BVH for triangles that may intersect a point.
    ///
    /// Returns indices of all triangles whose bounding boxes contain the point
    /// (within tolerance).
    #[must_use]
    pub fn query_point(&self, point: &Point3<f64>, tolerance: f64) -> Vec<u32> {
        let query_bbox = Aabb::from_min_max(*point, *point).padded(tolerance);
        self.query(&query_bbox, 0.0)
    }

    /// Get the total number of triangles in the BVH.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.triangle_count
    }

    /// Check if the BVH is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Get the root bounding box of the BVH.
    #[must_use]
    pub fn root_bbox(&self) -> Option<&Aabb> {
        self.root.as_ref().map(BvhNode::bbox)
    }

    /// Get statistics about the BVH structure.
    #[must_use]
    pub fn stats(&self) -> BvhStats {
        let mut stats = BvhStats::default();
        if let Some(ref root) = self.root {
            Self::collect_stats(root, 0, &mut stats);
        }
        stats
    }

    fn collect_stats(node: &BvhNode, depth: usize, stats: &mut BvhStats) {
        stats.max_depth = stats.max_depth.max(depth);

        match node {
            BvhNode::Leaf { triangles, .. } => {
                stats.leaf_count += 1;
                stats.total_triangles_in_leaves += triangles.len();
                stats.max_leaf_size = stats.max_leaf_size.max(triangles.len());
            }
            BvhNode::Internal { left, right, .. } => {
                stats.internal_count += 1;
                Self::collect_stats(left, depth + 1, stats);
                Self::collect_stats(right, depth + 1, stats);
            }
        }
    }
}

/// Statistics about BVH structure.
#[derive(Debug, Default, Clone)]
pub struct BvhStats {
    /// Number of internal (branch) nodes.
    pub internal_count: usize,
    /// Number of leaf nodes.
    pub leaf_count: usize,
    /// Maximum depth of the tree.
    pub max_depth: usize,
    /// Maximum number of triangles in any leaf.
    pub max_leaf_size: usize,
    /// Total triangles stored across all leaves.
    pub total_triangles_in_leaves: usize,
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_test_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Create a simple box-like mesh with 12 triangles (2 per face)
        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        // Bottom face
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        // Top face
        mesh.faces.push([4, 6, 5]);
        mesh.faces.push([4, 7, 6]);
        // Front face
        mesh.faces.push([0, 5, 1]);
        mesh.faces.push([0, 4, 5]);
        // Back face
        mesh.faces.push([2, 7, 3]);
        mesh.faces.push([2, 6, 7]);
        // Left face
        mesh.faces.push([0, 3, 7]);
        mesh.faces.push([0, 7, 4]);
        // Right face
        mesh.faces.push([1, 5, 6]);
        mesh.faces.push([1, 6, 2]);

        mesh
    }

    #[test]
    fn test_aabb_from_triangle() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.5);

        let bbox = Aabb::from_triangle(&v0, &v1, &v2);

        assert!((bbox.min.x - 0.0).abs() < 1e-10);
        assert!((bbox.min.y - 0.0).abs() < 1e-10);
        assert!((bbox.min.z - 0.0).abs() < 1e-10);
        assert!((bbox.max.x - 1.0).abs() < 1e-10);
        assert!((bbox.max.y - 1.0).abs() < 1e-10);
        assert!((bbox.max.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_aabb_intersects() {
        let a = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));

        let b = Aabb::from_min_max(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));

        let c = Aabb::from_min_max(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));

        assert!(a.intersects(&b, 0.0));
        assert!(!a.intersects(&c, 0.0));
        assert!(a.intersects(&c, 1.5)); // With large tolerance
    }

    #[test]
    fn test_aabb_center() {
        let bbox = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0));

        let center = bbox.center();
        assert!((center.x - 1.0).abs() < 1e-10);
        assert!((center.y - 2.0).abs() < 1e-10);
        assert!((center.z - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_aabb_longest_axis() {
        let bbox_x = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 1.0, 1.0));

        let bbox_y = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 10.0, 1.0));

        let bbox_z = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 10.0));

        assert_eq!(bbox_x.longest_axis(), 0);
        assert_eq!(bbox_y.longest_axis(), 1);
        assert_eq!(bbox_z.longest_axis(), 2);
    }

    #[test]
    fn test_bvh_build_empty() {
        let mesh = IndexedMesh::new();
        let bvh = Bvh::build(&mesh, 8);

        assert!(bvh.is_empty());
        assert_eq!(bvh.triangle_count(), 0);
    }

    #[test]
    fn test_bvh_build_single_triangle() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
        mesh.faces.push([0, 1, 2]);

        let bvh = Bvh::build(&mesh, 8);

        assert!(!bvh.is_empty());
        assert_eq!(bvh.triangle_count(), 1);
    }

    #[test]
    fn test_bvh_build_box() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build(&mesh, 4);

        assert!(!bvh.is_empty());
        assert_eq!(bvh.triangle_count(), 12);

        let stats = bvh.stats();
        assert!(stats.leaf_count > 0);
        assert_eq!(stats.total_triangles_in_leaves, 12);
    }

    #[test]
    fn test_bvh_query_all() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build(&mesh, 4);

        // Query with a large box that contains everything
        let query_bbox =
            Aabb::from_min_max(Point3::new(-1.0, -1.0, -1.0), Point3::new(2.0, 2.0, 2.0));

        let results = bvh.query(&query_bbox, 0.0);
        assert_eq!(results.len(), 12);
    }

    #[test]
    fn test_bvh_query_partial() {
        let mesh = create_test_mesh();
        // Use smaller leaf size for more precise queries
        let bvh = Bvh::build(&mesh, 1);

        // Query with a small box in one corner
        let query_bbox =
            Aabb::from_min_max(Point3::new(-0.1, -0.1, -0.1), Point3::new(0.1, 0.1, 0.1));

        let results = bvh.query(&query_bbox, 0.0);
        // Should find some triangles near the origin (BVH returns conservative candidates)
        assert!(!results.is_empty());
        // With leaf_size=1, we get better spatial separation
        assert!(results.len() <= 12);
    }

    #[test]
    fn test_bvh_query_none() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build(&mesh, 4);

        // Query with a box far from the mesh
        let query_bbox =
            Aabb::from_min_max(Point3::new(10.0, 10.0, 10.0), Point3::new(11.0, 11.0, 11.0));

        let results = bvh.query(&query_bbox, 0.0);
        assert!(results.is_empty());
    }

    #[test]
    fn test_bvh_query_point() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build(&mesh, 4);

        // Query a point inside the box
        let results = bvh.query_point(&Point3::new(0.5, 0.5, 0.5), 0.1);
        assert!(!results.is_empty());

        // Query a point outside the box
        let results = bvh.query_point(&Point3::new(10.0, 10.0, 10.0), 0.1);
        assert!(results.is_empty());
    }

    #[test]
    fn test_bvh_root_bbox() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build(&mesh, 4);

        let root_bbox = bvh.root_bbox().unwrap();
        assert!((root_bbox.min.x - 0.0).abs() < 1e-10);
        assert!((root_bbox.min.y - 0.0).abs() < 1e-10);
        assert!((root_bbox.min.z - 0.0).abs() < 1e-10);
        assert!((root_bbox.max.x - 1.0).abs() < 1e-10);
        assert!((root_bbox.max.y - 1.0).abs() < 1e-10);
        assert!((root_bbox.max.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_bvh_parallel_build() {
        let mesh = create_test_mesh();
        let bvh = Bvh::build_parallel(&mesh, 4, 4);

        assert!(!bvh.is_empty());
        assert_eq!(bvh.triangle_count(), 12);
    }

    #[test]
    fn test_aabb_expand() {
        let mut bbox = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));

        let other = Aabb::from_min_max(Point3::new(-1.0, 0.5, 0.5), Point3::new(0.5, 2.0, 0.5));

        bbox.expand(&other);

        assert!((bbox.min.x - (-1.0)).abs() < 1e-10);
        assert!((bbox.max.y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_aabb_surface_area() {
        let bbox = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0));

        // Surface area = 2 * (1*2 + 2*3 + 3*1) = 2 * (2 + 6 + 3) = 22
        assert!((bbox.surface_area() - 22.0).abs() < 1e-10);
    }

    #[test]
    fn test_aabb_padded() {
        let bbox = Aabb::from_min_max(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));

        let padded = bbox.padded(0.5);

        assert!((padded.min.x - (-0.5)).abs() < 1e-10);
        assert!((padded.max.x - 1.5).abs() < 1e-10);
    }
}
