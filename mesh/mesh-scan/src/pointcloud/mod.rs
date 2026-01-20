//! Point cloud data structures and operations.
//!
//! This module provides the [`PointCloud`] type for working with 3D point data
//! from scanners, depth cameras, and other sources.
//!
//! # Example
//!
//! ```
//! use mesh_scan::pointcloud::{PointCloud, CloudPoint};
//! use nalgebra::Point3;
//!
//! // Create a point cloud from positions
//! let positions = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 0.0, 0.0),
//!     Point3::new(0.0, 1.0, 0.0),
//! ];
//! let cloud = PointCloud::from_positions(&positions);
//!
//! assert_eq!(cloud.len(), 3);
//! assert!(!cloud.is_empty());
//! ```

pub mod io;
pub mod normals;

use mesh_types::{Aabb, IndexedMesh, Vertex, VertexColor};
use nalgebra::{Point3, Vector3};

use crate::error::ScanResult;
use crate::reconstruct::{ReconstructionParams, ReconstructionResult};

/// A point in a point cloud with optional attributes.
///
/// Each point has a 3D position and optional attributes like normal, color,
/// intensity, and confidence values from the scanner.
///
/// # Example
///
/// ```
/// use mesh_scan::pointcloud::CloudPoint;
/// use nalgebra::{Point3, Vector3};
///
/// // Simple point with just position
/// let p1 = CloudPoint::new(Point3::new(1.0, 2.0, 3.0));
///
/// // Point with position and normal
/// let p2 = CloudPoint::with_normal(
///     Point3::new(1.0, 2.0, 3.0),
///     Vector3::new(0.0, 0.0, 1.0),
/// );
///
/// assert!(p1.normal.is_none());
/// assert!(p2.normal.is_some());
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct CloudPoint {
    /// The 3D position of the point.
    pub position: Point3<f64>,

    /// Optional unit normal vector at this point.
    pub normal: Option<Vector3<f64>>,

    /// Optional RGB color.
    pub color: Option<VertexColor>,

    /// Optional intensity value (scanner-specific, typically 0.0-1.0).
    pub intensity: Option<f32>,

    /// Optional confidence value (0.0-1.0, higher = more reliable).
    pub confidence: Option<f32>,
}

impl CloudPoint {
    /// Creates a new point with just a position.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::CloudPoint;
    /// use nalgebra::Point3;
    ///
    /// let point = CloudPoint::new(Point3::new(1.0, 2.0, 3.0));
    /// assert_eq!(point.position.x, 1.0);
    /// ```
    #[must_use]
    pub const fn new(position: Point3<f64>) -> Self {
        Self {
            position,
            normal: None,
            color: None,
            intensity: None,
            confidence: None,
        }
    }

    /// Creates a point from x, y, z coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::CloudPoint;
    ///
    /// let point = CloudPoint::from_coords(1.0, 2.0, 3.0);
    /// assert_eq!(point.position.y, 2.0);
    /// ```
    #[must_use]
    pub const fn from_coords(x: f64, y: f64, z: f64) -> Self {
        Self::new(Point3::new(x, y, z))
    }

    /// Creates a point with position and normal.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::CloudPoint;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let point = CloudPoint::with_normal(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Vector3::new(0.0, 1.0, 0.0),
    /// );
    /// assert!(point.normal.is_some());
    /// ```
    #[must_use]
    pub const fn with_normal(position: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            position,
            normal: Some(normal),
            color: None,
            intensity: None,
            confidence: None,
        }
    }

    /// Creates a point with position, normal, and color.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::CloudPoint;
    /// use mesh_types::VertexColor;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let point = CloudPoint::with_normal_and_color(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Vector3::new(0.0, 1.0, 0.0),
    ///     VertexColor::new(255, 128, 64),
    /// );
    /// assert!(point.color.is_some());
    /// ```
    #[must_use]
    pub const fn with_normal_and_color(
        position: Point3<f64>,
        normal: Vector3<f64>,
        color: VertexColor,
    ) -> Self {
        Self {
            position,
            normal: Some(normal),
            color: Some(color),
            intensity: None,
            confidence: None,
        }
    }

    /// Converts this cloud point to a mesh vertex.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::CloudPoint;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let point = CloudPoint::with_normal(
    ///     Point3::new(1.0, 2.0, 3.0),
    ///     Vector3::new(0.0, 0.0, 1.0),
    /// );
    /// let vertex = point.to_vertex();
    /// assert_eq!(vertex.position.x, 1.0);
    /// ```
    #[must_use]
    pub const fn to_vertex(&self) -> Vertex {
        let mut vertex = Vertex::new(self.position);
        if let Some(normal) = self.normal {
            vertex.attributes.normal = Some(normal);
        }
        if let Some(color) = self.color {
            vertex.attributes.color = Some(color);
        }
        vertex
    }

    /// Returns true if this point has a normal.
    #[must_use]
    pub const fn has_normal(&self) -> bool {
        self.normal.is_some()
    }

    /// Returns true if this point has a color.
    #[must_use]
    pub const fn has_color(&self) -> bool {
        self.color.is_some()
    }
}

impl Default for CloudPoint {
    fn default() -> Self {
        Self::new(Point3::origin())
    }
}

/// A collection of 3D points with optional attributes.
///
/// Point clouds are commonly used to represent raw scan data before
/// surface reconstruction. This type provides methods for loading,
/// saving, processing, and converting point clouds.
///
/// # Example
///
/// ```
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::Point3;
///
/// let mut cloud = PointCloud::new();
/// cloud.push_coords(0.0, 0.0, 0.0);
/// cloud.push_coords(1.0, 0.0, 0.0);
/// cloud.push_coords(0.0, 1.0, 0.0);
///
/// assert_eq!(cloud.len(), 3);
/// ```
#[derive(Debug, Clone, Default)]
pub struct PointCloud {
    /// The points in this cloud.
    pub points: Vec<CloudPoint>,
}

impl PointCloud {
    /// Creates an empty point cloud.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    ///
    /// let cloud = PointCloud::new();
    /// assert!(cloud.is_empty());
    /// ```
    #[must_use]
    pub const fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Creates a point cloud with pre-allocated capacity.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    ///
    /// let cloud = PointCloud::with_capacity(1000);
    /// assert!(cloud.is_empty());
    /// ```
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
        }
    }

    /// Creates a point cloud from a slice of 3D positions.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let positions = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    /// ];
    /// let cloud = PointCloud::from_positions(&positions);
    /// assert_eq!(cloud.len(), 2);
    /// ```
    #[must_use]
    pub fn from_positions(positions: &[Point3<f64>]) -> Self {
        let points = positions.iter().map(|p| CloudPoint::new(*p)).collect();
        Self { points }
    }

    /// Creates a point cloud from mesh vertices.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use mesh_types::{IndexedMesh, Vertex};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    ///
    /// let cloud = PointCloud::from_mesh(&mesh);
    /// assert_eq!(cloud.len(), 2);
    /// ```
    #[must_use]
    pub fn from_mesh(mesh: &IndexedMesh) -> Self {
        let points = mesh
            .vertices
            .iter()
            .map(|v| {
                let mut point = CloudPoint::new(v.position);
                point.normal = v.attributes.normal;
                point.color = v.attributes.color;
                point
            })
            .collect();
        Self { points }
    }

    /// Returns the number of points in the cloud.
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Returns true if the cloud has no points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Returns true if all points have normals.
    #[must_use]
    pub fn has_normals(&self) -> bool {
        !self.points.is_empty() && self.points.iter().all(CloudPoint::has_normal)
    }

    /// Returns true if all points have colors.
    #[must_use]
    pub fn has_colors(&self) -> bool {
        !self.points.is_empty() && self.points.iter().all(CloudPoint::has_color)
    }

    /// Adds a point to the cloud.
    pub fn push(&mut self, point: CloudPoint) {
        self.points.push(point);
    }

    /// Adds a point with the given coordinates.
    pub fn push_coords(&mut self, x: f64, y: f64, z: f64) {
        self.points.push(CloudPoint::from_coords(x, y, z));
    }

    /// Adds a point at the given position.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let mut cloud = PointCloud::new();
    /// cloud.add_point(Point3::new(1.0, 2.0, 3.0));
    /// assert_eq!(cloud.len(), 1);
    /// ```
    pub fn add_point(&mut self, position: Point3<f64>) {
        self.points.push(CloudPoint::new(position));
    }

    /// Adds a point with position and normal.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut cloud = PointCloud::new();
    /// cloud.add_point_with_normal(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Vector3::new(0.0, 0.0, 1.0),
    /// );
    /// assert!(cloud.has_normals());
    /// ```
    pub fn add_point_with_normal(&mut self, position: Point3<f64>, normal: Vector3<f64>) {
        self.points.push(CloudPoint::with_normal(position, normal));
    }

    /// Returns the axis-aligned bounding box of the point cloud.
    ///
    /// Returns `None` if the cloud is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let positions = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(2.0, 3.0, 1.0),
    /// ];
    /// let cloud = PointCloud::from_positions(&positions);
    /// let bounds = cloud.bounds().unwrap();
    ///
    /// assert_eq!(bounds.min.x, 0.0);
    /// assert_eq!(bounds.max.x, 2.0);
    /// ```
    #[must_use]
    pub fn bounds(&self) -> Option<Aabb> {
        if self.points.is_empty() {
            return None;
        }

        let first = &self.points[0].position;
        let mut min = *first;
        let mut max = *first;

        for point in &self.points[1..] {
            let p = &point.position;
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        Some(Aabb::new(min, max))
    }

    /// Returns the centroid (center of mass) of the point cloud.
    ///
    /// Returns `None` if the cloud is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let positions = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    /// ];
    /// let cloud = PointCloud::from_positions(&positions);
    /// let centroid = cloud.centroid().unwrap();
    ///
    /// assert!((centroid.x - 1.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn centroid(&self) -> Option<Point3<f64>> {
        if self.points.is_empty() {
            return None;
        }

        let sum: Vector3<f64> = self.points.iter().map(|p| p.position.coords).sum();

        #[allow(clippy::cast_precision_loss)]
        let centroid = sum / self.points.len() as f64;

        Some(Point3::from(centroid))
    }

    /// Translates all points by the given offset.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut cloud = PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0)]);
    /// cloud.translate(Vector3::new(1.0, 2.0, 3.0));
    ///
    /// assert!((cloud.points[0].position.x - 1.0).abs() < 1e-10);
    /// ```
    pub fn translate(&mut self, offset: Vector3<f64>) {
        for point in &mut self.points {
            point.position += offset;
        }
    }

    /// Scales all points uniformly about the origin.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let mut cloud = PointCloud::from_positions(&[Point3::new(1.0, 2.0, 3.0)]);
    /// cloud.scale(2.0);
    ///
    /// assert!((cloud.points[0].position.x - 2.0).abs() < 1e-10);
    /// ```
    pub fn scale(&mut self, factor: f64) {
        for point in &mut self.points {
            point.position.coords *= factor;
        }
    }

    /// Scales all points uniformly about the centroid.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let mut cloud = PointCloud::from_positions(&[
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    /// ]);
    /// cloud.scale_centered(2.0);
    ///
    /// // Centroid was at (1,0,0), points scale relative to that
    /// assert!((cloud.points[0].position.x - (-1.0)).abs() < 1e-10);
    /// assert!((cloud.points[1].position.x - 3.0).abs() < 1e-10);
    /// ```
    pub fn scale_centered(&mut self, factor: f64) {
        if let Some(center) = self.centroid() {
            let offset = center.coords;
            for point in &mut self.points {
                point.position.coords = (point.position.coords - offset) * factor + offset;
            }
        }
    }

    /// Applies a rigid transform to all points.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use mesh_registration::RigidTransform;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut cloud = PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0)]);
    /// let transform = RigidTransform::from_translation(Vector3::new(5.0, 0.0, 0.0));
    /// cloud.transform(&transform);
    ///
    /// assert!((cloud.points[0].position.x - 5.0).abs() < 1e-10);
    /// ```
    pub fn transform(&mut self, transform: &mesh_registration::RigidTransform) {
        for point in &mut self.points {
            point.position = transform.transform_point(&point.position);
            if let Some(normal) = &mut point.normal {
                *normal = transform.rotation * *normal;
            }
        }
    }

    /// Downsamples the point cloud using voxel grid filtering.
    ///
    /// Points are grouped into voxels of the given size, and each voxel
    /// is represented by a single point (the centroid of points in that voxel).
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel in world units.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let positions: Vec<_> = (0..100)
    ///     .map(|i| Point3::new(i as f64 * 0.01, 0.0, 0.0))
    ///     .collect();
    /// let cloud = PointCloud::from_positions(&positions);
    ///
    /// let downsampled = cloud.downsample(0.1);
    /// assert!(downsampled.len() < cloud.len());
    /// ```
    #[must_use]
    pub fn downsample(&self, voxel_size: f64) -> Self {
        if self.points.is_empty() || voxel_size <= 0.0 {
            return self.clone();
        }

        use std::collections::HashMap;

        // Map voxel indices to accumulated point data
        let mut voxels: HashMap<(i64, i64, i64), (Vector3<f64>, usize)> = HashMap::new();

        for point in &self.points {
            let vx = (point.position.x / voxel_size).floor() as i64;
            let vy = (point.position.y / voxel_size).floor() as i64;
            let vz = (point.position.z / voxel_size).floor() as i64;

            let entry = voxels.entry((vx, vy, vz)).or_insert((Vector3::zeros(), 0));
            entry.0 += point.position.coords;
            entry.1 += 1;
        }

        let points = voxels
            .into_iter()
            .map(|(_, (sum, count))| {
                #[allow(clippy::cast_precision_loss)]
                let centroid = sum / count as f64;
                CloudPoint::new(Point3::from(centroid))
            })
            .collect();

        Self { points }
    }

    /// Reconstructs a mesh from this point cloud.
    ///
    /// # Errors
    ///
    /// Returns an error if the point cloud is empty or reconstruction fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_scan::pointcloud::PointCloud;
    /// use mesh_scan::reconstruct::ReconstructionParams;
    ///
    /// let cloud = PointCloud::new(); // Would load from file in practice
    /// let params = ReconstructionParams::default();
    /// let result = cloud.to_mesh(&params);
    /// ```
    pub fn to_mesh(&self, params: &ReconstructionParams) -> ScanResult<ReconstructionResult> {
        crate::reconstruct::reconstruct_surface(self, params)
    }
}

impl FromIterator<CloudPoint> for PointCloud {
    fn from_iter<I: IntoIterator<Item = CloudPoint>>(iter: I) -> Self {
        Self {
            points: iter.into_iter().collect(),
        }
    }
}

impl FromIterator<Point3<f64>> for PointCloud {
    fn from_iter<I: IntoIterator<Item = Point3<f64>>>(iter: I) -> Self {
        Self {
            points: iter.into_iter().map(CloudPoint::new).collect(),
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_cloud_point_new() {
        let point = CloudPoint::new(Point3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(point.position.x, 1.0);
        assert_relative_eq!(point.position.y, 2.0);
        assert_relative_eq!(point.position.z, 3.0);
        assert!(point.normal.is_none());
        assert!(point.color.is_none());
    }

    #[test]
    fn test_cloud_point_from_coords() {
        let point = CloudPoint::from_coords(1.0, 2.0, 3.0);
        assert_relative_eq!(point.position.x, 1.0);
    }

    #[test]
    fn test_cloud_point_with_normal() {
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let point = CloudPoint::with_normal(Point3::origin(), normal);
        assert!(point.has_normal());
        assert_relative_eq!(point.normal.unwrap().z, 1.0);
    }

    #[test]
    fn test_cloud_point_with_normal_and_color() {
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let color = VertexColor::new(255, 128, 64);
        let point = CloudPoint::with_normal_and_color(Point3::origin(), normal, color);
        assert!(point.has_normal());
        assert!(point.has_color());
        assert_eq!(point.color.unwrap().r, 255);
    }

    #[test]
    fn test_cloud_point_to_vertex() {
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let point = CloudPoint::with_normal(Point3::new(1.0, 2.0, 3.0), normal);
        let vertex = point.to_vertex();
        assert_relative_eq!(vertex.position.x, 1.0);
        assert!(vertex.attributes.normal.is_some());
    }

    #[test]
    fn test_point_cloud_new() {
        let cloud = PointCloud::new();
        assert!(cloud.is_empty());
        assert_eq!(cloud.len(), 0);
    }

    #[test]
    fn test_point_cloud_with_capacity() {
        let cloud = PointCloud::with_capacity(100);
        assert!(cloud.is_empty());
    }

    #[test]
    fn test_point_cloud_from_positions() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_positions(&positions);
        assert_eq!(cloud.len(), 3);
        assert!(!cloud.has_normals());
    }

    #[test]
    fn test_point_cloud_from_mesh() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));

        let cloud = PointCloud::from_mesh(&mesh);
        assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn test_point_cloud_push() {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::from_coords(1.0, 2.0, 3.0));
        assert_eq!(cloud.len(), 1);
    }

    #[test]
    fn test_point_cloud_push_coords() {
        let mut cloud = PointCloud::new();
        cloud.push_coords(1.0, 2.0, 3.0);
        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.points[0].position.x, 1.0);
    }

    #[test]
    fn test_point_cloud_bounds() {
        let positions = vec![Point3::new(0.0, 1.0, 2.0), Point3::new(3.0, 4.0, 5.0)];
        let cloud = PointCloud::from_positions(&positions);
        let bounds = cloud.bounds().unwrap();

        assert_relative_eq!(bounds.min.x, 0.0);
        assert_relative_eq!(bounds.min.y, 1.0);
        assert_relative_eq!(bounds.min.z, 2.0);
        assert_relative_eq!(bounds.max.x, 3.0);
        assert_relative_eq!(bounds.max.y, 4.0);
        assert_relative_eq!(bounds.max.z, 5.0);
    }

    #[test]
    fn test_point_cloud_bounds_empty() {
        let cloud = PointCloud::new();
        assert!(cloud.bounds().is_none());
    }

    #[test]
    fn test_point_cloud_centroid() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0)];
        let cloud = PointCloud::from_positions(&positions);
        let centroid = cloud.centroid().unwrap();

        assert_relative_eq!(centroid.x, 1.0);
        assert_relative_eq!(centroid.y, 2.0);
        assert_relative_eq!(centroid.z, 3.0);
    }

    #[test]
    fn test_point_cloud_centroid_empty() {
        let cloud = PointCloud::new();
        assert!(cloud.centroid().is_none());
    }

    #[test]
    fn test_point_cloud_translate() {
        let mut cloud = PointCloud::from_positions(&[Point3::new(1.0, 2.0, 3.0)]);
        cloud.translate(Vector3::new(10.0, 20.0, 30.0));

        assert_relative_eq!(cloud.points[0].position.x, 11.0);
        assert_relative_eq!(cloud.points[0].position.y, 22.0);
        assert_relative_eq!(cloud.points[0].position.z, 33.0);
    }

    #[test]
    fn test_point_cloud_scale() {
        let mut cloud = PointCloud::from_positions(&[Point3::new(1.0, 2.0, 3.0)]);
        cloud.scale(2.0);

        assert_relative_eq!(cloud.points[0].position.x, 2.0);
        assert_relative_eq!(cloud.points[0].position.y, 4.0);
        assert_relative_eq!(cloud.points[0].position.z, 6.0);
    }

    #[test]
    fn test_point_cloud_scale_centered() {
        let mut cloud =
            PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]);
        cloud.scale_centered(2.0);

        // Centroid is at (1, 0, 0)
        // Point at 0 -> -1 (distance from center * scale)
        // Point at 2 -> 3
        assert_relative_eq!(cloud.points[0].position.x, -1.0);
        assert_relative_eq!(cloud.points[1].position.x, 3.0);
    }

    #[test]
    fn test_point_cloud_transform() {
        use mesh_registration::RigidTransform;

        let mut cloud = PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0)]);
        let transform = RigidTransform::from_translation(Vector3::new(5.0, 10.0, 15.0));
        cloud.transform(&transform);

        assert_relative_eq!(cloud.points[0].position.x, 5.0);
        assert_relative_eq!(cloud.points[0].position.y, 10.0);
        assert_relative_eq!(cloud.points[0].position.z, 15.0);
    }

    #[test]
    fn test_point_cloud_downsample() {
        // Create 100 points along a line
        let positions: Vec<_> = (0..100)
            .map(|i| Point3::new(f64::from(i) * 0.01, 0.0, 0.0))
            .collect();
        let cloud = PointCloud::from_positions(&positions);

        // Downsample with voxel size 0.1 (should group ~10 points per voxel)
        let downsampled = cloud.downsample(0.1);

        // Should have significantly fewer points
        assert!(downsampled.len() < cloud.len());
        assert!(!downsampled.is_empty());
    }

    #[test]
    fn test_point_cloud_downsample_empty() {
        let cloud = PointCloud::new();
        let downsampled = cloud.downsample(0.1);
        assert!(downsampled.is_empty());
    }

    #[test]
    fn test_point_cloud_downsample_invalid_voxel_size() {
        let cloud = PointCloud::from_positions(&[Point3::origin()]);
        let downsampled = cloud.downsample(-1.0);
        assert_eq!(downsampled.len(), cloud.len());
    }

    #[test]
    fn test_point_cloud_has_normals() {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::with_normal(Point3::origin(), Vector3::z()));
        assert!(cloud.has_normals());

        cloud.push(CloudPoint::new(Point3::origin()));
        assert!(!cloud.has_normals());
    }

    #[test]
    fn test_point_cloud_has_colors() {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::with_normal_and_color(
            Point3::origin(),
            Vector3::z(),
            VertexColor::new(255, 255, 255),
        ));
        assert!(cloud.has_colors());

        cloud.push(CloudPoint::new(Point3::origin()));
        assert!(!cloud.has_colors());
    }

    #[test]
    fn test_point_cloud_from_iterator_points() {
        let points = vec![
            CloudPoint::from_coords(0.0, 0.0, 0.0),
            CloudPoint::from_coords(1.0, 1.0, 1.0),
        ];
        let cloud: PointCloud = points.into_iter().collect();
        assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn test_point_cloud_from_iterator_positions() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)];
        let cloud: PointCloud = positions.into_iter().collect();
        assert_eq!(cloud.len(), 2);
    }
}
