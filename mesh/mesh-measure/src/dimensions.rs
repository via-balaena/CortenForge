//! Mesh dimension extraction.
//!
//! Provides axis-aligned bounding box dimensions and basic mesh statistics.

use mesh_types::{IndexedMesh, MeshBounds, Point3};

/// Result of dimension extraction.
///
/// Contains bounding box information and derived measurements.
///
/// # Example
///
/// ```
/// use mesh_types::{unit_cube, IndexedMesh};
/// use mesh_measure::dimensions;
///
/// let cube = unit_cube();
/// let dims = dimensions(&cube);
///
/// assert!((dims.width - 1.0).abs() < 1e-10);
/// assert!((dims.depth - 1.0).abs() < 1e-10);
/// assert!((dims.height - 1.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone)]
pub struct Dimensions {
    /// Bounding box minimum point.
    pub min: Point3<f64>,
    /// Bounding box maximum point.
    pub max: Point3<f64>,
    /// Width (X dimension).
    pub width: f64,
    /// Depth (Y dimension).
    pub depth: f64,
    /// Height (Z dimension).
    pub height: f64,
    /// Diagonal length of bounding box.
    pub diagonal: f64,
    /// Volume of bounding box.
    pub bounding_volume: f64,
    /// Center of bounding box.
    pub center: Point3<f64>,
}

impl Default for Dimensions {
    fn default() -> Self {
        Self {
            min: Point3::origin(),
            max: Point3::origin(),
            width: 0.0,
            depth: 0.0,
            height: 0.0,
            diagonal: 0.0,
            bounding_volume: 0.0,
            center: Point3::origin(),
        }
    }
}

impl Dimensions {
    /// Get the shortest dimension.
    #[must_use]
    pub const fn min_extent(&self) -> f64 {
        self.width.min(self.depth).min(self.height)
    }

    /// Get the longest dimension.
    #[must_use]
    pub const fn max_extent(&self) -> f64 {
        self.width.max(self.depth).max(self.height)
    }

    /// Get the aspect ratio (max / min dimension).
    ///
    /// Returns `f64::INFINITY` if the minimum extent is zero.
    #[must_use]
    pub fn aspect_ratio(&self) -> f64 {
        let min = self.min_extent();
        if min.abs() < f64::EPSILON {
            f64::INFINITY
        } else {
            self.max_extent() / min
        }
    }

    /// Check if the mesh is approximately cubic (aspect ratio near 1).
    #[must_use]
    pub fn is_cubic(&self, tolerance: f64) -> bool {
        let ratio = self.aspect_ratio();
        (ratio - 1.0).abs() < tolerance
    }

    /// Get the size as a vector.
    #[must_use]
    pub const fn size(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(self.width, self.depth, self.height)
    }
}

/// Extract dimensions of a mesh.
///
/// Computes the axis-aligned bounding box and derived measurements.
///
/// # Arguments
///
/// * `mesh` - The mesh to measure
///
/// # Returns
///
/// A [`Dimensions`] struct with all measurements.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
/// use mesh_measure::dimensions;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 5.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let dims = dimensions(&mesh);
/// assert!((dims.width - 10.0).abs() < 1e-10);
/// assert!((dims.depth - 5.0).abs() < 1e-10);
/// ```
#[must_use]
pub fn dimensions(mesh: &IndexedMesh) -> Dimensions {
    if mesh.vertices.is_empty() {
        return Dimensions::default();
    }

    let bounds = mesh.bounds();
    let min = bounds.min;
    let max = bounds.max;

    let width = max.x - min.x;
    let depth = max.y - min.y;
    let height = max.z - min.z;

    Dimensions {
        min,
        max,
        width,
        depth,
        height,
        diagonal: height.mul_add(height, width.mul_add(width, depth * depth)).sqrt(),
        bounding_volume: width * depth * height,
        center: Point3::new(
            f64::midpoint(min.x, max.x),
            f64::midpoint(min.y, max.y),
            f64::midpoint(min.z, max.z),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{unit_cube, Vertex};

    fn create_test_cube(size: f64) -> IndexedMesh {
        let mut cube = unit_cube();
        cube.scale(size);
        cube
    }

    #[test]
    fn test_unit_cube_dimensions() {
        let cube = unit_cube();
        let dims = dimensions(&cube);

        assert!((dims.width - 1.0).abs() < 1e-10);
        assert!((dims.depth - 1.0).abs() < 1e-10);
        assert!((dims.height - 1.0).abs() < 1e-10);
        assert!((dims.bounding_volume - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_scaled_cube_dimensions() {
        let cube = create_test_cube(10.0);
        let dims = dimensions(&cube);

        assert!((dims.width - 10.0).abs() < 1e-10);
        assert!((dims.depth - 10.0).abs() < 1e-10);
        assert!((dims.height - 10.0).abs() < 1e-10);
        assert!((dims.bounding_volume - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_diagonal() {
        let cube = unit_cube();
        let dims = dimensions(&cube);

        let expected_diagonal = (3.0_f64).sqrt();
        assert!((dims.diagonal - expected_diagonal).abs() < 1e-10);
    }

    #[test]
    fn test_center() {
        let cube = unit_cube();
        let dims = dimensions(&cube);

        assert!((dims.center.x - 0.5).abs() < 1e-10);
        assert!((dims.center.y - 0.5).abs() < 1e-10);
        assert!((dims.center.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_empty_mesh_dimensions() {
        let mesh = IndexedMesh::new();
        let dims = dimensions(&mesh);

        assert!((dims.width).abs() < f64::EPSILON);
        assert!((dims.depth).abs() < f64::EPSILON);
        assert!((dims.height).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aspect_ratio() {
        let cube = unit_cube();
        let dims = dimensions(&cube);
        assert!((dims.aspect_ratio() - 1.0).abs() < 1e-10);
        assert!(dims.is_cubic(0.01));
    }

    #[test]
    fn test_non_cubic_mesh() {
        // Create a 10x2x1 box shape with vertices
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 2.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 2.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 2.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 2.0, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);

        let dims = dimensions(&mesh);
        // 10 / 1 = 10 (max/min)
        assert!((dims.aspect_ratio() - 10.0).abs() < 1e-10);
        assert!(!dims.is_cubic(0.5));
    }

    #[test]
    fn test_min_max_extent() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 5.0, 2.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 5.0, 2.0));
        mesh.faces.push([0, 1, 2]);

        let dims = dimensions(&mesh);
        assert!((dims.min_extent() - 2.0).abs() < 1e-10);
        assert!((dims.max_extent() - 10.0).abs() < 1e-10);
    }
}
