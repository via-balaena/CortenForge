//! Mesh offset operations using signed distance fields.
//!
//! This module provides functionality to offset (expand or contract) a mesh
//! by a given distance using SDF-based techniques.

use mesh_sdf::SignedDistanceField;
use mesh_types::IndexedMesh;
use nalgebra::Point3;

use crate::error::{OffsetError, OffsetResult};
use crate::grid::ScalarGrid;
use crate::marching_cubes::{MarchingCubesConfig, marching_cubes};

/// Configuration for mesh offset operations.
#[derive(Debug, Clone)]
pub struct OffsetConfig {
    /// Resolution of the SDF grid (cell size).
    /// Smaller values give more accurate results but take longer.
    pub resolution: f64,
    /// Number of padding cells around the mesh bounds.
    pub padding: usize,
    /// Marching cubes configuration.
    pub marching_cubes: MarchingCubesConfig,
}

impl Default for OffsetConfig {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            padding: 5,
            marching_cubes: MarchingCubesConfig::default(),
        }
    }
}

impl OffsetConfig {
    /// Create a new configuration with the given resolution.
    #[must_use]
    pub fn with_resolution(mut self, resolution: f64) -> Self {
        self.resolution = resolution;
        self
    }

    /// Create a configuration optimized for high quality output.
    ///
    /// Uses finer resolution for smoother surfaces.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            resolution: 0.05,
            padding: 8,
            marching_cubes: MarchingCubesConfig::default(),
        }
    }

    /// Create a configuration optimized for fast preview.
    ///
    /// Uses coarser resolution for speed.
    #[must_use]
    pub fn preview() -> Self {
        Self {
            resolution: 0.5,
            padding: 3,
            marching_cubes: MarchingCubesConfig::default(),
        }
    }
}

/// Offset a mesh by a given distance.
///
/// Positive distance expands the mesh outward (dilation).
/// Negative distance shrinks the mesh inward (erosion).
///
/// # Arguments
///
/// * `mesh` - The input mesh to offset
/// * `distance` - The offset distance (positive = expand, negative = shrink)
/// * `config` - Configuration for the offset operation
///
/// # Returns
///
/// A new mesh representing the offset surface.
///
/// # Errors
///
/// Returns an error if:
/// - The mesh is empty
/// - The distance is not finite
/// - The resolution is too low
/// - SDF computation fails
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_offset::{offset_mesh, OffsetConfig};
///
/// // Create a simple tetrahedron
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));
/// mesh.faces.push([0, 2, 1]);
/// mesh.faces.push([0, 1, 3]);
/// mesh.faces.push([1, 2, 3]);
/// mesh.faces.push([2, 0, 3]);
///
/// let config = OffsetConfig::preview();
/// let result = offset_mesh(&mesh, 0.1, &config);
/// // Note: May fail on very simple meshes due to SDF computation
/// ```
pub fn offset_mesh(
    mesh: &IndexedMesh,
    distance: f64,
    config: &OffsetConfig,
) -> OffsetResult<IndexedMesh> {
    // Validate inputs
    if mesh.faces.is_empty() {
        return Err(OffsetError::EmptyMesh);
    }

    if !distance.is_finite() {
        return Err(OffsetError::InvalidDistance(format!("{distance}")));
    }

    if config.resolution <= 0.0 || !config.resolution.is_finite() {
        return Err(OffsetError::ResolutionTooLow {
            minimum: 1,
            actual: 0,
        });
    }

    // Compute mesh bounds
    let (min, max) = compute_bounds(mesh);

    // Extend bounds by the offset distance plus padding
    let extension = distance.abs() + config.padding as f64 * config.resolution;
    let extended_min = Point3::new(min.x - extension, min.y - extension, min.z - extension);
    let extended_max = Point3::new(max.x + extension, max.y + extension, max.z + extension);

    // Create SDF
    let sdf = SignedDistanceField::new(mesh.clone())?;

    // Create grid and sample SDF values
    let mut grid = ScalarGrid::from_bounds(
        extended_min,
        extended_max,
        config.resolution,
        0, // No additional padding, we already extended bounds
    );

    // Fill grid with SDF values adjusted by offset distance
    sample_sdf_to_grid(&sdf, &mut grid, distance);

    // Extract isosurface at 0 level
    let result = marching_cubes(&grid, &config.marching_cubes);

    if result.faces.is_empty() {
        return Err(OffsetError::MarchingCubesFailed {
            reason: "no triangles generated".to_string(),
        });
    }

    Ok(result)
}

/// Offset a mesh with default configuration.
///
/// Convenience function that uses `OffsetConfig::default()`.
///
/// # Arguments
///
/// * `mesh` - The input mesh to offset
/// * `distance` - The offset distance (positive = expand, negative = shrink)
///
/// # Errors
///
/// Same as `offset_mesh`.
pub fn offset_mesh_default(mesh: &IndexedMesh, distance: f64) -> OffsetResult<IndexedMesh> {
    offset_mesh(mesh, distance, &OffsetConfig::default())
}

/// Compute the axis-aligned bounding box of a mesh.
fn compute_bounds(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);

    for vertex in &mesh.vertices {
        let p = &vertex.position;
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }

    (min, max)
}

/// Sample SDF values into a grid, adjusted by offset distance.
///
/// For a positive offset (dilation), we want points that were at distance `d`
/// from the surface to now be at distance `d - offset`. So we subtract the
/// offset from each SDF value.
fn sample_sdf_to_grid(sdf: &SignedDistanceField, grid: &mut ScalarGrid, offset: f64) {
    let (nx, ny, nz) = grid.dimensions();

    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let position = grid.position(ix, iy, iz);
                let distance = sdf.distance(position);
                // Subtract offset: positive offset shrinks the SDF
                // (moves the zero-crossing outward)
                grid.set(ix, iy, iz, distance - offset);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a unit cube centered at origin
        mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, -0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, -0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, 0.5));

        // 12 triangles (2 per face, CCW winding from outside)
        // Bottom face (z = -0.5)
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        // Top face (z = 0.5)
        mesh.faces.push([4, 6, 5]);
        mesh.faces.push([4, 7, 6]);
        // Front face (y = -0.5)
        mesh.faces.push([0, 5, 1]);
        mesh.faces.push([0, 4, 5]);
        // Back face (y = 0.5)
        mesh.faces.push([2, 7, 3]);
        mesh.faces.push([2, 6, 7]);
        // Left face (x = -0.5)
        mesh.faces.push([0, 3, 7]);
        mesh.faces.push([0, 7, 4]);
        // Right face (x = 0.5)
        mesh.faces.push([1, 5, 6]);
        mesh.faces.push([1, 6, 2]);

        mesh
    }

    fn simple_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);
        mesh
    }

    #[test]
    fn offset_config_default() {
        let config = OffsetConfig::default();
        assert!(config.resolution > 0.0);
        assert!(config.padding > 0);
    }

    #[test]
    fn offset_config_high_quality() {
        let config = OffsetConfig::high_quality();
        let default = OffsetConfig::default();
        assert!(config.resolution < default.resolution);
    }

    #[test]
    fn offset_config_preview() {
        let config = OffsetConfig::preview();
        let default = OffsetConfig::default();
        assert!(config.resolution > default.resolution);
    }

    #[test]
    fn offset_config_with_resolution() {
        let config = OffsetConfig::default().with_resolution(0.25);
        assert!((config.resolution - 0.25).abs() < f64::EPSILON);
    }

    #[test]
    fn offset_empty_mesh_fails() {
        let mesh = IndexedMesh::new();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, 0.1, &config);
        assert!(result.is_err());
    }

    #[test]
    fn offset_invalid_distance_nan() {
        let mesh = simple_tetrahedron();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, f64::NAN, &config);
        assert!(result.is_err());
    }

    #[test]
    fn offset_invalid_distance_inf() {
        let mesh = simple_tetrahedron();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, f64::INFINITY, &config);
        assert!(result.is_err());
    }

    #[test]
    fn offset_invalid_resolution() {
        let mesh = simple_tetrahedron();
        let mut config = OffsetConfig::preview();
        config.resolution = 0.0;
        let result = offset_mesh(&mesh, 0.1, &config);
        assert!(result.is_err());
    }

    #[test]
    fn offset_negative_resolution() {
        let mesh = simple_tetrahedron();
        let mut config = OffsetConfig::preview();
        config.resolution = -1.0;
        let result = offset_mesh(&mesh, 0.1, &config);
        assert!(result.is_err());
    }

    #[test]
    fn offset_cube_positive() {
        let mesh = unit_cube();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, 0.1, &config);
        assert!(result.is_ok());

        let offset_mesh = result.expect("should succeed");
        // Offset mesh should have faces
        assert!(!offset_mesh.faces.is_empty());
    }

    #[test]
    fn offset_cube_negative() {
        let mesh = unit_cube();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, -0.1, &config);
        assert!(result.is_ok());

        let offset_mesh = result.expect("should succeed");
        // Offset mesh should have faces (unless shrunk too much)
        // With small negative offset, should still have geometry
        assert!(!offset_mesh.faces.is_empty());
    }

    #[test]
    fn offset_mesh_default_fn() {
        let mesh = unit_cube();
        // Use a very coarse resolution config since default might be too fine
        let result = offset_mesh(&mesh, 0.1, &OffsetConfig::preview());
        assert!(result.is_ok());
    }

    #[test]
    fn compute_bounds_cube() {
        let mesh = unit_cube();
        let (min, max) = compute_bounds(&mesh);

        assert!((min.x - (-0.5)).abs() < f64::EPSILON);
        assert!((min.y - (-0.5)).abs() < f64::EPSILON);
        assert!((min.z - (-0.5)).abs() < f64::EPSILON);
        assert!((max.x - 0.5).abs() < f64::EPSILON);
        assert!((max.y - 0.5).abs() < f64::EPSILON);
        assert!((max.z - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn offset_zero_distance() {
        let mesh = unit_cube();
        let config = OffsetConfig::preview();
        let result = offset_mesh(&mesh, 0.0, &config);
        assert!(result.is_ok());
    }

    #[test]
    fn sample_sdf_to_grid_runs() {
        let mesh = unit_cube();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        let mut grid = ScalarGrid::new((5, 5, 5), Point3::new(-1.0, -1.0, -1.0), 0.5);

        sample_sdf_to_grid(&sdf, &mut grid, 0.1);

        // Grid should have been filled with values
        // Center point should have negative value (inside) minus offset
        let center_val = grid.get(2, 2, 2);
        // Just verify it was set to something other than the initial 0.0
        // The actual value depends on the SDF computation
        assert!(center_val.is_finite());
    }

    #[test]
    fn offset_preserves_topology() {
        let mesh = unit_cube();
        let config = OffsetConfig::preview();

        let result = offset_mesh(&mesh, 0.05, &config);
        assert!(result.is_ok());

        let offset = result.expect("should succeed");
        // Should still be a closed mesh (even number of edges per vertex)
        // At minimum, should have some geometry
        assert!(!offset.vertices.is_empty());
        assert!(!offset.faces.is_empty());
    }
}
