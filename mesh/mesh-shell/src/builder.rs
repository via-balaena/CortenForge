//! Fluent builder API for shell generation.
//!
//! This module provides an ergonomic builder pattern for configuring and
//! executing shell generation operations.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_shell::ShellBuilder;
//!
//! // Create a simple mesh (in practice, load from file)
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Generate shell with fluent API
//! let result = ShellBuilder::new(&mesh)
//!     .wall_thickness(2.0)
//!     .fast()
//!     .build();
//! ```

use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_types::IndexedMesh;

use crate::error::{ShellError, ShellResult};
use crate::shell::{ShellGenerationResult, ShellParams, WallGenerationMethod, generate_shell};

/// Result from `ShellBuilder` containing the generated mesh and statistics.
#[derive(Debug)]
pub struct ShellBuildResult {
    /// The generated shell mesh.
    pub mesh: IndexedMesh,
    /// Statistics from offset operation (if using offset).
    pub offset_applied: bool,
    /// Statistics from shell generation.
    pub shell_stats: ShellGenerationResult,
}

/// Fluent builder for shell generation.
///
/// `ShellBuilder` provides a chainable API for configuring shell generation
/// parameters before executing the operation.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_shell::ShellBuilder;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// // Simple usage with defaults
/// let result = ShellBuilder::new(&mesh)
///     .wall_thickness(2.0)
///     .build();
///
/// // Advanced usage with custom settings
/// let result = ShellBuilder::new(&mesh)
///     .offset(1.0)
///     .wall_thickness(2.0)
///     .voxel_size(0.5)
///     .high_quality()
///     .build();
/// ```
pub struct ShellBuilder<'a> {
    mesh: &'a IndexedMesh,
    // Offset parameters
    offset_mm: Option<f64>,
    offset_voxel_size_mm: f64,
    // Shell parameters
    wall_thickness_mm: f64,
    min_thickness_mm: f64,
    validate: bool,
    wall_method: WallGenerationMethod,
    sdf_voxel_size_mm: f64,
}

impl<'a> ShellBuilder<'a> {
    /// Create a new `ShellBuilder` for the given mesh.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The input mesh to generate a shell around
    #[must_use]
    pub const fn new(mesh: &'a IndexedMesh) -> Self {
        Self {
            mesh,
            // Offset disabled by default
            offset_mm: None,
            offset_voxel_size_mm: 0.75,
            // Sensible defaults for shell generation
            wall_thickness_mm: 2.5,
            min_thickness_mm: 1.5,
            validate: true,
            wall_method: WallGenerationMethod::Normal,
            sdf_voxel_size_mm: 0.5,
        }
    }

    // =========================================================================
    // Offset Configuration
    // =========================================================================

    /// Set the offset distance in mm.
    ///
    /// This applies an initial SDF-based offset to the mesh before generating
    /// the shell. Positive values expand outward, negative values shrink inward.
    ///
    /// If not set, shell is generated directly from the input mesh.
    ///
    /// # Arguments
    ///
    /// * `offset` - Offset distance in millimeters
    #[must_use]
    pub const fn offset(mut self, offset: f64) -> Self {
        self.offset_mm = Some(offset);
        self
    }

    /// Set the voxel size for offset SDF computation in mm.
    ///
    /// Smaller voxels give more detail but use more memory and time.
    /// The default (0.75mm) is a good balance for most use cases.
    ///
    /// # Arguments
    ///
    /// * `size` - Voxel size in millimeters
    #[must_use]
    pub const fn offset_voxel_size(mut self, size: f64) -> Self {
        self.offset_voxel_size_mm = size;
        self
    }

    // =========================================================================
    // Wall/Shell Configuration
    // =========================================================================

    /// Set uniform wall thickness in mm.
    ///
    /// This is the thickness of the shell walls. For 3D printing,
    /// typical values are 1.5-4mm depending on the application.
    ///
    /// # Arguments
    ///
    /// * `thickness` - Wall thickness in millimeters
    #[must_use]
    pub const fn wall_thickness(mut self, thickness: f64) -> Self {
        self.wall_thickness_mm = thickness;
        self
    }

    /// Set minimum acceptable wall thickness in mm.
    ///
    /// Used during validation to flag walls that are too thin
    /// for reliable 3D printing.
    #[must_use]
    pub const fn min_thickness(mut self, thickness: f64) -> Self {
        self.min_thickness_mm = thickness;
        self
    }

    /// Enable or disable post-generation validation.
    ///
    /// When enabled, the generated shell is validated for
    /// manifoldness, watertightness, and other quality metrics.
    #[must_use]
    pub const fn validate(mut self, enable: bool) -> Self {
        self.validate = enable;
        self
    }

    /// Set the voxel size for SDF-based wall generation in mm.
    ///
    /// Only used when `high_quality()` is set.
    #[must_use]
    pub const fn voxel_size(mut self, size: f64) -> Self {
        self.sdf_voxel_size_mm = size;
        self
    }

    /// Use normal-based wall generation (fast but less accurate).
    ///
    /// Each vertex is offset along its normal. Fast, but wall thickness
    /// may vary at corners (thinner at convex, thicker at concave).
    #[must_use]
    pub const fn fast_walls(mut self) -> Self {
        self.wall_method = WallGenerationMethod::Normal;
        self
    }

    /// Use SDF-based wall generation (slower but consistent thickness).
    ///
    /// Computes a signed distance field and extracts an isosurface.
    /// This ensures consistent wall thickness regardless of curvature.
    #[must_use]
    pub const fn sdf_walls(mut self) -> Self {
        self.wall_method = WallGenerationMethod::Sdf;
        self
    }

    // =========================================================================
    // Presets
    // =========================================================================

    /// Apply high-quality preset settings.
    ///
    /// Uses SDF-based wall generation with fine voxel resolution
    /// for consistent wall thickness and smooth surfaces.
    #[must_use]
    pub const fn high_quality(mut self) -> Self {
        self.wall_method = WallGenerationMethod::Sdf;
        self.sdf_voxel_size_mm = 0.3;
        self.validate = true;
        self
    }

    /// Apply fast preset settings.
    ///
    /// Uses normal-based wall generation with validation disabled.
    /// Good for quick previews or when speed is more important than quality.
    #[must_use]
    pub const fn fast(mut self) -> Self {
        self.wall_method = WallGenerationMethod::Normal;
        self.validate = false;
        self
    }

    // =========================================================================
    // Build
    // =========================================================================

    /// Build the shell with the configured parameters.
    ///
    /// This executes the full shell generation pipeline:
    /// 1. Apply optional offset to create inner surface
    /// 2. Generate outer surface with walls
    /// 3. Create rim connecting inner and outer
    /// 4. Validate result (if enabled)
    ///
    /// # Returns
    ///
    /// A `ShellBuildResult` containing the generated mesh and statistics.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The mesh is empty or invalid
    /// - The offset operation fails
    /// - Shell generation fails for any reason
    pub fn build(self) -> ShellResult<ShellBuildResult> {
        // Step 1: Apply optional offset
        let (inner_mesh, offset_applied) = if let Some(offset) = self.offset_mm {
            let config = OffsetConfig::default().with_resolution(self.offset_voxel_size_mm);
            let offset_mesh = offset_mesh(self.mesh, offset, &config)?;
            (offset_mesh, true)
        } else {
            (self.mesh.clone(), false)
        };

        // Step 2: Generate shell
        let shell_params = ShellParams {
            wall_thickness_mm: self.wall_thickness_mm,
            min_thickness_mm: self.min_thickness_mm,
            validate_after_generation: self.validate,
            wall_generation_method: self.wall_method,
            sdf_voxel_size_mm: self.sdf_voxel_size_mm,
        };

        let (shell_mesh, shell_stats) = generate_shell(&inner_mesh, &shell_params)?;

        Ok(ShellBuildResult {
            mesh: shell_mesh,
            offset_applied,
            shell_stats,
        })
    }

    /// Build only the offset surface (no shell walls).
    ///
    /// This is useful when you want just the offset surface without
    /// generating a full shell with walls.
    ///
    /// # Returns
    ///
    /// The offset mesh.
    ///
    /// # Errors
    ///
    /// Returns an error if offset is not set or if the operation fails.
    pub fn build_offset_only(self) -> ShellResult<IndexedMesh> {
        let offset = self.offset_mm.ok_or_else(|| {
            ShellError::invalid_params("offset must be set for build_offset_only")
        })?;

        let config = OffsetConfig::default().with_resolution(self.offset_voxel_size_mm);
        let result = offset_mesh(self.mesh, offset, &config)?;
        Ok(result)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_open_box() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 10.0));

        // 5 faces (open top)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_builder_defaults() {
        let mesh = create_open_box();
        let builder = ShellBuilder::new(&mesh);

        assert!((builder.wall_thickness_mm - 2.5).abs() < 1e-6);
        assert!(builder.validate);
        assert!(builder.offset_mm.is_none());
        assert_eq!(builder.wall_method, WallGenerationMethod::Normal);
    }

    #[test]
    fn test_builder_chaining() {
        let mesh = create_open_box();
        // Note: high_quality() sets sdf_voxel_size_mm to 0.3, so voxel_size() must come after
        let builder = ShellBuilder::new(&mesh)
            .offset(1.0)
            .wall_thickness(2.0)
            .high_quality()
            .voxel_size(0.5)
            .validate(false);

        assert!((builder.offset_mm.unwrap_or(0.0) - 1.0).abs() < 1e-6);
        assert!((builder.wall_thickness_mm - 2.0).abs() < 1e-6);
        assert!((builder.sdf_voxel_size_mm - 0.5).abs() < 1e-6);
        assert!(!builder.validate);
        assert_eq!(builder.wall_method, WallGenerationMethod::Sdf);
    }

    #[test]
    fn test_fast_preset() {
        let mesh = create_open_box();
        let builder = ShellBuilder::new(&mesh).fast();

        assert_eq!(builder.wall_method, WallGenerationMethod::Normal);
        assert!(!builder.validate);
    }

    #[test]
    fn test_high_quality_preset() {
        let mesh = create_open_box();
        let builder = ShellBuilder::new(&mesh).high_quality();

        assert_eq!(builder.wall_method, WallGenerationMethod::Sdf);
        assert!(builder.validate);
    }

    #[test]
    fn test_build_simple() {
        let mesh = create_open_box();
        let result = ShellBuilder::new(&mesh).wall_thickness(2.0).fast().build();

        assert!(result.is_ok());
        let shell = result.expect("should succeed");
        assert!(!shell.mesh.faces.is_empty());
        assert!(!shell.offset_applied);
    }

    #[test]
    fn test_build_offset_only_requires_offset() {
        let mesh = create_open_box();
        let result = ShellBuilder::new(&mesh).build_offset_only();

        assert!(result.is_err());
    }
}
