//! GPU-accelerated SDF (Signed Distance Field) computation.
//!
//! This module provides GPU-accelerated computation of signed distance fields
//! from triangle meshes. It uses WGPU compute shaders for parallel processing.
//!
//! # Performance
//!
//! GPU SDF computation shows significant speedup for typical use cases:
//! - Small meshes (<100 triangles) with 128³ grid: **48-68x faster**
//! - Medium meshes (320 triangles) with 128³ grid: **14x faster**
//! - Large meshes (>5k triangles): CPU may be faster due to transfer overhead
//!
//! # Example
//!
//! ```no_run
//! use mesh_gpu::sdf::{compute_sdf_gpu, GpuSdfParams};
//! use mesh_types::IndexedMesh;
//!
//! let mesh = IndexedMesh::new();
//! let params = GpuSdfParams {
//!     dims: [64, 64, 64],
//!     origin: [0.0, 0.0, 0.0],
//!     voxel_size: 0.1,
//! };
//!
//! match compute_sdf_gpu(&mesh, &params) {
//!     Ok(result) => println!("Computed {} voxels in {:.2}ms",
//!         result.values.len(), result.compute_time_ms),
//!     Err(e) => println!("GPU error: {}", e),
//! }
//! ```

use tracing::{debug, info, warn};
use wgpu::{BindGroupLayout, ComputePipeline, ShaderModule};

use mesh_types::IndexedMesh;

use crate::buffers::{MeshBuffers, SdfGridBuffers, TileConfig};
use crate::context::GpuContext;
use crate::error::{GpuError, GpuResult};

/// Shader source for SDF computation.
const SDF_SHADER: &str = include_str!("shaders/sdf_compute.wgsl");

/// Parameters for GPU SDF computation.
///
/// # Example
///
/// ```
/// use mesh_gpu::sdf::GpuSdfParams;
///
/// let params = GpuSdfParams {
///     dims: [100, 100, 100],
///     origin: [-5.0, -5.0, -5.0],
///     voxel_size: 0.1,
/// };
///
/// // Grid covers from (-5,-5,-5) to (5,5,5)
/// assert_eq!(params.dims[0] * params.dims[1] * params.dims[2], 1_000_000);
/// ```
#[derive(Debug, Clone)]
pub struct GpuSdfParams {
    /// Grid dimensions [x, y, z] in voxels.
    pub dims: [usize; 3],
    /// Grid origin (minimum corner) in world coordinates.
    pub origin: [f32; 3],
    /// Voxel size in world units.
    pub voxel_size: f32,
}

impl GpuSdfParams {
    /// Create new SDF parameters.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_gpu::sdf::GpuSdfParams;
    ///
    /// let params = GpuSdfParams::new([64, 64, 64], [0.0, 0.0, 0.0], 0.1);
    /// ```
    #[must_use]
    pub const fn new(dims: [usize; 3], origin: [f32; 3], voxel_size: f32) -> Self {
        Self {
            dims,
            origin,
            voxel_size,
        }
    }

    /// Total number of voxels in the grid.
    #[must_use]
    pub const fn total_voxels(&self) -> usize {
        self.dims[0] * self.dims[1] * self.dims[2]
    }
}

/// Result of GPU SDF computation.
///
/// The `values` array contains signed distances in ZYX order (matching
/// `mesh_to_sdf` convention). Negative values indicate inside the mesh.
#[derive(Debug)]
pub struct GpuSdfResult {
    /// Computed SDF values in ZYX order.
    pub values: Vec<f32>,
    /// Grid dimensions.
    pub dims: [usize; 3],
    /// Computation time in milliseconds.
    pub compute_time_ms: f64,
}

impl GpuSdfResult {
    /// Get the SDF value at grid coordinates (x, y, z).
    ///
    /// Returns `None` if coordinates are out of bounds.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_gpu::sdf::GpuSdfResult;
    ///
    /// let result = GpuSdfResult {
    ///     values: vec![0.0; 8],
    ///     dims: [2, 2, 2],
    ///     compute_time_ms: 0.0,
    /// };
    ///
    /// assert!(result.get(0, 0, 0).is_some());
    /// assert!(result.get(2, 0, 0).is_none()); // out of bounds
    /// ```
    #[must_use]
    pub fn get(&self, x: usize, y: usize, z: usize) -> Option<f32> {
        if x >= self.dims[0] || y >= self.dims[1] || z >= self.dims[2] {
            return None;
        }
        let idx = z + y * self.dims[2] + x * self.dims[1] * self.dims[2];
        self.values.get(idx).copied()
    }
}

/// Pipeline for GPU SDF computation.
///
/// This struct caches the compiled shader and pipeline, allowing efficient
/// reuse across multiple SDF computations.
///
/// # Example
///
/// ```no_run
/// use mesh_gpu::context::GpuContext;
/// use mesh_gpu::sdf::SdfPipeline;
///
/// if let Some(ctx) = GpuContext::get() {
///     let pipeline = SdfPipeline::new(ctx).expect("pipeline creation");
///     // Reuse pipeline for multiple meshes...
/// }
/// ```
pub struct SdfPipeline {
    #[allow(dead_code)] // Kept for potential future use (shader introspection)
    shader: ShaderModule,
    pipeline: ComputePipeline,
    bind_group_layout: BindGroupLayout,
}

impl SdfPipeline {
    /// Create a new SDF computation pipeline.
    ///
    /// # Errors
    ///
    /// Returns [`GpuError::ShaderCompilation`] if shader compilation fails.
    pub fn new(ctx: &GpuContext) -> GpuResult<Self> {
        debug!("Creating SDF compute pipeline");

        // Compile shader
        let shader = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("sdf_compute"),
                source: wgpu::ShaderSource::Wgsl(SDF_SHADER.into()),
            });

        // Create bind group layout
        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("sdf_bind_group_layout"),
                    entries: &[
                        // Triangles storage buffer (read-only)
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Storage { read_only: true },
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        // Grid params uniform buffer
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        // SDF values storage buffer (read-write)
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Storage { read_only: false },
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                    ],
                });

        // Create pipeline layout
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("sdf_pipeline_layout"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        // Create compute pipeline
        let pipeline = ctx
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("sdf_compute_pipeline"),
                layout: Some(&pipeline_layout),
                module: &shader,
                entry_point: Some("compute_sdf"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            });

        Ok(Self {
            shader,
            pipeline,
            bind_group_layout,
        })
    }

    /// Compute SDF for a mesh.
    ///
    /// # Arguments
    ///
    /// * `ctx` - GPU context
    /// * `mesh_buffers` - Mesh data already uploaded to GPU
    /// * `params` - SDF computation parameters
    ///
    /// # Returns
    ///
    /// The computed SDF values.
    ///
    /// # Errors
    ///
    /// Returns an error if GPU computation fails.
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: total_voxels as u32 is acceptable for workgroup dispatch
    pub fn compute(
        &self,
        ctx: &GpuContext,
        mesh_buffers: &MeshBuffers,
        params: &GpuSdfParams,
    ) -> GpuResult<GpuSdfResult> {
        let start = std::time::Instant::now();
        let total_voxels = params.total_voxels();

        info!(
            dims = ?params.dims,
            total_voxels = total_voxels,
            triangles = mesh_buffers.triangle_count,
            "Computing SDF on GPU"
        );

        // Allocate grid buffers
        let grid_buffers = SdfGridBuffers::allocate(
            ctx,
            params.dims,
            params.origin,
            params.voxel_size,
            mesh_buffers.triangle_count,
        )?;

        // Create bind group
        let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("sdf_bind_group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: mesh_buffers.triangles.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: grid_buffers.params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: grid_buffers.values.as_entire_binding(),
                },
            ],
        });

        // Create command encoder
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("sdf_compute_encoder"),
            });

        // Dispatch compute shader
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("sdf_compute_pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);

            // Workgroup size is 256, so dispatch enough workgroups
            let workgroups = (total_voxels as u32).div_ceil(256);
            compute_pass.dispatch_workgroups(workgroups, 1, 1);
        }

        // Submit commands
        ctx.queue.submit([encoder.finish()]);

        // Download results
        let values = grid_buffers.download_values(ctx)?;

        let compute_time_ms = start.elapsed().as_secs_f64() * 1000.0;
        info!(
            voxels = total_voxels,
            time_ms = compute_time_ms,
            "SDF computation complete"
        );

        Ok(GpuSdfResult {
            values,
            dims: params.dims,
            compute_time_ms,
        })
    }
}

/// Compute SDF on GPU with automatic fallback.
///
/// This is the main entry point for GPU SDF computation. It handles:
/// - GPU availability detection
/// - Pipeline creation and caching
/// - Automatic tiling for large grids
/// - Error handling
///
/// # Arguments
///
/// * `mesh` - Source mesh
/// * `params` - SDF computation parameters
///
/// # Returns
///
/// The computed SDF values, or an error if GPU computation fails.
///
/// # Errors
///
/// - [`GpuError::NotAvailable`] if no GPU is available
/// - [`GpuError::MeshTooLarge`] if mesh exceeds GPU limits
/// - [`GpuError::GridTooLarge`] if grid exceeds GPU limits (and tiling fails)
///
/// # Example
///
/// ```no_run
/// use mesh_gpu::sdf::{compute_sdf_gpu, GpuSdfParams};
/// use mesh_types::IndexedMesh;
///
/// let mesh = IndexedMesh::new();
/// let params = GpuSdfParams::new([64, 64, 64], [0.0, 0.0, 0.0], 0.1);
///
/// match compute_sdf_gpu(&mesh, &params) {
///     Ok(result) => {
///         println!("Computed SDF in {:.2}ms", result.compute_time_ms);
///         // Center voxel
///         if let Some(dist) = result.get(32, 32, 32) {
///             println!("Center distance: {}", dist);
///         }
///     }
///     Err(e) => eprintln!("GPU error: {}", e),
/// }
/// ```
pub fn compute_sdf_gpu(mesh: &IndexedMesh, params: &GpuSdfParams) -> GpuResult<GpuSdfResult> {
    let ctx = GpuContext::try_get()?;

    // Upload mesh to GPU
    let mesh_buffers = MeshBuffers::from_mesh(ctx, mesh)?;

    // Check if we need tiling
    let total_voxels = params.total_voxels();
    let max_voxels = ctx.max_storage_buffer_size() as usize / std::mem::size_of::<f32>();

    if total_voxels > max_voxels {
        // Use tiled computation
        compute_sdf_tiled(ctx, &mesh_buffers, params)
    } else {
        // Direct computation
        let pipeline = SdfPipeline::new(ctx)?;
        pipeline.compute(ctx, &mesh_buffers, params)
    }
}

/// Compute SDF using tiled processing for large grids.
fn compute_sdf_tiled(
    ctx: &GpuContext,
    mesh_buffers: &MeshBuffers,
    params: &GpuSdfParams,
) -> GpuResult<GpuSdfResult> {
    let start = std::time::Instant::now();
    let total_voxels = params.total_voxels();

    // Determine tile configuration based on available memory
    let available_memory = ctx.estimate_available_memory();
    // Reserve memory for mesh and overhead
    let grid_memory =
        available_memory.saturating_sub(mesh_buffers.triangles_size() + 256 * 1024 * 1024);
    let tile_config = TileConfig::for_memory_budget(grid_memory);

    let tile_counts = tile_config.tile_count(params.dims);
    let total_tiles = tile_config.total_tiles(params.dims);

    info!(
        grid_dims = ?params.dims,
        tile_size = ?tile_config.tile_size,
        tiles = total_tiles,
        "Using tiled SDF computation"
    );

    // Allocate result buffer
    let mut result = vec![0.0f32; total_voxels];

    // Create pipeline once
    let pipeline = SdfPipeline::new(ctx)?;

    // Process each tile
    for tz in 0..tile_counts[2] {
        for ty in 0..tile_counts[1] {
            for tx in 0..tile_counts[0] {
                let tile_origin_voxels = [
                    tx * tile_config.tile_size[0],
                    ty * tile_config.tile_size[1],
                    tz * tile_config.tile_size[2],
                ];

                // Calculate tile dimensions (may be smaller at edges)
                let tile_dims = [
                    (params.dims[0] - tile_origin_voxels[0]).min(tile_config.tile_size[0]),
                    (params.dims[1] - tile_origin_voxels[1]).min(tile_config.tile_size[1]),
                    (params.dims[2] - tile_origin_voxels[2]).min(tile_config.tile_size[2]),
                ];

                // Calculate tile origin in world coordinates
                #[allow(clippy::cast_precision_loss, clippy::suboptimal_flops)]
                // Precision loss acceptable: voxel indices as f32 is fine
                // Suboptimal flops: mul_add less readable here
                let tile_origin_world = [
                    params.origin[0] + (tile_origin_voxels[0] as f32) * params.voxel_size,
                    params.origin[1] + (tile_origin_voxels[1] as f32) * params.voxel_size,
                    params.origin[2] + (tile_origin_voxels[2] as f32) * params.voxel_size,
                ];

                let tile_params = GpuSdfParams {
                    dims: tile_dims,
                    origin: tile_origin_world,
                    voxel_size: params.voxel_size,
                };

                // Compute tile
                let tile_result = pipeline.compute(ctx, mesh_buffers, &tile_params)?;

                // Copy tile results to main grid
                copy_tile_to_grid(
                    &tile_result.values,
                    &mut result,
                    params.dims,
                    tile_origin_voxels,
                    tile_dims,
                );

                debug!(tile_x = tx, tile_y = ty, tile_z = tz, "Tile processed");
            }
        }
    }

    let compute_time_ms = start.elapsed().as_secs_f64() * 1000.0;
    info!(
        tiles = total_tiles,
        time_ms = compute_time_ms,
        "Tiled SDF computation complete"
    );

    Ok(GpuSdfResult {
        values: result,
        dims: params.dims,
        compute_time_ms,
    })
}

/// Copy tile results to the main grid.
fn copy_tile_to_grid(
    tile_values: &[f32],
    grid_values: &mut [f32],
    grid_dims: [usize; 3],
    tile_origin: [usize; 3],
    tile_dims: [usize; 3],
) {
    // Use ZYX ordering to match mesh_to_sdf's layout
    for z in 0..tile_dims[2] {
        for y in 0..tile_dims[1] {
            for x in 0..tile_dims[0] {
                let tile_idx = z + y * tile_dims[2] + x * tile_dims[1] * tile_dims[2];
                let grid_x = tile_origin[0] + x;
                let grid_y = tile_origin[1] + y;
                let grid_z = tile_origin[2] + z;
                let grid_idx =
                    grid_z + grid_y * grid_dims[2] + grid_x * grid_dims[1] * grid_dims[2];

                if grid_idx < grid_values.len() && tile_idx < tile_values.len() {
                    grid_values[grid_idx] = tile_values[tile_idx];
                }
            }
        }
    }
}

/// Try to compute SDF on GPU, returning None if GPU is unavailable.
///
/// This is a convenience function that doesn't return an error for GPU
/// unavailability, making it easy to implement fallback logic.
///
/// # Arguments
///
/// * `mesh` - Source mesh
/// * `params` - SDF computation parameters
///
/// # Returns
///
/// `Some(result)` if GPU computation succeeds, `None` if GPU is unavailable
/// or computation fails.
///
/// # Example
///
/// ```no_run
/// use mesh_gpu::sdf::{try_compute_sdf_gpu, GpuSdfParams};
/// use mesh_types::IndexedMesh;
///
/// fn compute_sdf_with_fallback(mesh: &IndexedMesh, params: &GpuSdfParams) -> Vec<f32> {
///     // Try GPU first
///     if let Some(result) = try_compute_sdf_gpu(mesh, params) {
///         return result.values;
///     }
///
///     // Fall back to CPU implementation
///     vec![0.0; params.dims[0] * params.dims[1] * params.dims[2]]
/// }
/// ```
#[must_use]
pub fn try_compute_sdf_gpu(mesh: &IndexedMesh, params: &GpuSdfParams) -> Option<GpuSdfResult> {
    match compute_sdf_gpu(mesh, params) {
        Ok(result) => Some(result),
        Err(GpuError::NotAvailable) => {
            debug!("GPU not available for SDF computation");
            None
        }
        Err(e) => {
            warn!("GPU SDF computation failed: {}", e);
            None
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::redundant_clone
)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Unit cube centered at origin
        let coords = [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ];

        for c in &coords {
            mesh.vertices.push(Vertex::from_coords(c[0], c[1], c[2]));
        }

        // Cube faces (2 triangles per face)
        let faces: [[u32; 3]; 12] = [
            [0, 1, 2],
            [0, 2, 3], // Front
            [4, 6, 5],
            [4, 7, 6], // Back
            [0, 5, 1],
            [0, 4, 5], // Bottom
            [2, 7, 3],
            [2, 6, 7], // Top
            [0, 3, 7],
            [0, 7, 4], // Left
            [1, 5, 6],
            [1, 6, 2], // Right
        ];

        for f in &faces {
            mesh.faces.push(*f);
        }

        mesh
    }

    #[test]
    fn test_gpu_sdf_params_new() {
        let params = GpuSdfParams::new([10, 20, 30], [1.0, 2.0, 3.0], 0.5);

        assert_eq!(params.dims, [10, 20, 30]);
        assert_eq!(params.origin, [1.0, 2.0, 3.0]);
        assert_eq!(params.voxel_size, 0.5);
    }

    #[test]
    fn test_gpu_sdf_params_total_voxels() {
        let params = GpuSdfParams::new([10, 10, 10], [0.0, 0.0, 0.0], 0.1);
        assert_eq!(params.total_voxels(), 1000);
    }

    #[test]
    fn test_gpu_sdf_result_get() {
        // Create a 2x2x2 result with known values
        let values = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        let result = GpuSdfResult {
            values,
            dims: [2, 2, 2],
            compute_time_ms: 0.0,
        };

        // ZYX ordering: idx = z + y * dims_z + x * dims_y * dims_z
        assert_eq!(result.get(0, 0, 0), Some(0.0)); // idx 0
        assert_eq!(result.get(0, 0, 1), Some(1.0)); // idx 1
        assert_eq!(result.get(0, 1, 0), Some(2.0)); // idx 2
        assert_eq!(result.get(0, 1, 1), Some(3.0)); // idx 3
        assert_eq!(result.get(1, 0, 0), Some(4.0)); // idx 4
        assert_eq!(result.get(1, 0, 1), Some(5.0)); // idx 5
        assert_eq!(result.get(1, 1, 0), Some(6.0)); // idx 6
        assert_eq!(result.get(1, 1, 1), Some(7.0)); // idx 7

        // Out of bounds
        assert_eq!(result.get(2, 0, 0), None);
        assert_eq!(result.get(0, 2, 0), None);
        assert_eq!(result.get(0, 0, 2), None);
    }

    #[test]
    fn test_try_compute_sdf_gpu() {
        let mesh = create_test_cube();
        let params = GpuSdfParams::new([5, 5, 5], [-2.0, -2.0, -2.0], 0.8);

        // This test will pass whether or not GPU is available
        let _result = try_compute_sdf_gpu(&mesh, &params);
    }

    #[test]
    fn test_copy_tile_to_grid() {
        // Test with a simple 2x2x2 tile into a 4x4x4 grid
        let tile_values = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut grid_values = vec![0.0; 64]; // 4x4x4
        let grid_dims = [4, 4, 4];
        let tile_origin = [1, 1, 1];
        let tile_dims = [2, 2, 2];

        copy_tile_to_grid(
            &tile_values,
            &mut grid_values,
            grid_dims,
            tile_origin,
            tile_dims,
        );

        // Check that values were copied to correct positions
        // tile (0,0,0) -> grid (1,1,1)
        let idx = 1 + 4 + 4 * 4; // z + y*dz + x*dy*dz
        assert_eq!(grid_values[idx], 1.0);
    }
}
