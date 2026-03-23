//! GPU buffer management for SDF grids.

#![allow(
    clippy::cast_possible_truncation, // f64→f32 and usize→u32 are intentional (GPU uses f32/u32)
    clippy::cast_precision_loss
)]

use bytemuck::{Pod, Zeroable};
use cf_geometry::SdfGrid;
use wgpu::util::DeviceExt;

use crate::GpuContext;

/// Grid metadata matching the WGSL `GridMeta` uniform struct.
///
/// 32 bytes, 16-byte aligned for uniform buffers.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct GridMeta {
    /// Grid X dimension (samples).
    pub width: u32,
    /// Grid Y dimension (samples).
    pub height: u32,
    /// Grid Z dimension (samples).
    pub depth: u32,
    /// Uniform cell spacing (meters).
    pub cell_size: f32,
    /// Grid origin X (minimum corner, meters).
    pub origin_x: f32,
    /// Grid origin Y.
    pub origin_y: f32,
    /// Grid origin Z.
    pub origin_z: f32,
    /// Padding for 16-byte alignment.
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: f32,
}

/// An SDF grid uploaded to GPU memory.
pub struct GpuSdfGrid {
    /// Storage buffer of f32 distance values (ZYX order).
    pub values_buffer: wgpu::Buffer,
    /// Uniform buffer with grid metadata.
    pub meta_buffer: wgpu::Buffer,
    /// CPU-side copy of metadata (for dispatch params).
    pub meta: GridMeta,
    /// Number of f32 values in the grid.
    pub num_values: usize,
}

impl GpuSdfGrid {
    /// Upload an `SdfGrid` to GPU memory.
    ///
    /// Converts f64 values to f32 and creates:
    /// - A storage buffer for grid values (`STORAGE | COPY_SRC`)
    /// - A uniform buffer for grid metadata (`UNIFORM`)
    #[must_use]
    pub fn upload(ctx: &GpuContext, grid: &SdfGrid) -> Self {
        let meta = GridMeta {
            width: grid.width() as u32,
            height: grid.height() as u32,
            depth: grid.depth() as u32,
            cell_size: grid.cell_size() as f32,
            origin_x: grid.origin().x as f32,
            origin_y: grid.origin().y as f32,
            origin_z: grid.origin().z as f32,
            _pad: 0.0,
        };

        // Convert f64 → f32
        let f32_values: Vec<f32> = grid.values().iter().map(|&v| v as f32).collect();

        let values_buffer = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sdf_grid_values"),
                contents: bytemuck::cast_slice(&f32_values),
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            });

        let meta_buffer = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sdf_grid_meta"),
                contents: bytemuck::bytes_of(&meta),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        Self {
            values_buffer,
            meta_buffer,
            meta,
            num_values: f32_values.len(),
        }
    }

    /// Read back GPU grid values to CPU for verification.
    ///
    /// Creates a staging buffer, copies, maps, and returns f32 values.
    #[must_use]
    pub fn readback(&self, ctx: &GpuContext) -> Vec<f32> {
        let size = (self.num_values * std::mem::size_of::<f32>()) as u64;

        let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sdf_grid_readback"),
            size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        encoder.copy_buffer_to_buffer(&self.values_buffer, 0, &staging, 0, size);
        ctx.queue.submit([encoder.finish()]);

        let slice = staging.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        match ctx.device.poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: Some(std::time::Duration::from_secs(5)),
        }) {
            Ok(_) => {}
            Err(e) => log::warn!("GPU poll timeout: {e:?}"),
        }

        let data = slice.get_mapped_range();
        bytemuck::cast_slice(&data).to_vec()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::cast_possible_truncation)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn grid_upload_roundtrip() {
        let ctx = match GpuContext::new() {
            Ok(c) => c,
            Err(e) => {
                eprintln!("  Skipping (no GPU): {e}");
                return;
            }
        };
        let grid = SdfGrid::sphere(Point3::origin(), 5.0, 16, 1.0);

        let gpu_grid = GpuSdfGrid::upload(&ctx, &grid);

        // Verify metadata
        assert_eq!(gpu_grid.meta.width, grid.width() as u32);
        assert_eq!(gpu_grid.meta.height, grid.height() as u32);
        assert_eq!(gpu_grid.meta.depth, grid.depth() as u32);
        assert!((f64::from(gpu_grid.meta.cell_size) - grid.cell_size()).abs() < 1e-6);

        // Readback and compare
        let readback = gpu_grid.readback(&ctx);
        let expected: Vec<f32> = grid.values().iter().map(|&v| v as f32).collect();
        assert_eq!(readback.len(), expected.len());
        for (i, (&got, &want)) in readback.iter().zip(expected.iter()).enumerate() {
            assert!(
                (got - want).abs() < 1e-6,
                "mismatch at index {i}: got {got}, want {want}",
            );
        }

        eprintln!(
            "  Grid uploaded: {}x{}x{} = {} values",
            grid.width(),
            grid.height(),
            grid.depth(),
            readback.len()
        );
    }
}
