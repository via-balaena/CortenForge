//! GPU buffer management for batched simulation.
//!
//! Provides the structure-of-arrays buffer layout for N environments
//! sharing one model. All values are f32 on GPU.

#![allow(
    clippy::cast_possible_truncation, // f64→f32 and usize→u32 are intentional
    clippy::cast_lossless,            // u32→u64 via From is verbose for buffer math
)]

use wgpu::BufferUsages;

use crate::context::GpuSimContext;
use crate::error::{GpuSimError, GpuSimResult};

/// Shader parameters matching the WGSL `Params` struct layout.
///
/// Uploaded once at construction to a uniform buffer.
/// Must exactly match the WGSL struct field order and alignment:
/// ```text
/// struct Params { num_envs: u32, nv: u32, timestep: f32, _pad: u32 }
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuParams {
    /// Environment count (capped at `u32::MAX` by the WGSL shader's u32
    /// indexing). `GpuBatchSim::new()` truncates `n: usize` to `u32`.
    pub num_envs: u32,
    /// Degrees of freedom per environment.
    pub nv: u32,
    /// Simulation timestep (f32, converted from model's f64).
    pub timestep: f32,
    _pad: u32,
}

impl GpuParams {
    /// Create params from a model and env count.
    ///
    /// The `nv` field is truncated from `usize` to `u32` (safe in practice —
    /// no model has 4B+ DOFs). The `timestep` is converted from f64 to f32.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // model fields accessed by value
    pub fn from_model(model: &sim_core::Model, num_envs: u32) -> Self {
        Self {
            num_envs,
            nv: model.nv as u32,
            timestep: model.timestep as f32,
            _pad: 0,
        }
    }
}

/// GPU buffers for N environments sharing one `Model` (structure-of-arrays layout).
///
/// Allocates contiguous storage buffers with stride = per-env size.
/// All values are f32 on GPU.
pub struct GpuEnvBuffers {
    /// qvel storage: N × nv f32 values.
    pub qvel: wgpu::Buffer,
    /// qacc storage: N × nv f32 values (read-only in integration shader).
    pub qacc: wgpu::Buffer,
    /// Staging buffer for GPU→CPU download (`MAP_READ` | `COPY_DST`).
    /// Only qvel is downloaded in Phase 10a.
    pub download_staging: wgpu::Buffer,
    /// Number of environments.
    pub num_envs: u32,
    /// Degrees of freedom per env.
    pub nv: u32,
}

impl GpuEnvBuffers {
    /// Allocate buffers for `num_envs` environments with `nv` DOFs each.
    ///
    /// Checks each buffer size against `limits.max_storage_buffer_binding_size`.
    ///
    /// When `num_envs == 0` or `nv == 0`, allocates minimum-size (4 byte)
    /// buffers to satisfy wgpu's `size > 0` requirement.
    ///
    /// # Errors
    ///
    /// Returns `Err(GpuSimError::BatchTooLarge)` if the required buffer size
    /// exceeds the device's max storage buffer binding size, or if the size
    /// computation overflows `u64`.
    pub fn new(ctx: &GpuSimContext, num_envs: u32, nv: u32) -> GpuSimResult<Self> {
        let limit = u64::from(ctx.limits.max_storage_buffer_binding_size);
        // Overflow-safe size computation. checked_mul catches u64
        // overflow; the subsequent limit check catches over-size.
        let raw_size = u64::from(num_envs)
            .checked_mul(u64::from(nv))
            .and_then(|x| x.checked_mul(std::mem::size_of::<f32>() as u64))
            .ok_or(GpuSimError::BatchTooLarge {
                num_envs: num_envs as usize,
                required: u64::MAX, // overflowed — true size exceeds u64
                limit,
            })?;
        if raw_size > limit {
            return Err(GpuSimError::BatchTooLarge {
                num_envs: num_envs as usize,
                required: raw_size,
                limit,
            });
        }
        let buf_size = raw_size.max(4); // wgpu requires size > 0

        let qvel = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_qvel"),
            size: buf_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let qacc = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_qacc"),
            size: buf_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let download_staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_staging"),
            size: buf_size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Ok(Self {
            qvel,
            qacc,
            download_staging,
            num_envs,
            nv,
        })
    }
}
