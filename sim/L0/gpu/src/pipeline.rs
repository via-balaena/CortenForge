//! GPU-accelerated batch simulation pipeline.
//!
//! Provides [`GpuBatchSim`] as a drop-in replacement for
//! [`sim_core::batch::BatchSim`] that offloads the integration step
//! to GPU compute shaders.

#![allow(
    clippy::cast_possible_truncation, // usize→u32 and f64→f32 are intentional
    clippy::cast_precision_loss,      // usize→f64 for wgpu constants map
    clippy::cast_lossless,            // u32→u64 and f32→f64 via From is verbose
)]

use std::collections::HashMap;
use std::sync::Arc;

use sim_core::batch::BatchSim;
use sim_core::{Data, Integrator, Model, StepError};
use wgpu::util::DeviceExt;

use crate::buffers::{GpuEnvBuffers, GpuParams};
use crate::context::GpuSimContext;
use crate::error::{GpuSimError, GpuSimResult};

/// WGSL shader source, loaded at compile time.
const INTEGRATE_SHADER: &str = include_str!("shaders/euler_integrate.wgsl");

/// GPU-accelerated batch simulation.
///
/// Drop-in replacement for [`sim_core::batch::BatchSim`] that offloads
/// the integration step to GPU compute shaders. All other pipeline stages
/// (FK, collision, constraints, dynamics) run on CPU.
///
/// # Phase 10a Scope
///
/// In this initial phase, only the Euler velocity integration
/// (`qvel += qacc * h`) runs on GPU. This validates the full GPU stack
/// while keeping physics complexity minimal. The CPU `BatchSim` is used
/// internally for the `forward()` pass, and only the integration step
/// is replaced with a GPU dispatch.
pub struct GpuBatchSim {
    /// Underlying CPU batch (owns Model + Data instances).
    cpu_batch: BatchSim,
    /// Per-env buffers on GPU (structure-of-arrays layout).
    buffers: GpuEnvBuffers,
    /// Compiled integration compute pipeline.
    integrate_pipeline: wgpu::ComputePipeline,
    /// Bind group layout for the integration shader.
    #[allow(dead_code)]
    integrate_bind_group_layout: wgpu::BindGroupLayout,
    /// Bind group binding params, qvel, and qacc to the integration shader.
    integrate_bind_group: wgpu::BindGroup,
    /// Uniform buffer for shader parameters.
    #[allow(dead_code)]
    params_buffer: wgpu::Buffer,
}

impl std::fmt::Debug for GpuBatchSim {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuBatchSim")
            .field("num_envs", &self.cpu_batch.len())
            .field("nv", &self.buffers.nv)
            .finish_non_exhaustive()
    }
}

impl GpuBatchSim {
    /// Create a GPU-accelerated batch of `n` environments.
    ///
    /// **Validation order** (checked in this order; first failure wins):
    /// 1. Returns `Err(GpuSimError::UnsupportedIntegrator)` if
    ///    `model.integrator` is not `Euler`.
    /// 2. Returns `Err(GpuSimError::NotAvailable)` if no GPU is available.
    /// 3. Returns `Err(GpuSimError::BatchTooLarge)` if a buffer exceeds
    ///    `limits.max_storage_buffer_binding_size`.
    ///
    /// # Errors
    ///
    /// See validation order above for the three error variants returned.
    #[allow(clippy::too_many_lines)] // GPU pipeline init is naturally verbose
    pub fn new(model: Arc<Model>, n: usize) -> GpuSimResult<Self> {
        // 1. Check integrator (before GPU init — no GPU needed for this check)
        if !matches!(model.integrator, Integrator::Euler) {
            return Err(GpuSimError::UnsupportedIntegrator(model.integrator));
        }

        // 2. Check GPU availability
        let ctx = GpuSimContext::get().ok_or(GpuSimError::NotAvailable)?;

        // 3. Allocate buffers (checks size limits)
        let num_envs = n as u32;
        let nv = model.nv as u32;
        let buffers = GpuEnvBuffers::new(ctx, num_envs, nv)?;

        // Create params uniform buffer
        let params = GpuParams::from_model(&model, num_envs);
        let params_buffer = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sim_params"),
                contents: bytemuck::cast_slice(&[params]),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        // Compile shader
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("euler_integrate"),
                source: wgpu::ShaderSource::Wgsl(INTEGRATE_SHADER.into()),
            });

        // Create bind group layout
        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("euler_integrate_bgl"),
                    entries: &[
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Storage { read_only: false },
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::COMPUTE,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Storage { read_only: true },
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
                label: Some("euler_integrate_pl"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        // Compute wg_size: next_power_of_2(nv), clamped to device max
        let max_wg = ctx.limits.max_compute_workgroup_size_x as usize;
        let wg_size = if model.nv == 0 {
            1 // Avoid 0-size workgroup; shader exits immediately for nv=0
        } else {
            model.nv.next_power_of_two().min(max_wg)
        };

        let constants = HashMap::from([("wg_size".to_string(), wg_size as f64)]);

        let integrate_pipeline =
            ctx.device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some("euler_integrate"),
                    layout: Some(&pipeline_layout),
                    module: &shader_module,
                    entry_point: Some("euler_integrate"),
                    compilation_options: wgpu::PipelineCompilationOptions {
                        constants: &constants,
                        ..Default::default()
                    },
                    cache: None,
                });

        // Create bind group
        let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("euler_integrate_bg"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: buffers.qvel.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: buffers.qacc.as_entire_binding(),
                },
            ],
        });

        // Create CPU batch
        let cpu_batch = BatchSim::new(model, n);

        Ok(Self {
            cpu_batch,
            buffers,
            integrate_pipeline,
            integrate_bind_group_layout: bind_group_layout,
            integrate_bind_group: bind_group,
            params_buffer,
        })
    }

    /// Create a GPU-accelerated batch, returning `None` only when no
    /// GPU is available (`NotAvailable`). Other errors
    /// (`UnsupportedIntegrator`, `BatchTooLarge`, etc.) are propagated
    /// so callers learn about fixable configuration problems rather
    /// than silently falling back to CPU.
    ///
    /// # Errors
    ///
    /// Returns `Err` for all `GpuSimError` variants except `NotAvailable`,
    /// which is mapped to `Ok(None)`.
    pub fn try_new(model: Arc<Model>, n: usize) -> GpuSimResult<Option<Self>> {
        match Self::new(model, n) {
            Ok(sim) => Ok(Some(sim)),
            Err(GpuSimError::NotAvailable) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Number of environments.
    #[must_use]
    pub fn len(&self) -> usize {
        self.cpu_batch.len()
    }

    /// Whether the batch is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.cpu_batch.is_empty()
    }

    /// Shared model reference.
    #[must_use]
    pub fn model(&self) -> &Model {
        self.cpu_batch.model()
    }

    /// Immutable access to environment `i`.
    #[must_use]
    pub fn env(&self, i: usize) -> Option<&Data> {
        self.cpu_batch.env(i)
    }

    /// Mutable access to environment `i` (for setting controls).
    pub fn env_mut(&mut self, i: usize) -> Option<&mut Data> {
        self.cpu_batch.env_mut(i)
    }

    /// Iterator over all environments (immutable).
    #[must_use]
    pub fn envs(&self) -> impl ExactSizeIterator<Item = &Data> {
        self.cpu_batch.envs()
    }

    /// Iterator over all environments (mutable).
    pub fn envs_mut(&mut self) -> impl ExactSizeIterator<Item = &mut Data> {
        self.cpu_batch.envs_mut()
    }

    /// Step all environments by one timestep.
    ///
    /// # Phase 10a Pipeline
    ///
    /// 1. CPU: `data.forward(model)` for each env (rayon-parallel).
    /// 2. CPU→GPU: Upload `qvel`, `qacc` for all envs.
    /// 3. GPU: `euler_integrate` shader: `qvel += qacc * h`.
    /// 4. GPU→CPU: Copy qvel to staging + download (single encoder submit).
    /// 5. CPU: activation integration, position integration, time advance.
    ///
    /// # Errors
    ///
    /// Returns `Err(GpuSimError)` if the GPU dispatch itself fails
    /// (device lost, buffer mapping error). On success, returns
    /// per-environment errors with the same semantics as
    /// [`BatchSim::step_all()`].
    pub fn step_all(&mut self) -> GpuSimResult<Vec<Option<StepError>>> {
        // Early return for empty batch — no GPU work needed
        if self.cpu_batch.is_empty() {
            return Ok(vec![]);
        }

        let ctx = GpuSimContext::get().ok_or(GpuSimError::NotAvailable)?;
        let model = Arc::clone(self.cpu_batch.model_arc());

        // 1. CPU forward pass (parallel across envs)
        let errors: Vec<Option<StepError>> = {
            #[cfg(feature = "parallel")]
            {
                use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
                self.cpu_batch
                    .envs_as_mut_slice()
                    .par_iter_mut()
                    .map(|data| data.forward(&model).err())
                    .collect()
            }
            #[cfg(not(feature = "parallel"))]
            {
                self.cpu_batch
                    .envs_mut()
                    .map(|data| data.forward(&model).err())
                    .collect()
            }
        };

        // 2. Upload qvel + qacc from ALL envs to GPU (f64→f32)
        self.upload_integration_inputs(ctx);

        // 3-4. GPU dispatch + readback
        self.dispatch_and_download(ctx)?;

        // 5. CPU: activation + position integration + time advance
        //    (only for envs whose forward() succeeded)
        for (i, err) in errors.iter().enumerate() {
            if err.is_none() {
                if let Some(data) = self.cpu_batch.env_mut(i) {
                    data.integrate_without_velocity(&model);
                }
            }
        }

        Ok(errors)
    }

    /// Reset environment `i` to initial state.
    pub fn reset(&mut self, i: usize) -> Option<()> {
        self.cpu_batch.reset(i)
    }

    /// Reset environments where `mask[i]` is true.
    pub fn reset_where(&mut self, mask: &[bool]) {
        self.cpu_batch.reset_where(mask);
    }

    /// Reset all environments.
    pub fn reset_all(&mut self) {
        self.cpu_batch.reset_all();
    }

    /// Create a CPU-only `BatchSim` with cloned state from this GPU batch.
    ///
    /// Returns a new `BatchSim` sharing the same `Arc<Model>` (via
    /// `Arc::clone`) with deep-cloned `Data` instances reflecting the
    /// current environment state. Useful for correctness validation:
    /// step both from identical state and compare results.
    #[must_use]
    pub fn cpu_reference(&self) -> BatchSim {
        let model = Arc::clone(self.cpu_batch.model_arc());
        let n = self.cpu_batch.len();
        let mut reference = BatchSim::new(Arc::clone(&model), n);
        for i in 0..n {
            if let (Some(src), Some(dst)) = (self.cpu_batch.env(i), reference.env_mut(i)) {
                *dst = src.clone();
            }
        }
        reference
    }

    // --- Private helpers ---

    /// Upload qvel + qacc from all CPU envs to GPU buffers (f64→f32).
    fn upload_integration_inputs(&self, ctx: &GpuSimContext) {
        let nv = self.buffers.nv as usize;
        let num_envs = self.buffers.num_envs as usize;

        if nv == 0 || num_envs == 0 {
            return;
        }

        let total = num_envs * nv;
        let mut qvel_data = Vec::with_capacity(total);
        let mut qacc_data = Vec::with_capacity(total);

        for i in 0..num_envs {
            if let Some(data) = self.cpu_batch.env(i) {
                for j in 0..nv {
                    qvel_data.push(data.qvel[j] as f32);
                    qacc_data.push(data.qacc[j] as f32);
                }
            }
        }

        ctx.queue
            .write_buffer(&self.buffers.qvel, 0, bytemuck::cast_slice(&qvel_data));
        ctx.queue
            .write_buffer(&self.buffers.qacc, 0, bytemuck::cast_slice(&qacc_data));
    }

    /// Dispatch `euler_integrate` shader and download updated qvel (f32→f64).
    fn dispatch_and_download(&mut self, ctx: &GpuSimContext) -> GpuSimResult<()> {
        let num_envs = self.buffers.num_envs;
        let nv = self.buffers.nv as usize;

        if nv == 0 || num_envs == 0 {
            return Ok(());
        }

        let buf_size = u64::from(num_envs) * (nv as u64) * (std::mem::size_of::<f32>() as u64);

        // Record compute pass + copy in a single command encoder
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("euler_integrate_encoder"),
            });

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("euler_integrate_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.integrate_pipeline);
            pass.set_bind_group(0, &self.integrate_bind_group, &[]);
            pass.dispatch_workgroups(num_envs, 1, 1);
        }

        // Copy qvel → staging for readback
        encoder.copy_buffer_to_buffer(
            &self.buffers.qvel,
            0,
            &self.buffers.download_staging,
            0,
            buf_size,
        );

        ctx.queue.submit(std::iter::once(encoder.finish()));

        // Map staging buffer for CPU read
        let staging_slice = self.buffers.download_staging.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        staging_slice.map_async(wgpu::MapMode::Read, move |result| {
            // Channel send failure means receiver dropped — nothing to do.
            drop(sender.send(result));
        });
        ctx.device.poll(wgpu::Maintain::Wait);

        receiver
            .recv()
            .map_err(|e| GpuSimError::BufferReadback(format!("channel recv failed: {e}")))?
            .map_err(|e| GpuSimError::BufferReadback(format!("buffer mapping failed: {e}")))?;

        // Scatter f32 GPU results back to f64 Data.qvel per environment
        {
            let mapped = staging_slice.get_mapped_range();
            let gpu_qvel: &[f32] = bytemuck::cast_slice(&mapped);
            for i in 0..num_envs as usize {
                if let Some(data) = self.cpu_batch.env_mut(i) {
                    let base = i * nv;
                    for j in 0..nv {
                        data.qvel[j] = f64::from(gpu_qvel[base + j]);
                    }
                }
            }
        }
        self.buffers.download_staging.unmap();

        Ok(())
    }
}
