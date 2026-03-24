//! GPU smooth dynamics pipeline — force assembly + Cholesky solve.
//!
//! Compiles `smooth.wgsl` (2 entry points):
//!   1. `smooth_assemble` — per-DOF: `qfrc_smooth` = applied + actuator + passive - bias
//!   2. `smooth_solve`    — single thread: L·L^T solve for `qacc_smooth`

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::PhysicsParams;
use crate::context::GpuContext;

use sim_core::types::Model;

/// Minimum uniform buffer offset alignment (`WebGPU` spec: 256 bytes).
const UNIFORM_ALIGN: u64 = 256;

/// GPU smooth dynamics pipeline.
pub struct GpuSmoothPipeline {
    assemble_pipeline: wgpu::ComputePipeline,
    solve_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    forces_bind_group: wgpu::BindGroup,
    factor_bind_group: wgpu::BindGroup,
    output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    nv: u32,
}

impl GpuSmoothPipeline {
    /// Create the smooth dynamics pipeline.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        let shader_source = include_str!("../shaders/smooth.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("smooth"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: physics params (dynamic uniform offset)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("smooth_params_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: wgpu::BufferSize::new(
                            std::mem::size_of::<PhysicsParams>() as u64,
                        ),
                    },
                    count: None,
                }],
            });

        // Group 1: force inputs (read-only)
        let forces_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("smooth_forces_layout"),
                entries: &[
                    storage_entry(0, true), // qfrc_bias
                    storage_entry(1, true), // qfrc_applied
                    storage_entry(2, true), // qfrc_actuator
                    storage_entry(3, true), // qfrc_passive
                ],
            });

        // Group 2: mass matrix factor (read-only)
        let factor_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("smooth_factor_layout"),
                entries: &[storage_entry(0, true)], // qM_factor
            });

        // Group 3: outputs (read-write)
        let output_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("smooth_output_layout"),
                entries: &[
                    storage_entry(0, false), // qfrc_smooth
                    storage_entry(1, false), // qacc_smooth
                ],
            });

        // ── Pipeline layout ───────────────────────────────────────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("smooth_pipeline_layout"),
                bind_group_layouts: &[
                    &params_layout,
                    &forces_layout,
                    &factor_layout,
                    &output_layout,
                ],
                push_constant_ranges: &[],
            });

        let assemble_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "smooth_assemble");
        let solve_pipeline = create_pipeline(ctx, &pipeline_layout, &shader_module, "smooth_solve");

        // ── Params uniform buffer (2 slots: assemble + solve) ─────────
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("smooth_params"),
            size: 2 * UNIFORM_ALIGN,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("smooth_params_bg"),
            layout: &params_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &params_buffer,
                    offset: 0,
                    size: wgpu::BufferSize::new(std::mem::size_of::<PhysicsParams>() as u64),
                }),
            }],
        });

        let forces_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("smooth_forces_bg"),
            layout: &forces_layout,
            entries: &[
                buf_entry(0, &state.qfrc_bias),
                buf_entry(1, &state.qfrc_applied),
                buf_entry(2, &state.qfrc_actuator),
                buf_entry(3, &state.qfrc_passive),
            ],
        });

        let factor_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("smooth_factor_bg"),
            layout: &factor_layout,
            entries: &[buf_entry(0, &state.qm_factor)],
        });

        let output_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("smooth_output_bg"),
            layout: &output_layout,
            entries: &[
                buf_entry(0, &state.qfrc_smooth),
                buf_entry(1, &state.qacc_smooth),
            ],
        });

        Self {
            assemble_pipeline,
            solve_pipeline,
            params_bind_group,
            forces_bind_group,
            factor_bind_group,
            output_bind_group,
            params_buffer,
            nv: model.nv,
        }
    }

    /// Dispatch smooth dynamics: assemble + Cholesky solve.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        cpu_model: &Model,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        let nv = self.nv;
        if nv == 0 {
            return;
        }

        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };

        let base_params = PhysicsParams {
            gravity: [
                cpu_model.gravity.x as f32,
                cpu_model.gravity.y as f32,
                cpu_model.gravity.z as f32,
                0.0,
            ],
            timestep: cpu_model.timestep as f32,
            nbody: model.nbody,
            njnt: model.njnt,
            nv: model.nv,
            nq: model.nq,
            n_env: state.n_env,
            current_depth: 0,
            nu: 0,
        };

        // Slot 0: assemble
        ctx.queue
            .write_buffer(&self.params_buffer, 0, bytemuck::bytes_of(&base_params));
        // Slot 1: solve
        ctx.queue.write_buffer(
            &self.params_buffer,
            UNIFORM_ALIGN,
            bytemuck::bytes_of(&base_params),
        );

        // Zero force accumulators (gravity-only: actuator + passive + applied = 0)
        let zero_bytes = vec![0u8; (nv as usize) * 4];
        ctx.queue.write_buffer(&state.qfrc_applied, 0, &zero_bytes);
        ctx.queue.write_buffer(&state.qfrc_actuator, 0, &zero_bytes);
        ctx.queue.write_buffer(&state.qfrc_passive, 0, &zero_bytes);

        // ── 1. smooth_assemble ───────────────────────────────────────
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("smooth_assemble"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.assemble_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[0]);
            pass.set_bind_group(1, &self.forces_bind_group, &[]);
            pass.set_bind_group(2, &self.factor_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(nv), state.n_env, 1);
        }

        // ── 2. smooth_solve ──────────────────────────────────────────
        {
            let offset = UNIFORM_ALIGN as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("smooth_solve"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.solve_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.forces_bind_group, &[]);
            pass.set_bind_group(2, &self.factor_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(1, state.n_env, 1);
        }
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

fn create_pipeline(
    ctx: &GpuContext,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    entry_point: &str,
) -> wgpu::ComputePipeline {
    ctx.device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry_point),
            layout: Some(layout),
            module,
            entry_point: Some(entry_point),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
}

const fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn buf_entry(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}
