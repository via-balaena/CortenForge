//! GPU implicit joint-damping (eulerdamp) pipeline.
//!
//! Compiles `eulerdamp.wgsl` (1 entry point): a single-threaded solve of
//! `(M + h·D)·qacc = qfrc_smooth − D·q̇` into a scratch Cholesky factor,
//! leaving crba's pure `qM`/`qM_factor` untouched. See the shader header for
//! the CPU reference and scope notes.

#![allow(clippy::cast_possible_truncation, clippy::too_many_lines)]

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::PhysicsParams;
use crate::context::GpuContext;

use sim_core::types::Model;

/// GPU eulerdamp pipeline: implicit damped velocity solve.
pub struct GpuEulerdampPipeline {
    solve_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    input_bind_group: wgpu::BindGroup,
    output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    nv: u32,
    n_env: u32,
}

impl GpuEulerdampPipeline {
    /// Create the eulerdamp pipeline.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        let shader_source = include_str!("../shaders/eulerdamp.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("eulerdamp"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // Group 0: physics params (single static uniform slot).
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("eulerdamp_params_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: wgpu::BufferSize::new(
                            std::mem::size_of::<PhysicsParams>() as u64,
                        ),
                    },
                    count: None,
                }],
            });

        // Group 1: per-DOF model (read) — damping.
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("eulerdamp_model_layout"),
                entries: &[storage_entry(0, true)], // dofs
            });

        // Group 2: inputs (read).
        let input_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("eulerdamp_input_layout"),
                entries: &[
                    storage_entry(0, true), // qM
                    storage_entry(1, true), // qfrc_smooth
                    storage_entry(2, true), // qvel
                ],
            });

        // Group 3: outputs (read-write).
        let output_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("eulerdamp_output_layout"),
                entries: &[
                    storage_entry(0, false), // qM_eulerdamp_factor (scratch)
                    storage_entry(1, false), // qacc
                ],
            });

        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("eulerdamp_pipeline_layout"),
                bind_group_layouts: &[&params_layout, &model_layout, &input_layout, &output_layout],
                push_constant_ranges: &[],
            });

        let solve_pipeline = ctx
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("eulerdamp_solve"),
                layout: Some(&pipeline_layout),
                module: &shader_module,
                entry_point: Some("eulerdamp_solve"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            });

        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("eulerdamp_params"),
            size: std::mem::size_of::<PhysicsParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("eulerdamp_params_bg"),
            layout: &params_layout,
            entries: &[buf_entry(0, &params_buffer)],
        });
        let model_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("eulerdamp_model_bg"),
            layout: &model_layout,
            entries: &[buf_entry(0, &model.dofs)],
        });
        let input_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("eulerdamp_input_bg"),
            layout: &input_layout,
            entries: &[
                buf_entry(0, &state.qm),
                buf_entry(1, &state.qfrc_smooth),
                buf_entry(2, &state.qvel),
            ],
        });
        let output_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("eulerdamp_output_bg"),
            layout: &output_layout,
            entries: &[
                buf_entry(0, &state.qm_eulerdamp_factor),
                buf_entry(1, &state.qacc),
            ],
        });

        Self {
            solve_pipeline,
            params_bind_group,
            model_bind_group,
            input_bind_group,
            output_bind_group,
            params_buffer,
            nv: model.nv,
            n_env: state.n_env,
        }
    }

    /// Write the `PhysicsParams` uniform (needs `timestep`, `nv`, `n_env`).
    pub fn write_params(&self, ctx: &GpuContext, model: &GpuModelBuffers, cpu_model: &Model) {
        if self.nv == 0 {
            return;
        }
        let params = PhysicsParams {
            gravity: [0.0, 0.0, 0.0, 0.0],
            timestep: cpu_model.timestep as f32,
            nbody: model.nbody,
            njnt: model.njnt,
            nv: model.nv,
            nq: model.nq,
            n_env: self.n_env,
            current_depth: 0,
            nu: 0,
        };
        ctx.queue
            .write_buffer(&self.params_buffer, 0, bytemuck::bytes_of(&params));
    }

    /// Encode the eulerdamp solve. Assumes `write_params()` ran this frame and
    /// `qM` / `qfrc_smooth` / `qvel` are populated (`crba` + `smooth_assemble`).
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        if self.nv == 0 {
            return;
        }
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("eulerdamp_solve"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.solve_pipeline);
        pass.set_bind_group(0, &self.params_bind_group, &[]);
        pass.set_bind_group(1, &self.model_bind_group, &[]);
        pass.set_bind_group(2, &self.input_bind_group, &[]);
        pass.set_bind_group(3, &self.output_bind_group, &[]);
        pass.dispatch_workgroups(1, self.n_env, 1);
    }

    /// Convenience: `write_params()` + `encode()`.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        cpu_model: &Model,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        self.write_params(ctx, model, cpu_model);
        self.encode(encoder);
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

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
