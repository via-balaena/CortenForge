//! GPU integration pipeline — semi-implicit Euler + quaternion exponential map.
//!
//! Compiles `integrate.wgsl` (1 entry point):
//!   `integrate_euler` — per-joint: qvel += dt*qacc, qpos += dt*f(qvel)

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

/// GPU integration pipeline.
pub struct GpuIntegratePipeline {
    euler_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    njnt: u32,
}

impl GpuIntegratePipeline {
    /// Create the integration pipeline.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        let shader_source = include_str!("../shaders/integrate.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("integrate"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: physics params (non-dynamic — single dispatch)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("integrate_params_layout"),
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

        // Group 1: joints model (read-only)
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("integrate_model_layout"),
                entries: &[storage_entry(0, true)], // joints
            });

        // Group 2: state (qpos + qvel read-write, qacc read)
        let state_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("integrate_state_layout"),
                entries: &[
                    storage_entry(0, false), // qpos
                    storage_entry(1, false), // qvel
                    storage_entry(2, true),  // qacc
                ],
            });

        // ── Pipeline layout ───────────────────────────────────────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("integrate_pipeline_layout"),
                bind_group_layouts: &[&params_layout, &model_layout, &state_layout],
                push_constant_ranges: &[],
            });

        let euler_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "integrate_euler");

        // ── Params uniform buffer (single slot) ───────────────────────
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("integrate_params"),
            size: 256, // single slot, 256-byte aligned
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("integrate_params_bg"),
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

        let model_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("integrate_model_bg"),
            layout: &model_layout,
            entries: &[buf_entry(0, &model.joints)],
        });

        let state_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("integrate_state_bg"),
            layout: &state_layout,
            entries: &[
                buf_entry(0, &state.qpos),
                buf_entry(1, &state.qvel),
                buf_entry(2, &state.qacc),
            ],
        });

        Self {
            euler_pipeline,
            params_bind_group,
            model_bind_group,
            state_bind_group,
            params_buffer,
            njnt: model.njnt,
        }
    }

    /// Dispatch semi-implicit Euler integration.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        cpu_model: &Model,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        if self.njnt == 0 {
            return;
        }

        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };

        let params = PhysicsParams {
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

        ctx.queue
            .write_buffer(&self.params_buffer, 0, bytemuck::bytes_of(&params));

        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("integrate_euler"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.euler_pipeline);
        pass.set_bind_group(0, &self.params_bind_group, &[]);
        pass.set_bind_group(1, &self.model_bind_group, &[]);
        pass.set_bind_group(2, &self.state_bind_group, &[]);
        pass.dispatch_workgroups(ceil64(self.njnt), state.n_env, 1);
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
