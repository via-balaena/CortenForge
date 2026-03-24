//! GPU velocity FK pipeline — body spatial velocities from qvel.
//!
//! Compiles `velocity_fk.wgsl` (1 entry point), creates bind groups,
//! and dispatches the forward tree scan for body cvel.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::FkParams;
use crate::context::GpuContext;

/// Minimum uniform buffer offset alignment (`WebGPU` spec: 256 bytes).
const UNIFORM_ALIGN: u64 = 256;

/// GPU velocity FK pipeline.
///
/// Owns 1 compute pipeline, 3 bind groups, and a params uniform buffer.
pub struct GpuVelocityFkPipeline {
    forward_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    max_depth: u32,
    nbody: u32,
}

impl GpuVelocityFkPipeline {
    /// Create the velocity FK pipeline from model and state buffers.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        let shader_source = include_str!("../shaders/velocity_fk.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("velocity_fk"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: params (dynamic uniform offset)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("vel_fk_params_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: wgpu::BufferSize::new(
                            std::mem::size_of::<FkParams>() as u64
                        ),
                    },
                    count: None,
                }],
            });

        // Group 1: static model (bodies only)
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("vel_fk_model_layout"),
                entries: &[storage_entry(0, true)], // bodies
            });

        // Group 2: state (body_xpos, cdof, qvel read; body_cvel write)
        let state_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("vel_fk_state_layout"),
                entries: &[
                    storage_entry(0, true),  // body_xpos
                    storage_entry(1, true),  // cdof
                    storage_entry(2, true),  // qvel
                    storage_entry(3, false), // body_cvel
                ],
            });

        // ── Pipeline layout ───────────────────────────────────────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("vel_fk_pipeline_layout"),
                bind_group_layouts: &[&params_layout, &model_layout, &state_layout],
                push_constant_ranges: &[],
            });

        let forward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "velocity_fk_forward");

        // ── Params uniform buffer ─────────────────────────────────────
        let n_slots = u64::from(model.max_depth) + 1;
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("vel_fk_params"),
            size: n_slots * UNIFORM_ALIGN,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vel_fk_params_bg"),
            layout: &params_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &params_buffer,
                    offset: 0,
                    size: wgpu::BufferSize::new(std::mem::size_of::<FkParams>() as u64),
                }),
            }],
        });

        let model_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vel_fk_model_bg"),
            layout: &model_layout,
            entries: &[buf_entry(0, &model.bodies)],
        });

        let state_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vel_fk_state_bg"),
            layout: &state_layout,
            entries: &[
                buf_entry(0, &state.body_xpos),
                buf_entry(1, &state.cdof),
                buf_entry(2, &state.qvel),
                buf_entry(3, &state.body_cvel),
            ],
        });

        Self {
            forward_pipeline,
            params_bind_group,
            model_bind_group,
            state_bind_group,
            params_buffer,
            max_depth: model.max_depth,
            nbody: model.nbody,
        }
    }

    /// Dispatch the velocity FK forward scan: one pass per depth level.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };

        // Write all depth-level param slots
        for depth in 0..=self.max_depth {
            let params = FkParams {
                current_depth: depth,
                nbody: model.nbody,
                njnt: model.njnt,
                ngeom: model.ngeom,
                nv: model.nv,
                n_env: state.n_env,
                nq: model.nq,
                _pad: 0,
            };
            let offset = u64::from(depth) * UNIFORM_ALIGN;
            ctx.queue
                .write_buffer(&self.params_buffer, offset, bytemuck::bytes_of(&params));
        }

        // Forward scan: one dispatch per depth level (root → leaves)
        for depth in 0..=self.max_depth {
            let offset = (u64::from(depth) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("velocity_fk_forward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.forward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), state.n_env, 1);
        }
    }
}

// ── Pipeline + bind group helpers ─────────────────────────────────────

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
