//! GPU forward kinematics pipeline.
//!
//! Compiles the `fk.wgsl` shader (4 entry points), creates bind group
//! layouts and bind groups, and dispatches the tree-scan FK passes.
//! Provides readback utilities for validation.

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

/// GPU forward kinematics pipeline.
///
/// Owns 4 compute pipelines (one per entry point), a shared pipeline
/// layout (4 bind groups), and pre-allocated bind groups + uniform buffer.
pub struct GpuFkPipeline {
    // Pipelines (one per entry point)
    fk_forward_pipeline: wgpu::ComputePipeline,
    geom_pipeline: wgpu::ComputePipeline,
    subtree_backward_pipeline: wgpu::ComputePipeline,
    subtree_normalize_pipeline: wgpu::ComputePipeline,

    // Bind groups (created once, reused every dispatch)
    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,
    geom_bind_group: wgpu::BindGroup,

    // Params uniform buffer with one 256-byte slot per depth level
    params_buffer: wgpu::Buffer,

    // Cached counts for dispatch sizing
    max_depth: u32,
    nbody: u32,
    ngeom: u32,
    n_env: u32,
}

impl GpuFkPipeline {
    /// Create the FK pipeline from model and state buffers.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        // ── Compile shader ────────────────────────────────────────────
        let shader_source = include_str!("../shaders/fk.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("fk"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: FkParams uniform (dynamic offset)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("fk_params_layout"),
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

        // Group 1: static model (2 storage read)
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("fk_model_layout"),
                entries: &[
                    storage_entry(0, true), // bodies
                    storage_entry(1, true), // joints
                ],
            });

        // Group 2: per-env state (10 bindings)
        let state_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("fk_state_layout"),
                entries: &[
                    storage_entry(0, true),  // qpos (read)
                    storage_entry(1, false), // body_xpos (rw)
                    storage_entry(2, false), // body_xquat (rw)
                    storage_entry(3, false), // body_xipos (rw)
                    storage_entry(4, false), // body_cinert (rw)
                    storage_entry(5, false), // cdof (rw)
                    storage_entry(6, false), // subtree_mass (rw)
                    storage_entry(7, false), // subtree_com (rw)
                    storage_entry(8, true),  // mocap_pos (read)
                    storage_entry(9, true),  // mocap_quat (read)
                ],
            });

        // Group 3: geom model + output (3 bindings)
        let geom_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("fk_geom_layout"),
                entries: &[
                    storage_entry(0, true),  // geom_models (read)
                    storage_entry(1, false), // geom_xpos (rw)
                    storage_entry(2, false), // geom_xmat (rw)
                ],
            });

        // ── Pipeline layout (shared by all 4 entry points) ───────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("fk_pipeline_layout"),
                bind_group_layouts: &[&params_layout, &model_layout, &state_layout, &geom_layout],
                push_constant_ranges: &[],
            });

        // ── Create 4 pipelines ────────────────────────────────────────
        let fk_forward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "fk_forward");
        let geom_pipeline = create_pipeline(ctx, &pipeline_layout, &shader_module, "fk_geom_poses");
        let subtree_backward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "fk_subtree_backward");
        let subtree_normalize_pipeline = create_pipeline(
            ctx,
            &pipeline_layout,
            &shader_module,
            "fk_subtree_normalize",
        );

        // ── Params uniform buffer ─────────────────────────────────────
        // One 256-byte slot per depth level (+ extras for geom/subtree dispatches)
        let n_slots = (u64::from(model.max_depth) + 1) + 3; // +3 for geom, subtree_backward, subtree_normalize
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("fk_params"),
            size: n_slots * UNIFORM_ALIGN,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("fk_params_bg"),
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
            label: Some("fk_model_bg"),
            layout: &model_layout,
            entries: &[buf_entry(0, &model.bodies), buf_entry(1, &model.joints)],
        });

        let state_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("fk_state_bg"),
            layout: &state_layout,
            entries: &[
                buf_entry(0, &state.qpos),
                buf_entry(1, &state.body_xpos),
                buf_entry(2, &state.body_xquat),
                buf_entry(3, &state.body_xipos),
                buf_entry(4, &state.body_cinert),
                buf_entry(5, &state.cdof),
                buf_entry(6, &state.subtree_mass),
                buf_entry(7, &state.subtree_com),
                buf_entry(8, &state.mocap_pos),
                buf_entry(9, &state.mocap_quat),
            ],
        });

        let geom_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("fk_geom_bg"),
            layout: &geom_layout,
            entries: &[
                buf_entry(0, &model.geoms),
                buf_entry(1, &state.geom_xpos),
                buf_entry(2, &state.geom_xmat),
            ],
        });

        Self {
            fk_forward_pipeline,
            geom_pipeline,
            subtree_backward_pipeline,
            subtree_normalize_pipeline,
            params_bind_group,
            model_bind_group,
            state_bind_group,
            geom_bind_group,
            params_buffer,
            max_depth: model.max_depth,
            nbody: model.nbody,
            ngeom: model.ngeom,
            n_env: state.n_env,
        }
    }

    /// Write all uniform parameter slots to the GPU queue.
    ///
    /// Uploads depth-level params for forward FK, geom poses, and subtree
    /// normalize into the shared params uniform buffer.
    pub fn write_params(&self, ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) {
        // Slot 0..=max_depth: forward FK (one per depth)
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

        // Geom dispatch slot
        let geom_slot = u64::from(self.max_depth) + 1;
        {
            let params = FkParams {
                current_depth: 0, // unused by geom shader
                nbody: model.nbody,
                njnt: model.njnt,
                ngeom: model.ngeom,
                nv: model.nv,
                n_env: state.n_env,
                nq: model.nq,
                _pad: 0,
            };
            ctx.queue.write_buffer(
                &self.params_buffer,
                geom_slot * UNIFORM_ALIGN,
                bytemuck::bytes_of(&params),
            );
        }

        // Subtree backward slots (reuse forward params with reversed depth)
        // We'll reuse the forward slots since they have the right depth values.

        // Subtree normalize slot
        let normalize_slot = geom_slot + 1;
        {
            let params = FkParams {
                current_depth: 0,
                nbody: model.nbody,
                njnt: model.njnt,
                ngeom: model.ngeom,
                nv: model.nv,
                n_env: state.n_env,
                nq: model.nq,
                _pad: 0,
            };
            ctx.queue.write_buffer(
                &self.params_buffer,
                normalize_slot * UNIFORM_ALIGN,
                bytemuck::bytes_of(&params),
            );
        }
    }

    /// Encode the full FK compute passes into a command encoder.
    ///
    /// Encodes forward FK, geom poses, subtree backward, and subtree
    /// normalize dispatches. Must be called after [`Self::write_params`].
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };
        let geom_slot = u64::from(self.max_depth) + 1;
        let normalize_slot = geom_slot + 1;

        // ── Forward FK: one pass per depth level ──────────────────────
        for depth in 0..=self.max_depth {
            let offset = (u64::from(depth) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("fk_forward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.fk_forward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.geom_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }

        // ── Geom poses: one dispatch ──────────────────────────────────
        if self.ngeom > 0 {
            let offset = (geom_slot * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("fk_geom_poses"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.geom_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.geom_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.ngeom), self.n_env, 1);
        }

        // ── Subtree COM: backward scan ────────────────────────────────
        for depth in (1..=self.max_depth).rev() {
            // Reuse forward param slot (same depth value)
            let offset = (u64::from(depth) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("fk_subtree_backward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.subtree_backward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.geom_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }

        // ── Subtree COM: normalize ────────────────────────────────────
        {
            let offset = (normalize_slot * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("fk_subtree_normalize"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.subtree_normalize_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.geom_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }
    }

    /// Dispatch the full FK sequence: forward scan → geom poses → subtree COM.
    ///
    /// Writes all depth-level params before encoding, then encodes all
    /// dispatches into the command encoder. After encoding, call
    /// `ctx.queue.submit()` to execute.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        self.write_params(ctx, model, state);
        self.encode(encoder);
    }
}

// ── Readback utilities ────────────────────────────────────────────────

/// Readback a GPU buffer of `vec4<f32>` to CPU as `Vec<[f32; 4]>`.
#[must_use]
pub fn readback_vec4s(ctx: &GpuContext, buffer: &wgpu::Buffer, count: usize) -> Vec<[f32; 4]> {
    let size = (count * 16) as u64;
    let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback_staging"),
        size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("readback"),
        });
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, size);
    ctx.queue.submit([encoder.finish()]);

    let slice = staging.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    poll_wait(ctx);

    let data = slice.get_mapped_range();
    let floats: &[[f32; 4]] = bytemuck::cast_slice(&data);
    floats.to_vec()
}

/// Readback a GPU buffer of `f32` to CPU.
#[must_use]
pub fn readback_f32s(ctx: &GpuContext, buffer: &wgpu::Buffer, count: usize) -> Vec<f32> {
    let size = (count * 4) as u64;
    let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback_staging"),
        size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("readback"),
        });
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, size);
    ctx.queue.submit([encoder.finish()]);

    let slice = staging.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    poll_wait(ctx);

    let data = slice.get_mapped_range();
    bytemuck::cast_slice::<u8, f32>(&data).to_vec()
}

fn poll_wait(ctx: &GpuContext) {
    match ctx.device.poll(wgpu::PollType::Wait {
        submission_index: None,
        timeout: Some(std::time::Duration::from_secs(5)),
    }) {
        Ok(_) => {}
        Err(e) => log::warn!("GPU poll timeout: {e:?}"),
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
