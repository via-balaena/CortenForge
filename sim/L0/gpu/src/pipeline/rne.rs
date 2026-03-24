//! GPU RNE pipeline — bias forces (gravity + Coriolis + gyroscopic).
//!
//! Compiles `rne.wgsl` (4 entry points), creates bind groups, and
//! dispatches the RNE sequence:
//!   1. `rne_gravity`  — per-joint parallel gravity projection
//!   2. `rne_forward`  — root→leaves bias acceleration scan
//!   3. `rne_backward` — leaves→root bias force accumulation (CAS atomics)
//!   4. `rne_project`  — per-DOF parallel S^T · cfrc projection

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use sim_core::types::Model;

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::PhysicsParams;
use crate::context::GpuContext;

/// Minimum uniform buffer offset alignment (`WebGPU` spec: 256 bytes).
const UNIFORM_ALIGN: u64 = 256;

/// GPU RNE pipeline.
pub struct GpuRnePipeline {
    gravity_pipeline: wgpu::ComputePipeline,
    forward_pipeline: wgpu::ComputePipeline,
    cfrc_init_pipeline: wgpu::ComputePipeline,
    backward_pipeline: wgpu::ComputePipeline,
    project_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,
    output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    max_depth: u32,
    nbody: u32,
    njnt: u32,
    nv: u32,
    n_env: u32,
}

impl GpuRnePipeline {
    /// Create the RNE pipeline from model and state buffers.
    #[must_use]
    pub fn new(
        ctx: &GpuContext,
        gpu_model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        model: &Model,
    ) -> Self {
        let shader_source = include_str!("../shaders/rne.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("rne"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: physics params (dynamic uniform offset)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_params_layout"),
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

        // Group 1: static model (bodies, joints, dofs)
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_model_layout"),
                entries: &[
                    storage_entry(0, true), // bodies
                    storage_entry(1, true), // joints
                    storage_entry(2, true), // dofs
                ],
            });

        // Group 2: FK / velocity FK outputs (read-only)
        let state_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_state_layout"),
                entries: &[
                    storage_entry(0, true), // body_xpos
                    storage_entry(1, true), // body_xquat
                    storage_entry(2, true), // body_cinert
                    storage_entry(3, true), // cdof
                    storage_entry(4, true), // body_cvel
                    storage_entry(5, true), // subtree_mass
                    storage_entry(6, true), // subtree_com
                    storage_entry(7, true), // qvel
                    storage_entry(8, true), // qpos
                ],
            });

        // Group 3: RNE outputs (read-write)
        let output_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_output_layout"),
                entries: &[
                    storage_entry(0, false), // body_cacc
                    storage_entry(1, false), // body_cfrc (atomic<u32>)
                    storage_entry(2, false), // qfrc_bias
                ],
            });

        // ── Pipeline layout ───────────────────────────────────────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("rne_pipeline_layout"),
                bind_group_layouts: &[&params_layout, &model_layout, &state_layout, &output_layout],
                push_constant_ranges: &[],
            });

        // ── Create 4 pipelines ────────────────────────────────────────
        let gravity_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "rne_gravity");
        let forward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "rne_forward");
        let cfrc_init_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "rne_cfrc_init");
        let backward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "rne_backward");
        let project_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "rne_project");

        // ── Params uniform buffer ─────────────────────────────────────
        // Slots: gravity(1) + forward(max_depth+1) + cfrc_init(1) + backward(max_depth) + project(1)
        let n_slots = u64::from(gpu_model.max_depth) * 2 + 5;
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_params"),
            size: n_slots * UNIFORM_ALIGN,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_params_bg"),
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
            label: Some("rne_model_bg"),
            layout: &model_layout,
            entries: &[
                buf_entry(0, &gpu_model.bodies),
                buf_entry(1, &gpu_model.joints),
                buf_entry(2, &gpu_model.dofs),
            ],
        });

        let state_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_state_bg"),
            layout: &state_layout,
            entries: &[
                buf_entry(0, &state.body_xpos),
                buf_entry(1, &state.body_xquat),
                buf_entry(2, &state.body_cinert),
                buf_entry(3, &state.cdof),
                buf_entry(4, &state.body_cvel),
                buf_entry(5, &state.subtree_mass),
                buf_entry(6, &state.subtree_com),
                buf_entry(7, &state.qvel),
                buf_entry(8, &state.qpos),
            ],
        });

        let output_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_output_bg"),
            layout: &output_layout,
            entries: &[
                buf_entry(0, &state.body_cacc),
                buf_entry(1, &state.body_cfrc),
                buf_entry(2, &state.qfrc_bias),
            ],
        });

        let _ = model; // used only for future extensions

        Self {
            gravity_pipeline,
            forward_pipeline,
            cfrc_init_pipeline,
            backward_pipeline,
            project_pipeline,
            params_bind_group,
            model_bind_group,
            state_bind_group,
            output_bind_group,
            params_buffer,
            max_depth: gpu_model.max_depth,
            nbody: gpu_model.nbody,
            njnt: gpu_model.njnt,
            nv: gpu_model.nv,
            n_env: state.n_env,
        }
    }

    /// Write the `PhysicsParams` uniform slots for the RNE sequence.
    ///
    /// Slot layout:
    /// - 0: gravity
    /// - `1..=max_depth+1`: forward (depth 0 → `max_depth`)
    /// - `max_depth+2`...: backward (`max_depth` → 1)
    /// - last: project
    pub fn write_params(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        cpu_model: &Model,
    ) {
        // ── Build base PhysicsParams ──────────────────────────────────
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
            nu: 0, // no actuators for now
        };

        // ── Write param slots ─────────────────────────────────────────
        // Slot 0: gravity (depth unused)
        let mut slot = 0u64;
        ctx.queue.write_buffer(
            &self.params_buffer,
            slot * UNIFORM_ALIGN,
            bytemuck::bytes_of(&base_params),
        );

        // Slots 1..=max_depth+1: forward scan (depth 0 → max_depth)
        slot = 1;
        for depth in 0..=self.max_depth {
            let params = PhysicsParams {
                current_depth: depth,
                ..base_params
            };
            ctx.queue.write_buffer(
                &self.params_buffer,
                slot * UNIFORM_ALIGN,
                bytemuck::bytes_of(&params),
            );
            slot += 1;
        }

        // Slots for backward scan (max_depth → 1)
        for depth in (1..=self.max_depth).rev() {
            let params = PhysicsParams {
                current_depth: depth,
                ..base_params
            };
            ctx.queue.write_buffer(
                &self.params_buffer,
                slot * UNIFORM_ALIGN,
                bytemuck::bytes_of(&params),
            );
            slot += 1;
        }

        // Slot for project (depth unused)
        let project_slot = slot;
        ctx.queue.write_buffer(
            &self.params_buffer,
            project_slot * UNIFORM_ALIGN,
            bytemuck::bytes_of(&base_params),
        );
    }

    /// Encode the RNE compute passes into the given command encoder.
    ///
    /// Must be called after [`write_params`](Self::write_params) has been
    /// invoked for this frame. The slot layout is deterministic from
    /// `self.max_depth`, so no cached state from `write_params` is needed.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };
        let backward_slot_start = u64::from(self.max_depth) + 2;
        let project_slot = backward_slot_start + u64::from(self.max_depth);

        // ── 1. rne_gravity — all joints parallel ─────────────────────
        if self.njnt > 0 {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rne_gravity"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.gravity_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[0]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.njnt), self.n_env, 1);
        }

        // ── 2. rne_forward — one dispatch per depth (0 → max_depth) ──
        for depth in 0..=self.max_depth {
            let offset = ((1 + u64::from(depth)) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rne_forward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.forward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }

        // ── 3a. rne_cfrc_init — all bodies parallel ──────────────────
        // Compute cfrc = I·a_bias + v ×* (I·v) for each body.
        // Must run AFTER forward scan, BEFORE backward accumulation.
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rne_cfrc_init"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.cfrc_init_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[0]); // depth unused
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }

        // ── 3b. rne_backward — one dispatch per depth (max_depth → 1)
        for (i, _depth) in (1..=self.max_depth).rev().enumerate() {
            let offset = ((backward_slot_start + i as u64) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rne_backward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.backward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), self.n_env, 1);
        }

        // ── 4. rne_project — all DOFs parallel ───────────────────────
        if self.nv > 0 {
            let offset = (project_slot * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rne_project"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.project_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.state_bind_group, &[]);
            pass.set_bind_group(3, &self.output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nv), self.n_env, 1);
        }
    }

    /// Dispatch the full RNE sequence (legacy convenience method).
    ///
    /// Calls [`write_params`](Self::write_params), zeros `qfrc_bias` and
    /// `body_cfrc` for backward compatibility, then calls
    /// [`encode`](Self::encode).
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        cpu_model: &Model,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        self.write_params(ctx, model, state, cpu_model);

        // ── Zero qfrc_bias and body_cfrc (legacy) ────────────────────
        if self.nv > 0 {
            let zero_bias = vec![0u8; (self.nv as usize) * 4];
            ctx.queue.write_buffer(&state.qfrc_bias, 0, &zero_bias);
        }
        {
            let zero_cfrc = vec![0u8; (self.nbody as usize) * 32];
            ctx.queue.write_buffer(&state.body_cfrc, 0, &zero_cfrc);
        }

        self.encode(encoder);
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
