//! GPU CRBA pipeline — mass matrix computation + dense Cholesky.
//!
//! Compiles `crba.wgsl` (4 entry points), creates bind group layouts and
//! bind groups, and dispatches the CRBA sequence:
//!   1. `crba_init` — convert cinert to crb accumulation format
//!   2. `crba_backward` — leaves-to-root CAS atomic accumulation
//!   3. `crba_mass_matrix` — per-DOF parallel M assembly
//!   4. `crba_cholesky` — dense Cholesky factorization (single thread)

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

/// GPU CRBA pipeline.
///
/// Owns 4 compute pipelines (one per entry point), a shared pipeline layout,
/// and pre-allocated bind groups + uniform buffer.
pub struct GpuCrbaPipeline {
    // Pipelines
    init_pipeline: wgpu::ComputePipeline,
    backward_pipeline: wgpu::ComputePipeline,
    mass_matrix_pipeline: wgpu::ComputePipeline,
    cholesky_pipeline: wgpu::ComputePipeline,

    // Bind groups
    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    fk_state_bind_group: wgpu::BindGroup,
    crba_output_bind_group: wgpu::BindGroup,

    // Params uniform buffer
    params_buffer: wgpu::Buffer,

    // Cached counts
    n_env: u32,
    max_depth: u32,
    nbody: u32,
    nv: u32,
}

impl GpuCrbaPipeline {
    /// Create the CRBA pipeline from model and state buffers.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) -> Self {
        let shader_source = include_str!("../shaders/crba.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("crba"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        // ── Bind group layouts ────────────────────────────────────────

        // Group 0: params (dynamic uniform offset)
        let params_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("crba_params_layout"),
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

        // Group 1: static model (bodies + dofs)
        let model_layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("crba_model_layout"),
                entries: &[
                    storage_entry(0, true), // bodies
                    storage_entry(1, true), // dofs
                ],
            });

        // Group 2: FK outputs (read-only for CRBA)
        let fk_state_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("crba_fk_state_layout"),
                    entries: &[
                        storage_entry(0, true), // body_xpos
                        storage_entry(1, true), // body_cinert
                        storage_entry(2, true), // cdof
                    ],
                });

        // Group 3: CRBA outputs (read-write)
        let crba_output_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("crba_output_layout"),
                    entries: &[
                        storage_entry(0, false), // body_crb (atomic<u32>)
                        storage_entry(1, false), // qM
                        storage_entry(2, false), // qM_factor
                    ],
                });

        // ── Pipeline layout ───────────────────────────────────────────
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("crba_pipeline_layout"),
                bind_group_layouts: &[
                    &params_layout,
                    &model_layout,
                    &fk_state_layout,
                    &crba_output_layout,
                ],
                push_constant_ranges: &[],
            });

        // ── Create 4 pipelines ────────────────────────────────────────
        let init_pipeline = create_pipeline(ctx, &pipeline_layout, &shader_module, "crba_init");
        let backward_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "crba_backward");
        let mass_matrix_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "crba_mass_matrix");
        let cholesky_pipeline =
            create_pipeline(ctx, &pipeline_layout, &shader_module, "crba_cholesky");

        // ── Params uniform buffer ─────────────────────────────────────
        // Slots: init(1) + backward(max_depth) + mass_matrix(1) + cholesky(1)
        let n_slots = u64::from(model.max_depth) + 4;
        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("crba_params"),
            size: n_slots * UNIFORM_ALIGN,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // ── Bind groups ───────────────────────────────────────────────
        let params_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("crba_params_bg"),
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
            label: Some("crba_model_bg"),
            layout: &model_layout,
            entries: &[buf_entry(0, &model.bodies), buf_entry(1, &model.dofs)],
        });

        let fk_state_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("crba_fk_state_bg"),
            layout: &fk_state_layout,
            entries: &[
                buf_entry(0, &state.body_xpos),
                buf_entry(1, &state.body_cinert),
                buf_entry(2, &state.cdof),
            ],
        });

        let crba_output_bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("crba_output_bg"),
            layout: &crba_output_layout,
            entries: &[
                buf_entry(0, &state.body_crb),
                buf_entry(1, &state.qm),
                buf_entry(2, &state.qm_factor),
            ],
        });

        Self {
            init_pipeline,
            backward_pipeline,
            mass_matrix_pipeline,
            cholesky_pipeline,
            params_bind_group,
            model_bind_group,
            fk_state_bind_group,
            crba_output_bind_group,
            params_buffer,
            n_env: state.n_env,
            max_depth: model.max_depth,
            nbody: model.nbody,
            nv: model.nv,
        }
    }

    /// Write all uniform param slots to the GPU queue.
    ///
    /// Uploads init, backward (one per depth level), `mass_matrix`, and cholesky
    /// param slots. Does **not** zero `qM` — the orchestrator handles that.
    pub fn write_params(&self, ctx: &GpuContext, model: &GpuModelBuffers, state: &GpuStateBuffers) {
        let base_params = FkParams {
            current_depth: 0,
            nbody: model.nbody,
            njnt: model.njnt,
            ngeom: model.ngeom,
            nv: model.nv,
            n_env: state.n_env,
            nq: model.nq,
            _pad: 0,
        };

        // Slot 0: init (depth unused)
        ctx.queue
            .write_buffer(&self.params_buffer, 0, bytemuck::bytes_of(&base_params));

        // Slots 1..=max_depth: backward (one per depth level)
        for depth in 1..=self.max_depth {
            let params = FkParams {
                current_depth: depth,
                ..base_params
            };
            let offset = u64::from(depth) * UNIFORM_ALIGN;
            ctx.queue
                .write_buffer(&self.params_buffer, offset, bytemuck::bytes_of(&params));
        }

        // Slot max_depth+1: mass_matrix (depth unused)
        let mm_slot = u64::from(self.max_depth) + 1;
        ctx.queue.write_buffer(
            &self.params_buffer,
            mm_slot * UNIFORM_ALIGN,
            bytemuck::bytes_of(&base_params),
        );

        // Slot max_depth+2: cholesky (depth unused)
        let chol_slot = mm_slot + 1;
        ctx.queue.write_buffer(
            &self.params_buffer,
            chol_slot * UNIFORM_ALIGN,
            bytemuck::bytes_of(&base_params),
        );
    }

    /// Encode the CRBA compute passes into a command encoder.
    ///
    /// Encodes init, backward, `mass_matrix`, cholesky. The caller must
    /// ensure params have been written (via [`write_params`]) and qM has been
    /// zeroed before submitting the resulting command buffer.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        let ceil64 = |n: u32| -> u32 { n.div_ceil(64) };
        let nv = self.nv;
        let n_env = self.n_env;
        let mm_slot = u64::from(self.max_depth) + 1;
        let chol_slot = mm_slot + 1;

        // ── 1. crba_init ──────────────────────────────────────────────
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("crba_init"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.init_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[0]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.fk_state_bind_group, &[]);
            pass.set_bind_group(3, &self.crba_output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), n_env, 1);
        }

        // ── 2. crba_backward (leaves → root) ─────────────────────────
        for depth in (1..=self.max_depth).rev() {
            let offset = (u64::from(depth) * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("crba_backward"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.backward_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.fk_state_bind_group, &[]);
            pass.set_bind_group(3, &self.crba_output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(self.nbody), n_env, 1);
        }

        // ── 3. crba_mass_matrix ───────────────────────────────────────
        if nv > 0 {
            let offset = (mm_slot * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("crba_mass_matrix"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.mass_matrix_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.fk_state_bind_group, &[]);
            pass.set_bind_group(3, &self.crba_output_bind_group, &[]);
            pass.dispatch_workgroups(ceil64(nv), n_env, 1);
        }

        // ── 4. crba_cholesky ──────────────────────────────────────────
        if nv > 0 {
            let offset = (chol_slot * UNIFORM_ALIGN) as u32;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("crba_cholesky"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.cholesky_pipeline);
            pass.set_bind_group(0, &self.params_bind_group, &[offset]);
            pass.set_bind_group(1, &self.model_bind_group, &[]);
            pass.set_bind_group(2, &self.fk_state_bind_group, &[]);
            pass.set_bind_group(3, &self.crba_output_bind_group, &[]);
            pass.dispatch_workgroups(1, n_env, 1);
        }
    }

    /// Dispatch the full CRBA sequence: init → backward → zero qM → mass matrix → Cholesky.
    ///
    /// Convenience method that calls [`write_params`], zeros qM, then [`encode`].
    /// Kept for backward compatibility.
    pub fn dispatch(
        &self,
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        state: &GpuStateBuffers,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        self.write_params(ctx, model, state);

        // Zero qM before mass matrix assembly
        let nv = self.nv;
        if nv > 0 {
            let zero_bytes = vec![0u8; (nv as usize) * (nv as usize) * 4];
            ctx.queue.write_buffer(&state.qm, 0, &zero_bytes);
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
