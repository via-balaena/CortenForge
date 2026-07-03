//! GPU constraint solve pipeline — assembly + Newton solver + force mapping.
//!
//! Session 5: Reads contacts from collision pipeline (Session 4), assembles
//! pyramidal constraint rows, runs primal Newton solver, maps forces back
//! to joint space. Replaces the gravity-only bridge (`qacc = qacc_smooth`).
//!
//! Three compute shaders:
//!   1. `assemble.wgsl`      — per-contact: PipelineContact → efc_J/D/aref
//!   2. `newton_solve.wgsl`  — single workgroup: Newton iterations → qacc
//!   3. `map_forces.wgsl`    — per-DOF: J^T · `efc_force` → `qfrc_constraint`
//!
//! **Limitation:** Currently only assembles contact constraints. Equality
//! constraints (Connect, Weld, Joint, Distance, Tendon) are fully implemented
//! on the CPU pipeline (`sim-core::constraint::equality`) but not yet ported
//! to GPU shaders.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::doc_markdown,
    clippy::too_many_lines
)]

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::{AssemblyParams, SolverParams};
use super::wgpu_helpers::{buf_entry, create_pipeline, storage_entry, uniform_entry};
use crate::context::GpuContext;

use sim_core::types::Model;

/// Default solref `[timeconst, dampratio]` — matches MuJoCo defaults.
const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];
/// Default solimp `[d0, d_width, width, midpoint, power]` — matches MuJoCo defaults.
const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

/// GPU constraint solve pipeline.
pub struct GpuConstraintPipeline {
    assemble_pipeline: wgpu::ComputePipeline,
    newton_pipeline: wgpu::ComputePipeline,
    map_forces_pipeline: wgpu::ComputePipeline,

    // Bind groups for assemble (4 groups)
    assemble_bg0: wgpu::BindGroup,
    assemble_bg1: wgpu::BindGroup,
    assemble_bg2: wgpu::BindGroup,
    assemble_bg3: wgpu::BindGroup,

    // Bind groups for newton (4 groups)
    newton_bg0: wgpu::BindGroup,
    newton_bg1: wgpu::BindGroup,
    newton_bg2: wgpu::BindGroup,
    newton_bg3: wgpu::BindGroup,

    // Bind groups for map_forces (3 groups)
    mapf_bg0: wgpu::BindGroup,
    mapf_bg1: wgpu::BindGroup,
    mapf_bg2: wgpu::BindGroup,

    nv: u32,
    max_contacts: u32,
    n_env: u32,
}

impl GpuConstraintPipeline {
    /// Create the constraint pipeline. Compiles shaders and creates bind groups.
    #[must_use]
    pub fn new(
        ctx: &GpuContext,
        model_bufs: &GpuModelBuffers,
        state_bufs: &GpuStateBuffers,
        cpu_model: &Model,
    ) -> Self {
        let nv = model_bufs.nv;
        // Capacity is owned by the state buffers (single-env vs batched differ), so
        // the shader strides + dispatch sizes match the buffers' actual sizing.
        let max_contacts = state_bufs.max_contacts;
        let max_constraints = super::types::CONSTRAINT_ROWS_PER_CONTACT * max_contacts;

        // ── Compile shaders ────────────────────────────────────────
        let assemble_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("assemble"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/assemble.wgsl").into()),
            });
        let newton_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("newton_solve"),
                source: wgpu::ShaderSource::Wgsl(
                    include_str!("../shaders/newton_solve.wgsl").into(),
                ),
            });
        let mapf_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("map_forces"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/map_forces.wgsl").into()),
            });

        // ── Assembly uniform buffer ────────────────────────────────
        let solref = DEFAULT_SOLREF;
        let solimp = DEFAULT_SOLIMP;
        let assembly_params = AssemblyParams {
            nv,
            max_contacts,
            max_constraints,
            nbody: model_bufs.nbody,
            timestep: cpu_model.timestep as f32,
            impratio: cpu_model.impratio as f32,
            solref_timeconst: solref[0] as f32,
            solref_dampratio: solref[1] as f32,
            solimp_d0: solimp[0] as f32,
            solimp_dwidth: solimp[1] as f32,
            solimp_width: solimp[2] as f32,
            solimp_midpoint: solimp[3] as f32,
            solimp_power: solimp[4] as f32,
            n_env: state_bufs.n_env,
            _pad: [0; 2],
        };
        // Written once here; the bind groups below retain it (a wgpu bind group
        // Arc-holds its buffers), so no owning field is needed on the struct.
        let assembly_params_buf =
            create_uniform(ctx, "assembly_params", bytemuck::bytes_of(&assembly_params));

        // ── Solver uniform buffer ──────────────────────────────────
        let solver_params = SolverParams {
            nv,
            max_iter: cpu_model.solver_iterations as u32,
            n_env: state_bufs.n_env,
            max_constraints,
        };
        // Retained by the newton + map_forces bind groups (see note above).
        let solver_params_buf =
            create_uniform(ctx, "solver_params", bytemuck::bytes_of(&solver_params));

        // ════════════════════════════════════════════════════════════
        //  ASSEMBLE bind group layouts + pipeline
        // ════════════════════════════════════════════════════════════

        let asm_layout0 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("asm_layout0"),
                entries: &[uniform_entry(0)],
            });
        let asm_layout1 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("asm_layout1"),
                entries: &[
                    storage_entry(0, true), // geoms
                    storage_entry(1, true), // bodies
                    storage_entry(2, true), // body_invweight0
                ],
            });
        let asm_layout2 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("asm_layout2"),
                entries: &[
                    storage_entry(0, true),  // contact_buffer
                    storage_entry(1, false), // contact_count (atomicLoad requires read_write)
                    storage_entry(2, true),  // body_xpos
                    storage_entry(3, true),  // body_xquat
                    storage_entry(4, true),  // qvel
                ],
            });
        let asm_layout3 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("asm_layout3"),
                entries: &[
                    storage_entry(0, false), // efc_J
                    storage_entry(1, false), // efc_D
                    storage_entry(2, false), // efc_aref
                    storage_entry(3, false), // constraint_count
                ],
            });

        let asm_pipe_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("assemble_pipe_layout"),
                bind_group_layouts: &[&asm_layout0, &asm_layout1, &asm_layout2, &asm_layout3],
                push_constant_ranges: &[],
            });
        let assemble_pipeline = create_pipeline(
            ctx,
            &asm_pipe_layout,
            &assemble_module,
            "assemble_constraints",
        );

        let assemble_bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("asm_bg0"),
            layout: &asm_layout0,
            entries: &[buf_entry(0, &assembly_params_buf)],
        });
        let assemble_bg1 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("asm_bg1"),
            layout: &asm_layout1,
            entries: &[
                buf_entry(0, &model_bufs.geoms),
                buf_entry(1, &model_bufs.bodies),
                buf_entry(2, &model_bufs.body_invweight0),
            ],
        });
        let assemble_bg2 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("asm_bg2"),
            layout: &asm_layout2,
            entries: &[
                buf_entry(0, &state_bufs.contact_buffer),
                buf_entry(1, &state_bufs.contact_count),
                buf_entry(2, &state_bufs.body_xpos),
                buf_entry(3, &state_bufs.body_xquat),
                buf_entry(4, &state_bufs.qvel),
            ],
        });
        let assemble_bg3 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("asm_bg3"),
            layout: &asm_layout3,
            entries: &[
                buf_entry(0, &state_bufs.efc_j),
                buf_entry(1, &state_bufs.efc_d),
                buf_entry(2, &state_bufs.efc_aref),
                buf_entry(3, &state_bufs.constraint_count),
            ],
        });

        // ════════════════════════════════════════════════════════════
        //  NEWTON bind group layouts + pipeline
        // ════════════════════════════════════════════════════════════

        let newton_layout0 =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("newton_layout0"),
                    entries: &[uniform_entry(0)],
                });
        let newton_layout1 =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("newton_layout1"),
                    entries: &[
                        storage_entry(0, true), // qM
                        storage_entry(1, true), // qacc_smooth
                        storage_entry(2, true), // qfrc_smooth
                    ],
                });
        let newton_layout2 =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("newton_layout2"),
                    entries: &[
                        storage_entry(0, true),  // efc_J
                        storage_entry(1, true),  // efc_D
                        storage_entry(2, true),  // efc_aref
                        storage_entry(3, false), // constraint_count (atomicLoad requires read_write)
                    ],
                });
        let newton_layout3 =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("newton_layout3"),
                    entries: &[
                        storage_entry(0, false), // qacc
                        storage_entry(1, false), // efc_force
                    ],
                });

        let newton_pipe_layout =
            ctx.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("newton_pipe_layout"),
                    bind_group_layouts: &[
                        &newton_layout0,
                        &newton_layout1,
                        &newton_layout2,
                        &newton_layout3,
                    ],
                    push_constant_ranges: &[],
                });
        let newton_pipeline =
            create_pipeline(ctx, &newton_pipe_layout, &newton_module, "newton_solve");

        let newton_bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("newton_bg0"),
            layout: &newton_layout0,
            entries: &[buf_entry(0, &solver_params_buf)],
        });
        let newton_bg1 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("newton_bg1"),
            layout: &newton_layout1,
            entries: &[
                buf_entry(0, &state_bufs.qm),
                buf_entry(1, &state_bufs.qacc_smooth),
                buf_entry(2, &state_bufs.qfrc_smooth),
            ],
        });
        let newton_bg2 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("newton_bg2"),
            layout: &newton_layout2,
            entries: &[
                buf_entry(0, &state_bufs.efc_j),
                buf_entry(1, &state_bufs.efc_d),
                buf_entry(2, &state_bufs.efc_aref),
                buf_entry(3, &state_bufs.constraint_count),
            ],
        });
        let newton_bg3 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("newton_bg3"),
            layout: &newton_layout3,
            entries: &[
                buf_entry(0, &state_bufs.qacc),
                buf_entry(1, &state_bufs.efc_force),
            ],
        });

        // ════════════════════════════════════════════════════════════
        //  MAP_FORCES bind group layouts + pipeline
        // ════════════════════════════════════════════════════════════

        let mapf_layout0 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("mapf_layout0"),
                entries: &[uniform_entry(0)],
            });
        let mapf_layout1 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("mapf_layout1"),
                entries: &[
                    storage_entry(0, true),  // efc_J
                    storage_entry(1, true),  // efc_force
                    storage_entry(2, false), // constraint_count (atomicLoad requires read_write)
                ],
            });
        let mapf_layout2 = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("mapf_layout2"),
                entries: &[storage_entry(0, false)], // qfrc_constraint
            });

        let mapf_pipe_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("mapf_pipe_layout"),
                bind_group_layouts: &[&mapf_layout0, &mapf_layout1, &mapf_layout2],
                push_constant_ranges: &[],
            });
        let map_forces_pipeline =
            create_pipeline(ctx, &mapf_pipe_layout, &mapf_module, "map_forces");

        let mapf_bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mapf_bg0"),
            layout: &mapf_layout0,
            entries: &[buf_entry(0, &solver_params_buf)],
        });
        let mapf_bg1 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mapf_bg1"),
            layout: &mapf_layout1,
            entries: &[
                buf_entry(0, &state_bufs.efc_j),
                buf_entry(1, &state_bufs.efc_force),
                buf_entry(2, &state_bufs.constraint_count),
            ],
        });
        let mapf_bg2 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mapf_bg2"),
            layout: &mapf_layout2,
            entries: &[buf_entry(0, &state_bufs.qfrc_constraint)],
        });

        Self {
            assemble_pipeline,
            newton_pipeline,
            map_forces_pipeline,
            assemble_bg0,
            assemble_bg1,
            assemble_bg2,
            assemble_bg3,
            newton_bg0,
            newton_bg1,
            newton_bg2,
            newton_bg3,
            mapf_bg0,
            mapf_bg1,
            mapf_bg2,
            nv,
            max_contacts,
            n_env: state_bufs.n_env,
        }
    }

    /// Encode constraint solve dispatches into the command encoder.
    ///
    /// Sequence: clear constraint_count → assemble → newton → map_forces.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder, state_bufs: &GpuStateBuffers) {
        if self.nv == 0 {
            return;
        }

        // Reset the per-env constraint row counters (one atomic u32 per env).
        // Must clear ALL n_env slots, not just env 0 — the allocator sized this
        // buffer ×n_env, so its paired clear is ×n_env too.
        encoder.clear_buffer(
            &state_bufs.constraint_count,
            0,
            Some(u64::from(self.n_env) * 4),
        );

        // ── 1. Constraint assembly ─────────────────────────────────
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("assemble_contacts"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.assemble_pipeline);
            pass.set_bind_group(0, &self.assemble_bg0, &[]);
            pass.set_bind_group(1, &self.assemble_bg1, &[]);
            pass.set_bind_group(2, &self.assemble_bg2, &[]);
            pass.set_bind_group(3, &self.assemble_bg3, &[]);
            // Env axis on gid.y: one workgroup column per env (assemble.wgsl reads
            // env_id = gid.y). Newton + map_forces stay env-0-only until PR D3.
            pass.dispatch_workgroups(self.max_contacts.div_ceil(256), self.n_env, 1);
        }

        // ── 2. Newton solver (single workgroup) ────────────────────
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("newton_solve"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.newton_pipeline);
            pass.set_bind_group(0, &self.newton_bg0, &[]);
            pass.set_bind_group(1, &self.newton_bg1, &[]);
            pass.set_bind_group(2, &self.newton_bg2, &[]);
            pass.set_bind_group(3, &self.newton_bg3, &[]);
            // One workgroup per env (newton_solve.wgsl reads env = workgroup_id.x).
            pass.dispatch_workgroups(self.n_env, 1, 1);
        }

        // ── 3. Force mapping ───────────────────────────────────────
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("map_forces"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.map_forces_pipeline);
            pass.set_bind_group(0, &self.mapf_bg0, &[]);
            pass.set_bind_group(1, &self.mapf_bg1, &[]);
            pass.set_bind_group(2, &self.mapf_bg2, &[]);
            // Env axis on gid.y (map_forces.wgsl reads env_id = gid.y).
            pass.dispatch_workgroups(self.nv.div_ceil(64), self.n_env, 1);
        }
    }
}

fn create_uniform(ctx: &GpuContext, label: &str, data: &[u8]) -> wgpu::Buffer {
    use wgpu::util::DeviceExt;
    ctx.device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        })
}
