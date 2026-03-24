//! Static model buffer upload for GPU physics pipeline.
//!
//! Packs per-body, per-joint, and per-geom model data into struct arrays
//! (`BodyModelGpu`, `JointModelGpu`, `GeomModelGpu`) and uploads to GPU.
//! This reduces static model data from ~18 separate buffers to 2,
//! staying well within WGSL storage buffer limits.

#![allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]

use sim_core::types::Model;
use wgpu::util::DeviceExt;

use super::types::{
    BodyModelGpu, DOF_PARENT_NONE, DofModelGpu, GeomModelGpu, JointModelGpu, MOCAP_NONE,
    jnt_type_to_gpu,
};
use crate::context::GpuContext;

/// Static model data on GPU. Created once at model upload.
pub struct GpuModelBuffers {
    /// Packed per-body data: `array<BodyModel>` (storage, read).
    pub bodies: wgpu::Buffer,
    /// Packed per-joint data: `array<JointModel>` (storage, read).
    pub joints: wgpu::Buffer,
    /// Packed per-geom data: `array<GeomModel>` (storage, read).
    pub geoms: wgpu::Buffer,
    /// Packed per-DOF data: `array<DofModel>` (storage, read).
    pub dofs: wgpu::Buffer,

    /// Maximum tree depth (controls dispatch count).
    pub max_depth: u32,
    /// Total body count.
    pub nbody: u32,
    /// Total joint count.
    pub njnt: u32,
    /// Total geom count.
    pub ngeom: u32,
    /// Total DOF count (velocity dimensions).
    pub nv: u32,
    /// Total position coordinate count.
    pub nq: u32,
    /// Number of mocap bodies.
    pub nmocap: u32,
}

impl GpuModelBuffers {
    /// Upload static model data to GPU.
    ///
    /// Computes `body_depth` and `max_depth` from `body_parent`, packs
    /// all per-element fields into struct arrays, and uploads.
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn upload(ctx: &GpuContext, model: &Model) -> Self {
        let nbody = model.nbody;
        let njnt = model.njnt;
        let ngeom = model.ngeom;
        let nv = model.nv;
        let nq = model.nq;

        // ── Compute body_depth from body_parent ───────────────────────
        let mut body_depth = vec![0u32; nbody];
        for b in 1..nbody {
            body_depth[b] = body_depth[model.body_parent[b]] + 1;
        }
        let max_depth = body_depth.iter().copied().max().unwrap_or(0);

        // ── Count mocap bodies ────────────────────────────────────────
        let mut nmocap = 0u32;
        for idx in model.body_mocapid.iter().flatten() {
            nmocap = nmocap.max(*idx as u32 + 1);
        }

        // ── Pack per-body data ────────────────────────────────────────
        let bodies_cpu: Vec<BodyModelGpu> = (0..nbody)
            .map(|b| {
                let mocap_id = model
                    .body_mocapid
                    .get(b)
                    .and_then(|o| *o)
                    .map_or(MOCAP_NONE, |idx| idx as u32);

                let pos = &model.body_pos[b];
                let qc = model.body_quat[b].as_ref().coords;
                let ipos = &model.body_ipos[b];
                let iqc = model.body_iquat[b].as_ref().coords;
                let inertia = &model.body_inertia[b];
                let mass = model.body_mass[b];

                BodyModelGpu {
                    parent: model.body_parent[b] as u32,
                    depth: body_depth[b],
                    jnt_adr: model.body_jnt_adr[b] as u32,
                    jnt_num: model.body_jnt_num[b] as u32,
                    dof_adr: model.body_dof_adr[b] as u32,
                    dof_num: model.body_dof_num[b] as u32,
                    mocap_id,
                    _pad: 0,
                    pos: [pos.x as f32, pos.y as f32, pos.z as f32, 0.0],
                    quat: [qc.x as f32, qc.y as f32, qc.z as f32, qc.w as f32],
                    ipos: [ipos.x as f32, ipos.y as f32, ipos.z as f32, 0.0],
                    iquat: [iqc.x as f32, iqc.y as f32, iqc.z as f32, iqc.w as f32],
                    inertia: [
                        inertia.x as f32,
                        inertia.y as f32,
                        inertia.z as f32,
                        mass as f32,
                    ],
                }
            })
            .collect();

        // ── Pack per-joint data ───────────────────────────────────────
        let joints_cpu: Vec<JointModelGpu> = (0..njnt)
            .map(|j| {
                let axis = &model.jnt_axis[j];
                let jpos = &model.jnt_pos[j];
                JointModelGpu {
                    jtype: jnt_type_to_gpu(model.jnt_type[j]),
                    qpos_adr: model.jnt_qpos_adr[j] as u32,
                    dof_adr: model.jnt_dof_adr[j] as u32,
                    _pad: 0,
                    axis: [axis.x as f32, axis.y as f32, axis.z as f32, 0.0],
                    pos: [jpos.x as f32, jpos.y as f32, jpos.z as f32, 0.0],
                }
            })
            .collect();

        // ── Pack per-geom data ────────────────────────────────────────
        let geoms_cpu: Vec<GeomModelGpu> = (0..ngeom)
            .map(|g| {
                let gpos = &model.geom_pos[g];
                let gqc = model.geom_quat[g].as_ref().coords;
                GeomModelGpu {
                    body_id: model.geom_body[g] as u32,
                    _pad: [0; 3],
                    pos: [gpos.x as f32, gpos.y as f32, gpos.z as f32, 0.0],
                    quat: [gqc.x as f32, gqc.y as f32, gqc.z as f32, gqc.w as f32],
                }
            })
            .collect();

        // ── Pack per-DOF data ────────────────────────────────────────
        let dofs_cpu: Vec<DofModelGpu> = (0..nv)
            .map(|dof| {
                let jnt = model.dof_jnt[dof];
                let arm_jnt = model.jnt_armature[jnt];
                let arm_dof = model.dof_armature.get(dof).copied().unwrap_or(0.0);
                DofModelGpu {
                    body_id: model.dof_body[dof] as u32,
                    parent: model.dof_parent[dof].map_or(DOF_PARENT_NONE, |p| p as u32),
                    armature: (arm_jnt + arm_dof) as f32,
                    _pad: 0,
                }
            })
            .collect();

        // ── Upload to GPU ─────────────────────────────────────────────
        let bodies = upload_structs(ctx, "bodies", &bodies_cpu);
        let joints = upload_structs(ctx, "joints", &joints_cpu);
        let geoms = upload_structs(ctx, "geoms", &geoms_cpu);
        let dofs = upload_structs(ctx, "dofs", &dofs_cpu);

        Self {
            bodies,
            joints,
            geoms,
            dofs,
            max_depth,
            nbody: nbody as u32,
            njnt: njnt as u32,
            ngeom: ngeom as u32,
            nv: nv as u32,
            nq: nq as u32,
            nmocap,
        }
    }
}

fn upload_structs<T: bytemuck::Pod>(ctx: &GpuContext, label: &str, data: &[T]) -> wgpu::Buffer {
    let bytes: &[u8] = if data.is_empty() {
        // wgpu requires non-zero buffer size; provide minimum
        &[0u8; 16]
    } else {
        bytemuck::cast_slice(data)
    };
    ctx.device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytes,
            usage: wgpu::BufferUsages::STORAGE,
        })
}
