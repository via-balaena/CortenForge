//! Static model buffer upload for GPU physics pipeline.
//!
//! Packs per-body, per-joint, and per-geom model data into struct arrays
//! (`BodyModelGpu`, `JointModelGpu`, `GeomModelGpu`) and uploads to GPU.
//! This reduces static model data from ~18 separate buffers to 2,
//! staying well within WGSL storage buffer limits.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use sim_core::types::Model;
use wgpu::util::DeviceExt;

use super::types::{
    BodyModelGpu, DOF_PARENT_NONE, DofModelGpu, GeomModelGpu, JointModelGpu, MOCAP_NONE,
    SDF_META_NONE, SdfMetaGpu, geom_type_to_gpu, jnt_type_to_gpu,
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

    // SDF grid data (Session 4 — collision pipeline)
    /// Concatenated SDF grid values: `array<f32>` covering all shapes.
    pub sdf_values: wgpu::Buffer,
    /// Per-shape metadata: `array<SdfMeta>`.
    pub sdf_metas: wgpu::Buffer,
    /// Number of SDF shapes.
    pub nshape: u32,

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

    // Constraint solve data (Session 5)
    /// Per-body inverse weight for diagonal approximation: `array<vec4<f32>>`.
    /// x = translational (1/mass), y = rotational (3/trace(I)), z = 0, w = 0.
    pub body_invweight0: wgpu::Buffer,
}

impl GpuModelBuffers {
    /// Upload static model data to GPU.
    ///
    /// Computes `body_depth` and `max_depth` from `body_parent`, packs
    /// all per-element fields into struct arrays, and uploads.
    #[must_use]
    // One-shot static-data upload: each body/joint/geom/dof field array is
    // packed and uploaded in one linear pass. Splitting into per-field
    // helpers would just spread the bytemuck calls without simplifying.
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
                    body_id: model.jnt_body[j] as u32,
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
                let gsize = &model.geom_size[g];
                let gfric = &model.geom_friction[g];
                let sdf_meta_idx = model
                    .geom_shape
                    .get(g)
                    .and_then(|o| *o)
                    .map_or(SDF_META_NONE, |s| s as u32);
                let rbound = model.geom_rbound.get(g).copied().unwrap_or(0.0);

                GeomModelGpu {
                    body_id: model.geom_body[g] as u32,
                    geom_type: geom_type_to_gpu(model.geom_type[g]),
                    contype: model.geom_contype[g],
                    conaffinity: model.geom_conaffinity[g],
                    pos: [gpos.x as f32, gpos.y as f32, gpos.z as f32, 0.0],
                    quat: [gqc.x as f32, gqc.y as f32, gqc.z as f32, gqc.w as f32],
                    size: [
                        gsize.x as f32,
                        gsize.y as f32,
                        gsize.z as f32,
                        rbound as f32,
                    ],
                    friction: [gfric.x as f32, gfric.y as f32, gfric.z as f32, 0.0],
                    sdf_meta_idx,
                    condim: model.geom_condim[g] as u32,
                    _pad: [0; 2],
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

        // ── Pack unified SDF grid data ────────────────────────────────
        let nshape = model.shape_data.len();
        let mut all_sdf_values: Vec<f32> = Vec::new();
        let mut sdf_metas_cpu: Vec<SdfMetaGpu> = Vec::new();

        for shape in &model.shape_data {
            let grid = shape.sdf_grid();
            let offset = all_sdf_values.len() as u32;
            all_sdf_values.extend(grid.values().iter().map(|&v| v as f32));
            sdf_metas_cpu.push(SdfMetaGpu {
                width: grid.width() as u32,
                height: grid.height() as u32,
                depth: grid.depth() as u32,
                cell_size: grid.cell_size() as f32,
                origin: [
                    grid.origin().x as f32,
                    grid.origin().y as f32,
                    grid.origin().z as f32,
                ],
                values_offset: offset,
            });
        }

        // ── Pack per-body inverse weight (Session 5) ────────────────
        let invweight0_cpu: Vec<[f32; 4]> = (0..nbody)
            .map(|b| {
                let trans = if b < model.body_invweight0.len() {
                    model.body_invweight0[b][0] as f32
                } else {
                    0.0
                };
                let rot = if b < model.body_invweight0.len() {
                    model.body_invweight0[b][1] as f32
                } else {
                    0.0
                };
                [trans, rot, 0.0, 0.0]
            })
            .collect();

        // ── Upload to GPU ─────────────────────────────────────────────
        let bodies = upload_structs(ctx, "bodies", &bodies_cpu);
        let joints = upload_structs(ctx, "joints", &joints_cpu);
        let geoms = upload_structs(ctx, "geoms", &geoms_cpu);
        let dofs = upload_structs(ctx, "dofs", &dofs_cpu);
        let sdf_values = upload_structs(ctx, "sdf_values", &all_sdf_values);
        let sdf_metas = upload_structs(ctx, "sdf_metas", &sdf_metas_cpu);
        let body_invweight0 = upload_structs(ctx, "body_invweight0", &invweight0_cpu);

        Self {
            bodies,
            joints,
            geoms,
            dofs,
            sdf_values,
            sdf_metas,
            nshape: nshape as u32,
            max_depth,
            nbody: nbody as u32,
            njnt: njnt as u32,
            ngeom: ngeom as u32,
            nv: nv as u32,
            nq: nq as u32,
            nmocap,
            body_invweight0,
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
