//! Per-environment mutable state buffers for GPU physics pipeline.
//!
//! These hold simulation state. Each environment has its own copy.
//! For `n_env=1` (interactive), all strides are 1× element count.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names, // body_xpos vs body_xipos is intentional (MuJoCo naming)
    missing_docs,
)]

use sim_core::types::Data;
use wgpu::util::DeviceExt;

use super::model_buffers::GpuModelBuffers;
use super::types::{MAX_PIPELINE_CONTACTS, f64s_to_f32s};
use crate::context::GpuContext;

/// Per-environment mutable state on GPU.
pub struct GpuStateBuffers {
    // Input (uploaded from CPU)
    pub qpos: wgpu::Buffer,

    // FK output — body poses
    pub body_xpos: wgpu::Buffer,
    pub body_xquat: wgpu::Buffer,
    pub body_xipos: wgpu::Buffer,
    pub body_cinert: wgpu::Buffer,

    // FK output — geom poses
    pub geom_xpos: wgpu::Buffer,
    pub geom_xmat: wgpu::Buffer,

    // FK output — motion subspace
    pub cdof: wgpu::Buffer,

    // FK output — subtree COM
    pub subtree_mass: wgpu::Buffer,
    pub subtree_com: wgpu::Buffer,

    // Mocap input (uploaded from CPU per frame)
    pub mocap_pos: wgpu::Buffer,
    pub mocap_quat: wgpu::Buffer,

    // CRBA state (Session 2)
    /// Composite rigid body inertia: `atomic<u32>` x 12 per body (3xvec4 as u32).
    pub body_crb: wgpu::Buffer,
    /// Dense mass matrix: `f32` x nv x nv.
    pub qm: wgpu::Buffer,
    /// Dense Cholesky factor: `f32` x nv x nv (lower triangular L).
    pub qm_factor: wgpu::Buffer,

    // Velocity FK state (Session 2)
    /// Joint velocities input: `f32` × nv (uploaded from CPU).
    pub qvel: wgpu::Buffer,
    /// Body spatial velocities: `vec4<f32>` × 2 per body [angular, linear].
    pub body_cvel: wgpu::Buffer,

    // RNE state (Session 3)
    /// Body bias accelerations: `vec4<f32>` × 2 per body [angular, linear].
    pub body_cacc: wgpu::Buffer,
    /// Body bias forces: `atomic<u32>` × 8 per body (2×vec4 as u32, for CAS backward scan).
    pub body_cfrc: wgpu::Buffer,
    /// Bias force vector (gravity + Coriolis + gyroscopic): `f32` × nv.
    pub qfrc_bias: wgpu::Buffer,

    // Force accumulators (Session 3 — zeroed for gravity-only)
    /// Applied generalized forces: `f32` × nv (uploaded from CPU, zero for gravity-only).
    pub qfrc_applied: wgpu::Buffer,
    /// Actuator forces: `f32` × nv (zero when nu=0).
    pub qfrc_actuator: wgpu::Buffer,
    /// Passive forces (springs/dampers): `f32` × nv (zero when no springs/dampers).
    pub qfrc_passive: wgpu::Buffer,

    // Smooth dynamics (Session 3)
    /// Total smooth force: `f32` × nv.
    pub qfrc_smooth: wgpu::Buffer,
    /// Unconstrained acceleration (M⁻¹ · `qfrc_smooth`): `f32` × nv.
    pub qacc_smooth: wgpu::Buffer,
    /// Final acceleration: `f32` × nv (= `qacc_smooth` for gravity-only; solver for Session 5+).
    pub qacc: wgpu::Buffer,

    // Collision state (Session 4)
    /// Per-geom world-frame AABB: 2×vec4 per geom `[min(xyz,0), max(xyz,0)]`.
    pub geom_aabb: wgpu::Buffer,
    /// Pipeline contacts: `PipelineContact` × `MAX_PIPELINE_CONTACTS`.
    pub contact_buffer: wgpu::Buffer,
    /// Active contact count: `atomic<u32>`.
    pub contact_count: wgpu::Buffer,

    pub n_env: u32,
}

impl GpuStateBuffers {
    /// Allocate per-env state buffers and upload initial qpos.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, data: &Data) -> Self {
        let n_env = 1u32;
        let nbody = u64::from(model.nbody);
        let ngeom = u64::from(model.ngeom);
        let nv = u64::from(model.nv);
        let nmocap = u64::from(model.nmocap.max(1)); // At least 1 for buffer sizing

        let usage_out = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC;
        let usage_in = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
        let usage_inout = usage_out | wgpu::BufferUsages::COPY_DST;

        // Upload initial qpos (f64 → f32)
        let qpos_f32 = f64s_to_f32s(data.qpos.as_slice());
        let qpos = upload_init(
            ctx,
            "qpos",
            bytemuck::cast_slice(&qpos_f32),
            usage_in | wgpu::BufferUsages::COPY_SRC,
        );

        // Body pose outputs: vec4<f32> per body
        let body_xpos = alloc(ctx, "body_xpos", nbody * 16, usage_inout);
        let body_xquat = alloc(ctx, "body_xquat", nbody * 16, usage_inout);
        let body_xipos = alloc(ctx, "body_xipos", nbody * 16, usage_inout);

        // Cinert: 12 floats (3 × vec4) per body
        let body_cinert = alloc(ctx, "body_cinert", nbody * 48, usage_inout);

        // Geom outputs: vec4 for pos, 3×vec4 for mat
        let geom_xpos = alloc(ctx, "geom_xpos", ngeom * 16, usage_inout);
        let geom_xmat = alloc(ctx, "geom_xmat", ngeom * 48, usage_inout);

        // cdof: 8 floats (2 × vec4) per DOF
        let cdof = alloc(ctx, "cdof", nv * 32, usage_inout);

        // Subtree COM
        let subtree_mass = alloc(ctx, "subtree_mass", nbody * 4, usage_inout);
        let subtree_com = alloc(ctx, "subtree_com", nbody * 16, usage_inout);

        // Mocap (uploaded from CPU per frame)
        let mocap_pos = alloc(ctx, "mocap_pos", nmocap * 16, usage_in);
        let mocap_quat = alloc(ctx, "mocap_quat", nmocap * 16, usage_in);

        // CRBA state (Session 2)
        // body_crb: 12 u32 (= 48 bytes) per body — 3×vec4 worth of atomic<u32>
        let body_crb = alloc(ctx, "body_crb", nbody * 48, usage_inout);
        // qM / qM_factor: nv × nv f32 (dense mass matrix)
        let nv_sq = nv.max(1) * nv.max(1);
        let qm = alloc(ctx, "qM", nv_sq * 4, usage_inout);
        let qm_factor = alloc(ctx, "qM_factor", nv_sq * 4, usage_inout);

        // Velocity FK state (Session 2)
        // qvel: nv f32 (uploaded from CPU)
        let qvel_f32 = f64s_to_f32s(data.qvel.as_slice());
        let qvel = upload_init(
            ctx,
            "qvel",
            bytemuck::cast_slice(&qvel_f32),
            usage_in | wgpu::BufferUsages::COPY_SRC,
        );
        // body_cvel: 2 × vec4 per body = 32 bytes/body
        let body_cvel = alloc(ctx, "body_cvel", nbody * 32, usage_inout);

        // RNE state (Session 3)
        // body_cacc: 2 × vec4 per body = 32 bytes/body (matching cvel layout)
        let body_cacc = alloc(ctx, "body_cacc", nbody * 32, usage_inout);
        // body_cfrc: atomic<u32> × 8 per body = 32 bytes/body (6 spatial force + 2 padding)
        let body_cfrc = alloc(ctx, "body_cfrc", nbody * 32, usage_inout);
        // Generalized force/acceleration vectors: nv f32 each
        let nv_bytes = nv.max(1) * 4;
        let qfrc_bias = alloc(ctx, "qfrc_bias", nv_bytes, usage_inout);
        let qfrc_applied = alloc(ctx, "qfrc_applied", nv_bytes, usage_inout);
        let qfrc_actuator = alloc(ctx, "qfrc_actuator", nv_bytes, usage_inout);
        let qfrc_passive = alloc(ctx, "qfrc_passive", nv_bytes, usage_inout);
        let qfrc_smooth = alloc(ctx, "qfrc_smooth", nv_bytes, usage_inout);
        let qacc_smooth = alloc(ctx, "qacc_smooth", nv_bytes, usage_inout);
        let qacc = alloc(ctx, "qacc", nv_bytes, usage_inout);

        // Collision state (Session 4)
        // geom_aabb: 2×vec4 per geom = 32 bytes/geom
        let geom_aabb = alloc(ctx, "geom_aabb", ngeom * 32, usage_inout);
        // contact_buffer: 48 bytes per PipelineContact × max_contacts
        let contact_buffer = alloc(
            ctx,
            "contact_buffer",
            u64::from(MAX_PIPELINE_CONTACTS) * 48,
            usage_inout,
        );
        // contact_count: single atomic u32
        let contact_count = alloc(ctx, "contact_count", 4, usage_inout);

        Self {
            qpos,
            body_xpos,
            body_xquat,
            body_xipos,
            body_cinert,
            geom_xpos,
            geom_xmat,
            cdof,
            subtree_mass,
            subtree_com,
            mocap_pos,
            mocap_quat,
            body_crb,
            qm,
            qm_factor,
            qvel,
            body_cvel,
            body_cacc,
            body_cfrc,
            qfrc_bias,
            qfrc_applied,
            qfrc_actuator,
            qfrc_passive,
            qfrc_smooth,
            qacc_smooth,
            qacc,
            geom_aabb,
            contact_buffer,
            contact_count,
            n_env,
        }
    }

    /// Re-upload qpos from CPU data (e.g., after qpos change).
    pub fn upload_qpos(&self, ctx: &GpuContext, data: &Data) {
        let qpos_f32 = f64s_to_f32s(data.qpos.as_slice());
        ctx.queue
            .write_buffer(&self.qpos, 0, bytemuck::cast_slice(&qpos_f32));
    }

    /// Re-upload qvel from CPU data (e.g., after velocity change).
    pub fn upload_qvel(&self, ctx: &GpuContext, data: &Data) {
        let qvel_f32 = f64s_to_f32s(data.qvel.as_slice());
        if !qvel_f32.is_empty() {
            ctx.queue
                .write_buffer(&self.qvel, 0, bytemuck::cast_slice(&qvel_f32));
        }
    }

    /// Upload mocap poses from CPU data.
    pub fn upload_mocap(&self, ctx: &GpuContext, data: &Data) {
        if !data.mocap_pos.is_empty() {
            let pos: Vec<[f32; 4]> = data
                .mocap_pos
                .iter()
                .map(|v| [v.x as f32, v.y as f32, v.z as f32, 0.0])
                .collect();
            ctx.queue
                .write_buffer(&self.mocap_pos, 0, bytemuck::cast_slice(&pos));
        }
        if !data.mocap_quat.is_empty() {
            let quat: Vec<[f32; 4]> = data
                .mocap_quat
                .iter()
                .map(|q| {
                    let c = q.as_ref().coords;
                    [c.x as f32, c.y as f32, c.z as f32, c.w as f32]
                })
                .collect();
            ctx.queue
                .write_buffer(&self.mocap_quat, 0, bytemuck::cast_slice(&quat));
        }
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

fn alloc(ctx: &GpuContext, label: &str, size: u64, usage: wgpu::BufferUsages) -> wgpu::Buffer {
    // Minimum 16 bytes (wgpu requires non-zero, and some drivers need 16)
    let size = size.max(16);
    ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size,
        usage,
        mapped_at_creation: false,
    })
}

fn upload_init(
    ctx: &GpuContext,
    label: &str,
    data: &[u8],
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    let data = if data.is_empty() { &[0u8; 16] } else { data };
    ctx.device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage,
        })
}
