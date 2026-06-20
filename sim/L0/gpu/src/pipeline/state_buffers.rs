//! Per-environment mutable state buffers for GPU physics pipeline.
//!
//! These hold simulation state. Each environment has its own copy.
//! For `n_env=1` (interactive), all strides are 1× element count.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names, // body_xpos vs body_xipos is intentional (MuJoCo naming)
    clippy::too_many_lines,
    missing_docs,
)]

use sim_core::types::Data;
use wgpu::util::DeviceExt;

use super::model_buffers::GpuModelBuffers;
use super::types::{MAX_CONSTRAINTS, MAX_PIPELINE_CONTACTS, f64s_to_f32s};
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
    /// Scratch Cholesky factor of `(M + h·D)` for the eulerdamp solve: `f32` x
    /// nv x nv. Kept separate from `qm_factor` so the constraint path's mass
    /// matrix stays the pure `M`.
    pub qm_eulerdamp_factor: wgpu::Buffer,

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

    // Constraint solve state (Session 5)
    /// Constraint Jacobian: `f32` × `max_constraints` × nv (row-major).
    pub efc_j: wgpu::Buffer,
    /// Constraint stiffness: `f32` × `max_constraints`.
    pub efc_d: wgpu::Buffer,
    /// Reference acceleration: `f32` × `max_constraints`.
    pub efc_aref: wgpu::Buffer,
    /// Solver output force: `f32` × `max_constraints`.
    pub efc_force: wgpu::Buffer,
    /// Active constraint row count: `atomic<u32>`.
    pub constraint_count: wgpu::Buffer,
    /// Constraint force in joint space: `f32` × nv.
    pub qfrc_constraint: wgpu::Buffer,

    pub n_env: u32,
}

impl GpuStateBuffers {
    /// Allocate per-env state buffers for a single environment and upload its
    /// initial qpos/qvel. Convenience wrapper over [`Self::new_batched`] for the
    /// interactive (`n_env = 1`) path — byte-identical to the old single-env
    /// allocator, so every existing caller is unaffected.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &GpuModelBuffers, data: &Data) -> Self {
        Self::new_batched(ctx, model, 1, &[data])
    }

    /// Allocate per-env state buffers for `n_env` environments and upload each
    /// env's initial qpos/qvel.
    ///
    /// Every mutable state buffer is sized `n_env ×` its single-env footprint and
    /// the env-striding shaders address env `k` at offset `k · count` (e.g.
    /// `env_id * nbody` for body buffers). The shared [`GpuModelBuffers`]
    /// (joint/dof/geom parameters) are NOT duplicated — they are identical across
    /// environments. The two atomic counters (`contact_count`/`constraint_count`)
    /// are also sized per-env so the collision/constraint batching slice (PR D)
    /// need not revisit the allocator.
    ///
    /// `per_env[k]` supplies env `k`'s initial `qpos`/`qvel`; its length must
    /// equal `n_env`. At `n_env = 1` this is byte-identical to the legacy
    /// single-env layout (every stride collapses to `1×`).
    ///
    /// # Panics
    /// Panics if `per_env.len() != n_env`.
    #[must_use]
    pub fn new_batched(
        ctx: &GpuContext,
        model: &GpuModelBuffers,
        n_env: u32,
        per_env: &[&Data],
    ) -> Self {
        assert_eq!(
            per_env.len(),
            n_env as usize,
            "new_batched: per_env has {} entries but n_env = {n_env}",
            per_env.len(),
        );
        let ne = u64::from(n_env);
        // These are the BATCHED totals (×n_env), used to size every per-env buffer.
        // Do NOT confuse with the shaders' `params.nv`/`params.nbody`, which are the
        // SINGLE-env counts used for per-row/per-body strides (env offset is then
        // `env_id * params.nv` etc.). A buffer's byte size is `ne · single-env`, so
        // here `nv = ne · model.nv`; the per-env stride lives in the shader.
        let nbody = ne * u64::from(model.nbody);
        let ngeom = ne * u64::from(model.ngeom);
        let nv = ne * u64::from(model.nv);
        let nmocap = ne * u64::from(model.nmocap.max(1)); // At least 1 for buffer sizing

        let usage_out = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC;
        let usage_in = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
        let usage_inout = usage_out | wgpu::BufferUsages::COPY_DST;

        // Upload initial qpos (f64 → f32), env states concatenated so env k lands
        // at byte offset k · nq · 4 — the layout fk.wgsl reads via `env_id * nq`.
        let qpos_f32 = concat_env_state(per_env, |d| d.qpos.as_slice());
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

        // Mocap (uploaded from CPU per frame). Sized ×n_env for layout uniformity,
        // but the FK shader does NOT env-stride mocap reads yet (it indexes
        // `mocap_pos[mocap_id]`, env 0 only); per-env mocap lands with the
        // collision/constraint batching slice. No conformance fixture uses mocap.
        let mocap_pos = alloc(ctx, "mocap_pos", nmocap * 16, usage_in);
        let mocap_quat = alloc(ctx, "mocap_quat", nmocap * 16, usage_in);

        // CRBA state (Session 2)
        // body_crb: 12 u32 (= 48 bytes) per body — 3×vec4 worth of atomic<u32>
        let body_crb = alloc(ctx, "body_crb", nbody * 48, usage_inout);
        // qM / qM_factor: nv × nv f32 (dense mass matrix), per env.
        let nv1 = u64::from(model.nv).max(1);
        let nv_sq = ne * nv1 * nv1;
        let qm = alloc(ctx, "qM", nv_sq * 4, usage_inout);
        let qm_factor = alloc(ctx, "qM_factor", nv_sq * 4, usage_inout);
        let qm_eulerdamp_factor = alloc(ctx, "qM_eulerdamp_factor", nv_sq * 4, usage_inout);

        // Velocity FK state (Session 2)
        // qvel: nv f32 per env (uploaded from CPU), env-concatenated like qpos.
        let qvel_f32 = concat_env_state(per_env, |d| d.qvel.as_slice());
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
        // Generalized force/acceleration vectors: nv f32 each, per env.
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
        // contact_buffer: 48 bytes per PipelineContact × max_contacts, per env.
        let contact_buffer = alloc(
            ctx,
            "contact_buffer",
            ne * u64::from(MAX_PIPELINE_CONTACTS) * 48,
            usage_inout,
        );
        // contact_count: one atomic u32 per env.
        let contact_count = alloc(ctx, "contact_count", ne * 4, usage_inout);

        // Constraint solve state (Session 5), per env.
        let max_c = u64::from(MAX_CONSTRAINTS);
        let efc_j = alloc(ctx, "efc_J", ne * max_c * nv1 * 4, usage_inout);
        let efc_d = alloc(ctx, "efc_D", ne * max_c * 4, usage_inout);
        let efc_aref = alloc(ctx, "efc_aref", ne * max_c * 4, usage_inout);
        let efc_force = alloc(ctx, "efc_force", ne * max_c * 4, usage_inout);
        let constraint_count = alloc(ctx, "constraint_count", ne * 4, usage_inout);
        let qfrc_constraint = alloc(ctx, "qfrc_constraint", nv_bytes, usage_inout);

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
            qm_eulerdamp_factor,
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
            efc_j,
            efc_d,
            efc_aref,
            efc_force,
            constraint_count,
            qfrc_constraint,
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

/// Concatenate one per-env f64 state slice (e.g. `qpos`/`qvel`) into a single
/// f32 buffer, env 0 first. Env `k`'s block lands at offset `k · len`, matching
/// the `env_id * n{q,v}` stride the shaders use. At `n_env = 1` this is exactly
/// the single env's `f64s_to_f32s`, so the legacy layout is preserved.
fn concat_env_state(per_env: &[&Data], pick: impl Fn(&Data) -> &[f64]) -> Vec<f32> {
    let mut out = Vec::new();
    for data in per_env {
        out.extend(f64s_to_f32s(pick(data)));
    }
    out
}

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
