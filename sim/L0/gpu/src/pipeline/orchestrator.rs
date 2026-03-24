//! GPU physics pipeline orchestrator — unified struct holding all 8 sub-pipelines.
//!
//! Session 6: Wraps FK, CRBA, velocity FK, RNE, smooth, collision, constraint,
//! and integration into a single `GpuPhysicsPipeline` that encodes N substeps
//! into one command buffer and submits once per frame.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use sim_core::types::{Data, MjJointType, Model};

use super::collision::GpuCollisionPipeline;
use super::constraint::GpuConstraintPipeline;
use super::crba::GpuCrbaPipeline;
use super::fk::GpuFkPipeline;
use super::integrate::GpuIntegratePipeline;
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
use super::velocity_fk::GpuVelocityFkPipeline;
use crate::context::GpuContext;

// ── Error type ────────────────────────────────────────────────────────

/// Errors from GPU pipeline creation or operation.
#[derive(Debug)]
pub enum GpuPipelineError {
    /// No GPU device available.
    NoGpu(String),
    /// Model has nv > 60 (exceeds shared memory budget for Newton solver).
    NvTooLarge(usize),
    /// Model contains a non-free joint type.
    UnsupportedJointType(usize, MjJointType),
}

impl std::fmt::Display for GpuPipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoGpu(msg) => write!(f, "GPU not available: {msg}"),
            Self::NvTooLarge(nv) => write!(f, "nv={nv} exceeds GPU limit of 60"),
            Self::UnsupportedJointType(j, jt) => {
                write!(
                    f,
                    "joint {j} has unsupported type {jt:?} (GPU requires Free)"
                )
            }
        }
    }
}

impl std::error::Error for GpuPipelineError {}

// ── Pipeline ──────────────────────────────────────────────────────────

/// Unified GPU physics pipeline holding all 8 sub-pipelines.
///
/// Encodes N substeps into a single command buffer and submits once per frame.
/// Reads back final qpos/qvel via staging buffers.
pub struct GpuPhysicsPipeline {
    ctx: GpuContext,
    model_bufs: GpuModelBuffers,
    state_bufs: GpuStateBuffers,

    fk: GpuFkPipeline,
    crba: GpuCrbaPipeline,
    velocity_fk: GpuVelocityFkPipeline,
    rne: GpuRnePipeline,
    smooth: GpuSmoothPipeline,
    collision: GpuCollisionPipeline,
    constraint: GpuConstraintPipeline,
    integrate: GpuIntegratePipeline,

    // Staging buffers for readback (MAP_READ | COPY_DST)
    staging_qpos: wgpu::Buffer,
    staging_qvel: wgpu::Buffer,

    // Cached model dimensions
    nq: u32,
    nv: u32,
}

impl GpuPhysicsPipeline {
    /// Create the GPU physics pipeline.
    ///
    /// Validates the model (free joints only, nv ≤ 60), uploads static model
    /// data, allocates state buffers, and creates all 8 sub-pipelines.
    ///
    /// # Errors
    ///
    /// Returns [`GpuPipelineError`] if the model is incompatible or no GPU is available.
    pub fn new(model: &Model, data: &Data) -> Result<Self, GpuPipelineError> {
        // ── Validation ────────────────────────────────────────────
        if model.nv > 60 {
            return Err(GpuPipelineError::NvTooLarge(model.nv));
        }
        for j in 0..model.njnt {
            if model.jnt_type[j] != MjJointType::Free {
                return Err(GpuPipelineError::UnsupportedJointType(j, model.jnt_type[j]));
            }
        }

        // ── GPU context ───────────────────────────────────────────
        let ctx = GpuContext::new().map_err(|e| GpuPipelineError::NoGpu(e.to_string()))?;

        // ── Upload model + state ──────────────────────────────────
        let model_bufs = GpuModelBuffers::upload(&ctx, model);
        let state_bufs = GpuStateBuffers::new(&ctx, &model_bufs, data);

        // ── Create all sub-pipelines ──────────────────────────────
        let fk = GpuFkPipeline::new(&ctx, &model_bufs, &state_bufs);
        let crba = GpuCrbaPipeline::new(&ctx, &model_bufs, &state_bufs);
        let velocity_fk = GpuVelocityFkPipeline::new(&ctx, &model_bufs, &state_bufs);
        let rne = GpuRnePipeline::new(&ctx, &model_bufs, &state_bufs, model);
        let smooth = GpuSmoothPipeline::new(&ctx, &model_bufs, &state_bufs);
        let collision = GpuCollisionPipeline::new(&ctx, model, &model_bufs, &state_bufs);
        let constraint = GpuConstraintPipeline::new(&ctx, &model_bufs, &state_bufs, model);
        let integrate = GpuIntegratePipeline::new(&ctx, &model_bufs, &state_bufs);

        // ── Staging buffers for readback ──────────────────────────
        let nq = model.nq as u32;
        let nv = model.nv as u32;
        let staging_qpos = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging_qpos"),
            size: u64::from(nq.max(1)) * 4,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let staging_qvel = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging_qvel"),
            size: u64::from(nv.max(1)) * 4,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        Ok(Self {
            ctx,
            model_bufs,
            state_bufs,
            fk,
            crba,
            velocity_fk,
            rne,
            smooth,
            collision,
            constraint,
            integrate,
            staging_qpos,
            staging_qvel,
            nq,
            nv,
        })
    }

    /// Run N substeps on GPU. Uploads current state, encodes all substeps
    /// in one command buffer, submits once, reads back final state.
    pub fn step(&self, cpu_model: &Model, data: &mut Data, num_substeps: u32) {
        // 1. Upload per-frame inputs (CPU → GPU)
        self.state_bufs.upload_qpos(&self.ctx, data);
        self.state_bufs.upload_qvel(&self.ctx, data);

        // 2. Write all uniform params (constant across substeps)
        self.fk
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
        self.crba
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
        self.velocity_fk
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
        self.rne
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);
        self.smooth
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);
        self.integrate
            .write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);

        // 3. Encode all substeps into ONE command buffer
        let mut encoder = self
            .ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("gpu_physics_step"),
            });
        for _ in 0..num_substeps {
            self.encode_substep(&mut encoder);
        }

        // 4. Copy final state to staging
        let qpos_size = u64::from(self.nq.max(1)) * 4;
        let qvel_size = u64::from(self.nv.max(1)) * 4;
        encoder.copy_buffer_to_buffer(&self.state_bufs.qpos, 0, &self.staging_qpos, 0, qpos_size);
        encoder.copy_buffer_to_buffer(&self.state_bufs.qvel, 0, &self.staging_qvel, 0, qvel_size);

        // 5. Single submit
        self.ctx.queue.submit([encoder.finish()]);

        // 6. Synchronous readback — map staging buffers directly
        let qpos_f32 = map_staging_f32(&self.ctx, &self.staging_qpos, self.nq as usize);
        let qvel_f32 = map_staging_f32(&self.ctx, &self.staging_qvel, self.nv as usize);

        // 7. f32 → f64 widening, write back to Data
        for (i, &v) in qpos_f32.iter().enumerate() {
            data.qpos[i] = f64::from(v);
        }
        for (i, &v) in qvel_f32.iter().enumerate() {
            data.qvel[i] = f64::from(v);
        }
        data.time += cpu_model.timestep * f64::from(num_substeps);
    }

    /// Encode one full substep into the command encoder.
    ///
    /// Clears per-substep buffers, then encodes all 8 pipeline stages.
    fn encode_substep(&self, encoder: &mut wgpu::CommandEncoder) {
        // ── Zero per-substep state buffers ────────────────────────
        encoder.clear_buffer(&self.state_bufs.body_cfrc, 0, None);
        encoder.clear_buffer(&self.state_bufs.qfrc_bias, 0, None);
        if self.nv > 0 {
            let nv_sq_bytes = u64::from(self.nv) * u64::from(self.nv) * 4;
            encoder.clear_buffer(&self.state_bufs.qm, 0, Some(nv_sq_bytes));
            encoder.clear_buffer(&self.state_bufs.qfrc_applied, 0, None);
            encoder.clear_buffer(&self.state_bufs.qfrc_actuator, 0, None);
            encoder.clear_buffer(&self.state_bufs.qfrc_passive, 0, None);
        }

        // ── 8 pipeline stages ─────────────────────────────────────
        self.fk.encode(encoder);
        self.crba.encode(encoder);
        self.velocity_fk.encode(encoder);
        self.rne.encode(encoder);
        self.smooth.encode(encoder);
        self.collision.encode(encoder, &self.state_bufs);
        self.constraint.encode(encoder, &self.state_bufs);
        self.integrate.encode(encoder);
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

/// Map a staging buffer (`MAP_READ`) and read f32 values from it.
///
/// The buffer must have been filled by a prior `copy_buffer_to_buffer` +
/// `queue.submit()` — this function only maps and reads.
fn map_staging_f32(ctx: &GpuContext, staging: &wgpu::Buffer, count: usize) -> Vec<f32> {
    let slice = staging.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    match ctx.device.poll(wgpu::PollType::Wait {
        submission_index: None,
        timeout: Some(std::time::Duration::from_secs(5)),
    }) {
        Ok(_) => {}
        Err(e) => log::warn!("GPU poll timeout: {e:?}"),
    }
    let data = slice.get_mapped_range();
    let floats: Vec<f32> = bytemuck::cast_slice::<u8, f32>(&data)[..count].to_vec();
    drop(data);
    staging.unmap();
    floats
}
