//! GPU physics pipeline orchestrator — unified struct holding all sub-pipelines.
//!
//! Session 6: Wraps FK, CRBA, velocity FK, RNE, smooth, collision, constraint,
//! and integration into a single `GpuPhysicsPipeline` that records N substeps
//! through a [`Recorder`], which submits them within its pass cap.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use sim_core::types::{Data, MjJointType, Model};

use super::collision::GpuCollisionPipeline;
use super::constraint::GpuConstraintPipeline;
use super::crba::GpuCrbaPipeline;
use super::eulerdamp::GpuEulerdampPipeline;
use super::fk::GpuFkPipeline;
use super::integrate::GpuIntegratePipeline;
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
use super::velocity_fk::GpuVelocityFkPipeline;
use crate::context::{GpuContext, GpuError};
use crate::submit::{Recorder, Recording};

// ── Error type ────────────────────────────────────────────────────────

/// Errors from GPU pipeline creation or operation.
#[derive(Debug)]
pub enum GpuPipelineError {
    /// No GPU device available: the context's own error.
    NoGpu(GpuError),
    /// Model has nv > 60 (exceeds shared memory budget for Newton solver).
    NvTooLarge(usize),
    /// Model contains a non-free joint type.
    UnsupportedJointType(usize, MjJointType),
}

impl std::fmt::Display for GpuPipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoGpu(err) => write!(f, "GPU not available: {err}"),
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

/// Unified GPU physics pipeline holding all sub-pipelines.
///
/// Records N substeps through a [`Recorder`], which submits them within its
/// pass cap, and reads the final qpos/qvel back.
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
    eulerdamp: GpuEulerdampPipeline,
    integrate: GpuIntegratePipeline,

    // Cached model dimensions (single-env counts; per-env buffer strides).
    nq: u32,
    nv: u32,
    // Number of environments stepped in lockstep. 1 = the interactive path
    // (byte-identical to the legacy single-env pipeline); >1 batches every
    // stage across envs via the env-strided shaders.
    n_env: u32,

    /// Whether any DOF has implicit damping. When true, the eulerdamp stage runs
    /// between the constraint stage and integration, solving `(M + h·D)·qacc =
    /// qfrc_smooth − D·q̇ + qfrc_constraint`. When false it is skipped entirely, so
    /// the undamped trajectory is byte-identical to the pre-wiring pipeline.
    has_damping: bool,
}

impl GpuPhysicsPipeline {
    /// Create the GPU physics pipeline for a single (interactive) environment.
    ///
    /// Validates the model (free joints only, nv ≤ 60), uploads static model
    /// data, allocates state buffers, and creates all sub-pipelines. This is a
    /// thin wrapper over [`Self::new_batched`] with `n_env = 1`, so the
    /// trajectory is byte-identical to the legacy single-env pipeline.
    ///
    /// # Errors
    ///
    /// Returns [`GpuPipelineError`] if the model is incompatible or no GPU is available.
    pub fn new(model: &Model, data: &Data) -> Result<Self, GpuPipelineError> {
        Self::new_batched(model, &[data])
    }

    /// Create the GPU physics pipeline for `datas.len()` environments stepped in
    /// lockstep. Every state buffer is sized `n_env ×` its single-env footprint
    /// and every stage dispatches `n_env` workgroup rows; the model is shared
    /// (identical across envs). At `n_env = 1` this is byte-identical to [`Self::new`].
    ///
    /// # Errors
    ///
    /// Returns [`GpuPipelineError`] if the model is incompatible or no GPU is available.
    ///
    /// # Panics
    ///
    /// Panics if `datas` is empty.
    pub fn new_batched(model: &Model, datas: &[&Data]) -> Result<Self, GpuPipelineError> {
        assert!(!datas.is_empty(), "new_batched: datas must be non-empty");
        let n_env = datas.len() as u32;

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
        let ctx = GpuContext::new().map_err(GpuPipelineError::NoGpu)?;

        // ── Upload model + state ──────────────────────────────────
        let model_bufs = GpuModelBuffers::upload(&ctx, model);
        // Single-env keeps the legacy per-env contact capacity; batched uses the
        // smaller per-env budget so the ×n_env buffers stay within GPU limits.
        let max_contacts = if n_env == 1 {
            super::types::MAX_PIPELINE_CONTACTS
        } else {
            super::types::DEFAULT_BATCHED_MAX_CONTACTS
        };
        let state_bufs =
            GpuStateBuffers::new_batched(&ctx, &model_bufs, n_env, datas, max_contacts);

        // ── Create all sub-pipelines ──────────────────────────────
        let fk = GpuFkPipeline::new(&ctx, &model_bufs, &state_bufs);
        let crba = GpuCrbaPipeline::new(&ctx, &model_bufs, &state_bufs);
        let velocity_fk = GpuVelocityFkPipeline::new(&ctx, &model_bufs, &state_bufs);
        let rne = GpuRnePipeline::new(&ctx, &model_bufs, &state_bufs, model);
        let smooth = GpuSmoothPipeline::new(&ctx, &model_bufs, &state_bufs);
        let collision = GpuCollisionPipeline::new(&ctx, model, &model_bufs, &state_bufs);
        let constraint = GpuConstraintPipeline::new(&ctx, &model_bufs, &state_bufs, model);
        let eulerdamp = GpuEulerdampPipeline::new(&ctx, &model_bufs, &state_bufs);
        let integrate = GpuIntegratePipeline::new(&ctx, &model_bufs, &state_bufs);

        // Eulerdamp runs only when the model actually has implicit damping;
        // otherwise it is skipped so the undamped path is byte-identical.
        let has_damping = (0..model.nv).any(|i| model.implicit_damping[i] > 0.0);

        let nq = model.nq as u32;
        let nv = model.nv as u32;

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
            eulerdamp,
            integrate,
            nq,
            nv,
            n_env,
            has_damping,
        })
    }

    /// Run `num_substeps` substeps on GPU for every environment in lockstep.
    /// Uploads each env's current state, records the substeps through a
    /// [`Recorder`], which submits them within its pass cap, and reads the final
    /// per-env state back into `datas`.
    ///
    /// `datas` must hold exactly `n_env` environments (the count fixed at
    /// construction); for the interactive path pass a one-element slice.
    ///
    /// # Panics
    ///
    /// Panics if `datas.len()` does not equal the pipeline's `n_env`.
    pub fn step(&self, cpu_model: &Model, datas: &mut [Data], num_substeps: u32) {
        assert_eq!(
            datas.len(),
            self.n_env as usize,
            "step: got {} envs but pipeline was built for n_env = {}",
            datas.len(),
            self.n_env,
        );

        // 1. Upload per-frame inputs (CPU → GPU), env-strided across all envs.
        let refs: Vec<&Data> = datas.iter().collect();
        self.state_bufs.upload_qpos(&self.ctx, &refs);
        self.state_bufs.upload_qvel(&self.ctx, &refs);

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
        if self.has_damping {
            self.eulerdamp
                .write_params(&self.ctx, &self.model_bufs, cpu_model);
        }

        // 3. Record the substeps, one recorder step each. The recorder submits
        //    within its pass cap; state lives in `state_bufs` across its ordered
        //    submits, so the trajectory does not depend on where they fall (T38).
        let mut rec = Recorder::new(&self.ctx);
        for _ in 0..num_substeps {
            rec.begin_step(&());
            self.encode_substep(&mut rec);
            rec.end_step();
        }

        // 4. Read the final state of every env back, after the last substep.
        let qpos_bytes = u64::from(self.n_env * self.nq) * 4;
        let qvel_bytes = u64::from(self.n_env * self.nv) * 4;
        let read = rec.read_many(&[
            (&self.state_bufs.qpos, qpos_bytes),
            (&self.state_bufs.qvel, qvel_bytes),
        ]);
        let qpos_f32: Vec<f32> = crate::submit::values(&read[0]);
        let qvel_f32: Vec<f32> = crate::submit::values(&read[1]);

        // 5. f32 → f64 widening, write each env's slice back to its Data. Env k's
        //    block sits at offset k · n{q,v}, matching the shaders' env stride.
        let nq = self.nq as usize;
        let nv = self.nv as usize;
        let dt = cpu_model.timestep * f64::from(num_substeps);
        for (k, data) in datas.iter_mut().enumerate() {
            for (i, &v) in qpos_f32[k * nq..(k + 1) * nq].iter().enumerate() {
                data.qpos[i] = f64::from(v);
            }
            for (i, &v) in qvel_f32[k * nv..(k + 1) * nv].iter().enumerate() {
                data.qvel[i] = f64::from(v);
            }
            data.time += dt;
        }
    }

    /// Record one full substep.
    ///
    /// Clears per-substep buffers, then encodes the pipeline stages (eulerdamp
    /// between constraint and integration only when the model has damping).
    fn encode_substep(&self, rec: &mut Recorder) {
        // ── Zero per-substep state buffers ────────────────────────
        rec.commands()
            .clear_buffer(&self.state_bufs.body_cfrc, 0, None);
        rec.commands()
            .clear_buffer(&self.state_bufs.qfrc_bias, 0, None);
        if self.nv > 0 {
            let nv_sq_bytes = u64::from(self.n_env) * u64::from(self.nv) * u64::from(self.nv) * 4;
            rec.commands()
                .clear_buffer(&self.state_bufs.qm, 0, Some(nv_sq_bytes));
            rec.commands()
                .clear_buffer(&self.state_bufs.qfrc_applied, 0, None);
            rec.commands()
                .clear_buffer(&self.state_bufs.qfrc_actuator, 0, None);
            rec.commands()
                .clear_buffer(&self.state_bufs.qfrc_passive, 0, None);
        }

        // ── pipeline stages ──────────────────────────────────────
        self.fk.encode(rec);
        self.crba.encode(rec);
        self.velocity_fk.encode(rec);
        self.rne.encode(rec);
        self.smooth.encode(rec);
        self.collision.encode(rec, &self.state_bufs);
        self.constraint.encode(rec, &self.state_bufs);
        // Implicit joint damping (MuJoCo eulerdamp): re-solve qacc with the damped
        // matrix and the contact-coupled RHS, between the constraint stage (which
        // writes qfrc_constraint) and integration. Skipped for undamped models so
        // their trajectory is byte-identical.
        if self.has_damping {
            self.eulerdamp.encode(rec);
        }
        self.integrate.encode(rec);
    }
}
