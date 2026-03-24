//! GPU-side types for the physics pipeline.
//!
//! All types are `#[repr(C)]`, `Pod`, `Zeroable` for direct GPU upload
//! via `bytemuck`. Layouts match the WGSL shader structs exactly.
//!
//! # Storage buffer limit
//!
//! `WebGPU` default allows 8 storage buffers per shader stage. To stay
//! practical, we pack per-body and per-joint model data into struct
//! arrays (`BodyModelGpu`, `JointModelGpu`, `GeomModelGpu`) instead of
//! individual buffers. This reduces Group 1 (static model) from 18
//! buffers to 2, keeping the total per-stage count at ~12.

#![allow(
    clippy::cast_possible_truncation, // f64→f32 intentional (GPU uses f32)
    clippy::cast_precision_loss,
    clippy::pub_underscore_fields, // _pad fields required for GPU struct alignment
    missing_docs,
)]

use bytemuck::{Pod, Zeroable};
use sim_core::types::MjJointType;

// ── Joint type enum (GPU mapping) ─────────────────────────────────────

pub const GPU_JNT_FREE: u32 = 0;
pub const GPU_JNT_BALL: u32 = 1;
pub const GPU_JNT_HINGE: u32 = 2;
pub const GPU_JNT_SLIDE: u32 = 3;

/// Convert CPU joint type to GPU enum value.
#[must_use]
pub const fn jnt_type_to_gpu(jtype: MjJointType) -> u32 {
    match jtype {
        MjJointType::Free => GPU_JNT_FREE,
        MjJointType::Ball => GPU_JNT_BALL,
        MjJointType::Hinge => GPU_JNT_HINGE,
        MjJointType::Slide => GPU_JNT_SLIDE,
    }
}

/// Sentinel value for `body_mocap_id` indicating "not a mocap body".
pub const MOCAP_NONE: u32 = 0xFFFF_FFFF;

/// Sentinel value for `DofModelGpu::parent` indicating no parent DOF.
pub const DOF_PARENT_NONE: u32 = 0xFFFF_FFFF;

// ── Packed GPU structs ────────────────────────────────────────────────

/// Per-body static model data. 112 bytes, 16-byte aligned.
///
/// Layout matches WGSL `BodyModel` exactly.
/// ```text
/// offset  0: parent, depth, jnt_adr, jnt_num       (4 × u32 = 16 bytes)
/// offset 16: dof_adr, dof_num, mocap_id, _pad       (4 × u32 = 16 bytes)
/// offset 32: pos                                     (vec4<f32> = 16 bytes)
/// offset 48: quat                                    (vec4<f32> = 16 bytes)
/// offset 64: ipos                                    (vec4<f32> = 16 bytes)
/// offset 80: iquat                                   (vec4<f32> = 16 bytes)
/// offset 96: inertia  (xyz = diagonal, w = mass)     (vec4<f32> = 16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct BodyModelGpu {
    pub parent: u32,
    pub depth: u32,
    pub jnt_adr: u32,
    pub jnt_num: u32,
    pub dof_adr: u32,
    pub dof_num: u32,
    pub mocap_id: u32,
    pub _pad: u32,
    pub pos: [f32; 4],
    pub quat: [f32; 4],
    pub ipos: [f32; 4],
    pub iquat: [f32; 4],
    /// xyz = diagonal inertia in body frame, w = body mass.
    pub inertia: [f32; 4],
}

/// Per-joint static model data. 48 bytes, 16-byte aligned.
///
/// Layout matches WGSL `JointModel` exactly.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct JointModelGpu {
    pub jtype: u32,
    pub qpos_adr: u32,
    pub dof_adr: u32,
    pub _pad: u32,
    pub axis: [f32; 4],
    pub pos: [f32; 4],
}

/// Per-geom static model data. 48 bytes, 16-byte aligned.
///
/// Layout matches WGSL `GeomModel` exactly.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct GeomModelGpu {
    pub body_id: u32,
    pub _pad: [u32; 3],
    pub pos: [f32; 4],
    pub quat: [f32; 4],
}

/// Per-DOF static model data. 16 bytes, 16-byte aligned.
///
/// Layout matches WGSL `DofModel` exactly.
/// ```text
/// offset 0: body_id, parent, armature, _pad  (2 × u32 + f32 + u32 = 16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct DofModelGpu {
    /// Body that owns this DOF (`model.dof_body[i]`).
    pub body_id: u32,
    /// Parent DOF in kinematic tree (`DOF_PARENT_NONE` if root DOF).
    pub parent: u32,
    /// Combined armature: `jnt_armature[jnt] + dof_armature[i]`.
    pub armature: f32,
    pub _pad: u32,
}

/// Per-dispatch FK parameters. 32 bytes, 16-byte aligned.
///
/// Written into a pre-allocated uniform buffer with 256-byte slots.
/// Selected via dynamic offset per depth level.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct FkParams {
    pub current_depth: u32,
    pub nbody: u32,
    pub njnt: u32,
    pub ngeom: u32,
    pub nv: u32,
    pub n_env: u32,
    pub nq: u32,
    pub _pad: u32,
}

// ── Conversion helpers ────────────────────────────────────────────────

/// Convert `Vec<f64>` to `Vec<f32>`.
#[must_use]
pub fn f64s_to_f32s(vals: &[f64]) -> Vec<f32> {
    vals.iter().map(|&v| v as f32).collect()
}
