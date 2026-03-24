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
use sim_core::types::{GeomType, MjJointType};

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

// ── Geom type enum (GPU mapping) ──────────────────────────────────────

pub const GPU_GEOM_PLANE: u32 = 0;
pub const GPU_GEOM_SPHERE: u32 = 1;
pub const GPU_GEOM_CAPSULE: u32 = 2;
pub const GPU_GEOM_CYLINDER: u32 = 3;
pub const GPU_GEOM_BOX: u32 = 4;
pub const GPU_GEOM_ELLIPSOID: u32 = 5;
pub const GPU_GEOM_MESH: u32 = 6;
pub const GPU_GEOM_HFIELD: u32 = 7;
pub const GPU_GEOM_SDF: u32 = 8;

/// Convert CPU geom type to GPU enum value.
#[must_use]
pub const fn geom_type_to_gpu(gtype: GeomType) -> u32 {
    match gtype {
        GeomType::Plane => GPU_GEOM_PLANE,
        GeomType::Sphere => GPU_GEOM_SPHERE,
        GeomType::Capsule => GPU_GEOM_CAPSULE,
        GeomType::Cylinder => GPU_GEOM_CYLINDER,
        GeomType::Box => GPU_GEOM_BOX,
        GeomType::Ellipsoid => GPU_GEOM_ELLIPSOID,
        GeomType::Mesh => GPU_GEOM_MESH,
        GeomType::Hfield => GPU_GEOM_HFIELD,
        GeomType::Sdf => GPU_GEOM_SDF,
    }
}

// ── Sentinel values ──────────────────────────────────────────────────

/// Sentinel value for `body_mocap_id` indicating "not a mocap body".
pub const MOCAP_NONE: u32 = 0xFFFF_FFFF;

/// Sentinel value for `DofModelGpu::parent` indicating no parent DOF.
pub const DOF_PARENT_NONE: u32 = 0xFFFF_FFFF;

/// Sentinel value for `GeomModelGpu::sdf_meta_idx` indicating "not an SDF geom".
pub const SDF_META_NONE: u32 = 0xFFFF_FFFF;

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
/// ```text
/// offset  0: jtype, qpos_adr, dof_adr, body_id   (4 × u32 = 16 bytes)
/// offset 16: axis                                   (vec4<f32> = 16 bytes)
/// offset 32: pos                                    (vec4<f32> = 16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct JointModelGpu {
    pub jtype: u32,
    pub qpos_adr: u32,
    pub dof_adr: u32,
    /// Body that owns this joint (`model.jnt_body[j]`).
    pub body_id: u32,
    pub axis: [f32; 4],
    pub pos: [f32; 4],
}

/// Per-geom static model data. 96 bytes, 16-byte aligned.
///
/// Layout matches WGSL `GeomModel` exactly.
/// ```text
/// offset  0: body_id, geom_type, contype, conaffinity  (4 × u32 = 16 bytes)
/// offset 16: pos                                         (vec4<f32> = 16 bytes)
/// offset 32: quat                                        (vec4<f32> = 16 bytes)
/// offset 48: size  (xyz = dims, w = rbound)              (vec4<f32> = 16 bytes)
/// offset 64: friction  (x=slide, y=torsion, z=roll, w=0) (vec4<f32> = 16 bytes)
/// offset 80: sdf_meta_idx, condim, _pad                  (4 × u32 = 16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct GeomModelGpu {
    pub body_id: u32,
    pub geom_type: u32,
    pub contype: u32,
    pub conaffinity: u32,
    pub pos: [f32; 4],
    pub quat: [f32; 4],
    /// xyz = type-specific dimensions, w = bounding radius.
    pub size: [f32; 4],
    /// x = sliding, y = torsional, z = rolling, w = 0.
    pub friction: [f32; 4],
    /// Index into `sdf_metas` array (`SDF_META_NONE` if not SDF).
    pub sdf_meta_idx: u32,
    /// Contact dimensionality (1, 3, 4, or 6).
    pub condim: u32,
    pub _pad: [u32; 2],
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

/// Per-dispatch dynamics parameters. 48 bytes, 16-byte aligned.
///
/// Used by RNE, smooth, and integrate shaders. Contains gravity and
/// timestep not present in `FkParams` (which serves kinematics only).
/// Written into a pre-allocated uniform buffer with 256-byte slots.
/// ```text
/// offset  0: gravity                                 (vec4<f32> = 16 bytes)
/// offset 16: timestep, nbody, njnt, nv               (4 × f32/u32 = 16 bytes)
/// offset 32: nq, n_env, current_depth, nu             (4 × u32 = 16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct PhysicsParams {
    /// Gravity vector (xyz, w=0).
    pub gravity: [f32; 4],
    pub timestep: f32,
    pub nbody: u32,
    pub njnt: u32,
    pub nv: u32,
    pub nq: u32,
    pub n_env: u32,
    pub current_depth: u32,
    pub nu: u32,
}

// ── SDF metadata (Session 4 — collision pipeline) ───────────────────

/// Per-shape SDF grid metadata. 32 bytes, 16-byte aligned.
///
/// Points into the unified `sdf_values` buffer via `values_offset`.
/// Layout matches WGSL `SdfMeta` exactly.
/// ```text
/// offset  0: width, height, depth, cell_size  (3 × u32 + f32 = 16 bytes)
/// offset 16: origin (vec3<f32>), values_offset (u32)  (16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct SdfMetaGpu {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
    pub cell_size: f32,
    pub origin: [f32; 3],
    /// Offset into the unified `sdf_values` buffer (element index, not byte offset).
    pub values_offset: u32,
}

/// Pipeline contact output. 48 bytes, 16-byte aligned.
///
/// Extended from the standalone `GpuContact` (32 bytes) with geom
/// indices and combined friction for constraint assembly (Session 5).
/// Layout matches WGSL `PipelineContact` exactly.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct PipelineContact {
    /// World-space contact position.
    pub point: [f32; 3],
    /// Penetration depth (≥ 0).
    pub depth: f32,
    /// World-space contact normal (geom1 → geom2).
    pub normal: [f32; 3],
    /// Geom index A.
    pub geom1: u32,
    /// Combined friction [slide, torsion, roll].
    pub friction: [f32; 3],
    /// Geom index B.
    pub geom2: u32,
}

/// Per-pair narrowphase dispatch parameters. 48 bytes, 16-byte aligned.
///
/// Written per narrowphase dispatch. The shader reads geom poses from
/// `geom_xpos`/`geom_xmat` directly (no pose matrices in params).
/// ```text
/// offset  0: geom1, geom2, src_sdf_meta_idx, dst_sdf_meta_idx  (4 × u32 = 16 bytes)
/// offset 16: surface_threshold, contact_margin, flip_normal, _pad  (16 bytes)
/// offset 32: friction (vec4<f32>)  (16 bytes)
/// ```
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct NarrowphaseParams {
    /// Source geom index (the SDF geom being traced).
    pub geom1: u32,
    /// Destination geom index (SDF or plane).
    pub geom2: u32,
    /// Index into `sdf_metas` for source SDF grid.
    pub src_sdf_meta_idx: u32,
    /// Index into `sdf_metas` for destination SDF grid (`SDF_META_NONE` for plane).
    pub dst_sdf_meta_idx: u32,
    /// Surface band filter: `src_cell_size * 2.0`.
    pub surface_threshold: f32,
    /// Contact detection range.
    pub contact_margin: f32,
    /// Normal convention: 0 = A→B (negate), 1 = B→A (keep).
    pub flip_normal: u32,
    pub _pad: u32,
    /// Combined friction [slide, torsion, roll, 0].
    pub friction: [f32; 4],
}

/// Maximum pre-allocated pipeline contacts.
pub const MAX_PIPELINE_CONTACTS: u32 = 32_768;

/// Maximum pre-allocated constraint rows (6 × `MAX_PIPELINE_CONTACTS`).
///
/// Worst case: every contact has condim=4 → 6 pyramidal rows each.
pub const MAX_CONSTRAINTS: u32 = 196_608;

// ── Constraint solve params (Session 5) ─────────────────────────────

/// Constraint assembly parameters. 64 bytes, 16-byte aligned.
///
/// Passed as a uniform to `assemble.wgsl`. Contains model-level defaults
/// for solref/solimp (per-contact overrides are not yet supported on GPU).
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct AssemblyParams {
    pub nv: u32,
    pub max_contacts: u32,
    pub max_constraints: u32,
    pub nbody: u32,
    pub timestep: f32,
    pub impratio: f32,
    /// Default solref: timeconst (standard mode).
    pub solref_timeconst: f32,
    /// Default solref: dampratio (standard mode).
    pub solref_dampratio: f32,
    /// Default solimp: impedance at zero violation.
    pub solimp_d0: f32,
    /// Default solimp: impedance at full violation.
    pub solimp_dwidth: f32,
    /// Default solimp: transition zone width (meters).
    pub solimp_width: f32,
    /// Default solimp: sigmoid midpoint.
    pub solimp_midpoint: f32,
    /// Default solimp: sigmoid power.
    pub solimp_power: f32,
    pub _pad: [f32; 3],
}

/// Newton solver parameters. 32 bytes, 16-byte aligned.
///
/// Passed as a uniform to `newton_solve.wgsl` and `map_forces.wgsl`.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct SolverParams {
    pub nv: u32,
    pub max_iter: u32,
    pub max_ls: u32,
    pub _pad0: u32,
    pub tolerance: f32,
    pub ls_tolerance: f32,
    pub meaninertia: f32,
    pub _pad1: f32,
}

// ── Conversion helpers ────────────────────────────────────────────────

/// Convert `Vec<f64>` to `Vec<f32>`.
#[must_use]
pub fn f64s_to_f32s(vals: &[f64]) -> Vec<f32> {
    vals.iter().map(|&v| v as f32).collect()
}
