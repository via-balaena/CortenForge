//! GPU pipeline validation tests.
//!
//! T1–T7:  FK — compare GPU body poses, cinert, cdof, subtree COM against CPU.
//! T8–T12: CRBA + velocity FK — compare GPU qM, cvel against CPU.
//! T13–T18: RNE + smooth + integration — bias forces, qacc_smooth, trajectories.
//! T19–T22: Collision — AABB, SDF-SDF, SDF-plane, full collision pipeline.
//! T23–T27: Constraint solve — assembly, Newton solver, force mapping, stability.

#![cfg(test)]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::doc_markdown,
    // `!(diff < tol)` is intentional: it treats NaN as a failure (unlike `>=`).
    clippy::neg_cmp_op_on_partial_ord
)]

// CI safety scanner: this file is test-only (gated by #[cfg(test)] in mod.rs).
#[cfg(test)]
const _: () = ();

use nalgebra::{UnitQuaternion, Vector3};
use std::f64::consts::PI;

use sim_core::types::{Data, MjJointType, Model};

use super::collision::GpuCollisionPipeline;
use super::constraint::GpuConstraintPipeline;
use super::crba::GpuCrbaPipeline;
use super::fk::{GpuFkPipeline, readback_f32s, readback_vec4s};
use super::integrate::GpuIntegratePipeline;
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
use super::types::PipelineContact;
use super::velocity_fk::GpuVelocityFkPipeline;
use crate::context::GpuContext;

// ── Helpers ───────────────────────────────────────────────────────────

/// Try to create GPU context; skip test if no GPU available.
macro_rules! gpu_or_skip {
    () => {
        match GpuContext::new() {
            Ok(ctx) => ctx,
            Err(e) => {
                eprintln!("  Skipping (no GPU): {e}");
                return;
            }
        }
    };
}

/// Compare GPU vec4 readback against CPU Vector3 values (xyz only).
fn compare_xpos(
    ctx: &GpuContext,
    buffer: &wgpu::Buffer,
    cpu_values: &[Vector3<f64>],
    label: &str,
    tol: f32,
) {
    let gpu = readback_vec4s(ctx, buffer, cpu_values.len());
    for (i, (g, c)) in gpu.iter().zip(cpu_values.iter()).enumerate() {
        let dx = (g[0] - c.x as f32).abs();
        let dy = (g[1] - c.y as f32).abs();
        let dz = (g[2] - c.z as f32).abs();
        let max_err = dx.max(dy).max(dz);
        assert!(
            max_err < tol,
            "{label}[{i}]: GPU=({:.6},{:.6},{:.6}) CPU=({:.6},{:.6},{:.6}) err={max_err:.2e}",
            g[0],
            g[1],
            g[2],
            c.x,
            c.y,
            c.z
        );
    }
}

/// Compare GPU vec4 readback against CPU UnitQuaternion values (xyzw).
fn compare_xquat(
    ctx: &GpuContext,
    buffer: &wgpu::Buffer,
    cpu_values: &[UnitQuaternion<f64>],
    label: &str,
    tol: f32,
) {
    let gpu = readback_vec4s(ctx, buffer, cpu_values.len());
    for (i, (g, c)) in gpu.iter().zip(cpu_values.iter()).enumerate() {
        let cc = c.as_ref().coords;
        // Quaternions q and -q represent the same rotation
        let sign = if g[3] * cc.w as f32 >= 0.0 {
            1.0f32
        } else {
            -1.0
        };
        let dx = (g[0] - sign * cc.x as f32).abs();
        let dy = (g[1] - sign * cc.y as f32).abs();
        let dz = (g[2] - sign * cc.z as f32).abs();
        let dw = (g[3] - sign * cc.w as f32).abs();
        let max_err = dx.max(dy).max(dz).max(dw);
        assert!(
            max_err < tol,
            "{label}[{i}]: GPU=({:.6},{:.6},{:.6},{:.6}) CPU=({:.6},{:.6},{:.6},{:.6}) err={max_err:.2e}",
            g[0],
            g[1],
            g[2],
            g[3],
            cc.x,
            cc.y,
            cc.z,
            cc.w
        );
    }
}

// ── T1: Flat tree — free body ─────────────────────────────────────────

#[test]
fn t1_free_body_poses() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    // Set position to (1, 2, 3) and a non-trivial rotation
    data.qpos[0] = 1.0;
    data.qpos[1] = 2.0;
    data.qpos[2] = 3.0;
    // Quaternion (w, x, y, z) = normalized (0.5, 0.5, 0.5, 0.5)
    let q = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(0.5, 0.5, 0.5, 0.5));
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    // CPU FK
    data.forward(&model).expect("CPU forward failed");

    // GPU FK
    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t1") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    // Compare
    compare_xpos(&ctx, &state_buf.body_xpos, &data.xpos, "xpos", 1e-5);
    compare_xquat(&ctx, &state_buf.body_xquat, &data.xquat, "xquat", 1e-5);
    eprintln!("  T1 passed: free body xpos/xquat match CPU");
}

// ── T2: Deep chain — 7-body hinge chain ───────────────────────────────

#[test]
fn t2_deep_hinge_chain() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(7, 1.0, 1.0);
    let mut data = model.make_data();
    // Set various hinge angles
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = -PI / 6.0;
    data.qpos[2] = PI / 3.0;
    data.qpos[3] = 0.0;
    data.qpos[4] = PI / 2.0;
    data.qpos[5] = -PI / 4.0;
    data.qpos[6] = PI / 8.0;

    // CPU FK
    data.forward(&model).expect("CPU forward failed");

    // GPU FK
    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t2") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    // Looser tolerance for 7-level chain (f32 accumulation)
    compare_xpos(&ctx, &state_buf.body_xpos, &data.xpos, "xpos", 1e-4);
    compare_xquat(&ctx, &state_buf.body_xquat, &data.xquat, "xquat", 1e-4);
    eprintln!("  T2 passed: 7-body hinge chain xpos/xquat match CPU");
}

// ── T3: Mixed joints — free + hinge ──────────────────────────────────

#[test]
fn t3_free_body_at_offset() {
    let ctx = gpu_or_skip!();

    // Free body at a specific position with non-trivial quaternion
    let model = Model::free_body(2.5, Vector3::new(0.5, 0.5, 0.5));
    let mut data = model.make_data();
    data.qpos[0] = 5.0;
    data.qpos[1] = -3.0;
    data.qpos[2] = 7.0;
    // Rotation: 45° around Z
    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0);
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t3") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    compare_xpos(&ctx, &state_buf.body_xpos, &data.xpos, "xpos", 1e-5);
    compare_xquat(&ctx, &state_buf.body_xquat, &data.xquat, "xquat", 1e-5);
    eprintln!("  T3 passed: free body at offset matches CPU");
}

// ── T4: Cinert validation ─────────────────────────────────────────────

#[test]
fn t4_cinert_matches_cpu() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    data.qpos[0] = 1.0;
    data.qpos[1] = 2.0;
    data.qpos[2] = 3.0;
    let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 3.0);
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t4") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    // Readback cinert (3 vec4 per body)
    let gpu_cinert = readback_vec4s(&ctx, &state_buf.body_cinert, model.nbody * 3);

    // Compare each body's cinert against CPU
    for body_id in 0..model.nbody {
        let base = body_id * 3;
        let gpu_mass = gpu_cinert[base][0];
        let gpu_h = [
            gpu_cinert[base][1],
            gpu_cinert[base][2],
            gpu_cinert[base][3],
        ];
        let gpu_i = [
            gpu_cinert[base + 1][0],
            gpu_cinert[base + 1][1],
            gpu_cinert[base + 1][2],
            gpu_cinert[base + 1][3],
            gpu_cinert[base + 2][0],
            gpu_cinert[base + 2][1],
        ];

        // Reconstruct CPU cinert compact form for comparison
        let cpu = &data.cinert[body_id];
        let cpu_mass = cpu[(3, 3)]; // lower-right diagonal = mass
        let cpu_h = Vector3::new(
            data.xipos[body_id].x - data.xpos[body_id].x,
            data.xipos[body_id].y - data.xpos[body_id].y,
            data.xipos[body_id].z - data.xpos[body_id].z,
        );

        // Extract rotational inertia about COM from CPU cinert
        // CPU cinert upper-left 3×3 = I_rot + m*(|h|²I - h⊗h)  [parallel axis]
        // GPU stores I_COM directly (pre-parallel-axis)
        // So extract I_COM = CPU_upper_left - m*(|h|²I - h⊗h)
        let hh = cpu_h.dot(&cpu_h);
        let cpu_i_com_00 = cpu[(0, 0)] - cpu_mass * (hh - cpu_h.x * cpu_h.x);
        let cpu_i_com_01 = cpu[(0, 1)] - cpu_mass * (0.0 - cpu_h.x * cpu_h.y);
        let cpu_i_com_02 = cpu[(0, 2)] - cpu_mass * (0.0 - cpu_h.x * cpu_h.z);
        let cpu_i_com_11 = cpu[(1, 1)] - cpu_mass * (hh - cpu_h.y * cpu_h.y);
        let cpu_i_com_12 = cpu[(1, 2)] - cpu_mass * (0.0 - cpu_h.y * cpu_h.z);
        let cpu_i_com_22 = cpu[(2, 2)] - cpu_mass * (hh - cpu_h.z * cpu_h.z);

        let tol = 1e-4;
        assert!(
            (gpu_mass - cpu_mass as f32).abs() < tol,
            "body {body_id} mass: GPU={gpu_mass:.6} CPU={cpu_mass:.6}",
        );

        for k in 0..3 {
            let g = gpu_h[k];
            let c = [cpu_h.x, cpu_h.y, cpu_h.z][k] as f32;
            assert!(
                (g - c).abs() < tol,
                "body {body_id} h[{k}]: GPU={g:.6} CPU={c:.6}"
            );
        }

        let cpu_i_vec = [
            cpu_i_com_00,
            cpu_i_com_01,
            cpu_i_com_02,
            cpu_i_com_11,
            cpu_i_com_12,
            cpu_i_com_22,
        ];
        for k in 0..6 {
            let g = gpu_i[k];
            let c = cpu_i_vec[k] as f32;
            assert!(
                (g - c).abs() < tol,
                "body {body_id} I[{k}]: GPU={g:.6} CPU={c:.6}"
            );
        }
    }
    eprintln!("  T4 passed: cinert matches CPU");
}

// ── T5: cdof validation ───────────────────────────────────────────────

/// Compute hinge cdof on CPU: [axis; cross(axis, body_origin - anchor)]
fn cpu_hinge_cdof(model: &Model, data: &Data, jnt_id: usize) -> [f32; 6] {
    let body_id = model.jnt_body[jnt_id];
    let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
    let anchor_world = data.xpos[body_id] + data.xquat[body_id] * model.jnt_pos[jnt_id];
    let r = data.xpos[body_id] - anchor_world;
    let lin = axis_world.cross(&r);
    [
        axis_world.x as f32,
        axis_world.y as f32,
        axis_world.z as f32,
        lin.x as f32,
        lin.y as f32,
        lin.z as f32,
    ]
}

#[test]
fn t5_cdof_matches_cpu() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(3, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = PI / 6.0;
    data.qpos[2] = -PI / 3.0;

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t5") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    // Readback cdof (2 vec4 per DOF)
    let gpu_cdof = readback_vec4s(&ctx, &state_buf.cdof, model.nv * 2);

    // All 3 DOFs are hinge joints
    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let cpu = cpu_hinge_cdof(&model, &data, jnt_id);

        let gpu_ang = &gpu_cdof[dof * 2];
        let gpu_lin = &gpu_cdof[dof * 2 + 1];

        let tol = 1e-4;
        for r in 0..3 {
            let g = gpu_ang[r];
            let c = cpu[r];
            assert!(
                (g - c).abs() < tol,
                "dof {dof} angular[{r}]: GPU={g:.6} CPU={c:.6}"
            );
        }
        for r in 0..3 {
            let g = gpu_lin[r];
            let c = cpu[r + 3];
            assert!(
                (g - c).abs() < tol,
                "dof {dof} linear[{r}]: GPU={g:.6} CPU={c:.6}"
            );
        }
    }
    eprintln!("  T5 passed: cdof matches CPU for 3-link pendulum");
}

/// Expected hinge/slide `cdof` from the CPU's stored PARTIAL frames
/// (`data.xaxis`/`data.xanchor`) — the exact source `joint_motion_subspace`
/// consumes. Unlike `cpu_hinge_cdof` (which rebuilds from the final `xquat`),
/// this is correct for a multi-joint body where later joints rotate the frame.
fn cpu_cdof_partial(model: &Model, data: &Data, jnt_id: usize) -> [f32; 6] {
    let body_id = model.jnt_body[jnt_id];
    let axis = data.xaxis[jnt_id];
    match model.jnt_type[jnt_id] {
        MjJointType::Hinge => {
            let r = data.xpos[body_id] - data.xanchor[jnt_id];
            let lin = axis.cross(&r);
            [
                axis.x as f32,
                axis.y as f32,
                axis.z as f32,
                lin.x as f32,
                lin.y as f32,
                lin.z as f32,
            ]
        }
        MjJointType::Slide => [0.0, 0.0, 0.0, axis.x as f32, axis.y as f32, axis.z as f32],
        other => panic!("cpu_cdof_partial: unexpected joint type {other:?}"),
    }
}

// ── T5b: cdof for a MULTI-JOINT body (partial-frame subspace) ─────────
// `n_link_pendulum` puts one joint per body, so the cdof loop's final-frame
// vs partial-frame distinction is invisible. `multi_joint_body` carries
// hinge→slide→hinge on a single body: the first hinge and the slide are each
// followed by a later *rotating* joint, so a final-frame cdof over-rotates
// their subspace. This is the only fixture that catches the GPU fk.wgsl
// partial-frame gap (mirrors CPU project-multi-joint-partial-frame).
#[test]
fn t5b_cdof_multi_joint_body() {
    let ctx = gpu_or_skip!();

    let model = Model::multi_joint_body();
    let mut data = model.make_data();
    data.qpos[0] = 0.4; // hinge
    data.qpos[1] = 0.15; // slide
    data.qpos[2] = -0.3; // hinge

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t5b") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    let gpu_cdof = readback_vec4s(&ctx, &state_buf.cdof, model.nv * 2);

    let tol = 1e-4;
    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let cpu = cpu_cdof_partial(&model, &data, jnt_id);
        let gpu_ang = &gpu_cdof[dof * 2];
        let gpu_lin = &gpu_cdof[dof * 2 + 1];
        for r in 0..3 {
            assert!(
                (gpu_ang[r] - cpu[r]).abs() < tol,
                "dof {dof} angular[{r}]: GPU={:.6} CPU={:.6}",
                gpu_ang[r],
                cpu[r]
            );
            assert!(
                (gpu_lin[r] - cpu[r + 3]).abs() < tol,
                "dof {dof} linear[{r}]: GPU={:.6} CPU={:.6}",
                gpu_lin[r],
                cpu[r + 3]
            );
        }
    }
    eprintln!("  T5b passed: cdof matches CPU for multi-joint body");
}

// ── T6: Subtree COM ───────────────────────────────────────────────────

#[test]
fn t6_subtree_com_matches_cpu() {
    let ctx = gpu_or_skip!();

    // Use pendulum with different masses via non-uniform inertia
    let model = Model::n_link_pendulum(3, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = -PI / 6.0;
    data.qpos[2] = PI / 3.0;

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t6") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    let gpu_com = readback_vec4s(&ctx, &state_buf.subtree_com, model.nbody);
    let gpu_mass = readback_f32s(&ctx, &state_buf.subtree_mass, model.nbody);

    let tol = 1e-4;
    for b in 0..model.nbody {
        let cm = gpu_mass[b];
        let cc = data.subtree_mass[b] as f32;
        assert!(
            (cm - cc).abs() < tol,
            "body {b} subtree_mass: GPU={cm:.6} CPU={cc:.6}"
        );

        for k in 0..3 {
            let g = gpu_com[b][k];
            let c = data.subtree_com[b][k] as f32;
            assert!(
                (g - c).abs() < tol,
                "body {b} subtree_com[{k}]: GPU={g:.6} CPU={c:.6}"
            );
        }
    }
    eprintln!("  T6 passed: subtree COM matches CPU");
}

// ── T7: Cinert for hinge chain ────────────────────────────────────────

#[test]
fn t7_cinert_hinge_chain() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(3, 1.0, 2.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = PI / 6.0;
    data.qpos[2] = -PI / 3.0;

    data.forward(&model).expect("CPU forward failed");

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t7") });
    pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    let gpu_cinert = readback_vec4s(&ctx, &state_buf.body_cinert, model.nbody * 3);

    let tol = 1e-3; // Looser for chain + rotated inertia
    for body_id in 1..model.nbody {
        let base = body_id * 3;
        let gpu_mass = gpu_cinert[base][0];
        let cpu = &data.cinert[body_id];
        let cpu_mass = cpu[(3, 3)];
        assert!(
            (gpu_mass - cpu_mass as f32).abs() < tol,
            "body {body_id} mass: GPU={gpu_mass:.6} CPU={cpu_mass:.6}"
        );
    }
    eprintln!("  T7 passed: cinert mass matches CPU for hinge chain");
}

// ── Helpers for CRBA / velocity FK tests ─────────────────────────────

/// Run FK + CRBA on GPU, return (model_buf, state_buf).
fn run_fk_and_crba(
    ctx: &GpuContext,
    model: &Model,
    data: &Data,
) -> (GpuModelBuffers, GpuStateBuffers) {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);
    let fk_pipeline = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("fk+crba"),
        });
    fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    (model_buf, state_buf)
}

/// Run FK + velocity FK on GPU, return (model_buf, state_buf).
fn run_fk_and_vel_fk(
    ctx: &GpuContext,
    model: &Model,
    data: &Data,
) -> (GpuModelBuffers, GpuStateBuffers) {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);
    let fk_pipeline = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("fk+vel_fk"),
        });
    fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel_fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    (model_buf, state_buf)
}

// ── T8: qM diagonal for free body ───────────────────────────────────

#[test]
fn t8_qm_diagonal_free_body() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    data.qpos[0] = 1.0;
    data.qpos[1] = 2.0;
    data.qpos[2] = 3.0;
    let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 3.0);
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    // CPU FK + CRBA
    data.forward(&model).expect("CPU forward failed");

    // GPU FK + CRBA
    let (_, state_buf) = run_fk_and_crba(&ctx, &model, &data);

    // Readback qM
    let nv = model.nv;
    let gpu_qm = readback_f32s(&ctx, &state_buf.qm, nv * nv);

    let tol = 1e-4;
    for i in 0..nv {
        let gpu_val = gpu_qm[i * nv + i];
        let cpu_val = data.qM[(i, i)] as f32;
        assert!(
            (gpu_val - cpu_val).abs() < tol,
            "qM[{i},{i}]: GPU={gpu_val:.6} CPU={cpu_val:.6} err={:.2e}",
            (gpu_val - cpu_val).abs()
        );
    }
    eprintln!("  T8 passed: qM diagonal matches CPU for free body (nv={nv})");
}

// ── T9: qM for 3-link pendulum (off-diagonal + spatial force transport)

#[test]
fn t9_qm_pendulum_full_matrix() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(3, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = PI / 6.0;
    data.qpos[2] = -PI / 3.0;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_fk_and_crba(&ctx, &model, &data);

    let nv = model.nv;
    let gpu_qm = readback_f32s(&ctx, &state_buf.qm, nv * nv);

    let tol = 1e-3;
    for i in 0..nv {
        for j in 0..nv {
            let gpu_val = gpu_qm[i * nv + j];
            let cpu_val = data.qM[(i, j)] as f32;
            assert!(
                (gpu_val - cpu_val).abs() < tol,
                "qM[{i},{j}]: GPU={gpu_val:.6} CPU={cpu_val:.6} err={:.2e}",
                (gpu_val - cpu_val).abs()
            );
        }
    }
    eprintln!("  T9 passed: full qM matches CPU for 3-link pendulum (nv={nv})");
}

// ── T10: qM for flat tree (multiple free bodies) ────────────────────

#[test]
fn t10_qm_flat_tree_free_bodies() {
    let ctx = gpu_or_skip!();

    // Build a 3-free-body scene
    let model = Model::free_body(1.5, Vector3::new(0.2, 0.3, 0.4));
    let mut data = model.make_data();
    // Set a non-trivial pose
    data.qpos[0] = 2.0;
    data.qpos[1] = -1.0;
    data.qpos[2] = 3.0;
    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 5.0);
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_fk_and_crba(&ctx, &model, &data);

    let nv = model.nv;
    let gpu_qm = readback_f32s(&ctx, &state_buf.qm, nv * nv);

    let tol = 1e-4;
    for i in 0..nv {
        for j in 0..nv {
            let gpu_val = gpu_qm[i * nv + j];
            let cpu_val = data.qM[(i, j)] as f32;
            assert!(
                (gpu_val - cpu_val).abs() < tol,
                "qM[{i},{j}]: GPU={gpu_val:.6} CPU={cpu_val:.6} err={:.2e}",
                (gpu_val - cpu_val).abs()
            );
        }
    }
    eprintln!("  T10 passed: qM matches CPU for free body (flat tree, nv={nv})");
}

// ── T10b: qM for a MULTI-JOINT body ──────────────────────────────────
// CRBA's mass-matrix kernel walks the dof_parent chain and projects cdof; the
// intra-body off-diagonals M[i,j] for two DOFs on the SAME body must use the
// partial-frame cdof (no spurious body-boundary transport between them). Guards
// CRBA on the hinge→slide→hinge single-body fixture.
#[test]
fn t10b_qm_multi_joint_body() {
    let ctx = gpu_or_skip!();

    let model = Model::multi_joint_body();
    let mut data = model.make_data();
    data.qpos[0] = 0.4;
    data.qpos[1] = 0.15;
    data.qpos[2] = -0.3;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_fk_and_crba(&ctx, &model, &data);

    let nv = model.nv;
    let gpu_qm = readback_f32s(&ctx, &state_buf.qm, nv * nv);

    let tol = 1e-4;
    for i in 0..nv {
        for j in 0..nv {
            let gpu_val = gpu_qm[i * nv + j];
            let cpu_val = data.qM[(i, j)] as f32;
            assert!(
                (gpu_val - cpu_val).abs() < tol,
                "qM[{i},{j}]: GPU={gpu_val:.6} CPU={cpu_val:.6} err={:.2e}",
                (gpu_val - cpu_val).abs()
            );
        }
    }
    eprintln!("  T10b passed: qM matches CPU for multi-joint body (nv={nv})");
}

// ── T11: cvel for free body with non-zero qvel ──────────────────────

#[test]
fn t11_cvel_free_body() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    // Pose
    data.qpos[0] = 1.0;
    data.qpos[1] = 2.0;
    data.qpos[2] = 3.0;
    let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0);
    data.qpos[3] = q.w;
    data.qpos[4] = q.i;
    data.qpos[5] = q.j;
    data.qpos[6] = q.k;

    // Velocity: linear x=1, angular z=5
    data.qvel[0] = 1.0;
    data.qvel[1] = 0.0;
    data.qvel[2] = 0.0;
    data.qvel[3] = 0.0;
    data.qvel[4] = 0.0;
    data.qvel[5] = 5.0;

    // CPU FK + velocity FK
    data.forward(&model).expect("CPU forward failed");

    // GPU FK + velocity FK
    let (_, state_buf) = run_fk_and_vel_fk(&ctx, &model, &data);

    // Readback cvel (2 vec4 per body)
    let gpu_cvel = readback_vec4s(&ctx, &state_buf.body_cvel, model.nbody * 2);

    let tol = 1e-5;
    for b in 0..model.nbody {
        let gpu_ang = &gpu_cvel[b * 2];
        let gpu_lin = &gpu_cvel[b * 2 + 1];
        let cpu = &data.cvel[b];

        for k in 0..3 {
            let g_a = gpu_ang[k];
            let c_a = cpu[k] as f32;
            assert!(
                (g_a - c_a).abs() < tol,
                "body {b} cvel_angular[{k}]: GPU={g_a:.6} CPU={c_a:.6}"
            );
            let g_l = gpu_lin[k];
            let c_l = cpu[k + 3] as f32;
            assert!(
                (g_l - c_l).abs() < tol,
                "body {b} cvel_linear[{k}]: GPU={g_l:.6} CPU={c_l:.6}"
            );
        }
    }

    // Verify cvel is non-zero for body 1 (the free body)
    if model.nbody > 1 {
        let ang_mag = gpu_cvel[2][0].powi(2) + gpu_cvel[2][1].powi(2) + gpu_cvel[2][2].powi(2);
        let lin_mag = gpu_cvel[3][0].powi(2) + gpu_cvel[3][1].powi(2) + gpu_cvel[3][2].powi(2);
        assert!(
            ang_mag + lin_mag > 1e-6,
            "cvel for body 1 is zero — velocity FK not working"
        );
    }

    eprintln!("  T11 passed: cvel matches CPU for free body with qvel");
}

// ── T12: cvel for 3-link pendulum at various qvel ───────────────────

#[test]
fn t12_cvel_pendulum() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(3, 1.0, 1.0);
    let mut data = model.make_data();
    // Pose
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = -PI / 6.0;
    data.qpos[2] = PI / 3.0;
    // Velocities
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.5;
    data.qvel[2] = 3.0;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_fk_and_vel_fk(&ctx, &model, &data);

    let gpu_cvel = readback_vec4s(&ctx, &state_buf.body_cvel, model.nbody * 2);

    let tol = 1e-4;
    for b in 0..model.nbody {
        let gpu_ang = &gpu_cvel[b * 2];
        let gpu_lin = &gpu_cvel[b * 2 + 1];
        let cpu = &data.cvel[b];

        for k in 0..3 {
            let g_a = gpu_ang[k];
            let c_a = cpu[k] as f32;
            assert!(
                (g_a - c_a).abs() < tol,
                "body {b} cvel_angular[{k}]: GPU={g_a:.6} CPU={c_a:.6} err={:.2e}",
                (g_a - c_a).abs()
            );
            let g_l = gpu_lin[k];
            let c_l = cpu[k + 3] as f32;
            assert!(
                (g_l - c_l).abs() < tol,
                "body {b} cvel_linear[{k}]: GPU={g_l:.6} CPU={c_l:.6} err={:.2e}",
                (g_l - c_l).abs()
            );
        }
    }

    eprintln!("  T12 passed: cvel matches CPU for 3-link pendulum");
}

// ── Session 3 helpers ────────────────────────────────────────────────

/// Run FK + CRBA + velocity FK + RNE on GPU, return (model_buf, state_buf).
fn run_through_rne(
    ctx: &GpuContext,
    model: &Model,
    data: &Data,
) -> (GpuModelBuffers, GpuStateBuffers) {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);
    let fk_pipeline = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne_pipeline = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("fk+crba+vel+rne"),
        });
    fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel_fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne_pipeline.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    (model_buf, state_buf)
}

/// Run FK + CRBA + velocity FK + RNE + smooth on GPU.
fn run_through_smooth(
    ctx: &GpuContext,
    model: &Model,
    data: &Data,
) -> (GpuModelBuffers, GpuStateBuffers) {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);
    let fk_pipeline = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne_pipeline = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);
    let smooth_pipeline = GpuSmoothPipeline::new(ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("fk+crba+vel+rne+smooth"),
        });
    fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel_fk_pipeline.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne_pipeline.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    smooth_pipeline.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    (model_buf, state_buf)
}

// ── T13: qfrc_bias gravity for free body ─────────────────────────────

#[test]
fn t13_qfrc_bias_gravity_free_body() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 5.0;
    // Identity quaternion
    data.qpos[3] = 1.0;

    // CPU forward (runs FK + RNE etc.)
    data.forward(&model).expect("CPU forward failed");

    // GPU through RNE
    let (_, state_buf) = run_through_rne(&ctx, &model, &data);

    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-4;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let c = data.qfrc_bias[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }
    eprintln!("  T13 passed: qfrc_bias gravity matches CPU for free body");
}

// ── T13b: qfrc_bias gravity for a ROTATED, off-COM free body ──────────
// T13 uses an identity orientation, so the free joint's ANGULAR gravity frame
// (the subspace maps body-local ω → world, hence Sᵀ projects the gravity torque
// into the BODY frame) is untested — a world-frame torque passes T13 anyway.
// The discriminator requires BOTH a non-identity orientation AND a real COM
// offset (xipos ≠ body origin) so the gravity torque is non-zero: with the COM
// at the origin (free_body's default body_ipos = 0) every angular DOF is
// identically zero regardless of frame, and the world-vs-body bug stays masked.
#[test]
fn t13b_qfrc_bias_gravity_rotated_free_body() {
    let ctx = gpu_or_skip!();

    let mut model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    // Genuine COM offset (free_body hardcodes body_ipos = 0). This makes the
    // subtree COM differ from the joint anchor, so gravity exerts a non-zero
    // torque whose body-frame vs world-frame projection actually diverges.
    model.body_ipos[1] = Vector3::new(0.08, -0.12, 0.05);

    let mut data = model.make_data();
    data.qpos[2] = 5.0;
    // Non-identity orientation (≈45° about a tilted axis), normalized.
    let q = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(0.3, 1.0, 0.5)),
        0.8,
    );
    let c = q.into_inner().coords; // (x, y, z, w)
    data.qpos[3] = c[3];
    data.qpos[4] = c[0];
    data.qpos[5] = c[1];
    data.qpos[6] = c[2];

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_rne(&ctx, &model, &data);
    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-4;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let cc = data.qfrc_bias[d] as f32;
        assert!(
            (g - cc).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={cc:.6} err={:.2e}",
            (g - cc).abs()
        );
    }
    // Guard against silent regression to the zero-torque tautology: the angular
    // DOFs (3–5) must carry real signal for the frame to be discriminated.
    let angular_signal = (3..6).any(|d| gpu_bias[d].abs() > 1e-3);
    assert!(
        angular_signal,
        "T13b must exercise non-zero angular gravity to test the body-frame projection"
    );
    eprintln!("  T13b passed: qfrc_bias gravity matches CPU for rotated off-COM free body");
}

// ── T14: qfrc_bias Coriolis for spinning free body ───────────────────

#[test]
fn t14_qfrc_bias_coriolis_spinning() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    // Identity pose at origin
    data.qpos[3] = 1.0;
    // High angular velocity (non-principal axis → gyroscopic torque)
    data.qvel[0] = 0.0; // linear x
    data.qvel[1] = 0.0; // linear y
    data.qvel[2] = 0.0; // linear z
    data.qvel[3] = 10.0; // angular x
    data.qvel[4] = 5.0; // angular y
    data.qvel[5] = 3.0; // angular z

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_rne(&ctx, &model, &data);
    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-3;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let c = data.qfrc_bias[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }

    // Verify Coriolis is non-zero (angular DOFs should have non-zero bias)
    let has_coriolis = (3..6).any(|d| gpu_bias[d].abs() > 1e-6);
    assert!(has_coriolis, "Expected non-zero Coriolis on angular DOFs");

    eprintln!("  T14 passed: qfrc_bias Coriolis matches CPU for spinning body");
}

// ── T15: qfrc_bias for 3-link pendulum ──────────────────────────────

#[test]
fn t15_qfrc_bias_pendulum() {
    let ctx = gpu_or_skip!();

    let model = Model::n_link_pendulum(3, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 4.0;
    data.qpos[1] = -PI / 6.0;
    data.qpos[2] = PI / 3.0;
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.5;
    data.qvel[2] = 3.0;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_rne(&ctx, &model, &data);
    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-3;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let c = data.qfrc_bias[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }
    eprintln!("  T15 passed: qfrc_bias matches CPU for 3-link pendulum");
}

// ── T15b: qfrc_bias for a NON-PARALLEL (spatial) 3-link chain ────────
// Guards the RNE forward velocity-product transport. `n_link_pendulum` is an
// all-Y (planar, parallel-axis) chain, where the `[0; ω_parent×r]` transport
// lever of the Coriolis term vanishes — so T15 stays green even if the shader
// drops it. A non-parallel ≥3-link chain is the first geometry where the lever
// is material and the leaf transports a nonzero parent bias acceleration; this
// is exactly where the CPU bug (fixed alongside this shader) showed up vs MuJoCo.
#[test]
fn t15b_qfrc_bias_spatial_chain() {
    let ctx = gpu_or_skip!();

    // Start from the 3-link chain and break parallelism/planarity: cycle the
    // hinge axes through Y/X/Z and offset the leaf laterally (mirrors the CPU
    // `three_link_spatial` conformance fixture). Mutating these public fields is
    // valid here — GPU and CPU consume the identical model, so the test asserts
    // GPU == CPU, and the CPU path is itself MuJoCo-validated.
    let mut model = Model::n_link_pendulum(3, 0.2, 0.4);
    model.jnt_axis[0] = Vector3::new(0.0, 1.0, 0.0);
    model.jnt_axis[1] = Vector3::new(1.0, 0.0, 0.0);
    model.jnt_axis[2] = Vector3::new(0.0, 0.0, 1.0);
    model.body_pos[3] = Vector3::new(0.03, 0.0, -0.2); // lateral offset → non-planar

    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.35;
    data.qpos[2] = 0.25;
    data.qvel[0] = 0.9;
    data.qvel[1] = -0.6;
    data.qvel[2] = 0.7;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_rne(&ctx, &model, &data);
    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-3;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let c = data.qfrc_bias[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }
    eprintln!("  T15b passed: qfrc_bias matches CPU for non-parallel 3-link chain");
}

// ── T15c: qfrc_bias for a MULTI-JOINT body ───────────────────────────
// Downstream of T5b: cdof feeds CRBA (qM) and RNE (qfrc_bias), so a corrupted
// partial-frame subspace propagates here too. Guards the full pipeline on the
// hinge→slide→hinge single-body fixture, with nonzero qvel to engage the
// velocity-dependent Coriolis terms.
#[test]
fn t15c_qfrc_bias_multi_joint_body() {
    let ctx = gpu_or_skip!();

    let model = Model::multi_joint_body();
    let mut data = model.make_data();
    data.qpos[0] = 0.4;
    data.qpos[1] = 0.15;
    data.qpos[2] = -0.3;
    data.qvel[0] = 0.8;
    data.qvel[1] = -0.5;
    data.qvel[2] = 0.6;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_rne(&ctx, &model, &data);
    let gpu_bias = readback_f32s(&ctx, &state_buf.qfrc_bias, model.nv);

    let tol = 1e-3;
    for d in 0..model.nv {
        let g = gpu_bias[d];
        let c = data.qfrc_bias[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qfrc_bias[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }
    eprintln!("  T15c passed: qfrc_bias matches CPU for multi-joint body");
}

// ── T16: qacc_smooth for free body under gravity ─────────────────────

#[test]
fn t16_qacc_smooth_free_body() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(2.5, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    data.qpos[2] = 5.0;
    data.qpos[3] = 1.0;

    data.forward(&model).expect("CPU forward failed");

    let (_, state_buf) = run_through_smooth(&ctx, &model, &data);
    let gpu_qacc = readback_f32s(&ctx, &state_buf.qacc_smooth, model.nv);

    let tol = 1e-4;
    for d in 0..model.nv {
        let g = gpu_qacc[d];
        let c = data.qacc_smooth[d] as f32;
        assert!(
            (g - c).abs() < tol,
            "qacc_smooth[{d}]: GPU={g:.6} CPU={c:.6} err={:.2e}",
            (g - c).abs()
        );
    }
    eprintln!("  T16 passed: qacc_smooth matches CPU for free body under gravity");
}

// ── T17: Gravity drop trajectory (2 seconds) ─────────────────────────

#[test]
fn t17_gravity_drop_trajectory() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(1.0, Vector3::new(0.1, 0.1, 0.1));
    let mut data = model.make_data();
    data.qpos[2] = 10.0; // z = 10
    data.qpos[3] = 1.0; // identity quat

    // Create GPU pipelines
    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(&ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(&ctx, &model_buf, &state_buf);
    let rne_pipeline = GpuRnePipeline::new(&ctx, &model_buf, &state_buf, &model);
    let smooth_pipeline = GpuSmoothPipeline::new(&ctx, &model_buf, &state_buf);
    let integrate_pipeline = GpuIntegratePipeline::new(&ctx, &model_buf, &state_buf);

    let nv = model.nv as u64;
    let dt = model.timestep;
    let nsteps = (2.0 / dt).round() as usize; // ~2 seconds

    // Run GPU physics loop
    for _step in 0..nsteps {
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("step"),
            });

        fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        crba_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        vel_fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        rne_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);
        smooth_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);

        // Gravity-only bridge: copy qacc_smooth → qacc
        encoder.copy_buffer_to_buffer(&state_buf.qacc_smooth, 0, &state_buf.qacc, 0, nv * 4);

        integrate_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);

        ctx.queue.submit([encoder.finish()]);
    }

    // Run CPU physics loop
    let mut cpu_data = model.make_data();
    cpu_data.qpos[2] = 10.0;
    cpu_data.qpos[3] = 1.0;
    for _step in 0..nsteps {
        cpu_data.step(&model).expect("CPU step failed");
    }

    // Readback GPU qpos
    let gpu_qpos = readback_f32s(&ctx, &state_buf.qpos, model.nq);

    // Compare z position (should be close to 10 - 0.5*9.81*4 ≈ -9.62)
    let gpu_z = gpu_qpos[2];
    let cpu_z = cpu_data.qpos[2] as f32;
    let z_err = (gpu_z - cpu_z).abs();

    eprintln!("  T17: GPU z={gpu_z:.4} CPU z={cpu_z:.4} err={z_err:.4}");
    assert!(
        z_err < 0.05,
        "Trajectory diverged: GPU z={gpu_z:.4} CPU z={cpu_z:.4} err={z_err:.4}"
    );

    // x and y should stay near 0
    assert!(gpu_qpos[0].abs() < 0.01, "x drifted: {}", gpu_qpos[0]);
    assert!(gpu_qpos[1].abs() < 0.01, "y drifted: {}", gpu_qpos[1]);

    eprintln!("  T17 passed: gravity drop trajectory matches CPU over {nsteps} steps");
}

// ── T18: Quaternion stability under high angular velocity ────────────

#[test]
fn t18_quaternion_stability() {
    let ctx = gpu_or_skip!();

    let model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    let mut data = model.make_data();
    data.qpos[3] = 1.0; // identity quat
    // High angular velocity around z (50 rad/s)
    data.qvel[3] = 0.0;
    data.qvel[4] = 0.0;
    data.qvel[5] = 50.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(&ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(&ctx, &model_buf, &state_buf);
    let rne_pipeline = GpuRnePipeline::new(&ctx, &model_buf, &state_buf, &model);
    let smooth_pipeline = GpuSmoothPipeline::new(&ctx, &model_buf, &state_buf);
    let integrate_pipeline = GpuIntegratePipeline::new(&ctx, &model_buf, &state_buf);

    let nv = model.nv as u64;
    let nsteps = 1000;

    for step in 0..nsteps {
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("quat_step"),
            });

        fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        crba_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        vel_fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        rne_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);
        smooth_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);
        encoder.copy_buffer_to_buffer(&state_buf.qacc_smooth, 0, &state_buf.qacc, 0, nv * 4);
        integrate_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);

        ctx.queue.submit([encoder.finish()]);

        // Check quaternion norm every 100 steps
        if (step + 1) % 100 == 0 {
            let qpos = readback_f32s(&ctx, &state_buf.qpos, model.nq);
            // Quaternion at qpos[3..7] (w,x,y,z)
            let w = qpos[3];
            let x = qpos[4];
            let y = qpos[5];
            let z = qpos[6];
            let norm = (w * w + x * x + y * y + z * z).sqrt();
            assert!(
                (norm - 1.0).abs() < 0.01,
                "Step {}: quaternion norm {norm:.6} deviated from 1.0",
                step + 1
            );
        }
    }

    eprintln!("  T18 passed: quaternion stable over {nsteps} steps at 50 rad/s");
}

// ══════════════════════════════════════════════════════════════════════
// T19–T23: Collision pipeline
// ══════════════════════════════════════════════════════════════════════

/// Add an SDF sphere geom to the specified body.
fn add_sdf_sphere_geom(model: &mut Model, body_id: usize, radius: f64, grid_res: usize) {
    use cf_geometry::SdfGrid;
    use nalgebra::Point3;
    use sim_core::ShapeConvex;
    use sim_core::sdf::PhysicsShape;
    use std::sync::Arc;

    let grid = SdfGrid::sphere(Point3::origin(), radius, grid_res, 1.0);
    let shape: Arc<dyn PhysicsShape> = Arc::new(ShapeConvex::new(Arc::new(grid)));
    let shape_id = model.shape_data.len();
    model.shape_data.push(shape);
    model.nshape = model.shape_data.len();

    let geom_id = model.ngeom;
    model.ngeom += 1;

    model.geom_type.push(sim_core::types::GeomType::Sdf);
    model.geom_body.push(body_id);
    model.geom_pos.push(Vector3::zeros());
    model.geom_quat.push(UnitQuaternion::identity());
    // SDF geom_size = half-extents of the grid's AABB
    let half = radius + 1.0; // conservative: radius + cell_size margin
    model.geom_size.push(Vector3::new(half, half, half));
    model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
    model.geom_condim.push(3);
    model.geom_contype.push(1);
    model.geom_conaffinity.push(1);
    model.geom_margin.push(1.0); // 1mm margin for SDF
    model.geom_gap.push(0.0);
    model.geom_priority.push(0);
    model.geom_solmix.push(1.0);
    model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.geom_solref.push([0.02, 1.0]);
    model.geom_fluid.push([0.0; 12]);
    model.geom_name.push(Some(format!("sdf_sphere_{geom_id}")));
    model.geom_rbound.push(radius);
    model.geom_aabb.push([0.0, 0.0, 0.0, half, half, half]);
    model.geom_mesh.push(None);
    model.geom_hfield.push(None);
    model.geom_shape.push(Some(shape_id));
    model.geom_group.push(0);
    model.geom_rgba.push([0.7, 0.3, 0.3, 1.0]);
    model.geom_user.push(vec![]);
    model.geom_plugin.push(None);

    // Update body geom tracking
    if model.body_geom_num[body_id] == 0 {
        model.body_geom_adr[body_id] = geom_id;
    }
    model.body_geom_num[body_id] += 1;
}

/// Readback pipeline contacts from GPU buffer.
fn readback_contacts(
    ctx: &GpuContext,
    contact_buffer: &wgpu::Buffer,
    contact_count_buffer: &wgpu::Buffer,
    max_contacts: usize,
) -> Vec<PipelineContact> {
    // Readback count
    let count_data = readback_f32s(ctx, contact_count_buffer, 1);
    let count = f32::to_bits(count_data[0]) as usize;
    let count = count.min(max_contacts);
    if count == 0 {
        return vec![];
    }

    // Readback contacts (as f32s, reinterpret as PipelineContact)
    // PipelineContact = 48 bytes = 12 f32s
    let raw = readback_f32s(ctx, contact_buffer, count * 12);
    let contacts: &[PipelineContact] = bytemuck::cast_slice(&raw);
    contacts[..count].to_vec()
}

#[test]
fn t19_aabb_matches_cpu() {
    let ctx = gpu_or_skip!();

    // Build a model with a ground plane + one SDF sphere at (0, 0, 5)
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    // Place sphere at (0, 0, 5)
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 5.0;
    data.qpos[3] = 1.0; // identity quat

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(&ctx, &model, &model_buf, &state_buf);

    // Run FK + AABB
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t19") });
    fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    collision_pipeline.encode(&mut encoder, &state_buf);
    ctx.queue.submit([encoder.finish()]);

    // Readback AABB for each geom (2×vec4 per geom)
    let n_aabb_vec4s = model.ngeom * 2;
    let aabb_data = readback_vec4s(&ctx, &state_buf.geom_aabb, n_aabb_vec4s);

    // Ground plane (geom 0): should have huge AABB
    let plane_min = &aabb_data[0];
    let plane_max = &aabb_data[1];
    assert!(
        plane_max[0] - plane_min[0] > 1e5,
        "Plane AABB should be huge, got x-range {}",
        plane_max[0] - plane_min[0]
    );

    // SDF sphere (geom 1): should be centered at (0,0,5) with half-extent ~6
    let sphere_min = &aabb_data[2];
    let sphere_max = &aabb_data[3];
    let center_x = (sphere_min[0] + sphere_max[0]) * 0.5;
    let center_z = (sphere_min[2] + sphere_max[2]) * 0.5;
    assert!(
        (center_x).abs() < 0.1,
        "Sphere AABB center x should be ~0, got {center_x}"
    );
    assert!(
        (center_z - 5.0).abs() < 0.1,
        "Sphere AABB center z should be ~5, got {center_z}"
    );

    // Half-extent should be > radius (5)
    let half_x = (sphere_max[0] - sphere_min[0]) * 0.5;
    assert!(
        (4.5..=8.0).contains(&half_x),
        "Sphere AABB half-extent should be ~5-6, got {half_x}"
    );

    eprintln!(
        "  T19 passed: plane AABB range={:.0}, sphere center=({center_x:.2},{:.2},{center_z:.2}) half={half_x:.2}",
        plane_max[0] - plane_min[0],
        (sphere_min[1] + sphere_max[1]) * 0.5,
    );
}

#[test]
fn t20_sdf_sdf_contacts_pipeline() {
    let ctx = gpu_or_skip!();

    // Two overlapping SDF spheres (radius 5, centers 8 apart → overlap)
    // Body 0 = world, Body 1 = free body
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    add_sdf_sphere_geom(&mut model, 0, 5.0, 16); // geom on world body at origin
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16); // geom on free body

    let mut data = model.make_data();
    // Place free body at (8, 0, 0) → spheres overlap by ~2mm
    data.qpos[0] = 8.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 0.0;
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(&ctx, &model, &model_buf, &state_buf);

    eprintln!("  T20: {} collision pairs", collision_pipeline.num_pairs());

    // Run FK + collision
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t20") });
    fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    collision_pipeline.encode(&mut encoder, &state_buf);
    ctx.queue.submit([encoder.finish()]);

    // Readback contacts
    let contacts = readback_contacts(
        &ctx,
        &state_buf.contact_buffer,
        &state_buf.contact_count,
        32768,
    );

    eprintln!("  T20: {} contacts found", contacts.len());
    assert!(
        !contacts.is_empty(),
        "SDF-SDF should find contacts (spheres overlap by ~2mm)"
    );

    // Check contact properties
    for (i, c) in contacts.iter().take(5).enumerate() {
        let n_len =
            (c.normal[0] * c.normal[0] + c.normal[1] * c.normal[1] + c.normal[2] * c.normal[2])
                .sqrt();
        eprintln!(
            "    [{i}] pos=({:.2},{:.2},{:.2}) depth={:.4} normal=({:.2},{:.2},{:.2}) |n|={n_len:.3}",
            c.point[0], c.point[1], c.point[2], c.depth, c.normal[0], c.normal[1], c.normal[2],
        );
        assert!(c.depth >= 0.0, "Contact depth should be >= 0");
        assert!(
            n_len > 0.9 && n_len < 1.1,
            "Contact normal should be unit length, got {n_len}"
        );
    }

    // Some contacts should have nonzero depth and X-normal component.
    // Note: SDF-SDF symmetric dispatches (A→B, B→A) produce contacts with
    // opposite normals, so the NET sum cancels. Check individual contacts.
    let deep_contacts: Vec<_> = contacts.iter().filter(|c| c.depth > 0.01).collect();
    eprintln!("  T20: {} contacts with depth > 0.01", deep_contacts.len());
    assert!(
        !deep_contacts.is_empty(),
        "Should have contacts with nonzero depth"
    );
    let has_x_normal = deep_contacts.iter().any(|c| c.normal[0].abs() > 0.3);
    assert!(has_x_normal, "Deep contacts should have X-normal component");

    eprintln!("  T20 passed: {} SDF-SDF contacts", contacts.len());
}

#[test]
fn t21_sdf_plane_contacts() {
    let ctx = gpu_or_skip!();

    // SDF sphere slightly below ground plane → SDF-plane contacts expected
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16);

    let mut data = model.make_data();
    // Place sphere center at z=3 → bottom of sphere at z=-2 → penetrates z=0 plane
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 3.0;
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(&ctx, &model, &model_buf, &state_buf);

    eprintln!("  T21: {} collision pairs", collision_pipeline.num_pairs());

    // Run FK + collision
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t21") });
    fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    collision_pipeline.encode(&mut encoder, &state_buf);
    ctx.queue.submit([encoder.finish()]);

    // Readback contacts
    let contacts = readback_contacts(
        &ctx,
        &state_buf.contact_buffer,
        &state_buf.contact_count,
        32768,
    );

    eprintln!("  T21: {} contacts found", contacts.len());
    assert!(
        !contacts.is_empty(),
        "SDF-plane should find contacts (sphere penetrates ground)"
    );

    // All contacts should be below the plane (z < margin) with normal ~(0,0,1)
    for (i, c) in contacts.iter().take(5).enumerate() {
        eprintln!(
            "    [{i}] pos=({:.2},{:.2},{:.2}) depth={:.4} normal=({:.2},{:.2},{:.2})",
            c.point[0], c.point[1], c.point[2], c.depth, c.normal[0], c.normal[1], c.normal[2],
        );
        assert!(c.depth >= 0.0, "Contact depth should be >= 0");
        // Normal should point up (away from plane)
        assert!(
            c.normal[2] > 0.9,
            "SDF-plane normal should point up, got z={:.3}",
            c.normal[2]
        );
    }

    // Contacts should be in the lower hemisphere (z < 3)
    let avg_z: f32 = contacts.iter().map(|c| c.point[2]).sum::<f32>() / contacts.len() as f32;
    assert!(
        avg_z < 1.5,
        "Contact positions should be near z=0 plane, avg_z={avg_z:.2}"
    );

    eprintln!(
        "  T21 passed: {} SDF-plane contacts, avg_z={avg_z:.2}",
        contacts.len()
    );
}

#[test]
fn t22_full_collision_pipeline() {
    let ctx = gpu_or_skip!();

    // Hockey-like scene: plane + 2 SDF spheres
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();

    // Sphere A on world body at origin
    add_sdf_sphere_geom(&mut model, 0, 5.0, 12);
    // Sphere B on free body
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    // Free body at (7, 0, 3) → SDF-SDF overlap + SDF-plane overlap
    data.qpos[0] = 7.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 3.0;
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);
    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(&ctx, &model, &model_buf, &state_buf);

    let n_pairs = collision_pipeline.num_pairs();
    eprintln!("  T22: {n_pairs} collision pairs");
    assert!(n_pairs > 0, "Should have at least one collision pair");

    // Run FK + collision
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("t22") });
    fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
    collision_pipeline.encode(&mut encoder, &state_buf);
    ctx.queue.submit([encoder.finish()]);

    let contacts = readback_contacts(
        &ctx,
        &state_buf.contact_buffer,
        &state_buf.contact_count,
        32768,
    );

    eprintln!("  T22: {} total contacts", contacts.len());
    assert!(
        !contacts.is_empty(),
        "Should find contacts (SDF overlap + plane penetration)"
    );

    // Verify geom indices are valid
    for c in &contacts {
        assert!(
            (c.geom1 as usize) < model.ngeom,
            "geom1={} out of range (ngeom={})",
            c.geom1,
            model.ngeom
        );
        assert!(
            (c.geom2 as usize) < model.ngeom,
            "geom2={} out of range (ngeom={})",
            c.geom2,
            model.ngeom
        );
    }

    // Verify friction values are non-negative
    for c in &contacts {
        assert!(c.friction[0] >= 0.0, "friction[0] should be >= 0");
        assert!(c.friction[1] >= 0.0, "friction[1] should be >= 0");
    }

    eprintln!(
        "  T22 passed: full collision pipeline, {n_pairs} pairs, {} contacts",
        contacts.len()
    );
}

// ══════════════════════════════════════════════════════════════════════
// T23–T27: Constraint solve
// ══════════════════════════════════════════════════════════════════════

/// Run full FK → CRBA → vel FK → RNE → smooth → collision → constraint pipeline
/// and return GPU qacc readback.
fn run_full_pipeline_with_constraints(
    ctx: &GpuContext,
    model: &Model,
    model_buf: &GpuModelBuffers,
    state_buf: &GpuStateBuffers,
) -> Vec<f32> {
    let fk_pipeline = GpuFkPipeline::new(ctx, model_buf, state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(ctx, model_buf, state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(ctx, model_buf, state_buf);
    let rne_pipeline = GpuRnePipeline::new(ctx, model_buf, state_buf, model);
    let smooth_pipeline = GpuSmoothPipeline::new(ctx, model_buf, state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(ctx, model, model_buf, state_buf);
    let constraint_pipeline = GpuConstraintPipeline::new(ctx, model_buf, state_buf, model);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("full_pipeline"),
        });

    fk_pipeline.dispatch(ctx, model_buf, state_buf, &mut encoder);
    crba_pipeline.dispatch(ctx, model_buf, state_buf, &mut encoder);
    vel_fk_pipeline.dispatch(ctx, model_buf, state_buf, &mut encoder);
    rne_pipeline.dispatch(ctx, model_buf, state_buf, model, &mut encoder);
    smooth_pipeline.dispatch(ctx, model_buf, state_buf, model, &mut encoder);
    collision_pipeline.encode(&mut encoder, state_buf);
    constraint_pipeline.encode(&mut encoder, state_buf);

    ctx.queue.submit([encoder.finish()]);

    readback_f32s(ctx, &state_buf.qacc, model.nv)
}

/// Readback constraint count from GPU.
fn readback_constraint_count(ctx: &GpuContext, state_buf: &GpuStateBuffers) -> u32 {
    let data = readback_f32s(ctx, &state_buf.constraint_count, 1);
    f32::to_bits(data[0])
}

#[test]
fn t23_constraint_assembly_produces_rows() {
    let ctx = gpu_or_skip!();

    // SDF sphere partially below ground → should produce contacts → constraint rows
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    // Place sphere center at z=3 → bottom of sphere at z=-2 → penetrates ground
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = 3.0;
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);

    let _qacc = run_full_pipeline_with_constraints(&ctx, &model, &model_buf, &state_buf);

    // Verify constraint rows were produced
    let n_constraints = readback_constraint_count(&ctx, &state_buf);
    eprintln!("  T23: constraint_count = {n_constraints}");
    assert!(
        n_constraints > 0,
        "Should produce constraint rows from SDF-plane contacts"
    );

    // condim=3 → 4 rows per contact
    assert!(
        n_constraints.is_multiple_of(4),
        "Constraint count should be multiple of 4 (condim=3 pyramidal), got {n_constraints}"
    );

    // Verify efc_D values are positive (regularization)
    let efc_d = readback_f32s(&ctx, &state_buf.efc_d, n_constraints as usize);
    for (i, &d) in efc_d.iter().enumerate() {
        assert!(
            d > 0.0,
            "efc_D[{i}] should be positive (regularization), got {d}"
        );
    }

    eprintln!("  T23 passed: {n_constraints} constraint rows, all efc_D > 0");
}

#[test]
fn t24_newton_solver_sphere_on_ground() {
    let ctx = gpu_or_skip!();

    // SDF sphere resting on ground — Newton solver should produce upward constraint force
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    data.qpos[2] = 3.0; // partially below ground
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);

    let qacc = run_full_pipeline_with_constraints(&ctx, &model, &model_buf, &state_buf);

    // Also get qacc_smooth for comparison
    let qacc_smooth = readback_f32s(&ctx, &state_buf.qacc_smooth, model.nv);

    eprintln!("  T24: qacc_smooth = {:?}", &qacc_smooth);
    eprintln!("  T24: qacc        = {:?}", &qacc);

    // qacc_smooth should have negative z (gravity pulling down)
    assert!(
        qacc_smooth[2] < -1.0,
        "qacc_smooth z should be < -1 (gravity), got {}",
        qacc_smooth[2]
    );

    // qacc should have z pushed upward relative to qacc_smooth
    // (constraint force opposes penetration)
    assert!(
        qacc[2] > qacc_smooth[2],
        "Constraint should push qacc z upward: qacc.z={} > qacc_smooth.z={}",
        qacc[2],
        qacc_smooth[2]
    );

    // All values should be finite
    for (i, &a) in qacc.iter().enumerate() {
        assert!(a.is_finite(), "qacc[{i}] = {a} is not finite");
    }

    eprintln!(
        "  T24 passed: qacc.z={:.4} > qacc_smooth.z={:.4} (constraint pushes up)",
        qacc[2], qacc_smooth[2]
    );
}

#[test]
fn t25_zero_contacts_qacc_equals_smooth() {
    let ctx = gpu_or_skip!();

    // Sphere well above ground — no contacts
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    data.qpos[2] = 100.0; // far above ground
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);

    let qacc = run_full_pipeline_with_constraints(&ctx, &model, &model_buf, &state_buf);
    let qacc_smooth = readback_f32s(&ctx, &state_buf.qacc_smooth, model.nv);

    // No contacts → constraint_count should be 0
    let n_constraints = readback_constraint_count(&ctx, &state_buf);
    assert_eq!(
        n_constraints, 0,
        "Should have 0 constraints far above ground"
    );

    // qacc should exactly match qacc_smooth (Newton solver cold-starts from qacc_smooth
    // and with 0 constraints, writes it back unchanged)
    for i in 0..model.nv {
        let diff = (qacc[i] - qacc_smooth[i]).abs();
        assert!(
            diff < 1e-6,
            "qacc[{i}]={} should match qacc_smooth[{i}]={}, diff={diff}",
            qacc[i],
            qacc_smooth[i]
        );
    }

    eprintln!("  T25 passed: zero contacts, qacc == qacc_smooth");
}

#[test]
fn t26_multi_substep_stability() {
    let ctx = gpu_or_skip!();

    // SDF sphere dropped onto ground — run 10 substeps, verify no explosion
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    data.qpos[2] = 4.0; // sphere center at z=4 → slight penetration
    data.qpos[3] = 1.0;

    let model_buf = GpuModelBuffers::upload(&ctx, &model);
    let state_buf = GpuStateBuffers::new(&ctx, &model_buf, &data);

    let fk_pipeline = GpuFkPipeline::new(&ctx, &model_buf, &state_buf);
    let crba_pipeline = GpuCrbaPipeline::new(&ctx, &model_buf, &state_buf);
    let vel_fk_pipeline = GpuVelocityFkPipeline::new(&ctx, &model_buf, &state_buf);
    let rne_pipeline = GpuRnePipeline::new(&ctx, &model_buf, &state_buf, &model);
    let smooth_pipeline = GpuSmoothPipeline::new(&ctx, &model_buf, &state_buf);
    let collision_pipeline = GpuCollisionPipeline::new(&ctx, &model, &model_buf, &state_buf);
    let constraint_pipeline = GpuConstraintPipeline::new(&ctx, &model_buf, &state_buf, &model);
    let integrate_pipeline = GpuIntegratePipeline::new(&ctx, &model_buf, &state_buf);

    let n_steps = 10;
    for step in 0..n_steps {
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("substep"),
            });

        fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        crba_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        vel_fk_pipeline.dispatch(&ctx, &model_buf, &state_buf, &mut encoder);
        rne_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);
        smooth_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);
        collision_pipeline.encode(&mut encoder, &state_buf);
        constraint_pipeline.encode(&mut encoder, &state_buf);
        integrate_pipeline.dispatch(&ctx, &model_buf, &state_buf, &model, &mut encoder);

        ctx.queue.submit([encoder.finish()]);

        // Check stability every step
        let qpos = readback_f32s(&ctx, &state_buf.qpos, model.nq);
        for (i, &p) in qpos.iter().enumerate() {
            assert!(
                p.is_finite(),
                "Step {step}: qpos[{i}] = {p} is not finite (explosion)"
            );
            assert!(
                p.abs() < 1000.0,
                "Step {step}: qpos[{i}] = {p} exploded (|val| > 1000)"
            );
        }
    }

    let final_qpos = readback_f32s(&ctx, &state_buf.qpos, model.nq);
    let final_z = final_qpos[2];
    eprintln!("  T26: after {n_steps} steps, z = {final_z:.4}");

    // Sphere should not have fallen through the ground (z should be > -10)
    assert!(final_z > -10.0, "Sphere fell through ground: z = {final_z}");

    eprintln!("  T26 passed: {n_steps} substeps stable, final z = {final_z:.4}");
}

// ═══════════════════════════════════════════════════════════════════════
// Session 6: Pipeline orchestration tests
// ═══════════════════════════════════════════════════════════════════════

use super::orchestrator::{GpuPhysicsPipeline, GpuPipelineError};

// ── T28: Model validation ─────────────────────────────────────────────

#[test]
fn t28_model_validation() {
    // Free-body-only model should succeed
    let model = Model::free_body(1.0, Vector3::new(0.1, 0.1, 0.1));
    let data = model.make_data();
    match GpuPhysicsPipeline::new(&model, &data) {
        Ok(_) => eprintln!("  T28: free-body model accepted"),
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T28: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Free-body model should be accepted, got: {e}"),
    }

    // Hinge joint should be rejected
    let hinge_model = Model::n_link_pendulum(3, 1.0, 1.0);
    let hinge_data = hinge_model.make_data();
    match GpuPhysicsPipeline::new(&hinge_model, &hinge_data) {
        Err(GpuPipelineError::UnsupportedJointType(_, _)) => {
            eprintln!("  T28: hinge model correctly rejected");
        }
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T28: skipping nv check (no GPU)");
            return;
        }
        Ok(_) => panic!("Hinge model should be rejected, but was accepted"),
        Err(e) => panic!("Hinge model should be rejected with UnsupportedJointType, got: {e}"),
    }

    eprintln!("  T28 passed: model validation works");
}

// ── T29: Single substep via orchestrator ──────────────────────────────

#[test]
fn t29_single_substep_orchestrator() {
    // Setup: SDF sphere dropped from z=10 onto ground
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    data.qpos[2] = 10.0;
    data.qpos[3] = 1.0;

    let pipeline = match GpuPhysicsPipeline::new(&model, &data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T29: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    // Run 1 substep via orchestrator
    pipeline.step(&model, std::slice::from_mut(&mut data), 1);

    // Verify state is updated (z should have decreased slightly due to gravity)
    eprintln!("  T29: after 1 step, z = {:.6}", data.qpos[2]);
    assert!(data.qpos[2] < 10.0, "z should decrease under gravity");
    assert!(data.qpos[2] > 9.9, "z should not jump too far in 1 step");
    assert!(data.qpos[0].abs() < 0.01, "x should stay near 0");
    assert!(data.qpos[1].abs() < 0.01, "y should stay near 0");

    // Verify time advanced
    assert!(
        (data.time - model.timestep).abs() < 1e-10,
        "time should advance by dt"
    );

    eprintln!("  T29 passed: single substep via orchestrator");
}

// ── T30: Multi-substep single-submit ──────────────────────────────────

#[test]
fn t30_multi_substep_single_submit() {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    // Run 10 substeps in ONE submit
    let mut data_batch = model.make_data();
    data_batch.qpos[2] = 10.0;
    data_batch.qpos[3] = 1.0;

    let pipeline = match GpuPhysicsPipeline::new(&model, &data_batch) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T30: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    pipeline.step(&model, std::slice::from_mut(&mut data_batch), 10);

    // Run 10 substeps in 10 separate submits
    let mut data_serial = model.make_data();
    data_serial.qpos[2] = 10.0;
    data_serial.qpos[3] = 1.0;

    let pipeline2 = GpuPhysicsPipeline::new(&model, &data_serial).unwrap();
    for _ in 0..10 {
        pipeline2.step(&model, std::slice::from_mut(&mut data_serial), 1);
    }

    // Compare final states
    eprintln!(
        "  T30: batch z={:.6}, serial z={:.6}",
        data_batch.qpos[2], data_serial.qpos[2]
    );

    for i in 0..model.nq {
        let diff = (data_batch.qpos[i] - data_serial.qpos[i]).abs();
        assert!(
            diff < 1e-4,
            "qpos[{i}] diverged: batch={:.6} serial={:.6} diff={diff:.2e}",
            data_batch.qpos[i],
            data_serial.qpos[i]
        );
    }
    for i in 0..model.nv {
        let diff = (data_batch.qvel[i] - data_serial.qvel[i]).abs();
        assert!(
            diff < 1e-3,
            "qvel[{i}] diverged: batch={:.6} serial={:.6} diff={diff:.2e}",
            data_batch.qvel[i],
            data_serial.qvel[i]
        );
    }

    eprintln!("  T30 passed: multi-substep single-submit matches serial");
}

// ── T31: GPU vs CPU trajectory comparison (5 seconds) ─────────────────

/// Per-engine physical invariants over a full bouncing-sphere rollout.
///
/// The GPU (SDF-cell collision) and CPU (analytic collision) produce
/// LEGITIMATELY different trajectories for the same drop — measured over 5 s the
/// GPU settles to rest by ~step 300 while the CPU is still bouncing at step 1200
/// (and the GPU rest height carries a coarse-SDF offset, ≠ the analytic radius).
/// So a cross-engine position match is the wrong gate (the old `z_diff < 1.5 m`
/// stopgap had to be loosened repeatedly and would mask a real per-engine bug).
/// Instead, assert the SHARED PHYSICAL TRUTHS each engine must satisfy on its
/// own — these are measured to hold for both:
/// - **No energy pumping:** mechanical energy never exceeds the initial value
///   (a solver injecting energy is a real bug a final-position check misses).
/// - **Net dissipation:** final energy is well below initial — the contact
///   actually engaged and removed energy (a vacuity guard: a body that missed
///   the plane or froze immediately would not dissipate).
/// - **Penetration bound:** the sphere never sinks more than `PEN_MAX` below the
///   plane (no tunnelling).
/// - **No launch / finite:** never rises above the drop height and stays finite.
///
/// These are PER-ENGINE physical gates: they catch a bug that breaks one engine's
/// physics (energy injection, tunnelling, divergence), not one that corrupts both
/// engines identically. Per-step CROSS-engine agreement is covered separately and
/// exactly by the injected-contact conformance suite (`contact_conformance_tests`,
/// byte-validated efc/qacc), which removes the collision-divergence this rollout
/// deliberately lets through.
struct RolloutInvariants {
    e0: f64,
    max_energy: f64,
    final_energy: f64,
    max_penetration: f64,
    max_z: f64,
    all_finite: bool,
}

impl RolloutInvariants {
    fn assert(&self, label: &str) {
        const DROP_Z: f64 = 10.0;
        // Soft analytic contact allows a small, bounded sink (measured CPU max
        // 0.137 on radius 5); PEN_MAX comfortably bounds it while catching
        // tunnelling (sinking through the geom).
        const PEN_MAX: f64 = 0.5;
        // Energy pumping tolerance: 1% of E0. Measured excess is exactly 0 for
        // both engines (semi-implicit Euler does not pump here); a real
        // energy-injecting solver adds tens of joules, far outside this.
        let energy_tol = 0.01 * self.e0;

        eprintln!(
            "  {label}: E0={:.3} maxE={:.3} (excess {:.2e}) finalE={:.3} maxpen={:.4} maxz={:.3}",
            self.e0,
            self.max_energy,
            self.max_energy - self.e0,
            self.final_energy,
            self.max_penetration,
            self.max_z,
        );

        assert!(self.all_finite, "{label}: non-finite state during rollout");
        assert!(
            self.max_energy <= self.e0 + energy_tol,
            "{label}: energy pumped to {:.3} above E0={:.3} (+{:.3} > tol {:.3})",
            self.max_energy,
            self.e0,
            self.max_energy - self.e0,
            energy_tol,
        );
        assert!(
            self.final_energy < 0.95 * self.e0,
            "{label}: no net dissipation — finalE={:.3} not below 0.95·E0={:.3} \
             (contact never engaged?)",
            self.final_energy,
            0.95 * self.e0,
        );
        assert!(
            self.max_penetration < PEN_MAX,
            "{label}: sphere sank {:.4} below the plane (limit {PEN_MAX}) — tunnelling",
            self.max_penetration,
        );
        assert!(
            self.max_z < DROP_Z + 0.1,
            "{label}: rose to z={:.3} above the drop height {DROP_Z} — energy injected",
            self.max_z,
        );
    }
}

#[test]
fn t31_gpu_vs_cpu_trajectory() {
    const RADIUS: f64 = 5.0;
    const DROP_Z: f64 = 10.0;

    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, RADIUS, 12);

    let g = model.gravity.z.abs();
    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];
    // Mechanical energy KE + PE for the single free body. `free_body` sets COM at
    // the body origin (ipos = 0), so PE = m·g·qpos[2]; the free joint's angular
    // velocity (qvel[3..6]) is body-frame, matching the body-frame principal
    // inertia, so rotational KE is ½·Σ Iᵢ·ωᵢ².
    let mech_e = |qpos: &[f64], qvel: &[f64]| -> f64 {
        let pe = mass * g * qpos[2];
        let ke_lin = 0.5 * mass * (qvel[0] * qvel[0] + qvel[1] * qvel[1] + qvel[2] * qvel[2]);
        let ke_ang = 0.5
            * (inertia.x * qvel[3] * qvel[3]
                + inertia.y * qvel[4] * qvel[4]
                + inertia.z * qvel[5] * qvel[5]);
        pe + ke_lin + ke_ang
    };
    let e0 = mass * g * DROP_Z;

    let dt = model.timestep;
    let nsteps = (5.0 / dt).round() as usize;

    // ── GPU rollout ──────────────────────────────────────────────────
    let mut gpu_data = model.make_data();
    gpu_data.qpos[2] = DROP_Z;
    gpu_data.qpos[3] = 1.0;
    let pipeline = match GpuPhysicsPipeline::new(&model, &gpu_data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T31: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    let mut gpu = RolloutInvariants {
        e0,
        max_energy: e0,
        final_energy: e0,
        max_penetration: 0.0,
        max_z: DROP_Z,
        all_finite: true,
    };
    for _ in 0..nsteps {
        pipeline.step(&model, std::slice::from_mut(&mut gpu_data), 1);
        let finite = (0..model.nq).all(|i| gpu_data.qpos[i].is_finite())
            && (0..model.nv).all(|i| gpu_data.qvel[i].is_finite());
        gpu.all_finite &= finite;
        if !finite {
            break;
        }
        gpu.final_energy = mech_e(gpu_data.qpos.as_slice(), gpu_data.qvel.as_slice());
        gpu.max_energy = gpu.max_energy.max(gpu.final_energy);
        gpu.max_penetration = gpu
            .max_penetration
            .max((RADIUS - gpu_data.qpos[2]).max(0.0));
        gpu.max_z = gpu.max_z.max(gpu_data.qpos[2]);
    }
    gpu.assert("GPU");

    // ── CPU rollout ──────────────────────────────────────────────────
    let mut cpu_data = model.make_data();
    cpu_data.qpos[2] = DROP_Z;
    cpu_data.qpos[3] = 1.0;
    let mut cpu = RolloutInvariants {
        e0,
        max_energy: e0,
        final_energy: e0,
        max_penetration: 0.0,
        max_z: DROP_Z,
        all_finite: true,
    };
    for _ in 0..nsteps {
        cpu_data.step(&model).expect("CPU step failed");
        let finite = (0..model.nq).all(|i| cpu_data.qpos[i].is_finite())
            && (0..model.nv).all(|i| cpu_data.qvel[i].is_finite());
        cpu.all_finite &= finite;
        if !finite {
            break;
        }
        cpu.final_energy = mech_e(cpu_data.qpos.as_slice(), cpu_data.qvel.as_slice());
        cpu.max_energy = cpu.max_energy.max(cpu.final_energy);
        cpu.max_penetration = cpu
            .max_penetration
            .max((RADIUS - cpu_data.qpos[2]).max(0.0));
        cpu.max_z = cpu.max_z.max(cpu_data.qpos[2]);
    }
    cpu.assert("CPU");
}

// ── T33: production step() applies implicit joint damping (eulerdamp wiring) ──

/// A damped free body with one geom but NO ground plane: the production
/// `GpuPhysicsPipeline::step()` runs the eulerdamp stage with zero contacts (no
/// collision pairs), so GPU and CPU integrate the SAME smooth damped trajectory
/// (no SDF-vs-analytic divergence). Two assertions:
/// 1. GPU-damped tracks CPU-damped (the eulerdamp wiring + solve are correct);
/// 2. GPU-damped differs materially from the UNDAMPED trajectory — proving
///    `step()` now actually applies damping. Before this wiring, `step()`
///    silently integrated undamped, so the GPU would have matched (2)'s undamped
///    reference instead.
///
/// Gravity is zeroed so the test isolates pure damping (exponential velocity
/// decay from the initial qvel), with no fall confounding the comparison.
#[test]
fn t33_step_applies_implicit_damping() {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    // One geom so the orchestrator's collision/constraint binding is valid; no
    // ground plane ⇒ no contact pairs ⇒ no collision divergence.
    add_sdf_sphere_geom(&mut model, 1, 0.5, 12);
    model.gravity = Vector3::zeros();

    // Undamped reference = the model before damping is added.
    let undamped = model.clone();

    // Add damping the production way: `dof_damping` drives the explicit passive
    // damper force −D·q̇ (qfrc_passive → qfrc_smooth on the CPU), and
    // `compute_implicit_params` derives `implicit_damping` (the (M + h·D) term,
    // and what the GPU uploads as per-DOF damping). Setting only one would make
    // the CPU and GPU see inconsistent damping.
    for i in 0..model.nv {
        model.dof_damping[i] = 3.0;
    }
    model.compute_implicit_params();

    let nsteps = 200usize;
    // Nonzero linear + angular velocity so the damper force −D·q̇ is live.
    let set_init = |d: &mut Data| {
        d.qpos[2] = 5.0;
        d.qpos[3] = 1.0; // identity quaternion [w,x,y,z]
        for (i, v) in [0.8, -0.5, 0.3, 1.2, -0.7, 0.4].into_iter().enumerate() {
            d.qvel[i] = v;
        }
    };

    // GPU damped trajectory via the production step() path.
    let mut gpu_data = model.make_data();
    set_init(&mut gpu_data);
    let pipeline = match GpuPhysicsPipeline::new(&model, &gpu_data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T33: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };
    for _ in 0..nsteps {
        pipeline.step(&model, std::slice::from_mut(&mut gpu_data), 1);
    }

    // CPU damped trajectory (reference).
    let mut cpu_data = model.make_data();
    set_init(&mut cpu_data);
    for _ in 0..nsteps {
        cpu_data.step(&model).expect("CPU damped step");
    }

    // CPU UNDAMPED trajectory (the pre-wiring behavior step() used to produce).
    let mut un_data = undamped.make_data();
    set_init(&mut un_data);
    for _ in 0..nsteps {
        un_data.step(&undamped).expect("CPU undamped step");
    }

    let qvel_diff = |a: &Data, b: &Data| {
        (0..model.nv)
            .map(|i| (a.qvel[i] - b.qvel[i]).abs())
            .fold(0.0_f64, f64::max)
    };

    // (1) GPU-damped tracks CPU-damped: the eulerdamp solve is correct.
    let gpu_vs_cpu = qvel_diff(&gpu_data, &cpu_data);
    // (2) GPU-damped is far from the undamped trajectory: damping actually fired.
    let damped_vs_undamped = qvel_diff(&gpu_data, &un_data);
    eprintln!(
        "  T33: GPU↔CPU damped max|Δqvel|={gpu_vs_cpu:.3e}, GPU-damped↔undamped max|Δqvel|={damped_vs_undamped:.3e}"
    );

    assert!(
        gpu_vs_cpu < 1e-3,
        "GPU step() damped trajectory diverged from CPU: max|Δqvel|={gpu_vs_cpu:.3e}"
    );
    assert!(
        damped_vs_undamped > 0.1,
        "step() produced ~undamped motion (max|Δqvel| vs undamped only {damped_vs_undamped:.3e}) \
         — eulerdamp not applied"
    );
}

// ── T32: Sustained multi-substep stress test ──────────────────────────

#[test]
fn t32_sustained_multi_substep() {
    // Run many batched substeps over 2 seconds — stress tests the
    // single-submit pattern over a sustained period.
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut data = model.make_data();
    data.qpos[2] = 10.0;
    data.qpos[3] = 1.0;

    let pipeline = match GpuPhysicsPipeline::new(&model, &data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T32: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    let dt = model.timestep;
    let total_time = 2.0;
    let substeps_per_batch: u32 = 10;
    let num_batches = (total_time / (dt * f64::from(substeps_per_batch))).round() as usize;

    for batch in 0..num_batches {
        pipeline.step(&model, std::slice::from_mut(&mut data), substeps_per_batch);

        // Check stability every batch
        for i in 0..model.nq {
            assert!(
                data.qpos[i].is_finite(),
                "Batch {batch}: qpos[{i}] = {} is not finite",
                data.qpos[i]
            );
            assert!(
                data.qpos[i].abs() < 1000.0,
                "Batch {batch}: qpos[{i}] = {} exploded",
                data.qpos[i]
            );
        }
    }

    let final_z = data.qpos[2];
    eprintln!(
        "  T32: after {num_batches} batches of {substeps_per_batch} substeps, z = {final_z:.4}"
    );

    // Sphere should not have fallen through ground
    assert!(final_z > -1.0, "Sphere fell through ground: z = {final_z}");

    // Time should have advanced correctly
    let expected_time = (num_batches as u32 * substeps_per_batch) as f64 * dt;
    let time_err = (data.time - expected_time).abs();
    assert!(
        time_err < 1e-6,
        "Time error: {time_err:.2e} (expected {expected_time:.4}, got {:.4})",
        data.time
    );

    eprintln!("  T32 passed: sustained multi-substep stable over {total_time}s");
}

// ── T34: Batched (n_env>1) full substep INCLUDING real GPU collision ──────
//
// The injected-contact conformance suite (contact_conformance_tests) skips
// collision because GPU SDF-cell contacts never match CPU analytic contacts. To
// validate COLLISION batching we use a GPU-only oracle instead: a batched env_k
// must match a SINGLE-env GPU run of the same state, run through the IDENTICAL
// substep helper — so n_env=1 is byte-identical and the only difference under test
// is the env stride. qpos/qvel after one substep are order-independent through the
// solve, so the non-deterministic per-cell contact ordering washes out.

/// Self-validating allowlist for the batched-collision substep test, keyed by env.
/// An armed env that suddenly MATCHES fails the test, so a fix that makes a env
/// conform forces the arm's removal here (mirrors the constraint suite's allowlist).
fn known_collision_batch_divergence(env: usize) -> Option<&'static str> {
    // **Empty — collision (aabb + sdf_*_narrow) is now env-strided**, so every env's
    // batched substep matches its single-env run. Self-validating: re-arm an env here
    // only if a NEW collision-batch divergence is exposed.
    let _ = env;
    None
}

/// Run ONE full GPU substep (fk → crba → velFK → rne → smooth → collision →
/// constraint → integrate) over `datas.len()` environments, returning each env's
/// (qpos, qvel). Mirrors `orchestrator::encode_substep` with per-env buffer clears
/// sized ×n_env. The fixture is undamped, so the eulerdamp stage is omitted (as the
/// orchestrator does when `has_damping` is false).
fn run_one_substep_batched(
    ctx: &GpuContext,
    model: &Model,
    datas: &[&Data],
) -> Vec<(Vec<f32>, Vec<f32>)> {
    let n_env = datas.len() as u32;
    let nq = model.nq;
    let nv = model.nv;
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new_batched(
        ctx,
        &model_buf,
        n_env,
        datas,
        super::types::MAX_PIPELINE_CONTACTS,
    );

    let fk = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);
    let smooth = GpuSmoothPipeline::new(ctx, &model_buf, &state_buf);
    let collision = GpuCollisionPipeline::new(ctx, model, &model_buf, &state_buf);
    let constraint = GpuConstraintPipeline::new(ctx, &model_buf, &state_buf, model);
    let integrate = GpuIntegratePipeline::new(ctx, &model_buf, &state_buf);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("batched collision substep"),
        });
    // Per-substep clears (mirror orchestrator::encode_substep, sized ×n_env).
    encoder.clear_buffer(&state_buf.body_cfrc, 0, None);
    encoder.clear_buffer(&state_buf.qfrc_bias, 0, None);
    if nv > 0 {
        let nv_sq_bytes = u64::from(n_env) * (nv as u64) * (nv as u64) * 4;
        encoder.clear_buffer(&state_buf.qm, 0, Some(nv_sq_bytes));
        encoder.clear_buffer(&state_buf.qfrc_applied, 0, None);
        encoder.clear_buffer(&state_buf.qfrc_actuator, 0, None);
        encoder.clear_buffer(&state_buf.qfrc_passive, 0, None);
    }
    fk.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    smooth.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    collision.encode(&mut encoder, &state_buf);
    constraint.encode(&mut encoder, &state_buf);
    integrate.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);

    let qpos_all = readback_f32s(ctx, &state_buf.qpos, nq * datas.len());
    let qvel_all = readback_f32s(ctx, &state_buf.qvel, nv * datas.len());
    (0..datas.len())
        .map(|k| {
            (
                qpos_all[k * nq..(k + 1) * nq].to_vec(),
                qvel_all[k * nv..(k + 1) * nv].to_vec(),
            )
        })
        .collect()
}

#[test]
fn t34_gpu_batched_collision_substep_matches_single() {
    let ctx = gpu_or_skip!();

    // SDF sphere on a ground plane; two envs at DISTINCT penetrating heights.
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16);

    let make = |z: f64, vz: f64| {
        let mut d = model.make_data();
        d.qpos[2] = z; // sphere centre z; < radius ⇒ penetrates the z=0 plane
        d.qpos[3] = 1.0; // identity quaternion
        d.qvel[2] = vz;
        d
    };
    let data0 = make(3.0, 0.0);
    let data1 = make(2.5, -0.5); // deeper + moving ⇒ a distinct result

    // Single-env GPU oracle: each state run alone through the identical substep.
    let single0 = run_one_substep_batched(&ctx, &model, &[&data0]).remove(0);
    let single1 = run_one_substep_batched(&ctx, &model, &[&data1]).remove(0);

    // Discriminator: the two single-env results must genuinely differ, else a stage
    // reading env 0's data for env 1 could not be caught (the run would be vacuous).
    let differ = single0
        .0
        .iter()
        .zip(&single1.0)
        .any(|(a, b)| (a - b).abs() > 1e-3);
    assert!(
        differ,
        "env 0/1 single-env results are near-identical — the batched check is vacuous"
    );

    // Batched run: both envs together on the shared batched buffers.
    let batched = run_one_substep_batched(&ctx, &model, &[&data0, &data1]);
    let oracle = [single0, single1];

    let mut fails: Vec<String> = Vec::new();
    for env in 0..2 {
        let (bq, bv) = &batched[env];
        let (sq, sv) = &oracle[env];
        let mut env_fails: Vec<String> = Vec::new();
        for i in 0..model.nq {
            let d = (bq[i] - sq[i]).abs();
            if !(d < 1e-4) {
                env_fails.push(format!(
                    "env{env} qpos[{i}] batched={:.6} single={:.6} d={d:.2e}",
                    bq[i], sq[i]
                ));
            }
        }
        for i in 0..model.nv {
            let d = (bv[i] - sv[i]).abs();
            if !(d < 1e-3) {
                env_fails.push(format!(
                    "env{env} qvel[{i}] batched={:.6} single={:.6} d={d:.2e}",
                    bv[i], sv[i]
                ));
            }
        }
        match known_collision_batch_divergence(env) {
            Some(reason) => {
                assert!(
                    !env_fails.is_empty(),
                    "env{env} is on the collision-batch allowlist ({reason}) but now \
                     MATCHES — collision was fixed; remove its arm."
                );
                eprintln!(
                    "  env{env}: EXPECTED-DIVERGENT ({} entries, {reason})",
                    env_fails.len()
                );
            }
            None => {
                if env_fails.is_empty() {
                    eprintln!("  env{env}: passed");
                } else {
                    fails.extend(env_fails);
                }
            }
        }
    }

    assert!(
        fails.is_empty(),
        "batched collision substep diverged from single-env:\n  {}",
        fails.join("\n  ")
    );
}

// ── T35: Batched orchestrator step() conformance ──────────────────────────
//
// Exercises the PRODUCTION `GpuPhysicsPipeline::step()` in batched mode — the
// write-params-once + `encode_substep` path the orchestrator actually uses, not
// the hand-rolled `run_one_substep_batched` helper above. Two envs stepped
// together must each match the same env stepped ALONE through a single-env
// `step()`, so the only thing under test is the n_env stride threaded through
// the whole orchestrator: batched upload, per-env buffer clears, multi-substep
// single-submit, and per-env readback/writeback.
//
// A free-fall fixture (sphere far above the plane, never contacting over the
// rollout) keeps the dynamics exact and deterministic, isolating the batching
// plumbing from the non-deterministic per-cell contact ordering that T34 covers
// separately. With no contact the per-env math is independent and identical, so
// the gate is tight (1e-6).
#[test]
fn t35_orchestrator_batched_step_matches_single() {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    // Two envs that genuinely differ: distinct heights + velocities, both far
    // enough above the plane that the rollout never contacts it (pure free fall).
    let make = |z: f64, vz: f64| {
        let mut d = model.make_data();
        d.qpos[2] = z;
        d.qpos[3] = 1.0; // identity quaternion
        d.qvel[2] = vz;
        d
    };
    let d0 = make(20.0, 0.0);
    let d1 = make(18.0, -2.0);
    let substeps = 50;

    // Build the batched pipeline first — this doubles as the GPU-availability gate.
    let pipe = match GpuPhysicsPipeline::new_batched(&model, &[&d0, &d1]) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T35: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("batched pipeline creation failed: {e}"),
    };

    // Single-env oracle: each env stepped alone through the production step()
    // (GPU confirmed present by the batched build above).
    let step_single = |init: &Data| -> Data {
        let p = GpuPhysicsPipeline::new(&model, init).expect("single-env pipeline");
        let mut d = init.clone();
        p.step(&model, std::slice::from_mut(&mut d), substeps);
        d
    };
    let single0 = step_single(&d0);
    let single1 = step_single(&d1);

    // Discriminator: the two single-env results must differ, else env0 reading
    // env1's slice (or vice versa) could not be caught — the check would be vacuous.
    assert!(
        (single0.qpos[2] - single1.qpos[2]).abs() > 1e-3,
        "env0/env1 single-env results near-identical — batched check is vacuous"
    );

    // Batched run: both envs together on the shared batched buffers.
    let mut datas = vec![d0, d1];
    pipe.step(&model, &mut datas, substeps);

    let oracle = [single0, single1];
    for env in 0..2 {
        for i in 0..model.nq {
            let diff = (datas[env].qpos[i] - oracle[env].qpos[i]).abs();
            assert!(
                diff < 1e-6,
                "env{env} qpos[{i}] batched={:.9} single={:.9} d={diff:.2e}",
                datas[env].qpos[i],
                oracle[env].qpos[i]
            );
        }
        for i in 0..model.nv {
            let diff = (datas[env].qvel[i] - oracle[env].qvel[i]).abs();
            assert!(
                diff < 1e-6,
                "env{env} qvel[{i}] batched={:.9} single={:.9} d={diff:.2e}",
                datas[env].qvel[i],
                oracle[env].qvel[i]
            );
        }
        assert!(
            (datas[env].time - oracle[env].time).abs() < 1e-12,
            "env{env} time advanced inconsistently"
        );
    }

    eprintln!("  T35 passed: batched step() matches single-env per env");
}

// ── T36: Batched orchestrator step() conformance WITH contact ─────────────
//
// Closes the gap T35 leaves open: T35 is free-fall, so it never drives the
// production `step()` / `encode_substep` path with active contacts, and T34
// exercises contact-batching only through the hand-rolled `run_one_substep_batched`
// helper — not the real orchestrator. This test runs the PRODUCTION `step()` in
// batched mode with two spheres penetrating the ground plane at distinct depths,
// so the per-substep contact + constraint clears and the collision/constraint
// stages are validated end-to-end across n_env.
//
// ONE substep only: per-cell contact ordering is non-deterministic, but qpos/qvel
// after a single substep are order-independent through the solve (the same property
// T34 relies on), so the batched env-k vs single-env-k comparison is stable. The
// gate matches T34's contact tolerances (1e-4 qpos / 1e-3 qvel), looser than T35's
// exact free-fall gate.
#[test]
fn t36_orchestrator_batched_step_with_contact_matches_single() {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16);

    // Two envs penetrating the z=0 plane at distinct depths + velocities, so the
    // contact response genuinely differs between them.
    let make = |z: f64, vz: f64| {
        let mut d = model.make_data();
        d.qpos[2] = z; // sphere centre z; < radius ⇒ penetrates the plane
        d.qpos[3] = 1.0; // identity quaternion
        d.qvel[2] = vz;
        d
    };
    let d0 = make(3.0, 0.0);
    let d1 = make(2.5, -0.5); // deeper + moving ⇒ a distinct contact result

    // Batched pipeline first — doubles as the GPU-availability gate.
    let pipe = match GpuPhysicsPipeline::new_batched(&model, &[&d0, &d1]) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T36: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("batched pipeline creation failed: {e}"),
    };

    // Single-env oracle: each env stepped alone through the production step().
    let step_single = |init: &Data| -> Data {
        let p = GpuPhysicsPipeline::new(&model, init).expect("single-env pipeline");
        let mut d = init.clone();
        p.step(&model, std::slice::from_mut(&mut d), 1);
        d
    };
    let single0 = step_single(&d0);
    let single1 = step_single(&d1);

    // Discriminator: the two contact responses must differ, else the cross-env
    // check would be vacuous.
    assert!(
        (single0.qvel[2] - single1.qvel[2]).abs() > 1e-3,
        "env0/env1 single-env contact results near-identical — batched check is vacuous"
    );

    // Batched run: both envs together through the production step().
    let mut datas = vec![d0, d1];
    pipe.step(&model, &mut datas, 1);

    let oracle = [single0, single1];
    for env in 0..2 {
        for i in 0..model.nq {
            let diff = (datas[env].qpos[i] - oracle[env].qpos[i]).abs();
            assert!(
                diff < 1e-4,
                "env{env} qpos[{i}] batched={:.6} single={:.6} d={diff:.2e}",
                datas[env].qpos[i],
                oracle[env].qpos[i]
            );
        }
        for i in 0..model.nv {
            let diff = (datas[env].qvel[i] - oracle[env].qvel[i]).abs();
            assert!(
                diff < 1e-3,
                "env{env} qvel[{i}] batched={:.6} single={:.6} d={diff:.2e}",
                datas[env].qvel[i],
                oracle[env].qvel[i]
            );
        }
    }

    eprintln!("  T36 passed: batched step() with contact matches single-env per env");
}

// ── T37: Large-batch allocation + step (scale wall #3 regression guard) ────
//
// Before the batched-capacity fix, the contact + constraint buffers were sized
// `n_env × MAX_PIPELINE_CONTACTS` (32_768/env) and blew past the GPU's max-buffer
// limit at n_env ≈ 16–32 — so `new_batched`/`step` PANICKED for any real batch.
// This guards that a large batch (n_env = 256) both ALLOCATES and STEPS with active
// contact, producing finite, per-env-distinct results. It would have panicked at
// construction pre-fix, so it pins the wall open.
#[test]
fn t37_large_batch_allocates_and_steps() {
    const N_ENV: usize = 256;

    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16);

    // Distinct penetration depths per env so the contact responses differ.
    let datas: Vec<Data> = (0..N_ENV)
        .map(|i| {
            let mut d = model.make_data();
            d.qpos[2] = 3.0 + (i as f64) * 0.001;
            d.qpos[3] = 1.0;
            d
        })
        .collect();
    let refs: Vec<&Data> = datas.iter().collect();

    let pipe = match GpuPhysicsPipeline::new_batched(&model, &refs) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T37: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("large-batch pipeline creation failed: {e}"),
    };
    drop(refs);

    let mut datas = datas;
    pipe.step(&model, &mut datas, 1);

    // Every env's result must be finite (a dropped/garbage env would NaN).
    for (env, d) in datas.iter().enumerate() {
        for i in 0..model.nq {
            assert!(
                d.qpos[i].is_finite(),
                "env{env} qpos[{i}] not finite: {}",
                d.qpos[i]
            );
        }
    }
    // Distinct inputs → distinct outputs (guards a stage reading env 0 for all envs).
    assert!(
        (datas[0].qpos[2] - datas[N_ENV - 1].qpos[2]).abs() > 0.0,
        "first/last env identical — batching may be collapsed"
    );

    eprintln!("  T37 passed: n_env={N_ENV} allocates + steps with contact");
}

// ── T38: Chunked substep submits (long-rollout hang, wall #2 regression guard) ──
//
// Before chunking, `step(num_substeps)` encoded EVERY substep into one command
// buffer (~14 cmds/substep) and submitted once; past ~100 substeps that overran a
// backend command limit and the synchronous readback poll then blocked forever
// (probe: 75 OK, 100 hang). `step()` now submits in bounded chunks of
// `SUBSTEP_CHUNK` (=32). This guards two things at once:
//   1. NO-HANG: `step(150)` (> SUBSTEP_CHUNK, the regime that used to hang) simply
//      completing is the regression guard — pre-fix this test would never return.
//   2. BYTE-IDENTICAL: chunk boundaries must not change the trajectory. State lives
//      in `state_bufs` across the ordered submits, and the f32→f64→f32 round-trip
//      between per-step uploads is lossless, so a single `step(150)` must match
//      150× `step(1)` exactly. A contactless free-fall fixture keeps the math the
//      simplest deterministic path (no contact set, no solver iteration variance).
#[test]
fn t38_chunked_substeps_no_hang_and_byte_identical() {
    const SUBSTEPS: u32 = 150; // > SUBSTEP_CHUNK (32): spans multiple chunks/submits.

    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    // One geom (no ground plane) → collision finds no pairs, so the body free-falls
    // under gravity. The geom still satisfies the orchestrator's geom-buffer needs.
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    let mut init = model.make_data();
    init.qpos[2] = 50.0;
    init.qpos[3] = 1.0; // unit quaternion (w)

    let pipeline = match GpuPhysicsPipeline::new(&model, &init) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T38: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    // Run A: one chunked call. Completing at all is the no-hang guard.
    let mut data_batch = init.clone();
    pipeline.step(&model, std::slice::from_mut(&mut data_batch), SUBSTEPS);

    // Run B: SUBSTEPS single-substep calls (each one chunk).
    let mut data_iter = init.clone();
    for _ in 0..SUBSTEPS {
        pipeline.step(&model, std::slice::from_mut(&mut data_iter), 1);
    }

    // Byte-identical: chunk-boundary independence (exact f64 equality).
    for i in 0..model.nq {
        assert_eq!(
            data_batch.qpos[i], data_iter.qpos[i],
            "qpos[{i}] differs: chunked step({SUBSTEPS}) {} vs {SUBSTEPS}× step(1) {}",
            data_batch.qpos[i], data_iter.qpos[i]
        );
    }
    for i in 0..model.nv {
        assert_eq!(
            data_batch.qvel[i], data_iter.qvel[i],
            "qvel[{i}] differs: chunked step({SUBSTEPS}) {} vs {SUBSTEPS}× step(1) {}",
            data_batch.qvel[i], data_iter.qvel[i]
        );
    }

    // Vacuity: the body must actually have fallen (otherwise the comparison is trivial).
    assert!(
        data_batch.qpos[2] < init.qpos[2] - 1.0,
        "free-fall did not advance: z {} → {}",
        init.qpos[2],
        data_batch.qpos[2]
    );

    eprintln!(
        "  T38 passed: step({SUBSTEPS}) completes (no hang) and is byte-identical to {SUBSTEPS}× step(1)"
    );
}

// ── T39: No-geom model steps without a binding panic (wall #1 regression guard) ──
//
// `upload_structs` backed an EMPTY struct array (a geom-less model's `geoms`)
// with a bare 16-byte stub, but the shader binds `array<GeomModel>` whose
// `min_binding_size` is one element = 96 bytes. So `step()` PANICKED at the
// first dispatch ("Buffer is bound with size 16 where the shader expects 96")
// for any model with `ngeom == 0` — a valid model the public API must accept.
// Every other fixture has geoms, so this was never exercised. The fix backs an
// empty buffer with one zeroed element; this guards that a geom-less free body
// constructs AND steps under gravity without panicking.
#[test]
fn t39_no_geom_model_steps() {
    // free_body adds NO geoms (ngeom == 0) — the degenerate-but-valid case.
    let model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    assert_eq!(model.ngeom, 0, "fixture must be geom-less to guard wall #1");

    let mut data = model.make_data();
    data.qpos[2] = 5.0;
    data.qpos[3] = 1.0; // unit quaternion (w)

    let pipeline = match GpuPhysicsPipeline::new(&model, &data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T39: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    // Pre-fix this dispatch panicked on the geoms binding (size 16 vs 96).
    let z0 = data.qpos[2];
    pipeline.step(&model, std::slice::from_mut(&mut data), 10);

    // Vacuity: with no contacts the body free-falls, so z must drop.
    assert!(
        data.qpos[2] < z0,
        "no-geom free body did not fall: z {z0} → {}",
        data.qpos[2]
    );
    assert!(data.qpos[2].is_finite(), "qpos[2] not finite");

    eprintln!(
        "  T39 passed: geom-less model steps (z {z0} → {:.4})",
        data.qpos[2]
    );
}
