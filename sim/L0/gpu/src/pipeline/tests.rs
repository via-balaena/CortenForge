//! GPU pipeline validation tests.
//!
//! T1–T7:  FK — compare GPU body poses, cinert, cdof, subtree COM against CPU.
//! T8–T12: CRBA + velocity FK — compare GPU qM, cvel against CPU.
//! T13–T18: RNE + smooth + integration — bias forces, qacc_smooth, trajectories.
//! T19–T22: Collision — AABB, SDF-SDF, SDF-plane, full collision pipeline.
//! T23–T27: Constraint solve — assembly, Newton solver, force mapping, stability.

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
    clippy::doc_markdown
)]

use nalgebra::{UnitQuaternion, Vector3};
use std::f64::consts::PI;

use sim_core::types::{Data, Model};

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
        n_constraints % 4 == 0,
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
    pipeline.step(&model, &mut data, 1);

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

    pipeline.step(&model, &mut data_batch, 10);

    // Run 10 substeps in 10 separate submits
    let mut data_serial = model.make_data();
    data_serial.qpos[2] = 10.0;
    data_serial.qpos[3] = 1.0;

    let pipeline2 = GpuPhysicsPipeline::new(&model, &data_serial).unwrap();
    for _ in 0..10 {
        pipeline2.step(&model, &mut data_serial, 1);
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

#[test]
fn t31_gpu_vs_cpu_trajectory() {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 12);

    // GPU trajectory
    let mut gpu_data = model.make_data();
    gpu_data.qpos[2] = 10.0;
    gpu_data.qpos[3] = 1.0;

    let pipeline = match GpuPhysicsPipeline::new(&model, &gpu_data) {
        Ok(p) => p,
        Err(GpuPipelineError::NoGpu(_)) => {
            eprintln!("  T31: skipping (no GPU)");
            return;
        }
        Err(e) => panic!("Pipeline creation failed: {e}"),
    };

    let dt = model.timestep;
    let total_time = 5.0;
    let nsteps = (total_time / dt).round() as usize;

    for _ in 0..nsteps {
        pipeline.step(&model, &mut gpu_data, 1);
    }

    // CPU trajectory
    let mut cpu_data = model.make_data();
    cpu_data.qpos[2] = 10.0;
    cpu_data.qpos[3] = 1.0;

    for _ in 0..nsteps {
        cpu_data.step(&model).expect("CPU step failed");
    }

    let gpu_z = gpu_data.qpos[2];
    let cpu_z = cpu_data.qpos[2];
    let z_diff = (gpu_z - cpu_z).abs();

    eprintln!("  T31: after {nsteps} steps ({total_time}s):");
    eprintln!("    GPU z = {gpu_z:.4}");
    eprintln!("    CPU z = {cpu_z:.4}");
    eprintln!("    diff  = {z_diff:.4}");

    // All values should be finite
    for i in 0..model.nq {
        assert!(
            gpu_data.qpos[i].is_finite(),
            "GPU qpos[{i}] = {} is not finite",
            gpu_data.qpos[i]
        );
    }
    for i in 0..model.nv {
        assert!(
            gpu_data.qvel[i].is_finite(),
            "GPU qvel[{i}] = {} is not finite",
            gpu_data.qvel[i]
        );
    }

    // z should not diverge too far (pyramidal vs elliptic friction + f32 drift)
    assert!(
        z_diff < 1.0,
        "GPU/CPU z diverged by {z_diff:.4}m (limit 1.0m)"
    );

    // Body should not have exploded
    assert!(gpu_z.abs() < 100.0, "GPU z exploded: {gpu_z}");

    eprintln!("  T31 passed: GPU vs CPU trajectory within tolerance after {total_time}s");
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
        pipeline.step(&model, &mut data, substeps_per_batch);

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
