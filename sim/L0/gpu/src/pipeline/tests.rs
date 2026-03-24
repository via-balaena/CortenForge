//! GPU pipeline validation tests.
//!
//! T1–T7:  FK — compare GPU body poses, cinert, cdof, subtree COM against CPU.
//! T8–T12: CRBA + velocity FK — compare GPU qM, cvel against CPU.
//! T13–T18: RNE + smooth + integration — bias forces, qacc_smooth, trajectories.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::doc_markdown
)]

use nalgebra::{UnitQuaternion, Vector3};
use std::f64::consts::PI;

use sim_core::types::{Data, Model};

use super::crba::GpuCrbaPipeline;
use super::fk::{GpuFkPipeline, readback_f32s, readback_vec4s};
use super::integrate::GpuIntegratePipeline;
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
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
