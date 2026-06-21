//! GPU↔CPU dynamics conformance harness (n_env = 1).
//!
//! Where `tests.rs` (T1–T32) validates the GPU shaders against hand-picked
//! pendulum / free-body fixtures, this suite drives the GPU pipeline over the
//! SAME joint × topology matrix the CPU analytic-vs-FD transition harness uses
//! ([`sim_core::test_fixtures::conformance::dynamics_conformance_matrix`]). One
//! shared fixture list means a GPU shader bug can no longer hide in a topology
//! that no GPU fixture happens to exercise — the exact failure mode that let a
//! run of CPU physics fixes (#346/#347/#348/#350/#351/#352) silently diverge
//! the GPU port (see `project-gpu-shader-conformance-gap`).
//!
//! For each case we run FK → CRBA → velocity-FK → RNE on the GPU and compare the
//! full chain against `data.forward()`:
//!   `xpos`/`xquat` · `subtree_com` · `cdof` · `cinert` · `cvel` · `qM` ·
//!   `cacc_bias` · `qfrc_bias`
//! at a non-degenerate operating point (off-axis COM, tilted quaternions,
//! NONZERO `qvel`, gravity on) so every Coriolis / subspace / gravity term is
//! live. The `cdof` oracle is computed from the CPU's PARTIAL frames
//! (`data.xaxis`/`data.xanchor`, plus the ball/free `[R]` blocks) rather than
//! `data.cdof`, which is vestigial (allocated as zeros, never written).
//!
//! Scope: n_env = 1, no contacts, no damping (Slice 1). The tendon case is
//! filtered out — there is no GPU tendon shader.

#![cfg(test)]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::needless_range_loop,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::too_many_arguments,
    clippy::too_many_lines,
    // `!(diff < tol)` is intentional: it treats NaN as a failure (unlike `>=`).
    clippy::neg_cmp_op_on_partial_ord
)]

// CI safety scanner: this file is test-only (gated by #[cfg(test)] in mod.rs).
#[cfg(test)]
const _: () = ();

use nalgebra::{DVector, Vector3};

use sim_core::mj_integrate_pos_explicit;
use sim_core::test_fixtures::conformance::{
    ConformanceCase, damped_conformance_matrix, dynamics_conformance_matrix,
};
use sim_core::types::{Data, MjJointType, Model};

use super::crba::GpuCrbaPipeline;
use super::eulerdamp::GpuEulerdampPipeline;
use super::fk::{GpuFkPipeline, readback_f32s, readback_vec4s};
use super::integrate::GpuIntegratePipeline;
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
use super::velocity_fk::GpuVelocityFkPipeline;
use crate::context::GpuContext;

/// Try to create a GPU context; skip the test (return) if none is available.
macro_rules! gpu_or_skip {
    () => {
        match GpuContext::new() {
            Ok(ctx) => ctx,
            Err(e) => {
                eprintln!("  Skipping conformance suite (no GPU): {e}");
                return;
            }
        }
    };
}

/// FK-stage tolerance (poses, subspace, inertia, subtree COM). f32 GPU vs f64
/// CPU over the matrix's small models.
const TOL_KINEMATIC: f32 = 1e-4;
/// RNE-stage tolerance (velocity-product / Coriolis / mass-matrix / bias).
/// Looser to absorb f32 accumulation across the deeper chains (nv up to 8).
const TOL_DYNAMIC: f32 = 1e-3;

/// CPU motion-subspace column(s) for one scalar DOF, in the same `[ang; lin]`
/// layout the GPU `cdof` buffer uses (2×vec4 per DOF). Mirrors
/// `joint_motion_subspace` but read out per-DOF from the PARTIAL frames — the
/// only trustworthy oracle, since `data.cdof` is never populated.
fn cpu_cdof_dof(model: &Model, data: &Data, dof: usize) -> [f32; 6] {
    let jnt = model.dof_jnt[dof];
    let body = model.jnt_body[jnt];
    let local = dof - model.jnt_dof_adr[jnt];
    let mk = |ang: Vector3<f64>, lin: Vector3<f64>| {
        [
            ang.x as f32,
            ang.y as f32,
            ang.z as f32,
            lin.x as f32,
            lin.y as f32,
            lin.z as f32,
        ]
    };
    match model.jnt_type[jnt] {
        MjJointType::Hinge => {
            let axis = data.xaxis[jnt];
            let r = data.xpos[body] - data.xanchor[jnt];
            mk(axis, axis.cross(&r))
        }
        MjJointType::Slide => mk(Vector3::zeros(), data.xaxis[jnt]),
        MjJointType::Ball => {
            // Angular column = body-orientation matrix column `local`; linear = 0
            // (rotation about the body origin, anchor at origin for these models).
            let rot = data.xquat[body].to_rotation_matrix().into_inner();
            let col = Vector3::new(rot[(0, local)], rot[(1, local)], rot[(2, local)]);
            mk(col, Vector3::zeros())
        }
        MjJointType::Free => {
            if local < 3 {
                // Linear DOF: world-basis translation.
                let mut e = Vector3::zeros();
                e[local] = 1.0;
                mk(Vector3::zeros(), e)
            } else {
                // Angular DOF: body-orientation matrix column `local - 3`.
                let rot = data.xquat[body].to_rotation_matrix().into_inner();
                let c = local - 3;
                let col = Vector3::new(rot[(0, c)], rot[(1, c)], rot[(2, c)]);
                mk(col, Vector3::zeros())
            }
        }
    }
}

/// Run FK → CRBA → velocity-FK → RNE on the GPU for `model` at the operating
/// point already loaded into `data`, returning the populated state buffers.
fn run_through_rne(ctx: &GpuContext, model: &Model, data: &Data) -> GpuStateBuffers {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);
    let fk = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("conformance fk+crba+vel+rne"),
        });
    fk.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);
    state_buf
}

/// Run FK → CRBA → velocity-FK → RNE on the GPU for `n_env` environments whose
/// initial states are `per_env`, returning the populated batched state buffers.
/// The pipelines pick up `state.n_env` and dispatch one workgroup column per env.
fn run_through_rne_batched(ctx: &GpuContext, model: &Model, per_env: &[&Data]) -> GpuStateBuffers {
    let n_env = per_env.len() as u32;
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new_batched(
        ctx,
        &model_buf,
        n_env,
        per_env,
        super::types::MAX_PIPELINE_CONTACTS,
    );
    let fk = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("batched conformance fk+crba+vel+rne"),
        });
    fk.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);
    state_buf
}

/// Failure sink: one entry per diverging scalar, so a single run yields the
/// complete divergence ledger instead of stopping at the first mismatch.
type Failures = Vec<String>;

/// Cases with a KNOWN, pre-existing GPU↔CPU divergence (filed in the divergence
/// ledger `project-gpu-shader-conformance-gap`), returning the reason. These are
/// out of Slice-1 scope to fix (no shader/infra change). The entry is
/// self-validating: a known-divergent case that suddenly MATCHES fails the test
/// (`known divergence appears FIXED`) so this allowlist can never silently mask a
/// fix — mirrors the CPU transition harness's `known_limit` discipline.
fn known_gpu_divergence(name: &str) -> Option<&'static str> {
    // No cases are currently known-divergent: the n_env=1 smooth/kinematic path
    // matches CPU across the whole conformance matrix. To re-arm, return
    // `Some("reason (ledger ref)")` for the diverging case name; `finish_case`
    // self-validates that an allowlisted case STILL diverges, so this can never
    // silently mask a fix (mirrors the CPU transition harness's `known_limit`).
    let _ = name;
    None
}

/// Record `|gpu − cpu| ≥ tol` (NaN-safe via `!(< tol)`).
fn check(fails: &mut Failures, gpu: f32, cpu: f32, tol: f32, what: String) {
    if !((gpu - cpu).abs() < tol) {
        fails.push(format!(
            "{what}: GPU={gpu:.6} CPU={cpu:.6} err={:.2e}",
            (gpu - cpu).abs()
        ));
    }
}

/// Compare one GPU `[ang; lin]` spatial value (2 consecutive vec4) against a CPU
/// `SpatialVector` (`[ang(0..3); lin(3..6)]`).
fn check_spatial(
    fails: &mut Failures,
    gpu: &[[f32; 4]],
    base: usize,
    cpu: &nalgebra::Vector6<f64>,
    tol: f32,
    case: &str,
    field: &str,
    idx: usize,
) {
    for k in 0..3 {
        check(
            fails,
            gpu[base][k],
            cpu[k] as f32,
            tol,
            format!("[{case}] {field}[{idx}] angular[{k}]"),
        );
        check(
            fails,
            gpu[base + 1][k],
            cpu[k + 3] as f32,
            tol,
            format!("[{case}] {field}[{idx}] linear[{k}]"),
        );
    }
}

/// Compare GPU `body_cinert` (compact `[mass, h, I_com]`) against the CPU
/// `Matrix6` cinert, reversing the parallel-axis term to recover `I_com`
/// (mirrors `tests.rs` T4). `h = xipos − xpos`.
fn check_cinert(
    fails: &mut Failures,
    ctx: &GpuContext,
    state: &GpuStateBuffers,
    data: &Data,
    model: &Model,
    case: &str,
    env: usize,
    n_env: usize,
) {
    let gpu = readback_vec4s(ctx, &state.body_cinert, model.nbody * 3 * n_env);
    let env_off = env * model.nbody * 3;
    for b in 1..model.nbody {
        let base = env_off + b * 3;
        let gpu_mass = gpu[base][0];
        let gpu_h = [gpu[base][1], gpu[base][2], gpu[base][3]];
        let gpu_i = [
            gpu[base + 1][0],
            gpu[base + 1][1],
            gpu[base + 1][2],
            gpu[base + 1][3],
            gpu[base + 2][0],
            gpu[base + 2][1],
        ];

        let cpu = &data.cinert[b];
        let cpu_mass = cpu[(3, 3)];
        let h = data.xipos[b] - data.xpos[b];
        let hh = h.dot(&h);
        let cpu_i = [
            cpu[(0, 0)] - cpu_mass * (hh - h.x * h.x),
            cpu[(0, 1)] - cpu_mass * (0.0 - h.x * h.y),
            cpu[(0, 2)] - cpu_mass * (0.0 - h.x * h.z),
            cpu[(1, 1)] - cpu_mass * (hh - h.y * h.y),
            cpu[(1, 2)] - cpu_mass * (0.0 - h.y * h.z),
            cpu[(2, 2)] - cpu_mass * (hh - h.z * h.z),
        ];

        check(
            fails,
            gpu_mass,
            cpu_mass as f32,
            TOL_DYNAMIC,
            format!("[{case}] cinert[{b}] mass"),
        );
        for k in 0..3 {
            check(
                fails,
                gpu_h[k],
                [h.x, h.y, h.z][k] as f32,
                TOL_DYNAMIC,
                format!("[{case}] cinert[{b}] h[{k}]"),
            );
        }
        for k in 0..6 {
            check(
                fails,
                gpu_i[k],
                cpu_i[k] as f32,
                TOL_DYNAMIC,
                format!("[{case}] cinert[{b}] I[{k}]"),
            );
        }
    }
}

/// Compare environment `env` of a (possibly batched) GPU state against the CPU
/// `forward()` result in `data`, field by field, using the Slice-1 oracle table
/// (`xpos`/`xquat` · `subtree_com`/`subtree_mass` · `cinert` · `cdof` · `cvel` ·
/// `cacc_bias` · `qM` · `qfrc_bias`).
///
/// Every per-env buffer is sized `n_env ×` its single-env footprint, so env `k`
/// lives at offset `k · count`; this reads the full batched buffer and slices
/// out `env`'s block. At `n_env = 1` it is the original single-env comparison.
fn compare_env_fields(
    fails: &mut Failures,
    ctx: &GpuContext,
    state: &GpuStateBuffers,
    data: &Data,
    model: &Model,
    name: &str,
    env: usize,
    n_env: usize,
) {
    let nv = model.nv;
    let nbody = model.nbody;
    let off_b = env * nbody; // body-indexed buffers
    let off_v = env * nv; // dof-indexed buffers

    // ── Kinematics ────────────────────────────────────────────────
    let gpu_xpos = readback_vec4s(ctx, &state.body_xpos, nbody * n_env);
    let gpu_xquat = readback_vec4s(ctx, &state.body_xquat, nbody * n_env);
    for b in 0..nbody {
        for k in 0..3 {
            check(
                fails,
                gpu_xpos[off_b + b][k],
                data.xpos[b][k] as f32,
                TOL_KINEMATIC,
                format!("[{name}] xpos[{b}][{k}]"),
            );
        }
        // q and −q are the same rotation: align sign on w.
        let cq = data.xquat[b].as_ref().coords;
        let sign = if gpu_xquat[off_b + b][3] * cq.w as f32 >= 0.0 {
            1.0f32
        } else {
            -1.0
        };
        let cc = [cq.x, cq.y, cq.z, cq.w];
        for k in 0..4 {
            check(
                fails,
                gpu_xquat[off_b + b][k],
                sign * cc[k] as f32,
                TOL_KINEMATIC,
                format!("[{name}] xquat[{b}][{k}]"),
            );
        }
    }

    // subtree_com / subtree_mass
    let gpu_com = readback_vec4s(ctx, &state.subtree_com, nbody * n_env);
    let gpu_smass = readback_f32s(ctx, &state.subtree_mass, nbody * n_env);
    for b in 0..nbody {
        check(
            fails,
            gpu_smass[off_b + b],
            data.subtree_mass[b] as f32,
            TOL_KINEMATIC,
            format!("[{name}] subtree_mass[{b}]"),
        );
        for k in 0..3 {
            check(
                fails,
                gpu_com[off_b + b][k],
                data.subtree_com[b][k] as f32,
                TOL_KINEMATIC,
                format!("[{name}] subtree_com[{b}][{k}]"),
            );
        }
    }

    // cinert (compact, parallel-axis reversed)
    check_cinert(fails, ctx, state, data, model, name, env, n_env);

    // cdof — partial-frame oracle (data.cdof is vestigial)
    let gpu_cdof = readback_vec4s(ctx, &state.cdof, nv * 2 * n_env);
    for dof in 0..nv {
        let cpu = cpu_cdof_dof(model, data, dof);
        for k in 0..3 {
            check(
                fails,
                gpu_cdof[(off_v + dof) * 2][k],
                cpu[k],
                TOL_KINEMATIC,
                format!("[{name}] cdof[{dof}] angular[{k}]"),
            );
            check(
                fails,
                gpu_cdof[(off_v + dof) * 2 + 1][k],
                cpu[k + 3],
                TOL_KINEMATIC,
                format!("[{name}] cdof[{dof}] linear[{k}]"),
            );
        }
    }

    // ── Velocity / dynamics ───────────────────────────────────────
    let gpu_cvel = readback_vec4s(ctx, &state.body_cvel, nbody * 2 * n_env);
    for b in 0..nbody {
        check_spatial(
            fails,
            &gpu_cvel,
            (off_b + b) * 2,
            &data.cvel[b],
            TOL_DYNAMIC,
            name,
            "cvel",
            b,
        );
    }

    let gpu_cacc = readback_vec4s(ctx, &state.body_cacc, nbody * 2 * n_env);
    for b in 0..nbody {
        check_spatial(
            fails,
            &gpu_cacc,
            (off_b + b) * 2,
            &data.cacc_bias[b],
            TOL_DYNAMIC,
            name,
            "cacc_bias",
            b,
        );
    }

    // qM (full matrix), env block of nv·nv
    let gpu_qm = readback_f32s(ctx, &state.qm, nv * nv * n_env);
    let off_qm = env * nv * nv;
    for i in 0..nv {
        for j in 0..nv {
            check(
                fails,
                gpu_qm[off_qm + i * nv + j],
                data.qM[(i, j)] as f32,
                TOL_DYNAMIC,
                format!("[{name}] qM[{i},{j}]"),
            );
        }
    }

    // qfrc_bias (Coriolis + gravity)
    let gpu_bias = readback_f32s(ctx, &state.qfrc_bias, nv * n_env);
    for d in 0..nv {
        check(
            fails,
            gpu_bias[off_v + d],
            data.qfrc_bias[d] as f32,
            TOL_DYNAMIC,
            format!("[{name}] qfrc_bias[{d}]"),
        );
    }

    // Vacuity guard (cf. T13b/T14): every fixture carries nonzero qvel, so the
    // GPU's velocity FK MUST produce nonzero `cvel` on the moving body. This
    // proves a green run actually compared live dynamics rather than
    // all-zeros-vs-all-zeros. (`qfrc_bias` is NOT a valid witness here:
    // `slide_single`'s axis is horizontal, so gravity projects to zero and a
    // lone slide DOF has no Coriolis — its qfrc_bias is legitimately zero.)
    assert!(
        gpu_cvel[off_b * 2..(off_b + nbody) * 2]
            .iter()
            .any(|v| v[0].abs() + v[1].abs() + v[2].abs() > 1e-4),
        "[{name}] env{env} cvel is all-zero on the GPU — velocity FK did not run; \
         the conformance check would be vacuous."
    );
}

/// The conformance sweep. Each non-tendon case in the shared matrix is run
/// through the GPU dynamics chain and asserted field-by-field against the CPU
/// `forward()`. A failure names the case + field + index so a new GPU↔CPU
/// divergence lands in the ledger with a precise locator.
#[test]
fn gpu_dynamics_matches_cpu_across_joint_matrix() {
    let ctx = gpu_or_skip!();

    let mut fails: Failures = Vec::new();

    for case in dynamics_conformance_matrix() {
        // No GPU tendon shader — the spatial-tendon case has no counterpart.
        if case.name == "spatial_tendon_spring" {
            continue;
        }
        let ConformanceCase {
            name,
            model,
            qpos,
            qvel,
        } = &case;

        let mut data = model.make_data();
        data.qpos.as_mut_slice().copy_from_slice(qpos);
        data.qvel.as_mut_slice().copy_from_slice(qvel);
        data.forward(model).expect("CPU forward");

        let before = fails.len();
        let state = run_through_rne(&ctx, model, &data);
        compare_env_fields(&mut fails, &ctx, &state, &data, model, name, 0, 1);

        // Separate this case's new failures so a known divergence neither gates
        // the suite nor silently hides a future GPU fix.
        let case_fails = fails.split_off(before);
        eprintln!(
            "  conformance[{name}] (nv={}, nbody={})",
            model.nv, model.nbody
        );
        finish_case(&mut fails, case_fails, name);
    }

    assert!(
        fails.is_empty(),
        "GPU↔CPU dynamics conformance found {} NEW divergence(s) (not on the known allowlist):\n  {}",
        fails.len(),
        fails.join("\n  ")
    );
}

/// Route a single case's new failures through the known-divergence allowlist.
///
/// - Allowlisted case: assert it STILL diverges (else the GPU was fixed →
///   remove the entry); the failures don't gate the suite.
/// - Otherwise: extend `fails` so any real divergence gates.
fn finish_case(fails: &mut Failures, case_fails: Vec<String>, name: &str) {
    match known_gpu_divergence(name) {
        Some(reason) => {
            assert!(
                !case_fails.is_empty(),
                "[{name}] is on the known-divergence allowlist ({reason}) but now MATCHES \
                 CPU — the GPU was fixed; remove its `known_gpu_divergence` entry."
            );
            eprintln!(
                "    EXPECTED-DIVERGENT ({} entries, {reason})",
                case_fails.len()
            );
        }
        None => {
            if case_fails.is_empty() {
                eprintln!("    passed");
            } else {
                eprintln!("    FAILED ({} divergences)", case_fails.len());
                fails.extend(case_fails);
            }
        }
    }
}

/// Compare GPU (readback `qpos`/`qvel`) vs CPU `Data` after a step. Handles any
/// joint mix: the only quaternion-bearing layout in these fixtures is a root
/// ball/free whose quaternion starts at `qpos[3]`, sign-aligned (`q ≡ −q`); for
/// pure hinge/slide chains the loop below is a plain element-wise compare.
fn check_step_state(
    fails: &mut Failures,
    name: &str,
    step: usize,
    gpu_qpos: &[f32],
    gpu_qvel: &[f32],
    cpu: &Data,
    nq: usize,
    nv: usize,
    tol: f32,
) {
    // qpos: a unit quaternion (root ball/free) needs sign-alignment; a scalar
    // (hinge/slide) does not. Detect by nq vs nv mismatch on the trailing block.
    let has_quat = nq == nv + 1; // single ball(+1) or free(+1) root → one extra qpos
    for i in 0..nq {
        let (g, c) = if has_quat && i >= nq - 4 {
            // quaternion block: sign-align on its w (first quat component).
            let qw_idx = nq - 4;
            let sign = if gpu_qpos[qw_idx] * cpu.qpos[qw_idx] as f32 >= 0.0 {
                1.0
            } else {
                -1.0
            };
            (gpu_qpos[i], sign * cpu.qpos[i] as f32)
        } else {
            (gpu_qpos[i], cpu.qpos[i] as f32)
        };
        check(fails, g, c, tol, format!("[{name}] step{step} qpos[{i}]"));
    }
    for i in 0..nv {
        check(
            fails,
            gpu_qvel[i],
            cpu.qvel[i] as f32,
            tol,
            format!("[{name}] step{step} qvel[{i}]"),
        );
    }
}

/// One contact-free GPU substep on a persistent state buffer: FK → CRBA →
/// velocity-FK → RNE → smooth → eulerdamp `(M+h·D)` solve → integrate. Mirrors
/// the T17/T18 hand-rolled trajectory loop (no collision/constraint stage, so it
/// works for every joint type — unlike the orchestrator, which is free-only and
/// needs geom buffers). The eulerdamp solve writes `qacc` (the contact-free
/// analog of the constraint stage); for an undamped model it reduces to
/// `M⁻¹·qfrc_smooth = qacc_smooth`, so it is safe for all cases. `qfrc_*`
/// accumulators are reset inside each pipeline's dispatch.
// (arg count covered by the module-level `#![allow(clippy::too_many_arguments)]`.)
fn gpu_substep(
    ctx: &GpuContext,
    model: &Model,
    model_buf: &GpuModelBuffers,
    state: &GpuStateBuffers,
    fk: &GpuFkPipeline,
    crba: &GpuCrbaPipeline,
    vel: &GpuVelocityFkPipeline,
    rne: &GpuRnePipeline,
    smooth: &GpuSmoothPipeline,
    eulerdamp: &GpuEulerdampPipeline,
    integrate: &GpuIntegratePipeline,
) {
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("damped substep"),
        });
    fk.dispatch(ctx, model_buf, state, &mut encoder);
    crba.dispatch(ctx, model_buf, state, &mut encoder);
    vel.dispatch(ctx, model_buf, state, &mut encoder);
    rne.dispatch(ctx, model_buf, state, model, &mut encoder);
    smooth.dispatch(ctx, model_buf, state, model, &mut encoder);
    // Implicit damped velocity solve → qacc = (M + h·D)⁻¹·(qfrc_smooth − D·q̇),
    // the contact-free analog of the constraint stage.
    eulerdamp.dispatch(ctx, model_buf, model, &mut encoder);
    integrate.dispatch(ctx, model_buf, state, model, &mut encoder);
    ctx.queue.submit([encoder.finish()]);
}

/// Slice-2 damped-integration conformance: step each damped fixture forward on
/// GPU (hand-rolled contact-free substep) and CPU (`Data::step`) in lockstep and
/// compare the `qpos`/`qvel` trajectory. The current GPU substep applies NO joint
/// damping (no passive `−D·q̇` force, no implicit `(M + h·D)` solve), so it
/// integrates the UNDAMPED trajectory — every damped case is EXPECTED-DIVERGENT
/// until Slice-2b, and the allowlist's self-validation forces the fix to flip
/// them green.
#[test]
fn gpu_damped_integration_matches_cpu() {
    const N_STEPS: usize = 10;

    let ctx = gpu_or_skip!();
    let checkpoints = [1usize, N_STEPS];
    let mut fails: Failures = Vec::new();

    for case in &damped_conformance_matrix() {
        let ConformanceCase {
            name,
            model,
            qpos,
            qvel,
        } = case;

        // Guard: the fixture must actually carry damping, else the test is a
        // tautology (undamped GPU == undamped CPU).
        assert!(
            (0..model.nv).any(|i| model.implicit_damping[i] > 0.0),
            "[{name}] fixture has no damping — vacuous eulerdamp test"
        );

        let mut data_cpu = model.make_data();
        data_cpu.qpos.as_mut_slice().copy_from_slice(qpos);
        data_cpu.qvel.as_mut_slice().copy_from_slice(qvel);

        // GPU state persists across substeps (integrate writes qpos/qvel back
        // into these buffers; the next substep's FK reads them).
        let model_buf = GpuModelBuffers::upload(&ctx, model);
        let state = GpuStateBuffers::new(&ctx, &model_buf, &data_cpu);
        let fk = GpuFkPipeline::new(&ctx, &model_buf, &state);
        let crba = GpuCrbaPipeline::new(&ctx, &model_buf, &state);
        let vel = GpuVelocityFkPipeline::new(&ctx, &model_buf, &state);
        let rne = GpuRnePipeline::new(&ctx, &model_buf, &state, model);
        let smooth = GpuSmoothPipeline::new(&ctx, &model_buf, &state);
        let eulerdamp = GpuEulerdampPipeline::new(&ctx, &model_buf, &state);
        let integrate = GpuIntegratePipeline::new(&ctx, &model_buf, &state);

        let before = fails.len();
        for step in 1..=N_STEPS {
            data_cpu.step(model).expect("CPU step");
            gpu_substep(
                &ctx, model, &model_buf, &state, &fk, &crba, &vel, &rne, &smooth, &eulerdamp,
                &integrate,
            );
            if checkpoints.contains(&step) {
                let gpu_qpos = readback_f32s(&ctx, &state.qpos, model.nq);
                let gpu_qvel = readback_f32s(&ctx, &state.qvel, model.nv);
                check_step_state(
                    &mut fails,
                    name,
                    step,
                    &gpu_qpos,
                    &gpu_qvel,
                    &data_cpu,
                    model.nq,
                    model.nv,
                    TOL_DYNAMIC,
                );
            }
        }
        let case_fails = fails.split_off(before);
        eprintln!("  damped[{name}] (nv={})", model.nv);
        finish_case(&mut fails, case_fails, name);
    }

    assert!(
        fails.is_empty(),
        "GPU↔CPU damped-integration conformance found {} NEW divergence(s):\n  {}",
        fails.len(),
        fails.join("\n  ")
    );
}

/// Build a state that is DISTINCT from `(qpos, qvel)` yet valid for every joint
/// type (quaternions stay normalized). The configuration is advanced along the
/// velocity via the CPU's manifold-aware position integrator
/// (`mj_integrate_pos_explicit`, the exp-map on SO(3) for ball/free) over a fixed
/// `dt` — independent of `model.timestep` (which is unset/zero in these
/// derivative-harness fixtures). The velocity is then set to a scaled copy of the
/// original, so env 1 differs from env 0 in BOTH `qpos` and `qvel`. Returns a
/// forward-evaluated `Data`.
fn perturbed_env(model: &Model, qpos: &[f64], qvel: &[f64]) -> Data {
    // Advance the configuration along qvel by a fixed, sizeable dt so the
    // perturbed pose clears the conformance tolerance for every fixture.
    let qpos0 = DVector::from_row_slice(qpos);
    let qvel0 = DVector::from_row_slice(qvel);
    let mut qpos1 = qpos0.clone();
    mj_integrate_pos_explicit(model, &mut qpos1, &qpos0, &qvel0, 0.3);

    let mut d = model.make_data();
    d.qpos.copy_from(&qpos1);
    // env 1's actual velocity: a distinct scaling of the original.
    for (dst, &v) in d.qvel.as_mut_slice().iter_mut().zip(qvel) {
        *dst = v * 1.3;
    }
    d.forward(model).expect("CPU forward (perturbed env)");
    d
}

/// Slice-2a batched (`n_env = 2`) forward conformance. For each non-tendon case
/// in the shared matrix, env 0 is the case state and env 1 is a DISTINCT
/// perturbed state ([`perturbed_env`]). The whole forward chain runs ONCE over
/// both environments on the batched buffers; each env is then read back from its
/// own stride and asserted against `forward(state_k)` via the Slice-1 oracle.
///
/// This is the verification that the forward path is env-correct after the
/// allocator + `fk.wgsl` env-offset fixes: if any stage dropped the env stride
/// (e.g. env 1 read env 0's `qpos`), env 1 would match `forward(state_0)` — which
/// differs from `forward(state_1)` — and the comparison would fail. A
/// per-case discriminator guard asserts the two CPU references actually differ,
/// so a green run can never be vacuous.
#[test]
fn gpu_batched_forward_matches_cpu() {
    let ctx = gpu_or_skip!();

    let mut fails: Failures = Vec::new();

    for case in dynamics_conformance_matrix() {
        // No GPU tendon shader — the spatial-tendon case has no counterpart.
        if case.name == "spatial_tendon_spring" {
            continue;
        }
        let ConformanceCase {
            name,
            model,
            qpos,
            qvel,
        } = &case;

        // env 0 = the case state; env 1 = a distinct, valid perturbation.
        let mut data0 = model.make_data();
        data0.qpos.as_mut_slice().copy_from_slice(qpos);
        data0.qvel.as_mut_slice().copy_from_slice(qvel);
        data0.forward(model).expect("CPU forward (env 0)");
        let data1 = perturbed_env(model, qpos, qvel);

        // Discriminator guard: the two envs must feed the GPU genuinely distinct
        // inputs, else a green batched run proves nothing (a stage that dropped
        // the env stride and read env 0's state would still "match"). qpos differs
        // by construction (the fk env-offset fix is exactly about per-env qpos);
        // qvel differs by the 1.3× scale. (A single hinge whose body sits at its
        // pivot moves only in xquat, not xpos — so we guard on qpos, not poses.)
        let qpos_differ = (0..model.nq).any(|i| (data0.qpos[i] - data1.qpos[i]).abs() > 1e-3);
        assert!(
            qpos_differ,
            "[{name}] env 0 and env 1 have near-identical qpos — the batched \
             conformance check would be vacuous (perturbation too small)."
        );

        let before = fails.len();
        let state = run_through_rne_batched(&ctx, model, &[&data0, &data1]);
        compare_env_fields(&mut fails, &ctx, &state, &data0, model, name, 0, 2);
        compare_env_fields(&mut fails, &ctx, &state, &data1, model, name, 1, 2);

        let case_fails = fails.split_off(before);
        eprintln!("  batched[{name}] (nv={}, nbody={})", model.nv, model.nbody);
        finish_case(&mut fails, case_fails, name);
    }

    assert!(
        fails.is_empty(),
        "GPU↔CPU batched forward conformance found {} NEW divergence(s):\n  {}",
        fails.len(),
        fails.join("\n  ")
    );
}
