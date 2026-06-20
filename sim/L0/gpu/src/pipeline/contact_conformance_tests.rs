//! GPU↔CPU contact/constraint conformance harness (n_env = 1, injected contacts).
//!
//! The contact-free suite ([`super::conformance_tests`]) covers the smooth /
//! damped path. This suite tackles the constraint stage — assembly
//! (`assemble.wgsl`) and the Newton solve (`newton_solve.wgsl`) — which the
//! 2026-06-18 divergence audit flagged as the biggest GPU↔CPU divergence cluster
//! (pyramidal R-scaling, fixed 4-point line-search, no convergence test). See
//! `project-gpu-shader-conformance-gap`.
//!
//! # Why injection, not collision
//!
//! GPU collision represents geoms as SDF grids and emits one contact per
//! penetrating cell; CPU collision uses analytic geoms and emits a single
//! contact. The two engines therefore never agree on the contact SET for the
//! same geometry — a collision-layer divergence that would swamp the
//! assembly/solver divergences under test here. So both engines are fed an
//! IDENTICAL, hand-built contact set and collision is skipped: the GPU
//! [`PipelineContact`] array is written straight into `contact_buffer`
//! (`COPY_DST`), and the CPU oracle injects the same set via
//! [`sim_core::test_fixtures::conformance::contact_constraint_oracle`]. The
//! shared [`ContactSpec`] source means neither side can drift. (Collision
//! conformance — SDF dedup, per-cell caps — is a separate later slice.)
//!
//! # Comparison channels
//!
//! Per case, two channels with their own tolerances and allowlist entries so a
//! fix can flip exactly one green:
//! - **assembly** (`efc_d`, `efc_aref`): per-row, compared as a sorted multiset
//!   (a single contact's pyramidal facet rows must align, but no fixed facet
//!   order is assumed). This is where the R-scaling / aref divergences live.
//! - **solve** (`qacc`, `qfrc_constraint`): nv-vectors (row-order-independent),
//!   looser tolerance (f32 GPU Cholesky vs f64 CPU). This is where the Newton
//!   solver divergences live.
//!
//! Scope: n_env = 1. The whole contact-constraint path now CONFORMS across
//! condim=1/3/4 (assembly R-scaling #358, Newton-solver stride #359, and the
//! oracle pinned to `SolverType::Newton` to match the GPU's solver). The
//! self-validating `known_contact_divergence` allowlist is currently empty; a
//! new divergence re-arms a single (case, channel) entry without masking.

#![cfg(test)]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::needless_pass_by_value,
    // `!(diff < tol)` is intentional: it treats NaN as a failure (unlike `>=`).
    clippy::neg_cmp_op_on_partial_ord
)]

#[cfg(test)]
const _: () = ();

use sim_core::test_fixtures::conformance::{
    ConstraintOracle, ContactSpec, contact_conformance_matrix, contact_constraint_oracle,
};
use sim_core::types::{Data, Model};

use super::constraint::GpuConstraintPipeline;
use super::crba::GpuCrbaPipeline;
use super::fk::{GpuFkPipeline, readback_f32s};
use super::model_buffers::GpuModelBuffers;
use super::rne::GpuRnePipeline;
use super::smooth::GpuSmoothPipeline;
use super::state_buffers::GpuStateBuffers;
use super::types::PipelineContact;
use super::velocity_fk::GpuVelocityFkPipeline;
use crate::context::GpuContext;

/// Try to create a GPU context; skip the test (return) if none is available.
macro_rules! gpu_or_skip {
    () => {
        match GpuContext::new() {
            Ok(ctx) => ctx,
            Err(e) => {
                eprintln!("  Skipping contact conformance suite (no GPU): {e}");
                return;
            }
        }
    };
}

/// Assembly-channel absolute floor (`efc_d`, `efc_aref`): tight, f32-vs-f64 only.
const TOL_ASSEMBLY: f32 = 1e-3;
/// Assembly-channel relative term, for headroom on larger `efc_aref` magnitudes.
const TOL_ASSEMBLY_REL: f32 = 1e-3;
/// Solve-channel relative tolerance (`qacc`, `qfrc_constraint`): looser to
/// absorb the f32 GPU Cholesky / iterative solve vs the f64 CPU solver.
const TOL_SOLVE_REL: f32 = 1e-2;
/// Solve-channel absolute floor (so near-zero DOFs don't trip the relative test).
const TOL_SOLVE_ABS: f32 = 1e-3;

type Failures = Vec<String>;

/// Allowlist of (case, channel) pairs with a KNOWN GPU↔CPU divergence, returning
/// the reason. **Currently empty — the whole contact-constraint path conforms.**
///
/// Self-validating, exactly like the contact-free suite's `known_gpu_divergence`:
/// an allowlisted (case, channel) that suddenly MATCHES fails the test, so the
/// allowlist can never silently mask a future fix. Re-arm one (case, channel) arm
/// here when a NEW divergence is exposed.
fn known_contact_divergence(case: &str, channel: &str) -> Option<&'static str> {
    // No (case, channel) pairs diverge any more — the whole contact-constraint
    // path (assembly + solve) conforms across condim=1/3/4:
    //
    // - **assembly** (efc_d/efc_aref/n_rows): the pyramidal facet R-override
    //   (Rpy = 2·mu_reg²·R, mu_reg = mu·√(1/impratio), mu = mu[0] for ALL facets
    //   including torsional) is ported (PR-2), so every row matches.
    // - **solve** (qacc/qfrc_constraint): PR-3 fixed the PHASE-1 Hessian stride bug.
    //   The last `contact_torsional` DOF5 (spin-axis) ~8.6% residual was NOT a GPU
    //   bug — the GPU's Newton/Gauss-Newton solver reaches the exact constraint-QP
    //   optimum, but the CPU oracle had used the first-order `SolverType::PGS`,
    //   which had only crawled the weakly-coupled spin DOF partway at the default
    //   iteration count (PGS@100 → 7.72, true optimum 8.385). The fixture now pins
    //   the oracle to `SolverType::Newton` (the GPU's algorithm), so it converges in
    //   ~1 iteration to the value the GPU already computed.
    //
    // Re-add a `match (case, channel)` arm here when a NEW divergence is exposed;
    // the harness self-validates against it (an allowlisted pair that MATCHES fails).
    let _ = (case, channel);
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

/// Relative-or-absolute check for solver outputs (forces span ~`m·g`).
fn check_rel(fails: &mut Failures, gpu: f32, cpu: f32, what: String) {
    let tol = TOL_SOLVE_ABS + TOL_SOLVE_REL * gpu.abs().max(cpu.abs());
    check(fails, gpu, cpu, tol, what);
}

/// Sorted-multiset compare of a per-row assembly vector (row-order-independent).
fn check_multiset(fails: &mut Failures, gpu: &[f32], cpu: &[f64], case: &str, field: &str) {
    if gpu.len() != cpu.len() {
        fails.push(format!(
            "[{case}] {field}: row count GPU={} CPU={}",
            gpu.len(),
            cpu.len()
        ));
        return;
    }
    let mut g: Vec<f32> = gpu.to_vec();
    let mut c: Vec<f32> = cpu.iter().map(|&v| v as f32).collect();
    // `total_cmp` (not `partial_cmp().unwrap()`): a GPU NaN — itself a real
    // divergence an expose harness must catch — would panic an unwrap sort;
    // here it sorts to an extreme and is reported cleanly by `check` (NaN-safe).
    g.sort_by(f32::total_cmp);
    c.sort_by(f32::total_cmp);
    for (i, (&gi, &ci)) in g.iter().zip(c.iter()).enumerate() {
        // Relative-or-absolute: the R-scaling divergence is a ~56% ratio (caught
        // at any sane rel tol), while a conformant `efc_aref` reaches magnitude
        // ~100 where a pure-absolute floor would have thin f32 headroom.
        let tol = TOL_ASSEMBLY + TOL_ASSEMBLY_REL * gi.abs().max(ci.abs());
        check(fails, gi, ci, tol, format!("[{case}] {field}[sorted {i}]"));
    }
}

/// Read the GPU constraint row counter (`constraint_count`, an atomic `u32`).
fn readback_count(ctx: &GpuContext, state: &GpuStateBuffers) -> u32 {
    let raw = readback_f32s(ctx, &state.constraint_count, 1);
    f32::to_bits(raw[0])
}

/// Build the GPU `PipelineContact` array from the CPU oracle's generated
/// contacts, so both engines solve the identical contact set.
fn gpu_contacts(contacts: &[ContactSpec]) -> Vec<PipelineContact> {
    contacts
        .iter()
        .map(|c| PipelineContact {
            point: [c.pos[0] as f32, c.pos[1] as f32, c.pos[2] as f32],
            depth: c.depth as f32,
            normal: [c.normal[0] as f32, c.normal[1] as f32, c.normal[2] as f32],
            geom1: c.geom1 as u32,
            // Friction [slide, torsion, roll]; assemble.wgsl uses the slide term
            // for sliding facets and the torsion term for condim=4 torsional facets.
            friction: [c.mu as f32, c.mu_torsion as f32, 0.0],
            geom2: c.geom2 as u32,
        })
        .collect()
}

/// Run FK → CRBA → velocity-FK → RNE → smooth → constraint on the GPU over the
/// injected contact set (no collision pipeline), returning the state buffers.
fn run_constraint_stage(
    ctx: &GpuContext,
    model: &Model,
    data: &Data,
    contacts: &[PipelineContact],
) -> GpuStateBuffers {
    let model_buf = GpuModelBuffers::upload(ctx, model);
    let state_buf = GpuStateBuffers::new(ctx, &model_buf, data);

    // Inject contacts straight into the buffers (collision is skipped).
    ctx.queue
        .write_buffer(&state_buf.contact_buffer, 0, bytemuck::cast_slice(contacts));
    let count = contacts.len() as u32;
    ctx.queue
        .write_buffer(&state_buf.contact_count, 0, bytemuck::bytes_of(&count));

    let fk = GpuFkPipeline::new(ctx, &model_buf, &state_buf);
    let crba = GpuCrbaPipeline::new(ctx, &model_buf, &state_buf);
    let vel = GpuVelocityFkPipeline::new(ctx, &model_buf, &state_buf);
    let rne = GpuRnePipeline::new(ctx, &model_buf, &state_buf, model);
    let smooth = GpuSmoothPipeline::new(ctx, &model_buf, &state_buf);
    let constraint = GpuConstraintPipeline::new(ctx, &model_buf, &state_buf, model);

    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("contact conformance fk..constraint"),
        });
    fk.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    crba.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    vel.dispatch(ctx, &model_buf, &state_buf, &mut encoder);
    rne.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    smooth.dispatch(ctx, &model_buf, &state_buf, model, &mut encoder);
    constraint.encode(&mut encoder, &state_buf);
    ctx.queue.submit([encoder.finish()]);
    state_buf
}

/// Route one (case, channel)'s failures through the known-divergence allowlist.
fn finish_channel(fails: &mut Failures, channel_fails: Vec<String>, case: &str, channel: &str) {
    match known_contact_divergence(case, channel) {
        Some(reason) => {
            assert!(
                !channel_fails.is_empty(),
                "[{case}] channel `{channel}` is on the known-divergence allowlist \
                 ({reason}) but now MATCHES CPU — the GPU was fixed; remove its \
                 `known_contact_divergence` entry."
            );
            eprintln!(
                "    {channel}: EXPECTED-DIVERGENT ({} entries, {reason})",
                channel_fails.len()
            );
        }
        None => {
            if channel_fails.is_empty() {
                eprintln!("    {channel}: passed");
            } else {
                eprintln!(
                    "    {channel}: FAILED ({} divergences)",
                    channel_fails.len()
                );
                fails.extend(channel_fails);
            }
        }
    }
}

/// Contact/constraint conformance sweep. For each injected-contact fixture, run
/// the GPU constraint stage and compare assembly + solve outputs against the CPU
/// oracle, routing each channel through the self-validating allowlist.
#[test]
fn gpu_constraint_matches_cpu_on_injected_contacts() {
    let ctx = gpu_or_skip!();
    let mut fails: Failures = Vec::new();

    for case in contact_conformance_matrix() {
        let oracle: ConstraintOracle = contact_constraint_oracle(&case);
        let model = &case.model;
        let nv = model.nv;

        let mut data = model.make_data();
        data.qpos.as_mut_slice().copy_from_slice(&case.qpos);
        data.qvel.as_mut_slice().copy_from_slice(&case.qvel);
        data.forward(model).expect("CPU forward (state setup)");

        let contacts = gpu_contacts(&oracle.contacts);
        let state = run_constraint_stage(&ctx, model, &data, &contacts);

        eprintln!("  contact[{}] (nv={nv}, rows={})", case.name, oracle.n_rows);

        // ── row count ──────────────────────────────────────────────────
        let gpu_rows = readback_count(&ctx, &state) as usize;
        let mut row_fails = Failures::new();
        check(
            &mut row_fails,
            gpu_rows as f32,
            oracle.n_rows as f32,
            0.5,
            format!("[{}] constraint row count", case.name),
        );
        finish_channel(&mut fails, row_fails, case.name, "n_rows");

        // ── assembly channel: efc_d, efc_aref (sorted multiset) ──────────
        let mut asm_fails = Failures::new();
        if gpu_rows == oracle.n_rows {
            let efc_d = readback_f32s(&ctx, &state.efc_d, gpu_rows);
            let efc_aref = readback_f32s(&ctx, &state.efc_aref, gpu_rows);
            check_multiset(&mut asm_fails, &efc_d, &oracle.efc_d, case.name, "efc_d");
            check_multiset(
                &mut asm_fails,
                &efc_aref,
                &oracle.efc_aref,
                case.name,
                "efc_aref",
            );
        }
        // Split efc_d vs efc_aref so each maps to its own allowlist entry.
        let (d_fails, aref_fails): (Vec<_>, Vec<_>) =
            asm_fails.into_iter().partition(|f| f.contains("efc_d"));
        finish_channel(&mut fails, d_fails, case.name, "efc_d");
        finish_channel(&mut fails, aref_fails, case.name, "efc_aref");

        // ── solve channel: qacc, qfrc_constraint (nv-vectors) ────────────
        let gpu_qacc = readback_f32s(&ctx, &state.qacc, nv);
        let gpu_qfrc = readback_f32s(&ctx, &state.qfrc_constraint, nv);

        // Vacuity guard on the REFERENCE: the CPU oracle must carry a nonzero
        // constraint response, else the solve comparison is all-zeros-vs-zeros.
        // A zero GPU response against a nonzero CPU one is a real divergence and
        // is recorded by `check_rel` below — NOT a vacuous run, so it must not
        // panic here.
        assert!(
            oracle.qfrc_constraint.iter().any(|v| v.abs() > 1e-6),
            "[{}] CPU oracle qfrc_constraint is all-zero — the fixture is vacuous.",
            case.name
        );

        let mut qacc_fails = Failures::new();
        for d in 0..nv {
            check_rel(
                &mut qacc_fails,
                gpu_qacc[d],
                oracle.qacc[d] as f32,
                format!("[{}] qacc[{d}]", case.name),
            );
        }
        finish_channel(&mut fails, qacc_fails, case.name, "qacc");

        let mut qfrc_fails = Failures::new();
        for d in 0..nv {
            check_rel(
                &mut qfrc_fails,
                gpu_qfrc[d],
                oracle.qfrc_constraint[d] as f32,
                format!("[{}] qfrc_constraint[{d}]", case.name),
            );
        }
        finish_channel(&mut fails, qfrc_fails, case.name, "qfrc_constraint");
    }

    assert!(
        fails.is_empty(),
        "GPU↔CPU contact conformance found {} NEW divergence(s) (not on the known \
         allowlist):\n  {}",
        fails.len(),
        fails.join("\n  ")
    );
}
