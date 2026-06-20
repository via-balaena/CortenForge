// smooth.wgsl — Smooth force assembly + Cholesky solve for qacc_smooth.
//
// 2 entry points:
//   1. smooth_assemble  — per-DOF: qfrc_smooth = applied + actuator + passive - bias
//   2. smooth_solve     — single thread: forward/back substitution using qM_factor

// ── Packed struct types ─────────────────────────────────────────────

struct PhysicsParams {
    gravity: vec4<f32>,
    timestep: f32,
    nbody: u32,
    njnt: u32,
    nv: u32,
    nq: u32,
    n_env: u32,
    current_depth: u32,
    nu: u32,
};

// ── Bindings ─────────────────────────────────────────────────────────

// Group 0: physics params
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Per-DOF model entry (matches `DofModel` in types.rs / eulerdamp.wgsl) — read
// for the implicit joint-damping coefficient.
struct DofModel {
    body_id: u32,
    parent: u32,
    armature: f32,
    damping: f32,
};

// Group 1: force inputs (read-only)
@group(1) @binding(0) var<storage, read> qfrc_bias: array<f32>;
@group(1) @binding(1) var<storage, read> qfrc_applied: array<f32>;
@group(1) @binding(2) var<storage, read> qfrc_actuator: array<f32>;
@group(1) @binding(3) var<storage, read> qfrc_passive: array<f32>;
@group(1) @binding(4) var<storage, read> qvel: array<f32>;
@group(1) @binding(5) var<storage, read> dofs: array<DofModel>;

// Group 2: mass matrix factor (read-only)
@group(2) @binding(0) var<storage, read> qM_factor: array<f32>;

// Group 3: outputs (read-write)
@group(3) @binding(0) var<storage, read_write> qfrc_smooth: array<f32>;
@group(3) @binding(1) var<storage, read_write> qacc_smooth: array<f32>;

// ── Entry point 1: Assemble qfrc_smooth ─────────────────────────────

@compute @workgroup_size(64)
fn smooth_assemble(@builtin(global_invocation_id) gid: vec3<u32>) {
    let d = gid.x;
    let env_id = gid.y;
    if d >= params.nv || env_id >= params.n_env { return; }

    let env_off = env_id * params.nv;
    let idx = env_off + d;

    // Implicit joint-damping passive force −D·q̇ (MuJoCo qfrc_passive). Folded in
    // HERE — before the constraint solve — so the constraint solver's
    // unconstrained reference (qacc_smooth) includes the damper, matching the CPU
    // (whose qfrc_passive flows into qfrc_smooth). The eulerdamp stage then makes
    // the damping implicit via (M + h·D) on the LHS; it must NOT re-subtract the
    // damper. `damping = 0` for undamped DOFs ⇒ this term is a no-op (byte-identical).
    qfrc_smooth[idx] = qfrc_applied[idx] + qfrc_actuator[idx]
                      + qfrc_passive[idx] - qfrc_bias[idx]
                      - dofs[d].damping * qvel[idx];
}

// ── Entry point 2: Cholesky solve (forward + backward substitution) ─

@compute @workgroup_size(64)
fn smooth_solve(@builtin(global_invocation_id) gid: vec3<u32>) {
    let env_id = gid.y;
    if gid.x != 0u || env_id >= params.n_env { return; }

    let nv = params.nv;
    let env_off = env_id * nv;
    let env_qm_off = env_id * nv * nv;

    // Copy qfrc_smooth → qacc_smooth (will be overwritten by solve)
    for (var i = 0u; i < nv; i++) {
        qacc_smooth[env_off + i] = qfrc_smooth[env_off + i];
    }

    // Forward substitution: L · y = b
    for (var i = 0u; i < nv; i++) {
        var sum = qacc_smooth[env_off + i];
        for (var k = 0u; k < i; k++) {
            sum -= qM_factor[env_qm_off + i * nv + k] * qacc_smooth[env_off + k];
        }
        let diag = qM_factor[env_qm_off + i * nv + i];
        qacc_smooth[env_off + i] = sum / diag;
    }

    // Backward substitution: L^T · x = y
    // Loop from nv-1 down to 0. Since nv is u32, use checked decrement.
    if nv > 0u {
        var i = nv - 1u;
        loop {
            var sum = qacc_smooth[env_off + i];
            for (var k = i + 1u; k < nv; k++) {
                // L^T[i,k] = L[k,i] (lower triangular stored row-major)
                sum -= qM_factor[env_qm_off + k * nv + i] * qacc_smooth[env_off + k];
            }
            let diag = qM_factor[env_qm_off + i * nv + i];
            qacc_smooth[env_off + i] = sum / diag;

            if i == 0u { break; }
            i -= 1u;
        }
    }
}
