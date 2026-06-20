// eulerdamp.wgsl — implicit joint-damping velocity solve (MuJoCo mj_Euler).
//
// CPU reference (sim-core integrate::Data::integrate, Euler + eulerdamp):
//   solve (M + h·D)·qacc = qfrc_smooth + qfrc_constraint   then   qvel += h·qacc
// where qfrc_smooth already includes the explicit passive damper force −D·q̇.
// The GPU folds that damper into qfrc_smooth in `smooth_assemble` (so the
// constraint solver's reference includes it, matching the CPU), so this kernel's
// RHS is simply qfrc_smooth + qfrc_constraint — it must NOT re-subtract −D·q̇.
//
// The MuJoCo-Euler constraint solve itself uses the PURE M (only
// ImplicitSpringDamper folds damping into the constraint Hessian), so the (M +
// h·D) correction lives entirely here, after the solve. This kernel re-solves
// into a SEPARATE scratch factor, leaving crba's qM / qM_factor the pure M the
// constraint/contact solver needs, and OVERWRITES qacc — the orchestrator runs
// it between the constraint stage (which writes qfrc_constraint, = 0 with no
// contacts) and integration. With no contacts the result is the contact-free
// damped solve.
//
// Single entry point, single thread per env (mirrors crba_cholesky +
// smooth_solve): build (M + h·D) → Cholesky → forward/back substitution.

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

struct DofModel {
    body_id: u32,
    parent: u32,
    armature: f32,
    damping: f32,
};

// Group 0: physics params
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: per-DOF model (read) — for `damping`
@group(1) @binding(0) var<storage, read> dofs: array<DofModel>;

// Group 2: inputs (read)
@group(2) @binding(0) var<storage, read> qM: array<f32>;
@group(2) @binding(1) var<storage, read> qfrc_smooth: array<f32>;
@group(2) @binding(2) var<storage, read> qfrc_constraint: array<f32>;

// Group 3: outputs (read-write)
@group(3) @binding(0) var<storage, read_write> qM_damped: array<f32>; // scratch L
@group(3) @binding(1) var<storage, read_write> qacc: array<f32>;

@compute @workgroup_size(64)
fn eulerdamp_solve(@builtin(global_invocation_id) gid: vec3<u32>) {
    let env_id = gid.y;
    if gid.x != 0u || env_id >= params.n_env { return; }

    let nv = params.nv;
    if nv == 0u { return; }
    let h = params.timestep;
    let env_off = env_id * nv;
    let env_qm_off = env_id * nv * nv;

    // Build (M + h·D) into the scratch factor buffer, and the total RHS
    // qfrc_smooth + qfrc_constraint into qacc (the solve overwrites it in place).
    // qfrc_smooth already carries the −D·q̇ damper (folded in by smooth_assemble);
    // qfrc_constraint is the constraint stage's per-DOF output, zero with no contacts.
    for (var i = 0u; i < nv; i++) {
        for (var j = 0u; j < nv; j++) {
            qM_damped[env_qm_off + i * nv + j] = qM[env_qm_off + i * nv + j];
        }
        qM_damped[env_qm_off + i * nv + i] += h * dofs[i].damping;
        qacc[env_off + i] = qfrc_smooth[env_off + i] + qfrc_constraint[env_off + i];
    }

    // Dense Cholesky factorization (lower triangular L), mirrors crba_cholesky.
    for (var j = 0u; j < nv; j++) {
        var sum = qM_damped[env_qm_off + j * nv + j];
        for (var k = 0u; k < j; k++) {
            let ljk = qM_damped[env_qm_off + j * nv + k];
            sum -= ljk * ljk;
        }
        qM_damped[env_qm_off + j * nv + j] = sqrt(max(sum, 1e-10));
        let ljj = qM_damped[env_qm_off + j * nv + j];

        for (var i = j + 1u; i < nv; i++) {
            var s = qM_damped[env_qm_off + i * nv + j];
            for (var k = 0u; k < j; k++) {
                s -= qM_damped[env_qm_off + i * nv + k]
                   * qM_damped[env_qm_off + j * nv + k];
            }
            qM_damped[env_qm_off + i * nv + j] = s / ljj;
            qM_damped[env_qm_off + j * nv + i] = 0.0;
        }
    }

    // Forward substitution: L · y = b   (qacc holds b, overwritten with y).
    for (var i = 0u; i < nv; i++) {
        var sum = qacc[env_off + i];
        for (var k = 0u; k < i; k++) {
            sum -= qM_damped[env_qm_off + i * nv + k] * qacc[env_off + k];
        }
        qacc[env_off + i] = sum / qM_damped[env_qm_off + i * nv + i];
    }

    // Backward substitution: Lᵀ · x = y.
    var i = nv - 1u;
    loop {
        var sum = qacc[env_off + i];
        for (var k = i + 1u; k < nv; k++) {
            // Lᵀ[i,k] = L[k,i] (lower triangular stored row-major)
            sum -= qM_damped[env_qm_off + k * nv + i] * qacc[env_off + k];
        }
        qacc[env_off + i] = sum / qM_damped[env_qm_off + i * nv + i];

        if i == 0u { break; }
        i -= 1u;
    }
}
