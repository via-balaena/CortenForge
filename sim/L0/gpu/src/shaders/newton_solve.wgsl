// newton_solve.wgsl — Primal Newton constraint solver.
//
// Single entry point dispatched as one workgroup of 256 threads:
//   newton_solve — iterative Newton solve for constrained acceleration
//
// Implements MuJoCo-style primal Newton method:
//   1. Initialize qacc = qacc_smooth (unconstrained acceleration)
//   2. For each Newton iteration:
//      a. Build Gauss-Newton Hessian: H = M + J_active^T D J_active
//      b. Cholesky factorize H (thread 0)
//      c. Compute gradient: grad = M*qacc - qfrc_smooth - J^T*force
//      d. Solve for search direction: search = -H^{-1} grad
//      e. Backtracking line search over {1.0, 0.5, 0.25}
//      f. Update qacc += alpha * search
//   3. Write final qacc and efc_force to global memory
//
// Active set: a constraint i is active iff J[i]*qacc - aref[i] < 0
// (violated inequality or equality constraint).

// ── Constants ───────────────────────────────────────────────────────────

const MAX_NV: u32 = 60u;
const WG_SIZE: u32 = 256u;

// ── Params struct ───────────────────────────────────────────────────────

// Fixed-config solver: hardcoded line search + max_iter bound + no-progress
// convergence. Mirrors the Rust `SolverParams` (16 bytes) — CPU solver-config
// knobs (tolerance / ls_iterations) are intentionally not plumbed.
struct SolverParams {
    nv:              u32,
    max_iter:        u32,
    n_env:           u32,
    max_constraints: u32,
};

// ── Bindings ────────────────────────────────────────────────────────────

// Group 0: solver parameters
@group(0) @binding(0) var<uniform> params: SolverParams;

// Group 1: mass matrix and smooth dynamics (read-only)
@group(1) @binding(0) var<storage, read> qM_buf:          array<f32>;
@group(1) @binding(1) var<storage, read> qacc_smooth_buf: array<f32>;
@group(1) @binding(2) var<storage, read> qfrc_smooth_buf: array<f32>;

// Group 2: constraint data (read-only)
@group(2) @binding(0) var<storage, read> efc_J_buf:    array<f32>;
@group(2) @binding(1) var<storage, read> efc_D_buf:    array<f32>;
@group(2) @binding(2) var<storage, read> efc_aref_buf: array<f32>;
@group(2) @binding(3) var<storage, read_write> constraint_count_buf: array<atomic<u32>>;

// Group 3: outputs (read-write)
@group(3) @binding(0) var<storage, read_write> qacc_buf:      array<f32>;
@group(3) @binding(1) var<storage, read_write> efc_force_buf: array<f32>;

// ── Shared memory ───────────────────────────────────────────────────────
//
// Total budget: 16,384 bytes
//   H_atomic:     MAX_NV * MAX_NV * 4 = 14,400 bytes
//   qacc_sh:      MAX_NV * 4           =    240 bytes
//   qacc_sm_sh:   MAX_NV * 4           =    240 bytes
//   grad_sh:      MAX_NV * 4           =    240 bytes
//   search_sh:    MAX_NV * 4           =    240 bytes
//   reduction_sh: WG_SIZE * 4          =  1,024 bytes
//                                       -------
//                                        16,384 bytes

var<workgroup> H_atomic:     array<atomic<u32>, 3600>;  // MAX_NV * MAX_NV
var<workgroup> qacc_sh:      array<f32, 60>;
var<workgroup> qacc_sm_sh:   array<f32, 60>;
var<workgroup> grad_sh:      array<f32, 60>;
var<workgroup> search_sh:    array<f32, 60>;
var<workgroup> reduction_sh: array<f32, 256>;

// Convergence flag and line-search result (must be module-scope workgroup vars)
var<workgroup> done:       u32;
var<workgroup> best_alpha: f32;

// ── CAS atomic helpers for H ────────────────────────────────────────────
// Same CAS pattern used in crba.wgsl for atomic f32 operations on
// workgroup memory (WGSL only provides atomic ops on u32/i32).

fn H_f32(r: u32, c: u32) -> f32 {
    return bitcast<f32>(atomicLoad(&H_atomic[r * MAX_NV + c]));
}

fn set_H_f32(r: u32, c: u32, val: f32) {
    atomicStore(&H_atomic[r * MAX_NV + c], bitcast<u32>(val));
}

fn atomic_add_H(idx: u32, val: f32) {
    var old = atomicLoad(&H_atomic[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let res = atomicCompareExchangeWeak(&H_atomic[idx], old, new_val);
        if res.exchanged { break; }
        old = res.old_value;
    }
}

// ── Cost evaluation ─────────────────────────────────────────────────────
// Evaluates the total cost (Gauss + constraint) at qacc + alpha * search.
// All 256 threads MUST participate — the function uses workgroupBarrier()
// for the parallel reduction. Only thread 0's return value is meaningful.

fn eval_cost(alpha: f32, nv: u32, nefc: u32, tid: u32, env: u32) -> f32 {
    var partial = 0.0;

    // Per-env base offsets into the global buffers (workgroup scratch is private).
    let env_efc = env * params.max_constraints;
    let env_efcj = env_efc * nv;

    // Constraint cost: sum over active (violated) constraints of 0.5 * D * jar^2
    for (var i = tid; i < nefc; i += WG_SIZE) {
        var jar_i = -efc_aref_buf[env_efc + i];
        let row_off = env_efcj + i * nv;
        for (var k = 0u; k < nv; k++) {
            jar_i += efc_J_buf[row_off + k] * (qacc_sh[k] + alpha * search_sh[k]);
        }
        if jar_i < 0.0 {
            partial += 0.5 * efc_D_buf[env_efc + i] * jar_i * jar_i;
        }
    }

    // Gauss cost: 0.5 * u^T * (M*qacc_trial - qfrc_smooth)
    // where u = qacc_trial - qacc_smooth. First nv threads each handle one DOF.
    if tid < nv {
        let env_qm = env * nv * nv;
        var ma_k = 0.0;
        for (var j = 0u; j < nv; j++) {
            ma_k += qM_buf[env_qm + tid * nv + j] * (qacc_sh[j] + alpha * search_sh[j]);
        }
        let u_k = (qacc_sh[tid] + alpha * search_sh[tid]) - qacc_sm_sh[tid];
        partial += 0.5 * u_k * (ma_k - qfrc_smooth_buf[env * nv + tid]);
    }

    // Parallel tree reduction across all 256 threads
    reduction_sh[tid] = partial;
    workgroupBarrier();
    for (var stride = 128u; stride > 0u; stride >>= 1u) {
        if tid < stride {
            reduction_sh[tid] += reduction_sh[tid + stride];
        }
        workgroupBarrier();
    }
    return reduction_sh[0];
}

// ── Main entry point ────────────────────────────────────────────────────

@compute @workgroup_size(256)
fn newton_solve(
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(workgroup_id) wid: vec3<u32>,
) {
    let tid = lid.x;
    // One workgroup per env (env = workgroup_id.x; dispatched (n_env,1,1)). The
    // guard is uniform across the workgroup (every thread shares wid.x), so the
    // early return never splits a workgroupBarrier.
    let env = wid.x;
    if env >= params.n_env {
        return;
    }
    let nv = params.nv;
    // Clamp to the per-env capacity: if assembly over-allocated (more contacts
    // than max_constraints), the counter is inflated past this env's row block, so
    // an unclamped loop would stride into the NEXT env's rows. Clamping contains an
    // over-budget step to its own block (excess rows dropped). A no-op normally.
    let nefc = min(atomicLoad(&constraint_count_buf[env]), params.max_constraints);

    // Per-env base offsets into the global state buffers. The workgroup-local
    // scratch (H_atomic / qacc_sh / grad_sh / search_sh / reduction_sh) is private
    // to this workgroup, so one-workgroup-per-env needs no scratch offset.
    let env_nv = env * nv;
    let env_qm = env * nv * nv;
    let env_efc = env * params.max_constraints;
    let env_efcj = env_efc * nv;

    // ── INITIALIZE: cold-start qacc = qacc_smooth ───────────────────────
    if tid < nv {
        qacc_sh[tid] = qacc_smooth_buf[env_nv + tid];
        qacc_sm_sh[tid] = qacc_smooth_buf[env_nv + tid];
    }
    workgroupBarrier();

    // Early exit: no active constraints means unconstrained solution is optimal
    if nefc == 0u {
        if tid < nv {
            qacc_buf[env_nv + tid] = qacc_sh[tid];
        }
        return;
    }

    // Initialize convergence flag
    if tid == 0u {
        done = 0u;
    }
    workgroupBarrier();

    // ── NEWTON ITERATION LOOP ───────────────────────────────────────────
    for (var iter = 0u; iter < params.max_iter; iter++) {
        if done != 0u { break; }

        // ── PHASE 1: Initialize Hessian H = M ──────────────────────────
        // All 256 threads cooperate to copy the mass matrix into shared H.
        // qM_buf is row-major with stride `nv`; H_atomic is strided by MAX_NV
        // (matching every other H access via `r * MAX_NV + c`). Translate the
        // flat copy index into (r, c) so the two layouts agree — copying flat
        // (stride-nv into stride-MAX_NV storage) corrupts H for nv < MAX_NV.
        for (var idx = tid; idx < nv * nv; idx += WG_SIZE) {
            let r = idx / nv;
            let c = idx % nv;
            atomicStore(&H_atomic[r * MAX_NV + c], bitcast<u32>(qM_buf[env_qm + idx]));
        }
        workgroupBarrier();

        // ── PHASE 2: Classify constraints, accumulate H and J^T*force ──
        // Each thread processes constraint rows: tid, tid+256, tid+512, ...
        // Active constraints (jar < 0) contribute D*J^T*J to H and
        // -D*jar*J^T to the force used in the gradient.

        // Per-thread partial J^T * force accumulator (register-resident)
        var partial_jtf: array<f32, 60>;
        for (var k = 0u; k < nv; k++) {
            partial_jtf[k] = 0.0;
        }

        for (var i = tid; i < nefc; i += WG_SIZE) {
            // Compute jar[i] = J[i] * qacc - aref[i]
            var jar_i = 0.0;
            let row_off = env_efcj + i * nv;
            for (var k = 0u; k < nv; k++) {
                jar_i += efc_J_buf[row_off + k] * qacc_sh[k];
            }
            jar_i -= efc_aref_buf[env_efc + i];

            // Satisfied constraint (jar >= 0) — skip
            if jar_i >= 0.0 { continue; }

            // Active constraint: accumulate H += D_i * J_i^T * J_i
            let d_i = efc_D_buf[env_efc + i];
            for (var r = 0u; r < nv; r++) {
                let j_r = efc_J_buf[row_off + r];
                if j_r == 0.0 { continue; }
                let d_j_r = d_i * j_r;
                for (var c = r; c < nv; c++) {
                    let j_c = efc_J_buf[row_off + c];
                    if j_c == 0.0 { continue; }
                    let val = d_j_r * j_c;
                    atomic_add_H(r * MAX_NV + c, val);
                    if r != c {
                        atomic_add_H(c * MAX_NV + r, val);
                    }
                }
            }

            // Accumulate J^T * force where force_i = -D_i * jar_i (positive for active)
            let force_i = -d_i * jar_i;
            for (var k = 0u; k < nv; k++) {
                partial_jtf[k] += force_i * efc_J_buf[row_off + k];
            }
        }
        workgroupBarrier();

        // ── PHASE 3: Cholesky factorize H in-place (thread 0) ──────────
        // Overwrites lower triangle of H with L such that H = L * L^T.
        if tid == 0u {
            for (var j = 0u; j < nv; j++) {
                var s = H_f32(j, j);
                for (var k = 0u; k < j; k++) {
                    let ljk = H_f32(j, k);
                    s -= ljk * ljk;
                }
                set_H_f32(j, j, sqrt(max(s, 1e-10)));
                let ljj = H_f32(j, j);

                for (var ii = j + 1u; ii < nv; ii++) {
                    var ss = H_f32(ii, j);
                    for (var k = 0u; k < j; k++) {
                        ss -= H_f32(ii, k) * H_f32(j, k);
                    }
                    set_H_f32(ii, j, ss / ljj);
                }
            }
        }
        workgroupBarrier();

        // ── PHASE 4: Compute gradient ───────────────────────────────────
        // grad = M * qacc - qfrc_smooth - J^T * efc_force
        //
        // First reduce partial_jtf across all 256 threads for each DOF k,
        // then thread 0 computes M*qacc and assembles the full gradient.
        for (var k = 0u; k < nv; k++) {
            reduction_sh[tid] = partial_jtf[k];
            workgroupBarrier();

            // Tree reduction over 256 threads
            for (var stride = 128u; stride > 0u; stride >>= 1u) {
                if tid < stride {
                    reduction_sh[tid] += reduction_sh[tid + stride];
                }
                workgroupBarrier();
            }

            if tid == 0u {
                // M * qacc for DOF k
                var ma_k = 0.0;
                for (var j = 0u; j < nv; j++) {
                    ma_k += qM_buf[env_qm + k * nv + j] * qacc_sh[j];
                }
                grad_sh[k] = ma_k - qfrc_smooth_buf[env_nv + k] - reduction_sh[0];
            }
            workgroupBarrier();
        }

        // ── PHASE 5: Search direction via Cholesky solve (thread 0) ────
        // Solve H * x = grad, then search = -x.
        // Uses the L factor stored in lower triangle of H_atomic.
        if tid == 0u {
            // Copy gradient into search buffer
            for (var i = 0u; i < nv; i++) {
                search_sh[i] = grad_sh[i];
            }

            // Forward substitution: L * y = grad
            for (var i = 0u; i < nv; i++) {
                for (var k = 0u; k < i; k++) {
                    search_sh[i] -= H_f32(i, k) * search_sh[k];
                }
                search_sh[i] /= H_f32(i, i);
            }

            // Backward substitution: L^T * x = y
            // (u32 cannot go negative, so use break-at-zero pattern)
            if nv > 0u {
                var i = nv - 1u;
                loop {
                    for (var k = i + 1u; k < nv; k++) {
                        search_sh[i] -= H_f32(k, i) * search_sh[k];
                    }
                    search_sh[i] /= H_f32(i, i);
                    if i == 0u { break; }
                    i -= 1u;
                }
            }

            // Negate: search = -H^{-1} * grad (descent direction)
            for (var i = 0u; i < nv; i++) {
                search_sh[i] = -search_sh[i];
            }
        }
        workgroupBarrier();

        // ── PHASE 6: Backtracking line search ───────────────────────────
        // Evaluate total cost at alpha = {0.0, 1.0, 0.5, 0.25}.
        // All 256 threads must participate in each eval_cost call
        // because it contains workgroupBarrier() in its reduction.
        let cost_0   = eval_cost(0.0,  nv, nefc, tid, env);
        let cost_1   = eval_cost(1.0,  nv, nefc, tid, env);
        let cost_05  = eval_cost(0.5,  nv, nefc, tid, env);
        let cost_025 = eval_cost(0.25, nv, nefc, tid, env);

        // Thread 0 picks the alpha that minimizes total cost
        if tid == 0u {
            var best_cost = cost_0;
            best_alpha = 0.0;
            if cost_1 < best_cost {
                best_cost = cost_1;
                best_alpha = 1.0;
            }
            if cost_05 < best_cost {
                best_cost = cost_05;
                best_alpha = 0.5;
            }
            if cost_025 < best_cost {
                best_cost = cost_025;
                best_alpha = 0.25;
            }
        }
        workgroupBarrier();

        // ── PHASE 7: Update qacc or signal convergence ──────────────────
        if best_alpha > 0.0 {
            if tid < nv {
                qacc_sh[tid] += best_alpha * search_sh[tid];
            }
        } else {
            // No step improved cost — converged (or stuck)
            if tid == 0u {
                done = 1u;
            }
        }
        workgroupBarrier();
    }

    // ── FINALIZE: Write qacc and efc_force to global memory ─────────────

    // Write final constrained acceleration
    if tid < nv {
        qacc_buf[env_nv + tid] = qacc_sh[tid];
    }

    // Write final constraint forces: force_i = -D_i * jar_i for active, 0 otherwise
    for (var i = tid; i < nefc; i += WG_SIZE) {
        var jar_i = 0.0;
        let row_off = env_efcj + i * nv;
        for (var k = 0u; k < nv; k++) {
            jar_i += efc_J_buf[row_off + k] * qacc_sh[k];
        }
        jar_i -= efc_aref_buf[env_efc + i];

        if jar_i < 0.0 {
            efc_force_buf[env_efc + i] = -efc_D_buf[env_efc + i] * jar_i;
        } else {
            efc_force_buf[env_efc + i] = 0.0;
        }
    }
}
