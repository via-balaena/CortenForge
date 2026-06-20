// map_forces.wgsl — Map constraint forces from constraint space to joint space.
//
// 1 entry point:
//   map_forces — per DOF: qfrc_constraint[d] = Σ_i efc_force[i] × efc_J[i * nv + d]
//
// Each thread handles one degree of freedom. It reads the current constraint
// count (written by the collision/constraint pipeline), then accumulates the
// transposed Jacobian-force product across all active constraints.

// ── Solver parameters ─────────────────────────────────────────────────

struct SolverParams {
    nv: u32,
    max_iter: u32,
    max_ls: u32,
    n_env: u32,
    tolerance: f32,
    ls_tolerance: f32,
    meaninertia: f32,
    max_constraints: u32,
};

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: solver parameters
@group(0) @binding(0) var<uniform> params: SolverParams;

// Group 1: constraint data (read-only)
@group(1) @binding(0) var<storage, read> efc_J_buf: array<f32>;
@group(1) @binding(1) var<storage, read> efc_force_buf: array<f32>;
@group(1) @binding(2) var<storage, read_write> constraint_count_buf: array<atomic<u32>>;

// Group 2: output (read-write)
@group(2) @binding(0) var<storage, read_write> qfrc_constraint_buf: array<f32>;

// ── Entry point ───────────────────────────────────────────────────────

@compute @workgroup_size(64)
fn map_forces(@builtin(global_invocation_id) gid: vec3<u32>) {
    let d = gid.x;
    let env_id = gid.y;
    if d >= params.nv || env_id >= params.n_env {
        return;
    }

    // Per-env base offsets: efc rows stride by max_constraints, qfrc by nv.
    let env_efc = env_id * params.max_constraints;
    let env_nv = env_id * params.nv;

    // Read the number of active constraints in THIS env (per-env atomic).
    let nefc = atomicLoad(&constraint_count_buf[env_id]);

    // Accumulate J^T × force for this DOF.
    var total = 0.0;
    for (var i = 0u; i < nefc; i++) {
        total += efc_force_buf[env_efc + i] * efc_J_buf[(env_efc + i) * params.nv + d];
    }

    qfrc_constraint_buf[env_nv + d] = total;
}
