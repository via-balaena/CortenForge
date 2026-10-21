// crba.wgsl — Composite Rigid Body Algorithm.
//
// 4 entry points dispatched in sequence:
//   1. crba_init           — per body: crb = cinert
//   2. crba_backward       — per depth level (leaves→root): CAS accumulate
//   3. crba_mass_matrix    — per DOF: build mass matrix M
//   4. crba_cholesky       — single thread: dense Cholesky factorization
//
// 1–3 are the CPU's `mj_crba` (MuJoCo `mj_crb`). crb is cinert's format,
// MuJoCo's 10-vector `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz, m·dx, m·dy, m·dz, m]`
// about the tree's subtree centre of mass, the one point every body of a tree
// is about: two bodies' inertias add element by element.

// ── Constants ─────────────────────────────────────────────────────────

const DOF_PARENT_NONE: u32 = 0xFFFFFFFFu;

// ── Packed struct types ───────────────────────────────────────────────

struct FkParams {
    current_depth: u32,
    nbody: u32,
    njnt: u32,
    ngeom: u32,
    nv: u32,
    n_env: u32,
    nq: u32,
    _pad: u32,
};

struct BodyModel {
    parent: u32,
    depth: u32,
    jnt_adr: u32,
    jnt_num: u32,
    dof_adr: u32,
    dof_num: u32,
    mocap_id: u32,
    rootid: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    ipos: vec4<f32>,
    iquat: vec4<f32>,
    inertia: vec4<f32>,
};

struct DofModel {
    body_id: u32,
    parent: u32,
    armature: f32,
    damping: f32,
    simple: u32,
    m0: f32,
};

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: per-dispatch params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> dofs: array<DofModel>;

// Group 2: FK outputs (read-only for CRBA)
@group(2) @binding(0) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> cdof: array<vec4<f32>>;

// Group 3: CRBA outputs (read-write)
@group(3) @binding(0) var<storage, read_write> body_crb: array<atomic<u32>>;
@group(3) @binding(1) var<storage, read_write> qM: array<f32>;
@group(3) @binding(2) var<storage, read_write> qM_factor: array<f32>;

// ── CAS atomic f32 add ───────────────────────────────────────────────
// WGSL doesn't allow passing storage pointers as function arguments.
// Access the global `body_crb` directly via index.

fn atomic_add_f32_crb(idx: u32, val: f32) {
    var old = atomicLoad(&body_crb[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&body_crb[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

// Load crb for a body from atomic buffer (12 u32 → the 10-vector in 3 vec4):
// [0] = (Ixx, Iyy, Izz, Ixy), [1] = (Ixz, Iyz, m·dx, m·dy), [2] = (m·dz, m, 0, 0).
fn load_crb(base: u32) -> array<vec4<f32>, 3> {
    return array<vec4<f32>, 3>(
        vec4<f32>(
            bitcast<f32>(atomicLoad(&body_crb[base + 0u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 1u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 2u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 3u])),
        ),
        vec4<f32>(
            bitcast<f32>(atomicLoad(&body_crb[base + 4u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 5u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 6u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 7u])),
        ),
        vec4<f32>(
            bitcast<f32>(atomicLoad(&body_crb[base + 8u])),
            bitcast<f32>(atomicLoad(&body_crb[base + 9u])),
            0.0,
            0.0,
        ),
    );
}

// A 10-vector inertia times a motion vector `[ω; v]`, the momentum
// `[I·ω + h × v; m·v − h × ω]` (`h` = m·d), as the CPU's `mju_mul_inert_vec`
// (MuJoCo `mju_mulInertVec`). Returns (angular, linear).
fn inert_mul(i: array<vec4<f32>, 3>, w: vec3<f32>, v: vec3<f32>) -> array<vec3<f32>, 2> {
    let ixx = i[0].x; let iyy = i[0].y; let izz = i[0].z; let ixy = i[0].w;
    let ixz = i[1].x; let iyz = i[1].y;
    let h = vec3<f32>(i[1].z, i[1].w, i[2].x);
    let m = i[2].y;
    let iw = vec3<f32>(
        ixx * w.x + ixy * w.y + ixz * w.z,
        ixy * w.x + iyy * w.y + iyz * w.z,
        ixz * w.x + iyz * w.y + izz * w.z,
    );
    return array<vec3<f32>, 2>(iw + cross(h, v), m * v - cross(h, w));
}

// ── Entry point 1: Initialize crb from cinert ─────────────────────────

@compute @workgroup_size(64)
fn crba_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let env_off = env_id * params.nbody;
    let ci_base = (env_off + body_id) * 3u;
    let ci0 = body_cinert[ci_base + 0u];
    let ci1 = body_cinert[ci_base + 1u];
    let ci2 = body_cinert[ci_base + 2u];

    let crb_base = (env_off + body_id) * 12u;
    atomicStore(&body_crb[crb_base + 0u], bitcast<u32>(ci0.x));
    atomicStore(&body_crb[crb_base + 1u], bitcast<u32>(ci0.y));
    atomicStore(&body_crb[crb_base + 2u], bitcast<u32>(ci0.z));
    atomicStore(&body_crb[crb_base + 3u], bitcast<u32>(ci0.w));
    atomicStore(&body_crb[crb_base + 4u], bitcast<u32>(ci1.x));
    atomicStore(&body_crb[crb_base + 5u], bitcast<u32>(ci1.y));
    atomicStore(&body_crb[crb_base + 6u], bitcast<u32>(ci1.z));
    atomicStore(&body_crb[crb_base + 7u], bitcast<u32>(ci1.w));
    atomicStore(&body_crb[crb_base + 8u], bitcast<u32>(ci2.x));
    atomicStore(&body_crb[crb_base + 9u], bitcast<u32>(ci2.y));
    atomicStore(&body_crb[crb_base + 10u], 0u);
    atomicStore(&body_crb[crb_base + 11u], 0u);
}

// ── Entry point 2: Backward scan (CAS accumulation) ──────────────────

// Each body's composite inertia added into its parent's (not the world's):
// every body of a tree is about the same point, so the sum moves nothing.
@compute @workgroup_size(64)
fn crba_backward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let body = bodies[body_id];
    if body.depth != params.current_depth { return; }
    if body_id == 0u { return; }
    if body.parent == 0u { return; }  // world body has no DOFs — skip

    let env_off = env_id * params.nbody;
    let child_base = (env_off + body_id) * 12u;
    let parent_base = (env_off + body.parent) * 12u;
    for (var k = 0u; k < 10u; k++) {
        atomic_add_f32_crb(parent_base + k, bitcast<f32>(atomicLoad(&body_crb[child_base + k])));
    }
}

// ── Entry point 3: Mass matrix assembly ──────────────────────────────

// Dof i's row: its armature, then `cdof_j · (crb_body(i) · cdof_i)` for it and
// each dof j it descends from. A simple dof's row is its diagonal alone.
@compute @workgroup_size(64)
fn crba_mass_matrix(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dof_i = gid.x;
    let env_id = gid.y;
    if dof_i >= params.nv || env_id >= params.n_env { return; }

    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;
    let env_qm_off = env_id * params.nv * params.nv;

    let dof = dofs[dof_i];

    // A simple dof's row is its diagonal alone, the model's `dof_M0` (made at
    // qpos0), as the CPU's `mj_crba` (MuJoCo `mj_crb`), so two slides of one
    // body on one axis are not coupled.
    if dof.simple != 0u {
        qM[env_qm_off + dof_i * params.nv + dof_i] = dof.m0;
        return;
    }

    let crb = load_crb((env_body_off + dof.body_id) * 12u);
    let cdof_i_ang = cdof[(env_dof_off + dof_i) * 2u + 0u].xyz;
    let cdof_i_lin = cdof[(env_dof_off + dof_i) * 2u + 1u].xyz;
    let buf = inert_mul(crb, cdof_i_ang, cdof_i_lin);

    // Diagonal: M[i,i] = armature + cdof[i] · buf
    qM[env_qm_off + dof_i * params.nv + dof_i] =
        dof.armature + dot(cdof_i_ang, buf[0]) + dot(cdof_i_lin, buf[1]);

    // Off-diagonal: M[j,i] = cdof[j] · buf for each dof j on the dof_parent chain
    var j = dof.parent;
    loop {
        if j == DOF_PARENT_NONE { break; }

        let cdof_j_ang = cdof[(env_dof_off + j) * 2u + 0u].xyz;
        let cdof_j_lin = cdof[(env_dof_off + j) * 2u + 1u].xyz;
        let m_ji = dot(cdof_j_ang, buf[0]) + dot(cdof_j_lin, buf[1]);

        qM[env_qm_off + j * params.nv + dof_i] = m_ji;
        qM[env_qm_off + dof_i * params.nv + j] = m_ji;  // symmetry

        j = dofs[j].parent;
    }
}

// ── Entry point 4: Dense Cholesky factorization ──────────────────────

@compute @workgroup_size(64)
fn crba_cholesky(@builtin(global_invocation_id) gid: vec3<u32>) {
    let env_id = gid.y;
    if gid.x != 0u || env_id >= params.n_env { return; }

    let nv = params.nv;
    let env_qm_off = env_id * nv * nv;

    // Copy qM → qM_factor
    for (var i = 0u; i < nv; i++) {
        for (var jj = 0u; jj < nv; jj++) {
            qM_factor[env_qm_off + i * nv + jj] = qM[env_qm_off + i * nv + jj];
        }
    }

    // Dense Cholesky: lower triangular L such that M = L·L^T
    for (var j = 0u; j < nv; j++) {
        // Diagonal element
        var sum = qM_factor[env_qm_off + j * nv + j];
        for (var k = 0u; k < j; k++) {
            let ljk = qM_factor[env_qm_off + j * nv + k];
            sum -= ljk * ljk;
        }
        qM_factor[env_qm_off + j * nv + j] = sqrt(max(sum, 1e-10));

        let ljj = qM_factor[env_qm_off + j * nv + j];

        // Off-diagonal elements (column j, rows i > j)
        for (var i = j + 1u; i < nv; i++) {
            var s = qM_factor[env_qm_off + i * nv + j];
            for (var k = 0u; k < j; k++) {
                s -= qM_factor[env_qm_off + i * nv + k]
                   * qM_factor[env_qm_off + j * nv + k];
            }
            qM_factor[env_qm_off + i * nv + j] = s / ljj;
            // Zero upper triangle
            qM_factor[env_qm_off + j * nv + i] = 0.0;
        }
    }
}
