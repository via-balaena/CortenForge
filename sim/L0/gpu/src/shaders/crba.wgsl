// crba.wgsl — Composite Rigid Body Algorithm.
//
// 4 entry points dispatched in sequence:
//   1. crba_init           — per body: convert cinert → crb format
//   2. crba_backward       — per depth level (leaves→root): CAS accumulate
//   3. crba_mass_matrix    — per DOF: build mass matrix M
//   4. crba_cholesky       — single thread: dense Cholesky factorization
//
// crb format: (m, m·h_x, m·h_y, m·h_z, I_ref_00..I_ref_22, pad, pad)
// where I_ref = I_COM + m·(|h|²·I₃ − h⊗h) (rotational inertia about body origin).
// This format is additive: element-wise addition of two crb values about
// the same reference point gives the correct combined spatial inertia.

// ── Common math (duplicated from fk.wgsl — no WGSL includes) ─────────

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let u = q.xyz;
    let s = q.w;
    return 2.0 * dot(u, v) * u
         + (s * s - dot(u, u)) * v
         + 2.0 * s * cross(u, v);
}

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
    _pad: u32,
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
    _pad: u32,
};

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: per-dispatch params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> dofs: array<DofModel>;

// Group 2: FK outputs (read-only for CRBA)
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> cdof: array<vec4<f32>>;

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

// Load crb for a body from atomic buffer (12 u32 → 10 useful f32).
// Returns: [0]=(m, mh_x, mh_y, mh_z), [1]=(I00,I01,I02,I11), [2]=(I12,I22,0,0)
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

// Store crb for a body into atomic buffer (10 useful f32 → 12 u32).
fn store_crb(base: u32, crb: array<vec4<f32>, 3>) {
    atomicStore(&body_crb[base + 0u], bitcast<u32>(crb[0].x));
    atomicStore(&body_crb[base + 1u], bitcast<u32>(crb[0].y));
    atomicStore(&body_crb[base + 2u], bitcast<u32>(crb[0].z));
    atomicStore(&body_crb[base + 3u], bitcast<u32>(crb[0].w));
    atomicStore(&body_crb[base + 4u], bitcast<u32>(crb[1].x));
    atomicStore(&body_crb[base + 5u], bitcast<u32>(crb[1].y));
    atomicStore(&body_crb[base + 6u], bitcast<u32>(crb[1].z));
    atomicStore(&body_crb[base + 7u], bitcast<u32>(crb[1].w));
    atomicStore(&body_crb[base + 8u], bitcast<u32>(crb[2].x));
    atomicStore(&body_crb[base + 9u], bitcast<u32>(crb[2].y));
    atomicStore(&body_crb[base + 10u], 0u);
    atomicStore(&body_crb[base + 11u], 0u);
}

// Shift crb from child origin to parent origin.
// d = xpos[child] - xpos[parent].
fn shift_crb(crb: array<vec4<f32>, 3>, d: vec3<f32>) -> array<vec4<f32>, 3> {
    let m = crb[0].x;
    if m < 1e-20 {
        return crb;
    }
    let mh = vec3<f32>(crb[0].y, crb[0].z, crb[0].w);
    let h = mh / m;
    let h_new = h + d;
    let mh_new = mh + m * d;

    let hh = dot(h, h);
    let hh_new = dot(h_new, h_new);
    let dhh = hh_new - hh;

    // I_ref_new[r][c] = I_ref_old[r][c] + m * (dhh * δ_rc - (h_new[r]*h_new[c] - h[r]*h[c]))
    let I00 = crb[1].x + m * (dhh - (h_new.x * h_new.x - h.x * h.x));
    let I01 = crb[1].y + m * (-(h_new.x * h_new.y - h.x * h.y));
    let I02 = crb[1].z + m * (-(h_new.x * h_new.z - h.x * h.z));
    let I11 = crb[1].w + m * (dhh - (h_new.y * h_new.y - h.y * h.y));
    let I12 = crb[2].x + m * (-(h_new.y * h_new.z - h.y * h.z));
    let I22 = crb[2].y + m * (dhh - (h_new.z * h_new.z - h.z * h.z));

    return array<vec4<f32>, 3>(
        vec4<f32>(m, mh_new.x, mh_new.y, mh_new.z),
        vec4<f32>(I00, I01, I02, I11),
        vec4<f32>(I12, I22, 0.0, 0.0),
    );
}

// Implicit 6×6 spatial inertia × cdof multiply (no matrix materialization).
// crb = (m, mh, I_ref). cdof stored as 2×vec4 (angular, linear).
// Returns buf = (buf_angular, buf_linear) as 2 vec3.
fn crb_mul_cdof(crb: array<vec4<f32>, 3>, cdof_ang: vec3<f32>, cdof_lin: vec3<f32>) -> array<vec3<f32>, 2> {
    let m = crb[0].x;
    let mh = vec3<f32>(crb[0].y, crb[0].z, crb[0].w);

    // I_ref upper triangle
    let I00 = crb[1].x; let I01 = crb[1].y; let I02 = crb[1].z;
    let I11 = crb[1].w; let I12 = crb[2].x; let I22 = crb[2].y;

    // I_ref * omega (symmetric 3×3 matmul)
    let Iw = vec3<f32>(
        I00 * cdof_ang.x + I01 * cdof_ang.y + I02 * cdof_ang.z,
        I01 * cdof_ang.x + I11 * cdof_ang.y + I12 * cdof_ang.z,
        I02 * cdof_ang.x + I12 * cdof_ang.y + I22 * cdof_ang.z,
    );

    // buf_angular = I_ref * ω + cross(mh, v)
    let buf_ang = Iw + cross(mh, cdof_lin);
    // buf_linear = cross(ω, mh) + m * v
    let buf_lin = cross(cdof_ang, mh) + m * cdof_lin;

    return array<vec3<f32>, 2>(buf_ang, buf_lin);
}

// ── Entry point 1: Initialize crb from cinert ─────────────────────────

@compute @workgroup_size(64)
fn crba_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let env_off = env_id * params.nbody;
    let ci_base = (env_off + body_id) * 3u;

    // Read cinert: (m, h.x, h.y, h.z), (I_COM_00..I_COM_11), (I_COM_12, I_COM_22, 0, 0)
    let ci0 = body_cinert[ci_base + 0u];
    let ci1 = body_cinert[ci_base + 1u];
    let ci2 = body_cinert[ci_base + 2u];

    let m = ci0.x;
    let h = ci0.yzw;
    let mh = m * h;

    // Parallel axis: I_ref = I_COM + m * (|h|²·I₃ − h⊗h)
    let hh = dot(h, h);
    let I_ref_00 = ci1.x + m * (hh - h.x * h.x);
    let I_ref_01 = ci1.y + m * (0.0 - h.x * h.y);
    let I_ref_02 = ci1.z + m * (0.0 - h.x * h.z);
    let I_ref_11 = ci1.w + m * (hh - h.y * h.y);
    let I_ref_12 = ci2.x + m * (0.0 - h.y * h.z);
    let I_ref_22 = ci2.y + m * (hh - h.z * h.z);

    let crb_base = (env_off + body_id) * 12u;
    let crb_val = array<vec4<f32>, 3>(
        vec4<f32>(m, mh.x, mh.y, mh.z),
        vec4<f32>(I_ref_00, I_ref_01, I_ref_02, I_ref_11),
        vec4<f32>(I_ref_12, I_ref_22, 0.0, 0.0),
    );
    store_crb(crb_base, crb_val);
}

// ── Entry point 2: Backward scan (CAS accumulation) ──────────────────

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

    // Load child crb
    let child_base = (env_off + body_id) * 12u;
    let child_crb = load_crb(child_base);

    // Shift from child origin to parent origin
    let d = body_xpos[env_off + body_id].xyz - body_xpos[env_off + body.parent].xyz;
    let shifted = shift_crb(child_crb, d);

    // CAS-atomic-add 10 useful floats into parent's crb
    let parent_base = (env_off + body.parent) * 12u;
    atomic_add_f32_crb(parent_base + 0u, shifted[0].x);   // m
    atomic_add_f32_crb(parent_base + 1u, shifted[0].y);   // mh_x
    atomic_add_f32_crb(parent_base + 2u, shifted[0].z);   // mh_y
    atomic_add_f32_crb(parent_base + 3u, shifted[0].w);   // mh_z
    atomic_add_f32_crb(parent_base + 4u, shifted[1].x);   // I_ref_00
    atomic_add_f32_crb(parent_base + 5u, shifted[1].y);   // I_ref_01
    atomic_add_f32_crb(parent_base + 6u, shifted[1].z);   // I_ref_02
    atomic_add_f32_crb(parent_base + 7u, shifted[1].w);   // I_ref_11
    atomic_add_f32_crb(parent_base + 8u, shifted[2].x);   // I_ref_12
    atomic_add_f32_crb(parent_base + 9u, shifted[2].y);   // I_ref_22
}

// ── Entry point 3: Mass matrix assembly ──────────────────────────────

@compute @workgroup_size(64)
fn crba_mass_matrix(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dof_i = gid.x;
    let env_id = gid.y;
    if dof_i >= params.nv || env_id >= params.n_env { return; }

    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;
    let env_qm_off = env_id * params.nv * params.nv;

    let dof = dofs[dof_i];
    let body_i = dof.body_id;

    // Load crb for body_i
    let crb_base = (env_body_off + body_i) * 12u;
    let crb = load_crb(crb_base);

    // cdof[dof_i] — angular and linear parts
    let cdof_i_ang = cdof[(env_dof_off + dof_i) * 2u + 0u].xyz;
    let cdof_i_lin = cdof[(env_dof_off + dof_i) * 2u + 1u].xyz;

    // buf = crb × cdof[i] (implicit 6×6 multiply)
    var buf = crb_mul_cdof(crb, cdof_i_ang, cdof_i_lin);

    // Diagonal: M[i,i] = cdof[i]^T · buf + armature
    let diag = dot(cdof_i_ang, buf[0]) + dot(cdof_i_lin, buf[1]) + dof.armature;
    qM[env_qm_off + dof_i * params.nv + dof_i] = diag;

    // Off-diagonal: walk dof_parent chain with spatial force transport
    var current_body = body_i;
    var j = dof.parent;
    loop {
        if j == DOF_PARENT_NONE { break; }

        let dof_j = dofs[j];
        let body_j = dof_j.body_id;

        // Spatial force transport when crossing body boundaries
        if body_j != current_body {
            let r = body_xpos[env_body_off + current_body].xyz
                  - body_xpos[env_body_off + body_j].xyz;
            // angular += cross(r, linear)
            buf[0] = buf[0] + cross(r, buf[1]);
            current_body = body_j;
        }

        // M[j,i] = cdof[j]^T · buf
        let cdof_j_ang = cdof[(env_dof_off + j) * 2u + 0u].xyz;
        let cdof_j_lin = cdof[(env_dof_off + j) * 2u + 1u].xyz;
        let m_ji = dot(cdof_j_ang, buf[0]) + dot(cdof_j_lin, buf[1]);

        qM[env_qm_off + j * params.nv + dof_i] = m_ji;
        qM[env_qm_off + dof_i * params.nv + j] = m_ji;  // symmetry

        j = dof_j.parent;
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
