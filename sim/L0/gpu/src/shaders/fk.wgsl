// fk.wgsl — GPU forward kinematics.
//
// 4 entry points dispatched in sequence:
//   1. fk_forward           — per depth level (body poses, cinert, cdof)
//   2. fk_geom_poses        — all geoms parallel (after FK)
//   3. fk_subtree_backward  — per depth level, leaves→root
//   4. fk_subtree_normalize — all bodies parallel
//
// Quaternion layout: vec4<f32> = (x, y, z, w) throughout.
// Matches nalgebra's memory order.

// ── Common math ───────────────────────────────────────────────────────

fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    );
}

fn quat_normalize(q: vec4<f32>) -> vec4<f32> {
    let len_sq = dot(q, q);
    if (len_sq < 1e-14) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    return q * inverseSqrt(len_sq);
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let u = q.xyz;
    let s = q.w;
    return 2.0 * dot(u, v) * u
         + (s * s - dot(u, u)) * v
         + 2.0 * s * cross(u, v);
}

fn quat_from_axis_angle(axis: vec3<f32>, angle: f32) -> vec4<f32> {
    let half = angle * 0.5;
    return vec4<f32>(axis * sin(half), cos(half));
}

fn quat_to_mat3(q: vec4<f32>) -> mat3x3<f32> {
    let x = q.x; let y = q.y; let z = q.z; let w = q.w;
    let x2 = x + x; let y2 = y + y; let z2 = z + z;
    let xx = x * x2; let xy = x * y2; let xz = x * z2;
    let yy = y * y2; let yz = y * z2; let zz = z * z2;
    let wx = w * x2; let wy = w * y2; let wz = w * z2;
    // Column-major: mat[col][row]
    return mat3x3<f32>(
        vec3<f32>(1.0 - (yy + zz), xy + wz, xz - wy),
        vec3<f32>(xy - wz, 1.0 - (xx + zz), yz + wx),
        vec3<f32>(xz + wy, yz - wx, 1.0 - (xx + yy)),
    );
}

// ── Constants ─────────────────────────────────────────────────────────

const JNT_FREE: u32  = 0u;
const JNT_BALL: u32  = 1u;
const JNT_HINGE: u32 = 2u;
const JNT_SLIDE: u32 = 3u;
const MOCAP_NONE: u32 = 0xFFFFFFFFu;

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
    inertia: vec4<f32>,  // xyz = diag inertia, w = mass
};

struct JointModel {
    jtype: u32,
    qpos_adr: u32,
    dof_adr: u32,
    body_id: u32,
    axis: vec4<f32>,
    pos: vec4<f32>,
};

struct GeomModel {
    body_id: u32,
    geom_type: u32,
    contype: u32,
    conaffinity: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    size: vec4<f32>,
    friction: vec4<f32>,
    sdf_meta_idx: u32,
    condim: u32,
    _pad0: u32,
    _pad1: u32,
};

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: per-dispatch params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only, packed structs)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> joints: array<JointModel>;

// Group 2: per-env state (read-write)
@group(2) @binding(0) var<storage, read> qpos: array<f32>;
@group(2) @binding(1) var<storage, read_write> body_xpos: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read_write> body_xquat: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read_write> body_xipos: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read_write> body_cinert: array<vec4<f32>>;
@group(2) @binding(5) var<storage, read_write> cdof: array<vec4<f32>>;
@group(2) @binding(6) var<storage, read_write> subtree_mass: array<f32>;
@group(2) @binding(7) var<storage, read_write> subtree_com: array<vec4<f32>>;
@group(2) @binding(8) var<storage, read> mocap_pos: array<vec4<f32>>;
@group(2) @binding(9) var<storage, read> mocap_quat: array<vec4<f32>>;

// Group 3: geom data (model + output)
@group(3) @binding(0) var<storage, read> geom_models: array<GeomModel>;
@group(3) @binding(1) var<storage, read_write> geom_xpos: array<vec4<f32>>;
@group(3) @binding(2) var<storage, read_write> geom_xmat: array<vec4<f32>>;

// ── Entry point 1: Forward FK ─────────────────────────────────────────

@compute @workgroup_size(64)
fn fk_forward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    let body = bodies[body_id];
    if (body.depth != params.current_depth) { return; }

    let env_off = env_id * params.nbody;

    // World body: identity pose, zero cinert
    if (body_id == 0u) {
        body_xpos[env_off] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        body_xquat[env_off] = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        body_xipos[env_off] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        let ci_base = env_off * 3u;
        body_cinert[ci_base + 0u] = vec4<f32>(0.0);
        body_cinert[ci_base + 1u] = vec4<f32>(0.0);
        body_cinert[ci_base + 2u] = vec4<f32>(0.0);
        subtree_mass[env_off] = body.inertia.w;
        subtree_com[env_off] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // Read parent pose (written in previous depth pass)
    let parent = body.parent;
    var pos = body_xpos[env_off + parent].xyz;
    var quat = body_xquat[env_off + parent];

    // Check mocap
    if (body.mocap_id != MOCAP_NONE) {
        pos = mocap_pos[body.mocap_id].xyz;
        quat = quat_normalize(mocap_quat[body.mocap_id]);
    } else {
        // Apply body offset in parent frame
        pos += quat_rotate(quat, body.pos.xyz);
        quat = quat_mul(quat, body.quat);

        // Joint loop
        let jnt_start = body.jnt_adr;
        let jnt_count = body.jnt_num;
        for (var ji = 0u; ji < jnt_count; ji++) {
            let j = jnt_start + ji;
            let jnt = joints[j];
            let qa = jnt.qpos_adr;  // For n_env=1, qpos offset is just adr

            if (jnt.jtype == JNT_FREE) {
                pos = vec3<f32>(qpos[qa], qpos[qa + 1u], qpos[qa + 2u]);
                // qpos layout: [w, x, y, z] → GPU layout: (x, y, z, w)
                quat = quat_normalize(vec4<f32>(
                    qpos[qa + 4u], qpos[qa + 5u], qpos[qa + 6u], qpos[qa + 3u]
                ));
            } else if (jnt.jtype == JNT_HINGE) {
                let angle = qpos[qa];
                let world_axis = quat_rotate(quat, jnt.axis.xyz);
                let world_anchor = pos + quat_rotate(quat, jnt.pos.xyz);

                // Normalize axis (guard degenerate)
                let axis_len = length(world_axis);
                var norm_axis = world_axis;
                if (axis_len > 1e-10) {
                    norm_axis = world_axis / axis_len;
                }

                let rot = quat_from_axis_angle(norm_axis, angle);
                // LEFT multiply (world-frame rotation)
                quat = quat_mul(rot, quat);
                // Pivot-point rotation
                pos = world_anchor + quat_rotate(rot, pos - world_anchor);
            } else if (jnt.jtype == JNT_BALL) {
                // qpos layout: [w, x, y, z] → GPU layout: (x, y, z, w)
                let q = quat_normalize(vec4<f32>(
                    qpos[qa + 1u], qpos[qa + 2u], qpos[qa + 3u], qpos[qa + 0u]
                ));
                // RIGHT multiply (local-frame rotation) — matches CPU
                quat = quat_mul(quat, q);
            } else if (jnt.jtype == JNT_SLIDE) {
                pos += quat_rotate(quat, jnt.axis.xyz) * qpos[qa];
            }
        }

        // Normalize after all joints
        quat = quat_normalize(quat);
    }

    // Write body pose
    body_xpos[env_off + body_id] = vec4<f32>(pos, 0.0);
    body_xquat[env_off + body_id] = quat;

    // ── Derived: xipos ────────────────────────────────────────────
    let xipos = pos + quat_rotate(quat, body.ipos.xyz);
    body_xipos[env_off + body_id] = vec4<f32>(xipos, 0.0);

    // ── Cinert (12 floats = 3 × vec4) ────────────────────────────
    let iquat = quat_mul(quat, body.iquat);
    let ximat = quat_to_mat3(iquat);
    let inertia = body.inertia.xyz;  // diagonal body-frame inertia
    let mass = body.inertia.w;

    // I_world = ximat * diag(inertia) * ximat^T — upper triangle only.
    // ximat is column-major: ximat[col][row].
    // Element (r,c) = Σ_k ximat[k][r] * inertia[k] * ximat[k][c]
    let i00 = ximat[0][0] * inertia.x * ximat[0][0]
            + ximat[1][0] * inertia.y * ximat[1][0]
            + ximat[2][0] * inertia.z * ximat[2][0];
    let i01 = ximat[0][0] * inertia.x * ximat[0][1]
            + ximat[1][0] * inertia.y * ximat[1][1]
            + ximat[2][0] * inertia.z * ximat[2][1];
    let i02 = ximat[0][0] * inertia.x * ximat[0][2]
            + ximat[1][0] * inertia.y * ximat[1][2]
            + ximat[2][0] * inertia.z * ximat[2][2];
    let i11 = ximat[0][1] * inertia.x * ximat[0][1]
            + ximat[1][1] * inertia.y * ximat[1][1]
            + ximat[2][1] * inertia.z * ximat[2][1];
    let i12 = ximat[0][1] * inertia.x * ximat[0][2]
            + ximat[1][1] * inertia.y * ximat[1][2]
            + ximat[2][1] * inertia.z * ximat[2][2];
    let i22 = ximat[0][2] * inertia.x * ximat[0][2]
            + ximat[1][2] * inertia.y * ximat[1][2]
            + ximat[2][2] * inertia.z * ximat[2][2];

    let h = xipos - pos;  // COM offset from body origin in world frame

    let ci_base = (env_off + body_id) * 3u;
    body_cinert[ci_base + 0u] = vec4<f32>(mass, h.x, h.y, h.z);
    body_cinert[ci_base + 1u] = vec4<f32>(i00, i01, i02, i11);
    body_cinert[ci_base + 2u] = vec4<f32>(i12, i22, 0.0, 0.0);

    // ── cdof (motion subspace, 2 × vec4 per DOF) ─────────────────
    let dof_start = body.dof_adr;
    let jnt_start2 = body.jnt_adr;
    let n_jnt = body.jnt_num;
    let env_dof_off = env_id * params.nv;

    var dof_idx = 0u;
    for (var ji = 0u; ji < n_jnt; ji++) {
        let j = jnt_start2 + ji;
        let jnt = joints[j];
        let dof_base = env_dof_off + dof_start + dof_idx;

        if (jnt.jtype == JNT_HINGE) {
            let axis = quat_rotate(quat, jnt.axis.xyz);
            let anchor = pos + quat_rotate(quat, jnt.pos.xyz);
            let r = pos - anchor;
            let lin = cross(axis, r);
            cdof[dof_base * 2u + 0u] = vec4<f32>(axis, 0.0);
            cdof[dof_base * 2u + 1u] = vec4<f32>(lin, 0.0);
            dof_idx += 1u;
        } else if (jnt.jtype == JNT_SLIDE) {
            let axis = quat_rotate(quat, jnt.axis.xyz);
            cdof[dof_base * 2u + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            cdof[dof_base * 2u + 1u] = vec4<f32>(axis, 0.0);
            dof_idx += 1u;
        } else if (jnt.jtype == JNT_BALL) {
            let rot = quat_to_mat3(quat);
            for (var k = 0u; k < 3u; k++) {
                cdof[(dof_base + k) * 2u + 0u] = vec4<f32>(rot[k], 0.0);
                cdof[(dof_base + k) * 2u + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            }
            dof_idx += 3u;
        } else if (jnt.jtype == JNT_FREE) {
            // Linear DOFs (0-2): world basis
            cdof[(dof_base + 0u) * 2u + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            cdof[(dof_base + 0u) * 2u + 1u] = vec4<f32>(1.0, 0.0, 0.0, 0.0);
            cdof[(dof_base + 1u) * 2u + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            cdof[(dof_base + 1u) * 2u + 1u] = vec4<f32>(0.0, 1.0, 0.0, 0.0);
            cdof[(dof_base + 2u) * 2u + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            cdof[(dof_base + 2u) * 2u + 1u] = vec4<f32>(0.0, 0.0, 1.0, 0.0);
            // Angular DOFs (3-5): body-frame axes rotated to world
            let rot = quat_to_mat3(quat);
            for (var k = 0u; k < 3u; k++) {
                cdof[(dof_base + 3u + k) * 2u + 0u] = vec4<f32>(rot[k], 0.0);
                cdof[(dof_base + 3u + k) * 2u + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            }
            dof_idx += 6u;
        }
    }

    // ── Subtree COM init ──────────────────────────────────────────
    subtree_mass[env_off + body_id] = mass;
    subtree_com[env_off + body_id] = vec4<f32>(mass * xipos, 0.0);
}

// ── Entry point 2: Geom poses ─────────────────────────────────────────

@compute @workgroup_size(64)
fn fk_geom_poses(@builtin(global_invocation_id) gid: vec3<u32>) {
    let geom_id = gid.x;
    let env_id = gid.y;
    if (geom_id >= params.ngeom || env_id >= params.n_env) { return; }

    let geom = geom_models[geom_id];
    let env_body_off = env_id * params.nbody;
    let pos = body_xpos[env_body_off + geom.body_id].xyz;
    let quat = body_xquat[env_body_off + geom.body_id];

    let env_geom_off = env_id * params.ngeom;
    let gidx = env_geom_off + geom_id;

    // World position
    geom_xpos[gidx] = vec4<f32>(pos + quat_rotate(quat, geom.pos.xyz), 0.0);

    // World rotation matrix (3 × vec4, column-major)
    let gquat = quat_mul(quat, geom.quat);
    let gmat = quat_to_mat3(gquat);
    let mat_base = gidx * 3u;
    geom_xmat[mat_base + 0u] = vec4<f32>(gmat[0], 0.0);
    geom_xmat[mat_base + 1u] = vec4<f32>(gmat[1], 0.0);
    geom_xmat[mat_base + 2u] = vec4<f32>(gmat[2], 0.0);
}

// ── Entry point 3: Subtree COM backward ───────────────────────────────

@compute @workgroup_size(64)
fn fk_subtree_backward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    let body = bodies[body_id];
    if (body.depth != params.current_depth) { return; }
    if (body_id == 0u) { return; }

    let env_off = env_id * params.nbody;
    let parent = body.parent;
    let parent_idx = env_off + parent;
    let child_idx = env_off + body_id;

    // Accumulate child into parent.
    // Race condition note: multiple children at the same depth may share
    // a parent. For small trees with n_env=1 (Session 1 scope), each
    // depth level fits in one workgroup and results are deterministic.
    // Session 2 adds CAS atomics for larger trees.
    subtree_mass[parent_idx] = subtree_mass[parent_idx] + subtree_mass[child_idx];
    subtree_com[parent_idx] = subtree_com[parent_idx] + subtree_com[child_idx];
}

// ── Entry point 4: Subtree COM normalize ──────────────────────────────

@compute @workgroup_size(64)
fn fk_subtree_normalize(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    let idx = env_id * params.nbody + body_id;
    let m = subtree_mass[idx];
    if (m > 1e-10) {
        subtree_com[idx] = subtree_com[idx] / m;
    } else {
        subtree_com[idx] = body_xipos[idx];
    }
}
