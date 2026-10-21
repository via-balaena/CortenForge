// fk.wgsl — GPU forward kinematics.
//
// 5 entry points dispatched in sequence:
//   1. fk_forward           — per depth level (body poses, joint axes and anchors)
//   2. fk_geom_poses        — all geoms parallel (after FK)
//   3. fk_subtree_backward  — one thread per env, leaves→root reduction
//   4. fk_subtree_normalize — all bodies parallel
//   5. fk_com_pos           — all bodies parallel (cinert, cdof)
//
// 1 is MuJoCo's `mj_kinematics`; 3–5 its `mj_comPos`: each body's inertia
// (`cinert`) and each dof's motion axis (`cdof`) in the world's orientation
// about its tree's subtree centre of mass, `subtree_com[body_rootid]`, as the
// CPU's.
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
/// MuJoCo's `mjMINVAL`.
const MJ_MINVAL: f32 = 1e-15;

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
    inertia: vec4<f32>,  // xyz = diag inertia, w = mass
};

struct JointModel {
    jtype: u32,
    qpos_adr: u32,
    dof_adr: u32,
    body_id: u32,
    axis: vec4<f32>,  // xyz = axis, w = qpos0 of a hinge or a slide
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
@group(2) @binding(4) var<storage, read_write> subtree_mass: array<f32>;
@group(2) @binding(5) var<storage, read_write> subtree_com: array<vec4<f32>>;
@group(2) @binding(6) var<storage, read> mocap_pos: array<vec4<f32>>;
@group(2) @binding(7) var<storage, read> mocap_quat: array<vec4<f32>>;
@group(2) @binding(8) var<storage, read_write> jnt_xaxis: array<vec4<f32>>;
@group(2) @binding(9) var<storage, read_write> jnt_xanchor: array<vec4<f32>>;

// Group 3: geom data (model + output), read by fk_geom_poses; and, at its
// own bindings, fk_com_pos's outputs. fk_com_pos's pipeline layout holds
// group 3's bindings 3 and 4, the other passes' layout its bindings 0–2 (one
// layout holding both would pass the device's 16 storage buffers per stage).
@group(3) @binding(0) var<storage, read> geom_models: array<GeomModel>;
@group(3) @binding(1) var<storage, read_write> geom_xpos: array<vec4<f32>>;
@group(3) @binding(2) var<storage, read_write> geom_xmat: array<vec4<f32>>;
@group(3) @binding(3) var<storage, read_write> body_cinert: array<vec4<f32>>;
@group(3) @binding(4) var<storage, read_write> cdof: array<vec4<f32>>;

// ── Entry point 1: Forward FK ─────────────────────────────────────────

@compute @workgroup_size(64)
fn fk_forward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    let body = bodies[body_id];
    if (body.depth != params.current_depth) { return; }

    let env_off = env_id * params.nbody;

    // World body: identity pose
    if (body_id == 0u) {
        body_xpos[env_off] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        body_xquat[env_off] = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        body_xipos[env_off] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
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

        // Joint loop. Each joint's axis and anchor are written in the frame
        // before it turns (MuJoCo's `xaxis`/`xanchor`, the CPU's), which
        // fk_com_pos reads: a later joint of the same body does not turn an
        // earlier one's.
        let jnt_start = body.jnt_adr;
        let jnt_count = body.jnt_num;
        for (var ji = 0u; ji < jnt_count; ji++) {
            let j = jnt_start + ji;
            let jnt = joints[j];
            // Env-strided qpos base: env k's coordinates start at k·nq. All the
            // joint-type reads below index off `qa`, so this single offset makes
            // every qpos read env-correct (byte-identical at n_env=1: env_id=0).
            let qa = env_id * params.nq + jnt.qpos_adr;
            let xj = env_id * params.njnt + j;

            if (jnt.jtype == JNT_FREE) {
                pos = vec3<f32>(qpos[qa], qpos[qa + 1u], qpos[qa + 2u]);
                // qpos layout: [w, x, y, z] → GPU layout: (x, y, z, w)
                quat = quat_normalize(vec4<f32>(
                    qpos[qa + 4u], qpos[qa + 5u], qpos[qa + 6u], qpos[qa + 3u]
                ));
                jnt_xaxis[xj] = vec4<f32>(jnt.axis.xyz, 0.0);
                jnt_xanchor[xj] = vec4<f32>(pos, 0.0);
            } else if (jnt.jtype == JNT_HINGE) {
                let angle = qpos[qa] - jnt.axis.w;
                let world_axis = quat_rotate(quat, jnt.axis.xyz);
                let world_anchor = pos + quat_rotate(quat, jnt.pos.xyz);
                jnt_xaxis[xj] = vec4<f32>(world_axis, 0.0);
                jnt_xanchor[xj] = vec4<f32>(world_anchor, 0.0);

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
                // A ball turns about the body's origin here (see
                // `GpuFkPipeline`), so that is its anchor.
                jnt_xaxis[xj] = vec4<f32>(quat_rotate(quat, jnt.axis.xyz), 0.0);
                jnt_xanchor[xj] = vec4<f32>(pos, 0.0);
                // qpos layout: [w, x, y, z] → GPU layout: (x, y, z, w)
                let q = quat_normalize(vec4<f32>(
                    qpos[qa + 1u], qpos[qa + 2u], qpos[qa + 3u], qpos[qa + 0u]
                ));
                // RIGHT multiply (local-frame rotation) — matches CPU
                quat = quat_mul(quat, q);
            } else if (jnt.jtype == JNT_SLIDE) {
                let world_axis = quat_rotate(quat, jnt.axis.xyz);
                jnt_xaxis[xj] = vec4<f32>(world_axis, 0.0);
                jnt_xanchor[xj] = vec4<f32>(pos + quat_rotate(quat, jnt.pos.xyz), 0.0);
                pos += world_axis * (qpos[qa] - jnt.axis.w);
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

    // ── Subtree COM init ──────────────────────────────────────────
    let mass = body.inertia.w;
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

// One thread per env performs the entire children→parent reduction
// SEQUENTIALLY, mirroring CPU `forward/position.rs`. Bodies are stored in
// topological order (a parent's index is always < its children's), so a
// single reverse sweep accumulates every body into its parent before the
// parent is itself accumulated — race-free WITHOUT atomics, since each
// thread touches only its own env's slots. The earlier per-body, per-depth
// scatter raced when sibling branches shared a parent (two non-atomic
// `subtree[parent] += child` writes collided and one was lost). The
// per-env parallelism kept here is the axis that matters for batching;
// the O(nbody) serial reduction is negligible beside the FK forward pass
// (same single-thread-per-env convention as smooth_solve / eulerdamp).
@compute @workgroup_size(64)
fn fk_subtree_backward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let env_id = gid.x;
    if (env_id >= params.n_env) { return; }

    let env_off = env_id * params.nbody;

    // Reverse sweep: b runs nbody → 2, body_id = b - 1 runs nbody-1 → 1.
    // Phrased this way to keep the u32 loop counter from underflowing.
    for (var b = params.nbody; b > 1u; b--) {
        let body_id = b - 1u;
        let parent = bodies[body_id].parent;
        let parent_idx = env_off + parent;
        let child_idx = env_off + body_id;
        subtree_mass[parent_idx] = subtree_mass[parent_idx] + subtree_mass[child_idx];
        subtree_com[parent_idx] = subtree_com[parent_idx] + subtree_com[child_idx];
    }
}

// ── Entry point 4: Subtree COM normalize ──────────────────────────────

@compute @workgroup_size(64)
fn fk_subtree_normalize(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    // As the CPU's (MuJoCo `mj_comPos`): a subtree of less than mjMINVAL of
    // mass takes the body's own centre of mass; another is scaled by the
    // reciprocal of its mass.
    let idx = env_id * params.nbody + body_id;
    let m = subtree_mass[idx];
    if (m < MJ_MINVAL) {
        subtree_com[idx] = body_xipos[idx];
    } else {
        subtree_com[idx] = subtree_com[idx] * (1.0 / m);
    }
}

// ── Entry point 5: Inertias and motion axes about the subtree COM ─────

// A rotation about `axis` through a point `offset` away from the reference
// point (`offset` = reference − point), `[axis; axis × offset]`, written at
// `cdof[dof]` (MuJoCo's `mju_dofCom`).
fn write_rotation_dof(dof: u32, axis: vec3<f32>, offset: vec3<f32>) {
    cdof[dof * 2u + 0u] = vec4<f32>(axis, 0.0);
    cdof[dof * 2u + 1u] = vec4<f32>(cross(axis, offset), 0.0);
}

// A translation along `axis`, `[0; axis]`, written at `cdof[dof]`.
fn write_translation_dof(dof: u32, axis: vec3<f32>) {
    cdof[dof * 2u + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    cdof[dof * 2u + 1u] = vec4<f32>(axis, 0.0);
}

// Each body's inertia and its joints' motion axes about its tree's subtree
// centre of mass `p`, after fk_subtree_normalize, as the CPU's `mj_com_pos`
// (MuJoCo `mj_comPos`).
//
// cinert is MuJoCo's 10-vector `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz, m·dx, m·dy,
// m·dz, m]` (`mju_inertCom`), `d` = xipos − p, in 3 × vec4: (Ixx, Iyy, Izz,
// Ixy), (Ixz, Iyz, m·dx, m·dy), (m·dz, m, 0, 0); the world's is 0. A dof's
// cdof is `[angular; linear]`: a hinge's axis, a ball's or a free joint's
// rotation the columns of the body's rotation, each about the joint's
// anchor; a slide's axis, a free joint's translation the world's axes.
@compute @workgroup_size(64)
fn fk_com_pos(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }

    let env_off = env_id * params.nbody;
    let ci_base = (env_off + body_id) * 3u;
    if (body_id == 0u) {
        body_cinert[ci_base + 0u] = vec4<f32>(0.0);
        body_cinert[ci_base + 1u] = vec4<f32>(0.0);
        body_cinert[ci_base + 2u] = vec4<f32>(0.0);
        return;
    }

    let body = bodies[body_id];
    let p = subtree_com[env_off + body.rootid].xyz;
    let quat = body_xquat[env_off + body_id];

    // ── cinert ────────────────────────────────────────────────────
    // I = ximat · diag(inertia) · ximatᵀ about the centre of mass (ximat is
    // column-major, ximat[col][row]: element (r, c) = Σ_k ximat[k][r] ·
    // inertia[k] · ximat[k][c]), then moved to p.
    let ximat = quat_to_mat3(quat_mul(quat, body.iquat));
    let inertia = body.inertia.xyz;
    let mass = body.inertia.w;
    let d = body_xipos[env_off + body_id].xyz - p;
    let ixx = ximat[0][0] * inertia.x * ximat[0][0]
            + ximat[1][0] * inertia.y * ximat[1][0]
            + ximat[2][0] * inertia.z * ximat[2][0]
            + mass * (d.y * d.y + d.z * d.z);
    let iyy = ximat[0][1] * inertia.x * ximat[0][1]
            + ximat[1][1] * inertia.y * ximat[1][1]
            + ximat[2][1] * inertia.z * ximat[2][1]
            + mass * (d.x * d.x + d.z * d.z);
    let izz = ximat[0][2] * inertia.x * ximat[0][2]
            + ximat[1][2] * inertia.y * ximat[1][2]
            + ximat[2][2] * inertia.z * ximat[2][2]
            + mass * (d.x * d.x + d.y * d.y);
    let ixy = ximat[0][0] * inertia.x * ximat[0][1]
            + ximat[1][0] * inertia.y * ximat[1][1]
            + ximat[2][0] * inertia.z * ximat[2][1]
            - mass * d.x * d.y;
    let ixz = ximat[0][0] * inertia.x * ximat[0][2]
            + ximat[1][0] * inertia.y * ximat[1][2]
            + ximat[2][0] * inertia.z * ximat[2][2]
            - mass * d.x * d.z;
    let iyz = ximat[0][1] * inertia.x * ximat[0][2]
            + ximat[1][1] * inertia.y * ximat[1][2]
            + ximat[2][1] * inertia.z * ximat[2][2]
            - mass * d.y * d.z;
    let h = mass * d;
    body_cinert[ci_base + 0u] = vec4<f32>(ixx, iyy, izz, ixy);
    body_cinert[ci_base + 1u] = vec4<f32>(ixz, iyz, h.x, h.y);
    body_cinert[ci_base + 2u] = vec4<f32>(h.z, mass, 0.0, 0.0);

    // ── cdof ──────────────────────────────────────────────────────
    let rot = quat_to_mat3(quat);
    let env_dof_off = env_id * params.nv;
    for (var ji = 0u; ji < body.jnt_num; ji++) {
        let j = body.jnt_adr + ji;
        let jnt = joints[j];
        let xj = env_id * params.njnt + j;
        let dof = env_dof_off + jnt.dof_adr;
        let offset = p - jnt_xanchor[xj].xyz;

        if (jnt.jtype == JNT_FREE) {
            write_translation_dof(dof + 0u, vec3<f32>(1.0, 0.0, 0.0));
            write_translation_dof(dof + 1u, vec3<f32>(0.0, 1.0, 0.0));
            write_translation_dof(dof + 2u, vec3<f32>(0.0, 0.0, 1.0));
            for (var k = 0u; k < 3u; k++) {
                write_rotation_dof(dof + 3u + k, rot[k], offset);
            }
        } else if (jnt.jtype == JNT_BALL) {
            for (var k = 0u; k < 3u; k++) {
                write_rotation_dof(dof + k, rot[k], offset);
            }
        } else if (jnt.jtype == JNT_SLIDE) {
            write_translation_dof(dof, jnt_xaxis[xj].xyz);
        } else if (jnt.jtype == JNT_HINGE) {
            write_rotation_dof(dof, jnt_xaxis[xj].xyz, offset);
        }
    }
}
