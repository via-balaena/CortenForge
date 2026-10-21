// velocity_fk.wgsl — Velocity forward kinematics.
//
// 1 entry point dispatched per depth level (root → leaves):
//   velocity_fk_forward — each body's spatial velocity (cvel) and each dof's
//                         axis rate (cdof_dot), as the CPU's `mj_fwd_velocity`
//                         (MuJoCo `mj_comVel`)
//
// cvel layout: 2×vec4 per body, about its tree's subtree centre of mass (the
// reference point of cdof):
//   cvel[body*2 + 0] = (ω_x, ω_y, ω_z, 0)  — angular velocity
//   cvel[body*2 + 1] = (v_x, v_y, v_z, 0)  — linear velocity
// cdof_dot layout: 2×vec4 per dof, [angular; linear] (as cdof).

// ── Constants ─────────────────────────────────────────────────────────

const JNT_FREE: u32  = 0u;
const JNT_BALL: u32  = 1u;

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

struct JointModel {
    jtype: u32,
    qpos_adr: u32,
    dof_adr: u32,
    body_id: u32,
    axis: vec4<f32>,
    pos: vec4<f32>,
};

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: per-dispatch params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> joints: array<JointModel>;

// Group 2: state
@group(2) @binding(0) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> qvel: array<f32>;
@group(2) @binding(2) var<storage, read_write> body_cvel: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read_write> cdof_dot: array<vec4<f32>>;

// ── Helpers ───────────────────────────────────────────────────────────

// The motion cross product `v × s` of the velocity (`w`, `v`) and dof `dof`'s
// axis, written as its cdof_dot (`spatial_cross_motion`, MuJoCo's
// `mju_crossMotion`).
fn write_cdof_dot(dof: u32, w: vec3<f32>, v: vec3<f32>) {
    let s_ang = cdof[dof * 2u + 0u].xyz;
    let s_lin = cdof[dof * 2u + 1u].xyz;
    cdof_dot[dof * 2u + 0u] = vec4<f32>(cross(w, s_ang), 0.0);
    cdof_dot[dof * 2u + 1u] = vec4<f32>(cross(w, s_lin) + cross(v, s_ang), 0.0);
}

// ── Entry point: Velocity FK forward scan ────────────────────────────

// A body's velocity is its parent's plus each of its joints' `cdof · qvel`,
// in the joints' order; a dof's cdof_dot is the velocity before its joint
// crossed with its axis, a ball's three sharing it, a free joint's rotation
// taking the velocity after its translation, whose own rates are 0.
@compute @workgroup_size(64)
fn velocity_fk_forward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let body = bodies[body_id];
    if body.depth != params.current_depth { return; }

    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;
    let cvel_base = (env_body_off + body_id) * 2u;

    // World body: zero velocity
    if body_id == 0u {
        body_cvel[cvel_base + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        body_cvel[cvel_base + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // The parent's velocity: within a tree every body's is about the same
    // point, and a tree's root takes the world's, 0.
    let parent_base = (env_body_off + body.parent) * 2u;
    var w = body_cvel[parent_base + 0u].xyz;
    var v = body_cvel[parent_base + 1u].xyz;

    for (var ji = 0u; ji < body.jnt_num; ji++) {
        let jnt = joints[body.jnt_adr + ji];
        let first = env_dof_off + jnt.dof_adr;
        // The dofs whose rates cross the velocity before them.
        var start = first;
        var count = 1u;
        if jnt.jtype == JNT_FREE {
            // The translation: no rate, its velocity added first.
            for (var k = 0u; k < 3u; k++) {
                let d = first + k;
                cdof_dot[d * 2u + 0u] = vec4<f32>(0.0);
                cdof_dot[d * 2u + 1u] = vec4<f32>(0.0);
                w += cdof[d * 2u + 0u].xyz * qvel[d];
                v += cdof[d * 2u + 1u].xyz * qvel[d];
            }
            start = first + 3u;
            count = 3u;
        } else if jnt.jtype == JNT_BALL {
            count = 3u;
        }
        for (var k = 0u; k < count; k++) {
            write_cdof_dot(start + k, w, v);
        }
        for (var k = 0u; k < count; k++) {
            let d = start + k;
            w += cdof[d * 2u + 0u].xyz * qvel[d];
            v += cdof[d * 2u + 1u].xyz * qvel[d];
        }
    }

    body_cvel[cvel_base + 0u] = vec4<f32>(w, 0.0);
    body_cvel[cvel_base + 1u] = vec4<f32>(v, 0.0);
}
