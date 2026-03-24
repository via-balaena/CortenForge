// velocity_fk.wgsl — Velocity forward kinematics.
//
// 1 entry point dispatched per depth level (root → leaves):
//   velocity_fk_forward — compute body spatial velocities (cvel) from qvel
//
// cvel layout: 2×vec4 per body
//   cvel[body*2 + 0] = (ω_x, ω_y, ω_z, 0)  — angular velocity
//   cvel[body*2 + 1] = (v_x, v_y, v_z, 0)  — linear velocity at body origin
//
// Uses cdof (from FK) to avoid joint-type branching:
//   cvel[body] += cdof[d] × qvel[d]  for each DOF d on the body.

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

// ── Bindings ──────────────────────────────────────────────────────────

// Group 0: per-dispatch params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;

// Group 2: state
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> qvel: array<f32>;
@group(2) @binding(3) var<storage, read_write> body_cvel: array<vec4<f32>>;

// ── Entry point: Velocity FK forward scan ────────────────────────────

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

    // Transport parent velocity to this body's origin
    let parent = body.parent;
    let parent_base = (env_body_off + parent) * 2u;
    let p_ang = body_cvel[parent_base + 0u].xyz;
    let p_lin = body_cvel[parent_base + 1u].xyz;

    // Lever arm: r = xpos[body] - xpos[parent]
    let r = body_xpos[env_body_off + body_id].xyz
          - body_xpos[env_body_off + parent].xyz;

    // Transport: ω_child = ω_parent, v_child = v_parent + ω_parent × r
    var omega = p_ang;
    var v_lin = p_lin + cross(p_ang, r);

    // Add joint velocity contributions using cdof
    let dof_start = body.dof_adr;
    let dof_count = body.dof_num;
    for (var d = 0u; d < dof_count; d++) {
        let dof_idx = env_dof_off + dof_start + d;
        let qv = qvel[dof_idx];
        omega += cdof[dof_idx * 2u + 0u].xyz * qv;
        v_lin += cdof[dof_idx * 2u + 1u].xyz * qv;
    }

    body_cvel[cvel_base + 0u] = vec4<f32>(omega, 0.0);
    body_cvel[cvel_base + 1u] = vec4<f32>(v_lin, 0.0);
}
