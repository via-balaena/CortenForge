// rne.wgsl — Recursive Newton-Euler bias forces.
//
// 4 entry points dispatched in sequence:
//   1. rne_forward    — per depth level (root→leaves): bias accelerations
//   2. rne_cfrc_init  — per body: each body's own bias force
//   3. rne_backward   — per depth level (leaves→root): CAS accumulate
//   4. rne_project    — per DOF: project cfrc to joint space → qfrc_bias
//
// qfrc_bias = gravity + Coriolis + centrifugal + gyroscopic bias forces, as
// the CPU's `mj_rne` (MuJoCo's with flg_acc 0). Every spatial quantity is
// about the body's tree's subtree centre of mass.

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

// ── Bindings ─────────────────────────────────────────────────────────

// Group 0: physics params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> dofs: array<DofModel>;

// Group 2: FK / velocity FK outputs (read-only)
@group(2) @binding(0) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> cdof_dot: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read> body_cvel: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read> qvel: array<f32>;

// Group 3: RNE outputs (read-write)
@group(3) @binding(0) var<storage, read_write> body_cacc: array<vec4<f32>>;
@group(3) @binding(1) var<storage, read_write> body_cfrc: array<atomic<u32>>;
@group(3) @binding(2) var<storage, read_write> qfrc_bias: array<f32>;

// ── CAS atomic f32 add for body_cfrc ─────────────────────────────────
// WGSL disallows storage pointers as function args (LIMITATIONS.md §8).

fn atomic_add_f32_cfrc(idx: u32, val: f32) {
    var old = atomicLoad(&body_cfrc[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&body_cfrc[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}

// ── Helpers ──────────────────────────────────────────────────────────

// Body `idx`'s cinert (the 10-vector `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz, m·dx,
// m·dy, m·dz, m]` in 3 vec4) times a motion vector `[ω; v]`, the momentum
// `[I·ω + h × v; m·v − h × ω]` (`h` = m·d), as the CPU's `mju_mul_inert_vec`
// (MuJoCo `mju_mulInertVec`). Returns (angular, linear).
fn cinert_mul(idx: u32, w: vec3<f32>, v: vec3<f32>) -> array<vec3<f32>, 2> {
    let c0 = body_cinert[idx * 3u + 0u];
    let c1 = body_cinert[idx * 3u + 1u];
    let c2 = body_cinert[idx * 3u + 2u];
    let h = vec3<f32>(c1.z, c1.w, c2.x);
    let m = c2.y;
    let iw = vec3<f32>(
        c0.x * w.x + c0.w * w.y + c1.x * w.z,
        c0.w * w.x + c0.y * w.y + c1.y * w.z,
        c1.x * w.x + c1.y * w.y + c0.z * w.z,
    );
    return array<vec3<f32>, 2>(iw + cross(h, v), m * v - cross(h, w));
}

// ── Entry point 1: Forward scan — bias accelerations ────────────────

// The world's acceleration is `[0; −gravity]`, so every body's carries
// gravity; a body's is its parent's plus `cdof_dot · qvel` over its dofs.
@compute @workgroup_size(64)
fn rne_forward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let body = bodies[body_id];
    if body.depth != params.current_depth { return; }

    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;
    let cacc_base = (env_body_off + body_id) * 2u;

    if body_id == 0u {
        body_cacc[cacc_base + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        body_cacc[cacc_base + 1u] = vec4<f32>(-params.gravity.xyz, 0.0);
        return;
    }

    let parent_base = (env_body_off + body.parent) * 2u;
    var a_ang = body_cacc[parent_base + 0u].xyz;
    var a_lin = body_cacc[parent_base + 1u].xyz;
    for (var k = 0u; k < body.dof_num; k++) {
        let d = env_dof_off + body.dof_adr + k;
        a_ang += cdof_dot[d * 2u + 0u].xyz * qvel[d];
        a_lin += cdof_dot[d * 2u + 1u].xyz * qvel[d];
    }

    body_cacc[cacc_base + 0u] = vec4<f32>(a_ang, 0.0);
    body_cacc[cacc_base + 1u] = vec4<f32>(a_lin, 0.0);
}

// ── Entry point 2: Each body's own bias force (all parallel) ────────
//
// cfrc = cinert·cacc + cvel ×* (cinert·cvel) for each body independently.
// Must run AFTER the forward scan, BEFORE the backward accumulation.

@compute @workgroup_size(64)
fn rne_cfrc_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }
    if body_id == 0u { return; }

    let idx = env_id * params.nbody + body_id;
    let v_ang = body_cvel[idx * 2u + 0u].xyz;
    let v_lin = body_cvel[idx * 2u + 1u].xyz;
    let a_ang = body_cacc[idx * 2u + 0u].xyz;
    let a_lin = body_cacc[idx * 2u + 1u].xyz;

    let ia = cinert_mul(idx, a_ang, a_lin);
    let iv = cinert_mul(idx, v_ang, v_lin);

    // v ×* (I·v), the force cross product:
    //   angular = ω × f_ang + v_lin × f_lin, linear = ω × f_lin
    let cfrc_ang = ia[0] + cross(v_ang, iv[0]) + cross(v_lin, iv[1]);
    let cfrc_lin = ia[1] + cross(v_ang, iv[1]);

    // Store via atomicStore (body_cfrc is zeroed before this dispatch,
    // so atomicStore is safe — no prior CAS additions to preserve)
    let cfrc_base = idx * 8u;
    atomicStore(&body_cfrc[cfrc_base + 0u], bitcast<u32>(cfrc_ang.x));
    atomicStore(&body_cfrc[cfrc_base + 1u], bitcast<u32>(cfrc_ang.y));
    atomicStore(&body_cfrc[cfrc_base + 2u], bitcast<u32>(cfrc_ang.z));
    atomicStore(&body_cfrc[cfrc_base + 3u], bitcast<u32>(0.0));
    atomicStore(&body_cfrc[cfrc_base + 4u], bitcast<u32>(cfrc_lin.x));
    atomicStore(&body_cfrc[cfrc_base + 5u], bitcast<u32>(cfrc_lin.y));
    atomicStore(&body_cfrc[cfrc_base + 6u], bitcast<u32>(cfrc_lin.z));
    atomicStore(&body_cfrc[cfrc_base + 7u], bitcast<u32>(0.0));
}

// ── Entry point 3: Backward accumulation (leaves → root) ───────────
//
// Each body's force added into its parent's (not the world's), one dispatch
// per depth level (max_depth → 1), CAS-atomic: every body of a tree is about
// the same point, so the sum moves nothing.

@compute @workgroup_size(64)
fn rne_backward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }

    let body = bodies[body_id];
    if body.depth != params.current_depth { return; }
    if body_id == 0u { return; }
    if body.parent == 0u { return; }  // world body has no DOFs

    let env_body_off = env_id * params.nbody;
    let cfrc_base = (env_body_off + body_id) * 8u;
    let parent_base = (env_body_off + body.parent) * 8u;
    // The torque (slots 0–2) and the force (4–6); slots 3 and 7 are padding.
    for (var k = 0u; k < 3u; k++) {
        atomic_add_f32_cfrc(parent_base + k, bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + k])));
        atomic_add_f32_cfrc(
            parent_base + 4u + k,
            bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 4u + k])),
        );
    }
}

// ── Entry point 4: Project to joint space ───────────────────────────

@compute @workgroup_size(64)
fn rne_project(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dof_id = gid.x;
    let env_id = gid.y;
    if dof_id >= params.nv || env_id >= params.n_env { return; }

    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;

    let dof = dofs[dof_id];
    let cfrc_base = (env_body_off + dof.body_id) * 8u;
    let cfrc_ang = vec3<f32>(
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 0u])),
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 1u])),
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 2u])),
    );
    let cfrc_lin = vec3<f32>(
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 4u])),
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 5u])),
        bitcast<f32>(atomicLoad(&body_cfrc[cfrc_base + 6u])),
    );

    // qfrc_bias[d] = cdof[d] · cfrc[body]
    let d = env_dof_off + dof_id;
    qfrc_bias[d] = dot(cdof[d * 2u + 0u].xyz, cfrc_ang) + dot(cdof[d * 2u + 1u].xyz, cfrc_lin);
}
