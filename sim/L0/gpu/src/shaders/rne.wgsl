// rne.wgsl — Recursive Newton-Euler bias forces.
//
// 4 entry points dispatched in sequence:
//   1. rne_gravity     — per joint: gravity projection to qfrc_bias
//   2. rne_forward      — per depth level (root→leaves): bias accelerations
//   3. rne_backward     — per depth level (leaves→root): bias forces + CAS accumulate
//   4. rne_project      — per DOF: project cfrc to joint space → qfrc_bias
//
// Computes qfrc_bias = gravity + Coriolis + gyroscopic bias forces.

// ── Constants ────────────────────────────────────────────────────────

const JNT_FREE: u32 = 0u;
const JNT_BALL: u32 = 1u;
const JNT_HINGE: u32 = 2u;
const JNT_SLIDE: u32 = 3u;

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
    _pad: u32,
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

struct DofModel {
    body_id: u32,
    parent: u32,
    armature: f32,
    damping: f32,
};

// ── Bindings ─────────────────────────────────────────────────────────

// Group 0: physics params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> joints: array<JointModel>;
@group(1) @binding(2) var<storage, read> dofs: array<DofModel>;

// Group 2: FK / velocity FK outputs (read-only). Bind only what RNE consumes —
// gravity projects onto cdof (no quat/qpos), matching the per-shader minimal
// binding convention of crba.wgsl / velocity_fk.wgsl.
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read> body_cvel: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read> subtree_mass: array<f32>;
@group(2) @binding(5) var<storage, read> subtree_com: array<vec4<f32>>;
@group(2) @binding(6) var<storage, read> qvel: array<f32>;

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

// Implicit 6×6 spatial inertia × 6-vector multiply using cinert format.
// cinert = (m, h, I_COM). Converts to I_ref inline (parallel axis).
// spatial_vec = (angular, linear).
// Returns (result_angular, result_linear).
fn cinert_mul_spatial(
    ci0: vec4<f32>, ci1: vec4<f32>, ci2: vec4<f32>,
    s_ang: vec3<f32>, s_lin: vec3<f32>
) -> array<vec3<f32>, 2> {
    let m = ci0.x;
    let h = ci0.yzw;
    let mh = m * h;

    // I_COM upper triangle
    let I00 = ci1.x; let I01 = ci1.y; let I02 = ci1.z;
    let I11 = ci1.w; let I12 = ci2.x; let I22 = ci2.y;

    // Parallel axis: I_ref = I_COM + m*(|h|²·I₃ − h⊗h)
    let hh = dot(h, h);
    let R00 = I00 + m * (hh - h.x * h.x);
    let R01 = I01 + m * (0.0 - h.x * h.y);
    let R02 = I02 + m * (0.0 - h.x * h.z);
    let R11 = I11 + m * (hh - h.y * h.y);
    let R12 = I12 + m * (0.0 - h.y * h.z);
    let R22 = I22 + m * (hh - h.z * h.z);

    // I_ref * angular (symmetric 3×3 matmul)
    let Iw = vec3<f32>(
        R00 * s_ang.x + R01 * s_ang.y + R02 * s_ang.z,
        R01 * s_ang.x + R11 * s_ang.y + R12 * s_ang.z,
        R02 * s_ang.x + R12 * s_ang.y + R22 * s_ang.z,
    );

    // buf_angular = I_ref * ω + cross(mh, v)
    let buf_ang = Iw + cross(mh, s_lin);
    // buf_linear = cross(ω, mh) + m * v
    let buf_lin = cross(s_ang, mh) + m * s_lin;

    return array<vec3<f32>, 2>(buf_ang, buf_lin);
}

// ── Entry point 1: Gravity (per-joint parallel) ──────────────────────

@compute @workgroup_size(64)
fn rne_gravity(@builtin(global_invocation_id) gid: vec3<u32>) {
    let jnt_id = gid.x;
    let env_id = gid.y;
    if jnt_id >= params.njnt || env_id >= params.n_env { return; }

    let jnt = joints[jnt_id];
    let body_id = jnt.body_id;
    let env_body_off = env_id * params.nbody;
    let env_dof_off = env_id * params.nv;

    let sm = subtree_mass[env_body_off + body_id];
    let sc = subtree_com[env_body_off + body_id].xyz;
    let grav = params.gravity.xyz;

    // Subtree gravitational wrench, expressed as a spatial force at this body's
    // ORIGIN: force gf = -subtree_mass·g (opposes gravity), torque about origin
    // = (subtree_com − xpos[body]) × gf.
    let gf = -sm * grav;
    let bxpos = body_xpos[env_body_off + body_id].xyz;
    let wrench_ang = cross(sc - bxpos, gf);
    let wrench_lin = gf;

    // Project onto every DOF's motion subspace: qfrc_bias[d] = cdof[d] · wrench.
    // Reusing the (partial-frame-correct) cdof — rather than re-deriving each
    // joint's world axis/anchor from the FINAL body quat — keeps the gravity
    // projection consistent with the motion subspace for a multi-joint body
    // (where an earlier joint's axis must not be over-rotated by later joints),
    // with one projection path for every joint type. See
    // project-multi-joint-partial-frame. NOTE: this references the wrench at the
    // body origin (cdof's reference point); for a ball with jnt_pos ≠ 0 that
    // differs from the CPU `mj_rne` ball anchor, but real ball joints sit at the
    // body origin so no fixture exercises the divergence.
    let dof_adr = jnt.dof_adr;
    var ndof = 1u;
    switch jnt.jtype {
        case 0u: { ndof = 6u; }  // JNT_FREE
        case 1u: { ndof = 3u; }  // JNT_BALL
        default: { ndof = 1u; }  // JNT_HINGE / JNT_SLIDE
    }
    for (var k = 0u; k < ndof; k++) {
        let d = env_dof_off + dof_adr + k;
        let cda = cdof[d * 2u + 0u].xyz;
        let cdl = cdof[d * 2u + 1u].xyz;
        qfrc_bias[d] = dot(cda, wrench_ang) + dot(cdl, wrench_lin);
    }
}

// ── Entry point 2: Forward scan — bias accelerations ────────────────

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

    // World body: zero bias acceleration
    if body_id == 0u {
        body_cacc[cacc_base + 0u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        body_cacc[cacc_base + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }

    let parent = body.parent;
    let parent_base = (env_body_off + parent) * 2u;

    // Transport parent bias acceleration to this body's origin
    let pa_ang = body_cacc[parent_base + 0u].xyz;
    let pa_lin = body_cacc[parent_base + 1u].xyz;
    let r = body_xpos[env_body_off + body_id].xyz
          - body_xpos[env_body_off + parent].xyz;

    var a_ang = pa_ang;
    var a_lin = pa_lin + cross(pa_ang, r);

    // Parent velocity, TRANSPORTED to this body's origin. cvel is referenced at
    // each body's xpos, so the velocity-product term c[i] = v[i] ×_m (S·qdot) =
    // X_b(v[parent]) ×_m (S·qdot) must use the transported parent velocity (angular
    // unchanged, linear += ω_parent × r) — the raw parent value at the parent origin
    // drops the [0; ω_parent×r] lever and corrupts non-parallel ≥3-link Coriolis.
    // (Mirrors the CPU `transport_motion_spatial` in dynamics/spatial.rs; r is the
    // same parent→child offset used for the bias-acceleration transport above.)
    let vp_ang = body_cvel[(env_body_off + parent) * 2u + 0u].xyz;
    let vp_lin = body_cvel[(env_body_off + parent) * 2u + 1u].xyz + cross(vp_ang, r);

    // Velocity-product acceleration from each DOF on this body
    // c[i] = X_b(v[parent]) ×_m (S[i] @ qdot[i])
    let dof_start = body.dof_adr;
    let dof_count = body.dof_num;
    for (var d = 0u; d < dof_count; d++) {
        let dof_idx = env_dof_off + dof_start + d;
        let qv = qvel[dof_idx];
        let vj_ang = cdof[dof_idx * 2u + 0u].xyz * qv;
        let vj_lin = cdof[dof_idx * 2u + 1u].xyz * qv;

        // spatial_cross_motion(vt, v_joint):
        //   result.angular = ω_parent × vj_ang
        //   result.linear  = ω_parent × vj_lin + vt_lin × vj_ang
        a_ang += cross(vp_ang, vj_ang);
        a_lin += cross(vp_ang, vj_lin) + cross(vp_lin, vj_ang);
    }

    // Free joint correction: subtract ω × v_linear
    // Accounts for spatial vs. world-frame acceleration convention.
    // A free body has dof_num == 6.
    if dof_count == 6u {
        let body_cvel_base = (env_body_off + body_id) * 2u;
        let omega = body_cvel[body_cvel_base + 0u].xyz;
        let v_lin_free = vec3<f32>(
            qvel[env_dof_off + dof_start + 0u],
            qvel[env_dof_off + dof_start + 1u],
            qvel[env_dof_off + dof_start + 2u],
        );
        a_lin -= cross(omega, v_lin_free);
    }

    body_cacc[cacc_base + 0u] = vec4<f32>(a_ang, 0.0);
    body_cacc[cacc_base + 1u] = vec4<f32>(a_lin, 0.0);
}

// ── Entry point 3a: Compute own bias force per body (all parallel) ───
//
// CPU does this in a single forward loop (rne.rs:252-268), computing
// cfrc[body] = I·a_bias + v ×* (I·v) for each body independently.
// Must run BEFORE the backward accumulation scan.

@compute @workgroup_size(64)
fn rne_cfrc_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id = gid.y;
    if body_id >= params.nbody || env_id >= params.n_env { return; }
    if body_id == 0u { return; }

    let env_body_off = env_id * params.nbody;
    let cvel_base = (env_body_off + body_id) * 2u;

    // Load body velocity and bias acceleration
    let v_ang = body_cvel[cvel_base + 0u].xyz;
    let v_lin = body_cvel[cvel_base + 1u].xyz;
    let a_ang = body_cacc[cvel_base + 0u].xyz;
    let a_lin = body_cacc[cvel_base + 1u].xyz;

    // Load cinert for this body (single-body, NOT composite)
    let ci_base = (env_body_off + body_id) * 3u;
    let ci0 = body_cinert[ci_base + 0u];
    let ci1 = body_cinert[ci_base + 1u];
    let ci2 = body_cinert[ci_base + 2u];

    // I × a_bias (inertial force from bias acceleration)
    let Ia = cinert_mul_spatial(ci0, ci1, ci2, a_ang, a_lin);

    // I × v (momentum)
    let Iv = cinert_mul_spatial(ci0, ci1, ci2, v_ang, v_lin);

    // v ×* (I × v) — spatial force cross product (gyroscopic)
    // ×* : result_ang = ω × f_ang + v_lin × f_lin
    //      result_lin = ω × f_lin
    let gyro_ang = cross(v_ang, Iv[0]) + cross(v_lin, Iv[1]);
    let gyro_lin = cross(v_ang, Iv[1]);

    // cfrc = I·a + v ×* (I·v)
    let cfrc_ang = Ia[0] + gyro_ang;
    let cfrc_lin = Ia[1] + gyro_lin;

    // Store via atomicStore (body_cfrc is zeroed before this dispatch,
    // so atomicStore is safe — no prior CAS additions to preserve)
    let cfrc_base = (env_body_off + body_id) * 8u;
    atomicStore(&body_cfrc[cfrc_base + 0u], bitcast<u32>(cfrc_ang.x));
    atomicStore(&body_cfrc[cfrc_base + 1u], bitcast<u32>(cfrc_ang.y));
    atomicStore(&body_cfrc[cfrc_base + 2u], bitcast<u32>(cfrc_ang.z));
    atomicStore(&body_cfrc[cfrc_base + 3u], bitcast<u32>(0.0));
    atomicStore(&body_cfrc[cfrc_base + 4u], bitcast<u32>(cfrc_lin.x));
    atomicStore(&body_cfrc[cfrc_base + 5u], bitcast<u32>(cfrc_lin.y));
    atomicStore(&body_cfrc[cfrc_base + 6u], bitcast<u32>(cfrc_lin.z));
    atomicStore(&body_cfrc[cfrc_base + 7u], bitcast<u32>(0.0));
}

// ── Entry point 3b: Backward accumulation (leaves → root) ───────────
//
// CPU: rne.rs:271-278 — for body_id in (1..nbody).rev(): cfrc[parent] += cfrc[child]
// GPU: one dispatch per depth level (max_depth → 1), CAS-atomic add.

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

    // Load this body's cfrc
    let cfrc_base = (env_body_off + body_id) * 8u;
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

    // CAS-atomic-add into parent with X_bᵀ spatial-force transport to the parent
    // origin: force unchanged, torque gains the (xpos[child]−xpos[parent])×f lever.
    // cfrc is a spatial force at each body's own origin, so a plain add across
    // bodies at different origins drops this lever and corrupts ancestor-DOF
    // Coriolis on a multi-link chain (matches CPU rne.rs after PR #350).
    let r = body_xpos[env_body_off + body_id].xyz
          - body_xpos[env_body_off + body.parent].xyz;
    let lever = cross(r, cfrc_lin);
    let parent_base = (env_body_off + body.parent) * 8u;
    atomic_add_f32_cfrc(parent_base + 0u, cfrc_ang.x + lever.x);
    atomic_add_f32_cfrc(parent_base + 1u, cfrc_ang.y + lever.y);
    atomic_add_f32_cfrc(parent_base + 2u, cfrc_ang.z + lever.z);
    atomic_add_f32_cfrc(parent_base + 4u, cfrc_lin.x);
    atomic_add_f32_cfrc(parent_base + 5u, cfrc_lin.y);
    atomic_add_f32_cfrc(parent_base + 6u, cfrc_lin.z);
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
    let body_id = dof.body_id;

    // Load cfrc for this body
    let cfrc_base = (env_body_off + body_id) * 8u;
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

    // cdof[d] for this DOF
    let cdof_ang = cdof[(env_dof_off + dof_id) * 2u + 0u].xyz;
    let cdof_lin = cdof[(env_dof_off + dof_id) * 2u + 1u].xyz;

    // qfrc_bias[d] += dot(cdof[d], cfrc[body])
    // (adds Coriolis/gyroscopic to gravity already written by rne_gravity)
    qfrc_bias[env_dof_off + dof_id] += dot(cdof_ang, cfrc_ang) + dot(cdof_lin, cfrc_lin);
}
