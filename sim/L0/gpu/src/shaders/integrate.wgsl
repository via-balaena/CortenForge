// integrate.wgsl — Semi-implicit Euler integration.
//
// 1 entry point:
//   integrate_euler — per joint: qvel += dt*qacc, qpos += dt*f(qvel)
//                     with quaternion exponential map for ball/free joints.

// ── Common math ─────────────────────────────────────────────────────

fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    // Quaternion multiply in (x,y,z,w) layout.
    return vec4<f32>(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    );
}

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

struct JointModel {
    jtype: u32,
    qpos_adr: u32,
    dof_adr: u32,
    body_id: u32,
    axis: vec4<f32>,
    pos: vec4<f32>,
};

// ── Bindings ─────────────────────────────────────────────────────────

// Group 0: physics params
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> joints: array<JointModel>;

// Group 2: state (read-write)
@group(2) @binding(0) var<storage, read_write> qpos: array<f32>;
@group(2) @binding(1) var<storage, read_write> qvel: array<f32>;
@group(2) @binding(2) var<storage, read> qacc: array<f32>;

// ── Entry point: Semi-implicit Euler integration ────────────────────

@compute @workgroup_size(64)
fn integrate_euler(@builtin(global_invocation_id) gid: vec3<u32>) {
    let jnt_id = gid.x;
    let env_id = gid.y;
    if jnt_id >= params.njnt || env_id >= params.n_env { return; }

    let jnt = joints[jnt_id];
    let dt = params.timestep;
    let env_dof_off = env_id * params.nv;
    let env_nq_off = env_id * params.nq;

    let d = env_dof_off + jnt.dof_adr;
    let q = env_nq_off + jnt.qpos_adr;

    switch jnt.jtype {
        case 2u, 3u: {  // JNT_HINGE, JNT_SLIDE
            // Semi-implicit Euler: update vel first, then pos
            qvel[d] += dt * qacc[d];
            qpos[q] += dt * qvel[d];
        }
        case 1u: {  // JNT_BALL
            // Velocity update: 3 DOFs
            qvel[d + 0u] += dt * qacc[d + 0u];
            qvel[d + 1u] += dt * qacc[d + 1u];
            qvel[d + 2u] += dt * qacc[d + 2u];

            // Quaternion integration via exponential map
            let omega = vec3<f32>(qvel[d], qvel[d + 1u], qvel[d + 2u]);
            let omega_norm = length(omega);
            let angle = omega_norm * dt;

            if angle > 1e-10 && omega_norm > 1e-10 {
                let axis = omega / omega_norm;
                let half_angle = angle * 0.5;
                let s = sin(half_angle);
                let c = cos(half_angle);
                // dq in (x,y,z,w) GPU layout
                let dq = vec4<f32>(s * axis.x, s * axis.y, s * axis.z, c);

                // Read quaternion from qpos (w,x,y,z) → swizzle to (x,y,z,w)
                let q_old = vec4<f32>(qpos[q + 1u], qpos[q + 2u], qpos[q + 3u], qpos[q]);

                // q_new = q_old * dq (right multiply — local frame)
                let q_new = quat_mul(q_old, dq);

                // Normalize
                let n = length(q_new);
                let q_norm = q_new / max(n, 1e-10);

                // Write back (x,y,z,w) → (w,x,y,z)
                qpos[q + 0u] = q_norm.w;
                qpos[q + 1u] = q_norm.x;
                qpos[q + 2u] = q_norm.y;
                qpos[q + 3u] = q_norm.z;
            }
        }
        case 0u: {  // JNT_FREE
            // Velocity update: 6 DOFs (linear 0-2, angular 3-5)
            qvel[d + 0u] += dt * qacc[d + 0u];
            qvel[d + 1u] += dt * qacc[d + 1u];
            qvel[d + 2u] += dt * qacc[d + 2u];
            qvel[d + 3u] += dt * qacc[d + 3u];
            qvel[d + 4u] += dt * qacc[d + 4u];
            qvel[d + 5u] += dt * qacc[d + 5u];

            // Position: linear (qpos 0-2 from qvel 0-2)
            qpos[q + 0u] += dt * qvel[d + 0u];
            qpos[q + 1u] += dt * qvel[d + 1u];
            qpos[q + 2u] += dt * qvel[d + 2u];

            // Quaternion: angular (qpos 3-6 from qvel 3-5)
            let omega = vec3<f32>(qvel[d + 3u], qvel[d + 4u], qvel[d + 5u]);
            let omega_norm = length(omega);
            let angle = omega_norm * dt;

            if angle > 1e-10 && omega_norm > 1e-10 {
                let axis = omega / omega_norm;
                let half_angle = angle * 0.5;
                let s = sin(half_angle);
                let c = cos(half_angle);
                let dq = vec4<f32>(s * axis.x, s * axis.y, s * axis.z, c);

                // qpos[q+3..q+6] stores (w,x,y,z) → swizzle to (x,y,z,w)
                let qa = q + 3u;
                let q_old = vec4<f32>(qpos[qa + 1u], qpos[qa + 2u], qpos[qa + 3u], qpos[qa]);

                let q_new = quat_mul(q_old, dq);
                let n = length(q_new);
                let q_norm = q_new / max(n, 1e-10);

                qpos[qa + 0u] = q_norm.w;
                qpos[qa + 1u] = q_norm.x;
                qpos[qa + 2u] = q_norm.y;
                qpos[qa + 3u] = q_norm.z;
            }
        }
        default: {}
    }
}
