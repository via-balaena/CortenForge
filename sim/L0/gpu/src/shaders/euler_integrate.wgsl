// euler_integrate.wgsl
// Phase 10a: Euler velocity integration on GPU.
// qvel[i] += qacc[i] * timestep for each DOF in each environment.

struct Params {
    num_envs: u32,
    nv: u32,
    timestep: f32,
    _pad: u32,
}

// Pipeline-overridable constant: set to next_power_of_2(nv) at
// pipeline creation via wgpu's `constants` map. Default 64 covers
// models with nv <= 64. WebGPU guarantees max workgroup size >= 256.
override wg_size: u32 = 64;

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> qvel: array<f32>;
@group(0) @binding(2) var<storage, read> qacc: array<f32>;

@compute @workgroup_size(wg_size, 1, 1)
fn euler_integrate(
    @builtin(workgroup_id) wg_id: vec3<u32>,
    @builtin(local_invocation_id) local_id: vec3<u32>,
) {
    let env_idx = wg_id.x;
    if env_idx >= params.num_envs { return; }

    // Strided loop: each thread processes DOFs at stride wg_size.
    // Handles nv > wg_size (e.g., nv=300 with wg_size=256).
    var dof_idx = local_id.x;
    let base = env_idx * params.nv;
    loop {
        if dof_idx >= params.nv { break; }
        let offset = base + dof_idx;
        qvel[offset] = qvel[offset] + qacc[offset] * params.timestep;
        dof_idx += wg_size;
    }
}
