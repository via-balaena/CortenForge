// sdf_plane_narrow.wgsl — SDF-plane narrowphase for the physics pipeline
//
// One thread per grid cell in the SDF geom. Tests whether the
// reconstructed surface point penetrates the plane. If so, atomically
// appends a PipelineContact.
//
// The plane equation is extracted from the plane geom's FK pose:
//   normal = geom_xmat column 2 (Z-axis of plane rotation)
//   offset = dot(normal, geom_xpos)
//
// params.geom1 = SDF geom index (source)
// params.geom2 = plane geom index (destination)
// params.src_sdf_meta_idx = SDF grid metadata index
// params.dst_sdf_meta_idx = unused (SDF_META_NONE)

// ── Types ──────────────────────────────────────────────────────────────

struct NarrowphaseParams {
    geom1: u32,
    geom2: u32,
    src_sdf_meta_idx: u32,
    dst_sdf_meta_idx: u32,
    surface_threshold: f32,
    contact_margin: f32,
    flip_normal: u32,
    _pad: u32,
    friction: vec4<f32>,
};

struct SdfMeta {
    width: u32,
    height: u32,
    depth: u32,
    cell_size: f32,
    origin: vec3<f32>,
    values_offset: u32,
};

struct PipelineContact {
    point: vec3<f32>,
    depth: f32,
    normal: vec3<f32>,
    geom1: u32,
    friction: vec3<f32>,
    geom2: u32,
};

// ── Bindings ───────────────────────────────────────────────────────────

@group(0) @binding(0) var<uniform> params: NarrowphaseParams;

@group(1) @binding(0) var<storage, read> sdf_metas: array<SdfMeta>;
@group(1) @binding(1) var<storage, read> sdf_values: array<f32>;

@group(2) @binding(0) var<storage, read> geom_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> geom_xmat: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> geom_aabb: array<vec4<f32>>;

@group(3) @binding(0) var<storage, read_write> contacts: array<PipelineContact>;
@group(3) @binding(1) var<storage, read_write> contact_count: atomic<u32>;

// ── Grid indexing ──────────────────────────────────────────────────────

fn grid_idx(x: u32, y: u32, z: u32, w: u32, h: u32) -> u32 {
    return z * w * h + y * w + x;
}

// ── Pose helpers ───────────────────────────────────────────────────────

fn geom_pose_mat4(g: u32) -> mat4x4<f32> {
    let col0 = geom_xmat[g * 3u + 0u];
    let col1 = geom_xmat[g * 3u + 1u];
    let col2 = geom_xmat[g * 3u + 2u];
    let pos  = geom_xpos[g];
    return mat4x4(
        vec4(col0.xyz, 0.0),
        vec4(col1.xyz, 0.0),
        vec4(col2.xyz, 0.0),
        vec4(pos.xyz, 1.0),
    );
}

// ── AABB overlap test ──────────────────────────────────────────────────

fn aabb_overlap(a_idx: u32, b_idx: u32) -> bool {
    let a_min = geom_aabb[a_idx * 2u + 0u].xyz;
    let a_max = geom_aabb[a_idx * 2u + 1u].xyz;
    let b_min = geom_aabb[b_idx * 2u + 0u].xyz;
    let b_max = geom_aabb[b_idx * 2u + 1u].xyz;

    return a_min.x <= b_max.x && b_min.x <= a_max.x
        && a_min.y <= b_max.y && b_min.y <= a_max.y
        && a_min.z <= b_max.z && b_min.z <= a_max.z;
}

// ── SDF trilinear interpolation ────────────────────────────────────────

fn sdf_trilinear(point: vec3<f32>, gm: SdfMeta) -> f32 {
    let rel = (point - gm.origin) / gm.cell_size;

    if rel.x < 0.0 || rel.y < 0.0 || rel.z < 0.0 {
        return 1e6;
    }
    let i = vec3<u32>(vec3<i32>(floor(rel)));
    if i.x >= gm.width - 1u || i.y >= gm.height - 1u || i.z >= gm.depth - 1u {
        return 1e6;
    }

    let f = fract(rel);
    let w = gm.width;
    let h = gm.height;
    let off = gm.values_offset;

    let c000 = sdf_values[off + grid_idx(i.x,      i.y,      i.z,      w, h)];
    let c100 = sdf_values[off + grid_idx(i.x + 1u, i.y,      i.z,      w, h)];
    let c010 = sdf_values[off + grid_idx(i.x,      i.y + 1u, i.z,      w, h)];
    let c110 = sdf_values[off + grid_idx(i.x + 1u, i.y + 1u, i.z,      w, h)];
    let c001 = sdf_values[off + grid_idx(i.x,      i.y,      i.z + 1u, w, h)];
    let c101 = sdf_values[off + grid_idx(i.x + 1u, i.y,      i.z + 1u, w, h)];
    let c011 = sdf_values[off + grid_idx(i.x,      i.y + 1u, i.z + 1u, w, h)];
    let c111 = sdf_values[off + grid_idx(i.x + 1u, i.y + 1u, i.z + 1u, w, h)];

    let c00 = mix(c000, c100, f.x);
    let c10 = mix(c010, c110, f.x);
    let c01 = mix(c001, c101, f.x);
    let c11 = mix(c011, c111, f.x);
    let c0  = mix(c00,  c10,  f.y);
    let c1  = mix(c01,  c11,  f.y);
    return mix(c0, c1, f.z);
}

fn sdf_trilinear_clamped(point: vec3<f32>, gm: SdfMeta) -> f32 {
    let max_bound = gm.origin + vec3<f32>(
        f32(gm.width - 1u),
        f32(gm.height - 1u),
        f32(gm.depth - 1u),
    ) * gm.cell_size;
    let clamped = clamp(point, gm.origin, max_bound);
    return sdf_trilinear(clamped, gm);
}

fn sdf_gradient(point: vec3<f32>, gm: SdfMeta) -> vec3<f32> {
    let d = sdf_trilinear(point, gm);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);
    }

    let eps = gm.cell_size * 0.5;
    let two_eps = 2.0 * eps;
    let dx = sdf_trilinear_clamped(point + vec3(eps, 0.0, 0.0), gm)
           - sdf_trilinear_clamped(point - vec3(eps, 0.0, 0.0), gm);
    let dy = sdf_trilinear_clamped(point + vec3(0.0, eps, 0.0), gm)
           - sdf_trilinear_clamped(point - vec3(0.0, eps, 0.0), gm);
    let dz = sdf_trilinear_clamped(point + vec3(0.0, 0.0, eps), gm)
           - sdf_trilinear_clamped(point - vec3(0.0, 0.0, eps), gm);

    let grad = vec3(dx / two_eps, dy / two_eps, dz / two_eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;
    } else {
        return vec3(0.0, 0.0, 1.0);
    }
}

// ── Main kernel ────────────────────────────────────────────────────────

@compute @workgroup_size(8, 8, 4)
fn sdf_plane_narrow(@builtin(global_invocation_id) gid: vec3<u32>) {
    // 0. AABB guard
    if !aabb_overlap(params.geom1, params.geom2) {
        return;
    }

    // 1. Load SDF grid metadata
    let gm = sdf_metas[params.src_sdf_meta_idx];

    let x = gid.x;
    let y = gid.y;
    let z = gid.z;
    if x >= gm.width || y >= gm.height || z >= gm.depth {
        return;
    }

    // 2. Read SDF value
    let idx = gm.values_offset + grid_idx(x, y, z, gm.width, gm.height);
    let sdf_value = sdf_values[idx];

    // 3. Surface filter
    if sdf_value > params.surface_threshold {
        return;
    }

    // 4. Compute local position
    let local = gm.origin + vec3(f32(x), f32(y), f32(z)) * gm.cell_size;

    // 5. Surface gradient
    let grad = sdf_gradient(local, gm);
    if dot(grad, grad) < 0.5 {
        return;
    }

    // 6. Surface reconstruction
    let surface_local = local - grad * sdf_value;

    // 7. Transform to world
    let sdf_pose = geom_pose_mat4(params.geom1);
    let world = (sdf_pose * vec4(surface_local, 1.0)).xyz;

    // 8. Extract plane equation from plane geom's FK pose
    // Plane normal = Z-axis (column 2) of plane's rotation matrix
    let plane_normal = geom_xmat[params.geom2 * 3u + 2u].xyz;
    let plane_pos = geom_xpos[params.geom2].xyz;
    let plane_offset = dot(plane_normal, plane_pos);

    // 9. Signed distance to plane (positive = above plane)
    let dist = dot(plane_normal, world) - plane_offset;

    // 10. Contact test
    if dist >= params.contact_margin {
        return;
    }
    let penetration = max(-dist, 0.0);

    // 11. Emit contact — normal points away from plane (push object up)
    let ci = atomicAdd(&contact_count, 1u);
    if ci < arrayLength(&contacts) {
        contacts[ci] = PipelineContact(
            world,
            penetration,
            plane_normal,       // Normal points UP (away from plane)
            params.geom1,       // SDF geom
            params.friction.xyz,
            params.geom2,       // Plane geom
        );
    }
}
