// sdf_sdf_narrow.wgsl — SDF-SDF narrowphase for the physics pipeline
//
// Adapted from trace_surface.wgsl. Key changes:
// 1. AABB guard — returns if geom pair AABBs don't overlap
// 2. Unified SDF buffer — reads from sdf_values[offset + idx]
// 3. Extended contact output — 48-byte PipelineContact with geom indices + friction
// 4. Poses read from FK output buffers (geom_xpos, geom_xmat)
//
// For symmetric SDF-SDF collision, dispatch twice with src/dst swapped.

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

fn geom_pose_inv_mat4(g: u32) -> mat4x4<f32> {
    // For rigid transform [R|t], inverse = [R^T | -R^T·t]
    let col0 = geom_xmat[g * 3u + 0u].xyz;
    let col1 = geom_xmat[g * 3u + 1u].xyz;
    let col2 = geom_xmat[g * 3u + 2u].xyz;
    let pos  = geom_xpos[g].xyz;
    // R^T rows = R columns
    let rt0 = vec3(col0.x, col1.x, col2.x);
    let rt1 = vec3(col0.y, col1.y, col2.y);
    let rt2 = vec3(col0.z, col1.z, col2.z);
    let neg_rt_t = -vec3(dot(rt0, pos), dot(rt1, pos), dot(rt2, pos));
    return mat4x4(
        vec4(rt0, 0.0),
        vec4(rt1, 0.0),
        vec4(rt2, 0.0),
        vec4(neg_rt_t, 1.0),
    );
}

// ── Grid indexing ──────────────────────────────────────────────────────

fn grid_idx(x: u32, y: u32, z: u32, w: u32, h: u32) -> u32 {
    return z * w * h + y * w + x;
}

// ── Source grid: trilinear interpolation (via unified buffer) ─────────

fn src_trilinear(point: vec3<f32>, gm: SdfMeta) -> f32 {
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

fn src_trilinear_clamped(point: vec3<f32>, gm: SdfMeta) -> f32 {
    let max_bound = gm.origin + vec3<f32>(
        f32(gm.width - 1u),
        f32(gm.height - 1u),
        f32(gm.depth - 1u),
    ) * gm.cell_size;
    let clamped = clamp(point, gm.origin, max_bound);
    return src_trilinear(clamped, gm);
}

fn src_gradient(point: vec3<f32>, gm: SdfMeta) -> vec3<f32> {
    let d = src_trilinear(point, gm);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);
    }

    let eps = gm.cell_size * 0.5;
    let two_eps = 2.0 * eps;
    let dx = src_trilinear_clamped(point + vec3(eps, 0.0, 0.0), gm)
           - src_trilinear_clamped(point - vec3(eps, 0.0, 0.0), gm);
    let dy = src_trilinear_clamped(point + vec3(0.0, eps, 0.0), gm)
           - src_trilinear_clamped(point - vec3(0.0, eps, 0.0), gm);
    let dz = src_trilinear_clamped(point + vec3(0.0, 0.0, eps), gm)
           - src_trilinear_clamped(point - vec3(0.0, 0.0, eps), gm);

    let grad = vec3(dx / two_eps, dy / two_eps, dz / two_eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;
    } else {
        return vec3(0.0, 0.0, 1.0);
    }
}

// ── Destination grid: trilinear interpolation (via unified buffer) ────

fn dst_trilinear(point: vec3<f32>, gm: SdfMeta) -> f32 {
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

fn dst_trilinear_clamped(point: vec3<f32>, gm: SdfMeta) -> f32 {
    let max_bound = gm.origin + vec3<f32>(
        f32(gm.width - 1u),
        f32(gm.height - 1u),
        f32(gm.depth - 1u),
    ) * gm.cell_size;
    let clamped = clamp(point, gm.origin, max_bound);
    return dst_trilinear(clamped, gm);
}

fn dst_gradient(point: vec3<f32>, gm: SdfMeta) -> vec3<f32> {
    let d = dst_trilinear(point, gm);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);
    }

    let eps = gm.cell_size * 0.5;
    let two_eps = 2.0 * eps;
    let dx = dst_trilinear_clamped(point + vec3(eps, 0.0, 0.0), gm)
           - dst_trilinear_clamped(point - vec3(eps, 0.0, 0.0), gm);
    let dy = dst_trilinear_clamped(point + vec3(0.0, eps, 0.0), gm)
           - dst_trilinear_clamped(point - vec3(0.0, eps, 0.0), gm);
    let dz = dst_trilinear_clamped(point + vec3(0.0, 0.0, eps), gm)
           - dst_trilinear_clamped(point - vec3(0.0, 0.0, eps), gm);

    let grad = vec3(dx / two_eps, dy / two_eps, dz / two_eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;
    } else {
        return vec3(0.0, 0.0, 1.0);
    }
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

// ── Main kernel ────────────────────────────────────────────────────────

@compute @workgroup_size(8, 8, 4)
fn sdf_sdf_narrow(@builtin(global_invocation_id) gid: vec3<u32>) {
    // 0. AABB guard — skip entire dispatch if no overlap
    if !aabb_overlap(params.geom1, params.geom2) {
        return;
    }

    // 1. Load grid metadata
    let src_gm = sdf_metas[params.src_sdf_meta_idx];
    let dst_gm = sdf_metas[params.dst_sdf_meta_idx];

    let x = gid.x;
    let y = gid.y;
    let z = gid.z;
    if x >= src_gm.width || y >= src_gm.height || z >= src_gm.depth {
        return;
    }

    // 2. Build pose matrices from FK output
    let src_pose = geom_pose_mat4(params.geom1);
    let dst_pose = geom_pose_mat4(params.geom2);
    let dst_pose_inv = geom_pose_inv_mat4(params.geom2);

    // 3. Read source SDF value
    let idx = src_gm.values_offset + grid_idx(x, y, z, src_gm.width, src_gm.height);
    let src_value = sdf_values[idx];

    // One-sided filter: skip far-outside cells
    if src_value > params.surface_threshold {
        return;
    }

    // 4. Compute local position from grid coordinates
    let local = src_gm.origin + vec3<f32>(f32(x), f32(y), f32(z)) * src_gm.cell_size;

    // 5. Gradient (centered differences, normalized)
    let grad = src_gradient(local, src_gm);
    if dot(grad, grad) < 0.5 {
        return;
    }

    // 6. Surface reconstruction: project onto zero-isosurface
    let surface_local = local - grad * src_value;

    // 7. Transform: src local → world
    let world = (src_pose * vec4<f32>(surface_local, 1.0)).xyz;

    // 8. Transform: world → dst local
    let dst_local = (dst_pose_inv * vec4<f32>(world, 1.0)).xyz;

    // 9. Query destination SDF distance
    let dst_dist = dst_trilinear(dst_local, dst_gm);

    // 10. Contact test
    if dst_dist >= params.contact_margin {
        return;
    }
    let penetration = max(-dst_dist, 0.0);

    // 11. Destination gradient for contact normal
    let dst_grad = dst_gradient(dst_local, dst_gm);
    if dot(dst_grad, dst_grad) < 0.5 {
        return;
    }

    // 12. Normal convention — matches CPU exactly
    var normal_world = (dst_pose * vec4<f32>(dst_grad, 0.0)).xyz;
    if params.flip_normal == 0u {
        normal_world = -normal_world;
    }

    // 13. Atomic append to contact buffer
    let ci = atomicAdd(&contact_count, 1u);
    if ci < arrayLength(&contacts) {
        contacts[ci] = PipelineContact(
            world,
            penetration,
            normal_world,
            params.geom1,
            params.friction.xyz,
            params.geom2,
        );
    }
}
