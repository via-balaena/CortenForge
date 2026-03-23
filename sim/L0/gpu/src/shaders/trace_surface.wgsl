// trace_surface.wgsl — Exact port of trace_surface_into_other()
//
// One thread per grid cell in the source SDF.
// Tests whether the reconstructed surface point penetrates the
// destination SDF. If so, atomically appends a contact.
//
// For symmetric SDF-SDF collision, dispatch twice with src/dst swapped.

// ── Bindings ───────────────────────────────────────────────────────────

@group(0) @binding(0) var<storage, read> src_grid: array<f32>;
@group(0) @binding(1) var<storage, read> dst_grid: array<f32>;
@group(0) @binding(2) var<uniform> src_meta: GridMeta;
@group(0) @binding(3) var<uniform> dst_meta: GridMeta;
@group(0) @binding(4) var<uniform> params: TraceParams;
@group(0) @binding(5) var<storage, read_write> contacts: array<GpuContact>;
@group(0) @binding(6) var<storage, read_write> contact_count: atomic<u32>;

// ── Types ──────────────────────────────────────────────────────────────

struct GridMeta {
    width: u32,
    height: u32,
    depth: u32,
    cell_size: f32,
    origin: vec3<f32>,
    _pad: f32,
}

struct TraceParams {
    src_pose: mat4x4<f32>,          // src local → world
    dst_pose: mat4x4<f32>,          // dst local → world (for normal rotation)
    dst_pose_inv: mat4x4<f32>,      // world → dst local
    surface_threshold: f32,          // src_cell_size * 2.0
    contact_margin: f32,
    flip_normal: u32,                // 0 = A→B (negate normal), 1 = B→A (keep)
    _pad: f32,
}

struct GpuContact {
    point: vec3<f32>,
    penetration: f32,
    normal: vec3<f32>,
    _pad: f32,
}

// ── Grid indexing ──────────────────────────────────────────────────────

fn grid_idx(x: u32, y: u32, z: u32, w: u32, h: u32) -> u32 {
    return z * w * h + y * w + x;
}

// ── Source grid: trilinear interpolation ────────────────────────────────

fn src_trilinear(point: vec3<f32>) -> f32 {
    let rel = (point - src_meta.origin) / src_meta.cell_size;

    if rel.x < 0.0 || rel.y < 0.0 || rel.z < 0.0 {
        return 1e6;
    }
    let i = vec3<u32>(vec3<i32>(floor(rel)));
    if i.x >= src_meta.width - 1u || i.y >= src_meta.height - 1u || i.z >= src_meta.depth - 1u {
        return 1e6;
    }

    let f = fract(rel);
    let w = src_meta.width;
    let h = src_meta.height;

    let c000 = src_grid[grid_idx(i.x,      i.y,      i.z,      w, h)];
    let c100 = src_grid[grid_idx(i.x + 1u, i.y,      i.z,      w, h)];
    let c010 = src_grid[grid_idx(i.x,      i.y + 1u, i.z,      w, h)];
    let c110 = src_grid[grid_idx(i.x + 1u, i.y + 1u, i.z,      w, h)];
    let c001 = src_grid[grid_idx(i.x,      i.y,      i.z + 1u, w, h)];
    let c101 = src_grid[grid_idx(i.x + 1u, i.y,      i.z + 1u, w, h)];
    let c011 = src_grid[grid_idx(i.x,      i.y + 1u, i.z + 1u, w, h)];
    let c111 = src_grid[grid_idx(i.x + 1u, i.y + 1u, i.z + 1u, w, h)];

    let c00 = mix(c000, c100, f.x);
    let c10 = mix(c010, c110, f.x);
    let c01 = mix(c001, c101, f.x);
    let c11 = mix(c011, c111, f.x);
    let c0  = mix(c00,  c10,  f.y);
    let c1  = mix(c01,  c11,  f.y);
    return mix(c0, c1, f.z);
}

fn src_trilinear_clamped(point: vec3<f32>) -> f32 {
    let max_bound = src_meta.origin + vec3<f32>(
        f32(src_meta.width - 1u),
        f32(src_meta.height - 1u),
        f32(src_meta.depth - 1u),
    ) * src_meta.cell_size;
    let clamped = clamp(point, src_meta.origin, max_bound);
    return src_trilinear(clamped);
}

// Source gradient: forward differences, normalized.
// Matches CPU: SdfGrid::gradient() in cf-geometry/src/sdf.rs
fn src_gradient(point: vec3<f32>) -> vec3<f32> {
    let d = src_trilinear(point);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);  // out of bounds
    }

    let eps = src_meta.cell_size * 0.5;
    let dx = src_trilinear_clamped(point + vec3(eps, 0.0, 0.0));
    let dy = src_trilinear_clamped(point + vec3(0.0, eps, 0.0));
    let dz = src_trilinear_clamped(point + vec3(0.0, 0.0, eps));

    let grad = vec3((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;
    } else {
        return vec3(0.0, 0.0, 1.0);  // +Z fallback
    }
}

// ── Destination grid: trilinear interpolation ──────────────────────────

fn dst_trilinear(point: vec3<f32>) -> f32 {
    let rel = (point - dst_meta.origin) / dst_meta.cell_size;

    if rel.x < 0.0 || rel.y < 0.0 || rel.z < 0.0 {
        return 1e6;
    }
    let i = vec3<u32>(vec3<i32>(floor(rel)));
    if i.x >= dst_meta.width - 1u || i.y >= dst_meta.height - 1u || i.z >= dst_meta.depth - 1u {
        return 1e6;
    }

    let f = fract(rel);
    let w = dst_meta.width;
    let h = dst_meta.height;

    let c000 = dst_grid[grid_idx(i.x,      i.y,      i.z,      w, h)];
    let c100 = dst_grid[grid_idx(i.x + 1u, i.y,      i.z,      w, h)];
    let c010 = dst_grid[grid_idx(i.x,      i.y + 1u, i.z,      w, h)];
    let c110 = dst_grid[grid_idx(i.x + 1u, i.y + 1u, i.z,      w, h)];
    let c001 = dst_grid[grid_idx(i.x,      i.y,      i.z + 1u, w, h)];
    let c101 = dst_grid[grid_idx(i.x + 1u, i.y,      i.z + 1u, w, h)];
    let c011 = dst_grid[grid_idx(i.x,      i.y + 1u, i.z + 1u, w, h)];
    let c111 = dst_grid[grid_idx(i.x + 1u, i.y + 1u, i.z + 1u, w, h)];

    let c00 = mix(c000, c100, f.x);
    let c10 = mix(c010, c110, f.x);
    let c01 = mix(c001, c101, f.x);
    let c11 = mix(c011, c111, f.x);
    let c0  = mix(c00,  c10,  f.y);
    let c1  = mix(c01,  c11,  f.y);
    return mix(c0, c1, f.z);
}

fn dst_trilinear_clamped(point: vec3<f32>) -> f32 {
    let max_bound = dst_meta.origin + vec3<f32>(
        f32(dst_meta.width - 1u),
        f32(dst_meta.height - 1u),
        f32(dst_meta.depth - 1u),
    ) * dst_meta.cell_size;
    let clamped = clamp(point, dst_meta.origin, max_bound);
    return dst_trilinear(clamped);
}

// Destination gradient: forward differences, normalized.
fn dst_gradient(point: vec3<f32>) -> vec3<f32> {
    let d = dst_trilinear(point);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);
    }

    let eps = dst_meta.cell_size * 0.5;
    let dx = dst_trilinear_clamped(point + vec3(eps, 0.0, 0.0));
    let dy = dst_trilinear_clamped(point + vec3(0.0, eps, 0.0));
    let dz = dst_trilinear_clamped(point + vec3(0.0, 0.0, eps));

    let grad = vec3((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;
    } else {
        return vec3(0.0, 0.0, 1.0);
    }
}

// ── Main kernel ────────────────────────────────────────────────────────

@compute @workgroup_size(8, 8, 4)
fn trace_surface(@builtin(global_invocation_id) gid: vec3<u32>) {
    let x = gid.x;
    let y = gid.y;
    let z = gid.z;
    if x >= src_meta.width || y >= src_meta.height || z >= src_meta.depth {
        return;
    }

    // 1. Read source SDF value
    let idx = grid_idx(x, y, z, src_meta.width, src_meta.height);
    let src_value = src_grid[idx];

    // One-sided filter: skip far-outside, process deep interior.
    // Matches CPU: `if src_value > surface_threshold { continue; }`
    if src_value > params.surface_threshold {
        return;
    }

    // 2. Compute local position from grid coordinates
    let local = src_meta.origin + vec3<f32>(f32(x), f32(y), f32(z)) * src_meta.cell_size;

    // 3. Gradient (forward differences, normalized unit vector)
    let grad = src_gradient(local);
    if dot(grad, grad) < 0.5 {
        return;  // degenerate or out-of-bounds (src_gradient returns zero vec)
    }

    // 4. Surface reconstruction: project grid point onto zero-isosurface
    let surface_local = local - grad * src_value;

    // 5. Transform: src local → world
    let world = (params.src_pose * vec4<f32>(surface_local, 1.0)).xyz;

    // 6. Transform: world → dst local
    let dst_local = (params.dst_pose_inv * vec4<f32>(world, 1.0)).xyz;

    // 7. Query destination SDF distance
    let dst_dist = dst_trilinear(dst_local);
    // Out-of-bounds → 1e6 → exceeds margin → skipped

    // 8. Contact test
    // Matches CPU: `if dst_dist >= margin { continue; }`
    if dst_dist >= params.contact_margin {
        return;
    }
    // Matches CPU: `let penetration = (-dst_dist).max(0.0);`
    let penetration = max(-dst_dist, 0.0);

    // 9. Destination gradient for contact normal
    let dst_grad = dst_gradient(dst_local);
    if dot(dst_grad, dst_grad) < 0.5 {
        return;
    }

    // 10. Normal convention — matches CPU exactly:
    //   flip_normal==0 (first call, A→B): negate dst outward gradient
    //   flip_normal==1 (second call, B→A): use dst outward gradient as-is
    var normal_world = (params.dst_pose * vec4<f32>(dst_grad, 0.0)).xyz;
    if params.flip_normal == 0u {
        normal_world = -normal_world;
    }

    // 11. Atomic append to contact buffer
    let ci = atomicAdd(&contact_count, 1u);
    if ci < arrayLength(&contacts) {
        contacts[ci] = GpuContact(world, penetration, normal_world, 0.0);
    }
}
