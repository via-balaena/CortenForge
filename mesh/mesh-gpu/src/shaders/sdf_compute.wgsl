// SDF (Signed Distance Field) computation shader.
//
// This shader computes the signed distance from each voxel in a 3D grid to the
// nearest point on a triangle mesh. The algorithm:
// 1. For each voxel, compute the unsigned distance to all triangles
// 2. Keep track of the minimum distance and corresponding triangle
// 3. Determine sign using raycast parity (odd intersections = inside)

// Triangle structure matching GpuTriangle in Rust
struct Triangle {
    v0: vec4<f32>,
    v1: vec4<f32>,
    v2: vec4<f32>,
}

// Grid parameters matching GpuGridParams in Rust
struct GridParams {
    origin: vec4<f32>,
    dims: vec4<u32>,
    voxel_size: f32,
    triangle_count: u32,
    _padding: vec2<f32>,
}

// Bind groups
@group(0) @binding(0) var<storage, read> triangles: array<Triangle>;
@group(0) @binding(1) var<uniform> grid: GridParams;
@group(0) @binding(2) var<storage, read_write> sdf_values: array<f32>;

// Compute the squared distance from a point to a triangle.
// Returns the squared distance to the closest point on the triangle.
fn point_triangle_distance_sq(p: vec3<f32>, v0: vec3<f32>, v1: vec3<f32>, v2: vec3<f32>) -> f32 {
    let e0 = v1 - v0;
    let e1 = v2 - v0;
    let v = p - v0;

    let d00 = dot(e0, e0);
    let d01 = dot(e0, e1);
    let d11 = dot(e1, e1);
    let d20 = dot(v, e0);
    let d21 = dot(v, e1);

    let denom = d00 * d11 - d01 * d01;

    // Check for degenerate triangle
    if abs(denom) < 1e-10 {
        // Fall back to edge distance
        let dist0 = point_segment_distance_sq(p, v0, v1);
        let dist1 = point_segment_distance_sq(p, v1, v2);
        let dist2 = point_segment_distance_sq(p, v2, v0);
        return min(min(dist0, dist1), dist2);
    }

    var s = (d11 * d20 - d01 * d21) / denom;
    var t = (d00 * d21 - d01 * d20) / denom;

    // Check if point projects inside triangle
    if s >= 0.0 && t >= 0.0 && s + t <= 1.0 {
        // Point projects inside triangle - compute perpendicular distance
        let closest = v0 + s * e0 + t * e1;
        let diff = p - closest;
        return dot(diff, diff);
    }

    // Point projects outside triangle - check edges and vertices
    var min_dist_sq = 1e30f;

    // Edge v0-v1
    if s < 0.0 {
        min_dist_sq = min(min_dist_sq, point_segment_distance_sq(p, v0, v1));
    }

    // Edge v0-v2
    if t < 0.0 {
        min_dist_sq = min(min_dist_sq, point_segment_distance_sq(p, v0, v2));
    }

    // Edge v1-v2
    if s + t > 1.0 {
        min_dist_sq = min(min_dist_sq, point_segment_distance_sq(p, v1, v2));
    }

    // Also check vertices
    min_dist_sq = min(min_dist_sq, distance_sq(p, v0));
    min_dist_sq = min(min_dist_sq, distance_sq(p, v1));
    min_dist_sq = min(min_dist_sq, distance_sq(p, v2));

    return min_dist_sq;
}

// Squared distance from point to line segment
fn point_segment_distance_sq(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = dot(ab, ab);

    if ab_len_sq < 1e-10 {
        return dot(ap, ap);
    }

    var t = dot(ap, ab) / ab_len_sq;
    t = clamp(t, 0.0, 1.0);

    let closest = a + t * ab;
    let diff = p - closest;
    return dot(diff, diff);
}

// Squared distance between two points
fn distance_sq(a: vec3<f32>, b: vec3<f32>) -> f32 {
    let d = a - b;
    return dot(d, d);
}

// Ray-triangle intersection test (Moller-Trumbore algorithm).
// Returns 1 if ray intersects triangle, 0 otherwise.
fn ray_triangle_intersect(origin: vec3<f32>, dir: vec3<f32>, v0: vec3<f32>, v1: vec3<f32>, v2: vec3<f32>) -> u32 {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = cross(dir, edge2);
    let a = dot(edge1, h);

    // Check if ray is parallel to triangle
    if abs(a) < 1e-7 {
        return 0u;
    }

    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * dot(s, h);

    if u < 0.0 || u > 1.0 {
        return 0u;
    }

    let q = cross(s, edge1);
    let v = f * dot(dir, q);

    if v < 0.0 || u + v > 1.0 {
        return 0u;
    }

    let t = f * dot(edge2, q);

    // Only count intersections in positive ray direction
    if t > 1e-7 {
        return 1u;
    }

    return 0u;
}

// Convert linear index to 3D grid coordinates.
// Uses ZYX ordering to match mesh_to_sdf's layout.
fn delinearize(idx: u32) -> vec3<u32> {
    let z = idx % grid.dims.z;
    let rem = idx / grid.dims.z;
    let y = rem % grid.dims.y;
    let x = rem / grid.dims.y;
    return vec3<u32>(x, y, z);
}

// Get world position of voxel center
fn voxel_center(coords: vec3<u32>) -> vec3<f32> {
    return vec3<f32>(
        grid.origin.x + (f32(coords.x) + 0.5) * grid.voxel_size,
        grid.origin.y + (f32(coords.y) + 0.5) * grid.voxel_size,
        grid.origin.z + (f32(coords.z) + 0.5) * grid.voxel_size
    );
}

// Main compute shader for SDF computation.
// Each thread processes one voxel.
@compute @workgroup_size(256, 1, 1)
fn compute_sdf(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let idx = global_id.x;
    let total_voxels = grid.dims.x * grid.dims.y * grid.dims.z;

    // Bounds check
    if idx >= total_voxels {
        return;
    }

    // Get voxel position
    let coords = delinearize(idx);
    let pos = voxel_center(coords);

    // Find minimum distance to all triangles
    var min_dist_sq = 1e30f;

    for (var i = 0u; i < grid.triangle_count; i = i + 1u) {
        let tri = triangles[i];
        let v0 = tri.v0.xyz;
        let v1 = tri.v1.xyz;
        let v2 = tri.v2.xyz;

        let dist_sq = point_triangle_distance_sq(pos, v0, v1, v2);
        min_dist_sq = min(min_dist_sq, dist_sq);
    }

    let unsigned_dist = sqrt(min_dist_sq);

    // Determine sign using raycast (positive X direction)
    var intersection_count = 0u;
    let ray_dir = vec3<f32>(1.0, 0.0, 0.0);

    for (var i = 0u; i < grid.triangle_count; i = i + 1u) {
        let tri = triangles[i];
        let v0 = tri.v0.xyz;
        let v1 = tri.v1.xyz;
        let v2 = tri.v2.xyz;

        intersection_count = intersection_count + ray_triangle_intersect(pos, ray_dir, v0, v1, v2);
    }

    // Odd intersection count means inside (negative distance)
    var sign = 1.0f;
    if (intersection_count & 1u) == 1u {
        sign = -1.0f;
    }

    sdf_values[idx] = sign * unsigned_dist;
}
