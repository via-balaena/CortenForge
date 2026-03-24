// aabb.wgsl — Per-geom world-frame AABB computation
//
// One thread per geom. Reads FK-computed geom_xpos and geom_xmat,
// computes axis-aligned bounding box in world frame.
//
// Matches CPU: aabb_from_geom_aabb() in forward/position.rs
//
// Output: geom_aabb[g*2+0] = min (vec4, w=0)
//         geom_aabb[g*2+1] = max (vec4, w=0)

// ── Types ──────────────────────────────────────────────────────────────

struct AabbParams {
    ngeom: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

struct GeomModel {
    body_id: u32,
    geom_type: u32,
    contype: u32,
    conaffinity: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    size: vec4<f32>,
    friction: vec4<f32>,
    sdf_meta_idx: u32,
    condim: u32,
    _pad0: u32,
    _pad1: u32,
};

// ── Geom type constants ────────────────────────────────────────────────

const GEOM_PLANE: u32     = 0u;
const GEOM_SPHERE: u32    = 1u;
const GEOM_CAPSULE: u32   = 2u;
const GEOM_CYLINDER: u32  = 3u;
const GEOM_BOX: u32       = 4u;
const GEOM_ELLIPSOID: u32 = 5u;
const GEOM_MESH: u32      = 6u;
const GEOM_HFIELD: u32    = 7u;
const GEOM_SDF: u32       = 8u;

// ── Bindings ───────────────────────────────────────────────────────────

@group(0) @binding(0) var<uniform> params: AabbParams;
@group(1) @binding(0) var<storage, read> geoms: array<GeomModel>;
@group(2) @binding(0) var<storage, read> geom_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> geom_xmat: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read_write> geom_aabb: array<vec4<f32>>;

// ── Local half-extents per geom type ───────────────────────────────────

fn local_half_extents(gtype: u32, size: vec4<f32>) -> vec3<f32> {
    switch gtype {
        case 0u: { // Plane — infinite
            return vec3(1e6, 1e6, 1e6);
        }
        case 1u: { // Sphere — radius in size.x
            return vec3(size.x, size.x, size.x);
        }
        case 2u: { // Capsule — radius=size.x, half_length=size.y
            return vec3(size.x, size.x, size.x + size.y);
        }
        case 3u: { // Cylinder — radius=size.x, half_length=size.y
            return vec3(size.x, size.x, size.y);
        }
        case 4u: { // Box — half-extents (sx, sy, sz)
            return size.xyz;
        }
        case 5u: { // Ellipsoid — semi-axes (rx, ry, rz)
            return size.xyz;
        }
        case 8u: { // SDF — use geom_size as conservative AABB
            return size.xyz;
        }
        default: { // Mesh, Hfield — conservative from geom_size
            return size.xyz;
        }
    }
}

// ── Main kernel ────────────────────────────────────────────────────────

@compute @workgroup_size(64)
fn compute_aabb(@builtin(global_invocation_id) gid: vec3<u32>) {
    let g = gid.x;
    if g >= params.ngeom {
        return;
    }

    let geom = geoms[g];
    let pos = geom_xpos[g].xyz;

    // geom_xmat: 3×vec4 per geom (column-major rotation matrix)
    let col0 = geom_xmat[g * 3u + 0u].xyz;
    let col1 = geom_xmat[g * 3u + 1u].xyz;
    let col2 = geom_xmat[g * 3u + 2u].xyz;

    // Local-frame AABB half-extents from geom type + size
    let half = local_half_extents(geom.geom_type, geom.size);

    // Rotate OBB to world → axis-aligned envelope
    // world_half[i] = sum_j |mat[i,j]| * half[j]
    let world_half = vec3(
        abs(col0.x) * half.x + abs(col1.x) * half.y + abs(col2.x) * half.z,
        abs(col0.y) * half.x + abs(col1.y) * half.y + abs(col2.y) * half.z,
        abs(col0.z) * half.x + abs(col1.z) * half.y + abs(col2.z) * half.z,
    );

    geom_aabb[g * 2u + 0u] = vec4(pos - world_half, 0.0);
    geom_aabb[g * 2u + 1u] = vec4(pos + world_half, 0.0);
}
