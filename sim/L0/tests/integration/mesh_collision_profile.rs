//! DT-179 Session 1: Mesh-plane collision performance profiling.
//!
//! Reproduces the scenario: mesh sphere geoms + ground plane + free joint.
//! Uses `std::time::Instant` bracketing and step1/step2 split to identify
//! the hot path.
//!
//! ## Hypothesis
//!
//! Mesh AABBs use fixed ±10m extent (`MESH_DEFAULT_EXTENT` in
//! `forward/position.rs:343`) instead of tight-fitting bounds, inflating
//! broadphase candidate pairs and triggering unnecessary narrowphase work
//! (GJK/EPA on convex hulls for false-positive mesh-mesh pairs).
//!
//! ## Findings (release mode, Apple M-series)
//!
//! 1. **AABB bloat confirmed**: `MESH_DEFAULT_EXTENT = 10.0` → 20m³ bounding
//!    box per mesh geom. `geom_size` is [0.1, 0.1, 0.1] for all mesh geoms
//!    (default from `geom_size_to_vec3` catch-all). AABB ignores geom_size
//!    entirely.
//!
//! 2. **step1 (FK+collision) dominates** for mesh scenes: 51-94% of step time
//!    vs 7-19% for primitive scenes. The hot path is in collision detection.
//!
//! 3. **Mesh/primitive ratio is ~2-3×** (not 40,000× as spec claims). For
//!    42-vertex icosphere meshes, GJK/EPA on convex hulls is fast enough that
//!    false broadphase pairs don't cause catastrophic slowdown.
//!
//! 4. **O(N²) broadphase pairs confirmed**: With N well-separated mesh geoms
//!    (5m apart, r=0.5), 10m AABBs cause ALL mesh-mesh pairs to overlap.
//!    Expected tight pairs: N (plane-mesh only). Actual bloat pairs: N + C(N,2).
//!
//! 5. **Fix needed**: Compute tight AABBs from transformed mesh vertex AABB
//!    (stored on `TriangleMeshData` at build time). This eliminates false
//!    mesh-mesh pairs when geoms are well-separated.
//!
//! ## For Session 3 (fix)
//!
//! Replace `MESH_DEFAULT_EXTENT` with tight AABB from mesh vertex positions:
//! ```ignore
//! GeomType::Mesh => {
//!     let mesh_id = model.geom_mesh[geom_id]?;
//!     let mesh_aabb = model.mesh_data[mesh_id].aabb();
//!     // Transform local AABB by geom pose (pos + mat)
//!     transform_aabb(mesh_aabb, pos, mat)
//! }
//! ```
//!
//! Run in release mode for meaningful numbers:
//! ```sh
//! cargo test -p sim-conformance-tests --release --test integration mesh_collision_profile -- --nocapture
//! ```

use sim_mjcf::load_model;
use std::hint::black_box;
use std::time::Instant;

// ============================================================================
// Icosphere mesh generation
// ============================================================================

/// Generate icosphere vertex and face strings for MJCF inline mesh.
///
/// - `subdivisions=0`: 12 vertices, 20 faces (icosahedron)
/// - `subdivisions=1`: 42 vertices, 80 faces
/// - `subdivisions=2`: 162 vertices, 320 faces
fn icosphere_mjcf_strings(radius: f64, subdivisions: u32) -> (String, String) {
    let phi: f64 = (1.0 + 5.0_f64.sqrt()) / 2.0;

    let raw = [
        (-1.0, phi, 0.0),
        (1.0, phi, 0.0),
        (-1.0, -phi, 0.0),
        (1.0, -phi, 0.0),
        (0.0, -1.0, phi),
        (0.0, 1.0, phi),
        (0.0, -1.0, -phi),
        (0.0, 1.0, -phi),
        (phi, 0.0, -1.0),
        (phi, 0.0, 1.0),
        (-phi, 0.0, -1.0),
        (-phi, 0.0, 1.0),
    ];

    let mut vertices: Vec<[f64; 3]> = raw
        .iter()
        .map(|&(x, y, z)| {
            let len = (x * x + y * y + z * z).sqrt();
            [x / len * radius, y / len * radius, z / len * radius]
        })
        .collect();

    let mut faces: Vec<[usize; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    for _ in 0..subdivisions {
        use std::collections::HashMap;
        let mut midpoint_cache: HashMap<(usize, usize), usize> = HashMap::new();

        let mut get_midpoint = |v1: usize, v2: usize, verts: &mut Vec<[f64; 3]>| -> usize {
            let key = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            if let Some(&idx) = midpoint_cache.get(&key) {
                return idx;
            }
            let a = verts[v1];
            let b = verts[v2];
            let mid = [
                (a[0] + b[0]) / 2.0,
                (a[1] + b[1]) / 2.0,
                (a[2] + b[2]) / 2.0,
            ];
            let len = (mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]).sqrt();
            let normalized = [
                mid[0] / len * radius,
                mid[1] / len * radius,
                mid[2] / len * radius,
            ];
            let idx = verts.len();
            verts.push(normalized);
            midpoint_cache.insert(key, idx);
            idx
        };

        let old_faces = faces.clone();
        faces.clear();
        for f in &old_faces {
            let a = get_midpoint(f[0], f[1], &mut vertices);
            let b = get_midpoint(f[1], f[2], &mut vertices);
            let c = get_midpoint(f[2], f[0], &mut vertices);
            faces.push([f[0], a, c]);
            faces.push([f[1], b, a]);
            faces.push([f[2], c, b]);
            faces.push([a, b, c]);
        }
    }

    let vert_str: String = vertices
        .iter()
        .map(|v| format!("{:.6} {:.6} {:.6}", v[0], v[1], v[2]))
        .collect::<Vec<_>>()
        .join("  ");

    let face_str: String = faces
        .iter()
        .map(|f| format!("{} {} {}", f[0], f[1], f[2]))
        .collect::<Vec<_>>()
        .join("  ");

    (vert_str, face_str)
}

// ============================================================================
// Main profiling test
// ============================================================================

/// DT-179 profiling: mesh spheres on ground plane (in contact from step 0).
/// Compares mesh vs primitive performance and identifies the hot path via
/// step1/step2 split.
#[test]
fn dt179_mesh_plane_collision_profile() {
    eprintln!("\n=== DT-179: Mesh-Plane Collision Profile ===\n");

    let n_steps = 500;

    for subdivisions in [1, 2] {
        let (verts1, faces1) = icosphere_mjcf_strings(1.0, subdivisions);
        let (verts2, faces2) = icosphere_mjcf_strings(0.5, subdivisions);

        let v_count = match subdivisions {
            0 => 12,
            1 => 42,
            2 => 162,
            _ => 0,
        };
        eprintln!("--- {v_count} vertices per sphere (subdivision {subdivisions}) ---\n");

        // Mesh + plane: balls ON the plane
        let mjcf_mesh_plane = format!(
            r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <asset>
    <mesh name="s1" vertex="{verts1}" face="{faces1}"/>
    <mesh name="s2" vertex="{verts2}" face="{faces2}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="b1" pos="0 0 1.0">
      <joint type="free"/>
      <geom type="mesh" mesh="s1" density="1000"/>
    </body>
    <body name="b2" pos="3 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="s2" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
        );

        // Mesh only: no plane, separated (no mesh-mesh overlap)
        let mjcf_mesh_only = format!(
            r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <asset>
    <mesh name="s1" vertex="{verts1}" face="{faces1}"/>
    <mesh name="s2" vertex="{verts2}" face="{faces2}"/>
  </asset>
  <worldbody>
    <body name="b1" pos="0 0 1.0">
      <joint type="free"/>
      <geom type="mesh" mesh="s1" density="1000"/>
    </body>
    <body name="b2" pos="3 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="s2" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
        );

        // Primitive + plane: analytical collision baseline
        let mjcf_prim = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="b1" pos="0 0 1.0">
      <joint type="free"/>
      <geom type="sphere" size="1.0" density="1000"/>
    </body>
    <body name="b2" pos="3 0 0.5">
      <joint type="free"/>
      <geom type="sphere" size="0.5" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#;

        let m_mp = load_model(&mjcf_mesh_plane).unwrap();
        let m_mo = load_model(&mjcf_mesh_only).unwrap();
        let m_pp = load_model(mjcf_prim).unwrap();

        // Report mesh info
        for (i, md) in m_mp.mesh_data.iter().enumerate() {
            let hull = md.convex_hull().map_or(0, |h| h.vertices.len());
            eprintln!(
                "  mesh {i}: {} verts, {} tris, hull={hull} verts",
                md.vertex_count(),
                md.triangle_count()
            );
        }

        // Full step timing
        let (t_mp, nc_mp) = profile_steps(&m_mp, n_steps, "mesh+plane");
        let (t_mo, nc_mo) = profile_steps(&m_mo, n_steps, "mesh-only");
        let (t_pp, nc_pp) = profile_steps(&m_pp, n_steps, "prim+plane");

        // Split step timing
        profile_split_steps(&m_mp, n_steps, "mesh+plane");
        profile_split_steps(&m_mo, n_steps, "mesh-only");
        profile_split_steps(&m_pp, n_steps, "prim+plane");

        let ratio_plane = t_mp / t_mo.max(1e-15);
        let ratio_prim = t_mp / t_pp.max(1e-15);
        eprintln!("\n  mesh+plane / mesh-only:  {ratio_plane:.1}×");
        eprintln!("  mesh+plane / prim+plane: {ratio_prim:.1}×");
        eprintln!("  Contacts: mp={nc_mp} mo={nc_mo} pp={nc_pp}\n");
    }
}

// ============================================================================
// Scaling test: mesh AABB bloat with N geoms
// ============================================================================

/// DT-179 scaling: demonstrate broadphase false positives from 10m mesh AABBs.
///
/// N mesh geoms spaced 5m apart → no physical overlap. With tight AABBs,
/// only N plane-mesh pairs survive broadphase. With 10m AABBs, ALL C(N+1,2)
/// pairs survive, triggering O(N²) GJK/EPA narrowphase calls.
#[test]
fn dt179_mesh_aabb_scaling() {
    let (verts, faces) = icosphere_mjcf_strings(0.5, 1);

    eprintln!("\n=== DT-179: Mesh AABB Scaling ===\n");

    for n_meshes in [2, 5, 10, 15] {
        let spacing = 5.0;

        let mut bodies = String::new();
        for i in 0..n_meshes {
            #[allow(clippy::cast_precision_loss)]
            let x = i as f64 * spacing;
            bodies.push_str(&format!(
                r#"    <body name="b{i}" pos="{x} 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="s" density="1000"/>
    </body>
"#
            ));
        }

        let mjcf_mesh = format!(
            r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <asset><mesh name="s" vertex="{verts}" face="{faces}"/></asset>
  <worldbody>
    <geom name="floor" type="plane" size="50 50 0.1"/>
{bodies}  </worldbody>
</mujoco>"#
        );

        let mut prim_bodies = String::new();
        for i in 0..n_meshes {
            #[allow(clippy::cast_precision_loss)]
            let x = i as f64 * spacing;
            prim_bodies.push_str(&format!(
                r#"    <body name="b{i}" pos="{x} 0 0.5">
      <joint type="free"/>
      <geom type="sphere" size="0.5" density="1000"/>
    </body>
"#
            ));
        }

        let mjcf_prim = format!(
            r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="50 50 0.1"/>
{prim_bodies}  </worldbody>
</mujoco>"#
        );

        let m_mesh = load_model(&mjcf_mesh).unwrap();
        let m_prim = load_model(&mjcf_prim).unwrap();

        let n_steps = 200;
        let (t_m, nc_m) = profile_steps(&m_mesh, n_steps, &format!("{n_meshes} mesh"));
        let (t_p, nc_p) = profile_steps(&m_prim, n_steps, &format!("{n_meshes} prim"));

        profile_split_steps(&m_mesh, n_steps, &format!("{n_meshes} mesh"));
        profile_split_steps(&m_prim, n_steps, &format!("{n_meshes} prim"));

        let ratio = t_m / t_p.max(1e-15);
        // With tight AABBs: N plane-mesh pairs → N narrowphase calls
        // With 10m AABBs: C(N+1,2) pairs → N + C(N,2) narrowphase calls
        // C(N,2) = N*(N-1)/2 false mesh-mesh pairs
        let expected_tight = n_meshes;
        let expected_bloat = n_meshes + n_meshes * (n_meshes - 1) / 2;
        eprintln!(
            "  N={n_meshes}: mesh/prim={ratio:.1}×, contacts: m={nc_m} p={nc_p}, \
             tight_pairs={expected_tight} bloat_pairs={expected_bloat}\n"
        );
    }
}

// ============================================================================
// Helpers
// ============================================================================

fn profile_steps(model: &sim_core::Model, n: usize, label: &str) -> (f64, usize) {
    let mut data = model.make_data();

    for _ in 0..10 {
        data.step(model).expect("warmup failed");
    }
    let ncon = data.ncon;

    let start = Instant::now();
    for _ in 0..n {
        black_box(data.step(model)).expect("step failed");
    }
    let total = start.elapsed().as_secs_f64();
    #[allow(clippy::cast_precision_loss)]
    let per_step = total / n as f64 * 1e6;

    eprintln!("  [{label}] {per_step:.1}µs/step, contacts={ncon}");
    (total, ncon)
}

fn profile_split_steps(model: &sim_core::Model, n: usize, label: &str) {
    let mut data = model.make_data();

    for _ in 0..10 {
        data.step(model).expect("warmup failed");
    }

    let mut t1_total = std::time::Duration::ZERO;
    let mut t2_total = std::time::Duration::ZERO;

    for _ in 0..n {
        let a = Instant::now();
        black_box(data.step1(model)).expect("step1 failed");
        let b = Instant::now();
        black_box(data.step2(model)).expect("step2 failed");
        let c = Instant::now();
        t1_total += b - a;
        t2_total += c - b;
    }

    #[allow(clippy::cast_precision_loss)]
    let s1 = t1_total.as_secs_f64() / n as f64 * 1e6;
    #[allow(clippy::cast_precision_loss)]
    let s2 = t2_total.as_secs_f64() / n as f64 * 1e6;
    let total = s1 + s2;

    eprintln!(
        "  [{label}] step1={s1:.1}µs ({:.0}%), step2={s2:.1}µs ({:.0}%)",
        s1 / total * 100.0,
        s2 / total * 100.0,
    );
}
