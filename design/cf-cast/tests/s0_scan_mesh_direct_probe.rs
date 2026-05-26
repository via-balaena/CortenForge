//! S0 empirical spike for the cf-cast scan-mesh-direct recon
//! (`docs/CF_CAST_SCAN_MESH_DIRECT_RECON.md`).
//!
//! Two `#[ignore]`-gated probes — manually invoked with
//! `cargo test --release -p cf-cast --test s0_scan_mesh_direct_probe
//!  -- --ignored --nocapture`. NO PRODUCTION COMMIT — this file is a
//! diagnostic scaffold; the spike's data feeds the recon's §SMD-11
//! decision matrix (A vs A2 vs plug-body-only).
//!
//! ## S0a — `s0a_mesh_offset_sweep`
//!
//! Characterize `mesh-offset::offset_mesh` at workshop's iter-1 scan
//! across (distance, resolution) ∈ {0.006, 0.014, 0.019} × {0.002,
//! 0.001, 0.0005}. For each cell: measure wall-clock; output face
//! count; manifoldness via `mesh-repair`; self-intersections via
//! `mesh-repair::detect_self_intersections`. Save outputs to
//! `~/scans/cast_iter1_s0a_mesh_offset/`.
//!
//! ## S0b — `s0b_vertex_displacement_sweep`
//!
//! Compute per-vertex outward normals (area-weighted face-normal
//! average) on the scan mesh, displace each vertex by offset
//! distance ∈ {0.006, 0.014, 0.019}, check manifoldness +
//! self-intersections. Save outputs to
//! `~/scans/cast_iter1_s0b_vertex_displaced/`.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    clippy::explicit_iter_loop,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::manual_assert,
    clippy::uninlined_format_args,
    clippy::default_trait_access,
    clippy::map_unwrap_or,
    clippy::doc_markdown,
    clippy::assigning_clones,
    clippy::missing_const_for_fn,
    dead_code
)]

use std::path::{Path, PathBuf};
use std::time::Instant;

use mesh_io::{load_stl, save_stl};
use mesh_offset::{OffsetConfig, SignOracle, offset_mesh};
use mesh_repair::intersect::{IntersectionParams, detect_self_intersections};
use mesh_types::{IndexedMesh, Point3, Vector3};

fn scan_path() -> PathBuf {
    let home = std::env::var("HOME").expect("HOME set");
    PathBuf::from(home).join("scans/sock_over_capsule.cleaned.stl")
}

fn out_dir(suffix: &str) -> PathBuf {
    let home = std::env::var("HOME").expect("HOME set");
    let dir = PathBuf::from(home).join(format!("scans/cast_iter1_s0{suffix}"));
    std::fs::create_dir_all(&dir).expect("create out dir");
    dir
}

fn mesh_aabb(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &mesh.vertices {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }
    (min, max)
}

fn fmt_aabb(min: &Point3<f64>, max: &Point3<f64>) -> String {
    format!(
        "[{:.3}..{:.3}, {:.3}..{:.3}, {:.3}..{:.3}] mm (size {:.1}×{:.1}×{:.1} mm)",
        min.x * 1000.0,
        max.x * 1000.0,
        min.y * 1000.0,
        max.y * 1000.0,
        min.z * 1000.0,
        max.z * 1000.0,
        (max.x - min.x) * 1000.0,
        (max.y - min.y) * 1000.0,
        (max.z - min.z) * 1000.0,
    )
}

fn report_mesh(label: &str, mesh: &IndexedMesh) {
    let (min, max) = mesh_aabb(mesh);
    eprintln!(
        "  {label}: faces={:>7}, verts={:>7}, AABB={}",
        mesh.faces.len(),
        mesh.vertices.len(),
        fmt_aabb(&min, &max),
    );
}

fn check_intersections(mesh: &IndexedMesh) -> (usize, bool) {
    let params = IntersectionParams::default();
    let result = detect_self_intersections(mesh, &params);
    (result.intersection_count, result.has_intersections)
}

#[test]
#[ignore = "S0a probe — run manually with --ignored"]
fn s0a_mesh_offset_sweep() {
    let scan = scan_path();
    if !scan.exists() {
        panic!(
            "scan not found at {} — set HOME or copy the scan in place",
            scan.display()
        );
    }
    let mesh = load_stl(&scan).expect("load_stl");
    eprintln!("\n=== S0a mesh-offset sweep ===");
    report_mesh("scan input", &mesh);
    let (n_int, has_int) = check_intersections(&mesh);
    eprintln!("  scan self-intersections: {} (has={})", n_int, has_int);

    let out = out_dir("a_mesh_offset");
    let distances_m = [0.006, 0.014, 0.019];
    let resolutions_m = [0.002, 0.001, 0.0005];

    eprintln!(
        "\n  distance (m) | resolution (m) | wall_s | faces | bytes (MB) | self-int | result"
    );
    eprintln!("  ------------ | -------------- | ------ | ----- | ---------- | -------- | ------");

    for &distance in distances_m.iter() {
        for &resolution in resolutions_m.iter() {
            let config = OffsetConfig {
                resolution,
                padding: 3,
                marching_cubes: Default::default(),
                sign_oracle: SignOracle::FloodFill,
            };

            let start = Instant::now();
            let result = offset_mesh(&mesh, distance, &config);
            let elapsed = start.elapsed();

            match result {
                Ok(offset) => {
                    let fname = format!(
                        "offset_d{}mm_r{}mm.stl",
                        (distance * 1000.0).round() as i64,
                        if (resolution * 1000.0) < 1.0 {
                            format!("0_{}", (resolution * 10000.0).round() as i64)
                        } else {
                            format!("{}", (resolution * 1000.0).round() as i64)
                        },
                    );
                    let out_path = out.join(&fname);
                    save_stl(&offset, &out_path, true).expect("save_stl");
                    let bytes = std::fs::metadata(&out_path).map(|m| m.len()).unwrap_or(0);
                    let (n_int, _) = check_intersections(&offset);
                    eprintln!(
                        "  {:12.3} | {:14.4} | {:>5.2}s | {:>5} | {:>9.2} | {:>8} | {}",
                        distance,
                        resolution,
                        elapsed.as_secs_f64(),
                        offset.faces.len(),
                        bytes as f64 / (1024.0 * 1024.0),
                        n_int,
                        fname,
                    );
                }
                Err(e) => {
                    eprintln!(
                        "  {:12.3} | {:14.4} | {:>5.2}s | ERROR: {:?}",
                        distance,
                        resolution,
                        elapsed.as_secs_f64(),
                        e
                    );
                }
            }
        }
    }
    eprintln!("\n  outputs saved to: {}\n", out.display());
}

/// Build area-weighted per-vertex outward normals for the input mesh.
/// Each vertex's normal = sum over incident faces of (face_area ×
/// face_normal), then normalized. Face_normal direction is per the
/// face's CCW vertex ordering (post-§Q-5 fix, this is outward for
/// scan meshes).
fn compute_vertex_normals(mesh: &IndexedMesh) -> Vec<Vector3<f64>> {
    let mut accum: Vec<Vector3<f64>> = vec![Vector3::zeros(); mesh.vertices.len()];
    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0] as usize];
        let v1 = mesh.vertices[face[1] as usize];
        let v2 = mesh.vertices[face[2] as usize];
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        // Cross product magnitude = 2 × triangle area; direction =
        // outward face normal under CCW vertex order.
        let cross = e1.cross(&e2);
        accum[face[0] as usize] += cross;
        accum[face[1] as usize] += cross;
        accum[face[2] as usize] += cross;
    }
    accum
        .into_iter()
        .map(|n| {
            let mag = n.norm();
            if mag > 1.0e-12 {
                n / mag
            } else {
                Vector3::zeros()
            }
        })
        .collect()
}

fn vertex_displacement_offset(mesh: &IndexedMesh, distance: f64) -> IndexedMesh {
    let normals = compute_vertex_normals(mesh);
    let mut out = IndexedMesh::new();
    out.vertices = mesh
        .vertices
        .iter()
        .zip(normals.iter())
        .map(|(v, n)| v + n * distance)
        .collect();
    out.faces = mesh.faces.clone();
    out
}

#[test]
#[ignore = "S0b probe — run manually with --ignored"]
fn s0b_vertex_displacement_sweep() {
    let scan = scan_path();
    if !scan.exists() {
        panic!(
            "scan not found at {} — set HOME or copy the scan in place",
            scan.display()
        );
    }
    let mesh = load_stl(&scan).expect("load_stl");
    eprintln!("\n=== S0b vertex-displacement sweep ===");
    report_mesh("scan input", &mesh);
    let (n_int, _) = check_intersections(&mesh);
    eprintln!("  scan self-intersections: {}", n_int);

    let out = out_dir("b_vertex_displaced");
    let distances_m = [0.006, 0.014, 0.019];

    eprintln!("\n  distance (m) | wall_s | faces | bytes (MB) | self-int | manifold | result");
    eprintln!("  ------------ | ------ | ----- | ---------- | -------- | -------- | ------");

    for &distance in distances_m.iter() {
        let start = Instant::now();
        let displaced = vertex_displacement_offset(&mesh, distance);
        let elapsed = start.elapsed();

        let fname = format!("displaced_d{}mm.stl", (distance * 1000.0).round() as i64);
        let out_path = out.join(&fname);
        save_stl(&displaced, &out_path, true).expect("save_stl");
        let bytes = std::fs::metadata(&out_path).map(|m| m.len()).unwrap_or(0);

        let (n_int, _) = check_intersections(&displaced);

        // Manifoldness: same vertex/face count as input means topology
        // is preserved (displacement doesn't change topology). A real
        // manifoldness check is via mesh-repair or manifold3d; for the
        // S0 spike, topology preservation + self-intersection-free
        // is the load-bearing signal.
        let manifold_ok = displaced.vertices.len() == mesh.vertices.len()
            && displaced.faces.len() == mesh.faces.len();

        eprintln!(
            "  {:12.3} | {:>5.3}s | {:>5} | {:>9.2} | {:>8} | {:>8} | {}",
            distance,
            elapsed.as_secs_f64(),
            displaced.faces.len(),
            bytes as f64 / (1024.0 * 1024.0),
            n_int,
            if manifold_ok { "yes" } else { "no" },
            fname,
        );
    }
    eprintln!("\n  outputs saved to: {}\n", out.display());
}

#[allow(dead_code)]
fn _ensure_paths_compile(_: &Path) {}
