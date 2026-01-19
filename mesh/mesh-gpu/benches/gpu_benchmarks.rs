//! GPU benchmarks for mesh-gpu.
//!
//! Run with: `cargo bench -p mesh-gpu`
//!
//! These benchmarks compare GPU and CPU SDF computation performance.

#![allow(
    missing_docs,
    clippy::cast_possible_truncation,
    clippy::assigning_clones,
    clippy::significant_drop_tightening
)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};

use mesh_gpu::{GpuContext, GpuSdfParams, try_compute_sdf_gpu};
use mesh_types::{IndexedMesh, Vertex};

/// Create a cube mesh with specified size.
fn create_cube(half_size: f64) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    let s = half_size;
    let coords = [
        [-s, -s, -s],
        [s, -s, -s],
        [s, s, -s],
        [-s, s, -s],
        [-s, -s, s],
        [s, -s, s],
        [s, s, s],
        [-s, s, s],
    ];

    for c in &coords {
        mesh.vertices.push(Vertex::from_coords(c[0], c[1], c[2]));
    }

    let faces: [[u32; 3]; 12] = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];

    for f in &faces {
        mesh.faces.push(*f);
    }

    mesh
}

/// Create an icosphere mesh with specified subdivisions.
/// Subdivisions: 0 = 20 faces, 1 = 80 faces, 2 = 320 faces, etc.
fn create_icosphere(subdivisions: u32) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    // Golden ratio
    let phi = f64::midpoint(1.0, 5.0_f64.sqrt());
    let norm = (1.0 + phi * phi).sqrt();
    let a = 1.0 / norm;
    let b = phi / norm;

    // Initial icosahedron vertices
    let initial_verts = [
        [-a, b, 0.0],
        [a, b, 0.0],
        [-a, -b, 0.0],
        [a, -b, 0.0],
        [0.0, -a, b],
        [0.0, a, b],
        [0.0, -a, -b],
        [0.0, a, -b],
        [b, 0.0, -a],
        [b, 0.0, a],
        [-b, 0.0, -a],
        [-b, 0.0, a],
    ];

    for v in &initial_verts {
        mesh.vertices.push(Vertex::from_coords(v[0], v[1], v[2]));
    }

    // Initial icosahedron faces
    let initial_faces: [[u32; 3]; 20] = [
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

    for f in &initial_faces {
        mesh.faces.push(*f);
    }

    // Subdivide
    for _ in 0..subdivisions {
        subdivide_mesh(&mut mesh);
    }

    mesh
}

/// Subdivide mesh by splitting each triangle into 4.
#[allow(clippy::cast_possible_truncation)]
fn subdivide_mesh(mesh: &mut IndexedMesh) {
    use std::collections::HashMap;

    let mut edge_midpoints: HashMap<(u32, u32), u32> = HashMap::new();
    let mut new_faces = Vec::new();

    // Clone faces to avoid borrowing conflict
    let old_faces = mesh.faces.clone();

    for face in &old_faces {
        let i0 = face[0];
        let i1 = face[1];
        let i2 = face[2];

        let m01 = get_midpoint(mesh, &mut edge_midpoints, i0, i1);
        let m12 = get_midpoint(mesh, &mut edge_midpoints, i1, i2);
        let m20 = get_midpoint(mesh, &mut edge_midpoints, i2, i0);

        new_faces.push([i0, m01, m20]);
        new_faces.push([i1, m12, m01]);
        new_faces.push([i2, m20, m12]);
        new_faces.push([m01, m12, m20]);
    }

    mesh.faces = new_faces;
}

/// Get midpoint vertex index, creating a new vertex if needed.
#[allow(clippy::cast_possible_truncation)]
fn get_midpoint(
    mesh: &mut IndexedMesh,
    edge_midpoints: &mut std::collections::HashMap<(u32, u32), u32>,
    i0: u32,
    i1: u32,
) -> u32 {
    let key = if i0 < i1 { (i0, i1) } else { (i1, i0) };

    if let Some(&idx) = edge_midpoints.get(&key) {
        return idx;
    }

    let p0 = &mesh.vertices[i0 as usize].position;
    let p1 = &mesh.vertices[i1 as usize].position;
    let mid = (p0.coords + p1.coords) / 2.0;
    let mid_normalized = mid.normalize();

    let idx = mesh.vertices.len() as u32;
    mesh.vertices.push(Vertex::from_coords(
        mid_normalized.x,
        mid_normalized.y,
        mid_normalized.z,
    ));
    edge_midpoints.insert(key, idx);
    idx
}

fn bench_sdf_gpu_varying_grid_size(c: &mut Criterion) {
    if !GpuContext::is_available() {
        println!("Skipping GPU benchmarks - no GPU available");
        return;
    }

    let mesh = create_cube(1.0);
    let mut group = c.benchmark_group("SDF GPU - Varying Grid Size");

    for grid_size in [16, 32, 64, 128] {
        let voxels = grid_size * grid_size * grid_size;
        group.throughput(Throughput::Elements(voxels as u64));

        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{grid_size}³")),
            &grid_size,
            |b, &size| {
                let params =
                    GpuSdfParams::new([size, size, size], [-2.0, -2.0, -2.0], 4.0 / size as f32);
                b.iter(|| {
                    let result = try_compute_sdf_gpu(black_box(&mesh), black_box(&params));
                    black_box(result)
                });
            },
        );
    }

    group.finish();
}

fn bench_sdf_gpu_varying_mesh_size(c: &mut Criterion) {
    if !GpuContext::is_available() {
        println!("Skipping GPU benchmarks - no GPU available");
        return;
    }

    let mut group = c.benchmark_group("SDF GPU - Varying Mesh Size");

    // Create meshes of different complexity
    let meshes = [
        ("cube-12tri", create_cube(1.0)),
        ("sphere-20tri", create_icosphere(0)),
        ("sphere-80tri", create_icosphere(1)),
        ("sphere-320tri", create_icosphere(2)),
        ("sphere-1280tri", create_icosphere(3)),
    ];

    let params = GpuSdfParams::new([64, 64, 64], [-2.0, -2.0, -2.0], 0.0625);
    let voxels = 64 * 64 * 64;
    group.throughput(Throughput::Elements(voxels));

    for (name, mesh) in &meshes {
        group.bench_with_input(BenchmarkId::from_parameter(name), mesh, |b, mesh| {
            b.iter(|| {
                let result = try_compute_sdf_gpu(black_box(mesh), black_box(&params));
                black_box(result)
            });
        });
    }

    group.finish();
}

fn bench_sdf_gpu_initialization(c: &mut Criterion) {
    if !GpuContext::is_available() {
        println!("Skipping GPU benchmarks - no GPU available");
        return;
    }

    let mesh = create_cube(1.0);
    let params = GpuSdfParams::new([32, 32, 32], [-2.0, -2.0, -2.0], 0.125);

    c.bench_function("SDF GPU - Warm (context cached)", |b| {
        // Ensure context is initialized
        let _ = GpuContext::get();

        b.iter(|| {
            let result = try_compute_sdf_gpu(black_box(&mesh), black_box(&params));
            black_box(result)
        });
    });
}

criterion_group!(
    benches,
    bench_sdf_gpu_varying_grid_size,
    bench_sdf_gpu_varying_mesh_size,
    bench_sdf_gpu_initialization,
);

criterion_main!(benches);
