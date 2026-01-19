//! Benchmarks for mesh-shell operations.
//!
//! Run with: cargo bench -p mesh-shell
//!
//! To compare against baseline:
//! 1. First run: cargo bench -p mesh-shell -- --save-baseline main
//! 2. After changes: cargo bench -p mesh-shell -- --baseline main

#![allow(
    missing_docs,
    clippy::cast_possible_truncation,
    clippy::assigning_clones
)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use mesh_shell::{ShellBuilder, ShellParams, generate_shell};
use mesh_types::{IndexedMesh, Vertex};
use std::collections::HashMap;

// =============================================================================
// Test Mesh Generation
// =============================================================================

/// Create a unit cube mesh (12 triangles).
fn create_cube() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    let verts = [
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ];

    for v in &verts {
        mesh.vertices.push(Vertex::from_coords(v[0], v[1], v[2]));
    }

    let faces: [[u32; 3]; 12] = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [2, 6, 7],
        [2, 7, 3],
        [0, 3, 7],
        [0, 7, 4],
        [1, 5, 6],
        [1, 6, 2],
    ];

    for f in &faces {
        mesh.faces.push(*f);
    }

    mesh
}

/// Create an icosphere mesh with specified subdivision level.
fn create_sphere(subdivisions: u32) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    let phi = f64::midpoint(1.0, 5.0_f64.sqrt());
    let a = 1.0;
    let b = 1.0 / phi;

    let ico_verts = [
        [0.0, b, -a],
        [b, a, 0.0],
        [-b, a, 0.0],
        [0.0, b, a],
        [0.0, -b, a],
        [-a, 0.0, b],
        [0.0, -b, -a],
        [a, 0.0, -b],
        [a, 0.0, b],
        [-a, 0.0, -b],
        [b, -a, 0.0],
        [-b, -a, 0.0],
    ];

    for v in &ico_verts {
        let len = v[2].mul_add(v[2], v[0].mul_add(v[0], v[1] * v[1])).sqrt();
        mesh.vertices
            .push(Vertex::from_coords(v[0] / len, v[1] / len, v[2] / len));
    }

    let ico_faces: [[u32; 3]; 20] = [
        [0, 1, 2],
        [3, 2, 1],
        [3, 4, 5],
        [3, 8, 4],
        [0, 6, 7],
        [0, 9, 6],
        [4, 10, 11],
        [6, 11, 10],
        [2, 5, 9],
        [11, 9, 5],
        [1, 7, 8],
        [10, 8, 7],
        [3, 5, 2],
        [3, 1, 8],
        [0, 2, 9],
        [0, 7, 1],
        [6, 9, 11],
        [6, 10, 7],
        [4, 11, 5],
        [4, 8, 10],
    ];

    for f in &ico_faces {
        mesh.faces.push(*f);
    }

    for _ in 0..subdivisions {
        mesh = subdivide_sphere(&mesh);
    }

    mesh
}

fn subdivide_sphere(mesh: &IndexedMesh) -> IndexedMesh {
    let mut new_mesh = IndexedMesh::new();
    new_mesh.vertices = mesh.vertices.clone();

    let mut edge_midpoints: HashMap<(u32, u32), u32> = HashMap::new();

    for face in &mesh.faces {
        let v0 = face[0];
        let v1 = face[1];
        let v2 = face[2];

        let m01 = get_midpoint(v0, v1, &mut new_mesh.vertices, &mut edge_midpoints);
        let m12 = get_midpoint(v1, v2, &mut new_mesh.vertices, &mut edge_midpoints);
        let m20 = get_midpoint(v2, v0, &mut new_mesh.vertices, &mut edge_midpoints);

        new_mesh.faces.push([v0, m01, m20]);
        new_mesh.faces.push([v1, m12, m01]);
        new_mesh.faces.push([v2, m20, m12]);
        new_mesh.faces.push([m01, m12, m20]);
    }

    new_mesh
}

fn get_midpoint(
    v1: u32,
    v2: u32,
    vertices: &mut Vec<Vertex>,
    edge_midpoints: &mut HashMap<(u32, u32), u32>,
) -> u32 {
    let key = if v1 < v2 { (v1, v2) } else { (v2, v1) };

    if let Some(&idx) = edge_midpoints.get(&key) {
        return idx;
    }

    let p1 = &vertices[v1 as usize];
    let p2 = &vertices[v2 as usize];

    let mx = f64::midpoint(p1.position.x, p2.position.x);
    let my = f64::midpoint(p1.position.y, p2.position.y);
    let mz = f64::midpoint(p1.position.z, p2.position.z);
    let len = mz.mul_add(mz, mx.mul_add(mx, my * my)).sqrt();

    let idx = vertices.len() as u32;
    vertices.push(Vertex::from_coords(mx / len, my / len, mz / len));
    edge_midpoints.insert(key, idx);
    idx
}

// =============================================================================
// Shell Generation Benchmarks
// =============================================================================

fn bench_shell_generation(c: &mut Criterion) {
    let mut group = c.benchmark_group("ShellGeneration");
    group.sample_size(10); // Shell generation is slow

    let test_cases = [
        ("cube_12tri", create_cube()),
        ("sphere_80tri", create_sphere(1)),
        ("sphere_320tri", create_sphere(2)),
    ];

    for (name, mesh) in &test_cases {
        group.throughput(Throughput::Elements(mesh.faces.len() as u64));

        // Fast method (normal-based)
        group.bench_with_input(BenchmarkId::new("fast", name), &mesh, |b, mesh| {
            let params = ShellParams::fast();
            b.iter(|| generate_shell(black_box(mesh), black_box(&params)));
        });

        // Builder API with fast preset
        group.bench_with_input(BenchmarkId::new("builder_fast", name), &mesh, |b, mesh| {
            b.iter(|| {
                ShellBuilder::new(black_box(mesh))
                    .wall_thickness(2.0)
                    .fast()
                    .build()
            });
        });
    }

    group.finish();
}

// =============================================================================
// Criterion Setup
// =============================================================================

criterion_group!(benches, bench_shell_generation);
criterion_main!(benches);
