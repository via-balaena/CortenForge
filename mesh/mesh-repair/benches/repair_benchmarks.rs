//! Benchmarks for mesh-repair operations.
//!
//! Run with: cargo bench -p mesh-repair
//!
//! To compare against baseline:
//! 1. First run: cargo bench -p mesh-repair -- --save-baseline main
//! 2. After changes: cargo bench -p mesh-repair -- --baseline main

#![allow(
    missing_docs,
    clippy::cast_possible_truncation,
    clippy::assigning_clones,
    clippy::significant_drop_tightening
)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use mesh_repair::intersect::IntersectionParams;
use mesh_repair::{RepairParams, validate_mesh, weld_vertices};
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

    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
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
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
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

    let mx = (p1.position.x + p2.position.x) / 2.0;
    let my = (p1.position.y + p2.position.y) / 2.0;
    let mz = (p1.position.z + p2.position.z) / 2.0;
    let len = (mx * mx + my * my + mz * mz).sqrt();

    let idx = vertices.len() as u32;
    vertices.push(Vertex::from_coords(mx / len, my / len, mz / len));
    edge_midpoints.insert(key, idx);
    idx
}

// =============================================================================
// Validation Benchmarks
// =============================================================================

fn bench_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("Validation");

    let test_cases = [
        ("cube_12tri", create_cube()),
        ("sphere_80tri", create_sphere(1)),
        ("sphere_320tri", create_sphere(2)),
        ("sphere_1280tri", create_sphere(3)),
        ("sphere_5120tri", create_sphere(4)),
    ];

    for (name, mesh) in &test_cases {
        group.throughput(Throughput::Elements(mesh.faces.len() as u64));

        group.bench_with_input(BenchmarkId::new("validate", name), mesh, |b, mesh| {
            b.iter(|| validate_mesh(black_box(mesh)))
        });
    }

    group.finish();
}

// =============================================================================
// Repair Benchmarks
// =============================================================================

fn bench_repair(c: &mut Criterion) {
    let mut group = c.benchmark_group("Repair");

    let test_cases = [
        ("cube_12tri", create_cube()),
        ("sphere_320tri", create_sphere(2)),
        ("sphere_1280tri", create_sphere(3)),
    ];

    for (name, mesh) in &test_cases {
        group.throughput(Throughput::Elements(mesh.faces.len() as u64));

        group.bench_with_input(BenchmarkId::new("weld_vertices", name), mesh, |b, mesh| {
            let mut m = mesh.clone();
            b.iter(|| {
                weld_vertices(&mut m, 1e-6);
            })
        });

        group.bench_with_input(BenchmarkId::new("full_repair", name), mesh, |b, mesh| {
            let mut m = mesh.clone();
            let params = RepairParams::default();
            b.iter(|| {
                let _ = mesh_repair::repair_mesh(&mut m, &params);
            })
        });
    }

    group.finish();
}

// =============================================================================
// Hole Filling Benchmarks
// =============================================================================

fn bench_hole_filling(c: &mut Criterion) {
    let mut group = c.benchmark_group("HoleFilling");

    // Create mesh with holes (open cube missing one face)
    let mut open_cube = create_cube();
    open_cube.faces.pop();
    open_cube.faces.pop();

    group.bench_function("fill_holes_cube", |b| {
        let mut mesh = open_cube.clone();
        b.iter(|| {
            let _ = mesh_repair::holes::fill_holes(&mut mesh, 100);
        })
    });

    group.finish();
}

// =============================================================================
// Intersection Detection Benchmarks
// =============================================================================

fn bench_intersection(c: &mut Criterion) {
    let mut group = c.benchmark_group("Intersection");

    let test_cases = [
        ("sphere_320tri", create_sphere(2)),
        ("sphere_1280tri", create_sphere(3)),
    ];

    for (name, mesh) in &test_cases {
        group.throughput(Throughput::Elements(mesh.faces.len() as u64));

        group.bench_with_input(
            BenchmarkId::new("detect_self_intersection", name),
            mesh,
            |b, mesh| {
                let params = IntersectionParams::default();
                b.iter(|| {
                    mesh_repair::intersect::detect_self_intersections(
                        black_box(mesh),
                        black_box(&params),
                    )
                })
            },
        );
    }

    group.finish();
}

// =============================================================================
// Criterion Setup
// =============================================================================

criterion_group!(
    benches,
    bench_validation,
    bench_repair,
    bench_hole_filling,
    bench_intersection,
);

criterion_main!(benches);
