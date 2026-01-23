//! Benchmarks for collision detection operations.
//!
//! Run with: cargo bench -p sim-core
//!
//! Performance targets (from MESH_MESH_COLLISION_PLAN.md):
//! - <5ms for 10k triangle pair test

#![allow(missing_docs, clippy::wildcard_imports)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use nalgebra::{Point3, UnitQuaternion};
use rand::Rng;

use sim_core::Pose;
use sim_core::mesh::{TriangleMeshData, mesh_mesh_contact, mesh_mesh_deepest_contact};

/// Generate a cube mesh with specified subdivision level.
/// Level 0 = 12 triangles (basic cube)
/// Level 1 = 48 triangles
/// Higher levels produce more triangles via subdivision.
fn generate_cube_mesh(subdivisions: u32) -> TriangleMeshData {
    let mut vertices = vec![
        // Bottom face
        Point3::new(-0.5, -0.5, -0.5),
        Point3::new(0.5, -0.5, -0.5),
        Point3::new(0.5, 0.5, -0.5),
        Point3::new(-0.5, 0.5, -0.5),
        // Top face
        Point3::new(-0.5, -0.5, 0.5),
        Point3::new(0.5, -0.5, 0.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(-0.5, 0.5, 0.5),
    ];

    let mut indices = vec![
        // Bottom (-Z)
        0, 1, 2, 0, 2, 3, // Top (+Z)
        4, 6, 5, 4, 7, 6, // Front (-Y)
        0, 5, 1, 0, 4, 5, // Back (+Y)
        2, 7, 3, 2, 6, 7, // Left (-X)
        0, 7, 4, 0, 3, 7, // Right (+X)
        1, 6, 2, 1, 5, 6,
    ];

    // Simple subdivision: split each triangle into 4
    for _ in 0..subdivisions {
        let mut new_indices = Vec::with_capacity(indices.len() * 4);

        for tri in indices.chunks(3) {
            let v0 = vertices[tri[0]];
            let v1 = vertices[tri[1]];
            let v2 = vertices[tri[2]];

            // Compute midpoints
            let m01 = Point3::from((v0.coords + v1.coords) * 0.5);
            let m12 = Point3::from((v1.coords + v2.coords) * 0.5);
            let m20 = Point3::from((v2.coords + v0.coords) * 0.5);

            let i0 = tri[0];
            let i1 = tri[1];
            let i2 = tri[2];
            let i01 = vertices.len();
            let i12 = i01 + 1;
            let i20 = i01 + 2;

            vertices.push(m01);
            vertices.push(m12);
            vertices.push(m20);

            // Create 4 new triangles
            new_indices.extend_from_slice(&[i0, i01, i20]);
            new_indices.extend_from_slice(&[i01, i1, i12]);
            new_indices.extend_from_slice(&[i20, i12, i2]);
            new_indices.extend_from_slice(&[i01, i12, i20]);
        }

        indices = new_indices;
    }

    TriangleMeshData::new(vertices, indices)
}

/// Benchmark mesh-mesh collision with varying mesh complexity.
fn bench_mesh_mesh_collision(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh_mesh_collision");

    // Test with different mesh complexities
    // 12 tri, 48 tri, ~200 tri, ~800 tri, ~3000 tri
    for subdivisions in [0, 1, 2, 3, 4] {
        let mesh_a = generate_cube_mesh(subdivisions);
        let mesh_b = generate_cube_mesh(subdivisions);
        let tri_count = mesh_a.triangle_count();

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.3, 0.0, 0.0));

        group.throughput(Throughput::Elements(
            (tri_count * mesh_b.triangle_count()) as u64,
        ));

        group.bench_with_input(
            BenchmarkId::new("overlapping", format!("{}x{}_tri", tri_count, tri_count)),
            &(&mesh_a, &mesh_b, &pose_a, &pose_b),
            |b, (mesh_a, mesh_b, pose_a, pose_b)| {
                b.iter(|| black_box(mesh_mesh_contact(mesh_a, pose_a, mesh_b, pose_b)));
            },
        );

        // Also test deepest contact (what's used in dispatch)
        group.bench_with_input(
            BenchmarkId::new("deepest", format!("{}x{}_tri", tri_count, tri_count)),
            &(&mesh_a, &mesh_b, &pose_a, &pose_b),
            |b, (mesh_a, mesh_b, pose_a, pose_b)| {
                b.iter(|| black_box(mesh_mesh_deepest_contact(mesh_a, pose_a, mesh_b, pose_b)));
            },
        );
    }

    group.finish();
}

/// Benchmark mesh-mesh collision with rotation (tests BVH transform).
fn bench_mesh_mesh_rotated(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh_mesh_rotated");

    let mesh_a = generate_cube_mesh(3); // ~768 triangles
    let mesh_b = generate_cube_mesh(3);
    let tri_count = mesh_a.triangle_count();

    group.throughput(Throughput::Elements(
        (tri_count * mesh_b.triangle_count()) as u64,
    ));

    // Test with different rotations
    for angle_deg in [0, 15, 45, 90] {
        let pose_a = Pose::identity();
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, (angle_deg as f64).to_radians());
        let pose_b = Pose::from_position_rotation(Point3::new(0.3, 0.0, 0.0), rotation);

        group.bench_with_input(
            BenchmarkId::new("rotation", format!("{}_deg", angle_deg)),
            &(&mesh_a, &mesh_b, &pose_a, &pose_b),
            |b, (mesh_a, mesh_b, pose_a, pose_b)| {
                b.iter(|| black_box(mesh_mesh_contact(mesh_a, pose_a, mesh_b, pose_b)));
            },
        );
    }

    group.finish();
}

/// Benchmark separate meshes (early rejection via BVH).
fn bench_mesh_mesh_separate(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh_mesh_separate");

    for subdivisions in [2, 3, 4] {
        let mesh_a = generate_cube_mesh(subdivisions);
        let mesh_b = generate_cube_mesh(subdivisions);
        let tri_count = mesh_a.triangle_count();

        let pose_a = Pose::identity();
        // Position mesh B far away (no collision)
        let pose_b = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        group.bench_with_input(
            BenchmarkId::new("no_overlap", format!("{}x{}_tri", tri_count, tri_count)),
            &(&mesh_a, &mesh_b, &pose_a, &pose_b),
            |b, (mesh_a, mesh_b, pose_a, pose_b)| {
                b.iter(|| black_box(mesh_mesh_contact(mesh_a, pose_a, mesh_b, pose_b)));
            },
        );
    }

    group.finish();
}

/// Benchmark to verify <5ms target for 10k triangle pair test.
/// This is the key performance metric from the plan.
fn bench_10k_triangle_pairs(c: &mut Criterion) {
    let mut group = c.benchmark_group("target_10k_pairs");

    // Create meshes that when combined give ~10k triangle pairs
    // ~100 triangles each -> 10k pairs
    // Use subdivision level 2 (~192 triangles) which gives ~37k pairs
    // Or level 1 (~48 triangles) which gives ~2.3k pairs
    // We'll test multiple scenarios

    // Scenario 1: Two ~100-triangle meshes (192 * 192 = ~37k pairs - above target)
    let mesh_a = generate_cube_mesh(2);
    let mesh_b = generate_cube_mesh(2);
    let pairs = mesh_a.triangle_count() * mesh_b.triangle_count();

    let pose_a = Pose::identity();
    let pose_b = Pose::from_position(Point3::new(0.3, 0.0, 0.0));

    group.throughput(Throughput::Elements(pairs as u64));

    group.bench_with_input(
        BenchmarkId::new("mesh_pair", format!("{}_pairs", pairs)),
        &(&mesh_a, &mesh_b, &pose_a, &pose_b),
        |b, (mesh_a, mesh_b, pose_a, pose_b)| {
            b.iter(|| black_box(mesh_mesh_deepest_contact(mesh_a, pose_a, mesh_b, pose_b)));
        },
    );

    // Scenario 2: Larger meshes (~768 triangles each -> ~590k pairs)
    let mesh_a_large = generate_cube_mesh(3);
    let mesh_b_large = generate_cube_mesh(3);
    let pairs_large = mesh_a_large.triangle_count() * mesh_b_large.triangle_count();

    group.throughput(Throughput::Elements(pairs_large as u64));

    group.bench_with_input(
        BenchmarkId::new("mesh_pair", format!("{}_pairs", pairs_large)),
        &(&mesh_a_large, &mesh_b_large, &pose_a, &pose_b),
        |b, (mesh_a, mesh_b, pose_a, pose_b)| {
            b.iter(|| black_box(mesh_mesh_deepest_contact(mesh_a, pose_a, mesh_b, pose_b)));
        },
    );

    group.finish();
}

/// Benchmark simulating humanoid self-collision scenario.
/// Multiple body parts potentially colliding.
fn bench_humanoid_self_collision(c: &mut Criterion) {
    let mut group = c.benchmark_group("humanoid_self_collision");
    // Reduce sample size for this expensive benchmark
    group.sample_size(50);

    // Simulate a humanoid with multiple body parts
    // Each part is a small mesh (~48 triangles)
    // Test collision between adjacent parts
    let mut rng = rand::thread_rng();

    // Create 10 body parts (simplified humanoid segments)
    let body_parts: Vec<_> = (0..10)
        .map(|_| generate_cube_mesh(1)) // ~48 triangles each
        .collect();

    // Create poses for body parts (arranged in a rough humanoid shape)
    let poses: Vec<_> = (0..10)
        .map(|i| {
            let x = rng.gen_range(-0.5..0.5);
            let y = rng.gen_range(-0.5..0.5);
            let z = i as f64 * 0.15; // Stack vertically
            Pose::from_position(Point3::new(x, y, z))
        })
        .collect();

    // Benchmark checking all pairs (simulating broad phase returning all pairs)
    group.bench_function("all_pairs_check", |b| {
        b.iter(|| {
            let mut total_contacts = 0;
            for i in 0..body_parts.len() {
                for j in (i + 1)..body_parts.len() {
                    if let Some(_contact) = mesh_mesh_deepest_contact(
                        &body_parts[i],
                        &poses[i],
                        &body_parts[j],
                        &poses[j],
                    ) {
                        total_contacts += 1;
                    }
                }
            }
            black_box(total_contacts)
        });
    });

    // Benchmark just adjacent pairs (what would happen with proper broad phase)
    group.bench_function("adjacent_pairs_only", |b| {
        b.iter(|| {
            let mut total_contacts = 0;
            for i in 0..(body_parts.len() - 1) {
                if let Some(_contact) = mesh_mesh_deepest_contact(
                    &body_parts[i],
                    &poses[i],
                    &body_parts[i + 1],
                    &poses[i + 1],
                ) {
                    total_contacts += 1;
                }
            }
            black_box(total_contacts)
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_mesh_mesh_collision,
    bench_mesh_mesh_rotated,
    bench_mesh_mesh_separate,
    bench_10k_triangle_pairs,
    bench_humanoid_self_collision,
);
criterion_main!(benches);
