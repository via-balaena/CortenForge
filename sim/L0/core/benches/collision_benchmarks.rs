//! Benchmarks for collision detection operations.
//!
//! Run with: cargo bench -p sim-core
//!
//! Performance targets (from `MESH_MESH_COLLISION_PLAN.md`):
//! - <5ms for 10k triangle pair test

#![allow(
    missing_docs,
    clippy::wildcard_imports,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::ignored_unit_patterns,
    clippy::unnecessary_cast
)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use rand::Rng;

use sim_core::Aabb;
use sim_core::Pose;
use sim_core::mesh::{
    TriangleMeshData, closest_point_on_triangle, mesh_mesh_contact, mesh_mesh_deepest_contact,
    triangle_box_contact, triangle_capsule_contact, triangle_sphere_contact,
};
use sim_core::mid_phase::{Bvh, BvhPrimitive};

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
            BenchmarkId::new("overlapping", format!("{tri_count}x{tri_count}_tri")),
            &(&mesh_a, &mesh_b, &pose_a, &pose_b),
            |b, (mesh_a, mesh_b, pose_a, pose_b)| {
                b.iter(|| black_box(mesh_mesh_contact(mesh_a, pose_a, mesh_b, pose_b)));
            },
        );

        // Also test deepest contact (what's used in dispatch)
        group.bench_with_input(
            BenchmarkId::new("deepest", format!("{tri_count}x{tri_count}_tri")),
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
            BenchmarkId::new("rotation", format!("{angle_deg}_deg")),
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
            BenchmarkId::new("no_overlap", format!("{tri_count}x{tri_count}_tri")),
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
        BenchmarkId::new("mesh_pair", format!("{pairs}_pairs")),
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
        BenchmarkId::new("mesh_pair", format!("{pairs_large}_pairs")),
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

// =============================================================================
// Triangle-Primitive Collision Benchmarks
// =============================================================================

/// Benchmark triangle-sphere collision detection.
fn bench_triangle_sphere(c: &mut Criterion) {
    let mut group = c.benchmark_group("triangle_sphere");

    // Create a triangle in the XY plane
    let v0 = Point3::new(-1.0, -1.0, 0.0);
    let v1 = Point3::new(1.0, -1.0, 0.0);
    let v2 = Point3::new(0.0, 1.0, 0.0);

    // Test with different sphere positions (hitting different triangle features)
    let test_cases = [
        ("face_hit", Point3::new(0.0, 0.0, 0.3), 0.5),
        ("edge_hit", Point3::new(0.0, -1.0, 0.3), 0.5),
        ("vertex_hit", Point3::new(-1.0, -1.0, 0.3), 0.5),
        ("miss", Point3::new(0.0, 0.0, 2.0), 0.5),
    ];

    for (name, center, radius) in test_cases {
        group.bench_with_input(BenchmarkId::new("contact", name), &(), |b, _| {
            b.iter(|| black_box(triangle_sphere_contact(v0, v1, v2, center, radius)));
        });
    }

    // Benchmark with many triangles (batch scenario)
    group.bench_function("batch_100_triangles", |b| {
        let triangles: Vec<_> = (0..100)
            .map(|i| {
                let offset = i as f64 * 0.1;
                (
                    Point3::new(-1.0 + offset, -1.0, 0.0),
                    Point3::new(1.0 + offset, -1.0, 0.0),
                    Point3::new(0.0 + offset, 1.0, 0.0),
                )
            })
            .collect();
        let sphere_center = Point3::new(5.0, 0.0, 0.3);
        let sphere_radius = 0.5;

        b.iter(|| {
            let mut count = 0;
            for (v0, v1, v2) in &triangles {
                if triangle_sphere_contact(*v0, *v1, *v2, sphere_center, sphere_radius).is_some() {
                    count += 1;
                }
            }
            black_box(count)
        });
    });

    group.finish();
}

/// Benchmark triangle-capsule collision detection.
fn bench_triangle_capsule(c: &mut Criterion) {
    let mut group = c.benchmark_group("triangle_capsule");

    let v0 = Point3::new(-1.0, -1.0, 0.0);
    let v1 = Point3::new(1.0, -1.0, 0.0);
    let v2 = Point3::new(0.0, 1.0, 0.0);

    let test_cases = [
        (
            "vertical_hit",
            Point3::new(0.0, 0.0, 0.2),
            Point3::new(0.0, 0.0, 1.0),
            0.3,
        ),
        (
            "horizontal_hit",
            Point3::new(-0.5, 0.0, 0.2),
            Point3::new(0.5, 0.0, 0.2),
            0.3,
        ),
        (
            "miss",
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.0, 0.0, 3.0),
            0.3,
        ),
    ];

    for (name, start, end, radius) in test_cases {
        group.bench_with_input(BenchmarkId::new("contact", name), &(), |b, _| {
            b.iter(|| black_box(triangle_capsule_contact(v0, v1, v2, start, end, radius)));
        });
    }

    group.finish();
}

/// Benchmark triangle-box collision detection.
fn bench_triangle_box(c: &mut Criterion) {
    let mut group = c.benchmark_group("triangle_box");

    let v0 = Point3::new(-1.0, -1.0, 0.0);
    let v1 = Point3::new(1.0, -1.0, 0.0);
    let v2 = Point3::new(0.0, 1.0, 0.0);

    let half_extents = Vector3::new(0.3, 0.3, 0.3);

    let test_cases = [
        ("axis_aligned_hit", Point3::new(0.0, 0.0, 0.2), 0.0),
        ("rotated_45_hit", Point3::new(0.0, 0.0, 0.2), 45.0),
        ("miss", Point3::new(0.0, 0.0, 2.0), 0.0),
    ];

    for (name, center, angle_deg) in test_cases {
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, (angle_deg as f64).to_radians());
        group.bench_with_input(BenchmarkId::new("contact", name), &(), |b, _| {
            b.iter(|| {
                black_box(triangle_box_contact(
                    v0,
                    v1,
                    v2,
                    center,
                    &rotation,
                    &half_extents,
                ))
            });
        });
    }

    group.finish();
}

/// Benchmark closest point on triangle computation.
fn bench_closest_point_on_triangle(c: &mut Criterion) {
    let mut group = c.benchmark_group("closest_point_triangle");

    let v0 = Point3::new(0.0, 0.0, 0.0);
    let v1 = Point3::new(1.0, 0.0, 0.0);
    let v2 = Point3::new(0.5, 1.0, 0.0);

    // Test different regions
    let test_cases = [
        ("inside_face", Point3::new(0.5, 0.3, 1.0)),
        ("vertex_v0", Point3::new(-1.0, -1.0, 0.0)),
        ("vertex_v1", Point3::new(2.0, -1.0, 0.0)),
        ("vertex_v2", Point3::new(0.5, 2.0, 0.0)),
        ("edge_v0v1", Point3::new(0.5, -1.0, 0.0)),
        ("edge_v1v2", Point3::new(1.0, 0.5, 0.0)),
        ("edge_v2v0", Point3::new(-0.5, 0.5, 0.0)),
    ];

    for (name, point) in test_cases {
        group.bench_with_input(BenchmarkId::new("region", name), &(), |b, _| {
            b.iter(|| black_box(closest_point_on_triangle(v0, v1, v2, point)));
        });
    }

    group.finish();
}

// =============================================================================
// BVH Benchmarks
// =============================================================================

/// Generate BVH primitives for testing.
fn generate_bvh_primitives(count: usize) -> Vec<BvhPrimitive> {
    let mut rng = rand::thread_rng();
    (0..count)
        .map(|idx| {
            let base = Point3::new(
                rng.gen_range(-10.0..10.0),
                rng.gen_range(-10.0..10.0),
                rng.gen_range(-10.0..10.0),
            );
            let v0 = base;
            let v1 = base + Vector3::new(rng.gen_range(0.1..0.5), 0.0, 0.0);
            let v2 = base + Vector3::new(0.0, rng.gen_range(0.1..0.5), 0.0);
            BvhPrimitive::from_triangle(v0, v1, v2, idx)
        })
        .collect()
}

/// Benchmark BVH construction.
fn bench_bvh_construction(c: &mut Criterion) {
    let mut group = c.benchmark_group("bvh_construction");

    for count in [100, 500, 1000, 5000] {
        let primitives = generate_bvh_primitives(count);

        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(
            BenchmarkId::new("build", format!("{count}_primitives")),
            &primitives,
            |b, primitives| {
                b.iter(|| black_box(Bvh::build(primitives.clone())));
            },
        );
    }

    group.finish();
}

/// Benchmark BVH queries.
fn bench_bvh_query(c: &mut Criterion) {
    let mut group = c.benchmark_group("bvh_query");

    // Build a BVH with 1000 primitives
    let primitives = generate_bvh_primitives(1000);
    let bvh = Bvh::build(primitives);

    // Test different query sizes
    let query_sizes = [
        ("small", Vector3::new(0.5, 0.5, 0.5)),
        ("medium", Vector3::new(2.0, 2.0, 2.0)),
        ("large", Vector3::new(5.0, 5.0, 5.0)),
    ];

    for (name, half_extents) in query_sizes {
        let query_aabb = Aabb::from_center(Point3::new(0.0, 0.0, 0.0), half_extents);

        group.bench_with_input(BenchmarkId::new("aabb", name), &query_aabb, |b, aabb| {
            b.iter(|| black_box(bvh.query(aabb)));
        });
    }

    // Benchmark many queries in sequence
    group.bench_function("batch_100_queries", |b| {
        let mut rng = rand::thread_rng();
        let queries: Vec<_> = (0..100)
            .map(|_| {
                Aabb::from_center(
                    Point3::new(
                        rng.gen_range(-5.0..5.0),
                        rng.gen_range(-5.0..5.0),
                        rng.gen_range(-5.0..5.0),
                    ),
                    Vector3::new(1.0, 1.0, 1.0),
                )
            })
            .collect();

        b.iter(|| {
            let mut total = 0;
            for query in &queries {
                total += bvh.query(query).len();
            }
            black_box(total)
        });
    });

    group.finish();
}

/// Benchmark BVH construction from triangle mesh (end-to-end).
fn bench_bvh_from_mesh(c: &mut Criterion) {
    let mut group = c.benchmark_group("bvh_from_mesh");

    for subdivisions in [1, 2, 3, 4] {
        // Generate mesh and then manually build BVH to benchmark construction
        let mesh = generate_cube_mesh(subdivisions);
        let tri_count = mesh.triangle_count();

        group.throughput(Throughput::Elements(tri_count as u64));
        group.bench_with_input(
            BenchmarkId::new("mesh", format!("{tri_count}_triangles")),
            &subdivisions,
            |b, &subdivisions| {
                b.iter(|| {
                    // This includes mesh creation + BVH build
                    black_box(generate_cube_mesh(subdivisions))
                });
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    // Mesh-mesh collision
    bench_mesh_mesh_collision,
    bench_mesh_mesh_rotated,
    bench_mesh_mesh_separate,
    bench_10k_triangle_pairs,
    bench_humanoid_self_collision,
    // Triangle-primitive collision
    bench_triangle_sphere,
    bench_triangle_capsule,
    bench_triangle_box,
    bench_closest_point_on_triangle,
    // BVH benchmarks
    bench_bvh_construction,
    bench_bvh_query,
    bench_bvh_from_mesh,
);
criterion_main!(benches);
