//! Benchmarks for SIMD operations.
//!
//! Run with: cargo bench -p sim-simd

#![allow(missing_docs, clippy::wildcard_imports)]

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use nalgebra::Vector3;
use rand::Rng;

use sim_simd::*;

fn random_vector(rng: &mut impl Rng) -> Vector3<f64> {
    Vector3::new(
        rng.gen_range(-100.0..100.0),
        rng.gen_range(-100.0..100.0),
        rng.gen_range(-100.0..100.0),
    )
}

fn random_vectors(n: usize) -> Vec<Vector3<f64>> {
    let mut rng = rand::thread_rng();
    (0..n).map(|_| random_vector(&mut rng)).collect()
}

fn bench_dot_product(c: &mut Criterion) {
    let mut group = c.benchmark_group("dot_product");

    for size in [4, 16, 64, 256, 1024] {
        let vectors = random_vectors(size);
        let direction = Vector3::new(1.0, 2.0, 3.0);

        group.throughput(Throughput::Elements(size as u64));

        // Scalar implementation
        group.bench_with_input(
            BenchmarkId::new("scalar", size),
            &(&vectors, &direction),
            |b, (vecs, dir)| {
                b.iter(|| {
                    let mut max_dot = f64::NEG_INFINITY;
                    let mut max_idx = 0;
                    for (i, v) in vecs.iter().enumerate() {
                        let dot = v.dot(dir);
                        if dot > max_dot {
                            max_dot = dot;
                            max_idx = i;
                        }
                    }
                    black_box((max_idx, max_dot))
                });
            },
        );

        // SIMD batch implementation
        group.bench_with_input(
            BenchmarkId::new("simd_batch", size),
            &(&vectors, &direction),
            |b, (vecs, dir)| {
                b.iter(|| black_box(find_max_dot(vecs, dir)));
            },
        );
    }

    group.finish();
}

fn bench_vec3x4_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("vec3x4_ops");

    let vectors = [
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0),
        Vector3::new(10.0, 11.0, 12.0),
    ];
    let batch_a = Vec3x4::from_vectors(vectors);
    let batch_b = Vec3x4::from_vectors(vectors);
    let direction = Vector3::new(1.0, 2.0, 3.0);

    group.bench_function("dot", |b| {
        b.iter(|| black_box(batch_a.dot(black_box(&direction))));
    });

    group.bench_function("dot_simd", |b| {
        b.iter(|| black_box(batch_a.dot_simd(black_box(&direction))));
    });

    group.bench_function("norm_squared", |b| {
        b.iter(|| black_box(batch_a.norm_squared()));
    });

    group.bench_function("norm", |b| {
        b.iter(|| black_box(batch_a.norm()));
    });

    group.bench_function("add", |b| {
        b.iter(|| black_box(batch_a.add(black_box(&batch_b))));
    });

    group.bench_function("cross", |b| {
        b.iter(|| black_box(batch_a.cross(black_box(&direction))));
    });

    group.bench_function("normalize", |b| {
        b.iter(|| black_box(batch_a.normalize()));
    });

    group.finish();
}

fn bench_contact_force(c: &mut Criterion) {
    let mut group = c.benchmark_group("contact_force");

    let penetrations = [0.01, 0.02, 0.005, 0.015];
    let approach_velocities = [0.1, -0.05, 0.0, 0.2];
    let tangent_velocities = Vec3x4::from_vectors([
        Vector3::new(0.5, 0.1, 0.0),
        Vector3::new(-0.3, 0.2, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.5, 0.0),
    ]);

    group.bench_function("normal_force_4", |b| {
        b.iter(|| {
            black_box(batch_normal_force_4(
                black_box(&penetrations),
                black_box(&approach_velocities),
                100_000.0,
                1.5,
                1000.0,
            ))
        });
    });

    let normal_magnitudes = [1000.0, 2000.0, 500.0, 1500.0];

    group.bench_function("friction_force_4", |b| {
        b.iter(|| {
            black_box(batch_friction_force_4(
                black_box(&tangent_velocities),
                black_box(&normal_magnitudes),
                0.5,
                0.001,
            ))
        });
    });

    group.finish();
}

fn bench_aabb_overlap(c: &mut Criterion) {
    let mut group = c.benchmark_group("aabb_overlap");

    let a_mins = Vec3x4::from_vectors([
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(2.0, 2.0, 2.0),
        Vector3::new(3.0, 3.0, 3.0),
    ]);
    let a_maxs = Vec3x4::from_vectors([
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(2.0, 2.0, 2.0),
        Vector3::new(3.0, 3.0, 3.0),
        Vector3::new(4.0, 4.0, 4.0),
    ]);
    let b_mins = Vec3x4::from_vectors([
        Vector3::new(0.5, 0.5, 0.5),
        Vector3::new(1.5, 1.5, 1.5),
        Vector3::new(5.0, 5.0, 5.0),
        Vector3::new(2.5, 2.5, 2.5),
    ]);
    let b_maxs = Vec3x4::from_vectors([
        Vector3::new(1.5, 1.5, 1.5),
        Vector3::new(2.5, 2.5, 2.5),
        Vector3::new(6.0, 6.0, 6.0),
        Vector3::new(3.5, 3.5, 3.5),
    ]);

    group.bench_function("batch_4", |b| {
        b.iter(|| {
            black_box(batch_aabb_overlap_4(
                black_box(&a_mins),
                black_box(&a_maxs),
                black_box(&b_mins),
                black_box(&b_maxs),
            ))
        });
    });

    group.finish();
}

fn bench_integration(c: &mut Criterion) {
    let mut group = c.benchmark_group("integration");

    let mut positions = Vec3x4::from_vectors([
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ]);
    let velocities = Vec3x4::splat(Vector3::new(1.0, 2.0, 3.0));
    let accelerations = Vec3x4::splat(Vector3::new(0.0, 0.0, -9.81));

    group.bench_function("position_4", |b| {
        b.iter(|| {
            batch_integrate_position_4(black_box(&mut positions), black_box(&velocities), 0.01);
        });
    });

    let mut vels = velocities;
    group.bench_function("velocity_4", |b| {
        b.iter(|| {
            batch_integrate_velocity_4(black_box(&mut vels), black_box(&accelerations), 0.01);
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_dot_product,
    bench_vec3x4_operations,
    bench_contact_force,
    bench_aabb_overlap,
    bench_integration,
);
criterion_main!(benches);
