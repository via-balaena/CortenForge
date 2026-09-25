//! The shared SDF lookup against the CPU path it single-sources:
//! `cf_geometry::SdfGrid::distance_clamped` and `gradient_clamped`, in f64.
//!
//! Plan §15g step 1: the largest difference over the test points must be at
//! most 1e-9 × the largest value.
//!
//! One known difference is kept out of that bar and counted: `cf-geometry`
//! clamps a point onto the grid in world units and then converts to grid
//! units, and on some grids the conversion rounds a far-face point past the
//! last sample. Where that happens to the point itself, its distance is the
//! largest value and its normal +z; where it happens only to one of the
//! gradient's probes, its normal is skewed. The shared lookup clamps in grid
//! units and reads the face. The margins print with `--nocapture`.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::float_cmp
)]

use cf_geometry::SdfGrid;
use nalgebra::Point3;
use sim_soft_explicit::f64 as shared;

/// A grid that is not a cube and not at the origin, so a swapped axis or a
/// dropped origin shows.
fn grid(f: impl Fn(Point3<f64>) -> f64) -> SdfGrid {
    SdfGrid::from_fn(13, 9, 11, 0.00173, Point3::new(-0.0137, 0.0211, -0.009), f)
}

/// A grid on which `cf-geometry`'s world-unit clamp rounds past the far
/// faces.
fn rounding_grid() -> SdfGrid {
    SdfGrid::from_fn(4, 4, 4, 0.1, Point3::new(0.1, 0.1, 0.1), |p| {
        p.coords
            .metric_distance(&Point3::new(0.27, 0.23, 0.26).coords)
            - 0.12
    })
}

/// Where `cf-geometry` falls back at `p`: whether the point itself, or any of
/// the six probes behind its gradient, lands outside after its own clamp
/// (`design/cf-geometry/src/sdf.rs`, `distance_clamped` and
/// `gradient_clamped`). Returns `(point, probe)`.
fn cpu_falls_back(grid: &SdfGrid, p: [f64; 3]) -> (bool, bool) {
    let o = grid.origin();
    let hi = [
        o.x + grid.extent_x(),
        o.y + grid.extent_y(),
        o.z + grid.extent_z(),
    ];
    let clamp = |q: [f64; 3]| {
        Point3::new(
            q[0].clamp(o.x, hi[0]),
            q[1].clamp(o.y, hi[1]),
            q[2].clamp(o.z, hi[2]),
        )
    };
    let center = clamp(p);
    let eps = grid.cell_size() * 0.5;
    let mut probes = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut q = [center.x, center.y, center.z];
            q[axis] += sign * eps;
            probes.push(clamp(q));
        }
    }
    (
        grid.distance(center).is_none(),
        probes.iter().any(|&q| grid.distance(q).is_none()),
    )
}

/// Whether `p` is within half a cell of a far face, or past one.
fn near_a_far_face(grid: &SdfGrid, p: [f64; 3]) -> bool {
    let o = grid.origin();
    let h = grid.cell_size();
    let sizes = [grid.width(), grid.height(), grid.depth()];
    (0..3).any(|axis| (p[axis] - [o.x, o.y, o.z][axis]) / h >= sizes[axis] as f64 - 1.5)
}

/// An obstacle-like field: a sphere, stretched and rippled so the gradient
/// varies across every cell.
fn field(p: Point3<f64>) -> f64 {
    let c = Point3::new(-0.002, 0.028, 0.0);
    let d = ((p.x - c.x) * 1.3)
        .hypot((p.y - c.y) * 0.8)
        .hypot(p.z - c.z)
        - 0.006;
    d + 0.0004 * (p.x * 900.0).sin() * (p.z * 700.0).cos()
}

fn layout(grid: &SdfGrid) -> shared::SdfGridLayout {
    let origin = grid.origin();
    shared::SdfGridLayout {
        origin_x: origin.x,
        origin_y: origin.y,
        origin_z: origin.z,
        cell_size: grid.cell_size(),
        size_x: grid.width() as u32,
        size_y: grid.height() as u32,
        size_z: grid.depth() as u32,
    }
}

/// The executor's side of a lookup: fetch the values the shared math names.
fn sample(grid: &SdfGrid, point: [f64; 3]) -> shared::SdfSample {
    let layout = layout(grid);
    let mut values = [0.0; 7];
    for probe in 0..shared::SDF_PROBE_COUNT {
        let coordinate = shared::sdf_probe_coordinate(point, layout, probe);
        let corners =
            shared::sdf_cell_corners(coordinate, layout).map(|i| grid.values()[i as usize]);
        values[probe as usize] = shared::sdf_trilinear(coordinate, corners);
    }
    shared::sdf_combine(values, layout)
}

/// Deterministic points: inside, on sample nodes and cell faces, on and past
/// every face, edge and corner, and far outside.
fn test_points(grid: &SdfGrid) -> Vec<[f64; 3]> {
    let o = grid.origin();
    let h = grid.cell_size();
    let hi = [
        o.x + grid.extent_x(),
        o.y + grid.extent_y(),
        o.z + grid.extent_z(),
    ];
    let mut points = Vec::new();
    // Pseudo-random, inside and up to two cells outside.
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1_u64 << 53) as f64
    };
    for _ in 0..4000 {
        let lerp = |lo: f64, hi: f64, t: f64| (hi - lo).mul_add(t * 1.2 - 0.1, lo);
        points.push([
            lerp(o.x, hi[0], next()),
            lerp(o.y, hi[1], next()),
            lerp(o.z, hi[2], next()),
        ]);
    }
    // Sample nodes, cell-face midpoints and the grid's own corners.
    for i in 0..grid.width() {
        for j in [0, 3, grid.height() - 1] {
            for k in [0, 5, grid.depth() - 1] {
                let node = [
                    (i as f64).mul_add(h, o.x),
                    (j as f64).mul_add(h, o.y),
                    (k as f64).mul_add(h, o.z),
                ];
                points.push(node);
                points.push([node[0] + 0.5 * h, node[1], node[2]]);
                points.push([node[0], node[1] + 0.5 * h, node[2] + 0.5 * h]);
            }
        }
    }
    // Every combination of below, on, inside and past each face.
    let axis = |lo: f64, hi: f64| {
        [
            lo - 3.0 * h,
            lo,
            0.5f64.mul_add(hi - lo, lo),
            hi,
            hi + 0.3 * h,
            hi + 40.0 * h,
        ]
    };
    for x in axis(o.x, hi[0]) {
        for y in axis(o.y, hi[1]) {
            for z in axis(o.z, hi[2]) {
                points.push([x, y, z]);
            }
        }
    }
    points
}

#[test]
fn the_shared_lookup_matches_the_cpu_path() {
    for (name, grid) in [
        ("offset grid", grid(field)),
        ("rounding grid", rounding_grid()),
    ] {
        let largest = grid.values().iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let (mut worst_distance, mut worst_normal) = (0.0_f64, 0.0_f64);
        let mut worst_point = [0.0; 3];
        let points = test_points(&grid);
        let (mut point_fallbacks, mut probe_fallbacks) = (0, 0);
        for &p in &points {
            let (point_falls_back, probe_falls_back) = cpu_falls_back(&grid, p);
            if point_falls_back || probe_falls_back {
                assert!(
                    near_a_far_face(&grid, p),
                    "{name}: fallback away from a far face at {p:?}"
                );
            }
            if point_falls_back {
                point_fallbacks += 1;
                continue;
            }
            let point = Point3::new(p[0], p[1], p[2]);
            let got = sample(&grid, p);
            let distance_error = (got.distance - grid.distance_clamped(point)).abs();
            let normal_error = if probe_falls_back {
                probe_fallbacks += 1;
                0.0
            } else {
                let expected_normal = grid.gradient_clamped(point);
                (0..3)
                    .map(|i| (got.normal[i] - expected_normal[i]).abs())
                    .fold(0.0, f64::max)
            };
            assert!(
                distance_error.is_finite() && normal_error.is_finite(),
                "non-finite at {p:?}"
            );
            if distance_error > worst_distance {
                worst_distance = distance_error;
                worst_point = p;
            }
            worst_normal = worst_normal.max(normal_error);
        }
        eprintln!(
            "MARGIN {name}: distance {worst_distance:e} against a bar of {:e}; normal {worst_normal:e}; \
             {} points; cf-geometry falls back at {point_fallbacks} for the point, \
             {probe_fallbacks} more for a gradient probe only",
            1e-9 * largest,
            points.len()
        );
        assert!(
            worst_distance <= 1e-9 * largest,
            "{name}: distance differs by {worst_distance:e} (largest value {largest:e}) at {worst_point:?}"
        );
        // A normal is unit length, so its largest value is 1.
        assert!(
            worst_normal <= 1e-9,
            "{name}: normal differs by {worst_normal:e}"
        );
        assert!(points.len() > 4000);
        if name == "rounding grid" {
            assert!(
                point_fallbacks > 0 && probe_fallbacks > 0,
                "the rounding grid no longer rounds; pick another"
            );
        } else {
            assert_eq!((point_fallbacks, probe_fallbacks), (0, 0), "{name} rounds");
        }
    }
}

#[test]
fn a_flat_field_falls_back_to_plus_z_on_both_paths() {
    let grid = grid(|_| 0.25);
    for p in [[0.0, 0.03, 0.0], [1.0, -1.0, 1.0]] {
        let expected = grid.gradient_clamped(Point3::new(p[0], p[1], p[2]));
        let got = sample(&grid, p);
        assert_eq!(got.normal, [expected.x, expected.y, expected.z]);
        assert_eq!(got.normal, [0.0, 0.0, 1.0]);
        assert!((got.distance - 0.25).abs() < 1e-15);
    }
}

/// The lookup at `f32`, what the GPU runs, against `f64` on the same grid.
#[test]
fn the_f32_lookup_agrees_with_f64() {
    use sim_soft_explicit::f32 as single;
    let grid = grid(field);
    let largest = grid.values().iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    let values32: Vec<f32> = grid.values().iter().map(|&v| v as f32).collect();
    let l = layout(&grid);
    let l32 = single::SdfGridLayout {
        origin_x: l.origin_x as f32,
        origin_y: l.origin_y as f32,
        origin_z: l.origin_z as f32,
        cell_size: l.cell_size as f32,
        size_x: l.size_x,
        size_y: l.size_y,
        size_z: l.size_z,
    };
    let (mut worst, mut worst_normal) = (0.0_f64, 0.0_f64);
    for p in test_points(&grid) {
        let reference = sample(&grid, p);
        let p32 = p.map(|c| c as f32);
        let mut values = [0.0_f32; 7];
        for probe in 0..single::SDF_PROBE_COUNT {
            let c = single::sdf_probe_coordinate(p32, l32, probe);
            let corners = single::sdf_cell_corners(c, l32).map(|i| values32[i as usize]);
            values[probe as usize] = single::sdf_trilinear(c, corners);
        }
        let got = single::sdf_combine(values, l32);
        worst = worst.max((f64::from(got.distance) - reference.distance).abs());
        worst_normal = (0..3).fold(worst_normal, |m, i| {
            m.max((f64::from(got.normal[i]) - reference.normal[i]).abs())
        });
    }
    eprintln!("MARGIN f32: distance {worst:e} of largest {largest:e}; normal {worst_normal:e}");
    // 16 f32 epsilons of the largest value.
    assert!(
        worst <= 16.0 * f64::from(f32::EPSILON) * largest,
        "f32 distance differs by {worst:e} of {largest:e}"
    );
    // Measured 1.4e-6.
    assert!(
        worst_normal <= 1e-5,
        "f32 normal differs by {worst_normal:e}"
    );
}
