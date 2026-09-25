//! The shared SDF lookup: a tricubic (Catmull–Rom) interpolant of the baked
//! grid and its exact gradient (plan §16o). It replaced a trilinear lookup
//! that matched `cf-geometry`'s, whose error on a curved surface fed energy
//! into the kinematic contact in a long hold.
//!
//! The margins print with `--nocapture`.

// Coordinates and grid indices read as x, y, z and i, j, k; the f32 check
// narrows f64 test values on purpose.
#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::float_cmp,
    clippy::many_single_char_names
)]

use sim_soft_explicit::executor::Obstacle;
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Pose, SdfGridLayout};
use sim_soft_explicit::fixtures::tube::Mandrel;

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A grid that is not a cube and not at the origin, so a swapped axis or a
/// dropped origin shows, with `f` baked at its samples.
fn baked(f: impl Fn([f64; 3]) -> f64) -> Obstacle {
    let grid = SdfGridLayout {
        origin_x: -0.0137,
        origin_y: 0.0211,
        origin_z: -0.009,
        cell_size: 0.00173,
        size_x: 13,
        size_y: 9,
        size_z: 11,
    };
    let mut values = Vec::new();
    for k in 0..grid.size_z {
        for j in 0..grid.size_y {
            for i in 0..grid.size_x {
                values.push(f(sample_point(grid, i, j, k)));
            }
        }
    }
    obstacle(grid, values)
}

fn obstacle(grid: SdfGridLayout, values: Vec<f64>) -> Obstacle {
    Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 1.0,
        poses: vec![IDENTITY],
        friction: 0.0,
    }
}

fn sample_point(grid: SdfGridLayout, i: u32, j: u32, k: u32) -> [f64; 3] {
    [
        grid.origin_x + f64::from(i) * grid.cell_size,
        grid.origin_y + f64::from(j) * grid.cell_size,
        grid.origin_z + f64::from(k) * grid.cell_size,
    ]
}

/// Points spread through the cells at least one cell from every face, where
/// the 4 × 4 × 4 stencil needs no clamping.
fn interior_points(grid: SdfGridLayout) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for n in 0..500_u32 {
        let t = f64::from(n);
        let fraction = |a: f64| (a * 0.618_033_988_75).fract();
        let span = |size: u32| f64::from(size - 3);
        points.push([
            grid.origin_x + (1.0 + fraction(t + 0.1) * span(grid.size_x)) * grid.cell_size,
            grid.origin_y + (1.0 + fraction(1.7 * t + 0.3) * span(grid.size_y)) * grid.cell_size,
            grid.origin_z + (1.0 + fraction(2.3 * t + 0.7) * span(grid.size_z)) * grid.cell_size,
        ]);
    }
    points
}

#[test]
fn the_lookup_passes_through_the_grid_values() {
    // A sample's position converts back to its grid coordinate only to
    // rounding, so it may land in the next cell at a fraction of 1 − ε.
    let o = baked(|p| (p[0] * 31.0).sin() + p[1] * p[2] * 400.0);
    let mut worst = 0.0_f64;
    for k in 0..o.grid.size_z {
        for j in 0..o.grid.size_y {
            for i in 0..o.grid.size_x {
                let value = o.values[((k * o.grid.size_y + j) * o.grid.size_x + i) as usize];
                worst = worst.max((o.sample(sample_point(o.grid, i, j, k)).distance - value).abs());
            }
        }
    }
    eprintln!("MARGIN at the samples: {worst:e} (bar 1e-14)");
    assert!(worst <= 1e-14);
}

#[test]
fn the_lookup_reproduces_a_field_quadratic_in_each_axis_away_from_the_faces() {
    // Catmull–Rom's slopes are central differences, exact for a quadratic,
    // so its cubic reproduces one; the tensor product reproduces every field
    // of degree at most two in each axis.
    let field = |p: [f64; 3]| {
        let [x, y, z] = p;
        0.3 + 2.0 * x - y + 0.5 * z + 40.0 * x * x - 30.0 * y * z + 20.0 * z * z + 500.0 * x * x * y
    };
    let gradient = |p: [f64; 3]| {
        let [x, y, z] = p;
        [
            2.0 + 80.0 * x + 1000.0 * x * y,
            -1.0 - 30.0 * z + 500.0 * x * x,
            0.5 - 30.0 * y + 40.0 * z,
        ]
    };
    let o = baked(field);
    let (mut worst_value, mut worst_normal) = (0.0_f64, 0.0_f64);
    for p in interior_points(o.grid) {
        let s = o.sample(p);
        worst_value = worst_value.max((s.distance - field(p)).abs());
        let g = gradient(p);
        let n = shared::vec3_scale(g, 1.0 / shared::vec3_length(g));
        worst_normal = worst_normal.max(shared::vec3_length(shared::vec3_sub(s.normal, n)));
    }
    eprintln!(
        "MARGIN quadratic field: value {worst_value:e}, normal {worst_normal:e} (bars 1e-13, 1e-12)"
    );
    assert!(worst_value <= 1e-13 && worst_normal <= 1e-12);
}

/// The 11 mm mandrel baked at A/20, as the tube runs it.
fn mandrel() -> (Mandrel, Obstacle) {
    let m = Mandrel { radius: 0.011 };
    let (grid, values) = m
        .baked([-0.025, -0.025, -0.105], [0.025, 0.025, 0.005], 0.0005)
        .unwrap();
    (m, obstacle(grid, values))
}

/// Points within 0.3 mm of the mandrel's surface, on its cylinder and its
/// nose.
fn near_the_surface(m: Mandrel) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for n in 0..2000_u32 {
        let t = f64::from(n);
        let angle = (t * 2.399_963_229_7).rem_euclid(std::f64::consts::TAU);
        let offset = ((t * 0.618_033_988_75).fract() - 0.5) * 0.0006;
        let r = m.radius + offset;
        if n % 2 == 0 {
            let z = -0.012 - (t * 0.414_213_562_4).fract() * 0.08;
            points.push([r * angle.cos(), r * angle.sin(), z]);
        } else {
            // On the nose: a point at polar angle `polar` from the tip.
            let polar = (t * 0.414_213_562_4).fract() * 1.4;
            points.push([
                r * polar.sin() * angle.cos(),
                r * polar.sin() * angle.sin(),
                -m.radius + r * polar.cos(),
            ]);
        }
    }
    points
}

#[test]
fn the_lookup_is_close_to_the_mandrel_at_the_tubes_cell() {
    let (m, o) = mandrel();
    let (mut worst_value, mut worst_normal) = (0.0_f64, 0.0_f64);
    for p in near_the_surface(m) {
        let s = o.sample(p);
        worst_value = worst_value.max((s.distance - m.distance(p)).abs());
        let h = 1e-7;
        let d = |q: [f64; 3]| m.distance(q);
        let g = [
            (d([p[0] + h, p[1], p[2]]) - d([p[0] - h, p[1], p[2]])) / (2.0 * h),
            (d([p[0], p[1] + h, p[2]]) - d([p[0], p[1] - h, p[2]])) / (2.0 * h),
            (d([p[0], p[1], p[2] + h]) - d([p[0], p[1], p[2] - h])) / (2.0 * h),
        ];
        let n = shared::vec3_scale(g, 1.0 / shared::vec3_length(g));
        worst_normal = worst_normal.max(shared::vec3_length(shared::vec3_sub(s.normal, n)));
    }
    eprintln!(
        "MARGIN mandrel at A/20: distance {:.3} um, normal {worst_normal:.2e} (bars 0.2 um, 1e-3)",
        1e6 * worst_value
    );
    assert!(worst_value <= 2e-7 && worst_normal <= 1e-3);
}

#[test]
fn a_point_outside_the_grid_reads_the_nearest_face() {
    let o = baked(|p| p[0] * 3.0 - p[1] + p[2] * p[2] * 50.0);
    let g = o.grid;
    let inside = [
        g.origin_x + 4.3 * g.cell_size,
        g.origin_y + f64::from(g.size_y - 1) * g.cell_size,
        g.origin_z + 2.6 * g.cell_size,
    ];
    let outside = [inside[0], inside[1] + 0.05, inside[2]];
    let (a, b) = (o.sample(outside), o.sample(inside));
    let normal = shared::vec3_length(shared::vec3_sub(a.normal, b.normal));
    eprintln!(
        "MARGIN the face: distance {:e}, normal {normal:e} (bars 1e-14, 1e-12)",
        (a.distance - b.distance).abs()
    );
    assert!((a.distance - b.distance).abs() <= 1e-14 && normal <= 1e-12);
}

#[test]
fn a_flat_field_falls_back_to_plus_z() {
    let o = baked(|_| 0.004);
    let s = o.sample([0.0, 0.03, -0.003]);
    assert_eq!(s.distance, 0.004);
    assert_eq!(s.normal, [0.0, 0.0, 1.0]);
}

#[test]
fn the_f32_lookup_agrees_with_f64() {
    use sim_soft_explicit::f32 as single;
    let (m, o) = mandrel();
    let g = o.grid;
    let layout = single::SdfGridLayout {
        origin_x: g.origin_x as f32,
        origin_y: g.origin_y as f32,
        origin_z: g.origin_z as f32,
        cell_size: g.cell_size as f32,
        size_x: g.size_x,
        size_y: g.size_y,
        size_z: g.size_z,
    };
    let (mut worst_value, mut worst_normal) = (0.0_f64, 0.0_f64);
    for p in near_the_surface(m) {
        let q = p.map(|c| c as f32);
        let c = single::sdf_grid_coordinate(q, layout);
        let axes = [
            single::sdf_tricubic_axis(c[0], g.size_x),
            single::sdf_tricubic_axis(c[1], g.size_y),
            single::sdf_tricubic_axis(c[2], g.size_z),
        ];
        let mut values = [0.0_f32; 64];
        for (index, value) in values.iter_mut().enumerate() {
            let (i, j, k) = (
                axes[0][index % 4],
                axes[1][index / 4 % 4],
                axes[2][index / 16],
            );
            *value = o.values[((k * g.size_y + j) * g.size_x + i) as usize] as f32;
        }
        let narrow = single::sdf_tricubic(c, values, layout);
        let wide = o.sample(p);
        worst_value = worst_value.max((f64::from(narrow.distance) - wide.distance).abs());
        let n = narrow.normal.map(f64::from);
        worst_normal = worst_normal.max(shared::vec3_length(shared::vec3_sub(n, wide.normal)));
    }
    eprintln!(
        "MARGIN f32 against f64 on the mandrel: distance {:.3} um, normal {worst_normal:.2e} (bars 0.1 um, 1e-3)",
        1e6 * worst_value
    );
    assert!(worst_value <= 1e-7 && worst_normal <= 1e-3);
}
