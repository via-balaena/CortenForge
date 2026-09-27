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

use sim_soft_explicit::executor::{Obstacle, ObstacleError, check_obstacle};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Pose, SdfGridLayout};
use sim_soft_explicit::fixtures::grid::bricks;
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
        fine: None,
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

/// Points within 0.3 mm of the mandrel's surface within 2 mm (4 cells) of
/// the seam where its nose meets its shank, where the true distance's
/// curvature jumps by 1/a across the seam.
fn across_the_seam(m: Mandrel) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for n in 0..2000_u32 {
        let t = f64::from(n);
        let angle = (t * 2.399_963_229_7).rem_euclid(std::f64::consts::TAU);
        let offset = ((t * 0.618_033_988_75).fract() - 0.5) * 0.0006;
        let arc = ((t * 0.414_213_562_4).fract() - 0.5) * 0.004;
        let r = m.radius + offset;
        // The profile's radius and height, `arc` along it from the seam.
        let (radial, z) = if arc <= 0.0 {
            (r, -m.radius + arc)
        } else {
            let polar = std::f64::consts::FRAC_PI_2 - arc / m.radius;
            (r * polar.sin(), -m.radius + r * polar.cos())
        };
        points.push([radial * angle.cos(), radial * angle.sin(), z]);
    }
    points
}

/// The lookup's largest distance and normal errors against the mandrel over
/// `points`.
fn errors(m: Mandrel, o: &Obstacle, points: &[[f64; 3]]) -> (f64, f64) {
    let (mut worst_value, mut worst_normal) = (0.0_f64, 0.0_f64);
    for &p in points {
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
    (worst_value, worst_normal)
}

#[test]
fn the_lookup_is_close_to_the_mandrel_at_the_tubes_cell() {
    let (m, o) = mandrel();
    let (away, away_normal) = errors(m, &o, &near_the_surface(m));
    let (seam, seam_normal) = errors(m, &o, &across_the_seam(m));
    eprintln!(
        "MARGIN mandrel at A/20: away from the seam {:.3} um, normal {away_normal:.2e} (bars 0.1 um, 1e-3); across it {:.3} um, normal {seam_normal:.2e} (bars 2 um, 3e-2)",
        1e6 * away,
        1e6 * seam
    );
    assert!(away <= 1e-7 && away_normal <= 1e-3);
    assert!(seam <= 2e-6 && seam_normal <= 3e-2);
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
            *value = o.values[single::sdf_grid_index(i, j, k, layout) as usize] as f32;
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

#[test]
fn the_lookup_reproduces_a_linear_field_everywhere_including_the_outermost_cells() {
    // Beyond a face the lookup extrapolates the grid linearly, so a plane is
    // exact up to and on every face, not only one cell in.
    let field = |p: [f64; 3]| 0.002 + 0.6 * p[0] - 0.48 * p[1] + 0.64 * p[2];
    let o = baked(field);
    let g = o.grid;
    let extent = |size: u32| f64::from(size - 1) * g.cell_size;
    let mut worst = (0.0_f64, 0.0_f64);
    for a in 0..=40_u32 {
        for b in 0..=40_u32 {
            for c in [0.0, 0.3, 0.7, 1.0] {
                let s = |t: u32| f64::from(t) / 40.0;
                let p = [
                    g.origin_x + s(a) * extent(g.size_x),
                    g.origin_y + s(b) * extent(g.size_y),
                    g.origin_z + c * extent(g.size_z),
                ];
                let sample = o.sample(p);
                let normal = [0.6, -0.48, 0.64];
                worst.0 = worst.0.max((sample.distance - field(p)).abs());
                worst.1 = worst
                    .1
                    .max(shared::vec3_length(shared::vec3_sub(sample.normal, normal)));
            }
        }
    }
    eprintln!(
        "MARGIN linear field, faces included: value {:e}, normal {:e} (bars 1e-15, 1e-12)",
        worst.0, worst.1
    );
    assert!(worst.0 <= 1e-15 && worst.1 <= 1e-12);
}

/// A curved field with no symmetry the grid shares: a sphere's distance off the grid's centre.
fn curved(p: [f64; 3]) -> f64 {
    ((p[0] + 0.004).powi(2) + (p[1] - 0.028).powi(2) + (p[2] + 0.001).powi(2)).sqrt() - 0.006
}

#[test]
fn the_brick_address_of_a_sample() {
    // A 13 × 25 × 17 lattice has 2 × 4 × 3 bricks, a different count along each axis. Sample (9, 3, 16) is in
    // brick (1, 0, 2), the map's (2 · 4 + 0) · 2 + 1 = 17th, at (1, 3, 0) inside it: offset (0 · 8 + 3) · 8 + 1 = 25.
    // Sample (12, 24, 16) is in brick (1, 3, 2), the map's (2 · 4 + 3) · 2 + 1 = 23rd.
    let fine = SdfGridLayout {
        size_x: 13,
        size_y: 25,
        size_z: 17,
        ..baked(|_| 0.0).grid
    };
    assert_eq!(
        [
            shared::sdf_bricks(13),
            shared::sdf_bricks(25),
            shared::sdf_bricks(17),
            shared::sdf_bricks(16)
        ],
        [2, 4, 3, 2]
    );
    assert_eq!(shared::sdf_brick_index(9, 3, 16, fine), 17);
    assert_eq!(shared::sdf_brick_offset(9, 3, 16), 25);
    assert_eq!(shared::sdf_brick_offset(15, 15, 15), 511);
    assert_eq!(shared::sdf_brick_index(12, 24, 16, fine), 23);
}

#[test]
fn a_stencils_bricks_and_where_its_samples_are_stored() {
    // A stencil over columns 7–10, rows 0–3 and layers 15–18 of the same lattice touches bricks 0 and 1 along x,
    // 0 along y, 1 and 2 along z: entries (1 · 4 + 0) · 2 + 0 = 8 and 9, repeated for y, then 16 and 17.
    let fine = SdfGridLayout {
        size_x: 13,
        size_y: 25,
        size_z: 17,
        ..baked(|_| 0.0).grid
    };
    let bricks_touched =
        shared::sdf_stencil_bricks([7, 8, 9, 10], [0, 1, 2, 3], [15, 16, 17, 18], fine);
    assert_eq!(bricks_touched, [8, 9, 8, 9, 16, 17, 16, 17]);
    // Sample (9, 2, 17) is in the last brick along x and z and the first along y: the sixth entry's slot.
    let slots = [30, 31, 32, 33, 34, 35, 36, 37];
    assert_eq!(
        shared::sdf_fine_index(9, 2, 17, 7, 0, 15, slots),
        35 * 512 + shared::sdf_brick_offset(9, 2, 17)
    );
    assert_eq!(
        shared::sdf_fine_index(7, 3, 15, 7, 0, 15, slots),
        30 * 512 + shared::sdf_brick_offset(7, 3, 15)
    );
    assert!(shared::sdf_fine_present(slots));
    for missing in 0..8 {
        let mut some = slots;
        some[missing] = shared::SDF_NO_BRICK;
        assert!(!shared::sdf_fine_present(some), "{missing}");
    }
}

#[test]
fn a_fine_grid_with_every_brick_reads_as_its_lattice() {
    // 13 × 9 × 11 samples: every edge brick is part past the lattice, and padded.
    let dense = baked(curved);
    let fine = bricks(dense.grid, &dense.values, |_| true);
    let mut points = interior_points(dense.grid);
    // And on the lattice's faces and corners, where the stencil is clamped.
    let g = dense.grid;
    let near_the_far_face = |size: u32| (f64::from(size - 1) - 1e-6) * g.cell_size;
    points.push(sample_point(g, 0, 0, 0));
    points.push([
        g.origin_x + near_the_far_face(g.size_x),
        g.origin_y + near_the_far_face(g.size_y),
        g.origin_z + near_the_far_face(g.size_z),
    ]);
    points.push([
        g.origin_x + near_the_far_face(g.size_x),
        g.origin_y + 4.3 * g.cell_size,
        g.origin_z + 0.2 * g.cell_size,
    ]);
    for p in &points {
        assert_eq!(fine.sample(*p), Some(dense.sample(*p)), "{p:?}");
    }
    // Through an obstacle whose own grid reads something else.
    let coarse = baked(|p| curved(p) + 0.001);
    let obstacle = Obstacle {
        fine: Some(fine),
        ..coarse
    };
    for p in &points {
        assert_eq!(obstacle.sample(*p), dense.sample(*p), "{p:?}");
    }
}

#[test]
fn where_a_brick_is_missing_the_grid_answers() {
    let dense = baked(curved);
    let coarse = baked(|p| curved(p) + 0.001);
    // Without brick (1, 0, 0): columns 8–15, rows 0–7, layers 0–7.
    let obstacle = Obstacle {
        fine: Some(bricks(dense.grid, &dense.values, |b| b != [1, 0, 0])),
        ..coarse.clone()
    };
    let g = dense.grid;
    let (mut fine_reads, mut coarse_reads) = (0, 0);
    for p in interior_points(g) {
        let c = shared::sdf_grid_coordinate(p, g);
        let columns = shared::sdf_tricubic_axis(c[0], g.size_x);
        let rows = shared::sdf_tricubic_axis(c[1], g.size_y);
        let layers = shared::sdf_tricubic_axis(c[2], g.size_z);
        let touches = columns.iter().any(|&i| i >= 8)
            && rows.iter().any(|&j| j < 8)
            && layers.iter().any(|&k| k < 8);
        if touches {
            assert_eq!(obstacle.sample(p), coarse.sample(p), "{p:?}");
            coarse_reads += 1;
        } else {
            assert_eq!(obstacle.sample(p), dense.sample(p), "{p:?}");
            fine_reads += 1;
        }
    }
    assert!(
        fine_reads > 50 && coarse_reads > 50,
        "{fine_reads} {coarse_reads}"
    );
    // Off the fine lattice the grid answers too, even with every brick.
    let full = Obstacle {
        fine: Some(bricks(dense.grid, &dense.values, |_| true)),
        ..coarse.clone()
    };
    let outside = [
        g.origin_x - 0.5 * g.cell_size,
        g.origin_y + 0.01,
        g.origin_z + 0.01,
    ];
    assert_eq!(full.sample(outside), coarse.sample(outside));
    // By each of the lattice's six faces: a hair outside, the coarse grid; a hair inside, the fine.
    let centre = [
        g.origin_x + 6.3 * g.cell_size,
        g.origin_y + 4.2 * g.cell_size,
        g.origin_z + 5.1 * g.cell_size,
    ];
    let low = [g.origin_x, g.origin_y, g.origin_z];
    let sizes = [g.size_x, g.size_y, g.size_z];
    for axis in 0..3 {
        let far = low[axis] + f64::from(sizes[axis] - 1) * g.cell_size;
        for (face, out) in [(low[axis], -1.0), (far, 1.0)] {
            let mut beyond = centre;
            beyond[axis] = face + out * 1e-6 * g.cell_size;
            assert_eq!(
                full.sample(beyond),
                coarse.sample(beyond),
                "axis {axis} {out}"
            );
            let mut within = centre;
            within[axis] = face - out * 1e-6 * g.cell_size;
            assert_eq!(
                full.sample(within),
                dense.sample(within),
                "axis {axis} {out}"
            );
        }
    }
}

#[test]
fn a_malformed_fine_grid_is_refused() {
    let dense = baked(curved);
    let good = bricks(dense.grid, &dense.values, |b| b != [0, 1, 0]);
    let with = |fine: sim_soft_explicit::executor::FineGrid| Obstacle {
        fine: Some(fine),
        ..dense.clone()
    };
    assert_eq!(check_obstacle(&with(good.clone())), Ok(()));
    let refused = |fine, words: &str| {
        let result = check_obstacle(&with(fine));
        assert!(
            matches!(&result, Err(ObstacleError::Invalid { reason }) if reason.contains(words)),
            "{result:?}"
        );
    };
    let mut short_map = good.clone();
    short_map.map.pop();
    refused(short_map, "one entry per brick");
    let mut part_brick = good.clone();
    part_brick.values.pop();
    refused(part_brick, "whole bricks");
    let mut stray_slot = good.clone();
    let slots = u32::try_from(stray_slot.values.len() / 512).unwrap();
    stray_slot.map[0] = slots;
    refused(stray_slot, "does not have");
    let mut adrift = good.clone();
    adrift.grid.origin_y = f64::INFINITY;
    refused(adrift, "not finite");
    let mut not_finite = good.clone();
    not_finite.values[3] = f64::NAN;
    refused(not_finite, "not finite");
    let mut flat = good.clone();
    flat.grid.cell_size = 0.0;
    refused(flat, "cell size");
    let mut empty = good.clone();
    empty.grid.size_y = 0;
    refused(empty, "no samples");
    let mut endless = good.clone();
    endless.grid.size_x = u32::MAX;
    refused(endless, "too long");
    // Bricks past a u32: each side as long as a brick count allows, so they multiply past a usize too; and
    // within a usize.
    let mut vast = good.clone();
    let longest = u32::MAX - shared::SDF_BRICK;
    (vast.grid.size_x, vast.grid.size_y, vast.grid.size_z) = (longest, longest, longest);
    refused(vast, "more bricks than a u32");
    let mut wide = good.clone();
    (wide.grid.size_x, wide.grid.size_y, wide.grid.size_z) = (1 << 21, 1 << 21, 8);
    refused(wide, "more bricks than a u32");
    // Exactly u32::MAX bricks: indexed to u32::MAX - 1, so accepted, and wrong only in its map's length.
    let mut full = good;
    (full.grid.size_x, full.grid.size_y, full.grid.size_z) = (65_537 * 8, 257 * 8, 255 * 8);
    refused(full, "one entry per brick");
}
